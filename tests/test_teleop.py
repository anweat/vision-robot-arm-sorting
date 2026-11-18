import json
import math
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/teleop'))
from core import Controller, SimArm, safe_arm
from calibration import fit, project
from camera import Camera
from server import App, handler, ThreadingHTTPServer


class RecordingArm(SimArm):
    def __init__(self):
        super().__init__()
        self.commands = []
        self.fail = False

    def write6(self, pose, duration_ms):
        self.commands.append(list(pose))
        if not self.fail:
            super().write6(pose, duration_ms)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.arm = RecordingArm()
        self.c = Controller(self.tmp.name, self.arm)

    def move(self, joint=1, delta=1, revision=None):
        with patch('core.time.sleep'):
            return self.c.jog('a',joint,delta,self.c.revision if revision is None else revision)

    def test_first_command_takes_control_and_others_wait(self):
        self.move()                                   # no enable step (doc 32 §1.2)
        self.assertEqual(self.c.status()['owner'],'a')
        with patch('core.time.sleep'):
            with self.assertRaises(ValueError):
                self.c.jog('b',1,1)                   # 'a' holds a live lease
        self.c.deadline=time.monotonic()-1            # the lease lapses without heartbeats
        with patch('core.time.sleep'):
            self.c.jog('b',1,1)
        self.assertEqual(self.c.status()['owner'],'b')
        self.c.take_over('a')
        self.assertEqual(self.c.status()['owner'],'a')
        self.assertEqual(len(self.arm.commands),2)

    def test_old_enable_still_takes_control_and_ignores_onsite(self):
        self.c.enable('a',False)
        with self.assertRaises(ValueError):
            self.c.enable('b',True)
        self.assertEqual(self.arm.commands,[])

    def test_small_step_and_revision_is_not_required(self):
        self.move()
        self.assertEqual(self.arm.commands,[[91,105,5,5,95,30]])
        self.move(revision=0)                         # a stale revision is no longer refused
        self.assertEqual(self.arm.commands[-1],[92,105,5,5,95,30])

    def test_limits_and_bad_arguments(self):
        for delta in (0,31,1.5):
            with self.assertRaises(ValueError):
                self.move(delta=delta)
        with self.assertRaises(safe_arm.MotionError):
            self.move(joint=3,delta=-16)              # J3 may go to -10, never further
        self.assertEqual(self.arm.commands,[])

    def test_keep_out_only_reported_unless_enforced(self):
        self.move(joint=2,delta=-20)                  # into the platform keep-out: reported, not refused
        self.assertLess(self.c.status()['clearance_cm'],0)
        sent=len(self.arm.commands)
        self.arm.pose=[90,105,5,5,95,30];self.c.refresh();self.c.command=None
        self.c.cfg['keep_out']=dict(self.c.base_cfg['keep_out'])   # what runtime.enforce_* would give
        with self.assertRaises(safe_arm.MotionError):
            self.move(joint=2,delta=-20)
        self.assertEqual(len(self.arm.commands),sent)  # nothing more was sent

    def test_read_failure_never_writes(self):
        self.arm.pose[2]=None
        with self.assertRaises(ValueError):
            self.move()
        self.assertEqual(self.arm.commands,[])
        self.assertFalse(self.c.status()['armed'])

    def test_deviation_holds_at_readback_without_retreat(self):
        self.c.cfg['motion']['readback_tolerance_deg']=.5
        self.arm.fail=True
        with self.assertRaises(ValueError):
            self.move(delta=2)
        # the step that met resistance, then one command to hold exactly where the arm is (not the start)
        self.assertEqual(self.arm.commands,[[92,105,5,5,95,30],[90,105,5,5,95,30]])
        self.assertEqual(self.arm.commands[1],self.arm.read())
        self.assertFalse(self.c.status()['armed'])

    def test_stop_ends_control_and_heartbeat_does_not_revive_a_lapsed_lease(self):
        self.move();self.c.stop()
        self.assertFalse(self.c.status()['armed'])
        self.c.deadline=time.monotonic()-1
        self.c.heartbeat('a')
        self.assertFalse(self.c.status()['armed'])

    def test_stop_during_read_prevents_dispatch(self):
        original=self.arm.read
        def read_and_stop():
            self.c.stop()
            return original()
        self.arm.read=read_and_stop
        with self.assertRaises(ValueError):
            self.move()
        self.assertEqual(self.arm.commands,[])

    def test_concurrent_commands_not_queued(self):
        entered, release = threading.Event(), threading.Event()
        original=self.arm.read
        def slow_read():
            entered.set();release.wait(2);return original()
        self.arm.read=slow_read
        errors=[]
        def work():
            try:self.move()
            except Exception as e:errors.append(e)
        t=threading.Thread(target=work);t.start();entered.wait(1)
        try:
            with self.assertRaises(ValueError):self.move()
        finally:
            release.set();t.join(3)
        self.assertFalse(errors)
        self.assertEqual(len(self.arm.commands),1)

    def test_hardware_needs_a_live_picture_but_no_onsite_flag(self):
        cam=Camera();self.c.hardware=True;self.c.camera=cam
        with self.assertRaises(ValueError):self.move()
        cam.publish(b'\xff\xd8\xff\xd9',time.time(),'capture')
        self.move()
        cam.stamp-=10
        with self.assertRaises(ValueError):self.move()
        self.assertEqual(len(self.arm.commands),1)

    def test_speed_sets_segment_size_and_timing(self):
        self.assertEqual(self.c.status()['speed']['value'],20)
        with self.assertRaises(ValueError):self.c.set_speed(41)
        with self.assertRaises(ValueError):self.c.set_speed(True)
        self.c.set_speed(6)
        m=self.c.cfg['motion']
        self.assertEqual((m['max_step_deg'],m['segment_ms']),(3,500))
        self.move(delta=10)
        j1=[90]+[q[0] for q in self.arm.commands]
        self.assertEqual((len(j1)-1,j1[-1]),(4,100))
        self.assertTrue(all(0<b-a<=3 for a,b in zip(j1,j1[1:])),j1)
        self.c.set_speed(37)
        self.assertEqual((round(self.c.cfg['motion']['max_step_deg'],1),self.c.cfg['motion']['segment_ms'],
                          self.c.cfg['motion']['settle_ms']),(14.8,400,120))   # the old explore timing

    def test_goto_named_and_dry_run(self):
        self.arm.pose=[92,110,8,8,120,133];self.c.refresh()
        plan=self.c.plan_goto(named='observe_v1')
        self.assertEqual(plan['target'],[90,105,5,5,120,133])
        self.assertEqual(self.arm.commands,[])
        with patch('core.time.sleep'):
            self.c.goto('a',named='observe_v1')
        self.assertEqual(self.arm.read(),[90,105,5,5,120,133])
        n=len(self.arm.commands)
        with patch('core.time.sleep'):
            self.c.goto('a',named='observe_v1')          # already there: nothing sent, no error
        self.assertEqual(len(self.arm.commands),n)
        with self.assertRaises(ValueError):self.c.plan_goto(named='nowhere')
        with self.assertRaises(ValueError):self.c.plan_goto([90,90,90])

    def test_goto_keeps_the_gripper_command_not_its_readback(self):
        # closed on a block: command 133, readback 129; a move that does not name J6 must keep sending 133
        with patch('core.time.sleep'):
            self.c.gripper('a','grip')
        self.arm.pose[5]=129;self.c.refresh()
        with patch('core.time.sleep'):
            self.c.goto('a',[90,110,5,5,95,None])
        self.assertTrue(all(q[5]==133 for q in self.arm.commands[1:]),self.arm.commands)

    def test_drive_moves_while_updates_come_and_stops_without_them(self):
        with patch('core.time.sleep'):
            self.c.drive('a','joint',[1,0,0,0])
            deadline=time.time()+2
            while len(self.arm.commands)<3 and time.time()<deadline:
                self.c.drive('a','joint',[1,0,0,0]);time.sleep(.01)
            self.c.drive('a','joint',[0,0,0,0])          # stick released
            deadline=time.time()+2
            while self.c.status()['drive']['active'] and time.time()<deadline:time.sleep(.01)
        self.assertFalse(self.c.status()['drive']['active'])
        j1=[q[0] for q in self.arm.commands]
        self.assertGreaterEqual(len(j1),3)
        self.assertEqual(j1,sorted(j1))
        self.assertTrue(all(q[1:]==[105,5,5,95,30] for q in self.arm.commands))
        with self.assertRaises(ValueError):self.c.drive('a','joint',[2,0,0,0])
        with self.assertRaises(ValueError):self.c.drive('a','fly',[0,0,0,0])

    def test_drive_dead_man_and_stop(self):
        with patch('core.time.sleep'):
            self.c.drive('a','cartesian',[0,0,1,0])
            deadline=time.time()+2
            while self.c.status()['drive']['active'] and time.time()<deadline:time.sleep(.01)
        self.assertFalse(self.c.status()['drive']['active'])   # no further updates: it stopped by itself
        sent=len(self.arm.commands)
        self.c.drive('a','joint',[1,0,0,0]);self.c.stop()
        time.sleep(.3)
        self.assertLessEqual(len(self.arm.commands),sent+1)
        self.assertFalse(self.c.status()['armed'])


class CalibrationTest(unittest.TestCase):
    def test_independent_point_and_distance(self):
        h=fit([[0,0,0,0],[640,0,32,0],[640,480,32,24],[0,480,0,24]])['h']
        self.assertAlmostEqual(math.dist(project(h,[100,100]),project(h,[160,180])),5)
        self.assertAlmostEqual(project(h,[320,240])[0],16)

    def test_degenerate_and_nonfinite_rejected(self):
        for points in ([],[[i,i,i,i] for i in range(4)],[[0,0,float('nan'),0]]*4):
            with self.assertRaises(ValueError):fit(points)

    def test_perspective_mapping(self):
        expected=[.8,.1,.02,-.2,.7,.05,.2,-.1]
        pixels=[[30,40],[600,40],[590,450],[20,440],[320,240]]
        actual=fit([p+project(expected,p) for p in pixels])['h']
        self.assertLess(math.dist(project(actual,[200,300]),project(expected,[200,300])),1e-7)


class HTTPTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        if not (ROOT/'data/captures/2026-09-24/map_v2_placed.jpg').exists():
            self.skipTest('needs the recorded frame under data/captures (not in the repository)')
        self.camera=Camera(replay=ROOT/'data/captures/2026-09-24/map_v2_placed.jpg')
        self.control=Controller(self.tmp.name,camera=self.camera)
        self.app=App(self.control,self.camera)
        self.http=ThreadingHTTPServer(('127.0.0.1',0),handler(self.app))
        self.thread=threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.http.server_close);self.addCleanup(self.http.shutdown)
        self.url=f'http://127.0.0.1:{self.http.server_port}'

    def post(self,path,body,token=True,origin=None):
        headers={'Content-Type':'application/json'}
        if token:headers['X-Console-Token']=self.app.token
        if origin:headers['Origin']=origin
        req=urllib.request.Request(self.url+path,json.dumps({'owner':'test',**body}).encode(),headers)
        with urllib.request.urlopen(req) as r:return json.load(r)

    def test_cross_site_and_missing_token_rejected(self):
        for kwargs in ({'token':False},{'origin':'http://evil.example'}):
            with self.assertRaises(urllib.error.HTTPError) as e:self.post('/api/enable',{},**kwargs)
            self.assertEqual(e.exception.code,409)
        self.assertFalse(self.control.status()['armed'])

    def test_freeze_calibration_measure_sample_export(self):
        frozen=self.post('/api/freeze',{})['frozen']
        points=[[0,0,0,0],[640,0,32,0],[640,480,32,24],[0,480,0,24]]
        self.post('/api/calibrate',{'frame_id':frozen['id'],'points':points,'plane':'test'})
        result=self.post('/api/measure',{'frame_id':frozen['id'],'pixels':[[100,100],[160,180]]})
        self.assertAlmostEqual(result['distance_cm'],5)
        result=self.post('/api/sample',{'label':'offline test'})
        self.assertEqual(result['evidence'],'simulation')
        self.assertTrue((Path(self.tmp.name)/result['image']).is_file())
        with urllib.request.urlopen(self.url+'/api/export') as r:export=json.load(r)
        self.assertEqual(len(export['measurements']),1)
        self.assertEqual(len(export['samples']),1)
        self.post('/api/freeze',{})
        with self.assertRaises(urllib.error.HTTPError):
            self.post('/api/measure',{'frame_id':frozen['id'],'pixels':[[10,10]]})

    def test_simulation_does_not_import_hardware_library(self):
        self.assertNotIn('Arm_Lib',sys.modules)
        self.assertEqual(self.app.status()['mode'],'simulation')


class BrowserGeometryTest(unittest.TestCase):
    def test_frontend_pose_and_camera_match_python_kinematics(self):
        import shutil
        import subprocess
        if not shutil.which('node'):
            self.skipTest('Node is needed for the independent frontend geometry check')
        hand=json.loads((ROOT/'config/calibration/hand_eye.json').read_text())['cam2gripper']
        poses=[[90,105,5,5,95,30],[45,130,20,30,160,90],[90,90,90,90,95,30],[130,100,10,15,120,80]]
        script="import {geometry} from './src/teleop/web/scene.js'; const input="+json.dumps([poses,hand])+"; console.log(JSON.stringify(input[0].map(q=>geometry(q,input[1]))));"
        out=json.loads(subprocess.check_output(['node','--input-type=module','-e',script],cwd=ROOT,text=True))
        for q,result in zip(poses,out):
            expected=safe_arm.workspace.kin.forward(q)['tip']
            self.assertLess(math.dist(result['tip'],expected),.0001)
            frame=safe_arm.workspace.kin.camera_mount_pose(q)  # the camera rides on the wrist, not with J5
            cam=[[sum(frame[i][k]*hand[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
            self.assertLess(math.dist(result['cam'],[row[3] for row in cam[:3]]),.0001)
            for j in range(3):
                self.assertLess(math.dist(result['camAxes'][j],[cam[i][j] for i in range(3)]),1e-10)

if __name__ == '__main__':unittest.main()
