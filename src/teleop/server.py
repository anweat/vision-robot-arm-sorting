#!/usr/bin/env python3
"""Local arm console. Default mode is simulation; never imports Arm_Lib in simulation."""
import argparse
import json
import secrets
import threading
import time
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from core import Controller, ROOT, safe_arm, RevisionConflict
from camera import Camera
from calibration import fit, project

WEB = Path(__file__).parent / 'web'


class App:
    def __init__(self, controller, camera, phone=None):
        self.control, self.camera = controller, camera
        self.phone=phone
        self.locator=None
        if controller.hardware:
            from localization import Locator
            self.locator=Locator(controller,camera,phone,controller.directory)
            threading.Thread(target=self.locator.run,daemon=True).start()
        self.token = secrets.token_urlsafe(32)
        self.episode = None  # dict(dir, label, steps) while recording
        controller.on_step = self.record_step
        self.lock = threading.Lock()
        self.calibration = None
        self.calibrations = []
        self.frozen = None
        self.samples = []
        self.measurements = []
        self.closed = threading.Event()
        (controller.directory/"session.json").write_text(json.dumps(dict(
            started_at=time.time(), mode=controller.mode, arm_config=controller.cfg,
            scene=self.scene(), camera=self.camera.info()), ensure_ascii=False, indent=2))

    def scene(self):
        cfg = self.control.cfg
        return dict(hand_eye=json.loads((ROOT/'config/calibration/hand_eye.json').read_text()),
                    intrinsics=json.loads((ROOT/'config/calibration/camera_intrinsics.json').read_text()),
                    keep_out=safe_arm.workspace.load_keep_out(cfg),
                    map=json.loads((ROOT/'config/map_a4_v2.json').read_text()))

    def record_step(self, kind, goal, observed):
        """Called by the controller after each verified step; saves frames taken after the step."""
        ep = self.episode
        if not ep:
            return
        done = time.time()
        deadline = done + .8
        jpeg, meta = self.camera.snapshot()
        while (meta.get('stamp') or 0) <= done and time.time() < deadline:
            time.sleep(.05)
            jpeg, meta = self.camera.snapshot()
        index = ep['steps'] = ep['steps'] + 1
        item = dict(index=index, t=done, kind=kind, goal=goal, observed=observed, profile=self.control.profile,
                    fk=safe_arm.workspace.kin.forward(observed), grip=self.control.grip,
                    board=dict(meta, after_step=(meta.get('stamp') or 0) > done))
        if jpeg:
            item['board_image'] = f'step_{index:03d}_board.jpg'
            (ep['dir']/item['board_image']).write_bytes(jpeg)
        if self.phone:
            photo, pmeta = self.phone.snapshot()
            item['phone'] = pmeta
            if photo:
                item['phone_image'] = f'step_{index:03d}_phone.jpg'
                (ep['dir']/item['phone_image']).write_bytes(photo)
        with (ep['dir']/'steps.jsonl').open('a') as f:
            f.write(json.dumps(item, ensure_ascii=False) + '\n')

    def record(self, on, label, note):
        c = self.control
        with self.lock:
            if on:
                if self.episode:
                    raise ValueError('已在录制；先结束当前试验')
                name = time.strftime('episode_%Y%m%d_%H%M%S') + ('_' + ''.join(ch for ch in label if ch.isalnum() or ch in '-_')[:40] if label else '')
                folder = c.directory/name
                folder.mkdir()
                self.episode = dict(dir=folder, label=label, steps=0, started=time.time())
                (folder/'episode.json').write_text(json.dumps(dict(label=label, note=note, started=time.time(),
                                                                   start_state=c.status()), ensure_ascii=False, indent=2))
                c.log('episode_start', episode=name, label=label)
            else:
                if not self.episode:
                    raise ValueError('当前没有在录制')
                ep, self.episode = self.episode, None
                meta = json.loads((ep['dir']/'episode.json').read_text())
                meta.update(ended=time.time(), steps=ep['steps'], end_state=c.status(), end_note=note)
                (ep['dir']/'episode.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))
                c.log('episode_end', episode=ep['dir'].name, steps=ep['steps'])

    def status(self):
        ep = self.episode
        recording = dict(name=ep['dir'].name, label=ep['label'], steps=ep['steps']) if ep else None
        return dict(**self.control.status(), camera=self.camera.info(), samples=len(self.samples), recording=recording,
                    phone=self.phone.info() if self.phone else None,
                    localization=self.locator.snapshot() if self.locator else None)

    def poll(self):
        while not self.closed.wait(.3):
            self.control.refresh()

    def action(self, path, body):
        c = self.control
        owner = body.get('owner')
        if not isinstance(owner, str) or not 1 <= len(owner) <= 80:
            raise ValueError('操作页面标识无效')
        if path == '/api/enable':
            c.enable(owner, body.get('onsite'))
        elif path == '/api/heartbeat':
            c.heartbeat(owner)
        elif path == '/api/stop':
            c.stop()
        elif path == '/api/jog':
            c.jog(owner, body.get('joint'), body.get('delta'), body.get('revision'))
        elif path == '/api/profile':
            name = body.get('name')
            if name is None and type(body.get('boundary')) is not bool:
                raise ValueError('测试档参数必须是布尔值')
            c.set_profile(owner,bool(body.get('boundary')),body.get('onsite'),name)
        elif path == '/api/cartesian':
            c.cartesian(owner, body.get('axis'), body.get('amount'), body.get('revision'))
        elif path == '/api/gripper':
            c.gripper(owner, body.get('action'), body.get('revision'))
        elif path == '/api/record':
            if type(body.get('on')) is not bool:
                raise ValueError('录制参数必须是布尔值')
            self.record(body['on'], str(body.get('label', ''))[:60], str(body.get('note', ''))[:2000])
        elif path == '/api/pose/plan':
            return c.plan_pose(body.get('target'))
        elif path == '/api/pose/execute':
            c.move_to_pose(owner, body.get('plan_id'), body.get('revision'))
        elif path == '/api/home/plan':
            return c.plan_home()
        elif path == '/api/home/execute':
            c.home(owner,body.get('plan_id'),body.get('revision'))
        elif path == '/api/localization/save':
            if not self.locator:raise ValueError('定位仅在实机接入后运行')
            return self.locator.save()
        elif path == '/api/verify':
            kinds={'grasped':'人工确认夹住物块','boundary_clear':'接近禁区但未接触',
                   'platform_contact':'人工确认平台接触','table_contact':'人工确认夹爪碰地','released':'人工确认已松开'}
            kind=body.get('kind')
            if kind not in kinds:raise ValueError('未知校验类型')
            if kind in ('platform_contact','table_contact'):c.stop()
            if c.busy:raise ValueError('已停止后续指令；等待本步结束再记录接触')
            result=self.action('/api/sample',dict(owner=owner,kind=kind,label=kinds[kind],note=body.get('note','')))
            return result
        elif path == '/api/freeze':
            with c.gate:
                if c.busy or time.time()-c.observed_at > 2:
                    raise ValueError('请等待动作停稳和回读刷新')
                jpeg, meta = self.camera.snapshot()
                if jpeg is None:
                    raise ValueError('没有画面；可用 --replay 加载已有 JPEG')
                if meta['kind'] == 'live' and (meta['age_s'] is None or not 0 <= meta['age_s'] < 2):
                    raise ValueError('画面已过期')
                stamp = time.time_ns()
                name = f'frame_{stamp}.jpg'
                (c.directory/name).write_bytes(jpeg)
                with self.lock:
                    self.frozen = dict(id=str(stamp), file=name, camera=meta, state=c.status())
                    self.calibration = None
                return dict(frozen=self.frozen)
        elif path == '/api/calibrate':
            with self.lock:
                if not self.frozen or body.get('frame_id') != self.frozen['id']:
                    raise ValueError('请先冻结画面，并在同一画面标定')
                result = fit(body.get('points'))
                result.update(id=str(time.time_ns()), frame=self.frozen, plane=str(body.get('plane',''))[:80], created_at=time.time())
                self.calibration = result
                self.calibrations.append(result)
                (c.directory / f"calibration_{result['id']}.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
                return result
        elif path == '/api/measure':
            with self.lock:
                if not self.calibration or body.get('frame_id') != self.frozen['id']:
                    raise ValueError('请先标定当前冻结帧')
                pixels = body.get('pixels')
                if not isinstance(pixels,list) or not 1 <= len(pixels) <= 2 or any(not isinstance(p,list) or len(p)!=2 for p in pixels):
                    raise ValueError('需要一个或两个像素点')
                xy = [project(self.calibration['h'],p) for p in pixels]
                import math
                result = dict(xy_cm=xy, distance_cm=math.dist(*xy) if len(xy)==2 else None,
                              pixels=pixels, t=time.time(), calibration_id=self.calibration['id'], plane=self.calibration['plane'], frame_id=self.frozen['id'])
                self.measurements.append(result)
                with (c.directory/'measurements.jsonl').open('a') as f:
                    f.write(json.dumps(result,ensure_ascii=False)+'\n')
                return result
        elif path == '/api/sample':
            with c.gate:
                if c.busy or time.time()-c.observed_at > 2:
                    raise ValueError('等待动作停稳和回读刷新后再采样')
                state = c.status()
                jpeg, meta = self.camera.snapshot()
                stamp = time.time_ns()
                name = f'sample_{stamp}.jpg' if jpeg else None
                if jpeg:
                    (c.directory/name).write_bytes(jpeg)
                item = dict(id=str(stamp), t=time.time(), kind=str(body.get('kind','boundary'))[:40],
                            label=str(body.get('label',''))[:100], note=str(body.get('note',''))[:2000],
                            state=state, image=name, camera=meta,
                            evidence='simulation' if not c.hardware else 'manual observation; geometry estimated')
                if self.phone:
                    photo,meta=self.phone.snapshot()
                    item['phone']=meta
                    if photo:
                        item['phone_image']=f'sample_{stamp}_phone.jpg'
                        (c.directory/item['phone_image']).write_bytes(photo)
                item['localization']=self.locator.snapshot() if self.locator else None
                # The enriched record is the authoritative standalone evidence file.
                (c.directory/f'sample_{stamp}.json').write_text(json.dumps(item,ensure_ascii=False,indent=2))
                with self.lock:
                    self.samples.append(item)
                    with (c.directory/'samples.jsonl').open('a') as f:
                        f.write(json.dumps(item,ensure_ascii=False)+'\n')
                c.log('sample', sample_id=item['id'])
                return item
        else:
            raise ValueError('未知操作')
        return self.status()


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, body, ctype='application/json', status=200):
            if not isinstance(body, bytes):
                body = json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',ctype)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def local(self):
            host = self.headers.get('Host','')
            if host not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'):
                raise ValueError('仅允许本机或 SSH 本地转发访问')
            origin = self.headers.get('Origin')
            if origin and origin != 'http://'+host:
                raise ValueError('拒绝跨站请求')

        def do_GET(self):
            try:
                self.local()
                path = urlsplit(self.path).path
                static = {'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','text/javascript'),
                          '/scene.js':('scene.js','text/javascript'),'/style.css':('style.css','text/css')}
                if path in static:
                    name, mime = static[path]
                    self.send((WEB/name).read_bytes(),mime)
                elif path == '/api/bootstrap':
                    self.send(dict(token=app.token, scene=app.scene(), status=app.status()))
                elif path == '/api/status':
                    self.send(app.status())
                elif path == '/api/export':
                    self.send(dict(schema_version=1,session=app.control.directory.name,
                                   config_sha256=app.control.config_hash,mode=app.control.mode,samples=app.samples,measurements=app.measurements,
                                   calibrations=app.calibrations,calibration=app.calibration))
                elif path in ('/api/annotated/board','/api/annotated/phone'):
                    name=path.rsplit('/',1)[-1]
                    with app.locator.lock if app.locator else threading.Lock():
                        jpeg=app.locator.images.get(name) if app.locator else None
                    self.send(jpeg,'image/jpeg') if jpeg else self.send({'error':'no localization frame'},status=503)
                elif path == '/api/phone':
                    jpeg,_=app.phone.snapshot() if app.phone else (None,None)
                    self.send(jpeg,'image/jpeg') if jpeg else self.send({'error':'no phone'},status=503)
                elif path == '/api/frame':
                    jpeg,_ = app.camera.snapshot()
                    self.send(jpeg,'image/jpeg') if jpeg else self.send({'error':'no frame'},status=503)
                elif path == '/api/frozen':
                    if app.frozen:
                        self.send((app.control.directory/app.frozen['file']).read_bytes(),'image/jpeg')
                    else:
                        self.send({'error':'no frozen frame'},status=404)
                else:
                    self.send({'error':'not found'},status=404)
            except (ValueError, OSError) as e:
                self.send({'error':str(e)},status=400)

        def do_POST(self):
            try:
                self.local()
                if not secrets.compare_digest(self.headers.get('X-Console-Token',''),app.token):
                    raise ValueError('页面凭证过期，请刷新')
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 32768:
                    raise ValueError('请求大小无效')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body,dict):
                    raise ValueError('请求必须是 JSON 对象')
                self.send(app.action(urlsplit(self.path).path,body))
            except (ValueError, TypeError, KeyError, safe_arm.MotionError) as e:
                # revision 冲突单独标 reason，网关据此重读 revision 重试；其余错误不带 reason，
                # 网关会原样把消息传给页面，不再被改写成"revision 反复过期"
                payload = {'error': str(e)}
                if isinstance(e, RevisionConflict):
                    payload['reason'] = 'revision_conflict'
                self.send(payload, status=409)
            except Exception:
                import traceback
                traceback.print_exc()   # the lock-out below is silent otherwise: say what raised and on which request
                print(f'internal error on POST {self.path}; control locked', flush=True)
                app.control.stop()
                self.send({'error':'服务内部错误，控制已锁定；检查本地运行环境'},status=500)
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=8765)
    p.add_argument('--hardware',action='store_true',help='board only; requires explicit --execute')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--phone-url',help='optional second camera HTTP snapshot')
    source = p.add_mutually_exclusive_group()
    source.add_argument('--camera-url',help='HTTP snapshot or RTSP URL; prefer env ARM_CAMERA_URL for credentials')
    source.add_argument('--replay',type=Path,help='offline JPEG; never enables hardware')
    p.add_argument('--output',type=Path)
    p.add_argument('--default-profile',choices=('explore','normal'),default='explore',
                   help='explore (default): course limits, platform keep-out shown but not enforced, small steps; '
                        'normal: the standard soft limits and keep-out used by the automatic tasks')
    args = p.parse_args()
    if args.hardware != args.execute:
        p.error('实机必须同时指定 --hardware --execute')
    import os
    camera = Camera(args.camera_url or (None if args.replay else os.environ.get('ARM_CAMERA_URL')),args.replay)
    phone=Camera(args.phone_url) if args.phone_url else None
    folder = args.output or ROOT/'data/teleop'/(time.strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(2))
    with ExitStack() as stack:
        stack.callback(camera.close)
        if phone:stack.callback(phone.close)
        arm = None
        if args.hardware:
            stack.enter_context(safe_arm.exclusive_lock())
            arm = safe_arm.Arm(safe_arm.load_config(safe_arm.DEFAULT_CONFIG))
        control = Controller(folder,arm=arm,hardware=args.hardware,camera=camera,default_profile=args.default_profile)
        app = App(control,camera,phone)
        if app.locator:stack.callback(app.locator.closed.set)
        stack.callback(app.closed.set)
        stack.callback(control.stop)
        threading.Thread(target=app.poll,daemon=True).start()
        server = ThreadingHTTPServer(('127.0.0.1',args.port),handler(app))
        stack.callback(server.server_close)
        print(f'Arm console [{control.mode}]: http://127.0.0.1:{args.port}/',flush=True)
        print(f'Session: {folder}',flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    main()
