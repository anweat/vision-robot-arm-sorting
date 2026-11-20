"""The simulation module (dev/sim): scene, grasp rule, rendered camera, detector check and its HTTP service.
Needs numpy and OpenCV (and the block faces under data/); skipped where they are missing."""
import json
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ('teleop', 'spatial', 'vision', 'sim'):
    sys.path.insert(0, str(ROOT / 'dev' / sub))
    sys.path.insert(0, str(ROOT / 'app' / 'src' / sub))
try:
    import cv2
    import numpy as np
    import world as W
    READY = (W.FACES_DIR / 'index.json').exists()
except ImportError:
    READY = False
if READY:
    import dofbot_kinematics as kin
    import evaluate as E
    import render as R
    from map_registration import TableMap, detect_markers, load_intrinsics, solve_camera_pose

OBSERVE = [90, 105, 5, 5, 95, 30]


def reach(xyz, grip=30):
    """Servo angles putting the tip at a base-frame point (m)."""
    sol = kin.inverse_within(*xyz, {j: (0, 180) for j in (1, 2, 3, 4)})
    return [round(v) for v in sol['servos']] + [95, grip]


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class WorldTest(unittest.TestCase):
    def setUp(self):
        self.world = W.World(0)
        self.arm = W.WorldArm(self.world)

    def test_a_placed_block_rests_on_the_calibrated_table_with_the_asked_face_up(self):
        bid = self.world.place('hazardous', [2, 10], 20, 'syringe')
        b = self.world.snapshot()['blocks'][0]
        self.assertEqual((b['id'], b['up'], b['map_cm'], b['yaw_deg']), (bid, 'syringe', [2.0, 10.0], 20.0))
        self.assertAlmostEqual(b['base_cm'][2], self.world.table_z * 100 + 1.5, places=2)
        self.assertAlmostEqual(self.world.table_z, -0.0142, places=4)   # the touch calibration, not a constant here

    def test_every_face_can_be_turned_up(self):
        for face, item in enumerate(self.world.items['kitchen']):
            bid = self.world.place('kitchen', [0, 10], 0, face)
            self.assertEqual(self.world.snapshot()['blocks'][-1]['up'], item)
            self.world.remove(bid)

    def test_closing_over_a_block_holds_it_and_reads_back_as_holding(self):
        bid = self.world.place('other', [0, 10])
        c = self.world.blocks[bid]['pose'][:3, 3].copy()
        down = reach((c[0], c[1], self.world.table_z + 0.012))
        self.arm.write6(down, 500)
        self.arm.write6(down[:5] + [133], 500)
        self.assertEqual(self.world.held, bid)
        self.assertEqual(self.arm.read()[5], W.HOLDING_READBACK)
        up = reach((c[0] + 0.05, c[1], 0.08), 133)
        self.arm.write6(up, 500)
        moved = self.world.blocks[bid]['pose'][:3, 3]
        self.assertGreater(moved[2], 0.05)
        self.assertAlmostEqual(moved[0] - c[0], 0.05, delta=0.012)
        self.arm.write6(up[:5] + [30], 500)
        self.assertIsNone(self.world.held)
        b = self.world.snapshot()['blocks'][0]
        self.assertAlmostEqual(b['base_cm'][2], self.world.table_z * 100 + 1.5, places=2)   # fell flat onto the table

    def test_the_four_bar_pads_square_and_centre_the_block_they_take(self):
        bid = self.world.place('other', [0, 10], 12)          # 12 deg off the jaw, 8 mm along it
        c = self.world.blocks[bid]['pose'][:3, 3].copy()
        down = reach((c[0] + 0.008, c[1], self.world.table_z + 0.012))
        self.arm.write6(down, 500)
        self.arm.write6(down[:5] + [133], 500)
        self.assertEqual(self.world.held, bid)
        jaw = W.BODY.link5(self.world.actual[:5])[:3, 0]
        pose = self.world.blocks[bid]['pose']
        self.assertGreater(max(abs(pose[:3, i] @ jaw) for i in range(3)), 0.999)   # a face square to the jaw
        pad = np.array(W.GR.pad_centre(self.world.actual[:5], W.GR.contact(W.BLOCK)))
        jaw_h = jaw[:2] / np.linalg.norm(jaw[:2])
        self.assertAlmostEqual(float((pose[:2, 3] - pad[:2]) @ jaw_h), 0.0, places=4)   # pushed to the middle

    def test_pads_above_the_block_or_beside_it_take_nothing_but_a_turned_block_is_squared(self):
        bid = self.world.place('other', [0, 10], 38)             # the real arm held blocks 36-39 deg off (doc 34 §3.2)
        c = self.world.blocks[bid]['pose'][:3, 3].copy()
        down = reach((c[0], c[1], self.world.table_z + 0.012))
        self.arm.write6(down, 500)
        self.arm.write6(down[:5] + [133], 500)
        self.assertEqual(self.world.held, bid)
        self.arm.write6(down[:5] + [30], 500)
        self.world.remove(bid)
        bid = self.world.place('other', [0, 10])
        aside = reach((c[0], c[1] + 0.015, self.world.table_z + 0.012))   # 1.5 cm across the jaw
        self.arm.write6(aside, 500)
        self.arm.write6(aside[:5] + [133], 500)
        self.assertIsNone(self.world.held)
        self.arm.write6(aside[:5] + [30], 500)
        high = reach((c[0], c[1], self.world.table_z + 0.03))    # pads would close above the top face
        self.arm.write6(high, 500)
        self.arm.write6(high[:5] + [133], 500)
        self.assertIsNone(self.world.held)

    def test_fingers_coming_down_on_another_block_are_recorded_apart_from_bin_contacts(self):
        bid = self.world.place('other', [0, 10])
        c = self.world.blocks[bid]['pose'][:3, 3]
        self.arm.write6(reach((c[0], c[1], self.world.table_z + 0.08), 170), 500)   # close high above it,
        self.arm.write6(reach((c[0], c[1], self.world.table_z + 0.02), 170), 500)   # then come down onto its top
        snap = self.world.snapshot()
        self.assertGreater(snap['block_contacts']['count'], 0)
        self.assertEqual(snap['block_contacts']['recent'][-1]['block'], bid)
        self.assertEqual(snap['contacts']['count'], 0)

    def test_the_body_model_is_the_planners_and_follows_the_gripper_opening(self):
        wide = self.world.body()
        self.assertEqual(set(wide['parts']), {'link1', 'link2', 'link3', 'link4',
                                              'gripper_fingers', 'gripper_links', 'gripper_static'})
        self.assertAlmostEqual(min(p[2] for p in wide['parts']['link1']), 0.064, delta=0.002)   # on the turntable
        self.arm.write6(OBSERVE[:5] + [133], 500)
        closed = self.world.body()
        self.assertLess(closed['gap_mm'], wide['gap_mm'])
        self.assertAlmostEqual(closed['gap_mm'], W.GG.gap_for_delta(W.GR.delta_for_j6(133)) * 1000, delta=0.1)

    def test_the_mesh_frames_move_the_fingers_as_the_planners_four_bar(self):
        q, delta = [107, 60, 26, 30, 92], W.GR.delta_for_j6(60)
        frames = W.World.mesh_frames(q, delta)
        samples = W.GG.load()['points_m']
        for name in ('finger_right', 'finger_left'):
            local = np.array(samples[name], float)
            T = frames['gripper_' + name]
            mine = local @ T[:3, :3].T + T[:3, 3]
            ref = np.array(W.GR.points(q, delta, parts=('fingers',)))
            self.assertLess(np.min(np.linalg.norm(ref[:, None] - mine[None], axis=2), axis=0).max(), 1e-9)
        self.assertTrue(np.allclose(frames['link5'], W.BODY.link5(q)))
        self.assertEqual(len([k for k in frames if k.startswith('gripper_')]), 6)

    def test_course_meshes_are_welded_and_thinned_without_degenerate_faces(self):
        import web_meshes as M
        cube = np.array([[[0, 0, 0], [1, 0, 0], [1, 1, 0]], [[0, 0, 0], [1, 1, 0], [0, 1, 0]],
                         [[0, 0, 1], [1, 1, 1], [1, 0, 1]], [[0, 0, 1], [0, 1, 1], [1, 1, 1]]], float) * 0.01
        v, f = M.weld(cube)
        self.assertEqual((len(v), len(f)), (8, 4))
        self.assertEqual(len(set(M.shells(f, len(v)))), 2)          # the two squares are separate shells
        tv, tf = M.thin(v, f, 0.02)                                   # a cell bigger than the part: nothing left
        self.assertEqual(len(tf), 0)
        tv, tf = M.thin(v, f, 0.001)
        self.assertEqual(len(tf), 4)

    def test_the_platform_lid_is_touched_where_the_real_arm_touched_it_and_only_there(self):
        # doc 30 §10.2/§11.1 and doc 28a §6: two recorded platform contacts, three near views that ran clear
        touched = ([103, 72, 19, 3, 92, 170], [90, 102, -5, -1, 92, 170])
        clear = ([49, 105, 5, 0, 92, 30], [124, 105, 5, 0, 92, 30], [51, 99, 5, 5, 92, 30], [107, 60, 26, 30, 92, 30])
        for pose, expected in [(p, True) for p in touched] + [(p, False) for p in clear]:
            self.arm.write6(OBSERVE, 0)
            before = self.world.platform_contact_count
            self.arm.write6(pose, 0)
            self.assertEqual(self.world.platform_contact_count > before, expected, pose)
        self.assertEqual(self.world.snapshot()['platform_contacts']['count'], 2)

    def test_closing_on_nothing_reads_back_as_empty(self):
        self.world.place('other', [0, 10])
        self.arm.write6(reach((0.08, 0.2, 0.0)), 500)
        self.arm.write6(reach((0.08, 0.2, 0.0), 133), 500)
        self.assertIsNone(self.world.held)
        self.assertEqual(self.arm.read()[5], W.EMPTY_READBACK)

    def test_a_block_released_over_another_lands_on_top(self):
        self.world.place('kitchen', [0, 12])
        bid = self.world.place('other', [0, 12])
        self.assertAlmostEqual(self.world.blocks[bid]['pose'][2, 3], self.world.table_z + 0.045, places=4)

    def test_a_block_let_go_over_a_bin_lands_on_its_floor_and_is_reported_inside(self):
        pid = self.world.add_prop('bin', [-10, 14], 30, category='kitchen', inner_cm=[8, 8], height_cm=4, wall_cm=0.2)
        bid = self.world.place('kitchen', [-10, 14])
        snap = self.world.snapshot()
        self.assertEqual(snap['blocks'][0]['inside'], pid)
        self.assertEqual(snap['props'][0]['holds'], [bid])
        self.assertAlmostEqual(snap['blocks'][0]['base_cm'][2], self.world.table_z * 100 + 0.2 + 1.5, places=2)
        outside = self.world.place('other', [5, 8])
        self.assertIsNone(self.world.snapshot()['blocks'][1]['inside'])
        self.world.remove(outside)

    def test_the_designed_bins_stand_where_the_design_file_plans_them(self):
        made = self.world.load_bins('bins_acrylic_v1')
        design = json.loads((ROOT / 'app/config/bins_acrylic_v1.json').read_text())
        self.assertEqual(len(made), len(design['bins']))
        for pid, entry in zip(made, design['bins'].values()):
            p = self.world.props[pid]
            self.assertTrue(np.allclose(p['pose'][:2, 3], entry['planned_centre_base_m'], atol=1e-6))
            self.assertEqual(p['category'], entry['category'])
            flange = p['pose'][:3, :3] @ [0, 1, 0]            # the plate's +Y points along the planned direction
            self.assertAlmostEqual(np.degrees(np.arctan2(flange[1], flange[0])), entry['planned_flange_direction_deg'], places=4)

    def test_the_map_can_be_changed_or_taken_away(self):
        self.assertTrue(self.world.matches_default_map())
        self.world.set_map('map_a4_v3')
        self.assertTrue(self.world.matches_default_map())      # v3 is v2 with the codes under the bins left out
        self.assertNotIn(40, self.world.table.layout)
        self.world.set_map('none')
        self.assertIsNone(self.world.snapshot()['map']['name'])
        self.assertFalse(self.world.matches_default_map())
        for bad in ('../arm_limits', 'arm_limits', 'map_missing'):
            with self.assertRaises(ValueError):
                self.world.set_map(bad)

    def test_an_injected_miss_closes_on_nothing(self):
        bid = self.world.place('other', [0, 10])
        c = self.world.blocks[bid]['pose'][:3, 3]
        self.world.set_faults(1.0)
        self.arm.write6(reach((c[0], c[1], self.world.table_z + 0.012)), 500)
        self.arm.write6(reach((c[0], c[1], self.world.table_z + 0.012), 133), 500)
        self.assertIsNone(self.world.held)
        self.assertEqual(self.arm.read()[5], W.EMPTY_READBACK)

    def test_bad_requests_are_refused(self):
        for bad in (dict(category='metal', map_cm=[0, 10]), dict(category='other', map_cm=[0]),
                    dict(category='other', map_cm=[0, 10], up='syringe'), dict(category='other', map_cm=[900, 10])):
            with self.assertRaises(ValueError):
                self.world.place(**bad)
        with self.assertRaises(ValueError):
            self.world.reset(99)


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class RenderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = W.World(0)
        cls.renderer = R.Renderer(cls.world.items)

    def test_the_map_codes_in_the_picture_give_back_the_camera_pose_that_drew_it(self):
        frame, _ = self.renderer.render(OBSERVE, [])
        image = R.sensor(frame, np.random.default_rng(0))
        markers = detect_markers(image, self.renderer.table.layout)
        self.assertGreaterEqual(len(markers), 3)
        k, dist = load_intrinsics()
        pose = solve_camera_pose(markers, self.renderer.table, k, dist)
        truth = self.renderer.table.base_to_map(self.renderer.base_T_cam(OBSERVE)[:3, 3])
        self.assertLess(pose.reprojection_px, 1.0)
        self.assertLess(float(np.linalg.norm(pose.centre - truth)), 0.003)

    def test_a_block_is_drawn_where_the_forward_camera_model_projects_it(self):
        self.world.reset(0)
        bid = self.world.place('hazardous', [0, 13], 15)
        pose = self.world.blocks[bid]['pose']
        with_block, truth = self.renderer.render(OBSERVE, self.world.render_list()[1])
        empty, _ = self.renderer.render(OBSERVE, [])
        changed = np.abs(with_block - empty).max(axis=2) > 40
        ys, xs = np.nonzero(changed)
        self.assertEqual(len(truth), 1)
        self.assertTrue(truth[0]['expected'])
        x0, y0, x1, y1 = truth[0]['bbox']
        # the hull of the cube's projected corners holds the cube; its shadow may spill a little outside
        inside = (xs >= x0 - 2) & (xs <= x1 + 2) & (ys >= y0 - 2) & (ys <= y1 + 2)
        self.assertGreater(inside.mean(), 0.8)
        uv, _ = self.renderer.project(OBSERVE, pose[:3, 3][None])
        self.assertTrue(changed[int(uv[0, 1]), int(uv[0, 0])])

    def test_without_a_map_no_code_is_seen_and_a_bin_wall_hides_the_block_behind_it(self):
        world = W.World(0, 'none')
        renderer = R.Renderer(world.items)
        renderer.set_map(world.table, world.map_data)
        frame, _ = renderer.render(OBSERVE, [])
        self.assertEqual(detect_markers(R.sensor(frame, np.random.default_rng(0)), TableMap().layout), {})
        bid = world.place('hazardous', [0, 13])
        open_view = renderer.render(OBSERVE, world.render_list()[1])[1][0]['visible_px']
        world.add_prop('box', [0, 9.5], 0, size_cm=[12, 1, 12])         # a tall board between camera and block
        hidden = renderer.render(OBSERVE, world.render_list()[1], world.solids())[1]
        self.assertLess(hidden[0]['visible_px'] if hidden else 0, open_view * 0.5)

    def test_a_block_out_of_view_is_not_in_the_truth(self):
        self.world.reset(0)
        self.world.place('other', [-13, 1])
        _, truth = self.renderer.render([120, 105, 5, 5, 95, 30], self.world.render_list()[1])
        self.assertEqual(truth, [])


@unittest.skipUnless(READY, 'needs numpy and OpenCV')
class CompareTest(unittest.TestCase):
    def test_blocks_are_scored_by_overlap_then_class(self):
        truth = [dict(id='b1', category='other', bbox=[0, 0, 100, 100], expected=True),
                 dict(id='b2', category='kitchen', bbox=[200, 0, 300, 100], expected=True),
                 dict(id='b3', category='hazardous', bbox=[400, 0, 500, 100], expected=True),
                 dict(id='b4', category='recyclable', bbox=[600, 0, 639, 100], expected=False)]
        dets = [dict(category='other', conf=.9, bbox=[2, 2, 98, 101]), dict(category='other', conf=.8, bbox=[205, 0, 300, 100]),
                dict(category='kitchen', conf=.5, bbox=[0, 300, 50, 350]), dict(category='other', conf=.4, bbox=[610, 0, 639, 60])]
        out = E.compare(truth, dets)
        self.assertEqual([b['result'] for b in out['blocks']], ['correct', 'wrong_class', 'missed', 'ignored'])
        self.assertEqual(out['summary'], dict(correct=1, wrong_class=1, missed=1, ignored=1, false_positives=1))


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class ServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sim_server as S
        cls.tmp = tempfile.TemporaryDirectory()
        cls.app, cls.camera = S.build(Path(cls.tmp.name) / 'session', blocks=3, seed=1)
        cls.httpd = S.console.ThreadingHTTPServer(('127.0.0.1', 0), S.handler(cls.app))
        cls.httpd.daemon_threads = True
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()
        threading.Thread(target=cls.app.poll, daemon=True).start()
        cls.base = f'http://127.0.0.1:{cls.httpd.server_port}'
        deadline = time.time() + 20
        while cls.camera.current() is None and time.time() < deadline:
            time.sleep(.05)

    @classmethod
    def tearDownClass(cls):
        cls.app.closed.set()
        cls.app.locator.closed.set()
        cls.camera.close()
        cls.httpd.shutdown()
        cls.httpd.server_close()
        cls.tmp.cleanup()

    def get(self, path, headers=None):
        return urllib.request.urlopen(urllib.request.Request(self.base + path, headers=headers or {}), timeout=20)

    def post(self, path, **body):
        token = json.loads(self.get('/api/bootstrap').read())['token']
        req = urllib.request.Request(self.base + path, json.dumps(dict(owner='test', **body)).encode(),
                                     {'Content-Type': 'application/json', 'X-Console-Token': token,
                                      'Origin': 'http://localhost:5173', 'Host': 'somewhere.else:1'})
        return json.loads(urllib.request.urlopen(req, timeout=30).read())

    def test_status_is_the_console_status_plus_the_scene(self):
        s = json.loads(self.get('/api/status').read())
        self.assertEqual(s['mode'], 'simulation')
        self.assertEqual(s['sim']['blocks'], len(json.loads(self.get('/api/sim/world').read())['blocks']))
        self.assertEqual((s['camera']['kind'], s['camera']['clock'], s['camera']['hardware_fresh']), ('live', 'sim', False))

    def test_the_camera_endpoints_serve_a_640x480_jpeg_to_any_origin(self):
        r = self.get('/snapshot.jpg', {'Origin': 'http://phone.local'})
        image = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertEqual(r.headers['Access-Control-Allow-Origin'], '*')
        self.assertGreater(float(r.headers['X-Frame-Time']), 0)
        self.assertEqual(self.get('/api/frame').headers['Content-Type'], 'image/jpeg')

    def test_a_cross_origin_client_can_edit_the_scene_and_drive_the_arm(self):
        world = self.post('/api/sim/reset', count=0)
        self.assertEqual(world['blocks'], [])
        world = self.post('/api/sim/block', category='kitchen', map_cm=[1, 9], yaw_deg=10, up='apple_core')
        self.assertEqual(world['blocks'][0]['up'], 'apple_core')
        s = self.post('/api/enable', onsite=False)
        s = self.post('/api/jog', joint=1, delta=1, revision=s['revision'])
        self.assertEqual(s['pose'][0], 91)
        self.post('/api/stop')
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.post('/api/sim/block', category='glass', map_cm=[1, 9])
        self.assertEqual(ctx.exception.code, 409)

    def test_map_and_props_can_be_changed_over_http(self):
        self.assertIn('map_a4_v3', json.loads(self.get('/api/sim/options').read())['maps'])
        world = self.post('/api/sim/map', name='map_a4_v3')
        self.assertEqual(world['map']['name'], 'map_a4_v3')
        self.assertEqual(json.loads(self.get('/api/bootstrap').read())['scene']['map']['name'], 'map_a4_v3')
        world = self.post('/api/sim/prop', kind='bin', category='other', map_cm=[12, 16], yaw_deg=0)
        pid = world['props'][-1]['id']
        self.assertEqual(world['props'][-1]['colour'], W.BIN_COLOURS['other'])
        world = self.post('/api/sim/prop/remove', id=pid)
        self.assertNotIn(pid, [p['id'] for p in world['props']])
        self.post('/api/sim/map', name='map_a4_v2')

    def test_the_log_is_served_by_line_number_as_records_and_as_a_stream(self):
        log = Path(self.tmp.name) / 'log.jsonl'
        log.write_text((ROOT / 'app/samples/log_sim_8rounds.jsonl').read_text())
        self.app.log_path = log
        total = len(log.read_text().splitlines())
        page = json.loads(self.get('/api/log?after=0&limit=10').read())
        self.assertEqual((page['total'], page['next'], [e['seq'] for e in page['events']]), (total, 10, list(range(1, 11))))
        tail = json.loads(self.get(f'/api/log?after={total - 2}').read())
        self.assertEqual([e['seq'] for e in tail['events']], [total - 1, total])
        self.assertEqual(json.loads(self.get(f'/api/log?after={total}').read())['events'], [])
        records = json.loads(self.get('/api/log/records').read())['records']
        summary = json.loads(self.get('/api/log/summary').read())
        self.assertEqual(len(records), summary['records'])
        self.assertEqual(summary['rounds_finished'], 8)
        stream = self.get(f'/api/log/stream?after={total - 1}')
        self.assertEqual(stream.headers['Content-Type'], 'text/event-stream; charset=utf-8')
        self.assertEqual(stream.readline(), f'id: {total}\n'.encode())
        self.assertEqual(json.loads(stream.readline()[6:])['event'], 'pipeline_end')
        stream.readline()
        with log.open('a') as f:                       # a line written later arrives on the open stream
            f.write('{"t": 1.0, "event": "stopped", "reason": "test"}\n')
        self.assertEqual(stream.readline(), f'id: {total + 1}\n'.encode())
        stream.close()

    def test_grips_can_be_made_to_miss(self):
        self.assertEqual(self.post('/api/sim/faults', grip_miss=1)['faults'], dict(grip_miss=1.0))
        with self.assertRaises(urllib.error.HTTPError):
            self.post('/api/sim/faults', grip_miss=3)
        self.post('/api/sim/faults', grip_miss=0)

    def test_the_shell_page_and_the_old_page_are_both_served(self):
        self.assertIn('机械臂仿真工作台', self.get('/sim/').read().decode())
        self.assertIn(b'geometry', self.get('/scene.js').read())


if __name__ == '__main__':
    unittest.main()
