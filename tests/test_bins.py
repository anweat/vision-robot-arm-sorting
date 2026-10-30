"""The acrylic bins in the pipeline (src/pipeline/bins.py), their simulated model (src/sim) and the drop over a bin
(console_grasp.plan_put with drop_tip_z). Needs numpy and OpenCV (and the block faces under data/); skipped
where they are missing."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('teleop', 'spatial', 'vision', 'sim', 'pipeline', 'tasks', 'tools'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
try:
    import cv2
    import numpy as np
    import world as W
    import bins as B
    READY = (W.FACES_DIR / 'index.json').exists()
except ImportError:
    READY = False
if READY:
    import console_grasp as G
    import render as R
    from camera_model import ArmCamera
    from map_registration import load_intrinsics

VIEWS = json.loads((ROOT / 'config' / 'bin_views_bins_acrylic_v1.json').read_text())['views']


def see(world, renderer, bias=(0, 0, 0, 0, 0)):
    """The bin views rendered in `world`, as locate() takes them (readback = command)."""
    cam, rng, views = ArmCamera(), np.random.default_rng(0), []
    world.set_faults(0, list(bias))
    for q in VIEWS.values():
        world.servos = list(q) + [92, 30]
        servos, blocks, _ = world.render_list()
        frame, _ = renderer.render(servos, blocks, world.solids(), world.glass(), world.decals())
        views.append(dict(image=R.sensor(frame, rng), base_T_cam=cam.base_T_cam(list(q) + [92])))
    return views


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class DesignTest(unittest.TestCase):
    def test_the_four_bins_load_with_their_categories_and_frame(self):
        bins = B.BinSet()
        self.assertEqual({n: b.category for n, b in bins.bins.items()},
                         {1: 'recyclable', 2: 'kitchen', 3: 'hazardous', 4: 'other'})
        b = bins.bins[1]
        self.assertTrue(np.allclose(b.to_base([0, 0]), b.xy))
        flange = b.to_base([0, 1])                       # +Y of the bin frame points outwards, along J1's bearing
        self.assertAlmostEqual(math.degrees(math.atan2(*(flange - b.xy)[::-1])), 24.0, places=3)
        self.assertTrue(np.allclose(b.to_local(b.to_base([0.013, -0.02])), [0.013, -0.02]))

    def test_half_side_selection(self):
        self.assertEqual(B.parse_only('right'), (1, 2))
        self.assertEqual(B.parse_only('left'), (3, 4))
        self.assertIsNone(B.parse_only('all'))
        self.assertEqual(B.parse_only('4,2'), (2, 4))
        self.assertEqual(sorted(B.BinSet(only=B.parse_only('left')).bins), [3, 4])
        with self.assertRaises(ValueError):
            B.parse_only('middle')
        with self.assertRaises(ValueError):
            B.BinSet(only=(9,))

    def test_containing_and_footprint(self):
        bins = B.BinSet()
        b = bins.bins[2]
        self.assertIs(bins.containing(b.to_base([0.03, -0.03])), b)
        self.assertIsNone(bins.containing(b.to_base([0.05, 0.0])))            # on the wall
        self.assertIs(bins.over_footprint(b.to_base([0.0, 0.058])), b)        # on the flange: the stickers
        self.assertIsNone(bins.containing(b.to_base([0.0, 0.058])))

    def test_the_planned_layout_leaves_two_millimetres_between_neighbouring_plates(self):
        """docs/bins/README §10: 1.9 mm between 1-2 and 3-4 (the check says so, and so does this)."""
        problems = B.table_check(B.BinSet(), 5.0)
        self.assertEqual(len(problems), 2)
        self.assertTrue(all('1.' in p for p in problems))
        self.assertEqual(B.table_check(B.BinSet(), 1.0), [])

    def test_the_view_poses_are_the_ones_the_finder_would_pick(self):
        cfg = json.loads((ROOT / 'config' / 'bin_views_bins_acrylic_v1.json').read_text())
        self.assertEqual(sorted(cfg['views']), ['1', '2', '3', '4'])
        self.assertEqual(cfg['design'], B.DEFAULT_DESIGN)


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class LocateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k, cls.dist = load_intrinsics()

    def stand(self, **jitter):
        world = W.World(0, 'map_a4_v3')
        world.load_bins('bins_acrylic_v1', **jitter)
        renderer = R.Renderer(world.items)
        renderer.set_map(world.table, world.map_data)
        return world, renderer

    def errors(self, world, found):
        out = {}
        for p in world.props.values():
            f = found[p['number']]
            yaw = math.degrees(math.atan2(p['pose'][1, 0], p['pose'][0, 0]))
            out[p['number']] = (float(np.linalg.norm(f['xy'] - p['pose'][:2, 3])) * 1000, (f['yaw_deg'] - yaw + 180) % 360 - 180)
        return out

    def test_bins_set_down_off_their_plan_are_found_from_their_stickers(self):
        world, renderer = self.stand(jitter_mm=12, jitter_deg=6, seed=3)
        found = B.locate(see(world, renderer), B.BinSet(), self.k, self.dist, world.table_z)
        self.assertEqual(sorted(found), [1, 2, 3, 4])
        for number, (mm, deg) in self.errors(world, found).items():
            self.assertLess(mm, 1.0, number)
            self.assertLess(abs(deg), 0.3, number)
            self.assertEqual(len(found[number]['markers']), 3)

    def test_a_joint_standing_off_its_readback_shifts_every_bin_by_the_same_few_millimetres(self):
        """The camera pose comes from the readback: a 1.5 deg offset on J1 moves the bins ~6 mm, which is the
        accuracy this method has on the real arm too (camera_model.py: about a centimetre)."""
        world, renderer = self.stand(jitter_mm=8, jitter_deg=4, seed=3)
        found = B.locate(see(world, renderer, (1.5, -1, 1, -1, 0)), B.BinSet(), self.k, self.dist, world.table_z)
        for number, (mm, deg) in self.errors(world, found).items():
            self.assertLess(mm, 12, number)
            self.assertLess(abs(deg), 3, number)

    def test_a_block_over_the_floor_code_does_not_stop_the_bin_being_found(self):
        world, renderer = self.stand()
        centre = world.table.base_to_map([*B.BinSet().bins[2].xy, 0])[:2] * 100
        world.place('kitchen', list(centre), 10)
        found = B.locate(see(world, renderer), B.BinSet(), self.k, self.dist, world.table_z)
        self.assertEqual(found[2]['markers'], [21, 22])
        self.assertLess(self.errors(world, found)[2][0], 1.0)

    def test_a_bin_no_view_shows_is_not_in_the_result_and_keeps_its_plan(self):
        world, renderer = self.stand()
        views = see(world, renderer)[:2]                      # only bins 1 and 2
        bins = B.apply(B.BinSet(), B.locate(views, B.BinSet(), self.k, self.dist, world.table_z))
        self.assertEqual([bins.bins[n].source for n in (1, 2, 3, 4)], ['markers', 'markers', 'planned', 'planned'])


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class SimBinTest(unittest.TestCase):
    def setUp(self):
        self.world = W.World(0, 'map_a4_v3')
        self.world.load_bins('bins_acrylic_v1')
        self.arm = W.WorldArm(self.world)
        self.bin = B.BinSet().bins[2]

    def release(self, local, yaw=0.0, above=0.07):
        """Hold a block over the bin at `local` (bin frame, m) `above` the table and let it go."""
        w = self.world
        m = w.table.base_to_map([*self.bin.to_base(local), 0])[:2] * 100
        bid = w.place('kitchen', list(m), yaw)
        b = w.blocks[bid]
        b['pose'][2, 3] = w.table_z + above
        w.held, b['grip'] = bid, np.eye(4)
        w.arm_moved([90, 105, 5, 5, 95, 11])                  # the gripper opens (30 -> 11 is "opening" in its readback)
        w.servos[5] = 30
        return w.snapshot()['blocks'][0]

    def test_a_block_let_go_over_the_middle_lands_on_the_floor_inside(self):
        b = self.release([0.0, 0.0])
        self.assertEqual((b['inside'], b['perched']), ('p2', False))
        self.assertAlmostEqual(b['base_cm'][2], (self.world.table_z + 0.002 + 0.015) * 100, places=1)

    def test_a_block_whose_corner_is_over_the_wall_hangs_on_the_rim(self):
        b = self.release([0.032, 0.0], yaw=45 + self.bin.yaw_deg)   # 45 deg to the walls: centre inside, corner 2.1 cm out
        self.assertTrue(b['perched'])
        self.assertIsNone(b['inside'])
        self.assertGreater(b['base_cm'][2], (self.world.table_z + 0.042) * 100)

    def test_a_block_let_go_beside_the_bin_is_not_inside(self):
        b = self.release([0.075, 0.0])
        self.assertIsNone(b['inside'])

    def test_walls_are_clear_the_codes_are_stickers_and_the_plate_is_drawn_under_them(self):
        w = self.world
        self.assertEqual(len(w.glass()), 4 * 5)               # plate + four walls, four bins
        self.assertEqual(w.solids(), [])
        marks = [d['id'] for d in w.decals() if d['kind'] == 'marker']
        self.assertEqual(sorted(marks), [10, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42])

    def test_standing_the_bins_again_replaces_them_and_half_a_side_can_be_chosen(self):
        self.world.load_bins('bins_acrylic_v1', only=(3, 4))
        self.assertEqual(sorted(p['number'] for p in self.world.props.values()), [3, 4])

    def test_the_arm_coming_down_on_a_bin_is_recorded_as_a_contact_and_not_stopped(self):
        pose = G.kin.inverse_within(*self.bin.xy, self.world.table_z + 0.02, {j: (0, 180) for j in (1, 2, 3, 4)})
        self.assertEqual(self.world.contact_count, 0)
        self.arm.write6([round(v) for v in pose['servos']] + [95, 30], 0)
        self.assertGreater(self.world.contact_count, 0)
        self.assertEqual(self.world.contacts[-1]['prop'], 'p2')

    def test_a_drop_over_the_opening_at_drop_height_touches_nothing(self):
        pose = G.kin.inverse_within(*self.bin.xy, G.bin_drop_tip_z(0.042), {j: (0, 180) for j in (1, 2, 3, 4)})
        self.arm.write6([round(v) for v in pose['servos']] + [95, 30], 0)
        self.assertEqual(self.world.contact_count, 0)

    def test_fault_inputs_are_checked(self):
        for bad in ([1, 2, 3], [0, 0, 0, 0, 99], 'x'):
            with self.assertRaises(ValueError):
                self.world.set_faults(None, bad)
        self.assertEqual(self.world.set_faults(None, [1, 0, 0, 0, 0]), dict(grip_miss=0.0, servo_bias_deg=[1.0, 0, 0, 0, 0]))

    def test_the_hidden_offset_moves_the_arm_but_not_its_readback(self):
        self.world.set_faults(None, [3, 0, 0, 0, 0])
        self.arm.write6([90, 105, 5, 5, 95, 30], 0)
        self.assertEqual(self.arm.read()[:5], [90, 105, 5, 5, 95])
        self.assertEqual(self.world.render_list()[0][0], 93)


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class DropPlanTest(unittest.TestCase):
    OBSERVE = [90, 105, 5, 5, 92]

    def setUp(self):
        G.DX, G.DY = 0.0005, 0.0675
        self.z = G.bin_drop_tip_z(B.BinSet().wall_top_m)

    def test_the_drop_height_clears_the_rim_by_the_block_hanging_below_the_fingertips(self):
        self.assertAlmostEqual(self.z, (-0.0142 + 0.042 + 0.012 + 0.015), places=4)

    def test_every_bin_centre_can_be_dropped_over_and_the_bins_are_not_swept_on_the_way(self):
        for number, b in B.BinSet().bins.items():
            where = ','.join(str(round(v * 100 - o * 100, 2)) for v, o in zip(b.xy, (G.DX, G.DY)))
            plan = G.plan_put(150, where, self.OBSERVE + [30], self.z)
            self.assertIsNotNone(plan, number)
            self.assertAlmostEqual(G.tip(plan['hover'])[2], self.z, delta=0.006)
            # the carry height (tip) keeps the held block's underside above the rim all the way round
            self.assertGreater(G.tip(plan['carry'])[2] - G.GRASP_HEIGHT, -0.0142 + 0.042)

    def test_without_a_drop_height_the_plan_is_the_old_one(self):
        plan = G.plan_put(150, '40', self.OBSERVE + [30])
        self.assertIsNone(plan['drop_tip_z'])
        self.assertAlmostEqual(G.tip(plan['hover'])[2], G.TABLE_Z + G.BLOCK + G.HOVER_ABOVE_TOP, delta=0.006)


if __name__ == '__main__':
    unittest.main()
