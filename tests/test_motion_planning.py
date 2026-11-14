"""src/planning: the four-bar gripper model and the MotionPlans built on it (pure python, no hardware)."""

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from planning import config as C  # noqa: E402
from planning import gripper as GR  # noqa: E402
from planning import planner as P  # noqa: E402
from planning import scene as S  # noqa: E402
from planning import solver as V  # noqa: E402
import dofbot_kinematics as kin  # noqa: E402
import gripper_geometry as gg  # noqa: E402

OBSERVE = [90, 105, 5, 5, 92]
# the five fingertip touches of teleop session 20260925_005756_78eb (J6 168)
TOUCHES = [[90, 31, 62, 26, 90, 168], [90, 47, 44, 17, 90, 168], [72, 46, 45, 18, 90, 168],
           [76, 23, 77, 21, 90, 168], [59, 28, 74, 11, 90, 168]]


class FourBarTest(unittest.TestCase):
    def test_both_sides_are_30_mm_parallelograms(self):
        for side in gg.load()["sides"].values():
            self.assertAlmostEqual(side["link_m"], 0.030, delta=0.0002)
            self.assertAlmostEqual(side["link_lower_m"], side["link_m"], delta=1e-5)

    def test_opening_pulls_the_pads_back_26_mm(self):
        closed = gg.pad_centre_link(GR.contact())[2]
        open_ = gg.pad_centre_link(GR.OPENINGS[0].delta)[2]
        self.assertAlmostEqual(closed, 0.1006, delta=0.0005)
        self.assertAlmostEqual(closed - open_, 0.026, delta=0.0015)

    def test_widest_opening_of_the_mesh(self):
        self.assertAlmostEqual(gg.max_gap(), 0.0614, delta=0.001)
        self.assertAlmostEqual(gg.gap_for_delta(GR.contact()), 0.030, delta=1e-5)

    def test_the_old_one_centimetre_offset_is_the_pad_retreat(self):
        # verified console grasp: FK tip 1 cm radially past the block centre at pitch 150. The closed pads'
        # middle comes back 9.7 mm of it; with the URDF's 2.15 mm J5 offset it lands 2.6 mm past the centre
        q = kin.inverse(0.0, 0.21, 0.0, 150)
        pad = GR.pad_centre(list(q) + [C.ROLL_ZERO], GR.contact())
        self.assertAlmostEqual(math.hypot(pad[0], pad[1]), 0.20, delta=0.003)

    def test_table_seen_by_these_fingers(self):
        r = gg.table_from_touches(TOUCHES)
        self.assertAlmostEqual(r["table_z_m"], C.TABLE_Z, delta=0.0005)
        self.assertLess(r["spread_m"], 0.0015)


class RollTest(unittest.TestCase):
    def test_j5_turns_the_real_jaw_the_other_way_from_servo5_pose(self):
        # 2026-09-26: J1 71, block yaw -10.9, J5 100 held it square
        q = kin.inverse(0.063, 0.1935, 0.0, 150)
        bearing, tilt = V.jaw_bearing_tilt(list(q) + [100])
        self.assertLess(abs(V.fold90(bearing + 10.9)), 2.0)

    def test_the_verified_grasp_is_planned_with_the_verified_roll(self):
        plan = P.plan_pick(OBSERVE, 0.063, 0.1935, -10.9)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertAlmostEqual(plan.summary["j5"], 100, delta=1.0)
        self.assertTrue(plan.summary["aligned"])


class PickTest(unittest.TestCase):
    def setUp(self):
        self.plan = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0)

    def test_steps_and_their_checks(self):
        self.assertTrue(self.plan.ok, self.plan.reasons)
        names = [s.name for s in self.plan.steps]
        self.assertEqual(names[-4:], ["open", "descend", "grip", "lift"])
        self.assertTrue(all(s.check.ok for s in self.plan.steps))
        self.assertTrue(all(C.LIMITS[j][0] <= s.joints[j - 1] <= C.LIMITS[j][1]
                            for s in self.plan.steps for j in range(1, 6)))
        json.dumps(self.plan.to_dict())

    def test_pads_close_around_the_middle_and_the_tips_stay_off_the_table(self):
        s = self.plan.summary
        self.assertAlmostEqual(s["pad_above_table_cm"], C.G["pad_centre_height_m"] * 100, delta=0.01)
        self.assertGreaterEqual(s["fingertips_above_table_cm"], C.G["fingertip_table_margin_m"] * 100)
        g = self.plan.steps[-2]
        self.assertGreaterEqual(GR.lowest(g.joints, GR.CLOSED) - C.TABLE_Z, C.G["closed_fingertip_table_margin_m"])

    def test_a_rolled_jaw_raises_the_pads(self):
        plan = P.plan_pick(OBSERVE, 0.0, 0.17, 30.0)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertGreater(plan.summary["pad_above_table_cm"], 1.8)
        self.assertLess(plan.summary["pad_above_table_cm"], 2.5)

    def test_a_block_at_45_deg_far_out_has_no_flat_grasp(self):
        plan = P.plan_pick(OBSERVE, 0.05, 0.24, 45.0)
        self.assertFalse(plan.ok)
        self.assertTrue(plan.reasons)


class ObstacleTest(unittest.TestCase):
    def test_neighbours_out_along_the_arm_leave_the_wide_opening(self):
        plan = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0, [S.block(0.0, 0.245), S.block(0.0, 0.155)])
        self.assertTrue(plan.ok, plan.reasons)
        self.assertEqual(plan.summary["opening"], "open")

    def test_neighbours_beside_it_narrow_the_opening(self):
        plan = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0, [S.block(0.055, 0.20), S.block(-0.055, 0.20)])
        self.assertTrue(plan.ok, plan.reasons)
        self.assertEqual(plan.summary["opening"], "narrow")

    def test_neighbours_too_close_beside_it_are_named(self):
        plan = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0, [S.block(0.045, 0.20, name="n1"), S.block(-0.045, 0.20, name="n2")])
        self.assertFalse(plan.ok)
        self.assertTrue(any("n1" in r or "n2" in r for r in plan.reasons))

    def test_a_tall_box_in_the_way_is_gone_over(self):
        a = V.solve_tcp([0.0, 0.20, C.TABLE_Z + 0.05], 150, 92, GR.contact()) + [92]
        b = V.solve_tcp([0.11, 0.15, C.TABLE_Z + 0.05], 150, 92, GR.contact()) + [92]
        wall = S.box_on_table(0.055, 0.20, 0.02, 0.06, 0.10, 0.0, "wall")
        self.assertLess(P.K.move_check(a, b, GR.delta_for_j6(30), None, [(wall, 0.0)]).obstacles, 0)   # straight hits it
        plan = P.plan_transit(a, b, 30, None, [wall])
        self.assertTrue(plan.ok, plan.reasons)
        self.assertGreater(len(plan.steps), 1)
        self.assertTrue(all(s.check.ok for s in plan.steps))


class PlaceTest(unittest.TestCase):
    def setUp(self):
        self.pick = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0)

    def test_set_down_level_on_the_table(self):
        plan = P.plan_place(self.pick.end, self.pick.held, -0.06, 0.18)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertAlmostEqual(plan.summary["block_bottom_above_table_cm"], C.R["table_gap_m"] * 100, delta=0.05)
        self.assertLessEqual(plan.summary["block_level_deg"], C.R["max_block_tilt_deg"]["table"])
        self.assertEqual([s.name for s in plan.steps][-3:], ["lower", "release", "back off"])

    def test_a_yaw_the_wrist_cannot_turn_to_is_refused_with_the_reason(self):
        plan = P.plan_place(self.pick.end, self.pick.held, -0.06, 0.20, 0.0)
        self.assertFalse(plan.ok)
        self.assertTrue(any("J5" in r for r in plan.reasons))

    def test_dropped_into_a_bin_over_its_rim(self):
        bins = S.planned_bins()
        others = [b for v in bins.values() for b in v]
        pick = P.plan_pick(OBSERVE, 0.0, 0.20, 0.0, others)
        self.assertTrue(pick.ok, pick.reasons)
        centre = json.loads((ROOT / "config" / f"{C.BINS_NAME}.json").read_text())["bins"]["2"]["planned_centre_base_m"]
        plan = P.plan_place(pick.end, pick.held, centre[0], centre[1], None, others, rim_z=S.bin_rim_z())
        self.assertTrue(plan.ok, plan.reasons)
        rim_above = (S.bin_rim_z() - C.TABLE_Z) * 100
        self.assertGreaterEqual(plan.summary["block_bottom_above_table_cm"], rim_above + C.R["bin_clearance_m"] * 100 - 0.05)
        self.assertTrue(all(s.check.ok for s in plan.steps))


if __name__ == "__main__":
    unittest.main()
