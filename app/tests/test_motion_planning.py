"""src/planning: the four-bar gripper, the grasp / release solvers, A* routes and MotionPlans, all against
collision maps (numpy; no hardware)."""

import json
import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import planning  # noqa: E402
from planning import body as B  # noqa: E402
from planning import checks as K  # noqa: E402
from planning import config as C  # noqa: E402
from planning import gripper as GR  # noqa: E402
from planning import grasp as GP  # noqa: E402
from planning import planner as P  # noqa: E402
from planning import scene as S  # noqa: E402
from planning import solver as V  # noqa: E402
from planning import tuning as TU  # noqa: E402
from planning.collision_map import CollisionMap  # noqa: E402
from planning.grasp import Target  # noqa: E402
from planning.release import Destination  # noqa: E402
import dofbot_kinematics as kin  # noqa: E402
import gripper_geometry as gg  # noqa: E402
import workspace  # noqa: E402

OBSERVE = [90, 105, 5, 5, 92]
T = TU.default()
# the five fingertip touches of teleop session 20260925_005756_78eb (J6 168)
TOUCHES = [[90, 31, 62, 26, 90, 168], [90, 47, 44, 17, 90, 168], [72, 46, 45, 18, 90, 168],
           [76, 23, 77, 21, 90, 168], [59, 28, 74, 11, 90, 168]]
NO_ERRORS = {"errors": {k: 0.0 for k in ("target_xy_m", "target_yaw_deg", "obstacle_xy_m", "obstacle_height_m",
                                         "table_z_m", "arm_xy_m", "arm_z_m")}}


def world(boxes=(), target=None, overrides=None):
    """A World over boxes; the target block (if given) is put in the map too and returned as a Target."""
    t = TU.load(overrides=overrides) if overrides else T
    boxes = list(boxes) + ([S.block(target[0], target[1], target[2] or 0.0, "tgt")] if target else [])
    m = CollisionMap.from_boxes(boxes, t.table_z)
    w = planning.world(m, t)
    return (w, Target.from_map(m, "tgt") if target else None)


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


class BodyTest(unittest.TestCase):
    def test_link_frames_follow_the_kinematics(self):
        for q in ([90, 105, 5, 5], [60, 80, 30, 20], [130, 40, 90, 60]):
            f = B.link_frames(q)
            fk = kin.forward(q)
            for got, want in ((f[1], "shoulder"), (f[3], "wrist")):
                self.assertLess(math.dist(got[:3, 3], fk[want]), 1e-4)      # forward() rounds to 0.1 mm

    def test_vectorized_platform_keep_out_is_the_console_one(self):
        rnd = random.Random(3)
        for _ in range(200):
            q = [rnd.uniform(20, 160), rnd.uniform(5, 175), rnd.uniform(-3, 175), rnd.uniform(-3, 175)]
            self.assertAlmostEqual(K.platform_keep_out(q), workspace.platform_clearance(q, K.KEEP_OUT), places=9)


class TuningTest(unittest.TestCase):
    def test_errors_widen_the_clearances(self):
        t0 = TU.load(overrides=NO_ERRORS)
        self.assertAlmostEqual(t0.neighbour(), t0.clear["neighbour_m"], places=9)
        self.assertGreater(T.neighbour(), t0.neighbour())
        rss = TU.load(overrides={"combine": "rss"})
        worst = TU.load(overrides={"combine": "sum"})
        self.assertLess(rss.neighbour(), worst.neighbour())
        self.assertAlmostEqual(worst.neighbour(), worst.clear["neighbour_m"] + worst.errors["obstacle_xy_m"]
                               + worst.errors["arm_xy_m"], places=9)

    def test_a_maps_own_sigma_wins_when_larger(self):
        m = CollisionMap.from_boxes([S.block(0.0, 0.2)], T.table_z, sigma_xy_m=0.02)
        self.assertAlmostEqual(T.with_map(m).obstacle_xy(), 0.02)

    def test_trims_reach_the_plan(self):
        w, tgt = world(target=(0.0, 0.20, 0.0), overrides={"trims": {"j5_trim_deg": -3, "grasp_offset_m": [0.004, 0, 0]}})
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons)
        grip = plan.steps[-2]
        self.assertEqual(grip.command[4], round(grip.joints[4] - 3))           # commands trimmed, checks on the model
        self.assertAlmostEqual(plan.summary["grasp"]["pad_cm"][1], 20.4, delta=0.01)


class RollTest(unittest.TestCase):
    def test_j5_turns_the_real_jaw_the_other_way_from_servo5_pose(self):
        # 2026-09-26: J1 71, block yaw -10.9, J5 100 held it square
        q = kin.inverse(0.063, 0.1935, 0.0, 150)
        bearing, tilt = V.jaw_bearing_tilt(list(q) + [100])
        self.assertLess(abs(V.fold90(bearing + 10.9)), 2.0)

    def test_the_verified_grasp_is_planned_with_the_verified_roll(self):
        w, tgt = world(target=(0.063, 0.1935, -10.9))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertAlmostEqual(plan.summary["j5"], 100, delta=1.0)
        self.assertTrue(plan.summary["aligned"])


class PickTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w, cls.tgt = world(target=(0.0, 0.20, 0.0))
        cls.plan = P.plan_pick(cls.w, OBSERVE, cls.tgt)

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
        self.assertAlmostEqual(s["pad_above_table_cm"], T.trims["pad_centre_height_m"] * 100, delta=0.01)
        self.assertGreaterEqual(s["fingertips_above_table_cm"], T.table(T.clear["fingertip_table_m"]) * 100 - 1e-6)
        g = self.plan.steps[-2]
        self.assertGreaterEqual(GR.lowest(g.joints, GR.CLOSED) - T.table_z, T.table(T.clear["closed_fingertip_table_m"]))

    def test_a_rolled_jaw_raises_the_pads(self):
        w, tgt = world(target=(0.0, 0.17, 30.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertGreater(plan.summary["pad_above_table_cm"], 1.8)
        self.assertLess(plan.summary["pad_above_table_cm"], 2.7)

    def test_a_block_at_45_deg_far_out_is_squared_by_a_level_jaw(self):
        # 2026-11-20: no flat squared grasp there, but the level jaw is within squaring_max_deg of its faces and
        # the closing pads turn it square (real grips held 36-39 deg off, doc 34 §8)
        w, tgt = world(target=(0.05, 0.24, 45.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons[:3])
        g = plan.summary["grasp"]
        self.assertFalse(g["aligned"])
        self.assertLessEqual(g["residual_deg"], C.G["squaring_max_deg"])
        self.assertLessEqual(g["tilt_deg"], C.G["max_grasp_tilt_deg"])

    def test_the_relaxed_tiers_still_keep_off_a_wall_beside_the_block(self):
        # only neighbouring blocks are relaxed: a bin wall (any non-block object) keeps every clearance
        wall = S.box_on_table(0.037, 0.20, 0.004, 0.08, 0.042, 0.0, "wall")
        w, tgt = world([wall], (0.0, 0.20, 0.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        if plan.ok:
            for st in plan.steps:
                self.assertFalse(st.check.worst_obstacle == "wall" and st.check.obstacles < 0, st.name)
        else:
            self.assertTrue(any("wall" in r for r in P.pick_plans.last_why))

    def test_a_retry_turns_the_jaw_or_says_it_could_not(self):
        # 2026-11-20 (doc 34 §8): an empty grip is retried across the other pair of faces when one is possible
        w, tgt = world(target=(0.0, 0.20, 10.0))
        first = P.plan_pick(w, OBSERVE, tgt)
        again = next(P.pick_plans(w, OBSERVE, tgt, avoid_j5=[first.summary["j5"]]))
        if again.summary.get("retry_same_jaw"):
            self.assertEqual(again.summary["j5"], first.summary["j5"])
        else:
            self.assertGreaterEqual(P._jaw_turn(again.summary["j5"], first.summary["j5"]), P.AVOID_J5_DEG)

    def test_a_grasp_never_tilts_the_jaw_past_the_limit(self):
        # quarter-turning the jaw at pitch ~150 tilts it ~30 deg; the real grip with a 29 deg tilt closed on nothing
        w, tgt = world([S.block(0.045, 0.20, name="n1"), S.block(-0.045, 0.20, name="n2")], (0.0, 0.20, 0.0), NO_ERRORS)
        for g in GP.candidates(w, tgt):
            self.assertLessEqual(g.tilt_deg, C.G["max_grasp_tilt_deg"])

    def test_near_the_base_steep_pitches_reach_and_the_platform_ends_it(self):
        # 2026-11-15: 13-15 cm out was refused because solve_tcp checked the joint floor on its first guess
        # and the pitch list ended at 170; it is reached at 170-180. At 11 cm the gripper servo's housing
        # meets the platform lid (mesh check), whatever the pitch.
        w, tgt = world(target=(0.0757, 0.1326, 9.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons[:3])
        self.assertGreaterEqual(plan.summary["pitch"], 165)
        self.assertFalse(plan.summary["pitch_verified"])
        w, tgt = world(target=(0.0, 0.11, 0.0))
        self.assertFalse(P.plan_pick(w, OBSERVE, tgt).ok)

    def test_pitches_are_solved_per_target(self):
        # 2026-11-16: no fixed pitch list and no reach cap; steep near the base, flat far out
        tz = TU.default().table_z + 0.018
        near = V.pitch_range([0.0, 0.12, tz], C.ROLL_ZERO, GR.contact())
        far = V.pitch_range([0.0, 0.30, tz], C.ROLL_ZERO, GR.contact())
        self.assertGreaterEqual(min(near), 165)
        self.assertGreater(max(near), 180)
        self.assertLess(max(far), 135)
        self.assertEqual(V.pitch_range([0.0, 0.34, tz], C.ROLL_ZERO, GR.contact()), [])

    def test_a_block_30_cm_out_is_reached_with_the_arm_stretched(self):
        w, tgt = world(target=(0.0, 0.30, 0.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons[-3:])
        self.assertLess(plan.summary["pitch"], 135)

    def test_solve_tcp_checks_the_floor_on_the_converged_pose(self):
        from planning import solver as V
        pad = [0.0, 0.12, TU.default().table_z + 0.018]
        q = V.solve_tcp(pad, 175, C.ROLL_ZERO, GR.contact())
        self.assertIsNotNone(q)
        self.assertGreaterEqual(min(q[1:4]), C.LIMITS[3][0])
        got = GR.pad_centre(q + [C.ROLL_ZERO], GR.contact())
        self.assertLess(max(abs(a - b) for a, b in zip(got, pad)), 2e-4)

    def test_a_turned_block_uses_j5_past_180_and_says_so(self):
        # yaw -20 at J1 90 wants the jaw turned the other way from its zero: J5 ~250 (half a turn on), which
        # is inside the course limit but past the 180 that has run on the arm
        w, tgt = world(target=(0.0, 0.20, -20.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons[:3])
        self.assertTrue(plan.summary["aligned"])
        self.assertGreater(plan.summary["j5"], 180)
        self.assertFalse(plan.summary["j5_verified"])


class ObstacleTest(unittest.TestCase):
    def test_neighbours_out_along_the_arm_leave_the_wide_opening(self):
        w, tgt = world([S.block(0.0, 0.245), S.block(0.0, 0.155)], (0.0, 0.20, 0.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertEqual(plan.summary["opening"], "open")

    def test_neighbours_beside_it_narrow_the_opening(self):
        w, tgt = world([S.block(0.06, 0.20, name="n1"), S.block(-0.06, 0.20, name="n2")], (0.0, 0.20, 0.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons)
        self.assertEqual(plan.summary["opening"], "narrow")

    def tiers(self, *names):
        old = C.G.get("tiers")
        C.G["tiers"] = list(names)
        self.addCleanup(C.G.__setitem__, "tiers", old)

    def test_the_error_budget_decides_a_tight_spot(self):
        # 5.5 cm apart: refused with the configured scene errors, a narrower opening without them
        self.tiers("full")
        boxes = [S.block(0.055, 0.20, name="n1"), S.block(-0.055, 0.20, name="n2")]
        w, tgt = world(boxes, (0.0, 0.20, 0.0))
        tight = P.plan_pick(w, OBSERVE, tgt)
        self.assertFalse(tight.ok)
        self.assertTrue(any("n1" in r or "n2" in r for r in tight.reasons))
        w0, tgt0 = world(boxes, (0.0, 0.20, 0.0), NO_ERRORS)
        exact = P.plan_pick(w0, OBSERVE, tgt0)
        self.assertTrue(exact.ok, exact.reasons)
        self.assertIn(exact.summary["opening"], ("narrow", "tight"))

    def test_with_the_full_tier_only_neighbours_too_close_are_named(self):
        self.tiers("full")
        w, tgt = world([S.block(0.045, 0.20, name="n1"), S.block(-0.045, 0.20, name="n2")], (0.0, 0.20, 0.0), NO_ERRORS)
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertFalse(plan.ok)
        self.assertTrue(any("n1" in r or "n2" in r for r in plan.reasons))

    def test_the_tiers_relax_only_the_neighbours_and_say_so(self):
        # 2026-11-20 (doc 34 §8): a tight spot the full clearances refuse is taken at a later tier, flagged
        boxes = [S.block(0.045, 0.20, name="n1"), S.block(-0.045, 0.20, name="n2")]
        w, tgt = world(boxes, (0.0, 0.20, 0.0))
        plan = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(plan.ok, plan.reasons[:3])
        self.assertIn(plan.summary["grasp_tier"], ("touching", "pipeline"))
        self.assertTrue(plan.summary["tier_note"])
        self.assertTrue(any(r.startswith("[full]") for r in P.pick_plans.last_why))
        # a clear table needs no relaxing
        w, tgt = world([], (0.0, 0.20, 0.0))
        self.assertEqual(P.plan_pick(w, OBSERVE, tgt).summary["grasp_tier"], "full")
        # the pipeline tier still keeps the platform, the table and the target's core
        self.tiers("pipeline")
        w, tgt = world([], (0.0, 0.11, 0.0))
        self.assertFalse(P.plan_pick(w, OBSERVE, tgt).ok)


class RouteTest(unittest.TestCase):
    A = V.solve_tcp([-0.06, 0.20, C.TABLE_Z + 0.05], 150, 92, GR.contact()) + [92]
    Bq = V.solve_tcp([0.10, 0.20, C.TABLE_Z + 0.05], 150, 92, GR.contact()) + [92]

    def test_walls_in_the_way_are_gone_over_by_astar(self):
        for h in (0.06, 0.14):
            w, _ = world([S.box_on_table(0.02, 0.20, 0.02, 0.08, h, 0.0, "wall")])
            self.assertFalse(K.move_check(w, self.A, self.Bq, GR.delta_for_j6(30)).ok)   # straight hits it
            plan = P.plan_transit(w, self.A, self.Bq, 30)
            self.assertTrue(plan.ok, plan.reasons)
            self.assertEqual(plan.summary["method"], "astar")
            self.assertGreater(len(plan.steps), 1)
            self.assertTrue(all(s.check.ok for s in plan.steps))

    def test_a_clear_way_stays_straight(self):
        plan = P.plan_transit(world()[0], self.A, self.Bq, 30)
        self.assertTrue(plan.ok)
        self.assertEqual(plan.summary["method"], "direct")

    def test_the_arm_links_are_checked_too(self):
        # a column beside the forearm's sweep: the gripper alone would pass over it
        w, _ = world([S.box_on_table(-0.07, 0.12, 0.03, 0.03, 0.14, 0.0, "column")])
        straight = K.move_check(w, OBSERVE, [140, 60, 40, 40, 92], GR.delta_for_j6(30))
        self.assertLess(straight.arm_body, 0)
        plan = P.plan_transit(w, OBSERVE, [140, 60, 40, 40, 92], 30)
        self.assertTrue(plan.ok, plan.reasons)

    def test_a_start_inside_something_is_named(self):
        w, _ = world([S.box_on_table(0.0, 0.20, 0.02, 0.02, 0.15, 0.0, "post")])
        a = V.solve_tcp([0.0, 0.20, C.TABLE_Z + 0.05], 150, 92, GR.contact()) + [92]
        plan = P.plan_transit(w, a, self.Bq, 30)
        self.assertFalse(plan.ok)
        self.assertIn("start pose", plan.reasons[0])


class PlaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.w, cls.tgt = world(target=(0.0, 0.20, 0.0))
        cls.pick = P.plan_pick(cls.w, OBSERVE, cls.tgt)
        cls.wp = cls.w.without(cls.tgt.label)

    def test_set_down_level_on_the_table(self):
        plan = P.plan_place(self.wp, self.pick.end, self.pick.held, Destination(-0.06, 0.18))
        self.assertTrue(plan.ok, plan.reasons)
        self.assertAlmostEqual(plan.summary["block_bottom_above_table_cm"], C.R["table_gap_m"] * 100, delta=0.05)
        self.assertLessEqual(plan.summary["block_level_deg"], C.R["max_block_tilt_deg"]["table"])
        self.assertEqual([s.name for s in plan.steps][-3:], ["lower", "release", "back off"])

    def test_a_yaw_the_wrist_cannot_turn_to_is_refused_with_the_reason(self):
        plan = P.plan_place(self.wp, self.pick.end, self.pick.held, Destination(-0.06, 0.20, 0.0))
        self.assertFalse(plan.ok)
        self.assertTrue(any("J5" in r for r in plan.reasons))

    def test_dropped_into_a_bin_over_its_rim(self):
        bins = [b for v in S.planned_bins().values() for b in v]
        w, tgt = world(bins, (0.0, 0.20, 0.0))
        pick = P.plan_pick(w, OBSERVE, tgt)
        self.assertTrue(pick.ok, pick.reasons)
        centre = json.loads((ROOT / "config" / f"{C.BINS_NAME}.json").read_text())["bins"]["2"]["planned_centre_base_m"]
        plan = P.plan_place(w.without(tgt.label), pick.end, pick.held, Destination(centre[0], centre[1], None, S.bin_rim_z()))
        self.assertTrue(plan.ok, plan.reasons)
        rim_above = (S.bin_rim_z() - C.TABLE_Z) * 100
        self.assertGreaterEqual(plan.summary["block_bottom_above_table_cm"], rim_above + C.R["bin_clearance_m"] * 100 - 0.05)
        self.assertTrue(all(s.check.ok for s in plan.steps))

    def test_the_grasp_is_chosen_for_where_the_block_goes(self):
        # a steep near pick leaves the block tilted for a far set-down; the joint plan finds one that is level
        w, tgt = world(target=(0.003, 0.16, 0.0))
        pick, place = P.plan_pick_and_place(w, OBSERVE, tgt, Destination(-0.05, 0.20))
        self.assertTrue(pick.ok and place.ok, (pick.reasons, place.reasons))


if __name__ == "__main__":
    unittest.main()
