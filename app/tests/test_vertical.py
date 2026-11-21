"""Vertical solving (doc 40): stacked blocks in the collision map, picking off and setting onto a block, setting
into a bin instead of dropping, release openings, and the readback correction of a set-down."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

import planning  # noqa: E402
from planning import config as C, correct as CR, gripper as GR, planner as PP, scene as S  # noqa: E402
from planning.collision_map import CollisionMap  # noqa: E402
from planning.grasp import Target  # noqa: E402
from planning.release import Destination  # noqa: E402

OBS = [90, 105, 5, 5, 92]
TZ = C.TABLE_Z


def on_top(bx, by, yaw, name, level=1):
    return S.Box("block", name, (bx, by, TZ + level * C.BLOCK + C.BLOCK / 2), (C.BLOCK / 2,) * 3, yaw)


class StackMapTest(unittest.TestCase):
    def setUp(self):
        self.m = CollisionMap.empty(table_z_m=TZ)
        self.m.add_box(S.block(0.0, 0.22, 0, "low"))
        self.m.add_box(on_top(0.0, 0.22, 5, "top"))

    def test_leaving_out_the_top_block_shows_the_one_under_it(self):
        low, top = self.m.object("low"), self.m.object("top")
        self.assertAlmostEqual(top["bottom_m"], C.BLOCK, places=4)
        self.assertEqual(self.m.support_of("top")["name"], "low")
        self.assertIsNone(self.m.support_of("low"))
        h = self.m.top_under(0.0, 0.22, 0.005)
        self.assertAlmostEqual(h, 2 * C.BLOCK, places=4)
        self.assertAlmostEqual(self.m.top_under(0.0, 0.22, 0.005, exclude=(top["id"],)), C.BLOCK, places=4)
        self.assertEqual(self.m.top_under(0.0, 0.22, 0.005, exclude=(top["id"], low["id"])), 0.0)

    def test_the_under_layer_survives_a_round_trip(self):
        m2 = CollisionMap.load(self.m.to_dict())
        self.assertTrue(np.array_equal(m2.under_labels, self.m.under_labels))
        self.assertEqual(m2.support_of("top")["name"], "low")


class VerticalPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        m = CollisionMap.empty(table_z_m=TZ)
        for n, bx in S.planned_bins().items():
            m.add_object(bx, dict(name=f"bin{n}", kind="bin", centre_m=list(bx[-1].centre[:2]), size_m=[0.084, 0.084, 0.042]))
        m.add_box(S.block(0.03, 0.26, 10, "b1"))
        m.add_box(S.block(-0.035, 0.26, 0, "b2"))
        m.add_box(S.block(0.0, 0.20, 0, "b3"))
        m.add_box(on_top(0.0, 0.20, 5, "b4"))
        cls.m, cls.w = m, planning.world(m)

    def test_stack_onto_a_block_sets_it_down_on_its_top(self):
        m, t = self.m, self.w.tuning
        b2 = m.object("b2")
        dest = Destination(b2["centre_m"][0], b2["centre_m"][1], 0.0, None, "on b2", support_z=t.table_z + C.BLOCK,
                           support=m.object_box("b2", t.table_z), support_label=b2["id"])
        pick, place = PP.plan_pick_and_place(self.w, OBS, Target.from_map(m, "b1", t.table_z), dest)
        self.assertTrue(place.ok, place.reasons[:3])
        s = place.summary
        self.assertEqual((s["into"], s["mode"]), ("stack", "set"))
        gap = (s["release"]["block_bottom_above_table_cm"] / 100) - C.BLOCK
        self.assertAlmostEqual(gap, C.R["stack_gap_m"], delta=0.0015)
        self.assertLessEqual(s["block_level_deg"], C.R["max_block_tilt_deg"]["stack"])

    def test_pick_the_top_of_a_stack(self):
        t = self.w.tuning
        target = Target.from_map(self.m, "b4", t.table_z)
        self.assertEqual(target.support.name, "b3")
        pick = next(PP.pick_plans(self.w, OBS, target, 30, None, []), None)
        self.assertIsNotNone(pick)
        self.assertGreater(pick.summary["pad_above_table_cm"], 100 * C.BLOCK)
        # the block's centre in the fingers: 3 mm under the pad middle (pads close 18 mm over its bottom)
        held = pick.held
        pad = GR.gg.pad_centre_link(GR.contact(C.BLOCK))
        centre = GR.held_base(pick.summary["grasp"]["grasp"], held)[0]
        self.assertAlmostEqual(centre[2], target.box(t.table_z).centre[2], delta=5e-4)
        self.assertNotAlmostEqual(held.centre[2], pad[2], delta=1e-3)

    def test_into_a_bin_set_lets_go_lower_than_a_drop(self):
        t = self.w.tuning
        out = {}
        for mode in ("set", "drop"):
            bx = S.planned_bins()["2"]
            walls = [b for b in bx if "wall" in b.name]
            cx, cy = sum(b.centre[0] for b in walls) / 4, sum(b.centre[1] for b in walls) / 4
            dest = Destination(cx, cy, None, S.bin_rim_z(), "bin2", support_z=t.table_z + 0.002 if mode == "set" else None,
                               mode=mode)
            _, place = PP.plan_pick_and_place(self.w, OBS, Target.from_map(self.m, "b1", t.table_z), dest)
            self.assertTrue(place.ok, (mode, place.reasons[:3]))
            out[mode] = place.summary["release"]["fall_cm"]
        self.assertLess(out["set"], out["drop"] - 2.0)

    def test_release_openings_go_from_wide_to_just_wider_than_the_block(self):
        ops = GR.release_openings(C.BLOCK)
        gaps = [o.gap_m for o in ops]
        self.assertEqual(gaps, sorted(gaps, reverse=True))
        self.assertGreaterEqual(min(gaps), C.BLOCK + C.R["min_extra_gap_m"] - 1e-9)
        self.assertGreater(len(ops), len(GR.OPENINGS))


class CorrectionTest(unittest.TestCase):
    def test_reach_takes_the_held_block_back_to_its_target(self):
        q = [80.0, 70.0, 10.0, 50.0, 270.0]
        held = GR.held_from_grasp(q, C.BLOCK)
        target = CR.centre(q, held)
        rb = [80.6, 69.2, 10.5, 50.4, 270.0]               # stopped short
        r = CR.reach(target, rb, held)
        self.assertTrue(r["applied"], r)
        self.assertLess(np.linalg.norm(CR.centre(r["command"], held) - target), 5e-4)
        self.assertFalse(CR.reach(target, q, held)["applied"])                 # already there
        far = [90.0, 60.0, 10.0, 50.0, 270.0]
        self.assertFalse(CR.reach(target, far, held)["applied"])               # not a servo error


if __name__ == "__main__":
    unittest.main()
