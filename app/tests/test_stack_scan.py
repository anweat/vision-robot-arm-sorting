"""Stack-aware scan (src/pipeline/stack_scan.py, doc 41) on synthetic views: detector boxes made by projecting known
cubes from field-scan and height-view camera poses, no images. Checks the level a block is given, that a table block
stays on the table when the views cannot tell, the support inferred under a raised block, the in-bin floor and the
located.json fields the planner reads."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ("", "pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub) if sub else str(ROOT / "src"))

import numpy as np  # noqa: E402

import stack_scan as SS  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import TableMap, load_intrinsics  # noqa: E402

FIELD = [[100, 125, 0, 0], [70, 125, 0, 0], [80, 120, 0, 0], [90, 105, 5, 0]]
HEIGHT = [[92, 60, 65, -10], [80, 155, -5, -5], [100, 150, -5, -5]]
CATS = ("kitchen", "recyclable", "hazardous", "other")


def make_views(blocks, poses, w=640, h=480):
    """Views whose boxes are the projected cubes (block: (map xy m, level, yaw deg, category)), the nearer block in
    front: a block whose box is mostly covered by a nearer one's gets the box of its visible rows (as a stack's
    lower block does)."""
    cam, table = ArmCamera(), TableMap()
    k, _ = load_intrinsics()
    views = []
    for i, q in enumerate(poses):
        pose = cam.pose_in_map(q + [92], table)
        v = SS.View(i, np.zeros((h, w, 3), np.uint8), pose, k, np.zeros(5), dict(image=f"v{i}", purpose="test"))
        drawn = []
        for xy, lv, yaw, cat in blocks:
            corners = SS.cube_corners(np.array(xy), SS.z_centre(lv), np.radians([yaw]))[0]
            box, whole, vis = v.boxes_of(corners)
            if not vis:
                continue
            depth = float(np.linalg.norm(pose.centre - [xy[0], xy[1], SS.z_centre(lv)]))
            drawn.append((depth, box, cat, lv))
        for depth, box, cat, lv in drawn:
            nearer = [b for d, b, _, l in drawn if d < depth - 1e-6 and l > lv]
            b = box.copy()
            for n in nearer:                   # a block on top hides this one's top rows
                if n[0] < b[2] and n[2] > b[0] and n[3] > b[1]:
                    b[1] = max(b[1], min(n[3], b[3] - 12))
            cut = b[0] <= 2 or b[1] <= 2 or b[2] >= w - 3 or b[3] >= h - 3
            v.boxes.append(SS.Box(i, b, cat, 0.9, bool(cut)))
        views.append(v)
    return views


def solve(blocks, poses, bins=None):
    sol = SS.Solver(make_views(blocks, poses), bins)
    return sol, sol.select(sol.hypotheses())


class LevelTest(unittest.TestCase):
    def test_two_stack_with_height_views(self):
        blocks = [((-0.04, 0.15), 0, 10, "kitchen"), ((-0.04, 0.15), 1, 25, "recyclable"),
                  ((0.06, 0.17), 0, 0, "hazardous")]
        _, chosen = solve(blocks, FIELD + HEIGHT)
        top = [h for h in chosen if np.linalg.norm(h.xy - [-0.04, 0.15]) < 0.01 and h.level == 1]
        self.assertTrue(top, [(h.level, h.xy) for h in chosen])
        table = [h for h in chosen if np.linalg.norm(h.xy - [0.06, 0.17]) < 0.01]
        self.assertEqual([h.level for h in table], [0])

    def test_table_blocks_stay_on_the_table(self):
        blocks = [((x, y), 0, yaw, CATS[i % 4]) for i, (x, y, yaw) in
                  enumerate([(-0.08, 0.12, 20), (-0.01, 0.15, -30), (0.06, 0.11, 5), (0.09, 0.17, 40)])]
        for poses in (FIELD, FIELD + HEIGHT):
            _, chosen = solve(blocks, poses)
            self.assertTrue(all(h.level == 0 for h in chosen), [(h.level, h.xy.round(3)) for h in chosen])

    def test_raised_needs_a_whole_box(self):
        sol, _ = solve([((0.0, 0.15), 1, 0, "other")], FIELD + HEIGHT)
        h = SS.Hyp(1, np.array([0.0, 0.15]), 0.0)
        for v in sol.views:
            for b in v.boxes:
                b.cut = True
        self.assertLess(sol.evaluate(h).score, 0)


class SupportTest(unittest.TestCase):
    def test_inferred_support_and_located_fields(self):
        sol, chosen = solve([((0.0, 0.16), 1, 0, "other")], FIELD + HEIGHT)   # nothing drawn under it
        top = [h for h in chosen if h.level == 1]
        if not top:                     # no box below it at all may keep it on the table: acceptable, not a phantom
            self.assertTrue(all(h.level == 0 for h in chosen))
            return
        s = sol.find_support(top[0], top, set())
        self.assertEqual(s.level, 0)
        self.assertLess(np.linalg.norm(s.xy - top[0].xy), 0.01)

    def test_floor_in_bin(self):
        bins = [dict(name="bin2", xy=np.array([0.094, 0.0725]), yaw=0.0, inner=0.08)]
        sol = SS.Solver([], bins)
        self.assertEqual(sol.floor_at((0.094, 0.0725)), (SS.BIN_FLOOR_M, "bin2"))
        self.assertEqual(sol.floor_at((0.094 - 0.037, 0.0725)), (0.0, None))   # against the wall: not on the floor
        self.assertEqual(sol.floor_at((0.0, 0.15)), (0.0, None))

    def test_located_format(self):
        scene = dict(registration=dict(map_origin_base_cm=[0.07, 6.75, -1.42], spread_cm=[0.1, 0.1, 0.1]), blocks=[
            dict(name="s1", level=0, bottom_cm=0.0, map_cm=[0.0, 15.0], yaw_deg=5.0, category=None, score=0.0,
                 support=None, inferred=True, covered=True, approx=False, in_bin=None, views=0),
            dict(name="s2", level=1, bottom_cm=3.0, map_cm=[0.0, 15.0], yaw_deg=5.0, category="kitchen", score=3.0,
                 support="s1", inferred=False, covered=False, approx=False, in_bin=None, views=4)])
        loc = SS.located(scene, Path("."))
        b1, b2 = loc["objects"]
        self.assertEqual((b1["bottom_m"], b2["bottom_m"]), (0.0, 0.03))
        self.assertEqual(b2["support"], "b1")
        self.assertIsNone(b1["class"])
        self.assertEqual(b1["sigma_yaw_deg"], SS.COVERED_SIGMA_YAW_DEG)
        self.assertAlmostEqual(b2["centre_m"][1], 0.2175, places=4)
        for key in ("id", "name", "kind", "class", "centre_m", "yaw_deg", "size_m", "sigma_xy_m", "sigma_yaw_deg"):
            self.assertIn(key, b2)


class PlanTest(unittest.TestCase):
    def test_plan_views_separate_the_levels(self):
        table = TableMap()
        base_T_map = np.eye(4)
        base_T_map[:3, 3] = table.offset
        cols = [dict(levels={0: np.array([0.0, 0.17]), 1: np.array([0.0, 0.15]), 2: np.array([0.0, 0.13])}, ids=set())]
        plan = SS.plan_views(cols, base_T_map, [90, 105, 5, 5, 92, 30], max_views=2)
        self.assertTrue(plan)
        self.assertGreaterEqual(max(p["gain"][0] for p in plan), 0.2)
        for p in plan:
            self.assertTrue(SS.path_ok([90, 105, 5, 5], p["pose"]))


if __name__ == "__main__":
    unittest.main()
