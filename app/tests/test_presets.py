"""stack / carry presets (config/flows/stack.yaml, carry.yaml; docs/current/35 §10.1): plan's free spots and
tower top, and the scene a place leaves behind."""
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flow import script as FS, steps as SP  # noqa: E402
from flow.session import FlowSession  # noqa: E402


def blk(name, x, y, **kw):
    return dict(name=name, id=int(name[1:]), kind="block", centre_m=[x, y], size_m=[0.03] * 3, yaw_deg=0.0,
                map_cm=[round(x * 100, 1), round((y - 0.0675) * 100, 1)], **kw)


def scene():
    return dict(objects=[blk("b1", 0.0, 0.20), blk("b2", 0.04, 0.24),
                         dict(name="bin2", id=9, kind="bin", centre_m=[0.095, 0.14], size_m=[0.094, 0.121, 0.042])])


class FreeSpotTest(unittest.TestCase):
    def test_spots_keep_clear_of_everything_and_move_the_block(self):
        sc = scene()
        spots = SP.free_spots(sc, sc["objects"][0])
        self.assertTrue(spots)
        r = 0.03 * 2 ** 0.5 / 2
        for s in spots:
            x0, y0, x1, y1 = SP.TABLE_AREA_M
            self.assertTrue(x0 - 1e-9 <= s[0] <= x1 + 1e-9 and y0 - 1e-9 <= s[1] <= y1 + 1e-9)
            self.assertGreaterEqual(((s[0] - 0.0) ** 2 + (s[1] - 0.20) ** 2) ** 0.5, SP.SPOT_MIN_MOVE_M - 1e-9)
            self.assertGreaterEqual(((s[0] - 0.04) ** 2 + (s[1] - 0.24) ** 2) ** 0.5, 2 * r + SP.SPOT_CLEAR_M - 1e-9)
        d = [round(((s[0]) ** 2 + (s[1] - 0.20) ** 2) ** 0.5, 6) for s in spots]
        self.assertEqual(d, sorted(d))                       # nearest first

    def test_a_full_page_has_no_spot(self):
        sc = dict(objects=[blk(f"b{i}", x / 100, y / 100) for i, (x, y) in
                           enumerate(((x, y) for x in range(-13, 14, 3) for y in range(9, 27, 3)), start=1)])
        self.assertEqual(SP.free_spots(sc, sc["objects"][0]), [])


class TowerTest(unittest.TestCase):
    def ctx(self, d):
        sc = scene()
        return SP.Context(FlowSession(Path(d)), scene=sc, first_scene=sc, block=dict(sc["objects"][0]))

    def test_the_first_block_is_the_base_and_the_tower_stops_at_its_height(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.ctx(d)
            out = SP.plan(c, to="stack", on="top")
            self.assertEqual((out.result, c.tower), ("base", ["b1"]))
            c.tower = ["b1", "b2", "b3"]
            self.assertEqual(SP.plan(c, to="stack", on="top", levels=3).result, "no_destination")

    def test_a_place_moves_the_block_in_the_scene(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.ctx(d)
            c.block = dict(c.scene["objects"][1], to={"stack_on": "b1"}, bin="on b1")
            c.tower = ["b1"]
            c.block_session().save("plan.json", dict(task=dict(pick="b2"), plans=[{}, dict(summary=dict(
                release=dict(block_centre_cm=[0.1, 20.05, 4.0], block_yaw_deg=2.0)))]))
            first = c.scene
            SP._scene_after_place(c)
            moved = next(o for o in c.scene["objects"] if o["name"] == "b2")
            self.assertEqual((moved["centre_m"], moved["bottom_m"], moved["yaw_deg"]), ([0.001, 0.2005], 0.03, 2.0))
            self.assertAlmostEqual(moved["map_cm"][1], 13.25, delta=0.06)          # the map coordinates follow
            self.assertEqual(c.tower, ["b1", "b2"])
            self.assertIsNot(c.scene, first)                                  # the round's first scene is kept
            self.assertEqual(next(o for o in first["objects"] if o["name"] == "b2")["centre_m"], [0.04, 0.24])
            self.assertTrue(c.map_path.exists())

    def test_a_block_put_into_a_bin_leaves_the_scene_as_before(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.ctx(d)
            c.block = dict(c.scene["objects"][0], to={"bin": "bin2", "mode": "set"})
            before = c.scene
            SP._scene_after_place(c)
            self.assertIs(c.scene, before)


class PresetScriptTest(unittest.TestCase):
    def test_the_presets_check_and_answer_the_new_results(self):
        for name in ("stack", "carry"):
            r = FS.check_text((ROOT / "config" / "flows" / f"{name}.yaml").read_text(encoding="utf-8"))
            self.assertTrue(r["ok"] and r["runnable"] and r["moves"], (name, r["errors"]))
            self.assertEqual(r["warnings"], [], name)
        self.assertIn("base", (ROOT / "config" / "flows" / "stack.yaml").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
