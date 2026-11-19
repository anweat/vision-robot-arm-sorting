"""planning.collision_map: the external scene format and its clearance queries (numpy)."""

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import planning  # noqa: E402
from planning import scene as S  # noqa: E402
from planning import tuning as TU  # noqa: E402
from planning.collision_map import FORMAT, CollisionMap  # noqa: E402

TZ = TU.default().table_z


class CollisionMapTest(unittest.TestCase):
    def setUp(self):
        self.m = CollisionMap.from_boxes([S.block(0.0, 0.20, 0.0, "a"), S.block(0.08, 0.20, 30.0, "b")], TZ)

    def test_objects_and_labels(self):
        self.assertEqual([o["name"] for o in self.m.objects], ["a", "b"])
        a = self.m.object("a")
        self.assertEqual(int((self.m.labels == a["id"]).sum()) > 0, True)
        self.assertAlmostEqual(float(self.m.height.max()), 0.03, places=6)

    def test_json_and_npz_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            for name in ("m.json", "m.npz"):
                p = self.m.save(Path(d) / name)
                back = CollisionMap.load(p)
                self.assertEqual(back.shape, self.m.shape)
                self.assertTrue(np.array_equal(back.labels, self.m.labels))
                self.assertTrue(np.allclose(back.height, self.m.height))
                self.assertEqual(back.objects, self.m.objects)
            raw = json.loads((Path(d) / "m.json").read_text())
            self.assertEqual(raw["format"], FORMAT)

    def test_refuses_other_frames_and_formats(self):
        d = self.m.to_dict()
        with self.assertRaises(ValueError):
            CollisionMap.load(dict(d, frame="map"))
        with self.assertRaises(ValueError):
            CollisionMap.load(dict(d, format="other/1"))

    def test_clearance_is_the_grown_map(self):
        top = TZ + 0.03
        # straight above the block: the vertical gap less the margin
        c, name = self.m.clearance(np.array([[0.0, 0.20, top + 0.02]]), TZ, 0.005)
        self.assertAlmostEqual(c, 0.015, delta=1e-9)
        self.assertEqual(name, "a")
        # beside it at half height: clear only beyond the margin (+- a cell)
        edge = 0.015
        for d, clear in ((0.012, True), (0.002, False)):
            c, _ = self.m.clearance(np.array([[edge + d, 0.20, TZ + 0.015]]), TZ, 0.008)
            self.assertEqual(c >= 0, clear, (d, c))

    def test_excluded_labels_and_unknown_cells(self):
        a = self.m.object("a")["id"]
        p = np.array([[0.0, 0.20, TZ + 0.01]])
        self.assertLess(self.m.clearance(p, TZ, 0.0)[0], 0)
        self.assertEqual(self.m.clearance(p, TZ, 0.0, exclude=[a])[0], math.inf)
        m = CollisionMap.empty(table_z_m=TZ)
        i, j, _ = m.cells_of(p)
        m.height[i[0], j[0]] = np.nan
        self.assertEqual(m.clearance(p, TZ, 0.0)[0], math.inf)                      # unknown = free
        self.assertLess(m.clearance(p, TZ, 0.0, unknown_top=0.05)[0], 0)           # unknown = 5 cm tall

    def test_bins_are_one_object(self):
        m = CollisionMap.empty(table_z_m=TZ)
        oid = m.add_object(S.bin_boxes((0.095, 0.14), 0.0, "bin2"), dict(name="bin2", kind="bin", centre_m=[0.095, 0.14]))
        self.assertEqual(len(m.objects), 1)
        inside = np.array([[0.095, 0.14, TZ + 0.02]])                           # inside the bin, above its plate
        self.assertGreater(m.clearance(inside, TZ, 0.004)[0], 0)
        wall = np.array([[0.095 + 0.041, 0.14, TZ + 0.02]])
        c, name = m.clearance(wall, TZ, 0.004)
        self.assertLess(c, 0)
        self.assertEqual(name, "bin2")
        self.assertEqual(oid, m.objects[0]["id"])

    def test_the_platform_is_dropped_from_a_map(self):
        m = CollisionMap.from_boxes([S.box_on_table(0.0, 0.0, 0.05, 0.05, 0.10, 0.0, "arm seen by a camera")], TZ)
        w = planning.world(m)
        self.assertEqual(float(np.nanmax(w.cmap.height)), 0.0)


if __name__ == "__main__":
    unittest.main()
