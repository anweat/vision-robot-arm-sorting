"""config/platform_model.json (Blender scene, rounded-square lid) against the keep-out and the hardware facts."""

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

import platform_model as pm  # noqa: E402

CFG = pm.load()
KEEP_OUT = json.loads((ROOT / "config" / "arm_limits.json").read_text(encoding="utf-8"))["keep_out"]


class PlatformModelTest(unittest.TestCase):
    def test_shared_numbers_match_the_keep_out(self):
        self.assertEqual(CFG["top_z_m"], KEEP_OUT["platform_min_z_m"])
        self.assertEqual(CFG["outline"]["front_y_m"], KEEP_OUT["platform_radius_m"])
        self.assertEqual(CFG["table_z_m"], KEEP_OUT["table_surface_z_m"])

    def test_front_edge_straight_ahead_is_8_cm(self):
        # below the rolled edge the lid reaches its full outline
        z = CFG["top_z_m"] - CFG["lid"]["edge_radius_m"] - 0.001
        self.assertAlmostEqual(pm.edge_distance(90, CFG, z=z), 0.08, delta=0.001)

    def test_it_is_a_square_not_a_disk(self):
        # sideways the lid reaches further than the 8 cm keep-out cylinder: that is what limits J1
        z = CFG["top_z_m"] - CFG["lid"]["edge_radius_m"] - 0.001
        self.assertGreater(pm.edge_distance(20, CFG, z=z), 0.10)
        self.assertGreater(pm.edge_distance(160, CFG, z=z), 0.10)

    def test_rolled_edge_lowers_the_lid_near_its_rim(self):
        top, e = CFG["top_z_m"], CFG["lid"]["edge_radius_m"]
        self.assertGreater(pm.depth(0.0, 0.0, top - 0.001, CFG), 0)        # flat top
        self.assertLess(pm.depth(0.0, 0.0795, top - 0.001, CFG), 0)        # just inside the outline, top level
        self.assertGreater(pm.depth(0.0, 0.0795, top - e - 0.001, CFG), 0)  # same xy, below the roll

    def test_near_view_49_passes_the_front_right_corner(self):
        # [49,105,5,0] ran 32 times without contact; its wrist passes (8.0, 6.6, 7.2) cm
        self.assertLess(pm.depth(0.0797, 0.0656, 0.0722, CFG), 0.003)

    def test_flange_clears_the_servo1_housing(self):
        # base_link.STL: r 4.0 cm flange at z 7.0 cm comes up through the lid
        self.assertGreaterEqual(CFG["hole"]["flange_inner_radius_m"], 0.040)
        self.assertGreater(CFG["hole"]["radius_m"], CFG["hole"]["flange_outer_radius_m"])

    def test_outline_polygon_is_closed_and_consistent(self):
        poly = pm.outline_polygon(CFG)
        for x, y in poly:
            self.assertAlmostEqual(pm.inside_distance_2d(x, y, CFG), 0.0, delta=1e-9)
        inset = pm.outline_polygon(CFG, inset=0.01)
        self.assertEqual(len(poly), len(inset))     # the Blender loft needs equal rings
        for x, y in inset:
            self.assertAlmostEqual(pm.inside_distance_2d(x, y, CFG), 0.01, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
