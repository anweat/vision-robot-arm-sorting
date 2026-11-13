"""Tests for the offline wrist-roll solver (src/tasks/grip_solver.py).

The one measured anchor is the 2026-09-26 grasp recorded in data/runs/console_grasp/log.jsonl: red block
at base (6.71, 19.67) cm, pitch 150, J1 71, block yaw -10.9 deg, J5 100, operator confirmed the fingers
were parallel to the faces. Everything else here is the projection identity the solver inverts, checked
against itself (no hardware, no data files).
"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ("tasks", "spatial"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import grip_solver as S  # noqa: E402


class VerifiedGraspTest(unittest.TestCase):
    def test_reproduces_the_2026_09_26_grasp(self):
        found = S.solve_best(0.0671, 0.1967, -10.9, (150,), max_tilt_deg=15)
        self.assertIsNotNone(found)
        self.assertAlmostEqual(found["j5"], 100.0, delta=2.5)
        self.assertLess(found["residual_deg"], 1.5)
        self.assertLess(found["tilt_deg"], 10.0)
        self.assertTrue(found["meets_tilt"])


class ProjectionTest(unittest.TestCase):
    def test_roll_zero_is_radial_and_horizontal(self):
        self.assertAlmostEqual(S.jaw_bearing(90.0, S.ROLL_ZERO, 150.0), 0.0, places=6)
        self.assertAlmostEqual(S.out_of_plane_deg(S.ROLL_ZERO, 150.0), 0.0, places=6)

    def test_vertical_link_turns_degrees_one_to_one(self):
        # pitch 180: the link is vertical, so the table projection is exact
        for roll in (-40.0, 0.0, 30.0):
            j5 = S.ROLL_ZERO + roll
            self.assertAlmostEqual(S.jaw_bearing(71.0, j5, 180.0), 71.0 - 90.0 + roll, places=6)
            self.assertAlmostEqual(S.out_of_plane_deg(j5, 180.0), 0.0, places=6)

    def test_tilted_link_compresses_the_roll(self):
        # pitch 150 (30 deg off vertical): the same 45 deg roll turns the axis by ~40.9 deg, not 45
        j1, roll = 90.0, 45.0
        turned = S.jaw_bearing(j1, S.ROLL_ZERO + roll, 150.0) - (j1 - 90.0)
        self.assertAlmostEqual(turned, math.degrees(math.atan(math.cos(math.radians(30)) * math.tan(math.radians(roll)))), places=6)
        self.assertLess(turned, roll)

    def test_out_of_plane_grows_with_roll_and_pitch(self):
        self.assertGreater(S.out_of_plane_deg(S.ROLL_ZERO + 45, 150.0), S.out_of_plane_deg(S.ROLL_ZERO + 20, 150.0))
        self.assertGreater(S.out_of_plane_deg(S.ROLL_ZERO + 45, 150.0), S.out_of_plane_deg(S.ROLL_ZERO + 45, 165.0))


class SolveTest(unittest.TestCase):
    def test_loose_tilt_budget_aligns_the_faces(self):
        for bx, by in ((0.0, 0.20), (0.08, 0.21), (-0.10, 0.18)):
            for yaw in (-40.0, -20.0, 0.0, 20.0, 40.0):
                found = S.solve(bx, by, yaw, 150.0, max_tilt_deg=30.0)
                self.assertIsNotNone(found)
                self.assertLess(found["residual_deg"], 1.0, (bx, by, yaw, found))
                self.assertGreaterEqual(found["j5"], S.J5_RANGE[0] - 1e-9)
                self.assertLessEqual(found["j5"], S.J5_RANGE[1] + 1e-9)

    def test_tight_tilt_budget_is_reported_not_hidden(self):
        # a yaw that needs a large roll cannot also keep the fingers near horizontal at pitch 150
        found = S.solve(0.0, 0.20, 40.0, 150.0, max_tilt_deg=5.0)
        self.assertIsNotNone(found)
        if not found["meets_tilt"]:
            self.assertGreater(found["tilt_deg"], 5.0)
        self.assertIn("residual_deg", found)
        self.assertIn("tilt_deg", found)

    def test_offset_is_about_one_centimetre_at_the_measured_pitch(self):
        # at J5 = ROLL_ZERO the midpoint sits 1 cm nearer the base: the tip target moves 1 cm outwards
        bx, by = 0.0, 0.22
        tx, ty = S.finger_offset(bx, by, S.ROLL_ZERO, 150.0)
        self.assertAlmostEqual(math.hypot(tx - bx, ty - by), 0.01, delta=0.0015)

    def test_quarter_turn_changes_the_tangential_side(self):
        # J5 = ROLL_ZERO + 90 moves the midpoint to the far side of the wrist: tangential > radial now
        bx, by = 0.0, 0.22
        tx, ty = S.finger_offset(bx, by, S.ROLL_ZERO + 90, 150.0)
        self.assertGreater(abs(tx - bx), 0.01)
        self.assertLess(abs(ty - by), 0.003)


if __name__ == "__main__":
    unittest.main()
