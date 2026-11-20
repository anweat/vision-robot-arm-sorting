import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402


class ForwardTest(unittest.TestCase):
    def test_upright_is_straight_up(self):
        tip = kin.forward([90, 90, 90, 90])["tip"]
        expected = kin.BASE_TO_SHOULDER + kin.UPPER_ARM + kin.FOREARM + kin.WRIST_TO_SERVO5 + kin.GRIPPER
        self.assertAlmostEqual(tip[0], 0, places=4)
        self.assertAlmostEqual(tip[1], 0, places=4)
        self.assertAlmostEqual(tip[2], expected, places=4)

    def test_directions_match_2026_09_24_observations(self):
        upright = kin.forward([90, 90, 90, 90])["tip"]
        self.assertLess(kin.forward([90, 100, 90, 90])["tip"][1], upright[1])  # servo 2 up -> leans back
        self.assertGreater(kin.forward([90, 90, 80, 90])["tip"][1], upright[1])  # servo 3 down -> folds forward

    def test_servo1_turns_about_z(self):
        a = kin.forward([60, 100, 40, 40])["tip"]
        b = kin.forward([90, 100, 40, 40])["tip"]
        self.assertAlmostEqual((a[0] ** 2 + a[1] ** 2) ** 0.5, b[1], places=3)
        self.assertAlmostEqual(a[2], b[2], places=4)


class Servo5PoseTest(unittest.TestCase):
    def test_rotation_is_orthonormal_and_z_follows_the_link(self):
        servos = [84, 108, 5, 8, 105]
        pose = kin.servo5_pose(servos)
        cols = [[pose[r][c] for r in range(3)] for c in range(3)]
        for i in range(3):
            for j in range(3):
                dot = sum(a * b for a, b in zip(cols[i], cols[j]))
                self.assertAlmostEqual(dot, 1.0 if i == j else 0.0, places=9)
        fk = kin.forward(servos[:4])
        link = [t - s for t, s in zip(fk["tip"], fk["servo5"])]
        norm = sum(v * v for v in link) ** 0.5
        for a, b in zip(cols[2], link):
            self.assertAlmostEqual(a, b / norm, delta=2e-3)  # forward() rounds to 0.1 mm
        self.assertEqual([round(pose[r][3], 4) for r in range(3)], fk["servo5"])


class InverseTest(unittest.TestCase):
    def test_round_trip(self):
        for servos in ([90, 53, 30, 36], [64, 23, 68, 56], [134, 65, 20, 29], [90, 125, 5, 5]):
            fk = kin.forward(servos)
            back = kin.inverse(*fk["tip"], fk["tip_pitch_deg"][0])
            for want, got in zip(servos, back):
                self.assertAlmostEqual(want, got, delta=0.3)

    def test_out_of_reach_returns_none(self):
        self.assertIsNone(kin.inverse(0, 0.6, 0.0, 180))

    def test_search_respects_limits(self):
        limits = {1: (20, 160), 2: (5, 175), 3: (5, 175), 4: (5, 175)}
        result = kin.inverse_within(0.0, 0.22, 0.02, limits)
        self.assertIsNotNone(result)
        for joint, angle in zip((1, 2, 3, 4), result["servos"]):
            self.assertTrue(limits[joint][0] <= angle <= limits[joint][1])


if __name__ == "__main__":
    unittest.main()
