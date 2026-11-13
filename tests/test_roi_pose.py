"""Tests for the classical ROI pose solver (src/vision/roi_pose.py).

The synthetic frames are drawn from the same pinhole model the solver inverts: a 3 cm cube at a known
place and yaw on the table plane, its silhouette dark and its top face light, blurred and noisy like the
sensor. No data/ assets are needed, so these run on a fresh clone.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for sub in ("spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

try:
    import cv2
    import roi_pose as R
    from map_registration import CameraPose
except ImportError:  # pragma: no cover
    cv2 = None


def camera_pose(centre=(0.0, -0.24, 0.32), target=(0.015, 0.10, 0.015)):
    """A CameraPose looking at `target` from `centre`, map -> camera (cv2 axes: x right, y down, z away)."""
    centre, target = np.asarray(centre, float), np.asarray(target, float)
    z = target - centre
    z /= np.linalg.norm(z)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    rotation = np.stack([x, y, z])
    return CameraPose(rotation, -rotation @ centre)


def intrinsics():
    return np.array([[977.383, 0.0, 320.0], [0.0, 977.383, 240.0], [0.0, 0.0, 1.0]]), np.zeros(5)


def draw_cube(pose, k, x, y, yaw, size=(480, 640)):
    """A synthetic wrist frame: dark cube silhouette and a light top face, blurred + noisy."""
    image = np.full((size[0], size[1], 3), 205, np.uint8)      # light paper
    corners = R.cube_corners_uv(pose, k, x, y, yaw)
    hull = cv2.convexHull(corners.astype(np.float32)).reshape(-1, 2)
    cv2.fillPoly(image, [hull.astype(np.int32)], (48, 44, 160))                 # cube sides: dark red
    top = R.top_face_uv(pose, k, x, y, yaw).astype(np.int32)
    cv2.fillPoly(image, [top], (232, 228, 208))                # top face: sticker white
    image = cv2.GaussianBlur(image, (0, 0), 1.1)
    noise = np.random.default_rng(0).normal(0, 2.5, image.shape)
    return np.clip(image + noise, 0, 255).astype(np.uint8), np.array([hull[:, 0].min(), hull[:, 1].min(),
                                                                     hull[:, 0].max(), hull[:, 1].max()])


def truth_yaw(corners):
    """Cube yaw from the top-face edge, folded to [-45, 45)."""
    edge = corners[1] - corners[0]
    return (math.degrees(math.atan2(edge[1], edge[0])) + 45) % 90 - 45


@unittest.skipIf(cv2 is None, "OpenCV not installed")
class SolveSyntheticCubeTest(unittest.TestCase):
    def setUp(self):
        self.k, self.dist = intrinsics()
        self.pose = camera_pose()

    def check(self, x, y, yaw_deg, tolerance_cm=0.5, tolerance_deg=3.0):
        image, box = draw_cube(self.pose, self.k, x, y, math.radians(yaw_deg))
        fix = R.solve_top_face(image, box, self.pose, self.k, self.dist)
        self.assertTrue(fix["ok"], fix.get("reason"))
        self.assertLess(math.hypot(fix["map_cm"][0] - x * 100, fix["map_cm"][1] - y * 100), tolerance_cm)
        self.assertLess(abs((fix["yaw_deg"] - yaw_deg + 45) % 90 - 45), tolerance_deg)
        return fix

    def test_recovers_centre_and_yaw_across_the_quarter_turn(self):
        for x, y, yaw in ((0.015, 0.10, 0.0), (0.015, 0.10, 30.0), (-0.02, 0.11, -35.0), (0.03, 0.09, 44.0)):
            with self.subTest(x=x, y=y, yaw=yaw):
                self.check(x, y, yaw)

    def test_quarter_turn_is_folded_into_minus_45_45(self):
        for yaw in (0.0, 90.0, 180.0):
            with self.subTest(yaw=yaw):
                fix = self.check(0.01, 0.10, yaw)
                self.assertAlmostEqual(fix["yaw_deg"], 0.0, delta=3.0)

    def test_reports_where_the_edges_were(self):
        fix = self.check(0.015, 0.10, 20.0)
        self.assertEqual(len(fix["corners_px"]), 4)
        self.assertGreater(fix["support"], 0.35)
        self.assertGreater(fix["box_iou"], 0.55)
        self.assertLess(fix["shift_cm"], 2.0)


@unittest.skipIf(cv2 is None, "OpenCV not installed")
class RejectTest(unittest.TestCase):
    def setUp(self):
        self.k, self.dist = intrinsics()
        self.pose = camera_pose()

    def test_box_touching_the_frame_edge_is_refused(self):
        image = np.full((480, 640, 3), 200, np.uint8)
        fix = R.solve_top_face(image, [0, 100, 40, 140], self.pose, self.k, self.dist)
        self.assertFalse(fix["ok"])
        self.assertIn("frame edge", fix["reason"])

    def test_empty_paper_is_refused(self):
        image = np.full((480, 640, 3), 200, np.uint8)
        image = cv2.GaussianBlur(image, (0, 0), 1.0)
        fix = R.solve_top_face(image, [280, 200, 360, 280], self.pose, self.k, self.dist)
        self.assertFalse(fix["ok"])
        self.assertIn("weak", fix["reason"])

    def test_tiny_box_is_refused(self):
        image = np.full((480, 640, 3), 200, np.uint8)
        fix = R.solve_top_face(image, [300, 220, 306, 226], self.pose, self.k, self.dist)
        self.assertFalse(fix["ok"])
        self.assertIn("small", fix["reason"])


if __name__ == "__main__":
    unittest.main()
