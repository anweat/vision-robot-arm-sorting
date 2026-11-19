"""Synthetic check of map registration: render the printed map as the wrist camera would
see it from a predicted pose, then recover that pose from the rendered image.

Needs OpenCV with aruco:
  uv run --with "opencv-python>=4.8,<5" --with numpy python -m unittest tests/test_map_registration.py
"""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))
sys.path.insert(0, str(ROOT / "src" / "vision"))

from camera_model import ArmCamera  # noqa: E402
from map_registration import TableMap, detect_markers, load_intrinsics, solve_camera_pose  # noqa: E402

PX_PER_M = 4000  # 4 px/mm page raster
PAGE_W, PAGE_H = 0.297, 0.210


def render_page(table: TableMap) -> np.ndarray:
    page = np.full((round(PAGE_H * PX_PER_M), round(PAGE_W * PX_PER_M)), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side = round(table.side * PX_PER_M)
    for mid, corners in table.layout.items():
        u0 = round((corners[0][0] + PAGE_W / 2) * PX_PER_M)
        v0 = round((PAGE_H - corners[0][1]) * PX_PER_M)
        page[v0:v0 + side, u0:u0 + side] = cv2.aruco.generateImageMarker(dictionary, mid, side, borderBits=1)
    return page


def render_view(table: TableMap, pose, k: np.ndarray) -> np.ndarray:
    page_to_map = np.array([[1 / PX_PER_M, 0, -PAGE_W / 2], [0, -1 / PX_PER_M, PAGE_H], [0, 0, 1]])
    map_to_img = k @ np.column_stack([pose.R[:, 0], pose.R[:, 1], pose.t])
    view = cv2.warpPerspective(render_page(table), map_to_img @ page_to_map, (640, 480),
                               flags=cv2.INTER_AREA, borderValue=20)
    return cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)


class MapRegistrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.table = TableMap()
        cls.k, _ = load_intrinsics()
        cls.dist = np.zeros(5)
        cls.camera = ArmCamera()

    def check_pose(self, servos, min_markers):
        truth = self.camera.pose_in_map(servos, self.table)
        image = render_view(self.table, truth, self.k)
        markers = detect_markers(image, self.table.layout)
        self.assertGreaterEqual(len(markers), min_markers)
        pose = solve_camera_pose(markers, self.table, self.k, self.dist, prior_centre=truth.centre)
        self.assertIsNotNone(pose)
        self.assertLess(np.linalg.norm(pose.centre - truth.centre), 0.003)
        # a table point at the block-top height must come back where it was
        point = truth.pixel_to_plane((320, 300), self.k, self.dist, z=0.03)
        uv = pose.project(point.reshape(1, 3), self.k, self.dist)[0]
        back = pose.pixel_to_plane(uv, self.k, self.dist, z=0.03)
        self.assertLess(np.linalg.norm(back - point), 0.002)
        return pose

    def test_far_scan_row_views(self):
        for s1 in (75, 90, 105):
            with self.subTest(s1=s1):
                self.check_pose([s1, 105, 5, 5, 95], min_markers=2)

    def test_near_view_with_one_marker_still_resolves(self):
        self.check_pose([90, 85, 5, 5, 95], min_markers=1)

    def test_pixel_to_plane_is_on_the_plane(self):
        pose = self.camera.pose_in_map([90, 105, 5, 5, 95], self.table)
        p = pose.pixel_to_plane((100, 400), self.k, self.dist, z=0.03)
        self.assertAlmostEqual(p[2], 0.03, places=9)


class MapDifferenceTest(unittest.TestCase):
    """A textured grey 3 cm patch placed on the rendered map must be found near its true position."""

    def test_object_on_map_is_found(self):
        import block_detector as bd
        import map_difference as md

        table, (k, _), cam = TableMap(), load_intrinsics(), ArmCamera()
        dist = np.zeros(5)
        pose = cam.pose_in_map([90, 105, 5, 5, 95], table)
        image = render_view(table, pose, k)
        centre = pose.pixel_to_plane((320, 260), k, dist, z=0.0)
        square = np.array([[centre[0] + dx, centre[1] + dy, 0.0] for dx, dy in
                           ((-0.015, 0.015), (0.015, 0.015), (0.015, -0.015), (-0.015, -0.015))])
        poly = pose.project(square, k, dist).astype(np.int32)
        cv2.fillConvexPoly(image, poly, (60, 90, 200))
        cv2.line(image, tuple(poly[0]), tuple(poly[2]), (20, 20, 20), 3)
        markers = detect_markers(image, table.layout)
        blobs = md.detect_objects(image, pose, k, dist, table.layout, markers)
        found = []
        for b in blobs:
            bd.locate(b, pose, k, dist, height=0.0)  # flat patch: its "top" is the table
            if b.size_m >= 0.02:
                found.append(np.array(b.map_xy))
        self.assertEqual(len(found), 1)
        self.assertLess(np.linalg.norm(found[0] - centre[:2]), 0.005)


if __name__ == "__main__":
    unittest.main()
