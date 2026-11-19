import math
import unittest

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dev" / "spatial"))
from multiview_demo import (
    Camera, Intrinsics, block_top_corners, estimate_block, intersect_height,
    triangulate,
)


class MultiViewTests(unittest.TestCase):
    def setUp(self):
        self.intrinsics = Intrinsics.from_horizontal_fov(640, 480, 110)
        self.point = (0.055, 0.21, 0.025)
        self.cameras = [
            Camera.look_at((-0.06, -0.04, 0.32), (0.0, 0.15, 0.0)),
            Camera.look_at((0.0, -0.04, 0.32), (0.0, 0.15, 0.0)),
            Camera.look_at((0.06, -0.04, 0.32), (0.0, 0.15, 0.0)),
        ]

    def test_two_and_three_views_recover_position(self):
        for count in (2, 3):
            observations = [(camera, camera.project(self.point, self.intrinsics))
                            for camera in self.cameras[:count]]
            reconstructed = triangulate(observations, self.intrinsics)
            self.assertLess(math.dist(reconstructed, self.point), 1e-9)

    def test_identical_view_rejected(self):
        camera = self.cameras[0]
        pixel = camera.project(self.point, self.intrinsics)
        with self.assertRaisesRegex(ValueError, "no translation baseline"):
            triangulate([(camera, pixel), (camera, pixel)], self.intrinsics)

    def test_rotation_without_translation_cannot_recover_depth(self):
        center = self.cameras[0].center
        rotated = Camera.look_at(center, self.point)
        other = Camera.look_at(center, (0.0, 0.15, 0.0))
        observations = [(camera, camera.project(self.point, self.intrinsics))
                        for camera in (rotated, other)]
        with self.assertRaisesRegex(ValueError, "no translation baseline"):
            triangulate(observations, self.intrinsics)

    def test_corner_order_provides_yaw(self):
        center, yaw = estimate_block(block_top_corners(self.point, 0.04, 25))
        self.assertLess(math.dist(center, self.point), 1e-9)
        self.assertAlmostEqual(yaw, 25)

    def test_one_view_recovers_position_when_top_height_is_known(self):
        camera = self.cameras[0]
        pixel = camera.project(self.point, self.intrinsics)
        reconstructed = intersect_height(camera, pixel, self.intrinsics,
                                         self.point[2])
        self.assertLess(math.dist(reconstructed, self.point), 1e-9)


if __name__ == "__main__":
    unittest.main()
