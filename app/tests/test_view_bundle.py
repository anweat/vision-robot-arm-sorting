"""spatial/view_bundle.py: the map and the scan views registered together from the codes (doc 34 §9)."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

import numpy as np  # noqa: E402
import cv2  # noqa: E402

import view_bundle as VB  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import TableMap, load_intrinsics  # noqa: E402

POSES = [[100, 125, 0, 0], [70, 125, 0, 0], [110, 105, 15, 0], [80, 120, 0, 0], [62, 106, 5, 0], [90, 105, 5, 0],
         [118, 106, 5, 0]]


def project(cam_T_map, obj, k, dist):
    rv, _ = cv2.Rodrigues(cam_T_map[:3, :3])
    p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rv, cam_T_map[:3, 3], k, dist)
    return p.reshape(-1, 2)


class BundleTest(unittest.TestCase):
    def setUp(self):
        self.table = TableMap()
        self.k, self.dist = load_intrinsics()
        cam = ArmCamera()
        rng = np.random.default_rng(3)
        self.base_T_map = np.eye(4)
        self.base_T_map[:3, 3] = self.table.offset
        self.fk, self.true, self.markers = [], [], []
        for p in POSES:
            fk = cam.base_T_cam(p + [92])
            d = np.concatenate([np.radians(rng.normal(0, 1.2, 3)), rng.normal(0, 0.004, 3)])
            true = VB._correct(fk, d)
            cam_T_map = np.linalg.inv(true) @ self.base_T_map
            seen = {}
            for mid, corners in self.table.layout.items():
                obj = np.column_stack([corners, np.zeros(4)])
                if (cam_T_map[:3, :3] @ obj.T + cam_T_map[:3, 3:4])[2].min() <= 0.02:
                    continue
                uv = project(cam_T_map, obj, self.k, self.dist)
                if (uv > 20).all() and (uv[:, 0] < 620).all() and (uv[:, 1] < 460).all():
                    seen[mid] = uv + rng.normal(0, 0.3, uv.shape)
            self.fk.append(fk)
            self.true.append(true)
            self.markers.append(seen)

    def table_err_mm(self, pose, true):
        """How far a table point in the middle of the view lands, through the solved pose vs the true one."""
        cam_T_map = np.linalg.inv(true) @ self.base_T_map
        centre = -cam_T_map[:3, :3].T @ cam_T_map[:3, 3]
        ray = cam_T_map[:3, :3].T @ np.array([0, 0, 1.0])
        p = centre + ray * (-centre[2] / ray[2])
        uv = project(cam_T_map, p[None], self.k, self.dist)[0]
        return float(np.linalg.norm(pose.pixel_to_plane(uv, self.k, self.dist)[:2] - p[:2])) * 1000

    def test_views_with_codes_land_within_a_millimetre_and_a_single_code_is_enough(self):
        markers = [dict(m) for m in self.markers]
        one = next(i for i, m in enumerate(markers) if len(m) >= 3)
        markers[one] = dict(list(markers[one].items())[:1])             # this view keeps a single code
        res = VB.solve(self.fk, markers, self.table, self.k, self.dist, self.base_T_map)
        for i, m in enumerate(markers):
            if m:
                self.assertLess(self.table_err_mm(res.poses[i], self.true[i]), 1.5 if i == one else 1.0, i)
        fk_pose = VB.CameraPose(*(lambda m: (m[:3, :3], m[:3, 3]))(np.linalg.inv(self.fk[one]) @ self.base_T_map))
        self.assertGreater(self.table_err_mm(fk_pose, self.true[one]), 2 * self.table_err_mm(res.poses[one], self.true[one]))

    def test_views_in_the_map_do_not_depend_on_the_start_and_a_view_without_codes_is_left_alone(self):
        # what the bundle pins down is each view in the map; its placement trades off against the shared bias
        # (scene.py keeps the median placement for the base frame), so a start 1 cm off changes nothing here
        markers = [dict(m) for m in self.markers]
        markers[0] = {}
        start = self.base_T_map.copy()
        start[:2, 3] += [0.008, -0.006]
        res = VB.solve(self.fk, markers, self.table, self.k, self.dist, start)
        for i, m in enumerate(markers):
            if m:       # where the view puts the table, which is what the top view is made of
                self.assertLess(self.table_err_mm(res.poses[i], self.true[i]), 0.5, i)
        self.assertEqual(res.used[0], [])
        self.assertTrue(math.isnan(res.reprojection_px[0]))
        self.assertLess(res.stats["reprojection_px_median"], 1.0)


if __name__ == "__main__":
    unittest.main()
