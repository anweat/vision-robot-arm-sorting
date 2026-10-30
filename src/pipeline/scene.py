"""From a scan (src/pipeline/scan.py) to a scene: map registration, orthophotos, cubes.

Map registration: each view that sees >= 3 markers gives base_T_map = (FK + hand-eye camera) @ (marker
PnP); the median over views is the map placement for this run (no touching needed: camera-only positions
matched the touch calibration to 1.4 mm, docs/05 2026-09-25). Each view's camera pose in the map comes
from its own markers when it sees >= 2, else from FK + hand-eye + the registered placement.

Orthophotos: the views resampled onto a horizontal plane in map coordinates at 1 mm/px, blended with
weights that fall to zero at the image border. On the plane of the block tops (z = 3 cm) a top face is
an undistorted 30 x 30 mm square wherever it was seen; the table plane (z = 0) is compared with the
rendered map to find what is not paper.

    python3 src/pipeline/scene.py data/runs/pipeline/<run>     # writes ortho_*.png and scene.json there
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import CameraPose, TableMap, detect_markers, load_intrinsics, solve_camera_pose  # noqa: E402

MM = 0.001
X_RANGE = (-0.16, 0.16)       # map metres covered by the orthophotos (the A4 map is +-0.1485 x 0..0.21)
Y_RANGE = (-0.01, 0.22)
BLOCK = 0.03


def to_matrix(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


class Scan:
    def __init__(self, run: Path):
        self.run = Path(run)
        self.views = json.loads((self.run / "views.json").read_text())
        self.table = TableMap()
        self.k, self.dist = load_intrinsics()
        self.cam = ArmCamera()
        self.images = [cv2.imread(str(self.run / v["image"])) for v in self.views]
        self.markers = [detect_markers(img, self.table.layout) for img in self.images]

    # ------------------------------------------------------------------ registration
    def register(self) -> dict:
        """base_T_map per view (>= 3 markers), and their median. Map v3 leaves out the codes under two bins,
        and blocks stand on others: when no view shows 3 codes, views with 2 are used (two codes give 8
        coplanar points, which the pose solves; it is only less well conditioned)."""
        base_T_map0 = to_matrix(np.eye(3), self.table.offset)
        per_view = []
        for need in (3, 2):
            for v, m in zip(self.views, self.markers):
                if len(m) < need:
                    continue
                base_T_cam = self.cam.base_T_cam(v["readback"])
                prior = (np.linalg.inv(base_T_map0) @ base_T_cam)[:3, 3]
                pose = solve_camera_pose(m, self.table, self.k, self.dist, prior_centre=prior)
                if pose is None or pose.reprojection_px > 2.5:
                    continue
                per_view.append(dict(image=v["image"], markers=sorted(m), reproj_px=round(pose.reprojection_px, 2),
                                     base_T_map=base_T_cam @ to_matrix(pose.R, pose.t)))
            if per_view:
                break
        if not per_view:
            raise RuntimeError("no view sees 2 markers: cannot register the map")
        t = np.array([p["base_T_map"][:3, 3] for p in per_view])
        yaw = np.array([math.atan2(p["base_T_map"][1, 0], p["base_T_map"][0, 0]) for p in per_view])
        median_t, median_yaw = np.median(t, axis=0), float(np.median(yaw))
        self.base_T_map = to_matrix(np.array([[math.cos(median_yaw), -math.sin(median_yaw), 0],
                                              [math.sin(median_yaw), math.cos(median_yaw), 0], [0, 0, 1]]), median_t)
        self.registration = dict(
            views=len(per_view), map_origin_base_cm=(median_t * 100).round(2).tolist(),
            yaw_deg=round(math.degrees(median_yaw), 2),
            spread_cm=(t.std(axis=0) * 100).round(2).tolist(),
            per_view=[dict(image=p["image"], markers=p["markers"], reproj_px=p["reproj_px"],
                           origin_cm=(p["base_T_map"][:3, 3] * 100).round(2).tolist()) for p in per_view])
        return self.registration

    def camera_poses(self) -> list[CameraPose]:
        """map -> camera for each view: its own markers when >= 2, else FK + hand-eye + registration."""
        poses = []
        for v, m in zip(self.views, self.markers):
            fk = np.linalg.inv(self.cam.base_T_cam(v["readback"])) @ self.base_T_map   # cam_T_map
            pose = None
            if len(m) >= 2:
                pose = solve_camera_pose(m, self.table, self.k, self.dist, prior_centre=-fk[:3, :3].T @ fk[:3, 3])
                if pose is not None and pose.reprojection_px > 2.5:
                    pose = None
            poses.append(pose if pose is not None else CameraPose(fk[:3, :3], fk[:3, 3], [], float("nan")))
        self.poses = poses
        return poses

    # ------------------------------------------------------------------ orthophotos
    def ortho(self, z: float, margin_px: int = 20) -> tuple[np.ndarray, np.ndarray]:
        """(image, weight) of all views resampled on the plane Z = z (map metres) at 1 mm/px.
        Row 0 is the far edge (largest y), so the image looks like the map seen from above."""
        xs = np.arange(X_RANGE[0], X_RANGE[1], MM)
        ys = np.arange(Y_RANGE[1], Y_RANGE[0], -MM)
        gx, gy = np.meshgrid(xs, ys)
        pts = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)])
        acc = np.zeros(gx.shape + (3,), np.float64)
        wsum = np.zeros(gx.shape, np.float64)
        for img, pose in zip(self.images, self.poses):
            cam_pts = pts @ pose.R.T + pose.t
            front = cam_pts[:, 2] > 0.02
            uv = pose.project(pts, self.k, self.dist).astype(np.float32)
            u, v = uv[:, 0].reshape(gx.shape), uv[:, 1].reshape(gx.shape)
            h, w = img.shape[:2]
            edge = np.minimum.reduce([u, v, w - 1 - u, h - 1 - v])
            weight = np.clip((edge - margin_px) / 60.0, 0, 1) * front.reshape(gx.shape)
            sample = cv2.remap(img, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            acc += sample.astype(np.float64) * weight[..., None]
            wsum += weight
        out = (acc / np.maximum(wsum, 1e-6)[..., None]).clip(0, 255).astype(np.uint8)
        return out, wsum

    def foreground(self, z: float, margin_px: int = 20) -> np.ndarray:
        """0..1 on the plane Z = z: the weighted share of views in which the point is not paper (the
        class-agnostic map-difference mask of each view, map_difference.object_mask)."""
        import map_difference as md
        xs = np.arange(X_RANGE[0], X_RANGE[1], MM)
        ys = np.arange(Y_RANGE[1], Y_RANGE[0], -MM)
        gx, gy = np.meshgrid(xs, ys)
        pts = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)])
        acc = np.zeros(gx.shape)
        wsum = np.zeros(gx.shape)
        for img, pose in zip(self.images, self.poses):
            mask, _, _ = md.object_mask(img, pose, self.k, self.dist, self.table.layout)
            uv = pose.project(pts, self.k, np.zeros(5)).astype(np.float32)      # the mask is undistorted
            u, v = uv[:, 0].reshape(gx.shape), uv[:, 1].reshape(gx.shape)
            h, w = img.shape[:2]
            edge = np.minimum.reduce([u, v, w - 1 - u, h - 1 - v])
            front = ((pts @ pose.R.T + pose.t)[:, 2] > 0.02).reshape(gx.shape)
            weight = np.clip((edge - margin_px) / 60.0, 0, 1) * front
            acc += cv2.remap(mask.astype(np.float32) / 255, u, v, cv2.INTER_LINEAR, borderValue=0) * weight
            wsum += weight
        return acc / np.maximum(wsum, 1e-6)

    @staticmethod
    def to_px(x: float, y: float) -> tuple[float, float]:
        return (x - X_RANGE[0]) / MM, (Y_RANGE[1] - y) / MM

    @staticmethod
    def to_map(col: float, row: float) -> tuple[float, float]:
        return X_RANGE[0] + col * MM, Y_RANGE[1] - row * MM


def main() -> None:
    run = Path(sys.argv[1])
    scan = Scan(run)
    reg = scan.register()
    print("registration:", json.dumps({k: v for k, v in reg.items() if k != "per_view"}))
    for p in reg["per_view"]:
        print("  ", p)
    scan.camera_poses()
    for name, z in (("table", 0.0), ("top", BLOCK)):
        img, w = scan.ortho(z)
        img[w < 1e-3] = (40, 40, 40)
        cv2.imwrite(str(run / f"ortho_{name}.png"), img)
    print("wrote", run / "ortho_table.png", run / "ortho_top.png")


if __name__ == "__main__":
    main()
