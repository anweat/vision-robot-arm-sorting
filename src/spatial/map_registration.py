"""Camera pose from the printed table map, and pixel <-> table conversions.

The map (config/map_a4_v2.json, made by src/tools/make_map.py) defines a right-handed
frame on the table: X to the arm's right, Y away from the arm, Z up, origin at the
centre of the page's near edge. The arm base frame is the same frame shifted by the
placement offsets in the JSON (measured by hand; verify on site).

Every visible map marker contributes its four corners to one planar PnP. With a single
marker the planar solution can flip, so the candidate whose camera lies above the map
and, if given, nearest the forward-kinematics prediction is kept.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
# The map the pipeline and the simulation assume is on the table: ARM_MAP=map_a4_v4 (a config/ name) picks another
# printed sheet, the default is the calibrated A4 v2. Everything that reads DEFAULT_MAP follows it.
DEFAULT_MAP = ROOT / "config" / f"{os.environ.get('ARM_MAP', 'map_a4_v2')}.json"


class TableMap:
    def __init__(self, path: Path = DEFAULT_MAP):
        data = json.loads(Path(path).read_text())
        self.layout = {int(k): np.array(v, float) for k, v in data["layout_m"].items()}
        self.side = float(data["side_m"])
        base = data["placement"]["base_from_map"]
        self.offset = np.array([base["dx"], base["dy"], base["dz"]], float)

    def map_to_base(self, p: Sequence[float]) -> np.ndarray:
        return np.asarray(p, float) + self.offset

    def base_to_map(self, p: Sequence[float]) -> np.ndarray:
        return np.asarray(p, float) - self.offset


def detect_markers(image: np.ndarray, ids: Sequence[int]) -> dict[int, np.ndarray]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, found, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(image)
    if found is None:
        return {}
    wanted = set(ids)
    return {int(i): c.reshape(4, 2) for i, c in zip(found.ravel(), corners) if int(i) in wanted}


@dataclass
class CameraPose:
    """map -> camera: X_cam = R @ X_map + t."""
    R: np.ndarray
    t: np.ndarray
    markers: list = field(default_factory=list)
    reprojection_px: float = float("nan")

    @property
    def centre(self) -> np.ndarray:
        return -self.R.T @ self.t

    def pixel_to_plane(self, uv: Sequence[float], k: np.ndarray, dist: np.ndarray, z: float = 0.0) -> np.ndarray:
        """Map-frame point where the pixel's ray meets the horizontal plane Z = z."""
        x, y = cv2.undistortPoints(np.array([[uv]], float), k, dist).ravel()
        direction = self.R.T @ np.array([x, y, 1.0])
        origin = self.centre
        if abs(direction[2]) < 1e-9:
            raise ValueError("ray parallel to the plane")
        s = (z - origin[2]) / direction[2]
        if s <= 0:
            raise ValueError("plane is behind the camera")
        return origin + s * direction

    def project(self, points_map: np.ndarray, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
        rvec, _ = cv2.Rodrigues(self.R)
        pix, _ = cv2.projectPoints(np.asarray(points_map, float).reshape(-1, 1, 3), rvec, self.t, k, dist)
        return pix.reshape(-1, 2)


def solve_camera_pose(markers: dict[int, np.ndarray], table: TableMap, k: np.ndarray, dist: np.ndarray,
                      prior_centre: Optional[np.ndarray] = None) -> Optional[CameraPose]:
    use = [m for m in markers if m in table.layout]
    if not use:
        return None
    obj = np.vstack([np.column_stack([table.layout[m], np.zeros(4)]) for m in use])
    img = np.vstack([markers[m] for m in use]).astype(float)
    _, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, img, k, dist, flags=cv2.SOLVEPNP_IPPE)
    best, best_key = None, None
    for rv, tv, err in zip(rvecs, tvecs, np.ravel(errors)):
        r = cv2.Rodrigues(rv)[0]
        pose = CameraPose(r, tv.ravel(), use, float(err))
        if pose.centre[2] <= 0:
            continue
        key = float(err) if prior_centre is None else float(np.linalg.norm(pose.centre - prior_centre))
        if best_key is None or key < best_key:
            best, best_key = pose, key
    if best is None:
        return None
    if len(use) > 1:  # refine jointly once the planar ambiguity is resolved
        rv, _ = cv2.Rodrigues(best.R)
        ok, rv, tv = cv2.solvePnP(obj, img, k, dist, rv, best.t.copy(), True, cv2.SOLVEPNP_ITERATIVE)
        if ok:
            best = CameraPose(cv2.Rodrigues(rv)[0], tv.ravel(), use)
    proj = best.project(obj, k, dist)
    best.reprojection_px = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1))))
    return best


def load_intrinsics(path: Path = ROOT / "config" / "calibration" / "camera_intrinsics.json") -> tuple[np.ndarray, np.ndarray]:
    data = json.loads(Path(path).read_text())
    return np.array(data["camera_matrix"], float), np.array(data["dist_coeffs"], float)
