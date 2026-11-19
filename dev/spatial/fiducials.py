# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy"]
# ///
"""Fixed ArUco fiducials (DICT_4X4_50, ids 20-23, 25 mm) as the table reference.

survey:   from one image, lay out every visible fiducial on the table plane in a 2D
          frame anchored at a reference fiducial (default id 22): origin at its centre,
          x along its top edge, y along its left edge towards its top, units metres.
register: from a later image, fit image -> table homography using every fiducial
          corner found, and report how well the fiducials agree (leave-one-out error).

Runs on the Mac (uv) or on the board (/root/arm-app/venv-vision/bin/python).
  python fiducials.py survey IMAGE --intrinsics camera_intrinsics.json --out layout.json
  python fiducials.py register IMAGE --intrinsics camera_intrinsics.json --layout layout.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

IDS = (20, 21, 22, 23)
SIDE_M = 0.025


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = json.loads(path.read_text())
    return np.array(data["camera_matrix"], float), np.array(data["dist_coeffs"], float)


def detect(image: np.ndarray) -> dict[int, np.ndarray]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(image)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners) if int(i) in IDS}


def undistort(points: np.ndarray, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """Pixel coordinates with lens distortion removed, still in pixels (P = K)."""
    return cv2.undistortPoints(points.reshape(-1, 1, 2).astype(np.float64), k, dist, P=k).reshape(-1, 2)


def marker_object_points(side: float = SIDE_M) -> np.ndarray:
    h = side / 2  # ArUco corner order: top-left, top-right, bottom-right, bottom-left
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)


def survey(markers: dict[int, np.ndarray], k: np.ndarray, dist: np.ndarray, ref: int) -> dict[int, list]:
    """Corner positions of every fiducial in the reference fiducial's 2D table frame."""
    if ref not in markers:
        raise RuntimeError(f"reference fiducial {ref} not visible")
    obj = marker_object_points()
    ok, rv, tv = cv2.solvePnP(obj, markers[ref], k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    ref_to_cam = np.eye(4)
    ref_to_cam[:3, :3], ref_to_cam[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
    # Intersect each corner's viewing ray with the reference plane (z = 0 in the ref frame).
    cam_to_ref = np.linalg.inv(ref_to_cam)
    origin = cam_to_ref[:3, 3]
    layout = {}
    for mid, pix in markers.items():
        rays = cv2.undistortPoints(pix.reshape(-1, 1, 2).astype(np.float64), k, dist).reshape(-1, 2)
        pts = []
        for x, y in rays:
            direction = cam_to_ref[:3, :3] @ np.array([x, y, 1.0])
            s = -origin[2] / direction[2]
            pts.append((origin + s * direction)[:2])
        layout[mid] = np.array(pts).round(5).tolist()
    return layout


def homography(markers: dict[int, np.ndarray], layout: dict[int, list], k: np.ndarray, dist: np.ndarray,
               use: tuple[int, ...]) -> np.ndarray:
    src = np.vstack([undistort(markers[m], k, dist) for m in use])
    dst = np.vstack([np.array(layout[m]) for m in use])
    h, _ = cv2.findHomography(src, dst, 0)
    return h


def to_table(h: np.ndarray, pix: np.ndarray, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(undistort(pix, k, dist).reshape(-1, 1, 2), h).reshape(-1, 2)


def leave_one_out(markers: dict[int, np.ndarray], layout: dict[int, list], k: np.ndarray, dist: np.ndarray) -> dict:
    """Predict each fiducial's corners from the others; error in mm."""
    common = [m for m in markers if m in layout]
    errors = {}
    for held in common:
        others = tuple(m for m in common if m != held)
        if not others:
            continue
        h = homography(markers, layout, k, dist, others)
        pred = to_table(h, markers[held], k, dist)
        errors[held] = round(float(np.linalg.norm(pred - np.array(layout[held]), axis=1).mean() * 1000), 2)
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("survey", "register"))
    parser.add_argument("image", type=Path)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--layout", type=Path, help="layout JSON (register)")
    parser.add_argument("--ref", type=int, default=22, help="reference fiducial for survey")
    parser.add_argument("--no-distortion", action="store_true", help="ignore distortion coefficients")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()

    k, dist = load_intrinsics(args.intrinsics)
    if args.no_distortion:
        dist = np.zeros(5)
    markers = detect(cv2.imread(str(args.image)))
    result: dict = {"image": str(args.image), "visible": sorted(markers)}
    if args.command == "survey":
        layout = survey(markers, k, dist, args.ref)
        result.update(reference=args.ref, side_m=SIDE_M, layout_m=layout,
                      centres_cm={m: (np.mean(c, axis=0) * 100).round(2).tolist() for m, c in layout.items()},
                      leave_one_out_mm=leave_one_out(markers, layout, k, dist))
    else:
        layout = {int(m): c for m, c in json.loads(args.layout.read_text())["layout_m"].items()}
        use = tuple(m for m in markers if m in layout)
        if len(use) < 2:
            raise SystemExit(f"need at least 2 known fiducials, saw {sorted(markers)}")
        h = homography(markers, layout, k, dist, use)
        result.update(used=list(use), image_to_table=h.tolist(),
                      leave_one_out_mm=leave_one_out(markers, layout, k, dist))
    print(json.dumps({k2: v for k2, v in result.items() if k2 not in ("layout_m", "image_to_table")}, indent=1))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
