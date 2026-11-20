"""Refine a detected 3 cm block to its pose on the table from the silhouette, no new model.

A detector says "there is a block in this box"; this module says *where the cube stands and how it is
turned*. The block is a 3 cm cube on the table, and the camera pose is known, so the cube's silhouette is
a known function of (x, y, yaw): project the 8 corners, take the convex hull. The block-paper boundary of
that hull is a strong closed edge whatever is printed on the top face (colour or sticker), which is exactly
where a top-face model fails: the sticker artwork and the printed map carry strong edges of their own.

Fit (x, y, yaw) by maximising
    silhouette score = mean |gradient . edge normal| along the projected hull, plus
    bbox match      = IoU of the projected hull box with the detector box.
The bbox term is a coarse prior (the detector box is tight but includes some shadow); the directional
gradient term fixes the yaw to a fraction of a degree and rejects the sticker artwork, whose gradients do
not line up with the outline. The gradient is sampled with the full distorsion model removed, on a plane
whose projection is a homography, so the search costs a few milliseconds per evaluation.

The top face centre is directly above the cube centre, so the pose gives it without fitting the top face:
    centre = (x, y) on the Z = 30 mm plane, yaw folded to [-45, 45) counter-clockwise positive
-- the same convention as `localization.locate_board` and `scene` cubes.

Failure is explicit (`ok=False` with a reason): a box at the frame edge, a fit that runs away from the
detector box, a silhouette that matches the box badly, or too few outline samples with edge support.

    python3 src/vision/roi_pose.py --run data/runs/pipeline/<run> --view 0     # fit every detection
    python3 src/vision/roi_pose.py --image frame.jpg --bbox 100 80 160 140 --servos 90 105 5 5 95
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

BLOCK = 0.03                 # cube edge, metres
HALF = BLOCK / 2
TOP_Z = BLOCK                # top face height
MIN_SCORE = 0.055            # mean |grad . normal| along the silhouette (normalised gradient)
MIN_IOU = 0.55               # projected hull box vs detector box
MAX_CENTRE_SHIFT_M = 0.02    # a fit this far from the detector box is not trusted
MIN_SUPPORT = 0.35           # share of outline samples with a real edge response
SAMPLES_PER_EDGE = 14
BBOX_WEIGHT = 0.20           # bbox match in the score: coarse prior, edge term does the yaw
PRIOR_WEIGHT = 0.04          # cost of moving 1 cm away from the detector/blob centre


# --------------------------------------------------------------------------- projection helpers

def undistort_frame(image: np.ndarray, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    map1, map2 = cv2.initUndistortRectifyMap(k, dist, None, k, (w, h), cv2.CV_32FC1)
    return cv2.remap(image, map1, map2, cv2.INTER_LINEAR)


def gradient_components(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """x and y Sobel gradients of the undistorted frame, normalised by the 99.5th percentile of |g|."""
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32), (0, 0), 1.0)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    high = float(np.percentile(np.hypot(gx, gy), 99.5))
    return gx / max(high, 1e-6), gy / max(high, 1e-6)


def cube_corners_uv(pose, k: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """The 8 cube corners in undistorted pixels; raises ValueError behind the camera."""
    c, s = math.cos(yaw), math.sin(yaw)
    points = np.array([[x + u * c - v * s, y + u * s + v * c, z]
                       for u, v in ((-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF))
                       for z in (0.0, TOP_Z)])
    cam = points @ pose.R.T + pose.t
    if (cam[:, 2] < 0.02).any():
        raise ValueError("cube behind the camera")
    uv = (k @ cam.T).T
    return uv[:, :2] / uv[:, 2:3]


def top_face_uv(pose, k: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """The four top-face corners in undistorted pixels, in the same order as `square_corners`."""
    c, s = math.cos(yaw), math.sin(yaw)
    points = np.array([[x + u * c - v * s, y + u * s + v * c, TOP_Z]
                       for u, v in ((-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF))])
    cam = points @ pose.R.T + pose.t
    uv = (k @ cam.T).T
    return uv[:, :2] / uv[:, 2:3]


def _sample(image: np.ndarray, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Bilinear samples at float pixel positions and the in-frame mask."""
    x, y = points[:, 0], points[:, 1]
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    h, w = image.shape[:2]
    inside = (x0 >= 0) & (y0 >= 0) & (x0 < w - 1) & (y0 < h - 1)
    x0, y0 = np.clip(x0, 0, w - 2), np.clip(y0, 0, h - 2)
    fx, fy = x - x0, y - y0
    v = (image[y0, x0] * (1 - fx) * (1 - fy) + image[y0, x0 + 1] * fx * (1 - fy)
         + image[y0 + 1, x0] * (1 - fx) * fy + image[y0 + 1, x0 + 1] * fx * fy)
    return v, inside


def _hull_samples(hull: np.ndarray, samples: int = SAMPLES_PER_EDGE):
    """Points along the hull edges, their outward normals, and per-sample edge lengths (vectorised)."""
    a = hull
    b = np.roll(hull, -1, axis=0)
    d = b - a
    length = np.linalg.norm(d, axis=1)
    keep = length >= 4.0
    a, b, d, length = a[keep], b[keep], d[keep], length[keep]
    if len(a) == 0:
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
    normals = np.stack([-d[:, 1], d[:, 0]], axis=1) / length[:, None]
    centre = hull.mean(axis=0)
    mid = (a + b) / 2
    flip = np.einsum("ij,ij->i", normals, mid - centre) < 0
    normals[flip] *= -1
    t = np.linspace(0.08, 0.92, samples)[None, :, None]
    base = a[:, None, :] + (b - a)[:, None, :] * t          # (E, S, 2)
    offsets = np.array([-0.6, 0.0, 0.6])[:, None, None, None]   # +/- sub-pixel across the edge
    points = base[None, :, :, :] + offsets * normals[None, :, None, :]
    points = points.reshape(-1, 2)
    repeat = np.repeat(normals[:, None, :], samples, axis=1)  # (E, S, 2)
    return points, np.tile(repeat.reshape(-1, 2), (3, 1)), np.tile(np.repeat(length, samples), 3)


def silhouette_score(gx: np.ndarray, gy: np.ndarray, pose, k: np.ndarray,
                     x: float, y: float, yaw: float) -> tuple[float, float, np.ndarray]:
    """(weighted mean |grad . normal|, edge support, hull) for one pose."""
    uv = cube_corners_uv(pose, k, x, y, yaw)
    hull = cv2.convexHull(uv.astype(np.float32)).reshape(-1, 2)
    points, normals, lengths = _hull_samples(hull)
    if len(points) == 0:
        return -1.0, 0.0, hull
    gxv, inside = _sample(gx, points)
    gyv, _ = _sample(gy, points)
    response = np.abs(gxv * normals[:, 0] + gyv * normals[:, 1])
    response = np.where(inside, response, 0.0)
    weights = lengths / lengths.sum()
    support = float(np.average(response > 0.10, weights=weights))
    return float(np.sum(response * weights)), support, hull


def _bbox(hull: np.ndarray) -> np.ndarray:
    return np.array([hull[:, 0].min(), hull[:, 1].min(), hull[:, 0].max(), hull[:, 1].max()])


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


# --------------------------------------------------------------------------- the fit

def _initial_centre(bbox: np.ndarray, pose, k: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """The detector box centre is the projection of the cube centre, so its ray meets Z = 15 mm there."""
    cx, cy = (bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0
    return pose.pixel_to_plane((cx, cy), k, dist, z=HALF)[:2]


def _objective(gx, gy, pose, k, bbox, x, y, yaw, prior=None):
    edge, support, hull = silhouette_score(gx, gy, pose, k, x, y, yaw)
    if edge < 0:
        return -1.0, 0.0, edge, support, hull
    match = _iou(_bbox(hull), bbox)
    score = edge + BBOX_WEIGHT * match
    if prior is not None:   # stay on the block the detector box and the colour blob name
        score -= PRIOR_WEIGHT * min(math.hypot(x - prior[0], y - prior[1]) / 0.01, 3.0)
    return score, match, edge, support, hull


def _search(gx, gy, pose, k, bbox, centre, initial_yaw=None, search_m=0.005, yaw_step=5.0, prior=None):
    x, y = float(centre[0]), float(centre[1])
    limit = math.hypot(search_m, search_m) + 0.006   # the fit may not walk to a neighbouring block
    def near(px, py):
        return math.hypot(px - x, py - y) <= limit
    if initial_yaw is None:
        yaws = list(np.arange(0.0, 90.0, yaw_step))
    else:   # a colour-face yaw already narrows the quarter turn
        yaws = [float((initial_yaw + d + 45) % 90 - 45) for d in (-8, -4, 0, 4, 8)]
    best = (-1.0, 0.0, 0.0, 0.0, 0.0)
    for angle in yaws:
        score, match, edge, support, hull = _objective(gx, gy, pose, k, bbox, x, y, math.radians(angle), prior)
        if score > best[0]:
            best = (score, angle, x, y, match)
    score, angle, bx, by, _ = best
    for dx in np.arange(-search_m, search_m + 1e-9, 0.0025):
        for dy in np.arange(-search_m, search_m + 1e-9, 0.0025):
            for da in np.arange(-4.0, 4.0 + 1e-9, 2.0):
                if not near(x + dx, y + dy):
                    continue
                s, match, edge, support, hull = _objective(gx, gy, pose, k, bbox, x + dx, y + dy,
                                                           math.radians(angle + da), prior)
                if s > score:
                    score, angle, bx, by = s, angle + da, x + dx, y + dy
    step_xy, step_a = 0.002, 2.0
    for _ in range(3):
        improved = True
        while improved:
            improved = False
            for ddx, ddy, dda in ((step_xy, 0, 0), (-step_xy, 0, 0), (0, step_xy, 0), (0, -step_xy, 0),
                                  (0, 0, step_a), (0, 0, -step_a)):
                if not near(bx + ddx, by + ddy):
                    continue
                s, match, edge, support, hull = _objective(gx, gy, pose, k, bbox, bx + ddx, by + ddy,
                                                           math.radians(angle + dda), prior)
                if s > score + 1e-7:
                    score, angle, bx, by = s, angle + dda, bx + ddx, by + ddy
                    improved = True
        step_xy, step_a = step_xy / 2, step_a / 2
    return bx, by, angle, score


def solve_top_face(image: np.ndarray, bbox, pose, k: np.ndarray, dist: np.ndarray,
                   initial_yaw: float | None = None, prior_xy=None) -> dict:
    """Cube pose from one detector box. See the module docstring for the contract."""
    fail = lambda why, **extra: dict(ok=False, reason=why, **extra)
    bbox = np.asarray(bbox, float)
    if image is None or image.ndim != 3:
        return fail("no image")
    h_img, w_img = image.shape[:2]
    if bbox[2] - bbox[0] < 14 or bbox[3] - bbox[1] < 14:
        return fail("box too small")
    if bbox[0] < 2 or bbox[1] < 2 or bbox[2] > w_img - 2 or bbox[3] > h_img - 2:
        return fail("box touches the frame edge")
    undistorted = undistort_frame(image, k, dist)
    gx, gy = gradient_components(undistorted)
    try:
        centre = _initial_centre(bbox, pose, k, dist)
    except ValueError as exc:
        return fail(f"centre ray: {exc}")
    prior = None
    if prior_xy is not None:   # the colour blob or the previous fit: keeps the search on this block
        prior = np.asarray(prior_xy, float)[:2]
        centre = prior.copy()
    if not (-0.2 < centre[0] < 0.2 and -0.1 < centre[1] < 0.25):
        return fail("centre off the map")
    x, y, yaw, score = _search(gx, gy, pose, k, bbox, centre, initial_yaw, prior=prior)
    edge, support, hull = silhouette_score(gx, gy, pose, k, x, y, math.radians(yaw))
    match = _iou(_bbox(hull), bbox)
    shift = float(np.hypot(x - centre[0], y - centre[1]))
    yaw_deg = round(float((yaw + 45) % 90 - 45), 2)
    uv = top_face_uv(pose, k, x, y, math.radians(yaw_deg))
    sides = [float(np.linalg.norm(uv[i] - uv[(i + 1) % 4])) for i in range(4)]
    reasons = []
    if edge < MIN_SCORE:
        reasons.append(f"weak outline {edge:.3f}")
    if support < MIN_SUPPORT:
        reasons.append(f"outline support {support:.2f}")
    if match < MIN_IOU:
        reasons.append(f"box match {match:.2f}")
    if shift > MAX_CENTRE_SHIFT_M:
        reasons.append(f"centre moved {shift * 100:.1f} cm")
    if reasons:
        return fail("; ".join(reasons), score=round(edge, 3), box_iou=round(match, 2),
                    support=round(support, 2), map_cm=[round(x * 100, 2), round(y * 100, 2)],
                    yaw_deg=yaw_deg)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    top = np.array([[x + u * c - v * s, y + u * s + v * c, TOP_Z]
                    for u, v in ((-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF))])
    original = pose.project(top, k, dist)
    return dict(ok=True, reason="", map_cm=[round(x * 100, 2), round(y * 100, 2)], yaw_deg=yaw_deg,
                corners_px=[[round(float(u), 1), round(float(v), 1)] for u, v in original],
                score=round(edge, 3), box_iou=round(match, 2), support=round(support, 2),
                side_px=[round(v, 1) for v in sides], shift_cm=round(shift * 100, 2), method="roi-silhouette")


# --------------------------------------------------------------------------- command line

def _annotate(image: np.ndarray, fix: dict) -> np.ndarray:
    out = image.copy()
    if fix.get("corners_px"):
        pts = np.array(fix["corners_px"], np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 0), 1, cv2.LINE_AA)
        for point in pts:
            cv2.circle(out, tuple(point), 2, (0, 0, 255), -1)
    text = (f"{fix['yaw_deg']:+.1f} deg  score {fix['score']:.2f}  iou {fix['box_iou']:.2f}"
            if fix.get("ok") else fix.get("reason", ""))
    colour = (0, 255, 0) if fix.get("ok") else (0, 0, 255)
    cv2.putText(out, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
    return out


def _detect(image: np.ndarray) -> list[dict]:
    from blocks_detector import BlocksDetector
    return BlocksDetector().detect(image, conf=0.25)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Fit a block box to the cube's silhouette (centre + yaw).")
    parser.add_argument("--image", type=Path, help="a 640x480 wrist frame")
    parser.add_argument("--bbox", type=float, nargs=4, help="x0 y0 x1 y1")
    parser.add_argument("--servos", type=float, nargs=5, help="J1..J5 for a predicted camera pose")
    parser.add_argument("--run", type=Path, help="a scan run (scene.Scan); fit every detected block")
    parser.add_argument("--view", type=int, default=0)
    parser.add_argument("--out", type=Path, help="write an annotated copy here")
    args = parser.parse_args(argv)
    if args.run:
        import scene as S
        scan = S.Scan(args.run)
        scan.register()
        poses = scan.camera_poses()
        image = scan.images[args.view]
        for det in _detect(image):
            fix = solve_top_face(image, det["bbox"], poses[args.view], scan.k, scan.dist)
            print(json.dumps(dict(detection=det, fit=fix), ensure_ascii=False))
        return 0
    if not args.image or not args.bbox:
        parser.error("--image and --bbox are required unless --run is used")
    from camera_model import ArmCamera
    from map_registration import TableMap, load_intrinsics
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f"cannot read {args.image}")
    k, dist = load_intrinsics()
    servos = list(args.servos) if args.servos else [90, 105, 5, 5, 95]
    pose = ArmCamera().pose_in_map(servos, TableMap())
    fix = solve_top_face(image, args.bbox, pose, k, dist)
    print(json.dumps(fix, ensure_ascii=False, indent=1))
    if args.out:
        cv2.imwrite(str(args.out), _annotate(image, fix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
