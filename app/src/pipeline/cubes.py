"""Find 30 mm cubes on the block-top orthophoto (src/pipeline/scene.py): centre and yaw, class-agnostic.

On the plane of the tops a cube's top face is an undistorted 30 x 30 px square (1 mm/px) whatever is
printed on it (colour or sticker), and its outline is a strong edge. For each yaw (0-90 deg; a square
repeats every 90) a kernel holding a 30 mm square outline is correlated with the edge-strength image; the
best responses are the cubes. Map markers are 15 mm and the map's lines are long and thin, so neither
fills the outline. Each pick suppresses its neighbourhood, so touching cubes are found one by one.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

SIDE_PX = 30        # 30 mm at 1 mm/px
YAW_STEP = 3


def edge_strength(image: np.ndarray, weight: np.ndarray | None = None) -> np.ndarray:
    """Colour gradient magnitude (max over channels), lightly smoothed; zero where no view covered."""
    img = cv2.GaussianBlur(image, (3, 3), 0).astype(np.float32)
    mags = []
    for c in range(3):
        gx = cv2.Sobel(img[..., c], cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(img[..., c], cv2.CV_32F, 0, 1, ksize=3)
        mags.append(np.hypot(gx, gy))
    mag = np.max(mags, axis=0)
    if weight is not None:
        mag[weight < 0.05] = 0
    return mag / (np.percentile(mag[mag > 0], 99) + 1e-6) if (mag > 0).any() else mag


def outline_kernel(yaw_deg: float, side: float = SIDE_PX, width: float = 1.5) -> np.ndarray:
    size = int(math.ceil(side * 1.5)) | 1
    k = np.zeros((size, size), np.float32)
    c = size / 2
    half = side / 2
    t = math.radians(yaw_deg)
    corners = [(c + x * math.cos(t) - y * math.sin(t), c + x * math.sin(t) + y * math.cos(t))
               for x, y in ((-half, -half), (half, -half), (half, half), (-half, half))]
    pts = (np.array(corners) * 16).astype(np.int32)
    cv2.polylines(k, [pts], True, 1.0, thickness=max(1, int(round(width))), lineType=cv2.LINE_AA, shift=4)
    return k / k.sum()


def square_kernel(yaw_deg: float, side: float = SIDE_PX * 0.8) -> np.ndarray:
    """Filled square (the inner 80 % of a top face), normalised to sum 1."""
    size = int(math.ceil(SIDE_PX * 1.5)) | 1
    k = np.zeros((size, size), np.float32)
    c, half, t = size / 2, side / 2, math.radians(yaw_deg)
    pts = np.array([(c + x * math.cos(t) - y * math.sin(t), c + x * math.sin(t) + y * math.cos(t))
                    for x, y in ((-half, -half), (half, -half), (half, half), (-half, half))]) * 16
    cv2.fillPoly(k, [pts.astype(np.int32)], 1.0, lineType=cv2.LINE_AA, shift=4)
    return k / k.sum()


def find_cubes(image: np.ndarray, weight: np.ndarray | None = None, max_cubes: int = 8,
               min_score: float = 0.35, foreground: np.ndarray | None = None, min_fill: float = 0.6) -> list[dict]:
    """Cubes on a 1 mm/px top-plane orthophoto: [{col, row, yaw_deg, score, fill}] best first.
    With `foreground` (0..1, not-paper evidence on the same grid) a square counts only if at least
    `min_fill` of its inside is foreground: paper edges, markers and view seams are paper."""
    mag = edge_strength(image, weight)
    best = np.zeros(mag.shape, np.float32)
    best_yaw = np.zeros(mag.shape, np.float32)
    best_fill = np.ones(mag.shape, np.float32)
    for yaw in range(0, 90, YAW_STEP):
        r = cv2.filter2D(mag, -1, outline_kernel(yaw), borderType=cv2.BORDER_CONSTANT)
        if foreground is not None:
            fill = cv2.filter2D(foreground.astype(np.float32), -1, square_kernel(yaw), borderType=cv2.BORDER_CONSTANT)
            r = np.where(fill >= min_fill, r, 0)
        better = r > best
        best[better], best_yaw[better] = r[better], yaw
        if foreground is not None:
            best_fill[better] = fill[better]
    cubes = []
    score = best.copy()
    for _ in range(max_cubes):
        row, col = np.unravel_index(np.argmax(score), score.shape)
        s = float(score[row, col])
        if s < min_score:
            break
        cubes.append(dict(col=int(col), row=int(row), yaw_deg=float(best_yaw[row, col]), score=round(s, 3),
                          fill=round(float(best_fill[row, col]), 2)))
        cv2.circle(score, (int(col), int(row)), int(SIDE_PX * 0.9), 0, -1)   # neighbours cannot be another cube
    return cubes


def refine_at(image: np.ndarray, weight: np.ndarray | None, col: float, row: float, window_px: int = 20,
              penalty_per_px: float = 0.01) -> dict:
    """The best 30 mm outline within window_px of (col, row), each pixel off costing penalty_per_px: a block
    whose position is known roughly (placed on a marker, or found by the detector) squared up on the
    orthophoto. White sticker tops have weak outlines, so shadows and sides would pull it off otherwise."""
    mag = edge_strength(image, weight)
    col, row = int(round(col)), int(round(row))
    yy, xx = np.mgrid[-window_px:window_px + 1, -window_px:window_px + 1]
    penalty = penalty_per_px * np.hypot(xx, yy)
    best = dict(col=col, row=row, yaw_deg=0.0, score=-1.0)
    for yaw in range(0, 90, YAW_STEP):
        r = cv2.filter2D(mag, -1, outline_kernel(yaw), borderType=cv2.BORDER_CONSTANT)
        win = r[row - window_px:row + window_px + 1, col - window_px:col + window_px + 1]
        if win.shape != penalty.shape:
            continue
        win = win - penalty
        i = np.unravel_index(win.argmax(), win.shape)
        if win[i] > best["score"]:
            best = dict(col=col - window_px + int(i[1]), row=row - window_px + int(i[0]), yaw_deg=float(yaw),
                        score=round(float(win[i]), 3))
    return best


def draw(image: np.ndarray, cubes: list[dict]) -> np.ndarray:
    out = image.copy()
    for i, c in enumerate(cubes):
        k = outline_kernel(c["yaw_deg"])
        half = SIDE_PX / 2
        t = math.radians(c["yaw_deg"])
        pts = np.array([(c["col"] + x * math.cos(t) - y * math.sin(t), c["row"] + x * math.sin(t) + y * math.cos(t))
                        for x, y in ((-half, -half), (half, -half), (half, half), (-half, half))], np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 255), 1)
        cv2.putText(out, str(i + 1), (c["col"] - 4, c["row"] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    return out
