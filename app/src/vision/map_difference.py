"""Class-agnostic object detection against the known printed map.

The map's appearance is known (white page, black markers at designed positions), so the
expected view can be rendered from the solved camera pose and compared with the real
frame. Whatever does not match the map inside the page area is an object candidate,
whatever its colour or sticker. Thin printed details (ids, ticks, dashed line) are
removed by morphology, and the marker areas are masked because small registration
errors light up their edges.

Shadows also differ from the map. Pixels that are only darker than expected, unsaturated
and smooth are treated as shadow and dropped; hard lamp shadows can still leak through,
and the plane + parallax comparison between two views is the fallback if they do.
"""

from __future__ import annotations

import cv2
import numpy as np

from block_detector import Blob, hue_label, white_balance

PAGE_W, PAGE_H = 0.297, 0.210
PX_PER_M = 2000  # 2 px/mm raster of the page


def render_page(layout: dict) -> np.ndarray:
    page = np.full((round(PAGE_H * PX_PER_M), round(PAGE_W * PX_PER_M)), 255, np.float32)
    for corners in layout.values():
        tl, br = np.array(corners[0]), np.array(corners[2])
        u0, v0 = round((tl[0] + PAGE_W / 2) * PX_PER_M), round((PAGE_H - tl[1]) * PX_PER_M)
        u1, v1 = round((br[0] + PAGE_W / 2) * PX_PER_M), round((PAGE_H - br[1]) * PX_PER_M)
        page[v0:v1, u0:u1] = 0.0  # marker drawn as a solid square; its interior is masked anyway
    return page


def expected_view(pose, k: np.ndarray, layout: dict, size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Rendered map (0..1, white = 1) and the page-area mask, in the undistorted camera image."""
    page_to_map = np.array([[1 / PX_PER_M, 0, -PAGE_W / 2], [0, -1 / PX_PER_M, PAGE_H], [0, 0, 1]])
    map_to_img = k @ np.column_stack([pose.R[:, 0], pose.R[:, 1], pose.t])
    h = map_to_img @ page_to_map
    page = render_page(layout)
    view = cv2.warpPerspective(page / 255.0, h, size, flags=cv2.INTER_AREA, borderValue=0)
    area = cv2.warpPerspective(np.ones_like(page), h, size, flags=cv2.INTER_NEAREST, borderValue=0)
    return view, cv2.erode((area > 0.5).astype(np.uint8), np.ones((9, 9), np.uint8))


def object_mask(image: np.ndarray, pose, k: np.ndarray, dist: np.ndarray, layout: dict,
                threshold: float = 0.28) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(mask, hsv, undistorted image): what does not match the rendered map inside the page, in the
    undistorted image, with shadows and marker areas removed (see the module docstring)."""
    size = (image.shape[1], image.shape[0])
    undist = cv2.undistort(image, k, dist)
    gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY).astype(np.float32)
    expected, area = expected_view(pose, k, layout, size)

    white = ((expected > 0.9) & (area > 0)).astype(np.float32)
    level = cv2.GaussianBlur(gray * white, (0, 0), 25) / np.maximum(cv2.GaussianBlur(white, (0, 0), 25), 1e-3)
    level = np.maximum(level, 1.0)
    black = np.median(gray[(expected < 0.1) & (area > 0)]) if np.any((expected < 0.1) & (area > 0)) else 0.0
    predicted = black + (level - black) * expected
    diff = np.abs(gray - predicted) / level

    hsv = cv2.cvtColor(white_balance(undist), cv2.COLOR_BGR2HSV)
    smooth = cv2.GaussianBlur(gray, (0, 0), 1.5)
    grad = np.hypot(cv2.Sobel(smooth, cv2.CV_32F, 1, 0), cv2.Sobel(smooth, cv2.CV_32F, 0, 1)) / level
    grad = cv2.dilate(grad, np.ones((5, 5), np.uint8))
    shadow = (gray < predicted) & (hsv[..., 1] < 45) & (grad < 0.35)
    mask = ((((diff > threshold) & ~shadow) | (hsv[..., 1] > 90)) & (area > 0)).astype(np.uint8) * 255
    # every designed marker, not only the detected ones: half-visible markers at the border count too
    for corners in layout.values():
        pts = np.column_stack([np.array(corners), np.zeros(4)])
        und = (k @ (pose.R @ pts.T + pose.t.reshape(3, 1))).T
        und = und[:, :2] / und[:, 2:3]
        centre = und.mean(axis=0)
        cv2.fillConvexPoly(mask, (centre + (und - centre) * 1.35).astype(np.int32), 0)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return mask, hsv, undist


def detect_objects(image: np.ndarray, pose, k: np.ndarray, dist: np.ndarray, layout: dict,
                   markers: dict, threshold: float = 0.28, min_area: int = 350) -> list[Blob]:
    size = (image.shape[1], image.shape[0])
    mask, hsv, _ = object_mask(image, pose, k, dist, layout, threshold)
    blobs = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        # back to distorted pixel coordinates so the shared locate() applies unchanged
        pts = c.reshape(-1, 2).astype(np.float64)
        rays = cv2.undistortPoints(pts.reshape(-1, 1, 2), k, np.zeros(5)).reshape(-1, 2)
        distorted, _ = cv2.projectPoints(np.column_stack([rays, np.ones(len(rays))]), np.zeros(3), np.zeros(3), k, dist)
        c_dist = distorted.reshape(-1, 1, 2).astype(np.int32)
        x, y, w, h = cv2.boundingRect(c_dist)
        region = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(region, [c], -1, 255, -1)
        sat = hsv[..., 1][region > 0]
        hues = hsv[..., 0][(region > 0) & (hsv[..., 1] > 90)]
        label = hue_label(float(np.median(hues))) if len(hues) > 0.15 * len(sat) else "object"
        m = cv2.moments(c_dist)
        cu, cv_ = m["m10"] / m["m00"], m["m01"] / m["m00"]
        p = c_dist.reshape(-1, 2)
        band = p[np.abs(p[:, 0] - cu) <= max(4, 0.2 * w)]
        band = band if len(band) else p
        partial = x <= 2 or y <= 2 or x + w >= size[0] - 2 or y + h >= size[1] - 2
        blobs.append(Blob(label, int(cv2.contourArea(c)), (x, y, w, h), (cu, cv_),
                          (float(cu), float(band[:, 1].max())), (float(cu), float(band[:, 1].min())),
                          c_dist, partial))
    return blobs
