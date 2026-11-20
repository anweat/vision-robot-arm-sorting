"""Baseline detector for the coloured 3 cm blocks on the printed map.

This finds saturated colour blobs (red / yellow / green / blue block bodies) and is a
stand-in until the sticker model exists; the colour label is NOT a garbage category.

Position, two cases (2026-09-24 blocks carry stickers on some faces, colour on others):
- colour on top: the blob outline projected onto the block-top plane covers about one
  block face, so its centroid on that plane is the block centre;
- sticker on top (colour only on the sides): the lowest blob pixel is the near bottom
  edge (table, Z = 0) and the highest the far top edge (Z = block height); the midpoint
  of the two, each intersected with its own plane, is the centre.
Blobs touching the image border are flagged partial and should not be used to locate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

HUES = (("red", 0, 10), ("yellow", 18, 38), ("green", 40, 85), ("blue", 90, 130), ("red", 165, 180))


@dataclass
class Blob:
    label: str
    area: int
    bbox: tuple[int, int, int, int]
    centroid_px: tuple[float, float]
    bottom_px: tuple[float, float]
    top_px: tuple[float, float]
    contour: np.ndarray = field(repr=False, default=None)
    partial: bool = False
    map_xy: Optional[tuple[float, float]] = None
    method: str = ""
    area_m2: float = 0.0
    size_m: float = 0.0


def hue_label(h: float) -> str:
    for name, lo, hi in HUES:
        if lo <= h <= hi:
            return name
    return "unknown"


def white_balance(image: np.ndarray) -> np.ndarray:
    """Scale channels so the brightest 10 % (the map paper) comes out neutral; undoes warm lamps."""
    flat = image.reshape(-1, 3).astype(np.float32)
    bright = flat[flat.sum(axis=1) >= np.percentile(flat.sum(axis=1), 90)]
    gains = bright.mean() / np.maximum(bright.mean(axis=0), 1.0)
    return np.clip(image.astype(np.float32) * gains, 0, 255).astype(np.uint8)


def detect_blobs(image: np.ndarray, min_area: int = 350, sat: int = 90, val: int = 50,
                 exclude: Optional[list[np.ndarray]] = None, per_hue: bool = False) -> list[Blob]:
    """exclude: pixel polygons to ignore, e.g. the map markers (grown slightly).
    per_hue: find each colour class separately, so touching blocks of different colours (and a warm
    background) do not merge into one blob (2026-09-26: red + yellow + table read as one 'red')."""
    hsv = cv2.cvtColor(white_balance(image), cv2.COLOR_BGR2HSV)
    base = cv2.inRange(hsv, (0, sat, val), (180, 255, 255))
    for poly in exclude or []:
        centre = poly.mean(axis=0)
        cv2.fillConvexPoly(base, (centre + (poly - centre) * 1.3).astype(np.int32), 0)
    if per_hue:
        masks = []
        for name in dict.fromkeys(n for n, _, _ in HUES):
            m = np.zeros_like(base)
            for n, lo, hi in HUES:
                if n == name:
                    m |= cv2.inRange(hsv[..., 0], lo, hi)
            masks.append((name, base & m))
    else:
        masks = [(None, base)]
    blobs = []
    for forced, mask in masks:
        blobs += _blobs_from_mask(mask, hsv, min_area, forced)
    return blobs


def _blobs_from_mask(mask: np.ndarray, hsv: np.ndarray, min_area: int, forced: Optional[str]) -> list[Blob]:
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blobs = []
    for c in contours:
        area = int(cv2.contourArea(c))
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        region = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(region, [c], -1, 255, -1)
        hues = hsv[..., 0][(region > 0) & (mask > 0)]
        # red wraps around 0/180; shift so the median is meaningful
        shifted = np.where(hues < 90, hues + 180, hues) if np.mean((hues < 12) | (hues > 165)) > 0.5 else hues
        label = forced or hue_label(float(np.median(shifted)) % 180)
        m = cv2.moments(c)
        cu, cv = m["m10"] / m["m00"], m["m01"] / m["m00"]
        pts = c.reshape(-1, 2)
        band = pts[np.abs(pts[:, 0] - cu) <= max(4, 0.2 * w)]  # central column of the blob
        if len(band) == 0:
            band = pts
        bottom = band[np.argmax(band[:, 1])]
        top = band[np.argmin(band[:, 1])]
        partial = x <= 2 or y <= 2 or x + w >= mask.shape[1] - 2 or y + h >= mask.shape[0] - 2
        blobs.append(Blob(label, area, (x, y, w, h), (cu, cv), (float(cu), float(bottom[1])),
                          (float(cu), float(top[1])), c, partial))
    return blobs


def locate(blob: Blob, pose, k: np.ndarray, dist: np.ndarray, height: float = 0.03) -> tuple[float, float]:
    """Block centre on the map (metres); see the module docstring for the two cases."""
    outline = np.array([pose.pixel_to_plane(p, k, dist, z=height)[:2] for p in blob.contour.reshape(-1, 2)])
    area = abs(cv2.contourArea(outline.astype(np.float32)))
    blob.area_m2 = float(area)
    if 0.5 * height ** 2 <= area <= 1.6 * height ** 2:
        m = cv2.moments(outline.astype(np.float32))
        centre = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        blob.method = "top-face"
    else:
        near_bottom = pose.pixel_to_plane(blob.bottom_px, k, dist, z=0.0)
        far_top = pose.pixel_to_plane(blob.top_px, k, dist, z=height)
        centre = (near_bottom[:2] + far_top[:2]) / 2
        blob.method = "edges"
    blob.map_xy = (float(centre[0]), float(centre[1]))
    # physical size: horizontal extent of the outline at mid-height, whatever face is visible
    mid = np.array([pose.pixel_to_plane(p, k, dist, z=height / 2)[:2] for p in blob.contour.reshape(-1, 2)])
    blob.size_m = float(max(np.ptp(mid[:, 0]), np.ptp(mid[:, 1])))
    return blob.map_xy


def draw(image: np.ndarray, blobs: list[Blob]) -> np.ndarray:
    out = image.copy()
    for b in blobs:
        x, y, w, h = b.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 1)
        for p, colour in ((b.bottom_px, (0, 0, 255)), (b.top_px, (255, 0, 0))):
            cv2.circle(out, (int(p[0]), int(p[1])), 4, colour, -1)
        text = b.label if b.map_xy is None else f"{b.label} {b.map_xy[0]*100:.1f},{b.map_xy[1]*100:.1f}cm {b.method}"
        if b.partial:
            text += " partial"
        cv2.putText(out, text, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return out
