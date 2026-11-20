"""Geometry of the platform (chassis + black lid) from config/platform_model.json.

A rounded square seen from above (corner radius `outline.corner_radius_m`) whose top edge rolls down
with radius `lid.edge_radius_m`, top at `top_z_m`, in the forward-kinematics base frame. Used by the
Blender scene (dev/scripts/blender_arm_sim.py), the hardware-log check (dev/scripts/blender_platform_check.py),
the control keep-out (src/spatial/workspace.py, with keep_out.platform_shape = "rounded_square") and tests.

`depth(x, y, z)` is how far a point is inside the platform (metres, > 0 inside, < 0 outside, the
negative value is only exact near the boundary). Works on floats, or on numpy arrays when numpy is
available.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

try:
    import numpy as _np
except ImportError:          # plain python (unit tests without numpy)
    _np = None

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "config" / "platform_model.json"


def load(path: Path = PATH) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _is_array(v) -> bool:
    return _np is not None and isinstance(v, _np.ndarray)


def inside_distance_2d(x, y, cfg: dict):
    """Distance inside the rounded-square outline seen from above (> 0 inside)."""
    o = cfg["outline"]
    cx, cy = 0.0, (o["front_y_m"] + o["back_y_m"]) / 2.0
    hx, hy = o["half_width_m"], (o["front_y_m"] - o["back_y_m"]) / 2.0
    r = o["corner_radius_m"]
    if _is_array(x) or _is_array(y):
        qx = _np.abs(x - cx) - (hx - r)
        qy = _np.abs(y - cy) - (hy - r)
        outside = _np.hypot(_np.maximum(qx, 0.0), _np.maximum(qy, 0.0))
        return -(outside + _np.minimum(_np.maximum(qx, qy), 0.0) - r)
    qx = abs(x - cx) - (hx - r)
    qy = abs(y - cy) - (hy - r)
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    return -(outside + min(max(qx, qy), 0.0) - r)


def depth(x, y, z, cfg: dict):
    """Depth of (x, y, z) inside the lid/chassis solid, rolled top edge included."""
    top, e = cfg["top_z_m"], cfg["lid"]["edge_radius_m"]
    a = inside_distance_2d(x, y, cfg)
    if _is_array(a) or _is_array(z):
        plain = _np.minimum(a, top - z)
        if e <= 0:
            return plain
        zone = (a < e) & (z > top - e)
        rolled = e - _np.hypot(e - a, z - (top - e))
        return _np.where(zone, rolled, plain)
    if e > 0 and a < e and z > top - e:
        return e - math.hypot(e - a, z - (top - e))
    return min(a, top - z)


def edge_distance(j1_deg: float, cfg: dict, z: float | None = None, step: float = 0.0005) -> float:
    """Horizontal distance from the J1 axis to the platform surface along the arm's azimuth at
    servo-1 angle `j1_deg`, at height z (default: just under the flat lid top)."""
    if z is None:
        z = cfg["top_z_m"] - 1e-6
    az = math.radians(j1_deg - 90.0)
    d = 0.0
    while d < 1.0:
        if depth(-d * math.sin(az), d * math.cos(az), z, cfg) < 0:
            return d
        d += step
    return d


def outline_polygon(cfg: dict, inset: float = 0.0, arc_segments: int = 12) -> list[tuple[float, float]]:
    """Counter-clockwise outline (x, y), shrunk by `inset` (for the flat top of the lid)."""
    o = cfg["outline"]
    w, f, b = o["half_width_m"] - inset, o["front_y_m"] - inset, o["back_y_m"] + inset
    r = max(o["corner_radius_m"] - inset, 0.0)
    centres = [(w - r, b + r, -90.0), (w - r, f - r, 0.0), (-w + r, f - r, 90.0), (-w + r, b + r, 180.0)]
    pts = []
    for cx, cy, a0 in centres:
        for i in range(arc_segments + 1):
            a = math.radians(a0 + 90.0 * i / arc_segments)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    out = []
    for p in pts:
        if not out or math.dist(out[-1], p) > 1e-9:
            out.append(p)
    if math.dist(out[0], out[-1]) <= 1e-9:
        out.pop()
    return out
