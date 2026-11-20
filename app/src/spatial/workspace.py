"""Keep-out checks for the gripper: the arm's own platform and the table.

Platform (refitted 2026-09-25): a box around the servo-1 axis, radius `platform_radius_m` (the
operator measured 8 cm from the turntable centre to the platform edge; the map, laid against that
edge, registers at y = 7.5-7.7 cm) and top `platform_min_z_m` (the 9 cm chassis over the table at
-1.42 cm gives +7.6 cm). The gripper is modelled as the gripper-servo housing, +-3.5 cm across over
the first 4 cm below the servo-5 axis (and 2 cm behind it), then the fingers, +-1.5 cm, down to the
tip. A hull point's margin is max(r - R, z - top) (its distance out of the box), and the platform
term must stay above `platform_margin_m`.

These housing sizes are fitted, in forward-kinematics coordinates, between every platform contact
recorded so far -- the hand-placed "servo edge on the frame" [93.4,95,-2,14.6] and the return
[76,86,7,7] -> [90,105,5,5] (2026-09-24), the console contact [103,72,19,3] (2026-09-25 00:50) and
the folded "servo resting on the platform" [90,102,-5,-1] (2026-09-25 19:00) -- which all come to
<= +0.1 cm, and every logged move that ran without contact (>= +0.6 cm; the near-strip refine views
[71,94,5,5] are the closest). The earlier cylinder (10.5 cm, above +4 cm, one +-2.5 cm band) put the
folded contact 3.7 cm inside and blocked the near-field views that ran safely.

Platform shape (2026-11-14): with `platform_shape` = "rounded_square" the same hull points are also
placed at the arm's azimuth (J1) and checked against the rounded-square lid of config/platform_model.json
(src/spatial/platform_model.py, from the operator's photo, corners fitted to the hardware log). The
platform term is the worse of the cylinder and the square, so the square only ever adds caution: it
matters away from J1 = 90, where the lid reaches 2-3 cm further out than the 8 cm cylinder. Both use
`platform_min_z_m` as the top, so the profiles that lower or disable it keep working.

The table term keeps that +-2.5 cm band (`hull_half_width_m`): `table_z_m` was fitted with it.
Joint-space moves are checked on interpolated samples, because a path can dip even when both ends
are clear.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence

import dofbot_kinematics as kin
import platform_model

ROOT = Path(__file__).resolve().parents[2]
DEFAULTS = {"platform_radius_m": 0.08, "platform_min_z_m": 0.076, "platform_margin_m": 0.004,
            "servo_half_width_m": 0.035, "servo_length_m": 0.04, "servo_back_m": 0.02,
            "finger_half_width_m": 0.015, "table_z_m": -0.045, "table_margin_m": 0.025,
            "hull_half_width_m": 0.025, "platform_shape": "cylinder"}


def load(path: Path = ROOT / "config" / "arm_limits.json") -> dict:
    return load_keep_out(json.loads(Path(path).read_text()))


def load_keep_out(arm_limits: dict) -> dict:
    cfg = dict(DEFAULTS)
    cfg.update({k: v for k, v in arm_limits.get("keep_out", {}).items() if not k.startswith("_")})
    if cfg["platform_shape"] == "rounded_square" and "platform_square" not in cfg:
        cfg["platform_square"] = platform_model.load()
    elif cfg["platform_shape"] not in ("cylinder", "rounded_square"):
        raise ValueError(f"unknown platform_shape {cfg['platform_shape']!r}")
    return cfg


def gripper_hull(servos: Sequence[float], half_width: float, samples: int = 13) -> list[tuple[float, float]]:
    """(horizontal distance from the servo-1 axis, height) of points around the gripper."""
    fk = kin.forward(list(servos)[:4])
    s5 = fk["servo5"]
    pitch = math.radians(fk["tip_pitch_deg"][0])
    r5 = math.hypot(s5[0], s5[1])
    along = (math.sin(pitch), math.cos(pitch))
    across = (math.cos(pitch), -math.sin(pitch))
    points = []
    for i in range(samples):
        a = kin.GRIPPER * i / (samples - 1)
        for b in (-half_width, 0.0, half_width):
            points.append((r5 + a * along[0] + b * across[0], s5[2] + a * along[1] + b * across[1]))
    return points


def platform_hull(servos: Sequence[float], cfg: dict, samples: int = 41) -> list[tuple[float, float]]:
    """(r, z) points of the gripper-servo housing and the fingers, in the plane of the arm."""
    fk = kin.forward(list(servos)[:4])
    s5 = fk["servo5"]
    pitch = math.radians(fk["tip_pitch_deg"][0])
    r5 = math.hypot(s5[0], s5[1])
    along = (math.sin(pitch), math.cos(pitch))
    across = (math.cos(pitch), -math.sin(pitch))
    back = cfg["servo_back_m"]
    points = []
    for i in range(samples):
        a = -back + (kin.GRIPPER + back) * i / (samples - 1)
        w = cfg["servo_half_width_m"] if a <= cfg["servo_length_m"] else cfg["finger_half_width_m"]
        for b in (-w, 0.0, w):
            points.append((r5 + a * along[0] + b * across[0], s5[2] + a * along[1] + b * across[1]))
    return points


def platform_clearance(servos: Sequence[float], cfg: dict) -> float:
    """Distance (m) of the gripper out of the platform, less the platform margin (worse of the
    cylinder and, if configured, the rounded-square lid)."""
    worst = float("inf")
    hull = platform_hull(servos, cfg)
    for r, z in hull:
        dr, dz = r - cfg["platform_radius_m"], z - cfg["platform_min_z_m"]
        worst = min(worst, math.hypot(dr, dz) if dr > 0 and dz > 0 else max(dr, dz))
    square = cfg.get("platform_square") if cfg["platform_shape"] == "rounded_square" else None
    if square is not None:
        square = dict(square, top_z_m=cfg["platform_min_z_m"])     # profiles move/disable the top
        yaw = math.radians(list(servos)[0] - 90.0)
        sx, cy = -math.sin(yaw), math.cos(yaw)
        for r, z in hull:
            worst = min(worst, -platform_model.depth(r * sx, r * cy, z, square))
    return worst - cfg["platform_margin_m"]


def clearance(servos: Sequence[float], cfg: dict) -> float:
    """Smallest margin in metres; negative means the pose violates a keep-out."""
    worst = platform_clearance(servos, cfg)
    for r, z in gripper_hull(servos, cfg["hull_half_width_m"]):
        worst = min(worst, z - cfg["table_z_m"] - cfg["table_margin_m"])
    return worst


def path_clearance(start: Sequence[float], goal: Sequence[float], cfg: dict, samples: int = 30) -> float:
    return min(clearance([a + (b - a) * i / samples for a, b in zip(start[:4], goal[:4])], cfg)
               for i in range(samples + 1))
