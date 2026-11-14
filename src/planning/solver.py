"""Pad-centre TCP inverse kinematics and wrist-roll alignment in the real link5 frame."""

from __future__ import annotations

import math
from typing import Sequence

from . import config as C
from . import gripper as GR
import dofbot_kinematics as kin   # noqa: E402
import gripper_geometry as gg     # noqa: E402


def solve_tcp(target: Sequence[float], pitch: float, j5: float, delta: float,
              floor: float | None = None, iterations: int = 8) -> list[float] | None:
    """J1-J4 that put the middle of the pads (at link angle `delta`) on `target` at this pitch and J5.
    Iterates FK-tip IK on the residual: the pad centre is not on the FK axis' 12 cm point and moves with J5."""
    floor = min(C.LIMITS[j][0] for j in (2, 3, 4)) if floor is None else floor
    c = gg.pad_centre_link(delta)
    tx, ty, tz = target
    for _ in range(iterations):
        q = kin.inverse(tx, ty, tz, pitch)
        if q is None or min(q[1:4]) < floor:
            return None
        got = gg.to_base(GR.link5(list(q) + [j5]), c)
        err = [target[i] - got[i] for i in range(3)]
        if max(abs(e) for e in err) < 2e-4:
            return [float(v) for v in q]
        tx, ty, tz = tx + err[0], ty + err[1], tz + err[2]
    return None


def tip(q4: Sequence[float]) -> list[float]:
    return kin.forward(list(q4)[:4])["tip"]


def pitch_of(q4: Sequence[float]) -> float:
    return kin.forward(list(q4)[:4])["tip_pitch_deg"][0]


def fold90(a: float) -> float:
    return (a + 45.0) % 90.0 - 45.0


def jaw_bearing_tilt(q5: Sequence[float]) -> tuple[float, float]:
    """Table-plane bearing of the jaw axis (deg from base +x) and its tilt out of the table plane (deg)."""
    a = GR.jaw_axis(q5)
    return math.degrees(math.atan2(a[1], a[0])), math.degrees(math.asin(min(1.0, abs(a[2]))))


def roll_candidates(q4: Sequence[float], yaw_deg: float | None, step: float = 0.5) -> list[dict]:
    """J5 values that square the jaw to faces at bearing `yaw_deg` (mod 90) at this arm pose, best first.

    Each local minimum of the folded residual over the J5 limits is a candidate, so half and quarter turns
    both appear; each carries its residual and the jaw's tilt out of the table plane (at pitch p a quarter
    turn tilts it by up to 180 - p, and tilted pads meet a block's top and bottom edges)."""
    lo, hi = C.LIMITS[5]
    if yaw_deg is None:
        b, t = jaw_bearing_tilt(list(q4) + [C.ROLL_ZERO])
        return [dict(j5=C.ROLL_ZERO, residual_deg=None, tilt_deg=t)]
    rows, j5 = [], lo
    while j5 <= hi + 1e-9:
        b, t = jaw_bearing_tilt(list(q4) + [j5])
        rows.append((j5, abs(fold90(b - yaw_deg)), t))
        j5 += step
    out = []
    for i, (j5, r, t) in enumerate(rows):
        left = rows[i - 1][1] if i else math.inf
        right = rows[i + 1][1] if i + 1 < len(rows) else math.inf
        if r <= left and r <= right:
            out.append(dict(j5=round(j5, 1), residual_deg=r, tilt_deg=t))
    tol, tilt = C.G["aligned_tolerance_deg"], C.G["max_jaw_tilt_deg"]
    out.sort(key=lambda c: (c["residual_deg"] > tol, c["tilt_deg"] > tilt,
                            c["residual_deg"] + 0.5 * c["tilt_deg"], abs(c["j5"] - C.ROLL_ZERO)))
    return out
