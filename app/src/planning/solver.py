"""Pad-centre TCP inverse kinematics and wrist-roll alignment in the real link5 frame."""

from __future__ import annotations

import math
from typing import Sequence

from . import config as C
from . import gripper as GR
import dofbot_kinematics as kin   # noqa: E402
import gripper_geometry as gg     # noqa: E402


def _ik(x: float, y: float, z: float, pitch_deg: float) -> list[float] | None:
    """dofbot_kinematics.inverse without its 0.1 deg rounding (the TCP iteration below needs a smooth map)."""
    r = math.hypot(x, y)
    s1 = 90 + math.degrees(math.atan2(-x, y)) if r > 1e-9 else 90.0
    pitch = math.radians(pitch_deg)
    reach = kin.WRIST_TO_SERVO5 + kin.GRIPPER
    wr = r - reach * math.sin(pitch)
    wz = z - reach * math.cos(pitch) - kin.BASE_TO_SHOULDER
    cos_bend = (wr * wr + wz * wz - kin.UPPER_ARM ** 2 - kin.FOREARM ** 2) / (2 * kin.UPPER_ARM * kin.FOREARM)
    if not -1 <= cos_bend <= 1:
        return None
    bend = math.acos(cos_bend)
    p2 = math.atan2(wr, wz) - math.atan2(kin.FOREARM * math.sin(bend), kin.UPPER_ARM + kin.FOREARM * math.cos(bend))
    return [s1, 90 - math.degrees(p2), 90 - math.degrees(bend), 90 - math.degrees(pitch - p2 - bend)]


def solve_tcp(target: Sequence[float], pitch: float, j5: float, delta: float,
              floor: float | None = None, iterations: int = 12) -> list[float] | None:
    """J1-J4 that put the middle of the pads (at link angle `delta`) on `target` at this pitch and J5.

    Iterates FK-tip IK on the residual (the pad centre is not on the FK axis' 12 cm point and moves with J5).
    The first guess already moves the FK tip out along the tool axis by the pads' depth, and the joint floor
    is checked on the converged pose only: before 2026-11-15 it was checked on that first guess, ~2 cm off,
    which refused most near-base and steep poses whose solution is inside the limits."""
    floor = min(C.LIMITS[j][0] for j in (2, 3, 4)) if floor is None else floor
    c = gg.pad_centre_link(delta)
    p = math.radians(pitch)
    r = math.hypot(target[0], target[1]) or 1.0
    ux, uy = target[0] / r, target[1] / r
    depth = kin.GRIPPER - c[2]          # the FK tip lies this far beyond the pads along the tool axis
    tx = target[0] + depth * math.sin(p) * ux
    ty = target[1] + depth * math.sin(p) * uy
    tz = target[2] + depth * math.cos(p)
    for _ in range(iterations):
        q = _ik(tx, ty, tz, pitch)
        if q is None:
            return None
        got = gg.to_base(GR.link5(list(q) + [j5]), c)
        err = [target[i] - got[i] for i in range(3)]
        if max(abs(e) for e in err) < 1e-4:
            if min(q[1:4]) < floor or q[1] < C.LIMITS[2][0]:
                return None
            return [round(float(v), 2) for v in q]
        tx, ty, tz = tx + err[0], ty + err[1], tz + err[2]
    return None


def pitch_range(target: Sequence[float], j5: float, delta: float, lo: float = 80.0, hi: float = 200.0,
                step: float = 1.0) -> list[float]:
    """Every pitch (deg, `step` apart) at which the pads reach `target` inside the joint limits: the
    approach angles open at this spot, found by solving rather than taken from a fixed list. Near the base
    only steep ones (up to ~188 with J4 at its floor) remain, far out only flat ones (down to ~100)."""
    out, p = [], lo
    while p <= hi + 1e-9:
        q = solve_tcp(target, p, j5, delta)
        if q is not None and all(C.LIMITS[j][0] <= q[j - 1] <= C.LIMITS[j][1] for j in (1, 2, 3, 4)):
            out.append(round(p, 3))
        p += step
    return out


def spread(feasible: Sequence[float], prefer: float, step: float, limit: int) -> list[float]:
    """Pitches to try out of `feasible`: every `step` degrees, both ends kept, nearest `prefer` first."""
    if not feasible:
        return []
    picked = [feasible[0], feasible[-1]]
    picked += [p for p in feasible if abs((p - feasible[0]) % step) < 1e-6 or abs((p - feasible[0]) % step - step) < 1e-6]
    picked += [min(feasible, key=lambda p: abs(p - prefer))]
    return sorted(set(picked), key=lambda p: abs(p - prefer))[:limit]


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


def roll_candidates(q4: Sequence[float], yaw_deg: float | None, step: float = 0.5,
                    squaring_deg: float = 0.0) -> list[dict]:
    """J5 values that square the jaw to faces at bearing `yaw_deg` (mod 90) at this arm pose, best first.

    Each local minimum of the folded residual over the J5 limits is a candidate, so half and quarter turns
    both appear; each carries its residual and the jaw's tilt out of the table plane (at pitch p a quarter
    turn tilts it by up to 180 - p, and tilted pads meet a block's top and bottom edges).

    `squaring_deg` > 0 adds the J5 values that keep the jaw level (least tilt, each half turn) when their
    residual is within that many degrees, marked `squares`: the closing pads turn the block square to
    themselves (the real arm held blocks 36-39 deg off, doc 34 §8). They go right after the aligned, level
    candidates -- for a block whose squared jaw would tilt one finger into the table."""
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
    if squaring_deg > 0:
        level = []
        for i, (j5, r, t) in enumerate(rows):          # local minima of the tilt
            left = rows[i - 1][2] if i else math.inf
            right = rows[i + 1][2] if i + 1 < len(rows) else math.inf
            if t <= left and t <= right and r <= squaring_deg and all(abs(c["j5"] - j5) > 2 for c in out + level):
                level.append(dict(j5=round(j5, 1), residual_deg=r, tilt_deg=t, squares=True))
        level.sort(key=lambda c: (c["tilt_deg"], c["residual_deg"], abs(c["j5"] - C.ROLL_ZERO)))
        at = sum(1 for c in out if c["residual_deg"] <= tol and c["tilt_deg"] <= tilt)
        out[at:at] = level[:2]
    return out
