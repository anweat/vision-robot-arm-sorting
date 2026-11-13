"""Wrist-roll (J5) alignment for a square block, with the pitch projection the console's formula misses.

The console (src/tasks/console_grasp.py `grip_roll`) assumes the jaw-opening axis turns 1 deg in the table
plane per deg of J5:
    want = ROLL_ZERO + ROLL_SIGN * (block_yaw - (J1 - 90))
That is exact only when the last link is vertical (pitch 180). At pitch p the link is tilted by
t = 180 - p, and the jaw axis (link5 x) projects as
    phi = (J1 - 90) + atan(cos t * tan r),      r = ROLL_SIGN * (J5 - ROLL_ZERO)
so J5 has to overshoot: r = atan(tan(phi - (J1 - 90)) / cos t). At pitch 150 the gap is up to ~4 deg; at
135 up to ~10 deg, and a 10 deg error already eats the 12 deg grasp tolerance. The same projection
shrinks the *tangential* part of the finger-midpoint offset by cos t; the console compensates the radial
part with cos t but the tangential part with cos t too, i.e. under-corrects it by up to 3.4 mm at pitch
135. And the roll must use the J1 that IK actually returns, not the block bearing (they differ by 1-3 deg
once the tip target is offset).

This module is the offline solver for the proposed replacement path. It scans J5 over the safe window,
iterates the offset -> IK -> J1 coupling, and scores each candidate by
    residual = |projected jaw bearing - block yaw| folded to +/-45 deg
    tilt     = out-of-plane angle of the jaw axis (0 = fingers close horizontally, face contact;
               at pitch 150 a 90 deg roll tilts them by 30 deg, which meets the block's top/bottom edges)
so the caller can demand residual <= a few degrees *and* tilt <= its budget, instead of only asking
whether a J5 exists.

Conventions are the calibrated ones, not the Isaac link5 frame: the link5 x axis from `servo5_pose` is
mirrored against the direction verified on this arm (2026-09-26, red block, J1 71, block yaw -10.9, J5
100, operator: fingers parallel), so the projection above uses ROLL_SIGN = +1 and the verified zero 92.
The tangential offset sign still comes from config (`finger_offset_tangent_sign`) and is unverified; one
3-pose test (docs/27) should decide it before this replaces the console path.

Nothing here moves hardware; it is pure arithmetic plus IK. Not wired into console_grasp yet.

    python3 src/tasks/grip_solver.py --bearing 71 --radius 21 --yaw -10.9 --pitch 150
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial",):
    sys.path.insert(0, str(ROOT / "src" / sub))

import numpy as np  # noqa: E402
import dofbot_kinematics as kin  # noqa: E402

CONFIG = json.loads((ROOT / "config" / "arm_limits.json").read_text())["gripper"]
ROLL_ZERO = float(CONFIG.get("roll_zero_deg", 92))
ROLL_SIGN = float(CONFIG.get("roll_sign", 1))
RADIAL_OFFSET_M = float(CONFIG.get("open_grasp_radial_offset_cm", 1.0)) / 100
TANGENT_SIGN = float(CONFIG.get("finger_offset_tangent_sign", 1))
J5_RANGE = (90.0, 180.0)
PITCH3D = 30.0                         # pitch at which RADIAL_OFFSET was measured (docs/05 2026-09-26)
LATERAL_OFFSET_M = RADIAL_OFFSET_M / math.cos(math.radians(PITCH3D))   # ~1.15 cm, 3D lateral offset
TABLE_Z = 0.0
GRASP_HEIGHT = 0.012


def fold90(angle: float) -> float:
    return (angle + 45.0) % 90.0 - 45.0


def jaw_bearing(j1: float, j5: float, pitch: float) -> float:
    """Table-plane bearing of the jaw-opening axis for a given J1/J5/pitch, degrees."""
    tilt = math.radians(180.0 - pitch)
    roll = math.radians(ROLL_SIGN * (j5 - ROLL_ZERO))
    return (j1 - 90.0) + math.degrees(math.atan2(math.cos(tilt) * math.sin(roll), math.cos(roll)))


def out_of_plane_deg(j5: float, pitch: float) -> float:
    """How far the jaw axis is out of the table plane (0 = horizontal fingers, face contact)."""
    tilt = math.radians(180.0 - pitch)
    roll = math.radians(ROLL_SIGN * (j5 - ROLL_ZERO))
    return math.degrees(math.asin(min(1.0, abs(math.sin(tilt) * math.sin(roll)))))


def finger_offset(bx: float, by: float, j5: float, pitch: float) -> tuple[float, float]:
    """Tip target that puts the *projected* finger midpoint on (bx, by); fixes the console's tangential term."""
    tilt = math.radians(180.0 - pitch)
    roll = math.radians(ROLL_SIGN * (j5 - ROLL_ZERO))
    radius = math.hypot(bx, by)
    if radius < 1e-6:
        return bx, by
    ux, uy = bx / radius, by / radius
    radial = LATERAL_OFFSET_M * math.cos(roll) * math.cos(tilt)
    tangent = LATERAL_OFFSET_M * math.sin(roll) * TANGENT_SIGN
    return bx + radial * ux - tangent * uy, by + radial * uy + tangent * ux


def _ik(tx: float, ty: float, z: float, pitch: float, floor: int = -3):
    q = kin.inverse(tx, ty, z, pitch)
    if q is None or min(q[1:4]) < floor:
        return None
    return [float(v) for v in q]


def solve(bx: float, by: float, block_yaw_deg: float, pitch: float, z: float = TABLE_Z + GRASP_HEIGHT,
          max_tilt_deg: float = 15.0, step: float = 0.5) -> dict | None:
    """Best J5 at one pitch, or None when the block is unreachable at all at that pitch.

    The best candidate is the smallest residual among those with tilt <= max_tilt_deg; if none meets the
    tilt budget, the smallest-tilt candidate is returned with `meets_tilt=False`, never silently.
    """
    candidates = []
    for j5 in np.arange(J5_RANGE[0], J5_RANGE[1] + 1e-9, step):
        tx, ty = bx, by
        q = None
        for _ in range(3):
            q = _ik(tx, ty, z, pitch)
            if q is None:
                break
            tx, ty = finger_offset(bx, by, j5, pitch)
        if q is None or _ik(tx, ty, z, pitch) is None:
            continue
        residual = abs(fold90(jaw_bearing(q[0], j5, pitch) - block_yaw_deg))
        tilt = out_of_plane_deg(j5, pitch)
        candidates.append(dict(j5=float(j5), residual_deg=residual, tilt_deg=tilt,
                               tip_target_cm=[round(tx * 100, 2), round(ty * 100, 2)],
                               servos=[round(v, 2) for v in (q[:4] + [j5])], j1_deg=round(q[0], 2)))
    if not candidates:
        return None
    within = [c for c in candidates if c["tilt_deg"] <= max_tilt_deg]
    pool = within or candidates
    best = min(pool, key=lambda c: (c["residual_deg"], c["tilt_deg"]))
    best = dict(best, meets_tilt=bool(within), pitch=pitch)
    return best


def solve_best(bx: float, by: float, block_yaw_deg: float, pitches=(150, 155, 145, 160, 140, 135, 165, 170),
               prefer: float = 150.0, max_tilt_deg: float = 15.0, **kwargs) -> dict | None:
    """Best (pitch, J5) over the allowed pitches: residual and tilt first, then closeness to `prefer`."""
    best = None
    for pitch in pitches:
        found = solve(bx, by, block_yaw_deg, pitch, max_tilt_deg=max_tilt_deg, **kwargs)
        if found is None:
            continue
        key = (found["residual_deg"] + 0.25 * max(0.0, found["tilt_deg"] - max_tilt_deg),
               not found["meets_tilt"], abs(pitch - prefer))
        if best is None or key < best[0]:
            best = (key, found)
    return None if best is None else best[1]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Offline J5 alignment check (no hardware).")
    parser.add_argument("--bearing", type=float, default=90.0, help="block position angle in the base frame, deg")
    parser.add_argument("--radius", type=float, default=22.0, help="block distance from the base axis, cm")
    parser.add_argument("--yaw", type=float, default=0.0, help="block face yaw in the base frame, deg")
    parser.add_argument("--pitch", type=float, default=150.0)
    parser.add_argument("--tilt", type=float, default=15.0, help="allowed jaw-axis out-of-plane angle, deg")
    args = parser.parse_args(argv)
    bx = args.radius / 100 * math.cos(math.radians(args.bearing))
    by = args.radius / 100 * math.sin(math.radians(args.bearing))
    print(json.dumps(solve_best(bx, by, args.yaw, (args.pitch,), max_tilt_deg=args.tilt), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
