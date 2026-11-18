#!/usr/bin/env python3
"""Planar FK/IK for the 5-joint Dofbot arm, in servo degrees as Arm_Lib uses them.

Geometry comes from the upstream URDF (资料/官方资料/Atlas-200I-DK-A2-机械臂/
ros2_robot_arm-upstream/ros2_ws/src/dofbot_moveit/urdf/dofbot.urdf, commit 9d10827)
and the 0.12 m gripper length hard-coded in upstream dofbot_server.cpp. Link
lengths are the upstream model, not measured on this arm.

Frame: origin at base_link, z up, +y is the arm's forward direction at servo 1 = 90,
units metres. Upstream maps servo angle s to joint angle (s - 90) deg; for servos 2-4
a positive joint angle tilts the arm backwards. On 2026-09-24 the real arm matched
this: raising servo 2 leaned it back, lowering servo 3 folded it forward.

Pitch is the direction of the last link (servo 4 to gripper tip) measured from
straight up, positive forward: 90 = horizontal, 180 = pointing straight down.
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Optional, Sequence

BASE_TO_SHOULDER = 0.064 + 0.0435
UPPER_ARM = 0.08285
FOREARM = 0.08285
WRIST_TO_SERVO5 = 0.07385
GRIPPER = 0.12


def _pitches(s2: float, s3: float, s4: float) -> tuple[float, float, float]:
    """Forward-from-vertical angles (rad) of upper arm, forearm and wrist link."""
    p2 = math.radians(90 - s2)
    p3 = p2 + math.radians(90 - s3)
    p4 = p3 + math.radians(90 - s4)
    return p2, p3, p4


def forward(servos: Sequence[float]) -> dict[str, list[float]]:
    """Joint positions in the base frame for servo angles 1-4 (5 and 6 do not move the tip)."""
    s1, s2, s3, s4 = servos[:4]
    p2, p3, p4 = _pitches(s2, s3, s4)
    yaw = math.radians(s1 - 90)
    points = {"shoulder": (0.0, BASE_TO_SHOULDER)}
    r, z = points["shoulder"]
    for name, length, pitch in (("elbow", UPPER_ARM, p2), ("wrist", FOREARM, p3),
                                ("servo5", WRIST_TO_SERVO5, p4), ("tip", GRIPPER, p4)):
        r, z = r + length * math.sin(pitch), z + length * math.cos(pitch)
        points[name] = (r, z)
    out = {name: [round(-r * math.sin(yaw), 4), round(r * math.cos(yaw), 4), round(z, 4)]
           for name, (r, z) in points.items()}
    out["tip_pitch_deg"] = [round(math.degrees(p4), 1)]
    return out


CAMERA_MOUNT_J5_DEG = 90


def camera_mount_pose(servos: Sequence[float]) -> list[list[float]]:
    """Frame the wrist camera is fixed to: the servo-5 frame with J5 held at 90 deg.

    The camera sits on the servo-5 housing, i.e. on the wrist link, and does not turn with J5
    (2026-09-25: map views at J5 = 90/119/149 gave the same camera offset only in this frame)."""
    return servo5_pose(list(servos)[:4] + [CAMERA_MOUNT_J5_DEG])


def servo5_pose(servos: Sequence[float]) -> list[list[float]]:
    """4x4 base-frame pose of a frame at the servo-5 axis centre, servo angles 1-5.

    z points along the last link towards the gripper tip, x is the pitch-joint axis
    before roll, and servo 5 rolls the frame about z by (s5 - 90) degrees. The roll
    zero is arbitrary but consistent, which is all hand-eye calibration needs."""
    s1, s2, s3, s4, s5 = servos[:5]
    _, _, p4 = _pitches(s2, s3, s4)
    yaw, roll = math.radians(s1 - 90), math.radians(s5 - 90)
    z = (0.0, math.sin(p4), math.cos(p4))
    x = (1.0, 0.0, 0.0)
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    xr = tuple(math.cos(roll) * a + math.sin(roll) * b for a, b in zip(x, y))
    yr = tuple(-math.sin(roll) * a + math.cos(roll) * b for a, b in zip(x, y))
    cy, sy = math.cos(yaw), math.sin(yaw)

    def rot(v):
        return (cy * v[0] - sy * v[1], sy * v[0] + cy * v[1], v[2])

    xr, yr, zr = rot(xr), rot(yr), rot(z)
    origin = forward(servos[:4])["servo5"]
    return [[xr[0], yr[0], zr[0], origin[0]], [xr[1], yr[1], zr[1], origin[1]],
            [xr[2], yr[2], zr[2], origin[2]], [0.0, 0.0, 0.0, 1.0]]


def inverse(x: float, y: float, z: float, pitch_deg: float,
            servo1_hint: Optional[float] = None) -> Optional[list[float]]:
    """Servo angles 1-4 putting the gripper tip at (x, y, z) with the given pitch; None if unreachable."""
    r = math.hypot(x, y)
    s1 = 90 + math.degrees(math.atan2(-x, y)) if r > 1e-9 else (servo1_hint or 90)
    pitch = math.radians(pitch_deg)
    reach = WRIST_TO_SERVO5 + GRIPPER
    wr = r - reach * math.sin(pitch)
    wz = z - reach * math.cos(pitch) - BASE_TO_SHOULDER
    d2 = wr * wr + wz * wz
    cos_bend = (d2 - UPPER_ARM ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * FOREARM)
    if not -1 <= cos_bend <= 1:
        return None
    bend = math.acos(cos_bend)  # forearm folds forward relative to the upper arm
    p2 = math.atan2(wr, wz) - math.atan2(FOREARM * math.sin(bend), UPPER_ARM + FOREARM * math.cos(bend))
    p3 = p2 + bend
    s2 = 90 - math.degrees(p2)
    s3 = 90 - math.degrees(bend)
    s4 = 90 - math.degrees(pitch - p3)
    return [round(s1, 1), round(s2, 1), round(s3, 1), round(s4, 1)]


def inverse_j2_fixed(s1: float, s2: float, x: float, y: float, z: float,
                     near: Sequence[float]) -> Optional[list[float]]:
    """Servos 3-4 (unrounded) putting the tip at (x, y, z) with servos 1-2 held at s1, s2; the pitch follows.
    Of the two bends, the one closer to `near` (servos 1-4). For the touch probe (doc 33): J2 moves in jumps
    of its 1.5-2 deg dead band, so a descent that leaves it alone is far steadier. None if out of reach or
    the tip is off the J1 ray."""
    r = math.hypot(x, y)
    p2 = math.radians(90 - s2)
    er, ez = UPPER_ARM * math.sin(p2), BASE_TO_SHOULDER + UPPER_ARM * math.cos(p2)
    a, b = FOREARM, WRIST_TO_SERVO5 + GRIPPER
    dr, dz = r - er, z - ez
    d2 = dr * dr + dz * dz
    c = (d2 - a * a - b * b) / (2 * a * b)
    if not -1 <= c <= 1:
        return None
    best = None
    for sign in (1, -1):
        bend = sign * math.acos(c)
        p3 = math.atan2(dr, dz) - math.atan2(b * math.sin(bend), a + b * math.cos(bend))
        p4 = p3 + bend
        q = [s1, s2, 90 - math.degrees(p3 - p2), 90 - math.degrees(p4 - p3)]
        cost = abs(q[2] - near[2]) + abs(q[3] - near[3])
        if best is None or cost < best[0]:
            best = (cost, q)
    return best[1]


def inverse_within(x: float, y: float, z: float, limits: dict[int, Sequence[float]],
                   pitches: Sequence[float] = tuple(range(180, 89, -1))) -> Optional[dict]:
    """First solution inside the joint limits, trying the most vertical approach first."""
    for pitch in pitches:
        servos = inverse(x, y, z, pitch)
        if servos and all(limits[j][0] <= a <= limits[j][1] for j, a in zip((1, 2, 3, 4), servos)):
            return {"servos": servos, "pitch_deg": pitch}
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    fk = sub.add_parser("fk", help="servo angles 1-4 -> positions")
    fk.add_argument("servos", type=float, nargs=4)
    ik = sub.add_parser("ik", help="tip position (m) -> servo angles 1-4")
    ik.add_argument("xyz", type=float, nargs=3)
    ik.add_argument("--pitch", type=float, help="fixed pitch in degrees; default searches 180..90")
    ik.add_argument("--config", help="arm_limits.json whose soft limits the search must respect")
    args = parser.parse_args()

    if args.command == "fk":
        print(json.dumps(forward(args.servos)))
    elif args.pitch is not None:
        print(json.dumps({"servos": inverse(*args.xyz, args.pitch), "pitch_deg": args.pitch}))
    else:
        limits = {j: (0, 180) for j in (1, 2, 3, 4)}
        if args.config:
            with open(args.config, encoding="utf-8") as f:
                limits = {int(k): v for k, v in json.load(f)["soft_limits_deg"].items()}
        print(json.dumps(inverse_within(*args.xyz, limits)))


if __name__ == "__main__":
    main()
