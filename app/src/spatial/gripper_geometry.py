"""The gripper as the four-bar it is: where the pads, the fingertips and the whole gripper are for any J6.

Geometry: config/gripper_geometry.json, extracted from the course link5.STL by
dev/scripts/blender_extract_gripper.py. Each finger is the coupler of a parallelogram (an idler link and a
gear link of 30.0 mm, equal and parallel), so it does not turn: it translates on a 30 mm circle. Opening
therefore pulls the pads back towards the wrist as well as apart -- 26 mm between "closed on a 3 cm block"
and "open at J6 30". That axial travel is what the old empirical "+1 cm radially for the open gripper"
was compensating (docs/current/29-夹爪四连杆与抓取解算.md).

J6 -> jaw gap comes from the official table (config/gripper_model.json, two points confirmed on this arm);
the gap -> link angle step is this geometry. Above the four-bar's 60.8 mm the table is clamped.

Frames. The URDF link5 frame of the mesh has the rotation of dofbot_kinematics.servo5_pose, but
  * J5 turns it the other way on the real arm: at J1 71 a block at yaw -10.9 was gripped square at J5 100
    (2026-09-26), which servo5_pose would turn by -8 deg, not +8. So the frame is servo5_pose at
    J5' = 90 - ROLL_SIGN * (J5 - ROLL_ZERO);
  * its origin sits 2.15 mm along -y of the pre-roll frame (URDF joint5 origin).
`link5_pose` returns that frame. Pure python (no numpy), so it runs on the board.
"""

from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Iterable, Sequence

import dofbot_kinematics as kin
import gripper_model

ROOT = Path(__file__).resolve().parents[2]
GEOMETRY = ROOT / "config" / "gripper_geometry.json"
ARM = ROOT / "config" / "arm_limits.json"
URDF_J5_OFFSET_Y = -0.00215
BLOCK = 0.03


@lru_cache(maxsize=None)
def load(path: str = str(GEOMETRY)) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _gripper_cfg() -> dict:
    return json.loads(ARM.read_text(encoding="utf-8"))["gripper"]


# ----------------------------------------------------------------------------------------- the four-bar

def _side_inner_x(side: dict, delta: float) -> float:
    """Signed x of a pad's inner face at link-angle change `delta` (deg, + opens)."""
    t0 = math.radians(side["theta0_deg"])
    t = t0 + math.radians(delta)
    return side["sign"] * (side["pad_inner_x_m"] + side["link_m"] * (math.sin(t) - math.sin(t0)))


def _side_dz(side: dict, delta: float) -> float:
    t0 = math.radians(side["theta0_deg"])
    return side["link_m"] * (math.cos(t0 + math.radians(delta)) - math.cos(t0))


def _side_dx(side: dict, delta: float) -> float:
    t0 = math.radians(side["theta0_deg"])
    return side["sign"] * side["link_m"] * (math.sin(t0 + math.radians(delta)) - math.sin(t0))


def gap_for_delta(delta: float, geo: dict | None = None) -> float:
    s = (geo or load())["sides"]
    return _side_inner_x(s["right"], delta) - _side_inner_x(s["left"], delta)


def delta_range(geo: dict | None = None) -> tuple[float, float]:
    """(closed, widest) link-angle changes: pads touching, and the four-bar's widest opening."""
    s = (geo or load())["sides"]
    widest = 90.0 - (s["right"]["theta0_deg"] + s["left"]["theta0_deg"]) / 2.0
    lo, hi = -60.0, widest
    for _ in range(60):                         # gap(delta) = 0
        mid = (lo + hi) / 2.0
        if gap_for_delta(mid, geo) > 0:
            hi = mid
        else:
            lo = mid
    return hi, widest


def max_gap(geo: dict | None = None) -> float:
    return gap_for_delta(delta_range(geo)[1], geo)


def delta_for_gap(gap: float, geo: dict | None = None) -> float:
    lo, hi = delta_range(geo)
    if gap >= gap_for_delta(hi, geo):
        return hi
    if gap <= 0:
        return lo
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if gap_for_delta(mid, geo) < gap:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def delta_for_j6(j6: float, geo: dict | None = None) -> float:
    """Link-angle change for a commanded J6 with nothing between the fingers."""
    return delta_for_gap(gripper_model.jaw_gap_m(j6), geo)


def pad_centre_link(delta: float, geo: dict | None = None) -> list[float]:
    """Middle between the two pads (link5 frame): where a block centre ends up when they close on it."""
    s = (geo or load())["sides"]
    x = (_side_inner_x(s["right"], delta) + _side_inner_x(s["left"], delta)) / 2.0
    y = sum((v["finger_y_range_m"][0] + v["finger_y_range_m"][1]) / 2.0 for v in s.values()) / 2.0
    z = sum((v["pad_z_range_m"][0] + v["pad_z_range_m"][1]) / 2.0 + _side_dz(v, delta) for v in s.values()) / 2.0
    return [x, y, z]


def pad_z_range_link(delta: float, geo: dict | None = None) -> tuple[float, float]:
    s = (geo or load())["sides"]
    lo = min(v["pad_z_range_m"][0] + _side_dz(v, delta) for v in s.values())
    hi = max(v["pad_z_range_m"][1] + _side_dz(v, delta) for v in s.values())
    return lo, hi


def fingertip_z_link(delta: float, geo: dict | None = None) -> float:
    s = (geo or load())["sides"]
    return max(v["tip_z_m"] + _side_dz(v, delta) for v in s.values())


def contact_delta(width: float = BLOCK, geo: dict | None = None) -> float:
    return delta_for_gap(width, geo)


def _rot_y(p, pivot, angle_deg):
    """Turn p about the y axis through `pivot` (x, z) by angle_deg, positive from +z towards +x."""
    a = math.radians(angle_deg)
    x, z = p[0] - pivot[0], p[2] - pivot[1]
    return [pivot[0] + x * math.cos(a) + z * math.sin(a), p[1], pivot[1] - x * math.sin(a) + z * math.cos(a)]


def points_link(delta: float, parts: Iterable[str] = ("fingers", "links", "static"),
                geo: dict | None = None) -> list[list[float]]:
    """Collision samples of the gripper at link-angle change `delta`, link5 frame."""
    g = geo or load()
    pts = g["points_m"]
    out: list[list[float]] = []
    parts = set(parts)
    for name, side in g["sides"].items():
        if "fingers" in parts:
            dx, dz = _side_dx(side, delta), _side_dz(side, delta)
            out += [[p[0] + dx, p[1], p[2] + dz] for p in pts[f"finger_{name}"]]
        if "links" in parts:
            turn = side["sign"] * delta
            out += [_rot_y(p, side["g_upper_m"], turn) for p in pts[f"upper_{name}"]]
            out += [_rot_y(p, side["g_lower_m"], turn) for p in pts[f"lower_{name}"]]
    if "static" in parts:
        out += [list(p) for p in pts["static"]]
    return out


# ------------------------------------------------------------------------------------------- frames

def roll_params() -> tuple[float, float]:
    g = _gripper_cfg()
    return float(g.get("roll_zero_deg", 92)), float(g.get("roll_sign", 1))


def link5_pose(servos: Sequence[float], roll_zero: float | None = None, roll_sign: float | None = None) -> list[list[float]]:
    """4x4 base-frame pose of the mesh's link5 frame for servo angles 1-5 (see the module note)."""
    if roll_zero is None or roll_sign is None:
        rz, rs = roll_params()
        roll_zero = rz if roll_zero is None else roll_zero
        roll_sign = rs if roll_sign is None else roll_sign
    q = list(servos)[:4]
    j5 = servos[4] if len(servos) > 4 and servos[4] is not None else roll_zero
    pose = kin.servo5_pose(q + [90.0 - roll_sign * (j5 - roll_zero)])
    pre = kin.servo5_pose(q + [90.0])
    for i in range(3):
        pose[i][3] += URDF_J5_OFFSET_Y * pre[i][1]
    return pose


def to_base(pose, p) -> list[float]:
    return [pose[i][0] * p[0] + pose[i][1] * p[1] + pose[i][2] * p[2] + pose[i][3] for i in range(3)]


def points_base(servos: Sequence[float], delta: float, parts=("fingers", "links", "static"),
                geo: dict | None = None) -> list[list[float]]:
    pose = link5_pose(servos)
    return [to_base(pose, p) for p in points_link(delta, parts, geo)]


def pad_centre_base(servos: Sequence[float], delta: float) -> list[float]:
    return to_base(link5_pose(servos), pad_centre_link(delta))


def jaw_axis_base(servos: Sequence[float]) -> list[float]:
    pose = link5_pose(servos)
    return [pose[0][0], pose[1][0], pose[2][0]]


def lowest_z(servos: Sequence[float], delta: float, parts=("fingers",)) -> float:
    return min(p[2] for p in points_base(servos, delta, parts))


# ------------------------------------------------------------------------------------- the table

def table_from_touches(poses: Sequence[Sequence[float]]) -> dict:
    """Table height in this model from fingertip touches (poses with J6): the lowest finger point of each.

    The touch calibration (spatial/touch_calibration.py) took the 12 cm FK tip as the touching point, which
    is 5-6 mm beyond the real closed fingertip; anchoring the model to the same touches keeps model heights
    and the real table consistent even if the mesh's fingers are a few mm off."""
    zs = [lowest_z(p, delta_for_j6(p[5])) for p in poses]
    mean = sum(zs) / len(zs)
    return dict(table_z_m=mean, per_touch_m=zs,
                spread_m=(sum((z - mean) ** 2 for z in zs) / max(len(zs) - 1, 1)) ** 0.5)
