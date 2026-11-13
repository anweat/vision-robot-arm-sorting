#!/usr/bin/env python3
"""Parametric model of the DOFBOT gripper: jaw opening, pad geometry, grasp point.

The data lives in ``config/gripper_model.json`` with a source and an evidence level on every
number; this module only turns it into geometry and interpolation.  Frame conventions follow
``dofbot_kinematics``: the link5 frame has its origin on the J5 axis, +z towards the fingertips,
+x along the jaw opening and +y lateral (towards the base at J5 = 90 deg).

What it answers:

  jaw_gap_m(angle)        how far apart the pads are for a commanded servo-6 angle
  servo6_for_gap(gap)     the least-open command that still clears a given width
  fits(width, angle)      can a block of this width sit between the pads at this angle
  pad_boxes(angle)        two boxes in the link frame, for collision or drawing
  grasp_point(servos)     where the block centre belongs, in the arm base frame
  envelope_points(angle)  sampled hull points, link frame

Nothing here is wired into the controller yet: the existing keep-out was fitted to contact data
and must not be replaced blind (see docs/atlas-200i-dk-a2/27-夹爪与整臂建模.md).  The module is
import-safe: stdlib only, no numpy, no bpy.

    python3 src/spatial/gripper_model.py table          # angle -> gap
    python3 src/spatial/gripper_model.py check 0.03 30  # would a 3 cm block fit at 30 deg
    python3 src/spatial/gripper_model.py grasp 107 60 26 30 92
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = ROOT / "config" / "gripper_model.json"
sys.path.insert(0, str(Path(__file__).resolve().parent))

import dofbot_kinematics as kin  # noqa: E402

BLOCK_3CM = 0.03


def load(path: Path | str = DEFAULT_MODEL) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _points(model: dict) -> list[tuple[float, float]]:
    """(servo angle deg, gap m), ascending angle == closing."""
    return [(float(a), float(g) / 1000.0) for a, g in model["jaw"]["points"]]


def jaw_gap_m(servo6_deg: float, model: dict | None = None) -> float:
    """Pad inner-face distance for a commanded servo-6 angle; clamped to the published range."""
    pts = _points(model or load())
    if servo6_deg <= pts[0][0]:
        return pts[0][1]
    if servo6_deg >= pts[-1][0]:
        return pts[-1][1]
    for (a0, g0), (a1, g1) in zip(pts, pts[1:]):
        if a0 <= servo6_deg <= a1:
            t = (servo6_deg - a0) / (a1 - a0)
            return g0 + t * (g1 - g0)
    return pts[-1][1]  # unreachable


def servo6_for_gap(gap_m: float, model: dict | None = None) -> float:
    """The smallest closing angle (i.e. least open) that still leaves `gap_m` between the pads."""
    pts = _points(model or load())
    gap_m = min(max(gap_m, pts[-1][1]), pts[0][1])
    for (a0, g0), (a1, g1) in zip(pts, pts[1:]):
        lo, hi = min(g0, g1), max(g0, g1)
        if lo <= gap_m <= hi:
            t = (gap_m - g0) / (g1 - g0) if g1 != g0 else 0.0
            return a0 + t * (a1 - a0)
    return pts[-1][0]


def fits(width_m: float, servo6_deg: float, margin_m: float = 0.0, model: dict | None = None) -> bool:
    return jaw_gap_m(servo6_deg, model) >= width_m + margin_m


def open_angle_for(width_m: float, margin_m: float = 0.01, model: dict | None = None) -> float:
    """Least-open command (largest angle) that clears `width_m` plus `margin_m`."""
    return servo6_for_gap(width_m + margin_m, model)


def pad_boxes(servo6_deg: float, model: dict | None = None) -> list[dict]:
    """The two finger pads as axis-aligned boxes in the link5 frame (metres).

    The Isaac four-bar is a parallelogram, so the pads stay parallel and only translate apart:
    the gap is the same along the whole pad, which is why one number describes it.
    """
    m = model or load()
    g = jaw_gap_m(servo6_deg, m)
    geo = m["geometry"]["pad"]
    half_t = geo["thickness_m"] / 2.0
    half_w = geo["width_m"] / 2.0
    z0, z1 = geo["z_range_from_j5_m"]
    boxes = []
    for sign, name in ((-1, "left"), (+1, "right")):
        x_inner = sign * g / 2.0
        x_outer = sign * (g / 2.0 + geo["thickness_m"])
        boxes.append({
            "name": name,
            "x_m": sorted([x_inner, x_outer]),
            "y_m": [-half_w, +half_w],
            "z_m": [z0, z1],
            # keep the half-thickness available for callers that grow the box by a margin
            "inner_face_x_m": x_inner,
            "half_thickness_m": half_t,
        })
    return boxes


def envelope_points(servo6_deg: float, model: dict | None = None) -> list[list[float]]:
    """Corner points of the finger pads plus the gripper housing, link5 frame, metres."""
    m = model or load()
    pts: list[list[float]] = []
    for box in pad_boxes(servo6_deg, m):
        for x in box["x_m"]:
            for y in box["y_m"]:
                for z in box["z_m"]:
                    pts.append([x, y, z])
    housing = m["envelope"]
    for x in (-housing["housing_half_width_m"], housing["housing_half_width_m"]):
        for y in (-housing["housing_half_width_m"], housing["housing_half_width_m"]):
            for z in housing["housing_z_range_m"]:
                pts.append([x, y, z])
    return pts


def _with_offset(pose: list[list[float]], offset: list[float]) -> list[float]:
    return [pose[i][3] + sum(pose[i][j] * offset[j] for j in range(3)) for i in range(3)]


def grasp_point(servos, model: dict | None = None, mode: str = "empirical") -> list[float]:
    """Block centre for a grasp, in the arm base frame, for servo angles 1-6.

    mode="empirical"  the point the verified console path uses: FK tip (12 cm) plus the measured
                      1.15 cm lateral offset (config/arm_limits.json open_grasp_radial_offset_cm).
    mode="geometric"  the middle of the real finger pads (10.66 cm from the J5 axis), no lateral
                      offset.  These two differ by ~1.5 cm; see the doc before switching.
    mode="fingertip"  the upstream FK tip, 12 cm, no correction (what most old code uses).
    """
    m = model or load()
    pose = kin.servo5_pose(list(servos)[:5])
    if mode == "empirical":
        offset = list(m["tcp"]["empirical_offset_link_m"])
        offset[2] = m["tcp"]["fk_tip_m"]
    elif mode == "geometric":
        offset = [0.0, 0.0, m["tcp"]["grasp_centre_m"]]
    elif mode == "fingertip":
        offset = [0.0, 0.0, m["tcp"]["fk_tip_m"]]
    else:
        raise ValueError(f"unknown mode {mode!r}")
    return _with_offset(pose, offset)


def summary(model: dict | None = None) -> dict:
    m = model or load()
    return {
        "max_gap_mm": round(jaw_gap_m(0.0, m) * 1000, 1),
        "min_gap_mm": round(jaw_gap_m(180.0, m) * 1000, 1),
        "gap_at_30_deg_mm": round(jaw_gap_m(30.0, m) * 1000, 1),
        "gap_at_11_deg_mm": round(jaw_gap_m(11.0, m) * 1000, 1),
        "grip_3cm_at_deg": round(servo6_for_gap(BLOCK_3CM, m), 1),
        "least_open_for_3cm_plus_1cm_deg": round(open_angle_for(BLOCK_3CM, 0.01, m), 1),
        "grasp_centre_m": m["tcp"]["grasp_centre_m"],
        "fk_tip_m": m["tcp"]["fk_tip_m"],
        "empirical_offset_m": m["tcp"]["empirical_offset_link_m"],
        "jaw_evidence": m["jaw"]["evidence"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("table", help="print the servo angle -> jaw gap curve")
    sub.add_parser("summary", help="one-screen summary of the model")
    chk = sub.add_parser("check", help="does a block of this width fit at this angle")
    chk.add_argument("width_m", type=float)
    chk.add_argument("angle_deg", type=float)
    chk.add_argument("--margin", type=float, default=0.0)
    gr = sub.add_parser("grasp", help="grasp point for five or six servo angles")
    gr.add_argument("servos", type=float, nargs="+")
    args = parser.parse_args()

    if args.cmd == "table":
        for angle in range(0, 181, 10):
            print(f"{angle:3d} deg  {jaw_gap_m(angle) * 1000:5.1f} mm")
    elif args.cmd == "summary":
        print(json.dumps(summary(), ensure_ascii=False, indent=1))
    elif args.cmd == "check":
        ok = fits(args.width_m, args.angle_deg, args.margin)
        print(json.dumps({
            "width_mm": args.width_m * 1000,
            "angle_deg": args.angle_deg,
            "gap_mm": round(jaw_gap_m(args.angle_deg) * 1000, 1),
            "margin_mm": args.margin * 1000,
            "fits": ok,
            "least_open_angle_deg": round(open_angle_for(args.width_m, args.margin), 1),
        }, ensure_ascii=False))
    elif args.cmd == "grasp":
        servos = list(args.servos) + [90.0] * (6 - len(args.servos))
        print(json.dumps({
            "servos": servos,
            "empirical": [round(v, 4) for v in grasp_point(servos, mode="empirical")],
            "geometric": [round(v, 4) for v in grasp_point(servos, mode="geometric")],
            "fingertip": [round(v, 4) for v in grasp_point(servos, mode="fingertip")],
        }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
