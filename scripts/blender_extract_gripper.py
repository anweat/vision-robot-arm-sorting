#!/usr/bin/env python3
"""Extract the gripper's four-bar geometry from the course link5 mesh into config/gripper_geometry.json.

link5.STL holds the whole gripper as 62 separate shells. They are sorted here into
  * the two fingers (with the pins that ride on them),
  * the upper (idler) links and the lower (gear) links, which turn about pivots on the gripper body,
  * everything else, which does not move with servo 6 (body plate, gears' axles, the SG-5010 servo).
The pivots are the centres of the pin shells. The two links of a side are equal and parallel, i.e. each
finger is the coupler of a parallelogram: it does not turn, it translates on a circle of the link length.

Output is in the URDF link5 frame (= dofbot_kinematics.servo5_pose up to the J5 sign and a 2.15 mm offset,
see src/spatial/gripper_geometry.py), metres. Point clouds are voxelized: they are collision samples,
not a mesh.

    /Applications/Blender.app/Contents/MacOS/Blender -b data/sim/dofbot_arm.blend \
        -P scripts/blender_extract_gripper.py -- [--out config/gripper_geometry.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
# approximate pivot centres (mm, x/z in the link5 frame), only used to pick the pin shells
APPROX = {
    "right": dict(g_upper=(4.4, 51.6), f_upper=(18.0, 78.3), f_lower=(26.3, 60.1)),
    "left": dict(g_upper=(-4.6, 51.6), f_upper=(-17.9, 78.5), f_lower=(-26.1, 60.2)),
}
VOXEL_MM = dict(finger=3.0, link=4.0, static=6.0)


def loose_parts(obj) -> list[np.ndarray]:
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    parent = np.arange(len(bm.verts))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for e in bm.edges:
        a, b = find(e.verts[0].index), find(e.verts[1].index)
        if a != b:
            parent[a] = b
    roots = np.array([find(i) for i in range(len(bm.verts))])
    co = np.array([v.co[:] for v in bm.verts]) * 1000.0          # mm
    bm.free()
    return [co[roots == r] for r in np.unique(roots)]


def voxel(points: np.ndarray, size: float) -> np.ndarray:
    _, idx = np.unique(np.round(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(idx)]


def box_contains(p: np.ndarray, xz, margin=3.0) -> bool:
    lo, hi = p.min(0), p.max(0)
    return lo[0] - margin <= xz[0] <= hi[0] + margin and lo[2] - margin <= xz[1] <= hi[2] + margin


def centre_xz(p: np.ndarray) -> tuple[float, float]:
    lo, hi = p.min(0), p.max(0)
    return (lo[0] + hi[0]) / 2.0, (lo[2] + hi[2]) / 2.0


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "config" / "gripper_geometry.json"))
    args = ap.parse_args(argv)

    parts = loose_parts(bpy.data.objects["dofbot_link5"])
    used = set()
    out_sides = {}
    clouds = {}
    for side, sign in (("right", 1.0), ("left", -1.0)):
        a = APPROX[side]
        fingers = [i for i, p in enumerate(parts) if p[:, 2].max() > 100.0 and np.sign(p[:, 0].mean()) == sign]
        # pins: the big screw shells whose box centre sits on a pivot
        def pin_near(xz):
            best = min((i for i, p in enumerate(parts) if len(p) > 1000),
                       key=lambda i: math.dist(centre_xz(parts[i]), xz))
            return best
        pins = {k: pin_near(v) for k, v in a.items()}
        piv = {k: centre_xz(parts[i]) for k, i in pins.items()}
        # parallelogram closure gives the lower ground pivot (the gear axis), which has no pin shell of its own
        piv["g_lower"] = (piv["g_upper"][0] + piv["f_lower"][0] - piv["f_upper"][0],
                          piv["g_upper"][1] + piv["f_lower"][1] - piv["f_upper"][1])
        finger_pins = [i for i, p in enumerate(parts) if len(p) > 1000 and i not in fingers and
                       min(math.dist(centre_xz(p), piv["f_upper"]), math.dist(centre_xz(p), piv["f_lower"])) < 3.0]
        upper = [i for i, p in enumerate(parts) if i not in fingers and len(p) < 1000 and
                 box_contains(p, piv["g_upper"]) and box_contains(p, piv["f_upper"]) and
                 not box_contains(p, piv["g_lower"], margin=1.0)]
        lower = [i for i, p in enumerate(parts) if i not in fingers and len(p) < 1000 and i not in upper and
                 box_contains(p, piv["g_lower"]) and box_contains(p, piv["f_lower"]) and
                 abs(p[:, 0].mean()) > 2.0 and np.sign(p[:, 0].mean()) == sign]
        used.update(fingers + finger_pins + upper + lower)
        link = math.dist(piv["g_upper"], piv["f_upper"])
        dx, dz = piv["f_upper"][0] - piv["g_upper"][0], piv["f_upper"][1] - piv["g_upper"][1]
        theta0 = math.degrees(math.atan2(sign * dx, dz))
        f = np.concatenate([parts[i] for i in fingers])
        pad_band = f[(f[:, 2] > 89.0) & (np.abs(f[:, 0]) < np.abs(f[:, 0]).min() + 1.0)]
        out_sides[side] = dict(
            sign=sign,
            g_upper_m=[round(v / 1000, 6) for v in piv["g_upper"]],
            g_lower_m=[round(v / 1000, 6) for v in piv["g_lower"]],
            f_upper_m=[round(v / 1000, 6) for v in piv["f_upper"]],
            f_lower_m=[round(v / 1000, 6) for v in piv["f_lower"]],
            link_m=round(link / 1000, 6),
            link_lower_m=round(math.dist(piv["g_lower"], piv["f_lower"]) / 1000, 6),
            theta0_deg=round(theta0, 3),
            pad_inner_x_m=round(float(np.abs(pad_band[:, 0]).mean()) / 1000, 6),
            pad_z_range_m=[round(float(pad_band[:, 2].min()) / 1000, 6), round(float(f[:, 2].max()) / 1000, 6)],
            tip_z_m=round(float(f[:, 2].max()) / 1000, 6),
            finger_y_range_m=[round(float(f[:, 1].min()) / 1000, 6), round(float(f[:, 1].max()) / 1000, 6)],
            shells=dict(finger=fingers, finger_pins=finger_pins, upper_link=upper, lower_link=lower),
        )
        clouds[f"finger_{side}"] = voxel(np.concatenate([f] + [parts[i] for i in finger_pins]), VOXEL_MM["finger"])
        clouds[f"upper_{side}"] = voxel(np.concatenate([parts[i] for i in upper]), VOXEL_MM["link"])
        clouds[f"lower_{side}"] = voxel(np.concatenate([parts[i] for i in lower]), VOXEL_MM["link"])
    static = [i for i in range(len(parts)) if i not in used]
    clouds["static"] = voxel(np.concatenate([parts[i] for i in static]), VOXEL_MM["static"])

    doc = {
        "_status": "Generated by scripts/blender_extract_gripper.py from the course link5.STL (via data/sim/dofbot_arm.blend). "
                   "Model geometry, not a measurement of this arm; see docs/atlas-200i-dk-a2/29-夹爪四连杆与抓取解算.md.",
        "_frame": "URDF link5 frame: origin on the J5 axis, z towards the fingertips, x = jaw opening axis, metres. "
                  "Each finger is the coupler of a parallelogram (upper idler + lower gear link, equal and parallel): "
                  "it translates by link*(sin(theta)-sin(theta0), cos(theta)-cos(theta0)) in (sign*x, z); theta is the "
                  "link angle from +z, opening outwards.",
        "voxel_mm": VOXEL_MM,
        "sides": out_sides,
        "static_shells": static,
        "points_m": {k: [[round(float(c) / 1000, 5) for c in p] for p in v] for k, v in clouds.items()},
    }
    Path(args.out).write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    for side, s in out_sides.items():
        print(side, {k: v for k, v in s.items() if k != "shells"}, {k: len(v) for k, v in s["shells"].items()})
    print({k: len(v) for k, v in clouds.items()}, "static shells", len(static))
    return 0


if __name__ == "__main__":
    sys.exit(main())
