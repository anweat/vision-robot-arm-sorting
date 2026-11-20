#!/usr/bin/env python3
"""Check a platform model against every hardware pose on record, using the real link meshes.

The arm's link3-link5 STL meshes (gripper and gripper servo included, from data/sim/dofbot_arm.blend)
are placed by the URDF chain at each readback found in data/**/*.jsonl (hardware mode only) and at
the recorded platform contacts. For a platform shape the script reports how deep the worst
contact-free pose goes into it and how shallow the shallowest contact is. A shape is consistent
with the hardware when every contact-free pose stays outside and every contact is at or inside the
boundary, both within `--tol-cm` (FK and mesh error are a few mm).

Shapes checked:
  * keep-out cylinder (app/config/arm_limits.json keep_out: radius, top), the one control uses;
  * app/config/platform_model.json (rounded square lid with a rolled top edge, app/src/spatial/platform_model.py);
  * the same square with sharp corners and edges, for comparison.

    /Applications/Blender.app/Contents/MacOS/Blender -b data/sim/dofbot_arm.blend \
        -P dev/scripts/blender_platform_check.py -- [--tol-cm 0.3] [--json out.json]

Exit code 1 when app/config/platform_model.json is not consistent with the hardware within the tolerance.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dev" / "spatial"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))
import platform_model  # noqa: E402

KEEP_OUT = json.loads((ROOT / "app" / "config" / "arm_limits.json").read_text(encoding="utf-8"))["keep_out"]
PLATFORM = platform_model.load()

# recorded platform contacts (readbacks), same list as app/tests/test_safe_arm.py KeepOutTest.CONTACT;
# for the 2026-09-24 return contact the start of the path is used (it was already touching)
CONTACTS = {
    "2026-09-24 scan near row": [90, 85, 5, 5],
    "2026-09-24 return start": [76, 86, 7, 7],
    "2026-09-24 hand-placed servo edge": [93.4, 95, -2, 14.6],
    "2026-09-25 00:50 console": [103, 72, 19, 3],
    "2026-09-25 19:00 folded": [90, 102, -5, -1],
}
J5_DEFAULT = 92.0
CHAIN = [((0, 0, 0.064), (0, 0, 0)), ((0, 0, 0.0435), (0, math.pi / 2, 0)),
         ((-0.08285, 0, 0), (0, 0, 0)), ((-0.08285, 0, 0), (0, 0, 0)),
         ((-0.07385, -0.00215, 0), (0, -math.pi / 2, 0))]


def fk(servos) -> list[np.ndarray]:
    def rpy(r, p, y):
        return Matrix.Rotation(y, 4, "Z") @ Matrix.Rotation(p, 4, "Y") @ Matrix.Rotation(r, 4, "X")
    out, m = [Matrix.Identity(4)], Matrix.Identity(4)
    for (xyz, angles), s in zip(CHAIN, servos[:5]):
        m = m @ Matrix.Translation(xyz) @ rpy(*angles) @ Matrix.Rotation(math.radians(s - 90.0), 4, "Z")
        out.append(m.copy())
    return [np.array(x) for x in out]


def voxel(points: np.ndarray, size: float) -> np.ndarray:
    _, idx = np.unique(np.round(points / size).astype(np.int64), axis=0, return_index=True)
    return points[idx]


def load_meshes(voxel_m: float) -> dict[str, np.ndarray]:
    return {n: voxel(np.array([v.co[:] for v in bpy.data.objects[f"dofbot_{n}"].data.vertices]), voxel_m)
            for n in ("link3", "link4", "link5")}


def low_points(meshes, servos, below: float) -> np.ndarray:
    mats = fk(servos)
    pts = np.concatenate([meshes[n] @ mats[i][:3, :3].T + mats[i][:3, 3]
                          for n, i in (("link3", 3), ("link4", 4), ("link5", 5))])
    return pts[pts[:, 2] < below]


def hardware_poses() -> list[tuple[str, list[float]]]:
    seen = {}
    for path in sorted(glob.glob(str(ROOT / "data" / "**" / "*.jsonl"), recursive=True)):
        if "/data/sim/" in path:
            continue
        for line in open(path, encoding="utf-8", errors="ignore"):
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("mode") not in (None, "hardware") or e.get("kind") == "platform_contact":
                continue
            for key in ("pose", "readback", "observed"):
                p = e.get(key)
                if isinstance(p, list) and len(p) >= 4 and all(isinstance(v, (int, float)) for v in p[:4]):
                    j5 = p[4] if len(p) >= 5 and isinstance(p[4], (int, float)) else J5_DEFAULT
                    q = [float(v) for v in p[:4]] + [float(j5)]
                    seen.setdefault(tuple(round(v) for v in q), (str(Path(path).relative_to(ROOT)), q))
    return list(seen.values())


def near_contact(q, tol_deg: float = 3.0) -> bool:
    return any(all(abs(a - b) <= tol_deg for a, b in zip(q[:4], c)) for c in CONTACTS.values())


# --- shapes: depth inside (m, > 0 inside) of the deepest point --------------------------------
def depth_cylinder(p, radius, top):
    return float(np.minimum(radius - np.hypot(p[:, 0], p[:, 1]), top - p[:, 2]).max())


def depth_platform(p, cfg):
    return float(platform_model.depth(p[:, 0], p[:, 1], p[:, 2], cfg).max())


def sharp(cfg: dict) -> dict:
    out = json.loads(json.dumps(cfg))
    out["outline"]["corner_radius_m"] = 0.0
    out["lid"]["edge_radius_m"] = 0.0
    return out


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol-cm", type=float, default=0.3, help="allowed FK + mesh error")
    ap.add_argument("--voxel-mm", type=float, default=3.0)
    ap.add_argument("--json", help="write the per-shape summary here")
    args = ap.parse_args(argv)
    tol = args.tol_cm / 100.0
    top = PLATFORM["top_z_m"]
    meshes = load_meshes(args.voxel_mm / 1000.0)
    global SHARP
    SHARP = sharp(PLATFORM)

    safe = []
    for src, q in hardware_poses():
        if near_contact(q):
            continue
        pts = low_points(meshes, q, top + 0.03)
        if len(pts):
            safe.append((src, q, pts))
    contacts = [(name, list(q) + [J5_DEFAULT], low_points(meshes, list(q) + [J5_DEFAULT], top + 0.03))
                for name, q in CONTACTS.items()]

    shapes = {
        "keep_out_cylinder": lambda p: depth_cylinder(p, KEEP_OUT["platform_radius_m"], KEEP_OUT["platform_min_z_m"]),
        "platform_model": lambda p: depth_platform(p, PLATFORM),
        "sharp_square": lambda p: depth_platform(p, SHARP),
    }
    print(f"hardware poses with links below {100 * (top + 0.03):.1f} cm: {len(safe)}; contacts: {len(contacts)}; "
          f"tolerance {args.tol_cm} cm")
    summary = {}
    for name, depth in shapes.items():
        d_safe = [(depth(p), src, q) for src, q, p in safe]
        d_safe.sort(key=lambda t: -t[0])
        d_cont = [(depth(p) if len(p) else -1.0, n) for n, _, p in contacts]
        bad_safe = [t for t in d_safe if t[0] > tol]
        missed = [t for t in d_cont if t[0] < -tol]
        summary[name] = {
            "worst_safe_depth_cm": round(100 * d_safe[0][0], 2), "worst_safe_pose": d_safe[0][2],
            "worst_safe_source": d_safe[0][1],
            "shallowest_contact_depth_cm": round(100 * min(d for d, _ in d_cont), 2),
            "safe_inside_beyond_tol": len(bad_safe), "contacts_outside_beyond_tol": len(missed),
            "consistent": not bad_safe and not missed,
        }
        print(f"\n[{name}] consistent={summary[name]['consistent']}")
        print(f"  deepest contact-free pose: {100 * d_safe[0][0]:+.2f} cm  {d_safe[0][2]}  ({d_safe[0][1]})")
        for d, src, q in bad_safe[:8]:
            print(f"    inside by {100 * d:.2f} cm: {[round(v, 1) for v in q]}  ({src})")
        for d, n in d_cont:
            print(f"  contact {n:34s} depth {100 * d:+.2f} cm" + ("   <- outside the shape" if d < -tol else ""))
    if args.json:
        Path(args.json).write_text(json.dumps(summary, indent=1, ensure_ascii=False), encoding="utf-8")
    return 0 if summary["platform_model"]["consistent"] else 1


if __name__ == "__main__":
    sys.exit(main())
