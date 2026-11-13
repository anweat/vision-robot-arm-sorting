#!/usr/bin/env python3
"""Extract the DOFBOT arm + gripper geometry from the local Isaac Sim USD asset.

The USD (``data/reference/isaac_dofbot/dofbot.usd``, sha256 in SOURCE.md) is NVIDIA's
published Yahboom Dofbot asset. It is the only known model that contains the gripper:
the official ROS 2 snapshot ships an URDF with five revolute joints and no gripper.

What this writes (``data/reference/isaac_dofbot/``, not in git):

  geometry.json          joints (bodies, local poses, axis, limits, drives), link world poses,
                         mesh world AABBs, and the derived gripper numbers used by
                         ``config/gripper_model.json``
  meshes/<link>.obj      gripper meshes (wrist + 3 finger links per side) in that link's own
                         frame, so they can be attached to a URDF/Blender joint at the origin

Run (``usd-core`` is cached in the workspace uv cache, ``~/.cache`` is not writable here):

    UV_CACHE_DIR=$PWD/.uv-cache uv run --with usd-core --with numpy \
        python scripts/extract_isaac_dofbot.py

Everything here is a *model value*, not a measurement of this arm. The USD is an
independent third-party build; differences against the real arm are expected (see
docs/atlas-200i-dk-a2/27-夹爪与整臂建模.md).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
USD_PATH = ROOT / "data" / "reference" / "isaac_dofbot" / "dofbot.usd"
OUT_DIR = USD_PATH.parent
MESH_DIR = OUT_DIR / "meshes"
GEOMETRY_JSON = OUT_DIR / "geometry.json"

# links whose mesh is small enough to be worth exporting for the gripper assembly
EXPORT_LINKS = [
    "link5", "link5/Wrist_Twist",
    "link5/Finger_Left_01", "link5/Finger_Left_02", "link5/Finger_Left_03",
    "link5/Finger_Right_01", "link5/Finger_Right_02", "link5/Finger_Right_03",
]


def vec(v) -> list[float]:
    return [float(x) for x in v]


def quat(q) -> list[float]:
    """Gf.Quatf/Gf.Quatd -> [w, x, y, z]."""
    if hasattr(q, "GetReal"):
        im = q.GetImaginary()
        return [float(q.GetReal()), float(im[0]), float(im[1]), float(im[2])]
    return [float(x) for x in q]


def mat_to_lists(m: Gf.Matrix4d) -> list[list[float]]:
    """USD matrices are row-major with row-vector convention; return a 4x4 row-major list."""
    return [[float(m[i][j]) for j in range(4)] for i in range(4)]


def prim_world(cache: UsdGeom.XformCache, prim):
    return cache.GetLocalToWorldTransform(prim)


def bbox_world(bb: UsdGeom.BBoxCache, prim) -> dict:
    r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return {}
    mn, mx = r.GetMin(), r.GetMax()
    return {"min_m": vec(mn), "max_m": vec(mx), "extent_m": vec(mx - mn)}


def write_obj(path: Path, points: np.ndarray, faces: np.ndarray, name: str) -> int:
    """Minimal OBJ writer: vertices already in the target frame, 1-based faces."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(f"# {name}\n# local frame of the link, metres\n")
        for p in points:
            f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        for tri in faces:
            f.write(f"f {tri[0] + 1} {tri[1] + 1} {tri[2] + 1}\n")
    return len(points)


def mesh_arrays(prim) -> tuple[np.ndarray, np.ndarray]:
    mesh = UsdGeom.Mesh(prim)
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=int)
    idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
    faces, k = [], 0
    for c in counts:
        tri = idx[k:k + c]
        k += c
        for i in range(1, len(tri) - 1):        # fan triangulation, the meshes are convex enough
            faces.append([tri[0], tri[i], tri[i + 1]])
    return pts, np.array(faces, dtype=int)


def main() -> int:
    stage = Usd.Stage.Open(str(USD_PATH))
    if stage is None:
        print(f"cannot open {USD_PATH}", file=sys.stderr)
        return 1
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])

    out: dict = {
        "source": {
            "usd": str(USD_PATH.relative_to(ROOT)),
            "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage),
            "upAxis": UsdGeom.GetStageUpAxis(stage),
            "note": "Model values from NVIDIA's published Yahboom Dofbot asset, not measurements of this arm.",
        },
        "joints": [],
        "links": {},
        "meshes": {},
    }

    # ---- joints -------------------------------------------------------------------------
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = {
            "path": prim.GetPath().pathString,
            "type": prim.GetTypeName(),
            "bodies": {},
            "local_pos": {},
            "local_rot_quat_wxyz": {},
            "axis": None,
            "limits": {},
            "drive": {},
        }
        for rel in prim.GetRelationships():
            name = rel.GetName()
            if name in ("physics:body0", "physics:body1"):
                targets = [str(t) for t in rel.GetTargets()]
                joint["bodies"][name.split(":")[1]] = targets[0] if targets else None
        for attr in prim.GetAttributes():
            n = attr.GetName()
            if n == "physics:localPos0":
                joint["local_pos"]["0"] = vec(attr.Get())
            elif n == "physics:localPos1":
                joint["local_pos"]["1"] = vec(attr.Get())
            elif n == "physics:localRot0":
                joint["local_rot_quat_wxyz"]["0"] = quat(attr.Get())
            elif n == "physics:localRot1":
                joint["local_rot_quat_wxyz"]["1"] = quat(attr.Get())
            elif n == "physics:axis":
                joint["axis"] = str(attr.Get())
            elif n == "physics:lowerLimit":
                joint["limits"]["lower_deg"] = float(attr.Get())
            elif n == "physics:upperLimit":
                joint["limits"]["upper_deg"] = float(attr.Get())
            elif n.startswith("drive:"):
                joint["drive"][n.removeprefix("drive:")] = attr.Get()
        out["joints"].append(joint)

    # ---- link world poses + mesh bounds -------------------------------------------------
    link_paths = ["/arm/base_link", "/arm/link1", "/arm/link2", "/arm/link3", "/arm/link4",
                  "/arm/link5", "/arm/link5/Wrist_Twist"]
    link_paths += [f"/arm/link5/Finger_{side}_0{i}" for side in ("Left", "Right") for i in (1, 2, 3)]
    for path in link_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        m = prim_world(cache, prim)
        t = m.ExtractTranslation()
        out["links"][path] = {
            "world_pos_cm": [round(t[i] * 100, 4) for i in range(3)],
            "world_matrix_rowmajor": mat_to_lists(m),
            "world_aabb": {k: [round(x * 100, 4) for x in v] if isinstance(v, list) else v
                           for k, v in bbox_world(bb, prim).items()},
        }

    # ---- mesh prims (world AABB, and gripper meshes exported in link frame) --------------
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        path = prim.GetPath().pathString
        out["meshes"][path] = {"world_aabb": {k: [round(x * 100, 4) for x in v]
                                              if isinstance(v, list) else v
                                              for k, v in bbox_world(bb, prim).items()}}

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for rel in EXPORT_LINKS:
        link = stage.GetPrimAtPath(f"/arm/{rel}")
        if not link:
            continue
        inv = prim_world(cache, link).GetInverse()
        exported = []
        for child in Usd.PrimRange(link):
            if not child.IsA(UsdGeom.Mesh):
                continue
            pts, faces = mesh_arrays(child)
            m = prim_world(cache, child)
            world = np.array([(m.Transform(Gf.Vec3d(*p))) for p in pts])
            local = np.array([inv.Transform(Gf.Vec3d(*p)) for p in world])
            name = child.GetName()
            obj = MESH_DIR / f"{rel.replace('/', '_')}__{name}.obj"
            write_obj(obj, local, faces, name)
            exported.append(str(obj.relative_to(ROOT)))
        out["links"].setdefault(f"/arm/{rel}", {})["exported_meshes"] = exported

    # ---- derived gripper numbers --------------------------------------------------------
    def wb(path: str) -> dict:
        return out["links"][path]["world_aabb"]

    wrist = out["links"]["/arm/link5/Wrist_Twist"]["world_pos_cm"]
    left3, right3 = wb("/arm/link5/Finger_Left_03"), wb("/arm/link5/Finger_Right_03")
    left1, right1 = wb("/arm/link5/Finger_Left_01"), wb("/arm/link5/Finger_Right_01")
    fingers = {
        "j5_axis_world_cm": wrist,
        "left_finger_pivot_world_cm": out["links"]["/arm/link5/Finger_Left_01"]["world_pos_cm"],
        "right_finger_pivot_world_cm": out["links"]["/arm/link5/Finger_Right_01"]["world_pos_cm"],
        "tip_highest_world_z_cm": max(left3["max_m"][2], right3["max_m"][2]),
        "j5_to_tip_cm": round(max(left3["max_m"][2], right3["max_m"][2]) - wrist[2], 4),
        "left_tip_x_range_cm": [left3["min_m"][0], left3["max_m"][0]],
        "right_tip_x_range_cm": [right3["min_m"][0], right3["max_m"][0]],
        "default_inner_gap_cm": round(right3["min_m"][0] - left3["max_m"][0], 4),
        "default_outer_span_cm": round(right3["max_m"][0] - left3["min_m"][0], 4),
        "proximal_link_extent_cm": {
            "left": left1["extent_m"], "right": right1["extent_m"]},
    }
    out["gripper_default_pose"] = fingers
    out["_how_to_read"] = (
        "World coordinates are the USD default pose (all drives at their authored target, "
        "which is NOT the servo's closed position). Lengths: 1 unit = 1 m, printed here in cm."
    )

    GEOMETRY_JSON.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"wrote {GEOMETRY_JSON.relative_to(ROOT)}")
    print(f"wrote {len(list(MESH_DIR.glob('*.obj')))} OBJ files in {MESH_DIR.relative_to(ROOT)}")
    print(json.dumps(fingers, ensure_ascii=False, indent=1))
    print("\nJoint chain (world z, cm):")
    for path in link_paths[:7]:
        if path in out["links"]:
            print(f"  {path:32s} {out['links'][path]['world_pos_cm']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
