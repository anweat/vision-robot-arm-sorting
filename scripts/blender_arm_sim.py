#!/usr/bin/env python3
"""Build the DOFBOT arm + gripper in Blender and render the poses the project actually uses.

The arm is the six course STL links assembled with the same URDF joint chain that
``app/src/spatial/dofbot_kinematics.py`` reproduces (verified: link origins match to <0.1 mm).
The gripper is the parametric model from ``app/config/gripper_model.json`` (two parallel pads,
opening from the official J6 table), not the smaller Isaac variant.

Run headless (Blender 5.2 is installed at /Applications/Blender.app):

    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        -P scripts/blender_arm_sim.py -- --out docs/images/sim --blend data/sim/dofbot_arm.blend

Writes pose renders to --out and a .blend (with the meshes embedded) to --blend.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
STL_DIR = ROOT / "资料" / "展开资源" / "三维结构文件" / "三维结构文件"
MODEL = json.loads((ROOT / "app" / "config" / "gripper_model.json").read_text(encoding="utf-8"))
_KEEP_OUT = json.loads((ROOT / "app" / "config" / "arm_limits.json").read_text(encoding="utf-8"))["keep_out"]
# chassis + black lid: app/config/platform_model.json (rounded square from the operator's photo, corner and
# edge rounding fitted to 1080 hardware poses on 2026-11-14).  Visual only - control uses keep_out.
sys.path.insert(0, str(ROOT / "dev" / "spatial"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))
import platform_model  # noqa: E402

PLATFORM = platform_model.load()
TABLE_Z = _KEEP_OUT["table_surface_z_m"]
PLATFORM_TOP_Z = PLATFORM["top_z_m"]                    # 0.076 == keep_out.platform_min_z_m
# The gripper servo (SG-5010, with its mounting ears) is part of link5.STL, beside the gear plate.
# Until 2026-11-14 the scene added a second, box-shaped servo on top of it; that was a duplicate.

LINK_STLS = ["base_link", "link1", "link2", "link3", "link4", "link5"]

# URDF joint chain (upstream dofbot.urdf, commit 9d10827): origin xyz, origin rpy, axis z
CHAIN = [
    ((0.0, 0.0, 0.064), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0435), (0.0, math.pi / 2, 0.0)),
    ((-0.08285, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-0.08285, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-0.07385, -0.00215, 0.0), (0.0, -math.pi / 2, 0.0)),
]


def T(x, y, z) -> Matrix:
    return Matrix.Translation((x, y, z))


def rpy(r, p, y) -> Matrix:
    return Matrix.Rotation(y, 4, "Z") @ Matrix.Rotation(p, 4, "Y") @ Matrix.Rotation(r, 4, "X")


def fk(servos) -> list[Matrix]:
    """World matrix of base_link, link1 ... link5 for servo angles (deg)."""
    out = [Matrix.Identity(4)]
    m = Matrix.Identity(4)
    for (xyz, angles), s in zip(CHAIN, servos[:5]):
        m = m @ T(*xyz) @ rpy(*angles) @ Matrix.Rotation(math.radians(s - 90.0), 4, "Z")
        out.append(m.copy())
    return out


def jaw_gap_m(servo6: float) -> float:
    pts = [(float(a), float(g) / 1000.0) for a, g in MODEL["jaw"]["points"]]
    if servo6 <= pts[0][0]:
        return pts[0][1]
    if servo6 >= pts[-1][0]:
        return pts[-1][1]
    for (a0, g0), (a1, g1) in zip(pts, pts[1:]):
        if a0 <= servo6 <= a1:
            return g0 + (servo6 - a0) / (a1 - a0) * (g1 - g0)
    return pts[-1][1]


def material(name: str, colour, metallic=0.15, roughness=0.45):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*colour, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def import_links() -> dict[str, object]:
    objs = {}
    for name in LINK_STLS:
        path = STL_DIR / f"{name}.STL"
        bpy.ops.wm.stl_import(filepath=str(path))
        obj = bpy.context.selected_objects[0]
        obj.name = f"dofbot_{name}"
        obj.data.name = f"dofbot_{name}_mesh"
        # the importer yields metres for these files; keep it explicit
        obj.data.materials.clear()
        obj.data.materials.append(material(
            f"mat_{name}",
            {"base_link": (0.16, 0.17, 0.19),
             "link1": (0.55, 0.57, 0.60),
             "link2": (0.55, 0.57, 0.60),
             "link3": (0.55, 0.57, 0.60),
             "link4": (0.55, 0.57, 0.60),
             "link5": (0.22, 0.23, 0.25)}[name],
            metallic=0.75, roughness=0.30))
        objs[name] = obj
    return objs


def _cube(name, size, colour, metallic=0.0, roughness=0.6):
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.select_set(False)
    obj.data.materials.append(material(f"mat_{name}", colour, metallic=metallic, roughness=roughness))
    return obj


def build_gripper() -> dict[str, object]:
    """Parametric pads (from app/config/gripper_model.json); the servo itself is already in link5.STL."""
    pad = MODEL["geometry"]["pad"]
    thick, width = pad["thickness_m"], pad["width_m"]
    z0, z1 = pad["z_range_from_j5_m"]
    parts = {}
    for side in ("left", "right"):
        parts[side] = _cube(f"gripper_{side}_pad", (thick, width, z1 - z0),
                            (0.08, 0.08, 0.09), roughness=0.9)
    return parts


def place_pose(links: dict, parts: dict, servos) -> None:
    mats = fk(servos)
    for name, obj in links.items():
        idx = LINK_STLS.index(name)
        obj.matrix_world = mats[idx]
    gap = jaw_gap_m(servos[5])
    pad = MODEL["geometry"]["pad"]
    thick, width = pad["thickness_m"], pad["width_m"]
    z0, z1 = pad["z_range_from_j5_m"]
    zc = (z0 + z1) / 2.0
    for side, sign in (("left", -1.0), ("right", +1.0)):
        local = T(sign * (gap / 2.0 + thick / 2.0), 0.0, zc)
        parts[side].matrix_world = mats[5] @ local


def _loft(name: str, rings: list[list[tuple]], colour, metallic=0.0, roughness=0.6):
    """Closed solid through stacked rings of (x, y, z) with equal vertex counts, capped at both ends."""
    import bmesh
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    bm = bmesh.new()
    vs = [[bm.verts.new(p) for p in ring] for ring in rings]
    bm.faces.new(list(reversed(vs[0])))
    bm.faces.new(vs[-1])
    n = len(rings[0])
    for lo, hi in zip(vs, vs[1:]):
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((lo[i], lo[j], hi[j], hi[i]))
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material(f"mat_{name}", colour, metallic=metallic, roughness=roughness))
    return obj


def _cut_cylinder(target, radius: float, z: float, depth: float = 0.2) -> None:
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=radius, depth=depth, location=(0.0, 0.0, z))
    cutter = bpy.context.active_object
    mod = target.modifiers.new("cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def add_platform(edge_segments: int = 8) -> dict[str, object]:
    """White chassis + black moulded lid (rounded square, rolled top edge) + turntable flange.

    base_link sits sunk in the chassis (base_link z 0..8.3 cm, lid top 7.6 cm); the servo-1 housing
    (r 4.0 cm flange at z 7.0 cm) comes up through the flange in the lid."""
    top = PLATFORM_TOP_Z
    lid_bottom = top - PLATFORM["lid"]["thickness_m"]
    e = PLATFORM["lid"]["edge_radius_m"]
    outline = platform_model.outline_polygon(PLATFORM)
    chassis = _loft("platform_chassis", [[(x, y, TABLE_Z) for x, y in outline],
                                         [(x, y, lid_bottom) for x, y in outline]],
                    (0.80, 0.81, 0.82), roughness=0.75)
    rings = [[(x, y, lid_bottom) for x, y in outline]]
    for k in range(edge_segments + 1):              # quarter-circle roll from the wall up to the flat top
        t = math.pi / 2.0 * k / edge_segments
        inset, z = e - e * math.cos(t), top - e + e * math.sin(t)
        rings.append([(x, y, z) for x, y in platform_model.outline_polygon(PLATFORM, inset=inset)])
    lid = _loft("platform_lid", rings, (0.035, 0.035, 0.04), metallic=0.05, roughness=0.6)

    hole = PLATFORM["hole"]
    _cut_cylinder(lid, hole["radius_m"], top)
    _cut_cylinder(chassis, hole["radius_m"], top)   # the housing stands in a well

    flange_top = top - 0.0015
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=hole["flange_outer_radius_m"],
                                        depth=hole["flange_thickness_m"],
                                        location=(0.0, 0.0, flange_top - hole["flange_thickness_m"] / 2.0))
    flange = bpy.context.active_object
    flange.name = "platform_flange"
    _cut_cylinder(flange, hole["flange_inner_radius_m"], flange_top)
    flange.data.materials.append(material("mat_platform_flange", (0.03, 0.03, 0.035),
                                          metallic=0.1, roughness=0.45))
    screws = []
    for i, a in enumerate(hole["screw_angles_deg"]):
        a = math.radians(a)
        bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=0.0035, depth=0.002,
                                            location=(hole["screw_radius_m"] * math.cos(a),
                                                      hole["screw_radius_m"] * math.sin(a), flange_top + 0.001))
        sc = bpy.context.active_object
        sc.name = f"platform_screw_{i}"
        sc.data.materials.append(material(f"mat_platform_screw_{i}", (0.70, 0.71, 0.73),
                                          metallic=0.9, roughness=0.3))
        screws.append(sc)
    return {"chassis": chassis, "lid": lid, "flange": flange, "screws": screws}


def add_stage(block_xy=(0.0, 0.22)) -> None:
    bpy.ops.mesh.primitive_plane_add(size=1.2, location=(0, 0.18, TABLE_Z))
    table = bpy.context.active_object
    table.name = "table"
    table.data.materials.append(material("mat_table", (0.30, 0.30, 0.29), metallic=0.0, roughness=0.85))

    bpy.ops.mesh.primitive_cube_add(size=0.03, location=(block_xy[0], block_xy[1], TABLE_Z + 0.015))
    block = bpy.context.active_object
    block.name = "block_3cm"
    block.data.materials.append(material("mat_block", (0.12, 0.35, 0.75), metallic=0.05, roughness=0.55))
    block.rotation_euler[2] = math.radians(15)


def add_lighting_and_camera():
    world = bpy.context.scene.world or bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.28, 0.30, 0.34, 1.0)
    bg.inputs[1].default_value = 0.28

    def area(name, loc, energy, size, rot):
        data = bpy.data.lights.new(name, "AREA")
        data.energy = energy
        data.size = size
        obj = bpy.data.objects.new(name, data)
        bpy.context.scene.collection.objects.link(obj)
        obj.location = loc
        obj.rotation_euler = rot
        return obj

    area("key", (0.60, -0.55, 0.95), 90, 1.0, (math.radians(48), 0, math.radians(38)))
    area("fill", (-0.75, -0.30, 0.55), 30, 1.2, (math.radians(65), 0, math.radians(-50)))
    area("rim", (0.0, 0.80, 0.65), 45, 0.8, (math.radians(-55), 0, 0))

    cam_data = bpy.data.cameras.new("Camera")
    cam_data.lens = 45
    cam = bpy.data.objects.new("Camera", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    target = Vector((0.0, 0.07, 0.21))
    cam.location = Vector((0.72, -0.62, 0.52))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam


def setup_render(engine: str, res=(1000, 800), samples=48):
    scene = bpy.context.scene
    scene.render.engine = engine
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    if engine == "CYCLES":
        scene.cycles.samples = samples
        scene.cycles.use_denoising = True
        scene.cycles.device = "CPU"
    elif engine == "BLENDER_EEVEE":
        scene.eevee.taa_render_samples = samples
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.exposure = 0.25
    scene.render.image_settings.file_format = "PNG"


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "images" / "sim"))
    ap.add_argument("--blend", default=str(ROOT / "data" / "sim" / "dofbot_arm.blend"))
    ap.add_argument("--engine", default="BLENDER_EEVEE")
    ap.add_argument("--samples", type=int, default=48)
    args = ap.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    links = import_links()
    parts = build_gripper()
    add_stage()
    add_platform()
    add_lighting_and_camera()
    setup_render(args.engine, samples=args.samples)

    poses = {
        "01_rest": [90, 90, 90, 90, 90, 30],
        "02_observe": [90, 105, 5, 5, 95, 30],
        "03_grasp": [107, 60, 26, 30, 92, 134],
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, servos in poses.items():
        place_pose(links, parts, servos)
        bpy.context.scene.render.filepath = str(out / f"{name}.png")
        bpy.ops.render.render(write_still=True)
        print(f"rendered {name} servos={servos}")

    blend = Path(args.blend)
    blend.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(blend), compress=True)
    print(f"saved {blend} ({blend.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
