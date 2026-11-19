#!/usr/bin/env python3
"""WIP: rigid-body (Bullet) setup for the DOFBOT arm and a 3 cm block.

Status 2026-11-13: the world runs (gravity, landing, pushes) but the *contact grasp* does not hold,
because link5.STL contains the whole gripper as one static mesh whose fingers sit ~28 mm apart, so a
30 mm block is pushed away on the way down.  Use scripts/blender_arm_playback.py for the visual
playback, and the URDF in app/config/urdf/ + MuJoCo/Isaac for real contact physics.

Loads the scene built by ``scripts/blender_arm_sim.py`` (``data/sim/dofbot_arm.blend``), then:

  * the arm links and gripper pads become *passive* rigid bodies whose transforms are keyframed
    from the URDF forward kinematics (the same chain the project's IK uses) - so they collide but
    never fall or get pushed;
  * the 3 cm block is an *active* rigid body (30 g, box collision, friction), so it settles on the
    table under gravity and is held only by contact/friction while the fingers close and lift;
  * the pads close from the official J6 jaw curve (open 30 deg -> 62 mm, grip 134 deg -> 30 mm).

Run:

    /Applications/Blender.app/Contents/MacOS/Blender -b data/sim/dofbot_arm.blend \
        -P scripts/blender_arm_physics.py -- --mp4 docs/sim/arm_grasp.mp4 --engine BLENDER_EEVEE
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
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "dev" / "spatial"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))

import blender_arm_sim as base  # noqa: E402  (FK, jaw curve, materials, stage)
import dofbot_kinematics as kin  # noqa: E402

FPS = 24
PITCH = 150.0
OPEN_DEG = 30
GRIP_DEG = 136
TABLE_Z = json.loads((ROOT / "app" / "config" / "arm_limits.json").read_text(encoding="utf-8"))[
    "keep_out"]["table_surface_z_m"]
BLOCK = 0.03


def pad_centre_world(servos) -> Vector:
    mats = base.fk(servos)
    pad = base.MODEL["geometry"]["pad"]
    zc = sum(pad["z_range_from_j5_m"]) / 2.0
    return mats[5] @ Vector((0.0, 0.0, zc))


def solve_tip_to_pad_centre(bx, by, bz, pitch_deg) -> list[float] | None:
    """Servo 1-4 that put the *pad centre* (not the FK tip) on the block centre."""
    p = math.radians(pitch_deg)
    d = Vector((0.0, math.sin(p), math.cos(p))) * (base.MODEL["tcp"]["fk_tip_m"]
                                                   - base.MODEL["tcp"]["grasp_centre_m"])
    tip_target = (bx + d.x, by + d.y, bz + d.z)
    servos = kin.inverse(*tip_target, pitch_deg)
    return servos


def find(name):
    return bpy.data.objects[name]


def set_pose(servos):
    """Place all arm objects for servo angles, exactly like the still renders do."""
    links = {n: find(f"dofbot_{n}") for n in base.LINK_STLS}
    parts = {"left": find("gripper_left_pad"), "right": find("gripper_right_pad")}
    base.place_pose(links, parts, servos)


def keyframe_pose(servos, frame: int) -> None:
    set_pose(servos)
    for name in [f"dofbot_{n}" for n in base.LINK_STLS] + ["gripper_left_pad", "gripper_right_pad"]:
        obj = find(name)
        obj.keyframe_insert("location", frame=frame)
        obj.keyframe_insert("rotation_euler", frame=frame)
        obj.keyframe_insert("scale", frame=frame)


def lerp_servos(a, b, t):
    return [x + (y - x) * t for x, y in zip(a, b)]


def build_timeline() -> list[tuple[int, list[float]]]:
    """(frame, servos), 24 fps. Descent and lift are straight vertical lines solved with IK, the way
    the console moves the arm (docs/05: "vertical descent to the block centre"), not servo lerps -
    a servo lerp between two poses sweeps the pads forward and pushes the block away."""
    open_pose = [90, 105, 5, 5, 95, OPEN_DEG]
    set_pose(open_pose)
    centre = pad_centre_world(open_pose)
    bx, by = centre.x, centre.y
    bz = TABLE_Z + BLOCK / 2.0

    def at_height(z: float, grip: float):
        servos = solve_tip_to_pad_centre(bx, by, z, PITCH)
        if servos is None:
            raise SystemExit(f"no IK at z={z:.3f}")
        return list(servos) + [95.0, grip]

    hover = at_height(bz + 0.06, OPEN_DEG)
    low = at_height(bz + 0.002, OPEN_DEG)

    samples: list[tuple[int, list[float]]] = []
    # 1-20 hold the observe pose, 20-40 travel to the hover pose
    for f in range(1, 41, 2):
        t = 0.0 if f <= 20 else (f - 20) / 20.0
        samples.append((f, lerp_servos(open_pose, hover, t)))
    # 40-60 vertical descent
    for f in range(40, 61, 2):
        t = (f - 40) / 20.0
        samples.append((f, at_height(bz + 0.06 + (0.002 - 0.06) * t, OPEN_DEG)))
    # 60-74 close the fingers
    for f in range(60, 75, 1):
        t = (f - 60) / 14.0
        samples.append((f, low[:5] + [OPEN_DEG + (GRIP_DEG - OPEN_DEG) * t]))
    # 74-98 vertical lift, holding
    for f in range(74, 99, 2):
        t = (f - 74) / 24.0
        samples.append((f, at_height(bz + 0.002 + 0.09 * t, GRIP_DEG)))
    # 98-126 turn the base 45 deg with the block held
    carry = at_height(bz + 0.092, GRIP_DEG)
    for f in range(98, 127, 2):
        t = (f - 98) / 28.0
        s1 = 90.0 + (45.0 - 90.0) * t
        samples.append((f, [s1] + carry[1:]))
    # 126-150 hold still
    for f in range(126, 151, 2):
        samples.append((f, [45.0] + carry[1:]))
    return samples


def add_physics(block_name="block_3cm", table_name="table"):
    scene = bpy.context.scene
    if scene.rigidbody_world is None:
        bpy.ops.rigidbody.world_add()
    world = scene.rigidbody_world
    world.substeps_per_frame = 20
    world.solver_iterations = 30
    world.time_scale = 1.0

    table = find(table_name)
    bpy.context.view_layer.objects.active = table
    bpy.ops.rigidbody.object_add(type="PASSIVE")
    table.rigid_body.collision_shape = "MESH"
    table.rigid_body.friction = 0.9
    table.rigid_body.use_margin = True
    table.rigid_body.collision_margin = 0.0005

    block = find(block_name)
    bpy.context.view_layer.objects.active = block
    bpy.ops.rigidbody.object_add(type="ACTIVE")
    block.rigid_body.mass = 0.03
    block.rigid_body.collision_shape = "BOX"
    block.rigid_body.friction = 1.0
    block.rigid_body.restitution = 0.05
    block.rigid_body.linear_damping = 0.05
    block.rigid_body.angular_damping = 0.1
    block.rigid_body.use_margin = True
    block.rigid_body.collision_margin = 0.0005

    for name in [f"dofbot_{n}" for n in base.LINK_STLS] + ["gripper_left_pad", "gripper_right_pad"]:
        obj = find(name)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.rigidbody.object_add(type="PASSIVE")
        obj.rigid_body.kinematic = True          # follow the keyframed transforms (Bullet "Animated")
        obj.rigid_body.collision_shape = "BOX" if "gripper" in name else "MESH"
        obj.rigid_body.friction = 2.0 if "gripper" in name else 1.0
        obj.rigid_body.use_margin = True
        obj.rigid_body.collision_margin = 0.0005

    return block, table


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", default=str(ROOT / "data" / "sim" / "frames"))
    ap.add_argument("--still-dir", default=str(ROOT / "docs" / "images" / "sim"))
    ap.add_argument("--samples", type=int, default=24)
    ap.add_argument("--end-frame", type=int, default=150)
    args = ap.parse_args(argv)

    samples = build_timeline()
    # place the block where the simulation expects it: under the pad centre at the hover pose
    set_pose(samples[0][1])
    centre = pad_centre_world(samples[0][1])
    block = find("block_3cm")
    block.location = (centre.x, centre.y, TABLE_Z + BLOCK / 2.0)
    block.rotation_euler = (0, 0, math.radians(15))

    add_physics()

    for frame, servos in samples:
        keyframe_pose(servos, frame)

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = args.end_frame
    scene.render.fps = FPS
    base.setup_render("BLENDER_EEVEE", res=(800, 640), samples=args.samples)

    # PNG sequence: the rigid-body world has to be stepped frame by frame, so render the whole
    # animation in order and cut stills out of it afterwards (jumping frames gives wrong physics).
    frames_dir = Path(args.frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(frames_dir / "frame_")
    bpy.ops.render.render(animation=True)
    print(f"frames in {frames_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
