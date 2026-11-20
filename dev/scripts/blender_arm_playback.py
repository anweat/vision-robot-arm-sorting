#!/usr/bin/env python3
"""Render a playback of the whole arm picking up a block (kinematic, no contact solver).

The course STL for link5 already contains the complete gripper - fingers, four-bar linkage, serrated
pads - as ONE static mesh, so Blender cannot open/close it.  This script therefore animates the arm
along the verified pick path (straight vertical descent, as app/src/tasks/console_grasp.py does) and
moves the block with the gripper from the grip moment on.  It is a *kinematic playback*: gravity and
contacts are not solved.  For contact physics the gripper has to be split into movable finger links
(see app/config/urdf/dofbot_with_gripper.urdf) and run in a real physics engine.

    /Applications/Blender.app/Contents/MacOS/Blender -b data/sim/dofbot_arm.blend \
        -P dev/scripts/blender_arm_playback.py -- --mp4 records/sim/arm_playback.mp4
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dev" / "scripts"))
sys.path.insert(0, str(ROOT / "dev" / "spatial"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))

import blender_arm_physics as sim  # noqa: E402  (timeline + pose helpers)
import blender_arm_sim as base  # noqa: E402


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default=str(ROOT / "data" / "sim" / "playback_frames"))
    ap.add_argument("--mp4", default=str(ROOT / "records" / "sim" / "arm_playback.mp4"))
    ap.add_argument("--samples", type=int, default=20)
    args = ap.parse_args(argv)

    # the STL gripper is static: hide the parametric pads so they do not double up
    for name in ("gripper_left_pad", "gripper_right_pad"):
        bpy.data.objects[name].hide_render = True
        bpy.data.objects[name].hide_viewport = True

    samples = sim.build_timeline()
    set_pose = sim.set_pose
    link5 = bpy.data.objects["dofbot_link5"]
    block = bpy.data.objects["block_3cm"]

    # where the block sits before it is picked, and the gripper pose at that moment
    GRIP_FRAME = 74
    grip_pose = dict(samples)[74]
    set_pose(grip_pose)
    bpy.context.view_layer.update()
    grip_matrix = link5.matrix_world.copy()
    set_pose(samples[0][1])
    bpy.context.view_layer.update()
    centre = sim.pad_centre_world(samples[0][1])
    block.location = (centre.x, centre.y, sim.TABLE_Z + sim.BLOCK / 2.0)
    block.rotation_euler = (0.0, 0.0, math.radians(15))
    bpy.context.view_layer.update()
    rest = block.matrix_world.copy()
    offset = grip_matrix.inverted() @ rest

    for frame, servos in samples:
        set_pose(servos)
        bpy.context.view_layer.update()
        for name in [f"dofbot_{n}" for n in base.LINK_STLS]:
            obj = bpy.data.objects[name]
            obj.keyframe_insert("location", frame=frame)
            obj.keyframe_insert("rotation_euler", frame=frame)
        if frame >= GRIP_FRAME:
            block.matrix_world = link5.matrix_world @ offset
        else:
            block.matrix_world = rest
        block.keyframe_insert("location", frame=frame)
        block.keyframe_insert("rotation_euler", frame=frame)

    # camera: look at the action zone
    cam = bpy.data.objects["Camera"]
    target = Vector((0.0, 0.15, 0.09))
    cam.location = Vector((0.40, -0.36, 0.22))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = 50

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 150
    scene.render.fps = 24
    base.setup_render("BLENDER_EEVEE", res=(800, 640), samples=args.samples)

    # Blender 5.2 here has no FFMPEG output: render a PNG sequence, encode it with OpenCV after
    frames = Path(args.frames)
    frames.mkdir(parents=True, exist_ok=True)
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(frames / "frame_")
    bpy.ops.render.render(animation=True)
    print(f"frames {frames}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
