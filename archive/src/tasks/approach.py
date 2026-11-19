#!/usr/bin/env python3
"""Hover over one scanned block, centre the gripper on it from close-up views, optionally grasp.

  approach.py --scan runs/scan_x/scan.json --target 1                    plan only
  approach.py --scan runs/scan_x/scan.json --target 1 --execute          hover, centre, return
  approach.py --scan runs/scan_x/scan.json --target 1 --execute --grasp  ... then descend, grip,
                                                                          lift, hold, put back

The gripper approaches along its own axis at the steepest pitch for which both the hover
point (tip `--hover` metres back along the axis from the grasp point) and the grasp point
(tip `GRASP_Z` above the table, centred on the block) are reachable in the soft limits.

The wrist camera sits about 1.6 cm off the gripper axis and 5 cm behind the tip, so a block
right under the gripper is too close and too far off-centre to be seen whole (2026-09-24).
The block is therefore measured from a "look" pose that puts the camera axis over it,
slightly higher (`--look`); its top centre is taken in the camera frame (map markers give the
camera pose, else forward kinematics), carried into the gripper frame with the hand-eye
transform and back into the base frame with forward kinematics of the same reading. The
gripper then moves over that estimate, a short relative move, so backlash and link errors
largely cancel. Only detections within `--gate` of the expected position are accepted.

Grasping uses the calibrated gripper angles in app/config/arm_limits.json; descents are straight
lines along the gripper axis, with the table keep-out margin reduced to GRASP_TABLE_MARGIN
(the platform keep-out still applies), and any joint meeting resistance makes the arm
retreat one step and stop.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[2]
for sub in ("adapters", "spatial", "vision"):
    sys.path.insert(0, str(APP / "src" / sub))

import block_detector as bd  # noqa: E402
import dofbot_kinematics as kin  # noqa: E402
import map_difference as md  # noqa: E402
import safe_arm  # noqa: E402
import workspace  # noqa: E402
from camera_client import snapshot_after  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import CameraPose, TableMap, detect_markers, load_intrinsics, solve_camera_pose  # noqa: E402

BLOCK_M = 0.03
GRASP_Z = 0.012            # tip height above the table at the grasp
GRASP_TABLE_MARGIN = 0.005
MIN_CLEARANCE_M = 0.045    # tip above the table on every interpolated step before the hover
DESCENT_STEPS = 4
WRIST = 95                 # servo 5 kept as in the scan poses
OBSERVE = [90, 105, 5, 5, WRIST]


def grasp_plan(block_base: np.ndarray, table_z: float, hover: float, limits: dict) -> dict | None:
    """Steepest pitch with reachable hover and grasp points along one straight axis line."""
    grasp_tip = np.array([block_base[0], block_base[1], table_z + GRASP_Z])
    yaw = math.atan2(-grasp_tip[0], grasp_tip[1])  # heading of servo 1 towards the block
    for pitch in range(180, 119, -1):
        p = math.radians(pitch)
        axis = np.array([-math.sin(yaw) * math.sin(p), math.cos(yaw) * math.sin(p), math.cos(p)])
        tips = [grasp_tip - hover * axis * (1 - i / DESCENT_STEPS) for i in range(DESCENT_STEPS + 1)]
        solutions = [kin.inverse(*tip, pitch) for tip in tips]
        if all(s is not None and all(limits[j][0] <= a <= limits[j][1] for j, a in zip((1, 2, 3, 4), s))
               for s in solutions):
            return {"pitch_deg": pitch, "hover_tip": tips[0], "grasp_tip": tips[-1],
                    "hover_servos": solutions[0], "grasp_servos": solutions[-1], "descent": solutions}
    return None


def look_plan(block_base: np.ndarray, table_z: float, distance: float, limits: dict) -> dict | None:
    """Steepest reachable pose with the tip `distance` back along its axis from the grasp point.
    Unlike grasp_plan only this one pose has to be reachable."""
    grasp_tip = np.array([block_base[0], block_base[1], table_z + GRASP_Z])
    yaw = math.atan2(-grasp_tip[0], grasp_tip[1])
    for pitch in range(180, 119, -1):
        p = math.radians(pitch)
        axis = np.array([-math.sin(yaw) * math.sin(p), math.cos(yaw) * math.sin(p), math.cos(p)])
        tip = grasp_tip - distance * axis
        s = kin.inverse(*tip, pitch)
        if s is not None and all(limits[j][0] <= a <= limits[j][1] for j, a in zip((1, 2, 3, 4), s)):
            return {"pitch_deg": pitch, "hover_tip": tip, "hover_servos": s}
    return None


def min_clearance(start: list, goal: list, table_z: float, samples: int = 20) -> float:
    lowest = float("inf")
    for i in range(samples + 1):
        pose = [a + (b - a) * i / samples for a, b in zip(start[:4], goal[:4])]
        fk = kin.forward(pose)
        lowest = min(lowest, *(fk[n][2] - table_z for n in ("wrist", "servo5", "tip")))
    return lowest


def camera_pose(image, servos, table, base_T_map, k, dist, cam):
    """Camera pose in the map frame: from markers when any are visible, else from kinematics."""
    markers = detect_markers(image, table.layout)
    map_T_cam = np.linalg.inv(base_T_map) @ cam.base_T_cam(servos)
    cam_T_map = np.linalg.inv(map_T_cam)
    fk_pose = CameraPose(cam_T_map[:3, :3], cam_T_map[:3, 3])
    pose = solve_camera_pose(markers, table, k, dist, prior_centre=fk_pose.centre) if markers else None
    if pose is None:
        return fk_pose, markers, "forward kinematics"
    return pose, markers, f"{len(markers)} markers"


def find_block(image, pose, markers, expected_map_xy, gate, table, k, dist):
    """Detected object nearest the expected position, only if within the gate."""
    blobs = md.detect_objects(image, pose, k, dist, table.layout, markers)
    best, best_d = None, gate
    for b in blobs:
        try:
            bd.locate(b, pose, k, dist, BLOCK_M)
        except ValueError:
            continue
        if b.size_m < 0.02:
            continue
        d = float(np.linalg.norm(np.array(b.map_xy) - expected_map_xy))
        if d < best_d:
            best, best_d = b, d
    return best, blobs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scan", type=Path, required=True, help="scan.json (or scan_replay.json) from scan.py")
    parser.add_argument("--target", type=int, default=0, help="index into the scan's targets")
    parser.add_argument("--hover", type=float, default=0.06, help="tip distance back along the axis from the grasp point")
    parser.add_argument("--iterations", type=int, default=4)
    parser.add_argument("--look", type=float, default=0.08, help="like --hover, for the pose that looks at the block")
    parser.add_argument("--no-look", action="store_true",
                        help="use the scan's multi-view position directly; for blocks so close to the platform that the "
                             "look pose sees only one marker and an off-centre block (2026-09-24: 1.5-2 cm worse than the scan)")
    parser.add_argument("--centre-tolerance", type=float, default=0.012,
                        help="accept a measurement when the block top is this close to the camera axis")
    parser.add_argument("--gate", type=float, default=0.025, help="ignore detections farther than this from the target")
    parser.add_argument("--grasp", action="store_true", help="after centring: descend, grip, lift, hold, put back")
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--out", type=Path, default=APP / "runs" / time.strftime("approach_%Y%m%d_%H%M%S"))
    args = parser.parse_args()

    scan = json.loads(args.scan.read_text())
    base_T_map = np.array(scan["map_in_base"]["matrix"])
    target = scan["targets"][args.target]
    map_xy = np.array(target["map_cm"]) / 100
    table_z = float((base_T_map @ np.array([0, 0, 0, 1.0]))[2])
    table, cam = TableMap(), ArmCamera()
    cam2grip = cam.cam2grip
    k, dist = load_intrinsics()

    from safe_arm import DEFAULT_CONFIG, MotionError, SafeArm, load_config

    cfg = load_config(DEFAULT_CONFIG)
    limits = {j: cfg["soft_limits_deg"][j] for j in (1, 2, 3, 4)}
    gripper = cfg["gripper"]

    block = (base_T_map @ np.array([map_xy[0], map_xy[1], BLOCK_M / 2, 1.0]))[:3]
    plan = grasp_plan(block, table_z, args.hover, limits)
    if plan is None:
        print(json.dumps({"error": "no reachable straight approach", "target": target}))
        return 2
    hover_pose = [*plan["hover_servos"], WRIST]
    keep_out = workspace.load()
    report = {"target": target, "table_z_cm": round(table_z * 100, 1), "block_base_cm": (block * 100).round(1).tolist(),
              "pitch_deg": plan["pitch_deg"], "hover_servos": plan["hover_servos"], "grasp_servos": plan["grasp_servos"],
              "hover_tip_above_table_cm": round((plan["hover_tip"][2] - table_z) * 100, 1),
              "clearance_on_way_cm": round(min_clearance(OBSERVE, hover_pose, table_z) * 100, 1),
              "keep_out_margin_cm": round(workspace.path_clearance(OBSERVE, hover_pose, keep_out) * 100, 1)}
    print(json.dumps(report))
    if report["clearance_on_way_cm"] < MIN_CLEARANCE_M * 100 or report["keep_out_margin_cm"] < 0:
        print("refusing: the path to the hover pose passes too close to the table or the platform")
        return 2
    if not args.execute:
        return 0

    args.out.mkdir(parents=True, exist_ok=True)
    arm = SafeArm(execute=True)
    cam_offset = cam2grip[:3, 3]  # camera origin in the wrist camera-mount frame; its axis is near the gripper's
    steps, grasp_log, measured = [], [], False
    try:
        for i in range(0 if args.no_look else args.iterations + 1):
            # look pose: camera axis (not the gripper axis) over the current block estimate
            probe = look_plan(block, table_z, args.look, limits)
            if probe is None:
                print("look pose unreachable")
                break
            rot = np.array(kin.camera_mount_pose(probe["hover_servos"]))[:3, :3]
            shift = rot @ np.array([cam_offset[0], cam_offset[1], 0.0])
            look = look_plan(block - np.array([shift[0], shift[1], 0.0]), table_z, args.look, limits)
            if look is None:
                print("look pose unreachable")
                break
            arm.move([*look["hover_servos"], WRIST], label=f"look {i}")
            time.sleep(0.8)
            image, stamp = snapshot_after(time.time() - 0.01)
            servos = arm.read()[:5]
            pose, markers, reference = camera_pose(image, servos, table, base_T_map, k, dist, cam)
            blob, blobs = find_block(image, pose, markers, map_xy, args.gate, table, k, dist)
            cv2.imwrite(str(args.out / f"look_{i}.jpg"), image)
            cv2.imwrite(str(args.out / f"look_{i}_annotated.jpg"), bd.draw(image, blobs))
            step = {"iteration": i, "servos": servos, "reference": reference, "frame_time": stamp,
                    "found": blob is not None}
            if blob is None:
                steps.append(step)
                print(json.dumps(step))
                break
            top_map = np.array([blob.map_xy[0], blob.map_xy[1], BLOCK_M])
            top_cam = pose.R @ top_map + pose.t
            block = (cam.base_T_cam(servos) @ np.append(top_cam, 1.0))[:3] - np.array([0, 0, BLOCK_M / 2])
            off_axis = float(np.hypot(top_cam[0], top_cam[1]))
            map_xy = np.array(blob.map_xy)
            step.update(map_cm=(map_xy * 100).round(2).tolist(), partial=blob.partial,
                        off_camera_axis_mm=round(off_axis * 1000, 1),
                        block_base_cm=(block * 100).round(2).tolist())
            steps.append(step)
            print(json.dumps(step))
            if not blob.partial and off_axis < args.centre_tolerance:
                measured = True
                break

        measured = measured or args.no_look
        plan = grasp_plan(block, table_z, args.hover, limits) if measured else None
        if plan is not None:
            arm.move([*plan["hover_servos"], WRIST], label="hover over block")
            time.sleep(0.8)
            image, _ = snapshot_after(time.time() - 0.01)
            cv2.imwrite(str(args.out / "hover.jpg"), image)
            if args.grasp:
                grasp_log = grasp(arm, plan, gripper, args.hold_seconds, MotionError)
        elif args.grasp:
            grasp_log = [{"skipped": "block not measured from a clear view"}]

        result = {"report": report, "steps": steps, "measured": measured, "grasp": grasp_log,
                  "final_block_base_cm": (block * 100).round(2).tolist(),
                  "hover_servos": None if plan is None else plan["hover_servos"]}
        (args.out / "approach.json").write_text(json.dumps(result, indent=2, default=float) + "\n")
        arm.move(OBSERVE, label="approach return")
    finally:
        arm.close()
    print("saved", args.out)
    return 0


def grasp(arm, plan, gripper: dict, hold_seconds: float, motion_error) -> list[dict]:
    """Descend along the axis, grip, lift back to the hover, hold, and put the block back."""
    log = []
    descent = [[*s, WRIST] for s in plan["descent"]]

    def record(event, **fields):
        entry = {"event": event, **fields}
        log.append(entry)
        print(json.dumps(entry))

    # open at least to open_deg; keep a wider opening (smaller angle) if the gripper already has one
    open_deg = min(gripper["open_deg"], arm.read()[5])
    arm.move([None] * 5 + [open_deg], label="grasp open")
    try:
        for n, pose in enumerate(descent[1:], start=1):
            arm.move(pose, label=f"grasp descend {n}", table_margin=GRASP_TABLE_MARGIN)
    except motion_error as exc:
        record("descent stopped", error=str(exc))
        arm.move(descent[0], label="grasp back to hover", table_margin=GRASP_TABLE_MARGIN)
        return log
    grip = gripper["grip_command_deg"]
    carry = [pose[:5] + [grip] for pose in descent]  # keep commanding the grip while carrying
    arm.move([None] * 5 + [grip], verify_gripper=False, label="grasp close")
    time.sleep(0.4)
    reading = arm.read()[5]
    state = safe_arm.grip_state(reading, gripper)
    record("gripped", gripper_readback=reading, holding=state == "holding", grip_state=state)
    for pose in reversed(carry[:-1]):
        arm.move(pose, verify_gripper=False, label="grasp lift", table_margin=GRASP_TABLE_MARGIN)
    readings = []
    end = time.time() + hold_seconds
    while time.time() < end:
        readings.append(arm.read()[5])
        time.sleep(1.0)
    record("held at hover", gripper_readbacks=readings, still_holding=all(
        safe_arm.grip_state(r, gripper) == "holding" for r in readings))
    for pose in carry[1:]:
        arm.move(pose, verify_gripper=False, label="place descend", table_margin=GRASP_TABLE_MARGIN)
    arm.move([None] * 5 + [open_deg], verify_gripper=False, label="place open")
    for pose in reversed(descent[:-1]):
        arm.move(pose, label="place lift", table_margin=GRASP_TABLE_MARGIN)
    record("placed back")
    return log


if __name__ == "__main__":
    sys.exit(main())
