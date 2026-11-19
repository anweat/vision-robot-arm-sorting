#!/usr/bin/env python3
"""Scan the printed map from several wrist-camera poses and list the blocks found.

Runs on the board with /root/arm-app/venv-vision/bin/python:
  archive/src/tasks/scan.py               print the pose plan and clearance checks only
  archive/src/tasks/scan.py --here        process the current view, no motion
  archive/src/tasks/scan.py --execute     visit every scan pose, then return to observe_v1

Each view is located on the map by its visible markers (independent of servo backlash);
blocks are therefore located on the map to millimetres. The map's pose in the arm base
frame comes from forward kinematics + hand-eye for every view and is averaged, so base
coordinates carry about 1 cm of error until a touch test refines them.
Output goes to runs/scan_<time>/: raw and annotated frames plus scan.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[2]
for sub in ("adapters", "spatial", "vision"):
    sys.path.insert(0, str(APP / "src" / sub))

import block_detector as bd  # noqa: E402
import map_difference as md  # noqa: E402
import dofbot_kinematics as kin  # noqa: E402
import workspace  # noqa: E402
from camera_client import snapshot_after  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import TableMap, detect_markers, load_intrinsics, solve_camera_pose  # noqa: E402

FAR = [[s1, 105, 5, 5, 95] for s1 in (70, 90, 110)]
# 2026-09-24: the earlier near row [s1,85,5,5] put the gripper next to the platform and the
# return from it hit the platform; these keep >= 1 cm outside the keep-out (workspace.py)
NEAR = [[s1, 95, 10, 5, 95] for s1 in (105, 90, 75)]
OBSERVE = [90, 105, 5, 5, 95]
MIN_TIP_CLEARANCE_M = 0.05
BLOCK_M = 0.03
MERGE_RADIUS_M = 0.025
MIN_SIZE_M = 0.02  # outlines smaller than this at mid-height are sticker art, print or noise


def to_matrix(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def mean_pose(mats: list[np.ndarray]) -> np.ndarray:
    """Chordal mean of rotations and plain mean of translations."""
    u, _, vt = np.linalg.svd(sum(m[:3, :3] for m in mats))
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return to_matrix(r, np.mean([m[:3, 3] for m in mats], axis=0))


def process(image, servos, table, k, dist, cam, detector="diff"):
    markers = detect_markers(image, table.layout)
    view = {"servos": servos, "markers": sorted(markers)}
    pose = solve_camera_pose(markers, table, k, dist, prior_centre=cam.pose_in_map(servos, table).centre)
    if pose is None:
        view["error"] = "no map marker visible"
        return view, [], None
    view["reprojection_px"] = round(pose.reprojection_px, 2)
    base_T_map = cam.base_T_cam(servos) @ to_matrix(pose.R, pose.t)  # base<-cam<-map
    if detector == "color":
        blobs = bd.detect_blobs(image, exclude=list(markers.values()))
    else:  # class-agnostic: anything on the page that is not the printed map
        blobs = md.detect_objects(image, pose, k, dist, table.layout, markers)
    for b in blobs:
        try:
            bd.locate(b, pose, k, dist, BLOCK_M)
        except ValueError:
            b.partial = True
    blobs = [b for b in blobs if b.size_m >= MIN_SIZE_M]
    view["blobs"] = [{"label": b.label, "partial": b.partial, "method": b.method, "size_cm": round(b.size_m * 100, 1),
                      "map_cm": None if b.map_xy is None else [round(v * 100, 2) for v in b.map_xy]} for b in blobs]
    return view, blobs, base_T_map


def merge(observations: list[tuple[str, tuple[float, float]]]) -> list[dict]:
    clusters: list[dict] = []
    for label, xy in observations:
        for c in clusters:
            if np.linalg.norm(np.array(xy) - c["xy"]) < MERGE_RADIUS_M:
                c["points"].append(xy)
                c["labels"].append(label)
                c["xy"] = np.median(c["points"], axis=0)
                break
        else:
            clusters.append({"xy": np.array(xy), "points": [xy], "labels": [label]})
    out = []
    for c in clusters:
        pts = np.array(c["points"])
        out.append({"label": max(set(c["labels"]), key=c["labels"].count), "views": len(pts),
                    "map_xy": c["xy"], "spread_mm": round(float(np.max(np.linalg.norm(pts - c["xy"], axis=1))) * 1000, 1)})
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--here", action="store_true")
    mode.add_argument("--replay", type=Path, help="reprocess a saved run directory (no arm, no camera)")
    parser.add_argument("--settle", type=float, default=0.8)
    parser.add_argument("--detector", choices=("diff", "color"), default="diff",
                        help="diff: anything that is not the printed map (default); color: saturated blobs")
    parser.add_argument("--out", type=Path, default=APP / "runs" / time.strftime("scan_%Y%m%d_%H%M%S"))
    args = parser.parse_args()

    table, cam = TableMap(), ArmCamera()
    k, dist = load_intrinsics()
    poses = FAR + NEAR
    keep_out = workspace.load()
    for a, b in zip([OBSERVE] + poses, poses + [OBSERVE]):  # every move of the run, including the return
        clearance = kin.forward(b[:4])["tip"][2] - table.offset[2]
        margin = workspace.path_clearance(a, b, keep_out)
        print(json.dumps({"pose": b, "tip_above_map_cm": round(clearance * 100, 1),
                          "keep_out_margin_cm": round(margin * 100, 1)}))
        if clearance < MIN_TIP_CLEARANCE_M or margin < 0:
            print("refusing: a scan move comes too close to the table or the platform")
            return 2
    if args.replay:
        return replay(args.replay, table, k, dist, cam, args.detector)
    if not (args.execute or args.here):
        return 0

    from safe_arm import SafeArm

    arm = SafeArm(execute=args.execute)
    args.out.mkdir(parents=True, exist_ok=True)
    views, map_poses, observations = [], [], []
    try:
        for index, pose in enumerate([None] if args.here else poses):
            if pose is not None:
                arm.move(pose, label=f"scan {index}")
            time.sleep(args.settle)
            image, stamp = snapshot_after(time.time() - 0.01)
            servos = arm.read()[:5]
            view, blobs, base_T_map = process(image, servos, table, k, dist, cam, args.detector)
            view.update(index=index, frame_time=stamp, image=f"view_{index:02d}.jpg")
            cv2.imwrite(str(args.out / view["image"]), image)
            cv2.imwrite(str(args.out / f"view_{index:02d}_annotated.jpg"), bd.draw(image, blobs))
            views.append(view)
            print(json.dumps(view))
            if base_T_map is not None:
                map_poses.append(base_T_map)
                observations += [(b.label, b.map_xy) for b in blobs if not b.partial and b.map_xy]
        if args.execute:
            arm.move(OBSERVE, label="scan return")
    finally:
        arm.close()

    result = summarise(views, map_poses, observations, arm.cfg["soft_limits_deg"])
    (args.out / "scan.json").write_text(json.dumps(result, indent=2, default=float) + "\n")
    print("saved", args.out)
    return 0


def replay(run: Path, table, k, dist, cam, detector: str) -> int:
    from safe_arm import DEFAULT_CONFIG, load_config

    saved = json.loads((run / "scan.json").read_text())
    views, map_poses, observations = [], [], []
    for old in saved["views"]:
        image = cv2.imread(str(run / old["image"]))
        view, blobs, base_T_map = process(image, old["servos"], table, k, dist, cam, detector)
        view.update(index=old["index"], image=old["image"])
        cv2.imwrite(str(run / f"view_{old['index']:02d}_replay.jpg"), bd.draw(image, blobs))
        views.append(view)
        if base_T_map is not None:
            map_poses.append(base_T_map)
            observations += [(b.label, b.map_xy) for b in blobs if not b.partial and b.map_xy]
    result = summarise(views, map_poses, observations, load_config(DEFAULT_CONFIG)["soft_limits_deg"])
    (run / "scan_replay.json").write_text(json.dumps(result, indent=2, default=float) + "\n")
    return 0


def summarise(views, map_poses, observations, soft_limits) -> dict:
    result = {"views": views}
    if map_poses:
        base_T_map = mean_pose(map_poses)
        spread = [np.linalg.norm(m[:3, 3] - base_T_map[:3, 3]) for m in map_poses]
        result["map_in_base"] = {"matrix": base_T_map.round(5).tolist(),
                                 "origin_cm": (base_T_map[:3, 3] * 100).round(1).tolist(),
                                 "yaw_deg": round(float(np.degrees(np.arctan2(base_T_map[1, 0], base_T_map[0, 0]))), 1),
                                 "per_view_spread_cm_max": round(float(max(spread)) * 100, 1)}
        targets = []
        for t in merge(observations):
            base = (base_T_map @ np.array([*t["map_xy"], BLOCK_M / 2, 1.0]))[:3]  # block centre
            grasp = kin.inverse_within(base[0], base[1], base[2],  # tip at mid-height of the block
                                       {j: soft_limits[j] for j in (1, 2, 3, 4)})
            targets.append({"label": t["label"], "views": t["views"], "spread_mm": t["spread_mm"],
                            "map_cm": (np.array(t["map_xy"]) * 100).round(1).tolist(),
                            "base_cm": (base * 100).round(1).tolist(), "reachable": grasp is not None,
                            "grasp_ik": grasp})
        result["targets"] = targets
        print(json.dumps({"map_in_base": {k: v for k, v in result["map_in_base"].items() if k != "matrix"},
                          "targets": [{k: v for k, v in t.items() if k != "grasp_ik"} for t in targets]}, indent=1))
    return result




if __name__ == "__main__":
    sys.exit(main())
