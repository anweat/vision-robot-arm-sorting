#!/usr/bin/env python3
"""Search for arm poses that put a bin's three stickers in the wrist camera's view, and write config/bin_views_<design>.json.

    uv run --with 'opencv-python<5' --with numpy src/tools/find_bin_views.py            # print the best per bin
    uv run --with 'opencv-python<5' --with numpy src/tools/find_bin_views.py --write    # and write the config

The field-scan poses (src/pipeline/field_scan.py) look at the map; the outer bins stand at |x| ~ 20 cm, off the
paper, and only one marker of one of them shows in those eight views (2026-10-30, simulation). So each bin gets a
view of its own. A pose qualifies when all three stickers are fully inside the frame (30 px margin, >= 50 px
across), the gripper stays 3 cm clear of the platform and the table (src/spatial/workspace.py), the soft limits hold,
and the tip never dips below the table on the way from the observe pose. Among those the one with the most room
round the stickers wins (a person set the bin down, so it will not be exactly where planned), weighed against
looking nearly straight down (a plane seen square-on gives the best PnP) and a short trip from the observe pose.

The camera model is the project's own (camera_model.py: forward kinematics + hand-eye), so this says where the
stickers should appear if the bins stand where the design plans them -- not that the real camera sees them.
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

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "pipeline", "tasks"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import bins as B  # noqa: E402
import workspace  # noqa: E402
from camera_model import ArmCamera  # noqa: E402
from map_registration import load_intrinsics  # noqa: E402

ARM = json.loads((ROOT / "config" / "arm_limits.json").read_text())
KEEP_OUT = workspace.load_keep_out(ARM)
LIMITS = {int(k): v for k, v in ARM["soft_limits_deg"].items()}
TABLE_Z = ARM["keep_out"]["table_surface_z_m"]
OBSERVE = [90, 105, 5, 5]
MARGIN_PX = 30
MIN_SIDE_PX = 50
MIN_CLEARANCE_M = 0.03     # the console's own margin is 1 cm; views are taken often, so ask for more
def out_path(design: str, relaxed: bool = False) -> Path:
    """Strict poses are what the pipeline uses; relaxed ones (explore-profile joint limits, 1.1 cm clearance) are written
    beside them under a name nothing reads, to be tried by hand first."""
    return ROOT / "config" / f"bin_views_{design}{'_relaxed' if relaxed else ''}.json"



def marker_points(b: B.Bin, bins: B.BinSet) -> dict[int, np.ndarray]:
    plane_z = TABLE_Z + bins.marker_z_m
    return {i: np.array([[*b.to_base(c), plane_z] for c in corners]) for i, corners in b.corners.items()}


def score(q, pts: dict[int, np.ndarray], cam: ArmCamera, k, dist, ids) -> dict | None:
    base_T_cam = cam.base_T_cam(list(q) + [92])
    cam_T_base = np.linalg.inv(base_T_cam)
    sides, margin = [], 1e9
    for i in ids:
        p = pts[i]
        cam_pts = p @ cam_T_base[:3, :3].T + cam_T_base[:3, 3]
        if cam_pts[:, 2].min() < 0.05:
            return None
        uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), cv2.Rodrigues(cam_T_base[:3, :3])[0], cam_T_base[:3, 3], k, dist)
        uv = uv.reshape(-1, 2)
        if (uv < MARGIN_PX).any() or (uv[:, 0] > 640 - MARGIN_PX).any() or (uv[:, 1] > 480 - MARGIN_PX).any():
            return None
        sides.append(min(np.linalg.norm(uv[j] - uv[(j + 1) % 4]) for j in range(4)))
        margin = min(margin, uv.min(), 640 - uv[:, 0].max(), 480 - uv[:, 1].max())
    if min(sides) < MIN_SIDE_PX:
        return None
    tilt = math.degrees(math.acos(float(np.clip(-base_T_cam[2, 2], -1, 1))))      # optical axis vs straight down
    return dict(tilt_deg=round(tilt, 1), min_side_px=round(min(sides)), margin_px=round(float(margin)))


# the explore profile's joint limits (motion_policy.explore_config): the course limits, J2-J4 down to -10; the field
# scan's folded near views [50, 105, 5, 0] already use J4 = 0 and a 1.1 cm clearance
RELAXED = dict(limits={1: (0, 180), 2: (-10, 180), 3: (-10, 180), 4: (-10, 180)}, clearance_m=0.011, j2=(30, 130),
               j3=(-10, 90), j4=(-10, 90))
STRICT = dict(limits=LIMITS, clearance_m=MIN_CLEARANCE_M, j2=(60, 130), j3=(0, 90), j4=(0, 90))
MODE = STRICT


def pose_ok(q) -> bool:
    if any(not MODE["limits"][i + 1][0] <= q[i] <= MODE["limits"][i + 1][1] for i in range(4)):
        return False
    if workspace.clearance(q, KEEP_OUT) < MODE["clearance_m"]:
        return False
    # the tip never dips below the table on the way from the observe pose (the console refuses such a path)
    import dofbot_kinematics as kin
    for t in np.linspace(0, 1, 25):
        mid = [a + (b - a) * t for a, b in zip(OBSERVE, q)]
        if workspace.clearance(mid, KEEP_OUT) < 0.0 or kin.forward(mid)["tip"][2] < TABLE_Z + 0.02:
            return False
    return True


def search(bins: B.BinSet) -> dict[int, list[dict]]:
    cam, (k, dist) = ArmCamera(), load_intrinsics()
    out = {}
    for number, b in bins.bins.items():
        pts = marker_points(b, bins)
        ids = [b.ids["floor"], b.ids["flange_left"], b.ids["flange_right"]]
        bearing = 90 + math.degrees(math.atan2(-b.xy[0], b.xy[1]))
        found = []
        for j1 in range(round(bearing) - 40, round(bearing) + 41, 3):
            for j2 in range(MODE["j2"][0], MODE["j2"][1] + 1, 5):
                for j3 in range(MODE["j3"][0], MODE["j3"][1] + 1, 5):
                    for j4 in range(MODE["j4"][0], MODE["j4"][1] + 1, 5):
                        q = [j1, j2, j3, j4]
                        got = score(q, pts, cam, k, dist, ids)
                        if got is None or not pose_ok(q):
                            continue
                        got.update(servos=q, travel=round(sum(abs(a - c) for a, c in zip(q, OBSERVE))),
                                   clearance_cm=round(workspace.clearance(q, KEEP_OUT) * 100, 1))
                        found.append(got)
        # room to spare round the stickers matters most: a person set the bin down, it will not be where planned
        found.sort(key=lambda r: r["tilt_deg"] + 0.05 * r["travel"] - 0.15 * min(r["margin_px"], 150))
        out[number] = found
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--design", default=B.DEFAULT_DESIGN)
    p.add_argument("--relaxed", action="store_true",
                   help="allow the explore profile's limits and a 1.1 cm clearance (for bins close to the arm)")
    p.add_argument("--write", action="store_true", help="write config/bin_views_<design>.json")
    args = p.parse_args()
    global MODE
    if args.relaxed:
        MODE = RELAXED
    bins = B.BinSet(args.design)
    result = search(bins)
    chosen = {}
    for number, rows in result.items():
        print(f"bin {number} ({bins.bins[number].category}): {len(rows)} poses show all three stickers")
        for r in rows[:3]:
            print("   ", r)
        if not rows:
            raise SystemExit(f"{number} 号桶没有任何位姿能同时看到三枚码：先调整桶的摆放（r 或 J1）")
        chosen[str(number)] = rows[0]["servos"]
    if args.write:
        out = out_path(args.design, args.relaxed)
        out.write_text(json.dumps({
            "_status": f"{time.strftime('%Y-%m-%d')}: found in the camera model (forward kinematics + hand-eye) for the "
                       f"PLANNED bin poses of config/{args.design}.json; not tried on the real arm. Re-run when the "
                       "bins are moved. J1-J4, J5 is the roll zero.",
            "design": args.design, "mode": "relaxed" if args.relaxed else "strict", "views": chosen,
            "details": {str(n): rows[0] for n, rows in result.items()}}, indent=1, ensure_ascii=False) + "\n")
        print("wrote", out)


if __name__ == "__main__":
    main()
