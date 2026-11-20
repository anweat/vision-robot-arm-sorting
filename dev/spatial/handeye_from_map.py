"""Hand-eye (camera on the wrist) from board-camera views of the map, whose pose in base coordinates
is already known (touch_calibration.py writes it to app/config/map_a4_v2.json).

With the map pose fixed, every view alone gives the camera in the wrist camera-mount frame
(kin.camera_mount_pose, J5 held at 90); the spread across views is the check. A joint fit then
minimises marker-corner reprojection over all views. Unlike calibrate_from_views.py this does not
have to trade the target's height against the camera offset, which near-parallel views cannot
separate.

    python3 dev/spatial/handeye_from_map.py data/captures/2026-09-25/map_handeye [--write]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "app" / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402
from camera_model import DEFAULT_HAND_EYE, ArmCamera  # noqa: E402
from map_registration import TableMap, detect_markers, load_intrinsics, solve_camera_pose  # noqa: E402


def to_matrix(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def solve(views_dir: Path, table: TableMap, k: np.ndarray, dist: np.ndarray, prior: ArmCamera) -> dict:
    from scipy.optimize import least_squares

    base_T_map = to_matrix(np.eye(3), table.offset)
    views = []
    for rec in json.loads((views_dir / "views.json").read_text()):
        image = cv2.imread(str(views_dir / rec["image"]))
        markers = detect_markers(image, list(table.layout))
        mount = np.array(kin.camera_mount_pose(rec["readback"]))
        guess = np.linalg.inv(base_T_map) @ mount @ prior.cam2grip  # resolves the planar pose ambiguity
        pose = solve_camera_pose(markers, table, k, dist, prior_centre=guess[:3, 3])
        if pose is None:
            continue
        per_view = np.linalg.inv(mount) @ base_T_map @ np.linalg.inv(to_matrix(pose.R, pose.t))
        obj = np.vstack([np.column_stack([table.layout[m], np.zeros(4)]) for m in markers])
        img = np.vstack([markers[m] for m in markers]).astype(float)
        views.append(dict(image=rec["image"], readback=rec["readback"], mount=mount, per_view=per_view,
                          obj=obj, img=img, markers=len(markers)))
    if len(views) < 3:
        raise SystemExit(f"need at least 3 views with markers, got {len(views)}")

    def unpack(x):
        return to_matrix(cv2.Rodrigues(x[:3])[0], x[3:])

    def residuals(x):
        cam2mount, out = unpack(x), []
        for v in views:
            cam_T_map = np.linalg.inv(v["mount"] @ cam2mount) @ base_T_map
            rv, _ = cv2.Rodrigues(cam_T_map[:3, :3])
            proj, _ = cv2.projectPoints(v["obj"], rv, cam_T_map[:3, 3], k, dist)
            out.append((proj.reshape(-1, 2) - v["img"]).ravel())
        return np.concatenate(out)

    x0_m = views[0]["per_view"]
    x0 = np.r_[cv2.Rodrigues(x0_m[:3, :3])[0].ravel(), x0_m[:3, 3]]
    fit = least_squares(residuals, x0, loss="soft_l1", f_scale=3.0)
    cam2mount = unpack(fit.x)
    r = fit.fun.reshape(-1, 2)
    per_view_px, i = [], 0
    for v in views:
        n = len(v["obj"])
        per_view_px.append(float(np.sqrt((r[i:i + n] ** 2).sum(1).mean())))
        i += n
    spread = np.array([v["per_view"][:3, 3] for v in views]).std(axis=0)
    return dict(cam2mount=cam2mount, views=views, rms_px=float(np.sqrt((r ** 2).sum(1).mean())),
                per_view_px=per_view_px, per_view_spread_cm=spread * 100)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("views_dir", type=Path)
    parser.add_argument("--write", action="store_true", help=f"replace {DEFAULT_HAND_EYE.relative_to(ROOT)} (old copy kept)")
    args = parser.parse_args()
    table, (k, dist) = TableMap(), load_intrinsics()
    result = solve(args.views_dir, table, k, dist, ArmCamera())
    c = result["cam2mount"]
    for v, px in zip(result["views"], result["per_view_px"]):
        print(f"{v['image']} {v['readback'][:5]} markers {v['markers']}: alone -> camera in mount "
              f"{(v['per_view'][:3, 3] * 100).round(2)} cm; joint-fit reprojection {px:.1f} px")
    axis = math.degrees(math.acos(min(1.0, abs(c[2, 2]))))
    print(f"per-view spread (sd) {result['per_view_spread_cm'].round(2)} cm; joint fit: camera in mount "
          f"{(c[:3, 3] * 100).round(2)} cm, optical axis vs link {axis:.1f}°, reprojection rms {result['rms_px']:.1f} px")
    if not args.write:
        return
    old = DEFAULT_HAND_EYE.with_name("hand_eye_before_map_fit.json")
    old.write_text(DEFAULT_HAND_EYE.read_text())
    placement = json.loads((ROOT / "app" / "config" / "map_a4_v2.json").read_text())["placement"]
    DEFAULT_HAND_EYE.write_text(json.dumps({
        "source": str(args.views_dir.relative_to(ROOT) if args.views_dir.is_absolute() else args.views_dir),
        "gripper_frame": "dofbot_kinematics.camera_mount_pose (servo-5 axis centre, J5 held at 90; the camera does not turn with J5)",
        "method": "map views with the map pose fixed from touch calibration; joint reprojection least squares",
        "map_placement": placement,
        "reprojection_rms_px": round(result["rms_px"], 2),
        "per_view_reprojection_px": [round(p, 2) for p in result["per_view_px"]],
        "per_view_spread_cm": result["per_view_spread_cm"].round(2).tolist(),
        "t_cam_in_gripper_cm": (c[:3, 3] * 100).round(2).tolist(),
        "optical_axis_vs_link_deg": round(axis, 2),
        "cam2gripper": c.round(6).tolist(),
    }, indent=2, ensure_ascii=False) + "\n")
    print("written:", DEFAULT_HAND_EYE.relative_to(ROOT), "(previous kept as", old.name + ")")


if __name__ == "__main__":
    main()
