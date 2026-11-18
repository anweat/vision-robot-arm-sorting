"""Offline check of the wrist-camera model against recorded images, with no pose solve.

The model under test is the one the pipeline and the simulator share: intrinsics and distortion
(config/calibration/camera_intrinsics.json), base_T_cam = FK(readback) @ cam2gripper
(spatial/camera_model.py, config/calibration/hand_eye.json) and the printed map placed in the base frame.

For every recorded image that has a servo readback, the ArUco map markers are detected and:

  * reprojection: each detected marker's 4 layout corners (map frame, Z = 0) are put in the base frame,
    projected with cv2.projectPoints through the model at that readback, and compared with the detected
    corners (pixels). Nothing is fitted to the image.
  * back-projection: each detected marker centre is cast through the same camera pose onto the table
    plane and compared with its layout centre (cm, map frame).

Both are done for two map placements:
  touch       base = map + (dx, dy, dz) from the map JSON (touch calibration 2026-09-25, no yaw);
  registered  base = Rz(yaw) @ map + origin from the run's scene.json `registration` x / y / yaw, with z
              kept at the touch value. That registration was solved from the same views through FK, so it
              absorbs a constant camera bias: the gap between the two placements is "map moved or
              mean model bias", what remains under `registered` is per-view model error.
`rel` columns: the corner error left after subtracting each view's mean pixel offset (views with >= 2
markers). A pose error is mostly such a shift; what is left grows with radius if k1 / k2 or f are wrong.
Per run (or capture folder), the table z that minimises the median back-projection error is found by a
1-D search and reported next to the calibrated -1.42 cm.

Pose families: handeye (data/captures/*/map_handeye, J2 95-115 / J3 5-9 / J4 ~6), refine (views.json
`refine_for` or a look pose with J4 >= 4), near (J2 >= 100, J3 3-7, J4 <= 2: the folded near views of
field_scan.SCAN_POSES), high (the other scan views), touch (teleop touch samples).

    uv run --with 'opencv-contrib-python<5' --with numpy python src/tools/validate_camera.py --placement both
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

from camera_model import ArmCamera  # noqa: E402
from map_registration import CameraPose, TableMap, detect_markers, load_intrinsics  # noqa: E402

DATA = ROOT / "data"
SKIP_RUNS = {"20261009_143843"}                      # registration flipped (yaw -176.6 deg)
OVERWRITTEN_VIEW06 = {"20261106_154755", "20261106_155229", "20261106_160044", "20261106_160241", "20261106_160502"}
TOUCH_SESSION = "20260925_005756_78eb"             # the touch calibration the map placement comes from
RADIAL_BINS = [(0, 100), (100, 200), (200, 300), (300, 401)]
Z_SEARCH_CM = np.arange(-4.0, 2.0001, 0.05)        # table z in base, cm


def collect_views(limit: int | None) -> list[dict]:
    """Every (image, readback) pair found, with its source group, date and registration if any."""
    views = []

    def add_folder(folder: Path, source: str, family_hint: str | None = None):
        vj = folder / "views.json"
        if not vj.exists():
            return
        reg = None
        if (folder / "scene.json").exists():
            reg = json.loads((folder / "scene.json").read_text()).get("registration")
        seen06 = False
        for v in json.loads(vj.read_text()):
            if not v.get("readback") or not (folder / v["image"]).exists():
                continue
            if folder.name in OVERWRITTEN_VIEW06 and v["image"] == "view_06.jpg" and not seen06:
                seen06 = True                      # its jpg was overwritten by the refine view
                continue
            views.append(dict(path=folder / v["image"], readback=v["readback"], source=source,
                              group=f"{source}/{folder.name}", date=date_of(folder), reg=reg,
                              family=family_hint or family_of(v)))

    for run in sorted((DATA / "runs" / "pipeline").glob("*/")):
        if run.name not in SKIP_RUNS:
            add_folder(run, "pipeline")
    for rnd in sorted((DATA / "datasets" / "blocks4").glob("*/round_*/")):
        add_folder(rnd, "blocks4")
    for cap in sorted((DATA / "captures").glob("*/*/")):
        add_folder(cap, "capture", "handeye" if cap.name == "map_handeye" else "calib")
    for sample in sorted((DATA / "teleop" / TOUCH_SESSION).glob("sample_*.json")):
        s = json.loads(sample.read_text())
        img = sample.parent / (s.get("image") or sample.with_suffix(".jpg").name)
        if img.exists() and (s.get("state") or {}).get("pose"):
            views.append(dict(path=img, readback=s["state"]["pose"], source="teleop", group=f"teleop/{sample.parent.name}",
                              date=date_of(sample.parent), reg=None, family="touch"))
    return views[:limit] if limit else views


def date_of(folder: Path) -> str:
    for part in [folder.name, folder.parent.name, folder.parent.parent.name]:
        digits = part.replace("-", "")[:8]
        if digits.isdigit() and digits.startswith("20"):
            return f"{digits[:4]}-{digits[4:6]}-{digits[6:8]}"
    return "unknown"


def family_of(v: dict) -> str:
    j2, j3, j4 = v["readback"][1:4]
    if v.get("refine_for") is not None or j4 >= 4:
        return "refine"
    if j2 >= 100 and 3 <= j3 <= 7 and j4 <= 2:
        return "near"
    return "high"


def rz(yaw_deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def placements(view: dict, table: TableMap, wanted: str) -> dict[str, np.ndarray]:
    """base_T_map for each placement that applies to this view."""
    out = {}
    if wanted in ("touch", "both"):
        m = np.eye(4)
        m[:3, 3] = table.offset
        out["touch"] = m
    if wanted in ("registered", "both") and view["reg"]:
        m = np.eye(4)
        m[:3, :3] = rz(view["reg"]["yaw_deg"])
        x, y, _ = view["reg"]["map_origin_base_cm"]
        m[:3, 3] = [x / 100, y / 100, table.offset[2]]
        out["registered"] = m
    return out


def camera_pose(base_T_cam: np.ndarray, base_T_map: np.ndarray) -> CameraPose:
    cam_T_map = np.linalg.inv(base_T_cam) @ base_T_map
    return CameraPose(cam_T_map[:3, :3], cam_T_map[:3, 3])


def evaluate(views: list[dict], table: TableMap, wanted: str) -> tuple[list[dict], list[dict], list[dict]]:
    k, dist = load_intrinsics()
    cam = ArmCamera()
    corners, centres, zfit = [], [], []
    cast = defaultdict(list)                 # (placement, group) -> [(undistorted ray in map, centre, layout xy)]
    for v in views:
        img = cv2.imread(str(v["path"]))
        found = detect_markers(img, table.layout) if img is not None else {}
        v["n_markers"] = len(found)
        base_T_cam = cam.base_T_cam(v["readback"])
        for pname, base_T_map in placements(v, table, wanted).items():
            pose = camera_pose(base_T_cam, base_T_map)
            meta = dict(placement=pname, source=v["source"], group=v["group"], image=v["path"].name,
                        date=v["date"], family=v["family"], readback=list(v["readback"][:5]))
            first = len(corners)
            for mid, det in found.items():
                layout = table.layout[mid]
                proj = pose.project(np.column_stack([layout, np.zeros(4)]), k, dist)
                for ci in range(4):
                    err = proj[ci] - det[ci]
                    corners.append(dict(meta, marker=mid, corner=ci, det=det[ci].round(2).tolist(),
                                        proj=proj[ci].round(2).tolist(), err_px=round(float(np.hypot(*err)), 3),
                                        d_px=err.round(3).tolist(),
                                        r_px=round(float(np.hypot(det[ci][0] - 320, det[ci][1] - 240)), 1)))
                uv = det.mean(axis=0)
                truth = layout.mean(axis=0)
                try:
                    hit = pose.pixel_to_plane(uv, k, dist, 0.0)
                except ValueError:
                    continue
                d = (hit[:2] - truth) * 100
                centres.append(dict(meta, marker=mid, err_cm=round(float(np.hypot(*d)), 3),
                                    dx_cm=round(float(d[0]), 3), dy_cm=round(float(d[1]), 3),
                                    r_px=round(float(np.hypot(uv[0] - 320, uv[1] - 240)), 1)))
                x, y = cv2.undistortPoints(np.array([[uv]], float), k, dist).ravel()
                cast[(pname, v["group"])].append((pose.centre, pose.R.T @ np.array([x, y, 1.0]), truth))
            if len(found) >= 2:   # residual left after removing this view's mean image shift (no fit beyond that)
                rows = corners[first:]
                shift = np.mean([r["d_px"] for r in rows], axis=0)
                for r in rows:
                    r["err_px_shift_removed"] = round(float(np.hypot(*(np.array(r["d_px"]) - shift))), 3)
    for (pname, group), rays in sorted(cast.items()):
        best = None
        for z_cm in Z_SEARCH_CM:
            dz = z_cm / 100 - table.offset[2]          # plane height in the map frame
            errs = [np.hypot(*((o + (dz - o[2]) / r[2] * r)[:2] - t)) for o, r, t in rays]
            med = float(np.median(errs)) * 100
            if best is None or med < best[1]:
                best = (z_cm, med)
        zfit.append(dict(placement=pname, group=group, n=len(rays), best_table_z_cm=round(best[0], 2),
                         median_cm_at_best=round(best[1], 3), calibrated_z_cm=round(table.offset[2] * 100, 2)))
    return corners, centres, zfit


def stats(values: list[float]) -> dict:
    if not values:
        return dict(n=0)
    a = np.asarray(values, float)
    return dict(n=int(a.size), median=round(float(np.median(a)), 3), p90=round(float(np.percentile(a, 90)), 3))


def radial_bin(r: float) -> str:
    return next(f"{lo}-{hi - 1}" for lo, hi in RADIAL_BINS if lo <= r < hi)


def summarise(corners: list[dict], centres: list[dict]) -> dict:
    keys = {"all": lambda r: "all", "family": lambda r: r["family"], "date": lambda r: r["date"],
            "radius_px": lambda r: radial_bin(r["r_px"]), "group": lambda r: r["group"],
            "date_family": lambda r: f"{r['date']} {r['family']}"}
    out = {}
    for pname in sorted({r["placement"] for r in corners}):
        cs = [r for r in corners if r["placement"] == pname]
        ms = [r for r in centres if r["placement"] == pname]
        out[pname] = {}
        for kname, key in keys.items():
            groups = defaultdict(lambda: dict(px=[], rel=[], cm=[], dx=[], dy=[]))
            for r in cs:
                groups[key(r)]["px"].append(r["err_px"])
                if "err_px_shift_removed" in r:
                    groups[key(r)]["rel"].append(r["err_px_shift_removed"])
            for r in ms:
                g = groups[key(r)]
                g["cm"].append(r["err_cm"]), g["dx"].append(r["dx_cm"]), g["dy"].append(r["dy_cm"])
            out[pname][kname] = {g: dict(px=stats(v["px"]), px_shift_removed=stats(v["rel"]), cm=stats(v["cm"]),
                                         mean_dx_cm=round(float(np.mean(v["dx"])), 3) if v["dx"] else None,
                                         mean_dy_cm=round(float(np.mean(v["dy"])), 3) if v["dy"] else None)
                                 for g, v in sorted(groups.items())}
    return out


def print_table(summary: dict, zfit: list[dict], views: list[dict]) -> None:
    used = [v for v in views if v["n_markers"]]
    print(f"views with a readback: {len(views)}, with >= 1 map marker: {len(used)}")
    for pname, by in summary.items():
        print(f"\n== placement: {pname}")
        for kname in ("all", "family", "date", "radius_px", "date_family"):
            print(f"-- by {kname}")
            print(f"   {'group':24s} {'corners':>7s} {'med px':>7s} {'p90 px':>7s} {'markers':>7s} {'med cm':>7s} "
                  f"{'p90 cm':>7s} {'dx cm':>6s} {'dy cm':>6s} {'rel med':>7s} {'rel p90':>7s}")
            for g, s in by[kname].items():
                px, cm = s["px"], s["cm"]
                print(f"   {g:24s} {px['n']:7d} {px.get('median', float('nan')):7.2f} {px.get('p90', float('nan')):7.2f} "
                      f"{cm['n']:7d} {cm.get('median', float('nan')):7.2f} {cm.get('p90', float('nan')):7.2f} "
                      f"{s['mean_dx_cm'] if s['mean_dx_cm'] is not None else float('nan'):6.2f} "
                      f"{s['mean_dy_cm'] if s['mean_dy_cm'] is not None else float('nan'):6.2f} "
                      f"{s['px_shift_removed'].get('median', float('nan')):7.2f} {s['px_shift_removed'].get('p90', float('nan')):7.2f}")
    print("\n-- table z minimising median back-projection error (calibrated -1.42 cm)")
    for z in zfit:
        print(f"   {z['placement']:10s} {z['group']:34s} n={z['n']:3d} z={z['best_table_z_cm']:6.2f} cm "
              f"-> {z['median_cm_at_best']:.2f} cm")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=DATA / "reports" / f"camera_validation_{time.strftime('%Y%m%d')}.json")
    ap.add_argument("--limit", type=int, help="only the first N views (quick check)")
    ap.add_argument("--placement", choices=("touch", "registered", "both"), default="both")
    ap.add_argument("--map", default="map_a4_v2", help="config/<name>.json; all recorded sessions used v2")
    args = ap.parse_args()

    table = TableMap(ROOT / "config" / f"{args.map}.json")
    views = collect_views(args.limit)
    corners, centres, zfit = evaluate(views, table, args.placement)
    summary = summarise(corners, centres)
    print_table(summary, zfit, views)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(dict(
        created=time.strftime("%Y-%m-%d %H:%M:%S"), map=args.map, placement=args.placement,
        views=[dict(group=v["group"], image=v["path"].name, family=v["family"], date=v["date"],
                    readback=v["readback"][:5], n_markers=v["n_markers"]) for v in views],
        summary=summary, table_z=zfit, corners=corners, centres=centres), indent=1, ensure_ascii=False))
    print(f"\nwrote {args.out.relative_to(ROOT) if args.out.is_relative_to(ROOT) else args.out}")


if __name__ == "__main__":
    main()
