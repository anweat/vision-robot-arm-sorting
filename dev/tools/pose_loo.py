"""Held-out map codes on the real scans: how well each way of finding a view's camera pose predicts codes it was
not given. No truth needed: the printed map is the truth.

    uv run --with 'opencv-python<5' --with numpy python dev/tools/pose_loo.py [data/runs/pipeline/<run> ...]

For every view that sees >= 3 codes, it is given only `keep` of them (0, 1 or 2; every choice) and the rest are
held out; the other views keep all theirs. Then its pose is found
  old     as scene.py did before 2026-11-20: 2 codes -> PnP on them; fewer -> forward kinematics + hand-eye on
          the median placement of the views that see >= 2 (registration as before, the held-out view reduced);
  bundle  spatial/view_bundle.py with the reduced view (all views together, the servo pose as a prior).
Scored on the held-out codes: corner reprojection (px) and their centres cast onto the table (mm from the
layout). This is the error a block on that part of the table would get from that view.
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "pipeline", "vision"):
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import scene as SC  # noqa: E402
import view_bundle as VB  # noqa: E402
from map_registration import CameraPose, solve_camera_pose  # noqa: E402


def median_placement(scan: SC.Scan, markers: list[dict]) -> np.ndarray:
    base0 = SC.to_matrix(np.eye(3), scan.table.offset)
    t, yaw = [], []
    for v, m in zip(scan.views, markers):
        if len(m) < 2:
            continue
        bc = scan.cam.base_T_cam(v["readback"])
        pose = solve_camera_pose(m, scan.table, scan.k, scan.dist, prior_centre=(np.linalg.inv(base0) @ bc)[:3, 3])
        if pose is None or pose.reprojection_px > 2.5:
            continue
        b = bc @ SC.to_matrix(pose.R, pose.t)
        t.append(b[:3, 3])
        yaw.append(math.atan2(b[1, 0], b[0, 0]))
    if not t:
        return base0
    y = float(np.median(yaw))
    m = SC.to_matrix(np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]),
                     np.median(np.array(t), axis=0))
    m[2, 3] = scan.table.offset[2]
    return m


def held_errors(pose: CameraPose, held: dict, scan: SC.Scan) -> tuple[list, list]:
    px, mm = [], []
    for mid, det in held.items():
        obj = np.column_stack([scan.table.layout[mid], np.zeros(4)])
        proj = pose.project(obj, scan.k, scan.dist)
        px.append(float(np.sqrt(np.mean(np.sum((proj - det) ** 2, axis=1)))))
        try:
            c = pose.pixel_to_plane(det.mean(axis=0), scan.k, scan.dist, 0.0)
            mm.append(float(np.linalg.norm(c[:2] - scan.table.layout[mid].mean(axis=0))) * 1000)
        except ValueError:
            pass
    return px, mm


def run_one(run: Path, out: list) -> None:
    scan = SC.Scan(run)
    for i, m in enumerate(scan.markers):
        if len(m) < 3:
            continue
        ids = sorted(m)
        for keep in (0, 1, 2):
            for kept in itertools.combinations(ids, keep):
                markers = [dict(x) for x in scan.markers]
                markers[i] = {k: m[k] for k in kept}
                held = {k: m[k] for k in ids if k not in kept}
                bc = scan.cam.base_T_cam(scan.views[i]["readback"])
                # old
                place = median_placement(scan, markers)
                pose = None
                if keep >= 2:
                    fk = np.linalg.inv(bc) @ place
                    pose = solve_camera_pose(markers[i], scan.table, scan.k, scan.dist, prior_centre=-fk[:3, :3].T @ fk[:3, 3])
                    if pose is not None and pose.reprojection_px > 2.5:
                        pose = None
                if pose is None:
                    fk = np.linalg.inv(bc) @ place
                    pose = CameraPose(fk[:3, :3], fk[:3, 3])
                opx, omm = held_errors(pose, held, scan)
                # bundle
                res = VB.solve([scan.cam.base_T_cam(v["readback"]) for v in scan.views], markers, scan.table,
                               scan.k, scan.dist, place)
                bpx, bmm = held_errors(res.poses[i], held, scan)
                out.append(dict(run=run.name, view=i, keep=keep, kept=list(kept), old_px=opx, old_mm=omm,
                                bundle_px=bpx, bundle_mm=bmm))


def q(v: list, p: float) -> float:
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def main() -> int:
    runs = [Path(a) for a in sys.argv[1:]] or sorted(
        p for p in (ROOT / "data" / "runs" / "pipeline").iterdir() if (p / "views.json").exists())
    rows: list = []
    for r in runs:
        run_one(r, rows)
        print(r.name, len(rows), flush=True)
    print()
    for keep in (0, 1, 2):
        sel = [r for r in rows if r["keep"] == keep]
        for name in ("old", "bundle"):
            px = [e for r in sel for e in r[f"{name}_px"]]
            mm = [e for r in sel for e in r[f"{name}_mm"]]
            print(f"view given {keep} code(s), {name:6s}: held-out corners px median {q(px, .5):5.1f} p90 {q(px, .9):5.1f}; "
                  f"table mm median {q(mm, .5):4.1f} p90 {q(mm, .9):4.1f}  ({len(mm)} codes, {len(sel)} trials)")
    out = ROOT / "data" / "reports" / "pose_loo.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main())
