"""How joint errors turn into a miss at the grasp, and how much a close look before the grasp takes back.

A geometric Monte Carlo on the project's own model (dofbot_kinematics, hand_eye.json, the planner's grasp
pose); no images, no simulator console. For a block at (x, y):

  scan     the block's top centre is seen from every field_scan.SCAN_POSES view that has it in frame; the
           camera really stands at FK(q + e_view), the estimate casts the pixel back through FK(q) onto the
           top plane, and the views are averaged (what locate does with many views).
  look     one extra view from the closest pose that centres the block in the image at 10-20 cm (the
           camera sits 5.7 cm off the tool axis, so at the hover the block is ~150 px below the frame).
  grasp    the planner's grasp for the estimate is commanded; the pads really go to FK(q_grasp + e_grasp).
           The miss is the pads' xy against the block's true centre.

Joint error models, each per degree of error (results scale ~linearly at this size):
  zero     one constant offset per joint for the whole run (servo zero / horn misalignment)
  sag      offset proportional to the horizontal lever beyond J2-J4 (gravity on backlash + compliance)
  random   independent per pose (which side of the backlash the servo stopped on)

    uv run --with numpy --with 'opencv-contrib-python<5' python dev/sim/error_budget.py [--trials 200] [--deg 1]
Results say what the model predicts, not what the arm does.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("", "spatial", "pipeline"):
    sys.path.insert(0, str(ROOT / "dev" / sub) if sub else str(ROOT / "app" / "src"))
    if sub:
        sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
from camera_model import ArmCamera  # noqa: E402

import planning  # noqa: E402
from planning import checks as K  # noqa: E402
from planning import config as C  # noqa: E402
from planning import grasp as G  # noqa: E402
from planning import gripper as GR  # noqa: E402
from planning import solver as V  # noqa: E402

SCAN_POSES = [[100, 125, 0, 0], [70, 125, 0, 0], [110, 105, 15, 0], [80, 120, 0, 0], [40, 90, 30, 0],
              [62, 106, 5, 0], [90, 105, 5, 0], [118, 106, 5, 0]]      # pipeline/field_scan.py, 2026-11-14
F_PX, W, H = 977.383, 640, 480
LEVER_SCALE_M = 0.10
DETECT_PX = 1.5            # pixel noise of a block-centre detection (within-view residual of the map corners ~3 px / corner)

CAM = ArmCamera()


def lever(q4) -> np.ndarray:
    p2 = math.radians(90 - q4[1])
    p3 = p2 + math.radians(90 - q4[2])
    p4 = p3 + math.radians(90 - q4[3])
    a, b, c = kin.UPPER_ARM * math.sin(p2), kin.FOREARM * math.sin(p3), kin.WRIST_TO_SERVO5 * math.sin(p4)
    return np.array([0.0, a + b + c, b + c, c]) / LEVER_SCALE_M


def project(T: np.ndarray, X: np.ndarray):
    p = np.linalg.inv(T) @ np.r_[X, 1.0]
    if p[2] <= 0.03:
        return None
    return np.array([F_PX * p[0] / p[2] + W / 2, F_PX * p[1] / p[2] + H / 2]), p[2]


def cast(T: np.ndarray, uv: np.ndarray, z: float) -> np.ndarray:
    ray = T[:3, :3] @ np.array([(uv[0] - W / 2) / F_PX, (uv[1] - H / 2) / F_PX, 1.0])
    o = T[:3, 3]
    return o + (z - o[2]) / ray[2] * ray


def in_frame(uv, margin=40) -> bool:
    return margin < uv[0] < W - margin and margin < uv[1] < H - margin


def look_pose(X: np.ndarray, near_q4) -> list | None:
    """J1-J4 nearest `near_q4` (joint distance) that put X within 30 px of the image centre at 10-20 cm."""
    bearing = math.atan2(-X[0], X[1])
    r0 = math.hypot(X[0], X[1])
    best = None
    for pitch in range(120, 181, 5):
        for dr in np.arange(-0.10, 0.101, 0.005):
            for z in np.arange(0.04, 0.22, 0.01):
                r = r0 + dr
                q = kin.inverse(-r * math.sin(bearing), r * math.cos(bearing), z, pitch)
                if q is None or not all(C.LIMITS[j][0] <= q[j - 1] <= C.LIMITS[j][1] for j in (1, 2, 3, 4)):
                    continue
                pr = project(CAM.base_T_cam(q + [90]), X)
                if pr is None or not (0.10 <= pr[1] <= 0.20):
                    continue
                # turn J1 so the block is centred across the image too
                du = pr[0][0] - W / 2
                q1 = list(q)
                q1[0] = q[0] - math.degrees(math.atan2(du, F_PX)) * 0.9
                pr = project(CAM.base_T_cam(q1 + [90]), X)
                if pr is None or math.hypot(*(pr[0] - [W / 2, H / 2])) > 30:
                    continue
                if K.platform_keep_out(q1) < 0:
                    continue
                cost = max(abs(a - b) for a, b in zip(q1, near_q4))
                if best is None or cost < best[0]:
                    best = (cost, q1)
    return None if best is None else best[1]


def errors(kind: str, deg: float, rng, poses: list) -> list[np.ndarray]:
    """Joint error (deg, J1-J4) at each pose of one trial."""
    if kind == "zero":
        e = rng.normal(0, deg, 4)
        return [e for _ in poses]
    if kind == "sag":
        s = rng.normal(0, deg, 4)
        return [s * lever(q) for q in poses]
    if kind == "random":
        return [rng.normal(0, deg, 4) for _ in poses]
    raise ValueError(kind)


def locate(views: list, errs: list, X: np.ndarray, rng) -> np.ndarray | None:
    est = []
    for q, e in zip(views, errs):
        nominal = CAM.base_T_cam(list(q) + [90])
        pr = project(CAM.base_T_cam(list(np.add(q, e)) + [90]), X)          # where the camera really is
        if pr is None or not in_frame(pr[0]):
            continue
        est.append(cast(nominal, pr[0] + rng.normal(0, DETECT_PX, 2), X[2]))
    return np.mean(est, axis=0) if est else None


def pads_at(q4, j5, delta) -> np.ndarray:
    return np.array(GR.pad_centre(list(q4) + [j5], delta))


def trial(w, X, grasp, kind, deg, rng, look_q):
    """(scan miss, look miss) in metres for one draw of the joint errors."""
    t = w.tuning
    seen = [q for q in SCAN_POSES if (lambda p: p is not None and in_frame(p[0]))(project(CAM.base_T_cam(q + [90]), X))]
    poses = seen + [look_q, grasp.grasp[:4]]
    errs = errors(kind, deg, rng, poses)
    e_scan, e_look, e_grasp = errs[:len(seen)], errs[len(seen)], errs[-1]
    out = []
    for est in (locate(seen, e_scan, X, rng), locate([look_q], [e_look], X, rng)):
        if est is None:
            out.append(None)
            continue
        pad = [est[0], est[1], t.table_z + float(t.trims["pad_centre_height_m"])]
        q = V.solve_tcp(pad, grasp.pitch, grasp.j5, GR.contact())
        if q is None:
            out.append(None)
            continue
        real = pads_at(np.add(q, e_grasp), grasp.j5, GR.contact())
        out.append(math.hypot(real[0] - X[0], real[1] - X[1]))
    return out


def stats(v) -> dict:
    v = np.array([x for x in v if x is not None]) * 100
    if not len(v):
        return dict(n=0)
    return dict(n=int(len(v)), median_cm=round(float(np.median(v)), 2), p90_cm=round(float(np.percentile(v, 90)), 2))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--deg", type=float, default=1.0, help="sigma of the joint error, deg")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / f"error_budget_{time.strftime('%Y%m%d')}.json")
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    w = planning.world()
    tz = w.tuning.table_z
    blocks = [(0.0, 0.15), (-0.06, 0.18), (0.05, 0.21), (0.0, 0.24)]
    report = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), deg=a.deg, trials=a.trials, detect_px=DETECT_PX, blocks={})
    for bx, by in blocks:
        X = np.array([bx, by, tz + C.BLOCK])
        cs = G.candidates(w, G.Target(bx, by, 0.0, C.BLOCK, "b", "b"), limit=1)
        if not cs:
            report["blocks"][f"{bx * 100:.0f},{by * 100:.0f}"] = dict(note="no grasp")
            continue
        g = cs[0]
        look = look_pose(X, g.hover[:4])
        row = dict(grasp=[round(v, 1) for v in g.grasp], pitch=g.pitch, look=None if look is None else [round(v, 1) for v in look],
                   scan_views=sum(1 for q in SCAN_POSES if (lambda p: p is not None and in_frame(p[0]))(project(CAM.base_T_cam(q + [90]), X))))
        for kind in ("zero", "sag", "random"):
            res = [trial(w, X, g, kind, a.deg, rng, look) for _ in range(a.trials)] if look else []
            row[kind] = dict(scan=stats([r[0] for r in res]), look=stats([r[1] for r in res]))
        report["blocks"][f"{bx * 100:.0f},{by * 100:.0f}"] = row
        print(f"block ({bx * 100:.0f}, {by * 100:.0f}) cm: grasp {row['grasp']} look {row['look']} views {row['scan_views']}")
        for kind in ("zero", "sag", "random"):
            print(f"   {kind:6s} scan {row[kind]['scan']}  look {row[kind]['look']}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", a.out.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
