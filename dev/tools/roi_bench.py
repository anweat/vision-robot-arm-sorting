"""The ROI silhouette fit (app/src/vision/roi_pose.py) against the other block estimates, per view and pooled.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/roi_bench.py sim \
        [--console http://127.0.0.1:8770] [--seeds 1-8] [--jitter 0.7,0.8,0.8,0.8,0]
    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/roi_bench.py real [runs ...]

For every block of a scan's scene (app/src/pipeline/run.build_scene, views placed by spatial/view_bundle.py):
  ortho     the scene's own estimate (the 30 mm square on the block-top orthophoto)
  detector  the detector boxes' centres cast onto z = 1.5 cm, averaged over views
  roi       roi_pose.solve_top_face on each view's detector box for this block (prior: the ortho centre); the
            fits that pass its quality gates, pooled (median xy, yaw by the folded median)
sim: each against the simulator's truth (cm, deg). real: no truth -- the spread of the per-view ROI fits, and
how far each estimate lies from the others.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("tools", "tasks", "pipeline", "spatial", "vision", "adapters"):
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))
sys.path.insert(0, str(ROOT / "app" / "src"))


def fold(d: float) -> float:
    return (d + 45) % 90 - 45


def yaw_err(a: float, b: float) -> float:
    return abs(fold(a - b))


def pooled_yaw(yaws: list[float]) -> float:
    """Median of square orientations (period 90): on the 4x angle circle."""
    a = np.radians(np.array(yaws) * 4)
    return float(np.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())) / 4)


def block_estimates(run: Path) -> tuple[dict, list[dict]]:
    """The scene and, per block, its ortho / detector / ROI estimates (map cm, yaw deg)."""
    import run as R
    import scene as S
    import roi_pose as RP
    from blocks_detector import BlocksDetector
    with contextlib.redirect_stdout(io.StringIO()):
        scene = R.build_scene(run)
        scan = S.Scan(run)
        scan.register()
        scan.camera_poses()
    det = BlocksDetector()
    boxes = []                                 # per view: (bbox, cast centre cm)
    for v, (img, pose) in enumerate(zip(scan.images, scan.poses)):
        h, w = img.shape[:2]
        for d in det.detect(img, conf=R.DET_MIN_CONF):
            x0, y0, x1, y1 = d["bbox"]
            if x0 <= 2 or y0 <= 2 or x1 >= w - 3 or y1 >= h - 3:
                continue
            p = pose.pixel_to_plane(((x0 + x1) / 2, (y0 + y1) / 2), scan.k, scan.dist, z=S.BLOCK / 2)
            boxes.append((v, d["bbox"], [p[0] * 100, p[1] * 100]))
    out = []
    for cube in scene["cubes"]:
        mine = [b for b in boxes if math.dist(b[2], cube["map_cm"]) <= 2.5]
        fits = []
        for v, bbox, _ in mine:
            r = RP.solve_top_face(scan.images[v], bbox, scan.poses[v], scan.k, scan.dist,
                                  initial_yaw=cube["yaw_deg"], prior_xy=[c / 100 for c in cube["map_cm"]])
            fits.append(dict(view=v, ok=r["ok"], map_cm=r.get("map_cm"), yaw=r.get("yaw_deg"), reason=r.get("reason", "")))
        good = [f for f in fits if f["ok"]]
        roi = None
        if good:
            xy = np.median([f["map_cm"] for f in good], axis=0)
            roi = dict(map_cm=[round(float(v), 2) for v in xy], yaw=round(pooled_yaw([f["yaw"] for f in good]), 1),
                       n=len(good), spread_cm=round(float(np.median([math.dist(f["map_cm"], xy) for f in good])), 2))
        detector = [round(float(v), 2) for v in np.mean([b[2] for b in mine], axis=0)] if mine else None
        sq = cube.get("square") or cube            # the square before the scene's own cross-check moved it
        out.append(dict(ortho=dict(map_cm=sq["map_cm"], yaw=sq["yaw_deg"]), detector=detector, roi=roi,
                        final=dict(map_cm=cube["map_cm"], yaw=cube["yaw_deg"], result=(cube.get("checks") or {}).get("result")),
                        fits=fits, boxes=len(mine)))
    return scene, out


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def summary(name: str, errs: list, yaws: list | None = None) -> str:
    s = f"{name:9s} n {len(errs):3d}  centre cm median {q(errs, .5):.2f} p90 {q(errs, .9):.2f} max {max(errs) if errs else float('nan'):.2f}"
    if yaws:
        s += f"  yaw median {q(yaws, .5):.1f} p90 {q(yaws, .9):.1f} max {max(yaws):.1f}"
    return s


def sim(a) -> int:
    import console_step as CS
    CS.BASE = a.console.rstrip("/")
    from flow import stages as ST
    from flow.session import FlowSession
    tok = CS.call("/api/bootstrap")["token"]
    CS.call("/api/sim/faults", {"pose_jitter_deg": [float(v) for v in a.jitter.split(",")]}, token=tok)
    err = {k: [] for k in ("ortho", "detector", "roi", "final")}
    yaw = {k: [] for k in ("ortho", "roi", "final")}
    rows = []
    try:
        for seed in seeds(a.seeds):
            world = CS.call("/api/sim/reset", {"count": a.blocks, "seed": seed}, token=tok)
            truth = world["blocks"]
            sess = FlowSession(ROOT / "data" / "sim" / "roi_bench" / f"s{seed}")
            shutil.rmtree(sess.path, ignore_errors=True)
            sess = FlowSession(ROOT / "data" / "sim" / "roi_bench" / f"s{seed}")
            with contextlib.redirect_stdout(io.StringIO()):
                ST.scan(sess, a.console, really=True, refine=a.refine)
            _, ests = block_estimates(sess.file("scan"))
            for e in ests:
                t = min(truth, key=lambda b: math.dist(b["map_cm"], e["ortho"]["map_cm"]))
                if math.dist(t["map_cm"], e["ortho"]["map_cm"]) > 4:
                    continue
                row = dict(seed=seed, truth=t["map_cm"], truth_yaw=t["yaw_deg"], up=t.get("up"),
                           ortho=math.dist(e["ortho"]["map_cm"], t["map_cm"]), ortho_yaw=yaw_err(e["ortho"]["yaw"], t["yaw_deg"]),
                           detector=math.dist(e["detector"], t["map_cm"]) if e["detector"] else None,
                           roi=math.dist(e["roi"]["map_cm"], t["map_cm"]) if e["roi"] else None,
                           roi_yaw=yaw_err(e["roi"]["yaw"], t["yaw_deg"]) if e["roi"] else None,
                           roi_n=e["roi"]["n"] if e["roi"] else 0, boxes=e["boxes"],
                           final=math.dist(e["final"]["map_cm"], t["map_cm"]), final_yaw=yaw_err(e["final"]["yaw"], t["yaw_deg"]),
                           result=e["final"]["result"],
                           fails=[f["reason"][:40] for f in e["fits"] if not f["ok"]])
                rows.append(row)
                err["final"].append(row["final"])
                yaw["final"].append(row["final_yaw"])
                err["ortho"].append(row["ortho"])
                yaw["ortho"].append(row["ortho_yaw"])
                if row["detector"] is not None:
                    err["detector"].append(row["detector"])
                if row["roi"] is not None:
                    err["roi"].append(row["roi"])
                    yaw["roi"].append(row["roi_yaw"])
                print(f"seed {seed} {t['category'][:4]} up {str(t.get('up'))[:12]:12s} yaw {t['yaw_deg']:6.1f}: ortho "
                      f"{row['ortho']:.2f} cm / {row['ortho_yaw']:.0f} deg, detector "
                      f"{row['detector'] if row['detector'] is None else round(row['detector'], 2)}, roi "
                      f"{None if row['roi'] is None else round(row['roi'], 2)} cm / {row['roi_yaw']} deg "
                      f"({row['roi_n']}/{row['boxes']} views ok) -> scene {row['final']:.2f} cm / {row['final_yaw']:.1f} deg "
                      f"[{row['result']}]", flush=True)
    finally:
        CS.call("/api/sim/faults", {"pose_jitter_deg": [0, 0, 0, 0, 0]}, token=tok)
    print()
    print(summary("ortho", err["ortho"], yaw["ortho"]))
    print(summary("detector", err["detector"]))
    print(summary("roi", err["roi"], yaw["roi"]))
    print(summary("scene", err["final"], yaw["final"]))
    from collections import Counter
    print("cross-check results:", dict(Counter(r["result"] for r in rows)))
    out = ROOT / "data" / "sim" / "roi_bench.json"
    out.write_text(json.dumps(rows, indent=1, ensure_ascii=False))
    return 0


def real(a) -> int:
    runs = [Path(r) for r in a.runs] or sorted(
        p for p in (ROOT / "data" / "runs" / "pipeline").iterdir() if (p / "views.json").exists())
    spread, ortho_roi, ortho_roi_yaw, det_roi, fails, total, okc = [], [], [], [], {}, 0, 0
    rows = []
    for run in runs:
        _, ests = block_estimates(run)
        for e in ests:
            total += len(e["fits"])
            okc += sum(f["ok"] for f in e["fits"])
            for f in e["fits"]:
                if not f["ok"]:
                    key = f["reason"].split(" ")[0] + " " + (f["reason"].split(" ")[1] if " " in f["reason"] else "")
                    fails[key] = fails.get(key, 0) + 1
            if e["roi"]:
                if e["roi"]["n"] >= 2:
                    spread.append(e["roi"]["spread_cm"])
                ortho_roi.append(math.dist(e["roi"]["map_cm"], e["ortho"]["map_cm"]))
                ortho_roi_yaw.append(yaw_err(e["roi"]["yaw"], e["ortho"]["yaw"]))
                if e["detector"]:
                    det_roi.append(math.dist(e["roi"]["map_cm"], e["detector"]))
            rows.append(dict(run=run.name, **e))
        print(run.name, len(ests), flush=True)
    print()
    print(f"per-view ROI fits: {okc}/{total} pass the gates; failures {sorted(fails.items(), key=lambda kv: -kv[1])[:6]}")
    print(f"blocks with a pooled ROI fit: {len(ortho_roi)}; with >= 2 views: {len(spread)}")
    print(f"ROI fits of one block, spread around their median: median {q(spread, .5):.2f} p90 {q(spread, .9):.2f} cm")
    print(f"ROI vs ortho: centre median {q(ortho_roi, .5):.2f} p90 {q(ortho_roi, .9):.2f} max {max(ortho_roi):.2f} cm; "
          f"yaw median {q(ortho_roi_yaw, .5):.1f} p90 {q(ortho_roi_yaw, .9):.1f} deg; >0.8 cm apart {sum(d > .8 for d in ortho_roi)}")
    print(f"ROI vs detector: centre median {q(det_roi, .5):.2f} p90 {q(det_roi, .9):.2f} cm")
    out = ROOT / "data" / "reports" / "roi_bench_real.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rows, indent=1, ensure_ascii=False, default=float))
    return 0


def seeds(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("where", choices=("sim", "real"))
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--console", default="http://127.0.0.1:8770")
    ap.add_argument("--seeds", default="1-8")
    ap.add_argument("--blocks", type=int, default=4)
    ap.add_argument("--jitter", default="0.7,0.8,0.8,0.8,0")
    ap.add_argument("--refine", action="store_true", help="close looks at the blocks the scene flags (flow scan refine: true)")
    a = ap.parse_args()
    return sim(a) if a.where == "sim" else real(a)


if __name__ == "__main__":
    sys.exit(main())
