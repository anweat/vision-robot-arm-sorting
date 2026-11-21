"""Stack-aware scan (app/src/pipeline/stack_scan.py) against the simulator's truth, and on the real scans.

    # simulator (start it first: launch config arm-sim-stack, port 8772, map_a4_v4 + bins_acrylic_v2_pair)
    ARM_MAP=map_a4_v4 uv run --with 'opencv-python<5' --with numpy --with onnxruntime \
        python dev/tools/stack_bench.py sim [--console http://127.0.0.1:8772] [--cases two,three,...] [--field-only]
    # offline on the real field scans (no height views there): no block may come out raised
    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/stack_bench.py real [runs ...]

sim: each case puts blocks on the simulated table (/api/sim/block on an occupied place stacks), scans with
stack_scan.scan (field views + planned height views), measures, and compares with /api/sim/world: per true
block the nearest found block of the same height -- level right or wrong, centre error (cm), yaw error (deg)
-- and the found blocks no true block explains (phantoms). Heights compare bottoms with the truth's base_cm
minus table_z_cm (the simulator's table, -1.42 cm; the planner's is -0.87 cm, doc 40 §2).
Writes data/sim/stack_bench/<case>/ (the run, stack_scene.json, located.json) and data/sim/stack_bench.json.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("", "tools", "tasks", "pipeline", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "app" / "src" / sub) if sub else str(ROOT / "app" / "src"))

# name -> blocks to place in order: (category, map_cm, yaw_deg, up). A place already taken stacks.
CASES = {
    "two": [("kitchen", [-5.0, 14.0], 10, 0), ("recyclable", [-5.0, 14.0], 25, 0),
            ("hazardous", [6.0, 17.0], 0, 1)],
    "three": [("other", [4.0, 15.0], 0, 0), ("hazardous", [4.0, 15.0], -5, 0), ("kitchen", [4.0, 15.0], 20, 0),
              ("recyclable", [-8.0, 17.0], 15, 2)],
    "turned": [("hazardous", [-2.0, 16.0], 0, 0), ("other", [-2.0, 16.0], 40, 1),
               ("kitchen", [8.0, 13.0], -20, 0), ("recyclable", [8.0, 13.0], 30, 3)],
    "adjacent": [("kitchen", [0.0, 15.0], 0, 0), ("hazardous", [0.0, 15.0], 10, 0),
                 ("other", [3.2, 15.0], 0, 2), ("recyclable", [0.0, 18.2], 5, 1)],
    "bin": [("kitchen", [9.43, 7.25], 15, 0), ("hazardous", [-9.57, 7.25], -10, 1),
            ("other", [2.0, 15.0], 0, 0), ("recyclable", [2.0, 15.0], 15, 0)],
    "far": [("other", [-3.0, 19.0], 0, 0), ("kitchen", [-3.0, 19.0], 30, 0),
            ("recyclable", [10.0, 18.0], 0, 0), ("hazardous", [10.0, 18.0], 0, 0), ("other", [10.0, 18.0], 15, 1)],
    "table": [("kitchen", [-8.0, 12.0], 20, 0), ("hazardous", [-1.0, 15.0], -30, 1), ("other", [6.0, 11.0], 5, 2),
              ("recyclable", [9.0, 17.0], 40, 3), ("kitchen", [1.0, 19.0], 0, 4)],
}


def fold(d: float) -> float:
    return (d + 45) % 90 - 45


def truth_blocks(world: dict) -> list[dict]:
    tz = world["table_z_cm"]
    out = []
    for b in world["blocks"]:
        bottom = b["base_cm"][2] - 1.5 - tz
        out.append(dict(id=b["id"], category=b["category"], map_cm=b["map_cm"], yaw=b["yaw_deg"], inside=b.get("inside"),
                        bottom_cm=round(bottom, 2), level=int(round(max(bottom, 0) / 3.0))))
    return out


def compare(truth: list[dict], found: list[dict]) -> dict:
    """Pair true and found blocks (nearest in 3D first, <= 2.5 cm across and one level apart at most)."""
    pairs = sorted(((math.dist(t["map_cm"], f["map_cm"]) + 3 * abs(t["level"] - f["level"]), i, j)
                    for i, t in enumerate(truth) for j, f in enumerate(found)
                    if math.dist(t["map_cm"], f["map_cm"]) <= 2.5 and abs(t["level"] - f["level"]) <= 1))
    ti, fj, rows = set(), set(), []
    for _, i, j in pairs:
        if i in ti or j in fj:
            continue
        ti.add(i)
        fj.add(j)
        t, f = truth[i], found[j]
        rows.append(dict(truth=t["id"], category=t["category"], level=t["level"], found_level=f["level"],
                         level_ok=t["level"] == f["level"], bottom_err_cm=round(f["bottom_cm"] - t["bottom_cm"], 2),
                         xy_err_cm=round(math.dist(t["map_cm"], f["map_cm"]), 2),
                         yaw_err_deg=None if f["inferred"] else round(abs(fold(f["yaw_deg"] - t["yaw"])), 1),
                         class_ok=None if f["category"] is None else f["category"] == t["category"], covered=f.get("covered", False), inferred=f["inferred"], in_bin=f["in_bin"],
                         truth_in=t["inside"], name=f["name"]))
    missed = [truth[i]["id"] for i in range(len(truth)) if i not in ti]
    phantoms = [found[j]["name"] for j in range(len(found)) if j not in fj]
    return dict(rows=rows, missed=missed, phantoms=phantoms,
                levels_ok=sum(r["level_ok"] for r in rows) == len(truth) and not missed and not phantoms)


def sim(a) -> int:
    import console_step as CS
    CS.BASE = a.console.rstrip("/")
    import stack_scan as SS
    tok = CS.call("/api/bootstrap")["token"]
    if a.jitter:
        CS.call("/api/sim/faults", {"pose_jitter_deg": [float(v) for v in a.jitter.split(",")]}, token=tok)
    report = {}
    names = a.cases.split(",") if a.cases else list(CASES)
    try:
        for name in names:
            CS.call("/api/sim/reset", {"count": 0, "seed": 1}, token=tok)
            for cat, xy, yaw, up in CASES[name]:
                r = CS.call("/api/sim/block", {"category": cat, "map_cm": xy, "yaw_deg": yaw, "up": up}, token=tok)
                if r.get("error"):
                    raise RuntimeError(f"{name}: {r['error']}")
            world = CS.call("/api/sim/world")
            truth = truth_blocks(world)
            run = ROOT / "data" / "sim" / "stack_bench" / name / (("field" if a.field_only else "scan")
                                                                  + ("_jitter" if a.jitter else "") + a.tag)
            if not a.reuse:
                shutil.rmtree(run, ignore_errors=True)
                t0 = time.time()
                with contextlib.redirect_stdout(io.StringIO()):
                    SS.scan(run, a.console, a.bins, field_poses=None, plan_only=a.field_only)
                took = time.time() - t0
            else:
                took = None
            t0 = time.time()
            res = SS.measure(run, a.bins)
            loc = SS.located(res, run, bins_design=a.bins)
            (run / "located.json").write_text(json.dumps(loc, indent=1, ensure_ascii=False, default=SS._plain))
            cmp_ = compare(truth, res["blocks"])
            views = json.loads((run / "views.json").read_text())
            world_after = CS.call("/api/sim/world")
            moved = [b["id"] for b, c in zip(world["blocks"], world_after["blocks"])
                     if math.dist(b["base_cm"], c["base_cm"]) > 0.2]
            report[name] = dict(cmp_, views=len(views), height_views=sum(v.get("purpose") == "height" for v in views),
                                scan_s=None if took is None else round(took), measure_s=round(time.time() - t0, 1),
                                contacts=world_after["block_contacts"]["count"] - world["block_contacts"]["count"],
                                moved=moved, truth=truth)
            print(f"== {name}: {'OK' if cmp_['levels_ok'] else 'WRONG'}  views {len(views)} "
                  f"(height {report[name]['height_views']})  missed {cmp_['missed']} phantoms {cmp_['phantoms']}"
                  f"  moved {moved}", flush=True)
            for r in cmp_["rows"]:
                print(f"   {r['truth']:4s} {r['category']:10s} level {r['level']} -> {r['found_level']}"
                      f"{'' if r['level_ok'] else ' X'}  xy {r['xy_err_cm']:.2f} cm  yaw {r['yaw_err_deg']}  "
                      f"class {'unknown' if r['class_ok'] is None else 'ok' if r['class_ok'] else 'X'}"
                      f"{' covered' if r['covered'] else ''}{' inferred' if r['inferred'] else ''}"
                      f"{' in ' + r['in_bin'] if r['in_bin'] else ''}", flush=True)
    finally:
        if a.jitter:
            CS.call("/api/sim/faults", {"pose_jitter_deg": [0, 0, 0, 0, 0]}, token=tok)
    out = ROOT / "data" / "sim" / (("stack_bench_field" if a.field_only else "stack_bench") + a.tag + ".json")
    out.write_text(json.dumps(report, indent=1, ensure_ascii=False))
    print("cases with every level right:", sum(r["levels_ok"] for r in report.values()), "/", len(report))
    rows = [r for c in report.values() for r in c["rows"]]
    for name, sel in (("top/uncovered", [r for r in rows if not r["covered"]]), ("covered", [r for r in rows if r["covered"]])):
        xy = sorted(r["xy_err_cm"] for r in sel)
        yaw = sorted(r["yaw_err_deg"] for r in sel if r["yaw_err_deg"] is not None)
        q = lambda v, p: v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")
        print(f"{name:14s} n {len(sel):2d}  level ok {sum(r['level_ok'] for r in sel)}  class ok "
              f"{sum(r['class_ok'] is True for r in sel)} wrong {sum(r['class_ok'] is False for r in sel)} unknown "
              f"{sum(r['class_ok'] is None for r in sel)}  xy cm median {q(xy, .5):.2f} p90 {q(xy, .9):.2f} max "
              f"{max(xy) if xy else float('nan'):.2f}  yaw deg median {q(yaw, .5):.1f} max {max(yaw) if yaw else float('nan'):.1f}")
    return 0


def real(a) -> int:
    import stack_scan as SS
    from blocks_detector import BlocksDetector
    det = BlocksDetector()
    runs = [Path(r) for r in a.runs] or sorted(p for p in (ROOT / "data" / "runs" / "pipeline").iterdir()
                                               if (p / "views.json").exists())
    rows, raised = [], []
    for run in runs:
        try:
            res = SS.measure(run, None, roi=not a.no_roi, detector=det)
        except Exception as e:  # noqa: BLE001
            print(f"{run.name}: {e}")
            rows.append(dict(run=run.name, error=str(e)))
            continue
        bl = res["blocks"]
        up = [b for b in bl if b["level"] > 0 or b["inferred"]]
        raised += [dict(run=run.name, **b) for b in up]
        margins = [round(b["alt"]["score"] - b["score"], 2) for b in bl if b.get("alt") and b["level"] == 0]
        rows.append(dict(run=run.name, blocks=len(bl), raised=len(up), alt_minus_table=margins))
        print(f"{run.name}: {len(bl)} blocks, raised {len(up)}  (best raised alternative minus table: {margins})",
              flush=True)
    out = ROOT / "data" / "reports" / "stack_bench_real.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(rows=rows, raised=raised), indent=1, ensure_ascii=False, default=SS._plain))
    print(f"{len(runs)} runs, {sum(r.get('blocks', 0) for r in rows)} blocks, raised {len(raised)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("where", choices=("sim", "real"))
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--console", default="http://127.0.0.1:8772")
    ap.add_argument("--bins", default="bins_acrylic_v2_pair")
    ap.add_argument("--cases")
    ap.add_argument("--field-only", action="store_true", help="no height views (the field scan alone)")
    ap.add_argument("--reuse", action="store_true", help="measure the runs already scanned")
    ap.add_argument("--jitter", help="pose_jitter_deg for the simulator, e.g. 0.7,0.8,0.8,0.8,0")
    ap.add_argument("--tag", default="", help="suffix of the scan directories and the report (repeat runs)")
    ap.add_argument("--no-roi", action="store_true")
    a = ap.parse_args()
    return sim(a) if a.where == "sim" else real(a)


if __name__ == "__main__":
    sys.exit(main())
