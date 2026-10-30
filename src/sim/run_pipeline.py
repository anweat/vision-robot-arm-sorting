#!/usr/bin/env python3
"""Run the real sorting pipeline (src/pipeline/run.py) against the simulation back end.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime src/sim/run_pipeline.py --rounds 2

Start src/sim/sim_server.py first (map_a4_v2, no bins: the pipeline still puts blocks on map codes
40/41/43/44; or map_a4_v3 with `--bins bins_acrylic_v1` here to sort into the acrylic bins). Nothing in the pipeline is changed: the same scan, scene building, planning, grasping and
logging code runs, only pointed at the simulation's address, and its files go under data/sim/ instead of
data/runs/, so simulated events never mix with the real arm's log.

Each round puts fresh blocks at random, scans, sorts, and writes the same events the real arm writes to
data/sim/log.jsonl -- the file the simulation serves at /api/log.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))
SIM = ROOT / "data" / "sim"
CATEGORIES = ("recyclable", "kitchen", "hazardous", "other")
# Where a round's blocks are put: the band the real rounds use (map rows 8-18 cm). Nearer than about
# y = 8 cm only the folded near views see a block, the known weak strip of the scan (docs/10, risk 1).
X_CM, Y_CM, APART_CM, CLEAR_OF_BIN_CM = (-11.0, 11.0), (9.0, 17.0), 5.0, 3.2
BINS_Y_CM, BINS_APART_CM, BINS_CLEAR_CM = (10.0, 17.8), 4.5, 2.0


def lay_out(count: int, rng: random.Random, bins_cm: list, apart_cm: float = APART_CM, y_cm=Y_CM,
            clear_cm: float = CLEAR_OF_BIN_CM, categories=CATEGORIES) -> list[dict]:
    """Random block places. `bins_cm` is either marker centres (keep `clear_cm` from each) or bin plates
    (see plates(): keep `clear_cm` off each plate)."""
    spots: list[tuple[float, float]] = []
    for _ in range(count):
        for _ in range(2000):
            p = (rng.uniform(*X_CM), rng.uniform(*y_cm))
            if all(math.dist(p, q) >= apart_cm for q in spots) and all(
                    (not on_plate(p, b, clear_cm)) if len(b) == 4 else math.dist(p, b) >= clear_cm for b in bins_cm):
                spots.append(p)
                break
        else:
            raise SystemExit("no room for that many blocks")
    return [dict(category=categories[i % len(categories)], map_cm=[round(x, 1), round(y, 1)], yaw_deg=round(rng.uniform(-45, 45)),
                 up=rng.randrange(5)) for i, (x, y) in enumerate(spots)]


def plates(world: dict) -> list[tuple[float, float, float, float, float]]:
    """Each bin's plate as (centre x, centre y, yaw, plate part) in map cm, for keeping blocks off it."""
    out = []
    for p in world["props"]:
        if p.get("number") is None:
            continue
        out.append((*p["map_cm"], p["yaw_deg"], p["parts"][0]))
    return out


def on_plate(point, plate, margin_cm: float) -> bool:
    cx, cy, yaw, part = plate
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    dx, dy = point[0] - cx, point[1] - cy
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    (px, py, _), (w, d, _) = part["centre_cm"], part["size_cm"]
    return abs(lx - px) <= w / 2 + margin_cm and abs(ly - py) <= d / 2 + margin_cm


def judge(truth: dict, estimated, contacts_before: int) -> dict:
    """Where every block ended up, against the simulation's own truth; and how far off each bin was found."""
    by_id = {p["id"]: p for p in truth["props"]}
    right = wrong = perched = outside = 0
    for b in truth["blocks"]:
        bin_ = by_id.get(b["inside"]) if b["inside"] else None
        if b.get("perched"):
            perched += 1
        elif bin_ is None:
            outside += 1
        elif bin_["category"] == b["category"]:
            right += 1
        else:
            wrong += 1
    found = []
    for p in truth["props"]:
        if p.get("number") is None or estimated is None:
            continue
        b = estimated.bins[p["number"]]
        true_xy = [p["base_T_prop"][0][3], p["base_T_prop"][1][3]]
        found.append(dict(bin=p["number"], source=b.source,
                          error_mm=round(math.dist(true_xy, b.xy) * 1000, 1),
                          yaw_error_deg=round((b.yaw_deg - p["yaw_deg"] + 180) % 360 - 180, 2)))
    contacts = truth["contacts"]["count"] - contacts_before
    line = (f"{len(truth['blocks'])} blocks: {right} in the right bin, {wrong} in the wrong one, {perched} perched on a rim, "
            f"{outside} not in a bin; bins found off by {[f['error_mm'] for f in found]} mm; arm/block touched a bin {contacts} times")
    return dict(blocks=len(truth["blocks"]), in_right_bin=right, in_wrong_bin=wrong, perched=perched, not_in_a_bin=outside,
                contacts=contacts, bins=found, contact_events=truth["contacts"]["recent"][-contacts:] if contacts else [], line=line)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="http://127.0.0.1:8770", help="the simulation back end")
    parser.add_argument("--rounds", type=int, default=1)
    parser.add_argument("--blocks", type=int, default=4)
    parser.add_argument("--seed", type=int, help="for repeatable block layouts")
    parser.add_argument("--grip-miss", type=float, help="set the simulation's share of grips that close on nothing")
    parser.add_argument("--keep-scene", action="store_true", help="sort what is on the table instead of resetting it")
    parser.add_argument("--dry-run", action="store_true", help="scan and plan only")
    parser.add_argument("--bins", nargs="?", const="bins_acrylic_v1", metavar="DESIGN",
                        help="stand the designed acrylic bins on the table and sort into them (map_a4_v3)")
    parser.add_argument("--bin-side", default="all", metavar="left|right|all|1,2",
                        help="stand only these bins (a half-side start for a small table); blocks are of their categories")
    parser.add_argument("--accumulate", action="store_true",
                        help="keep the blocks already in bins between rounds (fills the bins up; default: table cleared)")
    parser.add_argument("--slot-cm", type=float, metavar="V",
                        help="try drop slots at (+-V, +-V) cm in the bin frame instead of config/sorting.json's")
    parser.add_argument("--bin-jitter-mm", type=float, default=0.0, help="random placement error of each bin, +- mm")
    parser.add_argument("--bin-jitter-deg", type=float, default=0.0, help="random placement error of each bin, +- degrees")
    parser.add_argument("--servo-bias", type=lambda v: [float(x) for x in v.split(",")], metavar="D1,D2,D3,D4,D5",
                        help="degrees the arm really stands off its readback on joints 1-5 (backlash stand-in)")
    args = parser.parse_args()

    import console_step
    console_step.BASE = args.base.rstrip("/")           # before the modules below copy it
    import console_grasp as G
    import bins as B
    import field_scan as SC
    import run as R

    boot = console_step.call("/api/bootstrap")
    if boot.get("status", {}).get("mode") != "simulation" or "sim" not in boot["status"]:
        raise SystemExit(f"{args.base} is not the simulation back end; refusing to run")
    G.RUNS = SIM
    R_state = SIM / "bins_state.json"
    R.STATE_FILE = R_state                     # the simulation's own counts, never the real arm's
    R_state.unlink(missing_ok=True)           # a new simulation run starts with empty bins
    G.PLAN = SIM / "plan.json"
    SC.RUNS = SIM / "pipeline"
    SIM.mkdir(parents=True, exist_ok=True)

    faults = {k: v for k, v in (("grip_miss", args.grip_miss), ("servo_bias_deg", args.servo_bias)) if v is not None}
    if faults:
        fault = console_step.call("/api/sim/faults", faults, token=boot["token"])
        if fault.get("error"):
            raise SystemExit(fault["error"])
    if args.bins:
        only = B.parse_only(args.bin_side)
        request = dict(name=args.bins, jitter_mm=args.bin_jitter_mm, jitter_deg=args.bin_jitter_deg, seed=args.seed)
        if only is not None:
            request["only"] = list(only)
        world = console_step.call("/api/sim/bins", request, token=boot["token"])
        if world.get("error"):
            raise SystemExit(world["error"])
    import sort_blocks as SB
    if args.slot_cm is not None:
        v = args.slot_cm
        SB.SORTING["bins"]["slots_cm"] = [[-v, -v], [v, v], [-v, v], [v, -v]]
    bins_cm = [] if args.bins else [[v * 100 for v in G.marker_centre(m)] for m in SB.SORTING["category_bins"].values()]
    rng = random.Random(args.seed)
    reports = []
    for i in range(args.rounds):
        if not args.keep_scene:
            truth = console_step.call("/api/sim/world")
            # the bins take the sides of the table: what is left is a V-shaped corridor, and the arm cannot
            # reach nearer than ~10.7 cm up it (MIN_REACH), so blocks go in the upper part of it
            layout = lay_out(args.blocks, rng, plates(truth), BINS_APART_CM, BINS_Y_CM, BINS_CLEAR_CM,
                             [p["category"] for p in truth["props"] if p.get("number")]) if args.bins \
                else lay_out(args.blocks, rng, bins_cm)
            if args.accumulate:
                clear = [("/api/sim/remove", dict(id=b["id"])) for b in truth["blocks"] if not b["inside"]]
            else:
                clear = [("/api/sim/reset", dict(count=0))]
            for request in clear + [("/api/sim/block", b) for b in layout]:
                world = console_step.call(*request, token=boot["token"])
                if world.get("error"):
                    raise SystemExit(world["error"])
        contacts_before = console_step.call("/api/sim/world")["contacts"]["count"]
        G.require_empty_gripper()
        with G.Session() as sess:
            if args.bins:
                R.use_bins(B.BinSet(args.bins, only))      # the keep-out round the bins, before the arm moves at all
            run = SC.scan(session=sess)
            if args.bins:
                if SC.has_bin_views(args.bins):
                    SC.scan_bins(run, sess, args.bins, only)
                R.use_bins(R.locate_bins(run, args.bins, only))
            scene = R.build_scene(run)
            R.use_registration(scene["registration"])
            if not args.dry_run:
                scene = R.refine(run, scene, sess)
                R.use_registration(scene["registration"])
            jobs = R.decide(scene)
            counts = None if args.dry_run else R.execute(run, scene, jobs, sess)
        truth = console_step.call("/api/sim/world")
        (run / "world_after.json").write_text(json.dumps(truth, indent=1, ensure_ascii=False))
        print(f"round {i + 1}/{args.rounds}: {counts}  ->  {run.relative_to(ROOT)}")
        if args.bins:
            report = judge(truth, R.BINS, contacts_before)
            reports.append(report)
            (run / "bins_report.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
            print("   ", report["line"])
        time.sleep(1)
    if reports:
        total = {k: sum(r[k] for r in reports) for k in ("blocks", "in_right_bin", "in_wrong_bin", "perched", "not_in_a_bin", "contacts")}
        out = SIM / "bins_summary.json"
        out.write_text(json.dumps(dict(total=total, rounds=reports), indent=1, ensure_ascii=False))
        print(f"all rounds: {total}  ->  {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
