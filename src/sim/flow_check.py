"""The whole flow against the simulator console, judged by the simulator's ground truth.

    scan -> locate -> map -> plan (first block that has a plan to one of the table spots) -> execute
         -> scan + locate again -> verify, and the truth: localization error per block, where the block
            really ended up, contacts.

Needs the simulator console running (`src/sim/sim_server.py --port 8770`, launch config `arm-sim`). It
resets the simulated table. Results only say the geometry and the interfaces agree with each other; the
simulator renders with the same camera model it locates with (2026-11-15 camera check: the real camera
model is off by ~11 px / 0.3 cm per view), so localization here is optimistic.

    uv run --with 'opencv-contrib-python<5' --with numpy --with onnxruntime python src/sim/flow_check.py \
        [--layout reachable|random] [--seed 7] [--session data/flow/sim_check]
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from flow import stages as ST          # noqa: E402
from flow.session import FlowSession   # noqa: E402

REACHABLE = (("kitchen", [0.0, 13.0], 5.0), ("hazardous", [-7.0, 14.0], -10.0),
             ("recyclable", [7.5, 11.5], 15.0), ("other", [3.5, 17.5], 0.0))
# one block at the map's close edge (11.75 cm from J1) and three with the arm stretched (27-29 cm), 2026-11-16
EDGE = (("kitchen", [0.0, 5.0], 0.0), ("hazardous", [13.0, 18.0], 10.0),
        ("recyclable", [0.0, 22.0], 0.0), ("other", [-13.0, 18.0], -10.0))
# a level set-down keeps the grasp's pitch (5 joints), so a block picked steep next to the platform can only go
# down near it, and one picked flat far out only far out: spots for both
SPOTS = ([-0.04, 0.17], [0.05, 0.215], [-0.10, 0.17], [0.045, 0.13], [-0.045, 0.13], [0.065, 0.27], [-0.065, 0.27])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--console", default=ST.SIM_CONSOLE)
    ap.add_argument("--layout", choices=("reachable", "random", "edge"), default="reachable")
    ap.add_argument("--truth-locate", action="store_true",
                    help="plan from the simulator's true block poses instead of what the scan located (tests planning "
                         "and execution where perception does not reach, e.g. the strip next to the platform)")
    ap.add_argument("--pick", choices=("first", "nearest", "farthest"), default="first",
                    help="which plannable block to execute (every block is planned and reported)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--session", default=str(ROOT / "data" / "flow" / "sim_check"))
    a = ap.parse_args(argv)
    ST._use_console(a.console)
    import console_step as cs
    boot = cs.call("/api/bootstrap")
    if boot.get("status", {}).get("mode") != "simulation":
        raise SystemExit(f"{a.console} is not the simulator; refusing")
    tok = boot["token"]
    if a.layout == "random":
        cs.call("/api/sim/reset", {"count": 4, "seed": a.seed}, token=tok)
    else:
        cs.call("/api/sim/reset", {"count": 0}, token=tok)
        for cat, xy, yaw in (EDGE if a.layout == "edge" else REACHABLE):
            r = cs.call("/api/sim/block", {"category": cat, "map_cm": xy, "yaw_deg": yaw}, token=tok)
            if r.get("error"):
                raise SystemExit(r["error"])
    truth = cs.call("/api/sim/world")
    root = Path(a.session)
    shutil.rmtree(root, ignore_errors=True)
    s1 = FlowSession(root / "before")
    report: dict = dict(layout=a.layout, truth_before=[dict(id=b["id"], category=b["category"], base_cm=b["base_cm"][:2],
                                                            yaw_deg=b["yaw_deg"]) for b in truth["blocks"]])
    t0 = time.time()
    ST.scan(s1, a.console, really=True)
    loc = ST.locate(s1)
    errs = []
    for o in loc["objects"]:
        xy = [v * 100 for v in o["centre_m"]]
        tb = min(truth["blocks"], key=lambda b: math.dist(b["base_cm"][:2], xy))
        errs.append(dict(name=o["name"], truth=tb["id"], class_=o["class"], truth_class=tb["category"],
                         error_cm=round(math.dist(tb["base_cm"][:2], xy), 2),
                         yaw_error_deg=round((o["yaw_deg"] - tb["yaw_deg"] + 45) % 90 - 45, 1)))
    report.update(located=errs, missed=len(truth["blocks"]) - len(loc["objects"]))
    if a.truth_locate:
        loc = dict(loc, objects=[dict(id=i + 1, name=b["id"], kind="block", **{"class": b["category"]},
                                      centre_m=[v / 100 for v in b["base_cm"][:2]], yaw_deg=b["yaw_deg"],
                                      size_m=[0.03] * 3, sigma_xy_m=0.005) for i, b in enumerate(truth["blocks"])],
                   source="simulator truth (--truth-locate)")
        s1.save("located.json", loc)
        report["truth_locate"] = True
    ST.build_map(s1)
    rb = cs.status()["pose"]
    chosen, refused, plannable = None, [], []
    for o in loc["objects"]:
        ok = None
        for spot in SPOTS:
            if math.dist(spot, o["centre_m"]) < 0.04:          # a spot next to where it stands proves nothing
                continue
            p = ST.plan(s1, dict(pick=o["name"], to={"table_m": spot}, start=rb[:5], start_j6=rb[5]))
            if p["ok"]:
                ok = (o["name"], spot, p)
                break
            refused.append(dict(block=o["name"], spot=spot, why=[r for r in p["reasons"] if "out of reach" not in r
                                                                 and "reach it inside" not in r][:2]))
        if ok:
            pk = ok[2]["plans"][0]["summary"]
            plannable.append(dict(block=o["name"], r_cm=round(math.hypot(*o["centre_m"]) * 100, 1), spot=ok[1],
                                  pitch=pk["pitch"], j5=pk["j5"], hull_cm=pk.get("console_hull_min_cm"), plan=ok))
    if plannable:
        key = {"first": lambda c: 0, "nearest": lambda c: c["r_cm"], "farthest": lambda c: -c["r_cm"]}[a.pick]
        best = min(plannable, key=key)
        name, spot, _ = best["plan"]          # plan.json holds the last plan made: make the chosen one again
        chosen = (name, spot, ST.plan(s1, dict(pick=name, to={"table_m": spot}, start=rb[:5], start_j6=rb[5])))
    report["plannable"] = [{k: v for k, v in c.items() if k != "plan"} for c in plannable]
    report["refused"] = refused
    if chosen is None:
        report["result"] = "no block had a plan"
        print(json.dumps(report, ensure_ascii=False, indent=1))
        return 1
    name, spot, p = chosen
    report["planned"] = dict(block=name, spot=spot, steps=[f"{s['plan']}:{s['name']}" for s in p["steps"]])
    report["execute"] = ST.execute(s1, really=True, console=a.console)
    truth2 = cs.call("/api/sim/world")
    want = p["plans"][1]["summary"]["target_base_cm"]
    tb = min(truth2["blocks"], key=lambda b: math.dist(b["base_cm"][:2], want))
    report["truth_after"] = dict(block=tb["id"], base_cm=tb["base_cm"], off_target_cm=round(math.dist(tb["base_cm"][:2], want), 2),
                                 yaw_deg=tb["yaw_deg"], contacts=truth2.get("contacts"))
    s2 = FlowSession(root / "after")
    ST.scan(s2, a.console, really=True)
    ST.locate(s2)
    report["verify"] = ST.verify(s1, s2.file("located.json"))
    report["seconds"] = round(time.time() - t0)
    (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=1))
    return 0 if report["verify"]["ok"] and report["execute"]["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
