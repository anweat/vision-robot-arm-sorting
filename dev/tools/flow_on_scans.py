"""The new step flow on the real arm's recorded scans, offline: locate -> map -> plan for every block found, and
the plans set against the grips the real arm made in the same run (doc 34 §4.2). Nothing moves.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/flow_on_scans.py \
        [data/runs/pipeline/<run> ...] [--out data/flow/real_replay]

Default: every data/runs/pipeline/<run> with views.json. Each block is first planned as a pick alone (from the
observe pose), then pick + place to where the verified pipeline would have put it -- its category's map code
(app/config/sorting.json) -- else the first free table spot of dev/sim/flow_check.py (a spot within 4 cm of a
located block is skipped). The real
grips come from data/runs/console_grasp/log.jsonl: the gripped events between that run's pipeline_start and the
next, each tied to the block of the pipeline_sorted/missed that follows (map cm through the run's
registration), then to the nearest located block (<= 2 cm).
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "app" / "src"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))
sys.path.insert(0, str(ROOT / "dev" / "sim"))

from flow import stages as ST  # noqa: E402
from flow.session import FlowSession  # noqa: E402
from planning import gripper as GR  # noqa: E402
from planning import grasp as GP, planner as PL  # noqa: E402
from planning.collision_map import CollisionMap  # noqa: E402
import planning  # noqa: E402
from flow_check import SPOTS  # noqa: E402
sys.path.insert(0, str(ROOT / "app" / "src" / "tasks"))
import console_grasp as G  # noqa: E402

SORTING = json.loads((ROOT / "app" / "config" / "sorting.json").read_text())

LOG = ROOT / "data" / "runs" / "console_grasp" / "log.jsonl"
OBSERVE = [90, 105, 5, 5, 95]


def real_grips(events: list[dict], run: Path) -> list[dict]:
    out, inside, origin = [], False, None
    for i, e in enumerate(events):
        if e["event"] == "pipeline_start":
            inside = Path(e.get("run", "")).name == run.name
            origin = e.get("registration", {}).get("map_origin_base_cm")
        elif inside and e["event"] == "gripped" and origin:
            for f in events[i + 1:i + 8]:
                if f["event"] in ("pipeline_sorted", "pipeline_missed") and "cube" in f:
                    rb = (e.get("grip") or {}).get("readback")
                    out.append(dict(pose=e["pose"][:5], readback=rb,
                                    held=rb is not None and 100 <= rb <= 131,
                                    xy=[(f["cube"][0] + origin[0]) / 100, (f["cube"][1] + origin[1]) / 100]))
                    break
    return out


def destinations(o: dict, loc: dict, objs: list[dict]) -> list[tuple[str, list]]:
    """Where the verified pipeline would have put this block -- its category's map code (app/config/sorting.json,
    through the scan's registration) -- then the free table spots of flow_check, as fallbacks."""
    out = []
    origin = (loc.get("registration") or {}).get("map_origin_base_cm")
    if o.get("class") in SORTING["category_bins"] and origin:
        mx, my = G.marker_centre(SORTING["category_bins"][o["class"]])
        out.append((f"marker {SORTING['category_bins'][o['class']]}", [mx + origin[0] / 100, my + origin[1] / 100]))
    out += [("spot", s) for s in SPOTS if not any(math.dist(s, p["centre_m"]) < 0.04 for p in objs)]
    return out


def one(run: Path, out_root: Path, events: list[dict]) -> dict:
    root = out_root / run.name
    shutil.rmtree(root, ignore_errors=True)
    s = FlowSession(root)
    loc = ST.locate(s, scan_dir=run)
    ST.build_map(s)
    objs = loc["objects"]
    cm = CollisionMap.load(s.file("collision_map.json"))
    w = planning.world(cm)
    rows = []
    for o in objs:
        row = dict(name=o["name"], cls=o.get("class"), xy_cm=[round(v * 100, 1) for v in o["centre_m"]],
                   r_cm=round(math.hypot(*o["centre_m"]) * 100, 1), ok=False, pick=False)
        why: list = []
        pick = next(PL.pick_plans(w, OBSERVE, GP.Target.from_map(cm, o["name"]), 30, None, why), None)
        if pick is None:
            row["why"] = [r for r in why if "out of reach" not in r][:3]
            rows.append(row)
            continue
        g = pick.summary["grasp"]
        row.update(pick=True, pick_tier=pick.summary.get("grasp_tier"), pitch=g["pitch"], j5=g["j5"], grasp=g["grasp"], aligned=g["aligned"],
                   squares=bool(g.get("squares")), method=pick.summary.get("method", "pads"),
                   hull_cm=pick.summary.get("console_hull_min_cm"))
        for kind, spot in destinations(o, loc, objs):
            p = ST.plan(s, dict(pick=o["name"], to={"table_m": spot}, start=OBSERVE, start_j6=30))
            if p["ok"]:
                tiers = [pl["summary"].get("grasp_tier") or pl["summary"].get("place_tier") for pl in p.get("plans", [])[:2]]
                row.update(ok=True, to=kind, spot=[round(v, 3) for v in spot], tiers=tiers)
                break
            row.setdefault("why", [r for r in p["reasons"] if "out of reach" not in r][:2])
        rows.append(row)
    grips = []
    for g in real_grips(events, run):
        near = min(objs, key=lambda o: math.dist(o["centre_m"], g["xy"]), default=None)
        if near is None or math.dist(near["centre_m"], g["xy"]) > 0.02:
            continue
        row = next(r for r in rows if r["name"] == near["name"])
        pad_real = GR.pad_centre(g["pose"], GR.contact())
        grips.append(dict(block=near["name"], real_held=g["held"], real_pose=g["pose"], planned=row["ok"],
                          picked=row["pick"], planned_grasp=row.get("grasp"),
                          real_pad_to_block_mm=round(math.dist(pad_real[:2], near["centre_m"]) * 1000, 1)))
    return dict(run=run.name, located=len(objs), pickable=sum(r["pick"] for r in rows),
                plannable=sum(r["ok"] for r in rows), blocks=rows, real_grips=grips)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "flow" / "real_replay")
    a = ap.parse_args()
    runs = a.runs or sorted(p for p in (ROOT / "data" / "runs" / "pipeline").iterdir() if (p / "views.json").exists())
    events = [json.loads(l) for l in LOG.read_text().splitlines() if l.strip()] if LOG.exists() else []
    report = []
    for run in runs:
        try:
            r = one(run, a.out, events)
        except Exception as exc:      # a scan the flow cannot read is a result too
            r = dict(run=run.name, error=f"{type(exc).__name__}: {exc}")
        report.append(r)
        if "error" in r:
            print(f"{r['run']}: ERROR {r['error'][:160]}")
            continue
        refused = [f"{b['name']}@{b['r_cm']}cm{'' if b['pick'] else ' (pick)'}: {'; '.join(b.get('why', []))[:110]}"
                   for b in r["blocks"] if not b["ok"]]
        print(f"{r['run']}: located {r['located']}, pick {r['pickable']}, pick+place {r['plannable']}"
              + (f"; refused {refused}" if refused else ""))
        for g in r["real_grips"]:
            print(f"    real grip {g['block']}: {'held' if g['real_held'] else 'empty'}, pick {g['picked']}, planned {g['planned']}, "
                  f"real pad {g['real_pad_to_block_mm']} mm from the located centre")
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))
    ok = [r for r in report if "error" not in r]
    grips = [g for r in ok for g in r["real_grips"]]
    from collections import Counter
    print("pick tiers:", dict(Counter(b.get("pick_tier") for r in ok for b in r["blocks"] if b["pick"])),
          "| pick+place tiers:", dict(Counter(tuple(b.get("tiers", ())) for r in ok for b in r["blocks"] if b["ok"])))
    print(f"\n{len(ok)}/{len(report)} scans read; blocks: pick {sum(r['pickable'] for r in ok)}, "
          f"pick+place {sum(r['plannable'] for r in ok)} of {sum(r['located'] for r in ok)}; "
          f"real grips matched {len(grips)}, of them pick {sum(g['picked'] for g in grips)}, "
          f"pick+place {sum(g['planned'] for g in grips)}, real held {sum(g['real_held'] for g in grips)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
