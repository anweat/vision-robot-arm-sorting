"""The new step flow on the real arm's recorded scans, offline: locate -> map -> plan for every block found, and
the plans set against the grips the real arm made in the same run (doc 34 §4.2). Nothing moves.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/flow_on_scans.py \
        [data/runs/pipeline/<run> ...] [--out data/flow/real_replay]

Default: every data/runs/pipeline/<run> with views.json. Each block is planned from the observe pose to the
first free table spot of dev/sim/flow_check.py (a spot within 4 cm of a located block is skipped). The real
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
from flow_check import SPOTS  # noqa: E402

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


def one(run: Path, out_root: Path, events: list[dict]) -> dict:
    root = out_root / run.name
    shutil.rmtree(root, ignore_errors=True)
    s = FlowSession(root)
    loc = ST.locate(s, scan_dir=run)
    ST.build_map(s)
    objs = loc["objects"]
    rows = []
    for o in objs:
        row = dict(name=o["name"], cls=o.get("class"), xy_cm=[round(v * 100, 1) for v in o["centre_m"]],
                   r_cm=round(math.hypot(*o["centre_m"]) * 100, 1), ok=False)
        for spot in SPOTS:
            if any(math.dist(spot, p["centre_m"]) < 0.04 for p in objs):
                continue
            p = ST.plan(s, dict(pick=o["name"], to={"table_m": spot}, start=OBSERVE, start_j6=30))
            if p["ok"]:
                g = p["plans"][0]["summary"]["grasp"]
                row.update(ok=True, spot=spot, pitch=g["pitch"], j5=g["j5"], grasp=g["grasp"],
                           platform_mesh_cm=min(c["platform_mesh_cm"] for c in g["checks"].values()
                                                if c.get("platform_mesh_cm") is not None),
                           hull_cm=p["plans"][0]["summary"].get("console_hull_min_cm"))
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
                          planned_grasp=row.get("grasp"),
                          real_pad_to_block_mm=round(math.dist(pad_real[:2], near["centre_m"]) * 1000, 1)))
    return dict(run=run.name, located=len(objs), plannable=sum(r["ok"] for r in rows), blocks=rows, real_grips=grips)


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
        refused = [f"{b['name']}@{b['r_cm']}cm: {'; '.join(b.get('why', []))[:90]}" for b in r["blocks"] if not b["ok"]]
        print(f"{r['run']}: located {r['located']}, planned {r['plannable']}"
              + (f"; refused {refused}" if refused else ""))
        for g in r["real_grips"]:
            print(f"    real grip {g['block']}: {'held' if g['real_held'] else 'empty'}, planned {g['planned']}, "
                  f"real pad {g['real_pad_to_block_mm']} mm from the located centre")
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))
    ok = [r for r in report if "error" not in r]
    grips = [g for r in ok for g in r["real_grips"]]
    print(f"\n{len(ok)}/{len(report)} scans read; blocks planned {sum(r['plannable'] for r in ok)}/{sum(r['located'] for r in ok)}; "
          f"real grips matched {len(grips)}, of them planned {sum(g['planned'] for g in grips)}, "
          f"real held {sum(g['real_held'] for g in grips)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
