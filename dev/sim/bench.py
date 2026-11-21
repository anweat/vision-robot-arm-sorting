"""Run a simulator scenario: lay out the table, plan each case from where the arm stands, execute it through the
console, judge it by the simulator's truth -- and the same again with parameters changed, to compare them.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/sim/bench.py list
    uv run ... python dev/sim/bench.py run vertical                                  # one run
    uv run ... python dev/sim/bench.py run vertical --set fault.stop_short_deg=1.5 --set execute.motion=continuous
    uv run ... python dev/sim/bench.py run vertical --sweep execute.motion=steps,continuous --repeat 3
    uv run ... python dev/sim/bench.py run vertical --sweep "fault.pose_jitter_deg=[0,0,0,0,0];[0.7,0.8,0.8,0.8,0]"

Needs the simulator console (launch config `arm-sim`, port 8770; `--console` for another); it resets its table.
Scenarios are dev/sim/scenarios/<name>.yaml (format: docs/current/16 §11). The overrides (`--set`, `--sweep`, a
scenario's own `set:`) are dev/sim/harness.py's: fault.* tuning.* plan.* console.speed execute.*. A sweep takes
`key=v1,v2,...` (or `;` between values that hold commas); several sweeps run every combination; --repeat runs
each combination with seeds seed, seed+1, ...

Each run writes <out>/<run>/<case>/ (the flow session: located.json, collision_map.json, plan.json,
execution.jsonl) and <out>/<run>/report.json; <out>/summary.json has every run and a table per case. Default out:
data/sim/bench/<scenario>/<time>. Results only say the geometry and the interfaces agree with each other
(docs/current/16): not a real-arm result.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import shutil
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import harness as H                    # noqa: E402
from flow import stages as ST          # noqa: E402
from flow.session import FlowSession   # noqa: E402

SCENARIOS = HERE / "scenarios"
SCENARIO_KEYS = ("name", "title", "note", "map", "bins", "seed", "faults", "set", "locate", "observe", "table_trim",
                 "blocks", "cases")
CASE_KEYS = ("name", "pick", "to", "expect", "note")


def load(name_or_path) -> dict:
    from miniyaml import load as yload
    p = Path(name_or_path)
    if not p.exists():
        p = SCENARIOS / f"{name_or_path}.yaml"
    if not p.exists():
        have = ", ".join(sorted(q.stem for q in SCENARIOS.glob("*.yaml")))
        raise SystemExit(f"no scenario {name_or_path!r} (have: {have})")
    sc = yload(p.read_text(encoding="utf-8"))
    bad = [k for k in sc if k not in SCENARIO_KEYS]
    bad += [f"cases[{i}].{k}" for i, c in enumerate(sc.get("cases") or []) for k in c if k not in CASE_KEYS]
    if bad:
        raise SystemExit(f"{p}: unknown keys {bad}")
    if not sc.get("cases"):
        raise SystemExit(f"{p}: no cases")
    tags = {b["tag"] for b in sc.get("blocks") or []}
    for c in sc["cases"]:
        if c["pick"] not in tags:
            raise SystemExit(f"{p}: case {c.get('name')} picks {c['pick']!r}, not one of the blocks {sorted(tags)}")
    sc.setdefault("name", p.stem)
    sc["source"] = str(p)
    return sc


# ---------------------------------------------------------------------------------------------------- one run

def resolve_to(to: dict, sim: H.Sim, ids: dict, block: dict, world: dict) -> dict:
    """A case's `to` with tags turned into the simulator's ids and `bin: auto` into the bin of the block's class."""
    to = dict(to)
    if "stack_on" in to:
        to["stack_on"] = ids[to["stack_on"]]
    if to.get("bin") == "auto":
        to["bin"] = sim.bin_of(block["category"], world)
        if to["bin"] is None:
            raise RuntimeError(f"no bin stands for {block['category']}")
    return to


def judge(expect: dict, landed: dict, b: dict, ids: dict, executed: bool) -> bool:
    if not executed or landed.get("toppled"):
        return False
    if "on" in expect:
        want = "table" if expect["on"] == "table" else ids.get(expect["on"], expect["on"])
        if landed.get("on") != want:
            return False
    if expect.get("inside") and not b.get("inside"):
        return False
    return True


def locate_case(sim: H.Sim, sess: FlowSession, sc: dict, world: dict, block_id: str) -> str:
    """located.json for the case (truth, or a scan through the observation script); the located name of the block."""
    if sc.get("locate", "truth") == "truth":
        sess.save("located.json", sim.located(sc.get("bins"), world))
        return block_id
    ST.scan(sess, console=sim.url, bins_design=sc.get("bins"), really=True, observe=sc.get("observe", "field"))
    loc = ST.locate(sess, bins_design=sc.get("bins"))
    truth = sim.block_truth(block_id, world)
    blocks = [o for o in loc["objects"] if o.get("kind") == "block"]
    if not blocks:
        raise RuntimeError("the scan found no block")
    near = min(blocks, key=lambda o: math.dist([v * 100 for v in o["centre_m"]], truth["base_cm"][:2]))
    return near["name"]


def run_once(sim: H.Sim, sc: dict, ov: H.Overrides, seed: int, out: Path) -> dict:
    sim.map(sc.get("map"))
    ids = sim.lay_out(sc.get("blocks") or [], seed)
    sim.bins(sc.get("bins"))
    faults = sim.faults(**dict(sc.get("faults") or {}, **ov.fault))
    H.set_speed(sim, ov.console.get("speed"))
    tuning = H._merge(sim.table_trim() if sc.get("table_trim", sc.get("locate", "truth") == "truth") else {}, ov.tuning)
    from planning import tuning as TU
    tu = TU.load(overrides=tuning or None)
    undo = ov.apply_plan()
    report = dict(scenario=sc["name"], seed=seed, blocks=ids, faults=faults, overrides=ov.describe(),
                  sim_speed=sim.speed, cases=[])
    t0 = time.time()
    try:
        for case in sc["cases"]:
            name = case.get("name") or f"case{len(report['cases'])}"
            w = sim.world()
            block_id = ids[case["pick"]]
            b0 = sim.block_truth(block_id, w)
            sess = FlowSession(out / name)
            row = dict(case=name, block=block_id)
            try:
                pick = locate_case(sim, sess, sc, w, block_id)
                to = resolve_to(case["to"], sim, ids, b0, w)
                ST.build_map(sess, tuning=tu)
                rb = sim.pose()
                task = dict(pick=pick, to=to, start=rb[:5], start_j6=rb[5], tuning=tuning)
                p = ST.plan(sess, task)
            except Exception as e:          # noqa: BLE001 -- a case that cannot be set up is a failed case, not a stop
                row.update(ok=False, planned=False, reasons=[f"{type(e).__name__}: {e}"])
                report["cases"].append(row)
                print(json.dumps(row, ensure_ascii=False))
                continue
            row.update(to=to, planned=p["ok"])
            if not p["ok"]:
                row.update(ok=False, reasons=p["reasons"][:4])
                report["cases"].append(row)
                print(json.dumps(row, ensure_ascii=False))
                continue
            pk, pl = p["plans"][0]["summary"], p["plans"][1]["summary"]
            row.update(grasp=dict(pitch=pk["pitch"], tier=pk.get("grasp_tier"), bottom_cm=pk["target"].get("bottom_cm", 0)),
                       release=dict(pitch=pl["pitch"], j5=pl["j5"], mode=pl.get("mode"), into=pl.get("into"),
                                    open_j6=pl["open_j6"], gap_mm=pl.get("open_gap_mm"), level_deg=pl["block_level_deg"],
                                    fall_cm=pl["release"].get("fall_cm")))
            began = time.time()
            ex = ST.execute(sess, really=True, console=sim.url, **ov.execute_kwargs())
            row["execute_s"] = round(time.time() - began, 2)
            corr = [r.get("correction") for r in ex["rows"] if r.get("correction")]
            row.update(executed=ex["ok"], stopped=ex["stopped"], motion=ex.get("motion"),
                       stops=sum(1 for r in ex["rows"] if r.get("stop", True)), steps=len(ex["rows"]),
                       correction=corr[0] if corr else None)
            w2 = sim.world()
            b = sim.block_truth(block_id, w2)
            landed = b.get("landed") or {}
            want = pl["target_base_cm"]
            c0, c1 = sim.counts(w), sim.counts(w2)
            row["truth"] = dict(landed=landed, inside=b.get("inside"), perched=b.get("perched"),
                                off_target_mm=round(math.dist(b["base_cm"][:2], want) * 10, 1),
                                **{k: c1[k] - c0[k] for k in c0})
            row["ok"] = judge(case.get("expect") or {}, landed, b, ids, ex["ok"])
            report["cases"].append(row)
            print(json.dumps(row, ensure_ascii=False))
    finally:
        undo()
    report["seconds"] = round(time.time() - t0)
    report["ok"] = all(c["ok"] for c in report["cases"])
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    return report


# -------------------------------------------------------------------------------------------- sweeps and summary

def sweep_values(spec: str) -> tuple[str, list]:
    if "=" not in spec:
        raise SystemExit(f"--sweep {spec!r}: key=v1,v2")
    k, vs = spec.split("=", 1)
    parts = vs.split(";") if ";" in vs else vs.split(",")
    return k.strip(), [H.parse_value(v.strip()) for v in parts]


def label(combo: dict, seed: int) -> str:
    bits = [f"{k.split('.')[-1]}={json.dumps(v) if isinstance(v, list) else v}" for k, v in combo.items()]
    return ("_".join(bits) + "_" if bits else "") + f"seed{seed}"


def summarize(runs: list[dict]) -> list[dict]:
    """Per (case, combination): ok count, means of off-target / fall / execute time."""
    table = {}
    for r in runs:
        for c in r["cases"]:
            key = (c["case"], json.dumps(r["combo"], sort_keys=True))
            t = table.setdefault(key, dict(case=c["case"], combo=r["combo"], runs=0, ok=0, off_mm=[], fall_mm=[], exec_s=[], stops=[]))
            t["runs"] += 1
            t["ok"] += bool(c.get("ok"))
            tr = c.get("truth") or {}
            if tr:
                t["off_mm"].append(tr["off_target_mm"])
                if (tr.get("landed") or {}).get("fall_mm") is not None:
                    t["fall_mm"].append(tr["landed"]["fall_mm"])
            if c.get("execute_s") is not None:
                t["exec_s"].append(c["execute_s"])
                t["stops"].append(c["stops"])
    mean = lambda xs: round(statistics.fmean(xs), 2) if xs else None      # noqa: E731
    return [dict(case=t["case"], combo=t["combo"], ok=f"{t['ok']}/{t['runs']}", off_target_mm=mean(t["off_mm"]),
                 fall_mm=mean(t["fall_mm"]), execute_s=mean(t["exec_s"]), stops=mean(t["stops"]))
            for t in table.values()]


def run(name, sets=None, sweeps=(), repeat: int = 1, seed: int | None = None, console: str | None = None,
        out: Path | None = None) -> dict:
    sc = load(name)
    sim = H.Sim(console)
    base = H.Overrides(sc.get("set") or {}).merged(H.Overrides(sets or {}))
    keys = [sweep_values(s) for s in sweeps]
    combos = [dict(zip([k for k, _ in keys], vals)) for vals in itertools.product(*[v for _, v in keys])] or [{}]
    seed0 = int(sc.get("seed", 1) if seed is None else seed)
    out = Path(out or ROOT / "data" / "sim" / "bench" / sc["name"] / time.strftime("%Y%m%d_%H%M%S"))
    shutil.rmtree(out, ignore_errors=True)
    runs = []
    single = len(combos) == 1 and repeat == 1
    for combo in combos:
        ov = base.merged(H.Overrides(combo))
        for k in range(repeat):
            s = seed0 + k
            where = out if single else out / label(combo, s)
            print(f"== {sc['name']} {label(combo, s)}", flush=True)
            r = run_once(sim, sc, ov, s, where)
            r["combo"] = combo
            runs.append(r)
    summary = dict(scenario=sc["name"], source=sc["source"], console=sim.url, sim_speed=sim.speed,
                   overrides=base.describe(), sweeps=[k for k, _ in keys], repeat=repeat,
                   table=summarize(runs), runs=[{k: v for k, v in r.items() if k != "cases"} | dict(
                       cases=[dict(case=c["case"], ok=c.get("ok")) for c in r["cases"]]) for r in runs],
                   ok=all(r["ok"] for r in runs))
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    return summary


def print_table(summary: dict) -> None:
    print(f"\n{summary['scenario']}  (simulator speed x{summary['sim_speed']:g}; execute_s is simulated wall time)")
    print(f"{'case':10s} {'combination':34s} {'ok':>5s} {'off mm':>7s} {'fall mm':>8s} {'exec s':>7s} {'stops':>6s}")
    for t in summary["table"]:
        combo = ", ".join(f"{k.split('.')[-1]}={v}" for k, v in t["combo"].items()) or "-"
        f = lambda v: "-" if v is None else f"{v:g}"     # noqa: E731
        print(f"{t['case']:10s} {combo[:34]:34s} {t['ok']:>5s} {f(t['off_target_mm']):>7s} {f(t['fall_mm']):>8s} "
              f"{f(t['execute_s']):>7s} {f(t['stops']):>6s}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    r = sub.add_parser("run")
    r.add_argument("scenario")
    r.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    r.add_argument("--sweep", action="append", default=[], metavar="KEY=V1,V2")
    r.add_argument("--repeat", type=int, default=1)
    r.add_argument("--seed", type=int)
    r.add_argument("--console")
    r.add_argument("--out", type=Path)
    a = ap.parse_args(argv)
    if a.cmd == "list":
        for p in sorted(SCENARIOS.glob("*.yaml")):
            sc = load(p)
            print(f"{p.stem:14s} {len(sc['cases'])} cases  {sc.get('title', '')}")
        return 0
    s = run(a.scenario, H.parse_sets(a.set), a.sweep, a.repeat, a.seed, a.console, a.out)
    print_table(s)
    return 0 if s["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
