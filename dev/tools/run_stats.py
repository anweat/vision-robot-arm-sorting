"""Per-block records and success rates from the console log, so a claim about the pipeline rests on
counted rounds instead of one demonstration.

    python3 dev/tools/run_stats.py                       # every run in data/runs/console_grasp/log.jsonl
    python3 dev/tools/run_stats.py --since 2026-09-27    # only runs that started on or after that day
    python3 dev/tools/run_stats.py --json stats.json     # the same, machine readable

A run is one pipeline pass: the `pipeline_start`/`pipeline_end` pair, or, for logs written before those
events existed, everything from one scan's first view to the next scan's. Each block that was acted on
gives one record: how long each phase took, how often the gripper had to be woken, the grip readback,
and the outcome. Blocks the run skipped are recorded with their reason, not dropped, because a round
that sorts three of four is not a round that sorted everything.

What this cannot tell you: whether a block ended up in the right bin. The log holds what was commanded
and read back, and a servo readback is not a picture of the table -- only a check scan is. Rates here
are "the arm reported a hold and released over the bin", not verified placements.
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT / "data" / "runs" / "console_grasp" / "log.jsonl"
PHASES = ("approach", "wake", "grip", "lift", "put")


def read(path: Path) -> list[dict]:
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass                      # a torn last line while the run is still writing
    return out


def split_runs(events: list[dict]) -> list[dict]:
    """One dict per pipeline pass. `pipeline_start` opens a run; failing that (older logs) the first
    scan view after any block work does, so a log without the newer events still splits sensibly."""
    runs, cur = [], None

    def new_run(e, explicit):
        nonlocal cur
        cur = dict(t=e["t"], run=e.get("run"), explicit=explicit, events=[], scan_views=0, refines=0)
        runs.append(cur)

    for e in events:
        kind = e.get("event")
        if kind == "pipeline_start":
            if cur is None or cur["events"]:
                new_run(e, True)
            else:                       # the scan right before it is this pass's own scan
                cur.update(explicit=True, run=e.get("run") or cur["run"])
        elif kind == "scan_view" and (cur is None or cur["events"]
                                      or (cur["run"] and e.get("run") and cur["run"] != e["run"])):
            new_run(e, False)
        if cur is None:
            continue
        if kind == "scan_view":
            cur["scan_views"] += 1
            cur["run"] = e.get("run") or cur["run"]
            cur["scan_end"] = e["t"]
        elif kind == "refine_view":
            cur["refines"] += 1
            cur["scan_end"] = e["t"]
        else:
            cur["events"].append(e)
    return [r for r in runs if r["events"] or r["scan_views"]]


def blocks_of(run: dict) -> list[dict]:
    """One record per block the run acted on, in order. Phases are measured between the events the
    console already writes; a phase the log does not cover stays None rather than being guessed."""
    out, cur = [], None
    prev_end = run.get("scan_end", run["t"])
    for e in run["events"]:
        kind = e.get("event")
        if kind == "hovering":
            cur = dict(at=e["t"], cube=None, wake_refusals=0, not_closed=0, grip_readback=None,
                       outcome=None, reason=None, bin=None, category=None, source=None,
                       aligned=e.get("aligned"), block_yaw_deg=e.get("block_yaw_deg"),
                       approach=round(e["t"] - prev_end, 1), **{p: None for p in PHASES[1:]})
            out.append(cur)
            cur["_hover_t"] = e["t"]
        elif cur is None:
            if kind == "pipeline_skip":
                out.append(dict(at=e["t"], cube=e.get("cube"), category=e.get("category"), outcome="skipped",
                                reason=e.get("reason"), wake_refusals=0, not_closed=0, grip_readback=None,
                                bin=None, source=None, aligned=None, block_yaw_deg=None,
                                **{p: None for p in PHASES}))
            continue
        elif kind == "gripper_wake_refused":
            cur["wake_refusals"] += 1
        elif kind in ("gripper_awake", "gripper_still_asleep"):
            cur["wake"] = round(e["t"] - cur["_hover_t"], 1)
            cur["_wake_t"] = e["t"]
        elif kind == "gripper_not_closed":
            cur["not_closed"] += 1
        elif kind == "gripped":
            cur["grip_readback"] = (e.get("grip") or {}).get("readback")
            cur["grip"] = round(e["t"] - cur.get("_wake_t", cur["_hover_t"]), 1)
            cur["_grip_t"] = e["t"]
        elif kind == "lifted":
            cur["lift"] = round(e["t"] - cur.get("_grip_t", cur["_hover_t"]), 1)
        elif kind == "put_planned":
            cur["_put_t"] = e["t"]
        elif kind == "put":
            cur["put"] = round(e["t"] - cur.get("_put_t", cur["_hover_t"]), 1)
            prev_end = e["t"]
        elif kind in ("pipeline_sorted", "pipeline_missed", "scramble_moved", "scramble_returned",
                      "pipeline_cleared", "pipeline_put_failed"):
            cur.update(cube=e.get("cube") or e.get("frm"), category=e.get("category"),
                       bin=e.get("bin") or e.get("marker"), source=e.get("source"), seconds=e.get("seconds"),
                       outcome=dict(pipeline_sorted="sorted", pipeline_missed="missed",
                                    scramble_moved="scattered", scramble_returned="put back",
                                    pipeline_cleared="cleared", pipeline_put_failed="put back")[kind],
                       reason=e.get("reason"))
            prev_end = e["t"]
            cur = None
    for b in out:
        for k in [k for k in b if k.startswith("_")]:
            del b[k]
        if b["outcome"] is None:                    # the run stopped part-way through this block
            b["outcome"] = "unfinished"
    return out


def summarize(runs: list[dict]) -> dict:
    blocks = [b for r in runs for b in r["blocks"]]
    acted = [b for b in blocks if b["outcome"] != "skipped"]
    sorted_ = [b for b in acted if b["outcome"] in ("sorted", "scattered", "cleared")]
    first_try = [b for b in sorted_ if not b["not_closed"]]
    reasons: dict[str, int] = {}
    for b in blocks:
        if b["outcome"] == "skipped" and b["reason"]:
            key = b["reason"].split(":")[0].split("(")[0].strip()
            reasons[key] = reasons.get(key, 0) + 1

    def times(key):
        vals = [b[key] for b in acted if b.get(key) is not None]
        return dict(n=len(vals), median=round(statistics.median(vals), 1), max=max(vals)) if vals else None

    by_align = {}
    for b in acted:
        if b.get("aligned") is not None:
            got = by_align.setdefault("对齐" if b["aligned"] else "未对齐", [0, 0])
            got[b["outcome"] in ("sorted", "scattered", "cleared")] += 1
    return dict(
        grip_by_alignment={k: dict(失败=v[0], 成功=v[1]) for k, v in by_align.items()},
        runs=len(runs), blocks_acted_on=len(acted), sorted=len(sorted_),
        missed=sum(b["outcome"] == "missed" for b in acted), put_back=sum(b["outcome"] == "put back" for b in acted),
        unfinished=sum(b["outcome"] == "unfinished" for b in acted), skipped=sum(b["outcome"] == "skipped" for b in blocks),
        grip_first_try=len(first_try), grip_retried=len(sorted_) - len(first_try),
        woke_first_try=sum(b["wake_refusals"] == 0 for b in acted), wake_refusals=sum(b["wake_refusals"] for b in acted),
        skip_reasons=reasons, phase_seconds={p: times(p) for p in PHASES},
        block_seconds=times("seconds"),
        scan_views=[r["scan_views"] for r in runs], refines=[r["refines"] for r in runs],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", nargs="?", type=Path, default=LOG)
    parser.add_argument("--since", help="only runs starting on or after this date, YYYY-MM-DD local time")
    parser.add_argument("--json", type=Path, help="write the records and the summary here")
    args = parser.parse_args()

    runs = split_runs(read(args.log))
    if args.since:
        cut = time.mktime(time.strptime(args.since, "%Y-%m-%d"))
        runs = [r for r in runs if r["t"] >= cut]
    for r in runs:
        r["blocks"] = blocks_of(r)

    for r in runs:
        print(f"\n{time.strftime('%m-%d %H:%M', time.localtime(r['t']))}  {r.get('run') or '-'}  "
              f"{r['scan_views']} 视角 + {r['refines']} 补拍")
        for b in r["blocks"]:
            where = f"({b['cube'][0]:6.1f},{b['cube'][1]:5.1f})" if b.get("cube") else "      ?      "
            phases = " ".join(f"{p}{b[p]:>5.1f}" for p in PHASES if b.get(p) is not None)
            extra = f"  唤醒重试 {b['wake_refusals']}" if b["wake_refusals"] else ""
            extra += f"  夹取重试 {b['not_closed']}" if b["not_closed"] else ""
            extra += f"  夹取读回 {b['grip_readback']}" if b["grip_readback"] else ""
            print(f"  {b['outcome']:10s} {where} {str(b.get('category') or '-'):11s} {phases}{extra}"
                  + (f"  [{b['reason']}]" if b.get("reason") else ""))

    s = summarize(runs)
    print(f"\n{s['runs']} 轮，动作 {s['blocks_acted_on']} 块：成功 {s['sorted']}，夹空 {s['missed']}，"
          f"放回 {s['put_back']}，未完成 {s['unfinished']}，跳过 {s['skipped']}")
    if s["blocks_acted_on"]:
        print(f"夹取一次成功 {s['grip_first_try']}/{s['sorted']}，夹爪一次唤醒 {s['woke_first_try']}/{s['blocks_acted_on']}"
              f"（共 {s['wake_refusals']} 次唤醒被拒）")
        for p in PHASES:
            t = s["phase_seconds"][p]
            if t:
                print(f"  {p:9s} 中位 {t['median']:5.1f} s  最大 {t['max']:5.1f} s  (n={t['n']})")
        if s["block_seconds"]:
            print(f"  每块合计   中位 {s['block_seconds']['median']:5.1f} s  最大 {s['block_seconds']['max']:5.1f} s")
    if s["grip_by_alignment"]:
        # whether ROLL_90_MIN_PITCH may be relaxed rests on this, not on a guess
        print("按手指对齐:", ", ".join(f"{k} 成功 {v['成功']}/{v['成功'] + v['失败']}"
                                       for k, v in s["grip_by_alignment"].items()))
    if s["skip_reasons"]:
        print("跳过原因:", ", ".join(f"{k} × {v}" for k, v in sorted(s["skip_reasons"].items(), key=lambda kv: -kv[1])))

    if args.json:
        args.json.write_text(json.dumps(dict(runs=runs, summary=s), ensure_ascii=False, indent=1))
        print("写入", args.json)


if __name__ == "__main__":
    main()
