"""Offline round acceptance report for gateway/flow logs (also reads legacy pipeline logs).

python dev/tools/acceptance_stats.py log.jsonl --expected-blocks 4 --json report.json
--placements accepts a JSON list: {round: 1, block: "b1", verified: true/false,
evidence: "photo/or simulator truth", target_m: [x,y], measured_m: [x,y]}.
Software 'sorted' is reported separately from independently verified placement. A missing observation
is unknown, never success. No cross-device timestamps are combined. Exit 1 means damaged/inconsistent input.
"""
import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_logs import distribution, files, number, read, write_json
import run_stats as legacy

TERMINAL = {"pipeline_sorted": "sorted", "pipeline_missed": "missed", "pipeline_skip": "skipped",
            "pipeline_approach_failed": "failed", "pipeline_put_failed": "failed", "pipeline_cleared": "cleared"}


def rounds_from_events(events):
    """Flow boundaries use step:flow; old pipeline boundaries use pipeline_start/end. No time-gap guessing."""
    rounds, current, block = [], None, None
    for e in events:
        is_flow = e.get("event") == "step" and e.get("step") == "flow"
        if (is_flow and e.get("state") == "started") or e.get("event") == "pipeline_start":
            current = dict(id=e.get("session") or e.get("run"), format="flow" if is_flow else "pipeline",
                           started_at=e.get("t"), state="unfinished", events=[])
            rounds.append(current)
            block = None
        if current is None:
            continue
        current["events"].append(e)
        if e.get("event") == "step" and e.get("block"):
            block = e["block"]
        if e.get("event") in TERMINAL:
            e = dict(e, _block=block)
            current["events"][-1] = e
        if (is_flow and e.get("state") != "started") or e.get("event") == "pipeline_end":
            current["state"] = e.get("state", "done")
            current["seconds"] = e.get("seconds")
            if current["seconds"] is None and number(e.get("t")) and number(current["started_at"]):
                current["seconds"] = max(0, e["t"] - current["started_at"])
            current = None
    return rounds


def analyze(inputs, expected_blocks=None, placements=None):
    warnings, rounds = [], []
    for path in files(inputs, "flow.jsonl"):
        events, problems = read(path)
        warnings.extend(problems)
        valid = []
        for index, event in enumerate(events, 1):
            if not isinstance(event.get("event"), str) or not number(event.get("t")):
                warnings.append(f"{path}: record {index}: event and finite numeric t are required")
            else:
                valid.append(event)
        parsed = rounds_from_events(valid)
        if not parsed:
            warnings.append(f"{path}: no explicit round start")
        for r in parsed:
            r["source"] = str(path)
            rounds.append(r)
    for index, r in enumerate(rounds, 1):
        by_block, reasons, starts, phases = {}, Counter(), {}, {}
        r.update(round=index, retries=0, alarms=0, verification="unknown")
        # Preserve the old pipeline's detailed phase interpretation, without using its heuristic boundaries.
        legacy_blocks = legacy.blocks_of(dict(t=r["started_at"], events=r["events"])) if r["format"] == "pipeline" else []
        for e in r.pop("events"):
            name = e.get("event")
            if name == "step":
                key = (e.get("block"), e.get("step"), e.get("path"))
                if e.get("state") == "started" and number(e.get("t")):
                    starts[key] = e["t"]
                elif key in starts and number(e.get("t")):
                    duration = e["t"] - starts.pop(key)
                    if duration >= 0:
                        phases.setdefault(e["step"], []).append(duration)
            if name == "step" and e.get("block"):
                b = by_block.setdefault(e["block"], dict(block=e["block"], outcome="unfinished", seconds=None))
                if e.get("step") == "retry" and e.get("state") == "started":
                    r["retries"] += 1
                if e.get("step") != "retry" and e.get("state") == "failed":
                    b["last_failure"] = e.get("summary") or e.get("result") or e["step"]
                    if number(e.get("t")):
                        b.setdefault("failure_detected_at", e["t"])
            if name == "step" and e.get("step") == "verify" and e.get("state") != "started":
                r["verification"] = e.get("result", "unknown")
            if name == "alarm":
                r["alarms"] += 1
                reasons[e.get("kind", "alarm")] += 1
            if name in TERMINAL:
                key = e.get("_block") or f"record{len(by_block) + 1}"
                b = by_block.setdefault(key, dict(block=key))
                b.update(outcome=TERMINAL[name], category=e.get("category"), cube=e.get("cube"),
                         bin=e.get("bin"), seconds=e.get("seconds"), reason=e.get("reason"))
                if number(b.get("failure_detected_at")) and number(e.get("t")):
                    b["handling_seconds"] = max(0, e["t"] - b["failure_detected_at"])
                if b.get("reason"):
                    reasons[b["reason"]] += 1
        for b in by_block.values():
            if b["outcome"] == "unfinished" and b.get("last_failure"):
                b["outcome"] = "failed"
                b["reason"] = b["last_failure"]
                reasons[b["reason"]] += 1
        if r["format"] == "pipeline":
            # Include interrupted old-style blocks, absent a terminal pipeline event.
            for b in legacy_blocks:
                if b["outcome"] == "unfinished":
                    key = f"unfinished{len(by_block) + 1}"
                    by_block[key] = dict(b, block=key)
        r["blocks"] = list(by_block.values())
        for b in r["blocks"]:
            b["placement"] = dict(verified=None, evidence=None)
        r["counts"] = dict(Counter(b["outcome"] for b in r["blocks"]))
        r["failure_reasons"] = dict(reasons)
        r["step_seconds"] = {k: distribution(v) for k, v in phases.items()}
        r["expected_blocks"] = expected_blocks
        r["unaccounted"] = max(0, expected_blocks - len(r["blocks"])) if expected_blocks is not None else None
        if expected_blocks is not None and len(r["blocks"]) > expected_blocks:
            warnings.append(f"{r['source']}: round {index}: {len(r['blocks'])} records exceed expected {expected_blocks}")
        r["all_reported_sorted"] = bool(r["blocks"]) and r["state"] == "done" and r["verification"] != "mismatch" \
            and all(b["outcome"] == "sorted" for b in r["blocks"]) \
            and (expected_blocks is None or len(r["blocks"]) == expected_blocks)
    seen = set()
    for observation in placements or []:
        try:
            if not isinstance(observation, dict):
                raise ValueError("placement must be an object")
            n, block = observation["round"], observation["block"]
            if not isinstance(n, int) or isinstance(n, bool) or not 1 <= n <= len(rounds):
                raise ValueError("invalid round")
            if (n, block) in seen:
                raise ValueError("duplicate placement")
            b = next(b for b in rounds[n - 1]["blocks"] if b["block"] == block)
            if type(observation.get("verified")) is not bool or not observation.get("evidence"):
                raise ValueError("verified must be boolean and evidence is required")
            value = dict(verified=observation["verified"], evidence=observation["evidence"])
            if "target_m" in observation or "measured_m" in observation:
                a, z = observation["target_m"], observation["measured_m"]
                if len(a) != len(z) or len(a) not in (2, 3) or not all(number(x) for x in [*a, *z]):
                    raise ValueError("positions must have matching 2 or 3 finite coordinates")
                value["offset_mm"] = round(math.dist(a, z) * 1000, 3)
            b["placement"] = value
            seen.add((n, block))
        except (KeyError, TypeError, ValueError, StopIteration) as exc:
            warnings.append(f"placement {observation!r}: {exc}")
    blocks = [b for r in rounds for b in r["blocks"]]
    sorted_blocks = [b for b in blocks if b["outcome"] == "sorted"]
    verified = sum(b["placement"]["verified"] is True for b in blocks)
    known = sum(b["placement"]["verified"] is not None for b in blocks)
    denominator = expected_blocks * len(rounds) if expected_blocks is not None else len(blocks)
    def durations(values):
        return distribution([x for x in values if number(x) and x >= 0])
    summary = dict(rounds=len(rounds), rounds_finished=sum(r["state"] == "done" for r in rounds),
                   rounds_all_reported_sorted=sum(r["all_reported_sorted"] for r in rounds),
                   blocks=len(blocks), counts=dict(Counter(b["outcome"] for b in blocks)), denominator=denominator,
                   reported_success_rate=len(sorted_blocks) / denominator if denominator and len(sorted_blocks) <= denominator else None,
                   placement_verified=verified, placement_observed=known, placement_unknown=max(0, denominator - known),
                   unaccounted_blocks=sum(r["unaccounted"] or 0 for r in rounds),
                   verified_success_rate=verified / denominator if denominator and known == denominator else None,
                   verified_fraction_lower_bound=verified / denominator if denominator else None,
                   retries=sum(r["retries"] for r in rounds), alarms=sum(r["alarms"] for r in rounds),
                   block_seconds=durations(b.get("seconds") for b in blocks),
                   failure_handling_seconds=durations(b.get("handling_seconds") for b in blocks if b["outcome"] != "sorted"),
                   round_seconds=durations(r.get("seconds") for r in rounds))
    return dict(evidence="software logs; placements only when independently supplied", summary=summary,
                rounds=rounds, warnings=warnings)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--expected-blocks", type=int)
    ap.add_argument("--placements", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)
    if args.expected_blocks is not None and args.expected_blocks < 1:
        ap.error("--expected-blocks must be positive")
    try:
        observations = json.loads(args.placements.read_text(encoding="utf-8-sig")) if args.placements else []
        if not isinstance(observations, list):
            raise ValueError("placements must be a list")
        report = analyze(args.inputs, args.expected_blocks, observations)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    for warning in report["warnings"]:
        print(warning, file=sys.stderr)
    if args.json:
        write_json(args.json, report)
    return 1 if report["warnings"] or not report["rounds"] else 0


if __name__ == "__main__":
    sys.exit(main())
