#!/usr/bin/env python3
"""The contract for log.jsonl, the event log a front end shows: which events it may rely on, their fields,
and the per-block records derived from them. Standard library only.

    python3 src/tools/log_contract.py check <log.jsonl> [...]     # every line against the contract
    python3 src/tools/log_contract.py records <log.jsonl>         # the per-block records, as JSON lines
    python3 src/tools/log_contract.py schema                      # JSON Schema (docs/contracts/log_event.schema.json)

The log is written by src/tasks/console_grasp.log (one JSON object per line, `t` and `event` always
present) and holds many more events than are listed here: planning and debugging detail that changes with
the code. A reader must ignore events it does not know and fields it does not know. What is listed here is
what the writers promise to keep.

STATUS of each event: 'stable' is written by the code today (seen in real-arm logs, or in the code path);
'planned' is named in docs/atlas-200i-dk-a2/10-需求规格.md and has no writer yet -- its fields are a
proposal to build against, and may still change until the writer exists.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

CATEGORIES = ("recyclable", "kitchen", "hazardous", "other")
VERSION = 1

# field types: num, int, str, bool, xy ([x, y] numbers, map cm), pose ([6 numbers or null], servo degrees),
# list, obj, category (one of CATEGORIES); a trailing '?' also allows null
GRIP = "obj"   # {action, command, readback, at, holding, result}; result: holding / empty / not_closed

EVENTS: dict[str, dict] = {
    # ---- one sorting round
    "pipeline_start": dict(status="stable", meaning="一轮分拣开始；列出扫描找到的全部物块",
                           required=dict(run="str", cubes="list", registration="obj")),
    "pipeline_sorted": dict(status="stable", meaning="一块已放到它的目标位", record="sorted",
                            required=dict(cube="xy", category="category", bin="str", seconds="num"), optional=dict(source="str")),
    "pipeline_missed": dict(status="stable", meaning="夹空或搬运途中滑脱", record="missed",
                            required=dict(cube="xy", category="category?", reason="str", seconds="num")),
    "pipeline_skip": dict(status="stable", meaning="没有去抓：类别证据不足、够不着、已在目标位等", record="skipped",
                          required=dict(cube="xy", category="category?", reason="str")),
    "pipeline_approach_failed": dict(status="stable", meaning="接近途中遇阻，放弃这一块并退回观察位", record="failed",
                                     required=dict(cube="xy", category="category?", reason="str", seconds="num")),
    "pipeline_put_failed": dict(status="stable", meaning="夹住了但放不到目标位，已就地或原位放下", record="failed",
                                required=dict(cube="xy", category="category?", bin="str", reason="str"), optional=dict(released="str")),
    "pipeline_cleared": dict(status="stable", meaning="把挡在别的目标位上的块先挪开（不算分拣完成）", record="cleared",
                             required=dict(cube="xy", category="category?", bin="str", seconds="num"), optional=dict(source="str")),
    "pipeline_stuck": dict(status="stable", meaning="剩下的块互相挡着且挪不开，本轮到此为止",
                           required=dict(bin="str", blocker="xy")),
    "pipeline_end": dict(status="stable", meaning="一轮结束及计数",
                         required=dict(run="str", sorted="int", skipped="int", missed="int", cleared="int", seconds="num")),
    # ---- progress within a round (for a live view; not needed for records)
    "scan_view": dict(status="stable", meaning="扫描到第几个视角",
                      required=dict(run="str", view="int", pose="pose"), optional=dict(markers="list?", official="list")),
    "gripped": dict(status="stable", meaning="合爪后的读回判定", required=dict(pose="pose", grip=GRIP), optional=dict(tip_cm="list")),
    "put": dict(status="stable", meaning="已在目标位松手", required=dict(target="str", released_at="pose"),
                optional=dict(target_map_cm="xy", pose="pose")),
    # ---- faults worth showing
    "gripper_not_closed": dict(status="stable", meaning="夹爪舵机没有响应合爪指令", required=dict(attempt="int", readback="num")),
    "gripper_awake": dict(status="stable", meaning="小步唤醒后夹爪恢复响应", required=dict(start="num", readback="num")),
    "gripper_still_asleep": dict(status="stable", meaning="唤醒后夹爪仍不响应", required=dict(start="num", readback="num")),
    "gripper_wake_refused": dict(status="stable", meaning="唤醒动作被控制台拒绝", required=dict(reason="str")),
    "stopped": dict(status="stable", meaning="任务中止", required=dict(reason="str")),
    "table_z_disagrees": dict(status="stable", meaning="扫描得到的桌面高度与标定值相差过大，需重新标定",
                              required=dict(scan_cm="num", calibrated_cm="num"), optional=dict(note="str")),
    "interaction": dict(status="stable", meaning="手势或语音触发、且已被受理执行的指令（E1、E7）；被忽略或拒绝的不写；语音没有 confidence",
                        required=dict(source="str", command="str"), optional=dict(confidence="num", detail="str")),
    # ---- planned: no writer yet
    "mode_changed": dict(status="planned", meaning="作业模式切换（W1）",
                         required=dict(mode="str", source="str"), optional=dict(previous="str")),
    "unknown_object": dict(status="planned", meaning="发现不属于四类的物体，不抓取（E6）",
                           required=dict(cube="xy", image="str"), optional=dict(votes="obj")),
    "bin_full": dict(status="planned", meaning="某个容器判定为已满，停止向它投放（E3）",
                     required=dict(bin="str", count="int"), optional=dict(category="category", by="str")),
    "stall": dict(status="planned", meaning="舵机堵转，已停止后续指令（W6）",
                  required=dict(joint="int", expected="num", readback="num"), optional=dict(pose="pose")),
}
RECORD_RESULTS = ("sorted", "missed", "skipped", "failed", "cleared")
MODES = ("manual", "preset", "auto")
SOURCES = ("web", "mobile", "voice", "gesture", "timer", "button")


def _ok(value, kind: str) -> bool:
    if kind.endswith("?"):
        return value is None or _ok(value, kind[:-1])
    number = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool)   # noqa: E731
    if kind == "num":
        return number(value)
    if kind == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if kind == "str":
        return isinstance(value, str)
    if kind == "bool":
        return isinstance(value, bool)
    if kind == "xy":
        return isinstance(value, list) and len(value) == 2 and all(number(v) for v in value)
    if kind == "pose":
        return isinstance(value, list) and len(value) == 6 and all(v is None or number(v) for v in value)
    if kind == "list":
        return isinstance(value, list)
    if kind == "obj":
        return isinstance(value, dict)
    if kind == "category":
        return value in CATEGORIES
    raise ValueError(kind)


def validate(event) -> list[str]:
    """Problems with one decoded log line; [] when it keeps the contract. An event name that is not in
    the contract is not a problem: readers skip those."""
    if not isinstance(event, dict):
        return ["不是 JSON 对象"]
    problems = []
    if not _ok(event.get("t"), "num"):
        problems.append("t 缺失或不是数字")
    name = event.get("event")
    if not isinstance(name, str):
        return problems + ["event 缺失或不是字符串"]
    spec = EVENTS.get(name)
    if spec is None:
        return problems
    for field, kind in spec["required"].items():
        if field not in event:
            problems.append(f"{name} 缺少 {field}")
        elif not _ok(event[field], kind):
            problems.append(f"{name}.{field} 应为 {kind}，实际 {json.dumps(event[field], ensure_ascii=False)[:60]}")
    for field, kind in spec.get("optional", {}).items():
        if field in event and not _ok(event[field], kind):
            problems.append(f"{name}.{field} 应为 {kind}，实际 {json.dumps(event[field], ensure_ascii=False)[:60]}")
    if name == "mode_changed" and event.get("mode") not in MODES:
        problems.append(f"mode_changed.mode 应为 {'/'.join(MODES)}")
    return problems


def read(path: Path, after: int = 0, limit: int | None = None) -> tuple[list[dict], int]:
    """Events with seq > after, each given its `seq` (1-based line number), and the file's line count.
    Lines that are not JSON keep their number and come back as {"seq", "event": "_invalid", "raw"}; a
    last line still being written (no newline yet) is left for the next read."""
    out, total = [], 0
    path = Path(path)
    if not path.exists():
        return out, 0
    with path.open("rb") as f:
        for raw in f:
            if not raw.endswith(b"\n"):
                break
            total += 1
            if total <= after or (limit is not None and len(out) >= limit):
                continue
            try:
                event = json.loads(raw)
                if not isinstance(event, dict):
                    raise ValueError
            except ValueError:
                event = dict(event="_invalid", raw=raw.decode("utf-8", "replace").rstrip("\n")[:500])
            out.append(dict(seq=total, **event))
    return out, total


def records(events: list[dict]) -> list[dict]:
    """One record per block handled, the six items the requirement asks a log page to show: time,
    category, where it was, where it went, how long, and the result -- plus which round it belongs to.

    A round is the events from a pipeline_start to the next pipeline_end; `round` counts the
    pipeline_start events seen so far in what was passed in, so pass the log from its beginning for
    stable numbers. Records outside any round (an older log format) get round 0."""
    out, round_no, run, open_round = [], 0, None, False
    for e in events:
        name = e.get("event")
        if name == "pipeline_start":
            round_no, run, open_round = round_no + 1, e.get("run"), True
        elif name == "pipeline_end":
            open_round = False
        result = EVENTS.get(name, {}).get("record")
        if result is None or validate(e):
            continue
        out.append(dict(seq=e.get("seq"), t=e["t"], round=round_no if open_round else 0, run=run if open_round else None,
                        category=e.get("category"), cube=e["cube"], bin=e.get("bin"), seconds=e.get("seconds"),
                        result=result, reason=e.get("reason"), source=e.get("source")))
    return out


def summary(events: list[dict]) -> dict:
    """Counts a statistics page starts from."""
    rows = records(events)
    by_result = {r: sum(x["result"] == r for x in rows) for r in RECORD_RESULTS}
    by_category = {c: dict(sorted=0, missed=0, skipped=0, failed=0) for c in CATEGORIES}
    for x in rows:
        if x["category"] in by_category and x["result"] in by_category[x["category"]]:
            by_category[x["category"]][x["result"]] += 1
    rounds = [e for e in events if e.get("event") == "pipeline_end" and not validate(e)]
    times = sorted(x["seconds"] for x in rows if x["result"] == "sorted" and x["seconds"] is not None)
    return dict(contract_version=VERSION, events=len(events), records=len(rows), by_result=by_result, by_category=by_category,
                rounds_finished=len(rounds), rounds_all_sorted=sum(1 for e in rounds if e["missed"] == 0 and e["skipped"] == 0),
                seconds_per_sorted_block_median=times[len(times) // 2] if times else None,
                seconds_per_round=[e["seconds"] for e in rounds])


def schema() -> dict:
    """JSON Schema (2020-12) for one log line, generated from EVENTS so the two cannot drift apart."""
    number = {"type": "number"}
    kinds = {"num": number, "int": {"type": "integer"}, "str": {"type": "string"}, "bool": {"type": "boolean"},
             "xy": {"type": "array", "items": number, "minItems": 2, "maxItems": 2},
             "pose": {"type": "array", "items": {"type": ["number", "null"]}, "minItems": 6, "maxItems": 6},
             "list": {"type": "array"}, "obj": {"type": "object"}, "category": {"enum": list(CATEGORIES)}}

    def of(kind):
        if kind.endswith("?"):
            return {"anyOf": [kinds[kind[:-1]], {"type": "null"}]}
        return kinds[kind]

    cases = []
    for name, spec in EVENTS.items():
        props = {f: of(k) for f, k in {**spec["required"], **spec.get("optional", {})}.items()}
        cases.append({"if": {"properties": {"event": {"const": name}}, "required": ["event"]},
                      "then": {"description": f"[{spec['status']}] {spec['meaning']}", "properties": props,
                               "required": list(spec["required"])}})
    return {"$schema": "https://json-schema.org/draft/2020-12/schema", "title": f"log.jsonl event, contract v{VERSION}",
            "description": "One line of log.jsonl. Events and fields not listed here are allowed and must be ignored by readers.",
            "type": "object", "required": ["t", "event"],
            "properties": {"t": {"type": "number", "description": "epoch seconds, writer's clock"},
                           "event": {"type": "string"}, "seq": {"type": "integer", "description": "added by the HTTP reader: 1-based line number"}},
            "allOf": cases}


def main(argv: list[str]) -> int:
    if not argv or argv[0] not in ("check", "records", "schema"):
        print(__doc__)
        return 2
    if argv[0] == "schema":
        print(json.dumps(schema(), ensure_ascii=False, indent=1))
        return 0
    bad = 0
    for name in argv[1:]:
        events, total = read(Path(name))
        if argv[0] == "records":
            for r in records(events):
                print(json.dumps(r, ensure_ascii=False))
            continue
        known = sum(e["event"] in EVENTS for e in events)
        problems = [(e["seq"], p) for e in events for p in (["不是合法 JSON"] if e["event"] == "_invalid" else validate(e))]
        for seq, p in problems[:40]:
            print(f"{name}:{seq}: {p}")
        bad += len(problems)
        print(f"{name}: {total} 行，契约内事件 {known} 条，其余 {total - known} 条按约定忽略；问题 {len(problems)} 处；"
              f"记录 {len(records(events))} 条")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
