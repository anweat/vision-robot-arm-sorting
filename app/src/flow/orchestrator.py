"""Runs a flow script (config/flows/*.yaml) or one step on its own (docs/current/32 §2–§3).

    python src/flow/orchestrator.py --script config/flows/sort.yaml --console http://127.0.0.1:8770 --really
    python src/flow/orchestrator.py --script config/flows/sort.yaml --source data/flow/x/located.json   # dry run
    python src/flow/orchestrator.py --step plan  --session data/flow/S --block b3
    python src/flow/orchestrator.py --step grasp --session data/flow/S --block b3 --really --console URL

Without --really nothing moves: scan needs `source:` (or --source), execute runs as a dry run. With --log the
contract events (`step`, `alarm`, `pipeline_*`, `stopped`) also go to that log, which the gateway and the
simulator serve to the pages. The interpreter knows only the keys below; anything else is an error before
anything runs.

    - step: <name>            with: {...}            on_result: {<result>: [actions], default: [actions]}
    - for_each: blocks        order: reach|scene     do: [actions]
    - retry: {times: N, until: <result>, do: [actions], else: [actions]}
        (inside do, a step result with no branch ends that attempt instead of raising an alarm)
    - repeat: {times: N, do: [actions]}
    - alarm: <kind>           then: skip_block|stop|continue
    - record: sorted|missed|skipped
    - next_block: true        - stop: <reason>

Step names and their `with:` / results: src/flow/capabilities.py (also served by the gateway at
/api/scripts/capabilities). Checking with paths: src/flow/script.py. A script that uses a capability the
board does not have yet (status unavailable) is refused before anything moves. Step events carry `path`,
the action's place in the script (`steps[1].do[2]`), so a page can light up the line that is running.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "tools"))

from flow import actions as A                   # noqa: E402
from flow import miniyaml                       # noqa: E402
from flow import script as FS                   # noqa: E402
from flow import steps as SP                    # noqa: E402
from flow.capabilities import CAPABILITIES      # noqa: E402
from flow.session import FlowSession, load_json  # noqa: E402

STEPS = {**SP.STEPS, **A.STEPS}
# the console's "遇到阻力或偏差…: servo 3: expected 40, read 46" (teleop/core.py): logged as a contract `stall`
STALL_RE = re.compile(r"servo\s*(\d+)\s*:\s*expected\s*(-?\d+(?:\.\d+)?)\s*,\s*read\s*(-?\d+(?:\.\d+)?)")
SUCCESS = {name: tuple(cap["success"]) for name, cap in CAPABILITIES.items()}


class SkipBlock(Exception):
    pass


class StopFlow(Exception):
    pass


class AttemptFailed(Exception):
    """Inside retry.do: a step ended with a result nobody handled; this attempt is over, the retry decides."""


# ------------------------------------------------------------------------------------------- loading

def load_script(path: str | Path) -> dict:
    text = Path(path).read_text(encoding="utf-8")
    script = json.loads(text) if str(path).endswith(".json") else miniyaml.load(text)
    validate(script)
    return script


def validate(script) -> None:
    """Raise ValueError with the first problem (src/flow/script.py has the full check with paths)."""
    FS.validate(script)


# ------------------------------------------------------------------------------------------ running

class Runner:
    def __init__(self, ctx: SP.Context, script: dict | None = None):
        self.ctx = ctx
        self.script = script or {}
        self.alarm_streak = 0
        self.block_alarmed = False
        self.block_started = 0.0
        self.counts = dict(sorted=0, missed=0, skipped=0, alarms=0)
        self.last: SP.Outcome | None = None
        self.retrying = 0

    # ---- events --------------------------------------------------------------------------------------
    def emit(self, event: str, **fields) -> None:
        self.ctx.emit(event, **fields)

    def run(self) -> dict:
        began = time.time()
        sid = self.ctx.sess.path.name
        self.emit("step", session=sid, step="flow", state="started",
                  summary=self.script.get("title") or self.script.get("name", "flow"), path="")
        state, reason = "done", ""
        try:
            self.preflight()
            self.actions(self.script["steps"], in_block=False, path="steps")
        except StopFlow as exc:
            state, reason = "stopped", str(exc)
            self.emit("stopped", reason=reason)
        except Exception as exc:                    # a step raised: stop where we are, say why
            state, reason = "failed", f"{type(exc).__name__}: {exc}"
            self.emit("stopped", reason=reason)
        out = dict(state=state, reason=reason, seconds=round(time.time() - began, 1), session=str(self.ctx.sess.path),
                   **self.counts)
        self.emit("step", session=sid, step="flow", state=state, path="",
                  summary=reason or f"放好 {self.counts['sorted']}，夹空 {self.counts['missed']}，跳过 {self.counts['skipped']}")
        self.ctx.sess.save("flow_result.json", out)
        return out

    def preflight(self) -> None:
        """Before anything moves: a script that needs a capability the board does not have yet is refused whole,
        rather than stopping halfway with the arm somewhere in the middle of the table."""
        c = FS.check(self.script)
        if c["unavailable"]:
            raise StopFlow("脚本用到了还不可用的能力：" + "；".join(
                f"{u}（{CAPABILITIES[u]['needs']}）" for u in c["unavailable"]))

    def actions(self, actions, in_block: bool, path: str = "steps") -> None:
        for i, a in enumerate(actions):
            self.action(a, in_block, f"{path}[{i}]")

    def action(self, a: dict, in_block: bool, path: str = "") -> None:
        if "step" in a:
            self.step(a, in_block, path)
        elif "for_each" in a:
            self.for_each(a, path)
        elif "retry" in a:
            self.retry(a["retry"], in_block, path)
        elif "repeat" in a:
            self.repeat(a["repeat"], in_block, path)
        elif "alarm" in a:
            self.alarm(a["alarm"], a.get("then", "continue"), a.get("summary"))
        elif "record" in a:
            self.record(a["record"], a.get("reason"))
        elif "next_block" in a:
            raise SkipBlock()
        elif "stop" in a:
            raise StopFlow(str(a["stop"]))

    def step(self, a: dict, in_block: bool, path: str = "") -> SP.Outcome:
        name, params = a["step"], dict(a.get("with") or {})
        sid = self.ctx.sess.path.name
        block = self.ctx.block["name"] if self.ctx.block else None
        self.emit("step", session=sid, step=name, state="started", block=block, path=path)
        try:
            out = STEPS[name](self.ctx, **params)
        except Exception as exc:
            out = SP.Outcome("failed", f"{type(exc).__name__}: {exc}")
        self.last = out
        branches = a.get("on_result") or {}
        ok = SUCCESS.get(name, ())
        # a result the script answers with an empty branch is an expected one (the closing scan finding the table
        # empty); a result with actions behind it (plan rejected -> skip) still shows as failed
        expected = out.result in ok or branches.get(out.result) == []
        self.emit("step", session=sid, step=name, state="done" if expected else "failed",
                  result=out.result, summary=out.summary, block=block, path=path)
        hit = STALL_RE.search(out.summary or "") if out.result not in ok else None
        if hit:                                     # the console stopped on resistance: the W6 stall record
            self.emit("stall", joint=int(hit.group(1)), expected=float(hit.group(2)), readback=float(hit.group(3)),
                      where=name, reason=(out.summary or "")[:300])
        if out.result in branches:
            self.actions(branches[out.result], in_block, f"{path}.on_result.{out.result}")
        elif out.result not in ok and not (name == "verify" and out.result == "mismatch"):
            if "default" in branches:
                self.actions(branches["default"], in_block, f"{path}.on_result.default")
            elif self.retrying:                     # inside retry.do: end this attempt, let retry judge
                raise AttemptFailed(out.result)
            else:                                   # nobody said what to do with this result
                self.alarm(f"{name}_{out.result}", "skip_block" if in_block else "stop", out.summary)
        return out

    def for_each(self, a: dict, path: str = "") -> None:
        if self.ctx.scene is None:
            raise StopFlow("for_each blocks 之前没有扫描结果")
        for block in SP.order_blocks(self.ctx.scene, a.get("order", "reach")):
            self.ctx.block = dict(block)
            self.block_alarmed = False
            self.block_started = time.time()
            try:
                self.actions(a["do"], in_block=True, path=f"{path}.do")
            except SkipBlock:
                pass
            self.alarm_streak = self.alarm_streak + 1 if self.block_alarmed else 0
            limit = int(self.script.get("stop_after_alarms", 3))
            if self.alarm_streak >= limit:
                raise StopFlow(f"连续 {limit} 块报警，整轮停止")
        self.ctx.block = None

    def retry(self, r: dict, in_block: bool, path: str = "") -> None:
        for attempt in range(1, r["times"] + 1):
            self.emit("step", session=self.ctx.sess.path.name, step="retry", state="started",
                      summary=f"第 {attempt}/{r['times']} 次重试", path=path,
                      block=self.ctx.block["name"] if self.ctx.block else None)
            self.retrying += 1
            try:
                self.actions(r["do"], in_block, f"{path}.retry.do")
            except AttemptFailed:
                pass
            finally:
                self.retrying -= 1
            if self.last is not None and self.last.result == r.get("until"):
                return
        self.actions(r.get("else") or [], in_block, f"{path}.retry.else")

    def repeat(self, r: dict, in_block: bool, path: str = "") -> None:
        for n in range(1, r["times"] + 1):
            self.emit("step", session=self.ctx.sess.path.name, step="repeat", state="started",
                      summary=f"第 {n}/{r['times']} 遍", path=path,
                      block=self.ctx.block["name"] if self.ctx.block else None)
            self.actions(r["do"], in_block, f"{path}.repeat.do")

    def alarm(self, kind: str, then: str, summary: str | None = None) -> None:
        block = self.ctx.block["name"] if self.ctx.block else None
        summary = summary or (self.last.summary if self.last else kind)
        self.counts["alarms"] += 1
        self.block_alarmed = True
        self.emit("alarm", kind=kind, summary=summary, block=block, needs_ack=True)
        if then == "skip_block":
            raise SkipBlock()
        if then == "stop":
            raise StopFlow(f"报警 {kind}：{summary}")

    def record(self, what: str, reason: str | None = None) -> None:
        b = self.ctx.block
        cube, cat = SP.map_cm(b), b.get("class")
        seconds = round(time.time() - self.block_started, 1)
        why = reason or (self.last.summary if self.last else "")
        self.counts[what] += 1
        if what == "sorted":
            self.emit("pipeline_sorted", cube=cube, category=cat, bin=b.get("bin", "table"), seconds=seconds,
                      source="flow")
        elif what == "missed":
            self.emit("pipeline_missed", cube=cube, category=cat, reason=why, seconds=seconds)
        else:
            self.emit("pipeline_skip", cube=cube, category=cat, reason=why)


# -------------------------------------------------------------------------------------------- sinks

def make_sink(sess: FlowSession, log: Path | None, echo: bool = True):
    """Every event into <session>/flow.jsonl; contract events also into `log` (validated first: an event
    that does not keep the contract is kept in the session log and reported on stderr, not written to `log`)."""
    import log_contract as L

    def sink(event: str, **fields) -> None:
        # None means "not given" -- except for a field the contract requires and allows to be null
        # ("category?": a block of unknown category is still recorded with category null)
        required = (L.EVENTS.get(event) or {}).get("required", {})
        fields = {k: v for k, v in fields.items() if v is not None or str(required.get(k, "")).endswith("?")}
        sess.log(event, **fields)
        line = dict(t=time.time(), event=event, **fields)
        if echo:
            print(json.dumps(line, ensure_ascii=False), flush=True)
        if log is None:
            return
        problems = L.validate(line)
        if problems:
            print(f"[contract] {event}: {problems}", file=sys.stderr, flush=True)
            return
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open("a", encoding="utf-8") as f:
            f.write(json.dumps(line, ensure_ascii=False) + "\n")
    return sink


def set_speed(console: str | None, speed) -> None:
    """The script's speed, through the console (any client may set it)."""
    if not console or speed is None:
        return
    import console_step
    console_step.BASE = console.rstrip("/")
    boot = console_step.call("/api/bootstrap")
    console_step.call("/api/speed", dict(speed=speed), token=boot.get("token", ""), owner="flow")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument("--script", help="flow script, e.g. config/flows/sort.yaml")
    what.add_argument("--step", choices=sorted(STEPS) + ["retry"], help="run one step on its own")
    ap.add_argument("--session", help="session directory (default: a new data/flow/<stamp>)")
    ap.add_argument("--block", help="--step: the block (name in the session's latest scan)")
    ap.add_argument("--with", dest="params", default="{}", help="--step: JSON parameters")
    ap.add_argument("--console", help="console URL (simulator http://127.0.0.1:8770)")
    ap.add_argument("--really", action="store_true", help="move the arm (else: dry run)")
    ap.add_argument("--source", help="use this located.json for every scan instead of scanning")
    ap.add_argument("--log", type=Path, help="contract log to append events to (gateway / simulator log)")
    ap.add_argument("--check", action="store_true", help="--script: only load and validate the script")
    args = ap.parse_args(argv)

    if args.script and args.check:
        load_script(args.script)
        print("ok")
        return 0
    sess = FlowSession(args.session)
    ctx = SP.Context(sess=sess, console=args.console, really=args.really, sink=make_sink(sess, args.log))
    if args.script:
        script = load_script(args.script)
        if args.source:
            _with_source(script["steps"], args.source)
        if args.really:
            set_speed(args.console, script.get("speed"))
        out = Runner(ctx, script).run()
        return 0 if out["state"] == "done" else 1
    params = json.loads(args.params) or default_params(args.step)
    if args.step == "scan" and args.source:
        params.setdefault("source", args.source)
    if args.block:
        ctx.load_latest()
        ctx.block = next((b for b in SP.blocks(ctx.scene) if b["name"] == args.block), None)
        if ctx.block is None:
            raise SystemExit(f"最新一次扫描里没有 {args.block}")
        prior = sess.path / "blocks" / args.block / "plan.json"
        if prior.exists():
            ctx.block["bin"] = (load_json(prior)["task"]["to"]).get("bin", "table")
    if args.step == "retry":                    # S5 alone: the retry script, with its alarm
        script = load_script(ROOT / "config" / "flows" / "retry.yaml")
        if args.source:
            _with_source(script["steps"], args.source)
        out = Runner(ctx, script).run()
        return 0 if out["state"] == "done" else 1
    ctx.emit("step", session=sess.path.name, step=args.step, state="started",
             block=ctx.block["name"] if ctx.block else None)
    try:
        out = STEPS[args.step](ctx, **params)
    except Exception as exc:
        out = SP.Outcome("failed", f"{type(exc).__name__}: {exc}")
    ok = out.result in SUCCESS.get(args.step, ())
    ctx.emit("step", session=sess.path.name, step=args.step, state="done" if ok else "failed", result=out.result,
             summary=out.summary, block=ctx.block["name"] if ctx.block else None)
    print(json.dumps(dict(result=out.result, summary=out.summary, data=out.data, session=str(sess.path)),
                     ensure_ascii=False, default=str))
    return 0 if ok else 1


def default_params(step: str, script: Path = ROOT / "config" / "flows" / "sort.yaml") -> dict:
    """A step run on its own takes the `with:` it has in the default flow (e.g. scan locates the same bins)."""
    def walk(actions):
        for a in actions or []:
            if a.get("step") == step and a.get("with"):
                return dict(a["with"])
            for sub in [a.get("do"), *(a.get("on_result") or {}).values(),
                        (a.get("retry") or {}).get("do"), (a.get("retry") or {}).get("else")]:
                found = walk(sub)
                if found is not None:
                    return found
        return None
    try:
        return walk(load_script(script)["steps"]) or {}
    except (OSError, ValueError):
        return {}


def _with_source(actions, source: str) -> None:
    """--source: every scan step reads this located.json instead of moving the arm."""
    for a in actions:
        if a.get("step") == "scan":
            a.setdefault("with", {})
            a["with"] = dict(a["with"] or {}, source=source)
        for sub in (a.get("on_result") or {}).values():
            _with_source(sub, source)
        for key in ("do",):
            if isinstance(a.get(key), list):
                _with_source(a[key], source)
        if "retry" in a:
            _with_source(a["retry"].get("do") or [], source)
            _with_source(a["retry"].get("else") or [], source)
        if "repeat" in a:
            _with_source(a["repeat"].get("do") or [], source)


if __name__ == "__main__":
    sys.exit(main())
