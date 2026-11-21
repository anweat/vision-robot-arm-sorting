"""Flow scripts: parse, check, summarise, store. Standard library only (the gateway uses it without numpy).

    check_text(text) -> dict(ok, errors, warnings, script, uses, moves, unavailable, partial)

Every problem carries a `path` into the script (`steps[2].do[0].on_result.empty[1]`) so a page can point at the
line it is about. Errors make a script unrunnable; warnings do not (an `on_result` branch for a result the
step never gives, a `with:` key the step ignores, a capability that is not wired up yet). The orchestrator
runs only scripts with no errors and refuses, before anything moves, one that uses an `unavailable`
capability (capabilities.py).

Scripts live in two places: config/flows/ (shipped with the app, read-only from the pages) and data/flows/
(saved from any end). Both are listed together; a saved script may not take a shipped one's name.
"""
from __future__ import annotations

import hashlib
import json
import re
import time
from pathlib import Path

from . import miniyaml
from .capabilities import CAPABILITIES, CONTROL, ORDERS, RECORDS, THEN, TOP_KEYS

NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")
RESERVED = {"check", "save", "delete", "capabilities"}   # gateway paths under /api/scripts/
MAX_TEXT = 64_000


class ScriptError(ValueError):
    """The first error of a script, for callers that want an exception (orchestrator.validate)."""


# ------------------------------------------------------------------------------------------------ checking

class _Checker:
    def __init__(self):
        self.errors: list[dict] = []
        self.warnings: list[dict] = []
        self.uses: dict[str, int] = {}
        self.blockless: list[tuple[str, str]] = []     # (path, step) of block steps outside for_each

    def err(self, path: str, msg: str) -> None:
        self.errors.append(dict(path=path, message=msg))

    def warn(self, path: str, msg: str) -> None:
        self.warnings.append(dict(path=path, message=msg))

    def script(self, s) -> None:
        if not isinstance(s, dict):
            return self.err("", "流程脚本顶层必须是映射")
        extra = set(s) - set(TOP_KEYS)
        if extra:
            self.err("", f"流程脚本有未知的顶层键：{sorted(extra)}")
        if not isinstance(s.get("steps"), list) or not s["steps"]:
            return self.err("steps", "流程脚本缺少 steps 列表")
        sp = s.get("speed")
        if sp is not None and (not _num(sp) or not 3 <= sp <= 40):
            self.err("speed", "speed 应为 3–40 的数字（°/s）")
        mo = s.get("motion")
        if mo is not None and mo not in ("steps", "continuous"):
            self.err("motion", "motion 应为 steps 或 continuous")
        n = s.get("stop_after_alarms")
        if n is not None and (not isinstance(n, int) or isinstance(n, bool) or n < 1):
            self.err("stop_after_alarms", "stop_after_alarms 应为 ≥1 的整数")
        self.actions(s["steps"], "steps", in_block=False)

    def actions(self, actions, where: str, in_block: bool) -> None:
        if not isinstance(actions, list):
            return self.err(where, f"{where} 必须是列表")
        for i, a in enumerate(actions):
            self.action(a, f"{where}[{i}]", in_block)

    def action(self, a, at: str, in_block: bool) -> None:
        if not isinstance(a, dict):
            return self.err(at, f"{at} 必须是映射")
        kind = next((k for k in CONTROL if k in a), None)
        if kind is None:
            return self.err(at, f"{at} 不认识：{_short(a)}")
        extra = set(a) - set(CONTROL[kind]["keys"])
        if extra:
            self.err(at, f"{at}（{kind}）有未知的键：{sorted(extra)}")
        if kind == "step":
            self.step(a, at, in_block)
        elif kind == "for_each":
            if a["for_each"] != "blocks":
                self.err(at, f"{at}：for_each 只支持 blocks")
            if in_block:
                self.err(at, f"{at}：for_each 不能嵌套")
            if a.get("order", "reach") not in ORDERS:
                self.err(at, f"{at}：order 只允许 {ORDERS}")
            self.actions(a.get("do"), f"{at}.do", True)
        elif kind == "retry":
            r = a["retry"]
            if not isinstance(r, dict) or set(r) - {"times", "until", "do", "else"} or \
                    not isinstance(r.get("times"), int) or isinstance(r.get("times"), bool) or r["times"] < 1:
                return self.err(at, f"{at}：retry 需要 times（≥1）、until、do，可选 else")
            self.actions(r.get("do"), f"{at}.retry.do", in_block)
            self.actions(r.get("else") or [], f"{at}.retry.else", in_block)
            if r.get("until") is None:
                self.warn(at, f"{at}：retry 没写 until，do 总会做满 {r['times']} 次再走 else")
        elif kind == "repeat":
            r = a["repeat"]
            if not isinstance(r, dict) or set(r) - {"times", "do"} or not isinstance(r.get("times"), int) or \
                    isinstance(r.get("times"), bool) or not 1 <= r["times"] <= 1000:
                return self.err(at, f"{at}：repeat 需要 times（1–1000）和 do")
            self.actions(r.get("do"), f"{at}.repeat.do", in_block)
        elif kind == "alarm":
            if not isinstance(a["alarm"], str) or not a["alarm"]:
                self.err(at, f"{at}：alarm 的值是报警类型（字符串）")
            if a.get("then", "continue") not in THEN:
                self.err(at, f"{at}：then 只允许 {tuple(THEN)}")
            if a.get("then") == "skip_block" and not in_block:
                self.err(at, f"{at}：skip_block 只能在 for_each 里用")
        elif kind == "record":
            if a["record"] not in RECORDS:
                self.err(at, f"{at}：record 只允许 {tuple(RECORDS)}")
            if not in_block:
                self.err(at, f"{at}：record 只能在 for_each 里用")
        elif kind == "next_block" and not in_block:
            self.err(at, f"{at}：next_block 只能在 for_each 里用")

    def step(self, a: dict, at: str, in_block: bool) -> None:
        name = a["step"]
        cap = CAPABILITIES.get(name)
        if cap is None:
            return self.err(at, f"{at}：没有步骤 {name!r}（可用：{', '.join(CAPABILITIES)}）")
        self.uses[name] = self.uses.get(name, 0) + 1
        if cap["block"] and not in_block:
            self.blockless.append((at, name))
        params = a.get("with")
        if params is not None and not isinstance(params, dict):
            self.err(at, f"{at}：with 必须是映射")
            params = {}
        params = params or {}
        for key, spec in cap["params"].items():
            if spec.get("required") and key not in params and spec.get("default") in (None, ""):
                self.err(f"{at}.with.{key}", f"{at}：{name} 缺少参数 {key}（{spec['label']}）")
        for key, value in params.items():
            spec = cap["params"].get(key)
            if spec is None:
                self.warn(f"{at}.with.{key}", f"{at}：{name} 不认参数 {key}，会被忽略")
                continue
            problem = _param_problem(value, spec)
            if problem:
                self.err(f"{at}.with.{key}", f"{at}：参数 {key} {problem}")
        branches = a.get("on_result")
        if branches is None:
            return
        if not isinstance(branches, dict):
            return self.err(f"{at}.on_result", f"{at}：on_result 必须是映射")
        for result, sub in branches.items():
            if result != "default" and result not in cap["results"]:
                self.warn(f"{at}.on_result.{result}",
                          f"{at}：{name} 不会给出 {result}（会给：{', '.join(cap['results'])}）")
            self.actions(sub, f"{at}.on_result.{result}", in_block)


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _short(a) -> str:
    return json.dumps(a, ensure_ascii=False)[:80]


def _param_problem(value, spec: dict) -> str | None:
    t = spec["type"]
    if value is None:
        return None if not spec.get("required") else "不能为空"
    if t == "enum":
        return None if value in spec["enum"] else f"只允许 {spec['enum']}"
    if t in ("num", "int"):
        if not _num(value) or (t == "int" and not isinstance(value, int)):
            return "应为整数" if t == "int" else "应为数字"
        lo, hi = spec.get("min"), spec.get("max")
        if (lo is not None and value < lo) or (hi is not None and value > hi):
            return f"应在 {lo}–{hi}{spec.get('unit', '')}"
        return None
    if t == "pose":
        ok = isinstance(value, list) and len(value) == 6 and all(
            v is None or (_num(v) and 0 <= v <= 270) for v in value) and any(v is not None for v in value)
        return None if ok else "应为 6 个角度（0–270 度，null 表示不动，至少一个不是 null）"
    if t == "bool":
        return None if isinstance(value, bool) else "应为 true / false"
    if t == "str":
        return None if isinstance(value, str) else "应为字符串"
    return None


def check(script) -> dict:
    c = _Checker()
    c.script(script)
    if c.blockless:
        names = "、".join(dict.fromkeys(n for _, n in c.blockless))
        c.warn(c.blockless[0][0], f"{names} 要有当前块，放在 for_each 外只在单步执行（指定了块）时能跑")
    uses = sorted(c.uses)
    unavailable = [u for u in uses if CAPABILITIES[u]["status"] == "unavailable"]
    partial = [u for u in uses if CAPABILITIES[u]["status"] == "partial"]
    for u in unavailable:
        c.warn("", f"用到了还不可用的能力 {u}（{CAPABILITIES[u]['label']}）：{CAPABILITIES[u]['needs']}；运行时会在动臂前拒绝")
    for u in partial:
        c.warn("", f"{u}（{CAPABILITIES[u]['label']}）：{CAPABILITIES[u]['needs']}")
    return dict(ok=not c.errors, errors=c.errors, warnings=c.warnings, uses=uses,
                moves=any(CAPABILITIES[u]["moves"] for u in uses), unavailable=unavailable, partial=partial,
                runnable=not c.errors and not unavailable)


def parse_text(text: str, json_: bool = False):
    if not isinstance(text, str):
        raise ScriptError("脚本内容必须是文本")
    if len(text) > MAX_TEXT:
        raise ScriptError(f"脚本超过 {MAX_TEXT} 字符")
    try:
        return json.loads(text) if json_ else miniyaml.load(text)
    except Exception as exc:                        # miniyaml ValueError / PyYAML YAMLError / JSON error
        raise ScriptError(f"解析失败：{exc}") from exc


def check_text(text: str, json_: bool = False) -> dict:
    try:
        script = parse_text(text, json_)
    except ScriptError as exc:
        return dict(ok=False, errors=[dict(path="", message=str(exc))], warnings=[], script=None, uses=[],
                    moves=False, unavailable=[], partial=[], runnable=False)
    out = check(script)
    out["script"] = script
    return out


def validate(script) -> None:
    """Raise ScriptError with the first error (the orchestrator's old contract)."""
    errors = check(script)["errors"]
    if errors:
        raise ScriptError(errors[0]["message"])


# ------------------------------------------------------------------------------------------------- storage

def rev(text: str) -> str:
    return hashlib.sha1(text.encode("utf-8")).hexdigest()[:12]


class Store:
    """config/flows (builtin, read-only) + data/flows (saved). Names are file stems, [a-z0-9_-]."""

    def __init__(self, builtin: Path, saved: Path):
        self.builtin, self.saved = Path(builtin), Path(saved)
        self._names: dict[str, tuple[float, str]] = {}   # path -> (mtime, title) cache for list()

    def path(self, name: str) -> Path | None:
        if not isinstance(name, str) or not NAME_RE.match(name):
            return None
        for d in (self.saved, self.builtin):
            p = d / f"{name}.yaml"
            if p.is_file():
                return p
        return None

    def is_builtin(self, name: str) -> bool:
        return (self.builtin / f"{name}.yaml").is_file()

    def list(self) -> list[dict]:
        out = []
        for d, builtin in ((self.builtin, True), (self.saved, False)):
            if not d.is_dir():
                continue
            for p in sorted(d.glob("*.yaml")):
                if not NAME_RE.match(p.stem) or (not builtin and self.is_builtin(p.stem)):
                    continue
                text = p.read_text(encoding="utf-8", errors="replace")
                c = check_text(text)
                s = c["script"] if isinstance(c["script"], dict) else {}
                out.append(dict(name=p.stem, builtin=builtin, title=s.get("title") or s.get("name") or p.stem,
                                description=s.get("description"), rev=rev(text), mtime=p.stat().st_mtime,
                                ok=c["ok"], runnable=c["runnable"], uses=c["uses"], moves=c["moves"],
                                unavailable=c["unavailable"], partial=c["partial"],
                                errors=len(c["errors"]), warnings=len(c["warnings"])))
        return out

    def get(self, name: str) -> dict | None:
        p = self.path(name)
        if p is None:
            return None
        text = p.read_text(encoding="utf-8", errors="replace")
        return dict(name=name, builtin=p.parent == self.builtin, text=text, rev=rev(text),
                    mtime=p.stat().st_mtime, path=str(p), check=check_text(text))

    def save(self, name: str, text: str, base_rev: str | None = None) -> dict:
        """Write data/flows/<name>.yaml. Refuses a builtin name, a script with errors, and (with base_rev) a file
        someone else changed since it was loaded: ValueError(message) / ConflictError(current)."""
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise ValueError("脚本名只允许小写字母、数字、_ 和 -，以字母或数字开头，最长 40")
        if name in RESERVED:
            raise ValueError(f"{name} 是保留名，换一个")
        if self.is_builtin(name):
            raise ValueError(f"{name} 是随应用发布的内置脚本，只读；换个名字另存")
        c = check_text(text)
        if not c["ok"]:
            raise ValueError("脚本有错误，没有保存：" + c["errors"][0]["message"])
        target = self.saved / f"{name}.yaml"
        if base_rev is not None and target.is_file():
            current = target.read_text(encoding="utf-8", errors="replace")
            if rev(current) != base_rev:
                raise ConflictError(dict(name=name, text=current, rev=rev(current)))
        self.saved.mkdir(parents=True, exist_ok=True)
        tmp = target.with_suffix(f".tmp{int(time.time() * 1000)}")
        tmp.write_text(text if text.endswith("\n") else text + "\n", encoding="utf-8")
        tmp.replace(target)
        return self.get(name)

    def delete(self, name: str, base_rev: str | None = None) -> bool:
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise ValueError("脚本名不合法")
        if self.is_builtin(name):
            raise ValueError(f"{name} 是内置脚本，不能删")
        target = self.saved / f"{name}.yaml"
        if not target.is_file():
            return False
        if base_rev is not None:
            current = target.read_text(encoding="utf-8", errors="replace")
            if rev(current) != base_rev:
                raise ConflictError(dict(name=name, text=current, rev=rev(current)))
        target.unlink()
        return True


class ConflictError(Exception):
    def __init__(self, current: dict):
        super().__init__("脚本已被另一端改过")
        self.current = current
