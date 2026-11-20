"""The flow steps that are plain console actions, not part of the grasp chain (capabilities.py groups
motion / interact / signal / flow): pose, gripper, speed, wait, wait_gesture, notify, and the ones the
board cannot do yet (led, buzzer, say, wait_voice), which refuse with what is missing.

Same shape as src/flow/steps.py: `fn(ctx, **params) -> Outcome`. Without `ctx.really` nothing moves (a dry
run answers "空跑"). Tests pass `ctx.backend` with the methods used here (goto / gripper / speed / interact).
Only the standard library plus src/tools/console_step.py.
"""
from __future__ import annotations

import time

from .capabilities import CAPABILITIES
from .steps import Context, Outcome

POLL_S = 0.3


class Unavailable(RuntimeError):
    pass


def _console(ctx: Context):
    """console_step pointed at ctx.console (the same module the grasp steps use)."""
    import console_step
    if ctx.console:
        console_step.BASE = ctx.console.rstrip("/")
    return console_step


def _with_session(ctx: Context, fn):
    cs = _console(ctx)
    with cs.Session(owner="flow-action") as s:
        return fn(s)


def pose(ctx: Context, named: str = "observe_v1", **_) -> Outcome:
    backend = ctx.make_backend()
    if backend is not None and hasattr(backend, "goto"):
        backend.goto(named)
        return Outcome("done", f"已到 {named}")
    if not ctx.really:
        return Outcome("done", f"空跑：未动（{named}）")
    try:
        _with_session(ctx, lambda s: s._post("/api/goto", dict(named=named), timeout=120))
    except Exception as exc:
        return Outcome("failed", f"到 {named} 失败：{exc}")
    return Outcome("done", f"已到 {named}")


def gripper(ctx: Context, action: str = "open", **_) -> Outcome:
    backend = ctx.make_backend()
    if backend is not None:
        backend.gripper(action, {"grip": 133, "open": 30, "wide": 11}.get(action, 30))
        result = backend.grip_result() if hasattr(backend, "grip_result") else None
    elif not ctx.really:
        return Outcome("done", f"空跑：夹爪未动（{action}）")
    else:
        try:
            r = _with_session(ctx, lambda s: s.grip(action))
        except Exception as exc:
            return Outcome("failed", f"夹爪 {action} 失败：{exc}")
        result = (r.get("grip") or {}).get("result") if isinstance(r, dict) else None
    if action == "grip" and result in ("holding", "empty"):
        return Outcome(result, "合爪：夹住" if result == "holding" else "合爪：空爪")
    return Outcome("done", {"open": "已张开", "grip": "已合拢", "wide": "已张到最大"}.get(action, action))


def speed(ctx: Context, value: float = 20, **_) -> Outcome:
    backend = ctx.make_backend()
    if backend is not None and hasattr(backend, "speed"):
        backend.speed(value)
        return Outcome("done", f"速度 {value}°/s")
    if not ctx.really:
        return Outcome("done", f"空跑：速度未改（{value}°/s）")
    cs = _console(ctx)
    try:
        boot = cs.call("/api/bootstrap")
        r = cs.call("/api/speed", dict(speed=value), token=boot.get("token", ""), owner="flow")
    except Exception as exc:
        return Outcome("failed", f"改速度失败：{exc}")
    if r.get("error"):
        return Outcome("failed", f"改速度被拒：{r['error']}")
    return Outcome("done", f"速度 {value}°/s")


def wait(ctx: Context, seconds: float = 2, **_) -> Outcome:
    end = time.time() + float(seconds)
    while time.time() < end:                    # a terminate() from the gateway ends the process meanwhile
        time.sleep(min(POLL_S, max(0.0, end - time.time())))
    return Outcome("done", f"等了 {seconds} s")


def _interact(ctx: Context) -> dict:
    backend = ctx.make_backend()
    if backend is not None and hasattr(backend, "interact"):
        return backend.interact()
    return _console(ctx).call("/api/interact")


def wait_gesture(ctx: Context, label: str = "ok", timeout_s: float = 60, **_) -> Outcome:
    """Wait for the recogniser to report `label` (the console's intents.py decides what counts: threshold,
    N frames in a row). Looks at /api/interact `recent` for a gesture newer than the start: accepted (it also
    ran the mapped command) or rejected (not armed, cooldown) both mean the hand was seen."""
    if not ctx.really and ctx.backend is None:
        return Outcome("matched", f"空跑：当作看到了 {label}")
    began = time.time()
    seen_ids: set = set()
    try:
        first = _interact(ctx)
        seen_ids = {r.get("id") for r in first.get("recent", [])}
    except Exception as exc:
        return Outcome("failed", f"读不到手势接口：{exc}")
    while time.time() - began < float(timeout_s):
        try:
            state = _interact(ctx)
        except Exception as exc:
            return Outcome("failed", f"读不到手势接口：{exc}")
        for r in state.get("recent", []):
            if r.get("id") in seen_ids:
                continue
            seen_ids.add(r.get("id"))
            if r.get("source") == "gesture" and r.get("input") == label:
                return Outcome("matched", f"看到手势 {label}（{r.get('status')}：{r.get('reason', '')}）")
        time.sleep(POLL_S)
    return Outcome("timeout", f"{timeout_s} s 内没看到手势 {label}")


def notify(ctx: Context, text: str = "", level: str = "info", **_) -> Outcome:
    return Outcome("done", str(text))


def _unavailable(name: str):
    def fn(ctx: Context, **_) -> Outcome:
        raise Unavailable(f"{CAPABILITIES[name]['label']}不可用：{CAPABILITIES[name]['needs']}")
    fn.__name__ = name
    return fn


STEPS = dict(pose=pose, gripper=gripper, speed=speed, wait=wait, wait_gesture=wait_gesture, notify=notify,
             led=_unavailable("led"), buzzer=_unavailable("buzzer"), say=_unavailable("say"),
             wait_voice=_unavailable("wait_voice"))
