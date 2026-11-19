"""声光反馈（W5）：成功绿灯、识别失败黄灯、抓取失败红灯并鸣一次。

Board-only: `Arm_Lib` lives in the board's conda env, so off the board every call is a silent no-op
(the pipeline must never fail because a lamp is missing). The 2026-09-18 hardware smoke test proved
`Arm_RGB_set` and `Arm_Buzzer_On/Off` answer without I2C errors, but nobody has looked at the actual
colour or listened to the buzzer yet, so treat "which colour" as unverified (docs/05).

    fb = feedback.Feedback()
    fb.ok()        # green, short beep
    fb.warn()      # yellow
    fb.fail()      # red, longer beep
"""

from __future__ import annotations

import json
import os
import time
import urllib.request

CONSOLE_URL = os.environ.get("ARM_CONSOLE", "http://127.0.0.1:8765").rstrip("/")

GREEN = (0, 255, 0)
YELLOW = (255, 200, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
OFF = (0, 0, 0)


class Feedback:
    """Three states on the expansion board.

    优先走控制台的 /api/led 与 /api/beep：I2C 总线被控制台进程独占，流水线直接开 Arm_Device 会
    I2C error。控制台不可用时退回本地 Arm_Lib；两者都没有就静默。
    """

    def __init__(self, enabled: bool = True, hold_s: float = 1.2):
        self.enabled = enabled
        self.hold_s = hold_s
        self.arm = None
        self.token = ""
        self.http = False
        self.error = ""
        if not enabled:
            return
        try:
            with urllib.request.urlopen(CONSOLE_URL + "/api/bootstrap", timeout=2) as r:
                self.token = json.loads(r.read()).get("token", "")
            self.http = bool(self.token)
            return
        except Exception as exc:
            self.error = f"http: {type(exc).__name__}"
        try:
            from Arm_Lib import Arm_Device

            self.arm = Arm_Device()
            time.sleep(0.1)
        except Exception as exc:            # not on the board, or the I2C bus is busy
            self.error = f"{type(exc).__name__}: {exc}"
            self.arm = None

    # ---- transport ------------------------------------------------------------------------------
    def _call(self, path: str, body: dict) -> str:
        if self.http:
            try:
                body = dict(body, owner="feedback")
                req = urllib.request.Request(CONSOLE_URL + path, data=json.dumps(body).encode(),
                                             method="POST")
                req.add_header("Content-Type", "application/json")
                req.add_header("X-Console-Token", self.token)
                with urllib.request.urlopen(req, timeout=3) as r:
                    json.loads(r.read())
                return "ok"
            except Exception as exc:
                self.error = f"http: {type(exc).__name__}: {exc}"
                return f"failed ({self.error})"
        return ""

    # ---- states ---------------------------------------------------------------------------------
    def ok(self) -> str:
        return self._show(GREEN, beep_ms=120, name="ok")

    def warn(self) -> str:
        return self._show(YELLOW, beep_ms=0, name="warn")

    def fail(self) -> str:
        return self._show(RED, beep_ms=400, name="fail")

    def ready(self) -> str:
        """扫描完成、等操作员确认（挥手 ok）：蓝灯 + 短鸣。"""
        return self._show(BLUE, beep_ms=200, name="ready")

    def off(self) -> str:
        return self._show(OFF, beep_ms=0, name="off")

    # ---- internals ------------------------------------------------------------------------------
    def _show(self, colour, beep_ms: int, name: str) -> str:
        if self.http:
            got = self._call("/api/led", dict(r=colour[0], g=colour[1], b=colour[2]))
            if got != "ok":
                return got
            if beep_ms:
                self._call("/api/beep", dict(ms=beep_ms))
            return name
        if self.arm is None:
            return f"skipped ({self.error or 'disabled'})"
        try:
            self.arm.Arm_RGB_set(*colour)
            if beep_ms:
                self.arm.Arm_Buzzer_On(1)
                time.sleep(beep_ms / 1000.0)
                self.arm.Arm_Buzzer_Off()
            return name
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            return f"failed ({self.error})"


if __name__ == "__main__":
    import sys

    state = sys.argv[1] if len(sys.argv) > 1 else "ok"
    fb = Feedback()
    action = {"ok": fb.ok, "warn": fb.warn, "fail": fb.fail, "off": fb.off}.get(state)
    if action is None:
        print(f"用法: python3 {sys.argv[0]} ok|warn|fail|off")
        raise SystemExit(2)
    print(state, "->", action(), "| error:", fb.error or "-")
