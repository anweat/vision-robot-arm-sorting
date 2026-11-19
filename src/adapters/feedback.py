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

import time

GREEN = (0, 255, 0)
YELLOW = (255, 200, 0)
RED = (255, 0, 0)
OFF = (0, 0, 0)


class Feedback:
    """Three states on the expansion board; quietly does nothing when Arm_Lib is unavailable."""

    def __init__(self, enabled: bool = True, hold_s: float = 1.2):
        self.enabled = enabled
        self.hold_s = hold_s
        self.arm = None
        self.error = ""
        if not enabled:
            return
        try:
            from Arm_Lib import Arm_Device

            self.arm = Arm_Device()
            time.sleep(0.1)
        except Exception as exc:            # not on the board, or the I2C bus is busy
            self.error = f"{type(exc).__name__}: {exc}"
            self.arm = None

    # ---- states ---------------------------------------------------------------------------------
    def ok(self) -> str:
        return self._show(GREEN, beep_ms=120, name="ok")

    def warn(self) -> str:
        return self._show(YELLOW, beep_ms=0, name="warn")

    def fail(self) -> str:
        return self._show(RED, beep_ms=400, name="fail")

    def off(self) -> str:
        return self._show(OFF, beep_ms=0, name="off")

    # ---- internals ------------------------------------------------------------------------------
    def _show(self, colour, beep_ms: int, name: str) -> str:
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
