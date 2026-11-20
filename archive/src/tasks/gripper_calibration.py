#!/usr/bin/env python3
"""Find where the gripper stops closing: empty, or around a block held between the fingers.

Servo 6 closes in small steps while joints 1-5 hold their starting readback. When the
readback falls behind the command by more than --lag degrees the fingers have met
resistance; the gripper is immediately commanded back to the readback minus 2 degrees,
so the plastic-gear servo never stalls for long (the manual says a stall longer than
3 s trips its protection). Angles grow as the gripper closes (upstream: 0 open, ~130 grip).

  gripper_calibration.py --label empty --execute
  gripper_calibration.py --label block30mm --execute     # someone holds a block in the jaws
Results are appended to runs/gripper_calibration.jsonl.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(APP / "src" / "adapters"))

from safe_arm import DEFAULT_CONFIG, DEFAULT_LOG_DIR, Arm, Logger, MotionError, exclusive_lock, load_config  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", required=True)
    parser.add_argument("--stop", type=int, default=170, help="never command beyond this angle")
    parser.add_argument("--step", type=int, default=3)
    parser.add_argument("--segment-ms", type=int, default=400)
    parser.add_argument("--lag", type=int, default=3, help="readback behind command by more than this = contact")
    parser.add_argument("--end", type=int, help="angle to leave the gripper at afterwards (default: starting angle)")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    cfg = load_config(DEFAULT_CONFIG)
    lo, hi = cfg["soft_limits_deg"][6]
    stop = min(args.stop, hi)
    log = Logger(DEFAULT_LOG_DIR)
    with exclusive_lock():
        arm = Arm(cfg)
        start = arm.read()
        if any(v is None for v in start):
            raise MotionError(f"unreadable joints {start}")
        hold = start[:5]
        end = args.end if args.end is not None else max(start[5], lo)
        log("gripper_plan", label=args.label, execute=args.execute, start=start, stop=stop, step=args.step, end=end)
        if not args.execute:
            return 0
        trace, result = [], {"label": args.label, "start": start[5], "contact_command": None}
        for target in range(max(start[5], lo) + args.step, stop + 1, args.step):
            arm.write6(hold + [target], args.segment_ms)
            time.sleep(args.segment_ms / 1000 + 0.25)
            reading = arm.read()[5]
            trace.append([target, reading])
            if reading is None or target - reading > args.lag:
                relief = max(lo, (reading if reading is not None else target) - 2)
                arm.write6(hold + [relief], 300)
                time.sleep(0.6)
                result.update(contact_command=target, readback_at_contact=reading, relieved_to=relief,
                              readback_after_relief=arm.read()[5])
                break
        result["trace"] = trace
        log("gripper_result", **result)
        arm.write6(hold + [end], 800)
        time.sleep(1.1)
        result["final"] = arm.read()
        log("gripper_done", final=result["final"])
    out = APP / "runs" / "gripper_calibration.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "a", encoding="utf-8") as f:
        f.write(json.dumps({"t": time.time(), **result}) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
