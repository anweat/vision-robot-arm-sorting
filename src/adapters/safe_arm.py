#!/usr/bin/env python3
"""Slow, limit-checked Dofbot motion through the batch write path only.

Only ``Arm_serial_servo_write6`` is used: on this board, single-servo writes to
servos 1-5 did not take effect (2026-09-18 smoke test). A move is split into
small interpolated segments and every segment is verified by readback before
the next one is sent. Any failed readback, I2C error text or deviation stops
further commands; nothing here is an emergency stop for motion already sent.

Without ``--execute`` every command only prints its plan.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "spatial"))
import workspace  # noqa: E402

JOINTS = (1, 2, 3, 4, 5, 6)
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "arm_limits.json"
DEFAULT_LOG_DIR = Path.home() / "arm-logs"
LOCK_PATH = "/tmp/safe_arm.lock"


class MotionError(RuntimeError):
    pass


def load_config(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    for key in ("course_limits_deg", "soft_limits_deg"):
        cfg[key] = {int(k): tuple(v) for k, v in cfg[key].items()}
    return cfg


def validate_target(current: Sequence[Optional[int]], target: Sequence[int], cfg: dict[str, Any]) -> None:
    """A joint that moves must end inside the course and soft limits. A joint that keeps its
    current reading is left alone even if it sits slightly outside (e.g. after being pushed
    by hand), so the others can move, and such a joint may step back towards its range as long
    as every step ends closer to it than it started."""
    def outside(limits, value):
        lo, hi = limits
        return max(lo - value, value - hi, 0)

    for joint, cur, tgt in zip(JOINTS, current, target):
        if cur is not None and tgt == cur:
            continue
        soft, course = cfg["soft_limits_deg"][joint], cfg["course_limits_deg"][joint]
        if cur is not None and outside(soft, cur) > 0 and outside(soft, tgt) < outside(soft, cur) \
                and outside(course, tgt) <= outside(course, cur):
            continue  # a joint already past its limit may step back towards it
        lo, hi = cfg["course_limits_deg"][joint]
        if not lo <= tgt <= hi:
            raise MotionError(f"servo {joint}: target {tgt} outside course limit [{lo}, {hi}]")
        lo, hi = cfg["soft_limits_deg"][joint]
        if not lo <= tgt <= hi:
            raise MotionError(f"servo {joint}: target {tgt} outside soft limit [{lo}, {hi}]")


def check_keep_out(current: Sequence[int], target: Sequence[int], cfg: dict[str, Any],
                   recover: bool = False, table_margin: Optional[float] = None) -> float:
    """Reject paths that bring the gripper into the platform or table keep-out.

    A start that is already inside is refused too: on 2026-09-24 the return from such a
    pose looked like it only improved the margin and still hit the platform. With
    recover=True (a person watching decided) a path that never gets worse than its start
    is allowed."""
    ko = workspace.load_keep_out(cfg)
    if table_margin is not None:  # grasping: the fingertips must get close to the table
        ko["table_margin_m"] = table_margin
    along = workspace.path_clearance(current, target, ko)
    start = workspace.clearance(current, ko)
    if along < 0 and not (recover and along >= start - 1e-9):
        raise MotionError(f"path enters the keep-out by {-along * 100:.1f} cm (start margin {start * 100:+.1f} cm)"
                          + ("" if recover else "; use recover only while watching the arm"))
    return along


def grip_state(reading: Optional[float], gripper: dict) -> str:
    """Classify the gripper readback after a grip command: 'holding' (stopped on an object),
    'empty' (closed to the command), 'not_closed' (never really closed: stalled or ignored)."""
    if reading is None:
        return "unknown"
    if reading >= gripper["holding_if_readback_below_deg"]:
        return "empty"
    if reading < gripper.get("holding_if_readback_above_deg", 100):
        return "not_closed"
    return "holding"


# What Arm_serial_servo_write6 accepts at most per servo (it only refuses values above these; the console's own
# limits let J2-J4 go below zero). A joint can *read* past it -- the folded arm reads J2 = 183 -- and the first step
# back towards its range is then the value just inside the call's range, not one more degree outside it:
# 2026-11-06, the console's folded-to-observe plan began [.., 181, ..] and the library refused it.
WRITE_MAX = (180, 180, 180, 180, 270, 180)


def plan_segments(current: Sequence[int], target: Sequence[int], max_step: float) -> list[list[int]]:
    """Linear interpolation so that no joint moves more than max_step per segment (apart from the step that
    brings a joint from outside what the servo call accepts to its edge, WRITE_MAX)."""
    largest = max(abs(t - c) for c, t in zip(current, target))
    if largest == 0:
        return []
    count = math.ceil(largest / max_step)

    def value(j: int, c: int, t: int, i: int) -> int:
        v = round(c + (t - c) * i / count)
        # only a joint heading for a target the call accepts is held to its edge on the way; a target beyond it is left
        # as it is, for validate_target to refuse
        return min(v, WRITE_MAX[j]) if j < len(WRITE_MAX) and t <= WRITE_MAX[j] else v

    return [[value(j, c, t, i) for j, (c, t) in enumerate(zip(current, target))] for i in range(1, count + 1)]


def resolve_target(current: Sequence[Optional[int]], base: Sequence[Optional[int]],
                   sets: dict[int, int], deltas: dict[int, int]) -> list[int]:
    target: list[Optional[int]] = [b if b is not None else c for b, c in zip(base, current)]
    for joint, value in sets.items():
        target[joint - 1] = value
    for joint, value in deltas.items():
        if current[joint - 1] is None:
            raise MotionError(f"servo {joint}: cannot apply a delta to an unreadable joint")
        target[joint - 1] = current[joint - 1] + value
    for joint, value in zip(JOINTS, target):
        if value is None:
            raise MotionError(f"servo {joint}: current reading unknown and no target given")
    return [int(v) for v in target]


def parse_assignments(items: Sequence[str], signed: bool) -> dict[int, int]:
    out: dict[int, int] = {}
    for item in items:
        joint_text, _, value_text = item.partition("=")
        joint = int(joint_text)
        if joint not in JOINTS or not value_text:
            raise MotionError(f"bad assignment {item!r}; use SERVO=ANGLE, e.g. 2=85")
        if not signed and value_text[0] in "+-":
            raise MotionError(f"{item!r}: use --delta for relative moves")
        out[joint] = int(value_text)
    return out


class Arm:
    """Arm_Lib wrapper: batch writes only, retries on reads, I2C error text treated as failure."""

    def __init__(self, cfg: dict[str, Any]):
        from Arm_Lib import Arm_Device

        self.dev = Arm_Device()
        self.attempts = cfg["motion"]["read_attempts"]
        time.sleep(0.1)

    def read(self) -> list[Optional[int]]:
        values: list[Optional[int]] = []
        for joint in JOINTS:
            value = None
            for _ in range(self.attempts):
                with contextlib.redirect_stdout(io.StringIO()):
                    value = self.dev.Arm_serial_servo_read(joint)
                if value is not None:
                    break
                time.sleep(0.05)
            if value is None:
                value = self.read_unclamped(joint)
            values.append(value)
            time.sleep(0.02)
        return values

    def read_unclamped(self, joint: int) -> Optional[int]:
        """Same conversion as Arm_Lib but without its 0-180 (0-270) cut-off, which returns None
        for a joint pushed a few degrees past its end. Values far outside stay None."""
        try:
            self.dev.bus.write_byte_data(self.dev.addr, joint + 0x30, 0x0)
            time.sleep(0.005)
            raw = self.dev.bus.read_word_data(self.dev.addr, joint + 0x30)
        except OSError:
            return None
        raw = (raw >> 8 & 0xFF) | (raw << 8 & 0xFF00)
        if raw == 0:
            return None
        if joint == 5:
            angle = 270 * (raw - 380) / (3700 - 380)
            ok = -15 <= angle <= 285
        else:
            angle = 180 * (raw - 900) / (3100 - 900)
            ok = -15 <= angle <= 195
        if joint in (2, 3, 4):
            angle = 180 - angle
        return int(round(angle)) if ok else None

    def write6(self, pose: Sequence[int], duration_ms: int) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.dev.Arm_serial_servo_write6(*pose, duration_ms)
        if buf.getvalue().strip():
            raise MotionError(f"Arm_serial_servo_write6 reported: {buf.getvalue().strip()}")


class Logger:
    def __init__(self, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / time.strftime("%Y%m%d.jsonl")

    def __call__(self, event: str, **fields: Any) -> None:
        record = {"t": round(time.time(), 3), "event": event, **fields}
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(json.dumps(record, ensure_ascii=False))


def check_reached(observed: Sequence[Optional[int]], expected: Sequence[int],
                  tolerance: float, skip: Sequence[int]) -> list[str]:
    problems = []
    for joint, obs, exp in zip(JOINTS, observed, expected):
        if joint in skip:
            continue
        if obs is None:
            problems.append(f"servo {joint}: readback failed")
        elif abs(obs - exp) > tolerance:
            problems.append(f"servo {joint}: expected {exp}, read {obs}")
    return problems


def run_segments(arm: Arm, log: Logger, segments: list[list[int]], cfg: dict[str, Any],
                 segment_ms: int, skip_verify: Sequence[int], start: Optional[Sequence[int]] = None) -> None:
    """Send segments one by one. A joint lagging its command (or an unreadable joint) means
    the arm met resistance: it is sent back to the last pose that was verified (the start for
    the first segment) so no servo keeps pushing, and the move stops there."""
    settle = cfg["motion"]["settle_ms"] / 1000
    tolerance = cfg["motion"]["readback_tolerance_deg"]
    last_ok = list(start) if start is not None else None
    for index, pose in enumerate(segments, start=1):
        arm.write6(pose, segment_ms)
        time.sleep(segment_ms / 1000 + settle)
        observed = arm.read()
        problems = check_reached(observed, pose, tolerance, skip_verify)
        log("segment", index=index, of=len(segments), command=pose, readback=observed, problems=problems)
        if problems:
            if last_ok is not None and None not in last_ok:
                try:
                    arm.write6(last_ok, segment_ms)
                    time.sleep(segment_ms / 1000 + settle)
                    log("retreat", to=last_ok, readback=arm.read())
                except MotionError as exc:
                    log("retreat_failed", to=last_ok, error=str(exc))
            raise MotionError("resistance or readback failure, retreated and stopped: " + "; ".join(problems))
        last_ok = pose


@contextlib.contextmanager
def exclusive_lock():
    import fcntl

    with open(LOCK_PATH, "w") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise MotionError(f"another safe_arm process holds {LOCK_PATH}")
        yield


def cmd_move(args: argparse.Namespace, cfg: dict[str, Any], arm: Arm, log: Logger) -> None:
    current = arm.read()
    if any(c is None for c in current):
        raise MotionError(f"unreadable joints in {current}; run recenter-unknown first")
    base: list[Optional[int]] = [None] * 6
    if args.pose:
        base = list(cfg["named_poses"][args.pose])
    target = resolve_target(current, base, parse_assignments(args.set, False),
                            parse_assignments(args.delta, True))
    validate_target(current, target, cfg)
    margin = check_keep_out(current, target, cfg, recover=args.recover, table_margin=args.table_margin)
    step = min(args.step or cfg["motion"]["max_step_deg"], cfg["motion"]["max_step_deg"])
    segment_ms = max(args.segment_ms or cfg["motion"]["segment_ms"], cfg["motion"]["segment_ms"])
    segments = plan_segments(current, target, step)
    log("plan", execute=args.execute, current=current, target=target, segments=len(segments),
        keep_out_margin_cm=round(margin * 100, 1),
        step_deg=step, segment_ms=segment_ms, est_seconds=round(len(segments) * (segment_ms + cfg["motion"]["settle_ms"] + 400) / 1000, 1))
    if args.execute and segments:
        run_segments(arm, log, segments, cfg, segment_ms, [6] if args.no_verify_gripper else [], start=current)
        log("done", readback=arm.read())


class SafeArm:
    """In-process access with the same guarantees as the CLI `move`: soft limits, small
    interpolated steps, readback verification, one process at a time.

    With execute=False nothing is written; move() logs the plan and returns the target.
    Pass `arm` (anything with read() and write6()) to use a stand-in without Arm_Lib."""

    def __init__(self, execute: bool = False, config: Path = DEFAULT_CONFIG, log_dir: Path = DEFAULT_LOG_DIR,
                 arm: Optional[Any] = None):
        self.cfg = load_config(config)
        self.log = Logger(log_dir)
        self.execute = execute
        self._lock = None
        if execute:
            import fcntl

            self._lock = open(LOCK_PATH, "w")
            try:
                fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self._lock.close()
                raise MotionError(f"another safe_arm process holds {LOCK_PATH}")
        self.arm = arm if arm is not None else Arm(self.cfg)

    def close(self) -> None:
        if self._lock:
            self._lock.close()
            self._lock = None

    def read(self) -> list[Optional[int]]:
        return self.arm.read()

    def move(self, target: Sequence[Optional[float]], verify_gripper: bool = True, label: str = "",
             recover: bool = False, table_margin: Optional[float] = None) -> list[int]:
        """Move to servo angles 1..6; None or a missing trailing value keeps the current reading."""
        current = self.read()
        if any(c is None for c in current):
            raise MotionError(f"unreadable joints in {current}")
        if len(target) > len(JOINTS):
            raise MotionError(f"target has {len(target)} values, expected at most {len(JOINTS)}")
        padded = list(target) + [None] * (len(JOINTS) - len(target))  # missing joints keep their reading
        goal = [c if t is None else int(round(t)) for c, t in zip(current, padded)]
        validate_target(current, goal, self.cfg)
        margin = check_keep_out(current, goal, self.cfg, recover=recover, table_margin=table_margin)
        segments = plan_segments(current, goal, self.cfg["motion"]["max_step_deg"])
        self.log("plan", label=label, execute=self.execute, current=current, target=goal, segments=len(segments),
                 keep_out_margin_cm=round(margin * 100, 1))
        if not self.execute:
            return goal
        if segments:
            run_segments(self.arm, self.log, segments, self.cfg, self.cfg["motion"]["segment_ms"],
                         [] if verify_gripper else [6], start=current)
        readback = self.read()
        self.log("done", label=label, readback=readback)
        return readback


def cmd_recenter_unknown(args: argparse.Namespace, cfg: dict[str, Any], arm: Arm, log: Logger) -> None:
    """Command unreadable joints 2-4 to 90 in one slow batch write; readable joints hold their reading."""
    current = arm.read()
    unknown = [j for j, v in zip(JOINTS, current) if v is None]
    if not unknown:
        log("plan", execute=args.execute, current=current, note="all joints readable, nothing to do")
        return
    if set(unknown) - {2, 3, 4}:
        raise MotionError(f"unreadable joints {unknown}; only 2-4 may be recentered blindly")
    target = [90 if v is None else v for v in current]
    validate_target(current, target, cfg)
    log("plan", execute=args.execute, current=current, target=target, duration_ms=args.duration_ms)
    if args.execute:
        run_segments(arm, log, [target], cfg, args.duration_ms, [])
        log("done", readback=arm.read())


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("read", help="print current readings")

    move = sub.add_parser("move", help="slow interpolated move (plan only without --execute)")
    move.add_argument("--pose", help="named pose from config; null entries keep current")
    move.add_argument("--set", action="append", default=[], metavar="SERVO=ANGLE")
    move.add_argument("--delta", action="append", default=[], metavar="SERVO=+-DEG")
    move.add_argument("--step", type=float, help="max degrees per segment (cannot exceed config)")
    move.add_argument("--segment-ms", type=int, help="time per segment (cannot go below config)")
    move.add_argument("--no-verify-gripper", action="store_true", help="skip servo 6 readback check, e.g. when it holds a block")
    move.add_argument("--recover", action="store_true",
                      help="allow leaving the keep-out along a path that never gets worse; only while watching")
    move.add_argument("--table-margin", type=float, help="reduced table keep-out margin (m) for grasp moves")
    move.add_argument("--execute", action="store_true")

    recenter = sub.add_parser("recenter-unknown", help="bring unreadable joints 2-4 to 90 in one slow batch write")
    recenter.add_argument("--duration-ms", type=int, default=4000)
    recenter.add_argument("--execute", action="store_true")

    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    log = Logger(args.log_dir)
    try:
        with exclusive_lock():
            arm = Arm(cfg)
            if args.command == "read":
                log("read", readback=arm.read())
            elif args.command == "move":
                cmd_move(args, cfg, arm, log)
            else:
                cmd_recenter_unknown(args, cfg, arm, log)
    except (MotionError, KeyError) as exc:
        log("error", message=str(exc))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
