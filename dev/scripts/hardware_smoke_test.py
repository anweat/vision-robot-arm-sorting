#!/usr/bin/env python3
"""Atlas/Dofbot hardware smoke tests with course-defined servo limits."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any


SERVO_LIMITS = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 180),
    4: (0, 180),
    5: (90, 270),
    6: (0, 180),
}


def arm_device():
    from Arm_Lib import Arm_Device

    return Arm_Device()


def read_angle(arm: Any, servo_id: int, attempts: int = 10) -> tuple[int | None, list[int | None]]:
    samples: list[int | None] = []
    for _ in range(attempts):
        value = arm.Arm_serial_servo_read(servo_id)
        samples.append(value)
        if value is not None:
            return value, samples
        time.sleep(0.05)
    return None, samples


def read_raw_position(arm: Any, servo_id: int) -> int | None:
    try:
        arm.bus.write_byte_data(arm.addr, servo_id + 0x30, 0x00)
        time.sleep(0.01)
        value = arm.bus.read_word_data(arm.addr, servo_id + 0x30)
    except Exception:
        return None
    if value == 0:
        return None
    return (value >> 8 & 0xFF) | (value << 8 & 0xFF00)


def safe_write(arm: Any, servo_id: int, angle: int, duration_ms: int) -> None:
    if servo_id not in SERVO_LIMITS:
        raise ValueError(f"unknown servo id: {servo_id}")
    lower, upper = SERVO_LIMITS[servo_id]
    if not lower <= angle <= upper:
        raise ValueError(f"servo {servo_id} target {angle} outside course limit [{lower}, {upper}]")
    arm.Arm_serial_servo_write(servo_id, angle, duration_ms)


def read_arm_state(arm: Any) -> dict[str, Any]:
    state: dict[str, Any] = {
        "hardware_version": arm.Arm_get_hardversion(),
        "servos": {},
    }
    for servo_id in range(1, 7):
        angle, samples = read_angle(arm, servo_id)
        state["servos"][str(servo_id)] = {
            "ping": arm.Arm_ping_servo(servo_id),
            "angle": angle,
            "read_samples": samples,
            "raw_position": read_raw_position(arm, servo_id),
            "course_limit": SERVO_LIMITS[servo_id],
        }
        time.sleep(0.05)
    return state


def test_rgb(arm: Any) -> None:
    for color in ((40, 0, 0), (0, 40, 0), (0, 0, 40)):
        arm.Arm_RGB_set(*color)
        time.sleep(0.4)
    arm.Arm_RGB_set(0, 0, 0)
    print("RGB_TEST=PASS")


def test_buzzer(arm: Any) -> None:
    try:
        arm.Arm_Buzzer_On(1)
        time.sleep(0.3)
    finally:
        arm.Arm_Buzzer_Off()
    print("BUZZER_TEST=PASS")


def test_camera() -> None:
    import cv2

    camera = cv2.VideoCapture(0)
    try:
        if not camera.isOpened():
            raise RuntimeError("camera /dev/video0 did not open")
        frame = None
        for _ in range(10):
            ok, candidate = camera.read()
            if ok and candidate is not None:
                frame = candidate
                break
            time.sleep(0.1)
        if frame is None:
            raise RuntimeError("camera opened but no frame was captured")
        height, width = frame.shape[:2]
        print(json.dumps({"CAMERA_TEST": "PASS", "width": width, "height": height}, ensure_ascii=False))
    finally:
        camera.release()


def choose_probe_target(servo_id: int, current: int, step: int = 5) -> int:
    lower, upper = SERVO_LIMITS[servo_id]
    if not lower <= current <= upper:
        raise ValueError(f"servo {servo_id} current angle {current} outside course limit [{lower}, {upper}]")
    if current + step <= upper - 2:
        return current + step
    if current - step >= lower + 2:
        return current - step
    raise ValueError(f"servo {servo_id} angle {current} has no safe probe target")


def test_motion(arm: Any, step: int = 5) -> None:
    results = []
    for servo_id in range(1, 7):
        current, start_samples = read_angle(arm, servo_id)
        if current is None:
            results.append(
                {"servo": servo_id, "status": "SKIP", "reason": "read failed", "read_samples": start_samples}
            )
            continue
        try:
            target = choose_probe_target(servo_id, current, step=step)
        except ValueError as exc:
            results.append({"servo": servo_id, "status": "SKIP", "reason": str(exc)})
            continue

        safe_write(arm, servo_id, target, 1500)
        time.sleep(1.8)
        observed, _ = read_angle(arm, servo_id)
        safe_write(arm, servo_id, current, 1500)
        time.sleep(1.8)
        restored, _ = read_angle(arm, servo_id)
        reached = observed is not None and abs(observed - target) <= 3
        returned = restored is not None and abs(restored - current) <= 3
        results.append(
            {
                "servo": servo_id,
                "status": "PASS" if reached and returned else "FAIL",
                "start": current,
                "target": target,
                "observed": observed,
                "restored": restored,
                "course_limit": SERVO_LIMITS[servo_id],
            }
        )
    print(json.dumps({"MOTION_TEST": results}, ensure_ascii=False))


def center_unreadable_servos(arm: Any) -> None:
    """Slowly place unreadable arm joints 2-4 at the course-defined 90-degree pose."""
    results = []
    for servo_id in (2, 3, 4):
        before, samples = read_angle(arm, servo_id)
        ping = arm.Arm_ping_servo(servo_id)
        if before is not None:
            results.append({"servo": servo_id, "status": "SKIP", "reason": "already readable", "angle": before})
            continue
        if ping != 0xDA:
            results.append({"servo": servo_id, "status": "SKIP", "reason": "ping failed", "ping": ping})
            continue
        safe_write(arm, servo_id, 90, 3000)
        time.sleep(3.4)
        after, _ = read_angle(arm, servo_id)
        results.append(
            {
                "servo": servo_id,
                "status": "PASS" if after is not None and abs(after - 90) <= 3 else "FAIL",
                "before": before,
                "before_samples": samples,
                "target": 90,
                "after": after,
            }
        )
    print(json.dumps({"CENTER_UNREADABLE_TEST": results}, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--read-only", action="store_true")
    group.add_argument("--rgb", action="store_true")
    group.add_argument("--buzzer", action="store_true")
    group.add_argument("--camera", action="store_true")
    group.add_argument("--motion", action="store_true")
    group.add_argument("--center-unreadable", action="store_true")
    parser.add_argument("--step", type=int, default=5, help="motion probe size in degrees (default: 5)")
    args = parser.parse_args()

    if args.step < 1 or args.step > 10:
        parser.error("--step must be in the safe test range 1..10")

    if args.camera:
        test_camera()
        return

    arm = arm_device()
    if args.read_only:
        print(json.dumps(read_arm_state(arm), ensure_ascii=False))
    elif args.rgb:
        test_rgb(arm)
    elif args.buzzer:
        test_buzzer(arm)
    elif args.motion:
        test_motion(arm, step=args.step)
    elif args.center_unreadable:
        center_unreadable_servos(arm)


if __name__ == "__main__":
    main()
