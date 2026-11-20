#!/usr/bin/env python3
"""Verify the course Arm_serial_servo_write6 path with a small safe delta."""

import json
import time

from Arm_Lib import Arm_Device


LIMITS = [(0, 180), (0, 180), (0, 180), (0, 180), (90, 270), (0, 180)]


def read_all(arm):
    values = []
    for servo_id in range(1, 7):
        value = None
        for _ in range(10):
            value = arm.Arm_serial_servo_read(servo_id)
            if value is not None:
                break
            time.sleep(0.05)
        values.append(value)
        time.sleep(0.02)
    return values


def validate_pose(pose):
    if len(pose) != 6:
        raise ValueError("pose must have six values")
    for servo_id, (angle, (lower, upper)) in enumerate(zip(pose, LIMITS), start=1):
        if angle is None or not lower <= angle <= upper:
            raise ValueError(f"servo {servo_id}: {angle} outside [{lower}, {upper}]")


def main():
    arm = Arm_Device()
    time.sleep(0.1)
    baseline = read_all(arm)
    validate_pose(baseline)

    target = []
    for angle, (lower, upper) in zip(baseline, LIMITS):
        target.append(angle + 5 if angle + 5 <= upper - 2 else angle - 5)
    validate_pose(target)

    arm.Arm_serial_servo_write6(*target, 1000)
    time.sleep(1.3)
    observed = read_all(arm)

    arm.Arm_serial_servo_write6(*baseline, 1000)
    time.sleep(1.3)
    restored = read_all(arm)

    per_servo = []
    for servo_id, (start, expected, actual, final) in enumerate(
        zip(baseline, target, observed, restored), start=1
    ):
        reached = actual is not None and abs(actual - expected) <= 3
        returned = final is not None and abs(final - start) <= 3
        per_servo.append(
            {
                "servo": servo_id,
                "status": "PASS" if reached and returned else "FAIL",
                "start": start,
                "target": expected,
                "observed": actual,
                "restored": final,
                "course_limit": LIMITS[servo_id - 1],
            }
        )

    print(json.dumps({"COURSE_GROUP_MOTION_TEST": per_servo}, ensure_ascii=False))


if __name__ == "__main__":
    main()
