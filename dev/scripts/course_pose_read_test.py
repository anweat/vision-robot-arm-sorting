#!/usr/bin/env python3
"""Run the course read task around its documented all-90-degree pose."""

import json
import time

from Arm_Lib import Arm_Device


LIMITS = {
    1: (0, 180),
    2: (0, 180),
    3: (0, 180),
    4: (0, 180),
    5: (90, 270),
    6: (0, 180),
}


def course_read_once(arm):
    values = []
    for servo_id in range(1, 7):
        values.append(arm.Arm_serial_servo_read(servo_id))
        time.sleep(0.01)
    return values


def read_with_retries(arm, servo_id, attempts=10):
    samples = []
    for _ in range(attempts):
        value = arm.Arm_serial_servo_read(servo_id)
        samples.append(value)
        if value is not None:
            return value, samples
        time.sleep(0.05)
    return None, samples


def checked_write(arm, servo_id, angle, duration_ms):
    lower, upper = LIMITS[servo_id]
    if not lower <= angle <= upper:
        raise ValueError(f"servo {servo_id}: {angle} outside [{lower}, {upper}]")
    arm.Arm_serial_servo_write(servo_id, angle, duration_ms)


def main():
    arm = Arm_Device()
    time.sleep(0.1)

    exact_before = course_read_once(arm)
    baseline = {}
    baseline_samples = {}
    for servo_id in range(1, 7):
        value, samples = read_with_retries(arm, servo_id)
        baseline[servo_id] = value
        baseline_samples[servo_id] = samples

    # Course task 1/6 documented reset pose. All values satisfy the selected limits.
    arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    time.sleep(1.0)

    exact_at_90 = course_read_once(arm)
    verified_at_90 = {}
    verified_samples = {}
    for servo_id in range(1, 7):
        value, samples = read_with_retries(arm, servo_id)
        verified_at_90[servo_id] = value
        verified_samples[servo_id] = samples

    # Restore every joint whose pre-test angle was readable.
    for servo_id, angle in baseline.items():
        if angle is not None:
            checked_write(arm, servo_id, angle, 1000)
            time.sleep(1.1)

    final = {}
    for servo_id in range(1, 7):
        value, _ = read_with_retries(arm, servo_id)
        final[servo_id] = value

    print(
        json.dumps(
            {
                "course_read_before": exact_before,
                "baseline": baseline,
                "baseline_samples": baseline_samples,
                "course_read_at_90": exact_at_90,
                "verified_at_90": verified_at_90,
                "verified_samples": verified_samples,
                "final_after_restore": final,
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
