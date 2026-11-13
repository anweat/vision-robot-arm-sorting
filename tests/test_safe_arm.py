import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "adapters"))

import safe_arm  # noqa: E402

CFG = safe_arm.load_config(ROOT / "config" / "arm_limits.json")


class PlanSegmentsTest(unittest.TestCase):
    def test_no_joint_exceeds_step(self):
        current = [90, 90, 90, 90, 92, 11]
        target = [110, 80, 90, 90, 100, 60]
        segments = safe_arm.plan_segments(current, target, 5)
        self.assertEqual(segments[-1], target)
        previous = current
        for pose in segments:
            self.assertTrue(all(abs(a - b) <= 5 for a, b in zip(pose, previous)))
            previous = pose

    def test_the_first_step_out_of_a_joint_reading_past_180_stays_inside_what_the_servo_call_accepts(self):
        """The folded arm reads [91, 183, -5, -1, 100, -3]; the plan to the observe pose used to start with J2 = 181,
        which Arm_serial_servo_write6 refuses (2026-11-06, real arm)."""
        segments = safe_arm.plan_segments([91, 183, -5, -1, 100, -3], [90, 105, 5, 5, 100, -3], 2)
        self.assertEqual(segments[-1], [90, 105, 5, 5, 100, -3])
        self.assertTrue(all(pose[1] <= 180 for pose in segments))
        self.assertEqual(segments[0][1], 180)
        self.assertTrue(all(abs(a - b) <= 2 for a, b in zip(segments[1], segments[0])))

    def test_no_motion_gives_no_segments(self):
        self.assertEqual(safe_arm.plan_segments([90] * 6, [90] * 6, 5), [])


class ValidateTargetTest(unittest.TestCase):
    def test_holding_a_joint_outside_soft_limit_is_allowed(self):
        # servo 5 read 92 on 2026-09-24, below the 95 soft limit but inside the course limit
        safe_arm.validate_target([90, 90, 90, 90, 92, 11], [95, 90, 90, 90, 92, 11], CFG)

    def test_moving_outside_soft_limit_is_rejected(self):
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.validate_target([90, 90, 90, 90, 100, 11], [90, 90, 90, 90, 91, 11], CFG)

    def test_course_limit_always_applies(self):
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.validate_target([90] * 6, [200, 90, 90, 90, 90, 90], CFG)
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.validate_target([90, 90, 90, 90, 90, 90], [90, 90, 90, 90, 0, 90], CFG)


class HoldOutsideLimitTest(unittest.TestCase):
    def test_joint_pushed_past_its_end_may_hold_while_others_move(self):
        safe_arm.validate_target([93, 95, -2, 15, 95, 0], [93, 95, -2, 25, 95, 0], CFG)

    def test_it_may_only_move_back_towards_its_range(self):
        # 2026-09-24: with 2 deg jogs a joint at -4 could never reach 5 in one step, so stepping
        # back is allowed as long as every step ends closer to the range; moving further out is not
        safe_arm.validate_target([93, 95, -2, 25, 95, 0], [93, 95, 5, 25, 95, 0], CFG)
        safe_arm.validate_target([93, 95, -2, 25, 95, 0], [93, 95, -1, 25, 95, 0], CFG)
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.validate_target([93, 95, -2, 25, 95, 0], [93, 95, -3, 25, 95, 0], CFG)


class StepBackIntoLimitsTest(unittest.TestCase):
    def test_joint_past_its_limit_may_step_back_but_not_further_out(self):
        current = [90, 108, -4, 3, 95, 133]
        safe_arm.validate_target(current, [90, 108, -2, 3, 95, 133], CFG)   # closer to the range
        safe_arm.validate_target(current, [90, 108, -4, 5, 95, 133], CFG)   # J4 back to its soft limit
        for bad in ([90, 108, -6, 3, 95, 133], [90, 108, -4, 1, 95, 133]):  # further out
            with self.assertRaises(safe_arm.MotionError):
                safe_arm.validate_target(current, bad, CFG)

    def test_joint_inside_limits_cannot_leave_them(self):
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.validate_target([90, 105, 5, 5, 95, 11], [90, 105, 3, 5, 95, 11], CFG)


class KeepOutTest(unittest.TestCase):
    """Refitted on 2026-09-25: moves that ran safely must pass, every platform contact must fail."""

    SAFE = [([90, 105, 5, 5], [90, 105, 5, 5]), ([70, 105, 5, 5], [110, 105, 5, 5]),
            ([84, 108, 5, 8], [96, 102, 8, 5]), ([93, 133, 2, 3], [90, 105, 5, 5]),
            ([90, 105, 5, 5], [90, 130, 5, 5]),
            # near-strip refine views of 2026-09-25
            ([71, 94, 5, 5], [71, 94, 5, 5]), ([51, 99, 5, 5], [51, 99, 5, 5])]
    # "close to the platform, no contact" (2026-09-25 00:50); the fingertips were near the table, so
    # only the platform term applies
    PLATFORM_CLEAR = [[99, 72, 13, 12], [93, 81, 23, -2]]
    CONTACT = [([90, 85, 5, 5], [90, 85, 5, 5]), ([76, 86, 7, 7], [90, 105, 5, 5]),
               ([93.4, 95, -2, 14.6], [93.4, 95, -2, 14.6]),
               ([103, 72, 19, 3], [103, 72, 19, 3]),          # console contact 2026-09-25 00:50
               ([90, 102, -5, -1], [90, 102, -5, -1])]        # folded, servo on the platform, 19:00

    def test_known_safe_moves_pass(self):
        for start, goal in self.SAFE:
            self.assertGreaterEqual(safe_arm.check_keep_out(start, goal, CFG), 0, (start, goal))

    def test_close_but_clear_of_the_platform(self):
        ko = safe_arm.workspace.load_keep_out(CFG)
        for pose in self.PLATFORM_CLEAR:
            self.assertGreaterEqual(safe_arm.workspace.platform_clearance(pose, ko), 0, pose)

    def test_known_contacts_are_inside(self):
        ko = safe_arm.workspace.load_keep_out(CFG)
        for start, goal in self.CONTACT:
            self.assertLess(safe_arm.workspace.path_clearance(start, goal, ko), 0, (start, goal))

    def test_leaving_the_contact_that_happened_is_refused(self):
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.check_keep_out([76, 86, 7, 7], [90, 105, 5, 5], CFG)

    def test_recovery_needs_the_flag_and_must_not_get_worse(self):
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.check_keep_out([93, 95, -2, 15], [93, 95, -2, 25], CFG)
        safe_arm.check_keep_out([93, 95, -2, 15], [93, 95, -2, 25], CFG, recover=True)
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.check_keep_out([90, 105, 5, 5], [90, 85, 5, 5], CFG, recover=True)


class ResolveTargetTest(unittest.TestCase):
    def test_pose_null_keeps_current_and_overrides_apply(self):
        current = [86, 90, 91, 90, 92, 11]
        target = safe_arm.resolve_target(current, [90, 90, 90, 90, None, None], {6: 60}, {1: -4})
        self.assertEqual(target, [82, 90, 90, 90, 92, 60])

    def test_assignment_parsing(self):
        self.assertEqual(safe_arm.parse_assignments(["2=85"], False), {2: 85})
        self.assertEqual(safe_arm.parse_assignments(["1=-5"], True), {1: -5})
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.parse_assignments(["1=-5"], False)
        with self.assertRaises(safe_arm.MotionError):
            safe_arm.parse_assignments(["7=10"], False)


class CheckReachedTest(unittest.TestCase):
    def test_reports_failed_reads_and_deviation_but_honours_skip(self):
        problems = safe_arm.check_reached([90, None, 80, 90, 90, 40], [90, 90, 90, 90, 90, 90], 6, [6])
        self.assertEqual(problems, ["servo 2: readback failed", "servo 3: expected 90, read 80"])


class FakeArm:
    def __init__(self, pose):
        self.pose = list(pose)
        self.writes = []

    def read(self):
        return list(self.pose)

    def write6(self, pose, duration_ms):
        assert len(pose) == 6
        self.writes.append(list(pose))
        self.pose = list(pose)


class BlockedArm(FakeArm):
    """Joint 2 cannot get past 100: the readback stays there, like hitting an obstacle."""

    def write6(self, pose, duration_ms):
        super().write6(pose, duration_ms)
        self.pose[1] = min(self.pose[1], 100)


class SafeArmTest(unittest.TestCase):
    def make(self, execute, pose=(90, 104, 5, 6, 95, 11)):
        import tempfile
        fake = FakeArm(pose)
        return safe_arm.SafeArm(execute=execute, log_dir=Path(tempfile.mkdtemp()), arm=fake), fake

    def test_short_target_keeps_the_gripper(self):
        arm, fake = self.make(execute=False)
        self.assertEqual(arm.move([70, 105, 5, 5, 95]), [70, 105, 5, 5, 95, 11])
        self.assertEqual(fake.writes, [])

    def test_execute_writes_six_values_in_small_steps(self):
        original = safe_arm.time.sleep
        safe_arm.time.sleep = lambda s: None
        try:
            arm, fake = self.make(execute=True)
            self.assertEqual(arm.move([70, 105, 5, 5, 95]), [70, 105, 5, 5, 95, 11])
            arm.close()
        finally:
            safe_arm.time.sleep = original
        self.assertTrue(all(len(p) == 6 for p in fake.writes))
        self.assertEqual(len(fake.writes), 4)

    def test_resistance_retreats_to_last_verified_pose(self):
        import tempfile
        original = safe_arm.time.sleep
        safe_arm.time.sleep = lambda s: None
        try:
            fake = BlockedArm((90, 90, 90, 90, 95, 11))
            arm = safe_arm.SafeArm(execute=True, log_dir=Path(tempfile.mkdtemp()), arm=fake)
            with self.assertRaises(safe_arm.MotionError):
                arm.move([90, 115, 90, 90, 95])
            arm.close()
        finally:
            safe_arm.time.sleep = original
        self.assertEqual(fake.writes[-1], [90, 100, 90, 90, 95, 11])  # sent back to the last verified pose
        self.assertEqual(fake.writes[-2], [90, 105, 90, 90, 95, 11])  # the command that met resistance

    def test_too_many_values_rejected(self):
        arm, _ = self.make(execute=False)
        with self.assertRaises(safe_arm.MotionError):
            arm.move([90] * 7)


if __name__ == "__main__":
    unittest.main()
