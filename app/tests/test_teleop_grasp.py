"""Grasp-oriented console features: Cartesian steps, gripper presets, grasp profile, recording."""
import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/teleop'))
from core import Controller, SimArm, safe_arm  # noqa: E402
from camera import Camera  # noqa: E402
from server import App  # noqa: E402

kin = safe_arm.workspace.kin


class GripperArm(SimArm):
    """The gripper stops at 127 as if a 3 cm block were between the fingers."""

    def write6(self, pose, duration_ms):
        super().write6(pose, duration_ms)
        self.pose[5] = min(self.pose[5], 127)


class Base(unittest.TestCase):
    arm_class = SimArm

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.arm = self.arm_class()
        self.c = Controller(self.tmp.name, self.arm)
        self.c.enable('a')
        patcher = patch('core.time.sleep')
        patcher.start()
        self.addCleanup(patcher.stop)


class DeadbandArm(SimArm):
    """J1 ignores commands less than 2 deg away from where it stands, like the board's base servo."""

    def write6(self, pose, duration_ms):
        pose = list(pose)
        if abs(pose[0] - self.pose[0]) < 2:
            pose[0] = self.pose[0]
        super().write6(pose, duration_ms)


class StepAccumulationTest(Base):
    arm_class = DeadbandArm

    def test_one_degree_jogs_accumulate_through_a_deadband(self):
        self.c.jog('a', 1, 1, self.c.revision)
        self.assertEqual(self.arm.read()[0], 90)       # 91 is inside the deadband
        self.c.jog('a', 1, 1, self.c.revision)
        self.assertEqual(self.arm.read()[0], 92)       # the second jog sends 92, not 91 again
        self.assertEqual(self.c.status()['command'][0], 92)

    def test_readback_wins_after_the_arm_was_moved_by_hand(self):
        self.c.jog('a', 1, 2, self.c.revision)
        self.arm.pose[0] = 110                          # pushed by hand, far from the last command
        self.c.refresh()
        self.c.jog('a', 1, 1, self.c.revision)
        self.assertEqual(self.c.status()['command'][0], 111)  # built on the readback 110, not on the old command 92


class RuntimeLimitsTest(unittest.TestCase):
    """The console's one set of limits (doc 32 §1.1): what the old explore profile allowed."""
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.arm = SimArm()
        self.c = Controller(self.tmp.name, self.arm)
        self.c.enable('a')
        patcher = patch('core.time.sleep'); patcher.start(); self.addCleanup(patcher.stop)

    def test_course_limits_and_platform_not_enforced_but_reported(self):
        self.arm.pose = [90, 85, 5, 5, 95, 30]         # the near-row pose that is inside the platform keep-out
        self.c.refresh(); self.c.enable('a')
        self.c.jog('a', 3, -2, self.c.revision)         # 3 deg: outside the soft limit, inside the course limit
        self.assertEqual(self.arm.read()[2], 3)
        self.assertLess(self.c.status()['clearance_cm'], 0)  # the standard model's margin is still shown
        with self.assertRaises(safe_arm.MotionError):
            for _ in range(10):
                self.c.jog('a', 3, -2, self.c.revision)  # explore allows J3 down to -10, never further
        self.assertEqual(self.arm.read()[2], -9)

    def test_wrist_joints_may_go_slightly_negative_but_not_past_minus_ten(self):
        self.arm.pose = [90, 105, 0, 0, 95, 30]
        self.c.refresh(); self.c.enable('a')
        self.c.jog('a', 4, -2, self.c.revision)
        self.assertEqual(self.arm.read()[3], -2)
        self.arm.pose = [90, 105, 0, -10, 95, 30]
        self.c.refresh(); self.c.enable('a')
        with self.assertRaises(safe_arm.MotionError):
            self.c.jog('a', 4, -1, self.c.revision)
        self.arm.pose = [0, 105, 0, 0, 95, 30]
        self.c.refresh(); self.c.enable('a')
        with self.assertRaises(safe_arm.MotionError):  # J1 keeps its cable limit
            self.c.jog('a', 1, -1, self.c.revision)

    def test_unreachable_cartesian_step_keeps_control(self):
        with self.assertRaises(safe_arm.MotionError):
            for _ in range(40):
                self.c.cartesian('a', 'y', 1, self.c.revision)
        self.assertTrue(self.c.status()['armed'])
        self.assertEqual(self.c.status()['error'], '')

    def test_large_jog_runs_as_verified_segments(self):
        start = self.arm.read()[0]
        step = self.c.cfg['motion']['max_step_deg']
        with patch.object(self.arm, 'write6', wraps=self.arm.write6) as write:
            self.c.jog('a', 1, 20)
        writes = [c.args[0][0] for c in write.call_args_list]
        self.assertEqual(writes[-1], start + 20)
        self.assertEqual(len(writes), math.ceil(20 / step))
        self.assertTrue(all(0 < b - a <= step for a, b in zip([start] + writes, writes)), writes)

    def test_home_from_inside_the_reported_keep_out_is_allowed(self):
        self.arm.pose = [90, 85, 5, 5, 95, 30]
        self.c.refresh()
        self.c.home('a')
        self.assertEqual(self.arm.read()[:4], [90, 105, 5, 5])


class CartesianTest(Base):
    def setUp(self):
        super().setUp()
        self.arm.pose = [90, 110, 30, 30, 95, 30]  # elbow bent: clear of joint limits and of full reach
        self.c.refresh()
        self.c.enable('a')

    def test_tip_moves_about_one_centimetre_along_the_axis(self):
        before = kin.forward(self.arm.read()[:4])['tip']
        self.c.cartesian('a', 'y', 1, self.c.revision)
        after = kin.forward(self.arm.read()[:4])['tip']
        self.assertAlmostEqual(after[1] - before[1], 0.01, delta=0.004)
        self.assertAlmostEqual(after[2], before[2], delta=0.004)
        self.assertEqual(self.arm.read()[4:], [95, 30])  # wrist and gripper untouched
        self.assertEqual(self.c.status()['error'], '')

    def test_pitch_step_keeps_the_tip(self):
        before = kin.forward(self.arm.read()[:4])
        self.c.cartesian('a', 'pitch', 2, self.c.revision)
        after = kin.forward(self.arm.read()[:4])
        self.assertAlmostEqual(after['tip_pitch_deg'][0] - before['tip_pitch_deg'][0], 2, delta=1)
        self.assertLess(sum((a - b) ** 2 for a, b in zip(after['tip'], before['tip'])) ** .5, 0.005)

    def test_bad_steps_rejected(self):
        for axis, amount in (('y', 6), ('q', 1), ('z', True), ('pitch', 16), ('y', 0)):
            with self.assertRaises(ValueError):
                self.c.cartesian('a', axis, amount, self.c.revision)

    def test_unreachable_target_is_refused(self):
        with self.assertRaises(safe_arm.MotionError):  # refused before anything is sent
            for _ in range(40):
                self.c.cartesian('a', 'y', 2, self.c.revision)


class WholePlanCheckTest(Base):
    def test_rejected_segment_sends_nothing(self):
        # J3/J4 already at their lower limit (-10), so moving the tip down while keeping the pitch fails
        self.arm.pose = [90, 90, -10, -10, 95, 30]
        self.c.refresh(); self.c.enable('a')
        commands = []
        original = self.arm.write6
        self.arm.write6 = lambda pose, ms: (commands.append(list(pose)), original(pose, ms))
        with self.assertRaises(safe_arm.MotionError) as ctx:
            self.c.cartesian('a', 'z', -1, self.c.revision)
        self.assertIn('未发送任何指令', str(ctx.exception))
        self.assertEqual(commands, [])
        self.assertEqual(self.arm.read(), [90, 90, -10, -10, 95, 30])
        self.assertTrue(self.c.status()['armed'])  # nothing was sent, so the operator keeps control
        self.assertEqual(self.c.status()['error'], '')


class GripperTest(Base):
    def test_empty_grip_is_reported_as_not_holding(self):
        self.c.gripper('a', 'grip', self.c.revision)
        self.assertEqual(self.arm.read()[5], 133)
        self.assertFalse(self.c.status()['grip']['holding'])

    def test_open_presets(self):
        self.c.gripper('a', 'wide', self.c.revision)
        self.assertEqual(self.arm.read()[5], 11)
        self.c.gripper('a', 'open', self.c.revision)
        self.assertEqual(self.arm.read()[5], 30)


class StuckGripperArm(SimArm):
    """The gripper ignores a grip command and stays open, as three grips did on 2026-09-25."""

    def write6(self, pose, duration_ms):
        keep = self.pose[5]
        super().write6(pose, duration_ms)
        self.pose[5] = keep


class StuckGripperTest(Base):
    arm_class = StuckGripperArm

    def test_gripper_that_never_closed_is_not_holding(self):
        self.arm.pose[5] = 29
        self.c.refresh()
        self.c.gripper('a', 'grip', self.c.revision)
        grip = self.c.status()['grip']
        self.assertEqual((grip['readback'], grip['result'], grip['holding']), (29, 'not_closed', False))


class GripStateTest(unittest.TestCase):
    def test_bands_from_the_2026_09_25_session(self):
        g = safe_arm.load_config(safe_arm.DEFAULT_CONFIG)['gripper']
        self.assertEqual([safe_arm.grip_state(r, g) for r in (29, 124, 128, 129, 131, 133, None)],
                         ['not_closed', 'holding', 'holding', 'holding', 'empty', 'empty', 'unknown'])


class BlockInGripperTest(Base):
    arm_class = GripperArm

    def test_block_between_fingers_counts_as_holding_and_is_not_a_fault(self):
        self.c.gripper('a', 'grip', self.c.revision)
        status = self.c.status()
        self.assertTrue(status['grip']['holding'])
        self.assertEqual(status['error'], '')
        self.assertTrue(status['armed'])


class StepTest(Base):
    def test_any_integer_jog_up_to_thirty_degrees(self):
        self.c.jog('a', 1, 7)
        self.assertEqual(self.arm.read()[0], 97)


class EnforcedKeepOutTest(Base):
    """runtime.enforce_platform = true: a start inside may only move so the margin does not get worse
    (the old recover profile, now always on when the keep-out is enforced)."""
    STUCK = [90, 108, -4, 3, 95, 133]  # board reading 2026-09-24 23:20: gripper hanging inside the platform keep-out

    def setUp(self):
        super().setUp()
        self.c.cfg['keep_out'] = dict(self.c.base_cfg['keep_out'])
        self.arm.pose = list(self.STUCK)
        self.c.refresh()

    def test_gets_out_in_improving_steps_and_worse_is_refused(self):
        margins = [self.c.status()['clearance_cm']]
        for joint in (3, 3, 4, 3):
            self.c.jog('a', joint, 2)
            margins.append(self.c.status()['clearance_cm'])
        self.assertEqual(margins, sorted(margins))
        self.assertGreaterEqual(margins[-1], 0)
        self.arm.pose = list(self.STUCK); self.c.refresh(); self.c.command = None
        with self.assertRaises(safe_arm.MotionError):
            self.c.jog('a', 2, -2)


class RecordingTest(Base):
    def test_episode_saves_one_record_per_step(self):
        cam = Camera()
        cam.publish(b'\xff\xd8frame\xff\xd9', time.time() + 5, 'capture')  # a frame "after" the step
        app = App(self.c, cam)
        app.record(True, 'blue test', 'first try')
        self.c.cartesian('a', 'y', 1, self.c.revision)
        self.c.gripper('a', 'wide', self.c.revision)
        app.record(False, '', 'done')
        episode = next(Path(self.tmp.name).glob('episode_*'))
        steps = [json.loads(line) for line in (episode/'steps.jsonl').read_text().splitlines()]
        kinds = [s['kind'] for s in steps]  # a Cartesian step may be split into several joint segments
        self.assertEqual(kinds[-1], 'gripper')
        self.assertTrue(set(kinds[:-1]) == {'cartesian'} and kinds[:-1])
        self.assertEqual([s['index'] for s in steps], list(range(1, len(steps) + 1)))
        self.assertTrue(all((episode/s['board_image']).exists() and s['board']['after_step'] for s in steps))
        meta = json.loads((episode/'episode.json').read_text())
        self.assertEqual(meta['steps'], len(steps))
        self.assertEqual(meta['label'], 'blue test')
        with self.assertRaises(ValueError):
            app.record(False, '', '')


if __name__ == '__main__':
    unittest.main()


class PoseModeTest(unittest.TestCase):
    """Pose set on the 3D model, previewed, then executed in verified 5 deg segments."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.arm = SimArm()
        self.arm.pose = [90, 90, 40, 40, 90, 30]
        self.c = Controller(self.tmp.name, self.arm)
        self.c.refresh(); self.c.enable('a')
        patcher = patch('core.time.sleep'); patcher.start(); self.addCleanup(patcher.stop)

    def test_preview_sends_nothing_then_executes_in_segments_keeping_the_gripper(self):
        with patch.object(self.arm, 'write6', wraps=self.arm.write6) as write:
            plan = self.c.plan_pose([100, 70, 50, 30, 90, 170])
            self.assertEqual(write.call_count, 0)
            step = self.c.cfg['motion']['max_step_deg']
            self.assertEqual((plan['segments'], plan['largest_deg'], plan['target'][5]), (-(-20 // step), 20, 30))
            self.c.move_to_pose('a', plan['id'], self.c.revision)
        self.assertEqual(write.call_count, -(-20 // step))
        self.assertTrue(all(max(abs(a - b) for a, b in zip(w.args[0], prev)) <= step
                            for w, prev in zip(write.call_args_list[1:], [c.args[0] for c in write.call_args_list])))
        self.assertEqual(self.arm.read(), [100, 70, 50, 30, 90, 30])
        with self.assertRaises(ValueError):  # a preview is used once
            self.c.move_to_pose('a', plan['id'], self.c.revision)

    def test_pose_changed_after_preview_is_refused_without_locking(self):
        plan = self.c.plan_pose([100, 70, 50, 30, 90])
        self.arm.pose[1] = 80  # moved by hand after the preview
        with self.assertRaises(safe_arm.MotionError):  # PlanRejected: nothing sent, control kept
            self.c.move_to_pose('a', plan['id'], self.c.revision)
        self.assertEqual(self.arm.read()[1], 80)
        self.assertTrue(self.c.status()['armed'])

    def test_sagging_joint_between_command_and_readback_still_executes(self):
        self.c.jog('a', 2, -5, self.c.revision)      # command J2 85
        self.arm.pose[1] = 88                          # the arm sags back 3 deg (within the 4 deg tolerance)
        self.c.refresh()
        plan = self.c.plan_pose([100, 80, 50, 30, 90])  # preview starts at the readback
        self.c.move_to_pose('a', plan['id'], self.c.revision)
        self.assertEqual(self.arm.read()[:5], [100, 80, 50, 30, 90])

    def test_out_of_limit_path_is_refused_at_preview(self):
        with self.assertRaises(ValueError):
            self.c.plan_pose([90, 90, 40, 40, 300])
