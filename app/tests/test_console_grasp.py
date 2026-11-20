import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'tasks'))
import console_grasp as G  # noqa: E402


class QualityTest(unittest.TestCase):
    def board(self, **kw):
        return dict(dict(label='blue', bbox=[200, 150, 80, 80], method='top-face', map_cm=[-6.0, 13.0]), **kw)

    def test_good_view_needs_no_second_look(self):
        self.assertEqual(G.quality(self.board(), dict(label='blue', map_cm=[-6.1, 12.8])), [])

    def test_partial_or_disagreeing_views_ask_for_a_second_look(self):
        # 2026-09-26: blue at the image border read (-3.78, 13.41) on the board; the phone had (-6.11, 12.84)
        self.assertTrue(G.quality(self.board(bbox=[5, 150, 60, 80]), None))
        self.assertTrue(G.quality(self.board(map_cm=[-3.78, 13.41]), dict(label='blue', map_cm=[-6.11, 12.84])))
        self.assertEqual(G.quality(self.board(method='edges'), None), [])          # sticker-up block, whole view
        self.assertTrue(G.quality(None, dict(label='blue', map_cm=[-6.1, 12.8])))


class WakeTest(unittest.TestCase):
    """The J6 readback, not the console's acceptance, tells that the servo woke up."""

    class Arm:
        """A servo that ignores commands until `wakes_after` jogs have been sent: the grip command does
        nothing and the readback stays where it was, which is what "not_closed" reports."""

        def __init__(self, wakes_after, refuse=(), asleep=True):
            self.j6, self.jogs, self.rearms, self.grips = 29, 0, 0, 0
            self.wakes_after, self.refuse, self.asleep = wakes_after, refuse, asleep
            self.held = False

        def jog(self, joint, delta):
            self.jogs += 1
            if self.jogs >= self.wakes_after:
                self.asleep = False
                self.j6 += delta + 2
            if self.jogs in self.refuse:
                raise G.ConsoleError('servo 6: expected 34, read 29')

        def rearm(self):
            self.rearms += 1

        def grip(self, action):
            self.grips += 1
            if not self.asleep:
                self.held, self.j6 = True, 120

        def status(self):
            grip = ({'result': 'holding', 'readback': 120} if self.held else
                    {'result': 'not_closed', 'readback': self.j6})
            return {'pose': [90, 90, 5, 5, 92, self.j6], 'grip': grip}

    def setUp(self):
        self.saved = G.status, G.log, G.time.sleep
        G.log = lambda *a, **k: None
        G.time.sleep = lambda s: None

    def tearDown(self):
        G.status, G.log, G.time.sleep = self.saved

    def test_an_awake_servo_is_gripped_without_waking_it(self):
        # the wake cost 7.3 s of every 35.5 s block (2026-09-27); an awake servo must not pay it
        arm = self.Arm(wakes_after=1, asleep=False)
        G.status = arm.status
        self.assertEqual(G.wake_and_grip(arm)['result'], 'holding')
        self.assertEqual((arm.jogs, arm.grips), (0, 1))

    def test_an_ignored_grip_is_followed_by_waking_and_gripping_again(self):
        # 2026-09-25 19:30: two +2 steps were accepted while J6 still read 29; the grip was then ignored
        arm = self.Arm(wakes_after=3, refuse=(2,))
        G.status = arm.status
        self.assertEqual(G.wake_and_grip(arm)['result'], 'holding')
        self.assertEqual((arm.jogs, arm.rearms, arm.grips), (3, 1, 2))

    def test_a_servo_that_never_wakes_stops_after_its_tries(self):
        arm = self.Arm(wakes_after=99)
        G.status = arm.status
        self.assertEqual(G.wake_and_grip(arm, tries=2)['result'], 'not_closed')
        self.assertEqual(arm.grips, 2)

    def test_a_servo_that_never_wakes_is_reported(self):
        arm = self.Arm(wakes_after=99)
        G.status = arm.status
        self.assertFalse(G.wake_gripper(arm, steps=4))
        self.assertEqual(arm.jogs, 4)


class EmptyGripperTest(unittest.TestCase):
    """2026-09-27: a run scanned all eight views with a block in the fingers and reported three blocks
    on the table; the first hover of any sort would then have dropped the fourth."""

    def setUp(self):
        self.saved = G.status
        self.addCleanup(lambda: setattr(G, 'status', self.saved))

    def at(self, readback):
        G.status = lambda: {'pose': [90, 105, 5, 5, 92, readback]}

    def test_a_run_is_refused_while_a_block_is_held(self):
        self.at(127)
        self.assertTrue(G.holding_now())
        with self.assertRaises(G.ConsoleError):
            G.require_empty_gripper()

    def test_an_open_gripper_starts_normally(self):
        self.at(29)
        self.assertFalse(G.holding_now())
        G.require_empty_gripper()

    def test_fingers_closed_on_nothing_are_not_holding(self):
        self.at(133)                       # past the block width: closed on air
        self.assertFalse(G.holding_now())


class ReleaseSafelyTest(unittest.TestCase):
    """2026-09-27: a place descent met resistance, the console disarmed, and the recovery was refused
    for a stale readback -- the run stopped with the block still in the fingers."""

    class Sess:
        def __init__(self, rearm_fails=False):
            self.calls, self.rearm_fails = [], rearm_fails

        def rearm(self):
            self.calls.append('rearm')
            if self.rearm_fails:
                raise G.ConsoleError('姿态模式需要探索档或标准档、稳定且新鲜的回读')

        def grip(self, action):
            self.calls.append(('grip', action))

    def setUp(self):
        self.saved = G.status, G.log, G.place, G.tip
        G.log = lambda *a, **k: None
        G.place = lambda sess=None: self.calls_place.append(sess)
        self.calls_place = []
        self.addCleanup(self.restore)

    def restore(self):
        G.status, G.log, G.place, G.tip = self.saved

    def at_height(self, z_cm):
        G.status = lambda: {'pose': [90, 90, 5, 5, 92, 129]}
        G.tip = lambda pose: [0.0, 0.2, z_cm / 100]

    def test_it_re_arms_before_anything_else(self):
        self.at_height(z_cm=0.5)
        sess = self.Sess()
        G.release_safely(sess)
        self.assertEqual(sess.calls[0], 'rearm')

    def test_a_block_already_down_among_the_others_is_let_go_there(self):
        self.at_height(z_cm=1.0)                  # table is at about -0.5 cm, a 3 cm block
        sess = self.Sess()
        self.assertEqual(G.release_safely(sess), '在原地松手')
        self.assertIn(('grip', 'open'), sess.calls)
        self.assertEqual(self.calls_place, [])    # not carried back across the map

    def test_a_block_held_high_is_carried_back_to_where_it_was_picked_up(self):
        self.at_height(z_cm=12.0)
        self.assertEqual(G.release_safely(self.Sess()), '放回抓取位置')
        self.assertEqual(len(self.calls_place), 1)

    def test_a_refused_re_arm_is_raised_so_the_caller_stops(self):
        self.at_height(z_cm=1.0)
        with self.assertRaises(G.ConsoleError):
            G.release_safely(self.Sess(rearm_fails=True))


class PlatformTest(unittest.TestCase):
    def setUp(self):
        self.saved = G.DX, G.DY, G.TABLE_Z

    def tearDown(self):
        G.DX, G.DY, G.TABLE_Z = self.saved

    def test_near_strip_block_plans_clear_of_the_platform(self):
        # the green block of 2026-09-25 19:15 at map (-6.3, 8.1): the 10.5 cm cylinder blocked it
        G.DX, G.DY, G.TABLE_Z = -0.008, 0.0755, -0.0064
        a = G.plan_approach(*G.map_to_base(-0.063, 0.081), 0)
        self.assertIsNotNone(a)
        self.assertTrue(G.clear_of_platform(a['hover']))

    def test_folding_into_the_platform_is_refused(self):
        with self.assertRaises(G.ConsoleError):
            G.check_path([90, 105, 5, 5], [90, 102, -5, -1])     # the folded contact pose
        G.check_path([90, 103, -5, -1], [90, 105, 5, 5])         # leaving it is allowed


class PathTest(unittest.TestCase):
    def test_observe_to_hover_is_allowed(self):
        G.check_path([90, 104, 5, 5], [74, 56, 53, 11])

    def test_rising_from_a_low_grip_pose_is_allowed(self):
        G.check_path([107, 60, 26, 30], [108, 59, 49, 12])   # grip pose (tip ~0) back up to its hover

    def test_rising_from_a_sagged_pose_is_allowed(self):
        G.check_path([88, 65, -3, 60], [88, 66, 20, 38])   # after a servo power cycle the tip rested at -2.1 cm

    def test_path_through_the_table_is_refused(self):
        with self.assertRaises(G.ConsoleError):
            G.check_path([90, 104, 5, 5], [90, 10, 94, 8])   # ends at tip z -3.8 cm, below the table


if __name__ == '__main__':
    unittest.main()


class SortDecisionTest(unittest.TestCase):
    def setUp(self):
        import sort_blocks
        self.S = sort_blocks

    def at(self, marker, dx=0.0, dy=0.0):
        x, y = G.marker_centre(marker)
        return {'map_cm': [x * 100 + dx, y * 100 + dy], 'source': 'phone'}

    def test_decisions(self):
        bins = {c: self.S.SORTING['category_bins'][k] for c, k in self.S.SORTING['colour_category'].items()}
        found = {'green': self.at(bins['green'], 0.5), 'blue': self.at(bins['yellow']), 'red': self.at('42'),
                 'yellow': {'map_cm': [-13.0, 21.5]}}                                    # ~29.6 cm from the base
        self.assertEqual(self.S.decide('green', found)[0], 'skip')          # already on its bin
        self.assertEqual(self.S.decide('blue', found)[0], 'move')           # sitting on yellow's bin
        self.assertEqual(self.S.decide('red', found)[0], 'move')
        self.assertIn('move it closer', self.S.decide('yellow', found)[1])  # ~27 cm from the base
        found['green'] = self.at(bins['red'])
        self.assertIn('occupied', self.S.decide('red', found)[1])
        self.assertEqual(self.S.decide('purple', found)[1], 'not seen')


class RollTest(unittest.TestCase):
    FLAT, STEEP = 150, 170

    def residual(self, j1, yaw, j5):
        """How far the fingers end up from the block's faces, in degrees (a square block, so mod 90)."""
        off = ((yaw - (j1 - 90)) - (j5 - G.ROLL_ZERO)) % 90
        return min(off, 90 - off)

    def test_a_quarter_turn_is_only_offered_at_a_near_vertical_pitch(self):
        # 2026-09-26: J5 167 at a flat pitch opened the fingers along the pitch plane and missed
        self.assertEqual(G.grip_roll(88, -16.6, self.FLAT), G.ROLL_ZERO)   # needs -14.6, below J5 90
        self.assertEqual(G.grip_roll(88, -16.6, self.STEEP), 167)          # a quarter turn reaches it

    def test_the_verified_grasps_are_unchanged(self):
        self.assertEqual(G.grip_roll(90, 0, self.FLAT), G.ROLL_ZERO)
        self.assertEqual(G.grip_roll(71, -10.9, self.FLAT), 100)           # verified by eye 2026-09-26

    def test_the_roll_that_flicked_a_block_away_is_now_reached(self):
        """2026-09-27: a block at yaw 21 from J1 64 wanted J5 139, two degrees outside the old window,
        so it was gripped 43 deg off its faces and flicked away."""
        self.assertEqual(G.grip_roll(64, 21, self.FLAT), 139)
        self.assertEqual(self.residual(64, 21, 139), 0)

    def test_every_orientation_lines_up_at_a_near_vertical_pitch(self):
        for j1 in range(40, 141, 7):
            for yaw in range(-45, 46, 3):
                j5 = G.grip_roll(j1, yaw, self.STEEP)
                self.assertTrue(G.J5_RANGE[0] <= j5 <= G.J5_RANGE[1])
                self.assertLessEqual(self.residual(j1, yaw, j5), G.ROLL_ALIGNED_DEG, (j1, yaw, j5))

    def test_a_flat_pitch_aligns_what_it_can_and_says_so(self):
        """Half turns alone cover half the orientations; plan_approach prefers a pitch that aligns."""
        covered = sum(bool(G.roll_options(j1, yaw, self.FLAT))
                      for j1 in range(40, 141, 7) for yaw in range(-45, 46, 3))
        total = len(range(40, 141, 7)) * len(range(-45, 46, 3))
        self.assertGreater(covered / total, 0.45)
        for j1 in range(40, 141, 7):
            for yaw in range(-45, 46, 3):
                for j5 in G.roll_options(j1, yaw, self.FLAT):
                    self.assertLessEqual(self.residual(j1, yaw, j5), G.ROLL_ALIGNED_DEG, (j1, yaw, j5))


class CategoryTest(unittest.TestCase):
    def test_model_evidence_and_colour(self):
        import sort_blocks as S
        self.assertIn('official model', S.category_of('red', {'label': 'Syringe', 'category': 'hazardous', 'conf': 0.8})[1])
        # the model has no 'other' class: a yellow block called Fish_bone stays 'other' and is logged as a disagreement
        cat, src = S.category_of('yellow', {'label': 'Fish_bone', 'category': 'kitchen', 'conf': 0.54})
        self.assertEqual((cat, src.startswith('colour; official model disagreed')), ('other', True))
        self.assertIn('fallback', S.category_of('blue', None)[1])
        self.assertIn('fallback', S.category_of('green', {'label': 'Fish_bone', 'category': 'kitchen', 'conf': 0.3})[1])
        self.assertEqual(S.bin_for('red')[0], '43')


class ResolveTest(unittest.TestCase):
    def test_evidence(self):
        import sort_blocks as S
        fish = {'label': 'Fish_bone', 'category': 'kitchen', 'conf': 0.8}
        self.assertEqual(S.resolve(None, fish)[0], 'kitchen')                        # sticker-up block: model decides
        self.assertEqual(S.resolve('yellow', dict(fish, conf=0.54))[0], 'other')     # model has no 'other'
        self.assertEqual(S.resolve('blue', None)[0], 'recyclable')
        self.assertIsNone(S.resolve(None, None)[0])
        self.assertIsNone(S.resolve(None, dict(fish, conf=0.3))[0])


class CallSignatureTest(unittest.TestCase):
    """2026-09-29: plan_put still called grip_roll with two arguments after pitch became required.
    The TypeError was not a ConsoleError, so nothing caught it: the run died between the lift and the
    place, and the arm was left holding the block."""

    def test_every_grip_roll_call_passes_a_pitch(self):
        import ast
        from pathlib import Path
        root = Path(__file__).resolve().parents[1] / 'src'
        for path in root.rglob('*.py'):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'grip_roll':
                    self.assertEqual(len(node.args), 3, f'{path.name}:{node.lineno}')

    def test_plan_put_returns_poses_for_a_reachable_target(self):
        """Exercises the call that was broken, on the geometry a carry actually uses."""
        steps = G.plan_put(145, '43', [90, 105, 5, 5, 92, 129])
        self.assertIsNotNone(steps)
        self.assertIn('hover', steps)
