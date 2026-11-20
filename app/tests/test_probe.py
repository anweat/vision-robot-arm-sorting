import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
import console_grasp as G  # noqa: E402


class ProbeDownTest(unittest.TestCase):
    """The console accepts a blocked step of 1-2 deg silently, so contact is read from the height that
    comes back, not from a refusal (the lesson of the gripper that accepted steps while reading 29)."""

    class Sess:
        """A fake arm whose tip stops descending once it reaches `blocked_at` (metres, FK z)."""

        def __init__(self, blocked_at=None, refuse_at=None):
            self.z = G.TABLE_Z + 0.045
            self.blocked_at, self.refuse_at, self.steps = blocked_at, refuse_at, 0

        def cart(self, axis, amount_cm):
            self.steps += 1
            if abs(amount_cm) not in (0.5, 1, 2, 3):
                raise G.ConsoleError(f'步长只允许 (0.5, 1, 2, 3)')   # the console's own whitelist
            if self.refuse_at is not None and self.steps >= self.refuse_at:
                raise G.ConsoleError('遇到阻力或偏差，已停在当前读回位置：servo 3: expected 20, read 24')
            want = self.z + amount_cm / 100
            self.z = want if self.blocked_at is None else max(want, self.blocked_at)

        def rearm(self):
            pass

    def setUp(self):
        self.saved = G.status, G.tip, G.log, G.time.sleep
        G.log = lambda *a, **k: None
        G.time.sleep = lambda s: None
        self.addCleanup(self.restore)

    def restore(self):
        G.status, G.tip, G.log, G.time.sleep = self.saved

    def run_probe(self, sess, to_above_cm=0.4):
        G.status = lambda: {'pose': [90, 90, 5, 5, 92, 170]}
        G.tip = lambda pose: [0.0, 0.2, sess.z]
        return G.probe_down(sess, 0.0, 0.2, 150, to_z=G.TABLE_Z + to_above_cm / 100)

    def test_a_block_top_is_found_at_its_height(self):
        sess = self.Sess(blocked_at=G.TABLE_Z + 0.03)          # a 3 cm block
        got = self.run_probe(sess)
        self.assertTrue(got['contacted'])
        self.assertEqual(got['why'], 'readback stalled')
        self.assertAlmostEqual(got['above_table_cm'], 3.0, delta=0.4)

    def test_nothing_there_stops_at_the_floor_without_going_below_it(self):
        sess = self.Sess(blocked_at=None)
        got = self.run_probe(sess, to_above_cm=0.4)
        self.assertFalse(got['contacted'])
        # the floor keeps the fingers clear of the table; a clamped command with a full step sent would
        # have put them under it
        self.assertGreaterEqual(round((sess.z - G.TABLE_Z) * 100, 2), got['floor_cm'])
        self.assertLessEqual(got['above_table_cm'] - got['floor_cm'], got['resolution_cm'] + 0.01)

    def test_a_resistance_refusal_counts_as_contact(self):
        got = self.run_probe(self.Sess(refuse_at=3))
        self.assertTrue(got['contacted'])
        self.assertIn('resistance', got['why'])

    def test_a_refusal_that_is_not_about_resistance_is_raised_not_called_contact(self):
        """2026-09-29: the console rejected a 0.3 cm step for its size, the probe read that as contact,
        and four blocks got a top-face height without the arm descending a millimetre."""
        sess = self.Sess()
        sess.cart = lambda axis, amount: (_ for _ in ()).throw(G.ConsoleError('步长只允许 (0.5, 1, 2, 3)'))
        G.status = lambda: {'pose': [90, 90, 5, 5, 92, 170]}
        G.tip = lambda pose: [0.0, 0.2, G.TABLE_Z + 0.045]
        with self.assertRaises(G.ConsoleError):
            G.probe_down(sess, 0.0, 0.2, 150, to_z=G.TABLE_Z + 0.004)

    def test_the_step_is_one_the_console_accepts(self):
        self.assertIn(round(G.PROBE_STEP * 100, 2), (0.5, 1, 2, 3))

    def test_the_surface_is_bracketed_between_the_stall_and_the_step_above(self):
        got = self.run_probe(self.Sess(blocked_at=G.TABLE_Z + 0.03))
        self.assertGreaterEqual(got['last_free_cm'], got['above_table_cm'])
        self.assertLessEqual(got['last_free_cm'] - got['above_table_cm'], 2 * got['resolution_cm'] + 0.01)

    def test_one_late_readback_is_not_contact(self):
        """One stalled reading is a readback that had not caught up yet, not a block; the step after it
        catches up. Only two in a row mean something is there."""
        sess = self.Sess()
        G.status = lambda: {'pose': [90, 90, 5, 5, 92, 170]}
        # the reading on step 2 lags by a whole step; every other reading is honest
        G.tip = lambda pose: [0.0, 0.2, sess.z + (0.003 if sess.steps == 2 else 0.0)]
        got = G.probe_down(sess, 0.0, 0.2, 150, to_z=G.TABLE_Z + 0.004)
        self.assertFalse(got['contacted'])

    def test_a_lower_block_reads_lower(self):
        sess = self.Sess(blocked_at=G.TABLE_Z + 0.015)
        self.assertAlmostEqual(self.run_probe(sess)['above_table_cm'], 1.5, delta=0.4)


    def test_it_starts_from_the_readback_not_from_a_commanded_height(self):
        """A readback above the commanded start would otherwise read as contact on the first step."""
        sess = self.Sess(blocked_at=G.TABLE_Z + 0.03)
        sess.z = G.TABLE_Z + 0.055                     # 1 cm higher than the nominal start
        G.status = lambda: {'pose': [90, 90, 5, 5, 92, 170]}
        G.tip = lambda pose: [0.0, 0.2, sess.z]
        got = G.probe_down(sess, 0.0, 0.2, 150, to_z=G.TABLE_Z + 0.004)
        self.assertTrue(got['contacted'])
        self.assertAlmostEqual(got['above_table_cm'], 3.0, delta=0.4)


class SlipTest(unittest.TestCase):
    def test_the_slip_threshold_is_below_the_readback_tolerance(self):
        """A block leaving the fingers closes them further; the check has to notice less than the 4 deg
        the console lets a command drift by, or a slip looks like ordinary noise."""
        self.assertLess(G.SLIP_DEG, G.CONFIG['motion']['readback_tolerance_deg'])

    def test_a_holding_readback_plus_the_threshold_is_still_short_of_closed_on_nothing(self):
        g = G.CONFIG['gripper']
        self.assertLess(g['block30_contact_readback_deg'] + G.SLIP_DEG, g['empty_closes_freely_to_deg'])


if __name__ == '__main__':
    unittest.main()
