import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
import console_grasp as G  # noqa: E402
import run as R  # noqa: E402

BINS = {'recyclable': '40', 'kitchen': '41', 'hazardous': '43', 'other': '44'}


def at(marker):
    return [round(v * 100, 1) for v in G.marker_centre(marker)]


def job(category, map_cm, skip=None):
    cube = dict(map_cm=list(map_cm), yaw_deg=0.0, category=category, source='test')
    return dict(cube=cube, bin=BINS.get(category), skip=skip)


class ExecuteTest(unittest.TestCase):
    """The order blocks are carried in, and what happens when a bin is not free."""

    def setUp(self):
        self.moves, self.missed_at, self.current, self.put_fails = [], set(), None, set()
        saved = (G.hover, G.grasp, G.put, G.log, G.observe, G.PLAN, R.plan_for, G.place,
                 G.release_safely, G.back_off)

        def restore():
            (G.hover, G.grasp, G.put, G.log, G.observe, G.PLAN, R.plan_for, G.place,
             G.release_safely, G.back_off) = saved

        self.addCleanup(restore)
        G.log = lambda *a, **k: None
        G.observe = lambda sess=None: None
        G.PLAN = ROOT / 'data' / 'runs' / 'console_grasp' / 'plan_test.json'
        G.PLAN.parent.mkdir(parents=True, exist_ok=True)
        self.addCleanup(lambda: G.PLAN.unlink(missing_ok=True))
        R.plan_for = lambda cube, prefer=150: dict(plan='x')

        def grasp(sess):
            if self.current is not None and tuple(self.current) in self.missed_at:
                raise G.GripMissed('空夹')

        def hover(sess):
            if self.current is not None and tuple(self.current) in self.hover_fails:
                raise G.ConsoleError('遇到阻力或偏差，已停在当前读回位置：servo 3: expected 7, read 12')

        def put(where, sess):
            self.moves.append((self.current and tuple(self.current), where))
            if where in self.put_fails:
                raise G.ConsoleError(f'no hover over {where}')

        G.hover, G.grasp, G.put = hover, grasp, put
        G.place = lambda sess=None: self.moves.append(('place', None))
        self.backed_off = 0
        G.back_off = lambda sess=None: setattr(self, 'backed_off', self.backed_off + 1)
        self.hover_fails = set()
        G.release_safely = lambda sess=None: (self.moves.append(('place', None)) or '放回抓取位置')

    def run_jobs(self, jobs):
        # `current` follows whichever block the code is carrying: plan_for is called with its cube first
        real_plan_for = R.plan_for

        def plan_for(cube, prefer=150):
            self.current = cube['map_cm']
            return real_plan_for(cube, prefer)

        R.plan_for = plan_for
        return R.execute('run', dict(cubes=[j['cube'] for j in jobs], registration={}), jobs,
                         None, rng=random.Random(5))

    def test_blocks_with_free_bins_are_sorted_in_order(self):
        counts = self.run_jobs([job('hazardous', [5, 8]), job('kitchen', [-5, 8])])
        self.assertEqual([m[1] for m in self.moves], ['43', '41'])
        self.assertEqual((counts['sorted'], counts['cleared'], counts['skipped']), (2, 0, 0))

    def test_a_block_waits_for_the_one_standing_on_its_bin(self):
        """The kitchen block sits on the hazardous bin: move it home first, then the hazardous block
        follows -- no clearing move needed."""
        counts = self.run_jobs([job('hazardous', [5, 8]), job('kitchen', at('43'))])
        self.assertEqual([m[1] for m in self.moves], ['41', '43'])
        self.assertEqual((counts['sorted'], counts['cleared']), (2, 0))

    def test_two_blocks_on_each_others_bins_are_broken_apart(self):
        """Neither can go home while the other is there, so one is lifted onto a free marker first."""
        counts = self.run_jobs([job('hazardous', at('44')), job('other', at('43'))])
        self.assertEqual(counts, dict(sorted=2, skipped=0, missed=0, cleared=1, seconds=counts['seconds']))
        self.assertEqual([m[1] for m in self.moves][-2:], ['43', '44'])
        self.assertNotIn(self.moves[0][1], ('43', '44'))          # the first move is out of the way

    def test_a_missed_grip_is_counted_and_the_round_goes_on(self):
        self.missed_at = {tuple(at('30'))}
        counts = self.run_jobs([job('hazardous', at('30')), job('kitchen', [-5, 8])])
        self.assertEqual((counts['sorted'], counts['missed']), (1, 1))
        self.assertEqual([m[1] for m in self.moves], ['41'])

    def test_skipped_blocks_are_counted_and_never_carried(self):
        counts = self.run_jobs([job(None, [5, 8], skip='no category'), job('kitchen', [-5, 8])])
        self.assertEqual((counts['sorted'], counts['skipped']), (1, 1))
        self.assertEqual([m[1] for m in self.moves], ['41'])

    def test_an_unreachable_block_is_skipped_not_attempted(self):
        jobs = [job('hazardous', [5, 8]), job('kitchen', [-5, 8])]
        unreachable = jobs[0]['cube']['map_cm']
        base = R.plan_for
        R.plan_for = lambda cube, prefer=150: None if cube['map_cm'] == unreachable else base(cube, prefer)
        counts = self.run_jobs(jobs)
        self.assertEqual((counts['sorted'], counts['skipped']), (1, 1))
        self.assertEqual([m[1] for m in self.moves], ['41'])


    def test_a_block_that_cannot_be_placed_is_put_back_not_carried_on(self):
        """2026-09-27: a failed put left the arm holding the block, and the next hover opened the
        fingers and dropped it."""
        self.put_fails = {'43'}
        counts = self.run_jobs([job('hazardous', [5, 8]), job('kitchen', [-5, 8])])
        self.assertEqual([m[1] for m in self.moves], ['43', None, '41'])
        self.assertEqual((counts['sorted'], counts['skipped']), (1, 1))

    def test_failing_to_put_it_back_stops_the_round(self):
        import scramble as SCR

        def boom(sess=None):
            raise G.ConsoleError('路径被拒')

        self.put_fails = {'43'}
        G.release_safely = boom
        with self.assertRaises(SCR.StillHolding):
            self.run_jobs([job('hazardous', [5, 8]), job('kitchen', [-5, 8])])


class ApproachFailureTest(ExecuteTest):
    """2026-09-29: resistance during an approach was not caught, so one block ending badly ended the
    whole round -- and left the arm just inside the platform envelope for whatever ran next."""

    def test_the_round_goes_on_after_an_approach_is_stopped(self):
        self.hover_fails = {tuple(at('30'))}
        counts = self.run_jobs([job('hazardous', at('30')), job('kitchen', [-5, 8])])
        self.assertEqual((counts['sorted'], counts['skipped']), (1, 1))
        self.assertEqual([m[1] for m in self.moves], ['41'])

    def test_the_arm_retreats_before_the_next_block(self):
        self.hover_fails = {tuple(at('30'))}
        self.run_jobs([job('hazardous', at('30')), job('kitchen', [-5, 8])])
        self.assertEqual(self.backed_off, 1)


class RegistrationTest(unittest.TestCase):
    """The scan says where the map is; the fingertips say how high the table is. 2026-09-29: taking the
    scan's z made every descent aim 0.8 cm high."""

    def setUp(self):
        saved = G.DX, G.DY, G.TABLE_Z, G.log
        self.logged = []
        G.log = lambda event, **k: self.logged.append((event, k))
        self.addCleanup(lambda: setattr_all(saved))

        def setattr_all(v):
            G.DX, G.DY, G.TABLE_Z, G.log = v

    def test_x_and_y_are_taken_from_the_scan(self):
        R.use_registration(dict(map_origin_base_cm=[-1.27, 7.81, -0.62]))
        self.assertAlmostEqual(G.DX * 100, -1.27, places=6)
        self.assertAlmostEqual(G.DY * 100, 7.81, places=6)

    def test_the_calibrated_table_height_survives_a_scan(self):
        before = G.TABLE_Z
        R.use_registration(dict(map_origin_base_cm=[-1.27, 7.81, -0.62]))
        self.assertEqual(G.TABLE_Z, before)

    def test_a_big_disagreement_is_reported(self):
        R.use_registration(dict(map_origin_base_cm=[0.0, 7.0, 5.0]))
        self.assertEqual([e for e, _ in self.logged], ['table_z_disagrees'])

    def test_a_small_disagreement_is_not_noise_in_the_log(self):
        R.use_registration(dict(map_origin_base_cm=[0.0, 7.0, -0.62]))
        self.assertEqual(self.logged, [])


if __name__ == '__main__':
    unittest.main()
