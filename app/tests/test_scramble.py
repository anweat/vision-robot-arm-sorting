import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
import console_grasp as G  # noqa: E402
import scramble as S  # noqa: E402


class CandidatesTest(unittest.TestCase):
    def test_the_bins_are_not_offered_as_places_to_scatter_to(self):
        """A block started on its own bin is a block the round never sorts."""
        self.assertFalse(set(S.candidates()) & S.bin_markers())

    def test_candidates_keep_a_margin_inside_the_reach(self):
        """2026-09-27: 码 49 was drawn at 27.6 cm from the base, the block landed at 28.1, and the
        round after could not pick it up again."""
        import math
        for marker in S.candidates():
            bx, by = G.map_to_base(*G.marker_centre(marker))
            self.assertLessEqual(math.hypot(bx, by) * 100, G.MAX_REACH * 100 - S.REACH_MARGIN_CM, marker)

    def test_every_candidate_has_a_reachable_grasp(self):
        for marker in S.candidates():
            bx, by = G.map_to_base(*G.marker_centre(marker))
            self.assertIsNotNone(G.plan_approach(bx, by, None), marker)


class DrawTest(unittest.TestCase):
    def apart(self, markers):
        centres = [[v * 100 for v in G.marker_centre(m)] for m in markers]
        return min(math.dist(a, b) for i, a in enumerate(centres) for b in centres[i + 1:])

    def test_drawn_markers_are_never_neighbours(self):
        for seed in range(25):
            markers = S.draw(4, random.Random(seed))
            self.assertEqual(len(set(markers)), 4)
            self.assertGreaterEqual(self.apart(markers), S.MIN_APART_CM)

    def test_the_draw_is_repeatable_for_a_seed(self):
        self.assertEqual(S.draw(4, random.Random(3)), S.draw(4, random.Random(3)))

    def test_a_pool_that_cannot_seat_them_is_an_error_not_a_tighter_layout(self):
        with self.assertRaises(ValueError):
            S.draw(3, random.Random(0), pool=['40', '41'])        # two markers, three blocks
        with self.assertRaises(ValueError):
            S.draw(2, random.Random(0), pool=['40', '41'])        # neighbours, 6 cm apart


class FakeStages(unittest.TestCase):
    """hover / grasp / put / place replaced, so the ordering and the recovery can be tested off the arm."""

    def setUp(self):
        self.saved = (S.G.hover, S.G.grasp, S.G.put, S.G.log, S.G.PLAN, S.G.place, S.G.can_put,
                      S.G.status, S.G.release_safely)
        self.calls, self.put_fails = [], set()
        S.G.hover = lambda sess: self.calls.append('hover')
        S.G.grasp = lambda sess: self.calls.append('grasp')
        S.G.place = lambda sess=None: self.calls.append('place')
        S.G.release_safely = lambda sess=None: (self.calls.append('place') or '放回抓取位置')
        S.G.can_put = lambda pitch, marker, pose: marker not in self.unplaceable
        S.G.status = lambda: {'pose': [90, 105, 5, 5, 92, 129]}
        self.unplaceable = set()

        def put(where, sess):
            self.calls.append(('put', where))
            if where in self.put_fails:
                raise S.G.ConsoleError(f'no hover over {where}')

        S.G.put = put
        S.G.log = lambda *a, **k: None
        S.G.PLAN = ROOT / 'data' / 'runs' / 'console_grasp' / 'plan_test.json'
        S.G.PLAN.parent.mkdir(parents=True, exist_ok=True)
        self.addCleanup(lambda: S.G.PLAN.unlink(missing_ok=True))

    def tearDown(self):
        (S.G.hover, S.G.grasp, S.G.put, S.G.log, S.G.PLAN, S.G.place, S.G.can_put,
         S.G.status, S.G.release_safely) = self.saved


class MoveTest(FakeStages):
    def test_a_move_runs_the_verified_stages_in_order(self):
        cube = dict(map_cm=[5.0, 8.0], yaw_deg=0.0, category='kitchen')
        out = S.move(cube, '38', lambda c: dict(plan='x', pitch=145), None)
        self.assertEqual(self.calls, ['hover', 'grasp', ('put', '38')])
        self.assertEqual(out['marker'], '38')
        self.assertEqual(out['frm'], [5.0, 8.0])

    def test_an_unreachable_block_stops_before_any_motion(self):
        with self.assertRaises(G.ConsoleError):
            S.move(dict(map_cm=[0, 30], yaw_deg=0), '38', lambda c: None, None)
        self.assertEqual(self.calls, [])

    def test_the_farthest_block_is_moved_first(self):
        """Putting a near block down first would leave the arm reaching over it for the far one."""
        cubes = [dict(map_cm=[0.0, 8.0], yaw_deg=0), dict(map_cm=[0.0, 18.0], yaw_deg=0)]
        moved = S.scramble(cubes, lambda c: dict(plan='x', pitch=145), None, random.Random(1))
        self.assertEqual([m['frm'] for m in moved], [[0.0, 18.0], [0.0, 8.0]])


class HoldingTest(FakeStages):
    """A block in the fingers has to end up somewhere. 2026-09-27: three scatter moves gripped a block,
    could not place it, and the next block's hover opened the fingers and dropped it."""

    def plan(self, c):
        return dict(plan='x', pitch=145)

    def test_an_unplaceable_target_is_skipped_before_the_block_is_carried_to_it(self):
        self.unplaceable = {'38'}
        out = S.move(dict(map_cm=[5.0, 8.0], yaw_deg=0.0), ['38', '35'], self.plan, None)
        self.assertEqual(out['marker'], '35')
        self.assertEqual([c for c in self.calls if isinstance(c, tuple)], [('put', '35')])

    def test_a_put_that_fails_while_holding_falls_back_to_the_next_marker(self):
        self.put_fails = {'38'}
        out = S.move(dict(map_cm=[5.0, 8.0], yaw_deg=0.0), ['38', '35'], self.plan, None)
        self.assertEqual(out['marker'], '35')
        self.assertEqual([c for c in self.calls if isinstance(c, tuple)], [('put', '38'), ('put', '35')])

    def test_the_block_goes_back_where_it_came_from_when_nothing_works(self):
        self.put_fails = {'38', '35'}
        with self.assertRaises(S.G.ConsoleError) as e:
            S.move(dict(map_cm=[5.0, 8.0], yaw_deg=0.0), ['38', '35'], self.plan, None)
        self.assertNotIsInstance(e.exception, S.StillHolding)
        self.assertEqual(self.calls[-1], 'place')

    def test_failing_to_put_it_back_is_a_stop_the_session_error(self):
        self.put_fails = {'38'}

        def boom(sess=None):
            raise S.G.ConsoleError('路径被拒')

        S.G.release_safely = boom
        with self.assertRaises(S.StillHolding):
            S.move(dict(map_cm=[5.0, 8.0], yaw_deg=0.0), ['38'], self.plan, None)


class AlternativesTest(unittest.TestCase):
    def test_the_drawn_marker_comes_first_and_occupied_places_are_left_out(self):
        taken = [[v * 100 for v in G.marker_centre('35')]]
        alts = S.alternatives('38', taken)
        self.assertEqual(alts[0], '38')
        self.assertNotIn('35', alts)


if __name__ == '__main__':
    unittest.main()
