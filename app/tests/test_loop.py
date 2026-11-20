import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
import console_grasp as G  # noqa: E402
import loop as L  # noqa: E402


def cube(map_cm, category, **kw):
    return dict(map_cm=list(map_cm), yaw_deg=0.0, category=category, source='test', **kw)


def sorted_table():
    return [cube(L.bin_centre(c), c) for c in ('recyclable', 'kitchen', 'hazardous', 'other')]


class BootstrapTest(unittest.TestCase):
    """Identity comes from the arrangement the operator verified, never from the detector."""

    def test_a_sorted_table_names_every_block_by_its_bin(self):
        tracked = L.bootstrap(sorted_table())
        self.assertEqual([t['category'] for t in tracked], ['recyclable', 'kitchen', 'hazardous', 'other'])

    def test_the_detectors_own_labels_are_not_used(self):
        cubes = sorted_table()
        for c in cubes:                                  # every block misread as the same category
            c['category'] = 'other'
        self.assertEqual([t['category'] for t in L.bootstrap(cubes)],
                         ['recyclable', 'kitchen', 'hazardous', 'other'])

    def test_an_unsorted_table_is_refused(self):
        cubes = sorted_table()
        cubes[0]['map_cm'] = [0.0, 3.0]                  # one block off its bin
        with self.assertRaises(SystemExit):
            L.bootstrap(cubes)

    def test_two_blocks_on_one_bin_is_refused(self):
        cubes = sorted_table()
        cubes[0]['map_cm'] = list(L.bin_centre('kitchen'))
        with self.assertRaises(SystemExit):
            L.bootstrap(cubes)


class MatchTest(unittest.TestCase):
    def test_a_detection_is_scored_against_the_tracked_identity(self):
        tracked = [dict(category='kitchen', map_cm=[0.0, 8.0]), dict(category='other', map_cm=[12.0, 8.0])]
        pairs, extras = L.match(tracked, [cube([0.3, 8.2], 'other'), cube([12.0, 8.0], 'other')])
        s = L.score(pairs, extras)
        self.assertEqual((s['found'], s['right']), (2, 1))
        self.assertEqual(s['wrong'], [dict(truth='kitchen', said='other', at=[0.0, 8.0])])
        self.assertEqual(s['extras'], [])

    def test_a_block_with_no_detection_near_it_counts_as_missed(self):
        tracked = [dict(category='kitchen', map_cm=[0.0, 8.0])]
        pairs, extras = L.match(tracked, [cube([9.0, 8.0], 'kitchen')])
        s = L.score(pairs, extras)
        self.assertEqual((s['found'], s['missed'], s['extras']), (0, ['kitchen'], [[9.0, 8.0]]))

    def test_each_detection_is_claimed_once(self):
        tracked = [dict(category='kitchen', map_cm=[0.0, 8.0]), dict(category='other', map_cm=[1.0, 8.0])]
        pairs, _ = L.match(tracked, [cube([0.2, 8.0], 'kitchen')])
        self.assertEqual(sum(p['cube'] is not None for p in pairs), 1)


class PhaseTest(unittest.TestCase):
    def setUp(self):
        self.moves = []
        saved = L.SCR.move, L.R.decide, L.R.execute, L.R.plan_for, G.log, G.back_off
        self.addCleanup(lambda: setattr_all(saved))

        def setattr_all(v):
            L.SCR.move, L.R.decide, L.R.execute, L.R.plan_for, G.log, G.back_off = v

        self.backed_off = 0
        G.back_off = lambda sess=None: setattr(self, 'backed_off', self.backed_off + 1)

        G.log = lambda *a, **k: None
        L.R.plan_for = lambda c, prefer=150: dict(plan='x')
        def move(cube_, markers, plan_for, sess):
            marker = markers[0] if isinstance(markers, list) else markers
            self.moves.append((tuple(cube_['map_cm']), marker))
            return dict(marker=marker, frm=cube_['map_cm'],
                        target_map_cm=[round(v * 100, 1) for v in G.marker_centre(marker)])

        L.SCR.move = move
        L.R.decide = lambda scene: [dict(cube=c, bin=L.bin_of(c['category']), skip=None) for c in scene['cubes']]
        L.R.execute = lambda run, scene, jobs, sess, rng=None: (
            self.moves.append(('sort', len(jobs))) or dict(sorted=len(jobs), missed=0, skipped=0, cleared=0))

    def test_a_sorted_table_is_scattered(self):
        tracked = L.bootstrap(sorted_table())
        out = L.phase(tracked, dict(cubes=sorted_table()), None, random.Random(2))
        self.assertEqual(out['phase'], 'scatter')
        self.assertEqual(len(self.moves), 4)
        self.assertFalse(L.on_bins(tracked))                 # the tracked positions followed the arm

    def test_a_scattered_table_is_sorted_and_the_blocks_are_tracked_to_their_bins(self):
        tracked = [dict(category='kitchen', map_cm=[0.0, 8.0]), dict(category='other', map_cm=[12.0, 18.0])]
        scene = dict(cubes=[cube([0.0, 8.0], 'kitchen'), cube([12.0, 18.0], 'other')])
        out = L.phase(tracked, scene, None, random.Random(2))
        self.assertEqual(out['phase'], 'sort')
        self.assertEqual([t['map_cm'] for t in tracked], [L.bin_centre('kitchen'), L.bin_centre('other')])

    def test_a_failed_scatter_move_is_recorded_and_the_block_stays_put(self):
        def boom(cube_, markers, plan_for, sess):
            raise G.GripMissed('空夹')

        L.SCR.move = boom
        tracked = L.bootstrap(sorted_table())
        before = [list(t['map_cm']) for t in tracked]
        out = L.phase(tracked, dict(cubes=sorted_table()), None, random.Random(2))
        self.assertEqual(out['failed'], 4)
        self.assertEqual([t['map_cm'] for t in tracked], before)
        # a stop leaves the arm where it stopped; the next block must not plan its path from there
        self.assertEqual(self.backed_off, 4)


class ProgressTest(PhaseTest):
    def test_a_phase_reports_how_many_blocks_actually_moved(self):
        tracked = L.bootstrap(sorted_table())
        out = L.phase(tracked, dict(cubes=sorted_table()), None, random.Random(2))
        self.assertEqual(out['moved_count'], 4)

    def test_a_scatter_where_every_move_failed_reports_no_progress(self):
        def boom(cube_, markers, plan_for, sess):
            raise G.GripMissed('空夹')

        L.SCR.move = boom
        out = L.phase(L.bootstrap(sorted_table()), dict(cubes=sorted_table()), None, random.Random(2))
        self.assertEqual(out['moved_count'], 0)


if __name__ == '__main__':
    unittest.main()
