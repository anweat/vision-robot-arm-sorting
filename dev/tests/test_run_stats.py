import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'dev' / sub))
    sys.path.insert(0, str(ROOT / 'app' / 'src' / sub))
import run_stats as S  # noqa: E402


def log(events):
    """Events as (seconds-from-zero, event, fields) -> the parsed runs."""
    return S.split_runs([dict(t=float(t), event=e, **f) for t, e, f in events])


def block(t0, run='r', sorted_at=None, missed=False, wake_refusals=0, aligned=None):
    """The event sequence the console writes for one block."""
    out = [(t0, 'hovering', dict(pose=[], tip_cm=[], colour=None, aligned=aligned))]
    out += [(t0 + 1 + i, 'gripper_wake_refused', dict(reason='servo 6')) for i in range(wake_refusals)]
    t = t0 + 1 + wake_refusals
    out += [(t, 'gripper_awake', dict(start=29, readback=33))]
    if missed:
        return out + [(t + 2, 'pipeline_missed', dict(cube=[1.0, 2.0], category='kitchen', reason='空夹', seconds=5))]
    out += [(t + 1, 'gripped', dict(pose=[], tip_cm=[], grip=dict(readback=129))),
            (t + 3, 'lifted', dict(pose=[], gripper_readback=129)),
            (t + 3, 'put_planned', dict(target_map_cm=[6, 13], pitch=145, hover=[], j5=92)),
            (t + 20, 'put', dict(target='43', target_map_cm=[6, 13], released_at=[])),
            (t + 20, 'pipeline_sorted', dict(cube=[1.0, 2.0], bin='43', category='hazardous',
                                             source='detector', seconds=sorted_at or 25))]
    return out


class SplitTest(unittest.TestCase):
    def test_an_explicit_start_opens_a_run_and_scan_views_are_counted(self):
        runs = log([(0, 'scan_view', dict(run='a')), (5, 'scan_view', dict(run='a')),
                    (10, 'pipeline_start', dict(run='a', cubes=[], registration={}))]
                   + block(12) + [(40, 'pipeline_end', dict(run='a', sorted=1, skipped=0, missed=0, seconds=40))])
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]['scan_views'], 2)

    def test_a_log_without_the_newer_events_splits_on_the_next_scan(self):
        """Runs recorded before pipeline_start existed still have to come apart, or every night
        collapses into one 'run' and the per-round rates are meaningless."""
        runs = log([(0, 'scan_view', dict(run='a'))] + block(5)
                   + [(60, 'scan_view', dict(run='b'))] + block(65))
        self.assertEqual(len(runs), 2)
        self.assertEqual([r['run'] for r in runs], ['a', 'b'])


class BlocksTest(unittest.TestCase):
    def runs_with(self, events):
        runs = log(events)
        for r in runs:
            r['blocks'] = S.blocks_of(r)
        return runs

    def test_phases_and_the_wake_retries_are_measured(self):
        r = self.runs_with([(0, 'scan_view', dict(run='a'))] + block(10, wake_refusals=3))[0]
        b, = r['blocks']
        self.assertEqual(b['outcome'], 'sorted')
        self.assertEqual(b['wake_refusals'], 3)
        self.assertEqual(b['wake'], 4.0)          # hovering -> awake, the three refusals inside it
        self.assertEqual(b['lift'], 2.0)
        self.assertEqual(b['put'], 17.0)
        self.assertEqual(b['grip_readback'], 129)

    def test_a_miss_is_kept_as_a_record_not_dropped(self):
        r = self.runs_with([(0, 'scan_view', dict(run='a'))] + block(10, missed=True))[0]
        self.assertEqual([b['outcome'] for b in r['blocks']], ['missed'])

    def test_skipped_blocks_are_recorded_with_their_reason(self):
        r = self.runs_with([(0, 'scan_view', dict(run='a')),
                            (5, 'pipeline_skip', dict(cube=[1, 2], category='other', reason='bin 44 occupied'))])[0]
        b, = r['blocks']
        self.assertEqual((b['outcome'], b['reason']), ('skipped', 'bin 44 occupied'))

    def test_a_run_cut_short_leaves_the_block_unfinished_rather_than_sorted(self):
        r = self.runs_with([(0, 'scan_view', dict(run='a')),
                            (10, 'hovering', dict(pose=[], tip_cm=[], colour=None)),
                            (12, 'gripper_awake', dict(start=29, readback=33))])[0]
        self.assertEqual([b['outcome'] for b in r['blocks']], ['unfinished'])


class SummaryTest(unittest.TestCase):
    def test_rates_count_blocks_over_several_runs(self):
        events = ([(0, 'scan_view', dict(run='a'))] + block(10) + block(40, wake_refusals=2)
                  + [(100, 'scan_view', dict(run='b'))] + block(110, missed=True)
                  + [(140, 'pipeline_skip', dict(cube=[0, 0], category='kitchen', reason='already on its bin (kitchen)'))])
        runs = log(events)
        for r in runs:
            r['blocks'] = S.blocks_of(r)
        s = S.summarize(runs)
        self.assertEqual((s['runs'], s['blocks_acted_on'], s['sorted'], s['missed'], s['skipped']), (2, 3, 2, 1, 1))
        self.assertEqual(s['woke_first_try'], 2)
        self.assertEqual(s['wake_refusals'], 2)
        self.assertEqual(s['skip_reasons'], {'already on its bin': 1})
        self.assertEqual(s['phase_seconds']['put']['n'], 2)

    def test_grip_success_is_split_by_whether_the_fingers_lined_up(self):
        """Corner-on fingers flick a block away (2026-09-27); whether flat pitches may use a quarter
        turn is decided from this split, not from a guess."""
        events = ([(0, 'scan_view', dict(run='a'))] + block(10, aligned=True)
                  + block(40, aligned=False, missed=True) + block(70, aligned=False))
        runs = log(events)
        for r in runs:
            r['blocks'] = S.blocks_of(r)
        s = S.summarize(runs)
        self.assertEqual(s['grip_by_alignment']['对齐'], dict(失败=0, 成功=1))
        self.assertEqual(s['grip_by_alignment']['未对齐'], dict(失败=1, 成功=1))

    def test_a_torn_line_does_not_stop_the_read(self):
        with tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False) as f:
            f.write(json.dumps(dict(t=1.0, event='scan_view', run='a')) + '\n{"t": 2.0, "even')
        self.assertEqual(len(S.read(Path(f.name))), 1)


if __name__ == '__main__':
    unittest.main()
