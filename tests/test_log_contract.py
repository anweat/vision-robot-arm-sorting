"""The log.jsonl contract (src/tools/log_contract.py) against the checked-in samples. Standard library only."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'tools'))
import log_contract as L  # noqa: E402

SAMPLES = ROOT / 'docs' / 'samples'


class ContractTest(unittest.TestCase):
    def test_both_sample_logs_keep_the_contract(self):
        for name in ('log_real_arm.jsonl', 'log_sim_8rounds.jsonl'):
            events, total = L.read(SAMPLES / name)
            self.assertEqual(len(events), total)
            self.assertEqual([e['seq'] for e in events], list(range(1, total + 1)))
            self.assertEqual([(e['seq'], p) for e in events for p in L.validate(e)], [], name)

    def test_the_simulated_sample_has_every_kind_of_outcome_a_page_must_show(self):
        events, _ = L.read(SAMPLES / 'log_sim_8rounds.jsonl')
        rows = L.records(events)
        self.assertEqual({r['result'] for r in rows}, {'sorted', 'missed', 'skipped'})
        self.assertEqual(sorted({r['round'] for r in rows}), list(range(1, 9)))
        s = L.summary(events)
        self.assertEqual(s['rounds_finished'], 8)
        self.assertEqual(sum(s['by_result'].values()), len(rows))
        # each round's own count agrees with the records inside it
        ends = [e for e in events if e['event'] == 'pipeline_end']
        for n, end in enumerate(ends, 1):
            mine = [r for r in rows if r['round'] == n]
            self.assertEqual(sum(r['result'] == 'sorted' for r in mine), end['sorted'])
            self.assertEqual(sum(r['result'] == 'missed' for r in mine), end['missed'])

    def test_a_record_carries_the_six_items_the_requirement_names(self):
        row = L.records([dict(seq=1, t=5.0, event='pipeline_start', run='r', cubes=[[1, 2]], registration={}),
                         dict(seq=2, t=9.0, event='pipeline_sorted', cube=[1, 2], category='kitchen', bin='41', seconds=4)])[0]
        self.assertEqual((row['t'], row['category'], row['cube'], row['bin'], row['seconds'], row['result'], row['round']),
                         (9.0, 'kitchen', [1, 2], '41', 4, 'sorted', 1))

    def test_broken_events_are_reported_and_unknown_ones_are_not(self):
        self.assertEqual(L.validate(dict(t=1.0, event='some_debug_detail', anything=[1])), [])
        self.assertTrue(L.validate(dict(t=1.0, event='pipeline_sorted', cube=[1], category='kitchen', bin='41', seconds=3)))
        self.assertTrue(L.validate(dict(t=1.0, event='pipeline_sorted', cube=[1, 2], category='glass', bin='41', seconds=3)))
        self.assertTrue(L.validate(dict(event='stopped', reason='x')))
        self.assertEqual(L.validate(dict(t=1.0, event='pipeline_skip', cube=[1, 2], category=None, reason='no evidence')), [])
        self.assertEqual(L.validate(dict(t=1.0, event='stall', joint=3, expected=40, readback=31)), [])
        self.assertTrue(L.validate(dict(t=1.0, event='mode_changed', mode='turbo', source='web')))

    def test_reading_resumes_by_line_number_and_leaves_a_half_written_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'log.jsonl'
            path.write_text('{"t": 1, "event": "a"}\nnot json\n{"t": 3, "event": "c"}\n{"t": 4, "eve')
            events, total = L.read(path)
            self.assertEqual((total, [e['event'] for e in events]), (3, ['a', '_invalid', 'c']))
            events, total = L.read(path, after=2)
            self.assertEqual([(e['seq'], e['event']) for e in events], [(3, 'c')])
            self.assertEqual(L.read(path, after=0, limit=1)[0][0]['seq'], 1)
            self.assertEqual(L.read(Path(tmp) / 'missing.jsonl'), ([], 0))

    def test_the_checked_in_schema_is_the_one_the_code_generates(self):
        # 必须显式写 utf-8：Windows 默认 locale 是 GBK，不写会 UnicodeDecodeError（2026-10-22）
        on_disk = json.loads((ROOT / 'docs/contracts/log_event.schema.json').read_text(encoding='utf-8'))
        self.assertEqual(on_disk, L.schema())


if __name__ == '__main__':
    unittest.main()
