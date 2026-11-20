import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'dev' / 'tools'))
import analyze_session as A  # noqa: E402


class AnalyzeSessionTest(unittest.TestCase):
    def test_summary_reclassifies_grips_and_lists_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            s = Path(tmp) / '20260925_000000_test'
            (s / 'episode_1').mkdir(parents=True)
            pose, stuck = [109, 22, 78, 20, 92, 29], [109, 22, 78, 20, 92, 29]
            held = pose[:5] + [129]
            events = [dict(t=0, event='enable'),
                      dict(t=1, event='command', target=pose), dict(t=2, event='readback', observed=pose, kind='jog', profile='explore'),
                      dict(t=3, event='command', target=pose[:5] + [133]), dict(t=4, event='readback', observed=stuck, kind='gripper', profile='explore'),
                      dict(t=4.1, event='gripper', action='grip', command=133, readback=29, holding=True),
                      dict(t=5, event='command', target=pose[:5] + [133]), dict(t=6, event='readback', observed=held, kind='gripper', profile='explore'),
                      dict(t=6.1, event='gripper', action='grip', command=133, readback=129, holding=False),
                      dict(t=7, event='plan_rejected', error='计划被拒绝：目标位置超出可达范围'),
                      dict(t=8, event='plan_rejected', error='计划被拒绝：目标位置超出可达范围')]
            (s / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
            (s / 'episode_1' / 'episode.json').write_text(json.dumps(dict(label='g1', ended=9)))
            (s / 'episode_1' / 'steps.jsonl').write_text(json.dumps(dict(index=1, t=6, kind='gripper', observed=held,
                                                                         phone_image='step_001_phone.jpg')))
            gripper = A.safe_arm.load_config(A.safe_arm.DEFAULT_CONFIG)['gripper']
            text, frames = A.analyse(s, gripper, -8.5)
        self.assertIn('| 29→29 | not_closed | holding |', text)
        self.assertIn('| 29→129 | holding | not holding |', text)
        self.assertIn('拒绝 ×2', text)
        self.assertIn('夹住时的正解高度', text)
        self.assertEqual([f['phone'] for f in frames], ['episode_1/step_001_phone.jpg'])


if __name__ == '__main__':
    unittest.main()
