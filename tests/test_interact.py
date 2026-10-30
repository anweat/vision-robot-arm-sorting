"""Voice/gesture -> command rules (src/interact/intents.py) and their HTTP side in the simulation."""
import json
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for sub in ('interact', 'tools', 'teleop', 'spatial', 'vision', 'sim'):
    sys.path.insert(0, str(ROOT / 'src' / sub))
import intents as I  # noqa: E402
import log_contract as L  # noqa: E402


class Fake:
    def __init__(self):
        self.calls, self.now = [], 1000.0

    def execute(self, command, preset):
        self.calls.append((command, preset))
        if command == 'run_preset':
            raise NotImplementedError('预设动作尚未接入')
        if command == 'start':
            raise ValueError('有页面持有操作权')
        return 'ok'

    def make(self):
        return I.Interaction(self.execute, clock=lambda: self.now)


class VoiceTest(unittest.TestCase):
    def setUp(self):
        self.f = Fake()
        self.it = self.f.make()

    def test_stop_works_while_unarmed_and_writes_a_voice_event_without_confidence(self):
        r = self.it.voice('请停止。')
        self.assertEqual((r['status'], r['command']), ('accepted', 'stop'))
        self.assertEqual(L.validate(r['event']), [])
        self.assertNotIn('confidence', r['event'])

    def test_anything_but_stop_needs_arming(self):
        r = self.it.voice('启动')
        self.assertEqual(r['status'], 'rejected')
        self.assertEqual(self.f.calls, [])
        self.it.arm(True)
        self.assertEqual(self.it.voice('启动')['reason'], '有页面持有操作权')   # refused by the executor, said why

    def test_text_without_a_command_word_is_ignored(self):
        self.assertEqual(self.it.voice('今天天气不错')['status'], 'ignored')
        self.assertEqual(self.f.calls, [])

    def test_two_commands_in_one_sentence_run_nothing_but_a_stop_word_wins(self):
        self.it.arm(True)
        self.assertEqual(self.it.voice('启动然后跳舞')['status'], 'ignored')
        self.assertEqual(self.it.voice('启动然后停止')['command'], 'stop')

    def test_an_unimplemented_preset_is_rejected_not_faked(self):
        self.it.arm(True)
        r = self.it.voice('跳舞')
        self.assertEqual((r['status'], r['implemented']), ('rejected', False))
        self.assertNotIn('event', r)

    def test_cooldown_applies_to_repeated_commands_but_never_to_stop(self):
        self.it.arm(True)
        self.it.voice('停止')
        self.assertEqual(self.it.voice('停止')['status'], 'accepted')
        self.f.calls.clear()
        self.it.cfg['voice']['words']['run_preset:wave'] = ['挥手']
        self.it.voice('挥手')
        self.assertEqual(self.it.voice('挥手')['reason'], '冷却中')
        self.f.now += 3
        self.assertEqual(self.it.voice('挥手')['status'], 'rejected')   # preset again refused, but no longer cooling

    def test_bad_input_is_an_error(self):
        with self.assertRaises(ValueError):
            self.it.voice(None)


class GestureTest(unittest.TestCase):
    def setUp(self):
        self.f = Fake()
        self.it = self.f.make()

    def frames(self, label, conf, n):
        return [self.it.gesture(label, conf) for _ in range(n)]

    def test_palm_fires_stop_once_after_three_frames_and_carries_the_probability(self):
        out = self.frames('palm', 0.9, 5)
        self.assertEqual([r['status'] for r in out], ['ignored', 'ignored', 'accepted', 'ignored', 'ignored'])
        self.assertEqual(out[2]['event']['confidence'], 0.9)
        self.assertEqual(L.validate(out[2]['event']), [])
        self.assertEqual(self.f.calls, [('stop', None)])

    def test_a_weak_frame_breaks_the_run(self):
        self.frames('palm', 0.9, 2)
        self.it.gesture('palm', 0.3)
        self.assertEqual([r['status'] for r in self.frames('palm', 0.9, 3)], ['ignored', 'ignored', 'accepted'])

    def test_unbound_gestures_never_run_anything(self):
        for label in ('fist', 'like', 'ok'):
            self.assertTrue(all(r['status'] == 'ignored' for r in self.frames(label, 0.99, 6)))
        self.assertEqual(self.f.calls, [])

    def test_holding_again_after_another_gesture_fires_again_after_the_cooldown(self):
        self.frames('palm', 0.9, 3)
        self.frames('fist', 0.9, 1)
        self.f.now += 3
        self.assertEqual(self.frames('palm', 0.9, 3)[-1]['status'], 'accepted')

    def test_bad_input_is_an_error(self):
        for args in (('wave', 0.9), ('palm', 1.5), ('palm', True), ('palm', None)):
            with self.assertRaises(ValueError):
                self.it.gesture(*args)


class ConfigTest(unittest.TestCase):
    def test_the_shipped_config_binds_only_known_gestures(self):
        cfg = I.load_config()
        self.assertEqual(set(cfg['gesture']['map']), set(cfg['gesture']['labels']))
        self.assertEqual(cfg['gesture']['labels'], ['fist', 'like', 'ok', 'palm'])   # the hrnet.om output order


try:
    import cv2  # noqa: F401
    import numpy  # noqa: F401
    import world as W
    READY = (W.FACES_DIR / 'index.json').exists()
except ImportError:
    READY = False


@unittest.skipUnless(READY, 'needs numpy, OpenCV and data/reference/block_faces')
class SimEndpointTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sim_server as S
        cls.tmp = tempfile.TemporaryDirectory()
        cls.app, cls.camera = S.build(Path(cls.tmp.name) / 'session', blocks=2, seed=1)
        cls.app.log_path = Path(cls.tmp.name) / 'log.jsonl'
        cls.httpd = S.console.ThreadingHTTPServer(('127.0.0.1', 0), S.handler(cls.app))
        cls.httpd.daemon_threads = True
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()
        cls.base = f'http://127.0.0.1:{cls.httpd.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.app.closed.set()
        cls.app.locator.closed.set()
        cls.camera.close()
        cls.httpd.shutdown()
        cls.httpd.server_close()
        cls.tmp.cleanup()

    def get(self, path):
        return json.loads(urllib.request.urlopen(self.base + path, timeout=20).read())

    def post(self, path, **body):
        token = self.get('/api/bootstrap')['token']
        req = urllib.request.Request(self.base + path, json.dumps(body).encode(),
                                     {'Content-Type': 'application/json', 'X-Console-Token': token})
        return json.loads(urllib.request.urlopen(req, timeout=30).read())

    def test_voice_stop_and_palm_gesture_reach_the_log_as_interaction_events(self):
        self.assertFalse(self.get('/api/interact')['armed'])
        self.assertEqual(self.post('/api/interact/voice', text='停止')['status'], 'accepted')
        for _ in range(3):
            r = self.post('/api/interact/gesture', label='palm', confidence=0.93)
        self.assertEqual(r['status'], 'accepted')
        events = self.get('/api/log')['events']
        got = [(e['source'], e['command'], e.get('confidence')) for e in events if e['event'] == 'interaction']
        self.assertEqual(got, [('voice', 'stop', None), ('gesture', 'stop', 0.93)])
        self.assertTrue(all(not L.validate(e) for e in events))
        self.assertEqual(self.get('/api/interact')['recent'][-1]['input'], 'palm')

    def test_refused_and_ignored_recognitions_are_shown_but_not_logged(self):
        before = len(self.get('/api/log')['events'])
        self.assertEqual(self.post('/api/interact/voice', text='启动')['status'], 'rejected')   # unarmed
        self.post('/api/interact/arm', armed=True)
        r = self.post('/api/interact/voice', text='跳舞')
        self.assertEqual((r['status'], r['implemented']), ('rejected', False))
        self.post('/api/interact/arm', armed=False)
        self.assertEqual(len(self.get('/api/log')['events']), before)
        reasons = [x['reason'] for x in self.get('/api/interact')['recent']]
        self.assertTrue(any('预设动作' in x for x in reasons))


if __name__ == '__main__':
    unittest.main()
