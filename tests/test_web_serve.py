"""The outward HTTP service (src/web/serve.py) the pages are built against. Standard library only.

It must answer the same four paths as the simulation backend (日志契约 §5), so a page written
here keeps working when the data source switches from simulation to the real machine. It also
carries the stage-1 acceptance numbers: page rows == log lines, and events rendered within 1 s.
"""
import json
import http.client
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'web'))
sys.path.insert(0, str(ROOT / 'src' / 'tools'))
import serve as S  # noqa: E402
import log_contract as L  # noqa: E402

SAMPLE = ROOT / 'docs' / 'samples' / 'log_sim_8rounds.jsonl'


class ServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.log = Path(cls.tmp.name) / 'log.jsonl'
        cls.log.write_text(SAMPLE.read_text(encoding='utf-8'), encoding='utf-8')
        cls.state = S.State(cls.log)
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler)
        cls.server.state = cls.state
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.tmp.cleanup()

    def _get(self, path):
        with urllib.request.urlopen(f'http://127.0.0.1:{self.port}{path}', timeout=5) as r:
            return r.status, r.read(), r.headers

    def _get_json(self, path):
        status, body, _ = self._get(path)
        return status, json.loads(body)

    def _sse(self, path, want=4000):
        """SSE 是常连接，只读够 want 字节就断开，不能读到 EOF。"""
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        try:
            conn.request('GET', path)
            response = conn.getresponse()
            ctype = response.getheader('Content-Type') or ''
            buf = b''
            while len(buf) < want:
                chunk = response.read1(want - len(buf))
                if not chunk:
                    break
                buf += chunk
            return response.status, ctype, buf.decode('utf-8')
        finally:
            conn.close()

    def _post(self, body):
        req = urllib.request.Request(f'http://127.0.0.1:{self.port}/api/cmd',
                                     data=json.dumps(body).encode(), method='POST',
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b'{}')

    # ---- the four log paths --------------------------------------------
    def test_log_pagination_matches_the_log_line_count(self):
        _, total = L.read(self.log)
        status, page = self._get_json('/api/log?after=0&limit=3')
        self.assertEqual(status, 200)
        self.assertEqual(page['contract_version'], L.VERSION)
        self.assertEqual([e['seq'] for e in page['events']], [1, 2, 3])
        self.assertEqual(page['total'], total)
        status, rest = self._get_json(f"/api/log?after={page['next']}&limit=2000")
        self.assertEqual(page['total'], len(page['events']) + len(rest['events']))

    def test_records_and_summary_agree_with_the_contract_implementation(self):
        events, _ = L.read(self.log)
        _, records = self._get_json('/api/log/records')
        _, summary = self._get_json('/api/log/summary')
        self.assertEqual(records['records'], L.records(events))
        self.assertEqual(summary, L.summary(events))

    def test_every_served_event_keeps_the_contract(self):
        _, page = self._get_json('/api/log?after=0&limit=2000')
        problems = [(e['seq'], p) for e in page['events'] for p in L.validate(e)]
        self.assertEqual(problems, [])

    def test_log_stream_pushes_numbered_sse_events(self):
        status, ctype, chunk = self._sse('/api/log/stream?after=0')
        self.assertEqual(status, 200)
        self.assertIn('text/event-stream', ctype)
        self.assertIn('id: 1\n', chunk)
        self.assertIn('"seq":1', chunk.replace(' ', ''))    # data 行是带 seq 的事件 JSON

    def test_stream_resumes_from_last_event_id(self):
        status, ctype, chunk = self._sse('/api/log/stream?after=3')
        self.assertEqual(status, 200)
        ids = [int(line[4:]) for line in chunk.splitlines() if line.startswith('id: ')]
        self.assertTrue(ids)
        self.assertNotIn(1, ids)
        self.assertEqual(min(ids), 4)

    # ---- status and cmd --------------------------------------------------
    def test_status_reports_mode_and_latest_readback(self):
        _, status = self._get_json('/api/status')
        self.assertEqual(status['mode'], self.state.mode)
        self.assertEqual(status['contract_version'], L.VERSION)

    def test_set_mode_is_accepted_and_written_to_the_log(self):
        before, _ = L.read(self.log)
        status, reply = self._post({'cmd': 'set_mode', 'mode': 'auto', 'source': 'web'})
        self.assertEqual(status, 200)
        self.assertEqual(reply['mode'], 'auto')
        self.assertEqual(self.state.status()['mode'], 'auto')
        events, _ = L.read(self.log)
        changed = [e for e in events if e['event'] == 'mode_changed']
        self.assertEqual(len(changed), 1)
        self.assertEqual(L.validate(changed[0]), [])        # 拟定事件也要守契约
        self.assertEqual(changed[0]['previous'], 'manual')
        self.assertEqual(len(events), len(before) + 1)

    def test_rejected_commands_do_not_touch_the_log(self):
        before, _ = L.read(self.log)
        for body, code in [({'cmd': 'set_mode', 'mode': 'nope'}, 400),
                           ({'cmd': 'nope'}, 400),
                           ({'cmd': 'set_mode'}, 400),
                           ({'cmd': 'set_joints', 'joints_deg': [90] * 6}, 501),
                           ({'cmd': 'move_to', 'target_cm': [0, 16, 2]}, 501),
                           ({'cmd': 'run_preset', 'preset': 'wiggle'}, 400)]:
            status, reply = self._post(body)
            self.assertEqual(status, code, body)
            self.assertIn('error', reply, body)
        after, _ = L.read(self.log)
        self.assertEqual(len(after), len(before))           # 没有写入半条事件

    def test_stop_is_accepted(self):
        status, reply = self._post({'cmd': 'stop', 'source': 'web'})
        self.assertEqual(status, 200)
        self.assertTrue(reply['ok'])

    # ---- pages and misc ---------------------------------------------------
    def test_pages_are_served_from_the_same_port(self):
        for path in ('/', '/style.css', '/js/app.js', '/js/contract.js', '/js/api.js'):
            status, body, headers = self._get(path)
            self.assertEqual(status, 200, path)
        status, _, _ = self._get('/js/app.js')
        self.assertEqual(status, 200)

    def test_unknown_paths_are_404_not_crashes(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self._get('/api/nope')
        self.assertEqual(ctx.exception.code, 404)
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self._get('/js/../../serve.py')
        self.assertEqual(ctx.exception.code, 404)

    def test_replayer_writes_the_sample_at_wall_clock_time(self):
        state = S.State(Path(self.tmp.name) / 'replay.jsonl')
        replayer = S.Replayer(SAMPLE, state, speed=50)
        replayer.start()
        replayer.join(timeout=30)
        events, total = L.read(state.log_path)
        self.assertEqual(len(events), total)
        self.assertGreater(len(events), 100)
        for e in events:
            self.assertLessEqual(e['t'], time.time() + 1)   # t 已改成写入时刻
        self.assertEqual(L.validate([e for e in events if e['event'] in L.EVENTS][0]), [])

    # ---- 写操作的边界：同源 + JSON，不放开跨源 -------------------------------
    def _post_raw(self, body: bytes, headers: dict):
        req = urllib.request.Request(f'http://127.0.0.1:{self.port}/api/cmd',
                                     data=body, method='POST', headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b'{}')

    def test_cross_origin_write_is_rejected(self):
        status, reply = self._post_raw(
            json.dumps({'cmd': 'stop'}).encode(),
            {'Content-Type': 'application/json', 'Origin': 'http://evil.example'})
        self.assertEqual(status, 403)
        self.assertIn('error', reply)

    def test_same_origin_write_is_accepted(self):
        status, reply = self._post_raw(
            json.dumps({'cmd': 'stop'}).encode(),
            {'Content-Type': 'application/json',
             'Origin': f'http://127.0.0.1:{self.port}', 'Host': f'127.0.0.1:{self.port}'})
        self.assertEqual(status, 200)
        self.assertTrue(reply['ok'])

    def test_non_json_write_is_rejected(self):
        # 表单/纯文本可以绕过 CORS 预检直接打过来，必须按 Content-Type 挡掉
        status, _ = self._post_raw(b'cmd=stop', {'Content-Type': 'text/plain'})
        self.assertEqual(status, 415)

    def test_no_cors_wildcard_on_answers(self):
        _, _, headers = self._get('/api/status')
        self.assertNotIn('Access-Control-Allow-Origin', headers)


class TokenTest(unittest.TestCase):
    """--token 打开后：静态页面照常能开（不然白屏），API 才校验口令。"""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.state = S.State(Path(cls.tmp.name) / 'log.jsonl', token='s3cret')
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler)
        cls.server.state = cls.state
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.tmp.cleanup()

    def _status(self, path):
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{self.port}{path}', timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def test_static_pages_load_without_a_token(self):
        for path in ('/', '/style.css', '/js/app.js'):
            self.assertEqual(self._status(path), 200, path)

    def test_api_needs_the_token(self):
        self.assertEqual(self._status('/api/status'), 401)
        self.assertEqual(self._status('/api/status?token=wrong'), 401)
        self.assertEqual(self._status('/api/status?token=s3cret'), 200)


if __name__ == '__main__':
    unittest.main()
