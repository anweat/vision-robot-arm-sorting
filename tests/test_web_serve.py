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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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


class _FakeConsole(BaseHTTPRequestHandler):
    """最小假控制台：GET /api/bootstrap 给凭证、GET /api/status 给 revision，
    其余 POST 按 cfg['move_queue'] 排队返回（测 Console.move 的 409 分类与重试）。"""

    def log_message(self, *a):
        pass

    def _send(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        cfg = self.server.cfg
        if self.path == "/api/bootstrap":
            return self._send({"token": "t"})
        if self.path == "/api/status":
            cfg["status_calls"] += 1
            return self._send({"revision": cfg["revision"]})
        self._send({}, 404)

    def do_POST(self):
        cfg = self.server.cfg
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n:
            self.rfile.read(n)
        cfg["move_calls"] += 1
        if cfg["move_queue"]:
            status, payload = cfg["move_queue"].pop(0)
            return self._send(payload, status)
        self._send({"ok": True})


class ConsoleMoveTest(unittest.TestCase):
    """1.4：只有 reason=revision_conflict 的 409 才重读 revision 重试；其余 409 原样上抛，
    不能把'未使能/正在运动/画面过期'改写成'revision 反复过期'吞掉真因。"""

    def setUp(self):
        self.cfg = {"revision": 18, "status_calls": 0, "move_calls": 0, "move_queue": []}
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _FakeConsole)
        self.server.cfg = self.cfg
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.console = S.Console(f"http://127.0.0.1:{self.port}")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_non_revision_409_is_raised_verbatim_without_retry(self):
        msg = "未使能、连接超时或正在运动；指令不排队"
        self.cfg["move_queue"] = [(409, {"error": msg})]
        with self.assertRaises(S.ConsoleError) as cm:
            self.console.move("/api/move", {"joints": [90, 90, 90, 90, 90, 90]})
        self.assertEqual(str(cm.exception), msg)          # 真因不被改写
        self.assertIsNone(cm.exception.reason)            # 不带 reason → 不当 revision 冲突
        self.assertEqual(cm.exception.status, 409)
        self.assertEqual(self.cfg["move_calls"], 1)       # 没有重试

    def test_revision_conflict_retries_then_succeeds(self):
        self.cfg["move_queue"] = [
            (409, {"error": "状态已变化，请刷新后重新点动", "reason": "revision_conflict"}),
            (200, {"ok": True}),
        ]
        out = self.console.move("/api/move", {"joints": [90, 90, 90, 90, 90, 90]})
        self.assertEqual(out, {"ok": True})
        self.assertEqual(self.cfg["move_calls"], 2)       # 重读 revision 后重试了一次
        self.assertGreaterEqual(self.cfg["status_calls"], 2)

    def test_repeated_revision_conflict_gives_up_after_two_attempts(self):
        self.cfg["move_queue"] = [
            (409, {"error": "状态已变化，请刷新后重新点动", "reason": "revision_conflict"}),
            (409, {"error": "状态已变化，请刷新后重新点动", "reason": "revision_conflict"}),
        ]
        with self.assertRaises(S.ConsoleError) as cm:
            self.console.move("/api/move", {"joints": [90, 90, 90, 90, 90, 90]})
        self.assertEqual(cm.exception.reason, "revision_conflict")
        self.assertEqual(self.cfg["move_calls"], 2)


class PerceptionEndpointTest(unittest.TestCase):
    """语音 / 手势端点：text / label 直通联调整条链路，模型未部署时 501 不假装识别，
    识别输入写 interaction 事件且守日志契约。"""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.log = Path(cls.tmp.name) / 'log.jsonl'
        cls.log.write_text(SAMPLE.read_text(encoding='utf-8'), encoding='utf-8')
        # 不挂控制台、不配模型目录：测直通与降级，不碰真推理
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

    def _post(self, path, body):
        req = urllib.request.Request(f'http://127.0.0.1:{self.port}{path}',
                                     data=json.dumps(body).encode(), method='POST',
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b'{}')

    def _new_events(self, before_count):
        events, _ = L.read(self.log)
        return events[before_count:]

    # ---- voice ----------------------------------------------------------
    def test_voice_text_maps_to_command_and_logs_interaction(self):
        before, _ = L.read(self.log)
        status, reply = self._post('/api/voice', {'text': '启动', 'dispatch': False})
        self.assertEqual(status, 200)
        self.assertEqual(reply['command'], {'cmd': 'start'})
        self.assertFalse(reply['dispatched'])
        inter = [e for e in self._new_events(len(before)) if e.get('event') == 'interaction']
        self.assertEqual(len(inter), 1)
        self.assertEqual(inter[0]['source'], 'voice')
        self.assertEqual(inter[0]['command'], 'recognize')   # 与 _cmd 链的执行事件区分开
        self.assertEqual(inter[0]['detail'], '启动')
        self.assertEqual(L.validate(inter[0]), [])           # 拟定事件也要守契约

    def test_voice_stop_dispatches_through_cmd_without_console(self):
        # stop 不依赖控制台：dispatch=true 应经 _cmd 返回 200，而不是 501
        status, reply = self._post('/api/voice', {'text': '停止'})
        self.assertEqual(status, 200)
        self.assertTrue(reply['ok'])

    def test_voice_audio_without_model_is_501_not_500(self):
        status, reply = self._post('/api/voice', {'audio': 'ZmFrZQ=='})   # base64 "fake"
        self.assertEqual(status, 501)
        self.assertIn('模型', reply['error'])

    def test_voice_without_input_is_400(self):
        status, _ = self._post('/api/voice', {})
        self.assertEqual(status, 400)

    def test_voice_unmatched_text_returns_command_none(self):
        before, _ = L.read(self.log)
        status, reply = self._post('/api/voice', {'text': '今天天气不错'})
        self.assertEqual(status, 200)
        self.assertIsNone(reply['command'])
        self.assertFalse(reply['dispatched'])
        # 没映射出指令也不应写半条执行事件，只写识别记录
        new = self._new_events(len(before))
        self.assertTrue(all(e.get('event') == 'interaction' for e in new))

    # ---- gesture --------------------------------------------------------
    def test_gesture_label_maps_to_command_and_logs_interaction(self):
        before, _ = L.read(self.log)
        status, reply = self._post('/api/gesture', {'label': 'fist', 'dispatch': False})
        self.assertEqual(status, 200)
        self.assertEqual(reply['command'], {'cmd': 'gripper', 'action': 'grip'})
        inter = [e for e in self._new_events(len(before)) if e.get('event') == 'interaction']
        self.assertEqual(len(inter), 1)
        self.assertEqual(inter[0]['source'], 'gesture')
        self.assertEqual(inter[0]['detail'], 'fist')
        self.assertEqual(L.validate(inter[0]), [])

    def test_gesture_ok_dispatches_stop_through_cmd(self):
        status, reply = self._post('/api/gesture', {'label': 'ok'})   # dispatch 默认 true
        self.assertEqual(status, 200)
        self.assertTrue(reply['ok'])

    def test_gesture_image_without_model_is_501(self):
        status, reply = self._post('/api/gesture', {'image': 'ZmFrZQ=='})
        self.assertEqual(status, 501)
        self.assertIn('模型', reply['error'])

    def test_gesture_snapshot_without_camera_url_is_501(self):
        status, _ = self._post('/api/gesture', {'snapshot': True})
        self.assertEqual(status, 501)

    def test_gesture_empty_label_is_not_a_failure(self):
        # 低于阈值会返回空 label：不下发，但接口本身 200，前端按 note 提示
        status, reply = self._post('/api/gesture', {'label': ''})
        self.assertEqual(status, 200)
        self.assertEqual(reply['label'], '')
        self.assertFalse(reply['dispatched'])

    def test_unknown_paths_still_404(self):
        status, _ = self._post('/api/voice2', {})
        self.assertEqual(status, 404)


if __name__ == '__main__':
    unittest.main()
