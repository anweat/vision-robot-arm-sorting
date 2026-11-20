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

SAMPLE = ROOT / 'samples' / 'log_sim_8rounds.jsonl'


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
    """控制台的 409 一律原样上抛，不改写真因（画面过期、另一端持有操作权、正在运动）。"""

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

    def test_no_revision_retry_any_more(self):
        """文档 32：控制台不再要求 revision，网关也不再重读重试；连 revision_conflict 也原样上抛。"""
        self.cfg["move_queue"] = [
            (409, {"error": "状态已变化，请刷新后重新点动", "reason": "revision_conflict"}),
            (200, {"ok": True}),
        ]
        with self.assertRaises(S.ConsoleError) as cm:
            self.console.move("/api/move", {"joints": [90, 90, 90, 90, 90, 90]})
        self.assertEqual(cm.exception.reason, "revision_conflict")
        self.assertEqual(self.cfg["move_calls"], 1)
        self.assertEqual(self.cfg["status_calls"], 0)      # 不再为了 revision 先读状态


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

    # ---- 语音/手势：判定在网关（intents），受理的指令由网关执行并写 interaction ----------------
    def test_interact_state_is_local(self):
        with urllib.request.urlopen(f'http://127.0.0.1:{self.port}/api/interact', timeout=5) as r:
            state = json.loads(r.read())
        self.assertIn('armed', state)
        self.assertIn('listen', state)
        self.assertEqual(state['gesture']['labels'], ['fist', 'like', 'ok', 'palm'])

    def test_unarmed_gesture_is_recorded_not_run(self):
        self._post('/api/interact/arm', {'armed': False})
        before, _ = L.read(self.log)
        for _ in range(3):
            status, reply = self._post('/api/interact/gesture', {'label': 'ok', 'confidence': 0.95})
        self.assertEqual((status, reply['status']), (200, 'rejected'))
        self.assertIn('未武装', reply['reason'])
        with urllib.request.urlopen(f'http://127.0.0.1:{self.port}/api/interact', timeout=5) as r:
            recent = json.loads(r.read())['recent']
        self.assertEqual((recent[-1]['source'], recent[-1]['input']), ('gesture', 'ok'))   # 流程脚本的「等手势」读这里
        self.assertEqual([e for e in self._new_events(len(before)) if e.get('event') == 'interaction'], [])

    def test_stop_is_always_allowed_and_logged_once(self):
        self._post('/api/interact/arm', {'armed': False})
        before, _ = L.read(self.log)
        status, reply = self._post('/api/interact/voice', {'text': '请停止'})
        self.assertEqual((status, reply['status'], reply['command']), (200, 'accepted', 'stop'))
        new = self._new_events(len(before))
        self.assertEqual([e['event'] for e in new], ['stopped', 'interaction'])
        self.assertEqual(L.validate(new[-1]), [])

    def test_armed_start_without_console_is_rejected_with_reason(self):
        self._post('/api/interact/arm', {'armed': True})
        try:
            status, reply = self._post('/api/interact/voice', {'text': '开始'})
        finally:
            self._post('/api/interact/arm', {'armed': False})
        self.assertEqual((status, reply['status']), (200, 'rejected'))
        self.assertIn('控制台', reply['reason'])

    def test_bad_input_is_409(self):
        self.assertEqual(self._post('/api/interact/gesture', {'label': 'wave', 'confidence': 0.9})[0], 409)
        self.assertEqual(self._post('/api/interact/voice', {'text': 5})[0], 409)

    def test_old_voice_and_gesture_paths_are_gone(self):
        # 旧的 /api/voice、/api/gesture 已下线：两套识别口径不能并存
        self.assertEqual(self._post('/api/voice', {'text': '启动'})[0], 404)
        self.assertEqual(self._post('/api/gesture', {'label': 'fist'})[0], 404)

    def test_unknown_paths_still_404(self):
        status, _ = self._post('/api/voice2', {})
        self.assertEqual(status, 404)


if __name__ == '__main__':
    unittest.main()


class FlowEndpointTest(unittest.TestCase):
    """文档 32：任务页与报警红条用的 /api/flow、alarm_ack、release；不接控制台也要答得上来。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = Path(self.tmp.name) / 'log.jsonl'
        sess = ROOT / 'data' / 'flow' / f'_test_{time.time_ns()}'
        (sess / 'scan_01').mkdir(parents=True)
        self.addCleanup(lambda: __import__('shutil').rmtree(sess, ignore_errors=True))
        (sess / 'scan_01' / 'located.json').write_text(json.dumps(dict(objects=[
            dict(name='b1', kind='block', centre_m=[0.0, 0.2], **{'class': 'kitchen'}),
            dict(name='b2', kind='block', centre_m=[0.095, 0.14], **{'class': 'kitchen'}),
            dict(name='bin2', kind='bin', centre_m=[0.095, 0.14], yaw_deg=0, size_m=[0.084, 0.084, 0.042],
                 **{'class': 'kitchen'})])), encoding='utf-8')
        lines = [dict(t=1.0, event='step', session=sess.name, step='scan', state='done'),
                 dict(t=2.0, event='alarm', kind='grip_failed', summary='空爪', block='b1', needs_ack=True),
                 dict(t=3.0, event='alarm', kind='jammed', summary='卡住', needs_ack=True),
                 dict(t=4.0, event='alarm_ack', kind='jammed', by='web')]
        self.log.write_text(''.join(json.dumps(x, ensure_ascii=False) + '\n' for x in lines), encoding='utf-8')
        self.state = S.State(self.log)
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), S.Handler)
        self.server.state = self.state
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def call(self, path, body=None):
        req = urllib.request.Request(f'http://127.0.0.1:{self.port}{path}',
                                     data=json.dumps(body).encode() if body is not None else None,
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_flow_summary_lists_table_blocks_bins_and_open_alarms(self):
        status, flow = self.call('/api/flow')
        self.assertEqual(status, 200)
        self.assertEqual(flow['scan'], 'scan_01')
        self.assertEqual([(b['name'], b['in_bin']) for b in flow['blocks']], [('b1', None), ('b2', 'bin2')])
        self.assertEqual([a['kind'] for a in flow['alarms']], ['grip_failed'])     # jammed was acknowledged
        self.assertFalse(flow['process']['running'])

    def test_ack_all_and_events_keep_the_contract(self):
        self.assertEqual(self.call('/api/cmd', {'cmd': 'alarm_ack', 'kind': '*'})[0], 200)
        self.assertEqual(self.call('/api/flow')[1]['alarms'], [])
        events, _ = L.read(self.log)
        self.assertTrue(all(not L.validate(e) for e in events))

    def test_voice_cannot_start_while_an_alarm_is_open_and_release_does_not_stop(self):
        reply = self.state.start_task(source='voice')
        self.assertFalse(reply['ok'])
        self.assertIn('报警', reply['error'])
        status, body = self.call('/api/cmd', {'cmd': 'release'})
        self.assertEqual((status, body['ok']), (200, True))
        self.assertFalse(any(e['event'] == 'stopped' for e in L.read(self.log)[0]))


class StartTaskChoiceTest(unittest.TestCase):
    """「开始」默认跑已验证的旧流水线；--task flow 才跑分步编排器（文档 34 §4.2）。"""

    class _Console:
        base = 'http://127.0.0.1:8770'

        def stop(self):
            return None

    class _Proc:
        pid = 4242

        def poll(self):
            return None

    def started(self, task, pipeline_args=()):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        state = S.State(Path(tmp.name) / 'log.jsonl', console=self._Console())
        state.task, state.pipeline_args = task, list(pipeline_args)
        state.flow_out = None
        seen = {}

        def popen(cmd, **kw):
            seen['cmd'] = cmd
            kw['stdout'].close()
            return self._Proc()
        real = S.subprocess.Popen
        S.subprocess.Popen = popen
        try:
            reply = state.start_task(source='web')
        finally:
            S.subprocess.Popen = real
        return reply, seen['cmd']

    def test_the_default_start_runs_the_verified_pipeline_with_its_options(self):
        reply, cmd = self.started('pipeline', ['--bins', 'bins_acrylic_v2_pair'])
        self.assertTrue(reply['ok'])
        self.assertEqual(reply['task'], 'pipeline')
        self.assertTrue(cmd[1].endswith('src/pipeline/task.py'))
        self.assertEqual(cmd[2:4], ['--console', 'http://127.0.0.1:8770'])
        self.assertEqual(cmd[-2:], ['--bins', 'bins_acrylic_v2_pair'])
        self.assertNotIn('--really', cmd)

    def test_the_step_flow_is_still_there_when_asked_for(self):
        reply, cmd = self.started('flow')
        self.assertEqual(reply['task'], 'flow')
        self.assertTrue(cmd[1].endswith('src/flow/orchestrator.py'))
        self.assertIn('--really', cmd)
