"""Actual loopback HTTP gateway -> orchestrator child -> simulator; never accepts a hardware URL.

Run: python -m unittest discover -s dev/tests -p test_gateway_integration.py -v
OFFLINE_ACCEPTANCE_OUT optionally retains isolated logs/sessions instead of deleting them.
"""
import copy
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for sub in ("app/src/web", "app/src/tools", "app/src", "dev/sim", "dev/tools"):
    sys.path.insert(0, str(ROOT / sub))
import yaml
import serve as S
import harness as H
import log_contract as L
import acceptance_stats as A
import readback_stats as RB


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def request(base, path, body=None):
    req = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        with exc:
            return exc.code, json.loads(exc.read())


def until(fn, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = fn()
        if result:
            return result
        time.sleep(0.1)
    raise AssertionError(f"condition did not become true in {timeout}s")


class QuietHandler(S.Handler):
    def log_message(self, *args):
        pass


class GatewayIntegrationHarness(unittest.TestCase):
    group_name = "gateway"
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        if os.environ.get("OFFLINE_ACCEPTANCE_OUT"):
            cls.root = Path(os.environ["OFFLINE_ACCEPTANCE_OUT"]).resolve() / cls.group_name
            cls.root.mkdir(parents=True, exist_ok=False)
        cls.sim_url = f"http://127.0.0.1:{free_port()}"
        cls.sim_output = (cls.root / "simulator.out").open("w", encoding="utf-8")
        cls.process = subprocess.Popen([sys.executable, str(ROOT / "dev/sim/sim_server.py"),
            "--port", cls.sim_url.rsplit(":", 1)[1], "--blocks", "0", "--speed", "100",
            "--map", "map_a4_v4", "--bins", "bins_acrylic_v2_pair", "--output", str(cls.root / "sim")],
            cwd=ROOT, stdout=cls.sim_output, stderr=subprocess.STDOUT,
            env=dict(os.environ, PYTHONUTF8="1"))
        cls.addClassCleanup(cls.cleanup)
        def ready():
            if cls.process.poll() is not None:
                raise AssertionError((cls.root / "simulator.out").read_text(encoding="utf-8"))
            try:
                return request(cls.sim_url, "/api/bootstrap")[1]
            except OSError:
                return None
        until(ready, 60)
        cls.sim = H.Sim(cls.sim_url)  # refuses anything whose mode is not simulation

    @classmethod
    def cleanup(cls):
        if cls.process.poll() is None:
            cls.process.terminate()
            try:
                cls.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                cls.process.kill()
                cls.process.wait(timeout=10)
        cls.sim_output.close()
        cls.temp.cleanup()

    def setUp(self):
        self.folder = self.root / self._testMethodName
        self.folder.mkdir()
        self.state = S.State(self.folder / "log.jsonl", console=S.Console(self.sim_url), flow_root=self.folder / "flow")
        self.state.scripts = S.FS.Store(self.folder / "builtin", self.folder / "scripts")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), QuietHandler)
        self.server.daemon_threads = True
        self.server.state = self.state
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.state.base_url = self.url
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close_gateway)
        self.sim.reset(17)
        self.sim.bins("bins_acrylic_v2_pair")
        self.sim.faults()
        # Put the simulator at the same initial pose used by the flow planner.
        self.sim.post("/api/goto", dict(owner="setup", named="observe_v1"))
        self.sim.post("/api/stop", dict(owner="setup"))
        tuning = json.loads((ROOT / "app/config/planning_tuning.json").read_text(encoding="utf-8"))
        tuning["trims"].update(self.sim.table_trim()["trims"])
        self.tuning = self.folder / "tuning.json"
        self.tuning.write_text(json.dumps(tuning), encoding="utf-8")
        self.environment = patch.dict(os.environ, ARM_PLANNING_TUNING=str(self.tuning), PYTHONUTF8="1")
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def close_gateway(self):
        self.state.stop_flow()
        self.state.arm_off()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)

    def cmd(self, cmd, **body):
        return request(self.url, "/api/cmd", dict(cmd=cmd, source="web", **body))

    def save(self, script, name="offline"):
        status, result = request(self.url, "/api/scripts/save", dict(name=name, text=yaml.safe_dump(script, allow_unicode=True, sort_keys=False)))
        self.assertEqual(status, 200, result)
        return name

    def wait_done(self):
        until(lambda: self.state.flow is not None and self.state.flow.poll() is not None, 180)
        _, status = request(self.url, "/api/status")
        return status["flow"]

    def events(self):
        _, page = request(self.url, "/api/log?limit=2000")
        return page["events"]

    def source_script(self, category="kitchen", xy=(0, 13)):
        bid = self.sim.block(category, xy, 5)
        source = self.folder / "located.json"
        source.write_text(json.dumps(self.sim.located("bins_acrylic_v2_pair")), encoding="utf-8")
        script = yaml.safe_load((ROOT / "app/config/flows/sort.yaml").read_text(encoding="utf-8"))
        from flow.orchestrator import _with_source
        _with_source(script["steps"], str(source))
        return bid, source, script

class GatewayIntegrationTest(GatewayIntegrationHarness):
    def test_full_round_scan_to_truth_records_and_reports(self):
        bid = self.sim.block("kitchen", [0, 21], 5, up="fish_bone")
        script = yaml.safe_load((ROOT / "app/config/flows/sort.yaml").read_text(encoding="utf-8"))
        script["steps"][0]["with"]["refine"] = False
        script["steps"][0]["with"]["min_views"] = 3
        name = self.save(script)
        self.assertEqual(self.cmd("run_script", name=name)[0], 200)
        result = self.wait_done()
        output = self.state.flow_out.read_text(encoding="utf-8", errors="replace")
        self.assertEqual(result["exit_code"], 0, output)
        events = self.events()
        self.assertEqual([L.validate(e) for e in events], [[] for _ in events])
        _, records = request(self.url, "/api/log/records")
        _, summary = request(self.url, "/api/log/summary")
        self.assertEqual(summary["records"], len(records["records"]))
        self.assertEqual(summary["by_result"]["sorted"], 1, output)
        self.assertEqual(summary["by_result"]["missed"], 0)
        truth = self.sim.world()
        block = self.sim.block_truth(bid, truth)
        self.assertTrue(block["inside"], block)
        self.assertFalse(block["held"])
        self.assertEqual(self.sim.counts(truth), dict(contacts=0, block_contacts=0, platform_contacts=0))
        self.assertTrue(any(e.get("step") == "verify" and e.get("result") == "ok" for e in events), output)
        report = A.analyze([self.state.log_path], expected_blocks=1)
        self.assertEqual(report["summary"]["counts"], dict(sorted=1))
        self.assertEqual(report["summary"]["placement_unknown"], 1)
        RB.write_json(self.folder / "truth.json", truth)
        RB.write_json(self.folder / "acceptance.json", report)
        stats = RB.analyze([self.folder / "flow"])
        self.assertTrue(stats["groups"])
        RB.write_json(self.folder / "readback.json", stats)

    def test_grip_miss_retries_then_records_miss_and_alarm(self):
        _, _, script = self.source_script()
        self.sim.faults(grip_miss=1)
        name = self.save(script)
        self.assertEqual(self.cmd("run_script", name=name)[0], 200)
        self.wait_done()
        report = A.analyze([self.state.log_path])
        self.assertGreaterEqual(report["summary"]["retries"], 1)
        self.assertEqual(report["summary"]["counts"].get("sorted", 0), 0)
        self.assertEqual(report["summary"]["counts"].get("missed"), 1, self.events())
        self.assertTrue(any(e.get("event") == "alarm" and e.get("kind") == "grip_failed" for e in self.events()))
        _, flow = request(self.url, "/api/flow")
        self.assertTrue(flow["alarms"])
        self.assertEqual(self.cmd("alarm_ack", kind="*")[0], 200)
        self.assertEqual(request(self.url, "/api/flow")[1]["alarms"], [])

    def test_missing_bin_is_skipped_without_execution(self):
        _, _, script = self.source_script("other")
        name = self.save(script)
        self.cmd("run_script", name=name)
        self.wait_done()
        self.assertEqual(A.analyze([self.state.log_path])["summary"]["counts"], dict(skipped=1))
        self.assertFalse(list((self.folder / "flow").rglob("execution.jsonl")))

    def test_unreachable_plan_is_rejected_without_execution(self):
        _, source, script = self.source_script()
        scene = json.loads(source.read_text(encoding="utf-8"))
        scene["objects"][0]["centre_m"] = [0, 1.2]
        source.write_text(json.dumps(scene), encoding="utf-8")
        self.cmd("run_script", name=self.save(script))
        self.wait_done()
        self.assertTrue(any(e.get("step") == "plan" and e.get("result") == "rejected" for e in self.events()))
        self.assertEqual(A.analyze([self.state.log_path])["summary"]["counts"], dict(skipped=1))
        self.assertFalse(list((self.folder / "flow").rglob("execution.jsonl")))

    def test_console_disconnection_fails_and_never_sorts(self):
        self.state.console = S.Console(f"http://127.0.0.1:{free_port()}", timeout=0.2)
        self.cmd("run_script", name=self.save(dict(steps=[dict(step="home")])))
        result = self.wait_done()
        self.assertNotEqual(result["exit_code"], 0)
        self.assertTrue(any(e.get("event") == "stopped" for e in self.events()))
        self.assertEqual(A.analyze([self.state.log_path])["summary"]["counts"].get("sorted", 0), 0)

    def test_concurrent_start_and_stop_leave_no_running_child(self):
        name = self.save(dict(steps=[dict(step="wait", **{"with": dict(seconds=30)}),
                                    dict(step="notify", **{"with": dict(text="must not run")})]))
        with ThreadPoolExecutor(max_workers=2) as pool:
            replies = list(pool.map(lambda _: self.cmd("run_script", name=name), range(2)))
        self.assertEqual(sorted(r[0] for r in replies), [200, 409])
        until(lambda: any(e.get("step") == "wait" and e.get("state") == "started" for e in self.events()))
        child = self.state.flow
        self.assertEqual(self.cmd("stop")[0], 200)
        self.assertIsNotNone(child.poll())
        self.assertFalse(request(self.url, "/api/status")[1]["flow"]["running"])
        self.assertFalse(any(e.get("step") == "notify" for e in self.events()))
        report = A.analyze([self.state.log_path])
        self.assertEqual(report["rounds"][0]["state"], "unfinished")
        self.assertEqual(report["summary"]["rounds_all_reported_sorted"], 0)


if __name__ == "__main__":
    unittest.main()
