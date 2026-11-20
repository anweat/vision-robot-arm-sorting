"""Flow scripts v2 (docs/atlas-200i-dk-a2/35): the capability table, the checker with paths, the shared
script store, the new console actions in the orchestrator (repeat, path on step events, refusing an
unavailable capability before anything moves), and the gateway's /api/scripts endpoints.

    python3 tests/test_script.py          (the orchestrator part needs numpy: uv run --with numpy ...)
"""
import json
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "tools"))
sys.path.insert(0, str(ROOT / "src" / "web"))

from flow import script as FS                 # noqa: E402
from flow.capabilities import CAPABILITIES, catalog  # noqa: E402
import log_contract as L                      # noqa: E402

try:
    import numpy  # noqa: F401
    from flow import orchestrator as O        # noqa: E402
    from flow import steps as SP              # noqa: E402
    from flow.session import FlowSession      # noqa: E402
except ImportError:                           # the checker and the gateway parts run without numpy
    O = None


class CheckTest(unittest.TestCase):
    def test_shipped_scripts_have_no_errors(self):
        for p in sorted((ROOT / "config" / "flows").glob("*.yaml")):
            c = FS.check_text(p.read_text(encoding="utf-8"))
            self.assertTrue(c["ok"], (p.name, c["errors"]))
        c = FS.check_text((ROOT / "config" / "flows" / "ready_signal.yaml").read_text(encoding="utf-8"))
        self.assertFalse(c["runnable"])
        self.assertEqual(c["unavailable"], ["buzzer", "led", "say"])

    def test_problems_point_at_their_line(self):
        c = FS.check(dict(steps=[dict(step="scan"),
                                 dict(for_each="blocks", do=[dict(step="gripper", **{"with": dict(action="squeeze")}),
                                                             dict(step="wait", on_result=dict(done=[dict(record="nope")]))]),
                                 dict(step="speed", **{"with": dict(value=99)})]))
        paths = {e["path"]: e["message"] for e in c["errors"]}
        self.assertIn("steps[1].do[0].with.action", paths)
        self.assertIn("steps[1].do[1].on_result.done[0]", paths)
        self.assertIn("steps[2].with.value", paths)
        self.assertFalse(c["ok"])

    def test_warnings_do_not_block(self):
        c = FS.check(dict(steps=[dict(step="scan", **{"with": dict(colour=1)}, on_result=dict(teleported=[]))]))
        self.assertTrue(c["ok"])
        self.assertEqual({w["path"] for w in c["warnings"]}, {"steps[0].with.colour", "steps[0].on_result.teleported"})

    def test_required_param(self):
        c = FS.check(dict(steps=[dict(step="notify")]))
        self.assertEqual(c["errors"][0]["path"], "steps[0].with.text")

    def test_catalog_is_json_and_complete(self):
        cat = json.loads(json.dumps(catalog()))
        for name, cap in cat["capabilities"].items():
            self.assertIn(cap["status"], cat["statuses"], name)
            self.assertTrue(set(cap["success"]) <= set(cap["results"]), name)
            if cap["status"] != "ready":
                self.assertTrue(cap.get("needs"), name)

    def test_parse_errors_are_reported_not_raised(self):
        c = FS.check_text("steps:\n  - step: scan\n   with: {}\n")
        self.assertFalse(c["ok"])
        self.assertIn("解析失败", c["errors"][0]["message"])


class StoreTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        d = Path(self.tmp.name)
        (d / "builtin").mkdir()
        (d / "builtin" / "sort.yaml").write_text("name: sort\nsteps:\n  - step: home\n", encoding="utf-8")
        self.store = FS.Store(d / "builtin", d / "saved")

    def test_save_conflict_delete(self):
        with self.assertRaises(ValueError):
            self.store.save("sort", "steps:\n  - step: home\n")              # builtin is read-only
        with self.assertRaises(ValueError):
            self.store.save("Bad Name", "steps:\n  - step: home\n")
        with self.assertRaises(ValueError):
            self.store.save("x", "steps:\n  - step: dance\n")                  # errors are not saved
        got = self.store.save("x", "title: X\nsteps:\n  - step: home\n")
        self.assertEqual(got["check"]["uses"], ["home"])
        self.store.save("x", "title: X2\nsteps:\n  - step: home\n", base_rev=got["rev"])
        with self.assertRaises(FS.ConflictError) as cm:                      # someone saved in between
            self.store.save("x", "title: X3\nsteps:\n  - step: home\n", base_rev=got["rev"])
        self.assertIn("X2", cm.exception.current["text"])
        names = [(s["name"], s["builtin"], s["title"]) for s in self.store.list()]
        self.assertEqual(names, [("sort", True, "sort"), ("x", False, "X2")])
        self.assertTrue(self.store.delete("x"))
        self.assertFalse(self.store.delete("x"))
        with self.assertRaises(ValueError):
            self.store.delete("sort")


class Arm:
    """Enough of a console for the plain actions."""

    def __init__(self, gestures=()):
        self.calls, self.gestures = [], list(gestures)

    def goto(self, named):
        self.calls.append(("goto", named))

    def gripper(self, action, j6):
        self.calls.append(("gripper", action))

    def grip_result(self):
        return "empty"

    def speed(self, v):
        self.calls.append(("speed", v))

    def interact(self):
        return dict(recent=[dict(id=i + 1, source="gesture", input=g, status="rejected") for i, g in enumerate(self.gestures)])


@unittest.skipIf(O is None, "needs numpy (the orchestrator imports the planner)")
class RunTest(unittest.TestCase):
    def run_script(self, script, arm):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        events = []
        ctx = SP.Context(sess=FlowSession(Path(tmp.name) / "s"), backend=lambda: arm,
                         sink=lambda e, **f: events.append(dict(event=e, **f)))
        O.validate(script)
        return O.Runner(ctx, script).run(), events

    def test_every_capability_has_an_implementation(self):
        self.assertEqual(set(CAPABILITIES), set(O.STEPS))

    def test_gripper_check_runs_and_steps_carry_paths(self):
        script = O.load_script(ROOT / "config" / "flows" / "gripper_check.yaml")
        arm = Arm()
        out, events = self.run_script(script, arm)
        self.assertEqual(out["state"], "done", out)
        self.assertEqual(arm.calls, [("goto", "observe_v1"), ("gripper", "open"), ("gripper", "grip"), ("gripper", "open")])
        done = [(e["path"], e["result"]) for e in events if e["event"] == "step" and e["state"] != "started" and e["step"] != "flow"]
        self.assertEqual(done[3], ("steps[3]", "empty"))
        for e in events:
            if e["event"] in L.EVENTS:
                self.assertEqual(L.validate(dict(t=1.0, **{k: v for k, v in e.items() if v is not None})), [], e)

    def test_repeat_and_wait_gesture(self):
        script = dict(steps=[dict(repeat=dict(times=3, do=[dict(step="speed", **{"with": dict(value=10)})])),
                             dict(step="wait_gesture", **{"with": dict(label="ok", timeout_s=1)},
                                  on_result=dict(timeout=[dict(stop="没等到")]))])
        arm = Arm(gestures=["palm"])
        out, events = self.run_script(script, arm)
        self.assertEqual(out["state"], "stopped")
        self.assertEqual(out["reason"], "没等到")
        self.assertEqual(arm.calls, [("speed", 10)] * 3)
        paths = [e["path"] for e in events if e["event"] == "step" and e["step"] == "speed" and e["state"] == "done"]
        self.assertEqual(paths, ["steps[0].repeat.do[0]"] * 3)

    def test_unavailable_capability_is_refused_before_anything_moves(self):
        script = dict(steps=[dict(step="gripper", **{"with": dict(action="open")}), dict(step="led", **{"with": dict(color="red")})])
        arm = Arm()
        out, _ = self.run_script(script, arm)
        self.assertEqual(out["state"], "stopped")
        self.assertIn("led", out["reason"])
        self.assertEqual(arm.calls, [])


class GatewayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import serve as S
        cls.S = S
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cls.log = d / "log.jsonl"
        cls.log.write_text("", encoding="utf-8")
        cls.state = S.State(cls.log)
        cls.state.scripts = FS.Store(ROOT / "config" / "flows", d / "saved")
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), S.Handler)
        cls.server.state = cls.state
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.tmp.cleanup()

    def call(self, path, body=None):
        url = f"http://127.0.0.1:{self.port}{path}"
        req = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                                     method="GET" if body is None else "POST",
                                     headers={"Content-Type": "application/json", "Origin": f"http://127.0.0.1:{self.port}"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    def test_list_get_check(self):
        status, body = self.call("/api/scripts")
        self.assertEqual(status, 200)
        names = {s["name"]: s for s in body["scripts"]}
        self.assertTrue({"sort", "gesture_scan", "gripper_check", "ready_signal"} <= set(names))
        self.assertFalse(names["ready_signal"]["runnable"])
        status, cat = self.call("/api/scripts/capabilities")
        self.assertIn("wait_gesture", cat["capabilities"])
        status, got = self.call("/api/scripts/gripper_check")
        self.assertEqual(got["check"]["uses"], ["gripper", "notify", "pose", "wait"])
        self.assertNotIn("path", got)
        self.assertEqual(self.call("/api/scripts/nope")[0], 404)
        status, c = self.call("/api/scripts/check", dict(text="steps:\n  - step: dance\n"))
        self.assertEqual((status, c["ok"], c["errors"][0]["path"]), (200, False, "steps[0]"))

    def test_save_writes_an_event_and_conflicts(self):
        text = "title: 测试\nsteps:\n  - step: home\n"
        status, got = self.call("/api/scripts/save", dict(name="t1", text=text))
        self.assertEqual(status, 200, got)
        events, _ = L.read(self.log)
        changed = [e for e in events if e["event"] == "script_changed"]
        self.assertEqual((changed[-1]["name"], changed[-1]["action"], changed[-1]["by"]), ("t1", "saved", "web"))
        self.assertEqual(L.validate(changed[-1]), [])
        self.call("/api/scripts/save", dict(name="t1", text=text + "  - step: home\n", rev=got["rev"]))
        status, body = self.call("/api/scripts/save", dict(name="t1", text=text, rev=got["rev"]))
        self.assertEqual(status, 409)
        self.assertIn("current", body)
        self.assertEqual(self.call("/api/scripts/save", dict(name="sort", text=text))[0], 400)
        self.assertEqual(self.call("/api/scripts/delete", dict(name="t1"))[0], 200)

    def test_run_script_refuses_before_spawning(self):
        status, body = self.call("/api/cmd", dict(cmd="run_script", name="ready_signal"))
        self.assertEqual(status, 409)
        self.assertIn("还不可用", body["error"])
        status, body = self.call("/api/cmd", dict(cmd="run_script", name="gripper_check"))
        self.assertEqual(status, 409)                       # no console in this test
        self.assertIn("未接控制台", body["error"])


if __name__ == "__main__":
    unittest.main()
