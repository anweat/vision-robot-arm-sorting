"""HTTP recognition results -> gateway rules -> real local simulator, no microphones/cameras."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_gateway_integration as G


class InteractionGatewayTest(G.GatewayIntegrationHarness):
    group_name = "interaction"

    def call(self, kind, body):
        status, result = G.request(self.url, f"/api/interact/{kind}", body)
        self.assertEqual(status, 200, result)
        return result

    def frames(self, label, confidence=0.99, n=3):
        return [self.call("gesture", dict(label=label, confidence=confidence)) for _ in range(n)]

    def test_unarmed_rejection_voice_mapping_and_stop_priority(self):
        self.assertEqual(self.call("voice", dict(text="请开始"))["status"], "rejected")
        self.assertIsNone(self.state.flow)
        # Stop works without arming, and wins when another command word occurs in the same sentence.
        result = self.call("voice", dict(text="先开始然后停止"))
        self.assertEqual((result["status"], result["command"]), ("accepted", "stop"))
        self.assertEqual(self.call("voice", dict(text="今天天气不错"))["status"], "ignored")
        self.assertTrue(any(e.get("event") == "stopped" for e in self.events()))

    def test_gesture_threshold_frames_latch_and_voice_cooldown(self):
        self.call("arm", dict(armed=True))
        self.assertTrue(all(r["status"] == "ignored" for r in self.frames("like", 0.2)))
        frames = self.frames("like")
        self.assertEqual([r["status"] for r in frames], ["ignored", "ignored", "accepted"])
        self.assertEqual(self.frames("like", n=1)[0]["status"], "ignored")
        self.assertEqual(self.call("voice", dict(text="张开"))["status"], "ignored")  # same command shares cooldown across sources
        self.state.interaction.clock = lambda: G.time.time() + 3
        self.assertEqual(self.call("voice", dict(text="张开"))["status"], "accepted")
        self.assertEqual(self.call("voice", dict(text="张开"))["status"], "ignored")
        events = [e for e in self.events() if e.get("event") == "interaction"]
        self.assertEqual(sum(e.get("source") == "gesture" for e in events), 1)
        self.assertEqual(sum(e.get("source") == "voice" for e in events), 1)

    def test_alarms_block_start_until_ack_and_unavailable_is_refused(self):
        # A stationary script proves the start route without relying on object perception.
        self.state.scripts.save("sort", "steps:\n  - step: wait\n    with: {seconds: 0.2}\n")
        self.state.task = "flow"
        patcher = G.patch.object(G.S, "FLOW_SCRIPT", self.state.scripts.get("sort")["path"])
        patcher.start()
        self.addCleanup(patcher.stop)
        self.state.append(dict(t=G.time.time(), event="alarm", kind="check", summary="test", needs_ack=True))
        self.call("arm", dict(armed=True))
        self.assertEqual(self.call("voice", dict(text="开始"))["status"], "rejected")
        self.assertIsNone(self.state.flow)
        self.cmd("alarm_ack", kind="*")
        self.state.interaction.last_fire.clear()
        self.assertEqual(self.frames("ok")[-1]["status"], "accepted")
        self.assertEqual(self.wait_done()["exit_code"], 0)
        before = self.state.flow.pid
        for step, params in (("wait_voice", dict(words="开始")), ("say", dict(text="离线测试"))):
            name = "offline-" + step.replace("_", "-")
            self.save(dict(steps=[dict(step=step, **{"with": params})]), name)
            status, _ = self.cmd("run_script", name=name)
            self.assertEqual(status, 409)
            self.assertEqual(self.state.flow.pid, before)
