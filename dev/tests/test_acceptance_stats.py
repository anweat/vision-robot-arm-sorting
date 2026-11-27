import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import acceptance_stats as A


class AcceptanceStatsTest(unittest.TestCase):
    def report(self, events, **kwargs):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "flow.jsonl"
            p.write_text("\n".join(json.dumps(dict(t=i, **e)) for i, e in enumerate(events)), encoding="utf-8")
            return A.analyze([p], **kwargs)

    def test_new_flow_records_do_not_require_legacy_hovering(self):
        events = [dict(event="step", step="flow", state="started", session="s"),
                  dict(event="step", step="plan", state="started", block="b1"),
                  dict(event="pipeline_sorted", cube=[0, 10], category="kitchen", bin="bin2", seconds=2),
                  dict(event="step", step="plan", state="started", block="b2"),
                  dict(event="step", step="retry", state="started", block="b2"),
                  dict(event="pipeline_missed", cube=[3, 10], seconds=3, reason="empty"),
                  dict(event="alarm", kind="grip_failed"),
                  dict(event="step", step="flow", state="done")]
        r = self.report(events, expected_blocks=4)
        self.assertEqual(r["summary"]["counts"], dict(sorted=1, missed=1))
        self.assertEqual(r["summary"]["reported_success_rate"], 0.25)
        self.assertEqual(r["summary"]["retries"], 1)
        self.assertEqual(r["rounds"][0]["unaccounted"], 2)
        self.assertFalse(r["rounds"][0]["all_reported_sorted"])
        self.assertEqual(r["summary"]["placement_unknown"], 4)
        self.assertIsNone(r["summary"]["verified_success_rate"])

    def test_stopped_round_and_restarted_round_stay_separate(self):
        e = [dict(event="step", step="flow", state="started", session="one"),
             dict(event="step", step="plan", state="started", block="b1"),
             dict(event="step", step="flow", state="started", session="two"),
             dict(event="step", step="flow", state="done")]
        r = self.report(e)
        self.assertEqual(r["summary"]["counts"], dict(unfinished=1))
        self.assertEqual(r["rounds"][0]["state"], "unfinished")
        self.assertEqual(r["summary"]["rounds_all_reported_sorted"], 0)

    def test_failed_block_is_not_success_and_verify_mismatch_blocks_full_success(self):
        e = [dict(event="step", step="flow", state="started"),
             dict(event="step", step="grasp", state="failed", block="b1", summary="disconnected"),
             dict(event="step", step="flow", state="stopped")]
        self.assertEqual(self.report(e)["summary"]["counts"], dict(failed=1))
        e[1:2] = [dict(event="step", step="plan", state="started", block="b1"),
                  dict(event="pipeline_sorted", seconds=1),
                  dict(event="step", step="verify", state="failed", result="mismatch")]
        e[-1]["state"] = "done"
        self.assertEqual(self.report(e)["summary"]["rounds_all_reported_sorted"], 0)

    def test_legacy_terminal_records_and_interruption(self):
        e = [dict(event="pipeline_start", run="r", cubes=[], registration={}),
             dict(event="pipeline_skip", cube=[0, 0], reason="no bin"),
             dict(event="hovering", cube=[1, 2]),
             dict(event="stopped", reason="operator")]
        r = self.report(e)
        self.assertEqual(r["summary"]["counts"], dict(skipped=1, unfinished=1))

    def test_independent_placement_and_duplicate_rejection(self):
        e = [dict(event="step", step="flow", state="started"),
             dict(event="step", step="plan", state="started", block="b1"),
             dict(event="pipeline_sorted", seconds=1), dict(event="step", step="flow", state="done")]
        obs = dict(round=1, block="b1", verified=True, evidence="simulation truth", target_m=[0, 0], measured_m=[0.003, 0.004])
        r = self.report(e, placements=[obs, obs])
        self.assertEqual(r["summary"]["placement_verified"], 1)
        self.assertEqual(r["rounds"][0]["blocks"][0]["placement"]["offset_mm"], 5)
        self.assertEqual(len(r["warnings"]), 1)

    def test_phase_failure_duration_and_partial_jsonl(self):
        e = [dict(event="step", step="flow", state="started"),
             dict(event="step", step="check", state="started", block="b1"),
             dict(event="step", step="check", state="failed", result="empty", block="b1"),
             dict(event="pipeline_missed", reason="empty", seconds=3),
             dict(event="step", step="flow", state="done")]
        r = self.report(e)
        self.assertEqual(r["summary"]["failure_handling_seconds"]["max"], 1)
        self.assertEqual(r["rounds"][0]["step_seconds"]["check"]["max"], 1)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "flow.jsonl"
            p.write_text(json.dumps(dict(t=0, event="pipeline_start", run="x")) + '\n{"event":', encoding="utf-8")
            self.assertEqual(len(A.analyze([p])["warnings"]), 1)
            p.write_text('{"event":"pipeline_start","t":"bad"}', encoding="utf-8")
            self.assertEqual(len(A.analyze([p])["warnings"]), 2)


if __name__ == "__main__":
    unittest.main()
