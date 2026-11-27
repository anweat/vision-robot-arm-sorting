import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_gateway_integration as G
import bench


class ScenarioRegressionTest(G.GatewayIntegrationHarness):
    group_name = "scenarios"

    def test_empty_table_stops_without_sorting(self):
        source = self.folder / "empty.json"
        source.write_text(json.dumps(self.sim.located()), encoding="utf-8")
        script = dict(steps=[dict(step="scan", **{"with": dict(source=str(source))}, on_result={"empty": [dict(stop="empty table")]})])
        self.cmd("run_script", name=self.save(script))
        self.assertNotEqual(self.wait_done()["exit_code"], 0)
        self.assertFalse(any(e.get("event") == "pipeline_sorted" for e in self.events()))
        self.assertFalse(list((self.folder / "flow").rglob("execution.jsonl")))

    def test_negative_plan_and_faults_in_both_motion_modes(self):
        for motion in ("steps", "continuous"):
            with self.subTest(motion=motion):
                result = bench.run("offline_edges", sets={"execute.motion": motion}, console=self.sim_url, out=self.folder / motion)
                self.assertTrue(result["ok"], result)
                report = json.loads((self.folder / motion / "report.json").read_text(encoding="utf-8"))
                self.assertFalse(report["cases"][0]["planned"])
                self.assertFalse((self.folder / motion / "unreachable/execution.jsonl").exists())
                self.assertTrue(report["cases"][1]["truth"]["inside"])

    def test_vertical_reports_unstack_and_drop_contacts_as_failures(self):
        result = bench.run("vertical", sets={"fault.stop_short_deg": 0}, console=self.sim_url, out=self.folder / "vertical")
        report = json.loads((self.folder / "vertical/report.json").read_text(encoding="utf-8"))
        cases = {c["case"]: c for c in report["cases"]}
        for name in ("stack", "stack2", "bin_set"):
            self.assertTrue(cases[name]["ok"], cases[name])
        # Known planner/model issue, retained explicitly: a completed execution must still FAIL on contacts.
        # If these contacts are fixed, replace this reproducer with a successful zero-contact assertion.
        for name in ("unstack", "bin_drop"):
            self.assertGreater(cases[name]["truth"]["block_contacts"], 0, cases[name])
            self.assertTrue(cases[name]["contact_records"])
            self.assertFalse(cases[name]["ok"])
        self.assertFalse(result["ok"])
