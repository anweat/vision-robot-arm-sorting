"""src/flow/orchestrator.py + steps.py + miniyaml.py: the YAML flow, offline (dry-run backend, scenes from file)."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "tools"))

from flow import miniyaml, orchestrator as O, stages as ST, steps as SP  # noqa: E402
from flow.session import FlowSession  # noqa: E402

LOCATED = dict(objects=[
    dict(id=1, name="b1", kind="block", **{"class": "kitchen"}, centre_m=[0.0, 0.20], yaw_deg=5.0, size_m=[0.03] * 3,
         sigma_xy_m=0.005, map_cm=[0.0, 10.0]),
    dict(id=2, name="b2", kind="block", **{"class": "other"}, centre_m=[0.07, 0.24], yaw_deg=0.0, size_m=[0.03] * 3,
         sigma_xy_m=0.005, map_cm=[7.0, 14.0]),
    dict(id=3, name="bin2", kind="bin", **{"class": "kitchen"}, design="bins_acrylic_v2_pair", centre_m=[0.095, 0.14],
         yaw_deg=0.0, size_m=[0.084, 0.084, 0.042], sigma_xy_m=0.01),
])


class Arm(ST.DryRun):
    """Dry run whose grip reads what the test says, and whose J6 can drift during the lift."""

    def __init__(self, grips=("holding",), slip=0):
        super().__init__(ST.OBSERVE + [30])
        self.grips, self.slip, self.closed = list(grips), slip, False

    def gripper(self, action, j6):
        self.closed = j6 > 100
        return super().gripper(action, j6)

    def grip_result(self):
        return self.grips.pop(0) if len(self.grips) > 1 else self.grips[0]

    def move(self, cmd5):
        super().move(cmd5)
        if self.closed and self.slip:
            self.pose[5] += self.slip


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.located = self.dir / "located.json"
        self.located.write_text(json.dumps(LOCATED))
        self.events = []

    def run_flow(self, arm, path=ROOT / "config" / "flows" / "sort.yaml"):
        script = O.load_script(path)
        O._with_source(script["steps"], str(self.located))
        sess = FlowSession(self.dir / "s")
        ctx = SP.Context(sess=sess, backend=lambda: arm, sink=lambda e, **f: self.events.append(dict(event=e, **f)))
        return O.Runner(ctx, script).run(), ctx

    def names(self, event):
        return [e for e in self.events if e["event"] == event]


class ScriptTest(unittest.TestCase):
    def test_scripts_load_and_match_pyyaml(self):
        for name in ("sort.yaml", "retry.yaml"):
            text = (ROOT / "config" / "flows" / name).read_text(encoding="utf-8")
            parsed = miniyaml.parse(text)
            O.validate(parsed)
            try:
                import yaml
            except ImportError:
                continue
            self.assertEqual(parsed, yaml.safe_load(text), name)

    def test_miniyaml_subset(self):
        self.assertEqual(miniyaml.parse("a: 1\nb: [x, 'y z', {k: 2.5, n: null}]\nc:\n  - true\n  - - 1\n"),
                         dict(a=1, b=["x", "y z", dict(k=2.5, n=None)], c=[True, [1]]))
        self.assertEqual(miniyaml.parse("- step: a  # comment\n  with: {x: 1}\n- step: b\n"),
                         [dict(step="a", **{"with": dict(x=1)}), dict(step="b")])
        for bad in ("a: &x 1\n", "a: |\n  text\n", "a: 1\n   b: 2\n", "a: 1\na: 2\n"):
            with self.assertRaises(ValueError, msg=bad):
                miniyaml.parse(bad)

    def test_validation_refuses_unknown_keys_and_misplaced_actions(self):
        bad = [dict(steps=[dict(step="scan", wiht={})]),
               dict(steps=[dict(step="dance")]),
               dict(steps=[dict(record="sorted")]),
               dict(steps=[dict(alarm="x", then="skip_block")]),
               dict(steps=[dict(retry=dict(times=0, do=[]))]),
               dict(steps=[dict(step="scan")], extra=1),
               dict(steps=[dict(for_each="blocks", do=[dict(for_each="blocks", do=[])])])]
        for script in bad:
            with self.assertRaises(ValueError, msg=script):
                O.validate(script)


class FlowRunTest(Base):
    def test_full_round_sorts_what_has_a_bin_and_skips_the_rest(self):
        out, ctx = self.run_flow(Arm())
        self.assertEqual(out["state"], "done", out)
        self.assertEqual((out["sorted"], out["skipped"], out["missed"], out["alarms"]), (1, 1, 0, 0))
        sorted_ = self.names("pipeline_sorted")
        self.assertEqual((sorted_[0]["cube"], sorted_[0]["category"], sorted_[0]["bin"]), ([0.0, 10.0], "kitchen", "bin2"))
        self.assertIn("没有对应的桶", self.names("pipeline_skip")[0]["reason"])
        steps = [(e["step"], e["state"]) for e in self.names("step") if e["state"] != "started"]
        self.assertEqual(steps[:6], [("scan", "done"), ("plan", "done"), ("grasp", "done"), ("check", "done"),
                                     ("place", "done"), ("plan", "failed")])
        # the source scene does not change, so the round's verify sees b1 still there: reported, not a crash
        self.assertEqual(steps[-2], ("verify", "failed"))
        self.assertEqual(steps[-1], ("flow", "done"))

    def test_empty_grip_retries_then_alarms_and_records_a_miss(self):
        out, _ = self.run_flow(Arm(grips=("empty",)))
        self.assertEqual(out["state"], "done", out)
        self.assertEqual((out["sorted"], out["missed"]), (0, 1))
        retries = [e for e in self.names("step") if e["step"] == "retry"]
        self.assertEqual(len(retries), 2)
        alarm = self.names("alarm")[0]
        self.assertEqual((alarm["kind"], alarm["needs_ack"], alarm["block"]), ("grip_failed", True, "b1"))
        self.assertIn("重试", alarm["summary"])
        self.assertIn("重试", self.names("pipeline_missed")[0]["reason"])

    def test_second_attempt_holds_and_the_block_is_placed(self):
        out, _ = self.run_flow(Arm(grips=("empty", "holding")))
        self.assertEqual((out["sorted"], out["missed"], out["alarms"]), (1, 0, 0))
        self.assertEqual(len([e for e in self.names("step") if e["step"] == "retry"]), 1)

    def test_jam_alarms_and_skips(self):
        out, _ = self.run_flow(Arm(grips=("not_closed",)))
        self.assertEqual(self.names("alarm")[0]["kind"], "jammed")
        self.assertEqual(out["sorted"], 0)

    def test_slip_during_lift_is_caught(self):
        sess = FlowSession(self.dir / "slip")
        ctx = SP.Context(sess=sess, backend=lambda: arm)
        arm = Arm(slip=5)
        SP.scan(ctx, source=str(self.located))
        ctx.block = dict(SP.blocks(ctx.scene)[0])
        self.assertEqual(SP.plan(ctx).result, "ok")
        self.assertEqual(SP.grasp(ctx).result, "done")
        self.assertEqual(SP.check(ctx).result, "slipped")

    def test_contract_events_are_valid(self):
        import log_contract as L
        self.run_flow(Arm(grips=("empty",)))
        for e in self.events:
            self.assertEqual(L.validate(dict(t=1.0, **{k: v for k, v in e.items() if v is not None})), [], e)

    def test_scan_without_really_and_without_source_refuses(self):
        ctx = SP.Context(sess=FlowSession(self.dir / "x"))
        with self.assertRaises(RuntimeError):
            SP.scan(ctx)


class StandaloneStepTest(Base):
    def test_steps_run_alone_from_the_session_files(self):
        s = str(self.dir / "alone")
        self.assertEqual(O.main(["--step", "scan", "--session", s, "--source", str(self.located)]), 0)
        self.assertEqual(O.main(["--step", "plan", "--session", s, "--block", "b1"]), 0)
        self.assertEqual(O.main(["--step", "plan", "--session", s, "--block", "b2"]), 1)   # no bin for "other"
        self.assertEqual(O.main(["--step", "grasp", "--session", s, "--block", "b1"]), 0)  # dry run
        self.assertEqual(O.main(["--step", "check", "--session", s, "--block", "b1"]), 0)
        self.assertTrue((Path(s) / "blocks" / "b1" / "grasp.json").exists())

    def test_check_only_validates(self):
        self.assertEqual(O.main(["--script", str(ROOT / "config" / "flows" / "sort.yaml"), "--check"]), 0)


if __name__ == "__main__":
    unittest.main()
