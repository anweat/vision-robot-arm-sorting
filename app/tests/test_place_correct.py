"""Capability -> YAML -> steps -> execution -> actual correction log, in both motion modes."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from flow import orchestrator as O, stages as ST, steps as SP
from flow.session import FlowSession
from flow.capabilities import CAPABILITIES
from test_orchestrator import LOCATED


class PlaceCorrectTest(unittest.TestCase):
    def test_switch_reaches_execution_in_both_motion_modes(self):
        for motion in ("steps", "continuous"):
            for enabled in (False, True):
                with self.subTest(motion=motion, correct=enabled), tempfile.TemporaryDirectory() as tmp:
                    source = Path(tmp) / "located.json"
                    source.write_text(json.dumps(LOCATED), encoding="utf-8")
                    arm = ST.DryRun(ST.OBSERVE + [30])
                    ctx = SP.Context(FlowSession(Path(tmp) / "session"), backend=lambda: arm)
                    script = dict(motion=motion, steps=[dict(step="scan", **{"with": dict(source=str(source))}),
                        dict(for_each="blocks", do=[dict(step="plan"), dict(step="grasp"),
                             dict(step="place", **{"with": dict(correct=enabled)})])])
                    O.validate(script)
                    result = O.Runner(ctx, script).run()
                    self.assertEqual(result["state"], "done")
                    execution = Path(tmp) / "session" / "blocks" / "b1" / "execution.jsonl"
                    rows = [json.loads(s) for s in execution.read_text(encoding="utf-8").splitlines()]
                    self.assertEqual(any("correction" in r for r in rows), enabled)

    def test_default_and_type_validation(self):
        self.assertIs(CAPABILITIES["place"]["params"]["correct"]["default"], True)
        with self.assertRaises(ValueError):
            O.validate(dict(steps=[dict(for_each="blocks", do=[dict(step="place", **{"with": dict(correct="false")})])]))


if __name__ == "__main__":
    unittest.main()
