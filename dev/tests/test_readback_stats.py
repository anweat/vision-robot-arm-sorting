import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import readback_stats as R


class ReadbackStatsTest(unittest.TestCase):
    def test_directions_signed_absolute_missing_and_passing(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "execution.jsonl"
            rows = [dict(event="start", readback=[20] * 6),
                    dict(event="step", kind="move", command=[30] * 6, readback=[28] * 6),
                    dict(event="step", kind="move", command=[10] * 6, readback=[13] * 6, stop=False),
                    dict(event="step", kind="move", command=[15] * 6, readback=None, stop=False),
                    dict(event="step", kind="gripper", command=[15] * 5 + [133], readback=[15] * 5 + [129])]
            p.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
            report = R.analyze([tmp, p])
            self.assertEqual(len(report["files"]), 1)
            self.assertEqual(report["missing_readbacks"], 5)
            groups = {(r["joint"], r["direction"], r["passing"]): r for r in report["groups"]}
            self.assertEqual(groups[1, "positive", False]["signed_deg"]["median"], -2)
            self.assertEqual(groups[1, "negative", True]["absolute_deg"]["p90"], 3)
            self.assertEqual(groups[6, "positive", False]["signed_deg"]["median"], -4)

    def test_quantiles_and_pre_settle_observations(self):
        self.assertEqual(R.distribution([-5, -1, 0, 1, 9])["p90"], 9)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "execution.jsonl"
            p.write_text(json.dumps(dict(event="step", kind="move", command=[30], readback=[30], first_readback=[28])), encoding="utf-8")
            self.assertEqual(R.analyze([p], "first")["groups"][0]["signed_deg"]["median"], -2)
            self.assertEqual(R.analyze([p])["groups"][0]["signed_deg"]["median"], 0)

    def test_damaged_lines_are_reported_and_fail_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "execution.jsonl"
            p.write_text('{"event":', encoding="utf-8")
            self.assertEqual(len(R.analyze([p])["warnings"]), 1)
            self.assertEqual(R.main([str(p)]), 1)
            p.write_text('{"event":"start","readback":[NaN]}', encoding="utf-8")
            self.assertEqual(len(R.analyze([p])["warnings"]), 1)


if __name__ == "__main__":
    unittest.main()
