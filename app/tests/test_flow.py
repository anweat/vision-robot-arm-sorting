"""src/flow: the stages after perception, offline (map -> plan -> dry-run execute -> verify). No console."""

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flow import stages as ST  # noqa: E402
from flow.session import FlowSession  # noqa: E402

LOCATED = dict(objects=[
    dict(id=1, name="b1", kind="block", **{"class": "kitchen"}, centre_m=[0.0, 0.20], yaw_deg=5.0, size_m=[0.03] * 3,
         sigma_xy_m=0.005),
    dict(id=2, name="b2", kind="block", **{"class": "other"}, centre_m=[0.07, 0.24], yaw_deg=0.0, size_m=[0.03] * 3,
         sigma_xy_m=0.005),
    dict(id=3, name="bin2", kind="bin", **{"class": "kitchen"}, design="bins_acrylic_v2_pair", centre_m=[0.095, 0.14],
         yaw_deg=0.0, size_m=[0.084, 0.084, 0.042], sigma_xy_m=0.01),
])


class FlowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.sess = FlowSession(Path(self.tmp.name) / "s")
        self.sess.save("located.json", LOCATED)
        ST.build_map(self.sess)

    def tearDown(self):
        self.tmp.cleanup()

    def test_table_task_end_to_end(self):
        p = ST.plan(self.sess, dict(pick="b1", to={"table_m": [-0.05, 0.20]}))
        self.assertTrue(p["ok"], p["reasons"])
        self.assertEqual([q["kind"] for q in p["plans"]], ["pick", "place", "transit"])
        self.assertEqual(p["steps"][-1]["joints"][:5], ST.OBSERVE)
        r = ST.execute(self.sess)                                  # dry run: nothing moves
        self.assertTrue(r["ok"])
        self.assertEqual(r["backend"], "dry-run")
        rows = [json.loads(line) for line in self.sess.file("execution.jsonl").read_text().splitlines()]
        self.assertEqual(sum(row["event"] == "step" for row in rows), len(p["steps"]))
        after = copy.deepcopy(LOCATED)
        after["objects"][0]["centre_m"] = [v / 100 for v in p["plans"][1]["summary"]["target_base_cm"]]
        self.assertTrue(ST.verify(self.sess, after)["ok"])
        self.assertFalse(ST.verify(self.sess, LOCATED)["ok"])     # nothing moved

    def test_bin_task(self):
        p = ST.plan(self.sess, dict(pick="b1", to={"bin": "bin2"}))
        self.assertTrue(p["ok"], p["reasons"])
        self.assertEqual(p["plans"][1]["summary"]["into"], "bin")
        after = copy.deepcopy(LOCATED)
        after["objects"] = after["objects"][1:]
        self.assertTrue(ST.verify(self.sess, after)["ok"])

    def test_execute_refuses_a_failed_plan_or_a_moved_arm(self):
        ST.plan(self.sess, dict(pick="b1", to={"table_m": [0.0, 0.05]}))        # inside the platform
        with self.assertRaises(RuntimeError):
            ST.execute(self.sess)
        ST.plan(self.sess, dict(pick="b1", to={"table_m": [-0.05, 0.20]}))
        with self.assertRaises(RuntimeError):
            ST.execute(self.sess, backend=ST.DryRun([60, 90, 30, 30, 92, 30]))

    def test_a_dry_run_can_stop_part_way(self):
        ST.plan(self.sess, dict(pick="b1", to={"table_m": [-0.05, 0.20]}))
        r = ST.execute(self.sess, stop_after=3)
        self.assertFalse(r["ok"])
        self.assertEqual(r["done"], 3)

    def test_settle_brings_a_short_servo_onto_the_command(self):
        p = ST.plan(self.sess, dict(pick="b1", to={"table_m": [-0.05, 0.20]}))

        def short(prev, cmd):                       # J2 stops 2 deg short whenever it is raised (teleop readbacks)
            out = list(cmd)
            if cmd[1] > prev[1] + 0.5:
                out[1] = max(prev[1], cmd[1] - 2.0)
            return out

        r = ST.execute(self.sess, backend=ST.DryRun(list(ST.OBSERVE) + [30], tracking=short))
        self.assertTrue(r["ok"], r)
        rows = [json.loads(line) for line in self.sess.file("execution.jsonl").read_text().splitlines()]
        moves = [row for row in rows if row["event"] == "step" and row["kind"] == "move"]
        self.assertTrue(any(row.get("settle") for row in moves))
        for row in moves:
            self.assertLess(max(abs(a - b) for a, b in zip(row["readback"][:5], row["command"][:5])), 1.5, row)
        r = ST.execute(self.sess, backend=ST.DryRun(list(ST.OBSERVE) + [30], tracking=short), settle=False)
        rows = [json.loads(line) for line in self.sess.file("execution.jsonl").read_text().splitlines()]
        last = [row for row in rows if row["event"] == "step" and row["kind"] == "move"][-len(moves):]
        self.assertTrue(any(max(abs(a - b) for a, b in zip(row["readback"][:5], row["command"][:5])) >= 2 - 1e-9
                            for row in last))
        self.assertEqual(len(p["steps"]), r["steps"])

    def test_moving_stages_need_really(self):
        with self.assertRaises(RuntimeError):
            ST.scan(self.sess, ST.SIM_CONSOLE)


if __name__ == "__main__":
    unittest.main()
