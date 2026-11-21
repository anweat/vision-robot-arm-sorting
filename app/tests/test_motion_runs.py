"""Continuous moves (docs/current/42 §4): the console's trajectory and how flow/moves.py joins a plan's steps."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "teleop"))
sys.path.insert(0, str(ROOT / "src"))

from core import Controller, SimArm  # noqa: E402
from flow import moves as MV, stages as ST  # noqa: E402
from flow.session import FlowSession  # noqa: E402


class LagArm(SimArm):
    """Reads back `lag` degrees short on J2 until told it settled (a servo still arriving)."""

    def __init__(self, lag=0.0):
        super().__init__()
        self.lag = lag
        self.commands = []

    def write6(self, pose, duration_ms):
        self.commands.append(list(pose))
        super().write6(pose, duration_ms)

    def read(self):
        p = list(super().read())
        p[1] -= self.lag
        return p


class TrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def controller(self, lag=0.0):
        arm = LagArm(lag)
        return Controller(self.tmp.name, arm), arm

    def run_traj(self, c, waypoints, stops=()):
        sleeps = []
        with patch("core.time.sleep", side_effect=lambda s: sleeps.append(s)):
            out = c.trajectory("a", waypoints, list(stops))
        return out, sleeps

    def test_passes_through_without_the_settle_and_stops_where_asked(self):
        c, arm = self.controller()
        m = c.cfg["motion"]
        out, sleeps = self.run_traj(c, [[90, 115, 5, 5, None, None], [90, 125, 5, 5, None, None],
                                        [None, None, None, None, None, 60], [90, 105, 5, 5, None, 30]], stops=[2])
        t = out["trajectory"]
        self.assertEqual(t["poses"][2], [90, 125, 5, 5, 95, 60])        # null keeps the waypoint before's value
        self.assertEqual([r[:6] for r in t["reached"]], t["poses"])
        self.assertEqual(arm.commands[-1], [90, 105, 5, 5, 95, 30])
        settle = (m["segment_ms"] + m["settle_ms"]) / 1000
        passing = (m["segment_ms"] + m.get("pass_settle_ms", 0)) / 1000
        stops = sum(1 for s in sleeps if abs(s - settle) < 1e-9)
        self.assertEqual(stops, 2)                                       # the J6 waypoint and the last
        self.assertEqual(sum(1 for s in sleeps if abs(s - passing) < 1e-9), len(arm.commands) - 2)

    def test_the_whole_chain_is_checked_before_anything_moves(self):
        c, arm = self.controller()
        with self.assertRaises(Exception):
            self.run_traj(c, [[90, 115, 5, 5, None, None], [90, 115, -40, 5, None, None]])
        self.assertEqual(arm.commands, [])
        with self.assertRaises(ValueError):
            c.plan_trajectory([[90, 115, 5]])
        p = c.plan_trajectory([[90, 115, 5, 5, None, None], [80, 115, 5, 5, None, None]])
        self.assertEqual(p["stops"], [1])
        self.assertGreater(p["segments"], 1)
        self.assertEqual(arm.commands, [])

    def test_a_lag_passes_between_stops_but_not_at_one(self):
        c, _ = self.controller(lag=5.0)          # more than readback_tolerance_deg (4), less than pass (8)
        with self.assertRaises(ValueError):       # the last waypoint is a stop: held to the normal tolerance
            self.run_traj(c, [[90, 115, 5, 5, None, None], [90, 125, 5, 5, None, None]])
        c2, arm2 = self.controller(lag=5.0)
        c2.cfg["motion"]["readback_tolerance_deg"] = 6   # the stop now accepts it: the passes did before
        out, _ = self.run_traj(c2, [[90, 115, 5, 5, None, None], [90, 125, 5, 5, None, None]])
        self.assertEqual(arm2.commands[-1][:4], [90, 125, 5, 5])
        self.assertEqual(out["trajectory"]["stops"], [1])


PLAN_STEPS = [("pick", "to hover", "move", None), ("pick", "open", "gripper", "open"), ("pick", "descend", "move", None),
              ("pick", "grip", "gripper", "grip"), ("pick", "lift", "move", None), ("place", "carry 1/2", "move", None),
              ("place", "carry 2/2", "move", None), ("place", "lower", "move", None), ("place", "release", "gripper", "j6"),
              ("place", "back off", "move", None), ("transit", "transit", "move", None)]


def steps():
    cmds = [[100, 60, 50, 20, 200, 122], [100, 60, 50, 20, 200, 30], [100, 50, 35, 50, 200, 30], [100, 50, 35, 50, 200, 133],
            [100, 60, 50, 20, 200, 133], [110, 70, 40, 10, 200, 133], [112, 72, 38, 8, 200, 133], [112, 80, 20, 20, 200, 133],
            [112, 80, 20, 20, 200, 100], [112, 72, 38, 8, 200, 100], [90, 105, 5, 5, 92, 100]]
    return [dict(plan=p, name=n, kind=k, action=a, command=c, check=dict(ok=True))
            for (p, n, k, a), c in zip(PLAN_STEPS, cmds)]


class RunsTest(unittest.TestCase):
    def test_steps_mode_sends_every_step_alone(self):
        s = steps()
        self.assertEqual(MV.runs(s, range(len(s)), "steps"), [[i] for i in range(len(s))])

    def test_continuous_joins_between_the_grip_and_the_lower(self):
        s = steps()
        self.assertEqual(MV.runs(s, range(len(s)), "continuous"), [[0, 1, 2], [3], [4, 5, 6, 7], [8, 9, 10]])
        self.assertEqual(MV.runs(s, range(len(s)), "continuous", correct=False), [[0, 1, 2], [3], [4, 5, 6, 7, 8, 9, 10]])
        self.assertEqual(MV.stops(s, [0, 1, 2]), [1, 2])               # the opening, and the run's end
        self.assertEqual(MV.waypoint(s[1]), [None] * 5 + [30])
        self.assertEqual(MV.waypoint(s[0]), [100, 60, 50, 20, 200, None])
        with self.assertRaises(ValueError):
            MV.runs(s, range(3), "fast")


class ExecuteContinuousTest(unittest.TestCase):
    def test_a_dry_run_in_continuous_mode_logs_every_step(self):
        with tempfile.TemporaryDirectory() as d:
            sess = FlowSession(Path(d))
            plan = dict(ok=True, steps=steps(), plans=[], task=dict(start=[100, 60, 50, 20, 200], start_j6=122))
            sess.save("plan.json", plan)
            for motion in ("steps", "continuous"):
                r = ST.execute(sess, backend=ST.DryRun([100, 60, 50, 20, 200, 122]), motion=motion, correct=False)
                self.assertTrue(r["ok"], r["stopped"])
                self.assertEqual([row["i"] for row in r["rows"]], list(range(11)))
                self.assertEqual(r["rows"][-1]["readback"][:5], [90, 105, 5, 5, 92])
            self.assertEqual(r["motion"], "continuous")
            grip = next(row for row in r["rows"] if row["name"] == "grip")
            self.assertEqual(grip["grip_result"], "holding")
            start = json.loads((Path(d) / "execution.jsonl").read_text().splitlines()[-len(r["rows"]) - 2])
            self.assertEqual(start["runs"], [[0, 2], [4, 10]])


if __name__ == "__main__":
    unittest.main()
