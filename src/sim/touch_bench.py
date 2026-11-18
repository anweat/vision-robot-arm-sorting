"""Bench the console's touch probe against a servo model fitted to the board (doc 33 §3); nothing real moves.

The model is what three probes on 2026-11-18 showed, not a physical servo. Each of J2-J4 sits at its command
plus a fixed bias plus AR(1) noise, as measured after removing the trend (deg): J2 bias -0.5, sd 0.15, lag-1
correlation +0.3; J3 sd 0.26, -0.6 (it zigzags about its command); J4 bias -0.35, sd 0.1, -0.2
(scaled by 0.8 below so that the simulated lag matches the board's). With `stick`,
J2 does not follow a command that rises (the arm leaning back against gravity: the free-air probe, where it
stayed put while its command climbed 0.08 deg a step). Readback is quantized to one count (0.08 deg); a
surface gives way by `soft` of the overshoot (0.5: pressed, the tip sank at about half the commanded rate).
Compared per descent mode (all three joints straight down, or J2 held):

  * false contacts with the surface out of reach,
  * how many steps past the true contact the probe stops (pressing),
  * the knee fit's height error (src/tools/fit_touch.onset), which is what the calibration uses.

    uv run --with numpy python src/sim/touch_bench.py [--trials 40] [--soft 0.5]
"""
from __future__ import annotations

import argparse
import random
import statistics
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for sub in ("teleop", "spatial", "adapters", "tools"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
from core import Controller, SimArm  # noqa: E402
import fit_touch  # noqa: E402

# bias, sd, lag-1 corr (deg). The sd are the detrended per-joint spreads x 0.8, the scale at which the
# simulated lag (readback minus command, mm) matches the board's: sd 0.67-0.75, 7-step slope sd 0.125
NOISE = {2: (-0.5, 0.12, 0.3), 3: (0.0, 0.21, -0.6), 4: (-0.35, 0.08, -0.2)}
COUNT = 180 / 2200                  # deg per position count


class ServoModelArm(SimArm):
    def __init__(self, pose, surface_z, soft, rng, stick=False):
        super().__init__()
        self.pose = [float(v) for v in pose]       # where the joints really are
        self.surface, self.soft, self.rng, self.stick = surface_z, soft, rng, stick
        self.touched_at = None                     # command index of the first contact
        self.n = 0
        self.e = {j: 0.0 for j in NOISE}
        self.cmd = [float(v) for v in pose]

    @staticmethod
    def z(q):
        return kin.forward(list(q)[:4])["tip"][2]

    def write6(self, pose, duration_ms):
        self.n += 1
        goal = [float(v) for v in pose]
        for j, (bias, sd, phi) in NOISE.items():
            self.e[j] = phi * self.e[j] + sd * (1 - phi * phi) ** 0.5 * self.rng.gauss(0, 1)
            goal[j - 1] = float(pose[j - 1]) + bias + self.e[j]
        if self.stick and float(pose[1]) > self.cmd[1]:
            goal[1] = self.pose[1]                 # J2 stays put while its command climbs
        self.cmd = [float(v) for v in pose]
        if self.z(goal) >= self.surface:
            self.pose = goal
            return
        if self.touched_at is None:
            self.touched_at = self.n
        floor = self.surface - self.soft * (self.surface - self.z(goal))
        a, b = 0.0, 1.0
        for _ in range(30):
            m = (a + b) / 2
            q = [x + (y - x) * m for x, y in zip(self.pose, goal)]
            a, b = (m, b) if self.z(q) >= floor else (a, m)
        self.pose = [x + (y - x) * a for x, y in zip(self.pose, goal)]

    def read(self):
        return [int(round(v)) for v in self.pose]

    def read_precise(self, joints=(1, 2, 3, 4)):
        return {j: round(round(self.pose[j - 1] / COUNT) * COUNT, 2) for j in joints}


START = [90, 68, 26, 20, 92, 168]          # the board's hover for J1 90 / 19 cm / pitch 157


def one(lock: bool, surface_below_mm: float, soft: float, seed: int, stick: bool = False, start=None) -> dict:
    rng = random.Random(seed)
    start = list(start or START)
    z0 = ServoModelArm.z(start)
    arm = ServoModelArm(start, z0 - surface_below_mm / 1000, soft, rng, stick)
    with tempfile.TemporaryDirectory() as tmp:
        c = Controller(tmp, arm)
        c.command = list(start)
        with patch("core.time.sleep"):
            r = c.touch("bench", max_cm=4, step_mm=1, stall_mm=3, after=3, lock_j2=lock)
    out = dict(contacted=r["contacted"], steps=r["steps"], touched_at=arm.touched_at)
    if arm.touched_at is not None:
        out["late"] = r["steps"] - arm.touched_at
    knee = fit_touch.onset(r["trace"])
    if knee is not None and arm.touched_at is not None:
        out["knee_err_mm"] = (ServoModelArm.z(knee["q"]) - arm.surface) * 1000
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--soft", type=float, default=0.5)
    a = ap.parse_args()
    for stick, start, label in ((False, START, "J2 follows (19 cm, pitch 157)"),
                                (True, START, "J2 sticks going up"),
                                (False, [90, 52, 21, 64, 92, 168], "27 cm, pitch 134")):
        print(label)
        for lock in (False, True):
            name = "  J2 held " if lock else "  straight"
            air = [one(lock, 80, a.soft, s, stick, start) for s in range(a.trials)]      # surface out of reach
            hit = [one(lock, 22 + (s % 7), a.soft, 1000 + s, stick, start) for s in range(a.trials)]
            late = [h["late"] for h in hit if h.get("contacted") and "late" in h]
            knees = [h["knee_err_mm"] for h in hit if "knee_err_mm" in h]
            kt = (f"knee error median {statistics.median(knees):+.2f} mm, sd {statistics.pstdev(knees):.2f}, "
                  f"worst {max(knees, key=abs):+.2f} ({len(knees)} fits)") if knees else "no knee fits"
            print(f"{name}: false contacts in air {sum(r['contacted'] for r in air)}/{len(air)}; "
                  f"found {len(late)}/{len(hit)}, stopped {statistics.median(late) if late else '-'} "
                  f"(max {max(late) if late else '-'}) steps after contact; {kt}")


if __name__ == "__main__":
    main()
