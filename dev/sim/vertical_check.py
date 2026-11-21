"""Stacking, unstacking and setting blocks into bins, end to end against the simulator, judged by its truth.

    uv run --with 'opencv-python<5' --with numpy python dev/sim/vertical_check.py [--stop-short 1.5] [--no-correct]
        [--motion steps|continuous] [--seed 3] [--session data/flow/vertical_check]

Since 2026-11-21 this is dev/sim/bench.py's scenario `vertical` (dev/sim/scenarios/vertical.yaml: the blocks, the five
cases and what each must end as) with this script's old options as overrides; `bench.py run vertical --sweep ...`
compares settings. Needs the simulator console (launch config `arm-sim`); it resets the table. The blocks are
located from the simulator's truth (heights included), so this checks the planning and the execution, not
perception (doc 40 §4; the stack scan is dev/tools/stack_bench.py). Results only say the geometry and the
interfaces agree with each other (docs/current/16).

Cases, in order (each starts from what the one before left):
  stack     A onto B, faces aligned with B
  stack2    C onto A (three high)
  unstack   C off the stack onto the table
  bin_set   D lowered into the bin of its class (mode set)
  bin_drop  C dropped into the bin of its class from above the rim (mode drop, the old way), for comparison
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bench as B                      # noqa: E402
from flow import stages as ST          # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--console", default=ST.SIM_CONSOLE)
    ap.add_argument("--stop-short", type=float, default=1.5, help="sim fault: each joint stops up to this short (deg)")
    ap.add_argument("--no-correct", action="store_true", help="execute without the readback correction")
    ap.add_argument("--motion", choices=("steps", "continuous"), default="steps")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--session", default=str(HERE.parents[1] / "data" / "flow" / "vertical_check"))
    a = ap.parse_args(argv)
    sets = {"fault.stop_short_deg": a.stop_short, "execute.correct": not a.no_correct, "execute.motion": a.motion}
    s = B.run("vertical", sets, seed=a.seed, console=a.console, out=Path(a.session))
    B.print_table(s)
    return 0 if s["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
