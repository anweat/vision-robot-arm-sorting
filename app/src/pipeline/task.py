"""One sorting round with the verified pipeline (run.py: scan, scene, the console_grasp paths), started by the
board gateway (web/serve.py `--task pipeline`, the default) for the page's / app's / voice / gesture / timer
"start". Doc 34 §4.2: the step flow's planner (src/flow + src/planning) refuses most blocks of the real scans,
so it is not the default until it is reworked.

    python3 src/pipeline/task.py --console http://127.0.0.1:8765 --log data/web/log.jsonl [run.py options]

--console points console_step (and the modules that copy its BASE at import) at the console before they are
imported; --log appends every event console_grasp.log writes (pipeline_*, hovering, gripped, put, ...) to the
gateway's live log too, so the pages see them over SSE. Every other option goes to run.py unchanged
(--bins DESIGN, --bin-side, --dry-run, --scan DIR). Moves the arm. Against the simulator console its records go
under data/sim/ instead of data/runs/, as with dev/sim/run_pipeline.py.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("tools", "tasks", "pipeline", "spatial", "vision", "adapters"):
    sys.path.insert(0, str(ROOT / "src" / sub))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--console", required=True)
    ap.add_argument("--log", type=Path, help="also append the events to this log (the gateway's)")
    a, rest = ap.parse_known_args(argv)
    import console_step
    console_step.BASE = a.console.rstrip("/")          # before the modules below copy it
    import console_grasp as G
    if a.log is not None:
        own = G.log
        a.log.parent.mkdir(parents=True, exist_ok=True)

        def log(event: str, **data) -> None:
            own(event, **data)
            with a.log.open("a", encoding="utf-8") as f:
                f.write(json.dumps(dict(t=time.time(), event=event, **data), ensure_ascii=False) + "\n")

        G.log = log
    import run as R
    import field_scan as SC
    boot = console_step.call("/api/bootstrap")
    if boot.get("status", {}).get("mode") == "simulation":
        # a simulated round never writes into the real arm's records (as dev/sim/run_pipeline.py does)
        sim = ROOT / "data" / "sim"
        sim.mkdir(parents=True, exist_ok=True)
        G.RUNS, G.PLAN, SC.RUNS = sim, sim / "plan.json", sim / "pipeline"
        R.STATE_FILE = sim / "bins_state.json"
    sys.argv = [str(Path(R.__file__))] + rest
    R.main()
    return 0


if __name__ == "__main__":
    sys.exit(main())
