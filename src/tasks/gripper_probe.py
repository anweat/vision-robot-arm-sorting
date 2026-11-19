"""Close the gripper 1 deg at a time through the console and find, from the raw readback, where the fingers meet
something: each other (empty), or a block between them. Doc 29 §6.

The official 'width -> J6' table puts 30 mm at J6 134 and the pads meeting at 180, but on this arm a 3 cm block
holds the servo at readback 127-129 under the 133 command, and a hand-held one started to resist at ~124.
Which is right decides the grip command (133 may be pressing 5-9 deg past contact, the stall the plastic-gear
servo trips its protection on) and the narrow openings. This measures it.

    python3 src/tasks/gripper_probe.py empty [--from 150 --to 175]   # nothing in the jaws: where do the pads meet
    python3 src/tasks/gripper_probe.py block [--from 110 --to 140]   # a block held square between the pads

Joints 1-5 keep their pose (run it at the observe pose); J6 goes up 1 deg per step through /api/goto, the raw
readback comes from /api/precise. Contact = the readback falls `--lag` deg further behind its command than it
did over the first steps; the gripper is then sent back to `--back` deg below that readback at once, so the
servo never stalls for long (manual: protection after 3 s). Results: data/runs/gripper_probe.jsonl.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "tools"))

from console_step import ConsoleError, Session, call  # noqa: E402

OUT = ROOT / "data" / "runs" / "gripper_probe.jsonl"


def precise() -> dict:
    return call("/api/precise")


class Probe(Session):
    def goto(self, pose):
        for k in range(10):
            try:
                return self._post("/api/goto", {"pose": [int(round(v)) for v in pose]}, timeout=60)
            except ConsoleError as e:
                if "新鲜的回读" not in str(e) or k == 9:
                    raise
                time.sleep(0.4)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("label", choices=("empty", "block"))
    ap.add_argument("--from", dest="start", type=int)
    ap.add_argument("--to", dest="stop", type=int)
    ap.add_argument("--lag", type=float, default=1.5)
    ap.add_argument("--back", type=int, default=3)
    a = ap.parse_args()
    start = a.start if a.start is not None else (150 if a.label == "empty" else 110)
    stop = a.stop if a.stop is not None else (175 if a.label == "empty" else 140)
    trace, contact = [], None
    with Probe(owner="gripper-probe") as s:
        here = precise()["pose"]
        arm5 = here[:5]
        s.goto(arm5 + [start])
        for j6 in range(start, stop + 1):
            s.goto(arm5 + [j6])
            time.sleep(0.3)
            r = precise()["precise"][5]
            trace.append([j6, r])
            lags = [c - v for c, v in trace]
            base = statistics.median(lags[:5])
            print(f"J6 {j6}: readback {r:.2f} (lag {lags[-1] - base:+.2f})", flush=True)
            if len(trace) >= 3 and lags[-1] - base >= a.lag:
                contact = dict(command=j6, readback=r, lag=round(lags[-1] - base, 2))
                s.goto(arm5 + [int(r) - a.back])          # let go at once
                break
        time.sleep(0.5)
        after = precise()["precise"][5]
        s.goto(arm5 + [30])                               # the usual open gripper
    row = dict(t=time.time(), label=a.label, start=start, stop=stop, lag_threshold=a.lag, contact=contact,
               baseline_lag=round(statistics.median([c - v for c, v in trace[:5]]), 2), trace=trace,
               relieved_readback=after, arm=arm5)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("a") as f:
        f.write(json.dumps(row) + "\n")
    print("contact:", contact or f"none up to {stop}")


if __name__ == "__main__":
    main()
