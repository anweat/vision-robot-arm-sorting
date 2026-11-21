"""Whole-field scan with the wrist camera only (no phone), through the running console.

Five high views chosen from the calibrated camera model (docs/05, 2026-09-26) cover the far grasp band
(map y 10-17 cm, |x| <= 13 cm) with a 30 px image margin; three folded near views (camera looking down in
front of the platform, clear of the rounded-square platform keep-out of 2026-11-14) add the near strip (the side
ones moved from J1 50/125 to 62/118 on 2026-11-14, which leaves the near corners of the map less covered). At each view the arm must be at rest; the frame and the console's
official-model detections are taken from a board result computed after the stop.

Since 2026-11-21 the poses, waits and what to do with a blocked view are an observation script
(config/observe/field.yaml, run by src/pipeline/observe.py; pictures through src/adapters/capture.py); this module
keeps the entry points the pipeline and the flow call.

    python3 src/pipeline/field_scan.py [run_dir]      # moves the arm; operator watching
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("tools", "tasks", "adapters"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import capture as CAP  # noqa: E402
import console_grasp as G  # noqa: E402
from console_step import BASE, Session, status  # noqa: E402

import observe as OB  # noqa: E402

# The field scan's poses are config/observe/field.yaml (2026-11-21; the comments on how they were chosen are there).
# SCAN_POSES stays for the modules that read the poses (stack_scan, dev/sim, dev/tools).
SCAN_SCRIPT = "field"
SCAN_POSES = [list(v["pose"][:4]) for v in OB.load(SCAN_SCRIPT)["views"]]
RUNS = ROOT / "data" / "runs" / "pipeline"


def frame_at_rest(settle: float = 0.8, timeout: float = 10.0, board: bool = True) -> tuple[bytes, dict]:
    """A frame taken after the readback has been still for `settle` seconds, with the console's board result for
    this pose when one arrives (src/adapters/capture.py, which also makes sure the frame is newer than the stop)."""
    try:
        s = CAP.shot(None, board=board, settle_s=settle, still_timeout_s=timeout)
    except CAP.CaptureError as e:
        raise G.ConsoleError(str(e)) from e
    return s.jpeg, s.status


def scan(run: Path | None = None, session: Session | None = None, script=SCAN_SCRIPT) -> Path:
    """The observation script `script` (default config/observe/field.yaml) into <run>/views.json."""
    run = run or RUNS / time.strftime("%Y%m%d_%H%M%S")
    OB.run(script, run, session)
    if not (run / "views.json").exists():          # every view skipped: an empty scan, not a missing file
        (run / "views.json").write_text("[]")
    return run


def bin_views_path(design: str) -> Path:
    return ROOT / "config" / f"bin_views_{design}.json"


def has_bin_views(design: str) -> bool:
    """Whether dev/tools/find_bin_views.py found view poses for this design. Without them the bins are taken
    where they were planned (seated on the printed outlines of the map), not looked for."""
    return bin_views_path(design).exists()


def bin_views_script(design: str | None = None, only=None) -> dict:
    """config/bin_views_<design>.json as an observation script (one view per bin, no board result, back to the
    observe pose at the end)."""
    cfg = json.loads(bin_views_path(design or "bins_acrylic_v1").read_text())
    views = [dict(pose=list(pose), name=f"bin_{number}", bin=int(number), board=False)
             for number, pose in sorted(cfg["views"].items(), key=lambda kv: int(kv[0]))
             if only is None or int(number) in only]
    return dict(name=f"bin_views_{cfg.get('design', design)}", views=views, event="bin_view", end="observe",
                on_blocked="stop")       # as before 2026-11-21: a bin view the way to which is blocked stops the scan


def scan_bins(run: Path, session: Session | None = None, design: str | None = None, only=None) -> Path:
    """One view of each bin's stickers, from the poses config/bin_views_<design>.json holds (dev/tools/find_bin_views.py).
    Written beside the field scan as bin_views.json + bin_view_<n>.jpg; views.json stays the map's own, since
    these views look past the paper and would only smear the orthophotos. The arm goes back to the observe
    pose at the end."""
    sc = bin_views_script(design, only)
    if sc["views"]:
        OB.run(sc, run, session, views_file="bin_views.json", image="bin_view_{bin}.jpg")
    else:
        (run / "bin_views.json").write_text("[]")
    return run


if __name__ == "__main__":
    print(scan(Path(sys.argv[1]) if len(sys.argv) > 1 else None))
