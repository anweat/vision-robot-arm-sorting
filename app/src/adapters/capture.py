"""Taking a picture through the console: wait until the arm is still, then a frame captured after that.

Every scan (field_scan, stack_scan, the close looks of run.refine, the bin views, pipeline/observe.py) took its
pictures through field_scan.frame_at_rest; this is that call on its own, so a scan script, a bench or a page can
take a picture without importing the scan, and the rule for a usable frame lives in one place:

  1. still     two readbacks `settle_s` apart agree within `still_deg` on every joint, the console busy at neither;
  2. frame     /api/frame. A console that sends X-Frame-Time (the teleop server and the simulator, 2026-11-21) is
               asked again until the frame is newer than the first of the two still readbacks, so a frame from
               the camera's buffer taken while the arm was still arriving is not used. Both times are the board's
               clock (console and camera run there), never this machine's. An older console without the header:
               the frame is taken as it comes, as before.
  3. board     optional: the console's own localization result for this pose (markers, the official model's
               detections), when one computed after the stop arrives within `board_timeout_s`; else the status as
               it is (a view without enough markers is still a useful view).

    python3 src/adapters/capture.py [out.jpg] [--console URL] [--no-board]     # one picture where the arm stands

Nothing here moves the arm.
"""
from __future__ import annotations

import json
import sys
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "src" / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "src" / "tools"))

import console_step  # noqa: E402

SETTLE_S = 0.8          # two readbacks this far apart must agree...
STILL_DEG = 1           # ...within this on every joint
STILL_TIMEOUT_S = 10.0
FRAME_TIMEOUT_S = 3.0
BOARD_TIMEOUT_S = 8.0


class CaptureError(RuntimeError):
    pass


@dataclass
class Shot:
    jpeg: bytes
    status: dict                         # the console status the picture goes with (the board result's, if any)
    frame: dict = field(default_factory=dict)   # {stamp, seq} from the frame's headers (empty: an old console)
    board: dict | None = None            # localization.board for this pose, None if it did not come
    waited_s: float = 0.0

    @property
    def pose(self) -> list:
        return self.status["pose"]

    def record(self, image: str, **extra) -> dict:
        """The views.json entry the scans write (image, readback, markers, official, t) plus `extra`."""
        b = self.board or {}
        out = dict(image=image, readback=self.status["pose"], markers=b.get("markers"), official=b.get("official", []),
                   t=time.time())
        if self.frame:
            out["frame"] = self.frame
        out.update(extra)
        return out


def _base(console: str | None) -> str:
    return (console or console_step.BASE).rstrip("/")


def status(console: str | None = None) -> dict:
    with urllib.request.urlopen(_base(console) + "/api/status", timeout=5) as r:
        return json.loads(r.read())


def _close(a, b, tol: float) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b) if x is not None and y is not None)


def wait_still(console: str | None = None, settle_s: float = SETTLE_S, still_deg: float = STILL_DEG,
               timeout_s: float = STILL_TIMEOUT_S) -> tuple[dict, dict]:
    """(first, second): two statuses `settle_s` apart with the arm not busy and still between them."""
    end = time.time() + timeout_s
    last = status(console)
    while time.time() < end:
        time.sleep(settle_s)
        now = status(console)
        if not now.get("busy") and not last.get("busy") and _close(now["pose"], last["pose"], still_deg):
            return last, now
        last = now
    raise CaptureError("the arm did not come to rest")


def frame(console: str | None = None, after: float | None = None, timeout_s: float = FRAME_TIMEOUT_S) -> tuple[bytes, dict]:
    """(jpeg, {stamp, seq}) from /api/frame; with `after`, one captured later than that (board clock)."""
    end = time.time() + timeout_s
    while True:
        with urllib.request.urlopen(_base(console) + "/api/frame", timeout=5) as r:
            jpeg = r.read()
            stamp, seq = r.headers.get("X-Frame-Time"), r.headers.get("X-Frame-Seq")
        meta = {} if stamp is None else dict(stamp=float(stamp), seq=int(seq) if seq else None)
        if after is None or not meta or meta["stamp"] > after:
            return jpeg, meta
        if time.time() > end:
            raise CaptureError(f"no frame newer than the still readback within {timeout_s} s "
                               f"(latest {meta['stamp']:.2f}, wanted > {after:.2f})")
        time.sleep(0.05)


def board_result(still: dict, console: str | None = None, timeout_s: float = BOARD_TIMEOUT_S) -> dict | None:
    """The console status once its localization result for this pose (computed after `still`) is in, else None."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        s = status(console)
        board = (s.get("localization") or {}).get("board") or {}
        if board.get("t", 0) > still["observed_at"] and board.get("servos") and _close(board["servos"], still["pose"], 1):
            return s
        time.sleep(0.4)
    return None


def shot(console: str | None = None, board: bool = True, settle_s: float = SETTLE_S, still_deg: float = STILL_DEG,
         still_timeout_s: float = STILL_TIMEOUT_S, board_timeout_s: float = BOARD_TIMEOUT_S) -> Shot:
    """Wait still, take a frame captured after that, and (board=True) the console's result for the pose."""
    t0 = time.time()
    first, now = wait_still(console, settle_s, still_deg, still_timeout_s)
    jpeg, meta = frame(console, after=first.get("observed_at") or None)
    s, b = now, None
    if board:
        got = board_result(now, console, board_timeout_s)
        if got is not None:
            s, b = got, (got.get("localization") or {}).get("board")
    return Shot(jpeg=jpeg, status=s, frame=meta, board=b, waited_s=round(time.time() - t0, 2))


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", nargs="?", type=Path, default=Path("frame.jpg"))
    ap.add_argument("--console")
    ap.add_argument("--no-board", action="store_true")
    a = ap.parse_args(argv)
    s = shot(a.console, board=not a.no_board)
    a.out.write_bytes(s.jpeg)
    print(json.dumps(dict(out=str(a.out), pose=s.pose, frame=s.frame, waited_s=s.waited_s,
                          markers=(s.board or {}).get("markers")), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
