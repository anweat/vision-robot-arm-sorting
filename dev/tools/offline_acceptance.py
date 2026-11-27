"""One-command regression for the five offline tasks, with retained evidence and a failing exit code.

python dev/tools/offline_acceptance.py --out data/offline-acceptance/<new-run>
Requires numpy, opencv-python<5, onnxruntime and pyyaml. Starts its own loopback-only simulator;
no hardware URL is accepted. Output directory must be new to avoid overwriting earlier evidence.
"""
import argparse
import contextlib
import datetime
import json
import importlib.metadata
import os
import platform
import subprocess
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "data/offline-acceptance" / time.strftime("%Y%m%d_%H%M%S"))
    args = ap.parse_args(argv)
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    # Capture source identity before any test mutates process-global settings.
    metadata_warnings = []
    def git(*arguments):
        got = subprocess.run(["git", *arguments], cwd=ROOT, capture_output=True, encoding="utf-8", errors="replace")
        if got.returncode:
            metadata_warnings.append(got.stderr.strip())
            return None
        return got.stdout.strip()
    revision, status = git("rev-parse", "HEAD"), git("status", "--porcelain")
    suite = unittest.TestSuite()
    for directory, pattern in (("app/tests", "test_place_correct.py"), ("dev/tests", "test_readback_stats.py"),
                               ("dev/tests", "test_acceptance_stats.py"), ("dev/tests", "test_gateway_integration.py")):
        # Each test directory has its own discovery root; do not reuse the loader's cached top level.
        suite.addTests(unittest.TestLoader().discover(str(ROOT / directory), pattern=pattern))
    previous = os.environ.get("OFFLINE_ACCEPTANCE_OUT")
    os.environ["OFFLINE_ACCEPTANCE_OUT"] = str(out)
    began = time.monotonic()
    try:
        with (out / "tests.out").open("w", encoding="utf-8") as stream:
            with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
                result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    finally:
        if previous is None:
            os.environ.pop("OFFLINE_ACCEPTANCE_OUT", None)
        else:
            os.environ["OFFLINE_ACCEPTANCE_OUT"] = previous
    report = dict(date=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                  device=platform.node(), platform=platform.platform(), python=sys.version, revision=revision,
                  working_tree_dirty=bool(status) if status is not None else None,
                  metadata_warnings=metadata_warnings, evidence="offline/simulation only", hardware_connected=False,
                  dependencies={name: importlib.metadata.version(name) for name in ("numpy", "opencv-python", "onnxruntime", "pyyaml")},
                  tests=result.testsRun, failures=[dict(test=str(t), detail=d) for t, d in result.failures],
                  errors=[dict(test=str(t), detail=d) for t, d in result.errors],
                  skipped=[dict(test=str(t), reason=d) for t, d in result.skipped],
                  seconds=round(time.monotonic() - began, 2), ok=result.wasSuccessful() and not result.skipped)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{'PASS' if report['ok'] else 'FAIL'}: {result.testsRun} tests; evidence: {out}")
    if not report["ok"]:
        print((out / "tests.out").read_text(encoding="utf-8"))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
