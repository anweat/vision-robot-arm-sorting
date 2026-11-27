"""Full offline checks with separate results and retained evidence.

python -X utf8 dev/tools/check_offline.py --profile python --out data/checks/new-run
python -X utf8 dev/tools/check_offline.py --profile all --android-serial emulator-5580 --out data/checks/new-run
Requires dev/requirements-offline.txt and `python -m playwright install chromium`.
The all profile additionally requires JDK 21, Android SDK and an already booted emulator.
"""
import argparse
import datetime
import importlib.metadata
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def static_checks():
    sys.path.insert(0, str(ROOT / "app/src"))
    sys.path.insert(0, str(ROOT / "app/src/tools"))
    import log_contract as L
    from flow import script
    expected = json.loads((ROOT / "app/contracts/log_event.schema.json").read_text(encoding="utf-8"))
    if L.schema() != expected: raise ValueError("log schema differs from the event table")
    paths = list((ROOT / "app/config/flows").glob("*.yaml"))
    for path in paths:
        checked = script.check_text(path.read_text(encoding="utf-8"))
        if not checked["ok"]: raise ValueError(f"{path}: {checked['errors']}")
    for name in ("placement-template.json", "placement-example-simulation.json"):
        rows = json.loads((ROOT / "records/hardware-tests" / name).read_text(encoding="utf-8"))
        if not isinstance(rows, list) or not rows: raise ValueError(f"{name}: empty measurement template")
    print(f"log schema consistent; {len(paths)} flow scripts valid; measurement templates parse")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", choices=("python", "android", "all"), default="python")
    ap.add_argument("--android-serial")
    ap.add_argument("--adb", default=shutil.which("adb"))
    ap.add_argument("--out", type=Path, default=ROOT / "data/checks" / time.strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--static-only", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.static_only:
        static_checks(); return 0
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, PYTHONUTF8="1", OFFLINE_ACCEPTANCE_OUT=str(out / "python_dev"))
    # Use the JDK actually on PATH if the inherited JAVA_HOME is stale (Windows desktop setups).
    java = shutil.which("java")
    if java and not (Path(env.get("JAVA_HOME", "")) / "bin/java.exe").exists() and os.name == "nt":
        env["JAVA_HOME"] = str(Path(java).resolve().parents[1])
    python = [sys.executable, "-X", "utf8"]
    checks = []
    def stage(name, command, cwd=ROOT, timeout=1200):
        print(f"## {name}", flush=True)
        began = time.monotonic()
        log = out / (name + ".out")
        try:
            with log.open("w", encoding="utf-8") as stream:
                process = subprocess.run(command, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)
            text = log.read_text(encoding="utf-8", errors="replace")
            ran = re.search(r"Ran (\d+) tests in", text)
            skipped = re.search(r"skipped=(\d+)", text)
            value = dict(name=name, ok=process.returncode == 0, exit_code=process.returncode,
                         tests=int(ran[1]) if ran else None, skipped=int(skipped[1]) if skipped else 0)
        except Exception as exc:
            value = dict(name=name, ok=False, error=str(exc))
        value.update(seconds=round(time.monotonic() - began, 2), log=str(log))
        checks.append(value)
        print(json.dumps(value, ensure_ascii=False), flush=True)
    if args.profile in ("python", "all"):
        stage("schema_scripts_templates", python + [str(Path(__file__).resolve()), "--static-only"])
        stage("python_app", python + ["-m", "unittest", "discover", "-s", "app/tests"])
        stage("python_dev", python + ["-m", "unittest", "discover", "-s", "dev/tests"])
    if args.profile in ("android", "all"):
        android = ROOT / "app/mobile/android"
        wrapper = [str(android / "gradlew.bat")] if os.name == "nt" else ["bash", str(android / "gradlew")]
        stage("android_build_unit", wrapper + [":app:assembleDebug", ":app:testDebugUnitTest", ":app:assembleDebugAndroidTest", "--console=plain"], cwd=android)
        if args.android_serial and args.adb:
            stage("android_emulator", python + [str(ROOT / "dev/tools/android_offline.py"), "--serial", args.android_serial,
                  "--adb", args.adb, "--out", str(out / "android_emulator")], timeout=360)
        else:
            checks.append(dict(name="android_emulator", ok=False, error="--android-serial and adb are required; not skipped"))
    findings = []
    for path in (out / "python_dev").rglob("report.json"):
        report = json.loads(path.read_text(encoding="utf-8"))
        for case in report.get("cases", []):
            if not case.get("ok") and case.get("contact_records"):
                findings.append(dict(scenario=report.get("scenario"), case=case["case"],
                                     contacts=case["contact_records"], evidence=str(path)))
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, encoding="utf-8").stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, encoding="utf-8").stdout
    def version(name):
        try: return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: return None
    report = dict(date=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                  device=platform.node(), platform=platform.platform(), python=sys.version, revision=revision,
                  working_tree_dirty=bool(dirty), profile=args.profile, ok=all(c["ok"] for c in checks), checks=checks, known_findings=findings,
                  evidence="offline/simulation/emulator; no physical hardware", hardware_connected=False,
                  dependencies={name: version(name) for name in ("numpy", "opencv-python", "onnxruntime", "pyyaml", "playwright")})
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{'PASS' if report['ok'] else 'FAIL'} ({args.profile}); known contact findings: {len(findings)}; report: {out / 'report.json'}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
