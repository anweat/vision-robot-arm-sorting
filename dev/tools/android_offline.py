"""Run Android instrumentation against an isolated loopback simulator/gateway; emulator serial required.

Build APKs first: gradlew :app:assembleDebug :app:assembleDebugAndroidTest
python dev/tools/android_offline.py --serial emulator-5580 --out data/android-check/new-run
"""
import argparse
import csv
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dev/tests"))
from offline_fixture import FixtureGateway, S


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial", required=True)
    ap.add_argument("--adb", default=shutil.which("adb"))
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    if not args.serial.startswith("emulator-") or not args.adb:
        ap.error("an emulator serial and adb are required; physical devices are not accepted")
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    def adb(*arguments, timeout=60, binary=False):
        r = subprocess.run([args.adb, "-s", args.serial, *arguments], capture_output=True, timeout=timeout)
        if r.returncode: raise RuntimeError(r.stderr.decode("utf-8", "replace"))
        return r.stdout if binary else r.stdout.decode("utf-8", "replace")
    fixture, process, stream = None, None, None
    reverse = None
    report = dict(ok=False, evidence="Android emulator and local simulator only", serial=args.serial)
    try:
        if adb("shell", "getprop", "sys.boot_completed").strip() != "1": raise RuntimeError("emulator has not booted")
        if adb("shell", "getprop", "ro.kernel.qemu").strip() != "1": raise RuntimeError("device is not an emulator")
        for apk in ("app-debug.apk", "app-debug-androidTest.apk"):
            where = "debug" if apk == "app-debug.apk" else "androidTest/debug"
            path = ROOT / "app/mobile/android/app/build/outputs/apk" / where / apk
            adb("install", "-r", "-t", str(path), timeout=120)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0)); port = sock.getsockname()[1]
        stream = (out / "simulator.out").open("w", encoding="utf-8")
        sim_url = f"http://127.0.0.1:{port}"
        process = subprocess.Popen([sys.executable, str(ROOT / "dev/sim/sim_server.py"), "--port", str(port),
            "--blocks", "0", "--speed", "100", "--output", str(out / "sim")], cwd=ROOT,
            env=dict(os.environ, PYTHONUTF8="1"), stdout=stream, stderr=subprocess.STDOUT)
        console = S.Console(sim_url, timeout=2)
        deadline = time.monotonic() + 60
        while True:
            try:
                status = console.status()
                if status.get("mode") != "simulation": raise RuntimeError("not a simulator")
                break
            except S.ConsoleError:
                if process.poll() is not None or time.monotonic() > deadline: raise
                time.sleep(0.2)
        fixture = FixtureGateway(out / "gateway")
        fixture.state.console = console
        fixture.state.scripts.save("mobile-demo", "title: 手机离线联调\nsteps:\n  - step: notify\n    with: {text: 手机联调}\n  - step: wait\n    with: {seconds: 4}\n  - alarm: offline_check\n    summary: 手机确认测试\n    then: continue\n")
        fixture.state.scripts.save("voice-unavailable", "title: 语音未接入\nsteps:\n  - step: wait_voice\n    with: {words: 开始}\n")
        gateway_port = fixture.url.rsplit(":", 1)[1]
        reverse = "tcp:" + gateway_port
        adb("reverse", reverse, reverse)
        result = adb("shell", "am", "instrument", "-w", "-r", "-e", "class", "com.example.myapplication00001.OfflineGatewayTest",
                     "-e", "gatewayUrl", fixture.url, "com.example.myapplication00001.test/androidx.test.runner.AndroidJUnitRunner", timeout=240)
        (out / "instrumentation.out").write_text(result, encoding="utf-8")
        if "OK (3 tests)" not in result or "FAILURES" in result: raise RuntimeError(result)
        for suffix in ("csv", "xlsx"):
            data = adb("exec-out", "run-as", "com.example.myapplication00001", "cat", "files/offline-records." + suffix, binary=True)
            (out / ("records." + suffix)).write_bytes(data)
        csv_rows = list(csv.reader(io.StringIO((out / "records.csv").read_text(encoding="utf-8-sig"))))
        if len(csv_rows) != 1001: raise RuntimeError(f"CSV rows: {len(csv_rows)}")
        events, _ = S.L.read(fixture.state.log_path)
        expected_ids = {str(r["seq"]) for r in S.L.records(events)}
        if {r[0] for r in csv_rows[1:]} != expected_ids: raise RuntimeError("CSV record identities differ from gateway")
        from xml.etree import ElementTree as ET
        with zipfile.ZipFile(out / "records.xlsx") as archive:
            sheet = ET.fromstring(archive.read("xl/worksheets/sheet1.xml"))
            if len(sheet.findall(".//{http://schemas.openxmlformats.org/spreadsheetml/2006/main}row")) != 1001:
                raise RuntimeError("XLSX row count differs")
            exported = {"".join(row[0].itertext()) for row in sheet.findall(".//{http://schemas.openxmlformats.org/spreadsheetml/2006/main}row")[1:]}
            if exported != expected_ids: raise RuntimeError("XLSX record identities differ from gateway")
        for name in ("offline-script.png", "offline-records.png"):
            image = adb("exec-out", "run-as", "com.example.myapplication00001", "cat", "files/" + name, binary=True)
            (out / name).write_bytes(image)
        report.update(ok=True, tests=3, csv_records=len(csv_rows)-1, xlsx_records=1000,
                      android_api=adb("shell", "getprop", "ro.build.version.sdk").strip())
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        if reverse:
            try: adb("reverse", "--remove", reverse)
            except Exception: pass
        if fixture: fixture.close()
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=10)
        if stream: stream.close()
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
