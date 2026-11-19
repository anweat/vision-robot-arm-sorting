#!/usr/bin/env python3
"""演示脚本：两块依次抓取，第二块故意夹空 → 自动重扫重抓；LED 与蜂鸣器随状态变化。

用法（在板子上，/root/arm-app 目录）：

    ./venv-vision/bin/python scripts/demo_miss_retry.py

    # 想换“第几次夹空”：--force-miss 2（默认第 2 次，即第一块正常、第二块夹空）
    # 不想清桶计数：--keep-bins
    # 只看脚本会做什么：--dry

前置条件（脚本会自检并在缺失时打印提示）：
  * 相机 :8080、控制台 :8765、网关 :8081 已启动（网关必须带 NPU 环境）
  * 桌上放一红一绿两块（有害 + 厨余），都在 A4 纸范围内
  * 控制台持有 I2C，所以 LED/蜂鸣器由控制台执行（feedback.py 走 /api/led 与 /api/beep）

脚本讲的“故事”：
  1. 扫描 → 台上有两块
  2. 第一块：抓中 → 进桶 → 绿灯 + 短鸣
  3. 第二块：合爪被强制夹空 → 黄灯 + 蜂鸣 → 退回观察位 → 重扫 → 找回 → 再抓 → 绿灯 + 短鸣
  4. 结束：打印本轮统计
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = os.environ.get("ARM_CONSOLE", "http://127.0.0.1:8765").rstrip("/")
PY = ROOT / "venv-vision" / "bin" / "python"
LOG = ROOT / "data" / "runs" / "web" / "log.jsonl"

C = dict(ok="\033[32m", warn="\033[33m", bad="\033[31m", dim="\033[2m", off="\033[0m", bold="\033[1m")


def say(tag: str, text: str) -> None:
    colour = {"ok": C["ok"], "warn": C["warn"], "bad": C["bad"], "dim": C["dim"], "bold": C["bold"], "": ""}[tag]
    print(f"{colour}{text}{C['off'] if colour else ''}", flush=True)


def preflight() -> bool:
    ok = True
    try:
        with urllib.request.urlopen(CONSOLE + "/api/status", timeout=3) as r:
            st = json.loads(r.read())
        say("ok", f"控制台在线：姿态 {st.get('pose')}，错误 {st.get('error') or '无'}")
        if st.get("error"):
            say("warn", f"控制台报错：{st['error']}（先点一次使能或回观察位）")
            ok = False
    except Exception as exc:                               # noqa: BLE001
        say("bad", f"控制台不可用（{CONSOLE}）：{exc}")
        ok = False
    try:
        with urllib.request.urlopen(CONSOLE + "/api/frame", timeout=3) as r:
            size = len(r.read())
        say("ok", f"相机在线：一帧 {size // 1024} KB")
    except Exception as exc:                               # noqa: BLE001
        say("bad", f"相机不可用：{exc}")
        ok = False
    if not PY.exists():
        say("bad", f"找不到 {PY}")
        ok = False
    return ok


def go_observe() -> None:
    """先回观察位：扫描从当前姿态出发，停在悬停位会第一步就被挡。"""
    sys.path.insert(0, str(ROOT / "src" / "tools"))
    try:
        from console_step import Session, status
        print(f"  当前姿态 {status().get('pose')} -> 回观察位", flush=True)
        with Session() as s:
            print("  现在", s.pose([90, 105, 5, 5, 92, 29]).get("pose"), flush=True)
    except Exception as exc:                                  # noqa: BLE001
        say("warn", f"回观察位失败：{exc}（扫描可能被挡）")


def reset_bins(design: str) -> None:
    path = ROOT / "data" / "runs" / "bins_state.json"
    if not path.exists():
        say("dim", "没有 bins_state.json，跳过清零")
        return
    d = json.loads(path.read_text())
    for _, counts in d.items():
        for k in counts:
            counts[k] = 0
    path.write_text(json.dumps(d, indent=1, ensure_ascii=False) + "\n")
    say("dim", f"桶计数清零：{json.dumps(d, ensure_ascii=False)}")


def tail_new(path: Path, offset: int) -> tuple[list[dict], int]:
    if not path.exists():
        return [], offset
    with path.open("rb") as f:
        f.seek(offset)
        chunk = f.read()
        offset = f.tell()
    out = []
    for line in chunk.decode("utf-8", "replace").splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out, offset


def narrate(events: list[dict], state: dict) -> None:
    for e in events:
        name = e.get("event")
        if name == "pipeline_start":
            say("bold", f"▶ 开始一轮：桌上 {len(e.get('cubes') or [])} 个块")
        elif name == "pipeline_order":
            order = e.get("order") or []
            say("dim", "  抓取顺序（就近优先）：" + " → ".join(f"{v * 100:.1f} cm" for v in order))
        elif name == "pipeline_sorted":
            state["sorted"] += 1
            say("ok", f"✅ 第 {state['sorted']} 块进桶 {e.get('bin')}（{e.get('category')}），"
                      f"{e.get('seconds')} s —— 绿灯 + 短鸣")
        elif name == "pipeline_retry":
            state["retry"] += 1
            say("warn", f"🔁 夹空 → 黄灯 + 蜂鸣 → 退回重扫：{e.get('reason')}")
        elif name == "pipeline_retry_failed":
            say("bad", f"⚠ 重扫没找回：{e.get('reason')}")
        elif name == "pipeline_missed":
            state["missed"] += 1
            say("bad", f"❌ 夹空（{e.get('reason')}）—— 红灯 + 长鸣")
        elif name == "pipeline_skip":
            state["skipped"] += 1
            say("dim", f"  跳过 {e.get('cube')}：{e.get('reason')}")
        elif name == "pipeline_end":
            say("bold", f"■ 本轮结束：放好 {e.get('sorted')}，跳过 {e.get('skipped')}，"
                        f"夹空 {e.get('missed')}，{e.get('seconds')} s")


def main() -> int:
    ap = argparse.ArgumentParser(description="演示：第二次夹空后自动重试")
    ap.add_argument("--force-miss", type=int, default=0,
                    help="0=不干预（由人把块拨开制造真实夹空）；N=第 N 次合爪强制夹空（自检用）")
    ap.add_argument("--keep-bins", action="store_true", help="不清零桶计数")
    ap.add_argument("--design", default="bins_acrylic_v2_pair")
    ap.add_argument("--dry", action="store_true", help="只做自检和清桶，不跑")
    ap.add_argument("--no-home", action="store_true", help="不先回观察位")
    args = ap.parse_args()

    say("bold", "=== 分拣演示：第一块成功，第二块夹空后自动重试 ===")
    if not preflight():
        say("bad", "自检没过，先修上面几项再跑")
        return 2
    if not args.keep_bins:
        reset_bins(args.design)
    if not args.no_home:
        go_observe()
    if args.dry:
        say("dim", "dry：自检通过，未启动分拣")
        return 0

    offset = LOG.stat().st_size if LOG.exists() else 0
    env = dict(os.environ, ARM_MAP=os.environ.get("ARM_MAP", "map_a4_v4"), ARM_LOG=str(LOG))
    if args.force_miss:
        env["ARM_FORCE_MISS"] = str(args.force_miss)
        say("warn", f"注意：第 {args.force_miss} 次合爪会被强制夹空（自检模式）")
    else:
        say("warn", "真实夹空模式：请在第二块开始下降/合爪时把块拨开 2-3 cm，制造一次真实夹空")
    proc = subprocess.Popen([str(PY), str(ROOT / "src" / "pipeline" / "run.py"), "--bins", args.design],
                            cwd=ROOT, env=env)
    state = dict(sorted=0, skipped=0, missed=0, retry=0)
    started = time.time()
    while proc.poll() is None:
        events, offset = tail_new(LOG, offset)
        narrate(events, state)
        time.sleep(0.4)
    events, offset = tail_new(LOG, offset)
    narrate(events, state)

    print()
    say("bold", "=== 结果 ===")
    say("ok" if state["sorted"] else "bad", f"放好 {state['sorted']} 块 / 跳过 {state['skipped']} / "
                                          f"夹空 {state['missed']} / 重试 {state['retry']} 次 / "
                                          f"用时 {time.time() - started:.0f} s / 退出码 {proc.returncode}")
    if state["retry"] and state["sorted"] >= 2:
        say("ok", "演示成功：第一块正常、第二块夹空后重扫找回并抓住")
    elif state["retry"]:
        say("warn", "重试发生了，但整体没完成两块（检查桌上是否只剩一块，或桶是否满）")
    else:
        say("warn", "没有触发重试：第二块要真的夹空才会重试（把块拨开晚一点/早一点都行，别提前拿走）")
    return 0 if proc.returncode == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
