"""Summarise a teleop session directory so it can be reviewed without reading raw logs or every frame.

Writes `analysis.md` (also printed) and, with OpenCV available, `contact_sheet.jpg` of the key moments
(grips, samples, episode start/end) into the session directory. Read-only apart from those two files.

    python3 dev/tools/analyze_session.py data/teleop/20260925_001846_4a2a
    python3 dev/tools/analyze_session.py --fetch latest          # copy the newest session from the board first
    uv run --with 'opencv-contrib-python<5' --with numpy python dev/tools/analyze_session.py --fetch latest --sheet

Heights: the "dz" column compares the kinematic tip height with where a gripped 3 cm block's centre
should be (default: the table estimate keep_out.table_surface_z_m in app/config/arm_limits.json + 1.5 cm).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("adapters", "spatial"):
    sys.path.insert(0, str(ROOT / "dev" / sub))
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
import safe_arm  # noqa: E402
import workspace  # noqa: E402

BOARD = "root@192.168.137.100"
BOARD_DIR = "/root/arm-app/data/teleop"
LOCAL_DIR = ROOT / "data" / "teleop"
SSH = ["ssh", "-o", "ControlPath=~/.ssh/cm-%C", "-o", "ConnectTimeout=5", BOARD]


def fetch(name: str) -> Path:
    """Copy one session from the board (tar over the existing SSH control connection)."""
    if name == "latest":
        name = subprocess.run(SSH + [f"ls -t {BOARD_DIR} | head -1"], check=True, capture_output=True,
                              text=True).stdout.strip()
    if not name or "/" in name or name.startswith("."):
        raise SystemExit(f"bad session name: {name!r}")
    LOCAL_DIR.mkdir(parents=True, exist_ok=True)
    tar = subprocess.Popen(SSH + [f"cd {BOARD_DIR} && tar czf - {name}"], stdout=subprocess.PIPE)
    subprocess.run(["tar", "xzf", "-", "-C", str(LOCAL_DIR)], stdin=tar.stdout, check=True)
    if tar.wait():
        raise SystemExit("fetch failed")
    return LOCAL_DIR / name


CONTACT_KINDS = {"platform_contact": "平台接触", "table_contact": "夹爪碰地", "boundary_clear": "接近未接触"}


def load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def tip(q) -> dict:
    fk = kin.forward(list(q)[:4])
    x, y, z = (v * 100 for v in fk["tip"])
    return dict(x=x, y=y, z=z, reach=math.hypot(x, y), pitch=fk["tip_pitch_deg"][0])


def fmt_tip(t: dict) -> str:
    return f"({t['x']:.1f}, {t['y']:.1f}, {t['z']:.1f}) cm, pitch {t['pitch']:.0f}°"


def short(text: str, n: int = 90) -> str:
    text = str(text).replace("\n", " ")
    return text if len(text) <= n else text[: n - 1] + "…"


def analyse(session: Path, gripper: dict, block_z_cm: float) -> tuple[str, list[dict]]:
    events = load_jsonl(session / "events.jsonl")
    if not events:
        raise SystemExit(f"no events in {session}")
    t0 = events[0]["t"]
    rel = lambda t: t - t0  # noqa: E731
    out = [f"# 会话 {session.name}", ""]
    kinds = Counter(e["event"] for e in events)
    reads = [e for e in events if e["event"] == "readback"]
    step_kinds = Counter(e.get("kind", "?") for e in reads)
    out += [f"- 时长 {rel(events[-1]['t']) / 60:.1f} min；事件 {len(events)} 条："
            + "，".join(f"{k} {v}" for k, v in kinds.most_common()),
            f"- 执行步数 {len(reads)}：" + "，".join(f"{k} {v}" for k, v in step_kinds.most_common()),
            f"- 档位：" + "，".join(f"{k} {v}" for k, v in Counter(e.get("profile") for e in reads).items())]
    if reads:
        out.append(f"- 起始读回 {reads[0]['observed']}，结束读回 {reads[-1]['observed']}")

    # problems, grouped so repeated refusals take one line
    out += ["", "## 故障、阻力与拒绝"]
    faults = [e for e in events if e["event"] == "fault"]
    holds = [e for e in events if e["event"] == "hold_at_readback"]
    for e in faults:
        out.append(f"- {rel(e['t']):6.1f}s 故障（锁定）：{short(e.get('error', ''), 120)}")
    for e in holds:
        out.append(f"- {rel(e['t']):6.1f}s 遇阻停在读回 {e.get('hold')}")
    rejected = Counter(short(e.get("error", ""), 100) for e in events if e["event"] == "plan_rejected")
    for msg, n in rejected.most_common():
        out.append(f"- 拒绝 ×{n}：{msg}")
    if not (faults or holds or rejected):
        out.append("- 无")

    # gripper actions, re-classified with the current config
    out += ["", "## 夹爪动作（按当前配置重新判定）",
            "| 时间 s | 动作 | J6 前→读回 | 重新判定 | 记录时判定 | 关节 1–4 | 正解夹爪尖 |", "|---|---|---|---|---|---|---|"]
    grasps = []
    for i, e in enumerate(events):
        if e["event"] != "gripper":
            continue
        before = next((x["observed"] for x in reversed(events[:i - 1]) if x["event"] == "readback"), None)
        pose = next((x["observed"] for x in reversed(events[:i]) if x["event"] == "readback"), None)
        state = safe_arm.grip_state(e["readback"], gripper) if e["action"] == "grip" else "—"
        logged = e.get("result") or ("holding" if e.get("holding") else "not holding") if e["action"] == "grip" else "—"
        t = tip(pose) if pose else None
        out.append(f"| {rel(e['t']):.1f} | {e['action']} {e['command']}° | {before[5] if before else '?'}→{e['readback']} "
                   f"| {state} | {logged} | {pose[:4] if pose else '?'} | {fmt_tip(t) if t else '?'} |")
        if state == "holding" and pose and (not grasps or grasps[-1]["q"] != pose[:4]):
            grasps.append(dict(t=rel(e["t"]), q=pose[:4], readback=e["readback"], **t))

    if grasps:
        out += ["", f"## 夹住时的正解高度（参考：物块中心 {block_z_cm:.1f} cm）",
                "| 时间 s | 关节 1–4 | 读回 | 夹爪尖 x, y, z cm | 伸出 cm | 俯角 | dz cm |", "|---|---|---|---|---|---|---|"]
        for g in grasps:
            out.append(f"| {g['t']:.1f} | {g['q']} | {g['readback']} | {g['x']:.1f}, {g['y']:.1f}, {g['z']:.1f} "
                       f"| {g['reach']:.1f} | {g['pitch']:.0f}° | {g['z'] - block_z_cm:+.1f} |")

    samples = load_jsonl(session / "samples.jsonl")
    if not samples:
        samples = [json.loads(p.read_text()) for p in sorted(session.glob("sample_*.json"))]
    out += ["", "## 采样与人工校验"]
    for s in samples:
        pose = s.get("state", {}).get("pose")
        t = tip(pose) if pose and all(v is not None for v in pose[:4]) else None
        note = f"；备注：{short(s['note'])}" if s.get("note") else ""
        out.append(f"- {rel(s['t']):6.1f}s {s.get('kind')} · {s.get('label')}{note} · 读回 {pose} · "
                   f"{fmt_tip(t) if t else ''} · 图 {s.get('phone_image') or s.get('image')}")
    if not samples:
        out.append("- 无")

    contacts = [s for s in samples if s.get("kind") in CONTACT_KINDS and s.get("state", {}).get("pose")]
    if contacts:
        keep_out = workspace.load_keep_out(safe_arm.load_config(safe_arm.DEFAULT_CONFIG))
        out += ["", "## 接触与边界样本（标准禁入区模型对照；正解高度有偏差，见上表）",
                "| 时间 s | 人工观察 | 关节 1–4 | 正解夹爪尖 | 标准模型余量 cm |", "|---|---|---|---|---|"]
        for s in contacts:
            pose = s["state"]["pose"]
            margin = workspace.clearance(pose, keep_out) * 100
            out.append(f"| {rel(s['t']):.1f} | {CONTACT_KINDS[s['kind']]} | {pose[:4]} | {fmt_tip(tip(pose))} | {margin:+.1f} |")

    out += ["", "## 录制"]
    frames = []
    for s in samples:
        frames.append(dict(t=rel(s["t"]), label=f"sample {s.get('kind')}", phone=s.get("phone_image"),
                           board=s.get("image"), pose=s.get("state", {}).get("pose")))
    for ep in sorted(session.glob("episode_*")):
        meta = json.loads((ep / "episode.json").read_text()) if (ep / "episode.json").exists() else {}
        steps = load_jsonl(ep / "steps.jsonl")
        grips = [s for s in steps if s.get("kind") == "gripper"]
        dur = (steps[-1]["t"] - steps[0]["t"]) / 60 if steps else 0
        ended = "已结束" if "ended" in meta else "未结束"
        text = "；".join(x for x in (meta.get("label"), meta.get("note"), meta.get("end_note")) if x)
        out.append(f"- {ep.name}：{len(steps)} 步，{dur:.1f} min，{ended}{('；' + short(text)) if text else ''}；"
                   f"夹爪步 {[s['index'] for s in grips]}")
        keep = ([steps[0]] if steps else []) + grips + ([steps[-1]] if len(steps) > 1 else [])
        keep = list({s["index"]: s for s in keep}.values())  # the first/last step may also be a grip
        for s in keep:
            frames.append(dict(t=rel(s["t"]), label=f"{ep.name[-6:]}#{s['index']} {s.get('kind')}",
                               phone=f"{ep.name}/{s['phone_image']}" if s.get("phone_image") else None,
                               board=f"{ep.name}/{s['board_image']}" if s.get("board_image") else None,
                               pose=s.get("observed")))
    if not list(session.glob("episode_*")):
        out.append("- 无")
    frames.sort(key=lambda f: f["t"])
    return "\n".join(out) + "\n", frames


def contact_sheet(session: Path, frames: list[dict], limit: int) -> Path | None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("（未安装 OpenCV，跳过缩略图）", file=sys.stderr)
        return None
    if len(frames) > limit:  # keep first/last, thin the middle evenly
        idx = sorted({round(i * (len(frames) - 1) / (limit - 1)) for i in range(limit)})
        frames = [frames[i] for i in idx]
    tiles = []
    for f in frames:
        tile = np.full((300, 480, 3), 24, np.uint8)
        phone = cv2.imread(str(session / f["phone"])) if f.get("phone") else None
        board = cv2.imread(str(session / f["board"])) if f.get("board") else None
        if phone is not None:
            tile[30:300] = cv2.resize(phone, (480, 270), interpolation=cv2.INTER_AREA)
        if board is not None:
            inset = cv2.resize(board, (160, 120), interpolation=cv2.INTER_AREA)
            tile[300 - 122:298, 480 - 162:478] = inset
            cv2.rectangle(tile, (480 - 163, 300 - 123), (479, 299), (0, 220, 255), 1)
        pose = f.get("pose")
        text = f"{f['t']:.0f}s {f['label']}  {pose if pose else ''}"
        cv2.putText(tile, text[:62], (6, 21), cv2.FONT_HERSHEY_SIMPLEX, .5, (230, 230, 230), 1, cv2.LINE_AA)
        tiles.append(tile)
    if not tiles:
        return None
    cols = min(4, len(tiles))
    while len(tiles) % cols:
        tiles.append(np.full_like(tiles[0], 12))
    rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    path = session / "contact_sheet.jpg"
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="*", type=Path, help="session directories")
    parser.add_argument("--fetch", metavar="NAME", help="copy a session from the board first ('latest' for the newest)")
    parser.add_argument("--sheet", action="store_true", help="also write contact_sheet.jpg (needs OpenCV)")
    parser.add_argument("--frames", type=int, default=12, help="max tiles in the contact sheet")
    parser.add_argument("--block-center-z-cm", type=float,
                        help="default: keep_out.table_surface_z_m in app/config/arm_limits.json + 1.5 cm")
    args = parser.parse_args()
    sessions = list(args.sessions)
    if args.fetch:
        sessions.append(fetch(args.fetch))
    if not sessions:
        parser.error("give a session directory or --fetch")
    config = safe_arm.load_config(safe_arm.DEFAULT_CONFIG)
    gripper = config["gripper"]
    if args.block_center_z_cm is None:
        args.block_center_z_cm = config["keep_out"].get("table_surface_z_m", -0.035) * 100 + 1.5
    for session in sessions:
        text, frames = analyse(session, gripper, args.block_center_z_cm)
        if args.sheet:
            sheet = contact_sheet(session, frames, max(2, args.frames))
            if sheet:
                text += f"\n缩略图：{sheet.relative_to(ROOT) if sheet.is_relative_to(ROOT) else sheet}（{min(len(frames), args.frames)} 帧，大图为手机，右下角小图为板载）\n"
        (session / "analysis.md").write_text(text)
        print(text)


if __name__ == "__main__":
    main()
