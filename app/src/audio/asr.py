"""WeNet 语音识别（课程任务 2）：把板上 `.om` 的推理包成仓库里的一个入口。

课程要求的六个部分（`pad_sequence` / `resample` / `compute_fbank` / `load_vocab` /
`WeNetASR` / `transcribe`）都在课程提供的 `model.py` 里；本文件只做三件事：找到模型目录、
把 `model.py` 加进 `sys.path` 后加载、给出一个命令行入口。**不复制、不改写**课程源码，
这样它与官方样例逐字节一致，升级时只换 `data/models/wenet_asr/` 里的三个文件。

模型目录需要（课程包 `wenet.zip` 或 Release `assets-v4`）：

    data/models/wenet_asr/
      offline_encoder.om   78 MB，Ascend310B1，输入 speech 1x1478x80 + speech_lengths 1
      vocab.txt            121 KB，索引 -> 汉字
      model.py             课程/官方样例的 WeNetASR（torchaudio + ais_bench）
      sample.wav           14.78 s 的样例音频

    python3 src/audio/asr.py                                  # 转写默认样例
    python3 src/audio/asr.py --wav data/models/wenet_asr/scan_w0.wav --runs 3

单次音频上限 1478 帧 = 14.78 s；`model.py` 里超出会截断、不足会补齐。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIR = ROOT / "data" / "models" / "wenet_asr"
SAMPLES = ("sample.wav",)


class AsrUnavailable(RuntimeError):
    """模型目录不完整，或者这台机器上没有 ais_bench / torchaudio。"""


def model_dir(path: str | Path | None = None) -> Path:
    d = Path(path) if path else DEFAULT_DIR
    missing = [n for n in ("offline_encoder.om", "vocab.txt", "model.py") if not (d / n).is_file()]
    if missing:
        raise AsrUnavailable(f"{d} 缺少 {', '.join(missing)}（见 docs/atlas-200i-dk-a2/19 或 Release assets-v4）")
    return d


def load(path: str | Path | None = None):
    """返回课程给的 WeNetASR 实例；`model.py` 从模型目录导入，不修改它。"""
    d = model_dir(path)
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))
    try:
        from model import WeNetASR  # type: ignore  # 课程/官方样例随模型目录提供
    except Exception as exc:                                   # noqa: BLE001
        raise AsrUnavailable(f"导入 {d / 'model.py'} 失败：{type(exc).__name__}: {exc}") from exc
    return WeNetASR(str(d / "offline_encoder.om"), str(d / "vocab.txt"))


def transcribe(wav: str | Path, path: str | Path | None = None) -> dict:
    """转写一个 wav，返回 {text, seconds, wav, dir}。"""
    wav = Path(wav)
    if not wav.is_file():
        raise AsrUnavailable(f"找不到音频：{wav}")
    asr = load(path)
    t0 = time.perf_counter()
    text = asr.transcribe(str(wav))
    return dict(text=text, seconds=round(time.perf_counter() - t0, 4), wav=str(wav),
                dir=str(model_dir(path)))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wav", default=str(DEFAULT_DIR / SAMPLES[0]), help="要转写的 wav（默认样例）")
    p.add_argument("--dir", help="模型目录（默认 data/models/wenet_asr）")
    p.add_argument("--runs", type=int, default=1, help="重复次数，用于看每次文本是否一致")
    args = p.parse_args(argv)
    try:
        results = [transcribe(args.wav, args.dir) for _ in range(max(1, args.runs))]
    except AsrUnavailable as exc:
        print(f"语音识别不可用：{exc}", file=sys.stderr)
        return 2
    texts = {r["text"] for r in results}
    print(json.dumps(dict(wav=results[0]["wav"], runs=len(results),
                          seconds_median=sorted(r["seconds"] for r in results)[len(results) // 2],
                          same_text=len(texts) == 1, text=results[0]["text"]),
                     ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
