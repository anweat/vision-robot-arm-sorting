"""语音 / 手势识别：把音频或图像变成指令，再交给 serve.py 经现有 /api/cmd 下发。

设计原则（与调研文档 19/20 节对齐）：

1. **识别结果走现有 cmd 接口**：/api/cmd 已预留 source=voice/gesture（接口文档 §8.2），
   这里只负责"把输入变成识别结果"，运动串行化、限位、审计事件全复用 _cmd，不在别处开口子。
2. **模型只能在 Atlas 板上跑**：WeNet `offline_encoder.om` 与 hrnet.om 都依赖
   `ais_bench`（Ascend 运行时），开发机 / 仿真环境没有。所以本模块顶层只依赖标准库，
   numpy / cv2 / ais_bench / 官方 model.py 都在函数里延迟 import；模型未配置或加载失败
   时抛 `ModelUnavailable`，由调用方转成 501，绝不假装识别出了什么。
3. **映射是纯函数**：语音文本→cmd、手势类别→cmd 不依赖模型，可单独单测；即使模型没部署，
   前端传 `text` / `label` 也能联调整条链路（映射→下发→写 interaction 事件）。
"""
from __future__ import annotations

import base64
from pathlib import Path

# 手势类别 → 固定动作。4 类都映射到"安全动作"：合爪 / 松爪 / 启停，不直接下发大角度运动，
# 避免误识别触发大幅动作。调研 20 节提到现场 fist 常被背景干扰判成 palm，阈值要保守（0.6）。
GESTURE_LABELS = ("fist", "like", "ok", "palm")
GESTURE_COMMANDS: dict[str, dict] = {
    "fist": {"cmd": "gripper", "action": "grip"},   # 握拳 = 合爪抓取
    "palm": {"cmd": "gripper", "action": "open"},   # 张开 = 松爪放下
    "like": {"cmd": "start"},                        # 点赞 = 启动一轮分拣
    "ok":   {"cmd": "stop"},                         # OK = 停止
}

# 语音关键词 → cmd。对贪心 CTC 输出做子串匹配，不接语言模型（调研 19 节）。
# 顺序从"最安全"到"最动作"：先判停止，再启动，再跳舞。命中第一个即返回。
VOICE_KEYWORDS: tuple[tuple[tuple[str, ...], dict | None], ...] = (
    (("停止", "停下", "停一下", "别动", "stop"), {"cmd": "stop"}),
    (("启动", "开始", "开始分拣", "干活", "start"), {"cmd": "start"}),
    (("跳舞", "跳个舞", "跳支舞", "来个舞蹈", "dance"), {"cmd": "run_preset", "preset": "dance"}),
    # 复位/归位暂无执行侧映射（run_preset 待硬件组），先忽略，避免误触发未接入的动作
    (("复位", "归位", "回零"), None),
)


class ModelUnavailable(Exception):
    """模型未部署：没传 --asr-dir / --gesture-dir、目录里缺文件、或当前环境没有 ais_bench。

    调用方应转成 501 并把原因原样告诉前端，不能静默猜一个识别结果。
    """


# ---- 输入解码 --------------------------------------------------------------

def _decode_b64(data) -> bytes:
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if not isinstance(data, str):
        raise ValueError("audio / image 应为 base64 字符串")
    try:
        return base64.b64decode(data, validate=False)
    except Exception as exc:                       # noqa: BLE001 - 统一转成 ValueError
        raise ValueError("base64 解码失败") from exc


# ---- 纯映射（不依赖模型，可单测）-------------------------------------------

def voice_command(text: str) -> dict | None:
    """识别文本 → cmd body；没命中任何关键词返回 None。"""
    if not isinstance(text, str):
        return None
    t = text.strip().lower()
    if not t:
        return None
    for keywords, cmd in VOICE_KEYWORDS:
        if any(k.lower() in t for k in keywords):
            return dict(cmd) if cmd is not None else None
    return None


def gesture_command(label: str) -> dict | None:
    """手势类别 → cmd body；未知类别返回 None。"""
    cmd = GESTURE_COMMANDS.get(label)
    return dict(cmd) if cmd is not None else None


# ---- 板上推理（延迟 import，只在 Atlas 板可用）-----------------------------

def _check_model_dir(model_dir, *names: str) -> Path:
    d = Path(model_dir)
    missing = [n for n in names if not (d / n).is_file()]
    if missing:
        raise ModelUnavailable(
            f"模型未部署：目录 {d} 里缺 {', '.join(missing)}（启动时加对应 --xxx-dir <目录>）")
    return d


def transcribe(audio: bytes, model_dir) -> tuple[str, float | None]:
    """板上 WeNet 推理：wav 字节 → (识别文本, 置信度)。

    置信度贪心 CTC 不给直接值，返回 None。失败抛 ModelUnavailable / ValueError。
    官方前后处理封装在模型目录自带的 model.py（WeNetASR）里，动态加到 sys.path。
    """
    audio = _decode_b64(audio)
    if not audio:
        raise ValueError("没有音频数据")
    d = _check_model_dir(model_dir, "offline_encoder.om", "vocab.txt")
    import os
    import sys
    import tempfile
    try:
        if str(d) not in sys.path:
            sys.path.insert(0, str(d))
        from model import WeNetASR  # type: ignore   # 板上模型目录自带
    except ImportError as exc:
        raise ModelUnavailable(
            f"无法加载语音推理依赖（ais_bench / numpy / torchaudio 或官方 model.py 缺失）：{exc}") from exc
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    try:
        tmp.write(audio)
        tmp.close()
        asr = WeNetASR(str(d / "offline_encoder.om"), str(d / "vocab.txt"))
        feats, lengths = asr.preprocess(tmp.name)
        output = asr.model.infer([feats, lengths])
        text = asr.post_process(output)
        if isinstance(text, (list, tuple)):
            text = text[0] if text else ""
        return str(text or "").strip(), None
    except ModelUnavailable:
        raise
    except Exception as exc:                           # noqa: BLE001 - 推理失败统一报模型不可用
        raise ModelUnavailable(f"语音推理失败：{exc}") from exc
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass


def _read_labels(d: Path) -> list[str]:
    p = d / "labels.txt"
    if p.is_file():
        try:
            rows = [x.strip() for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]
            if rows:
                return rows
        except OSError:
            pass
    return list(GESTURE_LABELS)


def classify(image: bytes, model_dir, threshold: float = 0.6) -> tuple[str, float, list[float]]:
    """板上 hrnet 推理：图像字节 → (类别, 置信度, 各类概率)。

    低于 threshold 时类别返回空串（调用方视为"未识别到手势"，不下发）。
    预处理与课程 notebook 一致：Resize(224) + RGB/255 + NCHW，无归一化。
    """
    image = _decode_b64(image)
    if not image:
        raise ValueError("没有图像数据")
    d = _check_model_dir(model_dir, "hrnet.om")
    try:
        import cv2
        import numpy as np
        from ais_bench.infer.interface import InferSession
    except ImportError as exc:
        raise ModelUnavailable(
            f"无法加载手势推理依赖（ais_bench / numpy / cv2 未安装）：{exc}") from exc
    buf = np.frombuffer(image, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("图像解码失败（需 jpg / png 字节）")
    resized = cv2.resize(img, (224, 224), interpolation=cv2.INTER_LINEAR)
    rgb = resized[..., ::-1].astype(np.float32) / 255.0
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])
    try:
        session = InferSession(0, str(d / "hrnet.om"))
        logits = np.asarray(session.infer([tensor])[0]).reshape(-1)
    except ModelUnavailable:
        raise
    except Exception as exc:                           # noqa: BLE001
        raise ModelUnavailable(f"手势推理失败：{exc}") from exc
    e = np.exp(logits - logits.max())
    probs = (e / e.sum()).tolist()
    idx = int(np.argmax(probs))
    conf = float(probs[idx])
    labels = _read_labels(d)
    label = labels[idx] if idx < len(labels) else GESTURE_LABELS[idx % len(GESTURE_LABELS)]
    if conf < threshold:
        label = ""                                     # 低于阈值：当未识别到，不映射动作
    return label, conf, [round(float(p), 4) for p in probs]


def grab_snapshot(camera_url: str, timeout: float = 5.0) -> bytes:
    """从板载相机服务取一帧 jpg（gesture_capture.py 同一路径 /snapshot.jpg）。

    serve.py 与相机服务可能不同机，地址用 --camera-url 配；没配就不启用取帧。
    """
    import urllib.request
    try:
        with urllib.request.urlopen(camera_url, timeout=timeout) as res:
            return res.read()
    except Exception as exc:                           # noqa: BLE001
        raise ModelUnavailable(f"取不到相机帧（{camera_url}）：{exc}") from exc
