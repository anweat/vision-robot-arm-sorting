"""Turns what the voice and gesture recognisers say into commands, and nothing else. Standard library only.

A recogniser (on the board or on the development machine) only reports text or a label with a probability.
This module decides whether that is a command: keyword match, probability threshold, N consecutive frames,
cooldown and the "armed" switch. It does not move anything; the caller supplies `execute(command, preset)`,
which either does the thing or raises ValueError / NotImplementedError with the reason shown to the user.

Wire format and rules: docs/atlas-200i-dk-a2/25-语音手势接口.md. Mapping and thresholds: config/interaction.json.
"""
from __future__ import annotations

import json
import re
import threading
import time
from collections import deque
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[2] / "config" / "interaction.json"
STATUSES = ("accepted", "ignored", "rejected")
_PUNCT = re.compile(r"[\s，。！？、,.!?;；:：\"'“”‘’]+")


def load_config(path: Path = CONFIG) -> dict:
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    for source in ("voice", "gesture"):
        if source not in cfg:
            raise ValueError(f"interaction config lacks {source}")
    return cfg


def _split(key: str) -> tuple[str, str | None]:
    command, _, preset = key.partition(":")
    return command, preset or None


class Interaction:
    """State of the voice/gesture side: armed switch, gesture streak, cooldowns, the last recognitions."""

    def __init__(self, execute, config: dict | None = None, clock=time.time, keep: int = 50):
        self.cfg = config or load_config()
        self.execute, self.clock = execute, clock
        self.lock = threading.Lock()
        self.armed = False                  # a recogniser starts unarmed: one explicit act arms it (stop always works)
        self.seq = 0
        self.recent: deque = deque(maxlen=keep)
        self.streak = (None, 0)             # (gesture label, consecutive frames at/above the threshold)
        self.latched = False                # this streak already fired; a different label or a gap re-opens it
        self.last_fire: dict[str, float] = {}

    # ---- public ------------------------------------------------------------------------------------------
    def describe(self) -> dict:
        g = self.cfg["gesture"]
        return dict(armed=self.armed, always_allowed=self.cfg.get("always_allowed", []),
                    voice=dict(words=self.cfg["voice"]["words"], cooldown_s=self.cfg["voice"]["cooldown_s"]),
                    gesture=dict(labels=g["labels"], map=g["map"], threshold=g["threshold"], frames=g["frames"],
                                 cooldown_s=g["cooldown_s"]),
                    streak=dict(label=self.streak[0], frames=self.streak[1]), recent=list(self.recent))

    def arm(self, armed: bool) -> dict:
        with self.lock:
            self.armed = bool(armed)
            self.streak, self.latched = (None, 0), False
            return self._note(dict(source="system", input="arm" if armed else "disarm", status="accepted",
                                   command=None, reason="已武装" if armed else "已解除武装"))

    def voice(self, text) -> dict:
        """One recognised sentence."""
        if not isinstance(text, str) or len(text) > 500:
            raise ValueError("text 须为不超过 500 字的字符串")
        v = self.cfg["voice"]
        flat = _PUNCT.sub("", text)
        hits: dict[str, str] = {}           # command key -> the word that matched
        for key, words in v["words"].items():
            for w in words:
                if w in flat:
                    hits.setdefault(key, w)
        base = dict(source="voice", input=text)
        with self.lock:
            if not hits:
                return self._note(dict(base, status="ignored", command=None, reason="没有命中指令词"))
            stops = [k for k in hits if _split(k)[0] == "stop"]
            if stops:                       # a stop word wins over everything else in the sentence
                key = stops[0]
            elif len(hits) > 1:
                return self._note(dict(base, status="ignored", command=None,
                                       reason="同一句命中多条指令（" + "、".join(hits.values()) + "），不执行"))
            else:
                key = next(iter(hits))
            return self._fire(base, key, hits[key], v["cooldown_s"], detail=text)

    def gesture(self, label, confidence) -> dict:
        """One classified frame. Fires once per unbroken run of `frames` frames of the same mapped label."""
        g = self.cfg["gesture"]
        if label not in g["labels"]:
            raise ValueError("label 须为 " + "/".join(g["labels"]))
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1:
            raise ValueError("confidence 须为 0–1 的数字")
        base = dict(source="gesture", input=label, confidence=round(float(confidence), 4))
        with self.lock:
            if confidence < g["threshold"]:
                self.streak, self.latched = (None, 0), False
                return self._note(dict(base, status="ignored", command=None, reason=f"置信度低于 {g['threshold']}"), quiet=True)
            name, n = self.streak
            n = n + 1 if name == label else 1
            if name != label:
                self.latched = False
            self.streak = (label, n)
            mapped = g["map"].get(label)
            if mapped is None:
                return self._note(dict(base, status="ignored", command=None, reason="这个手势没有绑定指令（待需求方确认）"), quiet=True)
            if n < g["frames"]:
                return self._note(dict(base, status="ignored", command=None, reason=f"连续 {n}/{g['frames']} 帧"), quiet=True)
            if self.latched:
                return self._note(dict(base, status="ignored", command=None, reason="本次保持已触发过"), quiet=True)
            self.latched = True
            return self._fire(base, mapped, label, g["cooldown_s"], detail=label, confidence=base["confidence"])

    # ---- internals (lock held) ---------------------------------------------------------------------------
    def _fire(self, base, key, word, cooldown, detail, confidence=None) -> dict:
        command, preset = _split(key)
        base = dict(base, command=command, preset=preset, matched=word)
        now = self.clock()
        if command not in self.cfg.get("always_allowed", []):
            if not self.armed:
                return self._note(dict(base, status="rejected", reason="未武装：先 POST /api/interact/arm"))
            if now - self.last_fire.get(key, -1e9) < cooldown:
                return self._note(dict(base, status="ignored", reason="冷却中"))
        self.last_fire[key] = now
        try:
            info = self.execute(command, preset)
        except NotImplementedError as exc:
            return self._note(dict(base, status="rejected", reason=str(exc), implemented=False))
        except ValueError as exc:
            return self._note(dict(base, status="rejected", reason=str(exc)))
        event = dict(t=now, event="interaction", source=base["source"], command=command, detail=detail)
        if confidence is not None:
            event["confidence"] = confidence        # voice has no confidence: the field is simply absent
        return self._note(dict(base, status="accepted", reason=info or "已执行", event=event))

    def _note(self, item: dict, quiet: bool = False) -> dict:
        """Keep the recognition for the page. Per-frame gesture noise (`quiet`) is not kept unless it fired."""
        self.seq += 1
        item = dict(item, id=self.seq, t=self.clock())
        if not quiet:
            self.recent.append(item)
        return item
