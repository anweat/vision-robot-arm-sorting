"""A flow session: one directory with the files the stages hand to each other."""

from __future__ import annotations

import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FLOW_ROOT = ROOT / "data" / "flow"


class FlowSession:
    def __init__(self, path: str | Path | None = None):
        self.path = (Path(path) if path else FLOW_ROOT / time.strftime("%Y%m%d_%H%M%S")).resolve()
        self.path.mkdir(parents=True, exist_ok=True)

    def file(self, name: str) -> Path:
        return self.path / name

    def has(self, name: str) -> bool:
        return self.file(name).exists()

    def save(self, name: str, obj) -> Path:
        p = self.file(name)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(obj, ensure_ascii=False, indent=1, default=_plain), encoding="utf-8")
        return p

    def load(self, name: str):
        p = self.file(name)
        if not p.exists():
            raise FileNotFoundError(f"{p} is missing: run the stage that writes it, or pass the file explicitly")
        return json.loads(p.read_text(encoding="utf-8"))

    def append(self, name: str, row: dict) -> None:
        with self.file(name).open("a", encoding="utf-8") as f:
            f.write(json.dumps(dict(t=round(time.time(), 3), **row), ensure_ascii=False, default=_plain) + "\n")

    def log(self, event: str, **data) -> None:
        self.append("flow.jsonl", dict(event=event, **data))


def load_json(src):
    """A dict, or the JSON in a path."""
    if src is None or isinstance(src, dict):
        return src
    return json.loads(Path(src).read_text(encoding="utf-8"))


def _plain(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serializable: {type(o)}")
