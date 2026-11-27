"""Shared, dependency-free JSONL input for offline reports. Never silently discard damaged lines."""
import json
import math
from pathlib import Path


def files(inputs, pattern):
    found = []
    for value in inputs:
        p = Path(value)
        candidates = sorted(p.rglob(pattern)) if p.is_dir() else [p]
        for candidate in candidates:
            candidate = candidate.resolve()
            if candidate not in found:
                found.append(candidate)
    if not found:
        raise ValueError(f"no {pattern} files found")
    return found


def read(path):
    rows, warnings = [], []
    def nonfinite(value):
        raise ValueError(f"non-finite JSON number: {value}")
    for number, line in enumerate(Path(path).read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line, parse_constant=nonfinite)
            if not isinstance(row, dict):
                raise ValueError("expected an object")
            rows.append(row)
        except ValueError as exc:
            warnings.append(f"{path}:{number}: {exc}")
    return rows, warnings


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distribution(values):
    import statistics
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), median=round(statistics.median(values), 6),
                p90=round(values[math.ceil(0.9 * len(values)) - 1], 6), max=round(values[-1], 6))


def write_json(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
