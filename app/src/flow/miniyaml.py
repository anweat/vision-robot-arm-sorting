"""A small YAML subset, for flow scripts on machines without PyYAML (the board is not known to have it).

`load(text)` uses PyYAML when it is installed; otherwise this parser, which understands what the scripts in
config/flows/ use and refuses the rest loudly rather than guessing:

  * block mappings `key: value` and block sequences `- item`, nested by indentation (spaces only);
  * a sequence item that starts a mapping (`- step: scan` with more keys indented under it);
  * flow collections `[a, b]`, `{k: v}` (nested), on one line;
  * scalars: int, float, true/false, null/~, 'single' / "double" quoted or plain strings;
  * `# comments` (outside quotes) and blank lines.

Not supported (ValueError): anchors/aliases, tags, multi-line strings (| >), multiple documents.
"""
from __future__ import annotations

import re

_INT = re.compile(r"^[-+]?\d+$")
_FLOAT = re.compile(r"^[-+]?(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?$")


def load(text: str):
    try:
        import yaml  # type: ignore
    except ImportError:
        return parse(text)
    return yaml.safe_load(text)


def parse(text: str):
    lines = []
    for n, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise ValueError(f"line {n}: tabs in indentation")
        body = _strip_comment(raw).rstrip()
        if not body.strip() or body.strip() == "---":
            continue
        if body.lstrip()[:1] in ("&", "*", "!", "|", ">") or re.search(r":\s*[|>&*!]", body):
            raise ValueError(f"line {n}: anchors, tags and block strings are not supported: {raw.strip()}")
        lines.append((len(body) - len(body.lstrip()), body.strip(), n))
    if not lines:
        return None
    value, i = _block(lines, 0, lines[0][0])
    if i != len(lines):
        raise ValueError(f"line {lines[i][2]}: unexpected indentation")
    return value


def _strip_comment(s: str) -> str:
    quote = None
    for i, ch in enumerate(s):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == "#" and (i == 0 or s[i - 1] in " \t"):
            return s[:i]
    return s


def _block(lines, i, indent):
    if lines[i][1].startswith("- ") or lines[i][1] == "-":
        return _seq(lines, i, indent)
    return _map(lines, i, indent)


def _seq(lines, i, indent):
    out = []
    while i < len(lines) and lines[i][0] == indent and (lines[i][1].startswith("- ") or lines[i][1] == "-"):
        rest = lines[i][1][1:].strip()
        n = lines[i][2]
        if not rest:                                   # "-" alone: the item is the indented block below
            i += 1
            if i >= len(lines) or lines[i][0] <= indent:
                out.append(None)
                continue
            value, i = _block(lines, i, lines[i][0])
            out.append(value)
        elif rest.startswith("- ") or rest == "-":     # "- - x": a sequence inside a sequence
            inner = indent + 2
            value, used = _seq([(inner, rest, n)] + lines[i + 1:], 0, inner)
            out.append(value)
            i += used
        elif _is_key(rest):                            # "- key: ..." starts a mapping at indent + 2
            inner = indent + 2
            sub = [(inner, rest, n)] + [ln for ln in lines[i + 1:]]
            value, used = _map(sub, 0, inner)
            out.append(value)
            i += used                                  # used counts the synthetic first line too
        else:
            out.append(_scalar_or_flow(rest, n))
            i += 1
    if i < len(lines) and lines[i][0] > indent:
        raise ValueError(f"line {lines[i][2]}: unexpected indentation")
    return out, i


def _map(lines, i, indent):
    out = {}
    while i < len(lines) and lines[i][0] == indent and not lines[i][1].startswith("- "):
        text, n = lines[i][1], lines[i][2]
        if not _is_key(text):
            raise ValueError(f"line {n}: expected 'key: value', got {text!r}")
        key, rest = _split_key(text)
        if key in out:
            raise ValueError(f"line {n}: duplicate key {key!r}")
        i += 1
        if rest:
            out[key] = _scalar_or_flow(rest, n)
        elif i < len(lines) and (lines[i][0] > indent or (lines[i][0] == indent and lines[i][1].startswith("- "))):
            out[key], i = _block(lines, i, lines[i][0])
        else:
            out[key] = None
    if i < len(lines) and lines[i][0] > indent:
        raise ValueError(f"line {lines[i][2]}: unexpected indentation")
    return out, i


def _is_key(text: str) -> bool:
    if text[:1] in "[{'\"":
        return False
    m = re.match(r"^([^:]+?):(\s|$)", text)
    return bool(m)


def _split_key(text: str):
    m = re.match(r"^([^:]+?):(\s+(.*))?$", text)
    return m.group(1).strip(), (m.group(3) or "").strip()


def _scalar_or_flow(text: str, n: int):
    if text[:1] in "[{":
        value, pos = _flow(text, 0, n)
        if text[pos:].strip():
            raise ValueError(f"line {n}: trailing text after a flow collection: {text[pos:]!r}")
        return value
    return _scalar(text, n)


def _flow(s: str, pos: int, n: int):
    """Parse a flow collection starting at s[pos] ('[' or '{'); returns (value, next position)."""
    opening = s[pos]
    closing = "]" if opening == "[" else "}"
    pos += 1
    items, mapping = [], {}
    while True:
        pos = _skip(s, pos)
        if pos >= len(s):
            raise ValueError(f"line {n}: unclosed {opening}")
        if s[pos] == closing:
            return (items if opening == "[" else mapping), pos + 1
        if opening == "{":
            key, pos = _flow_atom(s, pos, n, stop=":")
            pos = _skip(s, pos)
            if pos >= len(s) or s[pos] != ":":
                raise ValueError(f"line {n}: expected ':' in a flow mapping")
            value, pos = _flow_atom(s, _skip(s, pos + 1), n, stop=",}")
            mapping[str(key)] = value
        else:
            value, pos = _flow_atom(s, pos, n, stop=",]")
            items.append(value)
        pos = _skip(s, pos)
        if pos < len(s) and s[pos] == ",":
            pos += 1


def _flow_atom(s: str, pos: int, n: int, stop: str):
    if pos < len(s) and s[pos] in "[{":
        return _flow(s, pos, n)
    if pos < len(s) and s[pos] in "'\"":
        q = s[pos]
        end = s.find(q, pos + 1)
        if end < 0:
            raise ValueError(f"line {n}: unclosed quote")
        return s[pos + 1:end], end + 1
    end = pos
    while end < len(s) and s[end] not in stop:
        end += 1
    return _scalar(s[pos:end].strip(), n), end


def _skip(s: str, pos: int) -> int:
    while pos < len(s) and s[pos] == " ":
        pos += 1
    return pos


def _scalar(text: str, n: int):
    if text[:1] in "'\"":
        if len(text) < 2 or text[-1] != text[0]:
            raise ValueError(f"line {n}: bad quoted string {text!r}")
        return text[1:-1]
    low = text.lower()
    if low in ("null", "~", ""):
        return None
    if low == "true":
        return True
    if low == "false":
        return False
    if _INT.match(text):
        return int(text)
    if _FLOAT.match(text):
        return float(text)
    return text
