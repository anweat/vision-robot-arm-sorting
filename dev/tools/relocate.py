#!/usr/bin/env python3
"""Move tracked files with `git mv` and rewrite every path that points at them.

    python3 dev/tools/relocate.py moves.tsv            # dry run: what would move, which files would change
    python3 dev/tools/relocate.py moves.tsv --apply    # do it

moves.tsv: one `old<TAB>new` per line (file or directory, repo-relative; `#` starts a comment).
A directory entry moves every tracked file under it; a more specific entry wins over a directory entry.

Rewritten, in every tracked text file (md/py/js/sh/...):
  1. Markdown links and `src=`/`href=` targets in .md/.html: resolved against the file's old location, then
     recomputed from its new location to the target's new location. Only targets that are tracked files or
     directories are touched; URLs, anchors and anything outside the repo stay as written.
  2. Repo-root paths in plain text (`docs/foo/bar.md`, `scripts/x.py`): replaced when the old path starts at a
     path boundary (not after a letter, `/`, `.`, `-` or `_`), so `app/scripts/x` or `/root/arm-app/scripts/x`
     never match an entry for `scripts/x`. One-segment directory entries rewrite only their files' full paths.
Nothing else (code that builds paths at run time, `Path(__file__).parents[n]`) is changed: check those by hand.
"""
from __future__ import annotations

import argparse
import os
import posixpath
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TEXT = {'.md', '.py', '.js', '.sh', '.cmd', '.ps1', '.json', '.yaml', '.yml', '.html', '.css', '.kts', '.kt', '.txt', '.toml', '.cfg'}
LINK = re.compile(r'(\]\()([^)\s]+)(\))|((?:src|href)=")([^"]+)(")')


def git(*args: str) -> str:
    return subprocess.run(['git', '-C', str(ROOT), '-c', 'core.quotepath=off', *args],
                          check=True, capture_output=True, text=True).stdout


def load_moves(path: Path) -> list[tuple[str, str]]:
    moves = []
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].rstrip() if not line.lstrip().startswith('#') else ''
        if not line.strip():
            continue
        old, new = (p.strip().rstrip('/') for p in line.split('\t'))
        moves.append((old, new))
    return moves


def expand(moves: list[tuple[str, str]], tracked: list[str]) -> dict[str, str]:
    """Every tracked file -> its new path (most specific entry wins)."""
    ordered = sorted(moves, key=lambda m: -len(m[0]))
    out = {}
    for f in tracked:
        for old, new in ordered:
            if f == old:
                out[f] = new
                break
            if f.startswith(old + '/'):
                out[f] = new + f[len(old):]
                break
    return out


def dir_map(moves: list[tuple[str, str]], fmap: dict[str, str], tracked: list[str]) -> dict[str, str]:
    """Directories that move as a whole (every tracked file under them moves by the same prefix)."""
    dirs = {}
    for old, new in moves:
        under = [f for f in tracked if f.startswith(old + '/')]
        if under and all(fmap.get(f) == new + f[len(old):] for f in under):
            dirs[old] = new
    return dirs


def new_path(p: str, fmap: dict[str, str], dmap: dict[str, str]) -> str | None:
    if p in fmap:
        return fmap[p]
    for old in sorted(dmap, key=len, reverse=True):
        if p == old or p.startswith(old + '/'):
            return dmap[old] + p[len(old):]
    return None


def rewrite_links(text: str, src_old: str, src_new: str, fmap, dmap, exists) -> str:
    base_old, base_new = posixpath.dirname(src_old), posixpath.dirname(src_new)

    def fix(target: str) -> str:
        if re.match(r'^[a-z][a-z0-9+.-]*:', target, re.I) or target.startswith(('#', '/', '<')):
            return target
        m = re.match(r'^([^#?]*)(.*)$', target)
        path, tail = m.group(1), m.group(2)
        line = ''
        lm = re.match(r'^(.*?)(:\d+(?:-\d+)?)$', path)
        if lm and not exists(posixpath.normpath(posixpath.join(base_old, path))):
            path, line = lm.group(1), lm.group(2)
        if not path:
            return target
        trail = '/' if path.endswith('/') else ''
        resolved = posixpath.normpath(posixpath.join(base_old, path))
        if resolved.startswith('..') or not exists(resolved):
            return target
        dest = new_path(resolved, fmap, dmap) or resolved
        if dest == resolved and base_old == base_new:
            return target
        rel = posixpath.relpath(dest, base_new or '.')
        return rel + trail + line + tail

    def sub(m: re.Match) -> str:
        if m.group(1):
            return m.group(1) + fix(m.group(2)) + m.group(3)
        return m.group(4) + fix(m.group(5)) + m.group(6)

    return LINK.sub(sub, text)


def rewrite_plain(text: str, pairs: list[tuple[str, str]]) -> str:
    # Mask link targets so a relative link is never treated as a root path a second time.
    masked: list[str] = []

    def mask(m: re.Match) -> str:
        masked.append(m.group(0))
        return f'\0{len(masked) - 1}\0'

    body = LINK.sub(mask, text)
    for old, new in pairs:
        after = r'(?![A-Za-z0-9_\-]|\.[A-Za-z0-9])'
        body = re.sub(r'(?<![A-Za-z0-9_/.\-\0])' + re.escape(old) + after, new, body)
    return re.sub(r'\0(\d+)\0', lambda m: masked[int(m.group(1))], body)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('moves', type=Path)
    ap.add_argument('--apply', action='store_true')
    args = ap.parse_args()

    tracked = git('ls-files').splitlines()
    tracked_set = set(tracked)
    tracked_dirs = {posixpath.dirname(f) for f in tracked}
    for d in list(tracked_dirs):
        while d:
            d = posixpath.dirname(d)
            tracked_dirs.add(d)
    # Untracked local material (資料/, data/) counts too, so links into it are re-based when their file moves.
    exists = lambda p: p in tracked_set or p in tracked_dirs or (ROOT / p).exists()  # noqa: E731

    moves = load_moves(args.moves)
    fmap = expand(moves, tracked)
    dmap = dir_map(moves, fmap, tracked)
    clash = [n for n in set(fmap.values()) if n in tracked_set and n not in fmap]
    if clash:
        print('target already exists:', *clash, sep='\n  ')
        return 1
    # Plain-text pairs: files, then whole directories; longest first so a file entry beats its directory.
    # A one-segment directory (`scripts`, `tests`) is left out: `scripts/x` inside app/ means app/scripts/x on the
    # board, so only the moved files' own full paths are rewritten.
    pairs = sorted({**fmap, **{o: n for o, n in dmap.items() if '/' in o}}.items(), key=lambda p: -len(p[0]))

    changed = []
    for f in tracked:
        if Path(f).suffix.lower() not in TEXT:
            continue
        p = ROOT / f
        if p.is_symlink() or not p.is_file():
            continue
        try:
            text = p.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            continue
        dest = fmap.get(f, f)
        out = text
        if Path(f).suffix.lower() in {'.md', '.html'}:
            out = rewrite_links(out, f, dest, fmap, dmap, exists)
        out = rewrite_plain(out, pairs)
        if out != text:
            changed.append(f)
            if args.apply:
                p.write_text(out, encoding='utf-8')

    print(f'{len(fmap)} files move, {len(changed)} files rewritten')
    for f in changed:
        print('  edit', f)
    if not args.apply:
        for o, n in sorted(fmap.items()):
            print(f'  mv {o} -> {n}')
        return 0
    for o, n in fmap.items():
        (ROOT / n).parent.mkdir(parents=True, exist_ok=True)
        git('mv', o, n)
    # Remove directories the moves left empty (untracked leftovers such as .DS_Store keep them).
    for o in sorted({posixpath.dirname(o) for o in fmap}, key=len, reverse=True):
        d = ROOT / o
        while d != ROOT and d.is_dir() and not any(x for x in d.iterdir() if x.name != '.DS_Store'):
            for x in d.iterdir():
                x.unlink()
            d.rmdir()
            d = d.parent
    return 0


if __name__ == '__main__':
    sys.exit(main())
