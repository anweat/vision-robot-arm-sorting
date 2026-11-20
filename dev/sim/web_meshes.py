"""The course STL meshes of the arm, thinned for the simulator page (dev/sim/web): the same parts the Blender scene
(dev/scripts/blender_arm_sim.py) is built from, so the page shows the arm's real shape instead of the planner's
collision samples.

    uv run --with numpy python dev/sim/web_meshes.py [--cell-mm 2.5] [--stl-dir DIR] [--out data/sim/web_meshes.json]

Input: 资料/展开资源/三维结构文件/三维结构文件/{base_link,link1..link5}.STL (course package, not in the repository).
Output: data/sim/web_meshes.json (this machine's copy). The repository keeps one at dev/sim/assets/web_meshes.json
(2026-11-20, so a fresh clone -- e.g. on Windows without 资料/ -- draws the arm instead of its point samples);
the simulator serves data/sim's when present, else the repository's. After changing the gripper model, regenerate
and refresh the repository copy:  ... web_meshes.py --out dev/sim/assets/web_meshes.json

Each STL is in its URDF link frame (metres). link5 holds the whole gripper as separate shells; each shell is
given to the part whose collision samples (app/config/gripper_geometry.json, dev/scripts/blender_extract_gripper.py)
lie nearest to it -- finger_right/left move with the four-bar coupler, upper_/lower_ links turn about their
pivots, everything else is static -- so the page can open and close the fingers the way planning.gripper does.
Thinning is vertex clustering on a grid of `--cell-mm`.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
STL_DIR = ROOT / "资料" / "展开资源" / "三维结构文件" / "三维结构文件"
OUT = ROOT / "data" / "sim" / "web_meshes.json"
GEOMETRY = ROOT / "app" / "config" / "gripper_geometry.json"
LINKS = ("base_link", "link1", "link2", "link3", "link4", "link5")
GRIPPER_PARTS = ("finger_right", "upper_right", "lower_right", "finger_left", "upper_left", "lower_left", "static")


def read_stl(path: Path) -> np.ndarray:
    """(n, 3, 3) triangles of a binary STL, metres."""
    data = path.read_bytes()
    n = int(np.frombuffer(data, "<u4", 1, 80)[0])
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    if 84 + n * rec.itemsize != len(data):
        raise ValueError(f"{path.name}: not a binary STL")
    return np.frombuffer(data, rec, n, 84)["v"].astype(float)


def weld(tris: np.ndarray, tol: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, faces) with coincident corners merged."""
    keys = np.round(tris.reshape(-1, 3) / tol).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    return uniq * tol, inverse.reshape(-1, 3)


def shells(faces: np.ndarray, nverts: int) -> np.ndarray:
    """Connected-component label of every face (faces sharing a vertex are connected)."""
    parent = np.arange(nverts)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b, c in faces:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    roots = np.array([find(v) for v in range(nverts)])
    return roots[faces[:, 0]]


def thin(verts: np.ndarray, faces: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Vertex clustering: one vertex per occupied grid cell (their mean), degenerate faces dropped."""
    if len(faces) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    used = np.unique(faces)
    keys = np.floor(verts[used] / cell).astype(np.int64)
    _, cluster = np.unique(keys, axis=0, return_inverse=True)
    cluster = cluster.ravel()
    remap = np.full(len(verts), -1)
    remap[used] = cluster
    count = np.bincount(cluster)
    out_v = np.zeros((len(count), 3))
    for k in range(3):
        out_v[:, k] = np.bincount(cluster, weights=verts[used, k]) / count
    f = remap[faces]
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[keep]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return out_v, f[np.sort(first)]


def pack(verts: np.ndarray, faces: np.ndarray) -> dict:
    return dict(v=np.round(verts * 1e4).astype(int).ravel().tolist(), f=faces.astype(int).ravel().tolist())


def gripper_parts(verts: np.ndarray, faces: np.ndarray) -> dict[str, np.ndarray]:
    """Faces of link5 grouped by gripper part, each shell to the part with the nearest collision samples."""
    samples = json.loads(GEOMETRY.read_text(encoding="utf-8"))["points_m"]
    clouds = {name: np.array(samples[name], float) for name in GRIPPER_PARTS}
    label = shells(faces, len(verts))
    groups: dict[str, list[int]] = {name: [] for name in GRIPPER_PARTS}
    for shell in np.unique(label):
        idx = np.nonzero(label == shell)[0]
        pts = verts[np.unique(faces[idx])]
        pts = pts[:: max(1, len(pts) // 300)]
        best = min(GRIPPER_PARTS, key=lambda n: float(np.median(
            np.min(np.linalg.norm(pts[:, None, :] - clouds[n][None, :, :], axis=2), axis=1))))
        groups[best] += idx.tolist()
    return {name: faces[np.array(sorted(ix), int)] if ix else np.zeros((0, 3), int) for name, ix in groups.items()}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl-dir", type=Path, default=STL_DIR)
    ap.add_argument("--cell-mm", type=float, default=2.5)
    ap.add_argument("--gripper-cell-mm", type=float, default=1.5)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    if not a.stl_dir.is_dir():
        print(f"no course meshes at {a.stl_dir}", file=sys.stderr)
        return 1
    parts = {}
    for link in LINKS:
        verts, faces = weld(read_stl(a.stl_dir / f"{link}.STL"))
        if link != "link5":
            v, f = thin(verts, faces, a.cell_mm / 1000)
            parts[link] = dict(frame=link, **pack(v, f))
            print(f"{link}: {len(faces)} -> {len(f)} triangles")
            continue
        for name, f0 in gripper_parts(verts, faces).items():
            v, f = thin(verts, f0, (a.gripper_cell_mm if name != "static" else a.cell_mm) / 1000)
            parts["gripper_" + name] = dict(frame="gripper_" + name if name != "static" else "link5", **pack(v, f))
            print(f"link5/{name}: {len(f0)} -> {len(f)} triangles")
    out = dict(_source="course STL meshes thinned by dev/sim/web_meshes.py; vertices in 0.1 mm, faces as vertex "
                       "indices; each part is drawn with frames[part.frame] from /api/sim/body",
               cell_mm=a.cell_mm, gripper_cell_mm=a.gripper_cell_mm, parts=parts)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, separators=(",", ":")))
    print(f"wrote {a.out} ({a.out.stat().st_size // 1024} KB, "
          f"{sum(len(p['f']) // 3 for p in parts.values())} triangles)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
