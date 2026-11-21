"""How much the views disagree about where a block is -- the jitter of the stitched top view -- on the real scans.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/ortho_consistency.py [runs ...]

For each scan and each way of placing the views (scene.POSES "pnp" -- before 2026-11-20 -- and "bundle"): the
scene's blocks are found as usual, then every view that sees a block whole is resampled alone onto the plane of
the block tops and the block's square is fitted there (cubes.refine_at, 12 px window around the scene's
centre). The spread of those per-view centres around their median is the stitching error for that block; no
truth is needed. Views placed by forward kinematics alone are reported apart (they are the ones down-weighted).
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "pipeline", "vision", "tasks", "tools"):
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import cubes as C  # noqa: E402
import scene as S  # noqa: E402


def per_view_centres(scan: S.Scan, cube_map_cm) -> list[tuple[int, float, float, bool]]:
    out = []
    images, poses, weights = scan.images, scan.poses, list(getattr(scan, "view_weight", [1.0] * len(scan.poses)))
    for i, (img, pose) in enumerate(zip(images, poses)):
        scan.images, scan.poses, scan.view_weight = [img], [pose], [1.0]
        try:
            top, w = scan.ortho(S.BLOCK)
        finally:
            scan.images, scan.poses, scan.view_weight = images, poses, weights
        col, row = S.Scan.to_px(cube_map_cm[0] / 100, cube_map_cm[1] / 100)
        r0, r1, c0, c1 = int(row) - 18, int(row) + 18, int(col) - 18, int(col) + 18
        if r0 < 0 or c0 < 0 or r1 > w.shape[0] or c1 > w.shape[1] or (w[r0:r1, c0:c1] > 0.2).mean() < 0.95:
            continue                       # this view does not see the whole block
        r = C.refine_at(top, w, col, row, window_px=12)
        x, y = S.Scan.to_map(r["col"], r["row"])
        out.append((i, x * 1000, y * 1000, not pose.markers))     # placed by forward kinematics
    return out


def main() -> int:
    runs = [Path(a) for a in sys.argv[1:]] or sorted(
        p for p in (ROOT / "data" / "runs" / "pipeline").iterdir() if (p / "views.json").exists())
    import run as R
    result = {}
    for mode in ("pnp", "bundle"):
        S.POSES = mode
        devs, devs_fk, pairs = [], [], []
        for run in runs:
            with contextlib.redirect_stdout(io.StringIO()):
                scene = R.build_scene(run)
                scan = S.Scan(run)
                scan.register()
                scan.camera_poses()
            for cube in scene["cubes"]:
                cs = per_view_centres(scan, cube["map_cm"])
                if len(cs) < 2:
                    continue
                med = np.median([[c[1], c[2]] for c in cs], axis=0)
                for _, x, y, fk in cs:
                    (devs_fk if fk else devs).append(float(math.hypot(x - med[0], y - med[1])))
                for a_, b_ in ((a_, b_) for ia, a_ in enumerate(cs) for b_ in cs[ia + 1:]):
                    pairs.append(float(math.hypot(a_[1] - b_[1], a_[2] - b_[2])))
        q = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))] if v else float("nan")   # noqa: E731
        mean = lambda v: sum(v) / len(v) if v else float("nan")   # noqa: E731
        print(f"{mode:6s}: same block, any two views: {mean(pairs):.2f} mm mean, median {q(pairs, .5):.2f}, p90 "
              f"{q(pairs, .9):.2f}, >3 mm {sum(p > 3 for p in pairs)}/{len(pairs)}")
        result[mode] = dict(pairs_n=len(pairs), pairs_mean_mm=mean(pairs), pairs_median_mm=q(pairs, .5),
                            pairs_p90_mm=q(pairs, .9), n=len(devs), median_mm=q(devs, .5), p90_mm=q(devs, .9),
                            fk_n=len(devs_fk), fk_median_mm=q(devs_fk, .5), fk_p90_mm=q(devs_fk, .9))
        print(f"{mode:6s}: per-view block centre off the views' median: views placed by codes {q(devs, .5):.2f} / "
              f"{q(devs, .9):.2f} mm (median / p90, {len(devs)}); by forward kinematics {q(devs_fk, .5):.2f} / "
              f"{q(devs_fk, .9):.2f} mm ({len(devs_fk)})", flush=True)
    out = ROOT / "data" / "reports" / "ortho_consistency.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
