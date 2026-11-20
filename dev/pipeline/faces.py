"""Which block is this? Category evidence from every face of it the scan saw, without a trained detector.

Every face of a block belongs to its category (dev/vision/face_templates.py). For each block and view,
the faces turned towards the camera -- the top and up to two sides -- are cut out with the block's pose
from the scene and the view's camera pose, and squared up (homography, 128 px). A face that is mostly
saturated is the colour face (hue -> category). Otherwise it is a sticker, compared with the 16 sticker
templates: grey-level correlation at the four quarter turns plus the hue histogram of the drawing, and
the thin rim of block colour around the sticker adds a small vote. Each face gives category
probabilities; the views are fused with weights that grow with the face's size in pixels.

    python3 dev/pipeline/faces.py <scan_dir> [--sheet]   # prints per-block evidence (uses scene.json)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "dev" / sub))
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import scene as S  # noqa: E402
from block_detector import hue_label  # noqa: E402

CATEGORIES = ("recyclable", "kitchen", "hazardous", "other")
COLOUR_CATEGORY = {"blue": "recyclable", "green": "kitchen", "red": "hazardous", "yellow": "other"}
# data/ (Release assets-v4) wins, else the copy in the repository (2026-11-20)
TEMPLATES = next((d for d in (ROOT / "data" / "reference" / "block_faces", ROOT / "dev" / "sim" / "assets" / "block_faces")
                  if (d / "index.json").exists()), ROOT / "data" / "reference" / "block_faces")
SIZE = 128
MIN_FACE_PX = 45          # a face smaller than this in the image is not read
MIN_SQUARENESS = 0.45     # shortest / longest projected edge: flatter views are too foreshortened
TEMPERATURE = 0.06        # softmax over template scores
MIN_INSIDE = 0.85         # share of a face that must be inside the image (a block at the border still counts)


# ----------------------------------------------------------------------------------------- templates

def white_balance(face: np.ndarray) -> np.ndarray:
    """Scale each channel so the sticker card (the brightest tenth) reads white."""
    f = face.astype(np.float32)
    ref = np.percentile(f.reshape(-1, 3), 90, axis=0)
    return np.clip(f * (235.0 / np.maximum(ref, 20)), 0, 255).astype(np.uint8)


def inner(face: np.ndarray, margin: float = 0.1) -> np.ndarray:
    m = int(face.shape[0] * margin)
    return face[m:-m, m:-m]


def grey_feature(face: np.ndarray) -> np.ndarray:
    g = cv2.cvtColor(cv2.resize(inner(face), (64, 64), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(g, (3, 3), 0).astype(np.float32)


def hue_hist(face: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(inner(face), cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] > 60) & (hsv[..., 2] > 50)).astype(np.uint8)
    h = cv2.calcHist([hsv], [0], mask, [30], [0, 180]).ravel()
    return h / max(h.sum(), 1.0)


class Templates:
    def __init__(self, folder: Path = TEMPLATES):
        index = json.loads((folder / "index.json").read_text())
        self.items = []
        for e in index:
            if e["colour_face"]:
                continue
            face = white_balance(cv2.imread(str(folder / e["file"])))
            grey = grey_feature(face)
            self.items.append(dict(item=e["item"], category=e["category"], hist=hue_hist(face),
                                   greys=[np.rot90(grey, k).copy() for k in range(4)]))

    def scores(self, face: np.ndarray) -> list[tuple[float, dict]]:
        face = white_balance(face)
        g = grey_feature(face)
        h = hue_hist(face)
        out = []
        for t in self.items:
            ncc = max(float(cv2.matchTemplate(g, tg, cv2.TM_CCOEFF_NORMED)[0, 0]) for tg in t["greys"])
            hist = 1.0 - cv2.compareHist(h.astype(np.float32), t["hist"].astype(np.float32), cv2.HISTCMP_BHATTACHARYYA)
            out.append((ncc + 0.5 * hist, t))
        return sorted(out, key=lambda s: -s[0])


# ----------------------------------------------------------------------------------------- one face

def colour_face(face: np.ndarray) -> tuple[str | None, float]:
    """(colour, share of saturated pixels) of a face; a colour face is > 0.5 saturated."""
    hsv = cv2.cvtColor(inner(face, 0.15), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    sat = hsv[(hsv[:, 1] > 90) & (hsv[:, 2] > 35)]
    share = len(sat) / max(1, len(hsv))
    if share < 0.5:
        return None, share
    hues = sat[:, 0].astype(float)
    if np.mean((hues < 12) | (hues > 165)) > 0.5:
        hues = np.where(hues < 90, hues + 180, hues)
    label = hue_label(float(np.median(hues)) % 180)
    return (label if label in COLOUR_CATEGORY else None), share


def rim_vote(face: np.ndarray) -> dict[str, float]:
    """Category shares among the saturated pixels of the thin block-colour rim around a sticker."""
    n = face.shape[0]
    ring = np.zeros(face.shape[:2], np.uint8)
    cv2.rectangle(ring, (1, 1), (n - 2, n - 2), 1, max(2, n // 32))
    hsv = cv2.cvtColor(face, cv2.COLOR_BGR2HSV)
    px = hsv[(ring > 0) & (hsv[..., 1] > 90) & (hsv[..., 2] > 50)]
    votes = {c: 0.0 for c in CATEGORIES}
    for p in px:
        lab = hue_label(float(p[0]))
        if lab in COLOUR_CATEGORY:
            votes[COLOUR_CATEGORY[lab]] += 1
    total = sum(votes.values())
    return {c: v / total for c, v in votes.items()} if total >= 10 else {}


def face_probs(face: np.ndarray, templates: Templates) -> dict:
    colour, share = colour_face(face)
    if colour:
        p = {c: 0.02 for c in CATEGORIES}
        p[COLOUR_CATEGORY[colour]] = 0.94
        return dict(kind="colour", colour=colour, share=round(share, 2), probs=p)
    ranked = templates.scores(face)
    s = np.array([r[0] for r in ranked])
    w = np.exp((s - s.max()) / TEMPERATURE)
    w /= w.sum()
    probs = {c: 1e-3 for c in CATEGORIES}
    for wi, (_, t) in zip(w, ranked):
        probs[t["category"]] += float(wi)
    rim = rim_vote(face)
    for c, v in rim.items():            # the rim is 1 mm wide: a gentle nudge only
        probs[c] *= 1 + 0.5 * v
    z = sum(probs.values())
    return dict(kind="sticker", item=ranked[0][1]["item"], score=round(float(s[0]), 3),
                margin=round(float(s[0] - s[1]), 3), rim=rim, probs={c: v / z for c, v in probs.items()})


# ----------------------------------------------------------------------------------------- geometry

def faces_of(cube: dict) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """(name, 4 corners in map metres in order around the face, outward normal) for top and sides."""
    x, y = cube["map_cm"][0] / 100, cube["map_cm"][1] / 100
    t = math.radians(cube["yaw_deg"])
    h = S.BLOCK / 2
    ax, ay = np.array([math.cos(t), math.sin(t), 0.0]), np.array([-math.sin(t), math.cos(t), 0.0])
    c = np.array([x, y, h])
    up = np.array([0, 0, 1.0])
    out = [("top", np.array([c + h * up + h * (sx * ax + sy * ay)
                             for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]), up)]
    for k, n in enumerate((ax, ay, -ax, -ay)):
        side = np.cross(up, n)
        out.append((f"side{k}", np.array([c + h * n + h * (s1 * side + s2 * up)
                                          for s1, s2 in ((-1, 1), (1, 1), (1, -1), (-1, -1))]), n))
    return out


def observe(scan: S.Scan, cubes: list[dict], templates: Templates) -> list[list[dict]]:
    """Per cube, the readable faces over all views with their category probabilities."""
    obs = [[] for _ in cubes]
    for v, (img, pose) in enumerate(zip(scan.images, scan.poses)):
        cam_centre = -pose.R.T @ pose.t
        h_img, w_img = img.shape[:2]
        # nearer blocks hide farther ones: each block's projected outline, by distance from the camera
        outlines = []
        for c in cubes:
            pts = np.vstack([f[1] for f in faces_of(c)])
            uv = pose.project(pts, scan.k, scan.dist).astype(np.float32)
            outlines.append((float(np.linalg.norm(np.array([c["map_cm"][0] / 100, c["map_cm"][1] / 100, S.BLOCK / 2]) - cam_centre)),
                             cv2.convexHull(uv)))
        for i, c in enumerate(cubes):
            for name, corners, normal in faces_of(c):
                centre = corners.mean(axis=0)
                if np.dot(normal, cam_centre - centre) <= 0.01 * np.linalg.norm(cam_centre - centre):
                    continue                                    # turned away (or edge-on)
                uv = pose.project(corners, scan.k, scan.dist).astype(np.float32)
                frame = np.float32([[0, 0], [w_img - 1, 0], [w_img - 1, h_img - 1], [0, h_img - 1]])
                inside_area, _ = cv2.intersectConvexConvex(cv2.convexHull(uv), frame)
                if inside_area < MIN_INSIDE * abs(cv2.contourArea(uv)):
                    continue                                    # mostly cut off by the image border
                edges = [np.linalg.norm(uv[j] - uv[(j + 1) % 4]) for j in range(4)]
                size = math.sqrt(abs(cv2.contourArea(uv)))
                if size < MIN_FACE_PX or min(edges) < MIN_SQUARENESS * max(edges):
                    continue
                my_dist = outlines[i][0]
                hidden = False
                for j, (d, hull) in enumerate(outlines):
                    if j != i and d < my_dist:
                        inside = sum(cv2.pointPolygonTest(hull, (float(p[0]), float(p[1])), False) >= 0 for p in uv)
                        inside += cv2.pointPolygonTest(hull, tuple(map(float, uv.mean(axis=0))), False) >= 0
                        if inside >= 1:
                            hidden = True
                if hidden:
                    continue
                hmat = cv2.getPerspectiveTransform(uv, np.float32([[0, 0], [SIZE - 1, 0], [SIZE - 1, SIZE - 1], [0, SIZE - 1]]))
                face = cv2.warpPerspective(img, hmat, (SIZE, SIZE))
                res = face_probs(face, templates)
                # sides are seen obliquely and often in the block's own shadow: half weight
                weight = min(1.0, size / 100) * (1.0 if name == "top" else 0.5)
                obs[i].append(dict(view=v, face=name, size_px=round(size), weight=weight, image=face, **res))
    return obs


def fuse(observations: list[dict]) -> dict:
    """Weighted log-probability sum over faces; confidence = top probability after fusion."""
    if not observations:
        return dict(category=None, confidence=0.0, probs={}, faces=0)
    logp = {c: 0.0 for c in CATEGORIES}
    for o in observations:
        for c in CATEGORIES:
            logp[c] += o["weight"] * math.log(max(o["probs"][c], 1e-4))
    m = max(logp.values())
    p = {c: math.exp(v - m) for c, v in logp.items()}
    z = sum(p.values())
    p = {c: v / z for c, v in p.items()}
    best = max(p, key=p.get)
    return dict(category=best, confidence=round(p[best], 3), probs={c: round(v, 3) for c, v in p.items()},
                faces=len(observations))


def evaluate(dataset: Path, templates: Templates) -> None:
    """Fused category against the tracked truth for every labelled round of a collect.py dataset."""
    confusion = {t: {p: 0 for p in CATEGORIES + (None,)} for t in CATEGORIES}
    for line in (dataset / "track.jsonl").read_text().splitlines():
        rec = json.loads(line)
        if not rec.get("labelled"):
            continue
        scan = S.Scan(dataset / rec["round"])
        scan.register()
        scan.camera_poses()
        cubes = rec["labelled"]
        for c, o in zip(cubes, observe(scan, cubes, templates)):
            f = fuse(o)
            confusion[c["category"]][f["category"]] += 1
            top = [x.get("item") or x.get("colour") for x in o if x["face"] == "top"]
            mark = "ok " if f["category"] == c["category"] else "BAD"
            print(f"{rec['round']} {mark} truth {c['category']:10s} -> {f['category'] or '-':10s} {f['confidence']:.2f} "
                  f"({f['faces']} faces; tops {top})")
    print("confusion (truth -> predicted):")
    for t, row in confusion.items():
        print(f"  {t:10s}", {k or "none": v for k, v in row.items() if v})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scan", type=Path, help="a scan directory, or a collect.py dataset with --eval")
    parser.add_argument("--eval", action="store_true")
    parser.add_argument("--sheet", action="store_true", help="write faces_<i>.jpg with the cut-out faces")
    parser.add_argument("--sides-only", action="store_true", help="ignore top faces (a test on colour-up scans)")
    args = parser.parse_args()
    if args.eval:
        return evaluate(args.scan, Templates())
    scene = json.loads((args.scan / "scene.json").read_text())
    scan = S.Scan(args.scan)
    scan.register()
    scan.camera_poses()
    obs = observe(scan, scene["cubes"], Templates())
    for c, o in zip(scene["cubes"], obs):
        use = [x for x in o if not (args.sides_only and x["face"] == "top")]
        f = fuse(use)
        print(f"block {c['map_cm']} top {c.get('colour') or '-':6s} -> {f['category']} {f['confidence']:.2f} "
              f"from {f['faces']} faces; " + ", ".join(f"v{x['view']}{x['face'][0]}:{x.get('item') or x.get('colour')}" for x in use))
        if args.sheet and use:
            tiles = [cv2.putText(x["image"].copy(), f"v{x['view']} {x['face']}", (2, 12), 0, 0.4, (0, 0, 255), 1) for x in use]
            rows = [np.hstack(tiles[k:k + 8] + [np.zeros_like(tiles[0])] * (8 - len(tiles[k:k + 8]))) for k in range(0, len(tiles), 8)]
            cv2.imwrite(str(args.scan / f"faces_{c['map_cm'][0]}_{c['map_cm'][1]}.jpg"), np.vstack(rows))


if __name__ == "__main__":
    main()
