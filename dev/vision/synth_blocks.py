"""Synthetic training frames: the blocks' own faces rendered into real scan views, with labels for free.

The real dataset (dev/pipeline/collect.py) is expensive -- the operator places four blocks per round -- and
it covers the map unevenly: the near strip and the left/right edges have few blocks, and only the faces
that happened to be up. Here the 20 photographed faces (dev/vision/face_templates.py: 4 colour faces and
16 stickers, 5 per block, the 6th face painted the block's colour) are rendered as 30 mm cubes into
views that already exist, using that view's calibrated camera pose, so a cube can be put anywhere on
the map at any yaw with any face up, and its box is known exactly.

What is modelled, because it is what the real frames show:
  * perspective: each visible face is a homography of its template onto the projected quad
  * lighting: one light direction per frame, Lambert shading per face, the whole cube scaled to the
    brightness of the paper around it (the board camera's exposure changes with the arm's pose)
  * contact shadow: the base square, sheared away from the light, darkened and blurred
  * the camera: slight blur and sensor noise, matched to the background's own noise

Real blocks in the background view keep their real labels, so a frame can hold both.

    python3 dev/vision/synth_blocks.py data/datasets/blocks4/<ds> --out data/datasets/blocks4/<ds>_synth -n 400
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "dev" / sub))
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import scene as S  # noqa: E402

CLASSES = ["recyclable", "kitchen", "hazardous", "other"]
FACES_DIR = ROOT / "data" / "reference" / "block_faces"
BLOCK = 0.03
IMAGE_W, IMAGE_H = 640, 480
MIN_INSIDE = 0.1          # share of the box inside the frame for it to be labelled (as collect.py)
MIN_BOX_PX = 8
APART_CM = 4.2            # centres of two blocks cannot be nearer than this
OCCLUDE_PX = 60           # a synthetic cube nearer than a real one and this close on screen would hide it
# where blocks may be put: the map, a little beyond the grasp band, so edges and the near strip get covered
X_RANGE_CM = (-13.0, 13.0)
Y_RANGE_CM = (3.0, 19.5)


# ------------------------------------------------------------------------------- the blocks' faces

def load_blocks() -> dict[str, list[np.ndarray]]:
    """category -> its 6 face textures (colour face, 4 stickers, and a plain face in the block's colour)."""
    index = json.loads((FACES_DIR / "index.json").read_text())
    out: dict[str, list[np.ndarray]] = {c: [] for c in CLASSES}
    plain: dict[str, np.ndarray] = {}
    for e in index:
        img = cv2.imread(str(FACES_DIR / e["file"]))
        out[e["category"]].append(img)
        if e["colour_face"]:
            # the 6th face: the block's own colour, from the middle of its colour face
            n = img.shape[0]
            patch = img[n // 3:2 * n // 3, n // 3:2 * n // 3]
            plain[e["category"]] = cv2.resize(cv2.blur(patch, (9, 9)), (n, n))
    for c, p in plain.items():
        out[c].append(p)
    return out


# ------------------------------------------------------------------------------- geometry

def cube_faces(x: float, y: float, yaw_deg: float) -> list[tuple[np.ndarray, np.ndarray]]:
    """[(4 corners in map metres, outward normal)] for the top face then the four sides, corners ordered
    so that the template's top-left maps to the first corner."""
    t = math.radians(yaw_deg)
    h = BLOCK / 2
    ax = np.array([math.cos(t), math.sin(t), 0.0])
    ay = np.array([-math.sin(t), math.cos(t), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    c = np.array([x, y, h])
    faces = [(np.array([c + h * up + h * (sx * ax + sy * ay)
                        for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]), up)]
    for n in (ax, ay, -ax, -ay):
        side = np.cross(up, n)
        faces.append((np.array([c + h * n + h * (s1 * side + s2 * up)
                                for s1, s2 in ((-1, 1), (1, 1), (1, -1), (-1, -1))]), n))
    return faces


def corners(x: float, y: float, yaw_deg: float) -> np.ndarray:
    t = math.radians(yaw_deg)
    h = BLOCK / 2
    ax = np.array([math.cos(t), math.sin(t), 0.0])
    ay = np.array([-math.sin(t), math.cos(t), 0.0])
    c = np.array([x, y, 0.0])
    return np.array([c + sx * h * ax + sy * h * ay + np.array([0, 0, z])
                     for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)) for z in (0.0, BLOCK)])


# ------------------------------------------------------------------------------- rendering

def local_brightness(img: np.ndarray, uv: np.ndarray) -> float:
    """Median grey of the paper around a projected footprint, as a fraction of the templates' paper white."""
    x0, y0 = np.clip(uv.min(axis=0) - 25, [0, 0], [IMAGE_W - 1, IMAGE_H - 1]).astype(int)
    x1, y1 = np.clip(uv.max(axis=0) + 25, [1, 1], [IMAGE_W, IMAGE_H]).astype(int)
    patch = img[y0:y1, x0:x1]
    if patch.size == 0:
        return 1.0
    grey = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    return float(np.percentile(grey, 80)) / 200.0


def render_cube(img: np.ndarray, pose, k, dist, x: float, y: float, yaw: float,
                textures: list[np.ndarray], light: np.ndarray, rng: random.Random) -> np.ndarray | None:
    """Draw one cube into `img` (in place on a copy) and return its box, or None if it is not usable."""
    cam_centre = -pose.R.T @ pose.t
    quads = []
    for i, (face, normal) in enumerate(cube_faces(x, y, yaw)):
        centre = face.mean(axis=0)
        towards = cam_centre - centre
        if np.dot(normal, towards) <= 0.02 * np.linalg.norm(towards):
            continue
        uv = pose.project(face, k, dist).astype(np.float32)
        if cv2.contourArea(uv) < 25:
            continue
        quads.append((float(np.linalg.norm(centre - cam_centre)), i, uv, normal))
    if not quads:
        return None
    all_uv = pose.project(corners(x, y, yaw), k, dist).astype(np.float32)
    if not (-200 < all_uv[:, 0].mean() < IMAGE_W + 200 and -200 < all_uv[:, 1].mean() < IMAGE_H + 200):
        return None
    scale = np.clip(local_brightness(img, all_uv) * rng.uniform(0.92, 1.08), 0.35, 1.35)

    # contact shadow: the cube's body cast on the table along the light (offset = height * light_xy / light_z)
    base = np.array([p for p in corners(x, y, yaw)[0::2]])
    offset = np.array([-light[0], -light[1], 0.0]) / max(light[2], 0.3) * BLOCK * rng.uniform(0.8, 1.3)
    shadow_uv = pose.project(np.vstack([base, base + offset]), k, dist).astype(np.float32)
    layer = img.astype(np.float32)
    mask = np.zeros(img.shape[:2], np.float32)
    cv2.fillConvexPoly(mask, cv2.convexHull(shadow_uv).astype(np.int32), 1.0)
    mask = cv2.GaussianBlur(mask, (0, 0), rng.uniform(2.5, 5.0)) * rng.uniform(0.25, 0.5)
    layer *= (1 - mask)[..., None]

    order = sorted(quads, key=lambda q: -q[0])          # far faces first
    for _, i, uv, normal in order:
        lam = 0.55 + 0.45 * max(0.0, float(np.dot(normal, light)))
        tex = textures[i]
        if i == 0:                                      # the top face may be read at any of four turns
            tex = np.rot90(tex, rng.randint(0, 3)).copy()
        n = tex.shape[0]
        # the paint is not even and the camera vignettes: a mild gradient across the face
        gx, gy = np.meshgrid(np.linspace(-1, 1, n), np.linspace(-1, 1, n))
        grad = 1 + rng.uniform(-0.09, 0.09) * gx + rng.uniform(-0.09, 0.09) * gy
        src = np.float32([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]])
        h_mat = cv2.getPerspectiveTransform(src, uv)
        warped = cv2.warpPerspective(tex.astype(np.float32) * (lam * scale) * grad[..., None], h_mat, (IMAGE_W, IMAGE_H))
        face_mask = np.zeros(img.shape[:2], np.float32)
        cv2.fillConvexPoly(face_mask, uv.astype(np.int32), 1.0)
        # the blocks' edges are rounded and the camera is soft: pull the mask in and blur it
        face_mask = cv2.erode(face_mask, np.ones((3, 3), np.uint8))
        face_mask = cv2.GaussianBlur(face_mask, (0, 0), 1.1)
        layer = layer * (1 - face_mask)[..., None] + warped * face_mask[..., None]
    img[:] = np.clip(layer, 0, 255).astype(np.uint8)

    x0, y0 = all_uv.min(axis=0)
    x1, y1 = all_uv.max(axis=0)
    cx0, cy0 = max(x0, 0), max(y0, 0)
    cx1, cy1 = min(x1, IMAGE_W - 1), min(y1, IMAGE_H - 1)
    if cx1 - cx0 < MIN_BOX_PX or cy1 - cy0 < MIN_BOX_PX:
        return None
    if (cx1 - cx0) * (cy1 - cy0) < MIN_INSIDE * (x1 - x0) * (y1 - y0):
        return None
    return np.array([cx0, cy0, cx1, cy1])


def camera_noise(img: np.ndarray, rng: random.Random) -> np.ndarray:
    out = img.astype(np.float32)
    if rng.random() < 0.7:
        out = cv2.GaussianBlur(out, (0, 0), rng.uniform(0.3, 1.1))
    out += np.random.normal(0, rng.uniform(1.0, 4.0), out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------------------- one frame

def yolo_line(cls: int, box: np.ndarray) -> str:
    x0, y0, x1, y1 = box
    return (f"{cls} {(x0 + x1) / 2 / IMAGE_W:.6f} {(y0 + y1) / 2 / IMAGE_H:.6f} "
            f"{(x1 - x0) / IMAGE_W:.6f} {(y1 - y0) / IMAGE_H:.6f}\n")


def visible_spots(pose, k, dist, margin: int = -60) -> list[tuple[float, float]]:
    """Map points (cm) whose block centre projects into this view. A view looking off the map sees few or
    none, and rendering there would waste the frame; `margin` < 0 keeps partly visible blocks too."""
    xs = np.arange(X_RANGE_CM[0], X_RANGE_CM[1] + 0.01, 1.0)
    ys = np.arange(Y_RANGE_CM[0], Y_RANGE_CM[1] + 0.01, 1.0)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel() / 100, gy.ravel() / 100, np.full(gx.size, BLOCK / 2)])
    front = (pts @ pose.R.T + pose.t)[:, 2] > 0.05
    uv = pose.project(pts, k, dist)
    ok = front & (uv[:, 0] > margin) & (uv[:, 0] < IMAGE_W - margin) & \
        (uv[:, 1] > margin) & (uv[:, 1] < IMAGE_H - margin)
    return [(float(a), float(b)) for a, b, good in zip(gx.ravel(), gy.ravel(), ok) if good]


def free_spot(spots: list[tuple[float, float]], taken: list[tuple[float, float]],
              rng: random.Random) -> tuple[float, float] | None:
    for _ in range(60):
        x, y = rng.choice(spots)
        x, y = x + rng.uniform(-0.5, 0.5), y + rng.uniform(-0.5, 0.5)
        if all(math.dist((x, y), t) >= APART_CM for t in taken):
            return x, y
    return None


def make_frame(img: np.ndarray, pose, k, dist, real: list[tuple[float, float]], blocks: dict,
               rng: random.Random, n_cubes: int) -> tuple[np.ndarray, list[str]]:
    out = img.copy()
    theta = rng.uniform(0, 2 * math.pi)
    light = np.array([math.cos(theta) * 0.55, math.sin(theta) * 0.55, rng.uniform(0.55, 0.95)])
    light /= np.linalg.norm(light)
    spots = visible_spots(pose, k, dist)
    if not spots:
        return camera_noise(out, rng), []
    cam_centre = -pose.R.T @ pose.t

    def screen(cm):
        p3 = np.array([cm[0] / 100, cm[1] / 100, BLOCK / 2])
        return pose.project(p3[None], k, dist)[0], float(np.linalg.norm(p3 - cam_centre))

    # a real block in this view keeps its real label, so nothing synthetic may come in front of it
    real_screen = [screen(c) for c in real]

    def hides_a_real_block(spot):
        uv, depth = screen(spot)
        return any(depth < d - 1e-6 and math.dist(uv, ruv) < OCCLUDE_PX for ruv, d in real_screen)

    taken = list(real)
    placed = []
    for _ in range(n_cubes):
        for _try in range(8):
            spot = free_spot(spots, taken, rng)
            if spot is None or not hides_a_real_block(spot):
                break
        else:
            spot = None
        if spot is None:
            break
        taken.append(spot)
        placed.append((spot, rng.choice(CLASSES), rng.uniform(-45, 45)))
    placed.sort(key=lambda p: -np.linalg.norm(np.array([p[0][0] / 100, p[0][1] / 100, BLOCK / 2]) - cam_centre))
    lines = []
    for (x, y), category, yaw in placed:                # far cubes first, so near ones occlude them
        textures = blocks[category][:]
        rng.shuffle(textures)
        box = render_cube(out, pose, k, dist, x / 100, y / 100, yaw, textures, light, rng)
        if box is not None:
            lines.append(yolo_line(CLASSES.index(category), box))
    return camera_noise(out, rng), lines


# ------------------------------------------------------------------------------- dataset

def backgrounds(dataset: Path, exclude: set[str] = frozenset()) -> list[dict]:
    """Every labelled view of the real dataset with its camera pose and the real blocks in it.
    `exclude` keeps the validation rounds out: a synthetic frame drawn on a validation view would put
    that view (and its real blocks) into training."""
    out = []
    for line in (dataset / "track.jsonl").read_text().splitlines():
        rec = json.loads(line)
        if not rec.get("labelled") or rec["round"] in exclude:
            continue
        run = dataset / rec["round"]
        if not (run / "views.json").exists():
            continue
        scan = S.Scan(run)
        try:
            scan.register()
        except RuntimeError:
            continue
        scan.camera_poses()
        for i, v in enumerate(scan.views):
            stem = f"{rec['round']}_{Path(v['image']).stem}"
            lab = dataset / "labels" / f"{stem}.txt"
            out.append(dict(image=str(run / v["image"]), stem=stem, pose=scan.poses[i], k=scan.k, dist=scan.dist,
                            real=[(c["map_cm"][0], c["map_cm"][1]) for c in rec["labelled"]],
                            lines=lab.read_text().splitlines(True) if lab.exists() else []))
    return out


def build(dataset: Path, out: Path, count: int, seed: int, max_cubes: int, keep_real: bool,
          exclude: set[str] = frozenset()) -> None:
    rng = random.Random(seed)
    np.random.seed(seed)
    blocks = load_blocks()
    bgs = backgrounds(dataset, exclude)
    if not bgs:
        raise SystemExit("no usable background views in " + str(dataset))
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)
    print(f"{len(bgs)} background views, rendering {count} frames")
    boxes = 0
    for i in range(count):
        bg = bgs[i % len(bgs)] if i < len(bgs) else rng.choice(bgs)
        img = cv2.imread(bg["image"])
        frame, lines = make_frame(img, bg["pose"], bg["k"], bg["dist"], bg["real"], blocks, rng,
                                  rng.randint(1, max_cubes))
        if keep_real:
            lines = list(bg["lines"]) + lines
        name = f"synth_{i:05d}_{bg['stem']}"
        cv2.imwrite(str(out / "images" / f"{name}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
        (out / "labels" / f"{name}.txt").write_text("".join(lines))
        boxes += len(lines)
    (out / "dataset.yaml").write_text(f"path: {out.resolve()}\ntrain: images\nval: images\n"
                                      f"nc: {len(CLASSES)}\nnames: {json.dumps(CLASSES)}\n")
    (out / "source.json").write_text(json.dumps(dict(dataset=str(dataset), count=count, seed=seed,
                                                     max_cubes=max_cubes, keep_real=keep_real,
                                                     excluded_rounds=sorted(exclude)), indent=1))
    print(f"wrote {count} frames, {boxes} boxes -> {out}")


def preview(dataset: Path, out: Path, n: int = 8) -> Path:
    tiles = []
    for img_path in sorted((out / "images").glob("*.jpg"))[:n]:
        img = cv2.imread(str(img_path))
        for line in (out / "labels" / f"{img_path.stem}.txt").read_text().splitlines():
            c, x, y, w, h = line.split()
            x, y, w, h = float(x) * IMAGE_W, float(y) * IMAGE_H, float(w) * IMAGE_W, float(h) * IMAGE_H
            cv2.rectangle(img, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 255), 2)
            cv2.putText(img, CLASSES[int(c)], (int(x - w / 2), max(12, int(y - h / 2) - 4)), 0, 0.45, (0, 255, 255), 1)
        tiles.append(cv2.resize(img, (320, 240)))
    sheet = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles) - 3, 4)])
    path = out / "preview.jpg"
    cv2.imwrite(str(path), sheet)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", type=Path, help="the real dataset the backgrounds come from")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("-n", "--count", type=int, default=400)
    parser.add_argument("--max-cubes", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-real", action="store_true", help="drop the real blocks' labels (they stay in view)")
    parser.add_argument("--exclude", default="", help="comma-separated rounds to keep out (the val rounds)")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    build(args.dataset.resolve(), args.out.resolve(), args.count, args.seed, args.max_cubes, not args.no_real,
          {r for r in args.exclude.split(",") if r})
    if args.preview:
        print("preview:", preview(args.dataset.resolve(), args.out.resolve()))


if __name__ == "__main__":
    main()
