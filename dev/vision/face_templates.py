"""Face templates of the four blocks, cut from the operator's phone photos in `materials/photos/blocks/` (2026-09-24).

Each photo shows the four blocks in a 2 x 2 group, one face up each, always in the same places:
top left the yellow block (other), top right blue (recyclable), bottom left green (kitchen), bottom
right red (hazardous) -- the thin rim of block colour around every sticker shows it. Two photos show the
colour faces, four show the 16 stickers. The group is rectified (its outer corners are found from the
bright faces against the grey table; for two photos with strong shadows they are given by hand), split
2 x 2, and each face trimmed to its bright square.

    python3 dev/vision/face_templates.py     # writes data/reference/block_faces/*.png and index.json
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PHOTOS = ROOT / "block"
OUT = ROOT / "data" / "reference" / "block_faces"
SIZE = 128
PLACES = ("other", "recyclable", "kitchen", "hazardous")          # tl, tr, bl, br
# photo (sorted name order) -> the face in each place; None: colour faces
FACES = [
    None,
    None,
    ("disposable_chopsticks", "zip_top_can", "fish_bone", "used_batteries"),
    ("cigarette_butts", "book", "egg_shell", "syringe"),
    ("toilet_paper", "newspaper", "watermelon_rind", "expired_tablets"),
    ("peach_pit", "old_school_bag", "apple_core", "expired_cosmetics"),
]
COLOUR = {"other": "yellow", "recyclable": "blue", "kitchen": "green", "hazardous": "red"}
# group corners (tl, tr, br, bl) read off a grid for the photos whose shadow defeats the automatic quad
MANUAL = {2: [(255, 450), (1017, 445), (1032, 1212), (242, 1207)],
          3: [(250, 400), (1095, 392), (1102, 1232), (240, 1237)]}


def _ordered(box: np.ndarray) -> np.ndarray:
    s, d = box.sum(1), np.diff(box, axis=1).ravel()
    return np.float32([box[s.argmin()], box[d.argmin()], box[s.argmax()], box[d.argmax()]])


def group_quad(im: np.ndarray) -> np.ndarray:
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    bg = np.median(g[:, :60])
    m = ((g > bg + 30) | ((hsv[..., 1] > 100) & (hsv[..., 2] > 70))).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((31, 31), np.uint8))
    c = max(cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    hull = cv2.convexHull(c)
    for eps in np.linspace(0.01, 0.1, 30):
        a = cv2.approxPolyDP(hull, eps * cv2.arcLength(hull, True), True)
        if len(a) == 4:
            return _ordered(a.reshape(4, 2).astype(np.float32))
    return _ordered(cv2.boxPoints(cv2.minAreaRect(c)))


def trim(tile: np.ndarray) -> np.ndarray:
    """The face inside a quarter of the group: the largest bright/saturated square, squared up."""
    g = cv2.cvtColor(tile, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(tile, cv2.COLOR_BGR2HSV)
    m = ((g > np.percentile(g, 30)) | (hsv[..., 1] > 100)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    c = max(cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    (w, h), side = rect[1], tile.shape[0]
    if w * h < 0.6 * side * side or max(w, h) > 1.25 * min(w, h):
        # a face with thin line art (fish bone) splits into pieces: take the quarter with a small margin
        m0 = int(side * 0.04)
        return cv2.resize(tile[m0:side - m0, m0:side - m0], (SIZE, SIZE))
    q = _ordered(cv2.boxPoints(rect))
    h = cv2.getPerspectiveTransform(q, np.float32([[0, 0], [SIZE - 1, 0], [SIZE - 1, SIZE - 1], [0, SIZE - 1]]))
    return cv2.warpPerspective(tile, h, (SIZE, SIZE))


def build() -> list[dict]:
    OUT.mkdir(parents=True, exist_ok=True)
    index = []
    for n, photo in enumerate(sorted(PHOTOS.glob("*.jpg"))):
        if n == 1:            # the second colour photo repeats the first
            continue
        im = cv2.imread(str(photo))
        q = np.float32(MANUAL[n]) if n in MANUAL else group_quad(im)
        w = 600
        group = cv2.warpPerspective(im, cv2.getPerspectiveTransform(q, np.float32([[0, 0], [w, 0], [w, w], [0, w]])), (w, w))
        for k, place in enumerate(PLACES):
            r, c = divmod(k, 2)
            tile = group[r * w // 2:(r + 1) * w // 2, c * w // 2:(c + 1) * w // 2]
            name = f"colour_{COLOUR[place]}" if FACES[n] is None else FACES[n][k]
            face = trim(tile)
            cv2.imwrite(str(OUT / f"{place}__{name}.png"), face)
            index.append(dict(file=f"{place}__{name}.png", category=place, item=name, photo=photo.name,
                              colour_face=FACES[n] is None))
    (OUT / "index.json").write_text(json.dumps(index, indent=1, ensure_ascii=False))
    return index


def sheet(index: list[dict]) -> np.ndarray:
    tiles = [cv2.imread(str(OUT / e["file"])) for e in index]
    rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
    return np.vstack(rows)


if __name__ == "__main__":
    idx = build()
    cv2.imwrite(str(OUT / "sheet.jpg"), sheet(idx))
    print(len(idx), "faces ->", OUT)
