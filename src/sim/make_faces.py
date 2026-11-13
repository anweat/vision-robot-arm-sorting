#!/usr/bin/env python3
"""Generate the missing reference block-face textures so src/sim can render without the team's photos.

The repository ships no data/reference/block_faces/ (it is not committed), but src/sim/world.py reads
index.json there and src/sim/render.py loads <category>__<item>.png out of the same folder. Until the real
photographed faces arrive, this writes plain colour art for the four categories:

  index entry -> {"item": suffix, "category": c, "colour_face": bool, "file": "<category>__<suffix>.png"}
  render.py   -> loads  <category>__colour_<colour>.png   and   <category>__<item>.png

So the item names must be the file suffixes, and colour_<colour> must match the hardcoded colour path.

    python src/sim/make_faces.py
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
FACES = ROOT / "data" / "reference" / "block_faces"

# must match src/sim/world.py
CATEGORIES = ("recyclable", "kitchen", "hazardous", "other")
COLOURS = {"recyclable": "blue", "kitchen": "green", "hazardous": "red", "other": "yellow"}
BGR = {"blue": (205, 110, 45), "green": (95, 165, 75), "red": (65, 65, 215), "yellow": (75, 185, 225)}
LABEL = {"recyclable": "R", "kitchen": "K", "hazardous": "H", "other": "O"}
SIZE = 256


def shade(face: np.ndarray, centre=(0.0, 0.0)) -> np.ndarray:
    """Lambert-ish vignette so the six faces do not read as one flat colour."""
    ys, xs = np.mgrid[0:SIZE, 0:SIZE] / SIZE
    fall = 1.0 - 0.35 * (((xs - centre[0]) ** 2 + (ys - centre[1]) ** 2) / 2.0) ** 0.6
    return np.clip(face * fall[..., None], 0, 255).astype(np.uint8)


def main() -> None:
    FACES.mkdir(parents=True, exist_ok=True)
    index: list[dict] = []

    for category in CATEGORIES:
        colour = COLOURS[category]
        base = np.full((SIZE, SIZE, 3), BGR[colour], np.uint8)

        colour_file = f"{category}__colour_{colour}.png"
        cv2.imwrite(str(FACES / colour_file), shade(base))
        index.append({"item": f"colour_{colour}", "category": category, "colour_face": True,
                      "file": colour_file, "source": "generated"})

        for i in range(1, 5):                     # four stickers round the sides
            sticker_file = f"{category}__sticker_{i}.png"
            face = shade(base, centre=(0.35 + 0.1 * i, 0.4))
            cv2.putText(face, LABEL[category], (SIZE // 2 - 34, SIZE // 2 + 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.6, (250, 250, 250), 7, cv2.LINE_AA)
            cv2.rectangle(face, (14, 14), (SIZE - 14, SIZE - 14), (250, 250, 250), 6)
            cv2.imwrite(str(FACES / sticker_file), face)
            index.append({"item": f"sticker_{i}", "category": category, "colour_face": False,
                          "file": sticker_file, "source": "generated"})

    (FACES / "index.json").write_text(json.dumps(index, indent=1, ensure_ascii=False), encoding="utf-8")
    print(f"wrote {len(index)} index entries and their textures -> {FACES}")


if __name__ == "__main__":
    main()
