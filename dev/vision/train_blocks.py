"""Train the 4-class block detector (recyclable / kitchen / hazardous / other) on collect.py datasets.

The images of one scan round are eight views of the same scene, so validation holds out whole rounds
(else the score only says the model remembers the scene). Training uses Ultralytics YOLOv8n (pretrained
yolov8n.pt), 640 px, on CUDA (the operator's RTX 4050 laptop), Apple MPS or CPU; the best weights are
exported to ONNX (opset 12, static 1x3x640x640) for `atc` on the board.

    python3 dev/vision/train_blocks.py data/datasets/blocks4/<ds> [--epochs 80] [--val round_004,round_009]
    python3 dev/vision/train_blocks.py data/datasets/blocks4/<ds> --package   # zip to train elsewhere

On another machine, from the unpacked zip:  pip install ultralytics  then
    python train_blocks.py dataset --epochs 150
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import zipfile
from pathlib import Path

CLASSES = ["recyclable", "kitchen", "hazardous", "other"]
DEFAULT_VAL = 3          # rounds held out when none are named: the last colour-up round and two hand placements


def rounds_of(ds: Path) -> dict[str, list[str]]:
    """Image stems per round from images/<round>_view_XX.jpg."""
    out: dict[str, list[str]] = {}
    for img in sorted((ds / "images").glob("*.jpg")):
        rnd = "_".join(img.stem.split("_")[:2])
        out.setdefault(rnd, []).append(img.stem)
    return out


def pick_val(ds: Path, rounds: list[str]) -> list[str]:
    placed, auto = [], []
    track = ds / "track.jsonl"
    modes = {}
    if track.exists():
        for line in track.read_text().splitlines():
            r = json.loads(line)
            modes[r["round"]] = r.get("mode", "auto")
    for r in rounds:
        (placed if modes.get(r) == "placed" else auto).append(r)
    chosen = auto[-1:] + placed[1::max(1, len(placed) // 2)][:DEFAULT_VAL - 1]
    return chosen or rounds[-1:]


def link(src: Path, dst: Path) -> None:
    try:
        os.symlink(os.path.relpath(src, dst.parent), dst)
    except OSError:                     # Windows without link rights
        shutil.copy(src, dst)


def build_split(ds: Path, val: list[str], synth: Path | None = None, name: str = "split") -> Path:
    """split/{train,val}/{images,labels} as links into images/ and labels/, and split/data.yaml.
    Validation is always real held-out rounds; `synth` (a dev/vision/synth_blocks.py dataset) joins train."""
    split = ds / name
    if split.exists():
        shutil.rmtree(split)
    per_round = rounds_of(ds)
    counts = {"train": 0, "val": 0}
    for rnd, stems in per_round.items():
        part = "val" if rnd in val else "train"
        for sub in ("images", "labels"):
            (split / part / sub).mkdir(parents=True, exist_ok=True)
        for stem in stems:
            for sub, ext in (("images", ".jpg"), ("labels", ".txt")):
                link(ds / sub / f"{stem}{ext}", split / part / sub / f"{stem}{ext}")
            counts[part] += 1
    if synth is not None:
        for img in sorted((synth / "images").glob("*.jpg")):
            link(img, split / "train" / "images" / img.name)
            link(synth / "labels" / f"{img.stem}.txt", split / "train" / "labels" / f"{img.stem}.txt")
            counts["train"] += 1
        counts["synth"] = len(list((synth / "images").glob("*.jpg")))
    (split / "data.yaml").write_text(f"path: {split.resolve()}\ntrain: train/images\nval: val/images\n"
                                     f"nc: {len(CLASSES)}\nnames: {json.dumps(CLASSES)}\n")
    print(f"split: train {counts['train']} images ({counts.get('synth', 0)} synthetic), "
          f"val {counts['val']} images (rounds {', '.join(val)})")
    return split / "data.yaml"


def device() -> str:
    import torch
    if torch.cuda.is_available():
        return "0"
    if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


AUG = dict(hsv_h=0.005, hsv_s=0.4, hsv_v=0.4, degrees=10, translate=0.1, scale=0.3, fliplr=0.5, mosaic=1.0)


def train(ds: Path, epochs: int, val: list[str], model: str, batch: int, synth: Path | None = None,
          finetune: int = 0) -> Path:
    """One run, or -- with `synth` and `finetune` -- pretrain on synthetic + real, then fine-tune on real
    alone at a tenth of the learning rate, which is what decides how the real camera's blocks look."""
    from ultralytics import YOLO
    runs = ds / "train_runs"
    data = build_split(ds, val, synth, "split")
    name = "blocks4"
    YOLO(model).train(data=str(data), epochs=epochs, imgsz=640, batch=batch, device=device(), project=str(runs),
                      name=name, exist_ok=True, patience=max(20, epochs // 4), seed=0, **AUG)
    if synth is not None and finetune:
        real_data = build_split(ds, val, None, "split_real")
        name = "blocks4_ft"
        YOLO(str(runs / "blocks4" / "weights" / "best.pt")).train(
            data=str(real_data), epochs=finetune, imgsz=640, batch=batch, device=device(), project=str(runs),
            name=name, exist_ok=True, patience=max(15, finetune // 3), seed=0, lr0=0.001, warmup_epochs=1.0, **AUG)
        data = real_data
    best = runs / name / "weights" / "best.pt"
    metrics = YOLO(str(best)).val(data=str(data), imgsz=640, device=device(), split="val", plots=False)
    summary = dict(map50=float(metrics.box.map50), map50_95=float(metrics.box.map),
                   per_class_map50={CLASSES[i]: float(v) for i, v in enumerate(metrics.box.ap50)},
                   val_rounds=val, epochs=epochs, model=model,
                   synth=str(synth) if synth else None, finetune_epochs=finetune)
    (runs / name / "summary.json").write_text(json.dumps(summary, indent=1))
    print("val:", json.dumps(summary))
    onnx = YOLO(str(best)).export(format="onnx", imgsz=640, opset=12, simplify=True, dynamic=False)
    print("onnx:", onnx)
    return Path(onnx)


def package(ds: Path) -> Path:
    """dataset (images, labels, track.jsonl) + this script, for training on another machine."""
    out = ds.parent / f"{ds.name}_train_package.zip"
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(__file__, "train_blocks.py")
        for sub in ("images", "labels"):
            for f in sorted((ds / sub).iterdir()):
                z.write(f, f"dataset/{sub}/{f.name}")
        if (ds / "track.jsonl").exists():
            z.write(ds / "track.jsonl", "dataset/track.jsonl")
        z.writestr("README.txt", "pip install ultralytics\npython train_blocks.py dataset --epochs 150\n"
                                 "then bring back dataset/train_runs/blocks4/weights/best.pt and best.onnx\n")
    print("package:", out, f"{out.stat().st_size / 1e6:.1f} MB")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--val", help="comma-separated rounds to hold out")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--synth", type=Path, help="a synth_blocks.py dataset to add to training")
    parser.add_argument("--finetune", type=int, default=0, help="epochs of real-only fine-tuning after it")
    parser.add_argument("--package", action="store_true")
    args = parser.parse_args()
    ds = args.dataset.resolve()
    if args.package:
        package(ds)
        return
    rounds = sorted(rounds_of(ds))
    val = args.val.split(",") if args.val else pick_val(ds, rounds)
    train(ds, args.epochs, val, args.model, args.batch,
          args.synth.resolve() if args.synth else None, args.finetune)


if __name__ == "__main__":
    sys.exit(main())
