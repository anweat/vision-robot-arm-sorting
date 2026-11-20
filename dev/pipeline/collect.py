"""Automatic dataset for a 4-class block detector: the arm turns the blocks over and labels its own scans.

Each block is one category on every face (its colour face and its four stickers, `materials/photos/blocks/` photos), so a
box around a whole block labelled with that block's category is right whichever face is up. The
labels come from tracking, not from recognising:

  round 0   all blocks colour face up: the scene (app/src/pipeline/run.py) gives each block's category
  each round
    1. scan (8 views); the blocks are found class-agnostically; each tracked block is matched to the
       scene block within MATCH_CM of where it was, the block just turned over to the one nearest where
       it was dropped; a coloured top must agree with the tracked category
    2. every view: each block's 3 cm box, projected with that view's camera pose, becomes a YOLO box
    3. one block (in turn) is picked at pitch ~135 and turned over (tumble): the wrist rolls towards 180
       and the pitch flattens while it is high, tilting the held block 50-70 deg, and it is let go 1-2 cm
       above a free spot, so it lands on another face

A round whose matching is not clean (a block missing, an extra one, a colour that disagrees) keeps its
images but writes no labels: an unlabelled block in a labelled image would teach "background".

    python3 dev/pipeline/collect.py --rounds 12          # moves the arm; operator watching
    python3 dev/pipeline/collect.py --relabel <dataset>  # rebuild labels from the saved rounds
    python3 dev/pipeline/collect.py --out <dataset> --rounds 0 --match-cm 3
                                                         # one scan-only round after the operator turned
                                                         # the blocks over by hand, in place

Output data/datasets/blocks4/<ts>/: round_XXX/ (views, scene), images/ + labels/ (YOLO), preview/,
track.jsonl, dataset.yaml.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "dev" / sub))
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

import console_grasp as G  # noqa: E402
import run as R  # noqa: E402
import scene as S  # noqa: E402
import sort_blocks as SB  # noqa: E402

CLASSES = ["recyclable", "kitchen", "hazardous", "other"]
OUT = ROOT / "data" / "datasets" / "blocks4"
MATCH_CM = 2.0          # a block that was not handled may have been nudged this much
DROP_MATCH_CM = 6.0     # a tumbled block lands within this of the drop point
FREE_CM = 7.0           # a drop point keeps this far from the other blocks
GRASP_PITCH = 135       # with J5 near 92 this allows the largest tilt (68 deg at pitch 120, J5 180)
MIN_VISIBLE = 0.1       # a block cut by the image border is still labelled (clipped) if this share is inside:
                        # an unlabelled block would be learnt as background
IMAGE_W, IMAGE_H = 640, 480


# ----------------------------------------------------------------------------------------- geometry

def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def held_rotation(p0: float, r0: float, p1: float, r1: float) -> np.ndarray:
    """Rotation of a block gripped at pitch p0 / J5 r0 when the gripper is at pitch p1 / J5 r1, in the
    arm's vertical plane (the base angle only turns it about the vertical)."""
    return _rx(180 - p1) @ _rz(r1 - r0) @ _rx(180 - p0).T


def tilt_deg(rot: np.ndarray) -> float:
    """Angle between the block's original bottom normal and straight down."""
    return math.degrees(math.acos(float(np.clip(-(rot @ np.array([0, 0, -1.0]))[2], -1, 1))))


def lowest_corner(rot: np.ndarray) -> float:
    """How far below its centre the rotated block's lowest corner is (m)."""
    h = G.BLOCK / 2
    corners = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
    return float(-(corners @ rot.T)[:, 2].min())


def cube_corners(cube: dict) -> np.ndarray:
    """The 8 corners of a block in map metres (bottom on the paper)."""
    x, y = cube["map_cm"][0] / 100, cube["map_cm"][1] / 100
    t = math.radians(cube["yaw_deg"])
    h = S.BLOCK / 2
    pts = []
    for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h)):
        for z in (0.0, S.BLOCK):
            pts.append([x + dx * math.cos(t) - dy * math.sin(t), y + dx * math.sin(t) + dy * math.cos(t), z])
    return np.array(pts)


def yolo_boxes(scan: S.Scan, view: int, cubes: list[dict]) -> list[tuple[int, float, float, float, float]]:
    """(class, cx, cy, w, h) normalised, for each block that is in this view."""
    pose = scan.poses[view]
    out = []
    for c in cubes:
        pts = cube_corners(c)
        cam = pts @ pose.R.T + pose.t
        if (cam[:, 2] < 0.03).any():
            continue
        uv = pose.project(pts, scan.k, scan.dist)
        x0, y0 = uv.min(axis=0)
        x1, y1 = uv.max(axis=0)
        cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, IMAGE_W - 1), min(y1, IMAGE_H - 1)
        if cx1 - cx0 < 8 or cy1 - cy0 < 8:
            continue
        if (cx1 - cx0) * (cy1 - cy0) < MIN_VISIBLE * (x1 - x0) * (y1 - y0):
            continue
        out.append((CLASSES.index(c["category"]), (cx0 + cx1) / 2 / IMAGE_W, (cy0 + cy1) / 2 / IMAGE_H,
                    (cx1 - cx0) / IMAGE_W, (cy1 - cy0) / IMAGE_H))
    return out


# ----------------------------------------------------------------------------------------- tracking

def initial_track(scene: dict) -> list[dict]:
    cats = [c["category"] for c in scene["cubes"]]
    if None in cats or len(set(cats)) != len(cats):
        raise SystemExit(f"round 0 needs every block colour face up and one per category; the scene has {cats}")
    return [dict(category=c["category"], map_cm=c["map_cm"]) for c in scene["cubes"]]


def match(scene: dict, track: list[dict], moved: int | None, drop_cm: list[float] | None) -> tuple[list[dict] | None, str]:
    """Scene blocks labelled with the tracked categories, or (None, why)."""
    free = list(scene["cubes"])
    labelled = []
    order = [i for i in range(len(track)) if i != moved] + ([moved] if moved is not None else [])
    for i in order:
        ref, limit = (drop_cm, DROP_MATCH_CM) if i == moved else (track[i]["map_cm"], MATCH_CM)
        near = min(free, key=lambda c: math.dist(c["map_cm"], ref), default=None)
        if near is None or math.dist(near["map_cm"], ref) > limit:
            return None, f"{track[i]['category']} not found within {limit} cm of {ref}"
        free.remove(near)
        colour_cat = SB.SORTING["colour_category"].get(near["colour"]) if near["colour"] else None
        if colour_cat and colour_cat != track[i]["category"]:
            return None, f"block at {near['map_cm']} shows {near['colour']} but is tracked as {track[i]['category']}"
        labelled.append(dict(near, category=track[i]["category"], track=i))
    if free:
        return None, f"{len(free)} extra block(s) in the scene: {[c['map_cm'] for c in free]}"
    return labelled, ""


def write_labels(out: Path, round_dir: Path, labelled: list[dict]) -> int:
    scan = S.Scan(round_dir)
    scan.register()
    scan.camera_poses()
    (out / "images").mkdir(exist_ok=True)
    (out / "labels").mkdir(exist_ok=True)
    (out / "preview").mkdir(exist_ok=True)
    boxes_total = 0
    for i, v in enumerate(scan.views):
        name = f"{round_dir.name}_{Path(v['image']).stem}"
        boxes = yolo_boxes(scan, i, labelled)
        shutil.copy(round_dir / v["image"], out / "images" / f"{name}.jpg")
        (out / "labels" / f"{name}.txt").write_text("".join(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n" for c, x, y, w, h in boxes))
        boxes_total += len(boxes)
        img = scan.images[i].copy()
        for c, x, y, w, h in boxes:
            p0 = (int((x - w / 2) * IMAGE_W), int((y - h / 2) * IMAGE_H))
            p1 = (int((x + w / 2) * IMAGE_W), int((y + h / 2) * IMAGE_H))
            cv2.rectangle(img, p0, p1, (0, 255, 255), 2)
            cv2.putText(img, CLASSES[c], (p0[0], max(12, p0[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(str(out / "preview" / f"{name}.jpg"), img)
    return boxes_total


def write_yaml(out: Path) -> None:
    (out / "dataset.yaml").write_text(f"path: {out}\ntrain: images\nval: images\nnc: {len(CLASSES)}\n"
                                      f"names: {json.dumps(CLASSES)}\n")


# ----------------------------------------------------------------------------------------- motion

def drop_candidates(track: list[dict], moving: int, rng: random.Random) -> list[tuple[float, float]]:
    """Free map points (cm) in the grasp band, shuffled."""
    others = [t["map_cm"] for i, t in enumerate(track) if i != moving]
    pts = [(x, y) for x in range(-11, 12) for y in range(8, 18)
           if all(math.dist((x, y), o) >= FREE_CM for o in others)]
    rng.shuffle(pts)
    return pts


def plan_tumble(target_cm: tuple[float, float], p0: int, r0: int) -> dict | None:
    """Pitch and J5 at the drop point that tilt the held block most, with reachable high and low poses."""
    bx, by = G.map_to_base(target_cm[0] / 100, target_cm[1] / 100)
    if G.plan_approach(bx, by, 0) is None:          # it must be pickable again later
        return None
    options = []
    for p1 in range(max(115, p0 - 20), min(170, p0 + 15) + 1, 5):
        for r1 in range(G.J5_RANGE[0], G.J5_RANGE[1] + 1, 5):
            rot = held_rotation(p0, r0, p1, r1)
            options.append((tilt_deg(rot), p1, r1, rot))
    options.sort(key=lambda o: -o[0])
    high0 = G.solve(bx, by, G.TABLE_Z + 0.065, [p0], floor=-3)
    if high0 is None or not G.clear_of_platform(high0[1]):
        return None
    for tilt, p1, r1, rot in options[:40]:
        high = G.solve(bx, by, G.TABLE_Z + 0.065, [p1], floor=-3)
        # centre high enough that the lowest corner is ~1 cm up (+1 cm for the fingers' offset from the axis);
        # a block tilted less than ~50 deg would settle back, so it drops from higher and lands with a knock
        up = 0.02 if tilt >= 50 else 0.035
        low = G.solve(bx, by, G.TABLE_Z + lowest_corner(rot) + up, [p1], floor=-3)
        if high and low and G.clear_of_platform(high[1]) and G.clear_of_platform(low[1]):
            return dict(target_cm=list(target_cm), pitch=p1, j5=r1, tilt_deg=round(tilt, 1),
                        high0=high0[1], high=high[1], low=low[1])
    return None


def tumble(sess, plan: dict) -> None:
    """Carry the held block (as in console_grasp.put) to above the drop point at the grasp pitch, roll and
    flatten while high, go down and let go, rise."""
    grasp = json.loads(G.PLAN.read_text())
    s = G.status()
    r0 = s["pose"][4]
    carry = None
    for dp in (0, 5, -5, 10):
        for radius in (G.CARRY_RADIUS, 0.16, 0.18):
            carry = G.solve(0, radius, G.TABLE_Z + G.CARRY_ABOVE_TABLE, [grasp["pitch"] + dp])
            if carry:
                break
        if carry:
            break
    if carry is None:
        raise G.ConsoleError("no carry pose")
    a = [s["pose"][0]] + carry[1][1:]
    b = [plan["high0"][0]] + a[1:]
    for x, y in ((s["pose"], a), (a, b), (b, plan["high0"]), (plan["high0"], plan["high"]), (plan["high"], plan["low"])):
        G.check_path(x, y)
    sess.pose(a + [r0])
    sess.pose(b + [r0])
    sess.pose(plan["high0"] + [r0])
    sess.pose(plan["high"] + [plan["j5"]])
    sess.pose(plan["low"] + [plan["j5"]])
    sess.grip("open")
    sess.pose(plan["high"] + [plan["j5"]])
    G.log("tumbled", **{k: plan[k] for k in ("target_cm", "pitch", "j5", "tilt_deg")})


def act(sess, track: list[dict], cubes: list[dict], r: int, rng: random.Random) -> tuple[int | None, list | None]:
    """Turn over the least recently handled block that can be picked at pitch <= GRASP_PITCH (only far
    blocks, map y >= ~14 cm, can: 2026-09-25 19:52 a block picked at 140 and tilted 48 deg landed on the
    same face). If none can, move the least recently handled block to a free far spot instead, so it can
    be turned over later. Returns (tracked index, drop point cm) of the block that moved."""
    def cube_of(i):
        return min(cubes, key=lambda c: math.dist(c["map_cm"], track[i]["map_cm"]))
    order = sorted(range(len(track)), key=lambda i: track[i].get("moved_round", -1))
    choice = None
    for i in order:
        plan = R.plan_for(cube_of(i), prefer=GRASP_PITCH)
        if plan and plan["pitch"] <= GRASP_PITCH:
            choice = (i, plan, "tumble")
            break
    if choice is None:
        i = order[0]
        choice = (i, R.plan_for(cube_of(i)), "relocate")
    i, plan, mode = choice
    if plan is None:
        print(f"{track[i]['category']}: no grasp this round")
        return None, None
    G.PLAN.write_text(json.dumps(plan, indent=1, ensure_ascii=False))
    G.hover(sess)
    try:
        G.grasp(sess)
    except G.GripMissed as e:
        print("missed:", e)
        return i, track[i]["map_cm"]         # it may have been nudged
    track[i]["moved_round"] = r
    free = drop_candidates(track, i, rng)
    if mode == "tumble":
        r0 = G.status()["pose"][4]
        plans = [t for t in (plan_tumble(p, plan["pitch"], r0) for p in free[:80]) if t]
        if plans:
            best = max(t["tilt_deg"] for t in plans)
            tp = rng.choice([t for t in plans if t["tilt_deg"] >= best - 3])
            tumble(sess, tp)
            return i, tp["target_cm"]
        print("no tumble found; moved instead")
    far = [p for p in free if p[1] >= 15 and (G.plan_approach(*G.map_to_base(p[0] / 100, p[1] / 100), 0, GRASP_PITCH) or {}).get("pitch", 999) <= GRASP_PITCH]
    for target in far + [p for p in free if p not in far]:
        try:
            G.put(f"{target[0]},{target[1]}", sess)    # plans before it moves; an unplannable spot is skipped
        except G.ConsoleError as e:
            if str(e).startswith(("no hover", "no carry")):
                continue
            raise
        G.log("relocated", category=track[i]["category"], to_cm=list(target))
        return i, list(target)
    G.put(f"{track[i]['map_cm'][0]},{track[i]['map_cm'][1]}", sess)
    return i, track[i]["map_cm"]


# ----------------------------------------------------------------------------------------- placed by hand

COLOUR_OF = {"other": "yellow", "recyclable": "blue", "kitchen": "green", "hazardous": "red"}
PLACE_MARKERS = [str(m) for m in range(35, 50)]      # map rows y = 8, 13, 18 cm: all in the scan's view
PLACE_APART_CM = 7.0                                  # not on neighbouring markers (6 x 5 cm grid)


def plan_placement(rng: random.Random) -> dict:
    """Four markers, pairwise >= PLACE_APART_CM apart, one per block colour."""
    while True:
        chosen = rng.sample(PLACE_MARKERS, 4)
        pts = [[v * 100 for v in G.marker_centre(m)] for m in chosen]
        if all(math.dist(a, b) >= PLACE_APART_CM for i, a in enumerate(pts) for b in pts[i + 1:]):
            colours = ["yellow", "blue", "red", "green"]
            rng.shuffle(colours)
            return dict(zip(colours, chosen))


def refine_on_ortho(top: np.ndarray, weight: np.ndarray, centre_cm: list[float], window_mm: int = 20) -> tuple[list[float], float, float]:
    """Centre and yaw of a block placed by hand near centre_cm (hand placements were up to 17 mm off the
    marker centre; the blocks are >= 70 mm apart): cubes.refine_at on the top-plane orthophoto."""
    import cubes as C
    col, row = S.Scan.to_px(centre_cm[0] / 100, centre_cm[1] / 100)
    r = C.refine_at(top, weight, col, row, window_mm)
    x, y = S.Scan.to_map(r["col"], r["row"])
    return [round(x * 100, 2), round(y * 100, 2)], (-r["yaw_deg"] + 45) % 90 - 45, r["score"]


def placed_round(out: Path, placement: dict, session) -> None:
    """Scan once and label the blocks where the operator put them."""
    import field_scan as SC
    rounds = sorted(int(p.name.split("_")[1]) for p in out.glob("round_*"))
    round_dir = out / f"round_{(rounds[-1] + 1) if rounds else 0:03d}"
    SC.scan(run=round_dir, session=session)
    scan = S.Scan(round_dir)
    reg = scan.register()
    scan.camera_poses()
    top, weight = scan.ortho(S.BLOCK)
    labelled = []
    for colour, marker in placement.items():
        centre = [v * 100 for v in G.marker_centre(marker)]
        map_cm, yaw, score = refine_on_ortho(top, weight, centre)
        cube = dict(map_cm=map_cm, yaw_deg=round(yaw, 1), category=SB.SORTING["colour_category"][colour], placed_on=marker, outline=round(score, 3))
        seen, _ = R.top_colour(top, dict(col=int(S.Scan.to_px(map_cm[0] / 100, map_cm[1] / 100)[0]),
                                         row=int(S.Scan.to_px(map_cm[0] / 100, map_cm[1] / 100)[1]), yaw_deg=0))
        cube["top_colour"] = seen
        if seen and seen != colour:
            print(f"warning: marker {marker} should hold {colour} but its top is {seen}")
        if math.dist(map_cm, centre) > 0.7:
            print(f"note: {colour} sits {math.dist(map_cm, centre):.1f} cm from marker {marker} centre")
        labelled.append(cube)
    n = write_labels(out, round_dir, labelled)
    rec = dict(round=round_dir.name, t=time.time(), mode="placed", placement=placement, labelled=labelled,
               registration={k: v for k, v in reg.items() if k != "per_view"})
    with (out / "track.jsonl").open("a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(f"{round_dir.name}: {n} boxes; " + ", ".join(f"{c['category']}@{c['placed_on']} top {c['top_colour'] or 'sticker'} "
                                                     f"yaw {c['yaw_deg']}" for c in labelled))


# ----------------------------------------------------------------------------------------- main

def main() -> None:
    global MATCH_CM
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--out", type=Path, help="continue this dataset directory")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--match-cm", type=float, default=MATCH_CM,
                        help="how far a block may have moved between rounds (hand flips in place: 3)")
    parser.add_argument("--plan-placement", action="store_true", help="choose the next hand placement (no motion)")
    parser.add_argument("--placed", action="store_true", help="scan and label the placement the operator made")
    parser.add_argument("--relabel", type=Path, help="rebuild images/labels of a dataset from its track.jsonl")
    args = parser.parse_args()
    MATCH_CM = args.match_cm
    if args.plan_placement or args.placed:
        out = (args.out or OUT / time.strftime("%Y%m%d_%H%M%S")).resolve()
        out.mkdir(parents=True, exist_ok=True)
        write_yaml(out)
        nxt = out / "next_placement.json"
        if args.plan_placement:
            placement = plan_placement(random.Random(args.seed or time.time()))
            nxt.write_text(json.dumps(placement))
            names = {"yellow": "黄", "blue": "蓝", "red": "红", "green": "绿"}
            print("  ".join(f"{names[c]} -> {m} 号" for c, m in placement.items()))
            return
        placement = json.loads(nxt.read_text())
        with G.Session() as sess:
            placed_round(out, placement, sess)
            G.observe(sess)
        nxt.unlink()
        return
    if args.relabel:
        out = args.relabel.resolve()
        for line in (out / "track.jsonl").read_text().splitlines():
            rec = json.loads(line)
            if rec.get("labelled"):
                n = write_labels(out, out / rec["round"], rec["labelled"])
                print(rec["round"], n, "boxes")
        write_yaml(out)
        return
    import field_scan as SC
    out = (args.out or OUT / time.strftime("%Y%m%d_%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    write_yaml(out)
    rng = random.Random(args.seed or time.time())
    records = [json.loads(line) for line in (out / "track.jsonl").read_text().splitlines()] if (out / "track.jsonl").exists() else []
    track = records[-1]["track"] if records else None
    moved, drop = (records[-1].get("moved"), records[-1].get("drop_cm")) if records else (None, None)
    start = len(records)
    with G.Session() as sess:
        for r in range(start, start + args.rounds + 1):     # the last round only scans
            round_dir = out / f"round_{r:03d}"
            SC.scan(run=round_dir, session=sess)
            scene = R.build_scene(round_dir)
            R.use_registration(scene["registration"])
            if track is None:
                track = initial_track(scene)
            labelled, why = match(scene, track, moved, drop)
            rec = dict(round=round_dir.name, t=time.time(), moved=None, drop_cm=None, labelled=labelled, why=why)
            if labelled is None:
                print(f"{round_dir.name}: no labels ({why})")
                G.log("collect_unlabelled", round=round_dir.name, why=why)
                if "not found" in why or "extra" in why:
                    rec["track"] = track
                    with (out / "track.jsonl").open("a") as f:
                        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                    raise SystemExit("tracking lost: put the blocks back where track.jsonl says, or restart with colour faces up")
            else:
                n = write_labels(out, round_dir, labelled)
                track = [dict(t, map_cm=c["map_cm"]) for t, c in zip(track, sorted(labelled, key=lambda c: c["track"]))]
                print(f"{round_dir.name}: {n} boxes; tops {[(c['category'], c['colour'] or 'sticker') for c in labelled]}")
            moved, drop = None, None
            if r < start + args.rounds:
                moved, drop = act(sess, track, labelled or scene["cubes"], r, rng)
            rec.update(moved=moved, drop_cm=drop, track=track)
            with (out / "track.jsonl").open("a") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        G.observe(sess)
    print("dataset:", out)


if __name__ == "__main__":
    main()
