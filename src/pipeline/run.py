"""Camera-only sorting pipeline: scan -> register the map -> find cubes -> classify -> grasp and place.

    python3 src/pipeline/run.py                      # scan now, plan, sort (moves the arm; operator watching)
    python3 src/pipeline/run.py --scan <run_dir>     # reuse a scan
    python3 src/pipeline/run.py --dry-run            # scan + plan only; writes scene.json and scene.png

The map is registered from this scan (no touching, no phone). Cube centre and yaw come from the top-plane
orthophoto (src/pipeline/cubes.py); category from the official 3-class model (its detections from the scan
views, projected onto the top plane) when confident and consistent with the top colour, else the colour
(src/tasks/sort_blocks.resolve). The motion reuses the verified console stages (hover, grasp, put) with
the plan made from the scan instead of a close look.

With --bins the targets are the physical acrylic bins (config/bins_acrylic_v1.json) instead of map codes: each
bin is found from its own stickers in extra views (field_scan.scan_bins), a block is carried over its bin's
opening and let go from above the rim, into the next free slot (config/sorting.json "bins").
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import console_grasp as G  # noqa: E402
import cubes as C  # noqa: E402
import bins as B  # noqa: E402
import scene as S  # noqa: E402
import scramble as SCR  # noqa: E402
import sort_blocks as SB  # noqa: E402
from block_detector import hue_label  # noqa: E402

COLOURS = ("red", "green", "blue", "yellow")
DET_MERGE_CM = 2.5       # detections of one block from different views land within this on the mid plane
DET_MIN_CONF = 0.4
DET_STRONG_SHARE = 0.6   # the leading category's share of the summed confidences
DET_STRONG_SUM = 0.8
DET_CUT_MERGE_CM = 3.5   # a box cut by the image border casts its centre off the block's
DET_SAME_BLOCK_CM = 3.0  # a cluster this near an outline-found block is that block
DET_SINGLE_VIEW_CONF = 0.75
ORTHO_AGREE_CM = 1.2     # else the orthophoto square latched onto a side face (blue, 2.1 cm off, 2026-09-25)
MIN_SEPARATION_CM = 3.2  # two 3 cm blocks cannot be nearer; a nearer pair is one block smeared on the plane
SMEAR_CM = 5.0           # an outline with no detection this near a detected block is that block's own side


def top_colour(top: np.ndarray, cube: dict) -> tuple[str | None, float]:
    """Dominant saturated hue inside the inner 60 % of the top face, or None for a white/sticker top."""
    mask = np.zeros(top.shape[:2], np.uint8)
    k = C.square_kernel(cube["yaw_deg"], C.SIDE_PX * 0.6)
    h = k.shape[0] // 2
    r0, c0 = cube["row"] - h, cube["col"] - h
    if r0 < 0 or c0 < 0 or r0 + k.shape[0] > mask.shape[0] or c0 + k.shape[1] > mask.shape[1]:
        return None, 0.0
    mask[r0:r0 + k.shape[0], c0:c0 + k.shape[1]] = (k > 0).astype(np.uint8)
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    inside = hsv[mask > 0]
    # V > 35: the green top reads V ~50 in the folded near views (2026-09-25); paper and shadows stay out on S
    saturated = inside[(inside[:, 1] > 90) & (inside[:, 2] > 35)]
    share = len(saturated) / max(1, len(inside))
    if share < 0.4:
        return None, round(share, 2)
    hues = saturated[:, 0].astype(float)
    if np.mean((hues < 12) | (hues > 165)) > 0.5:
        hues = np.where(hues < 90, hues + 180, hues)
    label = hue_label(float(np.median(hues)) % 180)
    return (label if label in COLOURS else None), round(share, 2)


def official_on_cubes(scan: S.Scan, cubes: list[dict]) -> None:
    """Attach to each cube the best official detection whose box centre, cast onto the top plane, lands
    within 2.5 cm of the cube centre (background detections land nowhere near a cube)."""
    for c in cubes:
        c["sticker"] = None
    for view, pose in zip(scan.views, scan.poses):
        for d in view.get("official") or []:
            if "bbox" not in d or d.get("conf", 0) < SB.SORTING["model_min_conf"]:
                continue
            x0, y0, x1, y1 = d["bbox"]
            p = pose.pixel_to_plane(((x0 + x1) / 2, (y0 + y1) / 2), scan.k, scan.dist, z=S.BLOCK)
            near = min(cubes, key=lambda c: math.dist(c["map_cm"], (p[0] * 100, p[1] * 100)), default=None)
            if near is None or math.dist(near["map_cm"], (p[0] * 100, p[1] * 100)) > 2.5:
                continue
            if near["sticker"] is None or d["conf"] > near["sticker"]["conf"]:
                near["sticker"] = dict(label=d["label"], category=d["category"], conf=d["conf"], view=view["image"])


def detector_clusters(scan: S.Scan) -> list[dict]:
    """The 4-class block detector (src/vision/blocks_detector.py) on every view; each box centre is cast
    onto the plane of the block centres (z = 1.5 cm), and the casts within DET_MERGE_CM are one block,
    with confidence votes per category. Boxes cut by the image border are left out: their centre is not
    the block's. [] when no detector can run here."""
    try:
        from blocks_detector import BlocksDetector
        det = BlocksDetector()
    except Exception as e:          # no model file, no onnxruntime / ais_bench
        print("block detector unavailable:", e)
        return []
    casts, cut = [], []
    for v, (img, pose) in enumerate(zip(scan.images, scan.poses)):
        h, w = img.shape[:2]
        for d in det.detect(img, conf=DET_MIN_CONF):
            x0, y0, x1, y1 = d["bbox"]
            p = pose.pixel_to_plane(((x0 + x1) / 2, (y0 + y1) / 2), scan.k, scan.dist, z=S.BLOCK / 2)
            c = dict(xy=[p[0] * 100, p[1] * 100], category=d["category"], conf=d["conf"], view=v)
            (cut if x0 <= 2 or y0 <= 2 or x1 >= w - 3 or y1 >= h - 3 else casts).append(c)
    clusters: list[dict] = []
    for c in sorted(casts, key=lambda c: -c["conf"]):
        near = [k for k in clusters if math.dist(k["xy"], c["xy"]) <= DET_MERGE_CM]
        if near:
            k = near[0]
            k["members"].append(c)
            wsum = sum(m["conf"] for m in k["members"])
            k["xy"] = [sum(m["xy"][i] * m["conf"] for m in k["members"]) / wsum for i in (0, 1)]
        else:
            clusters.append(dict(xy=list(c["xy"]), members=[c]))
    # A box cut by the image border has a biased centre, so it joins a whole box's cluster when there is
    # one nearby. Blocks near the map's edges are cut in every view, though (11 of 60 were lost this way,
    # 2026-09-26), so the rest cluster among themselves and are marked approximate: the orthophoto then
    # squares them up over a wider window.
    leftover = []
    for c in cut:
        near = min(clusters, key=lambda k: math.dist(k["xy"], c["xy"]), default=None)
        if near is not None and math.dist(near["xy"], c["xy"]) <= DET_CUT_MERGE_CM:
            near.setdefault("cut", []).append(c)
        else:
            leftover.append(c)
    for c in sorted(leftover, key=lambda c: -c["conf"]):
        near = [k for k in clusters if k.get("approx") and math.dist(k["xy"], c["xy"]) <= DET_CUT_MERGE_CM]
        if near:
            near[0]["members"].append(c)
        else:
            clusters.append(dict(xy=list(c["xy"]), members=[c], approx=True))
    for k in clusters:
        k["members"] += k.pop("cut", [])
    for k in clusters:
        votes: dict[str, float] = {}
        for m in k["members"]:
            votes[m["category"]] = votes.get(m["category"], 0.0) + m["conf"]
        k["votes"] = {c: round(v, 2) for c, v in sorted(votes.items(), key=lambda t: -t[1])}
        k["views"] = len({m["view"] for m in k["members"]})
        k["xy"] = [round(v, 2) for v in k["xy"]]
        k["approx"] = bool(k.get("approx"))
        del k["members"]
    return clusters


def categorize(cube: dict) -> tuple[str | None, str]:
    """A coloured top decides (it is the block's own colour); otherwise the detector's views vote; the
    official 3-class model only when neither says anything."""
    by_colour = SB.SORTING["colour_category"].get(cube["colour"]) if cube.get("colour") else None
    votes = (cube.get("detector") or {}).get("votes") or {}
    lead = max(votes, key=votes.get) if votes else None
    strong = bool(lead) and votes[lead] >= DET_STRONG_SUM and votes[lead] / sum(votes.values()) >= DET_STRONG_SHARE
    if by_colour:
        if strong and lead != by_colour:
            return by_colour, f"colour; detector disagreed ({votes})"
        return by_colour, "colour" + (", detector agrees" if strong else "")
    if strong:
        return lead, f"detector {votes} over {cube['detector']['views']} views"
    return SB.resolve(None, cube.get("sticker"))


def dedupe(cubes: list[dict]) -> list[dict]:
    """Drop a block too near a better-evidenced one: on the plane of the tops a block's own side smears
    downwards and is squared up as a second block 2.5-3.3 cm away (2026-09-26). Detector evidence outranks
    a bare outline, then the stronger outline."""
    order = sorted(cubes, key=lambda c: (c.get("detector") is not None, c["score"]), reverse=True)
    kept: list[dict] = []
    for c in order:
        # a block the detector never saw, next to one it did, is that block's own painted side cast onto
        # the top plane (a yellow side read as a second block 3.3 cm away, 2026-09-26)
        limit = SMEAR_CM if c.get("detector") is None else MIN_SEPARATION_CM
        if all(math.dist(c["map_cm"], k["map_cm"]) >= (limit if k.get("detector") is not None else MIN_SEPARATION_CM)
               for k in kept):
            kept.append(c)
    return [c for c in cubes if c in kept]


def build_scene(run: Path) -> dict:
    scan = S.Scan(run)
    reg = scan.register()
    scan.camera_poses()
    top, weight = scan.ortho(S.BLOCK)
    fg = scan.foreground(S.BLOCK)
    # the map difference only knows the paper: a cube half over the map's edge had fill 0.3 (green,
    # 2026-09-25 19:28). A saturated face is a cube wherever it lies (table S/V ~0/0, paper 2 % saturated).
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    coloured = ((hsv[..., 1] > 90) & (hsv[..., 2] > 35)).astype(np.float32)
    fg = np.maximum(fg, cv2.medianBlur((coloured * 255).astype(np.uint8), 5).astype(np.float32) / 255)
    found = C.find_cubes(top, weight, min_score=0.2, foreground=fg)
    cubes = []
    for c in found:
        x, y = S.Scan.to_map(c["col"], c["row"])
        yaw_map = (-(c["yaw_deg"]) + 45) % 90 - 45        # image rows run towards -y: the angle flips sign
        colour, share = top_colour(top, c)
        # coverage at the cube: a cube at the edge of what the views saw is only partly measured
        r0, r1, c0, c1 = max(0, c["row"] - 18), c["row"] + 18, max(0, c["col"] - 18), c["col"] + 18
        coverage = float((weight[r0:r1, c0:c1] > 0.05).mean())
        cubes.append(dict(map_cm=[round(x * 100, 2), round(y * 100, 2)], yaw_deg=round(yaw_map, 1), score=c["score"],
                          fill=c["fill"], colour=colour, colour_share=share, coverage=round(coverage, 2),
                          px=[c["col"], c["row"]], img_yaw=c["yaw_deg"]))
    # the detector: its evidence for the blocks found above, and the blocks the orthophoto missed (a white
    # sticker top on white paper has almost no outline: 1 of 4 found on 2026-09-25 22:10)
    # Match clusters to the blocks the outline search found, nearest pair first and one cluster per block:
    # a cluster whose centre is pulled off by clipped boxes would otherwise claim its neighbour's block, and
    # the neighbour's cluster would claim it back, losing a block entirely (blue at the map's edge, 2026-09-26).
    clusters = detector_clusters(scan)
    pairs = sorted(((math.dist(c["map_cm"], k["xy"]), ki, ci)
                    for ki, k in enumerate(clusters) for ci, c in enumerate(cubes)
                    if math.dist(c["map_cm"], k["xy"]) <= DET_SAME_BLOCK_CM), key=lambda p: p[0])
    taken_k: set[int] = set()
    taken_c: set[int] = set()
    matched: dict[int, tuple[dict, float]] = {}
    for dist, ki, ci in pairs:
        if ki in taken_k or ci in taken_c:
            continue
        taken_k.add(ki)
        taken_c.add(ci)
        matched[ki] = (cubes[ci], dist)     # the cube itself: the list is edited below
    for ki, k in enumerate(clusters):
        if ki in matched:
            cube, dist = matched[ki]
            cube["detector"] = k
            if dist <= ORTHO_AGREE_CM:
                continue
            cubes.remove(cube)              # far apart: re-square it where the detector puts it
        elif k["views"] < 2 and max(k["votes"].values()) < DET_SINGLE_VIEW_CONF:
            continue
        col, row = S.Scan.to_px(k["xy"][0] / 100, k["xy"][1] / 100)
        r = C.refine_at(top, weight, col, row, window_px=25 if k["approx"] else 12)
        found.append(dict(col=r["col"], row=r["row"], yaw_deg=r["yaw_deg"], score=r["score"], fill=None))
        x, y = S.Scan.to_map(r["col"], r["row"])
        colour, share = top_colour(top, r)
        r0, r1, c0, c1 = max(0, r["row"] - 18), r["row"] + 18, max(0, r["col"] - 18), r["col"] + 18
        cubes.append(dict(map_cm=[round(x * 100, 2), round(y * 100, 2)], yaw_deg=round((-r["yaw_deg"] + 45) % 90 - 45, 1),
                          score=r["score"], fill=None, colour=colour, colour_share=share,
                          coverage=round(float((weight[r0:r1, c0:c1] > 0.05).mean()), 2),
                          px=[r["col"], r["row"]], img_yaw=r["yaw_deg"], detector=k, found_by="detector"))
    cubes = dedupe(cubes)
    found = [f for f in found if any(c["px"] == [f["col"], f["row"]] for c in cubes)]
    official_on_cubes(scan, cubes)
    for c in cubes:
        c["category"], c["source"] = categorize(c)
    vis = C.draw(top, found)
    for i, c in enumerate(cubes):
        tag = f"{c['colour'] or '-'} {(c['category'] or '?')[:4]}"
        cv2.putText(vis, tag, (c["px"][0] - 18, c["px"][1] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.imwrite(str(run / "scene.png"), cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    scene = dict(run=str(run.relative_to(ROOT) if run.is_relative_to(ROOT) else run), registration={k: v for k, v in reg.items() if k != "per_view"},
                 cubes=cubes, t=time.time())
    (run / "scene.json").write_text(json.dumps(scene, indent=1, ensure_ascii=False))
    return scene


REFINE_BELOW_COVERAGE = 0.98
# 2026-09-26, 60 placed blocks: every block whose centre or yaw came out wrong had either a weak outline
# (score below 0.4) or was found only in boxes cut by the image border ("approx"), which are the blocks at
# the edge of what the scan saw and are smeared on the top-plane orthophoto. A close view fixes both.
REFINE_BELOW_SCORE = 0.4


def needs_refine(cube: dict) -> bool:
    return (cube["coverage"] < REFINE_BELOW_COVERAGE or cube["score"] < REFINE_BELOW_SCORE
            or bool((cube.get("detector") or {}).get("approx")))


def refine(run: Path, scene: dict, session) -> dict:
    """One extra view aimed at each cube that was only partly seen or weakly detected, then the scene
    is rebuilt from all views. A wrong yaw lands the fingers on the cube's corners, and a jammed grip
    once latched the gripper servo until a power cycle (docs/05, 2026-09-26)."""
    import field_scan as SC
    weak = [c for c in scene["cubes"] if needs_refine(c)
            and not (BINS is not None and BINS.containing(G.map_to_base(c["map_cm"][0] / 100, c["map_cm"][1] / 100),
                                                          SB.SORTING["bins"]["look_margin_cm"] / 100))]   # a block in a bin is done
    if not weak:
        return scene
    views = json.loads((run / "views.json").read_text())
    for c in weak:
        bx, by = G.map_to_base(c["map_cm"][0] / 100, c["map_cm"][1] / 100)
        try:
            look = G.look_pose(bx, by, G.status()["pose"])
            G.move_to(session, look, G.ROLL_ZERO)
        except G.ConsoleError as e:        # no view of it that keeps the fingers off a bin: it is judged from the scan alone
            G.log("refine_skipped", cube=c["map_cm"], reason=str(e))
            continue
        jpeg, s = SC.frame_at_rest()
        name = f"view_{len(views):02d}.jpg"
        (run / name).write_bytes(jpeg)
        board = s["localization"].get("board") or {}
        views.append(dict(image=name, readback=s["pose"], command=look + [G.ROLL_ZERO], markers=board.get("markers"),
                          official=board.get("official", []), t=time.time(), refine_for=c["map_cm"]))
        G.log("refine_view", cube=c["map_cm"], pose=s["pose"])
    (run / "views.json").write_text(json.dumps(views, indent=1, ensure_ascii=False))
    return build_scene(run)


STATE_FILE = ROOT / "data" / "runs" / "bins_state.json"    # blocks given to each bin, across rounds and runs
BINS: B.BinSet | None = None      # set by use_bins(): the targets are acrylic bins, not map codes
TABLE_Z_DISAGREE_CM = 1.5   # beyond this the scan and the touched table are not describing one surface


def use_registration(reg: dict) -> None:
    """Where the map lies, for every base <-> map conversion. Its x and y are taken; its z is not.

    The scan's z comes through the hand-eye and the marker pose, and it reads the paper about 0.8 cm
    higher than the fingertips find the table: touching four marker centres on 2026-09-29 agreed with the
    calibrated -1.42 cm to a median of 0.05 cm, while that run's registration said -0.62 cm. Taking the
    scan's z made every descent aim 0.8 cm high, which on a 3 cm block puts the fingers in its top third
    -- where a closing finger has the most leverage to tip it over, and two blocks were flicked away.
    The height a descent has to be right about is the one the fingertips measured, so it stays."""
    G.DX, G.DY = reg["map_origin_base_cm"][0] / 100, reg["map_origin_base_cm"][1] / 100
    seen = reg["map_origin_base_cm"][2]
    if abs(seen - G.TABLE_Z * 100) > TABLE_Z_DISAGREE_CM:
        G.log("table_z_disagrees", scan_cm=round(seen, 2), calibrated_cm=round(G.TABLE_Z * 100, 2),
              note="用的是标定值；差得这么多说明相机、地图或机械臂安装变了，该重新做触点标定")


def plan_for(cube: dict, prefer: int = 150) -> dict | None:
    """The console_grasp plan for one cube, from the scan (no close look)."""
    bx, by = G.map_to_base(cube["map_cm"][0] / 100, cube["map_cm"][1] / 100)
    approach = G.plan_approach(bx, by, cube["yaw_deg"], prefer)
    if approach is None:
        return None
    return dict(colour=cube.get("colour"), created=time.time(), block_base_cm=[round(bx * 100, 2), round(by * 100, 2)],
                board=dict(map_cm=cube["map_cm"], sticker=cube.get("sticker")), block_yaw_deg=cube["yaw_deg"],
                block_centre_z_cm=round((G.TABLE_Z + G.GRASP_HEIGHT) * 100, 2), **approach)


def use_bins(found: B.BinSet | None) -> None:
    """Sort into these bins from now on (None: back to the map codes)."""
    global BINS
    BINS = found
    boxes = []
    for b in (found.bins.values() if found else ()):
        c, s = math.cos(math.radians(b.yaw_deg)), math.sin(math.radians(b.yaw_deg))
        half = found.design["geometry_mm"]["outer"] / 2000
        boxes.append(([[c, -s], [s, c]], [float(b.xy[0]), float(b.xy[1]), G.TABLE_Z + found.wall_top_m / 2],
                      [half, half, found.wall_top_m / 2]))
    G.set_obstacles(boxes)


def saved_counts(design: str) -> dict[int, int]:
    """How many blocks this process or an earlier run gave each bin. The scan alone undercounts: blocks stacked in a
    bin hide one another, and a bin that looks half empty from above may be full."""
    try:
        return {int(n): int(c) for n, c in json.loads(STATE_FILE.read_text()).get(design, {}).items()}
    except (OSError, ValueError):
        return {}


def save_counts() -> None:
    try:
        state = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        state = {}
    state[BINS.name] = {str(n): c for n, c in BINS.given.items()}
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps(state, indent=1))


def forget_counts(design: str) -> None:
    """The blocks were taken out of the bins: start counting again."""
    try:
        state = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        return
    state.pop(design, None)
    STATE_FILE.write_text(json.dumps(state, indent=1))


def bin_drop_z(bins: B.BinSet) -> float:
    return G.bin_drop_tip_z(bins.wall_top_m)


def bin_slots() -> list:
    return B.drop_slots(SB.SORTING["bins"]["slots_cm"])


def usable_slots(b: B.Bin) -> list[int]:
    """Indices of the drop slots of bin `b` the arm can reach at drop height, those inside the soft joint
    limits first (the outer corners of bins 1 and 4 sit at J1 ~ 20 and ~ 160)."""
    ok = []
    for i in range(len(bin_slots())):
        where = ",".join(str(v) for v in bin_drop_map_cm(b, i))
        plan = G.plan_put(150, where, list(G.OBSERVE) + [G.ROLL_ZERO], bin_drop_z(BINS))
        if plan is not None:
            ok.append((not plan["within_soft_limits"], i))
    return [i for _, i in sorted(ok)]


def bin_drop_map_cm(b: B.Bin, slot: int) -> list[float]:
    """Map cm of the point above which a block goes into `slot` of bin `b`."""
    x, y = B.drop_xy(b, bin_slots()[slot])
    return [round(float(x - G.DX) * 100, 2), round(float(y - G.DY) * 100, 2)]


def decide_bins(scene: dict) -> list[dict]:
    """Jobs for physical bins: a block inside a bin's opening is done (or in the wrong bin and left there:
    it cannot be taken out again), a detection on a bin's plate that is not in the opening is the plate's
    own stickers and paper, not a block."""
    margin = SB.SORTING["bins"]["look_margin_cm"] / 100
    holding = {n: 0 for n in BINS.bins}
    jobs = []
    for c in scene["cubes"]:
        c["_base"] = G.map_to_base(c["map_cm"][0] / 100, c["map_cm"][1] / 100)
        c["_in_bin"] = BINS.containing(c["_base"], margin)
        if c["_in_bin"] is not None:
            holding[c["_in_bin"].number] += 1
    saved = saved_counts(BINS.name)
    for n, held in holding.items():
        BINS.given[n] = max(held, saved.get(n, 0))
        holding[n] = BINS.given[n]
    BINS.slots = {b.number: usable_slots(b) for b in BINS.bins.values()}
    reachable = {n: bool(slots) for n, slots in BINS.slots.items()}
    G.log("bin_slots", slots={str(n): slots for n, slots in BINS.slots.items()},
          note="每个桶机械臂在投放高度够得着的落点序号（先软限位内的）；容量 = 个数")
    for c in scene["cubes"]:
        holder, base = c.pop("_in_bin"), c.pop("_base")
        if holder is None and BINS.over_footprint(base, margin) is not None:
            G.log("bin_plate_ignored", cube=c["map_cm"], bin=str(BINS.over_footprint(base, margin).number))
            continue
        target = BINS.of_category(c["category"]) if c["category"] else None
        bx, by = base
        reach = math.hypot(bx, by)
        why = None
        if c["category"] is None:
            why = c["source"]
        elif holder is not None:
            why = f"already in bin {holder.number}" + ("" if holder.category == c["category"] else
                                                       f" ({holder.category}, but it is {c['category']}): left there")
        elif target is None:
            why = f"no bin for {c['category']}"
        elif not reachable[target.number]:
            why = f"bin {target.number} cannot be reached at drop height"
        elif holding[target.number] >= len(BINS.slots[target.number]):
            why = f"bin {target.number} is full ({holding[target.number]} blocks)"
            G.log("bin_full", bin=str(target.number), count=holding[target.number], category=target.category, by="pipeline")
        elif plan_for(c) is None:
            why = f"{reach*100:.1f} cm from the base: no reachable grasp at pitch 130-170"
        elif c["coverage"] < 0.9:
            why = f"only {c['coverage']*100:.0f}% of it was in view; rescan with it nearer the middle"
        jobs.append(dict(cube=c, bin=str(target.number) if target else None, skip=why))
    return jobs


def decide(scene: dict) -> list[dict]:
    if BINS is not None:
        return decide_bins(scene)
    bins = {cat: [v * 100 for v in G.marker_centre(m)] for cat, m in SB.SORTING["category_bins"].items()}
    jobs = []
    for c in scene["cubes"]:
        home = [cat for cat, p in bins.items() if math.dist(c["map_cm"], p) <= SB.SORTING["in_place_cm"] + 0.5]
        bx, by = G.map_to_base(c["map_cm"][0] / 100, c["map_cm"][1] / 100)
        reach = math.hypot(bx, by)
        why = None
        if c["category"] is None:
            why = c["source"]
        elif home and home[0] == c["category"]:
            why = f"already on its bin ({c['category']})"
        elif plan_for(c) is None:
            why = f"{reach*100:.1f} cm from the base: no reachable grasp at pitch 130-170"
        elif c["coverage"] < 0.9:
            why = f"only {c['coverage']*100:.0f}% of it was in view; rescan with it nearer the middle"
        jobs.append(dict(cube=c, bin=SB.SORTING["category_bins"].get(c["category"]) if c["category"] else None,
                         skip=why))
    return jobs


def blocker_of(bin_: str, positions: list[dict], mover: dict | None = None) -> dict | None:
    """The block sitting on `bin_` that is not the one being carried there."""
    target = [v * 100 for v in G.marker_centre(bin_)]
    for p in positions:
        if p is not mover and math.dist(p["map_cm"], target) <= SB.SORTING["occupied_cm"]:
            return p
    return None


def execute(run, scene: dict, jobs: list[dict], sess, rng=None) -> dict:
    """Sort every block the scan gave a category to. A block whose bin is occupied waits for the block
    on that bin to move first; when no block can move -- two blocks each standing on the other's bin --
    the one in the way is lifted onto a free marker, because a round that stops there sorts nothing."""
    rng = rng or random.Random()
    started_run = time.time()
    G.log("pipeline_start", run=str(run), cubes=[c["map_cm"] for c in scene["cubes"]],
          registration=scene["registration"])
    counts = dict(sorted=0, skipped=0, missed=0, cleared=0)
    positions = [dict(map_cm=list(j["cube"]["map_cm"]), cube=j["cube"]) for j in jobs]
    by_cube = {id(p["cube"]): p for p in positions}

    pending = []
    for j in jobs:
        if j["skip"]:
            counts["skipped"] += 1
            G.log("pipeline_skip", cube=j["cube"]["map_cm"], category=j["cube"]["category"], reason=j["skip"])
        else:
            pending.append(j)

    def carry(j, bin_or_marker, why):
        """One block to one place; True when it is there."""
        if BINS is not None and why == "sort":
            n = int(bin_or_marker)
            if BINS.given[n] >= len(BINS.slots[n]):        # earlier blocks of this round filled it: do not pick this one up
                G.log("bin_full", bin=bin_or_marker, count=BINS.given[n], category=BINS.bins[n].category, by="pipeline")
                G.log("pipeline_skip", cube=j["cube"]["map_cm"], category=j["cube"]["category"],
                      reason=f"bin {n} is full ({BINS.given[n]} blocks)")
                counts["skipped"] += 1
                return False
        plan = plan_for(j["cube"])
        if plan is None:
            counts["skipped"] += 1
            G.log("pipeline_skip", cube=j["cube"]["map_cm"], category=j["cube"]["category"],
                  reason="no reachable grasp")
            return False
        G.PLAN.write_text(json.dumps(plan, indent=1, ensure_ascii=False))
        started = time.time()
        try:
            G.hover(sess)
        except G.ConsoleError as e:
            # resistance on the way in: one block's approach failing must not end the round, and the arm
            # may have stopped somewhere the next approach could not plan a path out of
            counts["skipped"] += 1
            G.log("pipeline_approach_failed", cube=j["cube"]["map_cm"], category=j["cube"]["category"],
                  reason=str(e), seconds=round(time.time() - started))
            G.back_off(sess)
            return False
        try:
            G.grasp(sess)
        except G.GripMissed as e:
            counts["missed"] += 1
            G.log("pipeline_missed", cube=j["cube"]["map_cm"], category=j["cube"]["category"], reason=str(e),
                  seconds=round(time.time() - started))
            return False
        try:
            if BINS is not None and why == "sort":
                b = BINS.bins[int(bin_or_marker)]
                slot = BINS.slots[b.number][BINS.given[b.number]]
                where_cm = bin_drop_map_cm(b, slot)
                G.put(",".join(str(v) for v in where_cm), sess, drop_tip_z=bin_drop_z(BINS))
            else:
                G.put(bin_or_marker, sess)
        except G.ConsoleError as e:
            # holding a block with nowhere to put it: the next hover would open the fingers and drop it
            try:
                where = G.release_safely(sess)
            except G.ConsoleError as back:
                raise SCR.StillHolding(f"放不下也放不回，机械臂还夹着这一块：{back}") from back
            counts["skipped"] += 1
            G.log("pipeline_put_failed", cube=j["cube"]["map_cm"], bin=bin_or_marker,
                  category=j["cube"]["category"], reason=str(e), released=where)
            return False
        here = by_cube[id(j["cube"])]
        extra = {}
        if BINS is not None and why == "sort":
            here["map_cm"] = where_cm
            BINS.given[int(bin_or_marker)] += 1
            save_counts()
            extra = dict(slot=slot, drop_map_cm=where_cm)
        else:
            here["map_cm"] = [round(v * 100, 1) for v in G.marker_centre(bin_or_marker)]
        G.log("pipeline_sorted" if why == "sort" else "pipeline_cleared",
              cube=j["cube"]["map_cm"], bin=bin_or_marker, category=j["cube"]["category"],
              source=j["cube"]["source"], seconds=round(time.time() - started), **extra)
        counts["sorted" if why == "sort" else "cleared"] += 1
        return True

    while pending:
        free = [j for j in pending if BINS is not None or blocker_of(j["bin"], positions, by_cube[id(j["cube"])]) is None]
        if not free:
            # every remaining block is blocked by another one: lift one blocker out of the way first
            stuck = pending[0]
            blocker = blocker_of(stuck["bin"], positions, by_cube[id(stuck["cube"])])
            job = next((j for j in pending if j["cube"] is blocker["cube"]), dict(cube=blocker["cube"], bin=None))
            taken = [p["map_cm"] for p in positions]
            try:
                spot = next((m for m in SCR.draw(len(pending) + 2, rng)
                             if all(math.dist([v * 100 for v in G.marker_centre(m)], p) >= SCR.MIN_APART_CM
                                    for p in taken)), None)
            except ValueError:
                spot = None
            if spot is None or not carry(job, spot, "clear"):
                G.log("pipeline_stuck", bin=stuck["bin"], blocker=blocker["map_cm"])
                break
            continue
        j = free[0]
        pending.remove(j)
        carry(j, j["bin"], "sort")

    try:
        G.observe(sess)
    except G.ConsoleError as e:        # the arm is back at the observe pose; only the localization check after it failed
        G.log("observe_unchecked", reason=str(e))
    counts["seconds"] = round(time.time() - started_run)
    G.log("pipeline_end", run=str(run), **counts)
    return counts


def locate_bins(run: Path, design: str, only=None) -> B.BinSet:
    """The bins of `design` (only those numbered in `only`, when given) found in the run's bin views
    (planned places for any no view showed)."""
    found = B.BinSet(design, only)
    if (run / "bin_views.json").exists():
        B.locate_run(run, found, G.TABLE_Z)
    else:
        print(f"{run} 里没有 bin_views.json：桶按设计位置处理，不是找到的位置")
    for b in found.bins.values():
        print(f"bin {b.number} {b.category:10s} {b.source:7s} base ({b.xy[0]*100:6.1f}, {b.xy[1]*100:5.1f}) cm  yaw {b.yaw_deg:6.1f}"
              + (f"  off plan {b.shift_mm():.1f} mm / {b.turn_deg():.1f} deg, {b.views} views" if b.source == "markers" else ""))
    for problem in B.table_check(found, SB.SORTING["bins"]["min_plate_gap_mm"]):
        print("注意：", problem)
    G.log("bins_located", design=design, bins=[b.summary() for b in found.bins.values()])
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scan", type=Path, help="use this scan directory instead of scanning")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--bins", nargs="?", const=B.DEFAULT_DESIGN, metavar="DESIGN",
                        help="sort into the acrylic bins of config/<DESIGN>.json instead of the map codes")
    parser.add_argument("--empty-bins", action="store_true",
                        help="the blocks were taken out of the bins: forget how many each was given, and stop")
    parser.add_argument("--bin-side", default="all", metavar="left|right|all|1,2",
                        help="with --bins: only these bins stand on the table (a half-side start for a small table); "
                             "blocks of the other categories are left alone")
    args = parser.parse_args()
    if args.empty_bins:
        forget_counts(args.bins or B.DEFAULT_DESIGN)
        print("每个桶的计数已清零")
        return
    import field_scan as SC
    if not (args.dry_run and args.scan):
        G.require_empty_gripper()
    with (G.nullcontext(None) if args.dry_run and args.scan else G.Session()) as sess:
        only = B.parse_only(args.bin_side)
        if args.bins:
            use_bins(B.BinSet(args.bins, only))             # the keep-out round the bins, before the arm moves at all
        run = (args.scan.resolve() if args.scan else None) or SC.scan(session=sess)
        if args.bins:
            if not args.scan and SC.has_bin_views(args.bins):
                SC.scan_bins(run, sess, args.bins, only)
            use_bins(locate_bins(run, args.bins, only))
        scene = build_scene(run)
        use_registration(scene["registration"])
        if not args.dry_run:
            scene = refine(run, scene, sess)
            use_registration(scene["registration"])
        jobs = decide(scene)
        for j in jobs:
            c = j["cube"]
            p = None if j["skip"] else plan_for(c)
            grasp = f" pitch {p['pitch']} hover +{p['hover_above_top_cm']:.1f} cm J5 {p['j5']}" if p else ""
            print(f"cube map ({c['map_cm'][0]:6.1f}, {c['map_cm'][1]:5.1f}) yaw {c['yaw_deg']:5.1f}{grasp}  top {c['colour'] or '-':6s} "
                  f"sticker {c['sticker'] and (c['sticker']['label'], c['sticker']['conf'])}  -> {c['category']} "
                  f"({c['source']})  {'SKIP: ' + j['skip'] if j['skip'] else 'bin ' + j['bin']}")
        print("registration:", scene["registration"])
        if args.dry_run:
            return
        execute(run, scene, jobs, sess)


if __name__ == "__main__":
    main()
