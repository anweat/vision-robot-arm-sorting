#!/usr/bin/env python3
"""Where to stand N acrylic bins on a table that is only as wide as an A4 sheet's long edge.

    uv run --with 'opencv-python<5' --with numpy src/tools/layout_bins.py --bins 2 --depth-cm 21
    uv run --with 'opencv-python<5' --with numpy src/tools/layout_bins.py --bins 2 --depth-cm 30 --top 8

The table is a rectangle in map coordinates (x +-14.85 cm, y from 0 at the arm platform's front to `--depth-cm`); the
arm stands at (0, -6.75) in them (the placement of config/map_a4_v3.json). The fixed plate is 94 x 121 mm
(config/bins_acrylic_v1.json), so a bin in the 4-bin plan (centres 20 cm to the side, flange pointing outwards) hangs
off a 29.7 cm table. Here the bins are placed symmetrically about the arm's centre line, with the flange (the sticker
strip) pointing away from the arm, towards it, or radially outwards, and each layout is scored by what is left for the
blocks:

  * the plates (grown by 5 mm) must lie inside the table, 1 cm in front of the arm platform's front face;
  * every drop slot must be reachable at drop height (console_grasp.plan_put), counted;
  * the pick zone is the ring the arm can grasp in (reach 17.5-26 cm from the base) on the table, 2.5 cm clear of every
    plate; its area and the number of 5 cm-apart block places that fit are reported -- that is what the bins take away.

Nothing here moves anything; the numbers are geometry and the upstream link model, not a trial on the table.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import bins as B  # noqa: E402
import console_grasp as G  # noqa: E402

TABLE_HALF_WIDTH_CM = 14.85
ARM_Y_MAP_CM = -6.75
PICK_REACH_CM = (17.5, 26.0)
PICK_CLEAR_CM = 2.5
CENTRE_HALF_WIDTH_CM = 9.0
PLATE_MARGIN_CM = 0.5
FRONT_CLEAR_CM = 1.0       # a plate's near edge stays this far in front of the arm platform's front face (map y = 0)
SPOT_APART_CM = 5.0
FLANGE_CHOICES = {"away": 90.0, "toward": 270.0, "outward": None}      # None: radially outwards from the arm


def plate_polygon(b: B.Bin, bins: B.BinSet, grow_m: float = 0.0) -> np.ndarray:
    g = bins.design["geometry_mm"]
    half = g["plate"][0] / 2000 + grow_m
    y0, y1 = g["plate_y_range"][0] / 1000 - grow_m, g["plate_y_range"][1] / 1000 + grow_m
    return np.array([b.to_base(p) for p in ((-half, y0), (half, y0), (half, y1), (-half, y1))])


def inside_polygon(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Convex polygon (counter-clockwise or clockwise) membership for an (n, 2) array."""
    sign = None
    ok = np.ones(len(points), bool)
    for i in range(len(poly)):
        a, c = poly[i], poly[(i + 1) % len(poly)]
        cross = (c[0] - a[0]) * (points[:, 1] - a[1]) - (c[1] - a[1]) * (points[:, 0] - a[0])
        if sign is None:
            sign = 1 if cross.mean() >= 0 else -1
        ok &= cross * sign >= 0
    return ok


def place(bins: B.BinSet, centres_base_cm, flange_deg) -> None:
    """Put each bin at its centre with its flange pointing at flange_deg (None: radially outwards)."""
    for b, (x, y), phi in zip(bins.bins.values(), centres_base_cm, flange_deg):
        b.xy = np.array([x, y]) / 100
        direction = math.degrees(math.atan2(y, x)) if phi is None else phi
        b.yaw_deg = direction - 90.0


def pick_zone(bins: B.BinSet, depth_cm: float) -> tuple[float, int]:
    """(area in cm2 of the middle of the table the arm can grasp at and no plate is near, how many `SPOT_APART_CM`-apart
    block places fit there, greedily)."""
    xs = np.arange(-TABLE_HALF_WIDTH_CM + 1.5, TABLE_HALF_WIDTH_CM - 1.5 + 1e-9, 0.5)
    ys = np.arange(1.5, depth_cm - 1.5 + 1e-9, 0.5)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    base = pts + np.array([G.DX * 100, G.DY * 100])
    reach = np.hypot(base[:, 0], base[:, 1])
    ok = (reach >= PICK_REACH_CM[0]) & (reach <= PICK_REACH_CM[1])
    for b in bins.bins.values():
        ok &= ~inside_polygon(base / 100, plate_polygon(b, bins, PICK_CLEAR_CM / 100))
    # block places: only in the middle of the table (|x| <= 9 cm), where the arm's views and approach are best
    ok &= np.abs(pts[:, 0]) <= CENTRE_HALF_WIDTH_CM
    spots: list[np.ndarray] = []
    order = np.lexsort((np.abs(pts[:, 0]), -pts[:, 1]))
    for i in order:
        if ok[i] and all(np.linalg.norm(pts[i] - s) >= SPOT_APART_CM for s in spots):
            spots.append(pts[i])
    return float(ok.sum()) * 0.25, len(spots)


def evaluate(bins: B.BinSet, depth_cm: float, slots_cm) -> dict | None:
    """Score a placement, None when a plate leaves the table."""
    for b in bins.bins.values():
        poly = plate_polygon(b, bins, PLATE_MARGIN_CM / 100) * 100 - np.array([G.DX * 100, G.DY * 100])
        if (np.abs(poly[:, 0]) > TABLE_HALF_WIDTH_CM).any() or (poly[:, 1] < FRONT_CLEAR_CM).any() or (poly[:, 1] > depth_cm).any():
            return None
    z = G.bin_drop_tip_z(bins.wall_top_m)
    reach, soft = [], []
    for b in bins.bins.values():
        got = 0
        inside = 0
        for slot in slots_cm:
            x, y = b.to_base(np.array(slot) / 100)
            plan = G.plan_put(150, f"{(x - G.DX) * 100:.2f},{(y - G.DY) * 100:.2f}", list(G.OBSERVE) + [G.ROLL_ZERO], z)
            got += plan is not None
            inside += plan is not None and plan["within_soft_limits"]
        reach.append(got)
        soft.append(inside)
    area, spots = pick_zone(bins, depth_cm)
    gaps = B.table_check(bins, 10.0)
    return dict(reachable_slots=reach, soft_slots=soft, pick_area_cm2=round(area), spots=spots, close_plates=gaps)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bins", type=int, default=2, choices=(2, 4))
    p.add_argument("--numbers", help="which bins of the design (default: 2,3 for two; 1,2,3,4 for four)")
    p.add_argument("--depth-cm", type=float, default=21.0, help="table depth in front of the arm platform (A4 short edge = 21)")
    p.add_argument("--slot-cm", type=float, default=1.2)
    p.add_argument("--top", type=int, default=6)
    args = p.parse_args()
    cfg = G.MAP["placement"]["base_from_map"]
    G.DX, G.DY = cfg["dx"], cfg["dy"]
    numbers = tuple(int(v) for v in args.numbers.split(",")) if args.numbers else ((2, 3) if args.bins == 2 else (1, 2, 3, 4))
    bins = B.BinSet(only=numbers)
    v = args.slot_cm
    slots = [[-v, -v], [v, v], [-v, v], [v, -v]]
    results = []
    for flange_name, phi in FLANGE_CHOICES.items():
        for x0 in np.arange(5.0, 12.01, 0.5):                       # |x| of the bin centres, base cm
            for y in np.arange(12.0, 30.01, 0.5):                   # base y of the centres
                if args.bins != 2:
                    continue                                        # four bins do not fit this table at all
                centres, flange = [(x0, y), (-x0, y)], [phi, phi]    # away / toward are mirror-symmetric as they are
                place(bins, centres, flange)
                r = evaluate(bins, args.depth_cm, slots)
                if r is None or min(r["reachable_slots"]) < len(slots) or r["close_plates"]:
                    continue
                results.append((r["spots"], r["pick_area_cm2"], min(r["soft_slots"]), flange_name, x0, y, r))
    results.sort(key=lambda t: (-t[0], -t[1], -t[2]))
    place(bins, [(30, 60)] * len(bins.bins), [90.0] * len(bins.bins))      # bins far off the table: the zone with none
    print(f"for reference, with no bins: {pick_zone(bins, args.depth_cm)[1]} block places 5 cm apart, pick area "
          f"{pick_zone(bins, args.depth_cm)[0]:.0f} cm2")
    print(f"table {2 * TABLE_HALF_WIDTH_CM:.1f} x {args.depth_cm:g} cm, {args.bins} bins, drop slots +-{v} cm: "
          f"{len(results)} placements keep every plate on the table and every slot reachable")
    seen = set()
    shown = 0
    for spots, area, soft, flange_name, x0, y, r in results:
        key = (flange_name, round(x0), round(y))
        if key in seen:
            continue
        seen.add(key)
        base_x, base_y = x0, y
        print(f"  flange {flange_name:8s} centres (+-{base_x:.1f}, {base_y:.1f}) base cm = map y {base_y - G.DY * 100:.1f}: "
              f"{spots} block places 5 cm apart, pick area {area:.0f} cm2, slots reachable {r['reachable_slots']} "
              f"(inside soft limits {r['soft_slots']})")
        shown += 1
        if shown >= args.top:
            break
    if not results:
        print("  none: widen the table, or make the plates smaller, or put one bin at a time")


if __name__ == "__main__":
    main()
