# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy", "pillow"]
# ///
"""Generate the slotted acrylic bin design: fabrication drawings, cut DXF, marker stickers and layout JSON.

  uv run src/tools/make_bins.py --thickness-mm 2.0

One bin is a slotted base plate plus four identical side panels: each panel has a 14 x t
tab at both ends that plugs through a 14 x t slot in the base, so the walls locate
themselves before gluing and no engraving is needed. The walls still butt in a pinwheel.
Measure the sheet with calipers before ordering: wall length = inner + thickness,
panel height = wall height + thickness, slot width = thickness.

Bin frame: origin at the centre of the opening on the plate's top face, +Y towards the
marker flange, X to the right when looking along +Y, millimetres in the drawings and
metres in the JSON.

Everything written here is a design value. Nothing has been cut or measured yet; the
placement page only checks the planned release points against the upstream link model
(src/spatial/dofbot_kinematics.py) and the soft limits in config/arm_limits.json.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import date
from pathlib import Path

import cv2
import numpy as np

import bins_core as core
from bins_core import Design, bin_pose, placements

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "spatial"))
import dofbot_kinematics as kin  # noqa: E402


def release_check(d: Design, pose, arm: dict, clearance_mm: float) -> dict:
    """Release point above the bin centre in the upstream link model, soft limits, pitch 170-130."""
    table = arm["keep_out"]["table_surface_z_m"]
    grasp = arm["gripper"]["grasp_height_cm"] / 100
    z = table + (d.t + d.wall_h + clearance_mm) / 1000 + grasp
    limits = {int(k): v for k, v in arm["soft_limits_deg"].items()}
    found = kin.inverse_within(pose[0] / 1000, pose[1] / 1000, z, limits, tuple(range(170, 129, -5)))
    return {"tip_z_base_cm": round(z * 100, 2), "solution": found}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default="bins_acrylic_v1")
    p.add_argument("--thickness-mm", type=float, default=2.0, help="measured sheet thickness")
    p.add_argument("--inner-mm", type=float, default=80.0, help="square opening")
    p.add_argument("--wall-mm", type=float, default=40.0, help="wall height above the floor")
    p.add_argument("--margin-mm", type=float, default=5.0, help="plate rim outside the walls on three sides")
    p.add_argument("--flange-mm", type=float, default=32.0, help="plate rim on the marker side")
    p.add_argument("--marker-mm", type=float, default=20.0)
    p.add_argument("--marker-pitch-mm", type=float, default=50.0, help="distance between the two flange markers")
    p.add_argument("--tab-mm", type=float, default=14.0, help="length of the two bottom tabs on each side panel")
    p.add_argument("--slot-mm", type=float, default=None,
                   help="slot width only (default = measured thickness); use for fit, not for sheet thickness")
    p.add_argument("--bins", type=int, default=4)
    p.add_argument("--sheet-mm", type=float, nargs=2, default=[297.0, 388.0],
                   help="stock sheet for nesting; default = the tight parts envelope (300x400 standard leaves 3/12 mm margin)")
    p.add_argument("--place", nargs="+", default=["22,24", "22,55", "22,125", "22,156"],
                   help="planned bin centres: r_cm,theta_deg (theta = servo-1 angle, flange points outwards) or "
                        "x_cm,y_cm,flange_deg (base frame; flange_deg is the direction the sticker strip points, "
                        "90 = away from the arm)")
    p.add_argument("--numbers", help="bin numbers (1 recyclable, 2 kitchen, 3 hazardous, 4 other) the --place entries are "
                                     "for, in order, e.g. 2,3: the sticker categories and marker ids follow the number")
    p.add_argument("--stickers-only", action="store_true",
                   help="write only the sticker sheet and the layout JSON: the plates are already cut "
                        "(drawings and DXF of the full set stay as they are)")
    p.add_argument("--parts-set", default="bins_acrylic_v1_drawings.pdf", help="with --stickers-only: the drawing set the plates were cut from")
    p.add_argument("--release-clearance-mm", type=float, default=10.0, help="block bottom above the wall top at release")
    p.add_argument("--drawing-date", default=date.today().isoformat())
    p.add_argument("--revision", default="A")
    p.add_argument("--preview-dir", type=Path, help="also write each page as a PNG here")
    a = p.parse_args()
    numbers = [int(v) for v in a.numbers.split(",")] if a.numbers else list(range(1, len(a.place) + 1))
    if len(numbers) != len(a.place) or len(set(numbers)) != len(numbers) or not set(numbers) <= {1, 2, 3, 4}:
        p.error("--numbers must name each --place entry once, from 1-4")
    a.bins = len(a.place)
    import bins_fab
    bins_fab.NUMBERS = numbers
    d = Design(a)

    arm = json.loads((ROOT / "config" / "arm_limits.json").read_text())
    map_cfg = json.loads((ROOT / "config" / "map_a4_v2.json").read_text())
    poses = [bin_pose(*rt) if len(rt) == 2 else (rt[0] * 10, rt[1] * 10, rt[2]) for rt in placements(a.place)]
    checks = [release_check(d, pose, arm, a.release_clearance_mm) for pose in poses]

    out = ROOT / "docs" / "bins"
    out.mkdir(exist_ok=True)

    from bins_fab import page_stickers, write_outputs  # local import: bins_fab only needs the geometry above
    if a.stickers_only:
        stickers, bins = page_stickers(d, a.bins)
        stickers.save(out / f"{a.name}_stickers_A4.pdf", resolution=600)
        report = [f"stickers only: {a.name}_stickers_A4.pdf for bins {numbers}; plates as cut from {a.parts_set}"]
    else:
        bins, report = write_outputs(d, poses, checks, arm, map_cfg, a, out)

    h = d.side / 2
    for number, entry in bins.items():
        entry["corners_m"] = {
            str(entry["ids"][k]): [[round(v / 1000, 5) for v in pt] for pt in
                                   ((x - h, y + h), (x + h, y + h), (x + h, y - h), (x - h, y - h))]
            for k, (x, y) in d.marker_centres().items()}
        pose, chk = poses[numbers.index(int(number))], checks[numbers.index(int(number))]
        entry["planned_centre_base_m"] = [round(pose[0] / 1000, 4), round(pose[1] / 1000, 4)]
        entry["planned_flange_direction_deg"] = pose[2]
        entry["planned_release"] = chk

    cfg = {
        "_status": f"design values from src/tools/make_bins.py for docs/bins/{a.name}_*; slot-and-tab acrylic box, "
                   "not cut, not measured; placements and release solutions are plans from the upstream link model only",
        "drawing": {"number": "BIN-ACR-01", "revision": a.revision, "date": a.drawing_date,
                    "set": a.parts_set if a.stickers_only else f"{a.name}_drawings.pdf"},
        "dictionary": core.DICTIONARY, "side_m": d.side / 1000,
        "bin_frame": "origin at the centre of the opening on the plate's top face; +Y towards the marker flange, metres",
        "corner_order": "ArUco order TL, TR, BR, BL with the marker top pointing to +Y",
        "_corner_order_note": "defined in the bin frame (x right, +Y towards the flange, y up); OpenCV image coordinates have y down, so flip y before comparing",
        "marker_plane_above_table_m": round(d.t / 1000 + 0.0001, 5),
        "_marker_plane_note": "plate t plus printing paper (~0.1 mm); measured from the table the plate stands on",
        "geometry_mm": {"thickness": d.t, "inner": d.inner, "outer": d.outer, "wall_height": d.wall_h,
                        "wall_length": d.wall_len, "panel_height": d.panel_h,
                        "plate": [d.base_w, d.base_d], "plate_y_range": [d.y0, d.y1],
                        "rim_top_above_table": d.t + d.wall_h,
                        "tab_length": d.tab_len, "tab_height": d.t, "notch_length": d.notch_len,
                        "slot_width": d.slot_w, "slots_per_wall": 2,
                        "base_slots": len(d.base_slots()), "base_slot_holes": len(d.base_slot_holes())},
        "bins": bins,
    }
    (ROOT / "config" / f"{a.name}.json").write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")

    for line in report:
        print(line)
    polys = [np.array(core.to_base(pose, d.plate()), np.float32) for pose in poses]
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            if cv2.intersectConvexConvex(polys[i], polys[j])[0] > 1.0:
                print(f"WARNING: plates of bins {numbers[i]} and {numbers[j]} overlap")
    for i, (pose, chk) in enumerate(zip(poses, checks)):
        print(f"bin {numbers[i]}: centre ({pose[0] / 10:.1f}, {pose[1] / 10:.1f}) cm, "
              f"release tip z {chk['tip_z_base_cm']} cm -> {chk['solution']}")

    if a.preview_dir:
        a.preview_dir.mkdir(parents=True, exist_ok=True)


if __name__ == "__main__":
    main()
