# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy", "pillow"]
# ///
"""Generate the printable A4 table map with an ArUco grid, plus its layout JSON.

  uv run src/tools/make_map.py --name dofbot_map_a4_v2
  uv run src/tools/make_map.py --name dofbot_map_a4_v4 --bins config/bins_acrylic_v2_pair.json \
      --placement-from config/map_a4_v2.json --extra="-2.2,2;2.2,2;..."      # two bins, more codes where they are free
  uv run src/tools/make_map.py --name dofbot_map_a4_v3 --bins config/bins_acrylic_v1.json \
      --placement-from config/map_a4_v2.json

With --bins, codes that a planned bin plate would cover are left out (the others keep the
ids and positions of the full grid, so a v2 calibration still applies) and the plate
outlines are printed as seating marks.

Map frame: origin at the centre of the page's near edge (the edge pushed against the
arm's platform), X to the arm's right, Y away from the arm, metres in the JSON.
A self-check renders the page at roughly camera resolution and fits the detected
corners back onto the layout.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
DPI = 300
MM = DPI / 25.4
W_CM, H_CM = 29.7, 21.0
FONT = "/System/Library/Fonts/STHeiti Medium.ttc"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", default="dofbot_map_a4_v2")
    parser.add_argument("--cols", type=float, nargs="+", default=[-12, -6, 0, 6, 12], help="code centre X (cm)")
    parser.add_argument("--rows", type=float, nargs="+", default=[3, 8, 13, 18], help="code centre Y (cm)")
    parser.add_argument("--side-cm", type=float, default=1.5)
    parser.add_argument("--first-id", type=int, default=30)
    parser.add_argument("--base-offset-cm", type=float, default=8.0, help="servo-1 axis to platform front face")
    parser.add_argument("--mat-z-cm", type=float, default=-9.0, help="map plane height in the base frame")
    parser.add_argument("--extra", help='more codes where the grid has room: "x,y;x,y" in map cm (use --extra="-2,3;2,3" '
                                        'for a negative x). They take ids from --extra-first-id up, the grid keeps its ids')
    parser.add_argument("--extra-first-id", type=int, default=0)
    parser.add_argument("--bins", type=Path, help="bin design JSON from make_bins.py; needs --placement-from")
    parser.add_argument("--placement-from", type=Path, help="map JSON whose base_from_map placement is carried over")
    args = parser.parse_args()
    placement = json.loads(args.placement_from.read_text())["placement"] if args.placement_from else None
    plates = {}  # bin number -> (name, plate corners, opening corners) in map cm
    if args.bins:
        if not placement:
            raise SystemExit("--bins needs --placement-from to turn base coordinates into map coordinates")
        design, off = json.loads(args.bins.read_text()), placement["base_from_map"]
        geo = design["geometry_mm"]
        w, (y0, y1), inner = geo["plate"][0] / 20, [v / 10 for v in geo["plate_y_range"]], geo["inner"] / 20
        for number, entry in design["bins"].items():
            cx, cy = [v * 100 for v in entry["planned_centre_base_m"]]
            th = math.radians(entry["planned_flange_direction_deg"] - 90)

            def on_map(pts):
                return [(cx + x * math.cos(th) - y * math.sin(th) - off["dx"] * 100,
                         cy + x * math.sin(th) + y * math.cos(th) - off["dy"] * 100) for x, y in pts]
            plates[number] = (entry["name"], on_map([(-w, y0), (w, y0), (w, y1), (-w, y1)]),
                              on_map([(-inner, -inner), (inner, -inner), (inner, inner), (-inner, inner)]))

    count = len(args.cols) * len(args.rows)
    if args.first_id + count > 50:
        raise SystemExit("DICT_4X4_50 has ids 0-49 only")
    extra = [tuple(float(v) for v in item.split(",")) for item in args.extra.split(";")] if args.extra else []
    extra_ids = list(range(args.extra_first_id, args.extra_first_id + len(extra)))
    if set(extra_ids) & set(range(args.first_id, args.first_id + count)) or (extra_ids and extra_ids[-1] > 49):
        raise SystemExit("--extra ids collide with the grid's or run past 49")
    for x, y in extra:
        if abs(x) > W_CM / 2 - 1.0 or not 1.0 <= y <= H_CM - 1.0:
            raise SystemExit(f"extra code ({x}, {y}) is off the page or within 1 cm of its edge")
        if any(cv2.pointPolygonTest(np.array(plate, np.float32), (x, y), True) > -(args.side_cm / 2 + 0.3)
               for _, plate, _ in plates.values()):
            raise SystemExit(f"extra code ({x}, {y}) lies under, or within 3 mm of, a bin plate")

    def px(x_cm: float, y_cm: float) -> tuple[int, int]:
        return round((x_cm + W_CM / 2) * 10 * MM), round((H_CM - y_cm) * 10 * MM)

    page = Image.new("L", (round(W_CM * 10 * MM), round(H_CM * 10 * MM)), 255)
    draw = ImageDraw.Draw(page)
    font, small = ImageFont.truetype(FONT, 34), ImageFont.truetype(FONT, 26)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side_px = round(args.side_cm * 10 * MM)
    layout, mid = {}, args.first_id
    skipped = []
    for y in args.rows:
        for x in args.cols:
            # a code is dropped when a plate edge comes within 3 mm of its quiet zone
            if any(cv2.pointPolygonTest(np.array(plate, np.float32), (x, y), True) > -(args.side_cm / 2 + 0.3)
                   for _, plate, _ in plates.values()):
                skipped.append(mid)
                mid += 1
                continue
            marker = cv2.aruco.generateImageMarker(dictionary, mid, side_px, borderBits=1)
            cx, cy = px(x, y)
            page.paste(Image.fromarray(marker), (cx - side_px // 2, cy - side_px // 2))
            draw.text((cx + side_px // 2 + 6, cy + side_px // 2 - 24), str(mid), fill=0, font=small)
            h = args.side_cm / 2
            layout[mid] = [[(x - h) / 100, (y + h) / 100], [(x + h) / 100, (y + h) / 100],
                           [(x + h) / 100, (y - h) / 100], [(x - h) / 100, (y - h) / 100]]  # TL TR BR BL
            mid += 1

    for mid, (x, y) in zip(extra_ids, extra):
        marker = cv2.aruco.generateImageMarker(dictionary, mid, side_px, borderBits=1)
        cx, cy = px(x, y)
        page.paste(Image.fromarray(marker), (cx - side_px // 2, cy - side_px // 2))
        draw.text((cx + side_px // 2 + 6, cy + side_px // 2 - 24), str(mid), fill=0, font=small)
        h = args.side_cm / 2
        layout[mid] = [[(x - h) / 100, (y + h) / 100], [(x + h) / 100, (y + h) / 100],
                       [(x + h) / 100, (y - h) / 100], [(x - h) / 100, (y - h) / 100]]

    for number, (name, plate, opening) in plates.items():
        for pts, width in ((plate, 4), (opening, 2)):
            draw.line([px(*q) for q in pts + pts[:1]], fill=110, width=width)
        cx, cy = np.mean(opening, axis=0)
        if 0 < cx + W_CM / 2 < W_CM and 0 < cy < H_CM:
            draw.text(px(cx, cy), f"{number} 号桶\n{name}", fill=110, font=small, anchor="mm", align="center")

    x0 = px(0, 0)[0]
    for yy in np.arange(0.3, H_CM - 0.3, 0.6):  # dashed centre line, broken around markers on X = 0
        if 0 in args.cols and any(abs(yy + 0.15 - r) < args.side_cm / 2 + 0.4 for r in args.rows):
            continue
        draw.line([x0, px(0, yy)[1], x0, px(0, yy + 0.3)[1]], fill=150, width=3)
    draw.text(px(-6.5, 1.1), "↓ 此边贴紧机械臂底座平台前沿 · 虚线对准机械臂中心", fill=0, font=font)
    for xc in range(-14, 15):
        a = px(xc, 0)
        draw.line([a[0], a[1] - (30 if xc % 5 == 0 else 15), a[0], a[1]], fill=0, width=2)
    for yc in range(1, 21):
        a = px(-W_CM / 2, yc)
        draw.line([a[0], a[1], a[0] + (30 if yc % 5 == 0 else 15), a[1]], fill=0, width=2)
        if yc % 5 == 0:
            draw.text((a[0] + 36, a[1] - 14), str(yc), fill=0, font=small)
    draw.text(px(-14.2, 20.6), f"Dofbot 桌面地图 {args.name} · A4 横向 · 必须按 100%/实际尺寸打印 · "
              f"DICT_4X4_50 id {args.first_id}-{mid - 1}"
              + (f" + {extra_ids[0]}-{extra_ids[-1]}" if extra_ids else "") + f", {args.side_cm * 10:g} mm"
              + (f" · 桶位下不印码：{skipped}" if skipped else ""), fill=0, font=small)
    bar_y = 20.1
    for t in range(11):
        xx = px(3.0 + t, bar_y)[0]
        draw.line([xx, px(0, bar_y)[1] - 18, xx, px(0, bar_y)[1]], fill=0, width=3)
    draw.line([px(3.0, bar_y), px(13.0, bar_y)], fill=0, width=3)
    draw.text((px(13.3, bar_y)[0], px(0, bar_y)[1] - 22), "100 mm", fill=0, font=small)

    pdf = ROOT / "docs" / "calibration" / f"{args.name}.pdf"
    page.save(pdf, resolution=DPI)
    cfg = {
        "_status": f"design layout of docs/calibration/{args.name}.pdf; exact only if printed at 100%",
        "dictionary": "DICT_4X4_50", "side_m": args.side_cm / 100, "ids": list(layout),
        "map_frame": "origin at the centre of the page's near edge; X right (arm's right), Y away from the arm, metres",
        "corner_order": "ArUco order TL, TR, BR, BL with the marker top pointing to +Y",
        "layout_m": layout,
        "extra_ids": extra_ids,
        "placement": {
            "rule": "near edge flush against the front face of the arm's platform; dashed centre line under the arm's centre",
            "base_from_map": {"dx": 0.0, "dy": args.base_offset_cm / 100, "dz": args.mat_z_cm / 100,
                              "meaning": "base = (X + dx, Y + dy, Z + dz); the map plane is Z = 0"},
            "source": "user tape measurements 2026-09-24, about +-1 cm; verify with a touch test on site",
        },
    }
    if placement:
        cfg["placement"] = dict(placement, source=f"carried over from {args.placement_from}: " + placement["source"])
    if plates:
        cfg["bins"] = {"design": str(args.bins), "omitted_ids": skipped,
                       "plate_outline_map_cm": {n: [[round(v, 2) for v in q] for q in pl] for n, (_, pl, _) in plates.items()},
                       "_note": "outlines are the planned bin seats, not measured positions"}
    out = ROOT / "config" / f"{args.name.replace('dofbot_', '')}.json"
    out.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")

    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    small_img = cv2.resize(np.array(page), None, fx=0.3, fy=0.3, interpolation=cv2.INTER_AREA)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(small_img)
    found = sorted(int(v) for v in ids.ravel())
    src = np.vstack([c.reshape(4, 2) for c in corners])
    dst = np.vstack([np.array(layout[int(v)]) for v in ids.ravel()])
    h_fit, _ = cv2.findHomography(src, dst)
    err = np.linalg.norm(cv2.perspectiveTransform(src.reshape(-1, 1, 2), h_fit).reshape(-1, 2) - dst, axis=1)
    print(f"{pdf.relative_to(ROOT)}  {out.relative_to(ROOT)}")
    print(f"self-check: found {len(found)}/{len(layout)} ids, layout residual max {err.max() * 1000:.3f} mm")


if __name__ == "__main__":
    main()
