"""Fit the map placement and the table height from fingertip touches on marker centres.

The operator closes the gripper, touches the centre of map marker N with the fingertip and saves a
sample whose label starts with the marker id (e.g. "47刚触"). Forward kinematics of each touch gives
the tip in base coordinates; the markers give the same points in map coordinates. Least squares
over a translation (the map is laid square to the arm; yaw is reported, not applied) gives
`base_from_map`, and the mean tip height is the table. `--write` stores both in config.

    python3 dev/spatial/touch_calibration.py data/teleop/20260925_005756_78eb [--write]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "app" / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402
import workspace  # noqa: E402

MAP_CONFIG = ROOT / "app" / "config" / "map_a4_v2.json"
ARM_CONFIG = ROOT / "app" / "config" / "arm_limits.json"
TOUCH = re.compile(r"^\s*(\d{2})\s*.*触")


def touches(sessions: list[Path]) -> list[dict]:
    out, seen = [], set()
    for session in sessions:
        for path in sorted(session.glob("sample_*.json")):
            s = json.loads(path.read_text())
            m = TOUCH.match(s.get("label") or "") or TOUCH.match(s.get("note") or "")
            pose = (s.get("state") or {}).get("pose")
            if not m or not pose or any(v is None for v in pose[:4]):
                continue
            key = (int(m.group(1)), tuple(pose[:4]))
            if key in seen:  # the same touch saved twice
                continue
            seen.add(key)
            out.append(dict(marker=int(m.group(1)), pose=pose, sample=f"{session.name}/{path.name}"))
    return out


def fit(points: list[dict], layout: dict) -> dict:
    """Translation-only map placement, table height, and the residuals."""
    rows = []
    for p in points:
        corners = layout[str(p["marker"])] if str(p["marker"]) in layout else layout[p["marker"]]
        cx = sum(c[0] for c in corners) / 4
        cy = sum(c[1] for c in corners) / 4
        tip = kin.forward(p["pose"][:4])["tip"]
        hull = min(z for _, z in workspace.gripper_hull(p["pose"], workspace.DEFAULTS["hull_half_width_m"]))
        rows.append(dict(p, map_xy=(cx, cy), tip=tip, hull_min=hull))
    n = len(rows)
    dx = sum(r["tip"][0] - r["map_xy"][0] for r in rows) / n
    dy = sum(r["tip"][1] - r["map_xy"][1] for r in rows) / n
    z = sum(r["tip"][2] for r in rows) / n
    # yaw for information only
    mx, my = sum(r["map_xy"][0] for r in rows) / n, sum(r["map_xy"][1] for r in rows) / n
    tx, ty = sum(r["tip"][0] for r in rows) / n, sum(r["tip"][1] for r in rows) / n
    num = sum((r["map_xy"][0] - mx) * (r["tip"][1] - ty) - (r["map_xy"][1] - my) * (r["tip"][0] - tx) for r in rows)
    den = sum((r["map_xy"][0] - mx) * (r["tip"][0] - tx) + (r["map_xy"][1] - my) * (r["tip"][1] - ty) for r in rows)
    for r in rows:
        r["residual_cm"] = [round((r["tip"][0] - r["map_xy"][0] - dx) * 100, 2),
                            round((r["tip"][1] - r["map_xy"][1] - dy) * 100, 2), round((r["tip"][2] - z) * 100, 2)]
    rms = math.sqrt(sum(r["residual_cm"][0] ** 2 + r["residual_cm"][1] ** 2 for r in rows) / n)
    z_sd = math.sqrt(sum((r["tip"][2] - z) ** 2 for r in rows) / n)
    return dict(points=rows, dx=dx, dy=dy, table_z=z, table_z_sd=z_sd, xy_rms_cm=rms,
                yaw_deg=math.degrees(math.atan2(num, den)) if n > 1 else 0.0,
                hull_min=min(r["hull_min"] for r in rows))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--write", action="store_true", help="store base_from_map and the table height in config")
    args = parser.parse_args()
    points = touches(args.sessions)
    if len(points) < 3:
        raise SystemExit(f"need at least 3 marker touches, found {len(points)}")
    layout = json.loads(MAP_CONFIG.read_text())["layout_m"]
    result = fit(points, layout)
    for r in result["points"]:
        print(f"marker {r['marker']}: pose {r['pose'][:4]} tip ({r['tip'][0]*100:.1f}, {r['tip'][1]*100:.1f}, "
              f"{r['tip'][2]*100:.1f}) cm  residual {r['residual_cm']} cm")
    print(f"map origin in base ({result['dx']*100:.2f}, {result['dy']*100:.2f}) cm, yaw {result['yaw_deg']:.1f}° (not applied), "
          f"xy rms {result['xy_rms_cm']:.2f} cm")
    print(f"table z {result['table_z']*100:.2f} ± {result['table_z_sd']*100:.2f} cm; lowest gripper-hull point at a touch "
          f"{result['hull_min']*100:.2f} cm")
    if not args.write:
        return
    source = ", ".join(sorted({p["sample"].split("/")[0] for p in points}))
    m = json.loads(MAP_CONFIG.read_text())
    m["placement"]["base_from_map"].update(dx=round(result["dx"], 4), dy=round(result["dy"], 4), dz=round(result["table_z"], 4))
    m["placement"]["source"] = (f"touch calibration ({len(points)} marker touches, xy rms {result['xy_rms_cm']:.2f} cm, "
                                f"yaw {result['yaw_deg']:.1f} deg not applied): {source}")
    MAP_CONFIG.write_text(json.dumps(m, indent=2, ensure_ascii=False) + "\n")
    a = json.loads(ARM_CONFIG.read_text())
    a["keep_out"]["table_surface_z_m"] = round(result["table_z"], 4)
    a["keep_out"]["table_z_m"] = math.floor(result["hull_min"] * 1000) / 1000
    a["keep_out"]["_table_note"] = (f"touch calibration {source}: fingertip touches read table z {result['table_z']*100:.2f} "
                                    f"± {result['table_z_sd']*100:.2f} cm in FK coordinates (the arm's base_link sits recessed in "
                                    f"the 9 cm chassis). table_z_m is the gripper-hull model value at those touches.")
    ARM_CONFIG.write_text(json.dumps(a, indent=2, ensure_ascii=False) + "\n")
    print("written:", MAP_CONFIG.relative_to(ROOT), ARM_CONFIG.relative_to(ROOT))


if __name__ == "__main__":
    main()
