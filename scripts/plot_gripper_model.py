#!/usr/bin/env python3
"""Draw the gripper model: jaw curve and a cross-section at an open pose.

Writes docs/images/gripper_model.png.  Data comes from config/gripper_model.json so the figure
cannot drift from the model.  Run:

    UV_CACHE_DIR=$PWD/.uv-cache uv run --with matplotlib python scripts/plot_gripper_model.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))
import gripper_model as gm  # noqa: E402

OUT = ROOT / "docs" / "images" / "gripper_model.png"
MODEL = json.loads((ROOT / "config" / "gripper_model.json").read_text(encoding="utf-8"))


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.2), gridspec_kw={"width_ratios": [1.15, 1.0]})

    # --- jaw curve -----------------------------------------------------------------------
    official = MODEL["jaw"]["points"]
    ax1.plot([a for a, _ in official], [g for _, g in official], "o-", ms=3.5, lw=1.4,
             color="#1f5fa8", label="official table (DOFBOT-Pi/Pro)")
    angles = [a for a in range(0, 181)]
    ax1.plot(angles, [gm.jaw_gap_m(a) * 1000 for a in angles], lw=0.8, color="#9ec3e6")
    ax1.axhline(30, color="#c0392b", lw=1.0, ls="--")
    ax1.text(6, 31.5, "3 cm block", color="#c0392b", fontsize=8)
    ax1.axvspan(11, 30, color="#2e8b57", alpha=0.10)
    ax1.text(20.5, 8, "poses used\n(11–30°)", ha="center", fontsize=7.5, color="#2e8b57")
    ax1.annotate("local check: grips at 133°\ntable says 134°", xy=(133, 30), xytext=(95, 52),
                 fontsize=7.5, arrowprops=dict(arrowstyle="->", lw=0.8))
    ax1.set_xlabel("servo-6 command angle (deg, larger = more closed)")
    ax1.set_ylabel("jaw gap between pad faces (mm)")
    ax1.set_xlim(0, 180)
    ax1.set_ylim(0, 70)
    ax1.grid(alpha=0.25)
    ax1.legend(fontsize=7.5, loc="upper right")
    ax1.set_title("J6 angle → jaw opening", fontsize=10)

    # --- cross-section at 30 deg ---------------------------------------------------------
    gap = gm.jaw_gap_m(30.0)
    pad = MODEL["geometry"]["pad"]
    for box in gm.pad_boxes(30.0):
        x0, x1 = box["x_m"]
        ax2.add_patch(Rectangle((x0 * 100, box["z_m"][0] * 100), (x1 - x0) * 100,
                                (box["z_m"][1] - box["z_m"][0]) * 100,
                                facecolor="#444444", edgecolor="black", lw=0.6))
    hw = MODEL["envelope"]["housing_half_width_m"] * 100
    hz = MODEL["envelope"]["housing_z_range_m"]
    ax2.add_patch(Rectangle((-hw, hz[0] * 100), 2 * hw, (hz[1] - hz[0]) * 100,
                            facecolor="#c9ced6", edgecolor="#7a8290", lw=0.8, zorder=0))
    ax2.axvline(0, color="#999999", lw=0.8, ls=":")
    ax2.text(-46, 6, "J5 axis", fontsize=7, color="#666666")

    # three TCP candidates
    y = 0.0
    for name, z, colour, dx, dygot in (("fingertip 117 mm", 0.11699, "#1f5fa8", 6, 4),
                                       ("FK tip 120 mm", 0.12, "#c0392b", 6, -8),
                                       ("grasp centre 106.6 mm", 0.1066, "#2e8b57", 6, -4)):
        ax2.plot([0], [z * 100], marker="v", ms=7, color=colour)
        ax2.annotate(name, xy=(0, z * 100), xytext=(dx, z * 100 + dygot), fontsize=7.5, color=colour)

    ax2.annotate("", xy=(-gap / 2 * 100, 92), xytext=(gap / 2 * 100, 92),
                 arrowprops=dict(arrowstyle="<->", lw=0.9))
    ax2.text(0, 87, f"gap at 30° = {gap * 1000:.1f} mm", ha="center", fontsize=8)
    ax2.set_xlim(-52, 52)
    ax2.set_ylim(-2, 122)
    ax2.set_xlabel("jaw axis x (mm)")
    ax2.set_ylabel("along link z from the J5 axis (mm)")
    ax2.set_title("gripper cross-section (pads parallel, Isaac four-bar)", fontsize=10)
    ax2.grid(alpha=0.2)

    fig.suptitle("DOFBOT gripper model — config/gripper_model.json (2026-11-06)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT, dpi=150)
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
