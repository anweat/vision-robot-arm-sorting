"""Sort the blocks onto their garbage-category bins (config/sorting.json) through the running console.

The category of each block comes from the official 3-class model when it is confident and agrees with the
block colour; otherwise from the colour (a fact of this block set, docs/atlas-200i-dk-a2/07), and every
decision records which source it used, so the summary shows how much the official model contributed.

For each colour: skip it if it already sits on its bin, report it if it is missing or beyond the grasp
zone, refuse if another block occupies the bin, else locate -> hover -> grasp -> put (console_grasp), then
re-observe and record where it landed. One console lease covers the whole run. A block that cannot be
localized well is skipped; once a grasp has started, any refusal, missed grip or resistance stops the run
with the arm where it is (the gripper opened if the grip failed).

    python3 src/tasks/sort_blocks.py --dry-run         # what would be moved, nothing sent
    python3 src/tasks/sort_blocks.py                   # sort everything
    python3 src/tasks/sort_blocks.py --only red green  # a subset, in this order
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))

import console_grasp as G  # noqa: E402

SORTING = json.loads((ROOT / "config" / "sorting.json").read_text())


def category_of(colour: str, sticker: dict | None) -> tuple[str, str]:
    """(category, source) for a block from its colour and the official model's detection on it."""
    by_colour = SORTING["colour_category"][colour]
    if sticker and sticker.get("conf", 0) >= SORTING["model_min_conf"]:
        if sticker["category"] == by_colour:
            return by_colour, f"official model ({sticker['label']} {sticker['conf']:.2f}), colour agrees"
        return by_colour, f"colour; official model disagreed ({sticker['label']} {sticker['conf']:.2f} -> {sticker['category']})"
    return by_colour, "colour fallback (official model silent)"


def bin_for(colour: str, sticker: dict | None = None) -> tuple[str, str, str]:
    category, source = category_of(colour, sticker)
    return SORTING["category_bins"][category], category, source


def decide(colour: str, found: dict[str, dict]) -> tuple[str, str]:
    """('move' | 'skip', reason) for one colour given the current estimates (map cm)."""
    if colour not in found:
        return "skip", "not seen"
    bin_id, category, _ = bin_for(colour, found[colour].get("sticker"))
    bins = {colour: bin_id}
    here = found[colour]["map_cm"]
    target = [v * 100 for v in G.marker_centre(bins[colour])]
    if math.dist(here, target) <= SORTING["in_place_cm"]:
        return "skip", f"already on bin {bins[colour]} ({math.dist(here, target):.1f} cm)"
    bx, by = G.map_to_base(here[0] / 100, here[1] / 100)
    if math.hypot(bx, by) > G.MAX_REACH:
        return "skip", f"{math.hypot(bx, by)*100:.1f} cm from the base (> {G.MAX_REACH*100:.0f}); move it closer"
    if math.hypot(bx, by) < G.MIN_REACH:
        return "skip", (f"{math.hypot(bx, by)*100:.1f} cm from the base (< {G.MIN_REACH*100:.1f}); "
                        f"move it to map y {G.MIN_REACH*100 - G.DY*100:.0f}-{G.MAX_REACH*100 - G.DY*100:.0f} cm")
    for other, o in found.items():
        if other != colour and math.dist(o["map_cm"], target) <= SORTING["occupied_cm"]:
            return "skip", f"bin {bins[colour]} is occupied by {other}"
    return "move", f"({here[0]:.1f}, {here[1]:.1f}) -> bin {bins[colour]} {category} ({target[0]:.1f}, {target[1]:.1f})"


def resolve(colour: str | None, sticker: dict | None) -> tuple[str | None, str]:
    """(category, source) for one block from the evidence of a close look. `colour` is only a coloured top
    face seen by the phone (board colour labels follow stickers, not blocks, and are not used)."""
    model = sticker if sticker and sticker.get("conf", 0) >= SORTING["model_min_conf"] else None
    by_colour = SORTING["colour_category"].get(colour) if colour else None
    if model and (by_colour is None or model["category"] == by_colour):
        agree = ", colour agrees" if by_colour else ""
        return model["category"], f"official model ({model['label']} {model['conf']:.2f}){agree}"
    if model and by_colour:
        return by_colour, f"colour; official model disagreed ({model['label']} {model['conf']:.2f} -> {model['category']})"
    if by_colour:
        return by_colour, "colour fallback (official model silent)"
    return None, "no evidence: sticker not one of the model's 3 kinds and no coloured face on top"


def candidates(s: dict) -> list[dict]:
    """Blocks seen now, merged by position: the phone (coloured tops, whole map) and the board (any block,
    class-agnostic, but only where it looks)."""
    found = []
    for source in ("phone", "board"):
        for o in (s["localization"].get(source) or {}).get("objects", []):
            same = [c for c in found if math.dist(c["map_cm"], o["map_cm"]) < 2.5]
            if same:
                continue
            found.append(dict(map_cm=o["map_cm"], colour=o["label"] if source == "phone" else None, seen_by=source))
    return found


def bin_centres() -> dict[str, list[float]]:
    return {cat: [v * 100 for v in G.marker_centre(m)] for cat, m in SORTING["category_bins"].items()}


def on_bin(pos: list[float]) -> str | None:
    for cat, c in bin_centres().items():
        if math.dist(pos, c) <= SORTING["in_place_cm"] + 0.5:
            return cat
    return None


def phone_colour_at(pos: list[float]) -> str | None:
    o = G.pick((G.status()["localization"].get("phone") or {}).get("objects", []), near_cm=pos, within_cm=2.0)
    return o["label"] if o else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="list the blocks and what would happen; nothing sent")
    parser.add_argument("--max-blocks", type=int, default=6)
    args = parser.parse_args()
    results, failed = [], []
    try:
        with (G.nullcontext(None) if args.dry_run else G.Session()) as sess:
            for _ in range(args.max_blocks):
                s = G.status() if args.dry_run else G.observe(sess)
                todo = []
                for c in candidates(s):
                    home = on_bin(c["map_cm"])
                    if home:
                        continue
                    if any(math.dist(c["map_cm"], f) < 2.0 for f in failed):
                        continue
                    bx, by = G.map_to_base(c["map_cm"][0] / 100, c["map_cm"][1] / 100)
                    if not G.MIN_REACH <= math.hypot(bx, by) <= G.MAX_REACH:
                        results.append(dict(block=c["map_cm"], action="skip",
                                            reason=f"{math.hypot(bx, by)*100:.1f} cm from the base; place it at map y "
                                                   f"{G.MIN_REACH*100 - G.DY*100:.0f}-{G.MAX_REACH*100 - G.DY*100:.0f} cm"))
                        failed.append(c["map_cm"])
                        continue
                    todo.append(c)
                G.log("sort_round", blocks=[dict(c) for c in candidates(s)], todo=[c["map_cm"] for c in todo])
                if args.dry_run:
                    for c in todo:
                        print("would sort block at", c["map_cm"], "colour top:", c["colour"], "(seen by", c["seen_by"] + ")")
                    break
                if not todo:
                    break
                c = todo[0]
                started = time.time()
                try:
                    plan = G.locate(None, sess, near_cm=c["map_cm"])
                except G.ConsoleError as e:   # nothing touched yet: skip it, carry on
                    results.append(dict(block=c["map_cm"], action="skip", reason=f"could not localize: {e}"))
                    failed.append(c["map_cm"])
                    continue
                pos = plan["board"]["map_cm"]
                colour = phone_colour_at(pos) or c["colour"]
                category, source = resolve(colour, plan["board"].get("sticker"))
                G.log("category", block=pos, colour_top=colour, sticker=plan["board"].get("sticker"),
                      category=category, source=source)
                if category is None:
                    results.append(dict(block=pos, action="skip", reason=source))
                    failed.append(c["map_cm"])
                    continue
                bin_id = SORTING["category_bins"][category]
                target = bin_centres()[category]
                blocker = [o for o in candidates(G.status()) if math.dist(o["map_cm"], target) <= SORTING["occupied_cm"]
                           and math.dist(o["map_cm"], pos) > 2.0]
                if blocker:
                    results.append(dict(block=pos, action="skip", reason=f"bin {bin_id} ({category}) occupied"))
                    failed.append(c["map_cm"])
                    continue
                if not plan["pitch_verified"]:
                    results.append(dict(block=pos, action="skip", reason=f"only an unverified pitch {plan['pitch']} reaches it"))
                    failed.append(c["map_cm"])
                    continue
                G.hover(sess)
                try:
                    G.grasp(sess)
                except G.GripMissed as e:   # fingers closed on nothing, gripper fine: note it, go on
                    results.append(dict(block=pos, action="missed", category=category, source=source, reason=str(e)))
                    failed.append(c["map_cm"])
                    continue
                G.put(bin_id, sess)
                landed = G.pick(G.observe(sess)["localization"]["board"].get("objects", []) +
                                (G.status()["localization"].get("phone") or {}).get("objects", []), near_cm=target)
                results.append(dict(block=pos, action="moved", category=SORTING["category_names"][category], source=source,
                                    bin=bin_id, landed=landed and landed["map_cm"],
                                    error_cm=round(math.dist(landed["map_cm"], target), 2) if landed else None,
                                    seconds=round(time.time() - started)))
                G.log("sorted", **results[-1])
    except G.ConsoleError as e:
        G.log("sort_stopped", reason=str(e), done=results)
        print("\nSTOPPED:", e)
        sys.exit(1)
    print("\nsummary:")
    for r in results:
        print(" ", json.dumps(r, ensure_ascii=False))
    sources = [r["source"].split(" (")[0].split(";")[0] for r in results if r.get("source")]
    print("category decided by:", {k: sources.count(k) for k in sorted(set(sources))})


if __name__ == "__main__":
    main()
