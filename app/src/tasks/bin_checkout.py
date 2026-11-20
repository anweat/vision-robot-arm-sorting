"""Bring-up steps for the acrylic bins, from the harmless to the real, each its own command. None of them grips.

    python3 src/tasks/bin_checkout.py plan                      # no arm: where each drop slot is and whether it can be reached
    python3 src/tasks/bin_checkout.py locate [--bin-side left]  # looks at each bin's stickers from its own view pose, prints where
                                                                # the bins are and how far from the plan; moves the arm (4 views)
    python3 src/tasks/bin_checkout.py hover --bin 2 [--slot 0]  # carries the EMPTY gripper over a bin at the drop height, stops
                                                                # there for you to look, then comes back; moves the arm

`locate` and `hover` move the arm: someone has to be standing at it, the table around the bins clear, the bins fixed
down (a bin the gripper nudges is a bin in the wrong place for every block after). They ask for a typed `y` first.
`hover` checks what the simulation cannot: that the open fingers clear the walls (the simulation counts the contacts
of a fingertip line, not the finger shape), that the tip is where the forward kinematics says (the arm sags near full
reach), and that the drop point is over the opening, by eye, from above.

Each prints what it found; nothing is written to config. After `locate`, write the real bin positions down (the numbers
are in the run's bins_report) before the first block goes in.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import bins as B  # noqa: E402
import console_grasp as G  # noqa: E402
import field_scan as SC  # noqa: E402
import run as R  # noqa: E402
import sort_blocks as SB  # noqa: E402


def confirm(what: str) -> None:
    answer = input(f"{what}\n  周围安全、有人看着机械臂？输入 y 继续：").strip().lower()
    if answer != "y":
        raise SystemExit("已取消，机械臂没有动")


def stand(design: str, only, run: Path | None) -> B.BinSet:
    """The bins as `locate` found them in `run`, else as designed."""
    bins = B.BinSet(design, only)
    if run is not None and (run / "bin_views.json").exists():
        B.locate_run(run, bins, G.TABLE_Z)
    return bins


def cmd_plan(args) -> None:
    only = B.parse_only(args.bin_side)
    R.use_bins(B.BinSet(args.design, only))
    G.DX, G.DY = G.MAP["placement"]["base_from_map"]["dx"], G.MAP["placement"]["base_from_map"]["dy"]
    z = R.bin_drop_z(R.BINS)
    print(f"drop height: fingertip {z*100:.2f} cm above the base-frame zero (table {G.TABLE_Z*100:.2f}, rim {R.BINS.wall_top_m*100:.1f} above it)")
    for message in B.table_check(R.BINS, SB.SORTING["bins"]["min_plate_gap_mm"]):
        print("注意：", message)
    for b in R.BINS.bins.values():
        print(f"bin {b.number} {b.category}:")
        for i, slot in enumerate(R.bin_slots()):
            where = R.bin_drop_map_cm(b, i)
            plan = G.plan_put(150, ",".join(map(str, where)), list(G.OBSERVE) + [G.ROLL_ZERO], z)
            if plan is None:
                print(f"   slot {i} {where}: NOT reachable")
            else:
                print(f"   slot {i} {where}: hover {plan['hover']} pitch {plan['pitch']} J5 {plan['j5_put']}"
                      f"{'' if plan['within_soft_limits'] else '  (outside the soft limits)'}")


def cmd_locate(args) -> None:
    only = B.parse_only(args.bin_side)
    if not SC.has_bin_views(args.design):
        raise SystemExit(f"{args.design} 没有观察位姿（config/bin_views_{args.design}.json）：这套桶靠地图上的落位线摆放，不用相机找")
    G.require_empty_gripper()
    confirm(f"将依次把相机转到 {'全部' if only is None else only} 号桶的观察位姿拍照，共 {len(only or (1, 2, 3, 4))} 个视角")
    run = SC.RUNS / (time.strftime("%Y%m%d_%H%M%S") + "_bins")
    run.mkdir(parents=True, exist_ok=True)
    with G.Session() as sess:
        SC.scan_bins(run, sess, args.design, only)
    found = R.locate_bins(run, args.design, only)
    (run / "bins_report.json").write_text(json.dumps([b.summary() for b in found.bins.values()], indent=1, ensure_ascii=False))
    print("记录：", run)


def cmd_hover(args) -> None:
    only = B.parse_only(args.bin_side)
    run = Path(args.run).resolve() if args.run else None
    bins = stand(args.design, only, run)
    R.use_bins(bins)
    G.DX, G.DY = G.MAP["placement"]["base_from_map"]["dx"], G.MAP["placement"]["base_from_map"]["dy"]
    b = bins.bins[args.bin]
    where = ",".join(map(str, R.bin_drop_map_cm(b, args.slot)))
    z = R.bin_drop_z(bins)
    G.require_empty_gripper()
    s = G.status()
    plan = G.plan_put(150, where, s["pose"], z)
    if plan is None:
        raise SystemExit(f"{args.bin} 号桶槽位 {args.slot} 在投放高度够不着（plan_put 无解），不动")
    print(f"bin {b.number} ({b.source}) slot {args.slot} drop at map {where} cm, tip z {z*100:.2f} cm")
    print(f"path: carry {plan['carry']} -> turn {plan['turn']} -> hover {plan['hover']} (pitch {plan['pitch']}, J5 {plan['j5_put']})")
    j5 = plan["j5_put"] if args.j5 is None else args.j5
    if j5 != plan["j5_put"]:
        print(f"J5 用 {j5}（计划里是 {plan['j5_put']}）：空夹爪核对时手腕转角不影响路径，用来待在控制台当前档位的软限位内")
    confirm("将带着空夹爪走到桶口上方的投放高度，停住等你看，然后退回携带高度")
    with G.Session() as sess:
        try:
            if max(abs(a - b) for a, b in zip(plan["carry"], s["pose"][:4])) > 1:     # a repeat of the same pose is refused
                sess.pose(plan["carry"] + [s["pose"][4]])
            sess.pose(plan["turn"] + [j5])
            sess.pose(plan["hover"] + [j5])
        except G.ConsoleError as e:
            # a refused leg sends nothing, but the legs before it have run: the arm stands where the last one left it
            print(f"没走完：{e}\n机械臂现在在 {G.status()['pose']}")
            confirm("要把它退回观察位吗（取消则留在原地，之后用页面退回）")
            G.move_to(sess, G.OBSERVE)
            raise SystemExit(1)
        time.sleep(1.0)
        pose = G.status()["pose"]
        tip = G.tip(pose)
        print(f"readback {pose}; forward-kinematics tip ({tip[0]*100:.1f}, {tip[1]*100:.1f}, {tip[2]*100:.1f}) cm,"
              f" asked ({plan['tx']*100:.1f}, {plan['ty']*100:.1f}, {z*100:.1f})")
        print("现在看：夹爪在桶口正上方吗？张开的手指离桶壁有多远？（俯视；最好拍张照）")
        input("看完按回车，机械臂退回：")
        G.lift(sess, plan["turn"])
        G.move_to(sess, G.OBSERVE)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--design", default=B.DEFAULT_DESIGN)
    p.add_argument("--bin-side", default="all", metavar="left|right|all|1,2")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("plan")
    sub.add_parser("locate")
    h = sub.add_parser("hover")
    h.add_argument("--bin", type=int, required=True)
    h.add_argument("--slot", type=int, default=0)
    h.add_argument("--j5", type=int, help="wrist roll to use instead of the plan's (the console's normal profile keeps J5 in 95-180)")
    h.add_argument("--run", help="a `locate` run directory: hover over the bin where it was found there")
    args = p.parse_args()
    {"plan": cmd_plan, "locate": cmd_locate, "hover": cmd_hover}[args.cmd](args)


if __name__ == "__main__":
    main()
