"""Run one flow stage from the shell. Each stage reads the previous stage's file from the session directory
unless an input is given explicitly.

    python src/flow/cli.py locate  --session data/flow/S --scan data/runs/pipeline/20261106_160502 [--bins D]
    python src/flow/cli.py map     --session data/flow/S [--located other/located.json] [--extra boxes.json]
    python src/flow/cli.py plan    --session data/flow/S --pick b3 --to-bin bin2      (or --to-table X Y [--yaw D])
    python src/flow/cli.py plan    --session data/flow/S --task task.json --map external_map.json
    python src/flow/cli.py execute --session data/flow/S                     (dry run: nothing moves)
    python src/flow/cli.py execute --session data/flow/S --really [--console http://127.0.0.1:8770]
    python src/flow/cli.py scan    --session data/flow/S --really [--console URL] [--bins D]
    python src/flow/cli.py verify  --session data/flow/S --after data/flow/S2/located.json

scan and execute --really move the arm; on the real console they ask for an on-site confirmation first
(docs/05 hardware safety boundary). The simulator console is http://127.0.0.1:8770.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flow import stages as ST          # noqa: E402
from flow.session import FlowSession   # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="stage", required=True)
    for name in ("scan", "locate", "map", "plan", "execute", "verify"):
        sp = sub.add_parser(name)
        sp.add_argument("--session", help="session directory (default: a new data/flow/<stamp>)")
        if name in ("scan", "execute"):
            sp.add_argument("--really", action="store_true", help="move the arm")
            sp.add_argument("--console", help="console URL (default console_step.BASE, the real arm)")
        if name in ("scan", "locate"):
            sp.add_argument("--bins", help="bin design, e.g. bins_acrylic_v2_pair")
    sub.choices["locate"].add_argument("--scan", help="scan directory (default <session>/scan)")
    sub.choices["locate"].add_argument("--apply-yaw", action="store_true", help="apply the registration yaw")
    sub.choices["map"].add_argument("--located")
    sub.choices["map"].add_argument("--extra", help="JSON list of extra boxes {name, centre_m, size_m, yaw_deg}")
    sub.choices["map"].add_argument("--cell", type=float)
    pl = sub.choices["plan"]
    pl.add_argument("--task", help="task JSON (else built from --pick/--to-*)")
    pl.add_argument("--map", help="collision map (default <session>/collision_map.json)")
    pl.add_argument("--pick")
    pl.add_argument("--to-bin")
    pl.add_argument("--to-table", type=float, nargs=2, metavar=("X_M", "Y_M"))
    pl.add_argument("--yaw", type=float)
    pl.add_argument("--start", type=float, nargs=5)
    pl.add_argument("--tuning", help="JSON overrides of config/planning_tuning.json")
    ex = sub.choices["execute"]
    ex.add_argument("--plan")
    ex.add_argument("--stop-after", type=int)
    vf = sub.choices["verify"]
    vf.add_argument("--after", required=True)
    vf.add_argument("--before")
    vf.add_argument("--plan")
    vf.add_argument("--tolerance", type=float, default=0.015)
    a = ap.parse_args(argv)
    sess = FlowSession(a.session)
    if a.stage == "scan":
        out = ST.scan(sess, a.console, a.bins, a.really)
    elif a.stage == "locate":
        out = ST.locate(sess, a.scan, a.bins, a.apply_yaw)
        out = dict(objects=[{k: o[k] for k in ("name", "kind", "class", "centre_m", "yaw_deg", "sigma_xy_m")}
                            for o in out["objects"]], placed_by=out["placed_by"])
    elif a.stage == "map":
        extra = json.loads(Path(a.extra).read_text(encoding="utf-8")) if a.extra else ()
        m = ST.build_map(sess, a.located, extra, a.cell)
        out = dict(objects=[o["name"] for o in m.objects], shape=m.shape, file=str(sess.file("collision_map.json")))
    elif a.stage == "plan":
        if a.task:
            task = a.task
        else:
            to = {"bin": a.to_bin} if a.to_bin else {"table_m": a.to_table, "yaw_deg": a.yaw}
            task = dict(pick=int(a.pick) if a.pick and a.pick.isdigit() else a.pick, to=to, start=a.start,
                        tuning=json.loads(a.tuning) if a.tuning else None)
        out = ST.plan(sess, task, a.map)
        out = dict(ok=out["ok"], steps=[f"{s['plan']}:{s['name']} {s['command']}" for s in out["steps"]],
                   reasons=out["reasons"][:6])
    elif a.stage == "execute":
        out = ST.execute(sess, a.plan, a.console, a.really, stop_after=a.stop_after)
    else:
        out = ST.verify(sess, a.after, a.before, a.plan, a.tolerance)
    print(json.dumps(out, ensure_ascii=False, indent=1, default=str))
    print(f"session: {sess.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
