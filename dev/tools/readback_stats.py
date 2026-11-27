"""Read one or more execution.jsonl files/session directories; report readback minus command in degrees.

python dev/tools/readback_stats.py data/sim/session --json data/sim/readback.json
Only logged readbacks are observations. Continuous intermediate poses with null readback are counted as
missing, never fabricated. p90 uses nearest rank. Signed and absolute errors are both reported.
"""
import argparse
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_logs import distribution, files, number, read, write_json


def analyze(inputs, readback="final"):
    groups, warnings, sources = defaultdict(list), [], files(inputs, "execution.jsonl")
    missing = steps = 0
    for path in sources:
        rows, problems = read(path)
        warnings.extend(problems)
        previous = None
        for row in rows:
            if row.get("event") == "start":
                previous = row.get("readback")
                continue
            if row.get("event") != "step":
                continue
            steps += 1
            command = row.get("command")
            rb = row.get("first_readback", row.get("readback")) if readback == "first" else row.get("readback")
            if not isinstance(command, list):
                warnings.append(f"{path}: step {row.get('i')}: missing command")
                continue
            indices = [5] if row.get("kind") == "gripper" else range(min(5, len(command)))
            for joint in indices:
                if joint >= len(command) or not number(command[joint]):
                    continue
                if not isinstance(rb, list) or joint >= len(rb) or not number(rb[joint]):
                    missing += 1
                    continue
                before = previous[joint] if isinstance(previous, list) and joint < len(previous) else None
                delta = command[joint] - before if number(before) else None
                direction = "unknown" if delta is None else "positive" if delta > 1e-6 else "negative" if delta < -1e-6 else "stationary"
                passing = row.get("passing", row.get("stop") is False) is True
                groups[(joint + 1, direction, passing)].append(rb[joint] - command[joint])
            previous = command
    table = [dict(joint=j, direction=d, passing=p, signed_deg=distribution(v),
                  absolute_deg=distribution([abs(x) for x in v])) for (j, d, p), v in sorted(groups.items())]
    return dict(readback=readback, percentile="nearest rank", files=[str(p) for p in sources],
                steps=steps, missing_readbacks=missing, warnings=warnings, groups=table)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--json", type=Path)
    ap.add_argument("--readback", choices=("final", "first"), default="final")
    args = ap.parse_args(argv)
    try:
        report = analyze(args.inputs, args.readback)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    print("joint direction passing n signed_median signed_p90 signed_max abs_p90 abs_max (deg)")
    for row in report["groups"]:
        s, a = row["signed_deg"], row["absolute_deg"]
        print(row["joint"], row["direction"], row["passing"], s["n"], s["median"], s["p90"], s["max"], a["p90"], a["max"])
    print(f"missing readbacks: {report['missing_readbacks']}; warnings: {len(report['warnings'])}")
    for warning in report["warnings"]:
        print(warning, file=sys.stderr)
    if args.json:
        write_json(args.json, report)
    return 0 if report["groups"] and not report["warnings"] else 1


if __name__ == "__main__":
    sys.exit(main())
