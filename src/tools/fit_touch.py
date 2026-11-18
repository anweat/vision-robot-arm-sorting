"""Fit joint zero offsets and the table from fine touch-probe contacts (src/tasks/touch_survey.py, doc 33).

Each contact says: with the joints at the precise readback q, the lowest finger point of the gripper mesh
is on the table. With true angles q + d (d: constant zero offsets of J2-J4, optionally a J2 sag growing
with reach) and a table z0 + ax*x + ay*y, every contact gives one residual

    lowest_z(q + d) - (z0 + ax*x + ay*y)

and least squares over all of them gives d and the table. Only heights are measured, so J1's zero and
anything that moves the tip sideways stay out of reach; the plane tilt separates once the contacts cover
several J1 angles. Reported next to the fit: residuals with d = 0 (today's model), leave-one-out residuals,
and the parameter correlations (a pair near +-1 is not separated by this set of poses).

    uv run --with numpy python src/tools/fit_touch.py data/runs/touch/<time> [more runs] [--model offsets|sag|table]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402
import gripper_geometry as gg  # noqa: E402

MODELS = {
    "table": ("z0",),
    "plane": ("z0", "ax", "ay"),
    "offsets": ("z0", "ax", "ay", "d2", "d3", "d4"),
    "sag": ("z0", "ax", "ay", "d2", "d3", "d4", "s2"),
}


def onset(trace: list[dict]) -> dict | None:
    """Where the tip met the surface: the knee of the lag (readback minus command, per step) between a free
    part (flat, the dead band's offset) and a pressed part (rising: the tip held back). A desk mat never stops
    the tip outright (2026-11-18: it sank at ~half the commanded rate), so the last pose is up to a few mm
    into the mat; the knee is where it began. None when there is no clear knee."""
    lag = np.array([t["lag_mm"] for t in trace], dtype=float)
    n = len(lag)
    if n < 8:
        return None
    best = None
    for k0 in range(3, n - 2):
        a, b = lag[:k0 + 1], lag[k0 + 1:]
        c = float(np.mean(a))
        x = np.arange(1, len(b) + 1, dtype=float)
        s = float(np.sum(x * (b - c)) / np.sum(x * x))          # line through the knee
        sse = float(np.sum((a - c) ** 2) + np.sum((b - c - s * x) ** 2))
        if best is None or sse < best[0]:
            best = (sse, k0, c, s)
    _, k0, c, s = best
    if s < 0.3:
        return None
    return dict(step=k0, q=trace[k0]["q"], free_lag_mm=round(c, 2), pressed_mm_per_step=round(s, 2),
                pressed_steps=n - 1 - k0)


def contacts(runs: list[Path]) -> list[dict]:
    out = []
    for run in runs:
        for line in (run / "touches.jsonl").read_text().splitlines():
            row = json.loads(line)
            res = row["result"]
            if row.get("air"):
                continue
            knee = onset(row.get("trace") or [])
            if not res.get("contacted") and knee is None:
                continue
            q = res["pose"]                         # J1-J4 precise floats (last pose), J5/J6 their command
            start = knee["q"] if knee else q[:4]
            out.append(dict(run=run.name, i=row["i"], q=list(start), q_last=q[:4], j5=q[4],
                            j6=row.get("j6_readback") or q[5], why=res["why"], knee=knee,
                            contacted=res.get("contacted"), plan=row["plan"]))
    return out


def predict(c: dict, p: dict) -> float:
    q = list(c["q"])
    reach = math.hypot(*kin.forward(q)["tip"][:2])
    q[1] += p.get("d2", 0.0) + p.get("s2", 0.0) * (reach - 0.20) * 10   # s2: deg per 10 cm of reach beyond 20 cm
    q[2] += p.get("d3", 0.0)
    q[3] += p.get("d4", 0.0)
    low = gg.lowest_z(q + [c["j5"], c["j6"]], gg.delta_for_j6(c["j6"]))
    x, y = kin.forward(q)["tip"][:2]
    return low - (p.get("z0", 0.0) + p.get("ax", 0.0) * x + p.get("ay", 0.0) * y)


def residuals(cs, names, v):
    p = dict(zip(names, v))
    return np.array([predict(c, p) for c in cs])


PRIOR_DEG = 5.0      # weak prior on offsets / sag: keeps a direction the poses do not see from running away


def jacobian(cs, names, v):
    """Central differences. Steps are coarse on purpose: kin.forward rounds positions to 0.1 mm, so a step of
    0.001 deg only sees the orientation part of the tool (all three offsets then look alike)."""
    J = np.empty((len(cs), len(names)))
    for k, n in enumerate(names):
        h = 0.25 if n.startswith(("d", "s")) else (1e-3 if n == "z0" else 1e-2)
        up, dn = v.copy(), v.copy()
        up[k] += h
        dn[k] -= h
        J[:, k] = (residuals(cs, names, up) - residuals(cs, names, dn)) / (2 * h)
    return J


def fit(cs: list[dict], names: tuple) -> dict:
    """Levenberg-Marquardt (angles in deg, lengths in m) with a weak zero-mean prior on the angle terms."""
    v = np.zeros(len(names))
    v[0] = float(np.mean([predict(c, {}) for c in cs]))
    angle = np.array([n.startswith(("d", "s")) for n in names])
    w = np.where(angle, 1e-3 / PRIOR_DEG, 0.0)          # 1 mm of residual per PRIOR_DEG of offset

    def cost(x):
        r = residuals(cs, names, x)
        return float(r @ r + np.sum((w * x) ** 2))

    lam, c0 = 1e-3, cost(v)
    for _ in range(60):
        r = residuals(cs, names, v)
        J = jacobian(cs, names, v)
        A = J.T @ J + np.diag(w ** 2)
        g = J.T @ r + w ** 2 * v
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
        c1 = cost(v + step)
        if c1 < c0:
            v, c0, lam = v + step, c1, lam * 0.3
            if np.max(np.abs(step)) < 1e-6:
                break
        else:
            lam *= 10
            if lam > 1e8:
                break
    r = residuals(cs, names, v)
    J = jacobian(cs, names, v)
    dof = max(len(cs) - len(names), 1)
    sigma = float(np.sqrt(r @ r / dof))
    cov = np.linalg.inv(J.T @ J + np.diag(w ** 2)) * sigma ** 2
    sd = np.sqrt(np.diag(cov))
    corr = cov / np.outer(sd, sd)
    return dict(values=dict(zip(names, v.tolist())), sd=dict(zip(names, sd.tolist())), rms_m=sigma,
                residuals_m=r.tolist(), corr=corr.tolist(), cond=float(np.linalg.cond(J)), prior_deg=PRIOR_DEG)


def loo(cs: list[dict], names: tuple) -> list[float]:
    out = []
    for k in range(len(cs)):
        rest = cs[:k] + cs[k + 1:]
        if len(rest) <= len(names):
            return []
        f = fit(rest, names)
        out.append(predict(cs[k], f["values"]))
    return out


def summary(cs: list[dict], names: tuple) -> dict:
    f = fit(cs, names)
    base = [predict(c, {"z0": f["values"]["z0"]}) for c in cs]
    out = dict(model=list(names), n=len(cs), fit=f, loo_m=loo(cs, names),
               today_rms_m=float(np.sqrt(np.mean(np.square(base)))))
    return out


def fmt(v: dict, names: tuple, sd: dict) -> str:
    parts = []
    for n in names:
        if n in ("z0",):
            parts.append(f"{n} {v[n] * 100:+.2f}±{sd[n] * 100:.2f} cm")
        elif n in ("ax", "ay"):
            parts.append(f"{n} {v[n] * 1000:+.1f}±{sd[n] * 1000:.1f} mm/m")
        else:
            parts.append(f"{n} {v[n]:+.2f}±{sd[n]:.2f}°")
    return ", ".join(parts)


def repeatability(cs: list[dict]) -> list[dict]:
    """Knee heights (model lowest finger point) grouped by touch point and descent mode: how well a touch repeats."""
    groups: dict = {}
    for c in cs:
        p = c["plan"]
        key = (p["j1"], p["r_cm"], p["pitch"], bool(p.get("lock_j2")))
        z = gg.lowest_z(list(c["q"]) + [c["j5"], c["j6"]], gg.delta_for_j6(c["j6"]))
        groups.setdefault(key, []).append(dict(z=z, knee=c["knee"], why=c["why"]))
    out = []
    for (j1, r, pitch, lock), rows in sorted(groups.items()):
        zs = np.array([x["z"] for x in rows])
        out.append(dict(j1=j1, r_cm=r, pitch=pitch, lock_j2=lock, n=len(zs), mean_cm=float(zs.mean() * 100),
                        sd_mm=float(zs.std(ddof=1) * 1000) if len(zs) > 1 else None,
                        heights_cm=[round(float(z) * 100, 2) for z in zs],
                        pressed=[(x["knee"] or {}).get("pressed_steps") for x in rows]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--model", default="all", choices=(*MODELS, "all"))
    ap.add_argument("--json", type=Path, help="write the full result here")
    a = ap.parse_args()
    cs = contacts(a.runs)
    print(f"{len(cs)} contacts from {', '.join(r.name for r in a.runs)}")
    if not cs:
        return
    for g in repeatability(cs):
        sd = f"{g['sd_mm']:.1f} mm" if g["sd_mm"] is not None else "-"
        print(f"  J1 {g['j1']} r {g['r_cm']} pitch {g['pitch']} {'J2 held' if g['lock_j2'] else 'straight'}: "
              f"n {g['n']}, knee {g['mean_cm']:+.2f} cm, sd {sd}; {g['heights_cm']}, pressed steps {g['pressed']}")
    results = {}
    for name in (MODELS if a.model == "all" else [a.model]):
        names = MODELS[name]
        if len(cs) <= len(names) + 1:
            print(f"{name}: too few contacts for {len(names)} parameters")
            continue
        s = summary(cs, names)
        results[name] = s
        f = s["fit"]
        lo = s["loo_m"]
        loo_txt = (f"loo median {np.median(np.abs(lo)) * 1000:.1f} / max {np.max(np.abs(lo)) * 1000:.1f} mm"
                   if lo else "loo n/a")
        print(f"{name:>8}: rms {f['rms_m'] * 1000:.2f} mm, {loo_txt}; {fmt(f['values'], names, f['sd'])}")
        big = [(names[i], names[j], f["corr"][i][j]) for i in range(len(names)) for j in range(i + 1, len(names))
               if abs(f["corr"][i][j]) > 0.9]
        if big:
            print("          strongly correlated: " + ", ".join(f"{a}/{b} {c:+.2f}" for a, b, c in big))
    if results:
        first = next(iter(results.values()))
        print(f"today's model (one table height, no offsets): rms {first['today_rms_m'] * 1000:.2f} mm")
    if a.json:
        a.json.write_text(json.dumps(dict(contacts=cs, results=results), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
