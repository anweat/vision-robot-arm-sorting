"""Joint paths between two arm poses through the collision map: straight if clear, else A* over a lattice.

The lattice is a grid of FK-tip positions (tuning search.cell_m) anchored at the start's tip. A node is the
IK pose that puts the tip there; its pitch and J5 follow the start->goal blend by how far along the
straight line the node is (pitch falls back to the nearest of search.pitches_deg that reaches and is
clear). Nodes are checked with the coarse point set; edges lazily, when A* pops them (a failed edge sends
the node back with its next-best parent). The found chain is then shortcut and every final leg is
re-checked with the full point set, so a returned route never rests on the coarse checks.

Cost = tip travel + joint_cost per degree of the largest joint change + a clearance penalty near obstacles;
heuristic = straight tip distance to the goal (weighted by search.heuristic_weight, default 1.0).
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Sequence

from . import checks as K
from . import gripper as GR
import dofbot_kinematics as kin   # noqa: E402

NEIGHBOURS = [d for d in itertools.product((-1, 0, 1), repeat=3) if d != (0, 0, 0)]


@dataclass
class Route:
    ok: bool
    legs: list = field(default_factory=list)          # J1-J5 waypoints after the start, the last is the goal
    checks: list = field(default_factory=list)        # Check per leg (full point set)
    method: str = ""
    reason: str = ""
    stats: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return dict(ok=self.ok, method=self.method, reason=self.reason, stats=self.stats,
                    legs=[[round(v, 2) for v in q] for q in self.legs], checks=[c.to_dict() for c in self.checks])


def tip(q5) -> list:
    return kin.forward(list(q5)[:4])["tip"]


def pitch_of(q5) -> float:
    return kin.forward(list(q5)[:4])["tip_pitch_deg"][0]


def table_allowance(w: K.World, a5, b5, delta, held) -> float:
    """A move may start or end low (a grasp or release pose) but not dip below its ends on the way."""
    t = w.tuning
    ends = min(GR.lowest(a5, delta, held), GR.lowest(b5, delta, held)) - t.table_z
    return min(t.table(t.clear["path_table_m"]), ends - 0.002)


def plan_route(w: K.World, a5: Sequence[float], b5: Sequence[float], delta: float, held=None, *,
               target_margin: float | None = None, table_margin: float | None = None) -> Route:
    t0 = time.time()
    a5, b5 = [float(v) for v in list(a5)[:5]], [float(v) for v in list(b5)[:5]]
    tm = table_allowance(w, a5, b5, delta, held) if table_margin is None else table_margin
    kw = dict(target_margin=target_margin, table_margin=tm)
    direct = K.move_check(w, a5, b5, delta, held, **kw)
    if direct.ok:
        return Route(True, [b5], [direct], "direct", stats=dict(time_s=round(time.time() - t0, 3)))
    for name, q in (("start", a5), ("goal", b5)):
        c = K.pose_check(w, q, delta, held, **kw)
        if not c.ok:
            return Route(False, method="astar", reason=f"the {name} pose itself is not clear: {c.failed()}")
    bad: set = set()
    stats: dict = {}
    for repair in range(int(w.tuning.search.get("repairs", 4)) + 1):
        chain, stats = _astar(w, a5, b5, delta, held, kw, bad)
        stats.update(repairs=repair, time_s=round(time.time() - t0, 3))
        if chain is None:
            return Route(False, method="astar", reason=stats.pop("reason"), stats=stats)
        legs, checks, why, broken = _shortcut(w, chain, delta, held, kw)
        stats.update(lattice_nodes=len(chain), legs=len(legs or []), time_s=round(time.time() - t0, 3))
        if legs is not None:
            return Route(True, legs, checks, "astar", stats=stats)
        bad.add(broken)                 # an edge the coarse check let through: search again without it
    return Route(False, method="astar", reason=why, stats=stats)


def _astar(w: K.World, a5, b5, delta, held, kw, known_bad: set = frozenset()) -> tuple[list | None, dict]:
    t = w.tuning
    s = t.search
    cell, weight = float(s["cell_m"]), float(s.get("heuristic_weight", 1.0))
    ta, tb = tip(a5), tip(b5)
    pa, pb = pitch_of(a5), pitch_of(b5)
    seg = [tb[i] - ta[i] for i in range(3)]
    seg2 = sum(v * v for v in seg) or 1e-9
    zlo, zhi = t.table_z + s["z_min_above_table_m"], t.table_z + s["z_max_above_table_m"]
    pad = s["xy_pad_m"]
    box = [min(ta[0], tb[0]) - pad, max(ta[0], tb[0]) + pad, min(ta[1], tb[1]) - pad, max(ta[1], tb[1]) + pad]
    jc, cc, band = s["joint_cost_m_per_deg"], s["clearance_cost_m"], s["clearance_band_m"]
    pitches = [float(p) for p in s["pitches_deg"]]
    nodes: dict = {}

    def pos(k):
        return [ta[0] + k[0] * cell, ta[1] + k[1] * cell, ta[2] + k[2] * cell]

    def node(k):
        if k in nodes:
            return nodes[k]
        p = pos(k)
        out = None
        if box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3] and zlo <= p[2] <= zhi:
            f = max(0.0, min(1.0, sum((p[i] - ta[i]) * seg[i] for i in range(3)) / seg2))
            want = pa + f * (pb - pa)
            j5 = a5[4] + f * (b5[4] - a5[4])
            for pitch in sorted(set([round(want)] + pitches), key=lambda x: abs(x - want))[:5]:
                q = kin.inverse(p[0], p[1], p[2], pitch)
                if q is None:
                    continue
                q5 = [float(v) for v in q] + [j5]
                if not K.limits_ok(q5):
                    continue
                c = K.pose_check(w, q5, delta, held, coarse=True, **kw)
                if c.ok:
                    clr = min(c.obstacles, c.arm_body)
                    out = (q5, cc * max(0.0, 1.0 - clr / band) if band > 0 else 0.0)
                    break
        nodes[k] = out
        return out

    def h(k):
        return math.dist(pos(k), tb)

    def step_cost(qa, qb, pena, penb, da):
        return da + jc * max(abs(x - y) for x, y in zip(qa, qb)) + 0.5 * (pena + penb)

    start = (0, 0, 0)
    nodes[start] = (a5, 0.0)
    GOAL = "goal"
    g = {start: 0.0}
    parent = {start: None}
    bad: set = set()
    good: set = set()
    closed: set = set()
    heap = [(weight * h(start), 0.0, start)]
    expanded, limit = 0, int(s["max_expansions"])
    max_jump = 25.0

    def edge_ok(u, v, qu, qv):
        if (tuple(round(x, 3) for x in qu), tuple(round(x, 3) for x in qv)) in known_bad:
            return False
        if (u, v) in good:
            return True
        if (u, v) in bad:
            return False
        c = K.move_check(w, qu, qv, delta, held, coarse=True, skip_ends=True, **kw)
        (good if c.ok else bad).add((u, v))
        return c.ok

    while heap:
        f, gu, u = heapq.heappop(heap)
        if u in closed or gu > g.get(u, math.inf) + 1e-12:
            continue
        qu = a5 if u == start else (b5 if u == GOAL else nodes[u][0])
        pu = parent[u]
        if pu is not None:
            qp = a5 if pu == start else nodes[pu][0]
            if not edge_ok(pu, u, qp, qu):
                # lazy edge failed: best other closed neighbour as the parent, else drop it
                g.pop(u, None)
                best = None
                if u != GOAL:
                    for d in NEIGHBOURS:
                        n = (u[0] - d[0], u[1] - d[1], u[2] - d[2])
                        if n in closed and (n, u) not in bad and nodes.get(n):
                            qn = nodes[n][0]
                            if max(abs(x - y) for x, y in zip(qn, qu)) > max_jump:
                                continue
                            cost = g[n] + step_cost(qn, qu, nodes[n][1], nodes[u][1], math.dist(pos(n), pos(u)))
                            if best is None or cost < best[0]:
                                best = (cost, n)
                if best is not None:
                    g[u], parent[u] = best[0], best[1]
                    heapq.heappush(heap, (best[0] + weight * h(u), best[0], u))
                continue
        if u == GOAL:
            chain, k = [], u
            while k is not None:
                chain.append(b5 if k == GOAL else (a5 if k == start else nodes[k][0]))
                k = parent[k]
            return chain[::-1], dict(expanded=expanded, nodes_checked=len(nodes))
        closed.add(u)
        expanded += 1
        if expanded > limit:
            break
        pen_u = nodes[u][1]
        if math.dist(pos(u), tb) <= 1.75 * cell and max(abs(x - y) for x, y in zip(qu, b5)) <= max_jump * 2:
            cost = gu + step_cost(qu, b5, pen_u, 0.0, math.dist(pos(u), tb))
            if cost < g.get(GOAL, math.inf):
                g[GOAL], parent[GOAL] = cost, u
                heapq.heappush(heap, (cost, cost, GOAL))
        for d in NEIGHBOURS:
            v = (u[0] + d[0], u[1] + d[1], u[2] + d[2])
            if v in closed:
                continue
            nv = node(v)
            if nv is None or (u, v) in bad:
                continue
            if max(abs(x - y) for x, y in zip(qu, nv[0])) > max_jump:
                continue
            cost = gu + step_cost(qu, nv[0], pen_u, nv[1], cell * math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2))
            if cost < g.get(v, math.inf) - 1e-12:
                g[v], parent[v] = cost, u
                heapq.heappush(heap, (cost + weight * h(v), cost, v))
    why = (f"A*: no path within {limit} expansions" if expanded > limit
           else "A*: the free space around the start does not connect to the goal")
    return None, dict(expanded=expanded, nodes_checked=len(nodes), reason=why)


def _shortcut(w: K.World, chain: list, delta, held, kw) -> tuple[list | None, list, str, tuple | None]:
    """Greedy: from each kept pose, the farthest later pose a full-resolution straight move reaches.
    Returns (legs, checks, why, broken edge) with legs None when an adjacent pair fails at full resolution."""
    legs, checks, i = [], [], 0
    use_short = bool(w.tuning.search.get("shortcut", True))
    while i < len(chain) - 1:
        js = range(len(chain) - 1, i, -1) if use_short else [i + 1]
        for j in js:
            c = K.move_check(w, chain[i], chain[j], delta, held, **kw)
            if c.ok:
                legs.append(chain[j])
                checks.append(c)
                i = j
                break
        else:
            broken = (tuple(round(x, 3) for x in chain[i]), tuple(round(x, 3) for x in chain[i + 1]))
            return None, [], (f"A* leg {i}->{i + 1} passes the coarse check but not the full one "
                              f"({K.move_check(w, chain[i], chain[i + 1], delta, held, **kw).failed()})"), broken
    return legs, checks, "", None
