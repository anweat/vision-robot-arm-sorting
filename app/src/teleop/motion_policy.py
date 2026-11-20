"""The console's one set of limits, the speed setting and the paths built from them.

There are no profiles any more (docs/current/32-操作逻辑简化与分步任务.md §1.1): the console runs
with what the old `explore` profile did, because that is what the operator sessions and the automatic tasks
ran under since 2026-09-25. Two switches in `arm_limits.json` `runtime` decide whether the platform and the
table keep-out refuse a command or are only reported; by default (2026-11-18) both are only reported, and
the resistance stop (hold at readback) is what protects the arm."""
import copy
import math
import secrets
import time
from core import safe_arm

SPEED_DEFAULTS = dict(speed_default_deg_s=20, speed_min_deg_s=3, speed_max_deg_s=40)


def runtime_config(base):
    """Joint limits are the course limits with J2-J4 down to -10 deg (2026-09-24: 29 of 29 refusals were
    J3/J4 asking for -1..-2 deg; Arm_Lib only rejects > 180 and the joints were pushed to -4 by hand without
    harm). J1 and J5 keep their course limits: those are the short-cable joints."""
    cfg = copy.deepcopy(base)
    rt = base.get('runtime', {})
    limits = {j: tuple(v) for j, v in base['course_limits_deg'].items()}
    for j in (2, 3, 4):
        limits[j] = (rt.get('j234_lower_deg', -10), limits[j][1])
    cfg['soft_limits_deg'] = limits
    cfg['course_limits_deg'] = dict(limits)
    ko = cfg['keep_out']
    if not rt.get('enforce_platform', False):
        ko['platform_min_z_m'] = -1.0
    if rt.get('enforce_table', False):
        ko['table_margin_m'] = rt.get('table_margin_m', .005)
    else:
        ko['table_z_m'] = -1.0
    cfg['max_ik_jump_deg'] = rt.get('max_ik_jump_deg', 30)
    return cfg


def speed_range(motion):
    s = dict(SPEED_DEFAULTS, **{k: v for k, v in motion.items() if k in SPEED_DEFAULTS})
    return s['speed_min_deg_s'], s['speed_max_deg_s'], s['speed_default_deg_s']


def motion_for(speed, motion):
    """Segment size and timing for a joint speed in deg/s (doc 32 §1.3): slow speeds use fine segments,
    fast ones coarse segments, so every segment still takes about 0.4 s and is checked by readback.
    6 deg/s gives about the old standard timing, 37 deg/s the old explore timing (15 deg, 400 + 120 ms)."""
    step = min(15.0, max(3.0, speed * .4))
    out = dict(motion)
    out.update(max_step_deg=step, segment_ms=max(250, int(round(1000 * step / speed))),
               settle_ms=int(min(300, max(120, 300 - 5 * speed))), speed_deg_s=speed)
    return out


def cartesian_goal(current, axis, amount, cfg):
    """Joint segments that move the tip `amount` cm along base x/y/z, or change the tip pitch by
    `amount` degrees about the tip. Refuses IK failures and sudden branch changes."""
    import dofbot_kinematics as kin
    if len(current) != 6 or any(v is None for v in current):
        raise ValueError('关节不可读')
    fk = kin.forward(current[:4])
    tip = list(fk['tip']); pitch = fk['tip_pitch_deg'][0]
    if axis == 'pitch':
        pitch += amount
    else:
        tip['xyz'.index(axis)] += amount/100
    solution = kin.inverse(*tip, pitch)
    if solution is None:
        raise ValueError('目标位置超出可达范围')
    goal = [int(round(v)) for v in solution]+list(current[4:])
    jump = max(abs(a-b) for a, b in zip(goal[:4], current[:4]))
    if jump > cfg.get('max_ik_jump_deg', 15):
        raise ValueError(f'逆解跳变 {jump}°，拒绝执行')
    return safe_arm.plan_segments(current, goal, cfg['motion']['max_step_deg']) or [goal]


def drive_goal(base, mode, v, wrist, dt, speed, residual, cfg):
    """One segment of stick driving (doc 32 §1.5) from `base`, as a float pose; `residual` carries the
    fractions of a degree the integer servo commands could not take yet. Cartesian mode: v = [radial,
    turn (J1), up, pitch]; joint mode: v = [J1, J2, J3, J4]. `wrist` turns J5. Raises ValueError when the
    tip cannot get there; the caller keeps the arm where it is and tries again on the next stick update."""
    import dofbot_kinematics as kin
    step = speed * dt                         # deg this segment at full deflection
    want = [float(x) for x in base]
    if mode == 'joint':
        for j in range(4):
            want[j] += v[j] * step
    else:
        fk = kin.forward(base[:4])
        x, y, z = fk['tip']
        pitch = fk['tip_pitch_deg'][0]
        lin = speed * .0025 * dt              # m: 20 deg/s -> 5 cm/s
        r = math.hypot(x, y) + v[0] * lin
        j1 = base[0] + v[1] * step
        a = math.radians(j1 - 90)
        sol = kin.inverse(-r * math.sin(a), r * math.cos(a), z + v[2] * lin, pitch + v[3] * step, servo1_hint=j1)
        if sol is None:
            raise ValueError('目标位置超出可达范围')
        jump = max(abs(a_ - b) for a_, b in zip(sol, base[:4]))
        if jump > cfg.get('max_ik_jump_deg', 15):
            raise ValueError(f'逆解跳变 {jump:.0f}°，拒绝执行')
        want[:4] = sol
    want[4] += wrist * step
    goal = list(base)
    for j in range(5):
        exact = want[j] + residual[j]
        goal[j] = int(round(exact))
        residual[j] = exact - goal[j]
    return goal


def path_plan(current, target, cfg):
    """Segments from `current` to `target` (None = keep that joint), every one checked against the limits
    and the keep-out before anything is sent. Returns the preview the console shows."""
    if len(current) != 6 or any(v is None for v in current):
        raise ValueError('关节不可读，不能规划')
    target = [c if t is None else int(t) for c, t in zip(current, target)]
    segments = safe_arm.plan_segments(current, target, cfg['motion']['max_step_deg'])
    previous, margins = list(current), []      # no segments: already there (goto then sends nothing)
    for i, step in enumerate(segments):
        try:
            safe_arm.validate_target(previous, step, cfg)
            margins.append(safe_arm.check_keep_out(previous, step, cfg, recover=True))
        except safe_arm.MotionError as exc:
            raise ValueError(f'路径第 {i+1}/{len(segments)} 段不可行：{exc}') from exc
        previous = step
    m = cfg['motion']
    return dict(id=secrets.token_hex(12), created_at=time.time(), start=list(current), target=target,
                segments=segments, largest_deg=max(abs(a-b) for a, b in zip(current, target)),
                seconds=round(len(segments)*(m['segment_ms']+m['settle_ms'])/1000, 1),
                margin_cm=round(min(margins)*100, 2) if margins else None)


def home_plan(current, cfg):
    """Path to observe_v1 (J5/J6 kept)."""
    return path_plan(current, cfg['named_poses']['observe_v1'], cfg)
