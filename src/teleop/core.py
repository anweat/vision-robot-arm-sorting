"""One-step teleoperation. No queues, recovery moves, or automatic retreat.

There is no enable step and there are no limit profiles (docs/atlas-200i-dk-a2/32-操作逻辑简化与分步任务.md
§1): the first motion command of a client takes control if nobody else holds it, the client keeps it with
heartbeats (a dead-man lease of LEASE_S), and `stop` from anyone ends it. Speed is one number in deg/s.
Stick driving (`drive`) is a loop of short verified segments that ends when the stick updates stop.

When a step meets resistance (a joint lags its command beyond the tolerance), the arm is
commanded to hold at its readback so no servo keeps pushing into an obstacle; this is not a
retreat. The gripper and Cartesian steps go through the same checks as joint jogs.

Relative steps build on the last command while the readback still agrees with it (within the
readback tolerance), not on the readback itself: a 1 deg change can sit inside a servo's
deadband (2026-09-24: J1 91 -> read 90, twice), and building on the readback would resend the
same command forever. If someone moves the arm by hand, the readback disagrees and wins."""
import hashlib
import json
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'adapters'))
import safe_arm


class PlanRejected(safe_arm.MotionError):
    """A plan failed its checks before any command was sent; the operator keeps control."""


class RevisionConflict(ValueError):
    """客户端带的 revision 与控制台当前 revision 不一致：说明有人（或另一页面）在此
    期间动过臂。调用方（网关/前端）应重读 revision 再重试，而不是把它当普通错误。"""


class SimArm:
    def __init__(self):
        self.pose = [90, 105, 5, 5, 95, 30]

    def read(self):
        return self.pose[:]

    def write6(self, pose, duration_ms):
        self.pose = list(pose)


LEASE_S = 2.0           # control lapses this long after the last command or heartbeat
DRIVE_TIMEOUT_S = 0.3   # stick driving stops this long after the last stick update
DRIVE_SETTLE_MS = 60


class Controller:
    def __init__(self, directory, arm=None, hardware=False, camera=None, default_profile=None):
        # default_profile is accepted and ignored: callers from before the profiles were removed still pass it
        from motion_policy import runtime_config, speed_range
        self.base_cfg = safe_arm.load_config(safe_arm.DEFAULT_CONFIG)
        self.cfg = runtime_config(self.base_cfg)
        self.speed_limits = speed_range(self.base_cfg['motion'])
        self.speed = self.speed_limits[2]
        self._apply_speed()
        self.command = None  # last pose sent; relative steps build on it while the readback agrees
        self.home_preview = None
        self.pose_preview = None
        self.config_hash = hashlib.sha256(safe_arm.DEFAULT_CONFIG.read_bytes()).hexdigest()
        self.arm = arm or SimArm()
        self.hardware = hardware
        self.camera = camera
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.gate = threading.RLock()
        self.io = threading.Lock()
        self.deadline = 0
        self.owner = None
        self.busy = False
        self.error = ''
        self.pose = [None] * 6
        self.observed_at = 0
        self.target = None
        self.revision = 0       # bumped on every change; reported, no longer required from clients
        self.grip = None        # last gripper action and whether the readback says it holds something
        self.on_step = None     # callback(kind, goal, observed) after every verified step (recording)
        self.drive_cmd = None   # latest stick update: dict(owner, mode, v, wrist, at)
        self.driving = False
        self.drive_reason = ''
        self.time_scale = 1.0   # simulation only: segment times and waits are divided by this (sim_server --speed)
        self.refresh()

    # ---- bookkeeping ------------------------------------------------------------------------------------
    def log(self, event, **data):
        with self.gate:
            with (self.directory / 'events.jsonl').open('a') as f:
                f.write(json.dumps(dict(t=time.time(), event=event, mode=self.mode,
                                       config_sha256=self.config_hash, speed=self.speed, **data),
                                   ensure_ascii=False, allow_nan=False) + '\n')

    @property
    def mode(self):
        return 'hardware' if self.hardware else 'simulation'

    def _apply_speed(self):
        from motion_policy import motion_for
        self.cfg['motion'] = motion_for(self.speed, self.base_cfg['motion'])

    def refresh(self):
        if not self.io.acquire(blocking=False):
            return
        try:
            pose = self.arm.read()
            if len(pose) != 6 or any(v is None or not math.isfinite(v) for v in pose):
                raise ValueError('关节回读失败；禁止继续动作')
            with self.gate:
                self.pose, self.observed_at = list(pose), time.time()
        except Exception as exc:
            with self.gate:
                self.error = str(exc)
                self.deadline = 0
                self.observed_at = 0
        finally:
            self.io.release()

    def status(self):
        with self.gate:
            valid = len(self.pose) == 6 and all(v is not None and math.isfinite(v) for v in self.pose)
            fk = safe_arm.workspace.kin.forward(self.pose) if valid else None
            # always report the standard keep-out model's margin, also when the runtime only reports it
            margin = safe_arm.workspace.clearance(self.pose, safe_arm.workspace.load_keep_out(self.base_cfg)) if valid else None
            held = time.monotonic() < self.deadline
            lo, hi, default = self.speed_limits
            return dict(mode=self.mode, pose=self.pose[:], target=self.target, observed_at=self.observed_at,
                        grip=self.grip, gripper=self.cfg.get('gripper'), command=self.command,
                        age_s=time.time()-self.observed_at if self.observed_at else None,
                        armed=held, owner=self.owner if held else None, busy=self.busy, error=self.error,
                        revision=self.revision, fk=fk, clearance_cm=margin*100 if valid else None,
                        limits=self.cfg['soft_limits_deg'], session=self.directory.name,
                        speed=dict(value=self.speed, min=lo, max=hi, default=default,
                                   step_deg=self.cfg['motion']['max_step_deg'],
                                   segment_ms=self.cfg['motion']['segment_ms'], settle_ms=self.cfg['motion']['settle_ms']),
                        drive=dict(active=self.driving, mode=(self.drive_cmd or {}).get('mode'), reason=self.drive_reason),
                        runtime=dict(enforce_platform=self.base_cfg.get('runtime', {}).get('enforce_platform', False),
                                     enforce_table=self.base_cfg.get('runtime', {}).get('enforce_table', False)),
                        keep_out=safe_arm.workspace.load_keep_out(self.cfg),
                        standard_keep_out=safe_arm.workspace.load_keep_out(self.base_cfg))

    # ---- control (no enable step) --------------------------------------------------------------------
    def _claim(self, owner):
        """Give `owner` control unless someone else holds a live lease. Caller holds the gate."""
        if not isinstance(owner, str) or not owner:
            raise ValueError('操作端标识无效')
        now = time.monotonic()
        if self.owner not in (None, owner) and now < self.deadline:
            raise ValueError('另一端持有操作权；对方停止或超时后再试，或点“接管”')
        if self.owner != owner or now >= self.deadline:
            self.revision += 1
            self.log('control', owner=owner)
        self.owner, self.deadline = owner, now + LEASE_S

    def enable(self, owner, onsite=None):
        """Kept for clients from before doc 32: the same as taking control. `onsite` is ignored."""
        with self.gate:
            if self.busy and self.owner != owner:
                raise ValueError('动作尚未结束')
            self._claim(owner)
            self.error = ''

    def take_over(self, owner):
        """Stop whoever holds control, then hold it."""
        self.stop()
        with self.gate:
            self._claim(owner)
            self.error = ''

    def heartbeat(self, owner):
        with self.gate:
            if self.owner == owner and time.monotonic() < self.deadline:
                self.deadline = time.monotonic()+LEASE_S

    def stop(self):
        with self.gate:
            self.deadline = 0
            self.owner = None
            self.drive_cmd = None
            self.revision += 1
            self.log('stop_future_commands')

    def set_speed(self, speed, owner=None):
        lo, hi, _ = self.speed_limits
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed):
            raise ValueError('速度必须是数字（°/s）')
        if not lo <= speed <= hi:
            raise ValueError(f'速度只允许 {lo}–{hi} °/s')
        with self.gate:
            self.speed = float(speed)
            self._apply_speed()
            self.revision += 1
            self.log('speed', by=owner)
        return self.status()

    # ---- moves --------------------------------------------------------------------------------------
    def plan_goto(self, target=None, named=None):
        """Check the whole path to an absolute pose (None = keep that joint) or a named pose; nothing is sent."""
        from motion_policy import path_plan
        if named is not None:
            poses = self.cfg.get('named_poses', {})
            if named not in poses:
                raise ValueError(f'没有命名位 {named}')
            target = poses[named]
        if not isinstance(target, list) or len(target) not in (5, 6) or \
                any(v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))) for v in target):
            raise ValueError('目标姿态必须是 5 或 6 个角度（null 表示不动）')
        target = list(target) + [None] * (6 - len(target))
        with self.gate:
            if self.busy or time.time()-self.observed_at > 2:
                raise ValueError('需要静止且新鲜的回读')
            plan = path_plan(self.pose, target, self.cfg)
            plan['revision'] = self.revision
            plan['request'] = target           # None = keep: resolved against the last command when it runs
            return plan

    def goto(self, owner, target=None, named=None):
        """Go to an absolute or named pose in one call: the path is checked whole before the first segment."""
        plan = self.plan_goto(target, named)
        self.log('goto', plan=dict(plan, segments=len(plan['segments'])))
        step = self.cfg['motion']['max_step_deg']

        def segments(base):
            near = lambda q: all(abs(a-b) <= 2 for a, b in zip(q[:5], plan['start'][:5]))
            if not (near(base) or near(self.fresh_readback)):
                raise ValueError('姿态已改变，请重试')
            # a joint not asked for keeps its last command, not its readback: a gripper closed with 133 reads
            # back 129 on a block, and sending 129 would loosen the grip (2026-11-18, simulator: block dropped)
            target = [b if t is None else int(t) for b, t in zip(base, plan['request'])]
            return safe_arm.plan_segments(base, target, step)
        return self._move(owner, plan=segments, kind='goto')

    # previews kept for the old console page and console_step.Session; they are goto underneath
    def plan_home(self):
        with self.gate:
            self.home_preview = self.plan_goto(named='observe_v1')
            self.log('home_preview', plan=dict(self.home_preview, segments=len(self.home_preview['segments'])))
            return self.home_preview

    def home(self, owner, plan_id=None, revision=None):
        return self.goto(owner, named='observe_v1')

    def plan_pose(self, target):
        if not isinstance(target, list) or len(target) < 5 or any(type(v) is not int for v in target[:5]):
            raise ValueError('目标姿态必须是 5 个整数角度')
        with self.gate:
            self.pose_preview = self.plan_goto(list(target[:5]) + [None])
            if not self.pose_preview['segments']:
                self.pose_preview = None
                raise ValueError('目标与当前姿态相同')
            self.pose_preview['segments'] = len(self.pose_preview['segments'])
            self.log('pose_preview', plan=self.pose_preview)
            return self.pose_preview

    def move_to_pose(self, owner, plan_id, revision=None):
        with self.gate:
            p = self.pose_preview
            if not p or p['id'] != plan_id or time.time()-p['created_at'] > 30:
                raise ValueError('姿态预览过期，请重新预览')
            self.pose_preview = None
        return self.goto(owner, p['target'][:5] + [None])

    def jog(self, owner, joint, delta, revision=None):
        if type(joint) is not int or joint not in range(1, 7) or type(delta) is not int or not 1 <= abs(delta) <= 30:
            raise ValueError('只接受 1–6 号关节、±1–30° 的整数点动')
        return self._move(owner, joint=joint, delta=delta)

    def cartesian(self, owner, axis, amount, revision=None):
        """Move the gripper tip along a base axis (cm) or change its pitch (deg) about the tip,
        keeping the other of the two; joints come from the planar IK of the same model."""
        from motion_policy import cartesian_goal
        if axis not in ('x', 'y', 'z', 'pitch') or isinstance(amount, bool) or not isinstance(amount, (int, float)):
            raise ValueError('直角点动参数无效')
        top = 15 if axis == 'pitch' else 5
        if not 0 < abs(amount) <= top:
            raise ValueError(f'步长须在 0–{top}' + ('°' if axis == 'pitch' else ' cm'))
        return self._move(owner, plan=lambda current: cartesian_goal(current, axis, amount, self.cfg), kind='cartesian')

    def gripper(self, owner, action, revision=None):
        g = self.cfg.get('gripper')
        if not g:
            raise ValueError('配置中没有夹爪标定')
        targets = {'wide': g['open_wide_deg'], 'open': g['open_deg'], 'grip': g['grip_command_deg']}
        if action not in targets:
            raise ValueError('夹爪动作只允许 wide / open / grip')
        status = self._move(owner, plan=lambda current: [current[:5] + [targets[action]]],
                            kind='gripper', skip=[6] if action == 'grip' else [])
        reading = status['pose'][5]
        result = safe_arm.grip_state(reading, g) if action == 'grip' else None
        with self.gate:
            self.grip = dict(action=action, command=targets[action], readback=reading, at=time.time(),
                             holding=result == 'holding', result=result)
            self.log('gripper', **self.grip)
        return self.status()

    # ---- stick driving --------------------------------------------------------------------------------
    def drive(self, owner, mode, v, wrist=0):
        """Latest stick deflection (doc 32 §1.5). Starts the drive loop if it is not running; the loop stops
        on its own DRIVE_TIMEOUT_S after the last update, on all-zero deflection, and on stop."""
        if mode not in ('cartesian', 'joint'):
            raise ValueError('mode 只允许 cartesian / joint')
        if not isinstance(v, list) or len(v) != 4 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                                                          or not -1 <= x <= 1 for x in v + [wrist]):
            raise ValueError('v 须为 4 个 −1..1 的数，wrist 为 −1..1')
        with self.gate:
            if self.busy and not self.driving:
                raise ValueError('正在执行别的动作；等它结束')
            self._claim(owner)
            self.drive_cmd = dict(owner=owner, mode=mode, v=[float(x) for x in v], wrist=float(wrist), at=time.monotonic())
            start = not self.driving and any(abs(x) > 1e-3 for x in v + [wrist])
            if start:
                self.driving = True
                self.drive_reason = ''
        if start:
            threading.Thread(target=self._drive_loop, args=(owner,), daemon=True).start()
        return self.status()

    def _drive_loop(self, owner):
        from motion_policy import drive_goal
        residual = [0.0] * 5
        self.log('drive_start', owner=owner)
        try:
            while True:
                with self.gate:
                    cmd = self.drive_cmd
                    if not cmd or cmd['owner'] != owner or self.owner != owner \
                            or time.monotonic() - cmd['at'] > DRIVE_TIMEOUT_S \
                            or not any(abs(x) > 1e-3 for x in cmd['v'] + [cmd['wrist']]):
                        break
                    speed = self.speed
                dt = max(.2, 1.0 / speed)

                def segment(base, cmd=cmd, dt=dt, speed=speed):
                    goal = drive_goal(base, cmd['mode'], cmd['v'], cmd['wrist'], dt, speed, residual, self.cfg)
                    return [goal] if goal != list(base) else []
                try:
                    self._move(owner, plan=segment, kind='drive', timing=dt, claim=False)
                    self.drive_reason = ''
                except PlanRejected as exc:
                    self.drive_reason = str(exc).replace('计划被拒绝，未发送任何指令：', '')
                    residual[:] = [0.0] * 5
                    time.sleep(.1)
                except Exception as exc:
                    self.drive_reason = str(exc)
                    break
        finally:
            with self.gate:
                self.driving = False
                self.drive_cmd = None
            self.log('drive_end', owner=owner, reason=self.drive_reason)

    def _move(self, owner, revision=None, joint=None, delta=None, plan=None, kind='jog', skip=(), timing=None, claim=True):
        """`timing`: segment length in s for drive segments (their settle is short); else the speed's timing.
        `claim=False` (the drive loop): only an owner whose lease is still alive may move, so a stop that lands
        between two drive segments is never undone by the next segment taking control again."""
        with self.gate:
            if self.busy:
                raise ValueError('正在运动；指令不排队')
            if claim:
                self._claim(owner)
            elif self.owner != owner or time.monotonic() >= self.deadline:
                raise ValueError('已停止或操作权超时')
            if self.hardware and not self.camera.fresh():
                self.deadline = 0
                raise ValueError('画面过期，已锁定控制')
            self.busy = True
            self.revision += 1
        try:
            with self.io:
                current = self.arm.read()
                if len(current) != 6 or any(v is None or not math.isfinite(v) for v in current):
                    raise ValueError('关节回读失败')
                motion = self.cfg['motion']
                tolerance = motion['readback_tolerance_deg']
                synced = self.command is not None and all(abs(c-r) <= tolerance for c, r in zip(self.command, current))
                base = list(self.command) if synced else list(current)
                if plan:
                    self.fresh_readback = list(current)  # what this very move just read, for plans that check it
                    try:
                        goals = plan(base)
                    except ValueError as exc:  # unreachable / IK jump: nothing was sent
                        raise PlanRejected(f'计划被拒绝，未发送任何指令：{exc}') from exc
                else:
                    goal = list(base)
                    goal[joint-1] += delta
                    goals = safe_arm.plan_segments(base, goal, motion['max_step_deg']) or [goal]
                if not goals:
                    return self.status()
                # check the whole chain before sending anything, so a later segment cannot strand the arm midway;
                # a start already inside an enforced keep-out may only move so the margin does not get worse
                previous = list(current)
                for goal in goals:
                    try:
                        safe_arm.validate_target(previous, goal, self.cfg)
                        safe_arm.check_keep_out(previous, goal, self.cfg, recover=True)
                    except safe_arm.MotionError as exc:
                        hint = '；保持当前俯角无法到达，可先调整俯仰或前后位置' if kind == 'cartesian' else ''
                        raise PlanRejected(f'计划被拒绝，未发送任何指令：{exc}{hint}') from exc
                    previous = goal
                for goal in goals:
                    safe_arm.validate_target(current, goal, self.cfg)
                    margin = safe_arm.check_keep_out(current, goal, self.cfg, recover=True)
                    motion = self.cfg['motion']          # a speed change applies from the next segment
                    if timing:
                        largest = max(abs(a-b) for a, b in zip(goal[:5], current[:5]))
                        seg_ms = int(max(timing*1000, 1000*largest/self.speed))
                        settle_ms = DRIVE_SETTLE_MS
                    else:
                        seg_ms, settle_ms = motion['segment_ms'], motion['settle_ms']
                    with self.gate:
                        if self.owner != owner or time.monotonic() >= self.deadline:
                            raise ValueError('已停止或操作权超时')
                        if self.hardware and not self.camera.fresh():
                            raise ValueError('画面过期')
                        self.target = goal
                        self.log('command', current=current, target=goal, margin_cm=margin*100, kind=kind)
                        self.arm.write6(goal, int(seg_ms / self.time_scale))
                        self.command = list(goal)
                    time.sleep((seg_ms+settle_ms)/1000/self.time_scale)
                    observed = self.arm.read()
                    if len(observed) != 6 or any(v is None or not math.isfinite(v) for v in observed):
                        raise ValueError('动作后回读失败；停止后续指令，不自动退回')
                    problems = safe_arm.check_reached(observed, goal, tolerance, skip)
                    with self.gate:
                        self.pose, self.observed_at = list(observed), time.time()
                    self.log('readback', target=goal, observed=observed, problems=problems, kind=kind)
                    if problems:
                        # hold where the arm actually is, so no servo keeps pushing into the obstacle
                        hold = [o if o is not None else g for o, g in zip(observed, goal)]
                        self.arm.write6(hold, int(motion['segment_ms'] / self.time_scale))
                        self.command = list(hold)
                        self.log('hold_at_readback', hold=hold)
                        raise ValueError('遇到阻力或偏差，已停在当前读回位置：'+'; '.join(problems))
                    current = list(observed)
                    if self.on_step:
                        self.on_step(kind, goal, observed)
        except PlanRejected as exc:
            self.log('plan_rejected', error=str(exc))
            raise
        except Exception as exc:
            with self.gate:
                self.error, self.deadline = str(exc), 0
                self.observed_at = 0
                self.log('fault', error=str(exc))
            raise
        finally:
            with self.gate:
                self.busy = False
        return self.status()
