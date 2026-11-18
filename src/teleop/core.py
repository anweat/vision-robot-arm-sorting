"""One-step teleoperation. No queues, recovery moves, or automatic retreat.

When a step meets resistance (a joint lags its command beyond the tolerance), the arm is
commanded to hold at its readback so no servo keeps pushing into an obstacle; this is not a
retreat. The gripper and Cartesian steps go through the same checks as joint jogs.

Relative steps build on the last command while the readback still agrees with it (within the
readback tolerance), not on the readback itself: a 1 deg change can sit inside a servo's
deadband (2026-09-24: J1 91 -> read 90, twice), and building on the readback would resend the
same command forever. If someone moves the arm by hand, the readback disagrees and wins."""
import copy
import hashlib
import json
import math
import secrets
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


class Controller:
    def __init__(self, directory, arm=None, hardware=False, camera=None, default_profile='normal'):
        from motion_policy import profile_config
        self.base_cfg = safe_arm.load_config(safe_arm.DEFAULT_CONFIG)
        self.default_profile = default_profile
        self.cfg = profile_config(default_profile, self.base_cfg)
        self.profile = default_profile
        self.profile_until = 0
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
        self.revision = 0
        self.grip = None        # last gripper action and whether the readback says it holds something
        self.on_step = None     # callback(kind, goal, observed) after every verified step (recording)
        self.refresh()

    def log(self, event, **data):
        with self.gate:
            with (self.directory / 'events.jsonl').open('a') as f:
                f.write(json.dumps(dict(t=time.time(), event=event, mode=self.mode,
                                       config_sha256=self.config_hash, profile=self.profile, **data), ensure_ascii=False, allow_nan=False) + '\n')

    @property
    def mode(self):
        return 'hardware' if self.hardware else 'simulation'

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
            self.expire_profile()
            valid = len(self.pose) == 6 and all(v is not None and math.isfinite(v) for v in self.pose)
            fk = safe_arm.workspace.kin.forward(self.pose) if valid else None
            # always report the standard keep-out model, also in profiles that do not enforce it
            margin = safe_arm.workspace.clearance(self.pose, safe_arm.workspace.load_keep_out(self.base_cfg)) if valid else None
            return dict(mode=self.mode, pose=self.pose[:], target=self.target, observed_at=self.observed_at,
                        grip=self.grip, gripper=self.cfg.get('gripper'), command=self.command,
                        default_profile=self.default_profile,
                        age_s=time.time()-self.observed_at if self.observed_at else None,
                        armed=time.monotonic() < self.deadline, busy=self.busy, error=self.error,
                        revision=self.revision, fk=fk, clearance_cm=margin*100 if valid else None,
                        limits=self.cfg['soft_limits_deg'], session=self.directory.name,
                        profile=self.profile, profile_remaining_s=max(0,self.profile_until-time.monotonic()),
                        keep_out=safe_arm.workspace.load_keep_out(self.cfg))

    def enable(self, owner, onsite=False):
        with self.gate:
            if self.busy:
                raise ValueError('动作尚未结束')
            if self.owner != owner and time.monotonic() < self.deadline:
                raise ValueError('另一页面持有操作权')
            if self.hardware and (onsite is not True or not self.camera or not self.camera.fresh()):
                raise ValueError('实机需要现场安全确认及新鲜画面')
            if time.time() - self.observed_at > 2:
                raise ValueError('回读过期，请等待刷新')
            self.owner, self.deadline, self.error = owner, time.monotonic()+2, ''
            self.revision += 1
            self.log('enable', onsite=onsite)

    def heartbeat(self, owner):
        with self.gate:
            if self.owner == owner and time.monotonic() < self.deadline:
                self.deadline = time.monotonic()+2

    def stop(self):
        with self.gate:
            self.deadline = 0
            self.owner = None
            self.revision += 1
            self.log('stop_future_commands')

    def expire_profile(self):
        from motion_policy import TIMED_PROFILES, profile_config
        if self.profile in TIMED_PROFILES and time.monotonic() >= self.profile_until:
            self.cfg=profile_config(self.default_profile,self.base_cfg)
            self.profile=self.default_profile
            self.deadline=0
            self.revision+=1
            self.log('profile_expired')

    def set_profile(self, owner, boundary, onsite, name=None):
        from motion_policy import TIMED_PROFILES, profile_config
        name = name or ('boundary' if boundary else self.default_profile)
        with self.gate:
            if self.busy:
                raise ValueError('运动中不能切换限位')
            cfg = profile_config(name, self.base_cfg)
            if name in TIMED_PROFILES or (name == 'explore' and self.default_profile != 'explore'):
                if onsite is not True or self.owner != owner or time.monotonic() >= self.deadline:
                    raise ValueError('请先现场确认并使能')
            self.cfg, self.profile = cfg, name
            self.profile_until = time.monotonic()+60 if name in TIMED_PROFILES else 0
            self.revision+=1
            self.home_preview=None
            self.pose_preview=None
            self.log('profile', keep_out=self.cfg['keep_out'], limits=self.cfg['soft_limits_deg'])

    def plan_home(self):
        from motion_policy import home_plan
        with self.gate:
            self.expire_profile()
            if self.busy or self.profile not in ('normal','explore') or time.time()-self.observed_at > 2:
                raise ValueError('归位预览需要标准档或探索档、稳定且新鲜的回读')
            margin=safe_arm.workspace.clearance(self.pose,safe_arm.workspace.load_keep_out(self.base_cfg))
            if margin<0:
                raise ValueError(f'当前在标准禁入区内（余量 {margin*100:.1f} cm），归位路线按标准模型检查；先用小步点动移出（通常是加大 J3/J4）再预览归位')
            self.home_preview=home_plan(self.pose,self.base_cfg)  # the path home is always checked against the standard model
            self.home_preview['revision']=self.revision
            self.log('home_preview',plan=self.home_preview)
            return self.home_preview

    def home(self, owner, plan_id, revision):
        with self.gate:
            p=self.home_preview
            if not p or p['id'] != plan_id or time.time()-p['created_at'] > 30 or p['revision']!=revision:
                raise ValueError('归位预览过期，请重新预览')
        self._move(owner,revision,home=p)

    def plan_pose(self, target):
        """Preview a move of J1-J5 to a pose set on the 3D model (J6 keeps its gripper buttons).
        The whole path is checked with the current profile; nothing is sent here."""
        with self.gate:
            self.expire_profile()
            if self.busy or self.profile not in ('normal', 'explore') or time.time()-self.observed_at > 2:
                raise ValueError('姿态模式需要探索档或标准档、稳定且新鲜的回读')
            if not isinstance(target, list) or len(target) < 5 or any(type(v) is not int for v in target[:5]):
                raise ValueError('目标姿态必须是 5 个整数角度')
            start = list(self.pose)
            goal = list(target[:5]) + [start[5]]
            segments = safe_arm.plan_segments(start, goal, self.cfg['motion']['max_step_deg'])
            if not segments:
                raise ValueError('目标与当前姿态相同')
            previous, margins = start, []
            for step in segments:
                try:
                    safe_arm.validate_target(previous, step, self.cfg)
                    margins.append(safe_arm.check_keep_out(previous, step, self.cfg))
                except safe_arm.MotionError as exc:
                    raise ValueError(f'路径第 {len(margins)+1}/{len(segments)} 段不可行：{exc}') from exc
                previous = step
            motion = self.cfg['motion']
            self.pose_preview = dict(id=secrets.token_hex(12), created_at=time.time(), start=start, target=goal,
                                     segments=len(segments), revision=self.revision,
                                     largest_deg=max(abs(a-b) for a, b in zip(start, goal)),
                                     seconds=round(len(segments)*(motion['segment_ms']+motion['settle_ms'])/1000, 1),
                                     margin_cm=round(min(margins)*100, 1) if margins else None)
            self.log('pose_preview', plan=self.pose_preview)
            return self.pose_preview

    def move_to_pose(self, owner, plan_id, revision):
        with self.gate:
            p = self.pose_preview
            if not p or p['id'] != plan_id or time.time()-p['created_at'] > 30 or p['revision'] != revision:
                raise ValueError('姿态预览过期，请重新预览')
            self.pose_preview = None
        step = self.cfg['motion']['max_step_deg']

        def plan(base):
            # the preview starts from the readback, the step from the last command when they agree within
            # tolerance; a sagging J2 can hold them 3 deg apart (2026-09-26), so either may match the preview
            near = lambda q: all(abs(a-b) <= 2 for a, b in zip(q[:5], p['start'][:5]))
            if not (near(base) or near(self.fresh_readback)):
                raise ValueError('姿态已改变，请重新预览')
            return safe_arm.plan_segments(base, p['target'][:5]+[base[5]], step)
        return self._move(owner, revision, plan=plan, kind='pose')

    def jog(self, owner, joint, delta, revision):
        if type(joint) is not int or joint not in range(1, 7) or type(delta) is not int or abs(delta) not in (1, 2, 5, 10):
            raise ValueError('只接受 1–6 号关节、±1/2/5/10° 点动')
        if abs(delta) > 2 and self.profile not in ('normal', 'explore'):
            raise ValueError('抓取档、脱困档只允许 2° 以内的关节点动')
        return self._move(owner,revision,joint=joint,delta=delta)

    def cartesian(self, owner, axis, amount, revision):
        """Move the gripper tip along a base axis (cm) or change its pitch (deg) about the tip,
        keeping the other of the two; joints come from the planar IK of the same model."""
        from motion_policy import cartesian_goal
        if axis not in ('x', 'y', 'z', 'pitch') or isinstance(amount, bool) or not isinstance(amount, (int, float)):
            raise ValueError('直角点动参数无效')
        # pitch turns about the tip; with ~19 cm from wrist to tip, 5 deg already swings J3 by ~30 deg
        # large pitch steps are still bounded by the IK jump check (15 deg standard, 30 deg explore)
        allowed = (0.5, 1, 2, 3) if axis != 'pitch' else (1, 2, 5)
        if abs(amount) not in allowed:
            raise ValueError(f'步长只允许 {allowed}')
        if self.profile not in ('normal', 'explore') and abs(amount) > 1:
            raise ValueError('抓取档、脱困档只允许 1 cm / 1° 以内的直角点动')
        return self._move(owner, revision, plan=lambda current: cartesian_goal(current, axis, amount, self.cfg), kind='cartesian')

    def gripper(self, owner, action, revision):
        g = self.cfg.get('gripper')
        if not g:
            raise ValueError('配置中没有夹爪标定')
        targets = {'wide': g['open_wide_deg'], 'open': g['open_deg'], 'grip': g['grip_command_deg']}
        if action not in targets:
            raise ValueError('夹爪动作只允许 wide / open / grip')
        status = self._move(owner, revision, plan=lambda current: [current[:5] + [targets[action]]],
                            kind='gripper', skip=[6] if action == 'grip' else [])
        reading = status['pose'][5]
        result = safe_arm.grip_state(reading, g) if action == 'grip' else None
        with self.gate:
            self.grip = dict(action=action, command=targets[action], readback=reading, at=time.time(),
                             holding=result == 'holding', result=result)
            self.log('gripper', **self.grip)
        return self.status()

    def _move(self,owner,revision,joint=None,delta=None,home=None,plan=None,kind='jog',skip=()):
        with self.gate:
            self.expire_profile()
            if self.profile == 'boundary' and (home or plan or abs(delta)!=1):
                raise ValueError('临时测试档只允许 1° 关节点动，不能归位')
            if self.profile not in ('normal', 'explore') and home:
                raise ValueError('抓取档和脱困档不能归位；先恢复默认档')
            if self.busy or self.owner != owner or time.monotonic() >= self.deadline:
                raise ValueError('未使能、连接超时或正在运动；指令不排队')
            if revision != self.revision:
                raise RevisionConflict('状态已变化，请刷新后重新点动')
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
                tolerance=self.cfg['motion']['readback_tolerance_deg']
                synced=self.command is not None and all(abs(c-r)<=tolerance for c,r in zip(self.command,current))
                base=list(self.command) if synced else list(current)
                check_cfg=self.base_cfg if home else self.cfg
                if home:
                    if any(abs(a-b)>1 for a,b in zip(current,home['start'])):
                        raise ValueError('姿态改变，请重新预览归位')
                    from motion_policy import home_plan
                    goals=home_plan(current,self.base_cfg)['segments']
                elif plan:
                    self.fresh_readback=list(current)  # what this very move just read, for plans that check it
                    try:
                        goals=plan(base)
                    except ValueError as exc:  # unreachable / IK jump: nothing was sent
                        raise PlanRejected(f'计划被拒绝，未发送任何指令：{exc}') from exc
                else:
                    goal=list(base)
                    goal[joint-1]+=delta
                    # a 10 deg jog runs as 5 deg segments, each verified by readback like any other plan
                    goals=safe_arm.plan_segments(base,goal,self.cfg['motion']['max_step_deg']) or [goal]
                # check the whole chain before sending anything, so a later segment cannot strand the arm midway
                previous=list(current)
                for goal in goals:
                    try:
                        safe_arm.validate_target(previous,goal,check_cfg)
                        safe_arm.check_keep_out(previous,goal,check_cfg,recover=self.profile=='recover')
                    except safe_arm.MotionError as exc:
                        hint='；保持当前俯角无法到达，可先调整俯仰或前后位置' if kind=='cartesian' else ''
                        raise PlanRejected(f'计划被拒绝，未发送任何指令：{exc}{hint}') from exc
                    previous=goal
                for goal in goals:
                    safe_arm.validate_target(current,goal,check_cfg)
                    margin=safe_arm.check_keep_out(current,goal,check_cfg,recover=self.profile=='recover')
                    with self.gate:
                        self.expire_profile()
                        if self.owner!=owner or time.monotonic()>=self.deadline:
                            raise ValueError('已停止或操作权超时')
                        if self.hardware and not self.camera.fresh():
                            raise ValueError('画面过期')
                        self.target=goal
                        self.log('command',current=current,target=goal,margin_cm=margin*100)
                        self.arm.write6(goal,self.cfg['motion']['segment_ms'])
                        self.command=list(goal)
                    time.sleep((self.cfg['motion']['segment_ms']+self.cfg['motion']['settle_ms'])/1000)
                    observed=self.arm.read()
                    if len(observed)!=6 or any(v is None or not math.isfinite(v) for v in observed):
                        raise ValueError('动作后回读失败；停止后续指令，不自动退回')
                    problems=safe_arm.check_reached(observed,goal,self.cfg['motion']['readback_tolerance_deg'],skip)
                    with self.gate:
                        self.pose,self.observed_at=list(observed),time.time()
                    self.log('readback',target=goal,observed=observed,problems=problems,kind=kind)
                    if problems:
                        # hold where the arm actually is, so no servo keeps pushing into the obstacle
                        hold=[o if o is not None else g for o,g in zip(observed,goal)]
                        self.arm.write6(hold,self.cfg['motion']['segment_ms'])
                        self.command=list(hold)
                        self.log('hold_at_readback',hold=hold)
                        raise ValueError('遇到阻力或偏差，已停在当前读回位置：'+'; '.join(problems))
                    current=list(observed)
                    if self.on_step:
                        self.on_step(kind,goal,observed)
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
