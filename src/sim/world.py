"""The simulated table: 30 mm blocks on the map, and an arm that can pick them up.

Nothing here is measured on the real arm. Geometry is the project's own model (dofbot_kinematics,
config/calibration/hand_eye.json, the map placement in config/map_a4_v2.json); the grasp rule is a
stand-in for contact physics, chosen so that the readbacks the console classifies (129 holding, 132
closed on nothing; config/arm_limits.json "gripper") come out as they did on the real gripper.

What is modelled
  * blocks rest flat on the table, on another block or on a prop (the floor of a bin), are carried
    rigidly by the gripper (they turn with J5), and drop straight down when it opens
  * a grip closes on a block when the kinematic tip is over it and low enough
  * the printed map is whichever config/map_*.json is chosen, or none; its placement gives the table
  * props are things built from boxes that stand on the table: bins, or plain boxes

  * (optional, off by default) a hidden offset on joints 1-5: the arm stands where command + offset puts it
    while its readback still says "command", the way backlash makes the real forward kinematics wrong by
    about a centimetre; and a record of where the gripper or the carried block entered a prop
  * a block let go over a bin that does not fit inside the opening perches on the rim instead of dropping in

What is not: pushing, toppling, blocks colliding with each other, servo lag.

All poses are in the arm base frame, metres.
"""
from __future__ import annotations

import json
import math
import random
import sys
import threading
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "adapters"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
from map_registration import DEFAULT_MAP, TableMap  # noqa: E402

CATEGORIES = ("recyclable", "kitchen", "hazardous", "other")
COLOURS = {"recyclable": "blue", "kitchen": "green", "hazardous": "red", "other": "yellow"}
FACES_DIR = ROOT / "data" / "reference" / "block_faces"
BLOCK = 0.03
START_POSE = [90, 105, 5, 5, 95, 30]
MAP_X_CM, MAP_Y_CM = (-14.85, 14.85), (0.0, 21.0)
SPAWN_X_CM, SPAWN_Y_CM = (-11.0, 11.0), (4.0, 17.5)     # where a random reset puts blocks
APART_CM = 4.5
GRASP_RADIUS_M = 0.02      # tip to block centre, horizontally
GRASP_ABOVE_TOP_M = 0.005  # the tip may be this far above the block's top face and still close on it
HOLDING_READBACK, EMPTY_READBACK = 129, 132
CONFIG = ROOT / "config"
CONTACT_MARGIN_M = 0.002   # a finger point or block sample this near a prop box counts as touching it
CONTACT_STEPS = 6          # a commanded move is checked at this many poses along the way
MAX_CONTACTS = 200
MAX_BIAS_DEG = 10.0
GLASS_COLOUR = "#c8d8dc"
BIN_COLOURS = {"recyclable": "#3f7fd0", "kitchen": "#4aa35a", "hazardous": "#d0483f", "other": "#d9b53a"}

# local face order of a block: the outward normal of face i is FACE_AXES[i] in the block's own frame
FACE_AXES = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
TOP = 4


def face_items() -> dict[str, list[str]]:
    """category -> the item names of its six faces in local face order: four stickers round the sides,
    one on top, and a plain face in the block's colour underneath (src/vision/synth_blocks.py).
    The real blocks' face arrangement was not recorded, so this order is an assumption."""
    index = json.loads((FACES_DIR / "index.json").read_text())
    out = {}
    for c in CATEGORIES:
        stickers = [e["item"] for e in index if e["category"] == c and not e["colour_face"]]
        colour = next(e["item"] for e in index if e["category"] == c and e["colour_face"])
        out[c] = stickers[:4] + [colour, "plain"]
    return out


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def face_up(face: int) -> np.ndarray:
    """Rotation that turns local face `face` to point straight up."""
    a = FACE_AXES[face]
    if a[2] > 0.5:
        return np.eye(3)
    if a[2] < -0.5:
        return np.diag([1.0, -1.0, -1.0])
    axis = np.cross(a, [0, 0, 1.0])
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + k + k @ k      # 90 degrees about `axis`


def settle(r: np.ndarray) -> np.ndarray:
    """The nearest rotation in which the block lies flat (one face straight up)."""
    face = int(np.argmax(r[2] @ FACE_AXES.T))
    up = r @ FACE_AXES[face]
    axis = np.cross(up, [0, 0, 1.0])
    s, c = np.linalg.norm(axis), up[2]
    if s < 1e-9:
        return r
    axis /= s
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return (np.eye(3) + s * k + (1 - c) * k @ k) @ r


def named_config(name: str, prefix: str) -> Path:
    """config/<name>.json for a bare name starting with `prefix`; nothing outside config/ can be named."""
    if not isinstance(name, str) or not name.startswith(prefix) or not name.replace("_", "").isalnum():
        raise ValueError(f"名称必须是 config/ 下以 {prefix} 开头的文件名（不带 .json）")
    path = CONFIG / f"{name}.json"
    if not path.exists():
        raise ValueError(f"没有 config/{name}.json")
    return path


def colour_bgr(value) -> tuple[int, int, int]:
    if isinstance(value, str) and len(value) == 7 and value[0] == "#":
        try:
            return int(value[5:7], 16), int(value[3:5], 16), int(value[1:3], 16)
        except ValueError:
            pass
    raise ValueError("颜色必须是 #rrggbb")


def bin_parts(inner_cm, height_cm: float, wall_cm: float) -> list[tuple[list[float], list[float]]]:
    """An open-top bin as five boxes, [(centre, half size)] in metres, origin on the table under the
    middle of the opening: a floor `wall_cm` thick and four walls `height_cm` tall above it."""
    w, d, h, t = inner_cm[0] / 200, inner_cm[1] / 200, height_cm / 100, wall_cm / 100
    z = t + h / 2
    return [([0, 0, t / 2], [w + t, d + t, t / 2]),
            ([w + t / 2, 0, z], [t / 2, d + t, h / 2]), ([-w - t / 2, 0, z], [t / 2, d + t, h / 2]),
            ([0, d + t / 2, z], [w, t / 2, h / 2]), ([0, -d - t / 2, z], [w, t / 2, h / 2])]


class World:
    def __init__(self, seed: int = 0, map_name: str | None = DEFAULT_MAP.stem):
        self.lock = threading.RLock()
        self.props: dict[str, dict] = {}
        self.map_revision = 0
        self.set_map(map_name)
        self.items = face_items()
        self.rng = random.Random(seed)
        self.blocks: dict[str, dict] = {}
        self.held: str | None = None
        self.revision = 0
        self.next_id = 1
        self.servos = list(START_POSE)
        self.grip_miss = 0.0        # injected fault: the share of grips that close on nothing
        self.bias = [0.0] * 5       # injected fault: degrees the arm stands off its command, joints 1-5
        self.faults = random.Random(seed)
        self.contacts: list[dict] = []
        self.contact_count = 0

    # ------------------------------------------------------------------ the map and the props

    def set_map(self, name: str | None) -> None:
        """Lay another printed map on the table (a config/map_*.json name), or none. Positions given in
        map cm always use the placement of the map last laid; without a map, the default map's."""
        with self.lock:
            path = DEFAULT_MAP if name in (None, "none") else named_config(name, "map_")
            try:
                table = TableMap(path)
                data = json.loads(path.read_text())
            except (KeyError, ValueError) as exc:
                raise ValueError(f"config/{path.name} 不是可用的地图配置：缺少 {exc}") from exc
            self.table, self.table_z = table, float(table.offset[2])
            self.map_name = None if name in (None, "none") else name
            self.map_data = data if self.map_name else None
            self.map_revision += 1
            self.revision = getattr(self, "revision", 0) + 1

    def set_faults(self, grip_miss=None, servo_bias_deg=None) -> dict:
        """Make the simulated arm fail on purpose, so the failure paths have something to handle.
        `grip_miss`: share of grips that close on nothing. `servo_bias_deg`: five numbers, degrees the arm
        really stands off what it reads back (joints 1-5); the readback and so the pipeline's forward
        kinematics stay unaware of it."""
        if grip_miss is None and servo_bias_deg is None:
            raise ValueError("grip_miss 必须是 0–1 的数：合爪夹空的比例")
        if grip_miss is not None and (isinstance(grip_miss, bool) or not isinstance(grip_miss, (int, float))
                                      or not 0 <= grip_miss <= 1):
            raise ValueError("grip_miss 必须是 0–1 的数：合爪夹空的比例")
        if servo_bias_deg is not None and (
                not isinstance(servo_bias_deg, (list, tuple)) or len(servo_bias_deg) != 5
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                       or abs(v) > MAX_BIAS_DEG for v in servo_bias_deg)):
            raise ValueError(f"servo_bias_deg 必须是 5 个绝对值不超过 {MAX_BIAS_DEG:g} 的数（关节 1–5，度）")
        with self.lock:
            if grip_miss is not None:
                self.grip_miss = float(grip_miss)
            if servo_bias_deg is not None:
                self.bias = [float(v) for v in servo_bias_deg]
                self.revision += 1
            return self.fault_state()

    def fault_state(self) -> dict:
        out = dict(grip_miss=self.grip_miss)
        if any(self.bias):
            out["servo_bias_deg"] = list(self.bias)
        return out

    @property
    def actual(self) -> list[float]:
        """Where the joints really stand: the command plus the injected offset."""
        return [s + b for s, b in zip(self.servos[:5], self.bias)] + list(self.servos[5:])

    def matches_default_map(self) -> bool:
        """Whether every code on the table is where the default map (the one the console's localization
        assumes) has it; a map with some codes left out still matches."""
        with self.lock:
            if not self.map_name:
                return False
            standard = TableMap()
            return bool(self.table.layout) and np.allclose(self.table.offset, standard.offset) and all(
                m in standard.layout and np.allclose(c, standard.layout[m]) for m, c in self.table.layout.items())

    def add_prop(self, kind: str, map_cm, yaw_deg: float = 0.0, name: str = "", category: str | None = None,
                 colour: str | None = None, inner_cm=(8.0, 8.0), height_cm: float = 4.0, wall_cm: float = 0.2,
                 size_cm=(5.0, 5.0, 5.0), prop_id: str | None = None) -> str:
        """Stand a prop on the table. kind 'bin': an open box with the given inner opening, wall height
        and wall thickness; `category` marks which garbage it is for. kind 'box': a solid `size_cm`."""
        try:
            x, y, yaw_deg = float(map_cm[0]), float(map_cm[1]), float(yaw_deg)
            if kind == "bin":
                dims = [float(inner_cm[0]), float(inner_cm[1]), float(height_cm), float(wall_cm)]
                ok = all(1 <= v <= 40 for v in dims[:2]) and 0.5 <= dims[2] <= 30 and 0.1 <= dims[3] <= 2
                parts = bin_parts(dims[:2], dims[2], dims[3]) if ok else None
            elif kind == "box":
                dims = [float(v) for v in size_cm]
                ok = len(dims) == 3 and all(0.2 <= v <= 40 for v in dims)
                parts = [([0, 0, dims[2] / 200], [v / 200 for v in dims])] if ok else None
            else:
                raise ValueError("物体种类只有 bin（垃圾桶）和 box（实心盒）")
        except (TypeError, IndexError) as exc:
            raise ValueError("位置必须是 [x_cm, y_cm]，尺寸必须是数字") from exc
        if not ok or not all(map(math.isfinite, (x, y, yaw_deg))) or abs(x) > 60 or not -30 <= y <= 60:
            raise ValueError("尺寸或位置超出范围")
        if category is not None and category not in CATEGORIES:
            raise ValueError(f"类别必须是 {'/'.join(CATEGORIES)}")
        colour = colour or BIN_COLOURS.get(category, "#9aa7b0")
        colour_bgr(colour)
        with self.lock:
            if prop_id is None:
                prop_id = f"p{self.next_id}"
                self.next_id += 1
            pose = np.eye(4)
            pose[:3, :3] = rot_z(yaw_deg)
            pose[:3, 3] = self.table.map_to_base([x / 100, y / 100, 0])
            self.props[prop_id] = dict(kind=kind, name=str(name)[:40], category=category, colour=colour, pose=pose,
                                       parts=parts, dims=dims)
            self.revision += 1
            return prop_id

    def remove_prop(self, prop_id: str) -> None:
        with self.lock:
            if prop_id not in self.props:
                raise ValueError("没有这个物体")
            del self.props[prop_id]
            self.revision += 1

    def load_bins(self, name: str, jitter_mm: float = 0.0, jitter_deg: float = 0.0, seed: int | None = None,
                  offsets: dict | None = None, only=None) -> list[str]:
        """The bins of a design file (config/bins_*.json, src/tools/make_bins.py): the plate, the four
        pinwheel walls (82 x t, flush at the corners) and the stickers on the plate, at their planned
        places (only the numbers in `only`, if given), each moved by a random placement error of up to `jitter_mm` / `jitter_deg` (a person stood
        them there, not a CNC) and by `offsets` = {number: [dx_mm, dy_mm, dyaw_deg]} when given.
        Acrylic is drawn clear (`glass`), the stickers opaque; design values, not a measured installation."""
        design = json.loads(named_config(name, "bins_").read_text())
        rng = random.Random(seed)
        try:
            g = design["geometry_mm"]
            t, inner = g["thickness"] / 1000, g["inner"] / 1000
            (px, py), (y0, y1) = g["plate"], g["plate_y_range"]
            h, wall_h = inner / 2, g["wall_height"] / 1000
            z_wall, top = t + wall_h / 2, t + wall_h
            parts = [([0, (y0 + y1) / 2000, t / 2], [px / 2000, (y1 - y0) / 2000, t / 2])]
            for x0, x1, ya, yb in ((-h - t, h, -h - t, -h), (h, h + t, -h - t, h), (-h, h + t, h, h + t),
                                   (-h - t, -h, -h, h + t)):
                parts.append(([(x0 + x1) / 2, (ya + yb) / 2, z_wall], [(x1 - x0) / 2, (yb - ya) / 2, wall_h / 2]))
            side = design["side_m"]
            made = []
            with self.lock:
                for old in [pid for pid, p in self.props.items() if p.get("number") is not None]:
                    del self.props[old]             # standing them again replaces the last set
                for number, entry in design["bins"].items():
                    if only is not None and int(number) not in only:
                        continue
                    dx, dy, dyaw = (offsets or {}).get(number, (0.0, 0.0, 0.0))
                    if jitter_mm or jitter_deg:
                        dx += rng.uniform(-jitter_mm, jitter_mm)
                        dy += rng.uniform(-jitter_mm, jitter_mm)
                        dyaw += rng.uniform(-jitter_deg, jitter_deg)
                    x, y = self.table.base_to_map([*entry["planned_centre_base_m"], 0])[:2]
                    pid = self.add_prop("bin", [x * 100 + dx / 10, y * 100 + dy / 10],
                                        entry["planned_flange_direction_deg"] - 90 + dyaw,
                                        name=f"{number} 号桶 {entry['name']}", category=entry["category"],
                                        colour=GLASS_COLOUR, inner_cm=[g["inner"] / 10] * 2,
                                        height_cm=g["wall_height"] / 10, wall_cm=g["thickness"] / 10)
                    prop = self.props[pid]
                    prop["parts"], prop["glass"], prop["number"] = parts, True, int(number)
                    prop["planned"] = dict(map_cm=[round(x * 100, 3), round(y * 100, 3)],
                                           yaw_deg=entry["planned_flange_direction_deg"] - 90)
                    paper = [dict(kind="paper", centre=[0.0, (y1 - 15.5) / 1000], half=[px / 2000 - 0.0005, 0.0155], z=t + 0.00005),
                             dict(kind="paper", centre=[0.0, 0.0], half=[0.0395, 0.0395], z=t + 0.00005)]
                    marks = [dict(kind="marker", centre=[sum(c[0] for c in corners) / 4, sum(c[1] for c in corners) / 4],
                                  half=[side / 2, side / 2], id=int(mid), z=design["marker_plane_above_table_m"])
                             for mid, corners in entry["corners_m"].items()]
                    prop["decals"] = paper + marks
                    made.append(pid)
            return made
        except KeyError as exc:
            raise ValueError(f"config/{name}.json 不是桶的设计文件：缺少 {exc}") from exc

    def prop_under(self, xy, below: float = math.inf) -> tuple[float, str | None]:
        """(height of the highest prop surface under base xy that is not above `below`, the bin whose
        opening xy is in)."""
        top, inside = self.table_z, None
        for pid, p in self.props.items():
            local = p["pose"][:2, :2].T @ (np.asarray(xy, float) - p["pose"][:2, 3])
            for centre, half in p["parts"]:
                surface = p["pose"][2, 3] + centre[2] + half[2]
                if abs(local[0] - centre[0]) <= half[0] and abs(local[1] - centre[1]) <= half[1] and top < surface <= below:
                    top = surface
            if p["kind"] == "bin" and abs(local[0]) <= p["dims"][0] / 200 and abs(local[1]) <= p["dims"][1] / 200:
                inside = pid
        return top, inside

    # ------------------------------------------------------------------ editing the scene

    def reset(self, count: int = 4, seed: int | None = None) -> None:
        """Clear the table and put `count` blocks at random, one category each while they last."""
        if type(count) is not int or not 0 <= count <= 12:
            raise ValueError("物块数量必须是 0–12 的整数")
        with self.lock:
            if seed is not None:
                self.rng = random.Random(seed)
            self.blocks, self.held = {}, None
            spots: list[tuple[float, float]] = []
            for i in range(count):
                for _ in range(200):
                    x, y = self.rng.uniform(*SPAWN_X_CM), self.rng.uniform(*SPAWN_Y_CM)
                    clear = self.prop_under(self.table.map_to_base([x / 100, y / 100, 0])[:2])[0] == self.table_z
                    if clear and all(math.dist((x, y), s) >= APART_CM for s in spots):
                        break
                else:
                    raise ValueError("地图上放不下这么多互不相碰的物块")
                spots.append((x, y))
                self.place(CATEGORIES[i % 4], [x, y], self.rng.uniform(-45, 45), self.rng.randrange(5))
            self.revision += 1

    def place(self, category: str, map_cm, yaw_deg: float = 0.0, up=TOP, block_id: str | None = None) -> str:
        """Put a block flat on the table at a map position (cm). `up` is a face index 0-5 or one of the
        block's item names. With `block_id` an existing block is moved (and let go if it was held)."""
        if category not in CATEGORIES:
            raise ValueError(f"类别必须是 {'/'.join(CATEGORIES)}")
        if isinstance(up, str):
            if up not in self.items[category]:
                raise ValueError(f"{category} 的面只有：{', '.join(self.items[category])}")
            up = self.items[category].index(up)
        if type(up) is not int or not 0 <= up < 6:
            raise ValueError("朝上的面必须是 0–5 或该物块的贴纸名")
        try:
            x, y = float(map_cm[0]), float(map_cm[1])
            yaw_deg = float(yaw_deg)
        except (TypeError, ValueError, IndexError) as exc:
            raise ValueError("位置必须是 [x_cm, y_cm]，朝向是角度") from exc
        if not all(map(math.isfinite, (x, y, yaw_deg))) or abs(x) > 40 or not -10 <= y <= 50:
            raise ValueError("位置超出桌面范围")
        with self.lock:
            if block_id is not None and block_id not in self.blocks:
                raise ValueError("没有这个物块")
            pose = np.eye(4)
            pose[:3, :3] = rot_z(yaw_deg) @ face_up(up)
            pose[:3, 3] = self.table.map_to_base([x / 100, y / 100, 0])
            if block_id is not None:
                del self.blocks[block_id]       # it is lifted off before it is set down again
            pose[2, 3] = self.rest_height(pose[:2, 3])
            if block_id is None:
                block_id = f"b{self.next_id}"
                self.next_id += 1
            if self.held == block_id:
                self.held = None
            self.blocks[block_id] = dict(category=category, pose=pose, grip=None)
            self.revision += 1
            return block_id

    def remove(self, block_id: str) -> None:
        with self.lock:
            if block_id not in self.blocks:
                raise ValueError("没有这个物块")
            del self.blocks[block_id]
            if self.held == block_id:
                self.held = None
            self.revision += 1

    def rest_height(self, xy, ignore: str | None = None, below: float = math.inf) -> float:
        """Centre height of a block set down at base xy: on the table, a prop, or the highest block under
        it. `below` is the height it comes down from; surfaces above that are not landed on."""
        top = self.prop_under(xy, below)[0]
        for bid, b in self.blocks.items():
            if bid != ignore and bid != self.held and np.linalg.norm(b["pose"][:2, 3] - xy) < BLOCK / 2:
                top = max(top, b["pose"][2, 3] + BLOCK / 2)
        return top + BLOCK / 2

    # ------------------------------------------------------------------ the arm acting on it

    def arm_moved(self, servos) -> None:
        """Called with every pose the simulated arm takes."""
        with self.lock:
            before, self.servos = self.servos, list(servos)
            self.check_contacts(before)
            opening = self.servos[5] < before[5]
            closing = self.servos[5] > before[5] and self.servos[5] >= HOLDING_READBACK
            actual = self.actual
            if self.held and opening:
                b = self.blocks[self.held]
                b["pose"][:3, :3] = settle(b["pose"][:3, :3])
                b["pose"][2, 3], b["perched"] = self.landing(b["pose"], self.held)
                b["grip"], self.held = None, None
            elif self.held:
                b = self.blocks[self.held]
                b["pose"] = np.array(kin.servo5_pose(actual[:5])) @ b["grip"]
            elif closing:
                tip = np.array(kin.forward(actual)["tip"])
                best = None
                for bid, b in self.blocks.items():
                    c = b["pose"][:3, 3]
                    d = float(np.linalg.norm(c[:2] - tip[:2]))
                    if d <= GRASP_RADIUS_M and c[2] - BLOCK / 2 - 0.005 <= tip[2] <= c[2] + BLOCK / 2 + GRASP_ABOVE_TOP_M \
                            and (best is None or d < best[0]):
                        best = (d, bid)
                if best and self.faults.random() >= self.grip_miss:
                    self.held = best[1]
                    b = self.blocks[self.held]
                    b["grip"] = np.linalg.inv(np.array(kin.servo5_pose(actual[:5]))) @ b["pose"]
            self.revision += 1

    def landing(self, pose: np.ndarray, block_id: str) -> tuple[float, bool]:
        """(centre height, perched) of a block let go at `pose`. It drops to the highest surface under its
        centre; if one of its corners is over something higher (a bin's rim, say) it hangs on that edge
        instead -- `perched`, which a bin counts as not inside (its footprint is sampled on a 25 x 25 grid (1.3 mm)). Tilting and falling off are not modelled."""
        xy = pose[:2, 3]
        z = self.rest_height(xy, block_id, pose[2, 3])
        half = BLOCK / 2
        # a 2 mm wall lies between a block's corners as often as under one: look over its whole footprint
        reach = [self.prop_under(xy + pose[:2, :2] @ np.array([sx * half, sy * half]), pose[2, 3])[0] + half
                 for sx in np.linspace(-1, 1, 25) for sy in np.linspace(-1, 1, 25)]
        return (max(z, max(reach)), True) if max(reach) > z + 0.0005 else (z, False)

    def check_contacts(self, before) -> None:
        """Note every prop the gripper's fingers or the carried block enter on the way from the previous
        pose to this one (the fingers as points along the last link, the block as a grid inside it). The
        move is not stopped: the record is the sim's answer to "would this have hit the bin?"."""
        if not self.props:
            return
        bias = np.array(self.bias)
        start, goal = np.array(before[:5], float) + bias, np.array(self.servos[:5], float) + bias
        boxes = []
        for pid, p in self.props.items():
            for centre, half in p["parts"]:
                boxes.append((pid, p["pose"][:3, :3], p["pose"][:3, 3] + p["pose"][:3, :3] @ np.asarray(centre, float),
                              np.asarray(half, float) + CONTACT_MARGIN_M))
        held = self.blocks.get(self.held) if self.held else None
        grid = np.array([[i, j, k, 1.0] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]) * \
            np.array([BLOCK * 0.45] * 3 + [1.0])
        seen = set()
        for step in range(1, CONTACT_STEPS + 1):
            q = start + (goal - start) * step / CONTACT_STEPS
            pose = np.array(kin.servo5_pose(list(q)))
            o, x, y, z = pose[:3, 3], pose[:3, 0], pose[:3, 1], pose[:3, 2]
            points = [("finger", o + a * z + u * axis) for a in np.linspace(0.05, kin.GRIPPER, 8)
                      for u in (-0.015, 0.0, 0.015) for axis in (x, y)]
            if held is not None:
                points += [("block", p) for p in (pose @ held["grip"] @ grid.T).T[:, :3]]
            for what, point in points:
                for pid, r, centre, half in boxes:
                    if (pid, what) not in seen and np.all(np.abs(r.T @ (point - centre)) <= half):
                        seen.add((pid, what))
                        self.contact_count += 1
                        self.contacts.append(dict(prop=pid, name=self.props[pid]["name"], what=what,
                                                  at_cm=[round(float(v) * 100, 1) for v in point],
                                                  servos=[round(float(v), 1) for v in self.servos]))
                        del self.contacts[:-MAX_CONTACTS]

    def gripper_readback(self, command: float) -> float:
        with self.lock:
            return min(command, HOLDING_READBACK if self.held else EMPTY_READBACK)

    # ------------------------------------------------------------------ reading it

    def render_list(self) -> tuple[list[float], list[tuple[str, str, np.ndarray]], int]:
        """(where the joints really stand, blocks, revision): the camera sees the arm where it is, not
        where it reads back."""
        with self.lock:
            return (self.actual, [(bid, b["category"], b["pose"].copy()) for bid, b in self.blocks.items()],
                    self.revision)

    def _boxes(self, glass: bool) -> list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]]:
        with self.lock:
            out = []
            for p in self.props.values():
                if bool(p.get("glass")) != glass:
                    continue
                for centre, half in p["parts"]:
                    pose = p["pose"].copy()
                    pose[:3, 3] += pose[:3, :3] @ np.asarray(centre, float)
                    out.append((pose, np.asarray(half, float), colour_bgr(p["colour"])))
            return out

    def solids(self) -> list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]]:
        """Every opaque box of every prop: (base pose of the box, half size, BGR colour)."""
        return self._boxes(False)

    def glass(self) -> list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]]:
        """The clear boxes (acrylic bins): drawn as a tint over what is behind them, hiding nothing."""
        return self._boxes(True)

    def decals(self) -> list[dict]:
        """Flat stickers lying on the props: {pose (base, at the sticker centre, x right / y up the sticker),
        half (w, h), kind 'paper' | 'marker', id}."""
        with self.lock:
            out = []
            for p in self.props.values():
                for d in p.get("decals", ()):
                    pose = p["pose"].copy()
                    pose[:3, 3] += pose[:3, :3] @ np.array([d["centre"][0], d["centre"][1], d["z"]])
                    out.append(dict(pose=pose, half=tuple(d["half"]), kind=d["kind"], id=d.get("id")))
            return out

    def snapshot(self) -> dict:
        """Ground truth for a front end: what is on the table and where."""
        with self.lock:
            out = []
            for bid, b in self.blocks.items():
                r, c = b["pose"][:3, :3], b["pose"][:3, 3]
                face = int(np.argmax(r[2] @ FACE_AXES.T))
                side = r @ FACE_AXES[(face + 2) % 6 if face < 4 else 0]     # any face perpendicular to `face`
                yaw = (math.degrees(math.atan2(side[1], side[0])) + 45) % 90 - 45
                m = self.table.base_to_map(c)
                inside = None if bid == self.held else self.prop_under(c[:2])[1]
                if inside and c[2] - BLOCK / 2 > self.props[inside]["pose"][2, 3] + (self.props[inside]["dims"][2] + self.props[inside]["dims"][3]) / 100:
                    inside = None               # resting on something above the rim
                if b.get("perched"):
                    inside = None               # hanging on the rim: neither in nor out
                out.append(dict(id=bid, category=b["category"], colour=COLOURS[b["category"]],
                                up=self.items[b["category"]][face], held=bid == self.held, inside=inside,
                                perched=bool(b.get("perched")),
                                map_cm=[round(float(m[0]) * 100, 2), round(float(m[1]) * 100, 2)],
                                base_cm=[round(float(v) * 100, 2) for v in c], yaw_deg=round(yaw, 1),
                                on_map=bool(MAP_X_CM[0] <= m[0] * 100 <= MAP_X_CM[1] and MAP_Y_CM[0] <= m[1] * 100 <= MAP_Y_CM[1]),
                                base_T_block=[[round(float(v), 5) for v in row] for row in b["pose"]]))
            props = []
            for pid, p in self.props.items():
                m = self.table.base_to_map(p["pose"][:3, 3])
                props.append(dict(id=pid, kind=p["kind"], name=p["name"], category=p["category"], colour=p["colour"],
                                  number=p.get("number"), planned=p.get("planned"), glass=bool(p.get("glass")),
                                  map_cm=[round(float(m[0]) * 100, 2), round(float(m[1]) * 100, 2)],
                                  yaw_deg=round(math.degrees(math.atan2(p["pose"][1, 0], p["pose"][0, 0])), 1),
                                  dims_cm=p["dims"], holds=[b["id"] for b in out if b["inside"] == pid],
                                  base_T_prop=[[round(float(v), 5) for v in row] for row in p["pose"]],
                                  parts=[dict(centre_cm=[round(v * 100, 3) for v in c], size_cm=[round(v * 200, 3) for v in h])
                                         for c, h in p["parts"]]))
            off = self.table.offset
            return dict(revision=self.revision, block_side_cm=BLOCK * 100, table_z_cm=round(self.table_z * 100, 2),
                        holding=self.held, blocks=out, props=props, faces=self.items, faults=self.fault_state(),
                        contacts=dict(count=self.contact_count, recent=list(self.contacts[-10:])),
                        map=dict(name=self.map_name, base_from_map_cm=[round(float(v) * 100, 3) for v in off],
                                 page_cm=(self.map_data or {}).get("page_cm", [29.7, 21.0]) if self.map_name else None,
                                 markers=sorted(self.table.layout) if self.map_name else []),
                        evidence="simulation: model geometry and an assumed grasp rule, not measurements")


class WorldArm:
    """Drop-in for teleop.core.SimArm that tells the world where the arm is."""

    def __init__(self, world: World):
        self.world = world
        self.pose = list(START_POSE)
        world.arm_moved(self.pose)

    def read(self):
        return self.pose[:5] + [self.world.gripper_readback(self.pose[5])]

    def write6(self, pose, duration_ms):
        self.pose = list(pose)
        self.world.arm_moved(self.pose)
