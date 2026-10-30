"""The acrylic bins as the pipeline sees them: where each stands, found from its own stickers, and where a
block let go above it lands.

Design values come from config/bins_<name>.json (src/tools/make_bins.py): bin frame x right, +Y towards the
flange, origin at the middle of the opening on the plate's top face; three DICT_5X5_50 codes per bin (floor
10n, flange 10n+1 / 10n+2). A person stands the bins on the table, so the planned pose is only a first guess:
`locate` takes the wrist-camera frames that show the codes and solves, per view, the plane the codes lie in
(planar PnP, all codes of one bin together), puts it in the base frame through the forward kinematics and
hand-eye of that view, and fuses the views. A bin no frame shows keeps its planned pose, marked as such.

What this gives: the bin's x, y and yaw in the base frame, good to what the camera pose is good to (about a
centimetre at 25 cm, camera_model.py) -- the opening is 8 cm and a block 3 cm, so that is enough to drop in,
not enough to drop at a chosen millimetre. Nothing here moves the arm.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DESIGN = "bins_acrylic_v1"
DICTIONARY = cv2.aruco.DICT_5X5_50
MAX_REPROJ_PX = 3.0         # a code set that does not fit its plane better than this is a misread
MAX_TILT_DEG = 8.0          # a bin stands flat; a plane tilted more than this is a bad solution
MIN_SIDE_PX = 18            # a code smaller than this is not trusted for a pose


def load_design(name: str = DEFAULT_DESIGN) -> dict:
    return json.loads((ROOT / "config" / f"{name}.json").read_text())


def rot2(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def circular_mean_deg(angles) -> float:
    a = np.radians(np.asarray(angles, float))
    return math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))


@dataclass
class Bin:
    number: int
    category: str
    name: str
    ids: dict
    corners: dict[int, np.ndarray]      # marker id -> 4 x 2 corners (TL, TR, BR, BL) in the bin frame, metres
    xy: np.ndarray                      # base frame, metres
    yaw_deg: float                      # bin +Y points at yaw + 90 degrees from base +X
    source: str = "planned"             # planned | markers
    views: int = 0
    markers: list = field(default_factory=list)
    spread_mm: float | None = None
    reproj_px: float | None = None
    tilt_deg: float | None = None
    planned_xy: np.ndarray | None = None
    planned_yaw_deg: float | None = None

    def to_base(self, local_xy) -> np.ndarray:
        """Base-frame xy (metres) of a point given in the bin frame (metres)."""
        return self.xy + rot2(self.yaw_deg) @ np.asarray(local_xy, float)

    def to_local(self, base_xy) -> np.ndarray:
        return rot2(self.yaw_deg).T @ (np.asarray(base_xy, float) - self.xy)

    def shift_mm(self) -> float | None:
        """How far from its plan the bin was found."""
        return None if self.planned_xy is None else float(np.linalg.norm(self.xy - self.planned_xy)) * 1000

    def turn_deg(self) -> float | None:
        return None if self.planned_yaw_deg is None else (self.yaw_deg - self.planned_yaw_deg + 180) % 360 - 180

    def summary(self) -> dict:
        return dict(bin=str(self.number), category=self.category, source=self.source,
                    base_cm=[round(float(v) * 100, 2) for v in self.xy], yaw_deg=round(self.yaw_deg, 1),
                    views=self.views, markers=sorted(set(self.markers)), spread_mm=self.spread_mm,
                    reproj_px=self.reproj_px, tilt_deg=self.tilt_deg,
                    shift_mm=None if self.shift_mm() is None else round(self.shift_mm(), 1),
                    turn_deg=None if self.turn_deg() is None else round(self.turn_deg(), 1))


SIDES = {"right": (1, 2), "left": (3, 4)}      # base x > 0 is the arm's right; the design numbers them 1-4 from there


def parse_only(spec: str | None) -> tuple[int, ...] | None:
    """`right` / `left` (the two bins on that side) or numbers such as `1,2`; None for all bins. A half-side
    start lets a table too small for four bins be tried with two."""
    if not spec or spec == "all":
        return None
    if spec in SIDES:
        return SIDES[spec]
    try:
        return tuple(sorted({int(v) for v in spec.split(",")}))
    except ValueError as exc:
        raise ValueError("桶的选择应为 left / right / all，或形如 1,2 的桶号") from exc


class BinSet:
    """The bins of one design that stand on the table (all four, or a chosen few), with how many blocks each
    has been given."""

    def __init__(self, name: str = DEFAULT_DESIGN, only=None):
        self.name = name
        self.design = load_design(name)
        g = self.design["geometry_mm"]
        self.inner_m = g["inner"] / 1000
        self.wall_top_m = g["thickness"] / 1000 + g["wall_height"] / 1000     # above the table the plate stands on
        self.marker_z_m = self.design["marker_plane_above_table_m"]
        self.side_m = self.design["side_m"]
        self.bins: dict[int, Bin] = {}
        for number, e in self.design["bins"].items():
            if only is not None and int(number) not in only:
                continue
            cx, cy = e["planned_centre_base_m"]
            yaw = e["planned_flange_direction_deg"] - 90.0
            self.bins[int(number)] = Bin(
                int(number), e["category"], e["name"], e["ids"],
                {int(i): np.array(c, float) for i, c in e["corners_m"].items()}, np.array([cx, cy], float), yaw,
                planned_xy=np.array([cx, cy], float), planned_yaw_deg=yaw)
        if not self.bins:
            raise ValueError("没有选中任何桶")
        self.given: dict[int, int] = {n: 0 for n in self.bins}
        self.slots: dict[int, list[int]] = {}      # per bin, the slot indices the arm can reach (decide_bins fills it)

    def of_category(self, category: str) -> Bin | None:
        return next((b for b in self.bins.values() if b.category == category), None)

    def containing(self, base_xy, margin_m: float = 0.0) -> Bin | None:
        """The bin whose opening (grown by `margin_m`) holds this base-frame point."""
        for b in self.bins.values():
            local = np.abs(b.to_local(base_xy))
            if local[0] <= self.inner_m / 2 + margin_m and local[1] <= self.inner_m / 2 + margin_m:
                return b
        return None

    def over_footprint(self, base_xy, margin_m: float = 0.0) -> Bin | None:
        """The bin whose plate (flange included) lies under this point, for keeping blocks off a bin."""
        g = self.design["geometry_mm"]
        (y0, y1), half_w = g["plate_y_range"], g["plate"][0] / 2000
        for b in self.bins.values():
            local = b.to_local(base_xy)
            if abs(local[0]) <= half_w + margin_m and y0 / 1000 - margin_m <= local[1] <= y1 / 1000 + margin_m:
                return b
        return None


# ------------------------------------------------------------------------------------------ finding them

def detector() -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), params)


def plane_pose(b: Bin, found: dict[int, np.ndarray], k, dist, prior_cam_T_bin: np.ndarray | None):
    """cam_T_bin (4x4) of the bin's marker plane from the codes of this bin found in one image, plus the
    reprojection error in pixels. Both planar solutions are tried; the one nearest the prior wins."""
    ids = [i for i in found if i in b.corners]
    obj = np.vstack([np.column_stack([b.corners[i], np.zeros(4)]) for i in ids])
    img = np.vstack([found[i] for i in ids]).astype(float)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, k, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    best = None
    for rv, tv in zip(rvecs, tvecs):
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
        if m[2, 3] <= 0:
            continue
        key = np.linalg.norm(m[:3, 3] - prior_cam_T_bin[:3, 3]) if prior_cam_T_bin is not None else 0.0
        if best is None or key < best[0]:
            best = (key, rv, tv)
    if best is None:
        return None
    rv, tv = cv2.solvePnPRefineLM(obj, img, k, dist, best[1], best[2])
    proj, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rv, tv, k, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
    return m, err


def locate(views: list[dict], bins: BinSet, k, dist, table_z_m: float) -> dict[int, dict]:
    """Per bin the pose found from `views` = [{"image": BGR array, "base_T_cam": 4x4}, ...].

    Returns {number: {"xy", "yaw_deg", "views", "markers", "spread_mm", "reproj_px", "tilt_deg"}} for the
    bins at least one view showed with a usable code set; the others are not in it."""
    det = detector()
    per_bin: dict[int, list[dict]] = {}
    plane_z = table_z_m + bins.marker_z_m
    for v in views:
        corners, ids, _ = det.detectMarkers(v["image"])
        if ids is None:
            continue
        seen = {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)
                if any(int(i) in b.corners for b in bins.bins.values())}
        for b in bins.bins.values():
            mine = {i: c for i, c in seen.items() if i in b.corners}
            mine = {i: c for i, c in mine.items()
                    if min(np.linalg.norm(c[j] - c[(j + 1) % 4]) for j in range(4)) >= MIN_SIDE_PX}
            if not mine:
                continue
            cam_T_base = np.linalg.inv(v["base_T_cam"])
            planned = np.eye(4)
            planned[:3, :3] = np.array([[math.cos(math.radians(b.planned_yaw_deg)), -math.sin(math.radians(b.planned_yaw_deg)), 0],
                                        [math.sin(math.radians(b.planned_yaw_deg)), math.cos(math.radians(b.planned_yaw_deg)), 0],
                                        [0, 0, 1]])
            planned[:3, 3] = [*b.planned_xy, plane_z]
            got = plane_pose(b, mine, k, dist, cam_T_base @ planned)
            if got is None:
                continue
            cam_T_bin, err = got
            base_T_bin = v["base_T_cam"] @ cam_T_bin
            tilt = math.degrees(math.acos(float(np.clip(base_T_bin[2, 2], -1, 1))))
            if err > MAX_REPROJ_PX or tilt > MAX_TILT_DEG:
                continue
            per_bin.setdefault(b.number, []).append(dict(
                xy=base_T_bin[:2, 3], yaw=math.degrees(math.atan2(base_T_bin[1, 0], base_T_bin[0, 0])),
                markers=sorted(mine), reproj=err, tilt=tilt, z=float(base_T_bin[2, 3]), n=len(mine)))
    out = {}
    for number, rows in per_bin.items():
        xy = np.array([r["xy"] for r in rows])
        weights = np.array([r["n"] for r in rows], float)
        centre = np.average(xy, axis=0, weights=weights)
        out[number] = dict(
            xy=centre, yaw_deg=circular_mean_deg([r["yaw"] for r in rows]), views=len(rows),
            markers=sorted({m for r in rows for m in r["markers"]}),
            spread_mm=round(float(np.linalg.norm(xy - centre, axis=1).max()) * 1000, 1) if len(rows) > 1 else None,
            reproj_px=round(float(np.mean([r["reproj"] for r in rows])), 2),
            tilt_deg=round(float(np.mean([r["tilt"] for r in rows])), 1),
            z_cm=round(float(np.mean([r["z"] for r in rows])) * 100, 2))
    return out


def apply(bins: BinSet, found: dict[int, dict]) -> BinSet:
    for number, f in found.items():
        b = bins.bins[number]
        b.xy, b.yaw_deg, b.source = np.array(f["xy"], float), float(f["yaw_deg"]), "markers"
        b.views, b.markers, b.spread_mm = f["views"], list(f["markers"]), f["spread_mm"]
        b.reproj_px, b.tilt_deg = f["reproj_px"], f["tilt_deg"]
    return bins


def locate_run(run: Path, bins: BinSet, table_z_m: float) -> BinSet:
    """Fill `bins` from a scan's bin_views.json (src/pipeline/field_scan.py scan_bins): each entry carries
    the image and the servo readback it was taken at."""
    import sys
    sys.path.insert(0, str(ROOT / "src" / "spatial"))
    from camera_model import ArmCamera
    from map_registration import load_intrinsics
    k, dist = load_intrinsics()
    cam = ArmCamera()
    entries = json.loads((Path(run) / "bin_views.json").read_text())
    views = []
    for e in entries:
        img = cv2.imread(str(Path(run) / e["image"]))
        if img is not None:
            views.append(dict(image=img, base_T_cam=cam.base_T_cam(e["readback"])))
    return apply(bins, locate(views, bins, k, dist, table_z_m))


# ------------------------------------------------------------------------------------------ dropping in

def drop_slots(slots_cm) -> list[np.ndarray]:
    return [np.array(s, float) / 100 for s in slots_cm]


def drop_xy(b: Bin, slot_m) -> np.ndarray:
    """Base-frame point above which a block is let go into slot `slot_m` (bin frame, metres)."""
    return b.to_base(slot_m)


def plate_gap_mm(a: Bin, b: Bin, bins: BinSet, step_m: float = 0.002) -> float:
    """Smallest distance between two plates' outlines (negative when they overlap), by sampling a's outline."""
    g = bins.design["geometry_mm"]
    half = g["plate"][0] / 2000
    y0, y1 = [v / 1000 for v in g["plate_y_range"]]
    corners = [(-half, y0), (half, y0), (half, y1), (-half, y1)]
    best = math.inf
    for i in range(4):
        p, q = np.array(corners[i]), np.array(corners[(i + 1) % 4])
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(q - p) / step_m))):
            local = b.to_local(a.to_base(p + (q - p) * t))
            dx, dy = abs(local[0]) - half, max(y0 - local[1], local[1] - y1)
            outside = math.hypot(max(dx, 0), max(dy, 0))
            best = min(best, outside if outside > 0 else max(dx, dy))
    return best * 1000


def table_check(bins: BinSet, min_gap_mm: float = 5.0) -> list[str]:
    """Problems with where the bins stand: plates closer than `min_gap_mm` (the gripper needs room to open
    above a bin and a person nudging one must not push it into the next)."""
    out = []
    nums = sorted(bins.bins)
    for i, n in enumerate(nums):
        for m in nums[i + 1:]:
            gap = min(plate_gap_mm(bins.bins[n], bins.bins[m], bins), plate_gap_mm(bins.bins[m], bins.bins[n], bins))
            if gap < min_gap_mm:
                out.append(f"{n} 号与 {m} 号桶的底板只隔 {gap:.1f} mm" if gap > 0 else f"{n} 号与 {m} 号桶的底板重叠 {-gap:.1f} mm")
    return out
