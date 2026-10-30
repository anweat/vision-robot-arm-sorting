"""What the wrist camera would see: the printed map, the table and the blocks, ray-cast per pixel.

The camera is the project's own model: intrinsics and distortion from
config/calibration/camera_intrinsics.json, pose from the servo angles through dofbot_kinematics and
config/calibration/hand_eye.json (spatial/camera_model.py). Every pixel's ray is undistorted once, so the
picture has the lens distortion the detectors and the map PnP expect.

Drawn: the printed map the world has on its table (its ArUco codes redrawn from the map's JSON, not the
PDF; or a picture named by the JSON's "image"; or no map), a dark table around it, 30 mm blocks textured
with the photographed faces in data/reference/block_faces, the world's props as flat-coloured boxes,
Lambert shading, hard shadows softened by a blur, vignetting, blur and sensor noise.

Also drawn, for the designed bins (config/bins_*.json): the stickers on the plate (paper, DICT_5X5_50 codes on
the flange and the floor) as flat decals that blocks and walls can hide, and the acrylic as a faint tint over
whatever is behind it (a block inside a bin is seen through the wall; user-added `bin` props stay opaque).

Not drawn: the arm itself (the gripper fingers do not appear in the frame), the platform, refraction and
reflections in the acrylic, exposure changes, motion blur. A detector that passes here has been checked against the
geometry and the face artwork, not against the real camera.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "sim"):
    sys.path.insert(0, str(ROOT / "src" / sub))

from camera_model import ArmCamera  # noqa: E402
from map_registration import TableMap, load_intrinsics  # noqa: E402
import world as W  # noqa: E402

IMAGE_W, IMAGE_H = 640, 480
PAGE_CM = (29.7, 21.0)     # a map JSON may give its own "page_cm"; the origin is the middle of the near edge
PAGE_PPM = 5000            # page texture, pixels per metre
WOOD_PPM = 2000
PAPER = (232, 232, 228)    # BGR of a printed sticker's white
INK = (22, 22, 22)
GLASS_TINT = np.array([218, 214, 200], np.float32)    # BGR, clear acrylic over a dark table: faint, cool
BACKDROP = (48, 46, 44)    # above the table's horizon
MIN_VISIBLE_PX = 150       # a block showing fewer pixels than this is not expected to be detected
MIN_INSIDE = 0.1           # as the training labels: share of the full box that must be inside the frame


def page_texture(table: TableMap, data: dict) -> np.ndarray:
    """The sheet as a texture, far edge on the first row."""
    page_w, page_h = [v / 100 for v in data.get("page_cm", PAGE_CM)]
    w, h = round(page_w * PAGE_PPM), round(page_h * PAGE_PPM)
    if data.get("image"):
        picture = cv2.imread(str(ROOT / data["image"]))
        if picture is None:
            raise ValueError(f"地图图片读不出来：{data['image']}")
        page = cv2.resize(picture, (w, h), interpolation=cv2.INTER_AREA)
    else:
        page = np.full((h, w, 3), 236, np.uint8)

    def px(x, y):
        return round((x + page_w / 2) * PAGE_PPM), round((page_h - y) * PAGE_PPM)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side = round(table.side * PAGE_PPM)
    for mid, corners in table.layout.items():
        x0, y0 = px(*corners[0])                      # TL, the marker's top towards +Y
        if x0 < 0 or y0 < 0 or x0 + side > w or y0 + side > h:
            continue
        marker = cv2.aruco.generateImageMarker(dictionary, mid, side, borderBits=1)
        page[y0:y0 + side, x0:x0 + side] = np.where(marker[..., None] > 127, 236, 22)
        cv2.putText(page, str(mid), (x0 + side + 4, y0 + side), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
    if data.get("image"):
        return page
    for outline in data.get("bins", {}).get("plate_outline_map_cm", {}).values():      # seating marks (map v3)
        cv2.polylines(page, [np.array([px(x / 100, y / 100) for x, y in outline], np.int32)], True, (110, 110, 110), 2)
    rows = sorted({round(float(c[:, 1].mean()), 4) for c in table.layout.values()})
    for y in np.arange(0.003, page_h - 0.003, 0.006):   # dashed centre line, broken around the codes on X = 0
        if any(abs(y + 0.0015 - r) < table.side / 2 + 0.004 for r in rows):
            continue
        cv2.line(page, px(0, y), px(0, y + 0.003), (150, 150, 150), 2)
    for xc in range(-int(page_w * 50), int(page_w * 50) + 1):
        a = px(xc / 100, 0)
        cv2.line(page, a, (a[0], a[1] - (15 if xc % 5 == 0 else 8)), (30, 30, 30), 1)
    for yc in range(1, int(page_h * 100)):
        a = px(-page_w / 2, yc / 100)
        cv2.line(page, a, (a[0] + (15 if yc % 5 == 0 else 8), a[1]), (30, 30, 30), 1)
    return page


def wood_texture(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    noise = np.tile(rng.normal(0, 1, (512, 512)).astype(np.float32), (3, 3))     # tiled so the blur wraps
    grain = cv2.GaussianBlur(noise, (0, 0), sigmaX=24, sigmaY=2)[512:1024, 512:1024]
    grain /= max(float(np.abs(grain).max()), 1e-6)
    base = np.array([34, 38, 44], np.float32)        # BGR: the dark table of the real frames
    return np.clip(base * (1 + 0.22 * grain[..., None]), 0, 255).astype(np.uint8)


def load_faces(items: dict[str, list[str]]) -> dict[str, list[np.ndarray]]:
    """category -> six textures in the world's local face order."""
    out = {}
    for category, names in items.items():
        colour = cv2.imread(str(W.FACES_DIR / f"{category}__colour_{W.COLOURS[category]}.png"))
        n = colour.shape[0]
        plain = cv2.resize(cv2.blur(colour[n // 3:2 * n // 3, n // 3:2 * n // 3], (9, 9)), (n, n))
        out[category] = [plain if name == "plain" else cv2.imread(str(W.FACES_DIR / f"{category}__{name}.png"))
                         for name in names]
    return out


def cube_corners(pose: np.ndarray, half=W.BLOCK / 2) -> np.ndarray:
    local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    return local @ pose[:3, :3].T + pose[:3, 3]


def bilinear(tex: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Texture colours at float pixel positions (N,), edges clamped."""
    n = tex.shape[0] - 1
    x, y = np.clip(x, 0, n), np.clip(y, 0, n)
    x0, y0 = np.minimum(x.astype(int), n - 1), np.minimum(y.astype(int), n - 1)
    fx, fy = (x - x0)[:, None], (y - y0)[:, None]
    t = tex.astype(np.float32)
    return (t[y0, x0] * (1 - fx) + t[y0, x0 + 1] * fx) * (1 - fy) + (t[y0 + 1, x0] * (1 - fx) + t[y0 + 1, x0 + 1] * fx) * fy


def slab(origin: np.ndarray, direction: np.ndarray, h=W.BLOCK / 2) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Ray against a box of half size `h` in the box's own frame: (entry t, exit t, entry axis). A hit has
    exit >= entry; arrays broadcast, so either origins or directions may be a single vector."""
    safe = np.where(np.abs(direction) < 1e-9, 1e-9, direction)
    t1, t2 = (-h - origin) / safe, (h - origin) / safe
    near, far = np.minimum(t1, t2), np.maximum(t1, t2)
    return near.max(-1), far.min(-1), near.argmax(-1)


class Renderer:
    def __init__(self, items: dict[str, list[str]], supersample: int = 2, seed: int = 0):
        self.k, self.dist = load_intrinsics()
        self.camera = ArmCamera()
        self.ss = supersample
        w, h = IMAGE_W * supersample, IMAGE_H * supersample
        u, v = np.meshgrid((np.arange(w) + 0.5) / supersample - 0.5, (np.arange(h) + 0.5) / supersample - 0.5)
        normal = cv2.undistortPoints(np.stack([u, v], -1).reshape(-1, 1, 2), self.k, self.dist).reshape(h, w, 2)
        self.rays = np.dstack([normal, np.ones((h, w))]).astype(np.float32)     # camera frame, z = 1
        self.codes: dict[int, np.ndarray] = {}
        self.set_map(TableMap(), {})
        self.wood = wood_texture(seed)
        self.faces = load_faces(items)
        light = np.array([-0.35, -0.3, 0.89])
        self.light = light / np.linalg.norm(light)                              # unit vector towards the light
        yy, xx = np.mgrid[0:IMAGE_H, 0:IMAGE_W]
        r2 = ((xx - IMAGE_W / 2) ** 2 + (yy - IMAGE_H / 2) ** 2) / (IMAGE_W / 2) ** 2
        self.vignette = (1 - 0.16 * r2).astype(np.float32)[..., None]

    def set_map(self, table: TableMap, data: dict | None) -> None:
        """The sheet on the table: `data` is the map's JSON, or None for a bare table. `table` always gives
        the placement (where map coordinates and the table surface are)."""
        self.table = table
        self.page = None if data is None else page_texture(table, data)
        self.page_w, self.page_h = [v / 100 for v in (data or {}).get("page_cm", PAGE_CM)]

    def base_T_cam(self, servos) -> np.ndarray:
        return self.camera.base_T_cam(servos)

    def project(self, servos, points_base: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Pixels of base-frame points through cv2.projectPoints (the forward model the pipeline uses,
        independent of this renderer's inverse), and their depth in the camera frame."""
        cam_T_base = np.linalg.inv(self.base_T_cam(servos))
        pts = np.asarray(points_base, float).reshape(-1, 3)
        depth = pts @ cam_T_base[2, :3] + cam_T_base[2, 3]
        uv, _ = cv2.projectPoints(pts.reshape(-1, 1, 3), cv2.Rodrigues(cam_T_base[:3, :3])[0], cam_T_base[:3, 3],
                                  self.k, self.dist)
        return uv.reshape(-1, 2), depth

    def roi(self, servos, points_base: np.ndarray, pad: int = 6):
        """Supersampled-image slice covering the points, or the whole frame if one is behind the camera."""
        h, w = self.rays.shape[:2]
        uv, depth = self.project(servos, points_base)
        if depth.min() < 0.03:
            return slice(0, h), slice(0, w)
        x0, y0 = np.floor((uv.min(0) + 0.5) * self.ss - pad).astype(int)
        x1, y1 = np.ceil((uv.max(0) + 0.5) * self.ss + pad).astype(int)
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
        return (slice(y0, y1), slice(x0, x1)) if x1 > x0 and y1 > y0 else None

    def marker_cells(self, marker_id: int) -> np.ndarray:
        """The DICT_5X5_50 code as a 7 x 7 grid (1 = white), border included, top row first."""
        if marker_id not in self.codes:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
            self.codes[marker_id] = (cv2.aruco.generateImageMarker(dictionary, marker_id, 7, borderBits=1) > 127)
        return self.codes[marker_id]

    def draw_decal(self, image, depth, servos, origin, rays, decal) -> None:
        """A flat sticker: where its plane is the first thing the ray meets, paint paper or the code."""
        pose, (hw, hh) = decal["pose"], decal["half"]
        corners = pose[:3, 3] + np.array([[sx * hw, sy * hh, 0] for sx in (-1, 1) for sy in (-1, 1)]) @ pose[:3, :3].T
        area = self.roi(servos, corners)
        if area is None:
            return
        normal = pose[:3, 2]
        d = rays[area]
        denom = d @ normal
        t = ((pose[:3, 3] - origin) @ normal) / np.where(np.abs(denom) < 1e-6, 1e-6, denom)
        local = (origin + t[..., None] * d - pose[:3, 3]) @ pose[:3, :3]
        hit = (t > 0.01) & (t < depth[area]) & (np.abs(local[..., 0]) <= hw) & (np.abs(local[..., 1]) <= hh)
        if not hit.any():
            return
        colour = np.empty(local.shape[:2] + (3,), np.float32)
        colour[:] = PAPER
        if decal["kind"] == "marker":
            cells = self.marker_cells(decal["id"])
            col = np.clip(((local[..., 0] + hw) / (2 * hw) * 7).astype(int), 0, 6)
            row = np.clip(((hh - local[..., 1]) / (2 * hh) * 7).astype(int), 0, 6)
            colour[~cells[row, col]] = INK
        view, view_depth = image[area], depth[area]
        view[hit] = colour[hit]
        view_depth[hit] = t[hit]

    def draw_glass(self, image, depth, servos, origin, rays, glass) -> None:
        """Clear boxes: a tint that grows with the path the ray spends inside, over what is already drawn."""
        pose, half, _ = glass
        area = self.roi(servos, cube_corners(pose, half))
        if area is None:
            return
        rb = pose[:3, :3].astype(np.float32)
        local_o = ((origin - pose[:3, 3]) @ pose[:3, :3]).astype(np.float32)
        enter, leave, _ = slab(local_o, rays[area] @ rb, half.astype(np.float32))
        hit = (leave >= enter) & (enter > 0.01) & (enter < depth[area])
        if hit.any():
            alpha = np.clip(0.06 + 14.0 * (np.minimum(leave, depth[area]) - enter), 0, 0.45)[..., None]
            view = image[area]
            view[hit] = (view * (1 - alpha) + GLASS_TINT * alpha)[hit]

    def render(self, servos, blocks: list[tuple[str, str, np.ndarray]], solids=(), glass=(), decals=()
               ) -> tuple[np.ndarray, list[dict]]:
        """(clean 640x480 BGR frame as float32, truth for each block that is in view). `solids` are the
        world's prop boxes, (pose, half size, BGR): drawn, casting shadows and hiding blocks, never labelled.
        `decals` (World.decals) are stickers lying on props; `glass` (World.glass) clear boxes, tinted over
        everything behind them. Neither casts a shadow or hides a block.

        truth: id, category, bbox [x0, y0, x1, y1] of the whole cube clipped to the frame (the training
        labels' convention), visible_px (pixels not hidden by another block)."""
        base_T_cam = self.base_T_cam(servos)
        r, o = base_T_cam[:3, :3].astype(np.float32), base_T_cam[:3, 3]
        d = self.rays @ r.T
        zt = float(self.table.offset[2])
        down = d[..., 2] < -1e-6
        depth = np.where(down, (zt - o[2]) / np.where(down, d[..., 2], -1.0), np.inf).astype(np.float32)
        ground = o.astype(np.float32) + np.where(down, depth, 0)[..., None] * d
        mx, my = ground[..., 0] - float(self.table.offset[0]), ground[..., 1] - float(self.table.offset[1])
        image = cv2.remap(self.wood, (mx * WOOD_PPM) % 512, (my * WOOD_PPM) % 512, cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_WRAP).astype(np.float32)
        if self.page is not None:
            on_page = down & (np.abs(mx) <= self.page_w / 2) & (my >= 0) & (my <= self.page_h)
            page = cv2.remap(self.page, (mx + self.page_w / 2) * PAGE_PPM - 0.5, (self.page_h - my) * PAGE_PPM - 0.5,
                             cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            image[on_page] = page[on_page]
        image[~down] = BACKDROP
        boxes = [(pose, np.full(3, W.BLOCK / 2)) for _, _, pose in blocks] + [(pose, half) for pose, half, _ in solids]

        shadow = np.zeros(depth.shape, np.float32)
        to_light = self.light
        for pose, half in boxes:
            corners = cube_corners(pose, half)
            cast = corners - to_light * ((corners[:, 2] - zt) / to_light[2])[:, None]
            area = self.roi(servos, np.vstack([corners, cast]))
            if area is None:
                continue
            rb = pose[:3, :3].astype(np.float32)
            enter, leave, _ = slab((ground[area] - pose[:3, 3].astype(np.float32)) @ rb, (to_light @ pose[:3, :3]).astype(np.float32),
                                   half.astype(np.float32))
            shadow[area] = np.maximum(shadow[area], ((leave >= enter) & (leave > 1e-4) & down[area]).astype(np.float32))
        shadow = cv2.GaussianBlur(shadow, (0, 0), 2.2 * self.ss)
        image *= (1 - 0.42 * shadow)[..., None]

        ids = np.zeros(depth.shape, np.int16)
        paints = [self.faces[category] for _, category, _ in blocks] + [bgr for _, _, bgr in solids]
        for number, ((pose, half), paint) in enumerate(zip(boxes, paints), 1):
            area = self.roi(servos, cube_corners(pose, half))
            if area is None:
                continue
            rb = pose[:3, :3].astype(np.float32)
            local_o = ((o - pose[:3, 3]) @ pose[:3, :3]).astype(np.float32)
            local_d = d[area] @ rb
            enter, leave, axis = slab(local_o, local_d, half.astype(np.float32))
            hit = (leave >= enter) & (enter > 0.01) & (enter < depth[area])
            if not hit.any():
                continue
            point = local_o + enter[..., None] * local_d
            sign = np.take_along_axis(point, axis[..., None], -1)[..., 0] > 0
            view_image, view_depth, view_ids = image[area], depth[area], ids[area]
            for face in range(6):
                a = face // 2
                m = hit & (axis == a) & (sign == (face % 2 == 0))
                if not m.any():
                    continue
                if isinstance(paint, tuple):
                    colour = np.array(paint, np.float32)
                else:
                    tex = paint[face]
                    n = tex.shape[0]
                    p = point[m] / W.BLOCK + 0.5                       # 0..1 across the cube
                    colour = bilinear(tex, p[:, (a + 1) % 3] * n - 0.5, (1 - p[:, (a + 2) % 3]) * n - 0.5)
                normal = pose[:3, :3] @ W.FACE_AXES[face]
                view_image[m] = colour * (0.55 + 0.45 * max(0.0, float(normal @ to_light)))
            view_depth[hit] = enter[hit]
            view_ids[hit] = number

        for decal in decals:
            self.draw_decal(image, depth, servos, o, d, decal)
        for box in glass:
            self.draw_glass(image, depth, servos, o, d, box)

        frame = cv2.resize(image, (IMAGE_W, IMAGE_H), interpolation=cv2.INTER_AREA) * self.vignette
        frame = cv2.GaussianBlur(frame, (0, 0), 0.7)
        counts = np.bincount(ids.ravel(), minlength=len(boxes) + 1) / self.ss ** 2
        truth = []
        for number, (bid, category, pose) in enumerate(blocks, 1):
            uv, z = self.project(servos, cube_corners(pose))
            if z.min() < 0.03 or counts[number] < 1:
                continue
            x0, y0, x1, y1 = *uv.min(0), *uv.max(0)
            cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, IMAGE_W - 1), min(y1, IMAGE_H - 1)
            inside = max(cx1 - cx0, 0) * max(cy1 - cy0, 0) / max((x1 - x0) * (y1 - y0), 1e-9)
            truth.append(dict(id=bid, category=category, bbox=[round(float(v), 1) for v in (cx0, cy0, cx1, cy1)],
                              visible_px=int(counts[number]),
                              expected=bool(counts[number] >= MIN_VISIBLE_PX and inside >= MIN_INSIDE)))
        return frame, truth


def sensor(frame: np.ndarray, rng: np.random.Generator, noise: float = 2.0) -> np.ndarray:
    """The clean frame as the sensor would deliver it: fresh noise each time, 8 bit."""
    return np.clip(frame + rng.normal(0, noise, frame.shape).astype(np.float32), 0, 255).astype(np.uint8)


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path, help="JPEG to write")
    parser.add_argument("--servos", type=float, nargs=5, default=W.START_POSE[:5])
    parser.add_argument("--blocks", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    world = W.World(args.seed)
    world.reset(args.blocks)
    frame, truth = Renderer(world.items).render(list(args.servos) + [30], world.render_list()[1])
    cv2.imwrite(str(args.out), sensor(frame, np.random.default_rng(args.seed)))
    for t in truth:
        print(t)


if __name__ == "__main__":
    main()
