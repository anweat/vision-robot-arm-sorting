"""Shared geometry and drawing primitives for the acrylic bins.

`make_bins.py` builds one bin design from these classes and writes the runtime config;
`bins_fab.py` turns the same geometry into the fabrication drawing set. Keeping both on
one model means the drawings cannot drift from config/bins_*.json.

Bin frame: origin at the centre of the opening on the plate's top face, +Y towards the
marker flange, X to the right when looking along +Y, millimetres in the drawings and
metres in the JSON.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = "/System/Library/Fonts/STHeiti Medium.ttc"
DICTIONARY = "DICT_5X5_50"  # the map and the table fiducials use DICT_4X4_50, the ChArUco board its ids 0-16
CATEGORIES = [("recyclable", "可回收物"), ("kitchen", "厨余垃圾"), ("hazardous", "有害垃圾"), ("other", "其他垃圾")]
BLOCK_MM = 30.0
A3 = (420.0, 297.0)
A4 = (210.0, 297.0)

INK = (25, 25, 25)
DIM = (32, 74, 168)
NOTE = (92, 92, 92)
WARN = (168, 40, 32)
ACCENT = (196, 96, 24)
GUIDE = (140, 140, 140)

_DICTIONARIES: dict[str, object] = {}


def dictionary() -> object:
    if DICTIONARY not in _DICTIONARIES:
        _DICTIONARIES[DICTIONARY] = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    return _DICTIONARIES[DICTIONARY]


class Canvas:
    """A page in millimetres, y up, drawn at `scale` drawing-mm per real mm."""

    def __init__(self, size_mm: tuple[float, float], dpi: int, scale: float = 1.0,
                 origin_mm: tuple[float, float] = (0.0, 0.0)):
        self.k, self.scale, self.origin, self.h = dpi / 25.4, scale, origin_mm, size_mm[1]
        self.image = Image.new("RGB", (round(size_mm[0] * self.k), round(size_mm[1] * self.k)), "white")
        self.draw = ImageDraw.Draw(self.image)
        self._fonts: dict[int, ImageFont.FreeTypeFont] = {}

    def px(self, x: float, y: float) -> tuple[int, int]:
        return (round((self.origin[0] + x * self.scale) * self.k),
                round((self.h - self.origin[1] - y * self.scale) * self.k))

    def font(self, mm: float) -> ImageFont.FreeTypeFont:
        size = max(8, round(mm * self.k))
        if size not in self._fonts:
            self._fonts[size] = ImageFont.truetype(FONT, size)
        return self._fonts[size]

    def line(self, a, b, fill=INK, width_mm: float = 0.25, dash_mm: float = 0.0) -> None:
        w = max(1, round(width_mm * self.k))
        if not dash_mm:
            self.draw.line([self.px(*a), self.px(*b)], fill=fill, width=w)
            return
        length = math.dist(a, b) * self.scale
        n = max(1, int(length / dash_mm))
        for i in range(0, n, 2):
            p = [(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n) for t in (i, min(i + 1, n))]
            self.draw.line([self.px(*p[0]), self.px(*p[1])], fill=fill, width=w)

    def poly(self, points, outline=INK, fill=None, width_mm: float = 0.25, dash_mm: float = 0.0) -> None:
        if fill:
            self.draw.polygon([self.px(*p) for p in points], fill=fill)
        for a, b in zip(points, points[1:] + points[:1]):
            self.line(a, b, outline, width_mm, dash_mm)

    def rect(self, x: float, y: float, w: float, h: float, **kw) -> None:
        self.poly([(x, y), (x + w, y), (x + w, y + h), (x, y + h)], **kw)

    def text(self, x: float, y: float, s: str, mm: float = 3.0, fill=INK, anchor: str = "lm") -> None:
        self.draw.text(self.px(x, y), s, fill=fill, font=self.font(mm), anchor=anchor)

    def circle(self, x: float, y: float, r: float, outline=INK, fill=None, width_mm: float = 0.25) -> None:
        pts = [(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a)))
               for a in np.arange(0, 361, 15)]
        self.poly(pts, outline=outline, fill=fill, width_mm=width_mm)

    def balloon(self, x: float, y: float, s: str, r: float = 5.0, mm: float = 3.4) -> None:
        self.circle(x, y, r, fill=(255, 255, 255), width_mm=0.3)
        self.text(x, y, s, mm, INK, "mm")

    def arrow(self, p, direction, size: float = 3.0, fill=INK, width_mm: float = 0.25) -> None:
        d = math.hypot(*direction) or 1.0
        u = (direction[0] / d, direction[1] / d)
        n = (-u[1], u[0])
        tip = p
        base = (p[0] - u[0] * size, p[1] - u[1] * size)
        self.poly([tip, (base[0] + n[0] * size * 0.34, base[1] + n[1] * size * 0.34),
                   (base[0] - n[0] * size * 0.34, base[1] - n[1] * size * 0.34)],
                  outline=fill, fill=fill, width_mm=width_mm)

    def dim(self, a, b, off: float, label: str | None = None, mm: float = 2.8,
            color=DIM, gap_mm: float = 0.8, over_mm: float = 1.6, tick_mm: float = 1.5) -> None:
        """Dimension line between a and b, shifted `off` real-mm along the left normal of a->b."""
        d = math.dist(a, b)
        if not d:
            return
        n = (-(b[1] - a[1]) / d, (b[0] - a[0]) / d)
        s = 1.0 if off >= 0 else -1.0
        a2, b2 = (a[0] + n[0] * off, a[1] + n[1] * off), (b[0] + n[0] * off, b[1] + n[1] * off)
        for p, q in ((a, a2), (b, b2)):
            start = (p[0] + n[0] * s * gap_mm, p[1] + n[1] * s * gap_mm)
            end = (q[0] + n[0] * s * over_mm, q[1] + n[1] * s * over_mm)
            self.line(start, end, color, 0.13)
        self.line(a2, b2, color, 0.16)
        u = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
        for p in (a2, b2):
            self.line((p[0] - (u[0] + n[0]) * tick_mm, p[1] - (u[1] + n[1]) * tick_mm),
                      (p[0] + (u[0] + n[0]) * tick_mm, p[1] + (u[1] + n[1]) * tick_mm), color, 0.25)
        shift = 2.2 * s
        mid = ((a2[0] + b2[0]) / 2 + n[0] * shift, (a2[1] + b2[1]) / 2 + n[1] * shift)
        self.text(*mid, label or f"{d:g}", mm, color, "mm")

    def hatch_rect(self, x: float, y: float, w: float, h: float, spacing: float = 1.8,
                   angle: int = 45, fill=(120, 120, 120), width_mm: float = 0.1) -> None:
        """45-degree section hatch clipped to an axis-aligned rectangle."""
        if angle == 45:
            corners = (y - x, y - (x + w), (y + h) - x, (y + h) - (x + w))
            for c in np.arange(min(corners), max(corners) + 1e-6, spacing * math.sqrt(2)):
                seg = _clip_segment((x, x + c), (x + w, x + w + c), (x, y, x + w, y + h))
                if seg:
                    self.line(*seg, fill, width_mm)
        else:
            corners = (x + y, (x + w) + y, x + (y + h), (x + w) + (y + h))
            for c in np.arange(min(corners), max(corners) + 1e-6, spacing * math.sqrt(2)):
                seg = _clip_segment((x, c - x), (x + w, c - x - w), (x, y, x + w, y + h))
                if seg:
                    self.line(*seg, fill, width_mm)

    def marker(self, mid: int, cx: float, cy: float, side: float) -> None:
        x0, y0 = self.px(cx - side / 2, cy + side / 2)
        x1, _ = self.px(cx + side / 2, cy + side / 2)
        self.image.paste(Image.fromarray(cv2.aruco.generateImageMarker(dictionary(), mid, x1 - x0, borderBits=1)), (x0, y0))

    def overlay(self) -> "Canvas":
        """A scale-1 view sharing this image, for annotations already in page millimetres."""
        o = Canvas((self.image.width / self.k, self.image.height / self.k), round(self.k * 25.4))
        o.image, o.draw = self.image, self.draw
        return o


def _clip_segment(p0, p1, rect):
    """Liang-Barsky clip of a segment against (xmin, ymin, xmax, ymax)."""
    xmin, ymin, xmax, ymax = rect
    x0, y0 = p0
    dx, dy = p1[0] - x0, p1[1] - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)):
        if p == 0:
            if q < 0:
                return None
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return None
    return (x0 + t0 * dx, y0 + t0 * dy), (x0 + t1 * dx, y0 + t1 * dy)



def _simplify_rect_loop(loop):
    out = []
    n = len(loop)
    for k, p in enumerate(loop):
        a, b = loop[k - 1], loop[(k + 1) % n]
        if (a[0] == p[0] == b[0]) or (a[1] == p[1] == b[1]):
            continue
        out.append(p)
    return out


def _rect_union(rects):
    """Boundary loops of the union of axis-aligned rectangles, without internal edges."""
    xs = sorted({r[0] for r in rects} | {r[1] for r in rects})
    ys = sorted({r[2] for r in rects} | {r[3] for r in rects})
    covered = set()
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            cx, cy = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
            if any(r[0] <= cx <= r[1] and r[2] <= cy <= r[3] for r in rects):
                covered.add((i, j))
    edges: dict[tuple[float, float], list[tuple[float, float]]] = {}

    def add(a, b):
        edges.setdefault(a, []).append(b)

    for i, j in covered:
        x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
        if (i, j - 1) not in covered:
            add((x0, y0), (x1, y0))
        if (i + 1, j) not in covered:
            add((x1, y0), (x1, y1))
        if (i, j + 1) not in covered:
            add((x1, y1), (x0, y1))
        if (i - 1, j) not in covered:
            add((x0, y1), (x0, y0))
    loops = []
    while edges:
        start = next(iter(edges))
        loop, cur = [start], start
        while True:
            nxts = edges.get(cur)
            if not nxts:
                break
            nxt = nxts.pop()
            if not nxts:
                del edges[cur]
            loop.append(nxt)
            cur = nxt
            if cur == start:
                break
        if loop and loop[-1] == start:
            loop = loop[:-1]
        loop = _simplify_rect_loop(loop)
        if len(loop) >= 3:
            loops.append(loop)
    return loops


class Design:
    def __init__(self, a):
        self.t, self.inner, self.wall_h = a.thickness_mm, a.inner_mm, a.wall_mm
        self.margin, self.flange, self.side = a.margin_mm, a.flange_mm, a.marker_mm
        self.tab_len = getattr(a, "tab_mm", 14.0)
        slot = getattr(a, "slot_mm", None)
        self.slot_w = float(slot) if slot else self.t
        self.outer = self.inner + 2 * self.t
        self.wall_len = self.inner + self.t
        self.panel_h = self.wall_h + self.t
        self.notch_len = self.wall_len - 2 * self.tab_len
        self.base_w = self.outer + 2 * self.margin
        self.base_d = self.outer + self.margin + self.flange
        self.y0 = -(self.outer / 2 + self.margin)          # plate's near edge in the bin frame
        self.y1 = self.outer / 2 + self.flange             # plate's flange edge
        self.flange_y = self.outer / 2 + self.flange / 2   # marker centres on the flange
        self.flange_x = a.marker_pitch_mm / 2

    def marker_centres(self) -> dict[str, tuple[float, float]]:
        return {"floor": (0.0, 0.0), "flange_left": (-self.flange_x, self.flange_y),
                "flange_right": (self.flange_x, self.flange_y)}

    def marker_ids(self, number: int) -> dict[str, int]:
        return {"floor": 10 * number, "flange_left": 10 * number + 1, "flange_right": 10 * number + 2}

    def walls(self) -> list[list[tuple[float, float]]]:
        """Footprints of the four pinwheel walls in the bin frame, in order A(-Y), B(+X), C(+Y), D(-X)."""
        h, t = self.inner / 2, self.t
        first = [(-h - t, -h - t), (h, -h - t), (h, -h), (-h - t, -h)]  # near wall, running +X
        return [[(x * c - y * s, x * s + y * c) for x, y in first]
                for c, s in ((1, 0), (0, 1), (-1, 0), (0, -1))]

    def wall_boxes(self) -> list[tuple[float, float, float, float, float, float]]:
        """(x0, x1, y0, y1) footprints of the four walls, simplified for solid drawing."""
        h, t = self.inner / 2, self.t
        return [(-h - t, h, -h - t, -h), (h, h + t, -h - t, h),
                (-h, h + t, h, h + t), (-h - t, -h, -h, h + t)]

    def plate(self) -> list[tuple[float, float]]:
        w = self.base_w / 2
        return [(-w, self.y0), (w, self.y0), (w, self.y1), (-w, self.y1)]

    def base_slots(self) -> list[tuple[float, float, float, float]]:
        """(x0, x1, y0, y1) of the through-slots under each wall's two end tabs.

        The slot width is centred on the wall thickness, so --slot-mm can tighten or loosen
        the fit without changing the wall panel or the slot length.
        """
        out, half = [], self.slot_w / 2
        for x0, x1, y0, y1 in self.wall_boxes():
            if abs((x1 - x0) - self.wall_len) < 1e-6:          # runs along X
                cy = (y0 + y1) / 2
                for a, b in ((x0, x0 + self.tab_len), (x1 - self.tab_len, x1)):
                    out.append((a, b, cy - half, cy + half))
            else:                                              # runs along Y
                cx = (x0 + x1) / 2
                for a, b in ((y0, y0 + self.tab_len), (y1 - self.tab_len, y1)):
                    out.append((cx - half, cx + half, a, b))
        return out

    def base_slot_holes(self) -> list[list[tuple[float, float]]]:
        """Four L-shaped through-holes; at each corner the two 14 x slot_w slots are merged.

        Merging removes the shared 2 mm edge: it is not material, so it should not be a cut
        contour and must not pick up a separate kerf offset.
        """
        slots = self.base_slots()
        h, o = self.inner / 2, self.outer / 2
        pairs = [(0, 6, (-o, -h, -o, -h)), (1, 2, (h, o, -o, -h)),
                 (3, 5, (h, o, h, o)), (4, 7, (-o, -h, h, o))]
        holes = []
        for i, j, (x0, x1, y0, y1) in pairs:
            holes.extend(_rect_union([slots[i], slots[j], (x0, x1, y0, y1)]))
        return holes

    def wall_outline(self) -> list[tuple[float, float]]:
        """Local side panel, s along the wall length (0..82), z from the table (0..42).

        Two 14 x t tabs at the ends, the middle bottom edge raised by t: the tabs go
        through the base slots and end flush with the plate's underside.
        """
        L, T, H = self.wall_len, self.tab_len, self.panel_h
        return [(0, 0), (T, 0), (T, self.t), (L - T, self.t), (L - T, 0), (L, 0), (L, H), (0, H)]

    def panel_area(self) -> float:
        return self.wall_len * self.panel_h - self.notch_len * self.t

    def weight_g(self, density: float = 1.19) -> float:
        slots = len(self.base_slots()) * self.tab_len * self.slot_w
        base = self.base_w * self.base_d - slots
        area = base + 4 * self.panel_area()
        return area * self.t / 1000 * density


def placements(spec: list[str]) -> list[tuple[float, float]]:
    return [tuple(float(v) for v in s.split(",")) for s in spec]


def bin_pose(r_cm: float, theta_deg: float) -> tuple[float, float, float]:
    """Centre (mm, base frame) and the direction of the bin's +Y (radially outwards)."""
    th = math.radians(theta_deg)
    return r_cm * 10 * math.cos(th), r_cm * 10 * math.sin(th), theta_deg


def to_base(pose: tuple[float, float, float], points) -> list[tuple[float, float]]:
    cx, cy, th = pose
    c, s = math.cos(math.radians(th - 90)), math.sin(math.radians(th - 90))
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in points]
