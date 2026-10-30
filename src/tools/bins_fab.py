"""Fabrication drawing set for the acrylic bins: A3 PDF, cut DXF and A4 marker stickers.

Called from make_bins.py so the drawings, the runtime config and the stickers all come
from one geometry model. Every value here is a design value: nothing has been cut or
measured. The drawing set is meant to be sent to an acrylic shop as-is; docs/bins/README.md
explains what each file is and what must be checked before ordering.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
from PIL import Image

import bins_core as core
from bins_core import A3, A4, ACCENT, BLOCK_MM, CATEGORIES, DIM, GUIDE, INK, NOTE, WARN, Canvas, Design

DRAWING_NO = "BIN-ACR-01"
PROJECT = "桌面分拣机械臂（Atlas 200I DK A2）"
DRAWING_TITLE = "亚克力分类垃圾桶 · 打版图纸"
TOTAL_PAGES = 6
DPI = 300

# isometric projection, camera at (+X, +Y, +Z); screen x right, y up (Canvas coordinates)
ISO_COS = math.cos(math.radians(30))
EXPLODE_OUT = 30.0
EXPLODE_UP = 27.0


# --------------------------------------------------------------------------- sheet furniture

def new_page() -> Canvas:
    return Canvas(A3, DPI)


def page_frame(c: Canvas, page_no: int, title: str, subtitle: str, a, material: str | None = None,
               qty: str = "4 桶 / 套", scale_txt: str = "1:1（零件）  1:2（开料）",
               unit_txt: str = "mm · 未注公差 ±0.2 / ±0.5°", total_pages: int = TOTAL_PAGES) -> None:
    """A3 border, page heading and title block. Content must stay above y=54 on x>226."""
    if material is None:
        material = f"PMMA 透明浇铸 t={a.thickness_mm:g}"
    c.rect(8, 8, 404, 281, width_mm=0.6)
    c.rect(10, 10, 400, 277, width_mm=0.28)
    c.text(14, 282, title, 5.6)
    c.text(14, 276, subtitle, 3.0, NOTE)
    c.line((14, 272), (406, 272), width_mm=0.2)

    x0, y0, w = 226.0, 10.0, 182.0
    bounds = [10.0, 17.0, 24.0, 31.0, 38.0, 47.0, 54.0]
    c.rect(x0, y0, w, 44, width_mm=0.4)
    for y in bounds[1:-1]:
        c.line((x0, y), (x0 + w, y), width_mm=0.18)
    x1, x2, x3 = x0 + 14, x0 + 96, x0 + 114
    c.line((x1, bounds[1]), (x1, bounds[-1]), width_mm=0.18)
    for xx in (x2, x3):
        c.line((xx, bounds[0]), (xx, bounds[-1]), width_mm=0.18)
    rows = [
        ("项目", PROJECT, "图号", DRAWING_NO),
        ("图名", "亚克力分类垃圾桶", "版本", a.revision),
        ("材料", material, "日期", a.drawing_date),
        ("比例", scale_txt, "张次", f"{page_no}/{total_pages}"),
        ("单位", unit_txt, "数量", qty),
    ]
    for i, (l1, v1, l2, v2) in enumerate(rows):
        yc = (bounds[-1 - i] + bounds[-2 - i]) / 2
        c.text(x0 + 2, yc, l1, 2.5)
        c.text(x1 + 2, yc, v1, 2.7)
        c.text(x2 + 2, yc, l2, 2.5)
        c.text(x3 + 2, yc, v2, 2.7)
    yc = (bounds[0] + bounds[1]) / 2
    c.text(x0 + 2, yc, "设计：______   审核：______", 2.5)
    c.text(x2 + 2, yc, "加工：外协激光切割", 2.4)


def grid_table(c: Canvas, x: float, y_top: float, widths: list[float], rows: list[tuple],
               row_h: float = 7.0, header: bool = True, mm: float = 2.7, align: str | None = None) -> None:
    total = sum(widths)
    for r, row in enumerate(rows):
        y = y_top - (r + 1) * row_h
        if header and r == 0:
            c.rect(x, y, total, row_h, fill=(230, 237, 247), width_mm=0.18)
        c.line((x, y), (x + total, y), width_mm=0.18)
        for j, cell in enumerate(row):
            cx = x + sum(widths[:j])
            if header and r == 0:
                c.text(cx + 1.5, y + row_h / 2, str(cell), mm)
            elif align and j < len(align) and align[j] == "c":
                c.text(cx + widths[j] / 2, y + row_h / 2, str(cell), mm)
            else:
                c.text(cx + 1.5, y + row_h / 2, str(cell), mm)
    for j in range(len(widths) + 1):
        cx = x + sum(widths[:j])
        c.line((cx, y_top), (cx, y_top - len(rows) * row_h), width_mm=0.18)


def note_lines(c: Canvas, x: float, y_top: float, lines: list[str], mm: float = 3.0, lead: float = 6.4,
               color=INK) -> float:
    for i, s in enumerate(lines):
        c.text(x, y_top - i * lead, s, mm, color, "lm")
    return y_top - len(lines) * lead


# --------------------------------------------------------------------------- isometric sketches

def _box_faces(box) -> list[tuple[tuple[float, float, float], list[tuple[float, float, float]]]]:
    x0, x1, y0, y1, z0, z1 = box
    return [
        ((1, 0, 0), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
        ((-1, 0, 0), [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)]),
        ((0, 1, 0), [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)]),
        ((0, -1, 0), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
        ((0, 0, 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]),
        ((0, 0, -1), [(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)]),
    ]


def _iso(p) -> tuple[float, float]:
    return ((p[0] - p[1]) * ISO_COS, p[2] - (p[0] + p[1]) * 0.5)


def _corner_points(box):
    x0, x1, y0, y1, z0, z1 = box
    return [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


COL_PLATE = {(0, 0, 1): (231, 238, 246), (1, 0, 0): (204, 217, 229), (0, 1, 0): (187, 203, 219)}
COL_WALL = {(0, 0, 1): (212, 231, 248), (1, 0, 0): (174, 204, 231), (0, 1, 0): (150, 186, 219)}


def draw_iso(c: Canvas, d: Design, region: tuple[float, float, float, float], exploded: bool = False,
             block: bool = False, balloons: bool = True, legend: bool = True) -> None:
    """Axonometric view fitted into a page region (x0, y0, x1, y1) in page millimetres."""
    x0, y0, x1, y1 = region
    panels: list[tuple[tuple, str]] = [(
        (-d.base_w / 2, d.base_w / 2, d.y0, d.y1, 0.0, d.t), "plate")]
    normals = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    for idx, (wx0, wx1, wy0, wy1) in enumerate(d.wall_boxes()):
        dx, dy, dz = 0.0, 0.0, 0.0
        if exploded:
            n = normals[idx]
            dx, dy, dz = n[0] * EXPLODE_OUT, n[1] * EXPLODE_OUT, EXPLODE_UP
        # the wall panel starts on the plate top; in the exploded view the two tabs are
        # drawn below it, while in the assembled view they are hidden inside the slots
        panels.append(((wx0 + dx, wx1 + dx, wy0 + dy, wy1 + dy, d.t + dz, d.panel_h + dz), "wall"))
        if exploded:
            if abs((wx1 - wx0) - d.wall_len) < 1e-6:
                tabs = [(wx0, wx0 + d.tab_len), (wx1 - d.tab_len, wx1)]
                for a, b in tabs:
                    panels.append(((a + dx, b + dx, wy0 + dy, wy1 + dy, 0.0 + dz, d.t + dz), "wall"))
            else:
                tabs = [(wy0, wy0 + d.tab_len), (wy1 - d.tab_len, wy1)]
                for a, b in tabs:
                    panels.append(((wx0 + dx, wx1 + dx, a + dy, b + dy, 0.0 + dz, d.t + dz), "wall"))

    block_box = (-BLOCK_MM / 2, BLOCK_MM / 2, -BLOCK_MM / 2, BLOCK_MM / 2, d.t + d.wall_h + 10, d.t + d.wall_h + 10 + BLOCK_MM)
    probe = [p[0] for p in panels] + ([block_box] if block else [])
    pts = [_iso(p) for box in probe for p in _corner_points(box)]
    sx = [p[0] for p in pts]
    sy = [p[1] for p in pts]
    span_x, span_y = max(sx) - min(sx), max(sy) - min(sy)
    scale = min((x1 - x0 - 12) / span_x, (y1 - y0 - 12) / span_y)
    mid_x, mid_y = (min(sx) + max(sx)) / 2, (min(sy) + max(sy)) / 2
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

    def P(p):
        q = _iso(p)
        return (cx + (q[0] - mid_x) * scale, cy + (q[1] - mid_y) * scale)

    # exploded guide lines: from the seat to the exploded panel, drawn behind the boxes
    if exploded:
        for idx, (wx0, wx1, wy0, wy1) in enumerate(d.wall_boxes()):
            mx, my = (wx0 + wx1) / 2, (wy0 + wy1) / 2
            n = normals[idx]
            c.line(P((mx, my, d.t)), P((mx + n[0] * EXPLODE_OUT, my + n[1] * EXPLODE_OUT, d.t + EXPLODE_UP)),
                   GUIDE, 0.18, dash_mm=1.6)

    faces = []
    for box, kind in panels:
        palette = COL_PLATE if kind == "plate" else COL_WALL
        for n, corners in _box_faces(box):
            if n[0] + n[1] + n[2] <= 0:
                continue
            depth = sum(c[0] + c[1] + c[2] for c in corners) / 4
            faces.append((depth, [P(c) for c in corners], palette[n]))
    # sticker squares and the eight base slots sit on the plate's top face; give them that
    # face's depth so the painter's ordering still lets the walls hide them when assembled
    plate_top_depth = (d.y0 + d.y1) / 2 + d.t
    if exploded:
        for hole in d.base_slot_holes():
            corners = [(x, y, d.t + 0.03) for x, y in hole]
            faces.append((plate_top_depth + 0.05, [P(c) for c in corners], (255, 255, 255)))
    for key, (mx, my) in d.marker_centres().items():
        if not exploded and key == "floor":
            continue
        h = d.side / 2
        corners = [(mx - h, my - h, d.t + 0.05), (mx + h, my - h, d.t + 0.05),
                   (mx + h, my + h, d.t + 0.05), (mx - h, my + h, d.t + 0.05)]
        depth = max(sum(c[0] + c[1] + c[2] for c in corners) / 4, plate_top_depth + 0.1)
        faces.append((depth, [P(c) for c in corners], (28, 28, 28)))
    for depth, poly, colour in sorted(faces, key=lambda f: f[0]):
        outline = (28, 28, 28) if colour == (28, 28, 28) else ((70, 70, 70) if colour == (255, 255, 255) else (52, 72, 96))
        c.poly(poly, outline=outline, fill=colour, width_mm=0.16)

    if block:
        for n, corners in _box_faces(block_box):
            if n[0] + n[1] + n[2] <= 0:
                continue
            fill = (250, 234, 214) if n == (0, 0, 1) else (238, 214, 188)
            c.poly([P(c) for c in corners], fill=fill, outline=ACCENT, width_mm=0.22)
        c.line(P((0, 0, block_box[4])), P((0, 0, d.t + d.wall_h)), ACCENT, 0.2, dash_mm=1.6)
        top = [P(c) for c in [(block_box[0], block_box[2], block_box[5]), (block_box[1], block_box[2], block_box[5]),
                              (block_box[1], block_box[3], block_box[5]), (block_box[0], block_box[3], block_box[5])]]
        c.text((top[0][0] + top[2][0]) / 2, max(p[1] for p in top) + 5,
               "3 cm 物块（松手面 +10）", 2.6, ACCENT, "mm")

    if balloons and exploded:
        corner = P((-d.base_w / 2, d.y0, d.t))
        bx, by = corner[0], corner[1] + 14
        c.line((bx, by - 4.6), corner, INK, 0.18)
        c.balloon(bx, by, "1", 4.6, 3.0)
        c.text(bx + 6, by, "底板 ×4", 2.8, INK, "lm")
        for idx, (wx0, wx1, wy0, wy1) in enumerate(d.wall_boxes()):
            n = normals[idx]
            p = P(((wx0 + wx1) / 2 + n[0] * EXPLODE_OUT, (wy0 + wy1) / 2 + n[1] * EXPLODE_OUT,
                   d.panel_h / 2 + EXPLODE_UP))
            c.balloon(p[0], p[1], "2", 4.2, 2.8)
    if exploded and legend:
        lx, ly = x0 + 4, y0 + 4
        c.balloon(lx + 4, ly + 12, "1", 3.6, 2.6)
        c.text(lx + 11, ly + 12, "底板（每桶 1 块，8 个插槽，共 4 块）", 2.8, INK, "lm")
        c.balloon(lx + 4, ly + 3.5, "2", 3.6, 2.6)
        c.text(lx + 11, ly + 3.5, f"侧板（每桶 4 块，两端 {d.tab_len:g}×{d.t:g} 插舌，共 16 块）", 2.8, INK, "lm")


# --------------------------------------------------------------------------- pages

def page_cover(c: Canvas, d: Design, a, stats: dict) -> None:
    page_frame(c, 1, "01 封面 · 加工说明与零件表",
               f"插板式 · 透明浇铸 PMMA t={d.t:g} · 底板开槽 + 侧板插舌 + 溶剂粘接 · 一套 4 桶 · 单位 mm", a,
               scale_txt="示意（轴测）/ 表内 mm")
    c.text(16, 256, "亚克力分类垃圾桶", 10.0)
    c.text(16, 247, "打版图纸 · 插板式 4 桶 / 套", 5.0, DIM)
    c.line((16, 242), (222, 242), INK, 0.5)
    c.text(16, 235, "依据：docs/atlas-200i-dk-a2/12-实体容器选型约束.md · 图纸编号 BIN-ACR-01", 2.8, NOTE)

    draw_iso(c, d, (14, 92, 226, 230), exploded=True, balloons=True, legend=True)

    grid_table(c, 236, 254, [10, 22, 52, 13, 13, 54],
               [("件号", "名称", "单件尺寸 / mm", "每桶", "每套", "材料 / 说明"),
                ("①", "底板", f"{d.base_w:g} × {d.base_d:g} × {d.t:g}", "1", "4", f"8 个逻辑槽 14×{d.slot_w:g}（4 个 L 孔）"),
                ("②", "侧板", f"{d.wall_len:g} × {d.panel_h:g} × {d.t:g}", "4", "16", f"两端 {d.tab_len:g}×{d.t:g} 插舌，四块相同"),
                ("③", "贴纸", "耳板条 94×31 / 桶底片 79×79", "2 片", "8 片", "1 张 A4 出 8 片，共 3 码/桶")],
               row_h=7.2, mm=2.5)

    grid_table(c, 236, 205, [14, 98, 44],
               [("图号", "内容", "张次"),
                ("01", "封面 / 加工说明 / 零件表", "1/6"),
                ("02", "零件图（1:1）", "2/6"),
                ("03", "装配图（1:1）", "3/6"),
                ("04", "装配顺序 / 插接节点", "4/6"),
                ("05", "开料排版（1:2）", "5/6"),
                ("06", "附录：摆放核对（非加工）", "6/6")],
               row_h=6.0, mm=2.5)

    c.text(236, 158, f"关键尺寸与重量（按 t = {d.t:g} mm 计算）", 3.6)
    grid_table(c, 236, 153, [58, 112],
               [("项目", "数值"),
                ("内口 / 外廓", f"{d.inner:g} × {d.inner:g} / {d.outer:g} × {d.outer:g}"),
                ("壁高 / 总高", f"{d.wall_h:g} / {d.t + d.wall_h:g}"),
                ("底板", f"{d.base_w:g} × {d.base_d:g} × {d.t:g}（8 槽）"),
                ("侧板", f"{d.wall_len:g} × {d.panel_h:g} × {d.t:g}（16 块）"),
                ("定位码", "3 枚/桶，20 × 20，耳板码中心距 50"),
                ("单桶 / 整套重量", f"{d.weight_g():.0f} g / {d.weight_g() * a.bins:.0f} g（估）"),
                ("最小料板", f"{stats['sheet'][0]:g}×{stats['sheet'][1]:g}，利用率 {stats['sheet_util']:.1f}%")],
               row_h=6.2, mm=2.5)

    c.text(236, 100, "加工要求（给外协）", 3.4)
    note_lines(c, 236, 94, [
        f"1. 透明浇铸亚克力板；本图按 t = {d.t:g} 计算，下单前必须实测板厚并按实测值重算。",
        "2. CO₂ 激光切割，缝宽 0.1–0.2，按名义尺寸切；全部为直边 + 直槽，无浅雕。",
        f"3. 插板：底板 8 个逻辑槽 14×{d.slot_w:g}（四角合并为 4 个 L 形孔），侧板两端 14×{d.t:g} 插舌；槽宽默认 = 板厚。",
        "4. 插舌插到底与底板下表面平齐；槽口与侧板底边涂亚克力溶剂胶，24 h 固化。",
        "5. 数量：底板 4 + 侧板 16（一套 4 桶）。建议多切 2 块侧板备用。",
    ], 2.7, 5.4)

    c.text(16, 86, "状态与使用边界", 3.6)
    note_lines(c, 16, 80, [
        "· 本图为首版打版设计值（" + a.drawing_date + "），尚未加工、未实测；打版后按实测板厚与配合回写。",
        "· 板厚是主耦合尺寸：t 变化时侧板长/板高/插舌高随 t 变；槽宽默认 = t，松紧可用 --slot-mm 单独调。",
        "· 插板式：底板开槽 + 侧板插舌，取消浅雕；透明侧壁保留相机俯视核对能力。",
        "· 定位码为 DICT_5X5_50，边长 20 mm；耳板两枚中心距 50 mm，码顶朝 +Y（耳板方向）。",
        f"· 本图不含电气、机械臂安装内容；总高 {d.t + d.wall_h:g} 不含防滑垫；摆放可达性见第 6 页（仅规划核对，非加工）。",
        "· 打印：A3、100%、勿缩放；本 PDF 为 300 dpi，尺寸以标注为准。",
    ], 2.8, 5.8)


def page_parts(c: Canvas, d: Design, a) -> None:
    page_frame(c, 2, "02 零件图（1:1）", f"插板式 · 底板 8 个 14×{d.slot_w:g} 通槽 · 侧板两端 14×{d.t:g} 插舌 · 无浅雕", a,
               scale_txt="1:1")
    ox, oy = 75.0, 170.0

    # ---- base plate
    c.text(ox, 258, f"① 底板 {d.base_w:g}×{d.base_d:g}×{d.t:g}（每桶 1 块 · 共 4 块）", 4.0, INK, "mm")
    c.poly([(ox + x, oy + y) for x, y in d.plate()], width_mm=0.5)
    c.rect(ox - d.outer / 2, oy - d.outer / 2, d.outer, d.outer, outline=GUIDE, dash_mm=1.8, width_mm=0.18)
    for hole in d.base_slot_holes():
        c.poly([(ox + x, oy + y) for x, y in hole], fill="white", width_mm=0.35)
    c.line((ox - 4, oy), (ox + 4, oy), GUIDE, 0.2)
    c.line((ox, oy - 4), (ox, oy + 4), GUIDE, 0.2)
    c.text(ox + 5, oy + 4, "原点 0,0", 2.1, NOTE)
    c.text(ox, oy + 74 + 6, f"8 个逻辑槽 14×{d.slot_w:g}（四角合并为 4 个 L 形孔，共边已删）", 2.5, DIM, "mm")

    w, y0, y1 = d.base_w / 2, d.y0, d.y1
    c.dim((ox - w, oy + y0), (ox + w, oy + y0), -13, "94")
    c.dim((ox + w, oy + y0), (ox + w, oy + y1), -13, "121")
    c.dim((ox, oy + d.outer / 2), (ox, oy + y1), 8, "32")
    for label, lx, ly in (("A", ox, oy - 41), ("B", ox + 41, oy),
                          ("C", ox, oy + 41), ("D", ox - 41, oy)):
        c.text(lx, ly, label, 2.6, DIM, "mm")
    c.text(16, 103, "三边留边 5，耳板侧 32；插槽位于四壁足迹内，槽内边 = 内口边界", 2.5, NOTE)
    c.text(16, 98, "四壁外廓 84×84 由 8 个槽的位置保证；灰色虚线 84 仅为尺寸参考，不切割、不浅雕", 2.5, NOTE)

    # ---- side panel
    wx, wy = 232.0, 210.0
    c.text(wx + d.wall_len / 2, 258, f"② 侧板 {d.wall_len:g}×{d.panel_h:g}×{d.t:g}（每桶 4 块 · 共 16 块 · 四块相同）", 3.6, INK, "mm")
    c.poly([(wx + x, wy + y) for x, y in d.wall_outline()], width_mm=0.5, fill=(214, 232, 248))
    c.dim((wx, wy), (wx + d.wall_len, wy), 8, "82")
    c.dim((wx + d.wall_len, wy), (wx + d.wall_len, wy + d.panel_h), -10, "42")
    c.text(wx, 203, f"两端插舌 {d.tab_len:g}×{d.t:g}：插进底板槽，插到底后与底板下表面平齐；中间底边上移 {d.t:g} 坐在底板上", 2.4, NOTE)
    c.text(wx, 198, f"板厚 t = {d.t:g}；四块可互换（前后可翻面）；大面为外露面，两端短边端面与相邻侧板内面相接", 2.4, NOTE)

    # ---- slot coordinate table
    c.text(232, 190, "底板插槽坐标（桶坐标系，单位 mm）", 3.2)
    rows = [("槽", "所属", "x0…x1", "y0…y1")]
    names = ["A 近边", "A 近边", "B 右侧", "B 右侧", "C 远边", "C 远边", "D 左侧", "D 左侧"]
    for i, (x0, x1, y0_, y1_) in enumerate(d.base_slots()):
        rows.append((str(i + 1), names[i], f"{x0:g}…{x1:g}", f"{y0_:g}…{y1_:g}"))
    grid_table(c, 232, 186, [10, 42, 60, 60], rows, row_h=5.8, mm=2.4,
               align=[None, None, "c", "c"])
    c.text(232, 130, f"x 向右、y 向耳板；原点 = 内口中心。槽位 4 次旋转对称；角部两槽合并为一个 L 形孔（共边已删）；逻辑单槽 14×{d.slot_w:g}；A 近边、B 右侧、C 远边、D 左侧。", 2.3, NOTE)
    c.text(232, 122, "侧板插接参数", 3.2)
    grid_table(c, 232, 118, [52, 118],
               [("项目", "数值 / mm"),
                ("侧板长", f"{d.wall_len:g}"),
                ("侧板总高", f"{d.panel_h:g}"),
                ("端部插舌", f"{d.tab_len:g} × {d.t:g}（两端各 1）"),
                ("中间底边上移", f"{d.t:g}（缺口长 {d.notch_len:g}）"),
                ("槽宽", f"{d.slot_w:g}（默认 = 板厚）"),
                ("板厚", f"{d.t:g}")],
               row_h=5.8, mm=2.4, align=[None, "c"])

    # ---- fit / tolerance table
    c.text(16, 93, "加工与插接要求", 3.4)
    grid_table(c, 16, 89, [34, 175],
               [("项目", "要求"),
                ("切割", "CO₂ 激光，缝宽 ≤ 0.2；按名义尺寸切；全部直边/直槽，无浅雕"),
                ("插槽", f"14×{d.slot_w:g}；槽宽默认 = 板厚；仅调槽宽用 --slot-mm 1.9–2.1"),
                ("插舌", f"14 × {d.t:g}（两端各 1）；插到底与底板下表面平齐"),
                ("未注公差", "线性 ±0.2；垂直度 ±0.5°；内口 80 ± 0.3、外廓 84 ± 0.3"),
                ("数量", "底板 4 + 侧板 16；建议多切 2 块侧板作为试插/备用")],
               row_h=7.0, mm=2.4)

def page_assembly(c: Canvas, d: Design, a) -> None:
    page_frame(c, 3, "03 装配图（1:1）", "插板式：底板 8 槽 + 侧板 8 插舌 + 四角对接胶接 · 定位码位置与朝向", a,
               scale_txt="1:1 + 轴测示意")
    ox, oy = 125.0, 185.0

    # ---- top view
    c.text(72, 266, "装配俯视 1:1（含码）", 3.6, anchor="lm")
    for wall in d.walls():
        c.poly([(ox + x, oy + y) for x, y in wall], fill=(206, 226, 246), width_mm=0.3)
    c.poly([(ox + x, oy + y) for x, y in d.plate()], width_mm=0.45)
    for hole in d.base_slot_holes():
        c.poly([(ox + x, oy + y) for x, y in hole], outline=GUIDE, dash_mm=1.2, width_mm=0.18)
    for key, (mx, my) in d.marker_centres().items():
        c.marker(d.marker_ids(1)[key], ox + mx, oy + my, d.side)
    c.text(ox - 26, oy + 22, "桶底码 10", 2.4, NOTE, "mm")
    c.line((ox - 13, oy + 13), (ox - 22, oy + 18), NOTE, 0.18)
    c.text(ox - 25, oy + d.flange_y + 13, "11", 2.3, NOTE, "mm")
    c.text(ox + 25, oy + d.flange_y + 13, "12", 2.3, NOTE, "mm")
    c.text(ox, oy - 20, "内侧空腔 80×80", 2.4, NOTE, "mm")

    w = d.base_w / 2
    c.dim((ox - d.outer / 2, oy - d.outer / 2), (ox + d.outer / 2, oy - d.outer / 2), -13, "外廓 84", mm=2.6)
    c.dim((ox + w, oy + d.y0), (ox + w, oy + d.y1), -12, "121")
    c.dim((ox - w, oy + d.outer / 2), (ox - w, oy + d.y1), 12, "32")
    c.dim((ox - d.inner / 2, oy - d.inner / 2), (ox + d.inner / 2, oy - d.inner / 2), 8, "内口 80", mm=2.6)
    c.dim((ox + d.inner / 2, oy - d.inner / 2), (ox + d.inner / 2, oy + d.inner / 2), 8, "80", mm=2.6)

    c.line((ox + d.outer / 2 - 2, oy - d.outer / 2 + 2), (ox + 52, oy - 52), WARN, 0.28)
    c.text(ox + 53, oy - 52, "I", 2.9, WARN, "lm")
    c.text(ox + 30, oy - d.outer / 2 - 16, "节点 I 详见第 4 页", 2.3, WARN, "lm")

    # cutting plane A-A (view direction: -X, arrows point left)
    c.line((ox, oy + d.y1 + 3), (ox, oy + d.y0 - 3), DIM, 0.25, dash_mm=2.0)
    for yy in (oy + d.y1 + 3, oy + d.y0 - 3):
        c.arrow((ox, yy), (-1, 0), 2.4, DIM)
        c.text(ox + 3, yy, "A", 2.6, DIM, "lm")

    # ---- section A-A
    sx, sy = 70.0, 62.0  # bin y=0 at sx, z=0 at sy
    c.text(23, 112, "A—A 剖视 1:1", 3.8)

    def S(y, z):
        return (sx + y, sy + z)

    c.rect(sx + d.y0, sy, d.base_d, d.t, fill=(228, 228, 228), width_mm=0.35)
    c.hatch_rect(sx + d.y0, sy, d.base_d, d.t, spacing=1.5, angle=45)
    for y in (-d.outer / 2, d.outer / 2 - d.t):
        c.rect(sx + y, sy + d.t, d.t, d.wall_h, fill=(214, 232, 248), width_mm=0.35)
        c.hatch_rect(sx + y, sy + d.t, d.t, d.wall_h, spacing=1.5, angle=45)
    # marker stickers: one on the floor, two on the flange
    c.rect(sx - d.side / 2, sy + d.t, d.side, 0.5, fill=INK, width_mm=0.05)
    c.rect(sx + d.flange_y - d.side / 2, sy + d.t, d.side, 0.5, fill=INK, width_mm=0.05)
    c.line(S(d.flange_y, d.t + 4), S(d.flange_y, d.t + 1.0), NOTE, 0.2, dash_mm=1.0)
    c.line((sx + d.outer / 2 - 2, sy + d.t + 5.5), (sx + d.outer / 2 + 8, sy + d.t + 2.2), NOTE, 0.18)
    c.text(sx + d.outer / 2 - 2, sy + d.t + 5.5, "定位码（贴纸）", 2.3, NOTE, "rm")
    # table line
    c.line((sx - 52, sy), (sx + 82, sy), GUIDE, 0.25, dash_mm=3.0)
    c.text(sx + 80, sy - 3.5, "台面", 2.3, GUIDE)
    # release plane: 3 cm block bottom, 10 mm above the rim
    zb = d.t + d.wall_h + 10
    c.line((sx - BLOCK_MM / 2, sy + zb), (sx + BLOCK_MM / 2, sy + zb), ACCENT, 0.25, dash_mm=1.8)
    c.arrow((sx + BLOCK_MM / 2 + 1, sy + zb), (0, -1), 2.2, ACCENT)
    c.text(sx + BLOCK_MM / 2 + 4, sy + zb, "3 cm 物块松手面 = 42 + 10", 2.4, ACCENT, "lm")
    c.dim(S(-d.inner / 2, d.t), S(d.inner / 2, d.t), 8, "80", mm=2.5)
    c.dim(S(d.y0, 0), S(d.y1, 0), -12, "121")
    c.dim(S(d.outer / 2, 0), S(d.y1, 0), -7, "32", mm=2.5)
    c.dim(S(d.outer / 2, d.t), S(d.outer / 2, d.t + d.wall_h), -12, "40", mm=2.5)
    c.dim(S(d.outer / 2, 0), S(d.outer / 2, d.t + d.wall_h), -21, "42", mm=2.5)
    c.text(23, 43, f"板厚 t = {d.t:g}；壁高 {d.wall_h:g}；总高 {d.t + d.wall_h:g}（不含防滑垫）；插舌节点见第 4 页", 2.5, NOTE)

    # ---- assembled axonometric
    draw_iso(c, d, (198, 138, 408, 264), exploded=False, block=True, balloons=False, legend=False)
    c.text(303, 142, "成品轴测（示意）", 3.2, INK, "mm")

    # ---- notes
    c.text(198, 130, "装配与贴码要点", 3.4)
    note_lines(c, 198, 124, [
        "1. 四壁插舌对槽向下插到底；按 A 近边 → B 右侧 → C 远边 → D 左侧顺序，四角首尾相接。",
        "2. 先干装试配：插舌与底板下表面平齐、内口 80±0.3、外廓 84±0.3、四角接缝 ≤0.3，再上胶。",
        "3. 槽口、插舌与四角接缝涂亚克力溶剂胶；外侧贴美纹纸防溢流，24 h 固化。",
        "4. 贴码：耳板条上边对齐底板外边，桶底片落入内口；码顶朝 +Y（耳板方向），两枚耳板码中心距 50±0.3。",
        "5. 透明侧壁保证相机可俯视核对桶内物块；贴纸用哑光面，避免反光影响识别。",
        "6. 底板四角贴 ⌀8×2 mm 硅胶圆垫；贴后总高与码面高按实测回写（约 +1.5 mm）。",
        "7. 插舌与底板下表面平齐；若插不到底，先检查槽宽/毛刺，不要强行敲入。",
        "8. 轴测图中的 3 cm 物块仅示意松手高度，不属于交付零件。",
        "9. 封面展开轴测中桶底码与近侧壁顶角投影重合，仅示意；位置以俯视图为准。",
        "10. 俯视图中 4 个 L 形槽孔用灰色虚线表示（8 个逻辑槽合并），插接后隐藏在侧板下。",
    ], 2.65, 6.0)

def draw_sequence_step(c: Canvas, d: Design, ox: float, oy: float, step: int, k: float = 0.5) -> None:
    """Plan view of assembly step 1..4 at scale k; walls up to `step` are in place."""
    def P(p):
        return (ox + p[0] * k, oy + p[1] * k)

    c.poly([P(p) for p in d.plate()], width_mm=0.3, fill=(248, 248, 248))
    walls = d.walls()
    for i, wall in enumerate(walls):
        if i <= step:
            colour = ACCENT if i == step else (170, 205, 236)
            c.poly([P(p) for p in wall], fill=colour, width_mm=0.25)
    c.circle(ox, oy, 5.0, fill="white", width_mm=0.3)
    c.text(ox, oy, str(step + 1), 3.2, INK, "mm")


def page_sequence(c: Canvas, d: Design, a) -> None:
    page_frame(c, 4, "04 装配顺序与插接节点", "插板式：四壁插舌对槽向下插到底 · 四角首尾相接 · 节点 I 为插舌-底槽放大 4:1", a,
               scale_txt="1:2 顺序 / 4:1 节点")
    centers = [48.0, 108.0, 168.0, 228.0]
    labels = [("近边 A（−Y）", "两端插舌入槽，插到底"),
              ("右侧 B（+X）", "插舌入槽，端面顶住 A"),
              ("远边 C（+Y）", "插舌入槽，端面顶住 B"),
              ("左侧 D（−X）", "插舌入槽，收口顶住 C 与 A")]
    for i, (cx, (title, sub)) in enumerate(zip(centers, labels)):
        draw_sequence_step(c, d, cx, 214, i)
        c.text(cx, 183, title, 3.1, INK, "mm")
        c.text(cx, 177, sub, 2.4, NOTE, "mm")
        if i < 3:
            c.arrow((cx + 27, 214), (1, 0), 3.0, DIM)
    c.text(28, 266, "装配顺序 1:2（箭头 = 下一块侧板）", 3.6)

    # ---- tab / slot detail 4:1, section through a tab at x = 30
    def D(y, z):
        return (330 + (y + 41) * 4, 190 + z * 4)

    c.text(282, 266, "节点 I · 插舌-底槽 4:1（插到底平齐）", 3.4)
    # base plate cross-section, split by the 2 mm slot
    for y0, y1 in ((-47, -42), (-40, -33)):
        c.rect(*D(y0, 0), (y1 - y0) * 4, 2 * 4, fill=(228, 228, 228), width_mm=0.3)
        c.hatch_rect(*D(y0, 0), (y1 - y0) * 4, 2 * 4, spacing=1.4, angle=45)
    # wall panel: 2 mm thick, passes through the slot; tab is the bottom 2 mm
    c.rect(*D(-42, 0), 2 * 4, 12 * 4, fill=(214, 232, 248), width_mm=0.35)
    c.hatch_rect(*D(-42, 0), 2 * 4, 12 * 4, spacing=1.4, angle=45)
    # glue on both slot walls
    c.line(D(-42, 0), D(-42, 2), WARN, 0.6)
    c.line(D(-40, 0), D(-40, 2), WARN, 0.6)
    c.text(*D(-47, -2.6), f"底板 {d.t:g}（开 14×{d.slot_w:g} 通槽）", 2.3, NOTE, "lm")
    c.text(*D(-39.2, 7.2), f"侧板 {d.t:g} / 插舌 {d.tab_len:g}×{d.t:g}", 2.3, INK, "lm")
    c.line(D(-46.5, 2.8), D(-42.3, 1.2), WARN, 0.2)
    c.text(*D(-46.5, 2.8), "槽壁涂胶", 2.3, WARN, "lm")
    c.dim(D(-42, 9), D(-40, 9), 6, f"{d.slot_w:g}", mm=2.2)
    c.dim(D(-41, 0), D(-41, 2), -10, f"{d.t:g}", mm=2.2)
    c.text(*D(-47, 12.6), "插舌插到底后与底板下表面平齐；槽口、插舌、内侧接缝补胶", 2.2, NOTE)

    # ---- process table
    c.text(14, 162, "粘接工序卡（单桶）", 3.4)
    grid_table(c, 14, 157, [10, 116, 58, 46],
               [("步", "操作", "工具 / 材料", "检查"),
                ("1", "撕去粘接面保护膜，异丙醇清洁", "无尘布、异丙醇", "无油污、无毛屑"),
                ("2", "底板 8 槽去毛刺；侧板两端插舌试插", "美纹纸、锉刀", "插舌能插到底"),
                ("3", "按 A→B→C→D 顺序干装，四角首尾相接", "直角靠模", "内口 80±0.3、插舌平齐"),
                ("4", "槽口、插舌、四角接缝毛细渗胶", "亚克力溶剂胶、注射针", "胶线连续、无溢流"),
                ("5", "室温固化 24 h，不移动", "—", "无白化、无错位"),
                ("6", "贴耳板条与桶底片，压平", "A4 贴纸、刮板", "码顶朝 +Y、中心距 50±0.3"),
                ("7", "四角贴硅胶防滑垫，记录贴后总高", "8×2 mm 硅胶垫", "底面平、不晃动")],
               row_h=6.8, mm=2.3)

    # ---- post-glue check
    c.text(14, 96, "粘接后检查（单桶）", 3.3)
    note_lines(c, 14, 90, [
        "· 内口 80 ± 0.3；外廓 84 ± 0.3；对角线 113.1 ± 0.5（80 方）",
        f"· 插舌与底板下表面平齐；总高（不含垫）{d.t + d.wall_h:g} ± 0.5",
        "· 四角接缝、槽口胶线连续，无溢流、无白化",
        "· 定位码三枚均可识别；耳板码中心距 50 ± 0.3，码顶朝 +Y",
        "· 四角防滑垫齐全，放桌面不晃；记录实测板厚与内口",
    ], 2.5, 6.2)

    # ---- sticker notes
    c.text(260, 150, "贴码要点（贴纸见单独 A4）", 3.3)
    note_lines(c, 260, 144, [
        "· 耳板条 94×31：上边对齐底板外边，两侧与板边齐平。",
        "· 桶底片 79×79：落入内口，四边各留约 0.5 mm。",
        "· 码顶朝 +Y（耳板方向）；两枚耳板码中心距 50±0.3。",
        "· 打印必须 100%；先量 100 mm 校验尺，不足则重打。",
        "· 贴前擦净、贴后用刮板压平，避免气泡和翘边。",
        "· 建议多打印 1 份备用；贴错不要硬撕，整片更换。",
        "· 贴码后从相机视角抽查：三枚码均应可识别、无强反光。",
    ], 2.4, 5.2)

    # ---- delivery / spares
    c.text(260, 100, "交付与备件", 3.3)
    note_lines(c, 260, 94, [
        "· 图纸：本 PDF 共 6 页；贴纸 A4 单独 1 页。",
        "· DXF：bins_acrylic_v1_cut.dxf（排版）+ bins_acrylic_v1_cut_parts.dxf。",
        "· 主料：底板 4 + 侧板 16；建议多切 2 块侧板备用。",
        "· 辅料：贴纸 1 份、8×2 mm 硅胶垫 16 片。",
        "· 交付前按第 2 页表与第 4 页检查表自检。",
    ], 2.4, 5.2)

def sheet_layout(d: Design, bins: int, gap: float = 2.0, sheet: tuple[float, float] = (297.0, 388.0)) -> list[tuple[str, int, float, float, float, float]]:
    """Fabrication nesting: plates rotated 90° in one column, 16 notched walls in a 4x4 block."""
    out: list[tuple[str, int, float, float, float, float]] = []
    for i in range(bins):
        out.append(("plate", i, 0.0, i * (d.base_w + 4.0), d.base_d, d.base_w))
    x0 = d.base_d + gap
    cols = 4
    for i in range(4 * bins):
        out.append(("wall", i, x0 + (i % cols) * (d.panel_h + gap),
                    (i // cols) * (d.wall_len + gap), d.panel_h, d.wall_len))
    used_w = max(x + w for _, _, x, _, w, _ in out)
    used_h = max(y + h for _, _, _, y, _, h in out)
    if used_w > sheet[0] or used_h > sheet[1]:
        raise SystemExit(f"sheet layout needs {used_w:g} x {used_h:g} mm, larger than the {sheet[0]:g} x {sheet[1]:g} sheet")
    return out


def plate_in_sheet(d: Design, x: float, y: float, p) -> tuple[float, float]:
    """Map a bin-frame point of the base plate into the rotated plate position (x, y) on the sheet."""
    return (x + (d.y1 - p[1]), y + (p[0] + d.base_w / 2))


def wall_in_sheet(d: Design, x: float, y: float, p) -> tuple[float, float]:
    """Map wall-local (s along length, z up) to a 90°-rotated 42 x 82 sheet position."""
    return (x + (d.panel_h - p[1]), y + p[0])


def page_nest(c: Canvas, d: Design, a, layout) -> None:
    sw, sh = a.sheet_mm
    page_frame(c, 5, "05 开料排版（1:2）",
               f"最小料板 {sw:g} × {sh:g} mm（零件外包围）· 插板式：底板 8 槽 + 侧板插舌 · 仅 CUT 层", a,
               scale_txt="1:2")
    k = 0.5
    ox, oy = 38.0, 56.0

    def P(x, y):
        return (ox + x * k, oy + y * k)

    c.rect(ox, oy, sw * k, sh * k, outline=GUIDE, width_mm=0.35, dash_mm=2.0)
    c.text(ox, oy + sh * k + 5, f"最小料板范围 {sw:g}×{sh:g}（参考，不切）", 2.4, GUIDE)
    for kind, idx, x, y, w, h in layout:
        if kind == "plate":
            c.rect(*P(x, y), w * k, h * k, fill="white", width_mm=0.3)
            for hole in d.base_slot_holes():
                c.poly([P(*plate_in_sheet(d, x, y, p)) for p in hole], fill="white", width_mm=0.18)
            c.text(*P(x + w / 2, y + h / 2 + 6), "底板", 3.0, INK, "mm")
            c.text(*P(x + w / 2, y + h / 2 - 3), f"{d.base_w:g}×{d.base_d:g} ×{d.t:g}", 2.3, NOTE, "mm")
        else:
            pts = [wall_in_sheet(d, x, y, p) for p in d.wall_outline()]
            c.poly([P(*p) for p in pts], fill=(206, 226, 246), width_mm=0.28)
            c.text(*P(x + w / 2, y + h / 2 + 9), "侧板", 2.8, INK, "mm")
            c.text(*P(x + w / 2, y + h / 2 - 6), f"{d.wall_len:g}×{d.panel_h:g}", 2.2, NOTE, "mm")
    c.dim(P(0, 0), P(sw, 0), -8, f"{sw:g}")
    c.dim(P(0, 0), P(0, sh), 8, f"{sh:g}")
    used_w = max(x + w for _, _, x, _, w, _ in layout)
    used_h = max(y + h for _, _, _, y, _, h in layout)
    c.dim(P(sw, 0), P(sw, used_h), -8, f"用高 {used_h:g}")

    # ---- table and notes
    c.text(212, 262, "排版与材料", 3.4)
    plate_n = sum(1 for p in layout if p[0] == "plate")
    wall_n = sum(1 for p in layout if p[0] == "wall")
    base_net = d.base_w * d.base_d - len(d.base_slots()) * d.tab_len * d.slot_w
    area = plate_n * base_net + wall_n * d.panel_area()
    grid_table(c, 212, 257, [40, 64, 34, 50],
               [("零件", "尺寸 / mm", "数量", "备注"),
                ("① 底板", f"{d.base_w:g}×{d.base_d:g}×{d.t:g}", str(plate_n), "含 4 个 L 槽孔"),
                ("② 侧板", f"{d.wall_len:g}×{d.panel_h:g}×{d.t:g}", str(wall_n), "两端插舌，4×4"),
                ("最小料板", f"{sw:g}×{sh:g}×{d.t:g}", "1 张", "透明 PMMA"),
                ("用料", f"{area:.0f} mm²", "—", f"占料板 {area / (sw * sh) * 100:.1f}%"),
                ("尺寸范围", f"{used_w:g}×{used_h:g}", "—", "含 2–4 mm 间距")],
               row_h=7.0, mm=2.5, align=[None, "c", "c", None])

    note_lines(c, 212, 190, [
        "切割文件：bins_acrylic_v1_cut.dxf（排版）与 bins_acrylic_v1_cut_parts.dxf（单件）：36 条闭合线。",
        "单位 mm；仅 CUT 层，全部切穿；无 ENGRAVE/浅雕层。",
        "槽宽默认 = 板厚；只调槽宽用 --slot-mm 1.9–2.1，再重出 DXF。",
        "激光缝宽约 0.1–0.2 mm，本排版按名义尺寸，不另放量；零件之间不要共边切割。",
        "商家自行排版时只要求零件尺寸/槽位不变；底板可整体旋转 90°。",
        "切完保留保护膜，按零件分捆并标注板厚；不要用记号笔在粘接面写字。",
        f"一套 4 桶约 {d.weight_g() * 4:.0f} g；最小料板 {sw:g}×{sh:g} 刚好一套；若用 300×400 标准板，四周余 3/12 mm。",
        "插槽与插舌均切穿；相邻零件留 2–4 mm，避免共边切割。",
        "侧板在排版中旋转 90°：图上右侧边的缺口/插舌对应侧板底边。",
        "侧板间 2 mm 为最小网，请勿共边、勿再缩小；四角两槽已合并为 1 个 L 孔。",
    ], 2.5, 5.6)

def page_cut_sheet(c: Canvas, d: Design, a, layout) -> None:
    """One-page A3 print: geometry and dimension numbers only, no titles/notes/legend."""
    sw, sh = a.sheet_mm
    c.rect(10, 10, 400, 277, width_mm=0.28)
    k = 0.5
    ox = (420 - sw * k) / 2
    oy = (297 - sh * k) / 2

    def P(x, y):
        return (ox + x * k, oy + y * k)

    c.rect(ox, oy, sw * k, sh * k, outline=GUIDE, width_mm=0.35, dash_mm=2.0)
    for kind, idx, x, y, w, h in layout:
        if kind == "plate":
            c.rect(*P(x, y), w * k, h * k, fill="white", width_mm=0.35)
            for hole in d.base_slot_holes():
                c.poly([P(*plate_in_sheet(d, x, y, p)) for p in hole], fill="white", width_mm=0.2)
            c.text(*P(x + w / 2, y + h / 2 + 7), "底板", 3.2, INK, "mm")
            c.text(*P(x + w / 2, y + h / 2 - 5), f"{d.base_w:g}×{d.base_d:g}×{d.t:g}", 2.5, NOTE, "mm")
        else:
            c.poly([P(*wall_in_sheet(d, x, y, p)) for p in d.wall_outline()], fill=(206, 226, 246), width_mm=0.3)
            c.text(*P(x + w / 2, y + h / 2 + 10), "侧板", 2.9, INK, "mm")
            c.text(*P(x + w / 2, y + h / 2 - 8), f"{d.wall_len:g}×{d.panel_h:g}×{d.t:g}", 2.2, NOTE, "mm")

    c.dim(P(0, 0), P(sw, 0), -9, f"{sw:g}")
    c.dim(P(0, sh), P(sw, sh), 9, f"{sw:g}")
    c.dim(P(0, 0), P(0, sh), 9, f"{sh:g}")
    c.dim(P(sw, 0), P(sw, sh), -9, f"{sh:g}")

def page_layout(c: Canvas, d: Design, poses, checks, arm, map_cfg, a) -> None:
    page_frame(c, 6, "06 附录 · 摆放与可达性核对（非加工）",
               "俯视，基座坐标系，单位 cm · 规划值，未实测；仅用于装配后摆放参考", a,
               material="—（非加工）", qty="—", scale_txt="示意（约 1:2，非加工）",
               unit_txt="cm（示意，非加工）")
    k, ox, oy = 0.52, 200.0, 100.0

    def M(x, y):
        return (ox + x * k, oy + y * k)

    dy = map_cfg["placement"]["base_from_map"]["dy"] * 1000
    c.rect(*M(-148.5, dy), 297 * k, 210 * k, outline=(130, 130, 130), fill=(250, 250, 240))
    c.text(ox - 148.5 * k, oy + (dy + 210) * k + 4, "A4 地图 map_a4_v2（规划用）", 2.7, NOTE)
    for corners in map_cfg["layout_m"].values():
        xs = [p[0] * 1000 for p in corners]
        ys = [p[1] * 1000 + dy for p in corners]
        c.rect(*M(min(xs), min(ys)), (max(xs) - min(xs)) * k, (max(ys) - min(ys)) * k,
               fill=(90, 90, 90), outline=(90, 90, 90))

    def arc(r, a0, a1, **kw):
        pts = [(r * math.cos(math.radians(t)), r * math.sin(math.radians(t)))
               for t in np.arange(a0, a1 + 0.1, 2)]
        for p, q in zip(pts, pts[1:]):
            c.line(M(*p), M(*q), **kw)

    pr = arm["keep_out"]["platform_radius_m"] * 1000
    arc(pr, 0, 360, fill=INK, width_mm=0.4)
    c.text(*M(0, 0), "底座平台", 2.6, INK, "mm")
    lo, hi = arm["soft_limits_deg"]["1"]
    for r in (130, 280):
        arc(r, lo, hi, fill=(0, 140, 0), width_mm=0.3)
    for r in (180, 240):
        arc(r, lo, hi, fill=(0, 140, 0), width_mm=0.15)
    for ang in (lo, hi):
        c.line(M(130 * math.cos(math.radians(ang)), 130 * math.sin(math.radians(ang))),
               M(300 * math.cos(math.radians(ang)), 300 * math.sin(math.radians(ang))), (0, 140, 0), 0.3, dash_mm=1.5)
    c.text(ox, 258, "可达环带 13–28 cm（细线：容器中心建议 18–24 cm）", 2.6, (0, 120, 0), "mm")

    for i, (pose, chk) in enumerate(zip(poses, checks)):
        ok = chk["solution"] is not None
        c.poly([M(*p) for p in core.to_base(pose, d.plate())], width_mm=0.35, fill=(255, 255, 255))
        for wall in d.walls():
            c.poly([M(*p) for p in core.to_base(pose, wall)],
                   fill=(150, 195, 240) if ok else (240, 150, 150), width_mm=0.15)
        for x, y in core.to_base(pose, list(d.marker_centres().values())):
            c.rect(M(x, y)[0] - d.side * k / 2, M(x, y)[1] - d.side * k / 2,
                   d.side * k, d.side * k, fill=INK, width_mm=0.05)
        lx, ly = core.to_base(pose, [(0, -18)])[0]
        c.text(*M(lx, ly), f"{bin_number(i)}", 3.6, (200, 0, 0), "mm")

    c.text(14, 56, "附录：仅规划核对，不用于加工；1–2、3–4 桶板边净距约 1.9 mm，落位误差须 ≤1 mm；实机按安全边界验证。", 2.4, WARN)
    rows = [("桶", "类别", "中心 r / J1", "中心 x, y", "松手指尖 z", "逆解（俯角 / 舵机 1–4）")]
    for i, (pose, chk) in enumerate(zip(poses, checks)):
        sol = chk["solution"]
        rows.append((f"{bin_number(i)}", CATEGORIES[bin_number(i) - 1][1], f"{math.hypot(pose[0], pose[1]) / 10:.1f} / {pose[2]:.0f}°",
                     f"{pose[0] / 10:.1f}, {pose[1] / 10:.1f}", f"{chk['tip_z_base_cm']:.1f}",
                     f"{sol['pitch_deg']}° / {sol['servos']}" if sol else "无解"))
    grid_table(c, 14, 48, [12, 24, 34, 34, 34, 72], rows, row_h=6.0, mm=2.3,
               align=[None, None, "c", "c", "c", None])
    c.text(14, 51.5, "注：指尖 z 为基座坐标系；物块底面 = 指尖 z − 1.2 cm，台面比基座系低 1.42 cm。", 2.2, NOTE)


NUMBERS: list[int] | None = None      # which bins of the design the pages are for (make_bins --numbers); default 1..bins


def bin_number(i: int) -> int:
    return NUMBERS[i] if NUMBERS else i + 1


def page_stickers(d: Design, bins: int) -> tuple[Image.Image, dict]:
    c = Canvas(A4, 600)

    def T(x, y, s, mm=3.0, fill=INK):
        c.text(x, y, s, mm, fill, "mm")

    T(105, 290, f"垃圾桶定位码贴纸 · A4 · 必须按 100%/实际尺寸打印 · {core.DICTIONARY} · 码边长 {d.side:g} mm", 3.4)
    T(105, 284.5, "哑光不干胶纸打印，沿灰线裁下。耳板条贴底板耳板（外边与两侧边对齐），方片落入桶底。", 3.0, (90, 90, 90))
    grey, layout = (150, 150, 150), {}
    strip_h, floor = d.flange - 1.0, d.inner - 1.0
    for i in range(bins):
        number = bin_number(i)
        key, name = CATEGORIES[number - 1]
        ids = d.marker_ids(number)
        col, row = i % 2, i // 2
        # flange strip: its top edge is the plate's outer edge, so marker centres sit flange/2 below it
        x0, y1 = 6 + col * (d.base_w + 6), 276 - row * (strip_h + 6)
        c.rect(x0, y1 - strip_h, d.base_w, strip_h, outline=grey, width_mm=0.1)
        cx, cy = x0 + d.base_w / 2, y1 - d.flange / 2
        c.marker(ids["flange_left"], cx - d.flange_x, cy, d.side)
        c.marker(ids["flange_right"], cx + d.flange_x, cy, d.side)
        T(cx, cy + 5.0, f"{number}", 7.0)
        T(cx, cy - 4.5, name, 4.2)
        T(cx - d.flange_x, cy - d.side / 2 - 2.6, f"码 {ids['flange_left']}", 2.1, grey)
        T(cx + d.flange_x, cy - d.side / 2 - 2.6, f"码 {ids['flange_right']}", 2.1, grey)
        T(cx, y1 - 1.6, "▲ 此边对齐底板外边 · 码顶朝 +Y", 1.8, grey)
        # floor square
        fx, fy = 12 + col * (floor + 14), 276 - ((bins + 1) // 2) * (strip_h + 6) - 8 - (row + 1) * (floor + 6)
        c.rect(fx, fy, floor, floor, outline=grey, width_mm=0.1)
        c.marker(ids["floor"], fx + floor / 2, fy + floor / 2, d.side)
        T(fx + floor / 2, fy + floor - 6, f"▲ 朝耳板 · {number} {name} · 码 {ids['floor']}", 2.6, grey)
        layout[str(number)] = {"category": key, "name": name, "ids": ids}
    c.line((20, 13), (120, 13), width_mm=0.3)
    for k in range(11):
        c.line((20 + 10 * k, 13), (20 + 10 * k, 15.5), width_mm=0.3)
    c.text(126, 13.5, "100 mm 校验尺：打印后先量它，不足 100 则必须重打", 2.8, INK, "lm")
    c.text(126, 9.2, "建议多打印 1 份备用；先贴耳板条，再放桶底片；贴前擦净表面", 2.6, (120, 120, 120), "lm")
    c.text(20, 9.8, "0", 2.2, INK, "mm")
    c.text(120, 9.8, "100", 2.2, INK, "mm")
    return c.image, layout


# --------------------------------------------------------------------------- DXF

def _dxf_polyline(layer: str, pts, closed: bool = True) -> list[str]:
    out = ["0", "POLYLINE", "8", layer, "66", "1", "10", "0.0", "20", "0.0", "30", "0.0",
           "70", "1" if closed else "0"]
    for x, y in pts:
        out += ["0", "VERTEX", "8", layer, "10", f"{x:.3f}", "20", f"{y:.3f}", "30", "0.000"]
    out += ["0", "SEQEND", "8", layer]
    return out


def _dxf_table(name: str, entries: list[list[str]], count: int) -> list[str]:
    out = ["0", "TABLE", "2", name, "70", str(count)]
    for rows in entries:
        out += rows
    out += ["0", "ENDTAB"]
    return out


def write_dxf(path, cut: list[list[tuple[float, float]]]) -> None:
    pts = [p for poly in cut for p in poly]
    xmin, xmax = min(p[0] for p in pts), max(p[0] for p in pts)
    ymin, ymax = min(p[1] for p in pts), max(p[1] for p in pts)
    body = ["0", "SECTION", "2", "HEADER",
            "9", "$ACADVER", "1", "AC1009",
            "9", "$INSBASE", "10", "0.0", "20", "0.0", "30", "0.0",
            "9", "$EXTMIN", "10", f"{xmin:.3f}", "20", f"{ymin:.3f}", "30", "0.0",
            "9", "$EXTMAX", "10", f"{xmax:.3f}", "20", f"{ymax:.3f}", "30", "0.0",
            "9", "$LIMMIN", "10", "0.0", "20", "0.0",
            "9", "$LIMMAX", "10", "300.0", "20", "400.0",
            "9", "$INSUNITS", "70", "4",
            "9", "$MEASUREMENT", "70", "1",
            "0", "ENDSEC",
            "0", "SECTION", "2", "TABLES"]
    body += _dxf_table("LTYPE", [
        ["0", "LTYPE", "2", "CONTINUOUS", "70", "0", "3", "Solid line", "72", "65", "73", "0", "40", "0.0"]], 1)
    body += _dxf_table("LAYER", [
        ["0", "LAYER", "2", "0", "70", "0", "62", "7", "6", "CONTINUOUS"],
        ["0", "LAYER", "2", "CUT", "70", "0", "62", "7", "6", "CONTINUOUS"]], 2)
    body += ["0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]
    for poly in cut:
        body += _dxf_polyline("CUT", poly)
    body += ["0", "ENDSEC", "0", "EOF"]
    path.write_text("\n".join(body) + "\n")


def dxf_sheet(d: Design, bins: int, layout) -> list:
    cut = []
    for kind, idx, x, y, w, h in layout:
        if kind == "plate":
            cut.append([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
            for hole in d.base_slot_holes():
                cut.append([plate_in_sheet(d, x, y, p) for p in hole])
        else:
            cut.append([wall_in_sheet(d, x, y, p) for p in d.wall_outline()])
    return cut


def dxf_parts(d: Design, bins: int) -> list:
    cut = []
    gap = 10.0
    for i in range(bins):
        x, y = i * (d.base_w + gap), 0.0
        cut.append([(x, y), (x + d.base_w, y), (x + d.base_w, y + d.base_d), (x, y + d.base_d)])
        for hole in d.base_slot_holes():
            cut.append([(x + px + d.base_w / 2, y + py - d.y0) for px, py in hole])
    per_row = 6
    y0 = d.base_d + gap
    for i in range(4 * bins):
        x = (i % per_row) * (d.wall_len + gap)
        y = y0 + (i // per_row) * (d.panel_h + gap)
        cut.append([(x + p[0], y + p[1]) for p in d.wall_outline()])
    return cut

# --------------------------------------------------------------------------- driver

def write_outputs(d: Design, poses, checks, arm, map_cfg, a, out, parts=None):
    layout = sheet_layout(d, a.bins, 2.0, tuple(a.sheet_mm))
    pages = [
        new_page(), new_page(), new_page(), new_page(), new_page(), new_page(),
    ]
    base_net = d.base_w * d.base_d - len(d.base_slots()) * d.tab_len * d.slot_w
    net_area = base_net * a.bins + 4 * a.bins * d.panel_area()
    page_cover(pages[0], d, a, {"sheet_util": 100 * net_area / (a.sheet_mm[0] * a.sheet_mm[1]),
                                "sheet": tuple(a.sheet_mm)})
    page_parts(pages[1], d, a)
    page_assembly(pages[2], d, a)
    page_sequence(pages[3], d, a)
    page_nest(pages[4], d, a, layout)
    # page 6: reuse the original layout check, then add border and title block on top
    page_layout(pages[5], d, poses, checks, arm, map_cfg, a)
    pages[0].image.save(out / f"{a.name}_drawings.pdf", save_all=True, append_images=[p.image for p in pages[1:]], resolution=DPI)

    sheet_page = new_page()
    page_cut_sheet(sheet_page, d, a, layout)
    sheet_page.image.save(out / f"{a.name}_cut_sheet.pdf", resolution=DPI)

    stickers, bins = page_stickers(d, a.bins)
    stickers.save(out / f"{a.name}_stickers_A4.pdf", resolution=600)

    cut_sheet = dxf_sheet(d, a.bins, layout)
    cut_parts = dxf_parts(d, a.bins)
    write_dxf(out / f"{a.name}_cut.dxf", cut_sheet)
    write_dxf(out / f"{a.name}_cut_parts.dxf", cut_parts)

    if a.preview_dir:
        a.preview_dir.mkdir(parents=True, exist_ok=True)
        for i, p in enumerate(pages):
            p.image.save(a.preview_dir / f"{a.name}_preview_{i + 1}.png")
        stickers.save(a.preview_dir / f"{a.name}_preview_7.png")
        sheet_page.image.save(a.preview_dir / f"{a.name}_preview_8.png")

    # ---- self checks
    report = []
    identity = 4 * d.wall_len * d.t - (d.outer ** 2 - d.inner ** 2)
    report.append(f"slots: {len(d.base_slots())} logical x {d.tab_len:g}x{d.slot_w:g} mm -> "
                  f"{len(d.base_slot_holes())} L holes (merged corners); "
                  f"4 panels net {d.panel_area():g} mm2 each; notch {d.notch_len:g} x {d.t:g}")
    report.append(f"geometry footprint: 4 wall strips {4 * d.wall_len * d.t:g} mm2 vs ring "
                  f"{d.outer ** 2 - d.inner ** 2:g} mm2 (diff {identity:+.3f})")
    report.append(f"material: {net_area:g} mm2 net/set, "
                  f"{net_area / (a.sheet_mm[0] * a.sheet_mm[1]) * 100:.1f}% of one sheet, "
                  f"{d.weight_g():.1f} g/bin, {d.weight_g() * a.bins:.0f} g/set")
    used_w = max(x + w for _, _, x, _, w, _ in layout)
    used_h = max(y + h for _, _, _, y, _, h in layout)
    report.append(f"sheet layout: {used_w:g} x {used_h:g} mm in {a.sheet_mm[0]:g} x {a.sheet_mm[1]:g}, "
                  f"{len(layout)} parts, 2-4 mm gaps")
    h = d.side / 2
    for key, (mx, my) in d.marker_centres().items():
        if key == "floor":
            continue
        report.append(f"marker {key}: edge margin {d.y1 - (my + h):g} mm to plate edge, "
                      f"{my - h - d.outer / 2:g} mm to wall, pitch {2 * d.flange_x:g} mm")
    report.append(f"marker floor: clearance to wall {d.inner / 2 - h:g} mm per side")

    # sticker self-check: every id found at roughly camera resolution and designed spacing
    grey = cv2.resize(np.array(stickers.convert("L")), None, fx=0.12, fy=0.12, interpolation=cv2.INTER_AREA)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, core.DICTIONARY))
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(grey)
    found = {int(i): c.reshape(4, 2).mean(axis=0) for i, c in zip(ids.ravel(), corners)} if ids is not None else {}
    want = sorted(v for e in bins.values() for v in e["ids"].values())
    mm_per_px = 25.4 / (600 * 0.12)
    pitch = [np.linalg.norm(found[e["ids"]["flange_left"]] - found[e["ids"]["flange_right"]]) * mm_per_px
             for e in bins.values() if e["ids"]["flange_left"] in found and e["ids"]["flange_right"] in found]
    report.append(f"sticker self-check: found {len(set(found) & set(want))}/{len(want)} ids, "
                  f"flange pitch {min(pitch):.2f}-{max(pitch):.2f} mm (design {2 * d.flange_x:g})")
    return bins, report
