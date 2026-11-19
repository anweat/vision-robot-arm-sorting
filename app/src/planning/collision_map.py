"""The scene as an external collision map: a 2.5-D height grid over the table, in the arm base frame.

Whoever sees the table (the scan + localization, a depth view, the simulator, a hand-made file) writes one
of these; the planner only reads it. Format (JSON, or the same keys in an .npz):

    {
      "format": "dofbot-collision-map/1",
      "frame": "base", "units": "m",
      "origin_m": [x0, y0],          # corner of cell [0][0]
      "cell_m": 0.0025,
      "shape": [ny, nx],             # rows along +y, columns along +x
      "height_m": [[...], ...],      # top of whatever stands in the cell, above the table; 0 = free,
                                     # null = not observed (see tuning map.unknown_top_m)
      "labels": [[...], ...],        # optional: object id per cell (0 = none)
      "objects": [                   # optional: what the labelled cells are
        {"id": 1, "name": "b1", "kind": "block", "class": "kitchen",
         "centre_m": [x, y], "size_m": [sx, sy, sz], "yaw_deg": 12.0, "sigma_xy_m": 0.004}
      ],
      "table_z_m": -0.0087,          # informative: the table the heights were measured from
      "source": {...}                # free-form provenance (stamp, session, views)
    }

Heights are relative to the table, so a map made against one table estimate still stacks on the planner's
own table (tuning trims.table_z_m). Grid quantisation is +-cell/2. The platform and the arm itself are
modelled separately; cells over the platform are dropped on load (tuning map.clear_platform).

Queries are vectorized (numpy): `clearance(points, margin)` is the vertical distance of each point above
the map grown by a ball of radius `margin` (< 0 = inside), with the grown maps cached per margin.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from .scene import Box

FORMAT = "dofbot-collision-map/1"


class CollisionMap:
    def __init__(self, origin_m: Sequence[float], cell_m: float, height_m: np.ndarray,
                 labels: np.ndarray | None = None, objects: Iterable[dict] = (), table_z_m: float | None = None,
                 source: dict | None = None):
        self.origin = (float(origin_m[0]), float(origin_m[1]))
        self.cell = float(cell_m)
        self.height = np.asarray(height_m, dtype=float)
        self.labels = (np.zeros(self.height.shape, dtype=np.int32) if labels is None
                       else np.asarray(labels, dtype=np.int32))
        if self.labels.shape != self.height.shape:
            raise ValueError(f"labels {self.labels.shape} != height {self.height.shape}")
        self.objects = [dict(o) for o in objects]
        self.table_z_m = table_z_m
        self.source = dict(source or {})
        self._grown: dict = {}

    # ------------------------------------------------------------------------------------------ io
    @property
    def shape(self) -> tuple[int, int]:
        return self.height.shape

    @classmethod
    def load(cls, src) -> "CollisionMap":
        """From a path (.json / .npz) or an already parsed dict."""
        if isinstance(src, CollisionMap):
            return src
        if isinstance(src, (str, Path)):
            p = Path(src)
            if p.suffix == ".npz":
                z = np.load(p, allow_pickle=False)
                meta = json.loads(str(z["meta"])) if "meta" in z.files else {}
                return cls(meta.get("origin_m", z["origin_m"] if "origin_m" in z.files else (0, 0)),
                           float(meta.get("cell_m", z["cell_m"] if "cell_m" in z.files else 0.0025)),
                           z["height_m"], z["labels"] if "labels" in z.files else None,
                           meta.get("objects", ()), meta.get("table_z_m"), meta.get("source"))
            src = json.loads(p.read_text(encoding="utf-8"))
        d = src
        if d.get("format", FORMAT) != FORMAT:
            raise ValueError(f"unknown collision map format {d.get('format')!r}")
        if d.get("frame", "base") != "base":
            raise ValueError("collision maps are in the arm base frame")
        h = np.array([[np.nan if v is None else v for v in row] for row in d["height_m"]], dtype=float)
        if "shape" in d and tuple(d["shape"]) != h.shape:
            raise ValueError(f"shape {d['shape']} != height_m {h.shape}")
        return cls(d["origin_m"], d["cell_m"], h, d.get("labels"), d.get("objects", ()), d.get("table_z_m"),
                   d.get("source"))

    def to_dict(self) -> dict:
        h = [[None if math.isnan(v) else round(float(v), 4) for v in row] for row in self.height]
        return dict(format=FORMAT, frame="base", units="m", origin_m=list(self.origin), cell_m=self.cell,
                    shape=list(self.shape), height_m=h, labels=self.labels.tolist(), objects=self.objects,
                    table_z_m=self.table_z_m, source=self.source)

    def save(self, path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        if p.suffix == ".npz":
            meta = dict(origin_m=list(self.origin), cell_m=self.cell, objects=self.objects,
                        table_z_m=self.table_z_m, source=self.source, format=FORMAT)
            np.savez_compressed(p, height_m=self.height, labels=self.labels, meta=json.dumps(meta))
        else:
            p.write_text(json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        return p

    # --------------------------------------------------------------------------------- construction
    @classmethod
    def empty(cls, bounds_m: Sequence[float] = (-0.30, -0.12, 0.30, 0.34), cell_m: float = 0.0025,
              table_z_m: float | None = None) -> "CollisionMap":
        x0, y0, x1, y1 = bounds_m
        nx, ny = int(math.ceil((x1 - x0) / cell_m)), int(math.ceil((y1 - y0) / cell_m))
        return cls((x0, y0), cell_m, np.zeros((ny, nx)), None, (), table_z_m)

    @classmethod
    def from_boxes(cls, boxes: Iterable[Box], table_z: float, cell_m: float = 0.0025,
                   bounds_m: Sequence[float] = (-0.30, -0.12, 0.30, 0.34), sigma_xy_m: float | None = None,
                   source: dict | None = None) -> "CollisionMap":
        """Rasterize upright boxes (scene.Box, base frame). A cell is taken by a box when its centre is
        inside the box grown by half a cell; overlapping boxes keep the taller (and its label)."""
        m = cls.empty(bounds_m, cell_m, table_z)
        xs, ys = m.cell_centres()
        for i, b in enumerate(boxes, start=1):
            m.add_box(b, i, xs, ys, sigma_xy_m)
        m.source = dict(source or {"made_by": "CollisionMap.from_boxes"})
        return m

    def cell_centres(self) -> tuple[np.ndarray, np.ndarray]:
        ny, nx = self.shape
        xs = self.origin[0] + (np.arange(nx) + 0.5) * self.cell
        ys = self.origin[1] + (np.arange(ny) + 0.5) * self.cell
        return np.meshgrid(xs, ys)

    def add_box(self, b: Box, obj_id: int | None = None, xs=None, ys=None, sigma_xy_m: float | None = None,
                cls_name: str | None = None) -> int:
        if xs is None:
            xs, ys = self.cell_centres()
        obj_id = obj_id or (max([o["id"] for o in self.objects], default=0) + 1)
        c, s = math.cos(math.radians(b.yaw_deg)), math.sin(math.radians(b.yaw_deg))
        dx, dy = xs - b.centre[0], ys - b.centre[1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        g = self.cell / 2
        inside = (np.abs(lx) <= b.half[0] + g) & (np.abs(ly) <= b.half[1] + g)
        top = b.top - (self.table_z_m if self.table_z_m is not None else 0.0)
        take = inside & ~(self.height > top)
        self.height[take] = top
        self.labels[take] = obj_id
        o = dict(id=obj_id, name=b.name, kind=b.kind, centre_m=[round(b.centre[0], 5), round(b.centre[1], 5)],
                 size_m=[round(2 * h, 5) for h in b.half], yaw_deg=round(b.yaw_deg, 3))
        if sigma_xy_m is not None:
            o["sigma_xy_m"] = sigma_xy_m
        if cls_name:
            o["class"] = cls_name
        self.objects.append(o)
        self._grown.clear()
        return obj_id

    def add_object(self, boxes: Iterable[Box], meta: dict) -> int:
        """Several boxes as one object (a bin: walls + plate); `meta` is its record (name, kind, class,
        centre_m, size_m, yaw_deg, sigma_xy_m, ...). Returns its id."""
        obj_id = int(meta.get("id") or (max([o["id"] for o in self.objects], default=0) + 1))
        xs, ys = self.cell_centres()
        for b in boxes:
            self.add_box(b, obj_id, xs, ys)
            self.objects.pop()                     # add_box records each box; keep the one record below
        self.objects.append(dict(meta, id=obj_id))
        return obj_id

    def clear_where(self, mask: np.ndarray) -> None:
        self.height[mask] = 0.0
        self.labels[mask] = 0
        self._grown.clear()

    # ------------------------------------------------------------------------------------- objects
    def object(self, key) -> dict:
        for o in self.objects:
            if o["id"] == key or o.get("name") == key:
                return o
        raise KeyError(f"no object {key!r} in the map (have {[o.get('name', o['id']) for o in self.objects]})")

    def object_box(self, key, table_z: float) -> Box:
        """The object as a Box standing on the planner's table (z from its height)."""
        o = self.object(key)
        sx, sy, sz = o["size_m"]
        return Box(o.get("kind", "block"), o.get("name", str(o["id"])),
                   (o["centre_m"][0], o["centre_m"][1], table_z + sz / 2), (sx / 2, sy / 2, sz / 2),
                   float(o.get("yaw_deg", 0.0)))

    def object_sigma(self, default: float = 0.0) -> float:
        return max([float(o.get("sigma_xy_m", 0.0)) for o in self.objects] + [default])

    def label_name(self, label: int) -> str:
        if label <= 0:
            return "map"
        for o in self.objects:
            if o["id"] == label:
                return o.get("name", str(label))
        return f"cell label {label}"

    # ------------------------------------------------------------------------------------- queries
    def _base_heights(self, exclude: frozenset, unknown_top: float | None) -> np.ndarray:
        h = self.height.copy()
        if exclude:
            h[np.isin(self.labels, list(exclude))] = 0.0
        unk = np.isnan(h)
        h[unk] = -np.inf if unknown_top is None else unknown_top
        h[h <= 0.0] = -np.inf               # free cells: the table is checked on its own
        return h

    def grown(self, margin: float, dz: float = 0.0, exclude: Iterable[int] = (),
              unknown_top: float | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(heights above the table of the map grown by a ball of radius `margin` and raised by `dz`, label
        of the cell that sets each), cached. Cell-to-cell distances are centre to centre."""
        key = (round(margin, 4), round(dz, 4), frozenset(exclude), unknown_top)
        if key in self._grown:
            return self._grown[key]
        h = self._base_heights(key[2], unknown_top)
        lab = self.labels
        out = np.full(h.shape, -np.inf)
        out_lab = np.zeros(h.shape, dtype=np.int32)
        k = int(math.floor(margin / self.cell))
        ny, nx = h.shape
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                d = math.hypot(di, dj) * self.cell
                if d > margin + 1e-12:
                    continue
                cap = math.sqrt(max(margin * margin - d * d, 0.0))
                # out[i, j] <- h[i - di, j - dj] + cap
                src = h[max(0, -di):ny - max(0, di), max(0, -dj):nx - max(0, dj)] + cap
                srcl = lab[max(0, -di):ny - max(0, di), max(0, -dj):nx - max(0, dj)]
                sl = (slice(max(0, di), ny - max(0, -di)), slice(max(0, dj), nx - max(0, -dj)))
                better = src > out[sl]
                out[sl] = np.where(better, src, out[sl])
                out_lab[sl] = np.where(better, srcl, out_lab[sl])
        out = np.where(np.isfinite(out), out + dz, out)
        self._grown[key] = (out, out_lab)
        return out, out_lab

    def cells_of(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        j = np.floor((pts[:, 0] - self.origin[0]) / self.cell).astype(np.int64)
        i = np.floor((pts[:, 1] - self.origin[1]) / self.cell).astype(np.int64)
        ny, nx = self.shape
        inside = (i >= 0) & (i < ny) & (j >= 0) & (j < nx)
        return np.clip(i, 0, ny - 1), np.clip(j, 0, nx - 1), inside

    def clearance(self, pts: np.ndarray, table_z: float, margin: float, dz: float = 0.0,
                  exclude: Iterable[int] = (), unknown_top: float | None = None,
                  out_of_map_free: bool = True) -> tuple[float, str | None]:
        """(min vertical clearance of the points above the grown map, name of what sets it).
        +inf when nothing is under any point."""
        if pts.size == 0:
            return math.inf, None
        g, lab = self.grown(margin, dz, exclude, unknown_top)
        i, j, inside = self.cells_of(pts)
        top = g[i, j]
        if out_of_map_free:
            top = np.where(inside, top, -np.inf)
        else:
            top = np.where(inside, top, np.inf)
        c = pts[:, 2] - (table_z + top)
        k = int(np.argmin(c))
        if not np.isfinite(c[k]):
            return (math.inf, None) if c[k] > 0 else (-math.inf, "outside the map")
        return float(c[k]), self.label_name(int(lab[i[k], j[k]])) if inside[k] else "outside the map"

    def tallest(self, table_z: float, exclude: Iterable[int] = ()) -> float:
        h = self._base_heights(frozenset(exclude), None)
        return table_z + (float(h[np.isfinite(h)].max()) if np.isfinite(h).any() else 0.0)
