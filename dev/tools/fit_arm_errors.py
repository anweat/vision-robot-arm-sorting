"""Where the ~1 cm comes from: fit error models of the arm + camera to the recorded map views, cross-validated.

Input is the corner table of dev/tools/validate_camera.py (data/reports/camera_validation_*.json, placement
"touch" rows: every detected ArUco corner with the servo readback of its image). Nothing is re-detected.

Every model re-fits a map placement (dx, dy, yaw) per session group, since the pipeline registers the map on
every scan anyway (and the map was moved between sessions). On top of that, nested:

    M0  nominal            FK + hand_eye.json; one table z for all
    M1  + joint zeros      constant offsets on J1-J4 (what a servo zero calibration would correct)
    M2  + gravity sag      J2-J4 offsets that grow with the horizontal lever of the arm beyond the joint
                           (backlash taken up by gravity plus compliance: depends on the pose, deterministic)
    M3  + hand-eye         a small rotation / translation of the camera on the wrist

Parameters are bounded to physical ranges (BOUND: servo zeros / sag within 5 deg, the camera within 3 deg
and 1.5 cm of hand_eye.json); unbounded, J2-J4 zeros and sag trade off into 20-45 deg values that fit these
high views and mean nothing elsewhere. Scores are on held-out data: leave-one-date-out, where the held-out date only gets its per-group placement
fitted (what a scan's own registration gives). What remains under the best model is per-view error that no
fixed model of the arm explains (backlash state, readback quantisation, the image itself).

    uv run --with numpy --with scipy --with 'opencv-contrib-python<5' python dev/tools/fit_arm_errors.py \
        [--report data/reports/camera_validation_20261115.json] [--out data/reports/arm_error_fit_<date>.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dev" / "spatial"))
sys.path.insert(0, str(ROOT / "app" / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402
from map_registration import TableMap, load_intrinsics  # noqa: E402

LEVER_SCALE_M = 0.10
BOUND = dict(zero_deg=5.0, sag_deg=5.0, he_rot_deg=3.0, he_t_m=0.015, z_m=0.03, place_m=0.06, place_yaw_deg=10.0)
PRIOR = dict(zero_deg=3.0, sag_deg=3.0, he_rot_deg=2.0, he_t_m=0.01, place_m=0.03, place_yaw_deg=5.0, z_m=0.01)
MODELS = ("M0", "M1", "M2", "M3")


def _rodrigues(w: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(w, float).reshape(3, 1))[0]


def mount_poses(q: np.ndarray) -> np.ndarray:
    """camera_mount_pose (servo-5 frame at J5 = 90) for N x 4 servo angles, vectorized: N x 4 x 4."""
    s1, s2, s3, s4 = (q[:, i] for i in range(4))
    p2 = np.radians(90 - s2)
    p3 = p2 + np.radians(90 - s3)
    p4 = p3 + np.radians(90 - s4)
    r = kin.UPPER_ARM * np.sin(p2) + kin.FOREARM * np.sin(p3) + kin.WRIST_TO_SERVO5 * np.sin(p4)
    z = kin.BASE_TO_SHOULDER + kin.UPPER_ARM * np.cos(p2) + kin.FOREARM * np.cos(p3) + kin.WRIST_TO_SERVO5 * np.cos(p4)
    yaw = np.radians(s1 - 90)
    cy, sy = np.cos(yaw), np.sin(yaw)
    n = len(q)
    T = np.zeros((n, 4, 4))
    x = np.stack([cy, sy, np.zeros(n)], 1)
    yv = np.stack([-sy * np.cos(p4), cy * np.cos(p4), -np.sin(p4)], 1)
    zv = np.stack([-sy * np.sin(p4), cy * np.sin(p4), np.cos(p4)], 1)
    T[:, :3, 0], T[:, :3, 1], T[:, :3, 2] = x, yv, zv
    T[:, :3, 3] = np.stack([-r * sy, r * cy, z], 1)
    T[:, 3, 3] = 1
    return T


def levers(q: np.ndarray) -> np.ndarray:
    """Horizontal distance (m) from joints 2, 3, 4 to the servo-5 centre (where most of the load hangs)."""
    s2, s3, s4 = q[:, 1], q[:, 2], q[:, 3]
    p2 = np.radians(90 - s2)
    p3 = p2 + np.radians(90 - s3)
    p4 = p3 + np.radians(90 - s4)
    a, b, c = kin.UPPER_ARM * np.sin(p2), kin.FOREARM * np.sin(p3), kin.WRIST_TO_SERVO5 * np.sin(p4)
    return np.stack([a + b + c, b + c, c], 1)


class Data:
    def __init__(self, report: Path, table: TableMap):
        d = json.loads(report.read_text(encoding="utf-8"))
        rows = [c for c in d["corners"] if c["placement"] == "touch"]
        self.rows = rows
        self.q = np.array([r["readback"][:4] for r in rows], float)
        self.det = np.array([r["det"] for r in rows], float)
        self.P = np.array([np.r_[table.layout[r["marker"]][r["corner"]], 0.0] for r in rows], float)
        groups = sorted({r["group"] for r in rows})
        self.groups = groups
        self.g = np.array([groups.index(r["group"]) for r in rows])
        self.date = np.array([r["date"] for r in rows])
        self.view = np.array([f'{r["group"]}/{r["image"]}' for r in rows])
        self.family = np.array([r["family"] for r in rows])
        self.offset = table.offset.copy()
        self.lever = levers(self.q)


class Model:
    """Parameter vector: [globals | placements of the fitted groups]."""

    def __init__(self, name: str, data: Data, K, dist, cam2grip):
        self.name, self.d, self.K, self.dist, self.c2g = name, data, K, dist, cam2grip
        self.n_zero = 4 if name in ("M1", "M2", "M3") else 0
        self.n_sag = 3 if name in ("M2", "M3") else 0
        self.n_he = 6 if name == "M3" else 0
        self.n_glob = 1 + self.n_zero + self.n_sag + self.n_he      # + table z offset

    def split(self, x):
        i = 0
        dz = x[i]; i += 1
        zero = x[i:i + self.n_zero]; i += self.n_zero
        sag = x[i:i + self.n_sag]; i += self.n_sag
        he = x[i:i + self.n_he]; i += self.n_he
        return dz, zero, sag, he, x[i:]

    def servo(self, idx, zero, sag):
        q = self.d.q[idx].copy()
        if len(zero):
            q[:, :4] += zero
        if len(sag):
            q[:, 1:4] += sag * self.d.lever[idx] / LEVER_SCALE_M
        return q

    def cam_poses(self, idx, zero, sag, he):
        T = mount_poses(self.servo(idx, zero, sag)) @ self.c2g
        if len(he):
            C = np.eye(4)
            C[:3, :3] = _rodrigues(np.radians(he[:3]))
            C[:3, 3] = he[3:]
            T = T @ C
        return T

    def world_points(self, idx, dz, place, gmap):
        P = self.d.P[idx]
        out = np.empty_like(P)
        for gi, k in gmap.items():
            m = self.d.g[idx] == gi
            dx, dy, yaw = place[3 * k:3 * k + 3]
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            pts = P[m]
            out[m, 0] = c * pts[:, 0] - s * pts[:, 1] + self.d.offset[0] + dx
            out[m, 1] = s * pts[:, 0] + c * pts[:, 1] + self.d.offset[1] + dy
            out[m, 2] = self.d.offset[2] + dz
        return out

    def project(self, idx, x, gmap):
        dz, zero, sag, he, place = self.split(x)
        T = self.cam_poses(idx, zero, sag, he)
        W = self.world_points(idx, dz, place, gmap)
        Rinv = np.transpose(T[:, :3, :3], (0, 2, 1))
        pc = np.einsum("nij,nj->ni", Rinv, W - T[:, :3, 3])
        xn, yn = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
        r2 = xn * xn + yn * yn
        f = 1 + self.dist[0] * r2 + self.dist[1] * r2 * r2
        u = self.K[0, 0] * xn * f + self.K[0, 2]
        v = self.K[1, 1] * yn * f + self.K[1, 2]
        return np.stack([u, v], 1)

    def priors(self, x, n_place):
        dz, zero, sag, he, place = self.split(x)
        out = [np.atleast_1d(dz / PRIOR["z_m"])]
        if len(zero):
            out.append(zero / PRIOR["zero_deg"])
        if len(sag):
            out.append(sag / PRIOR["sag_deg"])
        if len(he):
            out.append(he[:3] / PRIOR["he_rot_deg"])
            out.append(he[3:] / PRIOR["he_t_m"])
        p = place.reshape(-1, 3)
        out.append(p[:, 0] / PRIOR["place_m"])
        out.append(p[:, 1] / PRIOR["place_m"])
        out.append(p[:, 2] / PRIOR["place_yaw_deg"])
        return np.concatenate(out)

    def bounds(self, n_place):
        """Physical ranges: servo zeros and sag within a few degrees, the camera within its mount."""
        b = [BOUND["z_m"]] + [BOUND["zero_deg"]] * self.n_zero + [BOUND["sag_deg"]] * self.n_sag
        if self.n_he:
            b += [BOUND["he_rot_deg"]] * 3 + [BOUND["he_t_m"]] * 3
        b += [BOUND["place_m"], BOUND["place_m"], BOUND["place_yaw_deg"]] * n_place
        b = np.array(b)
        return np.stack([-b, b])

    def fit(self, idx, x0_glob=None, freeze_glob=False):
        groups = sorted(set(self.d.g[idx].tolist()))
        gmap = {g: k for k, g in enumerate(groups)}
        glob = np.zeros(self.n_glob) if x0_glob is None else np.asarray(x0_glob, float)
        place0 = np.zeros(3 * len(groups))
        det = self.d.det[idx]

        def res(xv):
            x = np.r_[glob, xv] if freeze_glob else xv
            e = (self.project(idx, x, gmap) - det).ravel()
            pr = self.priors(x, len(groups))
            if freeze_glob:
                pr = pr[self.n_glob:]
            return np.r_[e, pr]

        x0 = place0 if freeze_glob else np.r_[glob, place0]
        lim = self.bounds(len(groups))
        if freeze_glob:
            lim = lim[:, self.n_glob:]
        x0 = np.clip(x0, lim[0] + 1e-9, lim[1] - 1e-9)
        sol = least_squares(res, x0, bounds=(lim[0], lim[1]), loss="soft_l1", f_scale=8.0, x_scale="jac", max_nfev=200)
        x = np.r_[glob, sol.x] if freeze_glob else sol.x
        return x, gmap

    def table_error_cm(self, idx, x, gmap):
        """Back-projection of each detected corner onto the fitted table plane, distance to its layout point."""
        dz, zero, sag, he, place = self.split(x)
        T = self.cam_poses(idx, zero, sag, he)
        W = self.world_points(idx, dz, place, gmap)
        und = cv2.undistortPoints(self.d.det[idx].reshape(-1, 1, 2), self.K, self.dist).reshape(-1, 2)
        ray_c = np.c_[und, np.ones(len(und))]
        ray = np.einsum("nij,nj->ni", T[:, :3, :3], ray_c)
        o = T[:, :3, 3]
        s = (W[:, 2] - o[:, 2]) / ray[:, 2]
        hit = o + s[:, None] * ray
        return np.hypot(hit[:, 0] - W[:, 0], hit[:, 1] - W[:, 1]) * 100


def stats(v) -> dict:
    v = np.asarray(v, float)
    return dict(n=int(len(v)), median=round(float(np.median(v)), 2), p90=round(float(np.percentile(v, 90)), 2))


def describe(m: Model, x) -> dict:
    dz, zero, sag, he, place = m.split(x)
    out = dict(table_z_cm=round(float((m.d.offset[2] + dz) * 100), 2))
    if len(zero):
        out["joint_zero_deg"] = dict(zip(("J1", "J2", "J3", "J4"), np.round(zero, 2).tolist()))
    if len(sag):
        out["sag_deg_per_10cm_lever"] = dict(zip(("J2", "J3", "J4"), np.round(sag, 2).tolist()))
    if len(he):
        out["hand_eye_rot_deg"] = np.round(he[:3], 2).tolist()
        out["hand_eye_t_cm"] = np.round(he[3:] * 100, 2).tolist()
    p = place.reshape(-1, 3)
    out["placement_spread"] = dict(dx_cm=round(float(np.std(p[:, 0]) * 100), 2), dy_cm=round(float(np.std(p[:, 1]) * 100), 2),
                                   yaw_deg=round(float(np.std(p[:, 2])), 2))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report", type=Path, default=ROOT / "data" / "reports" / "camera_validation_20261115.json")
    ap.add_argument("--map", default="map_a4_v2")
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / f"arm_error_fit_{time.strftime('%Y%m%d')}.json")
    a = ap.parse_args(argv)
    table = TableMap(ROOT / "app" / "config" / f"{a.map}.json")
    data = Data(a.report, table)
    K, dist = load_intrinsics()
    c2g = np.array(json.loads((ROOT / "app" / "config" / "calibration" / "hand_eye.json").read_text())["cam2gripper"], float)
    allidx = np.arange(len(data.rows))
    dates = sorted(set(data.date.tolist()))
    out = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), report=str(a.report.relative_to(ROOT)), corners=len(allidx),
               views=len(set(data.view.tolist())), groups=len(data.groups), dates=dates, models={})
    print(f"{len(allidx)} corners, {out['views']} views, {len(data.groups)} groups, dates {dates}")
    for name in MODELS:
        m = Model(name, data, K, dist, c2g)
        x, gmap = m.fit(allidx)
        px = np.hypot(*(m.project(allidx, x, gmap) - data.det).T)
        cm = m.table_error_cm(allidx, x, gmap)
        cv_px, cv_cm, per_date = [], [], {}
        for dte in dates:
            test = allidx[data.date == dte]
            train = allidx[data.date != dte]
            if len(test) < 20 or len(set(data.date[train].tolist())) < 1:
                continue
            xt, _ = m.fit(train)
            xg = xt[:m.n_glob]
            xs, gm = m.fit(test, x0_glob=xg, freeze_glob=True)
            e = np.hypot(*(m.project(test, xs, gm) - data.det[test]).T)
            c = m.table_error_cm(test, xs, gm)
            cv_px.append(e), cv_cm.append(c)
            per_date[dte] = dict(px=stats(e), cm=stats(c))
        fam = {f: dict(px=stats(px[data.family == f]), cm=stats(cm[data.family == f]))
               for f in sorted(set(data.family.tolist())) if (data.family == f).sum() >= 20}
        out["models"][name] = dict(params=describe(m, x), fit=dict(px=stats(px), cm=stats(cm)), family=fam,
                                   held_out=dict(px=stats(np.concatenate(cv_px)), cm=stats(np.concatenate(cv_cm)), date=per_date))
        r = out["models"][name]
        print(f"{name}: fit px {r['fit']['px']['median']}/{r['fit']['px']['p90']} cm {r['fit']['cm']['median']}/{r['fit']['cm']['p90']}"
              f" | held-out px {r['held_out']['px']['median']}/{r['held_out']['px']['p90']} cm {r['held_out']['cm']['median']}/{r['held_out']['cm']['p90']}")
        print("    ", json.dumps(r["params"], ensure_ascii=False))
    # what is left: per-view mean shift vs within-view spread under the last model
    m = Model(MODELS[-1], data, K, dist, c2g)
    x, gmap = m.fit(allidx)
    err = m.project(allidx, x, gmap) - data.det
    by = defaultdict(list)
    for i, v in enumerate(data.view):
        by[v].append(i)
    shift = np.array([np.hypot(*err[ix].mean(0)) for ix in by.values()])
    within = np.concatenate([np.hypot(*(err[ix] - err[ix].mean(0)).T) for ix in by.values() if len(ix) >= 8])
    out["residual_split"] = dict(view_shift_px=stats(shift), within_view_px=stats(within),
                                 note="view_shift: one offset per image (pose error of that view); within_view: what a per-view shift leaves")
    print("residual under", MODELS[-1], json.dumps(out["residual_split"], ensure_ascii=False))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", a.out.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
