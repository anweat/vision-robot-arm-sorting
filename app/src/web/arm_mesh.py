"""整臂网格与每段姿态（T12）：给实机三维视图画机械臂网格用。

网格：复用 `dev/sim/assets/web_meshes.json`（`dev/sim/web_meshes.py` 从课程 STL 抽稀，原始 STL 有版权不入库），
每个 part 带 `frame`（用哪个 4x4）、`v`（顶点，单位 0.1 mm）、`f`（三角面顶点索引）。

姿态：复用 `planning.body` 的 URDF 链（link1-4）与 `gripper_geometry` 的四连杆（link5 与夹爪各件），
和仿真页 `dev/sim/world.py:mesh_frames` **同一套实现**，保证网页画的臂与规划器/仿真一致，不另算一份。

依赖：numpy 与 `planning`/`gripper_geometry` 只在真正用到时导入；没有时抛 `ArmMeshUnavailable`，
服务按 501 返回、页面退回骨架——不因为一块可视化让服务起不来。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # app/
REPO = ROOT.parent
MESH_PATH = REPO / "dev" / "sim" / "assets" / "web_meshes.json"

_mesh_cache: dict | None = None
_deps_cache: tuple | None = None


class ArmMeshUnavailable(RuntimeError):
    """没有网格文件或缺 numpy/planning：调用方按 501 处理并退回骨架。"""


def mesh() -> dict:
    """课程 STL 抽稀后的网格（原样），第一次读入后缓存。"""
    global _mesh_cache
    if _mesh_cache is None:
        try:
            _mesh_cache = json.loads(MESH_PATH.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ArmMeshUnavailable(f"没有网格文件 {MESH_PATH}：{exc}") from exc
    return _mesh_cache


def _deps():
    """延迟导入 numpy 与运动学模块；失败给出可读原因。"""
    global _deps_cache
    if _deps_cache is None:
        try:
            import sys
            src = ROOT / "src"
            for p in (src, src / "spatial"):
                if str(p) not in sys.path:
                    sys.path.insert(0, str(p))
            import numpy as np
            from planning import body as body_mod
            import gripper_geometry as gg
        except Exception as exc:  # noqa: BLE001 - 统一转成可读的不可用原因
            raise ArmMeshUnavailable(f"三维网格需要 numpy 与 planning 模块：{exc}") from exc
        _deps_cache = (np, body_mod, gg)
    return _deps_cache


def _cm(m) -> list[list[float]]:
    """4x4 转嵌套 list，平移分量米→厘米（与 chain_cm、物块坐标一致），旋转不变。"""
    return [[round(float(m[r][c]) * (100.0 if (r < 3 and c == 3) else 1.0), 6) for c in range(4)]
            for r in range(4)]


def frames(joints_deg) -> dict:
    """按六个舵机角度算每个 mesh part 的 4x4 姿态（厘米），键与 `mesh()['parts'][*]['frame']` 对应。

    角度语义与 `planning.body` 一致（servo - 90 关于 z）；第 5、6 轴不进 link1-4 的链，
    第 6 轴经 `gripper_geometry.delta_for_j6` 决定夹爪开合。
    """
    np, body_mod, gg = _deps()
    q = [float(x) for x in joints_deg][:6]
    while len(q) < 6:
        q.append(90.0)
    q5, j6 = q[:5], q[5]
    delta = gg.delta_for_j6(j6)

    out: dict[str, list[list[float]]] = {"base_link": _cm(np.eye(4))}
    for i, f in enumerate(body_mod.link_frames(q[:4])):
        out[f"link{i + 1}"] = _cm(f)
    link5 = np.array(body_mod.link5(q5), dtype=float)
    out["link5"] = _cm(link5)

    for name, side in gg.load()["sides"].items():
        move = np.eye(4)
        move[0, 3], move[2, 3] = gg._side_dx(side, delta), gg._side_dz(side, delta)
        out[f"gripper_finger_{name}"] = _cm(link5 @ move)
        a = math.radians(side["sign"] * delta)
        c, s = math.cos(a), math.sin(a)
        for piece, pivot in (("upper", side["g_upper_m"]), ("lower", side["g_lower_m"])):
            turn = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)
            at, back = np.eye(4), np.eye(4)
            at[0, 3], at[2, 3] = pivot
            back[0, 3], back[2, 3] = -pivot[0], -pivot[1]
            out[f"gripper_{piece}_{name}"] = _cm(link5 @ at @ turn @ back)
    return out
