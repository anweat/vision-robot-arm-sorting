"""Motion planning on the four-bar gripper model against an external collision map (independent of the
console path). Each stage can be called on its own; nothing here talks to the arm or the console.

    config.py         model facts (config/motion_planning.json) and the read-only facts it builds on
    tuning.py         scene errors, clearances, trims, search settings (config/planning_tuning.json)
    collision_map.py  the scene: a 2.5-D height grid with labelled objects (external input, JSON / npz)
    scene.py          oriented boxes (blocks, bins, plain boxes) to build or test maps with
    body.py           collision samples of links 1-4 and the gripper (+ held block), base frame
    gripper.py        the gripper's four-bar states, openings, the held block
    checks.py         World (map + tuning + target) and the pose / move / sweep checks
    solver.py         pad-centre TCP inverse kinematics and wrist-roll (J5) alignment
    grasp.py          grasp solving -> Grasp candidates
    release.py        release solving -> Release candidates (table, stack, into a bin: set or drop)
    route.py          joint paths: straight, else A* over a tip lattice, shortcut and re-checked
    motion.py         the result objects: Check, Step, MotionPlan
    planner.py        plan_transit / plan_pick / plan_place -> MotionPlan
    correct.py        at a set-down: where the held block is by the servo readback, and the nudge that puts it
                      on target (used by flow.stages.execute)
"""

from __future__ import annotations


def world(cmap=None, tuning=None, overrides: dict | None = None):
    """A checks.World for a collision map (path, dict or CollisionMap; None = empty table)."""
    from . import checks, tuning as tu
    from .collision_map import CollisionMap
    t = tuning if tuning is not None else (tu.load(overrides=overrides) if overrides else tu.default())
    m = CollisionMap.empty(table_z_m=t.table_z) if cmap is None else CollisionMap.load(cmap)
    t = t.with_map(m)
    if t.map.get("clear_platform", True):
        import numpy as np
        from . import checks as K
        import platform_model
        xs, ys = m.cell_centres()
        inside = platform_model.inside_distance_2d(xs, ys, K.PLATFORM) > -m.cell
        if np.any(inside & (m.height != 0)):
            m.clear_where(inside)
    return checks.World(m, t)
