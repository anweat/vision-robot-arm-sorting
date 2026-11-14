"""Motion planning on the four-bar gripper model (independent of the console path).

    config.py   parameters (config/motion_planning.json) and the read-only facts it builds on
    scene.py    what stands on the table: blocks, bins, plain boxes (oriented boxes in the base frame)
    gripper.py  the gripper's states and collision samples (src/spatial/gripper_geometry.py)
    checks.py   joint limits, platform, table and obstacle margins of a pose or a joint-linear move
    solver.py   pad-centre TCP inverse kinematics and wrist-roll (J5) alignment
    motion.py   the result objects: Step, MotionPlan
    planner.py  plan_transit / plan_pick / plan_place -> MotionPlan

Nothing here talks to the arm or the console; a MotionPlan is a description for an executor to run.
"""
