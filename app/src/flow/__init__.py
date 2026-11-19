"""The arm's task flow as separate, file-backed stages, each callable on its own (python or CLI):

    scan      the arm visits the scan poses, saves the views            moves the arm (console)
    locate    views -> objects in the base frame (blocks, bins)         offline
    map       objects (+ extra boxes) -> collision map                  offline
    plan      collision map + task -> pick / place / return MotionPlans offline (src/planning)
    execute   a plan -> console commands, step by step, with readbacks  moves the arm (console) / dry run
    verify    objects before vs after -> did the block go where planned offline

A session directory (data/flow/<stamp>/) holds what each stage wrote: scan/ (the views), located.json,
collision_map.json, task.json, plan.json, execution.jsonl, verify.json, and flow.jsonl (what ran when).
Any stage can instead be given its input files explicitly (another session's, an external collision map,
a hand-written task), so a stage can be re-run, replaced or tested alone.

The existing pipeline (src/pipeline/run.py, src/tasks/console_grasp.py) is used for scanning and
perception only; planning and execution do not go through it.
"""
