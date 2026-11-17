---
name: assembling-puzzle-pieces
description: Pick one loose puzzle piece and insert it into its slot in the base along the slot axis - simulate the
  move ahead first, grasp at a pose defined on the CAD, verify the reach and the grip, lift, insert slowly, let go
  and look again. Use when a piece's slot is free and every piece it rests on is already in the base.
compatibility: requires gap>=0.1
metadata: {category: manipulation, tags: [manipulation, insertion, assembly, twinsmith]}
gap:
  requires: {connector: [robot.describe_gripper]}
  allowed_tools: [workcell.lookahead, workcell.approach, workcell.close, workcell.lift, workcell.insert,
                  workcell.rescan, robot.get_ee_pose, robot.get_gripper, robot.describe_gripper]
  exit_conditions:
    placed: The rescan saw the piece seated in its slot.
    retry: The rescan saw the piece loose and lying flat again; the same subgraph may run once more.
    changed: The rescan found another piece, or the base, elsewhere than the estimate had it; the table is scanned again before the build goes on.
    aborted: The lookahead on Newton said the insertion would not seat the piece or would move others.
    missed: The hand stopped too far from the grasp pose to have the piece between the pads.
    empty: The jaws closed on nothing; the table is scanned again and the piece tried from where it now lies.
    failed: A motion was refused or the piece cannot be picked again.
  produces_outputs: {}
  required_inputs: {}
---

# assembling-puzzle-pieces

One piece, one slot. The build order comes from taking the CAD apart, so the piece's slot axis is free and what it
rests on is in. The grasp is defined on the piece's CAD and chosen so the gripper fits both where the piece lies
and where it goes.

Two stock checks from open-robot-skills' `verifying-grasps` sit between the motions, with their scripts copied into
the graph: `verify_reach` after the approach (is the hand where it was sent, within the piece's own half width?)
and `verify_grip` after the close (did the jaws stop on something?). Both read the robot only.

## When to use

- For each piece of the build order, in order.
- Not for the base: use `placing-puzzle-base`.

## Recommended subgraph state flow

`START -> lookahead (workcell.lookahead)`; on `verdict: go` -> `approach (workcell.approach) -> ee
(robot.get_ee_pose) -> reach (verify_reach script)`; on `route: reached` -> `close (workcell.close) -> grip
(verify_grip script)`; on `route: held` or `unknown` -> `lift (workcell.lift) -> insert (workcell.insert) -> look
(workcell.rescan)`; `result: placed` -> `placed`, `retry` -> `retry`, `changed` -> `changed`. `on_error: failed`.
