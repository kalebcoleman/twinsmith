---
name: scanning-the-table
description: Scan the whole table before anything is planned - look with the wrist camera from above, and from the
  sides too when that leaves a piece in doubt, combine the views into one estimate of every piece (SAM 3.1 and
  the CAD match on the first view, the depth of every view checked against it), rebuild the simulated twin from
  that estimate and say what the build has to do next. Use
  when a graph starts on a table it has not seen, and again whenever a pick closed on nothing or a look found a
  piece or the base elsewhere than the last scan put it.
compatibility: requires gap>=0.1
metadata: {category: perception, tags: [perception, multi-view, sam3.1, twinsmith]}
gap:
  allowed_tools: [workcell.scan]
  exit_conditions:
    scanned: Every piece of the puzzle was found and posed, none of those still to be moved is in doubt, and the twin is rebuilt.
    failed: A piece is missing or in doubt, the camera, SAM 3.1 or the multi-view cycle failed, or the table was scanned too often.
  produces_outputs: {found: int, next: str}
  required_inputs: {}
---

# scanning-the-table

One tool, `workcell.scan`. It stops the wrist camera straight above the table (the view the matcher reads), hands
the view to the multi-view cycle as the folder a scan mover writes on the robot's side
(`scan_mover.multiview/1`), and takes the cycle's one estimate as where everything is. When the cycle leaves a
piece in doubt from above (two poses its depth cannot tell apart), the camera stops at the configured views from
the sides as well and the cycle is given all of them; the views from the sides add nothing to a table read
without doubt, and are not taken then.
From it the twin is rebuilt: the poses the lookahead simulates from, and a task file of the scene. The scan then
works out what is already done (the base in the middle, pieces seated in it) and returns `next`: the base's name,
the next piece of the build order, or `done`.

The registry's `scripts/next_step.py` is the router that follows the subgraph at the top level: it routes on `next`, so one scan
node serves the start of a build and every return to it. That is also what lets a table with another puzzle on it,
or one half built, be taken up where it stands.

## When to use

- As a graph's first step.
- After `assembling-puzzle-pieces` leaves by `empty`, `missed` or `changed`: what the graph believed about the
  table no longer holds.
- Not after an ordinary placement: `workcell.rescan` is one look and judges one piece against its slot.

## Recommended subgraph state flow

`START -> sweep (workcell.scan) -> scanned -> END`, with `on_error: failed` and outputs `found` and `next` from
`sweep`. At the top level: `scan -> next_step (router, scripts/next_step.py, input next) -> the base's subgraph,
a piece's subgraph, or the end`.
