---
name: surveying-puzzle-pieces
description: Find and name every loose piece of a lab puzzle on the table from one look straight down with the
  wrist camera - SAM 3.1 masks the pieces and scan to scene names and poses them against the CAD. Use when a graph
  must know where every piece of a known puzzle lies before it plans picks.
compatibility: requires gap>=0.1
metadata: {category: perception, tags: [perception, sam3.1, twinsmith]}
gap:
  allowed_tools: [workcell.survey]
  exit_conditions:
    located: Every piece of the puzzle was found and posed.
    failed: A piece is missing from the scan, or the camera or SAM 3.1 failed.
  produces_outputs: {}
  required_inputs: {}
---

# surveying-puzzle-pieces

SAM 3.1 is the segmenter and the CAD match is the authority: `workcell.survey` returns the pieces it named, and the
subgraph fails when one is missing. A graph that wants a second opinion asks a vision-language model afterwards
(`vlm.query` on `workcell.picture`), as the puzzle build's graph does; that answer is recorded and never overrules
the scan.

## When to use

- At the start of an assembly, with the pieces lying loose and the gripper clear of them.
- Not after a placement: use `workcell.rescan`, which judges one piece against its slot.

## Recommended subgraph state flow

`START -> survey (workcell.survey) -> located -> END`, with `on_error: failed`.
