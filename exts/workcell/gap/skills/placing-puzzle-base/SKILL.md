---
name: placing-puzzle-base
description: Move a puzzle's base plate to the middle of the table by gripping its centre island and sliding it on
  the table top, then look again to confirm where it stopped. Use when the base lies off to the side and the
  pieces must be inserted into it within the arm's best reach.
compatibility: requires gap>=0.1
metadata: {category: manipulation, tags: [manipulation, pushing, twinsmith]}
gap:
  allowed_tools: [workcell.place_base, workcell.rescan]
  exit_conditions:
    placed: The base was seen within 10 mm and 5 degrees of the middle.
    retry: The base stopped short of that but within 50 mm; the same subgraph may run once more, from where it now is.
    failed: The way was not free, no grasp was in reach, or the base stopped somewhere else.
  produces_outputs: {}
  required_inputs: {}
---

# placing-puzzle-base

The base is a 161 mm plate whose only raised feature is a 7 mm island: gripped there and lifted, it tilts in the
fingers. So it is slid, not carried. The fingertips stand in the two bar slots, close on the island at the
gripper's force, and the arm moves in a straight line at table height, turning the base on the way.

## When to use

- After the survey, before any piece is inserted.
- Only when the straight path to the middle is free of pieces; `workcell.place_base` refuses otherwise.

## Recommended subgraph state flow

`START -> slide (workcell.place_base) -> look (workcell.rescan)`; `result: placed` -> `placed`, `retry` -> `retry`.
`on_error: failed`. The plate slips in the fingers on its way and stops short, by 3 to 20 mm in the simulator; the look
measures where it stopped, and `retry` routes back to this subgraph for one short slide from there.
