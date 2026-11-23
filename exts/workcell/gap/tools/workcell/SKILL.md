---
name: workcell
description: The simulated UR5e work cell's own skills as tools - scan the table from several views with the wrist
  camera, SAM 3.1 and scan to scene, slide an assembly's base, simulate a pick and insertion ahead on Newton, approach,
  close, lift, insert and look again. Use when a graph assembles pieces of a pack on twinsmith's ur5e-workcell in Isaac Sim.
compatibility: requires gap>=0.1
metadata: {category: manipulation, tags: [manipulation, perception, simulation, twinsmith]}
gap:
  requires: {env: [WORKCELL_SKILLS_PORT]}
  tools:
    - workcell.scan: Sweep the wrist camera over the scan's views, combine them into one estimate, rebuild the twin, say what is next.
    - workcell.survey: Find every piece of the assembly on the table (wrist camera, SAM 3.1, scan to scene).
    - workcell.picture: One colour picture from the wrist camera, where it is now.
    - workcell.place_base: Grip the assembly's base by a feature its settings name and slide it to the middle of the table.
    - workcell.lookahead: Simulate a piece's pick and insertion ahead on Newton and say go or abort.
    - workcell.approach: Choose a grasp for a piece from its CAD, open the gripper and bring the pads round it.
    - workcell.close: Close the RG2 on the piece at the configured force.
    - workcell.lift: Lift the gripped piece straight up.
    - workcell.insert: Carry the held piece over its slot, check the base, lower it (the base checked on the way), let go and back away; or put the piece back.
    - workcell.rescan: Look again from the survey pose and say whether a piece is where it should be.
    - workcell.release: Open the gripper.
    - workcell.state: The robot's gripper pose and opening, and every piece's true pose with its slot's.
---

# workcell

The tools of this bundle do nothing themselves: each one asks a running build host to do it. A build host is a
project's own driver beside the simulator bridge: it holds the bridge, the assembly's CAD and settings, the SAM 3.1
tool and the scan, and serves one tool call at a time on `127.0.0.1:$WORKCELL_SKILLS_PORT` (a JSON line each way:
`{"tool", "inputs"}` in, `{"ok", "result"}` or `{"ok": false, "error"}` out). Nothing here can reach a real robot.

## When to use

- A graph that assembles pieces of a pack (such as `puzzles`) on the simulated UR5e work cell.
- Not for another robot or scene: the tools take piece names from the assembly the build host was started with.

## Notes

- A tool that cannot do what was asked raises, so its subgraph leaves through `on_error`.
- `workcell.lookahead` changes nothing: it saves the stage's physics state, tries the move and puts the state back.
  A host that corrects places from a table, or not at all, may simulate nothing and answer go at once.
- `workcell.insert` checks the base from one depth picture at the pre-place hover and while lowering. A base that moved takes its
  slots with it and the rest of the placement is laid out again and simulated ahead; `result` is then still `inserted`. When the
  check cannot vouch for the base, or other pieces are not where they were, the piece is put back where it was picked and
  `result` is `rescan`.
- `workcell.state` is ground truth from the simulator. Graphs use it for checkpoints, never for planning.
- The connector and runner for such a graph are `workcell_graph.py` in this registry.
