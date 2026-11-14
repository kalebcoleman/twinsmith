# BrickSim with these tools

[BrickSim](https://github.com/intelligent-control-lab/BrickSim) simulates LEGO bricks and how they connect, on Isaac Sim 5.1. These tools work next to it with no change to BrickSim: you start BrickSim as usual and add the bridge to its command. Everything here was run on Sim 5.1.0 with an RTX 5080, on 2026-11-06. Isaac Sim 5.1 has no Newton, so BrickSim runs on PhysX only; the target engine for newer builds is in [compatibility](COMPATIBILITY.md#target-engine).

## Run it with the bridge

BrickSim's own launcher accepts extra extensions. From a BrickSim checkout, using its own environment:

```bash
TWINSMITH_PORT=8766 OMNI_KIT_ACCEPT_EULA=YES .venv/bin/bricksim \
    --ext-folder <twinsmith>/exts --enable twinsmith.bridge --enable sim-tools demos/demo_assembly.py
```

Use `.venv/bin/bricksim`, not `uv run`, so that nothing in its environment is changed. Add `--enable workcell` only if you want the work cell's tools. The window opens with BrickSim's scene, the tools of the enabled toolsets answer, and `status` reports app version 5.1.0 (the 2026-11-06 run counted 69 tools before the toolsets existed).

## Start a demo from an agent

`demos/demo_assembly.py` builds its scene and then waits, paused, for Play. A plain `sim__timeline action=play` refuses with `TIMELINE_ALREADY_ACTIVE`, and its suggested stop would end the demo. Continue it with:

```
sim__stage action=info
sim__timeline action=play resume=true
```

The arm then builds the structure by itself: all four bricks onto the baseplate, in about four minutes. Scene edits stay refused while it runs, because physics writes where the demo set it, not to the session layer.

## Toolsets to enable next to BrickSim

Turn on `sim` (stage, prims, body state, contacts, raycast, joints, arm, cameras, `run_python`, and the Isaac Lab tools used to train a project task, see below) and `computer` (screenshots and recordings; explicit-only, so it is selected at startup): `TWINSMITH_TOOLSETS=default,+sim,+computer`. `sim` also holds the work cell's tools; BrickSim does not need them. Choose them with [toolsets](TOOLSETS.md#choosing-toolsets).

## Which tools work on bricks

Bricks are under `/World/managed/Parts/Part_N`, and the arm is `/World/Robot`.

| Tool | Result |
|---|---|
| `sim__stage action=tree root=/World/managed depth=3`, `sim__prim` actions `find`, `get` and `bounds` | Work |
| `sim__physics action=body_state` on a brick | Works, while the timeline runs |
| `sim__physics action=contacts paths=[...]` | Works: names the brick's colliders and what they touch |
| `sim__physics action=raycast` | Works |
| `sim__joints action=state`, `sim__arm action=fk` on `/World/Robot` | Work; the FK self-check agreed with physics to 2e-7 m |
| `sim__camera action=list`, `screenshot`, `screen_recording` | Work |
| `sim__viewport path=/World/LegoWorkspace` | Refused: that mesh is not a framing target. Give `eye` and `target` |
| `sim__arm action=ik move=true`, `sim__joints action=command` | Do not use while a demo runs: its own controller is commanding the arm |

These tools see rigid bodies and physics contacts. `sim__physics action=contacts` does not report which bricks are connected by studs, because that is BrickSim's own model and not a physics contact. Read it from its Python API with `run_python`, which runs code inside Kit: `bricksim.core.get_assembled_connections()` and `are_parts_connected()`. A tester with no other help got each brick's position and the connection graph this way.

## Training a project task

BrickSim projects can register their own Isaac Lab task packages through `ISAACLAB_TASK_PATH` and
`ISAACLAB_TASK_MODULES`; see [Your own task](TOOLS-ISAACLAB.md#6-your-own-task). Keep task-specific
training adapters, checkpoints and results in the project. Confirm the physics device in each run's logs
rather than inferring it from the task configuration.
