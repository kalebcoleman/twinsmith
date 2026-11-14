# Documentation index

Start with the [project README](../README.md). Agents should start with [AGENTS.md](../AGENTS.md).
When two pages disagree, trust the **current reference** over a design or history page, and the code
over both.

## Current reference

**Set up and connect**

| Page | Use it for |
|---|---|
| [Linux setup](LINUX-SETUP.md) | Prerequisites, install paths, build selection, the workspace folder and what is auto-detected, first-connection troubleshooting |
| [MCP server](MCP-SERVER.md) | Client configuration (Claude Code and Desktop, Codex, OpenCode, Antigravity), limiting what a client can reach |
| [Toolsets](TOOLSETS.md) | The toolsets, how to switch them (config file, environment, command line), the size of each profile |
| [Environment variables](ENVIRONMENT.md) | Every setting, its default and which process reads it |
| [Compatibility](COMPATIBILITY.md) | The target physics engine and its known limits, tested simulator builds and Isaac Sim 6.2 notes |
| [Asset packs](ASSET-PACKS.md) | The one pack model: `pack.yaml`, the asset root, list/doctor/adopt/pin/build, converting an old pack, moving packs between machines |
| [Importing any asset](ASSET-IMPORT.md) | One import for USD, URDF, xacro, SDF, MJCF and robot folders: formats, safety, physics after import |

**Tools**

| Page | Covers |
|---|---|
| [Tool arguments](TOOLS-ARGUMENTS.md) | Every tool's arguments and defaults, one section per toolset, generated from the code |
| [Scene and assets](TOOLS-SCENE.md) | `sim__*`, `assets__*`: trusted projects, contracts, sensors, physics, limits |
| [Host and bridge](TOOLS-HOST.md) | Gateway, desktop, opt-in input and sessions, bridge built-ins |
| [Isaac Lab](TOOLS-ISAACLAB.md) | `isaaclab__*`: discovery, jobs, guards, metrics |
| [RoboSimStudio](ROBOSIM.md) | The `robosim` tool: real-to-sim runs in RoboSimStudio's own environment |
| [System identification](SYSID.md) | The `sysid` tool: fit damping, friction, gains and mass to recorded trajectories with MuJoCo |
| [Hardware](HARDWARE.md) | The `hardware` tool (`cad`): KiCad checks, fab files and renders from tools beside twinsmith |
| [Planning and evaluation](PLANNING-EVAL.md) | The `gap` and `vabar` tools: Graph-as-Policy skill graphs in simulation and VABAR benchmark runs, from pinned checkouts |
| [Work cell](../exts/workcell/workcell/README.md) | The lab's UR5e Jenga work cell: BOM, private pack, tools. [Jenga variation contract](TOOLS-WORKCELL.md) |
| [BrickSim](BRICKSIM.md) | LEGO bricks with these tools: the bridge next to BrickSim, which tools work on bricks, its RL task |
| [Agent skill](AGENT-SKILLS.md) | The `twinsmith` skill and where each client finds it |

**Workflows and examples**

| Page | Use it for |
|---|---|
| [Examples](EXAMPLES.md) | What to ask an agent, with pictures from real runs |
| [ROS 2 and RViz 2](ROS2-RVIZ.md) | TF setup, RViz config, the TF doctor, Nav2 maps |
| [Verification](VERIFICATION.md) | The test matrix: CPU checks, the live smoke per toolset and engine, what each proves, what to record |

**Understand and extend**

| Page | Use it for |
|---|---|
| [How it works](ARCHITECTURE.md) | The whole system with a diagram: client, host server, bridge, Kit extensions, Isaac Lab jobs, asset packs, RoboSimStudio, where each setting is read. Its last section is the [target layout](ARCHITECTURE.md#target-layout-planned): groups, process boundaries, version pins, safety boundaries |
| [Host package readme](../mcp/README.md) | The server package, install, layout |
| [Tool authoring](../mcp/docs/tool-authoring.md) | Your own tool package: arguments, errors, flags, reloading, Kit surprises |
| [Bridge protocol](../mcp/docs/protocol.md) | Messages, finding a bridge, the token, scheduling, read-only mode, jobs, access from another machine |
| Extension notes: [bridge](../exts/twinsmith.bridge/docs/README.md) · [sim-tools](../exts/sim-tools/README.md) · [workcell](../exts/workcell/docs/README.md) | What each Kit extension is |

**Project**

| Page | Use it for |
|---|---|
| [Roadmap](ROADMAP.md) | The phases from cleanup to the real-to-sim-to-real Jenga loop, each with goal, scope, exit criteria and dependencies; the pull request queue; decisions needed |
| [Plans](plans/README.md) | One specification per upcoming pull request, ready to hand to an agent |
| [Open items](OPEN-ITEMS.md) | Unfinished work by topic, including the Newton qualification list |
| [Releasing](RELEASING.md) | Bump the version, tag, and the PyPI trusted publishing that uploads it |
| [Vendoring](VENDORING.md) · [Third-party notices](../THIRD-PARTY-NOTICES.md) | Licenses and origins |
| [Media](media/README.md) | Provenance of the images in the examples |

## NVIDIA resources

The [agent skill guide](AGENT-SKILLS.md#nvidias-skills) links NVIDIA's documentation MCP and official Isaac Sim and Isaac Lab skills.

## Evidence

Use [compatibility](COMPATIBILITY.md) for tested builds and known limits, and [verification](VERIFICATION.md) for checks. The
examples and navigation walkthrough keep their supporting material under `docs/evidence/`.

## Scripts

Run these from the checkout root. **Disposable** means the script changes the stage, so point it at a session you can throw
away. Scripts that import `mcp` need `uv run --locked --isolated --project mcp --extra scene-test --python 3.12 python scripts/<name>`
(`scene-test` adds the `pxr` and `numpy` the scene validators import); a plain `python3` will not find them.

| Purpose | Scripts |
|---|---|
| Find and prepare | `env.py` (find an install, create a workspace), `assets.py` (asset packs), `install.py` (register the bridge in a Kit app), `check_machine.sh` (6.2 preflight; can use the GPU) |
| Start | `run_kit.py` (what `run.sh` runs) |
| Project checks (CPU, also in CI) | `check_docs.py`, `check_assets.py`, `check_portable_paths.py`, `check_private_paths.py`, `gen_host_catalog.py`, `gen_tool_reference.py` (also rewrites the generated count blocks) |
| Work-cell check (live, disposable) | `workcell_smoke.py` (requires an explicitly selected disposable bridge and lab config; PhysX or Newton) |
| Smoke checks (live, disposable) | `smoke.py <toolset>`; `smoke.py --list` lists GPU/simulator requirements offline. Validators live in `scripts/smoke/`; helpers: `scene_mcp_client.py`, `smoke_mcp_surface.py`, `make_scene_fixture.py`, `make_scene_package_fixture.py` |
| Offline surface audits | `measure_tool_list.py [KEY=VALUE ...] [--json]`, `check_names.py --audit [--json]` (name enforcement is opt-in, not yet CI) |
| ROS 2 | `ros_env.sh` (source it), `ros2_rviz_config.py`, `ros2_tf_doctor.py`, `cmd_vel_sequence.py` |

`mcp/templates/` holds bridge integration templates.
