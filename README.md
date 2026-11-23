# twinsmith

twinsmith is the builder for robot digital twins. An AI agent uses it to make a simulated copy of a real robot cell and to
keep that copy close to the real one. It builds scenes in NVIDIA Isaac Sim and Isaac Lab, with Newton physics by default.
It brings in assets: your own packs, or NVIDIA's robots and rooms. It fits the twin to real recordings with
RoboSimStudio and MuJoCo system identification. It also authors, checks and runs Graph-as-Policy (GaP) skill graphs in
simulation. It speaks [MCP](https://modelcontextprotocol.io), so Claude Code, Codex, OpenCode and other MCP clients can
use it.

twinsmith holds only the general machinery. A particular task lives in its project's own repository, along with that
task's measured settings, benchmarks and results. The project pins a twinsmith commit and points twinsmith at its files
(see [Using twinsmith from a project](#using-twinsmith-from-a-project)). No twinsmith tool can reach a real robot.

[![CI](https://github.com/kalebcoleman/twinsmith/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/kalebcoleman/twinsmith/actions/workflows/ci.yml)
[![Source version 0.3.0](docs/media/badges/version.svg)](mcp/pyproject.toml)
[![Isaac Sim scoped tests](docs/media/badges/sim.svg)](docs/COMPATIBILITY.md)
[![Host Python 3.10+, Sim Python 3.12](docs/media/badges/python.svg)](docs/LINUX-SETUP.md#which-environment-does-what)
[![License and asset notices](docs/media/badges/license.svg)](docs/VENDORING.md)

[What it builds](#what-it-builds) · [Quick start](#quick-start) · [Tool groups](#tool-groups) · [Examples](docs/EXAMPLES.md) · [How it works](docs/ARCHITECTURE.md) ·
[Documentation index](docs/README.md) · [For agents](AGENTS.md)

## What it builds

| Area | What is here | Where | Not here yet |
|---|---|---|---|
| Digital twins (Isaac Sim, Isaac Lab, Newton) | Scenes, NVIDIA robots by name, import of USD, URDF, xacro, SDF, MJCF and meshes, physics, sensors and cameras, ROS 2 inside the simulator, Isaac Lab training jobs. One calibrated work cell (UR5e on a rail, RG2, wrist camera), built from a config file and an asset pack | `sim`, `workcell__*`, `isaaclab__*`; [Scene tools](docs/TOOLS-SCENE.md), [Work cell](exts/workcell/workcell/README.md) | The work cell code assumes a UR5e. A different cell needs its own config and pack, plus code changes wherever the UR5e is hard-coded |
| Assets | Your own packs in the asset root (`TWINSMITH_ASSET_ROOT`, by default `<workspace>/assets`), each one checked against its `pack.yaml`. Packs are pulled from and pushed to your own Hugging Face repositories, pinned to a commit. NVIDIA's robots and rooms come from the Isaac Sim assets server. Mesh prep: inspect, decimate, convex hulls, render and collision split, and validation for Newton | `assets__*`, `twinsmith assets`; [Asset packs](docs/ASSET-PACKS.md), [Importing assets](docs/ASSET-IMPORT.md) | No search or download from public libraries (Objaverse, Sketchfab, Hugging Face datasets or NVIDIA SimReady). For those, download the file yourself, then `assets__import_path` or `twinsmith assets adopt` |
| Real2Sim (RoboSimStudio, sysid) | A RoboSimStudio scene from a template (tee, box, Jenga, tower), checked against a reference clip, with push-path tuning and rollouts. MuJoCo system identification from joint trajectories. SAM 3.1 video masks | `robosim`, `sysid`, `sam3`; [RoboSimStudio](docs/ROBOSIM.md), [sysid](docs/SYSID.md) | No mass or friction fitting in RoboSimStudio, and no scene made from an asset pack or a scan. Fitted values are not written back into the twin |
| Planning (GaP) and evaluation (VABAR) | GaP's skill catalog, graph checks without a GPU, graph generation (paid model, owner-allowed), and runs of a graph on GaP's own simulated suites. A tool bundle that lets a graph drive the work cell, in `exts/workcell/gap/tools/workcell`. VABAR benchmark runs | `gap`, `vabar`; [Planning and evaluation](docs/PLANNING-EVAL.md) | `gap(action="run")` runs only GaP's simulated suites, not the work cell. The work cell runs through a script. GaP's self-learning loop isn't wrapped yet |
| Parts and hardware | PCB checks, fab files, renders, and a CAD part turned into a twin asset (KiCad, FreeCAD, Blender and SolidCraft run beside twinsmith) | `hardware`; [Hardware](docs/HARDWARE.md) | |

## Using twinsmith from a project

A project repository (a task, a thesis experiment, a lab demo) uses twinsmith as a tool, and twinsmith knows nothing
about the project:

1. **Pin it.** Name the twinsmith commit the project was run with. Each machine runs the server from a checkout at that
   commit (see [Linux setup](docs/LINUX-SETUP.md)).
2. **Keep the project's files in the project.** Task files are loaded with
   `workcell__cell(action="quick_start", task="<absolute path to task.json>")`, and a copy of the cell config with
   `config_path=`. GaP graphs and skills, run settings, benchmarks, measured corrections and results stay in the project
   too.
3. **Name assets by pack, not by path.** The project names `<pack id>/<version>`. The packs live in the asset root, and
   private ones are pinned in `~/.config/twinsmith/pack-sources.toml` and fetched with `twinsmith assets pull`.
4. **Drive the simulator from scripts by path.** A project script that needs more than the MCP tools can import
   twinsmith's host helpers from the pinned checkout (for example `scripts/workcell_session.py`'s `Session` for the work
   cell) and pass in its own config files.
5. **Keep the real robot in the project.** ROS drivers, the bridge to a real arm, and its safety gates are the
   project's code. Give the simulated arm the same joint limits that the real one is held to. A plan made in the twin
   is then one the real cell accepts.

## Quick start

You need Linux x86_64 (Ubuntu 24.04 or 22.04), [uv](https://docs.astral.sh/uv/getting-started/installation/), an NVIDIA GPU,
and an Isaac Sim 6.x that is already installed. This never downloads a simulator. Run every command from the checkout
root. If you have no Isaac Sim yet, read the [Linux setup](docs/LINUX-SETUP.md#new-pinned-62-installation) first.

**1. Install the server.**

```bash
git clone https://github.com/kalebcoleman/twinsmith.git
cd twinsmith
python3 scripts/env.py find     # lists the Isaac Sim installs it can see
./install.sh --no-sim                 # installs only the MCP server
export PATH="$HOME/.local/bin:$PATH"
twinsmith toolsets                    # prints the toolset table
```

You should see `find` list at least one Isaac Sim; with none it prints `no Isaac Sim / Isaac Lab environment found`, and you
set `ISAACSIM_PATH` or `TWINSMITH_SIM_PYTHON` (see [Linux setup](docs/LINUX-SETUP.md#what-is-auto-detected-and-where-it-looks)).
The installer prints `== using the Isaac Sim already installed: ...` and ends with `== install complete`.
`twinsmith toolsets` prints one row per toolset with `on` or `off` and a tool count.

**2. Start a simulator** and leave that terminal open. The first start can take minutes while shaders compile.

```bash
./run.sh --headless      # no window; leave --headless out for a window. A GPU is needed either way.
```

You should see Kit's startup log and no `FAIL:` line. When it is ready, this prints JSON that contains `"app_ready": true`:

```bash
python3 mcp/twinsmith/client.py status --port 8766
```

**3. Connect a client.** For Claude Code:

```bash
./install.sh --no-sim --claude-code
```

For another client, add this to its MCP configuration (see [client configuration](docs/MCP-SERVER.md#4-client-configuration)):

```json
{"mcpServers": {"twinsmith": {"command": "twinsmith", "env": {"TWINSMITH_PORT": "8766"}}}}
```

Only `core` is on by default. Turn on the simulator tools for every client in `~/.config/twinsmith/config.toml`:

```toml
[tools]
toolsets = ["default", "+sim", "+assets"]
```

Start a new client session. The client should list the `core` tools and those of the toolsets you added (see the table
below). An agent can also turn one on for its own session with `toolsets(action="enable", names=["sim"])`.

**4. Call `status`.** Ask the agent: *"Call status and tell me what it reports."* Or call the tool yourself. It answers
with `running: true`, the simulator's `pid`, `port` and `app_version`, the stage's `session_id` and `stage_id`, the
timeline state, and a `toolsets` field naming what is on and what is off. With no simulator it says `Nothing is running`
and names the next call. Call it once; the answer does not need repeating before each tool.

**5. Build a scene.** Ask: *"Call sim__quick_start, then take a picture with sim__camera and tell me what you see."*
`sim__quick_start` needs no arguments. It builds a ground plane, a physics scene, lights and a camera in a new stage, leaves
the timeline stopped, and returns the new `session_id`, `stage_id`, the paths it made and the next steps. Add a robot by
name (`robot="franka"`) or a whole scene (`environment="warehouse"`). More prompts are in [Examples](docs/EXAMPLES.md).

**6. From CAD to the twin (optional).** With [SolidCraft](docs/HARDWARE.md#from-a-solidcraft-part-to-the-work-cell)
installed (`scripts/install_solidcraft.sh --register`) and `+cad,+assets,+sim` on, ask: *"Design a parametric bracket in
solidcraft and save it to the cad folder, then build it with hardware action=part into the twin folder, make CoACD
collisions, validate it for Newton and set it on the work cell's bench."*

**7. Your own assets (optional).** The repository ships none, and none are needed for the steps above: NVIDIA's robots and
scenes load by name. A folder of your own files becomes a pack with `twinsmith assets adopt`. If you keep packs in a
Hugging Face repository, list them in `~/.config/twinsmith/pack-sources.toml` (format: `config/pack-sources.example.toml`) and pull them:
`twinsmith assets pull --all`. See [pulling and pushing packs](docs/ASSET-PACKS.md#pulling-and-pushing-packs).

If a step fails, see [troubleshooting the first connection](docs/LINUX-SETUP.md#troubleshooting-the-first-connection).

## Tool groups

Tools come in groups (toolsets) that you switch on or off. A client is shown only the tools of the enabled groups, which
keeps the tool list small. Only `core` is on by default. Related tools share one grouped tool with an `action` argument.
`sim__` marks a tool that runs inside Isaac Sim; the desktop tools have plain names and work on any X11 window.

<!-- generated:toolsets-full -->
| Group | Default | What it holds | Listed tools |
|---|---|---|---|
| **`core`** | always on | Connection, discovery, help, jobs and toolsets | `help`, `jobs`, `refresh_tools`, `status`, `toolsets`, `use_bridge` |
| **`computer`** | explicit only | Screen, windows, recordings, mouse and keyboard, simulator processes | `computer`, `doctor`, `join_videos`, `screen_recording`, `screenshot`, `simulators`, `windows` |
| **`sim`** | off | Isaac Sim and Isaac Lab: scenes, physics, sensors, code in Kit, ROS 2, the work cell, Lab jobs | `files`, `isaaclab__checkpoint_register`, `isaaclab__inspect`, `isaaclab__launch`, `isaaclab__metrics`, `reload_tools`, `run_python`, `run_script`, `sim__arm`, `sim__camera`, `sim__controller`, `sim__graph`, `sim__joints`, `sim__lidar`, `sim__light`, `sim__logs`, `sim__material`, `sim__occupancy_map`, `sim__physics`, `sim__prim`, `sim__quick_start`, `sim__ros2_tf`, `sim__simulation`, `sim__stage`, `sim__timeline`, `sim__ui`, `sim__viewport`, `workcell__cell`, `workcell__jenga`, `workcell__perception`, `workcell__robot` |
| **`assets`** | off | Import and manage assets | `assets__import_path`, `assets__library`, `assets__mesh`, `assets__pack`, `assets__validate` |
| **`cad`** | off | PCB checks, fab files, renders | `hardware` |
| **`real2sim`** | off | RoboSimStudio real-to-sim, MuJoCo sysid and SAM 3.1 video masks | `robosim`, `sam3`, `sysid` |
| **`planning`** | off | Graph-as-Policy skill graphs | `gap` |
| **`eval`** | off | VABAR benchmark | `vabar` |
<!-- /generated -->

Turn one on in `~/.config/twinsmith/config.toml`, with `TWINSMITH_TOOLSETS` or with `--toolsets`:

```toml
[tools]
toolsets = ["default", "+sim", "+real2sim", "+computer"]
```

`computer` (screen, mouse and keyboard, simulator launch) is explicit only: it is left out of `all`, and an agent cannot
turn it on with `toolsets(action="enable")`; only you can, at startup.

[TOOLSETS.md](docs/TOOLSETS.md) has the precedence, the syntax and the measured size of each profile. Every tool and
argument is in [TOOLS-ARGUMENTS.md](docs/TOOLS-ARGUMENTS.md), generated from the code.

**Physics engine.** The target engine and the current default are stated in
[COMPATIBILITY.md](docs/COMPATIBILITY.md#target-engine). The Isaac Sim builds tested so far, and their limits, are in the same page.

> [!WARNING]
> `run_python`, the file tools and trusted asset loading run with your account's permissions and are not sandboxed.
> Keep the bridge on loopback and approve only project folders you trust. The bridge accepts only connections that
> present its token, which your own account's clients find automatically, and `TWINSMITH_READ_ONLY=1` starts a simulator
> that agents can inspect but not change. Toolsets control what is listed, not what is allowed.
> See the [trust limits](docs/TOOLS-HOST.md#trust-and-operational-limits).

## Where to go next

| I want to... | Read |
|---|---|
| See what an agent can do, with pictures | [Examples](docs/EXAMPLES.md) |
| Understand the parts and where each setting is read | [How it works](docs/ARCHITECTURE.md) · [Environment variables](docs/ENVIRONMENT.md) |
| Choose which tools a client is shown | [Toolsets](docs/TOOLSETS.md) |
| Find an argument or default | [Tool arguments](docs/TOOLS-ARGUMENTS.md) |
| Set up a machine or configure a client | [Linux setup](docs/LINUX-SETUP.md) · [Client configuration](docs/MCP-SERVER.md#4-client-configuration) |
| Import my own robot, or use asset packs | [Importing assets](docs/ASSET-IMPORT.md) · [Asset packs](docs/ASSET-PACKS.md) |
| Use scene, physics and sensor tools | [Scene tools](docs/TOOLS-SCENE.md) |
| Train a policy with Isaac Lab | [Isaac Lab tools](docs/TOOLS-ISAACLAB.md) |
| Use ROS 2, RViz 2 or Nav2 | [ROS 2 and RViz 2](docs/ROS2-RVIZ.md) |
| Run the lab's UR5e work cell | [Work cell](exts/workcell/workcell/README.md) · [work-cell tools](docs/TOOLS-WORKCELL.md) |
| Use RoboSimStudio or fit a model to recordings | [RoboSimStudio](docs/ROBOSIM.md) · [sysid](docs/SYSID.md) |
| Author, check or run a GaP skill graph | [Planning and evaluation](docs/PLANNING-EVAL.md) |
| Run LEGO bricks (BrickSim) | [BrickSim](docs/BRICKSIM.md) |
| Check a machine or read the test matrix | [Verification](docs/VERIFICATION.md) · [Compatibility](docs/COMPATIBILITY.md) |
| Write my own tools | [Tool authoring](mcp/docs/tool-authoring.md) |
| Work as an agent in this repository | [AGENTS.md](AGENTS.md) · [Agent skill](docs/AGENT-SKILLS.md) |

## Development

The host server is in `mcp/`, the Kit extensions in `exts/`, and the helpers in `scripts/`. Before pushing, run the CPU-only
checks CI runs:

```bash
python3 scripts/check_docs.py
python3 scripts/check_assets.py
python3 scripts/check_portable_paths.py
python3 scripts/gen_host_catalog.py --check
python3 scripts/gen_tool_reference.py --check
cd mcp && uv run --locked --isolated --python 3.12 --extra test --extra isaaclab --extra scene-test pytest -q
```

CI covers the host only; it does not establish GPU or simulator behaviour. The live checks are in
[Verification](docs/VERIFICATION.md). Do not run `uv sync` or `uv run` in the repository root unless you want the 20 GB
simulator environment.

## Credits and license

MIT licensed. This project grew out of work by Aldrin Inbaraj, a lab mate: the UR5e work cell began as a fork of
his MIT-licensed [isaac-sim-mcp](https://github.com/inbarajaldrin/isaac-sim-mcp), and the bridge and client were first
built on his [omni-kit-mcp](https://github.com/inbarajaldrin/omni-kit-mcp) before being rewritten here. Thank you, Aldrin.
Component and asset terms differ, so read [VENDORING](docs/VENDORING.md) and the
[third-party notices](THIRD-PARTY-NOTICES.md) before redistributing. NVIDIA's installed runtime packages remain subject to
their own terms.
