# twinsmith

A tool for building and driving robot simulations and digital twins from an AI agent: you describe what you want, and
the agent builds the scene, moves the robot, looks through its cameras and reads its sensors. It works with
NVIDIA Isaac Sim and Isaac Lab today, and adds ROS 2, folders of verified assets called asset packs, a lab work cell,
and a bridge to RoboSimStudio. It speaks [MCP](https://modelcontextprotocol.io), so Claude Code, Codex, OpenCode
and other MCP clients can use it.

[![CI](https://github.com/kalebcoleman/twinsmith/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/kalebcoleman/twinsmith/actions/workflows/ci.yml)
[![Source version 0.3.0](docs/media/badges/version.svg)](mcp/pyproject.toml)
[![Isaac Sim scoped tests](docs/media/badges/sim.svg)](docs/COMPATIBILITY.md)
[![Host Python 3.10+, Sim Python 3.12](docs/media/badges/python.svg)](docs/LINUX-SETUP.md#which-environment-does-what)
[![License and asset notices](docs/media/badges/license.svg)](docs/VENDORING.md)

[Quick start](#quick-start) · [Tool groups](#tool-groups) · [Examples](docs/EXAMPLES.md) · [How it works](docs/ARCHITECTURE.md) ·
[Documentation index](docs/README.md) · [For agents](AGENTS.md)

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
| **`real2sim`** | off | RoboSimStudio real-to-sim and MuJoCo sysid | `robosim`, `sysid` |
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
| Use RoboSimStudio | [RoboSimStudio](docs/ROBOSIM.md) |
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
