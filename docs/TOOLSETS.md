# Toolsets

A **toolset** is a set of tools that are switched on or off together. The server registers only the tools of the
enabled toolsets, so a client is shown a short tool list. A toolset that is off registers nothing: no listed tool, no
hidden single-purpose tool. Every registered tool belongs to exactly one toolset.

Some listed tools are **grouped tools**: one tool with an `action` argument that runs several single-purpose tools, for
example `sim__prim(action="create")`. The counts below are listed tools. Every tool and its arguments are in
[TOOLS-ARGUMENTS.md](TOOLS-ARGUMENTS.md), which has one section per toolset.

## The toolsets

The table is generated from the code (`scripts/gen_tool_reference.py --write`); `twinsmith toolsets` prints the same
rows for the server you have installed, with the current state of each.

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

- `core` is always on and cannot be turned off. It is the only toolset on by default: name the others you want.
- `computer` is the desktop and its processes: screenshots, recordings, windows, mouse and keyboard (`computer`), the
  desktop readiness check (`doctor`), joining clips (`join_videos`) and the simulators this server starts and stops
  (`simulators`). It is **explicit only**: never part of a profile, not even `all`, and never enabled at run time. Name it
  at startup to get it (`+computer`). Its tools are not specific to Isaac Sim: `screenshot`, `windows`, `doctor` and
  `computer` work on any X11 window (FreeCAD, RViz 2, a browser), so they carry no `sim__` prefix. The
  [computer use workflow](TOOLS-HOST.md#computer-use-workflow) covers them with `sim__ui`.
- `sim` holds everything for Isaac Sim and Isaac Lab: the stage, prims, physics, joints, sensors, code in Kit
  (`run_python`, `files`, `reload_tools`, `sim__controller`), Kit's own log and UI, simulator-side ROS 2 (`sim__ros2_tf`,
  `sim__graph`, `sim__occupancy_map`), the lab's UR5e work cell (`workcell__*`, see its
  [README](../exts/workcell/workcell/README.md)), Isaac Lab jobs (`isaaclab__*`) and `run_script`, which runs any script as a
  host job. It also lists the tools a project's own Kit extension registers under a namespace this repository does not
  catalogue, once that extension connects.
- `real2sim` holds two grouped tools, each running in its own Python environment and needing no Kit bridge: `robosim`
  runs RoboSimStudio ([ROBOSIM.md](ROBOSIM.md)) and `sysid` fits model parameters with MuJoCo ([SYSID.md](SYSID.md)).
- `cad` holds `hardware`: KiCad checks, fab files and renders from the tools beside twinsmith (KiCad, KiKit, Blender);
  it needs no Kit bridge and never uploads or orders. See [HARDWARE.md](HARDWARE.md).
- `planning` holds `gap` (Graph-as-Policy, simulation only) and `eval` holds `vabar` (the VABAR benchmark). Each runs a
  pinned checkout with its own environment and needs no Kit bridge. See [PLANNING-EVAL.md](PLANNING-EVAL.md).

## Choosing toolsets

Five sources set the toolsets. Each later source overrides the earlier ones:

| Order | Source | Where |
|---|---|---|
| 1 | Built-in default | The `default` profile |
| 2 | User file | `~/.config/twinsmith/config.toml` |
| 3 | Project file | `twinsmith.toml`, found by walking up from the server's working directory |
| 4 | Environment | `TWINSMITH_TOOLSETS` |
| 5 | Command line | `twinsmith --toolsets ...` |

A source that does not mention toolsets leaves the choice from the sources before it. In the files the list is
`toolsets` under `[tools]`:

```toml
[tools]
toolsets = ["default", "+sim"]
```

In the environment and on the command line it is one comma-separated string, `default,+sim`.

### Syntax

The list starts from a profile and adjusts it, left to right:

- `default` or `all` replaces everything chosen so far with that profile.
- `+name`, or a bare `name`, adds a toolset.
- `-name` removes one.
- `core` is always on, so a list that names no profile starts from `core` alone.

An unknown name, or `-core`, stops the server at startup and lists the valid names. Earlier toolset names (`screen`,
`input`, `launch`, `lab`, `ros2`, `workcell`, `extensions`, and before them `scene`, `physics`, `sensors`, `code`,
`kit_debug`, `robosim`) and the `minimal` profile are unknown names now; see [renamed toolsets](#renamed-toolsets). The choice is read once, when the
server starts: after changing a file or variable, restart the MCP server (your client's reconnect or restart action).

Three worked examples:

1. **Scene work.** The simulator tools and asset import.

   ```toml
   [tools]
   toolsets = ["default", "+sim", "+assets"]
   ```

2. **Scene work with the desktop.** The same, plus screenshots, mouse and keyboard and simulator launch. Only a person
   can make this choice, at startup.

   ```bash
   TWINSMITH_TOOLSETS=default,+sim,+assets,+computer twinsmith toolsets
   ```

   `twinsmith toolsets` prints the result. In an MCP client's configuration the same value goes in the server's `env`.

3. **Everything.** Every group, the desktop included.

   ```bash
   twinsmith --toolsets all,+computer
   ```

`twinsmith --toolsets default,+sim toolsets` prints the table for that choice without starting the server.

### Profiles

| Profile | Contains |
|---|---|
| `default` | `core` only |
| `all` | every toolset except `computer` |

### Size of the tool list

A client reads every listed tool's description and argument schema, so a smaller list leaves more room for the task.
The counts below are generated from the code; the sizes were measured with
[`scripts/measure_tool_list.py`](../scripts/measure_tool_list.py), which builds the server offline and counts the
characters of what `tools/list` returns. Tokens are estimated as characters divided by four.

<!-- generated:profiles -->
| Profile | Toolsets | Listed tools |
|---|---|--:|
| `default` | `core` | 6 |
| `all` | `core`, `sim`, `assets`, `cad`, `real2sim`, `planning`, `eval` | 47 |
| `all,+computer` | `core`, `computer`, `sim`, `assets`, `cad`, `real2sim`, `planning`, `eval` | 54 |
<!-- /generated -->

Measured on 2026-11-13. Each `default,+name` row is `core` plus that one toolset.

| Selection | Listed tools | Characters | Estimated tokens |
|---|--:|--:|--:|
| `default` | 6 | 6,346 | 1,586 |
| `default,+computer` | 13 | 19,228 | 4,807 |
| `default,+sim` | 37 | 71,505 | 17,876 |
| `default,+assets` | 11 | 17,132 | 4,283 |
| `default,+cad` | 7 | 8,072 | 2,018 |
| `default,+real2sim` | 8 | 12,203 | 3,050 |
| `default,+planning` | 7 | 7,926 | 1,981 |
| `default,+eval` | 7 | 8,440 | 2,110 |
| `default,+sim,+assets` | 42 | 82,291 | 20,572 |
| `all` | 47 | 93,548 | 23,387 |
| `all,+computer` | 54 | 106,430 | 26,607 |

Before toolsets existed the server always listed 48 tools, about 28,450 estimated tokens. The listed text is a summary:
`action="help"` on a grouped tool and `help(tool="<name>")` return the full description. A test
(`mcp/tests/test_toolsets.py`) fails when a toolset or `all` grows past its budget. Measure your own selection; the argument is the
same text the server reads:

```bash
cd mcp
uv run --locked --isolated --python 3.12 --extra test python ../scripts/measure_tool_list.py TWINSMITH_TOOLSETS=default
```

## What is on, and what is off

- **`status`**, the tool an agent calls first, answers with or without a simulator, and carries a `toolsets` field:
  `on` (the enabled toolsets), `off` (each disabled toolset with a one-line summary) and `enable` (how to turn one on). It also reports the stage's physics engine once a simulator answers.
- **`twinsmith://toolsets`** is an MCP resource with the same JSON, for clients and people that read resources.
- **`help`** names the toolset when the tool for a task is off.
- **`twinsmith toolsets`** prints the table from a terminal.

## Enabling a toolset while the server runs

The `toolsets` tool is in `core`, so it is always listed. Its actions:

- `toolsets(action="list")` returns what `status` reports: the toolsets that are on, the ones that are off with a one-line
  summary and tool count, and which can be enabled now.
- `toolsets(action="enable", names=["sim"])` registers those toolsets' tools for the rest of this server process and
  tells the client the tool list changed. The result names the tools that became available. A client that loads tools
  lazily may need to look a new tool up by name before calling it.
- `toolsets(action="disable", names=["sim"])` removes them again. `core` cannot be disabled.

Rules the server enforces:

- `computer` is never enabled this way. It lets a client see and act on the desktop and start simulator processes, so
  only a person can turn it on, in the configuration, before the server starts. A call that names it is refused and
  changes nothing.
- A change made with this tool lasts until the server stops. It is not written to any file.
- `[tools] runtime_enable = false` in a configuration file, or `TWINSMITH_RUNTIME_ENABLE=0`, turns `enable` and `disable`
  off; `list` still answers.

What the tool changes is the listed tools; it does not change any permission.

## Toolsets control what is listed, not what is allowed

Toolsets are a visibility preference. They are **not a security boundary**. Two things are enforced inside the simulator,
whatever a client is shown:

- the bridge **token**, which every connection must present;
- **read-only mode** (`TWINSMITH_READ_ONLY=1` where Kit starts), which refuses every tool that changes state.

A client that can reach the bridge with the token can still send any registered call to it. To limit what an agent can
change, start the simulator read-only; see [limiting what a client can reach](MCP-SERVER.md#limiting-what-a-client-can-reach).

## Old settings

The tool-selection variables of 0.2 are gone and nothing reads them. Use `TWINSMITH_TOOLSETS`.

## Renamed toolsets

Old toolset names are not kept as aliases: a configuration that names one stops the server at startup with the list of
valid names. Rename them in `[tools] toolsets`, `TWINSMITH_TOOLSETS` and `--toolsets`:

| Before | Now |
|---|---|
| `screen`, `input`, `launch` | `computer` (explicit only, as `input` and `launch` were). `screen` alone could be enabled at run time; `computer` cannot, and selecting it for screenshots also lists mouse and keyboard and simulator launch |
| `lab`, `ros2`, `workcell`, `extensions` | `sim` |
| `scene`, `physics`, `sensors`, `kit_debug`, `code` | `sim` |
| `robosim` | `real2sim` (the tool keeps its name, `robosim`) |
| `minimal` | `default`, which is `core` alone |
| `[screen] cursor_badge` in `config.toml` | `[computer] cursor_badge` |
| The work cell's `real2sim`, `planning` and `eval` status tools, which answered "not implemented" | Deleted, with no replacement tool |
| The `planned` list in `status` and `twinsmith://toolsets` | Deleted |

Renamed tools, also with no alias:

| Before | Now |
|---|---|
| `sim__session` (`sim__launch`, `sim__list`, `sim__status`, `sim__stop`) | `simulators` (`launch_simulator`, `list_simulators`, `simulator_status`, `stop_simulator`) |
| `sim__timeline__control` | `sim__timeline` |
| `sim__viewport__frame` | `sim__viewport` |
| `sim__ros2__tf_setup` | `sim__ros2_tf` |
| `sim__map__occupancy` | `sim__occupancy_map` |

`core`, `sim`, `assets`, `cad`, `real2sim`, `planning` and `eval` keep their names.
