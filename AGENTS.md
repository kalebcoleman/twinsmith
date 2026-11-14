# Agent guide

Start here if you are an AI agent working in or with this repository. It is a map; the per-tool reference lives in `docs/`
([index](docs/README.md), [every tool's arguments](docs/TOOLS-ARGUMENTS.md), [toolsets](docs/TOOLSETS.md)).

## Setup guide for agents

- **Use the Isaac Sim that is already installed. Never download or install Isaac Sim unless the user explicitly asks for it.**
  Lab and shared PCs have their own install: find it with `python3 scripts/env.py find` and install only the MCP server with
  `./install.sh --no-sim` (it uses an installed Isaac Sim 6.x and still records the install it finds). Do not run
  `./install.sh --download-sim`, `uv sync`, `pip install isaacsim`, or `uv run`/`uv lock`/`uv add` in the repository root (the root
  project depends on Isaac Sim, so they create a ~20 GB `.venv`) without the user asking; run tools with `python3 scripts/...` or
  `cd mcp && uv run --locked --isolated ...`, and refresh the host lock only with `uv lock --directory mcp`. Do not replace or
  upgrade their Isaac Sim or Isaac Lab environment. If `find` shows nothing, stop and ask where Isaac Sim is installed
  (`ISAACSIM_PATH` or `TWINSMITH_SIM_PYTHON`) instead of installing one.
- **The physics engine.** Newton is the default; PhysX is opt-in. The statement (500 Hz step rate, raised contact buffers), how to choose an engine and the known limits are in
  [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md#target-engine). **The UR5e work cell builds on both engines: PhysX keeps the pack's drives, Newton uses the per-engine drives and solver settings in its config; its cameras are not qualified on Newton, and `mirror_real` is refused there; the Bota force/torque read (`workcell__robot(action="wrench")`) passes the static gravity check on both** ([work cell on Newton](docs/OPEN-ITEMS.md#work-cell-on-newton)). Cautions that stay: verify the selected
  engine (`status` reports it); never switch an open scene's engine; never fall back silently in either direction (a request for
  Newton must not become PhysX, and a request for PhysX must not become Newton); a CPU device does not identify the solver; Lab
  results do not qualify the work cell. Isaac Sim 5.1 and Isaac Lab 2.x are PhysX only. Do not claim a Newton result without a recorded
  run.
- Identify the requested Sim build and workflow from the context or installation; reuse their intended running bridge. The full
  `./install.sh --accept-eula` creates the locked **6.2.0.0** environment; it is not a selector for older builds. For an existing
  compatible 6.0 or 6.1 interpreter see [build selection](docs/LINUX-SETUP.md#choose-a-simulator-build). Follow the user's
  authorization for starting GPU work and avoid replacing an existing environment.
- Read [Linux prerequisites](docs/LINUX-SETUP.md) and the [compatibility matrix](docs/COMPATIBILITY.md) before recommending
  packages. The host, Sim, external system ROS and Isaac Lab can use different environments. Install external ROS or Isaac Lab only
  when the requested workflow needs them. The supplied launcher enables NVIDIA's bundled ROS bridge; that does not install system ROS.
- NVIDIA supplies the `omni.kit.hydra_texture` extension with the inspected Sim installations. Its `.pyi` files describe an API;
  do not tell users to pip-install stubs or `omni`. Kit modules become available inside the running app.
- Client setup is in [MCP-SERVER.md](docs/MCP-SERVER.md#4-client-configuration). After connecting, **call `status` once**, before the
  first simulator action: one reply names the selected bridge (PID, port), every advertised bridge, what is running, the stage's
  `session_id` and `stage_id`, and which toolsets are on and off. With nothing running it says so and names the next call. Do not
  repeat it before each call or after a context reset: a scene tool that meets a changed stage answers `STALE_TARGET` with the new
  IDs. A listed tool does not prove a running or compatible simulator.
- Native ROS camera and LiDAR render handles can make `sim__stage(action="save")` refuse with `SAVE_UNSUPPORTED`: inspect its `properties`
  and `retry_with`, and retry with `reset_camera_render_handles=true` only when normalizing the exported copy is intended. Pause
  before saving and verify the copy in a fresh process ([scene reference](docs/TOOLS-SCENE.md#saved-lidar-scenes-and-scan)).
- A `PHYSICS_STATE_INVALID` result from a physics read or step reports invalid observed runtime data: inspect its backend, path
  and field, do not rewrite the caller's pose, and do not count elapsed simulation time as valid physics or retry blindly. There is
  no rollback. `DEVICE_ERROR` quarantine still applies to GPU faults. Successful error reporting does not establish solver recovery.

## Which tools are listed: tool groups

Tools are grouped into **toolsets** that are switched on or off by `~/.config/twinsmith/config.toml`, a project `twinsmith.toml`,
`TWINSMITH_TOOLSETS` or `--toolsets` (later sources override earlier ones). Start with `status`: its `toolsets` field names what is on
and off and how to enable more. `twinsmith toolsets` prints the same table from a terminal. Disabled toolsets register no tools.
Toolsets control what is listed, not what is allowed: the bridge token and `TWINSMITH_READ_ONLY=1` are enforced inside Kit.

<!-- generated:toolsets-compact -->
| Toolset | Default | Tools | What it holds |
|---|---|--:|---|
| `core` | on | 6 | Connection, discovery, help, jobs and toolsets |
| `computer` | explicit only | 7 | Screen, windows, recordings, mouse and keyboard, simulator processes |
| `sim` | off | 31 | Isaac Sim and Isaac Lab: scenes, physics, sensors, code in Kit, ROS 2, the work cell, Lab jobs |
| `assets` | off | 5 | Import and manage assets |
| `cad` | off | 1 | PCB checks, fab files, renders |
| `real2sim` | off | 2 | RoboSimStudio real-to-sim and MuJoCo sysid |
| `planning` | off | 1 | Graph-as-Policy skill graphs |
| `eval` | off | 1 | VABAR benchmark |
<!-- /generated -->

The counts are listed tools (a grouped tool with an `action` argument counts once) and are generated from the code. The sizes of
each toolset are in [docs/TOOLSETS.md](docs/TOOLSETS.md#size-of-the-tool-list). Only `core` is on by default: scene work needs
`sim` (`TWINSMITH_TOOLSETS=default,+sim,+assets`, or `toolsets(action="enable", names=["sim"])` at run time); `sim` also holds
ROS 2, the work cell and Isaac Lab. `core` is always on. `computer` (screen, desktop input, simulator launch) is explicit only:
`all` excludes it and only the user can select it, at startup (`all,+computer`).

Every registered tool is in the generated [TOOLS-ARGUMENTS.md](docs/TOOLS-ARGUMENTS.md), grouped by toolset, and the pages
[TOOLS-HOST](docs/TOOLS-HOST.md), [TOOLS-SCENE](docs/TOOLS-SCENE.md), [TOOLS-ISAACLAB](docs/TOOLS-ISAACLAB.md),
[ROBOSIM](docs/ROBOSIM.md) and the [work cell](exts/workcell/workcell/README.md) describe behaviour. `workcell__cell(action="quick_start")`
builds a stopped scene; joint tools take the articulation root, not the robot's parent Xform (`sim__ros2_tf` reports it). See
the [scene workflow](docs/TOOLS-SCENE.md#a-typical-session).

After reopening a scene, its sensors and prims are not pack-owned: pass `adopt=true` to `sim__lidar(action="read")` or
`sim__camera(action="capture")`, and `allow_foreign=true` to `sim__prim(action="delete")`, which deactivates rather than removes
([reopened scenes](docs/TOOLS-SCENE.md#reopened-scenes)). Camera capture defaults to a fresh bounded renderer CPU readback; with
`wait_for_render=false` it writes the last validated image and reports its age, an unseeded camera returns `NOT_READY`, and cached
pixels can predate Stop, reset or scene edits.

## Getting the full surface

1. Call `status` first, once; it answers with or without a bridge. `refresh_tools` re-reads a bridge's tools after Kit starts or
   changes. From the checkout, `python3 mcp/twinsmith/client.py --list-bridges` lists advertised PID, host and port (exit 2 with `[]`
   if none). From elsewhere use `<repo>/mcp/.venv/bin/python -m twinsmith.client --list-bridges`.
2. Without the MCP tools, confirm the intended bridge with `python3 mcp/twinsmith/client.py status --host <host> --port <port>`; use
   `use_bridge` when needed. Pinned port variables override discovery, and an ambiguous unpinned selection is refused.
3. If none exists and the requested work needs one, `./run.sh --headless` starts a bridge and the work cell on port 8766 by default.
   Start additional instances only when explicitly requested and resource checks pass; use distinct ports, ROS domains and output
   directories. Headroom checks are snapshots, not reservations. Do not stop unrelated processes.
4. Refresh again. Clients that ignore `tools/list_changed` may need a connection restart; `list_tools --host <host> --port <port>` on
   the client lists the verified bridge directly.

`isaaclab__launch` (`train`, `eval`, `play`) refuses to start while a Kit bridge runs unless the user knowingly permits
`allow_concurrent_gpu=true`. See [Lab tools](docs/TOOLS-ISAACLAB.md). The [agent skill](docs/AGENT-SKILLS.md) adds workflows and
links to NVIDIA's official skills.

## Handle with care

- Project USD tools require an explicitly user-approved trusted root and shared host and Kit staging; copying includes the entire
  bounded project. Native USD parsers and resolvers are not sandboxed or network-isolated. Known missing or unsupported resources are
  errors; a successful import is not proof of visual fidelity or physics readiness. Check units, up-axis and the resulting prim and
  viewport ([limits](docs/TOOLS-SCENE.md#project-usd-and-trusted-roots)).
- `run_python` executes Python inside Kit, and `files` (`put`, `get`, `stat`) touch the filesystem. Verify the target PID and port
  once, before the first scene-changing call, and again only after `use_bridge` or a simulator restart.
- Long simulator work runs as a job: `run_python` with `background=true`, `sim__simulation(action="run")` and
  `sim__simulation(action="replay")` return a `job_id`. Wait with `jobs(action="status", wait_s=...)` rather than polling, stop with
  `jobs(action="cancel")`, and after a failed run call `sim__timeline` with `action=reset`. A Python controller is code in
  Kit and needs `TWINSMITH_ENABLE_SCENE_SCRIPTS=1` at launch; a `tcp://127.0.0.1:<port>` controller needs nothing.
- `isaaclab__launch` starts GPU jobs and is blocked by a running Kit bridge unless `allow_concurrent_gpu=true`; `jobs(action="cancel")`
  kills one.
- Screen tools return window titles and pixels to the model and save captures under `~/Pictures/twinsmith` and `~/Videos/twinsmith`.
- Desktop input (`computer`, in the explicit-only `computer` group) is off unless the user selects the group. It operates a shared
  X11 desktop and can trigger arbitrary application actions; inspect the actual window, its PID and a screenshot first.
- `simulators` (also in `computer`) starts and stops Isaac Sim processes. Its launch guard counts known bridge, managed Sim and configured Lab job processes and checks headroom,
  but cannot detect every untracked workload. Never stop an unrelated process to make room, and use separate desktops or coordinate
  input between agents.
- Annotations on tools (`readOnlyHint`, `destructiveHint`, ...) are hints, not enforcement. `TWINSMITH_READ_ONLY=1` in Kit is what
  refuses scene writes.

## Importing robots and assets

One tool, `assets__import_path`, imports USD, URDF, xacro, SDF, MuJoCo XML, meshes, ROS maps and folders that hold them
(`format=auto`). Read [docs/ASSET-IMPORT.md](docs/ASSET-IMPORT.md) for formats, options, error codes and limits. A file is converted
on the host and no code from it is ever run; every file it names must be inside an approved folder, and only the user can approve one
(ask for the smallest folder). `sim__quick_start environment=warehouse` builds a warehouse with a robot in it. A new format is an
option on that tool, not a new tool. Packs of assets are in [docs/ASSET-PACKS.md](docs/ASSET-PACKS.md).

## Changing tools

Every registered tool is documented in the generated [docs/TOOLS-ARGUMENTS.md](docs/TOOLS-ARGUMENTS.md), and its toolset in
`mcp/twinsmith/toolsets.py`. No tool count is typed in a page: counts sit in generated blocks. After changing a tool registry or
contract, run `python3 scripts/gen_host_catalog.py --write`, then `python3 scripts/gen_tool_reference.py --write` (it rewrites the
tool reference and every count block), and commit the results. `python3 scripts/check_docs.py` fails on a page that names a tool
that is not registered, on a retired phrase, on stale generated content and on private paths; CI runs it with the host test suite
(`pytest -q` from `mcp/`, see `.github/workflows/ci.yml`). `mcp/tests/test_environment_docs.py` keeps
[docs/ENVIRONMENT.md](docs/ENVIRONMENT.md) equal to the variables the code reads.

Use neutral `<repo>`, `<worktree>`, `<home>` and `<sim-install>` placeholders in public records. `check_docs.py` scans tracked files
and nested archives for private paths; it fails closed on unreadable or oversized archives. `python3 scripts/check_private_paths.py
--commits <range>` also checks commit identities. Inspect screenshot pixels separately. Keep generated logs, checkpoints and large
recordings outside Git; document only the results and media needed to understand or reproduce the work.
