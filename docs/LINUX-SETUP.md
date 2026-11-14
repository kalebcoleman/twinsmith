# Linux setup and dependencies

This guide covers the repository's Linux launcher, host MCP server, optional Isaac Lab tools, and where everything lives.
Install the parts needed for your workflow; the host server, simulator and external ROS nodes run in separate environments.

**Keep it organized.** Put the checkout, the asset root and Sim environments in one workspace folder and let the tool find an
existing install. See [One workspace folder](#one-workspace-folder).

## Platforms and prerequisites

This guide is **Linux x86_64 only**; Windows and macOS are not supported (a Windows `C:` install
cannot be used). The full simulator installer targets **Ubuntu 24.04 or 22.04 on x86_64**. It selects ROS 2 Jazzy on 24.04 and
Humble on 22.04; NVIDIA recommends 24.04/Jazzy and marks 22.04 support as deprecated. Check NVIDIA's current
[Isaac Sim 6.2 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html)
for GPU, driver, RAM and storage guidance; this repository's installer requires an NVIDIA GPU, a working driver,
and at least 40 GB of free space. The installer also needs `uv`, internet access, and acceptance of the NVIDIA
Omniverse EULA. `scripts/check_machine.sh` additionally checks `git`, `curl`, networking and the display.

The recorded test machine on 2026-10-31 was Ubuntu **24.04.5 LTS (Noble)**, x86_64, NVIDIA GeForce RTX 5080 with
driver 580.178.04, Python 3.12.3, Isaac Sim 6.2.0.0, Isaac Lab 3.0.0, and ROS 2 Jazzy. That is one tested
configuration, not a claim that every Ubuntu, driver, simulator or hardware combination has been tested. NVIDIA's
published tested driver can differ; consult the requirements link above.
See [compatibility](COMPATIBILITY.md) for tested simulator releases and their individual limitations.

## Which environment does what

| Environment | Required for | Selection |
|---|---|---|
| Host Python **3.10+** | MCP connection, tool discovery, desktop capture, Lab job management | `twinsmith` installed by `uv tool`; independent of Sim |
| Sim Python **3.12** | Kit, rendering, physics, bridge and scene/UR tools | Explicit `TWINSMITH_SIM_PYTHON`, else checkout `.venv`, else auto-detected (see [One workspace folder](#one-workspace-folder)) |
| Optional Isaac Lab | Training, evaluation and play | Existing `ISAACLAB_PYTHON` and `ISAACLAB_ROOT`; select a compatible Sim/Lab version pairing |
| Optional system ROS | External ROS nodes: RViz 2, Nav2 | Matching system ROS distribution, separate from Sim's bundled bridge |

No personal environment name or directory layout is required. Keep existing installations where they are;
`<workspace>` below means your workspace folder ([One workspace folder](#one-workspace-folder)).
A callable offline catalog proxy still needs its runtime prerequisites to execute.

## New pinned 6.2 installation

Use a **new dedicated checkout** (recommended location: `<workspace>/twinsmith`, see
[One workspace folder](#one-workspace-folder)) and run every command below from its root. Install
[uv](https://docs.astral.sh/uv/getting-started/installation/) first, and ensure Git and your NVIDIA
driver are available. Read NVIDIA's [requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html)
and EULA before accepting it. The repository installer checks Ubuntu, architecture, GPU,
driver and free space; its 40 GB floor is not a promise that every asset/cache workload fits.

```bash
mkdir -p <workspace>
git clone https://github.com/kalebcoleman/twinsmith.git <workspace>/twinsmith
cd <workspace>/twinsmith
uv --version
nvidia-smi
./install.sh --accept-eula --download-sim   # this section is the pinned 6.2 download; without --download-sim an installed Isaac Sim 6.x is used instead
export PATH="$HOME/.local/bin:$PATH"
uv pip check --python .venv/bin/python
.venv/bin/python -c 'import importlib.metadata as m, sys; print(sys.version); print(m.version("isaacsim"))'
```

`main` is the development branch, not a released stable tag. Record its exact
`git rev-parse HEAD` for reproducibility. Expect Sim **6.2.0.0** and Python **3.12**.
Stop and inspect any dependency-check failure before treating the environment as ready.

The convenience installer:

- When it finds an installed Isaac Sim 6.x and the checkout has no `.venv`, it installs only the MCP server and writes that
  interpreter into the Claude entries it creates. If nothing is found, it continues to the checks below and **stops at the
  EULA check unless `--accept-eula` was given or `OMNI_KIT_ACCEPT_EULA=YES` is already exported; in those two cases it
  downloads the environment**. An existing checkout `.venv` is re-synced unless you pass `--no-sim`, the one form that never
  downloads or syncs a simulator. `--no-sim` still looks for an installed Isaac Sim 6.x and records it in client entries.
- Runs **`uv sync --python 3.12` in the current directory** only when it is installing the simulator.
  That sync may update the root lock's environment; it does not enforce `--locked` and does not use
  `TWINSMITH_SIM_PYTHON` as its install target.
- Installs the host from `mcp/uv.lock`, not from a fresh resolve:
  `uv sync --project mcp --locked --extra isaaclab --no-dev`, with `UV_PROJECT_ENVIRONMENT` set to
  `mcp/.venv` so an activated simulator environment is left alone. `uv tool install` does not read
  `uv.lock`. The installer then links `~/.local/bin/twinsmith` at that executable. Keep the checkout.
- Creates or refreshes its `~/.local/bin/ffmpeg` shim, preserving unrelated files/symlinks.
- Changes client configuration **only** when requested with registration flags.
  `--claude-code` replaces the user-scope `twinsmith` entry and removes the legacy `twin` entry;
  `--claude-desktop` updates an existing Desktop config with a backup. `--claude` requests both.
  `--print-opencode` prints configuration without writing it. Missing clients/configs can be skipped;
  inspect installer output and use the [manual client guide](MCP-SERVER.md#4-client-configuration).
- Does not install system ROS, Isaac Lab or private workstation assets.

To record an installation, see [recording a run](VERIFICATION.md#recording-a-run-so-it-can-be-reproduced).
The full installer has not been qualified from a clean machine. Its commands and parser have
been inspected; the recorded fresh-venv experiment used a separate locked sync, not `install.sh`.

### Start the simulator

From the same checkout root, after accepting the EULA:

```bash
./run.sh --port 8766
```

`run.sh` chooses the Sim interpreter in this order: `TWINSMITH_SIM_PYTHON`,
then `<repo>/.venv/bin/python`, then auto-detection (`scripts/env.py resolve-python`). It prints
the interpreter it chose and how to pin it. See [what is auto-detected](#what-is-auto-detected-and-where-it-looks).

Use an unused port. A live bridge may already exist: inspect it before starting another process.
The launcher enables the bridge, the generic scene tools, the work-cell tools and NVIDIA's bundled ROS bridge. The physics
engine is chosen with `--physics-backend` or `TWINSMITH_PHYSICS_BACKEND`; Newton is the default and PhysX is opt-in (the
UR5e work cell needs PhysX); see
[compatibility](COMPATIBILITY.md#target-engine), and a request for an engine the build lacks starts nothing.

For a windowless GPU session, use `./run.sh --headless --port 8766` instead. To leave either running after your shell or a tool's time limit ends, use `setsid nohup ./run.sh --port 8766 > kit.log 2>&1 < /dev/null &`. Headless Sim still
requires the NVIDIA runtime and can render sensors, but has no application window for desktop
capture/input. Windowed tools were exercised on X11/XWayland; native Wayland is not qualified.
Keep the launch terminal available for startup errors. First startup can take minutes for shader
and extension preparation; wait for bridge readiness and `app_ready=true`, not a fixed sleep.
Record your setup and result as described in [recording a run](VERIFICATION.md#recording-a-run-so-it-can-be-reproduced).

`scripts/check_machine.sh` is an optional **6.2-specific** preflight. If a Sim interpreter exists,
it imports the runtime and may execute a Torch CUDA kernel. It is not a CPU-only check and should
not be run against an older build or alongside someone else's GPU job as a generic diagnostic.

## Connect and check without changing the scene

Leave the simulator running; use a second terminal from the same checkout root for terminal checks or registration.

Configure the host as a **stdio MCP server** using the [client guide](MCP-SERVER.md#4-client-configuration),
then reconnect the client. The server command is `twinsmith`, not the simulator Python.
`./install.sh --no-sim --claude-code --port 8766` is the explicit Claude Code registration shortcut;
it also replaces the host tool and handles the shim as described above.

In the MCP client, call `refresh_tools`, select the intended endpoint with `use_bridge` if needed,
then `status` and `sim__stage(action="info")` with no other arguments. Verify PID, port, build, stage identity
and readiness before any editing, reset or playback. If the client does not honor tool-list change
notifications, reconnect it. No bridge, missing extension or missing Lab installation is a failure
or unavailable capability, not a successful simulation operation.

For the equivalent read-only terminal check, use the repository's standard-library client:

```bash
# From checkout root; no Sim imports and no host-package installation needed for this CLI.
python3 mcp/twinsmith/client.py --list-bridges
# Set these to the intended endpoint from that listing, not an arbitrary visible window.
BRIDGE_HOST=localhost
BRIDGE_PORT=8766
python3 mcp/twinsmith/client.py status --host "$BRIDGE_HOST" --port "$BRIDGE_PORT"
python3 mcp/twinsmith/client.py sim.stage.info '{}' --host "$BRIDGE_HOST" --port "$BRIDGE_PORT"
```

The direct bridge CLI uses **`sim.stage.info`**; an MCP client calls **`sim__stage`** with `action="info"`.
`--list-bridges` returns `[]` and exit 2 when no bridge is advertised. With several bridges,
automatic selection refuses; explicitly choose the host/port you intend. These commands do not
build the demo or start physics. Once identity is confirmed, use the [agent workflow](../AGENTS.md)
and [scene reference](TOOLS-SCENE.md) for your actual task.

## Choose a simulator build

**Existing Sim (lab and shared PCs): use it, never download another one.** `./install.sh` detects
an installed Isaac Sim 6.x and then installs only the MCP server (`--no-sim` does the same
explicitly); it downloads the locked 6.2 environment only when nothing is found or with
`--download-sim`. Install only the host, inspect your interpreter, then select it explicitly if needed.
List what the tool can find with `python3 scripts/env.py find`. Replace `<env>` with your own
compatible installation. Do not rename, relocate or reinstall an environment just to match this guide
(a symlink into `<workspace>/envs/` is optional and changes nothing).

```bash
# From checkout root; this replaces only the user-level host tool and may create its ffmpeg shim.
# Export TWINSMITH_SIM_PYTHON first if the install is outside the searched locations, so it is recorded.
./install.sh --no-sim
TWINSMITH_SIM_PYTHON=<env>/bin/python
"$TWINSMITH_SIM_PYTHON" -c 'import importlib.metadata as m, sys; print(sys.version); print(m.version("isaacsim"))'
uv pip check --python "$TWINSMITH_SIM_PYTHON"
TWINSMITH_SIM_PYTHON="$TWINSMITH_SIM_PYTHON" ./run.sh --port 8766
```

Without the override, `run.sh` uses this checkout's `.venv/bin/python` and otherwise searches
the documented locations only ([what is auto-detected](#what-is-auto-detected-and-where-it-looks)).
A vendor installation still needs compatible dependencies for these extensions.
A plain Python `import omni` failure outside Kit does not prove the installation is broken.

| Requested build | Installation path | Qualification |
|---|---|---|
| **6.2.0.0** | New pinned checkout above, locked alternative, or explicit existing interpreter | Per-source/mode results in the matrix; inspect dependency conflicts in existing environments |
| **6.0.0.0, 6.0.0.1, 6.0.1.0, 6.1.0.0** | Separately prepared compatible Python 3.12 environment; explicit `TWINSMITH_SIM_PYTHON` | Scoped historical and follow-up results, not clean-install recipes or latest-fix certification for every mode |
| Other/future builds | No tested setup recipe here | Unverified |

Consult NVIDIA's version-specific [6.0 install guide](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_python.html)
or [6.0.1 guide](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_python.html)
when obtaining older builds. **Do not run the root 6.2 sync in those environments.**
Run their `uv pip install` from outside this checkout or with `--no-config`: inside it, `uv` applies the
root `pyproject.toml`'s 6.2 overrides (`torch`, `numpy`, `packaging`), which caused false conflicts and a
failing `uv pip check` on 6.0.0.1 and 6.1.0.0 until the environments were rebuilt.
The [compatibility record](COMPATIBILITY.md) is authoritative for measured scope: later save,
viewport and URDF follow-ups have superseded some early pending checks, while the original
6.0.1 windowed native-step stall and remaining matrix gates remain open.

## Prepare the work cell's asset pack

The host connection and `sim__stage(action="info")` do not require a robot or workstation, and NVIDIA's public robots
(`ur5e`, `franka`, `unitree_g1`, ...) load by name from the running simulator's own asset root. The UR5e Jenga work cell uses
[a private asset pack](../exts/workcell/workcell/README.md#private-asset-pack) (`ur5e-jenga-workcell/v1`) that you supply. The
repository ships **no assets**, `install.sh` installs packages, not assets, and nothing is ever downloaded for you.

Every pack lives in one **asset root**: `TWINSMITH_ASSET_ROOT`, else `<workspace>/assets`. See [asset packs](ASSET-PACKS.md) for the format and the tools.
Copy the pack folder onto the machine, then check it:

```bash
export TWINSMITH_ASSET_ROOT=<asset-root>        # outside Git; set it for the MCP server and for any Kit you start yourself
python3 scripts/assets.py list              # every <id>/<version> and whether it verifies
python3 scripts/assets.py doctor ur5e-jenga-workcell/v1
```

`list` also names any pack in the old format (files or a `SHA256SUMS`, no `pack.yaml`), which nothing reads; convert one with
`assets.py adopt --from-old` ([converting an old pack](ASSET-PACKS.md#converting-an-old-pack)). Setting the root
only in the MCP client cannot change an already-running Kit process's environment: give Kit the same value.

The files remain subject to their own terms (see [third-party notices](../THIRD-PARTY-NOTICES.md) and
[vendoring](VENDORING.md)); the installer's `--accept-eula` controls simulator setup and is not an asset-redistribution
license. Obtain the necessary permission before copying. A missing or unverifiable pack is a real failed operation: it must
not be counted as a loaded robot.

The **Vention bench** has no public download. Put an approved licensed copy in a pack with `assets.py adopt`. Basic
connection and generic scene work require neither.

## Troubleshooting the first connection

| Symptom | Check and next step |
|---|---|
| `twinsmith` not found | Add the uv tool bin directory to PATH, or use its absolute executable path in the client; keep the editable checkout available |
| Interpreter missing | Run `python3 scripts/env.py find`, select a compatible `TWINSMITH_SIM_PYTHON`, or install in the new checkout; do not overwrite an unrelated environment |
| `uv pip check` fails | Record exact versions and conflicts. Use a new isolated locked environment for comparison; do not infer a native failure's cause from metadata alone |
| Only `status`, `help` and the other `core` tools are listed | Only `core` is on by default. Add `+sim` (and `+assets`, `+computer` as needed) to `[tools] toolsets` or `TWINSMITH_TOOLSETS` and restart the server, or call `toolsets(action="enable", names=["sim"])` (not for `computer`, which is selected only at startup); see [toolsets](TOOLSETS.md#choosing-toolsets) |
| Tools are listed but runtime calls fail | Catalogs can be offline. Check the advertised bridge, `refresh_tools`, the [toolsets](TOOLSETS.md) and the selected endpoint |
| Port already in use / multiple bridges | Verify PID/host/port; reuse the intended bridge or select an unused port for an authorized new session |
| Startup still busy / `NOT_READY` | Inspect startup logs and stage readiness. Camera cache is unseeded until a successful tool capture; ordinary Play does not seed it |
| Missing `omni`, renderer or `.pyi` imports | Verify selected Sim and enabled Kit extensions. Do not pip-install `omni` or hand-copy type stubs |
| `Missing pack ...; sync this private pack first` | The pack is not under the asset root the process resolved. Run `python3 scripts/assets.py where` and `list`, and give Kit the same `TWINSMITH_ASSET_ROOT` |
| `Checksum mismatch` / `Pack contains files not listed` | The pack differs from its `pack.yaml`. Run `assets.py doctor <id>/<version>`; restore the file, or write a new version. See [asset packs](ASSET-PACKS.md) |
| `No asset root: ...` | Set `TWINSMITH_ASSET_ROOT`, or create `<workspace>/assets` |
| `No module named 'Xlib'` when starting the MCP host | You ran the host with Isaac Sim's Python. It has its own dependencies: use the installed `twinsmith`, or `cd mcp && uv run --locked --isolated ...` |
| `ModuleNotFoundError: No module named 'mcp'` from a helper script | Scene helpers need the MCP SDK, and the validators `pxr` and `numpy`: `uv run --locked --isolated --project mcp --extra scene-test --python 3.12 python scripts/<name>` from the checkout root. Do not use plain `uv run` in the root |
| `PHYSICS_NOT_READY` from a joint tool | Pass the articulation root, not the robot's parent Xform; `sim__ros2_tf` reports it. See the [typical session](TOOLS-SCENE.md#a-typical-session) |
| `/scan` missing after reopening a saved scene | Open it with `sim__stage(action="open")`, which repairs stale LiDAR products. See [saved LiDAR scenes](TOOLS-SCENE.md#saved-lidar-scenes-and-scan) |
| `refresh_tools` selected another port | It found no bridge on the pinned port and one session this server launched. The result names it; verify with `status` |
| `status` shows `host.stale: true` | This MCP server predates your `git pull`. Restart the MCP connection |
| `SAVE_UNSUPPORTED` | Inspect returned properties/retry guidance; pause and follow the explicit normalization contract in [scene tools](TOOLS-SCENE.md) |
| `TIMELINE_BUSY` or a call stops responding | Preserve logs and identity; do not repeatedly replay Play or assume a client timeout cancels native work. See the known stall in [compatibility](COMPATIBILITY.md) |
| No display / screenshot fails | Use the actual X11/XWayland display for window tools, or headless sensor capture; neither creates a desktop session |

## Python imports and libraries

The checked-in dependency lists are authoritative: [`pyproject.toml`](../pyproject.toml) and
[`uv.lock`](../uv.lock) for the simulator, and [`mcp/pyproject.toml`](../mcp/pyproject.toml) for the host. The
installer handles the dependencies below; they should not be mixed into a system Python by hand.

| Component | Interpreter and imports | How supplied |
|---|---|---|
| Host MCP server | Python 3.10 or newer; `mcp`, `mss`, `PIL` (Pillow), `Xlib` (python-xlib), `imageio_ffmpeg` | `./install.sh` or `./install.sh --no-sim` installs the host package and its dependencies. Screen capture requires an accessible graphical display; window selection/raising uses X11/XWayland. |
| Host Isaac Lab metrics | `tensorboard` | `./install.sh` installs the host with the `isaaclab` extra. If installing the host separately, use the `[isaaclab]` extra for metrics. |
| Isaac Sim and work cell | Python 3.12; `isaacsim`, `omni`, `carb`, `pxr`, `numpy`, `scipy`, `yaml` (PyYAML), plus project dependencies including `matplotlib`, `torch` and `torchvision` | The root environment is synced by `./install.sh`; Sim supplies its Kit/USD modules. Many Kit imports are available only after Kit starts, not from an arbitrary Python interpreter. PyYAML reads an optional camera configuration. |
| Simulator renderer and camera runtime | `omni.hydratexture` and `omni.kit.hydra_texture` inside Kit | Bundled NVIDIA extension in the inspected Sim installs; enabled/resolved as part of the Kit runtime where needed. `.pyi` files are type descriptions, not a separate user installation. Do not install an unrelated `omni` package or copy vendor files. |
| Simulator ROS bridge | `rclpy`, `sensor_msgs`, `tf2_msgs` and default message types inside Kit | Supplied by the enabled Isaac Sim ROS 2 bridge; the repository launcher enables it. Custom ROS interfaces need a workspace built for Sim's Python 3.12; see [NVIDIA's ROS setup](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_ros.html). |
| Isaac Lab jobs | Isaac Lab 3.0, its Python environment, compatible Isaac Sim, task modules and the selected training library | Install Isaac Lab separately. An activated environment or a checkout-local `.venv` can be discovered; use `ISAACLAB_PYTHON` and `ISAACLAB_ROOT` to select another installation. See [Isaac Lab tools](TOOLS-ISAACLAB.md). |
| External ROS clients | System ROS `ros2` for RViz 2 and Nav2, and Cyclone DDS | Install the matching ROS distribution; see [ROS 2 and RViz 2](ROS2-RVIZ.md#installing-ros-2-for-rviz-2-and-the-doctor). |

For recordings, `install.sh` supplies an `ffmpeg` shim backed by the host's `imageio-ffmpeg` package, unless an
existing `~/.local/bin/ffmpeg` is present. A working system `ffmpeg` can also be used. A windowed workflow needs
an accessible display; host window and input controls were tested with X11/XWayland, not native Wayland.
A headless Sim can still render sensor output but has no visible window to click or capture. The screen tools and
the X11 mouse and keyboard tool are in the `computer` toolset, which the user selects at startup as described in
[host tools](TOOLS-HOST.md#optional-desktop-input).

If a Kit module or renderer extension appears missing, first check the selected Sim interpreter and the
running Kit extension manager/startup log. Do not try to fix a Kit import by installing `omni` or `.pyi` files
into the host Python. The supplied launcher requests `twinsmith.bridge`, `sim-tools`, `workcell` (the work cell) and
`isaacsim.ros2.bridge`. After connecting the correct bridge, `status` and
`sim__stage(action="info")` help verify the running process and build; listing catalog tools by itself does not.

## Optional Lab, desktop controls and project assets

**Isaac Lab** is a separate installation with its own version pairing and interpreter, and its tools are in the `sim` toolset,
which is off by default: add `+sim` ([toolsets](TOOLSETS.md#choosing-toolsets)). Set
`ISAACLAB_PYTHON=<workspace>/envs/lab-3.0/bin/python` and
`ISAACLAB_ROOT=<workspace>/IsaacLab` (example locations; use your own) in the host/client environment, then call
`isaaclab__inspect(action="install")` before launching jobs. Invalid explicit paths fail discovery; they do
not silently install Lab. The host's `isaaclab` extra supplies TensorBoard metrics support,
not the Lab runtime. For the inspected Lab 3 video workflow, MoviePy
`>=1.0.3,<2.0.0.dev0` and `imageio-ffmpeg` belong in the **Lab interpreter**; use that
Lab checkout's video extra in a dedicated compatible environment. Plain headless training
does not require those video packages. Earlier video checks used a private overlay. A later self-contained Lab 3 / Sim 6.2
environment exercised training and video without inherited `.pth` files, but retains two
package-metadata conflicts. See [recording a run](VERIFICATION.md#recording-a-run-so-it-can-be-reproduced)
and [Lab versions, discovery and video modes](TOOLS-ISAACLAB.md); neither is a clean-machine
or full-installer certification.
A successful job launch is not a completed job: inspect status and logs. Jobs normally refuse
alongside an active Kit bridge; coordinate GPU ownership instead of bypassing the guard by default.

**Screen capture, desktop input and managed sessions** are the explicit-only `computer` toolset. Set
`TWINSMITH_TOOLSETS=all,+computer` in the client server environment only
when they are needed; it cannot be enabled at run time. Desktop input acts on a shared X11 session; verify window/PID first.
Managed launches require resource checks and distinct endpoints. See [host tools](TOOLS-HOST.md).

**Your project assets** need an explicitly approved trusted folder and a host/Kit-accessible
staging/cache location. Approval can include a bounded project copy, not just one USD file.
Loading native assets is not sandboxed. Use `assets__import_path` for generic project import.
No private workstation or personal asset is required for the initial read-only connection check.
Follow [project setup and exact tool parameters](TOOLS-SCENE.md#project-usd-and-trusted-roots)
and [third-party notices](../THIRD-PARTY-NOTICES.md).

## One workspace folder

Keep the twinsmith checkout, your asset stores and your Isaac Sim / Isaac Lab Python environments together in one folder.
This section says how, and how the tool finds things that are already installed somewhere else. It documents no Windows or
macOS paths: a Windows install, for example one under `C:` and reached from WSL as `/mnt/c/...`, cannot be used.

### Why one folder

- **Reproducible.** One root holds everything the setup needs. You can write down what is
  in it and rebuild it.
- **Projects never edit the MCP checkout.** The checkout is code only. Your scenes, notes and
  asset changes live in `projects/` and the asset root, so updating or re-cloning the checkout
  cannot lose them.
- **Easy to copy.** Moving to a second machine means copying the asset root and running `doctor` on each pack
  (see [Move to a second machine](#move-to-a-second-machine)).

### Layout

```text
<workspace>/
  twinsmith/     the checkout (code only, never edited by projects)
  assets/        your asset root: packs as <pack id>/<version>/pack.yaml; never pushed
  envs/          Isaac Sim / Isaac Lab Python environments (e.g. sim-6.2/, lab-3.0/)
  projects/      your projects, each may hold a twinsmith.json
  tools/         other tools that run next to the simulator (e.g. RoboSimStudio)
  runs/          logs, job outputs, captures
```

`<workspace>` can be any folder you own. The default is `~/robotics-workspace`, or whatever
`TWINSMITH_WORKSPACE` says. It must not be inside the checkout; `init` refuses that.

### Agent rules for every project

`init` writes `<workspace>/AGENTS.md` and `<workspace>/CLAUDE.md` (which only contains `@AGENTS.md`). Claude Code, Codex
and OpenCode read parent-folder instruction files, so every project under `projects/` gets them. The text is the `AGENTS`
constant in `scripts/env.py`. In short: load the `mcp__twinsmith` tools, run `status` once before the first simulator action, use the
screenshot, recording, contact and Isaac Lab tools, and say why before falling back to plain scripts. A project's own
`AGENTS.md` can add to it. Run `init` again on an old workspace to add the files; nothing is overwritten.

To see at the start of every session whether a bridge is up, add a Claude Code hook to `~/.claude/settings.json`.
`scripts/session_check.py` prints one line and always exits 0:

```json
{"hooks": {"SessionStart": [{"hooks": [{"type": "command",
  "command": "python3 /path/to/twinsmith/scripts/session_check.py"}]}]}}
```

### Set up a new machine

1. **Create the folder and clone into it.**

   ```bash
   export TWINSMITH_WORKSPACE=~/robotics-workspace    # choose your own location
   mkdir -p "$TWINSMITH_WORKSPACE"
   git clone <repo-url> "$TWINSMITH_WORKSPACE/twinsmith"
   cd "$TWINSMITH_WORKSPACE/twinsmith"
   ```

2. **Create the sub-folders.** `init` never overwrites anything that exists. New folders are
   not group- or world-writable (0755 under the usual umask).

   ```bash
   python3 scripts/env.py init
   ```

   It creates `assets`, `envs`, `projects`, `runs`, `tools` and a short README under
   the workspace (`$TWINSMITH_WORKSPACE`, else `~/robotics-workspace`). Existing files are never
   overwritten. Use `--workspace DIR` to name the folder explicitly.

3. **Make the choice permanent.** Add the export to your shell profile (for example
   `~/.bashrc`) and to the environment of your MCP client config
   ([client configuration](MCP-SERVER.md#4-client-configuration)).

   ```bash
   echo 'export TWINSMITH_WORKSPACE=~/robotics-workspace' >> ~/.bashrc
   ```

4. **Install or point to Isaac Sim.** For a new pinned 6.2 install, follow
   [the pinned installation](#new-pinned-62-installation); the installer creates
   `<repo>/.venv`, which is found automatically. If you already have Isaac Sim, see
   [I already have Isaac Sim installed](#i-already-have-isaac-sim-installed).

5. **Add your assets and check.** The asset root is `TWINSMITH_ASSET_ROOT`, else `<workspace>/assets`. Fill it with `scripts/assets.py adopt` or by copying a pack folder
   in (see [ASSET-PACKS.md](ASSET-PACKS.md)).

   ```bash
   export TWINSMITH_ASSET_ROOT="$TWINSMITH_WORKSPACE/assets"
   python3 scripts/assets.py list
   python3 scripts/env.py where
   ./run.sh --port 8766   # not if a Kit already holds this port; choose another with --port
   ```

### I already have Isaac Sim installed

Do not reinstall. Ask the tool what it can find:

```bash
python3 scripts/env.py find          # human-readable list
python3 scripts/env.py find --json   # same, for scripts
```

`find` lists the Isaac Sim and Isaac Lab environments it discovers, in the search order
below. It only looks at files. It does **not** run any interpreter, so a listed entry is
a candidate, not proof that the environment works.

Pick one:

```bash
python3 scripts/env.py resolve-python                       # print the chosen interpreter
python3 scripts/env.py resolve-python --require-version 6.2 # only a matching version
python3 scripts/env.py resolve-python --lab                 # prefer an environment that also has Isaac Lab
```

Then confirm it before you rely on it:

```bash
PY="$(python3 scripts/env.py resolve-python)" &&
  "$PY" -c 'import importlib.metadata as m, sys; print(sys.version); print(m.version("isaacsim"))' &&
  uv pip check --python "$PY"
```

**Pin it.** Auto-detect is a convenience. To make the choice explicit, set
`TWINSMITH_SIM_PYTHON`. It wins over everything else:

```bash
export TWINSMITH_SIM_PYTHON=<env>/bin/python
./run.sh --port 8766
```

**Link it into the workspace.** If you want the environment to be listed with the rest,
symlink it. Nothing is moved or modified:

```bash
ln -s <env> "$TWINSMITH_WORKSPACE/envs/sim-6.2"
```

`<env>` is the directory that holds `bin/python`. Anything in `<workspace>/envs/*/bin/python`
is found.

`run.sh` follows the same rules. It prints which interpreter it chose and how to pin it.

### What is auto-detected, and where it looks

The first match in this order is used, even if a newer Isaac Sim exists further down the
list; the version only filters. `run.sh` asks for any Isaac Sim 6.x
(`resolve-python --require-version 6`). Pin a specific interpreter with `TWINSMITH_SIM_PYTHON`.

1. `TWINSMITH_SIM_PYTHON`.
2. `<workspace>/envs/*/bin/python`.
3. `<repo>/.venv/bin/python`.
4. The active environment: `$VIRTUAL_ENV` or `$CONDA_PREFIX`.
5. Sibling environments next to the checkout.
6. Conda-style roots and common virtual-environment folders under your home directory, then
   project virtual environments one and two folders below it (`~/<project>/.venv`,
   `~/<group>/<project>/.venv`; hidden folders are skipped, nothing deeper is scanned).
7. Standalone installs: `$ISAACSIM_PATH`, `$ISAAC_PATH`, `~/isaacsim`,
   `~/isaac-sim`, `~/.local/share/ov/pkg/isaac-sim-*`, `/opt/nvidia/isaac-sim*`,
   `/isaac-sim`.

The asset root is found in this order (see [ASSET-PACKS.md](ASSET-PACKS.md#the-asset-root-and-how-it-is-found)):
`TWINSMITH_ASSET_ROOT`, then `<workspace>/assets`, else an error
that names the setting.

### What is not auto-detected

- Any Windows install, including under `C:` or `/mnt/c`. Not supported, untested.
- macOS. Not supported.
- Environments outside the places listed above. Set `TWINSMITH_SIM_PYTHON`, or symlink the
  environment into `<workspace>/envs/`.
- A version that matches. `find` does not execute Python, so check the version yourself
  as shown above, or use `--require-version`.
- System ROS. Isaac Sim's own ROS 2 bridge needs none. External clients (RViz 2, Nav2) use a system or RoboStack ROS; see
  [ROS 2 and RViz 2](ROS2-RVIZ.md#installing-ros-2-for-rviz-2-and-the-doctor).
- Isaac Lab for the `isaaclab__*` job tools is selected with `ISAACLAB_PYTHON` and
  `ISAACLAB_ROOT`, see [TOOLS-ISAACLAB.md](TOOLS-ISAACLAB.md).

### Environment variables

Every variable, with its default and the process that reads it, is in [ENVIRONMENT.md](ENVIRONMENT.md). Set them in the shell
that starts `run.sh` and in your MCP client's environment; setting one only in the client does not change an already-running
Kit process.

### Move to a second machine

Paths differ between machines. The bytes must not.

1. On the second machine, do [the setup above](#set-up-a-new-machine) steps 1 to 4.
2. Copy the pack folders (each `<pack id>/<version>/` with its `pack.yaml`) to the asset root there.
   Keep each version whole.
3. Run on the second machine:

   ```bash
   python3 scripts/assets.py list
   python3 scripts/assets.py doctor <pack id>/<version>
   ```

   Every version must say `verified`: `pack.yaml` carries the hashes, so a file that differs fails. See
   [ASSET-PACKS.md](ASSET-PACKS.md#moving-packs-between-machines).
4. Environments are not copied. Install the same Sim version on the second machine and check
   it with the version one-liner above.

### Workspace troubleshooting

**`find` lists nothing.** Your install is outside the search locations. Set
`TWINSMITH_SIM_PYTHON`, or symlink it into `<workspace>/envs/`.

**`init` refuses the folder.** It is inside the checkout. Choose a folder outside it.

**`run.sh` picked the wrong interpreter.** Read its printed line, then pin the one you want
with `TWINSMITH_SIM_PYTHON`. `python3 scripts/env.py where` shows the effective workspace
and interpreter.

**Assets are not found.** The asset root must be the one the Kit process sees: check
`TWINSMITH_ASSET_ROOT` / `TWINSMITH_WORKSPACE` in the Kit process's environment. Run
`python3 scripts/assets.py where` to see the root and how it was found.

**Found but broken.** An interpreter listed by `find` can still be incompatible. Run the
version one-liner and `uv pip check` shown above.
