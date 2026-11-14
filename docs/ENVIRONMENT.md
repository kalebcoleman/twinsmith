# Environment variables

Every environment variable the code reads is listed here once, with its default, the process that reads it and what it
means. `mcp/tests/test_environment_docs.py` fails when the code reads a variable that this page does not list, or when this
page lists one that the code reads nowhere.

**Which process reads it matters.** A variable set only in your MCP client's entry reaches the host server and the Isaac Lab
jobs it starts, but never a Kit process that is already running. Set Kit variables in the shell that starts `run.sh`. Set
host variables in the client's `env` for the `twinsmith` server, then reconnect.

The processes named in the *Read by* column are:

| Name | What it is |
|---|---|
| host | The `twinsmith` server your MCP client starts, and the command-line client `mcp/twinsmith/client.py` |
| `run.sh` | The launcher that starts Isaac Sim (and `scripts/run_kit.py`, which it runs) |
| Kit | The Isaac Sim process: the bridge, `sim-tools` and the work-cell extension |
| Lab job | An Isaac Lab process the host starts |
| installer | `install.sh` and `scripts/env.py` |

Besides these variables, the server reads two settings files: `~/.config/twinsmith/config.toml` and the nearest
`twinsmith.toml` above the working directory. They hold `[tools] toolsets` and `[sim] physics_backend`; see
[toolsets](TOOLSETS.md#choosing-toolsets). Their precedence is defaults, user file, project file, environment, command line.
A project's `twinsmith.json` is a different file, read for asset packs and the Isaac Lab install.

## Reaching a bridge

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_PORT` | unset for the host, `8766` for `run.sh` | host, `run.sh`, Kit | Bridge port. The host dials it, so it pins one bridge; Kit listens on it (it is also a stored Kit setting, `autostartPort`); `run.sh` starts the bridge on it (`--port` overrides) and exports it; the work-cell extension reads it too. With no port set the host uses the one bridge advertised on the machine, and refuses to guess among several. A taken port refuses at Kit startup. The installer writes this one. |
| `TWINSMITH_HOST` | the discovered bridge's host | host | Bridge host to dial. |
| `TWINSMITH_TOKEN` | a random token per launch | Kit, host | The token every connection must present. Set it for Kit to choose it; set the same value for a client that cannot read the bridge's token file (another machine or account). See [protocol](../mcp/docs/protocol.md#the-token). |
| `TWINSMITH_BIND` | `localhost` | Kit | Bind address. Beyond loopback needs a real trust boundary, because the bridge serves `run_python`. Stored Kit setting: `autostartHost`. |
| `TWINSMITH_AUTH` | `1` | Kit | `0` starts a bridge that accepts connections without a token. Stored Kit setting: `auth`. |
| `TWINSMITH_READ_ONLY` | `0` | Kit | `1` starts a bridge that runs only read-only tools. Stored Kit setting: `readOnly`. See [limiting what a client can reach](MCP-SERVER.md#limiting-what-a-client-can-reach). |
| `TWINSMITH_TOOL_MODULES` | unset | Kit | Your own [tool packages](../mcp/docs/tool-authoring.md), as module names. Stored Kit setting: `toolModules`. |
| `TWINSMITH_TOOL_PATHS` | unset | Kit | The folders those packages are in. Stored Kit setting: `toolPaths`. |
| `XDG_RUNTIME_DIR` | `/tmp/twinsmith-<uid>` for the portfile and token | host, Kit | Where a running bridge writes its portfile and token file, and where the screen tools look for the desktop's runtime directory. |

`scripts/install.py` writes the stored Kit settings; the environment wins over them.

## The server: toolsets, identity and log

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_TOOLSETS` | `default` | host | Toolset profile and adjustments. Overrides the files; `--toolsets` overrides it. See [toolsets](TOOLSETS.md). |
| `TWINSMITH_RUNTIME_ENABLE` | `1` | host | `0` stops the `toolsets` tool from enabling or disabling toolsets while the server runs; `[tools] runtime_enable = false` does the same. See [toolsets](TOOLSETS.md#enabling-a-toolset-while-the-server-runs). |
| `TWINSMITH_CURSOR_BADGE` | `0` | host | `1` draws a small click-through badge that glides to where `computer` and `sim__ui__act` input lands, then hides; it is hidden from every screenshot but shows in monitor recordings. Overrides `[computer] cursor_badge = true` in the configuration files. |
| `TWINSMITH_PHYSICS_BACKEND` | `newton` | host, `run.sh`, Kit | Physics engine for a new simulator, `physx` or `newton`. It overrides `[sim] physics_backend` in the configuration files; `--physics-backend` overrides it. The managed launch tool, `run.sh` and `scripts/run_kit.py` all read the same setting. Never falls back to the other engine; the UR5e work cell needs `physx`. See [compatibility](COMPATIBILITY.md#target-engine). |
| `TWINSMITH_NAME` | `twinsmith` | host | Name the server reports to the client. |
| `TWINSMITH_INSTRUCTIONS` | built in | host | Replaces the instructions text the server sends to the client. |
| `TWINSMITH_LOG` | `1` | host | `0` turns off the durable log file; stderr logging stays. See [diagnosing disconnects](MCP-SERVER.md#diagnosing-disconnects-the-server-log). |
| `TWINSMITH_LOG_DIR` | `$XDG_STATE_HOME/twinsmith`, else `~/.local/state/twinsmith` | host | Directory of the `server-<pid>.log` files (1 MB x 3, private). |
| `XDG_STATE_HOME` | `~/.local/state` | host | Base of the default log directory. |
| `DISPLAY` | `:0` | host | X11 display for the screen and desktop-input tools. Match your session. |
| `TWINSMITH_KIKIT` | `<workspace>/tools/kikit/.venv/bin/kikit` | host | The KiKit command `hardware` runs for `fab`. See [HARDWARE.md](HARDWARE.md). |
| `TWINSMITH_BLENDER` | `blender` on `PATH` | host | The Blender `hardware` runs for `render`. See [HARDWARE.md](HARDWARE.md). |
| `TWINSMITH_ROBOSIM_ROOT` | `<workspace>/tools/robosimstudio` | host | The RoboSimStudio checkout the `robosim` tool runs. See [ROBOSIM.md](ROBOSIM.md). |
| `TWINSMITH_GAP_ROOT` | `<workspace>/tools/graph-as-policy` | host | The Graph-as-Policy checkout the `gap` tool runs; the skill library is the `open-robot-skills` folder beside it. See [PLANNING-EVAL.md](PLANNING-EVAL.md). |
| `TWINSMITH_VABAR_ROOT` | `<workspace>/tools/vabar` | host | The VABAR checkout the `vabar` tool runs. See [PLANNING-EVAL.md](PLANNING-EVAL.md). |
| `TWINSMITH_SYSID_ROOT` | `<workspace>/tools/sysid` | host | The folder whose `.venv` (MuJoCo 4.0.1 with its `sysid` extra) the `sysid` tool runs. See [SYSID.md](SYSID.md). |

Toolsets are visibility preferences, not security: the token and read-only mode are enforced by the bridge.

## Starting the simulator

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_SIM_PYTHON` | auto-detect | `run.sh`, host, installer | The Isaac Sim interpreter. `run.sh` and managed sessions start it; the installer writes it into the client entries it creates. It wins over `<repo>/.venv` and over auto-detection. |
| `ISAACSIM_PATH`, `ISAAC_PATH` | unset | installer, Kit | A standalone Isaac Sim install folder to search. Kit reads only `ISAAC_PATH`, to find the install's version files. |
| `TWINSMITH_HEADLESS` | `0` | Kit | `1` runs without a window. `run.sh --headless` sets it. |
| `TWINSMITH_NO_HOT_RELOAD` | unset | `run.sh` | `1` stops Kit reloading extensions when their files change (the same as `--no-hot-reload`). Use it for recorded runs or when another process edits the checkout. `status` reports `extension.hot_reload`. |
| `TWINSMITH_CRASH_UPLOAD` | unset | `run.sh`, host | `1` keeps Kit's default of uploading a crash's minidump to NVIDIA. Otherwise `run.sh` and the Isaac Lab jobs the host starts keep dumps on this machine, because a minidump can contain process memory. |
| `TWINSMITH_PRINT_PYTHON` | unset | `run.sh` | `1` prints the interpreter `run.sh` chose and exits. |
| `OMNI_KIT_ACCEPT_EULA` | unset | installer, `run.sh`, host | `YES` accepts NVIDIA's EULA. `run.sh` and the Lab jobs the host starts set it when they start Kit; the installer needs it or `--accept-eula` before it downloads a simulator. |
| `TWINSMITH_SCENE_OUTPUT` | a per-process temp directory | Kit, host | Output root for `sim__stage__save`, `sim__camera__capture` and `sim__occupancy_map`. **Set it to keep results across restarts.** A managed session sets it to its own folder. |
| `TWINSMITH_ENABLE_SCENE_SCRIPTS` | unset | Kit | `1` enables `sim__controller` and a Python (`module:function`) controller in `sim__simulation(action="run")`, which run arbitrary Python in Kit. A `tcp://127.0.0.1:<port>` controller needs no setting. |
| `ROS_DOMAIN_ID` | `0` | Kit, host, `scripts/ros_env.sh` | ROS domain. External ROS clients must match; a managed session gets its own. |
| `ROS_DISTRO` | `humble` on Ubuntu 22.04, else `jazzy` | `run.sh`, `scripts/ros_env.sh` | Which of Isaac Sim's bundled ROS 2 libraries `run.sh` puts on the library path. |
| `ISAAC_ROS_SEARCH` | `/opt/ros:$HOME/ros2` | `scripts/ros_env.sh` | Colon-separated roots where `scripts/ros_env.sh` looks for a ROS 2 install. |
| `TWINSMITH_EXTS` | `<repo>/exts` | the Isaac Lab bridge template | Extensions folder used by the [Isaac Lab bridge template](../mcp/templates/isaaclab_bridge_launch.py). |

## Managed simulator sessions (`computer` toolset)

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_SIM_ROOT` | the repository root when available | host | Checkout containing `run.sh`. Set it when the host is installed outside a checkout. |
| `TWINSMITH_SIM_SESSIONS_DIR` | `~/.local/share/twinsmith/sim-sessions` | host | Session metadata, logs and output. |
| `TWINSMITH_PORT_RANGE` | unset | host | Ports `simulators` may use, as `18800-18899`. An omitted port is chosen inside it; a stated port outside it is refused. |
| `TWINSMITH_ROS_DOMAIN_RANGE` | unset | host | ROS domains it may use, as `40-49`, with the same rule. |
| `TWINSMITH_SIM_MAX_INSTANCES` | `2` | host | Maximum known bridge, managed Sim and Lab-job processes. `0` disables the limit. |
| `TWINSMITH_SIM_MIN_FREE_VRAM_MIB` | `8192` | host | Minimum free GPU memory at launch. |
| `TWINSMITH_SIM_MIN_FREE_RAM_MIB` | `4096` | host | Minimum available system RAM at launch. |
| `TWINSMITH_SIM_LAUNCHER` | unset | host | An absolute path to a project's own launcher for `simulators(action="launch")`. Only from here, never from a tool argument or project file. The file must be executable and owned by you or root. |

## Isaac Lab

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `ISAACLAB_PYTHON` | discovered | host, installer | The Lab interpreter. An invalid explicit path fails discovery instead of falling back. Takes priority over `isaaclab_python` in a named project's `twinsmith.json`. |
| `ISAACLAB_ROOT` | discovered | host, installer | The Lab checkout. |
| `ISAACLAB_INSTALLS` | unset | host | `os.pathsep` list of extra Isaac Lab environments (or folders of them) that the `install` argument can select by name. |
| `TWINSMITH_JOBS_DIR` | `~/.local/share/twinsmith/jobs` | host, Kit | Job records, for Isaac Lab jobs and simulator jobs alike. |
| `ISAACLAB_LOGS_DIR` | unset | host | Extra run directory for checkpoints and metrics. |
| `ISAACLAB_CHECKPOINT_ROOTS` | unset | host | `os.pathsep` list of extra folders train, eval and play may load checkpoints from. |
| `ISAACLAB_ALLOW_TMP` | unset | host | `1` allows the temp directory for metrics. Tests only. |
| `ISAACLAB_GPU_BUDGET_HOURS` | unset | host | Optional GPU-hour cap for Lab jobs. Off by default. |
| `ISAACLAB_GPU_LEDGER` | unset | host | Optional JSON file for that cap. |
| `ISAACLAB_TASK_MODULES` | unset | host, Lab job | Modules to import so custom tasks register. |
| `ISAACLAB_TASK_PATH` | unset | host | Added to `PYTHONPATH` for those modules. |
| `ISAACLAB_EXTERNAL_CALLBACK` | unset | host | Default `module.function` for Lab jobs: `--external_callback=` for rsl_rl train and play, called by the server for other libraries, `env_check` and `record`. |
| `CUDA_VISIBLE_DEVICES` | unset | host, Lab job | The GPU a Lab job uses; the GPU-hour ledger and the free-memory check read the first id. |

Details are in the [Isaac Lab reference](TOOLS-ISAACLAB.md).

## Assets, projects and the workspace

These must be the same for the host and for Kit: each reads its own environment.

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_ASSET_ROOT` | `<workspace>/assets` | host, Kit | The one [asset root](ASSET-PACKS.md#the-asset-root-and-how-it-is-found): packs are `<pack id>/<version>/pack.yaml` under it. With nothing found, the error names this setting. |
| `TWINSMITH_ASSET_CONFIG` | unset | host, Kit | The [trusted-project approval file](TOOLS-SCENE.md#one-time-approval). No file means no approved roots. |
| `TWINSMITH_STAGING` | `~/.cache/twinsmith/staging` | host, Kit | With no approval file: the private folder the host copies import snapshots into and Kit reads them from. |
| `TWINSMITH_USD_PYTHON` | unset | host | A Python with `usd-core` for USD dependency checks on the host; without it they use the selected Kit. |
| `TWINSMITH_WORKSPACE` | the folder that holds the checkout (when it has `envs/` or an asset folder), else `<home>/robotics-workspace`; `scripts/env.py init` uses `<home>/robotics-workspace` | host, Kit, installer | The [workspace folder](LINUX-SETUP.md#one-workspace-folder); `<workspace>/assets` is where the asset root is looked for. |
| `TWINSMITH_PROJECT` | walk up from the working directory | host, Kit | A project directory or its `twinsmith.json`; its `"pack"` key (`"<id>/<version>"`) is the default pack for `doctor` and `pin`. Only a project named here can set `isaaclab_python` and `isaaclab_root`. |

## Set by the code, read by the code

Handshakes between the server's own processes. They are not meant to be set by hand.

| Variable | Default | Read by | Meaning |
|---|---|---|---|
| `TWINSMITH_DEFER_START` | unset | Kit | `run.sh` sets it so the bridge binds only after the work-cell tools register. |
| `TWINSMITH_EXTERNAL_CALLBACK` | unset | Lab job | The host passes a Lab job its callback (`module.function`) in this variable. |
| `TWINSMITH_TF_DOCTOR_REEXEC` | unset | `scripts/ros2_tf_doctor.py` | Stops the doctor re-running itself in a second environment more than once. |

A managed session's Kit also receives `TWINSMITH_SIM_SESSION_ID` and `TWINSMITH_SIM_SESSION_DIR` from the host; no code reads
them, they identify the process to a person.
