# Isaac Lab Host Tools (`isaaclab__*`)

This document covers the host-side tools registered by `mcp/twinsmith/isaaclab/tools.py`
(`install.py`, `jobs.py`, `metrics.py`), which belong to the `sim` toolset (off by default: add `+sim`, see
[toolsets](TOOLSETS.md)), with `run_script`. It holds the behaviour, contracts, guards and limits. Every tool and
argument, with its default, is in the generated [TOOLS-ARGUMENTS.md](TOOLS-ARGUMENTS.md#sim).

## Naming: host-side tools

- These are **host-side** MCP tools, not bridge-discovered Kit tools. They are registered
  directly on the FastMCP server in `mcp/twinsmith/server.py` and run in the `twinsmith`
  process. They spawn and manage Isaac Lab subprocesses; no Kit process is required for
  the host to expose them, and they do not call `run_python`.
- MCP tool names cannot contain dots, so they appear as `isaaclab__<tool>`
  (double underscore). There is no `isaaclab.` bridge namespace behind them.
- A client is shown grouped tools with an `action` argument; the [tool details](#tool-details) below are titled by the
  single-purpose name each action runs, which still works when called by name from a script. The listed forms are
  `isaaclab__launch` (`train`, `eval`, `play`, `record`, `env_check`), `isaaclab__inspect` (`install`, `envs`,
  `preflight`, `checkpoints`), `isaaclab__metrics` (`summary`, `curve`), `isaaclab__checkpoint_register`, and `jobs`
  (`list`, `lab_status`, `tail`, `status`, `cancel`) for job control. [Grouped tools](TOOLS-HOST.md#grouped-tools) explains
  the mapping.
- Every tool returns a JSON string. Failures come back as `{"error": ...}` (often with
  `hint`) rather than exceptions, so a client can read and retry. An argument of the wrong type or a missing
  one is refused earlier by the MCP layer, as an MCP error.

## Setup and configuration

### Locating an Isaac Lab install

`isaaclab__install_info` and the job launchers resolve an install in this order
(`mcp/twinsmith/isaaclab/install.py`):

1. `ISAACLAB_PYTHON` (interpreter path), with `ISAACLAB_ROOT` if provided; otherwise a
   sibling `IsaacLab` directory is tried. If the explicitly configured paths are invalid,
   discovery returns no install (it does not fall back).
2. `ISAACLAB_ROOT/.venv/bin/python`, or the active `VIRTUAL_ENV` when the checkout has no `.venv`.
3. An activated `VIRTUAL_ENV`, if it belongs to an Isaac Lab checkout or has a sibling
   `IsaacLab` checkout with the reinforcement-learning scripts.
4. The current directory's `.venv` when the current directory is an Isaac Lab checkout.

Between steps 2 and 3, a project named by `TWINSMITH_PROJECT` in the server's own environment (a directory or its
`twinsmith.json`) can set `isaaclab_python` and optionally `isaaclab_root`, for a project with its own Lab. Set that variable
in the `env` of the client's server entry. Relative paths start at that file's folder:

```json
{"isaaclab_python": ".venv/bin/python", "isaaclab_root": "../IsaacLab"}
```

A file found only by searching up from the working directory is ignored for these two keys, and `isaaclab__install_info`
reports `project_setting_ignored` with what to set. Lab tools run this interpreter, and starting a client inside a cloned
repository must not let that repository choose what runs. The same rule applies to a project file's `pack`, which
is not an approval: no tool trusts a path because a project file names it.

The interpreter must be a virtual environment with Isaac Lab installed, which is checked from its files before anything
runs it. An invalid setting is an error, not a fallback. That check catches mistakes; it does not stop a deliberately
faked environment, so name only a project you trust. A model with `put_file` can write any file there, and `put_file`
is in the same `sim` toolset as the Lab tools, so it cannot be hidden while they are listed. Explicit `ISAACLAB_PYTHON` or `ISAACLAB_ROOT` win, and then
`isaaclab__install_info` reports `project_setting_ignored`; remove them from that server's configuration to use the
project's. It reports `source`, and `tested: false` for a pair other than the two this project claims (Lab 3.0.0 with Sim 6.2, Lab 2.3.2 with Sim 5.1):
the version read works, but jobs on it are untested. A pip-only Lab has no `scripts/reinforcement_learning`, so
`isaaclab__train`, `eval` and `play` refuse with "Isaac Lab checkout not found" until `isaaclab_root` points at a checkout.

The resolver does not scan the home directory or assume a project-specific path. If no standard
environment or current checkout is available, set `ISAACLAB_PYTHON` and, when needed,
`ISAACLAB_ROOT`, or set `ISAACLAB_ROOT` for a checkout with its own `.venv`.

Automatic environment discovery requires local Isaac Lab package metadata; it does not run an
interpreter to decide whether to select one. `isaaclab__install_info` then probes package metadata
(`isaaclab`, `isaacsim`) in a short-lived subprocess. It does not search `PATH` or
`ISAACLAB_PATH`, and does not validate every supported combination. Set explicit paths for
other layouts. `TWINSMITH_JOBS_DIR` overrides the job state directory (default
`~/.local/share/twinsmith/jobs`).

For example, substitute the paths for your own installation before starting the host server:

```bash
export ISAACLAB_PYTHON=/path/to/lab-env/bin/python
export ISAACLAB_ROOT=/path/to/IsaacLab
./install.sh --no-sim --claude --isaaclab-python "$ISAACLAB_PYTHON" --isaaclab-root "$ISAACLAB_ROOT"
```

The installation must already exist; `--no-sim` does not install Isaac Lab. Restart a client whose configuration was
changed, then call `isaaclab__install_info` and `isaaclab__envs_list` to check discovery.

### Task discovery

`isaaclab__envs_list` imports `isaaclab_tasks`, plus the comma-separated modules in
`ISAACLAB_TASK_MODULES` (with `ISAACLAB_TASK_PATH` added to `PYTHONPATH`), then reads
`gymnasium.registry`. Import failures are returned in `import_errors`. Custom task modules must
be explicitly configured; sibling packages are not imported automatically. Jobs use the
same resolver (`ISAACLAB_TASK_MODULES` / `ISAACLAB_TASK_PATH` are forwarded to the child).

#### Choosing the Isaac Lab / Sim install per call

The server's `ISAACLAB_PYTHON` / `ISAACLAB_ROOT` (Isaac Lab 3.0.0 with Sim 6.2 on this setup) stay the **default** and are
never changed. Every launching tool (`train`, `eval`, `play`, `env_check`, `record`, `run_script`, `envs_list`,
`install_info`) also takes an optional `install`, so one server can run Lab 2.3.2 with Sim 5.1 for one call and 3.0.0 for
the next, with no restart:

| `install` value | Meaning |
|---|---|
| omitted or `default` | The configured default. If the server was started with `TWINSMITH_PROJECT` naming a project whose `twinsmith.json` has `"isaaclab_install": "<selector>"`, that selector is used instead (an explicit `default` opts out). A project file found only by walking up from the working directory is never honored. |
| a name or version | A *known install*: `lab-2.3.2`, `2.3`, `2.3.2`, `3.0.0`. Names are virtual-environment folder names; versions are read from the folder name or the installed `isaaclab` distribution. Several matches is an error that lists them. |
| an absolute path | A virtual environment folder, its `bin/python`, or a Lab checkout holding a `.venv`. It must be a virtual environment with Isaac Lab installed (checked from its files before anything runs). |

Known installs are read from files, nothing is run: the configured default, its sibling environments, `<TWINSMITH_WORKSPACE>/envs/*`,
`ISAACLAB_INSTALLS` (`os.pathsep` list of venvs, interpreters, checkouts or folders of venvs) and what
`scripts/env.py find` reports when the script sits beside the package. `isaaclab__install_info` returns them as
`known_installs`. The checkout is the install's own (`<env>/IsaacLab`, a checkout holding the `.venv`, or `<env>-src` beside it),
so Lab 2.3's per-library scripts and `rl_library` rules apply as for any 2.3 call. A successful launch returns `install`
(`python`, `isaaclab_root`, `source`) so a wrong pick is visible. `metrics_summary`, `checkpoints_list` and `tensorboard_tail`
accept the logs of every known install. Mixing is the caller's job: a checkpoint trained on Lab 2.3 is for Lab 2.3.

#### Tasks from an external project

Tasks of a project that is not installed in the Lab environment register only when its package is importable
and its registration code runs. Every launching tool (`isaaclab__envs_list`, `train`, `eval`, `play`,
`env_check`, `record`) takes these optional arguments for this, per call (`envs_list` has no `cwd`):

| Argument | Effect |
|---|---|
| `python_paths` | List of existing absolute directories added to the front of the child's `PYTHONPATH`, before the install's own path and `ISAACLAB_TASK_PATH`, never replacing them. The documented way to add import roots. `python_path` is the older name; both may be given and are merged. Also on `run_script`. |
| `env` | Extra environment variables for the child: identifier names, string values. **One policy for every Isaac Lab tool, `run_script` included** (`install.validate_env`). `PYTHONPATH`, `PATH` and `LD_LIBRARY_PATH` are *prepended* to the install's own value, not replacing it; `OMNI_KIT_ACCEPT_EULA`, `CUDA_VISIBLE_DEVICES`, `HEADLESS`, `HOME`, `TMPDIR` and other `PYTHON*` variables are accepted (a tool's own `device`/`headless` argument still wins for the middle two). Refused, because they would replace or hijack the install, the job's supervising shell or this server, each with the argument to use instead: `PYTHONHOME`, `ISAACLAB_PYTHON`, `ISAACLAB_ROOT`, `ISAACSIM_PATH`, `TWINSMITH_SIM_PYTHON` (use `install`), `LD_PRELOAD`, `LD_AUDIT`, `DYLD_INSERT_LIBRARIES` (use a wrapper script run by `run_script`), `BASH_ENV`, `ENV`, `IFS`, `CDPATH`, `BASH_FUNC_*`, `TWINSMITH_*` and `TWINSMITH_JOBS_DIR` (server settings). Values are not stored in the job record, only the names. |
| `external_callback` | Dotted `module.function` (strict `^ident(\.ident)+$`). Works with every `rl_library`, `env_check` and `record`: with `rsl_rl` train/play (and eval) it is passed to the Isaac Lab script as `--external_callback=module.function`; everywhere else this server calls the function itself, with no arguments, after the task modules are imported and before the script runs (`launcher.py`, `eval_runner.py`, `check_runner.py`; the job gets `TWINSMITH_EXTERNAL_CALLBACK`). Giving it also in `extra_args` is refused. `envs_list` has no Lab script, so it imports the module and calls the function with no arguments. |
| `task_modules` | List of dotted module names imported before the task is looked up, for a project whose tasks register on import, e.g. `["my_project.tasks"]`. Same effect as `ISAACLAB_TASK_MODULES` in `env`, which it is merged with. |
| `cwd` | Working directory of the job (`train`, `eval`, `play`, `env_check`, `record`), for projects whose configs use relative data paths. Must be an existing folder, validated as for `run_script`. Default: the Isaac Lab checkout. |

Persistent equivalents, set in the MCP server's environment: `ISAACLAB_TASK_PATH` (same as `python_paths`),
`ISAACLAB_TASK_MODULES` (modules imported first) and `ISAACLAB_EXTERNAL_CALLBACK` (default for
`external_callback`). When any of these is supplied, the pre-launch check that refuses a task id the checkout does
not register is skipped, since the project may register it. Isaac Lab 3.0.0 runs `--external_callback` only in the
`rsl_rl` train and play entry points (eval is a play job), so for every other `rl_library` (rl_games, skrl, sb3) and for
`env_check` and `record` this server calls the callback itself; the `ISAACLAB_EXTERNAL_CALLBACK` default behaves the same.
A callback that fails ends the job with its traceback in `runs_tail(stream="stderr")`.

Example: `isaaclab__envs_list {"python_paths": ["<project>/src"], "external_callback": "my_project.register.register", "filter": "my_task"}`,
then `isaaclab__train` with the same three arguments.

When `isaaclab__envs_list` returns no tasks, or none match `filter`, the result carries a `hint` explaining this.

### TensorBoard readout

`isaaclab__metrics_summary` and `isaaclab__tensorboard_tail` read `events.out.tfevents.*`
files and require the `tensorboard` Python package. Install the extra:
from the repo root use `uv sync --project mcp --locked --extra isaaclab --no-dev`; `install.sh` includes this extra. Without it these tools
return `{"error": "install the 'isaaclab' extra: pip install 'twinsmith[isaaclab]'"}`.

### Progress marker

Any job (train, eval, script, ...) may print `TWINSMITH_PROGRESS: <value> [text]` on stdout, where `<value>` is a fraction
from 0 to 1 (`0.4`), a percentage (`40%`) or `done/total` (`3/10`). `isaaclab__runs_status` then returns
`progress: {fraction, percent, done?, total?, text?}` from the newest such line in the last 256 KiB of the log; a
malformed newest line is ignored. A job that prints no marker still gets `progress` when its training library prints
its own count: `Learning iteration 12/1500` (rsl_rl), `epoch: 12/1500` (rl_games) or a progress bar (`12/1500 [...it/s]`,
skrl); its `text` says it came from the training log. Otherwise there is no `progress` field.

### Health of a running job

A crashed Kit can stay alive after its traceback, and a hung one prints nothing: both read `running` with a live pid.
So a running job's status also has `last_output_age_s` (seconds since it last wrote to either log) and `health`:

| `health` | Meaning |
|---|---|
| `ok` | The process is alive and wrote output recently. |
| `crashed` | `stderr.log` ends in an uncaught Python exception and nothing was written to `stdout.log` after it, but the process is still alive: it is probably hanging in shutdown. `health_note` quotes the exception; read `isaaclab__runs_tail` with `stream="stderr"`, then cancel. |
| `stalled` | No output for more than 10 minutes. A long step or a quiet evaluation can do that; cancel it if `progress` and the log stay unchanged. |
| `exited` | The process is gone; the next status call records the final state and exit code. |

These are read from the logs and the process table, so they are hints, not proof: a job that prints nothing for a
long time by design reads `stalled`.

### Checkpoint roots for train, eval and play

A `checkpoint` must exist and, after resolving symlinks, lie under one of: the Isaac Lab checkout, the jobs folder, any
`python_paths` entry or the `cwd` passed to the same call (so a project's own `checkpoints/` folder works when its source
folder is on `python_paths`), or a folder listed in `ISAACLAB_CHECKPOINT_ROOTS` (`os.pathsep` list, for scratch or release
folders). A refusal names every allowed root.

### Log and checkpoint roots

`isaaclab__checkpoints_list`, `isaaclab__metrics_summary` and
`isaaclab__tensorboard_tail` only operate under allowed roots:

- `ISAACLAB_LOGS_DIR`, when set;
- `TWINSMITH_JOBS_DIR` (or `~/.local/share/twinsmith/jobs`), and the default jobs directory;
- `<isaaclab_root>/logs` and `<parent_of_isaaclab_root>/logs` (from the resolved install);
- the temp directory only when `ISAACLAB_ALLOW_TMP=1` (test fixtures).

Without `run_dir`, checkpoint discovery searches `ISAACLAB_LOGS_DIR` when set; otherwise it
searches `<isaaclab_root>/logs` (if an install root is resolved) and the jobs directory.
No other log directory is searched or allowed by default; set `ISAACLAB_LOGS_DIR` to add one. Paths outside the allowed roots are rejected with
`{"error": "path outside allowed roots", "allowed_roots": [...], "hint": ...}`; the hint says to use `run_dir` from `isaaclab__runs_status` or set `ISAACLAB_LOGS_DIR`.

### GPU resources and authorization

Training, eval and play jobs launch Isaac Lab's own RL stack. The tested configurations use a GPU; resource needs depend on the task and backend. These can be
long-running processes and must be started only with the user's explicit
authorization and with GPU capacity available. On a single GPU, a running Kit app and a
training job can contend for GPU memory. This pack does not evict or stop a sim; it only
refuses to start (see [Concurrency guards](#concurrency-guards)) unless the caller passes the
override flags. `headless=true` is the default for `train`/`eval` to avoid a second
renderer for ordinary jobs; adding `--video` requires rendering even headlessly.
`play` defaults to `headless=false` and opens a window. No actual training run is
required or performed by this documentation.

On a 16 GB RTX 5080, one short headless Cartpole training job for Lab 2.3.2 / Sim 5.1 and one for
Lab 3.0 / Sim 6.2 ran concurrently with PhysX, 16 environments and one iteration each; both exited
successfully and wrote checkpoints. Their Lab roots and output directories were separate. The second
job used `allow_concurrent=true` against the shared job-state directory. This is a small smoke test,
not a memory reservation or a guarantee for longer training, larger environments, rendering or a
simultaneously running Kit bridge. Keep the default guards unless you have checked the actual workload.

### Environment variables

Every `ISAACLAB_*` variable, with its default and the process that reads it, is in [ENVIRONMENT.md](ENVIRONMENT.md#isaac-lab).
The Kit-bridge guard probes the fixed bridge port (`TWINSMITH_PORT`, default 8766), and a job's `device`
argument sets `CUDA_VISIBLE_DEVICES` for it.

## What a checkout offers: the profile

`isaaclab__install_info` returns `profile`, read from the checkout's script files once per root, without importing Isaac Lab
or starting Kit: `lab_version`, `isaaclab_version`, `isaacsim_version`, `layout`, `libraries` and, per script, its declared
`flags` (including a helper beside it and AppLauncher's when that source is present). `flags: null` means unknown, not none.
Isaac Lab 3's scripts hand over to a CLI elsewhere, so theirs are unknown. `tested` is a separate statement: whether that Lab and Sim pair
was verified with real jobs; the tested pairs are Lab 3.0.0 on Sim 6.2 and Lab 2.3.2 on Sim 5.1.

| `layout` | Scripts | Library | Display |
|---|---|---|---|
| `unified` | `scripts/reinforcement_learning/{train,play}.py` | `--rl_library` | `--viz none\|kit` |
| `per_library` | `scripts/reinforcement_learning/<library>/{train,play}.py` | the folder; `rl_library` is required, and the refusal lists `libraries` | `--headless` when headless and without `--video` |
| `none` | not found: jobs are refused before starting, naming the expected paths | | |

`train`, `eval` and `play` build their command from this. A flag a script does not declare, generated or in `extra_args`, is refused
before anything starts, naming the flag and the alternative; otherwise a per-library script would pass it to Hydra after Kit started. With unknown flags nothing is refused.
`HEADLESS` is always set in the child.

**Isaac Lab 2.3.2 with Sim 5.1 is one of the two claimed pairs, and `tested` is true.** `Isaac-Cartpole-v0` went through train, resume, compare, eval and play with `rsl_rl` through these tools; tiny jobs ran with `skrl`, `sb3` and `rl_games` too, including checkpoints, metrics, eval and play, and the profile's flags matched each script's real `--help`. That shows the tools work on this pair, not long training or other tasks. Pass `rl_library`: its scripts are per library.
`rl_games` needs `minibatch_size` to divide `num_envs` times the task's `horizon_length` from its rl_games config, which varies by task (24 for `Isaac-Reach-UR10`; not the log's `seq_length`). A failure is a bare `AssertionError` on `batch_size % minibatch_size`. Measured on `Isaac-Reach-UR10`: `minibatch_size=64` or `192` at 16 environments, and `1536` at 64. `sb3` saves `model.zip`, which counts as a checkpoint only under an `sb3` path. The task's default can be larger than a small run's whole batch (24576 for `Isaac-Reach-Franka-v0`, against 64 environments times 24 steps), so pass the override. `rl_games` also saves a checkpoint only every `save_frequency` epochs (100 by default) and `save_best_after` (200), so a short run leaves nothing to evaluate unless you pass `agent.params.config.save_frequency=20` and `agent.params.config.save_best_after=10`; `rsl_rl` always saves at the end.

`isaaclab__envs_list` can't list tasks on this pair: importing `isaaclab_tasks` needs `pxr`, which exists only once Kit has started. It
reports `isaac_tasks_unavailable` and lists the ids it can read from the checkout's source, marked `source: "scanned"`. `train` and `play` work with those.

A job records its own exit code in its folder, so one that finishes after the host that started it has gone still reports `succeeded` or `failed` correctly. `failed` with `note: "host restarted"` now means it left no code: it was killed, or predates this. `timeout_s` is enforced by the job itself, in the shell that starts it, so it holds with no host running; `runs_status` echoes it as `timeout_s` when one was set, and a job stopped by it says so.

**A crash can exit 0.** Kit's shutdown can hide an uncaught exception: seen on Lab 2.3.2 with an unknown task. A job that exits 0 with a traceback at the end of stderr reports `failed` with a note.
One exception: a `train` job whose run folder already has a checkpoint still reports `succeeded`, so check its log and iteration count if it stopped early.

When `isaaclab_tasks` cannot be imported, `isaaclab__envs_list` adds `isaac_tasks_unavailable` and leaves out gymnasium's own tasks, such as classic-control `CartPole`.

## Parameter conventions

- All tools return JSON strings; the tables below list the parsed object shape.
- Required parameters are marked **required**. Optional parameters with `null` defaults can be omitted.
- `task` must match `^[A-Za-z0-9_.:-]+$`. `rl_library` must match `^[a-z0-9_]+$`.
- `extra_args` accepts syntactically validated `--flag`/`--flag=value` arguments and Hydra overrides
  such as `key.sub=value`, `+key=value`, or `~key`. It is not a semantic allowlist.
- Checkpoint paths are resolved and must exist and lie under the Isaac Lab root, the jobs
  directory, a `python_paths`/`cwd` folder of the same call or `ISAACLAB_CHECKPOINT_ROOTS`. `run_dir` for metrics must lie under the allowed roots above.
- Job ids match `^\d{8}-\d{6}-[A-Za-z0-9_.-]+$` (timestamp + task slug, with `-2`, `-3`
  suffixes if two launches land in the same second).
- Durations are in seconds. `num_envs` defaults to 64 for train, 1 for play.

## Tool details

### `isaaclab__install_info`

- Parameters: `install` (optional), see [choosing the install](#choosing-the-isaac-lab--sim-install-per-call).
- Returns: `{known_installs, python, isaaclab_version, isaacsim_version, isaaclab_root, venv, source, rl_libraries, tested, profile}` on success, with a `note` if this pair isn't tested;
  `{"error": "Isaac Lab install not found", "hint": "set ISAACLAB_PYTHON or ISAACLAB_ROOT"}` (or a probe error)
  on failure. A successful probe can still return null version fields if distributions are absent;
  it does not establish task import, renderer or training readiness.
- Example call shape: `{"tool": "isaaclab__install_info", "arguments": {}}`

### `isaaclab__envs_list`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `filter` | string | no | `null` | Case-insensitive substring matched against the task id. |
  | `python_paths` | list of strings | no | `null` | Directories added to the front of `PYTHONPATH` (`python_path` is the older name); see [external projects](#tasks-from-an-external-project). |
  | `env` | object | no | `null` | Extra environment variables for the probe. |
  | `external_callback` | string | no | `null` | `module.function` imported and called before the registry is read. |
  | `task_modules` | list of strings | no | `null` | Modules imported before the registry is read. |
- Returns: `{envs: [{id, entry_point}], import_errors?, applied?, hint?}`. `import_errors` appears only if a
  configured task module or callback failed. `applied` echoes the supplied `python_paths`/callback. `hint` appears
  when `envs` is empty (nothing registered, or `filter` matched nothing) and explains how external tasks register.
  Short-lived subprocess with a 60 s timeout.
- Example: `{"filter": "reach"}`

### `isaaclab__train`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `task` | string | yes | — | Task id (validated). |
  | `device` | string | no | `null` | Sets `CUDA_VISIBLE_DEVICES` when not `cpu`. |
  | `headless` | boolean | no | `true` | No window. Normally `--viz none`; `--video` selects Kit with child-only `HEADLESS=1`. `false` selects Kit with `HEADLESS=0`. |
  | `checkpoint` | string | no | `null` | Path to resume from (must be under the [checkpoint roots](#checkpoint-roots-for-train-eval-and-play)). |
  | `rl_library` | string | no | `null` | RL backend, e.g. `rl_games`, `rsl_rl`, `skrl`. Before launching, the tool runs `isaaclab__install_info`'s `rl_libraries` check in the same interpreter and refuses a library that is not importable there, naming the installed ones. If that check cannot run, the launch is not blocked. |
  | `physics` | string | no | `null` | `physx` or `newton`. Default: the task's own, often Newton on Lab 3. On Lab 3 it selects `physics=isaacsim_physx` or `physics=newton_mjwarp`; Lab 3's own `physics=physx` picks the optional OvPhysX runtime, which fails if it is not installed. Lab 2.x is PhysX only. A task with no engine presets fails with `did not match a selectable preset`: leave it out. |
  | `extra_args` | array[string] | no | `null` | Validated flags/Hydra overrides. |
  | `timeout_s` | number | no | `null` | The job's own watchdog enforces this limit, even after the host exits; `runs_status` echoes it. |
  | `allow_concurrent` | boolean | no | `false` | Override the job-vs-job guard. |
  | `allow_concurrent_gpu` | boolean | no | `false` | Override the Kit-bridge (GPU) guard. |
  | `label` | string | no | `null` | Short name shown in `runs_list`/`runs_status` (`isaaclab__eval` and `isaaclab__play` take it too, as does `run_script`). |
- Returns on success: `{job_id, log_dir, command}`. A guard refusal includes `error` and `hint`, along with `running_job_id`/`running_pid` (job guard) or `kit_bridges` (Kit guard). On no install: `{"error": "no Isaac Lab install found", "hint": ...}`.
- Example: `{"task": "Isaac-Reach-UR10", "num_envs": 64, "max_iterations": 2, "rl_library": "rl_games", "headless": true}`

### `isaaclab__eval`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `checkpoint` | string | yes | — | Required; validated path. |
  | `physics` | string | no | `null` | As for `isaaclab__train`. |
  | `num_episodes` | integer | no | `20` | Finished episodes to collect. Must be at least 1. |
  | `timeout_s` | number | no | `null` | Host watchdog, separate from the step bound. |
- Returns on launch: same shape as `isaaclab__train`. Runs `play.py` with `--checkpoint` (Isaac Lab has no separate
  eval entrypoint) and `--viz none|kit`, through a scoring launcher (see below). Once it finishes,
  `isaaclab__runs_status` also returns `eval_result`.
- Example: `{"task": "Isaac-Reach-UR10", "checkpoint": "/abs/path/to/model.pth", "rl_library": "rl_games", "headless": true, "num_episodes": 20}`

#### How eval produces a score

The job runs `mcp/twinsmith/isaaclab/eval_runner.py` with the Isaac Lab interpreter instead of `play.py`
directly. The runner imports the configured task modules, patches `gymnasium.make` so the environment it
returns has its `step` wrapped, and then runs the real `play.py` unchanged through `runpy` with the same
arguments. Isaac Lab itself is not modified, and `isaaclab__play` still runs the plain script.

- The wrapper sits on the object `gymnasium.make` returned, which is what the libraries' vector-environment
  wrappers (`RslRlVecEnvWrapper`, `RlGamesVecEnvWrapper`, `SkrlVecEnvWrapper`, `Sb3VecEnvWrapper`) call in
  their own `step`. It therefore sees exactly one call per simulator step, with the reward and
  terminated/truncated flags the environment produced. Works with torch tensors on any device, numpy or
  plain Python values.
- Each environment's reward is summed until that environment is terminated or truncated; that episode's
  undiscounted return and length (environment steps) are then recorded. Only complete episodes count.
- The run stops when at least `num_episodes` have finished **and** every parallel environment has finished
  at least one episode, or when `max_steps` environment steps have been taken. `max_steps` is the bound
  that makes a run end even when a task never terminates. `stopped_by` says which happened. Waiting for
  every environment's first finished episode avoids scoring only the fastest environments; a run cut short
  by `max_steps` says so in `note`.
- The runner then makes the simulation app's `is_running()` return false, so `play.py` leaves its loop and
  closes the environment and app itself; the job exits 0. If a play loop ignored that, the second step
  after the target raises a private exception instead of adding made-up episodes.
- `eval_result.json` is written atomically in the job folder and is what `runs_status` reports once it is
  there. Its fields are `episodes`, `mean_return`, `std_return`, `min_return`, `max_return`, `mean_length`,
  `steps`, `success_rate` (only when the environment reports success), `stopped_by`, `num_envs`,
  `num_episodes_requested`, `max_steps` and `note`. `steps` counts environment steps (one per simulator
  step across all parallel environments), not summed per-environment steps.
- If no episode finishes, `episodes` is `0` and the return fields are `null`, with a `note`; no score is
  estimated. A play script that never builds its environment through `gymnasium.make`, or never steps it,
  likewise produces no score: the runner reports that rather than inventing one. A multi-agent environment
  whose base `step` returns a per-agent reward dict is also reported as unscoreable. An uncaught error is
  written into the result and still fails the job. A job killed by `timeout_s` or `jobs action=cancel` may leave
  no result at all.
- A training log's mean reward is a different measurement: it usually covers a recent window of rollouts,
  often with stochastic actions, while eval runs the saved policy deterministically. Expect a gap in either
  direction; use `eval_result` when the question is how good the saved policy is.

Measured on Isaac Lab 2.3.2 with Isaac Sim 5.1 and `Isaac-Cartpole-v0` for `rsl_rl`, `rl_games`, `skrl` and `sb3`:
eval returned scores for early and trained checkpoints, and the trained checkpoint scored clearly higher.
Isaac Lab 3's unified `play.py` was not run here, so whether its libraries are wrapped the same way is not
verified; an unscoreable script shows up as `episodes: 0` / no score, not a number.

### `isaaclab__env_check`, `isaaclab__record`, `isaaclab__preflight`

`isaaclab__env_check(task, num_envs=4, steps=60, agent="zero")` starts a job that runs Lab's own
`scripts/environments/zero_agent.py` (or `random_agent.py`) through `check_runner.py`. The runner watches each
`env.step` and stops the loop after `steps`. `isaaclab__runs_status` then returns `env_check_result`: observation
shapes, action shape, reward mean/min/max, non-finite observation and reward counts, episode ends (and how many in
the first five steps), mean step time, `gpu_mib` for the process, and a `problems` list with `ok`. It checks any task
the same way; checks that depend on the task (a tip height, a tower standing) still need the task's own script.

`isaaclab__record(task, checkpoint=None, video_length=200)` records an MP4 with no window. With a checkpoint it runs
`play` with `--video`. Without one it runs zero or random actions through the same runner with gymnasium's
`RecordVideo` and `--enable_cameras`; `runs_status` lists `video_paths`.

`isaaclab__preflight(task=None, ledger=False, label=None, since=None)` reads `nvidia-smi`, the jobs this host started and any Kit bridge, and returns
`can_start` with reasons. `gpu.processes` lists every GPU compute process (`pid`, `name`, `mib`), and
`other_gpu_processes` leaves out those in a running job's process group, so other projects' work shows up. With a task that has a finished `env_check` it adds `suggested_num_envs`, treating half of
the measured GPU memory as fixed cost; the figure is deliberately low and should be confirmed with a short run.

#### Optional: GPU-hours used

`ledger=true` (or `label`, `since`) adds `gpu_hours`, computed from the start and end times in the job records: `used_hours` is the
wall-clock *union* of the jobs' intervals on each GPU, so jobs sharing a GPU are counted once; `naive_sum_hours` is the plain sum of
wall times, `counted` states the rule, and each entry of `jobs` has `wall_s`, `shared_s` and `attributed_hours`. `label` lists the jobs
whose label or id contains the text. Nothing is configured and nothing is refused by default.

An optional cap exists for those who want one: `ISAACLAB_GPU_BUDGET_HOURS`, or a JSON file `ISAACLAB_GPU_LEDGER` (default
`<jobs dir>/gpu_ledger.json`) with `budget_hours`, `since` and `carried_hours`. With a cap, `gpu_hours` shows `remaining_hours`, and
the job tools refuse to start once it is used up. A malformed setting is an error, never a silently absent cap.

### `run_script`

Runs **your own Python script** as a job of kind `script`, for work the registered-task tools cannot express
(a sweep, a standalone Kit script, a custom evaluation). **It executes arbitrary code from the script with your
rights on this host; nothing is sandboxed.** The tool is annotated not read-only and not idempotent, like the other
job tools. `script` must be an existing `.py` file; it is run as `<python> <script> <args...>` with `shell=False`,
never imported.

| Name | Default | Notes |
|---|---|---|
| `script` | required | Existing `.py` file. |
| `args` | none | List of strings. |
| `env` | none | Extra variables, under the same rules as the `env` argument of `isaaclab__train` (see [external projects](#tasks-from-an-external-project)). |
| `python_paths` | none | Folders put in front of `PYTHONPATH` (`python_path` is the older name). A `PYTHONPATH` in `env` is added the same way. |
| `python` | configured `ISAACLAB_PYTHON` | Must be an executable file. |
| `cwd` | the script's folder | Must exist. |
| `label` | script name | Used in the `job_id` and `runs_list`. |
| `timeout_s` | `600` | Hard wall limit. At expiry the whole process group gets SIGTERM, then SIGKILL; a shell guard enforces it even if the host goes away. |
| `min_free_vram_mb` | off | Refuse unless this much GPU memory (MiB) is free on GPU 0; refuses when `nvidia-smi` gives no answer, or when `CUDA_VISIBLE_DEVICES` points the script at another GPU. |
| `wait_for_vram_s` | `0` | Needs `min_free_vram_mb`: poll up to this long (at most 3600) before refusing. A cancelled call stops waiting and starts nothing. |
| `single_instance_key` | none | Refuse while a running job has the same key, so a double-queued sweep cannot run twice. |
| `allow_concurrent` | `false` | As for `isaaclab__train`: start although another job runs. |
| `allow_concurrent_gpu` | `false` | As for `isaaclab__train`: refuse while a Kit bridge runs unless `true`. |

The script leads its own session and process group, so `jobs(action="cancel")` and the timeout kill Kit, ffmpeg
and any other child it started. Output goes to the job's `stdout.log` and `stderr.log`; follow it with
`isaaclab__runs_tail`. This is the supported route for a project's own evaluator (`isaaclab__eval` only scores Isaac
Lab's `play.py` loop). A line `TWINSMITH_PROGRESS: <fraction | percent% | done/total> [text]` makes `isaaclab__runs_status`
report `progress` (see [progress](#progress-marker)). A script job's `run_dir` is the job folder, so `metrics_summary` and
`checkpoints_list` need the real run folder passed explicitly. A line `TWINSMITH_ARTIFACT: <path>` on stdout names an output file. When the job ends,
`isaaclab__runs_status` returns `script_result`: `status`, `exit_code`, `wall_time_s`, `log_path`,
`stderr_log_path` and `artifacts` (the printed paths, in order; they are not checked to exist).

#### General scripts

The runner under a name that does not say Isaac Lab, for a script that needs neither Isaac Lab nor a
simulator: a controller server for `sim__simulation__run`, a data conversion, a plot, one step of a sweep. It
starts the same kind of job (`script`) in the same job folder, so `isaaclab__runs_status` (with `wait_s`),
`isaaclab__runs_tail`, `isaaclab__runs_list` and `jobs(action="cancel")` follow and stop it, and the
`TWINSMITH_PROGRESS` and `TWINSMITH_ARTIFACT` lines work the same way. Its interpreter and GPU defaults are:

| Name | Default | Notes |
|---|---|---|
| `python` | `python3` on `PATH` | Any executable interpreter. `install` picks an Isaac Lab environment instead. |
| `uses_gpu` | `false` | `false`: the script starts at once, beside a running Isaac Sim or another job, is not counted in the GPU-hour ledger and does not block GPU jobs. `true`: the GPU guards of the managed runner apply (one GPU job at a time, none beside a running Kit bridge, the GPU-hour budget). `min_free_vram_mb` needs `uses_gpu=true`. |

`script`, `args`, `env`, `python_paths`, `cwd`, `label`, `timeout_s`, `single_instance_key`, `min_free_vram_mb` and
`wait_for_vram_s` are as above. It executes arbitrary code with your rights, with no sandbox.

### `isaaclab__play`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `checkpoint` | string | yes | — | Required; validated path. |
  | `headless` | boolean | no | `false` | Defaults to a window (`--viz kit`). |
  | `physics` | string | no | `null` | As for `isaaclab__train`. |
- Returns: same shape as `isaaclab__train`. Interactive; stop it with `jobs(action="cancel")`. With `headless=false` a window opens, titled "Isaac Lab 3.0.0" on that version and "Isaac Sim 5.1.0" on Lab 2.3.2; `isaaclab__runs_status` reports its `window` (id, title, size) for screenshot/recording.
- Example: `{"task": "Isaac-Reach-UR10", "checkpoint": "/abs/path/to/model.pth", "rl_library": "rl_games", "num_envs": 4, "headless": false}`

### `isaaclab__runs_list`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `state` | string | no | `null` | `"running"`, `"finished"` (any end state, including failed), `"all"`, or omitted for all; anything else returns an error naming these. |
  | `task` | string | no | `null` | Case-insensitive substring of the task id. |
  | `since` | string | no | `null` | ISO time (local time without a zone) or an age such as `90m`, `2h`, `1d`, `1w`. |
- Returns: `{runs: [{job_id, kind, task, state, started_at, elapsed_s, exit_code, log_dir, label?}], total, truncated, hint?}`, **newest first**.
  `total` counts every match before `limit`; `truncated` is true when `limit` cut some (then `hint` says how to narrow).
  Callers that need every job pass `limit=500` or narrow with `since`.
  Dead pids are reconciled to `failed` ("host restarted") on read.
- Example: `{"task": "g1", "state": "finished", "since": "1d", "limit": 10}`

### `isaaclab__runs_status`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `job_id` | string | one of | — | Validated id. |
  | `require_run_dir` | boolean | no | `false` | With `job_id`: an `error` instead of a status when no Lab run folder is known for the job. |
  | `log_root` | string | one of | — | Instead of a job: resolve a run folder by name under this logs folder (Lab `logs/`, a library folder, or `ISAACLAB_LOGS_DIR`; it must be inside the allowed roots). Needs `run_name`. |
  | `run_name` | string | with `log_root` | — | The `--run_name` the run used; matches folders `<timestamp>_<run_name>` exactly, so `recC` does not match `recC_v2`. |
  | `latest` | boolean | no | `false` | With `log_root`: take the newest when several folders match `run_name` (otherwise that is an `error` listing them, so a re-run never silently resolves to the wrong folder). |
  | `wait_s` | number | no | `0` | With `job_id`: block up to this many seconds (at most 3600) until the job is no longer running. See below. |
- Returns:
  `{state, pid_alive, elapsed_s, exit_code, last_log_lines, latest_checkpoint, run_dir, run_dir_source, log_dir, pid, environment_ready}`.
  `exit_code` is the Isaac Lab process's own (recorded by a shell wrapper, so it survives a host restart). `run_dir_source` says where `run_dir` came from: `lab_log` (the log names the run folder), `checkpoint` (the folder of the newest checkpoint) or `job_dir`, meaning no Lab run folder is known yet and `run_dir` is the job's own folder (a `run_dir_warning` says so; with `require_run_dir=true` it is an `error`).
  With `log_root` and `run_name` instead: `{run_dir, source: "log_root", run_name, latest_checkpoint, candidates?, note?}` (`candidates` lists every match newest first when `latest=true` chose among several). An empty or path-like name, a missing or foreign `log_root`, no match, or several matches without `latest=true` is an `error` (a no-match error lists the newest run folders it did find); the lookup never returns an empty or guessed path.
  For a job, also `physics` (the engine Lab's banner names, such as `newton_mjwarp` or `physx`; a task picks its own, and Newton is not PhysX)
  plus `window: {id, title, width, height}` when the job is live and owns a window, and `eval_result`
  (`episodes`, `mean_return`, ...) for an `isaaclab__eval` job whose runner has written it, including after the
  host that started the job has gone (it is read from `eval_result.json` in the job folder). `last_log_lines` is the last 20 useful lines of `stdout.log` (a progress-bar line keeps only its newest text after a carriage return, and a line that is only a bar is dropped, so skrl and sb3 tails show log text rather than bars); a failed job also has `last_stderr_lines`, the last 20 of `stderr.log`, where launch and import failures go. `latest_checkpoint` is the newest `*.pt`/`*.pth` under `run_dir` (or parsed from the log's "saving checkpoint '...'" lines); `run_dir` is the experiment directory reported by status (may be updated from the log). Discovery accepts both a complete experiment path and Isaac Lab 3's separate experiment-root/run-name lines, then locates checkpoints in that directory. `log_dir` remains the MCP job's stdout/stderr directory; it is not necessarily where training artifacts are written.
- **Waiting.** `wait_s` replaces a poll loop (and a background-task wait that returns at once). It returns the final status as
  soon as the job ends for any reason (success, failure, timeout, cancel), with `exit_code`, `last_log_lines` and `waited_s`.
  If the wait runs out first, the status is still `running` with `wait_timed_out: true`, and you call again. The wait polls every
  2 s on a worker thread and holds no lock, so other tool calls and launches proceed. Many MCP clients cut a call off after about a
  minute, so wait in slices of 30 to 60 s. `wait_s` without `job_id`, or outside 0 to 3600, is an `error`.
- Example: `{"job_id": "20261030-120000-Isaac-Reach-UR10"}`; to wait: `{"job_id": "20261030-120000-Isaac-Reach-UR10", "wait_s": 45}`

### `isaaclab__runs_tail`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `lines` | integer | no | `100` | Clamped to 1..1000. |
- Returns: `{lines: [str]}`.
- Example: `{"job_id": "20261030-120000-Isaac-Reach-UR10", "stream": "stderr", "lines": 50}`

### Cancelling jobs

Use `jobs(action="cancel", job_id="...")` to stop a Lab or script process group.
Inspect `jobs(action="lab_status", job_id="...")` to confirm the final state.

### `isaaclab__checkpoints_list`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `run_dir` | string | no | `null` | Restrict search to this run dir (must be under allowed roots). |
  | `task` | string | no | `null` | Substring filter against the full checkpoint path. |
- Returns: `{checkpoints: [{path, step, size_bytes, mtime}]}`, newest first, capped at 200. Without `run_dir`, searches the default log roots. `step` is parsed from the filename when possible, else `null`.
- Example: `{"run_dir": "/abs/path/to/run", "task": "model"}`

### `isaaclab__metrics_summary`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `run_dir` | string | one of | — | Must be under allowed roots. Or give `job_id`, or `compare_jobs` (final scalars of several jobs side by side). |
  | `tags` | array[string] | no | `null` | Tag list; `null` summarises every available tag. |
  | `eval_json` | array[string] | one of | `null` | Evaluation-result JSON files or glob patterns: switches to result-file mode (below). |
  | `baseline_json` / `baseline_label` | array[string] / string | no | `null` | With `eval_json`: a baseline's files or patterns, scored with the same criteria, and its name. |
- Returns: `{scalars: {<tag>: {last, min, max, step, count}}}`. Unknown tags return `{"error": "unknown tag", "available": [...]}`. Non-finite values are encoded as the strings `"nan"`, `"inf"`, `"-inf"`. Requires the `tensorboard` extra.
- Example: `{"run_dir": "/abs/path/to/run", "tags": ["rewards/iter", "Episode/dist_m"]}`

**Result-file mode (`eval_json`).** Reads your own evaluator's result files (any JSON objects) instead of TensorBoard scalars; the
files are read, never run or modified. `eval_json` and `baseline_json` entries are absolute paths or glob patterns
(`*`, `?`, `[...]`, `**`), expanded in sorted order (at most 50 files); a pattern that matches nothing is an error.

A criterion is `{key, op, value, label?}`: `key` is a dotted path to a number in the file (`stats.success_rate`,
`episodes.0.reward`; a list index is a number; a boolean counts as 0/1), `op` one of `>=` (default), `>`, `<=`, `<`, `==`, `!=`
(or `ge`, `gt`, `le`, `lt`, `eq`, `ne`), `value` the threshold, `label` a unique name (default `<key> <op> <value>`). `criteria` can
instead be the absolute path of a `.json` file (or `.yaml`/`.yml` when PyYAML is installed) holding that list.

With criteria the result is `{label, n_files, criteria, runs, summary, runs_passing_all, baseline?}`. Each run has `path`, `results`
(`label`, `key`, `value`, `passed` per criterion), `passed` (all criteria) and `checkpoint`/`task` when the file has them. `summary`
has per criterion `values`, `mean`, `min`, `max`, `passed_runs` and `n_runs`. With `baseline_json`, `baseline` has the same figures
plus `delta_mean` (this mean minus the baseline's, per criterion).
With no criteria nothing passes or fails: each run lists its `metrics` (every numeric value, flattened to dotted keys, at most 200),
`metrics` gives mean/min/max over the files, and `note` says how to add criteria; `baseline.delta_mean` is per metric.

A criterion whose key a file lacks, or whose value is not a number, is an `error` naming the file and the key path (and what the file
has there). Paths should be absolute: a relative one resolves from the server's working directory, and the error says where it
looked. `eval_json` cannot be combined with `run_dir`, `job_id`, `compare_jobs` or `tags`.

Example: `{"eval_json": ["<run>/eval_*.json"], "criteria": [{"key": "stats.success_rate", "op": ">=", "value": 0.9, "label": "success"}, {"key": "stats.mean_reward", "op": ">", "value": 100}], "label": "candidate", "baseline_json": ["<base>/eval_*.json"], "baseline_label": "baseline"}`

### `isaaclab__checkpoint_register`

Protects a checkpoint you must not lose (a baseline, an initial policy) by recording it in a Markdown table that only grows.
The only tool of the pack that writes next to your files, so it is annotated additive (not read-only, not destructive).

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `role` | string | yes | — | One line, at most 40 characters: `baseline`, `candidate`, `init`, `final`... |
  | `provenance` | string | yes | — | One line, at most 500 characters: run name, commit, settings, evaluation result. |
  | `registry` | string | no | `README.md` beside the checkpoint | A `.md` file in the checkpoint's folder or one above it; a path in the row is relative to it. |
  | `read_only` | boolean | no | `false` | Also remove write permission from the checkpoint file. Opt-in for every role, `baseline` included: checkpoints do get changed, so nothing is locked unless you ask. |
- Returns: `{registered: true, path, role, sha256, size_bytes, registry, row, same_content_as?, read_only?, git?}`. The table has the columns
  Checkpoint, Role, SHA-256, Size (bytes), Registered (UTC), Provenance; a README that has other text keeps it and gets the table at the end.
  One atomic write (a temporary file renamed over the README, under a lock) adds the row, and the old text is always an exact prefix of the new.
- Refusals (`error`): a path that already has a row is never overwritten; the error shows the existing role, hash and time and says whether the file
  on disk still matches that hash or now differs (register a changed checkpoint under a new file name). Also an empty role or provenance, a
  missing file, an unsupported extension, a registry that is not `.md`, is a symbolic link, or sits outside the checkpoint's folder tree.
  `same_content_as` lists other registered paths with identical bytes (not an error).
- Git: `*.pt` is usually ignored, so in a work tree `git` reports `ignored_by_git` and the command to use, `git add -f <checkpoint> <registry>`.
  The tool never runs git.
- Example: `{"path": "<repo>/checkpoints/baseline.pt", "role": "baseline", "provenance": "run 2026-11-01_12-00-00, iteration 1425; mean success 0.91 on the validation set"}`

### `isaaclab__tensorboard_tail`

- Parameters:

  | Name | Type | Required | Default | Notes |
  |---|---|---|---|---|
  | `run_dir` | string | yes | — | Must be under allowed roots. |
  | `tag` | string | yes | — | Scalar tag to read. |
  | `last_n` | integer | no | `50` | Last N points; must be an integer (not a boolean) or null. |
- Returns: `{points: [{step, value, wall_time}]}`. Unknown tag returns `{"error": "unknown tag", "available": [...]}`. Points are sorted by (step, wall_time) and sliced to the last `last_n` when `last_n > 0`.
- Example: `{"run_dir": "/abs/path/to/run", "tag": "rewards/iter", "last_n": 50}`

## Workflows

These describe the intended call sequence. Examples are illustrative shapes only; do not
start a job without the user's authorization and available GPU capacity.

### 1. Discover the install and tasks

1. `isaaclab__inspect(action="install")` — confirm interpreter, `isaaclab`/`isaacsim` versions, root, venv.
2. `isaaclab__inspect(action="envs")` with `{"filter": "reach"}` — confirm the task id exists and check
   `import_errors`.

### 2. Train

1. (Optional) `jobs(action="list")` with `{"state": "running"}` — ensure no other job is active.
2. `isaaclab__launch(action="train")` with the chosen task and parameters. It returns immediately with
   `{job_id, log_dir, command}`.
3. Poll with `jobs(action="lab_status")` and `jobs(action="tail")` (stdout/stderr) until the
   state is no longer `running`. For a windowed job, `runs_status` also reports `window`.
4. `isaaclab__inspect(action="checkpoints")` / `isaaclab__metrics(action="summary")` against the `run_dir`
   reported by status.

Example train call shape (illustrative):

```json
{
  "tool": "isaaclab__launch",
  "arguments": {
    "action": "train",
    "task": "Isaac-Reach-UR10",
    "num_envs": 64,
    "max_iterations": 2,
    "rl_library": "rl_games",
    "headless": true,
    "extra_args": ["agent.params.config.minibatch_size=1536"]
  }
}
```

### 3. Inspect checkpoints and metrics

1. `jobs(action="lab_status")` → read `run_dir` and `latest_checkpoint`.
2. `isaaclab__inspect(action="checkpoints")` with `{"run_dir": "<run_dir>"}`.
3. `isaaclab__metrics(action="summary")` with `{"run_dir": "<run_dir>"}` (or a `tags` list).
4. `isaaclab__metrics(action="curve")` with `{"run_dir": "<run_dir>", "tag": "<tag>", "last_n": 50}`.

### 4. Play / eval a checkpoint

1. `isaaclab__launch(action="play")` (or `isaaclab__launch(action="eval")`) with `task`, `checkpoint`, and options. Returns
   `{job_id, log_dir, command}`. For eval, wait for the job to finish and read `eval_result` from
   `jobs(action="lab_status")`: it ends on its own after `num_episodes` or `max_steps`, so no cancel is needed.
2. For a windowed `play`, call `jobs(action="lab_status")` to get `window.id`, then use the host
   `screenshot(window_id=...)` / `screen_recording(action="start", window_id=...)` tools to inspect.
   The viewport is black until the log contains "Completed setting up the environment"; check for it with
   `jobs(action="tail")`, since later lines can push it out of `last_log_lines`, then wait ~5 s before capturing.
3. Stop with `jobs(action="cancel")` when done.

Example play call shape (illustrative):

```json
{
  "tool": "isaaclab__launch",
  "arguments": {
    "action": "play",
    "task": "Isaac-Reach-UR10",
    "checkpoint": "/abs/path/to/model.pth",
    "num_envs": 4,
    "headless": false
  }
}
```

### 5. Cancel and clean up

- `jobs(action="cancel")` with `{"job_id": "...", "grace_s": 10}`.
- Re-check with `jobs(action="list")` / `jobs(action="lab_status")`. Cancelled jobs release the
  job-vs-job guard.

### 6. Your own task

A task is a Python package the tools import before Lab looks up its id. In the MCP server's environment,
set `ISAACLAB_TASK_PATH` to the folder containing the task package and `ISAACLAB_TASK_MODULES` to its module
name (or pass `python_paths` and `external_callback` per call, see
[external projects](#tasks-from-an-external-project)). Keep the package's `__init__.py` free of Isaac Lab imports because it is imported before Kit starts.
Task-specific environments, rewards and training code belong in the project using the MCP, not in this
repository. Check a new task with a scripted action before training; evaluate completed episodes separately
from training metrics and record the task configuration and seed.

### 7. Choose what a walking policy does

A trained velocity policy takes its command from the task, set through Hydra in `extra_args` for `eval` and `play`. On Lab 3.0.0 `ang_vel_z` is ignored while `heading_command` is on, so set the heading, or the robot turns to a random one:

```
physics=physx extra_args=["env.commands.base_velocity.rel_standing_envs=0.0",
  "env.commands.base_velocity.ranges.lin_vel_x=[0.7,0.7]", "env.commands.base_velocity.ranges.lin_vel_y=[0.0,0.0]", "env.commands.base_velocity.ranges.heading=[0.0,0.0]"]
```

Measured on `Isaac-Velocity-Flat-G1`, PhysX, 64 episodes: 98.4% reached the time limit with `track_lin_vel_xy_exp` 0.91, against 95.3% and 0.89 for the training commands.

### 8. A video of a play run, and its camera

`isaaclab__launch(action="play")` with `headless=true` and `extra_args=["--video", "--video_length", "400"]` writes an MP4; `jobs(action="lab_status")` lists them newest first in `video_paths`. It needs `moviepy<2`, `decorator<5` and `imageio-ffmpeg` in the Lab interpreter, and a job refuses if they are missing. Aim the camera at world coordinates with `env.sim.default_visualizer_cfg.eye=[x,y,z]` and `env.sim.default_visualizer_cfg.lookat=[x,y,z]`, on a task that sets `default_visualizer_cfg` (on one that does not, the job fails with `'NoneType' object has no attribute 'eye'`). With `num_envs=1` the environment sits at the origin.

### 9. Walking on PhysX and on Newton

Measured on `Isaac-Velocity-Flat-G1`, Lab 3.0.0, one run each, so this is not a statistical comparison.

| Engine | 1500 iterations | Final episode length | Eval return | Falls in eval |
|---|---|---|---|---|
| PhysX (`physics=physx`, 1024 environments) | 762 s | 990 of 1000 | 33.8 over 64 episodes | 3 of 64 |
| Newton MJWarp (the default) | 8.6 min | 948 of 1000 | 27.25 over 16 episodes | about 7% |

### 10. Chained jobs gated on exit codes

A second job (an eval of the checkpoint a train just wrote, a play after an eval) must start only if the first one
really succeeded. The host has no job dependency feature; the caller is the sequencer, and each hop is an explicit
check, not "the next step in the plan":

1. `isaaclab__launch(action="train")` returns `{job_id}`. Poll `jobs(action="lab_status")` until `state` is no longer `running`
   (`runs_list` with `{"state": "running"}` shows the others; the GPU guard allows one at a time anyway).
2. Continue only when `state == "succeeded"` **and** `exit_code == 0`. Treat `failed`, `cancelled` and `timeout` as
   stops: read `last_stderr_lines` and `last_log_lines`, report them, and do not start the next job. A process that
   exits 0 after an uncaught traceback is already reported as `failed` with a note, so gate on `state`, not on the
   exit code alone.
3. Take the next job's input from the first job's `latest_checkpoint` / `run_dir` in that same `runs_status` answer;
   do not guess a path. An empty `latest_checkpoint` after `succeeded` is also a stop.
4. Use `timeout_s` on each job so a hung stage ends itself even if the host is gone. After a host restart, call
   `runs_status` again: the exit code is recorded in the job folder, and `failed` with `note: "host restarted"` means
   no code was left.

For a chain run unattended outside the MCP (a cron script or a tmux shell calling Lab's own scripts), do the same
in shell. `set -e` stops at the first non-zero status; without it, check each status yourself, and never chain
with `;` or a bare `&`:

```bash
set -euo pipefail
train() { ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py "$@"; }   # your Lab launcher
play()  { ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py "$@"; }
train --task <task> --headless || { rc=$?; echo "train failed rc=$rc" >&2; exit "$rc"; }
play  --task <task> --headless --checkpoint <run_dir>/model_<n>.pt || { rc=$?; echo "play failed rc=$rc" >&2; exit "$rc"; }
```

Under `set -e` a failing stage inside `a | tee log` is hidden unless `pipefail` is set, as above. Lab's
launcher must be the one your install provides (see `isaaclab__inspect(action="install")`), not a guessed path.

## Concurrency guards

`isaaclab__train`, `isaaclab__eval` and `isaaclab__play` apply two independent guards
before spawning. The job-vs-job guard runs first.

1. **Job-vs-job guard** (`allow_concurrent`, default `false`): refuses to start if another
   Isaac Lab job started by this tool pack is still running. A job a watcher in this process
   owns counts as running until its process exits; a record from another process counts only
   if its pid is alive. Stale records of dead jobs do not block. The refusal includes `error`, `running_job_id`, `running_pid`, and `hint`.
   Pass `allow_concurrent=true` to start anyway.
2. **Kit-bridge (GPU) guard** (`allow_concurrent_gpu`, default `false`): refuses to start if
   a Kit bridge (a running Isaac Sim app) is reachable — either advertised via runtime
   portfiles (`client.discover_bridges`) or listening on the configured port (`TWINSMITH_PORT`,
   else `TWINSMITH_PORT`, else 8766; `0` or a non-integer disables the fixed-port probe).
   The refusal includes `error`, `kit_bridges`, and `hint`.
   Pass `allow_concurrent_gpu=true` to start anyway.

Each refusal's own `hint` is returned unchanged: it names the override (`allow_concurrent=true` or `allow_concurrent_gpu=true`), and the window paragraph is only added to a job that actually started.

The two flags are independent: `allow_concurrent` covers other Lab jobs only,
`allow_concurrent_gpu` covers the Kit app only. The probe is a TCP connect with a 0.3 s
timeout and does not touch the GPU. The guard is atomic only within one host process; two
separate host processes can still race. The Kit guard checks the configured port plus advertised portfiles. It does not independently scan port 8767
unless that port is configured or advertised.

### Checks before anything is launched

Three launch-time mistakes would otherwise surface tens of seconds later, after Kit has loaded,
so the tools check them first and refuse with a clear message. Each uses a short-lived probe of
the same Lab interpreter, cached for five minutes; if a probe cannot run, the check is skipped and
the launch proceeds as before.

- **`rl_library` not installed.** `isaaclab__install_info`'s `rl_libraries` check (importlib finding
  the package behind each library) is applied to the chosen library, and the refusal names the
  installed ones.
- **`--video` without its packages.** MoviePy and imageio-ffmpeg must be importable in the Lab
  interpreter; the refusal says `pip install moviepy imageio-ffmpeg` for that interpreter.
- **A task id this checkout does not register.** When no `ISAACLAB_TASK_MODULES` are configured,
  the ids in the checkout's `source/isaaclab_tasks` text are read (the same source scan
  `isaaclab__envs_list` uses) and an id whose family is there but whose exact name is not is refused
  with close matches. An id from an unscanned family is left to Lab, and configured task modules
  disable the check entirely, because they can register any id.

## Job model and state

- State directory: `~/.local/share/twinsmith/jobs` (or `TWINSMITH_JOBS_DIR`).
- Each job writes `job.json`, `stdout.log`, `stderr.log`; an `isaaclab__eval` job also writes
  `eval_result.json` (atomically) when its scoring runner finishes. Final status on exit is one of
  `succeeded`, `failed`, `cancelled`, `timeout`.
- Job ids: `YYYYMMDD-HHMMSS-<slug>` of the task, with `-2`, `-3`, ... if two launches land
  in the same second.
- The host keeps an in-memory map of active `Popen` handles; on server restart, records are
  rehydrated and dead "running" jobs take their recorded exit code, or `failed` with note "host restarted" if they left none.
- `jobs action=cancel` records `cancelled` before signalling, so the watcher does not overwrite it.
  Cancelling an already-finished job is a no-op.
- Subprocesses are launched with `shell=False`, an explicit argv list, and
  `start_new_session=True`, so the whole process group can be signalled. The child inherits a
  copy of the host environment plus `OMNI_KIT_ACCEPT_EULA=YES`, `PYTHONUNBUFFERED=1`, and
  `CUDA_VISIBLE_DEVICES` when `device` is set.

## Security and trust boundaries

- Jobs invoke `train.py`/`play.py` with `shell=False` and a validated argument list.
  Checkpoint paths are resolved against the Isaac Lab root, the jobs directory, the call's
  `python_paths`/`cwd` folders and `ISAACLAB_CHECKPOINT_ROOTS`; metrics paths against the allowed roots.
- `extra_args` is syntactically validated (flags and Hydra overrides) but is not a semantic
  allow-list of safe flags.
- The child process inherits a copy of the host environment; do not assume credentials or
  other environment variables are removed.
- Custom task modules (`ISAACLAB_TASK_MODULES`) execute Python when imported and must be
  trusted. Lab tools provide no `run_python` equivalent.

## Verified vs. version and platform limitations

Known limits of the per-call options:

- `isaaclab__preflight`, `runs_list` and the GPU-hour ledger look at the default install only; jobs do not record which `install` ran them.
- `isaaclab__checkpoint_register` takes any readable checkpoint file; the root rules above apply to the launching tools, not to it.
- Checkpoint and metric discovery is tested most for `rsl_rl` and `rl_games`: `checkpoints_list` finds stable-baselines3 `.zip` files only under a path containing `sb3`, and `metrics_summary` reads TensorBoard scalars only.

See the [compatibility matrix](COMPATIBILITY.md) for simulator and Isaac Lab version pairings and the scope of each test.

A self-contained Lab 3.0 / Sim 6.2 Newton MJWarp test used no inherited `.pth`, but
retained `fsspec` and `urllib3` package-metadata conflicts. It does not qualify generic
Sim Newton behavior or a clean-machine/full-installer setup. See
[recording a run](VERIFICATION.md#recording-a-run-so-it-can-be-reproduced) for recording an environment.

Verified with real jobs:

- Isaac Lab 3.0.0 / Isaac Sim 6.2.0.0 on Ubuntu 24.04 (RTX 5080 16 GB) and Ubuntu 22.04
  (RTX PRO 6000 Blackwell): a real 2-epoch `Isaac-Reach-UR10` rl_games run through
  `isaaclab__train` succeeded; `runs_status`, checkpoint discovery and TensorBoard metrics
  read it back. A custom `UR5e-Cell-Reach-Direct-v0` task was discovered (206 envs listed)
  and trained through the job layer.
- Windowed `isaaclab__play` opened "Isaac Lab 3.0.0"; status reported `window`; a screenshot
  and a 40-frame recording were captured. Viewport is black until "Completed setting up the
  environment".
- Job guards (job-vs-job, Kit-bridge, release after cancel) behaved as documented on the
  RTX 5080.
- Isaac Lab 3 has no `--headless` flag. Ordinary headless jobs pass `--viz none`.
  With `extra_args=["--video", "--video_length=60"]`, headless jobs use `--viz kit`
  and child-only `HEADLESS=1` so the renderer can produce frames without a window.
  Windowed jobs select Kit and `HEADLESS=0`. These settings do not change the host
  environment or another job. One explicit `--viz=VALUE` or `--visualizer=VALUE`
  extra argument overrides the generated selection and is forwarded once, with a
  response hint. Non-Kit visualizers retain their own Lab display configuration.
  Duplicate/missing visualizer values and explicit `--viz=none` with `--video`
  are refused before launch; other invalid backend choices fail in the job log.
- Lab's optional video dependencies belong in the **Lab interpreter**, not just
  the MCP host. The inspected Lab 3 launcher requires MoviePy 1.x
  (`moviepy>=1.0.3,<2.0.0.dev0`) and `imageio-ffmpeg` for `--video`. They are not
  required for plain headless training. Use the Lab checkout's documented `video`
  extra or install these into a separate Lab environment; `install.sh --no-sim`
  does not install or repair that environment. A job can start successfully and
  later fail for a missing dependency; inspect `runs_status` and `runs_tail`.
  The RL backend is chosen with `--rl_library`; when omitted, Isaac Lab 3.0.0 uses the task's registered `default_agent` if it has one; otherwise Lab exits 2 with "the following argument is required: --rl_library", which a misspelled task id also triggers.
  Name the library when you load a checkpoint: a task default of `rsl_rl` applied to an `rl_games`
  checkpoint fails inside the backend with `KeyError: 'actor_state_dict'`. `isaaclab__eval` and `isaaclab__play` therefore
  use the library named by a checkpoint under `logs/<library>/` when you pass none, and report it as
  `rl_library_from_checkpoint_path`. `--num_episodes` was
  dropped because only the rlinf backend has it. `extra_args` accepts flags and Hydra
  overrides; rl_games needs `minibatch_size` to divide `num_envs` times the task's horizon.
- Runs land under `<isaaclab_root>/logs/...`, not the job dir; `runs_status` finds the newest
  checkpoint from the training log and reports the experiment dir as `run_dir`.

Limitations and not verified:

- Long runs above ~15 min or above 1024 envs on the 5080 remain unverified.
- The host server requires Python >=3.10; the verified interpreter is Python 3.12. The Lab
  install is pinned to Isaac Lab 3.0.0 / Isaac Sim 6.2.0.0.
- Other operating systems, GPUs, Isaac Sim versions and installation layouts are not
  supported claims. A prior isolated test of the generic bridge in Isaac Sim 5.1 is not proof
  that the complete work-cell and Lab stack support 5.1.
- The Kit guard only probes the configured port plus advertised portfiles; two separate host
  processes can still race the guard.
- `metrics_summary` returns no scalars for a run too short to log any. Measured on Lab 2.3.2: `sb3` logs every
  `--log_interval` timesteps (default 100,000), `skrl` writes no event file when `max_iterations` is very small, and
  `rl_games` wrote nothing at 3 epochs. For a short `sb3` run, pass `extra_args=["--log_interval=1000"]`.
- The PyPI published install paths are not verified.
- The `tensorboard` extra must be installed for metrics readout; otherwise those two tools
  return an install hint.

## The bridge inside an Isaac Lab script

The bridge needs only Kit, so it also runs in the Kit app an Isaac Lab standalone script starts.
Start from [`mcp/templates/isaaclab_bridge_launch.py`](../mcp/templates/isaaclab_bridge_launch.py). Three things differ from the Isaac Sim app:

- The script enables the bridge itself, through `--kit_args`. What `scripts/install.py` stores
  applies to the Isaac Sim app only.
- Settings come from the environment, set before the app starts (`TWINSMITH_PORT`,
  `TWINSMITH_TOOL_PATHS`, `TWINSMITH_TOOL_MODULES`).
- The script owns the main loop, and the bridge answers only while Kit updates. In Isaac Lab
  3.0.0, `sim.step()` and `env.step()` update Kit only with a windowed Kit visualizer. Headless,
  the port accepts connections but calls that need the main loop never return (seen with
  `sim.step()`; for `env.step()` this is read from the source, not run). So call
  `simulation_app.update()` in your loop, with `/app/player/playSimulations` switched off around
  it as Isaac Lab does, so that Kit does not advance physics a second time. `status` and
  `job_status` answer regardless, because they do not need the main loop.

A Lab script is its own Kit process and needs its own port: the template uses 8767, leaving 8766
to the Isaac Sim app.

The `isaaclab__*` tools above do not use this: they start Lab scripts as jobs and need no bridge.
