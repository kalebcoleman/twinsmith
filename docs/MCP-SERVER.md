# Host MCP Server: `twinsmith`

The project provides a single host-side Model Context Protocol (MCP) server that exposes Isaac Sim's scene and asset tools, Isaac Lab jobs and desktop capture to any MCP client. Its tools are grouped into [toolsets](TOOLSETS.md) that you switch on or off; one lab's UR5e work cell is part of the `sim` toolset.

The server package is located at `mcp/` (Python package `twinsmith`). The primary CLI executable is `twinsmith`. There is one command name and no aliases.

For the whole system with a diagram (what MCP is, how the server reaches Isaac Sim, ports and routing), see [How twinsmith works](ARCHITECTURE.md).

```
Kit Process (Isaac Sim 6.2.0.0)
  workcell extension ──registers──► shared bridge :8766 ◄── other tool packs
                                    │ NDJSON over TCP
                                    ▼
                        twinsmith (stdio MCP server)
                          ├── workcell__*        the work cell (toolset `sim`); executes through the shared bridge
                          ├── use_bridge        switch target port at runtime
                          ├── sim__* / assets__*  generic scene tools, from the sim-tools extension
                          ├── refresh_tools     rediscover tools; can wait for a starting Kit
                          ├── isaaclab__*       Isaac Lab jobs, run on the host
                          ├── screenshot / screen_recording / windows / join_videos / doctor
                          └── computer          desktop input (toolset `computer`, explicit-only)
```

Inside Kit, `exts/workcell` registers on the `twinsmith.bridge` singleton alongside other packs. The launcher sets
the shared port to 8766 by default and delays binding until the work cell has registered its tools. Disabling the work cell
extension removes its live handlers without stopping the shared bridge; the host keeps their catalog entries visible, and an
in-flight call must drain before the work cell can re-register.

---

## 1. Installation

The host MCP server is installed automatically when running `./install.sh` from the repository root:

```bash
./install.sh --no-sim           # Host only, editable (uv sync into mcp/.venv); installs no Sim or Lab, records a Sim it finds
# Optional, only to register these clients:
./install.sh --no-sim --claude  # Also writes Claude Code and Claude Desktop configs
```

For the pinned simulator installation, follow [Linux setup](LINUX-SETUP.md); the full installer
requires `--accept-eula` (or an existing `OMNI_KIT_ACCEPT_EULA=YES`).

To install or reinstall only the host MCP tool manually, from the repository root:

```bash
uv sync --project mcp --locked --extra isaaclab --no-dev
```

`uv tool install` does not read `uv.lock`, so it is not the checkout install. `uv sync --locked` installs the versions in `mcp/uv.lock` and checks their hashes. The project is editable: modifications to `mcp/twinsmith/` take effect whenever the MCP server process restarts. `./install.sh --no-sim` runs the same sync and links `~/.local/bin/twinsmith`. Refresh the lock with `uv lock --directory mcp`, never `uv lock` in the repository root (that project is the simulator).

`install.sh` also creates an ffmpeg shim at `~/.local/bin/ffmpeg` (a fixed default) for the Kit viewport recorder, unless an
unrelated file or symlink already occupies that path. Direct `uv tool install` does not create the shim.

---

## 2. Available Tools

The tools a client is shown are the tools of the enabled [toolsets](TOOLSETS.md); `status` and `twinsmith toolsets` report which
are on. Every tool and its arguments are in [TOOLS-ARGUMENTS.md](TOOLS-ARGUMENTS.md), one section per toolset. In short:

- **Scene and assets** (`sim`, which also holds the ROS 2 tools, and `assets`; off by default like every toolset but `core`): the generic
  `sim__*` and `assets__*` tools are catalogue-listed once their toolset is on. `assets__library` (roots, scan, search) executes on the host; the others
  require `sim-tools` in the selected Kit. See [scene tools](TOOLS-SCENE.md) for trusted-root configuration and runtime
  prerequisites. The work cell's robot, sensor and bench files are packs in the asset root, which the host and Kit each resolve
  themselves: set `TWINSMITH_ASSET_ROOT` in the **Kit** environment too, or Kit looks in `<workspace>/assets`. See
  [Asset packs](ASSET-PACKS.md).
- **Work cell** (part of `sim`): the lab's UR5e Jenga cell. The host lists a packaged catalogue of its tools even
  before a simulator runs; calls need an Isaac Sim session running `exts/workcell`, and the bridge's live metadata takes precedence
  once connected. See its [README](../exts/workcell/workcell/README.md).
- **Bridge discovery** (`core`): `use_bridge` points the host at a different bridge port at runtime; `refresh_tools` forces
  rediscovery, and `refresh_tools(wait_s=120)` waits (up to 180 s) for a Kit that is still starting.
- **Desktop and window capture** (`computer`): `doctor`, `screenshot`, `screen_recording`, `windows` and `join_videos`. Window
  capture reads that window's own pixels, so with a compositing window manager (GNOME, KDE) a covered window is still captured;
  without one the covered part is black, and `doctor` says so. Captures are written to `~/Pictures/twinsmith/` and
  `~/Videos/twinsmith/`; recordings contain no audio.
- **Isaac Lab and script jobs** (part of `sim`, with `run_script`): see the [Isaac Lab reference](TOOLS-ISAACLAB.md).
- **Explicit-only** (`computer`): the capture tools above, desktop input (`computer`) and managed simulator sessions
  (`simulators`). It is not part of any profile, not even `all`; the user names it at startup (`+computer`), and
  `toolsets(action="enable")` refuses it. `real2sim` (RoboSimStudio, the `robosim` tool) is off
  by default like `sim`; see [ROBOSIM.md](ROBOSIM.md). So are `planning` (Graph-as-Policy, the `gap` tool) and `eval` (VABAR,
  the `vabar` tool); see [planning and evaluation](PLANNING-EVAL.md).

See the [host and bridge reference](TOOLS-HOST.md) for behaviour and limits.

### Agent workflow

Use the available tools to inspect and operate the user's authorized simulator
before asking the user to repeat terminal or UI steps. `windows(action="list")` identifies
the window and PID; `screenshot(window_id=...)` captures that window, even when
covered if a compositing window manager runs (`doctor` reports it). Check the capture directory before writing if workspace rules restrict
where files may be created. Screen tools are host-side and can work even when
the simulator bridge is disconnected. Screen tools and the X11 mouse and keyboard controls are selected
explicitly at startup with `TWINSMITH_TOOLSETS=default,+computer`; see the targeting rules and shared-desktop
limitations in [host tools](TOOLS-HOST.md#optional-desktop-input).

The host MCP server and the bridge inside Kit are separate processes. A working
window or Isaac Lab tool does not mean that the bridge is connected, and a
client that lists the UR5e tools or `run_python` has not proved that a bridge exists: these are catalog-listed
(call `status`). The client may have connected before Kit started (call `refresh_tools`, or
`refresh_tools(wait_s=120)` while Kit boots),
or the selected toolsets may be disabling bridge tools
(verify with the CLI below). Reuse the intended bridge when it meets the task; start another Kit
only when explicitly needed and after the [parallel-launch checks](TOOLS-HOST.md#optional-simulator-sessions).
If `refresh_tools` returns `Connection refused`, check for the intended Kit
process, its enabled bridge extension and listening port. The server cannot
control a simulator that did not start a bridge. Once a bridge is available,
call `refresh_tools`, then use `status` to verify its PID and port before
calling `run_python` or any scene-changing tool. When several instances run,
select the intended port with `use_bridge`; never assume the visible window is
the bridge target. A 6.2 host environment does not prove that a 5.1 process has
loaded a compatible bridge. The generic `twinsmith.bridge` was also tested in
an Isaac Sim 5.1 process on 2026-10-30: after enabling it and setting
`TWINSMITH_PORT`, `status` reported the expected 5.1 process and PID.

The host advertises `tools.listChanged` and sends `notifications/tools/list_changed` after
`refresh_tools` or `use_bridge` adds, updates or removes tools, so clients that honour it (Claude
Code 2.1.314 does) show the change mid-session. If a client does not expose newly discovered tools
after `refresh_tools`, reconnect it or use the standard-library CLI against
the verified bridge. List the advertised bridges first and use the host and port it reports; 8766
below is only the default port:

```bash
python3 mcp/twinsmith/client.py --list-bridges
python3 mcp/twinsmith/client.py status --port 8766
python3 mcp/twinsmith/client.py run_python '{"code":"result = 1"}' --port 8766
```

Run this from the checkout root; it needs no packages. `python -m twinsmith.client` is the same CLI, for an interpreter
that has the host installed. For a user-started Kit
process without the bridge, do not assume the host server can attach it; load
the extension in an authorized session or start a separate process when the
user and project rules permit it.

Prefer the connected simulator's own tools for playback, inspection and logs.
Use a terminal or other explicitly available local controls when appropriate,
and report actual tool limitations instead of repeatedly asking the user to
perform actions the agent can perform. Respect project-specific rules about
launching or restarting processes, output locations and preserving live scenes.
Do not launch a real robot driver for a simulation-only task.

---

## 3. Multi-Session Configuration (Ports)

By default, the bridge binds to port **8766**. Parallel sessions need their own bridge ports, ROS domains, scene outputs and asset staging directories. For example, start these in separate terminals after replacing the placeholders with existing paths and creating the output directories:

```bash
# Terminal 1: Sim 5.1
TWINSMITH_SIM_PYTHON=<sim51-env>/bin/python \
TWINSMITH_ASSET_CONFIG=<session51>/asset-config.json \
TWINSMITH_SCENE_OUTPUT=<session51>/output ROS_DOMAIN_ID=41 \
  ./run.sh --headless --port 18751

# Terminal 2: Sim 6.2
TWINSMITH_SIM_PYTHON=<sim62-env>/bin/python \
TWINSMITH_ASSET_CONFIG=<session62>/asset-config.json \
TWINSMITH_SCENE_OUTPUT=<session62>/output ROS_DOMAIN_ID=42 \
  ./run.sh --headless --port 18762
```

Each asset config must exist before that process starts; configure its own staging directory and approved roots as described in [trusted projects](TOOLS-SCENE.md#one-time-approval). Approved source roots can be shared when both sessions need the same read-only inputs; never share staging or output directories between concurrent jobs. ROS nodes that should talk to a session must use that session's domain. To discover the actual running processes and check the target before editing a scene:

```bash
python3 mcp/twinsmith/client.py --list-bridges
python3 mcp/twinsmith/client.py status --port 18751
python3 mcp/twinsmith/client.py status --port 18762
```

In an MCP client, call `status`, then select and verify the intended process with `use_bridge(port=18762)` and `status`. The selection belongs to the host MCP server process and persists until its restart. For agents working simultaneously, give each its **own MCP server process** and select its port there; otherwise one agent's `use_bridge` can redirect the other's next call. Keep each scene's returned `session_id` and `stage_id` with its own agent and recheck them after a restart or stage change.

Two headless processes, Sim 5.1 and 6.2, were exercised together on a 16 GB RTX 5080 with small robot-import and physics workloads. This is not a capacity guarantee: scene complexity, sensors, renderers and training can require much more memory. Check available GPU and system memory before starting, monitor both workloads, and stop only sessions you own. Headless still uses GPU resources.

For managed launches, `simulators(action="launch")` chooses distinct free ports and ROS domains, applies resource checks and gives each session its own output folder; its [configuration and limits](TOOLS-HOST.md#optional-simulator-sessions) describe how to select interpreters and inspect/stop sessions. Isaac Lab has separate job and GPU guards; its [concurrency rules](TOOLS-ISAACLAB.md#concurrency-guards) explain explicit overrides. Do not treat a guard override as extra GPU capacity.


---

## 4. Client Configuration

Replace `<repo>` with your checkout (suggested: `<workspace>/twinsmith`; see
[One workspace folder](LINUX-SETUP.md#one-workspace-folder)) and `<home>` with your home directory. Optional environment
variables for any client below:

- `TWINSMITH_WORKSPACE=<workspace>` makes `<workspace>/assets` the asset root; without it the workspace is the folder that holds the checkout (when it has `envs/` or an asset folder), else `<home>/robotics-workspace`.
- `TWINSMITH_SIM_ROOT=<repo>` tells managed `simulators` launches which checkout holds `run.sh`;
  set it when the host is installed outside the checkout.
- `TWINSMITH_SIM_PYTHON=<env>/bin/python` pins the Sim interpreter that `run.sh` uses.

All settings are listed in [Environment variables](ENVIRONMENT.md).

### Claude Code

Run `./install.sh --no-sim --claude-code`, or configure it with the `claude mcp` CLI:

```bash
claude mcp add --scope user twinsmith \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e TWINSMITH_PORT=8766 \
  -- "$(command -v twinsmith || echo "$HOME/.local/bin/twinsmith")"
```

### Claude Desktop

Add the server to `~/.config/Claude/claude_desktop_config.json`, or run `./install.sh --no-sim --claude-desktop`:

```json
{
  "mcpServers": {
    "twinsmith": {
      "command": "twinsmith",
      "args": [],
      "env": {
        "TWINSMITH_PORT": "8766",
        "DISPLAY": ":0"
      }
    }
  }
}
```
*(Ensure `DISPLAY` matches the active session, for example `:0` or `:1`.)*

### OpenCode

Add the server to `~/.config/opencode/opencode.jsonc` (`./install.sh --no-sim --print-opencode` prints an entry with your values):

```jsonc
"twinsmith": {
  "type": "local",
  "command": ["twinsmith"],
  "environment": {
    "TWINSMITH_PORT": "8766",
    "DISPLAY": ":0"
  }
}
```

### Codex

Add the server with the Codex CLI, or as a `[mcp_servers.twinsmith]` table in `~/.codex/config.toml`
(the key is `mcp_servers`, not `mcpServers`):

```bash
codex mcp add twinsmith --env DISPLAY="${DISPLAY:-:0}" --env TWINSMITH_PORT=8766 \
  -- "$(command -v twinsmith || echo "$HOME/.local/bin/twinsmith")"
```

```toml
[mcp_servers.twinsmith]
command = "<home>/.local/bin/twinsmith"

[mcp_servers.twinsmith.env]
DISPLAY = ":0"
TWINSMITH_PORT = "8766"
# For the isaaclab__* job tools:
# ISAACLAB_PYTHON = "<workspace>/envs/lab-3.0/bin/python"
# ISAACLAB_ROOT = "<workspace>/IsaacLab"
# TWINSMITH_WORKSPACE = "<workspace>"
```

### Antigravity

Add the server to `~/.gemini/config/mcp_config.json` (a workspace's `.agents/mcp_config.json` is not read; inside a workspace, only a plugin's `plugins/<name>/mcp_config.json` is),
or open the agent panel's **…** menu → **MCP Servers** → **Manage MCP Servers** → **View raw
config**. The file does not allow comments.

```json
{
  "mcpServers": {
    "twinsmith": {
      "command": "<home>/.local/bin/twinsmith",
      "args": [],
      "env": {
        "DISPLAY": ":0",
        "TWINSMITH_PORT": "8766"
      }
    }
  }
}
```

Format from [Antigravity's MCP docs](https://antigravity.google/docs/mcp) and
[Codex's MCP docs](https://developers.openai.com/codex/mcp). On the test machine, `codex exec`
called `windows(action="info")` through a Codex entry of this form (2026-10-31); the Antigravity entry has not
been tested here.

### Agent skill

The repository's `twinsmith` skill tells an agent how to use these tools safely in Claude Code,
Codex, OpenCode and Antigravity. See [AGENT-SKILLS.md](AGENT-SKILLS.md).

### Limiting what a client can reach

Bridge tools such as `run_python` and the work cell's `workcell__*` tools reach the client as soon as a Kit bridge
exists, whatever the host's local tools are set to. Decide per client how much of the server it should see, and be
clear about which of the settings below limit what.

Server side: choose [toolsets](TOOLSETS.md#choosing-toolsets), for example `TWINSMITH_TOOLSETS=default,+sim`. Disabled toolsets
register no tools, listed or hidden; screen capture, desktop input and process launch need `+computer` at startup. Toolsets are not a safety
boundary: anything reaching the bridge socket directly is unaffected by host registration choices.

**A simulator that cannot be changed.** Start Kit with `TWINSMITH_READ_ONLY=1`. The bridge then runs only the
tools registered as read-only (stage and prim reads, joint and body state, logs, `status`, `jobs(action="status")`,
`files(action="get")`, `files(action="stat")` and other tools annotated read-only) and refuses every other with the code `READ_ONLY` before the tool
is called: `run_python`, `files(action="put")`, every scene edit and every other work-cell tool. Read-only protects the simulator, not
the disk: `files(action="get")` still reads any file your account can read. This is enforced inside Kit, so it does not
depend on the client, on which tools a host lists, or on who dials the socket. `status` and the bridge listing report
`read_only`. Toolsets are not a read-only host mode: even `default` (`core` alone) includes job cancellation.

**Who may connect.** The bridge requires a token on every connection. It writes a random one to a file only your
account can read, and clients on that account (this server, the CLI, your scripts) read it from there, so nothing
needs configuring on one machine. Another account on the machine, or a remote client, must be given the token as
`TWINSMITH_TOKEN`. `TWINSMITH_AUTH=0` turns the requirement off. See the [protocol](../mcp/docs/protocol.md#the-token).

Client side (these are client settings, not part of this server; check your client's documentation):

- Register the server per project rather than globally if only some projects need it. In Claude Code that is the
  project or local scope instead of `--scope user`.
- In Claude Code, `--tools` restricts the built-in tools only. To start a process that must not see MCP tools at all,
  pair it with `--strict-mcp-config --mcp-config <file>`, where the file lists only the servers that process may use
  (an empty `{"mcpServers":{}}` for none).
- Give subagents an explicit tool allowlist. A subagent without one can inherit the parent's MCP tools.
- Every local tool carries MCP annotations (`readOnlyHint`, `destructiveHint`), which clients can use for approval
  prompts, and so do the scene, work-cell and built-in bridge tools. Annotations are hints, not an access control; the
  filters above decide what is registered at all, and only read-only mode is enforced at the bridge.

### Tool counts depend on toolsets

The number of tools a client is shown depends on the enabled toolsets; the counts per toolset and per profile, and the measured
size of each, are in [TOOLSETS.md](TOOLSETS.md#the-toolsets). Use `twinsmith toolsets`, the startup log or the
`twinsmith://toolsets` resource to inspect the selection.

### Diagnosing disconnects: the server log

**Each MCP client starts its own `twinsmith` process** (Claude Code, Codex, OpenCode and every subagent that
inherits the server each get one), and the process lives exactly as long as its client keeps stdin open. Selections
made with `use_bridge` belong to that one process. A client that reports "disconnected" and reconnects has therefore
ended one process and started another; the new process knows nothing about the old one.

A client discards the server's stderr, so the server also writes a durable log, one file per process:

| Setting | Meaning |
|---|---|
| `~/.local/state/twinsmith/server-<pid>.log` | Default file. `$XDG_STATE_HOME` is honoured. The directory is created `0700`, the file `0600`. |
| `TWINSMITH_LOG_DIR=<dir>` | Another directory. |
| `TWINSMITH_LOG=0` | No file (stderr only). |

Each file rotates at 1 MB and keeps two older copies; logs of ended processes beyond the newest 30 are deleted. Every
line carries `pid=`. Request arguments, tool results, file contents and environment values other than the tool filters
are never written. The log records:

- `process start`: pid, ppid and the parent's command name (the client), version, git commit of the checkout (or
  `unknown` for a wheel), Python, tool count and the `TWINSMITH_*` filters in force;
- `bridge connected` / `bridge connection lost` / `bridge call ... failed` / `bridge selected` (each Kit reconnect);
- `sent tools/list_changed`;
- `signal received: SIGTERM|SIGINT|SIGHUP|SIGPIPE` followed by `clean shutdown: signal ...`;
- `clean shutdown: stdio transport closed (client closed stdin: EOF)` when the client simply went away;
- `unhandled exception` / `server crashed` with the traceback, and Python tracebacks for native faults (SIGSEGV);
- `alive:` every five minutes with the current ppid. A changed ppid means the client died and the server was orphaned.

How to read it: find the newest files with `ls -t ~/.local/state/twinsmith/` and read the last lines of the one whose
`process start` line names your client.

| The log ends with | Meaning |
|---|---|
| `clean shutdown: ... EOF` | The client closed the pipe: it restarted or exited the server on its own (a client restart, a timeout, a reconnect button). Not a server fault. |
| `signal received: SIGTERM` or `SIGHUP` | Something sent the signal: the client, a terminal closing, `systemd`, or a cleanup command. |
| `unhandled exception` or `server crashed` | A server bug. Report the traceback. |
| No shutdown line at all | `SIGKILL`, the out-of-memory killer or a native crash that Python could not log. Check `dmesg` and `journalctl` for the pid. |

**Stale code.** The live servers may be editable installs of a git checkout that is updated in place (for example a
timer that fast-forwards it). Python has already imported its modules, so a running server keeps the old code until its
client reconnects. `status` returns `host.code_drift: true` and `host.code_drift_message` ("checkout moved from X to Y
since this server started; reconnect to load it") when the checkout's `HEAD` moved, or when a catalog or core module
file changed on disk, since the process started. The check reads `.git` files directly and caches the file check for
30 s; `help` repeats the warning, and the error of a `status` call that no bridge answered carries it too. The Kit
side has the same hazard: Kit reloads an extension in place when its files change unless `TWINSMITH_NO_HOT_RELOAD=1`,
and clients must then reconnect.

**Connection errors and what they tell you.** The bridge client (`twinsmith/client.py`) checks a kept connection before each
call (a Kit restart leaves a dead socket, which is detected and replaced) and re-resolves the endpoint on every reconnect (a
pinned port stays, an advertised one is rediscovered). A connection that has been idle for a second or more is first proved
alive with the cheap read-only `status` before a tool that must not be sent twice, so a Kit that restarted in the meantime is
noticed before anything is sent. What is retried depends on whether the request had already reached the bridge:

- **Before the request is sent** (connecting, the dead-socket check): retried once for any tool.
- **After it was sent:** never silently re-sent, because it may have run. Only tools that purely read are re-sent once
  (`IDEMPOTENT_TOOLS` in `client.py`, an explicit list: status, listings, `sim.stage.info`, `sim.prim.get`, `sim.logs.tail`, ...;
  `sim.logs.mark` and every tool that changes anything are not in it). Everything else raises the error below.

| Error | Meaning |
|---|---|
| `bridge not reachable at HOST:PORT (connection refused): nothing is listening there. Start Kit with ./run.sh --headless ...` | The port is closed: Kit is not running or listens elsewhere. The tool was not sent. `refresh_tools(wait_s=120)` waits for a Kit that is starting. |
| `bridge at HOST:PORT did not accept a connection within Ns ... Nothing was sent, so no tool ran` | Kit is starting or its main thread is busy. Safe to retry. |
| `the connection to the bridge broke after 'TOOL' was sent ... it may already have run in Kit ... It was NOT sent again` | Kit exited, restarted or the link broke after the request went out. The tool may have run, or still be running. Check first (`sim__stage(action="info")` for a scene tool, `jobs(action="lab_status")` / `jobs(action="list")` for a Lab job, `sim__logs(action="tail")`), then repeat only if it did not happen. If Kit restarted, select it again with `use_bridge` and call `refresh_tools`. The error is a `ConnectionError` subclass (`BridgeDispatchUncertain`). |
| `no response for 'TOOL' within Ns (the tool may still be executing on Kit's main thread)` | The request was sent and no reply came within the timeout. The tool may be running or may have run; check before repeating a non-idempotent call. |

Not done, deliberately: automatic reconnect with backoff beyond the single pre-send retry. After a Kit restart the tool list can
differ: call `refresh_tools`.

### Attaching the MCP to a Kit that a script launched

`simulators` (toolset `computer`) is optional and manages only the Kit processes it started. Any other Kit can be used, because
the host only needs a bridge to dial. The mechanism, from `exts/twinsmith.bridge`:

1. The Kit process must enable the extension: launch it with `--ext-folder <repo>/exts --enable twinsmith.bridge`
   (`run.sh` also enables `workcell` and `sim-tools`; the generic `sim__*` and `assets__*` tools need `sim-tools`).
2. The bridge binds one socket at startup, on `TWINSMITH_PORT` if set, else the persisted `autostartPort` setting that
   `scripts/install.py` writes. Its address is `TWINSMITH_BIND`, default `localhost`. With no port configured, no socket
   is started. There is **no fallback port**: if the declared port is taken, the extension refuses to start and says why.
   Choose a free port yourself, or `TWINSMITH_PORT=0` only for the deferred-start launchers (`TWINSMITH_DEFER_START=1`),
   which ask the OS for one.
3. The bridge writes an advertisement, `<pid>-<port>.json`, in `$XDG_RUNTIME_DIR/twinsmith/` (else
   `/tmp/twinsmith-<uid>/`) and removes it on exit.
4. The host picks its target in this order: an explicit port (`use_bridge`), `TWINSMITH_PORT`, then
   discovery of the advertisements **if exactly one bridge is advertised**. With several it refuses and lists them.

So for a Kit started by your own script: set `TWINSMITH_PORT=<free port>` in that script's environment, enable the
extension, wait for it to log `serving on`, then in the client call `status`, `use_bridge(port=<port>)` and
`status` and confirm the PID. `simulators(action="list")` shows such a bridge as `managed: false` (`external-<pid>-<port>`); `stop`
will not stop it. A host server started before Kit may need `refresh_tools(wait_s=120)`.

---


## 5. Verification

Verify the MCP server standalone without launching Isaac Sim:

```bash
# Starts the server and immediately exits upon EOF; verifies binary and dependencies
printf '' | twinsmith
```

Verify end-to-end communication with Isaac Sim running:

```bash
./run.sh --headless &                    # wait for the app and bridge to finish starting
python3 mcp/twinsmith/client.py status --port 8766
```

`status` should answer with `"running": true` and `"app_ready": true`. The live smoke checks, which build scenes and move joints
on a disposable simulator, are described in [verification](VERIFICATION.md#test-matrix).
