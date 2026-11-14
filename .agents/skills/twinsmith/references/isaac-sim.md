# Isaac Sim through twinsmith

Read this before calling bridge tools or editing a live stage. Full references are
`docs/TOOLS-ARGUMENTS.md`, `docs/TOOLS-HOST.md` and `docs/TOOLS-SCENE.md` in the twinsmith repository; which tools are
listed depends on the enabled toolsets (`status` names them, `docs/TOOLSETS.md` explains).

## How the pieces connect

```
MCP client ──stdio──► twinsmith (host process)
                        │  NDJSON over TCP (loopback by default)
                        ▼
                      twinsmith.bridge (extension inside Kit: Isaac Sim or Isaac Lab)
                        ├── built-ins: status, run_python, job_status, job_cancel,
                        │              reload_tools, put_file, get_file, stat_file
                        └── tool packs registered under a namespace, e.g. workcell.*, sim.*, assets.*
```

Bridge tools appear in MCP as `<namespace>__<tool>` (dots are not allowed in MCP names), so
`workcell.workcell_status` is called as `workcell__cell(action="status")`. The host and the bridge are separate
processes: screen and Isaac Lab tools working says nothing about the bridge.

## Choosing the bridge

- One bridge per Kit process. `./run.sh` listens on 8766 unless `TWINSMITH_PORT`
  or `--port` says otherwise; other apps may use other ports.
- Endpoint selection order: explicit port, then `TWINSMITH_PORT`, then a lone
  advertised bridge. With several bridges, discovery refuses to guess: call `use_bridge`.
- `use_bridge` lasts until the host restarts. Re-check `status` after switching.
- Reuse the intended reachable bridge. Start an additional isolated simulator only when the
  user requested one and resource checks pass. With the
  explicit-only `computer` toolset (selected at startup), `simulators(action="launch")` starts an isolated session (own port, ROS domain,
  output directory) after resource checks; poll its `status` action until `state` is `running`, then
  `use_bridge` to its port. The `stop` action needs `expected_pid` and `confirm_discard=true`.

## run_python

`run_python` executes on Kit's main thread. Preloaded names: `omni`, `carb`, `Usd`, `UsdGeom`,
`Sdf`, `Gf`. Assign `result` to return a value; it must be JSON-serialisable, so convert USD
types (`str(path)`, `list(vec)`).

Read the stage before changing it:

```python
stage = omni.usd.get_context().get_stage()
result = {
    "root_layer": stage.GetRootLayer().identifier,
    "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
    "prims": [str(p.GetPath()) for p in stage.Traverse()][:200],
}
```

Check a prim and its world transform. USD prim paths are case-sensitive: the supplied twin
uses `/World/UR5e`. For imported robots, discover the actual path with `sim__stage(action="tree")` or
`sim__prim(action="find")` using the current stage identity; substitute that path below rather than
assuming that name applies. Prefer `sim__prim(action="get")` when a named read is sufficient:

```python
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/UR5e")
if not prim.IsValid():
    result = {"error": "no prim at /World/UR5e"}
else:
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    result = {"type": prim.GetTypeName(), "translation": list(xf.ExtractTranslation())}
```

Timeline state:

```python
import omni.timeline
tl = omni.timeline.get_timeline_interface()
result = {"playing": tl.is_playing(), "time": tl.get_current_time()}
```

Rules of thumb:

- Keep each call short. Long loops block Kit's main thread and the UI with it.
- `persistent=true` with a `session_id` keeps variables between calls; opening a new stage clears
  them because their prim handles are dead.
- Prefer a namespaced tool over `run_python` when one exists, and `reload_tools` (then
  `refresh_tools`) after editing a tool pack instead of restarting Kit.
- `run_python` and the file tools run with the Kit process's permissions. Do not run code you have
  not read, and do not expose the bridge beyond loopback.

## Seeing the simulator

- `screenshot` without a selector picks the Isaac Sim / Isaac Lab window. A covered window is still
  captured when a compositing window manager runs; without one the covered part is black (`doctor` says which). `windows(action="list", filter="Isaac")` gives ids for `window_id`.
- `screen_recording` (`start`, `stop`, `status`) records a window or monitor to MP4 (no audio).
- `computer` (`click`, `drag`, `scroll`, `type`, `key`, `batch`) acts inside one window. It needs
  `expected_width` / `expected_height` from a fresh screenshot, so take one first. Pass the screenshot's printed `coord_space`
  token to give `x`, `y` in image pixels, and `screenshot_after=true` to see the result in the same call. Prefer `run_python` or
  a named tool over clicking UI; for Isaac Sim's own widgets use `sim__ui` (the main skill's computer-use section).
- `doctor` checks the desktop (X11, XTEST, lock, Kit window) before any of this.
- These tools are the explicit-only `computer` toolset: the user selects it at startup with `+computer`; it cannot be
  enabled at run time.

## Generic scene and project assets (`sim__*`, `assets__*`)

The `sim` and `assets` toolsets carry these tools, and `run.sh` enables `sim-tools`. The workflow, refusal codes
and the camera, save and import rules are in [the main skill](../SKILL.md#isaac-sim-inspect-and-change-a-live-scene); the contracts
are in `docs/TOOLS-SCENE.md`. After reopening a scene, adopt cameras and LiDARs for capture; generic edits require a stopped
timeline.

## Work cell (`workcell__*`, toolset `sim`)

The work cell uses external pack assets. `workcell__cell(action="quick_start")` builds a stopped scene; status tools expose the build report. Real-to-sim, planning and evaluation are interface-only. See `exts/workcell/workcell/README.md`.

## Typical safe sequence

1. `refresh_tools` → bridge listing → `use_bridge` if needed → `status`.
2. For generic work, `sim__stage(action="info")` and keep its `session_id` / `stage_id`. Otherwise read
   with `run_python`.
3. Read: `sim__stage(action="tree")` / `sim__prim(action="get")` or `run_python`, plus `screenshot`.
4. Change: one named tool (`sim__*`, `assets__*`, `workcell__*`) or one small `run_python` edit.
5. Verify: read the changed prim back, `screenshot` again.
6. Report what changed, with the PID, port and (for generic tools) the session/stage IDs.

## ROS 2 from an agent shell

Lessons from the TurtleBot3/Nav2 lab.

- Your shell must use the simulator's DDS settings or `ros2 topic list` stays empty while Kit
  publishes: `sim__stage(action="info")` returns them under `ros` with the exact shell line to run
  (`sim__graph(action="build")` repeats it for ROS graphs).
- `source scripts/ros_env.sh [install] [--overlay <nav2 env>]` (after `export ROS_DOMAIN_ID=<sim's>`)
  sources ROS, then `config/ros_dds.env`, then stops the `ros2` daemon. A daemon started with other
  DDS settings makes `ros2 topic list` / `node list` empty while data flows; otherwise use
  `--no-daemon` (`ros2 topic hz` has no such flag).
- Kit's bridge nodes do not show in `ros2 node list`; verify with topics and `/tf`.
- Background RViz needs a valid desktop `DISPLAY` exported in the launching shell alongside
  `ros_env.sh` (for example `export DISPLAY=:0` when that is the desktop's display).
- The harness kills a tool call's process group when it ends: start RViz and other long-lived
  processes with `setsid nohup <cmd> > log 2>&1 < /dev/null &`, record `$!`, stop that PID.
- Save your scene before stopping a managed simulator, then call
  `simulators(action="stop", session_id=<managed session id>, expected_pid=<its pid>, confirm_discard=true)`.
- Bound every ROS command: `ros2 topic pub --times N`, `timeout -s KILL <s> ...`. A plain
  `timeout 6 ros2 topic pub ... | tail` hung 120 s while the robot drove into a wall.
- Keyboard teleop needs a TTY. Drive with `python3 scripts/cmd_vel_sequence.py "0.15,0,5;0,0.5,3"`
  (m/s, rad/s, s per step); it always publishes zero at the end, also on Ctrl-C/SIGTERM.
- Nav2's `map` frame is the world frame: the map YAML `origin` is only the image's corner, so
  do not offset goals by it.
- Same distro on both sides: Jazzy on Ubuntu 24.04, Humble on 22.04 (`run.sh` picks by OS unless
  `ROS_DISTRO` is exported). Jazzy and Humble did not exchange messages with these DDS settings.
- RViz layout: `scripts/ros2_rviz_config.py --view topdown --scale 80 --target-frame base_link
  --hide-panels` (top-down, follows the robot, 3D view only). `Hide Left Dock: true` in a
  config does nothing in RViz2.
