# RoboSimStudio host bridge

`twinsmith.robosim.register_robosim_tools(server)` registers one grouped `robosim`
tool independently of Kit. **It needs no Isaac Sim or Kit bridge.** The server calls it only when the `real2sim` toolset is
selected (off by default, for example `TWINSMITH_TOOLSETS=default,+real2sim`, or
`toolsets(action="enable", names=["real2sim"])`). RSS is never imported
into the MCP host: every command is `python -m robosimstudio_real2sim_agent` run by RSS's own `.venv/bin/python`
(the module entry point; `examples/live/ctl` is a one-line wrapper around it).
Tested pin: **ba3650c** of <https://github.com/robosimstudio/robosimstudio>. The default T template pushed on `cpu`
(`rollout` with `routine="push"`, 6 s) gave the same result at this pin as at the previous one, `87b5e32`: solver `SolverMuJoCo`, pusher tracking error 2 mm, block end
`[0.0000007, 0.0881]` m.
Every response identifies the checkout commit and warns about a different pin.
Missing installations give setup guidance; the bridge never installs anything.

Set `TWINSMITH_ROBOSIM_ROOT` to the RSS checkout. Default:
`<workspace>/tools/robosimstudio`, with `TWINSMITH_WORKSPACE` selecting the workspace
(otherwise the folder that holds the checkout (when it has `envs/` or an asset folder), else `<home>/robotics-workspace`). No changes to RSS tracked source are needed.

**Which physics ran.** `device="cpu"` runs plain MuJoCo (MuJoCo's C library); `device="cuda:0"` runs
Newton's MuJoCo-Warp solver, for the template scene (RSS `sim/scene.py`, `build` docstring;
`specs/arrangement.py`, `MUJOCO_500HZ`). The template names that arrangement itself, 50 Hz control over 10
substeps, so both devices run the same MuJoCo physics. A scene that leaves RSS's default (`GENERIC`) puts
free rigid bodies on the **VBD** solver on a GPU, which ignores the pusher's coordinate writes: the pusher
never moves (this was the `cuda:0` failure before this change). Every run's status, every result and
`REPORT.md` say which device; the push result (`push_applied.solver`) names the solver class that stepped
the scene (`SolverMuJoCo` on both devices for the template). The first action on `cuda:0` can take minutes
while Warp compiles its kernels (`start` says so in `gpu_note`; later runs reuse the cache). This tool **compares images; it does
not fit mass or friction**, and the report says so. Fitting parameters to recorded trajectories is the job of
[`sysid`](SYSID.md), in the same toolset.

## A worked sequence

The whole flow is data: no scene file is written by hand. Times are seconds, lengths are metres.

1. `status()` — live servers and the five most recent runs, one line each (name, state, device, engine,
   age); says `bridge_needed: false`. `status(run=...)` gives one run in full, `status(all=true)` every run in
   full. If your client loads tools lazily, search for the tool by name (for example `robosim`) first.
2. `toolsets(action="enable", names=["real2sim"])` — only if the toolset is off.
3. `scene_template(shape="tee")` (or `"box"`, `"jenga"`) — optional. Writes the scene file (into the run
   folder when `run` is given, else `<workspace>/runs/robosim/_templates/`) and returns its **path**, the
   resolved template data, a compact parameter list, `size_meaning`, the `mask_key` and the coordinate frame.
   It does not return the source.
4. `start(name="trial", device="cpu", template={"shape": "tee"})` — builds the scene from plain data and writes
   `scene-v1.py` (and `scene.py`, the file RSS loads) into the run folder, then starts RSS held, **without file
   watching: the scene is frozen until `update_scene`**. `template={}` is the default T. A caller-supplied
   `scene="<abs>/scene.py"` (advanced, trusted Python) still works instead of `template`, never together.
   With no `port` it picks a free one (`port_chosen`, `url`). RSS's viewer listens on all interfaces.
5. `reference(run=..., video="<abs>/clip.mp4", start_s=0, end_s=6)` — with `start_s`/`end_s` the clip is
   **cut with ffmpeg into the run folder** (`reference-N-trim.<ext>`, one extra frame so the frame at `end_s`
   is in it) and that cut is what RSS reads: the contact sheet and the reported `duration` cover the trimmed
   span, and the result's `trimmed` block says so. Anchors default to the start, the middle and `end_s`
   (override with `at="0:start,2:a,6:end"`). Times you give and read (anchors, `start_s`, `end_s`) stay in
   original clip seconds. RSS cannot trim, so a window needs ffmpeg: the one on `PATH`, else the one the server's imageio-ffmpeg package ships. `at` alone needs none.
6. `compare(run=..., keys=["block=red"])` — the live render against the first anchor: IoU, centre offset,
   `width_ratio`, `height_ratio`, `area_ratio` (sim pixels / photo pixels) and `linear_scale_ratio` (RSS's own
   `size_ratio`, the square root of the area ratio). The mask for `block` covers every shape of the compound body.
7. `update_scene(run=..., template={"position": [-0.003, -0.072], "yaw": -0.25})` — rebuilds the scene **in the
   same run**: saves `scene-v2.py` (older versions stay), copies it to `scene.py` and asks the running server
   (same process) to reload. Keys you leave out keep the loaded template's values (a default push follows the
   object); an explicit `push` is kept. If the build fails RSS keeps the old scene, `scene.py` is put back and
   the version is recorded as not loaded. `scene="<abs>/scene2.py"` works as before.
8. `tune_push(run=..., keys=["block=red"], take="tune")` — see below. Returns the trial table and `best_push`.
9. `rollout(run=..., take="final", push=<best_push>, duration_s=6, record_fps=10)` — resets the scene to its
   start state, plays the template's `push` routine and records it. The result gives `steps`, `duration_s`,
   `control_hz`, `every`, `record_fps` (what was really recorded), `video_fps`, `frames`, `frames_expected`, the
   push as applied (`push_applied`, with the pusher's tracking error, the solver and the object's start and end)
   and `warnings`. **If the pusher did not follow its path** (`pusher_tracked` false: more than 5 mm off), the
   result starts with `headline` and `pusher_tracked: false`, and the first warning says so: that is a fault in
   the simulation setup (for example a solver that ignores the coordinate writes), not a push-path problem, and
   changing the waypoints will not help.
10. `compare_take(run=..., take="final", keys=["block=red"], start_s=0, end_s=6)` — see below.
11. `report(run=...)` — writes `REPORT.md` (engine, scene versions, a summary, every operation's JSON).
12. `stop(run=...)` — quits only this run's server and waits up to 15 s for the process to exit. It never
    force-kills. Stop every run you start.

### The template

`template` is plain data; every key is optional:

| key | meaning |
|---|---|
| `shape` | `tee` (default), `box` or `jenga` |
| `size` | tee `[bar_length, stem_length, thickness, height]`; box and jenga `[x, y, height]`; metres |
| `position`, `yaw` | `[x, y]` on the table (metres) and radians about +z; tee yaw 0 has the bar far, the stem toward the camera |
| `color` | `red`, `orange`, `yellow`, `green`, `blue`, `wood`, `"#rrggbb"` or `[r, g, b]` in 0–1 |
| `mass`, `friction` | kg (0.1; jenga 0.02) and the object–table coefficient (0.4) |
| `camera` | `{"eye": [x, y, z], "target": [x, y, z]}` |
| `push` | `{"waypoints": [[seconds, x, y], ...], "object_velocity": [vx, vy, wz]}`, the pusher path |

**Frame.** In the default camera image `+x` is right and `+y` is up (away from the camera); image y points down,
so a block that moves up-left in the picture has negative dx, dy pixels and moves toward −x, +y on the table.
`shape="jenga"` is one wooden block, default `size` `[0.072, 0.024, 0.014]` (72 x 24 x 14 mm; **x is the long
axis**), lying flat, with the same pusher, a
closer default camera and a default push that moves it about 6 cm. **Mask it with `keys=["block=wood"]`**
(RSS's `wood` colour word; `scene_template` returns the right key for any colour as `mask_key`; a custom colour
gets `block=#rrggbb~0.1`). Checked on the CPU render: the key picks out the block and nothing of the table.

**The table.** The table top is 3 mm above z = 0. RSS draws a ground plane at z = 0, and a table top at exactly
z = 0 is coplanar with it: the two fight per pixel and every render is black and white speckle. That was a
rendering artefact (z-fighting), not the table's material or the CPU path; it appeared with shadows on and off.
The table is plain light grey, so masks and pictures are clean.

### tune_push

`tune_push(run, keys, take, max_trials=6, push=None, duration_s=None, record_fps=10)` runs a bounded search over
the **pusher path** so that the simulated block's image displacement (first to last anchor) matches the clip's.
It starts from `push` (default the loaded template's push, else the scene's own `PUSH`), runs the existing
rollout and comparison for each trial (takes `<take>-1`, `<take>-2`, ...), and keeps the best. After a trial
it turns the path about the object's start by the measured direction error and stretches it from the
pusher's start by the measured length ratio (with the exponent seen so far); a trial whose pusher did not
track its path stops the search at once (`stopped_because: "pusher did not track"`, `ok: false`, `headline`
first); a trial where the block did not
move re-aims a straight push through the object along the clip's direction. It stops at the first trial with a
magnitude ratio of 0.8 to 1.25 and a direction error under 15 degrees, else after `max_trials` (at most 12).
The result has `trials` (scale, rotation, the push, `sim_px`, `clip_px`, `magnitude_ratio`,
`direction_error_deg`, start and end overlap, take), `best_push` (plain data for `rollout(push=...)` or
`update_scene(template={"push": ...})`), `within_tolerance`, `stopped_because` and `scope`. **It tunes the push
path, not mass or friction, and it matches displacement only, not the block's rotation.**

### compare_take verdicts

`compare_take` keeps RSS's own score and verdict, labelled `rss_score` and `rss_verdict` (they are an overlap
score and a did-it-move count and do not measure how far the block went). `verdict` and `bridge_verdict` are the
bridge's own, per object, from the image-centroid displacement and the end-frame overlap, with the thresholds
in the result (`bridge_verdict.thresholds`):

- `matches`: `magnitude_ratio` 0.5 to 2.0, `direction_error_deg` under 30 and end-frame overlap above 0.2;
- `moved, but not like the clip`: moved 3 px or more but a test failed; `failed` lists the numbers that did;
- `did not move`: the simulated displacement is under 3 px.

Where the per-anchor numbers come from: RSS's `compare --take` at the pin returns, per anchor, only the take frame it
was held against (`robosimstudio_real2sim_agent/checks.py`, `compare_take`; its keys only outline things on the
strip). So the bridge reads each anchor's masks itself. It runs `mcp/twinsmith/robosim/take_scores.py` in RSS's own
interpreter (`python -I`), which applies the same RSS functions a still `compare` uses (`photo_masks`, `fit_mask`,
`sim_masks`, `compare_masks`) to the take's `<take>_seg.npz` and the anchor's photograph. A take without a
segmentation, or any other failure there, is an error, not `did not move`. If a later RSS returns per-object readings
itself, they are used as given. Keys must be colour keys (`name=colour`): a seed or a mask marks one still only, so
RSS leaves it off the clip's side of a take, and `compare_take` and `tune_push` refuse it.

All long actions return quickly by default. Supply `wait_s` (0–120), or poll
`status(run=..., wait_s=120)` until `operation.state` is `completed` or `failed`.
Only one action operates a run at a time. `status` without a run lists
identity-checked live servers and the five newest runs, one line each. Runs are referenced by the exact name
returned by start. A different host can inspect completed runs and stop their
owned servers; an in-flight worker cannot resume after its host exits.

## How things move in RSS, and the push

What RSS supports (pin ba3650c; paths are in the checkout):

- **Writes to `sim.state.body_q` / `body_qd` do nothing.** They are outputs: the solver rebuilds them
  from `joint_q` and `joint_qd` each step (checked on CPU: a `body_q` write followed by `sim.run(1)` left
  the block where it was). `start` and `update_scene` refuse a scene that calls `body_q.assign(...)` or
  `body_qd.assign(...)` and say what to use instead.
- **A free body is moved through its own coordinates.** `domains/rigid/entity.py` `_add_primitive`
  records `coord_start` and `dof_start`; its `reset` writes `state.joint_q[coord_start:coord_start+7]`
  as `(x, y, z, qx, qy, qz, qw)`. The template's pusher is a free cylinder placed that way every control
  step, with its velocity written to `joint_qd[dof_start:dof_start+6]` as `(vx, vy, vz, wx, wy, wz)`,
  so it pushes through real contacts. The object's optional initial velocity is the same write. The pusher's
  height and upright orientation are written too (`z=PUSHER_Z`, identity rotation): left free in z, the cylinder
  drifted (up, then 1 cm into the table) under MuJoCo-Warp and rode over a Jenga block instead of pushing it.
- **Which solver steps the body matters.** The coordinate write moves a body only under a MuJoCo solver. RSS's
  default arrangement `GENERIC` runs free bodies on VBD on a GPU (`specs/arrangement.py`, the `"*"` backend),
  which does not read `joint_q` / `joint_qd`; a CPU device swaps in MuJoCo (`scene/physics/devices.py`). The
  template passes `physics=MUJOCO_500HZ` so one mechanism behaves the same on both devices.
- Other mechanisms exist and were not used or run here: a kinematic/mocap channel that places welded
  fixtures once per episode at reset (`domains/static/welded.py`, not a per-step path), and a PD wrench on
  a dynamic body, `ObjectDragger` in `core/draggers.py`, a `rigid_force` substep hook that adds to
  `state.body_f`. The template writes the solver's coordinates instead because the pusher then follows the
  waypoints exactly and needs no hook (a hook on CUDA forces a graph re-capture).

The push is plain data:

```json
{"waypoints": [[0.0, 0.10, -0.20], [1.0, 0.04, -0.14], [6.0, -0.12, 0.05]],
 "object_velocity": [0.0, 0.0, 0.0]}
```

`waypoints` are `[seconds, x, y]` pusher positions on the table (the pusher moves in straight lines
between them; times strictly increase; x and y within 2 m; speeds above 1 m/s give a warning).
`object_velocity` is an optional initial `[vx, vy, yaw_rate]` in m/s and rad/s; friction stops the object
again quickly (a box given 0.9 m/s travelled about 4 cm in the CPU test). Give one or both. Without `push`, `rollout` uses the scene's own
`PUSH` constant. `routine="settle"` steps with no push and is the control run: the block should stay.
Table frame: `+x` right and `+y` up in the default camera image (away from the camera), yaw `0` has the T's bar
far and its stem toward the camera.

If a routine does not apply `push` (no `push-<take>.json` is written), `rollout` fails and says to use
the template's `push` routine or to read `ROLLOUT['push']` in your own routine. Your own routine reads
`ROLLOUT` (a dict with `steps`, `take`, `push`) from its globals.

## Shapes

`shape="tee"` draws two boxes (bar and stem) on **one** rigid body. RSS draws an object's shape through
`ShapeSpec.add_to(...)` (`specs/asset.py`); the template's `TeeShape` overrides it. Mass is rescaled
for the whole compound (`scene/build/bodies.py` `set_body_mass`). `shape="box"` is one box and `shape="jenga"` a
flat 72 x 24 x 14 mm block. Dimensions are the template's `size`, written as the `TEE_*` and `BOX_HALF` constants.

## Timing

`control_hz` comes from the scene (50 Hz for the template under the pinned RSS, read at rollout time).

- `rollout`: give `duration_s` or `steps` (not both); with neither, the push's last waypoint time, else 2 s.
  `steps = round(duration_s · control_hz)`. `record_fps` (default 10) becomes a stride of
  `round(control_hz / record_fps)` steps; the real rate is `control_hz / every` and a warning appears if
  it differs from the request. Frames recorded: step 0, every stride, and the last state. Limit 20000 steps.
  The template's `push` and `settle` routines run exactly the asked steps; a routine of your own that
  ignores `ROLLOUT['steps']` runs its own length and the result warns when it differs.
- `compare_take`: clip window `start_s`–`end_s` (default: the reference's window, else the span of the
  anchors); simulated window `sim_start_s`–`sim_end_s` (default: starts at 0 and has the same length as
  the clip window). Each anchor at clip time `t` is held against sim time
  `sim_start + (t - start_s)·(sim_end - sim_start)/(end_s - start_s)`, written as `sim_t` into
  `ref/timeline-map-N.json` for RSS. The result's `time_mapping` lists the windows, the scale and, per
  anchor, the sim time requested and the one RSS used. It **warns when the two durations differ by more than
  10 percent**, and refuses a simulated window longer than the take. (RSS itself maps by proportion of
  the whole clip when no `sim_t` is given; against a 12 s clip, a 6 s anchor lands halfway through the
  take. That is the mapping this action replaces.)
- RSS posts its own verdict to its board when a take ends, using that proportional mapping. The
  `rollout` result leaves it out; use `compare_take`.
- Each `rollout` first resets the scene, so every take starts from the scene's start state. After a
  rollout the object stays where it ended until the next rollout or `update_scene`; `compare` and
  `snapshot` show that live state.

## Outputs and device isolation

Each run is `<workspace>/runs/robosim/<YYYYMMDD-HHMMSS>-<name>/`:

- `scene-vN.py` (every version), `scene.py` (the loaded one), `reference-*.mp4` (or original extension), `reference-N-trim.*`
  (the ffmpeg cut): copied inputs and hashes. A template-built scene records its data in `run.json`.
- `run.json`: device, checkout commit, inputs, scene versions, command argv/logs/results and timings.
- `serve.log`, `serve.pid`, `command-*.stdout/.stderr`: process evidence.
- `ref/`: extracted frames, contact sheet, `timeline.json`, and `timeline-map-N.json` per `compare_take`.
- `take-scores-N.json`: the request for `take_scores.py`, one per `compare_take` (keys, and each anchor's photograph and take frame).
- `snapshot-*.png`, comparison pictures/JSON, `<take>.mp4/.json/_seg.npz`, `push-<take>.json`, `REPORT.md`.

RSS caches are the only checkout-local writes: `TMPDIR=<rss>/.tmp`,
`XDG_CACHE_HOME=<rss>/.cache`, `WARP_CACHE_PATH=<rss>/.cache/warp`.
Bytecode writes are disabled. RSS can place a hashed control socket in TMPDIR
when the run path exceeds the Unix socket path limit. The bridge controls its own
output paths; trusted scene Python can still write arbitrary paths.

`start` **requires** `device="cpu"` or `device="cuda:0"`; subsequent calls inherit it.
CPU hides CUDA. The bridge checks `nvidia-smi` compute workloads before a GPU start and refuses unrelated
work unless `allow_concurrent_gpu=true` explicitly overrides that snapshot check. This is not
a GPU reservation (it can briefly count a server of your own that has just been stopped; retry). No
existing processes are killed to free resources. The unit tests use a fake RSS; the pusher, tee and Jenga were
also run live on `cuda:0` and `cpu`, with the same pusher tracking and block displacement on both.

This is **image-space comparison, not inverse-physics fitting**. There is no
automatic mass/friction estimation. Agents must supply a suitable scene and push and interpret imperfect
mask matches. RSS supplies no real reference video; provenance and licensing remain the caller's
responsibility. Missing masks yield null metrics rather than invented values. Scene Python is arbitrary
code, not sandboxed: do not use untrusted downloaded scenes.
