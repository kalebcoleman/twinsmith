# Project USD and generic scene tools

**Implementation preview; end-to-end acceptance incomplete.** These catalogued tools implement
trusted local/shared-mount projects and explicit scene operations. Untrusted isolation, authenticated
remote upload and named presets are **unavailable**, not silently substituted. Bounded USDZ support is available only in the trusted-project profile.
See [compatibility](COMPATIBILITY.md) for tested builds and known limits.
This page describes behavior and limits. For each tool's arguments and defaults, see [Tool arguments](TOOLS-ARGUMENTS.md).

## Start a scene

`sim__quick_start` builds a working scene with one call and no IDs: a ground plane, a physics scene, two lights and a camera
aimed at the origin. It uses the current stage if that is new and empty; otherwise it makes a new one. Called again with `robot` on a scene it
built, it adds that robot and keeps what you made (`kept_scene` in the result); `discard_changes=true` starts over. It adds no real-robot
wiring, so there is no mirror graph, gripper, `/joint_states` subscriber or force publisher. The lab's UR5e Jenga work cell is built
by `workcell__cell(action="quick_start")`, documented in its [README](../exts/workcell/workcell/README.md).

Its arguments (`robot`, `environment`, `ros`, `play`, `up_axis`, `meters_per_unit`, `discard_changes` and the robot-file options) are in
the [tool arguments](TOOLS-ARGUMENTS.md#sim__quick_start). A public robot name (`ur5e`, `franka`, `unitree_h1`, `unitree_g1`,
`turtlebot3_burger`, `carter_v1`, `nova_carter`, `jetbot`, `create3`) is loaded from this simulator's own assets root, checked first
and refused with the names if this build lacks it. A wheeled robot's result adds `robot.wheels` and `robot.cmd_vel_graph`, ready for
`sim__graph__build`; see the [worked example](ROS2-RVIZ.md#worked-example-a-public-robot-cmd_vel-and-a-nav2-goal). Without
`discard_changes=true`, a stage with unsaved edits is not replaced: the call refuses with `UNSAVED_CHANGES` and changes nothing.

### A Jenga tower in one call

`sim__quick_start environment=jenga` (default robot `franka`) builds a tower you can really knock over:

- 18 layers of 3 dynamic blocks (54), twice real size: 15 x 5 x 3 cm, 120 g, friction 0.5/0.4, on a 34 cm table, tower centre at (0.7, 0). Odd layers run along x, towards the arm, and their middle block is 0.4 mm thinner, so it is loose, as the block a player looks for is.
- a Franka on the floor beside the table, in a ready pose with its hand level at layer 9, holding a thin red push rod between open fingers. The rod is a collider on the hand's own body, so it adds no joint, and both fingers get a drive.
- PhysX at 120 Hz with 32 solver iterations per block. Newton runs the tower at 500 Hz: at 120 Hz it falls apart (see `NEWTON_HZ` in `jenga.py`).

The result's `environment.poke_example` has everything for one push: the `block`, the `approach`, `touch` and `through` points, and `ik` (the `link`, `offset` and `orientation` for `sim__arm__ik`). It is for layer 9. For another layer, change the `z` of each point by 3 cm a layer: the odd layers run along x, and only layers near 9 and above let the hand stay level.

To push a block out, in three calls: `sim__timeline action=play`; `sim__arm__ik` with `move=true`, `speed=0.1`, `link`, `offset` and `orientation` from `ik`, and `position` the `approach` point; then the same with `speed=0.02` and `position` the `through` point. `speed` runs the whole straight line in one call, and `path` in the result says where the tip really ended. `move=true` without `speed` goes straight to the solved joints, and nothing is commanded when the target was not reached. `sim__body__state paths=[block, top_block]` then shows what moved, at one instant. Measured in those three calls: the block out 11.9 cm and the top block moved 0.1 mm.

To knock something without the arm: `sim__physics__apply path=<block> impulse=[0.1,0,0]` gives a body one push (optionally `at` a world point). It needs a running simulation and edits nothing.

Measured live with PhysX. On Sim 6.2 and on 5.1 the tower stood still for 10 s, and the arm pushed the middle block of layer 9 out by 12.0 cm while the top block moved 0.3 mm; on 6.2 it was within 0.9 mm after the arm pulled back. With blocks all of one height, Sim 5.1's solver put the load on the middle block and pushing it carried the upper tower along until it fell, which is why that block is loose. The impulse is strong for a loose block: 0.35 N s threw it off the table with the top block unmoved, and by friction alone about 0.1 N s should stop it some 9 cm out (calculated, not run). The hand's palm is 20 cm wide, which is why the rod is there: without it the palm hits the tower and shoves everything above the layer. The arm stands on the floor because the Franka's wrist limit only lets the hand stay level about 60 cm or more above its base. There is no collision check or path planning: the arm goes straight to each solved pose, so keep the steps small near the tower. Blocks start exactly stacked, not with a real tower's small gaps and tilts.

### A warehouse in one call

`sim__quick_start environment=warehouse` (add `robot=nova_carter`, `ros=true` or `play=true` as you like; with no robot it uses `carter_v1`) builds:

- a 24 by 16 m room with 3.5 m walls;
- six shelving racks (three shelves each) with 90 loads, each box on its own pallet, in three rows with two long aisles (y = -2.5 and 2.5) and a cross aisle at x = 0;
- three loading docks in the east wall, each with a painted staging bay and two stacks of pallets and cartons, pilasters along the long walls, and a charging station where the robot starts;
- painted yellow lane lines along the aisles;
- the robot at the west end of the first aisle, facing east, ready to drive down it.

It is plain USD boxes with static colliders under `/World/Warehouse`, authored in the session layer: nothing is downloaded, so it works offline and
looks the same every time. The editor's viewport is aimed at the whole room. The result has `environment` (size, racks, aisles, the robot's start).
Live on Sim 6.2: the Carter started at (-10.5, -2.5) and drove down the aisle to x = -0.1 in six seconds with both wheels commanded at 6 rad/s.
To use it with Nav2, make the map of it with `sim__occupancy_map` (`min=[-12.5,-8.5] max=[12.5,8.5] z=[0.08,0.3] clear=[[-10.5,-2.5,0.6]] start=[-9,-2.5]`): a 500 by 339 cell map with the walls, pilasters, racks, dock stacks and charging station occupied (checked live). For a real floor plan, import a Nav2 map instead (`assets__import_path` on a map `.yaml`, see [ASSET-IMPORT.md](ASSET-IMPORT.md#ros-maps)): the Nav2
`warehouse` map is 30 by 50 m of walls. `ros=true` sets up the robot's ROS 2 topics as for any quick-start robot; that was not run against this room, so check the map and frames before sending Nav2 goals.

The result has the new `session_id` and `stage_id`, the paths, `robot.articulation_root` (joint tools need this prim, not the
robot prim), `assets` with each file's `source` and `location`, and `next_steps` with exact calls.

Robot files follow the same rules as `assets__*`. A public name (`ur5e`, `franka`, `unitree_g1`, ...) comes from the running
simulator's own asset root; nothing is downloaded by these tools. A pack file is `<pack id>/<version>/<file>` in the
[asset root](ASSET-PACKS.md): the manifest must list it and its bytes must match. An absolute path is snapshotted through your
approved roots by the host, and needs shared staging. A project file never names a folder or a robot. A robot file must match
`up_axis` and `meters_per_unit`; there is no conversion, and a mismatch refuses with
`UNITS_MISMATCH`, both sets of values and `retry_with`.
Everything is checked before the stage changes.

Edits are in the session layer. `sim__stage__save` flattens that layer, so the saved file contains the whole scene and opens on
its own. A File > Save of the root layer would not contain it. The camera has no render resources until
`sim__camera__capture` with `adopt=true`.

The camera frames the robot's first pose, so a moved arm can leave the picture. Move it with `sim__prim__set_transform`, or make another aimed
at the robot with `sim__camera__create`: `path="/World/Camera2", frame="world", position=[2,2,1.5], target=[0,0,0.4], focal_length=18`. That wider lens fits a UR5e in most poses.

## A typical session

Most tools take the `session_id` and `stage_id` from `sim__stage__info`; `sim__graph__node_types`, `sim__logs__tail`, `sim__logs__mark`, `sim__logs__summary` and the `sim__ui__*` tools do not. Call it first (or take the same IDs from `status`), and again after
anything that replaces the stage. The calls below ran against a live 6.2.0.0 headless PhysX session
([record](VERIFICATION.md#headless-physx-check)).

1. **Identify:** `sim__stage__info`, then `sim__stage__tree` with `root="/"` and `depth=2`.
2. **Author on a stopped timeline.** A parent must already exist, and a new stage has no `/World`, so create it first:
   `sim__prim__create` with `path="/World"` and `kind="Xform"`, then `kind="PhysicsScene"`, `"Cube"`, and so on.
3. **Look:** `sim__camera__create`, then `sim__camera__capture`. Output goes to `TWINSMITH_SCENE_OUTPUT`, which
   defaults to a per-process temp directory; set it at Kit launch to keep results.
4. **Initialize:** `sim__timeline` with `action="initialize"` briefly plays a stopped
   timeline and pauses it in one call; physics may advance. A runtime read confirms readiness.
   `sim__simulation__step` advances exact steps only while paused; `sim__simulation__run` runs a
   whole closed loop with your controller as a [job](#controller-runs-jobs-and-replay).
5. **Read joints:** pass the **articulation root**, not the robot's parent Xform: a robot's root is often a fixed `root_joint`
   below its prim, and the parent returns `PHYSICS_NOT_READY`. `sim__ros2_tf` reports the root as `articulation_root`.
6. **Stop before editing again:** `action="stop"`. Authored edits refuse with `TIMELINE_ACTIVE` while playing or paused.
   After a run that failed part-way, `action="reset"` cancels its jobs and stops in one call.
7. **Keep it:** `sim__stage__save` with `filename`, and optionally a `directory` under your home. `sim__stage__open`
   opens files under your home only.

A first generic Play on a timeline something else started refuses with `TIMELINE_ALREADY_ACTIVE`; stop it first. When stopping would end a project's own script that paused it (a BrickSim demo waits for Play), `action=play resume=true` continues it instead.
Helper scripts that speak MCP run as `uv run --locked --isolated --project mcp --extra scene-test --python 3.12 python scripts/<name>`
(`scene-test` adds the `pxr` and `numpy` the validators import).

## One-time approval

The user runs this outside the model-facing tool interface:

```bash
# From the checkout root:
uv run --locked --isolated --project mcp --python 3.12 python -m twinsmith.project_assets --config /absolute/private/approval.json \
  --root-id project --root /absolute/project/assets \
  --staging /absolute/private/staging --trust-project
```

Set `TWINSMITH_ASSET_CONFIG=/absolute/private/approval.json` in **both** the MCP host and
Kit launch environments, then restart them. `run.sh` enables `sim-tools`. Direct Kit users enable `sim-tools` beside `twinsmith.bridge` from this checkout.
No manifest or hand-written Python is needed. Approval is explicit configuration, not a universal
MCP approval dialog. No tool can grant roots. Launching outside the project works because paths
are absolute. Default configuration grants no roots.

A refused path now ends with this command filled in for that folder, for the agent to hand to you. Approve the smallest folder that holds the files: each import copies the whole approved folder (at most 1000 files or 256 MiB), so approving a large parent such as a workspace's `projects` folder makes imports fail with `LIMIT`.

Run the helper again with another `--root-id` and `--root` to add a folder; it only adds, and cannot
repoint an existing root or change staging. `--staging` alone, with no root, sets up staging for
importing [packs from your asset root](ASSET-PACKS.md#the-tools), which need no root
of their own. Host and Kit read the file at startup, so restart them after a change.

- A root that does not exist is reported in `assets__roots` as `missing_roots`, and the others keep
  working. It is not trusted if it comes back until a restart.
- Anything else wrong refuses the whole file with no fallback: a symlink in a root's path, a bad ID,
  or wrong staging permissions could mean a swapped folder.
- Roots must not overlap, because a snapshot copies a whole root and each file must belong to one.

The pack tools follow the same rule. `assets__pack_adopt` copies only from an approved root;
`assets__pack_build` reads sources and dependencies only from approved roots and the asset root,
which the user also sets outside the tools. All of them write only a new version under the asset root and never
modify a version that exists. See [the tools](ASSET-PACKS.md#the-tools).

Only use this for files and dependencies the user trusts with the OS account's permissions.
The raw bridge remains powerful and unauthenticated: keep it on a trusted local interface.
The broker is a same-user stdio service, **not** a remote multi-user service. An agent path
is not evidence of a Kit mount. Host and Kit must share the configured staging location; the
receiver verifies snapshot hashes before parsing. A shared mount must be deliberately configured.
A missing/different mount produces an error, never an implicit transfer via `put_file`.

## Tools

Names below are registered only when their toolset is enabled (`sim`, which includes the ROS 2 tools, and `assets`; both off by
default. `status` shows which are on, and `TWINSMITH_TOOLSETS=default,+sim,+assets` adds them; see [toolsets](TOOLSETS.md)).
Every tool and argument is in the generated [tool arguments](TOOLS-ARGUMENTS.md). The rows below are only the tools whose contract
says more than their generated description: refusals, error codes, limits and side effects. A tool with no row is fully described
there.
`assets__roots`,
`assets__scan`, `assets__search`, `assets__packs`, the `assets__pack_*` and `assets__mesh_*` tools and `assets__validate`
execute on the host even without Kit
(`assets__dependencies` and the USD parts of the pack tools use a host USD helper if one is set, otherwise the
selected Kit; see [asset packs](ASSET-PACKS.md#the-tools)). Other tools explain an absent
bridge/extension. The optional `simulators` process tools are separate (see [host tools](TOOLS-HOST.md#optional-simulator-sessions)).

| Tool | Contract |
|---|---|
| `assets__scan` | Bounded filename-only USD discovery on the MCP host. No USD parser. Results are untrusted data. A folder deeper than eight levels is skipped, not fatal: `skipped_deep` names it, `depth_limit` is 8, and the rest of the root is still listed. A snapshot still refuses, because copying part of a project silently would be wrong. |
| `assets__pack_adopt` | Copy a folder's asset files into a new `<id>/<version>` in the asset root and write its `pack.yaml` (sha256 of every file, the `source` and `license` you give, `redistributable` false unless you say so). The source must be under a user-approved root; no tool can approve folders. A version that exists is never modified. A symlink is never followed: a named one is refused, and one met while copying a folder is listed in `skipped_symlinks` with a note. `from_old` converts an old-format pack folder of the asset root into `<id>/v1`. Nothing is written inside the repository. |
| `assets__pack_build` | Copy USD, URDF, MJCF, SDF, mesh or map files and only what they depend on into a new pack version in the asset root, keeping relative layout, and write its `pack.yaml`. A file from `from_pack` keeps that pack's source, license and `redistributable`; any other file needs `source` and `license`. Sources must be under an approved root or the asset root; dependencies outside those are never copied. Unresolved or remote references refuse the build unless allow_unresolved=true, which records them; such a pack still will not import, and the result says `importable: false` with `import_blockers`. Checks the new pack afterwards. |
| `assets__mesh_split` | Writes `<stem>.usda`, `<stem>_render` and `<stem>_collision.usdc` into a new `output` folder as a job (see [asset packs](ASSET-PACKS.md#preparing-meshes-and-checking-an-asset)). Refuses an `output` that exists (`OUTPUT_EXISTS`), one inside the asset root (`INVALID_PATH`), a source or output outside approved roots (`ROOT_NOT_APPROVED`), and answers `MESH_EXTRA_MISSING` with the install command when the extra is absent. The source is never changed. |
| `assets__validate` | File-only rules for Newton or PhysX, each with level, prim, detail and fix; `ok` is false when an error-level rule fails. `kit=true` answers `NOT_AVAILABLE`. Same source and extra rules as `assets__mesh`. |
| `assets__import_path` | The one import for any robot or asset file: USD, URDF, xacro, SDF (Gazebo), MuJoCo MJCF, or a folder that holds one. `format=auto` picks it from the name; xacro and SDF are converted on the host, never by running their code. Under an approved root or a pack; source files never change. A USD composes with its own variant selections and payloads, and the result's `visuals` counts the geometry that will be drawn: a robot with physics but nothing to draw comes back with a warning instead of looking fine until you render it. See [ASSET-IMPORT.md](ASSET-IMPORT.md). |
| `sim__stage__info` | Read process incarnation, stage identity, units, timeline and bounded prim count. Start here before scene operations. Also returns `ros`: the simulator's ROS_DISTRO, ROS_DOMAIN_ID, RMW, localhost and Cyclone settings, plus the shell line that matches them. |
| `sim__quick_start` | Build a ground plane, physics scene, lights and an aimed camera with no IDs, optionally with a robot and ROS 2. See [Start a scene](#start-a-scene). |
| `sim__stage__open` | Open a .usd/.usda/.usdc/.usdz file under your home, or in this simulator's own output folder (where `sim__stage__save` writes by default), as the current stage (File > Open), timeline stopped; refuses to drop unsaved edits unless `discard_changes=true` (a fresh Kit's empty stage counts as nothing to lose). Returns the new `session_id`/`stage_id`, prim count and composition-error count; run_python sessions reset. |
| `sim__stage__tree` | Bounded stage hierarchy including inactive prims; `depth` limits levels below `root`; Kit's runtime `/Render` (Replicator) prims are hidden unless `include_runtime=true`. Text is untrusted data. |
| `sim__prim__create` | Create a Cube, Sphere, Cylinder, Capsule, Cone, Plane, Xform or PhysicsScene in the session layer. No arbitrary references/attributes. Stopped timeline, existing parent and no conflict required. Dimensions are stage units; `size` is the cube edge, the sphere/cylinder/capsule/cone diameter or height, or the Plane's width and length (its axis follows the stage up axis). `mass` (kilograms) or `density` (kg/m^3) authors PhysicsMassAPI on volume geometry; `rigid=true` gives it a runtime effect. |
| `sim__prim__create_many` | Create up to 200 prims in one call, all or none: a tower, a wall, a grid or a room without one call per piece. Each item takes `sim__prim__create`'s arguments plus an optional `material` path; `defaults` fills any key an item leaves out (kind, size, mass, material). A failing item removes the earlier ones and the error keeps its code and names its index and path. A shared `rigid`, `mass`, `density`, `collision` or `material` that doesn't apply to an item's kind (an Xform, PhysicsScene or Plane) is skipped for it and listed in `defaults_skipped`. `sim__material__apply` takes `paths` (up to 128) and checks every prim before binding any. |
| `sim__prim__set_transform` | Set a static local or world pose on **any** Xformable prim in the session layer, reporting the pose before and after; undo with `sim__prim__revert`. Authors one transform op and, when the prim's stack is not already that one op, a session `xformOpOrder` that replaces the composed order. Refuses instance proxies (name the instance root), articulation links other than the root (a joint attaches them; it names the robot Xform to move instead), time-sampled transforms, Kit runtime prims, prims with no Xformable schema and a running or paused timeline. Source files never change. No claim of runtime teleportation. |
| `sim__body__state` | Read initialized rigid-body world pose on selected PhysX or qualified Newton backend; fails when stopped/uninitialized or not a body. Supply `path` (or its aliases `prim` or `prim_path`). Position in stage units, quaternion [w,x,y,z]. Optional runtime velocities/mass from tensor API; no authored-USD fallback. `render_pose` compares that physics pose with the poses the renderer can draw the body from, Fabric and USD (PhysX usually writes USD; Newton writes Fabric and leaves USD at the authored pose): `source`, `agrees`, `position_gap`, `angle_gap_deg` and `checked`. When neither is within 5 mm and 2 degrees, a `warnings` entry says the viewport, camera captures and recordings do not show where the body is. It cannot see an instanced visual that stayed behind its link; `assets__import_path` expands those at import. |
| `sim__timeline` | Explicit play, pause, stop, initialize or reset. Initialize briefly plays a stopped timeline and pauses it, reporting that physics may advance; an already-paused timeline is left alone and an already-playing timeline is refused. Play waits up to 30 frames for the engine's tensor view and reports `physics_ready` (with a note when it did not appear), so the first read or step after it does not land before physics exists; a runtime read still confirms readiness. `reset` is the call after a failed run: it cancels this pack's running jobs, stops the viewport follow and stops the timeline, from playing, paused or already stopped, and is safe to repeat; the result has `was` and `cancelled_jobs`. Stop rewinds physics and awaits pending native render/writer completion. The timeout is cooperative. TIMELINE_BUSY or TIMELINE_INTERRUPTED requires inspection before retry. Exact stepping uses sim.simulation.step. |
| `sim__stage__save` | Export a flattened copy, including session edits, to a new basename ending .usda in configured output root. Never overwrite or switch the current stage. A result over 100 MB has a `size_note` saying why: flattening copies every referenced USD file's data into this one file. Paused Newton copies runtime rigid-body poses into the export only (<=256 bodies), not a solver checkpoint. External texture paths remain dependencies. ROS camera render handles require the explicit export option described below. `directory` writes into an existing folder under your home instead of the output root (never overwrites). Dangling internal references (a reference to a prim path that does not exist) no longer block camera render-handle saves. |
| `sim__viewport` | Point the active viewport camera. `path` alone: Kit's native select-and-frame; `framed_path` is the prim this call framed, and the Stage selection is restored afterwards (`selection`: framed for this call; Stage selection unchanged), so the Stage and Property panels keep their own selection. `path` + `margin` (metres): frame the prim's current bounding sphere plus the margin, keeping the view direction. `eye` + `target` (world, stage units): place the camera exactly (with `path`, `target` defaults to the prim's centre); the orbit pivot is set on the target. `follow=true` (with `path`): one Kit update subscription moves the camera with the prim at a fixed offset while the timeline runs (PhysX writes link poses to USD, so a robot whose links move under a fixed root is followed); `follow=false`, any new call, stage close, a viewport camera switch or an error stops it, and `sim__stage__info.viewport_follow` reports it and the last stop reason. All variants edit the active camera in session-layer edits; source layers and the previous edit target are preserved. No Kit undo entry. Refuses mismatched viewport contexts, look-through targets, unsafe session-sublayer camera opinions, empty bounds and subtrees above 10,000 composed prims. Reports the camera transform and `render_complete=false`: allow ordinary rendering, then capture the verified application window. Framing a smaller subtree can avoid surrounding occluders. Native failures can leave selection/session-camera changes; inspect before retrying. For a clean recording, `layout="clean"` hides the app's panels, menus, grid and selection outline (process-wide UI settings, not saved with the stage) and `layout="default"` puts back what was there; `caption` draws one line (up to 120 characters) over the bottom of the viewport and an empty string removes it. Given alone, they leave the camera where it is. A caption needs a viewport window: an app that has none refuses it, and a headless app draws it where nobody sees it, so label headless video with `sim__trajectory__replay` or `sim__simulation__run` instead. |
| `assets__search` | Fresh filename search across approved project/library roots. No network discovery, semantic service, manifest or fixed robot count. Refresh by calling again. A folder deeper than eight levels is skipped and named in `skipped_deep`, with `depth_limit`, instead of failing the whole search. |
| `sim__joints__inspect` | Inspect bounded authored joint limits/drives below an articulation, reporting USD degrees versus stage units. No runtime-state claim. On an articulation root, it returns the joints connected to it through its rigid bodies, with their authored limits and drives, plus `scope` and `articulation_root`; the root may be a joint (often `<robot>/root_joint`), a link carrying the articulation API, or the prim holding all of them. `limit` then caps the prims visited to find joints. A joint excluded from the articulation is left out. |
| `sim__joints__drive` | Set one revolute/prismatic joint drive target and gains in session layer; stopped timeline only. Angular target radians; gains use USD degrees-based convention. A target that is past an authored limit only by float32 rounding is put on the limit, and the result says so: a limit authored from 1.0 rad is stored as 57.2957764 degrees, one step below the exact conversion. `sim__joints__command` does the same with runtime limits. |
| `sim__simulation__run` | Run your own controller in the physics loop, as a job: every step it gets one rigid body's measured state and returns the force and torque for that step. A `tcp://127.0.0.1:<port>` controller is your own process; a `module:function` controller is code in Kit and needs `TWINSMITH_ENABLE_SCENE_SCRIPTS=1`. Writes a CSV log. See [controller runs](#controller-runs-jobs-and-replay). |
| `sim__camera__list` | List `Camera` prims under `root` with focal length and aperture (mm), clipping range (stage units and metres), projection, world pose and whether each is the active viewport camera. Bounded by `limit` and a 10,000-prim visit cap. Headless or no viewport reports `viewport_available=false`; a listed camera is not proof of render resources. |
| `sim__camera__create` | Create a persistent RTX camera on a stopped timeline, under any existing parent prim (e.g. a robot link). `target=[x,y,z]` aims it at a world point, so `position` and `target` are enough to watch something. Orientation as a `[w,x,y,z]` quaternion or as `rotate_xyz` Euler degrees (USD rotateXYZ; with `frame=local` authored as translate + rotateXYZ ops, as in the Property panel). Optional `focal_length` and `horizontal_aperture` (mm) and `clipping_range` (metres, default 0.01..1000, converted to stage units). Returns optics, horizontal FOV, local and world pose. Build identity and owned-CPU Replicator capabilities are checked; experimental.rtx is not required. USD axes: -Z forward, +Y up. Needs rendered frames; width/height capped at 1024. A USD camera has no tick rate: set a ROS rate on the camera helper (`frameSkipCount`). |
| `sim__camera__capture` | Capture RTX camera RGB into an owned CPU buffer and a new PNG in configured output root. Default waits up to five seconds for the first render completion, four further frames AND a matching renderer CPU transfer; no stale annotator fallback. Frame-number baseline is the first completion observed after the request, so an immediately created camera needs no caller-side warm-up. Returns dimensions/hash. No implicit Play or atomic physics-frame synchronization. |
| `sim__lidar__create` | Create an RTX LiDAR (`OmniLidar`) while stopped, at an exact path under an existing parent. `config` selects NVIDIA's registry entry (e.g. `Example_Rotary_2D`, `OS1` + `variant`) through `isaacsim.sensors.experimental.rtx.Lidar.create`, which the Create > Sensors menu uses; the asset is referenced from Isaac's assets root (network or local mirror), and an unknown name or variant lists the valid ones. Without `config`, a local generic LiDAR. `tick_rate` defaults to the scan rate; another rate warns about partial scans. Motion BVH is not required (6.2 enforces it only for radar); `motion_bvh` in the result says whether in-scan motion distortion is modelled. Returns prim type, sensor path, tick and scan rate. At most eight sensors; delete an exact owned sensor path to release resources. |
| `sim__graph__node_types` | Find OmniGraph node types in the running Isaac Sim by id or UI name ("Make Array" is `omni.graph.nodes.ConstructArray`), with exact port names, types, defaults and descriptions from the installed `.ogn` file or, for compiled nodes, the generated docs. Read-only. Ports include `allowed_values` for token inputs; constant nodes (no outputs) say to connect from `inputs:value`. |
| `sim__graph__build` | Create a whole OmniGraph in one call from `nodes` ([name, type] pairs), `values`, `connections` ([from, to]) and prim `targets`; timeline stopped. Checks every port against the created nodes (a typo lists the real ports), creates Make Array-style `inputs:inputN`, stores a token's value when given its key (`toStage` -> `Convert to stage units`), authors everything in USD so saves keep it, and rolls the graph back on any error. `replace=true` rebuilds an existing graph. Returns the graph as `sim__graph__inspect` does. Also accepts nodes as `{"name", "type"}` objects or with the type first (it says so), and connections as `[srcNode, srcPort, dstNode, dstPort]` or `{"from", "to"}`. For ROS graphs the result includes the shell line that matches the simulator's DDS settings. |
| `sim__graph__inspect` | Read a graph: each node's type, input values or upstream connections, and output connections, to check a hand-built Action Graph. Read-only. Lists allowed token values and each node's compute errors/warnings from its last run (`nodes_with_errors`). |
| `sim__graph__show` | Open a graph in the Action Graph editor of a windowed Isaac Sim (optionally undocked and sized), expand and auto-arrange its nodes, and zoom to fit them on every call, for screenshots and demo videos. `layout=true` arranges only graphs with unpositioned or collapsed nodes and stores the positions in the stage (`ui:nodegraph:node:pos`); stop the timeline first, or use `layout=false` while playing. A `TIMELINE_ACTIVE` refusal includes `retry_with`. It waits only the frames the editor needs (reported as `frames_waited`/`seconds`). `close=true` hides the editor. The undocked editor floats inside the Isaac Sim window. Uses the editor's internal API and refuses cleanly if it changed. |
| `sim__occupancy_map` | Generate a 2D occupancy map from the stage's colliders and write a Nav2 `map_server` PGM + YAML (origin = lower-left corner, +x right, +y up) to the output root. Starts from a stopped timeline and plays 8 frames through the guarded timeline path (the generator needs running physics; OnPlaybackTick graphs such as `/clock` run meanwhile), then stops. Free space is flood-filled from `start` (default: the bounds' centre; cells it cannot reach are unknown). A collider anywhere in the `z` range marks its cells occupied (measured on 6.0.1 and 6.2); `clear` circles mark the robot's footprint free. Z-up stages only. If a floor within one cell of `z_min` fills the slice, it uses a higher `z_min` and reports `z_used`. `z_note` is truthful about that: omap releases before 2.3.3 can still map colliders below `z_used` within about a cell, while later releases start at `z_used`. With no collider in the slice the map is written and its `warning` says no occupied cells were reached from `start`. It builds the grid itself, because NVIDIA's `getBuffer()` can overrun the heap and crash Kit. |
| `sim__ui__find` | List Isaac Sim's own UI widgets (buttons, fields, checkboxes, combo boxes, tree views; labels when a query is given) with text, tooltip, current value, a `ref`, and their centre in Isaac Sim window pixels (the same pixels as `screenshot(window_id=...)`), plus the visible Kit windows. Pixels are the window's real pixels at any display scale (`main_window` reports them with `dpi_scale`). Hidden dock tabs, closed menus and widgets inside a hidden part of a panel (the rows of a tree the Stage panel has switched away from) are left out. Read-only; works while the desktop is locked. Also returns `menu_bar` (top-level menu names; menus open with `sim__ui__act action=menu`). `zoom_ref="<ref>"` returns an enlarged crop of the Isaac Sim window around that widget instead of the list, with the widget's centre and size and the crop's own `coord_space` token, for reading small text or clicking inside the widget. |
| `sim__ui__act` | Drive Isaac Sim's UI through Kit's own input (`carb.input`, as `omni.kit.ui_test` does), not X11: `click`, `double_click`, `right_click`, `move`, `drag`, `scroll`, `type`, `keypress` (OpenAI-style key lists such as `["ctrl", "s"]`), `menu` (a path such as `Create/Mesh/Cube`; a wrong path lists the items a person sees there and closes the menu it opened, and an entry hidden in this layout is refused before anything opens), `tab` (bring a docked window such as Console to the front, like clicking its tab; `sim__ui__find` lists `hidden_tabs`), `close` (hide a Kit window or docked panel such as Console through `ui.Workspace.show_window(title, False)`, reporting `hidden`; reopen it with `tab`) and `wait`. A menu rather than an item, such as `Layouts` or `Window/Browsers`, is opened to list its items and closed again; run an item with `Layouts/<item>`. `scroll_x` and `scroll_y` are whole wheel steps. Unknown window titles and malformed menu paths are refused before any input. Targets a `ref` from `sim__ui__find` or window pixels; with `coord_space` set to the token a screenshot printed, `x`/`y` are pixels of that image (also a zoomed crop) and a token from before a resize is refused. `screenshot_after=true` returns the Isaac Sim window's picture after the action in the same answer, and `find_after="<text>"` a compact list of up to 10 widgets matching that text; a failed action still returns them when they can be taken, and a failed capture never hides a successful action. `actions=[...]` runs up to 10 steps in order, each an object such as `{"action": "click", "ref": "..."}` (inside a step the key is `action`; the top-level `frames` is each step's wait unless it gives its own): all are checked before any input, it stops at the first failure and reports how many completed, and with `screenshot_after=true` one screenshot is taken at the end. With the cursor badge on (`TWINSMITH_CURSOR_BADGE=1`) the target is marked by an `omni.ui` overlay inside Kit, with no added delay, and never appears in screenshots. Works on a locked or remote desktop, where X11 `computer` input is refused. UI actions can change or delete data like a person's clicks. |
| `sim__ros2_tf` | Build one ROS 2 graph for a robot: `/clock`, `/joint_states`, static `world -> base_link` (or `world -> odom` + odometry for a mobile base) and the link TF, rebuilt on every call so no frame is published twice. Resolves the articulation root, refuses colliding frame names; timeline stopped. See [ROS2-RVIZ.md](ROS2-RVIZ.md). After a save and reopen, calling it again deactivates the saved graph in the session layer and builds a new one, at `<graph>_r1` if the saved path is taken; `deactivated_saved_graphs` lists what it replaced. |
| `sim__logs__tail` | Read a bounded tail of the selected Kit log file, never arbitrary files. Default: the last 1 MiB, with warnings/errors and `[sim-tools]` lines in `lines` (the original behavior over a larger window, so three warnings are not pushed out by about 2 s of Info lines). With `min_level`, `channel`, `contains` or `cursor`: parsed `entries` (timestamp, level, channel, message capped at 2 KB), `next_cursor` and `truncated`. A cursor comes from `sim__logs__mark` or a previous `next_cursor`. Reads at most 1 MiB per call, never installs a log callback, and redacts obvious secrets. Also returns recent `scene_events`. |
| `sim__logs__mark` | Cursor at the current end of Kit's log: byte offset plus file identity (device, inode, size, mtime). Pass it to `sim__logs__tail` or `sim__logs__summary` to read only later lines; rotation, replacement or truncation returns `LOG_ROTATED` instead of unrelated lines. Reads the log path only. |
| `sim__arm__ik` | Inverse kinematics for any robot with revolute or prismatic joints: the joint values that put a link (the hand) at a target position and optional orientation, nearest to the current pose. Read from the robot's own USD joints, so it needs no per-robot description file. Solve only: it commands nothing, and there is no collision check or path. Needs a playing or paused timeline. It first compares its model with where physics reports the link and refuses (`IK_MODEL_MISMATCH`) if they disagree. Returns `joint_targets` to pass to `sim__joints__move`, the reached pose and the errors; `reached=false` means out of reach or past the limits, with `why_not_reached` when the cause is clear. `offset` places a fingertip or tool tip beyond the link's origin. A reachable pose can still be blocked by the ground or the robot itself: there is no collision check, so read `settled` from `sim__joints__move`. |
| `sim__arm__fk` | Forward kinematics: where a link (the hand) would be for given joint values, in the world and base frames, from the robot's own USD joints. Joints left out use their current values when the simulation runs, else zero; values outside the authored limits are warned. Commands nothing; pair it with `sim__arm__ik`. Works with the timeline stopped or running; running, it first checks its model against physics. |

Graph build accepts pairs or objects, JSON-encoded individual node/connection entries, and a
JSON string for each whole `nodes`, `connections`, `values` or `targets` container. A target
path list can also be JSON-encoded. Flat alternating node/name and connection/port pairs are
accepted when they have an even number of strings. JSON containers are limited to 64 nesting
levels and must not contain non-JSON constants such as `NaN`. For example, an Antigravity client can send:

```json
{"nodes": ["[\"tick\", \"omni.graph.action.OnPlaybackTick\"]", "[\"time\", \"isaacsim.core.nodes.IsaacReadSimulationTime\"]", "[\"clock\", \"isaacsim.ros2.bridge.ROS2PublishClock\"]"],
 "connections": ["[\"tick.outputs:tick\", \"clock.inputs:execIn\"]", "[\"time.outputs:simulationTime\", \"clock.inputs:timeStamp\"]"],
 "values": "{\"clock.inputs:topicName\": \"/clock\", \"clock.inputs:queueSize\": 10}"}
```

Without the `time` node, `/clock` publishes steadily with every stamp at zero, and the result's `notes` says so.

Supply the graph `path` and current session/stage IDs too. Type and port checks still apply.
For manual `run_python` diagnostics, prefer `sim__graph__node_types` and `sim__graph__inspect`.
If needed, `og.get_registered_nodes()` lists types; reacquire an attribute with
`og.Controller.attribute(full_port_path)` before `og.Controller.get(attr)` rather than retaining
a data handle across updates. `og.Controller.edit(graph_path, {og.Controller.Keys.SET_VALUES:
[(full_port_path, value)]})` authors a value for export; `Controller.set` is runtime-only.

Viewport framing reports both the requested and selected path. If a geometry-free
`base_footprint` has a visible `base_link` sibling and either link or their parent has a rigid-body
or articulation API, the tool selects that link; follow tracks the selected subtree. Other empty
subtrees return `NO_BOUNDS` with bounded nearby visible geometry `candidates` for an explicit
retry; inherited invisibility excludes an entire subtree. `candidates_truncated=true` means the
search hit its limit. If course map files are missing or the maps folder is empty,
`sim__occupancy_map` can generate a map from scene colliders such as
`/SimpleRoom`.

URDF conversion returns `converted_physics`: the original and selected variant, available
variants and bounded authored body/collision/joint/mass counts. Some bundled 6.x importers
select a nonexistent `physx` variant or leave physics unselected. The tool selects the
available shared `physics` description on its generated derivative before snapshotting;
flat output keeps its existing physics. A variant set without that shared description or
output missing expected bodies/joints returns `URDF_PHYSICS` before scene import. Source
URDFs and dependency files are preserved; conversion does not switch stages or edit its
source layers. Successful import adds the requested subtree to the session layer. Authored physics is not a
runtime pass: initialize the intended backend and observe bodies/joints before motion.

Other than discovery and stage-info, every tool requires exact `session_id` and `stage_id`
from stage-info. Tokens are stale-target guards, **not authentication**. Check stage-info again
after selecting/restarting Kit. Asset preparation checks the destination before copying files.
Public asset schemas accept approved root IDs/relative paths or an absolute path under an approved
root; broker snapshot parameters are internal.
Errors from these tools set MCP `isError`; legacy tools retain their prior result envelopes.
Connection/timeout failures after dispatch have unknown outcomes: inspect before retrying.

If a simulation call stops returning, stop dependent mutations; a client timeout
cannot preempt a synchronous native call. Preserve the request, endpoint, PID and
logs, and use bounded read-only health checks. For an owned managed session, the
`simulator_status` / `stop_simulator` tools (`computer` toolset) verify its identity; forced stop can lose
unsaved state and does not recover unknown in-flight effects. Restart from a known
saved input in a new verified session instead of replaying the pending mutation.
For an owned managed session, check `simulator_status`, then use `stop_simulator` with the recorded
managed ID, `expected_pid` and `confirm_discard=true`; use `force=true` only if it will not
exit normally. Verify the process has exited and resources are free before starting a new one.
Recheck endpoint, stage, backend and finite state on the new session. Never stop unrelated
processes, and do not mistake a fresh session for recovery of the original solver state.

### Project USD and trusted roots

For an ordinary object or robot, call `assets__roots`, then `sim__stage__info`, and pass the
absolute USD path to `assets__import_path` (or use the approved root ID and relative path with
`assets__import_path` with a root ID). The broker copies the bounded
approved project with relative layout intact. Kit checks the snapshot's USD layers, material
and texture dependencies and returns the created prim path, units and dependency counts. Inspect
the resulting prim or viewport before assuming physics is ready.

The user is responsible for trusting and having rights to the asset. Existing USD checks refuse
missing or outside-snapshot dependencies, URLs and unsupported paths; native USD parsing can
invoke installed resolvers before the check and this trusted workflow is not network isolation.
Absolute authored references are not rewritten into the snapshot. USD dependency inspection may
not identify every custom resolver, plugin, or directory-valued dependency. The path-based workflows
also check literal MDL texture, light-profile and BSDF-measurement paths and MaterialX
filename/include references. They refuse custom MDL modules, MDL/MaterialX inside USDZ,
MaterialX fileprefix, certain nested materials, network/absolute paths and non-literal or tiled
resource expressions they cannot validate. These checks cover common dependencies, not every
possible material or plugin: inspect the loaded asset and its renderer output. A success result
does not certify visual fidelity, physics readiness, or complete detection of arbitrary resources.
The tool copies the entire bounded approved project, not only the reported dependencies, and is
not an untrusted-USD sandbox. Keep the cache and snapshots while scenes still reference them.

The import workflows were exercised in both windowed and headless sessions
on installed Isaac Sim `6.0.0.0`, `6.0.0.1`, `6.0.1.0`, `6.1.0.0` and `6.2.0.0` on one
Linux/NVIDIA machine. These tests cover the documented sample projects, not every USD or future
release. In those tests, new cameras sometimes needed a retry after the first capture failed.

Pose coordinates use **stage units**, `orientation=[w,x,y,z]` must be a finite unit quaternion,
and `frame` is `local` or `world`. World transforms use the parent's inverse matrix; singular
parents are refused. Source and target units/up-axis must match. No automatic normalization.

**Up-axis and units.** A new Isaac Sim stage is Z-up with `metersPerUnit=1.0`. Many public USD assets
are Y-up, or in centimetres, and URDF imports are Z-up in metres. A mismatch refuses with
`UNITS_MISMATCH`, naming both sets of values. To match, make a scene with
`sim__quick_start`, for example `up_axis="Y"` and `meters_per_unit=0.01`, or open a file authored that way with
`sim__stage__open`. No tool converts an asset.

**URDF meshes** can be `package://` or relative paths, inside the approved root or pack. See [URDF robots](#urdf-robots).
Authored edits require a **stopped** timeline (paused physics is also refused). Saving a composed copy also permits paused physics; playing saves refuse. Physics state
observations do not establish joint stability, robot fidelity or hardware behavior.

Edits go to the session layer, never source layers or source project files. Imports use a wrapper
so asset-authored transforms survive. Parents must exist. Failure after an application exception
reports unknown effects, and failed creations/imports attempt to remove their new session-layer
prim. Deletion/clear only touch pack-owned session-layer subtrees; sensors/articulations are protected.
No generic attribute setter or arbitrary asset-valued entry point is provided. Reloading the pack loses pack ownership and invalidates its session token.

## Saved LiDAR scenes and `/scan`

A flattening save used to write a LiDAR render product's session-only `viewportHandle` into the file, and a raw `Stage.Export` still does, along with Replicator's runtime pipeline
(`SDGPipeline` graphs, `/Orchestrator`). Reopened, such a file has no `/scan`, while `sim__lidar__read` (a separate private product) and `sim__graph__inspect`'s wiring can look fine. There are two causes, and they are not the same on every build:

- **The handle.** Measured on 6.2.0.0 and read in its Replicator: `IsaacCreateRenderProduct` reuses the saved product and `attach_hydra_texture` returns early because the attribute exists. Deleting only that line from a file without a saved pipeline brought `/scan` back.
  This is not the mechanism on 6.0.0.0, which has no `attach_hydra_texture` and always creates a new product.
- **The saved pipeline.** OmniGraph loads it at open and it breaks later writers: `Could not process writer attach request ... getWrappedGraphFromNode`, `writer_computes` 0, and `sim__lidar__read adopt=true` or a new `sim__lidar__create` fail in that stage until a new stage. On 6.2.0.0 and 6.0.0.0, removing only the handle lines from a raw export left this failure; 6.1.0.0, 6.0.1.0 and 6.0.0.1 showed the same errors with the old repair, which deactivated the product. On 6.2.0.0, deactivating the pipeline in the session layer too was worse: Kit could not create nodes at that path.

What the tools do:

- **New saves:** a LiDAR handle behaves like a camera's: `SAVE_UNSUPPORTED` with `retry_with`, and `reset_camera_render_handles=true` removes it from every product a helper uses. Saves omit the runtime pipeline. The viewport's own product (`omni_kit_widget_viewport_*`) keeps `viewportHandle = 0`, which no helper reuses unless it targets the viewport camera at the same resolution. A session handle on a product no active helper uses also stays, and is listed in `stale_render_handles`.
- **While one is open:** Kit can log thousands of `OgnSdOnNewFrame ... discarded` errors from NVIDIA's plugin running the saved pipeline (1,941 in 16 s on 6.0.0.1). They stop once the rescued copy is open.
- **Existing files:** `sim__stage__open` never changes the file, and adds `stale_render_products` when it finds a stale LiDAR product or a saved pipeline.
  In a file with no saved pipeline, on a build where it was measured (6.2.0.0 and 5.1.0.0, headless PhysX), it deactivates the product in the session layer and a new one is made at Play. Pinned, connected or live products are reported and left alone.
  Otherwise nothing is deactivated, and `rescue` gives the fix below.
- **Rescue:** before any Play, with the timeline stopped, call `sim__stage__save` with a new filename and `reset_camera_render_handles=true`, then `sim__stage__open` the new file. The copy drops the saved LiDAR handles and pipeline. Measured with `/scan` messages on 6.2.0.0 and 5.1.0.0 from a raw export, without playing first. On 6.1.0.0 the same save worked after a paused session with the old repair; the other builds passed fresh save and reopen but have not run this exact sequence. A stale camera handle in the same file still refuses.
- **Opened another way** (GUI File > Open, `run_python`): no check runs. `sim__graph__inspect` reports `stale_handle`, and a save warns in `warnings` and `stale_render_handles`, rather than refusing and losing work.
- **Without the tools:** in a file with no saved pipeline, rebuild the graph at another width or height before the first Play (6.2.0.0). Removing the handle after a Play in the same session does not help, because the node has cached the path.
- **Diagnosing:** `sim__graph__inspect` adds `ros_render_status`, with `status` and `writer_computes` per helper, and `ros_publisher_note`, plus `ros_helpers_with_problems` when any helper has one, including a `texture_bound` helper whose `writer_computes` is 0 while playing (it can be 0 briefly after Play). `texture_bound` alone does not prove a healthy sensor. It reports render facts, **not publisher count**, which could not be read reliably from inside Kit. Confirm with `ros2 topic info`. It also adds `ros_domain_note` when a `ROS2Context` publishes on an authored domain other than `ROS_DOMAIN_ID`.

Stale **camera** handles have no open-time repair, because the camera contract is unchanged. A LiDAR with no geometry between its near and far range (1 m to 200 m for `Example_Rotary` and `Example_Rotary_2D`) advertises `/scan` but sent no
message in testing, and `sim__lidar__read` returns `NOT_READY`. UR5e Quick Start hides the ground plane's visual and puts the tabletop at z=0, so a 2D LiDAR above the bench can see nothing: on 6.2.0.0 all three test sensors there read `NOT_READY` until a wall stood in range.

Saves now accept a bare basename and add `.usda`, and report `graphs` (at most 32), `graph_count` and `graphs_truncated`, so a mislabeled checkpoint shows up. A tool's own `IsaacMCP_*` capture
product needs no flag and does not make a graph's product on the same camera ambiguous; it is removed whole from the copy. Two producers on one LiDAR (LaserScan and PointCloud) share one product, which saves normally. A camera or LiDAR graph that leaves `inputs:width` and `inputs:height` unauthored uses the live node's values when a product with that camera and resolution proves them; if not, the refusal names those inputs.

## Reopened scenes

Prims and sensors from an opened file are not pack-owned, even ones you made earlier, so guarded tools refuse with `NOT_OWNED` and name the option below in `retry_with`:

- **Reading sensors:** `adopt=true` on `sim__lidar__read` or `sim__camera__capture` creates render resources for the sensor at that exact path. For LiDAR this is not proven on 6.0.0.0: it stayed `NOT_READY` in all 6 reads while that file's `/scan` had a publisher. It never happens automatically, because it costs GPU. A LiDAR's first read can return `NOT_READY`: keep the timeline playing and retry after a few seconds (about 3 s in testing). An adopted sensor stays non-owned, counts toward the eight-sensor limit and is released on stage change. LiDAR frame and accumulation settings that differ are overridden in the session layer and listed in `session_overrides`. Cameras render at width 640. Kit's render product also lands in the in-memory root layer (`root_layer_note`): nothing reaches disk, but a person's File > Save would write it.
- **Deleting:** `allow_foreign=true` on `sim__prim__delete`. USD cannot remove a root-layer opinion from the session layer, so session-only prims are removed and anything else is **deactivated**. The path stays taken, so recreating there gives `CONFLICT`: use a new path, or open a saved copy, which omits the prim. Reopening the source restores it. World or default prims, articulations, registered-sensor subtrees, Kit runtime prims, instance proxies and single graph nodes still refuse. Source files are never changed.

## Changes from outside this session

Stage opens and closes and timeline play, pause and stop go to the Kit log tagged `[sim-tools]`, and `sim__logs__tail` returns the tagged lines and `scene_events` (the last 32 changes). A tool is credited only when it alone was running. If another tool that could have done it ran within 2 s the source is `unattributed`; "outside" is claimed only when no such tool was active.
`STALE_TARGET` names the current stage and adds `stage_change`. `TIMELINE_ALREADY_ACTIVE` and `TIMELINE_ACTIVE` name the stop call in `before_retry` and warn that Stop rewinds physics. `MISSING_TARGET` covers direct bridge callers with no IDs. The `assets__*` ID check runs on the host, so it does not get this explanation.

`sim__stage__open`'s `UNSAVED_CHANGES` now compares real content. Kit marks every opened file dirty by rewriting `customLayerData` and its own viewport cameras, which made the guard always fire, so those are ignored, as are Replicator's runtime `/Orchestrator` graphs from a Play. Session-layer edits from tools and real root-layer edits still refuse. Files over 512 MiB keep Kit's dirty flag and can still over-report.

## Bounds and security limits

Approved-root reads use descriptor-relative no-follow opens, inode/change checks, and reject
symlinks/special files, traversal, URLs, controls and replacement races detected during copying.
Filename scans count all entries, not only USDs. They fail explicitly on budget exhaustion rather
than claiming a complete partial result. Pagination is a fresh scan, not a stable filesystem snapshot.

Snapshot limits: 64 MiB/file, 256 MiB/project, 1,000 files, 10,000 entries, eight path components,
15-second enumeration budget, 16 retained snapshots/staging directory plus at most one
bounded temporary copy under the staging lock. Identical project/entry manifests reuse an
existing snapshot only after all its contents are verified again; changed projects consume
another slot. Existing snapshots are never evicted while a stage or saved output may use them.
The complete bounded
project is copied with relative layout intact. Hashes are checked on the Kit side. Source files
may change later without changing loaded copies. Files are private/read-only but the same OS user
can modify them: hashes and permissions are **not immutable OS isolation**. Do not expose this
configuration to hostile writers. Snapshots are retained because loaded/saved scenes may reference
textures in them; clean staging only after all consumers and exported scenes are no longer needed.
Quota exhaustion is explicit; there is no unsafe automatic deletion of live dependencies.

The trusted native `UsdUtils.ComputeAllDependencies` inspection can itself invoke resolvers/parsers.
It reports missing/outside-snapshot dependencies and rejects those imports. The sole external
exception is the exact installed Kit `OmniPBR.mdl` module; its path/hash/source are returned.
Archive-bearing inputs must pass the bounded trusted USDZ checks below; unsupported ZIP formats are refused before native parsing. This is **not** a
pre-parser sandbox or proof of exhaustive plugin behavior. Known OmniGraph prims are refused,
but the importer makes no claim that all executable asset content is detected or disabled.
No untrusted USD is parsed by this workflow; an isolated profile config is refused entirely, including MCP server startup (no raw tools are exposed by such a failed server). No isolated Kit endpoint is created.
There is no verified whole-lifetime isolated GPU Kit deployment or protected untrusted endpoint.

Scene traversal is capped at 10,000 visits; tree/physics responses at 200 nodes and the generic
bridge response at 64 KiB. `prim.get` omits arbitrary attribute/custom-data contents and expensive
recursive bounds. Native USD composition itself is not resource-isolated in this trusted profile.

Save flattens the **composed active state**: inactive prims and unselected variants are omitted.
It is not a lossless authoring-layer archive. Textures can retain snapshot paths. Output publishing
uses no-replace hard links; existing files and symlinks are not overwritten. The configured output
directory should be private to the OS user. Defaults are process-specific under the temporary directory.

A collider overlapping a robot can make PhysX report `Illegal BroadPhaseUpdateData - non-finite bounds` and `Invalid PhysX transform` on Play, on every build we tried. Move the collider clear of the robot; saving and reopening does not cause this.

Each new or opened stage leaves about 40 to 55 MB behind in Kit, which a 45-minute test on 6.2 traced to Kit's stage switch rather than to these tools; a long rebuild loop should restart Kit now and then. Generic edits are in the session layer and make no undo entry, so Ctrl+Z in Kit won't undo them and may undo an earlier edit of yours. `sim__prim__find` also lists Kit's runtime prims (`/Render`, `/Orchestrator`), which `sim__stage__tree` hides unless `include_runtime=true`. `simulator_status` reports `busy` when a bridge answers that an exclusive call holds Kit's main thread: the simulator is alive, so wait rather than relaunch.

## Extension hot reload

Kit watches extension folders. When a file in `exts/sim-tools` changes, Kit stops and restarts that extension inside the
running process, with the same PID. An edit to `twinsmith.bridge` restarts all three extensions. To prevent this, launch
with `./run.sh --no-hot-reload` or `TWINSMITH_NO_HOT_RELOAD=1`; see [ENVIRONMENT.md](ENVIRONMENT.md).

After a reload, `session_id` and `stage_id` change and the tools forget which prims they created. The stage itself is
unchanged. A call with an old `session_id` gets `STALE_TARGET` saying this happened. `sim__logs__tail` shows a
`scene_events` entry of kind `extension`, earlier events are gone, the Kit log has a `[sim-tools]` warning, and `status`
counts it under `extension.reloads`.

If a sim tool call is running when the reload happens, the sim and asset tools are missing until that call finishes,
then return by themselves. A wait over 30 s is logged as an error. The work cell does not have this: if a work-cell call is
running when `exts/workcell` reloads, its startup fails with a message saying so, and it needs another reload or a restart.

## Joint control by name, gains, settling and teleport

`sim__joints__gains`, `sim__joints__move` and `sim__joints__teleport` need an initialized articulation
(playing or paused), like `sim__joints__state`. `sim__joints__gains` and `sim__joints__move` change runtime
state only: gains, targets and teleported poses are not USD, are never saved, and are lost on Stop.

A typical steer-by-name loop:

```json
{"session_id": "...", "stage_id": "...", "path": "/World/ur5e/root_joint",
 "targets": {"shoulder_lift_joint": -1.2, "elbow_joint": 0.6}}
```

Pass that to `sim__joints__command`. An out-of-limit value is refused with its runtime limits; resend
with `clamp=true` to use the limit and get a `clamped` entry. A velocity target is a drive target, not the
measured speed; a wheel usually needs `stiffness=0` with damping, set through `sim__joints__gains`.

To reach a pose rather than acknowledge a target, pause the initialized timeline and call
`sim__joints__move` with the same `targets`. It steps up to `max_steps` and returns `settled`, the
steps used, simulated seconds and every joint's error. A joint that cannot reach its target is
reported in `blocked` with its remaining error, never as success: a low arm can be held by the ground,
and a weak drive by its load. `sim__joints__teleport` sets positions immediately for a state reset (for
example after editing a robot's layout); it bypasses motion and leaves drive targets alone, so the next
steps may pull the joints back.

`sim__joints__command` and `sim__joints__move` validate against the runtime limits from the selected
backend on Isaac Sim 5.1 and 6.0 through 6.2. `sim__joints__gains` reads and writes whichever gain API
that build exposes (`get_dof_stiffnesses`/`set_dof_stiffnesses`, or the experimental wrapper's gain pair)
and refuses with `PHYSICS_API_UNAVAILABLE` when neither exists, rather than changing something else.

## Editing any prim: move, physics, attributes, revert

These tools edit **any** prim on the stage, not only prims this pack created: imported assets, prims
from your own USD files and, in a disposable session, the work cell's prims. Safety comes from how the edit
is made and undone:

- Every edit is a session-layer opinion. The source USD files and the stage's root layer are never
  written; `source_files_changed` is always `false`.
- Each call reports the composed value before and after. `sim__prim__revert` removes the session
  opinions these tools recorded for one prim (or its subtree), restoring the composed values from the
  source files, and reports what it removed and the resulting values.
- While the timeline is playing or paused, these edits refuse with `TIMELINE_ACTIVE`; stop first.
  `sim__prim__set_transform` and `sim__physics__apply` also refuse while physics is active.

Moving a robot or a table is just `sim__prim__set_transform` with the prim's path. Two limits are real:
a prim attached by a joint is a link, and moving it would break its joints, so the refusal names the
robot Xform to move instead; a time-sampled transform would lose its animation, so animate a parent
Xform or the source clip. An instance proxy inside an instance is read-only: pass the instance root.
A prim with no Xformable schema (a `Scope`, for example) has no pose to set.

`sim__physics__apply` adds a collider (optionally with an `approximation`), a rigid body, the kinematic
flag, or `mass`/`density` (not both). A rigid body under an existing rigid-body ancestor is refused as
`NESTED_BODY`; a joint or articulation root cannot become a body. Turning off a collider or body that a
source file applies refuses with `READ_ONLY_SOURCE`, because removing another layer's opinion from the
session layer would not remove the schema: edit the source file instead.

`sim__prim__set_attribute` writes one typed attribute and refuses engine-owned names (`points`,
`normals`, `faceVertexIndices`, `faceVertexCounts`, `extent`, `xformOpOrder`, `xformOp:*`, `physics:*`,
`physx*:*`) with the tool that owns them. `value` is JSON; pass `type` to create a missing attribute.

Measured on 6.2.0.0 and 5.1.0.0: a visual-only cube
from a USD file took a collider and rigid body, fell onto a moved table and rested; two pack-created
cubes with masses 2.5 and 5.0 reported exactly those runtime masses through `sim__body__state`
`include_dynamics`; `revert` restored the table's and cube's authored poses, and the source file's
SHA-256 was unchanged.



## Reopening a saved robot

A reopened scene does not resume where the save left it. Saving while paused keeps the link poses, so the stopped viewport
shows that pose. On Play, the joints restart from the file's authored joint state, usually all zero, and the arm moves there.
A `sim__joints__command` target is runtime state and is not saved, so command again after Play. Checked on 6.2 with a UR5e:
the wrist was at the saved pose when reopened, returned to its zero pose on Play, and reached the same target when commanded again.

If the file holds a joint state publisher and a subscriber on one topic, `sim__stage__open` adds `joint_state_loop` with the
graph paths and the fix, and does not change the file. Such an arm sits still until it gets a command, then oscillates at its
velocity limit. A graph that only publishes is not reported.

`sim__stage__open` accepts files under your home and in this simulator's own output folder, so a default `sim__stage__save`
can be reopened as is. A symlink out of that folder, or a sibling folder, still refuses with `OUTSIDE_HOME`.

## Controller runs, jobs and replay

`sim__simulation__step` advances at most 120 steps and applies no forces. To fly or drive a body with your own
control law, `sim__simulation__run` runs the loop for you, per physics step, and `sim__trajectory__replay` turns the
log into a video. Both return a `job_id` at once; wait with `job_status(job_id, wait_s=30)` and stop with
`job_cancel` ([background jobs](TOOLS-HOST.md#background-jobs)).

### `sim__simulation__run`

Each step the controller is called with the body's measured state and answers with the wrench for that step:

```text
state    {"step", "time_s", "dt", "position", "orientation_wxyz", "linear_velocity",
          "angular_velocity", "mass", "params"}            world frame, stage units, radians
command  {"force": [x, y, z], "torque": [x, y, z], "frame": "world" | "body",
          "done": false, "log": {"name": number}}          every key optional; null means no wrench
```

The wrench acts at the body's centre of mass for one physics step, in the world frame unless `frame` says `body`.
(PhysX applies a force at the body's origin by default, so the run passes it the centre of mass; Newton applies it there itself.)
`done: true` ends the run early. Numbers under `log` become extra CSV columns (fixed by the first answer).

`controller` is one of:

- **`tcp://127.0.0.1:<port>`**: your own process. The simulator connects, then sends one JSON line per step and
  reads one back. Nothing of yours runs inside Kit, so this needs no setting, and the controller can be any language.
  `twinsmith.controller_server.serve(fn, port)` is a ready-made server (one stdlib file); start it as a managed job
  with `run_script`. Only loopback is accepted unless `TWINSMITH_ENABLE_SCENE_SCRIPTS=1`.
- **`package.module:function`** or **`/abs/path/file.py:function`**: a Python callable loaded into Kit. This is
  arbitrary code in the simulator process, so it needs `TWINSMITH_ENABLE_SCENE_SCRIPTS=1` in Kit's launch
  environment, the same opt-in as `sim__controller__*`. It is not a sandbox.

```python
# hover.py, started with run_script(script="/abs/hover.py"), then
# sim__simulation__run(body="/World/Drone/base", controller="tcp://127.0.0.1:9123", duration_s=10)
from twinsmith.controller_server import serve

def hover(state):
    error = 1.0 - state["position"][2]
    lift = state["mass"] * (9.81 + 4.0 * error - 3.0 * state["linear_velocity"][2])
    return {"force": [0, 0, lift], "log": {"error": error}}

serve(hover, port=9123)
```

What the run does:

- A stopped timeline is initialized first (played briefly, then paused, so physics may advance); a paused one is used
  as it is; a playing one is refused. The run steps physics itself through the same exact-step path as
  `sim__simulation__step`, `steps_per_frame` steps per app frame (default: about real time at 60 frames per second),
  so Kit keeps rendering and answering other tools. It ends **paused**, with the body where the run left it.
- `mode="play"` follows a playing timeline instead. A stopped timeline is started, a playing one is used as it is,
  and a paused one is refused (play it first); `steps_per_frame` does not apply. The controller runs
  in the physics engine's pre-step callback, and ROS publishers, action graphs and everything else that ticks per
  frame keep running. The run ends when its duration has passed and leaves the timeline **playing**; pausing or
  stopping under it ends it with `TIMELINE_INTERRUPTED`. Use the default `mode="step"` when a run must be repeatable
  or faster than real time, and `play` when the rest of the scene has to run with it.
- The call returns `job_id`, `steps`, `physics_dt`, `mass`, `backend` and `log_file`. `job_status` reports
  `progress` while it runs and, at the end, `steps`, `sim_time_s`, `ended_by` (`duration` or `controller`),
  `log_file`, `log_rows` and `final_state`.
- The CSV goes to a new file in the output folder (`filename`, default `run-<time>.csv`; never overwritten):
  `step, time_s`, then `px,py,pz`, `qw,qx,qy,qz`, `vx,vy,vz`, `wx,wy,wz`, `fx,fy,fz`, `tx,ty,tz` for the fields in
  `log`, then the controller's own columns. `log_every` thins it.
- A controller that raises, answers with something other than a wrench, or goes silent for 5 seconds ends the job as
  `failed` with `CONTROLLER_FAILED` and the step; the log holds the steps so far. Playing or stopping the timeline
  under a run ends it with `TIMELINE_INTERRUPTED`. `sim__timeline` with `action=reset` cancels it cleanly.
- Forces go through the selected engine's rigid-body tensor view, on PhysX and on Newton. A view that cannot apply
  forces refuses with `WRENCH_UNSUPPORTED`, with no fallback to another engine.

More than one body, and joints:

- `articulation` (the articulation root, often `<robot>/root_joint`) adds `joints: {names, positions, velocities}`
  to the state. The controller may then return `joint_targets` or `joint_velocity_targets` (drive targets, radians or
  stage units, kept until changed) and `joint_efforts` (N m or N, for that step only: a joint left out gets none),
  each as `{joint name: value}`. A stiff position drive resists an effort; lower its gains with `sim__joints__gains`
  to control by effort. `body` is still required: pass the link whose pose the controller watches.
- `observe` lists up to 8 more rigid bodies. Their states arrive under `observed[path]`, and the controller can push
  them with `wrenches: {path: {"force": [...], "torque": [...]}}`.
- The CSV gains `q:<joint>` and `dq:<joint>` columns for the articulation.

Recording the run:

- `camera` (from `sim__camera__create`) saves one frame per app frame, `<log name>-00001.png` and so on, and
  `job_status` then gives `video_path`, as for a replay. `fps` is video frames per simulated second, so the video plays
  in simulated time; `label` is drawn on it. It needs no window, so it works headless and on a locked screen. At most
  3600 frames per run.

### `sim__trajectory__replay`

A replay is a picture of a run, not a simulation: the timeline stays stopped, no physics runs, the poses are
session-layer overrides, and the prims go back to where they were when the job ends.

- `trajectory` is an absolute path under your home or the output folder (at most 64 MiB). A **CSV** needs a header
  with a time column (`time_s`, `t` or `time`) and a position (`px,py,pz` or `x,y,z`); `qw,qx,qy,qz` are optional,
  and without them the prim keeps its orientation. That is the layout `sim__simulation__run` writes. **JSON** or
  **JSONL** holds samples `{"time_s", "position", "orientation_wxyz"}`, or `{"time_s", "bodies": {"/World/A": {...}}}`
  to move up to 16 prims at once.
- `prim` names what to move for a single-body file: the robot's top Xform or a free body. A jointed link is refused
  with the Xform to pass instead, as for `sim__prim__set_transform`.
- Poses are interpolated to `fps` frames per second of video; `speed` plays faster or slower, and `start_s`/`end_s`
  cut a part. At most 3600 frames per job.
- With `camera` (from `sim__camera__create`; `adopt=true` for a camera that came with an opened scene) each frame is
  saved as `<name>-00001.png` in the output folder through `sim__camera__capture`, so it works headless and on a
  locked screen. When the job has ended, `job_status` adds `video_path`: the MCP server encodes the frames with the
  bundled ffmpeg and draws `label` on every frame. The MP4 goes to `~/Videos/twinsmith/<filename>`. If the server
  cannot read Kit's output folder (Kit on another machine) the result says so and keeps the frames;
  `join_videos(mode="frames")` encodes frames from any folder.
- Without `camera` nothing is saved: the poses are shown at wall-clock pace, for `screen_recording(action="start")`. Use
  `sim__viewport` with `layout="clean"` and `caption` first for a tidy picture.

## Sensors, physics and trusted controllers

A pending Replicator render/writer Stop can outlast the timeline's first stopped frame.
An explicit Stop uses the loaded orchestrator's documented asynchronous completion API
when its status remains STOPPING. This finalizes writers, restores capture settings and
clears pending schedules; it is a mutation. In the 6.0.0.1 reopen failure, passive status
observation never completed after the dispatcher reset to zero with old capture times.
The adapter guards each continuation against stage, timeline and native-operation changes,
awaits cancellation, and allows six ordinary updates for the installed delayed settings
restoration. It does not launch an independent completion task or reissue Play.
The five-second timeout is cooperative: synchronous native calls cannot be preempted,
and native callbacks or detached tasks are not a complete ownership/lifetime fence.
A failed/interrupted Stop can have partial effects; inspect the selected stage and timeline.

Play only passively observes a pending Stop and observes STOPPED across another ordinary
update; it never calls the mutating completion API. `TIMELINE_BUSY` means completion or
observation did not settle; `TIMELINE_INTERRUPTED` reports a changed operation or requested
state that was not retained. Retry explicitly after inspecting state. Native API absence
is `TIMELINE_API_UNAVAILABLE`; a scene without loaded Replicator needs no Replicator wait.
Pausing an already stopped timeline remains an observed no-op.

Runtime APIs use PhysX or the qualified Newton adapter described below. Generic Play temporarily selects the session edit target
so native physics writeback can override session-authored spawn poses. Stop restores the prior
edit target after the normal application frame, including Stop requested through the UR5e
tools or application UI. A subsequent Play, stage change or extension shutdown cancels
pending restoration; a layer deliberately selected by the user is preserved.
External edit-target changes cause exact stepping
to refuse, and Stop does not overwrite a target chosen by another editor. First generic Play on
an externally running timeline requires an explicit Stop. This fixes the independently observed
case where PhysX moved a body but stronger session poses left its image at spawn. `sim__simulation__step` accepts an initialized **paused**
timeline and 1–120 physics steps; it checks measured simulation-time delta and returns selected
body poses/DOF observations. It does not render, pump nested application updates, or claim that
changing an authored USD transform moved an active simulated body. Angular runtime values and
drive targets use radians; USD authored joint diagnostics and angular drive gains use degrees.

If Play came from outside `sim__timeline` (the UI or a script), the step refuses with
`EDIT_TARGET_CHANGED`, `started_outside_tools: true` and the exact calls: `before_retry` is Stop and `then` is Play and
Pause. A running timeline cannot be adopted: during that Play, physics was already writing poses to the selected layer
(checked on 6.2: the wrist's `xformOp:orient` appears in the root layer while playing and is gone after Stop), so moving
writeback to the session layer midway would leave those writes behind. Stop rewinds physics.

Invalid observed body poses, velocities or joint values return `PHYSICS_STATE_INVALID`
with the backend, path and field. This reports bad runtime data, not a malformed caller
pose. A step failure after the native call starts includes a `step` object with the
requested count, timestep, observed before/after physics times (after may be `null`),
measured/expected delta, failure phase and Fabric-update state. A matching clock delta
does not prove valid physics. Other native exceptions return `STEP_FAILED`; existing GPU fault
quarantine remains `DEVICE_ERROR`. There is no rollback, reset or automatic retry.
Inspect the solver diagnostics before continuing. A preflight refusal does not claim
that steps were attempted. These error-reporting safeguards do not repair native solver
instability or establish support for every contact/solver configuration.

RTX cameras use UsdGeom, an owned Replicator render product and an on-demand renderer
CPU readback. They attach no RGB annotator. Removing that redundant branch passed an initial
6.2 headless coexistence control, but later fresh camera-only and LiDAR-only controls still
reproduced paused native ROS RGB writer cycle warnings. That issue remains unresolved;
successful image/scan output does not establish that the rejected native connections are harmless.
The earlier experimental
CameraSensor path crashed in native CUDA; asking it for a CPU result at retrieval was
insufficient. Cameras do not require `isaacsim.sensors.experimental.rtx`.
New cameras use near/far clipping distances of 1 cm and 1 km, converted using
the stage's `metersPerUnit`. This avoids the USD fallback near plane of one
stage unit hiding nearby objects in meter-based scenes. Creation returns
`clipping_range_m` and `clipping_range_stage_units`; invalid or unrepresentable
stage units refuse before a camera or render product is created. Existing and
reopened cameras retain their authored clipping settings. Position and target
coordinates still use stage units; image content also depends on aim, lighting
and scene visibility.
The generic LiDAR uses the installed `IsaacSensorCreateRtxLidar` command and Cartesian CPU
annotator, including the older 6.0.0.0 API. A `config` LiDAR uses NVIDIA's 6.x
`Lidar.create(usd_path=<assets root>/Isaac/Sensors/..., tick_rate=...)` with the same annotator.
On 6.2.0.0 an `Example_Rotary_2D` LiDAR at `/turtlebot3_burger/base_scan/Lidar2D` returned
points through `sim__lidar__read`, and a `ROS2RtxLidarHelper` (`type=laser_scan`) published
3600-beam `/scan` messages, both with Motion BVH off. The helper has no `targetPrim` on any build we inspected, including 5.1 and 6.2. Wire
`OnPlaybackTick`'s `outputs:tick` to `IsaacCreateRenderProduct` (`cameraPrim` = the LiDAR), then to the helper's `renderProductPath`, which gave the same `/scan` on 6.0.0.0 through 6.2. `sim__lidar__create` returns this graph as `ros2_scan_graph`, ready for `sim__graph__build`, and its `cameraPrim` is the returned `sensor_path`: NVIDIA's config assets
can nest the OmniLidar below your `path` (`RPLIDAR_S2E`), and a product aimed at the wrapper only logs `not attached to RTX Lidar`. It also returns the tool's own `render_product`. On 6.2.0.0 and 5.1.0.0 a helper wired directly to it published too, but saves drop that product, so such a graph goes silent in a reopened file. On 6.0.0.0 that wiring advertised `/scan` with one publisher and sent nothing, while `ros2_scan_graph` published within a second, so use `ros2_scan_graph`.
One run on 6.1 saw 360 values of `-1.0` from that direct wiring until `sim__lidar__read` had run; that did not reproduce on 6.2.0.0, and 6.1 is untested here. The full build, running Kit and loaded extensions must match
a qualified stack; missing/conflicting identity and unknown builds fail before native creation.
`sim__stage__info.runtime` reports the actual package, full VERSION, Kit, extension, Warp,
solver and adapter information. Adding a release requires qualification evidence and a data
entry in `sim_versions.py`, not a major-version guess.

Motion BVH (`launch_simulator(enable_motion_bvh=true)` or `run.sh --motion-bvh`) is optional for
LiDAR: it models motion distortion within one sweep. Without it, 6.0.1.0 logs `Multi-tick is enabled but motion BVH is not active`
and `incorrect point cloud without motion effects`. That refers to this distortion; the static scans we measured were correct.
It stays explicit because it consumes resources and affects the whole process; RTX radar requires it.

`Example_Rotary_2D` scans a flat plane with a 1 m near range, so `NOT_READY` usually means no surface lies on that plane between 1 m
and its far range. On 6.0.1.0, at `base_link` of the UR5e workstation, every ray started inside the robot base or level with the bench
top and it read `NOT_READY`; raised 0.30 m it read 33,072 points, and a minimal stage behaved the same way. Mount it above what it
sits on, with a target at least 1 m away.
Create sensors stopped, explicitly play for warm-up, then pause to read completed output.
Camera capture normally waits up to five seconds for four matching Hydra drawable events
and a new renderer-managed CPU pixel transfer (`hydra-renderer-cpu-v1`). Advancing render
counters alone did not prevent a stale Replicator pointer after paused physics steps on
6.0.0.1. The default now bypasses that cached pointer, bounds and copies the callback buffer,
and returns `cpu_transfer_completed=true`. Timeout/stage change returns no image; a pending
native transfer prevents another request until completion. It does not step the Replicator
orchestrator or change timeline state. `wait_for_render=false` writes a PNG from the last
validated **tool-acquired** CPU frame, without queuing rendering, a transfer or an annotator.
It returns `NOT_READY` until a default/true capture has seeded the cache. Ordinary Play does
not seed or refresh it. This replaces the previous continuously updated Replicator-buffer
semantics. The response reports cache age and the acquisition's render/timeline observations
separately from this call; observed render numbers are not an exact callback-image ID.
Cached pixels may predate Stop/reset, pose edits or physics steps. A failed waiting capture
still fails; only an explicit nonwaiting call may reuse an older valid frame. Cache commit
occurs after final acquisition validation, before PNG publication: an output filename conflict
can therefore leave a new valid cached acquisition while reporting file failure.
Kit's legacy viewport gizmos draw the grid and axis lines into every render product while the timeline is not playing,
including a capture's own, and only a process-wide setting controls them. A fresh capture on a stopped or paused timeline therefore
turns `/app/viewport/grid/enabled` off for that capture and restores it, so a windowed viewport's grid blinks off briefly.
Measured on 6.2.0.0; a cached read (`wait_for_render=false`) and a playing timeline change nothing.
Each camera retains one read-only RGBA8 frame, discarded on sensor/stage teardown and never
reused by a new sensor at the same path. Eight 1024×1024 caches retain at most 32 MiB of pixel
data; converter lists, in-flight transfers and PNG/RGB copies are additional memory.
LiDAR acquisition is unchanged and defaults to the latest
completed scan: its acquisition clock requires Play, and requesting a fresh render while
paused returns `SENSOR_PAUSED`. Scan tick rate matches scan rate; accumulation is enabled.
Render completion is **not** an atomic physics/sensor snapshot or proof of a fresh rotating
scan. Camera sizes are 64–1024 pixels per side, at most eight live sensors; LiDAR returns at
most 256 finite nonzero XYZ points with sensor-to-world matrix, in meters.

`STALE_TARGET` includes `current_target` with the current session/stage IDs, PID, layer
identifier, default prim, units and up-axis. Review that identity and resend the chosen IDs
explicitly. A refused call never silently switches targets or performs the requested edit.

Delete an exact owned sensor path while stopped to detach its annotator and release its
render product. Clear/subtree deletion still protects sensors. Cleanup is bound to the
original stage and creation layers; foreign composition opinions cause a partial-cleanup
error instead of cross-layer deletion. Stage closing and extension shutdown release resources.
If an editor or another extension removes the sensor's USD prim, its registered resources
remain protected: creation at that same path returns `CONFLICT` rather than overwriting
the cleanup record. Use a new stage to release those resources before reusing the path.
Reported CUDA illegal-address errors latch `device_error`: runtime/sensor operations refuse
until process restart. A native crash may bypass that guard. Inspect, save when possible,
Stop, and restart only the owned disposable process. See the compatibility follow-up evidence
for measured outcomes and retained earlier failures; no indefinite lifetime guarantee is made.

Managed launches can select `physics_backend="newton"` or `"physx"` (the default is stated in
[compatibility](COMPATIBILITY.md#target-engine)); an unavailable requested engine fails instead of falling back. Newton uses its tensor adapter, explicit
paused solver steps and Fabric publication; it does not report authored USD as runtime pose.
On Sim 6.2 one GPU Newton exact-step probe returned `STEP_TIMING_MISMATCH`; inspect state and
do not assume exact stepping succeeds on every Newton scene. See [compatibility](COMPATIBILITY.md#target-engine).
The likely cause is two timesteps on one scene; see [Newton in the generic tools](#newton-in-the-generic-tools).
The older three releases need a scoped internal playing-flag adapter, restored even on error.
Direct Newton steps invalidate captured CUDA graphs to avoid stale dt/state buffers.
Newton requires meter stages and a qualified extension/library stack. Paused saves copy up to
256 runtime rigid-body poses into a private flattened export; the live/source USD stays unchanged.
This is a pose snapshot, not a solver checkpoint. Runtime mass/COM velocities are optional via
`sim__body__state(include_dynamics=true)`. Angular velocities are radians/s; mass includes kg
conversion. Validation is scoped to the recorded Newton/MuJoCo and PhysX configurations.

Controller tools execute arbitrary Python through ScriptNode and require the user's launch-time
`TWINSMITH_ENABLE_SCENE_SCRIPTS=1`. That explicitly enables Kit's ScriptNode consent setting;
it is not OS isolation. By default both tools return `SCRIPT_DISABLED`, and `sim__graph__build` refuses any node that takes `inputs:script` or `inputs:scriptPath` the same way. A file you open can still hold a ScriptNode; opening a file is trusting it, as Kit's own consent setting applies. The graph topology is
fixed to playback tick → inline ScriptNode; update recompiles owned code while stopped.
Controller USD edits must respect layer strength (for pack-authored geometry, use its session
layer). General graph topology and file/module hot-reload are not implemented. Existing trusted
`run_python` is unchanged, and is never used as a substitute in the capability mapping.



## URDF robots

From a public URDF to a moving joint, with these tools alone:

1. `assets__pack_build` copies the URDF and what it names into a new pack version. See [Asset packs](ASSET-PACKS.md#the-tools).
2. `sim__quick_start robot="<pack id>/<version>/<file>.urdf"`, or `assets__import_path source_path=<file>.urdf` in a scene you have.
   Pass `fixed_base=false` for a robot that drives or walks.
3. Play, then `sim__joints__state` and `sim__joints__command` on the reported `articulation_root`; `sim__joints__inspect` takes that same path (a link carrying the articulation API also works) to show authored limits and drives.

Both importers are supported: the 6.x `URDFImporter` and the 5.x `URDFParseAndImportFile` command. They were exercised on the builds listed in [compatibility](COMPATIBILITY.md). Files are resolved against a verified snapshot, and a rewritten URDF
with absolute paths into it goes to the importer. It stays in private staging. `mapped_files` and `packages` report the
mapping. Conversion writes service-owned derived files, the active root and session layers are compared before and after,
and a repeat import of the same file, options and importer reuses the conversion.

Each robot and option set uses two of the 256 snapshot slots in staging, one for the URDF and one for its conversion. No tool cleans them:
at `QUOTA`, stop the simulators using them and clear the staging folder yourself.

What the import does, and why:

| It does | Because | Turn it off |
|---|---|---|
| Authors position drives on each moving joint, and reports them under `converted_physics.drives` | A URDF has no position gains, so a 6.x conversion has stiffness 0 and a target does nothing | `default_drives=false`, then `sim__joints__drive` |
| Raises the robot so its lowest point is on a collision ground plane, and reports it in `placement` | A robot whose origin is its body starts partly under the floor. With a fixed base, the contact solver then throws its joints apart | Pass `position`, which is used as given, with a warning if it is under the ground |
| Fixes the base to the world, and reports `base` | This is what both importers do by default | `fixed_base=false` |
| Selects the `physics` variant (6.x) or `PhysX` (5.x) on the generated copy | They are the descriptions that hold the joints and masses | - |
| On 5.x, gives a private copy of a DAE with no `<unit>` the format's default of metres, and lists it in `repaired_meshes` | The 5.x converter crashes Kit on such a file | - |

The gains are upper-bound estimates from the URDF at its zero pose: stiff enough that gravity moves a joint by about 0.5 degrees,
critically damped for what the joint carries. They hold a pose. They are not a tuned controller. A continuous joint may be a
wheel, so it is sized for the whole robot. On 6.x a stiffness already in the output is kept (`kept_from_importer`); on 5.x the importer's own value is
replaced.

`warnings` names problems in the file, which the import does not change: inertia far too small for a link's mass and shape, moving
links with no mass, impossible inertia, effort limits below the gravity load, and joints whose limits exclude the zero pose (they jump on Play, and a
free-standing robot can fall over). `sim__joints__state` warns if joints go past their limits at run time.

**Refusals:** `XACRO` (expand it yourself, the message has the command), `PACKAGE_UNRESOLVED`, `PACKAGE_AMBIGUOUS`, `PACKAGE_NAME`, `MESH_MISSING`,
`OUTSIDE_ROOT`, `UNSAFE_ENTRY` (a symlink), `UNSUPPORTED_URI` (URLs, including `file://` and `model://`), `INVALID_PATH`, `URDF_XML` (DTDs, entities, bad XML),
`PACK_CORRUPT` and `ROOT_NOT_APPROVED`. Without a pack, a snapshot copies the whole root, so a big repository refuses with
`LIMIT`: use `assets__pack_build`.

**Up axis:** both importers write Z up in metres. A Y-up stage, or other units, refuses with `UNITS_MISMATCH` and the values to use. Nothing converts it.

**Limits:** drives are position drives, so a wheel takes an angle, not a speed. To drive a wheel by speed, set its `stiffness=0` with some `damping` through `sim__joints__gains`, then give `sim__joints__command` `velocities` (see [Joint control](#joint-control-by-name-gains-settling-and-teleport)); for ROS 2, use the `/cmd_vel` graph that `sim__quick_start` returns for a wheeled robot. Stronger drives that persist need `sim__joints__drive`.
Check a URDF before launching Kit with `assets__dependencies`, which reads it on the host and reports missing meshes and packages.
A robot starts at its zero pose: no tool sets a start pose, so a robot whose legs start outside their limits falls first, and then can be commanded.
A successful import does not prove fidelity: check it with `sim__camera__capture`.

## MuJoCo (MJCF) models

Most robots in [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (the Franka, UR5e, quadrupeds and humanoids)
ship only as MuJoCo XML. `assets__import_path` brings one in with the importer Isaac Sim 6.x ships (`isaacsim.asset.importer.mjcf`):

1. Download the model folder (a named source: see [Provenance](ASSET-PACKS.md#provenance)), and have the user approve
   **that folder**: the whole folder is copied for each import, so approve the one robot's folder, not the whole repository.
2. `assets__import_path source_path=<folder>/<model>.xml prim_path=/World/<name>` in a scene you have. Leave `fixed_base` out to keep what the
   model says: an arm without a free joint is fixed, a quadruped or humanoid with one floats and falls. Pass `fixed_base=true` to pin a
   floating robot, or `false` to free an arm.
3. Play, then `sim__joints__state`, `sim__joints__command` and `sim__joints__move` on the reported `articulation_root`.

**What is checked first, on the host:** every file the model names (`<include>`, `<mesh>`, `<texture>`, `<hfield>`, `<skin>`,
`<model>`, `<flexcomp>`), found the way MuJoCo finds them (relative to the model's folder, through `<compiler meshdir>`,
`texturedir` and `assetdir`), must be inside the approved folder. An absolute path, a URL, a backslash, a symlink or a path that
climbs out of the folder is refused, so a model cannot make the importer read a file you did not approve. DTDs and entities are
refused. The import then reads only the verified snapshot.

**Refusals:** `MJCF_XML` (bad XML, a DTD, or not a MuJoCo file), `MESH_MISSING`, `OUTSIDE_ROOT`, `UNSAFE_ENTRY`, `UNSUPPORTED_URI`,
`INVALID_PATH`, `UNSUPPORTED_FORMAT` (a URDF or a USD sent to the wrong tool), `ROOT_NOT_APPROVED`, `LIMIT`, `IMPORTER_UNAVAILABLE`
(Isaac Sim 5.0 and earlier have no MJCF importer) and `MJCF_PHYSICS` (the converted model lost joints or bodies).
`assets__dependencies` lists the same closure for a `.xml` model without importing it.

What it does: selects the shared `physics` variant of the converted copy, raises a floating robot whose lowest point is under a collision
ground plane (reported in `placement`, at most 3 m), and reports `base`, `converted_physics`, `articulation_root` and the included files.
MuJoCo-only features depend on the native importer; 5.1 also repairs a narrow class of prismatic joint equalities and reports
unrestored ones. Tendon imports carry a fidelity warning. See [asset import](ASSET-IMPORT.md#physics-after-the-import).

## Reproduce

```bash
python3 scripts/make_scene_fixture.py /absolute/new-fixture
# Approve new-fixture above with --root-id fixture, launch a disposable Sim on a free port, then:
MCPPY="uv run --locked --isolated --project mcp --extra scene-test --python 3.12 python"  # pxr and numpy for the validators
$MCPPY scripts/scene_mcp_client.py --port 18861 assets__roots '{}'
$MCPPY scripts/scene_mcp_client.py --port 18861 sim__stage__info '{}'
$MCPPY scripts/smoke/validate_scene_tools.py --port 18861 --output /absolute/transcript.json
```

The validator expects the approved root ID `fixture`; it changes the selected disposable stage.
`validate_scene_equivalents.py` also needs an approved root ID `agilebot` holding the robot folder from the
[Agilebot fixture](VENDORING.md) (`gbt-c5a` at the pinned commit; no script fetches it and it is not bundled, so clone it and
approve the folder as above). It exits with that instruction when the root is missing, and has no fallback.
`validate_scene_clear.py` runs last and checks what the equivalents run left on the stage, so it needs that run first.
It uses a real stdio MCP client/server and never `run_python`. CPU OpenUSD tests require the
`scene-test` extra; they do not test live PhysX/Kit. See evidence for precise scope.

## USDZ trusted-project subset

The broker validates stored, unencrypted, 64-byte-aligned USDZ containers before native USD
parsing. It checks central/local headers, CRCs, hashes, types, duplicate/case-folded names,
relative member paths, counts and sizes without extracting files. The default member must be
USD. Nested archives, ZIP64, compression, descriptors, links, traversal and unknown member
types refuse. Common multi-file USD plus texture packages retain their package paths;
resolved package members must belong to the verified manifest. This is a bounded supported
subset of USDZ, **not OS isolation**. Native resolver/plugins still run with the trusted
process's permissions. Retain verified staging snapshots while referenced/saved outputs use them.

Saved copies omit recognized process-bound Replicator `/Orchestrator` graphs, SyntheticData
pipelines and pack-owned render products; their
native graph/CPU handles cannot be reopened as USD. Recognized ROS camera writers are omitted
only when a preserved camera helper with the same topic/frame and an authored render-product
source can regenerate them. Camera and LiDAR definitions, materials and user Action Graphs
remain. The result lists `omitted_runtime_render_paths`. Unknown content
inside a native pipeline or `/Orchestrator`, or a preserved controller connected to omitted
runtime state (including relationship targets), returns `SAVE_UNSUPPORTED` before publication;
the live stage is unchanged.
Create fresh sensors after reopening; a saved scene is not a running sensor or physics checkpoint.

The earlier 6.0.0.0 reopened-camera `NOT_READY` failure was traced to a serialized
Replicator orchestration graph suppressing recreation of its Python update subscription.
New exports omit that recognized graph; fresh-process windowed and headless MCP reopen,
body-state/step and camera checks pass. Previously exported files can retain obsolete native
state: re-export a trusted scene through the corrected save tool into a new file, then reopen
that copy. No runtime patch, implicit stage reset or private Replicator lifecycle call is used.
See the compatibility evidence for the retained failure history and tested scope.

A camera graph can create a `viewportHandle` that identifies a texture in the current
process. Serializing that integer can prevent ROS camera publishers from rebuilding after
restart. `sim__stage__save` refuses such affected saves by default and lists the properties.
An agent exporting that scene can explicitly choose `reset_camera_render_handles=true`.
This removes only validated, session-only handles associated with recognized native camera
producers/helpers from the **new copy**, and returns `omitted_runtime_render_properties`.
It preserves the camera, render product, render settings, other attributes, live layers and
source files. The choice authorizes this export normalization; it does not prove who authored
an attribute. Root/cross-layer opinions, ambiguous products, handle dependencies, incomplete
composition or unavailable/mismatched native bindings still refuse with `SAVE_UNSUPPORTED`.
Binding validation reads the already-retained texture objects through a bounded adapter for
inspected Replicator cache layouts. It does not call create/attach APIs or pruning getters.
An unfamiliar layout refuses. It never passes a USD integer to the native factory lookup:
that lookup crashed during 6.2 live qualification; its handle contract was not established.
When any active, concrete recognized producer's USD `inputs:cameraPrim` relationship is empty,
save checks it even if no ROS camera helper is connected or no viewport handle exists. It can recover the target
from the existing OmniGraph runtime value only when its runtime render-product path,
camera relationship and resolution agree. It writes the target into the private exported copy
and lists it in `restored_camera_targets`; the live stage remains unchanged. An unavailable or
ambiguous runtime target refuses with the producer and camera details. A graph that has never
run may not provide enough evidence to recover a missing target: set its target and retry.
Pause before saving an active scene; saving while playing remains refused. After Stop a
native texture may have been torn down, so inspect a refusal rather than assuming it is live.

### Newton in the generic tools

Newton (Isaac Sim 6.2, MuJoCo-Warp) is the target engine; its status and known limits are in
[compatibility](COMPATIBILITY.md#target-engine). These are the differences in the generic scene tools, read from the installed extension; the live checks that prove them are in
`scripts/smoke/newton_checks.py` (`python3 scripts/smoke.py sim --backend newton --discard-changes --check newton_checks`).

- **Contact buffers.** MuJoCo-Warp holds a fixed number of contacts (`nconmax`, default 200 per world) and constraint rows
  (`njmax`, default 1200 per world) and drops what does not fit. A 54-block tower needs far more. `sim__quick_start` (every Newton
  quick scene, and the Jenga tower in particular) sizes both from the rigid-body count (32 contacts per body, 8 rows per contact, at
  least 512 and 2048) and reports them as `newton_buffers`; `sim__physics action=configure` does the same on a stopped timeline,
  and the handler takes `nconmax` / `njmax` there as overrides (the tool contract does not list them yet). The buffers are process-local (not saved in USD) and take effect on the next Play.
- **Reading them.** `sim__physics action=inspect` adds a `newton` section: the configured buffers, the last step's active contacts and
  constraints against their capacity, and MuJoCo-Warp's overflow flags. A full buffer or a set overflow bit adds a warning to `warnings`
  with the code `CONTACT_BUFFER_FULL`, `CONSTRAINT_BUFFER_FULL` or `NEWTON_SOLVER_OVERFLOW`. The flags persist until the solver resets, so a
  set bit means the buffer overflowed at some step since. Counters that cannot be read are reported as such, never as safe.
- **Contacts.** `sim__physics action=contacts` reads Newton's contact snapshot of the last step: pairs with `contact_points` and a summed
  world-frame `force` in newtons on `collider_a` (there is no impulse; `impulse` is null). It needs MuJoCo-Warp with contact outputs on.
- **Raycast** is refused under Newton (`RAYCAST_UNSUPPORTED`): the extension's raycast handler is an empty stub and Newton 1.6.1 has no
  public ray query. Use contacts, `sim__prim__bounds` or `sim__prim__measure`, or a PhysX session.
- **Push.** `sim__physics action=apply impulse=[...]` on Newton applies `impulse / (dt / substeps)` as a force for the first substep of the
  next step (Newton has no atomic impulse call) and reports `applied_force_n` and `force_duration_s`. A zero impulse reports `applied: false`.
- **Timestep.** A MuJoCo scene has two timesteps: `mjc:option:timestep` (what `get_physics_dt`, exact stepping and controllers use; default
  0.002 s) and `newton:timeStepsPerSecond` (what Newton and the clock follow; default 1000 Hz). Authored apart they disagree, which is the likely
  cause of the Sim 6.2 `STEP_TIMING_MISMATCH` (20 steps of 0.002 s read as 0.020 s). `sim__quick_start` and `sim__physics action=configure` now
  author both from one rate, and `sim__simulation action=step` (and the joint move) refuses with `STEP_DT_SOURCES_DISAGREE` before stepping if
  they differ. The step result's `newton` section holds the cross-check and Newton's own clock before and after, so a remaining mismatch can be
  diagnosed from one call.
- **Pictures.** Newton writes poses to Fabric and leaves USD at the authored pose. `sim__camera action=capture` and `sim__lidar action=read` carry a
  `physics_render` report (Fabric updates on or off); with them off the picture shows authored poses. A paused change made other than by
  `sim__simulation` step or run is not published until the next step.
- **Occupancy map.** The start-in-collider and colliders-in-volume checks are PhysX overlap queries and are skipped under Newton; the result says so in
  `checks_skipped`. The generator itself (`isaacsim.asset.gen.omap`) was not verified under Newton.
