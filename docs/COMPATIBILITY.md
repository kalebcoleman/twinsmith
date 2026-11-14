# Compatibility

The full installer is pinned to **Isaac Sim 6.2.0.0 and Python 3.12**. The MCP host
requires Python 3.10 or newer and can use a different interpreter. The results below
are scoped observations, not guarantees for every machine, asset, tool or configuration.
The primary tested pairings are **Isaac Sim 5.1 with Isaac Lab 2.3.2** and **Isaac Sim 6.2 with Isaac Lab 3.0.0**. The intermediate 6.x versions are best effort: selected Sim checks passed, but other workflows may have issues and their Lab pairings were not tested here. For installation choices see [Linux setup](LINUX-SETUP.md#choose-a-simulator-build).

## Target engine

**Newton is the default engine for the scenes these tools build. PhysX is opt-in: choose it with `[sim] physics_backend = "physx"`,
`TWINSMITH_PHYSICS_BACKEND=physx` or `--physics-backend physx` (`run.sh`). The UR5e work cell builds on either engine (pack
`ur5e-jenga-workcell/v2`); its force/torque read (`workcell__robot(action="wrench")`) passes the static gravity check on
both. On Newton its cameras and `mirror_real` are not qualified yet, and `mirror_real` is refused there
([open item](OPEN-ITEMS.md#work-cell-on-newton)).**

This is the one place the statement is written; other pages link here. For a session on Newton:

- **Step rate.** Scenes these tools build or configure step at **500 Hz**: the quick scene and the Jenga tower author it, and
  `sim__physics action=configure` without `dt` does too (with `dt` it uses yours). PhysX rates are unchanged (60 Hz, the tower 120 Hz).
  MuJoCo-Warp does not hold a stack at 120 Hz.
- **Contact and constraint buffers.** Raised from Newton's defaults (200 contacts, 1200 rows) on every scene these tools build or
  configure by the sizing rule in `exts/sim-tools/sim_tools/newton_support.py`: 32 contacts per rigid body and 8 rows per contact,
  never below 512 contacts and 2048 rows. They are process-local; `sim__physics action=inspect` shows them against what is in use.

The rules that go with the default do not change:

- **Verify the selected engine.** `status` reports the stage's `physics_engine`. A device name does not identify the solver:
  a CPU device does not mean PhysX, and `cuda:0` does not mean Newton.
- **Never switch silently, in either direction.** A session that cannot give the requested engine starts nothing and says
  why. An open scene's engine is never changed. A request for Newton never falls back to PhysX, and a request for PhysX never
  falls back to Newton.
- **Isaac Sim 5.1 has no Newton**, so it is PhysX only; BrickSim runs there. Isaac Lab 2.x is PhysX only. Isaac Lab 3.0 chooses
  its engine from the task's own presets, which these tools do not override.

### Known Newton limits

What was found on Isaac Sim 6.2. The first three lines are results reported from live testing; the run records are not yet in
this repository, so they are **not yet verified live here**.

| Area | Result on Newton |
|---|---|
| Exact stepping (`sim__simulation(action="step")`) | Passes. |
| A one-off push (`sim__physics(action="apply")`) | Passes. |
| Camera capture after motion | Passes. |
| The 54-block Jenga tower | Holds for 10 s at 500 Hz (1.7 mm, Sim 6.2 / Newton 1.6.1); it does not hold at 120 Hz. |
| A Franka joint move (`sim__joints(action="move")`) | **Does not yet settle within the PhysX tolerance.** |
| Raycast (`sim__physics(action="raycast")`) | **Not available**: refused with `RAYCAST_UNSUPPORTED`. |
| Occupancy-map start checks | Skipped (they are PhysX overlap queries); the result lists them in `checks_skipped`. |
| Work cell (`workcell__*` tools, `sim` toolset) | **PhysX and Newton.** On Newton it authors the config's per-engine arm drives (stiffness, damping, armature) and the tower's solver settings; live-checked on Isaac Sim 6.2 / Newton 1.6.1 for hold, steps, fingers and tower. The force/torque read passes the static gravity check on both engines (`scripts/workcell_static_wrench.py`); cameras and `mirror_real` are not qualified on Newton. See its [README](../exts/workcell/workcell/README.md#newton) and the [open item](OPEN-ITEMS.md#work-cell-on-newton). |

Earlier records, kept as they were observed. Narrow 6.2 Newton experiments used MuJoCo Warp on CPU; earlier tests also produced
native crashes, invalid state, rendering and contact failures. On 2026-11-06 a 6.2 headless Newton session reported the MuJoCo
solver on `cuda:0` and a cube fell under gravity, but 20 exact steps at 0.002 s returned `STEP_TIMING_MISMATCH` (observed
0.020 s, expected 0.040 s). The observed 0.020 s is exactly 20 steps of 1 ms, Newton's default rate (`newton:timeStepsPerSecond`
= 1000 Hz), while the expected 0.040 s used `mjc:option:timestep` (0.002 s), which is what `SimulationManager.get_physics_dt()`
reads on a MuJoCo scene. The tools now author both from one rate and refuse before stepping when they differ (see
[Newton in the generic tools](TOOLS-SCENE.md#newton-in-the-generic-tools)); the exact-stepping row above is the later result.
None of this qualifies broader scene, contact, robot or Lab workflows.

The checks that remain open on Newton are in the [Newton qualification list](OPEN-ITEMS.md#newton-qualification).

Isaac Sim 6.2 loaded its bundled Warp 1.17.0 even though the Python environment also contained a different
`warp-lang` distribution. Inspect the version **loaded inside Kit**, not just `pip show`. NVIDIA notes that
its [Newton integration](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/physics/newton_physics.html) uses
release-matched dependencies; installing a different version in the same environment can make it incompatible.

A 6.0.1.0 windowed PhysX step stalled at `20f2953`. Two later scoped replays did not
reproduce it, but its cause remains unknown. If a call stops returning, follow the
[unknown-outcome and recovery guidance](TOOLS-SCENE.md#bounds-and-security-limits); a client
timeout does not cancel native execution. Clean-machine/full-installer coverage and fresh-Kit
camera reuse remain separate limits.

## Simulator builds

The 6.0 and 6.1 probes used separately resolved environments, not this project's 6.2 lock.
Further short checks are recorded in [verification](VERIFICATION.md). The 20-check smoke is scoped: it does not exercise real
hardware or establish that every viewport, ROS, desktop or Lab configuration works.
A listed tool does not establish that a compatible bridge is running.

| Isaac Sim Python package | Project status | Evidence and limits |
|---|---|---|
| `6.2.0.0` | **Claimed, with Isaac Lab 3.0.0.** Pinned baseline; selected workflows tested | The 6.2 twin, bridge, desktop controls and managed sessions were exercised on Ubuntu 24.04.5 / RTX 5080. The supported import families, the warehouse and Jenga scenes, and Lab train, resume, compare, eval and play were run on it (2026-11-05). See the [host tools](TOOLS-HOST.md) and [Lab guide](TOOLS-ISAACLAB.md#verified-vs-version-and-platform-limitations). |
| `6.1.0.0` | Best effort; partial test | Headless and windowed 20-check smoke runs completed, including a decodable recording. Simulated object and camera transforms were received. Kit logged asynchronous task errors and a viewport exception on reset. An earlier Quick Start during startup did not complete. Lab and physical hardware were not tested on 6.1. |
| `6.0.1.0` | Best effort; partial test | The 20-check smoke completed in headless and windowed sessions, including decodable recordings. Simulated object and camera transform messages were received. Kit logged asynchronous task errors; Lab and physical hardware were not tested. |
| `6.0.0.1` | Best effort; partial test | Headless and windowed 20-check smoke runs completed, including a decodable recording. Simulated object and camera transforms were received. Kit logged asynchronous task errors. Lab and physical hardware were not tested. |
| `6.0.0.0` | Best effort; partial test | The initial object-pose graph failed because this release uses an older ROS publisher. With the compatibility fix, headless and windowed 20-check smoke runs passed, and simulated object and camera transforms were received. Kit logged asynchronous task errors; Lab and physical hardware were not tested. |
| Other 6.x releases | Not tested | A shared major version does not establish that this bridge, the work cell or Lab jobs work on a new release. |
| `5.1.0.0` | **Claimed, with Isaac Lab 2.3.2.** Not auto-detected for the Sim tools, so name its interpreter with `TWINSMITH_SIM_PYTHON`; the Isaac Lab job tools can target it per call with `install` (for example `install="lab-2.3.2"`) while 3.0.0 stays the default. `run.sh` finds its bundled ROS libraries and `sim-tools` has adapters for its older APIs | Representative USD, URDF, xacro, SDF, MJCF, glTF and ROS map imports; warehouse quick start with a Carter driving 15.7 m; and Lab train, resume, compare, eval and play were exercised. On 2026-11-06, the previously failing SO-101 and Panda MJCF models imported and each moved one joint; Go2 imported with a floating base and fell under gravity. Limits: URDF export writes an empty robot on this build, so it is refused with a reason; `sim__lidar(action="create")` needs `config`; USD scenes keep their own units, so check them. Earlier: the PR 70 retest and one project scene, where most generic tools worked. Isaac Sim 5.1 has no Newton, so it runs PhysX only. The work cell's full workflow was not verified on this build. |
| Earlier releases | Not tested | No compatibility claim. |

The 6.2 lock selects NumPy 2.3.1, matching the inspected `isaacsim-kernel` 6.2.0.0 wheel.
Existing environments made from an older lock may differ; inspect them before changing anything.

## Isaac Lab, ROS and platforms

- Historical short training and windowed play checks used Isaac Lab **3.0.0 / Isaac Sim 6.2.0.0** on the machines
  described in the [Lab guide](TOOLS-ISAACLAB.md#verified-vs-version-and-platform-limitations). Those checks do not
  establish long-running training, every task, or Lab behavior on the older Sim probes above.
- The two claimed Lab pairs are **3.0.0 with Sim 6.2** and **2.3.2 with Sim 5.1**; `isaaclab__inspect(action="install")` reports `tested: true` for them and false for any other. On both, a real Cartpole job went through train, resume, compare, eval and play with `rsl_rl` through the tools; 2.3.2 also ran tiny jobs with `rl_games`, `skrl` and `sb3`. That is not proof of every task or of long training. Task ids differ between them (`Isaac-Cartpole-v0` on 2.3.2, `Isaac-Cartpole` on 3.0.0), and 2.3.2 needs `rl_library` because its scripts are per library. Lab 3 tasks can default to Newton; pass `physics="physx"` to the MCP job tool to select PhysX where the task has that preset. See the [Lab guide](TOOLS-ISAACLAB.md#isaaclab__train) for task-specific limits.
- NVIDIA's published Isaac Lab and Sim pairings are shown below. The release dates are UTC. They describe NVIDIA's
  versions, not additional MCP tests: the 6.0 and 6.1 Sim smoke checks above did **not** test Isaac Lab.

  | Isaac Sim | Isaac Lab release | Published |
  |---|---|---|
  | 6.0.0 | [`v3.0.0-beta2`](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta2) | 2026-06-17 |
  | 6.0.1 | [`v3.0.0-beta2.patch1`](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta2.patch1) | 2026-07-02 |
  | 6.1 | [`v3.0.0-EA`](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-EA) (early access) | 2026-09-16 |
  | 6.2 | [`v3.0.0`](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0) (general availability) | 2026-10-20 |

  The Lab 3.0 general-availability tag is `v3.0.0`; NVIDIA's
  [cross-product matrix](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/compatibility.html) calls it
  `v3.0.0-GA`. The first 3.0 beta, also for Sim 6.0, was published on
  [2026-03-17](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta). Select matching ROS workspaces and
  custom interfaces as described in the matrix.
- ROS 2 Jazzy simulated sensor and TF checks have been exercised on the 6.2 Ubuntu 24.04 setup; the notes below record
  Humble checks on Ubuntu 22.04. No physical robot was used. See [ROS 2 and RViz 2](ROS2-RVIZ.md) and
  [verification](VERIFICATION.md).
- The simulator installer targets Ubuntu 22.04 or 24.04 on x86_64; the primary test machine was Ubuntu 24.04.5,
  RTX 5080, NVIDIA driver 580.178.04. Desktop input was tested on X11; native Wayland and other operating systems
  have not been established as supported for these tools. See [Linux setup](LINUX-SETUP.md) and
  [NVIDIA's 6.2 system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html).

## Checking an installation

For a pip-installed Sim environment, use its own interpreter:

```bash
/absolute/path/to/sim-env/bin/python -c 'import importlib.metadata as m, sys; print(sys.version); print(m.version("isaacsim"))'
```

The bridge's `status` also reports the running app's version and PID. Its app version may omit the fourth Python
package-version segment, so use the package metadata to distinguish exact builds. For a workstation installation,
NVIDIA also documents Help > About and the installation's `VERSION` file.

An existing separate environment can be selected with `TWINSMITH_SIM_PYTHON=/absolute/path/to/sim-env/bin/python` for
`./run.sh`, or in the MCP server's environment before using `simulators(action="launch")`. Selecting an interpreter is not a version
or dependency check. Keep the locked 6.2 environment intact when experimenting, use a disposable scene and an
unused bridge port and ROS domain, and verify the selected PID before scene-changing calls. See
[host tools](TOOLS-HOST.md#optional-simulator-sessions) for managed-session safeguards and
[verification](VERIFICATION.md) for what each live check changes.

For a simulated ROS check on the tested Ubuntu/Jazzy setup, select the disposable bridge, run `sim__quick_start` with a robot,
call `sim__ros2_tf`, then play the scene. In a separate terminal, from the repository root, use the same ROS domain as
the simulator and source the shared DDS configuration:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42  # replace with the disposable simulator's domain
source config/ros_dds.env
timeout 30 ros2 topic echo --once /tf tf2_msgs/msg/TFMessage
timeout 30 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState
```

These calls change the scene and exercise simulated publishers only. Use the matching ROS distribution and
workspace for other environments; a delivered transform is not a physical-robot or camera-image test.

## Project assets and generic scenes

Trusted project USD import was exercised on the five 6.x
builds above in headless and windowed sessions, using a limited set of projects on one Linux
machine. Initial captures from newly created cameras required retries in those tests. These
results do not establish support for every asset, sensor or backend. Trusted USDZ support is
bounded, but native parsers are not an archive sandbox. Isolated untrusted import and
authenticated remote upload remain unfinished. See [scene tools](TOOLS-SCENE.md) and
[asset import](ASSET-IMPORT.md) for current contracts.

## Isaac Sim 6.2 notes for 6.0 tutorials

Course labs and NVIDIA's ROS 2 tutorials often target Isaac Sim **6.0.1** (for example the RAS545
TurtleBot3 lab with ROS 2 Action Graphs, lidar, camera, TF, RViz 2 and Nav2). This repository runs
**6.2.0**, where a few nodes and APIs behave differently. Each note below was checked live on
Isaac Sim 6.2.0 (Ubuntu 22.04, bundled Humble bridge, RoboStack Humble client, 2026-11-03) with
`run_python` and `ros2 topic echo`. The 6.0 behaviour is what the tutorials describe; it was not
re-run here.

Before building a graph, look the node up with `sim__graph(action="node_types")` (by type id or UI name): it
returns this build's exact type id, ports and allowed token values. `sim__graph(action="build")` creates a
whole graph in one call and handles notes (b) and (c) for you: it stores a token's value when you
give its key, and it creates Make Array's extra `inputs:inputN` ports. See
[TOOLS-SCENE.md](TOOLS-SCENE.md).

### (a) Constant Token has no output port

`omni.graph.nodes.ConstantToken` has a single attribute, `inputs:value`. There is no
`outputs:value`; connecting from it fails with `OmniGraphError`. Connect **from `inputs:value`**:

```python
og.Controller.connect("/Graph/frame_name.inputs:value", "/Graph/make_array.inputs:input1")
```

### (b) Scale To/From Stage Units takes the token's value, not its key

`isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit` has `inputs:conversion` with allowed tokens
`Convert to stage units` and `Convert to meters` (raw map
`{"toStage": "Convert to stage units", "toMeters": "Convert to meters"}`; the default is
`Convert to stage units`). Set the **value** string. A key such as `toStage` is not one of the allowed
values; such a wrong token makes the node output zeros without an error (found while building
`sim__graph(action="build")`; `sim__graph(action="inspect")` lists the node's compute errors). `sim__graph(action="build")` stores the value when given the key.

### (c) Make Array needs its second input created

`omni.graph.nodes.ConstructArray` ("Make Array") starts with `inputs:arraySize`, `inputs:arrayType`,
`inputs:input0` and `outputs:array`. `inputs:input1` (and later ones) does not exist until you
create it; connecting to it before that fails:

```python
og.Controller.create_attribute(node, "inputs:input1", og.Type(og.BaseDataType.TOKEN),
                               og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
```

`sim__graph(action="build")` creates these inputs itself.

### (d) The 2D RTX lidar

Create the lab's 2D rotary lidar with the experimental RTX sensor API:

```python
from isaacsim.sensors.experimental.rtx import Lidar
lidar = Lidar.create("/turtlebot3_burger/base_scan/lidar", config="Example_Rotary_2D")
```

It authors an `OmniLidar` prim (applied schemas include `OmniSensorGenericLidarCoreAPI`). When the
config asset nests the sensor under a reference root, the transform goes on that root so the
sensor keeps the vendor mounting offset. Lidar acquisition needs Play. Motion BVH (`--motion-bvh`) is optional: without it scans are
correct, but distortion from motion within one scan is not modeled.

### (e) ROS 2 Publish Transform Tree: `IsaacComputeTransformTree` is preferred, not required

In 6.2, `isaacsim.ros2.bridge.ROS2PublishTransformTree` has two input paths:

1. **Prims only (6.0 tutorial style):** set `inputs:targetPrims` (and optionally
   `inputs:parentPrim`) on the publisher. This still works in 6.2: with the TurtleBot's
   articulation root `/turtlebot3_burger/base_footprint` as target and `/World` as parent, `/tf`
   carried the whole tree (`World -> base_footprint -> base_link -> base_scan, caster_back_link,
   wheel_left_link, wheel_right_link`), about 93 messages per second. Kit logs once:
   `OgnROS2PublishTransformTree: using targetPrims for internal computation is deprecated. Connect
   OgnIsaacComputeTransformTree to inputs:parentFrames/childFrames/translations/orientations instead.`
2. **With `isaacsim.core.nodes.IsaacComputeTransformTree` upstream (the 6.2 path):** tick ->
   compute `execIn`, compute `execOut` -> publisher `execIn`, and connect `outputs:parentFrames`,
   `childFrames`, `translations` and `orientations` to the publisher inputs of the same names.
   Set `targetPrims` (and `parentPrim`) on the **compute** node. The publisher uses these arrays when
   `parentFrames` is non-empty at its first compute.

So both layouts work: TF works without the compute node, with a deprecation warning,
and the compute node is the supported 6.2 layout. One difference was measured on this scene:

| Compute node `targetPrims` | Frames on `/tf` |
|---|---|
| `/turtlebot3_burger/base_footprint` (the articulation root link) | only `World -> base_footprint` (repeatable after Stop/Play) |
| `/turtlebot3_burger` (the robot prim above it) | the full link tree, as with path 1 |

With the compute node, target the robot's top prim, and check the frames with `ros2 topic echo
/tf` or `scripts/ros2_tf_doctor.py`. With `parentPrim=/World` the compute path also logged
`[PoseTree] parent getObjectType eInvalid for '/World'` each frame while still publishing `World`
as the parent; leaving `parentPrim` empty (World) avoided the warning.

`sim__ros2_tf` builds its own TF graphs and reports the frames; prefer it over hand-built
graphs unless the lab asks for its own graph.

### Other things labs run into

- The `PoseTree` warning naming a camera can repeat every frame. It is harmless, but it floods the log.
- A flattened checkpoint is large (about 97 MB for the RAS545 scene), because saves flatten everything.
- A LiDAR topic missing after reopening a saved scene has a known cause and fix; see [saved LiDAR scenes](TOOLS-SCENE.md#saved-lidar-scenes-and-scan).
- In the RAS545 starter, the ground plane is at z=-0.75, and it reports `composition_errors: 2` on opening. Neither is a fault of the tools.
- Lab instructions that assume ROS domain 0 need adapting if another simulator holds it. A saved graph can author its own domain: a `ROS2Context` node with `useDomainIDEnvVar` off publishes on its authored `domain_id`, whatever `ROS_DOMAIN_ID` and `sim__stage(action="info")` say, so a shell on another domain sees no topics. Lab files were seen authoring different domains, so check the graph; `sim__graph(action="inspect")` adds `ros_domain_note` when they differ.
- OpenCode Code Mode has no timers, so agents use shell sleeps, and tool results arrive as JSON inside a `result` string.

### ROS 2 distro

Isaac Sim 6.2 bundles both Humble and Jazzy bridge libraries. `run.sh` picks Humble on Ubuntu
22.04 and Jazzy otherwise, unless `ROS_DISTRO` is exported. Use the same distro for RViz 2, Nav2
and the `ros2` CLI: with `config/ros_dds.env` (Cyclone, loopback only), Jazzy -> Humble delivered 0 of 8
messages in a test on an Ubuntu 24.04 desktop (observed on that desktop; not re-run on 22.04). See
[ROS2-RVIZ.md](ROS2-RVIZ.md#installing-ros-2-for-rviz-2-and-the-doctor).
