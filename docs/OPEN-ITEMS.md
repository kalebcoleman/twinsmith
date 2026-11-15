# Open items

The unfinished work from the old working checklist, grouped by topic so each group can become an issue. Check an item only
after verifying it against the current code and the relevant Sim and Lab build. Current behaviour and tool arguments are in the
[documentation index](README.md).

Keep the MCP focused on reusable capabilities. Small Jenga and warehouse quick starts remain; substantial tasks, policies,
missions, checkpoints and run data belong in separate projects. Preserve existing tool contracts and user assets when making
changes. Do not start or stop unrelated jobs; record the source, versions, backend, observed effects and limits of a test.

## Newton qualification

Newton is the default engine for the scenes these tools build and PhysX is opt-in; the statement and the known limits are in
[compatibility](COMPATIBILITY.md#target-engine). Each result records the engine, the Isaac Sim build and the loaded Newton
version.

- [x] Hold a full 54-block Jenga tower for ten seconds on Newton with the contact-buffer check on (500 Hz; 1.7 mm).
- [ ] Make a Franka joint move settle within the PhysX tolerance on Newton.
- [ ] Decide `sim__physics(action="raycast")` on Newton: implement a query, or keep it refused and say so.
- [ ] The UR5e work cell on Newton: it builds and holds, and the force/torque read passes the static check; the cameras and `mirror_real` remain (see [Work cell on Newton](#work-cell-on-newton)).
- [x] Record the live runs that show exact stepping, the one-off push and camera capture after motion passing on Newton:
  [recorded 2026-11-14](VERIFICATION.md#headless-newton-check). To repeat it, run `smoke.py sim --backend newton --discard-changes
  --check newton_checks` (the [command](VERIFICATION.md#test-matrix) needs the `mcp/` environment). If a step-timing mismatch remains, read `newton.clocks_before`,
  `newton.clocks_after` and `newton.timestep_check` in the step error. Check solver steps, clock and body state independently
  before changing the timing guard.
- [ ] Qualify the Isaac Sim, Newton, Warp, MuJoCo and GPU/driver versions that actually load. Document installation and
  diagnostics without replacing the simulator's matched packages. Inspect the version loaded inside Kit, not `pip show`.
- [ ] Exercise GPU Newton with representative rigid contacts, articulations, control, sensors, headless and windowed scenes,
  longer runs and the matching Isaac Lab version. Compare measured results with PhysX, including failures, and verify the
  actual solver device rather than CUDA availability alone.
- [ ] Identify any agent-facing Warp or Newton tools from real workflows and verified gaps. Reuse the existing scene, physics
  and Lab tools where they suffice; propose additions separately with clear inputs, device selection, resource limits and
  measured behaviour.
- [ ] Per-engine friction and solver profiles for the work cell, tuned on each engine (the current values are provisional).
- [ ] The sensors smoke check returned `PHYSICS_NOT_READY` once when it ran straight after the Newton checks in the same Kit;
  it passed on a fresh session. Reproduce it and find the cause.
- [ ] Run the scene reopen smoke check on both engines. It needs a saved scene opened by hand and has not been run yet.

### Work cell on Newton

The work cell builds on Newton since 2026-11-14. Pack `ur5e-jenga-workcell/v2` fixes the asset side (the cell now uses `ur5e-workcell/v3`: the RG2, Quick Changer and Bota from the vendors' own CAD, the real wrist camera bracket, and the bench split into the static bench and the rail's carriage; the Jenga block is the task pack `jenga/v1`):
- a box collision layer for the bench;
- the RG2 in the arm's articulation, with mimic fingers and no zero-force drives.

The config's `newton` section gives the per-engine side: arm drives with armature, 500 Hz, and the tower's solver settings. The
live results for both engines are in the [work cell README](../exts/workcell/workcell/README.md#newton).

Done:

- [x] Bench collisions within Newton's contact buffer: 385 boxes instead of 1.63 M triangles, with no overflow.
- [x] One articulation; no zero-force drives; the RG2 fingers move, the mimic followers within 0.003 degrees.
- [x] Stable arm drives on MuJoCo-Warp at 500 Hz. At the presentation pose with gravity on, every joint holds within 0.05
  degrees, and 0.05 rad joint steps settle in 0.2 s. The pack's PhysX gains went non-finite there.
- [x] The tower in the cell: at most 0.54 mm over 11 s after a 1.2 s settle.
- [x] `workcell_smoke.py` takes the session's engine. The host tests cover the engine gating, the `newton` section's
  validation and the per-degree drive authoring. The Newton build itself is covered by the live run, not the host tests.

Still open:

- [x] The flange force and torque read on Newton: `newton_cell.prepare` enables Newton's `link_incoming_joint_force` output
  before initialization, and `workcell__robot(action="wrench")` reads it (the static check passes on both engines). The
  default source is the UR5e's own flange sensor (`ur5e_tool_flange_sensor`); the Bota's reading only with `bota.mounted`.
- [ ] The finger contact reads on Newton: they need Newton's contact snapshot instead of the PhysX contact view, and are
  refused there until then rather than inventing values.
- [ ] `mirror_real` (robot-only gravity off) on Newton. It is refused today. Newton's PhysX resolver may read
  `physxRigidBody:disableGravity`, which needs a live check.
- [ ] Per-engine friction profiles in `lab_hardware.json` (unconfirmed; they start equal to the PhysX values).
- [x] Measured finger width on Newton. The RG2 has one driven joint (`finger_joint`) since pack `v3`: 100.68 mm open after
  1.44 s and 0.38 mm closed after 1.68 s ([work cell README](../exts/workcell/workcell/README.md)).
- [ ] The same stroke on PhysX, and a grasp on PhysX: not measured with pack `v3`.
- [x] The static check at rest (flange wrench equals the weight below the sensor) on both engines: `scripts/workcell_static_wrench.py`
  passes on PhysX and Newton (2026-11-14, pack v3: 9.8492 N for the 1.004 kg below the flange; with `bota.mounted`, 10.5065 N
  at the Bota and 12.2723 N at the flange). It checks the modelled masses, not the real ones: those still need weighing.
- [ ] Renders (cell, wrist) on Newton, kept in the repository. The overhead camera is not built (`overhead_camera.enabled`).
- [ ] The untouched tower creeps on Newton. Measured 2026-11-15 (Isaac Sim 6.2.0, pack `v3`, arm still, after a 3 s
  settle): the top layer of the stock `jenga_tower` task moved 0.75 mm in 20 s (0.04 mm/s, in a steady direction, the
  lean growing 0.009 degrees per second); with the per-block spread of the first real-to-sim run 0.9 to 1.0 mm in 20 s.
  The earlier figure (0.54 mm over 11 s) is of the same order. At this rate a tower leans about half a degree a minute
  and will not stand through a game; every before-and-after measurement of a tower in the cell contains some of it.
  Find which contact setting it follows (the friction cone's softness, `impratio`, the iteration count) before using
  the cell's tower for anything slower than a single move.

### Work cell realism

Datasheet values are in `lab_hardware.json` with their sources (`inertials`, `bota`, `quick_changer`, `rg2`); these need the
real cell:

- [ ] Weigh the Quick Changer (both sides), the RG2, the D415, the printed camera bracket with its bolts, and a few Jenga
  blocks. The Bota is not mounted (`bota.mounted` is false). With the switch on, its two adapter plates are labelled
  placeholders (63 mm across, 8 mm thick, 67 g each): measure and weigh the real ones when the sensor goes on.
- [ ] Settle whether the RG2's datasheet 0.78 kg includes the Quick Changer tool side (0.14 kg); the cell takes it
  to be included and adds only the robot side (0.06 kg).
- [x] The wrist D415 and its printed bracket are on the RG2 body, with mass (0.064 kg and 0.100 kg) and convex colliders.
- [ ] The wrist camera's pose is provisional: one photo and two corrections by eye. The model is still 45 to 80 mm too low and
  its 39 degree tilt is derived, not measured. Replace it from better photos or the bracket's CAD file.
- [x] Vendor meshes in place of the placeholder boxes: the RG2, Quick Changer and Bota from the vendors' STEP files, the D415
  from RealSense's mesh.
- [ ] Measure the gap between the RG2's fingertips fully open (the model gives 100.8 mm, the datasheet 110 mm) and the robot
  base's height above the table top (80 mm is the bench CAD's).
- [ ] Tilt-test friction for block on block, block on the HDPE top and fingertip tape on block, and enter them with their spread.
- [ ] The UR5e base mass (4.0 kg) is UR's own "might be incorrect" figure; the base now rides the rail, so it is
  part of what `rail_joint`'s drive carries.
- [ ] Fit the arm's joint damping and armature to real joint logs with `sysid`; datasheets do not give the drive behaviour.

### Work cell rail

The Vention rail under the robot is a seventh axis (`rail_joint`) since pack `v3`; the arm's inverse kinematics stays
six-joint from the UR base. See the [work cell README](../exts/workcell/workcell/README.md#what-changes-for-callers).

- [ ] Read the real axis from the MachineMotion controller or the cell: travel between the sensors, soft limits, which end
  is home, which way positive runs, speed, acceleration and where the carriage rests. The simulated values (1270 mm, home at
  the -X end, 0.25 m/s, 0.5 m/s2) are Vention's figures and assumptions.
- [ ] `workcell__robot(action="rail")` sets a target and returns, and Newton applies no joint velocity limit: a single large
  step moves the carriage far faster than the real axis. A move at the configured speed that waits for arrival is not provided.
- [ ] Coordinated seven-axis inverse kinematics is not provided: move the rail first, then the arm.
- [ ] Validate the `rail`, `fingertips` and `looks` sections before the stage is replaced; today a bad `rail` section fails
  inside the build, after the old stage is gone.
- [ ] Host tests for the rail tool action (its three refusals, read against command, the velocity's sign).
- [ ] `sim__arm` falls back to the authored base pose, with only a note, when the engine cannot report the base. With the
  rail away from zero that would solve from the wrong base: refuse instead when the base is not fixed to the world.
- [ ] `scripts/workcell_rail_acceptance.py` records the flange force, the chain's base and the slip during the lift without
  asserting them, counts any bridge error on the first reach as "not reachable", and assumes a 500 Hz step. Its console lines are cut
  mid-record, so the figures are only readable with `--out`: print the short table at the end.
- [ ] Not run: the acceptance scenario and a grasp on PhysX; `rail.moving` false with the split bench in the simulator;
  `sim__joints` `move`, `drive`, `gains` and `inspect` with `rail_joint`; `mirror_real` with the rail.

## Real-to-sim with RoboSimStudio

The `robosim` tool reproduced a real push on the GPU (Newton's MuJoCo-Warp solver) and on the CPU: the block's image displacement
matched the clip within the tool's tolerance. What it does not do yet:

- [ ] Match the block's rotation. Only the displacement of its centre is tuned and judged; the overlap at the end frame was 0.39.
- [ ] Fit mass and friction. The push path is tuned; the physics parameters are not. The [`sysid`](SYSID.md) tool fits
  them from recorded trajectories and recovered a synthetic pendulum's damping and armature, but no real dataset has been
  fitted yet: the block needs object poses from the push recordings.
- [ ] Estimate the camera pose from the clip. The simulated view is set by hand, so only the block's mask is compared.
- [ ] Take the pusher's path from the clip. `tune_push` searches a straight two-point path.
- [ ] Measure the tower template's contact values on the lab's blocks. Its friction, damping ratio and mass are one
  stock clip's best point ([where they come from](ROBOSIM.md#where-the-defaults-come-from-and-what-they-are-worth)):
  that clip bounds friction from below only and does not identify the mass. Weigh a block, do a tilt test on the bench
  panel and on another block, and record a block being pushed out of a tower in the cell.
- [ ] Give the tower template a tuning action. A search is a script that drives the scene's `rollout`; through the tool
  each change is an `update_scene` and a `rollout`.

## Playing Jenga with the robot

One robot turn and one person's turn run in simulation
([the results](../exts/workcell/workcell/README.md#one-jenga-turn-look-predict-push-look-again)). What a game still needs:

- [ ] A pusher that touches one block. One RG2 fingertip is 11.4 mm high in a 14 mm slot and, in the cell, dragged the
  block under the pushed one and the tower above with it. A thin poker on a finger, or drawing the block out from the
  other side with the gripper once it stands proud, has not been tried.
- [ ] A lookahead that predicts the cell. It pushes with an 8 mm rod and stiffer contacts and was wrong about the one
  push made by a factor of four to five; give it the fingertip's shape and the cell's contact settings, then test its
  ranking over several pushes.
- [ ] Pulling a block out and putting it on top: grasp, withdraw, place. Only a push of 25 mm was made.
- [ ] Choosing the block: the candidates are given by hand. No rule for which blocks are loose, no game state (whose
  turn, which blocks are legal), no decision layer (the lab's VLM planner is not connected).
- [ ] The rescan after a turn that leaves the tower leaning, twisted or partly fallen; the one tried left it standing.
- [ ] Anything on the real robot: the scripts talk to the simulator bridge only.

## Perception for the Jenga task

The [`sam3`](SAM3.md) tool finds the blocks in a video or a recorded RGB-D clip and gives each visible block a pose from
depth. Checked on two stock clips and on the simulated wrist camera. What it does not do yet:

- [ ] Read a camera, a ROS topic or a bag. A recorder has to write the clip folder first; none exists for the real D415.
- [ ] Run on a real D415 recording. The simulated depth has no noise, no holes and no minimum distance.
- [ ] See the blocks a face hides. From one side, two of every three blocks in every other layer are not reported; the
  robot loop assumes they are there.
- [ ] SAM 3.1 loses the blocks in a moving close-up: in the wrist camera's clip of a push it went from 44 blocks to 1
  where SAM 3 kept 45 to 47 ([the comparison](SAM3.md#sam-31-against-sam-3)). Looks from a still camera are unaffected.
  Either look only from rest, or find what in the multiplex tracker lets go.
- [ ] Run the two Jenga scripts live with SAM 3.1. Their recorded clips were put through it; the simulator runs
  themselves were made with SAM 3.
- [ ] Drop the driver's `start_session` shim when upstream fixes its issue 544, and move the pin.
- [ ] Keep the model loaded between looks. A look is a 20 s job of which 2 s is tracking.
- [ ] Validate the hand-at-the-tower signal (`sam3` zones) on real footage of a hand at the lab's tower. It is covered
  by unit tests only; the human-turn script fakes it with a timer.
- [ ] Calibrate a second camera without markers, against the robot or against the table and tower
  ([what it takes](SAM3.md#a-second-camera-and-its-calibration)). No routine exists; the simulated one uses its true pose.
- [ ] Read a face seen in part correctly. A block's long side seen in part is taken for an end, which puts its centre
  tens of millimetres off; single views had one to three of these, and the fusion leaves them out only because other
  views disagree.
- [ ] Give the pack's Jenga block a look a camera can use. It is a plain one-colour box, and a tower of them renders as
  a featureless slab that SAM 3 returns as one object. The robot loop draws each block 1.5 mm smaller in its own shade,
  on the stage only; a block asset with rounded edges and wood grain would replace that.
- [ ] Make `report` say when the file is complete; it returns `running` and the caller cannot tell.
- [ ] The RoboSimStudio viewer listens on every network interface and has no localhost-only option at the tested pin
  (still none at `ba3650c`).
- [ ] A hand-written scene that keeps RoboSimStudio's default physics runs free bodies on its VBD solver on a GPU, which
  ignores the pusher. The tool reports this, but only the built-in template selects MuJoCo by itself.
- [ ] Repeat the GPU and CPU comparison over more clips and seeds; one pair was run for the T block.

## Release and compatibility

- [ ] Recheck guards and Jenga on both supported builds after the latest changes. Report failures separately; a passing host
  suite is not a live Sim test.
- [ ] Investigate the original 6.0.1 windowed PhysX step stall and fresh-Kit camera reuse; later non-reproduction is not a fix.
- [ ] Validate clean-machine setup, the full installer, external Lab discovery, and the supported ROS and display combinations
  before making broader compatibility claims.
- [ ] Re-run BrickSim on Isaac Sim 5.1 after the rename; it was skipped because its launcher has no headless option on the
  test PC.
- [ ] Hosts: the roadmap's [hardware and hosts](ROADMAP.md#hardware-and-hosts) section is a first statement; refine it as we go
  (measured GPU memory and run times per phase, booking the GPU around Jenga games, the home PC's row).
- [ ] Run the work-cell smoke once from the new workspace layout (`<home>/robotics-workspace` with `assets/`); the work cell's
  asset-root lookup for it is covered by unit tests only.
- [ ] When a release is requested, review licensing and notices, name availability and PyPI installation using
  [releasing](RELEASING.md) and [provenance](VENDORING.md). A release states the engine and Isaac Sim build it was qualified on.

## Robotics and learning

- [ ] Jenga: define push and extraction, tower-stability and settling thresholds; evaluate scripted and learned policies across
  seeds and held-out geometry, poses and friction. Diagnose the failed pull before attempting grasp, extraction and a complete
  turn.
- [ ] BrickSim: reproduce expert and learned-policy evaluations across seeds with equal completed episodes. Compare with and
  without the simulator-only connection signal; measure and enforce realistic joint speeds before making transfer claims. Test
  varied brick poses and sizes.
- [ ] Mobile mission: reproduce station-to-station requests in fresh sessions, including blocked and unreachable goals. Record
  arrivals, retries and failures; label simulated loading and unloading.
- [ ] Preserve project checkpoints and configurations outside this repository. For demonstrations, record seeds, environment,
  backend, success denominators, failures and limitations; label scripted versus learned behaviour and simulation-only
  observations in any media.
- [ ] Add or improve a general public gripper workflow and command by width, force or contact. Assess suction or temporary
  attach and detach separately and label non-physical behaviour.
- [ ] Review remaining IK needs: collision awareness, straight-line behaviour and non-metre orientation weighting. Investigate
  smooth trajectories during play and policy velocity control.
- [ ] Extend live body interaction beyond the existing impulse option where needed; define PhysX and Newton behaviour and
  validate effects with a disposable scene.
- [ ] Consider sequential RL sweeps and add an `rl_games` preflight for minibatch divisibility and short runs that do not save
  checkpoints. Verify job-side timeout and exit-code reporting.

## Assets and workspace

- [ ] Investigate named presets and project asset discovery without widening approved roots. Keep format support on the existing
  import tool; see [asset import](ASSET-IMPORT.md).
- [ ] Untrusted asset isolation and authenticated remote upload remain separate, unimplemented projects. Prove filesystem,
  network and plugin restrictions throughout inspection and loading, authenticated bounded transfers, and fail-closed behaviour
  before exposing either.
- [ ] Review asset import and export gaps and the need for CAD, splats or heightmaps against real workflows. Do not treat
  unsupported formats or partial conversions as complete.
- [ ] Investigate faithful MJCF tendon and tendon-actuator conversion on 5.1 and 6.2, including Panda gripper contact and
  coupling. Imports warn until the behaviour is verified or preserved.

## Tool surface and reliability

- [ ] Regroup the host modules into subpackages; the rename left the flat layout.
- [ ] Add live smoke checks for the `assets` and `computer` toolsets, and for the ROS 2, code, log and UI tools of `sim`;
  `scripts/smoke.py` has none.
- [ ] Inventory the registered tools, arguments, results, side effects and usage. Propose a keep, combine or remove list and a
  compatibility plan to the owner before changing public contracts.
- [ ] Review the work-cell tool descriptions, partial outcomes, confirmations and destructive effects. Keep existing behaviour
  compatible during any migration.
- [ ] Improve queued-call and process-death errors so they identify the operation and unknown effects; avoid retrying an
  in-flight mutation after a timeout.
- [ ] Review refusal ordering and diagnostics: invalid types and ranges, missing shared staging, unexpected parameters, and
  `TWINSMITH_SIM_MAX_INSTANCES=0`.
- [ ] Review independent light intensity and colour edits, bounded log filtering, stopped-body reads, LiDAR sample and
  `NOT_READY` descriptions, clock source selection, and flat-floor warnings.
- [ ] Review screenshot output-location guidance and object-pose descriptions.
- [ ] Decide whether read-only scene calls may safely omit stage and session IDs; preserve targeting guarantees for changes.
  Examine desktop dialogs, X11 targeting and other display backends.

## Documentation and validation

- [ ] Verify navigation yaw tolerance guidance, map rasterisation and open-floor `start` advice.
- [ ] Verify evaluation episode-count semantics, Lab video dependencies, LiDAR preset frequency, camera adoption after reopen,
  and how an agent adopts a URDF pack.
- [ ] Rerun the [example](EXAMPLES.md) prompts, including LiDAR, in representative disposable sessions.
- [ ] For each change, run the relevant local CI and document observed results and limitations; inspect any published
  screenshots and videos for private data and asset permissions.

## Possibly closed by later changes

These were open on the old checklist but name things the code no longer has. Close them, or restate them, when the issues are
made.

- Scope strict mode and required assets to the lab work cell without blocking public robots. The strict and offline asset settings
  were removed with the old asset model; packs are the only model now, and only the owner's pinned packs are pulled.
- Review work cell quick-start variants. The work cell's quick-start tools were replaced by the work cell's `workcell__cell`
  grouped tool.
