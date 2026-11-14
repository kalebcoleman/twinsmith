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

The work cell builds on Newton since 2026-11-14. Pack `ur5e-jenga-workcell/v2` fixes the asset side:
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
  before initialization, and `workcell__robot(action="wrench")` reads it (the static check passes on both engines).
- [ ] The finger contact reads on Newton: they need Newton's contact snapshot instead of the PhysX contact view, and are
  refused there until then rather than inventing values.
- [ ] `mirror_real` (robot-only gravity off) on Newton. It is refused today. Newton's PhysX resolver may read
  `physxRigidBody:disableGravity`, which needs a live check.
- [ ] Per-engine friction profiles in `lab_hardware.json` (unconfirmed; they start equal to the PhysX values).
- [ ] Measured finger width at open, mid and closed on both engines. The RG2 knuckle drives (1 N m) are slow. Of a -20 degree
  target, they reached -17.7 degrees on PhysX after 360 steps, and -15.2 and -15.5 degrees on Newton after 0.72 s.
- [x] The static check at rest (flange wrench equals the weight below the sensor) on both engines: `scripts/workcell_static_wrench.py`
  passes on PhysX and Newton in two poses (2026-11-13, pack v2: 15.009 N against 15.009 N; torques agree across engines). It
  checks the modelled masses, not the real ones: those still need weighing.
- [ ] Renders (cell, wrist, overhead) on Newton.

### Work cell realism

Datasheet values are in `lab_hardware.json` with their sources (`inertials`, `bota`, `quick_changer`, `rg2`); these need the
real cell:

- [ ] Weigh the Bota with its adapter plates, the Quick Changer (both sides), the RG2, the D415 with its printed mount, and a
  few Jenga blocks. The Bota's adapter plates are custom (both Rokubi faces have M3 patterns, not ISO-50) and are not modelled.
- [ ] Settle whether the RG2's datasheet 0.78 kg includes the Quick Changer tool side (0.14 kg); the cell counts both.
- [ ] Move the wrist D415 and its mount onto the RG2 body (below the F/T sensor), with mass and convex colliders, from the
  measured pose on the printed mount or its CAD. Today they sit above the sensor and weigh nothing.
- [ ] Replace the Bota and D415 placeholder boxes with vendor meshes (STEP needs converting to STL first; CAD is not importable).
- [ ] Tilt-test friction for block on block, block on the HDPE top and pad on block, and enter them with their spread.
- [ ] The UR5e base mass (4.0 kg) is UR's own "might be incorrect" figure; it does not affect dynamics while the base is fixed.
- [ ] Fit the arm's joint damping and armature to real joint logs with `sysid`; datasheets do not give the drive behaviour.

## Real-to-sim with RoboSimStudio

The `robosim` tool reproduced a real push on the GPU (Newton's MuJoCo-Warp solver) and on the CPU: the block's image displacement
matched the clip within the tool's tolerance. What it does not do yet:

- [ ] Match the block's rotation. Only the displacement of its centre is tuned and judged; the overlap at the end frame was 0.39.
- [ ] Fit mass and friction. The push path is tuned; the physics parameters are not. The [`sysid`](SYSID.md) tool fits
  them from recorded trajectories and recovered a synthetic pendulum's damping and armature, but no real dataset has been
  fitted yet: the block needs object poses from the push recordings.
- [ ] Estimate the camera pose from the clip. The simulated view is set by hand, so only the block's mask is compared.
- [ ] Take the pusher's path from the clip. `tune_push` searches a straight two-point path.
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

- [ ] Decide the seven shorter names proposed for listed tools during the rename (the list is in the rename pull request).
- [ ] Regroup the host modules into subpackages; the rename left the flat layout.
- [ ] Add live smoke checks for the `ros2`, `assets` and `screen` toolsets, and for the code, log and UI tools of `sim`;
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
  were removed with the old asset model; packs are the only model now, and nothing is downloaded.
- Review work cell quick-start variants. The work cell's quick-start tools were replaced by the work cell's `workcell__cell`
  grouped tool.
