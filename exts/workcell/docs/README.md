# workcell extension

**RISE lab's UR5e digital twin: Kaleb's work with that lab and that robot.** The rest of
this project is robot-agnostic; use `sim__quick_start` for a generic scene.

The extension integrates the pack-backed Workcell API with the shared
`twinsmith.bridge`; it has no socket server or UI panel of its own. The
workcell integration provides four grouped tools and seven actions:

| Group | Actions |
| --- | --- |
| `workcell__cell` | `quick_start`, `status`, `shutdown` |
| `workcell__robot` | `status`, `wrench` |
| `workcell__perception` | `status` |
| `workcell__jenga` | `build`, `status` |

`quick_start` resolves the external `ur5e-jenga-workcell/v2` pack and builds a stopped
scene on the session's engine, PhysX or Newton. `physics` mode uses gravity; `mirror_real` uses zero gravity (PhysX only)
but does not synchronize with hardware. An unsaved stage is replaced only with explicit consent.
`shutdown` releases workcell state without clearing the stage. Robot, perception and
Jenga status actions read build reports; perception status is not sensor capture.
Real-to-simulation synchronization, planning and evaluation are not part of the work cell.

`quick_start` answers with a summary (paths, engine, BOM verdict, Newton buffers, engine limitations);
`workcell__cell(action="status")` returns the full report. `environment="simple_room"` stands the cell in NVIDIA's Simple
Room, referenced from the Isaac Sim assets server (it needs network, and is checked before the stage is replaced): the room's
ground is level with the bench feet, room parts inside the bench (its own table) are deactivated, and every room collider but
the ground plane is off. The config's `bench_top` section colours the top panel and gives it a friction material on both
engines (black HDPE; the friction values are placeholders until a tilt test measures them). The `contact` section makes
contact the same on both engines: every collider without a material gets a default one (PhysX's 0.5; Newton would give
it 1.0), every material combines friction by max (MuJoCo's rule) and does not bounce, and on Newton, which reads one
coefficient, each material's static value is used. The finger pads carry convex colliders, so the pad material is what
touches a block. `workcell__robot(action="wrench")` reads the Bota F/T sensor on both engines with the weight it carries;
`scripts/workcell_static_wrench.py` checks the two agree at rest. The `inertials` section sets explicit mass, centre of
mass and principal inertia on named bodies (the UR5e links from Universal Robots' ROS 2 description, the RG2 links at the
datasheet's 0.78 kg), so neither engine computes them from collider shapes; the Bota and Quick Changer masses come from their
own sections.

There is no legacy extension ROS graph setup or custom `rclpy` build prerequisite for
this API. Generic `sim__*` tools are separate from the workcell interface.

**Candidate, not runtime-qualified:** integration does not establish a PhysX smoke,
render, sensor or physical-fidelity pass. The seeded Jenga build action replaces the
tower on a stopped timeline. Its default `measured_pending` profile and seed 42 are
unconfirmed, overwriteable configuration, not measurements. Nominal dimensions remain
72 × 24 × 14 mm. See the [variation and reset contract](../../../docs/TOOLS-WORKCELL.md#seeded-jenga-variation)
for distribution fields, sampling bounds, and the distinction between rebuilding and
restoring realized initial poses without resetting velocities or solver state.

- Workcell configuration, fidelity gaps and pack creation: [workcell README](../workcell/README.md)
- Hardware inventory and recorded prices: [BOM](../workcell/BOM.md)
- Tools and behavior: [workcell tools](../../../docs/TOOLS-WORKCELL.md)
- Arguments: [tool arguments](../../../docs/TOOLS-ARGUMENTS.md#sim)
- Generic scenes: [scene tools](../../../docs/TOOLS-SCENE.md)
- Asset policy: [asset packs](../../../docs/ASSET-PACKS.md)

The repository ships no pack binaries. Set `TWINSMITH_ASSET_ROOT` to override the asset
root; the loader appends `ur5e-jenga-workcell/v2` (the version in `config/lab_hardware.json`). Configuration uses logical asset IDs,
not file paths or a legacy-store fallback. Preserve the manifest, provenance, hashes and
dependency checks described in the workcell README when creating a private pack.
