# UR5e Jenga work cell BOM and private asset pack

Status: **candidate, not runtime-qualified.** The pack-backed Workcell API is integrated
on `workcell/lab-hardware`. Integration and CPU fixture checks do
not establish a PhysX, render, sensor or lab-fidelity pass. This documentation update
does not run the simulator or inspect private assets.

## Scope

The work cell is one UR5e on a Vention bench with a Bota Rokubi, OnRobot Quick Changer,
standard OnRobot RG2, black finger pads, a wrist D415 and an overhead D415 on a removable
gantry. The Jenga tower is 54 blocks of nominal 72 x 24 x 14 mm dimensions.

Machine-readable data:

- BOM: `<repo>/exts/workcell/workcell/bom.csv`
- Human-readable BOM: [BOM.md](BOM.md) (the same 13 logical rows and quantities)
- Hardware configuration: `<repo>/exts/workcell/config/lab_hardware.json`
- Audit helper: `<repo>/exts/workcell/workcell/bom.py`
- Audit test: `<repo>/mcp/tests/test_workcell_bom.py`

## Pack-backed Workcell API

The integration exposes four grouped tools with seven actions through `twinsmith.bridge`, in the `sim` toolset (off by
default: add `+sim`):

| Group | Actions | Purpose |
| --- | --- | --- |
| `workcell__cell` | `quick_start`, `status`, `shutdown` | Build a stopped pack-backed scene, read its report, or release workcell state without clearing the stage |
| `workcell__robot` | `status`, `wrench` | Read the robot build report, or the simulated Bota F/T sensor with the weight it carries |
| `workcell__perception` | `status` | Read camera configuration, not captured sensor data |
| `workcell__jenga` | `build`, `status` | Replace the tower on a stopped timeline or read its build report |

`quick_start` accepts `config_path`, `mode` (`physics` or `mirror_real`),
`discard_changes` and `environment` (`none` or `simple_room`), and answers with a summary;
`workcell__cell(action="status")` returns the full report. `simple_room` references NVIDIA's
Simple Room from the Isaac Sim assets server (checked before the stage is replaced) with its
ground level with the bench feet; room parts inside the bench are deactivated, every room
collider but the ground plane is off. A physics scene, joints and lights the room brings are
deactivated, its articulation roots removed, its rigid bodies made kinematic and its instanced
parts de-instanced. Physics mode uses gravity; mirror mode uses zero gravity, not a
real-robot connection. Replacing an unsaved stage requires explicit consent. There is no
extension panel or legacy ROS graph setup, and no planning,
real-hardware control or evaluation. Generic scene tools remain separate. See the
[tool reference](../../../docs/TOOLS-WORKCELL.md) for the baseline contract.

The Jenga build action accepts optional `profile` and `seed` overrides. Defaults select
`measured_pending` with seed 42: modest bounded uniform variation of mass, friction and
dimensions. All distribution numbers and the selection/seed defaults are explicitly
unconfirmed and easy to overwrite in the configuration. Placement follows sampled block
sizes, not independently randomized tower poses. Rebuild replaces the old tower;
the internal pose-reset helper restores realized initial transforms without resampling,
restoring velocities or resetting solver state. See the
[exact variation schema and limits](../../../docs/TOOLS-WORKCELL.md#seeded-jenga-variation).

## Hardware configuration and provenance

`lab_hardware.json` is the single hardware configuration. Every section field is
`{value, status, source}`; structured profiles are nested within `value`. Config statuses
are only `confirmed` or `unconfirmed`; the other labels below describe BOM/provenance
concepts, not accepted config statuses. The BOM does not replace it or silently choose between
conflicting measurements.

| Status | Meaning |
| --- | --- |
| `confirmed` | Measured, or stated by the lab owner or a datasheet |
| `unconfirmed` | Best current value; must be measured before fabrication or fidelity claims |
| `conflict` | Sources disagree; the configured value is the default, alternatives are recorded |
| `placeholder` | Stand-in geometry or physics value, not a hardware statement |
| `to_buy` | Item is not in the lab; the value is a design choice |

The pack-backed work-cell configuration selects the pack by `pack.id` and `pack.version`
and maps every physical scene role to a **logical asset ID** under `assets`. No asset file
path is stored in the configuration, and there is no old-store path fallback.

`documentation.bom_csv.value` is `../workcell/bom.csv`, resolved relative to the
configuration directory. Preserve that layout when copying a configuration, or overwrite
this field with the appropriate BOM path; it is not relative to the process working directory.

## BOM IDs and quantities

`bom.csv` uses one row per logical asset and instance counts in `quantity`. Every simulated
physical part must map to one of these rows.

| Area | BOM IDs and quantity |
| --- | --- |
| Scene | `bench` 1 |
| Robot | `robot` 1, `base_plate` 1, `bota` 1, `quick_changer` 1, `rg2` 1, `finger_pad` 2 |
| Cameras | `d415` 2, `wrist_mount` 1, `overhead_mount` 1, `overhead_release` 1, `overhead_shelf` 1 |
| Jenga | `jenga_block` 54 |

Both cameras use the single `d415` logical asset; the BOM records two physical units. The
two finger pads use the single `finger_pad` logical asset. A partial tower may instantiate
fewer blocks than the BOM quantity, but each instantiated block maps to `jenga_block`.

## Pricing

`bom.csv` separates money from hardware provenance:

- `price_status=verified_web_list`: a public manufacturer or store list price captured on
  2026-11-12 with its URL in `price_source`. It is a list price, not a quote.
- `price_status=quote_required`: no verified public price for the required configuration.
  `unit_price` is intentionally blank, never zero and never invented.

The only known item prices recorded for this BOM are the D415 body (272.00 USD each,
two units) and the Thorlabs KB3X3 release (117.27 USD, one unit). These are recorded
list prices, not newly checked prices or procurement quotes. Every other row, including
the complete overhead gantry, remains `quote_required`. No complete workcell total is
available; missing prices are not zero.

No purchase, order or financial action is performed by this documentation or its tests.

## Private asset pack

The pack is external to the repository:

- Pack ID and version: `ur5e-jenga-workcell/v1`
- Default root: `<workspace>/assets`;
  `<workspace>` is `TWINSMITH_WORKSPACE`, else the folder that holds the checkout (when it has `envs/` or an asset folder), else `<home>/robotics-workspace`
- Pack directory: `<workspace>/assets/ur5e-jenga-workcell/v1`
- Override: set `TWINSMITH_ASSET_ROOT` to a different asset root; the loader appends the
  pack ID and version.

All pack binaries are private, untracked lab data. They are not committed to the
repository and are not redistributable.

`pack.yaml` is JSON-compatible YAML with this shape:

```yaml
id: ur5e-jenga-workcell
version: v1
qualification: candidate; no PhysX, render, or lab-fidelity pass yet
files:
  - asset_id: robot
    path: robot/robot.usdc
    sha256: <64 lowercase hex characters>
    source: <where the CAD came from and its source hash>
    license: <license or private-use statement>
    redistributable: false
    fidelity: <what is genuine, flattened or proxied>
```

`asset_id` is optional in the schema and defaults to the parent directory name. The current
pack declares it for every file. Every referenced file needs `path`, `sha256`, `source`,
`license` and a boolean `redistributable`.

Loader contract:

- `asset_pack.AssetPack.asset(logical_id)` returns the absolute path for a logical ID.
- `asset_pack.resolve_pack(config)` resolves the configured pack ID and version.
- `asset_pack.verify_dependencies(pack)` reports the dependency closure and unresolved
  references.

The scene integration uses those functions. It must not fall back to an old store, a
previous checkout or a path in the configuration.

## Making or updating a pack

1. Work from a lab-authorized source copy. Do not delete or move an existing asset store.
2. Flatten the chosen USD, remove instanceable prototypes, keep the default prim and set
   metres, kilograms and Z-up. Replace proprietary MDL materials with USD Preview Surface
   materials when the MDL cannot ship in the private pack.
3. Remove external asset references that cannot be resolved inside the pack. The final
   file must be self-contained.
4. Write the file under
   `<root>/ur5e-jenga-workcell/v1/<asset_id>/<asset_id>.usdc` (or `.usda`).
5. Hash the final bytes with SHA-256 and record the source hash, license and
   `redistributable` flag in `pack.yaml`.
6. Run the dependency closure check. Each file should report one layer (the `v2` bench: its file and the two layers it
   sublayers), no external asset references and no unresolved references. A dependency failure blocks the pack; it is not
   worked around with a fallback path.
7. Load the pack through `AssetPack.asset` and `resolve_pack`, then run the BOM audit and the
   PhysX smoke before claiming runtime qualification or making any old-store report.

The BOM audit checks structural manifest fields and, when given a pack root, the actual file
hashes. It also requires every non-`materials` pack asset to have a BOM row and every BOM
asset to exist in the pack.

## Pack v2

`v2` is built from `v1` by `scripts/workcell_pack_v2.py` (CPU only; run it from `mcp/` with
`uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_pack_v2.py`). `v1` is only read, and
`v2` is written by the pack library's staged, verified writer. Each changed file's record names the `v1` file and hash it came
from. The changes:

- **Bench.** `bench/bench.usda` composes `bench_render.usdc` (the original meshes) and `bench_collision.usdc` (one oriented box
  per mesh, purpose `guide`), made with `assets__mesh` split. The work cell keeps a bench's own collision layer, so the 385
  triangle-mesh colliders (1.63 M triangles) that overflow Newton are no longer created.
- **RG2.** The gripper has no articulation root of its own and joins the arm's articulation through its mount joint. The
  massless `gripper_center` body and its fixed joint are a plain frame. The two loop-closing joints, excluded from the
  articulation, are removed together with the zero-force drives of the passive joints. Each finger's inner joints follow
  its driven knuckle through `NewtonMimicAPI`, which PhysX (Isaac Sim 6.2) and Newton 1.6 both read. The ratios are solved
  from the asset's own joint frames over the knuckle's limits: inner finger -0.9897 and inner knuckle +0.9919 times the
  driven joint, with a worst closure gap of 0.037 mm for the linear fit. `pack.yaml` records them under `mimic`. PhysX
  accepts a mimic only on revolute joints with finite limits, so each follower's limits are its leader's range mapped
  through the ratio, plus 5 degrees.
- **UR5e.** The disabled `robot_gripper_joint` (no second body) is removed. **The drive gains are unchanged.** They are stiff
  enough for PhysX with gravity on, and `assets__validate` warns (`DRIVE_GAIN`) that they are too stiff for a 500 Hz explicit
  solver; the gains Newton needs are a per-engine setting for Phase 2, not a pack edit.

`assets__validate` passes on the bench and the RG2, and gives only the six `DRIVE_GAIN` warnings on the UR5e.

**Live on PhysX (2026-11-14, Isaac Sim 6.2.0, headless, RTX PRO 6000 Blackwell, physics at 600 Hz).**
- `scripts/workcell_smoke.py` passes. The arm and gripper form one articulation, and nothing is patched at build time.
- Twelve DOFs: six arm joints, two driven knuckles and four mimic followers. Every value stays finite.
- With gravity on, the arm drifts at most 0.0002 degrees over 960 steps.
- Both knuckles were commanded to -20 degrees and reached -17.7 degrees after 360 steps, held back by the 1 N m drive.
- The four mimic followers matched their ratio to within 0.001 degrees.
- Kit logged no warnings or errors.
- The 54-block tower settled by at most 0.95 mm over 2 s.
- Not tested: Newton, grasp contact and cameras.

## Physical fidelity

The current candidate contains:

- Genuine UR5e, Vention bench, OnRobot Quick Changer and OnRobot RG2 meshes, flattened from
  preserved local vendor CAD. Their MDL materials were replaced with preview materials, so
  appearance is not vendor-accurate.
- No genuine Bota Rokubi, D415, wrist bracket or gantry CAD is available. These are
  dimensioned, unconfirmed proxies.
- The overhead support is the pack's three-member gantry (two posts and a cross-member).
  The posts stand on the bench top panel (inside its measured footprint, x up to 0.9436 m,
  y -0.811..-0.091 m), outside the arm's reach, and the camera hangs under the
  beam through the stacked pack proxies (camera, printed shelf, KB3X3 release plate, beam
  underside), each touching the next within 2 mm. The stack heights come from the proxy
  dimensions, the pack scales member thickness with the post span, and the real attachment
  details are unconfirmed, so this does not establish mounting feasibility. The KB3X3 is a
  camera-only release, not a load-bearing gantry base. Printed mounts and shelf remain proxies.
- The wrist D415 is 90 mm below the tool axis and tilted 8 degrees away from it. Projecting
  every RG2, Quick Changer and Bota vertex through the camera finds none inside the colour
  frustum plus a 3 degree margin (3.7 degrees clear of the edge). A render of the previous
  75 mm / 6 degree pose still showed the finger tips at the bottom edge of the frame.
- Pack leftovers that PhysX warned about are fixed in `v2` itself: the geometry-free `gripper_center` body is a plain
  frame, the UR5e's disabled `robot_gripper_joint` is gone, and the RG2's own articulation settings (16 velocity
  iterations) went with its articulation root. The assembly builder still patches them in the session layer when it loads
  `v1`. The RG2 stays the standard 110 mm gripper.
- A plain Jenga block proxy without measured chamfers.

This is not a high-fidelity-qualified model. Flattening preserves mesh and articulation
layout from the vendor files, but material appearance, collision behavior, mass properties
and alignment still require validation. Proxy dimensions come from the hardware
configuration or a design envelope and are not vendor drawings.

## Measurements still needed

| Item | Configured value | Status |
| --- | --- | --- |
| Bench height | 0.885 m default; 0.801 m alternative | `conflict` |
| Bench top | 1.4 x 0.9 m | `unconfirmed` |
| Base plate thickness | 0.005 m (5 mm), replacing the old zero placeholder | Provisional; installed thickness `unconfirmed` |
| Bota diameter / length / mass | 0.048 m / 0.032 m / 0.113 kg; sensor frame on the tool-side face (0.032 m) | `confirmed` (Bota datasheet and URDF); the lab's adapter plates are not modelled |
| Quick Changer standoff | 0.01592 m (15.92 mm) in CAD | CAD `confirmed`; installed size `unconfirmed` |
| Quick Changer mass | 0.2 kg (robot side 0.06 + tool side 0.14), pack inertia scaled to it | `unconfirmed` (OnRobot datasheet v2.0; 0.41 kg was the Dual Quick Changer; the RG2's 0.78 kg may include the tool side) |
| RG2 mass and link split | 0.78 kg total, split by pack-v2 hull volume (base 0.639 kg, CoM z 0.064 m) | total `confirmed` (OnRobot datasheet v1.8); split `unconfirmed` |
| UR5e link inertials | 3.761 / 8.058 / 2.846 / 1.37 / 1.3 / 0.365 kg with UR's CoMs and inertia tensors | `confirmed` (Universal Robots ROS 2 description `physical_parameters.yaml`; UR's DH article) |
| RG2 maximum opening | 0.110 m | `confirmed` |
| Finger pad size | 0.012 x 0.008 x 0.025 m | Provisional proxy; installed dimensions `unconfirmed` |
| D415 body | 0.099 x 0.020 x 0.023 m | `unconfirmed` |
| Wrist camera offset | -0.012, -0.09, -0.005 m in tool0 (design pose; Max's measured value was y = -0.038) | `unconfirmed` |
| Wrist camera tilt | 8 degrees about the optical X axis (design value) | `unconfirmed` |
| Overhead camera position | 0.91, -0.2, 1.02 m, under the gantry beam | `unconfirmed` |
| Gantry post positions and height | (0.91, -0.79) and (0.91, -0.10) m; 1.0944 m | `unconfirmed` |
| Shelf and release stack offsets | +0.0205 m and +0.0305 m above the camera (from proxy dimensions) | `unconfirmed` |
| Printed shelf | 0.1 x 0.06 x 0.008 m | `placeholder` |
| Jenga nominal block size | 0.072 x 0.024 x 0.014 m (72 x 24 x 14 mm) | Nominal dimensions `confirmed`; tolerances `unconfirmed` |
| Jenga block mass and friction | Not measured | `unconfirmed` |
| Jenga tower | 54 blocks, 18 layers | `confirmed` |

The lead's pending CSV updates distinguish the provisional 5 mm plate and confirmed
15.92 mm Quick Changer CAD standoff from unmeasured installed hardware. Older CSV notes
may still show a zero plate or an unqualified standoff confirmation until those updates
land; this documentation does not edit the CSV. Confirmed nominal Jenga dimensions do not
confirm manufacturing tolerances, mass, friction or chamfers.

Measure the bench, base plate, Bota envelope and mass, Quick Changer installed size and mass, pad geometry,
camera offsets and gantry before calling the model dimensionally faithful.

## BOM audit

`workcell.bom.audit_parts(parts, rows, stage)` accepts a report's `parts` list and an
optional USD stage. It checks that:

- every part has a nonempty `prim_path`, `asset_id` and `bom_id`;
- the BOM row exists and its logical `asset_id` matches the report;
- a supplied stage has the part prim and matching `workcell:asset_id` and
  `workcell:bom_id` attributes;
- every mesh, collider or rigid-body prim has a mapped BOM ancestor, including camera mount
  pieces and every Jenga block instance;
- every BOM row has at least one mapped physical part.

The audit walks the stage instead of trusting a manually curated list. A mesh anywhere
outside a mapped part, or a report part whose prim is missing, fails the audit.

## Newton

The cell builds on the session's engine and never switches it. On PhysX it keeps the pack's drives. On Newton (MuJoCo-Warp
integrates a drive's stiffness explicitly, so the pack's 9400 N m per degree diverges at 500 Hz) `newton_cell.prepare` authors,
on the stage and never in the pack:

- the config's `newton.drives` for the six arm joints. Each joint gets stiffness and damping in SI per radian, written in USD's
  per-degree units, and an armature of 0.1 kg m² on both `newton:armature` and `physxJoint:armature`. The gains keep
  `dt² k / armature` at or below 2, against an explicit-spring limit of at least 4:

  | Joints | Stiffness (N m/rad) | Damping (N m s/rad) |
  |---|--:|--:|
  | Shoulder pan | 25,000 | 300 |
  | Shoulder lift | 50,000 | 600 |
  | Elbow | 25,000 | 250 |
  | Wrists | 5,000 | 50 |

  Their maximum forces are the pack's 150 and 28 N m. The report's `stability` lists the margin per joint.
- the tower's settings at `newton.physics_hz` (500 Hz): an elliptic friction cone, 100 iterations, `impratio` 10 and a 4 mm
  contact gap on every collider;
- contact buffers sized for the bodies in the scene.

On Newton the build also refuses, before replacing the stage:
- a `newton` section that does not name all six arm joints with positive gains;
- `mirror_real` (gravity off on the robot);
- an RG2 with its own articulation root.

It also refuses a bench without its own collision layer, as in pack `v1`.

**Live on Newton (2026-11-14, Isaac Sim 6.2.0, Newton 1.6.1 / MuJoCo-Warp, headless, RTX PRO 6000 Blackwell, 500 Hz, pack
`v2`).**
- `scripts/workcell_smoke.py` passes: 70 bodies, contact buffers of 2240 and 17920 with no warnings, 455 colliders with the gap.
- At the presentation pose `[0, -80, 100, -110, -90, 0]` degrees, with gravity on, every joint holds within 0.05 degrees, creeping
  about 0.0002 degrees per second.
- A 0.05 rad step on each joint in turn settles within 0.2 s to the same offset, with no oscillation.
- The RG2 fingers move (they were locked on Newton with `v1`), and the four mimic followers stay within 0.003 degrees of their
  ratios.
- After a 1.2 s settle the 54-block tower drifts at most 0.54 mm (median 0.23 mm) over 11 s.
- No physics warnings in Kit's log.
- Commanding every joint at once from the zero pose (arm stretched out) drove the arm into the cell, so start from a pose
  inside the workspace.
- Not qualified on Newton: the cameras and `mirror_real`. The Bota force/torque read passes the static check on both
  engines (`scripts/workcell_static_wrench.py`).

## Authoring/BOM smoke

The cell builds on the session's engine (`run.sh --physics-backend physx|newton`). Run the smoke only against an explicitly
selected bridge and configuration:

```bash
python3 <repo>/scripts/workcell_smoke.py --host 127.0.0.1 --port <port> \
  --config <absolute-config-path> --mutate-stage
```

`--config` must be absolute on the Kit host. `--discard-changes` is allowed only with
explicit authorization because the smoke replaces the stage. The smoke checks the backend,
stopped timeline, units, gravity, Jenga layout and per-part attributes. Pair it with the BOM
audit on the returned parts before claiming a pass. Global gravity remains 9.81 m/s² in
both modes; mirror mode disables gravity only on robot assembly rigid bodies. This is
**stopped-stage authoring/BOM validation only**: no physics steps or rendered captures
are performed. Dynamics and full-cell, wrist and overhead captures require separate
home-PC qualification before claiming a runtime pass.

## Current status and gaps

- Earlier pack preparation reported 14 declared files with checked manifest fields,
  hashes and dependency closure. This documentation update does not recheck that private
  pack; those structural results do not establish a PhysX or render pass.
- The integrated baseline includes a BOM contract and stage-mapping audit with CPU
  fixtures. No real-stage BOM audit, PhysX smoke or render pass is claimed here.
- No genuine Bota, D415 or mount CAD is available; those parts remain unconfirmed proxies.
- The old-store unreferenced report must wait until the pack loads, PhysX smoke passes,
  renders are reviewed and the BOM audit passes on the real stage. It is intentionally not
  produced here.
