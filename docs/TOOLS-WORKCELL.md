# Work-cell tools: the Jenga variation contract

The work cell is the lab's UR5e Jenga cell. Its tools (`workcell__cell`, `workcell__robot`, `workcell__perception`,
`workcell__jenga`) belong to the `sim` toolset, which is off by default. They are
documented in the work cell's own [README](../exts/workcell/workcell/README.md), and every argument is in the generated
[tool arguments](TOOLS-ARGUMENTS.md#sim). This page keeps the one contract that README links to: the
exact schema of the seeded Jenga variation.

The work cell uses externally supplied, pack-resolved assets and ships no robot binaries. It builds a stopped scene; the default
mode uses gravity and `mirror_real` uses zero gravity. Replacing an unsaved stage requires explicit consent, and the stage
remains in place after shutdown. It builds on PhysX (the pack's drives) and on Newton (the config's `newton` drives and solver
settings; `mirror_real` refused there); see [compatibility](COMPATIBILITY.md#target-engine). There is no
force capture, motion planning, real-hardware control or evaluation implementation.

## Seeded Jenga variation

Use the `build` action of the `workcell__jenga` group
with `profile="measured_pending", seed=42` (single-purpose alias
`workcell__jenga(action="build")`) to rebuild
the tower in the current workcell. This is destructive: it replaces the old tower,
not the rest of the workcell. Stop the timeline first. Omitted `profile` and `seed`
use `jenga.variation_profile` and `jenga.variation_seed` from the hardware configuration.
A selected profile requires an integer seed. This wiring is CPU-tested, not a PhysX,
render, sensor or physical-fidelity qualification.

The nominal block dimensions remain **72 × 24 × 14 mm**. The supplied
`measured_pending` profile and seed `42` are explicitly **unconfirmed** defaults,
not measurements: uniform mass 0.014–0.016 kg, static friction 0.28–0.34,
dynamic friction 0.22–0.27, and additive size tolerance −0.0002–0.0002 m
independently on each axis. Replace these provisional numbers with measured values.
Edit the `value`, `status`, and `source` wrappers for `variation_profiles`,
`variation_profile`, and `variation_seed`; distribution objects live inside
`variation_profiles.value`. Set `variation_profile.value` to JSON `null` to disable
variation and use nominal dimensions and scalar mass/friction. Omitting the tool
argument does not disable a configured profile.

Each named profile has exactly `mass_kg`, `static_friction`, `dynamic_friction`, and
`size_tolerance_m` (three distribution objects). Supported distributions are:

| `distribution` | Additional fields |
| --- | --- |
| `constant` | `value` |
| `uniform` | `min`, `max` |
| `normal` | `mean`, `std`, `min`, `max` |

Values must be finite; mass is positive and friction nonnegative. Normal distributions
require positive standard deviation and a mean inside their bounds. Size tolerance
is limited to ±0.001 m by a software guard, not a measured manufacturing specification.
Bounded normal draws and friction pairs with dynamic friction above static friction
are rejected and retried up to 128 times, never clipped. Only the selected profile
is validated. Sampling uses a local seeded generator, leaving global random state alone.

The report records the profile, seed, sampling policy and each block's mass, friction,
dimensions, tolerance, initial position and rotation. Each block receives its own
physics material. Sampled widths center rows; alternating layers rotate 90 degrees
and sit above the tallest block below. Shorter blocks can leave gaps that need settling;
tower position and orientation are not independently randomized.

Rebuilding samples again (the same configuration and seed reproduce the realization).
The internal Jenga pose-reset helper instead restores recorded initial transforms;
it does not resample, restore velocities or reset solver state. It requires the caller
to stop the timeline and is not a separate exposed workcell action.
