# Asset packs

> Bringing one robot or asset **file** into a scene (USD, URDF, xacro, SDF, MJCF, or a folder that holds one)? See
> [ASSET-IMPORT.md](ASSET-IMPORT.md). This page is about packs: the folders of verified files a lab keeps and reuses.

- **The repository ships no assets.** It is code only, so nothing with a licence question is ever pushed.
- **There is one model.** A pack is `<asset root>/<pack id>/<version>/` with a `pack.yaml` that lists every file with its
  hash, source and licence. The UR5e Jenga work cell, the pack tools, the CLI and `sim__quick_start` all read it the same way.
- **Only `pull` downloads**, and only what your own sources file pins: a pack version from a Hugging Face repository at a
  fixed commit, checked against its `pack.yaml` before it lands ([pulling and pushing](#pulling-and-pushing-packs)). Nothing
  else fetches a file. NVIDIA's own robots (`franka`, `ur5e`, `unitree_g1`, ...) are not packs: `sim__quick_start` takes them
  by name from the running simulator's assets root.
- **A version never changes.** To change a pack, write a new version.

## What a pack is

```text
<asset root>/
  ur5e-jenga-workcell/            pack id
    v1/                           version
      pack.yaml                   the manifest
      robot/robot.usdc            every file the manifest lists, and nothing else
      bench/bench.usdc
```

`pack.yaml` is JSON-compatible YAML (JSON is valid YAML, so any reader takes it; the tools write JSON):

```yaml
id: ur5e-jenga-workcell
version: v1
files:
  - asset_id: robot                 # optional: a logical name the work cell loads by
    path: robot/robot.usdc          # relative to the version folder; no "..", no absolute path
    sha256: <64 lowercase hex characters>
    source: Preserved local vendor CAD, SHA256 5837bf...
    license: Private lab use only; redistribution not authorized
    redistributable: false          # a boolean, always present
    fidelity: ...                   # any extra fields are kept
    collision: {...}                # written by build for a split asset: see Preparing meshes
```

The reader is strict and fails closed. A pack is refused if the manifest is a symlink or does not match the requested id
and version, if a record lacks `path`, `sha256`, `source`, `license` or a boolean `redistributable`, if a path escapes the
folder or repeats, if a listed file is missing or its hash differs, if two files share an `asset_id`, or if the folder holds
a file the manifest does not list. The one implementation is `mcp/twinsmith/pack_manifest.py` (standard library only, no
Kit). `exts/workcell/workcell/asset_pack.py` implements the same contract for the work cell; a test runs both on the same
packs and checks that they give the same verdict and message.

## The asset root and how it is found

One rule resolves it, for the host, the CLI and Kit alike (Kit, which cannot import the host module, repeats it in `exts/workcell/workcell/asset_pack.py`). In order:

1. `TWINSMITH_ASSET_ROOT`, an absolute path.
2. `<workspace>/assets`, if that folder exists.
3. Otherwise an error that names the setting.

`<workspace>` is `TWINSMITH_WORKSPACE`, else the folder that holds the checkout (when it has `envs/` or an asset folder), else `<home>/robotics-workspace`.
Set `TWINSMITH_ASSET_ROOT` in the MCP server's `env` **and** in the environment of any Kit you start yourself: each reads
its own settings. `assets__pack action=list` reports what the host resolved, and warns when a connected Kit uses another root.

`python3 scripts/assets.py where` shows the root, how it was found, and the project file.

## Using a pack

| Where | How |
|---|---|
| `sim__quick_start` | `robot="<pack id>/<version>/<file.usd>"` (or `.urdf`): the manifest must list the file and its bytes must match |
| `assets__import_path` | a path inside a pack version imports without approving a root, if the whole pack verifies |
| The work cell | the work-cell configuration names `pack: {id, version}` and logical `asset_id`s |
| A project | `twinsmith.json` in the project folder: `{"pack": "<pack id>/<version>"}` is the default for `doctor` and `pin` |

A project file's `asset_path` and `strict` keys no longer exist; a file that has them is an error, not ignored. A project file
is found from the working directory, so it never approves a folder.

## The tools

The same eight actions are the MCP tool `assets__pack` (`action=list|doctor|dependencies|adopt|pin|build|pull|push`; each is
also callable as `assets__packs`, `assets__pack_doctor`, ...) and, apart from `dependencies`, the CLI `twinsmith assets`
(`python3 scripts/assets.py` from a checkout that is not installed; the examples use that form). They run on the host and
need no Kit, apart from parsing USD (below). A tool writes only under the asset root and never takes a destination path.
`pull` and `push` are described in [their own section](#pulling-and-pushing-packs).

**list** shows every `<id>/<version>` with its file count, size and whether it verifies. A folder with files or a `SHA256SUMS`
and no `<version>/pack.yaml` is listed as *old format*, with a hint to convert it.

```bash
python3 scripts/assets.py list
#   ur5e-jenga-workcell/v1: 14 file(s), 67.2 MB, verified
#   workcell: old format. Convert it with adopt ...
```

**doctor** verifies a pack against its manifest, then computes the dependency closure of its USD, URDF, MJCF, SDF, mesh and map
files: every reference must be a file of the same pack. It changes nothing. A pack that cannot be checked (no USD parser) is not
`ok`.

```bash
python3 scripts/assets.py doctor ur5e-jenga-workcell/v1
```

**adopt** copies a folder into a **new** `<id>/<version>` and writes `pack.yaml`. `source` and `license` are required;
`redistributable` is false unless you say so. A tool takes a folder under an approved root.

```bash
python3 scripts/assets.py adopt <folder> --pack my-robot/v1 --source "vendor CAD, 2026-11" --license "private lab use"
```

**pin** rewrites the `sha256` values of an existing manifest after showing what changed. It is for a deliberate edit of a pack
that no longer verifies; a version that verifies is not touched. The tool refuses a changed hash unless you name the file or
pass `force`. It cannot add a file the manifest does not list or restore one that is missing: write a new version.

```bash
python3 scripts/assets.py pin my-robot/v1 --dry-run
```

**build** copies named files and only what they depend on out of a pack (or from an approved folder) into a new version.
A file copied from a pack keeps that pack's `source`, `license`, `redistributable` and `asset_id`; any other file needs
`source` and `license`. Unresolved, remote or outside references refuse the whole build and copy nothing.

```bash
python3 scripts/assets.py build robot/robot.usdc --pack small/v1 --from-pack ur5e-jenga-workcell/v1
```

A new version is written into a hidden staging folder, checked with the same reader that will load it, and moved into place
only if it verifies; a failed write leaves nothing behind. A tool refuses a version that exists, a symlink anywhere in a
source or pack, a source outside the approved roots, and an asset root inside the repository.

**Where USD is parsed.** The host normally has no `pxr`, so the tools use a Python with `usd-core` named by
`TWINSMITH_USD_PYTHON` (`uv venv <dir> && uv pip install --python <dir>/bin/python usd-core`: no Kit or GPU, and a crash cannot
take down the server), else the selected Kit through `assets__dependencies`. URDF, MJCF, SDF, meshes and maps are read on the
host. A URDF pack is checked with the same rules as a build: `package://` is looked for inside the pack only (a `package.xml`
naming it, a folder of that name, or the folder itself), and nothing outside is searched.

## Preparing meshes and checking an asset

Two more host tools read a USD file under an approved root or the asset root, or a pack's file by `asset_id`. They need the
optional `mesh` extra in the server's environment. Without it, they answer `MESH_EXTRA_MISSING` with the command to add it:

```bash
UV_PROJECT_ENVIRONMENT=<repo>/mcp/.venv uv sync --project <repo>/mcp --locked --extra isaaclab --extra mesh --no-dev
```

The work runs in a subprocess of the server's interpreter, so `pxr`, trimesh and CoACD never load in the server itself.

**`assets__validate`** answers whether an asset will load on Newton or on PhysX, from the file alone. It returns `ok`, and
for every rule its level, the prim, a detail and a fix. On `engine="physx"` the Newton-only rules are `info`.

| Id | Level on Newton | Rule | Evidence |
|---|---|---|---|
| `UNITS` | error | `metersPerUnit` and `upAxis` are authored | Import contract |
| `TRI_COLLIDER` | error | Triangle-mesh colliders hold at most `triangle_budget` triangles (200,000) | The bench's 1.63 M collider triangles needed 7.5 M triangle pairs against Newton's 1 M |
| `TRI_DYNAMIC` | warning | No triangle-mesh collider sits on a moving rigid body (also a warning on PhysX) | PhysX does not simulate triangle meshes on dynamic bodies |
| `COLLIDER_RENDERED` | warning | A collision-only shape has purpose `guide` | The split contract below |
| `DRIVE_ZERO_FORCE` | error | No joint drive has `maxForce` 0 | MuJoCo: "actfrcrange[0] should be smaller than actfrcrange[1]" |
| `NESTED_ARTICULATION` | error | No articulation root lies inside another or is jointed into one | Newton's importer mis-lists the bodies |
| `MASS_MIN` | error | Every moving body has colliders, or an authored mass and inertia above MuJoCo's `mjMINVAL` | "mass and inertia of moving bodies must be larger than mjMINVAL" |
| `DRIVE_GAIN` | warning | `dt² k / I < 4 + 2 dt c / I` at 500 Hz: `k`, `c` in SI per radian or metre; `I` the driven body's largest inertia (else 0.01) plus the joint's armature (else Isaac Sim's 0.1). MuJoCo-Warp integrates stiffness explicitly and damping implicitly | 9400 N m per degree went non-finite |
| `MIMIC` | error (both engines) | A mimic joint and its leader are in the articulation, not excluded loop joints; it names one leader; revolute ones have finite limits | PhysX 6.2 rejects such a mimic; Newton skips one on a loop joint |

An excluded loop joint with no mimic is valid on both engines (Newton makes it a MuJoCo connect or weld). The rules read
the file: colliders a scene adds while it is built, as the work cell does for the `v1` bench, are not seen.

`kit=true`, which would add Isaac Sim's own Asset Validator, is refused for now: it has not been confirmed in Isaac Sim 6.2.

**`assets__mesh`** prepares meshes. `inspect` reports every mesh's triangles, bounds, whether it is closed, and its collider
approximation, and answers directly. The other actions write a **new** folder under an approved root, never inside the asset
root. They run as a job: wait with `jobs(action="lab_status", job_id=..., wait_s=60)`, and the result is in
`mesh_report.json` in that folder.

- `decimate`: a copy whose render meshes are decimated. This lowers render cost; it does not fix a collider overflow.
- `hulls`: a collision layer only. Each mesh gets `<parent>/<mesh>_collision` holding an oriented `box`, one `convex` hull,
  `coacd` pieces, or `auto` (the box unless it is more than twice the hull's volume, then CoACD). The shapes have purpose
  `guide` and `CollisionAPI`, and are in the mesh parent's frame, so bodies and joints are untouched. A flat mesh (a plate)
  always gets a box in its own plane, at least 0.5 mm thick on each side, so it keeps a volume, a mass and a contact depth.
- `split`: the asset as two layers and the file that composes them:

```text
bench/
  bench.usda             sublayers the two below; keeps units, up axis and default prim. Point the asset at this file.
  bench_collision.usdc   the explicit collision shapes
  bench_render.usdc      the source's root layer; the replaced meshes lose their collision schemas, other colliders stay
```

Which meshes get shapes: those under `prims`. Without `prims`, the file's mesh colliders get them, or every mesh when none of
them is a collider; the bench is the second case, because the work cell adds its colliders at build time. A work-cell bench
that brings its own collision layer keeps it, and one without gets triangle-mesh colliders as before. Paths are written
relative, through the USD API, so a home folder with `@` in its name (as on some university logins) still resolves. To make
the result a pack version, run `assets__pack action=build` on `<output>/bench.usda`.

`mesh_report.json` also holds the hash of every file the action wrote and a collision summary. When `build` copies the file
the summary describes (`bench.usda` for `split`, the `_collision.usdc` layer for `hulls`) and none of the hashed files has
changed, the record of that file in `pack.yaml` gets a `collision` field; the report itself is not copied. The hashes cover
the output folder only: if the source's render layer references other files and those change, the counts (and the collision
layer) are stale, and nothing detects it. For the work-cell bench (a box split of `v1`, byte-identical to `v2`'s three
files) it is:

```yaml
- asset_id: bench
  path: bench/bench.usda
  collision: {method: box, meshes: 385, boxes: 385, hulls: 0, triangles: 0, source_triangles: 1628132, thickened: 0}
```

`meshes` is how many meshes got shapes, `boxes` and `hulls` the shapes, `triangles` the hulls' triangles (a box is an analytic
shape, with none), `source_triangles` the triangles of the meshes they stand for, and `thickened` the boxes raised to the
minimum thickness. `coacd` and `auto` also record `max_hulls` and `threshold`. A record copied out of another pack keeps its `collision` field; `pin` removes it from the
records in the folder of a file whose hash it rewrites, because the counts no longer describe the files.

## Converting an old pack

The old model kept flat folders `<store>/<pack>/` with a `SHA256SUMS` and, for builds, a build-description file. Nothing reads
them any more. `list` shows them as old format. Convert one in place; the hashes come from the files themselves, not from the
old pins, and the old files are left where they are:

```bash
python3 scripts/assets.py adopt --from-old <asset root>/workcell --source "lab files, 2026-11" --license "private lab use"
# writes <asset root>/workcell/v1/pack.yaml (add --pack workcell/v2 to choose another version)
python3 scripts/assets.py doctor workcell/v1
```

The tool form is `assets__pack action=adopt pack="workcell/v1" from_old="workcell" source=... license=...`. Delete the old files
yourself once the new version has passed `doctor`.

## Making a new pack

For how a pack is authored (flattening the CAD, keeping it self-contained, recording the source hash, then the dependency
check and the BOM audit), follow [the work cell's pack guide](../exts/workcell/workcell/README.md#making-or-updating-a-pack).
This page does not repeat it.

## Without private packs

Nothing in twinsmith needs a pack of yours, a Hugging Face account or a token. With no sources file, `pull` answers
`NO_SOURCE` and every other tool works as before. Three ways to get assets into a scene:

| Source | How | Cost |
|---|---|---|
| NVIDIA's own robots and scenes | `sim__quick_start robot="franka"` (or `ur5e`, `unitree_g1`, ...), `environment="warehouse"`; they load by name from the running simulator's assets root | Free with Isaac Sim; not packs, nothing to pull |
| Files you already have (a URDF, a USD, vendor CAD you converted) | `assets__import_path` for one scene, or `twinsmith assets adopt <folder> --pack my-robot/v1 --source ... --license ...` to make a pack that every machine can check | Your files stay on your machine |
| A part an agent designs | SolidCraft beside twinsmith, then `hardware(action="part")`, `assets__mesh` and `assets__validate` ([from a SolidCraft part to the work cell](HARDWARE.md#from-a-solidcraft-part-to-the-work-cell)); `adopt` the result to keep it as a pack | Free, open source |

A public Hugging Face repository also works with no token: list the pack in the sources file below and `pull` is anonymous.
The UR5e work cell is the one feature that needs specific packs (`ur5e-workcell/v4`, `jenga/v1`); they hold vendor CAD that
may not be redistributed, so each owner builds them from their own copy ([rebuild](REBUILD.md#the-sequence)).

## Pulling and pushing packs

Keep your packs in a **private Hugging Face dataset repository** and pull them onto every machine. twinsmith stays open source
and ships no assets: the repository name, the pinned commits and the tokens are yours, in files outside every checkout.

| File | Holds | Secret? |
|---|---|---|
| `~/.config/twinsmith/pack-sources.toml` | Each pack version's repository, pinned commit and folder; `[hub]` defaults | No. `push` rewrites it |
| `~/.config/agent-secrets/hf_token` (`hub.token_file`) | A read token, used by `pull` | Yes, mode 600 |
| `~/.config/agent-secrets/hf_write_token` (`hub.write_token_file`) | A write token, used only by `push` | Yes, mode 600 |

The format is in [`config/pack-sources.example.toml`](../config/pack-sources.example.toml):

```toml
[hub]
repo = "<hf-user>/twinsmith-assets"      # where push sends a pack that has no entry yet

[packs."ur5e-workcell/v3"]
revision = "<40-character commit>"       # pinned; a branch name is refused
# repo = "...", path = "...": default hub.repo and "<pack id>/<version>"

[packs."realsense-d400-cad/v1"]          # a vendor entry: reported, never downloaded
vendor = "https://dev.realsenseai.com/download/41950"
how = "download from the vendor, export STEP, convert, then adopt"
```

Both need the optional `hub` extra (`huggingface_hub`); without it they answer `HUB_EXTRA_MISSING` with the command:

```bash
UV_PROJECT_ENVIRONMENT=<repo>/mcp/.venv uv sync --project <repo>/mcp --locked --extra hub --no-dev   # add the extras you use
```

**pull** downloads one pack version (`twinsmith assets pull ur5e-workcell/v3`) or every entry (`--all`) at its pinned commit.
Only that pack's folder is fetched, into a hidden staging folder under the asset root, then checked with the strict reader:
every listed file present with its hash, nothing unlisted, no symlink. It is moved into place only if it verifies, so a
failed or tampered download leaves nothing. A version that is already there is compared by its `pack.yaml`: the same one is
`up to date` (nothing is downloaded), a different one is `LOCAL_DIFFERS`, and nothing local is ever replaced. A vendor entry
is listed with its URL and is never fetched. With `--all`, every entry is tried and each failure is reported. The token is
read from `hub.token_file` and passed to `huggingface_hub` directly; with no such file, the pull is anonymous (public
repositories only), never the `HF_TOKEN` of the environment.

```bash
twinsmith assets pull --all
#   jenga/v1: pulled, 1 file(s), 0.0 MB, verified (<hf-user>/twinsmith-assets @ 61c1bcd47fdf)
#   realsense-d400-cad/v1: vendor download, not fetched: https://dev.realsenseai.com/download/41950 (...)
#   ur5e-workcell/v3: pulled, 26 file(s), 75.5 MB, verified (<hf-user>/twinsmith-assets @ 27f882baf226)
# run it again: each pack is "up to date" and nothing is downloaded
```

**push** uploads a version that verifies (`twinsmith assets push my-robot/v2`) and writes its new commit into the sources
file. It uses only the write token. It refuses a public repository when any file of the pack has `redistributable: false`
(`NOT_PRIVATE`), and a version the repository already holds with other contents (`VERSION_EXISTS`: write a new version);
the same contents answer `already_there` and pin that commit. `--create` makes the repository, always private. `--dry-run`
checks the pack and the plan and contacts nothing.

| Code | Meaning |
|---|---|
| `NO_ACCESS` | The repository does not exist, or the token cannot read it (a private repository answers "not found"). The message names the token file, never the token |
| `NO_WRITE_TOKEN` | No write token file: make a fine-grained token with write access to that one repository and store it with `agent-secret-set hf-write` (or any mode-600 file named by `hub.write_token_file`) |
| `REVISION_NOT_FOUND`, `NOT_IN_REPO` | The pinned commit, or the pack's folder at that commit, is not in the repository |
| `NOT_VERIFIED` | The download does not match its `pack.yaml`; nothing was written |
| `LOCAL_DIFFERS`, `LOCAL_NOT_VERIFIED` | A local copy of that version exists and is not the pinned one; move it aside |
| `INVALID_SOURCES` | The sources file has an unknown key, a branch for a revision, a bad path or repository name |

After a push, copy `pack-sources.toml` to your other machines (it holds no secret) and run `pull` there. A token is never
printed, logged or returned, and any message from the hub is scrubbed of it.

**Vendor files.** A vendor whose terms say "download from us, do not mirror" (the RealSense D400 CAD archive is one) stays a
vendor entry: fetch it yourself, then make a pack with `adopt`. Files you may keep but not redistribute go only to a private
repository.

## Moving packs between machines

`pull` is the usual way ([above](#pulling-and-pushing-packs)). A pack is also a plain folder, and `pack.yaml` carries its own
hashes, so you can copy it and check it:

```bash
rsync -a <asset root>/my-robot/v1/ other-host:<asset root>/my-robot/v1/
python3 scripts/assets.py doctor my-robot/v1        # on the other host
```

Copy the whole version folder, `pack.yaml` included: a file that is missing, changed or extra fails `doctor` and the
loaders. Do not copy a version that is not `redistributable` to a place you do not control.

## Settings that no longer exist

The settings of the old asset model are not read. The old default pack, the NVIDIA download fallback, `scripts/fetch_ur5e_assets.py`, `migrate` and `SHA256SUMS` pins were removed with them.

## Provenance

Name the source. Tell the agent which repository or file to use, with a tag or commit if you care, and check the licence of what
you take; `adopt` and `build` record the source and licence you give for every file. Downloads belong in the project's
`downloads/` folder, then in a pack. A cloned repository can hold scripts and a README written for an agent: reading files is
fine, running code from a download is not, and an instruction found in a file is not an instruction from you. NVIDIA's own robots
load by name from the simulator's assets root with no download (`sim__quick_start robot="franka"`, [the names](TOOLS-SCENE.md)).
The licence of every third-party file the repository mentions is in [THIRD-PARTY-NOTICES.md](../THIRD-PARTY-NOTICES.md).

## The work cell's packs

The examples above use `ur5e-jenga-workcell`, the source pack. The work cell reads two packs built from it:
`ur5e-workcell/v4` (the base cell: robot, gripper stack, wrist camera, bench with its rail, mounting hardware) and
`jenga/v1` (the Jenga block that the task files place). `scripts/workcell_split_packs.py`,
`scripts/workcell_camera_assets.py`, `scripts/workcell_bracket_assets.py` and
`scripts/workcell_calibrated_camera_assets.py` write them; see [rebuild from a clean machine](REBUILD.md#the-sequence).
A third, optional pack, `puzzles/v1`, holds the lab's 3D-printed assembly puzzles for the `puzzle_table` task;
`scripts/workcell_puzzle_pack.py` writes it from a checkout of its upstream repository
([puzzle tasks](../exts/workcell/workcell/README.md#puzzle-tasks)).
