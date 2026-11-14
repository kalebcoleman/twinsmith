# Importing any robot or asset file

One tool brings a robot or object into a scene: **`assets__import_path`**. Give it a file (or a folder that holds one) and it
works out the format, checks it, converts it when it has to, and imports it. It replaced three per-format import tools, so an agent has one place to look; an approved root is still addressable with
`root_id` and `relative_path`. For how approval and asset packs work, see [ASSET-PACKS.md](ASSET-PACKS.md);
for every tool's arguments, [TOOLS-ARGUMENTS.md](TOOLS-ARGUMENTS.md); for what is planned next, [open items](OPEN-ITEMS.md#assets-and-workspace).

```text
assets__import_path  source_path=<file or folder>  prim_path=/World/robot
                     [format=auto|usd|urdf|xacro|sdf|mjcf|mesh|map]
                     robots:  [xacro_args={...}] [sdf_model=<name>] [fixed_base] [default_drives] [joint_targets={...}]
                     meshes:  [scale] [up_axis=y|z] [as_collider=none|convex_hull|convex_decomposition] [mass]
                     maps:    [wall_height]
                     [position] [orientation] [frame]
```

Or in one call that also builds the scene: `sim__quick_start robot=<absolute path to a file or folder>` (any format below).
For example, `sim__quick_start {"robot":"/approved/arm/model.xml"}` returns the imported robot's
articulation and joints in `imported`. Use `assets__roots` to find approved paths. To set initial
joint targets or address a file with `root_id` and `relative_path`, use `assets__import_path` instead.

## Set up, once

1. **Isaac Sim** with this repository's MCP server (`./install.sh`, see [LINUX-SETUP.md](LINUX-SETUP.md)). The URDF and MJCF importers and the mesh converter are Isaac Sim's own. Representative formats were exercised on Isaac Sim 6.2 and 5.1; see [Tested](#tested) for scope and limits. Nothing else is needed for USD, URDF, xacro, SDF, meshes or maps: **no ROS, no `xacro` program, no Gazebo, no internet.**
2. **Put the file where the tools may read it.** Either in a verified pack version of your asset root (`<root>/<pack id>/<version>/`, made with `assets__pack action=adopt`; loads with no approval) or in any folder you approve once, as a trusted root. Approve the **smallest folder that holds the files** (one robot's folder, not your whole downloads or projects folder): every import copies the approved folder, up to 1000 files and 256 MiB. An agent cannot approve anything. When it needs approval, the refusal prints the exact command; you run it, then restart the MCP server and Isaac Sim, because both read the approval file at startup.
3. **Ask.** For example: *"Load the UR10e from <folder> and bend two joints"*, *"Put the Duck from <folder> in the scene as a prop that falls"*, *"Make a warehouse with a robot in it"*. The agent finds `assets__import_path` and picks the format.

## What it can do, in short

| You say | It does |
|---|---|
| Load this robot file (any of USD, URDF, xacro, SDF, MJCF) | Converts it on the host without running its code, imports it, and reports joints, limits, drives, placement and warnings |
| Load this folder | Finds the entry file (Gazebo model, ROS package, MuJoCo or SO-100 style folder) or lists the candidates |
| Load this mesh as a prop that falls / is solid | Converts glTF, GLB, OBJ, STL, FBX or PLY, with scale, up axis, a collision shape and mass |
| Turn this Nav2 map into walls | Wall boxes with colliders at the map's own coordinates |
| Save this robot as a URDF | `sim__stage__save format=urdf`, with the list of what a URDF cannot hold |
| Check this first | `assets__dependencies` lists the files a file needs, for every format, with no import |
| Start in a pose | `joint_targets` on the import |
| One call, scene and all | `sim__quick_start robot=<path>`; `sim__quick_start environment=warehouse` for a warehouse with a robot |

## Formats

| You have | `format` | What happens | Notes |
|---|---|---|---|
| USD, USDA, USDC, USDZ | `usd` | Referenced into the scene with its dependencies copied and checked | Textures and sublayers must be inside the approved folder |
| URDF (`.urdf`) | `urdf` | Converted with Isaac Sim's URDF importer | `package://` and relative meshes (STL, OBJ, DAE) resolved inside the folder |
| **xacro** (`.xacro`, `.urdf.xacro`) | `xacro` | Expanded on the host to a URDF, then imported as above | Pass the file's `$(arg ...)` values in `xacro_args`, for example `{"ur_type": "ur10e"}` |
| **SDF / Gazebo** (`.sdf`, `.world`) | `sdf` | Converted on the host to a URDF, then imported | A world with several models needs `sdf_model`; the refusal lists the names |
| MuJoCo (`.xml`) | `mjcf` | Converted with Isaac Sim's MJCF importer | Includes, meshes, textures and height fields followed inside the folder |
| **A mesh prop** (`.gltf` `.glb` `.obj` `.stl` `.fbx` `.ply`) | `mesh` | Converted with Kit's asset converter, referenced under your prim | See [Meshes](#meshes-and-props) |
| **A ROS / Nav2 map** (`.yaml` + `.pgm` or `.png`) | `map` | Occupied cells become wall boxes with static colliders | See [Maps](#ros-maps) |
| A folder | `auto` | The folder names its own entry file (see [Folders](#folders-and-bundles)) | |

A USD robot is usually several files: a top-level file whose variants and payloads pull in a physics layer and a base
layer that holds the visual meshes. Import the **top-level** file, so they compose together; importing the physics layer
alone gives a robot that simulates and cannot be seen. The result says what composed:

- `visuals.drawn_geometry` counts the geometry that will be rendered, and `visuals.expanded_instances` the instanced
  `visuals` prims that were expanded (a session-layer override) so each link's mesh follows physics. A result with
  physics and nothing to draw carries a `warnings` entry and the asset's `variant_selections`.
- A layer named only by a variant that is **not selected** (an optional gripper the pack does not ship) does not block
  the import: it is listed under `unselected_variant_dependencies`. A missing layer the selected variants do use, and
  any missing texture or material, is still refused with `DEPENDENCIES`.

Not importable yet: CAD (STEP, IGES, JT), Gaussian splats, heightmap terrain and Collada (`.dae`) as a prop of its own (Collada works as
the meshes of a URDF robot; as a prop it is refused with a message that says to convert it to glTF or OBJ). Isaac Sim ships converters for
CAD and splats; they are on the plan in [open items](OPEN-ITEMS.md#assets-and-workspace).

## What is read and what is never run

The same rules hold for every format.

1. **The host reads the file first, and runs nothing from it.** There is no `xacro` program, no Python from a file, no plugin, no download and no URL.
2. **Every file it names stays inside the folder you approved.** An absolute path, a URI, a backslash, a symlink or `..` out of the folder is refused, percent-encoded or not.
3. **The approved folder is copied to a private snapshot and verified.** The importer reads only the snapshot. A converted file (xacro, SDF) is written into that snapshot, hashed like the rest, and becomes its entry.
4. **The result says what it did.** The reply has an `import` block: the format, what was converted from what, the model chosen, the warnings and what was dropped.
5. **Approval stays on.** A path outside an approved root or your asset root is refused with the exact command the user runs. See [ASSET-PACKS.md](ASSET-PACKS.md#the-tools).

### xacro: what the expander reads

xacro is XML with `${expression}`, `$(command)`, macros, includes and conditions. The real program evaluates expressions with Python's
`eval`, so it can run anything; it is never used. `xacro_lite` parses expressions itself and evaluates only:

- arithmetic, comparisons, `and`/`or`/`not`, `a if c else b`, lists, dicts and indexing, and `math` (`pi`, `sin`, `radians`, `sqrt`, ...);
- the built-ins `abs min max round int float str len bool`, and the string methods `replace lower upper startswith endswith split strip join`;
- `xacro:property`, `xacro:arg`, `xacro:macro` (defaults, `*block` parameters, `^|` inheritance), `xacro:insert_block`, `xacro:include` (relative, `$(find pkg)` and `*` globs), `xacro:if` and `xacro:unless`;
- `$(arg)`, `$(find)`, `$(optenv)` (only its default is returned: the real environment is never read), `$(dirname)`, `$(eval)`;
- `xacro.load_yaml(file)` on a YAML file inside the folder (maps, lists, scalars, `!degrees`, `!radians`).

Everything else is refused **by name**, not skipped: `__import__`, `open`, attributes that start with `_`, lambdas, comprehensions,
`xacro:element`, `$(env)`, `$(anon)`. Powers, repeated strings, format widths, macro depth (64), include depth (16), the number of steps, the bytes parsed and substituted (256 MiB each) and the size of the output (200,000 elements, 64 MiB) are
bounded, so a hostile file ends with a `LIMIT` refusal and not a hang. `$(find pkg)` finds the package by its `package.xml` inside the folder; approve the package's folder, or the folder that holds the packages it uses.

Checked against the real program: the Universal Robots UR10e description expands to the same links, joints, limits (including the `!degrees` ones), meshes and inertias as the system `xacro`.

### SDF: what is converted and what is dropped

Converted: one model's links (pose, mass, inertia), visual and collision geometry (box, sphere, cylinder, mesh with scale, a plane as a thin box, a capsule as a cylinder),
colours, and joints (fixed, revolute, continuous, prismatic) with axes, limits, damping and friction. A revolute joint with no real limits becomes continuous (a wheel).
A **ball** joint becomes three continuous joints with two tiny links between them, and a two-axis joint becomes continuous; each is a warning in the result. Meshes are found by `model://name/path` or a relative path, inside the folder only.

Dropped, and listed under `sdf_dropped`: plugins (never run), sensors, lights, physics settings, other models, frames, and geometry a URDF cannot hold (heightmap, polyline, image).
A model whose joint names the `world` as its parent, or that is `<static>`, is imported with a fixed base; a model that moves defaults to `fixed_base=false`.

### Meshes and props

`assets__import_path source_path=<duck>.gltf prim_path=/World/duck as_collider=convex_hull mass=0.5 position=[0,0,1.5]` converts the mesh and drops it as a rigid body that falls and rests on the ground.

- **Units** are metres. A file in other units: `scale` (0.001 for millimetres). The result gives `size_m` so you can see when it is wrong; the `validation` block flags a model smaller than 5 mm or larger than 500 m.
- **Up axis:** glTF, GLB, FBX and OBJ are read as Y-up and rotated to the scene's Z-up; STL and PLY as Z-up. `up_axis=y|z` says what the file really is.
- **Collision:** `none` (default, looks only), `convex_hull`, `convex_decomposition`. `mass` makes it a dynamic rigid body (a convex hull collider unless you choose one).
- **What it names stays in the folder.** A glTF's buffers and images, an OBJ's material library and its textures are checked: one that is absolute, a URL, or climbs out of the folder is refused (`MESH_REFERENCE`); one that is simply absent is a warning and the mesh comes in without materials. FBX is binary, so its texture paths cannot be checked and it comes in without materials.
- Animations, cameras and lights in the file are ignored.

### ROS maps

`assets__import_path source_path=<folder>/map.yaml prim_path=/World/map wall_height=1.5` reads `image`, `resolution`, `origin`, `negate` and `occupied_thresh` from the YAML, reads the PGM (or a PNG) from the same folder, and builds the occupied cells as the fewest wall boxes the image allows (static colliders at the map's own coordinates). It is the inverse of `sim__occupancy_map`. A yaw in `origin` is not applied; the result warns and `orientation` on the import turns the map. There is a limit of 4,000 boxes: a noisy image is refused with a hint to clean it. Add a ground plane separately.

### Export

`sim__stage__save format=urdf filename=my_robot robot_path=/World/robot` writes a folder `my_robot/` with `my_robot.urdf` and `meshes/` (OBJ) through Isaac Sim's URDF exporter. It never overwrites. The result lists what a URDF cannot hold (drive gains, physics materials, sensors, material graphs). Approve the folder and import it again with `assets__import_path` to check the round trip: the exporter writes OBJ material files the importer then reports as absent, so colours may be lost.

## Folders and bundles

Pass a folder as `source_path` and the importer picks the entry file, or refuses and lists the candidates (never a guess between equals).

| The folder holds | Chosen |
|---|---|
| `model.config` (a Gazebo model) | the SDF it names, else `model.sdf` |
| `package.xml` (a ROS package) | a `.urdf.xacro` robot file, or a `.urdf`; macro and common files are not candidates |
| MuJoCo `.xml` files (a Menagerie robot, a LeRobot MJCF) | the robot model that a `scene.xml` includes; a lone model; `_mjx` variants only if nothing else |
| a URDF under `urdf/` (a LeRobot or Hugging Face arm repo) | that URDF |
| USD files | one named `scene`, `main`, `stage` or after the folder |

A ROS map folder (a YAML with `image` and `resolution`) is a candidate too. Mesh files are candidates only when nothing else is there. A folder with two equal candidates (two URDFs, say) is refused with `AMBIGUOUS_BUNDLE` and a `candidates` list; pass `source_path` with the file you want.

## What the result tells you

Every robot import returns what an agent needs to act without another call: `articulation_root`, `base`, `joints` (name, kind, limits in radians or stage units, drive gains), `placement`, `warnings`, and a `validation` block of checks (geometry present, finite bounds, a sane size, rigid bodies, articulation root, joints). `joint_targets={"elbow_joint": 1.5}` starts the robot in a pose (drive targets). A mesh returns `size_m`, `meshes`, `collider`; a map returns `walls` and `extent_m`. `assets__dependencies` reads every format above on the host (xacro, SDF, meshes, maps) and lists the files a file needs before you import it; give a xacro its `xacro_args` on the import, not on the check.

## From a shell

A tool can be called once from a shell, with no MCP client, against a bridge: `python3 mcp/twinsmith/client.py assets.import_path '{"session_id": "...", "stage_id": "...", "source_path": "...", "prim_path": "/World/r"}' --port <port>`. The test helper in the agent-testing skill accepts `./call.py --port <port> <tool> '<json>'` the same way; no `use_bridge` is needed with an explicit port.

## When it refuses: what each code means

| Code | Meaning | What to do |
|---|---|---|
| `ROOT_NOT_APPROVED` | The file or folder is outside every approved root and the asset root | Run the printed command (smallest folder), restart the MCP server and Isaac Sim |
| `AMBIGUOUS_BUNDLE` | A folder holds several possible robots | Pass `source_path` with the file you want (the reply lists them) |
| `XACRO_ARG` | A xacro file needs `$(arg ...)` values | Give them in `xacro_args`; the reply names every missing one |
| `SDF_MODEL` | A Gazebo world has several models | Pass `sdf_model` with the model's name |
| `MESH_REFERENCE` | A mesh names a file outside the folder (absolute, URL, `..`) | Put the files beside the mesh; this is refused on purpose |
| `UNITS_MISMATCH` | The stage's up axis or units differ from the asset's | Start a scene that matches (`sim__quick_start up_axis=... meters_per_unit=...`) |
| `UNSUPPORTED_FORMAT` | Not a format here (CAD, splats, `.dae` as a prop, a bare `package.xml`) | Convert it to glTF, OBJ or URDF first |
| `LIMIT` | The approved folder has over 1000 files or 256 MiB, or a file is too big | Approve a smaller folder |
| `QUOTA` | 256 private snapshots exist in staging (each import uses two), or the disk has under about 2.3 GB free | Stop the simulator, then clear the staging folder |
| `PACKAGE_MISSING` | A `$(find pkg)` or `package://` needs a package that is not in the approved folder | Approve the folder that holds it |
| `MJCF_51_DEFAULTS` / `MJCF_51_MODEL` | Sim 5.1's older importer cannot safely normalize that combination of defaults or mesh references | Simplify the MJCF layout, or use Sim 6.2 |

## Physics after the import

An import authors a valid robot; it does not make it stand or walk. What to expect:

- **Base.** `fixed_base=true` (the default for URDF and xacro) bolts the root to the world, so a humanoid or quadruped then hangs in the air. For a robot that drives or walks pass `fixed_base=false`; it then falls under gravity.
- **Drives.** URDF, xacro and SDF have no gains, so the import authors position drives that hold a pose (`default_drives`). **Continuous joints (wheels, casters) get no holding stiffness**, only damping, so a velocity command turns them: `sim__joints__command velocities={"left_wheel_joint": 6}`. A wheel that is held by a position drive does not roll.
- **MuJoCo torque motors.** A Menagerie robot whose actuators are `<motor>` torque motors (Go2, H1) imports with zero stiffness and damping: it collapses under gravity until you set gains or drive it with a controller. A model with position actuators (Panda, UR5e) imports with drives. This is faithful to the model, so the result **lists the undriven joints and warns**; `default_drives=true` gives them a generic holding drive (stiffness 100, damping 10 per revolute joint) as a starting point.
- **MuJoCo tendons and equalities.** Imports warn when the model declares fixed or spatial tendons: Isaac Sim may change or omit tendon stiffness, coupling or actuation. Sim 5.1 restores simple identity equalities between connected prismatic joints as PhysX mimic constraints; nonlinear, shifted, ambiguous or other cases are left to the importer and reported. On the Panda MJCF, 5.1 authored stiffness 1 for a zero-default tendon and 6.2's converted USD exposed no corresponding PhysX tendon attributes. With the equality repair, commanding one finger moved the other on both builds. This does not establish contact, actuator or MuJoCo solver fidelity.
- **No balance.** A humanoid or quadruped with a free base and gains still falls over unless a controller or a trained policy keeps it up. Walking is an Isaac Lab job (`isaaclab__train`, `isaaclab__play`): the stock `Isaac-Velocity-Flat-G1` task uses a free base (`fix_root_link=False`) and learns to balance, with the base-contact termination counting falls; it ran on the Newton backend, not PhysX.
- **Ground.** Leave `position` out to have a robot raised out of a ground plane (up to 3 m); a `position` you give is used as is, with a warning if it starts inside the ground.
- **Newton.** Before importing a USD for a Newton session, run `assets__validate` on it. It finds triangle-mesh colliders too large for Newton's contact buffer, zero-force drives, nested articulations and massless bodies from the file alone. `assets__mesh action=split` adds a separate box or hull collision layer and keeps the detailed mesh for rendering. See [asset packs](ASSET-PACKS.md#preparing-meshes-and-checking-an-asset).

## Limits to know

- **Approval is always on.** There is no switch to turn it off.
- **Snapshot quota.** Staging holds 256 snapshots and each import uses two, so about 128 different imports before a cleanup is needed; importing an unchanged folder again reuses its snapshot. A snapshot can be up to 256 MiB and nothing deletes them for you, so a long-used staging folder can grow large. A new one is refused when the disk would have under about 2 GB left.
- **Collada (`.dae`)** works as a URDF robot's meshes but not as a prop of its own; CAD and Gaussian splats are not supported.
- **FBX** comes in without materials (its texture paths cannot be checked). OBJ and glTF come in without materials if a named file is absent, with a warning.
- **The SDF reader** converts one model at a time and drops plugins, sensors, lights and physics settings (listed in the result). A ball joint becomes three continuous joints.
- **Not a physics guarantee.** An import authors a valid robot; whether it stands, drives or walks depends on its drives, its base and a controller (see above).

## Tested

On simultaneous headless Sim 5.1 and 6.2 processes, small USD, URDF, xacro, SDF, glTF, ROS map and Go2 MJCF imports passed. SO-101 and Panda also imported and moved; the Panda finger-coupling check is described above. These samples do not establish every asset variant or controller. The 5.1 adapter merges only unambiguous named defaults and gives colliding mesh stems unique names in a private copy; the source stays unchanged. It refuses ambiguous cases with a reason. URDF export is refused on 5.1 because that exporter wrote an empty robot. The remaining results below are the earlier 6.2 run.

Run with a visible simulator window on Isaac Sim 6.2, from real downloads, not made-up files.

| Format | Asset | Result |
|---|---|---|
| xacro | Universal Robots UR10e (`ur.urdf.xacro`, `ur_type=ur10e`) | imported, 14 meshes mapped, 6 joints, moved to targets |
| SDF | Gazebo `diff_drive.sdf`, model `vehicle_blue` | imported from a world of three models, 5 joints; drives forward once its wheels are free |
| MJCF | Menagerie UR5e, Panda, Go2, H1 | imported, joints commanded; Go2 and H1 collapse without a controller, as above |
| URDF | UR10e with `.dae` meshes | meshes visible |
| Folder | Menagerie `unitree_go2`, Gazebo model folders, ROS packages | entry file chosen; ambiguous folders listed |
| glTF | Khronos Duck (`Duck.gltf`, as a dynamic prop with a convex hull) | imported, 1.65 m (the file is in its own units; `scale` fixes it), falls and rests |
| STL, OBJ | UR10e `base.stl`; Menagerie Go2 `base_0.obj` | imported, Z-up and Y-up as stated; the OBJ's absent `.mtl` was a warning |
| Hostile glTF | buffer `../../../../etc/passwd`, image `file:///etc/hosts` | refused (`MESH_REFERENCE`), nothing read |
| ROS map | Nav2 `tb3_sandbox.yaml` | 870 occupied cells became 244 wall boxes |
| Export | the imported UR10e to URDF, then imported again | works; the OBJ material files were reported absent on re-import |
