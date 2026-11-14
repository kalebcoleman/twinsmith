"""Versioned public contracts; stdlib only, shared by host catalog and Kit pack."""


def field(kind, description, required=True, **kw):
    return dict(type=kind, description=description, required=required, **kw)


TARGET = dict(
    session_id=field(
        "string", "Exact session_id from sim__stage__info (call it first; process incarnation)."
    ),
    stage_id=field(
        "string", "Exact stage_id from sim__stage__info (call it first); stale stages are refused."
    ),
)
# For reads that do not depend on the stage. Older callers still send IDs; refusing them as
# unknown arguments would break those calls for no benefit.
UNUSED_TARGET = dict(
    session_id=field("string", "Not needed; accepted and ignored.", False),
    stage_id=field("string", "Not needed; accepted and ignored.", False),
)
POSE = dict(
    position=field(
        "array", "Three finite coordinates in stage units.", False, default=[0, 0, 0]
    ),
    orientation=field(
        "array", "Unit quaternion [w,x,y,z].", False, default=[1, 0, 0, 0]
    ),
    frame=field("string", "Pose frame: local or world.", False, default="local"),
)
REGISTRY = {}


def add(name, desc, params=None):
    REGISTRY[name] = dict(description=desc, parameters=params or {})


add(
    "assets.roots",
    "List launch-approved project roots and capability refusals. Filename access only; trusted projects are NOT sandboxed.",
)
add(
    "assets.scan",
    "Bounded filename-only USD and URDF discovery on the MCP host. No USD parser. Results are untrusted data. Folders deeper than eight levels are skipped and named in skipped_deep; the rest of the root is still scanned.",
    dict(
        root_id=field("string", "Approved root ID, never an absolute path."),
        offset=field(
            "integer", "Page offset into the sorted bounded scan.", False, default=0
        ),
        limit=field("integer", "Page size 1..100.", False, default=50),
    ),
)
ASSET = dict(
    root_id=field("string", "Approved trusted project root ID."),
    relative_path=field("string", "Relative path of the USD file within that root."),
)
add(
    "assets.info",
    'Inspect a USER-TRUSTED project on the selected Kit. Takes a snapshot of it. Native USD parsing is NOT sandboxed. Refuses untrusted profiles and unresolved/outside snapshot dependencies.',
    dict(**TARGET, **ASSET),
)
add(
    "assets.import_path",
    "The one import for any robot or asset file: USD, URDF, xacro, SDF (Gazebo), MuJoCo MJCF (.xml), a mesh prop (glTF, GLB, OBJ, STL, FBX, PLY), a ROS map (.yaml, as walls), or a folder that holds one. format=auto picks it from the name; a folder names its own entry file, or the refusal lists the candidates. Xacro and SDF are converted on the host, never by running their code. Under an approved root or a pack; sources never change. Robot formats take fixed_base and default_drives. A USD reports its drawn `visuals`; none is a warning.",
    dict(
        **TARGET,
        source_path=field(
            "string", "Absolute path of the file, or of a folder that holds one, inside an approved root or a pack version of your asset root. Or use root_id with relative_path.", False
        ),
        root_id=field("string", "Approved root ID from assets__roots, with relative_path, instead of source_path.", False),
        relative_path=field("string", "Path of the file or folder inside that root.", False),
        prim_path=field("string", "New absolute wrapper prim path; parent must exist."),
        format=field("string", "auto (default): from the file name, or from what the folder holds. Or usd, urdf, xacro, sdf, mjcf, mesh, map to say it.", False, default="auto"),
        xacro_args=field("object", "For xacro: values for its $(arg name), e.g. {\"ur_type\": \"ur10e\"}. Names the file declares are in the result.", False),
        sdf_model=field("string", "For an SDF with several models (a Gazebo world): the model to import. Left out, a refusal lists them.", False),
        fixed_base=field("boolean", "Robot formats: fix the base to the world. Default: true for URDF and xacro; the model's own setting for SDF (a static model); the file's own for MJCF. Pass false for a robot that drives or walks.", False),
        joint_targets=field("object", "Robot formats: start in a pose. {joint name: radians, or stage units for a sliding joint}, set as drive targets; unknown names are listed in the warnings.", False),
        scale=field("number", "Mesh files: multiply the size (the file's unit to metres, e.g. 0.001 for millimetres). Default 1: metres.", False),
        up_axis=field("string", "Mesh files: the file's own up axis, y or z. Default auto: glTF, GLB, FBX and OBJ are Y-up, STL and PLY Z-up; the result is Z-up.", False),
        as_collider=field("string", "Mesh files: none (default, looks only), convex_hull or convex_decomposition collision shapes.", False),
        wall_height=field("number", "ROS map (.yaml): wall height in metres. Default 1.", False),
        mass=field("number", "Mesh files: kilograms; makes it a dynamic rigid body (with a convex hull collider unless you pick one).", False),
        default_drives=field("boolean", "Author drives that hold a pose. URDF, xacro, SDF: default true (they have none). MJCF: default false; true gives joints with no drive (torque motors) a generic holding drive. Wheels and casters stay free.", False),
        position=field("array", "Three finite coordinates in stage units. Omit it for a robot format: the robot is then raised out of any ground plane (up to 3 m); a given position is used as is, with a warning if it starts inside the ground. USD files go to the origin.", False),
        orientation=field("array", "Unit quaternion [w,x,y,z].", False, default=[1, 0, 0, 0]),
        frame=field("string", "Pose frame: local or world.", False, default="local"),
    ),
)
add(
    "sim.stage.info",
    'Identify the open stage; call this first. Reads process incarnation, stage identity, units, timeline and bounded prim count.',
)
add(
    "sim.stage.open",
    "Open a USD file as the current stage. Accepts .usd/.usda/.usdc/.usdz under your home or this simulator's output folder, like File > Open, with the timeline stopped. Refuses to drop unsaved edits unless discard_changes=true. Reports stale saved LiDAR render products and never changes the file. Returns the new session_id and stage_id for later sim__* calls; run_python sessions reset.",
    dict(
        **TARGET,
        path=field("string", "USD file to open: absolute, under your home or this simulator's output folder. A leading ~ is expanded."),
        discard_changes=field("boolean", "Open even if the current stage has unsaved edits (they are lost).", False, default=False),
    ),
)
add(
    "sim.quick_start",
    "One call that builds a working scene, needing no IDs: a ground plane, physics scene, lights and a camera aimed at it, in the current stage if it is new and empty, otherwise a new one. With robot, called again on a scene it built, it adds the robot and keeps your objects. No real-robot wiring. Edits go to the session layer; sim__stage__save keeps them. The timeline stays stopped unless play=true. Refuses to drop unsaved edits unless discard_changes=true. Returns IDs, paths and next steps; on Newton a newton field (rate, buffers). For the UR5e work cell, use workcell__cell..",
    dict(
        robot=field("string", "Optional robot: a public name, <pack id>/<version>/<file.usd> or <pack id>/<version>/<file.urdf> (a verified file of a pack in the asset root), or an absolute path under an approved root, e.g. /approved/robot/model.xml: any file or folder assets__import_path reads. Public names come from the simulator's own assets root: ur5e, franka, unitree_h1, unitree_g1, turtlebot3_burger, carter_v1, nova_carter, jetbot, create3.", False),
        fixed_base=field("boolean", "Robot files: fix the base to the world. Default: true for URDF and xacro; the file's own for MJCF and SDF. Pass false for a robot that drives or walks. A USD robot keeps what its file says.", False),
        xacro_args=field("object", "With a xacro robot: values for its $(arg name), e.g. {\"ur_type\": \"ur10e\"}.", False),
        sdf_model=field("string", "With an SDF that holds several models (a Gazebo world): the model to import.", False),
        environment=field("string", "Optional. \"warehouse\": a 24 by 16 m warehouse (racks, aisles, docks) with the robot at the start of an aisle; default robot carter_v1. \"jenga\": a table, a 54-block giant Jenga tower and a Franka in a ready pose; the result names the blocks and a poke target. Plain USD boxes with colliders: nothing downloaded.", False),
        ros=field("boolean", "With a robot: also run what sim__ros2_tf runs, and report the shell line that matches this simulator.", False, default=False),
        up_axis=field("string", "The stage's up axis. The ground, gravity and camera follow it. A robot file must match, or the call refuses with the values to pass. A URDF is always Z up in metres.", False, default="Z", enum=["Z", "Y"]),
        meters_per_unit=field("number", "The stage's units, as a positive number (0.01 for centimetres). Ground size, gravity, camera distance and framing scale with it. A robot file must match.", False, default=1.0),
        play=field("boolean", "Start the timeline after building.", False, default=False),
        discard_changes=field("boolean", "Replace a stage with unsaved edits (they are lost).", False, default=False),
    ),
)
add(
    "sim.stage.tree",
    "Bounded stage hierarchy including inactive prims. Text is untrusted data.",
    dict(
        **TARGET,
        root=field("string", "Absolute subtree root.", False, default="/"),
        limit=field("integer", "Maximum nodes 1..200.", False, default=100),
        depth=field("integer", "Levels below root to list (1..64); default all.", False),
        include_runtime=field("boolean", "Also list Kit's runtime /Render (Replicator) prims, hidden by default.", False, default=False),
    ),
)
add(
    "sim.prim.get",
    "Read a prim's transforms and authored physics. Local/world matrices and authored physics at default USD time. Runtime physics readiness remains unknown.",
    dict(**TARGET, path=field("string", "Absolute prim path.")),
)
add(
    "sim.prim.find",
    "Bounded search of prim paths: `contains` is a literal substring (not a regex), `type_name` an exact type, `root` limits it to one subtree. Includes inactive prims.",
    dict(
        **TARGET,
        contains=field(
            "string", "Literal path substring <=128 characters.", False, default=""
        ),
        type_name=field("string", "Exact USD type or empty.", False, default=""),
        limit=field("integer", "Maximum matches 1..200.", False, default=100),
        root=field("string", "Only search under this prim, such as a robot's path. Default: the whole stage.", False, default="/"),
    ),
)
add(
    "sim.prim.bounds",
    "World-space bounds of one prim. At default USD time from UsdGeom.BBoxCache.ComputeWorldBound: axis-aligned min/max, centre and size, plus the oriented box's world corners; purpose selects default, render, proxy or guide geometry. Stage units; empty says the prim has no boundable content. Authored USD, not a runtime physics observation.",
    dict(
        **TARGET,
        path=field("string", "Absolute prim path."),
        purpose=field(
            "string",
            "Geometry purpose whose bounds to measure.",
            False,
            default="default",
            enum=["default", "render", "proxy", "guide"],
        ),
    ),
)
add(
    "sim.prim.measure",
    "Distance between two prims' world bounds. At default USD time: centre-to-centre and the shortest gap (0 when the axis-aligned bounds overlap), in stage units, with each prim's axis-aligned bound. Refuses when either prim has no bounds, naming it and pointing to sim__prim__bounds. Authored USD, not runtime physics.",
    dict(
        **TARGET,
        path_a=field("string", "First absolute prim path."),
        path_b=field("string", "Second absolute prim path."),
    ),
)
add(
    "sim.camera.list",
    'List Camera prims. Under root with focal length and aperture (mm), clipping range (stage units and metres), projection, world pose and whether each is the active viewport camera. Bounded by limit and a 10,000-prim visit cap; truncated says which. No viewport (headless) reports viewport_available=false instead of guessing. Authored USD, not a rendered image.',
    dict(
        **TARGET,
        root=field("string", "Absolute subtree root.", False, default="/"),
        limit=field("integer", "Maximum cameras 1..100.", False, default=100),
    ),
)
add(
    "sim.prim.create",
    'Create a primitive in the session layer. A Cube, Sphere, Cylinder, Capsule, Cone, Plane, Xform or PhysicsScene. No arbitrary references/attributes. Stopped timeline, existing parent and no conflict required. Dimensions are stage units; optional mass or density authors PhysicsMassAPI on geometry. Undo a created prim with sim__prim__delete; sim__prim__revert undoes the attributes.',
    dict(
        **TARGET,
        path=field("string", "New absolute prim path; every ancestor must already exist (create them with kind=Xform)."),
        kind=field(
            "string", "Cube, Sphere, Cylinder, Capsule, Cone, Plane, Xform or PhysicsScene."
        ),
        size=field(
            "number",
            "Positive cube edge, sphere/cylinder/capsule/cone diameter or height, or Plane width and length.",
            False,
            default=1,
        ),
        rigid=field(
            "boolean",
            "Author rigid body AND collider on geometry.",
            False,
            default=False,
        ),
        collision=field("boolean", "Author a static collider without a rigid body (walls, floors, shelves).", False),
        scale=field("array", "Three positive scale factors [x, y, z]: a 1 m Cube with scale [0.2, 6, 1] is a wall. Very flat floors can stop wheels; the result warns.", False),
        mass=field("number", "Mass in kilograms (needs rigid=true).", False),
        density=field("number", "Density in kg per cubic metre, instead of mass (needs rigid=true).", False),
        material=field("string", "Path of a material from sim__material__create, bound as the prim is made; nothing is created if it cannot be bound.", False),
        **POSE,
    ),
)
add(
    "sim.prim.set_transform",
    'Set a static pose on any Xformable prim. Local or world, in the session layer; reports the pose before and after; undo with sim__prim__revert. Refuses instance proxies (name the instance root), articulation links other than the root (moving a link breaks its joints), animated transforms, Kit runtime prims and a running or paused timeline. Authors one transform override, so source files never change. No claim of runtime teleportation.',
    dict(
        **TARGET,
        path=field("string", "Absolute Xformable prim path."),
        scale=field("array", "Positive XYZ scale in the requested pose frame. Omitted: keep the prim's current local scale, so moving a scaled table does not resize it.", False),
        **POSE,
    ),
)
add(
    "sim.physics.inspect",
    'Report authored collision, body, mass and joints. A bounded report that includes articulations. Does not assert initialized or stable physics. On Newton it adds a newton section (configured buffers, active contacts and constraints against capacity, overflow flags) and warnings CONTACT_BUFFER_FULL, CONSTRAINT_BUFFER_FULL, NEWTON_SOLVER_OVERFLOW, NEWTON_SOLVER_NOT_CONVERGED.',
    dict(
        **TARGET,
        root=field("string", "Absolute subtree path.", False, default="/"),
        limit=field("integer", "Maximum visited nodes 1..200.", False, default=100),
    ),
)
add(
    "sim.physics.raycast",
    'Cast one ray into the PhysX scene. Hits with position, normal, distance, collider and rigid-body paths. all=false returns the closest; all=true returns up to 32 sorted by distance. max_distance 0..100 stage units; direction is normalised and the result says so; scene_ready=false before physics initialises. Newton refuses RAYCAST_UNSUPPORTED. No app update is pumped.',
    dict(
        **TARGET,
        origin=field("array", "Ray origin [x, y, z] in world stage units."),
        direction=field("array", "Ray direction [x, y, z]; normalised before use."),
        max_distance=field("number", "Maximum distance in stage units, >0 and <=100.", False, default=100),
        all=field("boolean", "Return every hit up to 32 instead of only the closest.", False, default=False),
    ),
)
add(
    "sim.physics.contacts",
    'Read current contact pairs: PhysX impulses, or on Newton the last step\'s summed world-frame force in N on collider_a (impulse is null). For up to 32 bodies from the last delivered contact report. Subscribes on first use or Play. Only colliders with PhysxContactReportAPI report; others are listed with the fix, and none reporting refuses CONTACT_REPORT_DISABLED. Needs a playing or paused initialized timeline. Newton refuses CONTACT_QUERY_UNSUPPORTED only if forces are unavailable. No app update is pumped.',
    dict(
        **TARGET,
        paths=field("array", "1..32 absolute rigid-body or collider paths."),
    ),
)
add(
    "sim.body.state",
    "Read a rigid body's world pose while running. Initialized pose on the selected PhysX or qualified Newton backend; fails when stopped/uninitialized or not a body. Position in stage units, quaternion [w,x,y,z]. Optional runtime velocities/mass from tensor API; no authored-USD fallback. PHYSICS_STATE_INVALID identifies invalid observed data, not bad caller input. render_pose compares the poses the renderer draws from (Fabric, USD) with the physics pose; a `warnings` entry means pictures and recordings do not show where the body is.",
    dict(
        **TARGET,
        path=field("string", "Absolute rigid body path; prim and prim_path are aliases. Supply one.", False),
        paths=field("array", "Instead of path: up to 64 bodies read at one instant, returned as bodies.", False),
        prim=field("string", "Alias for path.", False),
        prim_path=field("string", "Alias for path.", False),
        include_dynamics=field(
            "boolean",
            "Include measured world COM linear/angular velocity and runtime mass, including kg conversion.",
            False,
            default=False,
        ),
    ),
)
add(
    "sim.timeline",
    "Play, pause, stop, initialize or reset. initialize briefly plays a stopped timeline then pauses it, so physics may advance; it refuses a playing timeline. Play reports physics_ready. Reset is the call after a failed run: it cancels this pack's jobs and stops the timeline; safe to repeat. sim__simulation__step advances exact steps. Stop rewinds physics and awaits pending render/writer completion. TIMELINE_BUSY or TIMELINE_INTERRUPTED requires inspection before retry. The session edit target is leased until Stop.",
    dict(**TARGET, action=field("string", "play, pause, stop, initialize (briefly play then pause; physics may advance), or reset (cancel jobs, stop, back to the authored start)."),
         resume=field("boolean", "Continue a timeline a script or the GUI paused, without the stop that would end it. Physics then writes where the script set it. With action=play.", False)),
)
add(
    "sim.stage.save",
    'Export a flattened copy of the stage to a new file. Includes session edits, to a new basename (.usda added if absent) in the output root or in `directory` under your home; with format=urdf one robot as URDF plus meshes. Never overwrites or switches the current stage. External texture paths remain dependencies; process-bound render pipelines are omitted, and unknown native pipeline content refuses.',
    dict(
        **TARGET,
        filename=field("string", "New basename, e.g. scene or scene.usda; no other extension, no folders."),
        directory=field("string", "Existing folder under your home to write into instead of the output root (absolute path). Never overwrites.", False),
        format=field("string", "usd (default): a flattened .usda copy of the scene. urdf: one robot as a folder <filename>/ with <filename>.urdf and meshes/ (OBJ), which lists what a URDF cannot hold (drives, physics materials, sensors).", False, default="usd", enum=["usd", "urdf"]),
        robot_path=field("string", "With format=urdf: the robot's articulation root or top prim. Default: the stage's default prim.", False),
        reset_camera_render_handles=field(
            "boolean",
            "Remove session-only ROS camera and LiDAR viewportHandle fields from the copy, so native textures rebuild after restart. This tool's own capture products need no flag. Default false refuses affected saves; true still refuses authored, ambiguous, connected or unavailable bindings. Originals and live stage are unchanged; omitted property paths are returned. This is export policy, not proof of authorship.",
            False,
            default=False,
        ),
    ),
)
add(
    "sim.viewport",
    "Point the active viewport camera, e.g. before a screenshot. path alone frames a prim; path+margin frames its bounds plus margin metres; eye+target places the camera exactly; follow tracks the prim. Edits only the active camera in the session layer, with no undo entry. framed_path is the prim this call framed; the Stage selection is put back afterwards, so the Stage and Property panels keep their own selection. Refuses look-through targets and empty bounds, listing candidates. Does not wait for rendered pixels.",
    dict(
        **TARGET,
        path=field("string", "Absolute prim path to frame or follow.", False, default=""),
        margin=field("number", "Extra radius in metres around the prim's bounding sphere (0..1000), e.g. 1.0 for a small robot that will drive around.", False),
        eye=field("array", "Camera position [x, y, z] in world coordinates (stage units).", False),
        target=field("array", "Point to look at [x, y, z] in world coordinates (stage units); default: the prim's centre.", False),
        follow=field("boolean", "true: follow path while playing (until stopped); false: stop following.", False),
        layout=field("string", "clean: hide the app's panels, menus, grid and selection outline for a recording. default: put them back. Alone, it leaves the camera where it is.", False, enum=["clean", "default"]),
        caption=field("string", "One line (<=120 characters) drawn over the bottom of the viewport, e.g. the run's name, for screenshots and screen recordings; an empty string removes it. Needs a viewport window.", False),
    ),
)

DISABLED = {
    "isolated_untrusted": "Disabled: no verified whole-lifetime GPU Kit filesystem/network/resolver/plugin/script enforcement. No fallback to trusted.",
    "remote_upload": "Disabled: authenticated distinct-host transport and authorization have not been validated. Use an explicitly configured shared mount.",
    "presets": "Named manifests are not implemented; select discovered files directly.",
}

add(
    "assets.search",
    "Fresh filename search across approved project/library roots. No network discovery, semantic service, manifest or fixed robot count. Refresh by calling again. Folders deeper than eight levels are skipped and named in skipped_deep.",
    dict(
        query=field(
            "string",
            "Case-insensitive filename substring <=128 characters.",
            False,
            default="",
        ),
        limit=field("integer", "Maximum results 1..100.", False, default=50),
    ),
)
# Tools that change nothing. The host marks them read-only for clients, and a Kit started read-only
# (TWINSMITH_READ_ONLY=1) runs only these.
READ_ONLY = {
    "assets.roots", "assets.scan", "assets.search", "assets.info", "assets.packs", "assets.pack_doctor", "assets.dependencies",
    "assets.mesh_inspect", "assets.validate",
    "sim.stage.info", "sim.stage.tree", "sim.prim.get", "sim.prim.find", "sim.prim.bounds", "sim.prim.measure",
    "sim.camera.list", "sim.physics.inspect", "sim.physics.raycast", "sim.physics.contacts", "sim.body.state",
    "sim.joints.inspect", "sim.joints.state", "sim.arm.fk", "sim.logs.tail", "sim.logs.mark", "sim.logs.summary",
    "sim.graph.node_types", "sim.graph.inspect", "sim.ui.find",
}
# Tools that can remove or replace what the user made.
DESTRUCTIVE = {
    "sim.prim.delete", "sim.stage.clear_owned", "sim.controller.create", "sim.controller.update", "sim.simulation.run",
    "sim.ui.act", "sim.stage.open", "sim.quick_start", "sim.graph.build", "assets.pack_adopt", "assets.pack_pin",
    "assets.pack_build",
}
# These run in the MCP host and work without Kit. Kit must not serve them.
HOST_ONLY = {
    "assets.roots",
    "assets.scan",
    "assets.search",
    "assets.packs",
    "assets.pack_doctor",
    "assets.pack_adopt",
    "assets.pack_pin",
    "assets.pack_build",
    "assets.mesh_inspect",
    "assets.mesh_decimate",
    "assets.mesh_hulls",
    "assets.mesh_split",
    "assets.validate",
}
add(
    "assets.packs",
    "List the asset packs in the asset root. Every <id>/<version> pack in the asset root (TWINSMITH_ASSET_ROOT, else <workspace>/assets) with its file count, size and whether it verifies against its pack.yaml, and any old-format pack with a hint to convert it with adopt. Host side, no Kit, no USD parser. Writes nothing. Kit reads its own settings: give host and Kit the same TWINSMITH_ASSET_ROOT. With a Kit connected it also lists kit_environment and warns when Kit and this host use a different asset root or project file.",
)
add(
    "assets.pack_doctor",
    "Check a pack without changing it. Every file listed in pack.yaml exists and matches its sha256, nothing unlisted or symlinked is in the folder, then the dependencies of its USD, URDF, MJCF, SDF, mesh and map files must all be files of the same pack (unresolved, unreadable, remote and outside-pack references are problems). Parses USD in a host helper (TWINSMITH_USD_PYTHON) or else the selected Kit; says so if neither ran, and ok is then false. A URDF's own references are read on the host.",
    dict(
        pack=field("string", "Pack as \"<pack id>/<version>\". Default: the project file's pack.", False),
    ),
)
add(
    "assets.dependencies",
    "Compute dependency closures for files under an approved root or the asset root. USD uses UsdUtils.ComputeAllDependencies through a host helper (TWINSMITH_USD_PYTHON) or else the selected Kit; native USD parsing is NOT sandboxed. A .urdf is read on the host with the build's package rules and returns its meshes, textures and package.xml, so a missing package is reported instead of a parser error. Copies nothing.",
    dict(paths=field("array", "Absolute USD or URDF file paths, 1 to 64.")),
)
add(
    "assets.pack_adopt",
    "Copy a folder into a new pack version. Copies a folder's asset files into a NEW pack version <id>/<version> in the asset root and write its pack.yaml (sha256 of every file, plus the source and license you give; redistributable is false unless you say so). The source must be under a user-approved root. A version that exists is never modified: give a new one. Symlinks are never followed. With from_old, converts an old-format pack folder of the asset root into <id>/v1, leaving the old files in place.",
    dict(
        pack=field("string", "The new pack as \"<pack id>/<version>\", for example my-robot/v1."),
        source=field("string", "Where the files came from, one line (a vendor, repository URL and commit, or file hash). Recorded for every file."),
        license=field("string", "The licence or terms, one line (for example \"private lab use only\"). Recorded for every file."),
        source_path=field("string", "Absolute folder under an approved root. Give this or from_old.", False),
        from_old=field("string", "Name of an old-format pack folder in the asset root to convert; pack must be <that name>/<version>.", False),
        redistributable=field("boolean", "The files may be redistributed. Default false.", False, default=False),
        include=field("array", "Extra file extensions to copy.", False),
        files=field("array", "Copy only these files or folders, relative to source_path. A named file is copied whatever its type.", False),
        dry_run=field("boolean", "Report the plan and write nothing.", False, default=False),
    ),
)
add(
    "assets.pack_pin",
    "Rewrite the sha256 values of an existing pack.yaml to match its files, and show what changed. A version that already verifies is not modified. A changed hash accepts whatever bytes are there, so it is refused with before and after hashes unless the file is named in files or force=true. Missing or unlisted files cannot be fixed by a hash: write a new version with adopt or build.",
    dict(
        pack=field("string", "Pack as \"<pack id>/<version>\". Default: the project file's pack.", False),
        files=field("array", "Pin only these relative paths, accepting their changes.", False),
        force=field("boolean", "Accept every changed file.", False, default=False),
        dry_run=field("boolean", "Show what would change and write nothing.", False, default=False),
    ),
)
add(
    "assets.pack_build",
    "Copy asset files and their dependencies into a new pack version. USD, URDF, MuJoCo XML, SDF, mesh or ROS map files and only what they depend on go into a NEW pack version in the asset root, keeping layout, and write its pack.yaml. A file copied out of from_pack keeps that pack's source, license and redistributable; any other file needs source and license. Sources must be under an approved root or the asset root. Unresolved or remote references refuse the build unless allow_unresolved=true. An existing version is never modified. For xacro use assets__import_path.",
    dict(
        pack=field("string", "Destination as \"<pack id>/<version>\"; the version must not exist."),
        sources=field(
            "array",
            "USD, URDF, MJCF (.xml), SDF, mesh or map (.yaml) files, 1 to 64: absolute paths, or paths inside from_pack. A USD scene outside from_pack and layout_root is read for its references and not copied.",
        ),
        from_pack=field(
            "string",
            "Pack as \"<pack id>/<version>\" that source names and logical references come from. It must verify.",
            False,
        ),
        layout_root=field(
            "string",
            "Absolute folder the pack layout is relative to. Default: from_pack; for a USD the source's folder; for a URDF the approved root or pack holding it, which is also where its ROS packages are looked for.",
            False,
        ),
        source=field("string", "Where files that do not come from from_pack came from, one line.", False),
        license=field("string", "Licence or terms of those files, one line.", False),
        redistributable=field("boolean", "Those files may be redistributed. Default false.", False, default=False),
        allow_unresolved=field(
            "boolean", "Build despite missing or remote references, and record them in pack.yaml.", False, default=False
        ),
        dry_run=field("boolean", "Report the closure and write nothing.", False, default=False),
    ),
)
# Mesh preparation and validation: host-side, in a subprocess that needs the optional mesh extra.
_MESH_SOURCE = dict(
    source_path=field("string", "Absolute USD file under an approved root or the asset root; or give pack and asset_id.", False),
    pack=field("string", "\"<pack id>/<version>\" holding asset_id. Default: the project file's pack.", False),
    asset_id=field("string", "Asset id of a file in pack, e.g. bench.", False),
    prims=field("array", "Only meshes at or under these prim paths.", False),
)
_MESH_OUTPUT = dict(
    output=field("string", "New absolute folder under an approved root, outside the asset root."),
)
_COLLISION = dict(
    method=field("string", "box (oriented box per mesh), convex (one hull), coacd (decomposition) or auto (box unless "
                 "over twice the hull's volume, then coacd).", False, default="box"),
    max_hulls=field("integer", "coacd/auto: most convex pieces per mesh, 1 to 256.", False, default=16),
    threshold=field("number", "coacd/auto: concavity, 0.01 to 1; lower is finer and slower.", False, default=0.05),
)
add(
    "assets.mesh_inspect",
    "Report every mesh of a USD file: triangles, bounds, closed or not, whether it is a collider and its approximation, "
    "and totals including the triangles in triangle-mesh colliders (what overflows Newton). Read-only; parses USD in a "
    "subprocess of this server's interpreter (the mesh extra). Native USD parsing is not sandboxed.",
    dict(**_MESH_SOURCE),
)
add(
    "assets.mesh_decimate",
    "Write a copy of a USD file whose render meshes are decimated (quadric) into a new folder, as a job. For render cost; "
    "it does not fix a collider overflow (use split). The source is never changed. Normals and texture coordinates of a "
    "decimated mesh are blocked because they no longer match its faces.",
    dict(**_MESH_SOURCE, **_MESH_OUTPUT,
         target_tris=field("integer", "About this many triangles in all, shared by mesh size. Give this or ratio.", False),
         ratio=field("number", "Keep this fraction of each mesh's triangles, above 0 and at most 1. Give this or target_tris.", False)),
)
add(
    "assets.mesh_hulls",
    "Write a collision layer for a USD file into a new folder, as a job: for each mesh, <parent>/<mesh>_collision with an "
    "oriented box or convex hulls (purpose guide, CollisionAPI, convexHull approximation), in the mesh parent's frame so "
    "rigid bodies and joints are untouched. Shapes go to the meshes under prims; else to the file's mesh colliders; else, "
    "when no mesh is a collider, to every mesh. Sublayer it over the asset, or use split for the composed result.",
    dict(**_MESH_SOURCE, **_MESH_OUTPUT, **_COLLISION),
)
add(
    "assets.mesh_split",
    "Write a USD asset as a render layer and a collision layer plus the file that composes them, into a new folder, as a "
    "job: <stem>_render (the source; the replaced meshes lose their collision schemas, other colliders stay), "
    "<stem>_collision.usdc (explicit shapes, purpose guide) and <stem>.usda (sublayers both, keeps the units, up axis and default prim). Point an asset at <stem>.usda; "
    "make it a pack version with assets__pack action=build, which records the collision counts in its pack.yaml record. "
    "The source is never changed.",
    dict(**_MESH_SOURCE, **_MESH_OUTPUT,
         collision=field("object", "Collision shape options: method, max_hulls, threshold (as for hulls).", False)),
)
add(
    "assets.validate",
    "Check from the file alone whether a USD asset loads on Newton or PhysX: units, triangle-mesh colliders over budget, "
    "rendered collision shapes, zero-force drives, nested articulations, massless moving bodies, drive gains too stiff "
    "for 500 Hz, invalid mimic joints. File only: colliders a scene adds at build time are not seen. Returns ok and each check's level, prim, detail and fix; on physx the "
    "Newton-only rules are info. Read-only, in a subprocess (the mesh extra).",
    dict(
        source_path=_MESH_SOURCE["source_path"], pack=_MESH_SOURCE["pack"], asset_id=_MESH_SOURCE["asset_id"],
        engine=field("string", "newton or physx.", False, default="newton"),
        triangle_budget=field("integer", "Triangle-mesh collider triangles allowed.", False, default=200000),
        kit=field("boolean", "Also run Isaac Sim's Asset Validator: not available yet, refused.", False, default=False),
    ),
)
add(
    "sim.prim.delete",
    "Delete one pack-owned subtree from the session layer only. Requires stopped physics and confirm=true; exact owned sensors release their resources; sensor-containing subtrees and articulation roots are refused. Prims from an opened file are not pack-owned, even ones you made earlier: allow_foreign=true deactivates such a prim in the session layer instead (USD cannot remove another layer's opinions from there). It stays composed as inactive and keeps its path; a sim__stage__save copy omits it; source files never change.",
    dict(
        **TARGET,
        path=field("string", "Pack-owned root path, or any prim with allow_foreign=true."),
        confirm=field("boolean", "Explicit confirmation; false refuses."),
        allow_foreign=field("boolean", "Also act on a prim these tools did not create in this session: deactivate it in the session layer (or remove it if all its opinions are session-layer ones). World/default prim, articulation and other protections still apply. Source files are unchanged.", False, default=False),
    ),
)
add(
    "sim.prim.clone",
    'Clone a pack-created leaf primitive. A simple leaf primitive into a new path, preserving session-layer attributes. Refuses references, children, sensors and articulations.',
    dict(
        **TARGET,
        path=field("string", "Pack-created source leaf."),
        new_path=field("string", "New path, not in use yet."),
    ),
)
add(
    "sim.stage.clear_owned",
    'Remove what these tools created. Only pack-created non-sensor, non-articulated subtrees; other scene content is kept. Explicit stopped target and confirm required. Returns removed/refused lists.',
    dict(**TARGET, confirm=field("boolean", "Explicit confirmation.")),
)
add(
    "sim.light.create",
    'Create a Sphere, Distant or Dome light. A SphereLight, DistantLight or DomeLight without asset-valued textures. Pose in stage units, orientation wxyz.',
    dict(
        **TARGET,
        path=field("string", "New prim path."),
        kind=field(
            "string",
            "SphereLight, DistantLight or DomeLight.",
            False,
            default="DistantLight",
        ),
        intensity=field(
            "number", "Nonnegative USD light intensity.", False, default=1000
        ),
        color=field("array", "RGB linear color 0..1.", False, default=[1, 1, 1]),
        **POSE,
    ),
)
add(
    "sim.light.modify",
    "Set a pack-owned light's intensity and color. Both required, on a stopped timeline.",
    dict(
        **TARGET,
        path=field("string", "Pack-owned light path."),
        intensity=field("number", "USD light intensity."),
        color=field("array", "RGB linear color 0..1."),
    ),
)
add(
    "sim.material.create",
    'Create a preview-surface or physics material. A UsdPreviewSurface or physics material with finite numeric parameters. No asset-valued attributes or texture URLs.',
    dict(
        **TARGET,
        path=field("string", "New Material path; parent must exist."),
        color=field("array", "RGB 0..1.", False, default=[0.5, 0.5, 0.5]),
        roughness=field("number", "0..1.", False, default=0.5),
        metallic=field("number", "0..1.", False, default=0),
        kind=field("string", "pbr or physics.", False, default="pbr"),
        static_friction=field("number", "Physics material static friction 0..10.", False, default=0.5),
        dynamic_friction=field("number", "Physics material dynamic friction 0..10.", False, default=0.5),
        restitution=field("number", "Physics material restitution 0..1.", False, default=0),
    ),
)
add(
    "sim.material.apply",
    'Bind a pack-created material to geometry. To an existing geometry prim in the session layer, preserving source files. With paths (up to 128 prims) every prim is checked first, so a bad one binds nothing.',
    dict(
        **TARGET,
        path=field("string", "Geometry prim. Give this or paths.", False),
        material_path=field("string", "Pack-created Material path."),
        paths=field("array", "Up to 128 geometry prims to bind the same material to in one call, instead of path.", False),
    ),
)
add(
    "sim.physics.configure",
    'Set PhysicsScene gravity and time step. For an explicitly selected scene, while stopped. No implicit scene creation or GPU/backend switch. On Newton it also sizes the solver\'s contact and constraint buffers, reported as newton_buffers, and without dt steps at 500 Hz.',
    dict(
        **TARGET,
        path=field("string", "PhysicsScene prim path."),
        gravity=field(
            "array",
            "Acceleration vector in stage units/s^2.",
            False,
            default=[0, 0, -9.81],
        ),
        dt=field(
            "number", "Physics timestep seconds, 1/1000..1/10. Default 1/60 on PhysX and 1/500 on Newton, "
                      "the rate this project's Newton scenes use.", False
        ),
        nconmax=field("integer", "Newton only: MuJoCo-Warp contact buffer per world. Default: sized from the rigid-body count. Refused on PhysX.", False),
        njmax=field("integer", "Newton only: constraint rows per world; same rules as nconmax.", False),
    ),
)
add(
    "sim.joints.inspect",
    'Read authored joint limits and drives. Bounded, for an articulation, reporting USD degrees versus stage units. No runtime-state claim. The path may be the articulation root (a joint such as <robot>/root_joint, or the link carrying the articulation API): the joints are then the ones reached through its rigid bodies.',
    dict(
        **TARGET,
        path=field("string", "Subtree, articulation root (joint or link), or the robot prim."),
        limit=field("integer", "Visited prim cap 1..200.", False, default=200),
    ),
)
add(
    "sim.joints.drive",
    "Set one joint's USD drive target and gains. One revolute/prismatic joint, in session layer; stopped timeline only. Angular target radians; gains use USD degrees-based convention.",
    dict(
        **TARGET,
        path=field("string", "Revolute or prismatic joint prim, e.g. /World/UR5e/joints/elbow_joint; sim__joints__inspect lists them."),
        target=field("number", "Radians for angular joint, stage units for prismatic."),
        stiffness=field("number", "Nonnegative USD drive stiffness. Too low and a gravity-loaded joint settles short of its target."),
        damping=field("number", "Nonnegative USD drive damping."),
        max_force=field("number", "Positive force/torque cap.", False, default=1000),
    ),
)
add(
    "sim.joints.state",
    'Read joint positions and velocities while running. While playing or paused, initialized articulation DOF positions/velocities on selected PhysX or qualified Newton tensor API. Stopped gives PHYSICS_NOT_READY: sim__timeline action=initialize briefly plays then pauses. Angular radians, linear stage units; no authored-target fallback. PHYSICS_STATE_INVALID identifies invalid runtime joint data.',
    dict(**TARGET, path=field("string", "Exact articulation root path, e.g. /World/UR5e/root_joint: robot.articulation_root from sim__quick_start, or sim__physics__inspect.")),
)
add(
    "sim.simulation.step",
    'Advance exact physics steps while paused. 1..120 steps on PhysX or qualified Newton, PAUSED and initialized: from a stopped timeline, call sim__timeline action=initialize (physics may advance). No app-update pumping or engine fallback. Returns measured time delta, poses for observe_prims, DOFs for observe_joints. A failure after stepping reports the clock and phase. No retry or rollback. On Newton it first refuses STEP_DT_SOURCES_DISAGREE if the scene\'s timestep differs from Newton\'s, and adds a newton section.',
    dict(
        **TARGET,
        steps=field("integer", "Number of physics steps 1..120."),
        observe_prims=field("array", "Up to 16 rigid body paths.", False, default=[]),
        observe_joints=field("array", "Up to 8 articulation paths.", False, default=[]),
    ),
)
add(
    "sim.simulation.run",
    "Run your own controller in the physics loop as a job. Each step gets the body's state {step, time_s, dt, position, orientation_wxyz, linear_velocity, angular_velocity, mass, params} and returns {force, torque} for that step; with articulation also joints, and joint_efforts or joint_targets back. Steps exactly from a pause (mode=step), or follows a playing timeline (mode=play). Returns job_id: job_status(job_id, wait_s=30), job_cancel. Writes a CSV log; with camera a video. module:function is OPT-IN CODE EXECUTION in Kit; tcp:// runs none.",
    dict(
        **TARGET,
        body=field("string", "Rigid body the controller acts on, e.g. a drone's base link."),
        controller=field("string", "tcp://127.0.0.1:<port>: your own process, one JSON line per step each way (twinsmith.controller_server.serve is a ready-made server; start it with run_script). Or /abs/file.py:function or package.module:function, loaded into Kit: def f(state) returning {'force': [x, y, z]}; state has position, linear_velocity, mass, dt, time_s and params. Kit must have been launched with TWINSMITH_ENABLE_SCENE_SCRIPTS=1; if not, the call says so and nothing runs."),
        duration_s=field("number", "Simulated seconds to run, up to 3600. The controller can end sooner with done=true."),
        params=field("object", "Passed to the controller as state['params'] each step, e.g. gains or a target.", False),
        frame=field("string", "Frame of the returned force and torque, applied at the centre of mass: world (default) or body.", False, default="world", enum=["world", "body"]),
        log=field("array", "Fields to log: position, orientation, linear_velocity, angular_velocity, force, torque. Default: all. Numbers the controller returns under log become columns too.", False),
        log_every=field("integer", "Log one row every this many physics steps, 1..10000.", False, default=1),
        filename=field("string", "New CSV name in the output folder, e.g. hover.csv. Default run-<time>.csv. Never overwrites.", False),
        mode=field("string", "step (default): exact steps from a paused timeline, repeatable and as fast as the machine allows. play: the timeline plays and the controller runs in the engine's pre-step callback, so ROS publishers and action graphs tick too; it keeps playing afterwards.", False, default="step", enum=["step", "play"]),
        steps_per_frame=field("integer", "Physics steps between app frames, 1..120. Default: about real time at 60 frames per second. With camera, fps sets it.", False),
        articulation=field("string", "Articulation root of a jointed robot (often <robot>/root_joint). The state then has joints {names, positions, velocities}, and the controller may return joint_efforts, joint_targets or joint_velocity_targets as {joint name: value} (N m or N, rad or stage units).", False),
        observe=field("array", "Up to 8 more rigid bodies whose state the controller gets under observed[path]. It may push them too, with wrenches: {path: {force, torque}}.", False),
        camera=field("string", "Record this camera while it runs (from sim__camera__create): one frame per app frame, and job_status then gives video_path. Works headless and on a locked screen.", False),
        adopt=field("boolean", "Record a camera that came with an opened scene (see sim__camera__capture adopt).", False),
        fps=field("number", "With camera: video frames per simulated second, 1..60.", False, default=30),
        label=field("string", "Short name for the job, shown by job_status and drawn on the video (<=80 characters).", False),
    ),
)
add(
    "sim.trajectory.replay",
    'Replay a saved trajectory into a labelled video. As a job: moves prims through the poses in a file and saves one camera frame per step. No physics runs and the prims go back afterwards; needs a stopped timeline. Returns job_id; job_status(job_id, wait_s=30) then gives video_path, an MP4 with the label drawn on it (works headless and on a locked screen). Without camera it shows the poses at wall-clock pace, for a screen recording.',
    dict(
        **TARGET,
        trajectory=field("string", "Absolute path under your home or the output folder. A CSV as sim__simulation__run writes it (time_s, px,py,pz, qw,qx,qy,qz; or t and x,y,z), or JSON / JSONL samples {time_s, position, orientation_wxyz} or {time_s, bodies: {prim: {...}}}."),
        prim=field("string", "Prim to move, for a file with one unnamed body: the robot's top Xform or a free body, not a jointed link.", False),
        camera=field("string", "Camera to record, from sim__camera__create or sim__camera__list. Omit to only show the poses.", False),
        adopt=field("boolean", "Record a camera that came with an opened scene (see sim__camera__capture adopt).", False),
        label=field("string", "One line (<=120 characters) drawn on every frame of the video, e.g. 'PhysX hover, kp=4'.", False),
        fps=field("number", "Video frames per second, 1..60.", False, default=30),
        speed=field("number", "Playback speed: 2 is twice as fast, 0.5 slow motion.", False, default=1),
        start_s=field("number", "Start at this trajectory time. Default: its first sample.", False),
        end_s=field("number", "End at this trajectory time. Default: its last sample.", False),
        filename=field("string", "New video name, e.g. hover.mp4. Default replay-<time>.mp4. Frames are saved beside it as <name>-00001.png.", False),
    ),
)
add(
    "sim.camera.create",
    "Create a persistent RTX camera on a stopped timeline. To watch something: path=/World/Cam, position=[3,-3,2], target=[0,0,0.5] aims it. On a robot link: path=/robot/base_footprint/Camera_1, position=[0.1,0,0.2] (local), rotate_xyz=[90,0,-90] (looks along the parent's +X). USD axes: -Z forward, +Y up. Clipping is in metres (default 0.01..1000), converted to stage units. Returns optics, FOV and the local/world pose. Needs rendered frames; width/height capped at 1024. A USD camera has no tick rate; set ROS rates on the camera helper.",
    dict(
        **TARGET,
        path=field("string", "New camera path; its parent prim (e.g. a robot link) must exist."),
        width=field("integer", "Image width 64..1024.", False, default=640),
        height=field("integer", "Image height 64..1024.", False, default=480),
        **POSE,
        target=field("array", "World point [x, y, z] to aim the camera at, instead of orientation or rotate_xyz.", False),
        rotate_xyz=field("array", "Orientation as USD rotateXYZ Euler angles in degrees (X, then Y, then Z), instead of the orientation quaternion. With frame=local it is authored as translate + rotateXYZ ops.", False),
        focal_length=field("number", "Focal length in mm (0.1..1000); default USD 50.", False),
        horizontal_aperture=field("number", "Horizontal aperture in mm (0.1..1000); default USD 20.955. The vertical aperture follows width/height.", False),
        clipping_range=field("array", "[near, far] clipping distances in metres.", False, default=[0.01, 1000.0]),
    ),
)
add(
    "sim.camera.capture",
    'Capture an RTX camera image to a PNG. Camera RGB into an owned CPU buffer and a new PNG in configured output root. Default waits up to five seconds for new render completions AND a matching renderer CPU transfer; no stale annotator fallback. Returns dimensions/hash. No implicit Play or atomic physics-frame synchronization. For a camera from an opened file, pass adopt=true. The result has a physics_render field.',
    dict(
        **TARGET,
        path=field("string", "Camera from sim__camera__create or sim__quick_start, or any other Camera with adopt=true."),
        filename=field("string", "New simple basename ending .png."),
        adopt=field("boolean", "If this exact prim is a Camera with no render resources in this session (e.g. from an opened file), create them here. They count toward the eight sensors and are released on stage change; the prim itself is not edited and stays non-owned.", False, default=False),
    ),
)
add(
    "sim.lidar.create",
    "Create an RTX LiDAR (OmniLidar) while stopped: NVIDIA's named config (e.g. config=Example_Rotary_2D at /robot/base_scan/Lidar2D) or, without config, a local generic LiDAR. An unknown config lists the valid ones. Returns prim type, sensor path, rates and ros2_scan_graph: pass that to sim__graph__build to publish /scan. At most eight sensors; delete an exact owned sensor path to release resources.",
    dict(
        **TARGET,
        path=field("string", "Path for the new LiDAR, not in use yet; its parent prim (e.g. a robot link) must exist."),
        **POSE,
        rotate_xyz=field("array", "Orientation as USD rotateXYZ Euler angles in degrees, instead of the orientation quaternion.", False),
        config=field("string", "NVIDIA lidar config name, e.g. Example_Rotary_2D, Example_Rotary, OS1, RPLIDAR_S2E. Empty: local generic LiDAR.", False, default=""),
        variant=field("string", "Config variant, e.g. OS1_REV6_32ch10hz1024res (an invalid one lists the valid ones).", False, default=""),
        tick_rate=field("number", "Sensor tick rate in Hz (0..240]; default: the config's scan rate (10 Hz for Example_Rotary_2D). A rate other than the scan rate yields partial scans.", False),
    ),
)
add(
    "sim.lidar.read",
    'Read LiDAR return points. CPU Cartesian points in sensor coordinates with sensor-to-world transform, filter invalid ranges, cap at 256 samples. Requires initialized sensor and rendered simulation frames. For a LiDAR from an opened file, pass adopt=true. The result has a physics_render field.',
    dict(
        **TARGET,
        path=field("string", "Pack-created LiDAR path, or an existing OmniLidar with adopt=true."),
        limit=field("integer", "Maximum returned points 1..256.", False, default=64),
        adopt=field("boolean", "If this exact prim is an OmniLidar with no render resources in this session (e.g. from an opened file), create them here. They count toward the eight sensors and are released on stage change; settings a read needs are overridden in the session layer only and reported as session_overrides. The prim stays non-owned.", False, default=False),
    ),
)
add(
    "sim.ros2_tf",
    "Build one ROS 2 graph for a robot, the same way every time: /clock (sim time), /joint_states, the static world->base_link (or world->odom + odometry for a mobile base) and the robot's link TF from Isaac. Rebuilds its own graph on each call, so no frame is ever published twice by this tool; reuses an existing /clock and reports other ROS 2 publishers in the scene. Timeline must be stopped; publishes while playing. Then use scripts/ros2_rviz_config.py and scripts/ros2_tf_doctor.py (docs/ROS2-RVIZ.md).",
    dict(
        **TARGET,
        robot=field("string", "Robot prim at or above its articulation root."),
        base_link=field("string", "Robot root link prim; default: a link named base_link, base_footprint or base.", False, default=""),
        base_frame=field("string", "TF name for the base link; default: the base_link prim name.", False, default=""),
        base=field("string", "fixed (arm on a table: static world->base) or mobile (odometry: world->odom->base).", False, default="fixed"),
        world_frame=field("string", "Root frame for RViz's Fixed Frame, e.g. world or map. Empty: publish no world link (a localization node provides map->odom).", False, default="world"),
        link_frames=field("string", "isaac: Isaac publishes every link frame. robot_state_publisher: Isaac publishes only /joint_states and the base link; run robot_state_publisher with the URDF. Never both.", False, default="isaac"),
        extra_frames=field("array", "Up to 16 more prims (cameras, LiDARs, tool0) to publish; frame name = prim name. One mounted on a robot link, or any with a fixed base, is published once as a static transform from its pose now.", False, default=[]),
        attach_to=field("string", "For a robot that is its own articulation mounted on another (RG2 on a UR5e wrist): the parent link's prim. Publishes parent->base_link dynamically on every tick instead of a static world link, so it follows the arm. Call this tool once per articulation, with a namespace for this one.", False, default=""),
        namespace=field("string", "ROS node namespace prepended to the topics.", False, default=""),
        joint_states_topic=field("string", "Joint state topic.", False, default="joint_states"),
        clock=field("string", "auto: reuse an existing /clock, else publish one shared clock for all robots (two clocks make RViz reset constantly); publish: always keep the tool's clock; skip: never create one.", False, default="auto", enum=["auto", "publish", "skip"]),
    ),
)
add(
    "sim.graph.node_types",
    "Find OmniGraph node types and their ports. By id or UI-name substring in the running Isaac Sim (e.g. 'Make Array' -> omni.graph.nodes.ConstructArray, 'PublishClock'), with exact input/output port names, types, defaults and descriptions from the installed .ogn files. Use before building Action Graphs; port names differ between Isaac versions. Only contains is required; a catalog read needs no session_id or stage_id.",
    dict(
        **UNUSED_TARGET,
        contains=field("string", "Substring of the node type id or its UI name, case-insensitive (<=64 chars)."),
        limit=field("integer", "Maximum node types returned, 1..40.", False, default=20),
        ports=field("boolean", "Include ports (default: only when 5 or fewer types match; at most 8 detailed).", False),
    ),
)
add(
    "sim.graph.build",
    'Build a whole OmniGraph in one call. E.g. an Action Graph for ROS 2: nodes, input values, connections and prim targets. Find node types and ports first with sim__graph__node_types. Checks every port, lists the real ones on a typo, and rolls the graph back on any error. Timeline stopped. Returns it as sim__graph__inspect does.',
    dict(
        **TARGET,
        path=field("string", "Graph prim path to create, e.g. /World/ROS_Clock or /turtlebot3_burger/ROS_Drive."),
        nodes=field("array", "[name, node_type] pairs or {name, type} objects; entries or whole list may be JSON strings, e.g. ['[\"tick\", \"omni.graph.action.OnPlaybackTick\"]']. A flat alternating list is also accepted."),
        values=field("object", "Input values: {\"node.inputs:port\": value}, or a JSON string containing that object.", False),
        connections=field("array", "[from, to] pairs or {from, to} objects; entries or whole list may be JSON strings. A flat alternating list is also accepted.", False),
        targets=field("object", "Prim targets: {\"node.inputs:targetPrim\": [\"/World/robot\"]}, or a JSON string containing that object or a target path list.", False),
        evaluator=field("string", "Graph evaluator: execution (Action Graph, default), push or dirty_push.", False, default="execution", enum=["execution", "push", "dirty_push"]),
        replace=field("boolean", "Delete an existing graph at path first (it is not restored if the new build fails).", False, default=False),
    ),
)
add(
    "sim.graph.inspect",
    "Read an OmniGraph's nodes and connections. Every node's type, input values or upstream connections, and output connections. Use to verify a hand-built Action Graph (missing wires, unset targets, ports created at runtime). ROS helpers report ros_render_status and writer_computes, which show a sensor that is not rendering; publisher count is not read here, so check with ros2 topic info. Read-only.",
    dict(
        **TARGET,
        path=field("string", "Graph prim path, e.g. /World/ROS_Clock."),
        limit=field("integer", "Maximum nodes returned, 1..128.", False, default=64),
    ),
)
add(
    "sim.graph.show",
    "Open a graph in the Action Graph editor. Of a windowed Isaac Sim and zoom to fit, e.g. for a demo video; layout=true arranges nodes and writes positions into the stage, so stop first or use layout=false while playing; TIMELINE_ACTIVE includes a retry_with call. TIMELINE_ACTIVE includes a retry_with call. close=true closes the editor. Uses the editor's internal API and refuses cleanly if it changed.",
    dict(
        **TARGET,
        path=field("string", "Graph prim path (not needed with close=true).", False, default=""),
        layout=field("boolean", "Expand and auto-arrange nodes that have no stored position yet (writes node positions).", False, default=False),
        width=field("integer", "Undock the editor window and size it (pixels, clamped to the Isaac Sim window); 0 keeps the docked layout.", False, default=0),
        height=field("integer", "Window height in pixels when width is set.", False, default=0),
        close=field("boolean", "Close the Action Graph editor window instead of showing a graph.", False, default=False),
    ),
)
add(
    "sim.ui.find",
    "Find Isaac Sim UI widgets. Lists Isaac Sim's own UI widgets (buttons, fields, checkboxes, combo boxes, tree views, and labels when a query is given) with their text, tooltip, current value, a ref, and their centre in pixels of the Isaac Sim window (the same pixels as screenshot(window_id=<Isaac Sim window>)), plus the visible Kit windows. Works while the desktop is locked or used remotely. Use the ref with sim__ui__act instead of guessing coordinates.",
    dict(
        query=field("string", "Case-insensitive text to match in a widget's name, text, tooltip, identifier or type, e.g. 'play' or 'Add'. Empty lists interactive widgets only.", False, default=""),
        window=field("string", "Only windows whose title contains this, e.g. 'Stage'.", False, default=""),
        limit=field("integer", "Maximum widgets returned, 1..100 (the reply is also capped near 48 KB).", False, default=40),
        zoom_ref=field("string", "Zoom on this widget ref instead of listing: returns an enlarged crop of the Isaac Sim window around it, with its own coord_space token.", False),
    ),
)
add(
    "sim.ui.act",
    "Click or type in Isaac Sim's UI; can delete data. Drives Isaac Sim's own UI through Kit's input system, not the desktop: click, double_click, right_click, move, drag, scroll, type, keypress, menu, tab, close, or wait. Target a widget by ref from sim__ui__find or by x, y window pixels. Works while the desktop is locked or used remotely (the X11 computer tool does not). screenshot_after=true returns the window after the action; or check with sim__ui__find. UI actions can change or delete scene data and files like a person clicking would.",
    dict(
        action=field("string", "What to do. tab brings a docked window to the front, like clicking its tab (sim__ui__find lists hidden_tabs); close hides a Kit window or docked panel such as 'Console' and reports whether it is now hidden (reopen with tab). Omit it with actions.", False, enum=["click", "double_click", "right_click", "move", "drag", "scroll", "type", "keypress", "menu", "tab", "close", "wait"]),
        ref=field("string", "Widget ref from sim__ui__find (its centre is used). For type/keypress it is clicked first to focus it.", False),
        x=field("integer", "X in Isaac Sim window pixels (when no ref).", False),
        y=field("integer", "Y in Isaac Sim window pixels (when no ref).", False),
        end_x=field("integer", "Drag end X in window pixels.", False),
        end_y=field("integer", "Drag end Y in window pixels.", False),
        text=field("string", "Text for action=type (1..500 characters, no newlines; send enter with keypress).", False),
        keys=field("array", "For action=keypress: one key plus optional modifiers, e.g. [\"ctrl\", \"s\"], [\"enter\"], [\"shift\", \"tab\"].", False),
        window=field("string", "Window or tab title for tab and close, e.g. 'Console'.", False),
        menu=field("string", "For action=menu: a menu path, e.g. 'Create/Mesh/Cube' (a menu an extension adds exists only once it has loaded), or a top-level menu such as 'Layouts' (opened to list its items, then closed again).", False),
        scroll_x=field("integer", "Horizontal wheel steps for action=scroll (-20..20).", False, default=0),
        scroll_y=field("integer", "Vertical wheel steps for action=scroll (-20..20; negative scrolls down).", False, default=0),
        frames=field("integer", "Frames to wait after the action so the UI updates (0..600).", False, default=3),
        actions=field("array", "Batch of up to 10 steps, each an object with action (not sub_action) and that action's arguments. E.g. [{\"action\": \"click\", \"ref\": \"...\"}, {\"action\": \"type\", \"text\": \"2\"}]. Steps run in order and stop at the first failure; they replace the top-level action fields, and frames is then each step's default wait.", False),
        coord_space=field("string", "window (Isaac Sim window pixels) or the coord_space token a screenshot printed (x/y are then pixels of that image).", False, default="window"),
        screenshot_after=field("boolean", "Also return a screenshot of the Isaac Sim window after the action(s).", False, default=False),
        find_after=field("string", "Also return up to 10 widgets matching this text after the action(s), compactly (same matching as sim__ui__find query).", False),
    ),
)
add(
    "sim.occupancy_map",
    "Generate a 2D occupancy map from the stage's colliders (Isaac's occupancy-map generator) and write a Nav2 map_server PGM + YAML (trinary, origin = lower-left corner, +x right, +y up) to the configured output root. Start with the timeline stopped: the tool plays 8 frames through the guarded timeline path (the generator needs running physics; OnPlaybackTick graphs such as /clock run meanwhile), then stops, so robots return to their start poses. Use when a lab or project does not provide map files.",
    dict(
        **TARGET,
        min=field("array", "World [x, y] lower corner of the map in metres."),
        max=field("array", "World [x, y] upper corner of the map in metres."),
        z=field("array", "World [z_min, z_max] in metres. A collider anywhere in this range marks its cells occupied, so keep z_min above the floor and z_max below anything the robot passes under", False, default=[0.05, 0.3]),
        clear=field("array", "Up to 16 [x, y, radius] circles (metres) to mark free, e.g. the robot's own footprint at its start pose.", False, default=[]),
        start=field("array", "World [x, y] in free space the generator flood-fills from; cells it cannot reach are unknown. Default: the centre of the bounds (move it if a table or pillar is there).", False),
        cell_size=field("number", "Cell size in metres (map resolution).", False, default=0.05),
        basename=field("string", "Output file stem (letters, digits, _ -); writes <basename>.pgm and .yaml.", False, default="map"),
    ),
)
add(
    "sim.logs.tail",
    'Read a bounded tail of the Kit log. Of the selected Kit log file, never arbitrary files. Default: last 1 MiB, warnings/errors and [sim-tools] lines in `lines`. With min_level, channel, contains or cursor: parsed `entries` (timestamp, level, channel, message capped at 2 KB), `next_cursor` and `truncated`; a cursor comes from sim__logs__mark or a previous next_cursor. Reads at most 1 MiB per call and never installs a log callback. Redacts obvious secrets. Also returns recent scene_events. Needs no session_id or stage_id.',
    dict(
        **UNUSED_TARGET,
        limit=field("integer", "Maximum entries or lines 1..100.", False, default=50),
        min_level=field(
            "string",
            "Lowest level to return, e.g. warning.",
            False,
            default="warning",
            enum=["debug", "info", "warning", "error", "fatal"],
        ),
        channel=field("string", "Case-insensitive substring of the log channel, e.g. omni.physx (<=64 characters).", False, default=""),
        contains=field("string", "Case-insensitive substring of the message (<=128 characters).", False, default=""),
        cursor=field(
            "object",
            "Cursor from sim__logs__mark or a previous next_cursor; reads only lines after it and detects rotation/truncation.",
            False,
        ),
    ),
)
add(
    "sim.logs.mark",
    "Mark the current end of Kit's log. A cursor: byte offset plus file identity (device, inode, size, mtime). Pass it to sim__logs__tail or sim__logs__summary to read only what happens next; rotation or truncation is reported instead of unrelated lines. Reads the log path only; installs no log callback. Needs no session_id or stage_id.",
    dict(**UNUSED_TARGET),
)
add(
    "sim.logs.summary",
    'Count log lines by level and channel. Since a sim__logs__mark cursor (or the retained 1 MiB window when omitted), with the first and last error. Reads at most 1 MiB per call; `truncated` and `next_cursor` say whether more remains. Same file parser and secret redaction as sim__logs__tail; installs no log callback. Needs no session_id or stage_id.',
    dict(
        **UNUSED_TARGET,
        cursor=field(
            "object",
            "Cursor from sim__logs__mark or a previous next_cursor; omit for the retained 1 MiB window.",
            False,
        ),
    ),
)
add(
    "sim.controller.create",
    'OPT-IN CODE EXECUTION: create a script controller. Creates an OnPlaybackTick -> ScriptNode controller using inline Python. Requires TWINSMITH_ENABLE_SCENE_SCRIPTS=1 in Kit launch; NOT a sandbox.',
    dict(
        **TARGET,
        path=field("string", "Path for the new graph, not in use yet."),
        code=field(
            "string", "Trusted setup(db)/compute(db) script, <=16384 characters."
        ),
    ),
)
add(
    "sim.controller.update",
    "OPT-IN CODE EXECUTION: replace a controller's code. Replaces inline code of this pack's controller and requests recompilation while stopped. Same explicit script enable required.",
    dict(
        **TARGET,
        path=field("string", "Pack controller path."),
        code=field("string", "Trusted replacement script <=16384 characters."),
    ),
)

add(
    "sim.arm.ik",
    'Inverse kinematics for a link target. For any robot with revolute or prismatic joints, read from its own USD joints (no per-robot file): the joint values that put a link at a target position and optional orientation, nearest the current pose. Solves only unless move=true; no collision check or path. Needs a playing or paused timeline. Refuses if its model disagrees with physics. Mimic joints are treated as independent. Returns joint_targets for sim__joints__move and the errors; reached=false means out of reach or limits.',
    dict(
        **TARGET,
        path=field("string", "Articulation root, as for sim__joints__state."),
        link=field("string", "The link to place: its name (such as panda_hand) or absolute path."),
        position=field("array", "Target [x, y, z] in stage units, in the world frame unless frame=base."),
        orientation=field("array", "Optional target orientation [w, x, y, z]. Omit to place the link by position only.", False),
        base_link=field("string", "Chain start: a link name or path. Default: the body the articulation hangs from (the root body, or the one fixed to the world).", False),
        offset=field("array", "Optional [x, y, z] in the link's frame, stage units: place this point (a fingertip) instead of the link's origin.", False),
        frame=field("string", "world (default) or base: the frame position and orientation are given in.", False, default="world"),
        move=field("boolean", "Also command the solved joint values (needs a playing timeline), so one call moves the arm; nothing is commanded when reached is false. Default false: solve only.", False),
        speed=field("number", "With move=true: go to position in a straight line at this speed (stage units per second), holding orientation, in this one call. Stops short at a point it cannot reach. The result's path says where the point really ended.", False),
        position_tolerance=field("number", "Metres of position error that counts as reached.", False, default=0.001),
        orientation_tolerance=field("number", "Radians of orientation error that counts as reached.", False, default=0.01),
    ),
)
add(
    "sim.arm.fk",
    "Forward kinematics: where a link would be. Where a link (the hand) would be for given joint values, from the robot's own USD joints, in the world and base frames. Joints you leave out use their current values when the simulation runs, else zero. Commands nothing; pair it with sim__arm__ik. Works stopped or running; when running it first checks its model against physics.",
    dict(
        **TARGET,
        path=field("string", "Articulation root, as for sim__joints__state."),
        link=field("string", "The link: its name (such as panda_hand) or absolute path."),
        joints=field("object", "Optional {joint name: value}: radians for revolute, stage units for prismatic, e.g. sim__arm__ik's joint_targets.", False),
        offset=field("array", "Optional [x, y, z] in the link's frame, stage units: place this point (a fingertip) instead of the link's origin.", False),
        base_link=field("string", "Chain start: a link name or path. Default: the articulation root's body.", False),
    ),
)
add(
    "sim.joints.command",
    'Command joint position or velocity targets. On an initialized articulation, by name (targets) or in DOF order (positions), and/or velocity targets, on the selected PhysX or qualified Newton backend. Radians for revolute, stage units for prismatic. Out-of-limit values are refused unless clamp=true, which reports what it clamped; unknown names list the valid ones. Returns applied targets and previous values, not achieved poses: compare sim__joints__state. Runtime only and lost on Stop; sim__joints__drive authors USD targets.',
    dict(
        **TARGET,
        path=field("string", "Exact initialized articulation root path."),
        positions=field(
            "array",
            "One finite position per DOF, in sim.joints.state order. Give this or targets, not both.",
            False,
        ),
        targets=field(
            "object",
            "{dof name: position}; names come from sim__joints__state. An unknown name is refused with the valid ones.",
            False,
        ),
        velocities=field(
            "object",
            "{dof name: velocity target}, rad/s for revolute and stage units/s for prismatic. A velocity drive needs damping (a wheel usually stiffness 0), so check the response.",
            False,
        ),
        clamp=field(
            "boolean",
            "Use the runtime limit for an out-of-limit value and report it, instead of refusing.",
            False,
            default=False,
        ),
    ),
)
add(
    "sim.joints.gains",
    "Read or set runtime drive gains. An initialized articulation's stiffness, damping and max_effort (the drive's torque/force cap) for named dofs or every DOF. These are PhysX/Newton runtime values, not USD: they are lost on Stop and never saved; sim__joints__drive authors USD gains. Omit every value to only read. Returns each DOF's values before and after. Needs a playing or paused timeline and a valid tensor view.",
    dict(
        **TARGET,
        path=field("string", "Exact initialized articulation root path."),
        dofs=field(
            "array", "DOF names; empty applies to every DOF. An unknown name is refused with the valid ones.", False, default=[]
        ),
        stiffness=field(
            "number", "Nonnegative position-drive stiffness (torque per radian, or force per stage unit).", False
        ),
        damping=field(
            "number", "Nonnegative position-drive damping (torque per rad/s, or force per stage unit/s).", False
        ),
        max_effort=field(
            "number", "Positive drive cap in torque (revolute) or force (prismatic).", False
        ),
    ),
)
add(
    "sim.joints.move",
    "Step a paused simulation until joints settle. Steps a PAUSED, initialized simulation until named joint targets are within tolerance or max_steps is used, then reports settled and each joint's error. From stopped, sim__timeline action=initialize briefly plays then pauses. A joint held by contact or a weak drive reports settled=false: clear the path, raise gains with sim__joints__gains, or retry with more steps. Runtime only; never pumps the app loop.",
    dict(
        **TARGET,
        path=field("string", "Exact initialized articulation root path."),
        targets=field(
            "object", "{dof name: position}, radians or stage units. An unknown name is refused with the valid ones."
        ),
        max_steps=field("integer", "Physics steps to try, 1..600.", False, default=600),
        tolerance=field(
            "number", "Per-joint position error that counts as settled, >0 (radians or stage units).", False, default=0.01
        ),
        clamp=field(
            "boolean", "Use the runtime limit for an out-of-limit target and report it, instead of refusing.",
            False, default=False,
        ),
    ),
)
add(
    "sim.joints.teleport",
    "Set joint positions directly (a state reset). Not motion: physics response, contacts and drive limits are bypassed, and drive targets are left unchanged so the next steps can pull the joints back. Positions are {dof name: value}, radians for revolute and stage units for prismatic. Needs an initialized (playing or paused) articulation and returns each joint's value before and after. Out-of-limit values are refused.",
    dict(
        **TARGET,
        path=field("string", "Exact initialized articulation root path."),
        positions=field(
            "object", "{dof name: position}. Unknown names are refused with the valid ones."
        ),
    ),
)
add(
    "sim.prim.set_attribute",
    'Set one typed attribute on any prim. In the session layer, reporting the composed value before and after; undo it with sim__prim__revert. Refuses engine- and schema-owned attributes (points, normals, faceVertexIndices, faceVertexCounts, extent, xformOpOrder, xformOp:*, physics:*, physx*:*), instance proxies and Kit runtime prims. type creates a missing attribute; without it the attribute must exist. Values are JSON; quaternions are [w,x,y,z].',
    dict(
        **TARGET,
        path=field("string", "Absolute prim path."),
        attribute=field("string", "Attribute name, e.g. visibility, purposes, user:owner or a namespaced name."),
        value=field(
            "any",
            "New value as JSON: true/false, a number, a string, or a flat list for a vector/quaternion. Converted to the attribute's USD type.",
        ),
        type=field(
            "string",
            "USD type for a new attribute: bool, int, uint, int64, uint64, float, double, string, token, float2/3/4, double2/3/4, int2/3, color3f/4f, normal3f, point3f, vector3f, quatf, quatd.",
            False,
        ),
    ),
)
add(
    "sim.physics.apply",
    'Add or remove colliders and rigid bodies, set mass. Add or remove a collider, rigid body and kinematic state, or set mass / density, on any prim; session layer only, and undo with sim__prim__revert. collider can set an approximation (convexHull, convexDecomposition, boundingSphere, boundingCube, meshSimplification, none). Refuses a rigid body under an existing rigid-body ancestor, a joint or articulation root as a body, both mass and density, and any change while the timeline is active.',
    dict(
        **TARGET,
        path=field("string", "Absolute prim path."),
        collider=field("boolean", "Add (true) or remove (false) UsdPhysics CollisionAPI.", False),
        approximation=field(
            "string",
            "Collider approximation, with collider=true; empty keeps the current one.",
            False,
            default="",
            enum=[
                "convexHull",
                "convexDecomposition",
                "boundingSphere",
                "boundingCube",
                "meshSimplification",
                "none",
            ],
        ),
        rigid_body=field("boolean", "Add (true) or remove (false) UsdPhysics RigidBodyAPI.", False),
        kinematic=field(
            "boolean", "Kinematic flag (physics:kinematicEnabled); needs a rigid body on this prim.", False
        ),
        mass=field(
            "number", "Rigid-body mass in kilograms (stage mass units), through PhysicsMassAPI. Refused together with density.", False
        ),
        density=field(
            "number", "Rigid-body density in kg/m^3; the collider's volume sets the mass. Refused together with mass.", False
        ),
        impulse=field("array", "A one-off push [x, y, z] in N s on a rigid body in the running simulation (play or pause), for poking something. Edits nothing, and is the only option here that works while playing. On Newton it is applied as impulse/(dt/substeps) force in the next step's first substep.", False),
        at=field("array", "With impulse: the world point [x, y, z] to push at. Default: the body's centre.", False),
    ),
)
add(
    "sim.prim.revert",
    "Undo this session's edits on a prim or subtree. Removes the session-layer opinions these tools authored on one prim or its subtree, restoring composed values from the source files, and reports what was removed and the values before and after. Source files never change. Prims the pack created stay: delete them with sim__prim__delete. Nothing recorded is a no-op, not an error. Only edits recorded in this session and stage can be undone.",
    dict(
        **TARGET,
        path=field("string", "Absolute prim path whose recorded edits are undone."),
        subtree=field(
            "boolean", "Also revert recorded edits on descendants.", False, default=True
        ),
    ),
)
add(
    "sim.prim.create_many",
    "Create up to 200 prims in one call, all or none. A tower, wall, grid or room without one sim__prim__create per piece. Each item takes that tool's arguments plus an optional material path; defaults fills any key an item leaves out, except keys that do not apply to its kind (an Xform skips rigid, mass and material; listed in defaults_skipped). Ancestors must exist, timeline stopped. A failing item removes the earlier ones; the error keeps its code and names index and path.",
    dict(
        **TARGET,
        items=field(
            "array",
            "1 to 200 objects taking create's arguments; path is required, kind may come from defaults.",
        ),
        defaults=field(
            "object",
            "Optional arguments shared by every item, for example {kind: Cube, rigid: true, mass: 0.015, material: /World/Looks/Wood}. An item's own value wins; keys that do not apply to an item's kind (rigid, mass or material on an Xform) are skipped for it.",
            False,
        ),
    ),
)

REGISTRY["sim.camera.capture"]["parameters"]["wait_for_render"] = field(
    "boolean",
    "True: wait up to five seconds for render completions and a new owned CPU transfer. False: reuse the last validated tool acquisition, possibly from before Stop/reset or scene edits; NOT_READY until a true capture seeds it. Ordinary Play never refreshes this cache. PNG I/O still occurs. No implicit physics advance or atomic sensor/physics guarantee.",
    False,
    default=True,
)
REGISTRY["sim.lidar.read"]["parameters"]["wait_for_render"] = field(
    "boolean",
    "Wait up to five seconds for one render; acquisition requires Play. False reads the latest completed scan, possibly stale. No implicit physics advance or atomic sensor/physics guarantee.",
    False,
    default=False,
)

# Clients validate the required list before a call reaches this server, with their own error text
# (OpenCode: "session_id: Missing key"), so the description is where to say how to get the IDs.
for _meta in REGISTRY.values():
    _target = _meta["parameters"].get("session_id")
    if _target and _target["required"]:
        _meta["description"] += " Requires session_id and stage_id from sim__stage__info."
