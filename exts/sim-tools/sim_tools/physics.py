"""Backend-specific runtime observations. Never substitute authored USD state."""

from twinsmith.project_assets import AssetError, refuse
from .core import vector

# Qualified by the disposable two-cycle motion/joint/timing/export trials on all
# five recorded releases. Build and loaded extension/Warp drift still refuses.
NEWTON_QUALIFIED = True


def body_path(args):
    paths = [args[key] for key in ("path", "prim", "prim_path") if args.get(key) is not None]
    if not paths or any(not isinstance(path, str) or not path for path in paths):
        refuse("INVALID_VALUE", "Supply path (or its alias prim or prim_path) as an absolute rigid body path.")
    if len(set(paths)) != 1:
        refuse("INVALID_VALUE", "path, prim and prim_path refer to the same body; supply only one path or matching values.")
    return paths[0]


def manager():
    try:
        from isaacsim.core.simulation_manager import SimulationManager as sm
    except ImportError:
        refuse(
            "PHYSICS_API_UNAVAILABLE",
            "Enable the installed isaacsim.core.simulation_manager extension.",
        )
    from .compat import adapt_simulation_manager

    sm = adapt_simulation_manager(sm)
    engine = sm.get_active_physics_engine()
    if engine not in ("physx", "newton") or (
        engine == "newton" and not NEWTON_QUALIFIED
    ):
        refuse(
            "BACKEND_UNVERIFIED",
            f"Runtime adapter for {engine!r} is not qualified; no PhysX or USD fallback.",
        )
    if engine == "newton":
        from .runtime import runtime_identity
        from twinsmith.sim_versions import newton_stack_issues

        issues = newton_stack_issues(runtime_identity())
        if issues:
            refuse(
                "BACKEND_BUILD_UNVERIFIED",
                "Cannot select a qualified Newton adapter: " + "; ".join(issues),
            )
    return sm


def array(value):
    """Observed tensor data as NumPy, whatever frontend this build's view returned."""
    if hasattr(value, "detach"):  # torch
        return value.detach().cpu().numpy()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return value


def _frontend_data(view, row):
    """One (1, DOFs) row in the same frontend and on the same device as the view's own arrays."""
    module = type(view.get_dof_positions()).__module__
    if module.startswith("warp"):
        import warp as wp

        return wp.array(row, dtype=wp.float32, device=view.get_dof_positions().device)
    if module.startswith("torch"):
        import torch

        return torch.from_numpy(row).to(device=view.get_dof_positions().device)
    return row


def _frontend_indices(view):
    """The articulation index in the view's own frontend."""
    import numpy as np

    module = type(view.get_dof_positions()).__module__
    if module.startswith("warp"):
        import warp as wp

        return wp.array(np.array([0], dtype=np.uint32), dtype=wp.uint32,
                        device=view.get_dof_positions().device)
    if module.startswith("torch"):
        import torch

        return torch.tensor([0], dtype=torch.int32, device=view.get_dof_positions().device)
    return np.array([0], dtype=np.uint32)


def dof_names(view):
    return list(view.shared_metatype.dof_names)


def dof_units(view):
    """Per-DOF position units: 'rad' for a revolute DOF, stage units for a prismatic one."""
    names = dof_names(view)
    try:
        from omni.physics.tensors import DofType

        types = list(view.shared_metatype.dof_types)
        return ["stage units" if t == DofType.Translation else "rad" for t in types]
    except Exception:  # noqa: BLE001 - informational only; the value is still valid
        return ["rad, or stage units for a prismatic DOF"] * len(names)


def dof_limits(view):
    """[[low, high], ...] for one articulation, from either build's return shape.

    The tensor view used here returns one ``(1, DOFs, 2)`` array on both Isaac Sim 5.1 and 6.x;
    the experimental wrapper on some builds returns ``(lower, upper)`` instead.
    """
    value = view.get_dof_limits()
    if isinstance(value, tuple):
        if len(value) != 2:
            refuse("PHYSICS_FORMAT", "Runtime DOF limits must be a pair of bounds.")
        lower, upper = (array(part).reshape(-1).tolist() for part in value)
        return [[float(lo), float(hi)] for lo, hi in zip(lower, upper)]
    data = array(value)
    if getattr(data, "ndim", None) != 3 or data.shape[0] != 1 or data.shape[2] != 2:
        refuse(
            "PHYSICS_FORMAT",
            "Runtime DOF limits have an unexpected shape on this build; no targets were changed.",
        )
    return [[float(lo), float(hi)] for lo, hi in data[0].tolist()]


def dof_gains(view):
    """(stiffnesses, dampings) for one articulation, from either build's gain API."""
    try:
        stiffnesses, dampings = view.get_dof_stiffnesses(), view.get_dof_dampings()
    except AttributeError:
        value = view.get_dof_gains()
        if not isinstance(value, tuple) or len(value) != 2:
            refuse(
                "PHYSICS_API_UNAVAILABLE",
                "This build exposes neither get_dof_stiffnesses/get_dof_dampings nor a gain pair; no gains were changed.",
            )
        stiffnesses, dampings = value
    return (
        array(stiffnesses).reshape(-1).tolist(),
        array(dampings).reshape(-1).tolist(),
    )


def dof_max_forces(view):
    """Drive caps, under this build's own name for them."""
    try:
        value = view.get_dof_max_forces()
    except AttributeError:
        try:
            value = view.get_dof_max_efforts()
        except AttributeError:
            refuse(
                "PHYSICS_API_UNAVAILABLE",
                "This build exposes neither get_dof_max_forces nor get_dof_max_efforts; no gains were changed.",
            )
    return array(value).reshape(-1).tolist()


def dof_row(view, value):
    """One articulation's DOF row as the frontend this build uses (warp on 6.x, NumPy/other below)."""
    import numpy as np

    return np.array(array(value), dtype=np.float32, copy=True).reshape(1, -1)


def dof_write(view, setter, row):
    """Call a tensor setter with the (1, DOFs) row and this view's own frontend/device."""
    getattr(view, setter)(_frontend_data(view, row), _frontend_indices(view))


def dof_set(view, getter, setter, values):
    """Write only the given per-DOF values, leaving the rest of the row untouched."""
    row = dof_row(view, getattr(view, getter)())
    for index, value in values.items():
        row[0, index] = value
    dof_write(view, setter, row)


def runtime_vector(value, count, *, path, field, backend):
    """Validate observed solver data without reporting it as a bad user pose.

    Do not return nonfinite numbers or replace them with authored USD values.
    This is an observation failure, not proof that resetting or retrying is safe.
    """
    try:
        return vector(value, count)
    except AssetError as exc:
        refuse(
            "PHYSICS_STATE_INVALID",
            f"{backend} returned invalid {field} for {path}: {exc}",
            path=path,
            field=field,
            backend=backend,
            effects="Runtime state is invalid. Inspect solver diagnostics before further stepping; no USD fallback or automatic reset was performed.",
        )


def prepare(stage):
    """Prepare only the already-selected engine, before first Play.

    6.0.0.0 uses the Python Newton tensor adapter; later releases ship a
    separate native tensor extension which the base Python app does not enable.
    This never switches engines, chooses a solver or alters a user's USD units.
    """
    sm = manager()
    if sm.get_active_physics_engine() != "newton":
        return
    from pxr import UsdGeom
    from .runtime import runtime_identity

    identity = runtime_identity()
    if not identity["qualified_build"]:
        refuse(
            "BACKEND_BUILD_UNVERIFIED",
            "Newton build identity is missing/conflicting or has not been qualified.",
        )
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        refuse(
            "BACKEND_UNITS_UNSUPPORTED",
            "Installed Newton requires metersPerUnit=1. Author/convert a separate project in meters; the existing stage has not been rescaled.",
        )
    if identity["release"] != "6.0.0.0":
        import omni.kit.app

        manager_ext = omni.kit.app.get_app().get_extension_manager()
        name = "isaacsim.physics.newton.tensors"
        if not manager_ext.is_extension_enabled(name):
            if not manager_ext.set_extension_enabled_immediate(name, True):
                refuse(
                    "PHYSICS_API_UNAVAILABLE",
                    "The selected Newton engine needs its installed isaacsim.physics.newton.tensors extension before Play.",
                )


def view_for(path, articulation=False):
    sm = manager()
    view = sm.get_physics_simulation_view()
    if view is None:
        refuse(
            "PHYSICS_NOT_READY",
            "Backend tensor view is not initialized. Play, wait for warm-up, then pause.",
        )
    result = (
        view.create_articulation_view(path)
        if articulation
        else view.create_rigid_body_view(path)
    )
    if result.count != 1 or not result.check():
        refuse(
            "PHYSICS_NOT_READY",
            "Expected exactly one valid runtime view; model/stage may have changed. No USD fallback.",
        )
    if articulation and result.max_dofs > 128:
        refuse("LIMIT", "Articulation exceeds 128 DOFs.")
    return result


def body_state(path, time_s, include_dynamics=False, stage=None):
    if type(include_dynamics) is not bool:
        refuse("INVALID_VALUE", "include_dynamics must be boolean.")
    sm = manager()
    engine = sm.get_active_physics_engine()
    if engine == "physx":
        import omni.physx

        pose = omni.physx.get_physx_interface().get_rigidbody_transformation(path)
        if not pose.get("ret_val"):
            refuse(
                "PHYSICS_NOT_READY",
                "PhysX has no initialized body at this path. No USD fallback.",
            )
        q, position = list(pose["rotation"]), list(pose["position"])
        source = "PhysX get_rigidbody_transformation"
    else:
        view = view_for(path)
        transform = array(view.get_transforms()).tolist()[0]
        if len(transform) != 7:
            refuse("PHYSICS_FORMAT", "Newton transform must have seven components.")
        position, q = transform[:3], transform[3:]
        source = "Newton rigid-body tensor view"
    runtime_vector(q, 4, path=path, field="orientation_xyzw", backend=engine)
    runtime_vector(position, 3, path=path, field="world_position", backend=engine)
    result = dict(
        path=path,
        world_position=position,
        orientation_wxyz=[q[3], *q[:3]],
        units="stage units",
        source=source,
        backend=engine,
        time_s=time_s,
        physics_time_s=sm.get_simulation_time(),
    )
    if include_dynamics:
        from pxr import UsdPhysics
        import math

        if stage is None:
            refuse("STALE_TARGET", "Dynamics require the explicitly selected stage.")
        view = view_for(path)
        velocities = array(view.get_velocities()).tolist()[0]
        runtime_vector(velocities, 6, path=path, field="velocities", backend=engine)
        masses = array(view.get_masses()).reshape(-1).tolist()
        if len(masses) != 1 or not math.isfinite(masses[0]) or masses[0] < 0:
            refuse("PHYSICS_FORMAT", "Expected one finite nonnegative runtime mass.")
        result["dynamics"] = dict(
            linear_velocity=velocities[:3],
            angular_velocity=velocities[3:],
            linear_units="stage units/second, world frame at center of mass",
            angular_units="radians/second, world frame",
            mass=masses[0],
            mass_kg=masses[0] * UsdPhysics.GetStageKilogramsPerUnit(stage),
            source=engine + " rigid-body tensor view",
        )
    return result


RENDER_POSITION_TOLERANCE_M = 0.005
RENDER_ANGLE_TOLERANCE_DEG = 2.0


def pose_gap(position_a, wxyz_a, position_b, wxyz_b):
    """(distance, angle in degrees) between two poses."""
    import math

    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position_a, position_b)))
    dot = abs(sum(a * b for a, b in zip(wxyz_a, wxyz_b)))
    norm = math.sqrt(sum(a * a for a in wxyz_a) * sum(b * b for b in wxyz_b)) or 1.0
    return distance, math.degrees(2 * math.acos(max(-1.0, min(1.0, dot / norm))))


def _matrix_pose(rows):
    from pxr import Gf

    transform = Gf.Transform(Gf.Matrix4d(*[float(v) for row in rows for v in row]))
    q = transform.GetRotation().GetQuat()
    return list(transform.GetTranslation()), [q.GetReal(), *q.GetImaginary()]


def _fabric_pose(path):
    """The world pose Fabric holds for a prim (what the renderer draws when physics writes to Fabric
    only), or None when this build or prim has none."""
    try:
        import omni.usd
        import usdrt

        prim = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id()).GetPrimAtPath(path)
        attribute = prim.GetAttribute("omni:fabric:worldMatrix") if prim else None
        value = attribute.Get() if attribute else None
        if value is None:
            return None
        return _matrix_pose([[value[i][j] for j in range(4)] for i in range(4)])
    except Exception:  # noqa: BLE001 - optional evidence; USD is checked instead
        return None


def render_check(stage, path, position, orientation_wxyz):
    """Compare a body's physics pose with the poses the renderer can draw it from.

    The renderer reads Fabric or USD, depending on how the engine publishes: PhysX usually writes
    USD, Newton writes Fabric and leaves USD at the authored pose. So the picture is right when
    either one holds the physics pose, and a warning means neither does: the viewport, camera
    captures and recordings then do not show where the body is. Returns (report, warning or None);
    the report is None when no pose could be read."""
    from pxr import Usd, UsdGeom

    poses = {}
    fabric = _fabric_pose(path)
    if fabric is not None:
        poses["Fabric"] = fabric
    prim = stage.GetPrimAtPath(path)
    xformable = UsdGeom.Xformable(prim) if prim else None
    if xformable:
        matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        poses["USD"] = _matrix_pose([[matrix[i][j] for j in range(4)] for i in range(4)])
    if not poses:
        return None, None
    tolerance = RENDER_POSITION_TOLERANCE_M / (UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    gaps = {source: pose_gap(position, orientation_wxyz, *pose) for source, pose in poses.items()}
    matching = [source for source, (distance, angle) in gaps.items()
                if distance <= tolerance and angle <= RENDER_ANGLE_TOLERANCE_DEG]
    source = matching[0] if matching else min(gaps, key=lambda name: gaps[name])
    distance, angle = gaps[source]
    report = dict(source=source, agrees=bool(matching), position_gap=round(distance, 6), angle_gap_deg=round(angle, 3),
                  checked=sorted(poses))
    if matching:
        return report, None
    return report, (
        f"The rendered pose of {path} is {distance:.4g} stage units and {angle:.3g} degrees away from its physics "
        f"pose (closest of {' and '.join(sorted(poses))}), so the viewport, camera captures and recordings do not show "
        "where the body is. Physics is not publishing poses to the renderer: on Newton check that Fabric updates are "
        "enabled and step with sim__simulation__step or sim__simulation__run (both publish each frame); instanced "
        "visuals can also stay behind (import with assets__import_path, which expands them).")


def render_publication():
    """How the engine's body poses reach the renderer, for a result that carries a picture or a scan; None on PhysX.

    PhysX writes USD, which every renderer reads. Newton writes only Fabric and leaves USD at the authored pose, so a picture
    shows the simulated pose only while Newton's Fabric updates are on, and after a paused step only once that step has been
    published (sim__simulation action=step and action=run do it; an exact step in your own code does not)."""
    from .newton_support import engine_name

    if engine_name() != "newton":
        return None
    try:
        from isaacsim.physics.newton import acquire_stage

        native = acquire_stage()
        enabled = bool(native.cfg.update_fabric) if native is not None else None
    except Exception:  # noqa: BLE001 - evidence for a caption; never worth failing the capture
        enabled = None
    info = dict(backend="newton", poses_published_to="Fabric (USD keeps the authored pose)", fabric_updates=enabled)
    if enabled is False:
        info["warning"] = ("Newton's Fabric updates are off, so this picture shows the authored poses, not the simulated ones. "
                           "Enable update_fabric in Newton's configuration before Play.")
    elif enabled is None:
        info["note"] = "Could not read Newton's Fabric setting; compare the picture with sim__body__state, which reports render_pose."
    else:
        info["note"] = ("Follows physics while playing, and after sim__simulation action=step or action=run. A paused change "
                        "made another way shows the last published pose.")
    return info


def physics_ready():
    """True once the selected engine's tensor view exists (a few frames after the first Play), False
    while it does not, None when this build cannot say (no simulation manager, an unqualified engine)."""
    try:
        return manager().get_physics_simulation_view() is not None
    except AssetError:
        return None
    except Exception:  # noqa: BLE001 - a view that cannot be asked for yet is not ready
        return False


def on_physics_step(callback):
    """Call ``callback(step_dt, context)`` before every physics step while the timeline plays.
    Returns the id to pass to remove_physics_step."""
    manager()  # refuses an engine these tools are not qualified for
    import isaacsim.core.simulation_manager as simulation_manager

    events = getattr(simulation_manager, "SimulationEvent", None)
    event = events.PHYSICS_PRE_STEP if events is not None else simulation_manager.IsaacEvents.PRE_PHYSICS_STEP
    return simulation_manager.SimulationManager.register_callback(callback, event=event)


def remove_physics_step(callback_id):
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.deregister_callback(callback_id)


def newton_stage(stage):
    import omni.usd
    from isaacsim.physics.newton import acquire_stage

    native = acquire_stage()
    if (
        native is None
        or not native.initialized
        or native.stage_id != omni.usd.get_context().get_stage_id()
        or omni.usd.get_context().get_stage() != stage
    ):
        refuse("PHYSICS_NOT_READY", "Newton's initialized stage is stale.")
    return native


CAPTURE_KEPT = ("6.2.0.0",)     # releases whose own Newton step is used as it is, CUDA graph capture included


def step(stage, steps):
    """Advance physics from a pause, without changing the timeline or yielding to the app loop.

    On the releases in CAPTURE_KEPT Newton steps as it does in play, with its captured CUDA graph: that release's
    ``step_sim`` recaptures the graph when the timestep or the substep count changes and keeps ``state_0`` in one
    buffer, and a captured step replays the same kernels. Measured on 6.2.0.0 with the UR5e work cell (22 bodies,
    801 shapes): 0.45 ms a step against 7.6 ms uncaptured, the same end state to 1e-8 after 500 steps (two uncaptured
    runs differ by as much), and joint gains written between steps take effect. A step that raises drops the graph, so
    the next one captures afresh: the release keeps whatever was captured before the failure.

    Older releases keep the direct path. Newton <=6.0.1 increments its clock while skipping the solver when its
    internal playing flag is false, so that flag is set for the step. Their capture code also skips the first
    captured step and bakes in dt; stepping the solver directly avoids both, and the cached graph is dropped because
    direct steps may swap state buffers it refers to.
    """
    sm = manager()
    if sm.get_active_physics_engine() != "newton":
        sm.step(steps=steps)
        return
    from .runtime import runtime_identity

    identity = runtime_identity()
    native = newton_stage(stage)
    if identity["release"] in CAPTURE_KEPT:
        try:
            sm.step(steps=steps)
        except BaseException:
            native.graph = None     # a step that failed while capturing leaves a cut-off graph, which later steps would replay
            raise
        return
    playing = native.playing
    capture = native.cfg.use_cuda_graph
    try:
        if identity["release"] in ("6.0.0.0", "6.0.0.1", "6.0.1.0"):
            native.playing = True
        native.cfg.use_cuda_graph = False
        native.graph = None
        sm.step(steps=steps)
    finally:
        native.playing = playing
        native.cfg.use_cuda_graph = capture
        native.graph = None


def sync_fabric(stage):
    """Publish paused Newton steps to rendering, without pumping Kit or editing USD."""
    sm = manager()
    if sm.get_active_physics_engine() != "newton":
        return False
    native = newton_stage(stage)
    if not callable(getattr(native, "update_fabric_attrs", None)):
        refuse(
            "PHYSICS_API_UNAVAILABLE",
            "Newton cannot publish paused runtime poses to Fabric.",
        )
    if not native.cfg.update_fabric:
        refuse(
            "FABRIC_DISABLED",
            "Enable Newton's Fabric updates before step-and-observe rendering.",
        )
    native.update_fabric_attrs()
    return True


def snapshot_poses(stage, time_s):
    """Bounded rigid-body poses for export only. Not a solver checkpoint."""
    sm = manager()
    if sm.get_active_physics_engine() != "newton":
        return []
    from pxr import UsdPhysics

    paths = []
    for index, prim in enumerate(stage.Traverse()):
        if index >= 10000:
            refuse("LIMIT", "Runtime snapshot traversal exceeds 10000 prims.")
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        ):
            if prim.IsInstance() or prim.IsInstanceProxy():
                refuse(
                    "SNAPSHOT_UNSUPPORTED",
                    "Runtime pose export of instanced rigid bodies is unsupported.",
                )
            paths.append(str(prim.GetPath()))
            if len(paths) > 256:
                refuse("LIMIT", "Runtime pose snapshot exceeds 256 rigid bodies.")
    return [body_state(p, time_s) for p in paths]
