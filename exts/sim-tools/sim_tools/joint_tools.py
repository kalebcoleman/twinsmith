"""Joint tools: inspect an articulation, set drives and gains, command, move, teleport and read state."""

import math
from pxr import Usd, UsdPhysics
from twinsmith.project_assets import AssetError, refuse
from .core import bounded, vector
from .values import finite_clock, scalar


def runtime_dof_limits(path):
    """This articulation's [[low, high], ...], or None if this build or backend can't say."""
    from .physics import dof_limits, view_for

    try:
        return dof_limits(view_for(path, articulation=True))
    except (AttributeError, IndexError, TypeError, ValueError):
        return None


def joints_past_limits(names, positions, limits, slack=0.02):
    """The DOFs clearly outside their limits. A drive can press slightly past one, so a small slack is allowed."""
    if not limits or len(limits) != len(positions):
        return []
    return [name for name, value, (low, high) in zip(names, positions, limits)
            if value < low - slack or value > high + slack]


# Joint limits are stored as float32, and a radian target is converted to degrees as float64, so a
# target exactly at a limit can land a few float32 steps past it (RG2: 1.0 rad is 57.29577951 degrees,
# authored as 57.29577637). About 8 float32 steps, relative to the limit's size.
LIMIT_ROUNDING = 1e-6


STATE_ECHO_TOLERANCE = 1e-4


UNSTABLE_SPEED = 500.0  # rad/s or stage units/s: no real joint turns this fast


def joints_too_fast(names, velocities):
    return [name for name, speed in zip(names, velocities) if abs(speed) > UNSTABLE_SPEED][:10]


def to_limit(value, lo, hi):
    """``value``, or the limit it only rounds past; None when it is really outside [lo, hi]."""
    for limit, past in ((lo, lo is not None and value < lo), (hi, hi is not None and value > hi)):
        if past:
            return limit if abs(value - limit) <= LIMIT_ROUNDING * max(1.0, abs(limit)) else None
    return value


def command_target(value, lo, hi, current):
    """A commanded position, or None if it is outside the limits. A joint resting on a limit reads a
    little past it, so sending back the pose from sim__joints__state means that limit, not an error."""
    limited = to_limit(value, lo, hi)
    if limited is None and abs(value - current) <= STATE_ECHO_TOLERANCE:
        return lo if value < lo else hi
    return limited


def bounded_names(names, limit=32):
    shown = ", ".join(names[:limit])
    return shown + (f", ... ({len(names)} in all)" if len(names) > limit else "")


def resolve_named(values, names, label):
    """[(index, name, requested)] from {dof name: number}; an unknown name lists the valid ones."""
    if not isinstance(values, dict) or not values:
        refuse(
            "INVALID_VALUE",
            f"{label} must be a non-empty object of {{dof name: number}}. Valid DOF names: "
            f"{bounded_names(names)}.",
        )
    unknown = [key for key in values if not isinstance(key, str) or key not in names]
    if unknown:
        refuse(
            "UNKNOWN_DOF",
            f"{label} names {unknown[:8]!r}, which this articulation does not have. Valid DOF names: "
            f"{bounded_names(names)}.",
            valid_names=names,
        )
    rows = []
    for name, value in values.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            refuse("INVALID_VALUE", f"{label}[{name!r}] must be a finite number, not {value!r}.")
        rows.append((names.index(name), name, float(value)))
    return rows


def allowed_targets(rows, limits, current, units, clamp, label):
    """Applied per-DOF targets, plus what was clamped; refuses a real excess unless clamp=true."""
    applied, clamped, rounded = {}, [], []
    for index, name, requested in rows:
        low, high = limits[index]
        if math.isnan(low) or math.isnan(high) or low > high:
            refuse("PHYSICS_FORMAT", f"Runtime limits for {name} are invalid; no targets were changed.")
        limited = command_target(requested, low, high, current[index])
        if limited is None:
            if not clamp:
                refuse(
                    "JOINT_LIMIT",
                    f"{label} {requested:g} for {name} ({units[index]}) is outside its runtime limits "
                    f"[{low:g}, {high:g}]. Resend with clamp=true to use the limit instead, or pick a value "
                    "inside the limits.",
                    name=name,
                    requested=requested,
                    limits=[low, high],
                    units=units[index],
                )
            limited = low if requested < low else high
            clamped.append(
                dict(name=name, requested=requested, applied=limited, limits=[low, high], units=units[index])
            )
        elif limited != requested:
            rounded.append(dict(name=name, requested=requested, applied=limited, units=units[index]))
        applied[index] = (name, requested, float(limited), low, high, units[index])
    return applied, clamped, rounded


# Visited while looking for an articulation's joints; the caller's limit bounds what is returned.
MAX_ARTICULATION_SCAN_PRIMS = 20000


def articulation_joints(stage, root, limit):
    """(joints, truncated) of the articulation under ``root``: the joints reached through its rigid
    bodies, in stage order. ``root`` may be the articulation's root joint, a link that carries the
    articulation API, or the prim holding all of them. A joint excluded from the articulation is left
    out, and so is another articulation under the same parent."""
    if root.IsA(UsdPhysics.Joint):
        # A root joint is a leaf under the joints' own folder: its siblings are the articulation's.
        parent = root.GetParent()
        scan = stage.GetPseudoRoot() if parent.IsPseudoRoot() else parent
    else:
        # A link's joints can sit in a sibling folder (a URDF import puts them under
        # <robot>/Asset/Physics while the bodies are under <robot>/Asset/Geometry), so look at the
        # whole stage. The body walk below still keeps other articulations out.
        scan = stage.GetPseudoRoot()
    found, truncated = [], False
    for count, prim in enumerate(Usd.PrimRange(scan)):
        if count >= MAX_ARTICULATION_SCAN_PRIMS:
            truncated = True
            break
        if prim.IsA(UsdPhysics.Joint):
            found.append(prim)

    def bodies(prim):
        joint = UsdPhysics.Joint(prim)
        return {str(t) for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()) for t in rel.GetTargets()}

    if root.IsA(UsdPhysics.Joint):
        reached = bodies(root)
        joints = [root]
    else:
        # A link or the robot's own prim: start from the rigid bodies at or below it. Its joints are
        # siblings, so a scan below the link alone would find none.
        reached = {str(prim.GetPath()) for prim in Usd.PrimRange(root)
                   if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
        joints = []
    pending = [p for p in found if p != root and not UsdPhysics.Joint(p).GetExcludeFromArticulationAttr().Get()]
    grew = True
    while grew:
        grew = False
        for prim in list(pending):
            if bodies(prim) & reached:
                reached |= bodies(prim)
                joints.append(prim)
                pending.remove(prim)
                grew = True
    order = {prim: index for index, prim in enumerate(found)}
    joints.sort(key=lambda prim: order.get(prim, -1))
    if len(joints) > limit:
        return joints[:limit], True
    return joints, truncated


class JointTools:
    """Mixed into Advanced: uses its scene (``self.s``), sensors and ``physics_manager``."""

    def joints(self, stage, p):
        s = self.s
        prim = s.prim(stage, p)
        if s.timeline.is_stopped() or not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            refuse(
                "PHYSICS_NOT_READY",
                "The timeline is stopped: play first." if s.timeline.is_stopped() else
                f"{p} is not an articulation root. sim__physics__inspect lists them, or sim__ros2_tf reports it.",
            )
        sm = self.physics_manager()
        from .physics import array as observed, runtime_vector, view_for

        art = view_for(p, articulation=True)

        def values(data):
            return observed(data).tolist()[0]

        names = list(art.shared_metatype.dof_names)
        positions = values(art.get_dof_positions())
        velocities = values(art.get_dof_velocities())
        backend = sm.get_active_physics_engine()
        runtime_vector(
            positions, len(names), path=p, field="joint_positions", backend=backend
        )
        runtime_vector(
            velocities, len(names), path=p, field="joint_velocities", backend=backend
        )
        return dict(
            path=p,
            positions=positions,
            velocities=velocities,
            names=names,
            source=sm.get_active_physics_engine() + " tensor view",
            backend=sm.get_active_physics_engine(),
            units="angular radians, prismatic stage units",
        )

    def tool_joints_inspect(self, name, stage, a, camera_frame=None):
        s = self.s
        nodes, truncated = bounded(stage, a["path"], a.get("limit", 200))
        # An articulation root is often a leaf joint (<robot>/root_joint): the joints that
        # matter are its siblings, connected through the links, not below it.
        root_prim = s.prim(stage, a["path"])
        scope = {}
        if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            nodes, truncated = articulation_joints(stage, root_prim, a.get("limit", 200))
            if nodes:
                scope = dict(scope="articulation", articulation_root=a["path"])
            else:  # nothing linked through bodies: keep the subtree view rather than an empty list
                nodes, truncated = bounded(stage, a["path"], a.get("limit", 200))
        joints = []
        for prim in nodes:
            if prim.IsA(UsdPhysics.Joint):
                attrs = {
                    key: prim.GetAttribute(key).Get()
                    for key in (
                        "physics:lowerLimit",
                        "physics:upperLimit",
                        "drive:angular:physics:targetPosition",
                        "drive:angular:physics:stiffness",
                        "drive:angular:physics:damping",
                        "drive:angular:physics:maxForce",
                        "drive:linear:physics:targetPosition",
                        "drive:linear:physics:stiffness",
                        "drive:linear:physics:damping",
                        "drive:linear:physics:maxForce",
                    )
                    if prim.HasAttribute(key)
                }
                entry = dict(path=str(prim.GetPath()), type=prim.GetTypeName(), authored=attrs)
                drive_keys = [k for k in attrs if ":physics:stiffness" in k or ":physics:damping" in k or ":physics:maxForce" in k]
                if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
                    if not drive_keys:
                        entry["no_drive"] = True  # nothing authored that could move it under a position target
                joints.append(entry)
        undriven = [j["path"] for j in joints if j.get("no_drive")]
        return dict(
            joints=joints,
            truncated=truncated,
            **({"joints_without_drive": undriven,
                "drive_note": "These joints have no authored drive, so a position target does nothing unless the joint is "
                              "mimicked or driven another way (a gripper's second finger is a common case). Give one gains "
                              "with sim__joints__gains or a drive with sim__joints__drive, on a stopped timeline."} if undriven else {}),
            **scope,
            units="USD angular degrees; prismatic stage units",
            source="authored USD, not DOF positions",
        )

    def tool_joints_drive(self, name, stage, a, camera_frame=None):
        s = self.s
        prim = s.prim(stage, a["path"])
        angular = prim.IsA(UsdPhysics.RevoluteJoint)
        if not angular and not prim.IsA(UsdPhysics.PrismaticJoint):
            refuse("WRONG_TYPE", "Expected revolute/prismatic joint.")
        target = scalar(a["target"], -1e6, 1e6)
        stiff = scalar(a["stiffness"])
        damp = scalar(a["damping"])
        force = scalar(a.get("max_force", 1000), 1e-9)
        usd_target = math.degrees(target) if angular else target
        lo = prim.GetAttribute("physics:lowerLimit").Get()
        hi = prim.GetAttribute("physics:upperLimit").Get()
        limited = to_limit(usd_target, lo, hi)
        if limited is None:
            unit = "degrees" if angular else "stage units"
            refuse("JOINT_LIMIT", f"Requested target ({usd_target:g} {unit}) is outside the authored joint "
                   f"limits [{lo}, {hi}] {unit}.", target=usd_target, lower=lo, upper=hi, unit=unit)
        at_limit = limited != usd_target
        usd_target = limited
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            drive = UsdPhysics.DriveAPI.Apply(
                prim, "angular" if angular else "linear"
            )
            drive.CreateTargetPositionAttr(usd_target)
            drive.CreateStiffnessAttr(stiff)
            drive.CreateDampingAttr(damp)
            drive.CreateMaxForceAttr(force)
        return dict(
            path=a["path"],
            requested_target=target,
            authored_target=usd_target,
            unit="degrees" if angular else "stage units",
            effect="drive target authored; motion unverified until runtime observation",
            **({"note": "Target rounded onto the authored limit, which is stored as float32."} if at_limit else {}),
        )

    def tool_joints_command(self, name, stage, a, camera_frame=None):
        state = self.joints(stage, a["path"])
        names = state["names"]
        if not names:
            refuse("JOINT_COUNT", "Selected articulation has no commandable DOFs.")
        positions_arg, targets_arg = a.get("positions"), a.get("targets")
        if positions_arg is None and targets_arg is None and a.get("velocities") is None:
            refuse(
                "INVALID_VALUE",
                "Give positions (one per DOF, in sim__joints__state order), targets ({dof name: value}) "
                f"or velocities ({{dof name: value}}). Valid DOF names: {bounded_names(names)}.",
            )
        if positions_arg is not None and targets_arg is not None:
            refuse("CONFLICT", "Give positions or targets, not both. Nothing was changed.")
        clamp = a.get("clamp", False)
        if type(clamp) is not bool:
            refuse("INVALID_VALUE", "clamp must be true or false.")
        from .physics import array as observed, dof_limits, dof_set, dof_units, view_for

        view = view_for(a["path"], articulation=True)
        if positions_arg is not None:
            positions = vector(positions_arg, len(names))
            rows = [(index, name, float(positions[index])) for index, name in enumerate(names)]
        elif targets_arg is not None:
            rows = resolve_named(targets_arg, names, "targets")
        else:
            rows = []
        limits, units = dof_limits(view), dof_units(view)
        applied, clamped, rounded = allowed_targets(
            rows, limits, state["positions"], units, clamp, "Target"
        )
        previous = dict(enumerate(observed(view.get_dof_position_targets()).reshape(-1).tolist()))
        if applied:
            dof_set(view, "get_dof_position_targets", "set_dof_position_targets",
                    {index: row[2] for index, row in applied.items()})
        velocities = []
        if a.get("velocities") is not None:
            velocity_rows = resolve_named(a["velocities"], names, "velocities")
            before_velocity = observed(view.get_dof_velocity_targets()).reshape(-1).tolist()
            dof_set(view, "get_dof_velocity_targets", "set_dof_velocity_targets",
                    {index: value for index, _, value in velocity_rows})
            for index, name, value in velocity_rows:
                velocities.append(dict(name=name, index=index, requested=value, applied=value,
                                       previous=before_velocity[index],
                                       units=units[index] + "/s"))
        return dict(
            path=a["path"],
            backend=state["backend"],
            names=names,
            units="rad for revolute DOFs, stage units for prismatic ones; velocities per second",
            targets=[
                dict(name=row[0], index=index, requested=row[1], applied=row[2], previous=previous.get(index),
                     limits=[row[3], row[4]], units=row[5], clamped=row[1] != row[2])
                for index, row in applied.items()
            ],
            **({"velocities": velocities} if velocities else {}),
            clamped=clamped,
            rounded=rounded,
            effect="Targets set; the joints have not moved yet. Compare sim__joints__state while the "
                   "simulation runs, or settle them with sim__joints__move. A joint that stops short is "
                   "usually in contact (a low arm against the ground) or has a controller replacing its target.",
            runtime_only="Lost on Stop and never saved; sim__joints__drive authors USD targets instead.",
        )

    def tool_joints_gains(self, name, stage, a, camera_frame=None):
        state = self.joints(stage, a["path"])
        names = state["names"]
        if not names:
            refuse("JOINT_COUNT", "Selected articulation has no commandable DOFs.")
        dofs = a.get("dofs") or []
        if not isinstance(dofs, list) or any(not isinstance(name, str) for name in dofs):
            refuse("INVALID_VALUE", f"dofs must be a list of DOF names; valid names: {bounded_names(names)}.")
        unknown = [name for name in dofs if name not in names]
        if unknown:
            refuse(
                "UNKNOWN_DOF",
                f"dofs names {unknown[:8]!r}, which this articulation does not have. Valid DOF names: "
                f"{bounded_names(names)}.",
                valid_names=names,
            )
        selected = [names.index(name) for name in dofs] if dofs else list(range(len(names)))
        values = {}
        for label in ("stiffness", "damping"):
            if a.get(label) is not None:
                values[label] = scalar(a[label])
        if a.get("max_effort") is not None:
            values["max_effort"] = scalar(a["max_effort"], 1e-9)
        from .physics import dof_gains, dof_max_forces, dof_row, dof_write, view_for

        view = view_for(a["path"], articulation=True)
        before_stiffness, before_damping = dof_gains(view)
        before_effort = dof_max_forces(view)
        if "stiffness" in values or "damping" in values:
            row = dof_row(view, before_stiffness)
            if "stiffness" in values:
                row[0, selected] = values["stiffness"]
            if "damping" in values:
                damping_row = dof_row(view, before_damping)
                damping_row[0, selected] = values["damping"]
                dof_write(view, "set_dof_dampings", damping_row)
            dof_write(view, "set_dof_stiffnesses", row)
        if "max_effort" in values:
            effort_row = dof_row(view, before_effort)
            effort_row[0, selected] = values["max_effort"]
            dof_write(view, "set_dof_max_forces", effort_row)
        after_stiffness, after_damping = dof_gains(view)
        after_effort = dof_max_forces(view)
        report = [
            dict(name=names[index], index=index,
                 before=dict(stiffness=before_stiffness[index], damping=before_damping[index],
                             max_effort=before_effort[index]),
                 after=dict(stiffness=after_stiffness[index], damping=after_damping[index],
                            max_effort=after_effort[index]))
            for index in selected
        ]
        changed = any(
            item["after"]["stiffness"] != item["before"]["stiffness"]
            or item["after"]["damping"] != item["before"]["damping"]
            or item["after"]["max_effort"] != item["before"]["max_effort"]
            for item in report
        )
        return dict(
            path=a["path"],
            backend=state["backend"],
            dofs=report,
            read_only=not values,
            applied=changed,
            units="stiffness: torque per radian (force per stage unit); damping: torque per rad/s; "
                  "max_effort: torque (force for prismatic)",
            source=state["backend"] + " tensor view",
            effect="Runtime gains changed for the selected DOFs; they are not USD, are lost on Stop and "
                   "are never saved. sim__joints__drive authors USD gains instead.",
        )

    def tool_joints_move(self, name, stage, a, camera_frame=None):
        s = self.s
        if s.timeline.is_stopped() or s.timeline.is_playing():
            refuse(
                "PHYSICS_NOT_READY",
                "Pause an initialized simulation first: from a stopped timeline, sim__timeline "
                "action=play and then action=pause. Move does not initialize by playing or pumping frames.",
            )
        s.check_runtime_edit(stage)
        max_steps, tolerance = a.get("max_steps", 600), a.get("tolerance", 0.01)
        if type(max_steps) is not int or not 1 <= max_steps <= 600:
            refuse("LIMIT", "max_steps must be an integer 1..600.")
        if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or not 0 < tolerance <= 100:
            refuse("INVALID_VALUE", "tolerance must be a finite number greater than 0.")
        clamp = a.get("clamp", False)
        if type(clamp) is not bool:
            refuse("INVALID_VALUE", "clamp must be true or false.")
        state = self.joints(stage, a["path"])
        names = state["names"]
        if not names:
            refuse("JOINT_COUNT", "Selected articulation has no commandable DOFs.")
        from .physics import array as tensor_array, dof_limits, dof_set, dof_units, step, view_for

        view = view_for(a["path"], articulation=True)
        rows = resolve_named(a["targets"], names, "targets")
        limits, units = dof_limits(view), dof_units(view)
        applied, clamped, rounded = allowed_targets(
            rows, limits, state["positions"], units, clamp, "Target"
        )
        sm = self.physics_manager()
        if state["backend"] == "newton":
            self.newton_timestep(sm, stage)  # refuses, before any target is written, when Newton's two timesteps disagree
        dof_set(view, "get_dof_position_targets", "set_dof_position_targets",
                {index: row[2] for index, row in applied.items()})
        before_time, dt = sm.get_simulation_time(), sm.get_physics_dt()
        if not finite_clock(before_time) or not finite_clock(dt) or dt <= 0:
            refuse("PHYSICS_CLOCK_INVALID",
                   "Runtime physics time must be finite and the timestep positive; no steps were taken.")
        tracked = {index: (row[0], row[2]) for index, row in applied.items()}
        positions, errors, steps_used, settled = state["positions"], [], 0, False
        phase = "stepping"
        try:
            while steps_used < max_steps and not settled:
                chunk = min(10, max_steps - steps_used)
                step(stage, chunk)
                steps_used += chunk
                phase = "observation"
                positions = tensor_array(view.get_dof_positions()).reshape(-1).tolist()
                errors = [
                    dict(name=name, index=index, target=target, position=positions[index],
                         error=positions[index] - target, units=units[index])
                    for index, (name, target) in tracked.items()
                ]
                settled = all(abs(item["error"]) <= tolerance for item in errors)
            phase = "clock_observation"
            after_time = sm.get_simulation_time()
        except Exception as exc:  # noqa: BLE001 - report partial execution, never retry
            if isinstance(exc, AssetError):
                raise
            if any(token in str(exc).lower() for token in
                   ("cuda error 700", "cudaerrorillegaladdress", "illegal memory access")):
                s.device_error = str(exc)[:500]
                refuse("DEVICE_ERROR", str(exc)[:1500],
                       step=dict(steps_used=steps_used, phase=phase, backend=state["backend"]),
                       effects="GPU context may be unusable. Do not retry runtime operations; inspect, save, "
                               "Stop, then restart the disposable Kit.")
            refuse("STEP_FAILED", str(exc)[:1500],
                   step=dict(steps_used=steps_used, phase=phase, backend=state["backend"]),
                   effects="Stepping was attempted and may have changed runtime state; no rollback or retry.")
        if not finite_clock(after_time):
            refuse("PHYSICS_CLOCK_INVALID",
                   "The physics clock is not finite after stepping; inspect the state before continuing.",
                   steps_used=steps_used, joints=errors)
        if not math.isclose(after_time - before_time, steps_used * dt, rel_tol=1e-5, abs_tol=1e-7):
            refuse("STEP_TIMING_MISMATCH",
                   "Observed physics time does not match the steps taken; inspect state before retrying.",
                   steps_used=steps_used, physics_time_before=before_time, physics_time_after=after_time,
                   expected_delta=steps_used * dt, joints=errors)
        largest = max(errors, key=lambda item: abs(item["error"])) if errors else None
        return dict(
            path=a["path"],
            backend=state["backend"],
            settled=settled,
            steps_used=steps_used,
            max_steps=max_steps,
            tolerance=tolerance,
            units="rad / stage units",
            physics_time_before=before_time,
            physics_time_after=after_time,
            simulated_seconds=after_time - before_time,
            joints=errors,
            clamped=clamped,
            rounded=rounded,
            **({"blocked": dict(
                name=largest["name"], error=largest["error"], units=largest["units"],
                note="The joint did not reach its target within max_steps. A contact, a too-weak drive or "
                    "a blocked path can hold it: clear the path, raise stiffness or max_effort with "
                    "sim__joints__gains, or retry with more steps.",
            )} if largest and not settled else {}),
        )

    def tool_joints_teleport(self, name, stage, a, camera_frame=None):
        state = self.joints(stage, a["path"])
        names = state["names"]
        if not names:
            refuse("JOINT_COUNT", "Selected articulation has no commandable DOFs.")
        from .physics import array as tensor_array, dof_limits, dof_set, dof_units, view_for

        view = view_for(a["path"], articulation=True)
        rows = resolve_named(a["positions"], names, "positions")
        limits, units = dof_limits(view), dof_units(view)
        for index, name, value in rows:
            low, high = limits[index]
            if value < low - LIMIT_ROUNDING * max(1.0, abs(low)) or value > high + LIMIT_ROUNDING * max(1.0, abs(high)):
                refuse(
                    "JOINT_LIMIT",
                    f"Teleport {value:g} for {name} ({units[index]}) is outside its runtime limits "
                    f"[{low:g}, {high:g}]. Choose a value inside the limits: a state reset outside them is "
                    "unphysical.",
                    name=name, limits=[low, high], units=units[index],
                )
        before = {index: state["positions"][index] for index, _, _ in rows}
        dof_set(view, "get_dof_positions", "set_dof_positions", {index: value for index, _, value in rows})
        zeroed = True
        try:
            dof_set(view, "get_dof_velocities", "set_dof_velocities", {index: 0.0 for index, _, _ in rows})
        except Exception:  # noqa: BLE001 - state reset still succeeded
            zeroed = False
        after = tensor_array(view.get_dof_positions()).reshape(-1).tolist()
        return dict(
            path=a["path"],
            backend=state["backend"],
            joints=[
                dict(name=name, index=index, before=before[index], after=after[index], units=units[index])
                for index, name, value in rows
            ],
            velocities_zeroed=zeroed,
            physics_bypassed=True,
            warning="Joint positions were set directly: no motion, contact response or damping was simulated. "
                    "Drive targets were left unchanged, so the next steps can pull the joints back; command "
                    "or move them if the new pose must hold.",
        )

    def tool_joints_state(self, name, stage, a, camera_frame=None):
        state = self.joints(stage, a["path"])
        past = joints_past_limits(state["names"], state["positions"], runtime_dof_limits(a["path"]))
        if past:
            state["warning"] = (f"Past their limits: {', '.join(past)}. Physics is probably unstable, and drive targets "
                                "may not be reached. Check overlapping colliders, very small inertias and the timestep.")
        too_fast = joints_too_fast(state["names"], state["velocities"])
        if too_fast:
            state["warning"] = ((state.get("warning", "") + " ").lstrip() + f"Far too fast: {', '.join(too_fast)}. Physics is "
                                "unstable, often because the robot started inside a collider such as the ground.")
        return state
