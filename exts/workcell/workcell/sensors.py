"""Read live tensor samples (PhysX or Newton); never manufacture missing measurements.

Call after a completed physics step, with its observed simulation timestamp.
These functions do not initialize, step, publish ROS, subtract gravity/bias or
claim hardware calibration. Sampling cadence is owned by the caller; rate_hz
is a requested cadence, not evidence of a delivered sensor rate.
"""

import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def _time(seconds):
    if isinstance(seconds, bool) or not math.isfinite(seconds) or seconds < 0:
        raise ValueError("physics_time_s must be observed nonnegative simulation time")
    return float(seconds)


def _unavailable(reason, status="unavailable"):
    return {"status": status, "reason": reason, "measurement": None}


ENGINES = ("physx", "newton")


def _numpy(values):
    """The view's array as NumPy, whatever frontend this build returns (warp on Isaac Sim 6.x, torch, NumPy)."""
    if hasattr(values, "detach"):
        values = values.detach().cpu()
    if hasattr(values, "numpy"):
        values = values.numpy()
    return np.asarray(values, dtype=float)


def _view_ready(view, backend, engines=ENGINES):
    if backend not in engines:
        return _unavailable(f"This sensor reads on {', '.join(engines)}, not {backend!r}", "unsupported")
    try:
        ready = view is not None and view.check()
    except Exception:
        ready = False
    if not ready:
        return _unavailable("Initialize a live tensor view before sampling")
    return None


FLANGE_SOURCE, BOTA_SOURCE = "ur5e_tool_flange_sensor", "bota_rokubi"


def read_wrench(stage, view, report, config, *, physics_time_s, backend, source=None):
    """The force/torque reading of the cell's wrist sensor: a fixed joint's reaction, in the sensor's frame.

    ``source`` picks the sensor. ``ur5e_tool_flange_sensor`` (the default when no Bota is mounted) is the UR5e's own
    sensor in its tool flange: the reaction at the joint between wrist 3 and the first part bolted to the flange, at
    the centre of the flange plane, in tool0's axes (frame ``flange_ft_sensor.frame_id``). It carries everything
    below the flange. ``bota_rokubi`` (the default when ``bota.mounted`` is true) is the reaction at the Bota's
    tool-side face, in the Bota's sensor frame (``bota.frame_id``); it carries everything below the sensor.

    view is a native tensor ArticulationView for report.articulation_root (any frontend), not the Isaac Core
    wrapper. On Newton the solver must compute link incoming joint forces (newton_cell.prepare turns that on before
    Play). The measured joint must remain in the articulation (no fixed-link merge). Values are
    [Fx,Fy,Fz,Tx,Ty,Tz], N and N m, exerted on the carried side by the robot side, torque about the frame's origin.
    This is a rigid-body joint reaction with gravity and inertial effects in it: raw, not payload-compensated, with no
    noise, drift or accuracy limit modelled, and not a calibrated hardware output.
    """
    bota = report.get("bota")
    if source is None:
        source = BOTA_SOURCE if bota else FLANGE_SOURCE
    if source == FLANGE_SOURCE:
        section, joint_path, frame_path = config.get("flange_ft_sensor", {}), report.get("flange_joint_path"), report.get("tool0_path")
        label = "UR5e tool flange sensor"
    elif source == BOTA_SOURCE:
        if not bota:
            return _unavailable("No Bota sensor is mounted (bota.mounted is false); read the tool flange sensor", "unsupported")
        section, joint_path, frame_path, label = config.get("bota", {}), bota["joint_path"], bota["sensor_path"], "Bota"
    else:
        return _unavailable(f"Unknown wrench source {source!r}", "unsupported")
    if section.get("measurement") != "incoming_joint_wrench":
        return _unavailable(f"{label} incoming_joint_wrench measurement is not configured", "unsupported")
    invalid = _view_ready(view, backend)
    if invalid:
        return invalid
    timestamp = _time(physics_time_s)
    try:
        if view.count != 1 or list(view.prim_paths) != [report["articulation_root"]]:
            return _unavailable("Tensor view must target exactly the reported arm articulation")
        frame = stage.GetPrimAtPath(frame_path) if frame_path else None
        joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(joint_path)) if joint_path else None
        targets = joint.GetBody1Rel().GetTargets() if joint else []
        body = stage.GetPrimAtPath(targets[0]) if len(targets) == 1 else None
        if not body or not frame or joint.GetExcludeFromArticulationAttr().Get():
            return _unavailable(f"{label} joint/body/frame topology is missing or changed")
        bodies = [prim for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"]))
                  if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() == body.GetName()]
        names = view.shared_metatype.link_indices
        if len(bodies) != 1 or body.GetName() not in names:
            return _unavailable(f"The link below the {label} is merged, absent or ambiguous in the runtime articulation")
        cache = UsdGeom.XformCache()                         # the frame and the body are fixed to each other
        frame_in_body = cache.GetLocalToWorldTransform(frame) * cache.GetLocalToWorldTransform(body).GetInverse()
        joint_in_body = Gf.Matrix4d().SetRotate(Gf.Quatd(joint.GetLocalRot1Attr().Get()))
        joint_in_body.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
        to_sensor = joint_in_body * frame_in_body.GetInverse()
        values = _numpy(view.get_link_incoming_joint_force())
        if values.size == view.max_links * 6:
            values = values.reshape(1, view.max_links, 6)
        if values.shape != (1, view.max_links, 6) or not np.isfinite(values).all():
            return _unavailable("PHYSICS_STATE_INVALID: invalid incoming joint wrench array")
        wrench = values[0, names[body.GetName()]]
        force = to_sensor.TransformDir(Gf.Vec3d(*map(float, wrench[:3])))
        torque = to_sensor.TransformDir(Gf.Vec3d(*map(float, wrench[3:])))
        torque += Gf.Cross(to_sensor.ExtractTranslation(), force)
        measurement = [float(value) for value in (*force, *torque)]
        if not all(math.isfinite(value) for value in measurement):
            return _unavailable("PHYSICS_STATE_INVALID: nonfinite transformed wrench")
        return {"status": "measured", "source": source, "measurement": measurement,
                "force_n": measurement[:3], "torque_nm": measurement[3:],
                "physics_time_s": timestamp, "backend": backend,
                "frame_id": section["frame_id"], "frame_path": str(frame.GetPath()),
                "source_joint": str(joint.GetPath()), "load_body_path": str(body.GetPath()),
                "quantity": "parent_on_child_joint_reaction",
                "gravity_compensated": False, "payload_compensated": False, "noise_modelled": False,
                "hardware_calibrated": False}
    except Exception as exc:
        return _unavailable(f"{label} tensor sampling unavailable: {exc}")


def read_finger_contacts(view, config, *, physics_time_s, physics_dt_s, backend):
    """Read net world-frame finger contact forces, explicitly NOT TAL221 readings.

    Set finger_force_sensors.measurement='net_contact_force' and body_paths to
    the two exact rigid finger bodies. A caller-created native RigidContactView
    must expose those sensor_paths in the same order and enable contact reports.
    Net forces include all contacts on each body, not only its black pad; they
    are neither individual-pad pressure nor a standard RG2 built-in sensor.
    fitted=True alone never enables measurements or implies purchased hardware.
    """
    settings = config.get("finger_force_sensors", {})
    if settings.get("measurement") != "net_contact_force":
        return _unavailable("Finger contact measurement is not configured; no load-cell readings", "unsupported")
    invalid = _view_ready(view, backend, engines=("physx",))   # Newton's contact snapshot is not wired yet
    if invalid:
        return invalid
    timestamp = _time(physics_time_s)
    if isinstance(physics_dt_s, bool) or not math.isfinite(physics_dt_s) or physics_dt_s <= 0:
        raise ValueError("physics_dt_s must be the positive simulated contact-report interval")
    try:
        paths = settings.get("body_paths", [])
        if len(paths) != 2 or len(set(paths)) != 2 or list(view.sensor_paths) != paths:
            return _unavailable("Contact view must target exactly the two configured finger bodies")
        values = _numpy(view.get_net_contact_forces(dt=physics_dt_s))
        if values.shape != (2, 3) or not np.isfinite(values).all():
            return _unavailable("PHYSICS_STATE_INVALID: invalid net contact force array")
        return {"status": "measured", "measurement": values.tolist(), "force_n": values.tolist(),
                "body_paths": list(paths), "frame_id": "world", "physics_time_s": timestamp,
                "physics_dt_s": float(physics_dt_s), "backend": backend,
                "quantity": "whole_finger_net_contact_force", "hardware_calibrated": False,
                "limitations": ["Not a TAL221/load-cell measurement or pad-only normal force."]}
    except Exception as exc:
        return _unavailable(f"Finger contact sampling unavailable: {exc}")
