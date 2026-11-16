"""The work cell on Newton (MuJoCo-Warp): the solver settings and the per-engine arm drives.

PhysX keeps the pack's authored drives (stiff, implicit springs). MuJoCo-Warp integrates a drive's stiffness
explicitly at 500 Hz, so the pack's 9400 N m per degree diverges there. On a Newton session the build
authors the ``newton`` section of the config instead: per joint, stiffness and damping in SI per radian and
an armature, written in USD's per-degree units on the stage (never the pack). The scene gets the settings
the generic tower qualified (rate, elliptic cone, solver iterations, 4 mm contact gap) plus the cell's
``impratio``, and contact buffers sized for every body in it.
"""

import math

try:  # package import inside Kit
    from . import rail as rail_axis
except ImportError:  # loaded by file path in host tests
    import importlib.util as _importlib_util
    from pathlib import Path as _Path
    _spec = _importlib_util.spec_from_file_location("workcell_rail", _Path(__file__).with_name("rail.py"))
    rail_axis = _importlib_util.module_from_spec(_spec)
    _spec.loader.exec_module(rail_axis)

DRIVE_KEYS = {"stiffness_nm_per_rad", "damping_nms_per_rad", "armature_kgm2"}
CONTACT_KEYS = {"solref", "solimp", "solmix"}
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def _number(value, name, positive=True):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'not negative'}")
    return value


def validate(config):
    """The ``newton`` section, checked before any stage is touched. A joint left out would keep the pack's PhysX gain,
    which diverges on MuJoCo-Warp, so all six arm joints are required."""
    newton = config.get("newton")
    if not isinstance(newton, dict) or not isinstance(newton.get("drives"), dict):
        raise ValueError("config has no newton.drives object: the work cell's Newton gains are per engine and must be configured")
    _number(newton.get("physics_hz"), "newton.physics_hz")
    _number(newton.get("impratio"), "newton.impratio")
    drives = newton["drives"]
    if set(drives) != set(ARM_JOINTS):
        raise ValueError(f"newton.drives must name exactly the arm joints {list(ARM_JOINTS)}")
    for joint, values in drives.items():
        if not isinstance(values, dict) or set(values) != DRIVE_KEYS:
            raise ValueError(f"newton.drives.{joint} needs exactly {sorted(DRIVE_KEYS)}")
        _number(values["stiffness_nm_per_rad"], f"newton.drives.{joint}.stiffness_nm_per_rad")
        _number(values["damping_nms_per_rad"], f"newton.drives.{joint}.damping_nms_per_rad", positive=False)
        _number(values["armature_kgm2"], f"newton.drives.{joint}.armature_kgm2")
    fingertip_contact(config)
    return drives


def fingertip_contact(config):
    """``fingertips.newton_contact``, checked: MuJoCo's contact softness for the fingertip colliders, or None when the
    config has none (the solver's defaults then apply). ``solref`` is (time constant in seconds, damping ratio); the
    time constant may not be under two steps, where the solver would clamp it. ``solimp`` is (impedance at first
    touch, impedance at full depth, depth of that change in metres, midpoint, power); ``solmix`` weighs these values
    against the other body's."""
    section = config.get("fingertips")
    values = section.get("newton_contact") if isinstance(section, dict) else None
    if values is None:
        return None
    if not isinstance(values, dict) or set(values) != CONTACT_KEYS:
        raise ValueError(f"fingertips.newton_contact needs exactly {sorted(CONTACT_KEYS)}")
    solref, solimp = values["solref"], values["solimp"]
    if not isinstance(solref, (list, tuple)) or len(solref) != 2 or not isinstance(solimp, (list, tuple)) or len(solimp) != 5:
        raise ValueError("fingertips.newton_contact: solref has 2 numbers and solimp has 5")
    for index, value in enumerate([*solref, *solimp]):
        _number(value, f"fingertips.newton_contact value {index}")
    _number(values["solmix"], "fingertips.newton_contact.solmix")
    least = 2.0 / _number(config["newton"]["physics_hz"], "newton.physics_hz")
    if solref[0] < least - 1e-12:
        raise ValueError(f"fingertips.newton_contact.solref's time constant {solref[0]} s is under two steps ({least:g} s)")
    if not 0 < solimp[0] <= solimp[1] < 1 or not 0 < solimp[3] < 1 or solimp[4] < 1:
        raise ValueError("fingertips.newton_contact.solimp needs 0 < dmin <= dmax < 1, a midpoint between 0 and 1 and a power of at least 1")
    return {"solref": [float(v) for v in solref], "solimp": [float(v) for v in solimp], "solmix": float(values["solmix"])}


def stability(drives, rate_hz):
    """``dt^2 k / I`` per joint with ``I`` the armature alone (a lower bound on what the joint carries), against the
    explicit-spring limit ``4 + 2 dt c / I``. Below 1 leaves a margin of four."""
    dt = 1.0 / rate_hz
    out = {}
    for joint, d in drives.items():
        inertia = d["armature_kgm2"]
        out[joint] = dict(ratio=round(dt * dt * d["stiffness_nm_per_rad"] / inertia, 4),
                          limit=round(4 + 2 * dt * d["damping_nms_per_rad"] / inertia, 4))
    return out


def prepare(stage, config, assembly_path, timeline):
    """Author the Newton scene settings, every collider's contact gap and the arm's Newton drives; size the
    contact buffers. Returns what was authored. Raises ValueError for a joint the config names and the
    assembly lacks."""
    from pxr import Sdf, Usd, UsdPhysics
    from sim_tools.newton_support import apply_buffers, collision_settings, prepare_buffers, scene_settings

    drives = validate(config)
    newton, pads = config["newton"], fingertip_contact(config)
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    settings = scene_settings(scene, newton["physics_hz"], tower=True)
    impratio = scene.GetAttribute("mjc:option:impratio")
    if not impratio or not impratio.Set(float(newton["impratio"])):
        raise ValueError("the Newton scene has no mjc:option:impratio; the work cell needs the MuJoCo-Warp solver")
    settings["mjc:option:impratio"] = float(newton["impratio"])
    newton_mu = None
    if config.get("contact", {}).get("newton_friction") == "static":
        newton_mu = 0
        for prim in stage.Traverse():   # Newton reads only dynamicFriction; give it the static value
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                api = UsdPhysics.MaterialAPI(prim)
                static = api.GetStaticFrictionAttr()
                if static and static.HasAuthoredValue():   # unauthored reads the default 0.0: keep dynamic then
                    api.CreateDynamicFrictionAttr().Set(float(static.Get()))
                    newton_mu += 1
    colliders, stiff_pads = 0, []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_settings(prim)
            colliders += 1
            # The gripper's own fingertips (scene.fingertip_friction finds them the same way): how far they give way.
            if pads and prim.GetName() == "fingertip" and str(prim.GetPath()).startswith(str(assembly_path) + "/"):
                prim.CreateAttribute("mjc:solref", Sdf.ValueTypeNames.DoubleArray).Set(pads["solref"])
                prim.CreateAttribute("mjc:solimp", Sdf.ValueTypeNames.DoubleArray).Set(pads["solimp"])
                prim.CreateAttribute("mjc:solmix", Sdf.ValueTypeNames.Double).Set(pads["solmix"])
                stiff_pads.append(str(prim.GetPath()))
    if pads and len(stiff_pads) != 2:
        raise ValueError(f"fingertips.newton_contact is configured, but the assembly has {len(stiff_pads)} fingertip colliders, not 2")
    joints = {prim.GetName(): prim for prim in Usd.PrimRange(stage.GetPrimAtPath(assembly_path)) if prim.IsA(UsdPhysics.Joint)}
    authored = {}
    for name, d in drives.items():
        prim = joints.get(name)
        if prim is None or "PhysicsDriveAPI:angular" not in prim.GetAppliedSchemas():
            raise ValueError(f"newton.drives names {name}, which is not a driven joint of the assembly")
        per_degree = math.pi / 180.0
        drive = UsdPhysics.DriveAPI(prim, "angular")
        drive.GetStiffnessAttr().Set(d["stiffness_nm_per_rad"] * per_degree)
        drive.GetDampingAttr().Set(d["damping_nms_per_rad"] * per_degree)
        for attr in ("newton:armature", "physxJoint:armature"):   # both, so no resolver default decides it
            prim.CreateAttribute(attr, Sdf.ValueTypeNames.Float).Set(float(d["armature_kgm2"]))
        authored[name] = dict(d)
    margins, rail_drive = stability(drives, newton["physics_hz"]), None
    if rail_axis.moving(config):                                # the robot's base is a seventh axis: its drive, per engine
        record = rail_axis.check(config)
        name, d = record["joint_name"], record["drive"]["newton"]
        prim = joints.get(name)
        if prim is None or "PhysicsDriveAPI:linear" not in prim.GetAppliedSchemas():
            raise ValueError(f"rail.joint_name names {name}, which is not a driven prismatic joint of the assembly")
        drive = UsdPhysics.DriveAPI(prim, "linear")             # a linear drive is per stage unit: metres, no conversion
        drive.GetStiffnessAttr().Set(d["stiffness_n_per_m"])
        drive.GetDampingAttr().Set(d["damping_ns_per_m"])
        for attr in ("newton:armature", "physxJoint:armature"):
            prim.CreateAttribute(attr, Sdf.ValueTypeNames.Float).Set(float(d["armature_kg"]))
        rail_drive = dict(d, joint=name, stability=rail_axis.stability(record, newton["physics_hz"]))
    buffers_config, buffers = prepare_buffers(stage, scene, timeline)
    # The wrist force/torque reading is a mount joint's incoming reaction; Newton computes it only when asked before Play.
    buffers_config.solver_outputs.link_incoming_joint_force = True
    apply_buffers(buffers_config)
    buffers["link_incoming_joint_force"] = True
    return dict(settings=settings, colliders_with_contact_gap=colliders, drives=authored, buffers=buffers,
                materials_with_static_mu=newton_mu,
                **({"fingertip_contact": dict(pads, colliders=stiff_pads)} if pads else {}),
                stability=margins, **({"rail_drive": rail_drive} if rail_drive else {}))
