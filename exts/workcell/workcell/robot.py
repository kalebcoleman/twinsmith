"""Pack-only workcell assembly and single-articulation NumPy tensor commands.

build_robot requires a stopped stage in metres, kilograms and Z-up. Each
config.asset_layout role declares model, body, mount_frame, bom_id and fidelity
(placeholder/unverified/verified). Paths are relative to the asset default prim;
"." names that prim. Robot additionally declares articulation_root and
flange_frame; RG2 may declare articulation_root for a separate articulation.
Verified fidelity requires a provenance string and remains a pack declaration.
An optional mount_pose {position_m, orientation_wxyz}, relative to mount_frame,
defines a virtual inlet frame without changing the asset. Each inlet is aligned
with the flange frame; downstream offsets use stack.axis_in_flange (default +X).

The stack follows one switch, bota.mounted (see stack.py). Off: flange, Quick
Changer, RG2, and no Bota prim is built. On: an adapter plate, the Bota and a
second adapter plate come first; the plates are labelled placeholder discs made
here from bota.robot_side_plate and bota.tool_side_plate, not pack assets. What
the RG2 body carries without a body of its own (the printed camera bracket and
the D415) is added to that body's mass properties.

base_plate is static pack geometry (no body/mount_frame layout required), placed
by base_plate.placement {position_m, orientation_wxyz} relative to the assembly.
This pose is required: plate thickness and robot base elevation are not inferred.
With base_plate.present false no plate is built: the robot stands on the plates
the bench asset already holds, at robot.placement.

With rail.moving true the robot's base is a seventh axis (rail.py): the pack's
rail_carriage asset (the bench's own carriage and robot plate) becomes a rigid
body under the robot's base, a geometry-free rail link fixed to the world is the
articulation's root, a prismatic joint named rail.joint_name joins the two, and
a fixed joint holds the robot's base on the carriage. The arm's root joint keeps
its prim path. Joint tools see one more degree of freedom, first in order.
No simulator lifecycle, ROS publishers, markers or asset-cache lookup lives here.
"""

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import importlib.util as _importlib_util

try:  # package import inside Kit
    from . import stack as tool_stack
except ImportError:  # loaded by file path in host tests
    _spec = _importlib_util.spec_from_file_location("workcell_stack", Path(__file__).with_name("stack.py"))
    tool_stack = _importlib_util.module_from_spec(_spec)
    _spec.loader.exec_module(tool_stack)
try:  # package import inside Kit
    from . import rail as rail_axis
except ImportError:  # loaded by file path in host tests
    _spec = _importlib_util.spec_from_file_location("workcell_rail", Path(__file__).with_name("rail.py"))
    rail_axis = _importlib_util.module_from_spec(_spec)
    _spec.loader.exec_module(rail_axis)


MODELS = {"robot": "ur5e", "bota": "bota_rokubi", "rail_carriage": "vention_rail_carriage",
          "quick_changer": "onrobot_quick_changer", "rg2": "onrobot_rg2",
          "finger_pad": "finger_pad", "base_plate": "base_plate"}


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result.tolist()


def _positive(value, name):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _pose(position, orientation=(1, 0, 0, 0)):
    position = _vector(position, 3, "position_m")
    orientation = _vector(orientation, 4, "orientation_wxyz")
    if not math.isclose(sum(value * value for value in orientation), 1, abs_tol=1e-6):
        raise ValueError("orientation_wxyz must be a unit quaternion")
    matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(*orientation))
    matrix.SetTranslateOnly(Gf.Vec3d(*position))
    return matrix


def _rigid(matrix):
    values = np.array(matrix)
    rotation = values[:3, :3]
    if (not np.isfinite(values).all()
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)
            or not np.allclose(values[:, 3], [0, 0, 0, 1], atol=1e-6)):
        raise ValueError("Mount/body transforms must be rigid, without scale or reflection")
    return matrix


def _units(stage):
    if (UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z
            or not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1)
            or not math.isclose(UsdPhysics.GetStageKilogramsPerUnit(stage), 1)):
        raise ValueError("Stage and pack assets must declare metre, kilogram, Z-up units")


def _relative(stage, root, path):
    relative = Sdf.Path(path)
    if (not relative.IsPrimPath() or relative.IsAbsolutePath()
            or ".." in str(relative).split("/")):
        raise ValueError(f"Expected a relative prim path, got {path!r}")
    prim = stage.GetPrimAtPath(root if path == "." else root.AppendPath(relative))
    if not prim:
        raise ValueError(f"Missing pack prim: {root}/{path}")
    return prim


def _world(prim):
    if not UsdGeom.Xformable(prim):
        raise ValueError(f"Expected a transformable frame: {prim.GetPath()}")
    return _rigid(UsdGeom.XformCache().GetLocalToWorldTransform(prim))


def _set_world(prim, matrix):
    parent = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
    xform = UsdGeom.Xformable(prim)
    xform.MakeMatrixXform().Set(_rigid(matrix * parent.GetInverse()))
    xform.SetResetXformStack(False)


def _mount_world(prim, layout):
    pose = layout.get("mount_pose", {})
    return _pose(pose.get("position_m", [0, 0, 0]),
                 pose.get("orientation_wxyz", [1, 0, 0, 0])) * _world(prim)


def _neutralize_frame_bodies(stage, root, keep):
    """Turn geometry-free rigid bodies held only by fixed joints into plain frames.

    A body without collision shapes has no valid mass or inertia; PhysX reports it and
    falls back to unit inertia. Such a pack body (the RG2 ``gripper_center`` tool frame)
    keeps its transform but loses RigidBodyAPI/MassAPI, and the fixed joint that only
    attached it is deactivated. The declared role body (``keep``) is never touched. Edits
    stay in the session layer; the pack is untouched.
    """
    frames = []
    for prim in list(Usd.PrimRange(root)):
        if prim.GetPath() == keep or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if any(item.IsA(UsdGeom.Gprim)
               for item in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())):
            continue
        joints = [item for item in Usd.PrimRange(root) if item.IsA(UsdPhysics.Joint)
                  and prim.GetPath() in (UsdPhysics.Joint(item).GetBody0Rel().GetTargets()
                                         + UsdPhysics.Joint(item).GetBody1Rel().GetTargets())]
        if not joints or any(not item.IsA(UsdPhysics.FixedJoint) for item in joints):
            continue  # a moving link or an unattached body is not a frame marker
        for item in joints:
            item.SetActive(False)
        for schema in (UsdPhysics.MassAPI, UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(schema)
        for name in prim.GetAppliedSchemas():
            if name.startswith("Physx") and "RigidBody" in name:
                prim.RemoveAppliedSchema(name)
        frames.append(str(prim.GetPath()))
    return frames


MAX_TGS_VELOCITY_ITERATIONS = 4


def _clamp_velocity_iterations(root):
    """Cap pack-authored articulation velocity iterations at the TGS-supported 4.

    The RG2 asset authors 16; PhysX warns about more than 4 in a TGS scene. Position
    iterations are left as authored. Session layer only.
    """
    clamped = []
    for prim in Usd.PrimRange(root):
        attribute = prim.GetAttribute("physxArticulation:solverVelocityIterationCount")
        if attribute and (attribute.Get() or 0) > MAX_TGS_VELOCITY_ITERATIONS:
            attribute.Set(MAX_TGS_VELOCITY_ITERATIONS)
            clamped.append(str(prim.GetPath()))
    return clamped


def _deactivate_dangling_joints(stage, root):
    """Deactivate disabled pack joints that name no second body.

    The UR5e asset ships a disabled ``robot_gripper_joint`` whose world-frame anchor does
    not coincide with the wrist, so PhysX warns about disjointed body transforms. The
    work cell mounts its own tool, so the leftover joint is dropped. Enabled joints with a
    missing body (the base ``root_joint``) are anchors and stay.
    """
    removed = []
    for prim in list(Usd.PrimRange(root)):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        dangling = not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets()
        if dangling and joint.GetJointEnabledAttr().Get() is False:
            prim.SetActive(False)
            removed.append(str(prim.GetPath()))
    return removed


def _fixed(stage, path, parent, child, world_frame, separate):
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    for side, body in enumerate((parent, child)):
        local = _rigid(world_frame * _world(body).GetInverse())
        getattr(joint, f"CreateBody{side}Rel")().SetTargets([body.GetPath()])
        getattr(joint, f"CreateLocalPos{side}Attr")().Set(Gf.Vec3f(local.ExtractTranslation()))
        getattr(joint, f"CreateLocalRot{side}Attr")().Set(Gf.Quatf(local.ExtractRotationQuat()))
    joint.CreateExcludeFromArticulationAttr().Set(separate)
    joint.CreateCollisionEnabledAttr().Set(False)
    return joint


def _source(assets, config, role):
    asset_id = config["assets"][role]
    layout = config["asset_layout"][role]
    if layout["model"] != MODELS[role]:
        raise ValueError(f"{role} requires model {MODELS[role]}, not a substitute")
    path = Path(assets.asset(asset_id))
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f"Pack asset {asset_id!r} is not an existing absolute file")
    if role == "rg2" and any(token in (asset_id + str(path)).lower()
                            for token in ("husarion", "rg2-ft", "rg2_ft")):
        raise ValueError("Husarion/RG2-FT is not the standard OnRobot RG2")
    source = Usd.Stage.Open(str(path))
    if source is None or not source.GetDefaultPrim():
        raise ValueError(f"Pack asset {asset_id!r} needs a USD default prim")
    _units(source)
    fidelity = layout.get("fidelity", "unverified")
    if fidelity not in ("placeholder", "unverified", "verified"):
        raise ValueError(f"Invalid fidelity for {role}")
    if fidelity == "verified" and not layout.get("provenance"):
        raise ValueError(f"Verified {role} requires provenance")
    if not isinstance(layout["bom_id"], str) or not layout["bom_id"]:
        raise ValueError(f"{role} needs a nonempty bom_id")
    return path, layout, asset_id


def base_plate_present(config):
    """Whether the separate base plate is built. Off when the bench asset already holds the real mounting plates."""
    value = (config.get("base_plate") or {}).get("present", True)
    if type(value) is not bool:
        raise ValueError("base_plate.present must be true or false")
    return value


def build_robot(stage, assets, config):
    """Assemble referenced parts transactionally; raise ValueError for bad layouts.

    Successful edits live in a new strongest session sublayer (returned as layer_id).
    Export/flatten the stage to persist them; saving only the root layer omits them.
    Source assets and unrelated stage opinions are never written. Callers own
    stopped-timeline enforcement, initialization, tensor view creation and sampling.
    """
    _units(stage)
    order = tool_stack.order(config)
    if not math.isclose(config["rg2"]["max_opening_m"], 0.110, abs_tol=1e-9):
        raise ValueError("Standard RG2 max_opening_m must be 0.110")
    rail = rail_axis.check(config) if rail_axis.moving(config) else None
    axis, lengths = tool_stack.axis(config), tool_stack.lengths(config)
    tool_stack.masses(config)                                            # refuses a bad or missing mass before any edit
    record = tool_stack.tool(config)
    root = Sdf.Path(config["robot"].get("assembly_path", "/World/RobotAssembly"))
    if not root.IsAbsolutePath() or not root.IsPrimPath() or stage.GetPrimAtPath(root):
        raise ValueError("robot.assembly_path must be a new absolute prim path")
    if root.GetParentPath() != Sdf.Path.absoluteRootPath and not stage.GetPrimAtPath(root.GetParentPath()):
        raise ValueError("Assembly parent must already exist")
    pads = config.get("finger_pads", {}).get("present", False)
    sources = {role: _source(assets, config, role)
               for role in ["robot", *(["base_plate"] if base_plate_present(config) else []),
                            *(["rail_carriage"] if config.get("rail") is not None else []),
                            *(role for role in order if role not in tool_stack.PLATES),
                            *(["finger_pad"] if pads else [])]}
    layer = Sdf.Layer.CreateAnonymous("workcell-robot.usda")
    session = stage.GetSessionLayer()
    session.subLayerPaths.insert(0, layer.identifier)
    try:
        with Usd.EditContext(stage, layer):
            report = _assemble(stage, root, sources, config, order, axis, lengths, pads, rail)
        report["layer_id"] = layer.identifier
        report["tool"] = record
        return report
    except Exception:
        session.subLayerPaths.remove(layer.identifier)
        raise


def _assemble(stage, root, sources, config, order, axis, lengths, pads, rail=None):
    assembly = UsdGeom.Xform.Define(stage, root).GetPrim()
    placement = config["robot"].get("placement", {})
    _set_world(assembly, _pose(placement.get("position_m", [0, 0, 0]),
                              placement.get("orientation_wxyz", [1, 0, 0, 0])))
    parts, roots, bodies, mounts = [], {}, {}, {}
    frame_bodies, dropped_joints, clamped = [], [], []
    limitations = ["Pack fidelity is declared, not independently verified; no runtime validation performed.",
                   "Pack collision geometry, inertia, joint limits and drives are preserved; no synthetic drives.",
                   "Closed-chain RG2 dynamics and separate-articulation mounting require PhysX validation.",
                   "Initial joint angles are not applied; command named joints after physics initialization.",
                   "Width calibration, pad geometry and mount poses need independent physical verification.",
                   "The force/torque read is a joint reaction of the rigid-body model: no sensor noise, drift, "
                   "accuracy limit or internal deformation is modelled.",
                   "Config dimensions, masses and sensor origins retain their source uncertainty."]

    def reference(role, path):
        source, layout, asset_id = sources[role]
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        if not prim.GetReferences().AddReference(str(source)):
            raise ValueError(f"Could not reference pack role {role}")
        if prim.IsInstance():
            raise ValueError(f"{role} root must be editable, not instanceable")
        fidelity = layout.get("fidelity", "unverified")
        for key, value in {"asset_id": asset_id, "bom_id": layout["bom_id"],
                           "model": layout["model"], "fidelity": fidelity}.items():
            prim.SetCustomDataByKey(f"workcell:{key}", value)
        parts.append({"prim_path": str(path), "asset_id": asset_id,
                      "bom_id": layout["bom_id"], "fidelity": fidelity})
        if fidelity != "verified":
            limitations.append(f"{role}: {fidelity} geometry; not a faithful CAD/dynamics claim.")
        return prim

    plate = None
    if base_plate_present(config):
        plate = reference("base_plate", root.AppendChild("base_plate"))
        for prim in Usd.PrimRange(plate):
            if (prim.HasAPI(UsdPhysics.RigidBodyAPI)
                    or prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.IsA(UsdPhysics.Joint)):
                raise ValueError("base_plate must be static geometry without bodies, joints or articulations")
            if UsdGeom.Xformable(prim):
                if UsdGeom.Xformable(prim).GetResetXformStack():
                    raise ValueError("base_plate transforms cannot reset the asset transform stack")
        plate_pose = config["base_plate"]["placement"]
        _set_world(plate, _pose(plate_pose["position_m"], plate_pose["orientation_wxyz"]) * _world(assembly))
        limitations.append("Base plate is static pack geometry; verify its explicit pose and robot elevation independently.")
    else:
        limitations.append("No separate base plate is built (base_plate.present is false): the robot stands on the "
                           "mounting plates of the bench asset, at robot.placement; that height is the bench CAD's, "
                           "not a lab measurement.")

    proxies = []
    for role in ["robot", *order]:
        if role in tool_stack.PLATES:
            roots[role] = bodies[role] = mounts[role] = _plate(stage, root.AppendChild(role), tool_stack.plate(config, role))
            proxies.append({"prim_path": str(roots[role].GetPath()), "what": "adapter plate beside the Bota sensor",
                            "shape": "placeholder disc from bota." + tool_stack.PLATES[role] + "; not measured"})
            limitations.append(f"{role}: placeholder disc; measure the real adapter plate (thickness, diameter, mass).")
            continue
        roots[role] = reference(role, root.AppendChild(role))
        if role == "robot":
            dropped_joints += _deactivate_dangling_joints(stage, roots[role])
        else:
            declared = sources[role][1]["body"]
            clamped += _clamp_velocity_iterations(roots[role])
            frame_bodies += _neutralize_frame_bodies(
                stage, roots[role],
                roots[role].GetPath() if declared == "." else roots[role].GetPath().AppendPath(declared))
        layout = sources[role][1]
        bodies[role] = _relative(stage, roots[role].GetPath(), layout["body"])
        mounts[role] = _relative(stage, roots[role].GetPath(), layout["mount_frame"])
        if not bodies[role].HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"{role}.body must have RigidBodyAPI")
        if not UsdPhysics.RigidBodyAPI(bodies[role]).GetRigidBodyEnabledAttr().Get():
            raise ValueError(f"{role}.body must be enabled")
        _world(bodies[role])
        if role != "robot":
            prims = list(Usd.PrimRange(roots[role]))
            for prim in prims:
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    body_api = UsdPhysics.RigidBodyAPI(prim)
                    if (not body_api.GetRigidBodyEnabledAttr().Get()
                            or body_api.GetKinematicEnabledAttr().Get()):
                        raise ValueError(f"{role} links must be enabled dynamic rigid bodies")
                    _world(prim)
                    ancestor = prim.GetParent()
                    while ancestor and ancestor.GetPath().HasPrefix(roots[role].GetPath()):
                        if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                            raise ValueError(f"{role} cannot nest rigid bodies")
                        ancestor = ancestor.GetParent()
                if UsdGeom.Xformable(prim) and UsdGeom.Xformable(prim).GetResetXformStack():
                    raise ValueError(f"{role} transforms cannot reset the asset transform stack")
            if any(prim.IsA(UsdPhysics.Joint) and
                   (not UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()
                    or not UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()) for prim in prims):
                raise ValueError(f"{role} cannot contain a joint fixed to the world")
            for prim in prims:
                if prim.IsA(UsdPhysics.Joint):
                    joint = UsdPhysics.Joint(prim)
                    for targets in (joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()):
                        if (len(targets) != 1 or not targets[0].HasPrefix(roots[role].GetPath())
                                or not stage.GetPrimAtPath(targets[0]).HasAPI(UsdPhysics.RigidBodyAPI)):
                            raise ValueError(f"{role} joints must target internal rigid bodies")
            if role != "rg2" and (sum(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in prims) != 1
                                  or any(prim.IsA(UsdPhysics.Joint)
                                         or prim.HasAPI(UsdPhysics.ArticulationRootAPI) for prim in prims)):
                raise ValueError(f"{role} must be one rigid adapter, without internal joints")

    robot_layout = sources["robot"][1]
    arm = _relative(stage, roots["robot"].GetPath(), robot_layout["articulation_root"])
    if not arm.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise ValueError("robot.articulation_root needs ArticulationRootAPI")
    flange = _relative(stage, roots["robot"].GetPath(), robot_layout["flange_frame"])
    if not flange.GetPath().HasPrefix(bodies["robot"].GetPath()):
        raise ValueError("Robot flange must belong to the declared wrist body")
    tool0_rel = robot_layout.get("tool0_frame")
    if tool0_rel:
        tool0 = _relative(stage, roots["robot"].GetPath(), tool0_rel)
        if not tool0.GetPath().HasPrefix(bodies["robot"].GetPath()):
            raise ValueError("tool0 must belong to the wrist body")
    else:
        if stage.GetPrimAtPath(flange.GetPath().AppendChild("tool0")):
            raise ValueError("Declare the existing tool0_frame rather than overwriting it")
        tool0 = UsdGeom.Xform.Define(stage, flange.GetPath().AppendChild("tool0")).GetPrim()
        _set_world(tool0, _pose([0, 0, 0], [.5, .5, .5, .5]) * _world(flange))
        limitations.append("tool0 synthesized using UR flange +X / tool0 +Z convention; verify pack frames.")

    rail_report = _rail(stage, root, roots["robot"], arm, mounts["robot"], rail, reference, limitations) if rail else None
    if rail is None and "rail_carriage" in sources:                      # a fixed base: the carriage is scenery under it
        part = reference("rail_carriage", root.AppendChild("rail"))
        _set_world(part, Gf.Matrix4d().SetTranslate(_world(mounts["robot"]).ExtractTranslation()))
        limitations.append("The robot's base is fixed (rail.moving is false): the rail's carriage is static geometry.")
    offset, previous, mount_joints, masses = 0.0, bodies["robot"], {}, tool_stack.masses(config)
    UsdGeom.Xform.Define(stage, root.AppendChild("joints"))
    gripper_articulation = None
    for role in order:
        frame = _pose((axis * offset).tolist()) * _world(flange)
        layout = sources[role][1] if role in sources else {}
        relative = _mount_world(mounts[role], layout) * _world(roots[role]).GetInverse()
        _set_world(roots[role], relative.GetInverse() * frame)
        separate = role == "rg2" and bool(layout.get("articulation_root"))
        declared_roots = [prim for prim in Usd.PrimRange(roots[role])
                          if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if separate:
            gripper_articulation = _relative(stage, roots[role].GetPath(), layout["articulation_root"])
        if declared_roots != ([gripper_articulation] if separate else []):
            raise ValueError(f"{role} articulation roots do not match the declared layout")
        joint = _fixed(stage, root.AppendPath(f"joints/{role}_mount"), previous,
                       bodies[role], frame, separate)
        mount_joints[role] = joint
        if role in ("bota", "quick_changer"):
            mass = masses[role]
            api = UsdPhysics.MassAPI.Apply(bodies[role])
            before, inertia = api.GetMassAttr().Get(), api.GetDiagonalInertiaAttr().Get()
            if before and inertia and any(inertia):   # keep the shape of an authored inertia, at the new mass
                api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*(float(v) * mass / before for v in inertia)))
            api.CreateMassAttr().Set(mass)
        previous = bodies[role]
        offset += lengths[role]

    bota = None
    if "bota" in order:
        sensor = UsdGeom.Xform.Define(stage, bodies["bota"].GetPath().AppendChild("bota_sensor")).GetPrim()
        origin = float(config["bota"]["sensor_origin_along_axis_m"])
        if not math.isfinite(origin) or not 0 <= origin <= lengths["bota"]:
            raise ValueError("Bota sensor origin must lie within configured adapter length")
        sensor_pose = _pose((axis * origin).tolist(), config["bota"].get("sensor_orientation_wxyz", [1, 0, 0, 0]))
        _set_world(sensor, sensor_pose * _mount_world(mounts["bota"], sources["bota"][1]))
        after = order[order.index("bota") + 1]                           # what the sensor's tool-side face holds
        bota = {"body_path": str(bodies["bota"].GetPath()), "sensor_path": str(sensor.GetPath()),
                "joint_path": str(mount_joints[after].GetPath()), "load_body_path": str(bodies[after].GetPath())}
    pad_paths = _pads(stage, root, roots["rg2"], config, reference) if pads else []
    section, carried = config.get("inertials"), []
    rg2_body = bodies["rg2"].GetName()
    group = next((name for name, links in (section or {}).items() if rg2_body in links), None)
    if tool_stack.carried_on_rg2(config):
        if group is None:
            raise ValueError("inertials must hold the RG2 body to add what it carries (the camera bracket and the D415)")
        combined, carried = tool_stack.rg2_body_with_carried(config, section[group][rg2_body])
        section = {**section, group: {**section[group], rg2_body: combined}}
    inertials = _inertials(stage, root, section) if section else {}
    if inertials:
        limitations.append("Explicit mass properties from the config's inertials replace the pack's on "
                           + ", ".join(sorted(inertials)) + ".")
    if carried:
        limitations.append("The RG2 body's mass properties include " + " and ".join(item["item"] for item in carried)
                           + " as point masses at their centres of gravity; their own inertia is left out.")
    if sources["rg2"][1].get("fidelity", "unverified") != "verified":
        limitations.append("Genuine standard RG2 asset remains unavailable/unverified; max opening is a command bound only.")
    return {"robot_path": str(roots["robot"].GetPath()), "articulation_root": str(arm.GetPath()),
            "base_plate_path": str(plate.GetPath()) if plate else None,
            "flange_path": str(flange.GetPath()), "tool0_path": str(tool0.GetPath()),
            "gripper_path": str(roots["rg2"].GetPath()),
            "gripper_articulation_root": str(gripper_articulation.GetPath()) if gripper_articulation else None,
            "flange_joint_path": str(mount_joints[order[0]].GetPath()), "flange_load_body_path": str(bodies[order[0]].GetPath()),
            "bota": bota, "proxies": proxies, "carried_on_rg2": carried, "pad_paths": pad_paths,
            "rail": rail_report, "arm_base_link": str(mounts["robot"].GetPath()),
            "parts": parts, "limitations": limitations, "max_opening_m": 0.110,
            "stack_order": list(order), "assembly_path": str(root),
            "frame_bodies_neutralized": frame_bodies, "joints_deactivated": dropped_joints,
            "velocity_iterations_clamped": clamped, "inertials_kg": inertials}


def _rail(stage, root, robot_root, arm_root, base_link, rail, reference, limitations):
    """Make the robot's base a seventh axis: rail link (the articulation's fixed root), prismatic joint, carriage.

    The carriage asset's origin is the robot's base with the world's axes, so the rail link and the carriage both take
    the base's position and the assembly's parent's orientation, and the joint frames are the identity. The joints are
    authored under the robot's own prim, beside the arm's root joint, where joint tools look for an articulation's
    joints. Nothing moves at the built pose: the joint's zero is the pose the cell is built in.
    """
    if not UsdPhysics.Joint(arm_root) or UsdPhysics.Joint(arm_root).GetBody1Rel().GetTargets() != [base_link.GetPath()]:
        raise ValueError("rail.moving needs the robot's articulation root to be a joint fixing its base link to the world")
    if arm_root.GetParent() != robot_root:
        raise ValueError("rail.moving needs the robot's root joint directly under the robot's prim")
    name = rail["joint_name"]
    if robot_root.GetChild(name) or robot_root.GetChild("carriage_mount"):
        raise ValueError(f"the robot asset already has a prim named {name} or carriage_mount")
    base_world = _world(base_link)
    frame = Gf.Matrix4d().SetTranslate(base_world.ExtractTranslation())    # the world's axes, at the robot's base
    part = reference("rail_carriage", root.AppendChild("rail"))            # the BOM part: the axis's moving half
    _set_world(part, frame)
    carriage = part
    for prim in Usd.PrimRange(part):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.Joint) or prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise ValueError("rail_carriage must be plain geometry with colliders: the builder makes it a body")
    UsdPhysics.RigidBodyAPI.Apply(carriage)
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeUntransformedBound(carriage).ComputeAlignedRange()
    if box.IsEmpty():
        raise ValueError("rail_carriage has no geometry")
    # The carriage rides on the rail: what touches the rail (the gantry plate and its bearing blocks, within 15 mm of
    # the carriage's underside) must not collide with it, or the contact would brake the axis. The robot plate keeps
    # its collider.
    boxes = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide", "proxy"])
    riding = []
    for prim in Usd.PrimRange(part):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            bound = boxes.ComputeRelativeBound(prim, part).ComputeAlignedRange()
            if bound.GetMin()[2] < box.GetMin()[2] + 0.015:
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                riding.append(str(prim.GetPath()))
    if not riding or len(riding) == sum(1 for prim in Usd.PrimRange(part) if prim.HasAPI(UsdPhysics.CollisionAPI)):
        raise ValueError("rail_carriage needs colliders both on the part that rides on the rail and above it")
    size, centre, mass = box.GetSize(), box.GetMidpoint(), rail["carriage_mass_kg"]
    api = UsdPhysics.MassAPI.Apply(carriage)                               # a uniform box of the carriage's own bounds
    api.CreateMassAttr().Set(mass)
    api.CreateCenterOfMassAttr().Set(Gf.Vec3f(centre))
    api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*(mass / 12.0 * (size[(i + 1) % 3] ** 2 + size[(i + 2) % 3] ** 2) for i in range(3))))
    api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
    link = UsdGeom.Xform.Define(stage, robot_root.GetPath().AppendChild("rail_link")).GetPrim()
    _set_world(link, frame)
    UsdPhysics.RigidBodyAPI.Apply(link)
    api = UsdPhysics.MassAPI.Apply(link)                                   # fixed to the world: its mass moves nothing
    api.CreateMassAttr().Set(rail["rail_mass_kg"])
    api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, -0.09))
    api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.05, 6.0, 6.0))
    api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
    root_joint = UsdPhysics.Joint(arm_root)
    root_joint.GetBody1Rel().SetTargets([link.GetPath()])                  # world -> rail link; same prim path as before
    root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(frame.ExtractTranslation()))
    root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    root_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint = UsdPhysics.PrismaticJoint.Define(stage, robot_root.GetPath().AppendChild(name))
    joint.CreateAxisAttr("XYZ"[rail["axis_in_world"].index(1.0)])
    joint.CreateBody0Rel().SetTargets([link.GetPath()])
    joint.CreateBody1Rel().SetTargets([carriage.GetPath()])
    for attr in (joint.CreateLocalPos0Attr(), joint.CreateLocalPos1Attr()):
        attr.Set(Gf.Vec3f(0, 0, 0))
    for attr in (joint.CreateLocalRot0Attr(), joint.CreateLocalRot1Attr()):
        attr.Set(Gf.Quatf(1, 0, 0, 0))
    lower, upper = rail["limits_m"]
    joint.CreateLowerLimitAttr().Set(lower)
    joint.CreateUpperLimitAttr().Set(upper)
    joint.CreateCollisionEnabledAttr().Set(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")           # PhysX's gains; Newton's are set by newton_cell
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(rail["drive"]["physx"]["stiffness_n_per_m"])
    drive.CreateDampingAttr(rail["drive"]["physx"]["damping_ns_per_m"])
    drive.CreateMaxForceAttr(rail["max_force_n"])
    drive.CreateTargetPositionAttr(rail["default_position_m"])
    joint.GetPrim().CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(rail["speed_m_s"])
    mount = _fixed(stage, robot_root.GetPath().AppendChild("carriage_mount"), carriage, base_link, base_world, False)
    limitations.append("The robot's base is a seventh axis: joint tools see " + name + " first, in metres from the built pose. "
                       "Inverse kinematics without base_link solves the rail too; pass the report's arm_base_link for the arm alone.")
    limitations.append("Rail: the carriage's gantry plate and bearing blocks have no collision (they ride on the rail); the "
                       "robot plate on it has.")
    limitations.append("Rail: the travel is Vention's for this actuator; speed and acceleration are simulation choices, and where "
                       "the real axis's zero is on the rail is an assumption. Only PhysX applies the speed as a joint velocity limit.")
    return {**rail_axis.caller_view(rail), "joint_path": str(joint.GetPath()), "carriage_path": str(carriage.GetPath()),
            "rail_link_path": str(link.GetPath()), "carriage_mount_path": str(mount.GetPath()),
            "carriage_mass_kg": mass, "part_path": str(part.GetPath()), "colliders_off_where_it_rides_on_the_rail": len(riding)}


def _plate(stage, path, record):
    """A placeholder adapter plate: one rigid disc along +X from its robot-side face, with a solid disc's inertia."""
    thickness, radius, mass = record["thickness_m"], record["diameter_m"] / 2.0, record["mass_kg"]
    body = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    api = UsdPhysics.MassAPI.Apply(body)
    api.CreateMassAttr().Set(mass)
    api.CreateCenterOfMassAttr().Set(Gf.Vec3f(thickness / 2.0, 0, 0))
    across = mass * (3 * radius * radius + thickness * thickness) / 12.0
    api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(mass * radius * radius / 2.0, across, across))
    api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
    disc = UsdGeom.Cylinder.Define(stage, path.AppendChild("disc"))
    disc.CreateAxisAttr("X")
    disc.CreateHeightAttr(thickness)
    disc.CreateRadiusAttr(radius)
    disc.CreateExtentAttr([Gf.Vec3f(-thickness / 2.0, -radius, -radius), Gf.Vec3f(thickness / 2.0, radius, radius)])
    disc.AddTranslateOp().Set(Gf.Vec3d(thickness / 2.0, 0, 0))
    disc.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.63, 0.65)])
    UsdPhysics.CollisionAPI.Apply(disc.GetPrim())
    body.SetCustomDataByKey("workcell:fidelity", "placeholder")
    body.SetCustomDataByKey("workcell:proxy", "adapter plate: a disc from the config, not a measured part")
    return body


INERTIAL_KEYS = {"mass_kg", "com_m", "diagonal_inertia_kgm2", "principal_axes_wxyz"}


def check_inertials(section):
    """Refuse malformed explicit mass properties before any stage is replaced. section: {group: {link: props}}."""
    if not isinstance(section, dict) or not section:
        raise ValueError("inertials must map groups of links to their mass properties")
    seen = set()
    for group, links in section.items():
        if not isinstance(links, dict) or not links:
            raise ValueError(f"inertials.{group} must map link names to mass properties")
        for link, props in links.items():
            if link in seen:
                raise ValueError(f"inertials names {link} twice")
            seen.add(link)
            if not isinstance(props, dict) or set(props) != INERTIAL_KEYS:
                raise ValueError(f"inertials.{group}.{link} needs exactly {sorted(INERTIAL_KEYS)}")
            vectors = (props["com_m"], props["diagonal_inertia_kgm2"], props["principal_axes_wxyz"])
            if not all(isinstance(vector, (list, tuple)) for vector in vectors):
                raise ValueError(f"inertials.{group}.{link} has a malformed vector")
            values = [props["mass_kg"], *props["com_m"], *props["diagonal_inertia_kgm2"], *props["principal_axes_wxyz"]]
            if (len(props["com_m"]) != 3 or len(props["diagonal_inertia_kgm2"]) != 3
                    or len(props["principal_axes_wxyz"]) != 4
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)):
                raise ValueError(f"inertials.{group}.{link} has a malformed vector or a non-finite value")
            if props["mass_kg"] <= 0 or min(props["diagonal_inertia_kgm2"]) <= 0:
                raise ValueError(f"inertials.{group}.{link} needs a positive mass and principal moments")
            a, b, c = sorted(props["diagonal_inertia_kgm2"])
            if a + b < c * (1 - 1e-6):
                raise ValueError(f"inertials.{group}.{link} principal moments break the triangle inequality")
            if abs(math.sqrt(sum(v * v for v in props["principal_axes_wxyz"])) - 1) > 1e-6:
                raise ValueError(f"inertials.{group}.{link} principal_axes_wxyz must be a unit quaternion")


def check_inertial_links(section, asset_paths):
    """Each inertial link must name exactly one rigid body across the given asset files (opened read-only), so a
    mistyped name is refused before any stage is replaced."""
    counts = {}
    for path in asset_paths:
        asset = Usd.Stage.Open(str(path), Usd.Stage.LoadAll)
        for prim in asset.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                counts[prim.GetName()] = counts.get(prim.GetName(), 0) + 1
    for group, links in section.items():
        for link in links:
            if counts.get(link, 0) != 1:
                raise ValueError(f"inertials.{group}.{link}: expected one rigid body of that name in the robot and "
                                 f"gripper assets, found {counts.get(link, 0)}")


def _inertials(stage, root, section):
    """Author explicit mass, centre of mass and principal inertia on named bodies of the assembly, so neither
    engine computes them from collider shapes. Every named link must exist exactly once."""
    check_inertials(section)
    bodies = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.setdefault(prim.GetName(), []).append(prim)
    applied = {}
    for group, links in section.items():
        for link, props in links.items():
            found = bodies.get(link, [])
            if len(found) != 1:
                raise ValueError(f"inertials.{group}.{link}: expected one rigid body of that name, found {len(found)}")
            api = UsdPhysics.MassAPI.Apply(found[0])
            api.CreateMassAttr().Set(float(props["mass_kg"]))
            api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*map(float, props["com_m"])))
            api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float, props["diagonal_inertia_kgm2"])))
            w, x, y, z = map(float, props["principal_axes_wxyz"])
            api.CreatePrincipalAxesAttr().Set(Gf.Quatf(w, x, y, z))
            applied[link] = float(props["mass_kg"])
    return applied


def _pads(stage, root, gripper, config, reference):
    settings = config["finger_pads"]
    mounts = settings.get("mounts", [])
    if len(mounts) != 2:
        raise ValueError("finger_pads.mounts needs two standard-RG2 finger body poses; legacy offsets are unsupported")
    material = UsdShade.Material.Define(stage, root.AppendChild("black_pad_material"))
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    for name in ("static", "dynamic"):
        value = _positive(settings[f"{name}_friction"], f"{name}_friction")
        getattr(physics, f"Create{name.title()}FrictionAttr")().Set(value)
    paths = []
    for index, mount in enumerate(mounts):
        parent = _relative(stage, gripper.GetPath(), mount["parent"])
        if not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError("Pad parent must be an RG2 rigid finger body")
        path = parent.GetPath().AppendChild(f"black_pad_{index}")
        if stage.GetPrimAtPath(path):
            raise ValueError("Pad path already exists in pack")
        pad = reference("finger_pad", path)
        if any(prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.Joint)
               for prim in Usd.PrimRange(pad)):
            raise ValueError("Pad assets must be collision/visual geometry, not nested rigid bodies")
        _set_world(pad, _pose(mount["position_m"], mount["orientation_wxyz"]) * _world(parent))
        for prim in Usd.PrimRange(pad):
            if prim.IsA(UsdGeom.Gprim):   # part of the finger body, not nested; a mesh as a convex hull (both engines)
                UsdPhysics.CollisionAPI.Apply(prim)
                if prim.IsA(UsdGeom.Mesh):
                    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
        binding = UsdShade.MaterialBindingAPI.Apply(pad)
        binding.Bind(material, UsdShade.Tokens.strongerThanDescendants)
        binding.Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
        paths.append(str(path))
    return paths


def _runtime(view):
    try:
        ready = view is not None and view.check() and view.count == 1
    except Exception:
        ready = False
    if not ready:
        raise RuntimeError("UNAVAILABLE: initialize one articulation tensor view first")
    names = list(view.shared_metatype.dof_names)
    if len(names) != len(set(names)):
        raise RuntimeError("UNSUPPORTED: duplicate runtime DOF names")
    return names


def _row(view, method, count):
    values = getattr(view, method)()
    if not isinstance(values, np.ndarray):
        raise RuntimeError("UNSUPPORTED: use a NumPy-frontend articulation tensor view")
    if values.shape != (1, count) or not np.isfinite(values).all():
        raise RuntimeError(f"PHYSICS_STATE_INVALID: invalid {method}")
    return values.copy()


def robot_state(view):
    """Read initialized NumPy ArticulationView DOFs, in radians/metres, no USD fallback."""
    names = _runtime(view)
    return {"status": "measured", "joint_names": names,
            "positions": _row(view, "get_dof_positions", len(names))[0].tolist(),
            "velocities": _row(view, "get_dof_velocities", len(names))[0].tolist(),
            "units": "revolute: rad, rad/s; prismatic: m, m/s (metre stage required)"}


def command_joints(view, targets):
    """Set named position targets, preserving untouched targets; never teleport/step."""
    names = _runtime(view)
    if not targets or set(targets) - set(names):
        raise ValueError("Expected a nonempty mapping of known joint names")
    row = _row(view, "get_dof_position_targets", len(names))
    limits = np.asarray(view.get_dof_limits())
    if (limits.shape != (1, len(names), 2) or np.isnan(limits).any()
            or np.any(limits[:, :, 0] > limits[:, :, 1])):
        raise RuntimeError("PHYSICS_STATE_INVALID: invalid DOF limits")
    for name, target in targets.items():
        index = names.index(name)
        if (isinstance(target, bool) or not math.isfinite(target)
                or not limits[0, index, 0] <= target <= limits[0, index, 1]):
            raise ValueError(f"Invalid or out-of-limit target for {name}")
        row[0, index] = target
    if not np.isfinite(row).all():
        raise ValueError("Targets overflow the runtime dtype")
    if view.set_dof_position_targets(row, np.array([0], dtype=np.uint32)) is False:
        raise RuntimeError("UNAVAILABLE: runtime rejected position targets")
    return {"status": "commanded", "targets": {name: float(target) for name, target in targets.items()}}


def command_gripper(view, width_m, config):
    """Interpolate an explicitly calibrated standard-RG2 width->named-joint table.

    rg2.width_calibration is a list of {width_m, joints:{name:value}} knots,
    strictly ascending and covering exactly 0..0.110 m. Angles are radians.
    This is a target, never a measured opening or a claimed Modbus interface.
    """
    if isinstance(width_m, bool) or not math.isfinite(width_m) or not 0 <= width_m <= 0.110:
        raise ValueError("Standard RG2 width must be finite and within 0..0.110 m")
    table = config["rg2"].get("width_calibration")
    if not table:
        return {"status": "unsupported", "reason": "Standard RG2 width_calibration is not configured"}
    widths = np.array([knot["width_m"] for knot in table], dtype=float)
    names = list(table[0]["joints"])
    if (len(table) < 2 or not names or not np.isfinite(widths).all()
            or widths[0] != 0 or not math.isclose(widths[-1], 0.110, abs_tol=1e-9)
            or not np.all(np.diff(widths) > 0)
            or any(set(knot["joints"]) != set(names) for knot in table)):
        raise ValueError("width_calibration must cover 0..0.110 m with consistent named joints")
    samples = np.array([[knot["joints"][name] for name in names] for knot in table], dtype=float)
    if not np.isfinite(samples).all():
        raise ValueError("width_calibration joint samples must be finite")
    result = command_joints(view, {name: float(np.interp(width_m, widths, samples[:, index]))
                                   for index, name in enumerate(names)})
    return {**result, "requested_width_m": float(width_m), "measured_width_m": None}
