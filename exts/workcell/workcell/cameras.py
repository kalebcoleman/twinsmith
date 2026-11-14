"""Work-cell cameras: a wrist D415 and, when ``overhead_camera.enabled``, an overhead one, built from pack geometry.

Plain functions, no work cell mixin. ``build_cameras`` is the entry point the workcell
scene calls; ``plan_cameras`` is the same plan without USD authoring, so host
tests can check intrinsics, optical offsets and BOM tagging on CPU.

Geometry may only come from the external versioned pack, through
``assets.asset(logical_id) -> absolute Path`` (a role-to-Path mapping is also
accepted for the scene integration). Nothing here reads ``settings.py``, the
asset cache or any checkout path, and nothing writes to the asset store.

Pack mesh conventions this module assumes (all parts: metres, Z-up, one default
Xform prim at the local origin, no joints):

* ``d415``: unconfirmed dimensioned proxy, body centred at the origin and
  modelled 0.099 x 0.020 x 0.023 m. Local axes are the D415 optical frame:
  +X right, +Y down, +Z forward. It is not a D455/rsd455 substitute. The render
  camera sits at this origin and applies Rx(180 deg) because a USD camera looks
  down -Z with +Y up.
* ``wrist_mount``: centred proxy box 0.030 x 0.060 x 0.012 m, local axes equal
  to the optical frame. It is centred between tool0 and the camera optical
  origin; the manifest may refine that with ``asset_layout.wrist_mount.mount_pose``.
* ``overhead_mount``: open three-member frame with a unit envelope, centred at
  the origin, axes equal to the world. Member proportions are provisional.
  It is scaled to the gantry envelope: extrusion thickness x post span x height,
  centred at the post midpoint and half the configured height.
* ``overhead_release`` / ``overhead_shelf``: centred boxes with world axes,
  placed at the camera optical position. The pack owns their real dimensions;
  the shelf may use ``overhead_mount.printed_shelf_size_m``, and
  ``asset_layout.<role>.mount_pose`` can separate their stack.

No fiducial or marker geometry is authored or referenced. Extrinsics are config
values, pending markerless hand-eye calibration.
"""

from __future__ import annotations

import math
from pathlib import Path

try:  # package import inside Kit
    from .hardware_config import camera_intrinsics
except ImportError:  # loaded by file path in host tests
    from hardware_config import camera_intrinsics


SCHEMA = 1
OVERHEAD_ASSET_ROLES = ("overhead_mount", "overhead_release", "overhead_shelf")
REQUIRED_ASSET_ROLES = ("d415", "wrist_mount", *OVERHEAD_ASSET_ROLES)
D415_BODY_SIZE_M = (0.099, 0.020, 0.023)
WRIST_BRACKET_SIZE_M = (0.030, 0.060, 0.012)
DEFAULT_APERTURE_MM = 36.0
DEFAULT_CLIPPING_RANGE_M = (0.1, 10.0)
USD_CAMERA_ROTATION = (0.0, 1.0, 0.0, 0.0)  # Rx(180 deg), wxyz
OVERHEAD_ROOT = "/World/Workcell/cameras/overhead"
REQUIRED_ROBOT_FRAMES = ("flange_path", "tool0_path", "gripper_path", "articulation_root", "robot_path")
REQUIRED_CONFIG_PATHS = (
    "pack.id", "pack.version",
    *(f"assets.{role}" for role in REQUIRED_ASSET_ROLES),
    "wrist_camera.offset_in_tool0_m", "wrist_camera.rotation_in_tool0_deg",
    "wrist_camera.resolution", "wrist_camera.color_fov_deg", "wrist_camera.ros_namespace",
    "wrist_camera.body_size_m", "wrist_camera.bracket_size_m",
    "overhead_camera.enabled", "overhead_camera.position_m", "overhead_camera.look_at_m", "overhead_camera.up_hint",
    "overhead_camera.resolution", "overhead_camera.color_fov_deg", "overhead_camera.ros_namespace",
    "overhead_mount.type", "overhead_mount.extrusion_size_m", "overhead_mount.post_xy_m",
    "overhead_mount.height_m", "overhead_mount.removable", "overhead_mount.printed_shelf_size_m",
    "robot.reach_m",
    "interfaces.wrist_camera_topics", "interfaces.overhead_camera_topics",
)
FORBIDDEN_PATH_TOKENS = ("ur5e-digital-twin", "/.cache/", "/raw/")
_REPO_ROOT = Path(__file__).resolve().parents[3]


def build_cameras(stage, assets, config, robot_report, *, author=None, config_path=None):
    """Author the wrist D415 camera, and the overhead one when enabled, and return a JSON report.

    ``assets`` exposes ``asset(logical_id) -> absolute Path``; the workcell scene
    may pass its resolved ``{role: Path}`` mapping instead. ``config`` is the
    flattened mapping from ``hardware_config.load_config()``. ``robot_report``
    provides at least the five frame keys listed in ``REQUIRED_ROBOT_FRAMES``.

    The report has ``cameras`` (wrist and overhead, each with ``prim_path``,
    optical transform, USD conversion, intrinsics, CameraInfo, RGB/depth and
    proposed topics), ``parts`` (one ``{prim_path, asset_id, bom_id}`` per
    referenced pack part), ``limitations`` and provenance. ``author`` is a test
    seam: ``author(stage, internal_report)``; the default uses pxr.
    """
    if stage is None:
        raise ValueError("build_cameras needs a USD stage")
    report = plan_cameras(config, robot_report, assets=assets, config_path=config_path)
    report["authoring"] = (author or author_report)(stage, report)
    return _public_report(report)


def plan_cameras(config, robot_report, assets=None, config_path=None):
    """Plan both cameras without USD; internal ``_path`` keys are stripped by build."""
    _check_flattened(config)
    robot = _robot_frames(robot_report)
    assets_cfg = config.get("assets") or {}
    if not isinstance(assets_cfg, dict):
        raise ValueError("config.assets must be a mapping of roles to pack ids")

    limitations = [
        "The D415 body is an explicitly unconfirmed dimensioned proxy (0.099 x 0.020 x 0.023 m), "
        "not a D455/rsd455 substitute and not a verified CAD model.",
        "Extrinsics come only from config; no fiducial or marker-based calibration is used, and "
        "markerless hand-eye calibration is pending.",
        "Rendered depth is single-pinhole distance_to_image_plane, not D415 stereo or IR depth; "
        "no projector, noise or stereo matching is modelled.",
        "ROS topics are proposed realsense2_camera 4.x names, not published or verified; no ROS graph "
        "is built. Max's pipeline reads the sensor through pyrealsense2 directly.",
        "Mount, shelf and release geometry uses the pack's centred-proxy convention; asset-local "
        "datums and stacked offsets still need CAD or manifest confirmation.",
        "Capture needs a running Isaac RTX renderer and a rendered frame; no pixels are synthesized.",
        "The capture helper targets Isaac Sim 6.x isaacsim.sensors.experimental.rtx; older releases "
        "only have the deprecated camera API and must not silently substitute another source.",
        "Camera bodies and mounts are visual-only: RigidBodyAPI and CollisionAPI are removed from "
        "referenced parts, so nothing becomes a nested rigid body under tool0.",
    ]
    wrist, wrist_parts, wrist_root = _plan_wrist(config, robot, assets, assets_cfg, limitations)
    overhead, overhead_parts, overhead_root = _plan_overhead(config, assets, assets_cfg, limitations)
    parts = wrist_parts + overhead_parts
    roots = [root for root in (wrist_root, overhead_root) if root]

    wanted = [camera for camera in (wrist, overhead) if camera["status"] != "disabled"]
    built = [camera for camera in wanted if camera["status"] == "built"]
    if len(built) == len(wanted) and len(parts) == (6 if overhead in wanted else 2):
        status = "ok"
    elif built:
        status = "partial"
    else:
        status = "unavailable"

    report = {
        "schema": SCHEMA,
        "status": status,
        "cameras": {"wrist": wrist, "overhead": overhead},
        "parts": parts,
        "roots": roots,
        "limitations": _dedupe(limitations),
        "provenance": {
            "config": {"path": str(config_path) if config_path else None,
                       "note": "Flattened by hardware_config.load_config(); per-leaf status and source stay in the raw JSON."},
            "pack": _value(config, "pack"),
            "assets": {"loader": _loader_name(assets), "resolved_roles": [p["role"] for p in parts]},
            "robot": robot,
            "conventions": {"optical": "+X right, +Y down, +Z forward",
                            "usd_camera": "-Z forward, +Y up",
                            "conversion": "Rx(180 deg) about the camera +X axis",
                            "rotation_in_tool0_deg_order": "extrinsic XYZ"},
        },
        "required_config": {path: _value(config, path) for path in REQUIRED_CONFIG_PATHS},
        "asset_conventions": _asset_conventions(config),
        "capture": _capture_contract(),
        "fiducials": {"used": False,
                      "note": "No fiducial or marker geometry is authored or referenced; extrinsics are config values."},
    }
    return report


def author_report(stage, report, usd=None):
    """Author the planned roots, pack references and cameras into ``stage``."""
    Gf, Usd, UsdGeom, UsdPhysics = usd or _import_usd()
    _require_units(stage, UsdGeom)
    for root in report["roots"]:
        parent = root.get("parent")
        if parent and not _valid(stage, parent):
            raise ValueError(f"robot frame {parent} is not on the stage; no cameras were built")

    for part in report["parts"]:
        if not part.get("_path"):
            raise ValueError(f"part {part['prim_path']} has no resolved pack path")

    for root in report["roots"]:
        prim = _define_xform(stage, UsdGeom, root["prim_path"])
        _set_transform(Gf, UsdGeom, prim, root["translation_m"], root["quaternion_wxyz"])

    parts_authored, physics_removed = [], []
    for part in report["parts"]:
        prim = stage.GetPrimAtPath(part["prim_path"])
        if not prim or not prim.IsValid():
            prim = _define_xform(stage, UsdGeom, part["prim_path"])
        if not UsdGeom.Xformable(prim):
            raise ValueError(f"part prim is not transformable: {part['prim_path']}")
        if prim.IsInstance():
            raise ValueError(f"part root must be editable, not instanceable: {part['prim_path']}")
        if not prim.HasAuthoredReferences():
            if not prim.GetReferences().AddReference(part["_path"]):
                raise ValueError(f"could not reference {part['asset_id']} at {part['prim_path']}")
        _set_transform(Gf, UsdGeom, prim, part["local"]["translation_m"],
                       part["local"]["quaternion_wxyz"], part["local"].get("scale"))
        prim.SetCustomDataByKey("workcell:asset_id", part["asset_id"])
        prim.SetCustomDataByKey("workcell:bom_id", part["bom_id"])
        prim.SetCustomDataByKey("workcell:fidelity", part["fidelity"])
        if any(item.IsA(UsdPhysics.Joint) for item in Usd.PrimRange(prim)):
            raise ValueError(f"camera part must not contain joints: {part['prim_path']}")
        physics_removed += _strip_physics(Usd, UsdPhysics, prim)
        parts_authored.append(part["prim_path"])

    cameras_authored = []
    for key in ("wrist", "overhead"):
        camera = report["cameras"][key]
        if camera["status"] != "built":
            continue
        cam = UsdGeom.Camera.Define(stage, camera["prim_path"])
        cam.CreateProjectionAttr().Set("perspective")
        cam.CreateFocalLengthAttr().Set(camera["intrinsics"]["focal_length_mm"])
        cam.CreateHorizontalApertureAttr().Set(camera["intrinsics"]["horizontal_aperture_mm"])
        cam.CreateVerticalApertureAttr().Set(camera["intrinsics"]["vertical_aperture_mm"])
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(*camera["depth"]["clipping_range_m"]))
        _set_transform(Gf, UsdGeom, cam.GetPrim(), camera["usd_camera"]["translation_m"],
                       camera["usd_camera"]["quaternion_wxyz"])
        cameras_authored.append(camera["prim_path"])

    wrist = report["cameras"]["wrist"]
    remaining = []
    if wrist["status"] == "built" and _valid(stage, wrist["root_path"]):
        root = stage.GetPrimAtPath(wrist["root_path"])
        remaining = [str(prim.GetPath()) for prim in Usd.PrimRange(root)
                     if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if remaining:
        report["limitations"].append(
            "Rigid bodies remain under the wrist mount after authoring: " + ", ".join(remaining)
            + "; verify the pack assets.")
    return {"roots": [root["prim_path"] for root in report["roots"]],
            "parts": parts_authored, "cameras": cameras_authored,
            "physics_removed": physics_removed, "rigid_bodies_remaining": remaining}


def make_camera_sensor(camera_prim, resolution, annotators=("rgb", "distance_to_image_plane")):
    """Create Isaac's official RTX ``CameraSensor`` for a built camera prim.

    ``resolution`` is ``[width, height]``; the sensor API takes ``(height, width)``.
    The extension is ``isaacsim.sensors.experimental.rtx`` in Isaac Sim 6.x.
    Requires a running renderer and a playing/rendered timeline; no pixels are
    synthesized here.
    """
    from isaacsim.sensors.experimental.rtx import CameraSensor

    width, height = _resolution(resolution)
    return CameraSensor(camera_prim, resolution=(height, width), annotators=list(annotators))


def read_frame(sensor, annotator="rgb"):
    """Return ``(data, info)`` from a sensor; refuse to invent a frame."""
    data, info = sensor.get_data(annotator)
    if data is None:
        raise RuntimeError(
            f"No {annotator} frame is available; play or step the timeline so the RTX renderer "
            "produces one. No pixels are synthesized.")
    return data, info


# --------------------------------------------------------------------------- plan

def _plan_wrist(config, robot, assets, assets_cfg, limitations):
    section = config.get("wrist_camera") or {}
    offset = _vector(section.get("offset_in_tool0_m"), 3, "wrist_camera.offset_in_tool0_m")
    rotation = _vector(section.get("rotation_in_tool0_deg"), 3, "wrist_camera.rotation_in_tool0_deg")
    if rotation is None:
        rotation = [0.0, 0.0, 0.0]
        limitations.append("wrist_camera.rotation_in_tool0_deg is missing; identity rotation assumed.")
    if offset is None:
        return _unavailable("wrist", "wrist_camera.offset_in_tool0_m is missing; the wrist camera cannot be placed."), [], None
    resolution, _, intrinsics = _camera_optics(section, "wrist_camera", limitations)
    if resolution is None:
        return _unavailable("wrist", "wrist_camera resolution or FOV is missing/invalid; the wrist camera cannot be built."), [], None

    tool0 = robot["tool0_path"]
    root = f"{tool0}/wrist_d415"
    cam_path = f"{root}/D415_color"
    optical_q = _euler_xyz_deg_to_quat(rotation)
    optical = _pose_block(offset, optical_q)
    # The wrist root prim already carries the optical pose; the camera prim under it only
    # needs the optical-to-USD conversion (applying optical_q again would double the tilt).
    usd = _usd_block(cam_path, root, [0.0, 0.0, 0.0], list(USD_CAMERA_ROTATION))

    body_size = _vector(section.get("body_size_m"), 3, "wrist_camera.body_size_m") or list(D415_BODY_SIZE_M)
    bracket_size = _vector(section.get("bracket_size_m"), 3, "wrist_camera.bracket_size_m") or list(WRIST_BRACKET_SIZE_M)
    # In the wrist root frame, tool0 sits at -R^T * offset; centre the proxy bracket between them.
    bridge = [-0.5 * value for value in _rotate_vector(_quat_conjugate(optical_q), offset)]
    # The pack bracket is a unit proxy; stretch it along its long axis so it spans the
    # lateral gap between the tool axis and the camera instead of leaving the camera floating.
    gap = math.hypot(offset[0], offset[1])
    stretch = max(1.0, gap / bracket_size[1])
    if stretch > 1.0:
        limitations.append(
            f"Wrist bracket proxy is stretched x{stretch:.2f} along its long axis to span the "
            f"{gap:.3f} m lateral tool0 offset; confirm the real bracket CAD before lab use.")

    parts = []
    bracket = _part(config, assets, assets_cfg, "wrist_mount", f"{root}/bracket", root,
                    bridge, optical_q, "printed wrist bracket", limitations,
                    scale=[1.0, stretch, 1.0] if stretch > 1.0 else None)
    if bracket:
        parts.append(bracket)
    body = _part(config, assets, assets_cfg, "d415", f"{root}/body", root,
                 [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], "D415 wrist body proxy", limitations)
    if body:
        parts.append(body)
    geometry = "referenced" if len(parts) == 2 else ("partial" if parts else "unavailable")

    namespace = section.get("ros_namespace") or "wrist_camera"
    if not section.get("ros_namespace"):
        limitations.append("wrist_camera.ros_namespace is missing; 'wrist_camera' assumed for proposed frames/topics.")
    frames = _frames(namespace)
    camera = {
        "status": "built",
        "prim_path": cam_path,
        "root_path": root,
        "parent": tool0,
        "mount": {"placement": section.get("mount_position"),
                  "asset_id": _value(config, "assets.wrist_mount"),
                  "bom_id": _bom_id(config, "wrist_mount"),
                  "offset_in_tool0_m": list(offset),
                  "rotation_in_tool0_deg": list(rotation)},
        "asset_id": _value(config, "assets.d415"),
        "bom_id": _bom_id(config, "d415"),
        "geometry_status": geometry,
        "body_geometry": _proxy_geometry("d415", body_size, "config.wrist_camera.body_size_m or pack proxy default"),
        "optical": optical,
        "usd_camera": usd,
        "intrinsics": intrinsics,
        "camera_info": _camera_info(frames["optical"], resolution, intrinsics),
        "rgb": _rgb_block(resolution),
        "depth": _depth_block(section, resolution),
        "frames": frames,
        "topics": _topics(config, "wrist_camera_topics", namespace, limitations),
        "physics": _physics_block(tool0),
    }
    limitations.append(
        "Wrist body geometry is centred on the optical origin; the pack declares no body-to-optical datum, "
        "and the bracket proxy is bridged from the configured tool0 offset, so overlap is possible.")
    root_entry = {"name": "wrist_mount", "prim_path": root, "parent": tool0,
                  "translation_m": list(offset), "quaternion_wxyz": list(optical_q)}
    return camera, parts, root_entry


def overhead_enabled(config):
    enabled = (config.get("overhead_camera") or {}).get("enabled")
    if type(enabled) is not bool:
        raise ValueError("overhead_camera.enabled must be true or false")
    return enabled


def _plan_overhead(config, assets, assets_cfg, limitations):
    section = config.get("overhead_camera") or {}
    if not overhead_enabled(config):
        return {"status": "disabled", "prim_path": None, "root_path": None, "geometry_status": "not built",
                "reason": "overhead_camera.enabled is false: the cell has the wrist camera only, "
                          "with no overhead camera, gantry, release plate or shelf."}, [], None
    position = _vector(section.get("position_m"), 3, "overhead_camera.position_m")
    target = _vector(section.get("look_at_m"), 3, "overhead_camera.look_at_m")
    up_hint = _vector(section.get("up_hint"), 3, "overhead_camera.up_hint") or [1.0, 0.0, 0.0]
    resolution, _, intrinsics = _camera_optics(section, "overhead_camera", limitations)
    if position is None or target is None or resolution is None:
        return _unavailable("overhead",
                            "overhead_camera position, look_at, resolution or FOV is missing/invalid; "
                            "the overhead camera cannot be built."), [], None

    mount = config.get("overhead_mount") or {}
    reach = _number(_value(config, "robot.reach_m"), "robot.reach_m")
    mount_check = _gantry_check(position, mount, reach, limitations)
    optical_q = _look_at_quaternion(position, target, up_hint)
    optical = _pose_block(position, optical_q)
    usd_q = _quat_mul(optical_q, USD_CAMERA_ROTATION)
    cam_path = f"{OVERHEAD_ROOT}/D415_color"
    usd = _usd_block(cam_path, OVERHEAD_ROOT, position, usd_q)

    body_size = _vector((config.get("wrist_camera") or {}).get("body_size_m"), 3,
                        "wrist_camera.body_size_m") or list(D415_BODY_SIZE_M)
    parts = []
    if mount_check["supports"]:
        mount_pose = mount_check["pose"]
        part = _part(config, assets, assets_cfg, "overhead_mount", f"{OVERHEAD_ROOT}/mount",
                     OVERHEAD_ROOT, mount_pose["translation_m"], mount_pose["quaternion_wxyz"],
                     "gantry mount proxy", limitations, scale=mount_pose["scale"])
        if part:
            parts.append(part)
    else:
        limitations.append("overhead_mount.post_xy_m/height_m are missing; the gantry proxy is not referenced.")
    shelf_size = _vector(mount.get("printed_shelf_size_m"), 3, "overhead_mount.printed_shelf_size_m")
    for role, purpose in (("overhead_release", "removable quick-release proxy"),
                          ("overhead_shelf", "printed camera shelf proxy")):
        part = _part(config, assets, assets_cfg, role, f"{OVERHEAD_ROOT}/{role.split('_')[-1]}",
                     OVERHEAD_ROOT, position, [1.0, 0.0, 0.0, 0.0], purpose, limitations)
        if part:
            parts.append(part)
    body = _part(config, assets, assets_cfg, "d415", f"{OVERHEAD_ROOT}/body", OVERHEAD_ROOT,
                 position, optical_q, "D415 overhead body proxy", limitations)
    if body:
        parts.append(body)
    geometry = "referenced" if len(parts) == 4 else ("partial" if parts else "unavailable")

    namespace = section.get("ros_namespace") or "overhead_camera"
    if not section.get("ros_namespace"):
        limitations.append("overhead_camera.ros_namespace is missing; 'overhead_camera' assumed for proposed frames/topics.")
    frames = _frames(namespace)
    camera = {
        "status": "built",
        "prim_path": cam_path,
        "root_path": OVERHEAD_ROOT,
        "parent": OVERHEAD_ROOT,
        "mount": {"type": mount.get("type"), "removable": mount.get("removable"),
                  "quick_release_base": mount.get("quick_release_base"),
                  "post_xy_m": mount.get("post_xy_m"), "height_m": mount.get("height_m"),
                  "reach_check": {key: mount_check[key]
                                  for key in ("status", "arm_reach_m", "supports", "camera_m", "ok")}},
        "asset_id": _value(config, "assets.d415"),
        "bom_id": _bom_id(config, "d415"),
        "geometry_status": geometry,
        "body_geometry": _proxy_geometry("d415", body_size, "config.wrist_camera.body_size_m or pack proxy default"),
        "optical": optical,
        "usd_camera": usd,
        "intrinsics": intrinsics,
        "camera_info": _camera_info(frames["optical"], resolution, intrinsics),
        "rgb": _rgb_block(resolution),
        "depth": _depth_block(section, resolution),
        "frames": frames,
        "topics": _topics(config, "overhead_camera_topics", namespace, limitations),
        "physics": _physics_block(None),
    }
    if shelf_size:
        camera["mount"]["printed_shelf_size_m"] = shelf_size
    limitations.append(
        "Overhead release and shelf proxies start at the camera optical position with world axes; "
        "asset_layout mount_pose entries stack them between the camera and the gantry beam, and the "
        "stack heights come from the pack's proxy dimensions, not a measured assembly.")
    root_entry = {"name": "overhead", "prim_path": OVERHEAD_ROOT, "parent": None,
                  "translation_m": [0.0, 0.0, 0.0], "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]}
    return camera, parts, root_entry


# ------------------------------------------------------------------------ parts

def _part(config, assets, assets_cfg, role, prim_path, parent, translation, orientation, purpose,
          limitations, scale=None):
    asset_id, path = _resolve_asset(assets, assets_cfg, role, limitations)
    if path is None:
        return None
    layout = _layout(config, role)
    bom_id = layout.get("bom_id") or role
    if not isinstance(bom_id, str) or not bom_id:
        raise ValueError(f"asset_layout.{role}.bom_id must be a nonempty string")
    fidelity = layout.get("fidelity", "unverified")
    if fidelity not in ("placeholder", "unverified", "verified"):
        raise ValueError(f"asset_layout.{role}.fidelity must be placeholder, unverified or verified")
    if fidelity != "verified":
        limitations.append(f"{role}: {fidelity} geometry declared by the pack; not a verified CAD/dynamics claim.")
    pose_t, pose_q, pose_source = _apply_mount_pose(translation, orientation, layout, role)
    return {
        "prim_path": prim_path,
        "parent": parent,
        "role": role,
        "asset_id": asset_id,
        "bom_id": bom_id,
        "fidelity": fidelity,
        "purpose": purpose,
        "pose_source": pose_source,
        "local": {"translation_m": list(pose_t), "quaternion_wxyz": list(pose_q),
                  **({"scale": list(scale)} if scale else {}),
                  "matrix4x4_row_major": _matrix4(pose_q, pose_t)},
        "_path": str(path),
    }


def _resolve_asset(assets, assets_cfg, role, limitations):
    asset_id = assets_cfg.get(role)
    if not isinstance(asset_id, str) or not asset_id:
        limitations.append(f"config.assets.{role} is missing; {role} geometry is not referenced.")
        return None, None
    if assets is None:
        limitations.append(f"No asset resolver was supplied; {role} geometry is not referenced.")
        return asset_id, None
    try:
        if callable(getattr(assets, "asset", None)):
            value = assets.asset(asset_id)
        elif isinstance(assets, dict):
            value = assets[role]
        else:
            raise TypeError("assets must expose asset(id) or map a role to an absolute Path")
    except (FileNotFoundError, KeyError, ValueError, TypeError) as exc:
        limitations.append(f"{role} asset {asset_id!r} is unavailable: {exc}")
        return asset_id, None
    try:
        path = Path(value)
    except TypeError:
        limitations.append(f"{role} asset {asset_id!r} did not resolve to a path.")
        return asset_id, None
    if not path.is_absolute():
        limitations.append(f"{role} asset {asset_id!r} is not an absolute path; refusing it.")
        return asset_id, None
    resolved = path.resolve()
    if resolved.is_relative_to(_REPO_ROOT):
        limitations.append(
            f"{role} asset {asset_id!r} resolves inside the checkout; work-cell geometry must come "
            "from the external versioned pack.")
        return asset_id, None
    text = str(resolved).casefold().replace("\\", "/")
    if any(token in text for token in FORBIDDEN_PATH_TOKENS):
        limitations.append(f"{role} asset {asset_id!r} resolves under an old cache/raw source path; refusing it.")
        return asset_id, None
    if role == "d415" and any(token in (asset_id + " " + text) for token in ("d455", "rsd455")):
        raise ValueError("the d415 role must not resolve to the D455 compatibility asset")
    return asset_id, resolved


def _apply_mount_pose(translation, orientation, layout, role):
    pose = layout.get("mount_pose")
    if not pose:
        return list(translation), list(orientation), "pack centred-proxy convention"
    position = _vector(pose.get("position_m", [0.0, 0.0, 0.0]), 3, f"asset_layout.{role}.mount_pose.position_m")
    quaternion = _quaternion(pose.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0]),
                             f"asset_layout.{role}.mount_pose.orientation_wxyz")
    if position is None or quaternion is None:
        raise ValueError(f"asset_layout.{role}.mount_pose needs finite position_m and orientation_wxyz")
    world_t = [a + b for a, b in zip(translation, _rotate_vector(orientation, position))]
    return world_t, _quat_mul(orientation, quaternion), f"config.asset_layout.{role}.mount_pose"


def _gantry_check(position, mount, reach, limitations):
    check = {"status": "unverified", "arm_reach_m": reach, "supports": [], "camera_m": None, "ok": None}
    if reach is not None and reach <= 0:
        raise ValueError("robot.reach_m must be finite and positive")
    if mount:
        if mount.get("type") != "gantry":
            raise ValueError("overhead_mount.type must be gantry; the clamp alternative is not implemented")
        if mount.get("removable") is not True:
            raise ValueError("overhead_mount.removable must be true; the overhead mount must be removable")
        posts = mount.get("post_xy_m")
        height = _number(mount.get("height_m"), "overhead_mount.height_m")
        if not isinstance(posts, (list, tuple)) or len(posts) != 2:
            raise ValueError("overhead_mount.post_xy_m must contain two [x, y] posts")
        xy = [_vector(post, 2, "overhead_mount.post_xy_m") for post in posts]
        if any(post is None for post in xy):
            raise ValueError("overhead_mount.post_xy_m must contain finite x/y pairs")
        if height is not None and height <= 0:
            raise ValueError("overhead_mount.height_m must be positive")
        supports = []
        for x, y in xy:
            ground = math.hypot(x, y)
            top = math.hypot(x, y, height) if height is not None else None
            supports.append({"xy_m": [x, y], "ground_m": ground, "top_m": top})
        camera_distance = math.sqrt(sum(value * value for value in position))
        check.update({"status": "checked", "supports": supports, "camera_m": camera_distance})
        if reach is not None:
            if camera_distance <= reach:
                raise ValueError(
                    f"overhead camera at {camera_distance:.3f} m is inside the {reach:.3f} m arm reach")
            if any(item["ground_m"] <= reach for item in supports):
                raise ValueError("every overhead gantry support must stand outside the arm reach")
            check["ok"] = True
        else:
            limitations.append("robot.reach_m is missing; overhead gantry clearance is unverified.")
        thickness = _number(mount.get("extrusion_size_m"), "overhead_mount.extrusion_size_m")
        thickness = thickness or 0.0381
        span_x = max(item["xy_m"][0] for item in supports) - min(item["xy_m"][0] for item in supports)
        span_y = max(item["xy_m"][1] for item in supports) - min(item["xy_m"][1] for item in supports)
        height_m = height or position[2]
        center = [sum(item["xy_m"][0] for item in supports) / 2,
                  sum(item["xy_m"][1] for item in supports) / 2,
                  height_m / 2]
        check["pose"] = {"translation_m": center,
                         "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                         "scale": [max(span_x, thickness), max(span_y, thickness), height_m]}
    else:
        limitations.append("overhead_mount is missing; the gantry mount and its clearance are unverified.")
    return check


# --------------------------------------------------------------------- metadata

def _camera_optics(section, label, limitations):
    resolution = _resolution(section.get("resolution"))
    fov = _vector(section.get("color_fov_deg"), 2, f"{label}.color_fov_deg")
    if resolution is None or fov is None:
        return None, None, None
    try:
        fx, fy, cx, cy = camera_intrinsics(list(resolution), list(fov))
    except ValueError as exc:
        limitations.append(f"{label} resolution/FOV is invalid: {exc}")
        return None, None, None
    aperture = DEFAULT_APERTURE_MM
    focal = fx * aperture / resolution[0]
    vertical = resolution[1] * focal / fy
    intrinsics = {
        "width": resolution[0], "height": resolution[1], "fov_deg": list(fov),
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "focal_length_mm": focal, "horizontal_aperture_mm": aperture, "vertical_aperture_mm": vertical,
        "aperture_assumption_mm": aperture,
        "pixel_model": "ideal pinhole, square pixels; no lens distortion",
        "camera_matrix": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
    }
    return resolution, fov, intrinsics


def _camera_info(frame_id, resolution, intrinsics):
    fx, fy, cx, cy = intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]
    return {
        "frame_id": frame_id,
        "width": resolution[0], "height": resolution[1],
        "distortion_model": "plumb_bob", "D": [0.0] * 5,
        "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "P": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        "binning_x": 0, "binning_y": 0,
        "note": "Proposed CameraInfo for the proposed realsense2_camera topics; not published or verified.",
    }


def _rgb_block(resolution):
    return {"encoding": "rgb8", "width": resolution[0], "height": resolution[1],
            "annotator": "rgb", "source": "Isaac RTX renderer",
            "note": "Rendered color; no synthetic pixels are generated by this module."}


def _depth_block(section, resolution):
    clipping = _vector(section.get("clipping_range_m"), 2, "camera.clipping_range_m")
    source = "config"
    if clipping is None:
        clipping = list(DEFAULT_CLIPPING_RANGE_M)
        source = "render default"
    if clipping[0] <= 0 or clipping[1] <= clipping[0]:
        raise ValueError("camera clipping_range_m must be positive and increasing")
    return {"encoding": "32FC1", "width": resolution[0], "height": resolution[1],
            "annotator": "distance_to_image_plane", "units": "m",
            "clipping_range_m": list(clipping), "clipping_source": source,
            "aligned_to": "color",
            "note": "Single-pinhole optical-axis depth from the RTX renderer; not D415 stereo or IR."}


def _topics(config, key, namespace, limitations):
    names = _value(config, f"interfaces.{key}")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) and name for name in names):
        limitations.append(f"interfaces.{key} is missing; no ROS topics are proposed for {namespace}.")
        return {"status": "unavailable", "verified": False, "names": [], "namespace": namespace}
    return {"status": "proposed", "verified": False, "namespace": namespace, "names": list(names),
            "note": "Proposed realsense2_camera 4.x names from config/interfaces; not published or verified. "
                    "Max's localizer reads the sensor through pyrealsense2 directly."}


def _frames(namespace):
    return {"link": f"{namespace}_link", "optical": f"{namespace}_color_optical_frame",
            "depth": f"{namespace}_aligned_depth_to_color_optical_frame"}


def _physics_block(tool0):
    return {"attachment": tool0,
            "rigid_body": "none; referenced RigidBodyAPI and CollisionAPI are removed at author time",
            "note": "Visual-only pack geometry follows the parent transform; no nested rigid body is created."}


def _asset_conventions(config):
    section = config.get("wrist_camera") or {}
    body = _vector(section.get("body_size_m"), 3, "wrist_camera.body_size_m") or list(D415_BODY_SIZE_M)
    bracket = _vector(section.get("bracket_size_m"), 3, "wrist_camera.bracket_size_m") or list(WRIST_BRACKET_SIZE_M)
    shelf = _vector((config.get("overhead_mount") or {}).get("printed_shelf_size_m"), 3,
                    "overhead_mount.printed_shelf_size_m") or [0.1, 0.06, 0.008]
    return {
        "d415": {"kind": "unconfirmed dimensioned proxy", "dimensions_m": body,
                 "default_prim": "centred Xform at the body geometric centre",
                 "axes": "D415 optical frame: +X right, +Y down, +Z forward",
                 "origin": "body centre; the render camera applies Rx(180 deg) for USD",
                 "forbidden_substitute": "D455/rsd455"},
        "wrist_mount": {"kind": "centred proxy box", "dimensions_m": bracket,
                        "axes": "optical frame", "placement": "centre between tool0 and the camera optical origin"},
        "overhead_mount": {"kind": "unconfirmed open gantry proxy", "dimensions_m": [1.0, 1.0, 1.0],
                           "scale": "extrusion thickness x post span x height, centred at the post midpoint",
                           "axes": "world / Z-up"},
        "overhead_release": {"kind": "centred proxy box", "dimensions_m": None,
                             "axes": "world / Z-up", "placement": "camera optical position; mount_pose may offset"},
        "overhead_shelf": {"kind": "centred proxy box", "dimensions_m": shelf,
                           "axes": "world / Z-up", "placement": "camera optical position; mount_pose may offset"},
    }


def _capture_contract():
    return {"backend": "isaacsim.sensors.experimental.rtx.CameraSensor",
            "verified": False,
            "functions": ["make_camera_sensor", "read_frame"],
            "note": "Isaac Sim 6.x only. Requires a running Isaac RTX renderer and a playing/rendered frame. "
                    "No pixels are synthesized; read_frame returns renderer data or raises."}


def _proxy_geometry(role, dimensions, source):
    return {"role": role, "kind": "unconfirmed dimensioned proxy", "dimensions_m": dimensions,
            "source": source, "verified": False}


def _unavailable(key, reason):
    return {"status": "unavailable", "prim_path": None, "root_path": None,
            "geometry_status": "unavailable", "reason": reason}


# ------------------------------------------------------------------------ math

def _pose_block(translation, quaternion):
    return {"translation_m": list(translation), "quaternion_wxyz": list(quaternion),
            "matrix4x4_row_major": _matrix4(quaternion, translation)}


def _usd_block(prim_path, parent, translation, quaternion):
    return {"prim_path": prim_path, "parent": parent,
            "translation_m": list(translation), "quaternion_wxyz": list(quaternion),
            "matrix4x4_row_major": _matrix4(quaternion, translation),
            "conversion": "Rx(180 deg) about +X maps the optical frame (+Z forward, +Y down) to a USD camera "
                          "(-Z forward, +Y up).",
            "optical_from_usd": {"rotation_deg_xyz": [180.0, 0.0, 0.0],
                                 "quaternion_wxyz": list(USD_CAMERA_ROTATION)}}


def _matrix4(quaternion, translation):
    w, x, y, z = quaternion
    rows = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]
    return [row + [float(translation[index])] for index, row in enumerate(rows)] + [[0.0, 0.0, 0.0, 1.0]]


def _euler_xyz_deg_to_quat(rotation):
    rx, ry, rz = (math.radians(value) for value in rotation)
    qx = (math.cos(rx / 2), math.sin(rx / 2), 0.0, 0.0)
    qy = (math.cos(ry / 2), 0.0, math.sin(ry / 2), 0.0)
    qz = (math.cos(rz / 2), 0.0, 0.0, math.sin(rz / 2))
    return _quat_normalize(_quat_mul(_quat_mul(qz, qy), qx))


def _look_at_quaternion(position, target, up_hint):
    forward = _normalize([target[i] - position[i] for i in range(3)], "look_at target")
    right = _normalize(_cross(forward, up_hint), "up_hint (parallel to view direction?)")
    down = _cross(forward, right)
    return _quat_from_basis(right, down, forward)


def _quat_from_basis(x_axis, y_axis, z_axis):
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        quat = (0.25 * scale, (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale)
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2
        quat = ((m21 - m12) / scale, 0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale)
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2
        quat = ((m02 - m20) / scale, (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale)
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2
        quat = ((m10 - m01) / scale, (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale)
    return _quat_normalize(quat)


def _quat_mul(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw)


def _quat_conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _quat_normalize(quaternion):
    length = math.sqrt(sum(value * value for value in quaternion))
    if length <= 0 or not math.isfinite(length):
        raise ValueError("Degenerate quaternion")
    return tuple(value / length for value in quaternion)


def _rotate_vector(quaternion, vector):
    w, x, y, z = quaternion
    qv = (x, y, z)
    t = [2 * (qv[1] * vector[2] - qv[2] * vector[1]),
         2 * (qv[2] * vector[0] - qv[0] * vector[2]),
         2 * (qv[0] * vector[1] - qv[1] * vector[0])]
    return [vector[0] + w * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
            vector[1] + w * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
            vector[2] + w * t[2] + (qv[0] * t[1] - qv[1] * t[0])]


def _cross(left, right):
    return [left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0]]


def _normalize(vector, label):
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-9 or not math.isfinite(length):
        raise ValueError(f"Cannot normalize {label}")
    return [value / length for value in vector]


# ------------------------------------------------------------------ config/io

def _check_flattened(config):
    if not isinstance(config, dict):
        raise ValueError("config must be the flattened mapping from hardware_config.load_config()")
    for section, fields in config.items():
        if isinstance(fields, dict):
            for key, value in fields.items():
                if isinstance(value, dict) and set(value) == {"value", "status", "source"}:
                    raise ValueError(f"config.{section}.{key} is a provenance record; pass the flattened config")


def _robot_frames(robot_report):
    if not isinstance(robot_report, dict):
        raise ValueError("robot_report must be a mapping")
    missing = [name for name in REQUIRED_ROBOT_FRAMES
               if not isinstance(robot_report.get(name), str) or not robot_report.get(name).strip()]
    if missing:
        raise ValueError(f"robot_report is missing frame paths: {', '.join(missing)}")
    frames = {name: robot_report[name] for name in REQUIRED_ROBOT_FRAMES}
    if any(not value.startswith("/") for value in frames.values()):
        raise ValueError("robot_report frame paths must be absolute prim paths")
    return frames


def _value(config, path):
    current = config
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def _layout(config, role):
    layout = (config.get("asset_layout") or {}).get(role)
    return layout if isinstance(layout, dict) else {}


def _bom_id(config, role):
    return _layout(config, role).get("bom_id") or role


def _vector(value, length, label):
    if value is None:
        return None
    try:
        values = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain {length} finite numbers") from exc
    if len(values) != length or not all(math.isfinite(item) for item in values):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return values


def _quaternion(value, label):
    values = _vector(value, 4, label)
    if values is None:
        return None
    if not math.isclose(sum(item * item for item in values), 1.0, abs_tol=1e-6):
        raise ValueError(f"{label} must be a unit quaternion")
    return values


def _resolution(value):
    if value is None:
        return None
    try:
        width, height = (int(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError("camera resolution must be [width, height]") from exc
    if len(list(value)) != 2 or width <= 0 or height <= 0:
        raise ValueError("camera resolution must be [width, height]")
    return [width, height]


def _number(value, label):
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _loader_name(assets):
    if assets is None:
        return None
    if isinstance(assets, dict):
        return "role_path_map"
    return type(assets).__name__


def _dedupe(items):
    seen, result = set(), []
    for item in items:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result


def _public_report(report):
    public = dict(report)
    public["parts"] = [{key: value for key, value in part.items() if not key.startswith("_")}
                       for part in report["parts"]]
    return public


# ----------------------------------------------------------------------- USD

def _import_usd():
    try:
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
    except ImportError as exc:
        raise ImportError(
            "build_cameras authors USD and needs Isaac Sim's pxr modules; plan_cameras runs without them"
        ) from exc
    return Gf, Usd, UsdGeom, UsdPhysics


def _require_units(stage, UsdGeom):
    if (UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z
            or not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0)):
        raise ValueError("work-cell cameras require a Z-up stage with metresPerUnit=1")


def _valid(stage, path):
    prim = stage.GetPrimAtPath(path)
    return bool(prim and prim.IsValid())


def _define_xform(stage, UsdGeom, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        return prim
    return UsdGeom.Xform.Define(stage, path).GetPrim()


def _set_transform(Gf, UsdGeom, prim, translation, quaternion, scale=None):
    xform = UsdGeom.Xformable(prim)
    if not xform:
        raise ValueError(f"Not transformable: {prim.GetPath()}")
    xform.ClearXformOpOrder()
    xform.SetResetXformStack(False)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*quaternion))
    if scale and any(not math.isclose(value, 1.0) for value in scale):
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _strip_physics(Usd, UsdPhysics, prim):
    removed = []
    for item in Usd.PrimRange(prim):
        if item.IsInstanceProxy():
            continue
        for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI):
            if item.HasAPI(api):
                item.RemoveAPI(api)
                removed.append(f"{item.GetPath()}:{api.__name__}")
    return removed
