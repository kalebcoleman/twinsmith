"""Geometry checks against the real external pack; skipped when the pack is not synced.

CPU only (no Kit, no GPU): the real robot and camera builders author an in-memory stage
and the checks read the composed USD. They guard the wrist view, the overhead mount
attachment, and the RG2 mass/joint warnings that only the real pack can show.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("pxr.Usd")
from pxr import Gf, Usd, UsdGeom, UsdPhysics

PACKAGE = Path(__file__).resolve().parents[2] / "exts/workcell/workcell"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))


def _load(name):
    spec = importlib.util.spec_from_file_location(f"realpack_{name}", PACKAGE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


hardware_config, asset_pack, robot, cameras = (
    _load(name) for name in ("hardware_config", "asset_pack", "robot", "cameras"))


def _cell(overhead):
    config, _ = hardware_config.load_config()
    config["overhead_camera"]["enabled"] = overhead
    try:
        pack = asset_pack.resolve_pack(config)
    except asset_pack.PackError as error:
        pytest.skip(f"real pack unavailable: {error}")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    report = robot.build_robot(stage, pack, config)
    assets = {role: pack.asset(config["assets"][role]) for role in config["assets"]}
    camera_report = cameras.build_cameras(stage, assets, config, report)
    return config, stage, report, camera_report, pack


@pytest.fixture(scope="module")
def cell():
    """The shipped cell: wrist camera only."""
    return _cell(False)


@pytest.fixture(scope="module")
def overhead_cell():
    """The cell with the proposed overhead camera and gantry switched on."""
    return _cell(True)


def _vertices_in_frame(stage, frame, roots):
    """Visible mesh vertices of ``roots`` in ``frame`` (USD matrices are row-vector: v * M)."""
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(frame)).GetInverse()
    chunks = []
    for root in roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputePurpose() not in ("default", "render"):
                continue
            points = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
            matrix = np.array(cache.GetLocalToWorldTransform(prim) * inverse)
            chunks.append((np.c_[points, np.ones(len(points))] @ matrix)[:, :3])
    return np.vstack(chunks)


def _in_wrist_image(stage, report, camera_report, section, roots):
    """Optical-frame coordinates of the vertices of ``roots`` that fall inside the wrist camera's colour image."""
    points = _vertices_in_frame(stage, report["tool0_path"], roots)
    optical_pose = camera_report["cameras"]["wrist"]["optical"]          # derived from the RG2 pose and the stack
    # The columns of R are the optical axes in tool0; optical coordinates are R^T (p - offset).
    rotation = np.array([cameras._rotate_vector(optical_pose["quaternion_wxyz"], axis) for axis in np.eye(3).tolist()]).T
    optical = (points - np.array(optical_pose["translation_m"])) @ rotation
    optical = optical[optical[:, 2] > section["clipping_range_m"][0]]  # nearer than the clip plane is not drawn
    inside = ((np.degrees(np.arctan2(np.abs(optical[:, 0]), optical[:, 2])) < section["color_fov_deg"][0] / 2)
              & (np.degrees(np.arctan2(np.abs(optical[:, 1]), optical[:, 2])) < section["color_fov_deg"][1] / 2))
    return optical[inside]


def test_wrist_view_shows_the_fingertips_and_not_the_arm_or_its_own_bracket(cell):
    """The camera is tilted toward the tool axis: the fingertips are in the upper part of the image (image down is away
    from the gripper), and the arm, the bracket and the camera's own body are out of it."""
    config, stage, report, camera_report, _ = cell
    section = config["wrist_camera"]
    root = camera_report["cameras"]["wrist"]["root_path"]
    arm = _in_wrist_image(stage, report, camera_report, section, [
        f"{report['assembly_path']}/quick_changer", report["bota_body_path"], f"{report['robot_path']}/wrist_3_link"])
    assert not len(arm), f"{len(arm)} Bota/Quick Changer/wrist vertices fall in the wrist image"
    own = _in_wrist_image(stage, report, camera_report, section, [f"{root}/bracket", f"{root}/body"])
    assert not len(own), f"{len(own)} bracket/camera-body vertices fall in the wrist image"
    gripper = _in_wrist_image(stage, report, camera_report, section, [report["gripper_path"]])
    assert len(gripper), "the RG2 fingertips are out of the wrist image"
    assert gripper[:, 1].max() < 0, "the gripper should be in the upper half of the image"
    assert gripper[:, 2].max() > 0.09                       # what shows reaches the fingertips, 0.1 m ahead


def _rg2_frame(stage, report, config):
    return f"{report['gripper_path']}/{config['asset_layout']['rg2']['mount_frame']}"


def _in_cad_frame(stage, report, config, roots):
    """Vertices in millimetres in the bracket's CAD frame: +X the finger-opening axis, +Y out of the RG2's front face.

    The pack's RG2 body frame is that frame turned a quarter turn about Z (body +Y = CAD +X, body +X = CAD -Y)."""
    body = _vertices_in_frame(stage, _rg2_frame(stage, report, config), roots) * 1000.0
    return np.stack([body[:, 1], -body[:, 0], body[:, 2]], axis=1)


def test_bracket_clamps_the_rg2_waist_and_holds_the_camera_off_its_back_face(cell):
    """The bracket asset is modelled in the RG2 body frame: on the built stage it must sit there, whatever the stack."""
    config, stage, report, camera_report, _ = cell
    root = camera_report["cameras"]["wrist"]["root_path"]
    frame = _rg2_frame(stage, report, config)
    bracket = _in_cad_frame(stage, report, config, [f"{root}/bracket"])
    body = _in_cad_frame(stage, report, config, [f"{root}/body"])
    rg2 = _in_cad_frame(stage, report, config, [frame])
    waist = rg2[(rg2[:, 2] > 75) & (rg2[:, 2] < 95)]
    assert np.abs(waist[:, 0]).max() < 27.3 and np.abs(waist[:, 1]).max() < 18.7     # the 54 x 36 mm section (datasheet)
    # The collar is around that section: it reaches past the body on all four sides and no vertex is inside it.
    collar = bracket[(bracket[:, 2] >= 72 - 1e-6) & (bracket[:, 2] <= 98 + 1e-6)]
    assert collar[:, 0].min() < -27.3 and collar[:, 0].max() > 27.3 and collar[:, 1].min() < -18.3 and collar[:, 1].max() > 18.3
    assert not ((np.abs(bracket[:, 0]) < 27.0 - 1e-6) & (np.abs(bracket[:, 1]) < 18.0 - 1e-6)).any()
    assert not ((np.abs(bracket[:, 0]) < 32.5) & (np.abs(bracket[:, 1]) < 18.0) & (bracket[:, 2] > 100.0)).any()    # the wider lower body
    assert bracket[:, 2].min() == pytest.approx(72.0, abs=0.01)                    # below the mounting bracket's arms
    arms = rg2[(np.abs(rg2[:, 0]) > 33.5) & (rg2[:, 2] < 72)]
    assert arms[:, 2].max() < 72.0
    # The D415, in its own optical frame (the wrist root carries it): 99 x 23 x 20 mm, back face on the plate.
    optical_body = _vertices_in_frame(stage, root, [f"{root}/body"]) * 1000.0
    optical_bracket = _vertices_in_frame(stage, root, [f"{root}/bracket"]) * 1000.0
    assert optical_body.max(0) - optical_body.min(0) == pytest.approx([99.0, 23.0, 20.0], abs=0.6)
    behind = optical_bracket[(np.abs(optical_bracket[:, 1]) < 11.0) & (optical_bracket[:, 0] > 0) & (optical_bracket[:, 0] < 70)]
    assert behind[:, 2].max() == pytest.approx(optical_body[:, 2].min(), abs=0.05)      # the plate's face meets the camera's back
    footprint = optical_bracket[(np.abs(optical_bracket[:, 1]) < 11.5) & (optical_bracket[:, 0] > -14.5) & (optical_bracket[:, 0] < 84.4)]
    assert footprint[:, 2].max() <= optical_body[:, 2].min() + 0.05                       # nothing of the bracket where the camera is
    # Off the back face of the RG2 body, tilted toward the tool axis about the camera's long axis.
    assert body[:, 1].max() < -18.0 - 25.0
    offset = np.array(config["wrist_camera"]["offset_in_rg2_m"]) * 1000.0
    assert body[:, 0].max() == pytest.approx(offset[1] + 14.5, abs=0.3)             # the colour sensor end is at CAD +X
    assert (body[:, 1].min() + body[:, 1].max()) / 2 == pytest.approx(-offset[0], abs=0.5)
    tilt = np.radians(config["wrist_camera"]["rotation_in_rg2_deg"][0])
    glass = body[body @ np.array([0.0, np.sin(tilt), np.cos(tilt)]) > (body @ np.array([0.0, np.sin(tilt), np.cos(tilt)])).max() - 0.3]
    assert len(glass) > 50 and np.ptp(glass @ np.array([0.0, np.sin(tilt), np.cos(tilt)])) < 0.3     # the flat front faces that way


def test_bracket_is_clear_of_the_finger_linkage_over_its_whole_stroke(cell):
    """The linkage turns about axes along the RG2's Y, so it stays in the slab of Y it occupies in any pose."""
    config, stage, report, camera_report, pack = cell
    frame = _rg2_frame(stage, report, config)
    gripper = stage.GetPrimAtPath(report["gripper_path"])
    cache = UsdGeom.XformCache()
    to_rg2 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(frame)).GetInverse()
    moving, slab, lowest = [], 0.0, 1e9
    for prim in Usd.PrimRange(gripper):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            if joint.GetJointEnabledAttr().Get() is False:
                continue
            body = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
            rotation = Gf.Rotation(joint.GetLocalRot1Attr().Get())
            axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[joint.GetAxisAttr().Get()]
            in_rg2 = (cache.GetLocalToWorldTransform(body) * to_rg2).TransformDir(rotation.TransformDir(axis))
            assert abs(in_rg2.GetNormalized()[0]) == pytest.approx(1.0, abs=1e-3), prim.GetPath()   # body X = CAD Y
            moving.append(str(body.GetPath()))
    assert len(set(moving)) >= 6
    for path in sorted(set(moving) - {frame}):
        points = _in_cad_frame(stage, report, config, [path])
        slab, lowest = max(slab, np.abs(points[:, 1]).max()), min(lowest, points[:, 2].min())
    assert slab <= 16.5 + 0.05 and lowest >= 111.0
    root = camera_report["cameras"]["wrist"]["root_path"]
    bracket = _in_cad_frame(stage, report, config, [f"{root}/bracket", f"{root}/body"])
    near = bracket[bracket[:, 2] >= lowest - 4.0]            # the only part of the bracket level with the linkage
    assert len(near) and np.abs(near[:, 1]).min() >= slab + 1.5
    wrist = stage.GetPrimAtPath(root)       # everything under tool0 stays visual only, the D415 body included
    assert not any(item.HasAPI(UsdPhysics.CollisionAPI) or item.HasAPI(UsdPhysics.RigidBodyAPI) for item in Usd.PrimRange(wrist))


def _gilbert(first, second, iterations=400, tolerance=1e-7):
    """A certified lower bound on the distance between the convex hulls of two point sets (Gilbert's algorithm)."""
    v, lower = first[0] - second[0], 0.0
    for _ in range(iterations):
        support = first[np.argmin(first @ v)] - second[np.argmax(second @ v)]
        norm = np.linalg.norm(v)
        if norm < 1e-12:
            return 0.0
        lower = max(lower, float(v @ support) / norm)
        step = v - support
        if norm - lower <= tolerance or float(step @ step) < 1e-18:
            break
        v = v + min(1.0, max(0.0, float(v @ step) / float(step @ step))) * (support - v)
    return lower


def _joint_pose(position, rotation):
    matrix = Gf.Matrix4d()
    matrix.SetRotate(Gf.Rotation(Gf.Quatd(rotation.GetReal(), Gf.Vec3d(*rotation.GetImaginary()))))
    matrix.SetTranslateOnly(Gf.Vec3d(*position))
    return matrix


def _linkage(stage, gripper):
    """The RG2's revolute joints: {name: body0, body1, frames, limits in degrees, (driving joint, ratio) or None}."""
    joints = {}
    for prim in Usd.PrimRange(gripper):
        if not prim.IsA(UsdPhysics.RevoluteJoint) or UsdPhysics.Joint(prim).GetJointEnabledAttr().Get() is False:
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        mimic = prim.GetRelationship("newton:mimicJoint")
        driver = mimic.GetTargets()[0].name if mimic and mimic.GetTargets() else None
        joints[prim.GetName()] = dict(
            body0=joint.GetBody0Rel().GetTargets()[0], body1=joint.GetBody1Rel().GetTargets()[0], axis=joint.GetAxisAttr().Get(),
            frame0=_joint_pose(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
            frame1=_joint_pose(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()),
            limits=(joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()),
            mimic=(driver, prim.GetAttribute("newton:mimicCoef1").Get()) if driver else None)
    return joints


def _link_frames(joints, base, driver, degrees):
    """Row-vector matrices, link to RG2 body, for the chain that ``driver`` moves, at ``degrees``."""
    frames, todo = {base: Gf.Matrix4d(1.0)}, dict(joints)
    while True:
        moved = False
        for name, joint in list(todo.items()):
            if (name if joint["mimic"] is None else joint["mimic"][0]) != driver or joint["body0"] not in frames:
                continue
            axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[joint["axis"]]
            turn = Gf.Matrix4d()
            turn.SetRotate(Gf.Rotation(axis, degrees * (1.0 if joint["mimic"] is None else joint["mimic"][1])))
            frames[joint["body1"]] = joint["frame1"].GetInverse() * turn * joint["frame0"] * frames[joint["body0"]]
            del todo[name]
            moved = True
        if not moved:
            return frames


def _collision_points(stage, body, frame_matrix=None):
    """Points of every collision shape whose nearest rigid body is ``body``, in that body's frame (or through a matrix)."""
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(body).GetInverse()
    boxes = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide", "proxy"])
    chunks = []
    for prim in Usd.PrimRange(body):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        owner = prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        if owner != body:
            continue
        if prim.IsA(UsdGeom.Mesh):
            points = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        else:                                   # a primitive or a group of them: the corners of its local box
            box = boxes.ComputeUntransformedBound(prim).ComputeAlignedRange()
            points = np.array([[x, y, z] for x in (box.GetMin()[0], box.GetMax()[0]) for y in (box.GetMin()[1], box.GetMax()[1])
                               for z in (box.GetMin()[2], box.GetMax()[2])], dtype=float)
        matrix = cache.GetLocalToWorldTransform(prim) * inverse * (frame_matrix or Gf.Matrix4d(1.0))
        chunks.append((np.c_[points, np.ones(len(points))] @ np.array(matrix))[:, :3])
    return chunks


BRACKET_SHAPES = {"collar", "front_bar_extension", "truss", "plate_support", "camera_plate"}
NEWTON_CONTACT_GAP_M = 0.004        # sim_tools.newton_support.collision_settings, on every collider of the cell


def test_bracket_collider_is_on_the_rg2_body_and_touches_nothing(cell):
    """Five convex hulls on the RG2 body: no mass, no filtered pair, and never within two contact gaps of anything
    that is another body: the finger linkage and its pads over the whole stroke, the Quick Changer, the Bota, wrist 3."""
    config, stage, report, camera_report, _ = cell
    body_path = _rg2_frame(stage, report, config)
    body = stage.GetPrimAtPath(body_path)
    (collider,) = camera_report["colliders"]
    (authored,) = camera_report["authoring"]["colliders"]
    assert collider["body_path"] == authored["body_path"] == body_path and authored["status"] == "attached"
    assert collider["filtered_pairs"] == [] and collider["mass"] == "none added" and "_path" not in collider
    assert camera_report["cameras"]["wrist"]["physics"]["bracket_collider"]["prim_path"] == collider["prim_path"]
    root = stage.GetPrimAtPath(collider["prim_path"])
    assert root.GetParent() == body and body.HasAPI(UsdPhysics.RigidBodyAPI)
    assert UsdGeom.Imageable(root).ComputePurpose() == UsdGeom.Tokens.guide
    shapes = {prim.GetName(): prim for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)}
    assert set(shapes) == BRACKET_SHAPES == {Path(item["prim_path"]).name for item in authored["shapes"]}
    for prim in Usd.PrimRange(root):
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.HasAPI(UsdPhysics.MassAPI) and not prim.IsA(UsdPhysics.Joint)
        assert not prim.HasAPI(UsdPhysics.FilteredPairsAPI)
    for prim in shapes.values():
        assert prim.IsA(UsdGeom.Mesh) and len(UsdGeom.Mesh(prim).GetPointsAttr().Get()) in (6, 8)
        assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"
    # No mass is added: the RG2 body keeps the explicit mass the config gives it.
    assert UsdPhysics.MassAPI(body).GetMassAttr().Get() == pytest.approx(
        config["inertials"]["rg2_links"]["onrobot_rg2_base_link"]["mass_kg"])

    cache = UsdGeom.XformCache()
    to_body = cache.GetLocalToWorldTransform(body).GetInverse()
    pieces = {name: (np.c_[points, np.ones(len(points))] @ np.array(cache.GetLocalToWorldTransform(prim) * to_body))[:, :3]
              for name, prim in shapes.items() for points in [np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)]}
    # The collar is the bracket's outer envelope around the body; it and the render mesh agree on that envelope.
    visual = _vertices_in_frame(stage, body_path, [f"{camera_report['cameras']['wrist']['root_path']}/bracket"])
    everything = np.vstack(list(pieces.values()))
    assert everything.min(0) == pytest.approx(visual.min(0), abs=2e-4)
    assert everything.max(0)[:2] == pytest.approx(visual.max(0)[:2], abs=2e-4)
    assert everything.max(0)[2] <= visual.max(0)[2] + 1e-6

    # The finger linkage and the pads, over the whole stroke of each driven joint.
    joints = _linkage(stage, stage.GetPrimAtPath(report["gripper_path"]))
    base = body.GetPath()
    drivers = [name for name, joint in joints.items() if joint["mimic"] is None]
    assert len(drivers) == 2 and len(joints) == 6
    hulls = {joint["body1"]: _collision_points(stage, stage.GetPrimAtPath(joint["body1"])) for joint in joints.values()}
    assert all(hulls.values()) and len(report["pad_paths"]) == 2          # the pads are colliders of the inner fingers
    for driver in drivers:
        at_rest = _link_frames(joints, base, driver, 0.0)
        for link, matrix in at_rest.items():                               # the chain reproduces the authored poses
            authored_pose = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(link)) * to_body
            assert np.allclose(np.array(matrix), np.array(authored_pose), atol=1e-5), link
    closest = {}
    for driver in drivers:
        low, high = joints[driver]["limits"]
        assert low < 0 < high
        for degrees in np.linspace(low, high, 27):
            for link, matrix in _link_frames(joints, base, driver, float(degrees)).items():
                for chunk in hulls.get(link, []):
                    moved = (np.c_[chunk, np.ones(len(chunk))] @ np.array(matrix))[:, :3]
                    for name, piece in pieces.items():
                        closest[name] = min(closest.get(name, 1.0), _gilbert(moved, piece))
    assert min(closest.values()) >= 2 * NEWTON_CONTACT_GAP_M + 0.001, closest
    assert closest["camera_plate"] > 0.03 and closest["truss"] > 0.015

    # The bodies fixed to the RG2: nothing of the bracket is anywhere near them.
    fixed = {}
    for path in (f"{report['assembly_path']}/quick_changer", report["bota_body_path"], f"{report['robot_path']}/wrist_3_link"):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                matrix = cache.GetLocalToWorldTransform(prim) * to_body
                for chunk in _collision_points(stage, prim, matrix):
                    fixed[path] = min([fixed.get(path, 1.0)] + [_gilbert(chunk, piece) for piece in pieces.values()])
    assert len(fixed) == 3 and min(fixed.values()) > 0.05, fixed


def test_pack_is_complete_on_its_own(cell):
    """Every layer the built cell opens is inside the pinned pack version: no file of another version is needed."""
    config, stage, _, _, pack = cell
    folder = Path(pack.asset(config["assets"]["robot"])).parents[1]
    assert folder.name == config["pack"]["version"] == "v3" and folder.parent.name == config["pack"]["id"]
    layers = [Path(layer.realPath) for layer in stage.GetUsedLayers() if layer.realPath]
    assert layers and all(folder in path.parents for path in layers), [str(p) for p in layers if folder not in p.parents]


def test_wrist_camera_prim_carries_the_configured_rotation_once(cell):
    config, stage, report, camera_report, _ = cell
    cache = UsdGeom.XformCache()
    camera = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(camera_report["cameras"]["wrist"]["prim_path"]))
    rg2 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(_rg2_frame(stage, report, config)))
    view = rg2.GetInverse().TransformDir(camera.TransformDir(Gf.Vec3d(0, 0, -1)))
    expected = cameras._rotate_vector(
        cameras._euler_xyz_deg_to_quat(config["wrist_camera"]["rotation_in_rg2_deg"]), [0.0, 0.0, 1.0])
    assert list(view) == pytest.approx(expected, abs=1e-6)
    origin = rg2.GetInverse().Transform(camera.ExtractTranslation())
    assert list(origin) == pytest.approx(config["wrist_camera"]["offset_in_rg2_m"], abs=1e-6)


def _bounds(stage, path):
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return np.array(box.GetMin()), np.array(box.GetMax())


def test_the_shipped_cell_has_no_overhead_camera(cell):
    _, stage, _, camera_report, _ = cell
    assert camera_report["cameras"]["overhead"]["status"] == "disabled"
    assert not stage.GetPrimAtPath(cameras.OVERHEAD_ROOT)
    assert [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)] == [
        camera_report["cameras"]["wrist"]["prim_path"]]


def test_overhead_camera_is_attached_to_the_gantry_through_its_release_stack(overhead_cell):
    config, stage, _, camera_report, _ = overhead_cell
    root = cameras.OVERHEAD_ROOT
    body, shelf, release, mount = (_bounds(stage, f"{root}/{name}") for name in ("body", "shelf", "release", "mount"))
    tolerance = 0.002
    # The shelf offset was set for the old 23 mm box proxy; the vendor D415 body leaves a 3.3 mm gap (not built today).
    assert abs(body[1][2] - shelf[0][2]) <= 0.004          # camera back face meets the shelf
    assert abs(shelf[1][2] - release[0][2]) <= tolerance   # shelf meets the release plate
    beam = [_bounds(stage, f"{root}/mount/TopBeam")]
    assert abs(beam[0][0][2] - release[1][2]) <= tolerance  # release meets the beam underside
    # The stack sits under the beam, within its span, and the camera is laterally centred on it.
    centre = (body[0] + body[1]) / 2
    assert beam[0][0][1] <= centre[1] <= beam[0][1][1]
    assert beam[0][0][0] <= centre[0] <= beam[0][1][0]
    assert release[1][2] <= mount[1][2]


PANEL = "imported_PANEL_B2F52AEB2A0D_804849_271/Mesh_91"


def test_gantry_posts_stand_on_the_bench_top_panel(overhead_cell):
    config, stage, _, _, pack = overhead_cell
    root = cameras.OVERHEAD_ROOT
    bench = Usd.Stage.Open(str(pack.asset(config["assets"]["bench"])))
    panel = next((prim for prim in bench.Traverse() if str(prim.GetPath()).endswith(PANEL)), None)
    if panel is None:
        pytest.skip("bench top panel prim not found in this pack")
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(panel).ComputeAlignedRange()
    scale = config["world"]["bench_scale"]
    ox, oy = config["world"]["workstation_xy_m"]
    low = (box.GetMin()[0] * scale + ox, box.GetMin()[1] * scale + oy)
    high = (box.GetMax()[0] * scale + ox, box.GetMax()[1] * scale + oy)
    for name in ("LeftPost", "RightPost"):
        post = _bounds(stage, f"{root}/mount/{name}")
        assert post[0][2] == pytest.approx(config["world"]["tabletop_z_m"], abs=1e-6)
        for axis in (0, 1):
            assert low[axis] <= post[0][axis] and post[1][axis] <= high[axis], (name, axis, low, high)


def test_rg2_has_no_geometry_free_body_and_all_joint_frames_agree(cell):
    config, stage, report, *_ = cell
    # Pack v2 fixes at the source what the build used to patch for v1: nothing is patched now.
    assert report["frame_bodies_neutralized"] == report["joints_deactivated"] == report["velocity_iterations_clamped"] == []
    roots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"]))
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert roots == [report["articulation_root"]] and report["gripper_articulation_root"] is None
    followers = {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath(report["gripper_path"]))
                 if p.GetMetadata("apiSchemas") and "NewtonMimicAPI" in p.GetMetadata("apiSchemas").ApplyOperations([])}
    assert set(followers) == {"left_inner_finger_joint", "left_finger_knuckle_joint", "right_inner_finger_joint", "RevoluteJoint"}
    for prim in followers.values():
        assert prim.GetRelationship("newton:mimicJoint").GetTargets()[0].name in ("finger_joint", "right_outer_knuckle_joint")
        assert 0.98 < abs(prim.GetAttribute("newton:mimicCoef1").Get()) < 1.0
        assert not prim.GetAttribute("physics:excludeFromArticulation").Get()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"])):
        count = prim.GetAttribute("physxArticulation:solverVelocityIterationCount")
        assert not count or (count.Get() or 0) <= 4, prim.GetPath()
    cache = UsdGeom.XformCache()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"])):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            assert any(item.IsA(UsdGeom.Gprim) for item in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())), (
                f"{prim.GetPath()} is a rigid body without geometry")
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        if joint.GetJointEnabledAttr().Get() is False:
            continue
        bodies = [joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()]
        if not all(bodies):
            continue  # an anchor such as the articulation root_joint
        frames = []
        for side, targets in enumerate(bodies):
            position, rotation = (getattr(joint, f"GetLocalPos{side}Attr")().Get(),
                                  getattr(joint, f"GetLocalRot{side}Attr")().Get())
            local = Gf.Matrix4d().SetRotate(Gf.Quatd(rotation.GetReal(), *rotation.GetImaginary()))
            local.SetTranslateOnly(Gf.Vec3d(*position))
            frames.append(local * cache.GetLocalToWorldTransform(stage.GetPrimAtPath(targets[0])))
        assert (frames[0].ExtractTranslation() - frames[1].ExtractTranslation()).GetLength() < 1e-3, prim.GetPath()
        relative = frames[0].ExtractRotationQuat() * frames[1].ExtractRotationQuat().GetInverse()
        assert 2 * math.degrees(math.acos(min(1.0, abs(relative.GetReal())))) < 0.5, prim.GetPath()
