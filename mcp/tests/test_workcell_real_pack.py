"""Geometry checks against the real external pack; skipped when the pack is not synced.

CPU only (no Kit, no GPU): the real robot and camera builders author an in-memory stage
and the checks read the composed USD. They guard the wrist view, the overhead mount
attachment, and the RG2 mass/joint warnings that only the real pack can show.
"""

import importlib.util
import json
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


def _cell(overhead, bota=False):
    config, _ = hardware_config.load_config()
    config["overhead_camera"]["enabled"] = overhead
    config["bota"]["mounted"] = bota                      # the one switch; nothing else is edited
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
def bota_cell():
    """The cell with the Bota switched on: the sensor and its two adapter plates between the flange and the Quick Changer."""
    return _cell(False, bota=True)


def _above_the_rg2(report):
    """The bodies between the arm and the RG2, and wrist 3 itself."""
    return [f"{report['assembly_path']}/{role}" for role in report["stack_order"] if role != "rg2"] + [f"{report['robot_path']}/wrist_3_link"]


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
    arm = _in_wrist_image(stage, report, camera_report, section, _above_the_rg2(report))
    assert not len(arm), f"{len(arm)} Quick Changer/wrist vertices fall in the wrist image"
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
    # The real body (OnRobot's CAD) where the collar sits: the datasheet's 54 x 36 mm section, leaning one degree, so
    # its centre is 0.4 to 0.8 mm behind the tool axis. It must lie inside the collar's pocket (0.3 mm a side nominal).
    # (The render mesh has few vertices on these flat faces; exts/workcell/cad/README.md has the section as measured.)
    waist = rg2[(rg2[:, 2] > 72) & (rg2[:, 2] < 98)]
    pocket_back, pocket_front = -0.6 - 18.3, -0.6 + 18.3
    assert len(waist) and np.abs(waist[:, 0]).max() < 27.3
    assert pocket_back < waist[:, 1].min() and waist[:, 1].max() < pocket_front
    ends = rg2[(rg2[:, 2] > 71) & (rg2[:, 2] < 104) & (np.abs(rg2[:, 0]) < 27.3)]
    assert pocket_back < ends[:, 1].min() and ends[:, 1].max() < pocket_front
    assert ends[:, 1].max() + ends[:, 1].min() < -0.5                                # the section sits behind the tool axis
    # The collar is around that section: it reaches past the body on all four sides and no vertex is inside the pocket.
    collar = bracket[(bracket[:, 2] >= 72 - 1e-6) & (bracket[:, 2] <= 98 + 1e-6)]
    assert collar[:, 0].min() < -27.3 and collar[:, 0].max() > 27.3 and collar[:, 1].min() < -18.9 and collar[:, 1].max() > 17.7
    assert not ((np.abs(bracket[:, 0]) < 27.3 - 1e-6) & (bracket[:, 1] > pocket_back + 1e-6) & (bracket[:, 1] < pocket_front - 1e-6)).any()
    # Lower down the body is wider (65 mm from Z = 120) and its front face is at 17.1 mm or less: the bar that runs down
    # the front face stays in front of it, and nothing else of the bracket is level with it.
    lower = rg2[(rg2[:, 2] > 104) & (rg2[:, 2] < 147) & (np.abs(rg2[:, 0]) < 32.6)]
    assert lower[:, 1].max() < 17.15 and lower[:, 1].min() > -19.65
    assert not ((np.abs(bracket[:, 0]) < 32.5) & (bracket[:, 1] > -19.6) & (bracket[:, 1] < 17.1) & (bracket[:, 2] > 104.0)).any()
    assert bracket[:, 2].min() == pytest.approx(72.0, abs=0.01)                    # below the mounting bracket's arms
    arms = rg2[(np.abs(rg2[:, 0]) > 27.5) & (rg2[:, 2] < 80)]
    assert 70.0 < arms[:, 2].max() < 72.0                                          # the arms end at 70.7 mm in OnRobot's CAD
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
    moving, slab, lowest, front, back = [], 0.0, 1e9, -1e9, 1e9
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
        front, back = max(front, points[:, 1].max()), min(back, points[:, 1].min())
    assert slab <= 16.7 and lowest >= 111.0
    assert front == pytest.approx(13.9, abs=0.1) and back == pytest.approx(-16.6, abs=0.1)     # not centred on the tool axis
    root = camera_report["cameras"]["wrist"]["root_path"]
    bracket = _in_cad_frame(stage, report, config, [f"{root}/bracket", f"{root}/body"])
    near = bracket[bracket[:, 2] >= lowest - 4.0]            # the only part of the bracket level with the linkage
    ahead, behind = near[near[:, 1] > 0], near[near[:, 1] < 0]
    assert len(ahead) and len(behind)
    assert ahead[:, 1].min() >= front + 1.5 and behind[:, 1].max() <= back - 1.5
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
    """Five convex hulls on the RG2 body: no filtered pair, and never within two contact gaps of anything that is
    another body: the finger linkage and its fingertips over the whole stroke, the Quick Changer, wrist 3."""
    config, stage, report, camera_report, _ = cell
    body_path = _rg2_frame(stage, report, config)
    body = stage.GetPrimAtPath(body_path)
    (collider,) = camera_report["colliders"]
    (authored,) = camera_report["authoring"]["colliders"]
    assert collider["body_path"] == authored["body_path"] == body_path and authored["status"] == "attached"
    assert collider["filtered_pairs"] == [] and "_path" not in collider
    assert collider["mass"] == "on the RG2 body, from wrist_camera.bracket_mass_kg"
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
    # The shapes add no mass by themselves: the body has its own explicit mass plus the bracket's and the camera's.
    assert UsdPhysics.MassAPI(body).GetMassAttr().Get() == pytest.approx(
        config["inertials"]["rg2_links"]["onrobot_rg2_base_link"]["mass_kg"] + config["wrist_camera"]["bracket_mass_kg"]
        + config["wrist_camera"]["mass_kg"])

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
    assert drivers == ["finger_joint"] and len(joints) == 6                  # one motor: every other joint follows it
    hulls = {joint["body1"]: _collision_points(stage, stage.GetPrimAtPath(joint["body1"])) for joint in joints.values()}
    assert all(hulls.values()) and report["pad_paths"] == []              # no add-on pads: the fingertips are part of the fingers
    for name in ("left_inner_finger", "right_inner_finger"):                # each finger: its carrier and its fingertip
        assert len(hulls[body.GetPath().GetParentPath().AppendChild(name)]) == 2
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
    for path in _above_the_rg2(report):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                matrix = cache.GetLocalToWorldTransform(prim) * to_body
                for chunk in _collision_points(stage, prim, matrix):
                    fixed[path] = min([fixed.get(path, 1.0)] + [_gilbert(chunk, piece) for piece in pieces.values()])
    assert len(fixed) == 2 and min(fixed.values()) > 0.05, fixed          # the Quick Changer and wrist 3


ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
POSES_DEG = {"as built (every joint at zero)": (0, 0, 0, 0, 0, 0), "tool down": (0, -80, 100, -110, -90, 0)}


def _posed_bodies(stage, assembly, degrees, rail_m=0.0):
    """Row-vector world matrices of the assembly's rigid bodies with the arm's joints at ``degrees`` and the rail at
    ``rail_m`` (every other joint at zero), by walking the joints from the bodies that nothing moves."""
    cache = UsdGeom.XformCache()
    angles = dict(zip(ARM_JOINTS, degrees))
    joints, bodies = [], [prim for prim in Usd.PrimRange(assembly) if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    for prim in Usd.PrimRange(assembly):
        if not prim.IsA(UsdPhysics.Joint) or UsdPhysics.Joint(prim).GetJointEnabledAttr().Get() is False:
            continue
        joint = UsdPhysics.Joint(prim)
        ends = [joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()]
        if not all(ends) or not all(stage.GetPrimAtPath(end[0]).HasAPI(UsdPhysics.RigidBodyAPI) for end in ends):
            continue                                     # an anchor to the world (no body, or a static frame)
        turn = Gf.Matrix4d(1.0)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()]
            turn.SetRotate(Gf.Rotation(axis, float(angles.get(prim.GetName(), 0.0))))
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get()]
            turn.SetTranslate(axis * float(rail_m))
        joints.append((ends[0][0], ends[1][0], _joint_pose(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                       _joint_pose(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()), turn))
    children = {child for _, child, *_ in joints}
    world = {prim.GetPath(): cache.GetLocalToWorldTransform(prim) for prim in bodies if prim.GetPath() not in children}
    while True:
        moved = False
        for parent, child, frame0, frame1, turn in joints:
            if parent in world and child not in world:
                world[child] = frame1.GetInverse() * turn * frame0 * world[parent]
                moved = True
        if not moved:
            break
    assert set(world) == {prim.GetPath() for prim in bodies}
    return world


def _static_collision_points(stage, root, meshes=False):
    """World points of every collision shape under ``root``: a mesh's points, or the corners of a primitive's box.
    With ``meshes``, every mesh counts, collider or not (the base plate is render geometry only)."""
    cache = UsdGeom.XformCache()
    boxes = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide", "proxy"])
    chunks = {}
    for prim in Usd.PrimRange(root):
        if not (prim.HasAPI(UsdPhysics.CollisionAPI) or (meshes and prim.IsA(UsdGeom.Mesh))):
            continue
        if prim.IsA(UsdGeom.Mesh):
            points = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        else:
            box = boxes.ComputeUntransformedBound(prim).ComputeAlignedRange()
            points = np.array([[x, y, z] for x in (box.GetMin()[0], box.GetMax()[0]) for y in (box.GetMin()[1], box.GetMax()[1])
                               for z in (box.GetMin()[2], box.GetMax()[2])], dtype=float)
        chunks[str(prim.GetPath())] = (np.c_[points, np.ones(len(points))] @ np.array(cache.GetLocalToWorldTransform(prim)))[:, :3]
    return chunks


RAIL_ENDS_M = {"rail at its default": 0.0, "rail at its -X end": -0.635, "rail at its +X end": 0.635}


def bracket_clearances(cell, rail_m=0.0):
    """{pose: {shape: {"bench" | "robot": (metres, nearest collider)}}} for the five bracket collision shapes and the
    D415 body box, against every collider of the bench (which holds the plates the robot stands on) and every robot
    body except the RG2 body the bracket belongs to."""
    config, stage, report, camera_report, pack = cell
    cache = UsdGeom.XformCache()
    assembly = stage.GetPrimAtPath(report["assembly_path"])
    body = stage.GetPrimAtPath(_rg2_frame(stage, report, config))
    to_body = cache.GetLocalToWorldTransform(body).GetInverse()
    root = stage.GetPrimAtPath(camera_report["colliders"][0]["prim_path"])
    shapes = {prim.GetName(): (np.c_[points, np.ones(len(points))] @ np.array(cache.GetLocalToWorldTransform(prim) * to_body))[:, :3]
              for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)
              for points in [np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)]}
    wrist = camera_report["cameras"]["wrist"]["root_path"]
    optical = _vertices_in_frame(stage, wrist, [f"{wrist}/body"])                       # the D415 body: no collider, checked anyway
    corners = np.array([[x, y, z] for x in (optical[:, 0].min(), optical[:, 0].max()) for y in (optical[:, 1].min(), optical[:, 1].max())
                        for z in (optical[:, 2].min(), optical[:, 2].max())])
    wrist_to_body = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(wrist)) * to_body
    shapes["d415_body (no collider)"] = (np.c_[corners, np.ones(8)] @ np.array(wrist_to_body))[:, :3]

    if not stage.GetPrimAtPath("/World/Bench"):                                          # placed as scene.py places it
        bench = UsdGeom.Xform.Define(stage, "/World/Bench")
        bench.GetPrim().GetReferences().AddReference(str(pack.asset(config["assets"]["bench"])))
        xy, scale = config["world"]["workstation_xy_m"], config["world"]["bench_scale"]
        bench.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], -config["world"]["bench_asset_top_z_m"]))
        bench.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    assert report["base_plate_path"] is None                                             # no proxy plate: the bench has the real ones
    static = {"bench": _static_collision_points(stage, stage.GetPrimAtPath("/World/Bench"))}
    assert len(static["bench"]) > 20
    links = {prim.GetPath(): _collision_points(stage, prim) for prim in Usd.PrimRange(assembly)
             if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim != body}
    assert sum(1 for chunks in links.values() if chunks) >= 11                          # the arm, the stack and the fingers

    result = {}
    for pose, degrees in POSES_DEG.items():
        world = _posed_bodies(stage, assembly, degrees, rail_m)
        if not any(degrees) and not rail_m:
            for path, matrix in world.items():                                           # the walk reproduces the built stage
                assert np.allclose(np.array(matrix), np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))), atol=1e-5), path
        placed = {name: (np.c_[points, np.ones(len(points))] @ np.array(world[body.GetPath()]))[:, :3] for name, points in shapes.items()}
        others = {"bench": static["bench"],
                  "robot": {f"{path.name}#{index}": (np.c_[chunk, np.ones(len(chunk))] @ np.array(world[path]))[:, :3]
                            for path, chunks in links.items() for index, chunk in enumerate(chunks)}}
        result[pose] = {}
        for name, points in placed.items():
            low, high = points.min(0), points.max(0)
            result[pose][name] = {}
            for kind, colliders in others.items():
                best = (1.0, None)
                for label, other in colliders.items():
                    gap = np.maximum(0.0, np.maximum(other.min(0) - high, low - other.max(0)))
                    if np.linalg.norm(gap) >= best[0]:
                        continue                                                         # its bounding box is already further
                    distance = _gilbert(points, other)
                    if distance < best[0]:
                        best = (distance, label)
                result[pose][name][kind] = best
    return result


@pytest.mark.parametrize("which", ["cell", "bota_cell"])
def test_bracket_and_camera_clear_the_bench_the_base_and_the_arm_at_the_built_pose_and_tool_down(which, request):
    """The cell is built with every joint at zero: the arm lies stretched out, the tool axis 141 mm above the table top.
    Nothing of the bracket or the camera may be within two of Newton's 4 mm contact gaps of the bench (with the plates
    the robot stands on) or any other body of the robot there, or with the tool pointing down, with the shipped stack and with the longer
    one that has the Bota in it. (With the RG2 mounted half a turn round, the camera plate was inside the bench at
    the built pose and threw the arm.)"""
    clearances = bracket_clearances(request.getfixturevalue(which))
    assert set(clearances) == set(POSES_DEG)
    for pose, shapes in clearances.items():
        assert set(shapes) == BRACKET_SHAPES | {"d415_body (no collider)"}
        for name, kinds in shapes.items():
            for kind, (distance, nearest) in kinds.items():
                assert distance >= 2 * NEWTON_CONTACT_GAP_M, f"{name} is {distance * 1000:.1f} mm from {kind} ({nearest}) at {pose}"
    built = clearances["as built (every joint at zero)"]
    assert built["camera_plate"]["bench"][0] > 0.10 and built["d415_body (no collider)"]["bench"][0] > 0.10      # above the tool axis


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
    # One motor: finger_joint is the only driven joint; the other side's arm and the four inner joints follow it. The
    # CAD's linkage is an exact parallelogram, so every ratio is exactly 1 or -1.
    assert set(followers) == {"right_outer_knuckle_joint", "left_inner_finger_joint", "left_finger_knuckle_joint",
                              "right_inner_finger_joint", "RevoluteJoint"}
    for name, prim in followers.items():
        assert prim.GetRelationship("newton:mimicJoint").GetTargets()[0].name == "finger_joint"
        assert prim.GetAttribute("newton:mimicCoef1").Get() == (-1.0 if "inner_finger_joint" in name else 1.0)
        assert not prim.GetAttribute("physics:excludeFromArticulation").Get()
        assert not prim.HasAPI(UsdPhysics.DriveAPI, "angular")
        joint = UsdPhysics.RevoluteJoint(prim)
        assert math.isfinite(joint.GetLowerLimitAttr().Get()) and math.isfinite(joint.GetUpperLimitAttr().Get())   # PhysX needs them
    driver = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(report["gripper_path"])) if p.GetName() == "finger_joint")
    assert driver.HasAPI(UsdPhysics.DriveAPI, "angular")
    limits = [UsdPhysics.RevoluteJoint(driver).GetLowerLimitAttr().Get(), UsdPhysics.RevoluteJoint(driver).GetUpperLimitAttr().Get()]
    assert limits == pytest.approx(config["rg2"]["joint_limits_deg"], abs=1e-3)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"])):
        count = prim.GetAttribute("physxArticulation:solverVelocityIterationCount")
        assert not count or (count.Get() or 0) <= 4, prim.GetPath()
    cache = UsdGeom.XformCache()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"])):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()) == report["rail"]["rail_link_path"]:
            mass = UsdPhysics.MassAPI(prim)      # the articulation's fixed root: no shape, so its mass properties are explicit
            assert mass.GetMassAttr().Get() > 0 and all(value > 0 for value in mass.GetDiagonalInertiaAttr().Get())
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
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


def _manifest(pack, path):
    return next(record for record in pack.manifest["files"] if record["path"] == path)


def test_the_default_cell_has_no_bota_and_one_switch_builds_it(cell, bota_cell):
    """The arm as it is: flange, Quick Changer, RG2. bota.mounted alone adds the sensor and both plates and moves the
    RG2, the bracket and the camera 48 mm down the tool axis."""
    config, stage, report, camera_report, _ = cell
    assert report["stack_order"] == ["quick_changer", "rg2"] and report["bota"] is None and report["proxies"] == []
    assert [str(prim.GetPath()) for prim in stage.Traverse() if prim.GetName().lower().startswith("bota")] == []
    on_config, on_stage, on_report, on_cameras, _ = bota_cell
    assert on_report["stack_order"] == ["bota_robot_plate", "bota", "bota_tool_plate", "quick_changer", "rg2"]
    sensor = on_stage.GetPrimAtPath(on_report["bota"]["body_path"])
    assert sensor.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.MassAPI(sensor).GetMassAttr().Get() == pytest.approx(0.113)
    assert len(on_report["proxies"]) == 2 and all("placeholder disc" in proxy["shape"] for proxy in on_report["proxies"])
    assert sum("adapter plate" in line for line in on_report["tool"]["unmeasured"]) == 2

    def in_flange(a_stage, a_report, path):
        cache = UsdGeom.XformCache()
        flange = cache.GetLocalToWorldTransform(a_stage.GetPrimAtPath(a_report["flange_path"]))
        return np.array((cache.GetLocalToWorldTransform(a_stage.GetPrimAtPath(path)) * flange.GetInverse()).ExtractTranslation())

    added = np.array([0.048, 0.0, 0.0])                                  # plate 8 mm + sensor 32 mm + plate 8 mm, along flange +X
    for path_off, path_on in ((_rg2_frame(stage, report, config), _rg2_frame(on_stage, on_report, on_config)),
                              (camera_report["cameras"]["wrist"]["prim_path"], on_cameras["cameras"]["wrist"]["prim_path"])):
        assert in_flange(on_stage, on_report, path_on) == pytest.approx(in_flange(stage, report, path_off) + added, abs=1e-6)
    # The RG2 body frame's origin is the rim of the Quick Changer's robot side: 2.5 mm on the robot's side of the flange.
    assert in_flange(stage, report, _rg2_frame(stage, report, config)) == pytest.approx([-0.0025, 0, 0], abs=1e-6)
    assert report["tool"]["tcp_in_tool0_m"] == pytest.approx([0, 0, 0.2161], abs=1e-4)
    assert on_report["tool"]["tcp_in_tool0_m"] == pytest.approx([0, 0, 0.2641], abs=1e-4)


def test_quick_changer_and_bota_are_the_vendors_cad_and_meet_the_rg2(cell, bota_cell):
    """Both parts come from the vendors' official STEP files (private pack, source and SHA-256 recorded, not
    redistributable). The Quick Changer's robot side is 13.6 mm high, as OnRobot's datasheet says, and ends where
    OnRobot's RG2 STEP puts it on the gripper."""
    config, stage, report, _, pack = cell
    for path, url, digest in (("quick_changer/quick_changer.usdc", "https://onrobot.com/storage/quick-changer-robot-side_v2_with_cable_holder.step",
                               "ac3cb44b90b27b172423400def5150a9af8b6e9338baf358f413118cbf5f38db"),
                              ("bota/bota.usdc", "https://shop.botasys.com/shop/rokubi-medusa-gen-0-series-kit-1877/document/180",
                               "90cb4bd0551a8090428c7fa470b3c3bcb1cf3c7e60e8363cc976efd8a82c7063"),
                              ("rg2/rg2.usdc", "https://onrobot.com/storage/303_rg2_tool.step",
                               "b3c41f38efafd9eab44c53f61d618e437f2fd0248cbc774c7384e838290ad184")):
        record = _manifest(pack, path)
        assert url in record["source"] and digest in record["source"] and record["redistributable"] is False
        assert "redistribution terms not known" in record["license"]
    folder = Path(pack.asset(config["assets"]["rg2"])).parents[1]
    assert not list(folder.rglob("*.step")) or {path.parent.name for path in folder.rglob("*.step")} == {"wrist_mount"}   # no vendor STEP in the pack
    changer = json.loads((folder / "quick_changer/quick_changer.json").read_text())
    assert changer["body_along_tool_axis_mm"] == pytest.approx([0.0, 13.6], abs=0.06) and changer["body_diameter_mm"] == pytest.approx(71.0, abs=0.3)
    assert "113292" in changer["left_out"] and "102697" in changer["parts"]
    sensor = json.loads((folder / "bota/bota.json").read_text())
    assert sensor["body_height_mm"] == pytest.approx(32.0, abs=0.06) and sensor["body_diameter_mm"] == pytest.approx(48.0, abs=0.1)
    gripper = json.loads((folder / "rg2/rg2.json").read_text())
    assert gripper["flange_plane_z_mm"] == pytest.approx(2.5, abs=0.01) and gripper["robot_side_height_mm"] == pytest.approx(13.6, abs=0.01)
    assert gripper["robot_side_tool_end_z_mm"] == pytest.approx(config["asset_layout"]["rg2"]["mount_pose"]["position_m"][2] * 1000.0, abs=0.01)
    assert config["quick_changer"]["standoff_m"] * 1000.0 == pytest.approx(gripper["robot_side_height_mm"], abs=0.01)
    # On the built stage the Quick Changer's collider runs from the flange plane to the RG2's inlet, 71 mm across.
    cache = UsdGeom.XformCache()
    flange = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(report["flange_path"]))
    body = stage.GetPrimAtPath(report["flange_load_body_path"])
    (hull,) = _collision_points(stage, body, cache.GetLocalToWorldTransform(body) * flange.GetInverse())
    assert [hull[:, 0].min(), hull[:, 0].max()] == pytest.approx([0.0, 0.0136], abs=6e-5)
    assert np.hypot(hull[:, 1], hull[:, 2]).max() == pytest.approx(0.0355, abs=1e-3)
    # The RG2 link no longer holds the robot side: nothing of it reaches the flange plane's side of the Quick Changer's rim.
    _, on_stage, on_report, _, _ = bota_cell
    assert on_stage.GetPrimAtPath(on_report["bota"]["body_path"] + "/visuals").IsA(UsdGeom.Mesh)


def test_the_weight_below_the_flange_is_the_configured_payload(cell, bota_cell):
    """What hangs from the flange in the simulation weighs what the payload table says, in both states: the bracket and
    the D415 are masses on the RG2 body, not weightless shapes. So the simulated flange reading at rest is the payload's
    weight, and with the Bota mounted its reading is everything below its tool-side face."""
    for config, stage, report, _, _ in (cell, bota_cell):
        payload = report["tool"]["payload"]
        below = set()
        frontier = [report["flange_load_body_path"]]
        joints = [UsdPhysics.Joint(prim) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint) and prim.IsActive()]
        while frontier:
            parent = frontier.pop()
            below.add(parent)
            for joint in joints:
                if [str(path) for path in joint.GetBody0Rel().GetTargets()] == [parent]:
                    frontier += [str(path) for path in joint.GetBody1Rel().GetTargets() if str(path) not in below]
        masses = {path: UsdPhysics.MassAPI(stage.GetPrimAtPath(path)).GetMassAttr().Get() for path in below}
        assert all(mass and mass > 0 for mass in masses.values()), masses
        assert sum(masses.values()) == pytest.approx(payload["mass_kg"], abs=2e-6)
        assert payload["mass_kg"] == pytest.approx(1.251 if report["bota"] else 1.004, abs=1e-6)
        carried = {item["item"]: item["mass_kg"] for item in report["carried_on_rg2"]}
        assert carried == {"wrist camera bracket and its screws": 0.1, "Intel RealSense D415": 0.064}
        if report["bota"]:
            before = [report["assembly_path"] + "/bota_robot_plate", report["bota"]["body_path"]]
            assert sum(masses[path] for path in before) == pytest.approx(0.067 + 0.113)
            assert sum(masses.values()) - sum(masses[path] for path in before) == pytest.approx(1.004 + 0.067, abs=2e-6)   # below the Bota


def test_carried_centres_of_gravity_follow_the_bracket_model(cell):
    """The configured centre of the D415's body and of the bracket are the bracket model's, turned to the RG2 body
    frame: they change when the camera pose does, and this says so."""
    config, _, _, _, pack = cell
    report = json.loads((Path(pack.asset(config["assets"]["wrist_mount"])).parent / "bracket.json").read_text())
    x, y, z = report["camera"]["body_centre_mm"]
    assert config["wrist_camera"]["cog_in_rg2_m"] == pytest.approx([-y / 1000.0, x / 1000.0, z / 1000.0], abs=5e-4)
    volume = sum(piece["volume_cm3"] for piece in report["pieces"])
    centre = sum(piece["volume_cm3"] * np.array(piece["centre_of_volume_mm"]) for piece in report["pieces"]) / volume
    print_only = np.array([-centre[1], centre[0], centre[2]]) / 1000.0
    assert np.linalg.norm(np.array(config["wrist_camera"]["bracket_cog_in_rg2_m"]) - print_only) < 0.006      # the bolts pull it toward the body
    assert volume == pytest.approx(99.1, abs=1.0)


def test_the_robot_stands_on_the_rails_carriage_and_the_bench_is_what_the_photos_show(cell):
    """The bench asset is Vention's CAD of the lab's bench. Pack v3 splits it: the static bench, and the carriage of
    its linear axis with the robot plate, which the robot's base sits on, 80 mm above the table top. No proxy plate is
    built, and the cable carrier, which the photos of the real cell do not show, is left out."""
    config, stage, report, _, pack = cell
    bracket_clearances(cell)                                             # places the bench as the scene does
    assert config["base_plate"]["present"] is False and report["base_plate_path"] is None
    assert not stage.GetPrimAtPath(report["assembly_path"] + "/base_plate")
    cache = UsdGeom.XformCache()
    base = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(report["arm_base_link"])).ExtractTranslation()
    assert list(base) == pytest.approx([0.0, 0.0, 0.08], abs=1e-6)
    bench = [prim.GetName() for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Bench"))]
    assert not [name for name in bench if name.startswith(("ST_RB_001_0002", "MO_LM_027_0001", "MO_DC_001", "ST_GP_001_0004_325"))]
    assert any(name.startswith("ST_EXT_007_0585_1530") for name in bench) and "MO_LM_024_XXXX_1720_133_belt" in bench   # the rail itself stays
    carriage = stage.GetPrimAtPath(report["rail"]["carriage_path"])
    names = [prim.GetName() for prim in Usd.PrimRange(carriage)]
    assert any(name.startswith("MO_LM_025_0004") for name in names) and any(name.startswith("Cut_Extrude2") for name in names)
    boxes = _static_collision_points(stage, carriage)
    plates = [chunk for chunk in boxes.values()
              if np.ptp(chunk[:, 0]) == pytest.approx(0.18, abs=2e-3) and np.ptp(chunk[:, 1]) == pytest.approx(0.18, abs=2e-3)]
    (plate,) = plates                                                    # the robot plate's collision box, 180 mm square
    assert plate[:, 2].max() == pytest.approx(0.08, abs=5e-4) and plate[:, 2].min() == pytest.approx(0.058, abs=5e-4)
    assert np.abs(plate[:, :2].mean(0)).max() < 0.002                    # centred under the robot's axis
    gantry = [chunk for chunk in boxes.values() if np.ptp(chunk[:, 0]) == pytest.approx(0.268, abs=2e-3) and np.ptp(chunk[:, 1]) == pytest.approx(0.268, abs=2e-3)]
    assert gantry and min(chunk[:, 2].min() for chunk in gantry) == pytest.approx(0.0375, abs=1.5e-3)      # 2 mm above the rail's top
    assert max(chunk[:, 2].max() for chunk in gantry) == pytest.approx(0.058, abs=1.5e-3)
    folder = Path(pack.asset(config["assets"]["bench"])).parent
    assert {path.name for path in folder.iterdir()} == {"bench.usda", "bench_cell.usda", "bench_full.usda", "bench_render.usdc", "bench_collision.usdc"}


def _sim_tools_ik():
    spec = importlib.util.spec_from_file_location("realpack_ik", PACKAGE.parents[1] / "sim-tools/sim_tools/ik.py")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as error:                                         # its package imports are not needed for the chain
        pytest.skip(f"sim_tools.ik cannot be loaded here: {error}")
    return module


def test_the_rail_is_a_seventh_axis_and_callers_see_exactly_this(cell):
    """What changes for a caller. The arm's root joint keeps its path and now fixes a rail link to the world; a
    prismatic rail_joint (metres, zero at the built pose, +X) carries the carriage, and a fixed joint holds the base on
    it. The arm tools' IK, which reads the chain from these prims, still starts at the arm's base link: an unchanged
    call solves the six arm joints from wherever the rail has put the base."""
    config, stage, report, _, _ = cell
    rail = report["rail"]
    assert report["articulation_root"] == report["robot_path"] + "/root_joint"       # the path callers already use
    root = UsdPhysics.Joint(stage.GetPrimAtPath(report["articulation_root"]))
    assert root.GetPrim().HasAPI(UsdPhysics.ArticulationRootAPI)
    assert [str(path) for path in root.GetBody1Rel().GetTargets()] == [rail["rail_link_path"]]
    joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(rail["joint_path"]))
    assert joint and joint.GetPrim().GetName() == rail["joint"] == "rail_joint" and joint.GetAxisAttr().Get() == "X"
    assert [joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()] == pytest.approx([-0.635, 0.635])
    assert [str(p) for p in joint.GetBody0Rel().GetTargets()] == [rail["rail_link_path"]]          # parent first: Newton refuses the reverse
    assert [str(p) for p in joint.GetBody1Rel().GetTargets()] == [rail["carriage_path"]]
    drive = UsdPhysics.DriveAPI(joint.GetPrim(), "linear")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(config["rail"]["drive"]["physx"]["stiffness_n_per_m"])
    assert drive.GetMaxForceAttr().Get() == pytest.approx(1240.0) and drive.GetTargetPositionAttr().Get() == 0.0
    assert joint.GetPrim().GetAttribute("physxJoint:maxJointVelocity").Get() == pytest.approx(0.25)
    cache = UsdGeom.XformCache()
    for path in (rail["rail_link_path"], rail["carriage_path"]):                    # the world's axes, at the robot's base
        matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        assert np.array(matrix) == pytest.approx(np.array(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, 0.08))), abs=1e-6)
    carriage = stage.GetPrimAtPath(rail["carriage_path"])
    assert carriage.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.MassAPI(carriage).GetMassAttr().Get() == pytest.approx(4.6)
    assert not _collision_points(stage, stage.GetPrimAtPath(rail["rail_link_path"]))               # the rail itself is the static bench
    enabled = [prim for prim in Usd.PrimRange(carriage) if prim.HasAPI(UsdPhysics.CollisionAPI)
               and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False]
    assert 0 < len(enabled) < sum(1 for prim in Usd.PrimRange(carriage) if prim.HasAPI(UsdPhysics.CollisionAPI))
    lowest = min(chunk[:, 2].min() for prim in enabled for chunk in [np.array(_static_collision_points(stage, prim)[str(prim.GetPath())])])
    assert lowest == pytest.approx(0.0496, abs=1e-3)                     # the robot plate and the screws beside it: 14 mm above the rail
    mount = UsdPhysics.FixedJoint(stage.GetPrimAtPath(rail["carriage_mount_path"]))
    assert [str(p) for p in mount.GetBody0Rel().GetTargets()] == [rail["carriage_path"]]
    assert [str(p) for p in mount.GetBody1Rel().GetTargets()] == [report["arm_base_link"]]
    assert rail["limits_m"] == [-0.635, 0.635] and rail["real_axis"]["limits_mm"] == pytest.approx([0.0, 1270.0])
    assert rail["real_axis"]["at_default_mm"] == pytest.approx(635.0) and "metres" in rail["units"]

    ik = _sim_tools_ik()
    robot_prim = stage.GetPrimAtPath(report["robot_path"])                 # where the joint tools look: beside the root joint
    joints = [prim for prim in Usd.PrimRange(robot_prim) if prim.IsA(UsdPhysics.Joint)]
    # The arm tools start the chain at the one body a joint ties to nothing else. That is still the arm's base link:
    # the root joint names a static frame as its first body, and a disabled marker joint holds the base link alone.
    assert [str(p) for p in root.GetBody0Rel().GetTargets()] == [rail["rail_anchor_path"]]
    assert not stage.GetPrimAtPath(rail["rail_anchor_path"]).HasAPI(UsdPhysics.RigidBodyAPI)
    marker = UsdPhysics.Joint(stage.GetPrimAtPath(rail["arm_base_marker_path"]))
    assert marker.GetJointEnabledAttr().Get() is False and not marker.GetBody0Rel().GetTargets()
    assert [str(p) for p in marker.GetBody1Rel().GetTargets()] == [report["arm_base_link"]]
    assert not marker.GetExcludeFromArticulationAttr().Get()               # or the joint tools would not see it
    assert ik.find_base(joints) == report["arm_base_link"]
    wrist = report["robot_path"] + "/wrist_3_link"
    arm = ik.build_chain(stage, joints, ik.find_base(joints), wrist)       # an unchanged call: six joints from the base
    assert arm.names == list(ARM_JOINTS) and "prismatic" not in arm.kinds
    here = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(wrist)).ExtractTranslation())
    assert arm.fk(np.zeros(6))[:3, 3] == pytest.approx(here, abs=1e-5)
    moved = ik.pose_matrix([0.3, 0.0, 0.08], ik.quat_to_matrix(0.0, 0.0, 0.0, 1.0))   # the base 0.3 m along the rail, as physics would report it
    shifted = ik.build_chain(stage, joints, report["arm_base_link"], wrist, moved)
    assert shifted.fk(np.zeros(6))[:3, 3] == pytest.approx(here + [0.3, 0, 0], abs=1e-5)


@pytest.mark.parametrize("where", ["rail at its -X end", "rail at its +X end"])
def test_bracket_and_camera_stay_clear_at_both_ends_of_the_rail(cell, where):
    """The guard distances of the built pose and tool down hold with the carriage at either end of its travel, where
    the arm is over the rail's end blocks and, at +X, its motor."""
    clearances = bracket_clearances(cell, RAIL_ENDS_M[where])
    for pose, shapes in clearances.items():
        for name, kinds in shapes.items():
            for kind, (distance, nearest) in kinds.items():
                assert distance >= 2 * NEWTON_CONTACT_GAP_M, f"{name} is {distance * 1000:.1f} mm from {kind} ({nearest}) at {pose}, {where}"


def test_nothing_of_the_robot_or_carriage_meets_the_bench_along_the_rail(cell):
    """With the tool down, no collider of the arm, the tool or the carriage's robot plate comes within two contact gaps
    of a bench collider anywhere along the travel."""
    config, stage, report, _, _ = cell
    bracket_clearances(cell)
    assembly = stage.GetPrimAtPath(report["assembly_path"])
    static = _static_collision_points(stage, stage.GetPrimAtPath("/World/Bench"))
    links = {}
    for prim in Usd.PrimRange(assembly):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            chunks = []
            for shape in Usd.PrimRange(prim):
                if shape.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(shape).GetCollisionEnabledAttr().Get() is not False:
                    owner = shape
                    while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                        owner = owner.GetParent()
                    if owner == prim:
                        chunks += [np.array(chunk) for chunk in _static_collision_points(stage, shape).values()]
            if chunks:
                links[prim.GetPath()] = chunks
    built = _posed_bodies(stage, assembly, (0, 0, 0, 0, 0, 0))
    worst = (1.0, None)
    for where, rail_m in RAIL_ENDS_M.items():
        world = _posed_bodies(stage, assembly, POSES_DEG["tool down"], rail_m)
        for path, chunks in links.items():
            move = np.array(built[path].GetInverse() * world[path])
            for chunk in chunks:
                points = (np.c_[chunk, np.ones(len(chunk))] @ move)[:, :3]
                low, high = points.min(0), points.max(0)
                for label, other in static.items():
                    bound = float(np.linalg.norm(np.maximum(0.0, np.maximum(other.min(0) - high, low - other.max(0)))))
                    if bound < worst[0]:             # the gap between bounding boxes never overstates the distance
                        distance = bound if bound >= 2 * NEWTON_CONTACT_GAP_M else max(bound, _gilbert(points, other))
                        if distance < worst[0]:
                            worst = (distance, f"{path.name} to {label.rsplit('/', 2)[-2]} with the {where}")
    assert worst[0] >= 2 * NEWTON_CONTACT_GAP_M, worst
