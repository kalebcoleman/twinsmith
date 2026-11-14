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


def test_wrist_view_is_clear_of_the_arm_the_bracket_and_the_camera_body(cell):
    """With the pose read from the photo (no tilt) the camera looks past the gripper: nothing of the robot is drawn.

    Whether the real image shows the fingertips is an open question for the lab: it fixes the bracket's tilt."""
    config, stage, report, camera_report, _ = cell
    section = config["wrist_camera"]
    root = camera_report["cameras"]["wrist"]["root_path"]
    arm = _in_wrist_image(stage, report, camera_report, section, [
        f"{report['assembly_path']}/quick_changer", report["bota_body_path"], f"{report['robot_path']}/wrist_3_link"])
    assert not len(arm), f"{len(arm)} Bota/Quick Changer/wrist vertices fall in the wrist image"
    own = _in_wrist_image(stage, report, camera_report, section, [f"{root}/bracket", f"{root}/body"])
    assert not len(own), f"{len(own)} bracket/camera-body vertices fall in the wrist image"
    gripper = _in_wrist_image(stage, report, camera_report, section, [report["gripper_path"]])
    # Image down is away from the gripper, so anything of it that shows is in the upper half.
    assert not len(gripper) or gripper[:, 1].max() < 0


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
    assert bracket[:, 2].min() == pytest.approx(72.0, abs=0.01)                    # below the mounting bracket's arms
    arms = rg2[(np.abs(rg2[:, 0]) > 33.5) & (rg2[:, 2] < 72)]
    assert arms[:, 2].max() < 72.0
    # The D415: long axis along X, back face on the plate, off the back face of the body.
    size = body.max(0) - body.min(0)
    assert size == pytest.approx([99.0, 23.0, 20.0], abs=0.6)
    plate = bracket[bracket[:, 1] < -40.0]
    assert body[:, 2].min() == pytest.approx(plate[:, 2].max(), abs=0.05)
    assert body[:, 1].max() < -18.0 - 30.0
    offset = np.array(config["wrist_camera"]["offset_in_rg2_m"]) * 1000.0
    assert body[:, 0].max() == pytest.approx(offset[1] + 14.5, abs=0.3)             # the colour sensor end is at CAD +X
    assert (body[:, 1].min() + body[:, 1].max()) / 2 == pytest.approx(-offset[0], abs=0.5)


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
    # The same holds for the collision approximation the pack carries (the builder removes it from the scene).
    asset = Usd.Stage.Open(str(pack.asset(config["assets"]["wrist_mount"])))
    colliders = [prim for prim in asset.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(colliders) == 9
    for prim in colliders:
        box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["guide"]).ComputeWorldBound(prim).ComputeAlignedRange()
        low, high = np.array(box.GetMin()) * 1000.0, np.array(box.GetMax()) * 1000.0       # body frame: X is CAD -Y
        assert high[2] < lowest or low[0] > slab or high[0] < -slab, prim.GetPath()
    wrist = stage.GetPrimAtPath(root)
    assert not any(item.HasAPI(UsdPhysics.CollisionAPI) or item.HasAPI(UsdPhysics.RigidBodyAPI) for item in Usd.PrimRange(wrist))


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
