"""sim.quick_start: real USD and the real resolver, with Kit's context and app faked."""

import asyncio
import hashlib
import json
import os
import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("pxr.Usd")
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts" / "sim-tools"))

from twinsmith.project_assets import AssetError, Broker  # noqa: E402
from twinsmith.scene_contracts import REGISTRY  # noqa: E402
from sim_tools import public_robots, quick_start as qs  # noqa: E402
from sim_tools.core import Scene  # noqa: E402

ROBOT_USDA = """#usda 1.0
(
    defaultPrim = "arm"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "arm" {
    def Xform "root_joint" (prepend apiSchemas = ["PhysicsArticulationRootAPI"]) {}
    def Xform "base_link" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}
    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
        def Xform "visuals" (instanceable = true; prepend references = </Mesh>) {}
    }
}
def Xform "Mesh" {
    def Cube "box" {}
}
"""


class Timeline:
    def __init__(self):
        self.playing = False

    def is_stopped(self):
        return not self.playing

    def is_playing(self):
        return self.playing

    def get_current_time(self):
        return 0.0


@pytest.fixture()
def kit(monkeypatch, tmp_path):
    """Fake omni.usd/omni.kit.app around a real stage; asset root and the simulator's assets root in tmp; no project file."""
    state = SimpleNamespace(stage=Usd.Stage.CreateInMemory(), new_stages=0)

    class Context:
        def get_stage(self):
            return state.stage

        async def new_stage_async(self):
            state.new_stages += 1
            state.stage = Usd.Stage.CreateInMemory()
            return True, ""

    class App:
        def is_app_ready(self):
            return True

        async def next_update_async(self):
            return None

    usd = types.ModuleType("omni.usd")
    usd.get_context = Context
    app_module = types.ModuleType("omni.kit.app")
    app_module.get_app = App
    monkeypatch.setitem(sys.modules, "omni", types.ModuleType("omni"))
    monkeypatch.setitem(sys.modules, "omni.usd", usd)
    monkeypatch.setitem(sys.modules, "omni.kit", types.ModuleType("omni.kit"))
    monkeypatch.setitem(sys.modules, "omni.kit.app", app_module)
    sys.modules["omni"].usd = usd
    sys.modules["omni"].kit = sys.modules["omni.kit"]
    sys.modules["omni.kit"].app = app_module
    store = tmp_path / "store"  # the asset root: packs as <id>/<version>/pack.yaml
    store.mkdir()
    # The simulator's own assets root, as a folder: the public names (ur5e included) come from here.
    nvidia = tmp_path / "nvidia-root"
    (nvidia / "Isaac/Robots/UniversalRobots/ur5e").mkdir(parents=True)
    (nvidia / "Isaac/Robots/UniversalRobots/ur5e/ur5e.usd").write_text(ROBOT_USDA)

    async def stat(url):
        return os.path.exists(url)

    monkeypatch.setattr(public_robots, "assets_root", lambda: str(nvidia))
    monkeypatch.setattr(public_robots, "client_stat", stat)
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(store))
    for name in ("TWINSMITH_PROJECT", "TWINSMITH_WORKSPACE"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    state.store = store
    state.nvidia = nvidia
    state.timeline = Timeline()
    state.scene = Scene(lambda: state.stage, state.timeline, output=tmp_path / "out")
    state.played = []

    async def control(args):
        state.played.append(args)
        state.timeline.playing = True

    state.ext = SimpleNamespace(scene=state.scene, _timeline_control=control)
    state.run = lambda **args: asyncio.run(qs.run(state.ext, args))

    def pack(pack_id, version, files):
        """A verifying pack in the asset root: ``files`` maps a path to its text."""
        directory = store / pack_id / version
        records = []
        for rel, text in sorted(files.items()):
            path = directory / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            records.append(dict(path=rel, sha256=hashlib.sha256(text.encode()).hexdigest(), source="test", license="test",
                                redistributable=False))
        (directory / "pack.yaml").write_text(json.dumps(dict(id=pack_id, version=version, files=records)))
        return directory

    state.pack = pack
    return state


def refused(kit, **args):
    with pytest.raises(AssetError) as error:
        kit.run(**args)
    return error.value


def test_default_needs_nothing_and_builds_a_working_scene(kit):
    original = kit.stage
    out = kit.run()
    stage = kit.stage
    assert stage is original and out["stage_switched"] is False  # an empty stage is reused
    assert stage.GetPrimAtPath(out["physics_scene"]).IsA(UsdPhysics.Scene)
    assert stage.GetPrimAtPath(out["ground"] + "/Geometry").IsA(UsdGeom.Mesh)
    plane = stage.GetPrimAtPath(out["ground"] + "/CollisionPlane")
    assert plane.IsA(UsdGeom.Plane) and plane.HasAPI(UsdPhysics.CollisionAPI)
    assert stage.GetPrimAtPath(out["camera"]).IsA(UsdGeom.Camera)
    assert stage.GetPrimAtPath(out["lights"][0]).IsA(UsdLux.DomeLight)
    assert out["session_id"] == kit.scene.session_id and out["stage_id"] == kit.scene.stage_id
    assert out["stopped"] is True and "robot" not in out and "ros" not in out and kit.played == []
    assert any("sim__camera__capture" in step for step in out["next_steps"])


def test_no_twin_wiring_is_built(kit):
    kit.run(robot="ur5e")
    assert not kit.stage.GetPrimAtPath("/Graph")
    assert not [p for p in kit.stage.Traverse() if "Subscribe" in p.GetTypeName() or "rg2" in p.GetName().lower()]


def test_camera_looks_at_the_scene(kit):
    kit.run()
    world = UsdGeom.Xformable(kit.stage.GetPrimAtPath(qs.CAMERA)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    eye = world.ExtractTranslation()
    assert list(eye) == pytest.approx(list(qs.EYE))
    forward = world.TransformDir((0, 0, -1)).GetNormalized()  # USD cameras look down -Z
    to_target = (type(eye)(*qs.EYE_TARGET) - eye).GetNormalized()
    assert forward * to_target == pytest.approx(1.0)


def test_camera_frames_the_robot_at_its_size(kit):
    kit.run(robot="ur5e")
    stage = kit.stage
    world = UsdGeom.Xformable(stage.GetPrimAtPath(qs.CAMERA)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        stage.GetPrimAtPath("/World/UR5e")).ComputeAlignedRange()
    to_center = (box.GetMidpoint() - world.ExtractTranslation())
    half_fov = math.atan(qs.HORIZONTAL_APERTURE / qs.CAPTURE_ASPECT / 2 / qs.FOCAL_LENGTH)
    # Aimed a little above the robot, to leave room for it to rise, with all of it still in view.
    off_axis = math.acos(world.TransformDir((0, 0, -1)).GetNormalized() * to_center.GetNormalized())
    assert 0 < off_axis
    assert off_axis + math.asin(box.GetSize().GetLength() / 2 / to_center.GetLength()) <= half_fov + 1e-6


def test_a_prim_with_no_bounds_keeps_the_default_aim(kit):
    kit.run()
    UsdGeom.Xform.Define(kit.stage, "/World/Nothing")
    assert qs.frame_camera_on(kit.stage, "/World/Nothing") is None


def test_edits_are_in_the_session_layer_only(kit):
    before = kit.stage.GetRootLayer().rootPrims
    kit.run(robot="ur5e")
    assert not list(before) and not list(kit.stage.GetRootLayer().rootPrims)
    assert kit.stage.GetSessionLayer().GetPrimAtPath("/World/UR5e")


def test_public_robot_reports_root_and_source_and_expands_visuals(kit):
    out = kit.run(robot="ur5e")
    assert out["robot"]["prim"] == "/World/UR5e" and out["robot"]["articulation_root"] == "/World/UR5e/root_joint"
    assert out["robot"]["expanded_visuals"] == 1
    assert out["assets"][0]["source"] == "nvidia_assets_root" and out["assets"][0]["location"].endswith("UniversalRobots/ur5e/ur5e.usd")
    assert any('path="/World/UR5e/root_joint"' in step for step in out["next_steps"])
    assert not kit.stage.GetPrimAtPath("/World/UR5e/link1/visuals").IsInstance()


@pytest.mark.parametrize("name,prim,path", [
    ("franka", "Franka", "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"),
    ("unitree_g1", "UnitreeG1", "Isaac/Robots/Unitree/G1/g1.usd"),
    ("ur5e", "UR5e", "Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"),
])
def test_the_public_names_still_resolve_through_the_simulators_assets_root(kit, name, prim, path):
    """franka, unitree_g1 and ur5e must place exactly as before: the file is found under the build's own root."""
    target = kit.nvidia / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(ROBOT_USDA)
    out = kit.run(robot=name)
    assert out["robot"]["prim"] == f"/World/{prim}" and kit.stage.GetPrimAtPath(f"/World/{prim}")
    report = out["assets"][0]
    assert report == dict(asset=path, source="nvidia_assets_root", location=str(target))


def test_the_warehouse_and_its_default_robot_do_not_touch_the_pack_model():
    """environment=warehouse builds plain USD in code; its default robot is carter_v1, a public name from the
    simulator's assets root. Neither goes through the pack reader, so removing the old resolver changes nothing for them."""
    root = Path(qs.__file__).parent
    for module in ("warehouse.py", "jenga.py", "quick_start.py", "public_robots.py"):
        source = (root / module).read_text()
        assert "twinsmith import assets" not in source and "asset_packs" not in source, module
    assert 'robot = "carter_v1" if environment == "warehouse" else "franka"' in (root / "quick_start.py").read_text()
    assert {"carter_v1", "franka"} <= set(public_robots.NVIDIA_ROBOTS)


def test_public_robot_the_simulators_root_lacks_is_refused_with_the_names(kit):
    (kit.nvidia / "Isaac/Robots/UniversalRobots/ur5e/ur5e.usd").unlink()
    error = refused(kit, robot="ur5e")
    assert error.code == "ROBOT_NOT_ON_THIS_BUILD" and "franka" in str(error)
    assert not kit.stage.GetPrimAtPath("/World")


def test_a_project_file_does_not_name_a_robot_or_a_folder(kit, tmp_path, monkeypatch):
    repo = tmp_path / "cloned"
    repo.mkdir()
    (repo / "twinsmith.json").write_text(json.dumps({"pack": "mine/v1"}))
    monkeypatch.chdir(repo)
    kit.pack("mine", "v1", {"arm.usd": ROBOT_USDA})
    out = kit.run(robot="ur5e")  # a public name is still the simulator's file, never the project's
    assert out["assets"][0]["source"] == "nvidia_assets_root"


def test_pack_file_is_verified_and_named(kit):
    pack = kit.pack("mine", "v1", {"arm.usd": ROBOT_USDA})
    out = kit.run(robot="mine/v1/arm.usd")
    assert out["robot"]["prim"] == "/World/arm" and out["robot"]["articulation_root"] == "/World/arm/root_joint"
    assert out["assets"][0]["source"] == "pack" and out["assets"][0]["location"].endswith("mine/v1/arm.usd")
    assert out["assets"][0]["asset"] == "mine/v1/arm.usd"
    (pack / "arm.usd").write_text(ROBOT_USDA + "\n# edited by hand\n")  # now it no longer matches pack.yaml
    assert refused(kit, robot="mine/v1/arm.usd", discard_changes=True).code == "ASSET_MISSING"
    assert "Checksum mismatch" in str(refused(kit, robot="mine/v1/arm.usd", discard_changes=True))


@pytest.mark.parametrize("robot,code", [
    ("nobody", "INVALID_VALUE"),
    ("mine/arm.usd", "INVALID_VALUE"),  # a version is required
    ("mine/v1/../../etc/passwd.usd", "INVALID_VALUE"),
    ("mine/v1/notes.txt", "INVALID_VALUE"),
    ("../x/v1/arm.usd", "INVALID_VALUE"),
    ("mine/v1/absent.usd", "ASSET_MISSING"),  # not listed in pack.yaml
    ("mine/v2/arm.usd", "ASSET_MISSING"),
    ("/etc/hostile.usd", "ROOT_NOT_APPROVED"),  # not through the host: no snapshot, so never read
    ("~/hostile.usd", "ROOT_NOT_APPROVED"),
])
def test_robot_refusals_change_nothing(kit, robot, code):
    kit.pack("mine", "v1", {"arm.usd": ROBOT_USDA, "notes.txt": "n"})
    assert refused(kit, robot=robot).code == code
    assert not kit.stage.GetPrimAtPath("/World")


def test_a_file_in_a_pack_that_pack_yaml_does_not_list_is_never_loaded(kit):
    directory = kit.pack("mine", "v1", {"arm.usd": ROBOT_USDA})
    (directory / "extra.usd").write_text(ROBOT_USDA)
    error = refused(kit, robot="mine/v1/extra.usd")
    assert error.code == "ASSET_MISSING" and "not listed" in str(error) and not kit.stage.GetPrimAtPath("/World")


def test_a_symlinked_pack_file_is_never_followed(kit, tmp_path):
    directory = kit.pack("mine", "v1", {"arm.usd": ROBOT_USDA})
    outside = tmp_path / "outside.usd"
    outside.write_text(ROBOT_USDA)
    (directory / "arm.usd").unlink()
    (directory / "arm.usd").symlink_to(outside)
    assert refused(kit, robot="mine/v1/arm.usd").code == "ASSET_MISSING"
    assert not kit.stage.GetPrimAtPath("/World")


def test_without_an_asset_root_a_pack_file_says_which_setting_to_make(kit, monkeypatch, tmp_path):
    monkeypatch.delenv("TWINSMITH_ASSET_ROOT")
    monkeypatch.setenv("TWINSMITH_WORKSPACE", str(tmp_path / "empty-workspace"))
    error = refused(kit, robot="mine/v1/arm.usd")
    assert error.code == "ASSET_MISSING" and "TWINSMITH_ASSET_ROOT" in str(error)


def test_a_robot_file_that_composes_to_nothing_is_refused(kit):
    kit.pack("mine", "v1", {"empty.usd": '#usda 1.0\n(\n    defaultPrim = "x"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\ndef Xform "x" {}\n'})
    error = refused(kit, robot="mine/v1/empty.usd")
    assert error.code == "IMPORT_FAILED" and not kit.stage.GetPrimAtPath("/World/empty")


def test_unsaved_work_is_refused_then_dropped_only_when_asked(kit):
    UsdGeom.Xform.Define(kit.stage, "/Mine")
    old = kit.stage
    error = refused(kit)
    assert error.code == "UNSAVED_CHANGES" and kit.stage is old and kit.new_stages == 0
    out = kit.run(discard_changes=True)
    assert kit.stage is not old and out["stage_switched"] is True and out["discarded_changes"] is True


def test_running_twice_is_refused_because_the_first_scene_is_unsaved_work(kit):
    kit.run()
    assert refused(kit).code == "UNSAVED_CHANGES"


def test_a_running_timeline_is_refused_with_the_stop_call(kit):
    kit.timeline.playing = True
    error = refused(kit)
    assert error.code == "TIMELINE_ACTIVE" and error.details["before_retry"]["arguments"]["action"] == "stop"
    assert not kit.stage.GetPrimAtPath("/World")


def test_play_goes_through_the_guarded_timeline_path(kit):
    out = kit.run(robot="ur5e", play=True)
    assert kit.played == [dict(session_id=out["session_id"], stage_id=out["stage_id"], action="play")]
    assert out["playing"] is True and not any(step.startswith("Play:") for step in out["next_steps"])


def test_ros_without_a_robot_is_refused_first(kit):
    assert refused(kit, ros=True).code == "INVALID_VALUE"
    assert not kit.stage.GetPrimAtPath("/World")


@pytest.mark.parametrize("name", ["ros", "play", "discard_changes"])
def test_flags_must_be_booleans(kit, name):
    assert refused(kit, **{name: "yes"}).code == "INVALID_VALUE"


def test_ros_uses_tf_setup_and_reports_the_shell_line(kit, monkeypatch):
    from sim_tools import ros2

    calls = []

    def setup(scene, stage, args):
        calls.append(args)
        return dict(graph="/Graph/g", topics=["/tf"], rviz_fixed_frame="world", clock=dict(source="c"), warnings=[])

    monkeypatch.setattr(ros2, "setup", setup)
    monkeypatch.setattr(Scene, "stage_info", lambda self, real=Scene.stage_info: {**real(self), "ros": {"shell": "export X"}})
    out = kit.run(robot="ur5e", ros=True)
    assert calls == [dict(robot="/World/UR5e")]
    assert out["ros"]["graph"] == "/Graph/g" and out["ros"]["shell"] == "export X"


def test_ros_failure_keeps_the_scene_and_says_why(kit, monkeypatch):
    from sim_tools import ros2

    def setup(scene, stage, args):
        raise AssetError("ROS2_GRAPH_FAILED", "no bridge extension")

    monkeypatch.setattr(ros2, "setup", setup)
    out = kit.run(robot="ur5e", ros=True)
    assert out["ros"]["error"] == "ROS2_GRAPH_FAILED" and out["robot"]["prim"] == "/World/UR5e"


def test_its_own_ros_path_cannot_trip_the_joint_state_loop(kit):
    """It builds no subscriber, so the default /joint_states topic has nothing to loop with."""
    from sim_tools import ros2

    kit.run(robot="ur5e")
    subscribers = [(str(p.GetPath()), "", "") for p in kit.stage.Traverse() if "SubscribeJointState" in p.GetTypeName()]
    assert subscribers == [] and ros2.joint_state_loop("joint_states", "", subscribers) == []


def test_save_keeps_the_scene_and_it_opens_on_its_own(kit, tmp_path):
    kit.run(robot="ur5e")
    saved = kit.scene.save(kit.stage, "kept")
    reopened = Usd.Stage.Open(saved["file"])
    assert reopened.GetPrimAtPath(qs.PHYSICS_SCENE).IsA(UsdPhysics.Scene)
    assert reopened.GetPrimAtPath(qs.GROUND + "/CollisionPlane").HasAPI(UsdPhysics.CollisionAPI)
    assert reopened.GetPrimAtPath(qs.CAMERA).IsA(UsdGeom.Camera)
    assert reopened.GetPrimAtPath("/World/UR5e/root_joint").HasAPI(UsdPhysics.ArticulationRootAPI)
    assert reopened.GetPrimAtPath("/World/UR5e/link1/visuals/box")
    assert reopened.GetDefaultPrim().GetName() == "World"
    assert not Sdf.Layer.FindOrOpen(saved["file"]).subLayerPaths


# ---- up axis and units ----


def world_of(stage, path):
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def test_y_up_scene_follows_the_axis(kit):
    out = kit.run(up_axis="Y")
    stage = kit.stage
    assert UsdGeom.GetStageUpAxis(stage) == "Y" and out["up_axis"] == "Y"
    scene = UsdPhysics.Scene(stage.GetPrimAtPath(qs.PHYSICS_SCENE))
    assert list(scene.GetGravityDirectionAttr().Get()) == [0, -1, 0]
    assert UsdGeom.Plane(stage.GetPrimAtPath(qs.GROUND + "/CollisionPlane")).GetAxisAttr().Get() == "Y"
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(qs.GROUND + "/Geometry"))
    points = mesh.GetPointsAttr().Get()
    assert {p[1] for p in points} == {0} and {abs(p[0]) for p in points} == {25} and {abs(p[2]) for p in points} == {25}
    first = points[1] - points[0]
    second = points[2] - points[1]
    assert list(Gf.Cross(first, second).GetNormalized()) == pytest.approx([0, 1, 0])  # faces up
    assert list(mesh.GetNormalsAttr().Get()[0]) == [0, 1, 0]
    assert list(mesh.GetExtentAttr().Get()[0]) == [-25, 0, -25]


@pytest.mark.parametrize("up,index", [("Z", 2), ("Y", 1)])
def test_camera_is_above_the_ground_and_upright_on_either_axis(kit, up, index):
    kit.run(up_axis=up)
    world = world_of(kit.stage, qs.CAMERA)
    assert world.ExtractTranslation()[index] == pytest.approx(qs.EYE[2])
    assert world.TransformDir((0, 1, 0))[index] > 0  # the camera's own up points up
    assert world.TransformDir((1, 0, 0))[index] == pytest.approx(0, abs=1e-9)  # and the horizon is level
    assert world.TransformDir((0, 0, -1)).GetNormalized()[index] < 0  # it looks down at the ground


@pytest.mark.parametrize("up,index", [("Z", 2), ("Y", 1)])
def test_sun_shines_down_on_either_axis(kit, up, index):
    kit.run(up_axis=up)
    assert world_of(kit.stage, qs.SUN).TransformDir((0, 0, -1))[index] < 0  # distant lights shine along -Z


def test_z_up_metres_stays_the_default_layout(kit):
    out = kit.run()
    scene = UsdPhysics.Scene(kit.stage.GetPrimAtPath(qs.PHYSICS_SCENE))
    assert out["up_axis"] == "Z" and out["meters_per_unit"] == 1.0
    assert scene.GetGravityMagnitudeAttr().Get() == pytest.approx(9.81)
    assert list(world_of(kit.stage, qs.CAMERA).ExtractTranslation()) == pytest.approx(list(qs.EYE))


def test_distances_scale_with_units(kit):
    out = kit.run(meters_per_unit=0.01)
    stage = kit.stage
    assert UsdGeom.GetStageMetersPerUnit(stage) == 0.01 and out["meters_per_unit"] == 0.01
    assert UsdPhysics.Scene(stage.GetPrimAtPath(qs.PHYSICS_SCENE)).GetGravityMagnitudeAttr().Get() == pytest.approx(981.0)
    points = UsdGeom.Mesh(stage.GetPrimAtPath(qs.GROUND + "/Geometry")).GetPointsAttr().Get()
    assert {abs(p[0]) for p in points} == {2500}
    assert world_of(stage, qs.CAMERA).ExtractTranslation()[2] == pytest.approx(qs.EYE[2] * 100)
    clip = UsdGeom.Camera(stage.GetPrimAtPath(qs.CAMERA)).GetClippingRangeAttr().Get()
    assert list(clip) == pytest.approx([1.0, 100000.0])


def y_up_file(kit, name="scene.usd"):
    kit.pack("chess", "v1", {name: '''#usda 1.0
(
    defaultPrim = "set"
    metersPerUnit = 0.01
    upAxis = "Y"
)
def Xform "set" {
    def Cube "piece" {
        double size = 4
        double3 xformOp:translate = (0, 2, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
'''})
    return f"chess/v1/{name}"


def test_a_matching_file_works_and_frames_at_its_size(kit):
    out = kit.run(robot=y_up_file(kit), up_axis="Y", meters_per_unit=0.01)
    stage = kit.stage
    assert out["robot"]["prim"] == "/World/scene" and stage.GetPrimAtPath("/World/scene/piece")
    world = world_of(stage, qs.CAMERA)
    assert world.ExtractTranslation()[1] > 0
    minimum = qs.MIN_FRAMED_RADIUS_M * 100  # 50 cm in a centimetre stage, not 0.5 units
    half_fov = math.atan(qs.HORIZONTAL_APERTURE / qs.CAPTURE_ASPECT / 2 / qs.FOCAL_LENGTH)
    assert math.asin(minimum / (world.ExtractTranslation() - Gf.Vec3d(0, 2, 0)).GetLength()) <= half_fov + 1e-6


@pytest.mark.parametrize("args", [{}, {"up_axis": "Y"}, {"meters_per_unit": 0.01}])
def test_a_mismatch_refuses_with_both_sets_and_the_exact_args(kit, args):
    error = refused(kit, robot=y_up_file(kit), **args)
    assert error.code == "UNITS_MISMATCH"
    assert error.details["file_units"] == dict(up_axis="Y", meters_per_unit=0.01)
    assert error.details["scene_units"] == dict(up_axis=args.get("up_axis", "Z"), meters_per_unit=args.get("meters_per_unit", 1.0))
    assert error.details["retry_with"] == dict(up_axis="Y", meters_per_unit=0.01)
    assert 'up_axis="Y" and meters_per_unit=0.01' in str(error)
    assert not kit.stage.GetPrimAtPath("/World")


def test_the_public_robot_on_a_y_up_stage_refuses_rather_than_lying_on_its_side(kit):
    assert refused(kit, robot="ur5e", up_axis="Y").details["retry_with"] == dict(up_axis="Z", meters_per_unit=1.0)


def test_an_unreadable_local_file_refuses_before_changing_anything(kit):
    kit.pack("chess", "v1", {"broken.usda": "#usda 1.0\ndef Oops {"})
    assert refused(kit, robot="chess/v1/broken.usda").code == "IMPORT_FAILED"
    assert not kit.stage.GetPrimAtPath("/World")


def test_a_file_that_cannot_be_read_is_refused_whatever_its_source(kit, monkeypatch):
    monkeypatch.setattr(qs, "file_units", lambda uri: (_ for _ in ()).throw(RuntimeError("no network")))
    wanted = dict(uri="https://x/y.usd", assets=[dict(source="nvidia_assets_root")])
    with pytest.raises(AssetError) as error:
        qs.check_resolved_units(wanted, "Z", 1.0)
    assert error.value.code == "IMPORT_FAILED"


@pytest.mark.parametrize("args", [{"up_axis": "X"}, {"up_axis": "y"}, {"up_axis": 1}, {"meters_per_unit": 0},
                                  {"meters_per_unit": -1}, {"meters_per_unit": True}, {"meters_per_unit": "1"},
                                  {"meters_per_unit": float("nan")}, {"meters_per_unit": float("inf")}])
def test_bad_axis_or_units_are_refused_first(kit, args):
    assert refused(kit, **args).code == "INVALID_VALUE"
    assert not kit.stage.GetPrimAtPath("/World")


def test_a_new_stage_gets_the_values_too(kit):
    UsdGeom.Xform.Define(kit.stage, "/Mine")
    kit.run(up_axis="Y", meters_per_unit=0.01, discard_changes=True)
    assert kit.new_stages == 1 and UsdGeom.GetStageUpAxis(kit.stage) == "Y"
    assert UsdGeom.GetStageMetersPerUnit(kit.stage) == 0.01


def test_save_and_reopen_keeps_the_axis_and_units(kit):
    kit.run(robot=y_up_file(kit), up_axis="Y", meters_per_unit=0.01)
    saved = kit.scene.save(kit.stage, "yup")
    reopened = Usd.Stage.Open(saved["file"])
    assert UsdGeom.GetStageUpAxis(reopened) == "Y" and UsdGeom.GetStageMetersPerUnit(reopened) == 0.01
    assert list(UsdPhysics.Scene(reopened.GetPrimAtPath(qs.PHYSICS_SCENE)).GetGravityDirectionAttr().Get()) == [0, -1, 0]
    assert reopened.GetPrimAtPath("/World/scene/piece")


def test_contract_needs_no_ids_and_cannot_name_a_command():
    params = REGISTRY["sim.quick_start"]["parameters"]
    assert set(params) == {"robot", "xacro_args", "sdf_model", "environment", "fixed_base", "ros", "up_axis", "meters_per_unit", "play", "discard_changes"}
    assert not any(field["required"] for field in params.values())


def test_host_turns_an_absolute_path_into_a_snapshot_or_refuses(monkeypatch):
    from twinsmith import server as gw

    calls = []

    class Client:
        def call(self, name, params=None):
            calls.append((name, params))
            return dict(staging="/stage") if name == "sim.stage.info" else dict(status="ok")

    class Broker:
        staging = "/stage"

        def locate(self, path, kind="usd"):
            assert path == "/approved/arm.usd"
            return "root", "arm.usd"

        def snapshot(self, root_id, relative_path, kind):
            assert (root_id, relative_path, kind) == ("root", "arm.usd", "usd")
            return dict(snapshot_id="s", manifest_sha256="m")

    state = SimpleNamespace(get_client=lambda: Client(), asset_broker=Broker())
    gw._dispatch("sim.quick_start", {"robot": "/approved/arm.usd"}, state)
    assert calls[-1] == ("sim.quick_start", {"robot": "/approved/arm.usd", "snapshot_id": "s", "manifest_sha256": "m"})
    state.asset_broker.staging = None
    calls.clear()
    answer = json.loads(gw._dispatch("sim.quick_start", {"robot": "/approved/arm.usd"}, state))
    assert answer["code"] == "NO_SHARED_STAGING" and all(name != "sim.quick_start" for name, _ in calls)


def test_host_passes_names_and_packs_straight_through(monkeypatch):
    from twinsmith import server as gw

    calls = []
    client = SimpleNamespace(call=lambda name, params=None: calls.append((name, params)) or dict(status="ok"))
    gw._dispatch("sim.quick_start", {"robot": "ur5e"}, SimpleNamespace(get_client=lambda: client))
    assert calls == [("sim.quick_start", {"robot": "ur5e"})]


def test_next_steps_send_inspect_to_the_articulation_root_not_a_link():
    steps = qs.next_steps({"session_id": "s", "stage_id": "g"}, "/World/Robot", "/World/Robot/root_joint", False)
    inspect = [step for step in steps if "sim__joints__inspect" in step]
    assert inspect and 'path="/World/Robot/root_joint"' in inspect[0]
    assert "articulation root" in inspect[0]


def test_public_robot_next_steps_never_name_a_link_for_joint_inspection(kit):
    out = kit.run(robot="ur5e")
    root = out["robot"]["articulation_root"]
    inspect = [step for step in out["next_steps"] if "sim__joints__inspect" in step]
    assert inspect and f'path="{root}"' in inspect[0]


class Calls:
    """Stands in for server._dispatch: records what quick start asks for and answers as the bridge would."""

    def __init__(self, imported=None, scene=None):
        self.calls = []
        self.imported = imported or dict(status="ok", articulation_root="/World/go2/Asset/base", articulation_roots=["/World/go2/Asset/base"], base="floating",
                                         placement=dict(lifted_by=0.25))
        self.scene = scene or {}

    def __call__(self, name, params, state=None):
        self.calls.append((name, params))
        if name == "sim.quick_start":
            return json.dumps(dict(status="ok", session_id="s", stage_id="t", playing=False, stopped=True, **self.scene))
        if name == "assets.import_path":
            return json.dumps(self.imported)
        return json.dumps(dict(status="ok"))

    def named(self, name):
        return [p for n, p in self.calls if n == name]


def quick_start(monkeypatch, calls, **params):
    from twinsmith import server

    monkeypatch.setattr(server, "_dispatch", calls)
    return json.loads(server._quick_start_with_import({"robot": "/data/go2", **params}, None))


def test_quick_start_with_a_file_that_fails_to_import_is_an_error_not_a_success(monkeypatch):
    calls = Calls(imported=dict(status="error", code="UNITS_MISMATCH", message="They differ."))
    result = quick_start(monkeypatch, calls, up_axis="Y")
    assert result["status"] == "error" and result["code"] == "UNITS_MISMATCH" and "not imported" in result["message"] and "They differ" in result["message"]
    assert result["scene_built"] is True and result["session_id"] == "s"


def test_quick_start_with_a_file_plays_after_the_import_not_before(monkeypatch):
    """The scene was playing when the import ran, so the import always refused with TIMELINE_ACTIVE."""
    calls = Calls()
    result = quick_start(monkeypatch, calls, play=True)
    assert "play" not in calls.named("sim.quick_start")[0]
    order = [n for n, _ in calls.calls]
    assert order.index("assets.import_path") < order.index("sim.timeline")
    assert result["playing"] is True and result["stopped"] is False


def test_quick_start_with_a_file_leaves_the_base_to_the_file_unless_told(monkeypatch):
    calls = Calls()
    quick_start(monkeypatch, calls)
    assert "fixed_base" not in calls.named("assets.import_path")[0]
    calls = Calls()
    quick_start(monkeypatch, calls, fixed_base=True)
    assert calls.named("assets.import_path")[0]["fixed_base"] is True


def test_the_quick_start_contract_does_not_invent_a_fixed_base():
    """A default of true bolted a MuJoCo Go2 and a Gazebo Pioneer to the world through quick start, and not through the import."""
    from twinsmith.scene_contracts import REGISTRY

    fields = REGISTRY["sim.quick_start"]["parameters"]
    assert "default" not in fields["fixed_base"] and "xacro_args" in fields and "sdf_model" in fields


def test_quick_start_with_a_file_in_an_environment_replaces_its_robot_and_goes_to_its_start(monkeypatch):
    """The environment's default robot was added too, and the file stood in the middle of the warehouse."""
    calls = Calls(scene=dict(environment=dict(robot_start=[-10.5, -2.5, 0.0])))
    result = quick_start(monkeypatch, calls, environment="warehouse")
    assert calls.named("sim.quick_start")[0]["no_default_robot"] is True
    moved = calls.named("sim.prim.set_transform")[0]
    assert moved["path"] == "/World/go2" and moved["position"] == [-10.5, -2.5, 0.25]  # keeps the lift out of the ground
    assert result["robot"]["articulation_root"] == "/World/go2/Asset/base" and result["robot"]["base"] == "floating"


def test_quick_start_preserves_an_explicit_position_in_an_environment(monkeypatch):
    calls = Calls(scene=dict(environment=dict(robot_start=[-10.5, -2.5, 0.0])))
    quick_start(monkeypatch, calls, environment="warehouse", position=[1, 2, 3])
    assert calls.named("assets.import_path")[0]["position"] == [1, 2, 3]
    assert not calls.named("sim.prim.set_transform")


def test_quick_start_reports_play_failure_after_import(monkeypatch):
    class PlayFails(Calls):
        def __call__(self, name, params, state=None):
            if name == "sim.timeline":
                return json.dumps(dict(status="error", code="NOT_READY", message="not initialized"))
            return super().__call__(name, params, state)

    result = quick_start(monkeypatch, PlayFails(), play=True)
    assert result["playing"] is False and result["play_error"]["code"] == "NOT_READY"


def test_quick_start_with_a_file_runs_ros_after_the_import(monkeypatch):
    """ros=true was refused with "needs a robot" though one was given."""
    calls = Calls()
    result = quick_start(monkeypatch, calls, ros=True)
    assert "ros" not in calls.named("sim.quick_start")[0]
    assert calls.named("sim.ros2_tf")[0]["robot"] == "/World/go2" and calls.named("sim.ros2_tf")[0]["base"] == "mobile"
    assert result["ros"]["status"] == "ok"


def test_a_urdf_that_holds_xacro_goes_through_the_import_from_quick_start(tmp_path):
    from twinsmith import server

    real = tmp_path / "plain.urdf"
    real.write_text('<robot name="r"/>')
    xacro = tmp_path / "burger.urdf"
    xacro.write_text('<robot name="r" xmlns:xacro="http://ros.org/wiki/xacro"/>')
    project = tmp_path / "project"
    project.mkdir()
    real.rename(project / real.name)
    xacro.rename(project / xacro.name)
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    approval = tmp_path / "approval.json"
    approval.write_text(json.dumps(dict(version=1, profile="trusted_project", roots={"fixture": str(project)}, staging=str(staging))))
    state = SimpleNamespace(asset_broker=Broker(str(approval)))
    assert server._quick_start_wants_import(str(project / real.name), state) is False
    assert server._quick_start_wants_import(str(project / xacro.name), state) is True
    assert server._quick_start_wants_import(str(project / "missing.urdf"), state) is False
    # Outside roots, symlinks and non-regular files are never read by the probe.
    outside = tmp_path / "outside.urdf"
    outside.write_text('<robot xmlns:xacro="http://ros.org/wiki/xacro"/>')
    (project / "link.urdf").symlink_to(outside)
    import os
    os.mkfifo(project / "pipe.urdf")
    assert server._quick_start_wants_import(str(outside), state) is False
    assert server._quick_start_wants_import(str(project / "link.urdf"), state) is False
    assert server._quick_start_wants_import(str(project / "pipe.urdf"), state) is False
