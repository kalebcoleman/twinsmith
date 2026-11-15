"""CPU USD contract tests; synthetic assets and tensors do not qualify PhysX."""

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("pxr.Usd")
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


ROOT = Path(__file__).resolve().parents[2] / "exts/workcell/workcell"


def load_module(name):
    spec = importlib.util.spec_from_file_location(f"workcell_{name}", ROOT / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


robot = load_module("robot")
sensors = load_module("sensors")


def stage_units(stage):
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1)


def body(stage, path, position=(0, 0, 0)):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*position))
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(0.1)
    return prim


class Pack:
    def __init__(self, paths):
        self.paths = paths
        self.calls = []

    def asset(self, asset_id):
        self.calls.append(asset_id)
        return self.paths[asset_id]


@pytest.fixture
def cell(tmp_path):
    paths, layouts = {}, {}
    for role, model in robot.MODELS.items():
        path = tmp_path / f"{role}.usda"
        source = Usd.Stage.CreateNew(str(path))
        stage_units(source)
        root_path = "/khi_rs080n" if role == "rg2" else "/Asset"
        root = UsdGeom.Xform.Define(source, root_path).GetPrim()
        source.SetDefaultPrim(root)
        UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(0.23, -0.17, 0.08))
        layout = {"model": model, "bom_id": f"BOM-{role}", "fidelity": "placeholder"}
        if role in ("finger_pad", "base_plate"):
            cube = UsdGeom.Cube.Define(source, root_path + "/pad")
            cube.CreateSizeAttr().Set(0.01)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        else:
            name = {"robot": "wrist_3_link", "rg2": "onrobot_rg2_base_link"}.get(role, role + "_body")
            base = body(source, root_path + "/" + name, (0.01, 0.02, 0.03))
            layout.update(body=name, mount_frame=name)
            if role == "robot":
                joint = UsdPhysics.FixedJoint.Define(source, root_path + "/root_joint")
                joint.CreateBody1Rel().SetTargets([base.GetPath()])
                UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
                flange = UsdGeom.Xform.Define(source, base.GetPath().AppendChild("flange"))
                flange.AddTranslateOp().Set(Gf.Vec3d(0.04, -0.01, 0.02))
                flange.AddRotateZOp().Set(37)
                layout.update(articulation_root="root_joint", flange_frame=name + "/flange")
            if role == "rg2":
                UsdPhysics.ArticulationRootAPI.Apply(root)
                layout["articulation_root"] = "."
                layout["mount_pose"] = {"position_m": [0, 0, 0],
                                        "orientation_wxyz": [math.sqrt(0.5), 0, -math.sqrt(0.5), 0]}
                for side, sign in (("left", 1), ("right", -1)):
                    finger = body(source, root_path + f"/{side}_inner_finger", (0.01, sign * 0.04, 0.15))
                    joint = UsdPhysics.RevoluteJoint.Define(source, root_path + f"/{side}_joint")
                    joint.CreateBody0Rel().SetTargets([base.GetPath()])
                    joint.CreateBody1Rel().SetTargets([finger.GetPath()])
                    joint.CreateAxisAttr().Set("X")
                    joint.CreateLowerLimitAttr().Set(-45)
                    joint.CreateUpperLimitAttr().Set(20)
        source.GetRootLayer().Save()
        paths[f"pack/{role}"] = path
        layouts[role] = layout
    config = {
        "assets": {role: f"pack/{role}" for role in layouts}, "asset_layout": layouts,
        "robot": {"placement": {"position_m": [1, 2, 3], "orientation_wxyz": [1, 0, 0, 0]}},
        "base_plate": {"placement": {"position_m": [0, 0, -0.01], "orientation_wxyz": [1, 0, 0, 0]}},
        "stack": {},
        "bota": {"mounted": False, "length_m": 0.032, "mass_kg": 0.12, "sensor_origin_along_axis_m": 0.016,
                 "frame_id": "bota_ft_sensor",
                 "robot_side_plate": {"thickness_m": 0.008, "diameter_m": 0.063, "mass_kg": 0.067},
                 "tool_side_plate": {"thickness_m": 0.006, "diameter_m": 0.063, "mass_kg": 0.05}},
        "flange_ft_sensor": {"frame_id": "tool0"},
        "quick_changer": {"standoff_m": 0.0136, "mass_kg": 0.06},
        "rg2": {"max_opening_m": 0.110},
        "finger_pads": {"present": True, "static_friction": 0.8, "dynamic_friction": 0.7,
                        "mounts": [{"parent": side + "_inner_finger", "position_m": [0, 0, 0.005],
                                    "orientation_wxyz": [1, 0, 0, 0]} for side in ("left", "right")]},
        "finger_force_sensors": {"fitted": False},
    }
    stage = Usd.Stage.CreateInMemory()
    stage_units(stage)
    UsdGeom.Xform.Define(stage, "/World")
    return stage, Pack(paths), config


def world(stage, path):
    return UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))


LENGTHS = {"bota_robot_plate": 0.008, "bota": 0.032, "bota_tool_plate": 0.006, "quick_changer": 0.0136, "rg2": 0}
PLATES = ("bota_robot_plate", "bota_tool_plate")


@pytest.mark.parametrize("mounted", [False, True])
def test_stack_follows_the_one_switch_and_sources_are_unchanged(cell, mounted):
    """bota.mounted is the only thing edited: off, the stack is flange, Quick Changer, RG2 and no Bota prim exists;
    on, an adapter plate, the Bota and a second plate come first and every later part moves down the tool axis."""
    stage, pack, config = cell
    config["bota"]["mounted"] = mounted
    order = (["bota_robot_plate", "bota", "bota_tool_plate"] if mounted else []) + ["quick_changer", "rg2"]
    originals = {key: path.read_bytes() for key, path in pack.paths.items()}
    root_before = stage.GetRootLayer().ExportToString()
    report = robot.build_robot(stage, pack, config)
    json.dumps(report, allow_nan=False)
    assert stage.GetRootLayer().ExportToString() == root_before
    assert stage.GetSessionLayer().subLayerPaths == [report["layer_id"]]
    assert report["stack_order"] == order == report["tool"]["order"]
    assert report["tool"]["bota_mounted"] is mounted
    assert report["gripper_articulation_root"] == report["gripper_path"]
    assert report["max_opening_m"] == 0.110
    flange = world(stage, report["flange_path"])
    offset = 0.0
    for role in order:
        path = report["assembly_path"] + "/" + role
        layout = config["asset_layout"].get(role, {})
        mount = stage.GetPrimAtPath(path + "/" + layout["mount_frame"]) if layout else stage.GetPrimAtPath(path)
        actual = robot._mount_world(mount, layout) * flange.GetInverse()
        assert np.array(actual) == pytest.approx(np.array(Gf.Matrix4d().SetTranslate(Gf.Vec3d(offset, 0, 0))))
        joint = UsdPhysics.FixedJoint.Get(stage, report["assembly_path"] + f"/joints/{role}_mount")
        frames = []
        for side in (0, 1):
            local = Gf.Matrix4d().SetRotate(Gf.Quatd(getattr(joint, f"GetLocalRot{side}Attr")().Get()))
            local.SetTranslateOnly(Gf.Vec3d(getattr(joint, f"GetLocalPos{side}Attr")().Get()))
            target = getattr(joint, f"GetBody{side}Rel")().GetTargets()[0]
            frames.append(local * world(stage, target))
        assert np.array(frames[0]) == pytest.approx(np.array(frames[1]), abs=1e-6)
        assert joint.GetExcludeFromArticulationAttr().Get() == (role == "rg2")
        offset += LENGTHS[role]
    assert report["tool"]["flange_to_rg2_inlet_m"] == pytest.approx(0.0596 if mounted else 0.0136)
    assert report["flange_joint_path"] == report["assembly_path"] + f"/joints/{order[0]}_mount"
    assert report["flange_load_body_path"].startswith(report["assembly_path"] + "/" + order[0])
    native_rg2 = world(stage, report["gripper_path"] + "/onrobot_rg2_base_link") * flange.GetInverse()
    assert list(native_rg2.TransformDir(Gf.Vec3d(0, 0, 1))) == pytest.approx([1, 0, 0])
    tool0 = world(stage, report["tool0_path"]) * flange.GetInverse()
    assert list(tool0.TransformDir(Gf.Vec3d(0, 0, 1))) == pytest.approx([1, 0, 0])
    assert len(report["parts"]) == (7 if mounted else 6)
    assert pack.calls == [f"pack/{role}" for role in ["robot", "base_plate", *(r for r in order if r not in PLATES), "finger_pad"]]
    for part in report["parts"]:
        prim = stage.GetPrimAtPath(part["prim_path"])
        assert prim.GetCustomDataByKey("workcell:asset_id") == part["asset_id"]
        assert prim.GetCustomDataByKey("workcell:bom_id") == part["bom_id"]
    bota_prims = [str(prim.GetPath()) for prim in stage.Traverse() if prim.GetName().lower().startswith("bota")]
    if mounted:
        assert report["bota"]["joint_path"] == report["assembly_path"] + "/joints/bota_tool_plate_mount"
        assert report["bota"]["load_body_path"] == report["assembly_path"] + "/bota_tool_plate"
        assert stage.GetPrimAtPath(report["bota"]["sensor_path"])
        assert [proxy["prim_path"] for proxy in report["proxies"]] == [report["assembly_path"] + "/" + role for role in PLATES]
        for role, record in (("bota_robot_plate", "robot_side_plate"), ("bota_tool_plate", "tool_side_plate")):
            plate = stage.GetPrimAtPath(report["assembly_path"] + "/" + role)
            assert plate.HasAPI(UsdPhysics.RigidBodyAPI) and plate.GetCustomDataByKey("workcell:fidelity") == "placeholder"
            assert UsdPhysics.MassAPI(plate).GetMassAttr().Get() == pytest.approx(config["bota"][record]["mass_kg"])
            assert UsdGeom.Cylinder(plate.GetChild("disc")).GetHeightAttr().Get() == pytest.approx(config["bota"][record]["thickness_m"])
        assert any("measure the real adapter plate" in line for line in report["limitations"])
        assert any("adapter plate" in line for line in report["tool"]["unmeasured"])
    else:
        assert report["bota"] is None and report["proxies"] == [] and bota_prims == []    # nothing of the sensor is built
        assert not any("adapter plate" in line for line in report["tool"]["unmeasured"])
    assert any("placeholder" in limitation for limitation in report["limitations"])
    assert not any("marker" in str(prim.GetPath()).lower() for prim in stage.Traverse())
    assert {key: path.read_bytes() for key, path in pack.paths.items()} == originals


def test_what_the_rg2_body_carries_is_in_its_mass(cell):
    """The printed bracket and the D415 have no rigid body of their own: their masses go on the RG2 body, so the
    force/torque reading holds them, as on the real arm."""
    stage, pack, config = cell
    base = {"mass_kg": 0.6, "com_m": [0.0, 0.0, 0.06], "diagonal_inertia_kgm2": [0.0012, 0.0011, 0.0004], "principal_axes_wxyz": [1, 0, 0, 0]}
    config["inertials"] = {"rg2_links": {"onrobot_rg2_base_link": dict(base)}}
    config["wrist_camera"] = {"bracket_mass_kg": 0.1, "bracket_cog_in_rg2_m": [0.01, -0.01, 0.09],
                              "mass_kg": 0.064, "cog_in_rg2_m": [0.066, -0.068, 0.117]}
    report = robot.build_robot(stage, pack, config)
    api = UsdPhysics.MassAPI(stage.GetPrimAtPath(report["gripper_path"] + "/onrobot_rg2_base_link"))
    assert api.GetMassAttr().Get() == pytest.approx(0.764)
    expected = (0.6 * np.array(base["com_m"]) + 0.1 * np.array([0.01, -0.01, 0.09]) + 0.064 * np.array([0.066, -0.068, 0.117])) / 0.764
    assert list(api.GetCenterOfMassAttr().Get()) == pytest.approx(expected.tolist(), abs=1e-7)
    moments = np.array(api.GetDiagonalInertiaAttr().Get())
    assert (moments > 0).all() and moments.sum() > sum(base["diagonal_inertia_kgm2"])    # the carried masses add inertia
    assert moments[0] + moments[1] >= moments[2] - 1e-12                                 # still a physical tensor
    assert [item["mass_kg"] for item in report["carried_on_rg2"]] == [0.1, 0.064]
    assert report["inertials_kg"]["onrobot_rg2_base_link"] == pytest.approx(0.764)
    assert any("point masses" in line for line in report["limitations"])


def test_carried_masses_need_the_rg2_bodys_own_mass_properties(cell):
    stage, pack, config = cell
    config["wrist_camera"] = {"mass_kg": 0.064, "cog_in_rg2_m": [0.066, -0.068, 0.117]}
    with pytest.raises(ValueError, match="inertials must hold the RG2 body"):
        robot.build_robot(stage, pack, config)
    assert not stage.GetPrimAtPath("/World/RobotAssembly")


def test_base_plate_pose_metadata_and_static_geometry(cell):
    stage, pack, config = cell
    rotation = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    config["robot"]["placement"]["orientation_wxyz"] = rotation
    config["base_plate"]["placement"] = {"position_m": [0.1, 0.2, -0.01], "orientation_wxyz": rotation}
    report = robot.build_robot(stage, pack, config)
    path = report["base_plate_path"]
    local = world(stage, path) * world(stage, report["assembly_path"]).GetInverse()
    assert np.array(local) == pytest.approx(np.array(robot._pose([0.1, 0.2, -0.01], rotation)))
    assert next(part for part in report["parts"] if part["prim_path"] == path) == {
        "prim_path": path, "asset_id": "pack/base_plate", "bom_id": "BOM-base_plate", "fidelity": "placeholder"}
    assert stage.GetPrimAtPath(path + "/pad").HasAPI(UsdPhysics.CollisionAPI)
    assert not stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI)
    assert Usd.Stage.Open(stage.Flatten()).GetPrimAtPath(path)


@pytest.mark.parametrize("fault", ["missing_pose", "dynamic_plate", "reset_stack", "missing_asset"])
def test_base_plate_failure_rolls_back(cell, fault):
    stage, pack, config = cell
    if fault == "missing_pose":
        del config["base_plate"]["placement"]
    elif fault == "missing_asset":
        del config["assets"]["base_plate"]
    else:
        source = Usd.Stage.Open(str(pack.paths["pack/base_plate"]))
        if fault == "dynamic_plate":
            UsdPhysics.RigidBodyAPI.Apply(source.GetDefaultPrim())
        else:
            UsdGeom.Xformable(source.GetDefaultPrim()).SetResetXformStack(True)
        source.GetRootLayer().Save()
    with pytest.raises((ValueError, KeyError)):
        robot.build_robot(stage, pack, config)
    assert not stage.GetPrimAtPath("/World/RobotAssembly")
    assert not stage.GetSessionLayer().subLayerPaths


def test_black_pads_follow_finger_frames_and_export(cell):
    stage, pack, config = cell
    config["bota"]["mounted"] = True
    report = robot.build_robot(stage, pack, config)
    for path in report["pad_paths"]:
        prim = stage.GetPrimAtPath(path)
        local, reset = UsdGeom.XformCache().ComputeRelativeTransform(prim, prim.GetParent())
        assert not reset
        assert list(local.ExtractTranslation()) == pytest.approx([0, 0, 0.005])
        material, _ = UsdShade.MaterialBindingAPI(prim.GetChild("pad")).ComputeBoundMaterial()
        shader = UsdShade.Shader(material.GetPrim().GetChild("surface"))
        assert list(shader.GetInput("diffuseColor").Get()) == [0, 0, 0]
        assert UsdPhysics.MaterialAPI(material).GetStaticFrictionAttr().Get() == pytest.approx(0.8)
        shapes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Gprim)]
        assert shapes and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in shapes)   # the pad, not the bare link, touches
        assert all(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == "convexHull"
                   for p in shapes if p.IsA(UsdGeom.Mesh))
    reopened = Usd.Stage.Open(stage.Flatten())
    assert reopened.GetPrimAtPath(report["bota"]["sensor_path"])
    before = stage.GetSessionLayer().ExportToString()
    with pytest.raises(ValueError, match="new absolute"):
        robot.build_robot(stage, pack, config)
    assert stage.GetSessionLayer().ExportToString() == before


@pytest.mark.parametrize("fault", ["stage_units", "asset_units", "up_axis", "missing_body", "bad_model",
                                    "bad_quaternion", "bad_order", "bad_width", "pads", "world_joint", "nested_body",
                                    "switch_not_a_boolean", "mounted_without_a_plate", "mounted_with_a_bad_bota_body"])
def test_invalid_configuration_rolls_back(cell, fault):
    stage, pack, config = cell
    if fault == "stage_units":
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    elif fault in ("asset_units", "up_axis", "world_joint", "nested_body"):
        source = Usd.Stage.Open(str(pack.paths["pack/rg2"]))
        if fault == "asset_units":
            UsdGeom.SetStageMetersPerUnit(source, 0.01)
        elif fault == "up_axis":
            UsdGeom.SetStageUpAxis(source, "Y")
        elif fault == "world_joint":
            UsdPhysics.Joint.Get(source, "/khi_rs080n/left_joint").GetBody0Rel().ClearTargets(True)
        else:
            body(source, "/khi_rs080n/onrobot_rg2_base_link/nested")
        source.GetRootLayer().Save()
    elif fault == "missing_body":
        config["asset_layout"]["quick_changer"]["body"] = "missing"
    elif fault == "switch_not_a_boolean":
        config["bota"]["mounted"] = "yes"
    elif fault == "mounted_without_a_plate":
        config["bota"]["mounted"] = True
        del config["bota"]["tool_side_plate"]
    elif fault == "mounted_with_a_bad_bota_body":
        config["bota"]["mounted"] = True
        config["asset_layout"]["bota"]["body"] = "missing"
    elif fault == "bad_model":
        config["asset_layout"]["rg2"]["model"] = "husarion_rg2_ft"
    elif fault == "bad_quaternion":
        config["robot"]["placement"]["orientation_wxyz"] = [2, 0, 0, 0]
    elif fault == "bad_order":
        config["stack"]["order"] = ["bota", "quick_changer", "rg2"]      # a leftover order is refused, not ignored
    elif fault == "bad_width":
        config["rg2"]["max_opening_m"] = 0.16
    elif fault == "pads":
        del config["finger_pads"]["mounts"]
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError):
        robot.build_robot(stage, pack, config)
    assert not stage.GetPrimAtPath("/World/RobotAssembly")
    assert not stage.GetSessionLayer().subLayerPaths
    assert stage.GetRootLayer().ExportToString() == before


class Articulation:
    count = 1
    max_links = 1

    def __init__(self, report=None, link="quick_changer_body"):
        self.shared_metatype = SimpleNamespace(dof_names=["left_joint", "right_joint"], link_indices={link: 0})
        self.prim_paths = [report["articulation_root"]] if report else []
        self.targets = np.array([[0.1, 0.2]], dtype=np.float32)
        self.wrench = np.array([[[0, 10, 0, 0, 0, 1]]], dtype=np.float32)

    def check(self):
        return True

    def get_dof_positions(self):
        return np.array([[0.05, 0.06]], dtype=np.float32)

    def get_dof_velocities(self):
        return np.array([[0.01, 0.02]], dtype=np.float32)

    def get_dof_position_targets(self):
        return self.targets

    def get_dof_limits(self):
        return np.array([[[-1, 1], [-1, 1]]], dtype=np.float32)

    def set_dof_position_targets(self, targets, indices):
        assert indices.dtype == np.uint32
        assert indices.tolist() == [0]
        self.targets = targets
        return True

    def get_link_incoming_joint_force(self):
        return self.wrench


def test_named_commands_and_state():
    view = Articulation()
    measured = robot.robot_state(view)
    assert measured["positions"] == pytest.approx([0.05, 0.06])
    robot.command_joints(view, {"left_joint": -0.4})
    assert view.targets[0].tolist() == pytest.approx([-0.4, 0.2])
    for targets in ({"missing": 0}, {"left_joint": 2}, {"left_joint": float("nan")}, {}):
        with pytest.raises(ValueError):
            robot.command_joints(view, targets)
    with pytest.raises(RuntimeError, match="UNAVAILABLE"):
        robot.robot_state(None)


def test_width_requires_explicit_calibration():
    view = Articulation()
    config = {"rg2": {}}
    assert robot.command_gripper(view, 0.055, config)["status"] == "unsupported"
    config["rg2"]["width_calibration"] = [
        {"width_m": 0, "joints": {"left_joint": -0.7, "right_joint": -0.6}},
        {"width_m": 0.110, "joints": {"left_joint": 0.3, "right_joint": 0.4}},
    ]
    result = robot.command_gripper(view, 0.055, config)
    assert result["targets"] == pytest.approx({"left_joint": -0.2, "right_joint": -0.1})
    assert result["measured_width_m"] is None
    for width in (-0.001, 0.111, float("nan")):
        with pytest.raises(ValueError):
            robot.command_gripper(view, width, config)


def test_flange_wrench_is_the_default_reading(cell):
    """No Bota on the arm: the reading is the UR5e's own flange sensor, the reaction at the joint between wrist 3
    and the first part on the flange, at the flange centre in tool0's axes, and nothing in it is named after a Bota."""
    stage, pack, config = cell
    report = robot.build_robot(stage, pack, config)
    view = Articulation(report)
    args = dict(physics_time_s=1.5, backend="physx")
    assert sensors.read_wrench(stage, view, report, config, **args)["measurement"] is None    # not configured: nothing is made up
    config["flange_ft_sensor"]["measurement"] = "incoming_joint_wrench"
    result = sensors.read_wrench(stage, view, report, config, **args)
    assert result["status"] == "measured" and result["source"] == sensors.FLANGE_SOURCE == "ur5e_tool_flange_sensor"
    assert result["frame_id"] == "tool0" and result["frame_path"] == report["tool0_path"]
    assert result["source_joint"] == report["flange_joint_path"] and result["load_body_path"] == report["flange_load_body_path"]
    assert "bota" not in json.dumps(result).lower().replace("robotassembly", "")
    # The view's wrench is in the joint (flange) frame: +Y force, +Z torque. tool0 +X is the flange's +Y, +Y its +Z.
    assert result["force_n"] == pytest.approx([10, 0, 0], abs=1e-6)
    assert result["torque_nm"] == pytest.approx([0, 1, 0], abs=1e-6)
    assert not result["gravity_compensated"] and not result["payload_compensated"] and not result["noise_modelled"]
    assert result["physics_time_s"] == 1.5
    refused = sensors.read_wrench(stage, view, report, config, source=sensors.BOTA_SOURCE, **args)
    assert refused["status"] == "unsupported" and "bota.mounted is false" in refused["reason"]
    assert sensors.read_wrench(stage, view, report, config, source="other", **args)["status"] == "unsupported"
    view.wrench[:] = np.nan
    assert sensors.read_wrench(stage, view, report, config, **args)["measurement"] is None
    view.shared_metatype.link_indices.clear()
    assert sensors.read_wrench(stage, view, report, config, **args)["status"] == "unavailable"


@pytest.mark.parametrize("rotated", [False, True])
def test_bota_wrench_is_read_at_its_tool_side_face_when_mounted(cell, rotated):
    """With the Bota mounted the default reading is the Bota's: the reaction at the joint on its tool-side face, in its
    sensor frame; the flange reading is still there for the asking."""
    stage, pack, config = cell
    config["bota"]["mounted"] = True
    if rotated:
        config["bota"]["sensor_orientation_wxyz"] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    report = robot.build_robot(stage, pack, config)
    view = Articulation(report, link="bota_tool_plate")
    args = dict(physics_time_s=1.5, backend="physx")
    assert sensors.read_wrench(stage, view, report, config, **args)["measurement"] is None
    config["bota"]["measurement"] = "incoming_joint_wrench"
    result = sensors.read_wrench(stage, view, report, config, **args)
    assert result["status"] == "measured" and result["source"] == sensors.BOTA_SOURCE == "bota_rokubi"
    assert result["frame_id"] == "bota_ft_sensor" and result["frame_path"] == report["bota"]["sensor_path"]
    assert result["source_joint"] == report["bota"]["joint_path"]
    assert result["force_n"] == pytest.approx([10, 0, 0] if rotated else [0, 10, 0], abs=1e-6)
    # The joint is on the tool-side face, 16 mm beyond the sensor frame along the tool axis: 1 + 0.016 * 10.
    assert result["torque_nm"] == pytest.approx([0, 0, 1.16], abs=1e-6)
    config["flange_ft_sensor"]["measurement"] = "incoming_joint_wrench"
    flange = sensors.read_wrench(stage, Articulation(report, link="bota_robot_plate"), report, config,
                                 source=sensors.FLANGE_SOURCE, **args)
    assert flange["status"] == "measured" and flange["frame_id"] == "tool0"
    assert flange["source_joint"] == report["assembly_path"] + "/joints/bota_robot_plate_mount"
    assert flange["force_n"] == pytest.approx([10, 0, 0], abs=1e-6)


class WarpLike:
    """What Isaac Sim 6.x tensor views return: an array object with .numpy(), flat on Newton."""

    def __init__(self, values):
        self.values = values

    def numpy(self):
        return self.values


def test_wrench_reads_a_warp_frontend_on_newton_and_refuses_an_unknown_engine(cell):
    stage, pack, config = cell
    config["flange_ft_sensor"]["measurement"] = "incoming_joint_wrench"
    report = robot.build_robot(stage, pack, config)
    view = Articulation(report)
    flat = view.wrench.reshape(-1).copy()
    view.get_link_incoming_joint_force = lambda: WarpLike(flat)
    result = sensors.read_wrench(stage, view, report, config, physics_time_s=0.5, backend="newton")
    assert result["status"] == "measured" and result["backend"] == "newton"
    assert result["force_n"] == pytest.approx([10, 0, 0], abs=1e-6)
    refused = sensors.read_wrench(stage, view, report, config, physics_time_s=0.5, backend="mujoco")
    assert refused["status"] == "unsupported" and refused["measurement"] is None


class Contacts:
    sensor_paths = ["/left", "/right"]

    def check(self):
        return True

    def get_net_contact_forces(self, dt):
        assert dt == 0.01
        return np.array([[1, 2, 3], [0, 0, 0]], dtype=np.float32)


def test_finger_measurement_is_explicit_and_never_fabricated():
    config = {"finger_force_sensors": {"fitted": True}}
    args = dict(physics_time_s=2, physics_dt_s=0.01, backend="physx")
    assert sensors.read_finger_contacts(Contacts(), config, **args)["measurement"] is None
    config["finger_force_sensors"].update(measurement="net_contact_force", body_paths=["/left", "/right"])
    result = sensors.read_finger_contacts(Contacts(), config, **args)
    assert result["measurement"] == [[1, 2, 3], [0, 0, 0]]
    assert result["quantity"] == "whole_finger_net_contact_force"
    assert sensors.read_finger_contacts(None, config, **args)["status"] == "unavailable"
    assert sensors.read_finger_contacts(Contacts(), config, **{**args, "backend": "newton"})["status"] == "unsupported"
    config["finger_force_sensors"]["body_paths"].reverse()
    assert sensors.read_finger_contacts(Contacts(), config, **args)["measurement"] is None


@pytest.mark.parametrize("method", ["check", "get_link_incoming_joint_force"])
def test_wrench_native_backend_failure_is_unavailable(cell, monkeypatch, method):
    stage, pack, config = cell
    report = robot.build_robot(stage, pack, config)
    config["flange_ft_sensor"]["measurement"] = "incoming_joint_wrench"
    view = Articulation(report)

    def fail():
        raise Exception("Failed to get incoming link joint forces from backend")

    monkeypatch.setattr(view, method, fail)
    result = sensors.read_wrench(stage, view, report, config, physics_time_s=1, backend="physx")
    assert result["status"] == "unavailable"
    assert result["measurement"] is None


@pytest.mark.parametrize("method", ["check", "get_net_contact_forces"])
def test_contact_native_backend_failure_is_unavailable(monkeypatch, method):
    config = {"finger_force_sensors": {"measurement": "net_contact_force",
                                       "body_paths": ["/left", "/right"]}}
    view = Contacts()

    def fail(**kwargs):
        raise Exception("Failed to get net contact forces from backend")

    monkeypatch.setattr(view, method, fail)
    result = sensors.read_finger_contacts(view, config, physics_time_s=1,
                                          physics_dt_s=0.01, backend="physx")
    assert result["status"] == "unavailable"
    assert result["measurement"] is None


def test_frame_only_body_and_dangling_disabled_joint_are_neutralized(cell):
    stage, pack, config = cell
    originals = {key: path.read_bytes() for key, path in pack.paths.items()}
    source = Usd.Stage.Open(str(pack.paths["pack/rg2"]))
    center = body(source, "/khi_rs080n/gripper_center", (0, 0, 0.2))
    joint = UsdPhysics.FixedJoint.Define(source, "/khi_rs080n/gripper_center_joint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/khi_rs080n/onrobot_rg2_base_link")])
    joint.CreateBody1Rel().SetTargets([center.GetPath()])
    source.GetPrimAtPath("/khi_rs080n").CreateAttribute(
        "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(16)
    source.GetRootLayer().Save()
    arm = Usd.Stage.Open(str(pack.paths["pack/robot"]))
    leftover = UsdPhysics.FixedJoint.Define(arm, "/Asset/robot_gripper_joint")
    leftover.CreateBody0Rel().SetTargets([Sdf.Path("/Asset/wrist_3_link")])
    leftover.CreateJointEnabledAttr().Set(False)
    arm.GetRootLayer().Save()
    saved = {key: path.read_bytes() for key, path in pack.paths.items()}

    report = robot.build_robot(stage, pack, config)

    assert report["frame_bodies_neutralized"] == [f"{report['gripper_path']}/gripper_center"]
    assert report["joints_deactivated"] == [f"{report['robot_path']}/robot_gripper_joint"]
    assert report["velocity_iterations_clamped"] == [report["gripper_path"]]
    assert stage.GetPrimAtPath(report["gripper_path"]).GetAttribute(
        "physxArticulation:solverVelocityIterationCount").Get() == 4
    frame = stage.GetPrimAtPath(report["frame_bodies_neutralized"][0])
    assert frame and not frame.HasAPI(UsdPhysics.RigidBodyAPI) and not frame.HasAPI(UsdPhysics.MassAPI)
    assert not stage.GetPrimAtPath(f"{report['gripper_path']}/gripper_center_joint").IsActive()
    assert not stage.GetPrimAtPath(report["joints_deactivated"][0]).IsActive()
    base = stage.GetPrimAtPath(f"{report['gripper_path']}/onrobot_rg2_base_link")
    assert base.HasAPI(UsdPhysics.RigidBodyAPI)  # the declared RG2 body is never neutralized
    for prim in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"])):
        if prim.IsA(UsdPhysics.Joint):
            assert UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()  # no dangling active joint
    assert {key: path.read_bytes() for key, path in pack.paths.items()} == saved != originals


def test_a_replaced_mass_keeps_the_shape_of_its_authored_inertia(cell):
    stage, pack, config = cell
    source = Usd.Stage.Open(str(pack.paths["pack/quick_changer"]))
    authored = next(p for p in source.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    UsdPhysics.MassAPI(authored).CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-4, 1e-4, 2e-4))
    source.Save()
    config["quick_changer"]["mass_kg"] = 0.2          # pack body 0.1 kg -> inertia doubles, same shape
    report = robot.build_robot(stage, pack, config)
    built = [p for p in Usd.PrimRange(stage.GetPrimAtPath(report["assembly_path"]))
             if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName() == authored.GetName()]
    api = UsdPhysics.MassAPI(built[0])
    assert api.GetMassAttr().Get() == pytest.approx(0.2)
    assert list(api.GetDiagonalInertiaAttr().Get()) == pytest.approx([2e-4, 2e-4, 4e-4])
