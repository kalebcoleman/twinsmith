"""CPU-only coverage for workcell preflight and tower layout."""

import asyncio
import importlib.util
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "exts/workcell/workcell"


@pytest.fixture()
def modules(monkeypatch):
    package = types.ModuleType("workcell_test")
    package.__path__ = [str(PACKAGE)]
    monkeypatch.setitem(sys.modules, "workcell_test", package)
    loaded = []
    for name in ("scene", "jenga"):
        spec = importlib.util.spec_from_file_location(f"workcell_test.{name}", PACKAGE / f"{name}.py")
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, spec.name, module)
        spec.loader.exec_module(module)
        loaded.append(module)
    return loaded


@pytest.mark.parametrize("disabled", [False, True], ids=["physics", "mirror_real"])
@pytest.mark.parametrize("asset_disabled", [False, True])
def test_robot_gravity_overrides_referenced_bodies_only(modules, monkeypatch, disabled, asset_disabled):
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import Sdf, UsdPhysics
    import pxr

    class PhysxRigidBodyAPI:
        @staticmethod
        def Apply(prim):
            return types.SimpleNamespace(CreateDisableGravityAttr=lambda: prim.CreateAttribute(
                "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool))

    monkeypatch.setattr(pxr, "PhysxSchema", types.SimpleNamespace(
        PhysxRigidBodyAPI=PhysxRigidBodyAPI), raising=False)
    scene, _ = modules
    asset = Usd.Stage.CreateInMemory()
    asset.SetDefaultPrim(asset.DefinePrim("/RG2", "Xform"))
    for index in range(7):
        body = asset.DefinePrim(f"/RG2/Body{index}", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(body)
        PhysxRigidBodyAPI.Apply(body).CreateDisableGravityAttr().Set(asset_disabled)
    asset_before = asset.GetRootLayer().ExportToString()
    stage = Usd.Stage.CreateInMemory()
    robot_layer = Sdf.Layer.CreateAnonymous()
    stage.GetSessionLayer().subLayerPaths.insert(0, robot_layer.identifier)
    with Usd.EditContext(stage, robot_layer):
        stage.DefinePrim("/World/RobotAssembly/rg2").GetReferences().AddReference(
            asset.GetRootLayer().identifier)
    for path in ("/World/Jenga/Block", "/World/RobotAssemblyOther/Body"):
        body = stage.DefinePrim(path, "Xform")
        UsdPhysics.RigidBodyAPI.Apply(body)
        PhysxRigidBodyAPI.Apply(body).CreateDisableGravityAttr().Set(False)
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityMagnitudeAttr().Set(9.81)
    root_before = stage.GetRootLayer().ExportToString()
    robot_before = robot_layer.ExportToString()

    scene._configure_robot_gravity(stage, "/World/RobotAssembly", disabled)

    for index in range(7):
        body = stage.GetPrimAtPath(f"/World/RobotAssembly/rg2/Body{index}")
        attribute = body.GetAttribute("physxRigidBody:disableGravity")
        assert attribute.Get() is disabled
        assert attribute.GetPropertyStack()[0].layer == stage.GetSessionLayer()
    assert not stage.GetPrimAtPath("/World/RobotAssembly/rg2").GetAttribute("physxRigidBody:disableGravity")
    assert stage.GetEditTarget().GetLayer() == stage.GetRootLayer()
    assert stage.GetRootLayer().ExportToString() == root_before
    assert robot_layer.ExportToString() == robot_before
    assert asset.GetRootLayer().ExportToString() == asset_before


def config(scene):
    return {"pack": {"id": "ur5e-jenga-workcell", "version": "v1"},
            "assets": {role: role for role in scene.ROLES},
            "world": {"workstation_xy_m": [0.2, -0.1], "bench_scale": 0.01,
                      "bench_asset_top_z_m": 0.8, "tabletop_z_m": 0.0},
            "jenga": {"block_size_m": [0.072, 0.024, 0.014], "block_mass_kg": 0.015,
                      "tower_layers_built": 18, "tower_xy_m": [0.7, -0.13],
                      "static_friction": 0.3, "dynamic_friction": 0.25}}


def install_contracts(monkeypatch, scene, tmp_path, values):
    monkeypatch.setattr(scene, "_bom_rows", lambda *args: [])
    hardware = types.ModuleType("workcell_test.hardware_config")
    hardware.load_config = lambda path: (values, {"provenance": True})
    pack_module = types.ModuleType("workcell_test.asset_pack")
    class Pack:
        manifest = {"id": "ur5e-jenga-workcell"}
        def verify_dependencies(self):
            return True
        def asset(self, logical_id):
            return tmp_path / logical_id
    pack_module.resolve_pack = lambda value: Pack()
    for role in scene.ROLES:
        (tmp_path / role).touch()
    monkeypatch.setitem(sys.modules, hardware.__name__, hardware)
    monkeypatch.setitem(sys.modules, pack_module.__name__, pack_module)


def test_preflight_resolves_all_roles(modules, monkeypatch, tmp_path):
    scene, _ = modules
    values = config(scene)
    install_contracts(monkeypatch, scene, tmp_path, values)
    result, raw, pack, assets = scene._preflight(None, "physics")
    assert result is values and raw["provenance"] and pack.manifest["id"]
    assert set(assets) == set(scene.ROLES)
    (tmp_path / "bench").unlink()
    with pytest.raises(ValueError, match="bench"):
        scene._preflight(None, "physics")


def test_bad_configuration_never_mutates_stage(modules, monkeypatch, tmp_path):
    scene, _ = modules
    values = config(scene)
    values["world"]["tabletop_z_m"] = 1
    install_contracts(monkeypatch, scene, tmp_path, values)
    with pytest.raises(ValueError, match="tabletop"):
        asyncio.run(scene.Workcell().quick_start())


def test_a_malformed_bench_top_never_mutates_stage(modules, monkeypatch, tmp_path):
    pytest.importorskip("pxr.Sdf")
    scene, _ = modules
    values = config(scene)
    values["bench_top"] = {"panel_path": "Panel", "color_rgb": [0.02, 0.02, 0.02], "roughness": 0.6,
                           "static_friction": 0.3, "dynamic_friction": 0.4}
    install_contracts(monkeypatch, scene, tmp_path, values)
    with pytest.raises(ValueError, match="bench_top.dynamic_friction"):
        asyncio.run(scene.Workcell().quick_start())


def test_unknown_task_never_reaches_the_stage(modules, monkeypatch, tmp_path):
    scene, _ = modules
    install_contracts(monkeypatch, scene, tmp_path, config(scene))
    with pytest.raises(ValueError, match="Unknown task"):
        asyncio.run(scene.Workcell().quick_start(task="no_such_task"))


def test_bom_path_is_relative_to_configuration(modules, monkeypatch, tmp_path):
    scene, _ = modules
    values = config(scene)
    values["documentation"] = {"bom_csv": "../parts.csv"}
    bom = types.ModuleType("workcell_test.bom")
    seen = []
    rows = [{"asset_id": "robot", "quantity": 1}]
    bom.load_bom = lambda path: seen.append(path.resolve()) or rows
    bom.verify_config_assets = lambda *args: None
    monkeypatch.setitem(sys.modules, bom.__name__, bom)
    assert scene._bom_rows(values, tmp_path / "config/cell.json") == rows
    assert seen == [tmp_path / "parts.csv"]


@pytest.mark.parametrize("condition,message", [
    ("idle", "Build the workcell"), ("stale", "stage changed"),
    ("playing", "Stop the timeline"), ("newton", "PhysX"),
])
def test_rebuild_refuses_unsafe_runtime(modules, monkeypatch, condition, message):
    scene, _ = modules
    stage = object()
    cell = scene.Workcell()
    if condition != "idle":
        cell._runtime = (stage, config(scene), {}, [])
        cell._report = {"state": "ready", "parts": [], "jenga": {}}
    omni = types.ModuleType("omni")
    omni.__path__ = []
    usd = types.ModuleType("omni.usd")
    usd.get_context = lambda: types.SimpleNamespace(
        get_stage=lambda: object() if condition == "stale" else stage)
    timeline = types.ModuleType("omni.timeline")
    timeline.get_timeline_interface = lambda: types.SimpleNamespace(is_stopped=lambda: condition != "playing")
    omni.usd, omni.timeline = usd, timeline
    physics = types.ModuleType("workcell_test.physics_compat")
    physics.selected_engine = lambda: "newton" if condition == "newton" else "physx"
    for module in (omni, usd, timeline, physics):
        monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(scene, "_replace_tower", lambda *args: pytest.fail("must not replace tower"))
    with pytest.raises(RuntimeError, match=message):
        cell.build_jenga()


@pytest.mark.parametrize("failure", ["build", "audit"])
def test_rebuild_failure_preserves_original_layer(modules, monkeypatch, failure):
    Usd = pytest.importorskip("pxr.Usd")
    scene, jenga = modules
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/Jenga/Original", "Xform")
    before = stage.GetRootLayer().ExportToString()

    def build(temporary, *args, **kwargs):
        if failure == "build":
            raise ValueError("build failed")
        temporary.DefinePrim("/World/Jenga/Replacement", "Xform")
        return {"parts": []}

    def audit(*args):
        assert stage.GetPrimAtPath("/World/Jenga/Replacement")
        raise ValueError("audit failed")

    monkeypatch.setattr(jenga, "build", build)
    monkeypatch.setattr(scene, "_finish_scene", audit)
    with pytest.raises(ValueError, match=failure):
        scene._replace_tower(stage, config(scene), {"jenga_block": "unused"}, [], [])
    assert stage.GetRootLayer().ExportToString() == before


def test_tower_layout_and_cached_status(modules):
    scene, jenga = modules
    poses = jenga.positions(config(scene))
    assert len(poses) == 54
    assert poses[0][0][2] == pytest.approx(0.007)
    assert poses[3][1] == 90 and poses[3][0][2] == pytest.approx(0.021)
    cell = scene.Workcell()
    report = cell.status()
    report["state"] = "changed"
    assert cell.status()["state"] == "idle"
    assert cell.shutdown()["stage_preserved"]


ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def newton_section(**drive):
    values = dict({"stiffness_nm_per_rad": 5000, "damping_nms_per_rad": 50, "armature_kgm2": 0.1}, **drive)
    return {"physics_hz": 500, "impratio": 10, "drives": {joint: dict(values) for joint in ARM}}


def load_newton_cell():
    spec = importlib.util.spec_from_file_location("workcell_test_newton_cell", PACKAGE / "newton_cell.py")
    cell = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cell)
    return cell


@pytest.mark.parametrize("values,mode,message", [
    ({}, "physics", "no newton section"),
    ({"newton": {"physics_hz": 500}}, "physics", "Invalid newton section"),
    ({"newton": newton_section(stiffness_nm_per_rad=0)}, "physics", "Invalid newton section"),
    ({"newton": newton_section()}, "mirror_real", "mirror_real"),
    ({"newton": newton_section(), "asset_layout": {"rg2": {"articulation_root": "."}}}, "physics", "separate RG2"),
])
def test_newton_refusals_come_before_stage_access(modules, monkeypatch, tmp_path, values, mode, message):
    scene, _ = modules
    install_contracts(monkeypatch, scene, tmp_path, {**config(scene), **values})
    physics = types.ModuleType("workcell_test.physics_compat")
    physics.selected_engine = lambda: "newton"
    monkeypatch.setitem(sys.modules, physics.__name__, physics)
    with pytest.raises(RuntimeError, match=message):
        asyncio.run(scene.Workcell().quick_start(mode=mode))


def test_newton_drives_need_all_six_arm_joints_and_report_their_margin():
    cell = load_newton_cell()
    section = newton_section()
    assert set(cell.validate({"newton": section})) == set(ARM)
    assert cell.stability(section["drives"], 500)["wrist_1_joint"] == {"ratio": 0.2, "limit": 6.0}   # dt^2 k / I, 4 + 2 dt c / I
    partial = dict(section, drives={"wrist_1_joint": section["drives"]["wrist_1_joint"]})
    for bad in ({}, {"newton": {}}, {"newton": partial}, {"newton": dict(section, impratio=0)},
                {"newton": dict(section, drives=[1])}, {"newton": newton_section(armature_kgm2=0)}):
        with pytest.raises(ValueError):
            cell.validate(bad)


def test_prepare_writes_per_degree_gains_and_both_armatures(monkeypatch):
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdPhysics

    cell = load_newton_cell()
    stage = Usd.Stage.CreateInMemory()
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
    scene.CreateAttribute("mjc:option:impratio", __import__("pxr").Sdf.ValueTypeNames.Float).Set(1.0)
    for name in ARM:
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/RobotAssembly/robot/joints/{name}").GetPrim()
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateStiffnessAttr(9400.0)
        drive.CreateDampingAttr(0.4)
    box = stage.DefinePrim("/World/Bench/box", "Cube")
    UsdPhysics.CollisionAPI.Apply(box)
    wood = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim("/World/Wood", "Material"))
    wood.CreateStaticFrictionAttr(0.3)
    wood.CreateDynamicFrictionAttr(0.25)
    seen = []
    support = types.ModuleType("sim_tools.newton_support")
    support.scene_settings = lambda prim, rate, tower: {"newton:timeStepsPerSecond": rate}
    support.collision_settings = lambda prim: seen.append(str(prim.GetPath()))
    pending = types.SimpleNamespace(solver_outputs=types.SimpleNamespace(link_incoming_joint_force=False))
    applied = []
    support.prepare_buffers = lambda *args: (pending, {"rigid_bodies": 0})
    support.apply_buffers = lambda config: applied.append(config.solver_outputs.link_incoming_joint_force)
    package = types.ModuleType("sim_tools")
    package.__path__ = []
    monkeypatch.setitem(sys.modules, "sim_tools", package)
    monkeypatch.setitem(sys.modules, "sim_tools.newton_support", support)
    report = cell.prepare(stage, {"newton": newton_section(stiffness_nm_per_rad=25000, damping_nms_per_rad=300),
                                  "contact": {"newton_friction": "static"}},
                          "/World/RobotAssembly", timeline=None)
    assert wood.GetDynamicFrictionAttr().Get() == pytest.approx(0.3) and report["materials_with_static_mu"] == 1
    joint = stage.GetPrimAtPath("/World/RobotAssembly/robot/joints/elbow_joint")
    assert joint.GetAttribute("drive:angular:physics:stiffness").Get() == pytest.approx(25000 * 3.141592653589793 / 180)
    assert joint.GetAttribute("drive:angular:physics:damping").Get() == pytest.approx(300 * 3.141592653589793 / 180)
    assert joint.GetAttribute("newton:armature").Get() == joint.GetAttribute("physxJoint:armature").Get() == pytest.approx(0.1)
    assert scene.GetAttribute("mjc:option:impratio").Get() == 10.0 and seen == ["/World/Bench/box"]
    assert set(report["drives"]) == set(ARM) and report["colliders_with_contact_gap"] == 1
    assert applied == [True] and report["buffers"]["link_incoming_joint_force"] is True   # the Bota reading on Newton


def test_a_bench_without_its_own_colliders_is_refused_on_newton(modules):
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdGeom

    scene, _ = modules
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Mesh.Define(stage, "/World/Bench/Frame/Mesh_0")
    with pytest.raises(ValueError, match="overflow Newton"):
        scene.bench_colliders(stage, engine="newton")
    assert not stage.GetPrimAtPath("/World/Bench/Frame/Mesh_0").GetAppliedSchemas()       # refused before authoring


def test_unsaved_stage_refused_without_replacement(modules, monkeypatch, tmp_path):
    scene, jenga = modules
    install_contracts(monkeypatch, scene, tmp_path, config(scene))
    physics = types.ModuleType("workcell_test.physics_compat")
    physics.selected_engine = lambda: "physx"
    monkeypatch.setitem(sys.modules, physics.__name__, physics)
    for name, function in (("robot", "build_robot"), ("cameras", "build_cameras")):
        module = types.ModuleType(f"workcell_test.{name}")
        setattr(module, function, lambda *args: None)
        monkeypatch.setitem(sys.modules, module.__name__, module)
    class Context:
        def get_stage(self):
            return object()
        async def new_stage_async(self):
            pytest.fail("stage must not be replaced")
    omni = types.ModuleType("omni")
    omni.__path__ = []
    usd = types.ModuleType("omni.usd")
    usd.get_context = Context
    timeline = types.ModuleType("omni.timeline")
    timeline.get_timeline_interface = lambda: types.SimpleNamespace(is_stopped=lambda: True)
    omni.usd, omni.timeline = usd, timeline
    for module in (omni, usd, timeline):
        monkeypatch.setitem(sys.modules, module.__name__, module)
    sim_tools = types.ModuleType("sim_tools")
    sim_tools.__path__ = []
    stage_open = types.ModuleType("sim_tools.stage_open")
    stage_open.stage_has_unsaved_work = lambda stage: True
    monkeypatch.setitem(sys.modules, "sim_tools", sim_tools)
    monkeypatch.setitem(sys.modules, "sim_tools.stage_open", stage_open)
    pxr = types.ModuleType("pxr")
    for name in ("Gf", "UsdGeom", "UsdLux", "UsdPhysics"):
        setattr(pxr, name, types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "pxr", pxr)
    with pytest.raises(RuntimeError, match="UNSAVED_CHANGES"):
        asyncio.run(scene.Workcell().quick_start())


def test_bench_keeps_its_own_collision_layer_and_else_gets_triangle_meshes(modules):
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdGeom, UsdPhysics

    scene, _ = modules
    plain = Usd.Stage.CreateInMemory()
    mesh = UsdGeom.Mesh.Define(plain, "/World/Bench/Frame/Mesh_0")
    assert scene.bench_colliders(plain) == "triangle meshes"
    assert mesh.GetPrim().GetAttribute("physics:approximation").Get() == "none"

    split = Usd.Stage.CreateInMemory()
    render = UsdGeom.Mesh.Define(split, "/World/Bench/Frame/Mesh_0")
    box = UsdGeom.Cube.Define(split, "/World/Bench/Frame/Mesh_0_collision/box")
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    assert scene.bench_colliders(split) == "own"
    assert not render.GetPrim().HasAPI(UsdPhysics.CollisionAPI)

    with pytest.raises(ValueError):
        scene.bench_colliders(Usd.Stage.CreateInMemory())


def test_scene_rows_leave_out_the_overhead_hardware_when_it_is_off(modules):
    scene, _ = modules
    roles = ("robot", "d415", "wrist_mount", "overhead_mount", "overhead_release", "overhead_shelf")
    rows = [{"bom_id": role, "asset_id": role, "quantity": 2 if role == "d415" else 1} for role in roles]
    config = {"assets": {role: role for role in roles}, "overhead_camera": {"enabled": True}}
    assert scene.scene_rows(config, rows) == rows
    config["overhead_camera"]["enabled"] = False
    assert {row["bom_id"]: row["quantity"] for row in scene.scene_rows(config, rows)} == {
        "robot": 1, "d415": 1, "wrist_mount": 1}
    assert rows[1]["quantity"] == 2     # the BOM rows themselves are not changed
