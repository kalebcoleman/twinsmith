"""The work cell's quick_start summary, the room around it and the black tabletop (CPU only; pxr for the USD parts)."""

import asyncio
import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "exts/workcell/workcell"


def load(name):
    spec = importlib.util.spec_from_file_location(f"workcell_room_test_{name}", PACKAGE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def full_report():
    blocks = [{"prim_path": f"/World/Jenga/Block_{i:02d}", "mass_kg": 0.015, "static_friction": 0.3} for i in range(54)]
    return {
        "state": "ready", "mode": "physics", "backend": "newton", "timeline": "stopped", "gravity_m_s2": -9.81,
        "robot_gravity_disabled": False, "tabletop_z_m": 0.0,
        "pack": {"id": "ur5e-jenga-workcell", "version": "v2", "qualification": "candidate",
                 "files": [{"path": f"f{i}.usda", "sha256": "0" * 64} for i in range(16)]},
        "robot": {"assembly_path": "/World/RobotAssembly", "robot_path": "/World/RobotAssembly/robot",
                  "articulation_root": "/World/RobotAssembly/robot/root_joint", "tool0_path": "/t", "gripper_path": "/g",
                  "max_opening_m": 0.11, "parts": [{"prim_path": "/p"}] * 7, "limitations": ["x"] * 16},
        "cameras": {"wrist": {"status": "built", "prim_path": "/w", "intrinsics": {"fx": 1.0} | {f"k{i}": i for i in range(40)}},
                    "overhead": {"status": "built", "prim_path": "/o"}},
        "jenga": {"block_count": 54, "layers": 18, "variation_profile": "p", "variation_seed": 42, "blocks": blocks,
                  "parts": blocks},
        "parts": [{"prim_path": f"/p{i}"} for i in range(68)],
        "bom_audit": {"ok": True, "errors": [], "bom_ids": {"jenga_block": 54}},
        "newton": {"settings": {"newton:timeStepsPerSecond": 500},
                   "buffers": {"rigid_bodies": 70, "nconmax": 2240, "njmax": 17920, "warnings": [], "sizing": "..."}},
        "environment": {"name": "simple_room", "deactivated_for_overlap": ["table_low_327"]},
        "limitations": ["Pack fidelity is declared."] * 36 + ["Newton: drives differ.", "Newton: cameras not qualified."],
    }


def test_quick_start_returns_a_summary_and_status_the_full_report():
    commands_module = load("commands")
    commands = commands_module.Commands()
    report = full_report()
    calls = []

    async def quick_start(**kwargs):
        calls.append(kwargs)
        return report

    commands.workcell = types.SimpleNamespace(quick_start=quick_start, status=lambda: report)
    summary = asyncio.run(commands._cmd_workcell_quick_start("c.json", "physics", True, "simple_room", "push_block"))
    assert calls == [{"config_path": "c.json", "mode": "physics", "discard_changes": True, "environment": "simple_room",
                      "task": "push_block"}]
    assert len(json.dumps(summary)) < len(json.dumps(report)) / 4
    assert summary["state"] == "ready" and summary["backend"] == "newton"
    assert summary["pack"] == {"id": "ur5e-jenga-workcell", "version": "v2", "qualification": "candidate"}
    assert summary["robot"]["articulation_root"] == "/World/RobotAssembly/robot/root_joint"
    assert summary["cameras"] == {"wrist": {"status": "built", "prim_path": "/w"},
                                  "overhead": {"status": "built", "prim_path": "/o"}}
    assert summary["jenga"]["block_count"] == 54 and "blocks" not in summary["jenga"]
    assert summary["parts"] == 68 and summary["bom_audit"] == {"ok": True, "errors": []}
    assert summary["newton"] == {"steps_per_second": 500, "rigid_bodies": 70, "nconmax": 2240, "njmax": 17920, "warnings": []}
    assert summary["environment"]["deactivated_for_overlap"] == ["table_low_327"]
    assert summary["limitations"]["count"] == 38 and len(summary["limitations"]["engine"]) == 2
    assert 'action="status"' in summary["full_report"]
    assert commands._cmd_workcell_status() is report     # status is unchanged: the whole report


def test_a_report_that_is_not_ready_passes_through():
    summarize = load("commands").summarize
    failed = {"state": "failed", "error": "boom", "stage_replaced": True}
    assert summarize(failed) == failed


def test_environment_names_and_refusals():
    rooms = load("environment")
    assert rooms.NAMES == ("none", "simple_room")
    assert rooms.resolve("none") is None and rooms.resolve(None) is None
    with pytest.raises(ValueError, match="environment must be one of"):
        rooms.resolve("warehouse", assets_root="https://x", reachable=lambda url: True)
    with pytest.raises(RuntimeError, match="no NVIDIA assets root"):
        rooms.resolve("simple_room", assets_root="", reachable=lambda url: True)
    with pytest.raises(RuntimeError, match="not reachable.*no stage was replaced"):
        rooms.resolve("simple_room", assets_root="https://assets/Isaac/6.2", reachable=lambda url: False)
    seen = []
    url = rooms.resolve("simple_room", assets_root="https://assets/Isaac/6.2/", reachable=lambda u: seen.append(u) or True)
    assert url == seen[0] == "https://assets/Isaac/6.2/Isaac/Environments/Simple_Room/simple_room.usd"


def _room_file(tmp_path):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    room = Usd.Stage.CreateNew(str(tmp_path / "room.usda"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    root = UsdGeom.Xform.Define(room, "/Room")
    room.SetDefaultPrim(root.GetPrim())
    UsdGeom.Xform.Define(room, "/Room/GroundPlane")
    plane = UsdGeom.Plane.Define(room, "/Room/GroundPlane/CollisionPlane")
    plane.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.2))          # the room's floor is 0.2 m above its origin
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    for name, centre, size in (("Table", (0, 0, 0.6), (1.6, 1.0, 0.8)), ("Wall", (4, 0, 1.5), (0.2, 8, 3)),
                               ("Floor", (0, 0, 0.1), (10, 10, 0.2))):
        UsdGeom.Xform.Define(room, f"/Room/{name}")
        cube = UsdGeom.Cube.Define(room, f"/Room/{name}/box")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    room.Save()
    return str(tmp_path / "room.usda")


def _cell(tmp_path):
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Bench")
    leg = UsdGeom.Cube.Define(stage, "/World/Bench/leg")     # the bench: 2 x 1 m, from z=-0.8 (feet) to 0 (top)
    leg.CreateSizeAttr(1.0)
    leg.AddTranslateOp().Set(Gf.Vec3d(0, -0.3, -0.4))
    leg.AddScaleOp().Set(Gf.Vec3f(2.0, 1.0, 0.8))
    return stage


def test_the_room_stands_on_the_bench_floor_and_gives_way_to_it(tmp_path):
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom, UsdPhysics

    rooms = load("environment")
    stage = _cell(tmp_path)
    report = rooms.place(stage, "simple_room", _room_file(tmp_path))
    assert report["path"] == "/World/Environment/simple_room"
    assert report["floor_z_m"] == pytest.approx(-0.8) and report["lifted_m"] == pytest.approx(-1.0)
    plane = stage.GetPrimAtPath("/World/Environment/simple_room/GroundPlane/CollisionPlane")
    z = UsdGeom.Xformable(plane).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2]
    assert z == pytest.approx(-0.8)                         # the room's ground is level with the bench's feet
    assert report["deactivated_for_overlap"] == ["Table"]   # the floor slab under it and the far wall stay
    assert not stage.GetPrimAtPath("/World/Environment/simple_room/Table").IsActive()
    assert stage.GetPrimAtPath("/World/Environment/simple_room/Wall").IsActive()
    assert report["colliders"]["ground_plane"].endswith("GroundPlane/CollisionPlane")
    assert UsdPhysics.CollisionAPI(plane).GetCollisionEnabledAttr().Get() in (None, True)
    for name in ("Wall", "Floor"):
        box = stage.GetPrimAtPath(f"/World/Environment/simple_room/{name}/box")
        assert UsdPhysics.CollisionAPI(box).GetCollisionEnabledAttr().Get() is False


def test_the_tabletop_gets_its_colour_on_the_mesh_and_its_friction_on_the_collider():
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    scene = load("scene")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/Bench/Panel")
    UsdGeom.Mesh.Define(stage, "/World/Bench/Panel/Mesh_91")
    box = UsdGeom.Cube.Define(stage, "/World/Bench/Panel/Mesh_91_collision/box")
    box.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    top = {"panel_path": "Panel", "color_rgb": [0.02, 0.02, 0.02], "roughness": 0.6,
           "static_friction": 0.3, "dynamic_friction": 0.25}
    report = scene.bench_top(stage, top)
    assert report["render_meshes"] == ["/World/Bench/Panel/Mesh_91"]
    assert report["colliders"] == ["/World/Bench/Panel/Mesh_91_collision/box"]
    material = UsdShade.Material.Get(stage, report["material"])
    physics = UsdPhysics.MaterialAPI(material.GetPrim())
    assert physics.GetStaticFrictionAttr().Get() == pytest.approx(0.3)
    assert physics.GetDynamicFrictionAttr().Get() == pytest.approx(0.25)
    mesh_binding = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath("/World/Bench/Panel/Mesh_91")).ComputeBoundMaterial()[0]
    collider_binding = UsdShade.MaterialBindingAPI(box.GetPrim()).ComputeBoundMaterial(materialPurpose="physics")[0]
    assert mesh_binding.GetPath() == collider_binding.GetPath() == material.GetPath()
    with pytest.raises(ValueError, match="not a prim under"):
        scene.bench_top(stage, dict(top, panel_path="Missing"))


def test_the_shipped_config_has_a_black_top_and_the_robot_facing_the_table():
    sys.path.insert(0, str(PACKAGE))
    try:
        hardware_config = load("hardware_config")
        values, raw = hardware_config.load_config()
    finally:
        sys.path.remove(str(PACKAGE))
    assert max(values["bench_top"]["color_rgb"]) < 0.1
    assert 0 < values["bench_top"]["dynamic_friction"] <= values["bench_top"]["static_friction"]
    assert values["robot"]["placement"]["orientation_wxyz"] == [0, 0, 0, 1]   # 180 degrees about z


def test_a_room_with_its_own_physics_bodies_instancing_and_lights_is_neutralised(tmp_path):
    """Simple Room 6.2 has none of these, but another room may: a second physics scene makes Newton refuse, a loose
    body falls on Play, an instanced collider cannot be switched off, and room lights change the cell's renders."""
    pytest.importorskip("pxr.Usd")
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    room = Usd.Stage.Open(_room_file(tmp_path))
    UsdPhysics.Scene.Define(room, "/Room/physicsScene")
    UsdLux.RectLight.Define(room, "/Room/Lamp")
    crate = UsdGeom.Cube.Define(room, "/Room/Crate")
    crate.AddTranslateOp().Set(Gf.Vec3d(3, 3, 0.7))
    UsdPhysics.RigidBodyAPI.Apply(crate.GetPrim())
    UsdPhysics.CollisionAPI.Apply(crate.GetPrim())
    UsdGeom.Xform.Define(room, "/Room/Shelf")
    shelf = UsdGeom.Cube.Define(room, "/Room/Shelf/board")
    shelf.AddTranslateOp().Set(Gf.Vec3d(-3, 3, 0.7))
    UsdPhysics.CollisionAPI.Apply(shelf.GetPrim())
    room.GetPrimAtPath("/Room/Shelf").SetInstanceable(True)
    room.Save()

    rooms = load("environment")
    stage = _cell(tmp_path)
    report = rooms.place(stage, "simple_room", str(tmp_path / "room.usda"))
    base = "/World/Environment/simple_room"
    assert report["physics_scenes_deactivated"] == 1 and not stage.GetPrimAtPath(f"{base}/physicsScene").IsActive()
    assert report["lights_deactivated"] == 1 and not stage.GetPrimAtPath(f"{base}/Lamp").IsActive()
    assert report["rigid_bodies_made_kinematic"] == 1
    assert UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"{base}/Crate")).GetKinematicEnabledAttr().Get() is True
    assert report["deinstanced"] == 1
    board = stage.GetPrimAtPath(f"{base}/Shelf/board")
    assert not board.IsInstanceProxy()
    assert UsdPhysics.CollisionAPI(board).GetCollisionEnabledAttr().Get() is False
    assert [p for p in Usd.PrimRange(stage.GetPseudoRoot()) if p.IsA(UsdPhysics.Scene) and p.IsActive()] == []


@pytest.mark.parametrize("change, message", [
    ({"color_rgb": [0.1, 0.1]}, "three numbers"),
    ({"color_rgb": [0.1, 0.1, 2.0]}, "three numbers"),
    ({"dynamic_friction": 0.5}, "must not exceed"),
    ({"static_friction": float("nan")}, "finite"),
    ({"roughness": 1.5}, "roughness must be from 0 to 1"),
    ({"panel_path": "/World/Bench/Panel"}, "relative"),
    ({"panel_path": "a b/../c"}, "not a valid USD prim path"),
    ({"color_rgb": [True, False, True]}, "three numbers"),
    ({"extra": 1}, "needs exactly"),
])
def test_a_malformed_bench_top_is_refused_before_the_stage_is_touched(change, message):
    pytest.importorskip("pxr.Sdf")
    scene = load("scene")
    top = {"panel_path": "Panel", "color_rgb": [0.02, 0.02, 0.02], "roughness": 0.6,
           "static_friction": 0.3, "dynamic_friction": 0.25}
    scene.check_bench_top(top)
    with pytest.raises(ValueError, match=message):
        scene.check_bench_top(dict(top, **change))
    without = dict(top)
    del without["roughness"]
    with pytest.raises(ValueError, match="needs exactly"):
        scene.check_bench_top(without)


def test_bodies_below_the_sensor_follow_the_joint_chain_downward():
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom, UsdPhysics

    scene = load("scene")
    stage = Usd.Stage.CreateInMemory()
    for name in ("wrist", "bota", "changer", "gripper", "finger"):
        UsdGeom.Xform.Define(stage, f"/R/{name}")
    for parent, child in (("wrist", "bota"), ("bota", "changer"), ("changer", "gripper"), ("gripper", "finger")):
        joint = UsdPhysics.FixedJoint.Define(stage, f"/R/joints/{child}")
        joint.CreateBody0Rel().SetTargets([f"/R/{parent}"])
        joint.CreateBody1Rel().SetTargets([f"/R/{child}"])
    assert sorted(scene.bodies_below(stage, "/R/bota")) == ["bota", "changer", "finger", "gripper"]
    assert scene.bodies_below(stage, "/R/finger") == ["finger"]


def test_the_static_wrench_check_passes_only_within_tolerance():
    spec = importlib.util.spec_from_file_location("static_wrench", ROOT / "scripts/workcell_static_wrench.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reading = {"status": "measured", "force_n": [0.0, 0.0, 15.0], "torque_nm": [0, 0, 0],
               "load_below_sensor": {"weight_n": 15.009, "mass_kg": 1.53}}
    assert module.check(reading, 0.01, 0.02)["pass"]
    assert not module.check(dict(reading, force_n=[0.0, 0.0, 16.0]), 0.01, 0.02)["pass"]
    assert not module.check({"status": "unavailable", "reason": "not initialized"}, 0.01, 0.02)["pass"]
    down = dict(reading, force_n=[-15.0, 0.0, 0.03])
    assert module.check(down, 0.01, 0.02, axis=(0, -1.0))["pass"]
    assert not module.check(dict(down, force_n=[15.0, 0.0, 0.03]), 0.01, 0.02, axis=(0, -1.0))["pass"]   # sign flipped
    assert not module.check(dict(down, force_n=[0.0, -15.0, 0.0]), 0.01, 0.02, axis=(0, -1.0))["pass"]  # wrong axis


def test_articulated_props_nested_instances_emissive_meshes_and_the_floor_holder(tmp_path):
    pytest.importorskip("pxr.Usd")
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    room = Usd.Stage.Open(_room_file(tmp_path))
    door = UsdGeom.Cube.Define(room, "/Room/Door/leaf")       # a hinged door: an articulation in the room
    door.AddTranslateOp().Set(Gf.Vec3d(3, -3, 1))
    UsdPhysics.RigidBodyAPI.Apply(door.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(room.GetPrimAtPath("/Room/Door"))
    hinge = UsdPhysics.RevoluteJoint.Define(room, "/Room/Door/hinge")
    hinge.CreateBody1Rel().SetTargets(["/Room/Door/leaf"])
    glow = UsdGeom.Mesh.Define(room, "/Room/GlowPanel")       # an emissive mesh, not a light prim
    UsdLux.MeshLightAPI.Apply(glow.GetPrim())
    UsdGeom.Xform.Define(room, "/Room/Outer/Inner")           # an instance inside an instance
    board = UsdGeom.Cube.Define(room, "/Room/Outer/Inner/board")
    board.AddTranslateOp().Set(Gf.Vec3d(-3, -3, 0.7))
    UsdPhysics.CollisionAPI.Apply(board.GetPrim())
    room.GetPrimAtPath("/Room/Outer/Inner").SetInstanceable(True)
    room.GetPrimAtPath("/Room/Outer").SetInstanceable(True)
    big = UsdGeom.Cube.Define(room, "/Room/GroundPlane/slab")  # the floor holder reaches into the bench's box
    big.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.6))
    big.AddScaleOp().Set(Gf.Vec3f(3, 3, 0.8))
    room.Save()

    rooms = load("environment")
    stage = _cell(tmp_path)
    UsdLux.DistantLight.Define(stage, "/World/Light")
    report = rooms.place(stage, "simple_room", str(tmp_path / "room.usda"))
    base = "/World/Environment/simple_room"
    assert report["joints_deactivated"] == 1 and not stage.GetPrimAtPath(f"{base}/Door/hinge").IsActive()
    assert report["articulation_roots_removed"] == 1
    assert not stage.GetPrimAtPath(f"{base}/Door").HasAPI(UsdPhysics.ArticulationRootAPI)
    assert stage.GetPrimAtPath(f"{base}/GlowPanel").IsActive() and report["lights_deactivated"] == 0
    assert report["deinstanced"] == 2
    nested = stage.GetPrimAtPath(f"{base}/Outer/Inner/board")
    assert not nested.IsInstanceProxy() and UsdPhysics.CollisionAPI(nested).GetCollisionEnabledAttr().Get() is False
    assert "GroundPlane" not in report["deactivated_for_overlap"] and "Table" in report["deactivated_for_overlap"]
    assert stage.GetPrimAtPath(f"{base}/GroundPlane").IsActive()
    assert stage.GetPrimAtPath("/World/Light").IsActive()      # the cell's own lights are never touched


CONTACT = {"default_static_friction": 0.5, "default_dynamic_friction": 0.5, "combine_mode": "max",
           "newton_friction": "static"}


def test_every_collider_gets_a_material_that_combines_like_mujoco_and_does_not_bounce():
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    scene = load("scene")
    stage = Usd.Stage.CreateInMemory()
    bare = UsdGeom.Cube.Define(stage, "/World/Robot/link").GetPrim()
    UsdPhysics.CollisionAPI.Apply(bare)
    wood = UsdShade.Material.Define(stage, "/World/Wood")
    UsdPhysics.MaterialAPI.Apply(wood.GetPrim()).CreateStaticFrictionAttr(0.3)
    block = UsdGeom.Cube.Define(stage, "/World/Block").GetPrim()
    UsdPhysics.CollisionAPI.Apply(block)
    UsdShade.MaterialBindingAPI.Apply(block).Bind(wood, materialPurpose="physics")
    report = scene.contact_materials(stage, CONTACT)
    assert report["default_bound_to"] == 1 and report["materials"] == 2
    bound, _ = UsdShade.MaterialBindingAPI(bare).ComputeBoundMaterial(materialPurpose="physics")
    assert bound.GetPath() == "/World/CellDefaultContactMaterial"
    kept, _ = UsdShade.MaterialBindingAPI(block).ComputeBoundMaterial(materialPurpose="physics")
    assert kept.GetPath() == "/World/Wood"
    for material in ("/World/Wood", "/World/CellDefaultContactMaterial"):
        prim = stage.GetPrimAtPath(material)
        assert prim.GetAttribute("physxMaterial:frictionCombineMode").Get() == "max"
        assert "PhysxMaterialAPI" in prim.GetMetadata("apiSchemas").GetAddedOrExplicitItems()   # hidden by usd-core
        assert UsdPhysics.MaterialAPI(prim).GetRestitutionAttr().Get() == 0.0


@pytest.mark.parametrize("change, message", [
    ({"combine_mode": "sum"}, "combine_mode"), ({"combine_mode": "average"}, "must be max"),
    ({"newton_friction": "mean"}, "newton_friction"),
    ({"default_dynamic_friction": 0.6}, "must not exceed"), ({"default_static_friction": True}, "finite"),
])
def test_a_malformed_contact_section_is_refused(change, message):
    scene = load("scene")
    scene.check_contact(CONTACT)
    with pytest.raises(ValueError, match=message):
        scene.check_contact(dict(CONTACT, **change))


def _robot_module():
    pytest.importorskip("pxr.Usd")
    sys.path.insert(0, str(PACKAGE))
    try:
        spec = importlib.util.spec_from_file_location("workcell.robot", PACKAGE / "robot.py",
                                                      submodule_search_locations=None)
        package = types.ModuleType("workcell")
        package.__path__ = [str(PACKAGE)]
        sys.modules.setdefault("workcell", package)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(PACKAGE))


GOOD = {"mass_kg": 1.37, "com_m": [0.0, -0.01634, -0.0018], "diagonal_inertia_kgm2": [0.0012, 0.0014, 0.0017],
        "principal_axes_wxyz": [1, 0, 0, 0]}


@pytest.mark.parametrize("change, message", [
    ({"mass_kg": 0}, "positive"), ({"com_m": [0, 0]}, "malformed"), ({"diagonal_inertia_kgm2": [0.001, 0.001, 0.01]}, "triangle"),
    ({"principal_axes_wxyz": [1, 1, 0, 0]}, "unit quaternion"), ({"mass_kg": float("inf")}, "non-finite"),
    ({"extra": 1}, "needs exactly"),
])
def test_malformed_inertials_are_refused(change, message):
    robot = _robot_module()
    robot.check_inertials({"arm": {"wrist_1_link": GOOD}})
    with pytest.raises(ValueError, match=message):
        robot.check_inertials({"arm": {"wrist_1_link": dict(GOOD, **change)}})
    with pytest.raises(ValueError, match="twice"):
        robot.check_inertials({"a": {"wrist_1_link": GOOD}, "b": {"wrist_1_link": GOOD}})


def test_inertials_are_authored_on_the_named_body_and_a_missing_one_is_refused():
    robot = _robot_module()
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/A")
    body = UsdGeom.Xform.Define(stage, "/A/robot/wrist_1_link").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    assert robot._inertials(stage, "/A", {"arm": {"wrist_1_link": GOOD}}) == {"wrist_1_link": 1.37}
    api = UsdPhysics.MassAPI(body)
    assert api.GetMassAttr().Get() == pytest.approx(1.37)
    assert list(api.GetCenterOfMassAttr().Get()) == pytest.approx([0.0, -0.01634, -0.0018])
    assert list(api.GetDiagonalInertiaAttr().Get()) == pytest.approx([0.0012, 0.0014, 0.0017])
    with pytest.raises(ValueError, match="expected one rigid body"):
        robot._inertials(stage, "/A", {"arm": {"wrist_2_link": GOOD}})


def test_the_shipped_inertials_are_ur5e_and_the_datasheet_rg2():
    sys.path.insert(0, str(PACKAGE))
    try:
        values, raw = load("hardware_config").load_config()
    finally:
        sys.path.remove(str(PACKAGE))
    arm = values["inertials"]["ur5e_links"]
    assert [arm[f"{n}_link"]["mass_kg"] for n in ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")] == [
        3.761, 8.058, 2.846, 1.37, 1.3, 0.365]                      # UR's UR5e table, not the CB-series UR5
    assert sum(link["mass_kg"] for link in values["inertials"]["rg2_links"].values()) == pytest.approx(0.78, abs=1e-5)
    assert values["bota"]["mass_kg"] == 0.113 and values["bota"]["sensor_origin_along_axis_m"] == 0.032
    assert raw["inertials"]["ur5e_links"]["status"] == "confirmed" and raw["inertials"]["rg2_links"]["status"] == "unconfirmed"


def test_a_scalar_vector_is_a_value_error_not_a_type_error():
    robot = _robot_module()
    for key in ("com_m", "diagonal_inertia_kgm2", "principal_axes_wxyz"):
        with pytest.raises(ValueError, match="malformed vector"):
            robot.check_inertials({"arm": {"wrist_1_link": dict(GOOD, **{key: 0.0})}})


def test_a_missing_link_on_a_multi_link_stage_is_refused_and_authors_nothing():
    robot = _robot_module()
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    for name in ("wrist_1_link", "wrist_2_link"):
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage, f"/A/robot/{name}").GetPrim())
    with pytest.raises(ValueError, match="wrist_3_link: expected one rigid body"):
        robot._inertials(stage, "/A", {"arm": {"wrist_1_link": GOOD, "wrist_3_link": GOOD}})


def test_preflight_link_names_are_checked_against_the_asset_files(tmp_path):
    robot = _robot_module()
    from pxr import Usd, UsdGeom, UsdPhysics

    paths = []
    for file, names in (("arm.usda", ("wrist_1_link", "wrist_2_link")), ("gripper.usda", ("finger", "finger"))):
        asset = Usd.Stage.CreateNew(str(tmp_path / file))
        for index, name in enumerate(names):
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(asset, f"/R/part{index}/{name}").GetPrim())
        asset.Save()
        paths.append(tmp_path / file)
    robot.check_inertial_links({"arm": {"wrist_1_link": GOOD}}, paths)
    with pytest.raises(ValueError, match="wrist_9_link: expected one rigid body .* found 0"):
        robot.check_inertial_links({"arm": {"wrist_9_link": GOOD}}, paths)
    with pytest.raises(ValueError, match="finger: expected one rigid body .* found 2"):
        robot.check_inertial_links({"gripper": {"finger": GOOD}}, paths)
