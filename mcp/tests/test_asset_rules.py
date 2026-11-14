"""asset_rules.validate: the rule table on a generated asset with known faults, before and after fixing them."""

from __future__ import annotations

import shutil

import pytest

pytest.importorskip("pxr")
pytest.importorskip("trimesh")

import mesh_fixture  # noqa: E402

from twinsmith import asset_rules, mesh_usd  # noqa: E402


def failed(report):
    return {c["id"] for c in report["checks"] if not c["ok"]}


@pytest.fixture
def asset(tmp_path):
    path = tmp_path / "src" / "cell.usdc"
    path.parent.mkdir()
    mesh_fixture.build(path)
    return path


def test_every_rule_has_levels_evidence_and_a_fix():
    for rule, spec in asset_rules.RULES.items():
        assert spec["newton"] in ("error", "warning") and spec["physx"] in ("error", "warning", "info")
        assert spec["rule"] and spec["evidence"] and asset_rules.FIXES[rule]


def test_fixture_fails_on_newton_and_passes_after_split_and_fixes(asset, tmp_path):
    report = asset_rules.validate(asset, "newton", triangle_budget=5000)
    assert not report["ok"]
    assert {"TRI_COLLIDER", "DRIVE_ZERO_FORCE", "NESTED_ARTICULATION"} <= failed(report)
    by_id = {c["id"]: c for c in report["checks"] if not c["ok"]}
    assert by_id["DRIVE_ZERO_FORCE"]["prim"] == "/World/Robot/joint1" and by_id["DRIVE_ZERO_FORCE"]["fix"]
    assert by_id["NESTED_ARTICULATION"]["prim"] == "/World/Robot/Link1/Gripper"

    fixed = tmp_path / "fixed" / "cell.usdc"
    fixed.parent.mkdir()
    shutil.copy(asset, fixed)
    mesh_fixture.remove_faults(fixed)
    out = tmp_path / "split"
    out.mkdir()
    composed = mesh_usd.split(fixed, out, collision=dict(method="box"))["output"]
    after = asset_rules.validate(composed, "newton", triangle_budget=5000)
    assert after["ok"], [c for c in after["checks"] if not c["ok"]]
    assert {c["id"] for c in after["checks"]} == set(asset_rules.RULES)


def test_physx_reports_newton_only_rules_as_info(asset):
    report = asset_rules.validate(asset, "physx", triangle_budget=5000)
    levels = {c["id"]: c["level"] for c in report["checks"] if not c["ok"]}
    assert levels["TRI_COLLIDER"] == levels["DRIVE_ZERO_FORCE"] == levels["NESTED_ARTICULATION"] == "info"
    assert report["ok"]


def test_units_mass_gain_and_mimic_rules(tmp_path):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    path = tmp_path / "bare.usda"
    stage = Usd.Stage.CreateNew(str(path))
    body = UsdGeom.Xform.Define(stage, "/A/Body")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())              # moving, no mass, no collider
    other = UsdGeom.Xform.Define(stage, "/A/Other")
    UsdPhysics.RigidBodyAPI.Apply(other.GetPrim())
    UsdPhysics.MassAPI.Apply(other.GetPrim()).CreateMassAttr(1.0)   # mass but no inertia and no collider
    leader = UsdPhysics.RevoluteJoint.Define(stage, "/A/leader")
    leader.CreateBody0Rel().SetTargets([body.GetPath()])
    leader.CreateBody1Rel().SetTargets([other.GetPath()])
    UsdPhysics.DriveAPI.Apply(leader.GetPrim(), "angular").CreateStiffnessAttr(9400.0)
    loop = UsdPhysics.RevoluteJoint.Define(stage, "/A/loop")
    loop.CreateBody0Rel().SetTargets([body.GetPath()])
    loop.CreateBody1Rel().SetTargets([other.GetPath()])
    loop.CreateExcludeFromArticulationAttr(True)
    loop.GetPrim().AddAppliedSchema("NewtonMimicAPI")          # a mimic on a loop joint: PhysX rejects it
    loop.GetPrim().CreateRelationship("newton:mimicJoint").SetTargets([leader.GetPath()])
    stage.GetRootLayer().Save()
    report = asset_rules.validate(path, "newton")
    assert {"UNITS", "MASS_MIN", "DRIVE_GAIN", "MIMIC"} <= failed(report)
    assert {c["prim"] for c in report["checks"] if c["id"] == "MASS_MIN" and not c["ok"]} == {"/A/Body", "/A/Other"}
    gain = next(c for c in report["checks"] if c["id"] == "DRIVE_GAIN" and not c["ok"])
    assert "armature 0.1" in gain["detail"] and "500 Hz" in gain["detail"] and gain["level"] == "warning"
    mimic = next(c for c in report["checks"] if c["id"] == "MIMIC")["detail"]
    assert "excluded" in mimic and "follower is a revolute joint without finite limits" in mimic
    # Armature and damping make the same stiffness stable.
    leader.GetPrim().CreateAttribute("newton:armature", Sdf.ValueTypeNames.Float).Set(5.0)
    stage.GetRootLayer().Save()
    assert "DRIVE_GAIN" not in failed(asset_rules.validate(path, "newton"))


def test_an_excluded_loop_joint_without_a_mimic_is_valid(asset, tmp_path):
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(asset))
    j = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/loop")
    j.CreateBody0Rel().SetTargets(["/World/Robot/Link0"])
    j.CreateBody1Rel().SetTargets(["/World/Robot/Link1"])
    j.CreateExcludeFromArticulationAttr(True)
    stage.GetRootLayer().Save()
    assert "MIMIC" not in failed(asset_rules.validate(asset, "newton", triangle_budget=10**7))


def test_a_triangle_mesh_on_a_moving_body_is_flagged(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    path = tmp_path / "dyn.usda"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    body = UsdGeom.Xform.Define(stage, "/B")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/B/m")
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)])
    mesh.CreateFaceVertexCountsAttr([3, 3, 3, 3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    other = UsdGeom.Mesh.Define(stage, "/B/only_mesh_collision_api")       # MeshCollisionAPI alone is no collider
    UsdPhysics.MeshCollisionAPI.Apply(other.GetPrim()).CreateApproximationAttr("none")
    stage.GetRootLayer().Save()
    report = asset_rules.validate(path, "physx")
    dynamic = [c for c in report["checks"] if c["id"] == "TRI_DYNAMIC" and not c["ok"]]
    assert [c["prim"] for c in dynamic] == ["/B/m"] and dynamic[0]["level"] == "warning"


def test_a_collision_shape_that_renders_is_a_warning(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    path = tmp_path / "shape.usda"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    cube = UsdGeom.Cube.Define(stage, "/W/part_collision/box")
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    stage.GetRootLayer().Save()
    report = asset_rules.validate(path)
    assert failed(report) == {"COLLIDER_RENDERED"} and report["ok"]


def test_engine_is_checked(asset):
    with pytest.raises(ValueError):
        asset_rules.validate(asset, "mujoco")


def new_stage(tmp_path, name="t.usda"):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(tmp_path / name))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, "Z")
    return stage


def check(stage, engine="newton", **kwargs):
    stage.GetRootLayer().Save()
    return asset_rules.validate(stage.GetRootLayer().realPath, engine, **kwargs)


def findings(report, rule):
    return {c["prim"]: c["detail"] for c in report["checks"] if c["id"] == rule and not c["ok"]}


def body(stage, path, *, mass=None, inertia=None):
    from pxr import UsdGeom, UsdPhysics

    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    if inertia is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateDiagonalInertiaAttr(inertia)
    return prim


def triangle(stage, path, *, collider=True):
    from pxr import UsdGeom, UsdPhysics

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    if collider:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh.GetPrim()


# At 500 Hz dt^2 = 4e-6, so a drive is flagged once k >= 4 I / 4e-6 (no damping); I = body + armature 0.1.
@pytest.mark.parametrize("kind, axis, k, mass, inertia, flagged", [
    ("PrismaticJoint", "linear", 2e6, 1.0, None, True),        # N/m against the body's mass: 2e6 >= 1.1e6
    ("PrismaticJoint", "linear", 2e6, 10.0, None, False),      # the heavier body takes it: 2e6 < 1.01e7
    ("Joint", "transX", 2e6, 1.0, None, True),                 # a D6 translation drive is linear
    ("Joint", "rotX", 20000.0, None, (1, 1, 1), True),         # per degree: 1.15e6 N m/rad >= 1.1e6
    ("Joint", "rotX", 20000.0, None, (2, 1, 1), False),        # the largest inertia is used
    ("RevoluteJoint", "angular", 0.0, None, (1, 1, 1), False),  # no stiffness, nothing to integrate
])
def test_drive_gain_uses_the_driven_bodys_mass_or_inertia(tmp_path, kind, axis, k, mass, inertia, flagged):
    from pxr import UsdPhysics

    stage = new_stage(tmp_path)
    link = body(stage, "/R/link", mass=mass, inertia=inertia)
    joint = getattr(UsdPhysics, kind).Define(stage, "/R/joint")
    joint.CreateBody0Rel().SetTargets([link.GetPath()])          # body0 only: jointed to the world
    UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis).CreateStiffnessAttr(k)
    assert bool(findings(check(stage), "DRIVE_GAIN")) is flagged


def test_an_authored_zero_armature_is_not_replaced_by_the_default(tmp_path):
    from pxr import Sdf, UsdPhysics

    stage = new_stage(tmp_path)
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/j")      # no body: I = 0.01; k = 500 / deg is stable only with 0.1
    UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular").CreateStiffnessAttr(500.0)
    assert not findings(check(stage), "DRIVE_GAIN")
    physx = joint.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float)
    physx.Set(0.0)
    assert "armature 0," in findings(check(stage), "DRIVE_GAIN")["/j"]
    physx.Set(5.0)
    assert not findings(check(stage), "DRIVE_GAIN")
    joint.GetPrim().CreateAttribute("newton:armature", Sdf.ValueTypeNames.Float).Set(0.0)   # Newton's own value wins
    assert "armature 0," in findings(check(stage), "DRIVE_GAIN")["/j"]


def test_mass_min_skips_still_bodies_and_counts_only_the_bodys_own_colliders(tmp_path):
    from pxr import UsdGeom, UsdPhysics

    stage = new_stage(tmp_path)
    body(stage, "/K").GetAttribute("physics:kinematicEnabled").Set(True)
    body(stage, "/D").GetAttribute("physics:rigidBodyEnabled").Set(False)
    body(stage, "/Zero", mass=0.0)                              # mass 0: computed from the collider
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/Zero/c").GetPrim())
    UsdPhysics.MassAPI.Apply(body(stage, "/Density")).CreateDensityAttr(1000.0)   # density needs a collider
    off = UsdGeom.Cube.Define(stage, "/Off/c").GetPrim()
    body(stage, "/Off")
    UsdPhysics.CollisionAPI.Apply(off).CreateCollisionEnabledAttr(False)
    body(stage, "/Parent")                                      # its only collider belongs to the nested body
    body(stage, "/Parent/Child")
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/Parent/Child/c").GetPrim())
    assert set(findings(check(stage), "MASS_MIN")) == {"/Density", "/Off", "/Parent"}


def test_separate_articulations_are_not_nested_unless_jointed(tmp_path):
    from pxr import UsdGeom, UsdPhysics

    stage = new_stage(tmp_path)
    for root in ("/W/Robot", "/W/Robot2"):                     # siblings; one name is a prefix of the other
        UsdPhysics.ArticulationRootAPI.Apply(UsdGeom.Xform.Define(stage, root).GetPrim())
        body(stage, f"{root}/L", mass=1.0, inertia=(1, 1, 1))
    joint = UsdPhysics.FixedJoint.Define(stage, "/W/tie")
    joint.CreateBody0Rel().SetTargets(["/W/Robot/L"])
    joint.CreateBody1Rel().SetTargets(["/W/Robot2/L"])
    joint.CreateExcludeFromArticulationAttr(True)
    assert not findings(check(stage), "NESTED_ARTICULATION")
    joint.GetExcludeFromArticulationAttr().Set(False)
    assert findings(check(stage), "NESTED_ARTICULATION") == {"/W/tie": "joins the articulations at /W/Robot, /W/Robot2"}


def test_physx_mimic_needs_exactly_one_leader_in_the_file(tmp_path):
    from pxr import UsdPhysics

    stage = new_stage(tmp_path)
    UsdPhysics.PrismaticJoint.Define(stage, "/M/lead")
    for name, leaders in (("zero", []), ("two", ["/M/lead", "/M/lead2"]), ("missing", ["/M/nowhere"]), ("good", ["/M/lead"])):
        prim = UsdPhysics.PrismaticJoint.Define(stage, f"/M/{name}").GetPrim()   # prismatic: no limits needed
        prim.AddAppliedSchema("PhysxMimicJointAPI:rotZ")
        prim.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets(leaders)
    UsdPhysics.PrismaticJoint.Define(stage, "/M/lead2")
    mimic = findings(check(stage, "physx"), "MIMIC")
    assert set(mimic) == {"/M/zero", "/M/two", "/M/missing"}
    assert "names 0 leaders" in mimic["/M/zero"] and "names 2 leaders" in mimic["/M/two"]
    assert "/M/nowhere is not a joint in this file" in mimic["/M/missing"]


def test_triangle_budget_counts_instances_and_only_triangle_mesh_colliders(tmp_path):
    from pxr import UsdPhysics

    stage = new_stage(tmp_path)
    stage.CreateClassPrim("/Proto")
    triangle(stage, "/Proto/m")
    for name in ("I1", "I2"):                                  # each instance is a collider in the scene
        prim = stage.DefinePrim(f"/W/{name}", "Xform")
        prim.GetReferences().AddInternalReference("/Proto")
        prim.SetInstanceable(True)
    UsdPhysics.CollisionAPI(triangle(stage, "/W/off")).CreateCollisionEnabledAttr(False)
    UsdPhysics.MeshCollisionAPI.Apply(triangle(stage, "/W/hull")).CreateApproximationAttr("convexHull")
    body(stage, "/W/K").GetAttribute("physics:kinematicEnabled").Set(True)
    triangle(stage, "/W/K/m")                                  # a triangle mesh on a kinematic body is fine
    at_budget = check(stage, triangle_budget=3)
    line = next(c for c in at_budget["checks"] if c["id"] == "TRI_COLLIDER")
    assert line["ok"] and line["detail"].startswith("3 triangles in 3 triangle-mesh colliders")
    assert not findings(at_budget, "TRI_DYNAMIC")
    assert "TRI_COLLIDER" in failed(check(stage, triangle_budget=2))


def test_units_names_what_is_missing(tmp_path):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(tmp_path / "mpu.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    assert findings(check(stage), "UNITS") == {None: "missing upAxis"}


def test_a_clean_file_has_one_passing_line_per_rule_and_totals_add_up(tmp_path, asset):
    clean = check(new_stage(tmp_path))
    assert clean["ok"] and [c["id"] for c in clean["checks"]] == list(asset_rules.RULES)
    assert all(c["ok"] and c["fix"] is None for c in clean["checks"])
    assert clean["totals"] == dict(errors=0, warnings=0, info=0, checks=len(asset_rules.RULES))
    report = asset_rules.validate(asset, "newton", triangle_budget=5000)
    bad = [c for c in report["checks"] if not c["ok"]]
    t = report["totals"]
    assert t["errors"] + t["warnings"] + t["info"] == len(bad) and t["checks"] == len(report["checks"])
    for rule in set(asset_rules.RULES) - failed(report):
        assert [c["ok"] for c in report["checks"] if c["id"] == rule] == [True]
