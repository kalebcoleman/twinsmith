"""mesh_usd: inspect, decimate, the collision layer and split, on a generated asset (the source is never changed)."""

from __future__ import annotations

import hashlib

import numpy as np
import pytest

pytest.importorskip("pxr")
pytest.importorskip("trimesh")
pytest.importorskip("coacd")

import mesh_fixture  # noqa: E402

from twinsmith import mesh_usd  # noqa: E402


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def asset(tmp_path):
    path = tmp_path / "src" / "bench.usdc"
    path.parent.mkdir()
    info = mesh_fixture.build(path)
    return path, info


def test_inspect_reports_meshes_colliders_and_triangle_mesh_totals(asset):
    path, info = asset
    report = mesh_usd.inspect(path)
    row = next(r for r in report["meshes"] if r["prim"] == info["mesh"])
    assert row["collider"] and row["approximation"] == "none" and row["watertight"]
    assert report["totals"]["triangle_mesh_collider_triangles"] == info["triangles"] == row["triangles"]
    assert report["meters_per_unit"] == 1.0 and report["up_axis"] == "Z"


def test_split_composes_render_and_collision_and_leaves_the_source(asset, tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    path, info = asset
    before = digest(path)
    out = tmp_path / "out"
    out.mkdir()
    result = mesh_usd.split(path, out, collision=dict(method="box"))
    assert digest(path) == before
    source, composed = Usd.Stage.Open(str(path)), Usd.Stage.Open(result["output"])
    bodies = lambda s: sorted(str(p.GetPath()) for p in s.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) or p.IsA(UsdPhysics.Joint))  # noqa: E731
    assert bodies(composed) == bodies(source) and bodies(source)
    assert composed.GetDefaultPrim().GetName() == "World" and UsdGeom.GetStageUpAxis(composed) == "Z"
    render = Usd.Stage.Open(result["render"])
    assert not mesh_usd.collision_schemas(render.GetPrimAtPath(info["mesh"]))      # replaced by the box
    assert mesh_usd.collision_schemas(render.GetPrimAtPath("/World/Robot/Link0/shape"))  # not a mesh: kept
    assert result["shapes"]["selected"] == "mesh colliders" and result["shapes"]["replaced"] == [info["mesh"]]
    assert result["shapes"]["method"] == "box" and result["shapes"]["triangles"] == 0      # a box is analytic
    assert result["shapes"]["source_triangles"] == len(mesh_usd.triangulate(
        *[getattr(UsdGeom.Mesh(source.GetPrimAtPath(info["mesh"])), a)().Get() for a in ("GetFaceVertexCountsAttr", "GetFaceVertexIndicesAttr")]))
    collision = Usd.Stage.Open(result["collision"])
    shapes = [p for p in collision.Traverse() if mesh_usd.collision_schemas(p)]
    assert shapes and all(UsdGeom.Imageable(p).ComputePurpose() == UsdGeom.Tokens.guide for p in shapes)
    # the box of the L sits where the L is: compare world bounds in the composed stage
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    mesh_box = cache.ComputeWorldBound(composed.GetPrimAtPath(info["mesh"])).ComputeAlignedRange()
    shape_box = cache.ComputeWorldBound(composed.GetPrimAtPath(info["mesh"] + "_collision")).ComputeAlignedRange()
    assert np.allclose(mesh_box.GetMin(), shape_box.GetMin(), atol=1e-3) and np.allclose(mesh_box.GetMax(), shape_box.GetMax(), atol=1e-3)
    # the composed asset's colliders are the explicit shapes only
    colliders = [str(p.GetPath()) for p in composed.Traverse() if mesh_usd.collision_schemas(p)]
    assert info["mesh"] not in colliders and info["mesh"] + "_collision/box" in colliders
    with pytest.raises(FileExistsError):
        mesh_usd.split(path, out)


def rendered(stage):
    """What a renderer sees: every prim that is not guide purpose, with its type, non-physics schemas and values."""
    from pxr import UsdGeom

    seen = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Imageable) and UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.guide:
            continue
        values = {a.GetName(): np.asarray(a.Get()).tobytes() if hasattr(a.Get(), "__len__") and not isinstance(a.Get(), str)
                  else repr(a.Get()) for a in prim.GetAttributes() if a.HasAuthoredValue() and not a.GetName().startswith("physics:")}
        seen[str(prim.GetPath())] = (prim.GetTypeName(), [n for n in prim.GetAppliedSchemas() if "Physics" not in n], values)
    return seen


def test_split_renders_what_the_source_renders(asset, tmp_path):
    from pxr import Usd

    path, _ = asset
    for method in ("box", "auto"):
        out = tmp_path / method
        out.mkdir()
        composed = Usd.Stage.Open(mesh_usd.split(path, out, collision=dict(method=method))["output"])
        assert rendered(composed) == rendered(Usd.Stage.Open(str(path)))


def test_hulls_with_coacd_writes_convex_pieces(asset, tmp_path):
    from pxr import Usd, UsdGeom

    path, info = asset
    dest = tmp_path / "bench_collision.usdc"
    result = mesh_usd.author_collision_layer(mesh_usd.open_stage(path), dest, method="coacd", max_hulls=4,
                                             prims=[info["mesh"]])
    assert result["meshes"] == 1 and 1 <= result["hulls"] <= 4 and result["boxes"] == 0
    layer = Usd.Stage.Open(str(dest))
    hulls = [p for p in layer.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(hulls) == result["hulls"]
    assert all(p.GetAttribute("physics:approximation").Get() == "convexHull" for p in hulls)
    assert result["triangles"] == sum(len(p.GetAttribute("faceVertexCounts").Get()) for p in hulls) > 0
    assert (result["method"], result["max_hulls"]) == ("coacd", 4)


def test_decimate_writes_a_lighter_copy(asset, tmp_path):
    path, info = asset
    out = tmp_path / "dec"
    out.mkdir()
    before = digest(path)
    result = mesh_usd.decimate(path, out, ratio=0.1, prims=[info["mesh"]])
    assert digest(path) == before
    assert result["triangles_after"] <= 0.11 * result["triangles_before"]
    report = mesh_usd.inspect(result["output"])
    assert next(r for r in report["meshes"] if r["prim"] == info["mesh"])["triangles"] == result["triangles_after"]


def test_triangulate_fans_polygons_and_drops_degenerate_faces():
    tris = mesh_usd.triangulate([4, 2, 3], [0, 1, 2, 3, 4, 5, 6, 7, 8])
    assert tris.tolist() == [[0, 1, 2], [0, 2, 3], [6, 7, 8]]


def test_schemas_without_a_plugin_are_still_seen_and_stripped(tmp_path):
    """usd-core has no PhysX schemas: GetAppliedSchemas hides PhysxCollisionAPI, the apiSchemas list does not."""
    from pxr import Usd, UsdGeom

    path = tmp_path / "physx.usda"
    stage = Usd.Stage.CreateNew(str(path))
    mesh = UsdGeom.Mesh.Define(stage, "/W/m")
    mesh.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
    assert "PhysxCollisionAPI" not in mesh.GetPrim().GetAppliedSchemas()
    assert mesh_usd.collision_schemas(mesh.GetPrim()) == ["PhysxCollisionAPI"]
    assert mesh_usd.strip_collision(stage, ["/W/m"]) == 1 and mesh_usd.collision_schemas(mesh.GetPrim()) == []


def _quad(stage, path, z=0.0):
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(0, 0, z), (1, 0, z), (1, 1, z), (0, 1, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    return mesh


def test_split_of_a_flat_root_mesh_keeps_its_box_inside_the_default_prim(tmp_path):
    from pxr import Usd, UsdGeom

    src = tmp_path / "plate.usda"
    stage = Usd.Stage.CreateNew(str(src))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(_quad(stage, "/plate").GetPrim())
    stage.GetRootLayer().Save()
    out = tmp_path / "out"
    out.mkdir()
    result = mesh_usd.split(src, out, collision=dict(method="coacd"))     # flat: every method falls back to the box
    assert result["shapes"]["boxes"] == 1 and result["shapes"]["hulls"] == 0 and result["shapes"]["thickened"] == 1
    composed = Usd.Stage.Open(result["output"])
    box = composed.GetPrimAtPath("/plate/plate_collision/box")
    # in metres, the plate's box gets the minimum thickness, so it keeps a volume and a mass
    assert min(box.GetAttribute("xformOp:scale").Get()) == pytest.approx(mesh_usd.MIN_HALF_THICKNESS_M)


def test_non_mesh_colliders_need_prims_named(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    src = tmp_path / "link.usda"
    stage = Usd.Stage.CreateNew(str(src))
    _quad(stage, "/link/visual", 0.5)
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Capsule.Define(stage, "/link/collider").GetPrim())
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="prims"):
        mesh_usd.targets(mesh_usd.open_stage(src))
    rows, selected = mesh_usd.targets(mesh_usd.open_stage(src), ["/link/visual"])
    assert selected == "prims" and len(rows) == 1


def test_copies_keep_unprefixed_relative_references(tmp_path):
    from pxr import Usd, UsdGeom

    folder = tmp_path / "src"
    folder.mkdir()
    geom = Usd.Stage.CreateNew(str(folder / "geom.usda"))
    geom.SetDefaultPrim(UsdGeom.Xform.Define(geom, "/g").GetPrim())
    _quad(geom, "/g/m")
    geom.GetRootLayer().Save()
    top = Usd.Stage.CreateNew(str(folder / "top.usda"))
    ref = UsdGeom.Xform.Define(top, "/A/ref").GetPrim()
    ref.GetReferences().AddReference("geom.usda")                 # no ./ prefix
    top.GetRootLayer().Save()
    out = tmp_path / "out"
    out.mkdir()
    copy = mesh_usd.copy_root_layer(folder / "top.usda", out / "top.usda")
    assert copy.GetPrimAtPath("/A/ref/m")


def test_decimate_skips_instance_proxies_and_blocks_subsets(tmp_path):
    from pxr import Usd, UsdGeom

    import mesh_fixture

    src = tmp_path / "inst.usda"
    stage = Usd.Stage.CreateNew(str(src))
    v, f = mesh_fixture.l_mesh(20)
    proto = UsdGeom.Mesh.Define(stage, "/Proto/m")
    proto.CreatePointsAttr([tuple(p) for p in v])
    proto.CreateFaceVertexCountsAttr([3] * len(f))
    proto.CreateFaceVertexIndicesAttr(f.reshape(-1).tolist())
    UsdGeom.Subset.CreateGeomSubset(proto, "half", UsdGeom.Tokens.face, list(range(len(f) // 2)), "materialBind")
    inst = UsdGeom.Xform.Define(stage, "/Inst").GetPrim()
    inst.GetReferences().AddInternalReference("/Proto")
    inst.SetInstanceable(True)
    stage.GetRootLayer().Save()
    out = tmp_path / "dec"
    out.mkdir()
    result = mesh_usd.decimate(src, out, ratio=0.2)
    assert result["meshes"] == 1 and result["skipped_instance_proxies"] == ["/Inst/m"]
    copy = Usd.Stage.Open(result["output"])
    subset = UsdGeom.Subset(copy.GetPrimAtPath("/Proto/m/half"))
    assert subset.GetIndicesAttr().Get() is None
