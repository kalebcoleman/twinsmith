"""mesh_ops: decimation, oriented boxes, convex hulls and CoACD on plain arrays."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("trimesh")
pytest.importorskip("coacd")
pytest.importorskip("fast_simplification")

from mesh_fixture import l_mesh  # noqa: E402

from twinsmith import mesh_ops  # noqa: E402


@pytest.fixture(scope="module")
def mesh():
    return l_mesh()


def test_fixture_is_a_closed_l_of_about_20k_triangles(mesh):
    info = mesh_ops.inspect(*mesh)
    assert 19_000 <= info["triangles"] <= 21_000 and info["watertight"]
    assert info["bounds"] == [[0.0, 0.0, 0.0], [2.0, 2.0, 0.2]]


def test_decimate_reaches_the_target_and_keeps_the_bounds(mesh):
    v, f = mesh_ops.decimate(*mesh, target_tris=2000)
    assert abs(len(f) - 2000) <= 100
    lo, hi = v.min(axis=0), v.max(axis=0)
    size = np.ptp(mesh[0], axis=0)
    assert np.all(np.abs(lo - mesh[0].min(axis=0)) <= 0.01 * size) and np.all(np.abs(hi - mesh[0].max(axis=0)) <= 0.01 * size)
    v2, f2 = mesh_ops.decimate(*mesh, ratio=0.5)
    assert abs(len(f2) - len(mesh[1]) / 2) <= 0.05 * len(mesh[1]) / 2


def test_decimate_arguments_are_checked(mesh):
    for kwargs in ({}, dict(target_tris=10, ratio=0.5), dict(ratio=0), dict(ratio=1.5), dict(target_tris=2)):
        with pytest.raises(ValueError):
            mesh_ops.decimate(*mesh, **kwargs)


def _inside_box(points, shape, tol=1e-6):
    w, x, y, z = shape["orientation"]
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    local = (points - np.asarray(shape["center"])) @ r
    return np.all(np.abs(local) <= np.asarray(shape["half_extents"]) + tol)


def test_box_encloses_every_point(mesh):
    rotated = mesh[0] @ np.array([[0.6, -0.8, 0], [0.8, 0.6, 0], [0, 0, 1]]) + [1, 2, 3]
    shape = mesh_ops.box(rotated)
    assert _inside_box(rotated, shape)
    assert mesh_ops.box_volume(shape) == pytest.approx(2 * 2 * 0.2, rel=0.02)


def test_coacd_gives_at_most_max_hulls_convex_pieces(mesh):
    import trimesh

    pieces = mesh_ops.coacd(*mesh, max_hulls=4, threshold=0.05)
    assert 1 <= len(pieces) <= 4
    for v, f in pieces:
        part = trimesh.Trimesh(v, f)
        assert part.is_convex or part.convex_hull.volume == pytest.approx(part.volume, rel=1e-3)


def test_auto_keeps_a_box_unless_it_is_over_twice_the_hull(mesh):
    assert mesh_ops.collision_shapes(*mesh, "auto", max_hulls=4)[0]["kind"] == "box"   # an L's box is 8/7 of its hull
    # a regular tetrahedron fills a third of its tightest box, so auto decomposes it
    tetra = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]], dtype=float)
    faces = np.array([[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
    shapes = mesh_ops.collision_shapes(tetra, faces, "auto", max_hulls=4)
    assert shapes and all(s["kind"] == "hull" for s in shapes)
    with pytest.raises(ValueError):
        mesh_ops.collision_shapes(tetra, faces, "sphere")


def test_flat_meshes_get_an_axis_aligned_box_by_every_method():
    plate = np.array([[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]], dtype=float)
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    for method in ("box", "convex", "coacd", "auto"):
        shapes = mesh_ops.collision_shapes(plate, faces, method)
        assert [s["kind"] for s in shapes] == ["box"]
        assert np.allclose(shapes[0]["half_extents"], [1, 0.5, 0])


def test_a_tilted_plate_gets_a_thin_box_in_its_plane_not_its_axis_aligned_box():
    tilt = np.array([[1, 0, 0], [0, 0.6, -0.8], [0, 0.8, 0.6]])
    plate = np.array([[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]], dtype=float) @ tilt.T + [1, 2, 3]
    for method in ("box", "auto"):
        shape = mesh_ops.collision_shapes(plate, [[0, 1, 2], [0, 2, 3]], method)[0]
        assert shape["kind"] == "box" and _inside_box(plate, shape)
        assert sorted(shape["half_extents"]) == pytest.approx([0, 0.5, 1], abs=1e-9)


def test_degenerate_points_get_an_enclosing_box_of_zero_volume():
    triangle = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    line = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]], dtype=float)
    point = np.ones((5, 3))
    duplicates = np.vstack([triangle, triangle])                  # six points, still a plane
    for pts in (triangle, line, point, duplicates):
        for method in ("box", "convex", "coacd", "auto"):
            [shape] = mesh_ops.collision_shapes(pts, [[0, 1, 2]], method)
            assert shape["kind"] == "box" and _inside_box(pts, shape) and mesh_ops.box_volume(shape) == pytest.approx(0, abs=1e-12)
    assert sorted(mesh_ops.box(line)["half_extents"]) == pytest.approx([0, 0, 1.5 * 3 ** 0.5])
    assert mesh_ops.box(point) == dict(center=[1.0, 1.0, 1.0], half_extents=[0.0, 0.0, 0.0], orientation=[1.0, 0, 0, 0])


def test_empty_or_broken_input_is_a_clear_error():
    with pytest.raises(ValueError, match="at least one point"):
        mesh_ops.box(np.zeros((0, 3)))
    with pytest.raises(ValueError, match="no points"):
        mesh_ops.collision_shapes(np.zeros((0, 3)), np.zeros((0, 3), dtype=int), "box")
    with pytest.raises(ValueError, match="does not exist"):
        mesh_ops.collision_shapes(np.zeros((3, 3)), [[0, 1, 3]], "box")


def test_duplicate_points_do_not_change_the_box_or_the_hull():
    tetra = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]], dtype=float)
    twice = np.vstack([tetra, tetra])
    assert mesh_ops.box_volume(mesh_ops.box(twice)) == pytest.approx(mesh_ops.box_volume(mesh_ops.box(tetra)))
    hv, hf = mesh_ops.convex(twice)
    assert len(hv) == 4 and mesh_ops.hull_volume(hv, hf) == pytest.approx(8 / 3)


def test_coacd_with_one_hull_gives_one_piece(mesh):
    assert len(mesh_ops.coacd(*mesh, max_hulls=1)) == 1
    for bad in (0, 257):
        with pytest.raises(ValueError):
            mesh_ops.coacd(*mesh, max_hulls=bad)


def test_decimate_leaves_a_mesh_already_at_or_under_the_target():
    triangle = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    v, f = mesh_ops.decimate(triangle, [[0, 1, 2]], ratio=0.5)       # ratio never asks for fewer than 4 triangles
    assert np.array_equal(v, triangle) and f.tolist() == [[0, 1, 2]]
