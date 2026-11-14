"""Mesh operations on plain arrays: inspect, decimate, and collision shapes (box, convex hull, CoACD).

No USD and no file I/O: ``mesh_usd`` reads and writes the prims. Every function takes ``vertices``
(N x 3 floats) and ``faces`` (M x 3 vertex indices) and needs the optional ``mesh`` extra
(trimesh, coacd, fast-simplification, numpy).
"""

from __future__ import annotations

import numpy as np


def _as_arrays(vertices, faces):
    v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    f = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    if len(f) and (f.min() < 0 or f.max() >= len(v)):
        raise ValueError("a face names a vertex that does not exist")
    return v, f


def _trimesh(v, f):
    import trimesh

    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def inspect(vertices, faces) -> dict:
    """Triangle count, axis-aligned bounds, and whether the surface is closed."""
    v, f = _as_arrays(vertices, faces)
    if not len(v):
        return dict(triangles=0, vertices=0, bounds=None, watertight=False)
    return dict(triangles=int(len(f)), vertices=int(len(v)),
                bounds=[v.min(axis=0).round(9).tolist(), v.max(axis=0).round(9).tolist()],
                watertight=bool(len(f) and _trimesh(v, f).is_watertight))


def decimate(vertices, faces, *, target_tris: int | None = None, ratio: float | None = None):
    """Quadric decimation to about ``target_tris`` triangles, or to ``ratio`` of the current count."""
    import fast_simplification

    v, f = _as_arrays(vertices, faces)
    if (target_tris is None) == (ratio is None):
        raise ValueError("give exactly one of target_tris or ratio")
    if ratio is not None and not 0 < ratio <= 1:
        raise ValueError("ratio must be greater than 0 and at most 1")
    target = int(target_tris) if target_tris is not None else max(4, round(len(f) * ratio))
    if target < 4:
        raise ValueError("target_tris must be at least 4")
    if target >= len(f):
        return v, f
    out_v, out_f = fast_simplification.simplify(v, f, target_reduction=1.0 - target / len(f))
    return np.asarray(out_v, dtype=np.float64), np.asarray(out_f, dtype=np.int64)


def box(vertices) -> dict:
    """The oriented box that encloses the points: center, half extents, and a [w, x, y, z] rotation.
    Points with no volume (a plate, a line) get a box of zero thickness in their own plane or along their line."""
    import trimesh

    v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    if not len(v):
        raise ValueError("a box needs at least one point")
    if _solid(v):
        to_origin, extents = trimesh.bounds.oriented_bounds(trimesh.convex.convex_hull(v))
        world = np.linalg.inv(to_origin)          # box frame -> mesh frame
        rotation, center, half = world[:3, :3], world[:3, 3], np.asarray(extents) / 2
    else:
        rotation, center, half = _flat_box(v)
    if np.linalg.det(rotation) < 0:           # keep a proper rotation: flip one axis
        rotation[:, 2] *= -1
    quat = trimesh.transformations.quaternion_from_matrix(np.vstack([np.hstack([rotation, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    return dict(center=np.asarray(center).tolist(), half_extents=np.asarray(half).tolist(), orientation=[float(q) for q in quat])


def _flat_box(v):
    """(rotation, center, half extents) for points with no volume: the smallest rectangle in their plane, or their
    segment. An axis-aligned plate (a floor) keeps the axis-aligned box, so its axes stay x, y and z."""
    import trimesh

    lo, hi = v.min(axis=0), v.max(axis=0)
    mean = v.mean(axis=0)
    _, s, axes = np.linalg.svd(v - mean)      # rows: principal directions; for a plate the last is its normal
    if len(s) > 1 and s[1] > 1e-9 * max(s[0], 1e-12):
        to_rect, _ = trimesh.bounds.oriented_bounds_2D((v - mean) @ axes[:2].T)
        axes = np.vstack([to_rect[:2, :2] @ axes[:2], axes[2]])
    local = (v - mean) @ axes.T
    a, b = local.min(axis=0), local.max(axis=0)
    half = (b - a) / 2
    if np.allclose(np.sort(half), np.sort((hi - lo) / 2), rtol=1e-7, atol=1e-12):
        return np.eye(3), (lo + hi) / 2, (hi - lo) / 2
    return axes.T.copy(), mean + ((a + b) / 2) @ axes, half


def _solid(points) -> bool:
    """Whether the points span three dimensions (a hull of flat points does not exist)."""
    if len(points) < 4:
        return False
    s = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    return bool(s[-1] > 1e-9 * max(s[0], 1e-12))


def box_volume(shape: dict) -> float:
    return float(np.prod(np.asarray(shape["half_extents"]) * 2))


def convex(vertices, faces=None):
    """One convex hull of the points, as (vertices, triangles)."""
    import trimesh

    v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    hull = trimesh.convex.convex_hull(v)
    return np.asarray(hull.vertices, dtype=np.float64), np.asarray(hull.faces, dtype=np.int64)


def hull_volume(vertices, faces) -> float:
    v, f = _as_arrays(vertices, faces)
    return float(abs(_trimesh(v, f).volume))


def coacd(vertices, faces, *, max_hulls: int = 16, threshold: float = 0.05, seed: int = 0):
    """Approximate convex decomposition: at most ``max_hulls`` convex pieces, each (vertices, triangles)."""
    import coacd as _coacd

    v, f = _as_arrays(vertices, faces)
    if not 1 <= int(max_hulls) <= 256:
        raise ValueError("max_hulls must be between 1 and 256")
    if not 0.01 <= float(threshold) <= 1:
        raise ValueError("threshold must be between 0.01 and 1")
    _coacd.set_log_level("error")
    parts = _coacd.run_coacd(_coacd.Mesh(v, f), threshold=float(threshold), max_convex_hull=int(max_hulls), seed=int(seed))
    return [(np.asarray(pv, dtype=np.float64), np.asarray(pf, dtype=np.int64)) for pv, pf in parts]


def collision_shapes(vertices, faces, method: str, *, max_hulls: int = 16, threshold: float = 0.05) -> list[dict]:
    """The collision shapes for one mesh: ``[{"kind": "box", ...}]`` or ``[{"kind": "hull", "points", "faces"}, ...]``.

    ``auto`` uses the box unless the box is more than twice the volume of the convex hull (an L, a frame, a
    bracket), and then a CoACD decomposition. A flat mesh (a plate, a floor) has no hull, so it always gets the box,
    of zero thickness in the mesh's plane."""
    if method not in ("box", "convex", "coacd", "auto"):
        raise ValueError("method must be box, convex, coacd or auto")
    vertices, faces = _as_arrays(vertices, faces)
    if not len(vertices):
        raise ValueError("the mesh has no points")
    if not _solid(vertices):
        method = "box"
    if method == "auto":
        shape = box(vertices)
        hv, hf = convex(vertices)
        hull = hull_volume(hv, hf) if len(hf) else 0.0
        if hull > 0 and box_volume(shape) > 2 * hull:
            return collision_shapes(vertices, faces, "coacd", max_hulls=max_hulls, threshold=threshold)
        return [dict(kind="box", **shape)]
    if method == "box":
        return [dict(kind="box", **box(vertices))]
    if method == "convex":
        hv, hf = convex(vertices)
        return [dict(kind="hull", points=hv, faces=hf)]
    if method == "coacd":
        return [dict(kind="hull", points=pv, faces=pf)
                for pv, pf in coacd(vertices, faces, max_hulls=max_hulls, threshold=threshold)]
    raise ValueError("method must be box, convex, coacd or auto")
