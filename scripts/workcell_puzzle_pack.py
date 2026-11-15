#!/usr/bin/env python3
"""Build the ``puzzles/v1`` task pack: the lab's 3D-printed assembly puzzles (two base plates and their pieces).

The geometry is from a checkout of github.com/inbarajaldrin/isaac-sim-mcp (MIT) at the commit pinned below:
``exts/ur5e-dt/assets/objects/fmb1`` (the red base) and ``fmb3`` (the green base). Those are Aldrin's USD conversions,
rescaled for the lab's prints, of object CAD from FMB, the Functional Manipulation Benchmark (Luo et al., UC Berkeley,
CC BY 4.0; see THIRD-PARTY-NOTICES.md). For each piece the script reads the
render mesh of ``<piece>.usda`` and the box colliders of ``<piece>_physics.usda`` and writes one flat file,
``<piece>/<piece>.usda``, in metres and Z up:

* ``Body``: the mesh, to look at. It is not a collider.
* ``Collision/box_NNN``: the piece as analytic boxes (purpose ``guide``). Every piece is a union of axis-aligned
  boxes, so the boxes are the piece, not an approximation: the script refuses a piece whose boxes do not fill its
  mesh's volume to 0.1 %.
* ``Looks/Surface``: one colour, from the lab photo of 2026-11-14 (the upstream fmb3 files have none).
* ``puzzle:volumeM3`` on the root: the solid volume, from which a task computes the mass.

``pack.yaml`` also records ``assemblies``: per base, the pose of each of its pieces in the base's frame when the
puzzle is assembled, in the order they go in. These are upstream's ``fmb_assembly1.json`` and ``fmb_assembly3.json``
with the angles rounded to right angles and each piece raised by however much its boxes sat inside the base's
(0.05 mm for the bars).

The pieces carry fiducial tags in the lab; none is modelled. Nothing upstream is run or changed, and a pack that
exists is left alone. Run from ``mcp/``:

    uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_puzzle_pack.py <checkout>
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "mcp"))

from twinsmith import asset_packs, pack_manifest  # noqa: E402

COMMIT = "27f610056017d7bffc2ff56fac6d219702b781a0"
ASSETS = "exts/ur5e-dt/assets"
OBJECTS = f"{ASSETS}/objects"
ASSEMBLIES = ("fmb_assembly1.json", "fmb_assembly3.json")     # one per base
PACK = ("puzzles", "v1")
# Piece -> (upstream folder, linear RGB). The colours are read off the lab photo; where upstream authors one (fmb1)
# it agrees, except u_brown, which is maroon on the table and near black upstream.
PIECES = {
    "base1": ("fmb1", (0.84, 0.05, 0.02)),
    "line_green": ("fmb1", (0.015, 0.09, 0.025)),
    "inverted_u_yellow": ("fmb1", (0.90, 0.48, 0.0)),
    "u_orange": ("fmb1", (0.93, 0.16, 0.01)),
    "u_brown": ("fmb1", (0.18, 0.02, 0.02)),
    "base3": ("fmb3", (0.02, 0.22, 0.07)),
    "line_brown": ("fmb3", (0.18, 0.02, 0.02)),
    "line_red": ("fmb3", (0.60, 0.02, 0.02)),
    "fork_orange": ("fmb3", (0.93, 0.16, 0.01)),
    "fork_yellow": ("fmb3", (0.90, 0.48, 0.0)),
}


def read_piece(objects: Path, folder: str, name: str):
    """(points in metres, face counts, face indices, boxes [(centre, half extents)]) in the piece's own frame."""
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(objects / folder / f"{name}_physics.usda"))
    cache = UsdGeom.XformCache()
    meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    if len(meshes) != 1:
        raise SystemExit(f"{name}: expected one mesh, found {len(meshes)}")
    mesh = UsdGeom.Mesh(meshes[0])
    to_world = cache.GetLocalToWorldTransform(meshes[0])
    points = np.array([to_world.Transform(point) for point in mesh.GetPointsAttr().Get()], dtype=float)
    counts, indices = list(mesh.GetFaceVertexCountsAttr().Get()), list(mesh.GetFaceVertexIndicesAttr().Get())
    boxes = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Cube) and prim.HasAPI(UsdPhysics.CollisionAPI):
            matrix = cache.GetLocalToWorldTransform(prim)
            half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
            rows = np.array([[matrix[i][j] for j in range(3)] for i in range(3)]) * half
            if np.abs(rows - np.diag(np.diag(rows))).max() > 1e-9:
                raise SystemExit(f"{name}: collider {prim.GetName()} is not axis aligned")
            boxes.append((np.array(matrix.ExtractTranslation()), np.abs(np.diag(rows))))
    if not boxes:
        raise SystemExit(f"{name}: upstream has no box colliders")
    return points, counts, indices, boxes


def mesh_volume(points, counts, indices):
    """Volume of a closed mesh: signed tetrahedra from the origin over a fan of every face."""
    total, at = 0.0, 0
    for count in counts:
        face = points[indices[at:at + count]]
        total += sum(np.dot(face[0], np.cross(face[i], face[i + 1])) for i in range(1, count - 1))
        at += count
    return abs(total) / 6


def union_volume(boxes):
    """Volume the boxes cover together, counted once where they overlap."""
    low, high = (np.array([centre + sign * half for centre, half in boxes]) for sign in (-1, 1))
    cuts = [np.unique(np.concatenate([low[:, axis], high[:, axis]])) for axis in range(3)]
    mids = np.meshgrid(*[(c[:-1] + c[1:]) / 2 for c in cuts], indexing="ij")
    inside = np.zeros(mids[0].shape, dtype=bool)
    for lo, hi in zip(low, high):
        inside |= np.logical_and.reduce([(mids[a] > lo[a]) & (mids[a] < hi[a]) for a in range(3)])
    cells = np.einsum("i,j,k->ijk", *[np.diff(c) for c in cuts])
    return float(cells[inside].sum())


def assembly(path: Path, boxes):
    """One base's assembled pieces, in the order they go in: [{"piece", "position_m", "rpy_deg"}] in the base's frame."""
    def turn(rpy):
        (cr, sr), (cp, sp), (cy, sy) = [(round(math.cos(math.radians(a))), round(math.sin(math.radians(a)))) for a in rpy]
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]], dtype=float)

    parts = sorted(json.loads(path.read_text())["components"], key=lambda part: part["assembly_order"])
    base, rows = boxes[parts[0]["name"]], []
    for part in parts[1:]:
        rpy = [90.0 * round(math.degrees(part["rotation"]["rpy"][axis]) / 90.0) for axis in "xyz"]
        at, lift = np.array([part["position"][axis] for axis in "xyz"]), 0.0
        for centre, half in boxes[part["name"]]:
            centre, half = turn(rpy) @ centre + at, np.abs(turn(rpy) @ half)
            for other, size in base:
                sunk = (other[2] + size[2]) - (centre[2] - half[2])
                if (np.abs(centre - other)[:2] < (half + size)[:2] - 1e-6).all() and 0 < sunk < 0.001:
                    lift = max(lift, sunk)
        rows.append({"piece": part["name"], "position_m": [round(float(v), 6) for v in at + [0.0, 0.0, lift]], "rpy_deg": rpy})
    return parts[0]["name"], rows


def write_piece(path: Path, name, colour, points, counts, indices, boxes, volume):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
    stage.SetDefaultPrim(root)
    root.CreateAttribute("puzzle:volumeM3", Sdf.ValueTypeNames.Double, custom=True).Set(volume)
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Body")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
    UsdGeom.Scope.Define(stage, "/Asset/Collision")
    for index, (centre, half) in enumerate(boxes):
        box = UsdGeom.Cube.Define(stage, f"/Asset/Collision/box_{index:03d}")
        box.CreateSizeAttr(2.0)
        box.CreatePurposeAttr(UsdGeom.Tokens.guide)
        box.AddTranslateOp().Set(Gf.Vec3d(*centre))
        box.AddScaleOp().Set(Gf.Vec3f(*half))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    material = UsdShade.Material.Define(stage, "/Asset/Looks/Surface")
    shader = UsdShade.Shader.Define(stage, "/Asset/Looks/Surface/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    stage.GetRootLayer().Save()


def main(argv) -> int:
    if len(argv) != 2:
        raise SystemExit(__doc__)
    checkout = Path(argv[1]).resolve()
    head = subprocess.run(["git", "-C", str(checkout), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    if head != COMMIT:
        raise SystemExit(f"{checkout} is at {head or 'no commit'}, not the pinned {COMMIT}")
    root = pack_manifest.resolve_asset_root().path
    if (root / PACK[0] / PACK[1] / "pack.yaml").is_file():
        print(json.dumps({"/".join(PACK): "exists, left alone"}))
        return 0
    report, files, records, boxes_of = {}, {}, {}, {}
    with tempfile.TemporaryDirectory() as scratch:
        for name, (folder, colour) in PIECES.items():
            points, counts, indices, boxes = read_piece(checkout / OBJECTS, folder, name)
            boxes_of[name] = boxes
            volume, covered = mesh_volume(points, counts, indices), union_volume(boxes)
            if abs(covered - volume) > 0.001 * volume:
                raise SystemExit(f"{name}: the boxes cover {covered * 1e6:.2f} cm3 of a {volume * 1e6:.2f} cm3 mesh")
            rel = f"{name}/{name}.usda"
            target = Path(scratch) / rel
            target.parent.mkdir()
            write_piece(target, name, colour, points, counts, indices, boxes, volume)
            size = (points.max(0) - points.min(0)) * 1000
            files[rel] = target
            records[rel] = dict(
                asset_id=name, redistributable=True,
                source=f"github.com/inbarajaldrin/isaac-sim-mcp@{COMMIT} {OBJECTS}/{folder}/{name}.usda (mesh) and "
                       f"{name}_physics.usda (box colliders), flattened by scripts/workcell_puzzle_pack.py",
                license="CC BY 4.0: FMB (Functional Manipulation Benchmark, Luo et al., UC Berkeley) object CAD, converted to "
                        "USD and rescaled in isaac-sim-mcp (MIT, Copyright (c) 2025 omni-mcp)",
                fidelity="Upstream CAD, not measured against the printed part; colour from the lab photo of 2026-11-14; "
                         "no fiducial tags modelled",
                size_mm=[round(float(v), 2) for v in size], volume_cm3=round(volume * 1e6, 3),
                collision=dict(method="box", boxes=len(boxes), source_triangles=int(sum(c - 2 for c in counts))))
            report[name] = dict(size_mm=records[rel]["size_mm"], volume_cm3=records[rel]["volume_cm3"], boxes=len(boxes))
        written = asset_packs.write_version(
            root, *PACK, files, records, defaults={}, notes=[],
            extra=dict(qualification="task pack: the lab's two 3D-printed assembly puzzles (red base fmb1, green base fmb3)",
                       derived_from=f"github.com/inbarajaldrin/isaac-sim-mcp@{COMMIT}",
                       assemblies=dict(assembly(checkout / ASSETS / name, boxes_of) for name in ASSEMBLIES)))
    print(json.dumps({"pack": written["pack"], "pieces": report}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
