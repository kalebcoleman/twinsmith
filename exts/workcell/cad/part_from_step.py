"""One rigid part from a vendor's STEP, read with FreeCAD (``freecadcmd``) and written as one mesh per material.

Used by ``scripts/workcell_stack_assets.py`` for the two parts that sit between the UR5e's tool flange and the RG2:
OnRobot's Quick Changer robot side and Bota Systems' Rokubi force/torque sensor. The STEP files are the vendors';
their redistribution terms are not known, so each is read from a local path, checked by SHA-256 by the caller, and
nothing of it is kept in this repository. A STEP is parsed as data: nothing in it is executed.

Environment: STEP_SRC (the STEP file), PART_OUT (folder), PART_SPEC (JSON):

* ``frame``: three rows; output point = rows . (CAD point - ``origin_mm``), millimetres. Must be a rotation.
* ``origin_mm``: the CAD point that becomes the output origin.
* ``drop``: part labels left out; ``materials``: ``{label prefix: material}``; ``default_material``.
* ``collider``: labels whose points bound the collision hull, with an optional ``clip_x_mm`` range in the output frame.
* ``caps``: triangles per material after decimation.

Writes ``<material>.stl``, ``collider_points.json`` and ``part.json`` (labels kept and left out, bounds, volumes,
centre of volume and triangle counts).
"""
import json
import os

import FreeCAD
import Import
import Mesh
import MeshPart
from FreeCAD import Matrix, Vector

SRC, OUT, SPEC = os.environ["STEP_SRC"], os.environ["PART_OUT"], json.loads(os.environ["PART_SPEC"])
LINEAR, ANGULAR, COARSE, TRIANGLE_CAP = 0.1, 0.4, (0.4, 0.8), 6000

rows, origin = SPEC["frame"], SPEC["origin_mm"]
moved = [-sum(rows[i][j] * origin[j] for j in range(3)) for i in range(3)]
TO_PART = Matrix(rows[0][0], rows[0][1], rows[0][2], moved[0], rows[1][0], rows[1][1], rows[1][2], moved[1],
                 rows[2][0], rows[2][1], rows[2][2], moved[2], 0, 0, 0, 1)
if abs(TO_PART.determinant() - 1.0) > 1e-9:
    raise SystemExit("PART ERROR: frame must be a rotation")

doc = FreeCAD.newDocument("part")
Import.insert(SRC, doc.Name)
doc.recompute()
solids, seen = [], set()
for obj in doc.Objects:
    if not hasattr(obj, "Shape") or obj.isDerivedFrom("App::Part") or obj.Shape.isNull() or len(obj.Shape.Solids) != 1:
        continue
    shape = obj.Shape.copy()
    shape.Placement = obj.getGlobalPlacement()
    key = (round(shape.Volume), round(shape.CenterOfMass.x, 1), round(shape.CenterOfMass.y, 1), round(shape.CenterOfMass.z, 1))
    if key in seen:
        continue
    seen.add(key)
    solids.append({"label": obj.Label, "shape": shape.transformGeometry(TO_PART)})
if not solids:
    raise SystemExit("PART ERROR: the STEP holds no solid")


def material_of(label):
    for prefix, material in SPEC.get("materials", {}).items():
        if label.startswith(prefix):
            return material
    return SPEC["default_material"]


def mesh_of(shape):
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=LINEAR, AngularDeflection=ANGULAR, Relative=False)
    if mesh.CountFacets > TRIANGLE_CAP:
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=COARSE[0], AngularDeflection=COARSE[1], Relative=False)
    return mesh


dropped = [prefix for prefix in SPEC.get("drop", [])]
meshes, kept, left_out, hull, volume, moment = {}, [], [], [], 0.0, Vector(0, 0, 0)
low, high = [1e9] * 3, [-1e9] * 3
for solid in solids:
    label, shape = solid["label"], solid["shape"]
    if any(label.startswith(prefix) for prefix in dropped) or shape.Volume < SPEC.get("min_volume_mm3", 0.0):
        left_out.append(label)
        continue
    mesh = mesh_of(shape)
    meshes.setdefault(material_of(label), Mesh.Mesh()).addMesh(mesh)
    kept.append(label)
    volume += shape.Volume
    moment += shape.Solids[0].CenterOfMass * shape.Volume
    points = [point for point in mesh.Topology[0]]
    for point in points:
        for axis in range(3):
            low[axis], high[axis] = min(low[axis], point[axis]), max(high[axis], point[axis])
    if any(label.startswith(prefix) for prefix in SPEC["collider"]["labels"]):
        clip = SPEC["collider"].get("clip_x_mm")
        hull += [[point.x, point.y, point.z] for point in points if clip is None or clip[0] <= point.x <= clip[1]]
if not hull:
    raise SystemExit("PART ERROR: no point is left for the collision hull")

report = {"kept": sorted(kept), "left_out": sorted(left_out), "triangles_before_decimation": sum(m.CountFacets for m in meshes.values()),
          "bounds_mm": [[round(v, 3) for v in low], [round(v, 3) for v in high]], "volume_cm3": round(volume / 1000.0, 3),
          "centre_of_volume_mm": [round(v, 3) for v in (moment.x / volume, moment.y / volume, moment.z / volume)], "triangles": {}}
for material, mesh in meshes.items():
    cap = SPEC.get("caps", {}).get(material, 4000)
    if mesh.CountFacets > cap:                                   # quadric edge collapse, at most 0.05 mm off the surface
        mesh.decimate(0.05, 1.0 - cap / mesh.CountFacets)
    report["triangles"][material] = mesh.CountFacets
    mesh.write(os.path.join(OUT, f"{material}.stl"))
with open(os.path.join(OUT, "collider_points.json"), "w") as handle:
    json.dump(hull[:: max(1, len(hull) // 20000)], handle)
with open(os.path.join(OUT, "part.json"), "w") as handle:
    json.dump(report, handle, indent=1)
print("PART " + json.dumps(report))
