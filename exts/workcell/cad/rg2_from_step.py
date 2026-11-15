"""OnRobot's official RG2 STEP, read with FreeCAD (``freecadcmd``) and written as one mesh per link and material.

The STEP ("RG2 Tier 2 - 101631", https://onrobot.com/storage/303_rg2_tool.step, linked from
https://onrobot.com/en/products/rg2-finger-gripper) is OnRobot's; its redistribution terms are not known, so it is read
from a local path, checked by SHA-256 by the caller, and nothing of it is kept in this repository. It is parsed as
data: nothing in it is executed.

CAD axes: +Y the approach axis (fingertips at +Y), +X the finger-opening axis, -Z out of the front face (the face with
the logo, the M3 pair 11 mm above the lower end of the body, and the Quick Changer's release button). Output frame,
millimetres ("gripper frame"): x = CAD x, y = -CAD z (out of the front face), z = CAD y measured from the top of the
Quick Changer tool side. The work cell's RG2 body frame (``onrobot_rg2_base_link``) is that frame turned a quarter turn
about z: body +Y = gripper +x, body +X = gripper -y (out of the back face).

Links, by the CAD's sub-assemblies: the base (Quick Changer tool side 101141, mounting bracket 101379, body shells and
what shows of 101848), the two outer arms (100876, 100883) with the cover plates, the two inner links (100892, 101779)
and the two finger carriers with their fingertips and rubber pads (101803, 100905). Parts hidden inside the body
shells, springs and connector pins are left out. The four pin axes of each side are read from the CAD's cylinders; the
links are then turned from the CAD's pose to the joint-zero pose of the work cell's RG2 (outer arm 46.04 degrees from
the approach axis), so joint values mean what they meant before.

Environment: STEP_SRC (the STEP file), RG2_OUT (folder). Writes ``<link>__<material>.stl``, ``fingertip_<side>.stl``
and ``rg2.json`` (pivots, fingertip pads, dimensions, triangle counts).
"""
import json
import math
import os

import FreeCAD
import Import
import Mesh
import MeshPart
from FreeCAD import Matrix, Vector

SRC, OUT = os.environ["STEP_SRC"], os.environ["RG2_OUT"]
ZERO_POSE_DEG = math.degrees(math.atan2(39.592, 38.177))     # the outer arm at joint zero, from the approach axis
LINEAR, ANGULAR, COARSE = 0.12, 0.45, (0.4, 0.8)             # tessellation: mm, radians; coarse for small busy parts
TRIANGLE_CAP = 4000                                          # per solid before the coarse setting is used
CAPS = {("base", "aluminium"): 40000, ("base", "steel"): 4000, ("outer", "aluminium"): 6000, ("outer", "black"): 6000,
        ("inner", "aluminium"): 5000, ("inner", "black"): 4000, ("finger", "aluminium"): 3200, "default": 2500}   # triangles

SIDES = {"right": {"outer": "100876", "inner": "100892", "finger": "101803", "sign": 1.0},
         "left": {"outer": "100883", "inner": "101779", "finger": "100905", "sign": -1.0}}
BODY_SHELLS = ("101662", "101804")
BLACK = ("100918", "100920", "101839", "101840",             # cover plates over the linkage
         "101542", "101676", "101543", "101682", "101544", "101673", "101547", "101778", "101550", "101777")
RUBBER_PREFIX = "100084"                                     # the fingertips' 1.5 mm pads


def chain(obj):
    names = []
    while obj is not None:
        names.append(obj.Label)
        parents = [p for p in obj.InList if p.isDerivedFrom("App::Part") or p.isDerivedFrom("App::LinkGroup")]
        obj = parents[0] if parents else None
    return names[::-1]


def cylinders(shape):
    """(x, y, area) of every cylinder along Z with a pin-sized radius."""
    found = []
    for face in shape.Faces:
        surface = face.Surface
        if surface.__class__.__name__ == "Cylinder" and abs(surface.Axis.z) > 0.999 and 0.8 <= surface.Radius <= 5.0:
            found.append((surface.Center.x, surface.Center.y, face.Area))
    return found


doc = FreeCAD.newDocument("rg2")
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
    path = chain(obj)
    solids.append({"label": obj.Label, "path": path[1:-1], "shape": shape})

shells = [s for s in solids if s["label"] in BODY_SHELLS]
if len(shells) != 2:
    raise SystemExit("RG2 ERROR: the two body shells were not found; this is not the expected RG2 STEP")
inside_low = Vector(min(s["shape"].BoundBox.XMin for s in shells) + 0.5, min(s["shape"].BoundBox.YMin for s in shells) + 0.5,
                    min(s["shape"].BoundBox.ZMin for s in shells) + 0.5)
inside_high = Vector(max(s["shape"].BoundBox.XMax for s in shells) - 0.5, max(s["shape"].BoundBox.YMax for s in shells) - 0.5,
                     max(s["shape"].BoundBox.ZMax for s in shells) - 0.5)


def link_of(solid):
    """The link a solid belongs to, or None for what is left out."""
    path, label, box = solid["path"], solid["label"], solid["shape"].BoundBox
    for side, parts in SIDES.items():
        for kind in ("outer", "inner", "finger"):
            if parts[kind] in path:
                return f"{side}_{kind}"
    if "sodemann" in label or "101310" in path or solid["shape"].Volume < 15.0:
        return None                                              # springs, connector pins, washers
    hidden = (label not in BODY_SHELLS and box.XMin > inside_low.x and box.XMax < inside_high.x and box.YMin > inside_low.y
              and box.YMax < inside_high.y and box.ZMin > inside_low.z and box.ZMax < inside_high.z)
    if hidden:
        return None
    if path and path[-1] == "101848" and box.YMin > 100.0 and abs(box.Center.x) > 14.0:
        return ("right" if box.Center.x > 0 else "left") + "_outer"      # cover plates and their pins ride on the arm
    return "base"


def material_of(solid, link):
    label, shape = solid["label"], solid["shape"]
    if label.startswith(RUBBER_PREFIX):
        return "rubber"
    if label in BLACK:
        return "black"
    box = shape.BoundBox
    if shape.Volume < 400.0 and max(box.XLength, box.YLength) < 9.0:
        return "steel"                                           # pins and screws
    return "aluminium"


for solid in solids:
    solid["link"] = link_of(solid)
    solid["material"] = material_of(solid, solid["link"]) if solid["link"] else None


def pivot(first, second, near):
    """The pin axis two groups share, near a seed: the area-weighted mean of their cylinders' centres."""
    total, x, y = 0.0, 0.0, 0.0
    for group in (first, second):
        for solid in solids:
            if solid["link"] != group:
                continue
            for cx, cy, area in cylinders(solid["shape"]):
                if math.hypot(cx - near[0], cy - near[1]) < 1.0:
                    total, x, y = total + area, x + cx * area, y + cy * area
    if total <= 0.0:
        raise SystemExit(f"RG2 ERROR: no pin between {first} and {second} near {near}")
    return x / total, y / total


SEEDS = {"A": ("base", "outer", (17.0, 107.5)), "B": ("base", "inner", (7.5, 124.0)),
         "C": ("outer", "finger", (47.0, 153.5)), "D": ("inner", "finger", (37.5, 170.0))}
pivots = {}
for side, parts in SIDES.items():
    for name, (first, second, seed) in SEEDS.items():
        groups = [group if group == "base" else f"{side}_{group}" for group in (first, second)]
        pivots[f"{side}_{name}"] = pivot(groups[0], groups[1], (parts["sign"] * seed[0], seed[1]))

TOP = -min(solid["shape"].BoundBox.YMin for solid in solids if solid["link"] == "base")      # top of the Quick Changer tool side
TO_GRIPPER = Matrix(1, 0, 0, 0,
                    0, 0, -1, 0,
                    0, 1, 0, TOP,
                    0, 0, 0, 1)


def about_y(point, degrees):
    """Rotation about the gripper frame's y through ``point`` that opens a +x arm by ``degrees``."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    rotation = Matrix(c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1)
    there, back = Matrix(), Matrix()
    there.move(Vector(-point[0], 0, -point[1]))
    back.move(Vector(point[0], 0, point[1]))
    return back.multiply(rotation).multiply(there)


def gripper_xz(xy):
    return xy[0], xy[1] + TOP


report = {"source": os.path.basename(SRC), "top_of_tool_side_above_cad_origin_mm": round(TOP, 3), "zero_pose_deg": round(ZERO_POSE_DEG, 4),
          "pivots_mm": {}, "links": {}, "fingertips": {}}
moves = {"base": Matrix()}
for side, parts in SIDES.items():
    a, b, c, d = (gripper_xz(pivots[f"{side}_{name}"]) for name in "ABCD")
    cad_deg = math.degrees(math.atan2(parts["sign"] * (c[0] - a[0]), c[1] - a[1]))
    turn = parts["sign"] * (ZERO_POSE_DEG - cad_deg)
    arm = math.hypot(c[0] - a[0], c[1] - a[1])
    zero = math.radians(ZERO_POSE_DEG)
    c_zero = (a[0] + parts["sign"] * arm * math.sin(zero), a[1] + arm * math.cos(zero))
    shift = Matrix()
    shift.move(Vector(c_zero[0] - c[0], 0, c_zero[1] - c[1]))
    moves[f"{side}_outer"], moves[f"{side}_inner"], moves[f"{side}_finger"] = about_y(a, turn), about_y(b, turn), shift
    d_zero = (d[0] + c_zero[0] - c[0], d[1] + c_zero[1] - c[1])
    report["pivots_mm"][side] = {"A": [round(v, 4) for v in a], "B": [round(v, 4) for v in b], "C": [round(v, 4) for v in c_zero],
                                 "D": [round(v, 4) for v in d_zero], "C_in_cad_pose": [round(v, 4) for v in c],
                                 "cad_pose_deg": round(cad_deg, 4), "arm_AC": round(arm, 4),
                                 "arm_BD": round(math.hypot(d[0] - b[0], d[1] - b[1]), 4),
                                 "AB": [round(b[0] - a[0], 4), round(b[1] - a[1], 4)], "CD": [round(d[0] - c[0], 4), round(d[1] - c[1], 4)]}


def mesh_of(shape):
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=LINEAR, AngularDeflection=ANGULAR, Relative=False)
    if mesh.CountFacets > TRIANGLE_CAP:
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=COARSE[0], AngularDeflection=COARSE[1], Relative=False)
    return mesh


meshes, tips = {}, {}
for solid in solids:
    link = solid["link"]
    if not link:
        continue
    shape = solid["shape"].transformGeometry(moves[link].multiply(TO_GRIPPER) if link != "base" else TO_GRIPPER)
    mesh = mesh_of(shape)
    key = (link, solid["material"])
    if key not in meshes:
        meshes[key] = Mesh.Mesh()
    meshes[key].addMesh(mesh)
    entry = report["links"].setdefault(link, {"solids": 0, "triangles": 0, "volume_cm3": 0.0, "parts": []})
    entry["solids"] += 1
    entry["triangles"] += mesh.CountFacets
    entry["volume_cm3"] = round(entry["volume_cm3"] + solid["shape"].Volume / 1000.0, 3)
    entry["parts"].append(solid["label"])
    if link.endswith("_finger") and (solid["material"] == "rubber" or solid["shape"].Volume > 3000.0 and solid["shape"].BoundBox.YMin > 170.0):
        side = link.split("_")[0]                                # the fingertip block and its rubber pad
        tips.setdefault(side, Mesh.Mesh()).addMesh(mesh)
        box = shape.BoundBox
        if solid["material"] == "rubber":
            face_x = box.XMin if side == "right" else box.XMax   # the gripping face looks at the other finger
            report["fingertips"].setdefault(side, {})["pad"] = {
                "face_x_mm": round(face_x, 3), "thickness_mm": round(box.XLength, 3), "length_along_z_mm": round(box.ZLength, 3),
                "width_along_y_mm": round(box.YLength, 3), "z_range_mm": [round(box.ZMin, 3), round(box.ZMax, 3)],
                "y_range_mm": [round(box.YMin, 3), round(box.YMax, 3)]}
        else:
            report["fingertips"].setdefault(side, {})["block_size_mm"] = [round(box.XLength, 3), round(box.YLength, 3), round(box.ZLength, 3)]

report["triangles_before_decimation"] = sum(mesh.CountFacets for mesh in meshes.values())
for (link, material), mesh in meshes.items():
    cap = CAPS.get((link.split("_")[-1], material), CAPS["default"])
    if mesh.CountFacets > cap:                                   # quadric edge collapse, at most 0.05 mm off the surface
        mesh.decimate(0.05, 1.0 - cap / mesh.CountFacets)
    report["links"][link].setdefault("triangles_by_material", {})[material] = mesh.CountFacets
    mesh.write(os.path.join(OUT, f"{link}__{material}.stl"))
for entry in report["links"].values():
    entry["triangles"] = sum(entry["triangles_by_material"].values())
for side, mesh in tips.items():
    mesh.write(os.path.join(OUT, f"fingertip_{side}.stl"))
everything = [solid["shape"].BoundBox for solid in solids if solid["link"]]
base = [solid["shape"].BoundBox for solid in solids if solid["link"] == "base"]
report["cad_pose"] = {"overall_size_mm": [round(max(b.XMax for b in everything) - min(b.XMin for b in everything), 2),
                                           round(max(b.YMax for b in everything) - min(b.YMin for b in everything), 2),
                                           round(max(b.ZMax for b in everything) - min(b.ZMin for b in everything), 2)],
                      "fingertip_end_z_mm": round(max(b.YMax for b in everything) + TOP, 2),
                      "body_lower_end_z_mm": round(max(s["shape"].BoundBox.YMax for s in shells) + TOP, 2),
                      "body_shell_x_mm": [round(min(s["shape"].BoundBox.XMin for s in shells), 2), round(max(s["shape"].BoundBox.XMax for s in shells), 2)],
                      "body_shell_cad_z_mm": [round(min(s["shape"].BoundBox.ZMin for s in shells), 2), round(max(s["shape"].BoundBox.ZMax for s in shells), 2)],
                      "base_x_mm": [round(min(b.XMin for b in base), 2), round(max(b.XMax for b in base), 2)]}
report["left_out"] = sorted(solid["label"] for solid in solids if not solid["link"])
report["triangles"] = sum(entry["triangles"] for entry in report["links"].values())
with open(os.path.join(OUT, "rg2.json"), "w") as handle:
    json.dump(report, handle, indent=1)
print("RG2 " + json.dumps({"triangles": report["triangles"], "links": {k: (v["solids"], v["triangles"]) for k, v in report["links"].items()},
                           "pivots": report["pivots_mm"]["right"], "left_out": len(report["left_out"])}))
