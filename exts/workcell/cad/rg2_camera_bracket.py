"""Wrist camera bracket on the OnRobot RG2, as a FreeCAD model (run with ``freecadcmd``; millimetres).

A parametric copy of the 3D-printed bracket on the lab's UR5e, read from one photo (see ``README.md`` beside this file).
It is NOT a plate between flange faces: it is a two-piece clamp collar around the RG2 body's 54 x 36 mm waist, below the
gripper's own mounting bracket and above the finger pivots, so the tool stack and the tool kinematics are unchanged and
the camera turns with the gripper. Two printed pieces:

* ``clamp_front``: the half on the RG2's front face (the face with the two M3 holes, where the lab's AprilTag is), with
  captive M4 nut pockets and a bar that runs down the front face;
* ``camera_arm``: the half on the back face, a triangulated truss arm with an open window, and the flat plate the D415
  screws to through its two rear M3 holes. It prints flat on the plate face without supports.

Four M4 cross bolts (two each side of the body, along the body's thickness) pull the halves together across a split.

CAD frame: origin on the tool axis at the top of the RG2's Quick Changer tool side, +Z along the approach axis toward
the fingertips, +X the finger-opening axis, +Y out of the front face. The RG2 datasheet's mounting plane is at Z = 16
(its 132 mm body length ends at Z = 148). The work cell pack's RG2 body frame (``onrobot_rg2_base_link``) is this frame
turned a quarter turn about Z (body +Y = CAD +X, body +X = CAD -Y); ``scripts/workcell_bracket_assets.py`` converts.

Sourced dimensions (vendor documents, fetched 2026-11-14):

* RG2 body 54 mm wide and 36 mm thick at the waist, 65 mm wide lower down, 132 mm from the mounting plane to the lower
  end of the body, mounting bracket 75 mm wide: OnRobot RG2 datasheet v1.8, section 1.3,
  https://onrobot.com/storage/datasheets/rg2/datasheet_rg2_v1.8_en.pdf
* D415 body 99 x 23 x 20 mm, two M3 mounting points on the back 45 mm apart with a maximum insertion depth of 3 mm:
  Intel RealSense D400 Series Datasheet rev 017 (September 2023), table 3-51 and figure 10-8,
  https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf

Everything else below is ESTIMATED FROM THE PHOTO (or a design choice where the photo shows nothing) and must be
measured; ``README.md`` lists each value with how it was scaled and its uncertainty.

Environment (all optional except BRACKET_OUT): BRACKET_OUT (folder), CAM_X_MM, CAM_Y_MM, CAM_Z_MM (the D415 colour
optical origin in the RG2 frame), BACK_MM (camera back face along the optical axis, negative), BODY_CX_MM (camera body
centre along optical x), all in this CAD frame. The camera looks along +Z with its long axis along X, turned 180
degrees about the optical axis (optical +x = -X, optical +y = -Y); no other orientation is modelled.
Writes clamp_front.step/.stl, camera_arm.step/.stl, wrist_bracket.step (both pieces as mounted) and bracket.json.
"""
import json
import math
import os

import FreeCAD
import Mesh
import Part
from FreeCAD import Vector

OUT = os.environ["BRACKET_OUT"]


def env(name, default):
    return float(os.environ.get(name, default))


P = {
    # --- sourced -------------------------------------------------------------------------------------------------
    "rg2_waist_width": 54.0,        # RG2 datasheet v1.8, 1.3
    "rg2_body_thickness": 36.0,     # RG2 datasheet v1.8, 1.3
    "rg2_lower_width": 65.0,        # RG2 datasheet v1.8, 1.3
    "d415_length": 99.0,            # D400 datasheet rev 017, table 3-51
    "d415_height": 23.0,
    "d415_depth": 20.0,
    "d415_m3_pitch": 45.0,          # D400 datasheet rev 017, figure 10-8
    "d415_m3_max_insertion": 3.0,
    # --- camera pose: estimated from the photo (config wrist_camera.offset_in_rg2_m, here in the CAD frame) -------
    "cam_x": env("CAM_X_MM", -33.0),
    "cam_y": env("CAM_Y_MM", -66.0),
    "cam_z": env("CAM_Z_MM", 117.0),
    "d415_back": env("BACK_MM", -10.2),         # vendor mesh: back face along the optical axis
    "d415_body_cx": env("BODY_CX_MM", 34.95),   # vendor mesh: body centre along optical x
    # --- clamp: estimated from the photo -------------------------------------------------------------------------
    "clamp_z0": 72.0,               # collar from here (just below the mounting bracket's arms, which end near Z = 69)
    "clamp_z1": 98.0,               # ... to here (the body starts to widen near Z = 100)
    "fit_clearance": 0.3,           # per side, on the 54 x 36 section
    "ear_width": 14.0,              # bolt boss beside the body, along X
    "front_thickness": 10.0,        # bar across the front face, along Y
    "back_thickness": 10.0,         # bar across the back face, along Y
    "split_gap": 0.6,               # between the halves, so the bolts clamp the body and not each other
    "front_bar_extension": 18.0,    # the front bar runs on down the flat front face, toward the AprilTag
    "bolt_pitch_z": 13.0,           # two bolts each side
    "bolt_clearance_d": 4.5,        # M4
    "bolt_head_d": 8.0,             # M4 socket head counterbore
    "bolt_head_depth": 4.4,
    "nut_across_flats": 7.2,        # M4 hex nut (7.0) plus clearance
    "nut_depth": 3.6,
    # --- arm and camera plate: estimated from the photo ----------------------------------------------------------
    "plate_thickness": 3.5,
    "plate_width": 41.0,            # along Y
    "plate_length": 100.0,          # along X
    "plate_shift_to_usb_end": 8.0,  # the plate overhangs the camera's cable end by this much more than the other end
    "plate_beyond_camera": 4.0,     # plate edge beyond the camera's far side (away from the gripper)
    "truss_member": 6.0,
    "truss_root_x": 24.0,           # where the truss meets the back bar on the far side of the body
    "truss_tip_x_from_camera_centre": -12.0,   # the truss meets the plate's half nearer the gripper
    "truss_tip_past_camera": 4.0,   # ... and reaches this far past the camera centreline, along -Y
    "m3_clearance_d": 3.4,
    # --- collision approximation: a design choice, checked against the pack's RG2 linkage ------------------------
    "collision_top_z_near_fingers": 104.0,
}

HX = P["rg2_waist_width"] / 2 + P["fit_clearance"]
HY = P["rg2_body_thickness"] / 2 + P["fit_clearance"]
HX_LOW = P["rg2_lower_width"] / 2 + 0.5                       # where the body has widened, below the collar
X_OUT = HX + P["ear_width"]
Y_FRONT, Y_BACK = HY + P["front_thickness"], -(HY + P["back_thickness"])
Z0, Z1 = P["clamp_z0"], P["clamp_z1"]
GAP = P["split_gap"] / 2
BOLT_X = HX + P["ear_width"] / 2
BOLT_Z = [(Z0 + Z1) / 2 - P["bolt_pitch_z"] / 2, (Z0 + Z1) / 2 + P["bolt_pitch_z"] / 2]

# The D415 in the RG2 frame: optical +x = -X, optical +y = -Y, optical +z = +Z.
BODY_CX = P["cam_x"] - P["d415_body_cx"]                      # camera body centre along X
CAM_BACK_Z = P["cam_z"] + P["d415_back"]                      # camera back face = plate front face
PLATE_Z0 = CAM_BACK_Z - P["plate_thickness"]
if PLATE_Z0 < Z1 + 2.0:
    raise SystemExit("BRACKET ERROR: the camera plate would sit inside the clamp collar; move the camera along +Z")
if P["cam_y"] + P["d415_height"] / 2 > Y_BACK - 8.0:
    raise SystemExit("BRACKET ERROR: the camera is too close to the RG2's back face for an arm")
PLATE_Y0 = P["cam_y"] - P["d415_height"] / 2 - P["plate_beyond_camera"]
PLATE_Y1 = PLATE_Y0 + P["plate_width"]
PLATE_X0 = BODY_CX - P["plate_length"] / 2 - P["plate_shift_to_usb_end"]
PLATE_X1 = PLATE_X0 + P["plate_length"]
M3_X = [BODY_CX - P["d415_m3_pitch"] / 2, BODY_CX + P["d415_m3_pitch"] / 2]


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def along_y(diameter, x, z, y0, y1):
    return Part.makeCylinder(diameter / 2, y1 - y0, Vector(x, y0, z), Vector(0, 1, 0))


def hexagon_along_y(across_flats, x, z, y0, y1):
    radius = across_flats / 2 / math.cos(math.pi / 6)
    points = [Vector(x + radius * math.cos(math.pi / 3 * k), y0, z + radius * math.sin(math.pi / 3 * k)) for k in range(7)]
    return Part.Face(Part.makePolygon(points)).extrude(Vector(0, y1 - y0, 0))


def bolts(solid, front):
    for side in (-1, 1):
        for z in BOLT_Z:
            x = side * BOLT_X
            solid = solid.cut(along_y(P["bolt_clearance_d"], x, z, Y_BACK - 1, Y_FRONT + 1))
            if front:   # captive nut, let into the front face
                solid = solid.cut(hexagon_along_y(P["nut_across_flats"], x, z, Y_FRONT - P["nut_depth"], Y_FRONT + 1))
            else:       # socket head, let into the back face
                solid = solid.cut(along_y(P["bolt_head_d"], x, z, Y_BACK - 1, Y_BACK + P["bolt_head_depth"]))
    return solid


def clamp_front():
    half = box(-X_OUT, X_OUT, GAP, Y_FRONT, Z0, Z1).cut(box(-HX, HX, GAP - 1, HY, Z0 - 1, Z1 + 1))
    bar = box(-P["rg2_waist_width"] / 2, P["rg2_waist_width"] / 2, HY, Y_FRONT, Z1, Z1 + P["front_bar_extension"])
    return bolts(half.fuse(bar), front=True).removeSplitter()


def camera_arm():
    # Everything rises from one flat face (the plate's camera face), so the piece prints on it without supports.
    half = box(-X_OUT, X_OUT, Y_BACK, -GAP, Z0, CAM_BACK_Z)
    half = half.cut(box(-HX, HX, -HY, 1, Z0 - 1, Z1))                       # the 54 x 36 waist
    half = half.cut(box(-HX_LOW, HX_LOW, -HY, 1, Z1, CAM_BACK_Z + 1))       # relief where the body widens
    half = bolts(half, front=False)
    tip = Vector(BODY_CX + P["truss_tip_x_from_camera_centre"], P["cam_y"] - P["truss_tip_past_camera"], 0)
    outline = [Vector(P["truss_root_x"], Y_BACK + 2.0, 0), Vector(-X_OUT, Y_BACK + 2.0, 0), tip]
    triangle = Part.Face(Part.makePolygon(outline + [outline[0]]))
    truss = triangle.cut(triangle.makeOffset2D(-P["truss_member"])).extrude(Vector(0, 0, CAM_BACK_Z - Z0))   # open window
    truss.translate(Vector(0, 0, Z0))
    plate = box(PLATE_X0, PLATE_X1, PLATE_Y0, PLATE_Y1, PLATE_Z0, CAM_BACK_Z)
    arm = half.fuse(truss).fuse(plate)
    for x in M3_X:
        arm = arm.cut(Part.makeCylinder(P["m3_clearance_d"] / 2, P["plate_thickness"] + 2, Vector(x, P["cam_y"], PLATE_Z0 - 1)))
    return arm.removeSplitter()


def describe(name, shape):
    box_ = shape.BoundBox
    return {"piece": name, "valid": bool(shape.isValid()), "solids": len(shape.Solids), "volume_cm3": round(shape.Volume / 1000, 2),
            "bounds_min_mm": [round(v, 2) for v in (box_.XMin, box_.YMin, box_.ZMin)],
            "bounds_max_mm": [round(v, 2) for v in (box_.XMax, box_.YMax, box_.ZMax)],
            "size_mm": [round(v, 2) for v in (box_.XLength, box_.YLength, box_.ZLength)]}


front, arm = clamp_front(), camera_arm()
pieces = {"clamp_front": front, "camera_arm": arm}
report = {"pieces": [describe(name, shape) for name, shape in pieces.items()], "parameters": P}
errors = [f"{item['piece']} is not one valid solid" for item in report["pieces"] if not item["valid"] or item["solids"] != 1]

# What the bracket must clear, as boxes in the RG2 frame.
d415 = box(BODY_CX - P["d415_length"] / 2, BODY_CX + P["d415_length"] / 2, P["cam_y"] - P["d415_height"] / 2,
           P["cam_y"] + P["d415_height"] / 2, CAM_BACK_Z, CAM_BACK_Z + P["d415_depth"])
waist = box(-P["rg2_waist_width"] / 2, P["rg2_waist_width"] / 2, -P["rg2_body_thickness"] / 2, P["rg2_body_thickness"] / 2, 60.0, 100.0)
lower = box(-P["rg2_lower_width"] / 2, P["rg2_lower_width"] / 2, -P["rg2_body_thickness"] / 2, P["rg2_body_thickness"] / 2, 100.0, 148.0)
# The finger linkage moves in the X-Z plane; in the pack's RG2 it stays within 16.5 mm of that plane and below Z = 111.
fingers = box(-80.0, 80.0, -16.5, 16.5, 111.0, 230.0)
checks = {"halves": front.common(arm).Volume}
for name, shape in pieces.items():
    for other, solid in (("d415", d415), ("rg2_waist", waist), ("rg2_lower_body", lower), ("finger_sweep", fingers)):
        checks[f"{name}_vs_{other}"] = shape.common(solid).Volume
errors += [f"{name} overlap {volume:.2f} mm3" for name, volume in checks.items() if volume > 1e-6]
report["interference_mm3"] = {name: round(volume, 4) for name, volume in checks.items()}
report["clearance_mm"] = {"camera_arm_to_finger_sweep": round(arm.distToShape(fingers)[0], 2),
                          "clamp_front_to_finger_sweep": round(front.distToShape(fingers)[0], 2),
                          "camera_to_rg2_back_face": round(-P["rg2_body_thickness"] / 2 - (P["cam_y"] + P["d415_height"] / 2), 2)}
report["hardware"] = {"cross_bolts": "4 x M4 socket head, about 55 mm, with 4 x M4 hex nuts in the front half",
                      "camera_screws": "2 x M3 x 6 (plate %.1f mm + at most %.0f mm into the camera)" % (P["plate_thickness"], P["d415_m3_max_insertion"])}
report["camera"] = {"optical_origin_mm": [P["cam_x"], P["cam_y"], P["cam_z"]], "body_centre_x_mm": round(BODY_CX, 2),
                    "back_face_z_mm": round(CAM_BACK_Z, 2), "m3_holes_xy_mm": [[round(x, 2), P["cam_y"]] for x in M3_X]}
# Collision approximation: five convex pieces in this CAD frame, each a list of corner points (the builder script makes
# one convex hull of each). The collider belongs to the RG2 body the bracket clamps, so the collar is its whole outer
# envelope (the space inside it is that same body). Near the finger linkage the pieces stop at COLLISION_TOP_Z, short
# of the printed part, so every piece stays at least 10 mm from the linkage over its whole stroke: more than two of
# Newton's 4 mm contact gaps, so no contact point is ever created (mcp/tests/test_workcell_real_pack.py sweeps it).
COLLISION_TOP_Z = P["collision_top_z_near_fingers"]


def corners(low, high):
    return [[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])]


TRUSS_XY = [[P["truss_root_x"], Y_BACK + 2.0], [-X_OUT, Y_BACK + 2.0],
            [BODY_CX + P["truss_tip_x_from_camera_centre"], P["cam_y"] - P["truss_tip_past_camera"]]]
report["collision_pieces_mm"] = {
    "collar": corners([-X_OUT, Y_BACK, Z0], [X_OUT, Y_FRONT, Z1]),
    "back_bar_upper": corners([-X_OUT, Y_BACK, Z1], [X_OUT, -HY, min(CAM_BACK_Z, COLLISION_TOP_Z)]),
    "front_bar_extension": corners([-P["rg2_waist_width"] / 2, HY, Z1],
                                   [P["rg2_waist_width"] / 2, Y_FRONT, min(Z1 + P["front_bar_extension"], COLLISION_TOP_Z)]),
    "truss": [[x, y, z] for z in (Z0, CAM_BACK_Z) for x, y in TRUSS_XY],
    "camera_plate": corners([PLATE_X0, PLATE_Y0, PLATE_Z0], [PLATE_X1, PLATE_Y1, CAM_BACK_Z]),
}
if errors:
    print("BRACKET ERROR: " + "; ".join(errors))
    raise SystemExit(1)
for name, shape in pieces.items():
    shape.exportStep(os.path.join(OUT, f"{name}.step"))
    vertices, facets = shape.tessellate(0.05)
    Mesh.Mesh([[tuple(vertices[i]) for i in facet] for facet in facets]).write(os.path.join(OUT, f"{name}.stl"))
Part.makeCompound([front, arm]).exportStep(os.path.join(OUT, "wrist_bracket.step"))
with open(os.path.join(OUT, "bracket.json"), "w") as handle:
    json.dump(report, handle, indent=1)
print("BRACKET " + json.dumps({"pieces": report["pieces"], "interference_mm3": report["interference_mm3"],
                               "clearance_mm": report["clearance_mm"]}))
