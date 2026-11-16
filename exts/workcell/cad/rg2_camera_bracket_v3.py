"""Wrist camera bracket on the OnRobot RG2, as a FreeCAD model (run with ``freecadcmd``; millimetres).

FROZEN MODEL OF PACK ``ur5e-workcell/v3``. This is the bracket model as it was when v3 was built (2026-11-14), kept
unchanged so that ``scripts/workcell_bracket_assets.py`` rebuilds v3's photo-estimated bracket at its pinned pose
(127 mm behind the tool axis, 39 degree tilt). Do not edit it and do not use it for the current cell: the shipped
``ur5e-workcell/v4`` bracket is ``rg2_camera_bracket.py``, built around the 2026-11-16 measured optical pose, and that
model does not cover the v3 pose. The pose and photo statements below describe v3, not the current camera.

A parametric copy of the 3D-printed bracket on the lab's UR5e, read from one photo (see ``README.md`` beside this file).
It is NOT a plate between flange faces: it is a two-piece clamp collar around the RG2 body's 54 x 36 mm waist, below the
gripper's own mounting bracket and above the finger pivots, so the tool stack and the tool kinematics are unchanged and
the camera turns with the gripper. Two printed pieces:

* ``clamp_front``: the half on the RG2's front face (the face with the two M3 holes, where the lab's AprilTag is), with
  captive M4 nut pockets and a bar that runs down the front face;
* ``camera_arm``: the half on the back face, a triangulated truss arm with an open window, and the flat plate the D415
  screws to through its two rear M3 holes. The plate is tilted about the camera's long axis so the camera looks at the
  fingertips; the piece prints on the collar's upper face (Z0 down) with supports under the overhanging plate.

Four M4 cross bolts (two each side of the body, along the body's thickness) pull the halves together across a split.

CAD frame: origin on the tool axis at the rim of the Quick Changer's robot side (2.5 mm on the robot's side of the
flange plane), +Z along the approach axis toward the fingertips, +X the finger-opening axis, +Y out of the front
face. The RG2 datasheet's mounting plane is at Z = 16 (its 132 mm body length ends at Z = 148). The work cell pack's
RG2 body frame (``onrobot_rg2_base_link``) is this frame
turned a quarter turn about Z (body +Y = CAD +X, body +X = CAD -Y); ``scripts/workcell_bracket_assets.py`` converts.

Sourced dimensions (vendor documents, fetched 2026-11-14):

* RG2 body 54 mm wide and 36 mm thick at the waist, 65 mm wide lower down, 132 mm from the mounting plane to the lower
  end of the body, mounting bracket 75 mm wide: OnRobot RG2 datasheet v1.8, section 1.3,
  https://onrobot.com/storage/datasheets/rg2/datasheet_rg2_v1.8_en.pdf
* Checked against OnRobot's official RG2 STEP (https://onrobot.com/storage/303_rg2_tool.step, 2026-11-14): the body's
  section where the collar sits is 54.0 x 36.0 mm, as the datasheet says, from Z = 71 to Z = 104; the mounting
  bracket's arms end at Z = 70.7; the body widens from Z = 104 and is 65 mm wide from Z = 120. The section is not
  centred on the tool axis: it leans 1.0 degree toward the back, its centre 0.39 mm behind the axis at Z = 72 and
  0.83 mm behind it at Z = 98 (``rg2_waist_centre_y``).
* D415 body 99 x 23 x 20 mm, two M3 mounting points on the back 45 mm apart with a maximum insertion depth of 3 mm:
  Intel RealSense D400 Series Datasheet rev 017 (September 2023), table 3-51 and figure 10-8,
  https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf

Everything else below is ESTIMATED FROM THE PHOTO (or a design choice where the photo shows nothing) and must be
measured; ``README.md`` lists each value with how it was scaled and its uncertainty.

Environment (all optional except BRACKET_OUT): BRACKET_OUT (folder), CAM_X_MM, CAM_Y_MM, CAM_Z_MM (the D415 colour
optical origin), TILT_DEG, BACK_MM (camera back face along the optical axis, negative), BODY_CX_MM (camera body
centre along optical x), all in this CAD frame. The camera's long axis is along X with optical +x = -X; at zero tilt
it looks along +Z with optical +y = -Y (away from the gripper). TILT_DEG turns it about its long axis so the optical
axis leans toward the tool axis (+Y). No other orientation is modelled.
Writes clamp_front.step/.stl, camera_arm.step/.stl, wrist_bracket.step (both pieces as mounted) and bracket.json.
"""
import json
import math
import os

import FreeCAD
import Mesh
import Part
from FreeCAD import Matrix, Vector

OUT = os.environ["BRACKET_OUT"]


def env(name, default):
    return float(os.environ.get(name, default))


P = {
    # --- sourced -------------------------------------------------------------------------------------------------
    "rg2_waist_width": 54.0,        # RG2 datasheet v1.8, 1.3
    "rg2_body_thickness": 36.0,     # RG2 datasheet v1.8, 1.3
    "rg2_lower_width": 65.0,        # RG2 datasheet v1.8, 1.3
    "rg2_waist_centre_y": -0.6,     # OnRobot's STEP: the 54 x 36 section's centre at the collar (-0.39 at Z 72, -0.83 at Z 98)
    "rg2_waist_end_z": 104.0,       # OnRobot's STEP: the body starts to widen here
    "d415_length": 99.0,            # D400 datasheet rev 017, table 3-51
    "d415_height": 23.0,
    "d415_depth": 20.0,
    "d415_m3_pitch": 45.0,          # D400 datasheet rev 017, figure 10-8
    "d415_m3_max_insertion": 3.0,
    # --- camera pose: estimated from the photo (config wrist_camera.offset_in_rg2_m, here in the CAD frame) -------
    "cam_x": env("CAM_X_MM", 35.0),             # Kaleb: the camera is in line with the gripper, so its body is centred on X = 0
    "cam_y": env("CAM_Y_MM", -127.0),           # Kaleb: further from the gripper than first modelled; photo fit with X fixed
    "cam_z": env("CAM_Z_MM", 102.0),            # as far toward the wrist as this arm shape allows; the fit says higher still
    "tilt_deg": env("TILT_DEG", 39.0),          # about the camera's long axis, optical axis toward the tool axis
    "d415_back": env("BACK_MM", -10.2),         # vendor mesh: back face along the optical axis
    "d415_body_cx": env("BODY_CX_MM", 34.95),   # vendor mesh: body centre along optical x
    # --- clamp: estimated from the photo -------------------------------------------------------------------------
    "clamp_z0": 72.0,               # collar from here (the mounting bracket's arms end at Z = 70.7 in OnRobot's STEP)
    "clamp_z1": 98.0,               # ... to here (the body starts to widen at Z = 104)
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
    "plate_width": 35.0,            # across the camera (its 23 mm side)
    "plate_length": 100.0,          # along X
    "plate_shift_to_usb_end": 8.0,  # the plate overhangs the camera's cable end by this much more than the other end
    "plate_beyond_camera": 6.0,     # plate edge beyond the camera's far side (away from the gripper)
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
YF, YB = P["rg2_waist_centre_y"] + HY, P["rg2_waist_centre_y"] - HY      # the pocket's front and back faces
X_OUT = HX + P["ear_width"]
Y_FRONT, Y_BACK = YF + P["front_thickness"], YB - P["back_thickness"]
Z0, Z1 = P["clamp_z0"], P["clamp_z1"]
GAP = P["split_gap"] / 2
BOLT_X = HX + P["ear_width"] / 2
BOLT_Z = [(Z0 + Z1) / 2 - P["bolt_pitch_z"] / 2, (Z0 + Z1) / 2 + P["bolt_pitch_z"] / 2]

# The D415 in the CAD frame: optical +x = -X; optical +y and +z are -Y and +Z turned by the tilt about X.
TILT = math.radians(P["tilt_deg"])
if not 0.0 <= P["tilt_deg"] <= 45.0:
    raise SystemExit("BRACKET ERROR: the model covers a tilt of 0 to 45 degrees toward the tool axis")
SIN, COS = math.sin(TILT), math.cos(TILT)
ORIGIN = Vector(P["cam_x"], P["cam_y"], P["cam_z"])
EX, EY, EZ = Vector(-1, 0, 0), Vector(0, -COS, SIN), Vector(0, SIN, COS)
CAMERA = Matrix(EX.x, EY.x, EZ.x, ORIGIN.x,
                EX.y, EY.y, EZ.y, ORIGIN.y,
                EX.z, EY.z, EZ.z, ORIGIN.z,
                0, 0, 0, 1)                                   # optical coordinates to the CAD frame
BODY_CX = P["cam_x"] - P["d415_body_cx"]                      # camera body centre along X
PLATE_BACK = P["d415_back"] - P["plate_thickness"]            # the plate's back face along the optical axis
# The plate in optical coordinates: x toward the cable end, y from the gripper side to the far side.
PLATE_OX = (P["d415_body_cx"] - P["plate_length"] / 2 + P["plate_shift_to_usb_end"],
            P["d415_body_cx"] + P["plate_length"] / 2 + P["plate_shift_to_usb_end"])
PLATE_OY = (P["d415_height"] / 2 + P["plate_beyond_camera"] - P["plate_width"], P["d415_height"] / 2 + P["plate_beyond_camera"])
M3_OX = [P["d415_body_cx"] - P["d415_m3_pitch"] / 2, P["d415_body_cx"] + P["d415_m3_pitch"] / 2]


def in_cad(x, y, z):
    """A point given in the camera's optical coordinates, in the CAD frame."""
    return ORIGIN + EX * x + EY * y + EZ * z


PLATE_NEAR = in_cad(0, PLATE_OY[0], PLATE_BACK)               # the back face's edge nearest the gripper
PLATE_MID = in_cad(0, 0, PLATE_BACK)                          # the back face under the camera's centreline
Z_CAP = PLATE_NEAR.z                                          # the truss stops here outside the plate's footprint
if Z_CAP < Z0 + 8.0:
    raise SystemExit("BRACKET ERROR: the camera plate would sit at or below the collar; move the camera along +Z")
if in_cad(0, -P["d415_height"] / 2, P["d415_back"]).y > Y_BACK - 8.0 or PLATE_NEAR.y > Y_BACK - 6.0:
    raise SystemExit("BRACKET ERROR: the camera is too close to the RG2's back face for an arm")


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def optical_box(x0, x1, y0, y1, z0, z1):
    """A box given in the camera's optical coordinates."""
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0)).transformGeometry(CAMERA)


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
    half = box(-X_OUT, X_OUT, GAP, Y_FRONT, Z0, Z1).cut(box(-HX, HX, GAP - 1, YF, Z0 - 1, Z1 + 1))
    bar = box(-P["rg2_waist_width"] / 2, P["rg2_waist_width"] / 2, YF, Y_FRONT, Z1, Z1 + P["front_bar_extension"])
    return bolts(half.fuse(bar), front=True).removeSplitter()


def truss_solid():
    """The arm: a triangle with an open window, standing on Z0. Outside the plate's footprint it stops level with the
    plate's nearest edge; under the plate it rises to the plate's tilted back face."""
    tip = Vector(BODY_CX + P["truss_tip_x_from_camera_centre"], PLATE_MID.y - P["truss_tip_past_camera"], 0)
    outline = [Vector(P["truss_root_x"], Y_BACK + 2.0, 0), Vector(-X_OUT, Y_BACK + 2.0, 0), tip]
    triangle = Part.Face(Part.makePolygon(outline + [outline[0]]))
    column = triangle.cut(triangle.makeOffset2D(-P["truss_member"])).extrude(Vector(0, 0, 120.0))   # open window
    column.translate(Vector(0, 0, Z0))
    low = column.common(box(-300, 300, -300, 300, Z0, Z_CAP))
    under = column.common(optical_box(-300, 300, PLATE_OY[0], PLATE_OY[1], -400, PLATE_BACK))
    return low, under, outline


def camera_arm():
    half = box(-X_OUT, X_OUT, Y_BACK, -GAP, Z0, Z1)
    half = bolts(half.cut(box(-HX, HX, YB, 1, Z0 - 1, Z1 + 1)), front=False)        # the 54 x 36 waist
    low, under, _ = truss_solid()
    plate = optical_box(PLATE_OX[0], PLATE_OX[1], PLATE_OY[0], PLATE_OY[1], PLATE_BACK, P["d415_back"])
    arm = half.fuse(low).fuse(under).fuse(plate)
    for x in M3_OX:
        hole = Part.makeCylinder(P["m3_clearance_d"] / 2, P["plate_thickness"] + 2, Vector(x, 0, PLATE_BACK - 1))
        arm = arm.cut(hole.transformGeometry(CAMERA))
    return arm.removeSplitter()


def describe(name, shape):
    box_ = shape.BoundBox
    return {"piece": name, "valid": bool(shape.isValid()), "solids": len(shape.Solids), "volume_cm3": round(shape.Volume / 1000, 2),
            "centre_of_volume_mm": [round(v, 2) for v in shape.Solids[0].CenterOfMass],
            "bounds_min_mm": [round(v, 2) for v in (box_.XMin, box_.YMin, box_.ZMin)],
            "bounds_max_mm": [round(v, 2) for v in (box_.XMax, box_.YMax, box_.ZMax)],
            "size_mm": [round(v, 2) for v in (box_.XLength, box_.YLength, box_.ZLength)]}


front, arm = clamp_front(), camera_arm()
pieces = {"clamp_front": front, "camera_arm": arm}
report = {"pieces": [describe(name, shape) for name, shape in pieces.items()], "parameters": P}
errors = [f"{item['piece']} is not one valid solid" for item in report["pieces"] if not item["valid"] or item["solids"] != 1]

# What the bracket must clear: the camera (tilted), the RG2 body, the slab the finger linkage moves in, and the
# path of a screwdriver to the two M3 screws behind the plate.
d415 = optical_box(P["d415_body_cx"] - P["d415_length"] / 2, P["d415_body_cx"] + P["d415_length"] / 2, -P["d415_height"] / 2,
                   P["d415_height"] / 2, P["d415_back"], P["d415_back"] + P["d415_depth"])
# The body as OnRobot's STEP has it, in boxes: the leaning 54 x 36 section within 0.25 mm at the collar, and the wider
# lower body (back face at -19.6 mm at its lower end, front face at 17.1 mm at its upper end).
waist = box(-P["rg2_waist_width"] / 2, P["rg2_waist_width"] / 2, P["rg2_waist_centre_y"] - P["rg2_body_thickness"] / 2 - 0.25,
            P["rg2_waist_centre_y"] + P["rg2_body_thickness"] / 2 + 0.25, 71.0, P["rg2_waist_end_z"])
lower = box(-P["rg2_lower_width"] / 2, P["rg2_lower_width"] / 2, -19.6, 17.1, P["rg2_waist_end_z"], 148.3)
# The finger linkage moves in the X-Z plane; in OnRobot's CAD it stays between 16.7 mm behind that plane and 13.9 mm in
# front of it, and below Z = 111.
fingers = box(-80.0, 80.0, -16.7, 14.0, 111.0, 230.0)
screws = [Part.makeCylinder(4.0, 60.0, Vector(x, 0, PLATE_BACK - 60.0 - 0.01)).transformGeometry(CAMERA) for x in M3_OX]
checks = {"halves": front.common(arm).Volume, "d415_vs_rg2_body": d415.common(waist).Volume + d415.common(lower).Volume,
          "d415_vs_finger_sweep": d415.common(fingers).Volume,
          "screwdriver_access": sum(arm.common(screw).Volume + front.common(screw).Volume + waist.common(screw).Volume
                                    + lower.common(screw).Volume for screw in screws)}
for name, shape in pieces.items():
    for other, solid in (("d415", d415), ("rg2_waist", waist), ("rg2_lower_body", lower), ("finger_sweep", fingers)):
        checks[f"{name}_vs_{other}"] = shape.common(solid).Volume
errors += [f"{name} overlap {volume:.2f} mm3" for name, volume in checks.items() if volume > 1e-6]
report["interference_mm3"] = {name: round(volume, 4) for name, volume in checks.items()}
report["clearance_mm"] = {"camera_arm_to_finger_sweep": round(arm.distToShape(fingers)[0], 2),
                          "clamp_front_to_finger_sweep": round(front.distToShape(fingers)[0], 2),
                          "camera_to_rg2_body": round(min(d415.distToShape(waist)[0], d415.distToShape(lower)[0]), 2),
                          "camera_to_finger_sweep": round(d415.distToShape(fingers)[0], 2),
                          "camera_arm_to_rg2_lower_body": round(arm.distToShape(lower)[0], 2)}
report["hardware"] = {"cross_bolts": "4 x M4 socket head, about 55 mm, with 4 x M4 hex nuts in the front half",
                      "camera_screws": "2 x M3 x 6 (plate %.1f mm + at most %.0f mm into the camera)" % (P["plate_thickness"], P["d415_m3_max_insertion"])}
centre = in_cad(P["d415_body_cx"], 0, P["d415_back"] + P["d415_depth"] / 2)
report["camera"] = {"optical_origin_mm": [P["cam_x"], P["cam_y"], P["cam_z"]], "tilt_deg": P["tilt_deg"],
                    "optical_axis": [round(v, 4) for v in EZ], "body_centre_mm": [round(v, 2) for v in centre],
                    "plate_back_face_nearest_edge_mm": [round(v, 2) for v in PLATE_NEAR],
                    "m3_holes_mm": [[round(v, 2) for v in in_cad(x, 0, P["d415_back"])] for x in M3_OX]}
# Collision approximation: five convex pieces in this CAD frame, each a list of corner points (the builder script makes
# one convex hull of each). The collider belongs to the RG2 body the bracket clamps, so the collar is its whole outer
# envelope (the space inside it is that same body). Near the finger linkage the front bar stops at COLLISION_TOP_Z,
# short of the printed part, so every piece stays at least 10 mm from the linkage over its whole stroke: more than two
# of Newton's 4 mm contact gaps, so no contact point is ever created (mcp/tests/test_workcell_real_pack.py sweeps it).
COLLISION_TOP_Z = P["collision_top_z_near_fingers"]


def corners(low, high):
    return [[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])]


_, UNDER, OUTLINE = truss_solid()
if UNDER.Volume < 1.0:
    errors.append("the truss does not reach under the camera plate")
SPAN_X = [vertex.X for vertex in UNDER.Vertexes] or [0.0]    # the part of the truss under the plate (its vertices:
SUPPORT_Y = min([vertex.Y for vertex in UNDER.Vertexes] or [0.0])   # a boolean result's bounding box is not tight)
SUPPORT_Z = PLATE_MID.z - (SUPPORT_Y - PLATE_MID.y) * SIN / COS     # the plate's back face above that far end
report["collision_pieces_mm"] = {
    "collar": corners([-X_OUT, Y_BACK, Z0], [X_OUT, Y_FRONT, Z1]),
    "front_bar_extension": corners([-P["rg2_waist_width"] / 2, YF, Z1],
                                   [P["rg2_waist_width"] / 2, Y_FRONT, min(Z1 + P["front_bar_extension"], COLLISION_TOP_Z)]),
    "truss": [[point.x, point.y, z] for z in (Z0, Z_CAP) for point in OUTLINE],
    "plate_support": [[x, y, z] for x in (min(SPAN_X), max(SPAN_X))
                      for y, z in ((PLATE_NEAR.y, Z_CAP), (SUPPORT_Y, Z_CAP), (SUPPORT_Y, max(SUPPORT_Z, Z_CAP + 0.01)))],
    "camera_plate": [list(in_cad(x, y, z)) for x in PLATE_OX for y in PLATE_OY for z in (PLATE_BACK, P["d415_back"])],
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
