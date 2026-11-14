"""Wrist camera bracket for the UR5e, as a FreeCAD model (run with ``freecadcmd``; millimetres, tool0 frame).

A split clamp ring around the wrist 3 housing behind the tool flange, an arm out to the underside, a web, and a tilted
plate the D415 screws to through its two M3 rear holes (45 mm apart). Clamping the housing leaves the flange stack (Bota,
Quick Changer, RG2) and so the tool kinematics unchanged. A buildable placeholder for the pose in
``config/lab_hardware.json``; the wrist diameter and the fit on the real arm are not measured.

Environment: BRACKET_OUT (folder), CAMERA_Y_MM, CAMERA_Z_MM, TILT_DEG (toward the tool axis), BACK_MM (camera back
face along the optical axis, negative), BODY_CX_MM (camera body centre along optical x).
Writes wrist_bracket.step and wrist_bracket.stl.
"""
import math
import os

import FreeCAD
import Mesh
import Part
from FreeCAD import Matrix, Vector

out = os.environ["BRACKET_OUT"]
cam_y, cam_z = float(os.environ["CAMERA_Y_MM"]), float(os.environ["CAMERA_Z_MM"])
tilt = math.radians(float(os.environ["TILT_DEG"]))
back, body_cx = float(os.environ["BACK_MM"]), float(os.environ["BODY_CX_MM"])

WRIST_R, WALL = 37.6, 8.0                 # Ø75 wrist 3 housing plus 0.1 mm clearance; ring wall
Z0, Z1 = -36.0, -20.0                     # ring, behind the Ø63 tool flange
PLATE = (62.0, 23.0, 4.0)                 # camera plate: along optical x, optical y, thickness
HOLE_PITCH, M3, M4 = 45.0, 3.4, 4.3

ring = Part.makeCylinder(WRIST_R + WALL, Z1 - Z0, Vector(0, 0, Z0))
ears = Part.makeBox(19.0, 14.0, Z1 - Z0, Vector(-9.5, -(WRIST_R + WALL) - 12.0, Z0))
# The plate's centre in tool0 (mid thickness): the arm runs under it and a web rises from the arm into the plate.
c, s = math.cos(tilt), math.sin(tilt)
mid = back - PLATE[2] / 2
plate_y, plate_z = cam_y - mid * s, cam_z + mid * c
arm = Part.makeBox(20.0, plate_y + 5.0 - 40.0, 8.0, Vector(-10.0, 40.0, -32.0))
web = Part.makeBox(20.0, 10.0, plate_z + 32.0, Vector(-10.0, plate_y - 5.0, -32.0))
body = ring.fuse(ears).fuse(arm).fuse(web)
body = body.cut(Part.makeCylinder(WRIST_R, Z1 - Z0 + 2, Vector(0, 0, Z0 - 1)))
body = body.cut(Part.makeBox(3.0, 30.0, Z1 - Z0 + 2, Vector(-1.5, -(WRIST_R + WALL) - 13.0, Z0 - 1)))        # clamp slot
body = body.cut(Part.makeCylinder(M4 / 2, 30.0, Vector(-15.0, -(WRIST_R + WALL) - 6.0, (Z0 + Z1) / 2), Vector(1, 0, 0)))

# The plate is built in the camera's optical frame (x right, y down, z forward), then placed in tool0:
# optical axes in tool0 for a camera on +y, turned 180 degrees about the tool axis and tilted toward it.
plate = Part.makeBox(PLATE[0], PLATE[1], PLATE[2], Vector(body_cx - PLATE[0] / 2, -PLATE[1] / 2, back - PLATE[2]))
for side in (-1, 1):
    plate = plate.cut(Part.makeCylinder(M3 / 2, PLATE[2] + 2, Vector(body_cx + side * HOLE_PITCH / 2, 0, back - PLATE[2] - 1)))
placement = Matrix(-1, 0, 0, 0,
                   0, -c, -s, cam_y,
                   0, -s, c, cam_z,
                   0, 0, 0, 1)
plate = plate.transformGeometry(placement)
bracket = body.fuse(plate).removeSplitter()
if not bracket.isValid() or len(bracket.Solids) != 1:
    print(f"BRACKET ERROR: not one valid solid ({len(bracket.Solids)} solids); the plate does not meet the arm")
    raise SystemExit(1)
bracket.exportStep(os.path.join(out, "wrist_bracket.step"))
vertices, facets = bracket.tessellate(0.05)
Mesh.Mesh([[tuple(vertices[i]) for i in facet] for facet in facets]).write(os.path.join(out, "wrist_bracket.stl"))
print(f"BRACKET volume_cm3={bracket.Volume / 1000:.2f} bounds={bracket.BoundBox}")
