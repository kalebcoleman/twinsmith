# RG2 wrist camera bracket

The shipped v4 pack places the D415 at the 2026-11-16 measured colour optical pose. It supersedes the
photo-only 39 degree / 127 mm camera placement. The bracket is a two-piece collar around the RG2 body,
with a sloped triangular support and a rear camera plate. Its dimensions remain an approximation of the
printed part; calibration measures the optical frame, not the plastic.

## Calibration and frame conversion

The recorded parent-from-child transform is controller flange/tool0 (+Z out) from OpenCV colour optical
(+X right, +Y down, +Z forward). Its translation is (-1.192, 87.424, 90.239) mm. Config retains the full
matrix, source hashes, board specification and residuals in `wrist_camera.calibration`.
The pack's `flange` has +X out: it is not the calibration parent.

With the recorded no-Bota stack, RG2-from-optical translation is (87.424, 1.192, 92.739) mm and
extrinsic XYZ rotation is (29.423733, -0.881226, -89.900407) degrees. The optical root carries this rotation;
the USD Camera child applies Rx(180 degrees) exactly once. The housing uses the vendor optical offset,
about 35 mm along optical X from the colour lens, rather than centring the housing on the lens.
Changing the stack moves the camera with the RG2; it does not reuse the no-Bota flange transform unchanged.

The 1280 x 720 intrinsics are fx=923.8001708984375, fy=923.6292114257812,
cx=622.886474609375, cy=351.83782958984375. Both focal lengths and the principal point are retained in the
USD apertures/offsets. Recorded distortion coefficients are zero; rendering uses an ideal pinhole.

All 15 original images were retained. Offline replay reproduces pooled board reprojection RMS 0.862 px
using the recorded controller TCP poses. Board spread is 1.70 mm RMS / 3.83 mm maximum. Leave-one-view-out
refits give median held-out 0.779 px and maximum 1.714 px; transform changes reach 0.915 mm / 0.103 degrees.
Those refits start at the full fit: these are same-session robustness checks, not independent physical
accuracy or proof of global convergence. Classical methods put flange Z near 79 mm versus refined 90 mm.
Printed board scale and flatness have not been independently verified.

All recorded joint configurations were also evaluated through the actual USD articulation at rail 655 mm.
Nominal FK differs from controller TCP by 0.88–1.04 mm and 0.26–0.30 degrees, producing board RMS
5.98–7.68 px per view. Do not absorb that discrepancy into the optical pose.
The fully open vendor fingertips approximately follow the observed image outlines. Tape, pad shape and
actual opening remain uncertain; gripper-opening telemetry was not recorded. Threshold masks are diagnostic,
not labelled silhouette ground truth. The bracket and housing project outside all 15 colour views.

## Sourced dimensions

| Value | Source |
|---|---|
| RG2 body 54 mm wide and 36 mm thick where the collar sits; 65 mm wide lower down; mounting bracket 75 mm wide; 132 mm from the mounting plane to the lower end of the body; two M3 holes 20 mm apart on the front face | OnRobot RG2 datasheet v1.8, section 1.3: <https://onrobot.com/storage/datasheets/rg2/datasheet_rg2_v1.8_en.pdf> |
| D415 body 99 x 23 x 20 mm, 64 g; two M3 mounting points on the back, 45 mm apart, maximum insertion depth 3 mm; 1/4-20 thread on the underside | Intel RealSense D400 Series Datasheet rev 017 (September 2023), table 3-51 and figure 10-8: <https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf> |
| D415 colour sensor 14.5 mm from one end of the body, back face 10.2 mm behind the optical origin | RealSense's own mesh and URDF offsets (the pack's `d415` asset, `scripts/workcell_camera_assets.py`) |
| Quick Changer robot side: diameter 71 mm, 13.6 mm from the robot flange to the tool; tool side diameter 63 mm | OnRobot Quick Changers datasheet v2.0, sections 1.3 and 1.7: <https://onrobot.com/storage/datasheets/quick-changers/datasheet_quick_changers_v2.0_en.pdf> (context only: the bracket does not touch it) |
| UR5e tool flange EN ISO 9409-1-50-4-M6, M8 8-pin tool connector | Universal Robots UR5e technical specification: <https://www.universal-robots.com/media/1807465/ur5e_e-series_datasheets_web.pdf> (context only: the bracket does not touch it) |

Checked against OnRobot's official RG2 STEP ("RG2 Tier 2 - 101631", <https://onrobot.com/storage/303_rg2_tool.step>),
which is now the pack's RG2, 2026-11-14. The frame's origin is the rim of the Quick Changer's robot side, 2.5 mm on
the robot's side of the flange plane; the datasheet's mounting plane is at Z = 16:

| What the bracket model assumed | In OnRobot's CAD | Change |
|---|---|---|
| Body section at the collar 54 x 36 mm | 54.0 x 36.0 mm from Z = 71 to Z = 104 | none |
| That section centred on the tool axis | It leans 1.0 degree toward the back: its centre is 0.39 mm behind the axis at Z = 72 and 0.83 mm at Z = 98 (front face at +17.6 to +17.2 mm, back face at -18.4 to -18.8 mm) | The collar's pocket is centred 0.6 mm behind the axis (`rg2_waist_centre_y`); with the pocket on the axis the back half overlapped the body by up to 0.5 mm |
| Mounting bracket's arms end near Z = 69 | Z = 70.7 | none: the collar starts at Z = 72, 1.3 mm below them |
| Body widens near Z = 100 | From Z = 104; 65 mm wide from Z = 120 | none: the collar ends at Z = 98 |
| Finger linkage within 16.5 mm of the mid-plane on both sides | 16.7 mm behind it and 13.9 mm in front of it, below Z = 111 | The printed front bar is 3.7 mm in front of the linkage's slab (it was taken as 1.8 mm) |
| Lower end of the body at Z = 148 | Z = 148.3 | none |

## Estimated from photo, measure to confirm

The supplied side photos show a collar below the RG2 mounting arms, two visible cross bolts per side,
a triangular support toward the camera on the face opposite the AprilTag, and taped fingertips.
They support the arrangement and approximate tilt; they do not provide a metric reconstruction of the bracket.
The model keeps a 72–98 mm collar, 0.3 mm radial fit clearance, 0.6 mm split, 18 mm front-bar extension,
35 mm plate width and 3.5 mm plate thickness. Bolt sizes, window and support thickness are estimates.
The support is rebuilt to meet the calibrated camera back plane, without imposing an artificial minimum
camera height. No camera transform was adjusted to conceal a geometry mismatch.

The CAD frame is +X finger opening, +Y toward the AprilTag, +Z toward the fingertips.
RG2 body coordinates are (-CAD_Y, CAD_X, CAD_Z). The housing centre is approximately
(-33.75, -87.57, 93.12) mm in CAD coordinates, so a near-centred colour lens does not imply a centred housing.
The model omits cable routing and tape thickness. Do not infer exact lens-to-tip alignment from perspective photos.

## Collision

Five convex hulls (`collar`, `front_bar_extension`, `truss`, `plate_support`, `camera_plate`) attach to the
RG2 rigid body, with no filtered collision pairs. Convex triangulation supports arbitrary vertex counts.
The front extension collider stops at Z=104 mm, 12 mm short of the visual bar, to clear the finger linkage.
Tests sweep the full gripper range and require at least 9 mm between bracket colliders and other moving bodies
(two Newton contact gaps plus margin). They also check the bench at zero and tool-down arm poses, rail endpoints,
and with/without Bota. This is geometric clearance testing, not a dynamic solver qualification.
The D415 housing remains visual only; the bracket and housing masses are carried by the RG2 body.
The bracket's 0.100 kg remains an unweighed estimate; its centre of gravity follows the rebuilt CAD.

FreeCAD reports two valid solids, zero overlap with the gripper/camera/finger sweep or between the pieces.
The clamp volume is 41.70 cm³ and arm volume 63.51 cm³. Visual front-bar clearance to the finger sweep is
3.7 mm; the arm clears it by 13 mm. These values describe the model, not measurements of the printed part.

## Build, print and assemble

From `mcp/`, with existing FreeCAD and the verified v3 pack:

```sh
uv run --locked --isolated --extra scene-test python ../scripts/workcell_calibrated_camera_assets.py
```

This copies v3 unchanged except for the wrist mount into a new self-contained v4 pack; existing v4 is left alone.
The raw CAD entry point is `BRACKET_OUT=<folder> freecadcmd exts/workcell/cad/rg2_camera_bracket.py`.
It exports STEP/STL and `bracket.json`; defaults match the shipped full optical pose. `CAM_X_MM`, `CAM_Y_MM`,
`CAM_Z_MM`, `TILT_DEG`, `PITCH_DEG`, `YAW_DEG`, `BACK_MM` and `BODY_CX_MM` allow explicit overrides.
The builder accepts tilt 0–45 degrees and pitch/yaw deviations below 5 degrees from [tilt, 0, -90].
The legacy v2-to-v3 builder remains available for rebuilding the vendor geometry; the calibrated builder needs
no vendor download. `rg2_camera_bracket_v3.py` beside the current model is the frozen model of pack v3 (the photo-only
39 degree / 127 mm pose): the v2-to-v3 builder runs it at that pinned pose, so a rebuilt v3 keeps v3's bracket. The
current model does not cover that pose, and the frozen one does not cover the measured pitch and yaw. These models are simulation approximations, not validated manufacturing drawings.
