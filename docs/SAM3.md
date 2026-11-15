# SAM 3 video masks (`sam3`)

`sam3` runs SAM 3's video predictor over a video file or a recorded RGB-D clip. For each text prompt it follows every
matching object through the clip and writes one mask image per frame. When the clip has depth and intrinsics, each
object also gets a 3-D point, and a box of known size gets a pose. It runs on the GPU in the SAM 3 tool bundle's own
Python environment and needs no Isaac Sim or Kit bridge. The tool is in the `real2sim` toolset, which is off by default
(`toolsets(action="enable", names=["real2sim"])`).

**It reads files only.** It does not open a camera, subscribe to a ROS topic or read a bag. A recorder writes the clip
folder described below; [`scripts/jenga_robot_loop_kit.py`](../scripts/jenga_robot_loop_kit.py) is one, for any
simulated camera: the work cell's wrist D415 or [a second camera](#a-second-camera-and-its-calibration). A real camera's
bag has to be exported to the same folder first; no exporter is provided.

## The environment

The host never imports torch or SAM 3. The model runs with the interpreter of the `sam3` bundle in the
`open-robot-skills` checkout beside Graph-as-Policy (`<workspace>/tools/open-robot-skills/tools/sam3/.venv`), the same
checkout and pin the [`gap`](PLANNING-EVAL.md#gap) tool reports. The owner creates it (`uv sync` in the bundle's folder)
and downloads the gated `facebook/sam3` weights once; [PLANNING-EVAL.md](PLANNING-EVAL.md) has the steps. `status` checks
the interpreter, that torch sees CUDA and that the weights are in the Hugging Face cache. A job runs offline
(`HF_HUB_OFFLINE=1`) and is given no token.

## The source

`source` is an absolute path to one of:

- **a video file** (`.mp4`, `.mov`, `.avi`, `.mkv`, `.webm`). `start_s` and `end_s` cut a window; `every` keeps every
  n-th frame. Masks only: a video has no depth.
- **a clip folder**:

  ```text
  <clip>/
    rgb/00000.png ...      colour frames (.png or .jpg), used in name order
    depth/00000.png ...    optional, one per colour frame: 16-bit PNG counts, or .npy in metres
    camera.json            needed with depth/
  ```

  `camera.json` holds `fx`, `fy`, `cx`, `cy` in pixels and may hold `depth_scale` (metres per PNG count, default
  0.001, a RealSense's millimetres), `fps`, and the camera's pose: `world_from_camera` (4 x 4, translation in the last
  column) for a fixed camera, or `frames: [{"world_from_camera": ...}, ...]` with one entry per colour frame for a
  camera that moves. Depth is distance along the optical axis, aligned to the colour image. The camera frame is the
  optical frame: +X right, +Y down, +Z forward.

Each refusal has its own code: `SOURCE_INVALID` (not a video, no `rgb/`, depth and colour counts differ),
`CAMERA_INVALID`, `DEPTH_MISSING` (a box size for a source without depth), `ALREADY_EXISTS` (a run name is used once).

## Actions

| Action | Does |
|---|---|
| `status` | The environment, the checkout's commit against the pin, and the five newest runs |
| `track` | Checks the source and the prompts, then starts a GPU job. `prompts` is a list of `TAG=TEXT`, for example `["tower=tower of wooden blocks", "blocks=wooden block"]`; `box_size_m` is `{tag: [length, width, height]}` for the tags whose objects are boxes of that size; `zone` is `{tag: zone tag}`, for example `{"hand": "tower"}` |
| `report` | `running` while the job runs. When it has finished: per prompt the number of frames, objects and poses, each zone's summary, and the paths of the outputs. With `tag` (and `frame`, default 0, -1 for the last) it also returns that frame's objects |
| `fuse` | Merges the box poses of `tag` from several finished `runs`, one per view of the same scene, into one list of boxes in `<name>/fused.json`. No job, no GPU |

`track` copies `sam3_driver.py` into `<workspace>/runs/sam3/<name>/` as `driver.py`, writes `request.json` and runs the
driver as a [job](TOOLS-ISAACLAB.md): follow it with `jobs(action="status", job_id=...)`. It refuses to start beside
another GPU job or a running simulator unless `allow_concurrent_gpu=true`.

## Outputs

In `<workspace>/runs/sam3/<name>/`:

| File | What |
|---|---|
| `frames/00000.jpg ...` | The frames SAM 3 saw |
| `<tag>/labels_NNNN.png` | One 8-bit image per frame: 0 is nothing, 1..N are that prompt's objects, numbered in order of first appearance. An object keeps its number through the clip. Larger objects are drawn first, so a small one inside a large one stays visible |
| `<tag>.json` | Per frame and object: `id`, `area`, centroid `cx`, `cy`, `bbox`, the mask's principal-axis angle `axis_deg`, `score`; with depth also `xyz_m` and, for a box, `pose` |
| `overlay.mp4` | Every prompt's masks tinted over the frames |
| `result.json` | The summary that `report` returns |

## Box poses from depth

A camera sees one face of a block in a tower. For an object whose prompt has a box size, the driver takes the largest
plane in the object's depth points (RANSAC with a 2 mm tolerance, so points of a second face or of the background at
the mask's edge are left out), finds the rectangle of least area around the plane's points, and names the face by
the pair of box sides its two extents are nearest to: `top` (length x width), `side` (length x height) or `end`
(width x height). The box's centre is half the remaining side behind the face.

`pose` holds `position_m` and `quaternion_xyzw` in the camera frame, `position_world_m` and `quaternion_world_xyzw`
when the clip gives the camera's pose, the `face`, its measured `extent_m`, `extent_error_m` (how far the extents are
from the box's sides, summed), `face_points`, `face_fraction` and `plane_rms_m`.

Limits to know:

- A box looks the same after a half turn about any of its axes, so the rotation is one of four equivalent ones.
- A face cut by an occluder or by the image edge reads short and can be named wrongly; check `extent_error_m`.
- Blocks hidden behind the visible face are not reported. From one side of a Jenga tower that is two of every three
  blocks in every other layer.
- Nothing is filtered over time: each frame's pose comes from that frame alone.

## Is a hand at the tower: zones

`zone={"hand": "tower"}` asks, frame by frame, whether an object of the first prompt is at the second prompt's
objects. The zone is the box around the zone objects' masks, grown on every side by half its longer side (for a
tower seen whole, about 12 cm beyond it). An
object is in it when its own box overlaps it and, where both have a depth point, the two are within 0.30 m along the
optical axis, so a hand close to the camera and far in front of the tower does not count.

Each object row of the zoned prompt gets `in_zone`, and `report` returns per zone `frames_present`,
`first_present_frame`, `last_present_frame` and `present_in_last_frame`. The robot loop waits on this: it goes on only
after a hand has been seen at the tower and two checks in a row have found none.

It is a per-clip answer, not a live one: a check is a job that loads the model (about 20 s) before it reads the
frames. Without depth it is a test in the picture only.

**Not yet validated on a hand.** The zone test is covered by unit tests on object rows, and the `hand` prompt found
the players' hands in the stock clip. It has not been run on footage of a hand at the lab's tower, which is where it
is to be validated; a simulated hand is not a test of it (SAM 3 calls a hand built from simple shapes a "robot hand",
not a "hand"). The human-turn script therefore takes its hand-present signal from its own timer.

## Several views into one set of boxes: fuse

One view sees one face of a tower. `fuse` takes runs that looked at the same scene from different sides, each with
the camera's pose in its `camera.json`, and merges the last frame's box poses in the world frame. Two detections are
the same box when their centres are nearer than half the box's smallest side (7 mm for a Jenga block) and their long
axes agree: blocks side by side are 24 mm apart, and a block in the next layer is 14 mm away and turned. A face whose
extents miss the box's sides by more than 12 mm is not used.

A fused box has `position_world_m` (the mean of its detections' centres), `quaternion_world_xyzw` and
`long_axis_world` (from the detection whose face matched best), the `views` and `faces` that saw it, `detections` and
`spread_m` (how far its detections' centres are from their mean). The answer also counts the boxes only one view
saw. A box that no view saw is not in the list: nothing is filled in.

## A second camera and its calibration

Nothing in the tool is specific to one camera. A clip is colour, depth, `fx`, `fy`, `cx`, `cy`, a depth scale and a pose
in the world, and `fuse` merges runs whatever camera made them. The Jenga scripts read further cameras from
[`scripts/jenga_cameras.json`](../scripts/jenga_cameras.json): each entry is that same description plus the image size,
for a camera that stands still, and is **off unless enabled** (`"enabled": true`, or `--enable-camera <name>` for one
run). The file's example is `tripod`, an RGB-D camera on a tripod beside the table at table level that sees the table,
the tower and the robot; its numbers are not a product's. It is not part of the `ur5e-workcell` pack or of
`lab_hardware.json`: in simulation the script draws a camera at that pose on the stage for the run, and with real
hardware a recorder would write the clip with these numbers in its `camera.json`.

What a real second camera needs before its poses can be fused:

- **Intrinsics and depth registration** from the camera's own factory calibration, read through its SDK: `fx`, `fy`,
  `cx`, `cy` of the colour stream, depth registered to colour, and the depth unit. Lens distortion is not modelled here;
  use the SDK's undistorted, registered frames.
- **`world_from_camera`**, six numbers, in the cell's world frame (the frame the robot's base and rail define). `fuse`
  joins two detections when their centres are within 7 mm, so the pose has to be good to about 2 mm and 0.2 degrees at
  the tower (0.2 degrees is 2 mm at 0.6 m).

No fiducial markers are used in this cell. Two ways to get the pose without them, neither implemented yet:

1. **Against the robot.** Hold the gripper still in 15 to 20 poses spread through the space the camera sees, at
   several rail positions. In each, segment the gripper and take one point of it that the model knows: the end face of
   a fingertip, a 20.5 x 11.4 mm rectangle whose centre is known in the tool frame from the gripper's CAD and in the
   world from the arm's joint angles and the rail position. The same face is found in the camera's depth the way a
   block's face is. The pose is the rigid transform that best maps the camera-frame points onto the world points
   (least squares, with the worst pairs left out). It is as good as the arm's and the rail's own accuracy, and it wants
   the points spread wide: points in one small region fix the position there and leave the rotation loose.
2. **Against the table and the tower.** The table top is a plane in the depth image: it fixes the camera's height and
   two of its three rotations. A freshly built tower fixes the rest: its blocks are seen by the wrist camera, whose
   poses are already in the world frame through the arm, and by the new camera; the blocks both saw give dozens of
   point pairs for the same least-squares fit. The tower is small, so use it with the table plane and with a few robot
   points far from it. This needs the wrist camera's own mount (`tool0` to camera) to be right first, and that
   hand-eye calibration is itself still pending in the cell's configuration.

After either, the check is the number the human-turn script reports as `agreement_between_cameras_mm`: how far apart
the two cameras put the same blocks.

## Verified

Local results on the ASU PC (2026-11-15, SAM 3 at the bundle's pin, RTX PRO 6000 Blackwell):

- **Video file.** Two stock clips of people playing Jenga, 1920 x 1080 and 1080 x 1920. On the first (164 frames,
  prompts tower, blocks and hand) the job took 280 s and found 42 blocks in the first frame; its label images are
  identical, pixel for pixel, to those of the stand-alone script this tool's driver was made from. On the second (every
  third frame, 70 frames) it found 53 blocks in the first frame.
- **Recorded RGB-D clip with a camera pose.** The work cell's simulated wrist D415 (1280 x 720), 0.45 m from the face of
  an 18-layer tower of 72 x 24 x 14 mm blocks: all 36 visible blocks found (27 ends, 9 sides), each with a pose, the
  centres a median 0.93 mm (largest 1.19 mm) from the simulator's. The numbers are in the
  [work cell's page](../exts/workcell/workcell/README.md#one-jenga-turn-look-predict-push-look-again).
  The simulated blocks had to be drawn with visible edges first: the pack's block is a plain one-colour box, and SAM 3
  returns a tower of them as one object.
- **Several views and a second camera.** Four wrist views and a fixed camera with other intrinsics and a millimetre
  depth unit, fused: all 53 blocks of a tower one block short, a median 0.85 mm (largest 2.1 mm) from the simulator's,
  with the wrongly read faces of single views left out
  ([the numbers](../exts/workcell/workcell/README.md#a-persons-turn-watch-look-again-from-all-sides-rebuild-the-twin)).
- Host tests (`test_sam3.py`): the checks above, the wrapper around a fake launcher, the zone test, the fusion and the
  pose arithmetic on synthetic faces. They do not run the model.

Not done: a real D415 recording, a hand at the lab's tower, a real second camera, a ROS topic or bag reader, a live
(frame by frame) mode, and tracking that survives an object leaving the picture and coming back.
