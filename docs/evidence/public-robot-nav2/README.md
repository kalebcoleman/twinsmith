# A public robot, /cmd_vel and a Nav2 goal: measurements

What was measured for the [worked example](../../ROS2-RVIZ.md#worked-example-a-public-robot-cmd_vel-and-a-nav2-goal), on one machine: Isaac Sim 6.2.0 and 5.1.0, headless PhysX, ROS 2 Jazzy, Nav2 from RoboStack. Isaac Sim 6.0 and 6.1 were not available here.

## Why the robot did not drive

Measured: `UsdGeom.Cylinder` wheels got no contact from a very flat `Cube` collider. The robot rests on its box colliders, or falls through if it has none low enough, while the wheels spin freely at exactly v/r.

Reproduced on 6.2 with a wrapper USD and a 20 x 20 x 0.1 cube floor. TurtleBot3 sat at `[0.0004, 0.0004, 0.0067]`, or `[0.0005, 0.0001, 0.0044]` for the older asset, with wheels at 6.06 rad/s for 0.2 m/s (0.2 / 0.033). Carter fell through the floor, to z = -26 after 3 s.

| Change, everything else the same | TurtleBot3 result |
|---|---|
| Plane or mesh floor | drives at 0.196 m/s, wheels 5.88 rad/s |
| Cube floor 1 m thick, or 2 m | drives |
| `/physics/collisionApproximateCylinders=true`, thin floor | drives |
| A Quick Start scene or a wrapper USD on `sim__stage__open`, plane or thick floor | both drive, so the wrapper is not the cause |

The table of floor sizes is in the [guide](../../ROS2-RVIZ.md#wheels-spin-but-the-robot-stays-put). 5.1 gave the same result for the thin floor, the convex setting and the thick floor. `sim__prim__create` now warns, and the guide gives the fix. A saved file's floor gets no warning.

Not the cause, from each robot's USD: kinematic flags (all false), fixed roots (no joint with an empty side), wheels without colliders (all have them). The physics variant is `physx` by default, and the 6.1 and 6.2 TurtleBot3 layers are byte-identical.

## TF loop

Measured with NVIDIA's older TurtleBot3 (`base_footprint` is a body fixed to `base_link`) on 6.2. PhysX rooted the articulation at `base_link`, and with the articulation as its target the TF node publishes parent -> root link and then the articulation's tree. With `base_link=base_footprint`, without static-edge handling for links fixed to the base:

```text
base_footprint -> base_link
base_link -> base_footprint
base_link -> base_scan, caster_back_link, wheel_left_link, wheel_right_link
```

There was no `odom -> base_footprint` at all, and Kit logged `received duplicate child frame 'base_footprint' from multiple contributors`. With the same graph built by hand, the doctor says:

```text
ERROR: the Fixed Frame 'odom' is not in the tree; roots are: none, every frame has a parent, so there is a TF loop.
ERROR: TF loop base_footprint -> base_link -> base_footprint: a frame is its own ancestor, so nothing in it can be placed.
```

and `view_frames` wrote an empty graph. On 6.1 the PhysX root may be `base_footprint` instead, a difference from 6.2 that was not checked.

With that handling, on the same robot, with either base:

```text
odom -> base_footprint        (live)
base_footprint -> base_link, base_scan, caster_back_link   (static, one message each)
base_footprint -> wheel_left_link, wheel_right_link        (live)
OK: every frame connects to 'odom'.
```

Kit logged no duplicate from the tool's own graph. A listed link that is the articulation's root still brings its whole tree, which is why links fixed to the base are published as static edges instead.

## LiDAR

`-1.0` means a beam with no return. On 6.2, a helper wired to the tool's own product, and one wired through `IsaacCreateRenderProduct`, both published full scans before any `sim__lidar__read`, with `Example_Rotary_2D`, `RPLIDAR_S2E` and the generic LiDAR. A scan of 360 values of `-1.0` until a read did not reproduce here.

| Wiring on 6.2 | Example_Rotary_2D | RPLIDAR_S2E |
|---|---|---|
| Tool's product | 3600 beams, none `-1.0` | 3200 beams, 2 `-1.0` |
| `IsaacCreateRenderProduct`, `cameraPrim` = requested path | 3600 beams | **0 messages**, `not attached to RTX Lidar`: the OmniLidar is nested below that path |
| `IsaacCreateRenderProduct`, `cameraPrim` = `sensor_path` | 3600 beams | 3200 beams, 2 `-1.0` |

The last row is what `sim__lidar__create` now returns as `ros2_scan_graph`, because saves drop the tool's own product.

On 5.1, `/scan` kept counting across Stop and ran 206 s ahead of `/clock`, so the recipe sets `resetSimulationTimeOnStop`, which is the default on 6.2.

## Reopening a saved scene

Saved with `sim__stage__save`, then opened with `sim__stage__open`, on 6.2 and 5.1, with the recipe's graphs: `/scan` published without adopting, the links came back at their saved poses, and the robot drove (TurtleBot3 on both builds, and Create 3 on 6.2). A save paused after driving kept the pose. Create 3 on the doc-text runs also published `/scan` after reopening on both builds.
A helper wired to the tool's own product can't do this, so its `/scan` can stay silent after a reopen.

"no usable time-code range" is NVIDIA's warning for a root layer with no `startTimeCode` and `endTimeCode`, as in a hand-written wrapper. A stage made by Quick Start has them. The warning says playback loops over one frame, but on 6.2 driving and `/scan` both still worked with it. `sim__stage__open` now reports it. The report's collapsed robot body did not reproduce on either build, so its cause is unknown.

## Turning in place, in real time on 5.1

| Robot | 0.5 rad/s command | 1.0 rad/s command |
|---|---|---|
| `create3` | 0.43 | 0.90 |
| `carter_v1` | 0.43 | 0.90 |
| `nova_carter` | 0.47 | 0.96 |
| `turtlebot3_burger` | 0.28 | 0.77 |

`jetbot` measured 0.52 at 0.5; its 1.0 reading wrapped past ±π and is not usable. Jetbot's wheels also ran about 6% faster than v/r (7.06 to 7.26 rad/s for 6.67), and it went 0.21 m/s for 0.2.

## Pictures

| File | Shows |
|---|---|
| [room-start-6.2.png](room-start-6.2.png) | From above on 6.2: the walls, the box and Create 3 at the start. The grid and the red and green lines are Kit's viewport overlays, not scene geometry, and they do not line up with the world origin |
| [goal-6.2.png](goal-6.2.png) | Same camera at `SUCCEEDED`: Create 3 is behind the box, at about (2.3, -0.05) |
| [goal-5.1.png](goal-5.1.png) | Same on 5.1, with no overlays |
| [room-map-6.2.png](room-map-6.2.png) | `sim__occupancy_map`'s map, 4x enlarged: walls and box black, and the robot's start cleared |
| [tf-tree-create3-6.2.png](tf-tree-create3-6.2.png) | `view_frames` during Nav2: `map -> odom -> base_link -> wheels` and the LiDAR frame, with one parent each |
| [tf-tree-turtlebot-base-footprint-6.2.png](tf-tree-turtlebot-base-footprint-6.2.png) | `view_frames` for the case that looped, after the fix |

RViz was not opened: these runs were headless, on a shared desktop.

## To run on 6.0 and 6.1

Needs ROS 2 with Nav2 and this repository. Set `<interp>` to the build's Python. Stop your other Kits first.

1. Start: `TWINSMITH_SIM_PYTHON=<interp> OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=42 ./run.sh --headless --realtime`.
2. Follow the [worked example](../../ROS2-RVIZ.md#worked-example-a-public-robot-cmd_vel-and-a-nav2-goal) as written, top to bottom, and note anything you had to change. Record the last `sim__body__state`, `Goal finished with status`, and both doctor outputs.
3. TF loop, the one open question: the 6.1 TurtleBot3 has `base_footprint` as an Xform, so use the older file. Write a wrapper USD that references `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd` under `/World/r`, and `sim__stage__open` it. Add a floor with `sim__prim__create` `/World/Floor`, `kind=Cube`, `scale=[20,20,1]`, `position=[0,0,-0.5]`, `collision=true`. Run `sim__ros2_tf` with `robot=/World/r`, `base=mobile`, `world_frame=""`, once with default `base_link` and once with `base_link=/World/r/base_footprint`. Each time, Play for a few seconds and record `articulation_root`, `base_link`, `ros2 run tf2_tools view_frames` and the doctor. Expect no `TF loop`, and no `duplicate child frame` in `sim__logs__tail`. Also record `assets[0].location` from any `sim__quick_start` result: it shows which assets root that install uses.
3b. TurtleBot3 by name: call `sim__quick_start` with `robot=turtlebot3_burger` and repeat the example with `--yaw-tolerance 3.2`, the LiDAR under `base_scan` as the guide's table says, and `base_link=base_link`.
4. Thin floor: `sim__stage__open` a wrapper USD for TurtleBot3, because Quick Start's own ground would hold the robot up. `sim__prim__create` `/World/Floor` with `scale=[20,20,0.1]`, `position=[0,0,-0.05]`, `collision=true`. Graph from `robot.cmd_vel_graph`, Play, `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`. Read `sim__joints__state` and `sim__body__state`. Expect wheels at exactly v/r and no movement. Then use `scale=[20,20,1]`, `position=[0,0,-0.5]`, and expect it to drive.
5. LiDAR: before `ros2_scan_graph`, use a graph with `ROS2RtxLidarHelper` whose `renderProductPath` is the returned `render_product`, and read `/scan` before any `sim__lidar__read`. Report whether it is all `-1.0`. If `ros2_scan_graph` itself refuses `resetSimulationTimeOnStop` or any other port on this build, report the message.
6. Save and reopen: after step 2's save, `sim__stage__open` the file, Play, and check `/scan` and `sim__body__state`. If the body looks collapsed, capture a picture with `sim__camera__capture` and record how the file was saved and whether the log says `no usable time-code range`.
