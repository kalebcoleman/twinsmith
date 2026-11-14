# ROS 2 TF and RViz 2 with Isaac Sim

Use this guide to get a correct TF tree and a working RViz 2 view of an Isaac Sim robot,
the same way on any machine. Labs written for Isaac Sim 6.0 should also read the
[Isaac Sim 6.2 notes](COMPATIBILITY.md#isaac-sim-62-notes-for-60-tutorials) (node and API differences, verified live). It covers three tools:

| Step | Tool | Runs where |
|---|---|---|
| 1. Publish `/clock`, `/joint_states` and the robot's TF | `sim__ros2_tf` (MCP) | inside Isaac Sim |
| 2. Write an RViz config with the right Fixed Frame | `python3 scripts/ros2_rviz_config.py` | anywhere (stdlib only) |
| 3. Check the live TF tree before trusting RViz | `python3 scripts/ros2_tf_doctor.py` | with ROS 2, or `--isaac-ros` |

The graph layout follows NVIDIA's own helpers that ship with Isaac Sim 6.2
(`isaacsim.ros2.nodes`, `impl/graph_api.py`: clock, TF and odometry graphs) and the
[Transform Trees and Odometry tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_tf.html),
and the RViz layout follows NVIDIA's `isaac_tutorials` configs in
[IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces). Frame names
follow [REP 105](https://www.ros.org/reps/rep-0105.html) (`map`, `odom`, `base_link`).

## Worked example: a public robot, /cmd_vel and a Nav2 goal

From nothing to a Nav2 goal reached, using only MCP tools and shell commands, with no `run_python`. Run it top to bottom. The robot is NVIDIA's
iRobot Create 3, in a room you build with tools. See the [results for each robot and build](#what-was-measured).

You need Isaac Sim, this repository's MCP server, and ROS 2 with Nav2 (`ros-<distro>-navigation2` and `ros-<distro>-nav2-bringup`, or a RoboStack environment).

**Terminal 1, the simulator.** `--realtime` caps it at 60 frames per second: uncapped, headless Kit ran about twice real time on 5.1, and Nav2 kept missing its control rate.

```bash
export ROS_DOMAIN_ID=42
./run.sh --headless --realtime
```

**1. Build the scene.** In your MCP client, call each tool below. `sim__quick_start` needs no IDs. Every later call also needs the `session_id` and `stage_id` it returns: add them to each block below. `sim__quick_start` and `sim__stage(action="info")` refuse them.

Tool `sim__quick_start`:

```json
{"robot": "create3"}
```

Keep the result: `robot.cmd_vel_graph` is used in step 3, and `assets` shows which file loaded. It came from this simulator's own assets root, so nothing is downloaded into the repository. If you already have a scene open, the call says how to replace it.
Names: `franka`, `turtlebot3_burger`, `carter_v1`, `nova_carter`, `jetbot`, `create3`, `ur5e`.

Four walls and a box to drive around, each with one `sim__prim(action="create")` call. Don't build a floor: Quick Start's ground is the floor, and [a flat box floor can stop a robot driving](#wheels-spin-but-the-robot-stays-put).

Tool `sim__prim`:

```json
{"action": "create", "path": "/World/WallEast", "kind": "Cube", "collision": true, "position": [3, 0, 0.5], "scale": [0.2, 6.2, 1]}
```

Tool `sim__prim`:

```json
{"action": "create", "path": "/World/WallWest", "kind": "Cube", "collision": true, "position": [-3, 0, 0.5], "scale": [0.2, 6.2, 1]}
```

Tool `sim__prim`:

```json
{"action": "create", "path": "/World/WallNorth", "kind": "Cube", "collision": true, "position": [0, 3, 0.5], "scale": [6.2, 0.2, 1]}
```

Tool `sim__prim`:

```json
{"action": "create", "path": "/World/WallSouth", "kind": "Cube", "collision": true, "position": [0, -3, 0.5], "scale": [6.2, 0.2, 1]}
```

Tool `sim__prim`:

```json
{"action": "create", "path": "/World/Box", "kind": "Cube", "collision": true, "position": [1.1, 0, 0.5], "scale": [0.8, 1.4, 1]}
```

A camera above the room, so you can look at the result:

Tool `sim__camera`:

```json
{"action": "create", "path": "/World/TopCamera", "position": [0, -0.5, 9], "target": [0, 0, 0], "frame": "world", "focal_length": 12}
```

**2. Add a 2D LiDAR.** `RPLIDAR_S2E` sees from 0.05 m, while `Example_Rotary_2D` is blind within 1 m, so a costmap would miss anything close to the robot.

Tool `sim__lidar`:

```json
{"action": "create", "path": "/World/Create3/base_link/Lidar2D", "config": "RPLIDAR_S2E", "position": [0, 0, 0.12]}
```

Keep the result: `sensor_path` is the OmniLidar inside that asset, and `ros2_scan_graph` is used in step 3.

**3. Connect ROS 2.** TF, odometry, `/clock` and `/joint_states`. `world_frame` is empty because AMCL publishes `map -> odom`. `extra_frames` puts the LiDAR's frame in the tree: use `sensor_path` from step 2.

Tool `sim__ros2_tf`:

```json
{"robot": "/World/Create3", "base": "mobile", "world_frame": "", "base_link": "base_link", "extra_frames": ["/World/Create3/base_link/Lidar2D/RPLidar_S2E"]}
```

Then call `sim__graph(action="build")` twice: once with every field of `robot.cmd_vel_graph` from step 1, and once with every field of `ros2_scan_graph` from step 2.

**4. Make the map.** This plays a few frames, then stops, so the robot returns to its start. `clear` removes the robot's own footprint, so use at least the robot's radius.

Tool `sim__occupancy_map`:

```json
{"min": [-3.5, -3.5], "max": [3.5, 3.5], "clear": [[0, 0, 0.3]], "start": [-1, -1], "basename": "room"}
```

The result's `yaml` is the map file for step 7. If `room` already exists, the call refuses: choose another `basename`.

**5. Play, and check the robot from ROS 2.**

Tool `sim__timeline`:

```json
{"action": "play"}
```

Look at the room from above:

Tool `sim__camera`:

```json
{"action": "capture", "path": "/World/TopCamera", "filename": "start.png"}
```

**Terminal 2, ROS 2.** Stay in the repository folder. Use the same domain, and add `--overlay <Nav2 prefix>` if Nav2 is installed separately.

```bash
export ROS_DOMAIN_ID=42
source scripts/ros_env.sh
ros2 topic list | grep -E 'clock|cmd_vel|odom|scan|^/tf'
python3 scripts/ros2_tf_doctor.py --fixed-frame odom
```

The doctor should end with `OK: every frame connects to 'odom'`. Drive forward at 0.15 m/s, turn on the spot, and read where the robot thinks it is. The first second or so can be lost while the publisher connects, so expect 0.3 to 0.6 m (the box in this scene stops a Create 3 at about 0.52 m). These times assume real time: `--realtime` is only a cap, and on a loaded machine we measured 0.08 to 0.4 of it. Compare `/clock` with the wall clock, and allow Nav2 longer if it's slow:

```bash
python3 scripts/cmd_vel_sequence.py "0.15,0,5;0,0.5,3;0,0,1"
timeout -s KILL 10 ros2 topic echo --once /odom --field pose.pose.position
```

The robot prim's own position agrees with `/odom`; `sim__physics(action="body_state")` with `path` `/World/Create3/base_link` reads it.

**6. Back to the start.** Stopping returns the robot to its start pose. Nav2 will assume it starts there.

Tool `sim__timeline`:

```json
{"action": "stop"}
```

Tool `sim__timeline`:

```json
{"action": "play"}
```

**7. Start Nav2**, in Terminal 2. Replace the map path with `yaml` from step 4. `scripts/nav2_params.py` writes a parameter file from your own Nav2's defaults, with this robot's base frame and start pose. `setsid nohup` keeps Nav2 running in the background, and its log goes to `/tmp/nav2.log`.

```bash
python3 scripts/nav2_params.py --out /tmp/isaac_nav2.yaml
MAP=<yaml from step 4>
setsid nohup ros2 launch nav2_bringup bringup_launch.py map:=$MAP params_file:=/tmp/isaac_nav2.yaml use_sim_time:=true autostart:=true use_composition:=False > /tmp/nav2.log 2>&1 < /dev/null &
```

Wait until the log says `Managed nodes are active`, about 20 s, twice (localization and navigation). Then check the whole tree, now from `map`:

```bash
python3 scripts/ros2_tf_doctor.py --fixed-frame map
```

**8. Send a goal** behind the box. The robot can't drive straight there, so it has to plan around it.

```bash
timeout -s KILL 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.2, y: 0.0}, orientation: {w: 1.0}}}}"
```

It prints `Goal finished with status: SUCCEEDED`. Check from the simulator, and look:

Tool `sim__physics`:

```json
{"action": "body_state", "path": "/World/Create3/base_link"}
```

Tool `sim__camera`:

```json
{"action": "capture", "path": "/World/TopCamera", "filename": "goal.png"}
```

**9. Clean up and save.** `pkill -INT -f '[p]arams_file:=/tmp/isaac_nav2.yaml'` (use the path you gave `nav2_params.py`) stops this Nav2 and not anyone else's; the brackets stop it matching your own shell. If it keeps running (a launch started with `setsid nohup` from a script ignored SIGINT in one run), `kill -TERM` the launch's PID, then its direct children (`pgrep -P <pid>`). A launch backgrounded by a script can ignore SIGINT, and SIGTERM on the launch alone can leave its nodes running, so confirm with `ros2 node list` and stop any left by PID. Then stop the timeline and save a copy. The flag removes this session's render handles from the copy; without it, the save refuses and says so. In the saved file, the drive and LiDAR graphs make their own render products at Play, so `/scan` comes back when you open it with `sim__stage(action="open")`.

Tool `sim__timeline`:

```json
{"action": "stop"}
```

Tool `sim__stage`:

```json
{"action": "save", "filename": "nav2_room", "reset_camera_render_handles": true}
```

### Other robots

Use the robot's `wheels` values from the Quick Start result. The base link and the LiDAR's parent differ per robot: find a link with `sim__prim(action="find")`.

| Robot | LiDAR parent | `base_link` | `nav2_params.py` flags |
|---|---|---|---|
| `create3` | `base_link`, 0.12 m up | `base_link` | none |
| `turtlebot3_burger` | `base_scan`, at its origin | `base_link` | `--yaw-tolerance 3.2` (see below) |
| `carter_v1`, `nova_carter` | `chassis_link`, **0.6 m up** | `chassis_link` | `--base-frame chassis_link --radius 0.5` (Carter's farthest body corner measured 0.48 m, so 0.45 leaves its rear corners outside), and a `clear` radius of 0.7 in step 4. Both reached the goal on 6.2 (0.2 to 0.26 m away) |
| `jetbot` | `chassis`, 0.12 m up | `chassis` | `--base-frame chassis --radius 0.12`, `clear` 0.15. Reached the goal on 6.2 (0.23 m) |

Mount the LiDAR clear of the robot's own body: 0.12 m is Create 3's height. Inside Carter's body `/scan` has a publisher and never a message; at 0.3 m it sees the chassis as a ring at 0.45 m; at 0.6 m it's clean. If `/scan` is silent, raise the sensor first. Send the first goal only after `/amcl_pose` is publishing: a goal sent as soon as the action server exists can abort short.

TurtleBot3's parent path is `/World/TurtleBot3/Geometry/base_footprint/base_link/base_scan` on 6.1 and 6.2, and `/World/TurtleBot3/base_scan` on 5.1 and 6.0, because NVIDIA restructured the asset.

TurtleBot3 Burger's rear caster is a box collider, and turning in place it reached 0.28 rad/s for a 0.5 rad/s command on 5.1, against 0.43 for Create 3; the caster dragging is the likely reason. In real time on 5.1 and 6.2, Nav2 reached the position but stalled on the final heading (`Failed to make progress`). `--yaw-tolerance 3.2` ignores the heading, and the goal then succeeds.

### What was measured

Each run used `--realtime`, the room above and a goal at (2.2, 0), with ROS 2 Jazzy, and PhysX. "Doc text" means a script ran this page's own blocks in order. Distance is the robot's real position from `sim__physics(action="body_state")` when Nav2 finished; Nav2's default tolerance is 0.25 m.

| Build | Robot | `/cmd_vel` (commanded, measured) | Nav2 goal |
|---|---|---|---|
| 6.2.0 | `create3` (doc text) | 0.2 m/s, 0.205 m/s | `SUCCEEDED` (0.105 m from the goal) |
| 5.1.0 | `create3` (doc text) | 0.2 m/s, 0.205 m/s | `SUCCEEDED` (0.07 m) |
| 6.2.0 | `turtlebot3_burger`, `--yaw-tolerance 3.2` | 0.2 m/s, 0.196 m/s | `SUCCEEDED` (0.235 m) |
| 5.1.0 | `turtlebot3_burger`, `--yaw-tolerance 3.2` | 0.2 m/s, 0.19 m/s | `SUCCEEDED` (0.224 m) |
| 6.2.0 | `carter_v1`, radius 0.45 | 0.5 m/s, 0.5 m/s | not reached. With `clear` 0.3 the map held its own body and planning failed. With 0.7 it planned around the box, then stopped 1.2 m short with `Failed to make progress`, on Nav2's default controller settings. Not tuned further |
| 6.2.0 | `carter_v1`, radius 0.5, in `sim__quick_start environment=warehouse`, Nav2 Jazzy 1.3.13 (MPPI, default tuning) | not measured | 9 of 9 goals reached between two stations 20 m apart through the cross aisle, one session, 0.07 to 0.23 m from the goal (Nav2's default tolerance is 0.25 m), one recovery. It reverses when the goal is behind it, so return legs took twice as long |
| 6.2.0, 5.1.0 | `carter_v1`, `nova_carter`, `jetbot` | 0.5, 0.5, 0.2 m/s; measured 0.5, 0.5, 0.21 m/s | not run, except Carter above |

Without `--realtime`, TurtleBot3 reached `SUCCEEDED` on 6.2 and `ABORTED` on 5.1 at the goal position. `franka` loads on both builds; it was not driven. Isaac Sim 6.0 and 6.1 weren't available on the machine used: NVIDIA's files for these names exist on both, and TurtleBot3's 6.1 files are identical to 6.2's, but the runs are still to do. RViz was not opened in these runs.

Details and pictures: [the evidence record](evidence/public-robot-nav2/README.md).


### Wheels spin but the robot stays put

The wheel joints turn at exactly v/r, and the chassis stays within a millimetre of its start. Most robots' wheels are `Cylinder` colliders, and they got **no contact** from a very flat `Cube` collider, so the robot rests on its other colliders or falls through the floor.
Measured with a cube floor on 6.2, with its top at z=0. The same 20 x 20 x 0.1 and 20 x 20 x 1 floors gave the same results on 5.1:

| Floor (x, y, thickness, m) | Longest/shortest | TurtleBot3 | Carter v1 |
|---|---|---|---|
| 20, 20, 0.1 | 200:1 | no drive: wheels 6.06 rad/s, moved 0 m | fell through the floor |
| 10, 10, 0.05 | 200:1 | no drive | not tested |
| 40, 40, 0.3 | 133:1 | no drive | not tested |
| 20, 20, 0.15 | 133:1 | no drive | not tested |
| 100, 100, 1 | 100:1 | drove | not tested |
| 20, 20, 0.2 | 100:1 | drove | erratic start |
| 5, 5, 0.1 | 50:1 | drove | erratic, thinner than its wheels |
| 20, 20, 0.5 and 20, 20, 1 | 40:1, 20:1 | drove | drove |
| Quick Start's ground (a plane), or a mesh | none | drove | drove |
| 20, 20, 0.1, launched with `--/physics/collisionApproximateCylinders=true` | 200:1 | drove, 0.178 m/s | drove unevenly: started 0.9 m off, wheels 1.99 and 2.34 rad/s |

- Use `sim__quick_start`'s ground. A thicker box, thicker than the wheels, also works; no safe ratio is claimed.
- `sim__prim(action="create")` warns about a box collider at 50:1 or more.
- In the run that found this, friction 1.5, mass on the root, a lower floor and position targets didn't change it. Kinematic flags and fixed roots aren't involved: the NVIDIA robots tested have neither.
- A course's own TurtleBot file that did drive wasn't available for comparison, so it isn't known why it drove.

## Installing ROS 2 (for RViz 2 and the doctor)

Isaac Sim ships its own ROS 2 libraries for the bridge, but RViz 2 needs a ROS 2 installation.

- **With sudo:** the official packages for your Ubuntu release (Humble on 22.04, Jazzy on 24.04):
  `sudo apt install ros-<distro>-ros-base ros-<distro>-rviz2 ros-<distro>-robot-state-publisher
  ros-<distro>-tf2-tools ros-<distro>-rmw-cyclonedds-cpp ros-<distro>-xacro`, then
  `source /opt/ros/<distro>/setup.bash`.
- **Without sudo** (lab and university accounts): [RoboStack](https://robostack.github.io/)
  installs ROS 2 into your home folder with micromamba:

  Pick the distro from the OS first, because it must match the simulator's bundled bridge (see the
  last item): **Jazzy on Ubuntu 24.04, Humble on Ubuntu 22.04.**

  ```bash
  . /etc/os-release
  case "$VERSION_ID" in 22.04) DISTRO=humble ;; *) DISTRO=jazzy ;; esac   # what run.sh picks
  mkdir -p ~/.local && curl -fsSL https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C ~/.local bin/micromamba
  ~/.local/bin/micromamba create -y -p ~/ros2/$DISTRO -c conda-forge -c robostack-$DISTRO \
      ros-$DISTRO-desktop ros-$DISTRO-rmw-cyclonedds-cpp ros-$DISTRO-tf2-tools ros-$DISTRO-xacro
  ~/.local/bin/micromamba run -p ~/ros2/$DISTRO rviz2
  # or, per shell: source scripts/ros_env.sh ~/ros2/$DISTRO   (activates it, then config/ros_dds.env)
  ```

  About 5.5 GB. Validated with Humble on Ubuntu 22.04 with Isaac Sim 6.2: the doctor and RViz 2 see
  the simulator's TF with `config/ros_dds.env` sourced. (Older Humble environments were created from
  the `robostack-staging` channel; they keep working.) On 24.04 with sudo, apt's
  `/opt/ros/jazzy` is the simpler choice.
- **Only the doctor:** `--isaac-ros` uses Isaac Sim's bundled ROS 2; no install needed.
- **Use the same distro as the simulator's bridge.** `run.sh` loads Isaac's bundled Humble on
  Ubuntu 22.04 and Jazzy on 24.04 (`ROS_DISTRO` overrides it). Humble and Jazzy nodes did not
  discover each other with `ROS_LOCALHOST_ONLY=1` in local tests, so install the matching distro
  (or point `ROS_DISTRO` at the one you have) rather than mixing them.

## The rules that keep RViz stable

Every "the map is wrong", "frames jump" or "RobotModel is white" problem we have seen comes from one of these:

1. **One publisher per frame.** Each frame has exactly one parent, from exactly one node.
   Isaac publishing the robot's links *and* `robot_state_publisher` publishing them from the
   URDF gives every link two sources, so RViz jumps between them. Pick one
   (`link_frames=isaac` or `link_frames=robot_state_publisher`).
2. **One clock.** Isaac stamps everything with simulation time and publishes `/clock`, from
   exactly one node. Two `/clock` publishers (for example two graphs that each
   publish one) disagree by a few frames, so time keeps jumping back: RViz logs "Detected
   jump back in time", resets and clears TF many times a second. `sim__ros2_tf` reuses
   an existing clock (`clock=auto`), or else publishes one shared clock
   (`/Graph/twinsmith_ros2_clock`) for every robot it sets up, and matches its TF stamps to that
   clock's stop/play behaviour. If another graph adds a second
   clock afterwards, call the tool again with `clock=auto`; it drops its clock and updates every robot it
   set up. The shared clock stays after the last robot graph is deleted (`sim__stage(action="clear_owned")`
   removes it). Every other node, RViz included, must run with
   `use_sim_time:=true`; mixing wall-clock and sim stamps makes TF lookups fail or extrapolate.
3. **One root.** RViz's *Fixed Frame* must be the root of a single connected tree
   (`world` for an arm on a table, `map` or `odom` for a mobile base). A frame that does not
   connect to it is drawn nowhere.
4. **`map -> odom` belongs to localization.** If AMCL or SLAM runs, it publishes `map -> odom`;
   then Isaac must publish only `odom -> base_link` (`world_frame=""`). A namespaced mobile base
   uses `<ns>_odom` (`a/b` gives `a__b_odom`; namespace segments may not start or end with `_`);
   set the localization node's `odom_frame_id` to it.
5. **Same DDS settings on both sides.** `run.sh` uses `config/ros_dds.env` (Cyclone DDS,
   localhost only, `ROS_DOMAIN_ID`). Source the same file in the RViz/ROS terminal, or nothing
   is discovered.

## 1. Publish the robot from Isaac Sim

Stop the timeline, then call the tool once per robot. Calling it again rebuilds that robot's
graph (`/Graph/twinsmith_ros2`, or `/Graph/twinsmith_ros2_<namespace>_<hash>`), so nothing is ever
published twice by it. The static `world -> base_link` (or `world -> <odom>`) is the pose at setup
time: call it again after moving the robot.

Tool `sim__ros2_tf`, with the IDs from `sim__stage(action="info")`:

```json
{"session_id": "<from sim__stage info>", "stage_id": "<from sim__stage info>", "robot": "/World/UR5e"}
```

| Situation | Arguments |
|---|---|
| Arm on a table, RViz shows TF frames | defaults: `base=fixed`, `world_frame=world`, `link_frames=isaac` |
| You want the URDF RobotModel in RViz | `link_frames=robot_state_publisher`, then run `robot_state_publisher` with the URDF and `use_sim_time:=true` |
| Mobile base, no localization | `base=mobile`, `base_link=<chassis prim>`, `world_frame=map` |
| Mobile base with AMCL/SLAM | `base=mobile`, `base_link=<chassis prim>`, `world_frame=""`, as in the [worked example](#worked-example-a-public-robot-cmd_vel-and-a-nav2-goal) |
| Cameras, LiDARs, `tool0` | `extra_frames=["/World/.../tool0", "/World/camera"]` (published relative to the base link) |
| Several robots | one call per robot, each with its own `namespace` (its own graph; `/joint_states` and `/odom` are namespaced, TF stays on `/tf`). Their link names must differ: set `isaac:nameOverride`. A namespaced mobile base gets its own odometry frame `<ns>_odom`. Each graph also publishes `/clock` (same sim time) |
| A gripper that is its own articulation on an arm | one call for the arm, then one for the gripper: `robot=/World/RG2_Gripper`, `base_link=/World/RG2_Gripper/rg2_base_link`, `namespace=rg2`, `attach_to=/World/UR5e/wrist_3_link` |

A gripper joined to an arm by a fixed joint with `excludeFromArticulation` is a separate articulation, so neither one's
link TF covers the joint, and the doctor reports its frames as not connected to `world`. A static transform would be wrong
once the arm moves. `attach_to` publishes `wrist_3_link -> rg2_base_link` on every playback tick from the two live poses,
in place of the world link. The parent must be a rigid body outside this robot, and the arm needs its own call so its
link connects to `world`. Do not also give the gripper a world link. Checked on 6.2 with Quick Start's UR5e and RG2: the
doctor found 17 frames all connected to `world`, and with the arm moving the gripper followed it about 1 m while the edge
stayed about constant.

One namespace holds one robot, because each namespace has one graph. Setting up a second robot in a namespace already used
refuses with `ROBOT_GRAPH_CONFLICT`, names the robot there, and gives a free namespace in `retry_with`, so the first robot's
joint states and frames stay. Repeating setup on the same robot rebuilds its graph as before. This also holds for a graph saved
in a file and reopened.

The tool refuses with `JOINT_STATE_LOOP` when another graph subscribes to the joint state topic it would publish. Where
another graph mirrors a real robot from `/joint_states`, publish the simulated state elsewhere:
`joint_states_topic=isaac/joint_states`. Without this, the arm chased its own state and oscillated at about 3.14 rad/s once
playing (checked on 6.2). The check sees graphs that exist at setup time: adding such a graph after this tool
creates the same loop, so call this tool again afterwards, which refuses until you pick another topic.

The tool finds the articulation root itself (often a fixed `root_joint` below the robot prim).
A mobile base publishes each link once: links fixed to the base as static edges, and moving links relative to the base, so the tree has
no loop whichever link you pick and whichever link PhysX uses as its root. Before this, a `base_footprint` fixed to `base_link` published both directions,
and Kit logged `received duplicate child frame` and dropped `odom -> base`. Check with the doctor, which reports a `TF loop`.
It refuses a `base_link` that is not one of the robot's links, a `base_frame` that differs from
the name Isaac publishes for it when Isaac publishes the links or extra frames (that would split
the tree; with `robot_state_publisher` and no extra frames it should be the URDF root), and links or extra frames whose TF
names collide (TF name = `isaac:nameOverride`, else the prim name). It returns the frame tree, the
topics, the ROS domain and the Fixed Frame to use (`rviz_fixed_frame`). Then play the timeline:
it publishes on every playback tick.

## 2. Write the RViz config

```bash
python3 scripts/ros2_rviz_config.py --fixed-frame world --out ~/lab.rviz \
    --image /camera/rgb --scan /scan --cloud /point_cloud --odom /odom
#   add --robot-model when robot_state_publisher provides /robot_description
export ROS_DOMAIN_ID=<the simulator's domain>
source scripts/ros_env.sh                      # ROS 2, then config/ros_dds.env, then daemon stop
setsid nohup rviz2 -d ~/lab.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 < /dev/null &
echo $! > /tmp/rviz.pid                         # stop it later with: kill "$(cat /tmp/rviz.pid)"
```

`setsid nohup` keeps RViz running after the command that started it ends (agent harnesses kill
the process group of each tool call). View and window options:

| Option | Effect |
|---|---|
| `--view orbit` (default) / `--view topdown` | 3D orbit camera, or a 2D `TopDownOrtho` view like a map |
| `--distance M` | orbit camera distance (default 3 m) |
| `--scale PX` | top-down zoom in pixels per metre (default 100) |
| `--target-frame base_link` | the view follows that frame (default: the Fixed Frame, a still view) |
| `--hide-panels` | leaves the Displays and Views panels out: the 3D view (and any image docks) only. Add panels back from RViz's Panels menu |
| `--width` / `--height` | window size in pixels (default 1400 x 900) |

Checked in rviz2 (Humble, RoboStack) with window captures: `Hide Left Dock: true` and per-panel
`collapsed: true` in a config file have no effect when RViz2 loads it, so `--hide-panels` omits the
panels instead. An Image display opens its own dock about 320 px wide; RViz2 has no per-display
size setting (the dock layout is a binary `QMainWindow State` that RViz writes itself), so drag
the dock edge and use File > Save Config to keep a larger image.

## Agent shells: environment and driving

**Keep the ROS environment out of the MCP client's shell.** After `source scripts/ros_env.sh` (especially with a RoboStack overlay), do not start the MCP server,
`call.py` or any `uv run` / `uv tool` program from that same shell: they pick up the overlay's Python libraries and fail on import, for example
`ImportError: libssl.so.3: version 'OPENSSL_3.2.0' not found` from `cryptography` (reproduced with a Jazzy + Nav2 RoboStack environment). Run the
simulator and the MCP client from one terminal, and ROS 2 commands from another. A script that does both must start the client with the
environment before sourcing it.

`scripts/ros_env.sh` must be **sourced**, and not in a pipeline: `source scripts/ros_env.sh ... | tail`
runs it in a subshell, so the environment is silently lost (redirect to a file instead). It sources a ROS 2 install (a path, or auto-detected:
`ROS_DISTRO` if exported, else Humble on 22.04 and Jazzy on 24.04, in `/opt/ros/<distro>` and
`~/ros2/<distro>`), any `--overlay` on top of it, then `config/ros_dds.env`, then runs
`ros2 daemon stop`, and prints what it chose:

```bash
export ROS_DOMAIN_ID=63                      # a prefix assignment (ROS_DOMAIN_ID=63 source ...) does not last
source scripts/ros_env.sh ~/ros2/humble --overlay ~/ros2/humble_nav2   # RoboStack + a Nav2 env
source scripts/ros_env.sh /opt/ros/jazzy --overlay ~/ros2/jazzy_nav2/install
source scripts/ros_env.sh --dry-run          # print the choice only
```

RoboStack prefixes are activated with micromamba (stacked for an overlay, with the overlay first on
`AMENT_PREFIX_PATH`). Prefer this to `micromamba run` for `ros2` commands: `micromamba run` takes a lock
on the prefix, which breaks concurrent `ros2` calls. The daemon matters: one started earlier with other DDS settings answers
`ros2 topic list` and `ros2 node list` with nothing while data flows. `--keep-daemon` skips the
stop; `ros2 topic list --no-daemon` avoids the daemon (`ros2 topic hz` has no such flag). Kit's
bridge nodes do not appear in `ros2 node list` at all; check topics and `/tf` instead.

`ros2 topic echo` truncates long arrays, so a LaserScan can look all invalid: judge `/scan` with a small
`rclpy` subscriber that prints the count and the range. `--field ranges` works too, but prints every value on one line. With nothing in range a LiDAR publishes no messages at all, and just after Play a scan can be all `-1.0`, so put something in range and wait before judging it.

Without a TTY there is no keyboard teleop. Drive with timed steps (linear m/s, angular rad/s,
seconds):

```bash
python3 scripts/cmd_vel_sequence.py "0.15,0,5;0,0.5,3;0,0,1"         # --topic, --rate, --stamped
python3 scripts/cmd_vel_sequence.py "0.15,0,5" --dry-run              # print the plan only
```

It always publishes a zero Twist at the end, on errors, and on Ctrl-C, SIGTERM or SIGHUP; only
SIGKILL skips that. For one-off messages use `ros2 topic pub --times N`, and wrap any command
that could block in `timeout -s KILL <s>`: a `timeout 6 ros2 topic pub ... | tail` hung for 120 s
in a lab run while the robot drove into a wall.

For Nav2, the `map` frame is the world frame. The `origin` in the map YAML is the position of the
image's lower-left corner, not an offset to add to goals.

## 3. Check the tree

```bash
set -a; . config/ros_dds.env; set +a
source /opt/ros/<distro>/setup.bash
python3 scripts/ros2_tf_doctor.py --fixed-frame world
# no system ROS 2 (only Isaac Sim installed): use its bundled ROS 2
python3 scripts/ros2_tf_doctor.py --fixed-frame world --isaac-ros
```

It listens for a few seconds and reports frames with two parents, frames on both `/tf` and
`/tf_static`, frames that do not reach the Fixed Frame, mixed sim/wall time, a missing
`/clock`, frames that stopped updating, and which nodes publish TF. It also compares the
header stamps of sensor topics (LaserScan, Image, CameraInfo, PointCloud2, Imu, Odometry, and
any `--topic`) with `/clock`. Exit 0 means the tree is consistent. Fix every `ERROR` before debugging RViz itself.

Example: an extra `robot_state_publisher` running next to Isaac's link TF:

```text
ERROR: frame 'wrist_3_link' has 2 parents (base_link, wrist_2_link): two publishers disagree about the tree, so RViz jumps between them. Keep exactly one.
ERROR: simulation-time and wall-clock stamps are mixed: ... Start every node with use_sim_time:=true.
WARNING: several nodes publish TF: /_Graph_twinsmith_ros2_TFRobotLinks, ..., /robot_state_publisher. If Isaac publishes the robot's links, do not also run robot_state_publisher for it.
```

## Frames Isaac Sim 6.2 publishes

In Isaac Sim 6.2, transform publication was modernized to use the standard OmniGraph nodes:

```
USD Stage Prims ──► IsaacComputeTransformTree ──► ROS2PublishTransformTree ──► /tf, /tf_cam, /objects_poses_sim
```

On 6.x builds that have the compute node, this replaces the deprecated `targetPrims` pose-tree interface. 5.1.0.0 and 6.0.0.0 don't have that node and use the older publisher. On 6.2 the older interface still published the whole tree in testing, but logs a deprecation warning.

### Frame names

Frame naming rules in `IsaacComputeTransformTree` preserve existing ROS consumer expectations:
- Unique prims retain their bare name (e.g., `base1`, `inverted_u_yellow`, `line_green`).
- Duplicate prim names under different parents resolve to `<parent>_<child>`.
- Any prim with the string attribute `isaac:nameOverride` uses that override unconditionally.

### The 180° camera rotation

`IsaacComputeTransformTree` deliberately applies a **180° rotation around the X-axis** to all `UsdGeomCamera` prims (`ros2.nodes/docs/CHANGELOG.md:310`, version 1.14.0):
- **Rationale:** USD native cameras look down the $-Z$ axis with $+Y$ up. ROS optical frames convention expects $+X$ forward, $+Y$ right, $+Z$ down (or $+Z$ forward for optical axes). NVIDIA corrected the camera transform evaluation in 6.2 to align with ROS standard conventions.
- **Impact:** Transforms published on `/tf_cam` reflect this 180° X rotation compared to the old 5.1 output. Plain `UsdGeomXform` objects (such as those published on `/objects_poses_sim`) are unaffected.
- **Instruction:** **Do not attempt to "fix" or revert this rotation.** The change reflects the official Isaac Sim ROS 2 node standard and represents the pipeline functioning correctly. Reference values are recorded in [`provenance/tf-baseline-6.2.json`](provenance/tf-baseline-6.2.json).

## Troubleshooting

- **A `ROS_LOCALHOST_ONLY is deprecated` line at every node start on Jazzy** is harmless: `config/ros_dds.env` keeps that setting so Humble and Jazzy clients agree.
- **`ros2 topic echo --once` on a costmap can return an old sample:** Nav2's costmap publisher is quiet with no subscriber, so keep a subscriber running while you look.
- **`sim__occupancy_map` writes to a temporary folder** that goes away with the simulator: copy the `.pgm` and `.yaml` out if you want to keep them.

| Symptom | Cause | Fix |
|---|---|---|
| RViz: "Frame [map] does not exist" | Fixed Frame is not the tree's root | Use `rviz_fixed_frame` from the tool, or the doctor's `roots` |
| Robot jumps or flickers | two publishers for the same frames | doctor shows "has 2 parents"; keep one (`link_frames`) |
| "Lookup would require extrapolation", nothing moves | wall-clock node in a sim-time tree | `use_sim_time:=true` everywhere; Isaac must be playing (`/clock`) |
| No topics at all | DDS or domain mismatch | source `config/ros_dds.env` and use the simulator's `ROS_DOMAIN_ID` (the tool reports it) |
| TF shows only the robot's top prim | targeted the robot Xform, not its articulation | the tool resolves the articulation root; for hand-built graphs target that prim |
| RobotModel white/missing | no `/robot_description` | `link_frames=robot_state_publisher` + `robot_state_publisher` with the URDF |
| RViz log floods "Detected jump back in time. Resetting RViz" | two `/clock` publishers | doctor lists them; keep one (`clock=auto` reuses the existing one) |
| RViz drops every scan: "timestamp on the message is earlier than all the data in the transform cache" (or `/scan` runs ahead of `/clock`: set `resetSimulationTimeOnStop=true` on the helper when the clock resets on Stop) | the lidar helper restarts at 0 on Stop but the clock graph's Read Simulation Time has `resetOnStop=False` | doctor reports the topic's offset; give every Read Simulation Time node the same `resetOnStop` (`True` with sensor helpers), then Stop and Play |
| LaserScan shown in RViz but no points visible | Isaac's 2D lidar publishes zero intensities and the display colours by intensity | use `Color Transformer: FlatColor` (`ros2_rviz_config.py` writes that) |
| ROS 2 node fails: "SharedMemory: unknown element" / "rmw handle is invalid" | an old `cyclonedds.xml` with `<SharedMemory>` and a Cyclone build without it | use this repository's current `config/cyclonedds.xml` |
| Wheels turn at v/r, chassis doesn't move | wheels are `Cylinder` colliders on a very flat box floor | [use Quick Start's ground](#wheels-spin-but-the-robot-stays-put) |
| Costmaps: "The tf tree is invalid because it contains a loop" | two publishers for one edge, or an older `sim__ros2_tf` on a `base_footprint` robot | doctor shows `TF loop`; rerun the current tool, stopped |
| `/scan` silent, Kit logs "not attached to RTX Lidar" | the render product targets the wrapper prim, not the OmniLidar | use `ros2_scan_graph` from `sim__lidar(action="create")`, whose target is its `sensor_path` |
| `/scan` silent after reopening a saved scene | the helper was wired to the tool's own `IsaacMCP_*` product, which saves drop | use `ros2_scan_graph`, which makes its own product at Play |
| `/scan` is all `-1.0` | no return for those beams: `-1.0` means nothing in range. A few are normal | put something within the sensor's range, on its scan plane |
| Kit logs "no usable time-code range" | a hand-written file has no `startTimeCode`/`endTimeCode` | add `startTimeCode = 0` and `endTimeCode = 1000000` to its header. `sim__stage(action="open")` reports this. Measured: driving still worked on 6.2 and 5.1, and `/scan` on 6.2 |
| Nav2 "Control loop missed its desired rate", "Costmap timed out" | headless Kit runs faster than real time | `./run.sh --headless --realtime` |
| Frames stop after reset | timeline stopped | play again; RViz resets once if `/clock` restarts from 0 |
| Robot vanishes after stop/play, "extrapolation into the past" | TF and `/clock` from different time sources | doctor reports the skew; re-run `sim__ros2_tf` (`clock=auto`) |

Validated on Isaac Sim 6.2 (UR5e + RG2 + camera: 17 frames, one root; NVIDIA Jetbot mobile
base: `map -> odom -> chassis -> wheels` with `/odom`), with the doctor using the bundled ROS 2.
