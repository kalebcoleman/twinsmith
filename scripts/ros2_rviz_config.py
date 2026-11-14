#!/usr/bin/env python3
"""Write an RViz 2 config for an Isaac Sim robot, with the right Fixed Frame and displays.

Standard library only; no ROS installation is needed to write the file.

    python3 scripts/ros2_rviz_config.py --fixed-frame world --out lab.rviz \\
        --robot-model --image /camera/rgb --scan /scan --cloud /point_cloud --odom /odom
    rviz2 -d lab.rviz --ros-args -p use_sim_time:=true

The layout follows NVIDIA's isaac_tutorials RViz configs (IsaacSim-ros_workspaces,
``camera_lidar.rviz``): a Grid and TF display, then one display per topic, with the
Fixed Frame set to the root of the TF tree that ``sim__ros2_tf`` reports
(``rviz_fixed_frame``). ``--robot-model`` adds a RobotModel that reads
``/robot_description``; it needs robot_state_publisher running with the URDF.

View and window options (checked in a real rviz2, Humble):

    --view topdown --scale 80 --target-frame base_link   # 2D map-style view that follows the robot
    --view orbit --distance 6 --target-frame base_link   # 3D orbit camera that follows the robot
    --hide-panels                                        # only the 3D view (and image panels)

``--hide-panels`` leaves the Displays and Views panels out of the file. RViz2 ignores
``Hide Left Dock: true`` and per-panel ``collapsed: true`` when it loads a config (the docks are
not visible yet, so its collapse code does nothing), which is why setting them did not work.
Panels can be added back from RViz's Panels menu.

Image size: an Image display opens its own dock (about 320 px wide in rviz2 Humble). RViz2 has no
per-display size key; the dock layout lives only in the binary ``QMainWindow State`` that RViz
writes itself. To keep a larger image, drag the dock edge in RViz and use File > Save Config.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

FRAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
TOPIC_RE = re.compile(r"^/[A-Za-z_][A-Za-z0-9_/]*$")

KINDS = {
    "image": ("rviz_default_plugins/Image", "Image"),
    "scan": ("rviz_default_plugins/LaserScan", "LaserScan"),
    "cloud": ("rviz_default_plugins/PointCloud2", "PointCloud2"),
    "odom": ("rviz_default_plugins/Odometry", "Odometry"),
}


def q(text: str) -> str:
    """YAML scalar: JSON strings are valid YAML double-quoted scalars."""
    return json.dumps(text)


def topic_block(value: str, indent: str) -> list[str]:
    return [
        f"{indent}Topic:",
        f"{indent}  Depth: 5",
        f"{indent}  Durability Policy: Volatile",
        f"{indent}  History Policy: Keep Last",
        f"{indent}  Reliability Policy: Reliable",
        f"{indent}  Value: {q(value)}",
    ]


def display(kind: str, topic: str, index: int) -> list[str]:
    cls, label = KINDS[kind]
    i = "    "
    lines = [f"  - Class: {cls}", f"{i}Enabled: true", f"{i}Name: {q(f'{label} {topic}')}", f"{i}Value: true"]
    if kind in ("scan", "cloud"):
        # Flat colour: Isaac's RTX 2D lidar publishes all-zero intensities, and with the Intensity
        # transformer RViz drew the scan invisibly.
        lines += [f"{i}Size (m): {0.05 if kind == 'scan' else 0.01}", f"{i}Style: Flat Squares",
                  f"{i}Color Transformer: FlatColor", f"{i}Color: 255; 0; 0", f"{i}Use Fixed Frame: true"]
    if kind == "odom":
        # One small arrow: the default 50 full-size arrows covered the robot and scan.
        lines += [f"{i}Keep: 1", f"{i}Position Tolerance: 0.1", f"{i}Angle Tolerance: 0.1",
                  f"{i}Shape:", f"{i}  Value: Arrow", f"{i}  Shaft Length: 0.3", f"{i}  Shaft Radius: 0.02",
                  f"{i}  Head Length: 0.08", f"{i}  Head Radius: 0.05", f"{i}  Color: 255; 25; 0"]
    if kind == "image":
        lines += [f"{i}Normalize Range: true"]
    return lines + topic_block(topic, i)


def view_block(view: str, target_frame: str, distance: float, scale: float) -> list[str]:
    """The ``Views: Current:`` block. ``target_frame`` "<Fixed Frame>" keeps the view still."""
    i = "      "
    if view == "topdown":
        return [f"{i}Class: rviz_default_plugins/TopDownOrtho",
                f"{i}Angle: 0",
                f"{i}Name: Current View",
                f"{i}Near Clip Distance: 0.01",
                f"{i}Scale: {scale:g}",
                f"{i}Target Frame: {q(target_frame)}",
                f"{i}Value: TopDownOrtho (rviz_default_plugins)",
                f"{i}X: 0",
                f"{i}Y: 0"]
    return [f"{i}Class: rviz_default_plugins/Orbit",
            f"{i}Distance: {distance:g}",
            f"{i}Focal Point:",
            f"{i}  X: 0",
            f"{i}  Y: 0",
            f"{i}  Z: 0",
            f"{i}Name: Current View",
            f"{i}Near Clip Distance: 0.01",
            f"{i}Pitch: 0.785398",
            f"{i}Target Frame: {q(target_frame)}",
            f"{i}Value: Orbit (rviz)",
            f"{i}Yaw: 0.785398"]


def render(fixed_frame: str, robot_model: bool, description_topic: str, topics: list[tuple[str, str]],
           view: str = "orbit", target_frame: str = "<Fixed Frame>", distance: float = 3.0,
           scale: float = 100.0, hide_panels: bool = False, width: int = 1400, height: int = 900) -> str:
    panels = (["Panels: []"] if hide_panels else [
        "Panels:",
        "  - Class: rviz_common/Displays",
        "    Name: Displays",
        "  - Class: rviz_common/Views",
        "    Name: Views",
    ])
    out = [
        "# Generated by twinsmith scripts/ros2_rviz_config.py",
        "# Open with: rviz2 -d <this file> --ros-args -p use_sim_time:=true",
        *panels,
        "Visualization Manager:",
        '  Class: ""',
        "  Displays:",
        "  - Class: rviz_default_plugins/Grid",
        "    Enabled: true",
        "    Name: Grid",
        "    Reference Frame: <Fixed Frame>",
        "    Plane: XY",
        "    Value: true",
        "  - Class: rviz_default_plugins/TF",
        "    Enabled: true",
        "    Name: TF",
        "    Frame Timeout: 15",
        "    Show Arrows: true",
        "    Show Axes: true",
        "    Show Names: true",
        "    Marker Scale: 0.3",
        "    Value: true",
    ]
    if robot_model:
        out += [
            "  - Class: rviz_default_plugins/RobotModel",
            "    Enabled: true",
            "    Name: RobotModel",
            "    Description Source: Topic",
            "    Description Topic:",
            "      Depth: 5",
            "      Durability Policy: Transient Local",
            "      History Policy: Keep Last",
            "      Reliability Policy: Reliable",
            f"      Value: {q(description_topic)}",
            "    Visual Enabled: true",
            "    Value: true",
        ]
    for n, (kind, topic) in enumerate(topics):
        # Displays are list items of Visualization Manager > Displays (same indentation as above).
        out += display(kind, topic, n)
    out += [
        "  Enabled: true",
        "  Global Options:",
        "    Background Color: 48; 48; 48",
        f"    Fixed Frame: {q(fixed_frame)}",
        "    Frame Rate: 30",
        "  Name: root",
        "  Tools:",
        "  - Class: rviz_default_plugins/Interact",
        "  - Class: rviz_default_plugins/MoveCamera",
        "  - Class: rviz_default_plugins/Select",
        "  - Class: rviz_default_plugins/FocusCamera",
        "  - Class: rviz_default_plugins/Measure",
        "  Transformation:",
        "    Current:",
        "      Class: rviz_default_plugins/TF",
        "  Value: true",
        "  Views:",
        "    Current:",
        *view_block(view, target_frame, distance, scale),
        "    Saved: ~",
        "Window Geometry:",
        f"  Width: {width}",
        f"  Height: {height}",
        # Kept false: RViz2 does not apply true at load (see the module docstring); --hide-panels
        # leaves the panels out instead.
        "  Hide Left Dock: false",
        "  Hide Right Dock: false",
        "",
    ]
    return "\n".join(out)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--fixed-frame", required=True,
                   help="root of the TF tree (sim__ros2_tf reports it as rviz_fixed_frame)")
    p.add_argument("--out", required=True, help="new .rviz file to write")
    p.add_argument("--robot-model", action="store_true",
                   help="add a RobotModel from --description-topic (needs robot_state_publisher)")
    p.add_argument("--description-topic", default="/robot_description")
    for kind in KINDS:
        p.add_argument(f"--{kind}", action="append", default=[], metavar="TOPIC",
                       help=f"add a {KINDS[kind][1]} display (repeatable)")
    p.add_argument("--view", choices=("orbit", "topdown"), default="orbit",
                   help="orbit: 3D camera (default); topdown: 2D TopDownOrtho, like a map")
    p.add_argument("--distance", type=float, default=3.0, help="orbit camera distance in m (default 3)")
    p.add_argument("--scale", type=float, default=100.0,
                   help="topdown zoom in pixels per metre (default 100; 50 shows a 20 m wide area at 1000 px)")
    p.add_argument("--target-frame", default=None,
                   help="frame the view follows, e.g. base_link (default: the Fixed Frame, a still view)")
    p.add_argument("--hide-panels", action="store_true",
                   help="leave out the Displays and Views panels: only the 3D view (and image docks)")
    p.add_argument("--width", type=int, default=1400, help="window width in px (default 1400)")
    p.add_argument("--height", type=int, default=900, help="window height in px (default 900)")
    p.add_argument("--force", action="store_true", help="replace an existing file")
    args = p.parse_args(argv)

    if not FRAME_RE.match(args.fixed_frame):
        p.error(f"--fixed-frame {args.fixed_frame!r} is not a valid ROS frame name")
    if args.target_frame is not None and not FRAME_RE.match(args.target_frame):
        p.error(f"--target-frame {args.target_frame!r} is not a valid ROS frame name")
    if not 0 < args.distance <= 1000:
        p.error("--distance must be > 0 and <= 1000 m")
    if not 0 < args.scale <= 100000:
        p.error("--scale must be > 0 (pixels per metre)")
    if not (200 <= args.width <= 10000 and 200 <= args.height <= 10000):
        p.error("--width and --height must be 200-10000 px")
    topics = [(kind, t) for kind in KINDS for t in getattr(args, kind)]
    for _, t in topics + ([("robot", args.description_topic)] if args.robot_model else []):
        if not TOPIC_RE.match(t):
            p.error(f"topic {t!r} must be absolute, like /scan")
    out = Path(args.out).expanduser()
    if out.suffix != ".rviz":
        p.error("--out must end with .rviz")
    if out.exists() and not args.force:
        p.error(f"{out} exists; pass --force to replace it")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(args.fixed_frame, args.robot_model, args.description_topic, topics,
                          view=args.view, target_frame=args.target_frame or "<Fixed Frame>",
                          distance=args.distance, scale=args.scale, hide_panels=args.hide_panels,
                          width=args.width, height=args.height))
    print(f"wrote {out}")
    print(f"open it with: rviz2 -d {out} --ros-args -p use_sim_time:=true")
    print("check the TF tree first: python3 scripts/ros2_tf_doctor.py --fixed-frame " + args.fixed_frame)
    return 0


if __name__ == "__main__":
    sys.exit(main())
