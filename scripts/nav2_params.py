#!/usr/bin/env python3
"""Write a Nav2 parameter file for a robot in Isaac Sim, from the nav2_params.yaml your Nav2 ships.

Standard library only. Source ROS 2 with Nav2 first (scripts/ros_env.sh), so it can find nav2_bringup.

    python3 scripts/nav2_params.py --out isaac_nav2.yaml                  # TurtleBot3 at the map origin
    python3 scripts/nav2_params.py --base-frame chassis_link --radius 0.45 --out carter_nav2.yaml

It only changes what Isaac's frames need: the robot's base frame (Nav2's defaults mix base_link and base_footprint, and
the frame must exist in your tree), the footprint radius, and AMCL's starting pose, so that no one has to click 2D Pose
Estimate in RViz. Use the frame sim__ros2_tf reports as base_frame, with world_frame="".
"""

import argparse
import os
from pathlib import Path
import re
import sys


def find_default(prefixes):
    for prefix in prefixes:
        candidate = Path(prefix) / "share" / "nav2_bringup" / "params" / "nav2_params.yaml"
        if candidate.is_file():
            return candidate
    return None


def rewrite(text, base_frame="base_link", radius=0.18, x=0.0, y=0.0, yaw=0.0, yaw_tolerance=None):
    text = re.sub(r'(\bbase_frame_id:\s*)"?[\w/]+"?', rf'\1"{base_frame}"', text)
    text = re.sub(r'(\brobot_base_frame:\s*)[\w/]+', rf'\g<1>{base_frame}', text)
    text = re.sub(r'(\brobot_radius:\s*)[\d.]+', rf'\g<1>{radius}', text)
    if yaw_tolerance is not None:
        text = re.sub(r'(\byaw_goal_tolerance:\s*)[\d.]+', rf'\g<1>{yaw_tolerance}', text)
    # AMCL: start at the known pose. Added to AMCL's own block when this Nav2's default file omits the keys.
    pose = f"x: {x}, y: {y}, z: 0.0, yaw: {yaw}"
    text = re.sub(r'\n\s*set_initial_pose:.*', '', text)
    text = re.sub(r'\n\s*initial_pose:\s*\{.*?\}', '', text)
    text = re.sub(r'\n\s*initial_pose:\s*\n(\s+(x|y|z|yaw):.*\n)+', '\n', text)
    return re.sub(r'(^amcl:\n\s*ros__parameters:\n)(\s*)', lambda m: f"{m[1]}{m[2]}set_initial_pose: true\n{m[2]}initial_pose: {{{pose}}}\n{m[2]}", text, count=1, flags=re.M)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, type=Path, help="file to write")
    parser.add_argument("--base-frame", default="base_link", help="the robot's base frame (default base_link)")
    parser.add_argument("--radius", type=float, default=0.18, help="robot radius in metres (default 0.18, a TurtleBot3 Burger)")
    parser.add_argument("--x", type=float, default=0.0, help="start x in the map, metres")
    parser.add_argument("--y", type=float, default=0.0, help="start y in the map, metres")
    parser.add_argument("--yaw", type=float, default=0.0, help="start yaw in the map, radians")
    parser.add_argument("--yaw-tolerance", type=float, help="final heading tolerance in radians (default: Nav2's). 3.2 ignores the heading")
    parser.add_argument("--source", type=Path, help="nav2_params.yaml to start from (default: the one in nav2_bringup)")
    args = parser.parse_args(argv)
    source = args.source or find_default(os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep))
    if not source or not Path(source).is_file():
        print("nav2_params: nav2_bringup not found. Source ROS 2 with Nav2 (scripts/ros_env.sh), or pass --source. If you did source it, "
              "check that it ran in this shell and not in a pipeline (`source ... | tail`) or a subshell: those silently lose the environment.", file=sys.stderr)
        return 2
    args.out.expanduser().write_text(rewrite(Path(source).read_text(), args.base_frame, args.radius, args.x, args.y, args.yaw, args.yaw_tolerance))
    print(f"{args.out} from {source}: base frame {args.base_frame}, radius {args.radius}, start ({args.x}, {args.y}, {args.yaw})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
