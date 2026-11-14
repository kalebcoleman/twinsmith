#!/usr/bin/env python3
"""Check a live ROS 2 TF tree before trusting RViz (run it with ROS 2 sourced).

    source /opt/ros/<distro>/setup.bash
    python3 scripts/ros2_tf_doctor.py --fixed-frame world [--seconds 3] [--json] [--topic /my_topic]

    # no system ROS 2: use the ROS 2 libraries bundled with the installed Isaac Sim
    python3 scripts/ros2_tf_doctor.py --fixed-frame world --isaac-ros

It listens to /tf, /tf_static and /clock for a few seconds (the Isaac Sim timeline must be
playing) and reports the problems that make RViz show a wrong or flickering tree:

- a frame with two parents, or published on both /tf and /tf_static (two publishers
  disagree, e.g. Isaac's link TF *and* robot_state_publisher);
- frames that do not connect to the Fixed Frame, or a Fixed Frame nobody publishes;
- simulation-time and wall-clock stamps mixed in one tree, or sim-time stamps with no
  /clock (some node is missing use_sim_time:=true);
- several /tf publishers, listed by node, when frames look duplicated;
- more than one /clock publisher, or /clock going backwards (RViz then keeps resetting);
- TF stamps far from /clock (two time sources, e.g. after a stop/play);
- sensor message stamps (LaserScan, Image, CameraInfo, PointCloud2, Odometry, Imu, or any
  --topic) far from /clock: RViz then drops every message ("timestamp on the message is
  earlier than all the data in the transform cache"). A sensor helper that restarts at 0 on
  Stop while the clock graph keeps counting (resetOnStop differs) does this.

Exit status: 0 no errors, 1 errors found, 2 could not run (ROS 2 not sourced, no data).
``analyze`` is pure and unit-tested without ROS.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

WALL_EPOCH = 1e9  # stamps above this are wall-clock time (2001+); simulation time starts at 0


RESET_TO = 0.5  # a /clock jump to below this many seconds is a timeline stop/play, not a second clock
CLOCK_SKEW = 1.0  # TF this far from /clock (sim seconds) cannot be looked up by use_sim_time nodes


def clock_jumps(times):
    """Return (resets, backward_steps) in a /clock sample sequence."""
    resets = back = 0
    for a, b in zip(times, times[1:]):
        if b < a:
            if b < RESET_TO and a - b > RESET_TO:
                resets += 1
            else:
                back += 1
    return resets, back


SENSOR_TYPES = ("sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
                "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Imu", "nav_msgs/msg/Odometry")


def analyze(edges, fixed_frame, *, clock_seen, tf_publishers=(), clock_publishers=(), clock_times=(),
            topic_stamps=None, unchecked=()):
    """Return ``(errors, warnings, info)`` for observed TF edges.

    ``edges``: iterable of dicts ``{"parent", "child", "static", "stamp"}`` (stamp in seconds).
    ``tf_publishers``: node names publishing /tf or /tf_static.
    ``topic_stamps``: ``{topic: [(header stamp, newest /clock when it arrived or None), ...]}``.
    ``unchecked``: requested topics that could not be checked (not found, no header, type not importable).
    """
    errors, warnings = [], []
    parents, kinds, stamps = {}, {}, {}
    for e in edges:
        child, parent = e["child"].lstrip("/"), e["parent"].lstrip("/")
        parents.setdefault(child, set()).add(parent)
        kinds.setdefault(child, set()).add("static" if e["static"] else "dynamic")
        if not e["static"]:
            stamps.setdefault(child, []).append(float(e["stamp"]))
    fixed_frame = fixed_frame.lstrip("/")

    frames = set(parents) | {p for ps in parents.values() for p in ps}
    for child, ps in sorted(parents.items()):
        if len(ps) > 1:
            errors.append(f"frame '{child}' has {len(ps)} parents ({', '.join(sorted(ps))}): two publishers "
                          "disagree about the tree, so RViz jumps between them. Keep exactly one.")
        if len(kinds[child]) > 1:
            errors.append(f"frame '{child}' is published on both /tf and /tf_static; publish it on one only.")
        if child == fixed_frame:
            warnings.append(f"the Fixed Frame '{fixed_frame}' has a parent ({', '.join(sorted(ps))}); "
                            "RViz normally uses the root of the tree.")

    roots = sorted(f for f in frames if f not in parents)
    if not frames:
        errors.append("no transforms received on /tf or /tf_static: is the timeline playing, and are both "
                      "sides on the same ROS_DOMAIN_ID?")
    elif fixed_frame not in frames:
        errors.append(f"the Fixed Frame '{fixed_frame}' is not in the tree; roots are: {', '.join(roots) or 'none, every frame has a parent, so there is a TF loop'}.")
        # Kit drops odom -> base as a duplicate child when links loop, so a loop can hide the Fixed Frame too.
        for child in sorted(parents):
            seen, frame = [], child
            while frame in parents and frame not in seen:
                seen.append(frame)
                frame = sorted(parents[frame])[0]
            if frame in seen:
                loop = seen[seen.index(frame):]
                start = loop.index(min(loop))
                loop = loop[start:] + loop[:start]
                message = (f"TF loop {' -> '.join(loop + [loop[0]])}: a frame is its own ancestor, so nothing in it can be placed. "
                           "Two publishers disagree about which frame is the parent; keep one.")
                if message not in errors:
                    errors.append(message)
    else:
        cycles = set()

        def root_of(f):
            seen = []
            while f in parents:
                if f in seen:  # a loop: two publishers made a frame its own ancestor
                    loop = seen[seen.index(f):]
                    start = loop.index(min(loop))  # one spelling per loop, whatever frame we came from
                    loop = loop[start:] + loop[:start]
                    cycles.add(" -> ".join(loop + [loop[0]]))
                    return None
                seen.append(f)
                f = sorted(parents[f])[0]
            return f
        fixed_root = root_of(fixed_frame)
        # Frames in or below a loop are reported as the loop; others must reach the Fixed Frame's root.
        rooted = {f: root_of(f) for f in frames}
        if fixed_root is None:
            # The Fixed Frame is in a loop. Its ancestors (through any parent) belong to that broken
            # tree and are covered by the loop and two-parents errors; report only the rest.
            ancestors, todo = set(), [fixed_frame]
            while todo:
                for parent in parents.get(todo.pop(), ()):
                    if parent not in ancestors:
                        ancestors.add(parent)
                        todo.append(parent)
            cut = sorted(f for f, r in rooted.items() if r is not None and f not in ancestors
                         and r not in ancestors)
        else:
            cut = sorted(f for f, r in rooted.items() if r is not None and r != fixed_root)
        for loop in sorted(cycles):
            errors.append(f"TF loop {loop}: a frame is its own ancestor, so nothing in it can be placed. "
                          "Two publishers disagree about which frame is the parent; keep one.")
        if cut:
            errors.append(f"{len(cut)} frame(s) do not connect to '{fixed_frame}': {', '.join(cut[:12])}"
                          + (" ..." if len(cut) > 12 else "") + f" (separate roots: {', '.join(roots)}).")

    domains = {}
    for child, values in stamps.items():
        for v in values:
            domains.setdefault("wall" if v > WALL_EPOCH else "sim", set()).add(child)
    if "sim" in domains and "wall" in domains:
        errors.append("simulation-time and wall-clock stamps are mixed: wall-clock frames "
                      f"{', '.join(sorted(domains['wall'])[:6])}; sim-time frames {', '.join(sorted(domains['sim'])[:6])}. "
                      "Start every node with use_sim_time:=true.")
    if "sim" in domains and not clock_seen:
        errors.append("TF uses simulation time but nothing publishes /clock; nodes with use_sim_time:=true "
                      "will wait forever. sim__ros2_tf publishes /clock.")
    clock_pubs = sorted(set(clock_publishers))
    if len(clock_pubs) > 1:
        errors.append(f"{len(clock_pubs)} nodes publish /clock ({', '.join(clock_pubs)}): time jumps back and forth, "
                      "so RViz keeps resetting ('Detected jump back in time') and TF is cleared. Keep one; "
                      "sim__ros2_tf reuses an existing clock.")
    else:
        resets, back = clock_jumps(list(clock_times))
        if back:
            errors.append(f"/clock went backwards {back} time(s) while listening (not a reset to 0): two clock "
                          "sources; RViz clears TF on every jump.")
        if resets:
            warnings.append(f"/clock restarted from 0 {resets} time(s): the timeline was stopped and played. "
                            "RViz resets once per restart; that is expected.")
    # Newest sample of each frame only (lists are in arrival order), so a stop/play while
    # listening does not compare pre-reset TF with the post-reset clock.
    # Each frame on its own: one robot on the wrong time base must not hide behind another.
    newest = {c: v[-1] for c, v in stamps.items() if v[-1] <= WALL_EPOCH}
    if newest and clock_times:
        off = {c: t - clock_times[-1] for c, t in newest.items() if abs(t - clock_times[-1]) > CLOCK_SKEW}
        if off:
            worst = max(off, key=lambda c: abs(off[c]))
            errors.append(f"{len(off)} frame(s) differ from /clock by more than {CLOCK_SKEW:g} s of sim time "
                          f"(worst: {worst} {off[worst]:+.1f} s; {', '.join(sorted(off)[:8])}): they use a different "
                          "time source (one restarts at 0 on stop/play, the other keeps counting), so use_sim_time "
                          "nodes cannot look them up. Re-run sim__ros2_tf (clock=auto).")
    if "wall" in domains and "sim" not in domains and clock_seen:
        warnings.append("/clock is published but TF uses wall-clock time; the TF publishers are not using "
                        "simulation time.")

    publishers = sorted(set(tf_publishers))
    if len(publishers) > 1 and any(("parents" in e or "/tf_static" in e or "TF loop" in e) for e in errors):
        warnings.append("several nodes publish TF: " + ", ".join(publishers)
                        + ". If Isaac publishes the robot's links, do not also run robot_state_publisher for it.")
    if "sim" in domains:  # compare
        # Compare within simulation time only: wall-clock stamps are a different clock.
        sim = {c: [x for x in v if x <= WALL_EPOCH] for c, v in stamps.items()}
        sim = {c: v for c, v in sim.items() if v}
        newest = max(max(v) for v in sim.values())
        stale = sorted(c for c, v in sim.items() if newest - max(v) > 1.0)
        if stale:
            warnings.append(f"frames not updated in the last second of sim time: {', '.join(stale[:8])}.")

    topic_stamps = topic_stamps or {}
    off, silent = {}, sorted(t for t, v in topic_stamps.items() if not v)
    wall = sorted(t for t, v in topic_stamps.items() if v and v[-1][0] > WALL_EPOCH)
    if wall and clock_times and clock_times[-1] <= WALL_EPOCH:  # bag playback can put wall time on /clock
        errors.append(f"{', '.join(wall[:8])} stamped with wall-clock time while /clock runs sim time: "
                      "use_sim_time nodes such as RViz drop them. The publisher is not using simulation time.")
    for topic, samples in sorted(topic_stamps.items()):
        timed = [(stamp, clock) for stamp, clock in samples if clock is not None and stamp <= WALL_EPOCH]
        if timed and samples[-1][0] <= WALL_EPOCH:
            stamp, clock = timed[-1]  # newest only, as for TF: a stop/play while listening is not a fault
            if abs(stamp - clock) > CLOCK_SKEW:
                off[topic] = stamp - clock
    if off:
        worst = max(off, key=lambda t: abs(off[t]))
        errors.append(f"{len(off)} topic(s) are stamped more than {CLOCK_SKEW:g} s of sim time away from /clock "
                      f"(worst: {worst} {off[worst]:+.1f} s; {', '.join(sorted(off)[:8])}): RViz and Nav2 drop "
                      "them ('timestamp on the message is earlier than all the data in the transform cache'). "
                      "Their publisher and the /clock graph use different time bases: usually one Read "
                      "Simulation Time node has resetOnStop=False while the sensor helper restarts at 0 on Stop. "
                      "Use the same setting everywhere (True is the safe choice with sensor helpers), then Stop and Play.")
    if silent:
        warnings.append(f"no messages on {', '.join(silent[:8])} while listening.")
    if unchecked:
        warnings.append(f"not checked: {', '.join(sorted(unchecked)[:8])} (not published, no header, or its "
                        "message package is not importable here).")

    info = dict(frames=len(frames), roots=roots, fixed_frame=fixed_frame,
                static=sorted(c for c, k in kinds.items() if k == {"static"}),
                time_source=sorted(domains), clock=clock_seen, tf_publishers=publishers, clock_publishers=clock_pubs,
                tree={c: sorted(p) for c, p in sorted(parents.items())},
                topics={t: len(v) for t, v in sorted(topic_stamps.items())}, unchecked=sorted(unchecked))
    return errors, warnings, info


def capture(seconds, topics=()):
    """Listen to /tf, /tf_static, /clock and sensor topics.

    Returns (edges, clock times, TF publishers, clock publishers, topic stamps, unchecked topics). Sensor topics are
    every topic of a SENSOR_TYPES type plus *topics*, subscribed after a short discovery pause.
    """
    import importlib

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("twinsmith_tf_doctor")
    edges, clock = [], []

    def on_tf(static):
        def cb(msg):
            for t in msg.transforms:
                edges.append(dict(parent=t.header.frame_id, child=t.child_frame_id, static=static,
                                  stamp=t.header.stamp.sec + t.header.stamp.nanosec * 1e-9))
        return cb

    static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, "/tf", on_tf(False), 100)
    node.create_subscription(TFMessage, "/tf_static", on_tf(True), static_qos)
    node.create_subscription(Clock, "/clock", lambda m: clock.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
    discover = time.monotonic() + 0.5  # then listen the full *seconds* with every subscription in place
    end = discover + seconds
    while time.monotonic() < discover:
        rclpy.spin_once(node, timeout_sec=0.05)

    topic_stamps = {}
    sensor_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)  # matches reliable and best-effort publishers
    wanted = {t if t.startswith("/") else "/" + t for t in topics} - {"/tf", "/tf_static", "/clock"}  # checked anyway
    for name, types in node.get_topic_names_and_types():
        kind = next((t for t in types if t in SENSOR_TYPES or name in wanted), None)
        if kind is None or name in ("/tf", "/tf_static", "/clock"):
            continue
        pkg, _, cls = kind.split("/")
        try:
            msg_type = getattr(importlib.import_module(f"{pkg}.msg"), cls)
        except (ImportError, AttributeError):
            continue
        if "header" not in getattr(msg_type, "get_fields_and_field_types", dict)():
            continue
        topic_stamps[name] = []

        def on_msg(msg, name=name):
            s = msg.header.stamp
            topic_stamps[name].append((s.sec + s.nanosec * 1e-9, clock[-1] if clock else None))
        node.create_subscription(msg_type, name, on_msg, sensor_qos)
    unchecked = sorted(wanted - set(topic_stamps))  # asked for but not found, no header, or not importable
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    publishers = set()
    for topic in ("/tf", "/tf_static"):
        for info in node.get_publishers_info_by_topic(topic):
            publishers.add(f"{info.node_namespace.rstrip('/')}/{info.node_name}")
    publishers.discard("/twinsmith_tf_doctor")
    clock_publishers = {f"{i.node_namespace.rstrip('/')}/{i.node_name}"
                        for i in node.get_publishers_info_by_topic("/clock")}
    node.destroy_node()
    rclpy.shutdown()
    return edges, clock, publishers, clock_publishers, topic_stamps, unchecked


def dds_env(env):
    """The DDS settings run.sh gives the simulator: config/ros_dds.env, sourced exactly as run.sh
    does (it keeps an exported ROS_DOMAIN_ID and forces the rest), so both sides agree."""
    keys = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI")
    script = ROOT / "config" / "ros_dds.env"
    out = subprocess.run(["bash", "-c", 'set -a; . "$1"; for k in "${@:2}"; do printf "%s=%s\\0" "$k" "${!k-}"; done',
                          "bash", str(script), *keys], capture_output=True, text=True, env=env,
                         stdin=subprocess.DEVNULL, timeout=30)
    if out.returncode:
        raise RuntimeError(f"could not read {script}: {out.stderr.strip()[:200]}")
    return dict(item.split("=", 1) for item in out.stdout.split("\0") if "=" in item and item.split("=", 1)[1])


def isaac_ros_env(env=None):
    """``(python, env)`` to run this script on the ROS 2 libraries bundled with Isaac Sim.

    Same choices as run.sh: the interpreter from scripts/env.py, the bundled distro
    (humble on Ubuntu 22.04, jazzy otherwise, or $ROS_DISTRO) and config/ros_dds.env's DDS
    settings, so the doctor discovers exactly what the simulator publishes.
    """
    env = dict(os.environ if env is None else env)
    run = dict(capture_output=True, text=True, env=env, stdin=subprocess.DEVNULL, timeout=60)
    python = subprocess.run([sys.executable, str(ROOT / "scripts" / "env.py"), "resolve-python",
                             "--require-version", "6"], **run).stdout.strip()
    if not python:
        raise RuntimeError("no Isaac Sim 6.x found (python3 scripts/env.py find)")
    # Locate the install from its file layout. Never `import isaacsim` here: that can stop and
    # wait for the EULA prompt.
    interp = Path(python)
    if interp.name == "python.sh":  # standalone install root
        roots = [interp.parent]
    elif interp.parts[-4:-1] == ("kit", "python", "bin"):  # standalone <root>/kit/python/bin/python3
        roots = [interp.parents[3]]
    else:
        purelib = subprocess.run([python, "-I", "-c", "import sysconfig; print(sysconfig.get_paths()['purelib'])"],
                                 **run).stdout.strip()
        roots = [Path(purelib) / "isaacsim"]
    distro = env.get("ROS_DISTRO") or ("humble" if "jammy" in Path("/etc/os-release").read_text(errors="replace")
                                        else "jazzy")
    # The bundled ROS 2 is in isaacsim.ros2.core on 6.x and isaacsim.ros2.bridge on 5.1.
    base = roots[0] / "exts" / "isaacsim.ros2.core" / distro
    if not (base / "rclpy").is_dir() and (roots[0] / "exts" / "isaacsim.ros2.bridge" / distro / "rclpy").is_dir():
        base = roots[0] / "exts" / "isaacsim.ros2.bridge" / distro
    if not (base / "rclpy").is_dir():
        raise RuntimeError(f"Isaac Sim has no bundled ROS 2 {distro} at {base}")
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(base / "rclpy"), env.get("PYTHONPATH")]))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, [str(base / "lib"), env.get("LD_LIBRARY_PATH")]))
    env.update(dds_env(env))
    env["TWINSMITH_TF_DOCTOR_REEXEC"] = "1"
    return python, env


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--fixed-frame", required=True, help="RViz Fixed Frame (sim__ros2_tf: rviz_fixed_frame)")
    p.add_argument("--seconds", type=float, default=3.0, help="how long to listen (default 3)")
    p.add_argument("--json", action="store_true")
    p.add_argument("--topic", action="append", default=[], metavar="TOPIC",
                   help="also check this topic's header stamps against /clock (sensor topics are checked anyway)")
    p.add_argument("--isaac-ros", action="store_true",
                   help="use the ROS 2 libraries bundled with Isaac Sim (no system ROS 2 needed)")
    args = p.parse_args(argv)
    if args.isaac_ros and os.environ.get("TWINSMITH_TF_DOCTOR_REEXEC") != "1":
        try:
            python, env = isaac_ros_env()
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
            print(f"--isaac-ros: {exc}", file=sys.stderr)
            return 2
        rest = [a for a in (argv if argv is not None else sys.argv[1:]) if a != "--isaac-ros"]
        return subprocess.run([python, str(Path(__file__).resolve()), *rest], env=env).returncode
    try:
        edges, clock_times, publishers, clock_publishers, topic_stamps, unchecked = capture(
            max(0.5, min(args.seconds, 30.0)), args.topic)
    except ImportError as exc:
        print(f"ROS 2 Python packages not found ({exc}). Source ROS 2 first "
              "(source /opt/ros/<distro>/setup.bash), or pass --isaac-ros to use Isaac Sim's bundled ROS 2.",
              file=sys.stderr)
        return 2
    errors, warnings, info = analyze(edges, args.fixed_frame, clock_seen=bool(clock_times), tf_publishers=publishers,
                                     clock_publishers=clock_publishers, clock_times=clock_times,
                                     topic_stamps=topic_stamps, unchecked=unchecked)
    if args.json:
        print(json.dumps(dict(errors=errors, warnings=warnings, **info), indent=2))
    else:
        print(f"TF: {info['frames']} frames, roots {info['roots']}, time {info['time_source'] or 'none'}, "
              f"/clock {'yes' if clock_times else 'no'} ({len(info['clock_publishers'])} publisher(s)), "
              f"TF publishers {info['tf_publishers'] or 'none'}"
              + (f", topics checked {sorted(info['topics'])}" if info["topics"] else ""))
        for e in errors:
            print("ERROR:", e)
        for w in warnings:
            print("WARNING:", w)
        if not errors:
            print(f"OK: every frame connects to '{info['fixed_frame']}'.")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
