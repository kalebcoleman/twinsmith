"""sim.ros2_tf: one consistent ROS 2 clock/TF/joint-state graph for a robot.

The layout follows NVIDIA's own graph helpers shipped with Isaac Sim 6.2
(``isaacsim.ros2.nodes`` ``impl/graph_api.py``: ``create_ros2_clock_graph``,
``create_ros2_tf_graph``, ``create_ros2_odometry_graph``) but only uses node
types and inputs that exist in every Isaac Sim 6.x release, so it also works on
6.0/6.1. One graph per robot namespace, rebuilt from scratch on every call, so repeated setup never
leaves a second publisher for the same frame behind.

Frame tree (REP 105 names):

    <world_frame> --static--> <base_frame> --(Isaac link TF)--> every robot link
    <world_frame> --static--> odom --(odometry)--> <base_frame>      (base="mobile";
                                                                      <ns>_odom with a namespace)
    <parent link> --dynamic--> <base_frame> --(Isaac link TF)--> every link  (attach_to: a gripper that is
                                                                      its own articulation on an arm)

``planned_graph`` is pure (no Kit imports) so the layout can be unit-tested.
"""

from __future__ import annotations

import hashlib
import math
import os
import re

GRAPH_PATH = "/Graph/twinsmith_ros2"
FRAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
SEGMENT_RE = re.compile(r"^[A-Za-z](?:[A-Za-z0-9_]*[A-Za-z0-9])?$")
TOPIC_RE = re.compile(r"^/?[A-Za-z_][A-Za-z0-9_/]*$")
CLOCK, CONTEXT, TIME, TICK = "PublishClock", "Context", "ReadSimTime", "OnPlaybackTick"


def _frame(name, what):
    if not isinstance(name, str) or not FRAME_RE.match(name):
        raise ValueError(f"{what} {name!r} is not a valid ROS frame name (letters, digits, _; not starting with a digit)")
    return name


def _namespace(namespace):
    """Normalised namespace ('' or 'a/b'); every segment must be a valid ROS name."""
    ns = (namespace or "").strip("/")
    # Segments may not start or end with '_' nor contain '__' (ROS 2 forbids repeated underscores),
    # so '__' in a flattened name can only come from '/': the '/' -> '__' mapping is one-to-one.
    if ns and not all(SEGMENT_RE.match(part) and "__" not in part for part in ns.split("/")):
        raise ValueError(f"namespace {namespace!r}: each segment must start with a letter, end with a letter or "
                         "digit and not contain '__' (twinsmith rule; keeps <ns>_odom and graph names one-to-one)")
    return ns


def odom_frame(namespace=""):
    """Odometry frame: 'odom', or '<ns>_odom' so several mobile robots never share one."""
    ns = _namespace(namespace)
    return f"{ns.replace('/', '__')}_odom" if ns else "odom"  # a/b -> a__b_odom; a_b -> a_b_odom


def planned_graph(*, robot, base_prim, base_frame, base_pose, mode="fixed", world_frame="world",
                  link_frames="isaac", extra_prims=(), static_frames=(), namespace="", joint_states_topic="joint_states",
                  base_tf_name=None, publish_clock=True, reset_on_stop=True, attach_parent=None,
                  attach_parent_frame=None, link_prims=None, static_links=()):
    """Return ``(nodes, values, connects, frames)`` for the graph.

    ``base_pose`` is ``((x, y, z) metres, (qx, qy, qz, qw))`` of the base link in the world at
    setup time; it becomes the static ``world -> base`` (fixed) or ``world -> odom`` (mobile)
    transform. ``frames`` describes the resulting tree for the caller.

    ``attach_parent`` is a prim path: instead of a world link, publish this base link relative to that
    prim on every tick, so it follows when the parent moves. ``attach_parent_frame`` is its TF name.

    ``link_prims`` lists the links for the live TF node, and None hands it the articulation, which walks the articulation's
    own tree. ``static_links`` are (TF name, translation, rotation) relative to the base: see link_publishing.
    """
    if mode not in ("fixed", "mobile"):
        raise ValueError("base must be 'fixed' or 'mobile'")
    if attach_parent and mode != "fixed":
        raise ValueError("attach_to needs base='fixed': the attached robot's pose comes from its parent, not odometry")
    if link_frames not in ("isaac", "robot_state_publisher"):
        raise ValueError("link_frames must be 'isaac' or 'robot_state_publisher'")
    _frame(base_frame, "base frame")
    namespace = _namespace(namespace)
    odom = odom_frame(namespace)
    links_parent = base_tf_name or base_frame  # what Isaac names the links' parent (the base prim)
    if world_frame:
        _frame(world_frame, "world_frame")
        if world_frame in (base_frame, odom if mode == "mobile" else None):
            raise ValueError(f"world_frame must differ from the base frame (and from '{odom}' for a mobile base)")
    if not TOPIC_RE.match(joint_states_topic or ""):
        raise ValueError(f"joint_states_topic {joint_states_topic!r} is not a valid topic")
    (tx, ty, tz), (qx, qy, qz, qw) = base_pose
    if not all(math.isfinite(v) for v in (tx, ty, tz, qx, qy, qz, qw)):
        raise ValueError("base pose is not finite")

    nodes = [
        (TICK, "omni.graph.action.OnPlaybackTick"),
        (CONTEXT, "isaacsim.ros2.bridge.ROS2Context"),
        (TIME, "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    values = [
        # Must match the /clock this robot's stamps are read against (see setup()): a clock that keeps
        # counting across stop/play next to TF that restarts at 0 leaves TF far in the past.
        (f"{TIME}.inputs:resetOnStop", bool(reset_on_stop)),
        ("PublishJointState.inputs:targetPrim", robot),
        ("PublishJointState.inputs:topicName", joint_states_topic.lstrip("/")),
        ("PublishJointState.inputs:nodeNamespace", namespace),
    ]
    publishers = ["PublishJointState"]
    if publish_clock:
        nodes.append((CLOCK, "isaacsim.ros2.bridge.ROS2PublishClock"))
        values.append((f"{CLOCK}.inputs:topicName", "clock"))
        publishers.append(CLOCK)
    connects = []
    frames = {"links": "isaac" if link_frames == "isaac" else "robot_state_publisher (from /joint_states)"}
    edges = []

    def raw_tf(name, parent, child, static, translation=None, rotation=None):
        nodes.append((name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        values.extend([
            (f"{name}.inputs:parentFrameId", parent),
            (f"{name}.inputs:childFrameId", child),
            (f"{name}.inputs:staticPublisher", static),
            # TF stays on the global /tf and /tf_static (namespace only applies to joint states and
            # odometry), so RViz, tf2 and the doctor see every robot's frames without remapping.
            (f"{name}.inputs:topicName", "tf_static" if static else "tf"),
        ])
        if translation is not None:
            values.append((f"{name}.inputs:translation", list(translation)))
            values.append((f"{name}.inputs:rotation", list(rotation)))
        publishers.append(name)
        edges.append(dict(parent=parent, child=child, static=static, source=name))

    if attach_parent:
        # Parent and child are separate articulations joined by a fixed joint outside both, so neither
        # one's link TF covers this edge. The node reads both poses on every tick.
        nodes.append(("TFAttach", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
        values.extend([
            ("TFAttach.inputs:parentPrim", attach_parent),
            ("TFAttach.inputs:targetPrims", [base_prim]),
            ("TFAttach.inputs:topicName", "tf"),
        ])
        publishers.append("TFAttach")
        edges.append(dict(parent=attach_parent_frame or attach_parent.rsplit("/", 1)[-1], child=links_parent,
                          static=False, source="TFAttach"))
    elif mode == "fixed":
        if world_frame:
            raw_tf("TFWorldToBase", world_frame, base_frame, True, (tx, ty, tz), (qx, qy, qz, qw))
    else:
        if world_frame:
            raw_tf("TFWorldToOdom", world_frame, odom, True, (tx, ty, tz), (qx, qy, qz, qw))
        nodes.extend([
            ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ])
        values.extend([
            # The base itself: the articulation's root link can be another link, and this pose is published as base_frame.
            ("ComputeOdometry.inputs:chassisPrim", base_prim),
            ("PublishOdometry.inputs:topicName", "odom"),
            ("PublishOdometry.inputs:odomFrameId", odom),
            ("PublishOdometry.inputs:chassisFrameId", base_frame),
            ("PublishOdometry.inputs:nodeNamespace", namespace),
        ])
        raw_tf("TFOdomToBase", odom, base_frame, False)
        connects.extend([
            (f"{TICK}.outputs:tick", "ComputeOdometry.inputs:execIn"),
            ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
            ("ComputeOdometry.outputs:execOut", "TFOdomToBase.inputs:execIn"),
            ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
            ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
            ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
            ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
            ("ComputeOdometry.outputs:position", "TFOdomToBase.inputs:translation"),
            ("ComputeOdometry.outputs:orientation", "TFOdomToBase.inputs:rotation"),
        ])
        publishers.append("PublishOdometry")

    if link_frames == "isaac":
        for index, (name, translation, rotation) in enumerate(static_links):
            raw_tf(f"TFFixedLink{index}", links_parent, name, True, translation, rotation)
        if link_prims != []:  # everything can be fixed to the base, leaving nothing live
            # Relative to the base link: base_frame -> every link (prim names).
            nodes.append(("TFRobotLinks", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
            values.extend([
                ("TFRobotLinks.inputs:parentPrim", base_prim),
                ("TFRobotLinks.inputs:targetPrims", [robot] if link_prims is None else list(link_prims)),
                ("TFRobotLinks.inputs:topicName", "tf"),
            ])
            publishers.append("TFRobotLinks")
            edges.append(dict(parent=links_parent, child="<robot links>", static=False, source="TFRobotLinks"))
    for index, (parent, name, translation, rotation) in enumerate(static_frames):
        raw_tf(f"TFStaticFrame{index}", parent, name, True, translation, rotation)
    if extra_prims:
        nodes.append(("TFExtraFrames", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
        values.extend([
            ("TFExtraFrames.inputs:parentPrim", base_prim),
            ("TFExtraFrames.inputs:targetPrims", list(extra_prims)),
            ("TFExtraFrames.inputs:topicName", "tf"),
        ])
        publishers.append("TFExtraFrames")
        edges.append(dict(parent=links_parent, child="<extra prims>", static=False, source="TFExtraFrames"))

    for name in publishers:
        connects.append((f"{CONTEXT}.outputs:context", f"{name}.inputs:context"))
        connects.append((f"{TIME}.outputs:simulationTime", f"{name}.inputs:timeStamp"))
        if name not in ("PublishOdometry", "TFOdomToBase"):  # driven by ComputeOdometry
            connects.append((f"{TICK}.outputs:tick", f"{name}.inputs:execIn"))
    frames["edges"] = edges
    frames["root"] = world_frame or (odom if mode == "mobile" else base_frame)
    frames["odom"] = odom if mode == "mobile" else None
    return nodes, values, connects, frames


def graph_path(namespace=""):
    """One graph per namespace, and so one robot per namespace: two robots need two namespaces, which
    setup enforces (check_graph_robot). The short hash keeps the mapping one-to-one ('a/b' and 'a_b' get
    different graphs)."""
    ns = _namespace(namespace)
    if not ns:
        return GRAPH_PATH
    return f"{GRAPH_PATH}_{ns.replace('/', '_')}_{hashlib.sha1(ns.encode()).hexdigest()[:6]}"


def graph_robot(stage, graph):
    """The articulation an existing graph publishes, from its joint state node's target, as USD has it, so a
    graph saved in a file reads the same. None if that cannot be told."""
    node = stage.GetPrimAtPath(f"{graph}/PublishJointState")
    relationship = node.GetRelationship("inputs:targetPrim") if node else None
    targets = relationship.GetTargets() if relationship else []
    return str(targets[0]) if len(targets) == 1 else None


def namespace_for(robot_path, taken=()):
    """A valid namespace from the robot's prim name, not in ``taken``."""
    name = re.sub(r"[^a-z0-9]+", "_", robot_path.rsplit("/", 1)[-1].lower()).strip("_")
    name = name if name and name[0].isalpha() else f"robot_{name}".rstrip("_")
    candidate, count = name, 1
    while candidate in taken:
        count += 1
        candidate = f"{name}_{count}"
    return candidate


def disable_runtime_graph(og, path):
    """Stop a graph Kit has already loaded. On 6.0.x deactivating its prim is not enough: it still published on Play
    alongside the rebuild, so /joint_states had two publishers. 6.1 stops on deactivation alone."""
    try:
        graph = og.get_graph_by_path(path)
        if graph and graph.is_valid():
            graph.set_disabled(True)
    except AttributeError:  # no such API on this build: deactivation is what's left
        pass


def rebuild_plan(own, graphs, active, owned):
    """Where to build this namespace's graph, and what to clear first. ``graphs`` is every prim beside it and ``active``
    the active ones. A graph this session made is removed. One from an opened file (a saved setup) can only be
    deactivated, which leaves its path taken, so the new one gets a free path. own_rN are earlier rebuilds, and two
    active graphs would publish every message twice."""
    pattern = re.compile(re.escape(own) + r"(_r\d+)?$")
    mine = [path for path in active if pattern.fullmatch(path)]
    remove = [path for path in mine if path in owned]
    deactivate = [path for path in mine if path not in owned]
    taken = set(graphs) - set(remove)
    graph, number = own, 0
    while graph in taken:
        number += 1
        graph = f"{own}_r{number}"
    return dict(graph=graph, remove=remove, deactivate=deactivate)


def check_graph_robot(stage, graph, articulation, robot_path, namespace=""):
    """Return (code, message, retry_with) when this namespace's graph belongs to another robot, else None.
    Rebuilding it would silently drop that robot's frames and joint states. Unreadable counts as another robot."""
    prim = stage.GetPrimAtPath(graph)
    if not prim or not prim.IsActive():  # a saved graph an earlier rebuild deactivated publishes nothing
        return None
    current = graph_robot(stage, graph)
    if current == articulation:
        return None
    candidates, taken = [namespace_for(robot_path)], set()
    while stage.GetPrimAtPath(graph_path(candidates[-1])):
        taken.add(candidates[-1])
        candidates.append(namespace_for(robot_path, taken))
    owner = f"robot {current}" if current else "a robot this tool cannot read from it"
    where = f"namespace {namespace!r}" if namespace else "no namespace"
    return ("ROBOT_GRAPH_CONFLICT", f"{graph} is the ROS 2 graph for {where}, and it already publishes {owner}. Setting up "
            f"{robot_path} there would replace it, and that robot's joint states and frames would disappear. Nothing was "
            f"changed. Each robot needs its own namespace: resend with namespace={candidates[-1]}.",
            dict(namespace=candidates[-1]))


def link_parents(joints, links):
    """{child link: parent link} from (body0, body1, fixed) joints where both sides are this robot's links."""
    paths = {path for path, _ in links}
    return {child: parent for parent, child, _ in joints if parent in paths and child in paths}


def fixed_to_base(joints, links, base_path):
    """The links rigidly joined to the base, through any chain of fixed joints, in link order."""
    paths = {path for path, _ in links}
    group, grew = {base_path}, True
    while grew:
        grew = False
        for one, other, fixed in joints:
            if fixed and one in paths and other in paths and (one in group) != (other in group):
                group |= {one, other}
                grew = True
    return [path for path, _ in links if path in group and path != base_path]


def link_publishing(links, base_path, mode, joints):
    """(links for the live TF node, links for one static edge each), or (None, ()) to give that node the articulation.

    Given an articulation, the node publishes parentPrim -> root link and then the articulation's own tree, and PhysX picks
    the root link. A TurtleBot's base_footprint fixed to base_link was rooted at base_link on 6.2. With parentPrim on any
    other link, both directions of an edge go out, a loop, and Kit then drops odom -> base as a duplicate child. A listed link
    that happens to be the root still brings its whole tree. So a mobile base, or a base with a parent link, publishes the
    links rigidly fixed to it as static edges (the root is one of them on these robots) and lists the rest. A fixed arm rooted at its base keeps the articulation's tree.
    """
    if mode != "mobile" and base_path not in link_parents(joints, links):
        return None, ()
    static = fixed_to_base(joints, links, base_path)
    moving = [path for path, _ in links if path != base_path and path not in static]
    return moving, static


def check_attachment(links, robot_path, parent_path, parent_tf_name, is_body):
    """Return ``(code, message)`` when ``attach_to`` cannot work, else None."""
    if parent_path == robot_path or parent_path.startswith(robot_path.rstrip("/") + "/"):
        return ("ATTACH_LOOP", f"attach_to {parent_path} is inside the robot {robot_path}, so its frame would "
                "depend on itself. Pass a link of the other robot, e.g. the arm's wrist link.")
    if not is_body:
        return ("ATTACH_NOT_A_LINK", f"attach_to {parent_path} is not a rigid body, so physics does not move it. "
                "Pass the link the robot is mounted on, e.g. the arm's wrist link.")
    if parent_tf_name in {n for _, n in links}:
        return ("DUPLICATE_FRAME_NAMES", f"attach_to {parent_path} publishes TF name {parent_tf_name!r}, which a "
                f"link of {robot_path} also uses. Rename one or set isaac:nameOverride.")
    return None


def check_frames(links, base_path, base_frame, base_tf_name, extras, world_frame, mode, link_frames,
                 namespace="", attached=False):
    """Return ``(code, message)`` for a frame layout that would break the TF tree, else None.

    ``links`` and ``extras`` are ``(prim path, TF name)`` pairs; the TF name is what Isaac
    publishes (``isaac:nameOverride`` or the prim name).
    """
    paths = {p for p, _ in links}
    if base_path not in paths:
        return ("BASE_NOT_A_LINK", f"base_link {base_path} is not one of the robot's rigid-body links; "
                "pass its root link (the one the robot is mounted or driven by).")
    if base_frame != base_tf_name and (link_frames == "isaac" or extras or attached):
        # Isaac names the parent of the link TF and of extra frames after the base prim.
        hint = ("With attach_to, Isaac publishes the base prim under its TF name, so base_frame must match it: leave "
                "base_frame empty or set isaac:nameOverride on the prim." if attached and not extras else
                "Extra frames are parented to the base prim's TF name, so base_frame must match it: leave "
                "base_frame empty or set isaac:nameOverride on the prim." if extras else
                "Leave base_frame empty, set isaac:nameOverride on the prim, or use link_frames=robot_state_publisher.")
        return ("BASE_FRAME_MISMATCH", f"base_frame {base_frame!r} differs from the name Isaac publishes for "
                f"{base_path} ({base_tf_name!r}), which would split the tree in two. " + hint)
    names = {}
    for p, n in links:
        names.setdefault(n, []).append(p)
    duplicates = {n: ps for n, ps in names.items() if len(ps) > 1}
    if duplicates and link_frames == "isaac":
        return ("DUPLICATE_FRAME_NAMES", "Several links share a TF name, so their frames would collide: "
                + "; ".join(f"{n}: {', '.join(ps[:3])}" for n, ps in list(duplicates.items())[:5])
                + ". Rename them or set isaac:nameOverride.")
    reserved = {world_frame, base_frame} | ({odom_frame(namespace)} if mode == "mobile" else set())
    seen = {}
    for p, n in extras:
        if n in names or n in reserved or n in seen:
            other = names.get(n, [seen.get(n)])[0] or "a frame this tool publishes"
            return ("DUPLICATE_FRAME_NAMES", f"extra frame {p} would publish TF name {n!r}, already used by {other}.")
        seen[n] = p
    return None


def cmd_vel_graph_args(articulation, wheels, path="/World/CmdVelDrive", topic="cmd_vel"):
    """Arguments for sim.graph.build: /cmd_vel (geometry_msgs/Twist) to a differential base's wheels."""
    return dict(
        path=path,
        nodes=[[TICK, "omni.graph.action.OnPlaybackTick"], [CONTEXT, "isaacsim.ros2.bridge.ROS2Context"],
               ["SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"],
               ["Linear", "omni.graph.nodes.BreakVector3"], ["Angular", "omni.graph.nodes.BreakVector3"],
               ["Differential", "isaacsim.robot.wheeled_robots.DifferentialController"],
               ["Articulation", "isaacsim.core.nodes.IsaacArticulationController"]],
        connections=[[f"{TICK}.outputs:tick", "SubscribeTwist.inputs:execIn"],
                     [f"{CONTEXT}.outputs:context", "SubscribeTwist.inputs:context"],
                     ["SubscribeTwist.outputs:execOut", "Differential.inputs:execIn"],
                     ["SubscribeTwist.outputs:execOut", "Articulation.inputs:execIn"],
                     ["SubscribeTwist.outputs:linearVelocity", "Linear.inputs:tuple"],
                     ["SubscribeTwist.outputs:angularVelocity", "Angular.inputs:tuple"],
                     ["Linear.outputs:x", "Differential.inputs:linearVelocity"],
                     ["Angular.outputs:z", "Differential.inputs:angularVelocity"],
                     ["Differential.outputs:velocityCommand", "Articulation.inputs:velocityCommand"]],
        values={"SubscribeTwist.inputs:topicName": topic,
                "Differential.inputs:wheelRadius": wheels["radius_m"],
                "Differential.inputs:wheelDistance": wheels["separation_m"],
                "Articulation.inputs:jointNames": list(wheels["joints"])},
        targets={"Articulation.inputs:targetPrim": [articulation]},
    )


def scan_graph_args(sensor_path, frame_id, path="/World/LidarScan", topic="scan"):
    """Arguments for sim.graph.build: a LiDAR to /scan through a render product the graph makes at Play.

    Target the OmniLidar itself, which NVIDIA's config assets can nest below the requested path. Aimed at the wrapper, the
    helper only logs "not attached to RTX Lidar". The sensor tools' own IsaacMCP_* product is not used: saves drop it, so a
    graph wired to it goes silent in the reopened file.
    """
    return dict(
        path=path,
        nodes=[[TICK, "omni.graph.action.OnPlaybackTick"], [CONTEXT, "isaacsim.ros2.bridge.ROS2Context"],
               ["RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"],
               ["ScanHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"]],
        connections=[[f"{TICK}.outputs:tick", "RenderProduct.inputs:execIn"],
                     ["RenderProduct.outputs:execOut", "ScanHelper.inputs:execIn"],
                     ["RenderProduct.outputs:renderProductPath", "ScanHelper.inputs:renderProductPath"],
                     [f"{CONTEXT}.outputs:context", "ScanHelper.inputs:context"]],
        values={"ScanHelper.inputs:topicName": topic, "ScanHelper.inputs:type": "laser_scan",
                "ScanHelper.inputs:frameId": frame_id,
                # Matches the clock this tool publishes. Without it, /scan on 5.1 kept counting across Stop and landed
                # 206 s ahead of /clock, and Nav2 drops such messages.
                "ScanHelper.inputs:resetSimulationTimeOnStop": True},
        targets={"RenderProduct.inputs:cameraPrim": [sensor_path]},
    )


# ----------------------------------------------------------------- Kit side

ROS2_PUBLISHERS = {
    "isaacsim.ros2.bridge.ROS2PublishClock": "/clock",
    "isaacsim.ros2.bridge.ROS2PublishTransformTree": "TF",
    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree": "TF",
    "isaacsim.ros2.bridge.ROS2PublishJointState": "joint states",
    "isaacsim.ros2.bridge.ROS2PublishOdometry": "odometry",
}


CLOCK_GRAPH = GRAPH_PATH + "_clock"  # the tool's single /clock, shared by every robot it sets up
PUBLISHER_KINDS = {t.rsplit(".", 1)[-1]: kind for t, kind in ROS2_PUBLISHERS.items()}


def _graphs(og):
    """Every graph, including subgraphs, skipping disabled ones."""
    todo, seen = list(og.get_all_graphs()), set()
    while todo:
        graph = todo.pop()
        path = graph.get_path_to_graph()
        if path in seen:
            continue
        seen.add(path)
        if getattr(graph, "is_disabled", lambda: False)():
            continue
        yield graph
        todo.extend(getattr(graph, "get_subgraphs", lambda: [])())


def other_ros2_publishers(own_graph):
    """ROS 2 clock/TF/joint-state/odometry publisher nodes outside *own_graph*.

    A second /clock makes time jump back and forth (RViz resets constantly), and a second TF or
    joint-state publisher for the same robot makes frames jump, so the caller has to know. Node
    types are matched by their last component, so the older ``omni.isaac.ros2_bridge`` names
    count too. ``tool`` marks graphs this tool built (other robots, or its shared clock).
    """
    import omni.graph.core as og

    found = []
    for graph in _graphs(og):
        path = graph.get_path_to_graph()
        if path == own_graph or path.startswith(own_graph + "/"):
            continue
        for node in graph.get_nodes():
            kind = PUBLISHER_KINDS.get(node.get_type_name().rsplit(".", 1)[-1])
            if kind:
                if kind == "/clock":  # a clock on another topic is not this stage's /clock
                    value = lambda name: (node.get_attribute(name).get() if node.get_attribute_exists(name) else "")
                    topic = value("inputs:topicName")
                    kind = resolved_topic(topic, value("inputs:nodeNamespace")) if topic else kind
                found.append(dict(node=node.get_prim_path(), publishes=kind,
                                  tool=path == GRAPH_PATH or path.startswith(GRAPH_PATH + "_")))
    return found


def resolved_topic(topic, namespace=""):
    """The absolute ROS topic a node's topicName and nodeNamespace give."""
    topic = topic or ""
    if topic.startswith("/"):
        return topic
    ns = (namespace or "").strip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def joint_state_loop(topic, namespace, subscribers):
    """Graph nodes that would drive a robot from this tool's own joint state publisher.

    ``subscribers`` is ``(node path, topicName, nodeNamespace)``. Quick Start's graph mirrors a real robot
    from /joint_states. Publishing the simulated state there makes the articulation chase its own delayed
    position, and it oscillates at its velocity limit as soon as the timeline plays."""
    published = resolved_topic(topic.lstrip("/"), namespace)
    return [node for node, sub_topic, sub_namespace in subscribers
            if resolved_topic(sub_topic, sub_namespace) == published]


# The .ogn defaults, for a node that authored no topic.
_DEFAULT_TOPICS = {"ROS2PublishJointState": "joint_states", "ROS2SubscribeJointState": "joint_command"}
MAX_LOOP_SCAN_PRIMS = 200000


def find_joint_state_loops(nodes):
    """Topics that have both a joint state publisher and subscriber in one stage, whatever the order they
    were made in. ``nodes`` is (node path, node type, topicName or None, nodeNamespace)."""
    topics = {}
    for path, kind, topic, namespace in nodes:
        short = kind.rsplit(".", 1)[-1]
        if short in _DEFAULT_TOPICS:
            name = resolved_topic(topic or _DEFAULT_TOPICS[short], namespace)
            side = "publishers" if short == "ROS2PublishJointState" else "subscribers"
            topics.setdefault(name, dict(topic=name, publishers=[], subscribers=[]))[side].append(path)
    return [entry for _, entry in sorted(topics.items()) if entry["publishers"] and entry["subscribers"]]


def stage_graph_nodes(stage, limit=MAX_LOOP_SCAN_PRIMS):
    """OmniGraph nodes as USD has them, so a saved file is seen without waiting for its graphs to load."""
    from pxr import Usd

    found = []
    for count, prim in enumerate(Usd.PrimRange(stage.GetPseudoRoot())):
        if count >= limit:
            break
        if prim.GetTypeName() == "OmniGraphNode":
            kind = prim.GetAttribute("node:type").Get()
            value = lambda name: (prim.GetAttribute(name).Get() if prim.HasAttribute(name) else None)
            found.append((str(prim.GetPath()), str(kind or ""), value("inputs:topicName"), value("inputs:nodeNamespace")))
    return found


def joint_state_loop_report(stage):
    """For sim.stage.open: a result field when this file contains the loop, else nothing. Changes nothing."""
    loops = find_joint_state_loops(stage_graph_nodes(stage))
    if not loops:
        return {}
    graph = loops[0]["publishers"][0].rsplit("/", 1)[0]  # sim.prim.delete acts on a graph, not one of its nodes
    return dict(joint_state_loop=dict(
        loops=loops,
        effect="The subscriber drives a robot from the topic this simulation publishes its own joint state on. If they "
               "are the same robot, it chases its own state and oscillates at its velocity limit once playing.",
        fix=f"The file was not changed. On a stopped timeline, call sim__prim__delete with path={graph}, allow_foreign=true and "
            "confirm=true: it deactivates that whole graph in the session layer, so its TF stops too. To publish TF and joint "
            "states again, call sim__ros2_tf with a namespace. sim__stage__save then writes a copy without the loop.",
    ))


def joint_state_subscribers(own_graph):
    """ROS 2 joint state subscribers outside *own_graph*: (node path, topicName, nodeNamespace)."""
    import omni.graph.core as og

    found = []
    for graph in _graphs(og):
        path = graph.get_path_to_graph()
        if path == own_graph or path.startswith(own_graph + "/"):
            continue
        for node in graph.get_nodes():
            if node.get_type_name().rsplit(".", 1)[-1] == "ROS2SubscribeJointState":
                value = lambda name: (node.get_attribute(name).get() if node.get_attribute_exists(name) else "")
                found.append((node.get_prim_path(), value("inputs:topicName"), value("inputs:nodeNamespace")))
    return found


def clock_resets_on_stop(clock_node_path):
    """resetOnStop of the IsaacReadSimulationTime feeding a /clock publisher, or None if unknown."""
    import omni.graph.core as og

    try:
        attr = og.Controller.attribute(clock_node_path + ".inputs:timeStamp")
        for upstream in attr.get_upstream_connections():
            node = upstream.get_node()
            if node.get_type_name().rsplit(".", 1)[-1] == "IsaacReadSimulationTime":
                return bool(node.get_attribute("inputs:resetOnStop").get())
    except Exception:
        return None
    return None


def clock_graph_plan():
    """Nodes, values and connections of the shared clock graph (NVIDIA's create_ros2_clock_graph)."""
    nodes = [(TICK, "omni.graph.action.OnPlaybackTick"), (CONTEXT, "isaacsim.ros2.bridge.ROS2Context"),
             (TIME, "isaacsim.core.nodes.IsaacReadSimulationTime"), (CLOCK, "isaacsim.ros2.bridge.ROS2PublishClock")]
    values = [(f"{TIME}.inputs:resetOnStop", True), (f"{CLOCK}.inputs:topicName", "clock")]
    connects = [(f"{TICK}.outputs:tick", f"{CLOCK}.inputs:execIn"),
                (f"{CONTEXT}.outputs:context", f"{CLOCK}.inputs:context"),
                (f"{TIME}.outputs:simulationTime", f"{CLOCK}.inputs:timeStamp")]
    return nodes, values, connects


def plan_clock(mode, publishers):
    """Decide the /clock for this call. Returns dict(publish, remove_tool_clock, reset_on_stop, source).

    ``publishers`` is other_ros2_publishers() output. The tool keeps its clock in one shared graph
    (CLOCK_GRAPH), never in a robot's graph, so deleting one robot never removes the clock.
    """
    if mode not in ("auto", "publish", "skip"):
        raise ValueError("clock must be auto, publish or skip")
    foreign = [o for o in publishers if o["publishes"] == "/clock" and not o["tool"]]
    tool_clock = [o for o in publishers if o["publishes"] == "/clock" and o["tool"]]
    if mode == "publish" or (mode == "auto" and not foreign):
        return dict(publish=True, remove_tool_clock=False, reset_on_stop=True, source=CLOCK_GRAPH + "/" + CLOCK,
                    foreign=[o["node"] for o in foreign])
    source = foreign[0]["node"] if foreign else (tool_clock[0]["node"] if tool_clock else None)
    return dict(publish=False, remove_tool_clock=bool(mode == "auto" and foreign and tool_clock), reset_on_stop=None,
                source=source, foreign=[o["node"] for o in foreign])

NAME_OVERRIDE = "isaac:nameOverride"


def extra_frame_parents(extras, link_paths, base_path, base_frame, mode):
    """Split extra frames into those a transform tree publishes and those published once as a static transform.

    A transform tree logs "[PoseTree] target getObjectType eInvalid" on every tick for a prim that isn't a rigid body, which
    buried Kit's log (24,135 of 24,547 lines). A sensor mounted below a link doesn't move relative to it, so its pose is known
    now. Returns (tree paths, [(path, name, parent prim path, parent frame)]). A prim that can move relative to every link,
    on a mobile robot, stays on the tree and keeps the warning.
    """
    links = set(link_paths)
    tree, static = [], []
    for path, name in extras:
        ancestor = path.rsplit("/", 1)[0]
        while ancestor and ancestor not in links and ancestor != base_path:
            ancestor = ancestor.rsplit("/", 1)[0]
        if path in links:
            tree.append(path)
        elif ancestor:
            static.append((path, name, ancestor, None if ancestor == base_path else ancestor.rsplit("/", 1)[-1]))
        elif mode == "fixed":
            static.append((path, name, base_path, None))
        else:
            tree.append(path)
    return tree, [(path, name, parent, frame or base_frame) for path, name, parent, frame in static]


def tf_name(prim):
    """The frame name Isaac publishes for *prim*: its isaac:nameOverride, else its prim name."""
    attr = prim.GetAttribute(NAME_OVERRIDE)
    value = attr.Get() if attr and attr.HasAuthoredValue() else None
    return value if isinstance(value, str) and value else prim.GetName()

def _links(stage, robot_prim, limit=512):
    """Rigid-body link prims below *robot_prim* (bounded), as (path, name)."""
    from pxr import Usd, UsdPhysics

    found = []
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            found.append((str(prim.GetPath()), tf_name(prim)))
            if len(found) >= limit:
                break
    return found


def _joint_bodies(robot_prim, limit=4000):
    """(body0, body1, fixed) of the joints below *robot_prim*; an empty side means the world."""
    from pxr import Usd, UsdPhysics

    found = []
    for count, prim in enumerate(Usd.PrimRange(robot_prim, Usd.TraverseInstanceProxies())):
        if count >= limit:
            break
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            side = [joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()]
            found.append((*(str(targets[0]) if targets else "" for targets in side), prim.IsA(UsdPhysics.FixedJoint)))
    return found


def _articulation_roots(robot_prim, limit=2000):
    """Prims at or below *robot_prim* that carry the ArticulationRootAPI (often a fixed root
    joint such as ``<robot>/root_joint``, not the robot's top prim). The TF, joint-state and
    odometry nodes need this exact prim: given only the top prim, the TF node publishes that one
    prim instead of the whole link tree."""
    from pxr import Usd, UsdPhysics

    found = []
    for i, prim in enumerate(Usd.PrimRange(robot_prim)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            found.append(str(prim.GetPath()))
        if i >= limit:
            break
    return found


def _relative_pose(stage, base, path, mpu):
    """(translation in metres, (qx, qy, qz, qw)) of the prim at *path* in the base's frame, from the stage."""
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    relative = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)) * cache.GetLocalToWorldTransform(base).GetInverse()
    t = relative.ExtractTranslation() * mpu
    q = relative.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
    return (t[0], t[1], t[2]), (*q.GetImaginary(), q.GetReal())


def setup(scene, stage, a):
    """Build (or rebuild) the graph. The timeline is stopped (checked by Scene.target)."""
    from twinsmith.project_assets import refuse
    from pxr import Usd, UsdGeom

    robot_path = a["robot"]
    robot = scene.prim(stage, robot_path)
    roots = _articulation_roots(robot)
    if not roots:
        refuse("NOT_ARTICULATION", f"{robot_path} has no articulation root at or below it; pass the robot's root prim.")
    if len(roots) > 1:
        refuse("SEVERAL_ARTICULATIONS", f"{robot_path} contains {len(roots)} articulations ({', '.join(roots[:4])}); "
               "pass a prim that contains one of them; publish frames of the others with extra_frames.")
    articulation = roots[0]
    links = _links(stage, robot)
    if not links:
        refuse("NO_LINKS", f"No rigid-body links found below {robot_path}.")
    base_path = a.get("base_link") or ""
    if base_path and not base_path.startswith("/"):  # a link's name, as BASE_LINK_REQUIRED lists them
        matches = [p for p, n in links if n == base_path]
        if len(matches) != 1:
            refuse("BASE_LINK_REQUIRED", f"base_link {base_path!r} matches {len(matches)} links. Pass its absolute prim path, "
                   "one of: " + ", ".join(p for p, _ in links[:12]))
        base_path = matches[0]
    if not base_path:
        by_name = {n: p for p, n in reversed(links)}
        named = [by_name[n] for n in ("base_link", "base_footprint", "base") if n in by_name]
        if not named:
            refuse("BASE_LINK_REQUIRED", "Pass base_link: the robot's root link prim. Links found: "
                   + ", ".join(n for _, n in links[:12]))
        base_path = named[0]
    base = scene.prim(stage, base_path)
    base_tf_name = tf_name(base)
    link_mode = a.get("link_frames", "isaac")
    base_frame = a.get("base_frame") or base_tf_name

    extra = list(a.get("extra_frames") or [])
    if len(extra) > 16:
        refuse("LIMIT", "At most 16 extra_frames.")
    extras = [(p, tf_name(scene.prim(stage, p))) for p in extra]
    attach_path = a.get("attach_to") or ""
    attach_name = None
    if attach_path:
        attach_prim = scene.prim(stage, attach_path)
        attach_name = tf_name(attach_prim)
        from pxr import UsdPhysics

        problem = check_attachment(links, robot_path, attach_path, attach_name,
                                   attach_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        if problem:
            refuse(*problem)
    try:
        problem = check_frames(links, base_path, base_frame, base_tf_name, extras,
                               a.get("world_frame", "world"), a.get("base", "fixed"), link_mode,
                               a.get("namespace", ""), attached=bool(attach_path))
    except ValueError as exc:
        refuse("INVALID_VALUE", str(exc))
    if problem:
        refuse(*problem)

    world = UsdGeom.XformCache().GetLocalToWorldTransform(base)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    t = world.ExtractTranslation() * mpu
    q = world.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
    pose = ((t[0], t[1], t[2]), (q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()))
    try:
        own = graph_path(a.get("namespace", ""))
    except ValueError as exc:
        refuse("INVALID_VALUE", str(exc))
    state_topic = a.get("joint_states_topic", "joint_states")
    loop = joint_state_loop(state_topic, a.get("namespace", ""), joint_state_subscribers(own))
    if loop:
        refuse("JOINT_STATE_LOOP", f"{', '.join(loop)} drives a robot from the topic this tool would publish joint "
               f"states on ({resolved_topic(state_topic.lstrip('/'), a.get('namespace', ''))}), so the robot would chase its "
               "own state and oscillate at its velocity limit once playing. Nothing was changed. Publish elsewhere, "
               "e.g. joint_states_topic=isaac/joint_states; TF does not depend on this topic.",
               subscribers=loop, retry_with=dict(joint_states_topic="isaac/joint_states"))
    others = other_ros2_publishers(own)
    try:
        clock = plan_clock(a.get("clock", "auto"), others)
    except ValueError as exc:
        refuse("INVALID_VALUE", str(exc))
    clock_warnings = []
    reset_on_stop = clock["reset_on_stop"]
    if clock["publish"] and stage.GetPrimAtPath(CLOCK_GRAPH) and CLOCK_GRAPH not in scene.owned:
        reset_on_stop = None  # an existing tool clock from before a reload: match what it does
        clock["source"] = CLOCK_GRAPH + "/" + CLOCK
    if reset_on_stop is None:
        reset_on_stop = clock_resets_on_stop(clock["source"]) if clock["source"] else None
        if reset_on_stop is None:
            reset_on_stop = True
            if clock["source"]:
                clock_warnings.append(f"Could not read how {clock['source']} counts time across stop/play; TF "
                                      "assumes it restarts at 0. Run the doctor after a stop/play.")
    moving, fixed_links = link_publishing(links, base_path, a.get("base", "fixed"), _joint_bodies(robot))
    static_links = [(tf_name(stage.GetPrimAtPath(path)), *_relative_pose(stage, base, path, mpu)) for path in fixed_links]
    tree_extras, mounted = extra_frame_parents(extras, [path for path, _ in links], base_path, base_frame, a.get("base", "fixed"))
    static_frames = [(frame, name, *_relative_pose(stage, stage.GetPrimAtPath(parent), path, mpu))
                     for path, name, parent, frame in mounted]
    try:
        nodes, values, connects, frames = planned_graph(
            robot=articulation, base_prim=base_path, base_frame=base_frame, base_pose=pose,
            mode=a.get("base", "fixed"), world_frame=a.get("world_frame", "world"),
            link_frames=a.get("link_frames", "isaac"), extra_prims=tree_extras, static_frames=static_frames,
            namespace=a.get("namespace", ""), joint_states_topic=a.get("joint_states_topic", "joint_states"),
            base_tf_name=base_tf_name, publish_clock=False, reset_on_stop=reset_on_stop,
            attach_parent=attach_path or None, attach_parent_frame=attach_name,
            link_prims=moving, static_links=static_links,
        )
    except ValueError as exc:
        refuse("INVALID_VALUE", str(exc))

    import omni.graph.core as og

    graph = own
    had_scope = bool(stage.GetPrimAtPath("/Graph"))
    existing = bool(stage.GetPrimAtPath(graph))  # read before removal: a removed prim is falsy
    conflict = check_graph_robot(stage, graph, articulation, robot_path, a.get("namespace", ""))
    if conflict:
        refuse(conflict[0], conflict[1], current_robot=graph_robot(stage, graph), graph=graph, retry_with=conflict[2])
    # This namespace's graphs, including a rebuild of a saved one. check_graph_robot has shown the main one is this robot's.
    parent = stage.GetPrimAtPath(own.rsplit("/", 1)[0])
    children = list(parent.GetAllChildren()) if parent else []
    plan = rebuild_plan(own, [str(p.GetPath()) for p in children], [str(p.GetPath()) for p in children if p.IsActive()], scene.owned)
    graph = plan["graph"]
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in plan["remove"]:
            stage.RemovePrim(path)
        for path in plan["deactivate"]:  # from an opened file: a session edit can't delete it, and the file stays as it is
            disable_runtime_graph(og, path)
            stage.GetPrimAtPath(path).SetActive(False)
    from pxr import Sdf

    keys = og.Controller.Keys
    # Relationship ("target") inputs take a path string, or a list of Sdf.Path for several targets,
    # exactly as NVIDIA's graph_api sets them.
    set_values = [
        (attr, [Sdf.Path(v) for v in value] if attr.endswith("targetPrims") else value)
        for attr, value in values
    ]
    try:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            og.Controller.edit(
                {"graph_path": graph, "evaluator_name": "execution"},
                {keys.CREATE_NODES: nodes, keys.CONNECT: connects, keys.SET_VALUES: set_values},
            )
    except Exception as exc:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if stage.GetPrimAtPath(graph):
                stage.RemovePrim(graph)
            scope = stage.GetPrimAtPath("/Graph")
            if scope and not had_scope and not scope.GetChildren():
                stage.RemovePrim("/Graph")
        scene.owned.discard(graph)
        refuse("ROS2_GRAPH_FAILED", f"Could not build the ROS 2 graph: {type(exc).__name__}: {str(exc)[:300]}. "
               "If a node type is unknown, launch Kit with isaacsim.ros2.bridge enabled (run.sh does). "
               + ("The previous graph at this path was removed; call the tool again once fixed."
                  if existing else "Nothing was left behind."))
    scene.owned.add(graph)

    # The shared clock graph: created once, removed when another (non-tool) clock takes over.
    clock_existing = bool(stage.GetPrimAtPath(CLOCK_GRAPH))
    if clock["remove_tool_clock"] and clock_existing and CLOCK_GRAPH in scene.owned:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.RemovePrim(CLOCK_GRAPH)
        scene.owned.discard(CLOCK_GRAPH)
    elif clock["publish"] and not clock_existing:
        c_nodes, c_values, c_connects = clock_graph_plan()
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                og.Controller.edit({"graph_path": CLOCK_GRAPH, "evaluator_name": "execution"},
                                   {keys.CREATE_NODES: c_nodes, keys.CONNECT: c_connects, keys.SET_VALUES: c_values})
            scene.owned.add(CLOCK_GRAPH)
        except Exception as exc:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                if stage.GetPrimAtPath(CLOCK_GRAPH):
                    stage.RemovePrim(CLOCK_GRAPH)
            clock_warnings.append(f"Could not build {CLOCK_GRAPH} ({type(exc).__name__}); nothing publishes /clock.")
    elif clock["remove_tool_clock"] and clock_existing:
        clock_warnings.append(f"Two /clock publishers: {CLOCK_GRAPH} (not created in this session, so it was not "
                              f"removed) and {', '.join(clock['foreign'])}. Delete {CLOCK_GRAPH}; RViz keeps "
                              "resetting while both run.")
    elif clock["publish"] and clock_existing and CLOCK_GRAPH not in scene.owned:
        clock_warnings.append(f"{CLOCK_GRAPH} exists but was not created in this session; it was left as is.")
    # Every robot graph this session owns must stamp with the same stop/play behaviour as /clock;
    # when the clock source changes, the robots set up earlier are updated too.
    updated = []
    for other in sorted(g for g in scene.owned if g != CLOCK_GRAPH and g != graph
                        and (g == GRAPH_PATH or g.startswith(GRAPH_PATH + "_"))):
        if not stage.GetPrimAtPath(other):  # deleted outside the tool (UI, run_python): forget it
            scene.owned.discard(other)
            continue
        try:
            attr = og.Controller.attribute(f"{other}/{TIME}.inputs:resetOnStop")
            if bool(attr.get()) != bool(reset_on_stop):
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    og.Controller.set(attr, bool(reset_on_stop))
                updated.append(other)
        except Exception as exc:
            clock_warnings.append(f"Could not update {other} to the current clock ({type(exc).__name__}); "
                                  "call this tool again for that robot.")
    if clock["publish"] and clock["foreign"]:
        clock_warnings.append(f"{len(clock['foreign']) + 1} /clock publishers ({', '.join(clock['foreign'])} and "
                              f"{CLOCK_GRAPH}): time jumps back and RViz keeps resetting. Use clock=auto.")

    ns = ("/" + a.get("namespace").strip("/")) if (a.get("namespace") or "").strip("/") else ""
    topics = ["/clock", "/tf", "/tf_static", ns + "/" + a.get("joint_states_topic", "joint_states").lstrip("/")]
    if a.get("base", "fixed") == "mobile":
        topics.append(f"{ns}/odom")
    fixed = frames["root"]
    rsp = a.get("link_frames", "isaac") == "robot_state_publisher"
    return dict(
        graph=graph,
        robot=robot_path,
        articulation_root=articulation,
        base_link=base_path,
        base_frame=base_frame,
        frames=frames,
        link_count=len(links),
        topics=topics,
        ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0 (unset)"),
        rviz_fixed_frame=fixed,
        publishes_when="timeline is playing (OnPlaybackTick); stamps use simulation time",
        next_steps=[
            "Play the timeline (sim__timeline action=play).",
            "Every ROS 2 node, RViz included, must use use_sim_time:=true and the same ROS_DOMAIN_ID.",
            f"python3 scripts/ros2_rviz_config.py --fixed-frame {fixed} --out <file>.rviz "
            + ("--robot-model " if rsp else "") + "then: rviz2 -d <file>.rviz --ros-args -p use_sim_time:=true",
            f"python3 scripts/ros2_tf_doctor.py --fixed-frame {fixed}  (ROS 2 sourced, or add --isaac-ros) "
            "before trusting RViz",
        ] + ([
            f"{attach_name}->{base_frame} is published on every tick and follows the parent. Set up the robot that "
            f"owns {attach_name} too, with this tool and a different namespace from this call (one namespace holds one "
            f"robot), so {attach_name} connects to {fixed}. Do not also give "
            f"{base_frame} a world link: a frame with two parents makes RViz jump.",
        ] if attach_path else [
            "The world->base (or world->odom) transform is the pose at setup time: stop and call this tool "
            "again after moving the robot.",
        ]) + ([
            "Run robot_state_publisher with the robot's URDF and use_sim_time:=true; it turns /joint_states "
            "into the link frames. Isaac does not publish them in this mode, so nothing is published twice. "
            f"base_frame ({base_frame!r}) must be the URDF's root link.",
        ] if rsp else [
            "Do NOT also run robot_state_publisher for this robot: Isaac already publishes its link frames, "
            "and two publishers for the same frames make RViz jump. Use link_frames=robot_state_publisher instead "
            "if you need a URDF RobotModel in RViz.",
        ]) + ([
            f"Do not run AMCL/SLAM with world_frame='map': they publish map->{frames['odom']} themselves. Use "
            f"world_frame='' (Isaac publishes only {frames['odom']}->base) when a localization node provides it, "
            f"and set that node's odom_frame_id to {frames['odom']}.",
        ] if a.get("base", "fixed") == "mobile" else []),
        rebuilt=existing,
        **({"deactivated_saved_graphs": plan["deactivate"]} if plan["deactivate"] else {}),
        **({"attached_to": dict(prim=attach_path, frame=attach_name)} if attach_path else {}),
        clock=dict(source=clock["source"] or "none", shared_tool_clock=clock["publish"],
                   tool_clock_present=bool(stage.GetPrimAtPath(CLOCK_GRAPH)),
                   stamps_reset_on_stop=reset_on_stop, robot_graphs_updated=updated,
                   **({"note": "Every robot set up by this tool shares this clock. Running the work cell's own setup "
                               "(quick_start) afterwards adds a second /clock: call this tool again "
                               "(clock=auto); it drops its clock and updates every robot it set up."}
                      if CLOCK_GRAPH in scene.owned else {})),
        other_ros2_publishers=others,
        warnings=clock_warnings + ([
            "Other graphs also publish TF or joint states ("
            + ", ".join(o["node"] for o in others if o["publishes"] != "/clock" and not o["tool"])
            + "). If they cover this robot, its frames are published twice and RViz jumps: delete those "
            "nodes or do not use this tool for that robot."
        ] if any(o["publishes"] != "/clock" and not o["tool"] for o in others) else []),
    )
