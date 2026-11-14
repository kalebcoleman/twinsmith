"""ROS 2 / RViz support: sim.ros2_tf graph layout, the RViz config writer and the TF doctor."""

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, REPO / rel)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ros2 = _load("sim_tools_ros2_under_test", "exts/sim-tools/sim_tools/ros2.py")
rviz = _load("ros2_rviz_config_under_test", "scripts/ros2_rviz_config.py")
doctor = _load("ros2_tf_doctor_under_test", "scripts/ros2_tf_doctor.py")

POSE = ((0.1, 0.2, 0.0), (0.0, 0.0, 0.0, 1.0))


def plan(**kw):
    args = dict(robot="/World/ur5e", base_prim="/World/ur5e/base_link", base_frame="base_link", base_pose=POSE)
    args.update(kw)
    return ros2.planned_graph(**args)


def _check_wiring(nodes, values, connects):
    names = {n for n, _ in nodes}
    assert len(names) == len(nodes), "node names must be unique"
    for src, dst in connects:
        assert src.split(".")[0] in names and dst.split(".")[0] in names, (src, dst)
    for attr, _ in values:
        assert attr.split(".")[0] in names, attr
    types = dict(nodes)
    publishers = [n for n, t in nodes if t.startswith("isaacsim.ros2.bridge.ROS2Publish")]
    for name in publishers:
        assert (f"Context.outputs:context", f"{name}.inputs:context") in connects
        assert (f"ReadSimTime.outputs:simulationTime", f"{name}.inputs:timeStamp") in connects
        assert any(d == f"{name}.inputs:execIn" for _, d in connects), f"{name} never ticks"
    return types


def test_fixed_arm_publishes_clock_joint_states_static_base_and_links():
    nodes, values, connects, frames = plan()
    types = _check_wiring(nodes, values, connects)
    v = dict(values)
    assert types["PublishClock"] == "isaacsim.ros2.bridge.ROS2PublishClock"
    assert v["TFWorldToBase.inputs:parentFrameId"] == "world"
    assert v["TFWorldToBase.inputs:childFrameId"] == "base_link"
    assert v["TFWorldToBase.inputs:staticPublisher"] is True
    assert v["TFWorldToBase.inputs:topicName"] == "tf_static"
    assert v["TFWorldToBase.inputs:translation"] == [0.1, 0.2, 0.0]
    assert v["TFWorldToBase.inputs:rotation"] == [0.0, 0.0, 0.0, 1.0]  # IJKR
    assert v["TFRobotLinks.inputs:parentPrim"] == "/World/ur5e/base_link"
    assert v["TFRobotLinks.inputs:targetPrims"] == ["/World/ur5e"]
    assert v["PublishJointState.inputs:topicName"] == "joint_states"
    assert "ComputeOdometry" not in types
    assert frames["root"] == "world"


def test_mobile_base_uses_odometry_and_a_static_world_to_odom():
    nodes, values, connects, frames = plan(mode="mobile", world_frame="map")
    types = _check_wiring(nodes, values, connects)
    v = dict(values)
    assert types["ComputeOdometry"] == "isaacsim.core.nodes.IsaacComputeOdometry"
    assert (v["TFWorldToOdom.inputs:parentFrameId"], v["TFWorldToOdom.inputs:childFrameId"]) == ("map", "odom")
    assert v["TFOdomToBase.inputs:staticPublisher"] is False
    assert ("ComputeOdometry.outputs:position", "TFOdomToBase.inputs:translation") in connects
    # Odometry outputs drive the odom->base TF; the playback tick does not fire it a second time.
    assert ("OnPlaybackTick.outputs:tick", "TFOdomToBase.inputs:execIn") not in connects
    assert v["PublishOdometry.inputs:chassisFrameId"] == "base_link"
    assert frames["root"] == "map"


def test_mobile_without_world_frame_leaves_map_to_odom_to_localization():
    nodes, *_ = plan(mode="mobile", world_frame="")
    assert "TFWorldToOdom" not in dict(nodes)


def test_robot_state_publisher_mode_publishes_no_link_frames():
    nodes, values, connects, frames = plan(link_frames="robot_state_publisher")
    assert "TFRobotLinks" not in dict(nodes)
    assert "PublishJointState" in dict(nodes)
    assert "robot_state_publisher" in frames["links"]


def test_extra_frames_are_published_relative_to_the_base():
    nodes, values, *_ = plan(extra_prims=["/World/cam", "/World/lidar"])
    v = dict(values)
    assert v["TFExtraFrames.inputs:targetPrims"] == ["/World/cam", "/World/lidar"]
    assert v["TFExtraFrames.inputs:parentPrim"] == "/World/ur5e/base_link"


def test_namespace_applies_to_joint_states_and_odometry_but_tf_stays_global():
    nodes, values, *_ = plan(namespace="/robot1/", mode="mobile")  # slashes are accepted
    v = dict(values)
    assert v["PublishJointState.inputs:nodeNamespace"] == "robot1"
    assert v["PublishOdometry.inputs:nodeNamespace"] == "robot1"
    for name, t in nodes:
        if "TransformTree" in t:
            assert f"{name}.inputs:nodeNamespace" not in v, name  # /tf, /tf_static for RViz and tf2


def test_each_namespace_gets_its_own_graph():
    assert ros2.graph_path("") == "/Graph/twinsmith_ros2"
    assert ros2.graph_path("a/b") != ros2.graph_path("a")
    assert ros2.graph_path("a/b") != ros2.graph_path("a_b")  # one-to-one
    assert ros2.graph_path("/robot1/").startswith("/Graph/twinsmith_ros2_robot1_")


@pytest.mark.parametrize("kw", [
    dict(base_frame="base link"), dict(base_frame="1base"), dict(world_frame="base_link"),
    dict(mode="mobile", world_frame="odom"), dict(mode="wheeled"), dict(link_frames="both"),
    dict(namespace="bad ns"), dict(base_pose=((float("nan"), 0, 0), (0, 0, 0, 1))),
])
def test_invalid_layouts_are_refused(kw):
    with pytest.raises(ValueError):
        plan(**kw)


def test_the_tool_contract_is_registered():
    sys.path.insert(0, str(REPO / "mcp"))
    from twinsmith.scene_contracts import REGISTRY

    params = REGISTRY["sim.ros2_tf"]["parameters"]
    assert {"session_id", "stage_id", "robot", "base", "world_frame", "link_frames", "extra_frames"} <= set(params)
    assert params["robot"]["required"] and not params["base"]["required"]


# --------------------------------------------------------------------- RViz config

def test_rviz_config_has_the_fixed_frame_and_one_display_per_topic(tmp_path):
    out = tmp_path / "lab.rviz"
    assert rviz.main(["--fixed-frame", "world", "--out", str(out), "--robot-model",
                      "--image", "/camera/rgb", "--scan", "/scan", "--cloud", "/points", "--odom", "/odom"]) == 0
    text = out.read_text()
    assert 'Fixed Frame: "world"' in text
    for cls in ("TF", "Grid", "RobotModel", "Image", "LaserScan", "PointCloud2", "Odometry"):
        assert f"rviz_default_plugins/{cls}" in text
    for topic in ("/camera/rgb", "/scan", "/points", "/odom", "/robot_description"):
        assert f'Value: "{topic}"' in text
    yaml = pytest.importorskip("yaml")
    doc = yaml.safe_load(text)
    vm = doc["Visualization Manager"]
    assert vm["Global Options"]["Fixed Frame"] == "world"
    assert len(vm["Displays"]) == 2 + 1 + 4


def test_rviz_config_refuses_bad_input(tmp_path, capsys):
    out = tmp_path / "x.rviz"
    for argv in (["--fixed-frame", "my frame", "--out", str(out)],
                 ["--fixed-frame", "world", "--out", str(out), "--scan", "scan"],
                 ["--fixed-frame", "world", "--out", str(tmp_path / "x.yaml")]):
        with pytest.raises(SystemExit):
            rviz.main(argv)
    out.write_text("mine")
    with pytest.raises(SystemExit):
        rviz.main(["--fixed-frame", "world", "--out", str(out)])
    assert out.read_text() == "mine"


# ------------------------------------------------------------------------ TF doctor

def e(parent, child, stamp=5.0, static=False):
    return dict(parent=parent, child=child, static=static, stamp=stamp)


GOOD = [e("world", "base_link", 0, True), e("base_link", "shoulder_link"), e("shoulder_link", "tool0")]


def test_a_clean_tree_has_no_errors():
    errors, warnings, info = doctor.analyze(GOOD, "world", clock_seen=True)
    assert errors == [] and info["roots"] == ["world"] and info["time_source"] == ["sim"]


def test_two_parents_is_the_flicker_error_and_names_the_publishers():
    edges = GOOD + [e("world", "tool0")]
    errors, warnings, _ = doctor.analyze(edges, "world", clock_seen=True,
                                         tf_publishers=["/isaac_sim", "/robot_state_publisher"])
    assert any("'tool0' has 2 parents" in x for x in errors)
    assert any("robot_state_publisher" in w for w in warnings)


def test_static_and_dynamic_for_one_frame_is_an_error():
    errors, *_ = doctor.analyze(GOOD + [e("world", "base_link", 5.0, False)], "world", clock_seen=True)
    assert any("both /tf and /tf_static" in x for x in errors)


def test_frames_that_do_not_reach_the_fixed_frame_are_reported():
    edges = GOOD + [e("odom", "base_footprint")]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True)
    assert any("do not connect to 'world'" in x and "odom" in x for x in errors)


def test_missing_fixed_frame():
    errors, *_ = doctor.analyze(GOOD, "map", clock_seen=True)
    assert any("'map' is not in the tree" in x for x in errors)


def test_mixed_time_sources_and_missing_clock():
    mixed = GOOD + [e("base_link", "camera_link", stamp=1.79e9)]
    errors, warnings, _ = doctor.analyze(mixed, "world", clock_seen=True)
    assert any("mixed" in x and "camera_link" in x for x in errors)
    assert not any("not updated" in w for w in warnings)  # wall stamps do not make sim frames stale
    errors, *_ = doctor.analyze(GOOD, "world", clock_seen=False)
    assert any("nothing publishes /clock" in x for x in errors)


def test_no_data_says_check_playing_and_domain():
    errors, *_ = doctor.analyze([], "world", clock_seen=False)
    assert any("ROS_DOMAIN_ID" in x for x in errors)


def test_doctor_without_ros_exits_2(monkeypatch, capsys):
    def no_ros(seconds, topics=()):
        raise ImportError("No module named 'rclpy'")

    monkeypatch.setattr(doctor, "capture", no_ros)
    assert doctor.main(["--fixed-frame", "world"]) == 2
    assert "Source ROS 2" in capsys.readouterr().err


def test_a_frame_that_stopped_updating_is_reported():
    edges = GOOD + [e("base_link", "camera_link", stamp=1.0), e("base_link", "shoulder_link", stamp=9.0)]
    _, warnings, _ = doctor.analyze(edges, "world", clock_seen=True)
    assert any("not updated" in w and "camera_link" in w for w in warnings)


# --------------------------------------------------------------- frame-name checks

LINKS = [("/r/base_link", "base_link"), ("/r/shoulder", "shoulder"), ("/r/wrist", "wrist")]


def check(**kw):
    args = dict(links=LINKS, base_path="/r/base_link", base_frame="base_link", base_tf_name="base_link",
                extras=[], world_frame="world", mode="fixed", link_frames="isaac")
    args.update(kw)
    return ros2.check_frames(**args)


def test_a_consistent_layout_passes():
    assert check() is None
    assert check(extras=[("/World/cam", "cam"), ("/r/wrist/tool0", "tool0")]) is None


def test_base_frame_override_that_splits_the_tree_is_refused():
    # Isaac names the links' parent after the prim, so a different static child splits the tree.
    assert check(base_frame="base_footprint")[0] == "BASE_FRAME_MISMATCH"
    assert check(base_frame="base_footprint", link_frames="robot_state_publisher") is None


def test_base_must_be_one_of_the_links():
    assert check(base_path="/r/flange")[0] == "BASE_NOT_A_LINK"


def test_link_and_extra_frame_name_collisions_are_refused():
    assert check(links=LINKS + [("/r/other/wrist", "wrist")])[0] == "DUPLICATE_FRAME_NAMES"
    assert check(links=LINKS + [("/r/other/wrist", "wrist")], link_frames="robot_state_publisher") is None
    for extras in ([("/a/Camera", "Camera"), ("/b/Camera", "Camera")], [("/x/wrist", "wrist")],
                   [("/x/world", "world")], [("/x/base_link", "base_link")]):
        assert check(extras=extras)[0] == "DUPLICATE_FRAME_NAMES", extras
    assert check(extras=[("/x/odom", "odom")], mode="mobile")[0] == "DUPLICATE_FRAME_NAMES"


def test_tf_name_uses_isaac_name_override():
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import Sdf

    stage = Usd.Stage.CreateInMemory()
    plain = stage.DefinePrim("/r/link_a")
    renamed = stage.DefinePrim("/r/link_b")
    renamed.CreateAttribute(ros2.NAME_OVERRIDE, Sdf.ValueTypeNames.String).Set("left_link")
    assert ros2.tf_name(plain) == "link_a" and ros2.tf_name(renamed) == "left_link"


def test_doctor_reports_a_tf_loop():
    edges = [e("world", "base_link", 0, True), e("base_link", "world")]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True)
    assert any("TF loop" in x for x in errors)


def test_doctor_names_publishers_for_tf_and_tf_static_duplicates():
    edges = GOOD + [e("world", "base_link", 5.0, False)]
    _, warnings, _ = doctor.analyze(edges, "world", clock_seen=True, tf_publishers=["/a", "/b"])
    assert any("several nodes publish TF" in w for w in warnings)


def test_isaac_ros_env_finds_the_5_1_bridge_folder(tmp_path, monkeypatch):
    import subprocess as sp

    root = tmp_path / "isaac-sim"
    python = root / "python.sh"
    (root / "exts" / "isaacsim.ros2.bridge" / "humble" / "rclpy").mkdir(parents=True)
    real = sp.run
    monkeypatch.setattr(doctor.subprocess, "run", lambda cmd, *a, **k: (
        sp.CompletedProcess(cmd, 0, stdout=f"{python}\n", stderr="") if "resolve-python" in cmd else real(cmd, *a, **k)))
    _, env = doctor.isaac_ros_env({"ROS_DISTRO": "humble", "PATH": "/usr/bin:/bin"})
    assert env["PYTHONPATH"].split(":")[0] == str(root / "exts" / "isaacsim.ros2.bridge" / "humble" / "rclpy")


@pytest.mark.parametrize("layout", ["venv", "python.sh", "kit"])
def test_isaac_ros_env_finds_the_bundled_ros_for_each_install_layout(tmp_path, monkeypatch, layout):
    import subprocess as sp

    if layout == "venv":
        root = tmp_path / "venv" / "lib" / "python3.12" / "site-packages" / "isaacsim"
        python = tmp_path / "venv" / "bin" / "python"
    elif layout == "python.sh":
        root = tmp_path / "isaac-sim"
        python = root / "python.sh"
    else:
        root = tmp_path / "isaac-sim"
        python = root / "kit" / "python" / "bin" / "python3"
    (root / "exts" / "isaacsim.ros2.core" / "jazzy" / "rclpy").mkdir(parents=True)
    real = sp.run

    def fake(cmd, *a, **k):
        if "resolve-python" in cmd:
            return sp.CompletedProcess(cmd, 0, stdout=f"{python}\n", stderr="")
        if "sysconfig" in " ".join(map(str, cmd)):
            return sp.CompletedProcess(cmd, 0, stdout=f"{root.parent}\n", stderr="")
        return real(cmd, *a, **k)

    monkeypatch.setattr(doctor.subprocess, "run", fake)
    got, env = doctor.isaac_ros_env({"ROS_DISTRO": "jazzy", "ROS_DOMAIN_ID": "7", "PATH": "/usr/bin:/bin",
                                     "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})
    assert got == str(python)
    assert env["PYTHONPATH"].split(":")[0] == str(root / "exts" / "isaacsim.ros2.core" / "jazzy" / "rclpy")
    assert env["ROS_DOMAIN_ID"] == "7"  # kept, as run.sh does
    assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp" and env["ROS_LOCALHOST_ONLY"] == "1"
    assert env["TWINSMITH_TF_DOCTOR_REEXEC"] == "1"


def test_isaac_ros_reexec_drops_the_flag_and_returns_the_child_status(monkeypatch):
    import subprocess as sp

    calls = []
    monkeypatch.delenv("TWINSMITH_TF_DOCTOR_REEXEC", raising=False)
    monkeypatch.setattr(doctor, "isaac_ros_env", lambda: ("/isaac/python", {"X": "1"}))
    monkeypatch.setattr(doctor.subprocess, "run",
                        lambda cmd, env=None: calls.append((cmd, env)) or sp.CompletedProcess(cmd, 1))
    assert doctor.main(["--fixed-frame", "world", "--isaac-ros", "--seconds", "1"]) == 1
    cmd, env = calls[0]
    assert cmd[0] == "/isaac/python" and "--isaac-ros" not in cmd and cmd[-3:] == ["world", "--seconds", "1"]


def test_namespaced_mobile_robots_get_their_own_odom_frame():
    _, values, _, frames = plan(mode="mobile", namespace="r1")
    v = dict(values)
    assert v["TFWorldToOdom.inputs:childFrameId"] == "r1_odom"
    assert v["TFOdomToBase.inputs:parentFrameId"] == "r1_odom"
    assert v["PublishOdometry.inputs:odomFrameId"] == "r1_odom"
    assert frames["odom"] == "r1_odom"
    assert dict(plan(mode="mobile")[1])["PublishOdometry.inputs:odomFrameId"] == "odom"
    assert check(extras=[("/x/r1_odom", "r1_odom")], mode="mobile", namespace="r1")[0] == "DUPLICATE_FRAME_NAMES"


@pytest.mark.parametrize("ns", ["a//b", "a/1b", "bad ns", "a__b", "a_/b", "a/_b", "a_"])
def test_invalid_namespaces_are_refused(ns):
    with pytest.raises(ValueError):
        ros2.graph_path(ns)
    with pytest.raises(ValueError):
        plan(namespace=ns)


def test_renamed_base_with_extra_frames_is_refused_even_with_robot_state_publisher():
    # Isaac names the extra frames' parent after the base prim, not after base_frame.
    assert check(base_frame="base_footprint", link_frames="robot_state_publisher",
                 extras=[("/World/cam", "cam")])[0] == "BASE_FRAME_MISMATCH"
    _, _, _, frames = plan(base_frame="base_footprint", base_tf_name="chassis",
                           link_frames="robot_state_publisher", extra_prims=["/World/cam"])
    assert [e["parent"] for e in frames["edges"] if e["source"] == "TFExtraFrames"] == ["chassis"]


def test_doctor_loop_does_not_hide_an_orphan_and_is_reported_once():
    edges = [e("world", "base", 0, True), e("a", "b"), e("b", "a"), e("c", "d")]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True)
    assert sum("TF loop" in x for x in errors) == 1
    assert any("do not connect to 'world'" in x and "c" in x and "d" in x for x in errors)


def test_isaac_ros_env_points_cyclone_at_the_repository_profile(tmp_path, monkeypatch):
    env = doctor.dds_env({"PATH": "/usr/bin:/bin"})
    assert env["CYCLONEDDS_URI"].endswith("config/cyclonedds.xml") and env["ROS_DOMAIN_ID"] == "0"


def test_odom_frames_are_one_to_one_across_namespaces():
    assert ros2.odom_frame("a/b") == "a__b_odom" and ros2.odom_frame("a_b") == "a_b_odom"


def test_doctor_reports_orphans_even_when_the_fixed_frame_is_in_a_loop():
    edges = [e("world", "base", 0, True), e("base", "world"), e("c", "d")]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True)
    assert any("TF loop" in x for x in errors)
    assert any("do not connect to 'world'" in x and "c" in x for x in errors)


def test_mismatch_advice_fits_the_layout():
    with_extras = check(base_frame="other", link_frames="robot_state_publisher", extras=[("/c", "cam")])[1]
    assert "link_frames=robot_state_publisher" not in with_extras and "isaac:nameOverride" in with_extras
    assert "link_frames=robot_state_publisher" in check(base_frame="other")[1]


def test_a_parent_of_a_looped_fixed_frame_is_not_called_disconnected():
    edges = [e("world", "base", 0, True), e("base", "world"), e("map", "world"), e("c", "d")]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True)
    cut = [x for x in errors if "do not connect" in x]
    assert cut and cut[0].split(": ", 1)[1].split(" (")[0] == "c, d"


def test_clock_can_be_left_to_an_existing_publisher():
    nodes, values, connects, frames = plan(publish_clock=False)
    assert "PublishClock" not in dict(nodes)
    assert not any("PublishClock" in a for a, _ in values)
    _check_wiring(nodes, values, connects)
    assert "PublishClock" in dict(plan()[0])


def test_the_contract_offers_clock_modes():
    sys.path.insert(0, str(REPO / "mcp"))
    from twinsmith.scene_contracts import REGISTRY

    assert REGISTRY["sim.ros2_tf"]["parameters"]["clock"]["default"] == "auto"


def test_doctor_flags_two_clock_publishers_and_backward_time():
    errors, _, info = doctor.analyze(GOOD, "world", clock_seen=True,
                                     clock_publishers=["/a/clock_pub", "/b/clock_pub"])
    assert any("2 nodes publish /clock" in x and "resetting" in x for x in errors)
    assert info["clock_publishers"] == ["/a/clock_pub", "/b/clock_pub"]
    errors, *_ = doctor.analyze(GOOD, "world", clock_seen=True, clock_publishers=["/a"],
                                clock_times=[5.0, 4.9, 5.0, 4.95, 5.1, 5.05])
    assert any("went backwards 3" in x for x in errors)
    assert doctor.analyze(GOOD, "world", clock_seen=True, clock_publishers=["/a"])[0] == []


def test_rviz_config_sets_a_usable_window_size(tmp_path):
    out = tmp_path / "w.rviz"
    rviz.main(["--fixed-frame", "world", "--out", str(out)])
    yaml = pytest.importorskip("yaml")
    geometry = yaml.safe_load(out.read_text())["Window Geometry"]
    assert geometry["Width"] >= 1200 and geometry["Height"] >= 800


def test_rviz_config_scan_is_flat_coloured_and_odometry_keeps_one_arrow(tmp_path):
    # Isaac's 2D lidar sends zero intensities (scan invisible with Intensity colouring),
    # and 50 default-size odometry arrows covered the view.
    out = tmp_path / "s.rviz"
    rviz.main(["--fixed-frame", "odom", "--out", str(out), "--scan", "/scan", "--odom", "/odom"])
    yaml = pytest.importorskip("yaml")
    shown = {d["Class"]: d for d in yaml.safe_load(out.read_text())["Visualization Manager"]["Displays"]}
    assert shown["rviz_default_plugins/LaserScan"]["Color Transformer"] == "FlatColor"
    odom = shown["rviz_default_plugins/Odometry"]
    assert odom["Keep"] == 1 and odom["Shape"]["Value"] == "Arrow" and odom["Shape"]["Shaft Length"] <= 0.5


def test_the_cyclone_profile_has_no_shared_memory_element():
    # Cyclone builds without iceoryx reject it and no ROS 2 client can start.
    text = (REPO / "config" / "cyclonedds.xml").read_text()
    assert "SharedMemory" not in text and "<AllowMulticast>false</AllowMulticast>" in text


@pytest.mark.parametrize("as_json", [False, True])
def test_doctor_command_reports_two_clocks_end_to_end(monkeypatch, capsys, as_json):
    # Runs main() (argument parsing, clock-jump counting, text and JSON output) on fake ROS data.
    edges = [dict(parent="world", child="base_link", static=True, stamp=0.0),
             dict(parent="base_link", child="tool0", static=False, stamp=5.0)]
    clock = [5.0, 4.9, 5.1, 5.0]  # two publishers a few frames apart
    monkeypatch.setattr(doctor, "capture", lambda seconds, topics: (edges, clock, {"/isaac"}, {"/twin_clock", "/tool_clock"}, {}, []))
    argv = ["--fixed-frame", "world"] + (["--json"] if as_json else [])
    assert doctor.main(argv) == 1
    out = capsys.readouterr().out
    if as_json:
        import json

        data = json.loads(out)
        assert data["clock_publishers"] == ["/tool_clock", "/twin_clock"]
        assert any("2 nodes publish /clock" in x for x in data["errors"])
    else:
        assert "ERROR: 2 nodes publish /clock" in out and "2 publisher(s)" in out


def test_doctor_counts_backward_clock_jumps(monkeypatch, capsys):
    edges = [dict(parent="world", child="base_link", static=True, stamp=0.0),
             dict(parent="base_link", child="tool0", static=False, stamp=5.0)]
    monkeypatch.setattr(doctor, "capture", lambda seconds, topics: (edges, [5.0, 4.0, 5.2], set(), {"/one"}, {}, []))
    assert doctor.main(["--fixed-frame", "world"]) == 1
    assert "went backwards 1 time" in capsys.readouterr().out


def test_a_timeline_restart_is_a_warning_not_an_error(monkeypatch, capsys):
    edges = [dict(parent="world", child="base_link", static=True, stamp=0.0),
             dict(parent="base_link", child="tool0", static=False, stamp=0.1)]
    monkeypatch.setattr(doctor, "capture", lambda seconds, topics: (edges, [5.0, 5.1, 0.0, 0.1], set(), {"/one"}, {}, []))
    assert doctor.main(["--fixed-frame", "world"]) == 0
    out = capsys.readouterr().out
    assert "restarted from 0 1 time" in out and "ERROR" not in out


def test_tf_far_from_the_clock_is_an_error():
    # The work cell's clock keeps counting across stop/play while TF restarted at 0.
    edges = GOOD[:1] + [e("base_link", "tool0", stamp=2.0)]
    errors, *_ = doctor.analyze(edges, "world", clock_seen=True, clock_publishers=["/twin"], clock_times=[121.9, 122.0])
    assert any("1 frame(s) differ from /clock" in x and "tool0 -120.0 s" in x for x in errors)
    assert doctor.analyze(edges, "world", clock_seen=True, clock_publishers=["/a"], clock_times=[1.9, 2.0])[0] == []


def test_robot_graphs_never_publish_the_clock_and_match_its_reset_behaviour():
    _, values, _, _ = plan(publish_clock=False, reset_on_stop=False)
    assert dict(values)["ReadSimTime.inputs:resetOnStop"] is False
    nodes, values, connects = ros2.clock_graph_plan()
    assert dict(nodes)["PublishClock"] == "isaacsim.ros2.bridge.ROS2PublishClock"
    assert ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp") in connects
    assert ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn") in connects


def pub(node, kind="/clock", tool=False):
    return dict(node=node, publishes=kind, tool=tool)


def test_clock_plan_reuses_a_foreign_clock_and_keeps_one_shared_tool_clock():
    twin = pub("/Graph/ActionGraph_UR5e/ros2_publish_clock")
    mine = pub(ros2.CLOCK_GRAPH + "/PublishClock", tool=True)
    p = ros2.plan_clock("auto", [twin])
    assert not p["publish"] and p["source"] == twin["node"] and p["reset_on_stop"] is None
    p = ros2.plan_clock("auto", [])
    assert p["publish"] and p["reset_on_stop"] is True and p["source"].startswith(ros2.CLOCK_GRAPH)
    p = ros2.plan_clock("auto", [mine])  # second robot: keep using the shared tool clock
    assert p["publish"] and not p["remove_tool_clock"]
    p = ros2.plan_clock("auto", [twin, mine])  # twin set up afterwards: drop the tool's clock
    assert not p["publish"] and p["remove_tool_clock"] and p["source"] == twin["node"]
    p = ros2.plan_clock("publish", [twin])
    assert p["publish"] and p["foreign"] == [twin["node"]]
    p = ros2.plan_clock("skip", [])
    assert not p["publish"] and p["source"] is None
    with pytest.raises(ValueError):
        ros2.plan_clock("always", [])


def test_publisher_scan_walks_subgraphs_skips_own_and_disabled_and_marks_tool_graphs(monkeypatch):
    import types

    class Node:
        def __init__(self, path, kind):
            self.path, self.kind = path, kind

        def get_prim_path(self):
            return self.path

        def get_type_name(self):
            return self.kind

        def get_attribute_exists(self, name):
            return False

    class Graph:
        def __init__(self, path, nodes=(), subs=(), disabled=False):
            self.path, self.nodes, self.subs, self.disabled = path, list(nodes), list(subs), disabled

        def get_path_to_graph(self):
            return self.path

        def get_nodes(self):
            return self.nodes

        def get_subgraphs(self):
            return self.subs

        def is_disabled(self):
            return self.disabled

    own = ros2.graph_path("r2")
    sub = Graph("/Graph/Outer/Inner", [Node("/Graph/Outer/Inner/c", "omni.isaac.ros2_bridge.ROS2PublishClock")])
    graphs = [
        Graph("/Graph/Outer", [Node("/Graph/Outer/x", "omni.graph.action.OnPlaybackTick")], [sub]),
        Graph(own, [Node(own + "/TFRobotLinks", "isaacsim.ros2.bridge.ROS2PublishTransformTree")]),
        Graph(ros2.graph_path("r1"), [Node("/r1/tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree")]),
        Graph("/Graph/Off", [Node("/Graph/Off/c", "isaacsim.ros2.bridge.ROS2PublishClock")], disabled=True),
    ]
    og = types.SimpleNamespace(get_all_graphs=lambda: graphs)
    monkeypatch.setitem(sys.modules, "omni", types.SimpleNamespace(graph=types.SimpleNamespace(core=og)))
    monkeypatch.setitem(sys.modules, "omni.graph", types.SimpleNamespace(core=og))
    monkeypatch.setitem(sys.modules, "omni.graph.core", og)
    found = ros2.other_ros2_publishers(own)
    assert dict(node="/Graph/Outer/Inner/c", publishes="/clock", tool=False) in found  # subgraph, old type name
    assert dict(node="/r1/tf", publishes="TF", tool=True) in found  # another robot from this tool
    assert not any(f["node"].startswith(own) for f in found)  # own graph excluded
    assert not any(f["node"] == "/Graph/Off/c" for f in found)  # disabled graph skipped


def test_the_clock_option_is_an_enum_in_the_contract():
    sys.path.insert(0, str(REPO / "mcp"))
    from twinsmith.scene_contracts import REGISTRY

    assert REGISTRY["sim.ros2_tf"]["parameters"]["clock"]["enum"] == ["auto", "publish", "skip"]


def test_stop_play_while_listening_is_not_a_skew_error():
    edges = GOOD[:1] + [e("base_link", "tool0", stamp=t) for t in (120.0, 120.1, 0.1)]
    errors, warnings, _ = doctor.analyze(edges, "world", clock_seen=True, clock_publishers=["/one"],
                                         clock_times=[120.0, 120.1, 0.0, 0.1])
    assert errors == [] and any("restarted from 0" in w for w in warnings)


def test_an_early_restart_counts_as_a_reset():
    assert doctor.clock_jumps([0.8, 0.0, 0.1]) == (1, 0)
    assert doctor.clock_jumps([5.0, 4.9, 5.0]) == (0, 1)


def test_reused_clock_reset_behaviour_is_read_from_its_time_node(monkeypatch):
    import types

    class Attr:
        def __init__(self, node):
            self.node = node

        def get_node(self):
            return self.node

    class Node:
        def __init__(self, kind, reset):
            self.kind, self.reset = kind, reset

        def get_type_name(self):
            return self.kind

        def get_attribute(self, name):
            return types.SimpleNamespace(get=lambda: self.reset)

    upstream = [Attr(Node("isaacsim.core.nodes.IsaacReadSimulationTime", False))]
    og = types.SimpleNamespace(Controller=types.SimpleNamespace(
        attribute=lambda path: types.SimpleNamespace(get_upstream_connections=lambda: upstream)))
    monkeypatch.setitem(sys.modules, "omni", types.SimpleNamespace(graph=types.SimpleNamespace(core=og)))
    monkeypatch.setitem(sys.modules, "omni.graph", types.SimpleNamespace(core=og))
    monkeypatch.setitem(sys.modules, "omni.graph.core", og)
    assert ros2.clock_resets_on_stop("/Graph/ActionGraph_UR5e/ros2_publish_clock") is False
    upstream[:] = []
    assert ros2.clock_resets_on_stop("/x") is None


def test_one_robot_on_the_wrong_time_base_is_not_hidden_by_another():
    edges = GOOD[:1] + [e("base_link", "tool0", stamp=120.0), e("r2_base", "r2_tool", stamp=0.3)]
    errors, *_ = doctor.analyze(edges + [e("world", "r2_base", 0, True)], "world", clock_seen=True,
                                clock_publishers=["/twin"], clock_times=[120.0])
    assert any("r2_tool" in x and "differ from /clock" in x for x in errors)


def test_sensor_stamps_far_from_the_clock_are_an_error():
    # The clock graph had resetOnStop=False, the lidar helper restarted at 0 on Stop,
    # and RViz dropped every /scan. TF looked fine, so only a per-topic check finds it.
    edges = GOOD[:1] + [e("base_link", "tool0", stamp=40.0)]
    stamps = {"/scan": [(3.0, 39.9), (3.1, 40.0)], "/camera/image_raw": [(40.0, 40.0)], "/odom": []}
    errors, warnings, info = doctor.analyze(edges, "world", clock_seen=True, clock_publishers=["/c"],
                                            clock_times=[39.9, 40.0], topic_stamps=stamps)
    assert any("1 topic(s) are stamped" in x and "/scan -36.9 s" in x and "resetOnStop" in x for x in errors)
    assert any("no messages on /odom" in w for w in warnings)
    assert info["topics"] == {"/camera/image_raw": 1, "/odom": 0, "/scan": 2}
    # Newest sample only: a stop/play while listening is not reported.
    ok = {"/scan": [(39.0, 39.0), (0.1, 0.1)]}
    assert doctor.analyze(edges[:1], "world", clock_seen=True, clock_publishers=["/c"], clock_times=[0.1],
                          topic_stamps=ok)[0] == []


def test_doctor_passes_extra_topics_to_capture(monkeypatch, capsys):
    seen = {}

    def fake(seconds, topics):
        seen["topics"] = topics
        return ([dict(parent="world", child="base_link", static=True, stamp=0.0)], [1.0], set(), {"/c"},
                {"/my_topic": [(1.0, 1.0)]}, [])

    monkeypatch.setattr(doctor, "capture", fake)
    assert doctor.main(["--fixed-frame", "world", "--topic", "/my_topic"]) == 0
    assert seen["topics"] == ["/my_topic"] and "topics checked ['/my_topic']" in capsys.readouterr().out


def test_unchecked_topics_and_wall_clock_sensor_stamps():
    # A headerless --topic (e.g. /cmd_vel) is "not checked", not "no messages";
    # a sensor on wall-clock time next to a sim-time /clock is an error.
    edges = GOOD[:1]
    errors, warnings, info = doctor.analyze(edges, "world", clock_seen=True, clock_publishers=["/c"],
                                            clock_times=[5.0], topic_stamps={"/scan": [(1.79e9, 5.0)]},
                                            unchecked=["/cmd_vel"])
    assert any("/scan stamped with wall-clock time" in x for x in errors)
    assert not any("stamped more than" in x for x in errors)
    assert any("not checked: /cmd_vel" in w for w in warnings) and not any("no messages" in w for w in warnings)
    assert info["unchecked"] == ["/cmd_vel"]


def test_wall_clock_sensor_stamps_are_fine_when_clock_is_wall_time_too():
    # `ros2 bag play --clock` publishes wall time on /clock.
    errors, *_ = doctor.analyze(GOOD[:1], "world", clock_seen=True, clock_publishers=["/bag"],
                                clock_times=[1.79e9], topic_stamps={"/scan": [(1.79e9, 1.79e9)]})
    assert not any("wall-clock time" in x for x in errors)


# ---- RViz view options, the /cmd_vel driver and scripts/ros_env.sh ---------------------

def _rviz(tmp_path, *extra):
    out = tmp_path / "v.rviz"
    assert rviz.main(["--fixed-frame", "World", "--out", str(out), "--force", *extra]) == 0
    yaml = pytest.importorskip("yaml")
    return yaml.safe_load(out.read_text())


def test_rviz_default_view_and_panels_are_unchanged(tmp_path):
    doc = _rviz(tmp_path)
    view = doc["Visualization Manager"]["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/Orbit" and view["Distance"] == 3
    assert view["Target Frame"] == "<Fixed Frame>"
    assert [p["Class"] for p in doc["Panels"]] == ["rviz_common/Displays", "rviz_common/Views"]


def test_rviz_topdown_view_follows_a_target_frame(tmp_path):
    view = _rviz(tmp_path, "--view", "topdown", "--scale", "80", "--target-frame", "base_link")[
        "Visualization Manager"]["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/TopDownOrtho"
    assert view["Value"] == "TopDownOrtho (rviz_default_plugins)"
    assert view["Scale"] == 80 and view["Target Frame"] == "base_link"


def test_rviz_orbit_distance(tmp_path):
    view = _rviz(tmp_path, "--distance", "6.5", "--target-frame", "base_link")["Visualization Manager"]["Views"]["Current"]
    assert view["Distance"] == 6.5 and view["Target Frame"] == "base_link"


def test_rviz_hide_panels_leaves_the_panels_out(tmp_path):
    # Verified in rviz2 Humble: "Hide Left Dock: true" and per-panel "collapsed: true" are ignored at
    # load; only a config without the panels shows the 3D view alone.
    doc = _rviz(tmp_path, "--hide-panels", "--image", "/camera/rgb", "--width", "1600")
    assert doc["Panels"] == []
    assert doc["Window Geometry"]["Width"] == 1600
    assert doc["Window Geometry"]["Hide Left Dock"] is False
    assert any(d["Class"] == "rviz_default_plugins/Image" for d in doc["Visualization Manager"]["Displays"])


@pytest.mark.parametrize("argv", [["--target-frame", "base link"], ["--distance", "0"], ["--scale", "-1"],
                                  ["--view", "side"], ["--width", "50"]])
def test_rviz_view_options_refuse_bad_values(tmp_path, argv):
    with pytest.raises(SystemExit):
        rviz.main(["--fixed-frame", "world", "--out", str(tmp_path / "x.rviz"), *argv])


cmdvel = _load("cmd_vel_sequence_under_test", "scripts/cmd_vel_sequence.py")


def test_cmd_vel_sequence_parses_and_times_steps():
    steps = cmdvel.parse_sequence("0.15,0,5; 0,0.5,3 ;0,0,1;")
    assert steps == [cmdvel.Step(0.15, 0.0, 5.0), cmdvel.Step(0.0, 0.5, 3.0), cmdvel.Step(0.0, 0.0, 1.0)]
    assert cmdvel.command_at(steps, 0.0).linear == 0.15
    assert cmdvel.command_at(steps, 5.0).angular == 0.5
    assert cmdvel.command_at(steps, 8.5) == cmdvel.Step(0.0, 0.0, 1.0)
    assert cmdvel.command_at(steps, 9.0) is None and cmdvel.command_at(steps, -1) is None
    assert "stop (zero Twist)" in cmdvel.describe(steps)


@pytest.mark.parametrize("text", ["", "1,2", "a,b,c", "0.1,0,0", "0.1,0,-1", "5,0,1", "0,9,1",
                                  "nan,0,1", "0.1,0,601"])
def test_cmd_vel_sequence_refuses_bad_steps(text):
    with pytest.raises(ValueError):
        cmdvel.parse_sequence(text)


FAKE_ROS = '''
import sys, types, json, os
log = open(os.environ["FAKE_ROS_LOG"], "a")
class _V: x = 0.0; y = 0.0; z = 0.0
class Twist:
    def __init__(self): self.linear, self.angular = _V(), _V()
class TwistStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.twist = Twist()
class _Pub:
    def get_subscription_count(self): return 1
    def publish(self, m):
        t = m.twist if isinstance(m, TwistStamped) else m
        log.write(json.dumps([t.linear.x, t.angular.z]) + "\\n"); log.flush()
class _Node:
    def create_publisher(self, *a): return _Pub()
    def get_clock(self): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))
    def destroy_node(self): pass
rclpy = types.ModuleType("rclpy")
rclpy.init = lambda **kw: None
rclpy.create_node = lambda name: _Node()
rclpy.try_shutdown = lambda: None
msg = types.ModuleType("geometry_msgs.msg"); msg.Twist, msg.TwistStamped = Twist, TwistStamped
sys.modules.update({"rclpy": rclpy, "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": msg})
'''


def _run_driver(tmp_path, sequence, signal_after=None):
    import json
    import os
    import signal as sig
    import subprocess
    import time
    (tmp_path / "sitecustomize.py").write_text(FAKE_ROS)
    log = tmp_path / "published.jsonl"
    env = dict(os.environ, PYTHONPATH=str(tmp_path), FAKE_ROS_LOG=str(log))
    proc = subprocess.Popen([sys.executable, str(REPO / "scripts/cmd_vel_sequence.py"), sequence, "--rate", "50"],
                            env=env, stderr=subprocess.PIPE, text=True)
    if signal_after is not None:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (log.exists() and log.read_text().count("\n") >= 3):
            time.sleep(0.05)
        proc.send_signal(signal_after)
    _, err = proc.communicate(timeout=30)
    return proc.returncode, [json.loads(line) for line in log.read_text().splitlines()], err


def test_cmd_vel_driver_publishes_the_sequence_then_zero(tmp_path):
    code, sent, _ = _run_driver(tmp_path, "0.2,0,0.2;0,0.4,0.2")
    assert code == 0
    assert [0.2, 0.0] in sent and [0.0, 0.4] in sent
    assert sent[-5:] == [[0.0, 0.0]] * 5


@pytest.mark.parametrize("signame", ["SIGINT", "SIGTERM", "SIGHUP"])
def test_cmd_vel_driver_stops_the_robot_on_a_signal(tmp_path, signame):
    import signal as sig
    code, sent, err = _run_driver(tmp_path, "0.3,0,60", signal_after=getattr(sig, signame))
    assert code == 130 and signame in err
    assert sent[0] == [0.3, 0.0] and sent[-5:] == [[0.0, 0.0]] * 5


ROS_ENV = REPO / "scripts" / "ros_env.sh"


def _fake_prefix(root, name, conda=False):
    prefix = root / name
    (prefix / "bin").mkdir(parents=True)
    (prefix / "setup.bash").write_text(f'export ROS_DISTRO={name.split("_")[0]}\n'
                                       f'export FAKE_SOURCED="${{FAKE_SOURCED:+$FAKE_SOURCED,}}{name}"\n'
                                       f'export PATH="{prefix}/bin:$PATH"\n')
    (prefix / "local_setup.bash").write_text(
        f'export AMENT_PREFIX_PATH="{prefix}${{AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}}"\n')
    if conda:
        (prefix / "conda-meta").mkdir()
    return prefix


def _source(tmp_path, args, extra_env=None):
    import os
    import subprocess
    bin_dir = tmp_path / "fakebin"
    bin_dir.mkdir(exist_ok=True)
    ros2 = bin_dir / "ros2"
    ros2.write_text('#!/bin/sh\necho "ros2 $*" >> "$FAKE_ROS2_LOG"\n')
    ros2.chmod(0o755)
    script = (f'set -- keep; source "{ROS_ENV}" {args}; rc=$?; '
              'echo "rc=$rc arg=$1 distro=$ROS_DISTRO sourced=$FAKE_SOURCED rmw=$RMW_IMPLEMENTATION '
              'domain=$ROS_DOMAIN_ID ament=$AMENT_PREFIX_PATH"')
    env = {"PATH": f"{bin_dir}:/usr/bin:/bin", "HOME": str(tmp_path), "FAKE_ROS2_LOG": str(tmp_path / "ros2.log"),
           **(extra_env or {})}
    done = subprocess.run(["bash", "--noprofile", "--norc", "-c", script], env=env, capture_output=True,
                          text=True, timeout=30)
    calls = (tmp_path / "ros2.log").read_text() if (tmp_path / "ros2.log").exists() else ""
    return done.stdout.strip(), done.stderr, calls


def test_ros_env_sources_install_then_dds_then_stops_the_daemon(tmp_path):
    base = _fake_prefix(tmp_path / "opt", "jazzy")
    out, err, calls = _source(tmp_path, f"{base}", {"ROS_DOMAIN_ID": "63"})
    assert "rc=0 arg=keep distro=jazzy sourced=jazzy rmw=rmw_cyclonedds_cpp domain=63" in out
    assert calls.strip() == "ros2 daemon stop"
    assert f"sourced {base}/setup.bash" in err and "stopped any ros2 daemon" in err


def test_ros_env_overlay_order_and_keep_daemon(tmp_path):
    base = _fake_prefix(tmp_path / "opt", "humble")
    overlay = _fake_prefix(tmp_path / "ws", "humble_nav2")
    out, _, calls = _source(tmp_path, f"{base} --overlay {overlay} --keep-daemon")
    assert "sourced=humble,humble_nav2" in out and calls == ""


def test_ros_env_stacked_conda_overlay_comes_first_on_the_ament_path(tmp_path):
    # Without micromamba on PATH the script sets CONDA_PREFIX/PATH itself; the re-applied
    # local_setup order must put the overlay first (RoboStack Humble + a Nav2 env).
    base = _fake_prefix(tmp_path / "ros2", "humble", conda=True)
    overlay = _fake_prefix(tmp_path / "ros2", "humble_nav2", conda=True)
    out, _, _ = _source(tmp_path, f"{base} --overlay {overlay} --keep-daemon")
    assert f"ament={overlay}:{base}" in out


def test_ros_env_auto_detects_and_reports_missing_installs(tmp_path):
    _fake_prefix(tmp_path / "roots", "jazzy")
    out, err, _ = _source(tmp_path, "--keep-daemon", {"ISAAC_ROS_SEARCH": str(tmp_path / "roots"),
                                                      "ROS_DISTRO": "jazzy"})
    assert "rc=0" in out and "distro=jazzy" in out
    out, err, _ = _source(tmp_path, "--keep-daemon", {"ISAAC_ROS_SEARCH": str(tmp_path / "none")})
    assert "rc=1" in out and "no ROS 2 install found" in err
    out, err, _ = _source(tmp_path, f"{tmp_path / 'missing'}")
    assert "rc=1" in out and "has no setup.bash" in err


def test_ros_env_must_be_sourced():
    import subprocess
    done = subprocess.run(["bash", str(ROS_ENV)], capture_output=True, text=True, timeout=10)
    assert done.returncode == 2 and "must be sourced" in done.stderr


def test_cmd_vel_sequence_refuses_non_finite_limits():
    import runpy
    import pytest as _pytest

    module = runpy.run_path(str(REPO / "scripts/cmd_vel_sequence.py"))
    with _pytest.raises(SystemExit):
        module["main"](["0.1,0,1", "--max-linear", "nan", "--dry-run"])
    assert module["main"](["0.1,0,1", "--dry-run"]) == 0


def test_attach_to_publishes_a_dynamic_parent_edge_and_no_world_link():
    nodes, values, connects, frames = plan(robot="/World/RG2", base_prim="/World/RG2/rg2_base_link",
                                           base_frame="rg2_base_link", attach_parent="/World/UR5e/wrist_3_link",
                                           attach_parent_frame="wrist_3_link")
    names = [n for n, _ in nodes]
    assert "TFAttach" in names and "TFWorldToBase" not in names
    values = dict((k, v) for k, v in values)
    assert values["TFAttach.inputs:parentPrim"] == "/World/UR5e/wrist_3_link"
    assert values["TFAttach.inputs:targetPrims"] == ["/World/RG2/rg2_base_link"]
    assert values["TFAttach.inputs:topicName"] == "tf"
    edge = next(e for e in frames["edges"] if e["source"] == "TFAttach")
    assert edge == dict(parent="wrist_3_link", child="rg2_base_link", static=False, source="TFAttach")
    assert ("OnPlaybackTick.outputs:tick", "TFAttach.inputs:execIn") in connects
    assert ("Context.outputs:context", "TFAttach.inputs:context") in connects
    assert ("ReadSimTime.outputs:simulationTime", "TFAttach.inputs:timeStamp") in connects


def test_attach_to_refuses_a_mobile_base():
    with pytest.raises(ValueError, match="attach_to"):
        plan(mode="mobile", attach_parent="/World/UR5e/wrist_3_link")


def test_without_attach_to_the_layout_is_unchanged():
    names = [n for n, _ in plan()[0]]
    assert "TFWorldToBase" in names and "TFAttach" not in names


RG2_LINKS = [("/World/RG2/rg2_base_link", "rg2_base_link"), ("/World/RG2/rg2_hand_link", "rg2_hand_link")]


@pytest.mark.parametrize("parent,name,is_body,code", [
    ("/World/RG2/rg2_hand_link", "rg2_hand_link", True, "ATTACH_LOOP"),
    ("/World/RG2", "RG2", True, "ATTACH_LOOP"),
    ("/World/UR5e/wrist_3_link", "wrist_3_link", False, "ATTACH_NOT_A_LINK"),
    ("/World/Other/rg2_hand_link", "rg2_hand_link", True, "DUPLICATE_FRAME_NAMES"),
])
def test_attachment_refusals(parent, name, is_body, code):
    assert ros2.check_attachment(RG2_LINKS, "/World/RG2", parent, name, is_body)[0] == code


def test_valid_attachment_passes():
    assert ros2.check_attachment(RG2_LINKS, "/World/RG2", "/World/UR5e/wrist_3_link", "wrist_3_link", True) is None


def test_attach_to_with_a_renamed_base_is_refused():
    problem = ros2.check_frames(RG2_LINKS, "/World/RG2/rg2_base_link", "gripper", "rg2_base_link", [], "world", "fixed",
                                "robot_state_publisher", attached=True)
    assert problem[0] == "BASE_FRAME_MISMATCH" and "attach_to" in problem[1]


def test_contract_exposes_attach_to_as_a_string():
    from twinsmith.scene_contracts import REGISTRY

    assert REGISTRY["sim.ros2_tf"]["parameters"]["attach_to"]["type"] == "string"


# ---- publishing into a joint state subscriber drives the robot from its own state ----


@pytest.mark.parametrize("topic,namespace,sub_topic,sub_namespace,loops", [
    ("joint_states", "", "/joint_states", "", True),
    ("/joint_states", "", "joint_states", "", True),
    ("joint_states", "rg2", "/rg2/joint_states", "", True),
    ("joint_states", "rg2", "joint_states", "rg2", True),
    ("joint_states", "rg2", "/joint_states", "", False),
    ("isaac/joint_states", "", "/joint_states", "", False),
])
def test_joint_state_loop_compares_resolved_topics(topic, namespace, sub_topic, sub_namespace, loops):
    found = ros2.joint_state_loop(topic, namespace, [("/Graph/n", sub_topic, sub_namespace)])
    assert found == (["/Graph/n"] if loops else [])


def test_setup_refuses_the_loop_before_touching_the_graph():
    import ast

    tree = ast.parse((REPO / "exts/sim-tools/sim_tools/ros2.py").read_text())
    source = ast.unparse(next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "setup"))
    assert source.index("JOINT_STATE_LOOP") < source.index("stage.RemovePrim(graph)")
    assert "retry_with" in source


# ---- a saved file that already holds the loop ----

PUB, SUB = "isaacsim.ros2.bridge.ROS2PublishJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"


@pytest.mark.parametrize("nodes,count", [
    ([("/G/p", PUB, "joint_states", ""), ("/H/s", SUB, "/joint_states", "")], 1),
    ([("/G/p", PUB, None, ""), ("/H/s", SUB, "joint_states", "")], 1),  # the publisher's default topic
    ([("/G/p", PUB, "joint_states", "rg2"), ("/H/s", SUB, "/rg2/joint_states", "")], 1),
    ([("/G/p", PUB, "joint_states", "")], 0),  # a graph that only publishes
    ([("/H/s", SUB, "joint_states", "")], 0),
    ([("/G/p", PUB, "isaac/joint_states", ""), ("/H/s", SUB, "/joint_states", "")], 0),
    ([("/G/p", PUB, "joint_states", ""), ("/H/s", SUB, None, "")], 0),  # the subscriber's default is joint_command
    ([("/G/p", "isaacsim.ros2.bridge.ROS2PublishClock", "joint_states", ""), ("/H/s", SUB, "joint_states", "")], 0),
    ([], 0),
])
def test_find_joint_state_loops(nodes, count):
    assert len(ros2.find_joint_state_loops(nodes)) == count


LOOP_USDA = """#usda 1.0
def "G" {
    def OmniGraphNode "p" {
        token node:type = "isaacsim.ros2.bridge.ROS2PublishJointState"
        string inputs:topicName = "joint_states"
    }
}
def "H" {
    def OmniGraphNode "s" {
        token node:type = "isaacsim.ros2.bridge.ROS2SubscribeJointState"
        string inputs:topicName = "/joint_states"
    }
}
"""


def test_a_saved_loop_is_reported_with_paths_and_the_fix_and_nothing_changes(tmp_path):
    pytest.importorskip("pxr.Usd")
    from pxr import Usd

    file = tmp_path / "loop.usda"
    file.write_text(LOOP_USDA)
    stage = Usd.Stage.Open(str(file))
    before = stage.GetRootLayer().ExportToString()
    report = ros2.joint_state_loop_report(stage)["joint_state_loop"]
    assert report["loops"] == [dict(topic="/joint_states", publishers=["/G/p"], subscribers=["/H/s"])]
    assert "path=/G," in report["fix"] and "with a namespace" in report["fix"] and "allow_foreign=true" in report["fix"] and "confirm=true" in report["fix"] and "not changed" in report["fix"]
    assert stage.GetRootLayer().ExportToString() == before and not list(stage.GetSessionLayer().rootPrims)


def test_a_graph_that_only_publishes_is_not_reported(tmp_path):
    pytest.importorskip("pxr.Usd")
    from pxr import Usd

    file = tmp_path / "pub.usda"
    file.write_text(LOOP_USDA.split('def "H"')[0])
    assert ros2.joint_state_loop_report(Usd.Stage.Open(str(file))) == {}


def test_stage_open_adds_the_report_without_being_able_to_fail_the_open():
    import ast

    tree = ast.parse((REPO / "exts/sim-tools/sim_tools/stage_open.py").read_text())
    helper = ast.unparse(next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_joint_state_loop"))
    assert "except Exception" in helper
    opener = ast.unparse(next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == "open_stage"))
    assert "**loop" in opener


# ---- one namespace, one robot ----


def _graph_stage(robot="/World/UR5e/root_joint", graph=ros2.GRAPH_PATH, tmp_path=None):
    pytest.importorskip("pxr.Usd")
    from pxr import Sdf, Usd

    stage = Usd.Stage.CreateInMemory()
    node = stage.DefinePrim(f"{graph}/PublishJointState", "OmniGraphNode")
    if robot is not None:
        node.CreateRelationship("inputs:targetPrim").SetTargets([Sdf.Path(robot)])
    return stage


def test_same_robot_rebuilds_and_another_robot_refuses():
    stage = _graph_stage()
    assert ros2.check_graph_robot(stage, ros2.GRAPH_PATH, "/World/UR5e/root_joint", "/World/UR5e") is None
    code, message, retry = ros2.check_graph_robot(stage, ros2.GRAPH_PATH, "/World/RG2_Gripper/rg2_base_link", "/World/RG2_Gripper")
    assert code == "ROBOT_GRAPH_CONFLICT" and retry == dict(namespace="rg2_gripper")
    assert "/World/UR5e/root_joint" in message and "namespace=rg2_gripper" in message


def test_both_orders_refuse_the_second_robot():
    gripper_first = _graph_stage("/World/RG2_Gripper/rg2_base_link")
    assert ros2.check_graph_robot(gripper_first, ros2.GRAPH_PATH, "/World/UR5e/root_joint", "/World/UR5e")[2] == dict(namespace="ur5e")


def test_no_graph_there_is_no_conflict():
    pytest.importorskip("pxr.Usd")
    from pxr import Usd

    assert ros2.check_graph_robot(Usd.Stage.CreateInMemory(), ros2.GRAPH_PATH, "/a", "/a") is None


def test_an_unreadable_graph_fails_safe():
    stage = _graph_stage(robot=None)
    assert ros2.graph_robot(stage, ros2.GRAPH_PATH) is None
    assert ros2.check_graph_robot(stage, ros2.GRAPH_PATH, "/World/UR5e/root_joint", "/World/UR5e")[0] == "ROBOT_GRAPH_CONFLICT"


def test_a_saved_and_reopened_graph_reads_the_same(tmp_path):
    from pxr import Usd

    file = tmp_path / "saved.usda"
    stage = _graph_stage()
    stage.GetRootLayer().Export(str(file))
    reopened = Usd.Stage.Open(str(file))
    assert ros2.graph_robot(reopened, ros2.GRAPH_PATH) == "/World/UR5e/root_joint"
    assert ros2.check_graph_robot(reopened, ros2.GRAPH_PATH, "/World/RG2/base", "/World/RG2")[0] == "ROBOT_GRAPH_CONFLICT"


def test_a_namespaced_graph_is_checked_too():
    graph = ros2.graph_path("rg2")
    stage = _graph_stage("/World/RG2/base", graph=graph)
    code, message, retry = ros2.check_graph_robot(stage, graph, "/World/UR5e/root_joint", "/World/UR5e", "rg2")
    assert code == "ROBOT_GRAPH_CONFLICT" and "namespace 'rg2'" in message and retry == dict(namespace="ur5e")


def test_the_suggested_namespace_is_valid_and_free():
    stage = _graph_stage()
    from pxr import Sdf

    stage.DefinePrim(f"{ros2.graph_path('ur5e')}/PublishJointState", "OmniGraphNode")
    retry = ros2.check_graph_robot(stage, ros2.GRAPH_PATH, "/World/Other/ur5e", "/World/Other/UR5e")[2]
    assert retry == dict(namespace="ur5e_2")
    assert ros2.graph_path(retry["namespace"])  # accepted by the namespace rules


@pytest.mark.parametrize("path,expected", [("/World/RG2_Gripper", "rg2_gripper"), ("/World/2arm", "robot_2arm"),
                                           ("/World/__x__", "x"), ("/World/UR5e", "ur5e"), ("/World/a--b", "a_b")])
def test_namespace_for_gives_valid_names(path, expected):
    name = ros2.namespace_for(path)
    assert name == expected and ros2._namespace(name) == name


def test_setup_checks_before_it_removes_anything_and_the_advice_names_the_namespace():
    import ast

    source = (REPO / "exts/sim-tools/sim_tools/ros2.py").read_text()
    tree = ast.parse(source)
    body = ast.unparse(next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "setup"))
    assert body.index("check_graph_robot") < body.index("stage.RemovePrim(graph)")
    assert "a different namespace from this call" in body


def test_rebuilding_a_saved_graph_deactivates_it_and_never_leaves_two():
    own, r1, r2 = ros2.GRAPH_PATH, ros2.GRAPH_PATH + "_r1", ros2.GRAPH_PATH + "_r2"
    assert ros2.rebuild_plan(own, [], [], set()) == dict(graph=own, remove=[], deactivate=[])
    assert ros2.rebuild_plan(own, [own], [own], {own}) == dict(graph=own, remove=[own], deactivate=[])
    assert ros2.rebuild_plan(own, [own], [own], set()) == dict(graph=r1, remove=[], deactivate=[own])
    # Again in this session: our rebuild goes, and the deactivated original still holds its path.
    assert ros2.rebuild_plan(own, [own, r1], [r1], {r1}) == dict(graph=r1, remove=[r1], deactivate=[])
    # A file saved after a rebuild, then reopened: that rebuild is the saved one now. Flatten dropped the inactive original.
    assert ros2.rebuild_plan(own, [r1], [r1], set()) == dict(graph=own, remove=[], deactivate=[r1])


def test_another_namespace_and_the_clock_graph_are_left_alone():
    own = ros2.GRAPH_PATH
    other, clock = ros2.graph_path("rg2"), ros2.CLOCK_GRAPH
    plan = ros2.rebuild_plan(own, [own, other, clock], [own, other, clock], {own, other})
    assert plan == dict(graph=own, remove=[own], deactivate=[])


def test_disabling_a_saved_graph_also_stops_it_at_runtime():
    calls = []

    class Graph:
        def is_valid(self):
            return True

        def set_disabled(self, value):
            calls.append(value)

    class Og:
        get_graph_by_path = staticmethod(lambda path: Graph())

    ros2.disable_runtime_graph(Og, "/Graph/x")
    assert calls == [True]
    ros2.disable_runtime_graph(object(), "/Graph/x")  # no such API on this build: not an error
    assert calls == [True]


def test_a_clock_on_another_topic_is_not_treated_as_this_stages_clock():
    # plan_clock only counts a publisher on /clock, so a user's clock on /s61clock leaves ours in place.
    other = dict(node="/World/C/clock", publishes="/s61clock", tool=False)
    plan = ros2.plan_clock("auto", [other])
    assert plan["publish"] is True and plan["foreign"] == []
    source = (Path(__file__).resolve().parents[2] / "exts/sim-tools/sim_tools/ros2.py").read_text(encoding="utf-8")
    assert 'kind = resolved_topic(topic, value("inputs:nodeNamespace")) if topic else kind' in source


def test_publisher_scan_reads_a_clock_nodes_real_topic(monkeypatch):
    class Attribute:
        def __init__(self, value):
            self.value = value

        def get(self):
            return self.value

    class Node:
        def __init__(self, path, topic):
            self.path, self.attrs = path, {"inputs:topicName": topic, "inputs:nodeNamespace": ""}

        def get_prim_path(self):
            return self.path

        def get_type_name(self):
            return "isaacsim.ros2.bridge.ROS2PublishClock"

        def get_attribute_exists(self, name):
            return name in self.attrs

        def get_attribute(self, name):
            return Attribute(self.attrs[name])

    class Graph:
        def get_path_to_graph(self):
            return "/World/ClockGraph"

        def get_nodes(self):
            return [Node("/World/ClockGraph/a", "s61clock"), Node("/World/ClockGraph/b", "clock")]

    import sys
    import types

    core = types.ModuleType("omni.graph.core")
    monkeypatch.setitem(sys.modules, "omni", types.ModuleType("omni"))
    monkeypatch.setitem(sys.modules, "omni.graph", types.ModuleType("omni.graph"))
    monkeypatch.setitem(sys.modules, "omni.graph.core", core)
    sys.modules["omni"].graph = sys.modules["omni.graph"]
    sys.modules["omni.graph"].core = core
    monkeypatch.setattr(ros2, "_graphs", lambda og: [Graph()])
    assert [o["publishes"] for o in ros2.other_ros2_publishers("/Graph/own")] == ["/s61clock", "/clock"]


def test_a_sensor_on_a_link_is_a_static_frame_and_only_a_free_one_stays_on_the_tree():
    links = ["/R/base_link", "/R/arm"]
    extras = [("/R/arm/cam", "cam"), ("/R/base_link/L/lidar", "lidar"), ("/World/lidar", "w"), ("/R/arm", "arm")]
    tree, static = ros2.extra_frame_parents(extras, links, "/R/base_link", "base_footprint", "mobile")
    assert tree == ["/World/lidar", "/R/arm"]
    assert static == [("/R/arm/cam", "cam", "/R/arm", "arm"), ("/R/base_link/L/lidar", "lidar", "/R/base_link", "base_footprint")]
    tree, static = ros2.extra_frame_parents([("/World/lidar", "w")], links, "/R/base_link", "base_link", "fixed")
    assert tree == [] and static == [("/World/lidar", "w", "/R/base_link", "base_link")]
