"""Pure logic behind sim.lidar.create config, sim.camera.create optics, sim.viewport
framing and the sim.graph.show window clamp (no Kit needed)."""

import importlib.util
import math
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "sim_tools_placement_under_test", REPO / "exts/sim-tools/sim_tools/placement.py")
placement = importlib.util.module_from_spec(spec)
spec.loader.exec_module(placement)

# The shape of isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS in Isaac Sim 6.2.
REGISTRY = {
    "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda": set(),
    "/Isaac/Sensors/NVIDIA/Example_Rotary.usda": set(),
    "/Isaac/Sensors/Ouster/OS1/OS1.usd": {"OS1_REV6_32ch10hz1024res", "OS1_REV7_128ch20hz512res"},
    "/Isaac/Sensors/SICK/LMS4000/SICK_LMS4000.usd": [
        {"Product": "LMS4111R", "Profile": "Profile01_3p0m_0p0833deg"},
    ],
    "/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd": set(),
}


@pytest.mark.parametrize("name", ["Example_Rotary_2D", "Example Rotary 2D",
                                  "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda"])
def test_lab_lidar_config_names_resolve(name):
    assert placement.resolve_lidar_config(name, None, REGISTRY) == (
        "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda", None)


def test_vendor_prefix_is_optional():
    assert placement.resolve_lidar_config("RPLIDAR_S2E", "", REGISTRY)[0].endswith("Slamtec_RPLIDAR_S2E.usd")
    assert placement.resolve_lidar_config("Slamtec_RPLIDAR_S2E", "", REGISTRY)[0].endswith("Slamtec_RPLIDAR_S2E.usd")


def test_unknown_config_lists_valid_names_and_suggests():
    with pytest.raises(placement.PlacementError) as err:
        placement.resolve_lidar_config("Example_Rotary2D", None, REGISTRY)
    assert err.value.code == "UNKNOWN_CONFIG"
    assert "Example_Rotary_2D" in str(err.value) and "Did you mean" in str(err.value)
    assert "OS1" in err.value.details["valid_configs"]


def test_variants_are_checked_and_listed():
    assert placement.resolve_lidar_config("OS1", "OS1_REV6_32ch10hz1024res", REGISTRY) == (
        "/Isaac/Sensors/Ouster/OS1/OS1.usd", "OS1_REV6_32ch10hz1024res")
    with pytest.raises(placement.PlacementError) as err:
        placement.resolve_lidar_config("OS1", "OS1_bogus", REGISTRY)
    assert err.value.code == "UNKNOWN_VARIANT" and "OS1_REV7_128ch20hz512res" in str(err.value)
    with pytest.raises(placement.PlacementError, match="no variants"):
        placement.resolve_lidar_config("Example_Rotary_2D", "x", REGISTRY)
    # Multi-set variants (SICK) are named by their values joined with "/".
    assert placement.resolve_lidar_config("LMS4000", "LMS4111R/Profile01_3p0m_0p0833deg", REGISTRY)[1] == {
        "Product": "LMS4111R", "Profile": "Profile01_3p0m_0p0833deg"}


@pytest.mark.parametrize("bad", [0, -1, 241, float("nan"), True, "10"])
def test_tick_rate_bounds(bad):
    with pytest.raises(placement.PlacementError):
        placement.tick_rate(bad)
    assert placement.tick_rate(10) == 10.0 and placement.tick_rate(None) is None


def test_lab_camera_rotate_xyz_looks_forward_along_parent_x():
    # Lab 3 Camera_1: rotateXYZ (90, 0, -90) on base_footprint. A USD camera looks along -Z with +Y up.
    q = placement.euler_xyz_to_quat([90, 0, -90])
    forward = placement.rotate(q, (0, 0, -1))
    up = placement.rotate(q, (0, 1, 0))
    assert forward == pytest.approx([1, 0, 0], abs=1e-9)
    assert up == pytest.approx([0, 0, 1], abs=1e-9)
    assert math.isclose(sum(c * c for c in q), 1.0)


def test_rotate_xyz_applies_x_first():
    # X then Y: (0, 1, 0) -> Rx(90) -> (0, 0, 1) -> Ry(90) -> (1, 0, 0). Z-first order would differ.
    q = placement.euler_xyz_to_quat([90, 90, 0])
    assert placement.rotate(q, (0, 1, 0)) == pytest.approx([1, 0, 0], abs=1e-9)


def test_orientation_and_rotate_xyz_are_exclusive():
    assert placement.orientation_args([1, 0, 0, 0], None) == ([1, 0, 0, 0], None)
    q, euler = placement.orientation_args([1, 0, 0, 0], [0, 0, 90])  # the default quaternion is not "given"
    assert euler == [0, 0, 90] and q == pytest.approx([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
    with pytest.raises(placement.PlacementError, match="not both"):
        placement.orientation_args([0, 1, 0, 0], [0, 0, 90])
    with pytest.raises(placement.PlacementError):
        placement.euler_xyz_to_quat([0, 0])


def test_camera_optics_lab_values_and_stage_units():
    o = placement.camera_optics(18, None, [0.01, 100], meters_per_unit=0.01)  # a centimetre stage
    assert o["focal_length"] == 18.0 and "horizontal_aperture" not in o
    assert o["clipping_range_m"] == [0.01, 100.0]
    assert o["clipping_range_stage_units"] == pytest.approx([1.0, 10000.0])
    assert placement.camera_optics()["clipping_range_m"] == [0.01, 1000.0]


@pytest.mark.parametrize("kw", [dict(focal_length=0), dict(focal_length=5000), dict(horizontal_aperture=-1),
                                dict(clipping_range=[1, 1]), dict(clipping_range=[0, 10]), dict(clipping_range=[1])])
def test_camera_optics_refusals(kw):
    with pytest.raises(placement.PlacementError):
        placement.camera_optics(**kw)


def test_fit_distance_fits_the_tighter_axis():
    # 90 degree horizontal FOV (aperture = 2 * focal); a 2:1 viewport has a narrower vertical FOV.
    d_square = placement.fit_distance(1.0, 10, 20, 1.0)
    assert d_square == pytest.approx(1 / math.sin(math.radians(45)))
    d_wide = placement.fit_distance(1.0, 10, 20, 2.0)
    assert d_wide == pytest.approx(1 / math.sin(math.atan(0.5)))
    assert d_wide > d_square
    assert placement.fov_degrees(10, 20) == pytest.approx(90)


def test_frame_eye_backs_off_along_the_view_direction():
    eye, d = placement.frame_eye([1, 2, 0], 1.0, [0, 0, -2], 10, 20, 1.0)
    assert eye == pytest.approx([1, 2, d]) and d > 1
    with pytest.raises(placement.PlacementError):
        placement.frame_eye([0, 0, 0], 1.0, [0, 0, 0], 10, 20, 1.0)


def test_look_at_rejects_degenerate_views():
    assert placement.look_at_valid([3, 0, 1], [0, 0, 0], [0, 0, 1]) == pytest.approx(math.sqrt(10))
    with pytest.raises(placement.PlacementError, match="differ"):
        placement.look_at_valid([0, 0, 0], [0, 0, 0], [0, 0, 1])
    with pytest.raises(placement.PlacementError, match="parallel"):
        placement.look_at_valid([0, 0, 5], [0, 0, 0], [0, 0, 1])


def test_floating_editor_is_clamped_inside_the_main_window():
    # A 1600x1000 editor in a 1440x900 Isaac Sim window keeps its title bar and close button visible.
    w, h, x, y = placement.clamp_window(1600, 1000, 40, 60, 1440, 900)
    assert x + w <= 1440 - 8 and y + h <= 900 - 8 and x >= 8 and y >= 30
    # The main window shrank after the editor was undocked at the right edge.
    w, h, x, y = placement.clamp_window(800, 600, 900, 100, 1000, 700)
    assert x + w <= 992 and y + h <= 692 and w == 800
    # Already inside: unchanged.
    assert placement.clamp_window(800, 600, 40, 60, 1920, 1080) == (800, 600, 40, 60)


def test_graph_fit_waits_for_real_node_bounds():
    placeholder = (500, 500, 0, 0, 1.248, 53.5, 0.0)
    real = (1000.0, 392.5, -900.0, -132.5, 0.731, 657.9, 265.4)
    assert not placement.bbox_settled(None, placeholder)
    assert not placement.bbox_settled(placeholder, placeholder)  # the placeholder never counts
    assert not placement.bbox_settled(placeholder, real)
    assert placement.bbox_settled(real, real)
    assert not placement.bbox_settled(real, (*real[:4], 0.9, 657.9, 265.4))  # zoom still moving


def test_a_straight_down_view_uses_a_sideways_up_and_other_views_keep_the_stage_up():
    assert placement.view_up([0, 0, 10], [0, 0, 0], [0, 0, 1]) == [0.0, 1.0, 0.0]
    assert placement.view_up([0, 10, 0], [0, 0, 0], [0, 1, 0]) == [0.0, 0.0, -1.0]
    assert placement.view_up([5, 0, 3], [0, 0, 0], [0, 0, 1]) == [0, 0, 1]
    placement.look_at_valid([0, 0, 10], [0, 0, 0], placement.view_up([0, 0, 10], [0, 0, 0], [0, 0, 1]))
