"""The robot's linear axis: what the configuration says, the units at the tool boundary, and what is refused. No Kit."""

import copy
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "exts/workcell/workcell"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"railtest_{name}", PACKAGE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rail, hardware = _load("rail"), _load("hardware_config")


@pytest.fixture()
def config():
    values, _ = hardware.load_config()
    return copy.deepcopy(values)


def test_the_shipped_rail_is_a_moving_axis_with_vention_travel_and_a_zero_at_the_built_pose(config):
    assert rail.moving(config) is True
    record = rail.check(config)
    assert record["joint_name"] == "rail_joint" and record["axis_in_world"] == [1.0, 0.0, 0.0]
    assert record["travel_m"] == 1.27 and record["limits_m"] == [-0.635, 0.635]      # Vention: 1530 mm extrusion - 260 mm
    assert record["default_position_m"] == 0.0                                       # the pose the cell is built in
    assert record["max_force_n"] == 1240.0                                           # Vention: peak axial force
    assert record["speed_m_s"] == 0.25 and record["acceleration_m_s2"] == 0.5        # simulation choices, see the sources
    assert set(record["drive"]) == {"physx", "newton"}
    margin = rail.stability(record, config["newton"]["physics_hz"])
    assert margin["ratio"] < 1.0 < margin["limit"]                                   # well inside the explicit-spring limit


def test_sources_say_what_is_known_and_what_is_assumed():
    _, raw = hardware.load_config()
    section = raw["rail"]
    for key in ("travel_m", "limits_m", "default_position_m", "real_zero_at_joint_m", "real_positive_direction",
                "speed_m_s", "acceleration_m_s2", "carriage_mass_kg", "actuator"):
        assert section[key]["status"] == "unconfirmed", key
    assert "docs.vention.com/docs/enclosed-timing-belt-actuator" in section["travel_m"]["source"]
    for key in ("real_zero_at_joint_m", "real_positive_direction"):
        assert section[key]["source"].startswith("ASSUMPTION, not from any source")
    for key in ("speed_m_s", "acceleration_m_s2"):
        assert section[key]["source"].startswith("Simulation choice, not the real value") and "Not found" in section[key]["source"]
    interface = raw["interfaces"]["rail_position_command"]
    assert interface["value"]["topic"] == "/vention/axis1_position_cmd" and interface["value"]["type"] == "std_msgs/msg/Float64"
    assert interface["value"]["unit"] == "mm" and "absolute" in interface["value"]["meaning"]
    assert "ac9c7da23cf7daeacd5c9ecc37c7d5d699c1e53b" in interface["source"]         # the lab's package, pinned
    assert "192.168" not in str(raw)                                                 # no address of the lab's network


def test_units_at_the_tool_boundary(config):
    """The joint is metres from the built pose; the real axis is millimetres, absolute from home. The shipped mapping
    (an assumption) puts home at the -X end of the travel, so the built pose is 635 mm."""
    record = rail.check(config)
    assert rail.to_real_mm(record, 0.0) == pytest.approx(635.0)
    assert rail.to_real_mm(record, -0.635) == pytest.approx(0.0) and rail.to_real_mm(record, 0.635) == pytest.approx(1270.0)
    assert rail.from_real_mm(record, 650.0) == pytest.approx(0.015)                  # the lab's README example
    assert rail.from_real_mm(record, 0.0) == pytest.approx(-0.635) and rail.from_real_mm(record, 100.0) == pytest.approx(-0.535)
    for outside in (-1.0, 1270.5, 5000.0):
        with pytest.raises(ValueError, match="outside the simulated travel"):
            rail.from_real_mm(record, outside)
    for bad in (float("nan"), True, "650"):
        with pytest.raises(ValueError):
            rail.from_real_mm(record, bad)
    view = rail.caller_view(record)
    assert view["joint"] == "rail_joint" and view["limits_m"] == [-0.635, 0.635] and "metres" in view["units"]
    assert view["real_axis"]["limits_mm"] == pytest.approx([0.0, 1270.0]) and view["real_axis"]["at_default_mm"] == pytest.approx(635.0)
    assert "millimetres" in view["real_axis"]["units"] and view["real_axis"]["status"].startswith("unconfirmed")
    mirrored = copy.deepcopy(config)
    mirrored["rail"].update(real_positive_direction=-1, real_zero_at_joint_m=0.635)   # home at the other end
    other = rail.check(mirrored)
    assert rail.to_real_mm(other, 0.0) == pytest.approx(635.0) and rail.from_real_mm(other, 0.0) == pytest.approx(0.635)
    assert rail.caller_view(other)["real_axis"]["limits_mm"] == pytest.approx([0.0, 1270.0])


@pytest.mark.parametrize("fault", ["moving_not_boolean", "limits_wider_than_travel", "default_not_zero", "default_outside",
                                    "bad_axis", "no_newton_drive", "extra_drive_key", "negative_speed", "bad_direction", "bad_name"])
def test_a_bad_rail_is_refused(config, fault):
    section = config["rail"]
    if fault == "moving_not_boolean":
        section["moving"] = "yes"
        with pytest.raises(ValueError, match="rail.moving"):
            rail.moving(config)
        return
    if fault == "limits_wider_than_travel":
        section["limits_m"] = [-0.7, 0.7]
    elif fault == "default_not_zero":
        section["default_position_m"] = 0.1
    elif fault == "default_outside":
        section["limits_m"] = [0.1, 0.6]
    elif fault == "bad_axis":
        section["axis_in_world"] = [0.7, 0.7, 0]
    elif fault == "no_newton_drive":
        del section["drive"]["newton"]
    elif fault == "extra_drive_key":
        section["drive"]["physx"]["armature_kg"] = 1.0
    elif fault == "negative_speed":
        section["speed_m_s"] = -0.1
    elif fault == "bad_direction":
        section["real_positive_direction"] = 0
    else:
        section["joint_name"] = "rail joint"
    with pytest.raises(ValueError, match="rail\\."):
        rail.check(config)


def test_no_rail_section_means_a_fixed_base():
    assert rail.moving({}) is False and rail.moving({"rail": {"moving": False}}) is False
