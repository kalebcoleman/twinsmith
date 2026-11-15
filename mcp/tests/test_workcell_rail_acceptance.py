"""The rail acceptance scenario's own arithmetic (scripts/workcell_rail_acceptance.py); the run itself needs a simulator."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("workcell_rail_acceptance", ROOT / "scripts/workcell_rail_acceptance.py")
scenario = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(scenario)


def test_the_block_is_out_of_reach_from_the_default_base_and_in_reach_after_the_rail_move(tmp_path):
    config = json.loads((ROOT / "exts/workcell/config/lab_hardware.json").read_text())
    low, high = config["rail"]["limits_m"]["value"]
    top = config["world"]["bench_top_size_xy_m"]["value"]
    for end in ("plus", "minus"):
        task = json.loads(scenario.task_file(tmp_path, end).read_text())
        ((x, y, yaw),) = task["jenga"]["block_poses"]["value"]
        target = scenario.RAIL_M[end]
        assert low <= target <= high                                     # inside the simulated travel
        assert math.hypot(x, y) > scenario.UR5E_REACH_M                  # from the default base at the cell's origin
        assert math.hypot(x - target, y) < 0.6                           # well inside the reach from the new base
        assert abs(x) < 0.94 and -0.81 < y < -0.09 and top == [1.4, 0.9]  # on the table top panel of the bench asset
        assert (tmp_path / task["documentation"]["bom_csv"]["value"]).is_file()
        assert task["jenga"]["block_mass_kg"]["value"] == scenario.BLOCK_MASS_KG


def test_the_rail_profile_keeps_the_configured_speed_and_acceleration():
    config = json.loads((ROOT / "exts/workcell/config/lab_hardware.json").read_text())
    speed, acceleration = config["rail"]["speed_m_s"]["value"], config["rail"]["acceleration_m_s2"]["value"]
    total, at = scenario.trapezoid(0.6, speed, acceleration)
    assert total == pytest.approx(0.6 / speed + speed / acceleration) == pytest.approx(2.9)
    assert at(0.0) == 0.0 and at(total) == pytest.approx(0.6) and at(total + 1.0) == pytest.approx(0.6)
    step = 0.02
    samples = [at(index * step) for index in range(int(total / step) + 2)]
    speeds = [(b - a) / step for a, b in zip(samples, samples[1:])]
    assert max(speeds) <= speed + 1e-9 and min(speeds) >= -1e-12
    assert max(abs(b - a) for a, b in zip(speeds, speeds[1:])) / step <= acceleration + 1e-6
    short_total, short = scenario.trapezoid(0.05, speed, acceleration)   # too short to reach the speed: a triangle
    assert short_total == pytest.approx(2 * math.sqrt(0.05 / acceleration)) and short(short_total) == pytest.approx(0.05)


def test_the_pad_gap_follows_the_rg2_linkage():
    assert scenario.pad_gap(scenario.finger_for_gap(0.040)) == pytest.approx(0.040)
    assert scenario.pad_gap(0.0) == pytest.approx(0.0753, abs=2e-4)                      # 75.3 mm at joint zero
    assert scenario.pad_gap(math.radians(26.065)) == pytest.approx(0.1008, abs=3e-4)     # the open limit
    assert scenario.pad_gap(math.radians(-44.009)) == pytest.approx(0.0, abs=3e-4)       # the pads meet
