"""CPU-only checks of the benchmark's disturbances (scripts/puzzle_disturb.py): where each kind puts things."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location("puzzle_disturb", ROOT / "scripts/puzzle_disturb.py")
disturb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(disturb)


def at(x, y, yaw_deg=0.0, z=0.01):
    out = disturb._yaw(yaw_deg)
    out[:3, 3] = [x, y, z]
    return out


TABLE = {"base": at(0.0, -0.46, 90.0), "bar_in": at(0.02, -0.45, 90.0, 0.02), "bar": at(-0.25, -0.40, 30.0), "fork": at(-0.21, -0.56)}


def test_a_knocked_piece_goes_across_its_grip_to_the_side_with_more_room():
    put = disturb.moves("piece_knocked", TABLE, "base", 0.08, mm=50.0, piece="bar", closing=[1, 0, 0])
    assert list(put) == ["bar"]
    went = put["bar"][:3, 3] - TABLE["bar"][:3, 3]
    across = TABLE["bar"][:3, :3] @ [1, 0, 0]
    assert abs(np.linalg.norm(went) - 0.05) < 1e-9 and abs(abs(went @ across) - 0.05) < 1e-9 and went[2] == 0
    other = TABLE["bar"][:3, 3] - went                                              # the other side was nearer the fork
    assert np.linalg.norm(put["bar"][:2, 3] - TABLE["fork"][:2, 3]) > np.linalg.norm(other[:2] - TABLE["fork"][:2, 3])
    assert np.allclose(put["bar"][:3, :3], TABLE["bar"][:3, :3])


def test_a_moved_base_takes_what_lies_in_it_and_nothing_else():
    put = disturb.moves("base_moved", TABLE, "base", 0.08, mm=25.0, deg=5.0)
    assert sorted(put) == ["bar_in", "base"]
    assert abs(np.linalg.norm(put["base"][:2, 3] - TABLE["base"][:2, 3]) - 0.025) < 1e-9
    turned = math.degrees(math.atan2(put["base"][1, 0], put["base"][0, 0])) - 90.0
    assert abs(turned - 5.0) < 1e-9
    assert np.allclose(np.linalg.inv(put["base"]) @ put["bar_in"], np.linalg.inv(TABLE["base"]) @ TABLE["bar_in"])      # it sits in the base as before


@pytest.mark.parametrize("mm", [50.0, 100.0])
def test_a_table_shift_moves_everything_alike_and_keeps_it_in_reach_where_it_can(mm):
    put = disturb.moves("table_shift", TABLE, "base", 0.08, mm=mm)
    went = [put[name][:3, 3] - TABLE[name][:3, 3] for name in TABLE]
    assert sorted(put) == sorted(TABLE) and all(np.allclose(w, went[0]) for w in went) and abs(np.linalg.norm(went[0]) - mm / 1000) < 1e-9
    low, high = np.array(disturb.REGION[0]), np.array(disturb.REGION[1])
    assert all(np.all(pose[:2, 3] >= low) and np.all(pose[:2, 3] <= high) for pose in put.values())                      # a way that keeps all four in reach exists here
    with pytest.raises(ValueError):
        disturb.moves("earthquake", TABLE, "base", 0.08)


def test_a_rotation_is_handed_over_as_the_quaternion_the_simulator_takes():
    for yaw in (0.0, 30.0, 90.0, 179.0, -120.0):
        x, y, z, w = disturb._quat(disturb._yaw(yaw)[:3, :3])
        assert abs(2 * math.degrees(math.atan2(z, w)) - yaw) % 360 < 1e-6 or abs(abs(2 * math.degrees(math.atan2(z, w)) - yaw) - 360) < 1e-6
        assert abs(x) < 1e-12 and abs(y) < 1e-12
    half_turn = np.diag([1.0, -1.0, -1.0])                                           # about x: w is zero, the other branch
    assert np.allclose(np.abs(disturb._quat(half_turn)), [1, 0, 0, 0])


def test_a_run_the_bridge_cannot_plan_gives_none(tmp_path):
    bridge = tmp_path / "ros2"
    (bridge / "gap_bridge").mkdir(parents=True)
    (bridge / "gap_bridge/__init__.py").write_text("")
    (bridge / "gap_bridge/cli.py").write_text("import sys\nsys.exit('REFUSED: no run_log.json')\n")
    run = tmp_path / "run"
    run.mkdir()
    assert disturb.replay(run, bridge) is None and "REFUSED" in (run / "bridge/export.log").read_text()
