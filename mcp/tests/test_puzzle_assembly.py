"""CPU-only checks of the puzzle build's planner (scripts/puzzle_assembly.py) and arm kinematics, on a small made-up pack."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


scan = load("puzzle_scan")
assembly = load("puzzle_assembly")
session = load("workcell_session")

box = lambda centre, half: (np.array(centre, dtype=float), np.array(half, dtype=float))
PIECES = {
    # A 100 x 100 x 10 mm plate with two walls 8 mm high that leave a slot 26 mm wide along y.
    "plate": {"boxes": [box([0, 0, -0.005], [0.05, 0.05, 0.005]), box([-0.0215, 0, 0.004], [0.0085, 0.05, 0.004]), box([0.0215, 0, 0.004], [0.0085, 0.05, 0.004])],
              "colour": [0.05, 0.3, 0.6]},
    "bar": {"boxes": [box([0, 0, 0], [0.012, 0.04, 0.012])], "colour": [0.7, 0.03, 0.03]},          # lies in the slot
    "cap": {"boxes": [box([0, 0, 0], [0.03, 0.01, 0.005])], "colour": [0.9, 0.5, 0.0]},             # lies across the bar
}
ASSEMBLIES = {"plate": [{"piece": "cap", "position_m": [0.0, 0.0, 0.029], "rpy_deg": [0.0, 0.0, 0.0]},
                        {"piece": "bar", "position_m": [0.0, 0.0, 0.012], "rpy_deg": [0.0, 0.0, 0.0]}]}
GRASP = {"point_m": [0.0, 0.0, 0.0125], "closing_axis": [1, 0, 0], "approach_axis": [0, 0, -1], "width_m": 0.024}
no_setback = lambda gap_m: 0.0


@pytest.fixture(scope="module")
def pack():
    return scan.Pack(PIECES, ASSEMBLIES, "toy/v1")


def lying(pack, name, xy, yaw_deg=0.0):
    rest = pack.shape(name).rests[0]
    return assembly.pose(scan.yaw_matrix(math.radians(yaw_deg)) @ rest.turn, [xy[0], xy[1], rest.drop])


def test_the_build_order_is_the_disassembly_reversed(pack):
    taken = assembly.disassembly(pack, "plate")
    assert [step["piece"] for step in taken] == ["cap", "bar"]                 # the cap lies on the bar, so it comes off first
    assert taken[1]["blocked_by"] == ["cap"]
    order = assembly.build_order(pack, "plate")
    assert [(step["piece"], step["needs"]) for step in order] == [("bar", []), ("cap", ["bar"])]
    assert all(step["axis"] == [0, 0, 1] for step in order)                    # both go in from above
    assert order[0]["clear_m"] >= 0.008                                        # the bar is clear once it is above the walls


def test_overlap_is_the_depth_two_boxes_are_in_each_other():
    unit = np.eye(3) * 0.01
    assert assembly.overlap((np.zeros(3), unit), (np.array([0.015, 0, 0]), unit)) == pytest.approx(0.005)
    assert assembly.overlap((np.zeros(3), unit), (np.array([0.025, 0, 0]), unit)) == 0.0
    turned = scan.yaw_matrix(math.radians(45)) * 0.01
    assert assembly.overlap((np.zeros(3), unit), (np.array([0.02, 0, 0]), turned.T)) == pytest.approx(0.01 * math.sqrt(2) - 0.01, abs=1e-9)


def test_a_grasp_must_fit_where_the_piece_lies_and_where_it_goes(pack):
    plate = lying(pack, "plate", [0.0, 0.0])
    now, seat = lying(pack, "bar", [0.2, 0.1], 30.0), plate @ assembly.slot_poses(pack, "plate")["bar"]
    found = assembly.choose_grasps(pack, "bar", GRASP, now, seat, {"plate": plate}, {"plate": plate}, no_setback, clear_m=0.03)
    assert found
    for option in found:
        assert option["pick"][2, 2] < -0.99                                    # from above
        assert np.allclose(option["pick"], now @ option["in_piece"]) and np.allclose(option["place"], option["seat"] @ option["in_piece"])
        assert assembly.pose_error(pack, "bar", option["seat"], seat) == pytest.approx((0.0, 0.0), abs=1e-9)
    low = dict(GRASP, point_m=[0.0, 0.0, 0.0])                                 # the pads would be in the slot's walls
    assert assembly.choose_grasps(pack, "bar", low, now, seat, {"plate": plate}, {"plate": plate}, no_setback, clear_m=0.03) == []
    beside = {"plate": plate, "cap": assembly.pose(np.eye(3), now[:3, 3] + now[:3, :3] @ [0.03, 0, 0])}      # where a finger must go
    assert assembly.choose_grasps(pack, "bar", GRASP, now, seat, beside, {"plate": plate}, no_setback, clear_m=0.03) == []


def test_pose_error_allows_for_a_piece_symmetries(pack):
    seat = assembly.pose(np.eye(3), [0.1, 0.2, 0.012])
    half_turn = seat @ assembly.pose(scan.yaw_matrix(math.pi), [0, 0, 0])
    assert assembly.pose_error(pack, "bar", half_turn, seat) == pytest.approx((0.0, 0.0), abs=1e-6)
    metres, degrees = assembly.pose_error(pack, "bar", seat @ assembly.pose(scan.yaw_matrix(math.radians(10)), [0.002, 0, 0]), seat)
    assert metres == pytest.approx(0.002) and degrees == pytest.approx(10.0)


def test_a_symmetric_base_is_read_the_way_it_was_read_before(pack):
    reading = lying(pack, "plate", [0.0, -0.46], 90.0)
    half_round = lying(pack, "plate", [0.002, -0.461], -89.0)                  # the same plate: a look can say either
    kept = assembly.nearest(pack, "plate", reading, half_round)
    assert scan.angle_between(kept[:3, :3], reading[:3, :3]) == pytest.approx(1.0, abs=1e-6)
    assert np.allclose(kept[:3, 3], half_round[:3, 3])
    slot = assembly.slot_poses(pack, "plate")["cap"]
    assert np.allclose((kept @ slot)[:3, 3], (half_round @ slot)[:3, 3], atol=1e-9)        # here the slot is on the axis of symmetry


def test_a_scattered_layout_leaves_the_middle_and_the_way_there_free(pack):
    target = lying(pack, "plate", [0.0, -0.46], 90.0)
    layout = assembly.scatter(pack, "plate", target, np.random.default_rng(3))
    again = assembly.scatter(pack, "plate", target, np.random.default_rng(3))
    assert list(layout) == ["plate", "cap", "bar"] and all(np.array_equal(layout[name], again[name]) for name in layout)
    others = {name: at for name, at in layout.items() if name != "plate"}
    assert assembly.path_clear(pack, "plate", layout["plate"], target, others)
    assert 0.24 <= abs(layout["plate"][0, 3] - target[0, 3]) <= 0.32          # the base starts off to one side
    names = list(layout)
    for index, first in enumerate(names):
        assert layout[first][2, 2] == pytest.approx(1.0) or abs(layout[first][2, 2]) == pytest.approx(1.0)
        for second in names[index + 1:]:
            assert assembly._apart(assembly.footprint(pack, first, layout[first]), assembly.footprint(pack, second, layout[second]))
    assert not assembly.path_clear(pack, "plate", layout["plate"], target, {"bar": lying(pack, "bar", target[:2, 3])})


def test_the_arm_closed_form_solutions_all_reach_the_pose():
    rng = np.random.default_rng(0)
    for _ in range(50):
        joints = rng.uniform(-3.0, 3.0, 6)
        target = session.ur_fk(joints)[-1]
        found = session.ur_ik(target)
        assert found and all(np.allclose(session.ur_fk(other)[-1], target, atol=1e-9) for other in found)
        assert any(np.allclose(np.exp(1j * np.array(other)), np.exp(1j * joints), atol=1e-6) for other in found)
    assert session.ur_ik(session.pose(np.eye(3), [2.0, 0.0, 0.3])) == []        # out of reach


def test_a_line_is_solved_in_one_piece_and_a_carry_returns_to_the_posture_the_cell_was_built_in():
    arm = object.__new__(session.Session)                                      # the solving alone: no bridge
    home = [math.radians(value) for value in (90, -80, 100, -110, -90, 0)]
    arm.world_from_base, arm.posture = session.pose(np.eye(3), [0.02, 0.0, 0.08]), home
    arm.commanded = dict(zip(session.ARM, home))
    arm.joints = lambda: dict(arm.commanded)
    start = arm.world_from_base @ session.ur_fk(home)[-1]
    goal = session.pose(scan.yaw_matrix(math.pi / 2) @ start[:3, :3], start[:3, 3] + [0.15, 0.05, 0.0])   # a slide with a quarter turn
    path = [arm.targets()]
    for index in range(1, 21):
        path.append(arm.ik(session.blend(start, goal, index / 20), near=path[-1]))
    assert max(abs(b[joint] - a[joint]) for a, b in zip(path, path[1:]) for joint in session.ARM) < math.radians(10)
    assert np.allclose(arm.world_from_base @ session.ur_fk([path[-1][joint] for joint in session.ARM])[-1], goal, atol=1e-9)
    others = session.ur_ik(session.ur_fk(home)[-1])
    arm.commanded = dict(zip(session.ARM, max(others, key=lambda joints: np.abs(np.array(joints) - home).max())))   # the same pose, another way
    arm.commanded["wrist_3_joint"] += 2 * math.pi                             # and wound a whole turn round
    assert np.allclose(list(arm.reconfigure(start).values()), home, atol=1e-6)                             # unwound, not followed
    with pytest.raises(session.MoveFailed, match="clear of the table"):
        arm.reconfigure(session.pose(np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]]), [0.3, -0.4, 0.03]))     # level, 30 mm above the table
    with pytest.raises(session.MoveFailed, match="cannot reach"):
        arm.ik(session.pose(start[:3, :3], [1.5, 0.0, 0.3]))


def test_the_demo_grips_within_the_rg2_specification():
    config = json.loads((ROOT / "scripts/puzzle_build.json").read_text())
    low, high = session.RG2_FORCE_N
    assert low <= config["grip_force_n"] <= high
    for name, grasp in config["grasps"].items():
        assert 0.0 < grasp["width_m"] + config["open_beyond_m"] <= session.RG2_STROKE_M, name
        closing, approach = np.array(grasp["closing_axis"]), np.array(grasp["approach_axis"])
        assert closing @ approach == 0 and np.linalg.norm(closing) == np.linalg.norm(approach) == 1, name
