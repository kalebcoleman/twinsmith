"""scripts/jenga_robot_loop.py and jenga_human_turn.py: the arithmetic between perception, prediction, the rebuilt twin
and the comparisons (no simulator)."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest

np = pytest.importorskip("numpy")
SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"


def script(name):
    sys.path.insert(0, str(SCRIPTS))                      # the scripts import one another
    try:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


@pytest.fixture(scope="module")
def loop():
    return script("jenga_robot_loop")


@pytest.fixture(scope="module")
def turn():
    return script("jenga_human_turn")


def seen_block(sam_id, face, centre, error=0.004):
    return {"id": sam_id, "pose": {"face": face, "position_world_m": list(centre), "extent_error_m": error}}


def test_blocks_seen_from_one_side_are_put_into_layers_and_slots(loop):
    objects, sam_id = [], 0
    for layer in range(6):
        z = (layer + 0.5) * 0.0137                        # a settled tower is a little lower than 14 mm a layer
        if layer % 2 == 0:
            for slot in range(3):
                sam_id += 1
                objects.append(seen_block(sam_id, "end", (0.70, -0.1333 + (slot - 1) * 0.024, z)))
        else:
            sam_id += 1
            objects.append(seen_block(sam_id, "side", (0.676, -0.1333, z)))
    objects += [seen_block(90, "top", (0.70, -0.1333, 0.09)), seen_block(91, "end", (0.70, -0.11, 0.2), error=0.03), {"id": 92}]
    blocks, axis, layers = loop.tower_from_blocks(objects)
    assert layers == 6 and axis == pytest.approx([0.70, -0.1333])
    assert sorted(blocks) == ["L00_0", "L00_1", "L00_2", "L01_0", "L02_0", "L02_1", "L02_2", "L03_0", "L04_0", "L04_1", "L04_2", "L05_0"]
    assert blocks["L02_2"]["centre"][1] == pytest.approx(-0.1093) and blocks["L02_2"]["face"] == "end"
    assert blocks["L03_0"]["face"] == "side" and blocks["L04_1"]["id"] == 10
    with pytest.raises(SystemExit):
        loop.tower_from_blocks([{"id": 1}])


def pose(x, y, z):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix


def test_tower_change_measures_lean_shift_and_a_fallen_block(loop):
    start = {f"L{layer:02d}_{slot}": pose(0.0, (slot - 1) * 0.024, (layer + 0.5) * 0.014) for layer in range(10) for slot in range(3)}
    assert loop.tower_change(start, start)["lean_deg"] == 0.0
    leaning = {name: pose(m[0, 3] + (m[2, 3] - 0.007) * math.tan(math.radians(2.0)), m[1, 3], m[2, 3]) for name, m in start.items()}
    change = loop.tower_change(start, leaning)
    assert change["lean_deg"] == pytest.approx(2.0, abs=0.01) and change["standing"]
    assert change["top_shift_m"] == pytest.approx(9 * 0.014 * math.tan(math.radians(2.0)), abs=1e-5)
    pushed = dict(start, L05_1=pose(0.03, 0.0, 0.05))     # pushed along x and dropped
    assert not loop.tower_change(start, pushed)["standing"]
    assert loop.tower_change(start, pushed, ["L05_1"]) == dict(loop.tower_change(start, start), standing=True)


def test_rotation_and_quaternion_agree(loop):
    for rotation in (np.eye(3), np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]]),
                     np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])):
        w, x, y, z = loop.rotation_quat(rotation)
        assert np.allclose(loop.quat_rotation(x, y, z, w), rotation) and w * w + x * x + y * y + z * z == pytest.approx(1.0)


def fused(centre, along="x", views=("front",), detections=1):
    return {"position_world_m": list(centre), "long_axis_world": [1.0, 0.0, 0.0] if along == "x" else [0.0, 1.0, 0.0],
            "views": list(views), "detections": detections}


def tower_boxes(layers=4, leave_out=()):
    """What a perfect fusion would give for a regular tower at (0.3, -0.42), settled 0.3 mm a layer."""
    boxes = []
    for layer in range(layers):
        for slot in range(3):
            if (layer, slot) in leave_out:
                continue
            across, z = (slot - 1) * 0.024, (layer + 0.5) * 0.0137
            boxes.append(fused((0.3, -0.42 + across, z)) if layer % 2 == 0 else fused((0.3 + across, -0.42, z), "y"))
    return boxes


def test_fused_boxes_become_named_tower_blocks_and_a_block_lying_elsewhere_is_set_aside(turn):
    wrong = [fused((0.3, -0.42, 2.5 * 0.0137 + 0.004), "y"),          # a long side seen in part and read as an end: lies the wrong way
             fused((0.3, -0.42 + 0.024, 1.5 * 0.0137 + 0.0065), "y")]   # and one between two layers
    boxes = tower_boxes(leave_out={(3, 1)}) + [fused((0.0, -0.67, 0.007))] + wrong
    rows, axis, loose, contradicting = turn.tower_from_boxes(boxes)
    assert axis == pytest.approx([0.3, -0.42]) and len(loose) == 1 and loose[0]["position_world_m"][0] == 0.0
    assert len(contradicting) == 1 and contradicting[0] is wrong[0]    # the second one fits layer 1 and loses to the block seen there
    assert sorted(rows) == ["L00_0", "L00_1", "L00_2", "L01_0", "L01_1", "L01_2", "L02_0", "L02_1", "L02_2", "L03_0", "L03_2"]
    assert (rows["L00_2"]["x"], rows["L00_2"]["y"], rows["L00_2"]["yaw"]) == pytest.approx((0.0, 0.024, 0.0))
    assert (rows["L03_0"]["x"], rows["L03_0"]["y"], rows["L03_0"]["yaw"]) == pytest.approx((-0.024, 0.0, math.pi / 2))
    assert rows["L03_0"]["layer"] == 3 and rows["L02_1"]["z"] == pytest.approx(2.5 * 0.0137)


def test_the_rebuilt_tower_is_scored_block_by_block(turn):
    rows, axis, _, _ = turn.tower_from_boxes(tower_boxes())
    rows["L01_1"]["x"] += 0.003
    rows["L02_0"]["yaw"] = math.radians(182.0)            # the same block, half a turn and two degrees on
    del rows["L03_1"]
    truth = {}
    for layer in range(4):
        for slot in range(3):
            across, matrix = (slot - 1) * 0.024, np.eye(4)
            if layer % 2:
                matrix[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
            matrix[:3, 3] = [0.3 + (across if layer % 2 else 0.0), -0.42 + (0.0 if layer % 2 else across), (layer + 0.5) * 0.0137]
            truth[f"L{layer:02d}_{slot}"] = matrix
    del truth["L00_1"]                                    # the camera saw a block that is not there
    table, missed, made_up = turn.compare(rows, axis, truth)
    assert (missed, made_up) == (["L03_1"], ["L00_1"]) and len(table) == 10
    by_name = {row["block"]: row for row in table}
    assert by_name["L01_1"]["xy_error_mm"] == pytest.approx(3.0) and by_name["L00_0"]["xy_error_mm"] == 0.0
    assert by_name["L02_0"]["yaw_error_deg"] == pytest.approx(2.0) and by_name["L01_0"]["yaw_error_deg"] == 0.0
    assert by_name["L03_0"]["seen_height_error_mm"] == 0.0 and by_name["L03_0"]["twin_height_error_mm"] == pytest.approx(3.5 * 0.3, abs=0.01)


def test_views_go_round_the_tower_with_an_upright_camera(turn):
    plan = turn.views(np.array([0.3, -0.42]), 0.25)
    assert [name for name, *_ in plan] == ["watch", "front", "above", "rail_side", "back"]
    watch = plan[0][1]
    assert watch[1] > -0.42 and watch[2] > 0.25 and plan[0][3] == plan[1][3]   # on the rail's side of the tower, above it; no rail move to the next view
    front = turn.look_at(plan[1][1], plan[1][2])
    assert np.allclose(front, [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])       # right is -y, down is -z, forward is +x
    for name, eye, target, base_x in plan:
        axes = turn.look_at(eye, target)
        assert np.allclose(axes.T @ axes, np.eye(3)) and np.linalg.det(axes) == pytest.approx(1.0)
        assert axes[2, 1] < 0 and abs(axes[2, 0]) < 1e-9                                 # image down points down; no roll
        assert 0.3 < np.linalg.norm(eye - target) < 0.65 and abs(base_x) <= 0.6


def test_the_second_camera_is_off_unless_enabled_and_is_described_generically(turn, tmp_path):
    assert turn.fixed_cameras(turn.CAMERAS) == {}                      # the shipped file enables nothing
    (tripod,) = turn.fixed_cameras(turn.CAMERAS, ["tripod"]).values()
    assert {"width", "height", "fx", "fy", "cx", "cy", "depth_scale", "world_from_camera"} <= set(tripod)
    pose = np.array(tripod["world_from_camera"])
    toward_tower = np.array([0.30, -0.42, 0.125]) - pose[:3, 3]
    assert pose[:3, 2] @ toward_tower / np.linalg.norm(toward_tower) > 0.999      # its optical axis is on the example tower
    with pytest.raises(SystemExit, match="no camera named"):
        turn.fixed_cameras(turn.CAMERAS, ["overhead"])
    bad = dict(tripod, enabled=True, world_from_camera=np.diag([1.0, 1.0, -1.0, 1.0]).tolist())   # a mirror, not a rotation
    (tmp_path / "cameras.json").write_text(__import__("json").dumps({"cameras": {"tripod": bad}}))
    with pytest.raises(SystemExit, match="proper rotation"):
        turn.fixed_cameras(tmp_path / "cameras.json")
