"""CPU-only checks of scan to scene (scripts/puzzle_scan.py) on ray-cast pictures of a small made-up pack."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("puzzle_scan", ROOT / "scripts/puzzle_scan.py")
scan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan)

box = lambda centre, half: (np.array(centre, dtype=float), np.array(half, dtype=float))
PIECES = {
    # A 100 x 100 x 20 mm plate with a slot 24 mm wide and 10 mm deep along y, off centre so the plate has no symmetry.
    "plate": {"boxes": [box([-0.019, 0, 0], [0.031, 0.05, 0.01]), box([0.043, 0, 0], [0.007, 0.05, 0.01]), box([0.024, 0, -0.005], [0.012, 0.05, 0.005])],
              "colour": [0.05, 0.3, 0.6]},
    "bar_red": {"boxes": [box([0, 0, 0], [0.01, 0.04, 0.01])], "colour": [0.7, 0.03, 0.03]},
    "bar_yellow": {"boxes": [box([0, 0, 0], [0.01, 0.04, 0.01])], "colour": [0.9, 0.5, 0.0]},
    "ell": {"boxes": [box([0, 0, 0], [0.03, 0.008, 0.008]), box([-0.022, 0.02, 0], [0.008, 0.012, 0.008])], "colour": [0.1, 0.5, 0.1]},
}
ASSEMBLIES = {"plate": [{"piece": "bar_red", "position_m": [0.024, 0.0, 0.01], "rpy_deg": [0.0, 0.0, 0.0]}]}
CAMERA = {"fx": 420.0, "fy": 420.0, "cx": 320.0, "cy": 180.0, "width": 640, "height": 360,
          "pose": [[1, 0, 0, 0.0], [0, -1, 0, 0.0], [0, 0, -1, 0.45], [0, 0, 0, 1]]}


@pytest.fixture(scope="module")
def pack():
    return scan.Pack(PIECES, ASSEMBLIES, "toy/v1")


def pose(xy, yaw_deg=0.0, turn=np.eye(3), z=0.0):
    return scan._pose(scan.yaw_matrix(math.radians(yaw_deg)) @ turn, [xy[0], xy[1], z])


def flat(pack, name, xy, yaw_deg):
    rest = pack.shape(name).rests[0]
    return pose(xy, yaw_deg, rest.turn, rest.drop)


def look(pack, poses, folder, drop=None, **more):
    scan.synthetic(pack, poses, CAMERA, folder / "clip", folder / "run", **more)
    if drop:                                               # a piece the segmenter did not find
        from PIL import Image

        labels = np.asarray(Image.open(folder / "run/pieces/labels_0000.png")).copy()
        labels[labels == list(poses).index(drop) + 1] = 0
        Image.fromarray(labels).save(folder / "run/pieces/labels_0000.png")
    return scan.match(scan.Scan(folder / "clip", folder / "run"), pack)


def test_shapes_know_their_symmetries_and_how_they_can_rest(pack):
    assert pack.shape_of["bar_red"] == pack.shape_of["bar_yellow"] != pack.shape_of["ell"]
    bar, plate, ell = pack.shape("bar_red"), pack.shape("plate"), pack.shape("ell")
    assert len(bar.symmetries) == 8 and len(plate.symmetries) == 1 and len(ell.symmetries) == 1
    assert sorted((rest.label, round(rest.height, 3)) for rest in bar.rests) == [("flat", 0.02), ("on end", 0.08)]
    assert plate.rests[0].label == "flat" and plate.rests[0].drop == pytest.approx(0.01)
    assert abs(bar.volume - 0.02 * 0.08 * 0.02) < 1e-9 and plate.volume == pytest.approx(0.1 * 0.1 * 0.02 - 0.024 * 0.1 * 0.01)


def test_rpy_round_trips_through_right_angles_too():
    rng = np.random.default_rng(0)
    for turn in scan.ROTATIONS + [scan.rpy_matrix(rng.uniform(-180, 180, 3)) for _ in range(50)]:
        assert np.allclose(scan.rpy_matrix(scan.matrix_rpy(turn)), turn, atol=1e-9)


def test_loose_pieces_are_named_by_shape_and_colour_and_posed(pack, tmp_path):
    truth = {"plate": flat(pack, "plate", (-0.08, 0.02), 25), "bar_red": flat(pack, "bar_red", (0.06, -0.05), 70),
             "bar_yellow": flat(pack, "bar_yellow", (0.07, 0.06), -40), "ell": flat(pack, "ell", (0.0, -0.08), 130)}
    estimate = look(pack, truth, tmp_path)
    result = scan.score(estimate, {name: p.tolist() for name, p in truth.items()}, pack)
    assert result["correct"] == 4 and not result["not_there"] and not estimate["not_seen"] and not estimate["unexplained"]
    assert result["position_error_mm"]["max"] < 2.0 and result["rotation_error_deg"]["max"] < 3.0
    assert all(piece["support"] == "table" and piece["rest"] == "flat" and not piece["flags"] for piece in estimate["pieces"])
    json.dumps(estimate)                                   # plain data


def test_a_piece_in_a_slot_is_assembled_at_the_slots_pose_whatever_its_colour(pack, tmp_path):
    plate = flat(pack, "plate", (0.02, 0.01), 200)
    slot = plate @ scan._pose(np.eye(3), ASSEMBLIES["plate"][0]["position_m"])
    truth = {"plate": plate, "bar_yellow": slot, "bar_red": flat(pack, "bar_red", (-0.1, -0.06), 10)}      # the other bar is in the slot
    estimate = look(pack, truth, tmp_path)
    found = {piece["name"]: piece for piece in estimate["pieces"]}
    assert found["bar_yellow"]["rest"] == "assembled" and found["bar_yellow"]["parent"] == "plate"
    assert found["bar_yellow"]["in_parent"] == {"position_m": [0.024, 0.0, 0.01], "rpy_deg": [0.0, 0.0, 0.0]}
    assert found["bar_red"]["support"] == "table" and "parent" not in found["bar_red"]
    result = scan.score(estimate, {name: p.tolist() for name, p in truth.items()}, pack)
    assert result["correct"] == 3 and result["position_error_mm"]["max"] < 2.0
    empty = look(pack, {"plate": plate, "bar_red": truth["bar_red"]}, tmp_path / "empty")
    assert {piece["name"]: piece["rest"] for piece in empty["pieces"]} == {"plate": "flat", "bar_red": "flat"}
    assert empty["not_seen"] == ["bar_yellow", "ell"]


def test_a_piece_on_end_or_on_another_is_said_to_be(pack, tmp_path):
    bar = pack.shape("bar_red")
    on_end = next(rest for rest in bar.rests if rest.label == "on end")
    truth = {"plate": flat(pack, "plate", (-0.05, 0.0), 0), "bar_red": pose((0.09, 0.03), 30, on_end.turn, on_end.drop),
             "ell": pose((-0.07, 0.0), 60, pack.shape("ell").rests[0].turn, 0.02 + pack.shape("ell").rests[0].drop)}      # lying on the plate
    found = {piece["name"]: piece for piece in look(pack, truth, tmp_path)["pieces"]}
    assert found["bar_red"]["rest"] == "on end" and "lying on end" in found["bar_red"]["flags"]
    assert found["ell"]["support"] == "raised" and any("on top of something" in flag for flag in found["ell"]["flags"])
    assert found["plate"]["flags"] == []


def test_what_the_segmenter_missed_is_still_matched_and_what_is_no_piece_is_listed(pack, tmp_path):
    truth = {"plate": flat(pack, "plate", (-0.08, 0.0), 10), "bar_red": flat(pack, "bar_red", (0.05, -0.04), 80),
             "ell": flat(pack, "ell", (0.06, 0.07), 0)}
    estimate = look(pack, truth, tmp_path, drop="bar_red")
    assert {piece["name"] for piece in estimate["pieces"]} == set(truth)
    assert next(piece for piece in estimate["pieces"] if piece["name"] == "bar_red")["object"].startswith("unsegmented:")
    odd = scan.Pack({**PIECES, "lump": {"boxes": [box([0, 0, 0], [0.03, 0.03, 0.015])], "colour": [0.5, 0.5, 0.5]}}, ASSEMBLIES)
    scan.synthetic(odd, {**truth, "lump": scan._pose(np.eye(3), [0.0, -0.09, 0.015])}, CAMERA, tmp_path / "odd/clip", tmp_path / "odd/run")
    estimate = scan.match(scan.Scan(tmp_path / "odd/clip", tmp_path / "odd/run"), pack)          # matched against the pack without it
    assert {piece["name"] for piece in estimate["pieces"]} == set(truth) and len(estimate["unexplained"]) == 1
    assert estimate["unexplained"][0]["centre_m"] == pytest.approx([0.0, -0.09], abs=0.003)


def test_noise_brightness_and_a_swap_between_two_looks(pack, tmp_path):
    before = {"plate": flat(pack, "plate", (0.0, 0.0), 0), "bar_red": flat(pack, "bar_red", (0.1, 0.05), 45)}
    after = {"ell": flat(pack, "ell", (0.0, 0.0), 20), "bar_red": before["bar_red"], "bar_yellow": flat(pack, "bar_yellow", (-0.1, 0.06), 0)}
    first, second = look(pack, before, tmp_path / "a", noise_m=0.001), look(pack, after, tmp_path / "b", noise_m=0.001, seed=3)
    change = scan.difference(second, first)
    assert change == {"added": ["bar_yellow", "ell"], "removed": ["plate"], "moved": [], "changed": True}
    assert not scan.difference(first, first)["changed"]
    result = scan.score(second, {name: p.tolist() for name, p in after.items()}, pack)
    assert result["correct"] == 3 and result["position_error_mm"]["max"] < 3.0


def test_a_clip_without_a_camera_pose_is_measured_from_its_table(pack, tmp_path):
    truth = {"bar_red": flat(pack, "bar_red", (0.03, 0.02), 0), "ell": flat(pack, "ell", (-0.05, -0.03), 90)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    camera = json.loads((tmp_path / "clip/camera.json").read_text())
    del camera["frames"]
    (tmp_path / "clip/camera.json").write_text(json.dumps(camera))
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    estimate = scan.match(looked, pack)
    assert looked.frame == estimate["frame"] == "table" and abs(looked.table["median_height_mm"]) < 0.5
    found = {piece["name"]: piece for piece in estimate["pieces"]}
    assert set(found) == set(truth) and all(piece["support"] == "table" for piece in found.values())
    apart = math.dist(found["bar_red"]["centroid_m"], found["ell"]["centroid_m"])
    centre = lambda name: truth[name][:3, :3] @ pack.shape(name).centroid + truth[name][:3, 3]
    assert apart == pytest.approx(math.dist(centre("bar_red"), centre("ell")), abs=0.002)


def test_the_estimate_of_the_real_pack_becomes_a_task_the_cell_can_load(tmp_path, monkeypatch):
    try:
        real = scan.load_pack("puzzles/v1")
    except Exception as error:                             # no asset root, or the pack is not on this machine
        pytest.skip(f"puzzles/v1 unavailable: {error}")
    truth = scan.task_poses(ROOT / "exts/workcell/tasks/puzzle_table.json")
    camera = dict(CAMERA, pose=scan._pose(np.diag([1.0, -1.0, -1.0]), [0.02, -0.50, 0.52]).tolist(), fx=462.0, fy=462.0)
    scan.synthetic(real, truth, camera, tmp_path / "clip", tmp_path / "run")
    estimate = scan.match(scan.Scan(tmp_path / "clip", tmp_path / "run"), real)
    result = scan.score(estimate, {name: p.tolist() for name, p in truth.items()}, real)
    assert result["correct"] == 10 and result["position_error_mm"]["max"] < 3.0
    assert sorted(piece["name"] for piece in estimate["pieces"] if piece.get("parent") == "base3") == ["fork_orange", "fork_yellow", "line_brown", "line_red"]
    written = json.loads(scan.write_task(estimate, tmp_path / "task", "a test").read_text())
    pieces = {piece["name"]: piece for piece in written["puzzle"]["pieces"]["value"]}
    assert pieces["fork_yellow"] == {"name": "fork_yellow", "asset": "fork_yellow", "parent": "base3", "position_m": [0.0, -0.036, 0.035],
                                     "rpy_deg": [-90.0, 0.0, 0.0]}
    assert list(pieces).index("base3") < list(pieces).index("fork_yellow") and set(written["assets"]) == set(pieces)
    rows = (tmp_path / "task/task.bom.csv").read_text().splitlines()
    assert len(rows) == 11 and rows[0].startswith("bom_id,")
