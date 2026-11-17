"""CPU-only checks of scan to scene (scripts/puzzle_scan.py) on ray-cast pictures of a small made-up pack."""

import importlib.util
import json
import math
import warnings
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


def off_frame(folder, drop_m, tilt_deg=0.0):
    """Give a clip a camera pose whose z = 0 is ``drop_m`` above the table the camera sees, tilted about x: what a real
    camera's calibration and depth do. Returns that frame from the true one."""
    off = scan._pose(scan.rpy_matrix([tilt_deg, 0.0, 0.0]), [0.0, 0.0, -drop_m])
    camera = json.loads((folder / "clip/camera.json").read_text())
    camera["frames"][0]["world_from_camera"] = (off @ np.array(camera["frames"][0]["world_from_camera"])).tolist()
    (folder / "clip/camera.json").write_text(json.dumps(camera))
    return off


def test_heights_are_taken_from_the_table_the_camera_sees_when_it_is_near_the_frames(pack, tmp_path):
    truth = {"plate": flat(pack, "plate", (-0.08, 0.02), 25), "bar_red": flat(pack, "bar_red", (0.06, -0.05), 70),
             "bar_yellow": flat(pack, "bar_yellow", (0.07, 0.06), -40), "ell": flat(pack, "ell", (0.0, -0.08), 130)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    off = off_frame(tmp_path, 0.0045, 0.4)                # the table reads 4.5 mm low and 0.4 degrees tilted
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    reference = looked.table["reference"]
    assert reference["applied"] and reference["tilt_deg"] == pytest.approx(0.4, abs=0.05) and -6.0 < reference["height_mm"] < -3.0
    assert abs(looked.table["median_height_mm"]) < 0.3     # what is left once heights are taken from that table
    estimate = scan.match(looked, pack)
    assert not estimate["not_seen"] and not estimate["unexplained"]
    assert all(piece["support"] == "table" and not piece["flags"] for piece in estimate["pieces"])      # none is "raised", none is doubted
    for piece in estimate["pieces"]:                       # where the frame says they are, on the table
        there = off @ truth[piece["name"]]
        assert math.dist(piece["position_m"][:2], there[:2, 3]) < 0.002


def test_a_table_far_from_the_frames_is_not_taken_for_a_bias(pack, tmp_path):
    truth = {"bar_red": flat(pack, "bar_red", (0.03, 0.02), 0), "ell": flat(pack, "ell", (-0.05, -0.03), 90)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    off_frame(tmp_path, 0.015)                             # 15 mm: a wrong camera pose, not a depth bias
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    reference = looked.table["reference"]
    assert not reference["applied"] and reference["height_mm"] == pytest.approx(-15.0, abs=0.5) and "more than 10 mm" in reference["why"]
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    off_frame(tmp_path, 0.05)                              # no table near z = 0 at all
    assert "fewer than" in scan.Scan(tmp_path / "clip", tmp_path / "run").table["reference"]["why"]


def test_a_thin_piece_with_a_skirt_of_mixed_depth_down_its_edges_is_still_its_shape(pack, tmp_path):
    from PIL import Image
    from scipy import ndimage

    truth = {"ell": flat(pack, "ell", (0.0, -0.02), 130), "bar_red": flat(pack, "bar_red", (0.08, 0.05), 20)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    labels = np.asarray(Image.open(tmp_path / "run/pieces/labels_0000.png")).copy()
    depth = np.asarray(Image.open(tmp_path / "clip/depth/00000.png")).copy()
    ell = labels == list(truth).index("ell") + 1
    skirt = ndimage.binary_dilation(ell, iterations=3) & (labels == 0)        # a depth camera's mixed pixels, and the mask over them
    depth[skirt] = np.round((0.45 - 0.008) / 0.0001).astype(depth.dtype)      # half way up the 16 mm piece
    labels[skirt] = list(truth).index("ell") + 1
    Image.fromarray(labels).save(tmp_path / "run/pieces/labels_0000.png")
    Image.fromarray(depth).save(tmp_path / "clip/depth/00000.png")
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    seen = next(item for item in looked.objects if item["name"] == "pieces:1")
    rest = pack.shape("ell").rests[0]
    assert seen["area"] > 1.3 * rest.area + 2e-4           # by its footprint it is too big to be the piece
    assert seen["top_area"] == pytest.approx(rest.top_area, rel=0.2)          # its upper surface is the piece's
    found = {piece["name"]: piece for piece in scan.match(looked, pack)["pieces"]}
    assert set(found) == set(truth)
    there = truth["ell"]
    assert math.dist(found["ell"]["position_m"][:2], there[:2, 3]) < 0.003


def tipped(pack, name, xy, yaw_deg, about, degrees, lift=0.0):
    """A piece lying flat, then tipped ``degrees`` about a horizontal axis and set down on its lowest point (plus ``lift``)."""
    rest = pack.shape(name).rests[0]
    turn = scan._axis_turn(np.array(about, dtype=float), math.radians(degrees)) @ scan.yaw_matrix(math.radians(yaw_deg)) @ rest.turn
    return scan._pose(turn, [xy[0], xy[1], scan._lowest(pack.shape(name), turn) + lift])


def error(pack, name, found, truth):
    """Position of the centroid and rotation of an estimated piece against the truth, allowing for its symmetries: (mm, degrees)."""
    shape, turn = pack.shape(name), np.array(found["rotation"])
    centroid = lambda pose: pose[:3, :3] @ shape.centroid + pose[:3, 3]
    there = scan._pose(turn, found["position_m"])
    return (1000 * float(np.linalg.norm(centroid(there) - centroid(truth))),
            min(scan.angle_between(truth[:3, :3] @ s, turn) for s, _ in shape.symmetries))


def test_a_piece_tipped_against_another_is_posed_in_all_six_freedoms(pack, tmp_path):
    truth = {"plate": flat(pack, "plate", (-0.08, 0.02), 25), "ell": tipped(pack, "ell", (0.07, -0.03), 130, (1, 0, 0), 24),
             "bar_yellow": tipped(pack, "bar_yellow", (0.06, 0.07), -40, (0, 1, 0), -31)}
    estimate = look(pack, truth, tmp_path, noise_m=0.0006)
    found = {piece["name"]: piece for piece in estimate["pieces"]}
    assert set(found) == set(truth) and not estimate["unexplained"]
    assert found["plate"]["rest"] == "flat" and not found["plate"]["flags"] and "tilt_deg" not in found["plate"]      # the fast path, untouched
    for name, degrees in (("ell", 24), ("bar_yellow", 31)):
        piece = found[name]
        assert piece["rest"] == "tilted" and piece["tilt_deg"] == pytest.approx(degrees, abs=2.0) and piece["support"] == "table"
        assert any(flag.startswith("tilted") for flag in piece["flags"]) and name in estimate["doubtful"]
        millimetres, turned = error(pack, name, piece, truth[name])
        assert millimetres < 3.0 and turned < 4.0, (name, millimetres, turned)
        assert piece["shape_cost"] <= scan.SHAPE_MAX                       # the same limit as a piece lying flat


def test_a_bar_lying_across_two_pieces_is_found_raised_and_the_pieces_under_it_too(pack, tmp_path):
    under = {"plate": flat(pack, "plate", (-0.03, 0.0), 0), "ell": flat(pack, "ell", (0.09, 0.0), 90)}
    bridge = tipped(pack, "bar_red", (0.035, 0.0), 90, (0, 1, 0), 3.0, lift=0.0)           # one end on the 20 mm plate, the other on the 16 mm ell
    bridge[2, 3] = 0.018 + scan._lowest(pack.shape("bar_red"), bridge[:3, :3])
    estimate = look(pack, {**under, "bar_red": bridge}, tmp_path, noise_m=0.0006)
    found = {piece["name"]: piece for piece in estimate["pieces"]}
    assert set(found) == {"plate", "ell", "bar_red"}
    assert found["bar_red"]["support"] == "raised" and any("on top of something" in flag for flag in found["bar_red"]["flags"])
    millimetres, turned = error(pack, "bar_red", found["bar_red"], bridge)
    assert millimetres < 4.0 and turned < 5.0, (millimetres, turned)
    for name in under:                                                    # each partly hidden by the bar, still where it lies
        assert error(pack, name, found[name], under[name])[0] < 3.0


def test_a_pose_that_another_pose_rivals_says_so_and_gives_the_other(pack, tmp_path):
    # A bar tipped on end against nothing in view: from above, leaning one way or the other can fit alike.
    truth = {"bar_red": tipped(pack, "bar_red", (0.0, 0.0), 0, (1, 0, 0), 40)}
    estimate = look(pack, truth, tmp_path, noise_m=0.0006)
    piece = estimate["pieces"][0]
    assert piece["name"] == "bar_red" and piece["rest"] == "tilted"
    if "other_pose" in piece:                                             # when it is ambiguous, the rival is given whole
        assert set(piece["other_pose"]) == {"degrees_away", "shape_cost", "rest", "position_m", "rpy_deg"}
        assert any("fits nearly as well" in flag for flag in piece["flags"])
    json.dumps(estimate)


def test_the_gripper_in_the_picture_is_not_an_object_on_the_table(pack, tmp_path):
    from PIL import Image

    truth = {"bar_red": flat(pack, "bar_red", (0.03, 0.02), 0)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    labels = np.asarray(Image.open(tmp_path / "run/pieces/labels_0000.png")).copy()
    depth = np.asarray(Image.open(tmp_path / "clip/depth/00000.png")).copy()
    labels[10:120, 20:140] = 2                                            # a mask over a fingertip 10 cm from the lens ...
    depth[10:120, 20:140] = 1000
    depth[100:120, 20:140] = np.round((0.45 - 0.009) / 0.0001).astype(depth.dtype)      # ... with a rim of mixed depth near the table
    Image.fromarray(labels).save(tmp_path / "run/pieces/labels_0000.png")
    Image.fromarray(depth).save(tmp_path / "clip/depth/00000.png")
    estimate = scan.match(scan.Scan(tmp_path / "clip", tmp_path / "run"), pack)
    assert [piece["name"] for piece in estimate["pieces"]] == ["bar_red"] and not estimate["unexplained"]


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
    assert pieces["fork_yellow"] == {"name": "fork_yellow", "asset": "fork_yellow", "parent": "base3", "position_m": [0.0, -0.036, 0.0351],
                                     "rpy_deg": [-90.0, 0.0, 0.0]}                          # the slot's pose, 0.1 mm above resting
    assert list(pieces).index("base3") < list(pieces).index("fork_yellow") and set(written["assets"]) == set(pieces)
    rows = (tmp_path / "task/task.bom.csv").read_text().splitlines()
    assert len(rows) == 11 and rows[0].startswith("bom_id,")


def test_what_stands_between_the_camera_and_the_table_hides_and_is_no_object(pack, tmp_path):
    hand = scan.Pack({**PIECES, "hand": {"boxes": [box([0, 0, 0], [0.008, 0.012, 0.01])], "colour": [0.8, 0.8, 0.8]}}, ASSEMBLIES)
    truth = {"plate": flat(pack, "plate", (0.0, 0.0), 15), "bar_red": flat(pack, "bar_red", (0.1, -0.05), 45)}
    over = scan._pose(np.eye(3), [0.0115, 0.0115, 0.32])            # 0.13 m under the camera: it hides a corner of the plate
    scan.synthetic(hand, {**truth, "hand": over}, CAMERA, tmp_path / "clip", tmp_path / "run")
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    estimate = scan.match(looked, pack)
    assert 1 < looked.table["blind_percent"] < 10 and not estimate["unexplained"]
    assert sorted(piece["name"] for piece in estimate["pieces"]) == ["bar_red", "plate"]
    result = scan.score(estimate, {name: p.tolist() for name, p in truth.items()}, pack)
    assert result["correct"] == 2 and result["position_error_mm"]["max"] < 2.5


def test_colours_are_measured_from_a_known_layout_and_then_used(pack, tmp_path):
    truth = {"bar_red": flat(pack, "bar_red", (0.05, 0.02), 20), "bar_yellow": flat(pack, "bar_yellow", (-0.06, -0.03), 100)}
    scan.synthetic(pack, truth, CAMERA, tmp_path / "clip", tmp_path / "run")
    seen = scan.measure_colours(tmp_path / "clip", truth, pack)
    assert set(seen) == set(truth) and seen["bar_red"][0] > 5 * seen["bar_red"][1] and seen["bar_yellow"][1] > 0.3 * seen["bar_yellow"][0]
    # A pack whose two bars are listed with each other's colours names them wrongly, until it is given the measured ones.
    swapped = {**PIECES, "bar_red": {**PIECES["bar_red"], "colour": PIECES["bar_yellow"]["colour"]},
               "bar_yellow": {**PIECES["bar_yellow"], "colour": PIECES["bar_red"]["colour"]}}
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    wrong = scan.score(scan.match(looked, scan.Pack(swapped, ASSEMBLIES)), {name: p.tolist() for name, p in truth.items()}, pack)
    right = scan.score(scan.match(looked, scan.Pack(swapped, ASSEMBLIES, colours=seen)), {name: p.tolist() for name, p in truth.items()}, pack)
    assert len(wrong["misidentified"]) == 2 and right["correct"] == 2


def test_two_pieces_are_never_left_in_one_place(pack):
    a = {"name": "bar_red", "pose": flat(pack, "bar_red", (0.0, 0.0), 0)}
    assert scan._overlap(pack, a, {"name": "bar_yellow", "pose": flat(pack, "bar_yellow", (0.005, 0.0), 0)}) == pytest.approx(0.015)
    assert scan._overlap(pack, a, {"name": "bar_yellow", "pose": flat(pack, "bar_yellow", (0.03, 0.0), 0)}) <= 0
    slot = flat(pack, "plate", (0.0, 0.0), 0) @ scan._pose(np.eye(3), ASSEMBLIES["plate"][0]["position_m"])
    assert scan._overlap(pack, {"name": "plate", "pose": flat(pack, "plate", (0.0, 0.0), 0)}, {"name": "bar_red", "pose": slot}) < 1e-6


def test_a_piece_is_lifted_off_another_only_when_lifting_takes_it_out_of_it(pack):
    rest = pack.shape("bar_yellow").rests[0]
    under = {"name": "bar_red", "pose": flat(pack, "bar_red", (0.0, 0.0), 0)}
    across = {"name": "bar_yellow", "pose": pose((0.0, 0.0), 90, rest.turn, rest.drop + 0.015)}        # lying across it, 5 mm into its top
    assert scan._overlap(pack, under, across) == pytest.approx(0.005)
    lifted = scan._lifted_clear(pack, across, under, 0.005)
    assert lifted[2, 3] == pytest.approx(rest.drop + 0.0205) and scan._overlap(pack, under, {"name": "bar_yellow", "pose": lifted}) <= 0
    assert across["pose"][2, 3] == pytest.approx(rest.drop + 0.015)                                    # the piece's own pose is not touched
    beside = {"name": "bar_yellow", "pose": pose((0.015, 0.0), 0, rest.turn, rest.drop + 0.005)}       # beside it, 5 mm into its side
    assert scan._overlap(pack, under, beside) == pytest.approx(0.005)
    assert scan._lifted_clear(pack, beside, under, 0.005) is None                                      # no lift parts them


def one_box_at_a_time(shape, turn, at, xy):
    """The heights ``_top_at`` gives, worked out the plain way: one box after another, each line clipped by each slab."""
    best = np.full(len(xy), -np.inf)
    for centre, half in shape.boxes:
        origin = np.column_stack([xy, np.zeros(len(xy))]) - (turn @ centre + at)
        low, high = np.full(len(xy), -np.inf), np.full(len(xy), np.inf)
        for axis in range(3):
            along, start = turn[2, axis], origin @ turn[:, axis]
            if abs(along) < 1e-9:
                outside = np.abs(start) > half[axis]
                low, high = np.where(outside, np.inf, low), np.where(outside, -np.inf, high)
            else:
                a, b = (-half[axis] - start) / along, (half[axis] - start) / along
                low, high = np.maximum(low, np.minimum(a, b)), np.minimum(high, np.maximum(a, b))
        best = np.maximum(best, np.where(high >= low, high, -np.inf))
    return best


def test_every_box_in_one_operation_gives_the_numbers_of_one_box_at_a_time(pack):
    rng = np.random.default_rng(5)
    for trial in range(90):
        shape = pack.shapes[trial % len(pack.shapes)]
        turn = scan.yaw_matrix(rng.uniform(0, 6.3)) @ scan.ROTATIONS[int(rng.integers(24))]      # level: slabs along the vertical
        if trial % 3:
            turn = scan._axis_turn(np.array([1.0, 0, 0]), (0.4, 1e-7)[trial % 3 - 1]) @ turn      # tipped, and all but level
        at = np.array([*rng.uniform(-0.3, 0.3, 2), 0.0])
        xy = at[:2] + rng.uniform(-0.08, 0.08, (400, 2))
        heights = scan._top_at(shape, turn, at, xy)
        assert np.isfinite(heights).any() and not np.isfinite(heights).all()
        assert np.array_equal(heights, one_box_at_a_time(shape, turn, at, xy))                # not close: the same numbers
        assert scan._lowest(shape, turn) == -min(float((turn @ centre)[2] - np.abs(turn[2] * half).sum()) for centre, half in shape.boxes)
        top = shape.top(turn, at + [0, 0, 0.03])
        assert np.array_equal(top[:, 2], one_box_at_a_time(shape, turn, at + [0, 0, 0.03], top[:, :2]))


def test_the_median_is_numpys_whatever_it_is_given():
    rng = np.random.default_rng(6)
    for trial in range(400):
        values = rng.normal(size=int(rng.integers(1, 40)))
        if trial % 4 == 0:
            values = np.round(values, 1)                                    # ties
        if trial % 9 == 0:
            values[int(rng.integers(len(values)))] = np.nan
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")                                 # numpy may say that it met something that is no number
            wanted = float(np.median(values))
        got = scan._median(values)
        assert got == wanted or (math.isnan(got) and math.isnan(wanted))


def test_a_base_plate_is_not_tried_on_an_object_far_too_small_to_be_one(tmp_path, monkeypatch):
    # The least a plate shows from above is its smallest face, from the pack's own boxes: standing on edge, the 100 x 20 mm
    # side that the slot's 24 x 10 mm notch is cut out of.
    plates = scan.Pack({"plate": PIECES["plate"]}, {"plate": []}, "toy/plate")
    plate = plates.shape("plate")
    assert plate.least_area == pytest.approx(0.100 * 0.020 - 0.024 * 0.010)
    limit = scan.BASE_SEEN_MIN * plate.least_area
    assert scan.BASE_SEEN_MIN == 0.5 and limit == pytest.approx(0.00088)                   # half of that face: 8.8 cm2
    assert scan.too_small_for_base({"area": limit - 4e-6}, plate) and not scan.too_small_for_base({"area": limit + 4e-6}, plate)   # a cell either side
    # Two blocks that are no piece of the pack, 20 mm wide and 39 and 41 mm long: with the bit of their walls the camera
    # sees, one shows a few percent under 8.8 cm2 and the other a few percent over.
    blocks = {"under": {"boxes": [box([0, 0, 0], [0.0195, 0.01, 0.01])], "colour": [0.7, 0.03, 0.03]},
              "over": {"boxes": [box([0, 0, 0], [0.0205, 0.01, 0.01])], "colour": [0.1, 0.5, 0.1]}}
    drawn = scan.Pack(blocks, {}, "toy/blocks")
    scan.synthetic(drawn, {"under": flat(drawn, "under", (-0.08, 0.0), 20), "over": flat(drawn, "over", (0.08, 0.0), 20)}, CAMERA, tmp_path / "clip", tmp_path / "run")
    looked = scan.Scan(tmp_path / "clip", tmp_path / "run")
    area = {item["name"]: item["area"] for item in looked.objects}
    assert 0.93 * limit < area["pieces:1"] < limit < area["pieces:2"] < 1.07 * limit
    tried, fit_free = [], scan.fit_free
    monkeypatch.setattr(scan, "fit_free", lambda scanned, item, shape: tried.append(item["name"]) or fit_free(scanned, item, shape))
    estimate = scan.match(looked, plates)
    assert tried == ["pieces:2"]                                               # the one just over is tried as the plate, the one just under is not
    assert not estimate["pieces"] and sorted(one["object"] for one in estimate["unexplained"]) == ["pieces:1", "pieces:2"]
    tried.clear()
    monkeypatch.setattr(scan, "BASE_SEEN_MIN", 0.0)                            # the test turned off: both, as before it was there
    assert scan.match(looked, plates)["unexplained"] == estimate["unexplained"] and tried == ["pieces:1", "pieces:2"]


def test_no_pose_of_a_plate_shows_less_than_its_smallest_face(pack):
    plate, rng, least = pack.shape("plate"), np.random.default_rng(7), np.inf
    for trial in range(300):
        w, x, y, z = (lambda q: q / np.linalg.norm(q))(rng.normal(size=4))
        turn = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        least = min(least, len(plate.top(turn, np.zeros(3), cell=0.001)) * 1e-6)
    assert least > 0.97 * plate.least_area                                     # tipped off a face it shows more, not less
