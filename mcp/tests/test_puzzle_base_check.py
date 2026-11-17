"""CPU-only checks of the fast base check (scripts/puzzle_base_check.py) on ray-cast depth of a small made-up pack."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import puzzle_scan as scan  # noqa: E402

spec = importlib.util.spec_from_file_location("puzzle_base_check", ROOT / "scripts/puzzle_base_check.py")
base_check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base_check)

box = lambda centre, half: (np.array(centre, dtype=float), np.array(half, dtype=float))      # noqa: E731
PIECES = {
    # A 160 x 160 x 20 mm plate with two slots 10 mm deep, one along y and a shorter one along x: no symmetry.
    "plate": {"boxes": [box([-0.05, 0, 0], [0.03, 0.08, 0.01]), box([0.045, 0, 0], [0.035, 0.08, 0.01]), box([-0.005, 0, -0.005], [0.015, 0.08, 0.005]),
                        box([-0.005, 0.06, 0.005], [0.015, 0.02, 0.005])], "colour": [0.05, 0.5, 0.2]},
    "bar": {"boxes": [box([0, 0, 0], [0.012, 0.05, 0.012])], "colour": [0.7, 0.03, 0.03]},
    "block": {"boxes": [box([0, 0, 0], [0.02, 0.02, 0.01])], "colour": [0.9, 0.5, 0.0]},
}
SEAT = scan._pose(np.eye(3), [-0.005, -0.02, 0.012])       # the bar in the long slot, in the plate's frame


@pytest.fixture(scope="module")
def pack():
    return scan.Pack(PIECES, {"plate": [{"piece": "bar", "position_m": [-0.005, -0.02, 0.012], "rpy_deg": [0, 0, 0]}]}, "toy/v1")


def plate(xy, yaw_deg=0.0):
    return scan._pose(scan.yaw_matrix(math.radians(yaw_deg)), [xy[0], xy[1], 0.01])


def camera(eye=(0.0, -0.46, 0.30), tilt_deg=0.0):
    """A wrist camera over the plate: straight down, or tipped ``tilt_deg`` about its own x axis."""
    c, s = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    turn = np.diag([1.0, -1.0, -1.0]) @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return {"fx": 924.0, "fy": 924.0, "cx": 640.0, "cy": 360.0, "width": 1280, "height": 720, "pose": scan._pose(turn, eye)}


def seen(pack, poses, cam, **more):
    return base_check.world_points(scan.render(pack, poses, cam, **more)[1], cam, stride=3)


@pytest.mark.parametrize("shift_mm, yaw_deg", [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (10.0, 0.0), (25.0, 5.0), (10.0, -5.0)])
def test_a_slid_base_is_found_from_one_depth_picture(pack, shift_mm, yaw_deg):
    last = plate((0.0, -0.46), 90.0)
    now = plate((shift_mm / 1000 * math.cos(0.6), -0.46 + shift_mm / 1000 * math.sin(0.6)), 90.0 + yaw_deg)
    found = base_check.check(seen(pack, {"plate": now}, camera(), noise_m=0.0005, seed=1), pack, "plate", last)
    off = np.linalg.norm(found["pose"][:2, 3] - now[:2, 3]) * 1000
    turned = abs(math.degrees(math.atan2(found["pose"][1, 0], found["pose"][0, 0])) - 90.0 - yaw_deg)
    assert off < 0.5 and turned < 0.35, found
    assert base_check.SETTINGS["moved_mm"] == 1.5              # 2 mm is a move: a fork put in 2.6 mm off jammed
    assert found["verdict"] == ("moved" if shift_mm > 1.5 or abs(yaw_deg) > 1.5 else "steady")
    assert abs(found["moved_mm"] - shift_mm) < 1.5 and found["agreement"] > 0.9 and found["on_base"] > 300
    assert found["fit_ms"] < 300.0                            # the budget: one picture's fit, on one core


def test_pieces_seated_in_the_base_move_with_it_and_loose_ones_explain_themselves(pack):
    last, now = plate((0.0, -0.46), 90.0), plate((0.012, -0.452), 93.0)
    block = scan._pose(np.eye(3), [-0.14, -0.40, 0.01])
    poses = {"plate": now, "bar": now @ SEAT, "block": block}
    points = seen(pack, poses, camera(tilt_deg=20.0))
    found = base_check.check(points, pack, "plate", last, seated={"bar": SEAT}, loose={"block": block})
    assert found["verdict"] == "moved" and np.linalg.norm(found["pose"][:2, 3] - now[:2, 3]) < 0.0007 and abs(found["turned_deg"] - 3.0) < 0.5
    bare = base_check.check(points, pack, "plate", last, loose={"block": block})       # the seated bar not told of: the picture has it all the same
    assert np.linalg.norm(bare["pose"][:2, 3] - now[:2, 3]) < 0.002


def test_something_else_moved_is_said_and_not_fitted_away(pack):
    last = plate((0.0, -0.46), 90.0)
    was, now = scan._pose(np.eye(3), [-0.14, -0.40, 0.01]), scan._pose(np.eye(3), [-0.125, -0.47, 0.01])
    found = base_check.check(seen(pack, {"plate": last, "block": now}, camera()), pack, "plate", last, loose={"block": was})
    assert found["verdict"] == "changed" and found["moved_mm"] < 1.0 and "explained by nothing" in found["why"]


def test_a_picture_that_cannot_vouch_for_the_base_says_unsure(pack):
    last = plate((0.0, -0.46), 90.0)
    gone = base_check.check(seen(pack, {"plate": plate((0.30, -0.46), 90.0)}, camera()), pack, "plate", last)       # slid right out of where the fit looks
    assert gone["verdict"] == "unsure" and "too little of it is in view" in gone["why"]
    far = base_check.check(seen(pack, {"plate": plate((0.039, -0.46), 90.0)}, camera()), pack, "plate", last)
    assert far["verdict"] == "unsure" and "as far as the fit looks" in far["why"]
    away = base_check.check(seen(pack, {"plate": last}, camera(eye=(0.0, 0.2, 0.30))), pack, "plate", last)         # the camera looks elsewhere
    assert away["verdict"] == "unsure"
    assert base_check.check(seen(pack, {"plate": last}, camera()), pack, "plate", scan._pose(scan.rpy_matrix([90, 0, 0]), [0, -0.46, 0.08]))["verdict"] == "unsure"


def test_a_close_slanted_view_with_the_hand_in_it_still_gives_the_yaw(pack):
    """As at a pre-place hover: the camera 0.2 m above the plate, tipped 29 degrees, a bar held over the middle, part of
    the plate out of the picture."""
    last = plate((0.0, -0.46), 90.0)
    held = scan._pose(np.eye(3), [0.0, -0.46, 0.075])
    for shift, yaw in (((0.0, 0.0), 0.0), ((0.006, -0.004), 1.0), ((-0.008, 0.006), -2.0)):
        now = plate((shift[0], -0.46 + shift[1]), 90.0 + yaw)
        found = base_check.check(seen(pack, {"plate": now, "bar": held}, camera(eye=(0.0, -0.56, 0.22), tilt_deg=29.0), noise_m=0.0005), pack, "plate", last)
        turned = abs(math.degrees(math.atan2(found["pose"][1, 0], found["pose"][0, 0])) - 90.0 - yaw)
        assert found["verdict"] != "unsure" and np.linalg.norm(found["pose"][:2, 3] - now[:2, 3]) < 0.0008 and turned < 0.5, (shift, yaw, found)


def test_what_the_hand_holds_above_the_base_is_left_out(pack):
    last, now = plate((0.0, -0.46), 90.0), plate((0.008, -0.455), 90.0)
    held = scan._pose(np.eye(3), [0.0, -0.46, 0.075])         # a bar held 40 mm above the plate's top: it hides a strip and is not the base
    found = base_check.check(seen(pack, {"plate": now, "bar": held}, camera()), pack, "plate", last)
    assert found["verdict"] == "moved" and np.linalg.norm(found["pose"][:2, 3] - now[:2, 3]) < 0.0007


def test_a_kept_check_can_be_fitted_again_from_its_files(pack, tmp_path, monkeypatch):
    from PIL import Image

    last, now, cam = plate((0.0, -0.46), 90.0), plate((0.009, -0.455), 91.0), camera()
    depth = scan.render(pack, {"plate": now, "bar": now @ SEAT}, cam)[1]
    (tmp_path / "depth").mkdir()
    Image.fromarray(np.round(depth / 0.0001).astype(np.uint16)).save(tmp_path / "depth" / "00007.png")
    kept = {"camera": {**{key: value for key, value in cam.items() if key != "pose"}, "pose": np.asarray(cam["pose"]).tolist(), "depth_scale": 0.0001},
            "base": "plate", "pack": "toy/v1", "last": last.tolist(), "seated": ["bar"], "loose": {}, "settings": {"moved_mm": 3.0}, "truth": now.tolist()}
    (tmp_path / "check_00007.json").write_text(__import__("json").dumps(kept))
    monkeypatch.setattr(base_check.puzzle_scan, "load_pack", lambda name: pack)
    found = base_check.again(tmp_path / "check_00007.json")
    assert found["verdict"] == "moved" and found["fit_against_truth_mm"] < 0.6 and found["fit_against_truth_deg"] < 0.5
