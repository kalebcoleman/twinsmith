"""CPU-only checks for the script that builds the real wrist bracket into pack v3 (no FreeCAD, no pack)."""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location("workcell_bracket_assets", ROOT / "scripts/workcell_bracket_assets.py")
bracket = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bracket)
CONFIG = json.loads((ROOT / "exts/workcell/config/lab_hardware.json").read_text())
CAD = (ROOT / bracket.CAD).read_text()


def test_it_builds_v3_from_v2_and_touches_no_other_pack():
    assert (bracket.PACK, bracket.SOURCE, bracket.TARGET) == ("ur5e-workcell", "v2", "v3")
    assert CONFIG["pack"]["id"]["value"] == bracket.PACK and CONFIG["pack"]["version"]["value"] == bracket.TARGET
    source = (ROOT / "scripts/workcell_bracket_assets.py").read_text()
    assert "ur5e-jenga-workcell" not in source and "jenga" not in source


def test_cad_frame_to_the_packs_rg2_body_frame():
    """The pack's RG2 body has +Y along the finger-opening axis and +X out of the back face."""
    assert bracket.to_body([1.0, 0.0, 0.0]).tolist() == [0.0, 1.0, 0.0]        # finger-opening axis
    assert bracket.to_body([0.0, 1.0, 0.0]).tolist() == [-1.0, 0.0, 0.0]       # front face is body -X
    assert bracket.to_body([0.0, 0.0, 1.0]).tolist() == [0.0, 0.0, 1.0]
    points = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]])
    assert np.linalg.det(np.array([bracket.to_body(axis) for axis in np.eye(3)])) == pytest.approx(1.0)
    assert bracket.to_body(points).shape == points.shape


def test_configured_pose_is_the_one_the_cad_model_covers():
    camera = bracket.camera_pose(CONFIG)
    offset = CONFIG["wrist_camera"]["offset_in_rg2_m"]["value"]
    assert bracket.to_body(camera).tolist() == pytest.approx([value * 1000.0 for value in offset])
    # The CAD file's own defaults are that pose, so running it by hand gives the pack's bracket.
    defaults = {name: float(value) for name, value in re.findall(r'env\("(CAM_[XYZ]_MM)", (-?[\d.]+)\)', CAD)}
    assert [defaults[f"CAM_{axis}_MM"] for axis in "XYZ"] == pytest.approx(camera)
    assert camera[1] < -(36.0 / 2 + 23.0 / 2)                     # behind the RG2's back face, by more than half a camera

    tilted = json.loads(json.dumps(CONFIG))
    tilted["wrist_camera"]["rotation_in_rg2_deg"]["value"] = [10.0, 0.0, -90.0]
    with pytest.raises(SystemExit, match="bracket model covers"):
        bracket.camera_pose(tilted)


def test_cad_file_separates_sourced_from_estimated_and_cites_the_vendors():
    assert "https://onrobot.com/storage/datasheets/rg2/datasheet_rg2_v1.8_en.pdf" in CAD
    assert "Intel-RealSense-D400-Series-Datasheet-September-2023.pdf" in CAD
    sourced, estimated = CAD.split("# --- sourced")[1].split("# --- camera pose: estimated from the photo")
    for name, value in (("rg2_waist_width", 54.0), ("rg2_body_thickness", 36.0), ("d415_length", 99.0), ("d415_m3_pitch", 45.0),
                        ("d415_m3_max_insertion", 3.0)):
        assert f'"{name}": {value}' in sourced
    assert '"clamp_z0"' in estimated and '"plate_thickness"' in estimated
    readme = (ROOT / "exts/workcell/cad/README.md").read_text()
    assert "## Estimated from photo, measure to confirm" in readme and "## Sourced dimensions" in readme
