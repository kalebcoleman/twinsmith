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
    assert CONFIG["pack"]["id"]["value"] == bracket.PACK and CONFIG["pack"]["version"]["value"] == "v4"
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
    camera, tilt = bracket.camera_pose(CONFIG)
    offset = CONFIG["wrist_camera"]["offset_in_rg2_m"]["value"]
    assert bracket.to_body(camera).tolist() == pytest.approx([value * 1000.0 for value in offset])
    assert tilt == CONFIG["wrist_camera"]["rotation_in_rg2_deg"]["value"][0] and 10.0 <= tilt <= 40.0    # it looks at the fingertips
    # The CAD file's own defaults are that pose, so running it by hand gives the pack's bracket.
    defaults = {name: float(value) for name, value in re.findall(r'env\("(CAM_[XYZ]_MM|TILT_DEG|PITCH_DEG|YAW_DEG)", (-?[\d.]+)\)', CAD)}
    assert [defaults[f"CAM_{axis}_MM"] for axis in "XYZ"] == pytest.approx(camera) and defaults["TILT_DEG"] == tilt
    assert [defaults[name] for name in ("TILT_DEG", "PITCH_DEG", "YAW_DEG")] == CONFIG["wrist_camera"]["rotation_in_rg2_deg"]["value"]
    assert camera[1] < -(36.0 / 2 + 23.0 / 2)                     # behind the RG2's back face, by more than half a camera

    def with_rotation(rotation):
        changed = json.loads(json.dumps(CONFIG))
        changed["wrist_camera"]["rotation_in_rg2_deg"]["value"] = rotation
        return changed

    assert bracket.camera_pose(with_rotation([0.0, 0.0, -90.0]))[1] == 0.0          # any tilt the model covers is taken
    assert bracket.camera_pose(with_rotation([40.0, 0.0, -90.0])) == (camera, 40.0)
    for rotation in ([50.0, 0.0, -90.0], [-5.0, 0.0, -90.0], [25.0, 5.0, -90.0], [25.0, 0.0, 90.0]):
        with pytest.raises(SystemExit, match="bracket model covers"):
            bracket.camera_pose(with_rotation(rotation))


def test_cad_file_separates_sourced_from_estimated_and_cites_the_vendors():
    assert "https://onrobot.com/storage/datasheets/rg2/datasheet_rg2_v1.8_en.pdf" in CAD
    assert "Intel-RealSense-D400-Series-Datasheet-September-2023.pdf" in CAD
    sourced, estimated = CAD.split("# --- sourced")[1].split("# --- camera pose: measured optical frame")
    for name, value in (("rg2_waist_width", 54.0), ("rg2_body_thickness", 36.0), ("d415_length", 99.0), ("d415_m3_pitch", 45.0),
                        ("d415_m3_max_insertion", 3.0)):
        assert f'"{name}": {value}' in sourced
    assert '"clamp_z0"' in estimated and '"plate_thickness"' in estimated
    readme = (ROOT / "exts/workcell/cad/README.md").read_text()
    assert "## Estimated from photo, measure to confirm" in readme and "## Sourced dimensions" in readme


def test_collision_pieces_are_few_convex_and_named_the_same_everywhere():
    """Five convex pieces; the scope name is shared with the builder that attaches it to the RG2 body."""
    import importlib.util as util

    spec = util.spec_from_file_location("bracket_test_cameras", ROOT / "exts/workcell/workcell/cameras.py")
    sys.path.insert(0, str(ROOT / "exts/workcell/workcell"))
    cameras = util.module_from_spec(spec)
    spec.loader.exec_module(cameras)
    assert bracket.COLLISION_SCOPE == cameras.COLLISION_SCOPE
    pieces = re.findall(r'^    "(\w+)": ', CAD.split('report["collision_pieces_mm"] = {')[1].split("\n}\n")[0], re.M)
    assert pieces == ["collar", "front_bar_extension", "truss", "plate_support", "camera_plate"]
    assert '"collision_top_z_near_fingers": 104.0' in CAD
    source = (ROOT / "scripts/workcell_bracket_assets.py").read_text()
    assert 'CreateApproximationAttr("convexHull")' in source and "RigidBodyAPI" not in source and "MassAPI" not in source


def test_calibrated_pack_preserves_source_and_vendor_files(tmp_path, monkeypatch, capsys):
    """A new mount must not silently rewrite vendor files or the preceding pack version."""
    import hashlib
    from types import SimpleNamespace
    import workcell_calibrated_camera_assets as calibrated

    seed = tmp_path / "seed.usda"
    seed.write_text('#usda 1.0\ndef Xform "Asset" {}\n')
    root = tmp_path / "assets"
    root.mkdir()
    calibrated.asset_packs.write_version(
        root, calibrated.PACK, calibrated.SOURCE, {"camera.usda": seed},
        {"camera.usda": {"asset_id": "d415", "source": "test fixture", "license": "test", "redistributable": False}},
        {}, [], extra={"mimic": {"preserved": True}},
    )
    source = root / calibrated.PACK / calibrated.SOURCE
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
    monkeypatch.setattr(calibrated.pack_manifest, "resolve_asset_root", lambda: SimpleNamespace(path=root))

    def fake_cad(work, d415):
        assert d415.read_bytes() == seed.read_bytes()
        folder = work / "wrist_mount"
        folder.mkdir()
        (folder / "wrist_mount.usdc").write_bytes(seed.read_bytes())
        return {"pieces": [], "interference_mm3": {}, "clearance_mm": {}}, {}

    monkeypatch.setattr(calibrated.bracket, "build_bracket", fake_cad)
    calibrated.main()
    assert json.loads(capsys.readouterr().out)["written"]["pack"] == "ur5e-workcell/v4"
    target = calibrated.pack_manifest.AssetPack(root, calibrated.PACK, calibrated.TARGET)
    assert target.asset("d415").read_bytes() == seed.read_bytes()
    assert target.asset("wrist_mount").is_file()
    assert target.manifest["mimic"] == {"preserved": True}
    assert before == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
    monkeypatch.setattr(calibrated.bracket, "build_bracket", lambda *args: pytest.fail("existing v4 was rebuilt"))
    calibrated.main()
    assert "left alone" in capsys.readouterr().out
