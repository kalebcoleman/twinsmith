"""Host-side checks for units, provenance and rejected work-cell configurations."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "lab_hardware_config", ROOT / "exts/workcell/workcell/hardware_config.py")
hardware = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hardware)


def test_real_command_units_are_tenths_of_a_millimetre():
    assert hardware.parse_width_command("240") == pytest.approx(0.024)
    assert hardware.parse_width_command("1100") == pytest.approx(0.110)
    assert hardware.parse_width_command("OPEN") == 0.110
    assert hardware.parse_width_command("close") == 0.0
    assert hardware.parse_width_command("query") is None


@pytest.mark.parametrize("command", ["1101", "-1", "24.0", "nan", "１２", "move 24", ""])
def test_bad_width_commands_do_not_actuate(command):
    with pytest.raises(ValueError):
        hardware.parse_width_command(command)


TOWER = ROOT / "exts/workcell/tasks/jenga_tower.json"


def test_default_cell_has_provenance_and_no_task_objects():
    cfg, raw = hardware.load_config()
    assert "jenga" not in cfg and "jenga_block" not in cfg["assets"]
    tower, _ = hardware.read_values(TOWER)
    assert tower["jenga"]["block_size_m"] == [0.072, 0.024, 0.014]
    assert tower["jenga"]["tower_layers_built"] == 18
    assert cfg["gravity"]["mode"] == "physics"
    assert raw["wrist_camera"]["offset_in_tool0_m"]["status"] == "unconfirmed"


@pytest.mark.parametrize("order", [["bota", "rg2", "quick_changer"], ["bota", "bota", "rg2"]])
def test_invalid_stack_rejected_before_scene_changes(tmp_path, order):
    _, raw = hardware.load_config()
    raw["stack"]["order"]["value"] = order
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="stack.order"):
        hardware.load_config(path)


def test_camera_intrinsics_match_configured_fov():
    import math
    fx, fy, cx, cy = hardware.camera_intrinsics([1280, 720], [69.4, 42.5])
    assert math.degrees(2 * math.atan(cx / fx)) == pytest.approx(69.4)
    assert math.degrees(2 * math.atan(cy / fy)) == pytest.approx(42.5)


def test_variation_defaults_are_explicitly_unconfirmed_and_realizable():
    robot_cfg, _ = hardware.load_config()
    cfg, raw = hardware.read_values(TOWER)
    for field in ("variation_profiles", "variation_profile", "variation_seed"):
        assert raw["jenga"][field]["status"] == "unconfirmed"
        assert raw["jenga"][field]["source"]
    assert cfg["jenga"]["variation_profile"] == "measured_pending"
    assert cfg["jenga"]["variation_seed"] == 42
    assert "layers" not in cfg["jenga"]
    assert "base_plate_thickness_m" not in robot_cfg["robot"]
    spec = importlib.util.spec_from_file_location(
        "config_jenga", ROOT / "exts/workcell/workcell/jenga.py")
    jenga = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(jenga)
    report = jenga.realize(cfg)
    assert report == jenga.realize(cfg, profile="measured_pending", seed=42)
    assert report != jenga.realize(cfg, seed=43)
    assert len(report["blocks"]) == 54
    for block in report["blocks"]:
        assert 0.014 <= block["mass_kg"] <= 0.016
        assert 0.28 <= block["static_friction"] <= 0.34
        assert 0.22 <= block["dynamic_friction"] <= 0.27
        assert all(-0.0002 <= value <= 0.0002 for value in block["size_tolerance_m"])
    cfg["jenga"]["variation_profile"] = None
    nominal = jenga.realize(cfg)
    assert all(block["size_m"] == [0.072, 0.024, 0.014] for block in nominal["blocks"])


def test_bom_path_is_relative_to_configuration_directory():
    cfg, raw = hardware.load_config()
    assert cfg["documentation"]["bom_csv"] == "../workcell/bom.csv"
    config_dir = ROOT / "exts/workcell/config"
    assert (config_dir / cfg["documentation"]["bom_csv"]).resolve() == (
        ROOT / "exts/workcell/workcell/bom.csv")
    assert (config_dir / cfg["documentation"]["bom_csv"]).is_file()
    assert raw["documentation"]["bom_csv"]["source"]


def test_finger_pads_sit_on_the_fingertip_gripping_face():
    """The pack's RG2 inner finger: the tip block spans z 0.020..0.053 m and its inner face is the plane y = 0.0189 m.
    A pad nearer the finger joint, or inside that face, is buried in the finger's collision shape and never touches a part."""
    cfg, _ = hardware.load_config()
    thickness, length = cfg["finger_pads"]["size_m"][1], cfg["finger_pads"]["size_m"][2]
    mounts = cfg["finger_pads"]["mounts"]
    assert [mount["parent"] for mount in mounts] == ["left_inner_finger", "right_inner_finger"]
    for mount in mounts:
        x, y, z = mount["position_m"]
        assert x == 0 and mount["orientation_wxyz"] == [1, 0, 0, 0]
        assert y - thickness / 2 >= 0.0189 - 1e-9                    # the whole pad stands proud of the finger face
        assert 0.020 <= z - length / 2 and z + length / 2 <= 0.053   # on the tip block, not at the joint
