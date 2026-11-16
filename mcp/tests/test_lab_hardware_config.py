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
    assert raw["wrist_camera"]["offset_in_rg2_m"]["status"] == "confirmed"
    for key in ("offset_in_rg2_m", "rotation_in_rg2_deg"):          # measured estimate, with limits retained
        assert "Measured estimate" in raw["wrist_camera"][key]["source"]
    assert raw["pack"]["version"]["value"] == "v4"
    assert "order" not in raw["stack"] and raw["bota"]["mounted"]["value"] is False   # one switch; the arm has no Bota


def _edited(tmp_path, mutate):
    _, raw = hardware.load_config()
    mutate(raw)
    path = tmp_path / "edited.json"
    path.write_text(json.dumps(raw))
    return path


@pytest.mark.parametrize("fault", ["leftover_order", "switch_not_a_boolean", "mounted_without_a_plate", "bad_standoff"])
def test_invalid_stack_rejected_before_scene_changes(tmp_path, fault):
    def mutate(raw):
        if fault == "leftover_order":
            raw["stack"]["order"] = {"value": ["bota", "quick_changer", "rg2"], "status": "unconfirmed", "source": "old file"}
        elif fault == "switch_not_a_boolean":
            raw["bota"]["mounted"]["value"] = 1
        elif fault == "mounted_without_a_plate":
            raw["bota"]["mounted"]["value"] = True
            del raw["bota"]["robot_side_plate"]
        else:
            raw["quick_changer"]["standoff_m"]["value"] = 0
    with pytest.raises(ValueError, match="stack.order|bota.mounted|robot_side_plate|standoff_m"):
        hardware.load_config(_edited(tmp_path, mutate))


def test_the_stack_is_the_real_arm_by_default_and_one_switch_mounts_the_bota(tmp_path):
    """Default: flange, Quick Changer, RG2, and the wrist force/torque reading is the UR5e's own flange sensor. With
    bota.mounted true and nothing else edited, the order, lengths, tool offset, payload and camera pose all follow."""
    off, raw = hardware.load_config()
    stack = hardware.stack
    assert stack.order(off) == ["quick_changer", "rg2"]
    assert stack.lengths(off) == {"quick_changer": 0.0136, "rg2": 0.0}
    tool = stack.tool(off)
    assert tool["flange_to_rg2_inlet_m"] == pytest.approx(0.0136)
    assert tool["rg2_body_in_tool0"]["translation_m"] == pytest.approx([0, 0, -0.0025])
    assert tool["tcp_in_tool0_m"] == pytest.approx([0, 0, 0.2161])
    payload = tool["payload"]
    assert [item["mass_kg"] for item in payload["items"]] == [0.06, 0.78, 0.1, 0.064]
    assert payload["mass_kg"] == pytest.approx(1.004)
    assert payload["cog_in_tool0_m"] == pytest.approx([0.0027676, 0.0069926, 0.0753646], abs=5e-5)     # housing centre differs from the colour optical origin
    assert payload["not_included"] == ["cables"]
    assert not any("adapter plate" in line for line in tool["unmeasured"])
    assert off["flange_ft_sensor"]["frame_id"] == "tool0" and off["flange_ft_sensor"]["measurement"] == "incoming_joint_wrench"
    assert off["flange_ft_sensor"]["wrench_topic"] == "/force_torque_sensor_broadcaster/wrench"
    assert [off["flange_ft_sensor"][key] for key in ("force_range_n", "force_accuracy_n", "torque_range_nm", "torque_accuracy_nm")] == [50.0, 4.0, 10.0, 0.3]

    on, _ = hardware.load_config(_edited(tmp_path, lambda raw: raw["bota"]["mounted"].update(value=True)))
    assert stack.order(on) == ["bota_robot_plate", "bota", "bota_tool_plate", "quick_changer", "rg2"]
    added = 0.008 + 0.032 + 0.008                                  # plate, sensor, plate
    mounted = stack.tool(on)
    assert mounted["flange_to_rg2_inlet_m"] == pytest.approx(0.0136 + added)
    assert mounted["rg2_body_in_tool0"]["translation_m"] == pytest.approx([0, 0, -0.0025 + added])
    assert mounted["rg2_body_in_tool0"]["quaternion_wxyz"] == pytest.approx(tool["rg2_body_in_tool0"]["quaternion_wxyz"])
    assert mounted["tcp_in_tool0_m"] == pytest.approx([0, 0, 0.2161 + added])
    assert mounted["payload"]["mass_kg"] == pytest.approx(1.004 + 0.067 + 0.113 + 0.067)
    assert mounted["payload"]["cog_in_tool0_m"] == pytest.approx([0.00222118, 0.00561199, 0.10374585], abs=5e-5)
    assert sum("adapter plate" in line for line in mounted["unmeasured"]) == 2     # the build says what is not measured
    for record in ("robot_side_plate", "tool_side_plate"):
        assert raw["bota"][record]["status"] == "unconfirmed" and raw["bota"][record]["source"].startswith("PLACEHOLDER, measure it")


def test_the_quick_changer_height_is_onrobots():
    values, raw = hardware.load_config()
    assert values["quick_changer"]["standoff_m"] == 0.0136 and values["quick_changer"]["mass_kg"] == 0.06
    assert "datasheet" in raw["quick_changer"]["standoff_m"]["source"] and "onrobot.com/storage/quick-changer-robot-side" in raw["quick_changer"]["standoff_m"]["source"]
    # OnRobot's RG2 STEP places the robot side on the gripper: the flange plane is 2.5 mm into the RG2 body frame.
    assert values["asset_layout"]["rg2"]["mount_pose"]["position_m"] == [0, 0, 0.0161]


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


def test_grip_is_on_the_real_fingertips_and_there_are_no_add_on_pads():
    """The real RG2 has its standard fingertips, wrapped in tape, and nothing added to them. The config has no pad
    bodies; the fingertip block and its rubber pad are OnRobot's CAD dimensions, and the friction on the fingertip
    collision shapes is an estimate for tape, marked as one."""
    cfg, raw = hardware.load_config()
    assert "finger_pads" not in cfg and "finger_pad" not in cfg["assets"] and "finger_pad" not in cfg["asset_layout"]
    tips = cfg["fingertips"]
    assert tips["block_size_m"] == [0.0099, 0.0205, 0.0299] and tips["pad_size_m"] == [0.0015, 0.0205, 0.0299]
    assert 0 < tips["dynamic_friction"] <= tips["static_friction"] <= 1.2
    for key in ("static_friction", "dynamic_friction"):
        assert raw["fingertips"][key]["status"] == "unconfirmed" and raw["fingertips"][key]["source"].startswith("Estimate")
    for key in ("block_size_m", "pad_size_m"):
        assert "onrobot.com/storage/303_rg2_tool.step" in raw["fingertips"][key]["source"]
    # One motor, one driven joint; the pads cannot open to the datasheet's 110 mm.
    assert cfg["rg2"]["driven_joint"] == "finger_joint"
    low, high = cfg["rg2"]["joint_limits_deg"]
    assert -46 < low < -43 and 25 < high < 27
    assert cfg["rg2"]["pad_gap_range_m"][0] == 0.0 and 0.100 < cfg["rg2"]["pad_gap_range_m"][1] < 0.102