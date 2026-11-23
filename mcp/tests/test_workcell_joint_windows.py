"""CPU-only checks of the work cell's joint windows (scripts/workcell_session.py): the limits-file schemas, the cell
config setting, and the refusals before anything reaches a bridge."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location("workcell_session", ROOT / "scripts/workcell_session.py")
session = importlib.util.module_from_spec(spec)
sys.modules["workcell_session"] = session
spec.loader.exec_module(session)

ARM = list(session.ARM)
RAW_MIN = [-40, -180, 0, -209.01, -139.98, -120]
RAW_MAX = [200, -15, 160, -43.17, 130.10, 200]
MARGINS = [0, 0, 0, 10, 10, 10]


def limits_v1(low=-10, high=10):
    return {"schema": "gap_bridge.cable_limits/1", "measured": False, "inset_margin_deg": 0, "joint_names": ARM,
            "units": {"angles": "deg", "boxes": "m"}, "joint_windows_deg": {"min": [low] * 6, "max": [high] * 6}}


def limits_v2():
    measurements = {joint: {"measured": margin > 0, "margin_deg": margin, **({"source": "test"} if margin else {})}
                    for joint, margin in zip(ARM, MARGINS)}
    return {"schema": "gap_bridge.cable_limits/2", "measured": False, "joint_names": ARM,
            "units": {"angles": "deg", "boxes": "m"}, "keep_out": [],
            "joint_windows_deg": {"min": RAW_MIN, "max": RAW_MAX, "measurements": measurements, "status": "unconfirmed", "source": "test"}}


def write(tmp_path, name, data):
    path = tmp_path / name
    path.write_text(json.dumps(data))
    return path


def bare():
    return object.__new__(session.Session)          # no bridge: nothing below may reach one


def test_v1_windows_are_used_as_written_and_refuse_before_any_bridge_call(tmp_path):
    arm = bare()
    arm.set_joint_windows(write(tmp_path, "cable.json", limits_v1()))
    assert arm.allowed_targets({ARM[0]: math.radians(-10)})
    assert not arm.allowed_targets({ARM[0]: math.radians(-10.5)})
    with pytest.raises(session.MoveFailed, match="nothing sent"):
        arm.drive([{}] * 100 + [{ARM[3]: float("nan")}], 1)
    with pytest.raises(session.MoveFailed, match="scene unchanged"):
        arm.build("any_task", [11] * 6, 0.05)


def test_v2_insets_each_joint_by_its_own_margin(tmp_path):
    path = write(tmp_path, "cable.json", limits_v2())
    windows, source = session.read_joint_windows(path)
    assert windows["min"] == pytest.approx([low + margin for low, margin in zip(RAW_MIN, MARGINS)])
    assert windows["max"] == pytest.approx([high - margin for high, margin in zip(RAW_MAX, MARGINS)])
    assert source["schema"] == "gap_bridge.cable_limits/2" and len(source["sha256"]) == 64
    arm = bare()
    arm.set_cable_limits(path)                       # the older name still works
    assert arm.allowed_targets({ARM[3]: math.radians(-199)}) and not arm.allowed_targets({ARM[3]: math.radians(-200)})


@pytest.mark.parametrize("change, match", [
    (lambda c: c.update(joint_names=ARM[::-1]), "controller order"),
    (lambda c: c.update(schema="gap_bridge.cable_limits/3"), "schema"),
    (lambda c: c["joint_windows_deg"]["measurements"].pop(ARM[0]), "six joints"),
    (lambda c: c["joint_windows_deg"]["measurements"][ARM[5]].update(margin_deg=-1), "margin_deg"),
    (lambda c: c["joint_windows_deg"]["measurements"][ARM[4]].update(margin_deg=200), "empty"),
    (lambda c: c["joint_windows_deg"].update(min=RAW_MIN[:5]), "six finite"),
    (lambda c: c["joint_windows_deg"].update(max=[*RAW_MAX[:2], 190, *RAW_MAX[3:]]), "inside the joint's range"),
])
def test_bad_limits_files_are_refused(tmp_path, change, match):
    cable = limits_v2()
    change(cable)
    with pytest.raises(ValueError, match=match):
        session.read_joint_windows(write(tmp_path, "cable.json", cable))


def test_cell_config_null_inline_and_limits_file(tmp_path):
    cell = lambda value: {"robot": {"joint_windows_deg": {"value": value, "status": "confirmed", "source": "test"}}}
    assert session.read_joint_windows(write(tmp_path, "none.json", cell(None)))[0] is None
    inline, source = session.read_joint_windows(write(tmp_path, "inline.json", cell({"min": [-5] * 6, "max": [5] * 6})))
    assert inline == {"min": [-5.0] * 6, "max": [5.0] * 6} and source["schema"] == "inline"
    (tmp_path / "limits").mkdir()
    write(tmp_path, "limits/cable.json", limits_v2())
    windows, source = session.read_joint_windows(write(tmp_path, "cell.json", cell({"limits_file": "limits/cable.json"})))
    assert windows["min"][3] == pytest.approx(-199.01) and source["via"].endswith("cell.json")
    with pytest.raises(ValueError, match="joint_windows_deg"):
        session.read_joint_windows(write(tmp_path, "bad.json", cell({"min": [5] * 6, "max": [-5] * 6})))


def test_the_base_cell_sets_no_windows():
    windows, source = session.read_joint_windows(session.CELL_CONFIG)
    assert windows is None and source["path"] == str(session.CELL_CONFIG.resolve())
    arm = bare()
    arm.set_joint_windows(session.CELL_CONFIG)
    assert arm.joint_windows is None and arm.inside_windows([4.0] * 6, dict.fromkeys(ARM, 0.0)) == [4.0] * 6


def test_inside_windows_turns_a_joint_to_its_equivalent_in_the_window(tmp_path):
    arm = bare()
    arm.set_joint_windows(write(tmp_path, "cable.json", limits_v2()))
    near = dict(zip(ARM, map(math.radians, [80, -100, 110, -90, 0, 0])))
    # wrist 3's inset window is -110..190 degrees: -175 is outside it, and its equivalent +185 inside.
    joints = [math.radians(value) for value in (80, -100, 110, -90, 0, -175)]
    fitted = arm.inside_windows(joints, near)
    assert math.degrees(fitted[5]) == pytest.approx(185)
    assert fitted[:5] == pytest.approx(joints[:5])
    joints[4] = math.radians(125)                    # wrist 2 past 120 (130.1 - 10) and no equivalent inside
    assert arm.inside_windows(joints, near) is None
