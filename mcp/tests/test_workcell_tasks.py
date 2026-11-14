"""CPU-only checks for the split between the base cell and the task packs loaded on top of it."""

import copy
import importlib.util
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "exts/workcell/workcell"


@pytest.fixture()
def cell(monkeypatch):
    package = types.ModuleType("workcell_tasks_test")
    package.__path__ = [str(PACKAGE)]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    loaded = {}
    for name in ("hardware_config", "bom", "asset_pack", "jenga", "tasks", "scene"):
        spec = importlib.util.spec_from_file_location(f"{package.__name__}.{name}", PACKAGE / f"{name}.py")
        loaded[name] = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, spec.name, loaded[name])
        spec.loader.exec_module(loaded[name])
    return types.SimpleNamespace(**loaded)


def test_base_cell_holds_no_task_objects(cell):
    config, raw = cell.hardware_config.load_config()
    assert "jenga" not in config and "jenga_block" not in config["assets"]
    assert config["pack"] == {"id": "ur5e-workcell", "version": "v3"}
    assert "jenga_block" not in cell.scene.ROLES
    rows = cell.bom.load_bom()
    assert not [row for row in rows if row["area"] == "jenga"]
    assert cell.bom.verify_config_assets(config, rows) == {row["asset_id"] for row in rows}


def test_a_cell_configuration_with_task_objects_is_refused(cell, tmp_path):
    import json

    raw = json.loads(cell.hardware_config.DEFAULT_CONFIG.read_text())
    raw["jenga"] = json.loads((cell.tasks.TASKS / "jenga_tower.json").read_text())["jenga"]
    path = tmp_path / "cell.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="must not hold task objects"):
        cell.hardware_config.load_config(path)


def test_task_names_and_refusals(cell):
    assert cell.tasks.names() == ["jenga_tower", "push_block"]
    for bad in ("no_such_task", "../jenga_tower", "", None):
        with pytest.raises(ValueError, match="task"):
            cell.tasks.load(bad)


@pytest.mark.parametrize("name,blocks", [("jenga_tower", 54), ("push_block", 1)])
def test_task_files_carry_their_own_pack_parameters_and_bom(cell, name, blocks):
    values, _ = cell.hardware_config.read_values(cell.tasks.TASKS / f"{name}.json")
    assert set(values) == cell.tasks.SECTIONS
    assert values["pack"] == {"id": "jenga", "version": "v1"} and values["assets"] == {"jenga_block": "jenga_block"}
    assert len(cell.jenga.realize(values)["blocks"]) == cell.jenga.block_count(values) == blocks
    rows = cell.bom.load_bom(cell.tasks.TASKS / values["documentation"]["bom_csv"])
    assert [(row["bom_id"], row["quantity"]) for row in rows] == [("jenga_block", blocks)]
    try:
        task = cell.tasks.load(name)
    except cell.asset_pack.PackError as error:
        pytest.skip(f"task pack unavailable: {error}")
    assert task["name"] == name and task["asset"].name == "jenga_block.usda" and task["rows"] == rows
    config, _ = cell.hardware_config.load_config()
    merged = cell.tasks.merged(config, task)
    assert merged["jenga"] == values["jenga"] and merged["assets"]["jenga_block"] == "jenga_block"
    assert "jenga" not in config and "jenga_block" not in config["assets"]      # the cell itself is not changed
    combined = cell.bom.load_bom() + task["rows"]
    assert cell.bom.verify_config_assets(merged, combined) == {row["asset_id"] for row in combined}


def test_loose_blocks_lie_flat_where_the_task_puts_them(cell):
    values, _ = cell.hardware_config.read_values(cell.tasks.TASKS / "push_block.json")
    (block,) = cell.jenga.realize(values)["blocks"]
    x, y, yaw = values["jenga"]["block_poses"][0]
    assert block["position_m"] == pytest.approx([x, y, 0.007]) and block["rotation_z_deg"] == yaw
    both = copy.deepcopy(values)
    both["jenga"]["tower_layers_built"] = 1
    with pytest.raises(ValueError, match="cannot both"):
        cell.jenga.realize(both)
    for bad in ([], [[0.0, 0.0]], "x"):
        broken = copy.deepcopy(values)
        broken["jenga"]["block_poses"] = bad
        with pytest.raises(ValueError, match="block_poses"):
            cell.jenga.realize(broken)


def test_a_task_bom_must_match_what_the_task_places(cell, tmp_path):
    import json
    import shutil

    for name in ("push_block.json", "jenga_tower.bom.csv"):
        shutil.copy(cell.tasks.TASKS / name, tmp_path / name)
    raw = json.loads((tmp_path / "push_block.json").read_text())
    raw["documentation"]["bom_csv"]["value"] = "jenga_tower.bom.csv"      # 54 blocks for a task that places one
    (tmp_path / "push_block.json").write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="number of blocks"):
        cell.tasks.load(str(tmp_path / "push_block.json"))
