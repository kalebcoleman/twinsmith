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
    for name in ("hardware_config", "bom", "asset_pack", "jenga", "puzzle", "tasks", "scene"):
        spec = importlib.util.spec_from_file_location(f"{package.__name__}.{name}", PACKAGE / f"{name}.py")
        loaded[name] = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, spec.name, loaded[name])
        spec.loader.exec_module(loaded[name])
    return types.SimpleNamespace(**loaded)


def test_base_cell_holds_no_task_objects(cell):
    config, raw = cell.hardware_config.load_config()
    assert "jenga" not in config and "jenga_block" not in config["assets"]
    assert config["pack"] == {"id": "ur5e-workcell", "version": "v4"}
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
    assert cell.tasks.names() == ["jenga_tower", "push_block", "puzzle_table"]
    for bad in ("no_such_task", "../jenga_tower", "", None):
        with pytest.raises(ValueError, match="task"):
            cell.tasks.load(bad)


@pytest.mark.parametrize("name,blocks", [("jenga_tower", 54), ("push_block", 1)])
def test_task_files_carry_their_own_pack_parameters_and_bom(cell, name, blocks):
    values, _ = cell.hardware_config.read_values(cell.tasks.TASKS / f"{name}.json")
    assert set(values) == {"pack", "assets", "documentation", "jenga"}
    assert values["pack"] == {"id": "jenga", "version": "v1"} and values["assets"] == {"jenga_block": "jenga_block"}
    assert len(cell.jenga.realize(values)["blocks"]) == cell.jenga.block_count(values) == blocks
    rows = cell.bom.load_bom(cell.tasks.TASKS / values["documentation"]["bom_csv"])
    assert [(row["bom_id"], row["quantity"]) for row in rows] == [("jenga_block", blocks)]
    try:
        task = cell.tasks.load(name)
    except cell.asset_pack.PackError as error:
        pytest.skip(f"task pack unavailable: {error}")
    assert task["name"] == name and task["kind"] == "jenga" and task["rows"] == rows
    assert {role: path.name for role, path in task["assets"].items()} == {"jenga_block": "jenga_block.usda"}
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
    with pytest.raises(ValueError, match="number of objects"):
        cell.tasks.load(str(tmp_path / "push_block.json"))


def _puzzle(cell):
    values, _ = cell.hardware_config.read_values(cell.tasks.TASKS / "puzzle_table.json")
    return values


def test_the_puzzle_task_places_every_piece_once_and_lists_it_in_its_bom(cell):
    values = _puzzle(cell)
    assert set(values) == {"pack", "assets", "documentation", "puzzle"} and values["pack"] == {"id": "puzzles", "version": "v1"}
    pieces = {piece["name"]: piece for piece in cell.puzzle.realize(values)["pieces"]}
    assert len(pieces) == 10 and set(pieces) == set(values["assets"])
    rows = cell.bom.load_bom(cell.tasks.TASKS / values["documentation"]["bom_csv"])
    assert {row["asset_id"]: row["quantity"] for row in rows} == cell.puzzle.quantities(values) == dict.fromkeys(values["assets"], 1)
    try:
        task = cell.tasks.load("puzzle_table")
    except cell.asset_pack.PackError as error:
        pytest.skip(f"task pack unavailable: {error}")
    assert task["kind"] == "puzzle" and {role: path.name for role, path in task["assets"].items()} == {
        role: f"{role}.usda" for role in values["assets"]}
    config, _ = cell.hardware_config.load_config()
    merged = cell.tasks.merged(config, task)
    assert merged["puzzle"] == values["puzzle"] and "puzzle" not in config
    combined = cell.bom.load_bom() + task["rows"]
    assert cell.bom.verify_config_assets(merged, combined) == {row["asset_id"] for row in combined}


def test_assembled_pieces_are_posed_in_their_base_and_follow_it(cell):
    values = _puzzle(cell)
    pieces = {piece["name"]: piece for piece in cell.puzzle.realize(values)["pieces"]}
    base, fork = pieces["base3"], pieces["fork_yellow"]
    assert fork["parent"] == "base3" and fork["position_m"] == pytest.approx([base["position_m"][0], base["position_m"][1] - 0.036, 0.0525])
    # Standing on edge: the fork's own -y (its centre tab) points up and its thickness, its own z, lies along the world's y.
    assert [row[1] for row in fork["rotation"]] == pytest.approx([0, 0, -1], abs=1e-12)
    assert [row[2] for row in fork["rotation"]] == pytest.approx([0, 1, 0], abs=1e-12)
    turned = copy.deepcopy(values)
    turned["puzzle"]["pieces"][0].update(position_m=[0.3, -0.5, 0.0175], rpy_deg=[0, 0, 90])
    moved = {piece["name"]: piece for piece in cell.puzzle.realize(turned)["pieces"]}
    assert moved["fork_yellow"]["position_m"] == pytest.approx([0.3 + 0.036, -0.5, 0.0525])
    assert moved["base1"]["position_m"] == pieces["base1"]["position_m"]              # no parent: it stays in the world


@pytest.mark.parametrize("change,message", [
    (lambda p: p["pieces"].append(dict(p["pieces"][0])), "used once"),
    (lambda p: p["pieces"][1].update(parent="fork_yellow"), "not an earlier piece"),
    (lambda p: p["pieces"][0].update(asset="no_such_role"), "does not map"),
    (lambda p: p["pieces"][0].update(position_m=[0.0, 0.0]), "three finite numbers"),
    (lambda p: p["pieces"][0].update(scale=2), "may name a parent"),
    (lambda p: p.update(density_kg_m3=0), "density"),
    (lambda p: p.update(dynamic_friction=0.9), "must not exceed"),
    (lambda p: p.update(pieces=[]), "1 to 64"),
    (lambda p: p.pop("static_friction"), "exactly"),
])
def test_a_broken_puzzle_section_is_refused(cell, change, message):
    values = copy.deepcopy(_puzzle(cell))
    change(values["puzzle"])
    with pytest.raises(ValueError, match=message):
        cell.puzzle.realize(values)


def test_a_task_is_of_exactly_one_kind(cell, tmp_path):
    import json
    import shutil

    for name in ("puzzle_table.json", "puzzle_table.bom.csv"):
        shutil.copy(cell.tasks.TASKS / name, tmp_path / name)
    raw = json.loads((tmp_path / "puzzle_table.json").read_text())
    raw["jenga"] = json.loads((cell.tasks.TASKS / "push_block.json").read_text())["jenga"]
    (tmp_path / "puzzle_table.json").write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="one of"):
        cell.tasks.load(str(tmp_path / "puzzle_table.json"))


def test_puzzle_pieces_are_built_as_bodies_with_the_pack_colliders_and_mass_from_volume(cell, tmp_path):
    pxr = pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    try:
        task = cell.tasks.load("puzzle_table")
    except cell.asset_pack.PackError as error:
        pytest.skip(f"task pack unavailable: {error}")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    report = cell.tasks.build(stage, task["assets"], {"assets": task["values"]["assets"], "puzzle": task["values"]["puzzle"]}, task)
    assert report["piece_count"] == 10 and [part["bom_id"] for part in report["parts"]] == list(task["values"]["assets"])
    pieces = {piece["name"]: piece for piece in report["pieces"]}
    assert pieces["base3"]["colliders"] == 22 and pieces["fork_yellow"]["colliders"] == 5 and pieces["line_red"]["colliders"] == 1
    line = stage.GetPrimAtPath("/World/Puzzle/line_red")
    assert line.HasAPI(UsdPhysics.RigidBodyAPI)
    assert UsdPhysics.MassAPI(line).GetMassAttr().Get() == pytest.approx(0.0259 * 0.0259 * 0.140 * 1240.0, rel=1e-5)
    material, _ = UsdShade.MaterialBindingAPI(line).ComputeBoundMaterial(materialPurpose="physics")
    assert UsdPhysics.MaterialAPI(material.GetPrim()).GetStaticFrictionAttr().Get() == pytest.approx(0.4)
    # The fork stands in the base: its colliders span 0 to 70 mm above the base's centre plane, 21 mm thick along y.
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["guide"]).ComputeWorldBound(stage.GetPrimAtPath("/World/Puzzle/fork_yellow")).ComputeAlignedRange()
    assert tuple(box.GetSize()) == pytest.approx((0.1092, 0.021, 0.070), abs=1e-6) and box.GetMin()[2] == pytest.approx(0.0175, abs=1e-6)
    with pytest.raises(ValueError, match="already exists"):
        cell.tasks.build(stage, task["assets"], {"assets": task["values"]["assets"], "puzzle": task["values"]["puzzle"]}, task)
