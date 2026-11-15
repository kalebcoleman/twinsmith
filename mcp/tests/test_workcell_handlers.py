"""CPU-only Jenga handler and public tool contract checks."""

import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from twinsmith.tool_groups import GROUPS
from twinsmith.tool_hints import CATALOG_HINTS


ROOT = Path(__file__).resolve().parents[2]


def load_commands():
    spec = importlib.util.spec_from_file_location(
        "twin_commands", ROOT / "exts/workcell/workcell/commands.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Commands()


@pytest.mark.parametrize("arguments,expected", [
    ({}, {"profile": None, "seed": None}),
    ({"profile": "measured_pending", "seed": 42},
     {"profile": "measured_pending", "seed": 42}),
    ({"seed": 0}, {"profile": None, "seed": 0}),
])
def test_jenga_build_forwards_overrides(arguments, expected):
    commands = load_commands()
    report = {"jenga": {"variation_seed": expected["seed"]}}
    build = Mock(return_value=report)
    commands.workcell = SimpleNamespace(build_jenga=build)
    assert commands._cmd_jenga_build(**arguments) is report
    build.assert_called_once_with(**expected)


def test_jenga_build_preserves_workcell_errors():
    commands = load_commands()
    commands.workcell = SimpleNamespace(build_jenga=Mock(side_effect=ValueError("invalid profile")))
    with pytest.raises(ValueError, match="invalid profile"):
        commands._cmd_jenga_build(profile="unknown")


def test_jenga_registry_catalog_group_and_hints_agree():
    tree = ast.parse((ROOT / "exts/workcell/workcell/extension.py").read_text())
    registry = next(ast.literal_eval(node.value) for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "MCP_TOOL_REGISTRY"
                            for target in node.targets))
    assert len(registry) == 9
    schema = registry["jenga_build"]["parameters"]
    assert {name: field["type"] for name, field in schema.items()} == {
        "profile": "string", "seed": "integer"}
    assert not any(field.get("required", False) for field in schema.values())
    catalog = json.loads((ROOT / "mcp/twinsmith/catalogs/workcell.json").read_text())
    assert catalog["tools"]["workcell.jenga_build"]["parameters"] == schema
    groups = {name: group for name, group in GROUPS.items() if name.startswith("workcell__")}
    assert len(groups) == 4
    assert groups["workcell__jenga"][1] == {
        "build": "workcell__jenga_build", "status": "workcell__jenga_status"}
    assert CATALOG_HINTS["workcell.jenga_build"]["destructiveHint"] is True
    assert CATALOG_HINTS["workcell.jenga_build"]["readOnlyHint"] is False
