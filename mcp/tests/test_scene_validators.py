"""Scene validation scripts wait on simulated time and name what is missing."""

import asyncio
import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _script(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_test", ROOT / "scripts" / "smoke" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _validator(name):
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        return _script(name)
    finally:
        sys.path.pop(0)


def test_sensor_validator_waits_on_simulated_time_with_a_bound(monkeypatch):
    sensors = _validator("validate_scene_sensors")
    times = iter([0.2, 0.6, 3.1])

    async def invoke(name, **args):
        assert name == "sim__timeline" and args["action"] == "play"
        return dict(time_s=next(times))

    assert asyncio.run(sensors.wait_simulated(invoke, {}, 3, poll=0))["time_s"] == 3.1

    async def stuck(name, **args):
        return dict(time_s=0.1)

    with pytest.raises(AssertionError, match="only 0.1 s simulated"):
        asyncio.run(sensors.wait_simulated(stuck, {}, 3, wall_limit=0, poll=0))


def test_equivalents_validator_names_the_missing_root():
    equivalents = _validator("validate_scene_equivalents")
    equivalents.require_root({"roots": [{"root_id": "agilebot"}]}, "agilebot")
    with pytest.raises(SystemExit, match="VENDORING.md"):
        equivalents.require_root({"roots": [{"root_id": "fixture"}]}, "agilebot")


def test_tool_listing_check_compares_the_listing_with_the_toolset_registry():
    tools = _validator("validate_scene_tools")
    sys.path.insert(0, str(ROOT / "mcp"))
    from twinsmith import config, toolsets

    default = config.select("default,+sim,+assets")
    listed = asyncio.run(tools.promised_tools(default))
    assert {"sim__prim", "run_python", "status", "sim__logs", "isaaclab__launch", "workcell__cell"} <= listed
    assert not {"computer", "hardware"} & listed
    assert asyncio.run(tools.check_registry(listed, default))["listed"] == len(listed)
    with pytest.raises(AssertionError, match="listed_but_not_promised"):
        asyncio.run(tools.check_registry(listed | {"hardware"}, default))
    with pytest.raises(AssertionError, match="promised_but_not_listed"):
        asyncio.run(tools.check_registry(listed - {"sim__prim"}, default))
    assert toolsets.owner("sim__prim") in default
