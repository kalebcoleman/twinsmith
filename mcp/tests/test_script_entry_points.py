"""Host scripts: the scene client keeps exported tool groups, and smoke usage errors never reach the server."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _script(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_test", ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scene_client_keeps_exported_tool_groups_and_still_pins_the_port(monkeypatch):
    client = _script("scene_mcp_client")
    monkeypatch.delenv("TWINSMITH_TOOLSETS", raising=False)
    monkeypatch.setenv("TWINSMITH_PORT", "1")
    env = client.parameters(18781).env
    assert env["TWINSMITH_TOOLSETS"] == "default,+sim,+assets"
    assert env["TWINSMITH_PORT"] == "18781"
    monkeypatch.setenv("TWINSMITH_TOOLSETS", "default,+computer")
    assert client.parameters(18781).env["TWINSMITH_TOOLSETS"] == "default,+computer"


def test_scene_client_sends_single_purpose_names_as_the_listed_group_with_an_action(monkeypatch):
    client = _script("scene_mcp_client")
    assert client.listed("sim__prim__create", {"path": "/World/A"}) == ("sim__prim", {"action": "create", "path": "/World/A"})
    assert client.listed("sim__body__state", {"path": "/p"}) == ("sim__physics", {"action": "body_state", "path": "/p"})
    assert client.listed("sim__ui__act", {"action": "click"}) == ("sim__ui", {"action": "act", "sub_action": "click"})
    assert client.listed("sim__timeline", {"action": "stop"}) == ("sim__timeline", {"action": "stop"})
    monkeypatch.delenv("TWINSMITH_TOOLSETS", raising=False)
    assert client.parameters(1, "all,+computer").env["TWINSMITH_TOOLSETS"] == "all,+computer"
    monkeypatch.setenv("TWINSMITH_TOOLSETS", "default,+computer")
    assert client.parameters(1, "all").env["TWINSMITH_TOOLSETS"] == "all"


@pytest.mark.parametrize("argv, code", [(["--help"], 0), (["-h"], 0), (["live"], 2), (["typo"], 2)])
def test_smoke_surface_usage_never_reaches_the_server(monkeypatch, capsys, argv, code):
    smoke = _script("smoke_mcp_surface")
    monkeypatch.setattr(smoke, "run", lambda plan: pytest.fail("must not call tools"))
    assert smoke.main(["smoke_mcp_surface.py", *argv]) == code
    captured = capsys.readouterr()
    assert "offline" in captured.out + captured.err
