import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]


def load_commands():
    spec = importlib.util.spec_from_file_location("workcell_commands", ROOT / "exts/workcell/workcell/commands.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Commands()


def test_workcell_forwarding_and_reports():
    commands = load_commands()
    calls = []

    async def quick_start(**kwargs):
        calls.append(kwargs)
        return {"state": "ready"}

    report = {"state": "ready", "robot": {"name": "arm"}, "cameras": ["camera"], "jenga": {"layers": 18}}
    commands.workcell = SimpleNamespace(quick_start=quick_start, status=lambda: report,
                                        shutdown=lambda: {"stage_preserved": True})
    summary = asyncio.run(commands._cmd_workcell_quick_start("config.json", "mirror_real", True))
    assert summary["state"] == "ready" and "full_report" in summary     # a summary; status has the whole report
    assert calls == [{"config_path": "config.json", "mode": "mirror_real", "discard_changes": True, "environment": "none",
                      "task": None}]
    assert commands._cmd_workcell_status() == report
    assert commands._cmd_robot_status()["robot"] == report["robot"]
    assert commands._cmd_perception_status()["cameras"] == report["cameras"]
    assert commands._cmd_jenga_status()["jenga"] == report["jenga"]
    assert commands._cmd_workcell_shutdown() == {"stage_preserved": True}


def test_the_work_cell_has_four_grouped_tools_and_no_stubs():
    import sys
    sys.path.insert(0, str(ROOT / "mcp"))
    from twinsmith.tool_groups import GROUPS
    commands = load_commands()
    for name in ("real2sim", "planning", "eval"):
        assert not hasattr(commands, f"_cmd_{name}_status")
    assert sorted(name for name in GROUPS if name.startswith("workcell__")) == [
        "workcell__cell", "workcell__jenga", "workcell__perception", "workcell__robot"]
