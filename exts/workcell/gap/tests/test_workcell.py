"""Unit tests for the workcell tool bundle and the work-cell graph connector (CPU-only: no simulator, no model)."""

import importlib.util
import json
import socket
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ("scan", "survey", "picture", "place_base", "lookahead", "approach", "close", "lift", "insert", "rescan", "release", "state")


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def connector():
    return load(ROOT / "workcell_graph.py")


def test_every_tool_is_registered_with_a_typed_signature(tool_registry):
    for name in TOOLS:
        described = tool_registry.get(f"workcell.{name}")
        assert described.summary and described.schema.outputs


def test_a_tool_asks_the_cell_host_and_raises_what_it_refuses(tool_registry, monkeypatch):
    bundle = sys.modules[tool_registry.get("workcell.survey").fn.__module__] if hasattr(tool_registry.get("workcell.survey"), "fn") \
        else load(ROOT / "tools/workcell/tools.py")
    monkeypatch.setattr(bundle, "_link", None)
    monkeypatch.delenv("WORKCELL_SKILLS_PORT", raising=False)
    with pytest.raises(RuntimeError, match="WORKCELL_SKILLS_PORT"):
        bundle.ask("release")
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    asked = []

    def host():
        stream = server.accept()[0].makefile("rw")
        for line in stream:
            asked.append(json.loads(line))
            reply = {"ok": True, "result": {"result": "open"}} if asked[-1]["tool"] == "release" else {"ok": False, "error": "no grasp of bar fits"}
            stream.write(json.dumps(reply) + "\n")
            stream.flush()

    threading.Thread(target=host, daemon=True).start()
    monkeypatch.setenv("WORKCELL_SKILLS_PORT", str(server.getsockname()[1]))
    assert bundle.ask("release") == {"result": "open"}
    with pytest.raises(RuntimeError, match="no grasp of bar fits"):
        bundle.ask("approach", piece="bar")
    assert asked == [{"tool": "release", "inputs": {}}, {"tool": "approach", "inputs": {"piece": "bar"}}]


def test_the_cross_check_reads_the_models_count():
    check = load(ROOT / "scripts/cross_check.py")
    assert check.run(None, answer="5\nA green base, two red bars and two forks.", found=5)["vlm_agrees"] is True
    assert check.run(None, answer="I see 4 parts.", found=5) == {"vlm_count": 4, "scan_count": 5, "vlm_agrees": False, "answer": "I see 4 parts."}
    assert check.run(None, answer="Several.", found=5)["vlm_count"] == -1


def test_the_router_takes_the_scans_next_step():
    assert load(ROOT / "scripts/next_step.py").run(None, next="bar") == {"route": "bar"}


def test_the_connector_reads_the_robot_and_the_truth_through_the_cells_state(connector):
    at = np.eye(4)
    at[:3, 3] = [0.1, -0.46, 0.03]
    cell = connector.Cell()
    pose = {"position": {"x": 0.1, "y": -0.46, "z": 0.2}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
    cell.state = lambda: {"ee_pose": pose, "gripper": {"position": 0.5}, "bodies": {"bar": at.tolist()}, "goals": {"slot_bar": at.tolist()}}
    assert cell.get_ee_pose() == pose and cell.get_gripper() == {"position": 0.5}
    assert cell.describe_gripper()["span_m"] == connector.RG2_STROKE_M
    world = cell.world_snapshot()
    assert world.body("bar").distance_to(world.body("slot_bar")) == 0.0
    assert np.allclose(world.body("bar").position, [0.1, -0.46, 0.03])
    assert set(cell.tool_registry.runtime_tools()) >= {"robot.get_ee_pose", "robot.get_gripper", "robot.describe_gripper"}
    assert sorted(p.name for p in connector.SCRIPTS.glob("*.py")) == ["cross_check.py", "next_step.py"]
