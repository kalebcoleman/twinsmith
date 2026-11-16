"""Unit tests for the workcell tool bundle and the puzzle graph (CPU-only: no simulator, no model)."""

import importlib.util
import json
import socket
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ("survey", "picture", "place_base", "lookahead", "approach", "close", "lift", "insert", "rescan", "release", "state")
PLAN = {"base": "plate", "order": [{"piece": "bar"}, {"piece": "cap"}]}


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def graph():
    return load(ROOT / "puzzle_graph.py")


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


def test_the_authored_graph_validates(graph, tmp_path):
    graph.author(PLAN, tmp_path / "graph")
    workflow = json.loads((tmp_path / "graph/workflow.json").read_text())
    assert list(workflow["subgraphs"]) == ["locate", "count_parts", "place_base", "assemble_bar", "assemble_cap", "judge_assembly"]
    assert workflow["subgraphs"]["assemble_cap"]["skill"] == "assembling-puzzle-pieces"
    routes = workflow["conditional_edges"]
    assert routes["assemble_bar"]["mapping"]["placed"] == "assemble_cap" and routes["assemble_bar"]["mapping"]["retry"] == "assemble_bar"
    assert routes["assemble_cap"]["mapping"]["placed"] == "judge_assembly" and routes["assemble_bar"]["mapping"]["aborted"] == "abort"
    assert sorted(p.name for p in (tmp_path / "graph/scripts").glob("*.py")) == ["cross_check.py", "verify_grip.py", "verify_reach.py"]
    assert sorted(p.stem for p in (tmp_path / "graph/checkpoints").glob("*.py")) == ["assemble_bar", "assemble_cap", "place_base"]
    registries = [arg for registry in (ROOT, graph.stock_scripts().parents[2]) for arg in ("--skills", str(registry))]
    checked = subprocess.run([str(Path(sys.executable).parent / "gap"), "run", str(tmp_path / "graph"), "--validate-only", *registries],
                             capture_output=True, text=True, env={"WORKCELL_SKILLS_PORT": "1", "PATH": "/usr/bin:/bin"})
    assert checked.returncode == 0 and "0 errors" in checked.stdout, checked.stdout + checked.stderr


def test_a_checkpoint_measures_a_piece_against_its_slot(graph, tmp_path):
    graph.author(PLAN, tmp_path / "graph")
    seated = load(tmp_path / "graph/checkpoints/assemble_bar.py").CHECKPOINTS[0]
    at = np.eye(4)
    at[:3, 3] = [0.1, -0.46, 0.03]
    off = at.copy()
    off[0, 3] += 0.004
    cell = graph.Cell()
    for bar, passes in ((at, True), (off, False)):
        cell.state = lambda bar=bar: {"bodies": {"bar": bar.tolist()}, "goals": {"slot_bar": at.tolist()}}
        assert seated.predicate(cell.world_snapshot()) is passes
    assert seated.name == "bar_seated" and seated.subgraph == "assemble_bar"
    assert set(cell.tool_registry.runtime_tools()) >= {"robot.get_ee_pose", "robot.get_gripper", "robot.describe_gripper"}
