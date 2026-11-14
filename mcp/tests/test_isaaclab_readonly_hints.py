"""Effects behind Isaac Lab inspection hints, exercised through MCP without Isaac Lab."""

import asyncio
import json
import sys

import pytest
from mcp.shared.memory import create_connected_server_and_client_session

from twinsmith import local_tools, server
from twinsmith.isaaclab import jobs


class EmptyBridge:
    def __init__(self, **kwargs):
        pass

    def call(self, name, params=None):
        assert name == "list_tools"
        return {"tools": {}}

    def close(self):
        pass


@pytest.fixture
def host(monkeypatch):
    monkeypatch.setattr(server, "BridgeClient", EmptyBridge)
    return server.build_server({"TWINSMITH_TOOLSETS": "default,+sim"})


def call(host, name, arguments=None):
    async def run():
        async with create_connected_server_and_client_session(host._mcp_server) as client:
            await client.initialize()
            result = await client.call_tool(name, arguments or {})
            assert not result.isError
            return json.loads(result.content[0].text)
    return asyncio.run(run())


@pytest.mark.parametrize("name", ["isaaclab__runs_list", "isaaclab__runs_status"])
def test_job_queries_reconcile_and_persist_orphans(host, tmp_path, monkeypatch, name):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    monkeypatch.setattr(jobs, "_job_alive", lambda data: False)
    job_id = "20200101-000000-old"
    job_dir = tmp_path / "jobs" / job_id
    job_dir.mkdir(parents=True)
    record = job_dir / "job.json"
    record.write_text(json.dumps({"job_id": job_id, "status": "running", "pid": 0,
                                  "started_at": 1}))

    result = call(host, name, {"job_id": job_id} if name.endswith("status") else {})
    persisted = json.loads(record.read_text())
    assert persisted["status"] == "failed"
    assert persisted["note"] == "host restarted"
    if name.endswith("status"):
        assert result["state"] == "failed"
    else:
        assert result["runs"][0]["state"] == "failed"


def test_job_tail_can_create_the_jobs_directory(host, tmp_path, monkeypatch):
    directory = tmp_path / "missing-jobs"
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(directory))
    assert not directory.exists()
    assert "error" in call(host, "isaaclab__runs_tail", {"job_id": "20200101-000000-missing"})
    assert directory.is_dir()


def test_envs_list_imports_configured_module_with_side_effects(host, tmp_path, monkeypatch):
    root = tmp_path / "IsaacLab"
    root.mkdir()
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "isaaclab_tasks.py").write_text("")
    (modules / "gymnasium.py").write_text("registry = {}\n")
    (modules / "custom_tasks.py").write_text(
        "import os\nfrom pathlib import Path\nPath(os.environ['TEST_TASK_MARKER']).write_text('imported')\n")
    marker = tmp_path / "task-marker"
    monkeypatch.setenv("ISAACLAB_PYTHON", sys.executable)
    monkeypatch.setenv("ISAACLAB_ROOT", str(root))
    monkeypatch.setenv("ISAACLAB_TASK_MODULES", "custom_tasks")
    monkeypatch.setenv("ISAACLAB_TASK_PATH", str(modules))
    monkeypatch.setenv("TEST_TASK_MARKER", str(marker))
    result = call(host, "isaaclab__envs_list")
    assert result["envs"] == []
    assert marker.read_text() == "imported"


def test_install_probe_runs_interpreter_startup_code(host, tmp_path, monkeypatch):
    root = tmp_path / "IsaacLab"
    root.mkdir()
    marker = tmp_path / "startup-marker"
    (tmp_path / "sitecustomize.py").write_text(
        "import os\nfrom pathlib import Path\nPath(os.environ['TEST_STARTUP_MARKER']).write_text('started')\n")
    monkeypatch.setenv("ISAACLAB_PYTHON", sys.executable)
    monkeypatch.setenv("ISAACLAB_ROOT", str(root))
    monkeypatch.setenv("PYTHONPATH", str(tmp_path))
    monkeypatch.setenv("TEST_STARTUP_MARKER", str(marker))
    result = call(host, "isaaclab__install_info")
    assert result["python"] == sys.executable
    assert marker.read_text() == "started"


def test_effectful_inspection_tools_are_not_readonly_and_have_conservative_hints(host):
    tools = host._tool_manager._tools   # every registered tool, listed or grouped
    for name in ("isaaclab__install_info", "isaaclab__envs_list"):
        ann = tools[name].annotations
        assert (ann.readOnlyHint, ann.destructiveHint, ann.idempotentHint, ann.openWorldHint) == (
            False, True, False, True)
    for name in ("isaaclab__runs_list", "isaaclab__runs_status"):
        ann = tools[name].annotations
        assert (ann.readOnlyHint, ann.destructiveHint, ann.idempotentHint, ann.openWorldHint) == (
            False, True, True, False)
    tail = tools["isaaclab__runs_tail"].annotations
    assert (tail.readOnlyHint, tail.destructiveHint, tail.idempotentHint, tail.openWorldHint) == (
        False, False, True, False)
    assert not tools["isaaclab__install_info"].annotations.readOnlyHint
