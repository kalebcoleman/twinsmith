"""Smoke routing and engine guards never require a simulator in unit tests."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "smoke.py"


def runner():
    spec = importlib.util.spec_from_file_location("smoke_runner", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_list_is_offline():
    result = subprocess.run([sys.executable, "-S", str(SCRIPT), "all", "--list"],
                            capture_output=True, text=True, check=True)
    assert "Running simulator" in result.stdout and "GPU" in result.stdout
    for checks in runner().TOOLSETS.values():
        for check in checks:
            assert check in result.stdout
    assert "no checks registered" in result.stdout


def test_list_needs_no_toolset_and_lists_all():
    result = subprocess.run([sys.executable, "-S", str(SCRIPT), "--list"],
                            capture_output=True, text=True, check=True)
    for toolset in runner().TOOLSETS:
        assert toolset in result.stdout


def test_a_run_without_a_toolset_is_refused():
    result = subprocess.run([sys.executable, "-S", str(SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 2 and "toolset" in result.stderr


@pytest.mark.parametrize("actual,requested", [("physx", "newton"), ("newton", "physx"), ("unknown", "physx")])
def test_backend_mismatch_refuses_before_checks(tmp_path, monkeypatch, capsys, actual, requested):
    module = runner()
    monkeypatch.setattr(module, "PROBE_TIMEOUT_S", 0)
    calls = []

    class Client:
        def __init__(self, host, port):
            assert host == "test-host"
            self.port = port

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def call(self, name, arguments):
            calls.append(name)
            return {"app_version": "test-build", "pid": 123} if name == "status" else {"runtime": {"physics_engine": actual}}

    def forbidden(*args):
        pytest.fail("A validator ran despite backend mismatch")

    monkeypatch.setattr(module, "execute", forbidden)
    monkeypatch.setenv("TWINSMITH_HOST", "original")
    assert module.main(["sim", "--host", "test-host", "--backend", requested,
                        "--output-dir", str(tmp_path)], client_factory=Client) == 2
    assert calls == ["status", "sim.stage.info"]
    assert "refusing to run" in capsys.readouterr().out
    result = json.loads((tmp_path / "results.json").read_text())
    first = result["checks"][0]
    assert first["status"] == "fail" and first["backend"] == actual
    assert first["simulator_build"] == "test-build" and first["seconds"] >= 0
    assert all(row["status"] == "not_run" for row in result["checks"][1:] if row["check"] not in module.ON_REQUEST)
    assert [row["status"] for row in result["checks"] if row["check"] in module.ON_REQUEST] == ["skipped", "skipped"]
    assert module.os.environ["TWINSMITH_HOST"] == "original"


def test_success_records_actual_backend(tmp_path, monkeypatch):
    module = runner()

    class Client:
        port = 8766

        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def call(self, name, arguments):
            return {"app_version": "test-build"} if name == "status" else {"runtime": {"physics_engine": "newton"}}

    executed = []
    monkeypatch.setattr(module, "execute", lambda name, *args: executed.append(name))
    assert module.main(["sim", "--backend", "newton", "--output-dir", str(tmp_path), "--check", "validate_scene_joints",
                        "--check", "validate_scene_edges", "--check", "newton_checks"], Client) == 0
    rows = json.loads((tmp_path / "results.json").read_text())["checks"]
    assert executed == ["validate_scene_joints", "validate_scene_tools", "validate_scene_equivalents",
                        "validate_scene_edges", "newton_checks"]  # edges works on the stage the others leave
    assert all(row["status"] == "pass" and row["backend"] == "newton" for row in rows)


def test_generic_module_import_does_not_start_kit():
    module = runner().load_check("validate_generic_scene")
    assert callable(module.run_checks)


class _Engine:
    """A bridge whose engine reading follows a script, the last value repeating."""

    port = 8766

    def __init__(self, readings):
        self.readings, self.calls = list(readings), 0

    def call(self, name, arguments):
        if name == "status":
            return {"app_version": "test-build", "pid": 1}
        self.calls += 1
        return {"runtime": {"physics_engine": self.readings.pop(0) if len(self.readings) > 1 else self.readings[0]}}


def test_probe_waits_for_the_engine_to_settle_on_the_requested_one():
    module, slept = runner(), []
    client = _Engine(["physx", "physx", "newton", "newton"])  # the bridge answers before Newton is the engine
    ok = module.probe(client, "newton", timeout=60, interval=1, sleep=slept.append, clock=lambda: len(slept))
    assert ok["backend"] == "newton" and slept == [1, 1, 1]


def test_probe_refuses_a_mismatch_only_after_the_bound_and_a_transient_error_is_retried():
    module, slept = runner(), []
    with pytest.raises(module.BackendMismatch, match="reports 'physx'"):
        module.probe(_Engine(["physx"]), "newton", timeout=5, interval=1, sleep=slept.append, clock=lambda: len(slept))
    assert len(slept) == 5

    class Starting(_Engine):
        def call(self, name, arguments):
            if name == "sim.stage.info" and not self.calls:
                self.calls += 1
                raise RuntimeError("starting")
            return super().call(name, arguments)

    assert module.probe(Starting(["physx"]), None, timeout=5, interval=1, sleep=lambda s: None)["backend"] == "physx"


def test_checks_that_share_a_stage_run_together_in_order_with_their_prerequisites():
    module = runner()
    assert module.plan(["validate_scene_clear", "validate_scene_tools", "validate_scene_equivalents"]) == [
        "validate_scene_tools", "validate_scene_equivalents", "validate_scene_clear"]
    assert module.plan(["validate_scene_clear"]) == ["validate_scene_tools", "validate_scene_equivalents", "validate_scene_clear"]
    everything = [name for names in module.TOOLSETS.values() for name in names]
    order = module.plan(everything)
    assert sorted(order) == sorted(everything)
    suite = [name for name in order if name in module.SUITE]
    assert suite == [name for name in module.SUITE if name in order] and suite[-1] == "validate_scene_clear"
    first = order.index(suite[0])
    assert order[first:first + len(suite)] == suite  # nothing between the checks that share a stage
    for name in order:
        for need in module.PREREQUISITE.get(name, ()):
            assert order.index(need) < order.index(name)


class _Bridge(_Engine):
    def __init__(self, engine="physx"):
        super().__init__([engine])
        self.stops = self.new_stages = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def call(self, name, arguments):
        if name == "sim.stage.info":
            return {"stopped": False, "session_id": "s", "stage_id": "t", "runtime": {"physics_engine": self.readings[0]}}
        if name == "run_python" and "new_stage" in arguments["code"]:
            self.new_stages += 1
            return {"result": True}
        if name == "sim.timeline":
            self.stops += 1
            assert arguments == {"action": "stop", "session_id": "s", "stage_id": "t"}
            return {}
        return super().call(name, arguments)


def test_each_independent_check_starts_stopped_but_a_prerequisite_pair_is_not_reset_between(tmp_path, monkeypatch):
    module = runner()
    monkeypatch.delenv("TWINSMITH_TOOLSETS", raising=False)
    bridge, ran, toolsets = _Bridge(), [], {}
    monkeypatch.setattr(module, "execute", lambda name, *a: (ran.append(name), toolsets.update({name: module.os.environ["TWINSMITH_TOOLSETS"]})))
    assert module.main(["all", "--discard-changes", "--second-port", "1", "--directory", "d", "--output-dir", str(tmp_path),
                        "--check", "validate_scene_clear", "--check", "validate_scene_prim_edit"], lambda **k: bridge) == 0
    assert ran == ["validate_scene_prim_edit", "validate_scene_tools", "validate_scene_equivalents", "validate_scene_clear"]
    assert bridge.stops == 4 and bridge.new_stages == 2  # each stopped; new stage for prim_edit and for the head of the suite
    assert toolsets["validate_scene_tools"] == "all" and toolsets["validate_scene_equivalents"] == "default,+sim,+assets"
    assert "TWINSMITH_TOOLSETS" not in module.os.environ


def test_a_failure_is_reported_with_its_leaf_exception_and_the_run_continues(tmp_path, monkeypatch, capsys):
    module, ran = runner(), []

    def execute(name, *a):
        ran.append(name)
        if name == "validate_scene_equivalents":
            raise BaseExceptionGroup("unhandled errors in a TaskGroup", [AssertionError({"isError": True})])
        if name == "validate_scene_prim_edit":
            raise SystemExit("needs the fixture")

    monkeypatch.setattr(module, "execute", execute)
    code = module.main(["sim", "--output-dir", str(tmp_path), "--discard-changes", "--directory", "d",
                        "--check", "validate_scene_tools", "--check", "validate_scene_clear", "--check", "validate_scene_prim_edit"],
                       lambda **k: _Bridge())
    rows = {row["check"]: row for row in json.loads((tmp_path / "results.json").read_text())["checks"]}
    assert code == 1 and ran == ["validate_scene_tools", "validate_scene_equivalents", "validate_scene_prim_edit"]
    assert rows["validate_scene_prim_edit"]["error"] == "SystemExit: needs the fixture"
    assert rows["validate_scene_equivalents"]["error"].startswith("AssertionError: {'isError': True}")
    assert "TaskGroup" not in rows["validate_scene_equivalents"]["error"]
    assert rows["validate_scene_clear"]["status"] == "not_run" and "validate_scene_equivalents" in rows["validate_scene_clear"]["error"]
    assert rows["validate_scene_tools"]["status"] == "pass"
    assert "AssertionError" in (tmp_path / "validate_scene_equivalents.traceback.txt").read_text()
    assert "leaf exception" in capsys.readouterr().err


def test_reopen_without_a_saved_session_is_reported_with_the_command_not_skipped(tmp_path, monkeypatch):
    module, ran = runner(), []
    monkeypatch.setattr(module, "execute", lambda name, *a: ran.append(name))
    assert module.main(["sim", "--output-dir", str(tmp_path), "--check", "validate_scene_reopen", "--port", "8792",
                        "--backend", "physx"], lambda **k: _Bridge()) == 0
    row = json.loads((tmp_path / "results.json").read_text())["checks"][0]
    assert ran == [] and row["status"] == "needs_setup"
    assert "--check validate_scene_reopen --saved" in row["error"] and "--port 8792" in row["error"]


def test_every_tool_a_check_calls_is_listed_by_the_toolsets_the_runner_starts_it_with():
    """The runner never relies on the default selection: each check's table entry lists what its source calls."""
    import re
    sys.path.insert(0, str(SCRIPT.parents[1] / "mcp"))
    from twinsmith import config, tool_groups, toolsets
    module = runner()
    for name in {n for names in module.TOOLSETS.values() for n in names} - {"smoke_test", "workcell_smoke", "validate_generic_scene"}:
        source = (SCRIPT.parent / "smoke" / f"{name}.py").read_text()
        enabled = config.select(module.CHECK_TOOLSETS.get(name, module.DEFAULT_TOOLSETS))
        for tool in set(re.findall(r'"((?:sim|assets)__[a-z0-9_]+)"', source)) - {"sim__prim__create_many"}:
            if tool in tool_groups.members() or tool in toolsets._OWNERS:
                assert toolsets.owner(tool) in enabled, (name, tool)


def test_a_python_without_the_mcp_sdk_is_told_the_command_that_has_it(monkeypatch, capsys):
    module = runner()
    real = module.importlib.util.find_spec
    monkeypatch.setattr(module.importlib.util, "find_spec", lambda name: None if name == "mcp" else real(name))
    assert module.main(["sim", "--port", "8766", "--check", "validate_scene_joints"]) == 2
    err = capsys.readouterr().err
    assert "has no mcp" in err and "uv run --locked --isolated" in err
    assert err.rstrip().endswith("../scripts/smoke.py sim --port 8766 --check validate_scene_joints")


def test_the_work_cell_check_needs_no_pxr_on_the_host_and_the_command_is_quoted(monkeypatch, capsys):
    module = runner()
    monkeypatch.setattr(module.importlib.util, "find_spec", lambda name: None)   # neither mcp nor pxr here
    assert module.main(["sim", "--output-dir", "/tmp/my runs;x", "--check", "workcell_smoke"]) == 2
    err = capsys.readouterr().err
    assert "has no mcp," in err and "pxr" not in err    # the check's pxr code runs inside Kit
    assert err.rstrip().endswith("smoke.py sim --output-dir '/tmp/my runs;x' --check workcell_smoke")


def test_the_one_check_that_imports_pxr_on_the_host_is_probed_for_it(monkeypatch, capsys):
    module = runner()
    real = module.importlib.util.find_spec
    monkeypatch.setattr(module.importlib.util, "find_spec", lambda name: None if name == "pxr" else real(name))
    assert module.main(["sim", "--check", "validate_scene_edges"]) == 2
    assert "has no pxr," in capsys.readouterr().err
