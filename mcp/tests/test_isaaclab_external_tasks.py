"""External task packages: python_path, env and external_callback on the Lab tools, the envs_list hint, runs_list filters.

No Isaac Lab and no GPU: fake interpreters and fake subprocess results."""

import asyncio
import json
import os
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import install, jobs, tools


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    for name in ("ISAACLAB_TASK_MODULES", "ISAACLAB_TASK_PATH", "ISAACLAB_EXTERNAL_CALLBACK"):
        monkeypatch.delenv(name, raising=False)
    jobs._active.clear()
    yield
    jobs._active.clear()


def make_root(tmp_path: Path) -> Path:
    root = tmp_path / "lab" / "IsaacLab"
    rl = root / "scripts" / "reinforcement_learning"
    rl.mkdir(parents=True)
    (rl / "train.py").write_text("")
    (rl / "play.py").write_text("")
    return root


def fake_python(tmp_path: Path) -> Path:
    p = tmp_path / "fake_python.py"
    p.write_text("#!/usr/bin/env python3\nimport json, os, sys\n"
                 "print(json.dumps({'argv': sys.argv[1:], 'pp': os.environ.get('PYTHONPATH'), 'foo': os.environ.get('MY_FLAG')}))\n")
    p.chmod(0o755)
    return p


def wait(job_id):
    deadline = time.time() + 30
    while time.time() < deadline:
        st = jobs.runs_status(job_id)
        if st["state"] != "running":
            return st
        time.sleep(0.05)
    raise AssertionError("job did not finish")


# --- validation ---------------------------------------------------------------------------------

def test_validate_accepts_good_options(tmp_path):
    out = install.validate_launch_options([str(tmp_path)], {"MY_FLAG": "1", "N": 3}, "g1_recovery.register.register", {})
    assert out == {"python_path": [str(tmp_path)], "env": {"MY_FLAG": "1", "N": "3"},
                   "external_callback": "g1_recovery.register.register"}


@pytest.mark.parametrize("key", ["LD_PRELOAD", "ld_audit", "PYTHONHOME", "TWINSMITH_X", "ISAACLAB_PYTHON", "BASH_ENV"])
def test_validate_refuses_protected_env(key):
    with pytest.raises(ValueError, match="not accepted"):
        install.validate_launch_options(env={key: "x"}, environ={})


@pytest.mark.parametrize("env", [{"1BAD": "x"}, {"A-B": "x"}, {"A": True}, {"A": "x\0y"}, ["A"]])
def test_validate_refuses_bad_env(env):
    with pytest.raises(ValueError):
        install.validate_launch_options(env=env, environ={})


def test_validate_allows_harmless_python_variable():
    assert install.validate_launch_options(env={"PYTHONHASHSEED": "0"}, environ={})["env"] == {"PYTHONHASHSEED": "0"}


@pytest.mark.parametrize("cb", ["register", "a.b;rm", "a..b", "a.b c", "--x.y", "a.b()", "1a.b", "a.b\n"])
def test_validate_refuses_bad_callback(cb):
    with pytest.raises(ValueError, match="external_callback"):
        install.validate_launch_options(external_callback=cb, environ={})


def test_validate_python_path_must_be_existing_absolute_dirs(tmp_path):
    for bad in ("rel/dir", str(tmp_path / "missing"), "", "a" + os.pathsep + "b"):
        with pytest.raises(ValueError):
            install.validate_launch_options(python_path=[bad], environ={})
    with pytest.raises(ValueError):
        install.validate_launch_options(python_path=str(tmp_path), environ={})


def test_callback_default_from_environment():
    out = install.validate_launch_options(environ={"ISAACLAB_EXTERNAL_CALLBACK": "p.m.f"})
    assert out["external_callback"] == "p.m.f"
    given = install.validate_launch_options(external_callback="x.y", environ={"ISAACLAB_EXTERNAL_CALLBACK": "p.m.f"})
    assert given["external_callback"] == "x.y"


# --- jobs ---------------------------------------------------------------------------------------

def test_train_passes_pythonpath_env_and_callback(tmp_path):
    root = make_root(tmp_path)
    pkg = tmp_path / "proj" / "src"
    pkg.mkdir(parents=True)
    res = jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="G1-GetUp-v0",
                     python_path=[str(pkg)], env={"MY_FLAG": "yes"}, external_callback="g1_recovery.register.register",
                     allow_concurrent=True, allow_concurrent_gpu=True)
    assert "--external_callback=g1_recovery.register.register" in res["command"]
    assert wait(res["job_id"])["state"] == "succeeded"
    out = json.loads(jobs.runs_tail(res["job_id"])["lines"][-1])
    assert out["pp"].split(os.pathsep)[0] == str(pkg)
    assert out["foo"] == "yes"
    assert "--external_callback=g1_recovery.register.register" in out["argv"]


def test_unknown_task_is_not_refused_when_external_tasks_are_given(tmp_path, monkeypatch):
    root = make_root(tmp_path)
    monkeypatch.setattr(install, "scan_task_ids", lambda r: ["Isaac-Reach-Franka-v0"])
    with pytest.raises(ValueError, match="not a task id"):
        jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="Isaac-Reach-Franka-v9",
                   allow_concurrent=True, allow_concurrent_gpu=True)
    res = jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="Isaac-Reach-Franka-v9",
                     external_callback="p.r.register", allow_concurrent=True, allow_concurrent_gpu=True)
    wait(res["job_id"])


def test_callback_in_extra_args_and_option_conflict(tmp_path):
    root = make_root(tmp_path)
    with pytest.raises(ValueError, match="not both"):
        jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="T", external_callback="a.b",
                   extra_args=["--external_callback=c.d"], allow_concurrent=True, allow_concurrent_gpu=True)


def test_bad_options_refuse_before_launch(tmp_path):
    root = make_root(tmp_path)
    with pytest.raises(ValueError, match="not accepted"):
        jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="T", env={"LD_PRELOAD": "/x.so"},
                   allow_concurrent=True, allow_concurrent_gpu=True)
    assert not (tmp_path / "jobs").exists() or not list((tmp_path / "jobs").iterdir())


def test_env_default_callback_is_used(tmp_path, monkeypatch):
    root = make_root(tmp_path)
    monkeypatch.setenv("ISAACLAB_EXTERNAL_CALLBACK", "p.m.f")
    res = jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="T",
                     allow_concurrent=True, allow_concurrent_gpu=True)
    assert "--external_callback=p.m.f" in res["command"]
    wait(res["job_id"])


def test_env_default_callback_yields_to_callback_in_extra_args(tmp_path, monkeypatch):
    root = make_root(tmp_path)
    monkeypatch.setenv("ISAACLAB_EXTERNAL_CALLBACK", "p.m.f")
    res = jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="T",
                     extra_args=["--external_callback=mine.reg"], allow_concurrent=True, allow_concurrent_gpu=True)
    assert "--external_callback=mine.reg" in res["command"]
    assert "--external_callback=p.m.f" not in res["command"]
    wait(res["job_id"])


@pytest.mark.parametrize("fn", ["env_check", "record"])
def test_check_and_record_forward_options(tmp_path, monkeypatch, fn):
    seen = {}
    monkeypatch.setattr(jobs, "_start_job", lambda *a, **kw: seen.update(kw) or {"job_id": "x"})
    getattr(jobs, fn)(python="python", root=tmp_path, task="T", python_path=["/p"], env={"A": "1"},
                      external_callback="a.b")
    assert (seen["python_path"], seen["env"], seen["external_callback"]) == (["/p"], {"A": "1"}, "a.b")


def test_record_with_checkpoint_forwards_options(tmp_path, monkeypatch):
    seen = {}
    monkeypatch.setattr(jobs, "_validate_checkpoint", lambda c, r, *a: str(c))
    monkeypatch.setattr(jobs, "_start_job", lambda *a, **kw: seen.update(kw) or {"job_id": "x"})
    jobs.record(python="python", root=tmp_path, task="T", checkpoint="/c.pt", external_callback="a.b")
    assert seen["external_callback"] == "a.b"


def test_no_options_adds_no_kwargs(tmp_path, monkeypatch):
    seen = {}
    monkeypatch.setattr(jobs, "_start_job", lambda *a, **kw: seen.update(kw) or {"job_id": "x"})
    jobs.train(python="python", root=tmp_path, task="T")
    assert not {"python_path", "env", "external_callback"} & set(seen)


def test_tools_declare_the_options():
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    listed = {t.name: t for t in asyncio.run(server.list_tools())}
    for name in ("train", "eval", "play", "env_check", "record", "envs_list"):
        props = listed[f"isaaclab__{name}"].inputSchema["properties"]
        assert {"python_path", "env", "external_callback"} <= set(props), name


# --- envs_list ----------------------------------------------------------------------------------

def make_install(tmp_path):
    venv = tmp_path / "venv"
    py = venv / "bin" / "python"
    py.parent.mkdir(parents=True)
    py.write_text("")
    root = tmp_path / "IsaacLab"
    root.mkdir()
    return py, root


def fake_run(payload):
    fake = MagicMock(returncode=0, stdout=json.dumps(payload) + "\n", stderr="")
    return patch("twinsmith.isaaclab.install.subprocess.run", return_value=fake)


def test_envs_list_hint_when_empty_or_filter_misses(tmp_path):
    py, root = make_install(tmp_path)
    env = {"ISAACLAB_PYTHON": str(py), "ISAACLAB_ROOT": str(root)}
    with fake_run({"envs": [{"id": "Isaac-A-v0", "entry_point": "x"}]}):
        miss = install.envs_list(filter="g1", env=env)
        hit = install.envs_list(filter="isaac", env=env)
    with fake_run({"envs": []}):
        empty = install.envs_list(env=env)
    assert miss["envs"] == [] and "external_callback" in miss["hint"] and "'g1'" in miss["hint"]
    assert "ISAACLAB_TASK_MODULES" in empty["hint"]
    assert "hint" not in hit


def test_envs_list_applies_options_to_the_probe(tmp_path):
    py, root = make_install(tmp_path)
    pkg = tmp_path / "proj"
    pkg.mkdir()
    env = {"ISAACLAB_PYTHON": str(py), "ISAACLAB_ROOT": str(root)}
    with fake_run({"envs": [{"id": "G1-GetUp-v0", "entry_point": "x"}]}) as run:
        out = install.envs_list(filter="g1", env=env, python_path=[str(pkg)], task_env={"MY_FLAG": "1"},
                                external_callback="g1_recovery.register.register")
    kwargs = run.call_args.kwargs
    assert kwargs["env"]["PYTHONPATH"].split(os.pathsep)[0] == str(pkg)
    assert kwargs["env"]["MY_FLAG"] == "1"
    assert "g1_recovery.register.register" in run.call_args.args[0][2]
    assert out["applied"]["external_callback"] == "g1_recovery.register.register"
    assert "hint" not in out


def test_envs_list_probe_runs_the_callback_for_real(tmp_path):
    """Run the probe with the test interpreter and a stub gymnasium: the callback registers a task."""
    stub = tmp_path / "stubs"
    stub.mkdir()
    (stub / "gymnasium.py").write_text("registry = {}\n")
    (stub / "isaaclab_tasks.py").write_text("")
    proj = tmp_path / "proj" / "g1_recovery"
    proj.mkdir(parents=True)
    (proj / "__init__.py").write_text("")
    (proj / "register.py").write_text("import gymnasium\nclass S:\n    entry_point='x:y'\n"
                                      "def register():\n    gymnasium.registry['G1-GetUp-v0'] = S()\n")
    lab = tmp_path / "IsaacLab"
    lab.mkdir()
    env = {"ISAACLAB_PYTHON": sys.executable, "ISAACLAB_ROOT": str(lab)}
    with patch.dict(os.environ, {"PYTHONPATH": str(stub)}):
        out = install.envs_list(filter="g1", env=env, python_path=[str(tmp_path / "proj")],
                                external_callback="g1_recovery.register.register")
    assert [e["id"] for e in out["envs"]] == ["G1-GetUp-v0"], out
    assert "hint" not in out


# --- runs_list ----------------------------------------------------------------------------------

def write_job(tmp_path, name, task, kind, status, started):
    d = tmp_path / "jobs" / name
    d.mkdir(parents=True)
    (d / "job.json").write_text(json.dumps({"id": name, "job_id": name, "task": task, "kind": kind, "status": status,
                                            "started_at": started, "finished_at": started + 5 if status != "running" else None,
                                            "log_dir": str(d)}))


@pytest.fixture
def many_jobs(tmp_path, monkeypatch):
    monkeypatch.setattr(jobs, "_orphaned", lambda *a: False)
    monkeypatch.setattr(jobs, "_enforce_orphan_timeout", lambda *a: None)
    now = time.time()
    for i in range(40):
        write_job(tmp_path, f"20260101-0000{i:02d}-j{i}", "G1-GetUp" if i % 2 else "Isaac-Reach",
                  "train" if i % 4 else "eval", "succeeded", now - (40 - i) * 3600)
    write_job(tmp_path, "20260101-999999-live", "G1-GetUp", "play", "running", now - 60)


def test_runs_list_default_is_bounded_newest_first(many_jobs):
    out = jobs.runs_list()
    assert len(out["runs"]) == 25 and out["total"] == 41 and out["truncated"] is True
    starts = [r["started_at"] for r in out["runs"]]
    assert starts == sorted(starts, reverse=True)
    assert out["runs"][0]["job_id"].endswith("live")
    assert "hint" in out


def test_runs_list_filters(many_jobs):
    g1 = jobs.runs_list(task="g1", limit=500)
    assert g1["total"] == 21 and not g1["truncated"] and all("G1" in r["task"] for r in g1["runs"])
    assert {r["kind"] for r in jobs.runs_list(kind="eval", limit=500)["runs"]} == {"eval"}
    assert jobs.runs_list(state="running")["total"] == 1
    assert jobs.runs_list(state="finished", limit=500)["total"] == 40
    assert jobs.runs_list(since="2h", limit=500)["total"] == 2   # the live job and the job 1h old
    assert jobs.runs_list(since="1d", limit=500)["total"] == 24
    future = time.strftime("%Y-%m-%dT%H:%M", time.localtime(time.time() + 3600))
    assert jobs.runs_list(since=future)["total"] == 0
    small = jobs.runs_list(limit=3)
    assert len(small["runs"]) == 3 and small["truncated"] and small["total"] == 41


@pytest.mark.parametrize("kwargs", [{"limit": 0}, {"limit": 501}, {"limit": True}, {"since": "yesterday"}, {"since": ""},
                                    {"state": "bogus"}])
def test_runs_list_rejects_bad_arguments(many_jobs, kwargs):
    with pytest.raises(ValueError):
        jobs.runs_list(**kwargs)


# --- where Isaac Lab 3.0.0 runs --external_callback: only the rsl_rl entry points ---------------------

def test_explicit_callback_is_run_by_the_server_for_other_libraries_and_check_jobs(tmp_path):
    root = make_root(tmp_path)
    (root / "scripts" / "environments").mkdir(parents=True)
    (root / "scripts" / "environments" / "zero_agent.py").write_text("")
    for kw in ({"kind": "train", "rl_library_check": "skrl"},
               {"kind": "env_check", "check_options": {"agent": "zero", "mode": "check", "steps": 1}}):
        res = jobs._start_job_locked(str(fake_python(tmp_path)), root, kw.pop("kind"), "T", [],
                                     external_callback="p.m.f", allow_concurrent=True, allow_concurrent_gpu=True, **kw)
        assert not any(str(c).startswith("--external_callback") for c in res["command"])
        wait(res["job_id"])


def test_env_default_callback_is_run_by_the_server_for_other_libraries(tmp_path, monkeypatch):
    root = make_root(tmp_path)
    monkeypatch.setenv("ISAACLAB_EXTERNAL_CALLBACK", "p.m.f")
    monkeypatch.setattr(jobs, "_require_library_installed", lambda *a: None)
    res = jobs.train(python=str(fake_python(tmp_path)), root=str(root), task="T", rl_library="skrl",
                     allow_concurrent=True, allow_concurrent_gpu=True)
    assert "--external_callback=p.m.f" not in res["command"]
    wait(res["job_id"])
