"""Launch options of the Isaac Lab tools: external_callback for every library, cwd, one env policy, python_paths,
checkpoint roots, task_modules, the progress marker and per-call install. No Isaac Lab and no GPU: fake interpreters."""

import asyncio
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import install, jobs, tools


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    for name in ("ISAACLAB_TASK_MODULES", "ISAACLAB_TASK_PATH", "ISAACLAB_EXTERNAL_CALLBACK", "ISAACLAB_CHECKPOINT_ROOTS"):
        monkeypatch.delenv(name, raising=False)
    jobs._active.clear()
    yield
    jobs._active.clear()


def make_root(tmp_path: Path) -> Path:
    root = tmp_path / "lab" / "IsaacLab"
    rl = root / "scripts" / "reinforcement_learning"
    rl.mkdir(parents=True, exist_ok=True)
    (rl / "train.py").write_text("")
    (rl / "play.py").write_text("")
    return root


def fake_python(tmp_path: Path) -> Path:
    p = tmp_path / "fake_python.py"
    p.write_text("#!/usr/bin/env python3\nimport json, os, sys\n"
                 "print(json.dumps({'argv': sys.argv[1:], 'cwd': os.getcwd(), 'cb': os.environ.get('TWINSMITH_EXTERNAL_CALLBACK'),"
                 " 'mods': os.environ.get('ISAACLAB_TASK_MODULES')}))\n"
                 "print('TWINSMITH_PROGRESS: 3/4 tasks 1-3 done')\n")
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


def launch(fn, tmp_path, **kw):
    return fn(python=str(fake_python(tmp_path)), root=str(make_root(tmp_path)), allow_concurrent=True,
              allow_concurrent_gpu=True, **kw)


def last_json(job_id):
    return json.loads(jobs.runs_tail(job_id)["lines"][-2])


# --- 1. external_callback for every library, and task_modules -----------------------------------

def test_rsl_rl_keeps_the_native_flag(tmp_path):
    res = launch(jobs.train, tmp_path, task="T", rl_library="rsl_rl", external_callback="p.reg.register")
    assert "--external_callback=p.reg.register" in res["command"]
    wait(res["job_id"])


@pytest.mark.parametrize("lib", ["rl_games", "skrl", "sb3"])
def test_other_libraries_use_the_launcher(tmp_path, lib):
    res = launch(jobs.train, tmp_path, task="T", rl_library=lib, external_callback="p.reg.register")
    assert not any(str(c).startswith("--external_callback") for c in res["command"])
    assert res["command"][1].endswith("launcher.py")
    st = wait(res["job_id"])
    assert st["state"] == "succeeded"
    assert last_json(res["job_id"])["cb"] == "p.reg.register"


def test_play_and_env_check_use_the_launcher_or_runner(tmp_path):
    ckpt = make_root(tmp_path) / "m.pth"
    ckpt.write_text("x")
    res = launch(jobs.play, tmp_path, task="T", checkpoint=str(ckpt), rl_library="rl_games", external_callback="p.reg.register")
    assert wait(res["job_id"])["state"] == "succeeded" and last_json(res["job_id"])["cb"] == "p.reg.register"
    (ckpt.parent / "scripts" / "environments").mkdir(parents=True)
    (ckpt.parent / "scripts" / "environments" / "zero_agent.py").write_text("")
    res = launch(jobs.env_check, tmp_path, task="T", external_callback="p.reg.register")  # used to be refused
    assert res["command"][1].endswith("check_runner.py")
    wait(res["job_id"])
    assert last_json(res["job_id"])["cb"] == "p.reg.register"


def test_task_modules_become_the_import_list(tmp_path):
    res = launch(jobs.train, tmp_path, task="T", rl_library="rl_games", task_modules=["pkg.tasks", "other"])
    wait(res["job_id"])
    assert last_json(res["job_id"])["mods"] == "pkg.tasks,other"
    with pytest.raises(ValueError, match="task_modules"):
        install.validate_launch_options(task_modules=["a b"], environ={})
    with pytest.raises(ValueError, match="task_modules"):
        install.validate_launch_options(task_modules="pkg.tasks", environ={})


def test_launcher_calls_the_callback_before_the_script(tmp_path):
    pkg = tmp_path / "proj"
    pkg.mkdir()
    (pkg / "myreg.py").write_text("import os\ndef register():\n    print('REGISTERED')\n")
    script = tmp_path / "s.py"
    script.write_text("print('SCRIPT')\n")
    launcher = Path(jobs.__file__).parent / "launcher.py"
    out = subprocess.run([sys.executable, str(launcher), str(script)], capture_output=True, text=True,
                         env={**os.environ, "PYTHONPATH": str(pkg), "TWINSMITH_EXTERNAL_CALLBACK": "myreg.register"})
    assert out.stdout.split() == ["REGISTERED", "SCRIPT"], out.stderr


# --- 2. cwd -------------------------------------------------------------------------------------

def test_cwd_is_the_job_working_directory(tmp_path):
    proj = tmp_path / "proj" / "simulation"
    proj.mkdir(parents=True)
    res = launch(jobs.train, tmp_path, task="T", cwd=str(proj))
    wait(res["job_id"])
    assert last_json(res["job_id"])["cwd"] == str(proj.resolve())
    assert jobs.runs_list()["runs"]  # the job record is readable


def test_cwd_must_be_an_existing_directory(tmp_path):
    with pytest.raises(ValueError, match="cwd is not a directory"):
        launch(jobs.train, tmp_path, task="T", cwd=str(tmp_path / "nope"))


def test_tools_declare_cwd_and_task_modules():
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    listed = {t.name: t for t in asyncio.run(server.list_tools())}
    for name in ("train", "eval", "play", "env_check", "record"):
        assert {"cwd", "task_modules"} <= set(listed[name if name == "run_script" else f"isaaclab__{name}"].inputSchema["properties"]), name
    assert "task_modules" in listed["isaaclab__envs_list"].inputSchema["properties"]


# --- 3. one env policy --------------------------------------------------------------------------

@pytest.mark.parametrize("key", ["OMNI_KIT_ACCEPT_EULA", "CUDA_VISIBLE_DEVICES", "HEADLESS", "MY_FLAG", "PYTHONPATH", "PATH",
                                 "LD_LIBRARY_PATH", "HOME", "TMPDIR", "PYTHONDONTWRITEBYTECODE"])
def test_shared_env_policy_allows(key):
    assert install.validate_env({key: "1"}) == {key: "1"}


@pytest.mark.parametrize("key,advice", [("PYTHONHOME", "install="), ("ISAACLAB_ROOT", "install="), ("ISAACLAB_PYTHON", "install="),
                                        ("LD_PRELOAD", "run_script"), ("LD_AUDIT", "run_script"), ("BASH_ENV", "shell"),
                                        ("TWINSMITH_X", "MCP client config"), ("TWINSMITH_JOBS_DIR", "server setting")])
def test_shared_env_policy_refuses_only_hijacks_and_says_what_to_use(key, advice):
    with pytest.raises(ValueError, match="not accepted") as err:
        install.validate_env({key: "x"})
    assert advice in str(err.value)


def test_path_like_variables_are_prepended_not_replaced():
    base = {"PATH": "/usr/bin", "PYTHONPATH": "/root", "LD_LIBRARY_PATH": "", "KEEP": "a"}
    install.apply_env(base, {"PATH": "/mine", "PYTHONPATH": "/proj", "LD_LIBRARY_PATH": "/lib", "KEEP": "b", "NEW": "c"})
    assert base == {"PATH": "/mine" + os.pathsep + "/usr/bin", "PYTHONPATH": "/proj" + os.pathsep + "/root",
                    "LD_LIBRARY_PATH": "/lib", "KEEP": "b", "NEW": "c"}


def test_pythonpath_in_env_reaches_the_job_in_front_of_the_install_path(tmp_path, monkeypatch):
    proj = tmp_path / "proj"
    proj.mkdir()
    script = tmp_path / "s.py"
    script.write_text("import os\nprint('PP=' + os.environ['PYTHONPATH'])\n")
    monkeypatch.setenv("PYTHONPATH", "/inherited")
    res = jobs.run_script(sys.executable, str(script), env={"PYTHONPATH": str(proj)}, python_path=[str(tmp_path)],
                          allow_concurrent=True, allow_concurrent_gpu=True)
    deadline = time.time() + 20
    while jobs.runs_status(res["job_id"])["state"] == "running" and time.time() < deadline:
        time.sleep(0.05)
    line = [x for x in jobs.runs_tail(res["job_id"])["lines"] if x.startswith("PP=")][0]
    assert line == "PP=" + os.pathsep.join([str(tmp_path), str(proj), "/inherited"])


def test_python_paths_is_the_documented_argument_and_python_path_its_alias(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    a.mkdir(), b.mkdir()
    assert install.merge_python_paths(None, None) is None
    assert install.merge_python_paths([str(b)], [str(a)]) == [str(a), str(b)]
    assert install.merge_python_paths([str(a)], [str(a)]) == [str(a)]
    with pytest.raises(ValueError, match="python_paths"):
        install.merge_python_paths(None, str(a))
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    listed = {t.name: t for t in asyncio.run(server.list_tools())}
    for name in ("train", "eval", "play", "env_check", "record", "envs_list"):
        assert {"python_paths", "python_path"} <= set(listed[f"isaaclab__{name}"].inputSchema["properties"]), name
    assert "python_paths" in listed["run_script"].inputSchema["properties"]


def test_play_and_run_script_agree(tmp_path):
    env = {"OMNI_KIT_ACCEPT_EULA": "YES"}
    ckpt = tmp_path / "lab" / "IsaacLab" / "m.pth"
    ckpt.parent.mkdir(parents=True, exist_ok=True)
    make_root(tmp_path)
    ckpt.write_text("x")
    assert launch(jobs.play, tmp_path, task="T", checkpoint=str(ckpt), env=env)["job_id"]
    script = tmp_path / "s.py"
    script.write_text("print(1)\n")
    assert jobs.run_script(sys.executable, str(script), env=env, allow_concurrent=True, allow_concurrent_gpu=True)["job_id"]
    for fn in (lambda: jobs.run_script(sys.executable, str(script), env={"PYTHONHOME": "/x"}),
               lambda: launch(jobs.train, tmp_path, task="T", env={"PYTHONHOME": "/x"})):
        with pytest.raises(ValueError, match="not accepted"):
            fn()


# --- 4. checkpoint roots ------------------------------------------------------------------------

def test_checkpoint_outside_roots_names_the_allowed_ones(tmp_path):
    root = make_root(tmp_path)
    ckpt = tmp_path / "scratch" / "p.pth"
    ckpt.parent.mkdir()
    ckpt.write_text("x")
    with pytest.raises(ValueError) as err:
        jobs._validate_checkpoint(str(ckpt), root)
    msg = str(err.value)
    assert "outside allowed roots" in msg and str(root.resolve()) in msg and "ISAACLAB_CHECKPOINT_ROOTS" in msg


def test_checkpoint_allowed_under_python_path_cwd_or_env_roots(tmp_path, monkeypatch):
    root = make_root(tmp_path)
    proj = tmp_path / "proj" / "simulation"
    (proj / "checkpoints").mkdir(parents=True)
    ckpt = proj / "checkpoints" / "p.pth"
    ckpt.write_text("x")
    assert jobs._validate_checkpoint(str(ckpt), root, [str(proj)]) == str(ckpt.resolve())
    assert jobs._validate_checkpoint(str(ckpt), root, None, str(proj)) == str(ckpt.resolve())
    monkeypatch.setenv("ISAACLAB_CHECKPOINT_ROOTS", str(proj))
    assert jobs._validate_checkpoint(str(ckpt), root) == str(ckpt.resolve())
    other = tmp_path / "elsewhere.pth"
    other.write_text("x")
    with pytest.raises(ValueError, match="outside allowed roots"):
        jobs._validate_checkpoint(str(other), root, [str(proj)], str(proj))


def test_checkpoint_symlink_escape_is_still_refused(tmp_path):
    root = make_root(tmp_path)
    proj = tmp_path / "proj"
    proj.mkdir()
    (tmp_path / "secret.pth").write_text("x")
    (proj / "link.pth").symlink_to(tmp_path / "secret.pth")
    with pytest.raises(ValueError, match="outside allowed roots"):
        jobs._validate_checkpoint(str(proj / "link.pth"), root, [str(proj)])


# --- 6. progress marker -------------------------------------------------------------------------

def test_progress_marker_is_reported(tmp_path):
    res = launch(jobs.train, tmp_path, task="T")
    st = wait(res["job_id"])
    assert st["progress"] == {"fraction": 0.75, "percent": 75.0, "done": 3.0, "total": 4.0, "text": "tasks 1-3 done"}


@pytest.mark.parametrize("line,expected", [
    ("TWINSMITH_PROGRESS: 0.5", 0.5), ("TWINSMITH_PROGRESS: 40% half", 0.4), ("TWINSMITH_PROGRESS: 2/8", 0.25),
    ("TWINSMITH_PROGRESS: nope", None), ("TWINSMITH_PROGRESS: 1.5", None), ("TWINSMITH_PROGRESS: 1/0", None), ("plain", None)])
def test_progress_parsing(tmp_path, line, expected):
    log = tmp_path / "l.log"
    log.write_text("x\n" + line + "\n")
    got = jobs._progress(log)
    assert (got["fraction"] if got else None) == expected


def test_progress_uses_the_newest_line(tmp_path):
    log = tmp_path / "l.log"
    log.write_text("TWINSMITH_PROGRESS: 0.1\nnoise\nTWINSMITH_PROGRESS: 0.9\n")
    assert jobs._progress(log)["fraction"] == 0.9


# --- 8. choosing an install per call ------------------------------------------------------------

def make_env(base: Path, name: str, lab_version: str, with_src: bool = True) -> Path:
    venv = base / name
    (venv / "bin").mkdir(parents=True)
    py = venv / "bin" / "python"
    py.write_text("#!/bin/sh\nexit 0\n")
    py.chmod(0o755)
    (venv / "lib" / "python3.12" / "site-packages" / f"isaaclab-{lab_version}.dist-info").mkdir(parents=True)
    if with_src:
        rl = base / f"{name}-src" / "scripts" / "reinforcement_learning"
        rl.mkdir(parents=True)
    return venv


@pytest.fixture
def two_installs(tmp_path, monkeypatch):
    envs = tmp_path / "envs"
    new, old = make_env(envs, "lab-3.0.0", "3.0.0"), make_env(envs, "lab-2.3.2", "0.54.2")
    monkeypatch.setenv("ISAACLAB_PYTHON", str(new / "bin" / "python"))
    monkeypatch.setenv("ISAACLAB_ROOT", str(envs / "lab-3.0.0-src"))
    monkeypatch.delenv("TWINSMITH_PROJECT", raising=False)
    return new, old, envs


def test_default_stays_the_configured_install(two_installs):
    new, _, _ = two_installs
    loc, err = install.resolve_install(None)
    assert err is None and loc["python"] == str(new / "bin" / "python")


@pytest.mark.parametrize("sel", ["lab-2.3.2", "2.3", "2.3.2", "v2.3.2"])
def test_known_installs_are_selected_by_name_or_version(two_installs, sel):
    _, old, envs = two_installs
    loc, err = install.resolve_install(sel)
    assert err is None and loc["python"] == str(old / "bin" / "python")
    assert loc["isaaclab_root"] == str(envs / "lab-2.3.2-src") and loc["source"] == f"install:{sel}"


def test_install_by_absolute_path_and_unknown_name(two_installs):
    _, old, _ = two_installs
    assert install.resolve_install(str(old))[0]["name"] == "lab-2.3.2"
    assert install.resolve_install(str(old / "bin" / "python"))[0]["name"] == "lab-2.3.2"
    loc, err = install.resolve_install("9.9")
    assert loc is None and "lab-2.3.2" in err and "lab-3.0.0" in err and "ISAACLAB_INSTALLS" in err
    loc, err = install.resolve_install("/nonexistent/venv")
    assert loc is None and "virtual environment with Isaac Lab" in err


def test_isaaclab_installs_env_adds_installs(two_installs, tmp_path, monkeypatch):
    extra = make_env(tmp_path / "elsewhere", "mylab-2.4.0", "0.60.0")
    monkeypatch.setenv("ISAACLAB_INSTALLS", str(extra))
    assert install.resolve_install("mylab-2.4.0")[0]["python"] == str(extra / "bin" / "python")


def test_project_file_install_wins_over_the_harness_default(two_installs, tmp_path, monkeypatch):
    _, old, _ = two_installs
    proj = tmp_path / "proj"
    proj.mkdir()
    (proj / "twinsmith.json").write_text(json.dumps({"isaaclab_install": "lab-2.3.2"}))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(proj))
    assert install.resolve_install(None)[0]["python"] == str(old / "bin" / "python")
    assert install.resolve_install("default")[0]["python"] == str(two_installs[0] / "bin" / "python")  # an explicit "default" opts out


def test_launch_uses_the_selected_install(two_installs, monkeypatch):
    _, old, envs = two_installs
    seen = {}
    monkeypatch.setattr(jobs, "train", lambda **kw: seen.update(kw) or {"job_id": "x"})
    out = json.loads(tools._launch(jobs.train, install="lab-2.3.2", task="T"))
    assert out["job_id"] == "x" and seen["python"] == str(old / "bin" / "python") and seen["root"] == str(envs / "lab-2.3.2-src")
    bad = json.loads(tools._launch(jobs.train, install="9.9", task="T"))
    assert "matches no known install" in bad["error"]


def test_install_info_lists_known_installs_and_tools_declare_install(two_installs):
    from mcp.server.fastmcp import FastMCP

    names = {r["name"] for r in install.known_installs()}
    assert names == {"lab-3.0.0", "lab-2.3.2"}
    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    listed = {t.name: t for t in asyncio.run(server.list_tools())}
    for name in ("train", "eval", "play", "env_check", "record", "envs_list", "run_script", "install_info"):
        assert "install" in listed[name if name == "run_script" else f"isaaclab__{name}"].inputSchema["properties"], name


# --- 9. audit: consistency fixes ----------------------------------------------------------------

def test_run_script_python_path_follows_the_shared_rule(tmp_path):
    script = tmp_path / "s.py"
    script.write_text("print(1)\n")
    for bad in (["relative/dir"], [str(tmp_path / "missing")], str(tmp_path)):
        with pytest.raises(ValueError, match="python_path"):
            jobs.run_script(sys.executable, str(script), python_path=bad)


def test_metrics_roots_include_checkpoint_roots_and_known_install_logs(two_installs, tmp_path, monkeypatch):
    from twinsmith.isaaclab import metrics

    scratch = tmp_path / "scratch"
    scratch.mkdir()
    monkeypatch.setenv("ISAACLAB_CHECKPOINT_ROOTS", str(scratch))
    roots = metrics._allowed_roots()
    assert os.path.realpath(scratch) in roots
    assert os.path.realpath(tmp_path / "envs" / "lab-2.3.2-src" / "logs") in roots
    assert metrics._resolve_run_dir(str(scratch / "run1"))[1] is None


def test_server_owned_env_error_says_where_to_set_it():
    with pytest.raises(ValueError, match="MCP client config"):
        install.validate_env({"TWINSMITH_PORT": "1"})
