"""run_script: managed user-script jobs (fake scripts, no GPU, nvidia-smi mocked)."""

import asyncio
import json
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import jobs, preflight, tools


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [])
    jobs._active.clear()
    yield
    for jid, proc in list(jobs._active.items()):
        try:
            os.killpg(proc.pid, 9)
        except OSError:
            pass
    jobs._active.clear()


def script(tmp_path, body, name="s.py"):
    p = tmp_path / name
    p.write_text(body)
    return p


def wait_done(job_id, limit=20):
    end = time.time() + limit
    while time.time() < end:
        st = jobs.runs_status(job_id)
        if st["state"] != "running":
            time.sleep(0.2)
            return jobs.runs_status(job_id)
        time.sleep(0.1)
    raise AssertionError("job did not finish")


def start(tmp_path, body, **kw):
    out = jobs.run_script(sys.executable, script(tmp_path, body), **kw)
    assert "job_id" in out, out
    return out


def test_success_collects_artifacts_exit_and_log(tmp_path):
    out = start(tmp_path, "import os,sys\nprint('hello', sys.argv[1:], os.environ['FOO'])\n"
                          "print('TWINSMITH_ARTIFACT: /tmp/a.mp4')\nprint('TWINSMITH_ARTIFACT: /tmp/b.json')\n",
                args=["x", "y"], env={"FOO": "bar"}, label="my sweep")
    st = wait_done(out["job_id"])
    assert st["state"] == "succeeded" and st["exit_code"] == 0
    r = st["script_result"]
    assert r["artifacts"] == ["/tmp/a.mp4", "/tmp/b.json"] and r["exit_code"] == 0
    assert r["wall_time_s"] >= 0 and Path(r["log_path"]).is_file()
    assert "hello ['x', 'y'] bar" in Path(r["log_path"]).read_text()
    runs = jobs.runs_list()["runs"]
    assert runs[0]["kind"] == "script" and runs[0]["task"] == "my sweep"
    assert jobs.runs_tail(out["job_id"])["lines"]


def test_failure_records_exit_code(tmp_path):
    out = start(tmp_path, "import sys\nsys.exit(3)\n")
    st = wait_done(out["job_id"])
    assert st["state"] == "failed" and st["exit_code"] == 3


def test_python_path_and_cwd(tmp_path):
    lib = tmp_path / "lib"
    lib.mkdir()
    (lib / "mymod.py").write_text("X = 7\n")
    work = tmp_path / "work"
    work.mkdir()
    out = start(tmp_path, "import os, mymod\nprint('X', mymod.X, os.getcwd())\n", python_path=[str(lib)], cwd=str(work))
    st = wait_done(out["job_id"])
    assert st["state"] == "succeeded"
    assert f"X 7 {work.resolve()}" in "\n".join(jobs.runs_tail(out["job_id"])["lines"])


def test_timeout_kills_whole_group(tmp_path):
    pidfile = tmp_path / "child.pid"
    body = ("import subprocess, sys, time\n"
            f"c = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(300)'])\n"
            f"open({str(pidfile)!r}, 'w').write(str(c.pid))\n"
            "time.sleep(300)\n")
    out = start(tmp_path, body, timeout_s=2)
    st = wait_done(out["job_id"], limit=30)
    assert st["state"] == "timeout"
    child = int(pidfile.read_text())
    end = time.time() + 15
    while time.time() < end and jobs._pid_alive(child):
        try:
            os.waitpid(child, os.WNOHANG)
        except ChildProcessError:
            pass
        time.sleep(0.2)
    assert not jobs._pid_alive(child) or _is_zombie(child)


def _is_zombie(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] == "Z"
    except OSError:
        return True


def test_cancel_kills_group_and_job_is_cancelled(tmp_path):
    pidfile = tmp_path / "child.pid"
    body = ("import subprocess, sys, time\n"
            "c = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(300)'])\n"
            f"open({str(pidfile)!r}, 'w').write(str(c.pid))\n"
            "time.sleep(300)\n")
    out = start(tmp_path, body)
    end = time.time() + 10
    while time.time() < end and not pidfile.exists():
        time.sleep(0.1)
    child = int(pidfile.read_text())
    res = jobs.runs_cancel(out["job_id"], grace_s=5)
    assert res["cancelled"] is True
    end = time.time() + 15
    while time.time() < end and not _is_zombie(child):
        time.sleep(0.2)
    assert _is_zombie(child)
    assert jobs.runs_status(out["job_id"])["state"] == "cancelled"


def test_single_instance_key_refuses_second(tmp_path):
    first = start(tmp_path, "import time\ntime.sleep(30)\n", single_instance_key="sweep-1")
    again = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n", "t.py"), single_instance_key="sweep-1",
                            allow_concurrent=True)
    assert "single_instance_key" in again["error"] and again["running_job_id"] == first["job_id"]
    other = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n", "u.py"), single_instance_key="other",
                            allow_concurrent=True)
    assert "job_id" in other
    jobs.runs_cancel(first["job_id"], grace_s=2)
    jobs.runs_cancel(other["job_id"], grace_s=2)


def test_concurrency_and_kit_bridge_guards(tmp_path, monkeypatch):
    first = start(tmp_path, "import time\ntime.sleep(30)\n")
    blocked = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n", "t.py"))
    assert "already running" in blocked["error"]
    jobs.runs_cancel(first["job_id"], grace_s=2)
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [{"host": "127.0.0.1", "port": 8766}])
    kit = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n", "t.py"))
    assert "Kit bridge" in kit["error"] and "allow_concurrent_gpu" in kit["hint"]
    ok = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n", "t.py"), allow_concurrent_gpu=True)
    assert "job_id" in ok
    wait_done(ok["job_id"])


@pytest.mark.parametrize("kwargs,msg", [
    ({"env": {"1BAD": "x"}}, "invalid env variable name"),
    ({"env": {"A-B": "x"}}, "invalid env variable name"),
    ({"env": {"OK": True}}, "must be a string"),
    ({"args": [1]}, "list of strings"),
    ({"timeout_s": 0}, "timeout_s"),
    ({"timeout_s": True}, "timeout_s"),
    ({"cwd": "/nonexistent-dir-xyz"}, "cwd"),
    ({"python": "/nonexistent/python"}, "python"),
    ({"min_free_vram_mb": 0}, "min_free_vram_mb"),
    ({"wait_for_vram_s": -1}, "wait_for_vram_s"),
])
def test_validation(tmp_path, kwargs, msg):
    kwargs = dict(kwargs)
    py = kwargs.pop("python", sys.executable)
    with pytest.raises(ValueError, match=msg):
        jobs.run_script(py, script(tmp_path, "print(1)\n"), **kwargs)
    assert not list((tmp_path / "jobs").glob("*")) if (tmp_path / "jobs").exists() else True


def test_script_must_be_existing_py_file(tmp_path):
    (tmp_path / "x.txt").write_text("print(1)")
    for bad in (tmp_path / "x.txt", tmp_path / "missing.py", tmp_path):
        with pytest.raises(ValueError, match="existing .py"):
            jobs.run_script(sys.executable, bad)


def test_never_runs_on_import_or_shell(tmp_path, monkeypatch):
    seen = {}
    real = jobs.subprocess.Popen

    def spy(cmd, **kw):
        seen["shell"] = kw.get("shell")
        seen["session"] = kw.get("start_new_session")
        return real(cmd, **kw)

    monkeypatch.setattr(jobs.subprocess, "Popen", spy)
    marker = tmp_path / "ran"
    out = start(tmp_path, f"open({str(marker)!r}, 'w').write('x')\n")
    wait_done(out["job_id"])
    assert seen == {"shell": False, "session": True}


def fake_gpu(free_seq):
    it = iter(free_seq)
    last = [free_seq[-1]]

    def gpu():
        try:
            last[0] = next(it)
        except StopIteration:
            pass
        return {"name": "G", "total_mib": 16000, "used_mib": 0, "free_mib": last[0], "processes": []}

    return gpu


def test_vram_refuses_when_not_enough(tmp_path, monkeypatch):
    monkeypatch.setattr(preflight, "gpu_state", fake_gpu([1000]))
    out = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), min_free_vram_mb=4000)
    assert "only 1000 MiB" in out["error"] and "wait_for_vram_s" in out["hint"]


def test_vram_waits_until_free(tmp_path, monkeypatch):
    monkeypatch.setattr(preflight, "gpu_state", fake_gpu([1000, 1000, 8000]))
    monkeypatch.setattr(jobs, "_VRAM_POLL_S", 0.05)
    out = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), min_free_vram_mb=4000, wait_for_vram_s=10)
    assert "job_id" in out
    wait_done(out["job_id"])


def test_vram_unknown_gpu_refuses(tmp_path, monkeypatch):
    monkeypatch.setattr(preflight, "gpu_state", lambda: None)
    out = jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), min_free_vram_mb=100)
    assert "nvidia-smi" in out["error"]


def test_preflight_lists_gpu_processes_with_names(monkeypatch):
    def smi(args):
        if args[0].startswith("--query-gpu"):
            return "RTX 5080, 16303, 9000, 7303\n"
        assert args[0] == "--query-compute-apps=pid,process_name,used_memory"
        return "4242, /opt/py/bin/python3, 8000\n77, kit, 1000\n"

    monkeypatch.setattr(preflight, "_smi", smi)
    monkeypatch.setattr(jobs, "_running_job", lambda: None)
    out = preflight.preflight()
    assert out["gpu"]["processes"][0] == {"pid": 4242, "name": "/opt/py/bin/python3", "mib": 8000}
    assert [p["pid"] for p in out["other_gpu_processes"]] == [4242, 77]


def test_tool_layer_json_and_errors(tmp_path, monkeypatch):
    monkeypatch.setattr(tools._install, "locate_install", lambda: {"python": sys.executable, "isaaclab_root": None})
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    afn = server._tool_manager._tools["run_script"].fn

    def fn(**kw):  # the tool is async so a VRAM wait cannot block the event loop
        return asyncio.run(afn(uses_gpu=True, **kw))

    bad = json.loads(fn(script=str(tmp_path / "nope.py")))
    assert "existing .py" in bad["error"]
    ok = json.loads(fn(script=str(script(tmp_path, "print('hi')\n"))))
    assert "job_id" in ok
    wait_done(ok["job_id"])
    ann = server._tool_manager._tools["run_script"].annotations
    assert ann.readOnlyHint is False and ann.idempotentHint is False
    monkeypatch.setattr(tools._install, "locate_install", lambda: None)
    assert "job_id" in json.loads(fn(script=str(tmp_path / "s.py")))


def test_wait_for_vram_needs_a_minimum(tmp_path):
    with pytest.raises(ValueError, match="needs min_free_vram_mb"):
        jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), wait_for_vram_s=5)


def test_vram_check_refuses_when_script_is_pinned_off_gpu0(tmp_path, monkeypatch):
    monkeypatch.setattr(preflight, "gpu_state", fake_gpu([9000]))
    with pytest.raises(ValueError, match="GPU 0 only"):
        jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), min_free_vram_mb=100, env={"CUDA_VISIBLE_DEVICES": "1"})


def test_cancelled_wait_never_launches(tmp_path, monkeypatch):
    import threading

    monkeypatch.setattr(preflight, "gpu_state", fake_gpu([1000]))
    monkeypatch.setattr(jobs, "_VRAM_POLL_S", 0.05)
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(jobs.LaunchCancelled):
        jobs.run_script(sys.executable, script(tmp_path, "print(1)\n"), min_free_vram_mb=4000, wait_for_vram_s=30, cancel=cancel)
    assert not list(jobs.jobs_dir().iterdir())
