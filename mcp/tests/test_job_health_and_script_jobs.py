"""A running job's health and progress, and scripts that run as managed jobs without a GPU or Isaac Lab."""

import asyncio
import json
import os
import shutil
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import twinsmith.server as gw
from twinsmith.isaaclab import jobs, ledger


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [])
    jobs._active.clear()
    yield
    for proc in list(jobs._active.values()):
        try:
            os.killpg(proc.pid, 9)
        except OSError:
            pass
    jobs._active.clear()


def script(tmp_path, body, name="s.py"):
    path = tmp_path / name
    path.write_text(body)
    return path


def wait_done(job_id, limit=20):
    end = time.time() + limit
    while time.time() < end:
        if jobs.runs_status(job_id)["state"] != "running":
            time.sleep(0.2)
            return jobs.runs_status(job_id)
        time.sleep(0.1)
    raise AssertionError("job did not finish")


def wait_for(job_id, text, limit=20):
    log = Path(jobs.runs_status(job_id)["log_dir"])
    end = time.time() + limit
    while time.time() < end:
        if any(text in (log / name).read_text(errors="replace") for name in ("stdout.log", "stderr.log")):
            return
        time.sleep(0.05)
    raise AssertionError(f"{text!r} never appeared")


# ---- progress read from the training library's own output ----

@pytest.mark.parametrize("log,expected", [
    ("Learning iteration 12/1500\n", (12, 1500)),
    ("\x1b[1m Learning iteration 3/10 \x1b[0m\nnoise\n                    Learning iteration 4/10\n", (4, 10)),
    ("fps step: 1200 epoch: 37/500 frames: 9000\n", (37, 500)),
    (" 45%|####      | 450/1000 [00:10<00:12, 44.2it/s]\n", (450, 1000)),
    ("Learning iteration 12/0\n", None),
    ("Learning iteration 12/5\n", None),
    ("saved 3/4 files\n", None),
    ("", None),
])
def test_iterations_are_read_from_what_the_library_prints(tmp_path, log, expected):
    path = tmp_path / "stdout.log"
    path.write_text(log)
    got = jobs._iteration_progress(path)
    assert (None if got is None else (got["done"], got["total"])) == expected
    if got:
        assert got["fraction"] == round(expected[0] / expected[1], 4) and "training log" in got["text"]
    assert jobs._iteration_progress(tmp_path / "missing.log") is None


def test_a_job_s_own_progress_line_wins_over_the_guess(tmp_path):
    out = jobs.run_script(sys.executable, script(tmp_path, "print('Learning iteration 1/100')\nprint('TWINSMITH_PROGRESS: 3/4 almost')\n"))
    status = wait_done(out["job_id"])
    assert status["progress"]["fraction"] == 0.75 and status["progress"]["text"] == "almost"
    out = jobs.run_script(sys.executable, script(tmp_path, "print('Learning iteration 30/60')\n", "t.py"))
    assert wait_done(out["job_id"])["progress"] == {"fraction": 0.5, "percent": 50.0, "done": 30, "total": 60,
                                                    "text": "iterations, from the training log"}


# ---- health of a job that is still recorded as running ----

def logs(tmp_path, stdout="", stderr="", stdout_age=0.0, stderr_age=0.0):
    folder = tmp_path / "job"
    folder.mkdir(exist_ok=True)
    now = time.time()
    for name, text, age in (("stdout.log", stdout, stdout_age), ("stderr.log", stderr, stderr_age)):
        (folder / name).write_text(text)
        os.utime(folder / name, (now - age, now - age))
    return folder


TRACEBACK = "Traceback (most recent call last):\n  File \"train.py\", line 3, in <module>\n    main()\nRuntimeError: CUDA out of memory\n"


def test_a_live_job_that_wrote_recently_is_ok(tmp_path):
    health = jobs._liveness(logs(tmp_path, "step 1\n", stdout_age=2, stderr_age=40), True)
    assert health["health"] == "ok" and 1 <= health["last_output_age_s"] <= 10 and "health_note" not in health


def test_a_traceback_at_the_end_of_stderr_with_a_live_process_reads_crashed(tmp_path):
    health = jobs._liveness(logs(tmp_path, "step 1\n", TRACEBACK, stdout_age=30, stderr_age=20), True)
    assert health["health"] == "crashed" and "CUDA out of memory" in health["health_note"]
    assert "job_cancel" in health["health_note"] and "stderr" in health["health_note"]


def test_a_job_that_kept_working_after_a_traceback_is_not_called_crashed(tmp_path):
    health = jobs._liveness(logs(tmp_path, "step 900\n", TRACEBACK, stdout_age=1, stderr_age=120), True)
    assert health["health"] == "ok"


def test_silence_reads_stalled_and_a_dead_process_reads_exited(tmp_path):
    quiet = jobs._liveness(logs(tmp_path, "step 1\n", stdout_age=jobs.STALLED_AFTER_S + 60, stderr_age=jobs.STALLED_AFTER_S + 90), True)
    assert quiet["health"] == "stalled" and quiet["last_output_age_s"] > jobs.STALLED_AFTER_S and "No output" in quiet["health_note"]
    assert jobs._liveness(logs(tmp_path, "step 1\n"), False)["health"] == "exited"
    assert jobs._liveness(tmp_path / "no-logs", True) == {"health": "ok"}


def test_status_tells_a_crashed_run_from_a_running_one(tmp_path):
    hung = ("import sys, time\nprint('started', flush=True)\n"
            "sys.stderr.write('Traceback (most recent call last):\\n  File \"x.py\", line 1, in <module>\\nValueError: bad task\\n')\n"
            "sys.stderr.flush()\ntime.sleep(60)\n")
    out = jobs.run_script(sys.executable, script(tmp_path, hung), timeout_s=120)
    wait_for(out["job_id"], "ValueError: bad task")
    status = jobs.runs_status(out["job_id"])
    assert status["state"] == "running" and status["pid_alive"] and status["health"] == "crashed"
    assert "bad task" in status["health_note"] and status["last_output_age_s"] < 30
    jobs.runs_cancel(out["job_id"], grace_s=1)
    healthy = jobs.run_script(sys.executable, script(tmp_path, "import time\nprint('working', flush=True)\ntime.sleep(60)\n", "ok.py"),
                              timeout_s=120, allow_concurrent=True)
    wait_for(healthy["job_id"], "working")
    assert jobs.runs_status(healthy["job_id"])["health"] == "ok"
    jobs.runs_cancel(healthy["job_id"], grace_s=1)
    assert "health" not in wait_done(healthy["job_id"]), "a finished job has a state and an exit code instead"


# ---- scripts that need no GPU ----

def test_a_script_without_a_gpu_starts_beside_a_simulator_and_other_jobs(tmp_path, monkeypatch):
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [{"host": "127.0.0.1", "port": 8766, "pid": 1}])
    long_running = "import time\nprint('up', flush=True)\ntime.sleep(60)\n"
    refused = jobs.run_script(sys.executable, script(tmp_path, long_running))
    assert "error" in refused and "job_id" not in refused, "the GPU default still refuses beside a running Kit"
    first = jobs.run_script(sys.executable, script(tmp_path, long_running), uses_gpu=False, timeout_s=120)
    second = jobs.run_script(sys.executable, script(tmp_path, long_running, "b.py"), uses_gpu=False, timeout_s=120)
    assert "job_id" in first and "job_id" in second, (first, second)
    assert jobs._running_job() is None, "a GPU job is not blocked by them"
    assert json.loads((Path(first["log_dir"]) / "job.json").read_text())["uses_gpu"] is False
    for job in (first, second):
        jobs.runs_cancel(job["job_id"], grace_s=1)


def test_scripts_without_a_gpu_spend_no_gpu_hours(tmp_path):
    out = jobs.run_script(sys.executable, script(tmp_path, "print('done')\n"), uses_gpu=False)
    wait_done(out["job_id"])
    counted = jobs.run_script(sys.executable, script(tmp_path, "print('done')\n", "g.py"))
    wait_done(counted["job_id"])
    assert [item["job_id"] for item in ledger._intervals(None, time.time())] == [counted["job_id"]]


def test_a_gpu_memory_check_needs_a_gpu_script(tmp_path):
    with pytest.raises(ValueError, match="uses_gpu=false"):
        jobs.run_script(sys.executable, script(tmp_path, "pass\n"), uses_gpu=False, min_free_vram_mb=100)
    with pytest.raises(ValueError, match="uses_gpu must be"):
        jobs.run_script(sys.executable, script(tmp_path, "pass\n"), uses_gpu="no")


# ---- run_script: the same runner under a general name ----

_LAB = None


def lab_server():
    """A server with the sim toolset, which holds run_script."""
    global _LAB
    if _LAB is None:
        _LAB = gw.build_server({"TWINSMITH_TOOLSETS": "default,+sim"})
    return _LAB


def tool(name):
    return lab_server()._tool_manager._tools[name]


def test_run_script_needs_no_install(tmp_path, monkeypatch):
    for variable in ("ISAACLAB_PYTHON", "ISAACLAB_ROOT", "ISAACLAB_INSTALLS", "TWINSMITH_PROJECT"):
        monkeypatch.delenv(variable, raising=False)
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [{"host": "127.0.0.1", "port": 8766, "pid": 1}])
    general = tool("run_script")
    assert "no simulator and no Isaac Lab" in general.description and "help(tool=" in general.description   # the full text is one call away
    assert "jobs action=status" in gw._tool_help(lab_server(), "run_script")   # named as a client is shown it
    assert general.parameters["properties"]["uses_gpu"]["default"] is False
    body = "import sys\nprint('TWINSMITH_PROGRESS: 1/1 done')\nprint(sys.argv[1])\n"
    out = json.loads(asyncio.run(general.fn(script=str(script(tmp_path, body)), args=["hello"], label="convert")))
    assert "job_id" in out, out
    assert out["command"][0] == (shutil.which("python3") or sys.executable)
    status = wait_done(out["job_id"])
    assert status["state"] == "succeeded" and status["script_result"]["exit_code"] == 0 and status["label"] == "convert"
    assert status["progress"]["fraction"] == 1.0
    assert jobs.runs_list(kind="script")["runs"][0]["job_id"] == out["job_id"], "one job system"


def test_run_script_with_a_gpu_keeps_the_guards(tmp_path, monkeypatch):
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [{"host": "127.0.0.1", "port": 8766, "pid": 1}])
    out = json.loads(asyncio.run(tool("run_script").fn(script=str(script(tmp_path, "pass\n")), uses_gpu=True)))
    assert "error" in out and "job_id" not in out


def test_a_bad_script_or_interpreter_is_an_error_not_a_job(tmp_path):
    general = tool("run_script")
    missing = json.loads(asyncio.run(general.fn(script=str(tmp_path / "none.py"))))
    assert "existing .py file" in missing["error"]
    bad = json.loads(asyncio.run(general.fn(script=str(script(tmp_path, "pass\n")), python=str(tmp_path / "no-python"))))
    assert "not an executable file" in bad["error"]
