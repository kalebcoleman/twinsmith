import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import jobs, rundir, tools


def status_tool(**kw):
    """Call isaaclab__runs_status the way a client does and parse its JSON."""
    import asyncio
    import inspect

    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    fn = server._tool_manager._tools["isaaclab__runs_status"].fn
    out = fn(**kw)
    return json.loads(asyncio.run(out) if inspect.isawaitable(out) else out)


@pytest.fixture(autouse=True)
def _dirs(tmp_path, monkeypatch):
    logs = tmp_path / "logs"
    logs.mkdir()
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    monkeypatch.setenv("ISAACLAB_LOGS_DIR", str(logs))
    monkeypatch.setenv("ISAACLAB_ALLOW_TMP", "1")
    return logs


def make_run(logs, experiment, stamp, name=None):
    d = logs / "rsl_rl" / experiment / (f"{stamp}_{name}" if name else stamp)
    (d / "nn").mkdir(parents=True)
    return d


def test_run_name_resolves_newest_exact_match(tmp_path, _dirs):
    old = make_run(_dirs, "g1", "2026-11-01_10-00-00", "recC")
    new = make_run(_dirs, "g1", "2026-11-02_10-00-00", "recC")
    make_run(_dirs, "g1", "2026-11-03_10-00-00", "recC_v2")  # a longer name is not a match
    (new / "model_5.pt").write_bytes(b"x")
    with pytest.raises(ValueError, match=r"2 run folders match.*latest=true"):
        rundir.from_log_root(str(_dirs), "recC")
    out = rundir.from_log_root(str(_dirs), "recC", latest=True)
    assert out["run_dir"] == str(new) and out["source"] == "log_root"
    assert out["candidates"] == [str(new), str(old)]
    assert out["latest_checkpoint"].endswith("model_5.pt")


def test_experiment_narrows_matches(tmp_path, _dirs):
    make_run(_dirs, "g1", "2026-11-01_10-00-00", "x")
    other = make_run(_dirs, "h1", "2026-11-02_10-00-00", "x")
    out = rundir.from_log_root(str(_dirs), "x", experiment="h1")
    assert out["run_dir"] == str(other) and "candidates" not in out


@pytest.mark.parametrize("kwargs, text", [
    ({}, "together"),
    ({"log_root": "/x"}, "together"),
    ({"run_name": "x"}, "together"),
    ({"job_id": "20261104-092041-T", "run_name": "x"}, "not both"),
    ({"log_root": "x", "run_name": ""}, "empty"),
    ({"log_root": "x", "run_name": "  "}, "empty"),
    ({"log_root": "", "run_name": "x"}, "empty"),
    ({"log_root": "x", "run_name": "../evil"}, "plain name"),
    ({"log_root": "x", "run_name": "a/b"}, "plain name"),
])
def test_bad_arguments_are_explicit_errors(kwargs, text):
    out = status_tool(**kwargs)
    assert text in out["error"] and "run_dir" not in out


def test_no_match_lists_what_exists(_dirs):
    make_run(_dirs, "g1", "2026-11-01_10-00-00", "other")
    with pytest.raises(ValueError, match=r"no run folder '<timestamp>_missing'.*2026-11-01_10-00-00_other"):
        rundir.from_log_root(str(_dirs), "missing")


def test_missing_and_foreign_log_root_are_errors(tmp_path, _dirs):
    assert "not a directory" in status_tool(log_root=str(_dirs / "nope"), run_name="x")["error"]
    err = status_tool(log_root="/etc", run_name="x")["error"]
    assert "outside allowed roots" in err and "ISAACLAB_LOGS_DIR" in err


def write_job(job_id, run_dir_text, tmp_path, status="succeeded"):
    jd = jobs.jobs_dir() / job_id
    jd.mkdir(parents=True)
    (jd / "stdout.log").write_text(run_dir_text)
    jobs._write_job(jd, {"job_id": job_id, "kind": "train", "task": "T", "status": status, "exit_code": 3,
                         "log_dir": str(jd), "run_dir": str(jd), "pid": None, "started_at": 1.0, "finished_at": 2.0})
    return jd


def test_job_resolves_from_log(tmp_path, _dirs):
    run = make_run(_dirs, "g1", "2026-11-02_10-00-00", "recC")
    jid = "20261102-100000-T"
    write_job(jid, f"[INFO]: Logging experiment in directory: {run.parent}\n"
                   f"Exact experiment name requested from command line: {run.name}\n", tmp_path)
    out = status_tool(job_id=jid, require_run_dir=True)
    assert out["run_dir"] == str(run) and out["run_dir_source"] == "lab_log" and out["exit_code"] == 3
    assert "run_dir_warning" not in out


def test_job_without_run_folder_warns_and_can_be_made_an_error(tmp_path):
    jid = "20261102-100001-T"
    jd = write_job(jid, "starting\n", tmp_path, status="failed")
    out = status_tool(job_id=jid)
    assert out["run_dir_source"] == "job_dir" and out["run_dir"] == str(jd.resolve())
    assert "not <timestamp>_<run_name>" in out["run_dir_warning"]
    strict = status_tool(job_id=jid, require_run_dir=True)
    assert "has not written a Lab run folder" in strict["error"] and str(jd) in strict["error"]
    assert "run_dir" not in strict


def test_log_naming_a_missing_folder_is_not_trusted(tmp_path, _dirs):
    jid = "20261102-100002-T"
    ghost = _dirs / "rsl_rl" / "g1" / "2026-11-02_10-00-00_gone"
    write_job(jid, f"Exact experiment name requested from command line: {ghost}\n", tmp_path)
    assert status_tool(job_id=jid, require_run_dir=True)["error"]


def test_unknown_job_is_an_error():
    assert "unknown job" in status_tool(job_id="20261102-100003-Nope")["error"]


def test_lab3_logs_only_the_timestamp_and_the_run_name_is_added_to_the_folder(tmp_path, _dirs):
    """Lab 3 logs only the timestamp as the experiment name; the run name is appended to the folder ('..._smoke')."""
    run = make_run(_dirs, "g1", "2026-11-09_12-51-31", "smoke")
    (run / "model_1.pt").write_bytes(b"x")
    jid = "20261109-125128-T"
    write_job(jid, f"[INFO] Logging experiment in directory: {run.parent}\n"
                   "Exact experiment name requested from command line: 2026-11-09_12-51-31\n", tmp_path)
    out = status_tool(job_id=jid, require_run_dir=True)
    assert out["run_dir"] == str(run) and out["run_dir_source"] == "lab_log" and out["latest_checkpoint"].endswith("model_1.pt")


def test_two_candidate_folders_for_one_logged_timestamp_are_not_guessed(tmp_path, _dirs):
    make_run(_dirs, "g1", "2026-11-09_12-51-31", "a")
    make_run(_dirs, "g1", "2026-11-09_12-51-31", "b")
    jid = "20261109-125129-T"
    write_job(jid, f"[INFO] Logging experiment in directory: {_dirs / 'rsl_rl' / 'g1'}\n"
                   "Exact experiment name requested from command line: 2026-11-09_12-51-31\n", tmp_path)
    assert status_tool(job_id=jid)["run_dir_source"] == "job_dir"
