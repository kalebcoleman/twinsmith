"""isaaclab__runs_status wait_s: block until the job ends, bounded, without starving other calls."""

import asyncio
import json
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import jobs, tools


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [])
    monkeypatch.setattr(tools, "_WAIT_POLL_S", 0.05)
    jobs._active.clear()
    yield
    for proc in list(jobs._active.values()):
        try:
            os.killpg(proc.pid, 9)
        except OSError:
            pass
    jobs._active.clear()


def status_fn():
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    return server._tool_manager._tools["isaaclab__runs_status"].fn


def launch(tmp_path, body):
    p = tmp_path / "s.py"
    p.write_text(body)
    return jobs.run_script(sys.executable, p, timeout_s=60)["job_id"]


def test_wait_returns_final_status_with_exit_code_and_tail(tmp_path):
    fn = status_fn()  # build the server first: on a slow runner its import took most of the job's sleep
    jid = launch(tmp_path, "import time,sys\nprint('hello')\ntime.sleep(0.6)\nprint('bye')\nsys.exit(5)\n")
    t0 = time.time()
    out = json.loads(asyncio.run(fn(job_id=jid, wait_s=20)))
    elapsed = time.time() - t0
    assert out["state"] == "failed" and out["exit_code"] == 5 and "wait_timed_out" not in out
    # The final state proves the call waited for the end; how long depends on the runner, so only bound it.
    assert elapsed < 10 and 0 < out["waited_s"] <= elapsed + 0.1
    assert "bye" in "\n".join(out["last_log_lines"])


def test_wait_times_out_still_running_with_tail(tmp_path):
    jid = launch(tmp_path, "import time\nprint('tick', flush=True)\ntime.sleep(30)\n")
    out = json.loads(asyncio.run(status_fn()(job_id=jid, wait_s=0.5)))
    assert out["state"] == "running" and out["wait_timed_out"] is True and out["waited_s"] >= 0.5
    assert out["exit_code"] is None and "tick" in "\n".join(out["last_log_lines"])
    jobs.runs_cancel(jid, grace_s=2)


def test_finished_job_returns_at_once(tmp_path):
    jid = launch(tmp_path, "print(1)\n")
    asyncio.run(status_fn()(job_id=jid, wait_s=10))
    t0 = time.time()
    out = json.loads(asyncio.run(status_fn()(job_id=jid, wait_s=30)))
    assert out["state"] == "succeeded" and time.time() - t0 < 2


def test_waiting_does_not_starve_other_calls(tmp_path):
    jid = launch(tmp_path, "import time\ntime.sleep(30)\n")
    fn = status_fn()

    async def scenario():
        waiter = asyncio.create_task(fn(job_id=jid, wait_s=1.5))
        await asyncio.sleep(0.2)
        t0 = time.time()
        quick = json.loads(await fn(job_id=jid))          # a plain status while the wait is pending
        listing = json.loads(await asyncio.to_thread(tools._guard, lambda: jobs.runs_list()))
        assert time.time() - t0 < 1.0 and quick["state"] == "running" and listing["total"] == 1
        assert not waiter.done()
        return json.loads(await waiter)

    assert asyncio.run(scenario())["wait_timed_out"] is True
    jobs.runs_cancel(jid, grace_s=2)


def test_cancel_during_wait_ends_it(tmp_path):
    jid = launch(tmp_path, "import time\ntime.sleep(30)\n")
    fn = status_fn()

    async def scenario():
        waiter = asyncio.create_task(fn(job_id=jid, wait_s=20))
        await asyncio.sleep(0.3)
        await asyncio.to_thread(jobs.runs_cancel, jid, 2)
        return json.loads(await asyncio.wait_for(waiter, 10))

    out = asyncio.run(scenario())
    assert out["state"] == "cancelled" and "wait_timed_out" not in out


@pytest.mark.parametrize("kwargs, text", [
    ({"wait_s": 5}, "needs a job_id"),
    ({"job_id": "20261102-100000-T", "wait_s": 99999}, "between 0 and 3600"),
    ({"job_id": "20261102-100000-T", "wait_s": -1}, "between 0 and 3600"),
    ({"job_id": "20261102-100000-Nope", "wait_s": 5}, "unknown job"),
])
def test_bad_wait_is_an_error(kwargs, text):
    assert text in json.loads(asyncio.run(status_fn()(**kwargs)))["error"]
