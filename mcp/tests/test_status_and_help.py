"""The one check-in call (status), waiting on simulator jobs, and help admitting what it cannot answer."""

import json
import socket
from pathlib import Path

import pytest
from PIL import Image

import twinsmith.server as gw
from twinsmith import screen, toolsets
from twinsmith.client import BridgeError

IDENTITY = {
    "pid": 4242, "port": 8766, "app": "Isaac Sim", "app_version": "6.2.0", "headless": True, "app_ready": True,
    "uptime_s": 12.5, "hostname": "lab", "bind_host": "127.0.0.1", "python": "3.12.0", "namespaces": ["sim", "assets"],
    "tools": 80, "busy": {"running": [], "waiting": 0, "exclusive": None},
    "jobs": {"running": 0, "jobs": []}, "read_only": False, "auth": True, "extension": {"commit": "unknown", "dirty": None},
}
STAGE = {"session_id": "s1", "stage_id": "g1", "source": "anon:0x1:World.usd", "default_prim": "/World", "playing": False,
         "stopped": True, "runtime": {"physics_engine": "newton"}, "prim_count": 9}


class Bridge:
    def __init__(self, answers):
        self.answers, self.calls, self.port = answers, [], 8766

    def call(self, tool, params=None, **kw):
        self.calls.append((tool, params))
        answer = self.answers[tool]
        if isinstance(answer, BaseException):
            raise answer
        return answer(params) if callable(answer) else answer


@pytest.fixture
def state(monkeypatch, tmp_path):
    monkeypatch.setattr(gw, "_advertised", lambda: [])
    monkeypatch.setattr(gw, "live_sessions", lambda env=None: [])
    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path / "jobs"))
    return gw._BridgeState()


def test_status_is_one_short_answer_with_the_stage_ids(state):
    state.client = Bridge({"status": IDENTITY, "sim.stage.info": STAGE})
    out = json.loads(gw._dispatch("status", {}, state))
    assert out["running"] is True and out["pid"] == 4242 and out["port"] == 8766 and out["app_ready"] is True
    assert out["stage"] == {"session_id": "s1", "stage_id": "g1", "source": "anon:0x1:World.usd", "default_prim": "/World",
                            "timeline": "stopped", "physics_engine": "newton"}
    assert not {"namespaces", "extension", "busy", "read_only", "bridges", "jobs_running"} & set(out)
    assert "no need to call this again" in out["note"]
    assert [tool for tool, _ in state.client.calls] == ["status", "sim.stage.info"], "one check-in is two cheap bridge reads"
    assert len(json.dumps(out)) < 2000


def test_status_detail_is_the_full_identity_and_never_reaches_an_older_bridge_as_an_argument(state):
    state.client = Bridge({"status": IDENTITY})
    out = json.loads(gw._dispatch("status", {"detail": True}, state))
    host = out.pop("host")
    assert "core" in out.pop("toolsets")["on"]
    assert set(out.pop("physics")) >= {"active", "default", "default_from"}
    assert out == IDENTITY and "version" in host
    assert state.client.calls[0] == ("status", None), "detail is the host's own switch"
    assert {name for name, _ in state.client.calls} <= {"status", "sim.stage.info"}   # the second one reads the active engine


def test_status_names_what_else_is_running(state, monkeypatch):
    busy = dict(IDENTITY, namespaces=[], read_only=True, busy={"running": ["run_python"], "waiting": 2, "exclusive": "run_python"},
                jobs={"running": 1, "jobs": [{"job_id": "job-1", "kind": "sim.simulation.run", "state": "running"}]})
    state.client = Bridge({"status": busy})
    monkeypatch.setattr(gw, "_advertised", lambda: [{"port": 8766, "pid": 4242}, {"port": 8770, "pid": 5151}])
    monkeypatch.setattr(gw, "live_sessions", lambda env=None: [{"id": "abc", "port": 8770, "pid": 5151, "physics_backend": "newton", "log": "x"}])
    out = json.loads(gw._dispatch("status", {}, state))
    assert out["jobs_running"][0]["job_id"] == "job-1" and out["busy"]["exclusive"] == "run_python" and out["read_only"] is True
    assert [b["port"] for b in out["bridges"]] == [8766, 8770]
    assert out["managed_sessions"] == [{"id": "abc", "port": 8770, "pid": 5151, "physics_backend": "newton"}]
    assert "sim-tools" in out["stage"]["note"], "a Kit without the scene pack says so instead of failing"


def test_nothing_running_is_an_answer_that_names_the_next_call(state, monkeypatch):
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free = probe.getsockname()[1]
    monkeypatch.setenv("TWINSMITH_PORT", str(free))
    text = gw._dispatch("status", {}, state)
    out = json.loads(text)
    assert out["running"] is False and "configured simulator" in out["message"] and "refresh_tools(wait_s=120)" in out["message"]
    assert out["bridges"] == [] and "version" in out["host"] and not gw._failed_tool_result(text)
    monkeypatch.setattr(gw, "_advertised", lambda: [{"port": 8770, "pid": 5151, "app": "Isaac Sim"}])
    elsewhere = json.loads(gw._dispatch("status", {}, state))
    assert elsewhere["running"] is False and "use_bridge(port=8770)" in elsewhere["message"]
    state.client = Bridge({"status": RuntimeError("multiple bridges running (ports 8766, 8770): set TWINSMITH_PORT")})
    several = json.loads(gw._dispatch("status", {}, state))
    assert several["running"] is False and "use_bridge(port)" in several["message"] and "Do not start another" in several["message"]


def test_a_bridge_that_answers_badly_is_still_an_error(state):
    for failure in (TimeoutError("timed out"), BridgeError("AUTH_REQUIRED: this bridge needs its token")):
        state.client = Bridge({"status": failure})
        assert gw._dispatch("status", {}, state).startswith("Error:")
    state.client = Bridge({"status": IDENTITY, "sim.stage.info": BridgeError("APP_STARTING")})
    assert "APP_STARTING" in json.loads(gw._dispatch("status", {}, state))["stage"]["error"]


# ---- job_status through the host ----

def test_a_simulator_job_s_wait_is_served_by_the_simulator_in_one_call(state):
    state.client = Bridge({"job_status": lambda params: {"job_id": "job-1", "state": "succeeded", "result": {"steps": 5}}})
    out = json.loads(gw._dispatch("job_status", {"job_id": "job-1", "wait_s": 30}, state))
    assert out["state"] == "succeeded"
    assert state.client.calls == [("job_status", {"job_id": "job-1", "wait_s": 30})], "one call: the bridge waits, not the host"
    for bad in (-1, True, float("nan"), "30"):
        refused = gw._dispatch("job_status", {"job_id": "job-1", "wait_s": bad}, state)
        assert gw._failed_tool_result(refused) and "wait_s" in refused
    assert len(state.client.calls) == 1, "a bad wait is refused before anything is asked"


def test_a_longer_wait_waits_the_most_a_call_may_instead_of_failing(state):
    state.client = Bridge({"job_status": lambda params: {"job_id": "job-1", "state": "running", "wait_timed_out": True}})
    out = json.loads(gw._dispatch("job_status", {"job_id": "job-1", "wait_s": 300}, state))
    assert state.client.calls == [("job_status", {"job_id": "job-1", "wait_s": 60.0})], "the simulator gets 60, not 300"
    assert out["wait_s_used"] == 60.0 and "Call again" in out["wait_note"] and out["state"] == "running" and "note" not in out


def test_a_clamped_wait_keeps_the_jobs_own_note_and_stops_at_sixty_on_the_host(state, monkeypatch):
    """A Lab or script job is waited for on the host: the clamp holds there too, and a failed job keeps its reason."""
    from twinsmith.isaaclab import jobs as lab_jobs

    clock = {"now": 0.0}
    monkeypatch.setattr("time.monotonic", lambda: clock["now"])
    monkeypatch.setattr("time.sleep", lambda s: clock.__setitem__("now", clock["now"] + s))
    monkeypatch.setattr(gw, "_job_record", lambda job_id: {"runner": "lab", "status": "running"})
    states = {"running": {"state": "running"}, "failed": {"state": "failed", "note": "the simulator exited first"}}
    current = {"state": "running"}
    monkeypatch.setattr(lab_jobs, "runs_status", lambda job_id: dict(states[current["state"]]))
    out = json.loads(gw._dispatch("job_status", {"job_id": "lab-1", "wait_s": 300}, state))
    assert out["state"] == "running" and out["wait_timed_out"] and out["wait_s_used"] == 60.0
    assert 60.0 <= clock["now"] < 61.0, clock          # stopped at the clamp, not 300 s
    current["state"] = "failed"
    ended = json.loads(gw._dispatch("job_status", {"job_id": "lab-1", "wait_s": 300}, state))
    assert ended["note"] == "the simulator exited first" and ended["wait_s_used"] == 60.0 and "wait_note" not in ended


def replay_result(folder, frames=4):
    folder.mkdir(parents=True, exist_ok=True)
    for index in range(1, frames + 1):
        Image.new("RGB", (64, 48), (index * 40, 10, 10)).save(folder / f"flight-{index:05d}.png")
    return {"job_id": "job-7", "state": "succeeded",
            "result": {"frames": frames, "encode": {"folder": str(folder), "pattern": "flight-*.png", "frames": frames,
                                                    "fps": 2.0, "label": "PhysX hover", "output": "flight.mp4"}}}


def test_a_finished_replay_gets_its_labelled_video_once(state, monkeypatch, tmp_path):
    try:
        screen._ffmpeg()
    except RuntimeError:
        pytest.skip("no ffmpeg")
    monkeypatch.setattr(screen, "VIDEO_DIR", tmp_path / "videos")
    state.client = Bridge({"job_status": replay_result(tmp_path / "out")})
    first = json.loads(gw._dispatch("job_status", {"job_id": "job-7"}, state))["result"]
    assert first["video_path"] == str(tmp_path / "videos" / "flight.mp4") and Path(first["video_path"]).stat().st_size > 0
    assert first["video"]["frames"] == 4 and first["video"]["label"] == "PhysX hover" and first["video"]["duration_s"] == 2.0
    written = Path(first["video_path"]).stat().st_mtime_ns
    again = json.loads(gw._dispatch("job_status", {"job_id": "job-7"}, state))["result"]
    assert again["video_path"] == first["video_path"] and Path(first["video_path"]).stat().st_mtime_ns == written
    assert sorted(p.name for p in (tmp_path / "videos").iterdir()) == ["flight.mp4"]


def test_frames_this_server_cannot_read_are_kept_and_the_reply_says_how_to_encode_them(state, monkeypatch, tmp_path):
    monkeypatch.setattr(screen, "VIDEO_DIR", tmp_path / "videos")
    reply = replay_result(tmp_path / "out")
    reply["result"]["encode"]["folder"] = str(tmp_path / "on-another-machine")
    state.client = Bridge({"job_status": reply})
    result = json.loads(gw._dispatch("job_status", {"job_id": "job-7"}, state))["result"]
    assert "video_path" not in result and "0 of 4 frames" in result["video_error"] and 'join_videos(mode="frames"' in result["video_error"]


# ---- help ----

# A usual selection for scene work: what these help answers assume.
EVERYDAY = frozenset({"core", "sim", "assets", "computer"})

def gaps(tmp_path):
    file = tmp_path / "state" / "help-gaps.jsonl"
    return [json.loads(line) for line in file.read_text().splitlines()] if file.exists() else []


def test_a_drone_controller_question_gets_the_controller_runner_not_the_arm_tools(tmp_path):
    answer = gw._help("run my own controller on a drone", enabled=EVERYDAY)
    assert "sim__simulation__run" in answer and "tcp://127.0.0.1" in answer and "job_status" in answer
    assert "sim__arm__ik" not in answer and "sim__joints__move" not in answer
    assert "help(task=..., no_fit=true)" in answer and gaps(tmp_path) == [], "an answered question is not a gap"


def test_replay_and_job_questions_name_their_tools():
    replay = gw._help("replay saved flights into a labelled video", enabled=EVERYDAY)
    assert "sim__trajectory__replay" in replay and "video_path" in replay
    waiting = gw._help("is it done yet, how do I wait for a long loop", enabled=EVERYDAY)
    assert "job_status(job_id, wait_s=30)" in waiting and "background=true" in waiting
    assert "run_script" in gw._help("run a script as a managed job outside isaac", enabled=EVERYDAY)
    assert "action=reset" in gw._help("the timeline was left playing, reset to a clean state", enabled=EVERYDAY)
    assert "Available: status" in gw._help("where am i, what is running", enabled=EVERYDAY - {"computer"})


def test_an_unanswered_question_says_so_gives_the_pattern_and_is_recorded_locally(tmp_path):
    answer = gw._help("simulate cloth tearing", enabled=EVERYDAY)
    assert answer.startswith("No twinsmith tool is known to fit 'simulate cloth tearing'.")
    assert "background=true" in answer and "next_update_async" in answer and str(tmp_path / "state" / "help-gaps.jsonl") in answer
    (record,) = gaps(tmp_path)
    assert record["task"] == "simulate cloth tearing" and record["reason"] == "no_match" and "time" in record


def test_no_fit_records_a_task_whose_named_tools_did_not_fit(tmp_path):
    answer = gw._help("move a joint along a spline with jerk limits", no_fit=True, enabled=EVERYDAY)
    assert answer.startswith("No twinsmith tool is known to fit") and "The closest topics" in answer and "sim__joints__move" in answer
    assert [g["reason"] for g in gaps(tmp_path)] == ["no_fit"]
    assert "full map" not in gw._help(None, no_fit=True, enabled=EVERYDAY) and len(gaps(tmp_path)) == 1, "no task, nothing to record"


def test_recording_can_be_turned_off_and_never_breaks_the_answer(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_LOG", "0")
    assert "recorded for the maintainer" not in gw._help("simulate cloth tearing") and gaps(tmp_path) == []
    monkeypatch.setenv("TWINSMITH_LOG", "1")
    (tmp_path / "state").write_text("a file where the folder should be")
    assert gw._help("simulate cloth tearing").startswith("No twinsmith tool is known to fit")


# ---- one pair of tools follows every job: simulator jobs and process jobs ----

def test_job_status_and_cancel_also_take_a_process_job_and_need_no_bridge(state, tmp_path, monkeypatch):
    import sys
    import time

    from twinsmith.isaaclab import jobs

    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [])
    script = tmp_path / "wait.py"
    script.write_text("import time\nprint('TWINSMITH_PROGRESS: 1/4 started', flush=True)\ntime.sleep(60)\n")
    started = jobs.run_script(sys.executable, script, uses_gpu=False, timeout_s=120)
    state.client = Bridge({})   # any bridge call would raise KeyError: none is made
    deadline = time.time() + 20
    while time.time() < deadline:
        running = json.loads(gw._dispatch("job_status", {"job_id": started["job_id"], "wait_s": 0.2}, state))
        if running.get("progress"):
            break
    assert running["state"] == "running" and running["wait_timed_out"] is True and running["progress"]["fraction"] == 0.25
    assert running["health"] == "ok" and running["job_id"] == started["job_id"] and state.client.calls == []
    cancelled = json.loads(gw._dispatch("job_cancel", {"job_id": started["job_id"]}, state))
    assert "error" not in cancelled
    final = json.loads(gw._dispatch("job_status", {"job_id": started["job_id"], "wait_s": 20}, state))
    assert final["state"] == "cancelled" and "wait_timed_out" not in final
    state.client = Bridge({"job_status": BridgeError("unknown job '20260101-000000-nothing'")})
    missing = gw._dispatch("job_status", {"job_id": "20260101-000000-nothing"}, state)
    assert gw._failed_tool_result(missing) and "unknown job" in missing, "no record here: the simulator is asked, and says so"


def kit_record(tmp_path, job_id, **fields):
    import os

    from twinsmith.isaaclab import jobs

    folder = tmp_path / "jobs" / job_id
    folder.mkdir(parents=True)
    record = {"id": job_id, "job_id": job_id, "kind": "sim.simulation.run", "task": "hover", "label": "hover", "runner": "kit",
              "uses_gpu": False, "status": "running", "pid": os.getpid(), "pid_start": jobs._process_start(os.getpid()),
              "port": 8766, "started_at": 1000.0, "finished_at": None, "exit_code": None, "log_dir": str(folder), **fields}
    (folder / "job.json").write_text(json.dumps(record))
    return record


def test_a_running_simulator_job_is_asked_of_its_simulator_and_a_finished_one_is_read_from_its_record(state, tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_PORT", "8766")
    kit_record(tmp_path, "20261111-120000-sim.simulation.run-1", progress={"fraction": 0.1})
    state.client = Bridge({"job_status": lambda params: {"job_id": params["job_id"], "state": "running", "output_tail": "live"},
                           "job_cancel": lambda params: {"job_id": params["job_id"], "cancel_requested": True}})
    live = json.loads(gw._dispatch("job_status", {"job_id": "20261111-120000-sim.simulation.run-1"}, state))
    assert live["output_tail"] == "live", "while it runs, the simulator's own answer"
    assert json.loads(gw._dispatch("job_cancel", {"job_id": "20261111-120000-sim.simulation.run-1"}, state))["cancel_requested"] is True
    kit_record(tmp_path, "20261111-120500-sim.trajectory.replay-2", status="succeeded", finished_at=1012.5, result={"frames": 45})
    calls = len(state.client.calls)
    done = json.loads(gw._dispatch("job_status", {"job_id": "20261111-120500-sim.trajectory.replay-2"}, state))
    assert done["state"] == "succeeded" and done["result"] == {"frames": 45} and done["elapsed_s"] == 12.5 and done["runner"] == "kit"
    assert len(state.client.calls) == calls, "a finished job needs no simulator: its record answers"


def test_a_simulator_job_whose_simulator_is_gone_reads_failed_and_cannot_signal_anything(state, tmp_path):
    kit_record(tmp_path, "20261111-130000-run_python-3", pid=999999999, pid_start="1")
    state.client = Bridge({})
    gone = json.loads(gw._dispatch("job_status", {"job_id": "20261111-130000-run_python-3"}, state))
    assert gone["state"] == "failed" and "simulator exited" in gone["note"] and gone["pid_alive"] is False
    kit_record(tmp_path, "20261111-130100-run_python-4")     # this test process stands in for a live simulator
    from twinsmith.isaaclab import jobs

    with pytest.raises(ValueError, match="runs inside Isaac Sim"):
        jobs.runs_cancel("20261111-130100-run_python-4")


def test_the_job_list_holds_both_kinds_newest_first(state, tmp_path, monkeypatch):
    import sys

    from twinsmith.isaaclab import jobs

    monkeypatch.setattr(jobs, "_probe_kit_bridges", lambda: [])
    kit_record(tmp_path, "20200101-000000-sim.simulation.run-1", status="succeeded", finished_at=1001.0)
    script = tmp_path / "quick.py"
    script.write_text("print('done')\n")
    started = jobs.run_script(sys.executable, script, uses_gpu=False)
    state.client = Bridge({"job_status": {"jobs": [{"job_id": "20991231-235959-run_python-9", "kind": "run_python", "state": "running"}], "running": 1}})
    listing = json.loads(gw._dispatch("job_status", {}, state))
    ids = [job["job_id"] for job in listing["jobs"]]
    assert ids == ["20991231-235959-run_python-9", started["job_id"], "20200101-000000-sim.simulation.run-1"]
    assert listing["jobs"][0]["runner"] == "kit" and listing["jobs"][2]["runner"] == "kit" and "runner" not in listing["jobs"][1]
    assert listing["running"] >= 1
    state.client = Bridge({"job_status": ConnectionRefusedError("no simulator")})
    assert len(json.loads(gw._dispatch("job_status", {}, state))["jobs"]) == 2, "with no simulator the records are the list"


def test_the_check_in_does_not_reread_jobs_and_sessions_that_have_ended(tmp_path, monkeypatch):
    from twinsmith.isaaclab import jobs as lab_jobs

    monkeypatch.setenv("TWINSMITH_JOBS_DIR", str(tmp_path))
    for name, status in (("a-done", "finished"), ("b-failed", "failed")):
        (tmp_path / name).mkdir()
        (tmp_path / name / "job.json").write_text(json.dumps({"id": name, "status": status, "pid": 1}))
    read = []
    original = lab_jobs._read_job
    monkeypatch.setattr(lab_jobs, "_read_job", lambda folder: (read.append(folder.name), original(folder))[1])
    assert lab_jobs.running_jobs() == [] and sorted(read) == ["a-done", "b-failed"]
    (tmp_path / "c-new").mkdir()
    (tmp_path / "c-new" / "job.json").write_text(json.dumps({"id": "c-new", "status": "finished"}))
    del read[:]
    assert lab_jobs.running_jobs() == [] and read == ["c-new"], "only the job not seen before is read"


@pytest.mark.parametrize("task", [
    "click a button in FreeCAD and type into a text editor window",
    "operate a desktop app with mouse and keyboard",
    "open the Window menu in Isaac Sim UI",
    "computer use",
])
def test_help_knows_the_computer_use_tools(task):
    answer = gw._help(task, enabled=toolsets.DEFAULT | {"computer", "sim"})
    assert not answer.startswith("No twinsmith tool is known to fit")
    assert "doctor" in answer and "computer" in answer and "sim__ui" in answer and "coord_space" in answer
