"""No RSS, network or GPU required: exercise its executable boundary."""

import asyncio
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

from twinsmith.robosim import PIN, RoboSimBridge, register_robosim_tools
from twinsmith.robosim import bridge as bridge_module


STUB = r'''
import hashlib, json, os, pathlib, re, signal, subprocess, sys, time
args = sys.argv[1:]
if args[:1] == ["-I"]:      # the bridge's take_scores.py, run in this interpreter
    assert pathlib.Path(args[1]).name == "take_scores.py", args
    request = json.loads(pathlib.Path(args[2]).read_text())
    run = pathlib.Path(request["run_dir"])
    args = ["take-scores"]
    if request["take"] == "unscorable":
        sys.stderr.write("take 'unscorable' has no segmentation")
        sys.exit(1)
else:
    run = pathlib.Path(args[args.index("--run-dir") + 1])
    if "robosimstudio_real2sim_agent.serve" in args:
        while True:
            time.sleep(0.1)
    args = args[args.index("--run-dir") + 2:]
action = args[0]
with (run / "calls.jsonl").open("a") as output:
    output.write(json.dumps({"args": args, "cwd": os.getcwd(), "env": {
        key: os.environ.get(key) for key in ["CUDA_VISIBLE_DEVICES", "ROBOSIMSTUDIO_DEVICE",
        "TMPDIR", "XDG_CACHE_HOME", "WARP_CACHE_PATH", "PYTHONDONTWRITEBYTECODE"]}}) + "\n")
def option(key):
    return args[args.index(key) + 1]
def write(name, text="fake"):
    path = pathlib.Path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return str(path)
def fake_physics(push):
    """Image displacement of the fake block as a known function of the push: 0.5 of the pusher's travel, turned 0.3 rad
    counter-clockwise, at 480 px per metre, image y pointing down (+y on the table is up in the picture)."""
    import math
    if not push or not push.get("waypoints"):
        return [0.0, 0.0]
    w = push["waypoints"]
    dx, dy = w[-1][1] - w[0][1], w[-1][2] - w[0][2]
    angle = math.atan2(dy, dx) + 0.3
    length = 0.5 * math.hypot(dx, dy)
    return [round(480 * length * math.cos(angle), 1), round(-480 * length * math.sin(angle), 1)]
def state():
    path = run / "state.json"
    return json.loads(path.read_text()) if path.exists() else {"generation": 1, "resets": 0, "last_error": ""}
def save(value):
    (run / "state.json").write_text(json.dumps(value))
reply = {"ok": True}
if action == "up":
    proc = subprocess.Popen([sys.executable, __file__, "-m", "robosimstudio_real2sim_agent.serve",
                             "--run-dir", str(run)], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    write(run / "serve.pid", str(proc.pid))
    time.sleep(0.05)
    reply.update(pid=proc.pid, url="http://localhost:" + option("--port"))
    if "BREAK" in (run / "scene.py").read_text():
        save({"generation": 1, "resets": 0, "last_error": "Traceback: scene would not build"})
elif action == "quit":
    if not (run / "no-quit").exists():
        os.kill(int((run / "serve.pid").read_text()), signal.SIGTERM)
elif action == "reload":
    current = state()
    if "BREAK" in (run / "scene.py").read_text():
        current["last_error"] = "Traceback: scene would not build"
        save(current)
        reply.update(ok=False, error=current["last_error"])
    else:
        current.update(generation=current["generation"] + 1, last_error="")
        save(current)
        reply.update(seconds=0.1, names=["block", "pusher", "table"])
elif action == "reset":
    current = state()
    current["resets"] += 1
    save(current)
elif action == "eval":
    code = " ".join(args[1:-1])
    if code.startswith("float("):
        reply.update(value=os.environ.get("FAKE_HZ", "50.0"), stdout="", playing=False)
    else:
        write(run / "rollout.txt", code)
        reply.update(value=None, stdout="", playing=False)
elif action == "ref":
    out = pathlib.Path(option("--out"))
    clip_text = pathlib.Path(args[2]).read_text()
    duration = float(clip_text.split()[1]) if clip_text.startswith("trimmed ") else 12.0
    anchors = []
    for item in option("--at").split(","):
        when, _, label = item.partition(":")
        t = round(duration - 0.001, 3) if when == "end" else float(when)
        write(out / f"t{t:06.2f}.jpg")
        anchors.append({"t": t, "file": f"t{t:06.2f}.jpg", "label": label})
    write(out / "timeline.json", json.dumps({"clip": "fake", "duration": duration, "anchors": anchors}))
    reply.update(frames=[a["file"] for a in anchors], contact=write(out / "contact.jpg"),
                 timeline=str(out / "timeline.json"), duration=duration, fps=10.0)
elif action == "snapshot":
    reply["image"] = write(option("--out"))
elif action == "compare":
    def reading(centre):
        return {"photo": {"centre": [100.0, 100.0], "pixels": 1000}, "sim": {"centre": centre, "pixels": 1100},
                "iou": 0.5, "offset_px": [2, 3], "width_ratio": 1.1, "height_ratio": 0.9, "size_ratio": 1.049}
    if "--take" in args:
        timeline = json.loads(pathlib.Path(option("--timeline")).read_text())
        rows = []
        last = len(timeline["anchors"]) - 1
        full = [-40.0, -40.0]
        if (run / "physics.json").exists():
            full = fake_physics(json.loads((run / (option("--take") + ".json")).read_text()).get("push"))
        for i, anchor in enumerate(timeline["anchors"]):
            moved = [100.0 + full[0] * i / last, 100.0 + full[1] * i / last]
            row_photo = [100.0 - 50 * i / last, 100.0 - 60 * i / last]
            entry = reading(moved)
            entry["photo"] = dict(entry["photo"], centre=row_photo)
            # RSS at the pin: which frame each anchor was held against, and no per-object readings
            rows.append({"t": anchor["t"], "label": anchor["label"], "file": anchor["file"], "frame": i,
                         "sim_t": anchor["sim_t"], "sim_step": round(anchor["sim_t"] * 50)})
            if (run / "later_rss.json").exists():      # a later RSS that returns the readings itself
                rows[-1]["objects"] = {"block": dict(reading(moved), photo={"centre": row_photo, "pixels": 1000}, iou=0.7)}
            if (run / "no_frame.json").exists():
                del rows[-1]["frame"]
        reply.update(anchors=rows, verdict="1 of 1 did what the clip did", score=0.4, strip=write(run / "strip.png"))
    else:
        reply.update(objects={"block": reading([102.0, 103.0])}, beside=write(option("--out")))
elif action == "take-scores":
    def reading(centre):
        return {"photo": {"centre": [100.0, 100.0], "pixels": 1000}, "sim": {"centre": centre, "pixels": 1100},
                "iou": 0.5, "offset_px": [2, 3], "width_ratio": 1.1, "height_ratio": 0.9, "size_ratio": 1.049}
    full = [-40.0, -40.0]
    if (run / "physics.json").exists():
        full = fake_physics(json.loads((run / (request["take"] + ".json")).read_text()).get("push"))
    last = len(request["anchors"]) - 1
    found = []
    for i, anchor in enumerate(request["anchors"]):
        assert anchor["frame"] == i and anchor["file"], anchor
        entry = reading([100.0 + full[0] * i / last, 100.0 + full[1] * i / last])
        entry["photo"] = dict(entry["photo"], centre=[100.0 - 50 * i / last, 100.0 - 60 * i / last])
        found.append({"frame": i, "objects": {key.split("=", 1)[0]: entry for key in request["keys"]}})
    reply["anchors"] = found
elif action == "play":
    text = (run / "rollout.txt").read_text() if (run / "rollout.txt").exists() else "ROLLOUT = {}"
    rollout = eval(text.split("=", 1)[1], {"None": None})
    write(run / "pending.json", json.dumps({"take": option("--record"), "at": time.time(), "every": int(option("--every")),
                                            "rollout": rollout, "routine": args[1] if not args[1].startswith("--") else None}))
    reply["playing"] = args[1] if not args[1].startswith("--") else "push"
elif action == "status":
    current = state()
    reply.update(playing=0, recording="", last_take="", last_error=current["last_error"], names=["block", "pusher", "table"],
                 cameras=["camera"], generation=current["generation"], resets=current["resets"],
                 routines=["push", "settle"], board=["a proportional verdict"])
    pending = run / "pending.json"
    if pending.exists():
        data = json.loads(pending.read_text())
        take = data["take"]
        if time.time() - data["at"] < 0.2:
            reply.update(playing=1, recording=take)
        elif not (run / (take + ".json")).exists():
            steps = data["rollout"].get("steps", 100)
            every = data["every"]
            marks = list(range(0, steps + 1, every)) + ([steps] if steps % every else [])
            frames = [{"i": i, "step": m, "t": round(m / 50.0, 4)} for i, m in enumerate(marks)]
            write(run / (take + ".mp4"))
            write(run / (take + ".json"), json.dumps({"name": take, "every": every, "fps": round(50 / every), "control_hz": 50.0,
                                                      "frames": frames, "push": data["rollout"].get("push")}))
            if data["rollout"].get("push") and "IGNORE_PUSH" not in (run / "scene.py").read_text():
                missed = "UNTRACKED_PUSHER" in (run / "scene.py").read_text()
                write(run / ("push-" + take + ".json"), json.dumps({"applied": True, "pusher_tracked": not missed,
                                                                    "pusher_tracking_error_max_m": 0.32423 if missed else 0.002,
                                                                    "solver": "SolverVBD" if missed else "SolverMuJoCo",
                                                                    "object_start_xy_m": [0.0, -0.075]}))
            reply["last_take"] = take
        else:
            reply["last_take"] = take
print(json.dumps(reply))
'''


@pytest.fixture
def fake(tmp_path, monkeypatch):
    root = tmp_path / "rss"
    python = root / ".venv/bin/python"
    module = root / "robosimstudio_real2sim_agent/__main__.py"
    python.parent.mkdir(parents=True)
    module.parent.mkdir(parents=True)
    python.write_text(f"#!{sys.executable}\n" + STUB)
    python.chmod(0o755)
    module.write_text("")
    original_run = subprocess.run
    gpu_calls = []

    ffmpeg_calls = []

    def run(argv, **kwargs):
        if argv[0].endswith("ffmpeg"):
            ffmpeg_calls.append(list(argv))
            if "-t" not in argv:  # the header probe: ffmpeg -hide_banner -i FILE
                return subprocess.CompletedProcess(argv, 1, "", "  Duration: 00:00:12.00, start: 0.000000, bitrate: 89 kb/s\n    Stream #0:0: Video: h264, yuv420p, 320x240, 10 fps, 10 tbr\n")
            Path(argv[-1]).write_text("trimmed " + argv[argv.index("-t") + 1])
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[0] == "git":
            return subprocess.CompletedProcess(argv, 0, PIN + "0" * 33 + "\n", "")
        if argv[0] == "nvidia-smi":
            gpu_calls.append(argv)
            return subprocess.CompletedProcess(argv, 0, "4242, somebody-elses-sim\n", "")
        return original_run(argv, **kwargs)

    monkeypatch.setattr(subprocess, "run", run)
    real_which = shutil.which
    monkeypatch.setattr(shutil, "which", lambda name, *a, **k: "/fake/bin/ffmpeg" if name == "ffmpeg" else real_which(name, *a, **k))
    bridge = RoboSimBridge(workspace=tmp_path / "workspace", root=root)
    bridge.ffmpeg_calls = ffmpeg_calls
    scene = tmp_path / "scene.py"
    scene.write_text("scene = 'fake'\n")
    video = tmp_path / "input.mp4"
    video.write_bytes(b"video")
    yield bridge, scene, video, gpu_calls
    for future in list(bridge._jobs.values()):
        future.result(timeout=10)
    for path in bridge.runs.glob("*/run.json"):
        bridge.call("stop", run=path.parent.name, wait_s=10)


def start(fake, **kwargs):
    bridge, scene, _, _ = fake
    return bridge.call("start", name="test", scene=str(scene), device="cpu", wait_s=10, **kwargs)


def complete(bridge, action, run, **kwargs):
    result = bridge.call(action, run=run, wait_s=10, **kwargs)
    assert result["ok"], (result.get("error"), result.get("operation"))
    assert result["operation"]["state"] == "completed", result
    return result["operation"]["result"]


@pytest.mark.parametrize("action", ["status", "scene_template", "start", "update_scene", "reference", "snapshot", "compare", "rollout", "tune_push", "compare_take", "stop", "report"])
def test_missing_install(tmp_path, action):
    result = RoboSimBridge(workspace=tmp_path).call(action)
    assert not result["ok"]
    assert PIN in result["error"] and "https://github.com/robosimstudio/robosimstudio" in result["error"]
    assert str(tmp_path / "tools/robosimstudio") in result["error"]
    assert result["rss_commit"] is None


def test_pin_mismatch(fake, monkeypatch):
    bridge = fake[0]
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: subprocess.CompletedProcess(args, 0, "abcdef0\n", ""))
    result = bridge.call("scene_template")
    assert result["ok"] and not result["pin_match"]
    assert result["rss_commit"] == "abcdef0"
    assert "differs" in result["warnings"][0]


def test_missing_venv(fake):
    bridge = fake[0]
    (bridge.root / ".venv/bin/python").unlink()
    assert "private environment missing" in bridge.call("status")["error"]


def test_unexecutable_install(fake):
    bridge = fake[0]
    (bridge.root / ".venv/bin/python").chmod(0o644)
    assert "private environment missing" in bridge.call("status")["error"]


def test_missing_module(fake):
    bridge = fake[0]
    (bridge.root / "robosimstudio_real2sim_agent/__main__.py").unlink()
    assert "private environment missing" in bridge.call("status")["error"]


def test_host_python_310_hashing(fake, monkeypatch):
    monkeypatch.delattr(hashlib, "file_digest", raising=False)
    bridge, scene, _, _ = fake
    result = start(fake)
    _, ledger = bridge._load(result["run"])
    assert ledger["inputs"]["scene"]["sha256"] == hashlib.sha256(scene.read_bytes()).hexdigest()


def test_name_length_before_creating_run(fake):
    bridge, scene, _, _ = fake
    result = bridge.call("start", name="a" * 65, scene=str(scene), device="cpu")
    assert "at most 64" in result["error"]
    assert not bridge.runs.exists()


def test_explicit_device_and_gpu_refusal(fake):
    bridge, scene, _, calls = fake
    result = bridge.call("start", name="test", scene=str(scene))
    assert "explicit device" in result["error"]
    result = bridge.call("start", name="test", scene=str(scene), device="cuda:0")
    assert "Other GPU work" in result["error"]
    assert calls
    assert not bridge.runs.exists()
    bridge._gpu_guard(True)
    bridge._gpu_guard(False, owned_pid=4242)


def test_gpu_check_fails_closed(fake, monkeypatch):
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: subprocess.CompletedProcess(args, 1, "", "failed"))
    with pytest.raises(ValueError, match="Cannot check GPU"):
        fake[0]._gpu_guard(False)


@pytest.mark.parametrize("error", [FileNotFoundError(), subprocess.TimeoutExpired("nvidia-smi", 10)])
def test_unavailable_gpu_check(fake, monkeypatch, error):
    def fail(*args, **kwargs):
        raise error

    monkeypatch.setattr(subprocess, "run", fail)
    with pytest.raises(ValueError, match="Cannot check GPU"):
        fake[0]._gpu_guard(False)


def test_complete_workflow_and_containment(fake):
    bridge, scene, video, gpu_calls = fake
    result = start(fake)
    assert result["ok"] and result["server_alive"]
    assert result["engine"]["physics"].startswith("plain MuJoCo")
    run = result["run"]
    path = Path(result["run_dir"])
    assert path.parent == bridge.runs and path.name.endswith("-test")
    assert (path / "scene.py").read_bytes() == scene.read_bytes() == (path / "scene-v1.py").read_bytes()
    reference = complete(bridge, "reference", run, video=str(video), at="0:start,6:end", width=100)
    assert reference["window"] is None
    complete(bridge, "snapshot", run, camera="camera", size="80x60")
    compare = complete(bridge, "compare", run, keys=["block=red"], camera="camera", size="80x60")
    assert compare["metrics"]["block"]["iou"] == 0.5
    take = complete(bridge, "rollout", run, routine="push", take="take1", duration_s=6.0, record_fps=10, camera="camera",
                    push={"waypoints": [[0, 0.1, -0.2], [6, 0.0, 0.0]]})
    assert Path(take["mp4"]).is_file() and Path(take["json"]).is_file()
    complete(bridge, "compare_take", run, take="take1", keys=["block=red"])
    report = complete(bridge, "report", run)
    text = Path(report["report"]).read_text()
    assert "0.5" in text and "does not fit mass or friction" in text and "plain MuJoCo" in text
    assert "rollout `take1`: 300 steps = 6.0 s simulated" in text and "scene-v1.py" in text
    assert bridge.call("status")["live_servers"]
    assert complete(bridge, "stop", run)["stopped"]
    assert not bridge.call("status", run=run)["server_alive"]
    ledger = json.loads((path / "run.json").read_text())
    assert ledger["rss_commit"].startswith(PIN)
    assert ledger["inputs"]["scene"]["sha256"]
    assert ledger["device"] == "cpu"
    calls = [json.loads(line) for line in (path / "calls.jsonl").read_text().splitlines()]
    by_action = {call["args"][0]: call["args"] for call in calls}
    port = by_action["up"][by_action["up"].index("--port") + 1]
    assert by_action["up"] == ["up", "--scene", str(path / "scene.py"), "--port", port, "--hold", "--no-watch", "--json"]
    assert by_action["ref"] == ["ref", "frames", str(path / "reference-2.mp4"), "--at", "0:start,6:end", "--width", "100", "--out", str(path / "ref"), "--json"]
    assert by_action["snapshot"] == ["snapshot", "--camera", "camera", "--size", "80x60", "--out", str(path / "snapshot-3.png"), "--json"]
    assert ["compare", "--camera", "camera", "--key", "block=red", "--size", "80x60", "--out", str(path / "comparison-4.png"), "--json"] in [call["args"] for call in calls]
    assert by_action["play"] == ["play", "push", "--record", "take1", "--every", "5", "--camera", "camera", "--json"]
    assert by_action["compare"][:5] == ["compare", "--take", "take1", "--key", "block=red"]
    assert by_action["compare"][5:] == ["--timeline", str(path / "ref/timeline-map-6.json"), "--json"]
    assert by_action["quit"] == ["quit", "--json"]
    for call in calls:
        assert call["cwd"] == str(path)
        assert call["env"]["CUDA_VISIBLE_DEVICES"] == ""
        assert call["env"]["ROBOSIMSTUDIO_DEVICE"] == "cpu"
        assert call["env"]["TMPDIR"] == str(bridge.root / ".tmp")
        assert call["env"]["XDG_CACHE_HOME"] == str(bridge.root / ".cache")
        assert call["env"]["WARP_CACHE_PATH"] == str(bridge.root / ".cache/warp")
    assert not gpu_calls
    assert sorted(str(file.relative_to(bridge.root)) for file in bridge.root.rglob("*") if file.is_file()) == [".venv/bin/python", "robosimstudio_real2sim_agent/__main__.py"]
    assert all("wall_s" in command and "finished_at" in command for command in ledger["commands"])
    script = str(Path(bridge_module.__file__).with_name("take_scores.py"))
    for command in ledger["commands"]:
        argv = command["argv"]
        assert argv[:3] == [str(bridge.root / ".venv/bin/python"), "-m", "robosimstudio_real2sim_agent"] or (
            argv[:3] == [str(bridge.root / ".venv/bin/python"), "-I", script] and Path(argv[3]).parent == path), argv


def test_async_poll_and_busy(fake):
    bridge = fake[0]
    run = start(fake)["run"]
    started = time.monotonic()
    result = bridge.call("rollout", run=run, take="async", wait_s=0)
    assert time.monotonic() - started < 1
    assert result["operation"]["state"] == "running"
    assert "active operation" in bridge.call("snapshot", run=run)["error"]
    result = bridge.call("status", run=run, wait_s=10)
    assert result["operation"]["state"] == "completed"
    assert result["operation"]["wall_s"] >= 0.2


def test_reload_ledger_after_acquiring_lock(fake, monkeypatch):
    bridge = fake[0]
    run = start(fake)["run"]
    original = bridge._load
    injected = False

    def load(name):
        nonlocal injected
        path, ledger = original(name)
        if not injected:
            injected = True
            advanced = dict(ledger, concurrent_marker="preserve")
            bridge._save(path, advanced)
        return path, ledger

    monkeypatch.setattr(bridge, "_load", load)
    complete(bridge, "report", run)
    assert original(run)[1]["concurrent_marker"] == "preserve"


def test_no_stopping_unowned_process(fake):
    bridge = fake[0]
    run = start(fake)["run"]
    path, ledger = bridge._load(run)
    original = ledger["server"]
    ledger["server"] = {"pid": os.getpid(), "start_ticks": "not-ours", "cmdline": []}
    bridge._save(path, ledger)
    assert not complete(bridge, "stop", run)["stopped"]
    assert "quit" not in [json.loads(line)["args"][0] for line in (path / "calls.jsonl").read_text().splitlines()]
    _, ledger = bridge._load(run)
    ledger["server"] = original
    bridge._save(path, ledger)


@pytest.mark.parametrize("kwargs", [{"run": "../escape"}, {"wait_s": 121}, {"wait_s": float("nan")}])
def test_invalid_inputs(fake, kwargs):
    assert not fake[0].call("status", **kwargs)["ok"]


def test_registration_one_group(fake):
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("test")
    register_robosim_tools(server, bridge=fake[0])
    tools = asyncio.run(server.list_tools())
    assert [tool.name for tool in tools] == ["robosim"]
    tool = tools[0]
    assert len(tool.description) < 400
    assert all(len(prop.get("description", "")) < 120 for prop in tool.inputSchema["properties"].values())
    assert tool.annotations.destructiveHint and not tool.annotations.readOnlyHint
    assert "update_scene" in tool.inputSchema["properties"]["action"]["enum"]
    for argument in ("push", "duration_s", "steps", "record_fps", "start_s", "end_s", "sim_start_s", "sim_end_s", "shape",
                     "template", "all", "max_trials"):
        assert argument in tool.inputSchema["properties"]
    assert "tune_push" in tool.inputSchema["properties"]["action"]["enum"]
    assert "jenga" in tool.inputSchema["properties"]["shape"]["enum"]
    assert "every" not in tool.inputSchema["properties"]
    flat = " ".join(tool.description.split())
    assert "frozen until update_scene" in flat and "no mass or friction" in flat
    result = asyncio.run(server.call_tool("robosim", {"action": "scene_template"}))
    reply = json.loads(result[0][0].text)
    text = Path(reply["path"]).read_text()
    assert "OBJECT_MASS" in text and "TeeShape" in text and "class TeeShape" not in json.dumps(reply)
    assert '"runs": []' in str(asyncio.run(server.call_tool("robosim", {"action": "status", "all": True})))  # all=true reached the bridge


# ---- new behaviour: template, push, timing, mapping, reload, trimming, stop ----

import ast
import socket

from twinsmith.robosim import plan, template


def finish(bridge, run, result):
    if (result.get("operation") or {}).get("state") == "running":
        result = bridge.call("status", run=run, wait_s=10)
    return result["operation"]["result"], result


def test_scene_template_writes_a_file_and_returns_its_path_not_the_source(fake):
    bridge = fake[0]
    tee = bridge.call("scene_template")
    assert tee["ok"] and tee["shape"] == "tee" and "scene" not in tee
    path = Path(tee["path"])
    assert path.is_file() and path.parent == bridge.runs / "_templates"
    text = path.read_text()
    assert 'SHAPE = "tee"' in text
    ast.parse(text)
    for needle in ("class TeeShape", "def push_routine", "def settle_routine", "routines = {", "PUSH = {", "coord_start", "dof_start",
                   "joint_q", "OBJECT_NAME"):
        assert needle in text
    assert "cube" not in text
    assert len(json.dumps(tee)) < 4000 and "class TeeShape" not in json.dumps(tee)
    assert set(tee["parameters"]) >= {"shape", "size", "position", "yaw", "color", "camera", "push"}
    assert tee["template"]["shape"] == "tee" and tee["mask_key"] == "block=red" and "+x is right and +y is up" in tee["frame"]
    assert "[bar_length, stem_length, thickness, height]" in tee["size_meaning"]
    assert "MuJoCo-Warp" in tee["note"] and "no mass or friction" in tee["note"] and "template={...}" in tee["use"]
    assert bridge.call("scene_template")["path"] == tee["path"]  # same content, same file
    box = bridge.call("scene_template", shape="box")
    assert 'SHAPE = "box"' in Path(box["path"]).read_text() and box["path"] != tee["path"]
    assert not bridge.call("scene_template", shape="sphere")["ok"]
    assert not bridge.call("scene_template", template={"colour": "red"})["ok"]
    with pytest.raises(ValueError):
        template.scene_text("cone")
    first = start(fake)
    inside = bridge.call("scene_template", run=first["run"], template={"shape": "tee", "yaw": 0.5})
    assert Path(inside["path"]).parent == Path(first["run_dir"]) and "OBJECT_XY_YAW = (0.0, 0.0, 0.5)" in Path(inside["path"]).read_text()


def test_jenga_template_is_a_flat_wooden_block_with_a_stated_mask_key(fake):
    bridge = fake[0]
    reply = bridge.call("scene_template", shape="jenga")
    text = Path(reply["path"]).read_text()
    assert 'SHAPE = "jenga"' in text and "BOX_HALF = (0.036, 0.012, 0.007)" in text
    assert "OBJECT_COLOUR = (0.76, 0.6, 0.42)" in text and "OBJECT_MASS = 0.02" in text
    assert reply["mask_key"] == "block=wood" and "72 x 24 x 14 mm" in reply["size_meaning"]
    assert reply["template"]["size"] == [0.072, 0.024, 0.014] and reply["template"]["color"] == "wood"
    assert 'elif SHAPE in ("box", "jenga")' in text and "PUSHER_RADIUS" in text
    ast.parse(text)
    assert "wood" in template.COLOURS and template.mask_key("wood") == "block=wood"
    assert template.mask_key("#c2996b") == "block=#c2996b~0.1" and template.mask_key([1, 0, 0], "thing") == "thing=#ff0000~0.1"


def test_template_table_is_above_the_ground_plane_so_it_does_not_speckle():
    text = template.scene_text("tee")
    top = float(next(line for line in text.splitlines() if line.startswith("TABLE_TOP =")).split("=")[1].split("#")[0])
    assert 0.001 <= top <= 0.01
    assert "at=(0.0, 0.0, TABLE_TOP - 0.02)" in text and "half_height + TABLE_TOP" in text and "PUSHER_HALF_HEIGHT + TABLE_TOP" in text
    assert "TABLE_COLOUR = (0.9, 0.9, 0.9)" in text and "ground plane" in text


def test_template_resolution_defaults_merging_and_errors():
    tee = template.resolve({})
    assert tee["shape"] == "tee" and tee["size"] == [0.13, 0.105, 0.035, 0.04] and tee["position"] == [0.0, 0.0]
    assert tee["push"] == {"waypoints": [[0.0, 0.0, -0.2], [2.0, 0.0, 0.0]], "object_velocity": None}
    constants = template.constants(tee)
    assert constants["TEE_BAR_HALF"] == (0.065, 0.0175) and constants["TEE_STEM_HALF"] == (0.0175, 0.0525)
    assert constants["OBJECT_HALF_HEIGHT"] == 0.02 and constants["OBJECT_XY_YAW"] == (0.0, 0.0, 0.0)
    moved = template.resolve({"position": [0.01, -0.07], "yaw": -0.25}, tee)
    assert moved["size"] == tee["size"] and moved["push"]["waypoints"][1][1:] == [0.01, -0.07]  # a default push follows the object
    pushed = template.resolve({"push": {"waypoints": [[0, 0, -0.2], [3, 0, 0.1]]}}, moved)
    assert pushed["push"]["waypoints"][1] == [3.0, 0.0, 0.1] and not pushed.get("push_default")
    again = template.resolve({"position": [0.05, 0.0]}, pushed)
    assert again["push"] == pushed["push"]  # an explicit push is kept
    other = template.resolve({"shape": "box"}, pushed)
    assert other["shape"] == "box" and other["size"] == [0.13, 0.14, 0.04] and other["push"] != pushed["push"]
    camera = template.resolve({"camera": {"eye": [0, 0, 0.5]}}, tee)["camera"]
    assert camera == {"eye": [0.0, 0.0, 0.5], "target": [-0.05, 0.12, 0.0]}
    for bad in ({"size": [0.1, 0.1]}, {"size": [0.1, 0.1, 0.1, 9.0]}, {"position": [1.0, 0.0]}, {"yaw": float("nan")}, {"color": "plaid"},
                {"mass": 0}, {"shape": "cone"}, {"nope": 1}, {"camera": {"eye": [0, 0, -1]}}, {"push": {"waypoints": [[0, 0, 0]]}},
                {"push": {"object_velocity": [1, 0, 0]}}, "tee"):
        with pytest.raises(ValueError):
            template.resolve(bad)


def test_start_and_update_scene_accept_plain_data_and_write_the_scene_file(fake, tmp_path):
    bridge, scene = fake[0], fake[1]
    first = bridge.call("start", name="data", device="cpu", template={"shape": "tee", "position": [0.01, -0.07], "yaw": -0.25,
                                                                       "size": [0.136, 0.11, 0.0368, 0.04]}, wait_s=10)
    assert first["ok"], first
    run, path = first["run"], Path(first["run_dir"])
    text = (path / "scene.py").read_text()
    assert text == (path / "scene-v1.py").read_text() and "OBJECT_XY_YAW = (0.01, -0.07, -0.25)" in text
    assert "TEE_BAR_HALF = (0.068, 0.0184)" in text and "TEE_STEM_HALF = (0.0184, 0.055)" in text
    ledger = json.loads((path / "run.json").read_text())
    assert ledger["scene_versions"][0]["source"] == "template" and ledger["scene_versions"][0]["template"]["yaw"] == -0.25
    assert ledger["inputs"]["scene"]["sha256"] == hashlib.sha256(text.encode()).hexdigest()
    updated = complete(bridge, "update_scene", run, template={"yaw": -0.3, "color": "blue"})
    assert updated["ok"] and updated["version"] == 2 and updated["template"]["yaw"] == -0.3
    assert updated["template"]["size"] == [0.136, 0.11, 0.0368, 0.04] and updated["template"]["position"] == [0.01, -0.07]  # merged
    assert updated["mask_key"] == "block=blue"
    assert "OBJECT_XY_YAW = (0.01, -0.07, -0.3)" in (path / "scene.py").read_text()
    assert (path / "scene-v1.py").read_text() == text
    # a file path still works, and is exclusive with template
    assert complete(bridge, "update_scene", run, scene=str(scene))["version"] == 3
    both = bridge.call("update_scene", run=run, scene=str(scene), template={})
    assert not both["ok"] and "not both" in both["error"]
    assert "template={...}" in bridge.call("update_scene", run=run)["error"]
    assert not bridge.call("start", name="x", device="cpu")["ok"] and not bridge.call("start", name="x", device="cpu", template={"bad": 1})["ok"]
    bad = bridge.call("update_scene", run=run, template={"yaw": "x"})
    assert not bad["ok"] and json.loads((path / "run.json").read_text())["scene_versions"][-1]["version"] == 3
    # after a scene from a file, the next template starts from the defaults
    assert complete(bridge, "update_scene", run, template={"yaw": 1.0})["template"]["size"] == [0.13, 0.105, 0.035, 0.04]


def test_status_lists_live_servers_and_the_five_newest_runs_one_line_each(fake):
    bridge = fake[0]
    runs = []
    for index in range(7):
        result = bridge.call("start", name=f"r{index}", device="cpu", template={}, wait_s=10)
        runs.append(result["run"])
    for run in runs[:5]:
        assert complete(bridge, "stop", run)["stopped"]
    listing = bridge.call("status")
    assert listing["ok"] and listing["total_runs"] == 7 and "runs" not in listing
    assert sorted(line["run"] for line in listing["live_servers"]) == sorted(runs[5:])
    assert [line["run"] for line in listing["recent_runs"]] == list(reversed(sorted(runs)))[:5]
    line = listing["recent_runs"][0]
    assert set(line) == {"run", "state", "device", "engine", "age", "line"} and line["state"] == "live"
    assert line["device"] == "cpu" and line["engine"] == "MuJoCo (CPU)" and line["age"].endswith("s")
    assert line["line"] == f"{line['run']} | live | cpu | MuJoCo (CPU) | {line['age']} old"
    assert listing["recent_runs"][-1]["state"] == "stopped" and "all=true" in listing["listing"]
    assert len(json.dumps(listing)) < 3500
    everything = bridge.call("status", show_all=True)
    assert len(everything["runs"]) == 7 and everything["runs"][0]["engine"]["physics"].startswith("plain MuJoCo")
    detail = bridge.call("status", run=runs[0])
    assert detail["run"] == runs[0] and detail["ledger"].endswith("run.json") and detail["server_alive"] is False


def test_reference_trims_the_clip_with_ffmpeg_and_keeps_original_seconds(fake):
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    result = complete(bridge, "reference", run, video=str(fake[2]), start_s=1, end_s=5)
    cut = [call for call in bridge.ffmpeg_calls if "-t" in call]
    assert len(cut) == 1 and cut[0][cut[0].index("-ss") + 1] == "1.000" and cut[0][cut[0].index("-t") + 1] == "4.100"
    assert cut[0][cut[0].index("-i") + 1] == str(path / "reference-2.mp4") and cut[0][-1] == str(path / "reference-2-trim.mp4")
    assert result["trimmed"]["duration_s"] == 4.0 and result["trimmed"]["file_duration_s"] == 4.1  # one extra frame holds the end_s frame
    assert "trimmed with ffmpeg" in result["trimmed"]["note"]
    assert result["duration"] == 4.0 and result["source_duration_s"] == 12.0 and result["window"] == {"start_s": 1, "end_s": 5}
    assert "contact sheet" in result["window_note"] and "trimmed" in result["window_note"]
    assert [a["t"] for a in result["anchors"]] == [1.0, 3.0, 5.0]  # original clip seconds
    ref_call = [json.loads(line)["args"] for line in (path / "calls.jsonl").read_text().splitlines() if '"ref"' in line][0]
    assert ref_call[2] == str(path / "reference-2-trim.mp4") and ref_call[4] == "0:start,2:middle,4:end"
    ledger = json.loads((path / "run.json").read_text())
    assert ledger["reference"]["offset_s"] == 1 and ledger["reference"]["duration_s"] == 4.0 and ledger["inputs"]["reference-2-trim"]["sha256"]
    # compare_take keeps original seconds: a 4 s take against the 1-5 s window
    complete(bridge, "rollout", run, take="t4", duration_s=4, push={"waypoints": [[0, 0, -0.2], [4, 0, 0]]})
    compared = complete(bridge, "compare_take", run, take="t4", keys=["block=red"])
    mapping = compared["time_mapping"]
    assert (mapping["clip_start_s"], mapping["clip_end_s"], mapping["sim_start_s"], mapping["sim_end_s"]) == (1, 5, 0, 4)
    assert [(a["clip_t"], a["sim_t_requested"]) for a in mapping["anchors"]] == [(1.0, 0.0), (3.0, 2.0), (5.0, 4.0)]
    assert [a["t"] for a in json.loads(Path(mapping["timeline"]).read_text())["anchors"]] == [1.0, 3.0, 5.0]
    # anchors given in original seconds are shifted for the cut clip; times outside the span are refused
    explicit = complete(bridge, "reference", run, video=str(fake[2]), start_s=2, end_s=8, at="2:a,5:b,8:c")
    assert [a["t"] for a in explicit["anchors"]] == [2.0, 5.0, 8.0] and explicit["duration"] == 6.0
    outside = bridge.call("reference", run=run, video=str(fake[2]), start_s=2, end_s=8, at="1:a,5:b", wait_s=10)
    assert "outside the trimmed span 2-8.1 s" in outside["operation"]["result"]["error"]
    # nothing to cut: the whole clip is used as it is
    before = len(bridge.ffmpeg_calls)
    whole = complete(bridge, "reference", run, video=str(fake[2]), start_s=0)
    assert whole["window"] == {"start_s": 0, "end_s": 12.0} and "trimmed" not in whole and len(bridge.ffmpeg_calls) == before + 1  # only the probe


def test_reference_window_without_ffmpeg_says_how_to_proceed(fake, monkeypatch):
    bridge = fake[0]
    run = start(fake)["run"]
    monkeypatch.setattr(shutil, "which", lambda *args, **kwargs: None)
    monkeypatch.setattr(bridge_module, "_bundled_ffmpeg", lambda: None)
    refused = bridge.call("reference", run=run, video=str(fake[2]), start_s=1, end_s=5, wait_s=10)["operation"]["result"]
    assert not refused["ok"] and "ffmpeg" in refused["error"] and "pass `at`" in refused["error"]
    assert complete(bridge, "reference", run, video=str(fake[2]), at="0:a,6:b")["ok"]  # `at` alone needs no ffmpeg


def test_ffmpeg_falls_back_to_the_bundled_binary_when_none_is_on_path(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda *args, **kwargs: None)
    monkeypatch.setattr(bridge_module, "_bundled_ffmpeg", lambda: "/bundled/ffmpeg")
    assert bridge_module._ffmpeg_exe() == "/bundled/ffmpeg"
    monkeypatch.setattr(shutil, "which", lambda *args, **kwargs: "/usr/bin/ffmpeg")
    assert bridge_module._ffmpeg_exe() == "/usr/bin/ffmpeg"


def test_status_says_no_bridge_and_names_the_engine(fake):
    bridge = fake[0]
    listing = bridge.call("status")
    assert listing["ok"] and listing["bridge_needed"] is False and "needs no Isaac Sim" in listing["note"]
    run = start(fake)["run"]
    status = bridge.call("status", run=run)
    assert status["bridge_needed"] is False and status["engine"]["device"] == "cpu"
    assert "plain MuJoCo" in status["engine"]["physics"] and "does not fit mass or friction" in status["engine"]["scope"]
    assert "MuJoCo-Warp" in plan.engine("cuda:0")["physics"]


def test_start_picks_a_free_port_and_reports_it(fake):
    bridge = fake[0]
    held = socket.socket()
    held.bind(("", 0))
    busy = held.getsockname()[1]
    try:
        assert bridge._free_port() != busy or not 8080 <= busy < 8280
        result = start(fake)
        operation = result["operation"]["result"]
        assert operation["port_requested"] is None and 1024 <= operation["port_chosen"] <= 65535
        assert operation["url"].endswith(str(operation["port_chosen"]))
        assert "all network interfaces" in operation["network"] and "no localhost-only option" in operation["network"]
        assert "frozen" in operation["scene_frozen"] and "update_scene" in operation["scene_frozen"]
    finally:
        held.close()


def test_free_port_skips_a_busy_port(fake, monkeypatch):
    sockets = []
    try:
        first = fake[0]._free_port()
        blocker = socket.socket()
        blocker.bind(("", first))
        sockets.append(blocker)
        assert fake[0]._free_port() != first
    finally:
        for blocker in sockets:
            blocker.close()


def test_explicit_port_is_used(fake):
    bridge, scene, _, _ = fake
    result = bridge.call("start", name="test", scene=str(scene), device="cpu", port=9123, wait_s=10)
    assert result["operation"]["result"]["port_chosen"] == 9123 and result["operation"]["result"]["port_requested"] == 9123
    assert not bridge.call("start", name="x", scene=str(scene), device="cpu", port=80)["ok"]


def test_update_scene_keeps_versions_and_reloads_in_the_same_server(fake, tmp_path):
    bridge, scene, _, _ = fake
    first = start(fake)
    run, pid = first["run"], first["server"]["pid"]
    path = Path(first["run_dir"])
    new = tmp_path / "second.py"
    new.write_text("scene = 'second'\n")
    result, status = finish(bridge, run, bridge.call("update_scene", run=run, scene=str(new), wait_s=10))
    assert result["ok"] and result["version"] == 2 and result["restarted"] is False and result["reloaded"] is True
    assert result["generation"] == {"before": 1, "after": 2} and result["routines"] == ["push", "settle"]
    assert "same process, not restarted" in result["what_happened"] and "older versions are kept" in result["what_happened"]
    assert (path / "scene-v1.py").read_text() == "scene = 'fake'\n"
    assert (path / "scene-v2.py").read_text() == "scene = 'second'\n" == (path / "scene.py").read_text()
    assert status["server"]["pid"] == pid and status["server_alive"]
    ledger = json.loads((path / "run.json").read_text())
    assert [v["version"] for v in ledger["scene_versions"]] == [1, 2] and ledger["scene_versions"][1]["loaded"] is True
    assert ledger["inputs"]["scene"]["sha256"] == hashlib.sha256(new.read_bytes()).hexdigest()
    third = tmp_path / "third.py"
    third.write_text("scene = 'third'\n")
    assert finish(bridge, run, bridge.call("update_scene", run=run, scene=str(third), wait_s=10))[0]["version"] == 3
    assert (path / "scene-v2.py").read_text() == "scene = 'second'\n"
    assert "Scene versions" in Path(complete(bridge, "report", run)["report"]).read_text()


def test_update_scene_failure_keeps_the_old_scene(fake, tmp_path):
    bridge, scene, _, _ = fake
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    bad = tmp_path / "bad.py"
    bad.write_text("scene = 'BREAK'\n")
    result, status = finish(bridge, run, bridge.call("update_scene", run=run, scene=str(bad), wait_s=10))
    assert not result["ok"] and result["loaded"] is False and "build failed" in result["what_happened"]
    assert "Traceback" in result["error"]
    assert (path / "scene.py").read_text() == "scene = 'fake'\n"
    assert (path / "scene-v2.py").read_text() == "scene = 'BREAK'\n"
    ledger = json.loads((path / "run.json").read_text())
    assert ledger["scene_versions"][1]["loaded"] is False and ledger["inputs"]["scene"]["sha256"] == ledger["scene_versions"][0]["sha256"]
    good = tmp_path / "good.py"
    good.write_text("scene = 'good'\n")
    assert finish(bridge, run, bridge.call("update_scene", run=run, scene=str(good), wait_s=10))[0]["ok"]


def test_update_scene_validates_before_touching_the_run(fake, tmp_path):
    bridge = fake[0]
    run = start(fake)["run"]
    assert "absolute .py" in bridge.call("update_scene", run=run)["error"]
    assert "absolute .py" in bridge.call("update_scene", run=run, scene="relative.py")["error"]
    assert "not alive" not in bridge.call("update_scene", run=run, scene=str(tmp_path / "missing.py"))["error"]
    complete(bridge, "stop", run)
    scene = tmp_path / "again.py"
    scene.write_text("scene = 1\n")
    assert "not alive" in bridge.call("update_scene", run=run, scene=str(scene))["error"]


@pytest.mark.parametrize("line", ["sim.state.body_q.assign(q)", "state.body_qd .assign( qd )"])
def test_scene_writing_body_q_is_an_error_naming_the_fix(fake, tmp_path, line):
    bridge, _, _, _ = fake
    bad = tmp_path / "bad_write.py"
    bad.write_text(f"def routine(sim):\n    {line}\n    yield\n")
    result = bridge.call("start", name="test", scene=str(bad), device="cpu")
    assert not result["ok"] and "discarded" in result["error"] and "joint_q" in result["error"] and "push" in result["error"]
    assert not bridge.runs.exists()
    run = start(fake)["run"]
    result = bridge.call("update_scene", run=run, scene=str(bad))
    assert not result["ok"] and "joint_qd" in result["error"]
    ok = tmp_path / "reads_only.py"
    ok.write_text("q = sim.state.body_q.numpy()\n")
    assert finish(bridge, run, bridge.call("update_scene", run=run, scene=str(ok), wait_s=10))[0]["ok"]


def test_rollout_records_what_was_asked_and_returns_the_numbers(fake):
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    push = {"waypoints": [[0, 0.1, -0.2], [3, 0.0, -0.05], [6, -0.1, 0.05]], "object_velocity": [0.0, 0.1, 0.0]}
    result = complete(bridge, "rollout", run, take="six", routine="push", push=push, duration_s=6, record_fps=10)
    assert (result["steps_asked"], result["steps"], result["duration_s"]) == (300, 300, 6.0)
    assert (result["control_hz"], result["every"], result["record_fps"], result["video_fps"]) == (50.0, 5, 10.0, 10)
    assert result["frames"] == result["frames_expected"] == 61
    assert result["push"]["waypoints"][1] == [3.0, 0.0, -0.05] and result["push"]["object_velocity"] == [0.0, 0.1, 0.0]
    assert result["push_applied"]["applied"] and result["engine"]["device"] == "cpu" and not result["warnings"]
    assert "board" not in result["status"] and "compare_take" in result["board_note"]
    calls = [json.loads(line)["args"] for line in (path / "calls.jsonl").read_text().splitlines()]
    kinds = [call[0] for call in calls]
    assert kinds.index("eval") < kinds.index("reset") < kinds.index("play")
    rollout_call = [call for call in calls if call[0] == "eval" and call[1].startswith("ROLLOUT")][0]
    literal = ast.literal_eval(rollout_call[1].split("=", 1)[1].strip())
    assert literal == {"steps": 300, "take": "six", "push": result["push"]}
    # steps instead of duration, a different rate, and a duration taken from the push
    by_steps = complete(bridge, "rollout", run, take="steps", steps=120, record_fps=25)
    assert (by_steps["steps"], by_steps["every"], by_steps["record_fps"], by_steps["frames"]) == (120, 2, 25.0, 61)
    by_push = complete(bridge, "rollout", run, take="pushlen", push={"waypoints": [[0, 0, -0.2], [2.5, 0, 0]]})
    assert (by_push["steps"], by_push["duration_s"]) == (125, 2.5)


def test_rollout_warns_when_the_rate_does_not_divide(fake):
    result = complete(fake[0], "rollout", start(fake)["run"], take="odd", duration_s=1, record_fps=12)
    assert result["every"] == 4 and result["record_fps"] == 12.5 and result["record_fps_asked"] == 12.0
    assert "12.5 frames per simulated second" in result["warnings"][0]


def test_rollout_refuses_a_routine_that_ignored_push(fake, tmp_path):
    bridge = fake[0]
    scene = tmp_path / "ignores.py"
    scene.write_text("IGNORE_PUSH = True\n")
    run = bridge.call("start", name="test", scene=str(scene), device="cpu", wait_s=10)["run"]
    result = bridge.call("rollout", run=run, take="ignored", duration_s=1, push={"waypoints": [[0, 0, 0], [1, 0.1, 0]]}, wait_s=10)
    assert not result["ok"]
    failure = result["operation"]["result"]
    assert "did not apply push" in failure["error"] and "ROLLOUT['push']" in failure["error"]
    # without push the same scene is fine
    assert complete(bridge, "rollout", run, take="plain", duration_s=1)["ok"]


@pytest.mark.parametrize("kwargs, message", [
    ({"duration_s": 2, "steps": 100}, "not both"),
    ({"duration_s": -1}, "positive"),
    ({"steps": 0}, "positive integer"),
    ({"duration_s": 1, "record_fps": 500}, "at most the control rate"),
    ({"duration_s": 1000}, "limit of"),
    ({"push": {"waypoints": [[0, 0, 0]]}}, "2 to"),
    ({"push": {"waypoints": [[1, 0, 0], [1, 1, 1]]}}, "strictly increase"),
    ({"push": {"waypoints": [[0, 0, 0], [1, 9, 9]]}}, "within"),
    ({"push": {"speed": 1}}, "unknown keys"),
    ({"push": {}}, "needs waypoints"),
    ({"push": "fast"}, "must be an object"),
    ({"push": {"object_velocity": [1, 2]}}, "[vx, vy, yaw_rate]"),
])
def test_rollout_argument_errors(fake, kwargs, message):
    bridge = fake[0]
    run = start(fake)["run"]
    result = bridge.call("rollout", run=run, take="bad", wait_s=10, **kwargs)
    error = result.get("error") or (result.get("operation") or {}).get("result", {}).get("error") or str(result.get("operation"))
    assert message in error, error


def test_fast_pusher_is_warned(fake):
    result = complete(fake[0], "rollout", start(fake)["run"], take="fast", push={"waypoints": [[0, 0, -0.2], [0.1, 0, 0.2]]})
    assert "passes through" in result["warnings"][0] or "pass through" in result["warnings"][0]


def test_plan_arithmetic():
    six = plan.rollout_plan(50, duration_s=6, record_fps=10)
    assert (six["steps"], six["every"], six["record_fps"], six["frames_expected"], six["duration_s"]) == (300, 5, 10.0, 61, 6.0)
    sixty = plan.rollout_plan(60, duration_s=2, record_fps=10)
    assert (sixty["steps"], sixty["every"], sixty["frames_expected"]) == (120, 6, 21)
    assert plan.rollout_plan(50, steps=7, record_fps=10)["frames_expected"] == 3     # frames at 0 and 5, plus the last state
    assert plan.rollout_plan(50, record_fps=10)["steps"] == 100                      # nothing asked: 2 s
    assert plan.rollout_plan(50, push={"waypoints": [[0, 0, 0], [3, 1, 1]]})["steps"] == 150
    assert plan.rollout_plan(50, duration_s=0.01)["steps"] == 1
    with pytest.raises(ValueError):
        plan.rollout_plan(0, duration_s=1)
    with pytest.raises(ValueError):
        plan.rollout_plan(50, duration_s=1, record_fps=0)


def test_time_mapping_defaults_explicit_and_warnings():
    default = plan.time_mapping([0.0, 3.0, 6.0], 6.0)
    assert default["sim_t"] == {0.0: 0.0, 3.0: 3.0, 6.0: 6.0} and default["sim_seconds_per_clip_second"] == 1.0
    assert default["warnings"] == [] and (default["sim_start_s"], default["sim_end_s"]) == (0.0, 6.0)
    shifted = plan.time_mapping([0.0, 3.0, 6.0], 10.0, sim_start=2.0)
    assert shifted["sim_t"] == {0.0: 2.0, 3.0: 5.0, 6.0: 8.0}
    squeezed = plan.time_mapping([0.0, 3.0, 6.0], 6.0, sim_end=3.0)
    assert squeezed["sim_t"] == {0.0: 0.0, 3.0: 1.5, 6.0: 3.0}
    assert "6 s" in squeezed["warnings"][0] and "3 s" in squeezed["warnings"][0]
    within = plan.time_mapping([0.0, 3.0, 6.0], 6.6, clip_start=0, clip_end=6, sim_start=0, sim_end=6.5)
    assert within["warnings"] == []                              # 8 percent apart: below the 10 percent line
    over = plan.time_mapping([0.0, 3.0, 6.0], 7.0, clip_start=0, clip_end=6, sim_start=0, sim_end=6.7)
    assert len(over["warnings"]) == 1                            # 11.7 percent apart
    windowed = plan.time_mapping([0.0, 3.0, 6.0, 9.0], 6.0, clip_start=3.0, clip_end=9.0)
    assert windowed["sim_t"] == {3.0: 0.0, 6.0: 3.0, 9.0: 6.0} and windowed["anchors_outside_window"] == [0.0]
    for kwargs in ({"sim_end": 7.0}, {"clip_end": 0.0}, {"sim_start": 5.0, "sim_end": 4.0}, {"clip_start": 7.0, "clip_end": 9.0}):
        with pytest.raises(ValueError):
            plan.time_mapping([0.0, 3.0, 6.0], 6.0, **kwargs)
    with pytest.raises(ValueError):
        plan.time_mapping([], 6.0)


def test_compare_take_uses_and_reports_the_explicit_mapping(fake):
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    complete(bridge, "rollout", run, take="t6", duration_s=6, push={"waypoints": [[0, 0, -0.2], [6, 0, 0]]})
    result = complete(bridge, "compare_take", run, take="t6", keys=["block=red"], start_s=0, end_s=6, sim_start_s=1, sim_end_s=4)
    mapping = result["time_mapping"]
    assert (mapping["clip_start_s"], mapping["clip_end_s"], mapping["sim_start_s"], mapping["sim_end_s"]) == (0, 6, 1, 4)
    assert mapping["sim_seconds_per_clip_second"] == 0.5 and mapping["take_duration_s"] == 6.0
    assert [(a["clip_t"], a["sim_t_requested"], a["sim_t_used"]) for a in mapping["anchors"]] == [(0.0, 1.0, 1.0), (3.0, 2.5, 2.5), (6.0, 4.0, 4.0)]
    assert len(result["warnings"]) == 1 and "0.5 simulated s per clip s" in result["warnings"][0]
    mapped = json.loads(Path(mapping["timeline"]).read_text())
    assert [a["sim_t"] for a in mapped["anchors"]] == [1.0, 2.5, 4.0]
    # default mapping: same duration, from zero, no warning
    plain = complete(bridge, "compare_take", run, take="t6", keys=["block=red"])
    assert plain["warnings"] == [] and plain["time_mapping"]["sim_end_s"] == 6.0
    # a window past the take is refused with the take's length
    bad = bridge.call("compare_take", run=run, take="t6", keys=["block=red"], sim_end_s=9, wait_s=10)
    assert "lasts 6 s" in bad["operation"]["result"]["error"]
    missing = bridge.call("compare_take", run=run, take="nope", keys=["block=red"], wait_s=10)
    assert "No readable take" in missing["operation"]["result"]["error"]


def test_compare_take_reports_centroid_displacement_and_area_ratio(fake):
    bridge = fake[0]
    run = start(fake)["run"]
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    complete(bridge, "rollout", run, take="d", duration_s=6)
    result = complete(bridge, "compare_take", run, take="d", keys=["block=red"])
    moved = result["displacement_px"]["block"]
    assert moved["clip_px"] == [-50.0, -60.0] and moved["sim_px"] == [-40.0, -40.0]
    assert moved["difference_px"] == [10.0, 20.0] and moved["same_sign_x"] and moved["same_sign_y"]
    assert moved["magnitude_ratio"] == round(math.hypot(40, 40) / math.hypot(50, 60), 3)
    first = result["anchors"][0]["objects"]["block"]
    assert first["area_ratio"] == 1.1 and first["linear_scale_ratio"] == 1.049 and "size_ratio" not in first
    assert first["width_ratio"] == 1.1 and first["height_ratio"] == 0.9
    assert "cube" not in json.dumps(result) and "does not fit mass or friction" in result["scope"]
    single = complete(bridge, "compare", run, keys=["block=red"])
    assert single["objects"]["block"]["area_ratio"] == 1.1 and "size_ratio" not in single["objects"]["block"]
    assert "square root of the area ratio" in single["ratio_note"]


def test_plan_ratios_displacement_and_validation_helpers():
    reading = {"photo": {"pixels": 1865, "centre": [1, 1]}, "sim": {"pixels": 3624, "centre": [2, 2]}, "size_ratio": 1.394,
               "width_ratio": 0.967, "height_ratio": 0.913, "iou": 0.3}
    out = plan.relabel_ratios(reading)
    assert out["area_ratio"] == round(3624 / 1865, 3) == 1.943 and out["linear_scale_ratio"] == 1.394 and "size_ratio" not in out
    assert plan.relabel_ratios({"photo": None, "sim": None, "size_ratio": None})["area_ratio"] is None
    assert plan.relabel_ratios(None) is None
    first = {"objects": {"b": {"photo": {"centre": [10, 10]}, "sim": {"centre": [5, 5]}}}}
    last = {"objects": {"b": {"photo": {"centre": [0, 20]}, "sim": {"centre": [5, 5]}}}}
    still = plan.centroid_displacement(first, last, "b")
    assert still["clip_px"] == [-10, 10] and still["sim_px"] == [0, 0] and still["same_sign_x"] is False and still["magnitude_ratio"] == 0.0
    assert plan.centroid_displacement(first, last, "missing")["clip_px"] is None
    clean, notes = plan.validate_push({"waypoints": [[0, 0, 0], [10, 0.1, 0]]})
    assert clean == {"waypoints": [[0.0, 0.0, 0.0], [10.0, 0.1, 0.0]], "object_velocity": None} and notes == []
    only_velocity, notes = plan.validate_push({"object_velocity": [0.5, 0, 0]})
    assert only_velocity["object_velocity"] == [0.5, 0.0, 0.0] and "stays off the table" in notes[0]
    for bad in ({"waypoints": [[0, 0, float("nan")], [1, 0, 0]]}, {"waypoints": [[0, True, 0], [1, 0, 0]]}):
        with pytest.raises(ValueError):
            plan.validate_push(bad)
    assert plan.scene_problems("q = sim.state.body_q.numpy()\nq[1][0] = 1\n") == []
    assert len(plan.scene_problems("sim.state.body_qd.assign(x)")) == 1


def test_reference_window(fake):
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    result = complete(bridge, "reference", run, video=str(fake[2]), start_s=1, end_s=5)
    assert result["window"] == {"start_s": 1, "end_s": 5} and "contact sheet" in result["window_note"]
    assert [a["t"] for a in result["anchors"]] == [1.0, 3.0, 5.0] and [a["label"] for a in result["anchors"]] == ["start", "middle", "end"]
    assert json.loads((path / "run.json").read_text())["reference"]["start_s"] == 1
    explicit = complete(bridge, "reference", run, video=str(fake[2]), start_s=2, at="2:a,4:b")
    assert explicit["window"] == {"start_s": 2, "end_s": 12.0} and [a["t"] for a in explicit["anchors"]] == [2.0, 4.0]
    only_start = complete(bridge, "reference", run, video=str(fake[2]), start_s=2)
    assert [a["t"] for a in only_start["anchors"]] == [2.0, 11.9] and only_start["duration"] == 10.0  # the clip's last frame
    only_end = complete(bridge, "reference", run, video=str(fake[2]), end_s=4)
    assert only_end["anchors"][0]["t"] == 0.0 and only_end["trimmed"]["source_end_s"] == 4 and only_end["duration"] == 4.0
    beyond = bridge.call("reference", run=run, video=str(fake[2]), start_s=1, end_s=20, wait_s=10)
    assert "beyond the clip's duration" in beyond["operation"]["result"]["error"]
    assert "greater than start_s" in bridge.call("reference", run=run, video=str(fake[2]), start_s=5, end_s=5)["error"]
    assert "zero or more" in bridge.call("reference", run=run, video=str(fake[2]), start_s=-1)["error"]


def test_stop_waits_a_bounded_time_and_reports_the_final_state(fake, monkeypatch):
    from twinsmith.robosim import bridge as bridge_module

    bridge = fake[0]
    first = start(fake)
    run, path, pid = first["run"], Path(first["run_dir"]), first["server"]["pid"]
    monkeypatch.setattr(bridge_module, "STOP_WAIT_S", 0.4)
    (path / "no-quit").write_text("")
    result = bridge.call("stop", run=run, wait_s=10)["operation"]["result"]
    assert not result["ok"] and result["stopped"] is False and "still alive after 0.4 s" in result["final_state"]
    assert "force-killed" in result["error"] and result["waited_s"] >= 0.4
    (path / "no-quit").unlink()
    monkeypatch.setattr(bridge_module, "STOP_WAIT_S", 15.0)
    done = complete(bridge, "stop", run)
    assert done["stopped"] and done["final_state"] == f"process {pid} has exited" and done["waited_s"] < 15
    again = complete(bridge, "stop", run)
    assert again["stopped"] is False and again["final_state"] == "not running"


def test_failed_scene_load_points_at_update_scene(fake, tmp_path):
    bridge = fake[0]
    scene = tmp_path / "broken.py"
    scene.write_text("scene = 'BREAK'\n")
    result = bridge.call("start", name="test", scene=str(scene), device="cpu", wait_s=10)
    assert not result["ok"] and "update_scene" in result["operation"]["result"]["error"]
    ledger = json.loads((Path(result["run_dir"]) / "run.json").read_text())
    assert ledger["scene_versions"][0]["loaded"] is False
    good = tmp_path / "fixed.py"
    good.write_text("scene = 'ok'\n")
    assert finish(bridge, result["run"], bridge.call("update_scene", run=result["run"], scene=str(good), wait_s=10))[0]["ok"]


def test_listed_description_and_arguments_stay_short(fake):
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("test")
    register_robosim_tools(server, bridge=fake[0])
    tool = asyncio.run(server.list_tools())[0]
    assert len(tool.description) < 400
    longest = max(len(prop.get("description", "")) for prop in tool.inputSchema["properties"].values())
    assert longest < 120, longest


# ---- the bridge's own verdict ----

def moved_entry(clip, sim):
    first = {"objects": {"b": {"photo": {"centre": [100.0, 100.0]}, "sim": {"centre": [100.0, 100.0]}}}}
    last = {"objects": {"b": {"photo": {"centre": [100.0 + clip[0], 100.0 + clip[1]]}, "sim": {"centre": [100.0 + sim[0], 100.0 + sim[1]]}}}}
    return plan.centroid_displacement(first, last, "b")


def test_verdict_thresholds_are_stated_and_applied():
    clip = (-53.8, -56.6)
    matches = plan.bridge_verdict(moved_entry(clip, (-65.4, -61.5)), 0.52)
    assert matches["verdict"] == "matches" and matches["failed"] == []
    assert matches["numbers"]["magnitude_ratio"] == 1.15 and matches["numbers"]["end_frame_overlap"] == 0.52
    low = plan.bridge_verdict(moved_entry(clip, (-11.6, -22.5)), 0.0)  # the first user test: 32 percent of the distance, overlap 0
    assert low["verdict"] == "moved, but not like the clip" and len(low["failed"]) == 2
    assert any("magnitude_ratio 0.32" in text and "0.5 to 2" in text for text in low["failed"])
    assert any("end_frame_overlap 0.0 is not above 0.2" in text for text in low["failed"])
    assert plan.bridge_verdict(moved_entry(clip, (-30.0, -30.0)), 0.3)["verdict"] == "matches"            # ratio 0.54
    assert plan.bridge_verdict(moved_entry(clip, (-27.0, -28.0)), 0.3)["verdict"] == "moved, but not like the clip"  # ratio 0.498
    assert plan.bridge_verdict(moved_entry(clip, (-100, -100)), 0.3)["verdict"] == "matches"               # ratio 1.81
    assert plan.bridge_verdict(moved_entry(clip, (-150, -150)), 0.3)["failed"][0].startswith("magnitude_ratio 2.7")
    assert plan.bridge_verdict(moved_entry(clip, (-65.4, -61.5)), 0.2)["verdict"] == "moved, but not like the clip"  # overlap must be above 0.2
    sideways = plan.bridge_verdict(moved_entry(clip, (60.0, -50.0)), 0.5)
    assert sideways["verdict"] == "moved, but not like the clip" and any("direction_error_deg" in text for text in sideways["failed"])
    still = plan.bridge_verdict(moved_entry(clip, (1.0, 2.0)), 0.5)
    assert still["verdict"] == "did not move" and "under 3 px" in still["failed"][0]
    assert plan.bridge_verdict(moved_entry((1, 1), (0, 0)), 0.5)["failed"][0].endswith("too)")
    assert plan.bridge_verdict(moved_entry(clip, (2.0, 2.0)), None)["verdict"] == "did not move"
    assert plan.bridge_verdict(plan.centroid_displacement({"objects": {}}, {"objects": {}}, "b"), None)["verdict"] == "did not move"
    thresholds = plan.VERDICT_THRESHOLDS
    assert thresholds["magnitude_ratio_range"] == [0.5, 2.0] and thresholds["direction_error_deg_below"] == 30
    assert thresholds["end_frame_overlap_above"] == 0.2 and thresholds["did_not_move_below_px"] == 3
    assert plan.overall_verdict({"a": matches, "b": low}) == "moved, but not like the clip"
    assert plan.overall_verdict({"a": matches, "b": still}) == "did not move" and plan.overall_verdict({}) is None
    assert plan.direction_error_deg((1, 0), (0, 1)) == 90.0 and plan.direction_error_deg((1, 0), (0, 0)) is None


def test_compare_take_reports_the_bridge_verdict_beside_rss_s(fake):
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    complete(bridge, "rollout", run, take="plain", duration_s=6)
    plain = complete(bridge, "compare_take", run, take="plain", keys=["block=red"])
    assert plain["verdict"] == "matches" and plain["rss_verdict"] == "1 of 1 did what the clip did" and plain["rss_score"] == 0.4
    assert "score" not in plain and "RSS's own" in plain["rss_note"]
    bridge_part = plain["bridge_verdict"]
    assert bridge_part["verdict"] == "matches" and bridge_part["thresholds"] == plan.VERDICT_THRESHOLDS and "not RSS's" in bridge_part["source"]
    numbers = bridge_part["by_object"]["block"]["numbers"]
    assert numbers["end_frame_overlap"] == 0.5 and numbers["sim_px"] == [-40.0, -40.0] and numbers["clip_px"] == [-50.0, -60.0]
    # RSS still says the block "did what the clip did" while the bridge says it barely moved
    (path / "physics.json").write_text("{}")
    complete(bridge, "rollout", run, take="still", duration_s=6)
    still = complete(bridge, "compare_take", run, take="still", keys=["block=red"])
    assert still["verdict"] == "did not move" and still["rss_verdict"] == "1 of 1 did what the clip did"
    assert "under 3 px" in still["bridge_verdict"]["by_object"]["block"]["failed"][0]
    complete(bridge, "rollout", run, take="short", duration_s=6, push={"waypoints": [[0, 0, -0.2], [6, 0, -0.17]]})
    short = complete(bridge, "compare_take", run, take="short", keys=["block=red"])
    assert short["verdict"] == "moved, but not like the clip" and any("magnitude_ratio" in text for text in short["bridge_verdict"]["by_object"]["block"]["failed"])
    text = Path(complete(bridge, "report", run)["report"]).read_text()
    assert "bridge verdict **did not move**" in text and "RSS's own score 0.4" in text and "magnitude ratio" in text


# ---- tune_push ----

def known_physics(clip=(-53.8, -56.6), *, gain=0.5, turn=0.3, pixels_per_m=480.0, floor=0.0):
    """A block whose image displacement is a known function of the push: gain times the pusher's travel, turned by
    ``turn`` radians, at ``pixels_per_m``; ``floor`` metres of travel does nothing (the pusher must first reach the block)."""
    def evaluate(push, trial):
        w = push["waypoints"]
        dx, dy = w[-1][1] - w[0][1], w[-1][2] - w[0][2]
        travel = max(0.0, math.hypot(dx, dy) - floor)
        angle = math.atan2(dy, dx) + turn
        length = gain * travel * pixels_per_m
        return {"clip_px": list(clip), "sim_px": [round(length * math.cos(angle), 1), round(-length * math.sin(angle), 1)],
                "end_overlap": 0.5}
    return evaluate


AXES = plan.image_axes(template.DEFAULT_CAMERA["eye"], template.DEFAULT_CAMERA["target"])
PUSH0 = {"waypoints": [[0.0, 0.0, -0.2], [6.0, 0.0, 0.0]], "object_velocity": None}


def test_image_axes_say_which_way_the_table_goes_in_the_picture():
    (rx, ry), (dx, dy) = AXES
    assert (rx, ry) == (1.0, -0.0) and dx == 0 and dy < 0  # +x is right; table +y is up in the image (image y points down)
    assert round(math.degrees(plan.table_angle((0, -10), AXES))) == 90 and round(math.degrees(plan.table_angle((10, 0), AXES))) == 0
    assert round(math.degrees(plan.table_angle((-53.8, -56.6), AXES))) == 133  # the real block moves toward -x, +y on the table
    with pytest.raises(ValueError):
        plan.image_axes((0, 0, 1), (0, 0, 0))


def test_transform_push_turns_about_the_object_and_stretches_from_the_pusher_start():
    pivot = (0.0, -0.075)
    assert plan.transform_push(PUSH0, pivot, 1.0, 0.0)["waypoints"] == PUSH0["waypoints"]
    double = plan.transform_push(PUSH0, pivot, 2.0, 0.0)
    assert double["waypoints"] == [[0.0, 0.0, -0.2], [6.0, 0.0, 0.2]]  # the start stays, the travel past the object grows
    turned = plan.transform_push(PUSH0, pivot, 1.0, math.pi / 2)
    assert turned["waypoints"][0][1:] == [0.125, -0.075] and turned["waypoints"][1][1:] == [-0.075, -0.075]
    both = plan.transform_push(PUSH0, pivot, 2.0, math.pi / 2)
    assert both["waypoints"][0][1:] == [0.125, -0.075] and both["waypoints"][1][1:] == [-0.275, -0.075]
    aimed = plan.aimed_push(PUSH0, pivot, math.pi / 2)
    assert aimed["waypoints"] == [[0.0, 0.0, -0.225], [6.0, 0.0, 0.025]]


def test_search_converges_on_a_known_function_and_stops_early():
    trials, best, why = plan.search_push(PUSH0, (0.0, -0.075), AXES, known_physics())
    assert why == "within tolerance" and best is trials[-1] and 1 < len(trials) <= 4
    assert trials[0]["scale"] == 1.0 and trials[0]["rotation_deg"] == 0.0 and trials[0]["push"] == PUSH0
    assert trials[0]["magnitude_ratio"] < 0.7 and trials[0]["direction_error_deg"] > 15
    assert 0.8 <= best["magnitude_ratio"] <= 1.25 and best["direction_error_deg"] < 15
    assert [t["trial"] for t in trials] == list(range(1, len(trials) + 1))
    assert best["rotation_deg"] != 0 and best["push"] != PUSH0 and all(len(t["push"]["waypoints"]) == 2 for t in trials)
    # the push that was kept does give a displacement within a factor of 1.5 on both axes
    sim = known_physics()(best["push"], 0)["sim_px"]
    assert sim[0] < 0 and sim[1] < 0 and 1 / 1.5 < sim[0] / -53.8 < 1.5 and 1 / 1.5 < sim[1] / -56.6 < 1.5
    # a push that already matches stops at the first trial
    first = plan.search_push(best["push"], (0.0, -0.075), AXES, known_physics())
    assert first[2] == "within tolerance" and len(first[0]) == 1


def test_search_is_bounded_and_keeps_the_best_trial():
    calls = []

    def stubborn(push, trial):
        calls.append(trial)
        return {"clip_px": [-53.8, -56.6], "sim_px": [20.0, 5.0 * trial], "end_overlap": 0.1}  # never close, always moves
    trials, best, why = plan.search_push(PUSH0, (0.0, -0.075), AXES, stubborn, max_trials=4)
    assert why == "max_trials" and len(trials) == 4 == len(calls) and best["trial"] in (1, 2, 3, 4)
    assert best["cost"] == min(t["cost"] for t in trials)
    only = plan.search_push(PUSH0, (0.0, -0.075), AXES, stubborn, max_trials=1)
    assert len(only[0]) == 1 and only[2] == "max_trials"
    for bad in (0, 13, True, 2.5):
        with pytest.raises(ValueError, match="max_trials"):
            plan.search_push(PUSH0, (0.0, 0.0), AXES, stubborn, max_trials=bad)


def test_search_aims_again_when_the_block_did_not_move():
    # the first push passes beside the block (travel under 0.3 m does nothing): the next trial aims through the object
    physics = known_physics(floor=0.05)

    def beside(push, trial):
        w = push["waypoints"]
        return {"clip_px": [-53.8, -56.6], "sim_px": [0.4, 0.2] if trial == 1 else physics(push, trial)["sim_px"], "end_overlap": 0.2}
    trials, best, why = plan.search_push(PUSH0, (0.0, -0.075), AXES, beside)
    assert trials[0]["magnitude_ratio"] < 0.02 and trials[0]["cost"] == 10.0
    assert trials[1]["scale"] == 1.0 and trials[1]["sim_px"][0] < 0 < 1  # re-aimed straight, unscaled, then refined
    assert why in ("within tolerance", "max_trials") and best["cost"] < 10


def test_search_refuses_a_clip_that_does_not_move_and_an_unknown_start():
    still = lambda push, trial: {"clip_px": [0.5, -0.5], "sim_px": [10.0, 10.0]}
    with pytest.raises(ValueError, match="no displacement to match"):
        plan.search_push(PUSH0, (0, 0), AXES, still)
    with pytest.raises(ValueError, match="start position is unknown"):
        plan.search_push(PUSH0, None, AXES, known_physics(), max_trials=3)
    revealed = lambda push, trial: dict(known_physics()(push, trial), pivot=(0.0, -0.075))
    assert plan.search_push(PUSH0, None, AXES, revealed)[2] == "within tolerance"  # a trial can tell where the object started


def prepare_tuning(fake, **start_arguments):
    bridge = fake[0]
    first = bridge.call("start", name="tune", device="cpu", template=start_arguments.get("template", {}), wait_s=10)
    run, path = first["run"], Path(first["run_dir"])
    (path / "physics.json").write_text("{}")
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    return bridge, run, path


def test_tune_push_runs_bounded_rollouts_and_returns_the_trial_table(fake):
    bridge, run, path = prepare_tuning(fake)
    result = complete(bridge, "tune_push", run, keys=["block=red"], take="tn", record_fps=10)
    assert result["ok"] and result["within_tolerance"] and result["stopped_because"] == "within tolerance"
    trials = result["trials"]
    assert 1 <= len(trials) <= 6 and [t["take"] for t in trials] == [f"tn-{k}" for k in range(1, len(trials) + 1)] == result["takes"]
    assert trials[0]["scale"] == 1.0 and trials[0]["push"]["waypoints"] == [[0.0, 0.0, -0.2], [2.0, 0.0, 0.0]]  # the template's own push
    for row in trials:
        assert set(row) >= {"trial", "scale", "rotation_deg", "push", "sim_px", "clip_px", "magnitude_ratio", "direction_error_deg", "end_overlap", "take"}
        assert (path / f"{row['take']}.mp4").is_file()
    best = result["best_push"]
    assert best == {"waypoints": trials[result["best_trial"] - 1]["push"]["waypoints"], "object_velocity": None}
    assert plan.validate_push(best)[0]["waypoints"] == best["waypoints"]
    assert 0.8 <= result["best_magnitude_ratio"] <= 1.25 and result["best_direction_error_deg"] < 15
    assert result["tolerance"] == {"magnitude_ratio": [0.8, 1.25], "direction_error_deg_below": 15.0}
    assert "PUSH PATH" in result["scope"] and "does not change mass, friction" in result["scope"]
    assert "best_push" in result["use"]["rollout"] and "template={'push': best_push}" in result["use"]["update_scene"]
    assert result["duration_s"] == 6.0
    # every trial rolled out 6 s at the clip rate, and the best push drives the next ordinary rollout and the scene update
    final = complete(bridge, "rollout", run, take="final", push=best, duration_s=6, record_fps=10)
    assert final["ok"] and final["push"]["waypoints"] == best["waypoints"]
    again = complete(bridge, "update_scene", run, template={"push": best})
    assert again["template"]["push"]["waypoints"] == best["waypoints"]
    assert "tune_push" in Path(complete(bridge, "report", run)["report"]).read_text()


def test_tune_push_is_bounded_by_max_trials_and_reports_a_miss(fake):
    bridge, run, path = prepare_tuning(fake)
    (path / "physics.json").write_text("{}")
    calls_before = len([1 for line in (path / "calls.jsonl").read_text().splitlines() if '"play"' in line])
    stuck = complete(bridge, "tune_push", run, keys=["block=red"], take="one", max_trials=1, push={"waypoints": [[0, 0, -0.2], [6, 0, -0.19]]})
    assert len(stuck["trials"]) == 1 and stuck["stopped_because"] == "max_trials" and not stuck["within_tolerance"]
    assert "Not within tolerance after 1 trials" in stuck["note"] and stuck["best_push"]["waypoints"] == [[0.0, 0.0, -0.2], [6.0, 0.0, -0.19]]
    plays = len([1 for line in (path / "calls.jsonl").read_text().splitlines() if '"play"' in line])
    assert plays - calls_before == 1
    assert bridge.call("tune_push", run=run, keys=["block=red"], max_trials=13)["ok"] is False


def test_tune_push_validates_before_running(fake):
    bridge = fake[0]
    run = bridge.call("start", name="v", device="cpu", template={}, wait_s=10)["run"]
    assert "needs reference frames" in bridge.call("tune_push", run=run, keys=["block=red"], wait_s=10)["operation"]["result"]["error"]
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,6:end")
    assert not bridge.call("tune_push", run=run, wait_s=10)["ok"]  # keys are required
    done = complete(bridge, "tune_push", run, keys=["block=red"], take="a", max_trials=2)
    assert done["ok"] and len(done["trials"]) == 2  # the fake block never moves far enough, so both trials run
    repeat = bridge.call("tune_push", run=run, keys=["block=red"], take="a", max_trials=2)
    assert not repeat["ok"] and "Take already exists" in repeat["error"]
    from_file = bridge.call("start", name="filescene", device="cpu", scene=str(fake[1]), wait_s=10)
    complete(bridge, "reference", from_file["run"], video=str(fake[2]), at="0:start,6:end")
    missing = bridge.call("tune_push", run=from_file["run"], keys=["block=red"], wait_s=10)["operation"]["result"]
    assert not missing["ok"] and "no readable PUSH" in missing["error"]
    given = complete(bridge, "tune_push", from_file["run"], keys=["block=red"], push={"waypoints": [[0, 0, -0.2], [6, 0, 0]]}, max_trials=1)
    assert given["ok"] and given["trials"][0]["push"]["waypoints"][0] == [0.0, 0.0, -0.2]


# ---- a pusher that did not track its path ----

def untracked_scene(tmp_path):
    scene = tmp_path / "untracked.py"
    scene.write_text("UNTRACKED_PUSHER = True\nPUSH = {'waypoints': [[0, 0, -0.2], [2, 0, 0]], 'object_velocity': None}\n")
    return scene


def test_untracked_pusher_message_blames_the_setup_not_the_path():
    assert plan.untracked_message({"pusher_tracked": True}) is None and plan.untracked_message(None) is None
    assert plan.untracked_message({"applied": True}) is None  # unknown is not a failure
    said = plan.untracked_message({"pusher_tracked": False, "pusher_tracking_error_max_m": 0.32423, "solver": "SolverVBD"})
    assert said.startswith("PUSHER DID NOT TRACK ITS PATH") and "0.32423" in said and "SolverVBD" in said
    assert "not in the push path" in said and "MUJOCO_500HZ" in said
    mujoco = plan.untracked_message({"pusher_tracked": False, "pusher_tracking_error_max_m": 0.1, "solver": "SolverMuJoCo"})
    assert "MUJOCO_500HZ" not in mujoco and "free body" in mujoco


def test_rollout_says_first_that_the_pusher_did_not_track(fake, tmp_path):
    bridge = fake[0]
    run = bridge.call("start", name="test", scene=str(untracked_scene(tmp_path)), device="cpu", wait_s=10)["run"]
    result = complete(bridge, "rollout", run, take="miss", duration_s=2, push={"waypoints": [[0, 0, -0.2], [2, 0, 0]]})
    assert list(result)[0] == "headline" and result["pusher_tracked"] is False
    assert result["headline"].startswith("PUSHER DID NOT TRACK ITS PATH") and result["warnings"][0] == result["headline"]
    assert not any("did not move" in w or "change the path" in w for w in result["warnings"])
    assert result["push_applied"]["solver"] == "SolverVBD"
    report = Path(complete(bridge, "report", run)["report"]).read_text()
    assert "PUSHER DID NOT TRACK ITS PATH" in report and "solver SolverVBD" in report
    fine = complete(bridge, "rollout", start(fake)["run"], take="fine", duration_s=2, push={"waypoints": [[0, 0, -0.2], [2, 0, 0]]})
    assert "headline" not in fine and "pusher_tracked" not in fine and fine["push_applied"]["pusher_tracked"] is True


def test_search_push_stops_at_the_first_trial_whose_pusher_did_not_track():
    seen = []

    def evaluate(push, trial):
        seen.append(trial)
        return {"clip_px": [-53.8, -56.6], "sim_px": [0.0, 0.0], "pusher_tracked": False}

    trials, best, why = plan.search_push(PUSH0, (0.0, -0.075), AXES, evaluate, max_trials=6)
    assert seen == [1] and len(trials) == 1 and why == "pusher did not track" and best["trial"] == 1


def test_tune_push_stops_after_one_trial_when_the_pusher_did_not_track(fake, tmp_path):
    bridge = fake[0]
    first = bridge.call("start", name="tune", device="cpu", scene=str(untracked_scene(tmp_path)), wait_s=10)
    run, path = first["run"], Path(first["run_dir"])
    (path / "physics.json").write_text("{}")
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    result = bridge.call("tune_push", run=run, keys=["block=red"], take="t", max_trials=6,
                       push={"waypoints": [[0, 0, -0.2], [6, 0, 0]]}, wait_s=10)["operation"]["result"]
    assert list(result)[0] == "headline" and result["ok"] is False and result["pusher_tracked"] is False
    assert result["stopped_because"] == "pusher did not track" and len(result["trials"]) == 1 and result["takes"] == ["t-1"]
    assert result["headline"].startswith("PUSHER DID NOT TRACK ITS PATH") and result["error"] == result["headline"]
    assert "Stopped after trial 1 of 6" in result["note"] and "Not within tolerance" not in result["note"]
    assert not (path / "t-2.json").exists()
    assert "tune_push stopped early: PUSHER DID NOT TRACK" in Path(bridge.call("report", run=run, wait_s=10)["operation"]["result"]["report"]).read_text()


def test_template_moves_the_pusher_under_mujoco_so_cuda_matches_cpu():
    text = template.scene_text("tee")
    assert "from robosimstudio.specs.physics import MUJOCO_500HZ" in text and "physics=MUJOCO_500HZ" in text
    assert '"solver": type(sim.pipeline.solver).__name__' in text
    assert 'z=PUSHER_Z)' in text and "q[c + 3:c + 7] = [0.0, 0.0, 0.0, 1.0]" in text  # the pusher is held upright at its height
    compile(text, "scene.py", "exec")


def test_start_on_cuda_warns_that_the_first_run_compiles_kernels_and_cpu_does_not(fake):
    bridge, scene, _, _ = fake
    gpu = bridge.call("start", name="g", scene=str(scene), device="cuda:0", allow_concurrent_gpu=True, wait_s=10)
    assert "compiles its kernels" in gpu["operation"]["result"]["gpu_note"] and "minutes" in gpu["operation"]["result"]["gpu_note"]
    assert "gpu_note" not in start(fake)["operation"]["result"]


def test_jenga_template_states_the_default_size_and_long_axis():
    assert "x is the long axis" in template.SIZE_MEANING["jenga"] and "0.072 x 0.024 x 0.014" in template.SIZE_MEANING["jenga"]
    assert "x is the long axis" in template.PARAMETERS["size"]


def test_the_workspace_follows_the_shared_rule(tmp_path, monkeypatch):
    from twinsmith import pack_manifest
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setattr(pack_manifest, "checkout_root", lambda: None)
    monkeypatch.delenv("TWINSMITH_WORKSPACE", raising=False)
    assert RoboSimBridge().workspace == tmp_path / "home" / "robotics-workspace"
    monkeypatch.setenv("TWINSMITH_WORKSPACE", str(tmp_path / "chosen"))
    assert RoboSimBridge().workspace == (tmp_path / "chosen").resolve()
    assert RoboSimBridge(workspace=tmp_path / "given").workspace == (tmp_path / "given").resolve()


def test_a_take_reply_without_readings_is_scored_from_the_take_and_a_failure_is_an_error(fake):
    """RSS at the pin answers compare --take with the frame each anchor was held against and nothing per object; the
    bridge reads the masks itself. When it cannot, the answer is an error, never "did not move"."""
    bridge = fake[0]
    first = start(fake)
    run, path = first["run"], Path(first["run_dir"])
    complete(bridge, "reference", run, video=str(fake[2]), at="0:start,3:middle,6:end")
    complete(bridge, "rollout", run, take="plain", duration_s=6)
    plain = complete(bridge, "compare_take", run, take="plain", keys=["block=red"])
    assert plain["verdict"] == "matches" and plain["anchors"][-1]["objects"]["block"]["iou"] == 0.5
    (request_file,) = path.glob("take-scores-*.json")
    request = json.loads(request_file.read_text())
    assert request["keys"] == ["block=red"] and [a["frame"] for a in request["anchors"]] == [0, 1, 2]
    (path / "later_rss.json").write_text("{}")             # RSS's own readings are used as given, without the script
    later = complete(bridge, "compare_take", run, take="plain", keys=["block=red"])
    assert later["anchors"][-1]["objects"]["block"]["iou"] == 0.7 and len(list(path.glob("take-scores-*.json"))) == 1
    (path / "later_rss.json").unlink()
    (path / "no_frame.json").write_text("{}")
    framed = bridge.call("compare_take", run=run, take="plain", keys=["block=red"], wait_s=10)
    framed = framed.get("operation", {}).get("result", framed)
    assert framed["ok"] is False and "no frame or file" in framed["error"], framed
    (path / "no_frame.json").unlink()
    complete(bridge, "rollout", run, take="unscorable", duration_s=6)
    failed = bridge.call("compare_take", run=run, take="unscorable", keys=["block=red"], wait_s=10)
    result = failed.get("operation", {}).get("result", failed)
    assert result["ok"] is False and "masks could not be read" in result["error"], result
    assert "no segmentation" in result["error"] and result.get("verdict") != "did not move"


RSS_ROOT = Path(os.environ.get("TWINSMITH_ROBOSIM_ROOT") or Path.home() / "robotics-workspace/tools/robosimstudio")


@pytest.mark.skipif(not (RSS_ROOT / ".venv/bin/python").exists(), reason="no RoboSimStudio checkout with its environment")
def test_take_scores_against_the_real_rss_functions(tmp_path):
    """take_scores.py in the real RSS interpreter on a small take: a half-size segmentation, as RSS records it."""
    import numpy as np
    from PIL import Image

    run = tmp_path / "run"
    (run / "ref").mkdir(parents=True)
    width, height = 64, 48
    seg = np.zeros((2, height // 2, width // 2), dtype=np.int32)
    seg[0, 4:12, 4:12] = 3                      # the block, top-left at the start
    seg[1, 4:12, 20:28] = 3                     # and further right at the end
    np.savez_compressed(run / "push_seg.npz", seg=seg)
    (run / "push.json").write_text(json.dumps({"frames": [{"i": 0, "t": 0.0, "step": 0}, {"i": 1, "t": 1.0, "step": 50}],
                                               "names": {"3": "block"}, "width": width, "height": height}))
    for name, x in (("a.png", 8), ("b.png", 40)):
        photo = np.full((height, width, 3), 255, dtype=np.uint8)
        photo[8:24, x:x + 16] = (220, 20, 20)
        Image.fromarray(photo).save(run / "ref" / name)
    request = tmp_path / "request.json"
    request.write_text(json.dumps({"run_dir": str(run), "take": "push", "keys": ["block=red"],
                                   "anchors": [{"file": "a.png", "frame": 0}, {"file": str(run / "ref/b.png"), "frame": 1}]}))
    script = Path(bridge_module.__file__).with_name("take_scores.py")
    done = subprocess.run([str(RSS_ROOT / ".venv/bin/python"), "-I", str(script), str(request)], cwd=tmp_path,
                          capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stderr[-2000:]
    rows = json.loads(done.stdout)["anchors"]
    first, last = (row["objects"]["block"] for row in rows)
    assert first["iou"] > 0.9 and last["iou"] > 0.9, rows
    moved_photo = last["photo"]["centre"][0] - first["photo"]["centre"][0]
    moved_sim = last["sim"]["centre"][0] - first["sim"]["centre"][0]
    assert abs(moved_photo - 32) < 1.5 and abs(moved_sim - 32) < 1.5, (moved_photo, moved_sim)
    request.write_text(json.dumps({"run_dir": str(run), "take": "push", "keys": ["block=red", "floor=red"],
                                   "anchors": [{"file": "a.png", "frame": 0}, {"file": "b.png", "frame": 5}]}))
    late = subprocess.run([str(RSS_ROOT / ".venv/bin/python"), "-I", str(script), str(request)], cwd=tmp_path,
                          capture_output=True, text=True, timeout=120)
    first, last = json.loads(late.stdout)["anchors"]
    assert first["objects"]["floor"]["sim"] is None                 # a name the take has no label for
    assert last["objects"]["block"]["sim"] is None and last["objects"]["block"]["photo"] is not None   # past the seg
    for key in ("block=@20,16", "block=red@20,16", "block=mask:a.png"):
        request.write_text(json.dumps({"run_dir": str(run), "take": "push", "keys": [key],
                                       "anchors": [{"file": "a.png", "frame": 0}]}))
        refused = subprocess.run([str(RSS_ROOT / ".venv/bin/python"), "-I", str(script), str(request)], cwd=tmp_path,
                                 capture_output=True, text=True, timeout=120)
        assert refused.returncode != 0 and "seed or a mask" in refused.stderr, (key, refused.stderr[-500:])
