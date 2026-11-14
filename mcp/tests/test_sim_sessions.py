"""Parallel launch policy and process ownership checks without starting Kit."""

import json
import errno
import os
import signal
import socket
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from twinsmith import config, sim_sessions as sim

_REAL_CHECK_PORT = sim._check_port   # the setup fixture replaces it


@pytest.fixture(autouse=True)
def runtime(tmp_path, monkeypatch):
    """The trusted runtime folder claims live in; XDG_RUNTIME_DIR names its parent for children."""
    folder = tmp_path / "twinsmith"
    folder.mkdir(mode=0o700)
    monkeypatch.setattr(sim.client, "_runtime_dir", lambda: str(folder))
    return folder


@pytest.fixture
def setup(monkeypatch, tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    run = root / "run.sh"
    run.write_text("#!/bin/sh\nexit 0\n")
    run.chmod(0o755)
    env = {"TWINSMITH_SIM_ROOT": str(root), "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path / "sessions"),
           "TWINSMITH_SIM_PYTHON": "/configured/sim-python",
           "TWINSMITH_WORKSPACE": "/configured/workspace"}
    monkeypatch.setenv("HOME", str(tmp_path / "home"))       # no configuration file of the machine's own
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [])
    monkeypatch.setattr(sim, "_boot_id", lambda: "test-boot")
    monkeypatch.setattr(sim, "_lab_pids", lambda env: set())
    monkeypatch.setattr(sim, "_check_port", lambda port, reserved=(), allowed=None: None)
    monkeypatch.setattr(sim, "_resources", lambda: {"gpu_total_mib": 16000, "gpu_free_mib": 12000,
                                                     "ram_available_mib": 24000})
    return env


def test_launch_checks_port_domain_limit_and_resources(setup, monkeypatch):
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{"pid": 10, "port": 8766}])
    monkeypatch.setattr(sim, "_domain_of", lambda pid: 7)
    with pytest.raises(ValueError, match="domain 7"):
        sim._launch(setup, 18766, 7, False, False)
    with pytest.raises(ValueError, match="port must"):
        sim._launch(setup, 80, 9, False, False)
    with pytest.raises(ValueError, match="ros_domain"):
        sim._launch(setup, 18766, 233, False, False)
    with pytest.raises(ValueError, match="Cyclone DDS"):
        sim._launch(setup, 18766, 232, False, False)
    setup["TWINSMITH_SIM_MAX_INSTANCES"] = "1"
    with pytest.raises(RuntimeError, match="already active"):
        sim._launch(setup, 18766, 9, False, False)
    setup["TWINSMITH_SIM_MAX_INSTANCES"] = "2"
    monkeypatch.setattr(sim, "_resources", lambda: {"gpu_total_mib": 16000, "gpu_free_mib": 100,
                                                     "ram_available_mib": 24000})
    with pytest.raises(RuntimeError, match="headroom"):
        sim._launch(setup, 18766, 9, False, False)


def test_known_lab_job_counts_toward_limit(setup, monkeypatch):
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{"pid": 10, "port": 8766}])
    monkeypatch.setattr(sim, "_lab_pids", lambda env: {20})
    with pytest.raises(RuntimeError, match="2 known simulator/Lab"):
        sim._launch(setup, 18766, 9, False, False)


def test_launch_records_identity_and_isolates_environment(setup, monkeypatch):
    captured = {}

    def popen(args, **kwargs):
        captured.update(args=args, **kwargs)
        return SimpleNamespace(pid=510, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    monkeypatch.setenv("PYTHONPATH", "/unrelated/project")
    monkeypatch.setenv("TWINSMITH_BIND", "0.0.0.0")
    monkeypatch.setenv("TWINSMITH_SIM_SESSION_ID", "foreign")
    monkeypatch.setenv("TWINSMITH_SIM_SESSION_DIR", "/foreign/session")
    result = sim._launch(setup, 18766, 197, True, False)
    assert result["state"] == "starting" and result["pid"] == 510
    saved = json.loads((Path(result["log"]).parent / "session.json").read_text())
    assert saved["id"] == result["id"] and saved["start_ticks"] == 12345
    assert "boot_id" not in result and "start_ticks" not in result
    child = captured["env"]
    assert child["ROS_DOMAIN_ID"] == "197" and child["TWINSMITH_PORT"] == "18766"
    assert child["TWINSMITH_BIND"] == "127.0.0.1" and child["MCP_CLIENT_OUTPUT_DIR"] == result["output_dir"]
    assert child["TWINSMITH_SIM_SESSION_ID"] == result["id"]
    assert child["TWINSMITH_SIM_SESSION_DIR"] == str(Path(result["log"]).parent)
    assert child["TWINSMITH_SIM_PYTHON"] == "/configured/sim-python"
    assert child["TWINSMITH_WORKSPACE"] == "/configured/workspace"
    assert child["TWINSMITH_PHYSICS_BACKEND"] == config.DEFAULT_PHYSICS_BACKEND     # no argument: the configured default
    assert result["physics_backend"] == config.DEFAULT_PHYSICS_BACKEND and result["physics_backend_source"] == "built-in"
    assert "PYTHONPATH" not in child
    assert "--headless" in captured["args"] and captured["start_new_session"] is True
    assert "--ur5e-panel" not in captured["args"]
    assert captured["args"][-1] == f"--/app/userConfigPath={result['user_config']}"


@pytest.mark.parametrize("motion_bvh", [False, True])
@pytest.mark.parametrize("backend", ["physx", "newton"])
def test_launch_selects_sensor_renderer(setup, monkeypatch, motion_bvh, backend):
    captured = {}

    def popen(args, **kwargs):
        captured["args"] = args
        captured["env"] = kwargs["env"]
        return SimpleNamespace(pid=510, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    result = sim._launch(setup, 18766, 197, False, False, motion_bvh, backend)
    assert ("--motion-bvh" in captured["args"]) is motion_bvh
    assert result["enable_motion_bvh"] is motion_bvh
    assert result["physics_backend"] == backend
    assert captured["env"]["TWINSMITH_PHYSICS_BACKEND"] == backend


def test_unknown_backend_refuses_before_process_or_session_creation(setup):
    with pytest.raises(ValueError, match="physics_backend"):
        sim._launch(setup, 18766, 197, False, False, physics_backend="automatic")
    assert not Path(setup["TWINSMITH_SIM_SESSIONS_DIR"]).exists()


def test_relative_sessions_directory_is_shared_with_simulator(setup, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    setup["TWINSMITH_SIM_SESSIONS_DIR"] = "relative/sessions"
    captured = {}

    def popen(args, **kwargs):
        captured.update(args=args, **kwargs)
        return SimpleNamespace(pid=510, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    result = sim._launch(setup, 18766, 197, False, False)
    session_dir = tmp_path / "relative" / "sessions" / result["id"]
    assert Path(result["log"]) == session_dir / "sim.log"
    assert Path(result["output_dir"]) == session_dir / "output"
    assert Path(result["user_config"]) == session_dir / "user.config.json"
    assert (session_dir / "session.json").is_file()
    assert captured["env"]["MCP_CLIENT_OUTPUT_DIR"] == str(session_dir / "output")
    assert captured["args"][-1] == f"--/app/userConfigPath={session_dir / 'user.config.json'}"
    assert captured["cwd"] != str(tmp_path)


def test_explicit_server_settings_do_not_inherit_an_unselected_sim_interpreter(setup, monkeypatch):
    setup.pop("TWINSMITH_SIM_PYTHON")
    setup.pop("TWINSMITH_WORKSPACE")
    monkeypatch.setenv("TWINSMITH_SIM_PYTHON", "/other/server/python")
    monkeypatch.setenv("TWINSMITH_WORKSPACE", "/other/ws")
    captured = {}

    def popen(args, **kwargs):
        captured.update(kwargs)
        return SimpleNamespace(pid=510, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    sim._launch(setup, 18766, 197, False, False)
    assert "TWINSMITH_SIM_PYTHON" not in captured["env"] and "TWINSMITH_WORKSPACE" not in captured["env"]


def test_failed_identity_check_stops_child_instead_of_waiting_indefinitely(setup, monkeypatch):
    calls = []
    proc = SimpleNamespace(pid=510, terminate=lambda: calls.append("terminate"),
                           wait=lambda timeout: calls.append(("wait", timeout)))
    real_identity = sim._process_identity
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *args, **kwargs: proc)
    monkeypatch.setattr(sim, "_process_identity",
                        lambda pid: None if pid == 510 else real_identity(pid))
    with pytest.raises(RuntimeError, match="could not record simulator process identity"):
        sim._launch(setup, 18766, 197, False, False)
    assert calls == ["terminate", ("wait", 5)]


def _record(env, session_id="a" * 32, **extra):
    path = Path(env["TWINSMITH_SIM_SESSIONS_DIR"]) / session_id
    path.mkdir(parents=True)
    data = {"id": session_id, "pid": 510, "start_ticks": 12345, "boot_id": "test-boot",
            "port": 18766, "ros_domain": 197, **extra}
    (path / "session.json").write_text(json.dumps(data))
    return data


def test_starting_managed_session_reserves_port_and_domain(setup, monkeypatch):
    _record(setup)
    monkeypatch.setattr(sim, "_alive", lambda data: True)
    with pytest.raises(ValueError, match="port 18766"):
        sim._launch(setup, 18766, 198, False, False)
    with pytest.raises(ValueError, match="domain 197"):
        sim._launch(setup, 18767, 197, False, False)


def test_status_verifies_bridge_pid_and_reports_reused_pid_as_exited(setup, monkeypatch):
    record = _record(setup)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.client, "call", lambda *a, **kw: {"pid": 510, "app_ready": True})
    assert sim._status(setup, record["id"])["state"] == "running"
    assert f"use_bridge(port={record['port']})" in sim._status(setup, record["id"])["next"]
    monkeypatch.setattr(sim.client, "call", lambda *a, **kw: {"pid": 510, "app_ready": False})
    assert sim._status(setup, record["id"])["state"] == "starting"
    monkeypatch.setattr(sim.client, "call", lambda *a, **kw: {"pid": 510})
    assert sim._status(setup, record["id"])["state"] == "readiness_unknown"
    monkeypatch.setattr(sim.client, "call", lambda *a, **kw: {"pid": 511})
    assert sim._status(setup, record["id"])["state"] == "port_conflict"
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 99999))
    assert sim._status(setup, record["id"])["state"] == "exited"
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim, "_boot_id", lambda: "new-boot")
    assert sim._status(setup, record["id"])["state"] == "exited"


@pytest.mark.parametrize("value", [None, 0, "true"])
def test_status_does_not_treat_unknown_readiness_as_ready(setup, monkeypatch, value):
    record = _record(setup)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.client, "call", lambda *a, **kw: {"pid": 510, "app_ready": value})
    assert sim._status(setup, record["id"])["state"] == "readiness_unknown"


def test_stop_requires_confirmation_matching_pid_and_process_group(setup, monkeypatch):
    record = _record(setup)
    signals = []
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(sim.os, "pidfd_open", lambda pid, flags: os.open("/dev/null", os.O_RDONLY))
    monkeypatch.setattr(sim.signal, "pidfd_send_signal",
                        lambda fd, sig, info=None, flags=0: signals.append((sig, flags)))
    with pytest.raises(ValueError, match="confirm_discard=true") as error:
        sim._stop(setup, record["id"], 510, False)
    assert f"session_id={record['id']!r}" in str(error.value) and "expected_pid=510" in str(error.value)
    with pytest.raises(ValueError, match="force=true"):
        sim._stop(setup, record["id"], 510, False, force=True)
    with pytest.raises(ValueError, match="expected PID"):
        sim._stop(setup, record["id"], 511, True)
    monkeypatch.setattr(sim.os, "getpgid", lambda pid: 500)
    with pytest.raises(RuntimeError, match="process group"):
        sim._stop(setup, record["id"], 510, True)
    assert signals == []
    monkeypatch.setattr(sim.os, "getpgid", lambda pid: pid)
    assert sim._stop(setup, record["id"], 510, True)["state"] == "stop_requested"
    assert signals == [(signal.SIGINT, sim._PIDFD_SIGNAL_PROCESS_GROUP)]
    assert sim._stop(setup, record["id"], 510, True, force=True)["signal"] == "SIGKILL"
    assert signals[-1] == (signal.SIGKILL, sim._PIDFD_SIGNAL_PROCESS_GROUP)


def test_stop_falls_back_to_pidfd_leader_only_on_older_kernels(setup, monkeypatch):
    record = _record(setup)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(sim.os, "pidfd_open", lambda pid, flags: os.open("/dev/null", os.O_RDONLY))
    calls = []

    def send(fd, sig, info=None, flags=0):
        calls.append(flags)
        if flags:
            raise OSError(errno.EINVAL, "group scope unavailable")

    monkeypatch.setattr(sim.signal, "pidfd_send_signal", send)
    result = sim._stop(setup, record["id"], 510, True)
    assert result["scope"] == "process" and calls == [sim._PIDFD_SIGNAL_PROCESS_GROUP, 0]


def test_stop_does_not_signal_if_identity_changes_after_opening_pidfd(setup, monkeypatch):
    record = _record(setup)
    states = iter([True, False])
    monkeypatch.setattr(sim, "_alive", lambda data: next(states))
    monkeypatch.setattr(sim.os, "pidfd_open", lambda pid, flags: os.open("/dev/null", os.O_RDONLY))
    signals = []
    monkeypatch.setattr(sim.signal, "pidfd_send_signal", lambda *args: signals.append(args))
    assert sim._stop(setup, record["id"], 510, True)["state"] == "exited"
    assert signals == []


@pytest.mark.parametrize("session_id", ["../bad", "abc", "/etc/passwd", "A" * 32])
def test_session_id_cannot_escape_sessions_directory(setup, session_id):
    with pytest.raises(ValueError, match="session ID"):
        sim._status(setup, session_id)


def test_session_lock_reports_non_linux_support(tmp_path, monkeypatch):
    monkeypatch.setattr(sim, "fcntl", None)
    with pytest.raises(RuntimeError, match="require Linux"):
        with sim._launch_lock(tmp_path / "sessions"):
            pass


# ---- launch_simulator refused a free port after stop_simulator --------------------------------

TIME_WAIT, FIN_WAIT2, LISTEN = "06", "05", "0A"


def _plain_bind_refuses(port):
    """The check before the fix: a bind with no SO_REUSEADDR."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            assert exc.errno == errno.EADDRINUSE
            return True
    return False


def _tcp_states(port):
    rows = Path("/proc/net/tcp").read_text().splitlines()[1:]
    return {row.split()[3] for row in rows if int(row.split()[1].split(":")[1], 16) == port}


def _leftover_connection(reuse=True):
    """A served connection the server closed first, as when a bridge process exits. The bridge's
    listener sets SO_REUSEADDR (reuse=True), and accepted sockets inherit it. Returns the port and
    the still-open client socket."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        if reuse:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        peer = socket.create_connection(("127.0.0.1", port), timeout=2)
        accepted, _ = listener.accept()
        accepted.close()
    assert peer.recv(1) == b""   # the client has seen the server's FIN
    return port, peer


def _wait_for_state(port, state):
    deadline = time.monotonic() + 2
    while state not in _tcp_states(port) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert state in _tcp_states(port), f"the test must really reach TCP state {state} on the server port"


def _time_wait_port(reuse=True):
    port, peer = _leftover_connection(reuse)
    peer.close()
    _wait_for_state(port, TIME_WAIT)
    return port


def test_time_wait_failed_the_old_check_and_passes_the_new_one():
    port = _time_wait_port()
    assert LISTEN not in _tcp_states(port)
    assert _plain_bind_refuses(port), "the reported failure: nothing listens, yet the old check refused"
    sim._check_port(port)


def test_a_connection_the_client_has_not_closed_yet_does_not_block_the_port():
    # The host keeps a persistent socket to its bridge, so after stop_simulator the server side can sit
    # in FIN_WAIT2 until the host next uses it.
    port, peer = _leftover_connection()
    with peer:
        _wait_for_state(port, FIN_WAIT2)
        assert _plain_bind_refuses(port)
        sim._check_port(port)




@pytest.mark.parametrize("address", ["127.0.0.1", "0.0.0.0"])
@pytest.mark.parametrize("reuse", [False, True])
def test_a_live_listener_is_still_refused_and_a_free_port_is_suggested(address, reuse):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        if reuse:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((address, 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        with pytest.raises(ValueError, match=f"bridge port {port} is already in use or cannot be bound") as caught:
            sim._check_port(port)
        message = str(caught.value)
        suggested = int(message.split("retry with port=")[1].rstrip("."))
        assert suggested != port
        assert sim._port_bind_error(suggested) is None


def test_the_real_bridge_can_bind_a_port_in_time_wait():
    # The check must not pass a port the launch then cannot use.
    import asyncio
    import threading

    from twinsmith.client import call
    from bridge_testing import make_bridge

    port = _time_wait_port()
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    bridge = make_bridge(loop)
    try:
        bridge.start(port)
        assert call("status", host="127.0.0.1", port=port, timeout=5)["port"] == port
    finally:
        bridge.stop()
        loop.call_soon_threadsafe(loop.stop)
        thread.join(timeout=2)
        loop.close()


def test_time_wait_from_a_listener_without_reuseaddr_stays_refused_and_says_why():
    # Not the bridge's case (it sets SO_REUSEADDR). The bridge could not bind this port either, so
    # refusing is correct, and the message must not claim a process is certainly holding it.
    port = _time_wait_port(reuse=False)
    with pytest.raises(ValueError, match="closed without SO_REUSEADDR"):
        sim._check_port(port)


def test_suggested_port_skips_reserved_ports_and_stays_in_range(monkeypatch):
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    assert "port=40003" in sim._next_free_port_hint(40000, {40001, 40002})
    assert "port=65535" in sim._next_free_port_hint(65534)
    assert "No free port" in sim._next_free_port_hint(65535)
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: OSError(errno.EADDRINUSE, "in use"))
    assert "No free port found in 40001-40064" in sim._next_free_port_hint(40000)


def test_launch_accepts_a_time_wait_port(setup, monkeypatch):
    monkeypatch.setattr(sim, "_check_port", _REAL_CHECK_PORT)
    monkeypatch.setattr(sim.subprocess, "Popen", lambda args, **kw: SimpleNamespace(pid=510, wait=lambda: 0))
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    port = _time_wait_port()
    assert sim._launch(setup, port, 9, True, False)["port"] == port


def test_a_reserved_port_refusal_suggests_a_port_too(setup, monkeypatch):
    _record(setup)
    monkeypatch.setattr(sim, "_alive", lambda data: True)
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    with pytest.raises(ValueError, match="port 18766 is reserved.*retry with port=18767"):
        sim._launch(setup, 18766, 198, False, False)


# ---- the ROS domain refusal says how to proceed ---------------------------------------------------

@pytest.mark.parametrize("holder", ["bridge", "session", "lab"])
def test_ros_domain_refusal_says_how_to_proceed(setup, monkeypatch, holder):
    monkeypatch.setattr(sim, "_domain_of", lambda pid: 0)
    if holder == "bridge":
        monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{"pid": 48526, "port": 8766}])
        named = "bridge PID 48526"
    elif holder == "session":
        data = _record(setup)
        (Path(setup["TWINSMITH_SIM_SESSIONS_DIR"]) / data["id"] / "session.json").write_text(
            json.dumps({**data, "ros_domain": 0}))
        monkeypatch.setattr(sim, "_alive", lambda data: True)
        named = "a managed session"
    else:
        monkeypatch.setattr(sim, "_lab_pids", lambda env: {77})
        named = "Lab job PID 77"
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, 18770, 0, False, False)
    message = str(caught.value)
    assert f"ROS domain 0 is already used by {named}" in message
    assert "another ros_domain" in message and "ROS_DOMAIN_ID" in message
    assert "allow_shared_domain=true" in message and "assume domain 0" in message


def test_allow_shared_domain_still_passes_the_guard(setup, monkeypatch):
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{"pid": 48526, "port": 8766}])
    monkeypatch.setattr(sim, "_domain_of", lambda pid: 0)
    monkeypatch.setattr(sim.subprocess, "Popen", lambda args, **kw: SimpleNamespace(pid=510, wait=lambda: 0))
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    assert sim._launch(setup, 18770, 0, True, True)["ros_domain"] == 0


def test_live_sessions_reports_the_session_id(setup, monkeypatch):
    data = _record(setup)
    monkeypatch.setattr(sim, "_alive", lambda data: True)
    assert sim.live_sessions(setup) == [
        {"id": data["id"], "port": data["port"], "pid": data["pid"], "started_at": None}]


def _tool_server(env):
    """The registered tool functions, called directly."""
    tools = {}

    class Fake:
        def tool(self, name=None, **_):
            def decorate(fn):
                tools[name or fn.__name__] = fn
                return fn
            return decorate

    sim.register_sim_session_tools(Fake(), groups=["computer"], env=env)
    return tools


# ---- externally launched bridges ----

EXTERNAL = {"pid": 4242, "port": 18800, "host": "localhost", "app": "BrickSim", "app_version": "5.1.0",
            "headless": True, "started": 123.0}


def test_list_includes_external_bridges_labeled_unmanaged(setup, monkeypatch):
    server = _tool_server(setup)
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [dict(EXTERNAL)])
    entry = server["list_simulators"]()["sessions"][0]
    assert entry["managed"] is False and entry["id"] == "external-4242-18800"
    assert (entry["pid"], entry["port"], entry["app"], entry["app_version"]) == (4242, 18800, "BrickSim", "5.1.0")


def test_listing_does_not_dial_an_external_bridge(setup, monkeypatch):
    server = _tool_server(setup)
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [dict(EXTERNAL)])
    monkeypatch.setattr(sim.client, "call", lambda *a, **k: pytest.fail("list_simulators must not call a bridge"))
    assert server["list_simulators"]()["sessions"]


def test_a_managed_session_is_not_also_listed_as_external(setup, monkeypatch):
    server = _tool_server(setup)
    record = sim._launch(setup, 18766, 9, True, False)
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{**EXTERNAL, "pid": record["pid"], "port": 18766}])
    monkeypatch.setattr(sim, "_alive", lambda session: True)
    sessions = server["list_simulators"]()["sessions"]
    assert [s["managed"] for s in sessions] == [True] and sessions[0]["id"] == record["id"]


def test_status_of_an_external_bridge_checks_identity(setup, monkeypatch):
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [dict(EXTERNAL)])
    answers = {"pid": 4242, "app_ready": True, "app": "BrickSim", "app_version": "5.1.0"}
    monkeypatch.setattr(sim.client, "call", lambda *a, **k: dict(answers))
    result = sim._status(setup, "external-4242-18800")
    assert result["managed"] is False and result["state"] == "running" and result["app_version"] == "5.1.0"
    assert "use_bridge(port=18800)" in result["next"] and "does not stop" in result["stop"]
    answers["pid"] = 999  # something else now answers that port
    assert sim._status(setup, "external-4242-18800")["state"] == "port_conflict"


def test_status_of_a_gone_external_bridge_says_so(setup):
    with pytest.raises(ValueError, match="no advertised bridge"):
        sim._status(setup, "external-4242-18800")


def test_stop_refuses_an_external_bridge_without_signalling(setup, monkeypatch):
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [dict(EXTERNAL)])
    monkeypatch.setattr(sim, "_signal_session", lambda *a: pytest.fail("must not signal"))
    with pytest.raises(ValueError, match="not a managed session"):
        sim._stop(setup, "external-4242-18800", 4242, True)
    with pytest.raises(ValueError, match="not a managed session"):
        sim._stop(setup, "external-4242-18800", 4242, True, force=True)


def test_managed_entries_say_so(setup):
    record = sim._launch(setup, 18766, 9, True, False)
    assert record["managed"] is True
    assert sim._status(setup, record["id"])["managed"] is True


# ---- a server-configured launcher ----


def launcher(tmp_path, mode=0o755):
    path = tmp_path / "project-launch.sh"
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(mode)
    return path


def test_configured_launcher_runs_with_run_sh_arguments_and_is_recorded(setup, tmp_path, monkeypatch):
    path = launcher(tmp_path)
    setup["TWINSMITH_SIM_LAUNCHER"] = str(path)
    seen = {}
    real = sim.subprocess.Popen

    def popen(args, **kw):
        seen.update(args=args, cwd=kw["cwd"])
        return real(["/bin/true"], **{k: v for k, v in kw.items() if k != "cwd"})

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 1))
    record = sim._launch(setup, 18766, 9, True, False)
    assert seen["args"][:4] == [str(path), "--port", "18766", "--headless"] and seen["cwd"] == str(tmp_path)
    assert record["launcher"] == str(path)


@pytest.mark.parametrize("value,error", [("relative/launch.sh", "absolute"), ("/no/such/launcher", "cannot be read")])
def test_configured_launcher_bad_paths(setup, value, error):
    setup["TWINSMITH_SIM_LAUNCHER"] = value
    with pytest.raises(RuntimeError, match=error):
        sim._launch(setup, 18766, 9, True, False)


def test_configured_launcher_must_not_be_writable_by_others(setup, tmp_path):
    setup["TWINSMITH_SIM_LAUNCHER"] = str(launcher(tmp_path, 0o775))
    with pytest.raises(RuntimeError, match="writable"):
        sim._launch(setup, 18766, 9, True, False)


def test_configured_launcher_must_be_executable(setup, tmp_path):
    setup["TWINSMITH_SIM_LAUNCHER"] = str(launcher(tmp_path, 0o644))
    with pytest.raises(RuntimeError, match="not an executable"):
        sim._launch(setup, 18766, 9, True, False)


def test_no_tool_argument_can_name_a_command(setup):
    import inspect

    server = _tool_server(setup)
    assert set(inspect.signature(server["launch_simulator"]).parameters) == {
        "port", "ros_domain", "headless", "allow_shared_domain", "enable_motion_bvh", "physics_backend"}


def test_launcher_setting_is_read_when_the_server_starts_not_at_each_call(setup, tmp_path, monkeypatch):
    server = _tool_server(setup)
    monkeypatch.setenv("TWINSMITH_SIM_LAUNCHER", "/no/such/launcher")
    assert "TWINSMITH_SIM_LAUNCHER" not in setup
    assert server["launch_simulator"](port=18766, ros_domain=9)["managed"] is True


# ---- Newton on a build without it ----


def fake_interpreter(prefix, release):
    python = prefix / "bin" / "python"
    python.parent.mkdir(parents=True)
    python.write_text("")
    (prefix / "lib" / "python3.11" / "site-packages" / f"isaacsim-{release}.dist-info").mkdir(parents=True)
    return python


def test_newton_on_51_refuses_before_starting_anything(setup, tmp_path, monkeypatch):
    setup["TWINSMITH_SIM_PYTHON"] = str(fake_interpreter(tmp_path / "py51", "5.1.0.0"))
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError, match="5.1.0.0 has no Newton") as error:
        sim._launch(setup, 18766, 9, True, False, physics_backend="newton")
    assert "no fallback" in str(error.value)


def test_physx_on_51_and_newton_on_62_are_not_refused(setup, tmp_path):
    setup["TWINSMITH_SIM_PYTHON"] = str(fake_interpreter(tmp_path / "py51", "5.1.0.0"))
    assert sim._launch(setup, 18766, 9, True, False, physics_backend="physx")["physics_backend"] == "physx"
    setup["TWINSMITH_SIM_PYTHON"] = str(fake_interpreter(tmp_path / "py62", "6.2.0.0"))
    assert sim._launch(setup, 18767, 10, True, False, physics_backend="newton")["physics_backend"] == "newton"


# ---- exited sessions in the listing ----


def _exited_record(base, session_id, started, pid=1):
    folder = base / session_id
    folder.mkdir(parents=True)
    (folder / "session.json").write_text(json.dumps(
        {"id": session_id, "pid": pid, "start_ticks": 1, "boot_id": "other-boot", "port": 18000, "ros_domain": 1,
         "started_at": started, "log": str(folder / "sim.log")}))
    (folder / "sim.log").write_text("kept")
    return folder


def test_listing_bounds_old_exited_sessions_without_deleting_them(setup, tmp_path):
    base = Path(setup["TWINSMITH_SIM_SESSIONS_DIR"])
    folders = [_exited_record(base, f"{i:032x}", float(i)) for i in range(7)]
    server = _tool_server(setup)
    listed = server["list_simulators"]()
    assert [s["id"] for s in listed["sessions"]] == [f"{i:032x}" for i in (6, 5, 4)]
    assert listed["exited_omitted"] == 4 and "include_exited" in listed["note"]
    assert len(server["list_simulators"](include_exited=True)["sessions"]) == 7
    assert all((f / "session.json").is_file() and (f / "sim.log").is_file() for f in folders)
    assert sim._status(setup, f"{0:032x}")["state"] == "exited"  # an unlisted one still answers


def test_listing_never_omits_a_live_session(setup, monkeypatch):
    base = Path(setup["TWINSMITH_SIM_SESSIONS_DIR"])
    for i in range(6):
        _exited_record(base, f"{i:032x}", float(i))
    monkeypatch.setattr(sim, "_alive", lambda session: session["id"] == f"{0:032x}")
    listed = _tool_server(setup)["list_simulators"]()
    assert listed["sessions"][0]["id"] == f"{0:032x}" and listed["sessions"][0]["state"] == "active"


def test_a_launch_inherits_the_servers_environment_and_scrubs_the_documented_keys(setup, monkeypatch):
    """docs/TOOLS-HOST.md says TWINSMITH_ASSET_ROOT and DISPLAY come from the server's environment."""
    seen = {}
    real = sim.subprocess.Popen

    def popen(args, **kw):
        seen.update(kw["env"])
        return real(["/bin/true"], **{k: v for k, v in kw.items() if k != "cwd"})

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 1))
    for key, value in {"TWINSMITH_ASSET_ROOT": "/assets", "DISPLAY": ":7", "PYTHONPATH": "/x", "TWINSMITH_HEADLESS": "1"}.items():
        monkeypatch.setenv(key, value)
    sim._launch(setup, 18766, 9, True, False)
    assert seen["TWINSMITH_ASSET_ROOT"] == "/assets" and seen["DISPLAY"] == ":7"
    for key in ("PYTHONPATH", "TWINSMITH_HEADLESS"):
        assert key not in seen
    assert seen["TWINSMITH_SIM_PYTHON"] == setup["TWINSMITH_SIM_PYTHON"]  # scrubbed, then set from the server's settings


# ---- allowed ranges: which ports and ROS domains a launch may use ---------------------------------

def test_a_configured_range_is_parsed_and_a_blank_one_is_unset():
    env = {"TWINSMITH_PORT_RANGE": "18800-18899", "TWINSMITH_ROS_DOMAIN_RANGE": " 40-49 "}
    assert sim._launch_range(env, "TWINSMITH_PORT_RANGE", 1024, 65535) == (18800, 18899)
    assert sim._launch_range(env, "TWINSMITH_ROS_DOMAIN_RANGE", 0, 231) == (40, 49)
    assert sim._launch_range({}, "TWINSMITH_PORT_RANGE", 1024, 65535) is None
    assert sim._launch_range({"TWINSMITH_PORT_RANGE": "  "}, "TWINSMITH_PORT_RANGE", 1024, 65535) is None


@pytest.mark.parametrize("value", ["18800", "18800-", "-18899", "eighteen", "18899-18800", "80-90", "1-70000"])
def test_a_malformed_port_range_is_an_error_not_ignored(value):
    with pytest.raises(ValueError, match="TWINSMITH_PORT_RANGE"):
        sim._launch_range({"TWINSMITH_PORT_RANGE": value}, "TWINSMITH_PORT_RANGE", 1024, 65535)


@pytest.mark.parametrize("value", ["40", "49-40", "0-232", "abc"])
def test_a_malformed_domain_range_is_an_error_not_ignored(value):
    with pytest.raises(ValueError, match="TWINSMITH_ROS_DOMAIN_RANGE"):
        sim._launch_range({"TWINSMITH_ROS_DOMAIN_RANGE": value}, "TWINSMITH_ROS_DOMAIN_RANGE", 0, 231)


def test_a_malformed_range_refuses_before_starting_anything(setup, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "eighteen-thousand"
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError, match="must be a range like 1024-65535"):
        sim._launch(setup, 18766, 9, True, False)
    assert not Path(setup["TWINSMITH_SIM_SESSIONS_DIR"]).exists()


def test_a_port_outside_the_range_is_refused_with_a_code_and_the_range(setup, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18899"
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, 19000, 9, True, False)
    assert caught.value.code == "PORT_OUT_OF_RANGE"
    assert "19000" in str(caught.value) and "TWINSMITH_PORT_RANGE 18800-18899" in str(caught.value)
    assert not Path(setup["TWINSMITH_SIM_SESSIONS_DIR"]).exists()


def test_a_ros_domain_outside_the_range_is_refused_with_a_code_and_the_range(setup, monkeypatch):
    setup["TWINSMITH_ROS_DOMAIN_RANGE"] = "40-49"
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, 18766, 50, True, False)
    assert caught.value.code == "ROS_DOMAIN_OUT_OF_RANGE"
    assert "TWINSMITH_ROS_DOMAIN_RANGE 40-49" in str(caught.value)


# ---- claims: one exclusive file per port and per ROS domain --------------------------------------

def test_two_threads_cannot_claim_the_same_port(runtime):
    owner = sim._claim_owner()
    barrier = threading.Barrier(2)
    results = []

    def take():
        barrier.wait()
        try:
            sim._claim("port", 18800, owner)
            results.append("claimed")
        except sim._Refusal as exc:
            results.append(exc.code)

    threads = [threading.Thread(target=take) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
    assert sorted(results) == ["PORT_CLAIMED", "claimed"]
    claim = json.loads((runtime / "port-18800.claim").read_text())
    assert claim["pid"] == os.getpid() and claim["start_ticks"] == owner["start_ticks"]


def test_two_host_processes_cannot_claim_the_same_port(runtime, tmp_path):
    """Two real host processes, released together: only one gets the port."""
    go = tmp_path / "go"
    script = textwrap.dedent("""
        import os, sys, time
        from twinsmith import sim_sessions as sim
        deadline = time.monotonic() + 20
        while not os.path.exists(sys.argv[1]):
            if time.monotonic() > deadline:
                print("never started")
                sys.exit(3)
            time.sleep(0.002)
        owner = sim._claim_owner()
        try:
            sim._claim("port", 18800, owner)
        except sim._Refusal as exc:
            print(exc.code)
            sys.exit(0)
        print("claimed")
        time.sleep(0.5)
        sim._release_claim(sim._claim_folder() / "port-18800.claim", owner)
    """)
    env = {**os.environ, "XDG_RUNTIME_DIR": str(tmp_path)}
    procs = [subprocess.Popen([sys.executable, "-c", script, str(go)], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True, env=env) for _ in range(2)]
    go.write_text("go")
    outputs = [proc.communicate(timeout=30)[0].strip() for proc in procs]
    assert sorted(outputs) == ["PORT_CLAIMED", "claimed"], outputs


def test_a_claim_whose_owner_died_is_removed(runtime):
    gone = subprocess.run([sys.executable, "-c", "import os; print(os.getpid())"],
                          capture_output=True, text=True, check=True)
    path = runtime / "port-18800.claim"
    path.write_text(json.dumps({"pid": int(gone.stdout), "start_ticks": 1, "boot_id": sim._boot_id()}))
    owner = sim._claim_owner()
    assert sim._claim("port", 18800, owner) == path
    assert json.loads(path.read_text())["pid"] == os.getpid()


def test_a_claim_with_a_reused_pid_is_stale(runtime):
    owner = sim._claim_owner()
    path = runtime / "ros_domain-40.claim"
    path.write_text(json.dumps({**owner, "start_ticks": owner["start_ticks"] + 1}))
    assert sim._claim("ros_domain", 40, owner) == path


def test_a_claim_from_another_boot_is_stale(runtime):
    owner = sim._claim_owner()
    path = runtime / "ros_domain-41.claim"
    path.write_text(json.dumps({**owner, "boot_id": "another-boot"}))
    assert sim._claim("ros_domain", 41, owner) == path


def test_a_symlink_at_a_claim_path_is_refused_and_left_alone(runtime, tmp_path):
    target = tmp_path / "elsewhere.json"
    target.write_text("{}")
    planted = runtime / "port-18800.claim"
    planted.symlink_to(target)
    with pytest.raises(RuntimeError, match="not a regular file"):
        sim._claim("port", 18800, sim._claim_owner())
    assert planted.is_symlink() and target.read_text() == "{}"


def test_an_untrusted_runtime_folder_is_refused(runtime):
    runtime.chmod(0o777)
    try:
        with pytest.raises(RuntimeError, match="not be writable by group or others"):
            sim._claim("port", 18800, sim._claim_owner())
    finally:
        runtime.chmod(0o700)


def test_a_stated_port_claimed_by_another_launch_is_refused(setup, runtime):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18899"
    sim._claim("port", 18800, sim._claim_owner())
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, 18800, 40, True, False)
    assert caught.value.code == "PORT_CLAIMED" and "18800" in str(caught.value)


def test_a_stated_domain_claimed_by_another_launch_is_refused(setup, runtime):
    setup["TWINSMITH_ROS_DOMAIN_RANGE"] = "40-49"
    sim._claim("ros_domain", 41, sim._claim_owner())
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, 18766, 41, True, False)
    assert caught.value.code == "ROS_DOMAIN_CLAIMED" and "41" in str(caught.value)


def test_allow_shared_domain_skips_the_domain_claim(setup, runtime, monkeypatch):
    sim._claim("ros_domain", 41, sim._claim_owner())
    _fake_spawn(monkeypatch)
    assert sim._launch(setup, 18766, 41, True, True)["ros_domain"] == 41


def test_a_failed_launch_releases_its_claims(setup, runtime, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18899"
    setup["TWINSMITH_ROS_DOMAIN_RANGE"] = "40-49"
    monkeypatch.setattr(sim, "_resources", lambda: {"gpu_total_mib": 16000, "gpu_free_mib": 100,
                                                     "ram_available_mib": 24000})
    with pytest.raises(RuntimeError, match="headroom"):
        sim._launch(setup, 18800, 40, True, False)
    assert not list(runtime.glob("*.claim"))


def test_a_domain_refusal_releases_the_port_claim(setup, runtime, monkeypatch):
    _record(setup)
    monkeypatch.setattr(sim, "_alive", lambda data: True)
    with pytest.raises(ValueError, match="domain 197"):
        sim._launch(setup, 18770, 197, False, False)
    assert not list(runtime.glob("*.claim"))


# ---- what a refusal suggests ---------------------------------------------------------------------

def test_the_hint_only_suggests_ports_inside_the_range(runtime, monkeypatch):
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    message = sim._next_free_port_hint(18999, (), (18800, 18899))
    assert "port=18800" in message and "TWINSMITH_PORT_RANGE 18800-18899" in message
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: OSError(errno.EADDRINUSE, "in use"))
    assert "No free, unclaimed port found in 18800-18899" in sim._next_free_port_hint(18999, (), (18800, 18899))


def test_the_hint_skips_claimed_ports_and_advertised_bridges(runtime, monkeypatch):
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    sim._claim("port", 18800, sim._claim_owner())
    assert "port=18801" in sim._next_free_port_hint(18805, (), (18800, 18810))
    assert "port=18802" in sim._next_free_port_hint(18805, {18801}, (18800, 18810))


def test_without_a_range_the_hint_says_a_port_may_be_someone_elses(runtime, monkeypatch):
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    message = sim._next_free_port_hint(18766, {18767})
    assert "port=18768" in message
    assert "another agent" in message and "TWINSMITH_PORT_RANGE" in message


# ---- choosing an omitted port and ROS domain -----------------------------------------------------

def _fake_spawn(monkeypatch, captured=None):
    captured = {} if captured is None else captured
    real_identity = sim._process_identity

    def popen(args, **kwargs):
        captured.update(args=args, env=kwargs["env"])
        return SimpleNamespace(pid=510, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity",
                        lambda pid: ("S", 12345) if pid == 510 else real_identity(pid))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    return captured


def test_an_omitted_port_and_domain_take_the_first_free_unclaimed_values(setup, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18810"
    setup["TWINSMITH_ROS_DOMAIN_RANGE"] = "40-49"
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    owner = sim._claim_owner()
    sim._claim("port", 18800, owner)      # another launch already holds these two
    sim._claim("ros_domain", 40, owner)
    captured = _fake_spawn(monkeypatch)
    result = sim._launch(setup, headless=True)
    assert (result["port"], result["ros_domain"]) == (18801, 41)
    assert result["auto_selected"] == ["port", "ros_domain"]
    assert captured["env"]["TWINSMITH_PORT"] == "18801" and captured["env"]["ROS_DOMAIN_ID"] == "41"
    assert captured["args"][captured["args"].index("--port") + 1] == "18801"
    held = json.loads((Path(sim.client._runtime_dir()) / "port-18801.claim").read_text())
    assert held["pid"] != os.getpid()  # handed to the simulator, so it outlives the launching call


def test_an_omitted_port_skips_a_live_session_and_an_advertised_bridge(setup, monkeypatch):
    setup["TWINSMITH_PORT"] = "18766"
    setup["TWINSMITH_SIM_MAX_INSTANCES"] = "3"
    _record(setup)
    monkeypatch.setattr(sim, "_alive", lambda data: True)
    monkeypatch.setattr(sim.client, "discover_bridges", lambda: [{"pid": 10, "port": 18767}])
    monkeypatch.setattr(sim, "_domain_of", lambda pid: 99)
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    _fake_spawn(monkeypatch)
    result = sim._launch(setup, ros_domain=5, headless=True)
    assert result["port"] == 18768 and result["auto_selected"] == ["port"]


def test_without_a_range_an_omitted_port_starts_at_the_configured_one(setup, monkeypatch):
    setup["TWINSMITH_PORT"] = "18800"
    setup["ROS_DOMAIN_ID"] = "40"
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    _fake_spawn(monkeypatch)
    result = sim._launch(setup, headless=True)
    assert (result["port"], result["ros_domain"]) == (18800, 40)
    assert result["auto_selected"] == ["port", "ros_domain"]


def test_without_a_range_a_stated_port_and_domain_still_work(setup, runtime, monkeypatch):
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    _fake_spawn(monkeypatch)
    result = sim._launch(setup, 18766, 197, True, False)
    assert (result["port"], result["ros_domain"]) == (18766, 197)
    assert "auto_selected" not in result
    assert all(json.loads(p.read_text())["pid"] != os.getpid() for p in runtime.glob("*.claim"))


def test_a_stated_port_and_domain_inside_the_range_are_claimed_and_used(setup, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18899"
    setup["TWINSMITH_ROS_DOMAIN_RANGE"] = "40-49"
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: None)
    captured = _fake_spawn(monkeypatch)
    result = sim._launch(setup, 18850, 45, True, False)
    assert (result["port"], result["ros_domain"]) == (18850, 45) and "auto_selected" not in result
    assert captured["env"]["ROS_DOMAIN_ID"] == "45"


def test_an_exhausted_range_says_so(setup, monkeypatch):
    setup["TWINSMITH_PORT_RANGE"] = "18800-18800"
    monkeypatch.setattr(sim, "_port_bind_error", lambda port: OSError(errno.EADDRINUSE, "in use"))
    with pytest.raises(ValueError) as caught:
        sim._launch(setup, headless=True)
    assert caught.value.code == "PORT_RANGE_EXHAUSTED" and "18800-18800" in str(caught.value)
    assert "TWINSMITH_PORT_RANGE" in str(caught.value)


def test_claims_handed_to_the_simulator_outlive_the_launching_host(runtime):
    """Review fix: a host that returns while Kit is starting must not free the port before the bridge advertises it."""
    owner = sim._claim_owner()
    sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        ticks = sim._process_identity(sleeper.pid)[1]
        with sim._claims() as claim:
            claim("port", 18810)
            claim.handover({"pid": sleeper.pid, "start_ticks": ticks, "boot_id": sim._boot_id()})
        assert json.loads((runtime / "port-18810.claim").read_text())["pid"] == sleeper.pid
        with pytest.raises(sim._Refusal):
            sim._claim("port", 18810, owner)
    finally:
        sleeper.kill()
        sleeper.wait()
    assert sim._claim("port", 18810, owner)  # simulator gone: stale, so taken


def test_a_claim_stays_live_while_a_group_member_outlives_its_launcher(runtime):
    """Review fix: the launcher exits before Kit; the claim must not go stale until the group is empty."""
    child = subprocess.Popen([sys.executable, "-c",
                              "import subprocess,sys,time;"
                              "subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)']);"
                              "time.sleep(0.2)"], start_new_session=True)
    try:
        ticks = sim._process_identity(child.pid)[1]
        data = {"pid": child.pid, "start_ticks": ticks, "boot_id": sim._boot_id()}
        child.wait()  # the launcher is gone; its child keeps the group alive
        assert sim._claim_is_live(data)
    finally:
        for member in sim._group_members(child.pid):
            os.kill(member, signal.SIGKILL)
    time.sleep(0.2)
    assert not sim._claim_is_live(data)


def test_a_failed_handover_does_not_release_the_claim(runtime, monkeypatch):
    owner = sim._claim_owner()
    monkeypatch.setattr(sim.os, "replace", lambda *a: (_ for _ in ()).throw(OSError("full")))
    with sim._claims() as claim:
        claim("port", 18820)
        claim.handover({"pid": 1, "start_ticks": 1, "boot_id": "x"})
    assert (runtime / "port-18820.claim").exists()
    with pytest.raises(sim._Refusal):
        sim._claim("port", 18820, owner)


_NEXT = {"n": 0}


def _launch_env(setup, monkeypatch):
    captured = {}
    _NEXT["n"] += 1

    def popen(args, **kwargs):
        captured["env"] = kwargs["env"]
        return SimpleNamespace(pid=511, wait=lambda: 0)

    monkeypatch.setattr(sim.subprocess, "Popen", popen)
    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.threading, "Thread", lambda **kwargs: SimpleNamespace(start=lambda: None))
    sim._launch(setup, 18890 + _NEXT["n"], 190 + _NEXT["n"], True, False)
    return captured["env"]


def test_a_launched_kit_gets_the_project_this_host_resolves(setup, monkeypatch, tmp_path):
    """Kit starts in another folder; without this it would never read the project's pack or asset_path."""
    project = tmp_path / "proj" / "twinsmith.json"
    monkeypatch.delenv("TWINSMITH_PROJECT", raising=False)
    monkeypatch.setattr(sim, "_host_project_file", lambda: project)
    assert _launch_env(setup, monkeypatch)["TWINSMITH_PROJECT"] == str(project)


def test_an_explicit_project_setting_is_kept_and_no_project_adds_nothing(setup, monkeypatch, tmp_path):
    monkeypatch.setenv("TWINSMITH_PROJECT", "/chosen/project")
    monkeypatch.setattr(sim, "_host_project_file", lambda: tmp_path / "other" / "twinsmith.json")
    assert _launch_env(setup, monkeypatch)["TWINSMITH_PROJECT"] == "/chosen/project"
    monkeypatch.delenv("TWINSMITH_PROJECT")
    monkeypatch.setattr(sim, "_host_project_file", lambda: None)
    assert "TWINSMITH_PROJECT" not in _launch_env(setup, monkeypatch)


def test_session_records_carry_the_argument_name_the_other_tools_take():
    """launch_simulator and list_simulators say `id`, while simulator_status and stop_simulator ask for `session_id`: give both."""
    record = sim._public({"id": "abc123", "pid": 5, "boot_id": "x", "start_ticks": 1})
    assert record["session_id"] == "abc123" and record["id"] == "abc123"
    assert "boot_id" not in record and "start_ticks" not in record
    assert "session_id" not in sim._public({"pid": 5})


# ---- a Newton session says whether Newton is ready, not just that Kit is up ----

def _newton_kit(monkeypatch, engine="newton", tensors=True, release="6.2.0.0", stage_error=None):
    def call(tool, *args, **kw):
        if tool == "status":
            return {"pid": 510, "app_ready": True}
        if stage_error is not None:
            raise stage_error
        return {"runtime": {"physics_engine": engine, "release": release,
                            "extensions": {"isaacsim.physics.newton.tensors": {"enabled": tensors}}}}

    monkeypatch.setattr(sim, "_process_identity", lambda pid: ("S", 12345))
    monkeypatch.setattr(sim.client, "call", call)


def test_a_running_newton_session_reports_engine_and_tensor_readiness(setup, monkeypatch):
    record = _record(setup, physics_backend="newton")
    _newton_kit(monkeypatch)
    status = sim._status(setup, record["id"])
    assert status["state"] == "running" and status["physics"] == {"ready": True, "engine": "newton", "tensor_extension_enabled": True}


@pytest.mark.parametrize("kit,problem", [
    (dict(tensors=False), "isaacsim.physics.newton.tensors is off"),
    (dict(engine="physx"), "the active engine is 'physx'"),
])
def test_a_newton_session_that_is_not_ready_says_why(setup, monkeypatch, kit, problem):
    record = _record(setup, physics_backend="newton")
    _newton_kit(monkeypatch, **kit)
    physics = sim._status(setup, record["id"])["physics"]
    assert physics["ready"] is False and problem in physics["note"] and "Do not start a campaign" in physics["note"]


def test_the_first_newton_release_needs_no_separate_tensor_extension(setup, monkeypatch):
    record = _record(setup, physics_backend="newton")
    _newton_kit(monkeypatch, tensors=False, release="6.0.0.0")
    assert sim._status(setup, record["id"])["physics"]["ready"] is True


def test_newton_readiness_is_unknown_until_the_scene_tools_answer_and_physx_sessions_have_none(setup, monkeypatch):
    record = _record(setup, physics_backend="newton")
    _newton_kit(monkeypatch, stage_error=sim.client.BridgeError("unknown tool: 'sim.stage.info'"))
    physics = sim._status(setup, record["id"])["physics"]
    assert physics["ready"] is None and "call simulator_status again" in physics["note"]
    plain = _record(setup, session_id="b" * 32, port=18999)
    _newton_kit(monkeypatch)
    assert "physics" not in sim._status(setup, plain["id"])


# ---- the configured default, and no silent fallback in either direction ----


def test_the_launch_argument_defaults_to_the_configured_backend_not_a_literal():
    import inspect
    assert inspect.signature(sim._launch).parameters["physics_backend"].default is None


def test_the_current_default_is_newton():
    """The one test that pins the value: flipping the default in twinsmith.config changes this line only."""
    assert config.DEFAULT_PHYSICS_BACKEND == "newton"


@pytest.mark.parametrize("configured,other", [("physx", "newton"), ("newton", "physx")])
def test_the_configured_default_applies_when_no_argument_is_given_and_an_argument_wins(setup, configured, other):
    setup["TWINSMITH_PHYSICS_BACKEND"] = configured
    started = sim._launch(setup, 18766, 9, True, False)
    assert started["physics_backend"] == configured and started["physics_backend_source"] == "environment"
    chosen = sim._launch(setup, 18767, 10, True, False, physics_backend=other)
    assert chosen["physics_backend"] == other and chosen["physics_backend_source"] == "argument"


def test_the_engine_is_always_named_to_the_new_process(setup, monkeypatch):
    """So a changed default can never change what an earlier call asked for."""
    seen = []
    monkeypatch.setattr(sim.subprocess, "Popen", lambda args, **k: seen.append(k["env"]) or SimpleNamespace(pid=510, wait=lambda: 0))
    monkeypatch.setattr(sim, "_process_identity", lambda pid: (pid, 12345))
    setup["TWINSMITH_PHYSICS_BACKEND"] = "newton"
    sim._launch(setup, 18766, 9, True, False, physics_backend="physx")
    sim._launch(setup, 18767, 10, True, False)
    assert [env["TWINSMITH_PHYSICS_BACKEND"] for env in seen] == ["physx", "newton"]


def test_a_configured_newton_on_a_build_without_it_starts_nothing_instead_of_falling_back_to_physx(setup, tmp_path, monkeypatch):
    setup["TWINSMITH_SIM_PYTHON"] = str(fake_interpreter(tmp_path / "py51", "5.1.0.0"))
    setup["TWINSMITH_PHYSICS_BACKEND"] = "newton"
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError, match="5.1.0.0 has no Newton") as error:
        sim._launch(setup, 18766, 9, True, False)
    assert "no fallback" in str(error.value) and "physics_backend=physx" in str(error.value)


def test_an_explicit_physx_on_that_build_still_starts_and_is_not_changed_by_the_configured_default(setup, tmp_path):
    setup["TWINSMITH_SIM_PYTHON"] = str(fake_interpreter(tmp_path / "py51", "5.1.0.0"))
    setup["TWINSMITH_PHYSICS_BACKEND"] = "newton"
    assert sim._launch(setup, 18766, 9, True, False, physics_backend="physx")["physics_backend"] == "physx"


def test_a_bad_configured_backend_starts_nothing_and_says_where_it_came_from(setup, monkeypatch):
    setup["TWINSMITH_PHYSICS_BACKEND"] = "bullet"
    monkeypatch.setattr(sim.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    with pytest.raises(ValueError, match="physics_backend must be physx or newton, not 'bullet' \\(from the environment\\)"):
        sim._launch(setup, 18766, 9, True, False)
