"""The doctor tool: what computer use needs here and what to fix. X11, the bridge and Kit are faked."""

import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest
from mcp.server.fastmcp import FastMCP

from twinsmith import screen, toolsets
from twinsmith.client import BridgeError

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/sim-tools"))


class FakeClient:
    """A bridge connection that answers sim.ui.find with ``answer`` (raised when it is an exception)."""

    answer: object = RuntimeError("no bridge found")
    calls: list = []

    def __init__(self, host=None, port=None, timeout=None):
        self.endpoint = (host, port, timeout)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def call(self, name, params):
        FakeClient.calls.append((self.endpoint, name, params))
        if isinstance(FakeClient.answer, Exception):
            raise FakeClient.answer
        return FakeClient.answer


@pytest.fixture
def desk(monkeypatch):
    """An unlocked X11 desktop with XTEST and SHAPE, no bridge and no Kit window."""
    disp = MagicMock()
    disp.has_extension.side_effect = lambda name: name in {"XTEST", "SHAPE"}
    monkeypatch.setattr(screen, "ENV_DISPLAY", ":1")
    monkeypatch.setenv("XDG_SESSION_TYPE", "x11")
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_probe_locked", lambda: False)
    monkeypatch.setattr(screen, "_bridge_pid", lambda port: None)
    monkeypatch.setattr(screen, "window_for_pid", lambda pid: None)
    monkeypatch.setattr(screen, "BridgeClient", FakeClient)
    monkeypatch.setattr(FakeClient, "answer", RuntimeError("no bridge found: none advertised in /run/user/1000"))
    monkeypatch.setattr(FakeClient, "calls", [])
    return disp


def _state(*on, badge=False):
    return SimpleNamespace(toolsets=frozenset({"core", "computer", *on}), runtime_enable=True,
                           config=SimpleNamespace(cursor_badge=badge), client=None)


def _doctor(state=None, port=8766):
    server = FastMCP("doctor-test")
    screen.register_screen_tools(server, port_provider=lambda: port, groups={"computer"}, state=state)
    tool = server._tool_manager._tools["doctor"]
    return tool, (lambda: json.loads(tool.fn()))


def test_doctor_is_a_read_only_screen_tool_without_arguments(desk):
    tool, _ = _doctor()
    assert toolsets.owner("doctor") == "computer" and "doctor" in toolsets.MEMBERS["computer"]
    assert tool.annotations.readOnlyHint is True
    assert tool.parameters.get("properties", {}) == {}
    assert len(tool.description) < 160 and "sim" not in tool.name


@pytest.mark.parametrize("session,wayland,x_opens,backend", [
    ("x11", None, True, "x11"), ("wayland", "wayland-0", True, "xwayland"), ("wayland", "wayland-0", False, "wayland"),
    (None, "wayland-0", False, "wayland"), ("tty", None, False, "none"), ("tty", None, True, "x11")])
def test_display_backend(session, wayland, x_opens, backend):
    assert screen.display_backend(session, wayland, x_opens) == backend


def test_an_ssh_session_without_display_reports_the_x_server_it_reaches(desk, monkeypatch):
    monkeypatch.setattr(screen, "ENV_DISPLAY", None)
    monkeypatch.setenv("XDG_SESSION_TYPE", "tty")
    report = _doctor(_state())[1]()
    assert (report["backend"], report["display"], report["ready"]["computer"]) == ("x11", None, True)


def test_no_display_and_no_bridge_still_answers(desk, monkeypatch):
    monkeypatch.setattr(screen, "ENV_DISPLAY", None)
    monkeypatch.delenv("XDG_SESSION_TYPE")
    monkeypatch.setattr(screen, "_xdisplay", MagicMock(side_effect=RuntimeError("cannot open X display :0: ...")))
    monkeypatch.setattr(screen, "_probe_locked", lambda: None)
    report = _doctor(_state(), port=None)[1]()
    assert (report["backend"], report["display"], report["locked"]) == ("none", None, None)
    assert report["x11"] == {"error": "cannot open X display :0"}
    assert report["bridge"] == {"port": None, "reachable": False, "reason": "no bridge is advertised and no port is set"}
    assert report["kit_window"]["found"] is False and report["ready"] == {"computer": False, "sim__ui": False}
    assert report["next"].startswith("computer: cannot open X display :0: computer needs a desktop session; sim__ui: ")
    assert "/run/user" not in json.dumps(report)


def test_pure_wayland_is_named(desk, monkeypatch):
    monkeypatch.setattr(screen, "ENV_DISPLAY", None)
    monkeypatch.setenv("XDG_SESSION_TYPE", "wayland")
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.setattr(screen, "_xdisplay", MagicMock(side_effect=RuntimeError("cannot open X display :0")))
    report = _doctor(_state())[1]()
    assert report["backend"] == "wayland" and report["ready"]["computer"] is False
    assert report["next"].startswith("computer: pure Wayland")


def test_xwayland_with_xtest_is_usable(desk, monkeypatch):
    monkeypatch.setenv("XDG_SESSION_TYPE", "wayland")
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    report = _doctor(_state())[1]()
    assert report["backend"] == "xwayland" and report["ready"]["computer"] is True


def test_locked_screen_blocks_computer_and_points_to_sim_ui(desk, monkeypatch):
    monkeypatch.setattr(screen, "_probe_locked", lambda: True)
    report = _doctor(_state())[1]()
    assert report["locked"] is True and report["ready"]["computer"] is False
    assert "locked" in report["next"] and "use sim__ui" in report["next"]


def test_unlocked_desktop_is_ready(desk):
    report = _doctor(_state(badge=True))[1]()
    assert report["x11"] == {"xtest": True, "shape": True, "compositing": True} and report["locked"] is False
    assert report["ready"]["computer"] is True and report["cursor_badge"] is True
    assert report["screenshot_max_width"] == 1600 and report["next"].startswith("sim__ui: no bridge answers")
    desk.close.assert_called_once()


def test_missing_xtest_blocks_computer(desk):
    desk.has_extension.side_effect = lambda name: name == "SHAPE"
    report = _doctor(_state())[1]()
    assert report["x11"] == {"xtest": False, "shape": True, "compositing": True} and "lacks XTEST" in report["next"]


@pytest.mark.parametrize("error,reason", [
    (ConnectionRefusedError(111, "refused"), "stale or stopped bridge"),
    (TimeoutError("timed out"), "no reply within 3 s"),
    (RuntimeError("multiple bridges running (a, b): pass port="), "use_bridge"),
])
def test_bridge_down_or_stale(desk, error, reason):
    FakeClient.answer = error
    report = _doctor(_state("sim"))[1]()
    assert report["bridge"]["reachable"] is False and reason in report["bridge"]["reason"]
    assert report["ready"]["sim__ui"] is False and reason in report["next"]


def test_bridge_up_with_readiness(desk, monkeypatch):
    FakeClient.answer = {"ui_test_enabled": False, "dpi_scale": 1.5, "main_window": {"width": 1920, "height": 1080},
                         "headless": False}
    monkeypatch.setattr(screen, "_bridge_pid", lambda port: 4242 if port == 8766 else None)
    monkeypatch.setattr(screen, "window_for_pid", lambda pid: {"id": 77, "pid": pid, "title": "Isaac Sim 6.0"})
    report = _doctor(_state("sim"))[1]()
    assert FakeClient.calls == [((None, 8766, screen.BRIDGE_PROBE_S), "sim.ui.find", {"readiness": True, "limit": 0})]
    assert screen.BRIDGE_PROBE_S <= 5
    assert report["bridge"] == {"port": 8766, "reachable": True, "ui_test_enabled": False, "dpi_scale": 1.5,
                                "main_window": {"width": 1920, "height": 1080}, "headless": False}
    assert report["kit_window"] == {"found": True, "id": 77, "pid": 4242} and "Isaac Sim" not in json.dumps(report)
    assert report["ready"] == {"computer": True, "sim__ui": True}
    assert report["next"] == "computer and sim__ui are ready"


def test_sim_ui_needs_sim(desk):
    FakeClient.answer = {"ui_test_enabled": True, "dpi_scale": 1.0, "main_window": {}, "headless": False}
    report = _doctor(_state())[1]()
    assert report["ready"]["sim__ui"] is False
    assert 'toolsets(action="enable", names=["sim"])' in report["next"]


@pytest.mark.parametrize("answer,expect", [
    ({"status": "error", "code": "LIMIT", "message": "limit must be 1..100."}, "predates the readiness probe"),
    (BridgeError("limit must be 1..100.", {"code": "LIMIT"}), "predates the readiness probe"),
    ({"matches": 3, "widgets": []}, "predates the readiness probe"),
    ({"status": "error", "code": "APPLICATION_ERROR", "message": "boom"}, "APPLICATION_ERROR: boom"),
])
def test_older_or_failing_kit_reports_unknown(desk, answer, expect):
    FakeClient.answer = answer
    report = _doctor(_state("sim"))[1]()
    assert report["bridge"]["reachable"] is True and report["bridge"]["readiness"] == "unknown"
    assert expect in report["bridge"]["reason"] and report["ready"]["sim__ui"] is False


@pytest.mark.parametrize("answer", [
    {"status": "error", "code": "NO_UI", "message": "headless"}, BridgeError("headless", {"code": "NO_UI"}),
    {"ui_test_enabled": False, "dpi_scale": 1.0, "main_window": {"width": 0, "height": 0}, "headless": True}])
def test_headless_kit_blocks_sim_ui(desk, answer):
    FakeClient.answer = answer
    report = _doctor(_state("sim"))[1]()
    assert report["bridge"]["headless"] is True and "headless" in report["next"]


def test_doctor_without_state_uses_the_registered_toolsets(desk):
    report = _doctor()[1]()
    assert report["cursor_badge"] is False and "computer_toolset" not in report


def test_kit_readiness_reads_and_never_enables(monkeypatch):
    from sim_tools import graph, ui

    class Manager:  # no set_extension_enabled*: calling one would raise AttributeError
        def __init__(self):
            self.asked = []

        def is_extension_enabled(self, name):
            self.asked.append(name)
            return False

    manager = Manager()
    app = ModuleType("omni.kit.app")
    app.get_app = lambda: SimpleNamespace(get_extension_manager=lambda: manager)
    omni_ui = ModuleType("omni.ui")
    # omni.ui measures the workspace in UI points: 959.8 x 540.1 at scale 2 is a 1920x1080 pixel window.
    omni_ui.Workspace = SimpleNamespace(get_dpi_scale=lambda: 2.0, get_main_window_width=lambda: 959.8,
                                        get_main_window_height=lambda: 540.1)
    kit = ModuleType("omni.kit")
    kit.app = app
    omni = ModuleType("omni")
    omni.kit, omni.ui = kit, omni_ui
    for name, module in (("omni", omni), ("omni.kit", kit), ("omni.kit.app", app), ("omni.ui", omni_ui)):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(graph, "_headless", lambda: False)
    assert ui.readiness() == {"ui_test_enabled": False, "headless": False, "dpi_scale": 2.0,
                              "main_window": {"width": 1920, "height": 1080}}
    assert manager.asked == ["omni.kit.ui_test"]
    monkeypatch.setattr(graph, "_headless", lambda: True)
    assert ui.find({"readiness": True, "limit": 0})["headless"] is True   # answered before any limit check

    def gone():
        raise RuntimeError("no workspace")

    omni_ui.Workspace = SimpleNamespace(get_dpi_scale=gone, get_main_window_width=gone, get_main_window_height=gone)
    assert ui.readiness() == {"ui_test_enabled": False, "headless": True, "dpi_scale": None,
                              "main_window": {"width": None, "height": None}}


def test_notes_say_what_works_with_a_limit(desk, monkeypatch):
    assert _doctor(_state())[1]()["notes"] == []
    monkeypatch.setattr(screen, "_probe_locked", lambda: None)
    monkeypatch.setenv("XDG_SESSION_TYPE", "wayland")
    desk.get_selection_owner.return_value = screen.X.NONE
    report = _doctor(_state())[1]()
    assert report["locked"] is None and report["ready"]["computer"] is True and report["backend"] == "xwayland"
    assert [note.split(":")[0] for note in report["notes"]] == [
        "lock state unknown (no screen saver service answered)", "XWayland", "no compositing manager"]
    assert report["x11"]["compositing"] is False


def test_the_first_sim_ui_call_enabling_ui_test_is_a_note_not_a_blocker(desk):
    FakeClient.answer = {"ui_test_enabled": False, "dpi_scale": 1.0, "main_window": {}, "headless": False}
    report = _doctor(_state("sim"))[1]()
    assert report["ready"]["sim__ui"] is True and "the first sim__ui call turns it on" in report["notes"][0]


def test_a_bridge_error_is_said_once_and_whole(desk):
    message = "AUTH_REQUIRED: this bridge needs its token. A client on the same account reads it from the token file."
    FakeClient.answer = {"status": "error", "code": "AUTH_REQUIRED", "message": message}
    assert _doctor(_state("sim"))[1]()["bridge"]["reason"] == message
