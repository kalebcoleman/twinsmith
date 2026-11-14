"""Unit tests for screen window enumeration (no X display required)."""

import types
from unittest.mock import MagicMock

import pytest
from Xlib import X

import twinsmith.screen as screen


def _make_window(wid, title, viewable=True, size=(1440, 900)):
    win = MagicMock()
    win.id = wid
    attrs = MagicMock()
    attrs.map_state = X.IsViewable if viewable else 0
    win.get_attributes.return_value = attrs
    geom = MagicMock()
    geom.width = size[0]
    geom.height = size[1]
    win.get_geometry.return_value = geom
    win.get_wm_class.return_value = ()
    return win


def test_visible_windows_walks_x_tree(monkeypatch):
    disp = MagicMock()
    root = MagicMock()
    disp.screen.return_value.root = root
    disp.intern_atom.side_effect = lambda name: name
    root.get_full_property.return_value = None  # _NET_CLIENT_LIST absent
    root.translate_coords.return_value = MagicMock(x=0, y=0)

    titles = {100: "Isaac Sim Python 6.2.0", 101: "Terminal"}
    wins = [_make_window(wid, t) for wid, t in titles.items()]

    def fake_title(win, d):
        return titles[win.id]

    root.query_tree.return_value.children = wins

    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_window_title", fake_title)
    monkeypatch.setattr(screen, "_window_pid", lambda win, d: None)

    result = screen._visible_windows()
    assert {w["title"] for w in result} == set(titles.values())
    assert len(result) == 2


def test_visible_windows_uses_net_client_list(monkeypatch):
    disp = MagicMock()
    root = MagicMock()
    disp.screen.return_value.root = root
    disp.intern_atom.side_effect = lambda name: name

    prop = MagicMock()
    prop.value = [200, 201]
    root.get_full_property.return_value = prop
    root.query_tree.return_value.children = []
    root.translate_coords.return_value = MagicMock(x=0, y=0)

    titles = {200: "Isaac Sim Python 6.2.0", 201: "Editor"}
    win_map = {}
    for wid, t in titles.items():
        w = _make_window(wid, t)
        win_map[wid] = w

    def fake_title(win, d):
        return titles[win.id]

    disp.create_resource_object.side_effect = lambda kind, wid: win_map[wid]

    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_window_title", fake_title)
    monkeypatch.setattr(screen, "_window_pid", lambda win, d: None)

    result = screen._visible_windows()
    assert {w["title"] for w in result} == set(titles.values())


def test_visible_windows_falls_back_when_client_list_lacks_window(monkeypatch):
    disp = MagicMock()
    root = MagicMock()
    disp.screen.return_value.root = root
    disp.intern_atom.side_effect = lambda name: name

    # _NET_CLIENT_LIST omits the Isaac window (id 300).
    prop = MagicMock()
    prop.value = [301]
    root.get_full_property.return_value = prop
    root.translate_coords.return_value = MagicMock(x=0, y=0)

    titles = {300: "Isaac Sim Python 6.2.0", 301: "Terminal"}
    win_map = {301: _make_window(301, "Terminal")}
    isaac_win = _make_window(300, "Isaac Sim Python 6.2.0")

    def fake_title(win, d):
        return titles[win.id]

    disp.create_resource_object.side_effect = lambda kind, wid: win_map[wid]
    root.query_tree.return_value.children = [isaac_win, win_map[301]]

    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_window_title", fake_title)
    monkeypatch.setattr(screen, "_window_pid", lambda win, d: None)

    result = screen._visible_windows()
    assert "Isaac Sim Python 6.2.0" in {w["title"] for w in result}


def test_visible_windows_skips_untitled(monkeypatch):
    disp = MagicMock()
    root = MagicMock()
    disp.screen.return_value.root = root
    disp.intern_atom.side_effect = lambda name: name
    root.get_full_property.return_value = None
    root.translate_coords.return_value = MagicMock(x=0, y=0)

    titled = _make_window(1, "Isaac Sim Python 6.2.0")
    untitled = _make_window(2, "")
    root.query_tree.return_value.children = [titled, untitled]

    def fake_title(win, d):
        return "Isaac Sim Python 6.2.0" if win.id == 1 else ""

    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_window_title", fake_title)
    monkeypatch.setattr(screen, "_window_pid", lambda win, d: None)

    result = screen._visible_windows()
    assert len(result) == 1
    assert result[0]["title"] == "Isaac Sim Python 6.2.0"


def test_auto_window_matches_isaac_prefix(monkeypatch):
    wins = [
        {"id": 1, "title": "Isaac Sim Python 6.2.0", "class": "", "width": 1440, "height": 900, "pid": None},
        {"id": 2, "title": "Terminal", "class": "", "width": 800, "height": 600, "pid": None},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    monkeypatch.setattr(screen, "_bridge_pid", lambda port: None)
    w = screen._auto_window()
    assert w is not None
    assert w["title"] == "Isaac Sim Python 6.2.0"


def test_auto_window_case_insensitive(monkeypatch):
    wins = [
        {"id": 1, "title": "isaac sim foo", "class": "", "width": 100, "height": 100, "pid": None},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    monkeypatch.setattr(screen, "_bridge_pid", lambda port: None)
    assert screen._auto_window()["title"] == "isaac sim foo"


def test_match_window_by_id(monkeypatch):
    wins = [
        {"id": 5, "title": "Isaac Sim Python 6.2.0", "class": "", "width": 100, "height": 100, "pid": None},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    w = screen._match_window(None, 5)
    assert w["id"] == 5


def test_window_pixels(monkeypatch):
    disp = MagicMock()
    obj = MagicMock()
    disp.create_resource_object.return_value = obj
    geom = MagicMock()
    geom.width = 4
    geom.height = 3
    obj.get_geometry.return_value = geom
    obj.get_image.return_value.data = b"\x00" * (4 * 3 * 4)
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    width, height, data = screen._window_pixels({"id": 7, "title": "t"})
    assert (width, height) == (4, 3)
    assert len(data) == 4 * 3 * 4

def test_auto_target_matches_isaac_lab_windows(monkeypatch):
    wins = [
        {"id": 1, "title": "Mozilla Firefox - Isaac Sim docs", "class": "firefox", "pid": 10, "width": 1900, "height": 1000},
        {"id": 2, "title": "Isaac Lab 3.0.0", "class": "Isaac Lab 3.0.0", "pid": 20, "width": 1440, "height": 900},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    assert screen._auto_window()["id"] == 2


def test_window_for_pid(monkeypatch):
    wins = [
        {"id": 1, "title": "a", "class": "", "pid": 7, "width": 10, "height": 10},
        {"id": 2, "title": "b", "class": "", "pid": 7, "width": 100, "height": 100},
        {"id": 3, "title": "c", "class": "", "pid": 8, "width": 999, "height": 999},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    assert screen.window_for_pid(7)["id"] == 2
    assert screen.window_for_pid(None) is None
    assert screen.window_for_pid(99) is None


def test_prefer_managed_client_over_larger_window_frame(monkeypatch):
    wins = [
        {"id": 1, "title": "Isaac Sim", "class": "frame", "pid": 7,
         "width": 1200, "height": 900, "is_client": False},
        {"id": 2, "title": "Isaac Sim", "class": "Isaac Sim", "pid": 7,
         "width": 1100, "height": 800, "is_client": True},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    monkeypatch.setattr(screen, "_bridge_pid", lambda port: 7)
    assert screen._match_window("Isaac Sim", None)["id"] == 2
    assert screen._auto_window(8766)["id"] == 2
    assert screen.window_for_pid(7)["id"] == 2
    assert screen._match_window(None, 1)["id"] == 1  # explicit ID is never substituted


def test_raising_by_title_rejects_windows_owned_by_different_processes(monkeypatch):
    wins = [
        {"id": 1, "title": "Isaac Sim", "class": "Isaac Sim", "pid": 7,
         "width": 1000, "height": 800, "is_client": True},
        {"id": 2, "title": "Isaac Sim", "class": "Isaac Sim", "pid": 8,
         "width": 1000, "height": 800, "is_client": True},
    ]
    monkeypatch.setattr(screen, "_visible_windows", lambda: wins)
    with pytest.raises(ValueError, match="multiple processes"):
        screen._match_window("Isaac Sim", None, True)
    assert screen._match_window(None, 2, True)["pid"] == 8


def test_raise_window_requests_and_verifies_window_manager_activation(monkeypatch):
    from mcp.server.fastmcp import FastMCP

    disp = MagicMock()
    root = disp.screen.return_value.root
    root.get_full_property.return_value.value = [7]
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_match_window", lambda *_: {"id": 7, "title": "Sim"})
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    server = FastMCP("screen-test")
    screen.register_screen_tools(server, groups={"computer"})
    result = server._tool_manager._tools["raise_window"].fn(window_id=7)
    assert result == "activated: Sim (id=7)"
    root.send_event.assert_called_once()
    disp.close.assert_called_once()


def test_raise_window_refuses_an_unexpected_pid(monkeypatch):
    from mcp.server.fastmcp import FastMCP

    monkeypatch.setattr(screen, "_match_window", lambda *_: {"id": 7, "title": "Sim", "pid": 22})
    server = FastMCP("screen-test")
    screen.register_screen_tools(server, groups={"computer"})
    with pytest.raises(ValueError, match="PID"):
        server._tool_manager._tools["raise_window"].fn(window_id=7, expected_pid=23)


def test_move_window_unmaximizes_moves_and_reports_geometry(monkeypatch):
    from mcp.server.fastmcp import FastMCP

    disp = MagicMock()
    obj = disp.create_resource_object.return_value
    obj.get_geometry.return_value = types.SimpleNamespace(width=900, height=1000)
    disp.screen.return_value.root.translate_coords.return_value = types.SimpleNamespace(x=960, y=40)
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_match_window", lambda *_: {"id": 7, "title": "RViz", "pid": 5})
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    server = FastMCP("screen-test")
    screen.register_screen_tools(server, groups={"computer"})
    tool = server._tool_manager._tools["move_window"].fn
    assert tool(x=960, y=40, width=900, height=1000, window_id=7).startswith("moved: RViz (id=7) now 900x1000+960+40")
    sent = [c.args[0] for c in disp.screen.return_value.root.send_event.call_args_list]
    assert len(sent) == 2 and sent[1]["data"][1][1:] == [960, 40, 900, 1000]
    with pytest.raises(ValueError, match="PID"):
        tool(x=0, y=0, width=900, height=900, window_id=7, expected_pid=99)
    with pytest.raises(ValueError, match="100..16384"):
        tool(x=0, y=0, width=10, height=900, window_id=7)


def test_move_window_waits_for_a_position_only_move(monkeypatch):
    from mcp.server.fastmcp import FastMCP

    disp = MagicMock()
    obj = disp.create_resource_object.return_value
    obj.get_geometry.return_value = types.SimpleNamespace(width=900, height=1000)
    positions = iter([types.SimpleNamespace(x=0, y=0)] * 2 + [types.SimpleNamespace(x=960, y=40)] * 50)
    disp.screen.return_value.root.translate_coords.side_effect = lambda *a: next(positions)
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_match_window", lambda *_: {"id": 7, "title": "RViz", "pid": 5})
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    server = FastMCP("screen-test")
    screen.register_screen_tools(server, groups={"computer"})
    out = server._tool_manager._tools["move_window"].fn(x=960, y=40, width=900, height=1000, window_id=7)
    assert "now 900x1000+960+40" in out  # not the stale +0+0 read right after sending


def test_locked_screen_is_detected_from_loginctl_or_the_screensaver(monkeypatch):
    import itertools

    from twinsmith import screen

    ticks = itertools.count(100, 10)  # each call is 10 s after the last: no cache hits

    outputs = {}
    monkeypatch.setattr(screen, "_LOCK_CACHE", [-1e9, False])
    monkeypatch.setattr(screen.time, "monotonic", lambda: 0.0 + len(outputs) * 0 + next(ticks))
    monkeypatch.setenv("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/1/bus")
    monkeypatch.setattr(screen.shutil, "which", lambda cmd: "/usr/bin/" + cmd)
    monkeypatch.setattr(screen.subprocess, "run",
                        lambda cmd, **kw: types.SimpleNamespace(stdout=outputs.get(cmd[0], "")))
    monkeypatch.setenv("XDG_SESSION_ID", "3")
    outputs.update(loginctl="yes\n")
    assert screen._screen_locked()
    outputs.update(loginctl="no", gdbus="(true,)")
    assert screen._screen_locked()
    outputs.update(gdbus="(false,)")
    assert not screen._screen_locked()
    monkeypatch.setattr(screen.shutil, "which", lambda cmd: None)
    assert not screen._screen_locked()


def test_lock_probe_is_cached_and_skips_gdbus_without_a_session_bus(monkeypatch):
    from twinsmith import screen

    calls = []
    monkeypatch.setattr(screen.shutil, "which", lambda cmd: "/usr/bin/" + cmd)
    monkeypatch.setattr(screen.subprocess, "run",
                        lambda cmd, **kw: calls.append(cmd[0]) or types.SimpleNamespace(stdout="(true,)"))
    monkeypatch.delenv("XDG_SESSION_ID", raising=False)
    monkeypatch.delenv("DBUS_SESSION_BUS_ADDRESS", raising=False)
    monkeypatch.setenv("XDG_RUNTIME_DIR", "/nonexistent-runtime-dir")
    monkeypatch.setattr(screen, "_LOCK_CACHE", [-1e9, False])
    assert not screen._screen_locked() and calls == []  # no bus: no gdbus autolaunch
    monkeypatch.setenv("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/1/bus")
    assert not screen._screen_locked()  # cached for 5 s
    monkeypatch.setattr(screen, "_LOCK_CACHE", [-1e9, False])
    assert screen._screen_locked() and calls == ["gdbus"]


def test_monitor_captures_warn_when_locked_and_window_captures_do_not(monkeypatch, tmp_path):
    from mcp.server.fastmcp import FastMCP

    from twinsmith import screen

    class FakeShot:
        size = (4, 2)
        bgra = bytes(4 * 2 * 4)

    class FakeMSS:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def grab(self, bbox):
            return FakeShot()

    monkeypatch.setattr(screen, "_screen_locked", lambda: True)
    monkeypatch.setattr(screen, "_composite", lambda bbox: (screen.PILImage.new("RGB", (4, 2)), 1))
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path)
    monkeypatch.setattr(screen.mss, "MSS", FakeMSS)
    monkeypatch.setattr(screen, "_bbox", lambda m=0: {"left": 0, "top": 0, "width": 4, "height": 2})
    window = {"id": 7, "title": "Isaac Sim", "x": 0, "y": 0, "width": 4, "height": 2}
    monkeypatch.setattr(screen, "_match_window", lambda *a, **k: window)
    monkeypatch.setattr(screen, "_grab_window_image", lambda win: screen.PILImage.new("RGB", (4, 2)))
    server = FastMCP("t")
    screen.register_screen_tools(server, groups={"computer"})
    shot = server._tool_manager._tools["screenshot"].fn
    assert "screen is locked" in shot(monitor=1)[1]
    assert "screen is locked" not in shot(window_id=7)[1]


def test_locked_screenshot_is_composed_bottom_to_top_without_wm_frames():
    from twinsmith import screen

    wins = [{"id": 1, "class": "Isaac"}, {"id": 2, "class": "mutter-x11-frames"}, {"id": 3, "class": "rviz2"},
            {"id": 4, "class": "unmanaged"}]
    order = [w["id"] for w in screen.composite_order(wins, stacking=[3, 1])]
    assert order == [3, 1, 4]  # unmanaged (popups, tooltips) above the managed windows


def test_lock_probe_is_unknown_when_only_loginctl_says_no(monkeypatch):
    # Started over SSH, XDG_SESSION_ID is the SSH session: "no" says nothing about the desktop.
    from twinsmith import screen

    outputs = {"loginctl": "no", "gdbus": ""}
    monkeypatch.setattr(screen.shutil, "which", lambda cmd: "/usr/bin/" + cmd)
    monkeypatch.setattr(screen.subprocess, "run", lambda cmd, **kw: types.SimpleNamespace(stdout=outputs[cmd[0]]))
    monkeypatch.setenv("XDG_SESSION_ID", "7")
    monkeypatch.setenv("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/1/bus")
    assert screen._probe_locked() is None
    outputs["gdbus"] = "(false,)"
    assert screen._probe_locked() is False
    outputs["gdbus"] = "(true,)"
    assert screen._probe_locked() is True
