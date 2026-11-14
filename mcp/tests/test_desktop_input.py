"""Focused checks for opt-in X11 input and its target guards (no display needed)."""

from contextlib import contextmanager, nullcontext
from types import SimpleNamespace

import pytest
from Xlib import X, XK
from mcp.server.fastmcp import FastMCP

from twinsmith import desktop_input as desktop


class Window:
    def __init__(self, wid, parent=None):
        self.id = wid
        self.parent = parent

    def query_tree(self):
        return SimpleNamespace(parent=self.parent)

    def query_pointer(self):
        return SimpleNamespace(child=None, mask=0)

    def get_attributes(self):
        return SimpleNamespace(map_state=X.IsViewable)

    def get_geometry(self):
        return SimpleNamespace(width=300, height=200)


class Display:
    def __init__(self):
        self.root = Window(1)
        self.window = Window(101, self.root)
        self.root.translate_coords = lambda *_: SimpleNamespace(x=100, y=100)
        self.pointer = (0, 0)
        self.covered = lambda x, y: False
        self.mask = 0
        self.keymap = [0] * 32
        self.focus = self.window
        self.grabbed = False
        self.closed = False
        self.events = []
        self.keyboard_owner = None  # another client's keyboard grab (lock screen, open menu)
        self.root.grab_keyboard = self.grab_keyboard

    def grab_keyboard(self, *args):
        return X.AlreadyGrabbed if self.keyboard_owner else X.GrabSuccess

    def ungrab_keyboard(self, *args):
        pass

    def screen(self):
        return SimpleNamespace(root=self.root, width_in_pixels=1000, height_in_pixels=800)

    def root_pointer(self):
        x, y = self.pointer
        inside = 100 <= x < 400 and 100 <= y < 300 and not self.covered(x, y)
        return SimpleNamespace(child=self.window if inside else None, mask=self.mask)

    def has_extension(self, name):
        return name == "XTEST"

    def grab_server(self):
        self.grabbed = True

    def ungrab_server(self):
        self.grabbed = False

    def sync(self):
        pass

    def close(self):
        self.closed = True

    def create_resource_object(self, kind, wid):
        assert kind == "window" and wid == self.window.id
        return self.window

    def query_keymap(self):
        return self.keymap

    def get_input_focus(self):
        return SimpleNamespace(focus=self.focus)

    def keysym_to_keycode(self, sym):
        return {ord("a"): 38, ord("A"): 38, ord("b"): 56,
                XK.string_to_keysym("Shift_L"): 50,
                XK.string_to_keysym("Control_L"): 37,
                XK.string_to_keysym("Return"): 36}.get(sym, 0)

    def keycode_to_keysym(self, code, index):
        return {38: (ord("a"), ord("A")), 56: (ord("b"), ord("B")),
                50: (XK.string_to_keysym("Shift_L"), 0),
                37: (XK.string_to_keysym("Control_L"), 0),
                36: (XK.string_to_keysym("Return"), 0)}[code][index]


@pytest.fixture
def desktop_setup(monkeypatch):
    disp = Display()
    disp.root.query_pointer = disp.root_pointer
    monkeypatch.setattr(desktop.screen, "_match_window", lambda query, wid: {"id": wid, "pid": 22})
    monkeypatch.setattr(desktop.screen, "_window_pid", lambda win, d: 22)
    monkeypatch.setattr(desktop.screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(desktop, "_desktop_lock", nullcontext)
    monkeypatch.setattr(desktop.time, "sleep", lambda _: None)
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: False)

    def fake_input(d, event, detail=0, **kwargs):
        d.events.append((event, detail, kwargs))
        if event == X.MotionNotify:
            d.pointer = (kwargs["x"], kwargs["y"])

    monkeypatch.setattr(desktop.xtest, "fake_input", fake_input)
    server = FastMCP("input-test")
    desktop.register_desktop_input_tools(server, groups={"computer"})
    funcs = {name: tool.fn for name, tool in server._tool_manager._tools.items()}
    return disp, funcs


def events(disp, event):
    return [detail for kind, detail, _ in disp.events if kind == event]


def test_click_and_scroll_send_bounded_events_to_target(desktop_setup):
    disp, tools = desktop_setup
    tools["computer_click"](101, 22, 20, 30, 300, 200, clicks=2)
    assert events(disp, X.ButtonPress) == [1, 1]
    assert events(disp, X.ButtonRelease) == [1, 1]
    assert disp.pointer == (120, 130) and disp.closed and not disp.grabbed
    tools["computer_scroll"](101, 22, 20, 30, 300, 200, "down", 3)
    assert events(disp, X.ButtonPress)[-3:] == [5, 5, 5]


def test_bad_geometry_or_covered_point_never_clicks(desktop_setup):
    disp, tools = desktop_setup
    with pytest.raises(ValueError, match="is 300x200, not the expected 299x200"):
        tools["computer_click"](101, 22, 20, 30, 299, 200)
    assert not disp.events and disp.closed and not disp.grabbed
    disp.covered = lambda x, y: True
    with pytest.raises(ValueError, match="covers"):
        tools["computer_click"](101, 22, 20, 30, 300, 200)
    assert not events(disp, X.ButtonPress)


def test_drag_checks_intermediate_points_before_pressing(desktop_setup):
    disp, tools = desktop_setup
    disp.covered = lambda x, y: 190 <= x <= 210
    with pytest.raises(ValueError, match="covers"):
        tools["computer_drag"](101, 22, 20, 30, 180, 30, 300, 200)
    assert not events(disp, X.ButtonPress)
    disp.covered = lambda x, y: False
    tools["computer_drag"](101, 22, 20, 30, 180, 30, 300, 200)
    assert events(disp, X.ButtonPress) == [1]
    assert events(disp, X.ButtonRelease) == [1]
    assert disp.pointer == (280, 130)


def test_typing_and_keys_require_focus_and_release_modifiers(desktop_setup):
    disp, tools = desktop_setup
    disp.focus = disp.root
    with pytest.raises(ValueError, match="focus"):
        tools["computer_type"](101, 22, "a")
    assert not disp.events
    disp.focus = disp.window
    tools["computer_type"](101, 22, "aA")
    assert events(disp, X.KeyPress) == [38, 50, 38]
    assert events(disp, X.KeyRelease) == [38, 38, 50]
    tools["computer_key"](101, 22, "a", ["ctrl"])
    assert events(disp, X.KeyPress)[-2:] == [37, 38]
    assert events(disp, X.KeyRelease)[-2:] == [38, 37]
    tools["computer_key"](101, 22, "A")
    assert events(disp, X.KeyPress)[-2:] == [50, 38]
    assert events(disp, X.KeyRelease)[-2:] == [38, 50]


def test_validation_rejects_unsupported_input_before_typing(desktop_setup):
    disp, tools = desktop_setup
    for value in ("", "é", "a\nb", "a" * 201, "a@"):
        with pytest.raises(ValueError):
            tools["computer_type"](101, 22, value)
    assert not events(disp, X.KeyPress)
    with pytest.raises(ValueError, match="modifiers"):
        tools["computer_key"](101, 22, "a", ["hyper"])
    with pytest.raises(ValueError, match="steps"):
        tools["computer_scroll"](101, 22, 0, 0, 300, 200, "down", 11)


def test_caps_lock_cannot_silently_change_typed_letters(desktop_setup):
    disp, tools = desktop_setup
    disp.mask = X.LockMask
    with pytest.raises(ValueError, match="Caps Lock"):
        tools["computer_type"](101, 22, "a")
    with pytest.raises(ValueError, match="Caps Lock"):
        tools["computer_key"](101, 22, "A")
    assert not events(disp, X.KeyPress)


def test_pid_change_and_active_input_abort(desktop_setup, monkeypatch):
    disp, tools = desktop_setup
    monkeypatch.setattr(desktop.screen, "_window_pid", lambda win, d: 23)
    with pytest.raises(ValueError, match="PID"):
        tools["computer_click"](101, 22, 1, 1, 300, 200)
    monkeypatch.setattr(desktop.screen, "_window_pid", lambda win, d: 22)
    disp.mask = X.Button1Mask
    with pytest.raises(ValueError, match="in use"):
        tools["computer_click"](101, 22, 1, 1, 300, 200)
    assert not disp.events and disp.closed and not disp.grabbed


def test_releases_button_and_server_on_injection_error(desktop_setup, monkeypatch):
    disp, tools = desktop_setup
    original = desktop.xtest.fake_input

    def broken(d, event, detail=0, **kwargs):
        original(d, event, detail, **kwargs)
        if event == X.ButtonPress:
            raise RuntimeError("injected failure")

    monkeypatch.setattr(desktop.xtest, "fake_input", broken)
    with pytest.raises(RuntimeError, match="injected failure"):
        tools["computer_click"](101, 22, 20, 30, 300, 200)
    assert events(disp, X.ButtonRelease) == [1]
    assert disp.closed and not disp.grabbed


def test_desktop_lock_rejects_other_cooperating_servers(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    with desktop._desktop_lock():
        with pytest.raises(RuntimeError, match="another MCP input"):
            with desktop._desktop_lock():
                pass


def test_desktop_input_reports_missing_unix_lock_support(monkeypatch):
    monkeypatch.setattr(desktop, "fcntl", None)
    with pytest.raises(RuntimeError, match="X11 Unix"):
        with desktop._desktop_lock():
            pass


def test_scroll_holds_one_lock_across_every_step(desktop_setup, monkeypatch):
    _, tools = desktop_setup
    calls = []

    @contextmanager
    def lock():
        calls.append("enter")
        try:
            yield
        finally:
            calls.append("exit")

    monkeypatch.setattr(desktop, "_desktop_lock", lock)
    tools["computer_scroll"](101, 22, 20, 30, 300, 200, "down", 3)
    assert calls == ["enter", "exit"]


def test_releases_pressed_keys_after_injection_error(desktop_setup, monkeypatch):
    disp, tools = desktop_setup
    original = desktop.xtest.fake_input

    def broken(d, event, detail=0, **kwargs):
        original(d, event, detail, **kwargs)
        if event == X.KeyPress and detail == 38:
            raise RuntimeError("injected key failure")

    monkeypatch.setattr(desktop.xtest, "fake_input", broken)
    with pytest.raises(RuntimeError, match="injected key failure"):
        tools["computer_key"](101, 22, "a", ["ctrl"])
    assert events(disp, X.KeyRelease) == [38, 37]
    assert disp.closed and not disp.grabbed


def test_a_locked_desktop_refuses_all_input(desktop_setup, monkeypatch):
    # Remote lab PC: the lock screen holds the grabs, so XTEST input (and typed text) would reach it.
    disp, tools = desktop_setup
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: True)
    for call in (lambda: tools["computer_click"](101, 22, 10, 10, 300, 200),
                 lambda: tools["computer_type"](101, 22, "secret"),
                 lambda: tools["computer_key"](101, 22, "enter"),
                 lambda: tools["computer_scroll"](101, 22, 10, 10, 300, 200, "down")):
        with pytest.raises(RuntimeError, match="sim__ui__act"):
            call()
    assert disp.events == []


def test_typing_is_refused_while_another_client_holds_the_keyboard(desktop_setup):
    disp, tools = desktop_setup
    disp.keyboard_owner = "menu"
    with pytest.raises(RuntimeError, match="keyboard grab"):
        tools["computer_type"](101, 22, "ab")
    assert events(disp, X.KeyPress) == []


def test_horizontal_scroll_and_alt_super_modifiers(desktop_setup):
    disp, tools = desktop_setup
    tools["computer_scroll"](101, 22, 10, 10, 300, 200, "left")
    tools["computer_scroll"](101, 22, 10, 10, 300, 200, "right")
    assert events(disp, X.ButtonPress) == [6, 7]
    with pytest.raises(ValueError, match="alt and super"):
        tools["computer_key"](101, 22, "a", ["hyper"])


def test_unknown_lock_state_fails_closed_when_the_keyboard_is_grabbed(desktop_setup, monkeypatch):
    disp, tools = desktop_setup
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: None)  # no GNOME screen saver answered
    disp.keyboard_owner = "locker"
    with pytest.raises(RuntimeError, match="could not tell whether the desktop is locked"):
        tools["computer_click"](101, 22, 10, 10, 300, 200)
    assert disp.events == []
    disp.keyboard_owner = None  # unknown but nobody holds the keyboard: an unlocked desktop
    tools["computer_click"](101, 22, 10, 10, 300, 200)
    assert events(disp, X.ButtonPress) == [1]


def test_the_lock_probe_runs_once_per_tool_call(desktop_setup, monkeypatch):
    disp, tools = desktop_setup
    calls = []
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: calls.append(1) or False)
    tools["computer_scroll"](101, 22, 10, 10, 300, 200, "down", 5)
    assert len(calls) == 1 and events(disp, X.ButtonPress) == [5] * 5


def test_answers_and_refusals_name_what_was_sent_and_what_is_accepted(desktop_setup):
    disp, tools = desktop_setup
    assert tools["computer_click"](101, 22, 20, 30, 300, 200, "right", 2).startswith("sent double right click to")
    assert tools["computer_click"](101, 22, 20, 30, 300, 200).startswith("sent click to window 101 at (20, 30)")
    assert tools["computer_key"](101, 22, "a", ["ctrl"]).startswith("sent key ctrl+a to")
    with pytest.raises(ValueError, match=r"\(300, 30\) is outside the 300x200 window"):
        tools["computer_click"](101, 22, 300, 30, 300, 200)
    with pytest.raises(ValueError, match="f1-f12 enter tab escape.*punctuation goes through action=type"):
        tools["computer_key"](101, 22, "esc")
    with pytest.raises(ValueError, match="key=enter"):
        tools["computer_type"](101, 22, "a\nb")
    disp.focus = disp.root
    with pytest.raises(ValueError, match="send the key to its parent window"):
        tools["computer_key"](101, 22, "escape")
