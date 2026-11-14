"""The opt-in cursor badge: configuration, the capture-lock protocol, the X11 badge and Kit's badge (all mocked)."""

import fcntl
import importlib.util
import os
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from PIL import Image as PILImage

from twinsmith import computer_use, config, cursor_badge, screen
from twinsmith import desktop_input as desktop
from test_desktop_input import desktop_setup  # noqa: F401 - the fake X display fixture

ROOT = Path(__file__).resolve().parents[2]
WATCH = cursor_badge._X11Badge._watch


@pytest.fixture(autouse=True)
def badge_env(monkeypatch, tmp_path):
    (tmp_path / "run").mkdir(mode=0o700)  # private, as /run/user/<uid> is
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    for name, value in (("_enabled", False), ("_badge", None), ("_unavailable", ""), ("_note", "")):
        monkeypatch.setattr(cursor_badge, name, value)
    monkeypatch.setattr(cursor_badge._X11Badge, "_watch", lambda self: None)  # no background thread
    return tmp_path / "run" / "twinsmith-desktop"


# ---------------------------------------------------------------- configuration

def test_flag_is_off_by_default_and_read_from_screen_table_and_env(tmp_path):
    home = tmp_path / "home"
    assert config.resolve(env={}, cwd=tmp_path, home=home).cursor_badge is False
    assert config.resolve(env={"TWINSMITH_CURSOR_BADGE": "1"}, cwd=tmp_path, home=home).cursor_badge is True
    (home / ".config/twinsmith").mkdir(parents=True)
    (home / ".config/twinsmith/config.toml").write_text("[computer]\ncursor_badge = true\n")
    assert config.resolve(env={}, cwd=tmp_path, home=home).cursor_badge is True
    (home / ".config/twinsmith/config.toml").write_text("[screen]\ncursor_badge = true\n")    # renamed, no alias
    with pytest.raises(ValueError, match=r"\[screen\] section is now \[computer\]"):
        config.resolve(env={}, cwd=tmp_path, home=home)
    (home / ".config/twinsmith/config.toml").write_text("[computer]\ncursor_badge = true\n")
    assert config.resolve(env={"TWINSMITH_CURSOR_BADGE": "off"}, cwd=tmp_path, home=home).cursor_badge is False
    with pytest.raises(ValueError, match="TWINSMITH_CURSOR_BADGE"):
        config.resolve(env={"TWINSMITH_CURSOR_BADGE": "maybe"}, cwd=tmp_path, home=home)


def test_server_passes_the_flag(monkeypatch, tmp_path):
    from twinsmith import server

    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("HOME", str(tmp_path))
    server.build_server(env={"TWINSMITH_CURSOR_BADGE": "1"})
    assert cursor_badge.enabled()
    server.build_server(env={})
    assert not cursor_badge.enabled()


def test_badge_args_tells_kit_the_hosts_folder_only_when_enabled(badge_env):
    params = {"action": "click", "x": 1, "y": 2}
    off = computer_use.badge_args(params, SimpleNamespace(config=config.Config()))
    assert off == params and "badge" not in off
    assert computer_use.badge_args(params, SimpleNamespace()) == params
    on = computer_use.badge_args(params, SimpleNamespace(config=config.Config(cursor_badge=True)))
    assert on == {**params, "badge": str(badge_env)} and "badge" not in params


# ---------------------------------------------------------------- capture lock protocol

def _flag(directory, pid, kind="kit"):
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)  # private, as the tools make it
    path = directory / f"badge-{pid}-{kind}.shown"
    path.write_text(cursor_badge.process_start(pid))  # as a shown badge writes it
    return path


def _sleeps(monkeypatch):
    calls = []
    real = time.sleep
    monkeypatch.setattr(cursor_badge.time, "sleep", lambda s: (calls.append(s), real(min(s, 0.005))))
    return calls


def test_capture_hold_adds_no_wait_when_idle(monkeypatch, badge_env):
    calls = _sleeps(monkeypatch)
    with cursor_badge.capture_hold():
        pass
    assert not badge_env.exists()  # never shown on this desktop: nothing created
    badge_env.mkdir(mode=0o700, parents=True)
    with cursor_badge.capture_hold():
        pass
    assert calls == []


def test_capture_hold_waits_for_an_overlay_to_hide(badge_env):
    flag = _flag(badge_env, os.getpid())
    fd = cursor_badge._open_lock(badge_env)
    seen = threading.Event()

    def overlay():  # what Kit's update loop does each frame
        while not cursor_badge.capture_held(fd):
            time.sleep(0.002)
        seen.set()
        time.sleep(0.03)  # the hidden frame is drawn
        flag.unlink()

    thread = threading.Thread(target=overlay)
    thread.start()
    with cursor_badge.capture_hold():
        assert seen.is_set() and not flag.exists()
        other = os.open(badge_env / cursor_badge.LOCK_NAME, os.O_RDWR)
        assert cursor_badge.capture_held(other)  # shared while capturing: no overlay may show
        os.close(other)
    thread.join()
    assert not cursor_badge.capture_held(fd)  # released afterwards
    os.close(fd)


def test_capture_hold_deletes_flags_of_dead_processes(monkeypatch, badge_env):
    proc = subprocess.Popen([sys.executable, "-c", "pass"])
    proc.wait()
    flag = _flag(badge_env, proc.pid)
    _flag(badge_env, "junk")  # unparseable names are ignored
    calls = _sleeps(monkeypatch)
    with cursor_badge.capture_hold():
        pass
    assert not flag.exists() and calls == []


def test_capture_hold_fails_closed_on_timeout(badge_env):
    flag = _flag(badge_env, os.getppid())  # alive, and never hides
    with pytest.raises(RuntimeError, match=rf"no picture was taken.*again in a few seconds.*{flag.name}"):
        with cursor_badge.capture_hold(wait_s=0.05):
            pytest.fail("captured while a badge was on screen")
    assert flag.exists()
    fd = cursor_badge._open_lock(badge_env)
    assert not cursor_badge.capture_held(fd)  # the lock was released on the error
    os.close(fd)


def test_screenshot_reads_pixels_inside_the_hold(monkeypatch, tmp_path):
    held = []

    @contextmanager
    def hold():
        held.append("in")
        yield
        held.append("out")

    def grab(win):
        assert held == ["in"]
        return PILImage.new("RGB", (40, 30))

    monkeypatch.setattr(screen.cursor_badge, "capture_hold", hold)
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path)
    monkeypatch.setattr(screen, "_match_window", lambda q, wid: dict(id=wid, title="t", x=0, y=0, width=40, height=30))
    monkeypatch.setattr(screen, "_grab_window_image", grab)
    screen.capture(window_id=5)
    assert held == ["in", "out"]


def test_kit_uses_the_folder_the_host_names(kit, badge_env, tmp_path, monkeypatch):
    """Not its own environment's: a server started without XDG_RUNTIME_DIR and a Kit with it would otherwise
    lock different folders, and the badge would be in the host's screenshots."""
    ui = kit.ui
    named = computer_use.badge_args({}, SimpleNamespace(config=config.Config(cursor_badge=True)))["badge"]
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "kits-own"))
    ui.badge(10, 10, named)
    first = ui._badge_state[0]
    assert first.flag.parent == badge_env and ui.BADGE_LOCK == cursor_badge.LOCK_NAME
    other = tmp_path / "other-host" / "twinsmith-desktop"
    other.parent.mkdir(mode=0o700)
    other.mkdir(mode=0o700)
    ui.badge(10, 10, str(other))  # another server asks: its lock is the one to watch now
    assert ui._badge_state[0].flag.parent == other and not list(badge_env.glob("*.shown"))
    held = {os.readlink(f"/proc/self/fd/{fd}") for fd in os.listdir("/proc/self/fd") if fd.isdigit()
            and os.path.exists(f"/proc/self/fd/{fd}")}
    assert str(badge_env / ui.BADGE_LOCK) not in held, "the first folder's lock file is not kept open"


def _folder(base, name="twinsmith-desktop", mode=0o700, parent_mode=0o700):
    (base / "p").mkdir()
    (base / "p").chmod(parent_mode)
    (base / "p" / name).mkdir()
    (base / "p" / name).chmod(mode)
    return base / "p" / name


@pytest.mark.parametrize("make", [
    lambda base: _folder(base, name="somewhere-else"),            # not a twinsmith-desktop folder
    lambda base: _folder(base, mode=0o777),                       # others can write to it
    lambda base: _folder(base, parent_mode=0o777),                # others can swap it: the /tmp fallback, squatted
    lambda base: base / "missing" / "twinsmith-desktop",          # Kit creates nothing from a bridge argument
    lambda base: "twinsmith-desktop",                             # relative
])
def test_kit_badge_stays_off_in_a_folder_that_is_not_private(kit, tmp_path, make):
    kit.ui.badge(10, 10, str(make(tmp_path)))
    assert kit.ui._badge_state == [None] and kit.fake.windows == [] and not (tmp_path / "missing").exists()


@pytest.mark.parametrize("loose", ["folder", "parent"])
def test_a_folder_others_can_write_to_is_not_used(badge_env, monkeypatch, loose):
    badge_env.mkdir()
    (badge_env if loose == "folder" else badge_env.parent).chmod(0o777)
    _flag(badge_env, os.getpid()).write_text(cursor_badge.process_start(os.getpid()))  # planted: would block
    with cursor_badge.capture_hold(wait_s=0.05):
        pass
    monkeypatch.setattr(cursor_badge, "_open_display", FakeBadgeDisplay)
    cursor_badge.configure(True)
    cursor_badge.glide(5, 5)
    assert "not a private folder" in cursor_badge.note()


# ---------------------------------------------------------------- X11 badge in desktop input

def test_flag_off_opens_no_badge_connection(monkeypatch, desktop_setup):  # noqa: F811
    disp, tools = desktop_setup
    monkeypatch.setattr(cursor_badge, "_open_display", lambda: pytest.fail("badge display opened"))
    out = tools["computer_click"](101, 22, 10, 20, 300, 200)
    assert out.startswith("sent click") and cursor_badge._badge is None


def test_glide_happens_before_the_server_grab(monkeypatch, desktop_setup):  # noqa: F811
    disp, tools = desktop_setup
    order = []
    monkeypatch.setattr(desktop.screen, "_match_window", lambda q, wid: {"id": wid, "pid": 22, "x": 100, "y": 100,
                                                                           "width": 300, "height": 200})
    monkeypatch.setattr(cursor_badge, "glide", lambda x, y: order.append(("glide", x, y, disp.grabbed)))
    monkeypatch.setattr(cursor_badge, "follow", lambda x, y: order.append(("follow", x, y, disp.grabbed)))
    grab = disp.grab_server
    monkeypatch.setattr(disp, "grab_server", lambda: (order.append(("grab",)), grab()))
    cursor_badge.configure(True)

    tools["computer_click"](101, 22, 10, 20, 300, 200, clicks=2)
    assert order == [("glide", 110, 120, False), ("grab",), ("grab",)]  # once: a double-click stays fast

    order.clear()
    tools["computer_scroll"](101, 22, 10, 20, 300, 200, direction="down", steps=3)
    assert order[:2] == [("glide", 110, 120, False), ("grab",)] and [o[0] for o in order].count("glide") == 1

    order.clear()
    tools["computer_drag"](101, 22, 10, 20, 70, 80, 300, 200)
    follows = [o for o in order if o[0] == "follow"]
    assert order[:2] == [("glide", 110, 120, False), ("grab",)]
    assert len(follows) == 12 and follows[-1][1:3] == (170, 180) and not any(o[3] for o in follows)


class FakeBadgeWindow:
    def __init__(self, log):
        self.log = log

    def configure(self, **kw):
        self.log.append(("configure", kw["x"], kw["y"]))

    def map(self):
        self.log.append(("map",))

    def unmap(self):
        self.log.append(("unmap",))

    def set_wm_class(self, *a):
        pass

    def shape_rectangles(self, op, kind, ordering, x, y, rects):
        self.log.append(("shape", kind, list(rects)))


class FakeBadgeDisplay:
    def __init__(self, shape=True):
        self.log, self.shape = [], shape
        self.window = FakeBadgeWindow(self.log)
        root = SimpleNamespace(create_window=lambda *a, **kw: (self.log.append(("create", kw)), self.window)[1],
                               query_pointer=lambda: SimpleNamespace(root_x=0, root_y=0))
        self.scr = SimpleNamespace(root=root, root_depth=24, white_pixel=1,
                                   default_colormap=SimpleNamespace(alloc_color=lambda *c: SimpleNamespace(pixel=7)))

    def has_extension(self, name):
        return self.shape and name == "SHAPE"

    def screen(self):
        return self.scr

    def sync(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.log.append(("close",))


def test_x11_badge_is_click_through_offset_and_hidden_for_captures(monkeypatch, badge_env):
    disp = FakeBadgeDisplay()
    monkeypatch.setattr(cursor_badge, "_open_display", lambda: disp)
    _sleeps(monkeypatch)
    cursor_badge.configure(True)
    cursor_badge.glide(500, 300)
    flag = badge_env / f"badge-{os.getpid()}-x11.shown"
    create = next(e for e in disp.log if e[0] == "create")[1]
    assert create["override_redirect"] is True
    assert ("shape", cursor_badge.shape.SK.Input, []) in disp.log  # empty input shape: click-through
    assert disp.log.index(("map",)) < [i for i, e in enumerate(disp.log) if e[0] == "configure"][-1]
    assert [e for e in disp.log if e[0] == "configure"][-1] == ("configure", 500 + cursor_badge.OFFSET,
                                                                300 + cursor_badge.OFFSET)
    assert cursor_badge.OFFSET > 0 and flag.exists()
    with cursor_badge.capture_hold():
        assert not flag.exists() and disp.log[-1] == ("unmap",)
    assert cursor_badge.note() == ""


def test_x11_badge_stays_hidden_while_a_capture_runs(monkeypatch, badge_env):
    disp = FakeBadgeDisplay()
    monkeypatch.setattr(cursor_badge, "_open_display", lambda: disp)
    badge_env.mkdir(mode=0o700, parents=True)
    fd = cursor_badge._open_lock(badge_env)
    fcntl.flock(fd, fcntl.LOCK_SH)  # a screenshot in another server
    cursor_badge.configure(True)
    cursor_badge.glide(5, 5)
    assert ("map",) not in disp.log and not list(badge_env.glob("*.shown"))
    os.close(fd)


def test_watcher_hides_for_another_servers_capture_and_after_the_linger(monkeypatch, badge_env):
    monkeypatch.setattr(cursor_badge._X11Badge, "_watch", WATCH)
    monkeypatch.setattr(cursor_badge, "POLL_S", 0.005)
    disp = FakeBadgeDisplay()
    monkeypatch.setattr(cursor_badge, "_open_display", lambda: disp)
    cursor_badge.configure(True)
    flag = badge_env / f"badge-{os.getpid()}-x11.shown"

    def hidden_within(seconds):
        deadline = time.monotonic() + seconds
        while flag.exists() and time.monotonic() < deadline:
            time.sleep(0.005)
        return not flag.exists() and disp.log[-1] == ("unmap",)

    cursor_badge.glide(5, 5)
    fd = cursor_badge._open_lock(badge_env)
    fcntl.flock(fd, fcntl.LOCK_SH)  # another server's screenshot: no direct hide, only the lock
    assert hidden_within(0.5)
    os.close(fd)
    monkeypatch.setattr(cursor_badge, "LINGER_S", 0.05)
    cursor_badge.glide(9, 9)
    assert flag.exists() and hidden_within(0.5)


def test_no_shape_extension_turns_the_badge_off_and_says_why(monkeypatch, badge_env):
    disp = FakeBadgeDisplay(shape=False)
    opened = []
    monkeypatch.setattr(cursor_badge, "_open_display", lambda: (opened.append(1), disp)[1])
    cursor_badge.configure(True)
    cursor_badge.glide(5, 5)
    cursor_badge.glide(6, 6)
    assert opened == [1] and ("close",) in disp.log and not any(e[0] == "create" for e in disp.log)
    assert "SHAPE" in cursor_badge.note() and cursor_badge.note() == ""


# ---------------------------------------------------------------- Kit badge

def _ui_module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "exts/sim-tools/sim_tools/ui.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class KitWindow:
    def __init__(self, title, docked=False, rect=(0, 0, 500, 500), **kw):
        self.title, self.visible, self.docked = title, kw.get("visible", True), docked
        self.position_x, self.position_y, self.width, self.height = rect

    def is_selected_in_dock(self):
        return True


@pytest.fixture
def kit(monkeypatch):
    cursor_badge.make_desktop_dir()  # the host makes the folder it names to Kit
    ui = _ui_module("sim_tools_ui_badge")
    fake = ModuleType("omni.ui")
    fake.windows = []
    fake.Workspace = SimpleNamespace(get_dpi_scale=lambda: 2.0, get_windows=lambda: fake.windows,
                                     get_window=lambda t: next((w for w in fake.windows if w.title == t), None))

    class Frame:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    class Window(KitWindow):
        def __init__(self, title, width, height, flags, visible):
            super().__init__(title, rect=(0, 0, width, height), visible=visible)
            self.flags, self.frame = flags, Frame()
            fake.windows.append(self)

    fake.Window, fake.Circle = Window, lambda **kw: None
    fake.WINDOW_FLAGS_NO_MOUSE_INPUTS, fake.WINDOW_FLAGS_NO_FOCUS_ON_APPEARING = 1, 2
    fake.WINDOW_FLAGS_NO_TITLE_BAR = 4  # the other cosmetic flags are missing: getattr fallback
    subs = []
    stream = SimpleNamespace(create_subscription_to_pop=lambda fn, name: (subs.append(fn), object())[1])
    app = ModuleType("omni.kit.app")
    app.get_app = lambda: SimpleNamespace(get_update_event_stream=lambda: stream)
    omni, omni_kit = ModuleType("omni"), ModuleType("omni.kit")
    omni.ui, omni.kit, omni_kit.app = fake, omni_kit, app
    for key, mod in (("omni", omni), ("omni.ui", fake), ("omni.kit", omni_kit), ("omni.kit.app", app)):
        monkeypatch.setitem(sys.modules, key, mod)
    return SimpleNamespace(ui=ui, fake=fake, subs=subs)


def test_kit_badge_is_left_out_of_windows_and_never_covers_a_widget(kit):
    ui, fake = kit.ui, kit.fake
    fake.windows[:] = [KitWindow("Stage", docked=True), KitWindow(ui.BADGE_TITLE, rect=(8, 8, 14, 14))]
    assert [w["title"] for w in ui.windows()] == ["Stage"]
    assert ui._shown("Stage//Frame/Button[0]", (10, 10), None) is None
    fake.windows.append(KitWindow("Floating", rect=(0, 0, 50, 50)))
    assert "Floating" in ui._shown("Stage//Frame/Button[0]", (10, 10), None)


def test_kit_badge_glides_then_hides_for_a_capture(kit, badge_env):
    ui = kit.ui
    ui.badge(100, 60, str(badge_env))
    badge = ui._badge_state[0]
    flag = badge_env / f"badge-{os.getpid()}-kit.shown"
    assert badge.window.visible and flag.exists() and badge.window.flags == 7
    assert badge.end == (50 + ui.BADGE_OFFSET, 30 + ui.BADGE_OFFSET)  # window pixels / dpi, off the hotspot
    badge.t0 -= 1.0  # the glide is over
    kit.subs[0]()
    assert (badge.window.position_x, badge.window.position_y) == badge.end
    fd = cursor_badge._open_lock(badge_env)
    fcntl.flock(fd, fcntl.LOCK_SH)  # a host screenshot holds the capture lock
    kit.subs[0]()
    assert not badge.window.visible and flag.exists()  # hidden on the frame that saw the lock...
    kit.subs[0]()
    kit.subs[0]()
    assert not flag.exists() and badge.sub is None  # ...and the flag goes once that frame is drawn
    os.close(fd)
    ui.badge(10, 10, str(badge_env))  # the next act shows it again
    assert badge.window.visible and flag.exists()


def test_kit_badge_failure_never_stops_an_act(kit, monkeypatch, badge_env):
    ui = kit.ui
    monkeypatch.setattr(ui, "_KitBadge", lambda directory: (_ for _ in ()).throw(RuntimeError("no omni.ui")))
    ui.badge(1, 1, str(badge_env))
    ui.badge(2, 2, str(badge_env))
    assert ui._badge_state == [None]


def test_a_kit_badge_that_fails_while_shown_leaves_no_flag_to_block_screenshots(kit, badge_env):
    ui = kit.ui
    ui.badge(10, 10, str(badge_env))
    shown = ui._badge_state[0]
    assert shown.window.visible and list(badge_env.glob("*.shown"))

    def broken(*args):
        raise RuntimeError("window gone")

    shown.glide_to = broken
    ui.badge(20, 20, str(badge_env))
    assert ui._badge_state == [None] and not shown.window.visible and shown.sub is None
    assert not list(badge_env.glob("*.shown")), "a live Kit's flag would fail every later screenshot"


def test_kit_badge_stays_off_without_the_no_input_flags(kit, badge_env):
    ui = kit.ui
    del kit.fake.WINDOW_FLAGS_NO_MOUSE_INPUTS
    ui.badge(1, 1, str(badge_env))
    assert ui._badge_state == [None] and kit.fake.windows == [] and not list(badge_env.glob("*.shown"))


# ---------------------------------------------------------------- stale flags, and where the badge may go


@pytest.mark.parametrize("content", ["1", ""])  # another process's start time; a flag left by an old version
def test_a_flag_whose_pid_was_reused_is_deleted(badge_env, content):
    stale = _flag(badge_env, os.getpid())
    stale.write_text(content)
    with cursor_badge.capture_hold(wait_s=0.05):
        pass
    assert not stale.exists()
    live = _flag(badge_env, os.getpid())
    live.write_text(cursor_badge.process_start(os.getpid()))
    with pytest.raises(RuntimeError, match="did not hide"):
        with cursor_badge.capture_hold(wait_s=0.05):
            pytest.fail("captured while a badge was on screen")
    assert cursor_badge.process_start(os.getpid()).isdigit() and cursor_badge.process_start(2 ** 22 + 7) == ""


def test_a_shown_badge_writes_its_start_time(monkeypatch, badge_env, kit):
    monkeypatch.setattr(cursor_badge, "_open_display", FakeBadgeDisplay)
    cursor_badge.configure(True)
    cursor_badge.glide(5, 5)
    kit.ui.badge(10, 10, str(badge_env))
    started = cursor_badge.process_start(os.getpid())
    assert {path.name.split("-")[2]: path.read_text() for path in badge_env.glob("*.shown")} == {
        "x11.shown": started, "kit.shown": started}


@pytest.mark.parametrize("call, why", [
    (lambda tools: tools["computer_click"](101, 22, 300, 20, 300, 200), "outside"),
    (lambda tools: tools["computer_click"](101, 22, 10, 20, 299, 200), "not the expected"),
])
def test_the_badge_does_not_glide_to_a_point_that_is_refused(monkeypatch, desktop_setup, call, why):  # noqa: F811
    _, tools = desktop_setup
    glides = []
    monkeypatch.setattr(desktop.screen, "_match_window", lambda q, wid: {"id": wid, "pid": 22, "x": 100, "y": 100,
                                                                           "width": 300, "height": 200})
    monkeypatch.setattr(cursor_badge, "glide", lambda x, y: glides.append((x, y)))
    cursor_badge.configure(True)
    with pytest.raises(ValueError, match=why):
        call(tools)
    assert glides == []


def test_the_badge_is_not_drawn_over_a_possible_lock_screen(monkeypatch, desktop_setup):  # noqa: F811
    disp, tools = desktop_setup
    glides = []
    monkeypatch.setattr(desktop.screen, "_match_window", lambda q, wid: {"id": wid, "pid": 22, "x": 100, "y": 100,
                                                                           "width": 300, "height": 200})
    monkeypatch.setattr(cursor_badge, "glide", lambda x, y: glides.append((x, y)))
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: None)  # no screen saver answered
    cursor_badge.configure(True)
    disp.keyboard_owner = "lock screen"
    with pytest.raises(RuntimeError, match="could not tell whether the desktop is locked"):
        tools["computer_click"](101, 22, 10, 20, 300, 200)
    assert glides == [] and not disp.events
    disp.keyboard_owner = None  # unknown, but nothing holds the keyboard: the click goes, and so does the badge
    tools["computer_click"](101, 22, 10, 20, 300, 200)
    assert glides == [(110, 120)]


def test_a_kit_badge_asked_to_show_while_it_hides_keeps_its_flag(kit, badge_env):
    ui = kit.ui
    ui.badge(10, 10, str(badge_env))
    badge = ui._badge_state[0]
    fd = cursor_badge._open_lock(badge_env)
    fcntl.flock(fd, fcntl.LOCK_SH)  # a host screenshot
    kit.subs[0]()  # hidden, but that frame is not drawn yet
    ui.badge(20, 20, str(badge_env))  # the next act asks for it during the capture
    assert not badge.window.visible and badge.flag.exists(), "the capture must still wait for the hidden frame"
    kit.subs[0]()
    kit.subs[0]()
    assert not badge.flag.exists()
    os.close(fd)
