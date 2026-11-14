"""Opt-in cursor badge: a small click-through disc that shows where ``computer`` input lands.

XTEST moves the real pointer, but a person watching the desktop (or a recording) easily misses where a
click went. With ``[computer] cursor_badge = true`` or ``TWINSMITH_CURSOR_BADGE=1`` a small override-redirect
window glides to the target before each click, drag or scroll, follows a drag, and hides after a short
linger. It sits a few pixels below and right of the hotspot and has an empty input shape, so it never
covers the target pixel and the covered-window check never sees it. Kit draws its own badge for
``sim__ui__act`` (``exts/sim-tools/sim_tools/ui.py``) with the same hiding protocol.

Hidden from every screenshot (the pattern of agent-sh/computer-use-linux's indicator): a capture holds a
shared flock on ``capture.lock``. An overlay creates its flag file ``badge-<pid>-<kind>.shown`` (holding its
process's start time, so a reused PID is not mistaken for it) before it maps, hides while anyone holds that
lock (probed with a non-blocking exclusive flock) and deletes the flag once the hidden frame is on screen.
The capturer waits, bounded, until no live flag remains. Recordings do not take the lock, so a monitor
recording shows the badge (a window recording reads that window's own pixels, which this separate X11
window is not part of).
"""

from __future__ import annotations

import logging
import math
import os
import stat
import threading
import time
from contextlib import contextmanager
from pathlib import Path

try:
    import fcntl
except ImportError:  # the host tools still import on non-Unix platforms
    fcntl = None

from Xlib import X
from Xlib.ext import shape

log = logging.getLogger(__name__)

SIZE = 14            # badge diameter, pixels: well under the 80 px windows(list) floor, and untitled
OFFSET = 6           # its corner sits this far right of and below the hotspot, so it never covers the target
GLIDE_S = 0.15
GLIDE_STEPS = 10
LINGER_S = 1.5
POLL_S = 0.02
FRAME_S = 0.04       # after an unmap, time for the compositor to put the hidden frame on screen
CAPTURE_WAIT_S = 1.0
LOCK_NAME = "capture.lock"
_COLOR = (0xFFFF, 0x8C00, 0x0000)

_enabled = False
_badge: "_X11Badge | None" = None
_unavailable = ""    # why the X11 badge is off for this process
_note = ""           # that reason, said once in a tool answer


def configure(enabled: bool) -> None:
    """Set from the resolved configuration at server start."""
    global _enabled
    _enabled = bool(enabled)


def enabled() -> bool:
    return _enabled


def desktop_dir() -> Path:
    """Where the desktop input lock, the capture lock and the flag files live. Kit is told this folder
    with each act, so both sides use the same one whatever their environments say."""
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/twinsmith-desktop-{os.getuid()}"
    return Path(runtime) / "twinsmith-desktop"


def make_desktop_dir() -> Path:
    """desktop_dir(), made private together with the folder above it."""
    directory = desktop_dir()
    directory.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    directory.mkdir(mode=0o700, exist_ok=True)
    return directory


def trusted(directory: Path) -> bool:
    """A folder this account owns and nobody else can write to, in a folder of the same kind. Another
    local user could otherwise make the predictable /tmp folder first, then hold the lock, plant a flag
    or swap the folder, and stop every screenshot or let a badge into one."""
    try:
        infos = [os.lstat(directory), os.lstat(directory.parent)]
    except OSError:
        return False
    return all(stat.S_ISDIR(info.st_mode) and info.st_uid == os.getuid() and not info.st_mode & 0o022
               for info in infos)


def write_flag(path: Path) -> None:
    """The flag with its process's start time, put in place whole: a reader never sees it half written."""
    draft = path.with_suffix(".draft")
    draft.write_text(process_start(os.getpid()))
    os.replace(draft, path)


def _open_lock(directory: Path) -> int:
    return os.open(directory / LOCK_NAME, os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0), 0o600)


def capture_held(fd: int) -> bool:
    """Is a capture running? A non-blocking exclusive probe, released at once."""
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True
    fcntl.flock(fd, fcntl.LOCK_UN)
    return False


def process_start(pid: int) -> str:
    """When the process started (clock ticks since boot, from /proc), or "" if it is gone or there is no /proc."""
    try:
        return Path(f"/proc/{pid}/stat").read_bytes().rsplit(b")", 1)[1].split()[19].decode()
    except (OSError, IndexError, ValueError):
        return ""


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        pass
    return True


def _live_flags(directory: Path) -> list[Path]:
    """Flag files of badges still on screen. A flag whose process is gone, or whose PID now belongs to
    another process (the start time it holds is not that process's), is deleted."""
    live = []
    for path in directory.glob("badge-*.shown"):
        try:
            pid = int(path.name.split("-")[1])
            started = path.read_text().strip()
        except (IndexError, ValueError, OSError):
            continue
        if _alive(pid) and started == process_start(pid):
            live.append(path)
        else:
            path.unlink(missing_ok=True)
    return live


@contextmanager
def capture_hold(wait_s: float = CAPTURE_WAIT_S):
    """Hold around a screenshot's pixel grab: every cursor badge is hidden while it is held.

    With no badge ever shown on this desktop it adds no waiting. A badge that does not hide within
    ``wait_s`` fails the capture closed rather than return a picture with the badge in it."""
    directory = desktop_dir()
    if fcntl is None or not trusted(directory):  # no folder: no badge was ever shown on this desktop
        yield
        return
    fd = _open_lock(directory)
    try:
        deadline = time.monotonic() + wait_s
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
                break
            except BlockingIOError:  # an overlay is probing the lock at this instant
                if time.monotonic() > deadline:
                    raise RuntimeError("the cursor badge lock stayed busy, so no picture was taken; "
                                       "take a screenshot again") from None
                time.sleep(0.002)
        if _badge is not None:
            _badge.hide_for_capture(deadline)
        while True:
            live = _live_flags(directory)
            if not live:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"a cursor badge did not hide within {wait_s:g} s, so no picture was taken (it would have "
                    f"been in it). Its process is busy or stuck (an Isaac Sim that is loading or stepping hides "
                    f"its badge on its next frame): take a screenshot again in a few seconds. If that process "
                    f"shows no badge, delete its flag file: {', '.join(sorted(map(str, live)))}")
            time.sleep(0.01)
        yield
    finally:
        os.close(fd)  # releases the shared lock


def _open_display():
    from . import screen

    return screen._xdisplay()


def _disc(size: int) -> list[tuple[int, int, int, int]]:
    """A filled circle as one rectangle per row, for the bounding shape."""
    rects, r = [], size / 2
    for row in range(size):
        dy = row + 0.5 - r
        half = math.sqrt(max(r * r - dy * dy, 0.0))
        left = round(r - half)
        if size - 2 * left > 0:
            rects.append((left, row, size - 2 * left, 1))
    return rects


class _X11Badge:
    """One override-redirect window on its own X connection; every X call holds ``self.lock``."""

    def __init__(self):
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.shown = False
        self.pos: tuple[int, int] | None = None
        self.last = 0.0
        directory = desktop_dir()
        self.flag = directory / f"badge-{os.getpid()}-x11.shown"
        self.fd = None
        self.disp = _open_display()
        try:
            make_desktop_dir()
            if not trusted(directory):
                raise RuntimeError(f"{directory} is not a private folder of this account")
            self.fd = _open_lock(directory)
            if not self.disp.has_extension("SHAPE"):
                raise RuntimeError("the X server has no SHAPE extension, so the badge cannot be made click-through")
            scr = self.disp.screen()
            try:
                pixel = scr.default_colormap.alloc_color(*_COLOR).pixel
            except Exception:  # noqa: BLE001 - a colour is cosmetic
                pixel = scr.white_pixel
            self.window = scr.root.create_window(-2 * SIZE, -2 * SIZE, SIZE, SIZE, 0, scr.root_depth,
                                                 X.InputOutput, X.CopyFromParent, background_pixel=pixel,
                                                 override_redirect=True, event_mask=0)
            self.window.set_wm_class("twinsmith-cursor-badge", "Twinsmith")
            self.window.shape_rectangles(shape.SO.Set, shape.SK.Input, X.Unsorted, 0, 0, [])
            self.window.shape_rectangles(shape.SO.Set, shape.SK.Bounding, X.Unsorted, 0, 0, _disc(SIZE))
            self.disp.sync()
        except Exception:
            self.close()
            raise
        threading.Thread(target=self._watch, name="cursor-badge", daemon=True).start()

    def close(self) -> None:
        self.flag.unlink(missing_ok=True)
        try:
            self.disp.close()
        except Exception:  # noqa: BLE001
            pass
        if self.fd is not None:
            os.close(self.fd)

    def _move(self, x: int, y: int) -> None:
        self.window.configure(x=x + OFFSET, y=y + OFFSET, stack_mode=X.Above)
        self.disp.flush()
        self.pos, self.last = (x, y), time.monotonic()

    def _show(self) -> bool:
        """Flag, then probe, then map: a capturer that locks after the probe still sees the flag."""
        write_flag(self.flag)
        if capture_held(self.fd):
            self.flag.unlink(missing_ok=True)
            return False
        self.window.map()
        self.shown = True
        self.wake.set()
        return True

    def _hide(self) -> None:
        if not self.shown:
            return
        self.window.unmap()
        self.disp.sync()
        time.sleep(FRAME_S)
        self.shown = False
        self.flag.unlink(missing_ok=True)

    def glide(self, x: int, y: int) -> None:
        """Glide to root pixel (x, y). Call only while no server grab is held (this connection would block)."""
        with self.lock:
            if self.pos is None:
                pointer = self.disp.screen().root.query_pointer()
                self.pos = (pointer.root_x, pointer.root_y)
            start = self.pos
            if not self.shown:
                self._move(*start)
                if not self._show():
                    return  # a capture is running: stay hidden for this action
        for i in range(1, GLIDE_STEPS + 1):
            with self.lock:
                if not self.shown:
                    return
                if capture_held(self.fd):
                    self._hide()
                    return
                self._move(round(start[0] + (x - start[0]) * i / GLIDE_STEPS),
                           round(start[1] + (y - start[1]) * i / GLIDE_STEPS))
            time.sleep(GLIDE_S / GLIDE_STEPS)

    def follow(self, x: int, y: int) -> None:
        with self.lock:
            if self.shown and not capture_held(self.fd):
                self._move(x, y)

    def hide_for_capture(self, deadline: float) -> None:
        if self.lock.acquire(timeout=max(0.0, deadline - time.monotonic())):
            try:
                self._hide()
            finally:
                self.lock.release()

    def _watch(self) -> None:
        while True:
            self.wake.wait()
            time.sleep(POLL_S)
            try:
                with self.lock:
                    if not self.shown:
                        self.wake.clear()
                    elif time.monotonic() - self.last > LINGER_S or capture_held(self.fd):
                        self._hide()
                        self.wake.clear()
            except Exception as exc:  # noqa: BLE001 - the X connection broke: stop drawing
                _disable(f"the badge window failed ({exc})")
                return


def _disable(reason: str) -> None:
    global _badge, _unavailable, _note
    badge, _badge = _badge, None
    _unavailable = _note = reason
    log.warning("cursor badge off: %s", reason)
    if badge is not None:
        badge.close()


def glide(x: int, y: int) -> None:
    """Before input, with no server grab held: glide the badge to root pixel (x, y). Best effort: a badge
    failure turns the badge off for this process (said once by note()) and never stops the input."""
    global _badge
    if not _enabled or _unavailable:
        return
    try:
        if _badge is None:
            _badge = _X11Badge()
        _badge.glide(x, y)
    except Exception as exc:  # noqa: BLE001
        _disable(str(exc))


def follow(x: int, y: int) -> None:
    """Move the shown badge at once (a drag's motion, after the server grab is released)."""
    if _badge is None:
        return
    try:
        _badge.follow(x, y)
    except Exception as exc:  # noqa: BLE001
        _disable(str(exc))


def note() -> str:
    """Once per process: why the enabled badge is off, to append to a tool answer."""
    global _note
    text, _note = _note, ""
    return f" (cursor badge off: {text})" if text else ""
