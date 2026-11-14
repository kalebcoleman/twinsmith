"""Explicitly enabled, window-targeted X11 desktop input.

XTEST input is ultimately sent to a shared desktop, not to a private application
sandbox. The checks here reduce accidental mis-targeting; they cannot make X11
safe against another client or a person changing the desktop concurrently.
"""

from __future__ import annotations

import hashlib
import os
import threading
import time
from contextlib import contextmanager
from typing import Annotated, Any, Callable, Iterable

from pydantic import Field

try:
    import fcntl
except ImportError:  # The standard host tools can still import on non-Unix platforms.
    fcntl = None

from Xlib import X, XK
from Xlib.ext import xtest
from mcp.server.fastmcp import FastMCP, Image
from mcp.types import CallToolResult, TextContent

from . import cursor_badge, screen
from .local_tools import DESKTOP_INPUT, ScopedTools

_input_lock = threading.Lock()
_WindowId = Annotated[int, Field(description="Window id from list_windows.")]
_Pid = Annotated[int, Field(description="The window's PID from list_windows; a different owner is refused.")]
_X = Annotated[int, Field(description="X in window pixels (or image pixels, see coord_space), from a fresh screenshot.")]
_Y = Annotated[int, Field(description="Y in window pixels (or image pixels, see coord_space), from a fresh screenshot.")]
_Width = Annotated[int, Field(description="The window's own width in pixels (windows list), not a downscaled image's; "
                                         "it ends a coord_space token (/WxH). A changed size is refused.")]
_Height = Annotated[int, Field(description="The window's own height in pixels, as for expected_width.")]
_CoordSpace = Annotated[str, Field(description="window (window pixels) or the coord_space token a screenshot "
                                               "printed (x/y are then pixels of that image or crop).")]
_ShotAfter = Annotated[bool, Field(description="Also return a screenshot of the window after the action.")]
_Steps = Annotated[list[dict[str, Any]], Field(
    min_length=1, max_length=10,
    description="1-10 steps run in order, each an object with action and that action's own arguments. "
                "E.g. [{\"action\": \"click\", \"x\": 10, \"y\": 20}, {\"action\": \"type\", \"text\": \"hi\"}].",
    json_schema_extra={"items": {
        "type": "object", "required": ["action"],
        "properties": {"action": {"type": "string", "enum": ["click", "drag", "scroll", "type", "key"]}},
        "description": "click: x, y, button, clicks; drag: x, y, end_x, end_y; scroll: x, y, direction, steps; "
                       "type: text; key: key, modifiers. x/y in coord_space."}})]

_MAX_TEXT = 200
_MAX_SCROLL = 10
_BUTTONS = {"left": 1, "middle": 2, "right": 3}
_KEYS = {
    "enter": "Return", "tab": "Tab", "escape": "Escape", "backspace": "BackSpace",
    "delete": "Delete", "left": "Left", "right": "Right", "up": "Up", "down": "Down",
    "home": "Home", "end": "End", "pageup": "Page_Up", "pagedown": "Page_Down",
    "space": "space",
    **{f"f{i}": f"F{i}" for i in range(1, 13)},
}
_KEY_NAMES = "f1-f12 enter tab escape backspace delete left right up down home end pageup pagedown space"
_MODIFIERS = {"ctrl": "Control_L", "shift": "Shift_L", "alt": "Alt_L", "super": "Super_L"}
_LOCKED = ("the desktop is locked or its screen saver is active (the PC is probably used remotely): X input "
           "would go to the lock screen, typed text into its password box, not to the window. For Isaac Sim "
           "use sim__ui__find and sim__ui__act, which drive Kit's own input; window screenshots still work.")
_BUSY_MASK = (X.ShiftMask | X.ControlMask | X.Mod1Mask | X.Mod3Mask | X.Mod4Mask | X.Mod5Mask
              | X.Button1Mask | X.Button2Mask | X.Button3Mask | X.Button4Mask | X.Button5Mask)


@contextmanager
def _desktop_lock():
    """Serialize cooperating MCP servers on this user's X display."""
    if fcntl is None:
        raise RuntimeError("desktop input requires a supported X11 Unix environment")
    directory = cursor_badge.make_desktop_dir()
    name = hashlib.sha256(screen.DISPLAY.encode()).hexdigest()[:16] + ".lock"
    flags = os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(directory / name, flags, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another MCP input action is using this desktop; retry") from exc
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


_call_lock_state: list = []  # the lock probe's answer, once per tool call or batch step


@contextmanager
def _action_lock():
    """Hold both locks across an entire action, including multi-step inputs."""
    with _input_lock, _desktop_lock():
        _call_lock_state.clear()
        try:
            yield
        finally:
            _call_lock_state.clear()


def _lock_state():
    """screen._probe_locked(), probed fresh once per tool call (a 10-step scroll probes once)."""
    if not _call_lock_state:
        _call_lock_state.append(screen._probe_locked())
    return _call_lock_state[0]


def _id(window) -> int:
    return int(getattr(window, "id", window or 0))


def _belongs_to(window, target_id: int, root_id: int) -> bool:
    """Is an X window inside the selected client (including the client itself)?"""
    seen: set[int] = set()
    while window and _id(window) not in seen:
        wid = _id(window)
        if wid == target_id:
            return True
        if wid == root_id or not hasattr(window, "query_tree"):
            break
        seen.add(wid)
        window = window.query_tree().parent
    return False


def _window_geometry(disp, window, expected_pid: int) -> tuple[int, int, int, int]:
    if window.get_attributes().map_state != X.IsViewable:
        raise ValueError("target window is no longer visible")
    pid = screen._window_pid(window, disp)
    if pid is None or pid != expected_pid:
        raise ValueError("target window PID is missing or changed; list windows again")
    geom = window.get_geometry()
    root = disp.screen().root
    pos = root.translate_coords(window, 0, 0)
    return pos.x, pos.y, geom.width, geom.height


def _check_idle(disp) -> None:
    if disp.screen().root.query_pointer().mask & _BUSY_MASK or any(disp.query_keymap()):
        raise ValueError("keyboard or mouse buttons are in use; release them and retry")


def _keyboard_grabbed(disp) -> bool:
    """True when another client holds the keyboard grab (a lock screen; also an open popup menu).

    Probed by grabbing and releasing at once; nothing is typed."""
    root = disp.screen().root
    status = root.grab_keyboard(False, X.GrabModeAsync, X.GrabModeAsync, X.CurrentTime)
    if status == X.GrabSuccess:
        disp.ungrab_keyboard(X.CurrentTime)
        disp.sync()
        return False
    return True  # AlreadyGrabbed, or GrabFrozen (another client froze the keyboard)


@contextmanager
def _target(window_id: int, expected_pid: int, badge_at: tuple[int, int, int, int] | None = None):
    """The checked target window under a server grab. ``badge_at`` (x, y and the expected window size) is
    where the cursor badge glides first, when the input can follow it there."""
    if window_id <= 0 or expected_pid <= 0:
        raise ValueError("window_id and expected_pid must be positive")
    locked = _lock_state()  # fresh for this call: it guards where keystrokes go
    if locked:
        raise RuntimeError(_LOCKED)
    # Require a fresh, titled top-level window, not an arbitrary X resource ID.
    visible = screen._match_window(None, window_id)
    if visible.get("pid") != expected_pid:
        raise ValueError("target window PID is missing or changed; list windows again")
    disp = screen._xdisplay()
    grabbed = False
    try:
        if not disp.has_extension("XTEST"):
            raise RuntimeError("the X11 display does not support the XTEST extension")
        if badge_at is not None and cursor_badge.enabled():
            x, y, width, height = badge_at
            # Not for a point the checks below refuse (outside, or a resized window), and not over a
            # possible lock screen: with the lock state unknown, a held keyboard grab refuses the input.
            if ((visible["width"], visible["height"]) == (width, height) and 0 <= x < width and 0 <= y < height
                    and (locked is False or not _keyboard_grabbed(disp))):
                # Before the server grab: the badge's own X connection would block under it.
                cursor_badge.glide(visible["x"] + x, visible["y"] + y)
        # Keep another X client from moving/replacing the target between the
        # final checks and injection. This is brief, and is not an X11 security boundary.
        disp.grab_server()
        grabbed = True
        disp.sync()
        if locked is None and _keyboard_grabbed(disp):
            # Fail closed: the lock state is unknown (no GNOME screen saver answered) and another
            # client holds the keyboard, which is what a lock screen does.
            raise RuntimeError("could not tell whether the desktop is locked, and another client holds the "
                               "keyboard grab (a lock screen or an open menu), so input could go there. "
                               "If it is a menu, wait for it to close and capture again; for Isaac Sim use "
                               "sim__ui__act, which does not go through the desktop.")
        window = disp.create_resource_object("window", window_id)
        geometry = _window_geometry(disp, window, expected_pid)
        _check_idle(disp)
        yield disp, window, geometry
        disp.sync()
    finally:
        try:
            if grabbed:
                disp.ungrab_server()
                disp.sync()
        finally:
            disp.close()


def _point(disp, window, geometry, x: int, y: int, width: int, height: int) -> None:
    left, top, actual_width, actual_height = geometry
    if width != actual_width or height != actual_height:
        raise ValueError(f"the window is {actual_width}x{actual_height}, not the expected {width}x{height}: pass the "
                         "window's own size (windows list, or the /WxH end of a coord_space token), not a "
                         "downscaled image's; if it was resized, capture it again")
    if not (0 <= x < width and 0 <= y < height):
        raise ValueError(f"({x}, {y}) is outside the {width}x{height} window; x and y are window pixels unless "
                         "coord_space names a screenshot")
    root = disp.screen().root
    root_x, root_y = left + x, top + y
    if not (0 <= root_x < disp.screen().width_in_pixels and
            0 <= root_y < disp.screen().height_in_pixels):
        raise ValueError("coordinates are outside the visible X11 desktop")
    xtest.fake_input(disp, X.MotionNotify, root=root.id, x=root_x, y=root_y)
    disp.sync()
    # Look through the actual stacking order after moving. Covered windows and
    # unrelated dialogs must not receive a click intended for this target.
    leaf = root
    while True:
        child = leaf.query_pointer().child
        if not child or not hasattr(child, "query_pointer"):
            break
        leaf = child
    if not _belongs_to(leaf, window.id, root.id):
        raise ValueError("another window covers that point; uncover the target (windows action=raise) and "
                         "capture again")


def _focus(disp, window) -> None:
    root = disp.screen().root
    focus = disp.get_input_focus().focus
    if not _belongs_to(focus, window.id, root.id):
        raise ValueError("target window does not have keyboard focus; click it and try again (for an open "
                         "menu or popup, send the key to its parent window)")


def _lookup(disp, symbol: int) -> tuple[int, bool]:
    code = disp.keysym_to_keycode(symbol)
    if not code:
        raise ValueError("the current keyboard layout cannot produce that key")
    if disp.keycode_to_keysym(code, 0) == symbol:
        return code, False
    if disp.keycode_to_keysym(code, 1) == symbol:
        return code, True
    raise ValueError("the current keyboard layout needs an unsupported modifier for that key")


def _send_key(disp, code: int, modifiers: list[int]) -> None:
    """Hold the key across a frame while allowing the target app to process input."""
    pressed: list[int] = []
    try:
        for keycode in [*modifiers, code]:
            pressed.append(keycode)
            xtest.fake_input(disp, X.KeyPress, keycode)
        disp.sync()
        disp.ungrab_server()
        disp.sync()
        time.sleep(0.08)
    finally:
        for keycode in reversed(pressed):
            xtest.fake_input(disp, X.KeyRelease, keycode)
        disp.sync()


def _click(disp, button: int) -> None:
    """Let a rendered app see the press for at least one frame before release."""
    try:
        xtest.fake_input(disp, X.ButtonPress, button)
        disp.sync()
        # The target was checked and the press was sent under the X server grab.
        # Release the grab so Kit can process the event and render the next frame.
        disp.ungrab_server()
        disp.sync()
        time.sleep(0.08)
    finally:
        xtest.fake_input(disp, X.ButtonRelease, button)
        disp.sync()


def _check_click(button: str, clicks: int) -> None:
    if button not in _BUTTONS or clicks not in (1, 2):
        raise ValueError("button must be left, middle or right; clicks must be 1 or 2")


def _scroll_button(direction: str, steps: int) -> int:
    buttons = {"up": 4, "down": 5, "left": 6, "right": 7}
    if direction not in buttons or not 1 <= steps <= _MAX_SCROLL:
        raise ValueError("direction must be up, down, left or right and steps must be 1–10")
    return buttons[direction]


def _check_text(text: str) -> None:
    if not text or len(text) > _MAX_TEXT or any(not 32 <= ord(c) <= 126 for c in text):
        raise ValueError("text must contain 1–200 printable ASCII characters; send a new line or a tab with "
                         "action=key (key=enter, key=tab)")


def _key_symbol(key: str, modifiers: list[str]) -> int:
    if len(set(modifiers)) != len(modifiers) or any(m not in _MODIFIERS for m in modifiers):
        raise ValueError("modifiers may contain ctrl, shift, alt and super, at most once each")
    if key.lower() in _KEYS:
        return XK.string_to_keysym(_KEYS[key.lower()])
    if len(key) == 1 and key.isascii() and key.isalnum():
        return ord(key)
    raise ValueError(f"unsupported key; use a letter, a digit, or one of: {_KEY_NAMES} (punctuation goes "
                     "through action=type)")


def _step_click(window_id: int, pid: int, x: int, y: int, width: int, height: int,
                button: str = "left", clicks: int = 1) -> str:
    _check_click(button, clicks)
    for i in range(clicks):  # glide once: a pause between the clicks would break a double-click
        with _target(window_id, pid, None if i else (x, y, width, height)) as (disp, window, geom):
            _point(disp, window, geom, x, y, width, height)
            _click(disp, _BUTTONS[button])
    kind = ("double " if clicks == 2 else "") + ("" if button == "left" else f"{button} ")
    return f"sent {kind}click to window {window_id} at ({x}, {y}){cursor_badge.note()}"


def _step_drag(window_id: int, pid: int, x: int, y: int, end_x: int, end_y: int, width: int, height: int) -> str:
    with _target(window_id, pid, (x, y, width, height)) as (disp, window, geom):
        # Check the entire sampled path before pressing, including any covered areas.
        for i in range(1, 13):
            px = round(x + (end_x - x) * i / 12)
            py = round(y + (end_y - y) * i / 12)
            _point(disp, window, geom, px, py, width, height)
        _point(disp, window, geom, x, y, width, height)
        root = disp.screen().root
        try:
            xtest.fake_input(disp, X.ButtonPress, 1)
            disp.sync()
            disp.ungrab_server()
            disp.sync()
            time.sleep(0.08)
            for i in range(1, 13):
                px = round(x + (end_x - x) * i / 12) + geom[0]
                py = round(y + (end_y - y) * i / 12) + geom[1]
                xtest.fake_input(disp, X.MotionNotify, root=root.id, x=px, y=py)
                disp.sync()
                cursor_badge.follow(px, py)
                time.sleep(0.01)
        finally:
            xtest.fake_input(disp, X.ButtonRelease, 1)
            disp.sync()
    return f"sent drag in window {window_id} from ({x}, {y}) to ({end_x}, {end_y}){cursor_badge.note()}"


def _step_scroll(window_id: int, pid: int, x: int, y: int, width: int, height: int,
                 direction: str, steps: int = 1) -> str:
    button = _scroll_button(direction, steps)
    for i in range(steps):
        with _target(window_id, pid, None if i else (x, y, width, height)) as (disp, window, geom):
            _point(disp, window, geom, x, y, width, height)
            _click(disp, button)
    return f"sent {steps} scroll step(s) {direction} to window {window_id}{cursor_badge.note()}"


def _step_type(window_id: int, pid: int, text: str) -> str:
    _check_text(text)
    with _target(window_id, pid) as (disp, window, _):
        if _keyboard_grabbed(disp):  # a menu, or a lock screen the lock probe did not detect
            raise RuntimeError("another client holds the keyboard grab (an open menu or a lock screen), so "
                               "typed text would go there, not to the window. Close the menu "
                               "(computer action=key, key=escape) or unlock the desktop, then capture again.")
        _focus(disp, window)
        if disp.screen().root.query_pointer().mask & X.LockMask:
            raise ValueError("turn off Caps Lock before typing")
        keys = [_lookup(disp, ord(char)) for char in text]
        shift, _ = _lookup(disp, XK.string_to_keysym("Shift_L"))
        sent = 0
        try:
            for code, shifted in keys:
                if sent:
                    disp.grab_server()
                    disp.sync()
                    _window_geometry(disp, window, pid)
                    _check_idle(disp)
                    _focus(disp, window)
                _send_key(disp, code, [shift] if shifted else [])
                sent += 1
        except Exception as exc:
            raise RuntimeError(f"typing stopped after {sent} character(s): {exc}") from exc
    return f"sent {len(text)} character(s) to window {window_id}"


def _step_key(window_id: int, pid: int, key: str, modifiers: list[str] | None = None) -> str:
    modifiers = modifiers or []
    symbol = _key_symbol(key, modifiers)
    with _target(window_id, pid) as (disp, window, _):
        _focus(disp, window)
        if len(key) == 1 and key.isascii() and key.isalpha() and (
                disp.screen().root.query_pointer().mask & X.LockMask):
            raise ValueError("turn off Caps Lock before sending a letter")
        code, shifted = _lookup(disp, symbol)
        modifier_codes = [_lookup(disp, XK.string_to_keysym(_MODIFIERS[m]))[0]
                          for m in modifiers]
        if shifted and "shift" not in modifiers:
            modifier_codes.append(_lookup(disp, XK.string_to_keysym("Shift_L"))[0])
        _send_key(disp, code, modifier_codes)
    return f"sent key {'+'.join([*modifiers, key])} to window {window_id}"


def _after(window_id: int, text: str, screenshot_after: bool):
    """The action's answer, and with screenshot_after the window once it has reacted (taken after every
    lock and server grab is released, with the same capture as the screenshot tool). A capture that fails
    leaves a note beside the answer instead."""
    if not screenshot_after:
        return f"{text}; verify the resulting UI"
    try:
        return [text, *screen.capture(window_id=window_id, settle=True)]
    except Exception as error:  # noqa: BLE001 - the input was sent; its answer must not be lost
        return f"{text}; screenshot: not taken ({error or type(error).__name__}); verify the resulting UI"


# One batch step: action -> (required, optional) arguments, and every argument's type.
_BATCH = {"click": (("x", "y"), ("button", "clicks")), "drag": (("x", "y", "end_x", "end_y"), ()),
          "scroll": (("x", "y", "direction"), ("steps",)), "type": (("text",), ()), "key": (("key",), ("modifiers",))}
_TYPES = {"x": int, "y": int, "end_x": int, "end_y": int, "clicks": int, "steps": int, "button": str,
          "direction": str, "text": str, "key": str, "modifiers": list}
_STEPS = {"click": _step_click, "drag": _step_drag, "scroll": _step_scroll, "type": _step_type, "key": _step_key}


def _batch_plan(steps: list, coord_space: str, width: int, height: int) -> list[tuple[str, dict]]:
    """Every step checked, and its points mapped to window pixels, before anything is sent."""
    if not isinstance(steps, list) or not 1 <= len(steps) <= 10:
        raise ValueError("actions must hold 1-10 steps; nothing was sent")
    plan = []
    for i, step in enumerate(steps):
        try:
            if not isinstance(step, dict) or step.get("action") not in _BATCH:
                raise ValueError("action must be click, drag, scroll, type or key")
            name = step["action"]
            required, optional = _BATCH[name]
            args = {key: value for key, value in step.items() if key != "action"}
            if set(required) - set(args) or set(args) - set(required) - set(optional):
                raise ValueError(f"{name} needs {', '.join(required)}"
                                 + (f" and may take {', '.join(optional)}" if optional else "") + " only")
            for key, value in args.items():
                if type(value) is not _TYPES[key] or (key == "modifiers" and not all(isinstance(m, str) for m in value)):
                    raise ValueError(f"{key} must be {'a list of strings' if key == 'modifiers' else _TYPES[key].__name__}")
            if name == "click":
                _check_click(args.get("button", "left"), args.get("clicks", 1))
            elif name == "scroll":
                _scroll_button(args["direction"], args.get("steps", 1))
            elif name == "type":
                _check_text(args["text"])
            elif name == "key":
                _key_symbol(args["key"], args.get("modifiers", []))
            for kx, ky in (("x", "y"), ("end_x", "end_y")):
                if kx in args:
                    args[kx], args[ky] = screen.map_point(coord_space, args[kx], args[ky], width, height)
                    if not (0 <= args[kx] < width and 0 <= args[ky] < height):
                        raise ValueError("coordinates are outside the selected window")
        except ValueError as exc:
            raise ValueError(f"actions[{i}]: {exc}; nothing was sent") from None
        plan.append((name, args))
    return plan


def _failed_after(window_id: int, text: str):
    """A failed action's answer with screenshot_after: an error that carries the window as it looks now
    (and so a fresh coord_space token)."""
    try:  # the failed step may have closed or resized the window; the failure text matters more
        shot = [block.to_image_content() if isinstance(block, Image) else TextContent(type="text", text=str(block))
                for block in screen.capture(window_id=window_id, settle=True)]
    except Exception as exc:  # noqa: BLE001
        shot = [TextContent(type="text", text=f"no screenshot: {exc}")]
    return CallToolResult(content=[TextContent(type="text", text=text), *shot],
                          structuredContent={"result": text}, isError=True)


def _refused(window_id: int, screenshot_after: bool, exc: Exception):
    """A refusal or failure: raised as it is, or with screenshot_after returned as an error with the picture."""
    if not screenshot_after:
        raise exc
    return _failed_after(window_id, str(exc))


def _run(window_id: int, screenshot_after: bool, step: Callable[[], str]):
    """One action as a tool call: ``step`` under both locks, then its answer."""
    try:
        with _action_lock():
            text = step()
    except Exception as exc:  # noqa: BLE001
        return _refused(window_id, screenshot_after, exc)
    return _after(window_id, text, screenshot_after)


def register_desktop_input_tools(server: FastMCP, groups: Iterable[str] | None = None) -> None:
    """Register input only when the user selected the explicit-only computer toolset at startup."""
    mcp = ScopedTools(server, groups)

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_click(window_id: _WindowId, expected_pid: _Pid, x: _X, y: _Y,
                       expected_width: _Width, expected_height: _Height,
                       button: Annotated[str, Field(description="left, middle or right.")] = "left",
                       clicks: Annotated[int, Field(description="1 or 2.")] = 1,
                       coord_space: _CoordSpace = "window", screenshot_after: _ShotAfter = False):
        """Click a visible point in one selected X11 window.

        Obtain window_id, PID and size from windows(action="list"); inspect a fresh screenshot first.
        Input affects the shared desktop and can trigger destructive or external actions.
        """
        def step():
            px, py = screen.map_point(coord_space, x, y, expected_width, expected_height)
            return _step_click(window_id, expected_pid, px, py, expected_width, expected_height, button, clicks)

        return _run(window_id, screenshot_after, step)

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_drag(window_id: _WindowId, expected_pid: _Pid, x: _X, y: _Y,
                      end_x: Annotated[int, Field(description="Drag end X, in the same space as x.")],
                      end_y: Annotated[int, Field(description="Drag end Y, in the same space as y.")],
                      expected_width: _Width, expected_height: _Height,
                      coord_space: _CoordSpace = "window", screenshot_after: _ShotAfter = False):
        """Drag with the left mouse button between two points in the same X11 window.

        Inspect a fresh screenshot and size first.
        Input affects the shared desktop and can trigger destructive or external actions.
        """
        def step():
            start = screen.map_point(coord_space, x, y, expected_width, expected_height)
            end = screen.map_point(coord_space, end_x, end_y, expected_width, expected_height)
            return _step_drag(window_id, expected_pid, *start, *end, expected_width, expected_height)

        return _run(window_id, screenshot_after, step)

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_scroll(window_id: _WindowId, expected_pid: _Pid, x: _X, y: _Y,
                        expected_width: _Width, expected_height: _Height,
                        direction: Annotated[str, Field(description="up, down, left or right.")],
                        steps: Annotated[int, Field(description="Scroll steps, 1-10.")] = 1,
                        coord_space: _CoordSpace = "window", screenshot_after: _ShotAfter = False):
        """Scroll up, down, left or right at a visible window point, at most ten steps per call.

        Inspect a fresh screenshot and size first.
        Input affects the shared desktop and can trigger destructive or external actions.
        """
        def step():
            px, py = screen.map_point(coord_space, x, y, expected_width, expected_height)
            return _step_scroll(window_id, expected_pid, px, py, expected_width, expected_height, direction, steps)

        return _run(window_id, screenshot_after, step)

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_type(window_id: _WindowId, expected_pid: _Pid,
                      text: Annotated[str, Field(description="Up to 200 printable ASCII characters.")],
                      screenshot_after: _ShotAfter = False):
        """Type up to 200 printable ASCII characters into the focused selected X11 window.

        It does not use the clipboard. Caps Lock and unsupported key mappings are rejected.
        Input affects the shared desktop and can trigger destructive or external actions.
        """
        return _run(window_id, screenshot_after, lambda: _step_type(window_id, expected_pid, text))

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_key(window_id: _WindowId, expected_pid: _Pid,
                     key: Annotated[str, Field(description=f"A letter, a digit, or {_KEY_NAMES}.")],
                     modifiers: Annotated[list[str] | None, Field(description="Any of ctrl, shift, alt, super.")] = None,
                     screenshot_after: _ShotAfter = False):
        """Send one navigation/editing key or ASCII letter/digit to the focused X11 window.

        Modifiers may contain ctrl, shift, alt and super. Input can trigger destructive or external actions.
        """
        return _run(window_id, screenshot_after, lambda: _step_key(window_id, expected_pid, key, modifiers))

    @mcp.tool(annotations=DESKTOP_INPUT)
    def computer_batch(window_id: _WindowId, expected_pid: _Pid, expected_width: _Width, expected_height: _Height,
                       actions: _Steps, coord_space: _CoordSpace = "window", screenshot_after: _ShotAfter = False):
        """Run 1-10 click, drag, scroll, type or key steps in one selected X11 window, in order.

        Every step is checked before any input is sent; each still checks the window, PID, size and
        covering windows. It stops at the first failing step and says how many ran.
        Input affects the shared desktop and can trigger destructive or external actions.
        """
        try:
            plan = _batch_plan(actions, coord_space, expected_width, expected_height)
        except ValueError as exc:  # nothing was sent, and no lock was taken
            return _refused(window_id, screenshot_after, exc)

        def steps():
            done: list[str] = []
            for name, args in plan:
                _call_lock_state.clear()  # a batch can run for minutes: each step probes the screen lock again
                size = {"width": expected_width, "height": expected_height} if "x" in args else {}
                try:
                    done.append(_STEPS[name](window_id, expected_pid, **args, **size))
                except Exception as exc:  # noqa: BLE001 - reported with the steps that ran
                    ran = "; ".join(f"{i}: {text}" for i, text in enumerate(done))
                    raise RuntimeError(
                        f"batch stopped: actions[{len(done)}] ({name}) failed: {str(exc).rstrip('.')}. {len(done)} "
                        f"of {len(plan)} step(s) ran before it{': ' + ran if ran else ''}. Capture the window "
                        "before retrying.") from exc
            return f"batch ran all {len(plan)} step(s): " + "; ".join(f"{i}: {text}" for i, text in enumerate(done))

        return _run(window_id, screenshot_after, steps)
