"""coord_space tokens: image pixels of a (downscaled or cropped) capture become window pixels in one
helper, screen.map_point, for the desktop computer tools and for Kit's sim__ui act (no display, no Kit)."""

import asyncio
import copy
import json
from contextlib import nullcontext
from types import SimpleNamespace

import pytest
from Xlib import X
from mcp.server.fastmcp import FastMCP
from mcp.server.fastmcp.exceptions import ToolError

from twinsmith import computer_use, desktop_input, grouped_tools, screen

FULL = "img:1600x900@0,0,2560x1440/2560x1440"     # a 2560x1440 window sent downscaled to 1600 wide


# ---- screen.map_point


def test_window_space_passes_points_through():
    for space in ("window", None, ""):
        assert screen.map_point(space, 7, 9, 10, 10) == (7, 9)


def test_pixel_centres_cover_the_window_from_first_to_last_pixel():
    assert screen.map_point(FULL, 0, 0, 2560, 1440) == (0, 0)
    assert screen.map_point(FULL, 1599, 899, 2560, 1440) == (2559, 1439)       # inside, never on the far edge
    assert screen.map_point(FULL, 800, 450, 2560, 1440) == (1280, 720)
    every = {screen.map_point(FULL, x, 0, 2560, 1440)[0] for x in range(1600)}
    assert min(every) == 0 and max(every) < 2560 and len(every) == 1600


def test_non_integer_scales_and_upscales_stay_inside_the_rect():
    token = screen.coord_token(3, 7, (0, 0, 7, 3), 7, 3)          # 7/3 across, 3/7 down
    xs = [screen.map_point(token, x, 0, 7, 3)[0] for x in range(3)]
    ys = [screen.map_point(token, 0, y, 7, 3)[1] for y in range(7)]
    assert xs == [1, 3, 5] and ys == [0, 0, 1, 1, 1, 2, 2]


def test_a_zoomed_crop_round_trips_through_its_offset():
    rect = (1000, 600, 400, 300)                                  # shown enlarged 4x
    token = screen.coord_token(1600, 1200, rect, 2560, 1440)
    assert token == "img:1600x1200@1000,600,400x300/2560x1440"
    for wx, wy in ((1000, 600), (1123, 777), (1399, 899)):
        ix, iy = (wx - 1000) * 4 + 2, (wy - 600) * 4 + 2           # inside that window pixel's 4x4 block
        assert screen.map_point(token, ix, iy, 2560, 1440) == (wx, wy)
    assert screen.map_point(f"  {token}\n", 0, 0, 2560, 1440) == (1000, 600)


def test_a_resized_window_is_refused():
    with pytest.raises(ValueError, match="2560x1440 window, but this one is 2560x1400"):
        screen.map_point(FULL, 10, 10, 2560, 1400)


@pytest.mark.parametrize("token", [
    "img:1600x900@0,0,2560x1440", "img:1600x900@0,0,2560x1440/2560x1440x", "img:1600x900/2560x1440",
    "1600x900@0,0,2560x1440/2560x1440", "img:-1x900@0,0,2560x1440/2560x1440", "screen", "WINDOW", 5])
def test_malformed_tokens_are_refused(token):
    with pytest.raises(ValueError, match="'window' or the coord_space token"):
        screen.map_point(token, 1, 1, 2560, 1440)


@pytest.mark.parametrize("token", [
    "img:0x900@0,0,2560x1440/2560x1440", "img:1600x900@0,0,0x1440/2560x1440",
    "img:1600x900@1,0,2560x1440/2560x1440", "img:1600x900@0,100,2560x1400/2560x1440"])
def test_inconsistent_tokens_are_refused(token):
    with pytest.raises(ValueError, match="inconsistent"):
        screen.map_point(token, 1, 1, 2560, 1440)


@pytest.mark.parametrize("point", [(-1, 0), (0, -1), (1600, 0), (0, 900), (5000, 5000)])
def test_points_outside_the_image_are_refused(point):
    with pytest.raises(ValueError, match="outside that 1600x900 image"):
        screen.map_point(FULL, *point, 2560, 1440)


@pytest.mark.parametrize("point", [("10", 5), (10.0, 5), (True, 5), (None, 5)])
def test_points_must_be_integers(point):
    with pytest.raises(ValueError, match="integer pixels"):
        screen.map_point(FULL, *point, 2560, 1440)


# ---- the grouped computer tool on a fake X display: a 300x200 window at (100, 100) on the root


class _Window:
    def __init__(self, wid, parent=None):
        self.id, self.parent = wid, parent

    def query_tree(self):
        return SimpleNamespace(parent=self.parent)

    def query_pointer(self):
        return SimpleNamespace(child=None, mask=0)

    def get_attributes(self):
        return SimpleNamespace(map_state=X.IsViewable)

    def get_geometry(self):
        return SimpleNamespace(width=300, height=200)


class _Display:
    def __init__(self):
        self.root = _Window(1)
        self.window = _Window(101, self.root)
        self.root.translate_coords = lambda *_: SimpleNamespace(x=100, y=100)
        self.root.query_pointer = self._pointer
        self.root.grab_keyboard = lambda *_: X.GrabSuccess
        self.pointer = (0, 0)
        self.events = []

    def _pointer(self):
        x, y = self.pointer
        inside = 100 <= x < 400 and 100 <= y < 300
        return SimpleNamespace(child=self.window if inside else None, mask=0)

    def screen(self):
        return SimpleNamespace(root=self.root, width_in_pixels=1000, height_in_pixels=800)

    def create_resource_object(self, kind, wid):
        return self.window

    def has_extension(self, name):
        return name == "XTEST"

    def query_keymap(self):
        return [0] * 32

    def ungrab_keyboard(self, *_):
        pass

    grab_server = ungrab_server = sync = close = ungrab_keyboard


@pytest.fixture
def computer(monkeypatch):
    disp = _Display()
    monkeypatch.setattr(screen, "_match_window", lambda query, wid: {"id": wid, "pid": 22})
    monkeypatch.setattr(screen, "_window_pid", lambda win, d: 22)
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_probe_locked", lambda: False)
    monkeypatch.setattr(desktop_input, "_desktop_lock", nullcontext)
    monkeypatch.setattr(desktop_input.time, "sleep", lambda _: None)

    def fake_input(d, event, detail=0, **kwargs):
        d.events.append((event, kwargs))
        if event == X.MotionNotify:
            d.pointer = (kwargs["x"], kwargs["y"])

    monkeypatch.setattr(desktop_input.xtest, "fake_input", fake_input)
    server = FastMCP("cu-mapping-test")
    desktop_input.register_desktop_input_tools(server, groups={"computer"})
    grouped_tools.sync(server)

    def call(**arguments):
        return asyncio.run(server.call_tool("computer", {"window_id": 101, "expected_pid": 22,
                                                        "expected_width": 300, "expected_height": 200, **arguments}))
    return disp, call


def presses(disp):
    """Root-space pointer position at each button press."""
    out, at = [], None
    for event, kwargs in disp.events:
        if event == X.MotionNotify:
            at = (kwargs["x"], kwargs["y"])
        elif event == X.ButtonPress:
            out.append(at)
    return out


def test_a_click_in_a_half_size_screenshot_lands_on_the_window_pixel(computer):
    disp, call = computer
    call(action="click", x=10, y=15, coord_space="img:150x100@0,0,300x200/300x200")
    assert presses(disp) == [(100 + 21, 100 + 31)]


def test_a_click_in_a_zoomed_crop_lands_on_the_window_pixel(computer):
    disp, call = computer
    call(action="click", x=9, y=1, coord_space="img:200x100@200,50,100x50/300x200")   # 2x zoom
    assert presses(disp) == [(100 + 204, 100 + 50)]


def test_both_ends_of_a_drag_are_mapped(computer):
    disp, call = computer
    call(action="drag", x=10, y=15, end_x=100, end_y=50, coord_space="img:150x100@0,0,300x200/300x200")
    motions = [(kw["x"], kw["y"]) for event, kw in disp.events if event == X.MotionNotify]
    assert presses(disp) == [(121, 131)] and motions[-1] == (100 + 201, 100 + 101)


def test_window_space_is_unchanged(computer):
    disp, call = computer
    call(action="click", x=20, y=30)
    assert presses(disp) == [(120, 130)]


@pytest.mark.parametrize("arguments, message", [
    (dict(action="click", x=10, y=15, coord_space="img:150x100@0,0,600x400/600x400"), "this one is 300x200"),
    (dict(action="click", x=150, y=15, coord_space="img:150x100@0,0,300x200/300x200"), "outside"),
    (dict(action="drag", x=1, y=1, end_x=1, end_y=100, coord_space="img:150x100@0,0,300x200/300x200"), "outside"),
    (dict(action="scroll", x=1, y=1, direction="down", coord_space="image"), "coord_space token"),
])
def test_a_refused_token_sends_no_input(computer, arguments, message):
    disp, call = computer
    with pytest.raises(ToolError, match=message):
        call(**arguments)
    assert disp.events == []


# ---- Kit's sim__ui act through the host hook


ISAAC = {"id": 7, "pid": 4242, "width": 1920, "height": 1080}
HALF = "img:960x540@0,0,1920x1080/1920x1080"


@pytest.fixture
def kit(monkeypatch):
    sent = []
    window = {"value": dict(ISAAC)}
    monkeypatch.setattr(computer_use, "kit_window", lambda state: window["value"])

    def run(params):
        before = copy.deepcopy(params)
        result = computer_use.kit_ui("sim.ui.act", params, None,
                                     lambda canonical, p, state: sent.append(p) or '{"status": "ok"}',
                                     lambda result: False)
        assert params == before, "the caller's arguments are never changed"
        return result
    return SimpleNamespace(run=run, sent=sent, window=window)


def refused(kit, params, message):
    with pytest.raises(ToolError) as error:
        kit.run(params)
    answer = json.loads(str(error.value))
    assert answer["code"] == "INVALID_VALUE" and message in answer["message"]
    assert kit.sent == [], "a refusal dispatches nothing"


def test_kit_points_are_mapped_and_coord_space_never_reaches_kit(kit):
    kit.run({"action": "drag", "x": 0, "y": 0, "end_x": 959, "end_y": 539, "coord_space": HALF})
    assert kit.sent == [{"action": "drag", "x": 1, "y": 1, "end_x": 1919, "end_y": 1079}]


def test_every_step_of_a_batch_is_mapped(kit):
    steps = [{"action": "click", "x": 100, "y": 50},
             {"action": "click", "ref": "Stage//Frame/Button[0]|Button:b"},
             {"action": "drag", "x": 10, "y": 20, "end_x": 30, "end_y": 40},
             {"action": "keypress", "keys": ["enter"], "x": None, "y": None},
             {"action": "wait", "frames": 5}]
    kit.run({"actions": steps, "coord_space": HALF})
    assert kit.sent == [{"actions": [
        {"action": "click", "x": 201, "y": 101},
        {"action": "click", "ref": "Stage//Frame/Button[0]|Button:b"},
        {"action": "drag", "x": 21, "y": 41, "end_x": 61, "end_y": 81},
        {"action": "keypress", "keys": ["enter"], "x": None, "y": None},
        {"action": "wait", "frames": 5}]}]


def test_a_crop_token_maps_kit_points_by_its_offset(kit):
    kit.run({"action": "click", "x": 7, "y": 3, "coord_space": "img:400x200@1000,500,200x100/1920x1080"})
    assert kit.sent == [{"action": "click", "x": 1003, "y": 501}]


def test_window_space_and_ref_targets_need_no_window(kit):
    kit.window["value"] = None
    kit.run({"action": "click", "x": 5, "y": 6})
    kit.run({"action": "click", "x": 5, "y": 6, "coord_space": "window"})
    kit.run({"action": "click", "ref": "a|Button:b", "coord_space": HALF})
    assert kit.sent == [{"action": "click", "x": 5, "y": 6}] * 2 + [{"action": "click", "ref": "a|Button:b"}]


def test_a_token_without_an_isaac_window_is_refused(kit):
    kit.window["value"] = None
    refused(kit, {"action": "click", "x": 5, "y": 6, "coord_space": HALF}, "Isaac Sim window")


def test_a_token_from_before_a_resize_is_refused(kit):
    kit.window["value"] = {**ISAAC, "width": 1280, "height": 720}
    refused(kit, {"actions": [{"action": "click", "x": 5, "y": 6}], "coord_space": HALF}, "this one is 1280x720")


@pytest.mark.parametrize("params, message", [
    ({"action": "click", "x": 5, "coord_space": HALF}, "x and y go together"),
    ({"action": "drag", "x": 5, "y": 5, "end_y": 9, "coord_space": HALF}, "end_x and end_y go together"),
    ({"action": "click", "x": "5", "y": 6, "coord_space": HALF}, "integer pixels"),
    ({"action": "click", "x": 960, "y": 6, "coord_space": HALF}, "outside"),
    ({"action": "click", "x": 5, "y": 6, "coord_space": "img:960x540"}, "coord_space token"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "click", "x": 5.5, "y": 1}],
      "coord_space": HALF}, "integer pixels"),
    ({"actions": [{"action": "click", "x": 1, "y": 1, "coord_space": "window"}], "coord_space": HALF},
     "whole call"),
    ({"actions": '[{"action": "click", "x": 1, "y": 1}]', "coord_space": HALF}, "list of action objects"),
    ({"actions": ["click"], "coord_space": HALF}, "list of action objects"),
])
def test_bad_kit_points_are_refused_before_dispatch(kit, params, message):
    refused(kit, params, message)


def test_a_refused_kit_point_with_screenshot_after_brings_the_picture(kit, monkeypatch):
    """The token is stale: the picture of the window as it is now carries the one to use instead."""
    kit.window["value"] = {**ISAAC, "width": 1280, "height": 720}
    monkeypatch.setattr(screen, "capture", lambda **kw: ["coord_space: img:1280x720@0,0,1280x720/1280x720"])
    result = kit.run({"action": "click", "x": 5, "y": 6, "coord_space": HALF, "screenshot_after": True})
    assert result.isError and json.loads(result.content[0].text)["effects"] == "No input was sent."
    assert result.content[1].text.startswith("coord_space: img:1280x720") and kit.sent == []
