"""Short action batches: computer action=batch on a fake X display, and Kit's sim.ui.act run_act with
fake omni/carb modules (no display, no Kit)."""

from __future__ import annotations

import asyncio
import importlib.util
import sys
from contextlib import contextmanager
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
from mcp.server.fastmcp import Image
from mcp.shared.memory import create_connected_server_and_client_session
from mcp.types import CallToolResult
from Xlib import X

from test_desktop_input import desktop_setup, events  # noqa: F401 - fixture
from twinsmith import desktop_input as desktop
from twinsmith import server as gw

ROOT = Path(__file__).resolve().parents[2]
CLICK = {"action": "click", "x": 20, "y": 30}


@pytest.fixture
def batch(desktop_setup, monkeypatch):  # noqa: F811 - the imported fixture
    disp, tools = desktop_setup
    locks, shots = [], []

    @contextmanager
    def counted():
        locks.append(1)
        yield

    monkeypatch.setattr(desktop, "_desktop_lock", counted)
    monkeypatch.setattr(desktop.screen, "capture", lambda **kw: shots.append(kw) or [Image(data=b"png", format="png"), "cap"])
    return SimpleNamespace(disp=disp, run=lambda steps, **kw: tools["computer_batch"](101, 22, 300, 200, steps, **kw),
                           locks=locks, shots=shots)


def test_batch_runs_steps_in_order_under_one_lock_and_one_screenshot(batch):
    out = batch.run([CLICK, {"action": "type", "text": "ab"}, {"action": "key", "key": "enter"},
                     {"action": "scroll", "x": 5, "y": 5, "direction": "down", "steps": 2}], screenshot_after=True)
    assert events(batch.disp, X.ButtonPress) == [1, 5, 5]
    assert events(batch.disp, X.KeyPress) == [38, 56, 36]
    assert batch.locks == [1] and batch.shots == [{"window_id": 101, "settle": True}]
    assert out[0].startswith("batch ran all 4 step(s): 0: sent click") and "3: sent 2 scroll" in out[0]


def test_batch_stops_at_the_first_failure_and_says_how_many_ran(batch):
    batch.disp.covered = lambda x, y: x >= 300
    with pytest.raises(RuntimeError, match=r"actions\[1\] \(click\) failed: another window covers.*1 of 3 step\(s\) ran"):
        batch.run([CLICK, {"action": "click", "x": 250, "y": 30}, {"action": "type", "text": "a"}])
    assert events(batch.disp, X.ButtonPress) == [1] and not events(batch.disp, X.KeyPress)
    assert batch.locks == [1] and not batch.disp.grabbed


def test_failed_batch_with_screenshot_is_an_error_result_with_the_picture(batch):
    batch.disp.focus = batch.disp.root
    result = batch.run([CLICK, {"action": "type", "text": "a"}], screenshot_after=True)
    assert isinstance(result, CallToolResult) and result.isError
    assert [block.type for block in result.content] == ["text", "image", "text"]
    assert "focus" in result.content[0].text and result.structuredContent == {"result": result.content[0].text}
    assert len(batch.shots) == 1


def test_a_failed_capture_keeps_the_batch_failure(batch, monkeypatch):
    def gone(**kw):
        raise RuntimeError("window closed")

    monkeypatch.setattr(desktop.screen, "capture", gone)
    batch.disp.focus = batch.disp.root
    result = batch.run([CLICK, {"action": "key", "key": "a"}], screenshot_after=True)
    assert result.isError and "1 of 2 step(s) ran" in result.content[0].text
    assert result.content[1].text == "no screenshot: window closed"


@pytest.mark.parametrize("bad", [
    {"action": "click", "x": 1},                                 # missing y
    {"action": "click", "x": 1, "y": 2, "text": "a"},            # another action's argument
    {"action": "click", "x": "1", "y": 2},                       # wrong type
    {"action": "click", "x": True, "y": 2},                      # bool is not a pixel
    {"action": "click", "x": 1, "y": 2, "clicks": 3},
    {"action": "scroll", "x": 1, "y": 2, "direction": "sideways"},
    {"action": "type", "text": "é"},
    {"action": "key", "key": "a", "modifiers": ["hyper"]},
    {"action": "key", "key": "a", "modifiers": "ctrl"},
    {"action": "move", "x": 1, "y": 2},
    {"action": "click", "x": 300, "y": 2},                       # outside the 300x200 window
    {"action": "drag", "x": 1, "y": 1, "end_x": 1, "end_y": 200},
    "click",
])
def test_a_malformed_later_step_sends_nothing(batch, bad):
    with pytest.raises(ValueError, match=r"actions\[1\]: .*nothing was sent"):
        batch.run([CLICK, bad])
    assert not batch.disp.events and not batch.locks


def test_each_step_probes_the_screen_lock_again(batch, monkeypatch):
    states = iter([False, True])
    monkeypatch.setattr(desktop.screen, "_probe_locked", lambda: next(states))
    with pytest.raises(RuntimeError, match=r"actions\[1\] \(type\) failed: the desktop is locked.*1 of 2"):
        batch.run([CLICK, {"action": "type", "text": "secret"}])
    assert events(batch.disp, X.ButtonPress) == [1] and not events(batch.disp, X.KeyPress)


def test_batch_maps_every_point_before_input(batch):
    with pytest.raises(ValueError, match=r"actions\[1\]: .*outside"):
        batch.run([CLICK, {"action": "click", "x": 999, "y": 1}], coord_space="img:150x100@0,0,300x200/300x200")
    assert not batch.disp.events
    batch.run([{"action": "drag", "x": 10, "y": 15, "end_x": 90, "end_y": 15}],
              coord_space="img:150x100@0,0,300x200/300x200")
    assert batch.disp.pointer == (100 + 181, 100 + 31)  # image pixel centres, in window pixels


def test_computer_batch_is_reachable_and_typed_through_the_group(batch, tmp_path):
    server = gw.build_server({"TWINSMITH_TOOLSETS": "default,+computer", "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path)})

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            listed = {tool.name: tool for tool in (await client.list_tools()).tools}
            answer = await client.call_tool("computer", {
                "action": "batch", "window_id": 101, "expected_pid": 22, "expected_width": 300,
                "expected_height": 200, "actions": [CLICK, {"action": "key", "key": "a", "modifiers": ["ctrl"]}]})
            batch.disp.covered = lambda x, y: True
            failed = await client.call_tool("computer", {
                "action": "batch", "window_id": 101, "expected_pid": 22, "expected_width": 300,
                "expected_height": 200, "screenshot_after": True, "actions": [CLICK]})
            return listed, answer, failed

    listed, answer, failed = asyncio.run(run())
    assert failed.isError and [block.type for block in failed.content] == ["text", "image", "text"]
    assert "0 of 1 step(s) ran" in failed.content[0].text
    assert "computer_batch" not in listed
    schema = listed["computer"].inputSchema["properties"]["actions"]
    assert schema["type"] == "array" and schema["maxItems"] == 10
    assert schema["items"]["properties"]["action"]["enum"] == ["click", "drag", "scroll", "type", "key"]
    assert not answer.isError and "batch ran all 2 step(s)" in answer.content[0].text
    assert events(batch.disp, X.KeyPress) == [37, 38] and batch.locks == [1, 1]  # one per call


# ------------------------------------------------------------------------------- Kit run_act


@pytest.fixture
def kit(monkeypatch):
    spec = importlib.util.spec_from_file_location("sim_tools_ui_batch", ROOT / "exts/sim-tools/sim_tools/ui.py")
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    sent = []

    class Provider:
        def buffer_mouse_event(self, mouse, kind, norm, mods, pixel):
            sent.append((kind, pixel) if kind != "SCROLL" else (kind, norm))

        def buffer_keyboard_char_event(self, keyboard, ch, mods):
            sent.append(("char", ch))

        def buffer_keyboard_key_event(self, keyboard, kind, key, flags):
            sent.append((kind, key))

    names = lambda *n: SimpleNamespace(**{k: k for k in n})  # noqa: E731
    carb, carb_input = ModuleType("carb"), ModuleType("carb.input")
    carb_input.acquire_input_provider = Provider
    carb_input.MouseEventType = names("MOVE", "LEFT_BUTTON_DOWN", "LEFT_BUTTON_UP", "RIGHT_BUTTON_DOWN",
                                      "RIGHT_BUTTON_UP", "SCROLL")
    carb_input.KeyboardEventType = names("KEY_PRESS", "KEY_RELEASE")
    carb_input.KeyboardInput = names("ENTER", "S", "LEFT_CONTROL")
    carb_input.KEYBOARD_MODIFIER_FLAG_CONTROL = 2
    carb.input = carb_input
    frames = []

    async def next_update_async():
        frames.append(1)

    omni, kit_mod, app_mod, appwindow, omni_ui = (ModuleType(n) for n in (
        "omni", "omni.kit", "omni.kit.app", "omni.appwindow", "omni.ui"))
    app_mod.get_app = lambda: SimpleNamespace(next_update_async=next_update_async)
    appwindow.get_default_app_window = lambda: SimpleNamespace(get_mouse=lambda: "m", get_keyboard=lambda: "k")
    omni_ui.Workspace = SimpleNamespace(get_dpi_scale=lambda: 1.0, get_main_window_width=lambda: 800,
                                        get_main_window_height=lambda: 600)
    omni.kit, omni.appwindow, omni.ui, kit_mod.app = kit_mod, appwindow, omni_ui, app_mod
    for name, module in (("carb", carb), ("carb.input", carb_input), ("omni", omni), ("omni.kit", kit_mod),
                         ("omni.kit.app", app_mod), ("omni.appwindow", appwindow), ("omni.ui", omni_ui)):
        monkeypatch.setitem(sys.modules, name, module)
    return SimpleNamespace(ui=ui, sent=sent, frames=frames, run=lambda a: asyncio.run(ui.run_act(a)))


def test_kit_batch_runs_each_step_and_reports_completed(kit):
    seen = []
    act = kit.ui.act

    async def spy(step):
        seen.append(step)
        return await act(step)

    kit.ui.act = spy
    out = kit.run({"action": "click", "frames": 3, "scroll_x": 0, "scroll_y": 0, "badge": "/run/x", "actions": [
        {"action": "click", "x": 10, "y": 20}, {"action": "type", "text": "hi", "frames": 0},
        {"action": "keypress", "keys": ["ctrl", "s"]}]})
    assert out["action"] == "batch" and out["completed"] == 3 and [s["action"] for s in out["steps"]] == [
        "click", "type", "keypress"]
    assert [s.get("badge") for s in seen] == ["/run/x"] * 3, "internal host keys reach every step"
    assert ("LEFT_BUTTON_DOWN", (10.0, 20.0)) in kit.sent and ("char", "h") in kit.sent
    assert kit.sent[-1] == ("KEY_RELEASE", "LEFT_CONTROL")
    assert "find_after" not in out


def test_kit_batch_stops_at_the_failing_step_and_says_input_was_sent(kit):
    workspace = sys.modules["omni.ui"].Workspace
    workspace.get_window, workspace.get_windows = lambda title: None, lambda: []  # found only when its step runs
    with pytest.raises(kit.ui.UIRefusal) as err:
        kit.run({"actions": [{"action": "click", "x": 10, "y": 20}, {"action": "tab", "window": "Console"},
                             {"action": "type", "text": "never"}]})
    exc = err.value
    assert exc.code == "NO_WINDOW" and str(exc).startswith("actions[1] (tab):")
    assert exc.details["completed"] == 1 and exc.details["failed_index"] == 1 and len(exc.details["steps"]) == 1
    assert "No input was sent" not in exc.effects and "sent input" in exc.effects
    envelope = kit.ui.refusal("sim.ui.act", exc)   # what extension.py returns to the host
    assert envelope["code"] == "NO_WINDOW" and envelope["effects"] == exc.effects and envelope["completed"] == 1
    assert ("char", "n") not in kit.sent and sum(kind == "LEFT_BUTTON_DOWN" for kind, _ in kit.sent) == 1


def test_kit_batch_failing_first_step_keeps_the_no_input_rule(kit):
    with pytest.raises(kit.ui.UIRefusal) as err:
        kit.run({"actions": [{"action": "click", "ref": "Stage//Frame/Button[0]"}]})  # no fingerprint: refused
    assert err.value.effects is None and err.value.details["completed"] == 0
    assert kit.ui.refusal("sim.ui.act", err.value)["effects"] == "No input was sent."
    late = kit.ui.UIRefusal("ACT_FAILED", "actions[0] (click): boom", completed=0)
    assert kit.ui.refusal("sim.ui.act", late)["effects"] == "Inspect the UI before retrying."
    assert not kit.sent


@pytest.mark.parametrize("a, needle", [
    ({"actions": [{"action": "wait", "frames": 600}, {"action": "wait", "frames": 10}]}, "frames in all"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}] * 11}, "1..10"),
    ({"actions": []}, "1..10"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "fly"}]}, "actions[1]: action must be"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "type", "text": "a\nb"}]}, "actions[1]: text"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "click"}]}, "actions[1]: Give ref"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "click", "x": 1, "y": 1, "actions": []}]},
     "actions[1]: an object"),
    ({"x": 5, "actions": [{"action": "click", "x": 1, "y": 1}]}, "give x inside each step"),
    ({"scroll_y": 2, "actions": [{"action": "wait"}]}, "give scroll_y inside each step"),
    ({"find_after": "", "actions": [{"action": "wait"}]}, "find_after"),
    ({"actions": [{"action": "click", "x": 1, "y": 1}, {"action": "click", "x": 900, "y": 1}]}, "actions[1]: (900, 1)"),
    ({"actions": [{"action": "wait"}, {"action": "drag", "x": 1, "y": 1, "end_x": 1, "end_y": 600}]}, "actions[1]: (1, 600)"),
])
def test_kit_batch_is_checked_whole_before_any_input(kit, a, needle):
    with pytest.raises(kit.ui.UIRefusal) as err:
        kit.run(a)
    assert err.value.code in ("INVALID_VALUE", "OUT_OF_WINDOW") and needle in str(err.value)
    assert err.value.effects is None and not kit.sent and not kit.frames  # extension.py: "No input was sent."


def test_find_after_is_compact_and_a_find_error_is_only_a_note(kit, monkeypatch):
    widget = dict(ref="Stage//Frame/Button[0]|Button:b", type="Button", window="Stage", x=10, y=20, width=5,
                  height=5, name="b", tooltip="long tooltip", value=1)
    queries = []
    monkeypatch.setattr(kit.ui, "find", lambda a: queries.append(a) or dict(matches=3, widgets=[widget], windows=[]))
    out = kit.run({"action": "wait", "frames": 1, "find_after": "b"})
    assert queries == [{"query": "b", "limit": 10}]
    assert out["find_after"] == {"query": "b", "matches": 3, "widgets": [
        {"ref": widget["ref"], "type": "Button", "name": "b", "value": 1, "x": 10, "y": 20}]}

    def broken(a):
        raise kit.ui.UIRefusal("NO_UI", "headless")

    monkeypatch.setattr(kit.ui, "find", broken)
    out = kit.run({"actions": [{"action": "wait"}], "find_after": "b"})
    assert out["completed"] == 1 and out["find_after"] == {"query": "b", "note": "find_after failed: headless"}
    with pytest.raises(kit.ui.UIRefusal) as err:   # a failed batch still reports what it finds
        kit.run({"actions": [{"action": "wait"}, {"action": "click", "ref": "Stage//Frame/Button[0]"}], "find_after": "b"})
    assert err.value.details["find_after"]["note"] == "find_after failed: headless"


def test_kit_batch_steps_wait_the_top_level_frames_unless_they_give_their_own(kit):
    out = kit.run({"frames": 7, "actions": [{"action": "wait"}, {"action": "wait", "frames": 9}]})
    assert [step["frames_waited"] for step in out["steps"]] == [7, 9]
    with pytest.raises(kit.ui.UIRefusal, match="frames in all"):
        kit.run({"frames": 400, "actions": [{"action": "wait"}, {"action": "wait"}]})
    with pytest.raises(kit.ui.UIRefusal, match="the key is action, not sub_action"):
        kit.run({"actions": [{"sub_action": "wait"}]})


def test_kit_scroll_sends_whole_wheel_steps(kit):
    """Kit reads the wheel delta from the event's first slot as it is: divided by the window size (as
    omni.kit.ui_test does) five steps moved a tree by a pixel."""
    kit.run({"action": "scroll", "x": 10, "y": 10, "scroll_y": -5})
    assert kit.sent[-1] == ("SCROLL", (0.0, -5.0))


def test_a_batch_refused_before_any_input_still_brings_the_picture(batch):
    result = batch.run([CLICK], coord_space="img:150x100@0,0,600x400/600x400", screenshot_after=True)
    assert isinstance(result, CallToolResult) and result.isError and "this one is 300x200" in result.content[0].text
    assert [block.type for block in result.content] == ["text", "image", "text"]
    assert not batch.disp.events and not batch.locks
