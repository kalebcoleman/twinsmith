"""Act and see in one call: screenshot_after on the computer tools and on sim__ui act/find."""

import asyncio
import io
import json

from mcp.server.fastmcp import FastMCP, Image
from mcp.shared.memory import create_connected_server_and_client_session
from PIL import Image as PILImage

from test_desktop_input import desktop_setup  # noqa: F401 - fixture
from twinsmith import computer_use, grouped_tools, screen, server
from twinsmith import desktop_input as desktop

PNG = Image(data=b"\x89PNG fake", format="png")


def _computer_click(arguments):
    """Call the listed ``computer`` group (action=click) through an MCP client, as an agent would."""
    app = FastMCP("act-and-see")
    desktop.register_desktop_input_tools(app, groups={"computer"})
    grouped_tools.sync(app)

    async def run():
        async with create_connected_server_and_client_session(app._mcp_server) as client:
            await client.initialize()
            return await client.call_tool("computer", {"action": "click", **arguments})

    return asyncio.run(run())


CLICK = {"window_id": 101, "expected_pid": 22, "x": 20, "y": 30, "expected_width": 300,
         "expected_height": 200, "screenshot_after": True}


def test_computer_click_returns_its_answer_and_the_window_after_every_lock_is_released(desktop_setup, monkeypatch):
    disp, _ = desktop_setup
    taken = []

    def capture(**kwargs):
        assert not desktop._input_lock.locked() and not disp.grabbed, "captured while input was held"
        taken.append(kwargs)
        return [PNG, "coord_space: img:300x200@0,0,300x200/300x200"]

    monkeypatch.setattr(screen, "capture", capture)
    result = _computer_click(CLICK)
    assert not result.isError
    assert [block.type for block in result.content] == ["text", "image", "text"]
    assert result.content[0].text == "sent click to window 101 at (20, 30)"
    assert result.content[2].text.startswith("coord_space:")
    assert taken == [{"window_id": 101, "settle": True}]  # read once the window has reacted
    assert disp.pointer == (120, 130)


def test_a_capture_that_fails_keeps_the_computer_action_answer(desktop_setup, monkeypatch):
    disp, _ = desktop_setup

    def capture(**kwargs):
        raise ValueError("no visible window with id 101")

    monkeypatch.setattr(screen, "capture", capture)
    result = _computer_click(CLICK)
    assert not result.isError
    assert len(result.content) == 1
    text = result.content[0].text
    assert text.startswith("sent click to window 101") and "screenshot: not taken (no visible window with id 101)" in text
    assert desktop.X.ButtonPress in [kind for kind, *_ in disp.events], "the click was sent"


def _kit(monkeypatch, answer, window=True, capture_error=None):
    """kit_ui with a fake bridge: returns the result and the params Kit received."""
    sent = []

    def dispatch(canonical, params, state):
        sent.append((canonical, params))
        return answer

    def capture(**kwargs):
        if capture_error:
            raise capture_error
        assert kwargs == {"window_id": 7, "region": None}
        return [PNG, "caption"]

    monkeypatch.setattr(computer_use, "kit_window", lambda state: {"id": 7} if window else None)
    monkeypatch.setattr(screen, "capture", capture)
    params = {"path": "Window//Button", "action": "click", "coord_space": "window", "screenshot_after": True}
    result = computer_use.kit_ui("sim.ui.act", params, None, dispatch, server._failed_tool_result)
    assert sent == [("sim.ui.act", {"path": "Window//Button", "action": "click"})], "host-only args reached Kit"
    return result


OK = json.dumps({"status": "ok", "clicked": "Window//Button"})
FAILED = json.dumps({"status": "error", "code": "NOT_FOUND", "message": "no widget"})


def test_kit_act_with_screenshot_returns_the_answer_and_the_window(monkeypatch):
    result = _kit(monkeypatch, OK)
    assert not result.isError
    assert [block.type for block in result.content] == ["text", "image", "text"]
    assert result.content[0].text == OK and result.structuredContent == {"result": OK}
    assert result.content[2].text == "caption"


def test_a_failed_kit_act_still_carries_the_picture(monkeypatch):
    result = _kit(monkeypatch, FAILED)
    assert result.isError
    assert [block.type for block in result.content] == ["text", "image", "text"]
    assert result.content[0].text == FAILED


def test_no_isaac_window_keeps_the_kit_answer_with_a_note(monkeypatch):
    result = _kit(monkeypatch, OK, window=False)
    assert not result.isError
    assert [block.text for block in result.content] == [
        OK, "screenshot: not taken (no visible Isaac Sim window for the selected bridge)"]
    assert result.structuredContent == {"result": OK}
    failed = _kit(monkeypatch, FAILED, window=False)
    assert failed.isError and [block.type for block in failed.content] == ["text", "text"]


def test_an_x_error_while_capturing_keeps_the_kit_answer(monkeypatch):
    result = _kit(monkeypatch, OK, capture_error=RuntimeError("cannot read pixels of window 7"))
    assert not result.isError
    assert result.content[1].text == "screenshot: not taken (cannot read pixels of window 7)"


def test_kit_act_without_screenshot_is_forwarded_unchanged(monkeypatch):
    sent = []
    params = {"path": "Window//Button", "action": "click", "coord_space": "window", "screenshot_after": False}
    result = computer_use.kit_ui("sim.ui.act", params, None, lambda c, p, s: sent.append(p) or OK,
                                 server._failed_tool_result)
    assert result == OK and sent == [{"path": "Window//Button", "action": "click"}]


def test_grouped_sim_ui_act_returns_the_picture_through_a_client(monkeypatch, tmp_path):
    """End to end: the catalog-built sim__ui__act behind the listed sim__ui group, over an MCP session."""
    sent = []
    answers = iter([OK, FAILED])
    monkeypatch.setattr(server, "_dispatch", lambda canonical, params, state: sent.append((canonical, params))
                        or next(answers))
    monkeypatch.setattr(computer_use, "kit_window", lambda state: {"id": 7, "width": 400, "height": 300})
    monkeypatch.setattr(screen, "capture", lambda **kwargs: [PNG, "caption"])
    app = server.build_server({"TWINSMITH_TOOLSETS": "default,+sim", "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path)})
    act = {"action": "act", "sub_action": "click", "x": 100, "y": 50, "screenshot_after": True,
           "coord_space": "img:200x150@0,0,400x300/400x300"}

    async def run():
        async with create_connected_server_and_client_session(app._mcp_server) as client:
            await client.initialize()
            return [await client.call_tool("sim__ui", act) for _ in range(2)]

    ok, failed = asyncio.run(run())
    assert not ok.isError and [block.type for block in ok.content] == ["text", "image", "text"]
    assert ok.content[0].text == OK and ok.content[1].data
    assert failed.isError and [block.type for block in failed.content] == ["text", "image", "text"]
    assert json.loads(failed.content[0].text)["code"] == "NOT_FOUND"
    canonical, params = sent[0]
    assert canonical == "sim.ui.act" and (params["x"], params["y"]) == (201, 101)   # image pixel centres
    assert not {"coord_space", "screenshot_after", "badge"} & set(params), "host-only arguments reached Kit"


# ------------------------------------------------------------------------------- failures and settling


def test_a_failed_single_action_with_screenshot_is_an_error_with_the_picture(desktop_setup, monkeypatch):
    disp, _ = desktop_setup
    disp.covered = lambda x, y: True
    monkeypatch.setattr(screen, "capture", lambda **kw: [PNG, "coord_space: img:300x200@0,0,300x200/300x200"])
    result = _computer_click(CLICK)
    assert result.isError and [block.type for block in result.content] == ["text", "image", "text"]
    assert "another window covers that point" in result.content[0].text
    assert desktop.X.ButtonPress not in [kind for kind, *_ in disp.events], "a refusal sends no click"
    # A stale token is refused the same way, and the picture brings the token to use instead.
    stale = _computer_click({**CLICK, "coord_space": "img:150x100@0,0,600x400/600x400"})
    assert stale.isError and "this one is 300x200" in stale.content[0].text
    assert stale.content[2].text.startswith("coord_space:")
    # Without screenshot_after a failure stays a plain error.
    plain = _computer_click({**CLICK, "screenshot_after": False})
    assert plain.isError and [block.type for block in plain.content] == ["text"]


def test_the_picture_after_an_action_waits_for_the_window_to_stop_changing(monkeypatch, tmp_path):
    frames = iter(["old", "repainting", "new", "new"])
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path)
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen, "_match_window", lambda q, wid: dict(id=wid, title="t", x=0, y=0, width=4, height=2))
    monkeypatch.setattr(screen, "_grab_window_image",
                        lambda win: PILImage.new("RGB", (4, 2), {"old": 0, "repainting": 90, "new": 255}[next(frames)]))
    image, caption = screen.capture(window_id=5, settle=True)
    assert PILImage.open(io.BytesIO(image.data)).getpixel((0, 0)) == (255, 0, 0), "the settled frame, not the first"
    assert next(frames, None) is None and "still changing" not in caption


def test_a_window_that_never_settles_is_captured_with_a_note(monkeypatch, tmp_path):
    shade = iter(range(256))
    clock = iter(x * 0.3 for x in range(100))
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path)
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(screen, "_match_window", lambda q, wid: dict(id=wid, title="t", x=0, y=0, width=4, height=2))
    monkeypatch.setattr(screen, "_grab_window_image", lambda win: PILImage.new("RGB", (4, 2), next(shade)))
    _, caption = screen.capture(window_id=5, settle=True)
    assert "note: the window was still changing after 1 s" in caption
    assert next(shade) < 8, "bounded: a playing simulation costs a few grabs, not a wait for stillness"
