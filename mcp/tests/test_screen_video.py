"""Multi-window recording layout, recording defaults, frame-aware window placement and tiling,
raise_window on a locked screen, and join_videos (pure parts; no X display or ffmpeg run needed)."""
import io
import threading
import time
import types
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from twinsmith import screen


# ------------------------------------------------------------------ layout math


@pytest.mark.parametrize("count,layout,expected", [
    (2, "side_by_side", (1, 2)), (3, "top_bottom", (3, 1)), (2, "grid", (1, 2)), (3, "grid", (2, 2)),
    (4, "grid", (2, 2)), (5, "grid", (2, 3)), (6, "grid", (2, 3)), (2, "left_right", (1, 2)),
])
def test_grid_shape(count, layout, expected):
    assert screen.grid_shape(count, layout) == expected


def test_grid_shape_refuses_unknown_layouts():
    with pytest.raises(ValueError):
        screen.grid_shape(2, "diagonal")


def test_fit_into_keeps_aspect_and_centres():
    assert screen.fit_into(1440, 900, 960, 900) == (0, 150, 960, 600)  # wide window, letterboxed
    assert screen.fit_into(400, 800, 800, 800) == (200, 0, 400, 800)  # tall window, pillarboxed
    assert screen.fit_into(0, 10, 10, 10) == (0, 0, 0, 0)


def test_side_by_side_scales_to_common_height_without_padding():
    # Isaac Sim 1440x900 and RViz 1200x1000 at their native sizes: common height 1000.
    w, h, cells = screen.multi_window_layout([(1440, 900), (1200, 1000)], "side_by_side", max_width=10000)
    assert (w, h) == (2800, 1000)
    assert cells == [(0, 0, 1600, 1000), (1600, 0, 1200, 1000)]


def test_layout_is_capped_at_max_width_even_and_gapless():
    w, h, cells = screen.multi_window_layout([(1440, 900), (1440, 900)], "side_by_side")
    assert w <= 1920 and w % 2 == 0 and h % 2 == 0
    assert (w, h) == (1920, 600)
    assert cells[0][0] == 0 and cells[0][0] + cells[0][2] == cells[1][0] and cells[1][0] + cells[1][2] == w


def test_top_bottom_and_grid_layouts():
    w, h, cells = screen.multi_window_layout([(640, 480), (480, 360)], "top_bottom")
    assert (w, h) == (640, 960) and cells == [(0, 0, 640, 480), (0, 480, 640, 480)]
    w, h, cells = screen.multi_window_layout([(800, 600)] * 3, "grid")
    assert (w, h) == (1600, 1200) and len(cells) == 3 and cells[2] == (0, 600, 800, 600)


def test_tall_layouts_are_capped_in_height():
    w, h, _ = screen.multi_window_layout([(1000, 2000)] * 3, "top_bottom")
    assert h <= screen.MAX_CANVAS_HEIGHT and w % 2 == 0


def test_never_upscales_the_canvas():
    assert screen.multi_window_layout([(300, 200), (300, 200)], "side_by_side")[:2] == (600, 200)


# ------------------------------------------------------------------ ffmpeg filters and defaults


def test_recording_filter_variants():
    assert screen.recording_filter(1920, 804, True).startswith("scale=1920:804:force_original_aspect_ratio")
    assert screen.recording_filter(1280, 480, False, prescaled=True) == "format=yuv420p"
    assert screen.recording_filter(1440, 900, False) == "scale=1440:900,format=yuv420p"


def _register(monkeypatch, tmp_path):
    from mcp.server.fastmcp import FastMCP

    monkeypatch.setattr(screen, "VIDEO_DIR", tmp_path)
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path)
    server = FastMCP("screen-test")
    screen.register_screen_tools(server, groups={"computer"})
    return {name: tool.fn for name, tool in server._tool_manager._tools.items()}


class _Popen:
    def __init__(self, cmd, **_):
        self.cmd = cmd
        self.stdin = io.BytesIO()
        self.stderr = io.BytesIO()


def _start(monkeypatch, tmp_path, windows, **kwargs):
    tools = _register(monkeypatch, tmp_path)
    started = {}
    monkeypatch.setattr(screen, "_match_window", lambda q, wid, *_: windows[wid] if wid is not None else windows[1])
    monkeypatch.setattr(screen, "_ffmpeg", lambda: "ffmpeg")
    monkeypatch.setattr(screen, "_primary_monitor_size", lambda: (3440, 1440))
    monkeypatch.setattr(screen, "_screen_locked", lambda: False)
    monkeypatch.setattr(screen.subprocess, "Popen", lambda cmd, **kw: started.setdefault("p", _Popen(cmd)))
    monkeypatch.setattr(screen.threading.Thread, "start", lambda self: None)
    monkeypatch.setattr(screen, "_current", None)
    text = tools["start_screen_recording"](**kwargs)
    rec = screen._current
    monkeypatch.setattr(screen, "_current", None)
    return text, rec, started["p"].cmd


WINDOWS = {
    1: {"id": 1, "title": "Isaac Sim", "x": 0, "y": 0, "width": 1440, "height": 900},
    2: {"id": 2, "title": "RViz", "x": 0, "y": 0, "width": 1200, "height": 1000},
}


def test_window_recording_defaults_to_native_size_not_the_monitor(monkeypatch, tmp_path):
    text, rec, cmd = _start(monkeypatch, tmp_path, WINDOWS, window_id=1)
    assert (rec.out_width, rec.out_height) == (1440, 900) and not rec.full_screen
    assert cmd[cmd.index("-vf") + 1] == "scale=1440:900,format=yuv420p"
    assert "1440x900, max width 1920" in text


def test_explicit_full_screen_letterboxes_into_the_monitor_shape_capped(monkeypatch, tmp_path):
    _, rec, cmd = _start(monkeypatch, tmp_path, WINDOWS, window_id=1, full_screen=True)
    assert (rec.out_width, rec.out_height) == (1920, 804)  # 3440x1440 monitor shape at max_width 1920
    assert "pad=1920:804" in cmd[cmd.index("-vf") + 1]
    _, rec, _ = _start(monkeypatch, tmp_path, WINDOWS, window_id=1, full_screen=True, max_width=3840)
    assert (rec.out_width, rec.out_height) == (3440, 1440)  # the old behaviour, on request


def test_multi_window_recording_composes_one_canvas(monkeypatch, tmp_path):
    text, rec, cmd = _start(monkeypatch, tmp_path, WINDOWS, window_ids=[1, 2])
    assert rec.windows == [WINDOWS[1], WINDOWS[2]] and rec.layout == "side_by_side"
    assert cmd[cmd.index("-s") + 1] == f"{rec.bbox['width']}x{rec.bbox['height']}"
    assert cmd[cmd.index("-vf") + 1] == "format=yuv420p"
    assert rec.bbox["width"] <= 1920 and len(rec.cells) == 2
    assert "2 windows, side_by_side" in text


@pytest.mark.parametrize("kwargs,match", [
    (dict(window_ids=[1]), "2..6"),
    (dict(window_ids=[1, 1]), "distinct"),
    (dict(window_ids=[1, 2], window_id=1), "alone"),
    (dict(window_ids=[1, 2], layout="diagonal"), "layout"),
])
def test_multi_window_argument_validation(monkeypatch, tmp_path, kwargs, match):
    with pytest.raises(ValueError, match=match):
        _start(monkeypatch, tmp_path, WINDOWS, **kwargs)


def test_compose_cells_paints_each_window_and_survives_a_closed_one():
    from Xlib.error import XError

    rec = screen.Recording(path=Path("x.mp4"), bbox={"width": 8, "height": 2}, fps=10, max_width=1920,
                           proc=MagicMock(), stop=threading.Event(),
                           windows=[{"id": 1, "title": "a"}, {"id": 2, "title": "b"}],
                           cells=[(0, 0, 4, 2), (4, 0, 4, 2)])
    red = types.SimpleNamespace(get_geometry=lambda: types.SimpleNamespace(width=4, height=2),
                                get_image=lambda *a: types.SimpleNamespace(data=b"\x00\x00\xff\x00" * 8))
    bad = MagicMock()
    bad.get_geometry.side_effect = XError(MagicMock(), b"\x00" * 32)  # e.g. BadMatch while minimized
    gone: dict = {}
    data = screen._compose_cells(rec, None, [red, bad], gone)
    assert len(data) == 8 * 2 * 4
    assert data[:4] == b"\x00\x00\xff\x00" and data[16:20] == b"\x00\x00\x00\x00"  # BGRX: red | black
    assert list(gone) == [1] and gone[1] is not None and "could not be read" in rec.warnings[0]
    # A transient failure is retried (not fatal); a destroyed window (BadWindow) is given up on.
    screen._compose_cells(rec, None, [bad, bad], {0: 0.0, 1: 0.0})
    from Xlib.error import BadWindow

    dead = MagicMock()
    dead.get_geometry.side_effect = BadWindow(MagicMock(), b"\x00" * 32)
    with pytest.raises(RuntimeError, match="none of the recorded windows"):
        screen._compose_cells(rec, None, [dead, dead], {})


def test_compose_cells_skips_a_zero_pixel_cell():
    # Extreme aspect ratios gave a 0-wide cell and image.resize((0, 0)) ended the recording.
    rec = screen.Recording(path=Path("x.mp4"), bbox={"width": 4, "height": 2}, fps=10, max_width=1920,
                           proc=MagicMock(), stop=threading.Event(),
                           windows=[{"id": 1, "title": "a"}], cells=[(0, 0, 0, 2)])
    red = types.SimpleNamespace(get_geometry=lambda: types.SimpleNamespace(width=4, height=2),
                                get_image=lambda *a: types.SimpleNamespace(data=b"\x00\x00\xff\x00" * 8))
    assert len(screen._compose_cells(rec, None, [red], {})) == 4 * 2 * 4


def test_record_loop_repeats_frames_to_keep_real_time(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(screen.time, "monotonic", lambda: now[0])

    def slow_capture(*_):
        now[0] += 0.5  # one capture takes 5 frame intervals at 10 fps
        return types.SimpleNamespace(data=b"\x00" * 16)

    obj = MagicMock()
    obj.get_image.side_effect = slow_capture
    disp = MagicMock()
    disp.create_resource_object.return_value = obj
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    rec = screen.Recording(path=Path("x.mp4"), bbox={"width": 2, "height": 2}, fps=10, max_width=1920,
                           proc=MagicMock(), stop=threading.Event(), window={"id": 1, "title": "a"},
                           max_seconds=0, started=0.0)
    screen._record_loop(rec)
    # The first capture finished 0.5 s in: frames 1-5 all carry it, so the video is not sped up.
    assert rec.captured == 1 and rec.frames == 5


def test_finish_reports_repeated_frames_and_composed_captures(tmp_path):
    path = tmp_path / "v.mp4"
    path.write_bytes(b"x")
    rec = screen.Recording(path=path, bbox={"width": 4, "height": 4}, fps=15, max_width=1920,
                           proc=MagicMock(stderr=io.BytesIO()), stop=threading.Event(), frames=60, captured=20,
                           composed_frames=20)
    result = screen._finish(rec)
    assert "frames were repeated" in result["capture_rate"] and "locked" in result["composed"]


# ------------------------------------------------------------------ placement


def test_placement_request_offsets_by_frame_extents():
    extents = (0, 0, 37, 0)  # mutter on Ubuntu 22.04: a 37 px title bar, no side borders
    assert screen.placement_request(100, 100, 640, 480, extents) == (100, 63, 640, 480)
    assert screen.client_target(100, 100, 640, 480, extents) == (100, 100, 640, 480)
    assert screen.placement_request(100, 100, 640, 517, extents, frame=True) == (100, 100, 640, 480)
    assert screen.client_target(100, 100, 640, 517, extents, frame=True) == (100, 137, 640, 480)
    assert screen.placement_request(10, 10, 500, 500, (4, 6, 30, 5)) == (6, -20, 500, 500)


def test_tile_rects_split_the_work_area():
    area = (0, 27, 3440, 1413)
    assert screen.tile_rects(area, 2, "left_right") == [(0, 27, 1720, 1413), (1720, 27, 1720, 1413)]
    assert screen.tile_rects(area, 2, "left_right", gap=8) == [(0, 27, 1716, 1413), (1724, 27, 1716, 1413)]
    top, bottom = screen.tile_rects((0, 0, 1920, 1080), 2, "top_bottom")
    assert top == (0, 0, 1920, 540) and bottom == (0, 540, 1920, 540)
    grid = screen.tile_rects((0, 0, 1000, 1000), 3, "grid")
    assert grid == [(0, 0, 500, 500), (500, 0, 500, 500), (0, 500, 500, 500)]
    with pytest.raises(ValueError):
        screen.tile_rects(area, 2, "side_by_side")


def test_intersect_and_monitor_lookup():
    assert screen.intersect((0, 0, 1920, 1080), (0, 27, 3840, 1053)) == (0, 27, 1920, 1053)
    assert screen.intersect((0, 0, 10, 10), (20, 20, 5, 5)) is None
    mons = [{"left": 0, "top": 0, "width": 3840, "height": 1080},
            {"left": 0, "top": 0, "width": 1920, "height": 1080},
            {"left": 1920, "top": 0, "width": 1920, "height": 1080}]
    assert screen._monitor_for((2000, 10), mons) == 2 and screen._monitor_for((-5, -5), mons) == 1


def test_frame_extents_reads_the_property_and_defaults_to_zero():
    disp = MagicMock()
    obj = MagicMock()
    obj.get_full_property.return_value = types.SimpleNamespace(value=[2, 2, 37, 2])
    assert screen.frame_extents(disp, obj) == (2, 2, 37, 2)
    obj.get_full_property.return_value = None
    assert screen.frame_extents(disp, obj) == (0, 0, 0, 0)
    obj.get_full_property.side_effect = RuntimeError("gone")
    assert screen.frame_extents(disp, obj) == (0, 0, 0, 0)


def test_workarea_uses_the_current_desktop():
    disp = MagicMock()
    root = disp.screen.return_value.root
    props = {"_NET_WORKAREA": types.SimpleNamespace(value=[0, 27, 3440, 1413, 0, 0, 1920, 1080]),
             "_NET_CURRENT_DESKTOP": types.SimpleNamespace(value=[1])}
    disp.intern_atom.side_effect = lambda name: name
    root.get_full_property.side_effect = lambda atom, _t: props.get(atom)
    assert screen._workarea(disp) == (0, 0, 1920, 1080)
    props.pop("_NET_WORKAREA")
    assert screen._workarea(disp) is None


def _fake_display(extents, geometry_fn):
    disp = MagicMock()
    obj = disp.create_resource_object.return_value
    obj.get_full_property.return_value = types.SimpleNamespace(value=list(extents))
    state = {"g": None}

    def get_geometry():
        x, y, w, h = geometry_fn(state)
        state["pos"] = (x, y)
        return types.SimpleNamespace(width=w, height=h)

    obj.get_geometry.side_effect = get_geometry
    disp.screen.return_value.root.translate_coords.side_effect = (
        lambda *a: types.SimpleNamespace(x=state["pos"][0], y=state["pos"][1]))
    return disp


def test_place_window_requests_client_geometry_and_reports_the_frame(monkeypatch):
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    sent = []
    disp = _fake_display((0, 0, 37, 0), lambda st: (100, 100, 640, 480) if sent[1:] else (5, 5, 300, 300))
    root = disp.screen.return_value.root
    root.send_event.side_effect = lambda msg, event_mask: sent.append(msg)
    r = screen._place_window(disp, {"id": 7, "title": "RViz"}, 100, 100, 640, 480)
    assert sent[1]["data"][1][1:] == [100, 63, 640, 480]  # frame corner above the client by the title bar
    assert r["client"] == (100, 100, 640, 480) and r["frame"] == (100, 63, 640, 517) and r["applied"]
    assert "now 640x480+100+100 client area, frame 640x517+100+63" in screen._placement_line(r)


def test_place_window_corrects_an_unexpected_offset_once(monkeypatch):
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    sent = []

    def geometry(_st):
        moves = sent[1:]
        if not moves:
            return (0, 0, 640, 480)
        x, y = moves[-1]["data"][1][1:3]
        return (x + 10, y + 45, 640, 480)  # a WM that offsets by its invisible borders, not the extents

    disp = _fake_display((0, 0, 37, 0), geometry)
    disp.screen.return_value.root.send_event.side_effect = lambda msg, event_mask: sent.append(msg)
    r = screen._place_window(disp, {"id": 7, "title": "x"}, 100, 100, 640, 480)
    assert r["corrected"] and r["applied"] and r["client"] == (100, 100, 640, 480)


def test_place_window_reports_a_clamp_and_negative_coordinates_pack(monkeypatch):
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    sent = []
    disp = _fake_display((0, 0, 37, 0), lambda st: (100, 64, 640, 480))  # held below a 27 px top panel
    disp.screen.return_value.root.send_event.side_effect = lambda msg, event_mask: sent.append(msg)
    r = screen._place_window(disp, {"id": 7, "title": "x"}, 100, 0, 640, 480)
    assert sent[1]["data"][1][2] == (-37) & 0xFFFFFFFF  # signed y sent as CARD32, not an OverflowError
    assert not r["applied"] and "clamped" in screen._placement_line(r)


def test_tile_windows_validates_before_touching_windows(monkeypatch, tmp_path):
    tools = _register(monkeypatch, tmp_path)
    monkeypatch.setattr(screen, "_match_window", MagicMock(side_effect=AssertionError("not reached")))
    with pytest.raises(ValueError, match="2..6"):
        tools["tile_windows"](window_ids=[1])
    with pytest.raises(ValueError, match="layout"):
        tools["tile_windows"](window_ids=[1, 2], layout="diagonal")
    with pytest.raises(ValueError, match="gap"):
        tools["tile_windows"](window_ids=[1, 2], gap=500)


def test_tile_windows_places_frames_in_work_area_tiles(monkeypatch, tmp_path):
    tools = _register(monkeypatch, tmp_path)
    wins = {1: {"id": 1, "title": "Isaac Sim", "x": 10, "y": 10, "width": 800, "height": 600},
            2: {"id": 2, "title": "RViz", "x": 900, "y": 10, "width": 800, "height": 600}}
    monkeypatch.setattr(screen, "_match_window", lambda q, wid, *_: wins[wid])
    monkeypatch.setattr(screen, "_monitors", lambda: [{"left": 0, "top": 0, "width": 1920, "height": 1080}] * 2)
    monkeypatch.setattr(screen, "_xdisplay", MagicMock)
    monkeypatch.setattr(screen, "_workarea", lambda d: (0, 27, 1920, 1053))
    placed = []

    def place(disp, win, x, y, w, h, frame=False):
        placed.append((win["id"], (x, y, w, h), frame))
        return dict(id=win["id"], title=win["title"], client=(x, y + 37, w, h - 37), frame=(x, y, w, h),
                    extents=(0, 0, 37, 0), requested=(x, y, w, h), requested_client=(x, y + 37, w, h - 37),
                    applied=True, corrected=False, moved=True)

    monkeypatch.setattr(screen, "_place_window", place)
    out = tools["tile_windows"](window_ids=[1, 2])
    assert placed == [(1, (0, 27, 960, 1053), True), (2, (960, 27, 960, 1053), True)]
    assert "work area 1920x1053+0+27" in out


def test_raise_window_on_a_locked_screen_says_raising_is_impossible(monkeypatch, tmp_path):
    tools = _register(monkeypatch, tmp_path)
    disp = MagicMock()
    disp.screen.return_value.root.get_full_property.return_value = types.SimpleNamespace(value=[99])
    monkeypatch.setattr(screen, "_xdisplay", lambda: disp)
    monkeypatch.setattr(screen, "_match_window", lambda *_: {"id": 7, "title": "Sim"})
    monkeypatch.setattr(screen.xevent, "ClientMessage", lambda **kwargs: kwargs)
    monkeypatch.setattr(screen.time, "sleep", lambda s: None)
    monkeypatch.setattr(screen, "_probe_locked", lambda: True)
    with pytest.raises(RuntimeError, match="screen is locked.*not needed for captures"):
        tools["raise_window"](window_id=7)
    monkeypatch.setattr(screen, "_probe_locked", lambda: False)
    with pytest.raises(RuntimeError, match="did not activate"):
        tools["raise_window"](window_id=7)


# ------------------------------------------------------------------ join_videos

FFMPEG_I = """Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'a.mp4':
  Duration: 00:01:02.50, start: 0.000000, bitrate: 20 kb/s
  Stream #0:0[0x1](und): Video: h264 (High) (avc1 / 0x31637661), yuv420p(progressive), 1440x900 [SAR 1:1 DAR 8:5], 18 kb/s, 15 fps, 15 tbr, 15360 tbn (default)
At least one output file must be specified
"""


def test_parse_probe_reads_size_fps_and_duration():
    assert screen.parse_probe(FFMPEG_I) == {"width": 1440, "height": 900, "fps": 15.0, "duration": 62.5}
    assert screen.parse_probe("a.txt: Invalid data found when processing input") is None


def test_join_plan_concat_pads_to_the_first_clip():
    infos = [dict(width=1440, height=900, fps=15.0, duration=5), dict(width=640, height=480, fps=30.0, duration=3)]
    plan = screen.join_plan(infos, "concat")
    assert (plan["width"], plan["height"], plan["fps"], plan["duration"]) == (1440, 900, 30, 8)
    assert "pad=1440:900" in plan["filter"] and plan["filter"].endswith("concat=n=2:v=1:a=0[v]")


def test_join_plan_hstack_common_height_and_width_cap():
    infos = [dict(width=1440, height=900, fps=15.0, duration=5), dict(width=1200, height=1000, fps=15.0, duration=7)]
    plan = screen.join_plan(infos, "hstack", max_width=1920)
    assert plan["width"] <= 1920 and plan["width"] % 2 == 0 and plan["height"] % 2 == 0
    assert plan["duration"] == 7 and "hstack=inputs=2:shortest=0" in plan["filter"]
    assert screen.join_plan(infos, "hstack", shortest=True)["duration"] == 5
    v = screen.join_plan(infos, "vstack", max_width=1920)
    assert v["width"] == 1440 and "vstack=inputs=2" in v["filter"]


def test_join_plan_validation():
    one = [dict(width=2, height=2, fps=None, duration=1)]
    with pytest.raises(ValueError, match="2.."):
        screen.join_plan(one, "concat")
    with pytest.raises(ValueError, match="mode"):
        screen.join_plan(one * 2, "xfade")
    assert screen.join_plan(one * 2, "concat")["fps"] == 15  # unknown input rate


def test_join_command_never_overwrites_and_drops_audio():
    cmd = screen.join_command("ffmpeg", [Path("/h/a.mp4"), Path("/h/b.mp4")], {"filter": "F"}, Path("/h/o.mp4"))
    assert cmd[:6] == ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n"]
    assert "-y" not in cmd and "-an" in cmd and cmd[-1] == "/h/o.mp4"
    assert cmd[cmd.index("-filter_complex") + 1] == "F"


def test_join_timeout_is_bounded():
    assert screen.join_timeout(0) == 60 and screen.join_timeout(100) == 330 and screen.join_timeout(10_000) == 900


def test_input_and_output_paths_stay_under_home_and_never_overwrite(monkeypatch, tmp_path):
    home = tmp_path / "home"
    videos = home / "Videos" / "twinsmith"
    videos.mkdir(parents=True)
    (videos / "a.mp4").write_bytes(b"x")
    outside = tmp_path / "elsewhere.mp4"
    outside.write_bytes(b"x")
    (videos / "link.mp4").symlink_to(outside)
    monkeypatch.setattr(screen, "VIDEO_DIR", videos)
    roots = [home.resolve(), videos.resolve()]
    assert screen.safe_input_video("a.mp4", roots) == (videos / "a.mp4").resolve()
    for bad, match in [(str(outside), "outside"), ("link.mp4", "outside"), ("../a.mp4", "file name"),
                       ("missing.mp4", "no such file"), (str(videos / "a.txt"), "video file")]:
        with pytest.raises(ValueError, match=match):
            screen.safe_input_video(bad, roots)
    assert screen.safe_output_video("new.mp4", roots) == videos.resolve() / "new.mp4"
    assert screen.safe_output_video(None, roots).name.startswith("joined-")
    for bad, match in [("a.mp4", "already exists"), ("x.mkv", ".mp4"), (str(tmp_path / "o.mp4"), "outside"),
                       ("sub/o.mp4", "file name"), (str(home / "nope" / "o.mp4"), "does not exist")]:
        with pytest.raises(ValueError, match=match):
            screen.safe_output_video(bad, roots)


def test_new_tools_are_grouped_and_annotated():
    from twinsmith import toolsets

    assert toolsets.owner("join_videos") == "computer"
    assert toolsets.owner("tile_windows") == "computer"
    assert "computer" not in toolsets.DEFAULT and "computer" not in toolsets.PROFILES["all"]


# ------------------------------------------------------------------ Kit UI: close and single-level menus


def _kit_ui():
    import importlib.util

    path = Path(__file__).resolve().parents[2] / "exts/sim-tools/sim_tools/ui.py"
    spec = importlib.util.spec_from_file_location("sim_tools_ui_pr_c", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_menu_paths_allow_a_single_level_and_refuse_empty_levels():
    ui = _kit_ui()
    assert ui.menu_path("Layouts") == "Layouts"
    assert ui.menu_path(" Window / Console ") == "Window/Console"
    for bad in ("", "Window//Console", "/Edit", "Edit/", None, 5, "x" * 201):
        with pytest.raises(ui.UIRefusal) as err:
            ui.menu_path(bad)
        assert err.value.code == "INVALID_VALUE"


def test_close_needs_a_window_title_and_is_a_known_action():
    ui = _kit_ui()
    assert "close" in ui.ACTIONS and ui.window_title("Console", "close") == "Console"
    for bad in (None, "", "   ", 3, "x" * 121):
        with pytest.raises(ui.UIRefusal, match="action=close") as err:
            ui.window_title(bad, "close")
        assert err.value.code == "INVALID_VALUE"


def test_ui_act_contract_matches_the_kit_actions():
    from twinsmith.scene_contracts import REGISTRY

    params = REGISTRY["sim.ui.act"]["parameters"]
    assert params["action"]["enum"] == list(_kit_ui().ACTIONS)
    assert "close" in params["window"]["description"] and "Layouts" in params["menu"]["description"]


def test_close_refusals_report_that_no_input_was_sent():
    ui = _kit_ui()
    for code in ("INVALID_VALUE", "NO_WINDOW"):
        assert ui.refusal("sim.ui.act", ui.UIRefusal(code, "x"))["effects"] == "No input was sent."


# ---- wait_for_window and stop_on_log_marker ----


@pytest.fixture()
def screen_state(monkeypatch):
    monkeypatch.setattr(screen, "_current", None)
    monkeypatch.setattr(screen, "_window_wait", None)
    monkeypatch.setattr(screen, "_window_wait_error", None)
    monkeypatch.setattr(screen, "WINDOW_WAIT_POLL_S", 0.01)


def _no_window(query, wid=None, *_):
    raise ValueError("no window")


def _wait_until(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_defaults_do_not_wait_or_watch(monkeypatch, tmp_path, screen_state):
    text, rec, _ = _start(monkeypatch, tmp_path, WINDOWS, window_id=1)
    assert "waiting" not in text and "stops when" not in text and rec.stopped_by is None


def test_wait_returns_at_once_and_starts_when_the_window_appears(monkeypatch, tmp_path, screen_state):
    tools = _register(monkeypatch, tmp_path)
    seen = {"up": False}
    monkeypatch.setattr(screen, "_match_window", lambda q, wid=None, *_: WINDOWS[1] if seen["up"] else _no_window(q))
    monkeypatch.setattr(screen, "_ffmpeg", lambda: "ffmpeg")
    monkeypatch.setattr(screen, "_primary_monitor_size", lambda: (3440, 1440))
    monkeypatch.setattr(screen, "_screen_locked", lambda: False)
    monkeypatch.setattr(screen.subprocess, "Popen", lambda cmd, **kw: _Popen(cmd))
    monkeypatch.setattr(screen, "_record_loop", lambda rec: None)
    text = tools["start_screen_recording"](window="Isaac", wait_for_window=5)
    assert text.startswith("waiting up to 5 s") and screen._current is None
    assert "waiting for a window" in tools["screen_recording_status"]()
    with pytest.raises(RuntimeError, match="waiting for a window"):
        tools["start_screen_recording"](window_id=1)
    seen["up"] = True
    assert _wait_until(lambda: screen._current is not None)
    assert screen._window_wait is None and screen._current.window == WINDOWS[1]


def test_wait_times_out_and_status_says_so(monkeypatch, tmp_path, screen_state):
    tools = _register(monkeypatch, tmp_path)
    monkeypatch.setattr(screen, "_match_window", _no_window)
    tools["start_screen_recording"](window="Isaac", wait_for_window=0.05)
    assert _wait_until(lambda: screen._window_wait is None)
    assert "nothing was recorded" in tools["screen_recording_status"]()
    assert screen._current is None


def test_stop_cancels_a_wait(monkeypatch, tmp_path, screen_state):
    tools = _register(monkeypatch, tmp_path)
    monkeypatch.setattr(screen, "_match_window", _no_window)
    tools["start_screen_recording"](window="Isaac", wait_for_window=60)
    assert "cancelled" in tools["stop_screen_recording"]()
    assert screen._window_wait is None and screen._current is None


@pytest.mark.parametrize("kwargs,message", [
    (dict(wait_for_window=5), "needs window"),
    (dict(window="a", window_id=1, wait_for_window=5), "needs window"),
    (dict(window="a", monitor=1, wait_for_window=5), "needs window"),
    (dict(window="a", wait_for_window=601), "wait_for_window"),
    (dict(window="a", wait_for_window=-1), "wait_for_window"),
    (dict(window="a", stop_on_log_marker=""), "stop_on_log_marker"),
    (dict(window="a", stop_on_log_marker="x" * 257), "stop_on_log_marker"),
    (dict(window="a", stop_log_file="/tmp/a.log"), "needs stop_on_log_marker"),
    (dict(window="a", stop_on_log_marker="x", stop_log_file="/etc/passwd"), "stop_log_file"),
    (dict(window="a", stop_on_log_marker="x", stop_log_file="relative/logs/Kit/a.log"), "stop_log_file"),
])
def test_bad_recording_arguments(monkeypatch, tmp_path, screen_state, kwargs, message):
    tools = _register(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match=message):
        tools["start_screen_recording"](**kwargs)


def _rec():
    return SimpleNamespace(stop=threading.Event(), stopped_by=None, warnings=[], warned=set())


def test_marker_added_after_start_stops_the_recording(tmp_path):
    log = tmp_path / "kit.log"
    log.write_bytes(b"DONE was here before\n")
    rec = _rec()
    thread = threading.Thread(target=screen._watch_log_marker, args=(rec, "DONE", lambda: log, 0.01), daemon=True)
    thread.start()
    time.sleep(0.05)
    assert not rec.stop.is_set()  # text from before the watcher started does not count
    with log.open("ab") as f:
        f.write(b"nothing yet\nDO")
    time.sleep(0.05)
    assert not rec.stop.is_set()
    with log.open("ab") as f:
        f.write(b"NE\n")  # split across two reads
    assert rec.stop.wait(2) and "DONE" in rec.stopped_by
    thread.join(2)


def test_marker_watcher_ends_with_the_recording_and_waits_for_a_log(tmp_path):
    rec = _rec()
    asked = []
    thread = threading.Thread(target=screen._watch_log_marker,
                              args=(rec, "DONE", lambda: asked.append(1), 0.01), daemon=True)
    thread.start()
    assert _wait_until(lambda: asked)
    rec.stop.set()
    thread.join(3)
    assert not thread.is_alive() and rec.stopped_by is None


def test_only_project_logs_can_be_watched(tmp_path, monkeypatch):
    monkeypatch.setattr(screen.Path, "home", lambda: tmp_path)
    assert screen._log_marker_file_allowed(tmp_path / ".local/share/twinsmith/sim-sessions/abc/sim.log")
    assert screen._log_marker_file_allowed(Path("/opt/x/kit/logs/Kit/Isaac-Sim Python/6.2/kit_1.log"))
    assert not screen._log_marker_file_allowed(tmp_path / ".ssh/id_rsa.log")
    assert not screen._log_marker_file_allowed(Path("/opt/x/kit/logs/Kit/secret.txt"))
    assert not screen._log_marker_file_allowed(tmp_path / ".local/share/twinsmith/../../../notes.log")


def test_finish_reports_why_it_stopped(tmp_path):
    path = tmp_path / "video.mp4"
    path.write_bytes(b"x")
    rec = screen.Recording(path=path, bbox={"width": 2, "height": 2, "left": 0, "top": 0}, fps=10, max_width=960,
                           proc=SimpleNamespace(stderr=None, wait=lambda timeout=None: 0), stop=threading.Event(),
                           stopped_by="log marker 'DONE' in /x.log")
    assert "DONE" in screen._finish(rec)["stopped_by"]


def test_marker_with_no_log_to_watch_warns(monkeypatch):
    monkeypatch.setattr(screen, "LOG_MARKER_WARN_S", 0.0)
    rec = _rec()
    thread = threading.Thread(target=screen._watch_log_marker, args=(rec, "DONE", lambda: None, 0.01), daemon=True)
    thread.start()
    assert _wait_until(lambda: rec.warnings)
    rec.stop.set()
    thread.join(3)
    assert "could not stop this recording" in rec.warnings[0] and len(rec.warnings) == 1
