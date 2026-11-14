"""The viewport grid is hidden only while a stopped capture is taken."""

import asyncio
import sys
from pathlib import Path
from types import ModuleType

import pytest


ROOT = Path(__file__).resolve().parents[2]


class FakeSettings:
    def __init__(self, **values):
        self.values, self.writes = {"/app/viewport/grid/enabled": True, **values}, []

    def get(self, key):
        return self.values.get(key)

    def set(self, key, value):
        self.writes.append(value)
        self.values[key] = value


def _capture_module(monkeypatch, settings):
    monkeypatch.syspath_prepend(str(ROOT / "exts/sim-tools"))
    from sim_tools import camera_capture

    carb_settings = ModuleType("carb.settings")
    carb_settings.get_settings = lambda: settings
    carb = ModuleType("carb")
    carb.settings = carb_settings
    monkeypatch.setitem(sys.modules, "carb", carb)
    monkeypatch.setitem(sys.modules, "carb.settings", carb_settings)
    return camera_capture


def _during(module, playing, body=lambda: None):
    frames = []

    async def next_update():
        frames.append(1)

    async def run():
        async with module.without_viewport_grid(playing, next_update):
            body()

    asyncio.run(run())
    return frames


GRID = "/app/viewport/grid/enabled"


def test_stopped_capture_hides_the_grid_first_and_only_during_the_capture(monkeypatch):
    settings = FakeSettings()
    module = _capture_module(monkeypatch, settings)
    seen = []
    frames = _during(module, False, lambda: seen.append(settings.values[GRID]))
    assert seen == [False] and settings.values[GRID] is True
    assert len(frames) == module.GRID_SETTLE_FRAMES, "a frame drawn before the change must not be read back"


def test_the_grid_comes_back_when_a_capture_fails(monkeypatch):
    settings = FakeSettings()

    def fail():
        raise ZeroDivisionError

    with pytest.raises(ZeroDivisionError):
        _during(_capture_module(monkeypatch, settings), False, fail)
    assert settings.values[GRID] is True


def test_a_playing_timeline_or_a_grid_a_person_turned_off_is_left_alone(monkeypatch):
    settings = FakeSettings()
    assert _during(_capture_module(monkeypatch, settings), True) == [] and settings.writes == []
    settings = FakeSettings(**{GRID: False})
    assert _during(_capture_module(monkeypatch, settings), False) == []
    assert settings.writes == [] and settings.values[GRID] is False


def test_a_build_without_the_setting_is_untouched(monkeypatch):
    settings = FakeSettings()
    del settings.values[GRID]
    assert _during(_capture_module(monkeypatch, settings), False) == [] and settings.writes == []


def test_a_paused_capture_publishes_newton_poses_before_it_renders():
    """A camera adopted after the last physics step still has its build-time pose in Fabric: the first picture then
    looks from the wrong place (seen live on Newton, 2026-11-14). The capture command syncs Fabric first, when paused."""
    source = (ROOT / "exts/sim-tools/sim_tools/extension.py").read_text()
    branch = source.split("if wait:\n                if not s.timeline.is_playing() and not s.timeline.is_stopped():", 1)
    assert len(branch) == 2, "the paused branch of camera capture is gone"
    assert branch[1].index("sync_fabric(s.stage())") < branch[1].index("await capture(entry[1], validate")
