"""Version adapters: the current API passes through untouched, and Isaac Sim 5.1's is translated."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("sim_compat_under_test", ROOT / "exts/sim-tools/sim_tools/compat.py")
compat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compat)


class Current:
    @staticmethod
    def get_active_physics_engine():
        return "newton"


class PhysxOnly:
    _physx_sim_interface = object()
    calls = []

    @classmethod
    def get_physics_sim_view(cls):
        return "view"

    @classmethod
    def step(cls, render=False):
        cls.calls.append(render)

    @classmethod
    def get_physics_dt(cls):
        return 0.5


class Unknown:
    pass


def test_a_build_with_the_current_api_gets_the_real_manager():
    assert compat.adapt_simulation_manager(Current) is Current


def test_an_unknown_manager_is_not_assumed_to_be_physx():
    assert compat.adapt_simulation_manager(Unknown) is Unknown


def test_isaac_51_manager_is_translated():
    PhysxOnly.calls.clear()
    sm = compat.adapt_simulation_manager(PhysxOnly)
    assert sm.get_active_physics_engine() == "physx"
    assert sm.get_physics_simulation_view() == "view"
    assert sm.get_physics_dt() == 0.5
    sm.step(steps=3)
    assert PhysxOnly.calls == [False, False, False]


def test_composition_errors_uses_the_native_call_when_present():
    class Stage:
        @staticmethod
        def GetCompositionErrors():
            return ("e",)

    assert compat.composition_errors(Stage) == ["e"]


def test_composition_errors_without_the_native_call_reads_prim_indexes(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Usd

    class OldStage:
        def __init__(self, stage):
            self.stage = stage

    path = tmp_path / "broken.usda"
    path.write_text('#usda 1.0\ndef "World" (references = @./missing.usda@) {}\n')
    stage = Usd.Stage.Open(str(path))
    native = stage.GetCompositionErrors()
    assert native
    original = getattr
    real = compat.__dict__.get("getattr", original)
    compat.getattr = lambda obj, name, default=None: None if name == "GetCompositionErrors" else real(obj, name, default)
    try:
        assert len(compat.composition_errors(stage)) >= 1
    finally:
        del compat.getattr


class LegacyHydra:
    EVENT_TYPE_DRAWABLE_CHANGED = 7


class LegacyTexture:
    def __init__(self):
        self.subscribed = None
        self.unsubscribed = 0

    def get_event_stream(self):
        return self

    def create_subscription_to_push_by_type(self, kind, callback, name):
        self.subscribed = (kind, callback, name)
        texture = self

        class Subscription:
            def unsubscribe(self):
                texture.unsubscribed += 1

        return Subscription()


def test_legacy_drawable_stream_delivers_the_payload_and_resets_once():
    texture, seen = LegacyTexture(), []
    assert compat.has_drawable_events(LegacyHydra, texture)
    subscription = compat.observe_drawable_changed(LegacyHydra, texture, seen.append, "x")
    kind, callback, name = texture.subscribed
    assert (kind, name) == (7, "x")

    class Event:
        payload = {"result_handle": 5}

    callback(Event())
    assert seen == [{"result_handle": 5}]
    subscription.reset()
    subscription.reset()
    assert texture.unsubscribed == 1


def test_a_texture_with_no_event_api_is_reported():
    assert not compat.has_drawable_events(object(), object())


def test_extra_frames_are_published_as_prim_paths():
    from twinsmith.server import build_server
    import asyncio

    tools = {tool.name: tool for tool in asyncio.run(build_server({"TWINSMITH_TOOLSETS": "default,+sim"}).list_tools())}
    items = tools["sim__ros2_tf"].inputSchema["properties"]["extra_frames"]
    variants = items.get("anyOf", [items])
    assert any(v.get("items", {}).get("type") == "string" for v in variants)
