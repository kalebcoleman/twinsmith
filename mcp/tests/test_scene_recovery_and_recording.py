"""Through the extension's own handlers: reset to a clean stop, physics readiness after Play, the
render-pose check on body state, and the clean recording layout with a caption."""

import asyncio
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

pytest.importorskip("pxr.Usd")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/sim-tools"))
from twinsmith.project_assets import AssetError
from twinsmith_bridge import jobs
from sim_tools import advanced, physics, viewport
from test_scene_usd import scene  # noqa: F401 - fixture


@pytest.fixture
def anyio_backend():
    return "asyncio"


@pytest.fixture
def kit(monkeypatch):
    """The Kit modules extension.py imports, as empty stand-ins, and the extension module itself."""
    modules = {}
    for name in ["omni", "omni.ext", "omni.usd", "omni.timeline", "omni.kit", "omni.kit.app", "omni.kit.undo",
                 "omni.kit.viewport", "omni.kit.viewport.utility", "omni.ui", "carb", "carb.settings"]:
        modules[name] = ModuleType(name)
        monkeypatch.setitem(sys.modules, name, modules[name])
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(modules[parent], child, modules[name])
    modules["omni.ext"].IExt = object
    state = SimpleNamespace(frames=0, settings={"/app/window/hideUi": False, "/app/viewport/grid/enabled": True}, ui=[])

    async def update():
        state.frames += 1
        await asyncio.sleep(0)

    modules["omni.kit.app"].get_app = lambda: SimpleNamespace(next_update_async=update, is_app_ready=lambda: True)
    modules["carb.settings"].get_settings = lambda: SimpleNamespace(get=state.settings.get, set=state.settings.__setitem__)
    path = Path(__file__).resolve().parents[2] / "exts/sim-tools/sim_tools/extension.py"
    spec = importlib.util.spec_from_file_location("sim_tools._test_recovery_extension", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    state.modules, state.module = modules, module
    return state


@pytest.fixture
def ext(kit, scene):  # noqa: F811
    """An extension over a real USD scene whose timeline the test drives."""
    timeline = SimpleNamespace(mode="stop")
    timeline.is_stopped = lambda: timeline.mode == "stop"
    timeline.is_playing = lambda: timeline.mode == "play"
    timeline.get_current_time = lambda: 0.0
    scene.timeline = timeline
    extension = kit.module.Extension()
    extension._operation_lock = asyncio.Lock()
    extension.scene = scene
    extension.bridge = SimpleNamespace(jobs=jobs.JobStore(asyncio.ensure_future))
    extension.follower = SimpleNamespace(stopped=[], stop=lambda reason: extension.follower.stopped.append(reason) or True,
                                         status=lambda: {})
    info = scene.stage_info()
    extension.ids = dict(session_id=info["session_id"], stage_id=info["stage_id"])

    async def control(args):  # the guarded native path is covered in test_scene_timeline.py
        timeline.mode = args["action"]
        return {"action": args["action"], "native_stop_observation": {"observed": "STOPPED"}}

    extension.native_control = control
    return extension


# ---- sim.timeline action=reset ----

@pytest.mark.anyio
async def test_reset_cancels_this_pack_s_jobs_stops_the_follow_and_stops_the_timeline(ext, monkeypatch):
    original = ext._timeline_control

    async def control(args):
        return await (ext.native_control(args) if args["action"] == "stop" else original(args))

    monkeypatch.setattr(ext, "_timeline_control", control)
    cleaned = []

    async def run(job):
        try:
            while True:
                job.checkpoint()
                await asyncio.sleep(0)
        finally:
            cleaned.append(job.kind)

    sim_job = ext.bridge.jobs.start("sim.simulation.run", run)
    other = ext.bridge.jobs.start("run_python", run)
    ext.scene.timeline.mode = "play"
    result = await ext.handler("sim.timeline")(**ext.ids, action="reset")
    assert result["status"] == "ok" and result["action"] == "reset" and result["was"] == "playing"
    assert result["stopped"] is True and result["playing"] is False and result["cancelled_jobs"] == [sim_job.id]
    assert result["viewport_follow_stopped"] is True and ext.follower.stopped == ["timeline reset"]
    assert sim_job.state == "cancelled" and cleaned == ["sim.simulation.run"], "the job ended before reset returned"
    assert other.state == "running", "a caller's own background code is not this pack's to cancel"
    ext.bridge.jobs.cancel(other.id)
    await asyncio.sleep(0)


@pytest.mark.anyio
async def test_reset_is_safe_to_repeat_on_a_stopped_timeline_and_checks_the_target(ext):
    for _ in range(2):
        result = await ext.handler("sim.timeline")(**ext.ids, action="reset")
        assert result["status"] == "ok" and result["was"] == "stopped" and result["cancelled_jobs"] == []
    stale = await ext.handler("sim.timeline")(session_id=ext.ids["session_id"], stage_id="other", action="reset")
    assert stale["status"] == "error" and stale["code"] == "STALE_TARGET"
    bad = await ext.handler("sim.timeline")(**ext.ids, action="reset", resume=True)
    assert bad["status"] == "error" and bad["code"] == "INVALID_VALUE"


@pytest.mark.anyio
async def test_a_reset_that_does_not_stop_is_an_error_not_a_clean_state(ext, monkeypatch):
    async def stuck(args):
        return {"action": "stop"}

    monkeypatch.setattr(ext, "_timeline_control", lambda args: stuck(args) if args["action"] == "stop" else None)
    ext.scene.timeline.mode = "pause"
    with pytest.raises(AssetError) as error:
        await ext._reset({**ext.ids, "action": "reset"})
    assert error.value.code == "TIMELINE_INTERRUPTED" and error.value.details["observed_state"]["was"] == "paused"


# ---- physics readiness after Play ----

@pytest.mark.anyio
async def test_play_waits_for_the_engine_and_reports_when_it_is_ready(ext, kit, monkeypatch):
    ext.scene.timeline.mode = "play"
    ready_at = kit.frames + 4
    monkeypatch.setattr(physics, "physics_ready", lambda: kit.frames >= ready_at)
    assert await ext._await_physics() == {"physics_ready": True, "physics_ready_frames": 4}
    monkeypatch.setattr(physics, "physics_ready", lambda: False)
    late = await ext._await_physics(frames=5)
    assert late["physics_ready"] is False and late["physics_ready_frames"] == 5 and "PHYSICS_NOT_READY" in late["physics_ready_note"]
    before = kit.frames
    monkeypatch.setattr(physics, "physics_ready", lambda: None)
    unknown = await ext._await_physics()
    assert unknown["physics_ready"] is None and kit.frames == before, "a build that cannot say is not waited on"


@pytest.mark.anyio
async def test_the_wait_ends_when_the_timeline_stops_under_it(ext, kit, monkeypatch):
    ext.scene.timeline.mode = "play"
    monkeypatch.setattr(physics, "physics_ready", lambda: False)

    async def update():
        kit.frames += 1
        ext.scene.timeline.mode = "stop"

    kit.modules["omni.kit.app"].get_app = lambda: SimpleNamespace(next_update_async=update)
    assert (await ext._await_physics())["physics_ready_frames"] == 1


# ---- sim.body.state: the rendered pose against the physics pose ----

def body_call(ext, monkeypatch, measured):
    stage = ext.scene.stage()
    ext.scene.create(stage, "/World/Drone", "Cube", position=[0, 0, 1], rigid=True)
    ext.scene.create(stage, "/World/Rock", "Cube", position=[5, 0, 0], rigid=True)
    ext.scene.timeline.mode = "pause"
    monkeypatch.setattr(advanced, "physics_manager", lambda: None)
    monkeypatch.setattr(physics, "body_state", lambda path, time_s, dynamics=False, stage=None: dict(
        path=path, world_position=measured[path], orientation_wxyz=[1.0, 0.0, 0.0, 0.0], backend="newton"))
    return ext.handler("sim.body.state")


@pytest.mark.anyio
async def test_body_state_says_when_the_picture_does_not_show_where_the_body_is(ext, monkeypatch):
    call = body_call(ext, monkeypatch, {"/World/Drone": [0.0, 0.0, 3.5], "/World/Rock": [5.0, 0.0, 0.0]})
    flown = await call(**ext.ids, path="/World/Drone")
    assert flown["status"] == "ok" and flown["render_pose"]["agrees"] is False and flown["render_pose"]["position_gap"] == pytest.approx(2.5)
    assert len(flown["warnings"]) == 1 and "do not show where the body is" in flown["warnings"][0]
    still = await call(**ext.ids, path="/World/Rock")
    assert still["render_pose"]["agrees"] is True and "warnings" not in still
    both = await call(**ext.ids, paths=["/World/Drone", "/World/Rock"])
    assert [("warnings" in body) for body in both["bodies"]] == [True, False]


# ---- sim.viewport: clean layout and caption ----

class Frame:
    def __init__(self, log):
        self.log = log

    def clear(self):
        self.log.append("clear")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@pytest.fixture
def ui(kit):
    log = kit.ui
    window = SimpleNamespace(get_frame=lambda name: log.append(name) or Frame(log))
    kit.modules["omni.kit.viewport.utility"].get_active_viewport_window = lambda: window
    kit.modules["omni.kit.viewport.utility"].get_active_viewport = lambda: None
    kit.modules["omni.kit.viewport.utility"].frame_viewport_selection = None
    kit.modules["omni.kit.undo"].disabled = None
    layout = lambda **kw: Frame(log)  # noqa: E731
    for name in ("VStack", "HStack", "ZStack"):
        setattr(kit.modules["omni.ui"], name, layout)
    kit.modules["omni.ui"].Spacer = lambda **kw: None
    kit.modules["omni.ui"].Rectangle = lambda **kw: None
    kit.modules["omni.ui"].Label = lambda text, **kw: log.append(("label", text))
    viewport._previous_layout.clear()
    return kit


@pytest.mark.anyio
async def test_the_clean_layout_hides_the_ui_and_default_puts_back_what_was_there(ext, ui):
    call = ext.handler("sim.viewport")
    clean = await call(**ext.ids, layout="clean")
    assert clean["status"] == "ok" and clean["layout"] == "clean"
    assert ui.settings == {"/app/window/hideUi": True, "/app/viewport/grid/enabled": False, "/app/viewport/outline/enabled": False}
    await call(**ext.ids, layout="clean")  # twice: the remembered values are still the originals
    restored = await call(**ext.ids, layout="default")
    assert restored["layout"] == "default"
    assert ui.settings["/app/window/hideUi"] is False and ui.settings["/app/viewport/grid/enabled"] is True
    assert ui.settings["/app/viewport/outline/enabled"] is False, "a setting that had no value before is left alone"


@pytest.mark.anyio
async def test_a_caption_is_drawn_over_the_viewport_and_an_empty_one_removes_it(ext, ui):
    call = ext.handler("sim.viewport")
    shown = await call(**ext.ids, caption="PhysX hover, kp=4")
    assert shown == {"status": "ok", "caption": "PhysX hover, kp=4"}
    assert ui.ui[:2] == [viewport.CAPTION_FRAME, "clear"] and ("label", "PhysX hover, kp=4") in ui.ui
    ui.ui.clear()
    removed = await call(**ext.ids, caption="")
    assert removed["caption"] == "" and ui.ui == [viewport.CAPTION_FRAME, "clear"], "cleared, nothing drawn"
    for bad in ("x" * 121, "two\nlines"):
        refused = await call(**ext.ids, caption=bad)
        assert refused["status"] == "error" and refused["code"] == "INVALID_VALUE"
    refused = await call(**ext.ids, layout="tidy")
    assert refused["code"] == "INVALID_VALUE"


@pytest.mark.anyio
async def test_a_headless_app_has_no_viewport_to_caption_and_says_what_to_use(ext, ui):
    ui.modules["omni.kit.viewport.utility"].get_active_viewport_window = lambda: None
    refused = await ext.handler("sim.viewport")(**ext.ids, caption="run 7")
    assert refused["code"] == "NO_VIEWPORT" and "sim__trajectory__replay" in refused["message"]
