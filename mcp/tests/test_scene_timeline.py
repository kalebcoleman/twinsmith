"""Passive native Stop observation and the actual asynchronous tool boundary."""

import asyncio
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/sim-tools"))
from twinsmith.project_assets import AssetError
from sim_tools import timeline


@pytest.fixture
def anyio_backend():
    return "asyncio"


@pytest.fixture
def native(monkeypatch):
    state = SimpleNamespace(name="STOPPING")

    async def complete():
        state.name = "STOPPED"

    orchestrator = SimpleNamespace(get_status=lambda: state, wait_until_complete_async=complete)
    monkeypatch.setitem(sys.modules, "omni.replicator.core", SimpleNamespace(orchestrator=orchestrator))
    return state


@pytest.mark.anyio
async def test_explicit_stop_completes_stuck_reset_schedule(native):
    frames = []

    async def update():
        frames.append(native.name)

    result = await timeline.complete_requested_stop(update, lambda: None)
    assert frames == ["STOPPED"] * 6
    assert result == {"observed": "STOPPED", "updates": 6, "completion": "wait_until_complete_async"}


@pytest.mark.anyio
@pytest.mark.parametrize("change", ["stage", "native_operation", "module"])
async def test_native_completion_cannot_resume_on_changed_target(native, monkeypatch, change):
    entered, resume = asyncio.Event(), asyncio.Event()
    mutations = []
    changed = False

    async def complete():
        entered.set()
        await resume.wait()
        mutations.append("native-finalization")

    orchestrator = sys.modules["omni.replicator.core"].orchestrator
    orchestrator.wait_until_complete_async = complete

    def validate():
        if changed and change == "stage":
            raise AssetError("STALE_TARGET", "stage replaced")

    task = asyncio.create_task(timeline.complete_requested_stop(lambda: asyncio.sleep(0), validate))
    await entered.wait()
    changed = True
    if change == "native_operation":
        native.name = "STARTED"
    elif change == "module":
        monkeypatch.setitem(sys.modules, "omni.replicator.core", SimpleNamespace(orchestrator=orchestrator))
    resume.set()
    with pytest.raises(AssetError) as error:
        await task
    assert error.value.code == ("STALE_TARGET" if change == "stage" else "TIMELINE_INTERRUPTED")
    assert not mutations


@pytest.mark.anyio
@pytest.mark.parametrize("cancel", [False, True])
async def test_completion_timeout_and_cancellation_leave_no_running_coroutine(native, cancel):
    entered, resume = asyncio.Event(), asyncio.Event()
    effects = []

    async def complete():
        try:
            entered.set()
            await resume.wait()
            effects.append("late-write")
        finally:
            effects.append("closed")

    sys.modules["omni.replicator.core"].orchestrator.wait_until_complete_async = complete
    task = asyncio.create_task(timeline.complete_requested_stop(lambda: asyncio.sleep(0), lambda: None, timeout=.02))
    await entered.wait()
    if cancel:
        task.cancel()
    with pytest.raises(asyncio.CancelledError if cancel else AssetError) as error:
        await task
    if not cancel:
        assert error.value.code == "TIMELINE_BUSY"
    resume.set()
    await asyncio.sleep(0)
    assert effects == ["closed"]


@pytest.mark.anyio
async def test_native_completion_exception_propagates_without_retry(native):
    calls = []

    async def complete():
        calls.append(1)
        await asyncio.sleep(0)
        raise RuntimeError("native partial failure")

    sys.modules["omni.replicator.core"].orchestrator.wait_until_complete_async = complete
    with pytest.raises(RuntimeError, match="native partial failure"):
        await timeline.complete_requested_stop(lambda: asyncio.sleep(0), lambda: None)
    assert calls == [1]


@pytest.mark.anyio
async def test_missing_completion_api_refuses_and_passive_play_never_invokes_it(native):
    del sys.modules["omni.replicator.core"].orchestrator.wait_until_complete_async
    with pytest.raises(AssetError) as error:
        await timeline.complete_requested_stop(lambda: asyncio.sleep(0), lambda: None)
    assert error.value.code == "TIMELINE_API_UNAVAILABLE"
    native.name = "STOPPED"
    assert await timeline.complete_requested_stop(None, lambda: None) == {"observed": "STOPPED", "updates": 0}


@pytest.mark.anyio
async def test_failed_native_await_on_replaced_stage_does_not_handle_error_in_new_stage(native):
    entered = asyncio.Event()
    wake = asyncio.get_running_loop().create_future()
    changed = False
    effects = []

    async def complete():
        entered.set()
        try:
            await wake
        except RuntimeError:
            effects.append("native-exception-mutation")

    sys.modules["omni.replicator.core"].orchestrator.wait_until_complete_async = complete

    def validate():
        if changed:
            raise AssetError("STALE_TARGET", "stage replaced")

    task = asyncio.create_task(timeline.complete_requested_stop(lambda: asyncio.sleep(0), validate))
    await entered.wait()
    changed = True
    wake.set_exception(RuntimeError("native wait failed"))
    with pytest.raises(AssetError, match="stage replaced"):
        await task
    assert not effects


@pytest.mark.anyio
async def test_completion_that_does_not_stop_is_not_reported_successful(native):
    async def incomplete():
        pass

    sys.modules["omni.replicator.core"].orchestrator.wait_until_complete_async = incomplete
    with pytest.raises(AssetError) as error:
        await timeline.complete_requested_stop(lambda: asyncio.sleep(0), lambda: None)
    assert error.value.code == "TIMELINE_BUSY"


@pytest.mark.anyio
async def test_waits_for_render_drain_then_reobserves_stopped(native):
    observations = []

    async def update():
        observations.append(native.name)
        if len(observations) == 2:
            native.name = "STOPPED"

    validated = []
    result = await timeline.settle_stop(update, lambda: validated.append(native.name))
    assert observations == ["STOPPING", "STOPPING", "STOPPED"]
    assert result == {"observed": "STOPPED", "updates": 3}
    assert validated[-1] == "STOPPED"


@pytest.mark.anyio
@pytest.mark.parametrize("status", ["STOPPED", "STARTED", "STARTING", "PAUSED", "STEPPED", "STEPPING", "LOADING", "INITIALIZING", None])
async def test_does_not_drive_unrelated_or_absent_orchestrator(native, monkeypatch, status):
    native.name = status
    if status is None:
        monkeypatch.delitem(sys.modules, "omni.replicator.core")

    def update():
        pytest.fail("No pending native Stop; no extra update or orchestrator mutation")

    assert await timeline.settle_stop(update, lambda: None) == {"observed": status, "updates": 0}


@pytest.mark.anyio
@pytest.mark.parametrize("change", ["stage", "timeline", "orchestrator", "unloaded"])
async def test_changed_target_or_native_operation_aborts(native, monkeypatch, change):
    changed = False

    async def update():
        nonlocal changed
        changed = True
        if change == "orchestrator":
            native.name = "STARTED"
        elif change == "unloaded":
            monkeypatch.delitem(sys.modules, "omni.replicator.core")

    def validate():
        if changed and change in {"stage", "timeline"}:
            raise AssetError("STALE_TARGET" if change == "stage" else "TIMELINE_INTERRUPTED", change)

    with pytest.raises(AssetError) as error:
        await timeline.settle_stop(update, validate)
    assert error.value.code == ("STALE_TARGET" if change == "stage" else "TIMELINE_INTERRUPTED")


@pytest.mark.anyio
async def test_unfinished_native_stop_has_bounded_timeout(native):
    never = asyncio.get_running_loop().create_future()
    with pytest.raises(AssetError) as error:
        await timeline.settle_stop(lambda: never, lambda: None, timeout=.001)
    assert error.value.code == "TIMELINE_BUSY"


@pytest.mark.anyio
async def test_fast_unfinished_frames_are_bounded(native):
    frames = []

    async def update():
        frames.append(1)

    with pytest.raises(AssetError) as error:
        await timeline.settle_stop(update, lambda: None, timeout=10)
    assert error.value.code == "TIMELINE_BUSY" and len(frames) == 1200


@pytest.mark.anyio
async def test_cancellation_does_not_issue_any_native_action(native):
    frame = asyncio.get_running_loop().create_future()
    task = asyncio.create_task(timeline.settle_stop(lambda: frame, lambda: None))
    await asyncio.sleep(0)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert native.name == "STOPPING"


@pytest.mark.parametrize("api", [None, SimpleNamespace(get_status=lambda: object()), SimpleNamespace(get_status=lambda: SimpleNamespace(name="FUTURE"))])
def test_missing_or_unknown_loaded_api_refuses(monkeypatch, api):
    monkeypatch.setitem(sys.modules, "omni.replicator.core", SimpleNamespace(orchestrator=api))
    with pytest.raises(AssetError) as error:
        timeline.native_status()
    assert error.value.code == "TIMELINE_API_UNAVAILABLE"


@pytest.fixture
def handler(monkeypatch, native):
    pytest.importorskip("pxr.Usd")
    modules = {}
    for name in ["omni", "omni.ext", "omni.usd", "omni.timeline", "omni.kit", "omni.kit.app", "twinsmith_bridge"]:
        modules[name] = ModuleType(name)
        monkeypatch.setitem(sys.modules, name, modules[name])
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(modules[parent], child, modules[name])
    modules["omni.ext"].IExt = object
    modules["twinsmith_bridge"].get_mcp_bridge = lambda: None
    modules["twinsmith_bridge"].ToolDefinition = object
    path = Path(__file__).resolve().parents[2] / "exts/sim-tools/sim_tools/extension.py"
    spec = importlib.util.spec_from_file_location("sim_tools._test_timeline_extension", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    state = SimpleNamespace(mode="stop", frames=0, calls=[], changed=False, interrupt=False, restored=0, edit_target="root")
    stage = SimpleNamespace(GetEditTarget=lambda: state.edit_target)

    async def update():
        await asyncio.sleep(0)
        state.frames += 1
        if native.name == "STOPPING" and state.frames >= 2:
            native.name = "STOPPED"
            state.mode = "stop"
        if state.interrupt:
            state.mode = "stop"

    def target(session, stage):
        if (session, stage) != ("session", "stage") or state.changed:
            raise AssetError("STALE_TARGET", "Stage changed")

    def restore(stage):
        state.restored += 1
        return True

    ext = module.Extension()
    ext._operation_lock = asyncio.Lock()
    ext.scene = SimpleNamespace(
        target=target, stage=lambda: stage, end_runtime_edit=restore, _runtime_edit_target=None,
        timeline=SimpleNamespace(is_stopped=lambda: state.mode == "stop", is_playing=lambda: state.mode == "play", get_current_time=lambda: 0),
    )

    def dispatch(name, args, **kw):
        target(args.pop("session_id"), args.pop("stage_id"))
        if getattr(state, "failure", None):
            raise state.failure
        state.calls.append((args["action"], state.frames, native.name))
        state.mode = args["action"]
        if state.mode == "stop":
            native.name = "STOPPING"
        return {"action": args["action"]}

    ext.dispatch = dispatch
    modules["omni.kit.app"].get_app = lambda: SimpleNamespace(next_update_async=update)
    return ext.handler("sim.timeline"), state


@pytest.mark.anyio
async def test_actual_handler_preserves_specific_partial_effects_and_error_identity(handler):
    call, state = handler
    state.failure = AssetError(
        "VIEWPORT_FRAME_FAILED", "Native framing failed",
        effects="Session camera edits may have occurred.",
        camera="/World/Camera", status="ok",
    )
    result = await call(session_id="session", stage_id="stage", action="pause")
    assert result == {
        "status": "error", "code": "VIEWPORT_FRAME_FAILED", "message": "Native framing failed",
        "effects": "Session camera edits may have occurred.", "camera": "/World/Camera",
    }


@pytest.mark.anyio
async def test_actual_handler_waits_before_one_play(handler, native):
    call, state = handler
    result = await call(session_id="session", stage_id="stage", action="play")
    assert result["status"] == "ok" and result["playing"]
    assert state.calls == [("play", 3, "STOPPED")]


@pytest.mark.anyio
async def test_initialize_plays_then_pauses_and_reports_effects(handler):
    call, state = handler
    result = await call(session_id="session", stage_id="stage", action="initialize")
    assert result["status"] == "ok" and result["paused"] and not result["stopped"]
    assert result["action"] == "initialize" and not result["already_paused"]
    assert [action for action, _, _ in state.calls] == ["play", "pause"]
    assert "physics may have advanced" in result["effects"]


@pytest.mark.anyio
async def test_initialize_never_interrupts_a_running_timeline(handler):
    call, state = handler
    state.mode = "play"
    result = await call(session_id="session", stage_id="stage", action="initialize")
    assert result["code"] == "TIMELINE_ACTIVE" and not state.calls
    assert state.mode == "play"


@pytest.mark.anyio
async def test_initialize_leaves_a_paused_timeline_alone_and_rejects_resume(handler):
    call, state = handler
    state.mode = "pause"
    result = await call(session_id="session", stage_id="stage", action="initialize")
    assert result["status"] == "ok" and result["already_paused"] and not state.calls
    result = await call(session_id="session", stage_id="stage", action="initialize", resume=True)
    assert result["code"] == "INVALID_VALUE" and not state.calls


@pytest.mark.anyio
async def test_initialize_reports_interrupted_play_without_retrying(handler, native):
    call, state = handler
    native.name = "STOPPED"
    state.interrupt = True
    result = await call(session_id="session", stage_id="stage", action="initialize")
    assert result["code"] == "TIMELINE_INTERRUPTED" and len(state.calls) == 1
    assert "may have advanced" in result["effects"]


@pytest.mark.anyio
async def test_actual_stop_returns_after_native_drain(handler, native):
    call, state = handler
    state.mode = "play"
    native.name = "STARTED"
    result = await call(session_id="session", stage_id="stage", action="stop")
    assert result["status"] == "ok" and result["stopped"]
    assert result["native_stop_observation"] == {"observed": "STOPPED", "updates": 6, "completion": "wait_until_complete_async"}
    assert state.restored == 1 and len(state.calls) == 1


@pytest.mark.anyio
async def test_actual_handler_returns_observed_failure_without_replay(handler, native):
    call, state = handler
    native.name = "STOPPED"
    state.interrupt = True
    result = await call(session_id="session", stage_id="stage", action="play")
    assert result["status"] == "error" and result["code"] == "TIMELINE_INTERRUPTED"
    assert result["observed_state"]["stopped"] and len(state.calls) == 1


@pytest.mark.anyio
async def test_stale_target_never_issues_play(handler):
    call, state = handler
    result = await call(session_id="session", stage_id="old-stage", action="play")
    assert result["code"] == "STALE_TARGET" and not state.calls
