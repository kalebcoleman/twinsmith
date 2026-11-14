"""Who changed the stage or the timeline: attribution wording, stale targets and running-tool reports."""

import asyncio
from pathlib import Path
import sys

import pytest

pytest.importorskip("pxr.Usd")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/sim-tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/twinsmith.bridge"))
from twinsmith.project_assets import AssetError
from pxr import Usd
from sim_tools.core import STAGE_TOOLS, TIMELINE_TOOLS, Scene, attribute_change


class Timeline:
    stopped = True

    def is_stopped(self):
        return self.stopped

    def is_playing(self):
        return not self.stopped

    def get_current_time(self):
        return 0.0


OUTSIDE = attribute_change({}, STAGE_TOOLS)


def refusal(call, *args, **kw):
    with pytest.raises(AssetError) as error:
        call(*args, **kw)
    return error.value


def test_attribution_only_credits_or_blames_when_it_can_tell():
    assert attribute_change(None, STAGE_TOOLS)["origin"] == "unknown"
    assert attribute_change({"sim.stage.open": True}, STAGE_TOOLS) == dict(origin="tool", source="sim__stage__open")
    assert attribute_change({"sim.timeline": True}, TIMELINE_TOOLS)["source"] == "sim__timeline"
    assert attribute_change({"sim.occupancy_map": True}, TIMELINE_TOOLS)["origin"] == "tool"
    # A run_python new_stage one second after sim__stage__open.
    recent = attribute_change({"sim.stage.open": False, "run_python": True}, STAGE_TOOLS)
    assert recent["origin"] == "unattributed" and "run_python" in recent["source"]
    assert attribute_change({"sim.stage.open": False}, STAGE_TOOLS)["origin"] == "unattributed"
    for tool in ("run_python", "workcell.new_stage", "sim.ui.act"):
        assert attribute_change({tool: True}, STAGE_TOOLS)["origin"] == "unattributed"
    # Tools that cannot change a stage do not hide a person in the GUI.
    for tool in ("status", "sim.stage.info", "sim.logs.tail", "assets.search", "get_file"):
        assert attribute_change({tool: True}, STAGE_TOOLS)["origin"] == "outside_tools"
    assert attribute_change({"sim.stage.open": True}, TIMELINE_TOOLS)["origin"] == "outside_tools"


def swap(scene, attribution):
    old = scene.stage_info()
    new = Usd.Stage.CreateInMemory("next.usda")
    scene.record_event("stage", "stage closing", attribution, old["source"])
    scene.close_stage()
    scene.get_stage = lambda: new
    return old, new


@pytest.fixture
def scene(tmp_path):
    stage = Usd.Stage.CreateInMemory()
    return Scene(lambda: stage, Timeline(), output=tmp_path)


def test_stale_target_says_outside_only_when_that_is_what_happened(scene):
    old, new = swap(scene, OUTSIDE)
    error = refusal(scene.target, old["session_id"], old["stage_id"])
    assert error.code == "STALE_TARGET" and "outside this session's tools" in str(error)
    assert new.GetRootLayer().identifier in str(error) and error.details["stage_change"]["origin"] == "outside_tools"
    assert "monotonic" not in error.details["stage_change"]
    assert error.details["current_target"]["source"] == new.GetRootLayer().identifier


def test_stale_target_credits_the_tool_and_does_not_claim_outside(scene):
    old, _ = swap(scene, attribute_change({"sim.stage.open": True}, STAGE_TOOLS))
    text = str(refusal(scene.target, old["session_id"], old["stage_id"]))
    assert "by sim__stage__open" in text and "outside" not in text


def test_stale_target_with_a_running_tool_says_it_cannot_tell(scene):
    old, _ = swap(scene, attribute_change({"run_python": True}, STAGE_TOOLS))
    text = str(refusal(scene.target, old["session_id"], old["stage_id"]))
    assert "could not tell" in text and "run_python" in text and "outside this session" not in text


def test_stale_target_without_a_recorded_change_stays_generic(scene):
    old = scene.stage_info()
    scene.get_stage = lambda: Usd.Stage.CreateInMemory()
    error = refusal(scene.target, old["session_id"], old["stage_id"])
    assert "outside" not in str(error) and "stage_change" not in error.details
    unknown = refusal(scene.target, scene.session_id, "never-issued")
    assert "outside" not in str(unknown) and "changed" in str(unknown)


def test_stale_session_id_is_not_blamed_on_a_stage_change(scene):
    old = scene.stage_info()
    text = str(refusal(scene.target, "other-process", old["stage_id"]))
    assert "Kit restarted" in text and "outside" not in text


def test_event_history_and_retired_ids_stay_bounded(scene):
    for index in range(100):
        scene.record_event("timeline", "timeline playing", OUTSIDE)
        scene.stage_id = f"s{index}"
        scene.retire_stage()
    assert len(scene.events) == 32 and len(scene._retired_stages) == 16


def test_mutation_while_playing_names_the_stop_call_for_every_mutation(scene):
    info = scene.stage_info()
    scene.timeline.stopped = False
    error = refusal(scene.target, info["session_id"], info["stage_id"], mutation=True)
    assert error.code == "TIMELINE_ACTIVE" and "sim__timeline action=stop" in str(error)
    assert "start pose" in str(error)
    assert error.details["before_retry"] == dict(
        tool="sim__timeline",
        arguments=dict(session_id=info["session_id"], stage_id=info["stage_id"], action="stop"))
    assert "retry_with" not in error.details


def test_timeline_already_active_says_how_to_continue_and_who_started_it(scene):
    stage = scene.stage()
    scene.record_event("timeline", "timeline playing", OUTSIDE)
    scene.timeline.stopped = False
    error = refusal(scene.begin_runtime_edit, stage)
    assert error.code == "TIMELINE_ALREADY_ACTIVE"
    assert "action=stop, then action=play" in str(error) and "outside the MCP tools" in str(error)
    assert error.details["before_retry"]["arguments"]["action"] == "stop"
    assert scene._runtime_edit_target is None


def test_bridge_reports_running_and_just_finished_tools():
    from twinsmith_bridge.bridge import McpBridge, ToolDefinition

    bridge = McpBridge(schedule_coroutine=lambda coro: None)
    seen = []
    registrar = bridge.register_owner("t", "t", display_name="t")

    async def handler():
        seen.append(bridge.tool_activity())
        return {}

    registrar.add(ToolDefinition(name="work", description="d", parameters={}, handler=handler))
    assert bridge.tool_activity() == {}
    asyncio.run(bridge.dispatch("t.work", {}))
    assert seen == [{"t.work": True}]
    assert bridge.tool_activity() == {"t.work": False}
    assert bridge.tool_activity(recent_s=0) == {}
