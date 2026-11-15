"""Single grouped FastMCP entry point; server wiring is deliberately separate."""

import asyncio
import json
from typing import Annotated, Literal

from mcp.types import ToolAnnotations
from pydantic import Field

from .bridge import RoboSimBridge


def register_robosim_tools(server, *, bridge=None, workspace=None, root=None):
    bridge = bridge or RoboSimBridge(workspace=workspace, root=root)

    @server.tool(name="robosim", annotations=ToolAnnotations(
        readOnlyHint=False, destructiveHint=True, idempotentHint=False, openWorldHint=True,
    ))
    async def robosim(
        action: Annotated[Literal["status", "start", "scene_template", "update_scene", "reference", "snapshot",
                                  "compare", "rollout", "tune_push", "compare_take", "stop", "report"], Field(description="Action to perform.")],
        run: Annotated[str | None, Field(description="Exact run name from start. status: omit for a compact list. scene_template: write the file there.")] = None,
        name: Annotated[str | None, Field(description="start: short run name; letters, digits, underscores and hyphens.")] = None,
        scene: Annotated[str | None, Field(description="start/update_scene, advanced: absolute trusted scene .py (runs code, not sandboxed). Else use template.")] = None,
        device: Annotated[Literal["cpu", "cuda:0"] | None, Field(description="start (required): cpu is plain MuJoCo, cuda:0 is Newton MuJoCo-Warp. Runs inherit it.")] = None,
        video: Annotated[str | None, Field(description="reference: absolute local video file; copied before extracting frames.")] = None,
        keys: Annotated[list[str] | None, Field(description="compare/compare_take: RSS object masks, e.g. ['block=red'] or ['block=#d01a10~0.1'].")] = None,
        take: Annotated[str | None, Field(description="rollout/compare_take: take name. tune_push: base name; trials are NAME-1, NAME-2.")] = None,
        routine: Annotated[str | None, Field(description="rollout: 'push' (the template's default), 'settle', a tower's 'pull', or a routine your scene defines.")] = None,
        camera: Annotated[str | None, Field(description="Camera name from start's RSS status; omit for the scene's first camera.")] = None,
        at: Annotated[str | None, Field(description="reference: RSS anchors in clip seconds, e.g. '0:start,3:middle,6:end'; default from start_s/end_s.")] = None,
        shape: Annotated[Literal["box", "tee", "jenga", "tower"], Field(description="scene_template: tee, box, jenga (one block) or tower (18 x 3 blocks), unless template gives one.")] = "tee",
        push: Annotated[dict | None, Field(description="rollout/tune_push: {'waypoints': [[s, x, y], ...], 'object_velocity': [vx, vy, wz]}. Tower: {'block', 'distance'}.")] = None,
        duration_s: Annotated[float | None, Field(description="rollout: simulated seconds; default the last waypoint time. Not with steps.")] = None,
        steps: Annotated[int | None, Field(description="rollout: control steps instead of duration_s.", ge=1)] = None,
        record_fps: Annotated[float, Field(description="rollout: frames per simulated second to record (nearest whole step stride).", gt=0)] = 10.0,
        start_s: Annotated[float | None, Field(description="reference: clip window start; compare_take: clip window start (default: reference's).", ge=0)] = None,
        end_s: Annotated[float | None, Field(description="reference/compare_take: clip window end in seconds.", ge=0)] = None,
        sim_start_s: Annotated[float | None, Field(description="compare_take: simulated time matching start_s; default 0.", ge=0)] = None,
        sim_end_s: Annotated[float | None, Field(description="compare_take: simulated time matching end_s; default same length as the clip window.", ge=0)] = None,
        size: Annotated[str, Field(description="snapshot/compare: WIDTHxHEIGHT pixels.")] = "320x240",
        width: Annotated[int, Field(description="reference: contact-sheet tile width.", ge=1, le=4096)] = 320,
        port: Annotated[int | None, Field(description="start: viewer port; omit to pick a free one. RSS listens on all interfaces.", ge=1024, le=65535)] = None,
        wait_s: Annotated[float, Field(description="Wait at most this many seconds; poll status with run for completion.", ge=0, le=120)] = 0,
        allow_concurrent_gpu: Annotated[bool, Field(description="start: knowingly permit cuda:0 despite other GPU workloads or an unavailable GPU check.")] = False,
        template: Annotated[dict | None, Field(description="start/update_scene/scene_template: plain data {shape, size, position, yaw, color, camera, push}; a tower has more.")] = None,
        all: Annotated[bool, Field(description="status without run: every run in full, not live servers and the 5 newest.")] = False,
        max_trials: Annotated[int | None, Field(description="tune_push: most rollouts to try (default 6, at most 12).", ge=1, le=12)] = None,
    ) -> str:
        """RoboSimStudio real-to-sim in its own Python, no Isaac bridge needed. start a scene from plain-data template
        (frozen until update_scene), reference a clip, compare, tune_push, rollout, compare_take, report, stop. Compares
        images; no mass or friction fitting. Scene code is not sandboxed; cuda:0 uses the GPU. Long actions: poll status.
        """
        arguments = locals().copy()
        arguments.pop("bridge", None)
        arguments["show_all"] = arguments.pop("all")
        result = await asyncio.to_thread(bridge.call, **arguments)
        return json.dumps(result)

    return bridge
