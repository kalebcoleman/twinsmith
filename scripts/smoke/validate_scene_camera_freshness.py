"""Real MCP cold camera/paused physics regression; no simulator internals.

Run after validate_scene_tools.py in an owned disposable session. No sleeping
warm-up hides the first before/after readback. Inspect pixels and runtime pose.
"""

import argparse
import asyncio
import json
import uuid
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from PIL import Image
from mcp import ClientSession
from mcp.client.stdio import stdio_client
from scene_mcp_client import call, parameters


async def validate(port, output, cycles=3):
    records, metrics = [], []
    out = Path(output)
    prefix = "/World/ColdCapture"
    run_id = uuid.uuid4().hex[:8]
    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, **args):
                record = await call(client, name, args)
                records.append(record)
                out.write_text(
                    json.dumps(dict(records=records, metrics=metrics), indent=2)
                )
                assert not record["isError"], record
                return record["result"]

            info = await invoke("sim__stage__info")
            assert info["stopped"]
            target = {k: info[k] for k in ("session_id", "stage_id")}
            found = await invoke(
                "sim__prim__find", **target, type_name="PhysicsScene", limit=10
            )
            if not found["paths"]:
                await invoke(
                    "sim__prim__create",
                    **target,
                    path="/World/Physics",
                    kind="PhysicsScene",
                )
            await invoke(
                "sim__physics__configure",
                **target,
                path="/World/Physics",
                gravity=[0, 0, -9.81],
                dt=1 / 120,
            )
            await invoke("sim__prim__create", **target, path=prefix, kind="Xform")
            await invoke(
                "sim__light__create",
                **target,
                path=prefix + "/Light",
                kind="DomeLight",
                intensity=1000,
            )
            await invoke(
                "sim__prim__create",
                **target,
                path=prefix + "/Floor",
                kind="Cube",
                position=[80, 0, -0.05],
                scale=[8, 8, 0.1],
                collision=True,
            )
            await invoke(
                "sim__prim__create",
                **target,
                path=prefix + "/Sample",
                kind="Cube",
                position=[80, 0, 2.5],
                size=0.25,
                rigid=True,
            )
            await invoke(
                "sim__material__create",
                **target,
                path=prefix + "/Red",
                color=[0.9, 0.01, 0.01],
            )
            await invoke(
                "sim__material__apply",
                **target,
                path=prefix + "/Sample",
                material_path=prefix + "/Red",
            )
            for cycle in range(cycles):
                for index, (width, height) in enumerate(
                    [(64, 64), (320, 240), (1024, 768)]
                ):
                    path = prefix + f"/Camera{index}"
                    await invoke(
                        "sim__camera__create",
                        **target,
                        path=path,
                        width=width,
                        height=height,
                        position=[80, -12, 5],
                        target=[80, 0, 1.3],
                        frame="world",
                    )
                    await invoke("sim__timeline", **target, action="play")
                    await invoke("sim__timeline", **target, action="pause")
                    times = await invoke("sim__stage__info")

                    async def capture(phase):
                        value = await invoke(
                            "sim__camera__capture",
                            **target,
                            path=path,
                            filename=f"cold-{run_id}-{cycle}-{index}-{phase}.png",
                        )
                        with Image.open(value["file"]) as image:
                            rgb = np.asarray(image).astype(float)
                        mask = (
                            (rgb[:, :, 0] > 1.5 * rgb[:, :, 1])
                            & (rgb[:, :, 0] > 1.5 * rgb[:, :, 2])
                            & (rgb[:, :, 0] > 50)
                        )
                        assert mask.any(), value
                        observation = value["render_observation"]
                        assert observation["cpu_transfer_completed"]
                        assert (
                            observation["timeline_time_before"]
                            == observation["timeline_time_after"]
                        )
                        assert value["adapter"] == "hydra-renderer-cpu-v1"
                        return float(np.nonzero(mask)[0].mean())

                    before = await capture("before")
                    for steps in (10, 30, 80, 120):
                        observed = await invoke(
                            "sim__simulation__step",
                            **target,
                            steps=steps,
                            observe_prims=[prefix + "/Sample"],
                        )
                    after = await capture("after")
                    body = await invoke(
                        "sim__body__state", **target, path=prefix + "/Sample"
                    )
                    metric = dict(
                        cycle=cycle,
                        resolution=[width, height],
                        before_red_y=before,
                        after_red_y=after,
                        body=body,
                        last_step=observed,
                        state_before=times,
                    )
                    metrics.append(metric)
                    out.write_text(
                        json.dumps(dict(records=records, metrics=metrics), indent=2)
                    )
                    assert after - before > height * 0.15, metric
                    # This verifies the runtime observation as well as pixels.
                    assert abs(body["world_position"][2] - 0.125) < 0.02, body
                    await invoke("sim__timeline", **target, action="stop")
                    await invoke("sim__prim__delete", **target, path=path, confirm=True)
    print(json.dumps(dict(passed=True, captures=len(metrics) * 2, cycles=cycles)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=3)
    args = parser.parse_args()
    asyncio.run(validate(args.port, args.output, args.cycles))
