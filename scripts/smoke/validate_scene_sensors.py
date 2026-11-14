"""MCP-only camera/LiDAR lifetime and movement checks in a disposable stage.

Run after the core/equivalents suite. Creates an isolated /World/SensorStress
subtree, uses the existing physics scene, and deletes only its own sensors.
No internal simulator APIs. PNG inspection uses the returned local artifact.
"""

import argparse
import asyncio
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time
import numpy as np
from PIL import Image
from mcp import ClientSession
from mcp.client.stdio import stdio_client
from scene_mcp_client import call, parameters


async def wait_simulated(invoke, t, seconds, wall_limit=180.0, poll=0.5):
    """Play until the timeline has advanced *seconds*: a loaded host can run far slower than real time."""
    deadline = time.monotonic() + wall_limit
    while True:
        # Play is idempotent, and its result is the only MCP read of the timeline clock.
        state = await invoke("sim__timeline", **t, action="play")
        if state["time_s"] >= seconds:
            return state
        assert time.monotonic() < deadline, f"only {state['time_s']} s simulated after {wall_limit} s"
        await asyncio.sleep(poll)


async def validate(port, output, cycles=3):
    records, metrics = [], []
    out = Path(output)
    prefix = "/World/SensorStress"
    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, **args):
                item = await call(client, name, args)
                item["recorded_ns"] = time.time_ns()
                records.append(item)
                out.write_text(json.dumps(records, indent=2))
                assert not item["isError"], item
                return item["result"]

            info = await invoke("sim__stage__info")
            assert info["stopped"] and info["app_ready"]
            t = {k: info[k] for k in ("session_id", "stage_id")}
            initial_count = info["sensor_resources"]["count"]
            await invoke("sim__prim__create", **t, path=prefix, kind="Xform")
            for suffix, args in [
                (
                    "Floor",
                    dict(position=[10, 0, -0.05], scale=[10, 10, 0.1], collision=True),
                ),
                ("Box", dict(position=[10, 0, 2], size=0.25, rigid=True)),
                ("Wall", dict(position=[30, 2.55, 1], scale=[8, 0.1, 8])),
            ]:
                await invoke(
                    "sim__prim__create",
                    **t,
                    path=prefix + "/" + suffix,
                    kind="Cube",
                    **args,
                )
            await invoke(
                "sim__material__create",
                **t,
                path=prefix + "/Red",
                color=[0.9, 0.02, 0.01],
            )
            await invoke(
                "sim__material__apply",
                **t,
                path=prefix + "/Box",
                material_path=prefix + "/Red",
            )
            resolutions = [(64, 64), (320, 240), (1024, 768)]
            for i, (width, height) in enumerate(resolutions):
                await invoke(
                    "sim__camera__create",
                    **t,
                    path=prefix + f"/Camera{i}",
                    width=width,
                    height=height,
                    frame="world",
                    position=[10, -6, 3],
                    target=[10, 0, 1],
                )
            await invoke(
                "sim__lidar__create",
                **t,
                path=prefix + "/Lidar",
                position=[30, 0, 1],
                orientation=[2**-0.5, 0, 0, 2**-0.5],
            )
            # Annotators need an explicit playing warm-up before first data.
            # Keep the body at its spawn pose with zero gravity for the optical
            # baseline, then restore the measured test gravity while stopped.
            await invoke(
                "sim__physics__configure",
                **t,
                path="/World/Physics",
                gravity=[0, 0, 0],
                dt=1 / 120,
            )
            await invoke("sim__timeline", **t, action="play")
            await asyncio.sleep(3)
            await invoke("sim__timeline", **t, action="pause")

            def pixels(image):
                with Image.open(image["file"]) as im:
                    arr = np.asarray(im).copy()
                assert arr.shape == (image["height"], image["width"], 3)
                assert arr.std() > 5
                rgb = arr.astype(float)
                red = (
                    (rgb[:, :, 0] > 1.5 * rgb[:, :, 1])
                    & (rgb[:, :, 0] > 1.5 * rgb[:, :, 2])
                    & (rgb[:, :, 0] > 50)
                )
                centroid = float(np.nonzero(red)[0].mean()) if red.any() else None
                return dict(
                    std=float(arr.std()), red_pixels=int(red.sum()), red_y=centroid
                )

            before = await invoke(
                "sim__camera__capture",
                **t,
                path=prefix + "/Camera1",
                filename="stress-before.png",
            )
            before_metrics = pixels(before)
            assert before_metrics["red_pixels"] >= 4, before_metrics
            await invoke("sim__timeline", **t, action="stop")
            await invoke(
                "sim__physics__configure",
                **t,
                path="/World/Physics",
                gravity=[0, 0, -9.81],
                dt=1 / 120,
            )
            for cycle in range(cycles):
                if cycle:
                    await invoke(
                        "sim__prim__set_transform",
                        **t,
                        path=prefix + "/Wall",
                        position=[30, 4.55, 1],
                        scale=[8, 0.1, 8],
                    )
                await wait_simulated(invoke, t, seconds=3)
                lidar = await invoke(
                    "sim__lidar__read", **t, path=prefix + "/Lidar", limit=256
                )
                expected = 2.5 if cycle == 0 else 4.5
                points = np.asarray(lidar["points"])
                assert np.any(np.abs(points[:, 0] - expected) < 0.04), lidar
                transform = np.asarray(lidar["sensor_to_world"])
                world = np.column_stack((points, np.ones(len(points)))) @ transform
                assert np.any(np.abs(world[:, 1] - expected) < 0.04), lidar
                assert np.allclose(transform[3, :3], [30, 0, 1], atol=1e-4)
                await invoke("sim__timeline", **t, action="pause")
                for tick, steps in enumerate([1, 10, 20, 60, 120, 120] * 4):
                    observation = await invoke(
                        "sim__simulation__step",
                        **t,
                        steps=steps,
                        observe_prims=[prefix + "/Box"],
                    )
                    assert (
                        abs(
                            observation["measured_delta"]
                            - steps * observation["physics_dt"]
                        )
                        < 1e-6
                    )
                    assert (
                        abs(observation["bodies"][0]["world_position"][2] - 0.125)
                        < 0.04
                    )
                    for i, (width, height) in enumerate(resolutions):
                        image = await invoke(
                            "sim__camera__capture",
                            **t,
                            path=prefix + f"/Camera{i}",
                            filename=f"stress-c{cycle}-t{tick}-r{i}.png",
                        )
                        assert (image["width"], image["height"]) == (width, height)
                        render = image["render_observation"]
                        assert (
                            render["waited"]
                            and render["frame_number_before"]
                            != render["frame_number_after"]
                        )
                        measured = pixels(image)
                        if i == 1:
                            assert measured["red_pixels"] >= 4, measured
                            assert measured["red_y"] > before_metrics["red_y"] + 5, (
                                before_metrics,
                                measured,
                            )
                        metrics.append(
                            dict(
                                cycle=cycle,
                                tick=tick,
                                resolution=[width, height],
                                sha256=image["sha256"],
                                position=observation["bodies"][0]["world_position"],
                                **measured,
                            )
                        )
                await invoke(
                    "sim__stage__save", **t, filename=f"sensor-stress-cycle{cycle}.usda"
                )
                await invoke("sim__timeline", **t, action="stop")
            for suffix in ["Camera0", "Camera1", "Camera2", "Lidar"]:
                await invoke(
                    "sim__prim__delete", **t, path=prefix + "/" + suffix, confirm=True
                )
            after = await invoke("sim__stage__info")
            assert not after["device_error"]
            assert after["sensor_resources"]["count"] == initial_count, after
            out.with_name(out.stem + "-metrics.json").write_text(
                json.dumps(metrics, indent=2)
            )
            print(
                json.dumps(
                    dict(
                        passed=True,
                        captures=len(metrics) + 1,
                        calls=len(records),
                        target=t,
                        before_metrics=before_metrics,
                    )
                )
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, choices=[1, 2, 3], default=3)
    args = parser.parse_args()
    asyncio.run(validate(args.port, args.output, args.cycles))
