"""Real MCP edge checks after the core/equivalents suite in a disposable session.

Approve the original package fixture as `package` before launch. This never uses
run_python, internal Kit APIs, or a stage reset. Results persist after each call.
"""

import argparse
import asyncio
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time

from mcp import ClientSession
from mcp.client.stdio import stdio_client
from scene_mcp_client import call, parameters


async def validate(port, output):
    records = []
    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, code=None, **args):
                item = await call(client, name, args)
                records.append(item)
                Path(output).write_text(json.dumps(records, indent=2))
                assert item["isError"] == (code is not None), item
                if code:
                    payload = item["result"]
                    if isinstance(payload, str):
                        payload = json.loads(payload[payload.index("{") :])
                    assert payload["code"] == code, item
                return item["result"]

            info = await invoke("sim__stage__info")
            assert info["stopped"] and info["app_ready"], info
            t = {k: info[k] for k in ("session_id", "stage_id")}
            for field in t:
                await invoke(
                    "sim__prim__create",
                    code="STALE_TARGET",
                    **{**t, field: "stale-unrelated-target"},
                    path="/World/ShouldNotExist",
                    kind="Cube",
                )
            await invoke(
                "sim__prim__get", code="NOT_FOUND", **t, path="/World/ShouldNotExist"
            )
            await invoke(
                "sim__simulation__step", code="PHYSICS_NOT_READY", **t, steps=1
            )
            await invoke(
                "sim__body__state", code="PHYSICS_NOT_READY", **t, path="/World/Falling"
            )
            await invoke(
                "sim__camera__create",
                code="LIMIT",
                **t,
                path="/World/InvalidCamera",
                width=1025,
            )
            await invoke(
                "sim__prim__get", code="NOT_FOUND", **t, path="/World/InvalidCamera"
            )
            package = await invoke(
                "assets__info", **t, root_id="package", relative_path="assembly.usdz"
            )
            assert package["packages"][0]["member_count"] == 2, package
            assert not package["sandboxed"]
            await invoke(
                "assets__import_path",
                **t,
                root_id="package",
                relative_path="assembly.usdz",
                prim_path="/World/Package",
                position=[-2, 0, 0.5],
            )
            cube = await invoke(
                "sim__prim__get", **t, path="/World/Package/Asset/Geometry"
            )
            assert cube["type"] == "Cube" and cube["world_position"] == [-2, 0, 0.5], (
                cube
            )
            saved = await invoke(
                "sim__stage__save",
                **t,
                filename="package-" + str(time.time_ns()) + ".usda",
            )
            assert Path(saved["file"]).is_file(), saved
            # Headless OpenUSD is only an additional export check; the imported
            # package above was composed and inspected in the selected live Kit.
            from pxr import Usd

            reopened = Usd.Stage.Open(saved["file"])
            assert (
                reopened.GetPrimAtPath("/World/Package/Asset/Geometry").GetTypeName()
                == "Cube"
            )
            await invoke(
                "sim__stage__save",
                code="CONFLICT",
                **t,
                filename=Path(saved["file"]).name,
            )
            # A newly re-created camera must produce a new frame after cleanup.
            await invoke("sim__timeline", **t, action="play")
            await asyncio.sleep(3)
            await invoke(
                "sim__simulation__step", code="PHYSICS_NOT_READY", **t, steps=1
            )
            await invoke("sim__timeline", **t, action="pause")
            await invoke("sim__simulation__step", code="LIMIT", **t, steps=121)
            await invoke(
                "sim__lidar__read",
                code="SENSOR_PAUSED",
                **t,
                path="/World/Lidar",
                wait_for_render=True,
            )
            image = await invoke(
                "sim__camera__capture",
                **t,
                path="/World/TestCamera",
                filename="recreated-" + str(time.time_ns()) + ".png",
            )
            render = image["render_observation"]
            assert (
                render["waited"]
                and render["frame_number_before"] != render["frame_number_after"]
            )
            await invoke("sim__timeline", **t, action="stop")
            await invoke("sim__prim__delete", **t, path="/World/Package", confirm=True)
            await invoke("sim__prim__get", code="NOT_FOUND", **t, path="/World/Package")
            after = await invoke("sim__stage__info")
            assert all(
                after[k] == info[k] for k in ("session_id", "stage_id", "source")
            ), after
            assert after["sensor_resources"]["count"] == 2, after
            print(
                json.dumps(
                    {
                        "passed": True,
                        "calls": len(records),
                        "target": t,
                        "saved": saved,
                        "image": image,
                    }
                )
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    asyncio.run(validate(args.port, args.output))
