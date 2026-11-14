"""Verify a saved stage opened at startup in a NEW disposable Kit process.

The test launcher opens the approved saved file before its main loop. Every
inspection, scene action and observation below uses the actual MCP interface.
The final sim__quick_start call (discard_changes=true) discards ONLY this disposable test stage.
"""

import argparse
import asyncio
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mcp import ClientSession
from mcp.client.stdio import stdio_client
from scene_mcp_client import call, parameters


async def validate(args):
    records = []
    saved = Path(args.saved).resolve()
    digest = hashlib.sha256(saved.read_bytes()).hexdigest()
    async with stdio_client(parameters(args.port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, code=None, **kw):
                item = await call(client, name, kw)
                records.append(item)
                Path(args.output).write_text(json.dumps(records, indent=2))
                assert item["isError"] == (code is not None), item
                result = item["result"]
                if code:
                    if isinstance(result, str):
                        result = json.loads(result[result.index("{") :])
                    assert result["code"] == code, item
                return result

            info = await invoke("sim__stage__info")
            assert info["app_ready"] and info["stopped"], info
            assert Path(info["source"]).resolve() == saved, info
            assert info["sensor_resources"]["count"] == 0, info
            t = {k: info[k] for k in ("session_id", "stage_id")}
            prim = await invoke("sim__prim__get", **t, path=args.body)
            assert abs(prim["world_position"][2] - args.expected_z) < 0.01, prim
            physics = await invoke("sim__physics__inspect", **t, root=args.body)
            assert (
                physics["nodes"][0]["rigid_body"] and physics["nodes"][0]["collision"]
            )
            await invoke(
                "sim__prim__create", **t, path="/World/ReopenVerification", kind="Xform"
            )
            x, y, z = prim["world_position"]
            await invoke(
                "sim__camera__create",
                **t,
                path="/World/ReopenVerification/Camera",
                position=[x + 3, y - 4, z + 3],
                target=[x, y, z],
                width=640,
                height=480,
                frame="world",
            )
            await invoke("sim__timeline", **t, action="play")
            await asyncio.sleep(3)
            await invoke("sim__timeline", **t, action="pause")
            before = await invoke(
                "sim__body__state", **t, path=args.body, include_dynamics=True
            )
            stepped = await invoke(
                "sim__simulation__step", **t, steps=120, observe_prims=[args.body]
            )
            after = await invoke(
                "sim__body__state", **t, path=args.body, include_dynamics=True
            )
            assert abs(after["world_position"][2] - args.expected_z) < 0.03, (
                before,
                after,
            )
            assert stepped["measured_delta"] > 0, stepped
            image = await invoke(
                "sim__camera__capture",
                **t,
                path="/World/ReopenVerification/Camera",
                filename="reopened-stage.png",
            )
            from PIL import Image
            import numpy as np

            pixels = np.asarray(Image.open(image["file"]).convert("RGB"))
            assert pixels.std() > 5 and pixels.shape == (480, 640, 3)
            await invoke("sim__viewport", **t, path=args.body.rsplit("/", 1)[0])
            await invoke("sim__timeline", **t, action="stop")
            await invoke("sim__quick_start", discard_changes=True)
            fresh = await invoke("sim__stage__info")
            assert fresh["stage_id"] != info["stage_id"]
            assert fresh["sensor_resources"]["count"] == 0, fresh
            await invoke("sim__prim__get", code="STALE_TARGET", **t, path=args.body)
            assert hashlib.sha256(saved.read_bytes()).hexdigest() == digest
            print(
                json.dumps(
                    dict(
                        passed=True,
                        calls=len(records),
                        saved_sha256=digest,
                        image=image,
                        runtime_after=after,
                        fresh=fresh,
                    )
                )
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--saved", required=True)
    parser.add_argument("--body", required=True)
    parser.add_argument("--expected-z", type=float, required=True)
    parser.add_argument("--output", required=True)
    asyncio.run(validate(parser.parse_args()))
