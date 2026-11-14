"""Live dedicated-tool assertions after validate_scene_tools.py on a disposable stage.

Requires approved `fixture` (including minimal.urdf), `agilebot` and explicit Kit
script opt-in. All product operations use real stdio MCP; no raw Python endpoint.
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
from scene_mcp_client import parameters, call


def require_root(listing, root_id):
    """Stop with the fix when a needed asset root is not approved; there is no fallback robot."""
    if not any(root.get("root_id") == root_id for root in listing.get("roots", [])):
        raise SystemExit(f"Asset root '{root_id}' is not approved. Clone the Agilebot gbt-c5a folder named in "
                         "docs/VENDORING.md and approve it as in docs/TOOLS-SCENE.md, then run this again.")


async def validate(port, output, backend=None):
    records = []
    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as c:
            await c.initialize()

            async def invoke(name, error=False, **args):
                item = await call(c, name, args)
                records.append(item)
                Path(output).write_text(json.dumps(records, indent=2))
                print(name, item["isError"], flush=True)
                assert item["isError"] == error, item
                return item["result"]

            info = await invoke("sim__stage__info")
            t = {k: info[k] for k in ("session_id", "stage_id")}
            await invoke("assets__search", query="robot")
            await invoke(
                "sim__prim__create",
                **t,
                path="/World/Visual",
                kind="Cube",
                position=[0, 0, 1],
            )
            await invoke(
                "sim__prim__create",
                **t,
                path="/World/Floor",
                kind="Cube",
                size=1,
                scale=[10, 10, 0.1],
                position=[0, 0, -0.05],
                collision=True,
            )
            await invoke(
                "sim__prim__clone", **t, path="/World/Visual", new_path="/World/Clone"
            )
            await invoke("sim__prim__delete", **t, path="/World/Clone", confirm=True)
            await invoke("sim__prim__get", error=True, **t, path="/World/Clone")
            light = await invoke(
                "sim__light__create",
                **t,
                path="/World/Light",
                kind="DomeLight",
                intensity=700,
            )
            assert light["intensity"] == 700
            light = await invoke(
                "sim__light__modify",
                **t,
                path="/World/Light",
                intensity=1000,
                color=[1, 0.9, 0.7],
            )
            assert light["intensity"] == 1000
            await invoke(
                "sim__material__create",
                **t,
                path="/World/Blue",
                color=[0.05, 0.3, 0.8],
                roughness=0.35,
            )
            binding = await invoke(
                "sim__material__apply",
                **t,
                path="/World/Visual",
                material_path="/World/Blue",
            )
            assert binding["material"] == "/World/Blue"
            await invoke(
                "sim__material__create",
                **t,
                path="/World/Contact",
                kind="physics",
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.1,
            )
            assert (
                await invoke(
                    "sim__material__apply",
                    **t,
                    path="/World/Floor",
                    material_path="/World/Contact",
                )
            )["purpose"] == "physics"
            physics = await invoke(
                "sim__physics__configure", **t, path="/World/Physics", dt=1 / 120
            )
            assert abs(physics["dt"] - 1 / 120) < 1e-8
            joints = await invoke(
                "sim__joints__inspect", **t, path="/World/Parent/Robot"
            )
            assert any(j["type"] == "PhysicsRevoluteJoint" for j in joints["joints"])
            await invoke(
                "sim__joints__drive",
                **t,
                path="/World/Parent/Robot/Asset/Elbow",
                target=0.2,
                stiffness=100,
                damping=10,
            )
            await invoke(
                "sim__joints__drive",
                error=True,
                **t,
                path="/World/Parent/Robot/Asset/Elbow",
                target=2,
                stiffness=100,
                damping=10,
            )
            await invoke(
                "sim__camera__create",
                **t,
                path="/World/TestCamera",
                position=[3, -5, 3],
                target=[0, 0, 1],
                frame="world",
                width=320,
                height=240,
            )
            await invoke(
                "sim__lidar__create", **t, path="/World/Lidar", position=[-3, 0, 1]
            )
            require_root(await invoke("assets__roots"), "agilebot")
            await invoke(
                "assets__info", **t, root_id="agilebot", relative_path="gbt-c5a.usd"
            )
            await invoke(
                "assets__import_path",
                **t,
                root_id="agilebot",
                relative_path="gbt-c5a.usd",
                prim_path="/World/Agilebot",
                position=[3, 0, 0],
            )
            await invoke(
                "sim__controller__create",
                **t,
                path="/World/Controller",
                code="def compute(db):\n return True\n",
            )
            before = await invoke("sim__stage__info")
            imported_urdf = await invoke(
                "assets__import_path",
                **t,
                root_id="fixture",
                relative_path="minimal.urdf",
                prim_path="/World/Urdf",
                position=[-1, 0, 1],
            )
            # Native conversion may succeed while its Physics variant composes
            # geometry only. Verify the actual imported subtree independently.
            assert imported_urdf["converted_physics"]["authored"]["rigid_bodies"] >= 1
            urdf_physics = await invoke(
                "sim__physics__inspect", **t, root="/World/Urdf", limit=100
            )
            assert not urdf_physics["truncated"], urdf_physics
            assert any(n["collision"] for n in urdf_physics["nodes"]), urdf_physics
            assert any(
                n["rigid_body"] and n["mass_authored"] and abs(n["mass"] - 1) < 1e-6
                for n in urdf_physics["nodes"]
            ), urdf_physics
            after = await invoke("sim__stage__info")
            assert (
                before["source"] == after["source"]
                and before["stage_id"] == after["stage_id"]
                and before["session_id"] == after["session_id"]
            )
            marker = 'from pxr import Usd, UsdGeom, Gf\nimport omni.usd\ndef compute(db):\n st=omni.usd.get_context().get_stage()\n p=st.GetPrimAtPath("/World/Visual")\n with Usd.EditContext(st,st.GetSessionLayer()):\n  UsdGeom.Xformable(p).GetOrderedXformOps()[0].Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,1)))\n return True\n'
            await invoke(
                "sim__controller__update", **t, path="/World/Controller", code=marker
            )
            await invoke("sim__viewport", **t, path="/World")
            await invoke("sim__timeline", **t, action="play")
            await asyncio.sleep(5)
            await invoke("sim__timeline", **t, action="pause")
            state = await invoke(
                "sim__joints__state", **t, path="/World/Parent/Robot/Asset"
            )
            assert abs(state["positions"][0] - 0.2) < 0.03, state
            await invoke(
                "sim__joints__command",
                **t,
                path="/World/Parent/Robot/Asset",
                positions=[-0.2],
            )
            await invoke(
                "sim__joints__command",
                error=True,
                **t,
                path="/World/Parent/Robot/Asset",
                positions=[2],
            )
            step = await invoke(
                "sim__simulation__step",
                **t,
                steps=120,
                observe_prims=["/World/Falling"],
                observe_joints=["/World/Parent/Robot/Asset"],
            )
            assert abs(step["measured_delta"] - 1) < 1e-5, step
            assert abs(step["articulations"][0]["positions"][0] + 0.2) < 0.04, step
            assert 0.05 < step["bodies"][0]["world_position"][2] < 0.2, (
                step
            )  # settled on authored static floor
            displayed = await invoke("sim__prim__get", **t, path="/World/Falling")
            assert backend is None or step["backend"] == backend, step
            if step["backend"] == "physx":
                assert all(
                    abs(a - b) < 1e-5
                    for a, b in zip(
                        displayed["world_position"], step["bodies"][0]["world_position"]
                    )
                )
            else:
                assert displayed["world_position"] == [2, 0, 3], displayed
                assert step["fabric_updated"]
            image = await invoke(
                "sim__camera__capture",
                **t,
                path="/World/TestCamera",
                filename="camera-" + str(time.time_ns()) + ".png",
            )
            assert image["width"] == 320 and image["height"] == 240
            lidar = await invoke(
                "sim__lidar__read", **t, path="/World/Lidar", limit=256
            )
            assert lidar["valid_count"] > 0
            # The near face of Visual at x=-.5 is 2.5 m from this sensor.
            assert any(abs(p[0] - 2.5) < 0.03 for p in lidar["points"]), lidar
            await invoke("sim__logs__tail", **t, limit=10)
            await invoke("sim__timeline", **t, action="stop")
            await invoke(
                "sim__controller__update",
                **t,
                path="/World/Controller",
                code=marker.replace("(0,0,1)", "(0,0,2)"),
            )
            await invoke("sim__timeline", **t, action="play")
            await asyncio.sleep(0.5)
            await invoke("sim__timeline", **t, action="pause")
            assert (await invoke("sim__prim__get", **t, path="/World/Visual"))[
                "world_position"
            ] == [0, 0, 2]
            await invoke("sim__timeline", **t, action="stop")
            saved = await invoke(
                "sim__stage__save",
                **t,
                filename="equivalents-" + str(time.time_ns()) + ".usda",
            )
            await invoke(
                "sim__prim__delete",
                **t,
                path="/World/TestCamera",
                confirm=True,
            )
            await invoke("sim__prim__get", error=True, **t, path="/World/TestCamera")
            await invoke(
                "sim__camera__capture",
                error=True,
                **t,
                path="/World/TestCamera",
                filename="deleted.png",
            )
            # Re-create at the same path to exercise native resource cleanup.
            await invoke(
                "sim__camera__create",
                **t,
                path="/World/TestCamera",
                width=320,
                height=240,
                position=[3, -5, 3],
                target=[0, 0, 1],
                frame="world",
            )
            # Clear runs only after the user-visible captures in a separate phase.
            print(
                json.dumps(
                    dict(
                        passed=True,
                        calls=len(records),
                        pid=info["pid"],
                        session=t,
                        image=image,
                        saved=saved,
                        point_count=lidar["valid_count"],
                    )
                )
            )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--output", required=True)
    a = p.parse_args()
    asyncio.run(validate(a.port, a.output))
