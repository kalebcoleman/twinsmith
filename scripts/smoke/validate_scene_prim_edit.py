"""Live validation: edit prims a pack did not create, then revert, on a USD file this run writes.

It opens a small authored scene (a table, a visual-only object, a multi-op Xform and an instance), moves
and gives physics to those prims, checks the object falls onto the table, proves the refusals, reverts,
and hashes the source files before and after. Then it opens a public robot and shows the root moves
while a link is refused.
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

SCENE = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        float physics:gravityMagnitude = 9.81
        float3 physics:gravityDirection = (0, 0, -1)
    }
    def Cube "Floor" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        double size = 1
        double3 xformOp:scale = (20, 20, 0.2)
        double3 xformOp:translate = (0, 0, -0.1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
    def Cube "Table" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        double size = 1
        double3 xformOp:scale = (0.8, 0.5, 0.05)
        double3 xformOp:translate = (0, 0, 0.5)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
    def Cube "Object"
    {
        double size = 0.2
        double3 xformOp:translate = (0, 0, 1.2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def Xform "Decor"
    {
        double3 xformOp:translate = (1, 0, 0)
        float3 xformOp:rotateXYZ = (0, 0, 30)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
    }
    def Xform "Animated"
    {
        double3 xformOp:translate.timeSamples = {
            0: (0, 0, 0),
            10: (2, 0, 0),
        }
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def Xform "Inst" (
        instanceable = true
        prepend references = @block.usda@</Block>
    )
    {
    }
}
"""

BLOCK = """#usda 1.0
(
    defaultPrim = "Block"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Block"
{
    def Cube "Body"
    {
        double size = 0.1
    }
}
"""


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def payload(result):
    """An MCP error arrives as text; its JSON envelope (with the code) follows the message."""
    if isinstance(result, dict):
        return result
    if isinstance(result, str):
        start = result.find("{")
        if start >= 0:
            try:
                return json.loads(result[start:])
            except ValueError:
                pass
    return {}


async def validate(port, output, directory, robot):
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    scene_file, block_file = folder / "m5-scene.usda", folder / "block.usda"
    scene_file.write_text(SCENE)
    block_file.write_text(BLOCK)
    hashes = {"scene": sha256(scene_file), "block": sha256(block_file)}
    records = []

    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, error=False, **args):
                item = await call(client, name, args)
                records.append(item)
                Path(output).write_text(json.dumps(records, indent=2))
                assert item["isError"] == error, item
                return payload(item["result"])

            opening = await invoke("sim__stage__info")
            opening_target = {k: opening[k] for k in ("session_id", "stage_id")}
            if not opening["stopped"]:
                await invoke("sim__timeline", **opening_target, action="stop")
            await invoke("sim__stage__open", **opening_target, path=str(scene_file),
                         discard_changes=True)
            info = await invoke("sim__stage__info")
            target = {k: info[k] for k in ("session_id", "stage_id")}
            report = dict(scene=str(scene_file), source_sha256=hashes, steps={})

            # Move prims the pack did not create: a table and a multi-op stack.
            table_before = await invoke("sim__prim__get", **target, path="/World/Table")
            object_authored = await invoke("sim__prim__get", **target, path="/World/Object")
            moved = await invoke("sim__prim__set_transform", **target, path="/World/Table",
                                 position=[0.25, 0, 0.5], frame="world")
            assert moved["before"]["world_position"] == table_before["world_position"]
            assert moved["world_position"] == [0.25, 0, 0.5]
            assert moved["source_files_changed"] is False

            decor = await invoke("sim__prim__set_transform", **target, path="/World/Decor",
                                 position=[1.5, 0, 0.2], frame="world")
            assert decor["world_position"] == [1.5, 0, 0.2]
            assert decor["authored"] == ["xformOp:transform", "xformOpOrder"]

            # Refusals before any edit to the object.
            animated = await invoke("sim__prim__set_transform", error=True, **target,
                                    path="/World/Animated", position=[0, 0, 0])
            assert animated["code"] == "ANIMATED_TRANSFORM"
            owned = await invoke("sim__prim__set_attribute", error=True, **target,
                                 path="/World/Table", attribute="xformOpOrder", value=[])
            assert owned["code"] == "ENGINE_OWNED_ATTRIBUTE"
            proxy = await invoke("sim__prim__set_transform", error=True, **target,
                                 path="/World/Inst/Body", position=[0, 0, 0])
            assert proxy["code"] == "INSTANCE_PROXY", proxy
            source_collider = await invoke("sim__physics__apply", error=True, **target,
                                           path="/World/Table", collider=False)
            assert source_collider["code"] == "READ_ONLY_SOURCE", source_collider

            # sim__prim__create adds a Plane, and writes typed attributes on any prim.
            plate = await invoke("sim__prim__create", **target, path="/World/Plate", kind="Plane",
                                 size=2, collision=True, position=[0, 0, 0.001])
            assert plate["type"] == "Plane" and plate["physics"]["collision"] is True
            visibility = await invoke("sim__prim__set_attribute", **target, path="/World/Decor",
                                      attribute="visibility", value="invisible")
            assert visibility["before"] == "inherited" and visibility["after"] == "invisible"
            assert visibility["created_attribute"] is False
            counter = await invoke("sim__prim__set_attribute", **target, path="/World/Decor",
                                   attribute="user:count", value=3, type="int")
            assert counter["after"] == 3 and counter["created_attribute"] is True
            report["steps"]["prim_create_plane"] = dict(type=plate["type"],
                                                        collision=plate["physics"]["collision"])
            report["steps"]["set_attribute"] = dict(visibility=visibility["after"],
                                                    count=counter["after"])

            # Give the visual-only object a collider and a rigid body, then let it fall onto the table.
            applied = await invoke("sim__physics__apply", **target, path="/World/Object",
                                   collider=True, approximation="convexHull", rigid_body=True,
                                   kinematic=False)
            assert applied["before"]["physics"]["collision"] is False
            assert applied["after"]["physics"]["collision"] is True
            assert applied["after"]["physics"]["rigid_body"] is True
            assert applied["after"]["physics"]["kinematic"] is False
            assert applied["after"]["physics"]["mass"] is None
            posed = await invoke("sim__prim__set_transform", **target, path="/World/Object",
                                 position=[0.25, 0, 1.2], frame="world")
            assert posed["world_position"] == [0.25, 0, 1.2]

            # A body edit while the timeline runs is refused.
            await invoke("sim__timeline", **target, action="play")
            await asyncio.sleep(0.4)
            running = await invoke("sim__prim__set_transform", error=True, **target,
                                   path="/World/Table", position=[0, 0, 0.5], frame="world")
            assert running["code"] == "TIMELINE_ACTIVE", running
            await asyncio.sleep(2.6)
            await invoke("sim__timeline", **target, action="pause")
            resting = await invoke("sim__body__state", **target, path="/World/Object",
                                   include_dynamics=True)
            z = resting["world_position"][2]
            assert 0.55 <= z <= 0.72, resting  # table top 0.525 + object half-height 0.1
            assert resting["dynamics"]["mass"] > 0
            report["steps"]["falls_onto_table"] = dict(z=z, mass=resting["dynamics"]["mass"],
                                                       expected_rest_z=0.625)
            await invoke("sim__timeline", **target, action="stop")

            # Mass authored on pack-created cubes is what the runtime reports.
            await invoke("sim__prim__create", **target, path="/World/MassA", kind="Cube",
                         size=0.2, rigid=True, mass=2.5, position=[-0.6, 0, 1.5])
            await invoke("sim__prim__create", **target, path="/World/MassB", kind="Cube",
                         size=0.2, rigid=True, mass=5.0, position=[-0.9, 0, 1.5])
            await invoke("sim__timeline", **target, action="play")
            await asyncio.sleep(3.0)
            await invoke("sim__timeline", **target, action="pause")
            first = await invoke("sim__body__state", **target, path="/World/MassA",
                                 include_dynamics=True)
            second = await invoke("sim__body__state", **target, path="/World/MassB",
                                  include_dynamics=True)
            assert abs(first["dynamics"]["mass"] - 2.5) < 1e-4, first
            assert abs(second["dynamics"]["mass"] - 5.0) < 1e-4, second
            assert abs(first["dynamics"]["mass_kg"] - 2.5) < 1e-4
            assert abs(second["dynamics"]["mass_kg"] - 5.0) < 1e-4
            report["steps"]["runtime_mass"] = dict(
                authored=[2.5, 5.0],
                runtime=[first["dynamics"]["mass"], second["dynamics"]["mass"]],
                runtime_kg=[first["dynamics"]["mass_kg"], second["dynamics"]["mass_kg"]],
            )
            await invoke("sim__timeline", **target, action="stop")

            # A body under a body is refused, on a prim the pack created this time.
            await invoke("sim__prim__create", **target, path="/World/Object/Core", kind="Xform")
            nested = await invoke("sim__physics__apply", error=True, **target,
                                  path="/World/Object/Core", rigid_body=True)
            assert nested["code"] == "NESTED_BODY", nested

            # Revert the object and the table; the composed values come back.
            reverted = await invoke("sim__prim__revert", **target, path="/World/Object",
                                    subtree=True)
            assert reverted["removed"] and reverted["not_reverted"] == [], reverted
            object_after = await invoke("sim__prim__get", **target, path="/World/Object")
            assert object_after["physics"]["collision"] is False
            assert object_after["physics"]["rigid_body"] is False
            assert object_after["world_matrix"] == object_authored["world_matrix"]

            table_reverted = await invoke("sim__prim__revert", **target, path="/World/Table")
            decor_reverted = await invoke("sim__prim__revert", **target, path="/World/Decor")
            table_after = await invoke("sim__prim__get", **target, path="/World/Table")
            decor_after = await invoke("sim__prim__get", **target, path="/World/Decor")
            assert table_after["world_matrix"] == table_before["world_matrix"]
            assert decor_after["world_position"] == [1, 0, 0]
            assert table_reverted["removed"] and decor_reverted["removed"]
            report["steps"]["revert"] = dict(
                object_removed=reverted["removed"], table_removed=table_reverted["removed"],
                decor_removed=decor_reverted["removed"],
                object_collision=object_after["physics"]["collision"],
                table_position=table_after["world_position"],
                decor_position=decor_after["world_position"],
            )
            await invoke("sim__stage__info")

            # Source files never change, byte for byte.
            assert sha256(scene_file) == hashes["scene"]
            assert sha256(block_file) == hashes["block"]
            report["source_sha256_after"] = {"scene": sha256(scene_file),
                                             "block": sha256(block_file)}

            # A public robot: the whole robot moves, one of its links is refused.
            started = await invoke("sim__quick_start", robot=robot, discard_changes=True)
            robot_target = {k: started[k] for k in ("session_id", "stage_id")}
            robot_prim = started["robot"]["prim"]
            root = started["robot"]["articulation_root"]
            tree = await invoke("sim__stage__tree", **robot_target, root=robot_prim, depth=2, limit=50)
            children = [node["path"] for node in tree["nodes"]
                        if node["path"] != root and node["path"] != robot_prim
                        and node["path"].startswith(robot_prim + "/")
                        and node["path"].count("/") == robot_prim.count("/") + 1]
            assert children, tree
            refused_link = await invoke("sim__prim__set_transform", error=True, **robot_target,
                                        path=children[0], position=[0, 0, 0])
            assert refused_link["code"] == "ARTICULATION_LINK", (children, refused_link)
            assert refused_link["articulation_root"] == root
            assert refused_link["retry_with"]["path"] == robot_prim
            robot_moved = await invoke("sim__prim__set_transform", **robot_target,
                                       path=robot_prim, position=[0.4, 0, 0], frame="world")
            assert robot_moved["world_position"] == [0.4, 0, 0]
            robot_reverted = await invoke("sim__prim__revert", **robot_target, path=robot_prim)
            assert robot_reverted["removed"]
            report["steps"]["robot"] = dict(
                prim=robot_prim, articulation_root=root, link=children[0],
                refusal=refused_link["message"][:300],
                moved_to=robot_moved["world_position"],
                restored=robot_reverted["removed"],
            )
            await invoke("sim__timeline", **robot_target, action="stop")
            print(json.dumps(dict(passed=True, calls=len(records), **report)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--directory", required=True, help="folder under your home or the output root")
    parser.add_argument("--robot", default="franka")
    args = parser.parse_args()
    asyncio.run(validate(args.port, args.output, args.directory, args.robot))
