"""Real stdio MCP client for manual/agent use. No bridge or private Kit APIs.

Usage: python scripts/scene_mcp_client.py --port 18861 sim__stage__info '{}'
       python scripts/scene_mcp_client.py --port 18861 list_tools   # every tool, with its arguments'
Uses this checkout's host server. TWINSMITH_ASSET_CONFIG is user launch configuration.
"""

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "mcp") not in sys.path:
    sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith import tool_groups  # noqa: E402  (pure data; needs no MCP SDK)


def parameters(port, toolsets=None):
    """Server parameters for one bridge. ``toolsets`` (an TWINSMITH_TOOLSETS value such as
    "all" or "default,+sim,+assets") overrides the environment; without it an exported
    TWINSMITH_TOOLSETS opts in, else a scene-only client is started."""
    return StdioServerParameters(
        command=sys.executable,
        args=["-m", "twinsmith.server"],
        env={
            # Defaults for a scene-only client; an exported value opts in (e.g. default,+computer).
            "TWINSMITH_TOOLSETS": "default,+sim,+assets",
            **os.environ,
            **({"TWINSMITH_TOOLSETS": toolsets} if toolsets else {}),
            "PYTHONPATH": str(ROOT / "mcp"),
            "TWINSMITH_PORT": str(port),
        },
    )


def listed(name, args):
    """The listed tool that runs a single-purpose tool: ``sim__prim__create`` is sent as
    ``sim__prim`` with ``action="create"``. A name that is not part of a group is sent as it is."""
    found = tool_groups.members().get(name)
    if not found:
        return name, args
    group, action = found
    args = {("sub_action" if key == "action" else key): value for key, value in args.items()}
    return group, {"action": action, **args}


async def call(client, name, args):
    """Call a tool by the name a client is shown. The transcript keeps the name the caller used
    (``tool``) and records what was sent (``sent``, ``sent_arguments``)."""
    sent, sent_args = listed(name, args)
    result = await client.call_tool(sent, sent_args)
    data = "\n".join(x.text for x in result.content if x.type == "text")
    try:
        data = json.loads(data)
    except ValueError:
        pass
    record = dict(tool=name, arguments=args, isError=result.isError, result=data)
    if sent != name:
        record.update(sent=sent, sent_arguments=sent_args)
    return record


async def main(port, name, args):
    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as c:
            await c.initialize()
            if name == "list_tools":
                return (await c.list_tools()).model_dump(mode="json")
            reply = await call(c, name, args)
            if reply["isError"] and str(reply["result"]).startswith("Unknown tool"):
                reply["hint"] = "Run this script with the tool name list_tools to see every tool and its arguments."
            return reply


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, required=True)
    p.add_argument("tool")
    p.add_argument("arguments", nargs="?", default="{}")
    a = p.parse_args()
    print(
        json.dumps(asyncio.run(main(a.port, a.tool, json.loads(a.arguments))), indent=2)
    )
