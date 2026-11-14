"""Drive the host MCP server in-process and record ok/error and runtime per tool call.

Goes through the same FastMCP dispatch an MCP client uses (argument validation, host
tools, bridge forwarding), so it exercises the full registered surface.

    uv run --project mcp python scripts/smoke_mcp_surface.py offline
    TWINSMITH_PORT=8780 uv run --project mcp python scripts/smoke_mcp_surface.py live PLAN.json

``offline`` calls the host/Isaac Lab tools, then two read-only bridge tools with the bridge pinned to a
closed port, so it never reaches a running simulator and shows the errors a client gets without one.
``live`` runs a JSON plan
(list of {"tool": name, "args": {...}}) against the bridge selected by TWINSMITH_PORT; it changes
the stage, so use a disposable simulator. Prints a markdown table.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time

os.environ.setdefault("TWINSMITH_TOOLSETS", "all,+computer")

# Nothing listens on port 1 without root; a pin takes precedence over bridge discovery.
OFFLINE_PORT = "1"
OFFLINE_PLAN = [
    ("refresh_tools", {}), ("screen_info", {}), ("list_windows", {}),
    ("screen_recording_status", {}), ("list_simulators", {}),
    ("isaaclab__install_info", {}), ("isaaclab__envs_list", {"filter": "Reach"}),
    ("isaaclab__runs_list", {}), ("isaaclab__checkpoints_list", {}),
    ("assets__roots", {}), ("assets__scan", {"root_id": "does-not-exist"}),
    ("assets__search", {"query": "ur5e"}),
    ("sim__stage__info", {}), ("workcell__get_recording_status", {}),
]


def _flatten(result) -> tuple[bool, str]:
    if isinstance(result, tuple):
        result = result[0]
    content = getattr(result, "content", result)
    text = " ".join(getattr(c, "text", "") for c in content)
    if getattr(result, "isError", False) or text.startswith("Error"):
        return False, text
    try:
        data = json.loads(text)
    except ValueError:
        return True, text
    bad = isinstance(data, dict) and (data.get("ok") is False or data.get("status") == "error" or "error" in data)
    return not bad, text


async def run(plan) -> list[dict]:
    from twinsmith.server import build_server
    server = build_server(dict(os.environ))
    rows = []
    for name, args in plan:
        start = time.monotonic()
        try:
            ok, text = _flatten(await server.call_tool(name, args))
        except Exception as exc:  # tool errors and validation errors both land here
            ok, text = False, f"{type(exc).__name__}: {exc}"
        row = {"tool": name, "args": args, "ok": ok, "seconds": round(time.monotonic() - start, 2),
               "text": text[:400].replace("\n", " ")}
        rows.append(row)
        print(f"| {name} | {'ok' if ok else 'ERROR'} | {row['seconds']}s | {row['text'][:160]} |", flush=True)
    return rows


class Live:
    """Stateful live caller: injects the current session_id/stage_id into tools that take them.

    Use from a phase script::

        live = Live(); await live.start()
        await live.call("sim__prim__create", {"path": "/World/Box", "kind": "Cube"})
    """

    def __init__(self) -> None:
        self.rows: list[dict] = []
        self.ids: dict = {}

    async def start(self):
        from twinsmith.server import build_server
        self.server = build_server(dict(os.environ))
        await self.server.call_tool("refresh_tools", {})
        await self.refresh_ids()

    async def refresh_ids(self):
        res = await self.server.call_tool("sim__stage__info", {})
        try:
            data = json.loads(_flatten(res)[1])
            self.ids = {"session_id": data["session_id"], "stage_id": data["stage_id"]}
        except Exception:
            self.ids = {}

    async def call(self, name: str, args: dict | None = None, ids: bool = True, limit: int = 300):
        args = dict(args or {})
        if ids and name.startswith(("sim__", "assets__")) and self.ids:
            tools = {t.name: t for t in await self.server.list_tools()}
            props = (tools[name].inputSchema or {}).get("properties", {}) if name in tools else {}
            for key, value in self.ids.items():
                if key in props:
                    args.setdefault(key, value)
        start = time.monotonic()
        try:
            ok, text = _flatten(await self.server.call_tool(name, args))
        except Exception as exc:
            ok, text = False, f"{type(exc).__name__}: {exc}"
        secs = round(time.monotonic() - start, 2)
        shown = {k: v for k, v in args.items() if k not in ("session_id", "stage_id")}
        self.rows.append({"tool": name, "args": shown, "ok": ok, "seconds": secs, "text": text})
        print(f"### {name} {json.dumps(shown)[:150]} -> {'ok' if ok else 'ERROR'} {secs}s\n{text[:limit]}\n", flush=True)
        return ok, text

    def save(self, path: str):
        old = json.load(open(path)) if os.path.exists(path) else []
        json.dump(old + self.rows, open(path, "w"), indent=1)


def main(argv: list[str]) -> int:
    mode = argv[1] if len(argv) > 1 else "offline"
    if mode in ("-h", "--help"):
        print(__doc__)
        return 0
    if mode not in ("offline", "live") or (mode == "live" and len(argv) < 3):
        print("usage: smoke_mcp_surface.py offline | live PLAN.json [OUT.json]", file=sys.stderr)
        return 2
    if mode == "offline":
        os.environ["TWINSMITH_PORT"] = OFFLINE_PORT
    plan = OFFLINE_PLAN if mode == "offline" else [(p["tool"], p.get("args", {})) for p in json.load(open(argv[2]))]
    rows = asyncio.run(run(plan))
    if len(argv) > 3:
        json.dump(rows, open(argv[3], "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
