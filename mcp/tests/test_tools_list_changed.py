"""tools/listChanged: the host advertises it and notifies after discovery.

Clients such as Claude Code only refresh their tool list mid-session when the
server advertised ``tools.listChanged`` at initialize AND sends
``notifications/tools/list_changed``. These tests drive the real host server
(``twinsmith.server.mcp``) through an in-memory MCP session against a fake bridge.
"""

import asyncio

import pytest

pytest.importorskip("mcp")

import mcp.types as types
from mcp.server.session import ServerSession
from mcp.shared.memory import create_connected_server_and_client_session

import twinsmith.server as gw
from twinsmith_bridge.schema import field_specs


def _meta(description, parameters=None):
    parameters = parameters or {}
    return {"description": description, "parameters": parameters,
            "fields": field_specs(parameters), "namespace": "demo", "owner_id": "A"}


class FakeBridge:
    """Stands in for BridgeClient: serves `status` and a mutable registry."""

    def __init__(self, tools):
        self.tools = tools
        self.port = 8766

    def call(self, tool, params=None, **kw):
        if tool == "list_tools":
            return {"tools": self.tools}
        if tool == "status":
            return {"pid": 1234, "port": self.port}
        raise AssertionError(f"unexpected bridge call {tool}")

    def close(self):
        pass


@pytest.fixture()
def host(monkeypatch):
    """The real host server with a clean dynamic registry and a fake bridge."""
    bridge = FakeBridge({})
    monkeypatch.setattr(gw, "_NAME_MAP", {})
    monkeypatch.setattr(gw, "_REGISTERED_META", {})
    monkeypatch.setattr(gw, "_client", None)
    monkeypatch.setattr(gw, "get_client", lambda: bridge)
    monkeypatch.setattr(gw, "BridgeClient", lambda port=None, **kw: bridge)
    # The tool manager is process-global: restore it so other tests see 21 tools.
    monkeypatch.setattr(gw.mcp._tool_manager, "_tools",
                        dict(gw.mcp._tool_manager._tools))
    # The fake bridge serves a project's own namespace, listed only with sim on (off by default).
    monkeypatch.setattr(gw._DEFAULT_STATE, "toolsets", gw._DEFAULT_STATE.toolsets | {"sim"})
    return bridge


def _session(notifications):
    async def handler(message):
        if isinstance(message, types.ServerNotification) and isinstance(
                message.root, types.ToolListChangedNotification):
            notifications.append(message)
    return create_connected_server_and_client_session(
        gw.mcp._mcp_server, message_handler=handler)


def _run(coro):
    return asyncio.run(coro)


def test_initialize_advertises_tools_list_changed(host):
    async def go():
        async with _session([]) as client:
            return (await client.initialize()).capabilities
    caps = _run(go())
    assert caps.tools is not None and caps.tools.listChanged is True


def test_explicit_notification_options_are_not_overridden(host):
    from mcp.server.lowlevel.server import NotificationOptions
    init = gw.mcp._mcp_server.create_initialization_options(
        notification_options=NotificationOptions(tools_changed=False))
    assert init.capabilities.tools.listChanged is False
    assert gw.mcp._mcp_server.create_initialization_options(
    ).capabilities.tools.listChanged is True


def test_ctx_is_injected_not_advertised(host):
    async def go():
        async with _session([]) as client:
            return {t.name: t for t in (await client.list_tools()).tools}
    tools = _run(go())
    assert "ctx" not in tools["refresh_tools"].inputSchema.get("properties", {})
    assert list(tools["use_bridge"].inputSchema["properties"]) == ["port"]
    assert "$defs" not in tools["refresh_tools"].inputSchema
    assert tools["use_bridge"].inputSchema["required"] == ["port"]


def test_refresh_notifies_once_on_change_and_never_when_unchanged(host):
    notifications = []

    async def go():
        async with _session(notifications) as client:
            await client.initialize()
            host.tools = {"demo.ping": _meta("ping")}         # appears after startup
            first = await client.call_tool("refresh_tools", {})
            after_first = len(notifications)
            second = await client.call_tool("refresh_tools", {})
            after_second = len(notifications)
            host.tools = {"demo.ping": _meta("ping v2")}      # updated
            await client.call_tool("refresh_tools", {})
            after_update = len(notifications)
            host.tools = {}                                   # removed
            await client.call_tool("refresh_tools", {})
            after_removal = len(notifications)
            names = {t.name for t in (await client.list_tools()).tools}
            return first, second, after_first, after_second, after_update, after_removal, names
    first, second, a1, a2, a3, a4, names = _run(go())
    assert "demo.ping" in first.content[0].text
    assert "registry unchanged" in second.content[0].text
    assert (a1, a2, a3, a4) == (1, 1, 2, 3)
    assert "demo__ping" not in names


def test_failed_discovery_sends_nothing(host, monkeypatch):
    def boom():
        raise OSError("bridge unavailable")
    monkeypatch.setattr(gw, "get_client", boom)
    notifications = []

    async def go():
        async with _session(notifications) as client:
            await client.initialize()
            return await client.call_tool("refresh_tools", {})
    result = _run(go())
    assert result.content[0].text.startswith("Error:")
    assert notifications == []


def test_use_bridge_notifies_once(host):
    notifications = []

    async def go():
        async with _session(notifications) as client:
            await client.initialize()
            host.tools = {"demo.ping": _meta("ping")}
            result = await client.call_tool("use_bridge", {"port": 8766})
            names = {t.name for t in (await client.list_tools()).tools}
            return result, names
    result, names = _run(go())
    assert "switched to bridge on port 8766" in result.content[0].text
    assert "demo__ping" in names
    assert len(notifications) == 1


def test_use_bridge_notifies_when_dropped_tools_are_not_replaced(host):
    """Switching to a bridge that serves nothing still removes the old tools."""
    notifications = []

    async def go():
        async with _session(notifications) as client:
            await client.initialize()
            host.tools = {"demo.ping": _meta("ping")}
            await client.call_tool("refresh_tools", {})
            before = len(notifications)
            host.tools = {}
            await client.call_tool("use_bridge", {"port": 8766})
            names = {t.name for t in (await client.list_tools()).tools}
            return before, names
    before, names = _run(go())
    assert before == 1 and len(notifications) == 2
    assert "demo__ping" not in names


def test_notification_failure_does_not_fail_the_tool(host, monkeypatch, caplog):
    async def broken(self):
        raise RuntimeError("stream closed")
    monkeypatch.setattr(ServerSession, "send_tool_list_changed", broken)
    async def go():
        async with _session([]) as client:
            await client.initialize()
            host.tools = {"demo.ping": _meta("ping")}
            refreshed = await client.call_tool("refresh_tools", {})
            switched = await client.call_tool("use_bridge", {"port": 8766})
            names = {t.name for t in (await client.list_tools()).tools}
            return refreshed, switched, names
    with caplog.at_level("WARNING"):
        refreshed, switched, names = _run(go())
    assert not refreshed.isError and not switched.isError
    assert "demo__ping" in names
    assert "Could not send tools/list_changed" in caplog.text


def test_direct_calls_without_a_context_still_work(host):
    host.tools = {"demo.ping": _meta("ping")}
    assert "demo.ping" in _run(gw.refresh_tools())
    assert "switched to bridge" in _run(gw.use_bridge(8766))
