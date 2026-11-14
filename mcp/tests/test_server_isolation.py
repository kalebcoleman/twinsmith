"""Servers built by ``build_server`` do not share bridge state.

Each server owns its bridge connection and its discovery bookkeeping, so a second server in the
same process registers the bridge's tools even when another already discovered them, and
refreshing, switching or shutting down one leaves the others alone. The default ``server.mcp``
keeps using the module-level state that older code and tests read.
"""

import asyncio

import pytest

pytest.importorskip("mcp")

import mcp.types as types
from mcp.shared.memory import create_connected_server_and_client_session

import twinsmith.screen as screen
import twinsmith.server as gw
from twinsmith import host_catalog, local_tools as lt
from twinsmith_bridge.schema import field_specs


@pytest.fixture(autouse=True)
def single_purpose_tool_list(member_listing):
    """These tests are about the single-purpose tools themselves, so they list them as before the groups."""
    pass


def _meta(description, namespace="demo"):
    return {"description": description, "parameters": {}, "fields": field_specs({}),
            "namespace": namespace, "owner_id": "A"}


class FakeBridge:
    """A bridge connection: serves `registries[port]` and records being closed."""

    def __init__(self, registries, port=8766):
        self.registries = registries
        self.port = port
        self.closed = False

    def call(self, tool, params=None, **kw):
        if tool == "list_tools":
            return {"tools": self.registries[self.port]}
        if tool == "status":
            return {"pid": 1, "port": self.port}
        if tool in self.registries[self.port]:
            return {"tool": tool, "port": self.port}
        raise AssertionError(f"unexpected bridge call {tool}")

    def close(self):
        self.closed = True


@pytest.fixture()
def world(monkeypatch, tmp_path):
    """Every BridgeClient the servers create, and a registry per port, all fake."""
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path / "shots")
    monkeypatch.setattr(screen, "VIDEO_DIR", tmp_path / "videos")
    # The default server's state must be untouched by anything built here.
    monkeypatch.setattr(gw, "_NAME_MAP", {})
    monkeypatch.setattr(gw, "_REGISTERED_META", {})
    monkeypatch.setattr(gw, "_client", None)
    registries = {8766: {"demo.ping": _meta("ping")},
                  9001: {"alt.tool": _meta("alt tool", namespace="alt")}}
    made = []

    def factory(port=None, **kw):
        client = FakeBridge(registries, port or 8766)
        made.append(client)
        return client

    monkeypatch.setattr(gw, "BridgeClient", factory)
    return type("World", (), {"registries": registries, "clients": made})


# The catalog's offline proxies are listed by every built server; these tests are about the
# bridge tools two servers discover (tests/test_host_catalog_gateway.py covers the proxies).
OFFLINE = {gw._mcp_name(canonical) for canonical in host_catalog.catalog_tool_names()}


def build(groups="default,+sim"):
    return gw.build_server({"TWINSMITH_TOOLSETS": groups})


def bridge_names(server):
    return {t.name for t in asyncio.run(server.list_tools())} - set(gw.build_server({"TWINSMITH_TOOLSETS": "all,+computer"})._tool_manager._tools) - OFFLINE


def session(server, notifications=None):
    async def handler(message):
        if notifications is not None and isinstance(message, types.ServerNotification) and \
                isinstance(message.root, types.ToolListChangedNotification):
            notifications.append(message)
    return create_connected_server_and_client_session(server._mcp_server, message_handler=handler)


async def listed(client):
    return {t.name for t in (await client.list_tools()).tools} - set(gw.build_server({"TWINSMITH_TOOLSETS": "all,+computer"})._tool_manager._tools) - OFFLINE


def test_each_server_has_its_own_state_and_the_default_uses_the_module_state():
    a, b = build(), build()
    assert gw._state_for(a) is not gw._state_for(b)
    assert gw._state_for(a) is not gw._DEFAULT_STATE
    assert gw._state_for(gw.mcp) is gw._DEFAULT_STATE


def test_a_second_server_registers_tools_the_first_already_discovered(world):
    a, b = build(), build()

    async def go():
        async with session(a) as ca:
            await ca.initialize()
            first = await listed(ca)
            async with session(b) as cb:
                await cb.initialize()
                return first, await listed(cb)

    first, second = asyncio.run(go())
    assert first == second == {"demo__ping"}
    assert gw.registered_tools(a) == gw.registered_tools(b) == ["demo.ping"]
    assert gw.registered_tools() == []              # the default instance saw none of it


def test_refresh_on_the_second_server_does_not_skip_tools(world):
    a, b = build(), build()
    world.registries[8766] = {}

    async def go():
        async with session(a) as ca, session(b) as cb:
            await ca.initialize()
            await cb.initialize()
            assert await listed(ca) == await listed(cb) == set()
            world.registries[8766] = {"demo.ping": _meta("ping")}
            ra = await ca.call_tool("refresh_tools", {})
            rb = await cb.call_tool("refresh_tools", {})
            return ra, rb, await listed(ca), await listed(cb)

    ra, rb, names_a, names_b = asyncio.run(go())
    assert "demo.ping" in ra.content[0].text and "demo.ping" in rb.content[0].text
    assert "unchanged" not in rb.content[0].text
    assert names_a == names_b == {"demo__ping"}


def test_use_bridge_on_one_server_leaves_the_other_alone(world):
    a, b = build(), build()
    notes_a, notes_b = [], []

    async def go():
        async with session(a, notes_a) as ca, session(b, notes_b) as cb:
            await ca.initialize()
            await cb.initialize()
            old_a, client_b = gw._state_for(a).client, gw._state_for(b).client
            switched = await ca.call_tool("use_bridge", {"port": 9001})
            result_a = await ca.call_tool("alt__tool", {})
            result_b = await cb.call_tool("demo__ping", {})
            await ca.call_tool("refresh_tools", {})  # unchanged: no second notification
            return (switched, await listed(ca), await listed(cb), old_a, client_b,
                    gw._state_for(a).client, gw._state_for(b).client, client_b.closed,
                    result_a, result_b, (len(notes_a), len(notes_b)))

    (switched, names_a, names_b, old_a, client_b, new_a, b_after, b_closed,
     result_a, result_b, notes) = asyncio.run(go())
    assert not switched.isError
    assert names_a == {"alt__tool"} and names_b == {"demo__ping"}
    assert old_a.closed and new_a.port == 9001           # A replaced and closed its own client
    assert b_after is client_b and client_b.port == 8766 and not b_closed
    assert '"port": 9001' in result_a.content[0].text
    assert '"port": 8766' in result_b.content[0].text
    assert notes == (1, 0)
    assert gw.registered_tools(a) == ["alt.tool"] and gw.registered_tools(b) == ["demo.ping"]
    assert gw.registered_tools() == []                    # the default instance is unchanged


def test_shutting_down_one_server_does_not_close_the_others_client(world):
    a, b = build(), build()

    async def go():
        async with session(b) as cb:
            await cb.initialize()
            client_b = gw._state_for(b).client
            async with session(a) as ca:
                await ca.initialize()
                client_a = gw._state_for(a).client
                assert client_a is not client_b
            closed_when_a_ended = (client_a.closed, client_b.closed)
            assert await listed(cb) == {"demo__ping"}       # B still serves its tools
            result = await cb.call_tool("demo__ping", {})
            assert '"port": 8766' in result.content[0].text
            return closed_when_a_ended, client_b

    (a_closed, b_closed), client_b = asyncio.run(go())
    assert a_closed is True and b_closed is False
    assert client_b.closed is True                           # closed by its own shutdown


def test_list_changed_is_advertised_and_notified_per_server(world):
    a, b = build(), build()
    world.registries[8766] = {}
    notes_a, notes_b = [], []

    async def go():
        async with session(a, notes_a) as ca, session(b, notes_b) as cb:
            caps = [(await c.initialize()).capabilities for c in (ca, cb)]
            world.registries[8766] = {"demo.ping": _meta("ping")}
            await ca.call_tool("refresh_tools", {})
            after_a = (len(notes_a), len(notes_b))
            await ca.call_tool("refresh_tools", {})            # unchanged: no notification
            await cb.call_tool("refresh_tools", {})
            return caps, after_a, (len(notes_a), len(notes_b))

    caps, after_a, after_both = asyncio.run(go())
    assert all(c.tools is not None and c.tools.listChanged is True for c in caps)
    assert after_a == (1, 0)
    assert after_both == (1, 1)


def test_the_screen_port_follows_the_servers_own_client(world, monkeypatch):
    monkeypatch.delenv("TWINSMITH_PORT", raising=False)
    monkeypatch.delenv("TWINSMITH_PORT", raising=False)
    a, b = build(), build()
    sa, sb = gw._state_for(a), gw._state_for(b)
    assert gw._screen_port(sa) is None and gw._screen_port(sb) is None
    sa.client = FakeBridge({}, port=9001)
    assert gw._screen_port(sa) == 9001 and gw._screen_port(sb) is None
