"""Rediscovery: a server's bridge tools follow what the selected bridge serves."""

import asyncio
import types

import pytest
from mcp.server.fastmcp import FastMCP

import twinsmith.server as gw
from twinsmith_bridge.schema import field_specs


def _meta(description, parameters, namespace, owner_id):
    """A tool as a bridge lists it, its argument list made by the bridge's own code."""
    return {"description": description, "parameters": parameters, "fields": field_specs(parameters),
            "namespace": namespace, "owner_id": owner_id}


@pytest.fixture()
def fresh_gateway(monkeypatch):
    monkeypatch.setattr(gw, "_NAME_MAP", {})
    monkeypatch.setattr(gw, "_REGISTERED_META", {})
    monkeypatch.setattr(gw._DEFAULT_STATE, "toolsets", gw.toolsets.PROFILES["all"])
    return FastMCP("test")


def _fake_registry(monkeypatch, tools):
    bridge = types.SimpleNamespace(call=lambda tool, params=None, **kw: {"tools": tools})
    monkeypatch.setattr(gw, "get_client", lambda: bridge)


def _served(server, name):
    return server._tool_manager.get_tool(name)


def test_a_tool_is_added_once_and_registered_again_when_it_changes(fresh_gateway, monkeypatch):
    listing = {"demo.ping": _meta("first", {"message": {"type": "string"}}, "demo", "A")}
    _fake_registry(monkeypatch, listing)
    assert gw.discover_and_register_tools(fresh_gateway) == ["demo.ping"]
    assert gw.discover_and_register_tools(fresh_gateway) == [], "nothing changed, nothing reported"
    listing["demo.ping"] = _meta("second", {"message": {"type": "string"}, "count": {"type": "integer"}}, "demo", "A")
    assert gw.discover_and_register_tools(fresh_gateway) == ["demo.ping (updated)"]
    tool = _served(fresh_gateway, "demo__ping")
    assert tool.description == "second" and "count" in tool.parameters["properties"]


def test_required_optional_and_defaulted_arguments_reach_the_schema(fresh_gateway, monkeypatch):
    arguments = {"note": {"type": "string"}, "code": {"type": "string", "required": True},
                 "steps": {"type": "integer", "default": 4}}
    _fake_registry(monkeypatch, {"demo.run": _meta("d", arguments, "demo", "A")})
    gw.discover_and_register_tools(fresh_gateway)
    schema = _served(fresh_gateway, "demo__run").parameters
    assert schema["required"] == ["code"]
    assert schema["properties"]["steps"]["default"] == 4 and "default" not in schema["properties"]["note"]  # a null default is not listed


def test_a_listing_without_argument_lists_is_refused(fresh_gateway, monkeypatch):
    _fake_registry(monkeypatch, {"demo.ping": {"description": "d", "parameters": {}, "namespace": "demo", "owner_id": "A"}})
    with pytest.raises(RuntimeError, match="field specs"):
        gw.discover_and_register_tools(fresh_gateway)


def test_the_namespace_and_builtin_filters_apply_to_rediscovery(fresh_gateway, monkeypatch):
    gw._state_for(fresh_gateway).toolsets = frozenset({"core", "sim"})
    _fake_registry(monkeypatch, {"pm.read_state": _meta("d", {}, "pm", "P"), "demo.ping": _meta("d", {}, "demo", "A"),
                                 "run_python": _meta("d", {}, None, "twinsmith.bridge")})
    assert gw.discover_and_register_tools(fresh_gateway) == ["demo.ping", "pm.read_state", "run_python"]
    assert set(fresh_gateway._tool_manager._tools) == {"pm__read_state", "demo__ping", "run_python"}
    gw._state_for(fresh_gateway).toolsets = frozenset({"core"})
    gw.discover_and_register_tools(fresh_gateway)
    assert not {"pm__read_state", "demo__ping", "run_python"} & set(fresh_gateway._tool_manager._tools)


def test_refresh_removes_departed_owner_without_touching_other_tools(fresh_gateway, monkeypatch):
    server = fresh_gateway
    tools = {
        "arm.move": _meta("arm", {}, "arm", "arm-owner"),
        "sim.stage.info": _meta("sim", {}, "sim", "sim-owner"),
    }
    _fake_registry(monkeypatch, tools)
    gw.discover_and_register_tools(server)
    del tools["sim.stage.info"]

    assert "sim.stage.info (removed)" in gw.discover_and_register_tools(server)
    assert _served(server, "arm__move") is not None
    assert _served(server, "sim__stage__info") is None
    assert "sim__stage__info" not in gw._NAME_MAP
    assert "sim__stage__info" not in gw._REGISTERED_META


def test_refresh_removes_tools_that_no_longer_match_view_filter(fresh_gateway, monkeypatch):
    server = fresh_gateway
    tools = {"arm.move": _meta("arm", {}, "arm", "arm-owner"),
             "sim.stage.info": _meta("sim", {}, "sim", "sim-owner")}
    _fake_registry(monkeypatch, tools)
    gw.discover_and_register_tools(server)
    gw._state_for(server).toolsets = frozenset({"core"})

    assert {"arm.move (removed)", "sim.stage.info (removed)"} <= set(gw.discover_and_register_tools(server))
    assert _served(server, "arm__move") is None
    assert _served(server, "sim__stage__info") is None


def test_failed_refresh_keeps_existing_registry(fresh_gateway, monkeypatch):
    server = fresh_gateway
    _fake_registry(monkeypatch, {"arm.move": _meta("arm", {}, "arm", "arm-owner")})
    gw.discover_and_register_tools(server)

    class Unreachable:
        def call(self, *args, **kwargs):
            raise OSError("bridge unavailable")

    monkeypatch.setattr(gw, "get_client", lambda: Unreachable())
    with pytest.raises(OSError, match="unavailable"):
        gw.discover_and_register_tools(server)
    assert _served(server, "arm__move") is not None


@pytest.mark.parametrize("response", [
    {},
    {"tools": None},
    {"tools": {"sim.stage.info": {"namespace": "sim"}}},
    {"tools": {"sim.stage.info": None}},
    {"tools": {
        "sim.stage.info": _meta("good", {}, "sim", "sim-owner"),
        "sim.bad": {"namespace": "sim", "fields": [{"name": "x"}]},
    }},
])
def test_malformed_refresh_keeps_existing_registry(fresh_gateway, monkeypatch, response):
    server = fresh_gateway
    _fake_registry(monkeypatch, {"arm.move": _meta("arm", {}, "arm", "arm-owner")})
    gw.discover_and_register_tools(server)

    class Invalid:
        def call(self, *args, **kwargs):
            return response

    monkeypatch.setattr(gw, "get_client", lambda: Invalid())
    with pytest.raises(RuntimeError):
        gw.discover_and_register_tools(server)
    assert _served(server, "arm__move") is not None
    assert _served(server, "sim__good") is None
    assert gw._NAME_MAP == {"arm__move": "arm.move"}


def test_bridge_tool_cannot_replace_a_static_host_tool(fresh_gateway, monkeypatch):
    server = fresh_gateway
    server.add_tool(lambda: "host", name="refresh_tools", description="host tool")
    _fake_registry(monkeypatch, {"refresh_tools": _meta("bridge tool", {}, None, "B")})
    with pytest.raises(RuntimeError, match="collid"):
        gw.discover_and_register_tools(server)
    assert _served(server, "refresh_tools").description == "host tool"
    assert "refresh_tools" not in gw._NAME_MAP


def test_two_bridge_names_cannot_map_to_the_same_mcp_name(fresh_gateway, monkeypatch):
    server = fresh_gateway
    _fake_registry(monkeypatch, {
        "demo.a/b": _meta("first", {}, "demo", "A"),
        "demo.a_b": _meta("second", {}, "demo", "A"),
    })
    with pytest.raises(RuntimeError, match="collid"):
        gw.discover_and_register_tools(server)
    assert not gw._NAME_MAP


def test_refresh_tools_without_a_simulator_says_what_to_do(monkeypatch):
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})

    def offline(_server):
        raise ConnectionRefusedError("[Errno 111] Connection refused")

    monkeypatch.setattr(gw, "discover_and_register_tools", offline)
    message = asyncio.run(gw._refresh_tools(server))
    assert "no Isaac Sim bridge is reachable" in message and "simulators action=launch" in message


def test_host_run_asset_tools_are_not_reported_as_unconfirmed(monkeypatch):
    offline = types.SimpleNamespace(names={"assets__roots": "assets.roots", "assets__scan": "assets.scan",
                                           "assets__search": "assets.search"})
    monkeypatch.setattr(gw, "_offline_for", lambda server: offline)
    assert gw._offline_note(object()) == ""
    offline.names["sim__stage__info"] = "sim.stage.info"
    note = gw._offline_note(object())
    assert "sim: 1" in note and "assets" not in note


def test_refresh_tools_with_several_bridges_says_pick_one(monkeypatch):
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})

    def several(_server):
        raise RuntimeError("multiple bridges running: ports 8766, 8781.")

    monkeypatch.setattr(gw, "discover_and_register_tools", several)
    message = asyncio.run(gw._refresh_tools(server))
    assert "use_bridge" in message and "do not start another" in message and "not reachable" not in message


def test_refresh_tools_names_managed_simulators_on_other_ports(monkeypatch):
    # The config pinned port 8766, launch_simulator used 8771, and refresh_tools
    # kept saying "no bridge" until the agent found use_bridge on its own.
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})

    def offline(_server):
        raise ConnectionRefusedError("[Errno 111] Connection refused")

    monkeypatch.setattr(gw, "discover_and_register_tools", offline)
    monkeypatch.setattr(gw, "_screen_port", lambda state=None: 8766)
    import time
    monkeypatch.setattr(gw, "live_sessions", lambda env=None: [
        {"port": 8766, "pid": 10, "started_at": time.time() - 95},
        {"port": 8771, "pid": 342055, "started_at": None}])
    message = asyncio.run(gw._refresh_tools(server))
    assert "port 8771 (PID 342055; call use_bridge(port) to select it)" in message
    # The configured port's own session is named too, with how long it has been starting.
    assert "port 8766 (PID 10, started 9" in message and "the configured port" in message
    assert "still starting" in message and "refresh_tools(wait_s=" in message
    monkeypatch.setattr(gw, "live_sessions", lambda env=None: [])
    assert "Managed simulators" not in asyncio.run(gw._refresh_tools(server))
