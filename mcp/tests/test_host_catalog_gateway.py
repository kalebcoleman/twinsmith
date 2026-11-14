"""Offline UR5e tool discovery: the host lists all 7 ``workcell__*`` tools with no bridge running.

A server built by ``build_server`` registers callable proxies for every tool in the packaged
catalog (``twinsmith/catalogs/workcell.json``). A call dials the selected bridge and, when there
is none or it lacks the extension, answers with an actionable error. A live bridge's own
``list_tools`` metadata replaces the proxy; the filters, hooks, per-server isolation and the
``refresh_tools`` / ``use_bridge`` semantics are unchanged. Nothing here starts a simulator
(``subprocess.Popen`` is made to fail), and every bridge is a fake or an in-process real socket.
"""

import asyncio
import importlib.util
import json
import os
import socket
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("mcp")

from mcp.server.fastmcp import FastMCP
from mcp.shared.memory import create_connected_server_and_client_session
import mcp.types as types

import twinsmith.screen as screen
import twinsmith.server as gw
from twinsmith import host_catalog, local_tools as lt, sim_sessions, tool_groups
from twinsmith_bridge.bridge import ToolDefinition
from twinsmith_bridge.schema import field_specs
from bridge_testing import LiveBridge
from test_workcell_shared_bridge import OWNER, attach, workcell_registry

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("gen_host_catalog", ROOT / "scripts" / "gen_host_catalog.py")
generator = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(generator)

PROXIES = {gw._mcp_name(canonical): canonical for canonical in host_catalog.catalog_tool_names() if canonical.startswith("workcell.")}
STATUS = "workcell__workcell_status"
# Registered host proxies: each catalogued member plus the grouped tools that list them.
REGISTERED = len(PROXIES) + sum(name.startswith("workcell__") for name in tool_groups.GROUPS)


@pytest.fixture(autouse=True)
def single_purpose_tool_list(member_listing):
    """These tests are about the single-purpose tools themselves, so they list them as before the groups."""
    pass


@pytest.fixture(autouse=True)
def isolated(monkeypatch, tmp_path):
    """No ambient bridge, filter or policy; capture dirs in tmp; and no way to spawn a process."""
    # This regression module exercises the existing UR5e-only bridge/catalog surface.
    # The full multi-catalog surface is covered in test_scene_gateway.py.
    full_catalog = host_catalog.load_catalog()
    monkeypatch.setattr(host_catalog, "load_catalog", lambda: __import__("copy").deepcopy({k:v for k,v in full_catalog.items() if k.startswith("workcell.")}))
    for var in ("TWINSMITH_PORT",
                "TWINSMITH_HOST", "TWINSMITH_TOOLSETS"):
        monkeypatch.delenv(var, raising=False)
    monkeypatch.setattr(screen, "SHOT_DIR", tmp_path / "shots")
    monkeypatch.setattr(screen, "VIDEO_DIR", tmp_path / "videos")

    def no_spawn(*args, **kwargs):
        raise AssertionError(f"the host started a process: {args!r}")

    monkeypatch.setattr(subprocess, "Popen", no_spawn)


class Cell:
    """Every tool answers with its own name and the arguments it received."""

    def __getattr__(self, name):
        if name.startswith("_cmd_"):
            return lambda **kwargs: {"handler": name[5:], "args": kwargs}
        raise AttributeError(name)


def attach_workcell(live, registry=None):
    attach(live.bridge, registry or workcell_registry(), Cell(), ToolDefinition, requested_port=live.port)


# These tests start fake bridges. A real managed simulator on the machine (its session record under the user's data folder)
# made refresh_tools add "a managed simulator is also running" and fail them, so every server looks in an empty folder.
_NO_REAL_SESSIONS = {"TWINSMITH_SIM_SESSIONS_DIR": tempfile.mkdtemp(prefix="twinsmith-no-sessions-")}


def build(env=None):
    return gw.build_server({"TWINSMITH_TOOLSETS": "all", **_NO_REAL_SESSIONS, **({} if env is None else env)})


def session(server, notes=None):
    async def handler(message):
        if notes is not None and isinstance(message, types.ServerNotification) and \
                isinstance(message.root, types.ToolListChangedNotification):
            notes.append(message)
    return create_connected_server_and_client_session(server._mcp_server, message_handler=handler)


def run(server, *steps, notes=None):
    """Run async steps (client -> value) in one session, return their values."""
    async def go():
        async with session(server, notes) as client:
            await client.initialize()
            return [await step(client) for step in steps]
    return asyncio.run(go())


async def tools(client):
    return {t.name: t for t in (await client.list_tools()).tools}


def call(name, args=None):
    async def step(client):
        result = await client.call_tool(name, args or {})
        return result.content[0].text
    return step


def enum_of(prop):
    """The enum of a property schema (optional enums render as anyOf [enum, null])."""
    return prop.get("enum") or next(o["enum"] for o in prop.get("anyOf", []) if "enum" in o)


def live_workcell_tools(server):
    return [name for name in gw.registered_tools(server) if name.startswith("workcell.")]


# ---- the snapshot ----------------------------------------------------------------------

def test_the_snapshot_is_packaged_valid_and_complete():
    assert len(PROXIES) == 8
    catalog = host_catalog.load_catalog()
    assert len(catalog) == 8 and all(name.startswith("workcell.") for name in catalog)
    assert {meta["owner_id"] for meta in catalog.values()} == {OWNER}
    # It is data inside the package, read through importlib.resources (what a wheel ships).
    from importlib import resources
    assert resources.files("twinsmith").joinpath("catalogs/workcell.json").is_file()
    # A caller cannot corrupt the cache.
    catalog["workcell.workcell_status"]["description"] = "tampered"
    assert host_catalog.load_catalog()["workcell.workcell_status"]["description"] != "tampered"


def test_the_snapshot_matches_the_extension_registry_exactly():
    """Drift guard: a registry edit without a regenerated snapshot fails here."""
    registry = workcell_registry()
    assert len(registry) == 8
    expected = {f"workcell.{name}": {
        "description": schema["description"],
        "parameters": schema.get("parameters", {}),
        "fields": field_specs(schema.get("parameters", {})),
        "namespace": "workcell", "owner_id": OWNER} for name, schema in registry.items()}
    assert host_catalog.load_catalog() == expected
    assert list(host_catalog.load_catalog()) == list(expected)          # declaration order too
    # The committed bytes are what the generator writes, and its --check agrees.
    path = ROOT / "mcp/twinsmith/catalogs/workcell.json"
    assert path.read_text(encoding="utf-8") == generator.render(generator.build_ur5e_catalog())
    assert generator.main(["--check"]) == 0


def test_the_generator_check_fails_on_drift(tmp_path, capsys):
    stale = tmp_path / "workcell.json"
    document = generator.build_ur5e_catalog()
    document["tools"]["workcell.workcell_status"]["description"] = "an old description"
    stale.write_text(generator.render(document), encoding="utf-8")
    assert generator.main(["--check", "--output", str(stale)]) == 1
    assert "stale" in capsys.readouterr().err
    assert generator.main(["--write", "--output", str(stale)]) == 0
    assert generator.main(["--check", "--output", str(stale)]) == 0


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d.update(version=99), "version"),
    (lambda d: d["tools"]["workcell.workcell_status"].update(namespace="other"), "namespace"),
    (lambda d: d["tools"]["workcell.jenga_build"]["fields"][0].update(kind="blob"), "malformed"),
    (lambda d: d["tools"]["workcell.jenga_build"]["fields"][0].update(name="renamed"), "do not match"),
    (lambda d: d["tools"].update({"elsewhere.tool": d["tools"]["workcell.workcell_status"]}), "namespace"),
])
def test_a_malformed_catalog_is_refused_loudly(monkeypatch, tmp_path, mutate, message):
    document = generator.build_ur5e_catalog()
    mutate(document)
    directory = tmp_path / "catalogs"
    directory.mkdir()
    (directory / "bad.json").write_text(json.dumps(document), encoding="utf-8")

    class Files:
        def joinpath(self, name):
            assert name == host_catalog.CATALOG_DIR
            return directory

    monkeypatch.setattr(host_catalog.resources, "files", lambda package: Files())
    with pytest.raises(host_catalog.CatalogError, match=message):
        host_catalog._read_all()


def test_a_missing_catalog_is_an_error_not_an_empty_one(monkeypatch, tmp_path):
    class Files:
        def joinpath(self, name):
            return tmp_path / "nowhere"

    monkeypatch.setattr(host_catalog.resources, "files", lambda package: Files())
    with pytest.raises(host_catalog.CatalogError, match="no packaged tool catalog"):
        host_catalog._read_all()


# ---- offline discovery and schema -------------------------------------------------------

def test_every_tool_is_listed_with_no_bridge_and_matches_the_registry_schema():
    server = build()
    (listed,) = run(server, tools)
    assert {name for name in listed if name.startswith("workcell__")} == set(PROXIES)
    registry = workcell_registry()
    for name, canonical in PROXIES.items():
        short = canonical.split(".", 1)[1]
        fields = field_specs(registry[short].get("parameters", {}))
        tool = listed[name]
        assert tool.description == registry[short]["description"]
        properties = tool.inputSchema.get("properties", {})
        assert list(properties) == [f["name"] for f in sorted(
            fields, key=lambda f: not (f["required"] and not f["has_default"]))]
        assert set(tool.inputSchema.get("required", [])) == {
            f["name"] for f in fields if f["required"] and not f["has_default"]}
        for f in fields:
            if f["kind"] == "enum":
                assert enum_of(properties[f["name"]]) == f["enum"], (name, f["name"])
    assert listed[STATUS].inputSchema.get("properties", {}) == {}
    assert set(listed["workcell__jenga_build"].inputSchema["properties"]) == {"profile", "seed"}
    assert gw.offline_tools(server) == sorted(PROXIES.values())
    assert live_workcell_tools(server) == []              # the roster of live bridge tools stays empty


def test_the_default_server_excludes_workcell():
    assert not set(PROXIES) & set(gw.build_server({})._tool_manager._tools)


def test_offline_tools_defaults_to_the_default_server_and_tracks_its_live_bridge(monkeypatch):
    default = build()
    other = build()
    monkeypatch.setattr(gw, "mcp", default)
    assert gw.offline_tools() == gw.offline_tools(None) == sorted(PROXIES.values())
    with LiveBridge() as live:
        attach_workcell(live)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        gw.discover_and_register_tools(default)
        assert gw.offline_tools() == []
        assert gw.offline_tools(other) == sorted(PROXIES.values())


def test_a_client_that_never_refreshes_still_sees_them_after_a_failed_startup_discovery():
    # Opening a session runs the lifespan, whose discovery fails (no bridge): the proxies remain.
    (listed,) = run(build(), tools)
    assert set(PROXIES) <= set(listed)


@pytest.mark.parametrize("groups", ["all", "default,+sim", "all,+computer"])
def test_local_tool_groups_filter_only_local_tools(groups):
    server = build({"TWINSMITH_TOOLSETS": groups})
    assert set(PROXIES) <= set(server._tool_manager._tools)
    assert all(__import__("twinsmith.toolsets", fromlist=["owner"]).owner(name) in gw._state_for(server).toolsets for name in server._tool_manager._tools)


def test_every_offline_proxy_answers_with_an_actionable_error_and_starts_nothing():
    server = build()
    registry = workcell_registry()
    values = {"boolean": True, "integer": 1, "number": 1.0, "array": [], "object": {}, "string": "x"}

    def args(canonical):
        out = {}
        for f in field_specs(registry[canonical.split(".", 1)[1]].get("parameters", {})):
            if f["required"] and not f["has_default"]:
                out[f["name"]] = f["enum"][0] if f["kind"] == "enum" else values[f["kind"]]
        return out

    answers = run(server, *[call(name, args(canonical)) for name, canonical in PROXIES.items()])
    for (name, canonical), answer in zip(PROXIES.items(), answers):
        assert answer.startswith(f"Error: {canonical} is not available"), (name, answer)
        assert "no Isaac Sim bridge could be reached" in answer
        for hint in ("--list-bridges", "use_bridge", "refresh_tools", "./run.sh --headless",
                     "never starts a simulator"):
            assert hint in answer, (name, hint)


def test_a_failure_after_connecting_is_not_reported_as_not_run(monkeypatch):
    """A connection lost mid-call (or a timeout) may have run the tool: keep the original error,
    never claim the call did not happen."""
    for error in (ConnectionError("bridge closed the connection mid-response"),
                  BrokenPipeError("broken pipe"), TimeoutError("no response within 300s")):
        class Dies:
            def call(self, tool, params=None, **kw):
                raise error

        server = build()
        gw._state_for(server).client = Dies()
        answer = gw._tool_implementation("workcell.workcell_status", {}, gw._state_for(server))
        assert answer == f"Error: {error}", answer



def test_several_bridges_and_no_choice_points_at_use_bridge(monkeypatch, tmp_path):
    import twinsmith.client as client_module
    monkeypatch.setattr(client_module, "discover_bridges", lambda: [
        {"pid": 1, "port": 8001, "app": "a"}, {"pid": 2, "port": 8002, "app": "b"}])
    (answer,) = run(build(), call(STATUS))
    assert "multiple bridges running" in answer and "use_bridge(port)" in answer


# ---- routing to a live bridge ------------------------------------------------------------

def test_a_proxy_call_reaches_a_bridge_that_appeared_after_startup(monkeypatch):
    server = build()
    with LiveBridge() as live:
        attach_workcell(live)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        play, assembled = run(
            server, call(STATUS),                                   # no refresh_tools first
            call("workcell__jenga_build", {"profile": "lab_unconfirmed", "seed": 42}))
    assert json.loads(play) == {"handler": "workcell_status", "args": {}}
    assert json.loads(assembled) == {"handler": "jenga_build", "args": {"profile": "lab_unconfirmed", "seed": 42}}


def test_refresh_makes_the_tools_live_without_telling_the_client_nothing_changed(monkeypatch):
    server = build()
    notes = []
    with LiveBridge() as live:
        attach_workcell(live)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        before, refreshed, after, again = run(
            server, tools, call("refresh_tools"), tools, call("refresh_tools"), notes=notes)
    assert "No new or changed tools" in refreshed and "Note:" not in refreshed
    assert not notes                                     # the listing a client has did not change
    assert set(PROXIES) <= set(before) and {k: v.inputSchema for k, v in before.items() if k in PROXIES} == {
        k: v.inputSchema for k, v in after.items() if k in PROXIES}
    assert live_workcell_tools(server) == sorted(PROXIES.values())     # now live, not proxies
    assert gw.offline_tools(server) == []
    assert "No new or changed tools" in again


def test_the_live_bridges_metadata_is_authoritative(monkeypatch):
    server = build()
    registry = workcell_registry()
    registry["workcell_status"] = {"description": "LIVE play", "parameters": {
        "speed": {"type": "number", "required": True}}}
    registry["brand_new"] = {"description": "only on this bridge", "parameters": {}}
    with LiveBridge() as live:
        attach_workcell(live, registry)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        changed = gw.discover_and_register_tools(server)
        notes = []
        listed, played = run(server, tools, call(STATUS, {"speed": 2.5}), notes=notes)
        refreshed, = run(server, call("refresh_tools"))
    assert "workcell.workcell_status (updated)" in changed and "workcell.brand_new" in changed
    assert sorted(c for c in changed if c.startswith("workcell.")) == [   # the other 22 are unchanged
        "workcell.brand_new", "workcell.workcell_status (updated)"]
    assert "No new or changed tools" in refreshed
    assert listed[STATUS].description == "LIVE play"
    assert listed[STATUS].inputSchema["required"] == ["speed"]
    assert "workcell__brand_new" in listed
    assert json.loads(played)["args"] == {"speed": 2.5}


def test_a_bridge_that_drops_the_extension_reverts_tools_to_their_proxies(monkeypatch):
    server = build()
    with LiveBridge() as live:
        attach_workcell(live, {**workcell_registry(), "workcell_status": {"description": "LIVE", "parameters": {}},
                           "brand_new": {"description": "extra", "parameters": {}}})
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        gw.discover_and_register_tools(server)
        assert gw.offline_tools(server) == [] and "workcell.brand_new" in live_workcell_tools(server)
        live.bridge.unregister_owner(OWNER)               # the work cell extension is unloaded
        changed = gw.discover_and_register_tools(server)
        listed, answer = run(server, tools, call(STATUS))
    assert sorted(changed) == ["workcell.brand_new (removed)", "workcell.workcell_status (offline)"]
    assert listed[STATUS].description == workcell_registry()["workcell_status"]["description"]
    assert set(PROXIES) <= set(listed) and "workcell__brand_new" not in listed
    assert gw.offline_tools(server) == sorted(PROXIES.values())
    assert "did not register it" in answer and "extension" in answer


# ---- disconnection and switches ----------------------------------------------------------

def test_calls_after_the_bridge_stops_say_so(monkeypatch):
    server = build()
    with LiveBridge() as live:
        attach_workcell(live)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        ok, = run(server, call("refresh_tools"))
        live_answer, = run(server, call(STATUS))
    assert json.loads(live_answer)["handler"] == "workcell_status"
    # The bridge is gone. The tools stay listed, a refresh reports the failure and changes nothing.
    stopped, refreshed, listed = run(server, call(STATUS), call("refresh_tools"), tools)
    assert "no Isaac Sim bridge could be reached" in stopped
    assert refreshed.startswith("Error:")
    assert set(PROXIES) <= set(listed)


def test_use_bridge_to_a_bridge_without_the_extension_keeps_every_proxy():
    server = build()
    with LiveBridge() as other:
        other.bridge.register_owner("demo", "demo").add(ToolDefinition(
            name="inspect", description="inspect", parameters={}, handler=lambda: {"seen": True}))
        switched, listed, play, inspect = run(
            server, call("use_bridge", {"port": other.port}), tools, call(STATUS),
            call("demo__inspect"))
    assert "switched to bridge" in switched
    assert "Note: workcell: 8 tool(s) are listed from the packaged catalog" in switched
    assert set(PROXIES) <= set(listed) and "demo__inspect" in listed
    assert "did not register it" in play and f"port {other.port}" in play
    assert json.loads(inspect) == {"seen": True}
    assert gw.offline_tools(server) == sorted(PROXIES.values())


def test_switching_between_a_twin_bridge_and_a_plain_one_follows_the_selected_bridge():
    server = build()
    with LiveBridge() as twin, LiveBridge() as plain:
        attach_workcell(twin)
        plain.bridge.register_owner("demo", "demo").add(ToolDefinition(
            name="inspect", description="inspect", parameters={}, handler=lambda: {}))
        notes = []
        to_twin, on_twin, to_plain, on_plain, back, again = run(
            server, call("use_bridge", {"port": twin.port}), call(STATUS),
            call("use_bridge", {"port": plain.port}), call(STATUS),
            call("use_bridge", {"port": twin.port}), call(STATUS), notes=notes)
        assert "Note:" not in to_twin and "Note:" not in back
        assert json.loads(on_twin)["handler"] == "workcell_status"
        assert "did not register it" in on_plain
        assert json.loads(again)["handler"] == "workcell_status"
        assert gw.offline_tools(server) == []
    assert notes                                          # each switch told the client


def test_a_failed_switch_target_changes_nothing():
    server = build()
    (answer,) = run(server, call("use_bridge", {"port": 1}))
    assert answer.startswith("Error:")
    assert gw.offline_tools(server) == sorted(PROXIES.values())


def test_use_bridge_whose_discovery_fails_still_leaves_the_proxies(monkeypatch):
    server = build()

    class Broken:
        port = 9
        def call(self, tool, params=None, **kw):
            if tool == "status":
                return {"pid": 1}
            raise OSError("bridge died mid-discovery")
        def close(self):
            pass

    monkeypatch.setattr(gw, "BridgeClient", lambda **kw: Broken())
    switched, listed = run(server, call("use_bridge", {"port": 9}), tools)
    assert "tool discovery failed" in switched
    assert set(PROXIES) <= set(listed)


def test_servers_keep_independent_bridges_and_independent_proxies(monkeypatch):
    monkeypatch.setenv("TWINSMITH_PORT", "1")           # b's own choice: nothing listens there
    a, b = build(), build()
    with LiveBridge() as twin:
        attach_workcell(twin)
        # One session: closing it drops the connection, and a server's choice lasts as long as it.
        _, on_a = run(a, call("use_bridge", {"port": twin.port}), call(STATUS))
        assert gw.offline_tools(a) == [] and gw.offline_tools(b) == sorted(PROXIES.values())
        (on_b,) = run(b, call(STATUS))
    assert json.loads(on_a)["handler"] == "workcell_status"
    assert "not available" in on_b
    assert gw._state_for(a) is not gw._state_for(b)


def test_bare_fastmcp_servers_do_not_grow_proxies(monkeypatch):
    monkeypatch.setattr(gw, "_NAME_MAP", {})
    monkeypatch.setattr(gw, "_REGISTERED_META", {})

    class Fake:
        def call(self, tool, params=None, **kw):
            return {"tools": {}}

    monkeypatch.setattr(gw, "get_client", lambda: Fake())
    bare = FastMCP("bare")
    assert gw.discover_and_register_tools(bare) == []
    assert not bare._tool_manager._tools and gw.offline_tools(bare) == []


# ---- filters, hooks, collisions ----------------------------------------------------------

def test_toolsets_filter_proxies():
    server = build({"TWINSMITH_TOOLSETS": "default"})
    assert not set(PROXIES) & set(server._tool_manager._tools)
    assert gw.offline_tools(server) == []
    assert set(PROXIES) <= set(build({"TWINSMITH_TOOLSETS": "default,+sim"})._tool_manager._tools)


def test_a_namespace_filter_that_hides_ur5e_keeps_other_tools_and_survives_switches(monkeypatch):
    server = build({"TWINSMITH_TOOLSETS": "default,+assets"})
    with LiveBridge() as live:
        attach_workcell(live)
        live.bridge.register_owner("demo", "demo").add(ToolDefinition(
            name="inspect", description="inspect", parameters={}, handler=lambda: {"ok": 1}))
        run(server, call("use_bridge", {"port": live.port}))
        (listed,) = run(server, tools)
    assert "demo__inspect" not in listed and not set(PROXIES) & set(listed)
    assert set(gw.registered_tools(server)) == {"status", "job_status", "job_cancel"}
    assert gw.offline_tools(server) == []


def test_changing_the_filter_is_applied_at_the_next_switch(monkeypatch):
    server = build()
    assert gw.offline_tools(server)
    gw._state_for(server).toolsets = frozenset({"core"})
    with LiveBridge() as live:
        live.bridge.register_owner("demo", "demo").add(ToolDefinition(
            name="inspect", description="inspect", parameters={}, handler=lambda: {}))
        run(server, call("use_bridge", {"port": live.port}))
        (listed,) = run(server, tools)
    assert not set(PROXIES) & set(listed) and gw.offline_tools(server) == []


@pytest.mark.parametrize("action", ["refresh_tools", "use_bridge"])
@pytest.mark.parametrize("one_live_tool", [False, True])
def test_filter_changes_notify_for_catalog_tools(monkeypatch, action, one_live_tool):
    class EmptyBridge:
        port = 9

        def call(self, tool, params=None, **kwargs):
            if tool == "list_tools":
                catalog = host_catalog.load_catalog()
                return {"tools": {"workcell.workcell_status": catalog["workcell.workcell_status"]} if one_live_tool else {}}
            return {"pid": 1, "port": self.port}

        def close(self):
            pass

    monkeypatch.setattr(gw, "BridgeClient", lambda **kwargs: EmptyBridge())
    server, notes = build(), []
    args = {"port": 9} if action == "use_bridge" else {}

    async def go():
        async with session(server, notes) as client:
            await client.initialize()
            before = await tools(client)
            gw._state_for(server).toolsets = frozenset({"core"})
            hidden = await call(action, args)(client)
            hidden_names, hidden_notes = await tools(client), len(notes)
            again = await call(action, args)(client)
            unchanged_notes = len(notes)
            gw._state_for(server).toolsets = frozenset({"core", "sim"})
            shown = await call(action, args)(client)
            return before, hidden, hidden_names, hidden_notes, again, unchanged_notes, shown, await tools(client)

    before, hidden, hidden_names, hidden_notes, again, unchanged_notes, shown, after = asyncio.run(go())
    assert set(PROXIES) <= set(before) and not set(PROXIES) & set(hidden_names)
    assert set(PROXIES) <= set(after)
    # refresh_tools counts catalogued members; use_bridge also counts the grouped tools that list them.
    expected = len(PROXIES) if action == "refresh_tools" else REGISTERED
    assert f"{expected} tool(s)" in hidden and f"{expected} tool(s)" in shown
    assert "0 tool(s)" in again or "registry unchanged" in again
    if action == "refresh_tools":
        assert all(f"{canonical} (removed)" in hidden for canonical in PROXIES.values())
        assert all(canonical in shown for canonical in PROXIES.values())
    assert (hidden_notes, unchanged_notes, len(notes)) == (1, 1, 2)


def test_switch_to_identical_client_surface_does_not_notify(monkeypatch):
    class Bridge:
        def __init__(self, port=None, **kwargs):
            self.port = port

        def call(self, tool, params=None, **kwargs):
            if tool == "status":
                return {"pid": 1, "port": self.port}
            catalog = host_catalog.load_catalog()
            return {"tools": {"workcell.workcell_status": catalog["workcell.workcell_status"]}}

        def close(self):
            pass

    monkeypatch.setattr(gw, "BridgeClient", Bridge)
    server, notes = build(), []
    async def go():
        async with session(server, notes) as client:
            await client.initialize()
            answer = await call("use_bridge", {"port": 9})(client)
            return answer, await tools(client), gw._state_for(server).client.port

    answer, listed, port = asyncio.run(go())
    assert "0 tool(s)" in answer and set(PROXIES) <= set(listed)
    assert port == 9 and not notes


def test_failed_switch_still_notifies_when_it_changes_offline_proxies(monkeypatch):
    class Bridge:
        def __init__(self, port=None, **kwargs):
            self.port = port

        def call(self, tool, params=None, **kwargs):
            if tool == "status":
                return {"pid": 1, "port": self.port}
            if self.port == 9:
                raise OSError("bridge died mid-discovery")
            return {"tools": {}}

        def close(self):
            pass

    monkeypatch.setattr(gw, "BridgeClient", Bridge)
    server, notes = build(), []

    async def go():
        async with session(server, notes) as client:
            await client.initialize()
            gw._state_for(server).toolsets = frozenset({"core"})
            answer = await call("use_bridge", {"port": 9})(client)
            return answer, await tools(client)

    answer, listed = asyncio.run(go())
    assert "tool discovery failed" in answer and f"Changed {REGISTERED} tool(s)" in answer
    assert not set(PROXIES) & set(listed) and len(notes) == 1


def test_the_builtins_gate_does_not_hide_catalog_tools(monkeypatch):
    server = build({"TWINSMITH_TOOLSETS": "default,+sim"})
    assert set(PROXIES) <= set(server._tool_manager._tools)
    with LiveBridge() as live:
        attach_workcell(live)
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        refreshed, listed = run(server, call("refresh_tools"), tools)
    assert "run_python" in listed and set(PROXIES) <= set(listed)          # sim holds both



def test_a_live_tool_may_not_take_an_offline_proxys_name_under_another_canonical(monkeypatch):
    server = build()
    state = gw._state_for(server)
    clash = {"description": "d", "parameters": {}, "fields": [], "namespace": "x", "owner_id": "B"}

    class Fake:
        def call(self, tool, params=None, **kw):
            return {"tools": {"workcell__workcell_status": clash}}   # same MCP name, other canonical

    state.client = Fake()
    with pytest.raises(RuntimeError, match="collides with host tool"):
        gw.discover_and_register_tools(server)
    assert gw.offline_tools(server) == sorted(PROXIES.values())     # nothing was lost


def test_a_catalog_name_that_collides_with_a_host_tool_fails_the_build(monkeypatch):
    meta = {"description": "d", "parameters": {}, "fields": [], "namespace": "isaaclab",
            "owner_id": "o"}
    monkeypatch.setattr(host_catalog, "load_catalog", lambda: {"isaaclab.install_info": meta})
    with pytest.raises(RuntimeError, match="collides with host tool 'isaaclab__install_info'"):
        build()


def test_the_host_tool_collision_check_still_protects_local_tools(monkeypatch):
    server = build()
    state = gw._state_for(server)
    meta = {"description": "d", "parameters": {}, "fields": [], "namespace": "isaaclab",
            "owner_id": "B"}

    class Fake:
        def call(self, tool, params=None, **kw):
            return {"tools": {"isaaclab.install_info": meta}}

    state.client = Fake()
    with pytest.raises(RuntimeError, match="collides with host tool"):
        gw.discover_and_register_tools(server)


def test_the_offline_proxies_carry_hints_from_their_own_table():
    server = build({"TWINSMITH_TOOLSETS": "default,+sim"})
    tool = server._tool_manager._tools[STATUS]
    assert tool.annotations.readOnlyHint is True and tool.annotations.destructiveHint is False


def test_a_tool_from_a_projects_own_extension_gets_no_hints():
    server = build({"TWINSMITH_TOOLSETS": "default,+sim"})
    gw.register_dynamic_tool(server, "demo__ping", "demo.ping", {"description": "d", "parameters": {}, "fields": []})
    assert server._tool_manager._tools["demo__ping"].annotations is None


def test_instructions_explain_selection_without_advertising_optional_tools():
    assert "status" in gw.SERVER_INSTRUCTIONS and "toolsets" in gw.SERVER_INSTRUCTIONS
    assert "workcell__" not in gw.SERVER_INSTRUCTIONS


def test_the_wheel_config_ships_the_whole_package_directory():
    """hatch's `packages = ["twinsmith"]` takes every file in the package, the catalog JSON
    included (verified with `uv build`); an include/exclude rule here could silently drop it."""
    try:
        import tomllib
    except ModuleNotFoundError:  # Python 3.10
        import tomli as tomllib
    with open(ROOT / "mcp/pyproject.toml", "rb") as handle:
        targets = tomllib.load(handle)["tool"]["hatch"]["build"]["targets"]
    assert targets["wheel"]["packages"] == ["twinsmith"]
    for target in targets.values():
        assert not {"exclude", "only-include", "only-packages"} & set(target)
    from importlib import resources
    packaged = resources.files("twinsmith").joinpath(host_catalog.CATALOG_DIR)
    assert sorted(entry.name for entry in packaged.iterdir() if entry.name.endswith(".json")) == ["assets.json", "builtins.json", "sim.json", "workcell.json"]



@pytest.mark.parametrize("arguments", [{}, {"profile": "lab_unconfirmed"}, {"seed": 42},
                                        {"profile": "lab_unconfirmed", "seed": 42}])
def test_jenga_is_accepted_by_real_mcp_schema_and_forwarded(monkeypatch, arguments):
    """Optional variation arguments survive the real MCP schema and gateway."""
    with LiveBridge() as live:
        attach_workcell(live)
        server = build({"TWINSMITH_PORT": str(live.port)})

        async def exercise(client):
            listed = await tools(client)
            assert set(listed["workcell__jenga_build"].inputSchema["properties"]) == {"profile", "seed"}
            response = await client.call_tool("workcell__jenga_build", arguments)
            assert not response.isError, response
            data = json.loads(response.content[0].text)
            assert data == {"handler": "jenga_build", "args": arguments}
            refused = await client.call_tool("workcell__jenga_build", {"seed": "not-an-integer"})
            assert refused.isError

        run(server, exercise)


# ---- refresh_tools(wait_s) waited on the pinned port, not launch_simulator's ----------

def dead_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@pytest.fixture()
def managed(monkeypatch, tmp_path):
    """A server with launch_simulator whose launches are faked as session records of a live process."""
    env = {"TWINSMITH_TOOLSETS": "all,+computer",
           "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path / "sessions")}

    def write_session(port, pid=None):
        pid = os.getpid() if pid is None else pid
        record = {"id": uuid.uuid4().hex, "pid": pid, "port": port, "ros_domain": 9,
                  "start_ticks": sim_sessions._process_identity(pid)[1],
                  "boot_id": sim_sessions._boot_id(), "started_at": time.time()}
        directory = tmp_path / "sessions" / record["id"]
        directory.mkdir(parents=True)
        (directory / "session.json").write_text(json.dumps(record))
        return record

    pid_box = {}
    monkeypatch.setattr(sim_sessions, "_launch", lambda settings, port, *args: write_session(port, pid_box.get("pid")))
    server = gw.build_server(env)

    async def selected(client):
        return gw._state_for(server).get_client().port

    return SimpleNamespace(server=server, write_session=write_session, selected=selected, pid_box=pid_box,
                           launch=lambda port: call("launch_simulator", {"port": port, "ros_domain": 9}),
                           state=gw._state_for(server))


def test_refresh_selects_the_launched_session_when_nothing_listens_on_the_pinned_port(managed, monkeypatch):
    pinned = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    notes = []
    with LiveBridge() as live:
        launched, message, port, status = run(
            managed.server, managed.launch(live.port), call("refresh_tools", {"wait_s": 5}),
            managed.selected, call("status"), notes=notes)
    session_id = json.loads(launched)["id"]
    assert port == live.port and json.loads(status)["port"] == live.port
    assert f"Selected the bridge on port {live.port} (PID {os.getpid()})" in message
    assert f"port {pinned}, which this server was using, did not answer" in message
    assert session_id in message and "launched by this server" in message
    assert f"switched to bridge on port {live.port}" in message   # the use_bridge path
    assert "Confirm the PID with status" in message
    assert notes, "the client must hear that the tool list changed"


def test_refresh_without_wait_also_selects_a_ready_launched_session(managed, monkeypatch):
    monkeypatch.setenv("TWINSMITH_PORT", str(dead_port()))
    with LiveBridge() as live:
        _, message, port = run(managed.server, managed.launch(live.port), call("refresh_tools"), managed.selected)
    assert port == live.port and "Selected the bridge" in message


def test_refresh_wait_polls_the_session_until_it_is_ready(managed, monkeypatch):
    pinned = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    monkeypatch.setattr(gw, "REFRESH_POLL_S", 0.01)
    real, checks = gw._session_not_ready, []

    def slow_boot(session):
        checks.append(session["port"])
        return "still booting" if len(checks) < 3 else real(session)

    monkeypatch.setattr(gw, "_session_not_ready", slow_boot)
    with LiveBridge() as live:
        _, message, port = run(managed.server, managed.launch(live.port),
                               call("refresh_tools", {"wait_s": 5}), managed.selected)
    assert port == live.port and checks == [live.port] * 3 and "Selected the bridge" in message


def test_refresh_wait_timeout_names_the_session_port_it_polled(managed, monkeypatch):
    pinned, session_port = dead_port(), dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    monkeypatch.setattr(gw, "REFRESH_POLL_S", 0.01)
    monkeypatch.setattr(gw, "REFRESH_WAIT_MAX_S", 0.05)
    _, message, port = run(managed.server, managed.launch(session_port),
                           call("refresh_tools", {"wait_s": 5}), managed.selected)
    assert port == pinned and "waited 0 s" in message
    assert f"polls port {pinned} and port {session_port}" in message
    assert "polls for you" not in message
    assert f"port {session_port} (PID {os.getpid()}) is not answering yet" in message


def test_refresh_never_leaves_a_pinned_bridge_that_answers(managed, monkeypatch):
    monkeypatch.setattr(gw, "_session_not_ready", lambda session: pytest.fail("polled away from a live pin"))
    with LiveBridge() as pinned, LiveBridge() as launched:
        monkeypatch.setenv("TWINSMITH_PORT", str(pinned.port))
        _, message, port = run(managed.server, managed.launch(launched.port),
                               call("refresh_tools", {"wait_s": 5}), managed.selected)
    assert port == pinned.port and "Selected the bridge" not in message
    assert f"stayed on port {pinned.port}, whose bridge answered" in message
    assert f"port {launched.port} (PID {os.getpid()})" in message and "use_bridge(port)" in message


def test_refresh_does_not_leave_a_pinned_port_that_accepts_but_is_still_booting(managed, monkeypatch):
    monkeypatch.setattr(gw, "DISCOVERY_TIMEOUT_S", 0.05)
    monkeypatch.setattr(gw, "_session_not_ready", lambda session: pytest.fail("left a booting pin"))
    with socket.socket() as booting, LiveBridge() as launched:
        booting.bind(("127.0.0.1", 0))
        booting.listen(8)
        pinned = booting.getsockname()[1]
        monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
        _, message, port = run(managed.server, managed.launch(launched.port),
                               call("refresh_tools"), managed.selected)
    assert port == pinned and "accepted the connection" in message and "Selected" not in message
    assert "polls for you" not in message and f"use_bridge(port={launched.port})" in message


def test_refresh_does_not_guess_between_several_managed_sessions(managed, monkeypatch):
    pinned = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    monkeypatch.setattr(gw, "_session_not_ready", lambda session: pytest.fail("guessed"))
    with LiveBridge() as first, LiveBridge() as second:
        _, _, message, port = run(managed.server, managed.launch(first.port), managed.launch(second.port),
                                  call("refresh_tools", {"wait_s": 0}), managed.selected)
    assert port == pinned and "Selected the bridge" not in message
    assert f"port {first.port}" in message and f"port {second.port}" in message
    assert "will not guess" in message and "polls for you" not in message


def test_refresh_names_but_does_not_select_a_session_this_server_did_not_launch(managed, monkeypatch):
    # The session directory is shared with other hosts and agents.
    pinned = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    with LiveBridge() as foreign:
        managed.write_session(foreign.port)
        message, port = run(managed.server, call("refresh_tools", {"wait_s": 0}), managed.selected)
    assert port == pinned and "Selected the bridge" not in message
    assert f"port {foreign.port}" in message and f"use_bridge(port={foreign.port})" in message
    assert "polls for you" not in message and f"keeps dialling port {pinned}" in message


def test_refresh_refuses_a_session_port_answered_by_another_process(managed, monkeypatch):
    pinned = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(pinned))
    managed.pid_box["pid"] = os.getppid()
    with LiveBridge() as impostor:
        _, message, port = run(managed.server, managed.launch(impostor.port),
                               call("refresh_tools"), managed.selected)
    assert port == pinned and "Selected the bridge" not in message
    assert f"answered as PID {os.getpid()}, not the managed session's PID {os.getppid()}" in message


def test_refresh_waits_for_the_session_app_to_finish_loading(managed, monkeypatch):
    from twinsmith_bridge import builtins as bridge_builtins

    monkeypatch.setenv("TWINSMITH_PORT", str(dead_port()))
    monkeypatch.setattr(bridge_builtins, "_app_ready", lambda: False)

    async def app_ready(client):
        monkeypatch.setattr(bridge_builtins, "_app_ready", lambda: True)

    with LiveBridge() as live:
        _, loading, _, ready, port = run(managed.server, managed.launch(live.port), call("refresh_tools"),
                                         app_ready, call("refresh_tools"), managed.selected)
    assert "still loading" in loading and "Selected the bridge" not in loading
    assert port == live.port and "Selected the bridge" in ready


def test_a_session_on_the_selected_port_is_waited_for_not_replaced(managed, monkeypatch):
    # The pin itself is the launched session, still starting: there is nothing to switch to.
    session_port = dead_port()
    monkeypatch.setenv("TWINSMITH_PORT", str(session_port))
    _, message, port = run(managed.server, managed.launch(session_port), call("refresh_tools"), managed.selected)
    assert port == session_port and f"polls port {session_port} for you" in message


def test_a_remote_dial_host_is_not_redirected_to_a_local_session(managed, monkeypatch):
    with LiveBridge() as live:
        record = managed.write_session(live.port)
        managed.state.launched_sessions.add(record["id"])
        monkeypatch.setenv("TWINSMITH_PORT", str(dead_port()))
        assert gw._followable_session(managed.state)["port"] == live.port
        monkeypatch.setenv("TWINSMITH_HOST", "192.0.2.1")
        assert gw._followable_session(managed.state) is None


def test_refresh_with_no_pin_and_no_bridge_reports_it(monkeypatch, tmp_path):
    # client.port raises when discovery finds nothing; the hint must not turn that into a crash.
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    server = build({"TWINSMITH_TOOLSETS": "all", "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path / "none")})
    (message,) = run(server, call("refresh_tools"))
    assert message.startswith("Error: no Isaac Sim bridge is reachable") and "polls for you" in message


def test_refresh_hint_with_no_pin_names_only_the_session_port(managed, monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    session_port = dead_port()
    _, message = run(managed.server, managed.launch(session_port), call("refresh_tools"))
    assert f"refresh_tools(wait_s=120) polls port {session_port}, and selects" in message
    assert "port None" not in message


def test_use_bridge_still_switches_to_any_port_and_keeps_its_result(managed, monkeypatch):
    monkeypatch.setenv("TWINSMITH_PORT", str(dead_port()))
    with LiveBridge() as live:
        message, port = run(managed.server, call("use_bridge", {"port": live.port}), managed.selected)
    assert message.startswith(f"switched to bridge on port {live.port}\n") and port == live.port
    assert "Selected the bridge" not in message


def test_launch_notes_no_longer_claim_refresh_cannot_switch(monkeypatch, tmp_path):
    record = {"id": "b" * 32, "pid": os.getpid(), "port": 18766, "ros_domain": 9,
              "start_ticks": sim_sessions._process_identity(os.getpid())[1], "boot_id": sim_sessions._boot_id()}
    directory = tmp_path / record["id"]
    directory.mkdir()
    (directory / "session.json").write_text(json.dumps(record))
    monkeypatch.setattr(sim_sessions.client, "call", lambda *a, **k: {"pid": os.getpid(), "app_ready": True})
    monkeypatch.setattr(sim_sessions, "_resources", lambda: {})
    status = sim_sessions._status({"TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path)}, record["id"])
    assert "use_bridge(port=18766)" in status["next"] and "while that port answers" in status["next"]


# ---- status reports what the MCP host process is running -------------------------

COMMIT_A, COMMIT_B = "a" * 40, "b" * 40


def fake_checkout(root, head=f"ref: refs/heads/main", loose=COMMIT_A):
    git = root / ".git"
    (git / "refs" / "heads").mkdir(parents=True)
    (git / "HEAD").write_text(head + "\n")
    if loose:
        (git / "refs" / "heads" / "main").write_text(loose + "\n")
    return root


def test_read_git_head_handles_loose_packed_detached_and_worktree_layouts(tmp_path):
    loose = fake_checkout(tmp_path / "loose")
    assert gw._read_git_head(loose) == COMMIT_A
    detached = fake_checkout(tmp_path / "detached", head=COMMIT_B)
    assert gw._read_git_head(detached) == COMMIT_B
    packed = fake_checkout(tmp_path / "packed", loose=None)
    (packed / ".git" / "packed-refs").write_text(
        f"# pack-refs with: peeled\n{COMMIT_B} refs/heads/other\n{COMMIT_A} refs/heads/main\n")
    assert gw._read_git_head(packed) == COMMIT_A
    # A worktree or submodule: .git is a file, and refs live in the common directory.
    common = fake_checkout(tmp_path / "main")
    private = common / ".git" / "worktrees" / "w"
    private.mkdir(parents=True)
    (private / "HEAD").write_text("ref: refs/heads/main\n")
    (private / "commondir").write_text("../..\n")
    worktree = tmp_path / "w"
    worktree.mkdir()
    (worktree / ".git").write_text(f"gitdir: {private}\n")
    assert gw._read_git_head(worktree) == COMMIT_A


@pytest.mark.parametrize("head", ["garbage", "ref: refs/heads/missing"])
def test_read_git_head_reports_nothing_rather_than_guessing(tmp_path, head):
    assert gw._read_git_head(fake_checkout(tmp_path / "bad", head=head)) is None
    assert gw._read_git_head(tmp_path / "absent") is None
    assert gw._read_git_head(None) is None


def test_find_checkout_recognizes_a_source_tree_but_not_an_installed_copy(tmp_path):
    tree = fake_checkout(tmp_path / "tree")
    (tree / "mcp" / "twinsmith").mkdir(parents=True)
    (tree / "mcp" / "pyproject.toml").write_text("")
    assert gw._find_checkout(str(tree / "mcp" / "twinsmith" / "server.py")) == tree.resolve()
    installed = fake_checkout(tmp_path / "repo") / ".venv" / "lib" / "site-packages" / "twinsmith"
    installed.mkdir(parents=True)
    assert gw._find_checkout(str(installed / "server.py")) is None


def test_host_identity_warns_only_after_the_checkout_moves(tmp_path, monkeypatch):
    checkout = fake_checkout(tmp_path / "tree")
    monkeypatch.setattr(gw, "_HOST_CHECKOUT", checkout)
    monkeypatch.setattr(gw, "_HOST_COMMIT", COMMIT_A)
    identity = gw._host_identity()
    assert identity["commit"] == identity["checkout_commit"] == COMMIT_A
    assert identity["stale"] is False and "warning" not in identity and identity["pid"] == os.getpid()
    (checkout / ".git" / "refs" / "heads" / "main").write_text(COMMIT_B + "\n")   # a git pull
    identity = gw._host_identity()
    assert identity["commit"] == COMMIT_A and identity["checkout_commit"] == COMMIT_B and identity["stale"] is True
    assert COMMIT_A[:12] in identity["warning"] and COMMIT_B[:12] in identity["warning"]
    assert "Restart the MCP connection" in identity["warning"]
    (checkout / ".git" / "HEAD").write_text("garbage")                              # unreadable now
    identity = gw._host_identity()
    assert identity["checkout_commit"] == "unknown" and identity["stale"] is None and "warning" not in identity


def test_host_identity_of_an_installed_wheel_is_just_the_version(monkeypatch):
    monkeypatch.setattr(gw, "_HOST_CHECKOUT", None)
    monkeypatch.setattr(gw, "_HOST_COMMIT", None)
    monkeypatch.setattr(gw, "_HOST_VERSION", "9.9.9")
    assert gw._host_identity() == {"version": "9.9.9", "pid": os.getpid()}


def test_status_through_the_host_adds_host_and_keeps_every_bridge_field(monkeypatch, tmp_path):
    from twinsmith.client import call as direct_call

    checkout = fake_checkout(tmp_path / "tree")
    monkeypatch.setattr(gw, "_HOST_CHECKOUT", checkout)
    monkeypatch.setattr(gw, "_HOST_COMMIT", COMMIT_B)   # the server started before the pull
    with LiveBridge() as live:
        monkeypatch.setenv("TWINSMITH_PORT", str(live.port))
        direct = direct_call("status", port=live.port, timeout=5)
        assert "host" not in direct, "the bridge must not grow a field this would overwrite"
        answer, short = run(build(), call("status", {"detail": True}), call("status"))
    payload = json.loads(answer)
    host = payload.pop("host")
    assert "core" in payload.pop("toolsets")["on"]
    assert {"default", "default_from"} <= set(payload.pop("physics"))      # the host adds the engine setting too
    assert set(payload) == set(direct) and payload["pid"] == direct["pid"] and payload["port"] == live.port
    assert host["stale"] is True and host["commit"] == COMMIT_B and "warning" in host
    assert str(tmp_path) not in answer, "status must not publish the checkout path"
    brief = json.loads(short)
    assert brief["running"] is True and brief["pid"] == direct["pid"] and brief["port"] == live.port
    assert not {"namespaces", "extension"} & set(brief), "the default reply is the short one"
    assert "warning" in brief["host"] and len(short) < len(answer)


def test_status_with_no_bridge_still_reports_the_host(monkeypatch, tmp_path):
    monkeypatch.setenv("TWINSMITH_PORT", str(dead_port()))
    checkout = fake_checkout(tmp_path / "tree")
    monkeypatch.setattr(gw, "_HOST_CHECKOUT", checkout)
    monkeypatch.setattr(gw, "_HOST_COMMIT", COMMIT_B)
    state = gw._state_for(build({"TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path / "none")}))
    answer = json.loads(gw._dispatch("status", {}, state))
    assert answer["running"] is False and "toolsets" in answer["message"], "no bridge is an answer, not an error"
    assert answer["bridges"] == [] and "Restart the MCP connection" in answer["host"]["warning"]
    other = gw._dispatch("workcell.workcell_status", {}, state)
    assert "MCP host:" not in other


# ---- help text must work on a stock python3 --------------------------------------

def test_bridge_listing_help_is_one_working_form(monkeypatch, tmp_path):
    bare = "python -m twinsmith.client"    # fails on a python3 without the package
    assert bare not in gw.LIST_BRIDGES_HELP
    assert "python3 mcp/twinsmith/client.py --list-bridges" in gw.LIST_BRIDGES_HELP
    assert (ROOT / "mcp" / "twinsmith" / "client.py").is_file()
    server = build({"TWINSMITH_TOOLSETS": "all", "TWINSMITH_SIM_SESSIONS_DIR": str(tmp_path / "none")})
    state = gw._state_for(server)
    listed = {tool.name: tool for tool in asyncio.run(server.list_tools())}
    refused = ConnectionRefusedError("[Errno 111] Connection refused")
    full_use_bridge = json.loads(gw._tool_help(server, "use_bridge"))["use_bridge"]["description"]   # the listing is short
    assert "Find candidates with status" in listed["use_bridge"].description
    messages = [gw._BRIDGE_HELP, full_use_bridge,
                gw._catalog_error("status", refused, state), gw._catalog_error("workcell.workcell_status", refused, state),
                gw._starting_hint(server, refused)]
    for message in messages:
        assert gw.LIST_BRIDGES_HELP in message and bare not in message
    # gen_tool_reference reads docstrings from the source, so use_bridge must keep one.
    reference_spec = importlib.util.spec_from_file_location("gen_tool_reference", ROOT / "scripts" / "gen_tool_reference.py")
    reference = importlib.util.module_from_spec(reference_spec)
    reference_spec.loader.exec_module(reference)
    summary = reference.local_tools(ROOT / "mcp" / "twinsmith" / "server.py")["use_bridge"][0]
    assert summary and listed["use_bridge"].description.startswith(summary)
    source = (ROOT / "mcp" / "twinsmith" / "server.py").read_text()
    assert source.count("--list-bridges`") == 2, "the two forms live only in LIST_BRIDGES_HELP"
    monkeypatch.setattr(gw, "discover_and_register_tools",
                        lambda s: (_ for _ in ()).throw(RuntimeError("multiple bridges running (a, b)")))
    several = asyncio.run(gw._refresh_tools(server))
    assert gw.LIST_BRIDGES_HELP in several and "use_bridge(port)" in several
