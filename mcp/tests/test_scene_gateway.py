import asyncio
import importlib.util
import json
from pathlib import Path
import pytest
from mcp.shared.memory import create_connected_server_and_client_session
from twinsmith import server as gw
from twinsmith.scene_contracts import REGISTRY
from twinsmith.host_catalog import load_catalog

_GRAPH_BUILD = Path(__file__).resolve().parents[2] / "exts/sim-tools/sim_tools/graph_build.py"
_spec = importlib.util.spec_from_file_location("sim_tools_graph_build_gateway", _GRAPH_BUILD)
gb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gb)
TICK, PUB = "omni.graph.action.OnPlaybackTick", "isaacsim.ros2.bridge.ROS2PublishClock"


@pytest.fixture(autouse=True)
def single_purpose_tool_list(member_listing):
    """These tests are about the single-purpose tools themselves, so they list them as before the groups."""
    pass


def test_full_catalog_and_errors(monkeypatch):
    catalog = load_catalog()
    builtins = {"run_python", "status", "reload_tools", "put_file", "get_file", "stat_file", "job_status", "job_cancel"}
    assert len(catalog) == 8 + len(REGISTRY) + len(builtins)
    assert {k for k in catalog if "." not in k} == builtins
    assert len([k for k in catalog if k.startswith("workcell.")]) == 8
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})

    class Offline:
        def call(self, *args, **kw):
            raise ConnectionRefusedError("test offline")

        def close(self):
            pass

    gw._state_for(server).client = Offline()

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as c:
            await c.initialize()
            listed = {t.name: t for t in (await c.list_tools()).tools}
            assert all(gw._mcp_name(k) in listed for k in REGISTRY)
            schema = listed["assets__import_path"].inputSchema
            assert {"session_id", "stage_id", "prim_path"} <= set(schema["required"])
            assert "snapshot_id" not in schema["properties"]
            assert schema["properties"]["orientation"]["items"]["type"] == "number"
            assert schema["properties"]["orientation"]["maxItems"] == 4
            assert "quaternion" in schema["properties"]["orientation"]["description"]
            assert not listed["assets__import_path"].annotations.readOnlyHint
            for name in ("assets__import_path",):
                schema = listed[name].inputSchema
                needed = {"session_id", "stage_id"} | ({"source_path"} if name != "assets__import_path" else set())
                assert needed <= set(schema["required"])
                assert not listed[name].annotations.readOnlyHint
            assert {"source_path", "root_id", "relative_path"} <= set(listed["assets__import_path"].inputSchema["properties"])
            assert "prim_path" in listed["assets__import_path"].inputSchema["required"]
            save_schema = listed["sim__stage__save"].inputSchema
            reset = save_schema["properties"]["reset_camera_render_handles"]
            assert reset["type"] == "boolean" and reset["default"] is False
            assert "reset_camera_render_handles" not in save_schema["required"]
            assert "export policy" in reset["description"]
            camera_wait = listed["sim__camera__capture"].inputSchema["properties"][
                "wait_for_render"
            ]
            lidar_wait = listed["sim__lidar__read"].inputSchema["properties"][
                "wait_for_render"
            ]
            assert camera_wait["default"] is True and lidar_wait["default"] is False
            assert (
                "NOT_READY" in camera_wait["description"]
                and "tool acquisition" in camera_wait["description"]
            )
            assert "completed scan" in lidar_wait["description"]
            # clear was typed list[float], so no [x, y, r] circle got through.
            clear = listed["sim__occupancy_map"].inputSchema["properties"]["clear"]
            assert clear["maxItems"] == 16
            assert clear["items"]["type"] == "array" and clear["items"]["items"]["type"] == "number"
            assert clear["items"]["minItems"] == clear["items"]["maxItems"] == 3
            roots = await c.call_tool("assets__roots", {})
            assert (
                not roots.isError and json.loads(roots.content[0].text)["roots"] == []
            )
            scan = await c.call_tool("assets__scan", {"root_id": "not-approved"})
            assert scan.isError and "ROOT_NOT_APPROVED" in scan.content[0].text
            info = await c.call_tool("sim__stage__info", {})
            assert info.isError and "not available" in info.content[0].text

    asyncio.run(run())


def test_brokers_are_per_server(tmp_path):
    root = tmp_path / "root"
    root.mkdir()
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    config = tmp_path / "cfg"
    config.write_text(
        json.dumps(
            dict(
                version=1,
                profile="trusted_project",
                roots={"one": str(root)},
                staging=str(staging),
            )
        )
    )
    a = gw.build_server({"TWINSMITH_ASSET_CONFIG": str(config)})
    b = gw.build_server({})
    assert list(gw._state_for(a).asset_broker.roots) == ["one"]
    assert not gw._state_for(b).asset_broker.roots


def test_generic_filters(monkeypatch):
    server = gw.build_server({"TWINSMITH_TOOLSETS": "default,+sim"})
    names = server._tool_manager._tools
    assert "sim__stage__info" in names and "assets__roots" not in names
    assert any(x.startswith("workcell__") for x in names) and any(x.startswith("isaaclab__") for x in names)


def test_unavailable_profile_does_not_start_trusted_server(tmp_path):
    cfg = tmp_path / "cfg.json"
    cfg.write_text('{"version":1,"profile":"isolated_untrusted"}')
    with pytest.raises(RuntimeError, match="No restricted endpoint"):
        gw.build_server({"TWINSMITH_ASSET_CONFIG": str(cfg)})


def test_scene_opt_out_is_per_server_offline_and_after_live_refresh(monkeypatch):
    from twinsmith.client import BridgeClient
    from bridge_testing import LiveBridge, td

    hidden = gw.build_server({"TWINSMITH_TOOLSETS": "default,+cad"})
    visible = gw.build_server({"TWINSMITH_TOOLSETS": "default,+sim,+assets"})
    # The bridge built-ins are listed offline too, so offline == with a bridge.
    assert "run_python" in visible._tool_manager._tools and "status" in hidden._tool_manager._tools
    assert "run_python" not in hidden._tool_manager._tools
    assert "sim__stage__info" in visible._tool_manager._tools
    assert "sim__stage__info" not in hidden._tool_manager._tools
    assert sum(n.startswith("workcell__") for n in visible._tool_manager._tools) == 12  # 8 members and their 4 groups
    assert not any(n.startswith("workcell__") for n in hidden._tool_manager._tools)
    with LiveBridge() as bridge:
        owner = bridge.bridge.register_owner("generic-sim", "sim")
        owner.add(td("stage.info", lambda: {"selected": True}))
        gw._state_for(hidden).client = BridgeClient(port=bridge.port)
        gw._state_for(visible).client = BridgeClient(port=bridge.port)

        async def run():
            for server, enabled in ((hidden, False), (visible, True)):
                async with create_connected_server_and_client_session(
                    server._mcp_server
                ) as client:
                    await client.initialize()
                    await client.call_tool("refresh_tools", {})
                    names = {t.name for t in (await client.list_tools()).tools}
                    assert ("sim__stage__info" in names) == enabled
                    assert ("assets__roots" in names) == enabled
                    assert ("run_python" in names) == enabled and ("workcell__workcell_status" in names) == enabled
                    if not enabled:
                        refused = await client.call_tool("sim__stage__info", {})
                        assert refused.isError

        asyncio.run(run())


def test_minimal_toolsets_keep_managed_sessions():
    server = gw.build_server(
        {"TWINSMITH_TOOLSETS": "default,+computer"}
    )
    assert "launch_simulator" in server._tool_manager._tools
    assert "sim__stage__info" not in server._tool_manager._tools


def test_asset_stale_target_includes_current_identity_before_snapshot(monkeypatch):
    server = gw.build_server({})
    state = gw._state_for(server)
    info = dict(
        session_id="current-session",
        stage_id="current-stage",
        pid=123,
        source="anon:current-stage",
        default_prim="/World",
        meters_per_unit=1,
        up_axis="Z",
    )

    class Client:
        def call(self, name):
            assert name == "sim.stage.info"
            return info

    state.client = Client()
    monkeypatch.setattr(
        state.asset_broker,
        "snapshot",
        lambda *a, **kw: pytest.fail("stale call copied source files"),
    )
    out = json.loads(
        gw._dispatch("assets.import_path", dict(session_id="old", stage_id="old"), state)
    )
    assert out["code"] == "STALE_TARGET"
    assert out["current_target"] == info


@pytest.mark.parametrize("tool", ["assets.import_path"])
def test_absolute_path_is_snapshotted_before_bridge_dispatch(tmp_path, tool):
    root = tmp_path / "root"
    root.mkdir()
    source = root / "asset.usda"
    source.write_text("#usda 1.0\n")
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            dict(
                version=1,
                profile="trusted_project",
                roots={"project": str(root)},
                staging=str(staging),
            )
        )
    )
    server = gw.build_server({"TWINSMITH_ASSET_CONFIG": str(config)})
    state = gw._state_for(server)
    seen = []

    class Client:
        def call(self, name, params=None):
            if name == "sim.stage.info":
                return dict(
                    session_id="session", stage_id="stage", staging=str(staging)
                )
            seen.append((name, params))
            filename, _ = state.asset_broker.verify(
                params["snapshot_id"], params["manifest_sha256"]
            )
            assert filename.read_text() == source.read_text()
            assert "source_path" not in params
            return dict(status="ok", prepared=True)

    state.client = Client()
    params = dict(session_id="session", stage_id="stage", source_path=str(source))
    if tool == "assets.import_path":
        params["prim_path"] = "/World/Imported"
    out = json.loads(gw._dispatch(tool, params, state))
    assert out["status"] == "ok" and seen[0][0] == tool
    denied = json.loads(
        gw._dispatch(
            tool, {**params, "source_path": str(tmp_path / "other.usda")}, state
        )
    )
    assert denied["code"] == "ROOT_NOT_APPROVED" and len(seen) == 1


def _urdf_state(tmp_path):
    root = tmp_path / "root"
    (root / "urdf").mkdir(parents=True)
    source = root / "urdf" / "r.urdf"
    source.write_text('<robot name="r"><link name="a"/></robot>')
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    config = tmp_path / "config.json"
    config.write_text(json.dumps(dict(version=1, profile="trusted_project", roots={"project": str(root)}, staging=str(staging))))
    state = gw._state_for(gw.build_server({"TWINSMITH_ASSET_CONFIG": str(config)}))
    seen = []

    class Client:
        def call(self, name, params=None):
            if name == "sim.stage.info":
                return dict(session_id="session", stage_id="stage", staging=str(staging))
            filename, manifest = state.asset_broker.verify(params["snapshot_id"], params["manifest_sha256"], kind="urdf")
            seen.append((name, params, manifest))
            assert filename.read_text() == source.read_text() and "source_path" not in params
            return dict(status="ok")

    state.client = Client()
    return state, source, seen


def test_a_urdf_by_source_path_is_snapshotted_before_dispatch_and_goes_to_one_tool(tmp_path):
    state, source, seen = _urdf_state(tmp_path)
    ids = dict(session_id="session", stage_id="stage", prim_path="/World/R")
    out = json.loads(gw._dispatch("assets.import_path", {**ids, "source_path": str(source)}, state))
    assert out["status"] == "ok" and out["import"]["format"] == "urdf"
    by_root = json.loads(gw._dispatch("assets.import_path", {**ids, "root_id": "project", "relative_path": "urdf/r.urdf"}, state))
    assert by_root["status"] == "ok"
    assert [item[0] for item in seen] == ["assets.import_path"] * 2 and seen[0][1]["format"] == "urdf"
    assert [item[2]["entry"] for item in seen] == ["urdf/r.urdf", "urdf/r.urdf"]
    assert seen[0][2]["root_name"] == "root"


@pytest.mark.parametrize("extra,code", [
    (dict(), "INVALID_VALUE"),
    (dict(root_id="project"), "INVALID_VALUE"),
    (dict(source_path="/x/y.urdf", root_id="project", relative_path="urdf/r.urdf"), "INVALID_VALUE"),
    (dict(root_id="nope", relative_path="r.urdf"), "ROOT_NOT_APPROVED"),
    (dict(source_path="/not/approved.urdf"), "ROOT_NOT_APPROVED"),
    (dict(source_path="relative/r.urdf"), "INVALID_PATH"),
])
def test_the_source_must_be_one_approved_absolute_path(tmp_path, extra, code):
    state, _, seen = _urdf_state(tmp_path)
    out = json.loads(gw._dispatch("assets.import_path", dict(session_id="session", stage_id="stage", prim_path="/World/R", **extra), state))
    assert out["code"] == code and not seen


def _urdf_pack(store, files):
    """A pack bot/v1 in the asset root that verifies against its pack.yaml."""
    import hashlib

    directory = store / "bot" / "v1"
    records = []
    for rel, text in sorted(files.items()):
        (directory / rel).parent.mkdir(parents=True, exist_ok=True)
        (directory / rel).write_text(text)
        records.append(dict(path=rel, sha256=hashlib.sha256(text.encode()).hexdigest(), source="t", license="t", redistributable=False))
    (directory / "pack.yaml").write_text(json.dumps(dict(id="bot", version="v1", files=records)))


def test_quick_start_sends_a_pack_urdf_through_the_host(tmp_path, monkeypatch):
    store = tmp_path / "store"
    _urdf_pack(store, {"r.urdf": '<robot name="r"><link name="a"/></robot>'})
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(store))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(tmp_path / "none"))
    state, _, seen = _urdf_state(tmp_path)
    assert json.loads(gw._dispatch("sim.quick_start", dict(robot="bot/v1/r.urdf"), state))["status"] == "ok"
    assert seen[0][0] == "sim.quick_start" and seen[0][1]["robot"] == "bot/v1/r.urdf" and seen[0][2]["root_name"] == "v1"
    out = json.loads(gw._dispatch("sim.quick_start", dict(robot="bot/v1/none.urdf"), state))
    assert out["code"] == "ASSET_MISSING" and len(seen) == 1  # not listed in pack.yaml
    out = json.loads(gw._dispatch("sim.quick_start", dict(robot="bot/r.urdf"), state))  # a version is required
    assert out["code"] == "INVALID_VALUE" and len(seen) == 1
    out = json.loads(gw._dispatch("sim.quick_start", dict(robot="bad..name/v1/r.urdf"), state))
    assert out["code"] in ("INVALID_VALUE", "ASSET_MISSING") and len(seen) == 1
    (store / "bot/v1/r.urdf").write_text('<robot name="changed"/>')  # no longer what pack.yaml says
    out = json.loads(gw._dispatch("sim.quick_start", dict(robot="bot/v1/r.urdf"), state))
    assert out["code"] == "ASSET_MISSING" and "Checksum mismatch" in out["message"] and len(seen) == 1


def test_quick_start_refuses_a_pack_xacro_with_the_guidance(tmp_path, monkeypatch):
    store = tmp_path / "store"
    _urdf_pack(store, {"r.urdf.xacro": "<robot/>"})
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(store))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(tmp_path / "none"))
    state, _, seen = _urdf_state(tmp_path)
    out = json.loads(gw._dispatch("sim.quick_start", dict(robot="bot/v1/r.urdf.xacro"), state))
    assert out["code"] == "XACRO" and not seen


def test_a_newer_bridge_parameter_does_not_break_discovery():
    # The two checkouts sync separately: a bridge that already serves sim.ui.act's `window` parameter
    # met an older host registry, and discovery failed for every tool ("invalid bridge tool").
    fields = [dict(name="action", kind="string", enum=None, required=True, default=None, has_default=False,
                   description="What to do."),
              dict(name="brand_new", kind="string", enum=None, required=False, default=None, has_default=False,
                   description="A parameter this host does not know yet.")]
    fields.append(dict(name="titles", kind="array", enum=None, required=False, default=None, has_default=False,
                       description="An unknown list of strings."))
    fields.append(dict(name="future", kind="hyperthing", enum=None, required=False, default=None,
                       has_default=False, description="An unknown kind."))
    fn = gw._build_tool_function("sim__ui__act", "sim.ui.act", "d", fields)
    assert {"brand_new", "titles", "future"} <= set(fn.__annotations__)
    import pydantic

    # Unknown arrays keep the bridge's plain list type: strings are not rejected as non-numbers.
    pydantic.TypeAdapter(fn.__annotations__["titles"]).validate_python(["Console"])
    pydantic.TypeAdapter(fn.__annotations__["brand_new"]).validate_python("x")


def test_scene_tools_refuse_unknown_arguments():
    # properties={...} was silently dropped by sim__graph__build and nothing was applied.
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            bad = await client.call_tool("sim__graph__build", {"session_id": "s", "stage_id": "t", "path": "/G",
                                                               "nodes": [["tick", TICK]], "properties": {"a": 1}})
            assert bad.isError and "properties" in bad.content[0].text and "xtra" in bad.content[0].text
            ok = await client.call_tool("sim__graph__build", {"session_id": "s", "stage_id": "t", "path": "/G",
                                                              "nodes": [{"name": "tick", "type": TICK}]})
            assert "xtra" not in ok.content[0].text   # reaches dispatch (fails offline, not on validation)
            listed = {t.name: t for t in (await client.list_tools()).tools}
            assert listed["sim__graph__build"].inputSchema.get("additionalProperties") is False

    asyncio.run(run())


def test_serialized_graph_payload_reaches_parser_through_mcp():
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})
    seen = []

    class Bridge:
        def call(self, name, args):
            assert name == "sim.graph.build"
            try:
                seen.append(gb.parse_spec(args))
                return {"status": "ok"}
            except gb.SpecError as exc:
                return {"status": "error", "code": exc.code, "message": str(exc)}
        def close(self):
            pass

    gw._state_for(server).client = Bridge()

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            args = {"session_id": "s", "stage_id": "t", "path": "/G",
                    "nodes": ['["tick", "omni.graph.action.OnPlaybackTick"]',
                              '["clock", "isaacsim.ros2.bridge.ROS2PublishClock"]'],
                    "connections": ['["tick.outputs:tick", "clock.inputs:execIn"]'],
                    "values": '{"clock.inputs:topicName": "/clock", "clock.inputs:queueSize": 10}'}
            result = await client.call_tool("sim__graph__build", args)
            assert not result.isError
            assert seen[-1]["nodes"] == [("tick", TICK), ("clock", PUB)]
            assert ("clock", "inputs:queueSize", 10) in seen[-1]["values"]
            args["nodes"] = '[invalid'
            bad = await client.call_tool("sim__graph__build", args)
            assert bad.isError and "nodes: expected" in bad.content[0].text
            assert "validation errors" not in bad.content[0].text

    asyncio.run(run())


def test_graph_schema_and_serialization_errors_at_mcp_boundary():
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})
    seen = []

    class Bridge:
        def call(self, name, args):
            assert name == "sim.graph.build"
            seen.append(args)
            try:
                gb.parse_spec(args)
                return {"status": "ok"}
            except gb.SpecError as exc:
                return {"status": "error", "code": exc.code, "message": str(exc)}
        def close(self):
            pass

    gw._state_for(server).client = Bridge()

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            listed = {t.name: t for t in (await client.list_tools()).tools}
            props = listed["sim__graph__build"].inputSchema["properties"]
            for field, expected in (("nodes", "array"), ("connections", "array"),
                                    ("values", "object"), ("targets", "object")):
                schema = props[field]
                assert schema.get("description")
                assert {branch.get("type") for branch in schema["anyOf"]} >= {"string", expected}
                assert all(branch.get("type") for branch in schema["anyOf"])
            valid = {"session_id": "s", "stage_id": "t", "path": "/G",
                     "nodes": [["clock", PUB]], "connections": [],
                     "values": {"clock.inputs:topicName": "/clock"},
                     "targets": {"clock.inputs:targetPrim": ["/World/Camera"]}}
            result = await client.call_tool("sim__graph__build", valid)
            assert not result.isError and seen[-1]["nodes"] == valid["nodes"]
            deep = "[" * 2000 + "]" * 2000
            cases = [
                ("nodes", deep),
                ("nodes", [deep]),
                ("connections", deep),
                ("values", '{"clock.inputs:value": NaN}'),
                ("targets", {"clock.inputs:targetPrim": deep}),
            ]
            for field, value in cases:
                args = dict(valid, **{field: value})
                before = len(seen)
                result = await client.call_tool("sim__graph__build", args)
                assert len(seen) == before + 1  # bridge parser, not FastMCP pre-parsing
                assert result.isError and field in result.content[0].text
                assert "expected" in result.content[0].text
                assert "recursion" not in result.content[0].text.lower()
                assert len(result.content[0].text) < 500

    asyncio.run(run())


def test_body_path_aliases_reach_the_bridge():
    server = gw.build_server({"TWINSMITH_TOOLSETS": "all"})
    seen = []
    class Bridge:
        def call(self, name, args):
            seen.append((name, args))
            return {"status": "ok"}
        def close(self):
            pass
    gw._state_for(server).client = Bridge()

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            for key in ("path", "prim", "prim_path"):
                result = await client.call_tool("sim__body__state", {"session_id": "s", "stage_id": "t", key: "/Robot"})
                assert not result.isError and seen[-1][1][key] == "/Robot"
    asyncio.run(run())
