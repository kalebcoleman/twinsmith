"""Toolset ownership, configuration precedence and registration boundaries."""

import asyncio
import json

import pytest
from mcp.server.fastmcp import FastMCP

from twinsmith import config, host_catalog, local_tools, server, tool_groups, toolsets


def build(selection="default"):
    return server.build_server({"TWINSMITH_TOOLSETS": selection})


def test_registry_owns_every_registered_tool():
    instance = build("all,+computer")
    for name, tool in instance._tool_manager._tools.items():
        assert toolsets.owner(name) in toolsets.TOOLSETS
        assert tool.annotations is not None
    for name, meta in host_catalog.load_catalog().items():
        assert toolsets.owner(name, meta["namespace"]) in toolsets.TOOLSETS
    assert sum(map(len, toolsets.MEMBERS.values())) == len(set().union(*toolsets.MEMBERS.values()))


@pytest.mark.parametrize("name", toolsets.TOOLSETS)
def test_each_toolset_alone_and_disabled_registers_nothing(name):
    instance = build("default,+" + name)
    assert all(toolsets.owner(tool) in {"core", name} for tool in instance._tool_manager._tools)
    if name != "core":
        disabled = build("all,+computer,-" + name)
        assert all(toolsets.owner(tool) != name for tool in disabled._tool_manager._tools)


def test_real2sim_is_off_by_default_and_registers_robosim_and_sysid():
    assert "real2sim" not in toolsets.DEFAULT and "real2sim" in config.select("all")
    assert not {"robosim", "sysid"} & set(build()._tool_manager._tools)
    alone = build("default,+real2sim")
    assert {name for name in alone._tool_manager._tools if toolsets.owner(name) == "real2sim"} == {"robosim", "sysid"}
    assert toolsets.MEMBERS["real2sim"] == {"robosim", "sysid"}


def test_only_core_is_on_by_default_and_status_lists_no_planned_toolsets():
    assert toolsets.DEFAULT == {"core"} and config.select("default") == {"core"}
    assert [name for name, spec in toolsets.TOOLSETS.items() if spec.default] == ["core"]
    assert set(toolsets.PROFILES) == {"default", "all"}
    assert config.select("all") == set(toolsets.TOOLSETS) - {"computer"}
    assert "planned" not in toolsets.describe(toolsets.DEFAULT)
    assert not hasattr(toolsets, "PLANNED")
    instance = build("default")
    assert set(server.grouped_tools.listed_names(instance)) == {"status", "help", "refresh_tools", "use_bridge", "jobs",
                                                                "toolsets"}
    assert {toolsets.owner(name) for name in instance._tool_manager._tools} == {"core"}


# What each toolset lists, on top of core: the mapping in docs/ARCHITECTURE.md, written out.
LISTED = {
    "core": {"status", "help", "refresh_tools", "use_bridge", "jobs", "toolsets"},
    "computer": {"screenshot", "screen_recording", "windows", "join_videos", "doctor", "computer", "simulators"},
    "sim": {"sim__quick_start", "sim__stage", "sim__prim", "sim__timeline", "sim__light", "sim__material",
            "sim__physics", "sim__joints", "sim__arm", "sim__simulation", "sim__camera", "sim__lidar",
            "sim__viewport", "run_python", "files", "reload_tools", "sim__controller", "sim__ui", "sim__logs",
            "sim__ros2_tf", "sim__graph", "sim__occupancy_map",
            "workcell__cell", "workcell__robot", "workcell__perception", "workcell__jenga",
            "isaaclab__launch", "isaaclab__inspect", "isaaclab__metrics", "isaaclab__checkpoint_register", "run_script"},
    "assets": {"assets__import_path", "assets__library", "assets__mesh", "assets__pack", "assets__validate"},
    "real2sim": {"robosim", "sysid"},
    "cad": {"hardware"},
    "planning": {"gap"},
    "eval": {"vabar"},
}


@pytest.mark.parametrize("name", toolsets.TOOLSETS)
def test_each_toolset_lists_exactly_its_mapped_tools(name):
    assert set(LISTED) == set(toolsets.TOOLSETS)
    listed = set(server.grouped_tools.listed_names(build("default,+" + name)))
    assert listed == LISTED["core"] | LISTED[name]


@pytest.mark.parametrize("value", ["scene", "physics", "sensors", "code", "kit_debug", "robosim", "minimal",
                                   "default,+scene", "minimal,+lab", "screen", "input", "launch", "lab", "ros2",
                                   "workcell", "extensions", "default,+sim,+screen", "all,+input"])
def test_old_toolset_names_stop_the_server_with_the_valid_names(value):
    with pytest.raises(ValueError, match="Unknown toolset") as error:
        config.select(value)
    assert "Valid names: default, all, core, computer, sim, assets, cad, real2sim, planning, eval" in str(error.value)
    with pytest.raises(ValueError, match="Valid names"):
        server.build_server({"TWINSMITH_TOOLSETS": value})


def test_no_workcell_stub_is_registered():
    names = set(build("all,+computer")._tool_manager._tools) | set(tool_groups.GROUPS)
    names |= {name.replace(".", "__") for name in host_catalog.load_catalog()}
    assert not [name for name in names if name.startswith("workcell__")
                and any(word in name for word in ("real2sim", "planning", "eval"))]


def test_profiles_and_guards():
    assert config.select("default") == toolsets.DEFAULT
    assert config.select("all").isdisjoint({"computer"})
    assert config.select("default,sim,+computer,-computer") == {"core", "sim"}
    with pytest.raises(ValueError, match="cannot be turned off"):
        config.select("-core")
    for value in ("oops", "default,+backend", "--sim", ""):
        with pytest.raises(ValueError, match="Valid names"):
            config.select(value)


def test_five_source_precedence_and_backend(tmp_path):
    home, project = tmp_path / "home", tmp_path / "project"
    user = home / ".config/twinsmith/config.toml"
    user.parent.mkdir(parents=True)
    child = project / "nested"
    child.mkdir(parents=True)
    def resolve(env=None, cli=None):
        return config.resolve(env or {}, cli, cwd=child, home=home)
    assert resolve().toolsets == toolsets.DEFAULT
    user.write_text('[tools]\ntoolsets = ["default", "+cad"]\n[sim]\nphysics_backend = "newton"\n')
    assert resolve().toolsets == {"core", "cad"}
    assert resolve().physics_backend == "newton"
    (project / "twinsmith.toml").write_text('[tools]\ntoolsets = ["default", "+computer"]\n')
    assert resolve().toolsets == {"core", "computer"}
    assert resolve({"TWINSMITH_TOOLSETS": "default,+assets"}).toolsets == {"core", "assets"}
    assert resolve({"TWINSMITH_TOOLSETS": "default,+assets"}, "default,+sim").toolsets == {"core", "sim"}
    assert resolve({"TWINSMITH_PHYSICS_BACKEND": "physx"}).physics_backend == "physx"
    with pytest.raises(ValueError, match="physics_backend"):
        resolve({"TWINSMITH_PHYSICS_BACKEND": "invalid"})


def test_registration_requires_ownership_and_annotations():
    scoped = local_tools.ScopedTools(FastMCP("guard"))
    with pytest.raises(local_tools.LocalToolsError, match="no toolset"):
        scoped.tool(annotations=local_tools.READ_ONLY)(lambda: None)
    with pytest.raises(local_tools.LocalToolsError, match="no ToolAnnotations"):
        scoped.tool(name="status")(lambda: None)


def test_live_visibility_uses_same_selection():
    instance = build("default,+assets")
    state = server._state_for(instance)
    assert not server._visible("demo.ping", {"namespace": "demo"}, state)
    assert not server._visible("sim.stage.info", {"namespace": "sim"}, state)
    assert not server._visible("workcell.future_tool", {"namespace": "workcell"}, state)
    state = server._state_for(build("default,+sim"))
    assert server._visible("demo.ping", {"namespace": "demo"}, state)          # an uncatalogued Kit extension
    assert server._visible("workcell.future_tool", {"namespace": "workcell"}, state)
    assert toolsets.owner("workcell.future_tool", "workcell") == "sim"
    assert toolsets.owner("isaaclab__future_tool") == "sim"
    with pytest.raises(ValueError, match="no toolset"):
        toolsets.owner("sim.unknown", "sim")


# Estimated tokens of each toolset on top of core, and of all: the measured size plus about 5 percent. Measured
# 2026-11-13 with scripts/measure_tool_list.py: default (core) 1,586; computer 4,807; sim 17,876 (with ROS 2, the
# work cell and Isaac Lab); assets 4,283; cad 2,018; real2sim 3,050; planning 1,981; eval 2,110; all 23,387;
# all,+computer 26,607.
BUDGETS = {"default": 1670, "default,+computer": 5050, "default,+sim": 18770, "default,+assets": 4500,
           "default,+cad": 2120, "default,+real2sim": 3210, "default,+planning": 2080, "default,+eval": 2220,
           "all": 24560, "all,+computer": 27940}


@pytest.mark.parametrize("profile,budget", BUDGETS.items())
def test_listing_size(profile, budget):
    instance = build(profile)
    tools = asyncio.run(instance.list_tools())
    payload = json.dumps([{key: value for key, value in tool.model_dump(mode="json").items()
                           if key in ("name", "description", "inputSchema", "annotations")} for tool in tools],
                         separators=(",", ":"))
    print(f"\n{profile}: {len(tools)} tools, {len(payload)} characters, {len(payload) // 4} estimated tokens")
    assert len(payload) // 4 < budget


@pytest.mark.parametrize("enabled", [True, False])
def test_capture_directories_follow_computer_toolset(tmp_path, monkeypatch, enabled):
    from twinsmith import screen
    shots, videos = tmp_path / "shots", tmp_path / "videos"
    monkeypatch.setattr(screen, "SHOT_DIR", shots)
    monkeypatch.setattr(screen, "VIDEO_DIR", videos)
    build("default,+computer" if enabled else "default")
    assert shots.exists() is enabled
    assert videos.exists() is enabled


def test_cli_table_and_override(monkeypatch, capsys):
    monkeypatch.setenv("TWINSMITH_TOOLSETS", "all")
    server.main(["--toolsets", "default,+cad", "toolsets"])
    output = capsys.readouterr().out
    assert "Toolset" in output
    for name in toolsets.TOOLSETS:
        row = next(row for row in output.splitlines() if row.startswith(name + " "))
        assert row.split()[1] == ("on" if name in {"core", "cad"} else "off")
        assert "+" + name in row


@pytest.mark.parametrize("selection", ["default", "default,+sim", "all,+computer"])
def test_status_resource_and_help_share_selection(selection, monkeypatch):
    instance = build(selection)
    state = server._state_for(instance)
    monkeypatch.setattr(state, "get_client", lambda: (_ for _ in ()).throw(ConnectionRefusedError("Connection refused")))
    monkeypatch.setattr(server, "_surroundings", lambda state: {"bridges": []})
    expected = toolsets.describe(state.toolsets)
    assert server._where_am_i({}, state)["toolsets"] == expected
    contents = asyncio.run(instance.read_resource("twinsmith://toolsets"))
    assert json.loads(list(contents)[0].content) == expected
    help_tool = instance._tool_manager._tools["help"]
    answer = help_tool.fn(task="train a policy")
    if "sim" in state.toolsets:
        assert "isaaclab__train" in answer
    else:
        assert "Toolsets off:" in answer and "sim" in answer
        assert "TWINSMITH_TOOLSETS" in answer
        assert "isaaclab__train" not in answer


def test_help_respects_disabled_code_and_mixed_topics(monkeypatch):
    monkeypatch.setattr(server, "_record_help_gap", lambda *args: None)
    answer = server._help("simulate cloth tearing", enabled={"core"})
    assert "run_python" not in answer
    assert "Toolsets off: sim" in answer
    answer = server._help("where am i", enabled={"core"})
    assert "Available: status" in answer
    assert "launch_simulator" not in answer
    assert "computer" in answer


def test_instructions_are_short_and_selection_neutral():
    assert 450 < len(server.SERVER_INSTRUCTIONS) < 700
    assert "status" in server.SERVER_INSTRUCTIONS and "toolsets" in server.SERVER_INSTRUCTIONS
    assert "isaaclab__" not in server.SERVER_INSTRUCTIONS


def test_sim_owns_the_whole_workcell_and_isaaclab_namespaces():
    owned = {name for name in build("default,+sim")._tool_manager._tools if name.startswith(("workcell__", "isaaclab__"))}
    jobs = set(tool_groups.GROUPS["jobs"][1].values())              # isaaclab__runs_* are actions of core's jobs
    assert all(toolsets.owner(name) == ("core" if name in jobs else "sim") for name in owned)
    assert {"workcell__jenga", "workcell__jenga_build", "workcell__cell", "isaaclab__launch", "isaaclab__train"} <= owned
    assert {name for name in build("default,+assets")._tool_manager._tools if name.startswith(("workcell__", "isaaclab__"))} <= jobs


# ---- enabling and disabling toolsets at run time ----

RUNTIME = [name for name, spec in toolsets.TOOLSETS.items() if name != "core" and not spec.explicit_only]


def offline_build(selection="default", extra=None):
    """A server with no bridge to find, so run-time changes use only host tools and the catalog."""
    instance = server.build_server({"TWINSMITH_TOOLSETS": selection, **(extra or {})})
    state = server._state_for(instance)
    state.get_client = lambda: (_ for _ in ()).throw(ConnectionRefusedError("Connection refused"))
    return instance, state


def change(instance, action, names=None, ctx=None):
    return asyncio.run(server._change_toolsets(instance, action, names, ctx))


def registered(instance):
    return set(instance._tool_manager._tools)


class FakeSession:
    def __init__(self):
        self.sent = 0

    async def send_tool_list_changed(self):
        self.sent += 1


class FakeContext:
    def __init__(self):
        self.session = FakeSession()
        self.request_context = self


@pytest.mark.parametrize("name", RUNTIME)
def test_enable_registers_exactly_what_startup_would(name):
    instance, _ = offline_build("default")
    before = set(server.grouped_tools.listed_names(instance))
    result = change(instance, "enable", [name])
    expected = build("default,+" + name)
    assert registered(instance) == registered(expected)
    assert set(server.grouped_tools.listed_names(instance)) == set(server.grouped_tools.listed_names(expected))
    assert set(result["tools_available"]) == set(server.grouped_tools.listed_names(instance)) - before
    assert result["changed"] == [name] and name in result["on"]
    assert "nothing was written" in result["scope"]
    assert "loads tools lazily, search for the tool by name (for example robosim) before calling it" in result["note"]


@pytest.mark.parametrize("name", RUNTIME)
def test_disable_removes_exactly_that_toolsets_tools(name):
    instance, _ = offline_build("all")
    result = change(instance, "disable", [name])
    assert registered(instance) == registered(build("all,-" + name))
    assert result["tools_removed"] and name not in result["on"]
    assert not any(toolsets.owner(tool) == name for tool in registered(instance) if tool not in tool_groups.GROUPS)


def test_enabled_tools_are_callable_and_disabled_ones_are_not():
    instance, _ = offline_build("default")
    with pytest.raises(Exception):
        asyncio.run(instance.call_tool("sim__prim", {"action": "help"}))
    change(instance, "enable", ["sim"])
    answer = asyncio.run(instance.call_tool("sim__prim", {"action": "help"}))
    assert "sim__prim__get" in str(answer) or "get" in str(answer)
    change(instance, "disable", ["sim"])
    with pytest.raises(Exception):
        asyncio.run(instance.call_tool("sim__prim", {"action": "help"}))


def test_enable_twice_is_harmless():
    instance, _ = offline_build("default")
    change(instance, "enable", ["sim", "cad"])
    once = registered(instance)
    listed = set(server.grouped_tools.listed_names(instance))
    again = change(instance, "enable", ["sim", "cad"])
    assert again["changed"] == [] and again["unchanged"] == ["sim", "cad"]
    assert "tools_available" in again and again["tools_available"] == []
    assert registered(instance) == once and set(server.grouped_tools.listed_names(instance)) == listed
    change(instance, "disable", ["sim"])
    change(instance, "enable", ["sim"])
    assert registered(instance) == once


@pytest.mark.parametrize("name", ["computer"])
def test_explicit_only_toolsets_can_never_be_enabled_at_run_time(name):
    from mcp.server.fastmcp.exceptions import ToolError
    instance, state = offline_build("default")
    before = registered(instance)
    with pytest.raises(ToolError) as error:
        change(instance, "enable", ["sim", name])
    assert "explicit-only" in str(error.value) and "TWINSMITH_TOOLSETS" in str(error.value)
    assert "restart" in str(error.value) and "Nothing changed" in str(error.value)
    assert registered(instance) == before and state.toolsets == {"core"}      # refused whole, "sim" too
    assert name not in server._where_am_i({}, state)["toolsets"]["can_enable"]
    assert name in server._where_am_i({}, state)["toolsets"]["off"]


def test_explicit_only_ones_stay_available_at_startup():
    instance, state = offline_build("default,+computer")
    assert {"computer", "computer_click", "simulators", "launch_simulator", "screenshot"} <= registered(instance)
    change(instance, "disable", ["computer"])          # a user's own startup choice can be dropped again
    assert not {"computer", "simulators", "screenshot"} & registered(instance)
    from mcp.server.fastmcp.exceptions import ToolError
    with pytest.raises(ToolError, match="explicit-only"):
        change(instance, "enable", ["computer"])


def test_core_cannot_be_disabled_and_unknown_names_are_refused():
    from mcp.server.fastmcp.exceptions import ToolError
    instance, state = offline_build("default,+sim")
    before = registered(instance)
    for action, names, text in (("disable", ["sim", "core"], "core cannot be disabled"),
                                ("enable", ["nonsense"], "Unknown toolset"),
                                ("enable", [], "needs names"), ("enable", None, "needs names"),
                                ("disable", ["lab"], "Unknown toolset")):
        with pytest.raises(ToolError, match=text):
            change(instance, action, names)
    assert registered(instance) == before and "sim" in state.toolsets and "toolsets" in registered(instance)


@pytest.mark.parametrize("how", ["env", "env_false", "user_file", "project_file"])
def test_runtime_enable_switch_turns_the_feature_off(how, tmp_path, monkeypatch):
    from mcp.server.fastmcp.exceptions import ToolError
    home = tmp_path / "home"
    (home / ".config/twinsmith").mkdir(parents=True)
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.chdir(tmp_path)
    extra = {}
    if how == "env":
        extra = {"TWINSMITH_RUNTIME_ENABLE": "0"}
    elif how == "env_false":
        extra = {"TWINSMITH_RUNTIME_ENABLE": "false"}
    elif how == "user_file":
        (home / ".config/twinsmith/config.toml").write_text("[tools]\nruntime_enable = false\n")
    else:
        (tmp_path / "twinsmith.toml").write_text("[tools]\nruntime_enable = false\n")
    instance, state = offline_build("default", extra)
    assert state.runtime_enable is False
    for action in ("enable", "disable"):
        with pytest.raises(ToolError, match="switched off"):
            change(instance, action, ["sim"])
    assert registered(instance) == registered(build("default"))
    listing = change(instance, "list")                       # reading is always allowed
    assert listing["runtime_enable"] is False and listing["can_enable"] == []
    assert "off" in listing["enable"].lower() and "startup" in listing["enable"] or "TWINSMITH_TOOLSETS" in listing["enable"]


def test_runtime_enable_default_and_bad_value():
    assert config.resolve({}).runtime_enable is True
    assert config.resolve({"TWINSMITH_RUNTIME_ENABLE": "1"}).runtime_enable is True
    with pytest.raises(ValueError, match="TWINSMITH_RUNTIME_ENABLE"):
        config.resolve({"TWINSMITH_RUNTIME_ENABLE": "maybe"})


def test_a_tool_list_change_is_announced_once_and_only_when_something_changed():
    instance, _ = offline_build("default")
    ctx = FakeContext()
    change(instance, "enable", ["cad"], ctx)
    assert ctx.session.sent == 1
    change(instance, "enable", ["cad"], ctx)                  # nothing changed: no notification
    change(instance, "list", None, ctx)
    assert ctx.session.sent == 1
    change(instance, "disable", ["cad"], ctx)
    assert ctx.session.sent == 2


def test_server_advertises_list_changed_for_the_tool():
    instance = build("default")
    options = instance._mcp_server.create_initialization_options()
    assert options.capabilities.tools.listChanged is True
    tool = instance._tool_manager._tools["toolsets"]
    assert tool.parameters["properties"]["action"]["enum"] == ["list", "enable", "disable"]
    assert toolsets.owner("toolsets") == "core"


def test_workcell_and_real2sim_register_without_a_bridge_or_checkout(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("TWINSMITH_ROBOSIM_ROOT", str(tmp_path / "no-checkout"))
    monkeypatch.delenv("TWINSMITH_SYSID_ROOT", raising=False)
    instance, _ = offline_build("default")
    result = change(instance, "enable", ["sim", "real2sim"])
    assert {"robosim", "workcell__cell", "workcell__jenga"} <= set(result["tools_available"])
    assert "dial the selected bridge" in result["note"]
    answer = asyncio.run(instance.call_tool("workcell__cell", {"action": "status"}))
    text = str(answer)
    assert "not available" in text and "no Isaac Sim bridge could be reached" in text
    answer = asyncio.run(instance.call_tool("robosim", {"action": "status"}))
    assert "missing or unreadable" in str(answer) and "never installs" in str(answer)
    answer = asyncio.run(instance.call_tool("sysid", {"action": "status"}))
    assert "ENVIRONMENT_MISSING" in str(answer) and "never installs" in str(answer)


def test_enable_also_reads_the_live_bridge_tools(monkeypatch):
    instance, state = offline_build("default")
    seen = []
    monkeypatch.setattr(server, "discover_and_register_tools", lambda srv: seen.append(sorted(state.toolsets)) or [])
    change(instance, "enable", ["sim"])
    assert seen and "sim" in seen[0]


def test_status_toolsets_field_says_how_to_enable():
    instance, state = offline_build("default")
    field = server._where_am_i({}, state)["toolsets"]
    assert 'toolsets(action="enable", names=[...])' in field["enable"]
    assert "computer" in field["enable"] and "startup" in field["enable"]
    assert field["can_enable"] == [name for name in toolsets.TOOLSETS if name not in state.toolsets
                                   and name not in toolsets.explicit_only()]
    assert field["tools"] == {name: len(tools) for name, tools in LISTED.items()}
    assert "planned" not in field
    contents = asyncio.run(instance.read_resource("twinsmith://toolsets"))
    assert json.loads(list(contents)[0].content) == field
    change(instance, "enable", ["sim"])
    assert "sim" in json.loads(list(asyncio.run(instance.read_resource("twinsmith://toolsets")))[0].content)["on"]


def test_listed_tool_counts_match_what_a_full_server_lists():
    instance = build("all,+computer")
    listed = server.grouped_tools.listed_names(instance)
    for name in toolsets.TOOLSETS:
        assert toolsets.listed_count(name) == sum(toolsets.owner(tool) == name for tool in listed), name


def test_help_names_the_enable_call_for_a_toolset_that_is_off():
    answer = server._help("train a policy")
    assert 'toolsets(action="enable", names=["sim"])' in answer
    switched_off = server._help("train a policy", runtime_enable=False)
    assert 'toolsets(action="enable"' not in switched_off and "switched off" in switched_off
    assert 'names=["computer"]' not in server._help("start a simulator")
    assert "Explicit-only" in server._help("start a simulator")


@pytest.mark.parametrize("bridges", [[], [{"port": 8766, "pid": 1}]])
def test_status_without_a_bridge_says_which_tools_need_none(monkeypatch, bridges):
    class Client:
        def call(self, *args, **kwargs):
            raise ConnectionRefusedError("refused")

    class State:
        toolsets = frozenset()
        runtime_enable = True

        def get_client(self):
            return Client()

    monkeypatch.setattr(server, "_surroundings", lambda state: {"bridges": bridges})
    monkeypatch.setattr(server, "_physics_status", lambda *args, **kwargs: {})
    answer = server._where_am_i({}, State())
    assert answer["running"] is False
    assert "The robosim (real2sim), isaaclab (sim) and computer tools need no bridge." in answer["message"]
    assert ("use_bridge(port=8766)" in answer["message"]) == bool(bridges)
