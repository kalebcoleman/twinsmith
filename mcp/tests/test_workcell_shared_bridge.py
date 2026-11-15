"""The UR5e pack shares a socket without owning other packs or their tools."""

import ast
from pathlib import Path
import runpy

import pytest

from twinsmith_bridge.bridge import McpBridge, ToolDefinition
from twinsmith.client import BridgeClient, BridgeError
from bridge_testing import LiveBridge


ROOT = Path(__file__).resolve().parents[2]
binding = runpy.run_path(str(ROOT / "exts/workcell/workcell/bridge_binding.py"))
attach = binding["attach"]
OWNER = binding["OWNER_ID"]


def workcell_registry():
    tree = ast.parse((ROOT / "exts/workcell/workcell/extension.py").read_text())
    return next(ast.literal_eval(node.value) for node in tree.body
                if isinstance(node, ast.Assign) and any(
                    isinstance(target, ast.Name) and target.id == "MCP_TOOL_REGISTRY"
                    for target in node.targets))


class Cell:
    def __getattr__(self, name):
        if name.startswith("_cmd_"):
            return lambda **kwargs: {"handler": name}
        raise AttributeError(name)


@pytest.fixture
def bridge():
    value = McpBridge(schedule_coroutine=lambda coro: None)
    yield value
    value.stop()


def test_twin_and_another_owner_share_one_socket_and_detach_independently(bridge):
    bridge.register_owner("other", "sim").add(ToolDefinition(
        name="inspect", description="test", parameters={}, handler=lambda: {"ok": True}))
    port = attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=0)
    assert port and bridge.listening_port == port
    tools = bridge.get_registered_tools()
    twin = {name for name in tools if name.startswith("workcell.")}
    assert twin == {f"workcell.{name}" for name in workcell_registry()}
    assert "sim.inspect" in tools

    bridge.unregister_owner(OWNER)
    assert bridge.listening_port == port
    assert "sim.inspect" in bridge.get_registered_tools()
    assert not any(name.startswith("workcell.") for name in bridge.get_registered_tools())

    # Re-register against the existing listener without opening a second socket.
    assert attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=port) == port
    assert "sim.inspect" in bridge.get_registered_tools()
    assert len([name for name in bridge.get_registered_tools() if name.startswith("workcell.")]) == 9


def test_registration_failure_rolls_back_only_the_twin(bridge):
    bridge.register_owner("other", "sim").add(ToolDefinition(
        name="inspect", description="test", parameters={}, handler=lambda: None))
    registry = {"ok": {"description": "ok"}, "missing": {"description": "missing"}}

    class Incomplete:
        def _cmd_ok(self):
            pass

    with pytest.raises(AttributeError):
        attach(bridge, registry, Incomplete(), ToolDefinition, requested_port=0)
    assert OWNER not in bridge.get_owners()
    assert "sim.inspect" in bridge.get_registered_tools()
    assert bridge.listening_port is None


def test_duplicate_owner_is_not_removed_on_failure(bridge):
    bridge.register_owner(OWNER, "workcell")
    with pytest.raises(ValueError, match="already registered"):
        attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=0)
    assert OWNER in bridge.get_owners()


def test_explicit_port_conflict_is_reported_before_registering(bridge):
    bridge.start(0)
    port = bridge.listening_port
    with pytest.raises(RuntimeError, match="conflicts"):
        attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=port + 1)
    assert OWNER not in bridge.get_owners()
    assert bridge.listening_port == port


def test_unbound_explicit_port_conflict_does_not_register(bridge):
    with pytest.raises(RuntimeError, match="conflicts"):
        attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=8766, configured_port=8767)
    assert OWNER not in bridge.get_owners()
    assert bridge.listening_port is None


def test_unset_twin_port_joins_an_existing_listener(bridge):
    bridge.start(0)
    port = bridge.listening_port
    assert attach(bridge, workcell_registry(), Cell(), ToolDefinition) == port


def test_ephemeral_request_rejoins_existing_listener_after_twin_reload(bridge):
    port = attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=0, configured_port=0)
    bridge.unregister_owner(OWNER)
    assert attach(bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=0, configured_port=0) == port


def test_both_owners_remain_callable_on_the_same_real_socket():
    with LiveBridge() as live:
        live.bridge.register_owner("other", "sim").add(ToolDefinition(
            name="inspect", description="test", parameters={}, handler=lambda: {"ok": True}))
        attach(live.bridge, workcell_registry(), Cell(), ToolDefinition, requested_port=live.port)
        with BridgeClient(port=live.port) as client:
            assert client.call("sim.inspect") == {"ok": True}
            assert client.call("workcell.workcell_status") == {"handler": "_cmd_workcell_status"}
            live.bridge.unregister_owner(OWNER)
            assert client.call("sim.inspect") == {"ok": True}
            with pytest.raises(BridgeError, match="unknown tool"):
                client.call("workcell.workcell_status")
