"""A wedged native bridge must not prevent host recovery via real MCP."""

import asyncio
import socket
import threading
import time

from mcp.shared.memory import create_connected_server_and_client_session
from twinsmith import server as gateway
from twinsmith.client import BridgeClient


def test_call_deadline_applies_to_connect(monkeypatch):
    observed = []

    def connect(address, timeout):
        observed.append(timeout)
        raise socket.timeout()

    monkeypatch.setattr(socket, "create_connection", connect)
    client = BridgeClient(host="127.0.0.1", port=1, timeout=300)
    try:
        client.call("list_tools", timeout=0.1)
    except TimeoutError:
        pass
    else:
        raise AssertionError("connection should time out")
    assert observed == [0.1]  # no retry after timeout


def test_stalled_bridge_keeps_sdk_initialization_and_host_tools_available(monkeypatch):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen()
    done = threading.Event()

    def serve():
        with listener.accept()[0] as peer:
            peer.recv(4096)
            done.wait(5)  # deliberately never answer the discovery request

    worker = threading.Thread(target=serve)
    worker.start()
    monkeypatch.setattr(gateway, "DISCOVERY_TIMEOUT_S", 0.15)
    env = {
        "TWINSMITH_TOOLSETS": "all,+computer",
        "TWINSMITH_PORT": str(listener.getsockname()[1]),
    }
    monkeypatch.setenv("TWINSMITH_PORT", env["TWINSMITH_PORT"])
    server = gateway.build_server(env)

    async def exercise():
        started = time.monotonic()
        async with create_connected_server_and_client_session(
            server._mcp_server
        ) as client:
            await client.initialize()
            names = {t.name for t in (await client.list_tools()).tools}
            assert {
                "simulators",
                "sim__stage",
                "workcell__cell",
            } <= names
            response = await client.call_tool("simulators", {"action": "list"})
            assert not response.isError
            assert time.monotonic() - started < 3

    try:
        asyncio.run(exercise())
    finally:
        done.set()
        worker.join(2)
        listener.close()
