"""The bridge: registering tools, the three rules for when a call runs, and what a client sees on the socket."""

import asyncio
import json
import os
import socket
import threading
import time

import pytest

from bridge_testing import LiveBridge, dispatch, make_bridge, receive, roundtrip, send, td
from twinsmith_bridge import protocol
from twinsmith_bridge.bridge import ToolDefinition, is_portfile_name, runtime_dir, valid_portfile
from twinsmith_bridge.protocol import ToolError
from twinsmith_bridge.schema import field_specs


# ---- registry ----

def test_builtins_are_served_bare_named_and_the_listing_hides_internal_ones():
    tools = make_bridge().get_registered_tools()
    assert {"run_python", "status", "job_status", "job_cancel", "put_file", "get_file", "stat_file",
            "reload_tools"} == set(tools)
    assert tools["status"]["namespace"] is None and tools["status"]["read_only"] is True
    assert "list_tools" in make_bridge().get_registered_tools(include_internal=True)


def test_an_owner_registers_tools_under_its_namespace():
    bridge = make_bridge()
    registrar = bridge.register_owner("pack", "demo", display_name="Demo pack")
    assert registrar.add(td("ping", lambda message="pong": {"echo": message}, {"message": {"type": "string"}})) == "demo.ping"

    @registrar.tool("Add two numbers", {"a": {"type": "number", "required": True}}, read_only=True)
    def add(a, b=1):
        return {"sum": a + b}

    tools = bridge.get_registered_tools()
    assert tools["demo.ping"]["owner_id"] == "pack" and tools["demo.add"]["read_only"] is True
    assert tools["demo.ping"]["fields"] == field_specs({"message": {"type": "string"}})
    assert dispatch(bridge, "demo.add", {"a": 2}) == {"status": "success", "result": {"sum": 3}}
    assert bridge.get_owners()["pack"]["tools"] == ["demo.ping", "demo.add"]


@pytest.mark.parametrize("register,problem", [
    (lambda b: b.register_owner("pack", "other"), "already registered"),
    (lambda b: b.register_owner("second", "demo"), "belongs to owner 'pack'"),
    (lambda b: b.register_owner("third", "not a name"), "dotted identifier"),
    (lambda b: b.register_owner("fourth", "fine").add(td("bad name", lambda: 1)), "dotted identifier"),
    (lambda b: b.register_owner("fifth", "ok").add(ToolDefinition(name="nohandler")), "no handler"),
])
def test_names_are_unique_and_well_formed(register, problem):
    bridge = make_bridge()
    bridge.register_owner("pack", "demo").add(td("ping", lambda: 1))
    with pytest.raises(ValueError, match=problem):
        register(bridge)
    with pytest.raises(ValueError, match="already registered"):
        bridge._add_tool("pack", td("ping", lambda: 2))


def test_unregistering_removes_only_that_owner_s_tools():
    bridge = make_bridge()
    bridge.register_owner("a", "one").add(td("x", lambda: 1))
    bridge.register_owner("b", "two").add(td("y", lambda: 2))
    assert bridge.unregister_owner("a") is True and bridge.unregister_owner("a") is False
    assert "one.x" not in bridge.get_registered_tools() and "two.y" in bridge.get_registered_tools()
    assert dispatch(bridge, "one.x")["code"] == "UNKNOWN_TOOL" and "two.y" in dispatch(bridge, "one.x")["known_tools"]
    bridge.register_owner("a", "one")   # free again at once
    assert bridge.unregister_owner("twinsmith.bridge") is False, "the built-ins cannot be removed"


def test_an_owner_with_a_call_running_stays_reserved_until_the_call_ends():
    bridge = make_bridge()
    release = None

    async def slow():
        await release.wait()
        return {"done": True}

    bridge.register_owner("pack", "demo").add(td("slow", slow))

    async def scenario():
        nonlocal release
        release = asyncio.Event()
        call = asyncio.ensure_future(bridge.dispatch("demo.slow"))
        await asyncio.sleep(0)
        with pytest.raises(RuntimeError, match="1 call"):
            bridge.unregister_owner("pack", require_idle=True)
        assert "demo.slow" in bridge.get_registered_tools(), "a refused removal changed nothing"
        bridge.unregister_owner("pack")
        owner = bridge.get_owners()["pack"]
        assert owner["draining"] is True and owner["tools"] == [] and owner["in_flight"] == 1
        with pytest.raises(ValueError, match="still finishing"):
            bridge.register_owner("pack", "demo")
        assert (await bridge.dispatch("demo.slow"))["code"] == "UNKNOWN_TOOL", "no new call reaches the old tools"
        release.set()
        assert (await call)["result"] == {"done": True}
        assert "pack" not in bridge.get_owners()
        bridge.register_owner("pack", "demo")

    asyncio.run(scenario())


# ---- replies ----

def test_a_tool_s_failures_are_replies_with_their_details():
    bridge = make_bridge()
    registrar = bridge.register_owner("pack", "demo")

    def refuse():
        raise ToolError("the stage changed", details={"current": "abc"}, code="STALE")

    registrar.add(td("refuse", refuse))
    registrar.add(td("crash", lambda: 1 / 0))
    registrar.add(td("typed", lambda count: count, {"count": {"type": "integer", "required": True}}))
    assert dispatch(bridge, "demo.refuse") == {"status": "error", "message": "the stage changed", "error_type": "ToolError",
                                              "current": "abc", "code": "STALE"}
    crashed = dispatch(bridge, "demo.crash")
    assert crashed["error_type"] == "ZeroDivisionError" and "ZeroDivisionError" in crashed["traceback"]
    wrong = dispatch(bridge, "demo.typed", {"cuont": 3})
    assert wrong["error_type"] == "TypeError" and wrong["expected_parameters"] == ["count"]
    assert dispatch(bridge, "demo.typed", {"count": 3})["result"] == 3
    assert bridge.tool_activity()["demo.typed"] is False, "finished a moment ago"


def test_frames_survive_values_json_cannot_hold():
    encoded = json.loads(protocol.encode({"result": {"x": float("nan"), "y": [float("inf")], "z": object}}))
    assert encoded["result"]["x"] == "nan" and encoded["result"]["y"] == ["inf"] and "object" in encoded["result"]["z"]
    with pytest.raises(ValueError):
        protocol.decode(b"[1, 2]")
    assert protocol.success(None) == {"status": "success", "result": {}}
    assert protocol.error("no", code="X", missing=None) == {"status": "error", "message": "no", "code": "X"}


# ---- over the socket ----

def test_a_client_lists_tools_and_calls_one():
    with LiveBridge() as live:
        live.bridge.register_owner("pack", "demo").add(td("ping", lambda message="pong": {"echo": message}))
        with live.client() as sock:
            listed = roundtrip(sock, {"type": "list_tools"})
            assert "demo.ping" in listed["result"]["tools"] and "list_tools" not in listed["result"]["tools"]
            assert roundtrip(sock, {"type": "demo.ping", "params": {"message": "hi"}}) == {"status": "success", "result": {"echo": "hi"}}
            assert roundtrip(sock, {"type": "nope"})["code"] == "UNKNOWN_TOOL"
            sock.sendall(b"this is not json\n")
            assert receive(sock)[0]["code"] == "MALFORMED"
            assert roundtrip(sock, {"type": "demo.ping"})["status"] == "success", "a bad line does not end the connection"


def test_request_ids_are_echoed_and_let_calls_share_a_connection():
    with LiveBridge() as live:
        gate = threading.Event()

        async def slow():
            while not gate.is_set():
                await asyncio.sleep(0.005)
            return {"which": "slow"}

        registrar = live.bridge.register_owner("pack", "demo")
        registrar.add(td("slow", slow))
        registrar.add(td("fast", lambda: {"which": "fast"}))
        with live.client() as sock:
            send(sock, {"id": "a", "type": "demo.slow"})
            send(sock, {"id": "b", "type": "demo.fast"})
            first, rest = receive(sock)
            assert first == {"id": "b", "status": "success", "result": {"which": "fast"}}, "the fast call did not wait"
            gate.set()
            assert receive(sock, rest)[0] == {"id": "a", "status": "success", "result": {"which": "slow"}}
            assert "id" not in roundtrip(sock, {"type": "demo.fast"}), "no id in, no id out"


def test_an_inline_tool_answers_while_kit_is_busy():
    with LiveBridge() as live:
        blocked = threading.Event()
        live.loop.call_soon_threadsafe(lambda: blocked.wait(2))   # the main loop is stuck in a long frame
        with live.client() as sock:
            started = time.monotonic()
            answer = roundtrip(sock, {"type": "status"})
            assert answer["status"] == "success" and time.monotonic() - started < 1
            assert answer["result"]["busy"] == {"running": [], "waiting": 0, "exclusive": None}
        blocked.set()


def test_an_exclusive_call_runs_alone_and_others_wait_their_turn():
    with LiveBridge() as live:
        order, gate = [], threading.Event()

        async def exclusive():
            order.append("exclusive starts")
            while not gate.is_set():
                await asyncio.sleep(0.005)
            order.append("exclusive ends")
            return {}

        registrar = live.bridge.register_owner("pack", "demo")
        registrar.add(td("alone", exclusive, exclusive=True))
        registrar.add(td("plain", lambda tag: order.append(tag) or {"tag": tag}, {"tag": {"type": "string"}}))
        with live.client() as sock:
            send(sock, {"id": 1, "type": "demo.alone"})
            while "exclusive starts" not in order:
                time.sleep(0.005)
            send(sock, {"id": 2, "type": "demo.plain", "params": {"tag": "first"}})
            send(sock, {"id": 3, "type": "demo.plain", "params": {"tag": "second"}})
            time.sleep(0.05)
            assert order == ["exclusive starts"] and live.bridge.busy() == {"running": ["demo.alone"], "waiting": 2, "exclusive": "demo.alone"}
            assert dispatch(live.bridge, "demo.plain", {"tag": "local"})["code"] == "BUSY", "an in-process call never waits"
            gate.set()
            replies, rest = [], b""
            for _ in range(3):
                reply, rest = receive(sock, rest)
                replies.append(reply["id"])
            assert replies == [1, 2, 3] and order == ["exclusive starts", "exclusive ends", "first", "second"]


def test_an_exclusive_call_waits_for_running_calls_and_keeps_its_place():
    with LiveBridge() as live:
        order, gate = [], threading.Event()

        async def slow():
            while not gate.is_set():
                await asyncio.sleep(0.005)
            order.append("slow")
            return {}

        registrar = live.bridge.register_owner("pack", "demo")
        registrar.add(td("slow", slow))
        registrar.add(td("alone", lambda: order.append("alone") or {}, exclusive=True))
        registrar.add(td("plain", lambda: order.append("plain") or {}))
        with live.client() as sock:
            send(sock, {"id": 1, "type": "demo.slow"})
            time.sleep(0.05)
            send(sock, {"id": 2, "type": "demo.alone"})
            send(sock, {"id": 3, "type": "demo.plain"})     # arrived after the exclusive one: runs after it
            time.sleep(0.05)
            assert order == []
            gate.set()
            rest = b""
            for _ in range(3):
                _, rest = receive(sock, rest)
            assert order == ["slow", "alone", "plain"]


def test_a_caller_that_hangs_up_does_not_leave_its_waiting_call_to_run():
    with LiveBridge() as live:
        ran, gate = [], threading.Event()

        async def hold():
            while not gate.is_set():
                await asyncio.sleep(0.005)
            return {}

        registrar = live.bridge.register_owner("pack", "demo")
        registrar.add(td("hold", hold, exclusive=True))
        registrar.add(td("mark", lambda: ran.append(1) or {}))
        with live.client() as holder:
            send(holder, {"id": 1, "type": "demo.hold"})
            time.sleep(0.05)
            with live.client() as quitter:
                send(quitter, {"id": 1, "type": "demo.mark"})
                time.sleep(0.05)
            time.sleep(0.05)
            gate.set()
            assert receive(holder)[0]["status"] == "success"
            time.sleep(0.05)
            assert ran == [] and live.bridge.busy()["waiting"] == 0


# ---- the token ----

def test_a_bridge_that_requires_its_token_refuses_a_connection_without_it(tmp_path):
    with LiveBridge(auth=True) as live:
        info = json.loads(next(p for p in (tmp_path / "xdg" / "twinsmith").iterdir() if p.suffix == ".json").read_text())
        token_file = info["token_file"]
        assert info["auth"] is True and os.stat(token_file).st_mode & 0o077 == 0, "only this account can read the token"
        token = open(token_file).read()
        with live.client() as sock:
            refused = roundtrip(sock, {"id": 9, "type": "status"})
            assert refused["code"] == "AUTH_REQUIRED" and refused["id"] == 9 and "TWINSMITH_TOKEN" in refused["message"]
            assert sock.recv(10) == b"", "and the connection is closed"
        with live.client() as sock:
            assert roundtrip(sock, {"type": "status", "token": "wrong" * 8})["code"] == "AUTH_REQUIRED"
        with live.client() as sock:
            assert roundtrip(sock, {"type": "status", "token": token})["status"] == "success"
            assert roundtrip(sock, {"type": "status"})["status"] == "success", "once per connection"
        with live.client() as sock:
            assert roundtrip(sock, {"type": "auth", "token": token})["result"] == {"authenticated": True}
    assert not os.path.exists(token_file), "stopping removes the token and the advertisement"


def test_the_client_finds_the_token_by_itself_and_an_explicit_one_wins(monkeypatch):
    from twinsmith.client import BridgeClient, BridgeError, call, discover_bridges

    with LiveBridge(auth=True) as live:
        monkeypatch.delenv("TWINSMITH_TOKEN", raising=False)
        assert call("status", port=live.port, timeout=5)["auth"] is True
        (advertised,) = discover_bridges()
        assert advertised["port"] == live.port and "token_file" not in advertised and "token" not in advertised
        monkeypatch.setenv("TWINSMITH_TOKEN", "not-the-token-" * 4)
        with pytest.raises(BridgeError, match="AUTH_REQUIRED"):
            BridgeClient(port=live.port, timeout=5).call("status")


# ---- read-only mode ----

def test_read_only_mode_runs_only_tools_declared_read_only():
    with LiveBridge(read_only=True) as live:
        ran = []
        registrar = live.bridge.register_owner("pack", "demo")
        registrar.add(td("look", lambda: {"seen": True}, read_only=True))
        registrar.add(td("change", lambda: ran.append(1) or {}))
        with live.client() as sock:
            assert roundtrip(sock, {"type": "demo.look"})["result"] == {"seen": True}
            refused = roundtrip(sock, {"type": "demo.change"})
            assert refused["code"] == "READ_ONLY" and refused["refused"] is True and "did not run" in refused["message"]
            assert roundtrip(sock, {"type": "run_python", "params": {"code": "result = 1"}})["code"] == "READ_ONLY"
            assert roundtrip(sock, {"type": "put_file", "params": {"path": "/tmp/x", "content_b64": ""}})["code"] == "READ_ONLY"
            status = roundtrip(sock, {"type": "status"})["result"]
            assert status["read_only"] is True and roundtrip(sock, {"type": "job_status"})["status"] == "success"
        assert ran == [] and live.bridge.refused_read_only == 3
        assert dispatch(live.bridge, "demo.change")["code"] == "READ_ONLY", "in-process callers are bound by it too"


def test_read_only_comes_from_the_environment_at_launch(monkeypatch):
    monkeypatch.setenv("TWINSMITH_READ_ONLY", "1")
    assert make_bridge().read_only is True
    monkeypatch.setenv("TWINSMITH_READ_ONLY", "0")
    assert make_bridge().read_only is False


# ---- advertising ----

def test_a_listening_bridge_advertises_itself_and_cleans_up(tmp_path):
    with LiveBridge() as live:
        folder = tmp_path / "xdg" / "twinsmith"
        assert runtime_dir() == str(folder) and oct(folder.stat().st_mode & 0o777) == "0o700"
        (portfile,) = folder.iterdir()
        info = json.loads(portfile.read_text())
        assert is_portfile_name(portfile.name) and valid_portfile(portfile.name, info)
        assert info["pid"] == os.getpid() and info["port"] == live.port and info["auth"] is False
        (folder / "999999999-1234.json").write_text("{}")     # left by a process that is gone
        with pytest.raises(RuntimeError, match="already listening"):
            live.bridge.start(0)
    assert list(folder.iterdir()) == [folder / "999999999-1234.json"]
    with LiveBridge():
        assert not (folder / "999999999-1234.json").exists(), "the next bridge sweeps stale advertisements"


def test_a_port_that_is_taken_is_an_error_not_another_port():
    with LiveBridge() as live:
        other = make_bridge()
        with pytest.raises(OSError):
            other.start(live.port)
        assert other.listening_port is None


# ---- speed and robustness of the socket -------------------------------------------------

def test_a_client_that_stops_reading_does_not_stop_the_others(tmp_path):
    """A tool's answer is sent from the main loop: a client that never reads a large one must not hold it."""
    import time

    from twinsmith.client import BridgeClient

    big = tmp_path / "big.bin"
    big.write_bytes(b"x" * (24 << 20))
    with LiveBridge() as live:
        live.bridge.register_owner("slow", "slow").add(td("big", lambda: {"text": "y" * (24 << 20)}))
        live.bridge.register_owner("fast", "fast").add(td("ping", lambda: "pong"))
        stalled = live.client()
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        send(stalled, {"id": 1, "type": "slow.big"})          # answered on the loop, never read
        send(stalled, {"id": 2, "type": "get_file", "params": {"path": str(big)}})
        time.sleep(0.3)
        with BridgeClient(port=live.port, timeout=3) as other:
            started = time.monotonic()
            assert other.call("fast.ping") == "pong" and other.call("status")["port"] == live.port
            assert time.monotonic() - started < 1.0
        stalled.close()


def test_a_large_file_goes_out_and_comes_back_whole(tmp_path):
    import base64
    import os as _os

    from twinsmith.client import BridgeClient

    data = _os.urandom(5 << 20)
    with LiveBridge() as live, BridgeClient(port=live.port, timeout=20) as client:
        written = client.call("put_file", {"path": str(tmp_path / "a" / "blob.bin"), "content_b64": base64.b64encode(data).decode()})
        assert written["bytes"] == len(data)
        assert base64.b64decode(client.call("get_file", {"path": written["path"]})["content_b64"]) == data
        assert client.call("stat_file", {"path": written["path"]})["bytes"] == len(data)


def test_the_file_tools_answer_while_a_call_has_the_loop_to_itself(tmp_path):
    """They touch only the disk, so they do not wait for Kit's main loop, or make it wait."""
    release = threading.Event()
    (tmp_path / "note.txt").write_text("hello")

    async def hold():
        await asyncio.get_running_loop().run_in_executor(None, release.wait, 5)
        return "released"

    with LiveBridge() as live:
        live.bridge.register_owner("x", "x").add(td("hold", hold, exclusive=True))
        busy, quick = live.client(), live.client()
        send(busy, {"id": 1, "type": "x.hold"})
        assert roundtrip(quick, {"id": 2, "type": "stat_file", "params": {"path": str(tmp_path / "note.txt")}})["result"]["bytes"] == 5
        release.set()
        assert receive(busy)[0]["result"] == "released"


def test_the_tool_listing_is_built_once_per_change_of_the_registry():
    bridge = make_bridge()
    first = bridge.get_registered_tools()
    assert bridge.get_registered_tools() is first
    registrar = bridge.register_owner("x", "x")
    registrar.add(td("ping", lambda: 1))
    assert "x.ping" in bridge.get_registered_tools() and "x.ping" not in first
    bridge.unregister_owner("x")
    assert "x.ping" not in bridge.get_registered_tools()
    assert bridge.tool_count() == len(bridge.get_registered_tools(include_internal=True))


def test_the_client_offers_its_token_once_per_connection(monkeypatch):
    from twinsmith import client as bridge_client

    seen = []
    with LiveBridge(auth=True, token="t" * 32) as live:
        original = live.bridge._frame
        monkeypatch.setattr(live.bridge, "_frame", lambda connection, line: (seen.append(json.loads(line)), original(connection, line))[1])
        monkeypatch.setenv("TWINSMITH_TOKEN", "t" * 32)
        with bridge_client.BridgeClient(port=live.port, timeout=5) as client:
            client.call("status")
            client.call("status")
            client.close()
            client.call("status")           # a new connection: offered again
    assert [(frame["type"], "token" in frame) for frame in seen] == [
        ("auth", True), ("status", False), ("status", False), ("auth", True), ("status", False)]


# ---- findings of the independent review -------------------------------------------------

def test_an_owner_is_told_when_its_last_call_has_ended():
    """The work cell releases its state in this callback: at once when idle, else when its calls are done."""
    release = threading.Event()

    async def slow():
        await asyncio.get_running_loop().run_in_executor(None, release.wait, 5)

    with LiveBridge() as live:
        drained = []
        live.bridge.register_owner("idle", "idle")
        assert live.bridge.unregister_owner("idle", on_drained=lambda: drained.append("idle")) and drained == ["idle"]
        live.bridge.register_owner("busy", "busy").add(td("slow", slow))
        client = live.client()
        send(client, {"id": 1, "type": "busy.slow"})
        deadline = time.monotonic() + 2
        while not live.bridge.get_owners()["busy"]["in_flight"] and time.monotonic() < deadline:
            time.sleep(0.01)
        live.bridge.unregister_owner("busy", on_drained=lambda: drained.append("busy"))
        assert drained == ["idle"] and live.bridge.get_owners()["busy"]["draining"]
        release.set()
        receive(client)
        deadline = time.monotonic() + 2
        while "busy" in live.bridge.get_owners() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert drained == ["idle", "busy"]


def test_the_token_is_written_only_into_a_folder_that_is_this_account_s(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    folder = tmp_path / "twinsmith"
    folder.mkdir(mode=0o777)
    os.chmod(folder, 0o777)
    outside = tmp_path / "outside.txt"
    outside.write_text("untouched")
    with LiveBridge(auth=True) as live:
        assert (os.stat(folder).st_mode & 0o777) == 0o700, "a folder others could write to is closed"
        token = folder / f"{os.getpid()}-{live.port}.token"
        assert (os.stat(token).st_mode & 0o777) == 0o600
    planted = folder / f"{os.getpid()}-1.token"

    class Fixed(type(live.bridge)):
        pass

    os.symlink(outside, planted)                # someone left a link where the token will go
    bridge = make_bridge(auth=True, token="s" * 32)
    bridge._port, bridge._bind_host = 1, "localhost"
    bridge._advertise()
    assert outside.read_text() == "untouched" and not os.path.islink(planted) and planted.read_text() == "s" * 32
    os.unlink(planted)
    (folder / f"{os.getpid()}-1.json").unlink()
    folder.rmdir()
    os.symlink(tmp_path, folder)                # the folder itself is a link: nothing is written
    bridge._advertised = []
    bridge._advertise()
    assert bridge._advertised == []


def test_a_result_json_cannot_hold_is_an_error_not_silence():
    with LiveBridge() as live:
        live.bridge.register_owner("x", "x").add(td("keys", lambda: {(1, 2): 3}))
        loop = {}
        loop["me"] = loop
        live.bridge.register_owner("y", "y").add(td("loop", lambda: loop))
        client = live.client()
        for name in ("x.keys", "y.loop"):
            reply = roundtrip(client, {"id": 9, "type": name})
            assert reply["id"] == 9 and reply["status"] == "error" and reply["code"] == "RESULT_NOT_JSON", reply
        assert roundtrip(client, {"id": 10, "type": "status"})["status"] == "success"


def test_a_token_that_is_not_ascii_is_refused_like_any_wrong_one():
    with LiveBridge(auth=True, token="t" * 32) as live:
        reply = roundtrip(live.client(), {"id": 1, "type": "status", "token": "é"})
        assert reply["code"] == "AUTH_REQUIRED"


def test_before_its_token_a_connection_may_send_only_a_small_request():
    with LiveBridge(auth=True, token="t" * 32) as live:
        client = live.client()
        client.sendall(b'{"type": "status", "pad": "' + b"x" * (protocol.MAX_UNAUTHENTICATED_BYTES + 10))
        assert receive(client)[0]["code"] == "AUTH_REQUIRED"
        ok = live.client()
        assert roundtrip(ok, {"id": 1, "type": "auth", "token": "t" * 32})["result"] == {"authenticated": True}
        big = roundtrip(ok, {"id": 2, "type": "status", "params": {}, "pad": "x" * (protocol.MAX_UNAUTHENTICATED_BYTES * 2)})
        assert big["status"] == "success"


def test_get_file_refuses_what_is_not_a_file_and_what_is_too_large(tmp_path, monkeypatch):
    from twinsmith_bridge import builtins as bridge_builtins

    bridge = make_bridge()
    assert dispatch(bridge, "get_file", {"path": "/dev/zero"})["code"] == "NOT_FOUND"
    (tmp_path / "big").write_bytes(b"x" * 100)
    monkeypatch.setattr(bridge_builtins, "MAX_GET_FILE_BYTES", 10)
    assert dispatch(bridge, "get_file", {"path": str(tmp_path / "big")})["code"] == "TOO_LARGE"


def test_a_job_wait_from_the_main_loop_does_not_freeze_it():
    """A panel click runs on Kit's main loop, which the job needs: waiting there could only time out."""
    with LiveBridge() as live:
        async def forever(job):
            await asyncio.sleep(30)

        async def ask():
            job = live.bridge.jobs.start("test", forever)
            import threading as real

            original = real.main_thread
            real.main_thread = real.current_thread        # this loop thread stands in for Kit's main thread
            try:
                started = time.monotonic()
                reply = (await live.bridge.dispatch("job_status", {"job_id": job.id, "wait_s": 5}))["result"]
            finally:
                real.main_thread = original
            live.bridge.jobs.cancel(job.id)
            return reply, time.monotonic() - started

        reply, took = asyncio.run_coroutine_threadsafe(ask(), live.loop).result(10)
        assert took < 1 and reply["wait_timed_out"] and "main loop" in reply["note"]


def test_a_job_wait_longer_than_a_call_may_hold_is_clamped():
    from twinsmith_bridge import builtins

    with LiveBridge() as live:
        async def quick(job):
            return {"done": True}

        async def ask():
            job = live.bridge.jobs.start("test", quick)
            await asyncio.sleep(0.2)
            return [(await live.bridge.dispatch("job_status", {"job_id": job.id, "wait_s": wait})) for wait in (300, -1)]

        clamped, refused = asyncio.run_coroutine_threadsafe(ask(), live.loop).result(10)
        assert clamped["status"] == "success" and clamped["result"]["wait_s_used"] == builtins.JOB_WAIT_MAX_S
        assert refused["status"] == "error" and refused["code"] == "INVALID_VALUE"
