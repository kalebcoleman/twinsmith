"""The MCP server an agent talks to: Isaac Sim and Isaac Lab as MCP tools.

It runs on the host, outside Kit, and reaches a simulator through the bridge extension's local
socket (``client.py``). It serves three kinds of tool:

* its own (screen capture, Isaac Lab jobs, asset packs, simulator sessions), which need no bridge;
* the tools of known extensions (scene tools, the UR5e work cell, the bridge's built-ins), listed from
  a packaged catalog (``host_catalog``) whether or not a simulator is running. Each dials the
  selected bridge when called and says what to do if none answers;
* whatever else the selected bridge reports in ``list_tools``, registered as it is discovered.
  What a live bridge reports replaces the catalog entry; a tool it stops serving goes back to it.

A client is shown related tools as one grouped tool with an ``action`` (``grouped_tools``). A
bridge tool ``<namespace>.<tool>`` is ``<namespace>__<tool>`` to MCP, which allows no dots.

Environment:

    TWINSMITH_PORT, TWINSMITH_HOST   the bridge to use. Unset: the one advertised on this machine.
    TWINSMITH_TOOLSETS               profile and adjustments (default,+sim,+assets).
    TWINSMITH_NAME, TWINSMITH_INSTRUCTIONS   the server's name and instructions.

Nothing here starts a simulator unless the explicit-only ``computer`` toolset is selected.
"""

import functools
import inspect
import json
import logging
import os
import re
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Annotated, Any, AsyncIterator, Dict, Iterable, List, Literal, Mapping, Optional

from pydantic import Field

from mcp.server.fastmcp import Context, FastMCP
from mcp.server.lowlevel.server import NotificationOptions

from . import config, diagnostics, grouped_tools, host_catalog, tool_groups, toolsets
from .scene_contracts import HOST_ONLY
from .tool_hints import CATALOG_HINTS
from .client import BridgeClient, BridgeError
from .isaaclab.tools import register_isaaclab_tools
from .local_tools import IDEMPOTENT_WRITE, READ_ONLY, ScopedTools, strip_schema_titles
from . import computer_use, cursor_badge
from .desktop_input import register_desktop_input_tools
from .sim_sessions import live_sessions, register_sim_session_tools
from .screen import register_screen_tools

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("twinsmith.server")

# How to find running bridges. A stock python3 has no twinsmith package, so `python -m twinsmith.client`
# alone fails there; the file itself is stdlib-only. status also lists advertised bridges.
LIST_BRIDGES_HELP = (
    "`python3 mcp/twinsmith/client.py --list-bridges` from a twinsmith checkout, "
    "`python3 <checkout>/mcp/twinsmith/client.py --list-bridges` from elsewhere, or status")

SERVER_NAME = os.getenv("TWINSMITH_NAME", "twinsmith")
SERVER_INSTRUCTIONS = (
    "Call status first, once: it identifies the bridge, running work and stage IDs, and lists enabled and "
    "disabled toolsets. The twinsmith://toolsets resource reports the same selection. "
    "Use help(task=...) to find a tool; a needed toolset that is off can be turned on with "
    "toolsets(action=\"enable\", names=[...]), except computer, which only the user selects at startup. Prefer listed tools over custom code. Confirm the intended simulator "
    "before changing a scene; do not start extra GPU work or stop unrelated processes without authorization. "
    "Tool visibility is not a security boundary. Respect bridge authentication and read-only refusals.")

RESPONSE_TIMEOUT_S = 300.0
# Discovery must not make host recovery tools wait behind a stalled native Kit
# call. Mutations keep their existing timeout and are never retried on timeout.
DISCOVERY_TIMEOUT_S = 5.0


# ---- What this host process is running (`opencode serve` keeps an MCP child per
# instance, so after a git pull an old server silently keeps running the old code) ----

def _package_version() -> str:
    from importlib import metadata
    try:
        return metadata.version("twinsmith")
    except metadata.PackageNotFoundError:
        return "unknown"


def _find_checkout(module_file: str = __file__) -> Optional[Path]:
    """The git checkout this module was imported from; None for an installed wheel."""
    package_parent = Path(module_file).resolve().parents[1]   # <checkout>/mcp in a source tree
    root = package_parent.parent
    return root if (package_parent / "pyproject.toml").is_file() and (root / ".git").exists() else None


def _read_git_head(root: Optional[Path]) -> Optional[str]:
    """The commit ``root`` has checked out, from its .git files, without running git. Handles
    worktrees and submodules (.git is a file), detached HEAD and packed refs; None if unreadable."""
    if root is None:
        return None
    commit = re.compile(r"^[0-9a-f]{40,64}$")
    try:
        git_dir = root / ".git"
        if git_dir.is_file():
            pointer = git_dir.read_text().strip()
            if not pointer.startswith("gitdir:"):
                return None
            git_dir = (root / pointer.split(":", 1)[1].strip()).resolve()
        head = (git_dir / "HEAD").read_text().strip()
        if not head.startswith("ref:"):
            return head if commit.match(head) else None
        ref = head.split(":", 1)[1].strip()
        common = git_dir
        if (git_dir / "commondir").is_file():
            common = (git_dir / (git_dir / "commondir").read_text().strip()).resolve()
        for base in (git_dir, common):
            if (base / ref).is_file():
                value = (base / ref).read_text().strip()
                return value if commit.match(value) else None
        if (common / "packed-refs").is_file():
            for line in (common / "packed-refs").read_text().splitlines():
                value, _, name = line.partition(" ")
                if name == ref and commit.match(value):
                    return value
    except (OSError, ValueError):   # UnicodeDecodeError is a ValueError
        return None
    return None


# Recorded once, when the server process starts.
_HOST_VERSION = _package_version()
_HOST_CHECKOUT = _find_checkout()
_HOST_COMMIT = _read_git_head(_HOST_CHECKOUT)
_HOST_DRIFT = diagnostics.DriftMonitor.for_package(Path(__file__).resolve().parent)


def _compare_with_bridge(identity: Dict[str, Any], extension: Any) -> None:
    """Add to the ``host`` object whether this gateway and the bridge run the same commit.

    ``bridge_match`` is true, false, or null when either side does not know (an installed wheel, an
    exported tree with no .git, or a bridge too old to report). Uncommitted edits are a warning either
    way, because then a commit does not say what is running."""
    extension = extension if isinstance(extension, dict) else {}
    host_commit, bridge_commit = identity.get("commit"), extension.get("commit")
    known = host_commit and bridge_commit and "unknown" not in (host_commit, bridge_commit)
    identity["bridge_match"] = (host_commit == bridge_commit) if known else None
    notes = [identity["warning"]] if identity.get("warning") else []
    if known and host_commit != bridge_commit:
        notes.append(f"This gateway runs commit {host_commit[:12]} but the bridge runs {bridge_commit[:12]}, so tool "
                     "schemas and behavior can differ. Restart whichever is older, or update its checkout.")
    if extension.get("dirty"):
        notes.append(f"The bridge's tree has {extension.get('dirty_count', 'some')} uncommitted changed file(s), so "
                     "its commit does not fully say what is running.")
    if notes:
        identity["warning"] = " ".join(notes)


def _host_identity(extension: Any = None) -> Dict[str, Any]:
    """The ``host`` object of ``status``: this MCP server process, not the Kit bridge.

    An installed wheel has no checkout and reports only its version. It compares commits, so
    uncommitted edits to the host code are not detected. ``extension`` is the bridge's own report."""
    identity: Dict[str, Any] = {"version": _HOST_VERSION, "pid": os.getpid()}
    if _HOST_CHECKOUT is None:
        if extension is not None:
            _compare_with_bridge(identity, extension)
        return identity
    current = _read_git_head(_HOST_CHECKOUT)
    identity.update(commit=_HOST_COMMIT or "unknown", checkout_commit=current or "unknown",
                    stale=(_HOST_COMMIT != current) if _HOST_COMMIT and current else None)
    if identity["stale"]:
        identity["warning"] = (
            f"This MCP server started from commit {_HOST_COMMIT[:12]} but its checkout is now at "
            f"{current[:12]}, so it is still running the old host code. Restart the MCP connection "
            "(for OpenCode, restart `opencode serve`) to load the new code.")
    identity.update(_HOST_DRIFT.describe(_HOST_COMMIT, current))
    if extension is not None:
        _compare_with_bridge(identity, extension)
    return identity


def _host_identity_text() -> str:
    """The same facts, for the error of a ``status`` call no bridge answered."""
    identity = _host_identity()
    commit = f", commit {identity['commit'][:12]}" if "commit" in identity else ""
    warning = f" Warning: {identity['warning']}" if "warning" in identity else ""
    drift = f" code_drift: true ({identity['code_drift_message']})." if identity.get("code_drift") else ""
    return f" MCP host: version {identity['version']}{commit}, PID {identity['pid']}.{warning}{drift}"


# ---- the bridge connection ----
# Each server keeps one BridgeClient open; client.py owns the socket and the message format.
#
# What a server knows about "its" bridge (the connection, which MCP names came from which
# bridge tools, and the meta they were registered with) lives in a _BridgeState, one per server
# built by build_server(), so two servers in one process do not share discovery state or a
# socket. The default instance (`mcp`) uses the module-level names below (_client, _NAME_MAP,
# _REGISTERED_META, get_client), which is what tests and scripts that predate build_server()
# read and patch.

_client: Optional[BridgeClient] = None

# MCP name -> the bridge's name for the tool.
_NAME_MAP: Dict[str, str] = {}
# MCP name -> what the bridge reported when the tool was registered. A later discovery compares
# against it, so a tool whose arguments or description changed is registered again.
_REGISTERED_META: Dict[str, Dict[str, Any]] = {}


def get_client() -> BridgeClient:
    """The default server's bridge client, connecting on first use."""
    global _client
    if _client is None:
        _client = BridgeClient(timeout=RESPONSE_TIMEOUT_S)   # no host or port: the client finds the bridge
    return _client


class _BridgeState:
    """One server's bridge connection and discovery bookkeeping."""

    def __init__(self) -> None:
        self.toolsets = toolsets.DEFAULT
        self.runtime_enable = True            # may toolsets(action="enable") change the selection
        self.client: Optional[BridgeClient] = None
        self.name_map: Dict[str, str] = {}
        self.registered_meta: Dict[str, Dict[str, Any]] = {}
        self.env: Optional[Mapping[str, str]] = None  # build_server's env (sim session directory)

    def get_client(self) -> BridgeClient:
        if self.client is None:
            self.client = BridgeClient(timeout=RESPONSE_TIMEOUT_S)
        return self.client


class _DefaultBridgeState(_BridgeState):
    """The default server's state, backed by the module-level names so code and tests that
    use (or monkeypatch) `_client`, `_NAME_MAP`, `_REGISTERED_META` and `get_client` see it."""

    @property
    def client(self) -> Optional[BridgeClient]:
        return _client

    @client.setter
    def client(self, value: Optional[BridgeClient]) -> None:
        global _client
        _client = value

    @property
    def name_map(self) -> Dict[str, str]:
        return _NAME_MAP

    @property
    def registered_meta(self) -> Dict[str, Dict[str, Any]]:
        return _REGISTERED_META

    def __init__(self) -> None:  # the base class would assign the properties above
        self.toolsets = toolsets.DEFAULT
        self.runtime_enable = True
        self.env = None

    def get_client(self) -> BridgeClient:
        return get_client()


_DEFAULT_STATE = _DefaultBridgeState()
_STATE_ATTR = "_twinsmith_bridge_state"


def _state_for(server: Any) -> _BridgeState:
    """The bridge state of a server built by build_server(); any other FastMCP (and the default
    instance) uses the module-level one."""
    return getattr(server, _STATE_ATTR, None) or _DEFAULT_STATE


# ---- registering what a bridge serves ----


def registered_tools(server: Any = None) -> List[str]:
    """The canonical names the selected bridge serves, sorted (default: the default server's).
    The live bridge's tools only (see ``offline_tools`` for the catalog proxies); empty before discovery."""
    return sorted(_state_for(server).name_map.values())


def offline_tools(server: Any = None) -> List[str]:
    """The canonical offline catalog proxy names, sorted (default: the default server's).

    These are listed and callable, but the selected bridge has not confirmed it serves them;
    ``registered_tools`` does not include them."""
    offline = _offline_for((globals().get("mcp") or __getattr__("mcp")) if server is None else server)
    return sorted(offline.names.values()) if offline is not None else []


def _mcp_name(canonical: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]", "_", canonical.replace(".", "__"))


def _visible(canonical: str, meta: Dict[str, Any], state=None) -> bool:
    """Visibility is fixed by this server's startup toolset selection."""
    selected = getattr(state, "toolsets", toolsets.DEFAULT)
    namespaces = {meta.get("namespace") for meta in host_catalog.load_catalog().values()}
    return toolsets.owner(canonical, meta.get("namespace"), namespaces) in selected


def _drop_tool(server: FastMCP, name: str) -> None:
    """Unregister one tool. FastMCP offers no call for this, so its tool table is edited directly."""
    server._tool_manager._tools.pop(name, None)


def discover_and_register_tools(server: FastMCP) -> List[str]:
    """Make this server's bridge tools match what the selected bridge serves now, and return what
    changed. If the bridge cannot be read, or its answer is malformed, nothing is changed: an
    unreadable bridge is not an empty one."""
    state = _state_for(server)
    offline = _offline_for(server)
    offline_names = offline.names if offline is not None else {}
    result = state.get_client().call("list_tools", timeout=DISCOVERY_TIMEOUT_S)
    if not isinstance(result, dict) or not isinstance(result.get("tools"), dict):
        raise RuntimeError("bridge list_tools response has no tools dictionary")
    if any(not isinstance(name, str) or not isinstance(meta, dict)
           for name, meta in result["tools"].items()):
        raise RuntimeError("bridge list_tools contains an invalid tool entry")
    try:
        tools = {name: meta for name, meta in result["tools"].items() if _visible(name, meta, state)}
    except ValueError as error:
        raise RuntimeError(f"{error}: this bridge serves a tool this server does not know. Run the MCP server and Kit "
                           "from the same twinsmith checkout, and restart Kit after updating it.") from error
    names = {}
    for canonical, meta in tools.items():
        if "fields" not in meta:
            raise RuntimeError(f"bridge tool {canonical!r} has no field specs")
        name = _mcp_name(canonical)
        if name in names and names[name] != canonical:
            raise RuntimeError(f"bridge tools {names[name]!r} and {canonical!r} collide as {name!r}")
        if (name in server._tool_manager._tools and name not in state.name_map
                and offline_names.get(name) != canonical):   # a live tool may replace its own proxy
            raise RuntimeError(f"bridge tool {canonical!r} collides with host tool {name!r}")
        names[name] = canonical

    # Build every changed tool on a temporary FastMCP registry first. A broken
    # field spec or signature must not remove an existing tool or leave the
    # host with only the first part of the new registry.
    staging = FastMCP("bridge discovery staging")
    staged = {}
    for canonical, meta in sorted(tools.items()):
        name = _mcp_name(canonical)
        if state.name_map.get(name) == canonical and state.registered_meta.get(name) == meta:
            continue
        try:
            register_dynamic_tool(staging, name, canonical, meta, state=state)
        except Exception as error:
            raise RuntimeError(f"invalid bridge tool {canonical!r}: {error}") from error
        staged[name] = (canonical, meta, staging._tool_manager._tools[name])

    changed = []
    departed = {}
    for mcp_name, canonical in list(state.name_map.items()):
        if canonical not in tools:
            previous_tool = server._tool_manager._tools.get(mcp_name)
            _drop_tool(server, mcp_name)
            state.name_map.pop(mcp_name, None)
            state.registered_meta.pop(mcp_name, None)
            departed[canonical] = previous_tool
    for name, (canonical, meta, tool) in staged.items():
        if name in offline_names:
            # A live copy replaces the proxy. Notify only if its client-visible
            # schema, description or other advertised metadata differs.
            if _tool_surface(server._tool_manager._tools.get(name)) != _tool_surface(tool):
                changed.append(f"{canonical} (updated)")
        else:
            changed.append(f"{canonical} (updated)" if name in state.name_map else canonical)
        server._tool_manager._tools[name] = tool
        state.name_map[name] = canonical
        state.registered_meta[name] = meta
    # A catalog tool this bridge does not (or no longer) serve stays listed as an offline proxy.
    offline_changes = _sync_offline(server, state)
    # A departed live tool may have been replaced by its catalog proxy. Its
    # visible change is handled below, so do not count the replacement twice.
    changed.extend(change for canonical, change in offline_changes.items() if canonical not in departed)
    for canonical, previous_tool in departed.items():
        reverted = offline.catalog.get(canonical) if offline is not None else None
        if reverted is not None and canonical in offline_names.values():
            if _tool_surface(previous_tool) != _tool_surface(server._tool_manager._tools.get(_mcp_name(canonical))):
                changed.append(f"{canonical} (offline)")
        else:
            changed.append(f"{canonical} (removed)")
    if changed:
        logger.info("bridge tools changed (%d): %s", len(changed), changed)
    return changed


# ==================== Offline catalog proxies ====================
#
# A server built by build_server() registers proxies for enabled catalogued tools.
# Toolset selection applies to both host tools and bridge tools, offline or not. The bookkeeping lives on the server object
# (not in _BridgeState) because the default state is shared with bare FastMCP instances used by
# scripts and tests, which must not grow proxies. Servers without it behave exactly as before.

_OFFLINE_ATTR = "_twinsmith_offline"


class _OfflineTools:
    """One server's catalog and which of its tools are currently served as offline proxies."""

    def __init__(self, catalog: Dict[str, Dict[str, Any]]) -> None:
        self.catalog = catalog                    # canonical -> catalog meta (unfiltered)
        self.names: Dict[str, str] = {}           # MCP name -> canonical, while a proxy


def _offline_for(server: Any) -> Optional[_OfflineTools]:
    return getattr(server, _OFFLINE_ATTR, None)


def _tool_surface(tool: Any) -> Any:
    """Client-visible fields, excluding the handler and bridge routing state."""
    if tool is None:
        return None
    return (tool.description, tool.parameters, getattr(tool, "title", None), getattr(tool, "meta", None),
            getattr(tool, "annotations", None), getattr(tool, "icons", None))


def _sync_offline(server: Any, state: _BridgeState) -> Dict[str, str]:
    """Make the server's offline proxies match the catalog, the filters and the live tools.

    Every catalog tool that passes the view filters and is not currently served by the selected
    bridge has a proxy; a tool the bridge now serves, or a filter now hides, loses it. The filters
    are re-read from the process environment each time, as live discovery does. Returns changes
    by canonical name; an unchanged replacement by a live tool is not a visible change."""
    changed: Dict[str, str] = {}
    offline = _offline_for(server)
    if offline is None:
        return changed
    wanted = {_mcp_name(canonical): (canonical, meta)
              for canonical, meta in offline.catalog.items() if _visible(canonical, meta, state)}
    for name in list(offline.names):
        if name in state.name_map:                # the live copy took over this name
            offline.names.pop(name)
        elif name not in wanted:                  # hidden since it was installed
            _drop_tool(server, name)
            canonical = offline.names.pop(name)
            changed[canonical] = f"{canonical} (removed)"
    for name, (canonical, meta) in wanted.items():
        if name in state.name_map:
            continue                              # served live (authoritative)
        if offline.names.get(name) == canonical:
            continue
        if name in server._tool_manager._tools and name not in offline.names:
            raise RuntimeError(f"offline catalog tool {canonical!r} collides with host tool {name!r}")
        was_proxy = name in offline.names
        _drop_tool(server, name)                  # FastMCP keeps the first tool of a name
        register_dynamic_tool(server, name, canonical, meta, state=state)
        offline.names[name] = canonical
        changed[canonical] = f"{canonical} (updated)" if was_proxy else canonical
    grouped_tools.sync(server, getattr(state, "env", None))   # the groups follow whatever is registered now
    return changed


# Catalogued tools this host server runs itself (filename-only project discovery); a bridge never
# serves them, so their absence from a bridge is expected and not worth a note.
HOST_RUN_TOOLS = HOST_ONLY


def _offline_note(server: Any) -> str:
    """One line for refresh/use_bridge results: which listed tools the bridge does not serve."""
    offline = _offline_for(server)
    if offline is None or not offline.names:
        return ""
    spaces: Dict[str, int] = {}
    for canonical in offline.names.values():
        if canonical in HOST_RUN_TOOLS:   # served by this host server, never by a bridge
            continue
        space = canonical.split(".", 1)[0] if "." in canonical else "built-ins"
        spaces[space] = spaces.get(space, 0) + 1
    if not spaces:
        return ""
    counts = ", ".join(f"{space}: {count}" for space, count in sorted(spaces.items()))
    return ("\nNote: " + counts + " tool(s) are listed from the packaged catalog but have not "
            "been confirmed on the selected bridge. Select a bridge that loaded the extension "
            "(use_bridge) or start one yourself, then call refresh_tools. They stay callable: "
            "each one dials the selected bridge when called.")


_BRIDGE_HELP = (
    "To use it: find a bridge that loaded the requested tool extension (workcell or sim-tools) with "
    + LIST_BRIDGES_HELP + ", and select it with use_bridge(port), or start "
    "Kit yourself (`./run.sh --headless`, or simulators action=launch when the computer toolset is on) "
    "and call refresh_tools. This server never starts a simulator "
    "on its own.")


def _needs_bridge_guidance(error: BaseException) -> bool:
    """Match endpoint failures that benefit from connection guidance.

    A retry can fail this way after an earlier request was sent, so the outcome
    must still be reported as unknown rather than asserting the tool did not run.
    """
    if isinstance(error, ConnectionRefusedError):
        return True
    return isinstance(error, RuntimeError) and str(error).startswith(
        ("no bridge found", "multiple bridges running"))


def _catalog_error(canonical: str, error: BaseException, state: Optional[_BridgeState]) -> str:
    """The actionable message for a catalogued tool whose call could not reach a serving bridge."""
    if isinstance(error, BridgeError):
        try:
            port = (state or _DEFAULT_STATE).get_client().port
            where = f"the selected bridge (port {port})"
        except Exception:  # noqa: BLE001
            where = "the selected bridge"
        return (f"Error: {canonical} is not available and was not run: {where} is running but did "
                f"not register it, so its Kit has not loaded the extension that provides it, or "
                f"runs an older one. {_BRIDGE_HELP}")
    if "." not in canonical:   # a bridge built-in: every bridge serves it, none was reachable
        return (f"Error: {canonical} is not available: no Isaac Sim bridge could be reached ({error}). "
                "If the connection failed after dispatch, the tool may have run; check the "
                "simulator state before retrying. If Kit was just started it may still be loading "
                f"(2-3 minutes on a first start). {_wait_advice(state or _DEFAULT_STATE, error)} Otherwise find a "
                f"running bridge with {LIST_BRIDGES_HELP}, and select it with "
                "use_bridge(port), or start Kit (`./run.sh --headless`, or simulators action=launch when "
                "the computer toolset is on). This server never starts a simulator on its own.")
    return (f"Error: {canonical} is not available: no Isaac Sim bridge could be reached ({error}). "
            "If the connection failed after dispatch, the tool may have run; check the "
            f"simulator state before retrying. {_BRIDGE_HELP}")


_WAITING_TOOLS = frozenset({"job_status",       # may wait (wait_s) or encode a replay's frames
                            "assets.mesh_inspect", "assets.validate"})   # parse a large USD in a subprocess
_PYTHON_TYPE = {"string": str, "integer": int, "number": float, "boolean": bool, "array": list, "object": dict}


def _build_tool_function(name: str, canonical: str, description: str,
                         fields: List[Dict[str, Any]], state: Optional[_BridgeState] = None):
    """A Python function for one bridge tool, with the signature FastMCP derives the tool's schema
    from. ``fields`` is the tool's argument list as the bridge reports it. An argument that is
    required and has no default is required here; any other defaults to its declared default, or
    to None, which is left out of the call so the tool's own default applies."""
    from typing import Literal

    mandatory = [f for f in fields if f["required"] and not f["has_default"]]
    signature = []
    annotations: Dict[str, Any] = {"return": str}
    for f in mandatory + [f for f in fields if f not in mandatory]:    # Python wants defaults last
        # A kind this host does not know is passed through untyped rather than failing discovery.
        annotation = Literal[tuple(f["enum"])] if f["kind"] == "enum" else _PYTHON_TYPE.get(f["kind"], Any)
        # These versioned contracts also carry usable client-side descriptions and
        # typed vectors. Leave the legacy bridge/UR5e schema surface unchanged.
        from .scene_contracts import REGISTRY
        if canonical in REGISTRY and f["name"] not in REGISTRY[canonical]["parameters"]:
            # A newer bridge can serve a parameter this host's registry does not know yet (the two
            # checkouts sync separately): keep the bridge's plain type and description instead of
            # failing discovery or guessing a typed vector.
            pass
        elif canonical in REGISTRY:
            from pydantic import PlainValidator, WithJsonSchema
            spec = REGISTRY[canonical]["parameters"][f["name"]]
            bounds = {}
            graph_container = canonical == "sim.graph.build" and f["name"] in (
                "nodes", "connections", "values", "targets")
            if graph_container:
                # FastMCP pre-parses strings for non-string annotations; that can
                # fail before the bridge can explain a malformed serialized spec.
                # Keep the underlying annotation str, but let the bridge validate
                # all shapes and publish the supported shapes to clients.
                field_name = f["name"]
                if field_name in ("nodes", "connections"):
                    container = {"type": "array", "items": {"anyOf": [
                        {"type": "array", "items": {"type": "string"}},
                        {"type": "object", "additionalProperties": {"type": "string"}},
                        {"type": "string"},
                    ]}}
                elif field_name == "targets":
                    container = {"type": "object", "additionalProperties": {"anyOf": [
                        {"type": "string"}, {"type": "array", "items": {"type": "string"}},
                    ]}}
                else:
                    container = {"type": "object", "additionalProperties": True}
                variants = [container, {"type": "string"}]
                if not f["required"]:
                    variants.append({"type": "null"})
                schema = {"anyOf": variants, "description": spec["description"]}
                annotation = Annotated[str, PlainValidator(lambda value: value), WithJsonSchema(schema)]
            elif spec.get("type") == "any":
                # `any` is not a JSON-Schema type. Pass the decoded JSON through unchanged, as the
                # graph containers do, and advertise the kinds a caller may send.
                annotation = Annotated[Any, PlainValidator(lambda value: value), WithJsonSchema({
                    "description": spec["description"],
                    "anyOf": [{"type": "boolean"}, {"type": "integer"}, {"type": "number"},
                              {"type": "string"}, {"type": "array"}, {"type": "object"}],
                })]
            elif f["kind"] == "array":
                strings = (f["name"] in {"observe_prims", "observe_joints", "extra_frames", "paths", "dofs", "log", "observe"}
                           or (canonical == "sim.ui.act" and f["name"] == "keys")
                           or canonical in _PACK_TOOLS or canonical in _MESH_TOOLS)  # arrays of paths or names
                annotation = list[str] if strings else list[float]
                if (canonical, f["name"]) in (("sim.prim.create_many", "items"), ("sim.ui.act", "actions")):
                    annotation = list[dict[str, Any]]  # prim specs: objects, not numbers
                if canonical == "sim.occupancy_map" and f["name"] == "clear":
                    # A list of [x, y, radius] circles; list[float] rejected every valid value.
                    annotation = list[Annotated[list[float], Field(min_length=3, max_length=3)]]
                lengths = {"position": 3, "offset": 3, "orientation": 4, "scale": 3,
                           "color": 3, "gravity": 3, "target": 3,
                           "rotate_xyz": 3, "eye": 3, "clipping_range": 2}
                if f["name"] in lengths:
                    bounds = {"min_length": lengths[f["name"]], "max_length": lengths[f["name"]]}
                else:
                    bounds = {"max_length": {"extra_frames": 16, "observe_prims": 16, "observe_joints": 8, "observe": 8, "clear": 16, "keys": 5, "actions": 10, "nodes": 64, "connections": 256, "items": 200}.get(f["name"], 128)}
            if bounds and not graph_container:
                annotation = Annotated[annotation, Field(**bounds)]
        graph_field = canonical == "sim.graph.build" and f["name"] in (
            "nodes", "connections", "values", "targets")
        if f in mandatory:
            signature.append(f["name"])
        else:
            default = f["default"] if f["has_default"] else None
            if default is None and not graph_field:
                annotation = Optional[annotation]
            signature.append(f"{f['name']}={default!r}")
        # On the whole parameter, not inside Optional: clients read the top-level description, and
        # nested under anyOf many never show it.
        if f.get("description") and not graph_field:
            annotation = Annotated[annotation, Field(description=f["description"])]
        annotations[f["name"]] = annotation

    # The description is attached afterwards: it is free text and must not be compiled as source.
    call = functools.partial(_tool_implementation, state=state, mcp_result=True)
    if canonical in _WAITING_TOOLS:
        # FastMCP runs a plain function on its event loop. A wait of up to a minute there would
        # stop the server answering pings and cancellations, so these run on a worker thread.
        import asyncio

        async def waiting(canonical, params, _call=call):
            return await asyncio.to_thread(_call, canonical, params)

        scope = {"_call": waiting}
        exec(f"async def {name}({', '.join(signature)}) -> str:\n    return await _call({canonical!r}, locals())\n", scope)
    else:
        scope = {"_call": call}
        returns = "" if canonical in computer_use.KIT_UI else " -> str"   # these can add an image
        exec(f"def {name}({', '.join(signature)}){returns}:\n    return _call({canonical!r}, locals())\n", scope)
    function = scope[name]
    function.__doc__ = description
    function.__annotations__.update(annotations)
    return function


def _tool_implementation(canonical: str, params: Dict[str, Any],
                         state: Optional[_BridgeState] = None, *,
                         mcp_result: bool = False) -> Any:
    """What every catalogued or discovered tool runs: forward the call, and mark a failure as one."""
    # Unset optional arguments are left out so the tool's own defaults apply.
    params = {k: v for k, v in params.items() if v is not None}
    if canonical in computer_use.KIT_UI:
        return computer_use.kit_ui(canonical, params, state, _dispatch, _failed_tool_result)
    result = _dispatch(canonical, params, state)
    failed = _failed_tool_result(result)
    from .scene_contracts import REGISTRY
    if canonical in REGISTRY:
        from mcp.server.fastmcp.exceptions import ToolError
        if failed:
            raise ToolError(result)
    elif mcp_result and failed:
        from mcp.types import CallToolResult, TextContent
        return CallToolResult(
            content=[TextContent(type="text", text=result)],
            structuredContent={"result": result},
            isError=True,
        )
    return result


def _failed_tool_result(result: str) -> bool:
    """Recognize gateway failures and the bridge's explicit application errors.

    JSON strings (including a user script returning "Error: ..." as data),
    arrays, and successful objects containing an error-looking message remain
    ordinary results. This does not infer success from an arbitrary payload.
    """
    if result.startswith("Error:"):
        return True
    try:
        data = json.loads(result)
    except (ValueError, TypeError):
        return False
    return isinstance(data, dict) and data.get("status") == "error"


_MESH_TOOLS = {"assets.mesh_inspect", "assets.mesh_decimate", "assets.mesh_hulls", "assets.mesh_split", "assets.validate"}
_PACK_TOOLS = {"assets.packs", "assets.pack_doctor", "assets.dependencies", "assets.pack_adopt",
               "assets.pack_pin", "assets.pack_build"}


def _dispatch(canonical: str, params: Dict[str, Any],
              state: Optional[_BridgeState] = None) -> str:
    """Run one tool by its canonical name: on the host where it is a host tool, else on the bridge."""
    try:
        if canonical == "sim.quick_start" and _quick_start_wants_import(params.get("robot"), state):
            return _quick_start_with_import(params, state)
        active = state or _DEFAULT_STATE
        kit_canonical, routed_info = canonical, None
        if canonical.startswith("assets."):
            from .project_assets import Broker, AssetError
            if not hasattr(active, "asset_broker"):
                active.asset_broker = Broker()
            broker = active.asset_broker
            try:
                if canonical == "assets.roots":
                    return json.dumps(broker.listing())
                if canonical == "assets.search":
                    return json.dumps(broker.search(**params))
                if canonical == "assets.scan":
                    return json.dumps(broker.scan(**params))
                if canonical in _MESH_TOOLS:
                    from . import mesh_tools
                    from .asset_packs import PackError
                    try:
                        return json.dumps(mesh_tools.run_tool(canonical, params, broker))
                    except PackError as e:
                        return json.dumps(dict(status="error", code=e.code, message=str(e), **e.details))
                if canonical in _PACK_TOOLS:
                    from .asset_packs import PackError, run_tool
                    try:
                        return json.dumps(run_tool(canonical, params, broker,
                                                   kit_call=lambda name, args: active.get_client().call(name, args)))
                    except PackError as e:
                        return json.dumps(dict(status="error", code=e.code, message=str(e), **e.details))
                    except ValueError as e:  # an invalid pack name or project file
                        return json.dumps(dict(status="error", code=getattr(e, "code", "INVALID_CONFIG"), message=str(e)))
                if canonical in ("assets.info", "assets.import_path"):
                    # Verify destination identity BEFORE reading/copying project files.
                    info = active.get_client().call("sim.stage.info")
                    if info.get("session_id") != params.get("session_id") or info.get("stage_id") != params.get("stage_id"):
                        raise AssetError("STALE_TARGET", "Review current_target and resend its IDs explicitly; no project was read and this call was not retargeted.",
                                         current_target={k: info.get(k) for k in ("session_id", "stage_id", "pid", "source", "default_prim", "meters_per_unit", "up_axis")})
                    if not broker.staging or str(broker.staging) != info.get("staging"):
                        raise AssetError("NO_SHARED_STAGING", "Host and Kit need the same user-approved staging mount/config. Remote upload is disabled. The user can create it with the approval helper in docs/TOOLS-SCENE.md, then restart both.")
                    if canonical == "assets.import_path":
                        from . import asset_import  # one entry for every format: the host picks and converts
                        kit_canonical, params, routed_info = asset_import.route(broker, params)
                        root_id = None
                    else:
                        root_id, relative_path = params["root_id"], params["relative_path"]
                    if root_id is not None:  # import_path was snapshotted by the router
                        snapshot = broker.snapshot(root_id, relative_path, kind="usd")
                        params = {**params, "snapshot_id": snapshot["snapshot_id"], "manifest_sha256": snapshot["manifest_sha256"]}
            except (AssetError, OSError) as e:
                return json.dumps(dict(status="error", code=getattr(e,"code","FILESYSTEM"), message=str(e), **getattr(e, "details", {})))
        if canonical == "sim.quick_start" and _quick_start_needs_host(params.get("robot")):
            # The same approval and snapshot as assets__import_path: no tool can trust a path by itself.
            from .project_assets import Broker, AssetError
            if not hasattr(active, "asset_broker"):
                active.asset_broker = Broker()
            broker = active.asset_broker
            try:
                info = active.get_client().call("sim.stage.info")
                if not broker.staging or str(broker.staging) != info.get("staging"):
                    raise AssetError("NO_SHARED_STAGING", "Host and Kit must use the same staging folder, and these don't: by default both use "
                                     "~/.cache/twinsmith/staging, so one of them has TWINSMITH_ASSET_CONFIG or TWINSMITH_STAGING set that the other "
                                     "lacks, or the folder can't be made private. Give both the same setting and restart both; "
                                     "docs/TOOLS-SCENE.md has the approval helper.")
                kind = "urdf" if str(params["robot"]).lower().endswith((".urdf", ".xacro")) else "usd"
                root_id, relative_path = broker.locate(_quick_start_robot_path(params["robot"]), kind=kind)
                snapshot = broker.snapshot(root_id, relative_path, kind=kind)
                params = {**params, "snapshot_id": snapshot["snapshot_id"], "manifest_sha256": snapshot["manifest_sha256"]}
            except (AssetError, OSError) as e:
                return json.dumps(dict(status="error", code=getattr(e, "code", "FILESYSTEM"), message=str(e), **getattr(e, "details", {})))
        if canonical == "status":
            return json.dumps(_where_am_i(params, active), default=repr)
        if canonical in ("job_status", "job_cancel"):
            return json.dumps(_job_call(canonical, params, active), default=repr)
        result = active.get_client().call(kit_canonical, params)
        if routed_info and isinstance(result, dict):
            result = {**result, "import": routed_info}
        return json.dumps(result, default=repr)
    except BridgeError as e:
        if (str(e).startswith("unknown tool") and "known_tools" in e.details
                and host_catalog.is_catalog_tool(canonical)):
            logger.error(f"Bridge does not serve catalogued tool {canonical}: {e}")
            return _catalog_error(canonical, e, state)
        detail = f"\n{json.dumps(e.details, default=repr)[:2000]}" if e.details else ""
        logger.error(f"Bridge error in {canonical}: {e}")
        return f"Error: {e}{detail}"
    except (OSError, TimeoutError, RuntimeError) as e:
        logger.error(f"Error in {canonical}: {e}")
        # A retry can fail to connect after an earlier attempt reached Kit, so
        # connection guidance must not claim the tool was never executed.
        host = _host_identity_text() if canonical == "status" else ""
        if _needs_bridge_guidance(e) and host_catalog.is_catalog_tool(canonical):
            return _catalog_error(canonical, e, state) + host
        return f"Error: {e}{host}"


# ---- status: one "where am I" answer, from the host's view and the selected bridge's ----

_STATUS_SHORT = ("pid", "port", "app", "app_version", "headless", "app_ready", "uptime_s", "hostname")


def _advertised() -> List[Dict[str, Any]]:
    from .client import discover_bridges

    try:
        return [{k: b.get(k) for k in ("port", "pid", "app") if b.get(k) is not None}
                for b in sorted(discover_bridges(), key=lambda b: int(b["port"]))]
    except Exception:  # noqa: BLE001 - discovery only enriches the answer
        return []


def _surroundings(state: _BridgeState) -> Dict[str, Any]:
    """What runs on this machine besides the selected bridge: advertised bridges, simulators this
    server manages, and running Isaac Lab jobs. Each part is left out when empty."""
    found: Dict[str, Any] = {"bridges": _advertised()}
    sessions = [{k: s.get(k) for k in ("id", "port", "pid", "physics_backend") if s.get(k) is not None}
                for s in live_sessions(getattr(state, "env", None))]
    if sessions:
        found["managed_sessions"] = sessions
    try:
        from .isaaclab import jobs as lab_jobs

        process_jobs = [j["job_id"] for j in lab_jobs.running_jobs() if j.get("runner") != "kit"][-5:]
        if process_jobs:
            found["lab_jobs_running"] = process_jobs
    except Exception:  # noqa: BLE001 - no Lab install, or an unreadable jobs folder
        pass
    return found


def _host_brief() -> Dict[str, Any]:
    host = _host_identity()
    return {k: host[k] for k in ("version", "commit", "stale", "warning", "code_drift", "code_drift_message")
            if host.get(k) not in (None, False)}


def _active_engine(state: _BridgeState) -> Optional[str]:
    """The engine the selected bridge runs, or None when it has no scene tools or does not say."""
    try:
        info = state.get_client().call("sim.stage.info", timeout=DISCOVERY_TIMEOUT_S)
        return (info.get("runtime") or {}).get("physics_engine") or None
    except Exception:  # noqa: BLE001 - status still answers without it
        return None


def _physics_status(state: _BridgeState, active: Optional[str] = None) -> Dict[str, Any]:
    """The physics engine: the one the selected bridge runs (when one answers), the configured default
    for new sessions, and where that default came from. A running session keeps its own engine."""
    settings = getattr(state, "config", None)
    if settings is not None:
        default, source = settings.physics_backend, settings.physics_backend_source
    else:
        default, source = config.resolve_physics_backend()
    out: Dict[str, Any] = {"active": active, "default": default, "default_from": source}
    if active and active != default:
        out["note"] = (f"The selected simulator runs {active}; the configured default, {default}, applies to new sessions "
                       "only. A running session's engine is never changed.")
    return out


def _where_am_i(params: Dict[str, Any], state: _BridgeState) -> Dict[str, Any]:
    """The ``status`` tool. Short by default: the selected bridge, what else runs here, and the
    stage IDs every scene tool needs. With no bridge it is still an answer, not an error."""
    detail = bool(params.get("detail"))
    try:
        identity = state.get_client().call("status")
    except (OSError, RuntimeError) as error:
        if isinstance(error, TimeoutError) or not _needs_bridge_guidance(error):
            raise
        around = _surroundings(state)
        if str(error).startswith("multiple bridges running"):
            message = ("Several Isaac Sim bridges are running and none is selected: pick one with "
                       "use_bridge(port). Do not start another simulator.")
        elif around["bridges"]:
            message = (f"Nothing answers on the port this server is set to use ({error}), but other bridges are "
                       f"advertised: call use_bridge(port={around['bridges'][0]['port']}) after confirming which "
                       "simulator you mean. The robosim (real2sim), isaaclab (sim) and computer tools need no bridge.")
        else:
            message = ("Nothing is running: no Isaac Sim bridge answers. Start your configured simulator, then "
                       "refresh_tools(wait_s=120). See toolsets for available host tools and optional launch support. "
                       "The robosim (real2sim), isaaclab (sim) and computer tools need no bridge.")
        return {"running": False, "message": message, **around, "host": _host_brief(),
                "physics": _physics_status(state), "toolsets": toolsets.describe(state.toolsets, state.runtime_enable)}
    if not isinstance(identity, dict):
        return identity
    host = _host_identity(identity.get("extension", {}))
    if detail:
        return {**identity, "host": host, "physics": _physics_status(state, _active_engine(state) if "sim" in (identity.get("namespaces") or []) else None),
                "toolsets": toolsets.describe(state.toolsets, state.runtime_enable)}
    out: Dict[str, Any] = {"running": True, **{k: identity[k] for k in _STATUS_SHORT if k in identity}}
    if "sim" in (identity.get("namespaces") or []):
        try:
            info = state.get_client().call("sim.stage.info", timeout=DISCOVERY_TIMEOUT_S)
            out["stage"] = {k: info.get(k) for k in ("session_id", "stage_id", "source", "default_prim") if k in info}
            out["stage"]["timeline"] = ("playing" if info.get("playing") else "stopped" if info.get("stopped") else "paused")
            engine = (info.get("runtime") or {}).get("physics_engine")
            if engine:
                out["stage"]["physics_engine"] = engine
        except Exception as error:  # noqa: BLE001 - the identity above is still the answer
            out["stage"] = {"error": str(error)[:200]}
    else:
        out["stage"] = {"note": "This Kit has not loaded sim-tools, so sim__* and assets__* will not run on it."}
    jobs = identity.get("jobs") or {}
    if jobs.get("running"):
        out["jobs_running"] = jobs.get("jobs", [])
    busy = identity.get("busy") or {}
    if busy.get("exclusive") or busy.get("waiting"):
        out["busy"] = busy
    if identity.get("read_only"):
        out["read_only"] = True
    around = _surroundings(state)
    if len(around["bridges"]) <= 1:
        around.pop("bridges")   # the only bridge is the one above
    out.update(around)
    out["host"] = {k: v for k, v in host.items() if k in ("version", "commit", "warning", "code_drift", "code_drift_message",
                                                           "extension_mismatch") and v not in (None, False)}
    out["physics"] = _physics_status(state, (out.get("stage") or {}).get("physics_engine"))
    out["toolsets"] = toolsets.describe(state.toolsets, state.runtime_enable)
    out["note"] = "Checked: no need to call this again before each tool. detail=true adds the full bridge identity."
    return out


# ---- jobs: one answer for a job wherever it runs --------------------------------------------
#
# A job inside Isaac Sim and a job that is its own process (Isaac Lab, a script) keep the same kind
# of record in the same folder. While a simulator job runs, its live state is asked of the bridge;
# everything else is read from the record, so it still answers after Kit or this server restarts.

JOB_WAIT_MAX_S = 60.0
JOB_WAIT_POLL_S = 1.0


def _job_record(job_id: Any) -> Optional[Dict[str, Any]]:
    from .isaaclab import jobs as lab_jobs

    if not isinstance(job_id, str) or not lab_jobs._JOB_ID_RE.match(job_id):
        return None
    try:
        return lab_jobs._read_job(lab_jobs._jobs_path() / job_id)
    except OSError:
        return None


def _bridge_for(record: Optional[Dict[str, Any]], state: _BridgeState):
    """The client for the simulator a job runs in: the selected bridge, or the record's own port."""
    port = (record or {}).get("port")
    if port and port != _screen_port(state):
        return BridgeClient(port=int(port), timeout=JOB_WAIT_MAX_S + 30), True
    return state.get_client(), False


def _wait_seconds(params: Dict[str, Any]) -> float:
    """``wait_s`` as a number of seconds, at most ``JOB_WAIT_MAX_S``: a longer ask waits that long rather than failing,
    so a client that asks for 300 s does not spin on errors. A negative or non-number value is refused."""
    wait = params.get("wait_s", 0) or 0
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not wait >= 0:
        raise ValueError(f"wait_s must be 0..{JOB_WAIT_MAX_S:g} seconds; call again to keep waiting.")
    return min(float(wait), JOB_WAIT_MAX_S)


def _job_call(canonical: str, params: Dict[str, Any], state: _BridgeState) -> Any:
    """job_status and job_cancel for any job."""
    import time

    from .isaaclab import jobs as lab_jobs

    job_id = params.get("job_id")
    try:
        wait = _wait_seconds(params) if canonical == "job_status" else 0.0
        asked = params.get("wait_s") if canonical == "job_status" else None
        if canonical == "job_status" and "wait_s" in params:
            params = {**params, "wait_s": wait}       # the simulator gets the wait it may hold
        record = _job_record(job_id)
        if job_id is None:
            return _job_listing(state)
        in_kit = record is None or (record.get("runner") == "kit" and record.get("status") == "running"
                                    and lab_jobs._job_alive(record))
        if in_kit:
            client, temporary = _bridge_for(record, state)
            try:
                result = client.call(canonical, params, timeout=wait + 30)
            finally:
                if temporary:
                    client.close()
        elif canonical == "job_cancel":
            if record.get("runner") == "kit":     # its simulator has gone: nothing is left to cancel
                ended = lab_jobs.runs_status(job_id)      # settles the record of a job the simulator left running
                result = {"cancelled": False, "state": ended.get("state"),
                          "note": "The simulator this job ran in has exited; the job ended with it."}
            else:
                result = lab_jobs.runs_cancel(job_id)
        else:
            deadline = time.monotonic() + wait
            while True:
                result = lab_jobs.runs_status(job_id)
                if result.get("state") != "running" or time.monotonic() >= deadline:
                    break
                time.sleep(min(JOB_WAIT_POLL_S, max(0.0, deadline - time.monotonic())))
            result = {"job_id": job_id, **result}
            if result.get("state") == "running" and wait:
                result["wait_timed_out"] = True
    except (ValueError, OSError) as error:
        if isinstance(error, (ConnectionError, TimeoutError)) or not isinstance(error, (ValueError, FileNotFoundError)):
            raise
        return {"status": "error", "code": "INVALID_VALUE" if "wait_s" in str(error) else "NOT_FOUND", "message": str(error)}
    if isinstance(asked, (int, float)) and not isinstance(asked, bool) and asked > wait and isinstance(result, dict):
        # its own key: the job's own note (a cut result, why it failed) stays as it is
        result = {**result, "wait_s_used": wait}
        if result.get("state") == "running":
            result["wait_note"] = (f"wait_s {asked:g} is more than the {JOB_WAIT_MAX_S:g} s a call may wait; waited "
                                   f"{wait:g} s. Call again to keep waiting.")
    spec = result.get("result", {}).get("encode") if isinstance(result, dict) and isinstance(result.get("result"), dict) else None
    if isinstance(spec, dict) and {"folder", "pattern", "frames", "output", "fps"} <= set(spec):   # a replay's frames
        result = {**result, "result": _encode_replay(result["result"], state, job_id)}
    return result


def _job_listing(state: _BridgeState) -> Dict[str, Any]:
    """Every job, newest first: the records, and whatever a reachable simulator knows beyond them."""
    from .isaaclab import jobs as lab_jobs

    listed = lab_jobs.runs_list(limit=25)
    jobs = {run["job_id"]: run for run in listed["runs"]}
    try:
        for job in state.get_client().call("job_status", {}, timeout=DISCOVERY_TIMEOUT_S).get("jobs", []):
            jobs.setdefault(job["job_id"], {**job, "runner": "kit"}).update(
                {key: job[key] for key in ("state", "progress", "elapsed_s") if key in job})
    except Exception:  # noqa: BLE001 - no simulator is running: the records are the answer
        pass
    ordered = sorted(jobs.values(), key=lambda job: job["job_id"], reverse=True)
    return {"jobs": ordered, "running": sum(job.get("state") == "running" for job in ordered),
            **({"truncated": True, "hint": listed["hint"]} if listed.get("truncated") else {})}


def _encode_replay(result: Dict[str, Any], state: _BridgeState, job_id: Any) -> Dict[str, Any]:
    """Encode a finished replay's frames into its labelled MP4, once per job. The frames are on Kit's
    machine: when this server cannot read them (a remote Kit) the result says so and keeps them."""
    from . import frames_video
    from .screen import VIDEO_DIR, _ffmpeg

    done = state.__dict__.setdefault("encoded_replays", {})
    key = (_screen_port(state), job_id)
    if key in done:
        return {**{name: value for name, value in result.items() if name not in ("encode", "note")}, **done[key]}
    spec = result["encode"]
    added: Dict[str, Any]
    try:
        frames = frames_video.frame_files(Path(spec["folder"]) / spec["pattern"])
        if len(frames) != spec["frames"]:
            raise RuntimeError(f"{len(frames)} of {spec['frames']} frames are readable in {spec['folder']} from this "
                               "server (Kit may run on another machine)")
        VIDEO_DIR.mkdir(parents=True, exist_ok=True)
        out = VIDEO_DIR / Path(spec["output"]).name
        if out.exists():
            out = out.with_name(f"{out.stem}-{job_id}{out.suffix}")
        video = frames_video.encode(frames, out, float(spec["fps"]), _ffmpeg(), spec.get("label"))
        added = {"video_path": video["path"], "video": {k: v for k, v in video.items() if k != "path"}}
    except Exception as error:  # noqa: BLE001 - the replay itself succeeded; say why there is no video
        added = {"video_error": f"{str(error)[:300]}. The frames are kept: join_videos(mode=\"frames\", "
                                f"inputs=[\"{spec['folder']}/{spec['pattern']}\"]) encodes them."}
        return {**result, **added}
    done[key] = added
    # The frames are a video now: what was needed to make it is no longer the caller's concern.
    return {**{name: value for name, value in result.items() if name not in ("encode", "note")}, **added}


def _quick_start_wants_import(robot, state=None) -> bool:
    """A robot given as an absolute path to a format only assets__import_path reads (MJCF, xacro, SDF, a mesh) or as a folder.
    Some packages (TurtleBot3) keep xacro in a file named .urdf, which the import reads and Kit's plain URDF path refuses."""
    text = os.path.expanduser(str(robot or ""))
    if not os.path.isabs(text):
        return False
    from . import asset_formats

    kind = asset_formats.format_of(text)
    if os.path.isdir(text) or kind in ("mjcf", "xacro", "sdf", "mesh", "map"):
        return True
    if kind == "urdf":
        # Do not inspect arbitrary host paths here. Resolve through the same approved-root
        # broker as import_path; its read refuses symlinks and non-regular files.
        from .project_assets import AssetError, Broker
        active = state or _DEFAULT_STATE
        if not hasattr(active, "asset_broker"):
            active.asset_broker = Broker()
        try:
            root_id, relative_path = active.asset_broker.locate(text, kind="urdf")
            head = active.asset_broker.read_file(root_id, relative_path)[:200_000]
        except (AssetError, OSError):
            return False  # The ordinary route returns the approval/filesystem error.
        return b"xacro:" in head or b"xmlns:xacro" in head
    return False


_IMPORT_OPTIONS = ("fixed_base", "position", "orientation", "xacro_args", "sdf_model", "default_drives", "scale", "as_collider", "mass", "wall_height")


def _quick_start_with_import(params: Dict[str, Any], state) -> str:
    """One call: a scene, then the robot or prop imported into it by assets__import_path (so every format works the same way).
    Play and ROS come after the import, which refuses a running timeline; an environment gets this robot, not its default one."""
    import re

    robot = os.path.expanduser(str(params["robot"]))
    environment = params.get("environment")
    base = {k: v for k, v in params.items() if k not in ("robot", "play", "ros", *_IMPORT_OPTIONS)}
    if environment:
        base["no_default_robot"] = True
    built = _dispatch("sim.quick_start", base, state)
    try:
        scene = json.loads(built)
    except ValueError:
        return built
    if not isinstance(scene, dict) or "session_id" not in scene:
        return json.dumps(scene) if not isinstance(scene, str) else scene
    ids = dict(session_id=scene["session_id"], stage_id=scene["stage_id"])
    stem = os.path.basename(robot.rstrip("/")).split(".")[0] or "robot"
    name = re.sub(r"[^A-Za-z0-9_]", "_", stem)
    name = name if re.match(r"[A-Za-z_]", name) else f"r_{name}"
    prim = f"/World/{name}"
    carry = {k: v for k, v in params.items() if k in _IMPORT_OPTIONS and v is not None}
    imported = _dispatch("assets.import_path", dict(ids, source_path=robot, prim_path=prim, **carry), state)
    try:
        imported = json.loads(imported)
    except ValueError:
        return imported
    if not isinstance(imported, dict) or imported.get("status") != "ok":
        if not isinstance(imported, dict):
            return json.dumps(imported)
        # Not a success with a note: the file is not in the scene.
        return json.dumps({**imported, **ids, "scene_built": True, "message": f"The scene was built, but this file was not imported into it: {imported.get('message')}"},
                          default=repr)
    root = imported.get("articulation_root")
    result = {**scene, "imported": imported,
              "note": "The scene was built, then the file was imported with assets__import_path. `imported` is that call's whole result."}
    if root or "dynamic" in imported:  # a robot or a prop, not a floor plan
        result["robot"] = dict(prim=prim, articulation_root=root, articulation_roots=imported.get("articulation_roots", []), base=imported.get("base"))
    start = (scene.get("environment") or {}).get("robot_start") or (scene.get("environment") or {}).get("robot_base")
    if start and "robot" in result and params.get("position") is None:
        lifted = (imported.get("placement") or {}).get("lifted_by") or 0.0
        moved = _dispatch("sim.prim.set_transform", dict(ids, path=prim, position=[start[0], start[1], start[2] + lifted]), state)
        if '"status": "ok"' not in str(moved) and '"status":"ok"' not in str(moved):
            result["robot"]["placement_note"] = f"It could not be moved to the environment's start: {str(moved)[:300]}"
    if params.get("ros") and "robot" in result:
        wired = _dispatch("sim.ros2_tf", dict(ids, robot=prim, **({"base": "mobile", "world_frame": ""} if imported.get("base") == "floating" else {})), state)
        try:
            result["ros"] = json.loads(wired)
        except ValueError:
            result["ros"] = dict(status="error", message=str(wired)[:600])
    elif params.get("ros"):
        result["ros"] = dict(status="error", message="This file is not a robot, so there is nothing to publish.")
    if params.get("play"):
        played = _dispatch("sim.timeline", dict(ids, action="play"), state)
        try:
            play_result = json.loads(played)
            result["playing"] = play_result.get("status") == "ok"
            result["stopped"] = not result["playing"]
            if not result["playing"]:
                result["play_error"] = play_result
        except (ValueError, AttributeError):
            result["play_error"] = str(played)[:300]
    return json.dumps(result, default=repr)


def _quick_start_needs_host(robot) -> bool:
    """An absolute path or a URDF needs the host's approval and snapshot; a USD in a pack does not."""
    text = str(robot or "")
    return os.path.isabs(os.path.expanduser(text)) or ("/" in text and text.lower().endswith((".urdf", ".xacro")))


def _quick_start_robot_path(robot: str) -> str:
    """An absolute path as given, or the verified file ``<pack id>/<version>/<file>`` in the asset root."""
    if os.path.isabs(os.path.expanduser(robot)):
        return robot
    from . import pack_manifest
    from .project_assets import AssetError

    try:
        root = pack_manifest.resolve_asset_root().path
    except pack_manifest.RootError as exc:
        raise AssetError("ASSET_MISSING", str(exc)) from exc
    try:
        return str(pack_manifest.resolve_pack_file(root, robot))
    except pack_manifest.ManifestError as exc:
        code = "INVALID_VALUE" if str(exc).startswith(("Invalid pack identifier", f"{robot!r} must be")) else "ASSET_MISSING"
        raise AssetError(code, f"{str(exc)[:300]} Name a file as <pack id>/<version>/<file>; assets__pack action=list shows what the "
                         f"asset root {root} holds.") from exc


def register_dynamic_tool(server: FastMCP, name: str, canonical: str,
                          meta: Dict[str, Any], state: Optional[_BridgeState] = None) -> None:
    """Register one bridge tool on ``server`` under its MCP name, with its hints."""
    description = meta.get("description") or f"Run {canonical} in the simulator."
    if "fields" not in meta:
        raise RuntimeError(f"the bridge listed {canonical!r} without its arguments ('fields'): its "
                           "twinsmith.bridge extension is older than this server; update it.")
    if state is None:
        state = _state_for(server)
    handler = _build_tool_function(name, canonical, description, meta["fields"],
                                   None if state is _DEFAULT_STATE else state)

    options: Dict[str, Any] = {}
    from .scene_contracts import DESTRUCTIVE, READ_ONLY as SCENE_READ_ONLY, REGISTRY
    from mcp.types import ToolAnnotations
    if canonical in CATALOG_HINTS:
        options["annotations"] = ToolAnnotations(**CATALOG_HINTS[canonical])
    if canonical in REGISTRY:
        read_only = canonical in SCENE_READ_ONLY
        options["annotations"] = ToolAnnotations(
            readOnlyHint=read_only, destructiveHint=canonical in DESTRUCTIVE, idempotentHint=read_only,
            openWorldHint=canonical.startswith(("assets.", "sim.controller.")) or canonical == "sim.simulation.run")
    server.tool(**options)(handler)
    if canonical in REGISTRY:
        _forbid_unknown_arguments(server, name)
    strip_schema_titles(server, name)
    grouped_tools.shorten_listed(server._tool_manager._tools[name])


def _explain_missing_target(model) -> None:
    """Replace pydantic's "Field required" for the stage IDs with what to do about it. Clients that
    enforce the published required list answer with their own text before reaching this server."""
    if not all(key in model.model_fields and model.model_fields[key].is_required()
               for key in ("session_id", "stage_id")):
        return
    validate = model.model_validate

    def model_validate(obj, **kw):
        if isinstance(obj, dict) and not (obj.get("session_id") and obj.get("stage_id")):
            raise ValueError("session_id and stage_id are required: call sim__stage__info first and "
                             "pass its exact IDs. This call did nothing.")
        return validate(obj, **kw)

    model.model_validate = model_validate


def _forbid_unknown_arguments(server: FastMCP, name: str) -> None:
    """Refuse arguments a scene tool does not have instead of silently dropping them.

    FastMCP otherwise ignores an extra key such as properties={...} on sim__graph__build: the call
    "succeeds" and the values it meant to set are never applied."""
    tool = server._tool_manager._tools.get(name)
    model = getattr(getattr(tool, "fn_metadata", None), "arg_model", None)
    if model is None:
        return
    model.model_config["extra"] = "forbid"
    model.model_rebuild(force=True)
    _explain_missing_target(model)
    if isinstance(getattr(tool, "parameters", None), dict):
        tool.parameters["additionalProperties"] = False   # so clients can see the rule up front


# ==================== Toolsets at run time ====================

RUNTIME_SCOPE = ("This server process only: nothing was written to any config file, and a restart returns to the "
                 "configured selection.")


def _register_host_tools(server: FastMCP, state: _BridgeState, groups: Iterable[str], env: Mapping[str, str]) -> None:
    """Register the host tools of the given toolsets (all of them at startup, only the newly enabled
    ones at run time). A toolset that is not in ``groups`` registers nothing."""
    groups = frozenset(groups)
    register_screen_tools(server, port_provider=lambda: _screen_port(state), groups=groups, state=state)
    register_desktop_input_tools(server, groups=groups)
    register_sim_session_tools(server, groups=groups, env=env,
                               on_launch=lambda record: state.launched_sessions.add(record["id"]))
    register_isaaclab_tools(server, groups=groups)
    single = []
    if "real2sim" in groups:
        from .robosim import register_robosim_tools
        from .sysid import register_sysid_tools
        register_robosim_tools(server)
        register_sysid_tools(server, roots=lambda: _approved_roots(state))
        single += ["robosim", "sysid"]
    if "cad" in groups:
        from .hardware import register_hardware_tools
        register_hardware_tools(server)
        single.append("hardware")
    if "planning" in groups:
        from .gap import register as register_gap
        register_gap(server)
        single.append("gap")
    if "eval" in groups:
        from .vabar import register as register_vabar
        register_vabar(server)
        single.append("vabar")
    for name in single:
        # Listed like every other host tool: no generated titles, no docstring indentation.
        tool = server._tool_manager._tools[name]
        tool.description = inspect.cleandoc(tool.description)
        strip_schema_titles(server, name)
        grouped_tools.shorten_listed(tool)


def _approved_roots(state) -> list:
    """The user-approved asset roots, reading the approval file now if no assets tool has run yet."""
    if not hasattr(state, "asset_broker"):
        from .project_assets import Broker
        state.asset_broker = Broker()
    return [root for root, _ in state.asset_broker.roots.values()]


def _drop_unselected(server: FastMCP, state: _BridgeState) -> None:
    """Unregister every tool whose toolset is no longer selected: live bridge tools, host tools and
    catalog proxies. The grouped tools are rebuilt from what is left by the caller's sync."""
    for mcp_name, canonical in list(state.name_map.items()):
        if not _visible(canonical, state.registered_meta.get(mcp_name, {}), state):
            _drop_tool(server, mcp_name)
            state.name_map.pop(mcp_name, None)
            state.registered_meta.pop(mcp_name, None)
    for name in list(server._tool_manager._tools):
        if name in tool_groups.GROUPS:
            continue
        try:
            group = toolsets.owner(name)
        except ValueError:
            continue
        if group not in state.toolsets:
            _drop_tool(server, name)


def _listed(server: FastMCP) -> List[str]:
    return sorted(grouped_tools.listed_names(server))


async def _change_toolsets(server: FastMCP, action: str, names: Optional[List[str]] = None,
                           ctx: Optional[Context] = None) -> Dict[str, Any]:
    """The ``toolsets`` tool: list, enable or disable toolsets for this server process only."""
    from mcp.server.fastmcp.exceptions import ToolError
    state = _state_for(server)
    if action == "list":
        return toolsets.describe(state.toolsets, state.runtime_enable)
    if action not in ("enable", "disable"):
        raise ToolError(f"action must be list, enable or disable, not {action!r}.")
    if not state.runtime_enable:
        raise ToolError(f"Run-time toolset changes are switched off on this server (tools.runtime_enable = false or "
                        f"TWINSMITH_RUNTIME_ENABLE=0), so {action} did nothing. The user changes the selection at "
                        f"startup. {toolsets.STARTUP}")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) for name in names):
        raise ToolError(f'{action} needs names, a list of toolset names, e.g. names=["sim"]. '
                        f"Valid names: {', '.join(toolsets.TOOLSETS)}.")
    names = list(dict.fromkeys(name.strip() for name in names))
    unknown = [name for name in names if name not in toolsets.TOOLSETS]
    if unknown:
        raise ToolError(f"Unknown toolset {', '.join(map(repr, unknown))}. Valid names: {', '.join(toolsets.TOOLSETS)}. Nothing changed.")
    if action == "enable":
        explicit = [name for name in names if toolsets.TOOLSETS[name].explicit_only]
        if explicit:
            raise ToolError(f"{', '.join(explicit)} cannot be enabled at run time: it is explicit-only "
                            f"({'; '.join(toolsets.TOOLSETS[name].summary for name in explicit)}). "
                            "Only the user can turn it on, at startup: add +" + explicit[0] + " to tools.toolsets in "
                            "~/.config/twinsmith/config.toml, set TWINSMITH_TOOLSETS, or pass --toolsets, then restart the "
                            "MCP server. Nothing changed.")
    elif "core" in names:
        raise ToolError("core cannot be disabled: it holds status, help and this tool. Nothing changed.")
    before_selection, before_listed, before_surface = state.toolsets, _listed(server), _tool_list_surface(server)
    changed = [name for name in names if (name in state.toolsets) != (action == "enable")]
    note = ""
    if changed:
        state.toolsets = (state.toolsets | set(changed)) if action == "enable" else (state.toolsets - set(changed))
        try:
            if action == "enable":
                _register_host_tools(server, state, changed, state.env or {})
            else:
                _drop_unselected(server, state)
            _sync_offline(server, state)
            if action == "enable":
                try:
                    discover_and_register_tools(server)
                except Exception as error:  # noqa: BLE001 - tools answer with their own bridge errors
                    note = (f"No bridge tools were read ({error}); the new tools are listed from the packaged catalog "
                            "and dial the selected bridge when called.")
        except Exception as error:
            state.toolsets = before_selection
            _drop_unselected(server, state)
            _sync_offline(server, state)
            raise ToolError(f"{action} {', '.join(changed)} failed and was undone: {error}") from error
    after_listed = _listed(server)
    if _changed_tool_names(before_surface, _tool_list_surface(server)):
        await _notify_tools_changed(ctx)
    result: Dict[str, Any] = {
        "action": action, "changed": changed, "unchanged": [name for name in names if name not in changed],
        "tools_available" if action == "enable" else "tools_removed":
            sorted(set(after_listed) - set(before_listed)) if action == "enable" else sorted(set(before_listed) - set(after_listed)),
        "on": [name for name in toolsets.TOOLSETS if name in state.toolsets],
        "scope": RUNTIME_SCOPE,
        "note": ("Clients that ignore tools/list_changed can call the tools named in tools_available directly. "
                 "If your client loads tools lazily, search for the tool by name (for example robosim) before calling it."
                 if action == "enable" and changed else "") + (" " + note if note else ""),
    }
    return {key: value for key, value in result.items() if value not in ("", None)}


# ---- the server ----

@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    state = _state_for(server)
    try:
        logger.info("starting with %d tools registered", len(server._tool_manager._tools))
        try:
            _sync_offline(server, state)
        except Exception as e:  # noqa: BLE001
            logger.error(f"Could not refresh the offline catalog tools: {e}")
        try:
            discover_and_register_tools(server)
        except Exception as e:
            logger.warning("no bridge answered at startup (%s); refresh_tools looks again once a simulator runs", e)
        yield {}
    finally:
        if state.client:
            state.client.close()
            state.client = None
        logger.info("stopped")


def _advertise_tools_list_changed(server: FastMCP) -> None:
    """Make the server advertise ``tools.listChanged = true``.

    FastMCP.run_stdio_async calls ``create_initialization_options()`` with no
    arguments, so the capability stays false unless we supply the option.
    Clients such as Claude Code only act on ``notifications/tools/list_changed``
    when the capability was advertised at initialize. The wrapper passes the
    option by default but still honours an explicit ``notification_options``.
    """
    original = server._mcp_server.create_initialization_options

    def create_initialization_options(*args: Any, **kwargs: Any) -> Any:
        if not args and kwargs.get("notification_options") is None:
            kwargs["notification_options"] = NotificationOptions(tools_changed=True)
        return original(*args, **kwargs)

    server._mcp_server.create_initialization_options = create_initialization_options


async def _notify_tools_changed(ctx: Optional[Context]) -> None:
    """Tell the client its tool list changed. Never raises: a failed
    notification must not fail the tool that changed the registry.

    Tools take ``ctx: Context = None`` rather than ``Context | None`` because older SDK
    releases only inject a parameter annotated exactly ``Context`` (a union can be
    treated as an ordinary argument and leak into the advertised schema).
    ``ctx`` is None when a tool is called directly rather than through a session."""
    if ctx is None:
        return
    try:
        await ctx.request_context.session.send_tool_list_changed()
    except Exception as e:  # noqa: BLE001
        logger.warning(f"Could not send tools/list_changed notification: {e}")
    else:
        logger.info("sent tools/list_changed to the client")


def _tool_list_surface(server: FastMCP) -> Dict[str, Any]:
    """The client-visible registry, excluding handlers and bridge routing state."""
    return {name: _tool_surface(tool) for name, tool in server._tool_manager._tools.items()}


def _changed_tool_names(before: Dict[str, Any], after: Dict[str, Any]) -> List[str]:
    return sorted(name for name in before.keys() | after.keys() if before.get(name) != after.get(name))


def _screen_port(state: Optional[_BridgeState] = None) -> int | None:
    """Bridge port for auto-targeting, before or after the client connects."""
    client = (state or _DEFAULT_STATE).client
    if client is not None:
        try:
            return client.port
        except Exception:  # noqa: BLE001 - e.g. nothing pinned and discovery found none or several
            return None
    raw = os.getenv("TWINSMITH_PORT")
    try:
        return int(raw) if raw else None
    except ValueError:
        return None


# refresh_tools(wait_s=...) polls at most this long for a bridge that is still starting.
REFRESH_WAIT_MAX_S = 180.0
REFRESH_POLL_S = 3.0


def _booting_error(error: BaseException) -> bool:
    """Discovery failures a Kit that is still starting produces: nothing advertised or listening
    yet, or a bridge whose main loop is still busy loading (``list_tools`` times out)."""
    return _needs_bridge_guidance(error) or isinstance(error, (TimeoutError, ConnectionError))


def _nothing_listening(error: BaseException) -> bool:
    """No bridge is on the selected port at all. A timeout is different: something accepted the
    connection, probably a bridge still booting, and that must not be switched away from."""
    return isinstance(error, ConnectionRefusedError) or (
        isinstance(error, RuntimeError) and str(error).startswith("no bridge found"))


def _followable_session(state: _BridgeState) -> Optional[Dict[str, Any]]:
    """The managed session refresh_tools may select when nothing listens on the selected port.

    Only when there is exactly one live session, this server process launched it (the directory is
    shared with other hosts and agents), it is on another port, and it would be dialled on loopback.
    Anything less is a guess, so the caller names the sessions instead."""
    from .client import _resolve_endpoint

    sessions = live_sessions(getattr(state, "env", None))
    if len(sessions) != 1 or sessions[0].get("id") not in getattr(state, "launched_sessions", ()):
        return None
    if sessions[0]["port"] == _screen_port(state):
        return None
    if _resolve_endpoint(None, sessions[0]["port"])[0] not in ("127.0.0.1", "localhost", "::1"):
        return None
    return sessions[0]


def _several_launched_sessions(state: _BridgeState, error: BaseException) -> bool:
    """A wait cannot select anything: nothing listens on the port, and this server launched more than one
    live simulator elsewhere. Sessions other clients launched do not count: someone may be starting Kit
    on the configured port beside them, which is what the wait is for."""
    if not _nothing_listening(error) or _screen_port(state) is None:
        return False
    launched = getattr(state, "launched_sessions", ())
    return sum(s.get("id") in launched for s in live_sessions(getattr(state, "env", None))) > 1


def _session_not_ready(session: Dict[str, Any]) -> Optional[str]:
    """Why ``session``'s bridge cannot be selected yet, or None if it can. Runs on a worker thread:
    while Kit boots these calls can block for the whole discovery timeout."""
    port, pid = session["port"], session["pid"]
    try:
        with BridgeClient(host="127.0.0.1", port=port, timeout=DISCOVERY_TIMEOUT_S) as client:
            identity = client.call("status", timeout=DISCOVERY_TIMEOUT_S)
            answered = identity.get("pid") if isinstance(identity, dict) else None
            if answered != pid:
                return (f"port {port} answered as PID {answered}, not the managed session's "
                        f"PID {pid}, so it was not selected")
            if identity.get("app_ready") is False:
                return f"port {port} (PID {pid}) answered but its Kit app is still loading"
            client.call("list_tools", timeout=DISCOVERY_TIMEOUT_S)
    except (OSError, BridgeError, ValueError, RuntimeError) as error:   # also a non-bridge on the port
        return f"port {port} (PID {pid}) is not answering yet ({error})"
    return None


def _advertised_elsewhere(state: _BridgeState) -> List[Dict[str, Any]]:
    """Live bridges advertised on another port than the one this server dials (best effort)."""
    from .client import discover_bridges

    current = _screen_port(state)
    try:
        return sorted((b for b in discover_bridges() if int(b["port"]) != current), key=lambda b: int(b["port"]))
    except Exception:  # noqa: BLE001 - only enriches an error message
        return []


def _dead_endpoint_hint(state: _BridgeState, error: BaseException, others: List[Dict[str, Any]]) -> str:
    """The configured bridge is dead and another is up: that is a wrong port, not a slow start."""
    client = getattr(state, "client", None)
    host = getattr(client, "_host", None) or "127.0.0.1"
    current = _screen_port(state)
    listing = ", ".join(f"port {b['port']} (PID {b.get('pid')}, {b.get('app') or 'unknown app'})" for b in others)
    sessions = live_sessions(getattr(state, "env", None))
    return (f"Error: nothing answers on {host}:{current}, the endpoint this server is set to use ({error}). Other bridges "
            f"are advertised and up: {listing}. This is not Kit still starting. Confirm which simulator you mean, then call "
            f"use_bridge(port={others[0]['port']}). Port {current} comes from use_bridge or TWINSMITH_PORT; "
            f"wait only if you are starting a simulator on port {current} yourself."
            + (" " + _wait_advice(state, error) if sessions and all(x["port"] != current for x in sessions) else ""))


def _wait_advice(state: _BridgeState, error: BaseException) -> str:
    """What waiting will actually do, for the port refresh_tools dials and the live sessions."""
    current = _screen_port(state)
    sessions = live_sessions(getattr(state, "env", None))
    follow = _followable_session(state) if _nothing_listening(error) else None
    if follow is not None:
        first = f"port {current} and " if current is not None else ""
        return (f"refresh_tools(wait_s=120) polls {first}port {follow['port']}, and selects port "
                f"{follow['port']} (the simulator this server launched) when it is ready"
                + (f" and port {current} still does not answer." if current is not None else "."))
    if current is not None and sessions and all(s["port"] != current for s in sessions):
        ports = ", ".join(str(s["port"]) for s in sessions)
        guess = " several managed sessions are live, so it will not guess;" if len(sessions) > 1 else ""
        return (f"refresh_tools keeps dialling port {current} and does not select port(s) {ports}:{guess} "
                f"check readiness with simulator_status, then call use_bridge(port={sessions[0]['port']}).")
    target = f" port {current}" if current is not None else ""
    return f"Wait until it is ready (refresh_tools(wait_s=120) polls{target} for you)."


def _starting_hint(server: FastMCP, error: BaseException, session_detail: Optional[str] = None) -> str:
    """Explain a discovery failure that usually means Kit is still starting, with any managed
    simulator session's age (launch_simulator records ``started_at``)."""
    import time
    state = _state_for(server)
    current = _screen_port(state)
    if current is not None and _nothing_listening(error) and _followable_session(state) is None:
        others = _advertised_elsewhere(state)
        if others:
            return _dead_endpoint_hint(state, error, others)
    sessions = live_sessions(getattr(state, "env", None))
    lines = []
    for session in sessions:
        age = (f", started {time.time() - session['started_at']:.0f} s ago"
               if session.get("started_at") else "")
        where = "the configured port" if session["port"] == current else "call use_bridge(port) to select it"
        lines.append(f"port {session['port']} (PID {session['pid']}{age}; {where})")
    managed = ("" if not lines else " Managed simulators started with launch_simulator are running on "
               + ", ".join(lines) + ".")
    detail = f" Last check: {session_detail}." if session_detail else ""
    if isinstance(error, TimeoutError):
        cause = (f"Error: the Isaac Sim bridge accepted the connection but did not answer list_tools "
                 f"in time ({error}). Kit is probably still starting: the bridge opens its port "
                 "before the app finishes loading, and a first start can take 2-3 minutes.")
    else:
        cause = (f"Error: no Isaac Sim bridge is reachable ({error}). Isaac Sim is probably not running "
                 "yet, or is still starting (a first start can take 2-3 minutes before the bridge "
                 "answers).")
    return (cause + managed + detail + " " + _wait_advice(state, error) + " If nothing is running, start "
            "it (simulators action=launch when the computer toolset is on, or ./run.sh), then call refresh_tools "
            f"again. List running bridges with {LIST_BRIDGES_HELP}.")


def _other_sessions_note(server: FastMCP) -> str:
    """After a successful refresh: say so if a managed session is live on another port, so a client
    that just called launch_simulator does not assume the answering bridge is its new simulator."""
    state = _state_for(server)
    current = _screen_port(state)
    others = [s for s in live_sessions(getattr(state, "env", None)) if s["port"] != current]
    if current is None or not others:
        return ""
    listing = ", ".join(f"port {s['port']} (PID {s['pid']})" for s in others)
    return (f"\nNote: this server stayed on port {current}, whose bridge answered. A managed simulator "
            f"is also running on {listing}; call use_bridge(port) to select it.")


def _probe_bridge(server: FastMCP) -> None:
    """Ask the bridge for its tools on a fresh connection (run on a worker thread by refresh_tools):
    while Kit boots, list_tools can block for the whole discovery timeout, and doing that on the
    event loop froze the server for most of a long wait."""
    from .client import _resolve_endpoint

    state = _state_for(server)
    if state.client is not None:
        # Resolve host and port as one pair: a discovered bridge may advertise a non-loopback host,
        # and passing only the port would dial 127.0.0.1 and look like a bridge that never starts.
        host, port = _resolve_endpoint(state.client._host, state.client._port)
        client = BridgeClient(host=host, port=port)
    else:
        client = BridgeClient()
    try:
        client.call("list_tools", timeout=DISCOVERY_TIMEOUT_S)
    finally:
        try:
            client.close()
        except Exception:  # noqa: BLE001
            pass


async def _refresh_tools(server: FastMCP, ctx: Optional[Context] = None, wait_s: float = 0) -> str:
    import asyncio
    import math
    import time
    try:
        wait = float(wait_s or 0)
    except (TypeError, ValueError):
        return "Error: wait_s must be a number of seconds (0 to 180)."
    if not math.isfinite(wait):
        return "Error: wait_s must be a finite number of seconds (0 to 180)."
    wait = min(max(wait, 0.0), REFRESH_WAIT_MAX_S)
    start = time.monotonic()
    deadline = start + wait
    attempts = 0
    session_detail = None

    while True:
        attempts += 1
        try:
            if wait:
                await asyncio.to_thread(_probe_bridge, server)  # registration stays on the loop thread
            added = discover_and_register_tools(server)
            break
        except Exception as e:
            if isinstance(e, RuntimeError) and str(e).startswith("multiple bridges running"):
                return (f"Error: {str(e).rstrip('.')}. Pick one with use_bridge(port) (each one's port and PID: "
                        f"{LIST_BRIDGES_HELP}); do not start another simulator.")
            if not _booting_error(e):
                return f"Error: {e}"
            state = _state_for(server)
            session = _followable_session(state) if _nothing_listening(e) else None
            if session is not None:
                session_detail = await asyncio.to_thread(_session_not_ready, session)
                if session_detail is None:
                    return await _select_launched_session(server, session, ctx)
            if wait and _several_launched_sessions(state, e):
                return _starting_hint(server, e, session_detail) + (
                    " Returned without waiting: this server launched several simulators that are still live, so it "
                    "will not choose one, and waiting would not change that. Pick one with use_bridge(port). Only if "
                    "one exits, or something starts on the configured port, would refresh_tools then succeed.")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                waited = (f" (waited {time.monotonic() - start:.0f} s, {attempts} attempt(s))" if wait else "")
                return _starting_hint(server, e, session_detail) + waited
            await asyncio.sleep(min(REFRESH_POLL_S, remaining))
    if added:
        await _notify_tools_changed(ctx)
    waited = f" after {attempts} attempt(s)" if attempts > 1 else ""
    return ((f"Registered/updated/removed {len(added)} tool(s){waited}: {added}" if added
             else f"No new or changed tools (registry unchanged){waited}.")
            + _offline_note(server) + _other_sessions_note(server))


async def _select_launched_session(server: FastMCP, session: Dict[str, Any],
                                   ctx: Optional[Context] = None) -> str:
    """Move to the session _followable_session chose, through the use_bridge path, and say so."""
    previous = _screen_port(_state_for(server))
    result = await _use_bridge(server, session["port"], ctx)
    if result.startswith("Error:"):
        return result
    why = (f"port {previous}, which this server was using, did not answer" if previous is not None
           else "no port is configured and no bridge is advertised")
    return (f"Selected the bridge on port {session['port']} (PID {session['pid']}): {why}, "
            f"and it is the only managed simulator running, launched by this server (session "
            f"{session['id']}). This lasts until use_bridge or a server restart. Confirm the PID with "
            f"status before scene changes.\n{result}")


USE_BRIDGE_TIMEOUT_S = 10.0


async def _use_bridge(server: FastMCP, port: int, ctx: Optional[Context] = None) -> str:
    state = _state_for(server)
    if type(port) is not int or not 1 <= port <= 65535:
        return f"Error: port must be a number from 1 to 65535, not {port!r}. Nothing changed."
    try:
        new_client = BridgeClient(port=port)
        # A short limit: this blocks the server, and a port that accepts TCP may not be a bridge at all.
        identity = new_client.call("status", timeout=USE_BRIDGE_TIMEOUT_S)
    except Exception as e:  # noqa: BLE001
        return (f"Error: no bridge answered on port {port} within {USE_BRIDGE_TIMEOUT_S:g} s: {e}. Nothing changed. "
                "It may be busy, or not a bridge: " + LIST_BRIDGES_HELP + " lists them.")
    before = _tool_list_surface(server)
    old_client = state.client
    state.client = new_client
    logger.info(f"bridge selected: use_bridge port {port} (previous client "
                f"{'closed' if old_client is not None else 'none'})")
    for name in list(state.name_map):
        _drop_tool(server, name)
    state.name_map.clear()
    state.registered_meta.clear()
    # The old bridge's catalog tools stay listed, as offline proxies, whatever the new one serves.
    _sync_offline(server, state)
    if old_client is not None:
        try:
            old_client.close()
        except Exception:  # noqa: BLE001
            pass
    try:
        discover_and_register_tools(server)
    except Exception as e:  # noqa: BLE001
        changed = _changed_tool_names(before, _tool_list_surface(server))
        if changed:
            await _notify_tools_changed(ctx)
        return (f"Connected to port {port} but tool discovery failed: {e}"
                + (f"\nChanged {len(changed)} tool(s): {changed}" if changed else "") + _offline_note(server))
    changed = _changed_tool_names(before, _tool_list_surface(server))
    if changed:
        await _notify_tools_changed(ctx)
    try:
        summary = json.dumps(identity, indent=2, sort_keys=True)
    except Exception:  # noqa: BLE001
        summary = str(identity)
    return (
        f"switched to bridge on port {port}\n{summary}\n"
        f"registered/updated/removed {len(changed)} tool(s)"
        + (f": {changed}" if changed else "") + _offline_note(server)
    )


def _reported(text: str) -> Any:
    """A gateway tool's text as an MCP result: the same words, with isError set when it failed, so a
    client does not read "Error: ..." as success. A use_bridge that connected but could not list the
    new bridge's tools also failed, although the switch took effect."""
    if text.startswith("Error:") or "but tool discovery failed:" in text.split("\n", 1)[0]:
        from mcp.types import CallToolResult, TextContent
        return CallToolResult(content=[TextContent(type="text", text=text)], isError=True)
    return text


_HELP_TOPICS = (
    ("see or record the result", ("screenshot", "record", "video", "picture", "frame", "evidence", "show", "look", "contact sheet", "sheet", "montage", "attach", "upload"),
     "screenshot(label=...) for a window; start_screen_recording / stop_screen_recording for video; "
     "join_videos to join clips, or join_videos(mode=\"sheet\", inputs=[...] or job_id=..., times=[...]) for a PNG contact sheet of the "
     "same moments of one or more clips with an upload-ready attachment descriptor (clips must be under your home folder); "
     "With no window (headless): sim__camera__create (position and target aim it) + sim__camera__capture for a picture, and "
     "camera= on sim__simulation__run or sim__trajectory__replay for a video. Save absolute paths and show them to the user."),
    ("find out why something failed (contacts, forces, poses)", ("contact", "collision", "fail", "force", "grasp", "stuck", "why", "debug", "pose"),
     "sim__physics__contacts, sim__physics__inspect, sim__body__state, sim__joints__state, sim__prim__measure, "
     "sim__prim__bounds, sim__physics__raycast."),
    ("build or change a scene", ("scene", "build", "robot", "load", "spawn", "import", "asset", "stage", "usd", "urdf"),
     "sim__quick_start (new scene; robot=franka, ur5e, unitree_g1, ... puts a robot in by name, also onto a scene it built "
     "earlier), sim__stage__info (existing), sim__prim__create / create_many, assets__import_path (a robot or asset file "
     "into the current scene), assets__search (files in your own folders). workcell__* only for the lab UR5e work cell."),
    ("move a robot's joints or arm", ("move", "joint", "arm", "ik", "drive", "grip", "gripper", "manipulator"),
     "sim__joints__state, sim__joints__move, sim__joints__drive, sim__arm__ik, sim__arm__fk. These drive joints; "
     "for forces on a free body use sim__simulation__run."),
    ("run your own controller on a body (drone, rover, any rigid body)",
     ("controller", "control", "closed loop", "closed-loop", "thrust", "torque", "wrench", "drone", "quadrotor", "quadcopter",
      "fly", "flight", "hover", "pid", "own code", "my own", "custom", "per step", "every step", "apply force"),
     "sim__simulation__run(body, controller, duration_s): each physics step your controller gets the body's state and "
     "returns {force, torque}; with articulation= it also gets joint state and may return joint_efforts or joint_targets. "
     "controller='tcp://127.0.0.1:<port>' is your own process (start it with run_script; "
     "twinsmith.controller_server.serve is a ready-made server), or 'module:function' loaded into Kit. camera= records it "
     "to a labelled video with no window. It returns a job_id: job_status(job_id, wait_s=30), job_cancel. The CSV log "
     "replays with sim__trajectory__replay."),
    ("replay a saved trajectory or flight into a labelled video",
     ("replay", "trajectory", "flight", "playback", "saved", "labelled", "labeled", "label", "caption", "pose driver"),
     "sim__camera__create, then sim__trajectory__replay(trajectory, prim, camera, label): drives the poses from a CSV or "
     "JSON file and job_status gives video_path, an MP4 with the label on it. Works headless. For a screen recording "
     "instead: sim__viewport(layout='clean', caption=...) and start_screen_recording."),
    ("follow, wait for or stop long work",
     ("job", "wait", "progress", "cancel", "done", "finished", "background", "long", "poll", "is it done", "async"),
     "job_status(job_id, wait_s=30) waits for any job and job_cancel stops it: simulator jobs (run_python(background=true), "
     "sim__simulation__run, sim__trajectory__replay) and Isaac Lab or script jobs alike. isaaclab__runs_status(job_id, "
     "wait_s=45) waits longer on a Lab job, and isaaclab__runs_tail reads its log."),
    ("run any script as a managed job (not only Isaac Lab)",
     ("script", "managed", "batch", "sweep", "subprocess", "evaluator", "any python", "outside isaac"),
     "run_script(script, args, python): your own Python script as a job with a log, timeout, progress and cancel; no "
     "simulator or Isaac Lab needed. Follow it with isaaclab__runs_status / runs_tail / runs_cancel, which manage every job."),
    ("get back to a clean state after a failed run", ("reset", "recover", "clean", "left playing", "still playing", "restart"),
     "sim__timeline action=reset: cancels running simulator jobs, stops the timeline and returns robots to "
     "their start pose. Then sim__stage__info."),
    ("train or evaluate a policy", ("train", "rl", "policy", "reward", "eval", "isaac lab", "isaaclab", "play", "checkpoint", "job", "wait", "baseline", "register", "summar", "pass count", "run dir", "exit code"),
     "isaaclab__envs_list, isaaclab__train, isaaclab__runs_status, isaaclab__runs_tail, isaaclab__metrics_summary, "
     "isaaclab__eval, isaaclab__play, job_cancel. Before a long run: isaaclab__preflight (GPU and jobs) and "
     "isaaclab__env_check (NaNs, resets, memory). isaaclab__runs_status(job_id, wait_s=45) waits for a job to end; "
     "isaaclab__metrics_summary(eval_json=[...absolute paths...]) summarises your evaluation-result JSON against criteria you give; isaaclab__checkpoint_register protects a baseline; run_script runs your own script as a job. "
     "isaaclab__record gives an MP4 with no window. No bridge needed."),
    ("rebuild a real scene from a video and compare it with the real clip (real-to-sim)",
     ("real-to-sim", "real to sim", "real2sim", "reference video", "video", "clip", "robosim", "robosimstudio", "match the real"),
     "robosim, in the real2sim toolset (off by default: toolsets action=enable names=[\"real2sim\"]). It needs no simulator bridge. "
     "Order: robosim action=start with a template (plain data), reference (frames from the clip), compare, update_scene, "
     "tune_push (makes the simulated block move like the real one), rollout, compare_take, report, stop. It compares images; "
     "it does not fit mass or friction (sysid does, from recorded trajectories)."),
    ("fit physical parameters (damping, armature, friction, gains, mass) to recorded trajectories (system identification)",
     ("system identification", "sysid", "identify", "fit damping", "fit friction", "fit mass", "armature", "damping",
      "recorded trajectories"),
     "sysid, in the real2sim toolset (off by default: toolsets action=enable names=[\"real2sim\"]). It needs no simulator bridge. "
     "sysid action=status checks its MuJoCo environment; fit (a dataset folder inside an approved root, with SOURCE.md) "
     "starts a job; jobs action=status follows it; report returns nominal, fitted and bounds per parameter."),
    ("plan a task as a skill graph or score it on a benchmark (Graph-as-Policy, VABAR)",
     ("graph-as-policy", "skill graph", "gap ", "vabar", "benchmark", "workflow graph", "open-robot-skills"),
     "gap in the planning toolset and vabar in the eval toolset (both off by default: toolsets action=enable "
     "names=[\"planning\", \"eval\"]). They need no simulator bridge and run their own pinned checkouts. gap action=status, "
     "tools, check, generate (paid model, only when the owner allows it), run (simulation only, a job). vabar action=status, "
     "list, run (a job), bench (print_only=true shows the plan), results. Simulation only."),
    ("operate a desktop app (FreeCAD, RViz, any X11 window) or Isaac Sim's own UI with mouse and keyboard",
     ("click", "typing", "type into", "type text", "keyboard", "mouse", "button", "menu", "toolbar", "dialog", "widget",
      "desktop", "gui", "freecad", "rviz", "computer use", "operate", "scroll"),
     "doctor says what can work here and what to fix. Any X11 window: list_windows, then screenshot(window_id=...) "
     "for a picture and its coord_space token, then computer_click, computer_type, computer_key or computer_batch with "
     "that token and screenshot_after=true. Isaac Sim's own UI, also while the desktop is locked: sim__ui__find, then "
     "sim__ui__act on a ref with screenshot_after=true."),
    ("start, check or recover a simulator", ("start", "launch", "bridge", "kit", "hang", "stuck kit", "status", "connect", "where am i", "running"),
     "status (one call: the bridge, what is running, the stage IDs; says so when nothing runs), refresh_tools(wait_s=120), "
     "use_bridge, launch_simulator, simulator_status, stop_simulator, sim__logs__tail."),
)

_RUN_PYTHON_PATTERN = (
    "When no tool fits, say so and use run_python: put the whole loop in `code`, `await "
    "omni.kit.app.get_app().next_update_async()` between steps, pass background=true, call job.report(fraction, text) "
    "as it goes and assign `result`; then job_status(job_id, wait_s=30). One call that runs to the end, not one call per step.")


def _help_gap_file(env: Optional[Mapping[str, str]] = None) -> Path:
    return diagnostics.log_dir(os.environ if env is None else env) / "help-gaps.jsonl"


def _record_help_gap(task: str, reason: str) -> Optional[Path]:
    """Append a question help could not answer to a local file the maintainer reads. Never
    raises and never leaves this machine; TWINSMITH_LOG=0 turns it off with the server log."""
    import time

    if not diagnostics.log_enabled(os.environ):
        return None
    try:
        path = _help_gap_file()
        path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        if path.exists() and path.stat().st_size > 1 << 20:
            return path   # full: a review is overdue, and growing forever helps nobody
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "task": task[:500], "reason": reason,
                                     "version": _HOST_VERSION}) + "\n")
        os.chmod(path, 0o600)
        return path
    except OSError:
        return None


def _help_matches(task: str) -> list:
    """Topics for a task, best first. A topic that only shares one common word with the task is
    dropped when another matches clearly better, so a drone question does not get the arm tools."""
    words = task.lower()
    scored = [(sum(bool(re.search(rf"\b{re.escape(k)}", words)) for k in topic[1]), -index, topic)
              for index, topic in enumerate(_HELP_TOPICS)]
    best = max((score for score, _, _ in scored), default=0)
    return [topic for score, _, topic in sorted(scored, reverse=True) if score and score * 2 >= best]


def _enable_toolsets(names, runtime_enable: bool = True) -> str:
    return "Toolsets off: " + ", ".join(sorted(names)) + ". " + toolsets.enable_hint(names, runtime_enable)


def _help_topic(name: str, text: str, enabled, runtime_enable: bool = True) -> str:
    available, missing = [], set()
    for candidate in dict.fromkeys(re.findall(r"\b[A-Za-z][A-Za-z0-9_]*\b", text)):
        try:
            group = toolsets.owner(candidate)
        except ValueError:
            continue
        if group in enabled:
            available.append(candidate)
        else:
            missing.add(group)
    if missing:
        text = ("Available: " + ", ".join(available) + ". " if available else "") + _enable_toolsets(missing, runtime_enable)
    return f"- {name}: {text}"


def _help(task: Optional[str], no_fit: bool = False, enabled=toolsets.DEFAULT,
                runtime_enable: bool = True) -> str:
    task = (task or "").strip()
    hits = _help_matches(task) if task else []
    lines = []
    if task and (no_fit or not hits):
        recorded = _record_help_gap(task, "no_fit" if no_fit else "no_match")
        lines.append(f"No twinsmith tool is known to fit {task!r}." if not hits or no_fit else "")
        lines.append(_RUN_PYTHON_PATTERN if "sim" in enabled else _enable_toolsets({"sim"}, runtime_enable))
        if recorded is not None:
            lines.append(f"This question was recorded for the maintainer in {recorded} (local file, never sent anywhere).")
        if hits:
            lines.append("The closest topics, in case one was missed:")
        elif task:
            lines.append("The full map, in case a tool was missed:")
    else:
        lines.append("Use these twinsmith tools before writing your own code. If you skip them, say why.")
    chosen = hits or list(_HELP_TOPICS)
    lines += [_help_topic(name, tools, enabled, runtime_enable) for name, _, tools in chosen]
    if task and hits and not no_fit:
        lines.append("None of these fits? Do not bend a tool to the job: call help(task=..., no_fit=true), which "
                     "records the gap and explains the code fallback.")
    lines = [line for line in lines if line]
    lines.append("First call: status, once (the bridge, running work, stage IDs and toolsets).")
    host = _host_identity()
    if host.get("code_drift"):
        lines.append(f"code_drift: true - {host['code_drift_message']}.")
    return "\n".join(lines)


def _tool_help(server: FastMCP, name: str) -> str:
    """The full description and arguments of one tool, as written before the tool list shortened them."""
    registered = server._tool_manager._tools
    name = name.strip().replace(".", "__")
    if name in tool_groups.GROUPS and name in registered:
        return f"{name} is a grouped tool: call {name}(action=\"help\") for every action's full description and arguments."
    if name not in registered:
        return f"No listed tool is called {name!r}. Listed tools: {', '.join(sorted(grouped_tools.listed_names(server)))}."
    tool = registered[name]
    full = grouped_tools.FULL.get(name)
    if full is None or full["listed"] != tool.description:     # not shortened: what is registered is the full text
        full = {"description": tool.description, "arguments": tool.parameters.get("properties", {}),
                "required": tool.parameters.get("required", [])}

    def listed(value):                  # the names a client can call, in the text but not in the keys
        if isinstance(value, str):
            return tool_groups.listed_form(value)
        if isinstance(value, dict):
            return {key: listed(item) if key == "description" or isinstance(item, (dict, list)) else item
                    for key, item in value.items()}
        return [listed(item) for item in value] if isinstance(value, list) else value

    return json.dumps({name: listed({key: full[key] for key in ("description", "arguments", "required")})}, indent=1)


def _register_gateway_tools(server: FastMCP, groups: Iterable[str], state: Optional[_BridgeState] = None) -> None:
    """Register ``refresh_tools`` and ``use_bridge`` (the ``gateway`` group) and ``help`` on ``server``.

    Bridge discovery itself runs from the server lifespan, so omitting these two tools does not
    stop startup discovery; it only removes the client's way to re-run or redirect it."""
    mcp = ScopedTools(server, groups)

    @mcp.tool(annotations=IDEMPOTENT_WRITE)
    async def refresh_tools(wait_s: Annotated[float, Field(description="Seconds to keep polling while no bridge answers, 0-180. A first Kit start can take 120-180.")] = 0, ctx: Context = None) -> Any:  # type: ignore[assignment]
        """Re-discover the bridge's tools and register new ones. Use after Kit starts or restarts,
        after reload_tools, or when tools seem missing.

        A port whose bridge answers is never switched away from. If nothing listens and this server
        launched the only live simulator, it selects that one and says so; otherwise it names
        candidates for use_bridge."""
        return _reported(await _refresh_tools(server, ctx, wait_s))

    @mcp.tool(annotations=READ_ONLY)
    def help(task: Annotated[str | None, Field(description="What you want to do, in a few words, e.g. 'record a video'. Omit for the full map.")] = None,
                   no_fit: Annotated[bool, Field(description="true: no tool named for this task fits; records the question locally and returns the run_python pattern.")] = False,
                   tool: Annotated[str | None, Field(description="A listed tool's name: returns its full description and arguments, which the tool list shortens.")] = None) -> str:
        """Start here. Say what you want to do and get the twinsmith tools to call, in order, instead of
        writing your own screenshot, recording, contact-logging or training code. With tool=name, the
        full text of one tool. Reads nothing from the simulator; works with no bridge."""
        live = state or _state_for(server)       # the selection can change at run time
        if tool:
            return _tool_help(server, tool)
        return _help(task, no_fit, live.toolsets, live.runtime_enable)

    @mcp.tool(name="toolsets", annotations=IDEMPOTENT_WRITE)
    async def toolsets_tool(action: Annotated[Literal["list", "enable", "disable"], Field(description="list: what is on or off. enable/disable: turn toolsets on or off for this server process.")],
                            names: Annotated[list[str] | None, Field(description="Toolset names for enable or disable, e.g. [\"sim\"]. Take them from list or status.")] = None,
                            ctx: Context = None) -> Any:  # type: ignore[assignment]
        """Toolsets: groups of tools that are on or off. list shows which are on, off with their tool counts, and which you can enable now; enable registers a toolset's tools (the tool list changes and the result names the new tools); disable removes them. Lasts for this server process only. computer cannot be enabled at run time; the user selects it at startup."""
        return await _change_toolsets(server, action, names, ctx)

    @mcp.tool(annotations=IDEMPOTENT_WRITE, description=(
        "Switch this MCP server to another running Kit bridge on this machine. Use it when several run "
        "or the wrong one answers. Find candidates with status. Replaces the bridge tools with that Kit's, "
        "within the enabled toolsets. Host tools are unaffected. Lasts until this server restarts."
        f"\n\nOther ways to list bridges: {LIST_BRIDGES_HELP}."))
    async def use_bridge(port: Annotated[int, Field(description="Bridge TCP port from --list-bridges or status.")], ctx: Context = None) -> Any:  # type: ignore[assignment]
        """Switch this MCP server to another running Kit bridge on this machine."""
        return _reported(await _use_bridge(server, port, ctx))


def build_server(env: Optional[Mapping[str, str]] = None,
                 state: Optional[_BridgeState] = None,
                 cli_toolsets: str | None = None) -> FastMCP:
    """Build an isolated server with a fixed, resolved toolset selection."""
    env = dict(os.environ if env is None else env)
    settings = config.resolve(env, cli_toolsets)
    groups = settings.toolsets
    instructions = env.get("TWINSMITH_INSTRUCTIONS", SERVER_INSTRUCTIONS)
    state = _BridgeState() if state is None else state
    state.toolsets = groups
    state.runtime_enable = settings.runtime_enable
    state.config = settings
    cursor_badge.configure(settings.cursor_badge)
    from .project_assets import Broker
    state.asset_broker = Broker(env.get("TWINSMITH_ASSET_CONFIG", ""))
    if state.asset_broker.requested_profile != "trusted_project":
        raise RuntimeError("PROFILE_UNAVAILABLE: isolated/unrecognized profiles cannot start this trusted MCP server. No restricted endpoint or OS sandbox is implemented; no fallback.")
    instructions = tool_groups.listed_form(instructions)
    server = FastMCP(SERVER_NAME, instructions=instructions, lifespan=server_lifespan)
    setattr(server, _STATE_ATTR, state)
    @server.resource("twinsmith://toolsets")
    def toolset_status() -> str:
        """Enabled and disabled toolsets for this server."""
        return json.dumps(toolsets.describe(state.toolsets, state.runtime_enable))

    _advertise_tools_list_changed(server)
    state.env = env
    state.launched_sessions = set()   # IDs launch_simulator created in this server process
    _register_host_tools(server, state, groups, env)
    _register_gateway_tools(server, groups, state)
    setattr(server, _OFFLINE_ATTR, _OfflineTools(host_catalog.load_catalog()))
    _sync_offline(server, state)
    return server


def __getattr__(name):
    if name == "mcp":
        instance = build_server(state=_DEFAULT_STATE)
        globals()[name] = instance
        return instance
    raise AttributeError(name)


async def refresh_tools(*args, **kwargs):
    instance = globals().get("mcp") or __getattr__("mcp")
    return await _refresh_tools(instance, *args, **kwargs)


async def use_bridge(*args, **kwargs):
    instance = globals().get("mcp") or __getattr__("mcp")
    return await _use_bridge(instance, *args, **kwargs)


def main(argv: list[str] | None = None) -> None:
    """Run the MCP server (stdio). `--help` and `--version` exit without starting it."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="twinsmith",
        description="MCP server for Isaac Sim and Isaac Lab: Kit bridge tools plus desktop "
        "screen capture. Speaks MCP over stdio; configure it in your MCP client, do not run it by hand. "
        "Environment: TWINSMITH_PORT (bridge port, default 8766), DISPLAY (screen tools).",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {_package_version()}")
    parser.add_argument("--toolsets", help="Profile and adjustments, e.g. default,+sim,+assets")
    parser.add_argument("command", nargs="?", choices=["toolsets"])
    args = parser.parse_args(argv)
    instance = build_server(cli_toolsets=args.toolsets)
    selected = _state_for(instance).toolsets
    if args.command == "toolsets":
        inventory = build_server(env={**os.environ, "TWINSMITH_TOOLSETS": "all,+computer"})
        listed = inventory._tool_manager.list_tools()
        print("Toolset       State Tools Summary / enable")
        for name, spec in toolsets.TOOLSETS.items():
            count = sum(toolsets.owner(tool.name) == name for tool in listed)
            print(f"{name:13} {'on' if name in selected else 'off':5} {count:5} {spec.summary}; +{name}")
        return
    log_path = diagnostics.setup_server_log(os.environ)
    if log_path is not None:
        diagnostics.install_handlers()
        diagnostics.install_crash_hooks()
        diagnostics.start_heartbeat()
    diagnostics.log_startup(_HOST_VERSION, _HOST_COMMIT, len(instance._tool_manager._tools), os.environ)
    logger.info("toolsets on=%s off=%s", ",".join(name for name in toolsets.TOOLSETS if name in selected),
                ",".join(name for name in toolsets.TOOLSETS if name not in selected))
    if log_path is not None:
        logger.info(f"server log: {log_path}")
    try:
        diagnostics.run_logged(instance.run)
    finally:
        diagnostics.teardown_server_log()


if __name__ == "__main__":
    main()
