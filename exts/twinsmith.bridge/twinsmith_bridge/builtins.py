"""The tools the bridge serves itself: who am I (``status``), run code (``run_python``), follow
background work (``job_status``, ``job_cancel``), move files, list bridges and reload a tool
package. Each handler returns its result or raises ``ToolError``.
"""

import ast
import base64
import binascii
import contextvars
import functools
import hashlib
import inspect
import io
import json
import os
import platform
import socket
import sys
import threading
import time
import traceback
from typing import Any, Dict, List, Optional

from .jobs import JobError
from .protocol import Base64, ToolError
from .sessions import PRELOADED

# ---- run_python --------------------------------------------------------------------------

_RUN_PYTHON_DESCRIPTION = (
    "Execute Python code on the Kit main thread with full access to the Omniverse APIs (omni, carb, pxr "
    "preloaded; assign to `result` to return a value). Top-level `await` works (e.g. `await "
    "omni.kit.app.get_app().next_update_async()` to let frames run between steps). Set persistent=True with "
    "a session_id to keep variables between calls. Opening a stage (including save-as) resets every "
    "session; the next call says so. For work longer than a few seconds pass background=true: it returns "
    "a job_id at once for job_status / job_cancel, instead of polling with more calls."
)
_RUN_PYTHON_PARAMETERS = {
    "code": {"type": "string", "required": True, "description": "Python code to execute on Kit's main thread"},
    "session_id": {"type": "string",
                   "description": "Session identifier for persistent execution. Use the same ID across calls "
                                  "to share state. Defaults to 'default'."},
    "persistent": {"type": "boolean",
                   "description": "If true, variables persist across calls with the same session_id. The reply "
                                  "lists only the names this call added or rebound (session_vars) and the "
                                  "total (session_var_count)."},
    "background": {"type": "boolean",
                   "description": "Run as a job and return its job_id at once. The code must await (e.g. `await "
                                  "omni.kit.app.get_app().next_update_async()` in its loop) and must not pump "
                                  "the app itself. A `job` object is preloaded: job.report(fraction, text) sets "
                                  "progress, job.checkpoint() raises once job_cancel was called. Follow with "
                                  "job_status(job_id, wait_s=30); `result` and prints come back there."},
    "label": {"type": "string", "description": "With background: a short name shown by job_status."},
}

_capture = contextvars.ContextVar("twinsmith_run_python_output", default=None)


class _Output:
    """The prints of one run_python call. Closed when the call ends, so a task the code left
    running prints to the console again instead of into a buffer nobody reads."""

    def __init__(self) -> None:
        self.buffer = io.StringIO()
        self.open = True


class _StdoutRouter:
    """Stands in for sys.stdout: a write goes to the current call's buffer, or to the real stream
    when the code writing is not inside a call (another Kit task, a task that outlived its call)."""

    _twinsmith_proxy = True

    def __init__(self, real: Any) -> None:
        self._real = real

    def _target(self) -> Any:
        output = _capture.get()
        return output.buffer if output is not None and output.open else self._real

    def write(self, text: str) -> int:
        return self._target().write(text)

    def writelines(self, lines: Any) -> None:
        self._target().writelines(lines)

    def flush(self) -> None:
        self._target().flush()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._real, name)


def _route_stdout() -> None:
    if not getattr(sys.stdout, "_twinsmith_proxy", False):
        sys.stdout = _StdoutRouter(sys.stdout)


def remove_stdout_proxy() -> None:
    """Put the real stream back (extension shutdown)."""
    while getattr(type(sys.stdout), "_twinsmith_proxy", False):
        sys.stdout = sys.stdout._real


class _PythonCall:
    """One run_python call: its namespace, its captured output, and how it reports."""

    def __init__(self, bridge: Any, code: str, session_id: str, persistent: bool) -> None:
        import carb
        import omni
        from pxr import Gf, Sdf, Usd, UsdGeom

        preloaded = {"omni": omni, "carb": carb, "Usd": Usd, "UsdGeom": UsdGeom, "Sdf": Sdf, "Gf": Gf}
        assert tuple(preloaded) == PRELOADED
        self.sessions, self.session_id, self.persistent = bridge.sessions, session_id, persistent
        self.before = self.sessions.namespace(session_id) if persistent else {}
        self.generation = self.sessions.generation
        self.namespace = {**preloaded, "__builtins__": __builtins__, **self.before}
        self.output = _Output()
        try:
            self.compiled = compile(code, "<run_python>", "exec", flags=ast.PyCF_ALLOW_TOP_LEVEL_AWAIT)
        except SyntaxError as exc:
            raise ToolError(f"SyntaxError: {exc}", details={"traceback": traceback.format_exc()})
        self.awaits = bool(self.compiled.co_flags & inspect.CO_COROUTINE)

    def _reset_note(self) -> str:
        reason = self.sessions.take_reset(self.session_id) if self.persistent else ""
        if not reason:
            return ""
        return (f"Session {self.session_id!r} was reset because {reason}: variables, imports and functions from "
                "earlier calls are gone (prim references would be stale). Re-run your setup code.")

    def failed(self, exc: BaseException) -> ToolError:
        """The error to raise for an exception from the code. Call inside the ``except`` block."""
        note = self._reset_note()
        details = {"output": self.output.buffer.getvalue(), "traceback": traceback.format_exc()}
        if note:
            details["session_reset"] = note
        elif self.persistent and isinstance(exc, NameError):
            details["hint"] = ("Persistent sessions keep only the variables of successful calls; names from a "
                               "call that raised are not saved.")
        return ToolError((note + " " if note and isinstance(exc, NameError) else "") + str(exc), details=details)

    def finished(self) -> Dict[str, Any]:
        reply: Dict[str, Any] = {"output": self.output.buffer.getvalue(), "result": self.namespace.get("result")}
        if not self.persistent:
            return reply
        reply["session_id"] = self.session_id
        if self.sessions.generation != self.generation:
            # A stage opened while this call ran: what it holds points at the old stage, so it is not saved.
            self.sessions.take_reset(self.session_id)
            reply["session_vars"] = []
            reply["session_reset"] = (f"A stage was opened during this call, so session {self.session_id!r} was not "
                                      "saved; variables from this and earlier calls are gone. Re-run your setup code.")
            return reply
        note = self._reset_note()
        self.namespace.pop("job", None)      # a background call's job handle is not a variable
        names = self.sessions.persist(self.session_id, self.namespace)
        # Only what this call added or rebound: a long session otherwise repeats every name each call.
        reply["session_vars"] = [n for n in names if n not in self.before or self.before[n] is not self.namespace[n]]
        reply["session_var_count"] = len(names)
        if note:
            reply["session_reset"] = note
        return reply

    def run_now(self) -> Dict[str, Any]:
        token = _capture.set(self.output)
        _route_stdout()
        try:
            exec(self.compiled, self.namespace)
        except Exception as exc:
            raise self.failed(exc) from None
        finally:
            self.output.open = False
            _capture.reset(token)
        return self.finished()

    async def run_awaiting(self) -> Dict[str, Any]:
        token = _capture.set(self.output)
        _route_stdout()
        try:
            await eval(self.compiled, self.namespace)
        except Exception as exc:
            raise self.failed(exc) from None
        finally:
            self.output.open = False
            _capture.reset(token)
        return self.finished()


def cmd_run_python(bridge, code: str, session_id: str = "default", persistent: bool = False,
                   background: bool = False, label: Optional[str] = None):
    """Run the caller's code in Kit. Returns the reply, or a coroutine for it when the code awaits."""
    call = _PythonCall(bridge, code, session_id, persistent)
    if not background:
        return call.run_awaiting() if call.awaits else call.run_now()
    if not call.awaits:
        raise ToolError("background=true needs code that awaits, e.g. `await omni.kit.app.get_app()."
                        "next_update_async()` inside its loop: code that never awaits blocks Kit until it "
                        "returns, so there would be nothing to poll. Nothing ran.")

    async def work(job):
        job.output = call.output.buffer.getvalue
        call.namespace["job"] = job
        try:
            reply = await call.run_awaiting()
        except ToolError as exc:
            exc.details.pop("output", None)      # job_status carries the output already
            raise
        reply.pop("output", None)
        return reply

    try:
        job = bridge.jobs.start("run_python", work, label=label, cooperative=False)
    except JobError as exc:
        raise ToolError(str(exc))
    return {"job_id": job.id, "state": job.state,
            "next": f"job_status(job_id={job.id!r}, wait_s=30) until state is not running; job_cancel stops it."}


# ---- jobs --------------------------------------------------------------------------------

_JOB_STATUS_DESCRIPTION = (
    "Check a background job in this Kit; with no job_id, list the jobs. run_python(background=true), "
    "sim__simulation__run and sim__trajectory__replay return a job_id. Gives state (running, succeeded, failed, cancelled), elapsed_s, "
    "progress, the last output and, when finished, the result or error. The list is newest first. Through the MCP server it also takes an Isaac Lab or script job_id (isaaclab__train, "
    "run_script, ...), with no bridge needed."
)
_JOB_STATUS_PARAMETERS = {
    "job_id": {"type": "string", "description": "job_id from the call that started the work. Omit to list jobs."},
    "wait_s": {"type": "number",
               "description": "Wait up to this many seconds (0-60; a larger value waits 60) for the job to end "
                              "before answering, instead of polling."},
}
JOB_WAIT_MAX_S = 60.0


def cmd_job_status(bridge, job_id: Optional[str] = None, wait_s: float = 0) -> Dict[str, Any]:
    """One job, or the list. Runs on the socket thread, so a wait holds up nothing in Kit."""
    if job_id is None:
        jobs = bridge.jobs.jobs()
        return {"jobs": [job.describe(full=False) for job in jobs],
                "running": sum(job.state == "running" for job in jobs)}
    if isinstance(wait_s, bool) or not isinstance(wait_s, (int, float)) or not wait_s >= 0:
        raise ToolError(f"wait_s must be 0..{JOB_WAIT_MAX_S:g} seconds; call again to keep waiting.", code="INVALID_VALUE")
    asked, wait_s = wait_s, min(float(wait_s), JOB_WAIT_MAX_S)   # a longer wait is a common ask: wait the most, not fail
    try:
        job = bridge.jobs.get(job_id)
    except JobError as exc:
        raise ToolError(str(exc), code="NOT_FOUND")
    on_main_loop = threading.current_thread() is threading.main_thread()   # a panel click, or bridge.dispatch
    if wait_s and job.state == "running" and not on_main_loop:
        job.ended.wait(wait_s)
    reply = job.describe()
    if asked > wait_s:
        reply["wait_s_used"] = wait_s
    if wait_s and job.state == "running":
        reply["wait_timed_out"] = True
        if on_main_loop:      # the job needs the loop this caller is on: waiting here could only time out
            reply["note"] = "Not waited for: this call is on Kit's main loop, which the job needs. Ask again."
    return reply


def cmd_job_cancel(bridge, job_id: str) -> Dict[str, Any]:
    try:
        return bridge.jobs.cancel(job_id)
    except JobError as exc:
        raise ToolError(str(exc), code="NOT_FOUND")


# ---- files -------------------------------------------------------------------------------

def _path(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def cmd_put_file(bridge, path: str, content_b64: str, mkdirs: bool = True) -> Dict[str, Any]:
    """Write a file on the simulator's machine."""
    try:
        data = base64.b64decode(content_b64, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise ToolError(f"content_b64 is not valid base64: {exc}")
    target = _path(path)
    try:
        if mkdirs and os.path.dirname(target):
            os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "wb") as stream:
            stream.write(data)
    except OSError as exc:
        raise ToolError(f"cannot write {target}: {exc}" + ("" if mkdirs else "; pass mkdirs=true to create the folder"))
    if target.endswith(".py"):
        from .packages import forget_bytecode

        forget_bytecode(os.path.dirname(target))    # the next import must not find the old bytecode
    return {"path": target, "bytes": len(data)}


MAX_GET_FILE_BYTES = 45 * 1024 * 1024      # base64 of this still fits one 64 MiB message


def cmd_get_file(bridge, path: str) -> Dict[str, Any]:
    target = _path(path)
    if not os.path.isfile(target):       # not a device or a pipe, which would never end
        raise ToolError(f"cannot read {target}: it is not a file" if os.path.exists(target) else f"cannot read {target}: no such file",
                        code="NOT_FOUND")
    if os.path.getsize(target) > MAX_GET_FILE_BYTES:
        raise ToolError(f"{target} is {os.path.getsize(target)} bytes; get_file sends at most {MAX_GET_FILE_BYTES} "
                        "in one message. Copy large files another way.", code="TOO_LARGE")
    try:
        with open(target, "rb") as stream:
            data = stream.read()
    except OSError as exc:
        raise ToolError(f"cannot read {target}: {exc}")
    # Encoded a piece at a time, so a large file never keeps the main loop waiting for the interpreter.
    step = 3 << 18
    encoded = "".join(base64.b64encode(data[start:start + step]).decode("ascii") for start in range(0, len(data), step))
    return {"path": target, "bytes": len(data), "content_b64": Base64(encoded)}


def cmd_stat_file(bridge, path: str) -> Dict[str, Any]:
    target = _path(path)
    if not os.path.isfile(target):
        note = {"note": "This path is a directory; stat_file reads files."} if os.path.isdir(target) else {}
        return {"path": target, "exists": False, **note}
    digest, size = hashlib.sha256(), 0
    with open(target, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
            size += len(block)
    return {"path": target, "exists": True, "bytes": size, "sha256": digest.hexdigest()}


# ---- identity ----------------------------------------------------------------------------

def _app_ready() -> Optional[bool]:
    try:
        from omni.kit.app import get_app

        ready = get_app().is_app_ready()
        return ready if isinstance(ready, bool) else None
    except Exception:  # noqa: BLE001 - outside Kit, or an app that cannot say
        return None


_HOSTNAME = socket.gethostname()
_SOURCE_TTL_S = 5.0
_source_cache: Dict[str, Any] = {"at": 0.0, "report": None}


def _source() -> Dict[str, Any]:
    """Which code this Kit runs. Read from git's files, so it is cached for a few seconds."""
    now = time.monotonic()
    if _source_cache["report"] is None or now - _source_cache["at"] > _SOURCE_TTL_S:
        try:
            from .source_state import report

            _source_cache["report"] = report()
        except Exception as exc:  # noqa: BLE001 - optional detail; status must still answer
            _source_cache["report"] = {"commit": "unknown", "dirty": None, "error": str(exc)[:200]}
        _source_cache["at"] = now
    return _source_cache["report"]


def cmd_status(bridge, detail: bool = True) -> Dict[str, Any]:
    """Which bridge answered: process, port, app, what is registered, what is running. ``detail`` is
    the MCP host's switch for a shorter reply; this verb always answers in full."""
    from .bridge import app_identity

    owners = bridge.get_owners()
    started = bridge._started_at
    return {
        **app_identity(),
        "pid": os.getpid(), "port": bridge._port, "bind_host": bridge._bind_host, "hostname": _HOSTNAME,
        "python": platform.python_version(), "app_ready": _app_ready(),
        "started": started, "uptime_s": round(time.time() - started, 1) if started else None,
        "namespaces": sorted(o["namespace"] for o in owners.values() if o["namespace"]),
        "tools": bridge.tool_count(),
        "busy": bridge.busy(),
        "jobs": bridge.jobs.summary(),
        "read_only": bridge.read_only,
        "auth": bridge._auth,
        "protocol": {"request_ids": True},
        "extension": _source(),
    }


def cmd_list_tools(bridge) -> Dict[str, Any]:
    return {"tools": bridge.get_registered_tools()}


def cmd_reload_tools(bridge, module: str) -> Dict[str, Any]:
    from .packages import reload_tool_module

    try:
        return reload_tool_module(bridge, module)
    except (Exception, SyntaxError) as exc:  # noqa: BLE001 - whatever the package's own code raises on import
        raise ToolError(f"cannot reload {module}: {type(exc).__name__}: {exc}. reload_tools takes a project's own tool package "
                        "(TWINSMITH_TOOL_MODULES). The bundled sim_tools and workcell reload through Kit's file "
                        "watcher when their files change; see `status`.")


def builtin_tools(bridge) -> List:
    """The definitions the bridge registers on itself. ``bridge`` may be None to read them only."""
    from .bridge import ToolDefinition

    def bound(function):
        return functools.partial(function, bridge)

    path = {"path": {"type": "string", "required": True, "description": "Path on the bridge's box (~ expands)."}}
    return [
        ToolDefinition(
            name="run_python", description=_RUN_PYTHON_DESCRIPTION, parameters=_RUN_PYTHON_PARAMETERS,
            # The code may advance the app from inside the call, so it runs alone.
            handler=bound(cmd_run_python), exclusive=True),
        ToolDefinition(
            name="reload_tools",
            description="Hot-reload one of a project's own tool packages: unregister it, import it again and "
                        "register its tools. Follow with refresh_tools so the new schemas are seen.",
            parameters={"module": {"type": "string", "required": True,
                                   "description": "The package's module name, as given in TWINSMITH_TOOL_MODULES."}},
            handler=bound(cmd_reload_tools)),
        ToolDefinition(
            name="put_file",
            description="Write a file on the simulator's machine (base64 content). Ships code and assets without "
                        "scp. Writing a .py clears the folder's bytecode cache. The content "
                        "travels in one message: for code and tool-sized assets, not gigabytes.",
            parameters={"path": {"type": "string", "required": True,
                                 "description": "Destination path on the bridge's box (~ expands)."},
                        "content_b64": {"type": "string", "required": True, "description": "File content, base64-encoded."},
                        "mkdirs": {"type": "boolean", "description": "Create missing parent directories (default true)."}},
            handler=bound(cmd_put_file), inline=True),
        ToolDefinition(
            name="get_file",
            description="Read a file from the simulator's machine. Returned base64-encoded (content_b64); the pull "
                        "half of put_file.",
            # The file tools touch only the disk, so they run on the caller's socket thread: a large
            # file is read, hashed or written without holding up a frame.
            parameters=path, handler=bound(cmd_get_file), read_only=True, inline=True),
        ToolDefinition(
            name="status",
            description="Where am I, in one call: the bridge that answered (pid, port, app, readiness), every "
                        "bridge advertised on this machine, the current stage's session_id and stage_id with "
                        "the timeline state, and running jobs. With nothing running it says so and names the "
                        "next call, instead of failing. Call it once before the first simulator action; there "
                        "is no need to repeat it between calls.",
            parameters={"detail": {"type": "boolean",
                                   "description": "true: the full bridge identity as well (namespaces, what "
                                                  "is running and waiting, extension source). "
                                                  "Default false: the short reply."}},
            handler=bound(cmd_status), read_only=True, inline=True),
        ToolDefinition(
            name="job_status", description=_JOB_STATUS_DESCRIPTION, parameters=_JOB_STATUS_PARAMETERS,
            handler=bound(cmd_job_status), read_only=True, inline=True),
        ToolDefinition(
            name="job_cancel",
            description="Stop a background job. job_id comes from the call that started it. It ends at its next step "
                        "and cleans up after itself; confirm with job_status. Through the MCP server it also "
                        "stops an Isaac Lab or script job.",
            parameters={"job_id": {"type": "string", "required": True, "description": "job_id of a running job."}},
            handler=bound(cmd_job_cancel)),
        ToolDefinition(
            name="stat_file",
            description="Check a file's existence, size and sha256 on the bridge's box. Skips unchanged pushes "
                        "and verifies assets.",
            parameters=path, handler=bound(cmd_stat_file), read_only=True, inline=True),
        ToolDefinition(name="list_tools", description="List all tools registered on this bridge.",
                       handler=bound(cmd_list_tools), internal=True, read_only=True, inline=True),
    ]
