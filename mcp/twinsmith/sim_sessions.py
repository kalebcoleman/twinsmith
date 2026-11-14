"""Opt-in, locally managed Isaac Sim launches with conservative concurrency checks.

Resource checks are snapshots, not a reservation or a guarantee against OOM. No
existing process is stopped automatically, including when resources later run low.
"""

from __future__ import annotations

import errno
import json
import os
import re
import signal
import socket
import stat
import subprocess
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Annotated, Callable, Collection, Literal, Iterable, Mapping

from pydantic import Field

try:
    import fcntl
except ImportError:  # Session launch is Linux-only; do not break unrelated host tools.
    fcntl = None

from mcp.server.fastmcp import FastMCP

from . import client, config
from .isaaclab import jobs as lab_jobs
from .local_tools import PYTHON_EXECUTION, ScopedTools
from .sim_versions import NEWTON_STACKS

_SESSION_ID = re.compile(r"^[0-9a-f]{32}$")
# An advertised bridge this server did not launch. Its ID is built from the listing, never stored.
_EXTERNAL_ID = re.compile(r"^external-(\d+)-(\d+)$")
_DEFAULT_ROOT = Path(__file__).resolve().parents[2]
# Linux 6.9 added a pidfd signal scope for an entire process group. Older
# kernels get a race-safe signal to the leader only.
_PIDFD_SIGNAL_PROCESS_GROUP = 1 << 2
# How far above a refused bridge port to look for a free one to suggest.
_PORT_SEARCH_SPAN = 64
# list_simulators shows this many exited sessions by default, newest first.
_RECENT_EXITED_LISTED = 3
# Where an automatic port search starts with no TWINSMITH_PORT_RANGE: the bridge's own default.
_DEFAULT_PORT = 8766
# A configured range such as 18800-18899 or 40-49.
_RANGE = re.compile(r"^(\d+)-(\d+)$")
# How often to retry a claim after removing one whose owner is gone.
_CLAIM_ATTEMPTS = 5


class _Refusal(ValueError):
    """A launch refused before starting anything; ``code`` is stable for clients."""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code


def _config_int(env: Mapping[str, str], name: str, default: int) -> int:
    raw = env.get(name, str(default))
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a non-negative integer") from exc
    if value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _sessions_dir(env: Mapping[str, str]) -> Path:
    value = env.get("TWINSMITH_SIM_SESSIONS_DIR")
    path = Path(value).expanduser() if value else Path.home() / ".local/share/twinsmith/sim-sessions"
    # The launcher runs in the simulator checkout, which may differ from the
    # server's working directory. Persist and pass the same paths to both.
    return path.resolve()


def _launch_range(env: Mapping[str, str], name: str, low: int, high: int) -> tuple[int, int] | None:
    """The ports or ROS domains launches may use, or None when the operator set no range.

    It comes only from the server's environment: on a shared machine the range is how an
    operator says which ports and domains are an agent's. A malformed value is an error, not
    something to ignore.
    """
    raw = (env.get(name) or "").strip()
    if not raw:
        return None
    match = _RANGE.fullmatch(raw)
    if not match:
        raise ValueError(f"{name} must be a range like {low}-{high}, not {raw!r}")
    first, last = int(match.group(1)), int(match.group(2))
    if not low <= first <= last <= high:
        raise ValueError(f"{name} must be an ascending range within {low}-{high}, not {raw!r}")
    return first, last


def _refuse_outside(value: int, allowed: tuple[int, int] | None, code: str, label: str,
                    setting: str, omission: str) -> None:
    if allowed and not allowed[0] <= value <= allowed[1]:
        raise _Refusal(code, f"{label} {value} is outside {setting} {allowed[0]}-{allowed[1]}. {omission}")


def _port_start(env: Mapping[str, str]) -> int:
    """Where an automatic port search begins with no range: the port this server was pinned to."""
    for name in ("TWINSMITH_PORT",):
        raw = (env.get(name) or "").strip()
        if raw:
            try:
                return int(raw)
            except ValueError as exc:
                raise ValueError(f"{name} must be an integer port, not {raw!r}") from exc
    return _DEFAULT_PORT


def _domain_start(env: Mapping[str, str]) -> int:
    """Where an automatic ROS domain search begins with no range: this server's own domain."""
    raw = (env.get("ROS_DOMAIN_ID") or "").strip()
    if not raw:
        return 0
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"ROS_DOMAIN_ID must be an integer, not {raw!r}") from exc


# ---- launch claims: one file per port and per ROS domain, in the bridge runtime folder ----
# A bind test alone cannot tell a port another agent was told to use, or one another launch is
# about to take. These claims are exclusive across host processes on this machine.


def _host_project_file():
    """The twinsmith.json this host resolves, or None (an unreadable or invalid one is not passed on)."""
    try:
        from . import pack_manifest

        return pack_manifest.project_file()
    except Exception:  # noqa: BLE001 - a launch must not fail because of a project file problem
        return None


def _claim_owner() -> dict:
    identity = _process_identity(os.getpid())
    if identity is None:
        raise RuntimeError("cannot read this process's identity to claim a port or ROS domain")
    return {"pid": os.getpid(), "start_ticks": identity[1], "boot_id": _boot_id()}


def _claim_folder() -> Path:
    folder = Path(client._runtime_dir())
    try:
        folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    except OSError as exc:
        raise RuntimeError(f"cannot create the bridge runtime folder {folder}: {exc}") from exc
    if not client._runtime_dir_trusted(str(folder)):
        raise RuntimeError(f"{folder} must belong to you and not be writable by group or others "
                           "before a launch can claim a port or ROS domain in it")
    return folder


def _read_claim(path: Path) -> dict | None:
    """A claim's contents, or None if it is gone. Never follows a symlink."""
    try:
        info = os.lstat(path)
    except FileNotFoundError:
        return None
    if not stat.S_ISREG(info.st_mode):
        raise RuntimeError(f"{path} is not a regular file; refusing to use it as a launch claim")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    except OSError as exc:
        raise RuntimeError(f"cannot read the launch claim {path}: {exc}") from exc
    try:
        with os.fdopen(fd, "rb") as stream:
            raw = stream.read(4096)
    except OSError as exc:
        raise RuntimeError(f"cannot read the launch claim {path}: {exc}") from exc
    try:
        data = json.loads(raw)
    except ValueError:
        return {}
    return data if isinstance(data, dict) else {}


def _claim_is_live(data) -> bool:
    """The claiming host is still the same process; a dead owner's claim is stale."""
    if not isinstance(data, dict):
        return False
    try:
        if data.get("boot_id") != _boot_id():
            return False
        pid, ticks = int(data["pid"]), int(data["start_ticks"])
    except (KeyError, TypeError, ValueError, RuntimeError):
        return False
    identity = _process_identity(pid)
    if identity and identity[0] != "Z" and identity[1] == ticks:
        return True
    if identity is None or identity[0] == "Z":
        # A launcher that didn't exec Kit can exit first. While a member of its group lives, the group ID
        # cannot be reused and the simulator still holds what the launch claimed.
        try:
            return bool(_group_members(pid))
        except OSError:
            return False
    return False


def _claimed(kind: str) -> set[int]:
    """Ports or ROS domains a live launch holds, best effort; a hint must not fail."""
    folder = Path(client._runtime_dir())
    if not client._runtime_dir_trusted(str(folder)):
        return set()
    pattern = re.compile(rf"^{kind}-(\d+)\.claim$")
    found = set()
    try:
        names = os.listdir(folder)
    except OSError:
        return found
    for name in names:
        match = pattern.fullmatch(name)
        if not match:
            continue
        try:
            if _claim_is_live(_read_claim(folder / name)):
                found.add(int(match.group(1)))
        except (OSError, RuntimeError, ValueError):
            continue
    return found


def _claim_fresh(path: Path, seconds: float = 5.0) -> bool:
    """True for a claim file created moments ago, whose owner may not have written it yet."""
    try:
        return time.time() - os.lstat(path).st_mtime < seconds
    except OSError:
        return False


@contextmanager
def _claim_lock(folder: Path):
    """Serialise claim creation and stale-claim removal across every host sharing the runtime folder, whatever
    its sessions folder: the inode check alone leaves a gap between looking and unlinking."""
    if fcntl is None:
        raise RuntimeError("managed simulator sessions require Linux")
    fd = os.open(folder / ".claims.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        os.close(fd)


def _claim(kind: str, value: int, owner: Mapping) -> Path:
    """Create this process's exclusive claim on one port or ROS domain."""
    folder = _claim_folder()
    with _claim_lock(folder):
        return _claim_locked(kind, value, owner, folder)


def _claim_locked(kind: str, value: int, owner: Mapping, folder: Path) -> Path:
    path = folder / f"{kind}-{value}.claim"
    payload = json.dumps(dict(owner)).encode()
    for _ in range(_CLAIM_ATTEMPTS):
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        except FileExistsError:
            # A planted symlink also lands here: O_EXCL refuses it, and _read_claim refuses to
            # follow it, so it is never treated as a claim or removed.
            try:
                before = os.lstat(path)
            except FileNotFoundError:
                continue
            data = _read_claim(path)
            if data is None:
                continue
            if _claim_is_live(data):
                raise _Refusal(f"{kind.upper()}_CLAIMED",
                               f"{kind} {value} is claimed by another launch (PID {data.get('pid')}). "
                               f"Omit {kind} to take the first free, unclaimed one, or choose another.")
            if not data.get("pid") and _claim_fresh(path):
                # Created, but its owner has not written the payload yet: refuse rather than
                # delete a claim that is being made right now.
                raise _Refusal(f"{kind.upper()}_CLAIMED",
                               f"{kind} {value} is being claimed by another launch right now. Retry.")
            try:
                if os.lstat(path).st_ino != before.st_ino:  # replaced since it was read
                    continue
                os.unlink(path)
            except FileNotFoundError:
                pass
            except OSError as exc:
                raise RuntimeError(f"cannot remove the stale launch claim {path}: {exc}") from exc
            continue
        except OSError as exc:
            raise RuntimeError(f"cannot claim {kind} {value} in {folder}: {exc}") from exc
        try:
            os.write(fd, payload)
        except OSError as exc:
            os.close(fd)
            try:
                os.unlink(path)
            except OSError:
                pass
            raise RuntimeError(f"cannot write the launch claim {path}: {exc}") from exc
        os.close(fd)
        return path
    raise RuntimeError(f"cannot claim {kind} {value} in {folder}: another process keeps replacing the claim")


def _release_claim(path: Path, owner: Mapping) -> None:
    """Remove this process's claim, and leave a claim another process made alone."""
    try:
        data = _read_claim(path)
    except RuntimeError:
        return
    if not isinstance(data, dict) or data.get("pid") != owner["pid"] or data.get("start_ticks") != owner["start_ticks"]:
        return
    try:
        os.unlink(path)
    except OSError:
        pass


@contextmanager
def _claims():
    """Yield a claim(kind, value) function; every claim is released on the way out."""
    owner = _claim_owner()
    held: list[Path] = []

    def take(kind: str, value: int) -> Path:
        path = _claim(kind, value, owner)
        held.append(path)
        return path

    def handover(new_owner: Mapping) -> None:
        """The simulator now holds what the launch claimed, until it exits: a host that returns while Kit is
        still starting must not free the port and domain before the bridge advertises them."""
        for path in list(held):
            tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
            try:
                fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
                with os.fdopen(fd, "wb") as stream:
                    stream.write(json.dumps(dict(new_owner)).encode())
                os.replace(tmp, path)
                held.remove(path)
            except OSError:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
        # A claim that could not be rewritten still names this host, which outlives the launch: it is kept
        # (not released) so a failed handover refuses a second launch instead of freeing a live simulator's port.
        held.clear()

    take.handover = handover
    try:
        yield take
    finally:
        for path in held:
            _release_claim(path, owner)


@contextmanager
def _launch_lock(base: Path):
    if fcntl is None:
        raise RuntimeError("managed simulator sessions require Linux")
    base.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (base / ".lock").open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def _process_identity(pid: int) -> tuple[str, int] | None:
    """Linux state and start tick, to avoid signalling a reused PID."""
    try:
        data = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return data[0], int(data[19])
    except (OSError, IndexError, ValueError):
        return None


def _boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError as exc:
        raise RuntimeError("cannot verify the system boot identity") from exc


def _alive(session: dict) -> bool:
    if session.get("boot_id") != _boot_id():
        return False
    identity = _process_identity(session.get("pid", 0))
    return bool(identity and identity[0] != "Z" and identity[1] == session.get("start_ticks"))


def _public(session: dict) -> dict:
    """Keep boot and kernel identity tokens internal to session management."""
    public = {key: value for key, value in session.items() if key not in {"boot_id", "start_ticks"}}
    if "id" in public:
        public["session_id"] = public["id"]  # the argument name simulator_status and stop_simulator take
    return public


def _group_members(leader: int) -> list[int]:
    """Other processes in this leader's group, from /proc."""
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit() or int(name) == leader:
            continue
        try:
            with open(f"/proc/{name}/stat", "rb") as stream:
                fields = stream.read().rsplit(b")", 1)[1].split()
        except (OSError, IndexError):
            continue
        if len(fields) > 2 and fields[2] == str(leader).encode() and fields[0] != b"Z":  # a zombie holds nothing
            found.append(int(name))
    return found


def _signal_group_members(leader: int, stop_signal: signal.Signals) -> None:
    """Signal the group's other members while its live leader keeps the group ID from being reused."""
    for member in _group_members(leader):
        try:
            fd = os.pidfd_open(member, 0)
        except OSError:
            continue
        try:
            if os.getpgid(member) == leader:
                signal.pidfd_send_signal(fd, stop_signal)
        except OSError:
            pass
        finally:
            os.close(fd)


def _in_session_group(pid, leader: int) -> bool:
    """The bridge can be a child of the process a launch recorded, when its launcher didn't exec Kit."""
    try:
        return isinstance(pid, int) and os.getpgid(pid) == leader
    except OSError:
        return False


def _signal_session(session: dict, stop_signal: signal.Signals) -> str:
    """Signal the verified process through a pidfd so PID reuse cannot redirect it."""
    pid = session["pid"]
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise RuntimeError("safe stop requires Linux pidfd support")
    fd = os.pidfd_open(pid, 0)
    try:
        # Check after opening: a recycled PID will fail the stored boot/start identity,
        # while a pidfd for a subsequently exited process cannot refer to its replacement.
        if not _alive(session):
            raise ProcessLookupError("managed process exited or its identity changed")
        if os.getpgid(pid) != pid:
            raise RuntimeError("process group changed; refusing to signal the session")
        try:
            signal.pidfd_send_signal(fd, stop_signal, None, _PIDFD_SIGNAL_PROCESS_GROUP)
            return "process_group"
        except OSError as exc:
            if exc.errno != errno.EINVAL:
                raise
            # Linux before 6.9 cannot signal a process group by pidfd. Do not fall back to killpg, which
            # could target a reused group ID. Members first, each by its own pidfd: a launcher that didn't
            # exec Kit, such as a build's python.sh, would otherwise die and leave Kit running.
            _signal_group_members(pid, stop_signal)
            signal.pidfd_send_signal(fd, stop_signal)
            return "process"
    finally:
        os.close(fd)


def _read(base: Path, session_id: str) -> dict:
    if not isinstance(session_id, str) or not _SESSION_ID.fullmatch(session_id):
        raise ValueError("invalid simulator session ID")
    try:
        data = json.loads((base / session_id / "session.json").read_text())
    except (OSError, ValueError) as exc:
        raise ValueError("simulator session not found or its metadata is unreadable") from exc
    if data.get("id") != session_id:
        raise ValueError("simulator session metadata does not match its ID")
    return data


def _sessions(base: Path) -> list[dict]:
    if not base.is_dir():
        return []
    result = []
    for path in base.iterdir():
        if not path.is_dir() or not _SESSION_ID.fullmatch(path.name):
            continue
        try:
            result.append(_read(base, path.name))
        except ValueError:
            continue
    return result


def _lab_pids(env: Mapping[str, str]) -> set[int]:
    """Count running Lab jobs in the configured jobs directory, without reconciling records."""
    configured = env.get("TWINSMITH_JOBS_DIR") or os.environ.get("TWINSMITH_JOBS_DIR")
    base = Path(configured).expanduser() if configured else Path.home() / ".local/share/twinsmith/jobs"
    if not base.is_dir():
        return set()
    found = set()
    for path in base.iterdir():
        if not path.is_dir():
            continue
        data = lab_jobs._read_job(path)
        if data and data.get("status") == "running" and lab_jobs._job_alive(data):
            found.add(int(data["pid"]))
    return found


def _domain_of(pid: int) -> int:
    try:
        entries = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
        for entry in entries:
            if entry.startswith(b"ROS_DOMAIN_ID="):
                return int(entry.split(b"=", 1)[1])
        return 0  # ROS 2 default when unset
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"cannot determine ROS domain of bridge process {pid}") from exc


def _resources() -> dict[str, int]:
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.total,memory.free", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, check=True, timeout=5,
        )
        lines = [line for line in result.stdout.splitlines() if line.strip()]
        if len(lines) != 1:
            raise RuntimeError("parallel launch currently requires exactly one NVIDIA GPU")
        total, free = (int(value.strip()) for value in lines[0].split(","))
        meminfo = Path("/proc/meminfo").read_text().splitlines()
        available_kib = next(int(line.split()[1]) for line in meminfo if line.startswith("MemAvailable:"))
        return {"gpu_total_mib": total, "gpu_free_mib": free, "ram_available_mib": available_kib // 1024}
    except (OSError, subprocess.SubprocessError, ValueError, StopIteration) as exc:
        raise RuntimeError("cannot check NVIDIA GPU and system memory; install nvidia-smi and use Linux") from exc


def _port_bind_error(port: int) -> OSError | None:
    """Try the bridge's own bind sequence (SO_REUSEADDR, bind, listen; bridge.py) on the managed
    bind address, and return what stopped it.

    A plain bind is the wrong test: after the previous bridge's process exits, its closed
    connections keep the port in TIME_WAIT for about a minute, a plain bind fails with EADDRINUSE,
    and yet the bridge could bind it. SO_REUSEADDR does not get past a socket in LISTEN."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
            sock.listen(1)
    except OSError as exc:
        return exc
    return None


def _next_free_port_hint(port: int, reserved: Collection[int] = (),
                         allowed: tuple[int, int] | None = None) -> str:
    """Suggest a port a bridge could bind, no live launch claims and no live bridge advertises.

    Still a snapshot, not a reservation: without a range another agent on this machine may have
    been told to use the port, so the message says how to claim a range.
    """
    if allowed:
        first, last = allowed
    else:
        first, last = port + 1, min(port + _PORT_SEARCH_SPAN, 65535)
    claimed = _claimed("port")
    for candidate in range(first, last + 1):
        if candidate in reserved or candidate in claimed or _port_bind_error(candidate) is not None:
            continue
        if allowed:
            return (f" Port {candidate} is free and unclaimed in TWINSMITH_PORT_RANGE {first}-{last}: "
                    f"retry with port={candidate}.")
        return (f" On a shared machine another agent may have been told to use it: set "
                "TWINSMITH_PORT_RANGE=18800-18899 in this server's environment so launches claim "
                f"ports in a range. Port {candidate} is free right now: retry with port={candidate}.")
    if first > last:
        return f" No free port found at or above {port}; choose another port."
    if allowed:
        return (f" No free, unclaimed port found in {first}-{last}; stop a session or widen "
                "TWINSMITH_PORT_RANGE.")
    return f" No free port found in {first}-{last}; choose another port."


def _check_port(port: int, reserved: Collection[int] = (),
                allowed: tuple[int, int] | None = None) -> None:
    error = _port_bind_error(port)
    if error is not None:
        raise ValueError(f"bridge port {port} is already in use or cannot be bound ({error}). "
                         "A port that only has recently closed bridge connections does not cause "
                         "this; another process holds it, or holds connections closed without "
                         "SO_REUSEADDR (those clear within about a minute)."
                         + _next_free_port_hint(port, reserved, allowed)) from error


def _pick_port(claim: Callable[[str, int], Path], port_range: tuple[int, int] | None, start: int,
               reserved: Collection[int]) -> int:
    """The first port in range that is free, unclaimed and not advertised by a live bridge."""
    first, last = port_range or (start, min(start + _PORT_SEARCH_SPAN, 65535))
    for candidate in range(first, last + 1):
        if candidate in reserved or _port_bind_error(candidate) is not None:
            continue
        try:
            claim("port", candidate)
        except _Refusal:  # another launch took it between the bind test and the claim
            continue
        return candidate
    raise _Refusal("PORT_RANGE_EXHAUSTED",
                   f"no free, unclaimed port in {first}-{last}. "
                   + ("Stop a session or widen TWINSMITH_PORT_RANGE."
                      if port_range else "Pass port=... explicitly, or set TWINSMITH_PORT_RANGE."))


def _pick_domain(claim: Callable[[str, int], Path], domain_range: tuple[int, int] | None, start: int,
                 taken: Collection[int]) -> int:
    """The first ROS domain in range that no known process uses and no launch claims."""
    first, last = domain_range or (start, 231)
    for candidate in range(first, last + 1):
        if candidate in taken:
            continue
        try:
            claim("ros_domain", candidate)
        except _Refusal:  # another launch took it
            continue
        return candidate
    raise _Refusal("ROS_DOMAIN_RANGE_EXHAUSTED",
                   f"no free, unclaimed ROS domain in {first}-{last}. "
                   + ("Stop a session or widen TWINSMITH_ROS_DOMAIN_RANGE."
                      if domain_range else "Pass ros_domain=... explicitly, or set TWINSMITH_ROS_DOMAIN_RANGE."))


def _known_domains(active: Iterable[dict], bridges: Iterable[Mapping], lab_pids: Collection[int]) -> dict[int, str]:
    """ROS domains known processes use, with what holds each."""
    found: dict[int, str] = {}
    for item in active:
        found.setdefault(int(item["ros_domain"]), "a managed session")
    for bridge in bridges:
        found.setdefault(_domain_of(int(bridge["pid"])), f"bridge PID {bridge['pid']}")
    for pid in lab_pids:
        found.setdefault(_domain_of(pid), f"Lab job PID {pid}")
    return found


def _domain_refusal(ros_domain: int, holder: str) -> ValueError:
    """The ROS domain guard's refusal, with the ways to proceed."""
    return ValueError(f"ROS domain {ros_domain} is already used by {holder}. To proceed, pass another "
                      "ros_domain and export the same ROS_DOMAIN_ID in every ROS shell that talks to "
                      "this simulator, or pass allow_shared_domain=true if sharing the domain is "
                      "intended. Lab instructions that assume domain 0 need adapting to the domain "
                      "you choose.")


def _configured_launcher(env: Mapping[str, str]) -> Path | None:
    """The launcher named by TWINSMITH_SIM_LAUNCHER, or None for this repository's run.sh.

    It comes only from the server's environment, which the MCP client configuration sets. A tool
    argument or a project file could be written by the model (put_file has no path limits), and a
    launcher is an arbitrary command. It is checked again before each launch. A launcher gets the
    same arguments as run.sh and must exec Kit, so that the process it started is the bridge."""
    value = (env.get("TWINSMITH_SIM_LAUNCHER") or "").strip()
    if not value:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise RuntimeError(f"TWINSMITH_SIM_LAUNCHER must be an absolute path, not {value!r}")
    try:
        info = path.stat()
    except OSError as exc:
        raise RuntimeError(f"TWINSMITH_SIM_LAUNCHER {value!r} cannot be read: {exc}") from exc
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f"TWINSMITH_SIM_LAUNCHER {value!r} is not an executable file")
    if info.st_uid not in (os.getuid(), 0) or info.st_mode & 0o022:
        raise RuntimeError(f"TWINSMITH_SIM_LAUNCHER {value!r} must belong to you (or root) and not be "
                           "writable by group or others")
    return path


def _interpreter_release(env: Mapping[str, str], root: Path) -> str | None:
    """The Isaac Sim release of the interpreter run.sh will use, read from its package metadata without
    running it. None when it cannot be told, e.g. when run.sh has to auto-detect or for a standalone install."""
    named = env.get("TWINSMITH_SIM_PYTHON")
    python = Path(named).expanduser() if named else root / ".venv" / "bin" / "python"
    for prefix in (python.parent.parent, python.resolve().parent.parent):
        found = sorted(prefix.glob("lib/python3.*/site-packages/isaacsim-*.dist-info"))
        if found:
            return found[-1].name[len("isaacsim-"):-len(".dist-info")]
    return None


def _launch(env: Mapping[str, str], port: int | None = None, ros_domain: int | None = None,
            headless: bool = False, allow_shared_domain: bool = False,
            enable_motion_bvh: bool = False,
            physics_backend: Literal["physx", "newton"] | None = None) -> dict:
    # No argument: the configured default (environment, project file, user file, then the built-in one).
    # Either way the engine is named to the new process, so a changed default never changes a request.
    source = "argument"
    if physics_backend is None:
        physics_backend, source = config.resolve_physics_backend(env)
    if physics_backend not in config.PHYSICS_BACKENDS:
        raise ValueError(f"physics_backend must be {' or '.join(config.PHYSICS_BACKENDS)}")
    port_range = _launch_range(env, "TWINSMITH_PORT_RANGE", 1024, 65535)
    domain_range = _launch_range(env, "TWINSMITH_ROS_DOMAIN_RANGE", 0, 231)
    auto_selected = [name for name, value in (("port", port), ("ros_domain", ros_domain)) if value is None]
    if port is not None:
        if not 1024 <= port <= 65535:
            raise ValueError("port must be between 1024 and 65535")
        _refuse_outside(port, port_range, "PORT_OUT_OF_RANGE", "port", "TWINSMITH_PORT_RANGE",
                        "Omit port to take the first free, unclaimed port in the range.")
    # The bundled Cyclone profile probes participant indices 0..100. At
    # domain 232 those discovery ports extend beyond UDP's 65535 limit.
    if ros_domain is not None:
        if not 0 <= ros_domain <= 231:
            raise ValueError("ros_domain must be between 0 and 231 for the bundled Cyclone DDS profile")
        _refuse_outside(ros_domain, domain_range, "ROS_DOMAIN_OUT_OF_RANGE", "ROS domain",
                        "TWINSMITH_ROS_DOMAIN_RANGE",
                        "Omit ros_domain to take the first free, unclaimed domain in the range.")

    custom = _configured_launcher(env)
    if custom is not None:
        launcher, root = custom, custom.parent
    else:
        root = Path(env.get("TWINSMITH_SIM_ROOT", str(_DEFAULT_ROOT))).expanduser().resolve()
        launcher = root / "run.sh"
        if not launcher.is_file() or not os.access(launcher, os.X_OK):
            raise RuntimeError("Isaac Sim launcher not found; set TWINSMITH_SIM_ROOT to the repository root")
        release = _interpreter_release(env, root)
        if physics_backend == "newton" and release and release not in NEWTON_STACKS:
            raise ValueError(f"Isaac Sim {release} has no Newton backend here (the selected interpreter's package "
                             "metadata; see docs/COMPATIBILITY.md). Nothing was started and there is no fallback: "
                             "use physics_backend=physx, or a 6.x interpreter for the Newton workflow.")
    min_gpu = _config_int(env, "TWINSMITH_SIM_MIN_FREE_VRAM_MIB", 8192)
    min_ram = _config_int(env, "TWINSMITH_SIM_MIN_FREE_RAM_MIB", 4096)
    max_instances = _config_int(env, "TWINSMITH_SIM_MAX_INSTANCES", 2)
    boot_id = _boot_id()
    base = _sessions_dir(env)
    with _launch_lock(base), _claims() as claim:
        active = [item for item in _sessions(base) if _alive(item)]
        bridges = client.discover_bridges()
        lab_pids = _lab_pids(env)
        existing_pids = ({int(b["pid"]) for b in bridges} | {int(s["pid"]) for s in active}
                         | lab_pids)
        if max_instances and len(existing_pids) >= max_instances:
            raise RuntimeError(f"{len(existing_pids)} known simulator/Lab process(es) are already active; "
                               f"TWINSMITH_SIM_MAX_INSTANCES is {max_instances}")
        reserved = {int(item["port"]) for item in active} | {int(b["port"]) for b in bridges}
        if port is None:
            port = _pick_port(claim, port_range, _port_start(env), reserved)
        else:
            claim("port", port)
            if any(item["port"] == port for item in active):
                raise ValueError(f"port {port} is reserved by a starting or running managed session."
                                 + _next_free_port_hint(port, reserved, port_range))
            _check_port(port, reserved, port_range)
        if ros_domain is None:
            taken = _known_domains(active, bridges, lab_pids)
            ros_domain = _pick_domain(claim, domain_range, _domain_start(env), set(taken))
        elif not allow_shared_domain:
            holder = _known_domains(active, bridges, lab_pids).get(ros_domain)
            if holder:
                raise _domain_refusal(ros_domain, holder)
            claim("ros_domain", ros_domain)
        resources = _resources()
        if resources["gpu_free_mib"] < min_gpu or resources["ram_available_mib"] < min_ram:
            raise RuntimeError(f"insufficient headroom: {resources}; require at least "
                               f"{min_gpu} MiB GPU and {min_ram} MiB system RAM free")
        session_id = uuid.uuid4().hex
        directory = base / session_id
        directory.mkdir(mode=0o700)
        log = directory / "sim.log"
        output = directory / "output"
        output.mkdir(mode=0o700)
        child_env = os.environ.copy()
        for key in ("PYTHONPATH", "TWINSMITH_HEADLESS", "TWINSMITH_SIM_PYTHON",
                    "TWINSMITH_WORKSPACE"):
            child_env.pop(key, None)
        child_env.update({"TWINSMITH_PORT": str(port),
                          "TWINSMITH_BIND": "127.0.0.1", "ROS_DOMAIN_ID": str(ros_domain),
                          "TWINSMITH_PHYSICS_BACKEND": physics_backend,
                          "MCP_CLIENT_OUTPUT_DIR": str(output),
                          "TWINSMITH_SCENE_OUTPUT": child_env.get("TWINSMITH_SCENE_OUTPUT") or str(output),
                          "TWINSMITH_SIM_SESSION_ID": session_id,
                          "TWINSMITH_SIM_SESSION_DIR": str(directory)})
        for key in ("TWINSMITH_SIM_PYTHON", "TWINSMITH_WORKSPACE"):
            if env.get(key):
                child_env[key] = env[key]
        # The project file this host sees is the one Kit must see: Kit starts in another folder, so without this it would
        # never read the project's pack, and the same call would resolve differently in host and Kit.
        project = _host_project_file()
        if project is not None:
            child_env.setdefault("TWINSMITH_PROJECT", str(project))
        user_config = directory / "user.config.json"
        args = ([str(launcher), "--port", str(port)] + (["--headless"] if headless else [])
                + (["--motion-bvh"] if enable_motion_bvh else [])
                + ["--", f"--/app/userConfigPath={user_config}"])
        with log.open("wb") as stream:
            proc = subprocess.Popen(args, cwd=str(root), env=child_env, stdin=subprocess.DEVNULL,
                                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        identity = _process_identity(proc.pid)
        if not identity:
            # A failed /proc read does not prove the child has exited. Never wait
            # indefinitely or leave an untracked simulator running.
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
            raise RuntimeError(f"could not record simulator process identity; stopped PID {proc.pid}; see {log}")
        record = {"id": session_id, "pid": proc.pid, "start_ticks": identity[1], "boot_id": boot_id, "port": port,
                  "ros_domain": ros_domain, "headless": headless,
                  "enable_motion_bvh": enable_motion_bvh,
                  "physics_backend": physics_backend, "physics_backend_source": source,
                  **({"launcher": str(custom)} if custom is not None else {}),
                  **({"auto_selected": auto_selected} if auto_selected else {}),
                  "started_at": time.time(),
                  "log": str(log), "output_dir": str(output), "user_config": str(user_config),
                  "initial_resources": resources}
        try:
            (directory / "session.json").write_text(json.dumps(record, indent=2))
        except OSError as exc:
            # Do not leave an untracked process if persisting its identity fails.
            try:
                _signal_session(record, signal.SIGKILL)
            except (OSError, RuntimeError) as cleanup:
                raise RuntimeError(f"could not save session metadata or stop PID {proc.pid}; "
                                   f"inspect {log}: {cleanup}") from exc
            raise
        threading.Thread(target=proc.wait, daemon=True).start()
        claim.handover({"pid": proc.pid, "start_ticks": identity[1], "boot_id": boot_id})
        return {**_public(record), "managed": True, "state": "starting",
                # Inherited, and deliberately not an argument: a pack in the asset root imports without approval.
                "asset_root": os.environ.get("TWINSMITH_ASSET_ROOT") or "default: <workspace>/assets; set TWINSMITH_ASSET_ROOT in the MCP server's environment",
                "note": (f"Use simulator_status to confirm bridge readiness, then use_bridge(port={port}) "
                         "to send tool calls to this simulator. If this is the only managed session "
                         "and the configured port does not answer, refresh_tools(wait_s=120) waits "
                         "for this session and selects it instead.")}


def _external_id(bridge: Mapping) -> str:
    return f"external-{int(bridge['pid'])}-{int(bridge['port'])}"


def _external_bridges(sessions: Iterable[dict]) -> list[dict]:
    """Advertised bridges no live managed session accounts for: something else launched them.
    This reads their portfiles and only tests that the port accepts a connection."""
    managed = {int(item["pid"]) for item in sessions if _alive(item)}
    managed |= {member for leader in list(managed) for member in _group_members(leader)}
    return [{"id": _external_id(bridge), "managed": False, "pid": int(bridge["pid"]), "port": int(bridge["port"]),
             "host": bridge.get("host"), "app": bridge.get("app"), "app_version": bridge.get("app_version"),
             "headless": bridge.get("headless"), "started_at": bridge.get("started"), "state": "active"}
            for bridge in client.discover_bridges() if int(bridge["pid"]) not in managed]


def _bridge_state(host: str | None, port: int, pid: int) -> tuple[str, dict]:
    """Readiness from the bridge's own ``status``, checked against the PID expected on that port."""
    try:
        response = client.call("status", host=host or "127.0.0.1", port=port, timeout=2)
    except client.BridgeError:  # it answered, but would not serve this client (its token was refused)
        return "busy", {}
    except (OSError, RuntimeError):
        return "starting_or_unresponsive", {}
    if response.get("pid") != pid and not _in_session_group(response.get("pid"), pid):
        return "port_conflict", response
    if response.get("app_ready") is True:
        return "running", response
    # Older or non-Kit bridges cannot attest to application readiness.
    return ("starting" if response.get("app_ready") is False else "readiness_unknown"), response


def _external_status(session_id: str) -> dict:
    match = _EXTERNAL_ID.fullmatch(session_id)
    pid, port = int(match.group(1)), int(match.group(2))
    bridge = next((b for b in client.discover_bridges() if _external_id(b) == session_id), None)
    if bridge is None:
        raise ValueError(f"no advertised bridge has PID {pid} and port {port}; it may have exited or restarted. "
                         "Call list_simulators.")
    state, response = _bridge_state(bridge.get("host"), port, pid)
    result = {"id": session_id, "managed": False, "pid": pid, "port": port, "host": bridge.get("host"),
              "app": response.get("app") or bridge.get("app"),
              "app_version": response.get("app_version") or bridge.get("app_version"),
              "headless": bridge.get("headless"), "started_at": bridge.get("started"), "state": state,
              "stop": "stop_simulator does not stop this: it was started outside twinsmith, so stop it where it was started."}
    if state in ("running", "readiness_unknown"):
        result["next"] = f"use_bridge(port={port}) sends tool calls to this simulator, after you confirm it is the one you mean."
    return result


def _newton_readiness(port: int) -> dict:
    """For a session launched with physics_backend=newton: is Newton the active engine, and is its tensor
    extension on (releases after 6.0.0.0 need it for every runtime read)? Read from the running Kit."""
    try:
        runtime = client.call("sim.stage.info", host="127.0.0.1", port=port, timeout=5).get("runtime") or {}
    except (client.BridgeError, OSError, RuntimeError) as exc:
        return {"ready": None, "note": f"Could not read the engine from Kit yet ({str(exc)[:160]}); call simulator_status again."}
    engine = runtime.get("physics_engine")
    extensions = runtime.get("extensions") or {}
    tensors = (extensions.get("isaacsim.physics.newton.tensors") or {}).get("enabled")
    needs_tensors = runtime.get("release") != "6.0.0.0"
    ready = engine == "newton" and (bool(tensors) or not needs_tensors)
    out = {"ready": ready, "engine": engine, "tensor_extension_enabled": bool(tensors)}
    if not ready:
        out["note"] = ("Newton is not ready in this session: " + ("the active engine is " + repr(engine) if engine != "newton"
                       else "isaacsim.physics.newton.tensors is off, so body and joint reads will fail")
                       + ". Do not start a campaign on it; stop it and launch again, and read sim__logs__tail for the cause.")
    return out


def _status(env: Mapping[str, str], session_id: str) -> dict:
    if isinstance(session_id, str) and _EXTERNAL_ID.fullmatch(session_id):
        return _external_status(session_id)
    session = _read(_sessions_dir(env), session_id)
    if not _alive(session):
        return {**_public(session), "managed": True, "state": "exited"}
    state, answer = _bridge_state("127.0.0.1", session["port"], session["pid"])
    try:
        resources = _resources()
    except RuntimeError as exc:
        resources = {"error": str(exc)}
    result = {**_public(session), "managed": True, "state": state, "current_resources": resources,
              "app": answer.get("app"), "app_version": answer.get("app_version")}
    if state == "port_conflict" and session.get("launcher"):
        result["note"] = ("Another PID answers this session's port. A custom launcher must exec Kit rather than "
                          "fork it, or the process recorded here is not the simulator.")
    if state == "running" and session.get("physics_backend") == "newton":
        result["physics"] = _newton_readiness(session["port"])
    if state in ("running", "readiness_unknown"):
        # The MCP server keeps dialling its configured port (TWINSMITH_PORT) while it
        # answers, and refresh_tools then does not switch.
        result["next"] = (f"if this MCP server is not already using port {session['port']}, call "
                          f"use_bridge(port={session['port']}) so sim__* and run_python go to this simulator; "
                          "refresh_tools alone stays on the configured port while that port answers.")
    return result


def _stop(env: Mapping[str, str], session_id: str, expected_pid: int, confirm_discard: bool,
          force: bool = False) -> dict:
    if isinstance(session_id, str) and _EXTERNAL_ID.fullmatch(session_id):
        raise ValueError(f"{session_id} is not a managed session: twinsmith did not start it, so stop_simulator will not "
                         "stop it. Stop it where it was started.")
    if not confirm_discard:
        force_arg = ", force=true" if force else ""
        raise ValueError(f"Stopping can discard unsaved scene changes. After saving, retry: "
                         f"stop_simulator(session_id={session_id!r}, expected_pid={expected_pid!r}, confirm_discard=true{force_arg}).")
    base = _sessions_dir(env)
    with _launch_lock(base):
        session = _read(base, session_id)
        if session.get("pid") != expected_pid:
            raise ValueError("expected PID does not match the managed session")
        if not _alive(session):
            return {"id": session_id, "pid": expected_pid, "state": "exited"}
        stop_signal = signal.SIGKILL if force else signal.SIGINT
        try:
            scope = _signal_session(session, stop_signal)
        except ProcessLookupError:
            return {"id": session_id, "pid": expected_pid, "state": "exited"}
    return {"id": session_id, "pid": expected_pid, "state": "stop_requested",
            "signal": stop_signal.name, "scope": scope}


def register_sim_session_tools(server: FastMCP, groups: Iterable[str] | None = None,
                               env: Mapping[str, str] | None = None,
                               on_launch: Callable[[dict], None] | None = None) -> None:
    """``on_launch`` receives each record launch_simulator creates, so the host can tell the sessions it
    launched from others in the shared session directory."""
    mcp = ScopedTools(server, groups)
    settings = dict(os.environ if env is None else env)

    @mcp.tool(name="launch_simulator", annotations=PYTHON_EXECUTION)
    def sim_launch(port: Annotated[int | None, Field(description="Bridge TCP port 1024-65535. Omit for the first free, unclaimed port in TWINSMITH_PORT_RANGE, or upward from the configured port.")] = None,
                   ros_domain: Annotated[int | None, Field(description="ROS_DOMAIN_ID 0-231. Omit for the first free, unclaimed domain in TWINSMITH_ROS_DOMAIN_RANGE, or upward from this server's domain.")] = None,
                   headless: Annotated[bool, Field(description="true: no window.")] = False,
                   allow_shared_domain: Annotated[bool, Field(description="true: allow a ROS domain another session already uses.")] = False,
                   enable_motion_bvh: Annotated[bool, Field(description="true: accurate RTX LiDAR motion compensation; costs rendering time and GPU memory.")] = False,
                   physics_backend: Annotated[Literal["physx", "newton"] | None, Field(description="Engine for this new process. Omit for the configured default (status shows it). An engine that cannot be given starts nothing: no fallback.")] = None) -> dict:
        """Launch a separate managed Isaac Sim instance, when explicitly asked.

        Omit port and ros_domain to take the first free, unclaimed value inside the configured range;
        the result says what was chosen, and a value outside its range is refused. Checks an instance
        limit, the port, known ROS domains and free resources; these are snapshots, not reservations.
        Returns before the bridge is ready: poll simulator_status, then use_bridge.
        Launching runs configured local code: run.sh, or the launcher in the server's
        TWINSMITH_SIM_LAUNCHER setting. No tool argument can name a command.
        """
        record = _launch(settings, port, ros_domain, headless, allow_shared_domain, enable_motion_bvh, physics_backend)
        if on_launch is not None:
            on_launch(record)
        return record

    @mcp.tool(name="list_simulators", annotations=PYTHON_EXECUTION)
    def sim_list(include_exited: Annotated[bool, Field(description="List every exited session, not only the most recent few.")] = False) -> dict:
        """List simulators: managed sessions (managed=true), including those started before this server
        restarted, and advertised bridges started some other way (managed=false; PID, port, app, version).

        Use each entry's id with simulator_status. stop_simulator refuses managed=false entries.
        Exited sessions are kept on disk with their logs, and only the most recent few are
        listed unless include_exited=true.
        """
        sessions = _sessions(_sessions_dir(settings))
        live = [session for session in sessions if _alive(session)]
        exited = sorted((session for session in sessions if not _alive(session)),
                        key=lambda session: session.get("started_at") or 0, reverse=True)
        shown = exited if include_exited else exited[:_RECENT_EXITED_LISTED]
        result = {"sessions": ([{**_public(session), "managed": True, "state": "active"} for session in live]
                               + [{**_public(session), "managed": True, "state": "exited"} for session in shown]
                               + _external_bridges(sessions))}
        if len(shown) < len(exited):
            result["exited_omitted"] = len(exited) - len(shown)
            result["note"] = (f"{result['exited_omitted']} older exited session(s) not listed; "
                              "include_exited=true lists them. Their records and logs stay on disk.")
        return result

    @mcp.tool(name="simulator_status", annotations=PYTHON_EXECUTION)
    def sim_status(session_id: Annotated[str, Field(description="id from list_simulators or launch_simulator.")]) -> dict:
        """Report a simulator's process and Kit readiness, by an id from list_simulators.

        A managed session also reports current resources. An external one (id external-<pid>-<port>)
        reports its live PID, port, app and version; the PID must match its advertisement.
        """
        return _status(settings, session_id)

    @mcp.tool(name="stop_simulator", annotations=PYTHON_EXECUTION)
    def sim_stop(session_id: Annotated[str, Field(description="id from list_simulators or launch_simulator.")],
                 expected_pid: Annotated[int, Field(description="PID from simulator_status; a different process is refused.")],
                 confirm_discard: Annotated[bool, Field(description="true: accept losing unsaved changes.")] = False,
                 force: Annotated[bool, Field(description="true: SIGKILL its process group; only if normal shutdown failed.")] = False) -> dict:
        """Stop only the identified managed process; unsaved changes can be lost. Refuses external bridges.

        Use force=true only if normal shutdown failed: it sends SIGKILL to the process group.
        """
        return _stop(settings, session_id, expected_pid, confirm_discard, force)


_ENDED_SESSIONS: set = set()    # (folder, id) of sessions seen dead: a session's process never comes back


def live_sessions(env: Mapping[str, str] | None = None) -> list[dict]:
    """``{id, port, pid, started_at}`` of managed simulator sessions that are still alive (best effort).
    Called on every status check-in, so a session seen dead is not read from disk again."""
    try:
        base = _sessions_dir(os.environ if env is None else env)
        live = []
        for path in (base.iterdir() if base.is_dir() else ()):
            key = (str(base), path.name)
            if key in _ENDED_SESSIONS or not _SESSION_ID.fullmatch(path.name) or not path.is_dir():
                continue
            try:
                x = _read(base, path.name)
            except ValueError:
                continue
            if _alive(x):
                live.append({"id": x.get("id"), "port": int(x["port"]), "pid": int(x["pid"]),
                             "started_at": float(x["started_at"]) if x.get("started_at") is not None else None})
            else:
                _ENDED_SESSIONS.add(key)
        return sorted(live, key=lambda x: (x["port"], x["pid"]))
    except Exception:  # noqa: BLE001 - only used to enrich an error message
        return []
