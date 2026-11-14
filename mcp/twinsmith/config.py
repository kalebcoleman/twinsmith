"""Resolve startup configuration without importing the MCP SDK or simulator."""

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

try:
    import tomllib
except ModuleNotFoundError:          # Python before 3.11: tomli, if it is installed
    try:
        import tomli as tomllib
    except ModuleNotFoundError:
        tomllib = None

from .toolsets import DEFAULT, PROFILES, TOOLSETS

def select(value) -> frozenset[str]:
    if isinstance(value, str):
        value = value.split(",")
    if not isinstance(value, list) or not value or not all(isinstance(item, str) for item in value):
        raise ValueError("toolsets must be a nonempty list of names or a comma-separated string")
    enabled = {"core"}
    for raw in value:
        item = raw.strip()
        if item in PROFILES:
            enabled = set(PROFILES[item])
            continue
        name = item.lstrip("+-")
        if name not in TOOLSETS or item not in {name, "+" + name, "-" + name}:
            raise ValueError(f"Unknown toolset {item!r}. Valid names: {', '.join((*PROFILES, *TOOLSETS))}")
        if item == "-core":
            raise ValueError("core cannot be turned off")
        if item.startswith("-"):
            enabled.discard(name)
        else:
            enabled.add(name)
    return frozenset(enabled | {"core"})


# The one place the default physics engine is written down.
DEFAULT_PHYSICS_BACKEND = "newton"
PHYSICS_BACKENDS = ("physx", "newton")


def _flag(value, name: str) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be true or false, not {value!r}")


@dataclass(frozen=True)
class Config:
    toolsets: frozenset[str] = DEFAULT
    physics_backend: str = DEFAULT_PHYSICS_BACKEND
    physics_backend_source: str = "built-in"       # built-in, user file, project file or environment
    runtime_enable: bool = True                    # may an agent call toolsets(action="enable")
    cursor_badge: bool = False                     # draw a click-through badge where computer input lands


def _minimal_sim_table(text: str, path: Path) -> dict:
    """Only ``[sim] physics_backend = "..."``, for a Python with no TOML reader (before 3.11, without
    tomli). Anything it cannot read is an error, never a quiet default."""
    section, found = "", {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if line.startswith("["):
            section = line.strip("[] \t")
        elif section == "sim":
            match = re.fullmatch(r"physics_backend\s*=\s*[\"']([^\"']*)[\"']", line)
            if match:
                found["physics_backend"] = match.group(1)
    if "physics_backend" in text and not found:
        raise ValueError(f"Cannot read physics_backend from {path} without a TOML reader: use Python 3.11 or "
                         "install tomli, or set TWINSMITH_PHYSICS_BACKEND.")
    return {"sim": found} if found else {}


def _config_files(cwd: Path | None = None, home: Path | None = None, minimal: bool = False):
    """The user and project configuration files that exist, lowest priority first, as (label, data).
    With ``minimal`` a Python that has no TOML reader still yields the physics setting."""
    current = (Path.cwd() if cwd is None else Path(cwd)).resolve()
    base = Path.home() if home is None else Path(home)
    user = base / ".config/twinsmith/config.toml"
    project = None
    for directory in (current, *current.parents):
        if (directory / "twinsmith.toml").is_file():
            project = directory / "twinsmith.toml"
            break
    for label, path in (("user file", user), ("project file", project)):
        if path is not None and path.is_file():
            if tomllib is None:
                if not minimal:
                    raise ValueError(f"Reading {path} needs Python 3.11 or the tomli package.")
                yield label, _minimal_sim_table(path.read_text(), path)
                continue
            with path.open("rb") as stream:
                yield label, tomllib.load(stream)


def resolve_physics_backend(env: Mapping[str, str] | None = None, cwd: Path | None = None,
                            home: Path | None = None) -> tuple[str, str]:
    """The configured physics backend and where it came from: built-in, user file, project file or
    environment. Reads only the physics setting, so a launcher can ask for it with nothing else
    resolved. This is the one place the default is written down; it changes no simulator's engine."""
    env = os.environ if env is None else env
    backend, source = DEFAULT_PHYSICS_BACKEND, "built-in"
    if env.get("TWINSMITH_PHYSICS_BACKEND"):          # the highest priority: the files need not be read
        backend, source = env["TWINSMITH_PHYSICS_BACKEND"], "environment"
    else:
        for label, data in _config_files(cwd, home, minimal=True):
            if "physics_backend" in data.get("sim", {}):
                backend, source = data["sim"]["physics_backend"], label
    if backend not in PHYSICS_BACKENDS:
        raise ValueError(f"physics_backend must be {' or '.join(PHYSICS_BACKENDS)}, not {backend!r} (from the {source})")
    return backend, source


def resolve(env: Mapping[str, str] | None = None, cli_toolsets: str | None = None,
            cwd: Path | None = None, home: Path | None = None) -> Config:
    env = os.environ if env is None else env
    enabled, runtime, badge = DEFAULT, True, False
    for _, data in _config_files(cwd, home):
        if "toolsets" in data.get("tools", {}):
            enabled = select(data["tools"]["toolsets"])
        if "runtime_enable" in data.get("tools", {}):
            runtime = _flag(data["tools"]["runtime_enable"], "tools.runtime_enable")
        if "screen" in data:
            raise ValueError("The [screen] section is now [computer] (cursor_badge = true under [computer]).")
        if "cursor_badge" in data.get("computer", {}):
            badge = _flag(data["computer"]["cursor_badge"], "computer.cursor_badge")
    if "TWINSMITH_TOOLSETS" in env:
        enabled = select(env["TWINSMITH_TOOLSETS"])
    if cli_toolsets is not None:
        enabled = select(cli_toolsets)
    if "TWINSMITH_RUNTIME_ENABLE" in env:
        runtime = _flag(env["TWINSMITH_RUNTIME_ENABLE"], "TWINSMITH_RUNTIME_ENABLE")
    if "TWINSMITH_CURSOR_BADGE" in env:
        badge = _flag(env["TWINSMITH_CURSOR_BADGE"], "TWINSMITH_CURSOR_BADGE")
    backend, source = resolve_physics_backend(env, cwd, home)
    return Config(enabled, backend, source, runtime, cursor_badge=badge)


def _owner_planning(home: Path | None = None) -> tuple[Path, dict]:
    """The ``[planning]`` table of the owner's own ``~/.config/twinsmith/config.toml`` (empty when unset), and its path.
    No project file is read, so a cloned repository cannot change it. A malformed file raises ValueError."""
    path = (Path.home() if home is None else Path(home)) / ".config/twinsmith/config.toml"
    if not path.is_file():
        return path, {}
    if tomllib is None:
        raise ValueError(f"Reading {path} needs Python 3.11 or the tomli package.")
    try:
        with path.open("rb") as stream:
            data = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ValueError(f"{path} is not readable TOML: {error}") from None
    planning = data.get("planning", {})
    if not isinstance(planning, dict):
        raise ValueError(f"[planning] in {path} must be a table.")
    return path, planning


def _owner_planning_flag(name: str, home: Path | None = None) -> bool:
    """``[planning] <name>`` from the owner's own file, false when unset; a value that is not a TOML boolean raises."""
    path, planning = _owner_planning(home)
    value = planning.get(name, False)
    if not isinstance(value, bool):   # an opt-in is a TOML boolean, never a string or number that might mean yes
        raise ValueError(f"planning.{name} in {path} must be true or false, not {value!r}.")
    return value


def allow_paid(home: Path | None = None) -> bool:
    """Whether the owner allows calls to paid models (``[planning] allow_paid``)."""
    return _owner_planning_flag("allow_paid", home)


def allow_scripts(home: Path | None = None) -> bool:
    """Whether the owner allows graph ``script`` and ``router`` nodes from outside the pinned checkouts
    (``[planning] allow_scripts``). Those nodes are arbitrary Python run inside Graph-as-Policy."""
    return _owner_planning_flag("allow_scripts", home)


def loopback_url(value, name: str) -> str:
    """An ``http(s)`` URL whose host is this machine (``localhost``, ``127.0.0.0/8`` or ``::1``), with no user or
    password in it. Raises ValueError otherwise."""
    import ipaddress
    from urllib.parse import urlsplit

    if not isinstance(value, str) or not value or any(ord(c) < 33 for c in value):
        raise ValueError(f"{name} must be a URL without spaces.")
    try:
        parts = urlsplit(value)
        host, _ = parts.hostname, parts.port      # .port raises on a malformed port
    except ValueError as error:
        raise ValueError(f"{name} is not a URL: {error}") from None
    if parts.scheme not in ("http", "https") or not host or parts.username is not None or parts.password is not None:
        raise ValueError(f"{name} must be http:// or https:// with a host and no user or password.")
    if host != "localhost":
        try:
            loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loopback = False
        if not loopback:
            raise ValueError(f"{name} must point at this machine (localhost, 127.0.0.1 or ::1), not {host!r}.")
    return value


def vlm_url(value, name: str) -> str:
    """A VLM endpoint the owner wrote: ``https`` to any host, or ``http`` only to this machine; never a user or password
    in it. Raises ValueError otherwise."""
    from urllib.parse import urlsplit

    if not isinstance(value, str) or not value or any(ord(c) < 33 for c in value):
        raise ValueError(f"{name} must be a URL without spaces.")
    try:
        parts = urlsplit(value)
        host, _ = parts.hostname, parts.port
    except ValueError as error:
        raise ValueError(f"{name} is not a URL: {error}") from None
    if not host or parts.username is not None or parts.password is not None:
        raise ValueError(f"{name} needs a host and no user or password.")
    if parts.query or parts.fragment or "?" in value or "#" in value:
        raise ValueError(f"{name} must not have a query or fragment: a key belongs in vlm_api_key_file.")
    if parts.scheme == "https":
        return value
    if parts.scheme == "http":
        return loopback_url(value, name)   # plain http only to this machine
    raise ValueError(f"{name} must be https://, or http:// to this machine.")


SECRET_FILE_MAX = 4096


def read_secret_file(path: Path, name: str) -> str:
    """One line from a key file the owner keeps private: a regular file (not a link) owned by them, readable by no one
    else, at most ``SECRET_FILE_MAX`` bytes. Error messages name the file, never its contents."""
    import stat as stat_module

    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0)
    try:
        fd = os.open(path, flags)      # never a link, and a FIFO cannot block the call
    except OSError as error:
        if error.errno == getattr(__import__("errno"), "ELOOP"):
            raise ValueError(f"{name} {path} must be a regular file, not a link or folder.") from None
        raise ValueError(f"{name} {path} cannot be read: {error.strerror}.") from None
    try:
        info = os.fstat(fd)            # the checks are on the file that was opened
        if not stat_module.S_ISREG(info.st_mode):
            raise ValueError(f"{name} {path} must be a regular file, not a link or folder.")
        if info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError(f"{name} {path} must be yours and readable only by you (chmod 600).")
        data = os.read(fd, SECRET_FILE_MAX + 1)
    finally:
        os.close(fd)
    if len(data) > SECRET_FILE_MAX:
        raise ValueError(f"{name} {path} is larger than a key ({SECRET_FILE_MAX} bytes).")
    try:
        lines = [line.strip() for line in data.decode().splitlines() if line.strip()]
    except UnicodeDecodeError:
        raise ValueError(f"{name} {path} cannot be read as text.") from None
    if len(lines) != 1 or any(ord(c) < 33 for c in lines[0]):
        raise ValueError(f"{name} {path} must hold one key on one line.")
    return lines[0]


def vlm_endpoint(home: Path | None = None) -> dict | None:
    """The owner's VLM endpoint for graph nodes from ``[planning] vlm_url``, ``vlm_model`` and ``vlm_api_key_file``, or
    None when unset. The key file is checked here and read only when a process starts (``read_secret_file``). A bad
    value raises ValueError."""
    path, planning = _owner_planning(home)
    names = ("vlm_url", "vlm_model", "vlm_api_key_file")
    if "vlm_url" not in planning:
        extra = [key for key in names if key in planning]
        if extra:
            raise ValueError(f"planning.{extra[0]} in {path} needs planning.vlm_url.")
        return None
    url = vlm_url(planning["vlm_url"], f"planning.vlm_url in {path}")
    model = planning.get("vlm_model")
    if model is not None and (not isinstance(model, str) or not model.strip() or any(ord(c) < 33 for c in model)):
        raise ValueError(f"planning.vlm_model in {path} must be a model name without spaces.")
    key_file = planning.get("vlm_api_key_file")
    if key_file is not None:
        if not isinstance(key_file, str) or not key_file.strip():
            raise ValueError(f"planning.vlm_api_key_file in {path} must be a file path.")
        key_file = Path(key_file).expanduser()
        if not key_file.is_absolute():
            raise ValueError(f"planning.vlm_api_key_file in {path} must be an absolute path or begin with the home folder (~).")
    from urllib.parse import urlsplit

    return {"url": url, "host": urlsplit(url).hostname, "model": model, "api_key_file": key_file}


def physics_backend(**kwargs) -> str:
    """Return the configured backend; this does not change any simulator's engine."""
    return resolve(**kwargs).physics_backend


def main(argv=None) -> int:
    """``python3 -m twinsmith.config --print physics_backend``: what a launcher needs, with only the
    standard library. ``physics_backend_source`` says where the value came from."""
    import argparse
    parser = argparse.ArgumentParser(prog="python3 -m twinsmith.config", description=main.__doc__)
    parser.add_argument("--print", dest="field", required=True, choices=["physics_backend", "physics_backend_source"])
    args = parser.parse_args(argv)
    try:
        backend, source = resolve_physics_backend()
    except ValueError as error:
        print(f"twinsmith configuration: {error}", file=__import__("sys").stderr)
        return 2
    print(backend if args.field == "physics_backend" else source)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
