"""Pack sources: where a pack version is pulled from, and ``pull`` / ``push`` against a Hugging Face repository.

The repository ships no assets. The owner lists where their packs live in their own sources file,
``~/.config/twinsmith/pack-sources.toml``, which is outside every checkout and holds no secret:

.. code-block:: toml

    [hub]
    repo = "<owner>/<repo>"                  # where push sends a pack with no entry yet
    repo_type = "dataset"                    # default dataset
    token_file = "~/.config/agent-secrets/hf_token"              # read token (default; absent: anonymous)
    write_token_file = "~/.config/agent-secrets/hf_write_token"  # push only (default)

    [packs."ur5e-workcell/v2"]
    repo = "<owner>/<repo>"                  # default: hub.repo
    revision = "<40-hex commit>"             # pinned: never a branch
    path = "ur5e-workcell/v2"                # folder in the repository; default "<id>/<version>"

    [packs."d415-cad/v1"]                    # a vendor entry: never downloaded, only reported
    vendor = "https://..."
    sha256 = "<64 hex>"
    how = "one line: how to fetch it and make the pack"

``pull`` downloads one pack's folder at its pinned commit into a hidden staging folder under the asset root, runs
the strict pack reader on it (every file listed and hashed, nothing extra, no symlink) and moves it into place only
if it verifies. A version that is already there is compared by its ``pack.yaml`` and never replaced. ``push``
uploads a version that verifies, refuses a public repository for a pack with non-redistributable files, never
changes a version the repository already holds, and pins the new commit in the sources file.

Tokens are read from mode-600 files with :func:`config.read_secret_file`, passed to ``huggingface_hub`` as an
argument, and never printed, logged or returned; any hub message is scrubbed of them. ``huggingface_hub`` is the
optional ``hub`` extra and is imported only here. This module never prints: stdout is the MCP protocol for a tool.
"""

from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

from . import config, pack_manifest
from .asset_packs import PackError, Staging, new_version, pack_dir, split_ref
from .pack_manifest import MANIFEST_NAME

SOURCES_FILE = ".config/twinsmith/pack-sources.toml"
DEFAULT_TOKEN_FILE = "~/.config/agent-secrets/hf_token"
DEFAULT_WRITE_TOKEN_FILE = "~/.config/agent-secrets/hf_write_token"
REPO_TYPES = ("dataset", "model")
HUB_EXTRA_COMMAND = ("UV_PROJECT_ENVIRONMENT=<repo>/mcp/.venv uv sync --project <repo>/mcp --locked --extra hub --no-dev "
                     "(add the extras you already use), or pip install 'twinsmith[hub]'")
WRITE_TOKEN_COMMAND = "agent-secret-set hf-write"

_REPO = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,95}/[A-Za-z0-9][A-Za-z0-9_.-]{0,95}")
_COMMIT = re.compile(r"[0-9a-f]{40}")
_SHA256 = re.compile(r"[0-9a-f]{64}")
_HUB_KEYS = {"repo", "repo_type", "token_file", "write_token_file"}
_REPO_KEYS = {"repo", "revision", "path", "repo_type"}
_VENDOR_KEYS = {"vendor", "sha256", "how"}
_HEADER = """\
# Where twinsmith pulls asset packs from: `twinsmith assets pull`, `twinsmith assets push`.
# Holds no secret: tokens are read from the files named here. `push` rewrites this file.
# Format: config/pack-sources.example.toml and docs/ASSET-PACKS.md in the twinsmith repository.
"""


@dataclass(frozen=True)
class Source:
    """One ``[packs."<id>/<version>"]`` entry."""

    pack_id: str
    version: str
    repo: str | None = None
    revision: str | None = None
    path: str | None = None
    repo_type: str = "dataset"
    vendor: str | None = None
    sha256: str | None = None
    how: str | None = None

    @property
    def ref(self) -> str:
        return f"{self.pack_id}/{self.version}"

    @property
    def folder(self) -> str:
        return self.path or self.ref


@dataclass(frozen=True)
class Sources:
    path: Path
    hub: dict
    packs: dict  # ref -> Source

    def token_file(self, key: str, default: str) -> tuple[Path, bool]:
        """The token file and whether the owner named it (a named file must exist)."""
        named = key in self.hub
        return Path(self.hub.get(key, default)).expanduser(), named


# ---------------------------------------------------------------------------
# The sources file
# ---------------------------------------------------------------------------


def sources_path(home: Path | None = None) -> Path:
    return (Path.home() if home is None else Path(home)) / SOURCES_FILE


def _text(value, where: str, pattern: re.Pattern | None = None, what: str = "one line of text") -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 500 or any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise PackError(f"{where} must be {what}.", "INVALID_SOURCES")
    if pattern is not None and not pattern.fullmatch(value):
        raise PackError(f"{where} must be {what}, not {value!r}.", "INVALID_SOURCES")
    return value


def _folder(value, where: str) -> str:
    _text(value, where, what="a folder in the repository such as \"my-robot/v1\"")
    parts = value.split("/")
    if value.startswith("/") or any(part in ("", ".", "..") for part in parts):
        raise PackError(f"{where} must be a relative folder with no \"..\", not {value!r}.", "INVALID_SOURCES")
    return value


def _token_path(value, where: str) -> str:
    _text(value, where, what="a file path")
    if not Path(value).expanduser().is_absolute():
        raise PackError(f"{where} must be an absolute path or begin with the home folder (~).", "INVALID_SOURCES")
    return value


def parse(data: dict, path: Path) -> Sources:
    """Check a parsed sources file. Anything it does not know is an error, never a quiet default."""
    unknown = set(data) - {"hub", "packs"}
    if unknown:
        raise PackError(f"{path}: unknown table {sorted(unknown)[0]!r}; expected [hub] and [packs.\"<id>/<version>\"].",
                        "INVALID_SOURCES")
    hub = data.get("hub", {})
    if not isinstance(hub, dict):
        raise PackError(f"{path}: [hub] must be a table.", "INVALID_SOURCES")
    extra = set(hub) - _HUB_KEYS
    if extra:
        raise PackError(f"{path}: unknown key hub.{sorted(extra)[0]}; known: {', '.join(sorted(_HUB_KEYS))}.", "INVALID_SOURCES")
    if "repo" in hub:
        _text(hub["repo"], f"{path}: hub.repo", _REPO, "\"<owner>/<name>\"")
    if "repo_type" in hub and hub["repo_type"] not in REPO_TYPES:
        raise PackError(f"{path}: hub.repo_type must be one of {', '.join(REPO_TYPES)}.", "INVALID_SOURCES")
    for key in ("token_file", "write_token_file"):
        if key in hub:
            _token_path(hub[key], f"{path}: hub.{key}")
    packs = {}
    table = data.get("packs", {})
    if not isinstance(table, dict):
        raise PackError(f"{path}: [packs] must be a table of \"<id>/<version>\" entries.", "INVALID_SOURCES")
    for ref, entry in table.items():
        where = f"{path}: packs.\"{ref}\""
        try:
            pack_id, version = pack_manifest.split_pack_ref(ref)
        except pack_manifest.ManifestError as exc:
            raise PackError(f"{where}: {exc}", "INVALID_SOURCES") from exc
        if not isinstance(entry, dict):
            raise PackError(f"{where} must be a table.", "INVALID_SOURCES")
        if "vendor" in entry:
            extra = set(entry) - _VENDOR_KEYS
            if extra:
                raise PackError(f"{where}: a vendor entry takes only {', '.join(sorted(_VENDOR_KEYS))}, not {sorted(extra)[0]}.",
                                "INVALID_SOURCES")
            vendor = _text(entry["vendor"], f"{where}.vendor", re.compile(r"https://\S+"), "an https:// URL")
            digest = _text(entry["sha256"], f"{where}.sha256", _SHA256, "64 lowercase hex characters") if "sha256" in entry else None
            how = _text(entry["how"], f"{where}.how") if "how" in entry else None
            packs[ref] = Source(pack_id, version, vendor=vendor, sha256=digest, how=how)
            continue
        extra = set(entry) - _REPO_KEYS
        if extra:
            raise PackError(f"{where}: unknown key {sorted(extra)[0]}; known: {', '.join(sorted(_REPO_KEYS))} "
                            f"(or {', '.join(sorted(_VENDOR_KEYS))} for a vendor entry).", "INVALID_SOURCES")
        repo = entry.get("repo", hub.get("repo"))
        if repo is None:
            raise PackError(f"{where} needs repo (or set hub.repo).", "INVALID_SOURCES")
        _text(repo, f"{where}.repo", _REPO, "\"<owner>/<name>\"")
        if "revision" not in entry:
            raise PackError(f"{where} needs revision: the 40-character commit to pull, never a branch.", "INVALID_SOURCES")
        revision = _text(entry["revision"], f"{where}.revision", _COMMIT, "a 40-character lowercase commit hash (pinned)")
        folder = _folder(entry["path"], f"{where}.path") if "path" in entry else None
        repo_type = entry.get("repo_type", hub.get("repo_type", "dataset"))
        if repo_type not in REPO_TYPES:
            raise PackError(f"{where}.repo_type must be one of {', '.join(REPO_TYPES)}.", "INVALID_SOURCES")
        packs[ref] = Source(pack_id, version, repo=repo, revision=revision, path=folder, repo_type=repo_type)
    return Sources(path=path, hub=dict(hub), packs=packs)


def load(home: Path | None = None) -> Sources:
    """The owner's sources file, or an empty one when it does not exist."""
    path = sources_path(home)
    if not path.exists():
        return Sources(path=path, hub={}, packs={})
    if config.tomllib is None:
        raise PackError(f"Reading {path} needs Python 3.11 or the tomli package: run twinsmith from its own environment.",
                        "INVALID_SOURCES")
    try:
        with path.open("rb") as stream:
            data = config.tomllib.load(stream)
    except (OSError, config.tomllib.TOMLDecodeError) as exc:
        raise PackError(f"{path} cannot be read: {exc}", "INVALID_SOURCES") from None
    return parse(data, path)


def _toml_value(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)  # a JSON string is a valid TOML basic string


def render(sources: Sources) -> str:
    lines = [_HEADER]
    if sources.hub:
        lines.append("[hub]")
        lines += [f"{key} = {_toml_value(sources.hub[key])}" for key in sorted(sources.hub)]
        lines.append("")
    for ref in sorted(sources.packs):
        src = sources.packs[ref]
        lines.append(f"[packs.{_toml_value(ref)}]")
        if src.vendor:
            pairs = [("vendor", src.vendor), ("sha256", src.sha256), ("how", src.how)]
        else:
            pairs = [("repo", None if src.repo == sources.hub.get("repo") else src.repo), ("revision", src.revision),
                     ("path", None if src.folder == ref else src.path),
                     ("repo_type", None if src.repo_type == sources.hub.get("repo_type", "dataset") else src.repo_type)]
        lines += [f"{key} = {_toml_value(value)}" for key, value in pairs if value is not None]
        lines.append("")
    return "\n".join(lines)


def write(sources: Sources) -> None:
    """Replace the sources file atomically (it holds no secret; mode 644)."""
    sources.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, tmp = tempfile.mkstemp(prefix=".pack-sources.", dir=sources.path.parent)
    try:
        with os.fdopen(fd, "w") as out:
            out.write(render(sources))
        os.chmod(tmp, 0o644)
        os.replace(tmp, sources.path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# ---------------------------------------------------------------------------
# Hub access
# ---------------------------------------------------------------------------


def hub_module():
    import logging

    for name in ("httpx", "httpx2", "huggingface_hub"):   # request lines are noise here; failures come back as errors
        logging.getLogger(name).setLevel(logging.WARNING)
    try:
        import huggingface_hub
    except ImportError:
        raise PackError(f"pull and push need huggingface_hub, the optional 'hub' extra: {HUB_EXTRA_COMMAND}.",
                        "HUB_EXTRA_MISSING") from None
    return huggingface_hub


def _read_token(sources: Sources, key: str, default: str, required: bool):
    path, named = sources.token_file(key, default)
    if not path.exists() and not path.is_symlink():
        if named or required:
            return None, path
        return False, path            # anonymous: never the environment's or the hub cache's token
    return config.read_secret_file(path, f"hub.{key}"), path


def _scrub(message: str, *secrets_: str | None) -> str:
    for secret in secrets_:
        if secret:
            message = message.replace(secret, "***")
    message = re.sub(r"hf_[A-Za-z0-9]{20,}", "***", message)
    return message[:400]


def _names(exc: BaseException) -> set[str]:
    return {cls.__name__ for cls in type(exc).__mro__}


def _status(exc: BaseException):
    response = getattr(exc, "response", None)
    return getattr(response, "status_code", None)


def hub_error(exc: BaseException, src_repo: str, repo_type: str, token, token_path: Path, revision=None) -> PackError:
    """A clear refusal for a hub failure. The message names the token file, never the token."""
    names, detail = _names(exc), _scrub(str(exc), token if isinstance(token, str) else None)
    who = (f"the token in {token_path}" if isinstance(token, str)
           else f"no token (there is no {token_path}; a private repository needs one: agent-secret-set hf)")
    if "GatedRepoError" in names:
        return PackError(f"{repo_type} {src_repo} is gated and {who} has not been granted access.", "NO_ACCESS")
    if "RepositoryNotFoundError" in names or _status(exc) in (401, 403):
        return PackError(f"{repo_type} {src_repo} cannot be read with {who}: it does not exist, or it is private and the "
                         "token has no read access to it (a private repository answers 'not found'). Check the token's "
                         "repository permissions at https://huggingface.co/settings/tokens.", "NO_ACCESS")
    if "RevisionNotFoundError" in names:
        return PackError(f"{repo_type} {src_repo} has no commit {revision}: fix the pinned revision.", "REVISION_NOT_FOUND")
    if names & {"EntryNotFoundError", "RemoteEntryNotFoundError"}:
        return PackError(f"{repo_type} {src_repo} at {revision} does not hold the file: {detail}", "NOT_IN_REPO")
    if names & {"ConnectionError", "ConnectError", "TimeoutException", "Timeout", "OfflineModeIsEnabled"}:
        return PackError(f"cannot reach the Hugging Face hub for {src_repo}: {detail}", "HUB_UNREACHABLE")
    return PackError(f"Hugging Face hub refused {repo_type} {src_repo}: {detail}", "HUB_ERROR")


def _quiet(hub) -> None:
    """No progress bars on a tool path, where stdout is the protocol."""
    utils = getattr(hub, "utils", None)
    if utils is not None and hasattr(utils, "disable_progress_bars"):
        utils.disable_progress_bars()


# ---------------------------------------------------------------------------
# pull
# ---------------------------------------------------------------------------


def _manifest_bytes(folder: Path) -> bytes | None:
    path = folder / MANIFEST_NAME
    return path.read_bytes() if path.is_file() and not path.is_symlink() else None


def pull_one(root: Path, src: Source, token, token_path: Path, hub) -> dict:
    """Pull one pack version. Returns a report; raises :class:`PackError`."""
    if src.vendor:
        return dict(pack=src.ref, outcome="vendor", url=src.vendor, sha256=src.sha256, how=src.how,
                    message="Not downloaded: the vendor's terms say fetch it from them. Fetch it yourself, then make "
                            "the pack with adopt or build.")
    root, folder = pack_dir(root, src.pack_id, src.version)
    where = dict(pack=src.ref, repo=src.repo, repo_type=src.repo_type, revision=src.revision, path=src.folder)

    def failed(exc):
        return hub_error(exc, src.repo, src.repo_type, token, token_path, src.revision)

    if folder.exists() or folder.is_symlink():
        local = pack_manifest.inspect_pack(root, src.pack_id, src.version)
        with tempfile.TemporaryDirectory(prefix=".pull-", dir=root) as tmp:
            try:
                got = hub.hf_hub_download(repo_id=src.repo, filename=f"{src.folder}/{MANIFEST_NAME}", repo_type=src.repo_type,
                                          revision=src.revision, local_dir=tmp, token=token)
            except Exception as exc:  # noqa: BLE001 - every hub failure becomes a named refusal
                raise failed(exc) from None
            remote = Path(got).read_bytes()
        if _manifest_bytes(folder) != remote:
            raise PackError(f"{src.ref} is already in {root} with a different pack.yaml than {src.repo} at {src.revision[:12]}. "
                            "A version never changes, so nothing was replaced: move the local folder aside or pin another "
                            "version.", "LOCAL_DIFFERS", **where)
        if not local["verified"]:
            raise PackError(f"{src.ref} is already in {root} but does not verify ({'; '.join(local['problems'][:3])}); nothing "
                            "was replaced. Move it aside and pull again.", "LOCAL_NOT_VERIFIED", **where)
        return dict(outcome="up_to_date", **where, files=local["files"], bytes=local["bytes"], pack_dir=str(folder))

    new_version(root, src.pack_id, src.version)      # refuses a symlinked pack folder
    notes: list = []                                  # returned, never printed: stdout is the protocol for a tool
    with Staging(root, src.pack_id, src.version, notes) as staging:
        download = staging.base / ".download"
        try:
            hub.snapshot_download(repo_id=src.repo, repo_type=src.repo_type, revision=src.revision,
                                  allow_patterns=[f"{src.folder}/*"], local_dir=str(download), token=token)
        except Exception as exc:  # noqa: BLE001
            raise failed(exc) from None
        fetched = download / src.folder
        if not (fetched / MANIFEST_NAME).is_file():
            raise PackError(f"{src.repo} at {src.revision[:12]} has no {src.folder}/{MANIFEST_NAME}: fix the entry's path.",
                            "NOT_IN_REPO", **where)
        os.rmdir(staging.path)
        os.rename(fetched, staging.path)
        report = pack_manifest.inspect_pack(staging.base, src.pack_id, src.version)
        if not report["verified"]:
            raise PackError(f"{src.ref} from {src.repo} does not verify, so nothing was written: "
                            + "; ".join(report["problems"][:5]), "NOT_VERIFIED", **where)
        staging.commit()
    return dict(outcome="pulled", **where, files=report["files"], bytes=report["bytes"], pack_dir=str(folder),
                **({"notes": notes} if notes else {}))


def pull(root: Path, pack: str | None = None, all_packs: bool = False, home: Path | None = None, hub=None,
         progress: bool = False) -> dict:
    """Pull one pack (``pack``) or every entry of the sources file (``all_packs``)."""
    if bool(pack) == bool(all_packs):
        raise PackError("name one pack as \"<pack id>/<version>\", or pull all of them.", "INVALID_VALUE")
    sources = load(home)
    if pack:
        pack = "/".join(split_ref(pack))
        if pack not in sources.packs:
            raise PackError(f"{pack} has no entry in {sources.path}. Add [packs.\"{pack}\"] with repo and revision "
                            "(config/pack-sources.example.toml shows the format).", "NO_SOURCE")
        chosen = [sources.packs[pack]]
    else:
        chosen = [sources.packs[ref] for ref in sorted(sources.packs)]
        if not chosen:
            raise PackError(f"{sources.path} lists no packs.", "NO_SOURCE")
    token, token_path = None, None
    if any(not src.vendor for src in chosen):
        hub = hub or hub_module()
        if not progress:
            _quiet(hub)
        token, token_path = _read_token(sources, "token_file", DEFAULT_TOKEN_FILE, required=False)
        if token is None:
            raise PackError(f"hub.token_file {token_path} does not exist.", "NO_TOKEN")
    results, failed = [], 0
    for src in chosen:
        try:
            results.append(pull_one(root, src, token, token_path, hub))
        except PackError as exc:
            if pack:
                raise
            failed += 1
            results.append(dict(pack=src.ref, outcome="error", code=exc.code, message=str(exc)))
    out = dict(sources_file=str(sources.path), asset_root=str(root), results=results)
    if failed:
        return dict(status="error", code="PULL_FAILED", message=f"{failed} of {len(results)} pack(s) failed.", **out)
    return dict(status="ok", **out)


# ---------------------------------------------------------------------------
# push
# ---------------------------------------------------------------------------


def push(root: Path, pack: str, home: Path | None = None, repo: str | None = None, create: bool = False,
         dry_run: bool = False, hub=None, progress: bool = False) -> dict:
    """Upload a pack version that verifies and pin the commit in the sources file."""
    pack_id, version = split_ref(pack)
    ref = f"{pack_id}/{version}"
    sources = load(home)
    existing = sources.packs.get(ref)
    if existing is not None and existing.vendor:
        raise PackError(f"{ref} is a vendor entry in {sources.path}: a vendor's files are fetched from the vendor, never "
                        "pushed.", "VENDOR_PACK")
    root, folder = pack_dir(root, pack_id, version)
    local = pack_manifest.inspect_pack(root, pack_id, version)
    if not local["verified"]:
        raise PackError(f"{ref} does not verify, so it is not pushed: " + "; ".join(local["problems"][:5]), "NOT_VERIFIED")
    if repo is not None:
        _text(repo, "repo", _REPO, "\"<owner>/<name>\"")
    repo = repo or (existing.repo if existing else None) or sources.hub.get("repo")
    if not repo:
        raise PackError(f"no repository: pass repo, or set hub.repo in {sources.path}.", "NO_REPO")
    repo_type = existing.repo_type if existing else sources.hub.get("repo_type", "dataset")
    path = existing.folder if existing else ref
    private_files = sum(1 for record in local["records"] if not record["redistributable"])
    plan = dict(pack=ref, repo=repo, repo_type=repo_type, path=path, files=local["files"], bytes=local["bytes"],
                not_redistributable=private_files, sources_file=str(sources.path))
    token, token_path = _read_token(sources, "write_token_file", DEFAULT_WRITE_TOKEN_FILE, required=True)
    if dry_run:
        return dict(status="ok", outcome="dry_run", **plan, write_token="found" if token else f"missing: {token_path}",
                    message="Nothing was uploaded or written.")
    if not token:
        raise PackError(f"push needs a write token in {token_path}. Make a fine-grained token with write access to {repo} "
                        f"only, then store it with: {WRITE_TOKEN_COMMAND}", "NO_WRITE_TOKEN", **plan)
    hub = hub or hub_module()
    if not progress:
        _quiet(hub)
    api = hub.HfApi(token=token)

    def failed(exc):
        return hub_error(exc, repo, repo_type, token, token_path)

    created = False
    try:
        info = api.repo_info(repo, repo_type=repo_type)
    except Exception as exc:  # noqa: BLE001
        if "RepositoryNotFoundError" not in _names(exc) or "GatedRepoError" in _names(exc):
            raise failed(exc) from None
        if not create:
            raise PackError(f"{repo_type} {repo} does not exist, or this write token cannot see it. Pass create to make it, "
                            "as a private repository.", "NO_REPO", **plan) from None
        try:
            api.create_repo(repo, repo_type=repo_type, private=True)
            info = api.repo_info(repo, repo_type=repo_type)
        except Exception as exc2:  # noqa: BLE001
            raise failed(exc2) from None
        created = True
    if not getattr(info, "private", False) and private_files:
        raise PackError(f"{repo_type} {repo} is public and {ref} has {private_files} file(s) that are not redistributable: "
                        "nothing was uploaded. Push it to a private repository.", "NOT_PRIVATE", **plan)
    head = info.sha
    try:
        remote = [name for name in api.list_repo_files(repo, repo_type=repo_type, revision=head) if name.startswith(path + "/")]
    except Exception as exc:  # noqa: BLE001
        raise failed(exc) from None
    expected = {f"{path}/{record['path']}" for record in local["records"]} | {f"{path}/{MANIFEST_NAME}"}
    if remote:
        with tempfile.TemporaryDirectory(prefix=".push-", dir=root) as tmp:
            try:
                got = hub.hf_hub_download(repo_id=repo, filename=f"{path}/{MANIFEST_NAME}", repo_type=repo_type,
                                          revision=head, local_dir=tmp, token=token)
                same = Path(got).read_bytes() == _manifest_bytes(folder)
            except Exception as exc:  # noqa: BLE001
                if not _names(exc) & {"EntryNotFoundError", "RemoteEntryNotFoundError"}:
                    raise failed(exc) from None
                same = False
        if not same or set(remote) != expected:
            raise PackError(f"{repo} already holds a different {ref} in {path}/ at {head[:12]}. A version never changes: "
                            "nothing was uploaded. Push a new version.", "VERSION_EXISTS", **plan)
        revision, outcome = head, "already_there"
    else:
        try:
            commit = api.upload_folder(repo_id=repo, repo_type=repo_type, folder_path=str(folder), path_in_repo=path,
                                       commit_message=f"Add pack {ref}", parent_commit=head)
        except Exception as exc:  # noqa: BLE001
            raise failed(exc) from None
        revision, outcome = commit.oid, "pushed"
    if not _COMMIT.fullmatch(str(revision or "")):
        raise PackError(f"the hub returned no commit for {ref}; check {repo} and pin it by hand.", "HUB_ERROR", **plan)
    packs = dict(sources.packs)
    packs[ref] = Source(pack_id, version, repo=repo, revision=revision, path=None if path == ref else path, repo_type=repo_type)
    write(Sources(path=sources.path, hub=sources.hub, packs=packs))
    return dict(status="ok", outcome=outcome, **plan, revision=revision, created_repo=created,
                message=f"Pinned {ref} at {revision[:12]} in {sources.path}. Copy that file to your other machines, then "
                        f"twinsmith assets pull {ref} there.")
