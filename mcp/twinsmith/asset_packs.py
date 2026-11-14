"""Adopt, convert, pin, check and build asset packs in the ``pack.yaml`` format. Standard library only; no Kit.

``scripts/assets.py`` (run by the user) and the host-side ``assets__pack_*`` MCP tools share
this code. The tools add the trust rules in :func:`run_tool`; the rules are documented in
``docs/ASSET-PACKS.md``. The reader and the asset root are in ``pack_manifest``. This module never
prints from a tool path: stdout is the MCP protocol there. USD files are parsed only by the helper
process or the selected Kit (``usd_closure``).
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import re
import secrets
import shutil
import subprocess
import sys
from pathlib import Path, PurePosixPath

from . import asset_formats, material_refs, mjcf_refs, pack_manifest, urdf_refs
from .pack_manifest import MANIFEST_NAME


def approval_help(folder=None):
    """The filled-in approval command (imported late: project_assets is the heavier module)."""
    from .project_assets import approval_help as _help

    return _help(folder)

USD_SUFFIXES = {".usd", ".usda", ".usdc", ".usdz"}
USD_PYTHON_ENV = "TWINSMITH_USD_PYTHON"
CLOSURE_TIMEOUT_S = 600
MAX_ADOPT_FILES = 20_000
MAX_REPORTED = 200

# File types copied by ``adopt``: scene and robot descriptions, meshes, materials and
# textures, plus layout JSON. Extend with --include.
ADOPT_SUFFIXES = {
    ".usd", ".usda", ".usdc", ".usdz", ".urdf", ".xacro", ".sdf", ".mjcf",
    ".obj", ".stl", ".dae", ".fbx", ".glb", ".gltf", ".ply", ".mtl",
    ".mdl", ".mtlx", ".png", ".jpg", ".jpeg", ".exr", ".hdr", ".tga", ".tif", ".tiff",
}
ADOPT_SKIP_DIRS = {".git", "node_modules", ".venv", "venv", "__pycache__", "build", "dist", ".tox"}


class PackError(ValueError):
    """An expected refusal: the CLI prints it, a tool returns it as ``status="error"``."""

    def __init__(self, message: str, code: str = "REFUSED", **details):
        super().__init__(message)
        self.code = code
        self.details = details


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inside_repo(path: Path) -> bool:
    root = pack_manifest.checkout_root()
    if root is None:
        return False
    try:
        path.resolve().relative_to(root)
    except ValueError:
        return False
    return True


def clean_relpath(raw: str, what: str) -> str:
    """Validate a relative asset path; return it in POSIX form."""
    if not raw or "\\" in raw or "\n" in raw or "\r" in raw or "\x00" in raw:
        raise PackError(f"unsafe {what}: {raw!r}")
    posix = PurePosixPath(raw)
    if posix.is_absolute() or raw.startswith("/") or (len(raw) > 1 and raw[1] == ":"):
        raise PackError(f"unsafe {what} (absolute path): {raw!r}")
    parts = [p for p in posix.parts if p != "."]
    if not parts or ".." in parts:
        raise PackError(f"unsafe {what} (empty or contains '..'): {raw!r}")
    for part in parts:
        if part != part.strip() or any(ord(c) < 32 or ord(c) == 127 for c in part):
            raise PackError(
                f"unsupported {what}: names must not start or end with whitespace "
                f"or contain control characters: {raw!r}"
            )
    return "/".join(parts)



# ---------------------------------------------------------------------------
# Safe installation
# ---------------------------------------------------------------------------

_NOFOLLOW = getattr(os, "O_NOFOLLOW", 0)
_DIR_FLAGS = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | _NOFOLLOW


def open_dir_chain(pack_folder: Path, parts: tuple[str, ...]) -> int:
    """Open *pack_folder*, then each folder in *parts* below it, creating missing ones.

    Every step uses ``O_NOFOLLOW`` relative to the previous directory handle, so a symlink planted
    anywhere below the pack folder is refused before anything is created through it. Returns a
    directory file descriptor the caller must close.
    """
    fd = os.open(pack_folder, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        for part in parts:
            try:
                nxt = os.open(part, _DIR_FLAGS, dir_fd=fd)
            except FileNotFoundError:
                os.mkdir(part, 0o755, dir_fd=fd)
                nxt = os.open(part, _DIR_FLAGS, dir_fd=fd)
            except OSError as exc:
                raise PackError(
                    f"refusing destination: {part!r} under {pack_folder} is a symlink or not a folder"
                ) from exc
            os.close(fd)
            fd = nxt
    except BaseException:
        os.close(fd)
        raise
    return fd


def _temp_name(name: str) -> str:
    return f".{name}.{secrets.token_hex(8)}.part"


def publish(dfd: int, tmp: str, name: str, force: bool) -> None:
    """Move *tmp* to *name* inside directory *dfd*; without ``force`` never replace anything."""
    if force:
        os.rename(tmp, name, src_dir_fd=dfd, dst_dir_fd=dfd)
        return
    try:
        os.link(tmp, name, src_dir_fd=dfd, dst_dir_fd=dfd, follow_symlinks=False)
    except FileExistsError:
        os.unlink(tmp, dir_fd=dfd)
        raise PackError(f"refusing to overwrite: {name}") from None
    except OSError:
        # No hard links on this filesystem: create the destination exclusively and copy.
        # O_EXCL means a file that appears in the meantime is never replaced.
        try:
            out_fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | _NOFOLLOW, 0o666, dir_fd=dfd)
        except FileExistsError:
            os.unlink(tmp, dir_fd=dfd)
            raise PackError(f"refusing to overwrite: {name}") from None
        try:
            with os.fdopen(out_fd, "wb") as out, open(
                tmp, "rb", opener=lambda path, flags: os.open(path, flags, dir_fd=dfd)
            ) as src:
                shutil.copyfileobj(src, out, 1 << 20)
        except BaseException:
            os.unlink(name, dir_fd=dfd)
            raise
    os.unlink(tmp, dir_fd=dfd)


def _unlink_quietly(dfd: int, name: str) -> None:
    try:
        os.unlink(name, dir_fd=dfd)
    except OSError:
        pass  # best effort: the error that caused the cleanup is the one worth reporting


def _write_new_file(dfd: int, name: str, write) -> str:
    """Create a uniquely named temp file in *dfd* (exclusive, no symlink following)."""
    tmp = _temp_name(name)
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | _NOFOLLOW, 0o666, dir_fd=dfd)
    try:
        with os.fdopen(fd, "wb") as out:
            write(out)
    except BaseException:
        os.unlink(tmp, dir_fd=dfd)
        raise
    return tmp


def install_file(src: Path, rel: str, pack_folder: Path, force: bool) -> None:
    """Copy *src* to ``<pack_folder>/<rel>`` without following symlinks below the pack folder."""
    *parents, name = PurePosixPath(rel).parts
    dfd = open_dir_chain(pack_folder, tuple(parents))
    try:
        def write(out):
            with src.open("rb") as handle:
                shutil.copyfileobj(handle, out, 1 << 20)

        tmp = _write_new_file(dfd, name, write)
        try:
            publish(dfd, tmp, name, force)
        except BaseException:
            _unlink_quietly(dfd, tmp)
            raise
    finally:
        os.close(dfd)


def _replace_text(pack_folder: Path, name: str, text: str) -> None:
    dfd = open_dir_chain(pack_folder, ())
    try:
        tmp = _write_new_file(dfd, name, lambda out: out.write(text.encode("utf-8")))
        try:
            os.rename(tmp, name, src_dir_fd=dfd, dst_dir_fd=dfd)
        except BaseException:
            _unlink_quietly(dfd, tmp)
            raise
    finally:
        os.close(dfd)


def private_dirs(path: Path, notes: list | None = None) -> None:
    """Create *path* (and missing parents) as 0755 and make sure *path* itself, the pack
    folder, is not group- or world-writable when it is the user's own
    (others must not be able to
    swap your assets). Folders above it are left as they are. A note goes to *notes*, or to
    stdout for the CLI."""
    missing = []
    probe = path
    while not probe.exists():
        missing.append(probe)
        probe = probe.parent
    for folder in reversed(missing):
        os.mkdir(folder, 0o755)
    try:
        st = path.stat()
    except OSError:
        return
    if st.st_uid == os.getuid() and st.st_mode & 0o022:
        os.chmod(path, st.st_mode & ~0o022 & 0o7777)
        note = f"note: removed group/world write permission from {path}"
        if notes is None:
            print(note)
        else:
            notes.append(note)


def install_many(sources: dict[str, Path], pack_folder: Path, notes: list | None = None) -> int:
    """Copy ``rel -> source`` into the new, empty ``pack_folder`` safely. Returns the number of files.

    Nothing is ever replaced: the folder is a version that did not exist a moment ago.
    """
    for rel in sorted(sources):
        try:
            rel.encode("utf-8")
        except UnicodeEncodeError:
            raise PackError(f"file name is not valid UTF-8: {rel!r}") from None
        target = pack_folder / rel
        if target.is_symlink() or inside_repo(target.parent):
            raise PackError(f"refusing unsafe destination: {target}")
    for rel in sorted(sources):
        install_file(sources[rel], rel, pack_folder, False)
    return len(sources)


def adopt_sources(root: Path, suffixes: set[str] | None, skipped: list | None = None) -> dict[str, Path]:
    """Every asset file below *root* (every file when *suffixes* is None), never following a symlink.

    A symbolic link is not copied, and is named in *skipped* when the caller passes a list: a source
    folder silently missing part of its tree would produce a pack that is not what it looks like.
    """
    found: dict[str, Path] = {}
    for current, dirs, files in os.walk(root, followlinks=False):
        here = Path(current)
        for name in sorted(dirs):
            if (here / name).is_symlink() and skipped is not None:
                skipped.append((here / name).relative_to(root).as_posix() + "/")
        dirs[:] = sorted(d for d in dirs if d not in ADOPT_SKIP_DIRS and not d.startswith(".")
                         and not (here / d).is_symlink())
        for name in files:
            path = here / name
            rel = path.relative_to(root).as_posix()
            if path.is_symlink():
                if skipped is not None:
                    skipped.append(rel)
                continue
            if (suffixes is not None and path.suffix.lower() not in suffixes) or not path.is_file():
                continue
            found[clean_relpath(rel, "file name")] = path
    return found


def named_sources(root: Path, names: list[str], suffixes: set[str], skipped: list | None = None) -> dict[str, Path]:
    """Only these files or folders, relative to *root*. A named file is copied whatever its type."""
    found: dict[str, Path] = {}
    for raw in names:
        rel = clean_relpath(raw, "file")
        path = root / rel
        if path.is_symlink() or any(_is_symlink_part(root, rel)):
            raise PackError(f"refusing symlink: {rel}", "UNSAFE_ENTRY")
        if path.is_dir():
            found.update({f"{rel}/{k}": v for k, v in adopt_sources(path, suffixes, skipped).items()})
        elif path.is_file():
            found[rel] = path
        else:
            raise PackError(f"not in {root}: {rel}", "NOT_FOUND")
    return found


def _is_symlink_part(root: Path, rel: str):
    parts = PurePosixPath(rel).parts
    for depth in range(1, len(parts)):
        yield root.joinpath(*parts[:depth]).is_symlink()


# ---------------------------------------------------------------------------
# USD dependency closure
# ---------------------------------------------------------------------------


def usd_python() -> str | None:
    """The interpreter that can parse USD on the host: the configured one, or this one."""
    configured = os.environ.get(USD_PYTHON_ENV, "").strip()
    if configured:
        return configured
    return sys.executable if importlib.util.find_spec("pxr") else None


def helper_closure(paths: list[str], python: str, timeout: float = CLOSURE_TIMEOUT_S) -> list[dict]:
    """Run ``usd_closure`` under *python*, isolated from this process and working directory."""
    helper = Path(__file__).with_name("usd_closure.py")
    try:
        done = subprocess.run(
            [python, "-I", str(helper)], input=json.dumps(paths), capture_output=True, text=True,
            timeout=timeout, cwd=str(helper.parent.parent),
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PackError(f"USD helper failed under {python}: {exc}", "USD_FAILED") from exc
    marker = "TWINSMITH_CLOSURE "
    for line in reversed(done.stdout.splitlines()):
        if line.startswith(marker):
            return json.loads(line[len(marker):])
    raise PackError(
        f"USD helper under {python} exited {done.returncode}: {done.stderr.strip()[-500:]}", "USD_FAILED"
    )


def run_closure(closure, paths: list[str], folder_of, projects: dict | None = None) -> list[dict]:
    """Closures for a mix of files: URDFs are read here, with no USD parser, and USD goes to *closure*.

    *folder_of* gives the folder a URDF's packages are looked for in. Nothing outside it is searched.
    """
    projects = {} if projects is None else projects
    results, usd = [], []
    for path in paths:
        if mjcf_refs.is_mjcf(path) and Path(path).name.lower() != "package.xml":
            folder = Path(folder_of(path))
            project = projects.setdefault(str(folder), urdf_refs.project_from_folder(folder))
            results.append(mjcf_refs.closure_result(path, project, folder))
            continue
        fmt = asset_formats.format_of(path)
        if fmt in ("xacro", "sdf", "mesh", "map") and not (fmt == "map" and Path(path).suffix.lower() not in (".yaml", ".yml")):
            folder = Path(folder_of(path))
            project = projects.setdefault(str(folder), urdf_refs.project_from_folder(folder))
            results.append(asset_formats.closure_result(path, project, folder, fmt))
            continue
        if not urdf_refs.is_urdf(path):
            usd.append(path)
            continue
        folder = Path(folder_of(path))
        project = projects.setdefault(str(folder), urdf_refs.project_from_folder(folder))
        results.append(urdf_refs.closure_result(path, project, folder))
    return results + (list(closure(usd)) if usd else [])


class ClosureBackend:
    """Compute closures with a host USD helper, else the selected Kit, and say which ran."""

    def __init__(self, python: str | None = None, kit_call=None, trusted=None):
        self.python = python or usd_python()
        self.kit_call = kit_call
        self.trusted = trusted
        self.name = "usd_helper" if self.python else "kit" if kit_call else None

    def available(self) -> bool:
        return self.name is not None

    def __call__(self, paths: list[str]) -> list[dict]:
        if self.trusted is not None:
            for path in paths:
                if not self.trusted(Path(path)):
                    raise PackError(
                        f"Will not parse USD outside approved roots and the asset root: {path}. "
                        "A folder needs approving as a root first." + approval_help(path),
                        "ROOT_NOT_APPROVED",
                    )
        if self.python:
            return helper_closure(paths, self.python)
        if self.kit_call:
            return self.kit_call(paths)
        raise PackError(unavailable_message(), "USD_UNAVAILABLE")


def unavailable_message() -> str:
    return (
        "No USD parser is available: the MCP host has no pxr and no Kit bridge answered. Set "
        f"{USD_PYTHON_ENV} to a Python that has usd-core (no GPU or Kit needed), or start Kit with "
        "sim-tools and select its bridge."
    )


# USD normalizes a reference, so "https://x" comes back as "https:/x". Two letters or more, so a
# Windows drive is not a scheme.
_URL = re.compile(r"[A-Za-z][A-Za-z0-9+.-]+:/")


def plain_path(value: str) -> str:
    """Turn a ``file:`` URL a resolver may report into a path."""
    if value.startswith("file://"):
        from urllib.parse import unquote, urlparse

        return unquote(urlparse(value).path)
    return value[5:] if value.startswith("file:/") else value


def classify_reference(reference: str) -> str:
    """Kind of an unresolved reference: ``remote``, ``runtime``, ``logical`` or ``file``.

    ``runtime`` is a bare MDL name such as ``OmniPBR.mdl``, which Kit's material library supplies
    and standalone USD cannot find. ``logical`` is a relative name with nothing to anchor it, as
    in a scene whose references are mapped through the resolver at run time.
    """
    if _URL.match(reference) and not reference.startswith("file:"):
        return "remote"
    if "/" not in reference and reference.lower().endswith(".mdl"):
        return "runtime"
    return "file" if plain_path(reference).startswith("/") else "logical"


def _under(path: Path, folder: Path) -> str | None:
    try:
        return path.relative_to(folder).as_posix()
    except ValueError:
        return None


def trusted_by_roots(broker, stores: list[Path]):
    """Return a check that a real path is under an approved root or the asset root (*stores*).

    Pass the asset root only. A project file is found by walking up from the working directory, so a
    cloned repository could name any folder: that must not count as an approval. A project's files
    stay usable by approving their folder as a root.
    """
    folders = [root.resolve() for root, _ in (broker.roots.values() if broker else ())]
    folders += [store.resolve() for store in stores]

    def check(path: Path) -> bool:
        return any(_under(path.resolve(), folder) is not None for folder in folders)

    return check


def build_pack(
    root: Path,
    pack_id: str,
    version: str,
    sources: list[str],
    closure,
    *,
    from_pack_dirs: list[Path] = (),
    layout_root: Path | None = None,
    trusted=None,
    allow_unresolved: bool = False,
    dry_run: bool = False,
    notes: list | None = None,
    urdf_root=None,
    defaults: dict | None = None,
    inherit: dict | None = None,
) -> dict:
    """Copy USD or URDF files and only what they depend on into a new pack version, keeping relative layout.

    A source is an absolute path or, with *from_pack_dirs*, a path inside that pack. A source
    outside both *from_pack_dirs* and *layout_root* (a scene whose references are logical names)
    is read for its references and not copied. Those names are looked up in *from_pack_dirs*.

    A file copied out of a pack keeps that pack's record (source, licence, ``redistributable``,
    ``asset_id``): *inherit* maps each source pack folder to its records by path. Any other file takes
    *defaults* (``source``, ``license``, ``redistributable``), which are then required.

    Anything unresolved, remote or out of bounds refuses the whole build unless
    *allow_unresolved*, which still copies nothing from outside. Runtime MDL names are reported
    and never block. The new version must not exist; the copy is checked afterwards by computing the
    closure in the new pack.
    """
    defaults = defaults or dict(redistributable=False)
    inherit = {Path(k): v for k, v in (inherit or {}).items()}
    new_version(root, pack_id, version)
    file_layout: dict[str, Path | None] = {}  # relative path -> the layout folder it was found under
    notes = [] if notes is None else notes
    from_dirs = [Path(d) for d in from_pack_dirs]
    base_of: dict[str, Path | None] = {}  # queued real path -> layout folder, None = read only
    queue: list[str] = []
    search_folder: dict[str, Path] = {}  # URDF -> the folder its ROS packages are looked for in
    urdf_projects: dict = {}

    def layout_for(path: Path) -> Path | None:
        for folder in ([layout_root] if layout_root else []) + from_dirs:
            if _under(path, folder) is not None:
                return folder
        return None

    def enqueue(path: Path, base: Path | None) -> None:
        if str(path) not in base_of:
            base_of[str(path)] = base
            queue.append(str(path))

    for source in sources:
        if source.startswith("/"):
            path = Path(source).resolve()
        else:
            rel = clean_relpath(source, "source")
            found = [d / rel for d in from_dirs if (d / rel).is_file()]
            if not found:
                raise PackError(
                    f"source {source!r} is not an absolute path and is not in the source pack",
                    "NOT_FOUND",
                )
            path = found[0].resolve()
        if trusted is not None and not trusted(path):
            raise PackError(
                f"source is outside approved roots and the asset root: {path}. Put it in a pack under your asset root, or "
                "have the user approve its folder once, outside the tools: docs/TOOLS-SCENE.md, One-time approval. "
                "assets__roots lists what is approved.",
                "ROOT_NOT_APPROVED",
            )
        if not path.is_file():
            raise PackError(f"source not found: {path}", "NOT_FOUND")
        if urdf_refs.is_xacro(path):
            raise PackError(f"{path.name} is a xacro file. A pack holds what a file names, and a xacro file needs its arguments to say what "
                            "that is: import it with assets__import_path and xacro_args instead.", "XACRO")
        fmt = asset_formats.format_of(path)
        if path.suffix.lower() == ".xml" and path.name.lower() != "package.xml" and fmt != "mjcf":
            raise PackError(f"{path.name} is not a MuJoCo model.", "UNSUPPORTED_FORMAT")
        is_urdf = urdf_refs.is_urdf(path) or fmt in ("mjcf", "sdf", "mesh", "map")  # read on the host: the file and what it names are copied
        if path.suffix.lower() not in USD_SUFFIXES and not is_urdf:
            raise PackError(f"source is not a file a pack can hold (USD, URDF, MuJoCo XML, SDF, mesh or ROS map): {path}", "UNSUPPORTED_FORMAT")
        base = layout_for(path)
        if is_urdf:
            base = base or (urdf_root(path) if urdf_root else path.parent)
            search_folder[str(path)] = base
            # A package known only by the name of this folder: lay out from one level up, so the copy
            # holds a folder of that name. Each file still has to pass the trust check below.
            peek = run_closure(closure, [str(path)], search_folder.__getitem__, urdf_projects)[0]
            if peek.get("root_folder_package"):
                base = base.parent
        elif base is None and not from_dirs:
            base = path.parent
        enqueue(path, base)

    files: dict[str, Path] = {}
    entries: set[str] = set()
    blocked: dict[str, list] = dict(unresolved=[], remote=[], outside=[], unreadable=[])
    runtime: set[str] = set()
    file_base: dict[Path, Path] = {}
    missing_textures: list[dict] = []
    urdf_unfollowable: list[dict] = []
    urdf_packages: dict = {}
    while queue:
        batch, queue[:] = list(queue), []
        for result in run_closure(closure, batch, search_folder.__getitem__, urdf_projects):
            origin = Path(result["path"])
            base = base_of[str(origin)]
            if result.get("error"):
                code = "XACRO" if "xacro" in result["error"].lower() else "USD_UNREADABLE"
                raise PackError(f"cannot read {origin}: {result['error']}", code, file=str(origin))
            runtime.update(result.get("runtime", []))
            if base is not None:
                entries.add(origin.relative_to(base).as_posix())
            for item in result.get("problems", []):
                blocked[item["kind"]].append(
                    dict(reference=item["reference"], reason=item["reason"], referenced_by=str(origin)))
            missing_textures += [{**item, "urdf": origin.name} for item in result.get("missing_textures", [])]
            urdf_unfollowable += result.get("unfollowable", [])
            urdf_packages.update(result.get("packages", {}))
            for dep in [*result["layers"], *result["assets"]]:
                if _URL.match(dep) and not dep.startswith("file:"):
                    blocked["remote"].append(dict(reference=dep, referenced_by=str(origin)))
                    continue
                real = Path(plain_path(dep)).resolve()
                if base is None and real == origin:
                    continue  # the read-only scene itself
                rel = _under(real, base) if base is not None else None
                if rel is None or (trusted is not None and not trusted(real)):
                    blocked["outside"].append(dict(file=str(real), referenced_by=str(origin)))
                    continue
                rel = clean_relpath(rel, "dependency")
                if rel in files and files[rel] != real:
                    raise PackError(f"two different files map to {rel}: {files[rel]} and {real}", "CONFLICT")
                files[rel] = real
                file_base[real] = base
                file_layout[rel] = base
            for reference in result["unresolved"]:
                kind = classify_reference(reference)
                holder = result.get("referenced_by", {}).get(reference, str(origin))
                if kind == "runtime":
                    runtime.add(reference)
                elif kind == "remote":
                    blocked["remote"].append(dict(reference=reference, referenced_by=holder))
                elif kind == "logical":
                    holders = [d for d in from_dirs if (d / reference).is_file()]
                    if holders:
                        enqueue((holders[0] / reference).resolve(), holders[0])
                    else:
                        blocked["unresolved"].append(dict(reference=reference, referenced_by=holder))
                elif Path(plain_path(reference)).exists():
                    blocked["unreadable"].append(dict(reference=reference, referenced_by=holder))
                else:
                    blocked["unresolved"].append(dict(reference=reference, referenced_by=holder))
    # USD stops at MDL and MaterialX files. Follow what they name, as import validation does.
    followed = material_refs.follow(file_base)
    for item in followed["references"]:
        real, holder = item["target"].resolve(), str(item["holder"])
        base = file_base[item["holder"]] if item["holder"] in file_base else None
        if base is None:  # a nested MaterialX document: it sits in its parent's layout
            base = next((b for b in file_base.values() if _under(item["holder"], b) is not None), None)
        rel = _under(real, base) if base is not None else None
        if rel is None or (trusted is not None and not trusted(real)):
            blocked["outside"].append(dict(file=str(real), referenced_by=holder))
        elif not item["exists"]:
            blocked["unresolved"].append(dict(reference=str(real), referenced_by=holder))
        else:
            rel = clean_relpath(rel, "material dependency")
            if rel in files and files[rel] != real:
                raise PackError(f"two different files map to {rel}: {files[rel]} and {real}", "CONFLICT")
            files[rel] = real
            file_layout[rel] = base
    if not files:
        raise PackError("nothing to copy: no source is inside the layout or the source pack", "NOT_FOUND")
    if missing_textures or urdf_unfollowable:
        notes.append("Some meshes name files that are absent or could not be read: see missing_textures and "
                     "unfollowable. Geometry still imports, and self_contained is false.")
    for rel in files:
        if rel == MANIFEST_NAME:
            raise PackError(f"{rel} in the closure would collide with the pack's own manifest", "CONFLICT")
    records = {rel: inherit.get(file_layout.get(rel), {}).get(rel) for rel in files if file_layout.get(rel) in inherit}
    if "source" not in defaults or "license" not in defaults:
        lacking = sorted(rel for rel in files if rel not in records or records[rel] is None)
        if lacking:
            raise PackError("source and license are required for files that do not come from a pack, so a record "
                            "can say where they came from: " + ", ".join(lacking[:10]) + (" ..." if len(lacking) > 10 else ""),
                            "MISSING_PROVENANCE")
    from . import mesh_worker   # standard library at import time; reads the report assets__mesh left beside a file

    for rel, path in files.items():
        collision = mesh_worker.collision_record(path) if path.suffix.lower() in USD_SUFFIXES else None
        if collision:
            records[rel] = dict(records.get(rel) or {}, collision=collision)
    report = dict(
        files=len(files), bytes=sum(p.stat().st_size for p in files.values()), entries=sorted(entries),
        # What to pass as sim__quick_start's robot: a pack keeps the source's folder layout, so it isn't just <pack>/<file>: it is <id>/<version>/<entry>.
        load_as=[f"{pack_id}/{version}/{entry}" for entry in sorted(entries)
                 if entry.lower().endswith((".urdf", ".xml", ".sdf", ".world", ".gltf", ".glb", ".obj", ".stl", ".fbx", ".ply", ".yaml", ".yml", ".usd", ".usda", ".usdc", ".usdz"))],
        unresolved=blocked["unresolved"][:MAX_REPORTED], remote=blocked["remote"][:MAX_REPORTED],
        outside=blocked["outside"][:MAX_REPORTED], unreadable=blocked["unreadable"][:MAX_REPORTED],
        runtime_materials=sorted(runtime), material_references=len(followed["references"]),
        unfollowable=[*followed["unfollowable"], *urdf_unfollowable][:MAX_REPORTED],
        missing_textures=missing_textures[:MAX_REPORTED],
        **({"urdf_packages": urdf_packages} if urdf_packages else {}),
    )
    if followed["unfollowable"]:
        notes.append(UNFOLLOWABLE_NOTE)
    if any(blocked.values()) and not allow_unresolved:
        listed = [
            f"  {kind}: {item.get('reference') or item.get('file')} (in {item['referenced_by']})"
            + (f": {item['reason']}" if item.get("reason") else "")
            for kind, items in blocked.items() for item in items
        ]
        raise PackError(
            "The closure is incomplete, so nothing was copied. Fix the references, or pass "
            "allow_unresolved=true to build without them (files outside are never copied):\n"
            + "\n".join(listed[:20]) + (f"\n  ... and {len(listed) - 20} more" if len(listed) > 20 else ""),
            "UNRESOLVED_DEPENDENCIES",
            **{k: v[:MAX_REPORTED] for k, v in blocked.items() if v},
        )
    if dry_run:
        return {**report, "dry_run": True, "copied": 0, "planned": sorted(files)[:MAX_REPORTED]}
    accepted = sorted({_accepted_name(item["reference"], base_of) for item in blocked["unresolved"]})
    written = write_version(root, pack_id, version, files, records, defaults, notes,
                            extra=dict(accepted_unresolved=accepted) if accepted else None)
    folder = Path(written["pack_dir"])
    try:
        verified = check_closure(folder, _root_files([dict(path=rel) for rel in files]), closure, accepted)
        verified.pop("used", None)
    except PackError as exc:
        verified = dict(checked=False, reason=str(exc))
    blockers = _import_blockers(verified)
    if blockers:
        notes.append(
            "This pack will not import with assets__import_path until these files are added: "
            + ", ".join(blockers[:10]) + (" ..." if len(blockers) > 10 else "")
        )
    return {**report, "dry_run": False, "copied": written["files"], "pack": written["pack"], "pack_dir": written["pack_dir"],
            "verified": verified, "importable": None if blockers is None else not blockers,
            **({"import_blockers": blockers} if blockers else {})}


def _accepted_name(reference: str, base_of: dict) -> str:
    """Name an accepted unresolved file relative to its layout, so it stays valid in the pack."""
    for base in {b for b in base_of.values() if b is not None}:
        rel = _under(Path(plain_path(reference)), base)
        if rel is not None:
            return rel
    return reference


def _import_blockers(verified: dict) -> list[str] | None:
    """What would make assets__import_path refuse this pack, or None if the check itself did not run."""
    if not verified.get("checked"):
        return None
    labels: set[str] = set()
    for key in ("missing_entries", "unresolved", "remote", "outside_pack", "unreadable"):
        for item in verified.get(key, []):
            label = (item.get("reference") or item.get("file")) if isinstance(item, dict) else item
            if label:
                labels.add(str(label))
    labels.update(verified.get("accepted_unresolved", []))
    return sorted(labels)


UNFOLLOWABLE_NOTE = (
    "These material files could not be read statically, so their own files were not followed and "
    "self_contained is false. assets__import_path refuses the same files; the work cell's own loading "
    "and Kit's material library may still work."
)


def check_closure(folder: Path, entries: list[str], closure, accepted=()) -> dict:
    """Compute the closure of a pack's entries and sort every dependency into what it means."""
    root = folder.resolve()
    used: set[str] = set()
    out: dict[str, list] = dict(unresolved=[], remote=[], outside_pack=[], unreadable=[])
    runtime: set[str] = set()
    accepted = set(accepted)
    accepted_found: set[str] = set()
    present = [rel for rel in entries if (folder / rel).is_file()]
    missing = [rel for rel in entries if not (folder / rel).is_file()]
    urdf_projects: dict = {}
    missing_textures: list[dict] = []

    def add_unresolved(reference: str, holder: str, code: str | None = None, reason: str | None = None) -> None:
        rel = _under(Path(plain_path(reference)), root) or reference
        if rel in accepted:
            accepted_found.add(rel)
        else:
            item = dict(reference=rel, referenced_by=_short(holder, root))
            if code:
                item["code"] = code
            if reason:
                item["reason"] = reason
            out["unresolved"].append(item)

    results = run_closure(closure, [str(root / rel) for rel in present], lambda _: root, urdf_projects) if present else []
    for result in results:
        origin = result["path"]
        if result.get("error"):
            out["unreadable"].append(dict(reference=_short(origin, root), detail=result["error"]))
            continue
        runtime.update(result.get("runtime", []))
        for item in result.get("problems", []):
            if item["kind"] == "unresolved":
                add_unresolved(item["reference"], origin, item.get("code"), item.get("reason"))
            elif item["kind"] == "remote":
                out["remote"].append(dict(reference=item["reference"], referenced_by=_short(origin, root)))
            else:
                out["outside_pack"].append(dict(file=item["reference"], referenced_by=_short(origin, root)))
        missing_textures += [{**item, "urdf": _short(origin, root)} for item in result.get("missing_textures", [])]
        for dep in [*result["layers"], *result["assets"]]:
            if _URL.match(dep) and not dep.startswith("file:"):
                out["remote"].append(dict(reference=dep, referenced_by=_short(origin, root)))
                continue
            real = Path(plain_path(dep)).resolve()
            rel = _under(real, root)
            if rel is None:
                out["outside_pack"].append(dict(file=str(real), referenced_by=_short(origin, root)))
            else:
                used.add(rel)
        for reference in result["unresolved"]:
            holder = result.get("referenced_by", {}).get(reference, origin)
            kind = classify_reference(reference)
            if kind == "runtime":
                runtime.add(reference)
            elif kind == "remote":
                out["remote"].append(dict(reference=reference, referenced_by=_short(holder, root)))
            elif kind == "file" and Path(plain_path(reference)).exists():
                out["unreadable"].append(dict(reference=_short(reference, root), referenced_by=_short(holder, root)))
            else:
                add_unresolved(reference, holder)
    followed = material_refs.follow(root / rel for rel in sorted(used))
    for item in followed["references"]:
        holder = str(item["holder"])
        rel = _under(item["target"].resolve(), root)
        if rel is None:
            out["outside_pack"].append(dict(file=str(item["target"]), referenced_by=_short(holder, root)))
        elif item["exists"]:
            used.add(rel)
        else:
            add_unresolved(str(item["target"]), holder)
    # A mesh or map is also a root of its own check. One that a URDF or another root names is checked as part
    # of that file, and an importer's refusal to take it as a prop on its own (a .dae) is not a problem.
    out["unreadable"] = [u for u in out["unreadable"] if "detail" not in u or u["reference"] not in used
                         or PurePosixPath(u["reference"]).suffix.lower() in USD_SUFFIXES]
    out["unfollowable"] = [dict(file=_short(i["file"], root), reason=i["reason"]) for i in followed["unfollowable"]]
    out["unfollowable"] += [item for result in results for item in result.get("unfollowable", [])]
    out["missing_textures"] = missing_textures
    for key in out:
        unique = {json.dumps(item, sort_keys=True): item for item in out[key]}
        out[key] = [unique[k] for k in sorted(unique)]
    return dict(
        checked=True, entries=entries, missing_entries=missing, used=sorted(used),
        accepted_unresolved=sorted(accepted_found), runtime_materials=sorted(runtime),
        self_contained=not (missing or any(out.values()) or accepted_found), **out,
    )


def _short(path: str, root: Path) -> str:
    return _under(Path(plain_path(path)), root) or path


# ---------------------------------------------------------------------------
# Writing a new version. A version is written once, into a hidden staging folder under the asset root, checked
# with the same verifier a reader uses, and renamed into place only if it verifies. An existing version is never
# modified, and a failed write leaves nothing behind.
# ---------------------------------------------------------------------------

MAX_PROVENANCE_CHARS = 300


def clean_text(value, what: str) -> str:
    """A required one-line note (source or licence)."""
    if (not isinstance(value, str) or not value.strip() or len(value) > MAX_PROVENANCE_CHARS
            or any(ord(c) < 32 or ord(c) == 127 for c in value)):
        raise PackError(f"{what}: one line of text, 1 to {MAX_PROVENANCE_CHARS} characters", "INVALID_VALUE")
    return value.strip()


def split_ref(value, what: str = "pack") -> tuple[str, str]:
    try:
        return pack_manifest.split_pack_ref(value, what)
    except pack_manifest.ManifestError as exc:
        raise PackError(str(exc), "INVALID_PACK") from exc


def pack_dir(root: Path, pack_id: str, version: str) -> tuple[Path, Path]:
    """Validate a destination; return ``(root, <root>/<id>/<version>)``. Nothing may land in the repository."""
    try:
        root, folder = pack_manifest.pack_directory(root, pack_id, version)
    except pack_manifest.ManifestError as exc:
        raise PackError(str(exc), "UNSAFE_DESTINATION" if "escapes" in str(exc) else "INVALID_PACK") from exc
    if inside_repo(root) or inside_repo(folder):
        raise PackError(
            f"refusing an asset root inside the repository checkout: {root}\n"
            "Private assets must never land in git; choose a directory outside it (see `assets.py where`).",
            "INSIDE_REPOSITORY",
        )
    return root, folder


def new_version(root: Path, pack_id: str, version: str) -> Path:
    """The destination of a write: a version that does not exist yet. Anything already there is refused."""
    root, folder = pack_dir(root, pack_id, version)
    if (root / pack_id).is_symlink():
        raise PackError(f"refusing symlinked pack folder: {root / pack_id}", "UNSAFE_DESTINATION")
    if folder.exists() or folder.is_symlink():
        raise PackError(
            f"{pack_id}/{version} already exists in {root}. A pack version is immutable once it exists: nothing "
            f"was changed. Write a new version instead, for example {pack_id}/v2.", "VERSION_EXISTS")
    return folder


class Staging:
    """``<root>/.staging-<token>/<id>/<version>``: the version as it will look, built where nothing reads it."""

    def __init__(self, root: Path, pack_id: str, version: str, notes: list | None = None):
        self.root, self.pack_id, self.version, self.notes = root, pack_id, version, notes
        self.final = root / pack_id / version
        self.base = root / f".staging-{secrets.token_hex(6)}"
        self.path = self.base / pack_id / version

    def __enter__(self) -> "Staging":
        os.makedirs(self.root, mode=0o755, exist_ok=True)  # the root's own permissions are the user's business
        os.mkdir(self.base, 0o755)
        os.makedirs(self.path, mode=0o755)
        return self

    def verify(self) -> None:
        """The pack must pass the strict reader before it is moved into place."""
        try:
            pack_manifest.AssetPack(self.base, self.pack_id, self.version)
        except pack_manifest.ManifestError as exc:
            raise PackError(f"the new pack does not verify, so nothing was written: {exc}", "NOT_VERIFIED") from exc

    def commit(self) -> None:
        private_dirs(self.final.parent, self.notes)
        if self.final.exists() or self.final.is_symlink():
            raise PackError(f"{self.pack_id}/{self.version} appeared while it was being written; nothing was replaced",
                            "VERSION_EXISTS")
        os.rename(self.path, self.final)
        private_dirs(self.final, self.notes)

    def __exit__(self, *exc) -> None:
        shutil.rmtree(self.base, ignore_errors=True)


def defaults_from(source, license_, redistributable, *, required: bool) -> dict:
    """What a file with no record of its own gets. ``redistributable`` is false unless said."""
    if redistributable is not None and type(redistributable) is not bool:
        raise PackError("redistributable must be true or false", "INVALID_VALUE")
    if required and (source is None or license_ is None):
        raise PackError("source and license are required: say where these files came from and under which terms "
                        "(for example source=\"vendor CAD, 2026-11\", license=\"private lab use only\").",
                        "MISSING_PROVENANCE")
    out: dict = dict(redistributable=bool(redistributable))
    if source is not None:
        out["source"] = clean_text(source, "source")
    if license_ is not None:
        out["license"] = clean_text(license_, "license")
    return out


def make_record(rel: str, path: Path, base: dict | None, defaults: dict) -> dict:
    """One manifest record. A file that came from a pack keeps that pack's record (source, licence,
    ``redistributable``, ``asset_id``, extra fields); anything else takes the defaults."""
    base = dict(base or {})
    record: dict = {}
    if "asset_id" in base:
        record["asset_id"] = base["asset_id"]
    record["path"], record["sha256"] = rel, sha256(path)
    for key in ("source", "license", "redistributable"):
        value = base.get(key, defaults.get(key))
        if value is None:
            raise PackError(f"{rel}: no source or license on record. Pass source and license.", "MISSING_PROVENANCE")
        record[key] = value
    record.update({k: v for k, v in base.items() if k not in record and k not in ("path", "sha256")})
    return record


def write_version(root: Path, pack_id: str, version: str, files: dict[str, Path], records: dict[str, dict],
                  defaults: dict, notes: list, extra: dict | None = None) -> dict:
    """Copy *files* into a new version, write its ``pack.yaml``, check it, and move it into place."""
    if not files:
        raise PackError("nothing to write: no files", "NOT_FOUND")
    if len(files) > MAX_ADOPT_FILES:
        raise PackError(f"more than {MAX_ADOPT_FILES} files; narrow the folder", "LIMIT")
    folder = new_version(root, pack_id, version)
    with Staging(folder.parent.parent, pack_id, version, notes) as staging:
        install_many(files, staging.path)
        manifest = dict(id=pack_id, version=version, **(extra or {}),
                        files=[make_record(rel, staging.path / rel, records.get(rel), defaults) for rel in sorted(files)])
        _replace_text(staging.path, MANIFEST_NAME, pack_manifest.manifest_text(manifest))
        staging.verify()
        staging.commit()
    return dict(pack=f"{pack_id}/{version}", pack_dir=str(folder), files=len(files),
                bytes=sum((folder / rel).stat().st_size for rel in files))


def adopt_folder(root: Path, pack_id: str, version: str, source: Path, *, source_note, license_, redistributable=None,
                 include=(), names=None, dry_run=False, notes: list | None = None, allow_all_types=False) -> dict:
    """Copy a folder's asset files into a new ``<id>/<version>`` and write its ``pack.yaml``."""
    notes = [] if notes is None else notes
    defaults = defaults_from(source_note, license_, redistributable, required=True)
    suffixes = None if allow_all_types else ADOPT_SUFFIXES | {("." + str(e).lstrip(".")).lower() for e in include}
    skipped: list[str] = []
    files = named_sources(source, names, suffixes or set(), skipped) if names else adopt_sources(source, suffixes, skipped)
    if skipped:
        notes.append(f"{len(skipped)} symbolic link(s) were not copied (they are never followed): "
                     + ", ".join(sorted(skipped)[:10]) + (" ..." if len(skipped) > 10 else ""))
    extra = {"skipped_symlinks": sorted(skipped)[:MAX_REPORTED]} if skipped else {}
    if not files:
        raise PackError(f"no asset files found in {source}", "NOT_FOUND", notes=notes)
    new_version(root, pack_id, version)  # refuse before any work, and in a dry run too
    if dry_run:
        return dict(pack=f"{pack_id}/{version}", dry_run=True, files=len(files), planned=sorted(files)[:MAX_REPORTED],
                    truncated=len(files) > MAX_REPORTED, notes=notes, **extra)
    written = write_version(root, pack_id, version, files, {}, defaults, notes)
    return dict(written, dry_run=False, notes=notes, **extra)


# The pins file of the old pack format (before pack.yaml). Packs in that format may still be on lab machines' disks,
# so the converter leaves it out of the new version.
OLD_FORMAT_FILES = ("SHA256SUMS",)


def convert_old(old_folder: Path, *, pack_id: str | None = None, version: str = "v1", source_note, license_,
                redistributable=None, dry_run=False, notes: list | None = None) -> dict:
    """Convert an old-format pack ``<store>/<pack>/`` into ``<store>/<pack>/<version>/`` with a ``pack.yaml``.

    Hashes come from the files themselves, not from the old ``SHA256SUMS``, which is not copied. The old files stay where they are; nothing reads them once the new version exists.
    """
    notes = [] if notes is None else notes
    old_folder = Path(old_folder)
    if old_folder.is_symlink() or not old_folder.is_dir():
        raise PackError(f"not a pack folder (or it is a symlink): {old_folder}", "NOT_FOUND")
    pack_id = pack_id or old_folder.name
    root = old_folder.parent.resolve()
    if (old_folder.resolve() != root / pack_id):
        raise PackError(f"the pack id must be the folder's name ({old_folder.name}); the new version is written "
                        f"inside it as {old_folder.name}/{version}", "INVALID_PACK")
    defaults = defaults_from(source_note, license_, redistributable, required=True)
    skipped: list[str] = []
    files = {rel: path for rel, path in adopt_sources(old_folder.resolve(), None, skipped).items()
             if rel not in OLD_FORMAT_FILES}
    if skipped:
        notes.append(f"{len(skipped)} symbolic link(s) were not copied (they are never followed): "
                     + ", ".join(sorted(skipped)[:10]) + (" ..." if len(skipped) > 10 else ""))
    new_version(root, pack_id, version)
    if not files:
        raise PackError(f"no files in {old_folder}", "NOT_FOUND")
    if dry_run:
        return dict(pack=f"{pack_id}/{version}", dry_run=True, files=len(files), planned=sorted(files)[:MAX_REPORTED],
                    truncated=len(files) > MAX_REPORTED, notes=notes)
    written = write_version(root, pack_id, version, files, {}, defaults, notes)
    notes.append(f"The old files are still in {old_folder}; nothing reads them. Delete them yourself once "
                 f"{pack_id}/{version} has passed `doctor`.")
    return dict(written, dry_run=False, notes=notes, converted_from=str(old_folder))


# ---------------------------------------------------------------------------
# Pin: rewrite the sha256 values of an existing manifest
# ---------------------------------------------------------------------------


def pin_changes(root: Path, pack_id: str, version: str) -> dict:
    """Compare a manifest's hashes with the files, hashing every file. Read only."""
    try:
        _, directory = pack_manifest.pack_directory(root, pack_id, version)
        manifest = pack_manifest.read_manifest(directory, pack_id, version)
    except pack_manifest.ManifestError as exc:
        raise PackError(str(exc), "UNSAFE_DESTINATION" if "escapes" in str(exc) else "PACK_NOT_FOUND") from exc
    checker = pack_manifest._Checker(directory)  # same path rules as the reader
    changed, missing, unchanged = [], [], 0
    for record in manifest["files"]:
        try:
            path = checker.check(record, hash_file=False)
        except pack_manifest.ManifestError as exc:
            if str(exc).startswith("Missing pack file"):
                missing.append(record["path"])
                continue
            raise PackError(f"{exc}. pin only rewrites hashes; fix the manifest or write a new version.", "UNPINNABLE") from exc
        digest = sha256(path)
        if digest == record["sha256"]:
            unchanged += 1
        else:
            changed.append(dict(file=record["path"], pinned=record["sha256"], actual=digest))
    unlisted = [p.relative_to(directory).as_posix() if p.is_relative_to(directory) else str(p) for p in checker.unlisted()]
    return dict(manifest=manifest, directory=directory, changed=changed, missing=missing, unlisted=unlisted,
                unchanged=unchanged)


def pin_pack(root: Path, pack_id: str, version: str, *, force: bool = False, files: list | None = None,
             dry_run: bool = False, partial: bool = False) -> dict:
    """Rewrite the ``sha256`` of changed files in an existing ``pack.yaml``.

    A version that already verifies has nothing to pin. A changed hash accepts whatever bytes are there,
    damage included, so it is refused unless the file is named in *files* or *force* is set (*partial*: the
    caller's own decision, so changed files that are not named are left alone instead of refused). Files that
    are missing or not listed cannot be fixed by a hash: write a new version with adopt or build.
    """
    report = pin_changes(root, pack_id, version)
    directory = report["directory"]
    if report["missing"] or report["unlisted"]:
        raise PackError(
            "pin only rewrites hashes of listed files that exist. Missing files and files the manifest does not "
            "list need a new version: write one with adopt or build.", "UNPINNABLE",
            missing=report["missing"][:MAX_REPORTED], unlisted=report["unlisted"][:MAX_REPORTED])
    named = {clean_relpath(f, "file") for f in (files or [])}
    changed = {item["file"] for item in report["changed"]}
    unknown = sorted(named - changed)
    if unknown:
        raise PackError(f"not changed files of this pack: {unknown[:10]}", "NOT_FOUND")
    blocked = [] if partial else sorted(rel for rel in changed if not force and rel not in named)
    if blocked and not dry_run:
        raise PackError(
            "These hashes would change. Check each file, then name it in files, or pass force=true only if every "
            "change is intended.", "PIN_CHANGED", changed=[c for c in report["changed"] if c["file"] in blocked][:MAX_REPORTED])
    accepted = [c for c in report["changed"] if (force and not partial) or c["file"] in named or (partial and not named)]
    if not report["changed"]:
        return dict(pinned=0, changed=[], unchanged=report["unchanged"], verifies=True,
                    note="Already verifies: nothing to pin. A verified version is not modified.", pack_dir=str(directory))
    if dry_run:
        return dict(dry_run=True, would_change=report["changed"][:MAX_REPORTED], pack_dir=str(directory))
    by_path = {c["file"]: c["actual"] for c in accepted}
    # A collision summary describes the split's files, which sit in one folder: a changed file there makes it stale.
    stale = {PurePosixPath(rel).parent for rel in by_path}
    manifest, dropped = report["manifest"], []
    for record in manifest["files"]:
        if record["path"] in by_path:
            record["sha256"] = by_path[record["path"]]
        if "collision" in record and PurePosixPath(record["path"]).parent in stale:
            del record["collision"]
            dropped.append(record["path"])
    _replace_text(directory, MANIFEST_NAME, pack_manifest.manifest_text(manifest))
    return dict(pinned=len(accepted), changed=accepted[:MAX_REPORTED], verifies=len(accepted) == len(report["changed"]),
                unchanged=report["unchanged"], pack_dir=str(directory),
                **({"collision_dropped": dropped} if dropped else {}),
                note="pack.yaml was rewritten as JSON (valid YAML); comments in it are not kept."
                     + (" The collision counts of files beside a changed file were removed: they no longer describe "
                        "the files." if dropped else ""))


# ---------------------------------------------------------------------------
# Doctor
# ---------------------------------------------------------------------------


def _root_files(records: list[dict]) -> list[str]:
    """Every listed USD, URDF, MuJoCo, SDF, mesh or map file: each is a root of the dependency check."""
    return sorted(
        rec["path"] for rec in records
        if PurePosixPath(rec["path"]).suffix.lower() in USD_SUFFIXES or urdf_refs.is_urdf(rec["path"])
        or asset_formats.format_of(rec["path"]) in ("mjcf", "sdf", "mesh", "map"))


def doctor_pack(root: Path, pack_id: str, version: str, *, closure=None) -> dict:
    """Verify a pack against its manifest, then check the dependency closure of its USD, URDF and MJCF files.

    Read only. Every file must be listed and match its hash, and every dependency must be a listed file of
    the same pack: a reference to something outside it, or to a missing file, is a problem. ``ok`` is false
    when the dependency check could not run: a clean file listing proves nothing about it.
    """
    report = pack_manifest.inspect_pack(root, pack_id, version)
    if report.get("directory") and not (Path(report["directory"]) / MANIFEST_NAME).exists():
        raise PackError(f"pack {pack_id}/{version} is not in {root}: " + "; ".join(report["problems"]), "PACK_NOT_FOUND")
    records = report.pop("records", [])
    result = dict(pack=f"{pack_id}/{version}", pack_dir=report.get("directory"),
                  files=dict(total=report["files"], bytes=report["bytes"], verified=report["verified"]))
    problems = list(report["problems"])
    accepted: list[str] = []
    if report.get("directory"):
        try:
            manifest = pack_manifest.read_manifest(Path(report["directory"]), pack_id, version)
            accepted = [a for a in manifest.get("accepted_unresolved", []) if isinstance(a, str)]
        except pack_manifest.ManifestError:
            pass
    chosen = _root_files(records)
    dependencies = None
    reason = unavailable_message()
    if not report.get("directory"):
        reason = "The pack could not be read, so its dependencies were not checked."
    elif not chosen:
        # Nothing to parse (textures, JSON): the closure is trivially complete.
        dependencies = dict(checked=True, entries=[], missing_entries=[], unresolved=[], remote=[], outside_pack=[],
                            unreadable=[], unfollowable=[], missing_textures=[], accepted_unresolved=[],
                            runtime_materials=[], self_contained=True, used=[])
        closure = None
    elif closure is not None and (getattr(closure, "available", lambda: True)()
                                  or not any(PurePosixPath(c).suffix.lower() in USD_SUFFIXES for c in chosen)):
        # Only a USD file needs the parser: URDF, MJCF, SDF, meshes and maps are read here.
        try:
            dependencies = check_closure(Path(report["directory"]), chosen, closure, accepted)
        except PackError as exc:
            reason = str(exc)
    if dependencies is None:
        result["dependencies"] = dict(checked=False, reason=reason, entries=chosen[:MAX_REPORTED])
    else:
        dependencies.pop("used", None)
        result["dependencies"] = dependencies
        result["dependencies"]["backend"] = getattr(closure, "name", None)
        problems += [f"{rel}: entry file is missing" for rel in dependencies["missing_entries"]]
        problems += [f"{u['reference']}: unresolved, referenced by {u['referenced_by']}"
                     + (f". {u['reason']}" if u.get("reason") else "")
                     + (" The pack needs a folder named " + u["reference"].split("/")[2]
                        + ", or a package.xml naming it"
                        if u.get("code") == "PACKAGE_UNRESOLVED" and u["reference"].startswith("package://") else "")
                     for u in dependencies["unresolved"]]
        problems += [f"{u['reference']}: unreadable USD" for u in dependencies["unreadable"]]
        problems += [f"{u['reference']}: remote, referenced by {u['referenced_by']}" for u in dependencies["remote"]]
        problems += [f"{u['file']}: outside the pack, referenced by {u['referenced_by']}" for u in dependencies["outside_pack"]]
        problems += [f"{rel}: accepted as unresolved when the pack was built, so assets__import_path refuses "
                     "this pack until a new version holds the file" for rel in dependencies["accepted_unresolved"]]
        if dependencies["unfollowable"]:
            result["warnings"] = [UNFOLLOWABLE_NOTE]
        if dependencies["missing_textures"]:
            result.setdefault("warnings", []).append(
                "Meshes name files that are absent: " + ", ".join(i["reference"] for i in dependencies["missing_textures"][:10]))
    result["problems"] = problems[:MAX_REPORTED]
    result["ok"] = not problems and dependencies is not None
    result["dependencies_checked"] = dependencies is not None
    return result


# ---------------------------------------------------------------------------
# MCP tools (host side)
# ---------------------------------------------------------------------------


def _context(root: "pack_manifest.AssetRoot | None", error: str | None = None) -> dict:
    """What this process resolved; the host and Kit can differ if their settings differ."""
    project = pack_manifest.project_file()
    try:
        selected = pack_manifest.project_pack()
    except ValueError as exc:  # an unreadable project file, or a pack that is not <id>/<version>
        raise PackError(str(exc), "INVALID_PACK") from exc
    out = dict(project_file=str(project) if project else None, selected_pack=selected,
               asset_root=str(root.path) if root else None, asset_root_source=root.source if root else None)
    if root and root.warnings:
        out["warnings"] = list(root.warnings)
    if error:
        out["asset_root_error"] = error
    return out


def _absolute(path, what: str) -> Path:
    if not isinstance(path, str) or not path.startswith("/"):
        raise PackError(f"{what} must be an absolute local path", "INVALID_PATH")
    if "\\" in path or ":" in path or any(ord(c) < 32 for c in path):
        raise PackError(f"{what}: URLs, backslashes and control characters are not supported", "INVALID_PATH")
    if any(part in ("", ".", "..") for part in path.split("/")[1:]):
        raise PackError(f"{what}: empty, dot and traversal components are forbidden", "INVALID_PATH")
    return Path(path)


def _approved_root(path: Path, broker) -> Path:
    """Return the approved root holding *path*, checking the root is still the approved one."""
    broker.require_profile()
    real = path.resolve()
    for root_id, (root, _) in broker.roots.items():
        if _under(real, root.resolve()) is not None:
            os.close(broker.root_fd(root_id))
            return root
    raise PackError(
        "The path is outside all user-approved roots." + approval_help(None),
        "ROOT_NOT_APPROVED",
    )


def _kit_closure(kit_call):
    if kit_call is None:
        return None

    def call(paths):
        try:
            result = kit_call("assets.dependencies", dict(paths=paths))
        except Exception as exc:  # noqa: BLE001 - no bridge, or a Kit without this tool
            raise PackError(f"{unavailable_message()} ({str(exc)[:300]})", "USD_UNAVAILABLE") from exc
        if not isinstance(result, dict) or "results" not in result:
            raise PackError(f"Kit could not compute the closure: {str(result)[:500]}", "USD_FAILED")
        return result["results"]

    return call


def _kit_environment(kit_call):
    """What the connected Kit resolves, or None when there is no Kit or it predates the report."""
    if kit_call is None:
        return None
    try:
        info = kit_call("sim.stage.info", {})
    except Exception:  # noqa: BLE001 - no bridge is the normal case for a host-side listing
        return None
    env = info.get("asset_environment") if isinstance(info, dict) else None
    return env if isinstance(env, dict) and "error" not in env else None


def environment_warnings(host: dict, kit: dict) -> list[str]:
    """Differences between the host's and Kit's asset settings that make the same call resolve differently."""
    out = []
    if host.get("asset_root") != kit.get("asset_root"):
        out.append(f"This host uses the asset root {host.get('asset_root')!r} but the connected Kit uses "
                   f"{kit.get('asset_root')!r}. Give both the same {pack_manifest.ROOT_ENV}, or Kit will not "
                   "see the packs the host lists.")
    if host.get("project_file") != kit.get("project_file"):
        out.append(f"This host uses the project file {host.get('project_file')!r} but the connected Kit uses {kit.get('project_file')!r}. "
                   "Start Kit with launch_simulator from this host, or set TWINSMITH_PROJECT to the project for Kit.")
    return out


def _pack_argument(params: dict, key: str, selected: str | None, what: str = "pack") -> tuple[str, str]:
    value = params.get(key) or (selected if key == "pack" else None)
    if not value:
        raise PackError(f"{what}: name it as \"<pack id>/<version>\" (the project file's 'pack' key also selects one).",
                        "INVALID_PACK")
    return split_ref(value, what)


def run_tool(canonical: str, params: dict, broker, kit_call=None) -> dict:
    """Run one host-side pack tool. Raises :class:`PackError` for refusals."""
    params = dict(params)
    try:
        root = pack_manifest.resolve_asset_root()
        root_error = None
    except pack_manifest.RootError as exc:
        root, root_error = None, str(exc)
    context = _context(root, root_error)

    def need_root() -> Path:
        if root is None:
            raise PackError(root_error, "NO_ASSET_ROOT")
        return root.path

    # Reads may report on the asset root; parsing USD and copying files need real approval.
    trusted = trusted_by_roots(broker, [root.path] if root else [])
    closure = ClosureBackend(kit_call=_kit_closure(kit_call), trusted=trusted)

    if canonical == "assets.packs":
        if root is None:
            return dict(status="ok", **context, listing=dict(root=None, packs=[], old_format=[]))
        report = dict(status="ok", **context, listing=pack_manifest.list_packs(root.path))
        host = dict(asset_root=context["asset_root"], project_file=context["project_file"])
        kit = _kit_environment(kit_call)
        if kit is not None:
            report["kit_environment"] = kit
            warnings = environment_warnings(host, kit)
            if warnings:
                report["warnings"] = [*report.get("warnings", []), *warnings]
        return report

    if canonical == "assets.pack_doctor":
        pack_id, version = _pack_argument(params, "pack", context["selected_pack"])
        return dict(status="ok", **context, **doctor_pack(need_root(), pack_id, version, closure=closure))

    if canonical == "assets.dependencies":
        paths = params.get("paths") or []
        if not paths or len(paths) > 64:
            raise PackError("paths must list 1..64 USD files", "LIMIT")
        checked = [_absolute(p, "path") for p in paths]
        broker.require_profile()
        for path in checked:
            if not trusted(path):
                raise PackError(
                    f"path is outside approved roots and the asset root: {path}", "ROOT_NOT_APPROVED"
                )

        def urdf_folder(path: str) -> Path:
            """Where a URDF's packages are looked for: its pack version, its approved root, or its own folder."""
            resolved = Path(path).resolve()
            if root is not None:
                rel = _under(resolved, root.path.resolve())
                if rel and len(rel.split("/")) > 2:
                    return root.path.resolve() / "/".join(rel.split("/")[:2])
            try:
                return _approved_root(resolved, broker).resolve()
            except PackError:
                return resolved.parent

        results = run_closure(closure, [str(p.resolve()) for p in checked], urdf_folder)
        backend = closure.name or ("host" if all(asset_formats.format_of(p) not in (None, "usd") for p in checked) else None)
        return dict(status="ok", backend=backend, results=results)

    if canonical == "assets.pack_adopt":
        pack_id, version = _pack_argument(params, "pack", None)
        dry_run = bool(params.get("dry_run", False))
        notes: list = list(context.get("warnings", []))
        common = dict(source_note=params.get("source"), license_=params.get("license"),
                      redistributable=params.get("redistributable"), dry_run=dry_run, notes=notes)
        if bool(params.get("from_old")) == bool(params.get("source_path")):
            raise PackError("Give exactly one of source_path (a folder under an approved root) or from_old "
                            "(the name of an old-format pack in the asset root).", "INVALID_VALUE")
        if params.get("from_old"):
            try:
                old_id = pack_manifest.identifier(params["from_old"])
            except pack_manifest.ManifestError as exc:
                raise PackError(f"from_old: the name of a pack folder in the asset root. {exc}", "INVALID_VALUE") from exc
            if old_id != pack_id:
                raise PackError(f"from_old={old_id!r} is written inside its own folder, so pack must start with "
                                f"{old_id}/, for example {old_id}/v1.", "INVALID_PACK")
            report = convert_old(need_root() / old_id, pack_id=old_id, version=version, **common)
            return dict(status="ok", **context, **{k: v for k, v in report.items() if k != "notes"}, notes=notes)
        src = _absolute(params["source_path"], "source_path")
        _approved_root(src, broker)
        real = src.resolve()
        if not real.is_dir():
            raise PackError(f"not a folder: {real}", "NOT_FOUND")
        target = need_root() / pack_id / version
        if _under(target.resolve(), real) is not None or _under(real, target.resolve()) is not None:
            raise PackError("source and pack folder overlap", "OVERLAP")
        report = adopt_folder(need_root(), pack_id, version, real, include=params.get("include") or (),
                              names=params.get("files"), **common)
        return dict(status="ok", **context, **{k: v for k, v in report.items() if k != "notes"}, notes=notes)

    if canonical == "assets.pack_pin":
        pack_id, version = _pack_argument(params, "pack", context["selected_pack"])
        report = pin_pack(need_root(), pack_id, version, force=bool(params.get("force", False)),
                          files=params.get("files"), dry_run=bool(params.get("dry_run", False)))
        return dict(status="ok", **context, **report)

    if canonical == "assets.pack_build":
        dest_id, dest_version = _pack_argument(params, "pack", None)
        sources = params.get("sources") or []
        if not sources or len(sources) > 64:
            raise PackError("sources must list 1..64 USD or URDF files", "LIMIT")
        broker.require_profile()
        asset_root = need_root()
        from_dirs, inherit = [], {}
        if params.get("from_pack"):
            source_id, source_version = _pack_argument(params, "from_pack", None, "from_pack")
            _, source_dir = pack_dir(asset_root, source_id, source_version)
            report = pack_manifest.inspect_pack(asset_root, source_id, source_version)
            if not report["verified"]:
                raise PackError(f"source pack {source_id}/{source_version} does not verify, so nothing is copied from it: "
                                + "; ".join(report["problems"][:3]), "PACK_NOT_VERIFIED")
            from_dirs = [source_dir]
            inherit = {source_dir: {r["path"]: r for r in pack_manifest.read_manifest(source_dir, source_id, source_version)["files"]}}
        for source in sources:
            if source.startswith("/"):
                _absolute(source, "source")
        layout = None
        if params.get("layout_root"):
            layout = _absolute(params["layout_root"], "layout_root").resolve()
            if not trusted(layout):
                raise PackError(f"layout_root is outside approved roots and the asset root: {layout}", "ROOT_NOT_APPROVED")
        defaults = defaults_from(params.get("source"), params.get("license"), params.get("redistributable"), required=False)
        # A URDF is read here, so a URDF-only build needs no USD parser.
        if not closure.available() and not all(urdf_refs.is_urdf(s) or urdf_refs.is_xacro(s) or asset_formats.format_of(s) in ("mjcf", "sdf", "mesh", "map") for s in sources):
            raise PackError(unavailable_message(), "USD_UNAVAILABLE")

        def urdf_root(path: Path) -> Path:
            """The pack version or approved root holding *path*, which is as far as its packages are looked for."""
            rel = _under(path, asset_root.resolve())
            if rel and len(rel.split("/")) > 2:
                return asset_root.resolve() / "/".join(rel.split("/")[:2])
            return _approved_root(path, broker).resolve()

        notes = list(context.get("warnings", []))
        report = build_pack(
            asset_root, dest_id, dest_version, sources, closure, from_pack_dirs=from_dirs, layout_root=layout,
            trusted=trusted, allow_unresolved=bool(params.get("allow_unresolved", False)),
            dry_run=bool(params.get("dry_run", False)), notes=notes, urdf_root=urdf_root, defaults=defaults, inherit=inherit,
        )
        return dict(status="ok", **context, backend=closure.name, notes=notes, **report)

    raise PackError(f"not a pack tool: {canonical}", "UNKNOWN")
