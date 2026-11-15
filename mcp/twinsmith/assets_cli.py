"""``twinsmith assets``: list, check, adopt, pin, build, pull and push asset packs: ``<asset root>/<pack id>/<version>/pack.yaml``.

One model: a pack version is a folder with a ``pack.yaml`` that lists every file with its sha256, source,
licence and ``redistributable`` flag. The asset root is ``$TWINSMITH_ASSET_ROOT``, else ``<workspace>/assets``
(see ``docs/ASSET-PACKS.md``). A version that exists is never modified: write a new version instead. Only
``pull`` downloads, and only what the owner's ``~/.config/twinsmith/pack-sources.toml`` pins.

Examples::

    twinsmith assets where
    twinsmith assets list
    twinsmith assets doctor ur5e-workcell/v3
    twinsmith assets adopt <folder> --pack my-robot/v1 --source "vendor CAD" --license "private"
    twinsmith assets adopt --from-old <root>/workcell --source "lab files" --license "private"
    twinsmith assets pin my-robot/v1
    twinsmith assets build robot/robot.usdc --pack small/v1 --from-pack my-robot/v1
    twinsmith assets pull --all
    twinsmith assets push my-robot/v1 --dry-run

From a checkout without installing: ``python3 scripts/assets.py <command>``.
Exit status: 0 success, 1 ``doctor`` or ``pull`` found a problem, 2 usage error or refusal.
Nothing is ever written inside this repository checkout.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import asset_packs as packs
from . import pack_manifest
from .asset_packs import PackError as CliError


def _root(args) -> Path:
    """The asset root: ``--root``, else the one resolution rule used everywhere."""
    if getattr(args, "root", None):
        return Path(args.root).expanduser().resolve()
    try:
        resolved = pack_manifest.resolve_asset_root()
    except pack_manifest.RootError as exc:
        raise CliError(str(exc), "NO_ASSET_ROOT") from exc
    for warning in resolved.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    return resolved.path


def _pack(args, key: str = "pack", what: str = "pack") -> tuple[str, str]:
    value = getattr(args, key, None)
    if not value and key == "pack":
        try:
            value = pack_manifest.project_pack()
        except ValueError as exc:
            raise CliError(str(exc)) from exc
    if not value:
        raise CliError(f"name the {what} as <pack id>/<version>")
    return packs.split_ref(value, what)


# ---------------------------------------------------------------------------
# where, list
# ---------------------------------------------------------------------------


def cmd_where(args) -> int:
    try:
        resolved = pack_manifest.resolve_asset_root()
        root, source, warnings, error = str(resolved.path), resolved.source, list(resolved.warnings), None
    except pack_manifest.RootError as exc:
        root, source, warnings, error = None, None, [], str(exc)
    try:
        selected = pack_manifest.project_pack()
    except ValueError as exc:
        selected, error = None, error or str(exc)
    project = pack_manifest.project_file()
    if args.json:
        print(json.dumps(dict(asset_root=root, source=source, project_file=str(project) if project else None,
                              selected_pack=selected, warnings=warnings, error=error), indent=2))
        return 0 if error is None else 2
    print(f"asset root:   {root or 'none'}" + (f" ({source})" if source else ""))
    print(f"project file: {project if project else 'none'}")
    print(f"pack:         {selected or 'none selected'}")
    for item in warnings:
        print(f"warning: {item}")
    if error:
        print(error, file=sys.stderr)
        return 2
    return 0


def cmd_list(args) -> int:
    root = _root(args)
    listing = pack_manifest.list_packs(root)
    if args.json:
        print(json.dumps(listing, indent=2))
        return 0
    print(f"asset root: {listing['root']}" + ("" if listing["exists"] else " (does not exist)"))
    for item in listing["packs"]:
        state = "verified" if item["verified"] else "NOT VERIFIED"
        print(f"  {item['id']}/{item['version']}: {item['files']} file(s), {_size(item['bytes'])}, {state}")
        for problem in item["problems"][:5]:
            print(f"      {problem}")
    for item in listing["old_format"]:
        print(f"  {item['id']}: old format. {item['hint']}")
    if not listing["packs"] and not listing["old_format"]:
        print("  no packs")
    return 0


# ---------------------------------------------------------------------------
# doctor
# ---------------------------------------------------------------------------


def cmd_doctor(args) -> int:
    root = _root(args)
    pack_id, version = _pack(args)
    closure = packs.ClosureBackend(args.usd_python)
    report = packs.doctor_pack(root, pack_id, version, closure=closure)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0 if report["ok"] else 1
    files, deps = report["files"], report["dependencies"]
    print(f"{report['pack']}: {files['total']} file(s), {_size(files['bytes'])}; "
          + ("every file matches pack.yaml" if files["verified"] else "does NOT verify"))
    if deps["checked"]:
        print(f"dependencies: checked ({deps.get('backend') or 'no parser needed'})"
              + ("; self-contained" if deps.get("self_contained") else ""))
    else:
        print(f"dependencies: NOT checked: {deps.get('reason')}")
    for problem in report["problems"]:
        print(f"  problem: {problem}")
    for warning in report.get("warnings", []):
        print(f"  warning: {warning}")
    print("ok" if report["ok"] else "NOT OK")
    return 0 if report["ok"] else 1


# ---------------------------------------------------------------------------
# adopt, pin, build
# ---------------------------------------------------------------------------


def _print_write(report: dict, as_json: bool) -> int:
    if as_json:
        print(json.dumps(report, indent=2))
        return 0
    if report.get("dry_run"):
        print(f"would write {report['pack']}: {report['files']} file(s)")
        for rel in report.get("planned", [])[:20]:
            print(f"  {rel}")
    else:
        print(f"wrote {report['pack']}: {report['files']} file(s), {_size(report['bytes'])}, into {report['pack_dir']}")
    for note in report.get("notes", []):
        print(note if note.startswith("note:") else f"note: {note}")
    return 0


def cmd_adopt(args) -> int:
    notes: list = []
    common = dict(source_note=args.source, license_=args.license, redistributable=args.redistributable or None,
                  dry_run=args.dry_run, notes=notes)
    if args.from_old:
        old = Path(args.from_old).expanduser().resolve()
        version = "v1"
        pack_id = old.name
        if args.pack:
            pack_id, version = packs.split_ref(args.pack, "--pack")
        return _print_write(packs.convert_old(old, pack_id=pack_id, version=version, **common), args.json)
    if not args.folder:
        raise CliError("give a folder to adopt, or --from-old <old pack folder>")
    pack_id, version = _pack(args)
    source = Path(args.folder).expanduser().resolve()
    if not source.is_dir():
        raise CliError(f"not a folder: {source}")
    root = _root(args)
    report = packs.adopt_folder(root, pack_id, version, source, include=args.include or (), names=args.file or None, **common)
    return _print_write(report, args.json)


def cmd_pin(args) -> int:
    root = _root(args)
    pack_id, version = _pack(args)
    shown = packs.pin_changes(root, pack_id, version)
    for item in shown["changed"]:
        print(f"changed: {item['file']}\n  pack.yaml: {item['pinned']}\n  file:      {item['actual']}")
    if not shown["changed"] and not shown["missing"] and not shown["unlisted"]:
        print(f"{pack_id}/{version} already verifies: nothing to pin.")
        return 0
    # The CLI is your own decision: it accepts every changed file, after showing it above.
    report = packs.pin_pack(root, pack_id, version, force=True, files=args.file or None, dry_run=args.dry_run,
                            partial=bool(args.file))
    print(json.dumps(report, indent=2) if args.json else
          ("dry run: nothing written" if args.dry_run else f"rewrote {report['pinned']} hash(es) in {report['pack_dir']}/pack.yaml"))
    return 0


def cmd_build(args) -> int:
    root = _root(args)
    dest_id, dest_version = _pack(args)
    from_dirs, inherit = [], {}
    if args.from_pack:
        source_id, source_version = packs.split_ref(args.from_pack, "--from-pack")
        _, source_dir = packs.pack_dir(root, source_id, source_version)
        check = pack_manifest.inspect_pack(root, source_id, source_version)
        if not check["verified"]:
            raise CliError(f"source pack {args.from_pack} does not verify: " + "; ".join(check["problems"][:3]), "PACK_NOT_VERIFIED")
        from_dirs = [source_dir]
        inherit = {source_dir: {r["path"]: r for r in pack_manifest.read_manifest(source_dir, source_id, source_version)["files"]}}
    closure = packs.ClosureBackend(args.usd_python)
    sources = [str(Path(s).expanduser().resolve()) if s.startswith(("/", "~", "./", "../")) or Path(s).exists() else s
               for s in args.sources]
    if not closure.available() and not all(packs.urdf_refs.is_urdf(s) for s in sources):
        raise CliError(packs.unavailable_message(), "USD_UNAVAILABLE")
    notes: list = []
    defaults = packs.defaults_from(args.source, args.license, args.redistributable or None, required=False)
    report = packs.build_pack(
        root, dest_id, dest_version, sources, closure, from_pack_dirs=from_dirs,
        layout_root=Path(args.layout_root).expanduser().resolve() if args.layout_root else None,
        allow_unresolved=args.allow_unresolved, dry_run=args.dry_run, notes=notes, defaults=defaults, inherit=inherit,
    )
    if args.json:
        print(json.dumps({**report, "notes": notes}, indent=2))
        return 0
    verb = "would copy" if args.dry_run else "wrote"
    print(f"{report['files']} file(s) in the closure, {_size(report['bytes'])}; {verb} {dest_id}/{dest_version}")
    print(f"entry files: {', '.join(report['entries'])}")
    for key in ("unresolved", "remote", "outside", "unreadable"):
        for item in report[key]:
            print(f"  {key}: {item.get('reference') or item.get('file')} (in {item['referenced_by']})")
    for item in report["unfollowable"]:
        print(f"  could not follow: {item['file']}: {item['reason']}")
    if report["runtime_materials"]:
        print(f"  runtime materials (Kit supplies these): {', '.join(report['runtime_materials'])}")
    for note in notes:
        print(f"note: {note}")
    verified = report.get("verified")
    if verified:
        print("checked the new pack: " + ("self-contained" if verified.get("self_contained") else "NOT self-contained"))
    return 0


# ---------------------------------------------------------------------------
# pull, push
# ---------------------------------------------------------------------------


def _size(n: int) -> str:
    return f"{n / 1e6:.1f} MB"


def cmd_pull(args) -> int:
    from . import pack_sources

    report = pack_sources.pull(_root(args), pack=args.pack, all_packs=args.all, progress=not args.json)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "ok" else 1
    for item in report["results"]:
        outcome = item["outcome"]
        if outcome in ("pulled", "up_to_date"):
            what = "pulled" if outcome == "pulled" else "up to date"
            print(f"  {item['pack']}: {what}, {item['files']} file(s), {_size(item['bytes'])}, verified "
                  f"({item['repo']} @ {item['revision'][:12]})")
        elif outcome == "vendor":
            print(f"  {item['pack']}: vendor download, not fetched: {item['url']}" + (f" ({item['how']})" if item.get("how") else ""))
        else:
            print(f"  {item['pack']}: FAILED [{item['code']}] {item['message']}")
    if report["status"] != "ok":
        sys.stdout.flush()
        print(f"error: {report['message']}", file=sys.stderr)
        return 1
    return 0


def cmd_push(args) -> int:
    from . import pack_sources

    pack_id, version = _pack(args)
    report = pack_sources.push(_root(args), f"{pack_id}/{version}", repo=args.repo, create=args.create,
                               dry_run=args.dry_run, progress=not args.json)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0
    print(f"{report['pack']} -> {report['repo_type']} {report['repo']}:{report['path']}/ "
          f"({report['files']} file(s), {_size(report['bytes'])}, {report['not_redistributable']} not redistributable)")
    if report["outcome"] == "dry_run":
        print(f"dry run: write token {report['write_token']}. {report['message']}")
    else:
        print(f"{report['outcome']}: {report['message']}")
    return 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="twinsmith assets", description=__doc__.split("\n\n")[0])
    parser.add_argument("--root", help="asset root to use instead of TWINSMITH_ASSET_ROOT / <workspace>/assets")
    sub = parser.add_subparsers(dest="command", required=True)

    where = sub.add_parser("where", help="show the asset root, how it was found, and the project file")
    where.add_argument("--json", action="store_true")

    listing = sub.add_parser("list", help="every <id>/<version> with its size and whether it verifies; old-format packs")
    listing.add_argument("--json", action="store_true")

    doctor = sub.add_parser("doctor", help="verify a pack against pack.yaml and check its USD/URDF/MJCF dependency closure")
    doctor.add_argument("pack", nargs="?", help="<pack id>/<version> (default: the project file's pack)")
    doctor.add_argument("--usd-python", help="a Python with usd-core, for the dependency check")
    doctor.add_argument("--json", action="store_true")

    adopt = sub.add_parser("adopt", help="copy a folder into a new <id>/<version> and write its pack.yaml")
    adopt.add_argument("folder", nargs="?", help="the folder to copy (not needed with --from-old)")
    adopt.add_argument("--pack", help="<pack id>/<version> to create")
    adopt.add_argument("--from-old", metavar="OLD_PACK_FOLDER", help="convert an old-format pack <root>/<pack>/ into <pack>/v1")
    adopt.add_argument("--source", required=True, help="where the files came from (one line)")
    adopt.add_argument("--license", required=True, help="the licence or terms (one line)")
    adopt.add_argument("--redistributable", action="store_true", help="the files may be redistributed (default: no)")
    adopt.add_argument("--include", action="append", help="extra file extension to copy")
    adopt.add_argument("--file", action="append", help="copy only this file or folder, relative to the folder")
    adopt.add_argument("--dry-run", action="store_true")
    adopt.add_argument("--json", action="store_true")

    pin = sub.add_parser("pin", help="rewrite the sha256 values of an existing pack.yaml, after showing what changed")
    pin.add_argument("pack", nargs="?", help="<pack id>/<version> (default: the project file's pack)")
    pin.add_argument("--file", action="append", help="pin only this file")
    pin.add_argument("--dry-run", action="store_true")
    pin.add_argument("--json", action="store_true")

    build = sub.add_parser("build", help="copy named files and only what they depend on into a new pack version")
    build.add_argument("sources", nargs="+", help="files: absolute paths, or paths inside --from-pack")
    build.add_argument("--pack", required=True, help="<pack id>/<version> to create")
    build.add_argument("--from-pack", help="<pack id>/<version> the sources and their records come from")
    build.add_argument("--layout-root", help="the folder the new layout is relative to")
    build.add_argument("--source", help="source for files that do not come from a pack")
    build.add_argument("--license", help="licence for files that do not come from a pack")
    build.add_argument("--redistributable", action="store_true")
    build.add_argument("--usd-python", help="a Python with usd-core")
    build.add_argument("--allow-unresolved", action="store_true")
    build.add_argument("--dry-run", action="store_true")
    build.add_argument("--json", action="store_true")

    pull = sub.add_parser("pull", help="download pinned pack versions from the sources file and verify them")
    target = pull.add_mutually_exclusive_group(required=True)
    target.add_argument("pack", nargs="?", help="<pack id>/<version>")
    target.add_argument("--all", action="store_true", help="every entry of ~/.config/twinsmith/pack-sources.toml")
    pull.add_argument("--json", action="store_true")

    push = sub.add_parser("push", help="upload a pack version that verifies and pin its commit in the sources file")
    push.add_argument("pack", nargs="?", help="<pack id>/<version> (default: the project file's pack)")
    push.add_argument("--repo", help="<owner>/<name> (default: the pack's entry, else hub.repo)")
    push.add_argument("--create", action="store_true", help="create the repository, private, if it does not exist")
    push.add_argument("--dry-run", action="store_true", help="check the pack and the plan; upload and write nothing")
    push.add_argument("--json", action="store_true")
    return parser


COMMANDS = dict(where=cmd_where, list=cmd_list, doctor=cmd_doctor, adopt=cmd_adopt, pin=cmd_pin, build=cmd_build,
                pull=cmd_pull, push=cmd_push)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return COMMANDS[args.command](args)
    except CliError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except (ValueError, OSError) as exc:  # an invalid project file or name, or the disk
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
