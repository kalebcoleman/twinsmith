#!/usr/bin/env python3
"""Show, check or set the twinsmith version.

The only place to edit the version is ``[project] version`` in ``mcp/pyproject.toml``, the package
published to PyPI. The running server reads it back from the installed metadata. A few files carry
a copy; ``set`` writes every copy and ``check`` fails when one differs, so a copy cannot drift:

* ``pyproject.toml``: the simulator environment (``twinsmith-sim``) shares the version;
* ``mcp/uv.lock`` and ``uv.lock``: the entry of each lock's own project;
* ``README.md`` and ``docs/media/badges/version.svg``: the source-version badge.

    python3 scripts/version.py                     # print the version
    python3 scripts/version.py set 0.4.0           # write it to the source and every copy
    python3 scripts/version.py check               # every copy equals the source
    python3 scripts/version.py check --tag v0.4.0  # ... and the tag is v<version>
    python3 scripts/version.py check --dist mcp/dist  # ... and the built files carry it

The release workflow (``.github/workflows/release.yml``) runs ``check --tag`` and ``check --dist``.
It needs only the standard library, so it runs with any ``python3``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = "mcp/pyproject.toml"
PACKAGE = "twinsmith"

# Canonical PEP 440 public versions only (1.2.3, 1.2.3rc1, 1.2.3.post1, 1.2.3.dev2): the tag, the
# wheel name and PyPI then all spell the version the same way.
_N = r"(?:0|[1-9]\d*)"   # no leading zeros: 01 is not canonical
VERSION = re.compile(rf"{_N}(?:\.{_N})*(?:(?:a|b|rc){_N})?(?:\.post{_N})?(?:\.dev{_N})?")

# The version line of a pyproject's [project] table: after "[project]" and before the next table.
PROJECT_VERSION = re.compile(r'(^\[project\]\n(?:(?!\[).*\n)*?version = ")([^"]*)(")', re.M)


def lock_entry(name: str, source: str) -> re.Pattern:
    """The version of a lock file's own project, which uv records with its source."""
    return re.compile(rf'(^name = "{re.escape(name)}"\nversion = ")([^"]*)("\nsource = \{{ {source} \}}$)', re.M)


BADGE_LINK = re.compile(r"(\[!\[Source version )([^\]]*)(\]\(docs/media/badges/version\.svg\)\])")


def _one(pattern: re.Pattern, text: str, where: str) -> re.Match:
    found = list(pattern.finditer(text))
    if len(found) != 1:
        raise SystemExit(f"{where}: expected one version field, found {len(found)}; update scripts/version.py")
    return found[0]


class Copy:
    """A file that holds the version once, found by a pattern whose second group is the version."""

    def __init__(self, path: str, pattern: re.Pattern):
        self.path, self.pattern = path, pattern

    def read(self) -> str:
        return _one(self.pattern, (ROOT / self.path).read_text(encoding="utf-8"), self.path).group(2)

    def write(self, version: str) -> None:
        file = ROOT / self.path
        text = file.read_text(encoding="utf-8")
        match = _one(self.pattern, text, self.path)
        file.write_text(text[:match.start(2)] + version + text[match.end(2):], encoding="utf-8")


class Badge:
    """docs/media/badges/version.svg: a "source" label and a value box sized to the version."""

    path = "docs/media/badges/version.svg"
    LABEL_WIDTH = 48

    def read(self) -> str:
        text = (ROOT / self.path).read_text(encoding="utf-8")
        match = re.search(r"<title>source: ([^<]*)</title>", text)
        if not match:
            raise SystemExit(f"{self.path}: no 'source: <version>' title; update scripts/version.py")
        return match.group(1)

    def render(self, version: str) -> str:
        value = 7 * len(version) + 8          # DejaVu Sans 11 px: about 7 px a character, 4 px each side
        width = self.LABEL_WIDTH + value
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="22" role="img" aria-label="source: {version}">\n'
            f"  <title>source: {version}</title>\n"
            f'  <rect width="{width}" height="22" rx="4" fill="#2563eb"/>\n'
            f'  <path d="M4 0H{self.LABEL_WIDTH}V22H4Q0 22 0 18V4Q0 0 4 0Z" fill="#343a40"/>\n'
            '  <g fill="white" text-anchor="middle" font-family="DejaVu Sans,Verdana,sans-serif" font-size="11">\n'
            f'    <text x="{self.LABEL_WIDTH / 2}" y="15">source</text>\n'
            f'    <text x="{self.LABEL_WIDTH + value / 2}" y="15">{version}</text>\n'
            "  </g>\n"
            "</svg>\n"
        )

    def write(self, version: str) -> None:
        (ROOT / self.path).write_text(self.render(version), encoding="utf-8")


SOURCE_FIELD = Copy(SOURCE, PROJECT_VERSION)
COPIES = (
    Copy("pyproject.toml", PROJECT_VERSION),
    Copy("mcp/uv.lock", lock_entry(PACKAGE, 'editable = "."')),
    Copy("uv.lock", lock_entry("twinsmith-sim", 'virtual = "."')),
    Copy("README.md", BADGE_LINK),
    Badge(),
)


def current() -> str:
    return SOURCE_FIELD.read()


def drift(version: str) -> list[str]:
    """Every copy that does not hold ``version``."""
    errors = []
    for copy in COPIES:
        found = copy.read()
        if found != version:
            errors.append(f"{copy.path} has {found}, {SOURCE} has {version}")
    return errors


def tag_errors(tag: str, version: str) -> list[str]:
    tag = tag.removeprefix("refs/tags/")
    return [] if tag == f"v{version}" else [f"tag {tag} does not match the package version: expected v{version}"]


def dist_errors(folder: Path, version: str) -> list[str]:
    """The folder holds exactly this version's sdist and pure-Python wheel."""
    if not folder.is_dir():
        return [f"{folder}: no such folder"]
    found = sorted(path.name for path in folder.iterdir() if not path.name.startswith("."))
    expected = [f"{PACKAGE}-{version}-py3-none-any.whl", f"{PACKAGE}-{version}.tar.gz"]
    return [] if found == expected else [f"{folder} holds {found}, expected {expected}"]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command")
    check = commands.add_parser("check", help="fail when a copy, the tag or the built files differ from the source")
    check.add_argument("--tag", help="a release tag (v<version>, or refs/tags/v<version>) that must match")
    check.add_argument("--dist", type=Path, help="a folder that must hold exactly this version's sdist and wheel")
    write = commands.add_parser("set", help=f"write a new version to {SOURCE} and every copy")
    write.add_argument("version")
    args = parser.parse_args(argv)

    if args.command == "set":
        if not VERSION.fullmatch(args.version):
            parser.error(f"{args.version!r} is not a canonical version such as 1.2.3 or 1.2.3rc1")
        for field in (SOURCE_FIELD, *COPIES):
            field.write(args.version)
        print(f"{PACKAGE} {args.version}: wrote {SOURCE} and {', '.join(copy.path for copy in COPIES)}")
        return 0

    version = current()
    if args.command is None:
        print(version)
        return 0

    errors = [] if VERSION.fullmatch(version) else [f"{SOURCE}: {version!r} is not a canonical version"]
    errors += drift(version)
    if args.tag is not None:
        errors += tag_errors(args.tag, version)
    if args.dist is not None:
        errors += dist_errors(args.dist, version)
    for error in errors:
        print(f"error: {error}", file=sys.stderr)
    if errors:
        print("Run `python3 scripts/version.py set <version>` to write every copy (docs/RELEASING.md).", file=sys.stderr)
        return 1
    print(f"{PACKAGE} {version}: every copy matches" + (f", and so does {args.tag}" if args.tag else "")
          + (f" and {args.dist}" if args.dist else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
