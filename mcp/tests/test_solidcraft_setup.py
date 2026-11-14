"""scripts/install_solidcraft.sh and scripts/register_solidcraft_mcp.py: pins and client registration."""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
REGISTER = REPO / "scripts" / "register_solidcraft_mcp.py"
INSTALL = REPO / "scripts" / "install_solidcraft.sh"


def _register(home: Path, *args: str) -> subprocess.CompletedProcess:
    # PATH without claude or codex: only the files this script edits itself are touched.
    env = {"HOME": str(home), "PATH": "/usr/bin:/bin"}
    return subprocess.run([sys.executable, "-I", str(REGISTER), *args], env=env, capture_output=True, text=True)


def _seed(home: Path) -> None:
    (home / ".config" / "opencode").mkdir(parents=True)
    (home / ".config" / "opencode" / "opencode.jsonc").write_text(
        '{\n  // keep me\n  "$schema": "https://opencode.ai/config.json",\n  "mcp": {\n'
        '    "freecad": { "type": "local", "command": ["uvx", "freecad-mcp"] }, /* and me */\n  },\n}\n')
    (home / ".config" / "Claude").mkdir(parents=True)
    desktop = home / ".config" / "Claude" / "claude_desktop_config.json"
    desktop.write_text(json.dumps({"mcpServers": {"freecad": {"command": "uvx"}}}, indent=2) + "\n")
    desktop.chmod(0o600)
    t3 = home / ".t3" / "userdata" / "providers" / "antigravity" / "abc" / "config"
    t3.mkdir(parents=True)
    (t3 / "mcp_config.json").write_text(json.dumps({"mcpServers": {}}))


def test_register_adds_one_headless_entry_per_client_and_is_idempotent(tmp_path):
    home = tmp_path / "home"
    home.mkdir()
    _seed(home)
    projects = home / "projects"
    projects.mkdir()
    first = _register(home, "--cli", str(home / "solidcraft-cli"), "--allow-path", str(projects))
    assert first.returncode == 0, first.stderr
    assert first.stdout.count(": added") == 3
    expected = [str(home / "solidcraft-cli"), "mcp", "--headless", "--allow-path", str(projects)]

    text = (home / ".config" / "opencode" / "opencode.jsonc").read_text()
    assert "// keep me" in text and "/* and me */" in text
    sys.path.insert(0, str(REGISTER.parent))
    try:
        from register_solidcraft_mcp import strip_jsonc
    finally:
        sys.path.pop(0)
    oc = strip_jsonc(text)["mcp"]
    assert oc["solidcraft"]["command"] == expected and "freecad" in oc

    desktop = home / ".config" / "Claude" / "claude_desktop_config.json"
    entry = json.loads(desktop.read_text())["mcpServers"]["solidcraft"]
    assert [entry["command"], *entry["args"]] == expected
    assert desktop.stat().st_mode & 0o777 == 0o600

    before = {p: p.read_bytes() for p in home.rglob("*.json*")}
    second = _register(home, "--cli", str(home / "solidcraft-cli"), "--allow-path", str(projects))
    assert second.returncode == 0 and "added" not in second.stdout
    assert {p: p.read_bytes() for p in home.rglob("*.json*")} == before


def test_register_dry_run_writes_nothing_and_needs_an_existing_folder(tmp_path):
    home = tmp_path / "home"
    home.mkdir()
    _seed(home)
    before = {p: p.read_bytes() for p in home.rglob("*.json*")}
    assert _register(home, "--allow-path", str(home / "missing")).returncode == 2
    (home / "projects").mkdir()
    dry = _register(home, "--allow-path", str(home / "projects"), "--dry-run")
    assert dry.returncode == 0 and dry.stdout.count(": added") == 3
    assert {p: p.read_bytes() for p in home.rglob("*.json*")} == before


def test_installer_pins_a_version_and_a_sha256():
    text = INSTALL.read_text()
    version = re.search(r'^SOLIDCRAFT_VERSION="(\d+\.\d+\.\d+)"', text, re.M)
    digest = re.search(r'^SOLIDCRAFT_SHA256="([0-9a-f]{64})"', text, re.M)
    assert version and digest
    assert "sha256sum -c" in text and not re.search(r"^[^#]*\bsudo\b", text, re.M)
    assert os.access(INSTALL, os.X_OK)


def test_jsonc_reader_ends_on_an_unterminated_comment():
    sys.path.insert(0, str(REGISTER.parent))
    try:
        from register_solidcraft_mcp import strip_jsonc
    finally:
        sys.path.pop(0)
    assert strip_jsonc('{"mcp": {}} /* open') == {"mcp": {}}
