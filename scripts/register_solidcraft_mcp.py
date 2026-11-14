#!/usr/bin/env python3
"""Add a headless ``solidcraft`` MCP server to every MCP client configuration on this computer.

    python3 scripts/register_solidcraft_mcp.py --cli ~/.local/bin/solidcraft-cli \
        --allow-path <workspace>/projects [--dry-run]

The entry runs ``solidcraft-cli mcp --headless --allow-path DIR``: an in-process design session with
no window, reading and writing files only in DIR (and the folders of designs it opens or saves).
Clients: Claude Code (user scope) and Codex through their own ``mcp add`` commands; OpenCode
(``opencode.json`` or ``opencode.jsonc``, edited as text so its comments stay); Claude Desktop,
Antigravity and T3 Code's Antigravity copies (``mcpServers`` JSON). A file that does not exist is
skipped, an existing ``solidcraft`` entry is left as it is, and nothing but file names and actions is
printed: these files can hold keys.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys


def strip_jsonc(text: str) -> dict:
    """Parse JSON with ``//`` and ``/* */`` comments and trailing commas."""
    out: list[str] = []
    i, in_string = 0, False
    while i < len(text):
        c = text[i]
        if in_string:
            out.append(c)
            if c == "\\":
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_string = False
        elif c == '"':
            in_string = True
            out.append(c)
        elif text.startswith("//", i):
            j = text.find("\n", i)
            i = len(text) if j < 0 else j
            continue
        elif text.startswith("/*", i):
            j = text.find("*/", i)
            i = len(text) if j < 0 else j + 2
            continue
        else:
            out.append(c)
        i += 1
    return json.loads(re.sub(r",(\s*[}\]])", r"\1", "".join(out)))


def atomic_write(path: str, text: str) -> None:
    """Replace *path* with *text*. The temporary file has the original's mode from the start: these files can hold keys."""
    mode = os.stat(path).st_mode & 0o777
    tmp = path + ".solidcraft-tmp"
    try:
        with os.fdopen(os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode), "w") as f:
            os.fchmod(f.fileno(), mode)                 # the umask may have cleared bits the original has
            f.write(text)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def add_opencode(path: str, command: list[str], dry_run: bool) -> str:
    text = open(path).read()
    if "solidcraft" in (strip_jsonc(text).get("mcp") or {}):
        return "already present"
    m = re.search(r'^(\s*)"mcp"\s*:\s*\{[ \t]*\n', text, re.M)
    if not m:
        return 'no "mcp" block, skipped'
    ind = m.group(1) + "  "
    block = (f'{ind}"solidcraft": {{\n{ind}  "type": "local",\n{ind}  "command": {json.dumps(command)},\n'
             f'{ind}  "enabled": true\n{ind}}},\n')
    new = text[: m.end()] + block + text[m.end():]
    if strip_jsonc(new)["mcp"]["solidcraft"]["command"] != command:
        return "edit did not parse back, skipped"
    if not dry_run:
        atomic_write(path, new)
    return "added"


def add_mcp_servers_json(path: str, command: list[str], dry_run: bool) -> str:
    data = json.load(open(path))
    servers = data.setdefault("mcpServers", {})
    if "solidcraft" in servers:
        return "already present"
    servers["solidcraft"] = {"command": command[0], "args": command[1:]}
    if not dry_run:
        atomic_write(path, json.dumps(data, indent=2) + "\n")
    return "added"


def run_cli(argv: list[str], dry_run: bool) -> str:
    if dry_run:
        return "would add"
    r = subprocess.run(argv, capture_output=True, text=True)
    return "added" if r.returncode == 0 else f"failed (exit {r.returncode})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cli", default=os.path.expanduser("~/.local/bin/solidcraft-cli"))
    ap.add_argument("--allow-path", required=True, help="the one folder the MCP server may use (must exist)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    home = os.path.expanduser("~")
    allow = os.path.abspath(os.path.expanduser(a.allow_path))
    if not os.path.isdir(allow):
        print(f"--allow-path {allow} does not exist; create it first", file=sys.stderr)
        return 2
    command = [os.path.abspath(os.path.expanduser(a.cli)), "mcp", "--headless", "--allow-path", allow]

    def show(where: str, result: str) -> None:
        print(f"{where.replace(home, '~')}: {result}")

    claude_json = os.path.join(home, ".claude.json")
    if shutil.which("claude") and os.path.exists(claude_json):
        if "solidcraft" in (json.load(open(claude_json)).get("mcpServers") or {}):
            show("claude-code", "already present")
        else:
            show("claude-code", run_cli(["claude", "mcp", "add", "--scope", "user", "solidcraft", "--", *command], a.dry_run))

    codex_cfg = os.path.join(os.environ.get("CODEX_HOME", os.path.join(home, ".codex")), "config.toml")
    if shutil.which("codex") and os.path.exists(codex_cfg):
        if re.search(r"^\[mcp_servers\.solidcraft\]", open(codex_cfg).read(), re.M):
            show("codex", "already present")
        else:
            show("codex", run_cli(["codex", "mcp", "add", "solidcraft", "--", *command], a.dry_run))

    for name in ("opencode.jsonc", "opencode.json"):
        path = os.path.join(home, ".config", "opencode", name)
        if os.path.exists(path):
            show(path, add_opencode(path, command, a.dry_run))

    plain = [
        os.path.join(home, ".config", "Claude", "claude_desktop_config.json"),
        os.path.join(home, ".gemini", "antigravity", "mcp_config.json"),
        os.path.join(home, ".gemini", "config", "mcp_config.json"),
        *sorted(glob.glob(os.path.join(home, ".t3", "userdata", "providers", "antigravity", "*", "config", "mcp_config.json"))),
    ]
    for path in plain:
        if os.path.exists(path):
            show(path, add_mcp_servers_json(path, command, a.dry_run))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
