#!/usr/bin/env python3
"""Check a twinsmith machine against docs/REBUILD.md and print one status table. Reads nothing secret.

    python3 scripts/verify_setup.py

Standard library only, so it runs before anything is installed. PASS: as the guide leaves it. WARN: optional or
only needed to rebuild something. FAIL: the work cell will not build here. Exit status 1 if anything fails.
Secrets are never opened: only whether the folder and files exist and who can read them.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FULL_TIER_MIB = 48_000          # local VLM, SAM 3, RoboSimStudio, Graph-as-Policy and VABAR beside the simulator
SOURCE_PACK = "ur5e-jenga-workcell/v2"
TOOLS = {"robosimstudio": ("mcp/twinsmith/robosim/bridge.py", "PIN"), "graph-as-policy": ("mcp/twinsmith/gap.py", "PIN"),
         "open-robot-skills": ("mcp/twinsmith/gap.py", "SKILLS_PIN"), "vabar": ("mcp/twinsmith/vabar.py", "PIN")}


def tier(vram_mib):
    """Which optional parts fit: 'full' from about 48 GB of GPU memory, else 'sim' (simulator and work cell only)."""
    return "full" if vram_mib >= FULL_TIER_MIB else "sim"


def workspace(environ=None, home=None):
    """The workspace folder, in the order scripts/env.py uses."""
    environ = os.environ if environ is None else environ
    if environ.get("TWINSMITH_WORKSPACE", "").strip():
        return Path(environ["TWINSMITH_WORKSPACE"]).expanduser()
    if any((REPO.parent / name).is_dir() for name in ("envs", "assets")):
        return REPO.parent
    return Path(home or Path.home()) / "robotics-workspace"


def pin(relative, name):
    match = re.search(rf'^{name} *= *"([0-9a-f.]+)"', (REPO / relative).read_text(), re.MULTILINE)
    return match.group(1) if match else None


def wanted_packs():
    """Pack id/version of the cell and of every task file, read from the repository's own definitions."""
    files = [REPO / "exts/workcell/config/lab_hardware.json", *sorted((REPO / "exts/workcell/tasks").glob("*.json"))]
    packs = []
    for path in files:
        section = json.loads(path.read_text())["pack"]
        name = f"{section['id']['value']}/{section['version']['value']}"
        if name not in packs:
            packs.append(name)
    return packs


def pack_state(root, name):
    """(ok, detail) for one pack: its manifest exists and every listed file has the recorded SHA-256."""
    folder = root / name
    manifest = folder / "pack.yaml"
    if not manifest.is_file():
        return False, "missing"
    try:
        files = json.loads(manifest.read_text())["files"]
        files = [(record["path"], record["sha256"]) for record in files]
    except (ValueError, KeyError, TypeError):
        return False, "pack.yaml is not the JSON manifest this check reads; use scripts/assets.py doctor"
    for relative, sha256 in files:
        path = folder / relative
        if not path.is_file():
            return False, f"file missing: {relative}"
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1 << 20), b""):
                digest.update(chunk)
        if digest.hexdigest() != sha256:
            return False, f"file changed: {relative}"
    return True, f"{len(files)} files match pack.yaml"


def private(path):
    """Only the owner can read it."""
    return path.exists() and stat.S_IMODE(path.stat().st_mode) & 0o077 == 0


def run(command, timeout=30, env=None):
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout, env=env).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def collect():
    rows = []

    def add(area, item, status, detail=""):
        rows.append((area, item, status, str(detail).replace(str(Path.home()), "~")))

    codename = ""
    if Path("/etc/os-release").is_file():
        codename = dict(line.split("=", 1) for line in Path("/etc/os-release").read_text().splitlines() if "=" in line).get(
            "VERSION_CODENAME", "")
    add("machine", "Ubuntu 22.04 or 24.04", "PASS" if codename in ("jammy", "noble") else "FAIL", codename or "unknown")
    gpu = run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader,nounits"]).strip().splitlines()
    try:
        name, memory, driver = (part.strip() for part in gpu[0].split(","))
        vram = int(memory)
        add("machine", "NVIDIA GPU", "PASS", f"{name}, {vram} MiB, driver {driver}")
    except (IndexError, ValueError):
        vram = 0
        add("machine", "NVIDIA GPU", "FAIL", "nvidia-smi reports no GPU")
    level = tier(vram)
    add("machine", "tier", "PASS", f"{level}: " + ("simulator, work cell and the optional tools" if level == "full"
                                                   else "simulator and work cell; the local VLM tier needs about 48 GB"))
    for tool, need in (("git", "FAIL"), ("uv", "FAIL"), ("curl", "FAIL"), ("freecadcmd", "WARN"), ("ffmpeg", "WARN")):
        add("machine", tool, "PASS" if shutil.which(tool) else need, shutil.which(tool) or "not on PATH"
            + (" (only to rebuild the wrist bracket)" if tool == "freecadcmd" else " (only for videos)" if tool == "ffmpeg" else ""))

    space = workspace()
    for name in ("assets", "envs", "projects", "runs", "tools"):
        add("workspace", name, "PASS" if (space / name).is_dir() else "FAIL" if name == "assets" else "WARN", space / name)
    listing = run([sys.executable, str(REPO / "scripts/env.py"), "find"], env=dict(os.environ, TWINSMITH_WORKSPACE=str(space)))
    found = [line for line in listing.splitlines() if re.search(r"\b6\.\d+\.\d+", line)]
    add("simulator", "Isaac Sim 6.x interpreter", "PASS" if found else "FAIL",
        found[0].split()[0] if found else "none found: python3 scripts/env.py find")

    root = Path(os.environ.get("TWINSMITH_ASSET_ROOT") or space / "assets").expanduser()
    for name in wanted_packs():
        ok, detail = pack_state(root, name)
        add("assets", name, "PASS" if ok else "FAIL", detail)
    ok, detail = pack_state(root, SOURCE_PACK)
    add("assets", SOURCE_PACK, "PASS" if ok else "WARN", detail + ("" if ok else " (private source of the cell pack; keep a backup)"))

    for name, (relative, constant) in TOOLS.items():
        wanted, checkout = pin(relative, constant), space / "tools" / name
        head = run(["git", "-C", str(checkout), "rev-parse", "HEAD"]).strip() if checkout.is_dir() else ""
        status = "PASS" if head and wanted and head.startswith(wanted) else "WARN"
        add("optional tools", name, status if level == "full" or head else "SKIP",
            f"at pin {wanted}" if status == "PASS" else f"{'at ' + head[:9] if head else 'not installed'}; pin {wanted}")
    endpoint = "http://127.0.0.1:8000/v1/models"
    try:
        with urllib.request.urlopen(endpoint, timeout=2) as reply:
            add("optional tools", "local VLM endpoint", "PASS", json.load(reply)["data"][0]["id"])
    except (OSError, ValueError, KeyError, IndexError):
        add("optional tools", "local VLM endpoint", "WARN" if level == "full" else "SKIP", f"no answer at {endpoint}")

    secrets, token = Path.home() / ".config/agent-secrets", Path.home() / ".cache/huggingface/token"
    if secrets.is_dir():
        files = [path for path in secrets.iterdir() if path.is_file()]
        closed = private(secrets) and all(private(path) for path in files)
        add("secrets", "secrets folder", "PASS" if closed else "FAIL",
            f"{len(files)} files, " + ("owner-only" if closed else "readable by others: chmod 700 the folder, 600 the files"))
    else:
        add("secrets", "secrets folder", "WARN", "~/.config/agent-secrets does not exist")
    add("secrets", "Hugging Face token file", "PASS" if token.is_file() and private(token) else "WARN",
        "present, owner-only" if token.is_file() and private(token) else "missing or readable by others (needed for SAM 3)")
    return rows


def main() -> int:
    rows = collect()
    widths = [max(len(row[i]) for row in rows + [("AREA", "ITEM", "STATUS", "")]) for i in range(3)]
    print(f"{'AREA':<{widths[0]}}  {'ITEM':<{widths[1]}}  {'STATUS':<{widths[2]}}  DETAIL")
    for area, item, status, detail in rows:
        print(f"{area:<{widths[0]}}  {item:<{widths[1]}}  {status:<{widths[2]}}  {detail}")
    failed = [row for row in rows if row[2] == "FAIL"]
    print(f"\n{len(rows)} checks: {sum(row[2] == 'PASS' for row in rows)} pass, {sum(row[2] == 'WARN' for row in rows)} warn, "
          f"{sum(row[2] == 'SKIP' for row in rows)} skipped, {len(failed)} fail")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
