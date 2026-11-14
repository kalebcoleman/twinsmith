"""Fake pinned checkouts for the wrapper tests: a git repository whose ``.venv/bin`` programs record their call."""

import json
from pathlib import Path
import subprocess
import sys
import time

FAKE = r'''#!{python}
import json, os, pathlib, sys
record = {{"argv": sys.argv, "cwd": os.getcwd(), "env": dict(os.environ)}}
with open(pathlib.Path(__file__).resolve().parents[2] / "calls.jsonl", "a") as stream:
    stream.write(json.dumps(record) + "\n")
if sys.argv[1:4] == ["-m", "vabar", "list"]:
    print("tool_hanging/wrenches_level1\nusbc_insert/insert_easy")
elif sys.argv[1:4] == ["-m", "vabar", "bench"] and "--print" in sys.argv:
    if "nope/L1" in sys.argv:
        print("[bench] no cell 'nope/L1'; the cells are ['tool_hanging/L1']", file=sys.stderr)
        sys.exit(2)
    print("# tool_hanging/L1 under config default: 1 jobs, 8 episodes", file=sys.stderr)
    print("python -m vabar run --task tool_hanging/wrenches_level1 --episodes 1,5 --bus-timeout 180")
elif "--validate-only" in sys.argv:
    if "broken" in sys.argv[-1]:
        print("[error] grasp_sg.pick: unknown skill 'pick_up'")
        print("FAIL: 1 error(s), 0 warning(s)")
        sys.exit(1)
    print("[warning] grasp_sg: output 'pose' is never read")
    print("OK: 0 errors, 1 warning(s)")
else:
    print(json.dumps(record))
'''


def fake_checkout(root: Path, programs=("python",)) -> str:
    """Create a one-commit git repository at ``root`` with fake ``.venv/bin`` programs; return its commit."""
    (root / ".venv/bin").mkdir(parents=True)
    for program in programs:
        path = root / ".venv/bin" / program
        path.write_text(FAKE.format(python=sys.executable))
        path.chmod(0o755)
    (root / "README").write_text("fake\n")
    git = ["git", "-C", str(root), "-c", "user.name=test", "-c", "user.email=test@example.invalid"]
    subprocess.run([*git, "init", "-q"], check=True)
    subprocess.run([*git, "add", "README"], check=True)
    subprocess.run([*git, "commit", "-q", "-m", "fake"], check=True)
    return subprocess.run([*git, "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()


def calls(root: Path) -> list:
    path = root / "calls.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def wait_for(job_id: str, timeout_s: float = 20) -> dict:
    from twinsmith.isaaclab import jobs

    deadline = time.monotonic() + timeout_s
    while True:
        status = jobs.runs_status(job_id)
        if status["state"] != "running" or time.monotonic() > deadline:
            return status
        time.sleep(0.05)
