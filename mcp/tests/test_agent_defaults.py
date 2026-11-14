import subprocess
import sys
from pathlib import Path

from twinsmith import screen
import twinsmith.server as gw

ROOT = Path(__file__).resolve().parents[2]


def test_label_tag_is_filename_safe():
    assert screen._label_tag(None) == "" and screen._label_tag("") == ""
    assert screen._label_tag("push done/../x") == "-push_done____x"
    assert len(screen._label_tag("a" * 100)) == 41


def test_instructions_tell_agents_to_prefer_the_tools():
    assert "Prefer listed tools" in gw.SERVER_INSTRUCTIONS and "status" in gw.SERVER_INSTRUCTIONS


# A usual selection for scene work: what these help answers assume.
EVERYDAY = frozenset({"core", "sim", "assets", "computer"})


def test_session_check_never_fails_and_nudges():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "session_check.py"), "--port", "1"],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0 and r.stdout.startswith("twinsmith:") and "mcp__twinsmith" in r.stdout


def test_help_routes_by_task_and_falls_back():
    one = gw._help("why did the grasp fail", enabled=EVERYDAY)
    assert "sim__physics__contacts" in one and "isaaclab__train" not in one
    assert "Toolsets off: sim" in gw._help("train a policy", enabled={"core", "assets"})
    both = gw._help("something odd", enabled=EVERYDAY)
    assert "No twinsmith tool is known to fit" in both and "screenshot" in both and "Toolsets off: sim" not in both
    assert "launch_simulator" in gw._help(None, enabled=EVERYDAY)
    assert "launch_simulator" not in gw._help(None, enabled={"core", "sim"}) and "computer" in gw._help(None, enabled={"core", "sim"})


def test_session_check_survives_bad_port_env_and_args():
    import os
    env = dict(os.environ, TWINSMITH_PORT="not-a-number")
    for args in ([], ["--bogus"]):
        r = subprocess.run([sys.executable, str(ROOT / "scripts" / "session_check.py"), *args],
                           capture_output=True, text=True, timeout=30, env=env)
        assert r.returncode == 0 and r.stdout.startswith("twinsmith:")


def test_help_matches_word_starts_not_substrings():
    assert "isaaclab__train" not in gw._help("build a world", enabled=EVERYDAY)
    assert "sim__quick_start" in gw._help("build a world", enabled=EVERYDAY)


def test_screenshot_label_is_not_shadowed_by_the_window_caption():
    import inspect
    body = inspect.getsource(screen.capture)  # what the screenshot tool returns
    assert "_label_tag(label)" in body and "\n    label = " not in body and "target_label = f" in body


def test_help_finds_contact_sheets_and_job_workflow_tools():
    sheet = gw._help("make a contact sheet from two videos to attach to a message", enabled=EVERYDAY)
    assert 'join_videos(mode="sheet"' in sheet and "home folder" in sheet
    work = gw._help("wait for a job, register a baseline and summarise pass counts", enabled=gw.toolsets.PROFILES["all"])
    for name in ("isaaclab__preflight", "isaaclab__runs_status(job_id, wait_s=45)", "isaaclab__checkpoint_register", "eval_json"):
        assert name in work
