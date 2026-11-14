import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import registry, tools


def ckpt(tmp_path, name="recC.pt", data=b"weights"):
    d = tmp_path / "checkpoints"
    d.mkdir(exist_ok=True)
    p = d / name
    p.write_bytes(data)
    return p


def test_register_writes_table_and_hash(tmp_path):
    p = ckpt(tmp_path)
    out = registry.register(str(p), "baseline", "g4j1 it1425 | 174.7/256")
    import hashlib
    assert out["sha256"] == hashlib.sha256(b"weights").hexdigest() and out["size_bytes"] == 7
    text = (tmp_path / "checkpoints" / "README.md").read_text()
    assert text.startswith("# Checkpoints\n\n| Checkpoint | Role |") and "`recC.pt`" in text and out["sha256"] in text
    assert "174.7/256" in text and "g4j1 it1425 \\| 174.7" in text  # the pipe is escaped, the table stays intact
    assert registry.existing_rows(text)[0]["provenance"] == "g4j1 it1425 | 174.7/256"


def test_second_checkpoint_appends_and_keeps_old_text(tmp_path):
    a, b = ckpt(tmp_path, "a.pt", b"a"), ckpt(tmp_path, "b.pt", b"b")
    registry.register(str(a), "baseline", "first")
    before = (tmp_path / "checkpoints" / "README.md").read_text()
    registry.register(str(b), "candidate", "second")
    after = (tmp_path / "checkpoints" / "README.md").read_text()
    assert after.startswith(before) and after.count("\n| `") == 2 and after.count("| Checkpoint |") == 1


def test_existing_prose_is_kept_and_table_added(tmp_path):
    p = ckpt(tmp_path)
    readme = tmp_path / "checkpoints" / "README.md"
    readme.write_text("# Notes\n\nKeep me.\n")
    registry.register(str(p), "init", "x")
    text = readme.read_text()
    assert text.startswith("# Notes\n\nKeep me.\n\n| Checkpoint |")


def test_registered_path_is_never_overwritten(tmp_path):
    p = ckpt(tmp_path)
    registry.register(str(p), "baseline", "orig")
    readme = tmp_path / "checkpoints" / "README.md"
    snapshot = readme.read_text()
    with pytest.raises(ValueError, match="already registered.*still matches"):
        registry.register(str(p), "baseline", "again")
    p.write_bytes(b"changed")
    with pytest.raises(ValueError, match="now DIFFERS"):
        registry.register(str(p), "candidate", "retrain")
    assert readme.read_text() == snapshot


def test_same_content_other_path_is_reported(tmp_path):
    registry.register(str(ckpt(tmp_path, "a.pt", b"same")), "baseline", "p")
    out = registry.register(str(ckpt(tmp_path, "b.pt", b"same")), "candidate", "p")
    assert out["same_content_as"] == ["a.pt"]


@pytest.mark.parametrize("kwargs, text", [
    ({"role": "", "provenance": "x"}, "role is required"),
    ({"role": "baseline", "provenance": " "}, "provenance is required"),
    ({"role": "a\nb", "provenance": "x"}, "one line"),
    ({"role": "baseline", "provenance": "x" * 501}, "one line"),
])
def test_fields_required(tmp_path, kwargs, text):
    with pytest.raises(ValueError, match=text):
        registry.register(str(ckpt(tmp_path)), **kwargs)


def test_bad_paths_and_registry(tmp_path):
    (tmp_path / "x.txt").write_text("x")
    for bad, text in [("", "path is required"), (str(tmp_path / "none.pt"), "not an existing file.*absolute path"), (str(tmp_path / "x.txt"), "not a checkpoint")]:
        with pytest.raises(ValueError, match=text):
            registry.register(bad, "r", "p")
    p = ckpt(tmp_path)
    with pytest.raises(ValueError, match="must be a .md"):
        registry.register(str(p), "r", "p", registry=str(tmp_path / "checkpoints" / "notes.txt"))
    with pytest.raises(ValueError, match="folder or one above"):
        registry.register(str(p), "r", "p", registry=str(tmp_path / "other" / "x.md"))
    assert not (tmp_path / "checkpoints" / "README.md").exists()


def test_registry_in_parent_uses_relative_path(tmp_path):
    p = ckpt(tmp_path)
    out = registry.register(str(p), "r", "p", registry=str(tmp_path / "CHECKPOINTS.md"))
    assert "`checkpoints/recC.pt`" in out["row"]


def test_read_only_option(tmp_path):
    p = ckpt(tmp_path)
    out = registry.register(str(p), "baseline", "p", read_only=True)
    assert out["read_only"] is True and not os.access(p, os.W_OK) or os.geteuid() == 0
    assert p.stat().st_mode & 0o222 == 0


def test_concurrent_registrations_keep_every_row(tmp_path):
    files = [ckpt(tmp_path, f"m{i}.pt", bytes([i])) for i in range(8)]
    errs = []

    def go(f):
        try:
            registry.register(str(f), "candidate", "p")
        except Exception as exc:  # noqa: BLE001
            errs.append(exc)

    threads = [threading.Thread(target=go, args=(f,)) for f in files]
    [t.start() for t in threads]
    [t.join() for t in threads]
    rows = registry.existing_rows((tmp_path / "checkpoints" / "README.md").read_text())
    assert not errs and sorted(r["path"] for r in rows) == sorted(f.name for f in files)


def test_git_hint_for_ignored_pt(tmp_path):
    if subprocess.run(["git", "--version"], capture_output=True).returncode != 0:
        pytest.skip("git missing")
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / ".gitignore").write_text("*.pt\n")
    out = registry.register(str(ckpt(tmp_path)), "baseline", "p")
    assert out["git"]["ignored_by_git"] is True and "git add -f" in out["git"]["hint"]


def test_tool_layer(tmp_path):
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    t = server._tool_manager._tools["isaaclab__checkpoint_register"]
    assert t.annotations.readOnlyHint is False and t.annotations.destructiveHint is False
    p = ckpt(tmp_path)
    assert json.loads(t.fn(path=str(p), role="baseline", provenance="x"))["registered"] is True
    assert "already registered" in json.loads(t.fn(path=str(p), role="baseline", provenance="x"))["error"]


@pytest.mark.parametrize("role", ["baseline", "candidate", "init", "final"])
def test_read_only_is_opt_in_and_off_for_every_role(tmp_path, role):
    p = ckpt(tmp_path, f"{role}.pt")
    out = registry.register(str(p), role, "p")
    assert "read_only" not in out
    assert p.stat().st_mode & 0o200  # still writable by the owner
    p.write_bytes(b"changed later")  # a registered checkpoint can still be changed on disk; the README row then shows the mismatch
