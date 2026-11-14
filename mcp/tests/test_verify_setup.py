"""CPU-only checks for scripts/verify_setup.py: its pure helpers, and that it opens no secret."""

import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("verify_setup", ROOT / "scripts/verify_setup.py")
verify = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify)


def test_tier_follows_gpu_memory():
    assert verify.tier(16_303) == "sim" and verify.tier(47_999) == "sim"
    assert verify.tier(48_000) == "full" and verify.tier(97_887) == "full"


def test_workspace_order(tmp_path):
    assert verify.workspace({"TWINSMITH_WORKSPACE": str(tmp_path)}) == tmp_path
    if not any((ROOT.parent / name).is_dir() for name in ("envs", "assets")):
        assert verify.workspace({}, home=tmp_path) == tmp_path / "robotics-workspace"


def test_wanted_packs_come_from_the_cell_and_task_files():
    assert verify.wanted_packs() == ["ur5e-workcell/v3", "jenga/v1"]


def test_pins_are_read_from_the_code():
    for relative, constant in verify.TOOLS.values():
        pin = verify.pin(relative, constant)
        assert pin and len(pin) >= 7 and int(pin, 16) >= 0


def test_pack_state_checks_every_hash(tmp_path):
    folder = tmp_path / "demo" / "v1"
    (folder / "a").mkdir(parents=True)
    (folder / "a" / "a.usda").write_text("one")
    record = {"path": "a/a.usda", "sha256": hashlib.sha256(b"one").hexdigest()}
    assert verify.pack_state(tmp_path, "demo/v1") == (False, "missing")
    (folder / "pack.yaml").write_text(json.dumps({"id": "demo", "version": "v1", "files": [record]}))
    assert verify.pack_state(tmp_path, "demo/v1") == (True, "1 files match pack.yaml")
    (folder / "a" / "a.usda").write_text("two")
    assert verify.pack_state(tmp_path, "demo/v1") == (False, "file changed: a/a.usda")
    (folder / "a" / "a.usda").unlink()
    assert verify.pack_state(tmp_path, "demo/v1") == (False, "file missing: a/a.usda")


def test_secrets_are_only_looked_at_from_outside(tmp_path):
    secret = tmp_path / "key"
    secret.write_text("value")
    secret.chmod(0o600)
    assert verify.private(secret)
    secret.chmod(0o644)
    assert not verify.private(secret)
    source = (ROOT / "scripts/verify_setup.py").read_text()
    body = source.split("secrets, token =", 1)[1]
    assert "read_text" not in body and "open(" not in body and ".name" not in body     # counts and modes only
