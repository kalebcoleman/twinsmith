"""twinsmith assets (scripts/assets.py from a checkout): where, list, doctor, adopt (and --from-old), pin, build, on the pack.yaml format."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path

import pytest

from twinsmith import pack_manifest as pm

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "scripts" / "assets.py"
SETTINGS = ("TWINSMITH_ASSET_ROOT", "TWINSMITH_WORKSPACE", "TWINSMITH_PROJECT", "TWINSMITH_USD_PYTHON")


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def cli(tmp_path, monkeypatch):
    for name in SETTINGS:
        monkeypatch.delenv(name, raising=False)
    home = tmp_path / "home@host"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(tmp_path / "no-project"))  # keeps cwd discovery out of the tests
    monkeypatch.chdir(tmp_path)
    spec = importlib.util.spec_from_file_location("isaac_assets_cli_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def root(tmp_path, monkeypatch):
    path = tmp_path / "assets"
    path.mkdir()
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(path))
    return path


def run(cli, capsys, *argv):
    code = cli.main(list(argv))
    out = capsys.readouterr()
    return code, out.out, out.err


def source_folder(tmp_path, files=None):
    folder = tmp_path / "src"
    for rel, data in (files or {"robot/a.urdf": "<robot name='a'/>", "robot/mesh.stl": "mesh"}).items():
        path = folder / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data.encode() if isinstance(data, str) else data)
    return folder


ADOPT = ("--source", "vendor CAD, 2026", "--license", "private lab use")


# ---------------------------------------------------------------- where


def test_where_reports_the_root_and_how_it_was_found(cli, capsys, root):
    code, out, _ = run(cli, capsys, "where", "--json")
    data = json.loads(out)
    assert code == 0 and data["asset_root"] == str(root.resolve()) and data["source"] == "environment"
    assert data["selected_pack"] is None and "removed_environment" not in data
    code, out, _ = run(cli, capsys, "where")
    assert code == 0 and "asset root" in out and str(root.resolve()) in out


def test_where_says_which_setting_to_make_when_there_is_no_root(cli, capsys, tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_WORKSPACE", str(tmp_path / "empty-ws"))
    code, _, err = run(cli, capsys, "where")
    assert code == 2 and "TWINSMITH_ASSET_ROOT" in err
    code, _, err = run(cli, capsys, "list")
    assert code == 2 and "TWINSMITH_ASSET_ROOT" in err


def test_where_names_the_project_pack(cli, capsys, root, tmp_path, monkeypatch):
    project = tmp_path / "proj"
    project.mkdir()
    (project / "twinsmith.json").write_text(json.dumps({"pack": "bricksim/v1"}))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(project))
    code, out, _ = run(cli, capsys, "where")
    assert code == 0 and "bricksim/v1" in out and "warning" not in out


def test_where_with_a_bad_project_file_is_a_clean_error(cli, capsys, root, tmp_path, monkeypatch):
    project = tmp_path / "proj"
    project.mkdir()
    (project / "twinsmith.json").write_text(json.dumps({"asset_path": ["x"]}))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(project))
    code, _, err = run(cli, capsys, "where")
    assert code == 2 and "not supported any more" in err


# ---------------------------------------------------------------- adopt, list, doctor, pin


def test_adopt_writes_a_verifying_pack_and_list_shows_it(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path)
    code, out, _ = run(cli, capsys, "adopt", str(source), "--pack", "mine/v1", *ADOPT)
    assert code == 0 and "wrote mine/v1: 2 file(s)" in out
    manifest = pm.AssetPack(root, "mine", "v1").manifest
    assert {r["path"]: r["sha256"] for r in manifest["files"]} == {"robot/a.urdf": sha(b"<robot name='a'/>"), "robot/mesh.stl": sha(b"mesh")}
    assert all(r["redistributable"] is False and r["source"] == "vendor CAD, 2026" for r in manifest["files"])
    code, out, _ = run(cli, capsys, "list")
    assert code == 0 and "mine/v1: 2 file(s)" in out and "verified" in out and "NOT VERIFIED" not in out
    code, out, _ = run(cli, capsys, "list", "--json")
    assert json.loads(out)["packs"][0]["verified"] is True


def test_adopt_needs_a_source_and_a_license_and_never_touches_a_version(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path)
    with pytest.raises(SystemExit):
        cli.main(["adopt", str(source), "--pack", "mine/v1"])  # argparse: --source and --license are required
    capsys.readouterr()
    assert run(cli, capsys, "adopt", str(source), "--pack", "mine/v1", *ADOPT)[0] == 0
    before = (root / "mine/v1/pack.yaml").read_bytes()
    code, _, err = run(cli, capsys, "adopt", str(source), "--pack", "mine/v1", *ADOPT)
    assert code == 2 and "immutable" in err and "mine/v2" in err
    assert (root / "mine/v1/pack.yaml").read_bytes() == before
    assert run(cli, capsys, "adopt", str(source), "--pack", "mine/v2", *ADOPT, "--redistributable")[0] == 0
    assert pm.AssetPack(root, "mine", "v2").manifest["files"][0]["redistributable"] is True


def test_adopt_dry_run_writes_nothing_and_a_bad_pack_name_is_refused(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path)
    code, out, _ = run(cli, capsys, "adopt", str(source), "--pack", "mine/v1", *ADOPT, "--dry-run")
    assert code == 0 and "would write mine/v1" in out and not (root / "mine").exists()
    for bad in ("mine", "../x/v1", "a/b/c"):
        code, _, err = run(cli, capsys, "adopt", str(source), "--pack", bad, *ADOPT)
        assert code == 2 and "pack" in err
    assert list(root.iterdir()) == []


def test_adopt_include_and_file(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path, {"a.usd": "a", "x.weird": "w", "tex/t.png": "t", "cards/c.png": "c"})
    assert run(cli, capsys, "adopt", str(source), "--pack", "p/v1", *ADOPT, "--include", "weird", "--file", "a.usd",
               "--file", "tex")[0] == 0
    assert sorted(r["path"] for r in pm.AssetPack(root, "p", "v1").manifest["files"]) == ["a.usd", "tex/t.png"]
    assert run(cli, capsys, "adopt", str(source), "--pack", "q/v1", *ADOPT, "--include", "weird")[0] == 0
    assert "x.weird" in [r["path"] for r in pm.AssetPack(root, "q", "v1").manifest["files"]]


def test_adopt_never_follows_a_symlink_and_says_so(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path, {"a.usd": "a"})
    secret = tmp_path / "secret.usd"
    secret.write_text("secret")
    (source / "link.usd").symlink_to(secret)
    code, out, _ = run(cli, capsys, "adopt", str(source), "--pack", "p/v1", *ADOPT)
    assert code == 0 and "symbolic link" in out and "link.usd" in out
    assert [r["path"] for r in pm.AssetPack(root, "p", "v1").manifest["files"]] == ["a.usd"]


def test_adopt_from_old_converts_in_place_and_list_stops_calling_it_old(cli, capsys, root):
    old = root / "workcell"
    (old / "robot").mkdir(parents=True)
    (old / "robot" / "ur5e.usd").write_bytes(b"robot")
    (old / "SHA256SUMS").write_text(f"{sha(b'robot')}  robot/ur5e.usd\n")
    code, out, _ = run(cli, capsys, "list")
    assert code == 0 and "workcell: old format" in out and "adopt" in out
    code, out, _ = run(cli, capsys, "adopt", "--from-old", str(old), "--source", "lab files", "--license", "private")
    assert code == 0 and "wrote workcell/v1" in out and "old files are still in" in out
    assert pm.AssetPack(root, "workcell", "v1").manifest["files"][0]["sha256"] == sha(b"robot")
    code, out, _ = run(cli, capsys, "list")
    assert "old format" not in out and "workcell/v1" in out
    assert (old / "SHA256SUMS").exists()


def test_adopt_from_old_with_a_chosen_version(cli, capsys, root):
    old = root / "workcell"
    old.mkdir()
    (old / "a.usd").write_bytes(b"a")
    assert run(cli, capsys, "adopt", "--from-old", str(old), "--pack", "workcell/v7", "--source", "s", "--license", "l")[0] == 0
    assert (root / "workcell/v7/pack.yaml").is_file()
    code, _, err = run(cli, capsys, "adopt", "--from-old", str(old), "--pack", "other/v1", "--source", "s", "--license", "l")
    assert code == 2 and "folder's name" in err


def test_doctor_verifies_a_pack_and_exits_1_on_a_problem(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path)
    run(cli, capsys, "adopt", str(source), "--pack", "mine/v1", *ADOPT)
    code, out, _ = run(cli, capsys, "doctor", "mine/v1", "--json")
    data = json.loads(out)
    assert code == 0 and data["ok"] is True and data["files"]["verified"] is True and data["dependencies"]["checked"] is True
    code, out, _ = run(cli, capsys, "doctor", "mine/v1")
    assert code == 0 and "every file matches pack.yaml" in out and out.strip().endswith("ok")
    (root / "mine/v1/robot/mesh.stl").write_text("changed")
    code, out, _ = run(cli, capsys, "doctor", "mine/v1")
    assert code == 1 and "does NOT verify" in out and "Checksum mismatch: robot/mesh.stl" in out and "NOT OK" in out


def test_doctor_without_a_pack_uses_the_project_file_or_asks(cli, capsys, root, tmp_path, monkeypatch):
    code, _, err = run(cli, capsys, "doctor")
    assert code == 2 and "<pack id>/<version>" in err
    make = tmp_path / "proj"
    make.mkdir()
    (make / "twinsmith.json").write_text(json.dumps({"pack": "mine/v1"}))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(make))
    run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "mine/v1", *ADOPT)
    assert run(cli, capsys, "doctor")[0] == 0


def test_doctor_of_an_absent_pack_is_a_refusal(cli, capsys, root):
    code, _, err = run(cli, capsys, "doctor", "nope/v1")
    assert code == 2 and "nope/v1" in err


def test_pin_shows_what_changed_then_rewrites_and_a_verified_pack_is_left_alone(cli, capsys, root, tmp_path):
    run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "mine/v1", *ADOPT)
    before = (root / "mine/v1/pack.yaml").read_bytes()
    code, out, _ = run(cli, capsys, "pin", "mine/v1")
    assert code == 0 and "already verifies" in out and (root / "mine/v1/pack.yaml").read_bytes() == before
    (root / "mine/v1/robot/mesh.stl").write_text("edited")
    code, out, _ = run(cli, capsys, "pin", "mine/v1", "--dry-run")
    assert code == 0 and "changed: robot/mesh.stl" in out and "dry run" in out and (root / "mine/v1/pack.yaml").read_bytes() == before
    code, out, _ = run(cli, capsys, "pin", "mine/v1")
    assert code == 0 and sha(b"mesh") in out and sha(b"edited") in out and "rewrote 1 hash" in out
    pm.AssetPack(root, "mine", "v1")  # verifies again


def test_pin_a_named_file_only(cli, capsys, root, tmp_path):
    run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "mine/v1", *ADOPT)
    (root / "mine/v1/robot/mesh.stl").write_text("edited")
    (root / "mine/v1/robot/a.urdf").write_text("<robot name='b'/>")
    assert run(cli, capsys, "pin", "mine/v1", "--file", "robot/mesh.stl")[0] == 0
    hashes = {r["path"]: r["sha256"] for r in json.loads((root / "mine/v1/pack.yaml").read_text())["files"]}
    assert hashes["robot/mesh.stl"] == sha(b"edited") and hashes["robot/a.urdf"] == sha(b"<robot name='a'/>")


def test_pin_cannot_add_a_file_the_manifest_does_not_list(cli, capsys, root, tmp_path):
    run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "mine/v1", *ADOPT)
    (root / "mine/v1/extra.usd").write_text("x")
    (root / "mine/v1/robot/mesh.stl").write_text("edited")
    code, _, err = run(cli, capsys, "pin", "mine/v1")
    assert code == 2 and "new version" in err


# ---------------------------------------------------------------- build


def test_build_copies_named_files_and_what_they_depend_on_into_a_new_version(cli, capsys, root, tmp_path):
    urdf = ("<robot name='r'><link name='l'><visual><geometry><mesh filename='meshes/a.stl'/></geometry></visual></link></robot>")
    source = source_folder(tmp_path, {"r.urdf": urdf, "meshes/a.stl": "a", "meshes/unused.stl": "u", "notes.txt": "n"})
    assert run(cli, capsys, "adopt", str(source), "--pack", "big/v1", "--include", "txt", *ADOPT)[0] == 0
    code, out, _ = run(cli, capsys, "build", "r.urdf", "--pack", "small/v1", "--from-pack", "big/v1")
    assert code == 0, out
    files = sorted(r["path"] for r in pm.AssetPack(root, "small", "v1").manifest["files"])
    assert files == ["meshes/a.stl", "r.urdf"] and "wrote small/v1" in out
    record = pm.AssetPack(root, "small", "v1").manifest["files"][0]
    assert record["source"] == "vendor CAD, 2026" and record["license"] == "private lab use"  # inherited
    code, _, err = run(cli, capsys, "build", "r.urdf", "--pack", "small/v1", "--from-pack", "big/v1")
    assert code == 2 and "immutable" in err
    code, out, _ = run(cli, capsys, "build", "r.urdf", "--pack", "small/v2", "--from-pack", "big/v1", "--dry-run")
    assert code == 0 and "would copy small/v2" in out and not (root / "small/v2").exists()


def test_build_refuses_a_source_pack_that_does_not_verify(cli, capsys, root, tmp_path):
    run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "big/v1", *ADOPT)
    (root / "big/v1/robot/mesh.stl").write_text("changed")
    code, _, err = run(cli, capsys, "build", "robot/a.urdf", "--pack", "small/v1", "--from-pack", "big/v1")
    assert code == 2 and "does not verify" in err and not (root / "small").exists()


def test_build_from_files_outside_a_pack_needs_source_and_license(cli, capsys, root, tmp_path):
    source = source_folder(tmp_path, {"r.urdf": "<robot name='r'/>"})
    code, _, err = run(cli, capsys, "build", str(source / "r.urdf"), "--pack", "mine/v1")
    assert code == 2 and "source and license" in err
    code, out, _ = run(cli, capsys, "build", str(source / "r.urdf"), "--pack", "mine/v1", "--source", "me", "--license", "mit")
    assert code == 0, out
    assert pm.AssetPack(root, "mine", "v1").manifest["files"][0]["license"] == "mit"


# ---------------------------------------------------------------- safe writes


def test_a_root_inside_the_repository_is_refused(cli, capsys, tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(REPO / "tmp-never-created"))
    code, _, err = run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)
    assert code == 2 and "inside the repository" in err and not (REPO / "tmp-never-created").exists()


def test_a_root_reached_through_a_symlink_into_the_repository_is_refused(cli, capsys, tmp_path, monkeypatch):
    link = tmp_path / "link"
    link.symlink_to(REPO)
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(link))
    code, _, err = run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)
    assert code == 2 and "inside the repository" in err


def test_a_symlinked_pack_folder_is_refused_and_writes_nothing_through_it(cli, capsys, root, tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    (root / "p").symlink_to(outside)
    code, _, err = run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)
    assert code == 2 and list(outside.iterdir()) == []


def test_new_folders_are_not_group_writable_even_with_umask_002(cli, capsys, tmp_path, monkeypatch):
    fresh = tmp_path / "fresh-root"
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(fresh))
    old = os.umask(0o002)
    try:
        assert run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)[0] == 0
    finally:
        os.umask(old)
    for folder in (fresh / "p", fresh / "p" / "v1", fresh / "p" / "v1" / "robot"):
        assert folder.stat().st_mode & 0o022 == 0, folder


def test_an_existing_group_writable_pack_folder_is_tightened(cli, capsys, root, tmp_path):
    (root / "p").mkdir()
    (root / "p").chmod(0o775)
    code, out, _ = run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)
    assert code == 0 and "removed group/world write permission" in out and (root / "p").stat().st_mode & 0o022 == 0


def test_a_failed_copy_leaves_nothing_behind(cli, capsys, root, tmp_path, monkeypatch):
    from twinsmith import asset_packs

    def boom(*args, **kwargs):
        raise OSError("disk exploded")

    monkeypatch.setattr(asset_packs.shutil, "copyfileobj", boom)
    code, _, err = run(cli, capsys, "adopt", str(source_folder(tmp_path)), "--pack", "p/v1", *ADOPT)
    assert code == 2 and "disk exploded" in err
    assert not (root / "p" / "v1").exists() and not [p for p in root.iterdir() if p.name.startswith(".staging")]


def test_the_cli_is_importable_without_a_simulator_and_has_no_removed_commands():
    spec = importlib.util.spec_from_file_location("isaac_assets_cli_test2", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from twinsmith import assets_cli

    assert module.main is assets_cli.main
    assert sorted(assets_cli.COMMANDS) == ["adopt", "build", "doctor", "list", "pin", "pull", "push", "where"]
    for removed in ("migrate", "unpin", "create", "add", "packs", "pack-build"):
        with pytest.raises(SystemExit):
            assets_cli.build_parser().parse_args([removed])
