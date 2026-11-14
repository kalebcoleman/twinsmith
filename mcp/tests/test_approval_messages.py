"""Approval refusals end in one instruction the user can follow, and host and Kit report when they disagree on the project."""

import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from twinsmith import asset_packs as packs
from twinsmith import project_assets as a
from twinsmith import sim_sessions as sim


def _printed_argv(text):
    """The arguments of the command on its own line in the message, as a shell would split them."""
    import shlex

    line = next(x for x in text.splitlines() if "twinsmith.project_assets" in x)
    words = shlex.split(line.strip())
    return words[words.index("twinsmith.project_assets") + 1:]  # after: <python> -m twinsmith.project_assets


def test_the_approval_command_is_filled_in(monkeypatch):
    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", "/srv/isaac/approval.json")
    text = a.approval_help("/work/projects/arm demo/assets")
    assert "--config /srv/isaac/approval.json" in text
    assert "--root '/work/projects/arm demo/assets'" in text and "--root-id arm-demo-assets" in text  # one shell word
    assert "ask the user to run" in text and "an agent cannot" in text
    assert "reconnect the MCP server and restart Kit" in text.replace("Then reconnect", "reconnect")  # both read the file at startup
    assert "smallest folder" in text and "1000 files" in text  # a large parent fails with LIMIT: do not advise it
    assert "covers every project" not in text
    assert "--trust-project" in text  # the helper exits with an error without it
    assert "--staging" in text  # the file does not exist yet, and the helper needs staging to create it


def test_an_existing_approval_file_needs_no_staging(tmp_path, monkeypatch):
    config = tmp_path / "approval.json"
    config.write_text("{}")
    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", str(config))
    text = a.approval_help("/work/projects/demo")
    assert "--trust-project" in text and "--staging" not in text


def test_the_printed_command_is_accepted_by_the_helper(tmp_path, monkeypatch):
    """Run the command the refusal prints (as its argv): the helper must not reject it."""
    import shlex

    config = tmp_path / "private" / "approval.json"
    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", str(config))
    monkeypatch.setenv("TWINSMITH_STAGING", str(tmp_path / "private" / "staging"))
    (tmp_path / "projects" / "demo").mkdir(parents=True)
    text = a.approval_help(str(tmp_path / "projects" / "demo" / "robot.urdf"))
    argv = _printed_argv(text)
    config.parent.mkdir(mode=0o700)
    assert a.main(argv) in (0, None)  # parser.error would raise SystemExit
    assert config.exists()


def test_a_file_is_turned_into_the_folder_to_approve(monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    text = a.approval_help("/work/projects/demo/assets/robot.urdf")
    assert "--root /work/projects/demo/assets " in text and "robot.urdf --" not in text and "--root-id demo-assets" in text
    assert "<a private approval.json" in text  # no config yet: it says what to create, never a guessed path


def test_without_a_folder_the_command_has_a_placeholder(monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    assert "--root <the folder>" in a.approval_help(None)


def test_a_root_id_is_made_safe(monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    assert "--root-id my-weird-name" in a.approval_help("/x/my weird;name")


def test_the_refusals_for_an_unapproved_path_carry_the_command(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    broker = a.Broker()
    outside = tmp_path / "proj" / "assets" / "robot.urdf"
    outside.parent.mkdir(parents=True)
    outside.write_text("<robot/>")
    with pytest.raises(a.AssetError) as error:
        broker.locate(str(outside), "urdf")
    assert error.value.code == "ROOT_NOT_APPROVED" and f"--root {outside.parent}" in str(error.value)


def test_pack_tools_refuse_with_the_command_too(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(tmp_path / "store"))
    (tmp_path / "outside").mkdir()
    with pytest.raises(packs.PackError) as error:
        packs.run_tool("assets.pack_adopt", dict(pack="mine/v1", source_path=str(tmp_path / "outside"), source="s", license="l"),
                       a.Broker())
    assert error.value.code == "ROOT_NOT_APPROVED" and "ask the user to run" in str(error.value)


# ---- host and Kit seeing the same project -------------------------------------------------------------------------

HOST = dict(project_file="/p/twinsmith.json", asset_root="/w/assets")


def test_no_warning_when_host_and_kit_agree():
    assert packs.environment_warnings(HOST, dict(HOST)) == []


def test_a_kit_that_does_not_see_the_project_is_named():
    warnings = packs.environment_warnings(HOST, dict(HOST, project_file=None, asset_root="/other"))
    assert any("project file" in w and "launch_simulator" in w and "TWINSMITH_PROJECT" in w for w in warnings)
    assert any("asset root" in w and "TWINSMITH_ASSET_ROOT" in w for w in warnings)


def test_a_different_asset_root_is_reported_on_its_own():
    warnings = packs.environment_warnings(HOST, dict(HOST, asset_root="/kit/assets"))
    assert len(warnings) == 1 and "/w/assets" in warnings[0] and "/kit/assets" in warnings[0]


def test_packs_lists_a_warning_when_kit_disagrees(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(tmp_path / "store"))
    monkeypatch.delenv("TWINSMITH_PROJECT", raising=False)
    monkeypatch.chdir(tmp_path)
    (tmp_path / "store").mkdir()
    kit = lambda tool, args: {"asset_environment": dict(project_file="/elsewhere/twinsmith.json", asset_root="/other")}
    out = packs.run_tool("assets.packs", {}, a.Broker(), kit_call=kit)
    assert out["kit_environment"]["project_file"] == "/elsewhere/twinsmith.json" and len(out["warnings"]) == 2


def test_packs_without_a_kit_is_unchanged(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(tmp_path / "store"))
    (tmp_path / "store").mkdir()
    out = packs.run_tool("assets.packs", {}, a.Broker())
    assert "warnings" not in out and "kit_environment" not in out
    broken = packs.run_tool("assets.packs", {}, a.Broker(), kit_call=lambda t, x: (_ for _ in ()).throw(OSError("no bridge")))
    assert "warnings" not in broken  # no bridge is the ordinary case, not a warning


def test_a_kit_that_predates_the_report_gives_no_warning(tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(tmp_path / "store"))
    (tmp_path / "store").mkdir()
    out = packs.run_tool("assets.packs", {}, a.Broker(), kit_call=lambda t, x: {"status": "ok"})
    assert "warnings" not in out


def test_a_path_with_a_space_is_printed_as_one_shell_word_and_the_helper_accepts_it(tmp_path, monkeypatch):
    import shlex

    config = tmp_path / "my private" / "approval.json"
    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", str(config))
    monkeypatch.setenv("TWINSMITH_STAGING", str(tmp_path / "my private" / "staging"))
    folder = tmp_path / "arm demo" / "assets"
    folder.mkdir(parents=True)
    text = a.approval_help(str(folder / "robot.urdf"))
    argv = _printed_argv(text)
    assert str(config) in argv and str(folder) in argv  # each path survived as a single argument
    config.parent.mkdir(mode=0o700)
    assert a.main(argv) in (0, None) and config.exists()


def test_no_folder_gives_a_placeholder_and_a_sensible_root_id(monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    text = a.approval_help(None)
    assert "--root-id project " in text and "--root-id None" not in text and "--root <the folder>" in text


def test_a_folder_named_with_a_dot_is_a_folder_not_a_file(tmp_path, monkeypatch):
    """`robots v1.2` and `repo.git` have a suffix but are folders: approve them, not their parent."""
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    folder = tmp_path / "my project" / "robots v1.2"
    folder.mkdir(parents=True)
    argv = _printed_argv(a.approval_help(str(folder)))
    assert argv[argv.index("--root") + 1] == str(folder)
    argv = _printed_argv(a.approval_help(str(tmp_path / "repo.git")))  # not on disk: not a known file suffix either
    assert argv[argv.index("--root") + 1] == str(tmp_path / "repo.git")
    (folder / "robot.urdf").write_text("<robot/>")
    argv = _printed_argv(a.approval_help(str(folder / "robot.urdf")))  # a file still names its folder
    assert argv[argv.index("--root") + 1] == str(folder)


@pytest.mark.parametrize("name", ["repo.git", "isaac_assets_v1.0", "x" * 80, "my robots (v2)"])
def test_the_root_id_is_one_the_approval_helper_accepts(tmp_path, monkeypatch, name):
    """Every printed command must run as printed: the helper takes only [A-Za-z0-9_-], 1 to 48 characters."""
    import re

    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", str(tmp_path / "private" / "approval.json"))
    monkeypatch.setenv("TWINSMITH_STAGING", str(tmp_path / "private" / "staging"))
    folder = tmp_path / name
    folder.mkdir()
    argv = _printed_argv(a.approval_help(str(folder)))
    assert re.fullmatch(r"[A-Za-z0-9_-]{1,48}", argv[argv.index("--root-id") + 1])
    (tmp_path / "private").mkdir(mode=0o700)
    assert a.main(argv) in (0, None)


def test_the_command_has_no_stray_arguments_after_it(monkeypatch):
    """A comma after --trust-project made a copied command fail: the command ends its own line."""
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    line = next(x for x in a.approval_help("/work/p/assets").splitlines() if "twinsmith.project_assets" in x)
    assert line.rstrip().endswith("--trust-project")


@pytest.mark.parametrize("evil", ["<x; touch /tmp/PWNED #", "<a;touch /tmp/PWNED", "<the folder> ; rm -rf ~", "$(touch /tmp/PWNED)", "`id`", "it's", "a\nb"])
def test_a_value_that_only_looks_like_a_placeholder_is_quoted(evil):
    """Only the two exact placeholders are left bare: a folder name or config path from anywhere else is quoted, so the
    pasted command runs the helper and nothing else."""
    import shlex

    assert a._shell_word("<the folder>") == "<the folder>" and a._shell_word(a._PLACEHOLDER_CONFIG) == a._PLACEHOLDER_CONFIG
    word = a._shell_word(evil)
    assert shlex.split(word) == [evil]  # one shell word that is exactly the value, nothing executed


def test_a_hostile_config_or_folder_cannot_add_commands_to_the_printed_line(monkeypatch):
    import shlex

    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", "<x;touch /tmp/PWNED2")
    text = a.approval_help("<y; touch /tmp/PWNED #/assets")
    argv = _printed_argv(text)
    assert argv[argv.index("--config") + 1] == "<x;touch /tmp/PWNED2"
    assert argv[argv.index("--root") + 1] == "<y; touch /tmp/PWNED #/assets"
    line = next(x for x in text.splitlines() if "twinsmith.project_assets" in x)
    words = shlex.split(line.strip())
    assert words[:2] == ["uv", "run"] and ";" not in words  # the line is one command: no extra tokens ran or split off


@pytest.mark.parametrize("name", ["model.glb", "scene.gltf", "mesh.fbx", "robot.sdf", "points.ply", "sky.hdr", "tex.exr", "table.csv"])
def test_more_file_suffixes_name_a_file_when_it_is_not_on_disk(monkeypatch, name):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    argv = _printed_argv(a.approval_help(f"/work/p/assets/{name}"))
    assert argv[argv.index("--root") + 1] == "/work/p/assets"


def _root_of(text):
    argv = _printed_argv(text)
    return argv[argv.index("--root") + 1]


def test_a_urdf_in_a_ros_package_approves_the_package_not_its_urdf_folder(tmp_path, monkeypatch):
    """The command offered `<package>/urdf`, whose meshes (package://...) live above it."""
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    pkg = tmp_path / "repo" / "robots" / "go2_description"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>")
    (pkg / "urdf" / "go2.urdf").write_text("<robot/>")
    (pkg / "urdf" / "deep" / "x").mkdir(parents=True)
    (pkg / "urdf" / "deep" / "x" / "other.urdf").write_text("<robot/>")
    assert _root_of(a.approval_help(str(pkg / "urdf" / "go2.urdf"))) == str(pkg)
    assert _root_of(a.approval_help(str(pkg / "urdf" / "deep" / "x" / "other.urdf"))) == str(pkg)


def test_a_urdf_folder_beside_meshes_widens_to_its_parent_without_a_package_file(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    robot = tmp_path / "robot"
    (robot / "urdf").mkdir(parents=True)
    (robot / "meshes").mkdir()
    (robot / "urdf" / "r.urdf").write_text("<robot/>")
    assert _root_of(a.approval_help(str(robot / "urdf" / "r.urdf"))) == str(robot)
    lone = tmp_path / "lone"
    lone.mkdir()
    (lone / "r.urdf").write_text("<robot/>")
    assert _root_of(a.approval_help(str(lone / "r.urdf"))) == str(lone)  # nothing to widen to


def test_the_package_search_never_reaches_the_home_folder_or_above(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    (tmp_path / "home" / "work").mkdir(parents=True)
    (tmp_path / "home" / "package.xml").write_text("<package/>")  # a stray file in the home folder
    (tmp_path / "home" / "work" / "r.urdf").write_text("<robot/>")
    assert _root_of(a.approval_help(str(tmp_path / "home" / "work" / "r.urdf"))) == str(tmp_path / "home" / "work")


def test_a_usd_or_mjcf_file_still_approves_its_own_folder(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    (tmp_path / "m").mkdir()
    (tmp_path / "m" / "package.xml").write_text("<package/>")
    for name in ("a.usd", "a.xml"):
        assert _root_of(a.approval_help(str(tmp_path / "m" / "sub" / name))) == str(tmp_path / "m" / "sub")


@pytest.mark.parametrize("path", ["/etc/hostname", "/etc", "/usr/share/x/robot.urdf", "/root/m.xml", "/", "/var/lib/x/y.usd", "/proc/self/environ"])
def test_no_command_is_offered_for_a_system_folder(path, monkeypatch):
    """An agent can name any path in a refusal; the user must not be handed a one-line command to approve /etc."""
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    text = a.approval_help(path)
    assert "twinsmith.project_assets" not in text and "no approval command is offered" in text


def test_no_command_is_offered_for_the_home_folder_or_its_private_folders(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    for sub in (".ssh", ".gnupg", ".aws", ".config/x", ".kube", ".mozilla/y"):
        (tmp_path / sub).mkdir(parents=True)
        assert "twinsmith.project_assets" not in a.approval_help(str(tmp_path / sub / "k.urdf")), sub
    assert "twinsmith.project_assets" not in a.approval_help(str(tmp_path / "robot.urdf"))  # the home folder itself
    (tmp_path / "projects" / "robot").mkdir(parents=True)
    assert "twinsmith.project_assets" in a.approval_help(str(tmp_path / "projects" / "robot" / "r.urdf"))  # a project folder is fine


def test_a_symlink_into_a_system_folder_is_judged_by_where_it_points(tmp_path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    (tmp_path / "link").symlink_to("/etc")
    assert "twinsmith.project_assets" not in a.approval_help(str(tmp_path / "link" / "hostname"))


def test_the_users_own_local_and_cache_folders_are_ordinary_project_homes(tmp_path, monkeypatch):
    """~/.local/share/<tool> is where many tools keep their data: a robot folder there is not a secret."""
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    for sub in (".local/share/robots/go2", ".cache/models/x"):
        (tmp_path / sub).mkdir(parents=True)
        assert "twinsmith.project_assets" in a.approval_help(str(tmp_path / sub / "m.xml")), sub


@pytest.mark.parametrize("path", ["/tmp/robot/m.xml", "/opt/robots/m.xml", "/mnt/data/robots/m.xml", "/srv/robots/m.xml"])
def test_a_folder_under_tmp_opt_mnt_or_srv_is_offered(path, monkeypatch):
    monkeypatch.delenv("TWINSMITH_ASSET_CONFIG", raising=False)
    assert "twinsmith.project_assets" in a.approval_help(path)


def test_the_printed_python_exists_after_install_sh(monkeypatch):
    """install.sh removes any uv tool and installs into mcp/.venv, so "$(uv tool dir)" named no Python.
    The command names this checkout's mcp project, the same in the MCP server and in Kit."""
    import shlex

    monkeypatch.setenv("TWINSMITH_ASSET_CONFIG", "/srv/isaac/approval.json")
    line = next(x for x in a.approval_help("/work/projects/demo").splitlines() if "twinsmith.project_assets" in x)
    assert "uv tool dir" not in line
    words = shlex.split(line.strip())
    project = Path(words[words.index("--project") + 1])
    assert words[:2] == ["uv", "run"] and (project / "pyproject.toml").is_file() and (project / "twinsmith").is_dir()
    assert words[words.index("twinsmith.project_assets") - 1] == "-m"


def test_an_installed_wheel_prints_the_servers_own_python(tmp_path):
    """A wheel has no uv project beside the package, so the command names the interpreter that has twinsmith."""
    site = tmp_path / "lib/python3.12/site-packages"
    site.mkdir(parents=True)
    assert a.helper_python(site, "/opt/tools/twin smith/bin/python") == "'/opt/tools/twin smith/bin/python'"
    assert a.helper_python(site, "") == "python3"
    assert a.helper_python(Path(a.__file__).resolve().parents[1], "/x").startswith("uv run --locked --isolated --project ")
