"""The release: one version source with checked copies (scripts/version.py), and a workflow that
publishes only from a v* tag through trusted publishing. See docs/RELEASING.md."""

import importlib.util
from pathlib import Path
import shutil

import pytest

ROOT = Path(__file__).resolve().parents[2]
WORKFLOWS = ROOT / ".github" / "workflows"
_spec = importlib.util.spec_from_file_location("release_version", ROOT / "scripts" / "version.py")
version = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(version)

COPIED = [version.SOURCE, *(copy.path for copy in version.COPIES)]


def test_every_copy_holds_the_source_version():
    current = version.current()
    assert version.VERSION.fullmatch(current)
    assert version.drift(current) == [], "run python3 scripts/version.py set <version>"
    assert version.main(["check"]) == 0


def test_the_badge_is_the_rendered_one():
    badge = version.Badge()
    assert (ROOT / badge.path).read_text(encoding="utf-8") == badge.render(version.current())


@pytest.fixture
def tree(tmp_path, monkeypatch):
    """A copy of the files that hold the version, for set and check to change."""
    for relative in COPIED:
        (tmp_path / relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, tmp_path / relative)
    monkeypatch.setattr(version, "ROOT", tmp_path)
    return tmp_path


def test_set_writes_every_copy_and_nothing_else(tree):
    old = version.current()
    before = {relative: (tree / relative).read_text(encoding="utf-8") for relative in COPIED}
    assert version.main(["set", "12.0.0rc3"]) == 0
    assert version.current() == "12.0.0rc3" and version.drift("12.0.0rc3") == []
    for relative in COPIED:
        old_lines = before[relative].splitlines()
        new_lines = (tree / relative).read_text(encoding="utf-8").splitlines()
        changed = [(a, b) for a, b in zip(old_lines, new_lines) if a != b]
        assert len(old_lines) == len(new_lines) and changed, relative
        if relative != version.Badge.path:     # the badge is redrawn; every other file changes one line
            ((a, b),) = changed
            assert b == a.replace(old, "12.0.0rc3"), relative
    assert version.main(["set", old]) == 0
    assert {relative: (tree / relative).read_text(encoding="utf-8") for relative in COPIED} == before


def test_a_drifted_copy_fails_the_check(tree, capsys):
    lock = tree / "mcp" / "uv.lock"
    current = version.current()
    lock.write_text(lock.read_text(encoding="utf-8").replace(
        f'name = "twinsmith"\nversion = "{current}"', 'name = "twinsmith"\nversion = "0.0.1"'), encoding="utf-8")
    assert version.drift(current) == [f"mcp/uv.lock has 0.0.1, mcp/pyproject.toml has {current}"]
    assert version.main(["check"]) == 1
    assert "scripts/version.py set" in capsys.readouterr().err


@pytest.mark.parametrize("bad", ["v1.0", "1.0.0-rc1", "1.0.0.RC1", "", "1.0 ", "01.2.3", "1.0.0rc01"])
def test_set_refuses_a_non_canonical_version(tree, bad):
    old = version.current()
    with pytest.raises(SystemExit):
        version.main(["set", bad])
    assert version.current() == old and version.drift(old) == []


def test_the_tag_must_be_v_and_the_version():
    assert version.tag_errors("v1.2.3", "1.2.3") == []
    assert version.tag_errors("refs/tags/v1.2.3rc1", "1.2.3rc1") == []
    for tag in ("1.2.3", "v1.2.4", "v1.2.3.0", "V1.2.3", "v1.2.3-rc1"):
        assert version.tag_errors(tag, "1.2.3"), tag


def test_dist_must_hold_exactly_this_version(tmp_path):
    good = ["twinsmith-1.2.3-py3-none-any.whl", "twinsmith-1.2.3.tar.gz"]
    for name in good:
        (tmp_path / name).touch()
    assert version.dist_errors(tmp_path, "1.2.3") == []
    assert version.dist_errors(tmp_path, "1.2.4")
    (tmp_path / "twinsmith-1.2.2.tar.gz").touch()
    assert version.dist_errors(tmp_path, "1.2.3")
    assert version.dist_errors(tmp_path / "missing", "1.2.3")


def _workflow(name):
    yaml = pytest.importorskip("yaml")
    document = yaml.safe_load((WORKFLOWS / name).read_text(encoding="utf-8"))
    document["on"] = document.pop(True, document.get("on"))   # YAML 1.1 reads the key `on` as True
    return document


def test_release_runs_only_on_a_version_tag_and_publishes_through_oidc():
    release = _workflow("release.yml")
    assert release["on"] == {"push": {"tags": ["v[0-9]*"]}}
    assert release["permissions"] == {}
    build, publish = release["jobs"]["build"], release["jobs"]["publish"]
    assert build["uses"] == "./.github/workflows/build-dist.yml"
    assert build["with"] == {"release-tag": "${{ github.ref_name }}"}
    assert publish["needs"] == "build"
    assert publish["environment"]["name"] == "pypi"
    assert publish["permissions"] == {"id-token": "write"}
    (step,) = [step for step in publish["steps"] if "pypi-publish" in step.get("uses", "")]
    assert step["uses"].startswith("pypa/gh-action-pypi-publish@")
    assert "with" not in step, "trusted publishing needs no password or token input"


def test_no_workflow_holds_a_publishing_secret_or_publishes_elsewhere():
    for path in sorted(WORKFLOWS.glob("*.yml")):
        text = path.read_text(encoding="utf-8")
        assert "secrets." not in text and "password" not in text.lower(), path.name
        assert "npm publish" not in text, path.name
        if path.name != "release.yml":
            assert "pypi-publish" not in text and "id-token" not in text, path.name


def test_the_check_build_runs_on_every_pull_request_without_publishing():
    ci = _workflow("ci.yml")
    assert "pull_request" in ci["on"]
    assert ci["jobs"]["package"]["uses"] == "./.github/workflows/build-dist.yml"
    assert "with" not in ci["jobs"]["package"]
    build = _workflow("build-dist.yml")
    assert list(build["on"]) == ["workflow_call"]
    assert build["permissions"] == {"contents": "read"}
    runs = "\n".join(step.get("run", "") for step in build["jobs"]["build"]["steps"])
    assert "uv build --no-sources" in runs and "scripts/version.py check --tag" in runs
