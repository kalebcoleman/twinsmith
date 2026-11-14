"""Pack sources: the sources file, pull and push against a fake Hugging Face hub (no network, no huggingface_hub)."""

from __future__ import annotations

import fnmatch
import hashlib
import json
from pathlib import Path

import pytest

from twinsmith import asset_packs, assets_cli, pack_manifest, pack_sources
from twinsmith.asset_packs import PackError

READ_TOKEN = "hf_readTOKENvalue0123456789abcdef"
WRITE_TOKEN = "hf_writeTOKENvalue0123456789abcdef"
REPO = "someone/lab-assets"
SETTINGS = ("TWINSMITH_ASSET_ROOT", "TWINSMITH_WORKSPACE", "TWINSMITH_PROJECT")


# ---------------------------------------------------------------------------
# A fake hub: repositories of commits, each a full {path: bytes} tree
# ---------------------------------------------------------------------------


class RepositoryNotFoundError(Exception):
    pass


class GatedRepoError(RepositoryNotFoundError):
    pass


class RevisionNotFoundError(Exception):
    pass


class EntryNotFoundError(Exception):
    pass


class Info:
    def __init__(self, private, sha):
        self.private, self.sha = private, sha


class Commit:
    def __init__(self, oid):
        self.oid = oid


class FakeHub:
    def __init__(self):
        self.repos = {}           # repo -> dict(private, readers, writers, commits=[(sha, tree)])
        self.calls = []

    def add_repo(self, repo, private=True, readers=(READ_TOKEN, WRITE_TOKEN), writers=(WRITE_TOKEN,)):
        sha = hashlib.sha1(repo.encode()).hexdigest()
        self.repos[repo] = dict(private=private, readers=set(readers), writers=set(writers), commits=[(sha, {})])

    def _repo(self, repo, token, write=False):
        entry = self.repos.get(repo)
        allowed = entry and (token in (entry["writers"] if write else entry["readers"]) or (not write and not entry["private"]))
        if not allowed:
            raise RepositoryNotFoundError(f"404 {repo} (sent {token})")   # a careless message that holds the token
        return entry

    def _tree(self, repo, revision, token):
        for sha, tree in self._repo(repo, token)["commits"]:
            if sha == revision:
                return tree
        raise RevisionNotFoundError(revision)

    def hf_hub_download(self, repo_id, filename, repo_type, revision, local_dir, token):
        self.calls.append(("file", repo_id, filename, token))
        tree = self._tree(repo_id, revision, token)
        if filename not in tree:
            raise EntryNotFoundError(filename)
        out = Path(local_dir) / filename
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(tree[filename])
        return str(out)

    def snapshot_download(self, repo_id, repo_type, revision, allow_patterns, local_dir, token):
        self.calls.append(("snapshot", repo_id, tuple(allow_patterns), token))
        tree = self._tree(repo_id, revision, token)
        for name, data in tree.items():
            if any(fnmatch.fnmatch(name, pattern) for pattern in allow_patterns):
                out = Path(local_dir) / name
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_bytes(data)
        meta = Path(local_dir) / ".cache/huggingface"          # what local_dir mode leaves behind
        meta.mkdir(parents=True, exist_ok=True)
        (meta / ".gitignore").write_text("*")
        return local_dir

    def HfApi(self, token):  # noqa: N802 - the hub's name
        hub = self

        class Api:
            def repo_info(self, repo, repo_type):
                entry = hub._repo(repo, token)
                return Info(entry["private"], entry["commits"][-1][0])

            def create_repo(self, repo, repo_type, private):
                hub.calls.append(("create", repo, private))
                hub.add_repo(repo, private=private, readers=(token,), writers=(token,))

            def list_repo_files(self, repo, repo_type, revision):
                return sorted(hub._tree(repo, revision, token))

            def upload_folder(self, repo_id, repo_type, folder_path, path_in_repo, commit_message, parent_commit):
                entry = hub._repo(repo_id, token, write=True)
                assert entry["commits"][-1][0] == parent_commit
                tree = dict(entry["commits"][-1][1])
                for path in Path(folder_path).rglob("*"):
                    if path.is_file():
                        tree[f"{path_in_repo}/{path.relative_to(folder_path).as_posix()}"] = path.read_bytes()
                sha = hashlib.sha1(json.dumps(sorted(tree)).encode() + commit_message.encode()
                                   + str(len(entry["commits"])).encode()).hexdigest()
                entry["commits"].append((sha, tree))
                hub.calls.append(("upload", repo_id, path_in_repo))
                return Commit(sha)

        return Api()


class NoHub:
    """Fails on any use: proves a code path never reaches the hub."""

    def __getattr__(self, name):
        raise AssertionError(f"the hub was used: {name}")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def home(tmp_path, monkeypatch):
    for name in SETTINGS:
        monkeypatch.delenv(name, raising=False)
    home = tmp_path / "home@host"
    (home / ".config/twinsmith").mkdir(parents=True)
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(tmp_path / "no-project"))
    return home


def secret(home, name, value):
    folder = home / ".config/agent-secrets"
    folder.mkdir(parents=True, exist_ok=True, mode=0o700)
    path = folder / name
    path.write_text(value + "\n")
    path.chmod(0o600)
    return path


def make_pack(root: Path, pack_id="robot", version="v1", redistributable=False, extra=b"") -> Path:
    folder = root / pack_id / version
    files = {"robot/robot.usda": b"#usda 1.0\n" + extra, "meshes/arm.obj": b"v 0 0 0\n" * 50}
    records = []
    for rel, data in files.items():
        path = folder / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        records.append(dict(path=rel, sha256=hashlib.sha256(data).hexdigest(), source="test", license="test",
                            redistributable=redistributable))
    (folder / "pack.yaml").write_text(pack_manifest.manifest_text(dict(id=pack_id, version=version, files=records)))
    return folder


def tree_hashes(folder: Path) -> dict:
    return {p.relative_to(folder).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob("*")) if p.is_file()}


def write_sources(home, text):
    (home / pack_sources.SOURCES_FILE).write_text(text)


# ---------------------------------------------------------------------------
# The sources file
# ---------------------------------------------------------------------------


def test_sources_file_is_parsed_and_rendered_back(home):
    write_sources(home, f'''
[hub]
repo = "{REPO}"

[packs."robot/v1"]
revision = "{'a' * 40}"

[packs."other/v2"]
repo = "x/y"
revision = "{'b' * 40}"
path = "nested/other-v2"
repo_type = "model"

[packs."cad/v1"]
vendor = "https://vendor.example/download/1"
sha256 = "{'c' * 64}"
how = "download it yourself, then adopt"
''')
    sources = pack_sources.load(home)
    robot, other, cad = sources.packs["robot/v1"], sources.packs["other/v2"], sources.packs["cad/v1"]
    assert (robot.repo, robot.folder, robot.repo_type) == (REPO, "robot/v1", "dataset")
    assert (other.repo, other.folder, other.repo_type) == ("x/y", "nested/other-v2", "model")
    assert cad.vendor and cad.repo is None
    pack_sources.write(sources)
    assert pack_sources.load(home) == sources


@pytest.mark.parametrize("entry, needle", [
    ('revision = "main"', "40-character"),
    ('revision = "' + "a" * 40 + '"\npath = "../escape"', "no \"..\""),
    ('revision = "' + "a" * 40 + '"\nbranch = "main"', "unknown key branch"),
    ('repo = "no-slash"\nrevision = "' + "a" * 40 + '"', "<owner>/<name>"),
    ('vendor = "http://insecure.example"', "https://"),
])
def test_a_bad_entry_is_an_error_not_a_default(home, entry, needle):
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n\n[packs."robot/v1"]\n{entry}\n')
    with pytest.raises(PackError) as caught:
        pack_sources.load(home)
    assert caught.value.code == "INVALID_SOURCES" and needle in str(caught.value)


def test_the_shipped_example_parses(home):
    example = Path(__file__).resolve().parents[2] / "config/pack-sources.example.toml"
    sources = pack_sources.parse(pack_sources.config.tomllib.loads(example.read_text()), example)
    assert sources.packs and all(src.vendor or src.revision for src in sources.packs.values())


# ---------------------------------------------------------------------------
# push
# ---------------------------------------------------------------------------


def test_push_dry_run_checks_locally_and_contacts_nothing(home, tmp_path):
    root = tmp_path / "assets"
    make_pack(root)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n')
    report = pack_sources.push(root, "robot/v1", home=home, dry_run=True, hub=NoHub())
    assert report["outcome"] == "dry_run" and report["write_token"].startswith("missing")
    assert report["not_redistributable"] == 2 and report["path"] == "robot/v1"


def test_push_without_a_write_token_names_the_command_and_never_uses_the_read_token(home, tmp_path):
    root = tmp_path / "assets"
    make_pack(root)
    secret(home, "hf_token", READ_TOKEN)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n')
    with pytest.raises(PackError) as caught:
        pack_sources.push(root, "robot/v1", home=home, hub=NoHub())
    assert caught.value.code == "NO_WRITE_TOKEN" and "agent-secret-set hf-write" in str(caught.value)


def test_push_creates_a_private_repo_pins_the_commit_and_is_idempotent(home, tmp_path):
    root = tmp_path / "assets"
    make_pack(root)
    secret(home, "hf_write_token", WRITE_TOKEN)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n')
    hub = FakeHub()
    with pytest.raises(PackError) as caught:
        pack_sources.push(root, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "NO_REPO"
    first = pack_sources.push(root, "robot/v1", home=home, create=True, hub=hub)
    assert first["outcome"] == "pushed" and first["created_repo"] is True
    assert ("create", REPO, True) in hub.calls
    pinned = pack_sources.load(home).packs["robot/v1"]
    assert pinned.revision == first["revision"] and pinned.repo == REPO
    again = pack_sources.push(root, "robot/v1", home=home, hub=hub)
    assert again["outcome"] == "already_there" and again["revision"] == first["revision"]
    assert [c for c in hub.calls if c[0] == "upload"] == [("upload", REPO, "robot/v1")]
    assert WRITE_TOKEN not in json.dumps([first, again]) + (home / pack_sources.SOURCES_FILE).read_text()


def test_push_refuses_a_public_repo_for_private_files_but_allows_redistributable_ones(home, tmp_path):
    root = tmp_path / "assets"
    make_pack(root)
    make_pack(root, "open", "v1", redistributable=True)
    secret(home, "hf_write_token", WRITE_TOKEN)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n')
    hub = FakeHub()
    hub.add_repo(REPO, private=False)
    with pytest.raises(PackError) as caught:
        pack_sources.push(root, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "NOT_PRIVATE"
    assert pack_sources.push(root, "open/v1", home=home, hub=hub)["outcome"] == "pushed"


def test_push_never_changes_a_version_the_repo_holds(home, tmp_path):
    root, other = tmp_path / "assets", tmp_path / "other"
    make_pack(root)
    make_pack(other, extra=b"# changed\n")
    secret(home, "hf_write_token", WRITE_TOKEN)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n')
    hub = FakeHub()
    hub.add_repo(REPO)
    pack_sources.push(root, "robot/v1", home=home, hub=hub)
    with pytest.raises(PackError) as caught:
        pack_sources.push(other, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "VERSION_EXISTS"


def test_push_refuses_a_pack_that_does_not_verify(home, tmp_path):
    root = tmp_path / "assets"
    folder = make_pack(root)
    (folder / "stray.txt").write_text("not listed")
    secret(home, "hf_write_token", WRITE_TOKEN)
    with pytest.raises(PackError) as caught:
        pack_sources.push(root, "robot/v1", home=home, repo=REPO, hub=NoHub())
    assert caught.value.code == "NOT_VERIFIED"


# ---------------------------------------------------------------------------
# pull
# ---------------------------------------------------------------------------


@pytest.fixture
def published(home, tmp_path):
    """Two packs pushed to a private fake repo from a source root; the sources file pins both and a vendor entry."""
    source = tmp_path / "source"
    make_pack(source)
    make_pack(source, "jenga", "v1")
    secret(home, "hf_write_token", WRITE_TOKEN)
    write_sources(home, f'[hub]\nrepo = "{REPO}"\n\n[packs."cad/v1"]\nvendor = "https://vendor.example/cad"\n')
    hub = FakeHub()
    hub.add_repo(REPO)
    for ref in ("robot/v1", "jenga/v1"):
        pack_sources.push(source, ref, home=home, hub=hub)
    return source, hub


def test_pull_all_into_a_clean_root_is_byte_identical_and_then_up_to_date(home, tmp_path, published):
    source, hub = published
    secret(home, "hf_token", READ_TOKEN)
    root = tmp_path / "clean"
    report = pack_sources.pull(root, all_packs=True, home=home, hub=hub)
    assert report["status"] == "ok"
    outcomes = {item["pack"]: item["outcome"] for item in report["results"]}
    assert outcomes == {"cad/v1": "vendor", "jenga/v1": "pulled", "robot/v1": "pulled"}
    for ref in ("robot/v1", "jenga/v1"):
        assert tree_hashes(root / ref) == tree_hashes(source / ref)
    assert sorted(p.name for p in root.iterdir()) == ["jenga", "robot"]     # no staging, no hub metadata left
    assert {c[3] for c in hub.calls if c[0] == "snapshot"} == {READ_TOKEN}
    again = pack_sources.pull(root, all_packs=True, home=home, hub=hub)
    assert {item["outcome"] for item in again["results"] if item["pack"] != "cad/v1"} == {"up_to_date"}
    assert len([c for c in hub.calls if c[0] == "snapshot"]) == 2
    assert READ_TOKEN not in json.dumps([report, again])


def test_pull_refuses_a_tampered_pack_and_leaves_nothing(home, tmp_path, published):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN)
    for _, tree in hub.repos[REPO]["commits"]:
        if "robot/v1/meshes/arm.obj" in tree:
            tree["robot/v1/meshes/arm.obj"] = b"tampered"
    root = tmp_path / "clean"
    with pytest.raises(PackError) as caught:
        pack_sources.pull(root, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "NOT_VERIFIED" and "arm.obj" in str(caught.value)
    assert list(root.iterdir()) == []


def test_pull_without_access_is_a_clear_error_that_never_shows_the_token(home, tmp_path, published):
    _, hub = published
    secret(home, "hf_token", "hf_someOtherTOKENwithoutAccess0123")
    with pytest.raises(PackError) as caught:
        pack_sources.pull(tmp_path / "clean", "robot/v1", home=home, hub=hub)
    message = str(caught.value)
    assert caught.value.code == "NO_ACCESS"
    assert "hf_token" in message and "hf_someOther" not in message


def test_pull_with_no_token_file_is_anonymous_not_the_environments_token(home, tmp_path, published, monkeypatch):
    _, hub = published
    monkeypatch.setenv("HF_TOKEN", READ_TOKEN)
    with pytest.raises(PackError) as caught:
        pack_sources.pull(tmp_path / "clean", "robot/v1", home=home, hub=hub)
    assert caught.value.code == "NO_ACCESS" and "agent-secret-set hf" in str(caught.value)
    assert hub.calls[-1][3] is False


def test_pull_never_replaces_a_local_version_that_differs(home, tmp_path, published):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN)
    root = tmp_path / "clean"
    local = make_pack(root, extra=b"# local edit\n")
    before = tree_hashes(local)
    with pytest.raises(PackError) as caught:
        pack_sources.pull(root, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "LOCAL_DIFFERS" and tree_hashes(local) == before


def test_pull_all_reports_each_failure_and_keeps_going(home, tmp_path, published):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN)
    text = (home / pack_sources.SOURCES_FILE).read_text()
    write_sources(home, text + f'\n[packs."missing/v1"]\nrevision = "{"d" * 40}"\n')
    report = pack_sources.pull(tmp_path / "clean", all_packs=True, home=home, hub=hub)
    assert report["status"] == "error" and report["code"] == "PULL_FAILED"
    failed = [item for item in report["results"] if item["outcome"] == "error"]
    assert [(item["pack"], item["code"]) for item in failed] == [("missing/v1", "REVISION_NOT_FOUND")]
    assert (tmp_path / "clean/robot/v1/pack.yaml").is_file()


def test_a_token_file_others_can_read_is_refused(home, tmp_path, published):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN).chmod(0o644)
    with pytest.raises(ValueError, match="chmod 600"):
        pack_sources.pull(tmp_path / "clean", "robot/v1", home=home, hub=hub)


def test_an_unknown_pack_names_the_sources_file(home, tmp_path):
    with pytest.raises(PackError) as caught:
        pack_sources.pull(tmp_path / "clean", "robot/v1", home=home, hub=NoHub())
    assert caught.value.code == "NO_SOURCE" and "pack-sources.toml" in str(caught.value)


def test_pull_refuses_an_asset_root_inside_the_checkout(home, published):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN)
    inside = Path(__file__).resolve().parents[2] / "build-test-assets"
    with pytest.raises(PackError) as caught:
        pack_sources.pull(inside, "robot/v1", home=home, hub=hub)
    assert caught.value.code == "INSIDE_REPOSITORY" and not inside.exists()


# ---------------------------------------------------------------------------
# CLI and tool
# ---------------------------------------------------------------------------


def test_cli_pull_and_push(home, tmp_path, published, monkeypatch, capsys):
    source, hub = published
    secret(home, "hf_token", READ_TOKEN)
    monkeypatch.setattr(pack_sources, "hub_module", lambda: hub)
    root = tmp_path / "clean"
    assert assets_cli.main(["--root", str(root), "pull", "--all"]) == 0
    out = capsys.readouterr().out
    assert "robot/v1: pulled" in out and "cad/v1: vendor download, not fetched" in out
    assert assets_cli.main(["--root", str(source), "push", "robot/v1", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["outcome"] == "already_there"
    with pytest.raises(SystemExit) as caught:
        assets_cli.main(["pull"])                # neither a pack nor --all
    assert caught.value.code == 2


def test_twinsmith_assets_runs_the_pack_commands_without_starting_the_server(home, tmp_path, monkeypatch, capsys):
    from twinsmith import server

    monkeypatch.setattr(server, "build_server", lambda **_: pytest.fail("the server was started"))
    (tmp_path / "a").mkdir()
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(tmp_path / "a"))
    with pytest.raises(SystemExit) as caught:
        server.main(["assets", "where", "--json"])
    assert caught.value.code == 0 and json.loads(capsys.readouterr().out)


def test_the_tool_pulls_and_reports_the_context(home, tmp_path, published, monkeypatch):
    _, hub = published
    secret(home, "hf_token", READ_TOKEN)
    monkeypatch.setattr(pack_sources, "hub_module", lambda: hub)
    root = tmp_path / "clean"
    root.mkdir()
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(root))
    report = asset_packs.run_tool("assets.pack_pull", {"pack": "robot/v1"}, broker=None)
    assert report["status"] == "ok" and report["asset_root"] == str(root)
    assert report["results"][0]["outcome"] == "pulled"
    dry = asset_packs.run_tool("assets.pack_push", {"pack": "robot/v1", "dry_run": True}, broker=None)
    assert dry["status"] == "ok" and dry["outcome"] == "dry_run" and dry["write_token"] == "found"


def test_hub_messages_are_scrubbed_of_tokens():
    exc = RuntimeError(f"boom with {READ_TOKEN} and hf_{'x' * 30}")
    error = pack_sources.hub_error(exc, REPO, "dataset", READ_TOKEN, Path("/k/hf_token"))
    assert READ_TOKEN not in str(error) and "hf_xxxx" not in str(error) and error.code == "HUB_ERROR"
