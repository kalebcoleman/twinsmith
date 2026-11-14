"""Adopt, convert, pin, doctor, closure builds and the host-side assets__pack_* tools, on the pack.yaml format."""

from __future__ import annotations

import hashlib
import json
import os
import stat
import sys
from pathlib import Path

import pytest

from twinsmith import asset_packs as packs
from twinsmith import pack_manifest as pm
from twinsmith.project_assets import Broker

REPO = Path(__file__).resolve().parents[2]
ENV = ("TWINSMITH_ASSET_ROOT", "TWINSMITH_WORKSPACE", "TWINSMITH_PROJECT", "TWINSMITH_USD_PYTHON")
PROVENANCE = dict(source="unit test", license="test fixture")


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def put(root: Path, rel: str, data="x") -> Path:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data if isinstance(data, bytes) else data.encode())
    return path


def make_pack(root: Path, pack_id: str, version: str, files: dict, **extra) -> Path:
    """A pack that verifies, written by hand (not by the code under test)."""
    directory = root / pack_id / version
    records = []
    for rel, data in sorted(files.items()):
        path = put(directory, rel, data)
        records.append(dict(path=rel, sha256=sha(path.read_bytes()), source="lab source", license="lab licence",
                            redistributable=False))
    (directory / "pack.yaml").write_text(json.dumps(dict(id=pack_id, version=version, **extra, files=records)))
    return directory


@pytest.fixture
def env(tmp_path, monkeypatch):
    for name in ENV:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "home" / ".cache"))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(tmp_path / "no-project"))
    root = tmp_path / "assets"
    root.mkdir()
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(root))
    return root


@pytest.fixture
def broker(tmp_path, env):
    root = tmp_path / "project"
    root.mkdir()
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    config = tmp_path / "approval.json"
    config.write_text(json.dumps(dict(version=1, profile="trusted_project", roots={"demo": str(root)}, staging=str(staging))))
    made = Broker(str(config))
    assert made.config_error is None
    return made


def fake_closure(graph):
    """A closure function from ``{path: (deps, unresolved)}``."""

    def call(paths):
        results = []
        for path in paths:
            deps, unresolved = graph.get(path, ([path], []))
            results.append(dict(path=path, layers=[path, *deps], assets=[], unresolved=list(unresolved),
                                referenced_by={}, error=None))
        return results

    return call


def verifies(root: Path, pack_id: str, version: str = "v1") -> pm.AssetPack:
    return pm.AssetPack(root, pack_id, version)  # the strict reader: raises if anything is off


# ---------------------------------------------------------------- pin: rewrite hashes of an existing manifest


def test_pin_has_nothing_to_do_for_a_version_that_verifies(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a"})
    before = (directory / "pack.yaml").read_bytes()
    report = packs.pin_pack(env, "p", "v1")
    assert report["pinned"] == 0 and report["verifies"] is True and "not modified" in report["note"]
    assert (directory / "pack.yaml").read_bytes() == before


def test_pin_refuses_a_changed_file_and_writes_nothing(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a"})
    (directory / "a.usd").write_bytes(b"damaged")
    before = (directory / "pack.yaml").read_bytes()
    with pytest.raises(packs.PackError) as caught:
        packs.pin_pack(env, "p", "v1")
    assert caught.value.code == "PIN_CHANGED"
    assert caught.value.details["changed"] == [dict(file="a.usd", pinned=sha(b"a"), actual=sha(b"damaged"))]
    assert (directory / "pack.yaml").read_bytes() == before


def test_pin_with_force_accepts_every_change_and_keeps_every_other_field(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a", "b.usd": b"b"}, qualification="candidate")
    manifest = json.loads((directory / "pack.yaml").read_text())
    manifest["files"][0]["fidelity"] = "flattened"
    (directory / "pack.yaml").write_text(json.dumps(manifest))
    (directory / "a.usd").write_bytes(b"a2")
    (directory / "b.usd").write_bytes(b"b2")
    report = packs.pin_pack(env, "p", "v1", force=True)
    assert report["pinned"] == 2 and report["verifies"] is True
    after = json.loads((directory / "pack.yaml").read_text())
    assert after["qualification"] == "candidate" and after["files"][0]["fidelity"] == "flattened"
    assert [f["sha256"] for f in after["files"]] == [sha(b"a2"), sha(b"b2")]
    verifies(env, "p")


def test_pin_drops_collision_counts_beside_a_changed_file(env):
    directory = make_pack(env, "p", "v1", {"bench/bench.usda": b"a", "bench/bench_collision.usdc": b"c", "other/o.usda": b"o"})
    manifest = json.loads((directory / "pack.yaml").read_text())
    for record in manifest["files"]:
        record["collision"] = dict(method="box", boxes=3)
    (directory / "pack.yaml").write_text(json.dumps(manifest))
    (directory / "bench" / "bench_collision.usdc").write_bytes(b"c2")
    report = packs.pin_pack(env, "p", "v1", files=["bench/bench_collision.usdc"])
    assert report["collision_dropped"] == ["bench/bench.usda", "bench/bench_collision.usdc"]
    after = {r["path"]: r for r in verifies(env, "p").manifest["files"]}
    assert "collision" not in after["bench/bench.usda"] and after["other/o.usda"]["collision"] == dict(method="box", boxes=3)


def test_pin_accepts_a_named_change_and_leaves_others_alone(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a", "b.usd": b"b"})
    (directory / "a.usd").write_bytes(b"a2")
    (directory / "b.usd").write_bytes(b"b2")
    with pytest.raises(packs.PackError, match="would change") as caught:
        packs.pin_pack(env, "p", "v1", files=["a.usd"])  # b is still changed and not named
    assert [c["file"] for c in caught.value.details["changed"]] == ["b.usd"]
    report = packs.pin_pack(env, "p", "v1", files=["a.usd"], partial=True)
    assert report["pinned"] == 1 and report["verifies"] is False
    hashes = {f["path"]: f["sha256"] for f in json.loads((directory / "pack.yaml").read_text())["files"]}
    assert hashes == {"a.usd": sha(b"a2"), "b.usd": sha(b"b")}
    with pytest.raises(packs.PackError, match="not changed files"):
        packs.pin_pack(env, "p", "v1", files=["nope.usd"])


def test_pin_cannot_fix_a_missing_or_unlisted_file(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a", "b.usd": b"b"})
    (directory / "a.usd").write_bytes(b"a2")
    (directory / "b.usd").unlink()
    with pytest.raises(packs.PackError) as caught:
        packs.pin_pack(env, "p", "v1", force=True)
    assert caught.value.code == "UNPINNABLE" and caught.value.details["missing"] == ["b.usd"]
    (directory / "b.usd").write_bytes(b"b")
    (directory / "new.usd").write_bytes(b"n")
    with pytest.raises(packs.PackError) as caught:
        packs.pin_pack(env, "p", "v1", force=True)
    assert caught.value.details["unlisted"] == ["new.usd"]
    assert json.loads((directory / "pack.yaml").read_text())["files"][0]["sha256"] == sha(b"a")  # nothing was written


def test_pin_dry_run_shows_the_change_and_writes_nothing(env):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a"})
    (directory / "a.usd").write_bytes(b"a2")
    before = (directory / "pack.yaml").read_bytes()
    report = packs.pin_pack(env, "p", "v1", dry_run=True)
    assert report["would_change"][0]["actual"] == sha(b"a2") and (directory / "pack.yaml").read_bytes() == before


# ---------------------------------------------------------------- classification


@pytest.mark.parametrize(
    "reference,kind",
    [
        ("OmniPBR.mdl", "runtime"),
        ("https://x/y.usd", "remote"),
        ("omniverse://x/y.usd", "remote"),
        ("https:/x/y.usd", "remote"),  # how USD reports it after normalizing
        ("file:/abs/gone.usd", "file"),
        ("robot/ur5e.usd", "logical"),
        ("/abs/missing.usd", "file"),
        ("materials/Local.mdl", "logical"),
    ],
)
def test_classify_reference(reference, kind):
    assert packs.classify_reference(reference) == kind


# ---------------------------------------------------------------- doctor (fake USD)


def doctor_setup(env):
    files = {"robot/ur5e.usd": b"root", "robot/SubUSDs/base.usd": b"base",
             **{f"robot/configuration/{n}.usd": b"cfg" + n.encode() for n in "abcd"}}
    folder = make_pack(env, "p", "v1", files)
    root = str(folder.resolve() / "robot/ur5e.usd")
    graph = {root: ([str(folder.resolve() / "robot/SubUSDs/base.usd")],
                    [str(folder.resolve() / "robot/SubUSDs/Robotiq.usd"), "OmniPBR.mdl"])}
    return folder, fake_closure(graph)


def test_doctor_reports_an_unresolved_reference_and_runtime_materials(env):
    folder, closure = doctor_setup(env)
    report = packs.doctor_pack(env, "p", "v1", closure=closure)
    assert [u["reference"] for u in report["dependencies"]["unresolved"]] == ["robot/SubUSDs/Robotiq.usd"]
    assert report["dependencies"]["runtime_materials"] == ["OmniPBR.mdl"]
    assert report["files"]["verified"] is True and report["ok"] is False
    assert len(report["problems"]) == 1 and "Robotiq" in report["problems"][0]


def test_every_listed_usd_is_a_root_and_a_clean_pack_is_ok(env):
    make_pack(env, "p", "v1", {"a.usd": b"a", "extra.usd": b"e"})
    report = packs.doctor_pack(env, "p", "v1", closure=fake_closure({}))
    assert report["ok"] is True and report["problems"] == [] and report["dependencies"]["entries"] == ["a.usd", "extra.usd"]


def test_doctor_reports_corrupt_missing_and_unlisted_files(env):
    folder, closure = doctor_setup(env)
    (folder / "robot/ur5e.usd").write_bytes(b"changed")
    (folder / "robot/configuration/a.usd").unlink()
    put(folder, "new.usd", b"new")
    report = packs.doctor_pack(env, "p", "v1", closure=closure)
    text = "\n".join(report["problems"])
    assert report["ok"] is False and report["files"]["verified"] is False
    assert "Checksum mismatch: robot/ur5e.usd" in text and "Missing pack file: robot/configuration/a.usd" in text
    assert "not listed in pack.yaml: new.usd" in text


def test_doctor_reports_files_outside_the_pack(env, tmp_path):
    folder = make_pack(env, "p", "v1", {"a.usd": b"a"})
    outside = str(tmp_path / "other.usd")
    closure = fake_closure({str(folder.resolve() / "a.usd"): ([outside], [])})
    report = packs.doctor_pack(env, "p", "v1", closure=closure)
    assert report["dependencies"]["outside_pack"][0]["file"] == outside and report["ok"] is False


def test_doctor_without_a_parser_says_so_and_is_not_ok(env, monkeypatch):
    doctor_setup(env)
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    report = packs.doctor_pack(env, "p", "v1", closure=packs.ClosureBackend())
    assert report["dependencies"]["checked"] is False and "No USD parser" in report["dependencies"]["reason"]
    assert report["ok"] is False and report["problems"] == [] and report["dependencies_checked"] is False


def test_a_pack_with_nothing_to_parse_needs_no_parser(env, monkeypatch):
    make_pack(env, "p", "v1", {"layout.json": b"{}", "tex.png": b"p"})
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    report = packs.doctor_pack(env, "p", "v1", closure=packs.ClosureBackend())
    assert report["ok"] is True and report["dependencies"]["checked"] is True


def test_a_urdf_only_pack_is_checked_without_a_usd_parser(env, monkeypatch):
    make_pack(env, "p", "v1", {"robot.urdf": '<robot name="r"><link name="l"><visual><geometry><mesh filename="m.stl"/></geometry></visual></link></robot>',
                               "m.stl": b"mesh"})
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    report = packs.doctor_pack(env, "p", "v1", closure=packs.ClosureBackend())
    assert report["ok"] is True, report["problems"]


def test_doctor_changes_nothing(env):
    folder, closure = doctor_setup(env)
    os.chmod(folder, 0o775)

    def state():
        return sorted((str(p), p.stat().st_mtime_ns, stat.S_IMODE(p.stat().st_mode)) for p in folder.rglob("*")) + [
            stat.S_IMODE(folder.stat().st_mode)]

    before = state()
    packs.doctor_pack(env, "p", "v1", closure=closure)
    assert state() == before


def test_doctor_of_an_absent_pack_is_an_error(env):
    with pytest.raises(packs.PackError) as caught:
        packs.doctor_pack(env, "nope", "v1")
    assert caught.value.code == "PACK_NOT_FOUND"


def test_doctor_names_a_symlink_that_import_would_refuse(env):
    folder = make_pack(env, "p", "v1", {"a.usda": "#usda 1.0\n"})
    (folder / "link.usda").symlink_to(folder / "a.usda")
    assert pm.symlinks(folder) == ["link.usda"]
    report = packs.doctor_pack(env, "p", "v1", closure=fake_closure({}))
    assert report["ok"] is False and any("Symlink in pack" in p and "link.usda" in p for p in report["problems"])


# ---------------------------------------------------------------- real USD

pxr = pytest.importorskip("pxr", reason="closure tests need usd-core (the scene-test extra)")
USD = packs.ClosureBackend(sys.executable)


def usda(*references):
    """A layer referencing each path. A path that contains ``@`` (a login such as ``user@domain``) is written between
    USD's triple delimiters, ``@@@...@@@``, as the Sdf API writes it; single ``@`` would end the path early."""
    def asset(ref):
        return f"@@@{ref}@@@" if "@" in ref else f"@{ref}@"

    body = "".join(f'def "R{i}" (\n  references = {asset(ref)}\n) {{}}\n' for i, ref in enumerate(references))
    return "#usda 1.0\n" + body


@pytest.fixture
def lab(tmp_path, env):
    """A source pack shaped like BrickSim's real need: a robot, a gripper, and files nothing refers to."""
    return make_pack(env, "lab", "v1", {
        "robot/ur5e.usda": usda("./SubUSDs/base.usda"), "robot/SubUSDs/base.usda": usda(),
        **{f"robot/configuration/{n}.usda": usda() for n in "abcd"},
        "gripper/QuickChanger.usda": usda("configuration/q.usda"), "gripper/configuration/q.usda": usda(),
        "objects/never_used.usda": usda(),
    })


def build(env, lab, sources, pack="bricksim", version="v1", **kwargs):
    """build_pack with ``lab/v1`` as the source pack; its records are inherited."""
    inherit = {lab.resolve(): {r["path"]: r for r in json.loads((lab / "pack.yaml").read_text())["files"]}}
    return packs.build_pack(env, pack, version, sources, USD, from_pack_dirs=[lab.resolve()], inherit=inherit,
                            defaults=kwargs.pop("defaults", dict(redistributable=False)), **kwargs)


def build_alone(env, sources, version="v1", pack="p", **kwargs):
    """build_pack from files outside any pack: source and license are then required."""
    kwargs.setdefault("defaults", dict(redistributable=False, **PROVENANCE))
    return packs.build_pack(env, pack, version, sources, USD, **kwargs)


def test_helper_reports_layers_and_unresolved(lab):
    robot = str((lab / "robot/ur5e.usda").resolve())
    result = USD([robot])[0]
    assert result["error"] is None
    assert sorted(Path(p).name for p in result["layers"]) == ["base.usda", "ur5e.usda"]


def test_helper_reports_an_unreadable_file(env):
    bad = put(env, "bad.usda", "this is not usd")
    assert USD([str(bad.resolve())])[0]["error"]


def test_helper_failure_is_a_clean_refusal(env):
    with pytest.raises(packs.PackError) as caught:
        packs.helper_closure(["x"], str(env / "no-such-python"))
    assert caught.value.code == "USD_FAILED"


def test_build_copies_only_the_closure_keeps_layout_and_writes_a_verifying_manifest(lab, env):
    report = build(env, lab, ["robot/ur5e.usda", "gripper/QuickChanger.usda"])
    dest = env / "bricksim" / "v1"
    assert sorted(p.relative_to(dest).as_posix() for p in dest.rglob("*") if p.is_file()) == sorted([
        "pack.yaml", "gripper/QuickChanger.usda", "gripper/configuration/q.usda", "robot/SubUSDs/base.usda", "robot/ur5e.usda"])
    assert report["copied"] == 4 and report["verified"]["self_contained"] is True and report["pack"] == "bricksim/v1"
    assert report["load_as"] == ["bricksim/v1/gripper/QuickChanger.usda", "bricksim/v1/robot/ur5e.usda"]  # what quick_start takes
    manifest = verifies(env, "bricksim").manifest
    assert {r["path"] for r in manifest["files"]} == {"gripper/QuickChanger.usda", "gripper/configuration/q.usda",
                                                      "robot/SubUSDs/base.usda", "robot/ur5e.usda"}
    assert all(r["source"] == "lab source" and r["license"] == "lab licence" and r["redistributable"] is False
               for r in manifest["files"])  # inherited from the source pack
    assert not [p for p in env.iterdir() if p.name.startswith(".staging")]


def test_a_built_file_keeps_its_own_record_and_an_outside_file_takes_the_defaults(lab, env, tmp_path):
    manifest = json.loads((lab / "pack.yaml").read_text())
    manifest["files"][0].update(asset_id="arm", redistributable=True, fidelity="flattened")
    (lab / "pack.yaml").write_text(json.dumps(manifest))
    first = manifest["files"][0]["path"]
    build(env, lab, [first])
    record = next(r for r in verifies(env, "bricksim").manifest["files"] if r["path"] == first)
    assert record["asset_id"] == "arm" and record["redistributable"] is True and record["fidelity"] == "flattened"
    scene = put(tmp_path / "project", "scene.usda", usda())
    build_alone(env, [str(scene)], pack="alone", layout_root=(tmp_path / "project").resolve())
    alone = verifies(env, "alone").manifest["files"][0]
    assert alone["source"] == "unit test" and alone["license"] == "test fixture" and alone["redistributable"] is False


def test_build_records_the_collision_counts_of_an_unchanged_split(env, tmp_path):
    from twinsmith import mesh_worker

    folder = tmp_path / "project" / "bench"
    top = put(folder, "bench.usda", "#usda 1.0\n(\n  subLayers = [@./bench_collision.usda@, @./bench_render.usda@]\n)\n")
    for name in ("bench_collision.usda", "bench_render.usda"):
        put(folder, name, usda())
    counts = dict(method="box", meshes=2, boxes=2, hulls=0, triangles=0, source_triangles=40)
    hashes = {p.name: sha(p.read_bytes()) for p in folder.iterdir()}
    put(folder, mesh_worker.REPORT, json.dumps(dict(sha256=hashes, collision={"for": "bench.usda", **counts})))
    build_alone(env, [str(top)], pack="split", layout_root=folder.resolve())
    records = {r["path"]: r for r in verifies(env, "split").manifest["files"]}
    assert records["bench.usda"]["collision"] == counts and "collision" not in records["bench_render.usda"]
    assert mesh_worker.REPORT not in records                    # the report is not part of the pack
    put(folder, "bench_collision.usda", usda() + "\n")          # edited after the split: the counts no longer hold
    build_alone(env, [str(top)], pack="split", version="v2", layout_root=folder.resolve())
    assert all("collision" not in r for r in verifies(env, "split", "v2").manifest["files"])


def test_a_collision_summary_is_used_only_for_its_own_unchanged_files(tmp_path):
    from twinsmith import mesh_worker

    def folder(name, report=None, **files):
        here = tmp_path / name
        for rel, data in (files or {"a.usda": "a", "a_collision.usdc": "c"}).items():
            put(here, rel, data)
        hashes = {p.name: sha(p.read_bytes()) for p in here.iterdir()}
        body = dict(sha256=hashes, collision={"for": "a.usda", "method": "box", "boxes": 2, "meshes": 2})
        put(here, mesh_worker.REPORT, json.dumps(report(body) if report else body))
        return here

    assert mesh_worker.collision_record(folder("ok") / "a.usda") == dict(method="box", boxes=2, meshes=2)
    assert mesh_worker.collision_record(folder("other") / "a_collision.usdc") is None          # describes a.usda only
    gone = folder("gone")
    (gone / "a_collision.usdc").unlink()
    assert mesh_worker.collision_record(gone / "a.usda") is None
    for name, edit in [("method", lambda r: r["collision"].update(method="mesh") or r),
                       ("count", lambda r: r["collision"].update(boxes=-1) or r),
                       ("type", lambda r: r["collision"].update(meshes="2") or r),
                       ("nested", lambda r: r["sha256"].update({"../a.usda": "0" * 64}) or r)]:
        assert mesh_worker.collision_record(folder(name, edit) / "a.usda") is None, name
    linked = folder("linked")
    (linked / "real.json").write_text((linked / mesh_worker.REPORT).read_text())
    (linked / mesh_worker.REPORT).unlink()
    (linked / mesh_worker.REPORT).symlink_to(linked / "real.json")
    assert mesh_worker.collision_record(linked / "a.usda") is None
    assert mesh_worker.collision_record(put(tmp_path / "bare", "a.usda", "a")) is None        # no report


def test_files_with_no_record_need_a_source_and_a_license(env, tmp_path):
    scene = put(tmp_path / "project", "scene.usda", usda())
    with pytest.raises(packs.PackError) as caught:
        packs.build_pack(env, "p", "v1", [str(scene)], USD, layout_root=(tmp_path / "project").resolve())
    assert caught.value.code == "MISSING_PROVENANCE" and not (env / "p").exists()


def test_a_scene_with_logical_references_is_read_and_not_copied(lab, env, tmp_path):
    scene = put(tmp_path / "project", "scene.usda", usda("robot/ur5e.usda", "gripper/QuickChanger.usda"))
    report = build(env, lab, [str(scene)])
    dest = env / "bricksim" / "v1"
    assert report["entries"] == ["gripper/QuickChanger.usda", "robot/ur5e.usda"]
    assert not (dest / "scene.usda").exists() and not (dest / "objects").exists()
    assert not (dest / "robot/configuration").exists()


def test_build_refuses_an_unresolved_reference_and_writes_nothing(lab, env):
    put(lab, "robot/ur5e.usda", usda("./SubUSDs/base.usda", "./SubUSDs/Robotiq.usda"))
    with pytest.raises(packs.PackError) as caught:
        packs.build_pack(env, "bricksim", "v1", ["robot/ur5e.usda"], USD, from_pack_dirs=[lab.resolve()],
                         defaults=dict(redistributable=False, **PROVENANCE))
    assert caught.value.code == "UNRESOLVED_DEPENDENCIES"
    item = caught.value.details["unresolved"][0]
    assert item["reference"].endswith("SubUSDs/Robotiq.usda") and item["referenced_by"].endswith("robot/ur5e.usda")
    assert not (env / "bricksim").exists()


def test_allow_unresolved_builds_and_doctor_names_what_was_recorded(lab, env):
    put(lab, "robot/ur5e.usda", usda("./SubUSDs/base.usda", "./SubUSDs/Robotiq.usda"))
    report = packs.build_pack(env, "bricksim", "v1", ["robot/ur5e.usda"], USD, from_pack_dirs=[lab.resolve()],
                              allow_unresolved=True, defaults=dict(redistributable=False, **PROVENANCE))
    assert report["copied"] == 2 and report["verified"]["accepted_unresolved"] == ["robot/SubUSDs/Robotiq.usda"]
    assert report["verified"]["self_contained"] is False
    assert report["importable"] is False and report["import_blockers"] == ["robot/SubUSDs/Robotiq.usda"]
    assert json.loads((env / "bricksim/v1/pack.yaml").read_text())["accepted_unresolved"] == ["robot/SubUSDs/Robotiq.usda"]
    doctor = packs.doctor_pack(env, "bricksim", "v1", closure=USD)
    assert doctor["ok"] is False
    assert any("accepted as unresolved" in p and "assets__import_path refuses" in p for p in doctor["problems"])
    # A version is never edited to add the file: the hash cannot fix it, and a new version can.
    put(env / "bricksim/v1", "robot/SubUSDs/Robotiq.usda", "#usda 1.0\n")
    with pytest.raises(packs.PackError) as caught:
        packs.pin_pack(env, "bricksim", "v1", force=True)
    assert caught.value.code == "UNPINNABLE"


def test_a_remote_reference_refuses_the_build(lab, env):
    put(lab, "robot/ur5e.usda", usda("https://example.invalid/x.usd"))
    with pytest.raises(packs.PackError) as caught:
        packs.build_pack(env, "b", "v1", ["robot/ur5e.usda"], USD, from_pack_dirs=[lab.resolve()],
                         defaults=dict(redistributable=False, **PROVENANCE))
    assert caught.value.details["remote"]


def test_a_dependency_outside_the_layout_is_never_copied(env, tmp_path):
    root = tmp_path / "project"
    put(tmp_path, "secret/other.usda", usda())
    scene = put(root, "a/scene.usda", usda("../../secret/other.usda"))
    with pytest.raises(packs.PackError) as caught:
        build_alone(env, [str(scene)], pack="b", layout_root=root.resolve())
    assert caught.value.details["outside"][0]["file"].endswith("secret/other.usda")
    report = build_alone(env, [str(scene)], pack="b", layout_root=root.resolve(), allow_unresolved=True)
    assert report["outside"] and not list((env / "b").rglob("other.usda"))
    assert report["entries"] == ["a/scene.usda"]


def test_relative_reference_up_the_tree_works_with_a_wide_layout_root(env, tmp_path):
    root = tmp_path / "project"
    put(root, "shared/other.usda", usda())
    scene = put(root, "a/scene.usda", usda("../shared/other.usda"))
    report = build_alone(env, [str(scene)], pack="b", layout_root=root.resolve())
    assert report["verified"]["self_contained"] is True and (env / "b/v1/shared/other.usda").is_file()


def test_the_check_after_copying_catches_an_absolute_reference(env, tmp_path):
    root = tmp_path / "project"
    other = put(root, "other.usda", usda())
    scene = put(root, "scene.usda", usda(str(other.resolve())))
    report = build_alone(env, [str(scene)], pack="b", layout_root=root.resolve())
    assert report["verified"]["self_contained"] is False
    assert report["verified"]["outside_pack"][0]["file"] == str(other.resolve())


def test_an_existing_version_is_never_modified_and_a_new_one_is_written(lab, env):
    build(env, lab, ["robot/ur5e.usda"])
    dest = env / "bricksim" / "v1"
    before = {p.relative_to(dest).as_posix(): p.read_bytes() for p in dest.rglob("*") if p.is_file()}
    put(lab, "robot/SubUSDs/base.usda", "#usda 1.0\n# lab edit\n")  # the source changes; the old version must not follow
    with pytest.raises(packs.PackError) as caught:
        build(env, lab, ["robot/ur5e.usda"])
    assert caught.value.code == "VERSION_EXISTS" and "v2" in str(caught.value)
    assert {p.relative_to(dest).as_posix(): p.read_bytes() for p in dest.rglob("*") if p.is_file()} == before
    with pytest.raises(packs.PackError):  # a dry run refuses too: it would be a lie
        build(env, lab, ["robot/ur5e.usda"], dry_run=True)
    # The source pack was edited by hand, so it no longer verifies; the tool refuses to build from it (see the tool tests).
    assert pm.inspect_pack(env, "lab", "v1")["verified"] is False


def test_dry_run_writes_nothing(lab, env):
    report = build(env, lab, ["robot/ur5e.usda"], dry_run=True)
    assert report["planned"] == ["robot/SubUSDs/base.usda", "robot/ur5e.usda"] and not (env / "bricksim").exists()


def test_a_failed_write_leaves_no_staging_folder_behind(lab, env, monkeypatch):
    def boom(self):
        raise packs.PackError("disk full", "FILESYSTEM")

    monkeypatch.setattr(packs.Staging, "verify", boom)
    with pytest.raises(packs.PackError, match="disk full"):
        build(env, lab, ["robot/ur5e.usda"])
    assert not (env / "bricksim" / "v1").exists() and not [p for p in env.iterdir() if p.name.startswith(".staging")]


def test_a_source_that_is_not_usd_or_not_there_is_refused(env, tmp_path):
    text = put(tmp_path, "a.txt")
    for source, code in ((str(text), "UNSUPPORTED_FORMAT"), (str(tmp_path / "gone.usda"), "NOT_FOUND"), ("rel.usda", "NOT_FOUND")):
        with pytest.raises(packs.PackError) as caught:
            build_alone(env, [source], pack="b")
        assert caught.value.code == code


def test_an_unreadable_source_refuses_even_with_allow_unresolved(env, tmp_path):
    bad = put(tmp_path, "bad.usda", "not usd")
    with pytest.raises(packs.PackError) as caught:
        build_alone(env, [str(bad)], pack="b", allow_unresolved=True)
    assert caught.value.code == "USD_UNREADABLE"


def test_doctor_finds_a_corrupt_sublayer(lab, env):
    build(env, lab, ["robot/ur5e.usda"])
    (env / "bricksim/v1/robot/SubUSDs/base.usda").write_text("broken")
    report = packs.doctor_pack(env, "bricksim", "v1", closure=USD)
    assert report["ok"] is False and "Checksum mismatch: robot/SubUSDs/base.usda" in report["problems"]
    assert report["dependencies"]["unreadable"] or report["dependencies"]["unresolved"]


def test_the_built_pack_composes(lab, env):
    from pxr import Usd

    build(env, lab, ["robot/ur5e.usda"])
    stage = Usd.Stage.Open(str(env / "bricksim/v1/robot/ur5e.usda"))
    assert stage.GetPrimAtPath("/R0") and not stage.GetPrimAtPath("/R0").GetPrimStack() == []


def test_a_file_kit_finds_in_its_own_install_is_runtime_not_outside(lab, env, tmp_path):
    """Seen live: Kit resolves OmniPBR.mdl to its library, standalone USD reports it unresolved."""
    from twinsmith.usd_closure import split_runtime

    install = tmp_path / "isaacsim"

    def kit_like(paths):
        results = USD(paths)
        for result in results:
            result["assets"].append(str(install / "kit/mdl/core/Base/OmniPBR.mdl"))
        return split_runtime(results, install)

    inherit = {lab.resolve(): {r["path"]: r for r in json.loads((lab / "pack.yaml").read_text())["files"]}}
    report = packs.build_pack(env, "bricksim", "v1", ["robot/ur5e.usda"], kit_like, from_pack_dirs=[lab.resolve()],
                              inherit=inherit, defaults=dict(redistributable=False))
    assert report["outside"] == [] and report["runtime_materials"] == ["OmniPBR.mdl"]
    assert report["verified"]["self_contained"] is True and report["verified"]["runtime_materials"] == ["OmniPBR.mdl"]


# ---------------------------------------------------------------- adopt and convert (library)


def test_adopt_writes_a_manifest_with_hashes_source_license_and_redistributable_false(env, tmp_path):
    source = tmp_path / "src"
    put(source, "robot/a.usd", b"a")
    put(source, "notes.txt", b"not an asset")
    report = packs.adopt_folder(env, "mine", "v1", source, source_note="vendor CAD", license_="private")
    assert report["files"] == 1 and report["pack"] == "mine/v1"
    record = verifies(env, "mine").manifest["files"][0]
    assert record == dict(path="robot/a.usd", sha256=sha(b"a"), source="vendor CAD", license="private", redistributable=False)
    assert not (env / "mine/v1/notes.txt").exists()


def test_adopt_can_say_redistributable_and_asks_for_provenance(env, tmp_path):
    source = tmp_path / "src"
    put(source, "a.usd")
    packs.adopt_folder(env, "mine", "v1", source, source_note="s", license_="MIT", redistributable=True)
    assert verifies(env, "mine").manifest["files"][0]["redistributable"] is True
    for bad in (dict(source_note=None, license_="x"), dict(source_note="x", license_=None), dict(source_note="", license_="x"),
                dict(source_note="two\nlines", license_="x"), dict(source_note="x" * 301, license_="x")):
        with pytest.raises(packs.PackError) as caught:
            packs.adopt_folder(env, "other", "v1", source, **bad)
        assert caught.value.code in ("MISSING_PROVENANCE", "INVALID_VALUE")
    with pytest.raises(packs.PackError, match="redistributable"):
        packs.adopt_folder(env, "other", "v1", source, source_note="s", license_="l", redistributable="yes")
    assert not (env / "other").exists()


def test_adopt_is_immutable_and_dry_run_refuses_too(env, tmp_path):
    source = tmp_path / "src"
    put(source, "a.usd", b"a")
    packs.adopt_folder(env, "mine", "v1", source, source_note="s", license_="l")
    before = (env / "mine/v1/pack.yaml").read_bytes()
    put(source, "b.usd", b"b")
    for dry in (False, True):
        with pytest.raises(packs.PackError) as caught:
            packs.adopt_folder(env, "mine", "v1", source, source_note="s", license_="l", dry_run=dry)
        assert caught.value.code == "VERSION_EXISTS"
    assert (env / "mine/v1/pack.yaml").read_bytes() == before and not (env / "mine/v1/b.usd").exists()
    packs.adopt_folder(env, "mine", "v2", source, source_note="s", license_="l")
    assert len(verifies(env, "mine", "v2").manifest["files"]) == 2


def test_convert_old_takes_hashes_from_the_files_not_from_sha256sums(env):
    old = env / "workcell"
    put(old, "robot/ur5e.usd", b"robot")
    put(old, "objects/fmb1.json", b"{}")
    put(old, "Isaac/Materials/m.mdl", b"mdl")
    (old / "SHA256SUMS").write_text(f"{'0' * 64}  robot/ur5e.usd\n")  # a wrong pin: ignored
    report = packs.convert_old(old, source_note="lab files", license_="private")
    assert report["pack"] == "workcell/v1" and report["files"] == 3 and report["converted_from"] == str(old)
    manifest = verifies(env, "workcell").manifest
    assert {r["path"]: r["sha256"] for r in manifest["files"]}["robot/ur5e.usd"] == sha(b"robot")
    assert sorted(r["path"] for r in manifest["files"]) == ["Isaac/Materials/m.mdl", "objects/fmb1.json", "robot/ur5e.usd"]
    assert (old / "SHA256SUMS").exists() and (old / "robot/ur5e.usd").exists()  # the old files are left in place
    assert any("old files are still in" in n for n in report["notes"])
    assert [p["id"] for p in pm.list_packs(env)["packs"]] == ["workcell"] and pm.list_packs(env)["old_format"] == []


def test_convert_old_refuses_a_second_time_a_symlink_and_a_wrong_id(env, tmp_path):
    old = env / "old"
    put(old, "a.usd", b"a")
    (old / "link.usd").symlink_to(old / "a.usd")
    report = packs.convert_old(old, source_note="s", license_="l", dry_run=True)
    assert report["planned"] == ["a.usd"] and any("link.usd" in n for n in report["notes"])
    packs.convert_old(old, source_note="s", license_="l")
    with pytest.raises(packs.PackError) as caught:
        packs.convert_old(old, source_note="s", license_="l")
    assert caught.value.code == "VERSION_EXISTS"
    with pytest.raises(packs.PackError, match="folder's name"):
        packs.convert_old(old, pack_id="other", source_note="s", license_="l")
    with pytest.raises(packs.PackError):
        packs.convert_old(env / "absent", source_note="s", license_="l")


# ---------------------------------------------------------------- the tools' trust rules


def tool(name, broker, kit_call=None, **params):
    return packs.run_tool(name, params, broker, kit_call=kit_call)


def refused(code, name, broker, **params):
    with pytest.raises(packs.PackError) as caught:
        tool(name, broker, **params)
    assert caught.value.code == code
    return caught.value


def test_packs_tool_reports_what_it_resolved(env, broker, capsys):
    make_pack(env, "p", "v1", {"a.usd": b"a"})
    result = tool("assets.packs", broker)
    assert result["asset_root"] == str(env.resolve()) and result["asset_root_source"] == "environment"
    assert result["selected_pack"] is None and result["listing"]["packs"][0]["id"] == "p"
    assert capsys.readouterr().out == ""  # stdout is the MCP protocol


def test_packs_tool_without_an_asset_root_says_which_setting_to_make(broker, monkeypatch, tmp_path):
    monkeypatch.delenv("TWINSMITH_ASSET_ROOT")
    monkeypatch.setenv("TWINSMITH_WORKSPACE", str(tmp_path / "empty-ws"))
    result = tool("assets.packs", broker)
    assert result["asset_root"] is None and "TWINSMITH_ASSET_ROOT" in result["asset_root_error"] and result["listing"]["packs"] == []
    refused("NO_ASSET_ROOT", "assets.pack_doctor", broker, pack="p/v1")


def adopt(broker, **params):
    return tool("assets.pack_adopt", broker, **{**PROVENANCE, **params})


def test_adopt_refuses_a_folder_outside_approved_roots(env, broker, tmp_path):
    outside = tmp_path / "outside"
    put(outside, "a.usd")
    refused("ROOT_NOT_APPROVED", "assets.pack_adopt", broker, pack="p/v1", source_path=str(outside), **PROVENANCE)
    assert not (env / "p").exists()


def test_adopt_does_not_treat_the_asset_root_as_approved(env, broker):
    put(env / "other", "a.usd")
    refused("ROOT_NOT_APPROVED", "assets.pack_adopt", broker, pack="p/v1", source_path=str(env / "other"), **PROVENANCE)


def test_a_symlink_inside_an_approved_root_cannot_reach_outside(env, broker, tmp_path):
    outside = tmp_path / "outside"
    put(outside, "a.usd")
    root = broker.roots["demo"][0]
    (root / "link").symlink_to(outside)
    refused("ROOT_NOT_APPROVED", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root / "link"), **PROVENANCE)


def test_a_config_error_refuses_writes_with_no_fallback(env, tmp_path):
    config = tmp_path / "bad.json"
    config.write_text("{not json")
    bad = Broker(str(config))
    root = put(tmp_path, "project2/a.usd").parent
    with pytest.raises(Exception, match="Configuration refused"):
        adopt(bad, pack="p/v1", source_path=str(root))


@pytest.mark.parametrize("path", ["relative/path", "/a/../b", "http://x/y", "/a//b"])
def test_adopt_refuses_bad_source_paths(env, broker, path):
    refused("INVALID_PATH", "assets.pack_adopt", broker, pack="p/v1", source_path=path, **PROVENANCE)


def test_adopt_copies_writes_the_manifest_and_never_modifies_a_version(env, broker, capsys):
    root = broker.roots["demo"][0]
    put(root, "robot/a.usd", b"a")
    put(root, "notes.txt", b"not an asset")
    first = adopt(broker, pack="p/v1", source_path=str(root), redistributable=True)
    assert first["files"] == 1 and first["pack"] == "p/v1"
    record = verifies(env, "p").manifest["files"][0]
    assert record["sha256"] == sha(b"a") and record["redistributable"] is True and not (env / "p/v1/notes.txt").exists()
    put(env / "p/v1", "robot/a.usd", b"lab edit")
    refused("VERSION_EXISTS", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), **PROVENANCE)
    assert (env / "p/v1/robot/a.usd").read_bytes() == b"lab edit"
    assert adopt(broker, pack="p/v2", source_path=str(root))["files"] == 1
    assert capsys.readouterr().out == ""


def test_adopt_dry_run_writes_nothing(env, broker):
    root = broker.roots["demo"][0]
    put(root, "a.usd")
    result = adopt(broker, pack="p/v1", source_path=str(root), dry_run=True)
    assert result["planned"] == ["a.usd"] and not (env / "p").exists()


def test_adopt_needs_exactly_one_of_a_source_folder_or_an_old_pack(env, broker):
    refused("INVALID_VALUE", "assets.pack_adopt", broker, pack="fresh/v1", **PROVENANCE)
    refused("INVALID_VALUE", "assets.pack_adopt", broker, pack="fresh/v1", source_path="/tmp", from_old="x", **PROVENANCE)
    assert not (env / "fresh").exists()


def test_adopt_requires_source_and_license(env, broker):
    root = broker.roots["demo"][0]
    put(root, "a.usd")
    refused("MISSING_PROVENANCE", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root))
    refused("MISSING_PROVENANCE", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), source="x")
    assert not (env / "p").exists()


def test_adopt_from_old_converts_a_pack_in_the_asset_root(env, broker):
    old = env / "workcell"
    put(old, "robot/ur5e.usd", b"r")
    (old / "SHA256SUMS").write_text("")
    listing = tool("assets.packs", broker)["listing"]
    assert [o["id"] for o in listing["old_format"]] == ["workcell"] and listing["packs"] == []
    result = adopt(broker, pack="workcell/v1", from_old="workcell")
    assert result["files"] == 1 and result["converted_from"] == str(old.resolve())
    verifies(env, "workcell")
    refused("INVALID_PACK", "assets.pack_adopt", broker, pack="other/v1", from_old="workcell", **PROVENANCE)  # the id is the folder
    refused("INVALID_VALUE", "assets.pack_adopt", broker, pack="workcell/v2", from_old="../x", **PROVENANCE)


def test_adopt_refuses_a_source_overlapping_the_pack(env, tmp_path, monkeypatch, broker):
    root = broker.roots["demo"][0]
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(root))
    put(root, "p/a.usd")
    refused("OVERLAP", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root / "p"), **PROVENANCE)
    refused("OVERLAP", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), **PROVENANCE)


@pytest.mark.parametrize("name", ["../escape/v1", "a/b/c", "..", "", "p", "p/", "/v1", "p/..", "bad name/v1", ".hidden/v1"])
def test_write_tools_refuse_invalid_pack_names(env, broker, name):
    refused("INVALID_PACK", "assets.pack_adopt", broker, pack=name, **PROVENANCE)
    refused("INVALID_PACK", "assets.pack_build", broker, pack=name, sources=["/x.usd"])


def test_write_tools_refuse_an_asset_root_inside_the_repository(env, broker, monkeypatch):
    root = broker.roots["demo"][0]
    put(root, "a.usd")
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(REPO / "tmp-never-created"))
    refused("INSIDE_REPOSITORY", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), **PROVENANCE)
    assert not (REPO / "tmp-never-created").exists()


def test_write_tools_refuse_a_pack_that_lands_inside_the_repository(env, broker, monkeypatch):
    root = broker.roots["demo"][0]
    put(root, "a.usd")
    monkeypatch.setenv("TWINSMITH_ASSET_ROOT", str(REPO.parent))
    refused("INSIDE_REPOSITORY", "assets.pack_adopt", broker, pack=f"{REPO.name}/v1", source_path=str(root), **PROVENANCE)


def test_write_tools_refuse_a_symlinked_pack_folder(env, broker, tmp_path):
    root = broker.roots["demo"][0]
    put(root, "a.usd")
    target = tmp_path / "elsewhere"
    (target / "v1").mkdir(parents=True)
    (env / "p").symlink_to(target)
    refused("UNSAFE_DESTINATION", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), **PROVENANCE)
    refused("UNSAFE_DESTINATION", "assets.pack_pin", broker, pack="p/v1")


def test_no_write_tool_accepts_a_destination_path():
    from twinsmith.scene_contracts import REGISTRY

    for name in ("assets.pack_adopt", "assets.pack_pin", "assets.pack_build"):
        assert not {"store", "root", "asset_root", "destination", "dest", "target"} & set(REGISTRY[name]["parameters"])
    assert "force" not in REGISTRY["assets.pack_adopt"]["parameters"] and "force" not in REGISTRY["assets.pack_build"]["parameters"]


def test_no_tool_can_approve_a_folder():
    from twinsmith.scene_contracts import REGISTRY

    assert not [name for name in REGISTRY if "approve" in name or "grant" in name]
    for name in REGISTRY:
        assert "trust_project" not in REGISTRY[name]["parameters"]


def test_pin_tool_needs_force_for_a_changed_file(env, broker):
    directory = make_pack(env, "p", "v1", {"a.usd": b"a"})
    assert tool("assets.pack_pin", broker, pack="p/v1")["pinned"] == 0
    (directory / "a.usd").write_bytes(b"b")
    assert refused("PIN_CHANGED", "assets.pack_pin", broker, pack="p/v1").details["changed"][0]["actual"] == sha(b"b")
    assert tool("assets.pack_pin", broker, pack="p/v1", force=True)["changed"][0]["actual"] == sha(b"b")
    verifies(env, "p")


def test_pin_tool_only_touches_the_asset_root(env, broker, tmp_path):
    other = tmp_path / "other-root"
    make_pack(other, "p", "v1", {"a.usd": b"a"})
    (other / "p/v1/a.usd").write_bytes(b"changed")
    refused("PACK_NOT_FOUND", "assets.pack_pin", broker, pack="p/v1")
    assert json.loads((other / "p/v1/pack.yaml").read_text())["files"][0]["sha256"] == sha(b"a")


def test_pin_and_doctor_default_to_the_project_files_pack(env, broker, monkeypatch, tmp_path):
    directory = make_pack(env, "chosen", "v1", {"a.usd": b"a"})
    (directory / "a.usd").write_bytes(b"b")
    project = tmp_path / "proj"
    project.mkdir()
    (project / "twinsmith.json").write_text(json.dumps({"pack": "chosen/v1"}))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(project))
    assert tool("assets.packs", broker)["selected_pack"] == "chosen/v1"
    assert tool("assets.pack_pin", broker, force=True)["pack_dir"].endswith("/chosen/v1")
    refused("INVALID_PACK", "assets.pack_doctor", broker, pack="no-version")


def test_dependencies_tool_checks_trust_before_parsing(env, broker, tmp_path, monkeypatch):
    calls = []
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", "/nonexistent/python")
    outside = put(tmp_path, "outside.usda")

    def kit(name, args):
        calls.append(name)
        return {}

    with pytest.raises(packs.PackError) as caught:
        packs.run_tool("assets.dependencies", dict(paths=[str(outside)]), broker, kit_call=kit)
    assert caught.value.code == "ROOT_NOT_APPROVED" and not calls
    refused("LIMIT", "assets.dependencies", broker, paths=[])


def test_dependencies_tool_on_the_asset_root_and_a_root(env, broker, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    root = broker.roots["demo"][0]
    a = put(env / "p" / "v1", "a.usda", usda("./missing.usda"))
    b = put(root, "b.usda", usda())
    result = tool("assets.dependencies", broker, paths=[str(a), str(b)])
    assert result["backend"] == "usd_helper"
    assert result["results"][0]["unresolved"][0].endswith("p/v1/missing.usda")
    assert result["results"][0]["referenced_by"]


def test_build_tool_refuses_sources_outside_trust(env, broker, tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    outside = put(tmp_path, "outside.usda", usda())
    refused("ROOT_NOT_APPROVED", "assets.pack_build", broker, pack="p/v1", sources=[str(outside)], **PROVENANCE)
    refused("ROOT_NOT_APPROVED", "assets.pack_build", broker, pack="p/v1", sources=[str(outside)], layout_root=str(tmp_path), **PROVENANCE)
    assert not (env / "p").exists()


def test_build_tool_does_not_copy_a_dependency_outside_trust(env, broker, tmp_path, monkeypatch):
    """A USD in an approved root must not be able to pull arbitrary files into a pack."""
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    root = broker.roots["demo"][0]
    secret = put(tmp_path, "private/secret.usda", usda())
    scene = put(root, "scene.usda", usda(str(secret.resolve())))
    error = refused("UNRESOLVED_DEPENDENCIES", "assets.pack_build", broker, pack="p/v1", sources=[str(scene)],
                    layout_root=str(root), **PROVENANCE)
    assert error.details["outside"][0]["file"] == str(secret.resolve())
    result = tool("assets.pack_build", broker, pack="p/v1", sources=[str(scene)], layout_root=str(root), allow_unresolved=True,
                  **PROVENANCE)
    assert result["status"] == "ok" and not list((env / "p").rglob("secret.usda"))


def test_build_tool_from_a_scene_in_a_root_and_a_pack_in_the_asset_root(env, broker, lab, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    root = broker.roots["demo"][0]
    scene = put(root, "scene.usda", usda("robot/ur5e.usda"))
    result = tool("assets.pack_build", broker, pack="bricksim/v1", sources=[str(scene)], from_pack="lab/v1")
    assert result["status"] == "ok" and result["backend"] == "usd_helper"
    assert result["verified"]["self_contained"] is True and (env / "bricksim/v1/robot/ur5e.usda").is_file()
    assert verifies(env, "bricksim").manifest["files"][0]["source"] == "lab source"
    doctor = tool("assets.pack_doctor", broker, pack="bricksim/v1")
    assert doctor["ok"] is True and doctor["dependencies"]["backend"] == "usd_helper"


def test_build_tool_refuses_to_copy_out_of_a_pack_that_does_not_verify(env, broker, lab, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    (lab / "robot/ur5e.usda").write_text("#usda 1.0\n# edited by hand\n")
    refused("PACK_NOT_VERIFIED", "assets.pack_build", broker, pack="b/v1", sources=["robot/ur5e.usda"], from_pack="lab/v1")
    assert not (env / "b").exists()


def test_a_project_file_with_an_asset_path_is_an_error_not_an_approval(env, broker, tmp_path, monkeypatch):
    claimed = tmp_path / "claimed"
    put(claimed, "scene.usda", usda())
    repo = tmp_path / "cloned-repo"
    repo.mkdir()
    (repo / "twinsmith.json").write_text(json.dumps(dict(asset_path=[str(claimed)])))
    monkeypatch.setenv("TWINSMITH_PROJECT", str(repo))
    with pytest.raises(packs.PackError, match="not supported any more") as caught:
        tool("assets.pack_build", broker, pack="p/v1", sources=[str(claimed / "scene.usda")])
    assert caught.value.code == "INVALID_PACK" and not (env / "p").exists()


def test_an_approved_folder_works(env, tmp_path, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    claimed = tmp_path / "claimed"
    scene = put(claimed, "scene.usda", usda())
    config = config_for(tmp_path, {"c": str(claimed)}, "approval2.json")
    result = tool("assets.pack_build", Broker(config), pack="p/v1", sources=[str(scene)], layout_root=str(claimed), **PROVENANCE)
    assert result["status"] == "ok" and (env / "p/v1/scene.usda").is_file()


def test_build_tool_refuses_an_absent_or_overlapping_source_pack(env, broker, lab, monkeypatch):
    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    refused("PACK_NOT_VERIFIED", "assets.pack_build", broker, pack="p/v1", sources=["a.usda"], from_pack="absent/v1")
    refused("VERSION_EXISTS", "assets.pack_build", broker, pack="lab/v1", sources=["robot/ur5e.usda"], from_pack="lab/v1")


def test_build_without_any_parser_says_how_to_get_one(env, broker, lab, monkeypatch):
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    error = refused("USD_UNAVAILABLE", "assets.pack_build", broker, pack="p/v1", sources=["robot/ur5e.usda"], from_pack="lab/v1")
    assert "TWINSMITH_USD_PYTHON" in str(error)


def test_without_a_host_parser_the_kit_computes_the_closure(env, broker, lab, monkeypatch):
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    seen = []

    def kit(name, args):
        seen.append(name)
        return dict(results=USD(args["paths"]))

    result = tool("assets.pack_build", broker, kit_call=kit, pack="b/v1", sources=["robot/ur5e.usda"], from_pack="lab/v1")
    assert result["backend"] == "kit" and set(seen) == {"assets.dependencies"} and (env / "b/v1/robot/ur5e.usda").is_file()


def test_doctor_tool_with_no_parser_and_a_dead_bridge_says_it_did_not_check(env, broker, monkeypatch):
    monkeypatch.setattr(packs, "usd_python", lambda: None)
    make_pack(env, "p", "v1", {"a.usda": usda()})

    def dead(name, args):
        raise OSError("connection refused")

    result = tool("assets.pack_doctor", broker, kit_call=dead, pack="p/v1")
    assert result["status"] == "ok" and result["dependencies_checked"] is False and result["ok"] is False
    assert "connection refused" in result["dependencies"]["reason"]


def test_invalid_project_pack_reaches_the_caller_as_an_error(env, tmp_path, monkeypatch):
    from twinsmith import server as gw

    folder = tmp_path / "proj"
    folder.mkdir()
    (folder / "twinsmith.json").write_text('{"pack": "../x"}')
    monkeypatch.setenv("TWINSMITH_PROJECT", str(folder))
    result = json.loads(gw._dispatch("assets.packs", {}))
    assert result["status"] == "error" and result["code"] == "INVALID_PACK"


def test_host_tools_run_through_the_gateway_without_kit(env, monkeypatch):
    from twinsmith import server as gw

    directory = make_pack(env, "p", "v1", {"a.usd": b"a"})
    (directory / "a.usd").write_bytes(b"b")
    state = gw._BridgeState()
    state.asset_broker = Broker("")
    listing = json.loads(gw._dispatch("assets.packs", {}, state))["listing"]
    assert [(p["id"], p["version"], p["verified"]) for p in listing["packs"]] == [("p", "v1", False)]
    result = json.loads(gw._dispatch("assets.pack_pin", dict(pack="p/v1", force=True), state))
    assert result["status"] == "ok" and result["pinned"] == 1
    refusal = json.loads(gw._dispatch("assets.pack_adopt", dict(pack="q/v1", source_path="/tmp", **PROVENANCE), state))
    assert refusal["code"] == "ROOT_NOT_APPROVED"
    missing = json.loads(gw._dispatch("assets.pack_doctor", dict(pack="nope/v1"), state))
    assert missing["status"] == "error" and missing["code"] == "PACK_NOT_FOUND"


def test_tool_annotations_match_what_each_tool_does(monkeypatch):
    from twinsmith import server as gw

    tools = gw.build_server({"TWINSMITH_TOOLSETS": "default,+assets"})._tool_manager._tools
    for name in ("assets__packs", "assets__pack_doctor", "assets__dependencies"):
        assert tools[name].annotations.readOnlyHint is True and tools[name].annotations.destructiveHint is False
    for name in ("assets__pack_adopt", "assets__pack_pin", "assets__pack_build"):
        assert tools[name].annotations.readOnlyHint is False and tools[name].annotations.destructiveHint is True


def test_only_the_dependency_tool_is_served_by_kit():
    from twinsmith import server as gw
    from twinsmith.scene_contracts import HOST_ONLY, REGISTRY

    source = (REPO / "exts/sim-tools/sim_tools/extension.py").read_text()
    assert "pack_manifest.resolve_asset_root()" in source and "assets.user_stores()" not in source
    assert "canonical not in HOST_ONLY" in source and 'name == "assets.dependencies"' in source
    assert gw.HOST_RUN_TOOLS == HOST_ONLY and HOST_ONLY <= set(REGISTRY)
    assert "assets.dependencies" not in HOST_ONLY


# ---------------------------------------------------------------- through a real MCP session


def mcp_call(server, name, args):
    import asyncio

    from mcp.shared.memory import create_connected_server_and_client_session

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            await client.initialize()
            result = await client.call_tool(name, args)
            return result.isError, "\n".join(x.text for x in result.content if x.type == "text")

    return asyncio.run(run())


def test_every_pack_tool_publishes_string_arrays(monkeypatch):
    """A gateway default types arrays as numbers, which refused every path before any code ran."""
    from twinsmith import server as gw
    from twinsmith.scene_contracts import REGISTRY

    tools = gw.build_server({"TWINSMITH_TOOLSETS": "default,+assets"})._tool_manager._tools
    checked = []
    for canonical in gw._PACK_TOOLS:
        schema = tools[canonical.replace(".", "__")].parameters["properties"]
        for name, spec in REGISTRY[canonical]["parameters"].items():
            if spec["type"] == "array":
                variants = schema[name].get("anyOf", [schema[name]])
                array = next(v for v in variants if v.get("type") == "array")
                assert array["items"]["type"] == "string", (canonical, name)
                checked.append(name)
    assert sorted(set(checked)) == ["files", "include", "paths", "sources"]


def test_pack_tools_take_their_arguments_through_a_real_mcp_session(env, tmp_path, monkeypatch):
    from twinsmith import server as gw

    monkeypatch.setenv("TWINSMITH_USD_PYTHON", sys.executable)
    root = tmp_path / "project"
    put(root, "robot/a.usda", usda("./b.usda"))
    put(root, "robot/b.usda", usda())
    put(root, "extra/c.usda", usda())
    put(root, "extra/notes.xyz", "x")
    staging = tmp_path / "staging"
    staging.mkdir(mode=0o700)
    config = tmp_path / "approval.json"
    config.write_text(json.dumps(dict(version=1, profile="trusted_project", roots={"demo": str(root)}, staging=str(staging))))
    server = gw.build_server({"TWINSMITH_ASSET_CONFIG": str(config), "TWINSMITH_TOOLSETS": "default,+assets"})

    def call(name, **args):
        failed, text = mcp_call(server, name, args)
        assert not failed, text
        return json.loads(text)

    built = call("assets__pack_build", pack="p/v1", sources=[str(root / "robot/a.usda")], layout_root=str(root), **PROVENANCE)
    assert built["status"] == "ok" and built["verified"]["self_contained"] is True
    assert call("assets__pack_doctor", pack="p/v1")["ok"] is True
    assert call("assets__dependencies", paths=[str(root / "robot/a.usda")])["results"][0]["error"] is None
    adopted = call("assets__pack_adopt", pack="q/v1", source_path=str(root), include=["xyz"], **PROVENANCE)
    assert adopted["status"] == "ok" and (env / "q/v1/extra/notes.xyz").is_file()
    (env / "q/v1/extra/c.usda").write_text("#usda 1.0\n# edit\n")
    failed, text = mcp_call(server, "assets__pack_pin", dict(pack="q/v1"))
    assert failed and "PIN_CHANGED" in text  # a refusal reaches the client as a tool error
    assert call("assets__pack_pin", pack="q/v1", files=["extra/c.usda"])["status"] == "ok"
    assert call("assets__packs")["status"] == "ok"
    failed, text = mcp_call(server, "assets__pack_pin", dict(pack="q/v1", store="/tmp"))
    assert failed or "store" in text


# ---------------------------------------------------------------- material references (MaterialX, MDL)


def mtlx(*textures, extra=""):
    body = "".join(f'<image name="i{n}" type="color3"><input name="file" type="filename" value="{t}"/></image>'
                   for n, t in enumerate(textures))
    return f'<?xml version="1.0"?><materialx version="1.38" {extra}>{body}</materialx>'


def textured(root, *textures, extra=""):
    put(root, "scene.usda", usda("./materials/m.mtlx"))
    put(root, "materials/m.mtlx", mtlx(*textures, extra=extra))
    put(root, "materials/tex/unused.jpg", b"not referenced")
    for texture in textures:
        if "{" not in texture:
            put(root, f"materials/{texture}", b"pixels")


def test_build_copies_materialx_textures_and_only_those(env, tmp_path):
    root = tmp_path / "src"
    textured(root, "tex/a.jpg", "tex/b.jpg")
    report = build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    dest = env / "p/v1"
    assert (dest / "materials/tex/a.jpg").is_file() and (dest / "materials/tex/b.jpg").is_file()
    assert not (dest / "materials/tex/unused.jpg").exists()
    assert report["verified"]["self_contained"] is True and report["material_references"] == 2
    assert {"materials/tex/a.jpg", "materials/tex/b.jpg"} <= {r["path"] for r in verifies(env, "p").manifest["files"]}


def test_a_missing_texture_refuses_the_build(env, tmp_path):
    root = tmp_path / "src"
    textured(root, "tex/a.jpg")
    (root / "materials/tex/a.jpg").unlink()
    with pytest.raises(packs.PackError) as caught:
        build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    assert caught.value.code == "UNRESOLVED_DEPENDENCIES" and "tex/a.jpg" in str(caught.value)
    assert not (env / "p").exists()


def test_a_texture_missing_from_a_pack_fails_doctor(env, tmp_path):
    root = tmp_path / "src"
    textured(root, "tex/a.jpg")
    build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    assert packs.doctor_pack(env, "p", "v1", closure=USD)["ok"] is True
    (env / "p/v1/materials/tex/a.jpg").unlink()
    report = packs.doctor_pack(env, "p", "v1", closure=USD)
    assert report["ok"] is False and report["dependencies"]["self_contained"] is False
    assert report["dependencies"]["unresolved"][0]["reference"] == "materials/tex/a.jpg"
    assert "Missing pack file: materials/tex/a.jpg" in report["problems"]


def test_a_texture_outside_the_layout_is_not_copied(env, tmp_path):
    root = tmp_path / "src"
    put(tmp_path, "secret/t.jpg", b"private")
    textured(root, "../../secret/t.jpg")
    with pytest.raises(packs.PackError) as caught:
        build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    assert caught.value.details["outside"][0]["file"].endswith("secret/t.jpg")


def test_nested_materialx_is_followed(env, tmp_path):
    root = tmp_path / "src"
    put(root, "scene.usda", usda("./a.mtlx"))
    put(root, "a.mtlx", '<materialx version="1.38"><xi:include xmlns:xi="x" href="b.mtlx"/></materialx>')
    put(root, "b.mtlx", mtlx("t.jpg"))
    put(root, "t.jpg", b"p")
    report = build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    assert (env / "p/v1/t.jpg").is_file() and report["verified"]["self_contained"] is True


@pytest.mark.parametrize("texture,extra", [("tex/{UDIM}.jpg", ""), ("tex/a.jpg", 'fileprefix="tex/"')])
def test_a_reference_that_cannot_be_followed_is_never_self_contained(env, tmp_path, texture, extra):
    root = tmp_path / "src"
    textured(root, texture, extra=extra)
    notes = []
    report = build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve(), notes=notes)
    assert report["unfollowable"] and report["verified"]["self_contained"] is False
    assert any("self_contained is false" in n for n in notes)
    doctor = packs.doctor_pack(env, "p", "v1", closure=USD)
    assert doctor["dependencies"]["unfollowable"] and doctor["dependencies"]["self_contained"] is False
    assert doctor["warnings"]


def test_mdl_literal_textures_are_followed_and_custom_modules_are_named(env, tmp_path):
    root = tmp_path / "src"
    put(root, "scene.usda", '#usda 1.0\ndef Shader "S" {\n uniform asset info:mdl:sourceAsset = @./m.mdl@\n}\n')
    put(root, "m.mdl", 'mdl 1.6;\nimport ::tex::*;\ntexture_2d t = texture_2d("tex/a.png");\n')
    put(root, "tex/a.png", b"p")
    report = build_alone(env, [str(root / "scene.usda")], layout_root=root.resolve())
    assert (env / "p/v1/tex/a.png").is_file() and report["verified"]["self_contained"] is True
    put(root, "m.mdl", 'mdl 1.6;\nimport ::nvidia::core_definitions::*;\n')
    report = build_alone(env, [str(root / "scene.usda")], pack="q", layout_root=root.resolve())
    assert "custom MDL module" in report["unfollowable"][0]["reason"]
    assert report["verified"]["self_contained"] is False


def test_the_shared_parser_matches_what_the_import_validator_uses():
    source = (REPO / "exts/sim-tools/sim_tools/core.py").read_text()
    assert "material_refs.scan_mdl(" in source and "material_refs.scan_materialx(" in source
    assert "ET.fromstring" not in source  # no second parser


# ---------------------------------------------------------------- adopt: named files


def test_adopt_can_copy_only_named_files_and_folders(env, broker):
    root = broker.roots["demo"][0]
    put(root, "a.usda", usda())
    put(root, "tex/t.jpg", b"t")
    put(root, "tex/deep/u.png", b"u")
    put(root, "cards/preview.png", b"no")
    put(root, "notes.xyz", b"named")
    result = adopt(broker, pack="p/v1", source_path=str(root), files=["a.usda", "tex", "notes.xyz"])
    assert result["files"] == 4 and not (env / "p/v1/cards").exists()
    assert {r["path"] for r in verifies(env, "p").manifest["files"]} == {"a.usda", "tex/t.jpg", "tex/deep/u.png", "notes.xyz"}


@pytest.mark.parametrize("name,code", [("../up.usda", "REFUSED"), ("/etc/passwd", "REFUSED"), ("gone.usda", "NOT_FOUND")])
def test_adopt_refuses_a_named_file_outside_the_source(env, broker, name, code):
    root = broker.roots["demo"][0]
    put(root, "a.usda", usda())
    refused(code, "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), files=[name], **PROVENANCE)
    assert not (env / "p").exists()


def test_adopt_refuses_a_named_symlink(env, broker, tmp_path):
    root = broker.roots["demo"][0]
    secret = put(tmp_path, "secret/s.jpg", b"s")
    (root / "link.jpg").symlink_to(secret)
    (root / "dir").symlink_to(secret.parent)
    refused("UNSAFE_ENTRY", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), files=["link.jpg"], **PROVENANCE)
    refused("UNSAFE_ENTRY", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), files=["dir/s.jpg"], **PROVENANCE)


def test_adopt_with_no_asset_files_says_so_and_writes_nothing(env, broker):
    root = broker.roots["demo"][0]
    put(root, "notes.txt")
    refused("NOT_FOUND", "assets.pack_adopt", broker, pack="p/v1", source_path=str(root), **PROVENANCE)
    assert not (env / "p").exists()


# ---------------------------------------------------------------- approval and packs in the asset root


def config_for(tmp_path, roots, name="approval3.json"):
    staging = tmp_path / "staging3"
    staging.mkdir(mode=0o700, exist_ok=True)
    config = tmp_path / name
    config.write_text(json.dumps(dict(version=1, profile="trusted_project", roots=roots, staging=str(staging))))
    return str(config)


def test_a_missing_root_is_reported_and_the_others_keep_working(tmp_path):
    good = tmp_path / "good"
    good.mkdir()
    made = Broker(config_for(tmp_path, {"good": str(good), "gone": str(tmp_path / "gone")}))
    assert made.config_error is None and set(made.roots) == {"good"}
    listing = made.listing()
    assert listing["missing_roots"] == [dict(root_id="gone", path=str(tmp_path / "gone"))]
    with pytest.raises(Exception, match="not approved"):
        made.root_fd("gone")


def test_a_root_that_returns_is_not_trusted_before_a_restart(tmp_path):
    gone = tmp_path / "gone"
    made = Broker(config_for(tmp_path, {"gone": str(gone)}))
    gone.mkdir()
    with pytest.raises(Exception, match="not approved"):
        made.root_fd("gone")


def test_a_symlinked_root_still_refuses_everything(tmp_path):
    good = tmp_path / "good"
    good.mkdir()
    (tmp_path / "link").symlink_to(good)
    made = Broker(config_for(tmp_path, {"other": str(tmp_path / "staging3" / "x"), "link": str(tmp_path / "link")}))
    assert made.config_error and made.roots == {} and made.missing_roots == {}


def test_overlapping_roots_still_refuse_even_when_one_is_missing(tmp_path):
    good = tmp_path / "good"
    good.mkdir()
    made = Broker(config_for(tmp_path, {"parent": str(good), "child": str(good / "absent")}))
    assert "overlap" in made.config_error and made.roots == {}


def test_the_helper_adds_a_root_to_an_existing_file(tmp_path, monkeypatch, capsys):
    from twinsmith import project_assets as pa

    one, two = tmp_path / "one", tmp_path / "two"
    one.mkdir()
    two.mkdir()
    config = tmp_path / "a.json"
    staging = tmp_path / "s"
    base = ["--config", str(config), "--trust-project"]
    pa.main(base + ["--root-id", "one", "--root", str(one), "--staging", str(staging)])
    pa.main(base + ["--root-id", "two", "--root", str(two)])
    assert set(Broker(str(config)).roots) == {"one", "two"}
    with pytest.raises(SystemExit):
        pa.main(base + ["--root-id", "one", "--root", str(two)])  # may not repoint
    with pytest.raises(SystemExit):
        pa.main(base + ["--root-id", "x", "--root", str(two), "--staging", str(tmp_path / "other")])
    with pytest.raises(SystemExit):
        pa.main(base + ["--root-id", "nested", "--root", str(one / "sub")])  # overlap: old file untouched
    assert set(Broker(str(config)).roots) == {"one", "two"} and not list(tmp_path.glob(".a.json.*"))
    with pytest.raises(SystemExit):
        pa.main(["--config", str(config), "--root-id", "z", "--root", str(two)])  # still needs --trust-project


def test_the_helper_can_set_up_staging_alone(tmp_path):
    from twinsmith import project_assets as pa

    config = tmp_path / "s.json"
    pa.main(["--config", str(config), "--trust-project", "--staging", str(tmp_path / "st")])
    made = Broker(str(config))
    assert made.config_error is None and made.staging == tmp_path / "st" and made.roots == {}


def test_a_pack_in_the_asset_root_can_be_located_and_snapshotted(env, tmp_path):
    made = Broker(config_for(tmp_path, {}))
    make_pack(env, "chess", "v1", {"scene.usda": usda()})
    root_id, rel = made.locate(str(env.resolve() / "chess/v1/scene.usda"))
    assert rel == "scene.usda" and root_id.startswith("pack:")
    snapshot = made.snapshot(root_id, rel)
    assert made.verify(snapshot["snapshot_id"], snapshot["manifest_sha256"])[1]["entry"] == "scene.usda"


def test_a_pack_that_does_not_verify_is_refused(env, tmp_path):
    made = Broker(config_for(tmp_path, {}))
    directory = make_pack(env, "chess", "v1", {"scene.usda": usda(), "other.usda": usda()})
    (directory / "scene.usda").write_text("#usda 1.0\n# changed\n")
    (directory / "other.usda").unlink()
    with pytest.raises(Exception, match="does not verify") as caught:
        made.locate(str(env.resolve() / "chess/v1/scene.usda"))
    assert caught.value.code == "PACK_CORRUPT"
    assert "Checksum mismatch: scene.usda" in str(caught.value) and "Missing pack file: other.usda" in str(caught.value)


def test_a_file_that_is_not_in_a_pack_version_is_still_not_approval(env, tmp_path, monkeypatch):
    made = Broker(config_for(tmp_path, {}))
    flat = put(env, "flat.usda", usda())
    two_deep = put(env, "chess/scene.usda", usda())  # a version is required: <id>/<version>/<file>
    outside = put(tmp_path, "elsewhere/scene.usda", usda())
    for path in (flat, two_deep, outside):
        with pytest.raises(Exception, match="outside all user-approved") as caught:
            made.locate(str(path))
        assert caught.value.code == "ROOT_NOT_APPROVED"


def test_a_symlink_in_a_pack_is_refused(env, tmp_path):
    made = Broker(config_for(tmp_path, {}))
    directory = make_pack(env, "chess", "v1", {"scene.usda": usda()})
    (directory / "evil.png").symlink_to(put(tmp_path, "secret.png", b"s"))
    with pytest.raises(Exception, match="[Ss]ymlink"):
        made.locate(str(env.resolve() / "chess/v1/scene.usda"))


# ---------------------------------------------------------------- refusals that say what to do


def test_a_units_mismatch_says_what_to_change():
    from twinsmith import project_assets as pa

    message = pa.units_mismatch_message("Y", 0.01, "Z", 1.0)
    assert "upAxis=Y" in message and "upAxis=Z" in message and "metersPerUnit=0.01" in message
    assert 'sim__quick_start with up_axis="Y" and meters_per_unit=0.01' in message and "must match" in message
    assert "run_python" not in message


def test_approval_helper_refuses_a_second_root_rather_than_dropping_the_first(tmp_path, capsys):
    from twinsmith import project_assets as _assets

    with pytest.raises(SystemExit):
        _assets.main(["--config", str(tmp_path / "c.json"), "--trust-project", "--root-id", "a", "--root", str(tmp_path),
                      "--root-id", "b", "--root", str(tmp_path)])
    assert "can be given once" in capsys.readouterr().err
    assert not (tmp_path / "c.json").exists()


def test_adopt_reports_a_symlink_it_did_not_copy(env, broker):
    root = broker.roots["demo"][0]
    put(root, "real/a.usda", "#usda 1.0\n")
    put(root, "solo.usda", "#usda 1.0\n")
    (root / "link").symlink_to(root / "real", target_is_directory=True)
    (root / "alias.usda").symlink_to(root / "solo.usda")

    out = adopt(broker, pack="linked/v1", source_path=str(root))
    assert out["status"] == "ok"
    assert out["skipped_symlinks"] == ["alias.usda", "link/"]
    assert any("symbolic link" in note and "alias.usda" in note for note in out["notes"])
    assert (env / "linked/v1/real/a.usda").is_file()
    assert (env / "linked/v1/solo.usda").is_file()
    assert not (env / "linked/v1/link").exists()
    assert not (env / "linked/v1/alias.usda").exists()

    dry = adopt(broker, pack="linked2/v1", source_path=str(root), dry_run=True)
    assert dry["skipped_symlinks"] == ["alias.usda", "link/"]
    assert not (env / "linked2").exists()
