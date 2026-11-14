"""hardware action=part and the SolidCraft line of status, with a fake solidcraft-cli.

The fake logs its argv and writes what the real one would (the design's parameters for `parameters.list`, STEP and STL
files, a measurement and a preview). No SolidCraft is needed. The STL it writes is a real 10 x 20 x 5 mm box, so the
part.usda step runs for real when the mesh extra is installed and is skipped with a warning when it is not.
"""

import json
import sys
from pathlib import Path

import pytest

from twinsmith import hardware

FAKE = """#!{python}
import json, os, pathlib, sys
args = sys.argv[1:]
with open({log!r}, "a") as log:
    log.write(json.dumps({{"argv": args, "env": sorted(os.environ)}}) + "\\n")
def value(flag):
    return [args[i + 1] for i, a in enumerate(args) if a == flag]
if args == ["--version"]:
    print("solidcraft-cli 0.42.0"); sys.exit(0)
if os.path.exists({fail!r}) and args[0] == "run" and "--results" not in args:
    print("feature Hole failed", file=sys.stderr); sys.exit(3)
if args[0] == "run" and "--results" in args:
    pathlib.Path(value("--json-out")[0]).write_text(json.dumps([{{"count": 2, "parameters": [
        {{"name": "L", "value": 10.0, "unit": "mm", "expression": "10", "source": "user"}},
        {{"name": "W", "value": 20.0, "unit": "mm", "expression": "20", "source": "user"}}]}}]))
elif args[0] == "run":
    for out in value("--out"):
        if out.endswith(".stl"):
            pathlib.Path(out).write_bytes(pathlib.Path({stl!r}).read_bytes())
        else:
            pathlib.Path(out).write_text("ISO-10303-21;")
elif args[0] == "eval":
    pathlib.Path(value("--json-out")[0]).write_text(json.dumps({{"bodies": [{{"name": "bracket", "volume_mm3": 1000.0,
        "bbox": {{"min": [0, 0, 0], "max": [10, 20, 5]}}, "closed": True}}]}}))
elif args[0] == "snapshot":
    pathlib.Path(value("--out")[0]).write_bytes(b"\\x89PNG")
"""


def _box_stl(path: Path) -> None:
    """A closed 10 x 20 x 5 mm box as ASCII STL, written without trimesh."""
    x, y, z = 10.0, 20.0, 5.0
    v = [(0, 0, 0), (x, 0, 0), (x, y, 0), (0, y, 0), (0, 0, z), (x, 0, z), (x, y, z), (0, y, z)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    lines = ["solid box"]
    for a, b, c in faces:
        lines += ["facet normal 0 0 0", "outer loop", *(f"vertex {' '.join(map(str, v[i]))}" for i in (a, b, c)),
                  "endloop", "endfacet"]
    path.write_text("\n".join(lines + ["endsolid box"]) + "\n")


@pytest.fixture
def cell(tmp_path, monkeypatch):
    project, bin_dir, log, stl = tmp_path / "project", tmp_path / "bin", tmp_path / "calls.jsonl", tmp_path / "box.stl"
    project.mkdir()
    bin_dir.mkdir()
    _box_stl(stl)
    (project / "bracket.sc").write_text("design")
    cli = bin_dir / "solidcraft-cli"
    cli.write_text(FAKE.format(python=sys.executable, log=str(log), stl=str(stl), fail=str(tmp_path / "fail")))
    cli.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_dir}:/usr/bin:/bin")
    monkeypatch.delenv("TWINSMITH_SOLIDCRAFT", raising=False)
    monkeypatch.setenv("SECRET_TOKEN", "x")

    class Cell:
        design = str(project / "bracket.sc")
        root = project
        tool = hardware.Hardware(workspace=tmp_path / "workspace", roots=[project])
        fail = tmp_path / "fail"

        @staticmethod
        def calls():
            return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []

    return Cell()


def test_status_reports_the_solidcraft_version_and_pin(cell):
    answer = cell.tool.call("status")
    assert answer["result"]["tools"]["solidcraft-cli"]["version"] == "0.42.0"
    assert answer["versions"]["solidcraft-cli"] == "0.42.0"
    assert not any("solidcraft" in w for w in answer["warnings"])


def test_missing_solidcraft_is_reported_and_part_refuses_cleanly(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", "/nonexistent")
    monkeypatch.delenv("TWINSMITH_SOLIDCRAFT", raising=False)
    project = tmp_path / "project"
    project.mkdir()
    (project / "bracket.sc").write_text("design")
    tool = hardware.Hardware(workspace=tmp_path / "workspace", roots=[project])
    assert "solidcraft-cli" in tool.call("status")["result"]["missing"]
    answer = tool.call("part", design=str(project / "bracket.sc"))
    assert answer["ok"] is False and answer["error"]["code"] == "TOOL_MISSING"
    assert "HARDWARE.md" in answer["error"]["message"]
    assert not (tmp_path / "workspace" / "runs").exists()


def test_the_workspace_install_is_found_without_path(tmp_path, monkeypatch, cell):
    fake = cell.tool.paths()["solidcraft-cli"]
    monkeypatch.setenv("PATH", "/nonexistent")
    installed = tmp_path / "ws" / "tools" / "solidcraft" / "current" / "bin"
    installed.mkdir(parents=True)
    (installed / "solidcraft-cli").symlink_to(fake)
    found = hardware.Hardware(workspace=tmp_path / "ws", roots=[cell.root]).paths()["solidcraft-cli"]
    assert found == str(installed / "solidcraft-cli")


def test_part_rebuilds_with_parameters_into_a_new_folder(cell):
    answer = cell.tool.call("part", design=cell.design, parameters={"L": 12.5}, name="bracket-a")
    assert answer["ok"], answer
    out = Path(answer["result"]["folder"])
    assert out == cell.tool.runs / "bracket-a"
    assert {"part.step", "part.stl", "measure.json", "preview.png", "parameters.json"} <= set(answer["result"]["files"])
    assert not any(name.endswith(".sc") for name in answer["result"]["files"])   # a zip would block the import snapshot
    assert answer["result"]["design"]["path"] == str(Path(cell.design).resolve())
    assert len(answer["result"]["design"]["sha256"]) == 64
    assert answer["result"]["parameters"] == {"L": {"from": 10.0, "to": 12.5, "unit": "mm"}}
    assert answer["result"]["bodies"][0]["volume_mm3"] == 1000.0
    script = json.loads((out / "parameters.json").read_text())
    assert script == {"commands": [{"command": "parameters.change", "params": {"name": "L", "expression": "12.5"}}]}
    built = [c["argv"] for c in cell.calls() if c["argv"][0] in ("run", "eval", "snapshot") and "--results" not in c["argv"]]
    assert [argv[0] for argv in built] == ["run", "eval", "snapshot"]
    for argv in built:    # each step applies the same parameters to the source design; nothing saves a design
        assert argv[1] == str(out / "parameters.json") and argv[argv.index("--in") + 1] == str(Path(cell.design).resolve())
        assert "--save" not in argv
    assert all("SECRET_TOKEN" not in c["env"] for c in cell.calls())
    assert answer["versions"] == {"solidcraft-cli": "0.42.0"}


def test_part_writes_a_metre_usd_when_the_mesh_extra_is_there(cell):
    pytest.importorskip("pxr")
    pytest.importorskip("trimesh")
    from pxr import Usd, UsdGeom

    answer = cell.tool.call("part", design=cell.design, output=str(cell.root / "bracket-usd"))
    assert answer["ok"] and not answer["warnings"], answer
    usd = answer["result"]["usd"]
    assert usd["size_m"] == [0.01, 0.02, 0.005] and usd["watertight"] and usd["triangles"] == 12
    stage = Usd.Stage.Open(usd["output"])
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == "Z"
    assert stage.GetDefaultPrim().GetPath() == "/part"
    assert "assets__mesh" in answer["result"]["next"]


def test_part_refusals_write_nothing(cell):
    cases = [
        ({"parameters": {"Height": 3}}, "UNKNOWN_PARAMETER"),
        ({"parameters": {"L": "3; rm"}}, "INVALID_VALUE"),
        ({"parameters": {"L; x": 3}}, "INVALID_VALUE"),
        ({"parameters": {"L": True}}, "INVALID_VALUE"),
        ({"parameters": {"L": float("nan")}}, "INVALID_VALUE"),
        ({"output": "/tmp/elsewhere-part"}, "ROOT_NOT_APPROVED"),
        ({"output": "relative/out"}, "INVALID_PATH"),
        ({"name": "../up"}, "INVALID_VALUE"),
    ]
    for extra, code in cases:
        answer = cell.tool.call("part", design=cell.design, **extra)
        assert answer["ok"] is False and answer["error"]["code"] == code, (extra, answer)
    assert cell.tool.call("part", design=str(cell.root / "x.step"))["error"]["code"] == "INVALID_VALUE"
    assert not cell.tool.runs.exists() or not any(cell.tool.runs.iterdir())
    assert sorted(p.name for p in cell.root.iterdir()) == ["bracket.sc"]


def test_a_failed_rebuild_leaves_no_folder(cell):
    cell.fail.write_text("1")
    answer = cell.tool.call("part", design=cell.design, name="broken")
    assert answer["error"]["code"] == "PART_FAILED" and "feature Hole failed" in answer["error"]["stderr"]
    assert not (cell.tool.runs / "broken").exists()
    assert answer["versions"] == {"solidcraft-cli": "0.42.0"}
