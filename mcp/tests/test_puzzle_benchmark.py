"""CPU-only checks of the puzzle build's benchmark (scripts/puzzle_benchmark.py): its cases, its layouts on a small
made-up pack, a case's row from the folder a build leaves, and the bar. No simulator."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


scan = load("puzzle_scan")
assembly = load("puzzle_assembly")
bench = load("puzzle_benchmark")
CONFIG = json.loads((ROOT / "scripts/puzzle_benchmark.json").read_text())
box = lambda centre, half: (np.array(centre, dtype=float), np.array(half, dtype=float))
PIECES = {"plate": {"boxes": [box([0, 0, -0.005], [0.05, 0.05, 0.005])], "colour": [0.05, 0.3, 0.6]},
          "bar": {"boxes": [box([0, 0, 0], [0.012, 0.04, 0.012])], "colour": [0.7, 0.03, 0.03]},
          "cap": {"boxes": [box([0, 0, 0], [0.03, 0.01, 0.005])], "colour": [0.9, 0.5, 0.0]}}
SLOTS = {"plate": [{"piece": "cap", "position_m": [0.0, 0.0, 0.029], "rpy_deg": [0.0, 0.0, 0.0]},
                   {"piece": "bar", "position_m": [0.0, 0.0, 0.012], "rpy_deg": [0.0, 0.0, 0.0]}]}
REGION, HIDDEN = [[-0.36, -0.64], [0.40, -0.30]], [[[-0.07, -0.38], [0.14, -0.25]]]


@pytest.fixture(scope="module")
def pack():
    return scan.Pack(PIECES, SLOTS, "toy/v1")


def target(pack):
    return assembly.pose(scan.yaw_matrix(np.pi / 2), [0.0, -0.46, pack.shape("plate").rests[0].drop])


def test_the_frozen_cases_are_six_layouts_of_three_seeds_and_four_disturbances_of_three():
    cases = bench.cases(CONFIG)
    names = [case["name"] for case in cases]
    assert len(names) == len(set(names))
    base = [case for case in cases if case["group"] == "base"]
    assert len({case["layout"] for case in base}) >= 6 and all(sum(other["layout"] == case["layout"] for other in base) >= 3 for case in base)
    assert len({(case["layout"], case["seed"]) for case in base}) == len(base) == 18
    assert CONFIG["perception"] == "sam3"
    disturbed = [case for case in cases if case["group"] == "disturbance"]
    assert sorted({case["disturbance"]["kind"] for case in disturbed}) == ["base_moved", "piece_knocked", "table_shift_100", "table_shift_50"]
    assert len(disturbed) == 12 and all(case["disturbance"]["on_caption"] for case in disturbed)
    # A disturbed case is a base case with something done to it: the same table, so the two can be read side by side.
    assert all(f"base/{case['layout']}/{case['seed']}" in names for case in disturbed)
    assert {case["disturbance"]["mm"] for case in disturbed if case["disturbance"]["kind"].startswith("table_shift")} == {50, 100}
    # None of the seeds the build or its table were worked out on.
    assert all(case["seed"] >= 21 for case in cases)
    assert all(case["layout"] in CONFIG["layouts"] for case in cases)
    stretch = {case["group"] for case in cases if case["group"].startswith("stretch/")}
    assert {"stretch/reach_far", "stretch/reach_side", "stretch/long_carry"} <= stretch
    far = [case["settings"]["base_target"]["position_m"][1] for case in cases if case["group"] == "stretch/reach_far"]
    assert far == sorted(far, reverse=True) and far[0] == -0.46      # outward from the build's own target, step by step
    # A disturbed case is built on the table of the base case it disturbs; a stretch case, whose target differs, on its own.
    tables = {case["name"]: case["table"] for case in cases}
    assert tables["base/far/63"] == tables["disturbance/base_moved/far_63"] == "far/63"
    assert tables["stretch/reach_far/y-540"] == "stretch/reach_far/y-540"


def test_every_case_has_its_table_and_a_table_is_kept_as_numbers():
    low, high = np.array(CONFIG["region"][0]), np.array(CONFIG["region"][1])
    pieces = ["base3", "line_brown", "line_red", "fork_orange", "fork_yellow"]
    for case in bench.cases(CONFIG):
        table = CONFIG["tables"][case["table"]]                                # a missing one fails here: the benchmark is not runnable without it
        assert list(table) == pieces, case["name"]                             # the base first
        spots = np.array([[x, y] for x, y, yaw in table.values()])
        assert (spots > low).all() and (spots < high).all(), case["name"]
        spec = CONFIG["layouts"][case["layout"]]
        loose = np.array([[x, y] for name, (x, y, yaw) in table.items() if name in spec.get("band_for", pieces[1:])])       # the pieces a band holds for
        if "pieces_y" in spec:
            assert (loose[:, 1] >= spec["pieces_y"][0]).all() and (loose[:, 1] <= spec["pieces_y"][1]).all(), case["name"]
        if "pieces_x" in spec:
            assert (loose[:, 0] >= spec["pieces_x"][0]).all() and (loose[:, 0] <= spec["pieces_x"][1]).all(), case["name"]
        if "side" in spec:
            middle = (case["settings"] or {}).get("base_target", {}).get("position_m", [0.0, -0.46])[0]
            assert np.sign(spots[0, 0] - middle) == spec["side"], case["name"]


def test_a_table_comes_back_as_the_poses_it_was_made_from(pack):
    middle = target(pack)
    poses = bench.lay_out(assembly, scan, pack, "plate", middle, np.random.default_rng(4), {}, REGION, HIDDEN)
    table = bench.table_of(assembly, scan, pack, poses)
    assert list(table) == ["plate", "cap", "bar"] and all(len(row) == 3 for row in table.values())
    again = bench.poses_of(assembly, scan, pack, json.loads(json.dumps(table)))
    for name in poses:
        assert np.allclose(again[name], poses[name], atol=2e-6), name


def test_settings_for_a_case_are_laid_over_the_builds_own():
    settings = {"base": "plate", "base_target": {"position_m": [0.0, -0.46], "yaw_deg": 90.0}, "speeds": {"insert_m_s": 0.01}}
    changed = bench.merged(settings, {"base_target": {"position_m": [0.24, -0.46]}})
    assert changed["base_target"] == {"position_m": [0.24, -0.46], "yaw_deg": 90.0} and changed["speeds"] == settings["speeds"]
    assert settings["base_target"]["position_m"] == [0.0, -0.46] and bench.merged(settings, None) == settings


def test_a_layout_is_of_its_kind_inside_the_region_clear_of_what_is_hidden_and_the_same_for_the_same_seed(pack):
    middle = target(pack)
    laid = lambda spec, seed: bench.lay_out(assembly, scan, pack, "plate", middle, np.random.default_rng(seed), spec, REGION, HIDDEN)
    low, high = np.array(REGION[0]), np.array(REGION[1])
    hidden = [np.array([a, [b[0], a[1]], b, [a[0], b[1]]]) for a, b in HIDDEN]
    for kind in ("scatter", "base_left", "base_right", "tight", "far", "near", "pieces_left", "short_slide"):
        spec = CONFIG["layouts"][kind]
        poses = laid(spec, 5)
        again = laid(spec, 5)
        assert list(poses) == ["plate", "cap", "bar"] and all(np.array_equal(poses[name], again[name]) for name in poses), kind
        assert not np.array_equal(poses["bar"], laid(spec, 6)["bar"])
        loose = {name: at for name, at in poses.items() if name != "plate"}
        for name, at in poses.items():
            corners = assembly.footprint(pack, name, at)
            assert (corners > low).all() and (corners < high).all(), (kind, name)
            assert all(assembly._apart(corners, patch) for patch in hidden), (kind, name)
        assert assembly.path_clear(pack, "plate", poses["plate"], middle, loose, margin_m=0.02), kind      # the way to the middle is free
        nearest, furthest = spec.get("base_side_m", [0.24, 0.32])
        assert nearest <= abs(poses["plate"][0, 3] - middle[0, 3]) <= furthest, kind
        if "side" in spec:
            assert np.sign(poses["plate"][0, 3] - middle[0, 3]) == spec["side"], kind
        banded = [at for name, at in loose.items() if name in spec.get("band_for", loose)]     # the pieces a band holds for: all, unless it names some
        if "pieces_y" in spec:
            assert all(spec["pieces_y"][0] <= at[1, 3] <= spec["pieces_y"][1] for at in banded), kind
        if "pieces_x" in spec:
            assert all(spec["pieces_x"][0] <= at[0, 3] <= spec["pieces_x"][1] for at in banded), kind
        gap = spec.get("gap_m", 0.035)
        names = list(loose)
        for index, first in enumerate(names):
            for second in names[index + 1:]:
                assert assembly._apart(assembly.footprint(pack, first, loose[first], gap), assembly.footprint(pack, second, loose[second])), kind
        if "cluster_m" in spec:
            spots = np.array([at[:2, 3] for at in loose.values()])
            assert np.linalg.norm(spots - spots.mean(0), axis=1).max() <= 2 * spec["cluster_m"], kind
    # A band that names its pieces holds for them and leaves the others anywhere.
    far = [laid({"pieces_y": [-0.64, -0.52], "band_for": ["cap"]}, seed) for seed in range(8)]
    assert all(poses["cap"][1, 3] <= -0.52 for poses in far) and any(poses["bar"][1, 3] > -0.52 for poses in far)
    with pytest.raises(ValueError, match="no layout of this kind"):        # a band no piece fits in
        bench.lay_out(assembly, scan, pack, "plate", middle, np.random.default_rng(1), {"pieces_y": [-0.31, -0.30]}, REGION, HIDDEN, tries=3)


def test_another_puzzles_pieces_lie_on_the_table_too_and_leave_the_puzzles_own_where_they_were():
    both = scan.Pack({**PIECES, "peg": {"boxes": [box([0, 0, 0], [0.02, 0.02, 0.01])], "colour": [0.1, 0.6, 0.1]}}, SLOTS, "toy/v1")
    middle = target(both)
    laid = lambda spec: bench.lay_out(assembly, scan, both, "plate", middle, np.random.default_rng(5), spec, REGION, HIDDEN)
    alone, crowded = laid({}), laid({"also": ["peg"]})
    assert list(crowded) == ["plate", "cap", "bar", "peg"]                 # the base first, then its own pieces, then the others
    assert all(np.array_equal(alone[name], crowded[name]) for name in alone)      # the same dice for the puzzle's own
    peg = assembly.footprint(both, "peg", crowded["peg"], 0.035)
    assert all(assembly._apart(peg, assembly.footprint(both, name, at)) for name, at in alone.items())
    assert all(assembly._apart(peg, assembly.footprint(both, "plate", at, 0.03)) for at in assembly.slide_path(crowded["plate"], middle))


def a_build(folder, seated, attempts=None, stopped="unset", scans=1, base_checks=None, success=None):
    """The folder a build leaves, as far as a row reads it."""
    folder.mkdir(parents=True)
    names = ["line_brown", "line_red", "fork_orange", "fork_yellow"]
    pieces = {name: {"attempts": (attempts or {}).get(name, 1), "final_seated": name in seated,
                     "final_position_error_mm": seated.get(name, 240.0)} for name in names}
    metrics = {"completed": len(seated) == 4, "pieces": pieces, "wall_seconds": 300.0, "simulated_seconds": 160.0, "correction": "table",
               "graph": {"run": {"success": len(seated) == 4 if success is None else success, "exit_status": "success" if len(seated) == 4 else "failure"}},
               "cell": {"friction_in_the_solver": {"pieces": {"line_brown": [0.4]}, "pads": [0.7]}}, "video": str(folder / "build.mp4")}
    if stopped != "unset":
        metrics["stopped"] = stopped
    if base_checks:
        metrics["base_checks"] = base_checks
    (folder / "metrics.json").write_text(json.dumps(metrics))
    steps = [{"kind": "scan", "piece": None, "parent": None}] * scans + [{"kind": "rescan", "piece": "fork_orange", "parent": None,
                                                                         "error": "fork_orange is not in its slot and cannot be picked again"}] * (len(seated) < 4)
    (folder / "run_log.json").write_text(json.dumps({"steps": steps, "looks": [{}] * 6}))
    return folder


def test_a_row_says_what_was_seated_how_well_and_what_it_cost(tmp_path):
    case = {"name": "base/far/61", "group": "base", "layout": "far", "seed": 61, "disturbance": None}
    ran = {"exit": 0, "wall_s": 340.0, "injected": None, "timed_out": False}
    whole = a_build(tmp_path / "a", {"line_brown": 0.9, "line_red": 0.5, "fork_orange": 1.1, "fork_yellow": 0.7}, attempts={"fork_yellow": 2},
                    stopped=None, scans=3, base_checks={"checks": [1, 2, 3], "shifts": [], "corrections": [1]})
    row = bench.row_of(whole, case, "integ", ran, {"goals": 52, "goals_refused": 0})
    assert (row["pieces_seated"], row["pieces_total"]) == (4, 4) and row["seat_error_mm"] == {"mean": 0.8, "max": 1.1}
    assert row["wall_s"] == 300.0 and row["rescans"] == 2 and row["failed_picks"] == 1 and row["stopped"] is None and row["recovered"] is None
    assert row["base_checks"] == {"checks": 3, "shifts": 0, "corrections": 1} and row["bridge"]["goals_refused"] == 0
    # A build of a checkout that does not say why it stopped (main): the row finds the refusal in its log.
    part = a_build(tmp_path / "b", {"line_brown": 0.9, "line_red": 0.5})
    row = bench.row_of(part, case, "main", {**ran, "exit": 0})
    assert row["pieces_seated"] == 2 and row["failed_picks"] == 2 and row["rescans"] == 0
    assert row["stopped"] == "rescan of fork_orange refused: fork_orange is not in its slot and cannot be picked again"
    # A disturbed case recovered only when every piece is seated and the graph ended in success.
    shaken = {**case, "name": "disturbance/base_moved/far_63", "group": "disturbance", "disturbance": {"kind": "base_moved", "mm": 25}}
    assert bench.row_of(whole, shaken, "integ", ran)["recovered"] is True
    assert bench.row_of(part, shaken, "integ", ran)["recovered"] is False
    assert bench.row_of(a_build(tmp_path / "c", dict.fromkeys(pieces_of(whole), 0.5), success=False), shaken, "integ", ran)["recovered"] is False
    # A build that left nothing (it crashed, or ran into the time limit) seats nothing.
    (tmp_path / "d").mkdir()
    (tmp_path / "d" / "build.log").write_text("  [ 12.0 s sim,   40 s wall] base3: looking again\nTraceback ...\nKeyError: 'x'\n")
    gone = bench.row_of(tmp_path / "d", case, "integ", {**ran, "exit": 1})
    assert gone["pieces_seated"] == 0 and gone["pieces_total"] is None and "KeyError" in gone["stopped"]
    assert bench.row_of(tmp_path / "d", case, "integ", {**ran, "exit": -9, "timed_out": True})["stopped"] == "the time limit"


def pieces_of(folder):
    return json.loads((folder / "metrics.json").read_text())["pieces"]


def rows_for(label, seated_by_case, wall=300.0, recovered=None, bridge=None):
    rows = []
    for name, seated in seated_by_case.items():
        group = name.split("/")[0]
        rows.append({"case": name, "group": group, "layout": name.split("/")[1], "label": label, "pieces_seated": seated, "pieces_total": 4,
                     "wall_s": wall, "recovered": (recovered or {}).get(name) if group == "disturbance" else None, "bridge": bridge,
                     "seat_error_mm": None, "rescans": 0, "failed_picks": 0, "stopped": None, "folder": "x"})
    return rows


def test_the_bar_is_read_off_the_rows():
    cases = {"base/far/61": 4, "base/far/62": 4, "base/near/71": 4, "base/near/72": 4, "disturbance/base_moved/far_63": 4}
    main = rows_for("main", {**cases, "base/far/62": 2, "disturbance/base_moved/far_63": 1})
    ours = rows_for("integ", cases, wall=280.0, recovered={"disturbance/base_moved/far_63": True}, bridge={"goals": 50, "goals_refused": 0})
    known = rows_for("table", {name: 4 for name in cases if name.startswith("base/")}, wall=290.0)
    said = {item["item"]: item for item in bench.bar(main + ours + known, "main", "integ", "table")}
    assert [item["passed"] for item in said.values()] == [True] * 5
    assert said["integ seats more than main, and no layout is worse"]["detail"].startswith("integ 20 of 20 (100 percent), main 15 of 20 (75 percent)")
    # One layout worse than main fails the first item even when the total is better.
    worse = rows_for("integ", {**cases, "base/near/71": 3}, recovered={"disturbance/base_moved/far_63": True})
    first = bench.bar(main + worse, "main", "integ")[0]
    assert first["passed"] is False and "worse on ['near']" in first["detail"]
    # 15 of 16 on the base cases is under 95 percent; a disturbed build that did not recover is under 90 percent of one.
    said = bench.bar(main + worse, "main", "integ")
    assert said[1]["passed"] is False and said[1]["detail"].startswith("15 of 16")
    lost = rows_for("integ", cases, recovered={"disturbance/base_moved/far_63": False})
    assert bench.bar(main + lost, "main", "integ")[2]["passed"] is False
    # Slower than the reference, a goal refused by the bridge, a seated build the bridge did not judge: each fails its item.
    slow = rows_for("integ", cases, wall=400.0, recovered={"disturbance/base_moved/far_63": True}, bridge={"goals": 50, "goals_refused": 3})
    said = bench.bar(main + slow + known, "main", "integ", "table")
    assert said[3]["passed"] is False and said[4]["passed"] is False and "15 goals refused in 5 of them" in said[4]["detail"]
    assert bench.bar(main + rows_for("integ", cases), "main", "integ")[4]["passed"] is False
    # What was not built cannot be judged, and is said so.
    assert [item["passed"] for item in bench.bar(main, "main", "integ")] == [None] * 5
    assert bench.bar(main + ours, "main", "integ")[3]["passed"] is None


def test_the_summary_gathers_every_row_and_writes_the_table(tmp_path, capsys):
    case = {"name": "base/far/61", "group": "base", "layout": "far", "seed": 61, "disturbance": None}
    ran = {"exit": 0, "wall_s": 340.0, "injected": None, "timed_out": False}
    for label, seated in (("main", {"line_brown": 0.9}), ("integ", {"line_brown": 0.9, "line_red": 0.5, "fork_orange": 1.1, "fork_yellow": 0.7})):
        folder = a_build(tmp_path / label / "base/far/61", seated, stopped=None if len(seated) == 4 else "rescan of line_red was refused")
        (folder / "row.json").write_text(json.dumps(bench.row_of(folder, case, label, ran)))
    bench.summary(type("Args", (), {"out": tmp_path, "main": "main", "ours": "integ", "reference": None})())
    rows = json.loads((tmp_path / "metrics.json").read_text())
    assert [(row["label"], row["pieces_seated"]) for row in rows] == [("integ", 4), ("main", 1)]
    text = (tmp_path / "summary.md").read_text()
    assert "| base/far/61 | 1 of 4, 0.9 / 0.9 mm, 300 s, 0 rescans, 3 failed picks | 4 of 4, 0.8 / 1.1 mm, 300 s, 0 rescans, 0 failed picks |" in text
    assert "| integ seats more than main, and no layout is worse | yes |" in text and "rescan of line_red was refused" in text
