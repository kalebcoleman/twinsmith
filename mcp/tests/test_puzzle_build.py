"""CPU-only checks of the puzzle build's options and its table of in-hand corrections (scripts/puzzle_build.py): no
simulator, no camera."""

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


build = load("puzzle_build")
CONFIG = json.loads((ROOT / "scripts/puzzle_build.json").read_text())
PLACED = [name for name in CONFIG["grasps"] if name != CONFIG["base"]]         # the pieces that are picked and inserted
RUN = ["--port", "1", "--out", "x", "--mutate-stage", "--perception", "oracle"]


def shift(x_mm, y_mm, z_mm):
    out = np.eye(4)
    out[:3, 3] = np.array([x_mm, y_mm, z_mm]) / 1000
    return out


def a_table(tmp_path, **changes):
    """A table as --write-corrections writes it, for the demo's own pieces."""
    rows = {piece: {"shift_mm": 4.0, "shift_deg": 0.0, "in_hand": shift(0, -2, -3.464).tolist(), "flipped": piece.startswith("fork"),
                    "grasp": {key: CONFIG["grasps"][piece][key] for key in build.GRASP_KEYS}} for piece in PLACED}
    table = {"schema": 1, "measured": {"pack": CONFIG["pack"], "grip_force_n": CONFIG["grip_force_n"]}, "pieces": rows}
    for key, value in changes.items():
        value(table) if callable(value) else table.update({key: value})
    path = tmp_path / "corrections.json"
    path.write_text(json.dumps(table))
    return path


def test_the_three_ways_to_correct_a_place_are_one_option_and_no_lookahead_still_means_none():
    assert build.arguments(RUN).correction == build.CORRECTION and build.CORRECTION in ("table", "lookahead")
    for way in ("table", "lookahead", "none"):
        assert build.arguments([*RUN, "--correction", way]).correction == way
    assert build.arguments([*RUN, "--no-lookahead"]).correction == "none"
    assert build.arguments([*RUN, "--no-lookahead", "--correction", "none"]).correction == "none"
    with pytest.raises(SystemExit):                            # the two say different things
        build.arguments([*RUN, "--no-lookahead", "--correction", "table"])
    for other in (["--correction", "table"], ["--correction", "none"], ["--no-lookahead"]):       # a table is made of what lookaheads read
        with pytest.raises(SystemExit):
            build.arguments([*RUN, *other, "--write-corrections", "t.json"])
    measuring = build.arguments([*RUN, "--write-corrections", "t.json"])
    assert measuring.correction == "lookahead" and measuring.write_corrections == Path("t.json")


def test_the_table_in_the_repository_has_a_measured_correction_for_every_piece_the_demo_places():
    table = json.loads(build.CORRECTIONS.read_text())
    assert sorted(table["pieces"]) == sorted(PLACED)
    found = {piece: {"in_hand": np.array(row["in_hand"])} for piece, row in table["pieces"].items()}
    used = build.read_corrections(build.CORRECTIONS, CONFIG, PLACED)
    assert sorted(used) == sorted(piece for piece, row in table["pieces"].items() if row["apply"])
    assert table["measured"]["how"] and table["measured"]["command"] and table["measured"]["friction_in_the_solver"]["pieces"]
    for piece, row in table["pieces"].items():
        mm, deg = build.in_hand_shift(found[piece]["in_hand"])
        assert (row["shift_mm"], row["shift_deg"]) == (mm, deg)                # the distance written beside an entry is the entry's own
        assert 0.0 < mm < 10.0 and deg < 5.0                                   # a piece shifts in the hand by millimetres, not by its own size
        assert np.allclose(found[piece]["in_hand"][:3, :3] @ found[piece]["in_hand"][:3, :3].T, np.eye(3), atol=1e-6)
        assert row["measured"]["lookahead_tries"] >= 1 and row["measured"]["rehearsal_final_error_mm"] <= CONFIG["seated"]["position_m"] * 1000
        assert row["apply"] or row["why_not_applied"]                          # a measurement that is not used says why


def test_a_table_is_refused_for_a_grasp_a_force_or_a_piece_it_was_not_measured_with(tmp_path):
    assert sorted(build.read_corrections(a_table(tmp_path), CONFIG, PLACED)) == sorted(PLACED)
    with pytest.raises(SystemExit, match="no correction for fork_yellow"):
        build.read_corrections(a_table(tmp_path, drop=lambda table: table["pieces"].pop("fork_yellow")), CONFIG, PLACED)
    with pytest.raises(SystemExit, match="measured with the grasp"):
        build.read_corrections(a_table(tmp_path, move=lambda table: table["pieces"]["line_red"]["grasp"].update(point_m=[0.0, 0.01, 0.0095])), CONFIG, PLACED)
    with pytest.raises(SystemExit, match="at 20.0 N"):
        build.read_corrections(a_table(tmp_path, measured={"pack": CONFIG["pack"], "grip_force_n": 20.0}), CONFIG, PLACED)
    with pytest.raises(SystemExit, match="other/v9"):
        build.read_corrections(a_table(tmp_path, measured={"pack": "other/v9", "grip_force_n": CONFIG["grip_force_n"]}), CONFIG, PLACED)
    # An entry may be kept as a measurement and not used: that piece is then placed by its grasp as defined.
    kept = build.read_corrections(a_table(tmp_path, keep=lambda table: table["pieces"]["line_red"].update(apply=False)), CONFIG, PLACED)
    assert sorted(kept) == sorted(set(PLACED) - {"line_red"})


def test_a_shift_measured_one_way_round_is_turned_with_the_gripper():
    cell = object.__new__(build.Build)
    cell.table = {"fork": {"in_hand": shift(0.5, -2.0, -4.0), "flipped": True}}
    assert np.array_equal(cell._table_shift("fork", True), shift(0.5, -2.0, -4.0))             # held as it was measured: as it is
    # Held half round, the piece sits in the gripper as before, seen from a gripper frame turned half round about the
    # way it points: what was to the gripper's -y is to its +y, and how far out along the fingers is the same.
    assert np.allclose(cell._table_shift("fork", False), shift(-0.5, 2.0, -4.0))
    assert build.in_hand_shift(cell._table_shift("fork", False)) == build.in_hand_shift(cell._table_shift("fork", True))


def test_without_a_lookahead_nothing_is_simulated_and_the_answer_is_go():
    step_log = sys.modules["step_log"]

    class NoCell:                                              # any use of the simulator fails the test
        def __getattr__(self, name):
            raise AssertionError(f"the simulator was asked for {name}")

    for way in ("table", "none"):
        cell = object.__new__(build.Build)
        cell.correction, cell.adjust, cell.cell, cell.said = way, {"bar": np.eye(4)}, NoCell(), []
        cell.log = step_log.StepLog(lambda: {"sim_s": 0.0, "frame": 0, "sample": 0, "spent": {}})
        cell.say = cell.said.append
        assert cell.lookahead("bar") == {"verdict": "go", "tries": []}
        assert cell.adjust == {}                               # and a correction left from an earlier lookahead is not kept
        assert cell.log.steps[0]["plan"] == {"simulated": False, "correction": way} and "no lookahead" in cell.said[0]


def test_a_table_is_written_from_the_rehearsals_that_seated_the_piece_and_read_back(tmp_path):
    read = lambda mm: {"in_hand": shift(0, 0, -mm).tolist(), "flipped": False, "way": 0, "shift_mm": mm, "shift_deg": 0.0}
    said = lambda verdict, seated, mm: {"verdict": verdict, "correction": read(mm),
                                        "tries": [{"position_error_mm": 0.6 if seated else 16.0, "rotation_error_deg": 0.1, "seated": seated,
                                                   "others_moved_mm": 0.1 if verdict == "go" else 4.2, "moved_most": "plate", "go": verdict == "go"}]}
    record = {"seed": 11, "perception": "oracle", "completed": False, "order": [{"piece": piece} for piece in PLACED],
              "cell": {"cell_pack": {"id": "ur5e-workcell", "version": "v4"}, "backend": "newton", "friction_in_the_solver": {"pieces": {}, "pads": [0.7]}},
              "pieces": {piece: {"lookahead": [said("abort", False, 9.0), said("go" if index else "abort", True, 1.0 + index)]} for index, piece in enumerate(PLACED)}}
    path = tmp_path / "table.json"
    build.write_corrections(path, record, CONFIG)
    table, found = json.loads(path.read_text()), build.read_corrections(path, CONFIG, PLACED)
    assert table["measured"]["seed"] == 11 and table["measured"]["cell_pack"] == "ur5e-workcell/v4" and table["measured"]["grip_force_n"] == CONFIG["grip_force_n"]
    for index, piece in enumerate(PLACED):                     # the rehearsal that seated the piece, not the one that left it 16 mm off
        assert table["pieces"][piece]["shift_mm"] == 1.0 + index and found[piece]["in_hand"][2, 3] == pytest.approx(-(1.0 + index) / 1000)
        assert table["pieces"][piece]["measured"]["pieces_in_the_base"] == PLACED[:index] and table["pieces"][piece]["apply"] is True
    # A rehearsal that seated the piece and said abort because something else moved is still a reading, and says so.
    assert table["pieces"][PLACED[0]]["measured"]["lookahead_verdict"] == "abort" and table["pieces"][PLACED[0]]["measured"]["rehearsal_moved_another_piece_mm"] == 4.2
    record["pieces"][PLACED[-1]]["lookahead"] = [said("abort", False, 9.0)]
    with pytest.raises(SystemExit, match=f"no lookahead of {PLACED[-1]}"):
        build.write_corrections(tmp_path / "none.json", record, CONFIG)
    assert not (tmp_path / "none.json").exists()
