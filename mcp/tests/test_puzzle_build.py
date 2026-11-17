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
scan, assembly, session = sys.modules["puzzle_scan"], sys.modules["puzzle_assembly"], sys.modules["workcell_session"]
CONFIG = json.loads((ROOT / "scripts/puzzle_build.json").read_text())
CORRECTED = CONFIG["corrected_from_table"]["pieces"]                           # the pieces whose place the table shifts
RUN = ["--port", "1", "--out", "x", "--mutate-stage", "--perception", "oracle"]
FRICTION = {"pieces": {"fork_orange": [0.4]}, "pads": [0.7]}
box = lambda centre, half: (np.array(centre, dtype=float), np.array(half, dtype=float))
TOY = {"plate": {"boxes": [box([0, 0, -0.005], [0.05, 0.05, 0.005]), box([-0.0215, 0, 0.004], [0.0085, 0.05, 0.004]), box([0.0215, 0, 0.004], [0.0085, 0.05, 0.004])],
                 "colour": [0.05, 0.3, 0.6]},
       "bar": {"boxes": [box([0, 0, 0], [0.012, 0.04, 0.012])], "colour": [0.7, 0.03, 0.03]},
       "rod": {"boxes": [box([0, 0, 0], [0.012, 0.04, 0.012])], "colour": [0.4, 0.2, 0.1]},     # the bar's shape again
       "cap": {"boxes": [box([0, 0, 0], [0.03, 0.01, 0.005])], "colour": [0.9, 0.5, 0.0]}}
TOY_SLOTS = {"plate": [{"piece": "bar", "position_m": [0.0, 0.0, 0.012], "rpy_deg": [0.0, 0.0, 0.0]}]}
TOY_GRASP = {"point_m": [0.0, 0.0, 0.0125], "closing_axis": [1, 0, 0], "approach_axis": [0, 0, -1], "width_m": 0.024}


def shift(x_mm, y_mm, z_mm):
    out = np.eye(4)
    out[:3, 3] = np.array([x_mm, y_mm, z_mm]) / 1000
    return out


def a_table(tmp_path, change=None):
    """A table as --write-corrections writes it, for the demo's own corrected pieces as one type."""
    table = {"measured": {"pack": CONFIG["pack"], "grip_force_n": CONFIG["grip_force_n"], "friction_in_the_solver": FRICTION},
             "entries": [{"pieces": list(CORRECTED), "grasp": {key: CONFIG["grasps"][CORRECTED[0]][key] for key in build.GRASP_KEYS},
                          "flipped": True, "in_hand": shift(0, -2, -3.464).tolist()}]}
    if change:
        change(table)
    path = tmp_path / "corrections.json"
    path.write_text(json.dumps(table))
    return path


def test_the_place_is_corrected_from_the_table_unless_asked_otherwise_and_there_is_one_way_to_ask():
    assert build.arguments(RUN).correction == "table"
    for way in ("table", "lookahead", "none"):
        assert build.arguments([*RUN, "--correction", way]).correction == way
    with pytest.raises(SystemExit):                            # one option says it; the old switch is gone and is not an alias
        build.arguments([*RUN, "--no-lookahead"])
    measuring = build.arguments([*RUN, "--write-corrections", "t.json"])       # a measuring run reads its shifts in lookaheads
    assert measuring.correction == "lookahead" and measuring.write_corrections == Path("t.json")


def test_the_table_in_the_repository_is_one_averaged_entry_per_piece_type_and_each_earns_its_place():
    table = json.loads(build.CORRECTIONS.read_text())
    found = build.read_corrections(build.CORRECTIONS, CONFIG)
    assert sorted(found["pieces"]) == sorted(CORRECTED) and found["friction"] == table["measured"]["friction_in_the_solver"]
    pack_free = [entry["pieces"] for entry in table["entries"]]
    assert pack_free == [["fork_orange", "fork_yellow"]]                       # the two forks are one shape held by one grasp: one entry
    assert set(table["measured"]["friction_in_the_solver"]["pieces"]["fork_orange"]) == {0.4}       # every colliding shape at the task's value
    for entry in table["entries"]:
        mean = np.array(entry["in_hand"])
        assert (entry["shift_mm"], entry["shift_deg"]) == build.in_hand_shift(mean)
        assert entry["readings"] == len(entry["each"]) >= build.MEASURE_LEAST and len({row["seed"] for row in entry["each"]}) >= 3
        assert np.allclose(1000 * mean[:3, 3], np.mean([row["xyz_mm"] for row in entry["each"]], axis=0), atol=0.01)
        assert np.allclose(mean[:3, :3] @ mean[:3, :3].T, np.eye(3), atol=1e-6) and 0.0 < entry["shift_mm"] < 10.0
        # An entry is worth having when a pick it was not measured on ends nearer the truth with it than without it.
        assert entry["check"]["corrected_median_mm"] < entry["check"]["uncorrected_median_mm"]
        for piece, row in entry["check"]["by_piece"].items():
            assert row["corrected_median_mm"] < row["uncorrected_median_mm"], piece


def test_a_table_is_refused_for_a_grasp_a_force_a_pack_or_pieces_it_was_not_measured_for(tmp_path):
    assert sorted(build.read_corrections(a_table(tmp_path), CONFIG)["pieces"]) == sorted(CORRECTED)
    with pytest.raises(SystemExit, match="the settings want"):                 # an entry short
        build.read_corrections(a_table(tmp_path, lambda table: table["entries"][0]["pieces"].pop()), CONFIG)
    with pytest.raises(SystemExit, match="the settings want"):                 # and an entry too many: a bar is placed by its grasp as defined
        build.read_corrections(a_table(tmp_path, lambda table: table["entries"][0]["pieces"].append("line_red")), CONFIG)
    with pytest.raises(SystemExit, match="measured with the grasp"):
        build.read_corrections(a_table(tmp_path, lambda table: table["entries"][0]["grasp"].update(point_m=[0.0, 0.01, 0.006])), CONFIG)
    with pytest.raises(SystemExit, match="at 20.0 N"):
        build.read_corrections(a_table(tmp_path, lambda table: table["measured"].update(grip_force_n=20.0)), CONFIG)
    with pytest.raises(SystemExit, match="other/v9"):
        build.read_corrections(a_table(tmp_path, lambda table: table["measured"].update(pack="other/v9")), CONFIG)


def test_pieces_of_one_shape_held_by_one_grasp_are_one_type():
    pack = scan.Pack(TOY, TOY_SLOTS, "toy/v1")
    config = {"grasps": {"bar": TOY_GRASP, "rod": dict(TOY_GRASP, what="as the bar"), "cap": TOY_GRASP}}
    assert build.piece_types(pack, config, ["bar", "cap", "rod"]) == [["bar", "rod"], ["cap"]]      # the cap has the grasp and another shape
    config["grasps"]["rod"] = dict(TOY_GRASP, width_m=0.02)
    assert build.piece_types(pack, config, ["bar", "rod"]) == [["bar"], ["rod"]]                    # the shape and another grasp


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


def test_a_corrected_place_that_puts_the_gripper_into_the_base_is_refused():
    import contextlib

    pack = scan.Pack(TOY, TOY_SLOTS, "toy/v1")
    plate = assembly.pose(np.eye(3), [0.0, 0.0, pack.shape("plate").rests[0].drop])
    now = assembly.pose(scan.yaw_matrix(0.5), [0.2, 0.1, pack.shape("bar").rests[0].drop])
    seat = plate @ assembly.slot_poses(pack, "plate")["bar"]
    hold = assembly.choose_grasps(pack, "bar", TOY_GRASP, now, seat, {"plate": plate}, {"plate": plate}, session.pad_setback, clear_m=0.03,
                                  open_m=0.016, hover_m=0.10)[0]
    cell = object.__new__(build.Build)
    cell.pack, cell.cell = pack, type("Timed", (), {"timed": lambda self, what: contextlib.nullcontext()})()
    hold.update(width_m=TOY_GRASP["width_m"], axis=np.array([0.0, 0.0, 1.0]), clear_m=0.03, others={"plate": plate}, corrected_by="table")
    cell._corrected_fits("bar", hold)                          # as defined (a correction of nothing) it fits
    hold["place"] = hold["place"] @ shift(0, 0, 3.0)           # a correction that lowers the gripper 3 mm: the pads are in the slot's walls
    with pytest.raises(build.Misfit, match="corrected from the table does not fit: the gripper would be in plate at the slot"):
        cell._corrected_fits("bar", hold)
    assert issubclass(build.Misfit, build.Refused)             # to the graph it is a refusal like any other


def test_the_table_is_the_mean_of_the_readings_of_a_type_with_a_check_that_leaves_each_layout_out(tmp_path):
    toy = scan.Pack({**TOY, "fork_orange": TOY["cap"], "fork_yellow": TOY["cap"]}, TOY_SLOTS, "toy/v1")
    reading = lambda seed, piece, xyz, flipped=True: {"seed": seed, "piece": piece, "verdict": "go", "flipped": flipped, "way": 0,
                                                      "in_hand": shift(*xyz).tolist(), "shift_mm": 0.0, "shift_deg": 0.0}
    readings = [reading(11, "fork_orange", (0.0, -1.0, -3.0)), reading(11, "fork_yellow", (0.0, 1.0, -5.0)),
                reading(12, "fork_orange", (0.0, 1.0, -4.0)), reading(13, "fork_yellow", (0.0, 1.0, -4.0), flipped=False)]
    built = {"cell_pack": {"id": "ur5e-workcell", "version": "v4"}, "backend": "newton", "friction_in_the_solver": FRICTION}
    path = tmp_path / "table.json"
    table = build.write_corrections(path, readings, built, CONFIG, toy, [11, 12, 13], [{"seed": 14, "why": "the base stopped short"}])
    assert json.loads(path.read_text()) == table and [entry["pieces"] for entry in table["entries"]] == [["fork_orange", "fork_yellow"]]
    entry = table["entries"][0]
    # The fourth reading was taken with the gripper half round: turned to the way of the first, its y is -1.
    assert [row["xyz_mm"] for row in entry["each"]] == [[0.0, -1.0, -3.0], [0.0, 1.0, -5.0], [0.0, 1.0, -4.0], [0.0, -1.0, -4.0]]
    assert np.allclose(1000 * np.array(entry["in_hand"])[:3, 3], [0.0, 0.0, -4.0]) and entry["shift_mm"] == 4.0 and entry["readings"] == 4
    # Each reading against the mean of the readings of the other layouts: layout 11's two against (0, 0, -4), 1.41 mm
    # each; layout 12's and 13's against the other three, 1.33 mm each. The median of the four is 1.37 mm.
    assert entry["check"]["by_piece"]["fork_orange"]["readings"] == 2 and entry["check"]["uncorrected_median_mm"] == pytest.approx(4.12, abs=0.01)
    assert entry["check"]["corrected_median_mm"] == pytest.approx(1.37, abs=0.01)
    assert table["measured"]["seeds"] == [11, 12, 13] and table["measured"]["layouts_without_a_reading"][0]["seed"] == 14
    assert sorted(build.read_corrections(path, CONFIG)["pieces"]) == sorted(CORRECTED)
    with pytest.raises(SystemExit, match="2 readings"):        # too few to call a mean
        build.write_corrections(tmp_path / "none.json", readings[:2], built, CONFIG, toy, [11], [])
    assert not (tmp_path / "none.json").exists()


def test_a_run_ended_by_an_invalid_physics_state_records_what_was_known_without_asking_the_simulator():
    class Gone:                                                # only the count of steps is read; nothing is asked
        steps = 5000

        def __getattr__(self, name):
            raise AssertionError(f"the simulator was asked for {name}")

    cell = object.__new__(build.Build)
    cell.cell, cell.metrics, cell.order, cell.correction = Gone(), {"bar": {"attempts": 1, "insert_success": True}}, [{"piece": "bar"}], "table"
    cell.args = type("Args", (), {"perception": "oracle"})()
    record = cell.stopped(session.PhysicsStateInvalid("values that are not finite numbers for the joint positions at physics step 5000"), 0.0)
    assert record["completed"] is False and record["stopped"].startswith("PHYSICS_STATE_INVALID: ") and "physics step 5000" in record["stopped"]
    assert record["pieces"] == cell.metrics and record["simulated_seconds"] == 10.0 and json.dumps(record)
