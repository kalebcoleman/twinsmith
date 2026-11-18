"""CPU-only checks of the puzzle build's options and its table of in-hand corrections (scripts/puzzle_build.py): no
simulator, no camera."""

import importlib.util
import json
import math
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
    table = {"measured": {"pack": CONFIG["pack"], "grip_force_n": CONFIG["grip_force_n"], "held_deg_s": CONFIG["speeds"]["held_deg_s"],
                          "friction_in_the_solver": FRICTION},
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
    with pytest.raises(SystemExit, match="carried at 45 degrees a second"):    # a held piece creeps in the fingers by the speed of its carry
        build.read_corrections(a_table(tmp_path, lambda table: table["measured"].update(held_deg_s=45.0)), CONFIG)


def test_a_piece_in_the_fingers_is_carried_slower_than_an_empty_hand_moves():
    assert CONFIG["speeds"]["held_deg_s"] < CONFIG["speeds"]["joint_deg_s"]
    carried = []

    class Cell:                                                 # the carry over the slot is all this lets through
        def gripper_line(self, pose, speed_m_s):
            carried.append(("straight up", float(pose[2, 3])))

        def gripper_carry(self, pose, speed_deg_s):
            carried.append(speed_deg_s)
            raise build.cell.MoveFailed("far enough")

    made = object.__new__(build.Build)
    made.config, made.cell, made.ahead = CONFIG, Cell(), True
    made.log = sys.modules["step_log"].StepLog(lambda: {"sim_s": 0.0, "frame": 0, "sample": 0, "spent": {}})
    made.held = {"width_m": 0.021, "path": {"carry": np.eye(4), "over": np.eye(4)}}
    made.say = lambda text: None
    with pytest.raises(build.cell.MoveFailed):
        made.insert("line_red")
    assert carried == [CONFIG["speeds"]["held_deg_s"]]
    # A piece the hand turns level on the way swings down under the hand: it goes straight up first, to turn above
    # what is built (on one layout the second fork, turned beside the base, knocked the first and the base with it).
    carried.clear()
    made.held["path"]["above"] = build.shifted(np.eye(4), [0, 0, 0.27])
    with pytest.raises(build.cell.MoveFailed):
        made.insert("fork_orange")
    assert carried == [("straight up", 0.27), CONFIG["speeds"]["held_deg_s"]]


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


def test_the_grasp_taken_is_the_first_that_keeps_wrist_3_in_its_window(monkeypatch):
    import contextlib

    spot = lambda x: assembly.pose(np.eye(3), [x, 0.0, 0.0])
    ways = [{"pick": spot(way), "place": spot(way + 0.5)} for way in range(3)]
    monkeypatch.setattr(assembly, "choose_grasps", lambda *given, **named: list(ways))

    class Arm:
        wrist_from_gripper = np.eye(4)

        def __init__(self, wrist_3_deg, out_of_reach=()):
            self.turns, self.far = wrist_3_deg, out_of_reach

        def timed(self, what):
            return contextlib.nullcontext()

        def reconfigure(self, pose):
            way = int(pose[0, 3])                             # every pose of a way has its x between way and way + 1
            if way in self.far or way >= len(self.turns):
                raise session.MoveFailed("out of reach")
            turn = self.turns[way]                            # one angle for the pick and the place, or one for each
            return {"wrist_3_joint": math.radians(turn[int(pose[0, 3] - way >= 0.5)] if isinstance(turn, tuple) else turn)}

    def with_range(arm, wrist_3_deg, avoid=()):
        cell = object.__new__(build.Build)
        cell.config = {"grasps": {"bar": TOY_GRASP}, "open_beyond_m": 0.016, "arm": {"wrist_3_deg": wrist_3_deg, "wrist_3_rise_deg": 90.0}}
        cell.pack, cell.cell, cell.avoid = None, arm, {"bar": set(avoid)}
        return cell

    taken = lambda arm, avoid=(): with_range(arm, [-60.0, 110.0], avoid)._grasp("bar", spot(0), spot(0.5), {}, {})

    first = taken(Arm([20.0, 30.0, 40.0]))
    assert first["way"] == 0 and first["wrist_3_deg"] == [20.0, 20.0, 20.0]        # all inside: the order of the ways decides, as before
    assert taken(Arm([150.0, 30.0, 40.0]))["way"] == 1                            # the first way winds the cord: the next inside the window
    assert taken(Arm([150.0, 30.0, 40.0]), avoid=[1])["way"] == 2                 # a way a lookahead found blocked is still passed over
    assert taken(Arm([150.0, -90.0, 170.0]))["way"] == 1                          # none inside (-60 to 110 here): the one least outside, and the record says so
    assert taken(Arm([150.0, -90.0, 170.0]))["in_window"] is False and taken(Arm([150.0, 150.0, 170.0]))["way"] == 0      # the first of equals
    assert taken(Arm([150.0, -90.0, 170.0], out_of_reach=[1]))["way"] == 0
    with pytest.raises(build.Refused, match="within reach"):
        taken(Arm([20.0, 30.0, 40.0], out_of_reach=[0, 1, 2]))
    # Both ends inside the window is not enough: from the pick to the place wrist 3 turns up a quarter turn at most.
    rising = taken(Arm([(-40.0, 69.0), (-20.0, 69.0), (90.0, 0.0)]))
    assert rising["way"] == 1 and rising["wrist_3_deg"] == [-20.0, 69.0, 69.0] and rising["in_window"] is True
    assert taken(Arm([(-40.0, 69.0), (90.0, 0.0)]))["way"] == 1                    # turning down is as far as the window is wide
    assert taken(Arm([(-40.0, 69.0)]))["in_window"] is False
    # The bar of benchmark table scatter/21 (the range is -90 to 100 there): put in at -108.2 degrees it is out of the range, which
    # is worse than turned up 108.7 degrees to 69.5 inside it, though the one is 18.2 degrees too far and the other 18.7.
    wide = lambda turns: (lambda cell: cell._grasp("bar", spot(0), spot(0.5), {}, {}))(with_range(Arm(turns), [-90.0, 100.0]))
    assert wide([(-39.2, -108.2), (-39.2, 69.5), (140.8, 69.5)])["way"] == 1


def test_a_piece_whose_pick_winds_wrist_3_out_of_its_window_is_turned_on_the_table_first():
    step_log = sys.modules["step_log"]
    pack = scan.Pack(TOY, TOY_SLOTS, "toy/v1")
    lying = assembly.pose(np.eye(3), [0.2, -0.4, pack.shape("bar").rests[0].drop])
    yaw_of = lambda at: math.degrees(math.atan2(at[1, 0], at[0, 0]))

    class Arm:                                                 # wrist 3 follows the gripper's turn about the vertical, from -150 as the bar lies
        wrist_from_gripper = np.eye(4)

        def __init__(self):
            self.did = []

        def reconfigure(self, pose):
            return {"wrist_3_joint": math.radians((yaw_of(pose) - 150 + 180) % 360 - 180)}

        def __getattr__(self, name):
            return lambda *given, **named: self.did.append(name)

    def turned(turn_grasps, room=True, turning_in_window=True):
        cell = object.__new__(build.Build)
        cell.config = {"turn_grasps": turn_grasps, "arm": {"wrist_3_deg": [-60.0, 110.0]}, "open_beyond_m": 0.016, "grip_force_n": 40.0,
                       "speeds": {"approach_m_s": 0.05, "joint_deg_s": 45.0, "held_deg_s": 20.0}}
        cell.pack, cell.cell, cell.estimate, cell.metrics, cell.said = pack, Arm(), {"bar": lying}, {"bar": {}}, []
        cell.say, cell._seat = cell.said.append, lambda piece: lying
        cell.log = step_log.StepLog(lambda: {"sim_s": 0.0, "frame": 0, "sample": 0, "spent": {}})

        def grasp(piece, now, target, around_now, around_then, axis=(0, 0, 1), clear_m=0.0, grasp=None):
            wrist = math.degrees(cell.cell.reconfigure(now)["wrist_3_joint"])
            inside = turning_in_window if grasp else -60 <= wrist <= 110
            return {"pick": now, "place": target, "width_m": 0.024, "wrist_3_deg": [round(wrist, 1)], "in_window": inside}

        cell._grasp = grasp
        # Where the look finds it: slid 1.5 mm, and read ``misread`` degrees from the turn the open step asked for.
        cell.misread = 1.2
        cell.lands = lambda: assembly.pose(scan.yaw_matrix(math.radians(cell.log.steps[-1]["plan"]["deg"] + cell.misread)), [0.2015, -0.4, lying[2, 3]])
        cell._look = lambda label: ({"bar": cell.lands()} if cell.misread is not None else {}, {})
        others = {} if room else {"plate": assembly.pose(np.eye(3), [0.2, -0.4, 0.0])}
        hold = {"pick": lying, "width_m": 0.024, "wrist_3_deg": [-150.0], "in_window": False}
        return cell, cell._turn("bar", hold, others, np.array([0.0, 0.0, 1.0]), 0.03)

    cell, done = turned({})
    assert done is False and cell.cell.did == []               # no grip to turn it by: it is picked as it lies
    across = dict(TOY_GRASP, closing_axis=[0, 1, 0], what="across")
    cell, done = turned({"bar": across})
    step = cell.log.steps[0]
    # From -150 the middle of the window (25) is 175 degrees on; of the angles tried, 15 degrees apart, 180 and 165 are as near.
    assert done is True and step["kind"] == "turn" and step["plan"]["deg"] in (165, 180) and cell.metrics["bar"]["turned_deg"] == step["plan"]["deg"]
    assert cell.cell.did == ["open_to", "gripper_carry", "gripper_line", "grip", "gripper_line", "gripper_carry", "gripper_line", "open_to", "gripper_line"]
    # Where it lies is the look's (it slid 1.5 mm); how it is turned is the gripper's, which squared it.
    apart = lambda a, b: abs((a - b + 180) % 360 - 180)
    assert np.allclose(cell.estimate["bar"][:3, 3], [0.2015, -0.4, lying[2, 3]]) and 1.0 < step["result"]["off_mm"] < 2.6
    assert apart(yaw_of(cell.estimate["bar"]), step["plan"]["deg"]) < 1e-6 and step["result"] == dict(step["result"], off_deg=1.2, turn_taken_from="the gripper")
    cell.misread, cell.estimate["bar"] = 8.0, lying            # turned again, and found 8 degrees from the gripper's turn: the look is believed
    assert cell._turn("bar", {"pick": lying, "width_m": 0.024, "wrist_3_deg": [-150.0], "in_window": False}, {}, np.array([0.0, 0.0, 1.0]), 0.03)
    again = cell.log.steps[-1]
    assert apart(yaw_of(cell.estimate["bar"]), again["plan"]["deg"] + 8.0) < 1e-6 and again["result"]["turn_taken_from"] == "the look"
    assert "wrist 3 at -150 degrees" in cell.said[0]
    assert turned({"bar": across}, room=False)[1] is False                       # another piece where it would lie
    assert turned({"bar": across}, turning_in_window=False)[1] is False          # the turning grip itself out of the window
    cell.misread, cell.estimate["bar"] = None, lying           # turned once more, and gone from the look after it
    with pytest.raises(build.Refused, match="does not find it"):
        cell._turn("bar", {"pick": lying, "width_m": 0.024, "wrist_3_deg": [-150.0], "in_window": False}, {}, np.array([0.0, 0.0, 1.0]), 0.03)


def test_the_turn_on_the_table_is_off_as_shipped_and_every_piece_that_needs_one_has_a_grip_for_it():
    assert CONFIG["arm"]["turn_on_table"] is False and CONFIG["arm"]["wrist_3_rise_deg"] == 90.0
    assert sorted(set(CONFIG["turn_grasps"]) - {"what"}) == sorted(set(CONFIG["grasps"]) - {CONFIG["base"]})
    source = (ROOT / "scripts/puzzle_build.py").read_text()
    assert 'self.config["arm"]["turn_on_table"] and not self.ahead and self._turn(' in source      # the one place a turn starts


def test_the_scan_looks_from_the_sides_only_when_the_view_from_above_leaves_a_piece_in_doubt(tmp_path):
    def scanned(doubt_from_above, perception="sam3", seen_from_above=("plate", "bar", "cap"), seen_beyond=()):
        cell = object.__new__(build.Build)
        cell.args, cell.said, cell.combined = type("Args", (), {"perception": perception})(), [], []
        cell.config = {"scan": {"beyond": [{"name": "beyond_right", "eye_m": [0.2, -0.4, 0.7], "look_at_m": [0.2, -0.4, 0.0], "roll_deg": 0.0}]}}
        cell.names, cell.base, cell.placed = ["plate", "bar", "cap"], "plate", []
        cell.say, cell.cell = cell.said.append, type("Arm", (), {"camera_to": lambda self, eye: cell.said.append("back above")})()
        cell._views = lambda: [("top", 0), ("left", 1), ("right", 2)]
        cell._sweep = lambda folder, views, taken: taken.extend({"name": name} for name, eye in views)
        folder = tmp_path / f"scan_{len(list(tmp_path.iterdir()))}"

        def combine(given):
            (given / "answer").mkdir(parents=True)
            cell.combined.append(sorted(path.name for path in given.iterdir()))
            if given.name == "beyond_right":                   # the cycle on the view beyond the picture: what lies there, and a piece seen from above too
                return {"doubtful": ["cap", "bar"], "pieces": [{"name": name, "from": "beyond"} for name in (*seen_beyond, "plate")]}
            return {"doubtful": doubt_from_above if len(cell.combined) == 1 else [], "views": len(cell.combined),
                    "pieces": [{"name": name, "from": "top"} for name in seen_from_above]}

        cell._combine = combine
        return cell, *cell._seen(folder)

    cell, report, views = scanned([])
    assert views == ["top"] and report["views"] == 1 and cell.said == []          # nothing in doubt: one view, one pass of the cycle
    cell, report, views = scanned(["bar"])
    assert views == ["top", "left", "right"] and (report["doubtful"], report["views"]) == ([], 2)
    assert cell.combined[1] == ["answer", "answer_from_above"] and "bar in doubt from above" in cell.said[0] and cell.said[-1] == "back above"
    # Doubt the build need not act on takes no more views: another puzzle's piece, a piece in its slot, the base it lies in.
    assert scanned(["peg"])[2] == ["top"]
    cell.placed = ["bar"]
    assert cell._in_doubt({"doubtful": ["plate", "bar", "cap", "peg"]}) == ["cap"] and cell._in_doubt({}) == []
    cell.placed = []
    assert cell._in_doubt({"doubtful": ["plate", "bar"]}) == ["bar", "plate"]      # a bare base is still to be slid: its doubt counts
    # A piece of the puzzle the view from above does not find is looked for beyond its picture, and joins the report.
    cell, report, views = scanned([], seen_from_above=("plate", "bar"), seen_beyond=("cap",))
    assert views == ["top", "beyond_right"] and "cap not in the picture; looking from above (beyond_right)" in cell.said[0]
    assert [(piece["name"], piece["from"]) for piece in report["pieces"]] == [("plate", "top"), ("bar", "top"), ("cap", "beyond")]
    assert report["doubtful"] == ["cap"] and len(cell.combined) == 2             # its doubt comes with it, the others' does not
    cell, report, views = scanned([], seen_from_above=("plate", "bar"))
    assert views == ["top", "beyond_right"] and [piece["name"] for piece in report["pieces"]] == ["plate", "bar"]      # not there either: the scan will say it is missing
    cell, report, views = scanned(["bar"], perception="oracle")
    assert (report, views, cell.combined) == ({}, ["top"], [])                    # the simulator's own poses: no cycle and no doubt


def test_a_symmetric_base_is_first_read_the_same_way_round_whichever_answer_the_matcher_gives():
    half_turn = (scan.yaw_matrix(math.pi), np.zeros(3))
    cell = object.__new__(build.Build)
    cell.base, cell.order = "plate", [{"piece": "bar"}]
    cell.pack = type("Pack", (), {"shape": lambda self, name: type("Shape", (), {"symmetries": [(np.eye(3), np.zeros(3)), half_turn]})()})()
    cell.slots = {"bar": assembly.pose(np.eye(3), [0.0, 0.025, 0.012])}        # the first piece's slot, off the base's middle
    one = assembly.pose(scan.yaw_matrix(0.3), [0.0, -0.46, 0.0])
    other = one @ assembly.pose(*half_turn)
    for read in (one, other):
        taken = cell._first_reading(read)
        assert np.allclose(taken, one) and (taken @ cell.slots["bar"])[1, 3] > (other @ cell.slots["bar"])[1, 3]


def test_a_piece_that_goes_in_level_has_its_base_looked_at_before_it_is_picked_with_the_hand_empty():
    step_log = sys.modules["step_log"]
    pack = scan.Pack(TOY, {"plate": [*TOY_SLOTS["plate"], {"piece": "cap", "position_m": [0.0, 0.0, 0.029], "rpy_deg": [0.0, 0.0, 0.0]}]}, "toy/v1")
    plate = assembly.pose(np.eye(3), [0.0, -0.46, pack.shape("plate").rests[0].drop])
    slid = plate.copy()
    slid[0, 3] += 0.004

    def before_the_pick(verdicts, approach_axis=(1, 0, 0)):
        cell = object.__new__(build.Build)
        cell.pack, cell.base, cell.slots = pack, "plate", assembly.slot_poses(pack, "plate")
        cell.config = {"grasps": {"cap": dict(TOY_GRASP, approach_axis=list(approach_axis))},
                       "base_check": {"corrections": 2, "look": {"offset_m": [-0.1, 0.05], "height_m": 0.45, "roll_deg": -120.0}}}
        cell.estimate, cell.base_goal, cell.placed = {"plate": plate, "bar": plate @ cell.slots["bar"], "cap": plate}, plate, ["bar"]
        cell.base_log, cell.said, cell.moves, cell.put_back, cell.unsure = {"corrections": []}, [], [], "cap", "an old doubt"
        cell.say, cell.log = cell.said.append, step_log.StepLog(lambda: {"sim_s": 0.0, "frame": 0, "sample": 0, "spent": {}})
        cell.cell = type("Arm", (), {"camera_to": lambda self, eye: cell.moves.append("camera_to")})()
        cell._shift_base = lambda piece, where: None
        left = list(verdicts)
        cell._base_check = lambda piece, where: {"verdict": left.pop(0), "pose": slid, "moved_mm": 4.0, "turned_deg": 0.0, "why": "too little of it in view"}
        cell._look_first("cap")
        return cell

    cell = before_the_pick(["steady"])
    assert cell.moves == ["camera_to"] and (cell.unsure, cell.put_back) == (None, None) and "before it is picked" in cell.said[0]
    assert cell._look_check("cap") is True                     # nothing left in doubt: it may be carried in, and nothing moves for that
    cell = before_the_pick(["moved", "steady"])                # a base found slid by a trusted fit is taken where it is, before anything is planned
    assert np.array_equal(cell.estimate["plate"], slid) and np.allclose(cell.estimate["bar"], slid @ cell.slots["bar"]) and cell.unsure is None
    assert cell.base_log["corrections"][0]["where"] == "look" and [step["kind"] for step in cell.log.steps] == ["look", "correct"]
    cell = before_the_pick(["unsure"])                         # not trusted: kept, and the piece is put back when it is to be carried in
    cell._set_down = lambda piece, why: cell.said.append(f"set down: {why}")
    assert cell.unsure == "too little of it in view" and cell._look_check("cap") is False and cell.said[-1] == "set down: too little of it in view"
    assert before_the_pick(["moved", "moved", "moved"]).unsure == "the base keeps moving"
    cell = before_the_pick([], approach_axis=(0, 0, -1))       # a piece that goes in from above is checked at its hover, not here
    assert cell.moves == [] and cell.unsure is None


def test_a_piece_one_look_reads_just_outside_its_seat_is_read_again_by_the_scan_before_the_build_gives_up():
    step_log = sys.modules["step_log"]
    pack = scan.Pack(TOY, TOY_SLOTS, "toy/v1")
    plate = assembly.pose(np.eye(3), [0.0, -0.46, pack.shape("plate").rests[0].drop])
    seat = plate @ assembly.slot_poses(pack, "plate")["bar"]

    def looked(off_mm, attempts=2):
        cell = object.__new__(build.Build)
        cell.pack, cell.base, cell.config = pack, "plate", {"seated": {"position_m": 0.0015, "rotation_deg": 2.0}}
        cell.slots, cell.names = assembly.slot_poses(pack, "plate"), ["plate", "bar", "cap"]
        cell.estimate, cell.base_goal, cell.placed, cell.said = {"plate": plate, "bar": seat}, plate, [], []
        cell.metrics, cell.say = {"bar": {"attempts": attempts}}, cell.said.append
        cell.log = step_log.StepLog(lambda: {"sim_s": 0.0, "frame": 0, "sample": 0, "spent": {}})
        read = seat.copy()
        read[0, 3] += off_mm / 1000
        cell._look = lambda label: ({"plate": plate, "bar": read}, {})
        cell.truth = lambda: {"plate": plate, "bar": seat}     # it is in its slot, whatever the look reads
        return cell

    cell = looked(1.0)
    assert cell.rescan("bar")["result"] == "placed" and cell.placed == ["bar"]
    cell = looked(1.63)                                        # as the bar of the knocked-piece case was read: 1.2 mm off in truth
    first = cell.rescan("bar")
    assert first["result"] == "changed" and first["changed"] == ["bar"] and cell.placed == [] and cell.metrics["bar"]["read_again"] is True
    with pytest.raises(build.Refused, match="not in its slot"):                 # once: a second such look is the end
        cell.rescan("bar")
    with pytest.raises(build.Refused, match="not in its slot"):                 # and a reading far from the seat is no near miss
        looked(4.0).rescan("bar")


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


def test_on_the_real_pieces_the_two_failures_only_a_lookahead_used_to_catch_are_refused():
    try:
        pack = scan.load_pack(CONFIG["pack"])
    except Exception as error:  # noqa: BLE001 - the pack is an asset outside the repository; without it there is nothing to measure against
        pytest.skip(f"the puzzles pack is not installed: {error}")
    recorded = json.loads((ROOT / "mcp/tests/puzzle_fixtures/refusals.json").read_text())["cases"]
    base, slots = CONFIG["base"], assembly.slot_poses(pack, CONFIG["base"])
    order = {step["piece"]: step for step in assembly.build_order(pack, base)}
    said = {}
    for case in recorded:
        believed = {name: np.array(at) for name, at in case["believed"].items()}
        piece, step = case["piece"], order[case["piece"]]
        others = {name: at for name, at in believed.items() if name != piece}
        fits = lambda at_the_slot, **more: assembly.choose_grasps(
            pack, piece, CONFIG["grasps"][piece], believed[piece], believed[base] @ slots[piece], others, at_the_slot, session.pad_setback,
            believed[base][:3, :3] @ np.array(step["axis"], dtype=float), step["clear_m"] + build.MARGIN_M, **more)
        # As the check was made until 2026-11-17 it found a way: the gripper only where it ends and at the gripped
        # width, and at the slot only the base and the pieces already placed.
        assert fits({name: believed[name] for name in [base, *case["placed"]]}, open_m=0.0, hover_m=0.0), case["case"]
        why = []
        assert fits(others, open_m=CONFIG["open_beyond_m"], hover_m=build.HOVER_M, refused=why) == [], case["case"]
        said[case["case"]] = why
    blocked = [why for name, why in said.items() if "blocked approach" in name]
    taken = [why for name, why in said.items() if "slot occupied" in name]
    assert len(blocked) == len(taken) == 4
    assert all(why == ["line_red is in the open gripper's way down to line_brown"] for why in blocked)
    assert all(why == ["line_red is where line_brown goes"] for why in taken)


def stopped_build(metrics, refusals=()):
    cell = object.__new__(build.Build)
    cell.metrics, cell.refusals = metrics, list(refusals)
    return cell


def test_a_build_that_did_not_finish_says_what_stopped_it_in_its_own_words():
    seated = {"attempts": 1, "final_seated": True}
    done = stopped_build({"bar": seated, "fork": seated})
    assert done.why_stopped({"success": True, "exit_status": "success"}) is None
    # The graph's own error names its first failed node, here a question to a model that was not up, which decides nothing.
    graph = {"success": False, "exit_status": "failure", "error": "first node error -- count_parts.ask: vlm: backend unavailable"}
    abort = lambda last: {"attempts": 0, "final_seated": False, "lookahead": [{"verdict": "abort", "tries": [last]}]}
    moved = stopped_build({"bar": seated, "fork": abort({"seated": True, "position_error_mm": 0.7, "others_moved_mm": 4.1, "moved_most": "plate"})})
    assert moved.why_stopped(graph) == "the lookahead said abort before fork: its rehearsal seated the piece and moved plate 4.1 mm"
    jammed = stopped_build({"bar": seated, "fork": abort({"seated": False, "position_error_mm": 133.1, "others_moved_mm": 36.0})})
    assert jammed.why_stopped(graph) == "the lookahead said abort before fork: its rehearsal left the piece 133.1 mm from its slot"
    misfit = stopped_build({"bar": abort({"seated": False, "misfit": "the place of bar corrected from the lookahead does not fit: the gripper would be in plate at the slot"})})
    assert misfit.why_stopped(graph).endswith("does not fit: the gripper would be in plate at the slot")
    refused = stopped_build({"bar": seated, "fork": {"attempts": 1, "final_seated": False}},
                            [{"tool": "approach", "piece": "bar", "why": "an earlier refusal the build got past"},
                             {"tool": "rescan", "piece": "fork", "why": "fork is not in its slot and cannot be picked again"}])
    assert refused.why_stopped(graph) == "rescan of fork was refused: fork is not in its slot and cannot be picked again"
    assert "vlm" not in refused.why_stopped(graph)
    # Every piece seated and the graph still ended in failure (a checkpoint, say): that is not a finished build either.
    assert stopped_build({"bar": seated}).why_stopped(graph) == "the graph ended in failure"
    assert stopped_build({"bar": seated, "fork": {"attempts": 0, "final_seated": False}}).why_stopped(graph) == "the graph ended in failure with ['fork'] not seated"


def test_the_script_exits_0_only_for_a_build_that_finished(monkeypatch, capsys):
    class NoBuild:
        def __init__(self, *args):
            pass

        def abandon(self):
            pass

    record = {"completed": True, "stopped": None, "simulated_seconds": 160.0, "wall_seconds": 260.0, "video": "build.mp4", "viewer": "viewer/index.html"}
    monkeypatch.setattr(build, "Build", NoBuild)
    monkeypatch.setattr(build, "run", lambda *args: record)
    monkeypatch.setattr(build, "scattered", lambda *args: {})
    monkeypatch.setattr(scan, "load_pack", lambda *args: None)
    monkeypatch.setattr(sys, "argv", ["puzzle_build.py", *RUN])
    build.main()                                               # finished: returns, which is exit code 0
    assert json.loads(capsys.readouterr().out)["stopped"] is None
    record.update(completed=False, stopped="rescan of fork was refused: fork is not in its slot and cannot be picked again")
    with pytest.raises(SystemExit) as stop:
        build.main()
    assert stop.value.code == 1 and "cannot be picked again" in json.loads(capsys.readouterr().out)["stopped"]
