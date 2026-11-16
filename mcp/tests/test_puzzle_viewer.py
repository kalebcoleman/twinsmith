"""CPU-only checks of the run's step log (scripts/step_log.py) and of the step viewer's builder (scripts/puzzle_viewer.py)
on a small made-up run."""

import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


step_log = load("step_log")
viewer = load("puzzle_viewer")
FPS = 5


class Clock:
    """A run's counters, advanced by hand: simulated seconds, with the film and the samples keeping pace."""

    def __init__(self):
        self.sim, self.spent, self.filming = 0.0, {}, True

    def __call__(self):
        return {"sim_s": self.sim, "frame": round(self.sim * FPS) if self.filming else 0, "sample": round(self.sim * 20), "spent": dict(self.spent)}

    def run(self, seconds, **spent):
        self.sim += seconds
        for key, value in spent.items():
            self.spent[key] = self.spent.get(key, 0) + value


class Robot:
    def __init__(self):
        self.clock = Clock()
        self.log = step_log.StepLog(self.clock)

    @step_log.stepped("lift")
    def lift(self, piece):
        self.log.note(waypoints=[{"name": "lift", "xyz": [0.1, -0.4, 0.14], "approach": [0, 0, -1], "closing": [1, 0, 0]}])
        self.clock.run(2.0, bridge_s=1.5, bridge_calls=3, physics_s=0.5, render_s=0.8)
        return {"rise_m": 0.119}

    @step_log.stepped("insert")
    def insert(self, piece):
        with self.log.step("carry", piece):
            self.clock.run(1.0, bridge_s=0.5, physics_s=0.25)
        with self.log.step("lower", piece):
            raise RuntimeError("blocked")


def test_a_step_knows_its_time_its_share_of_the_counters_and_its_children():
    robot = Robot()
    assert robot.lift("bar") == {"rise_m": 0.119}
    with pytest.raises(RuntimeError):
        robot.insert(piece="bar")
    lift, insert, carry, lower = robot.log.steps
    assert (lift["kind"], lift["piece"], lift["parent"], lift["result"]) == ("lift", "bar", None, {"rise_m": 0.119})
    assert (lift["sim_start"], lift["sim_s"], lift["frame_start"], lift["frame_end"], lift["sample_end"]) == (0.0, 2.0, 0, 10, 40)
    assert lift["spent"] == {"bridge_s": 1.5, "bridge_calls": 3, "physics_s": 0.5, "render_s": 0.8}
    assert lift["plan"]["waypoints"][0]["name"] == "lift"
    assert (carry["parent"], lower["parent"], insert["parent"]) == (insert["id"], insert["id"], None)
    assert carry["spent"] == {"bridge_s": 0.5, "physics_s": 0.25} and insert["sim_s"] == 1.0
    assert lower["error"] == insert["error"] == "blocked" and "result" not in insert
    assert robot.log.open == []


WORKFLOW = {
    "version": 3, "meta": {"name": "toy_build", "description": "Put the bar in."},
    "nodes": {"locate": {"type": "subgraph", "ref": "locate"}, "assemble_bar": {"type": "subgraph", "ref": "assemble_bar"},
              "done": {"type": "end", "status": "success"}, "abort": {"type": "end", "status": "failure"}},
    "edges": [["START", "locate"]],
    "conditional_edges": {"locate": {"router_field": "exit", "mapping": {"located": "assemble_bar", "failed": "abort"}},
                          "assemble_bar": {"router_field": "exit", "mapping": {"placed": "done", "retry": "assemble_bar", "failed": "abort"}}},
    "subgraphs": {
        "locate": {"skill": "surveying-puzzle-pieces", "nodes": {"survey": {"type": "tool", "tool": "workcell.survey"}, "located": {"type": "noop"}},
                   "edges": [["START", "survey"], ["survey", "located"], ["located", "END"]], "conditional_edges": {},
                   "exit": {"router_field": None, "success_values": ["located"]}, "on_error": "failed"},
        "assemble_bar": {"skill": "assembling-puzzle-pieces",
                         "nodes": {"look": {"type": "tool", "tool": "workcell.rescan"}, "lift": {"type": "tool", "tool": "workcell.lift"},
                                   "lookahead": {"type": "tool", "tool": "workcell.lookahead"}, "check": {"type": "script", "script": "scripts/verify_grip.py"},
                                   "placed": {"type": "noop"}, "retry": {"type": "noop"}, "aborted": {"type": "noop"}},
                         "edges": [["placed", "END"], ["retry", "END"], ["aborted", "END"], ["START", "lookahead"], ["lift", "check"], ["check", "look"]],
                         "conditional_edges": {"lookahead": {"router_field": "verdict", "mapping": {"go": "lift", "abort": "aborted"}},
                                               "look": {"router_field": "result", "mapping": {"placed": "placed", "retry": "retry"}}},
                         "exit": {"router_field": None, "success_values": ["placed", "retry", "aborted"]}, "on_error": "failed"}}}


def make_run(folder, wall=1000.0):
    """A run folder as scripts/puzzle_build.py leaves it, for a toy build: a survey, a lookahead of two tries, a lift
    and a second look. Wall time is made up: every step takes five times its simulated time and a second passes
    between steps; the graph's inner nodes start and end with their steps, and as in a real trace nothing times a
    subgraph as a whole. Returns the folder."""
    clock = Clock()
    log = step_log.StepLog(clock)
    events, tries = [], {}
    outline = lambda x: {"xyz": [x, -0.4, 0.01], "outline": [[x - 0.01, -0.44], [x + 0.01, -0.44], [x + 0.01, -0.36], [x - 0.01, -0.36]]}

    now = [wall]                                            # the wall clock, moved on by hand

    def mark(kind, name, status):
        events.append({"seq": len(events), "event_type": kind, "timestamp": now[0], "node_name": name, "status": status})

    def node(name, kind, piece=None, seconds=0.0, result=None, **plan):
        mark("node_started", name, "running")
        with log.step(kind, piece, **plan) as row:
            row["wall_start"] = now[0]
            clock.run(seconds, bridge_s=4 * seconds, bridge_calls=2, physics_s=seconds, render_s=2 * seconds)
            row["result"] = result
        row["wall_s"] = 5 * seconds
        now[0] += row["wall_s"]
        mark("node_finished", name, "ok")
        now[0] += 1.0

    node("locate.survey", "survey", seconds=2.0, result={"found": 2, "missing": []}, look=1)
    mark("node_started", "assemble_bar.lookahead", "running")
    with log.step("lookahead", "bar") as ahead:
        ahead["wall_start"] = now[0]
        for number, go in ((1, False), (2, True)):
            kept, clock.sim = clock.sim, 0.0
            with log.step("try", "bar", clip=f"bar_{number}") as attempt:
                with log.step("lift", "bar"):
                    clock.run(1.0, bridge_s=1.0, physics_s=0.5)
                attempt["result"] = {"go": go, "position_error_mm": 0.4 if go else 16.0, "others_moved_mm": 0.1 if go else 16.0}
            tries[f"bar_{number}"] = {"piece": "bar", "fps": FPS, "zero_s": 0.2, "captions": ["bar: lifting"] * 5,
                                      "samples": [[0.2 + 0.05 * k, 0.655, 1.5, -1.4, 1.7, -1.9, -1.5, 0.0, 0.04] for k in range(20)], "grips": [[0.4, 40.0]]}
            clock.sim = kept
        ahead["result"] = {"verdict": "go", "tries": [{"go": False}, {"go": True}]}
    ahead["wall_s"] = 20.0
    now[0] += 20.0
    mark("node_finished", "assemble_bar.lookahead", "ok")
    now[0] += 1.0
    node("assemble_bar.lift", "lift", "bar", 2.0, {"rise_m": 0.119}, waypoints=[{"name": "lift", "xyz": [0.1, -0.4, 0.14], "approach": [0, 0, -1], "closing": [1, 0, 0]}])
    node("assemble_bar.look", "rescan", "bar", 1.0, {"result": "placed", "position_error_mm": 0.5}, look=2)
    looks = [{"index": index, "label": label, "sim_s": 0.0, "truth": {"bar": outline(x), "plate": outline(0.0)}, "estimate": {"bar": outline(x + 0.001), "plate": outline(0.0)},
              "picture": f"look_{index:02d}_{label}.png", "masks": f"look_{index:02d}_{label}_masks.png",
              "scores": [{"name": "bar", "result": "correct", "as": "bar", "position_error_mm": 1.0, "rotation_error_deg": 0.2}]}
             for index, label, x in ((1, "survey", 0.1), (2, "bar", 0.0))]
    frames = round(clock.sim * FPS)
    folder = Path(folder)
    (folder / "graph").mkdir(parents=True)
    (folder / "trace").mkdir()
    (folder / "run_log.json").write_text(json.dumps({
        "steps": log.steps, "looks": looks, "tries": tries, "grips": [[2.4, 40.0], [4.0, 0.0]],
        "samples": [[0.2 + 0.05 * k, 0.655, 1.5 + 0.01 * k, -1.4, 1.7, -1.9, -1.5, 0.0, 0.05 - 0.0002 * k] for k in range(100)],
        "film": {"fps": FPS, "zero_s": 0.2, "cameras": ["wrist", "table"], "captions": ["working"] * frames},
        "sample_columns": ["seconds", "rail_m", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "pad_gap_m"],
        "wall_start": wall, "wall_s": 60.0}))
    (folder / "metrics.json").write_text(json.dumps({
        "seed": 7, "completed": True, "simulated_seconds": 5.0, "wall_seconds": 60.0, "perception": "sam3", "order": [{"piece": "bar"}],
        "pieces": {"bar": {"final_seated": True}}, "graph": {"run": {"success": True, "exit_status": "success", "error": None,
                                                                  "checkpoints": [{"name": "bar_seated", "subgraph": "assemble_bar", "passed": True}]}, "checks": None}}))
    (folder / "graph/workflow.json").write_text(json.dumps(WORKFLOW))
    (folder / "trace/dag_trace.json").write_text(json.dumps({"nodes": [], "edges": [], "events": events}))
    return folder


def test_the_graph_is_drawn_in_the_order_a_run_meets_it():
    drawn = viewer.graph(WORKFLOW)
    assert [node["name"] for node in drawn["nodes"]] == ["locate", "assemble_bar", "done", "abort"]
    bar = drawn["nodes"][1]
    assert bar["skill"] == "assembling-puzzle-pieces" and [child["name"] for child in bar["children"]] == ["lookahead", "lift", "check", "look"]
    assert bar["children"][0] == {"name": "lookahead", "type": "tool", "does": "workcell.lookahead", "routes": {"go": "lift", "abort": "aborted"}}
    assert bar["children"][2]["does"] == "scripts/verify_grip.py" and bar["exits"] == ["placed", "retry", "aborted", "failed"]
    assert {"from": "assemble_bar", "to": "assemble_bar", "label": "retry"} in drawn["edges"] and drawn["edges"][0] == {"from": "START", "to": "locate", "label": None}


def test_the_page_data_puts_steps_nodes_and_checkpoints_on_the_films_time(tmp_path):
    page = viewer.data(make_run(tmp_path / "run"))
    assert page["title"] == "run" and page["duration"] == 5.0 and [view["name"] for view in page["views"]] == ["table", "wrist"]
    kinds = [(step["kind"], step["depth"], step["try"], step["t0"], step["t1"]) for step in page["steps"]]
    assert kinds == [("survey", 0, None, 0.0, 2.0), ("lookahead", 0, None, 2.0, 2.0), ("try", 1, "bar_1", 0.0, 1.0), ("lift", 2, "bar_1", 0.0, 1.0),
                     ("try", 1, "bar_2", 0.0, 1.0), ("lift", 2, "bar_2", 0.0, 1.0), ("lift", 0, None, 2.0, 4.0), ("rescan", 0, None, 4.0, 5.0)]
    assert page["steps"][0]["clip"] == "media/steps/000_survey.mp4" and page["steps"][1]["clip"] is None and page["steps"][3]["clip"] is None
    assert page["tries"]["bar_1"]["src"] == "media/lookahead/bar_1.mp4" and page["tries"]["bar_1"]["duration"] == 1.0
    assert page["tries"]["bar_1"]["samples"][0][0] == 0.0 and page["tries"]["bar_1"]["grips"] == [[0.2, 40.0]]
    assert page["samples"][0] == [0.0, 655.0, 85.94, -80.21, 97.4, -108.86, -85.94, 0.0, 50.0] and page["grips"][0] == [2.2, 40.0]
    assert page["columns"][1:3] == ["rail, mm", "shoulder_pan, deg"]
    spans = page["spans"]
    assert spans["locate.survey"] == [[0.0, 2.0, "ok"]] and spans["assemble_bar.lookahead"] == [[2.0, 2.0, "ok"]]       # a lookahead is a moment of the film
    assert spans["assemble_bar.lift"] == [[2.0, 4.0, "ok"]]
    assert spans["locate"] == [[0.0, 2.0, "ok"]] and spans["assemble_bar"] == [[2.0, 5.0, "ok"]]                    # a subgraph spans its inner nodes
    assert spans["done"] == [[5.0, 5.0, "ok"]] and "abort" not in spans                                         # the end the run reached
    assert [(look["index"], look["t"], look["masks"]) for look in page["looks"]] == [(1, 2.0, "media/looks/look_01_survey_masks.png"), (2, 5.0, "media/looks/look_02_bar_masks.png")]
    assert page["checkpoints"] == [{"name": "bar_seated", "subgraph": "assemble_bar", "passed": True, "t": 5.0, "after": 7}]
    assert page["panels"] == list(viewer.PANELS) and json.loads(json.dumps(page)) == page


def test_the_time_breakdown_accounts_for_every_second_of_a_step(tmp_path):
    page = viewer.data(make_run(tmp_path / "run"))
    timing = page["timing"]
    by_kind = {row["kind"]: row for row in timing["rows"]}
    assert by_kind["lift"] == {"kind": "lift", "count": 1, "wall_s": 10.0, "sim_s": 2.0, "bridge_calls": 2, "physics": 2.0, "rendering": 4.0,
                               "bridge": 2.0, "sam3": 0.0, "scan": 0.0, "planning": 0.0, "other": 2.0}
    assert by_kind["lookahead"]["wall_s"] == 20.0 and by_kind["lookahead"]["physics"] == 1.0 and by_kind["lookahead"]["other"] == 18.0
    for row in (*timing["rows"], timing["total"]):
        assert sum(row[name] for name in timing["categories"]) == pytest.approx(row["wall_s"], abs=0.2)
    assert timing["total"]["wall_s"] == 45.0 and timing["slower_than_real_time"] == 12.0
    assert timing["outside_steps"] == {"vlm_s": 0.0, "graph_and_other_s": 15.0}


def test_the_viewer_is_built_with_its_page_data_and_clips(tmp_path):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        pytest.skip("no ffmpeg on PATH to encode the clips with")
    from PIL import Image

    run = make_run(tmp_path / "run")
    for folder, count in (("film", 25), ("lookahead/bar_1", 5), ("lookahead/bar_2", 5)):
        for camera, colour in (("table", (40, 90, 160)), ("wrist", (160, 90, 40))):
            (run / folder / camera / "rgb").mkdir(parents=True)
            for index in range(count):
                Image.new("RGB", (160, 90), tuple(min(255, value + 4 * index) for value in colour)).save(run / folder / camera / "rgb" / f"{index:05d}.jpg")
    for name in ("look_01_survey", "look_02_bar"):
        for suffix in ("", "_masks"):
            Image.new("RGB", (160, 90), (90, 90, 90)).save(run / f"{name}{suffix}.png")
    page = viewer.build(run, ffmpeg=ffmpeg)
    out = page.parent
    assert page.name == "index.html" and (out / "viewer.js").is_file() and (out / "viewer.css").is_file()
    text = (out / "run.js").read_text()
    assert text.startswith("window.RUN = ") and json.loads(text[len("window.RUN = "):].rstrip(";\n"))["title"] == "run"
    made = sorted(str(path.relative_to(out / "media")) for path in (out / "media").rglob("*.mp4"))
    assert made == ["lookahead/bar_1.mp4", "lookahead/bar_2.mp4", "steps/000_survey.mp4", "steps/006_lift.mp4", "steps/007_rescan.mp4", "table.mp4", "wrist.mp4"]
    probe = subprocess.run([ffmpeg, "-hide_banner", "-i", str(out / "media/steps/006_lift.mp4")], capture_output=True, text=True).stderr
    assert "1280x360" in probe and "5 fps" in probe                                # both cameras side by side, at the simulation's speed
    assert (out / "media/looks/look_02_bar_masks.png").is_file()
