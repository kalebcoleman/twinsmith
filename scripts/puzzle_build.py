#!/usr/bin/env python3
"""Build one puzzle with the UR5e, in simulation only: scan, slide the base to the middle, pick and insert each piece.

Against a disposable Isaac Sim bridge on Newton, with the work cell (``ur5e-workcell``) and the ``puzzles`` pack:

1. A random layout: one puzzle's pieces lying loose on the table, its base off to the side (``puzzle_assembly.scatter``).
2. The rail is parked (its own blocking move), then the arm is solved by six-joint IK from the UR base. The scan: the
   wrist D415 looks straight down and then from two sides, and the multi-view cycle (``--scan-cycle``: SAM 3.1 masks
   the pieces, ``puzzle_scan`` names and poses them, every view's depth is checked against that) gives one estimate.
   The graph comes back to the scan when a pick closes on nothing or a look finds the table changed.
3. The build order comes from taking the CAD apart (``puzzle_assembly.build_order``). Graph-as-Policy runs it as a
   graph (``exts/workcell/gap``): this script serves the work cell's skills to that graph, one call at a time.
4. The base is gripped by its centre island and slid to the middle. Each piece is picked, carried over its slot,
   lowered along the slot axis, released, and looked at again from the survey pose. A piece does not sit in the hand
   exactly as its grasp is defined, so the place is corrected by how it does sit (``--correction``): from a table
   with one measured shift per piece type, for the piece types the settings name (``table``,
   ``puzzle_build_corrections.json``; nothing is simulated ahead), or by simulating each pick ahead on Newton and reading it there (``lookahead``: the stage's state is saved, the
   pieces are put where the scan says they are, the pick and the insertion are run, the result is judged go or abort
   and the state is put back), or not at all (``none``).
5. ``metrics.json``: per piece, whether it was named, gripped and seated, how far its final pose is from the CAD's,
   which correction its place had and what a lookahead said; a video from the table camera and the wrist camera, and
   key frames.

Nothing here reaches a real robot: every call goes to the simulator bridge on this machine. It replaces the stage and
starts GPU jobs (SAM 3.1) beside the simulator.

    python3 scripts/puzzle_build.py --port <port> --out <folder> --seed 1 --mutate-stage --discard-changes \
        --allow-concurrent-gpu --scan-cycle <mv_cycle.sh> [--table-camera] [--colours <colours.json>] [--perception oracle]
        [--unqualified-cell-pack] [--correction table|lookahead|none] [--disturb PIECE]
"""

import argparse
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
import puzzle_assembly as assembly  # noqa: E402
import puzzle_base_check  # noqa: E402
import puzzle_scan  # noqa: E402
import puzzle_viewer  # noqa: E402
import workcell_session as cell  # noqa: E402
from step_log import StepLog, stepped  # noqa: E402
from twinsmith import gap  # noqa: E402
from twinsmith.isaaclab import jobs  # noqa: E402
from twinsmith.robosim.bridge import _ffmpeg_exe  # noqa: E402
from twinsmith.sam3 import Sam3  # noqa: E402

CONFIG = Path(__file__).with_name("puzzle_build.json")
CORRECTIONS = Path(__file__).with_name("puzzle_build_corrections.json")
MEASURE_LAYOUTS, MEASURE_LEAST = 6, 3            # a table is measured on this many layouts; an entry needs this many readings
HALF_TURN = np.diag([-1.0, -1.0, 1.0, 1.0])     # the gripper turned half round about the way it points
REGISTRY = ROOT / "exts/workcell/gap"
PUZZLE = "/World/Puzzle"
HOVER_M, LIFT_M, CARRY_M, RETREAT_M, MARGIN_M = 0.10, 0.12, 0.15, 0.08, 0.02
LOOKAHEAD_TRIES = 5
GRASP_KEYS = ("point_m", "closing_axis", "approach_axis", "width_m")
BASE_MOVED_M, BASE_MOVED_DEG = 0.003, 2.0       # a later look that puts the base further than this from where it was: it has moved
BASE_PLACED_M, BASE_PLACED_DEG = 0.010, 5.0     # the base is in the middle when a look finds it this near its goal
BASE_SLIDES, BASE_NEAR_M = 2, 0.05              # it is slid at most this many times; again only from this near its goal
TINTS = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230)]


class Refused(Exception):
    """A skill cannot do what was asked; the graph routes on it."""


class Misfit(Refused):
    """A place corrected for how the piece sits in the hand puts the gripper into something at the slot."""


def shifted(at, offset):
    """A pose moved by a world offset."""
    out = np.array(at, dtype=float)
    out[:3, 3] += offset
    return out


def waypoint(name, at):
    """A gripper pose of a plan as the run's log keeps it: where the pad centres' frame is, and which way the
    gripper points and closes."""
    return {"name": name, "xyz": [round(float(v), 5) for v in at[:3, 3]], "approach": [round(float(v), 4) for v in at[:3, 2]],
            "closing": [round(float(v), 4) for v in at[:3, 0]]}


def se3(at):
    """A pose as Graph-as-Policy's Se3Pose."""
    w, x, y, z = cell.matrix_quat(at[:3, :3])
    return {"position": dict(zip("xyz", map(float, at[:3, 3]))), "orientation": {"w": w, "x": x, "y": y, "z": z}}


def piece_types(pack, config, pieces):
    """``pieces`` grouped into types: pieces of one shape that are gripped by one grasp. [[piece, ...], ...]."""
    same_shape = lambda a, b: len(pack.shape(a).boxes) == len(pack.shape(b).boxes) and all(
        np.allclose(first[0], second[0]) and np.allclose(first[1], second[1]) for first, second in zip(pack.shape(a).boxes, pack.shape(b).boxes))
    grasp = lambda piece: {key: config["grasps"][piece][key] for key in GRASP_KEYS}
    types = []
    for piece in pieces:
        for group in types:
            if grasp(group[0]) == grasp(piece) and same_shape(group[0], piece):
                group.append(piece)
                break
        else:
            types.append([piece])
    return types


def read_corrections(path, config):
    """The table of in-hand corrections: ``{"pieces": {piece: {"in_hand", "flipped"}}, "friction": ...}``. ``in_hand``
    is 4 x 4, the gripper as a piece of that type sits in it, in the frame of the gripper as the grasp defines it;
    ``flipped`` whether that was measured with the gripper half round; ``friction`` the solver's coefficients it was
    measured at. A correction belongs to the pack, the grip force, the grasp and the pieces it was measured for: a
    table measured for others is refused. The pieces are the ones the settings name (``corrected_from_table``); every
    other piece is placed by its grasp as defined."""
    table = json.loads(Path(path).read_text())
    measured, wanted = table["measured"], sorted(config["corrected_from_table"]["pieces"])
    again = "Measure it again (--write-corrections)."
    if measured["pack"] != config["pack"] or measured["grip_force_n"] != config["grip_force_n"]:
        raise SystemExit(f"{path} was measured with {measured['pack']} at {measured['grip_force_n']} N; this build is of {config['pack']} at "
                         f"{config['grip_force_n']} N. {again}")
    if sorted(piece for entry in table["entries"] for piece in entry["pieces"]) != wanted:
        raise SystemExit(f"{path} has entries for {[entry['pieces'] for entry in table['entries']]}; the settings want {wanted} corrected. {again}")
    out = {}
    for entry in table["entries"]:
        for piece in entry["pieces"]:
            grasp = {key: config["grasps"][piece][key] for key in GRASP_KEYS}
            if entry["grasp"] != grasp:
                raise SystemExit(f"{path}: the correction for {piece} was measured with the grasp {entry['grasp']}, not with the configured "
                                 f"{grasp}. {again}")
            out[piece] = {"in_hand": np.array(entry["in_hand"], dtype=float), "flipped": bool(entry["flipped"])}
    return {"pieces": out, "friction": measured["friction_in_the_solver"]}


def mean_shift(shifts):
    """The mean of 4 x 4 in-hand shifts: the mean position and the mean rotation."""
    out = np.eye(4)
    out[:3, :3] = Rotation.from_matrix(np.array([shift[:3, :3] for shift in shifts])).mean().as_matrix()
    out[:3, 3] = np.mean([shift[:3, 3] for shift in shifts], axis=0)
    return out


def in_hand_shift(shift):
    """How far a piece sits in the hand from its grasp as defined: (millimetres, degrees) of a 4 x 4 shift."""
    return (round(1000 * float(np.linalg.norm(shift[:3, 3])), 3), round(puzzle_scan.angle_between(shift[:3, :3], np.eye(3)), 3))


class Build:
    """The work cell's skills for one layout. Each public method without a leading underscore that ``TOOLS`` names is
    one tool of the graph: it takes and returns plain data."""

    TURN_LIFT_M = 0.03                                      # how high a piece is lifted to be turned on the table
    TURN_LOOK_DEG = 3.0                                     # a look that reads a turned piece further than this from the gripper's turn is believed

    TOOLS = ("scan", "survey", "picture", "place_base", "rescan", "lookahead", "approach", "close", "lift", "insert", "release", "state")

    def __init__(self, args, config, pack):
        self.args, self.config, self.pack = args, config, pack
        self.base, self.out = config["base"], Path(args.out).resolve()
        self.cell = cell.Session(args.port, self.out)
        self.order = assembly.build_order(pack, self.base)
        self.slots = assembly.slot_poses(pack, self.base)
        self.names = [self.base, *self.slots]
        self.estimate, self.placed, self.held, self.looks, self.marks, self.adjust, self.avoid = {}, [], None, 0, [], {}, {}
        self.metrics = {name: {"attempts": 0} for name in self.slots}
        self.correction = args.correction                   # where a place's in-hand correction comes from: lookahead, table or none
        corrections = read_corrections(args.corrections, config) if self.correction == "table" else {"pieces": {}, "friction": None}
        self.table, self.table_friction = corrections["pieces"], corrections["friction"]
        self.base_slides = 0                                # how many times the base has been slid to the middle
        self.ahead = False                                  # true while simulating ahead: no metric is recorded
        self.log, self.look_log, self.tries = StepLog(self.cell.clock), [], {}
        self.refusals = []                                  # what the graph was refused, in order: [{"tool", "piece", "why"}]
        self.started, self.scans, self.disturbed = time.time(), 0, False
        self.base_log, self.shifted, self.put_back, self.checker, self.unsure = {"checks": [], "shifts": [], "corrections": []}, {}, None, None, None

    # ---- the world --------------------------------------------------------------------------------
    def truth(self):
        """The simulator's pose of every piece. A symmetric base has more than one true pose; the one given is the
        one nearest the robot's own reading, so the two name the slots alike."""
        found = self.cell.poses(self.paths.values())
        out = {name: found[path] for name, path in self.paths.items()}
        if self.base in self.estimate:
            out[self.base] = assembly.nearest(self.pack, self.base, self.estimate[self.base], out[self.base])
        return out

    def start(self, layout):
        """Build the cell with the layout, park the rail and start filming."""
        self.paths = {name: f"{PUZZLE}/{name}" for name in layout}      # all that lies on the table: another puzzle's pieces are seen and kept clear of
        with self.log.step("build stage"):
            task = puzzle_scan.write_task(assembly.layout_estimate(self.pack, layout), self.out / "task", "puzzle_build.py, a random layout")
            built = self.cell.build(task, self.config["start_joint_deg"], 0.05, self.args.discard_changes)
            name, qualified = assembly.cell_pack(built.get("pack"))
            if not qualified and not self.args.unqualified_cell_pack:
                raise SystemExit(f"The cell was built from {name}, but the gripper and camera keep-out boxes, what the gripper hides from the camera and "
                                 f"the measured runs are of {assembly.KEEP_OUT_PACK}: a grasp checked here may put the camera into a piece. "
                                 "Pass --unqualified-cell-pack to run anyway; metrics.json then says so. The stage has been replaced.")
            if self.args.table_camera:
                self.cell.add_camera("table", self.config["table_camera"])
            self.cell.start_film(self.out / "film", fps=5)
        with self.log.step("park rail", rail_mm=self.config["rail_mm"]) as row:
            self.say("parking the rail")
            row["result"] = rail = self.cell.park_rail(self.config["rail_mm"])
        friction = self.friction()
        if self.table and friction != self.table_friction:
            raise SystemExit(f"The table of corrections ({self.args.corrections}) was measured with the solver's friction at "
                             f"{self.table_friction}; this cell's is {friction}. How a piece sits in the hand depends on it: measure the "
                             "table again (--write-corrections), or run with --correction lookahead or none. The stage has been replaced.")
        return {"backend": built["backend"], "cell_pack": built["pack"], "cell_pack_qualified": qualified,
                "keep_out_measured_on": assembly.KEEP_OUT_PACK, "rigid_bodies": built["newton"]["rigid_bodies"], "rail": rail,
                "friction_in_the_solver": friction}

    def friction(self):
        """The friction coefficients Newton's own model holds for the pieces' colliding shapes and the fingertip pads
        (not what the stage says): {"pieces": {piece: [coefficients]}, "pads": [coefficients]}."""
        pads = [f"{finger}/fingertip" for finger in self.cell.fingers]
        found = self.cell.friction([*self.paths.values(), *pads])
        return {"pieces": {name: found[path] for name, path in self.paths.items()}, "pads": sorted({value for pad in pads for value in found[pad]})}

    def say(self, caption):
        self.doing = self.cell.caption = caption
        if not self.ahead:
            print(f"  [{self.cell.steps / cell.PHYSICS_HZ:6.1f} s sim, {time.time() - self.started:5.0f} s wall] {caption}", flush=True)

    def mark(self, label):
        """Remember this moment of the film as a key frame."""
        if not self.ahead and self.cell.film:
            self.cell.step(self.cell.film["every"])
            self.marks.append((label, len(self.cell.film["captions"]) - 1))

    # ---- perception -------------------------------------------------------------------------------
    def _look(self, label):
        """Look straight down from the survey pose and return what is on the table: {piece: pose} and the report."""
        survey, self.looks = self.config["survey"], self.looks + 1
        self.cell.open_to(0.0)
        eye = cell.pose(np.diag([1.0, -1.0, -1.0]), np.array(survey["look_at_m"]) + [0.0, 0.0, survey["height_m"]])
        self.cell.camera_to(eye)
        truth = self.truth()
        name = f"look_{self.looks:02d}_{label}"
        outlines = lambda poses: {piece: {"xyz": [round(float(v), 5) for v in at[:3, 3]],
                                          "outline": np.round(assembly.footprint(self.pack, piece, at), 5).tolist()} for piece, at in poses.items()}
        record = {"index": self.looks, "label": label, "sim_s": self.cell.steps / cell.PHYSICS_HZ, "truth": outlines(truth)}
        self.look_log.append(record)
        self.log.note(look=self.looks)
        if self.args.perception == "oracle":
            record.update(estimate=outlines(truth), oracle=True)
            return truth, {"perception": "oracle: the simulator's own poses"}
        clip = self.cell.record("wrist", self.out / "looks" / f"{self.looks:02d}_{label}", frames=2)
        run = f"{self.out.name}-{self.looks:02d}-{label}-{time.strftime('%H%M%S')}"
        with self.cell.timed("sam3_s"):
            sam3 = Sam3(job_status=jobs.runs_status)
            started = sam3.call("track", source=str(clip), name=run, prompts=list(self.config["sam3_prompts"]),
                                allow_concurrent_gpu=self.args.allow_concurrent_gpu)
            if not started.get("ok"):
                raise SystemExit(f"sam3 refused: {started.get('error')} {started.get('hint', '')}")
            while jobs.runs_status(started["job_id"])["state"] == "running":
                time.sleep(3)
            report = sam3.call("report", name=run)
            if report.get("state") != "finished":
                raise SystemExit(f"sam3 failed: {json.dumps(report)[:600]}")
        with self.cell.timed("scan_s"):
            scan = puzzle_scan.Scan(clip, report["run_dir"])
            estimate = puzzle_scan.match(scan, self.pack)
            estimate.update(clip=str(clip), run=report["run_dir"], score=puzzle_scan.score(estimate, {n: p.tolist() for n, p in truth.items()}, self.pack))
        (clip / "estimate.json").write_text(json.dumps(estimate, indent=1) + "\n")
        self._overlay(clip, Path(report["run_dir"]), self.out / f"{name}_masks.png")
        shutil.copy(clip / "rgb/00000.png", self.out / f"{name}.png")
        seen = {piece["name"]: cell.pose(np.array(piece["rotation"]), piece["position_m"]) for piece in estimate["pieces"]}
        record.update(estimate=outlines(seen), picture=f"{name}.png", masks=f"{name}_masks.png", unexplained=len(estimate.get("unexplained", [])),
                      scores=[{key: row.get(key) for key in ("name", "result", "as", "position_error_mm", "rotation_error_deg")}
                              for row in estimate["score"]["pieces"]])
        return seen, estimate

    @staticmethod
    def _overlay(clip, run, out):
        """The look's colour picture with every SAM 3.1 object tinted."""
        picture = np.asarray(Image.open(clip / "rgb/00000.png").convert("RGB")).astype(np.float32)
        count = 0
        for folder in sorted(p for p in run.iterdir() if p.is_dir() and any(p.glob("labels_*.png"))):
            labels = np.asarray(Image.open(sorted(folder.glob("labels_*.png"))[0]))
            for number in np.unique(labels[labels > 0]):
                mask = labels == number
                picture[mask] = 0.5 * picture[mask] + 0.5 * np.array(TINTS[count % len(TINTS)])
                count += 1
        Image.fromarray(picture.astype(np.uint8)).save(out)

    # ---- the graph's tools ------------------------------------------------------------------------
    @stepped("survey")
    def survey(self):
        """Find every piece of the puzzle on the table."""
        self.say("survey: wrist camera, SAM 3.1, scan to scene")
        self.estimate, report = self._look("survey")
        missing = [name for name in self.names if name not in self.estimate]
        for row in (report.get("score") or {}).get("pieces", []):
            if row["name"] in self.metrics:
                self.metrics[row["name"]].update(identified=row["result"] == "correct", identified_as=row.get("as"),
                                                 survey_position_error_mm=row.get("position_error_mm"),
                                                 survey_rotation_error_deg=row.get("rotation_error_deg"))
        self.survey_report = {"found": len(self.estimate), "pieces": sorted(self.estimate), "missing": missing, "unexplained": len(report.get("unexplained", [])),
                              "position_error_mm": (report.get("score") or {}).get("position_error_mm")}
        self.mark("survey")
        if missing:
            raise Refused(f"the survey did not find {missing}")
        return self.survey_report

    # ---- the scan: several views, one estimate, the twin rebuilt from it ---------------------------
    SCAN_MOVED_M, SCAN_MOVED_DEG = 0.005, 5.0       # a piece this far from where the last scan put it: the table has changed
    BASE_LINK_FROM_BASE = np.diag([-1.0, -1.0, 1.0, 1.0])     # ROS base_link is the UR controller's Base turned half round about z

    @staticmethod
    def _view(eye_m, look_at_m, roll_deg=0.0):
        """World from the wrist camera's optical frame for a view: where the camera is, the point it looks at, and its
        turn about the line of sight (0: the picture's right as near the world's x as the view allows). The turn
        changes nothing of what is in the middle of the picture; it sets how far wrist 3 has to turn to see it, and
        which side of the table the camera's own gripper hides."""
        forward = np.subtract(look_at_m, eye_m)
        forward = forward / np.linalg.norm(forward)
        right = np.array([1.0, 0.0, 0.0]) - forward[0] * forward
        right, roll = right / np.linalg.norm(right), math.radians(roll_deg)
        right = math.cos(roll) * right + math.sin(roll) * np.cross(forward, right)
        return cell.pose(np.column_stack([right, np.cross(forward, right), forward]), eye_m)

    def _views(self):
        """The scan's views, the survey's own first (the matcher reads the first): [(name, world from the wrist
        camera's optical frame)]."""
        survey = self.config["survey"]
        top = cell.pose(np.diag([1.0, -1.0, -1.0]), np.array(survey["look_at_m"]) + [0.0, 0.0, survey["height_m"]])
        return [("top", top), *((view["name"], self._view(view["eye_m"], view["look_at_m"], view.get("roll_deg", 0.0))) for view in self.config["scan"]["views"])]

    def _sweep(self, folder, views, taken):
        """Stop at each of ``views`` ([(name, world from camera)], numbered on from the views already ``taken``) and,
        when there is a camera to read, take its pictures there. With pictures, ``folder`` becomes what a scan mover
        writes on the robot's side (``scan_mover.multiview/1``: a manifest and one stationary capture per view, those
        taken before included). Adds to ``taken``."""
        frames, base = self.config["scan"]["frames"], self.cell.world_from_base
        self.cell.open_to(0.0)
        for number, (name, eye) in enumerate(views, start=len(taken)):
            self.say(f"scan: view {name}")
            self.cell.camera_to(eye)
            if self.args.perception == "oracle":
                taken.append({"name": name})
                continue
            clip = self.cell.record("wrist", folder / "views" / f"{number:02d}_{name}", frames=frames)
            camera = json.loads((clip / "camera.json").read_text())
            now = self.cell.joints()
            joints, seen = [now[joint] for joint in cell.ARM], np.array(camera["frames"][0]["world_from_camera"])
            flange = cell.ur_fk(joints)[-1]
            poses = {"controller_base_from_flange": flange, "controller_base_from_camera": np.linalg.inv(base) @ seen,
                     "base_link_from_flange": self.BASE_LINK_FROM_BASE @ flange, "base_link_from_camera": self.BASE_LINK_FROM_BASE @ np.linalg.inv(base) @ seen,
                     "world_from_camera": seen}
            intrinsics = {key: camera[key] for key in ("fx", "fy", "cx", "cy", "width", "height")}
            (clip / "intrinsics.json").write_text(json.dumps({"colour": intrinsics, "depth_scale": camera["depth_scale"]}) + "\n")
            (clip / "capture.json").write_text(json.dumps({
                "schema_version": 1, "rail_mm": self.config["rail_mm"], "world_from_base": base.tolist(), "disclosures": ["rendered by the simulator, not captured"],
                "frames": [{"index": index, "robot_before": {"joints_rad": joints}, "base_from_camera": poses["controller_base_from_camera"].tolist(),
                            "world_from_camera": frame["world_from_camera"]} for index, frame in enumerate(camera["frames"])]}) + "\n")
            taken.append({"index": number, "name": name, "folder": f"views/{number:02d}_{name}", "frames": frames, "joints_rad": joints,
                          **{key: at.tolist() for key, at in poses.items()}})
        if self.args.perception != "oracle":
            (folder / "manifest.json").write_text(json.dumps({
                "schema": "scan_mover.multiview/1", "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "state": "complete", "why": None, "command": "scan",
                "mock_robot": True, "rail_mm": self.config["rail_mm"], "joint_names": list(cell.ARM),
                "frames": {"base_link_from_controller_base": self.BASE_LINK_FROM_BASE.tolist(), "world_from_controller_base": base.tolist()},
                "camera": {"kind": "rendered", "synthetic": True, "colour": intrinsics, "depth_scale_m": camera["depth_scale"], "depth_aligned_to": "colour"},
                "capture": {"frames_per_view": frames, "written_by": "twinsmith scripts/puzzle_build.py, the scan step"}, "views": taken,
                "disclosures": ["NOT a capture: pictures rendered by the simulator (noise-free depth, exact intrinsics and camera poses)"]}, indent=1) + "\n")

    def _combine(self, folder):
        """The scan folder's views as one estimate, by the multi-view cycle ``--scan-cycle`` names (a command given a
        ``scan_mover.multiview/1`` folder, ``--out``, ``--name`` and ``--colours``, that leaves ``estimate.json`` in the
        matcher's shape with ``doubtful`` as it stands after its depth check). The cycle matches with this checkout."""
        asked = [str(self.args.scan_cycle), str(folder / "scan"), "--out", str(folder), "--name", "answer", "--no-latest",
                 *(["--colours", str(self.args.colours.resolve())] if self.args.colours else [])]
        with self.cell.timed("scan_s"):
            done = subprocess.run(asked, env={**os.environ, "TWINSMITH_REPO": str(ROOT), "TWINSMITH_SCAN_REPO": str(ROOT)}, capture_output=True, text=True)
        (folder / "cycle.log").write_text(done.stdout + done.stderr)
        if done.returncode or not (folder / "answer/estimate.json").is_file():
            raise Refused(f"the scan's cycle gave no estimate ({folder / 'cycle.log'}): {(done.stdout + done.stderr)[-300:]}")
        return json.loads((folder / "answer/estimate.json").read_text())

    def _first_reading(self, at):
        """The base's first reading, made the same every time: of the equal answers for a symmetric base (the matcher
        gives now one, now the other, and with it the side each piece goes in from), the one whose first slot is
        nearer the robot, which stands at y = 0; of two as near, the one further along -x."""
        first = self.slots[self.order[0]["piece"]]
        return max((at @ assembly.pose(turn, shift) for turn, shift in self.pack.shape(self.base).symmetries),
                   key=lambda reading: (round(float((reading @ first)[1, 3]), 4), -round(float((reading @ first)[0, 3]), 4)))

    def _in_doubt(self, report):
        """The pieces a scan's report leaves in doubt that the build still has to act on: this puzzle's own, not yet
        placed. Another puzzle's pieces stay obstacles where they were seen, and a base with pieces in it is not
        moved again and is read as it was last read (a base seen half a turn round is the same base), so doubt
        about it stops nothing: a finished puzzle whose base a last look found pushed was scanned, left its base
        in doubt, and the build ended refused with every piece seated."""
        settled = {*self.placed, *([self.base] if self.placed else [])}
        return sorted((set(report.get("doubtful", [])) & set(self.names)) - settled)

    def _seen(self, folder):
        """The scan's views taken and combined: (the cycle's report, the names of the views taken). From above first,
        the survey's own view, and from the sides only when that leaves a piece in doubt: the views from the sides
        tell a piece's rival poses apart and add nothing to a table read without doubt (in 16 scans of simulated
        builds the view from above left none). A piece of the puzzle that the view from above does not find at all
        is looked for from above the parts of the table its picture does not reach (``scan.beyond``), each such
        view read by the cycle on its own; found there, it joins the report. With ``--perception oracle`` there is
        no report and no doubt."""
        views, taken = self._views(), []
        self._sweep(folder / "scan", views[:1], taken)
        if self.args.perception == "oracle":
            return {}, [view["name"] for view in taken]
        report = self._combine(folder)
        if self._in_doubt(report):
            self.say(f"scan: {', '.join(self._in_doubt(report))} in doubt from above; the views from the sides")
            (folder / "answer").rename(folder / "answer_from_above")
            self._sweep(folder / "scan", views[1:], taken)
            self.cell.camera_to(views[0][1])
            report = self._combine(folder)
        names = [view["name"] for view in taken]
        for view in self.config["scan"]["beyond"]:          # what the survey's picture does not reach: looked for from above there, when it is missed
            missing = sorted(set(self.names) - {piece["name"] for piece in report["pieces"]})
            if not missing:
                break
            self.say(f"scan: {', '.join(missing)} not in the picture; looking from above ({view['name']})")
            there = []
            self._sweep(folder / view["name"] / "scan", [(view["name"], self._view(view["eye_m"], view["look_at_m"], view["roll_deg"]))], there)
            self.cell.camera_to(views[0][1])
            more = self._combine(folder / view["name"])
            found = [piece for piece in more["pieces"] if piece["name"] in missing]
            report["pieces"] += found
            report["doubtful"] = [*report.get("doubtful", []), *(name for name in more.get("doubtful", []) if name in missing)]
            names.append(view["name"])
        return report, names

    @stepped("scan")
    def scan(self):
        """Scan the table: take the views, combine them into one estimate, rebuild the twin from it (the estimate
        the lookahead simulates from, and its task file), and say what the build has to do next. It is the graph's
        first step, and the step it comes back to when a pick closed on nothing or a look found the table changed."""
        self.scans, self.looks = self.scans + 1, self.looks + 1
        if self.scans > len(self.slots) + 3:
            raise Refused(f"the table was scanned {self.scans - 1} times and the build is not done")
        folder = self.out / "scans" / f"{self.scans:02d}"
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True)
        at = self.cell.gripper_pose()
        if at[2, 3] < CARRY_M:                              # down among the pieces after a failed pick: let go and straight up first
            self.cell.open_to(0.08)
            self.cell.gripper_line(shifted(at, [0, 0, CARRY_M]), self.config["speeds"]["approach_m_s"])
        self.held = None
        report, views = self._seen(folder)
        truth, name = self.truth(), f"look_{self.looks:02d}_scan"
        outlines = lambda poses: {piece: {"xyz": [round(float(v), 5) for v in at[:3, 3]],
                                          "outline": np.round(assembly.footprint(self.pack, piece, at), 5).tolist()} for piece, at in poses.items()}
        record = {"index": self.looks, "label": "scan", "sim_s": self.cell.steps / cell.PHYSICS_HZ, "truth": outlines(truth), "views": views}
        self.look_log.append(record)
        self.log.note(look=self.looks, views=views)
        if self.args.perception == "oracle":
            seen = truth
            record.update(estimate=outlines(truth), oracle=True)
        else:
            report["score"] = puzzle_scan.score(report, {piece: at.tolist() for piece, at in truth.items()}, self.pack)
            seen = {piece["name"]: cell.pose(np.array(piece["rotation"]), piece["position_m"]) for piece in report["pieces"]}
            shutil.copy(folder / "answer/real_rgb.png", self.out / f"{name}.png")
            record.update(estimate=outlines(seen), picture=f"{name}.png", unexplained=len(report.get("unexplained", [])),
                          scores=[{key: row.get(key) for key in ("name", "result", "as", "position_error_mm", "rotation_error_deg")}
                                  for row in report["score"]["pieces"]])
        missing = [piece for piece in self.names if piece not in seen]
        if missing:
            raise Refused(f"the scan did not find {missing}")
        if self.base in self.estimate:                      # a symmetric base read half a turn round would rename the slots: keep to the last reading
            seen[self.base] = assembly.nearest(self.pack, self.base, self.estimate[self.base], seen[self.base])
        else:
            seen[self.base] = self._first_reading(seen[self.base])
        far = lambda piece, a, b: (lambda metres, degrees: metres > self.SCAN_MOVED_M or degrees > self.SCAN_MOVED_DEG)(*assembly.pose_error(self.pack, piece, a, b))
        moved = sorted(piece for piece in seen if piece in self.estimate and far(piece, seen[piece], self.estimate[piece]))
        before, self.estimate = self.placed, dict(seen)
        limit = self.config["seated"]
        near = lambda read: read["position_error_mm"] <= 2000 * limit["position_m"] and read["rotation_error_deg"] <= 2 * limit["rotation_deg"]
        # In its slot: read there; or placed before and not seen elsewhere; or put in, read just outside its seat by the
        # look after it (which sent the build here) and read no further out now. Two readings that near are a seated piece.
        self.placed = [piece for piece in self.slots if self._seated(piece, seen, seen[self.base])["seated"]
                       or (piece in before and not far(piece, seen[piece], self._seat(piece)))
                       or (self.metrics[piece].get("read_again") and near(self._seated(piece, seen, seen[self.base])))]
        for piece in self.placed:
            self.estimate[piece] = self._seat(piece)
        metres, degrees = assembly.pose_error(self.pack, self.base, seen[self.base], self._base_target())
        if self.placed:                                     # a base with pieces in it is not slid again: the build goes on where it lies
            self.base_goal = seen[self.base]
        elif metres <= BASE_PLACED_M and degrees <= BASE_PLACED_DEG:
            self.base_goal = getattr(self, "base_goal", self._base_target())
        if self.placed or (metres <= BASE_PLACED_M and degrees <= BASE_PLACED_DEG):
            following = next((step["piece"] for step in self.order if step["piece"] not in self.placed), "done")
        else:
            following, self.base_slides = self.base, 0
        doubtful = self._in_doubt(report)
        if doubtful:
            raise Refused(f"the scan leaves {doubtful} in doubt; one more view would be: {(report.get('next_view') or {}).get('view')}")
        puzzle_scan.write_task(assembly.layout_estimate(self.pack, {piece: self._twin_pose(piece) for piece in self.estimate}), folder / "task",
                               f"puzzle_build.py, scan {self.scans}")
        result = {"scan": self.scans, "found": len(seen), "pieces": sorted(seen), "missing": missing, "views": views, "moved": moved,
                  "placed": list(self.placed), "next": following, "unexplained": len(report.get("unexplained", [])),
                  "position_error_mm": (report.get("score") or {}).get("position_error_mm")}
        if self.scans == 1:                                 # what metrics.json calls the survey: the build's first sight of the table
            self.survey_report = result
            for row in (report.get("score") or {}).get("pieces", []):
                if row["name"] in self.metrics:
                    self.metrics[row["name"]].update(identified=row["result"] == "correct", identified_as=row.get("as"),
                                                     survey_position_error_mm=row.get("position_error_mm"),
                                                     survey_rotation_error_deg=row.get("rotation_error_deg"))
        self.mark(f"scan {self.scans}")
        return result

    def _changed(self, seen, handled):
        """The pieces a look finds elsewhere than the estimate has them, the one just handled apart: a loose piece
        moved, or the base no longer in its place. The graph then scans again before it goes on."""
        moved = []
        for piece, at in seen.items():
            if piece == handled or piece in self.placed or piece not in self.estimate:
                continue
            metres, degrees = assembly.pose_error(self.pack, piece, at, self.base_goal if piece == self.base else self.estimate[piece])
            limit = (BASE_PLACED_M, BASE_PLACED_DEG) if piece == self.base else (self.SCAN_MOVED_M, self.SCAN_MOVED_DEG)
            if metres > limit[0] or degrees > limit[1]:
                moved.append(piece)
        return sorted(moved)

    def _disturb(self, request):
        """``--disturb PIECE``: when that piece is first approached it is put 50 mm aside, across the way it is
        gripped, on the side with more room, as a hand or a cable would move it. The pick then closes on nothing,
        and the graph has to scan again to go on."""
        piece = request.get("inputs", {}).get("piece")
        if request["tool"] != "approach" or piece != self.args.disturb or self.disturbed:
            return
        self.disturbed = True
        truth = self.truth()
        across = truth[piece][:3, :3] @ np.array(self.config["grasps"][piece]["closing_axis"], dtype=float)
        room = lambda at: min(float(np.linalg.norm(at[:2, 3] - other[:2, 3])) for name, other in truth.items() if name != piece)
        aside = max((shifted(truth[piece], sign * 0.05 * across) for sign in (1, -1)), key=room)
        self.say(f"{piece}: moved 50 mm aside behind the robot's back (--disturb)")
        self.cell.place_bodies({self.paths[piece]: aside})
        self.cell.step(100)

    # ---- the base, watched while a piece is brought to it ------------------------------------------
    def _frame(self):
        """One depth picture from the wrist camera, where it is now: (depth in metres, the camera for
        ``puzzle_base_check.world_points``, milliseconds it took). The recorder stays up between pictures."""
        began = time.perf_counter()
        prim, body, intrinsics = self.cell.cameras["wrist"]
        if self.checker is None:
            self.checker = self.out / "base_checks"
            if self.checker.exists():
                shutil.rmtree(self.checker)
            self.cell.kit(f"base_check = Recorder({prim!r}, {body!r}, {str(self.checker)!r}, {[intrinsics['width'], intrinsics['height']]!r}, {intrinsics!r})")
        # Two pictures, the second one read: the simulator's renderer is one picture behind whatever was last put
        # somewhere without moving there (a rehearsal put back, the test's slide of the base). A camera has no such lag.
        # Between pictures the recorder is idle: kept awake, it has every render step of the build draw its picture too.
        index, frame, scale = self.cell.kit("base_check.idle(False); await shoot([base_check]); shot = await shoot([base_check]); base_check.idle(True); "
                                            "result = [shot[0], base_check.frames[shot[0]], base_check.intrinsics['depth_scale']]")
        depth = np.asarray(Image.open(self.checker / "depth" / f"{index:05d}.png"), dtype=np.float32) * scale
        return depth, {**intrinsics, "pose": np.array(frame["world_from_camera"]), "depth_scale": scale}, round(1000 * (time.perf_counter() - began), 1), index

    @stepped("base check")
    def _base_check(self, piece, where):
        """Is the base where the estimate has it? One depth picture and a fit of the base's CAD, with the pieces
        seated in it, from its last pose (``puzzle_base_check``): no masks, no SAM pass. Changes nothing. ``piece``
        is the one the check is made for: in the hand, and so not on the table, except at the ``look`` before its pick."""
        depth, camera, frame_ms, index = self._frame()
        settings = {key: value for key, value in self.config["base_check"].items() if key in puzzle_base_check.SETTINGS}
        points = puzzle_base_check.world_points(depth, camera, settings.get("stride", puzzle_base_check.SETTINGS["stride"]))
        loose = {name: at for name, at in self.estimate.items() if name not in (self.base, *self.placed) and (name != piece or where == "look")}
        found = puzzle_base_check.check(points, self.pack, self.base, self.estimate[self.base], {name: self.slots[name] for name in self.placed}, loose, settings)
        truth = self.truth()[self.base]                     # to score the fit; the robot never reads it
        (self.checker / f"check_{index:05d}.json").write_text(json.dumps({
            "what": "What this base check was given, beside its picture (depth/, rgb/): puzzle_base_check.check can be run on it again. truth scores the fit only.",
            "camera": {**{key: value for key, value in camera.items() if key != "pose"}, "pose": camera["pose"].tolist()}, "base": self.base, "pack": self.config["pack"],
            "last": self.estimate[self.base].tolist(), "seated": list(self.placed), "loose": {name: at.tolist() for name, at in loose.items()},
            "settings": settings, "piece": piece, "where": where, "truth": truth.tolist()}) + "\n")
        off = np.linalg.inv(truth) @ found["pose"]
        row = {**{key: value for key, value in found.items() if key != "pose"}, "piece": piece, "where": where, "frame": index, "frame_ms": frame_ms,
               "sim_s": round(self.cell.steps / cell.PHYSICS_HZ, 3), "wall": time.time(),
               "fit_against_truth_mm": round(1000 * float(np.linalg.norm(off[:2, 3])), 2),
               "fit_against_truth_deg": round(abs(math.degrees(math.atan2(off[1, 0], off[0, 0]))), 3)}
        self.base_log["checks"].append(row)
        (self.out / "base_checks.json").write_text(json.dumps(self.base_log, indent=1) + "\n")
        self.say(f"{piece}: base check at the {where}: {found['verdict']}" + (f", {found['moved_mm']:.1f} mm and {found['turned_deg']:.1f} degrees" if found["moved_mm"] else "")
                 + f" ({found['fit_ms']:.0f} ms fit, {frame_ms:.0f} ms picture)")
        return {**row, "pose": found["pose"].tolist()}

    def _moved_base(self, piece, found):
        """The base is elsewhere than the estimate had it: its slots and the pieces seated in it go with it, and what
        is left of this piece's way (over the slot, then down) is laid out again from where the base now is."""
        at, hold = np.array(found["pose"]), self.held
        moved = at @ np.linalg.inv(self.estimate[self.base])
        self.estimate[self.base] = self.base_goal = at
        for name in self.placed:
            self.estimate[name] = self._seat(name)
        hold.update(seat=moved @ hold["seat"], place=moved @ hold["place"], axis=moved[:3, :3] @ hold["axis"],
                    others={name: self._twin_pose(name) if name in self.placed else at for name, at in self.estimate.items() if name != piece})
        over = self._tcp(shifted(hold["place"], hold["axis"] * hold["clear_m"]), hold["width_m"])
        hold["path"].update(carry=shifted(over, [0, 0, CARRY_M]), over=over, place=self._tcp(shifted(hold["place"], hold["axis"] * 0.001), hold["width_m"]))

    def _ahead_place(self, piece):
        """Simulate ahead, on Newton, what is left of a placement after the base moved: from the hand as it is, with
        the piece in it, up, over the slot's new place, down, let go, away; judged as the lookahead judges, and the
        stage put back. Only with ``--correction lookahead``: a build that simulates nothing ahead simulates nothing here."""
        if self.correction != "lookahead":
            return {"verdict": "go", "tries": []}
        hold, speeds, held, clip = self.held, self.config["speeds"], self.held, f"{piece}_moved_{len(self.base_log['corrections']) + 1}"
        self.cell.checkpoint(self.paths.values())
        self.ahead = True
        try:
            self.cell.aside(self.out / "lookahead" / clip)
            with self.log.step("try", piece, clip=clip) as attempt:
                self.cell.place_bodies({self.paths[name]: self._twin_pose(name) for name in self.estimate if name != piece})
                self.cell.step(50)
                before = self.truth()
                try:
                    self.cell.gripper_carry(hold["path"]["carry"], speed_deg_s=speeds["joint_deg_s"])
                    self.cell.gripper_line(hold["path"]["over"], speeds["approach_m_s"])
                    self.cell.gripper_line(hold["path"]["place"], speeds["insert_m_s"], segment_m=0.005)
                    self.cell.open_to(hold["width_m"] + self.config["open_beyond_m"])
                    back = shifted(self.cell.gripper_pose(), -self.cell.gripper_pose()[:3, 2] * RETREAT_M)
                    self.cell.gripper_line(back, speeds["approach_m_s"])
                    self.cell.step(250)
                    after = self.truth()
                    others = max(float(np.linalg.norm(after[name][:3, 3] - before[name][:3, 3])) for name in after if name != piece)
                    row = {**self._seated(piece, after, after[self.base]), "blocked": None, "others_moved_mm": round(others * 1000, 2)}
                    row["go"] = bool(row["seated"] and others < 0.003)
                except cell.MoveFailed as error:
                    row = {"go": False, "error": str(error), "blocked": str(error)}
                attempt["result"] = row = {**row, "way": hold["way"], "adjusted": piece in self.adjust}
            filmed = self.cell.back()
            self.tries[clip] = {**{key: filmed[key] for key in ("fps", "zero_s", "captions", "samples", "grips")}, "piece": piece}
        finally:
            self.cell.restore()
            self.ahead, self.held = False, held
        return {"verdict": "go" if row["go"] else "abort", "tries": [row]}

    def _set_down(self, piece, why):
        """Put the piece back where it was picked and get clear: the table has to be scanned before it can be placed."""
        hold, speeds = self.held, self.config["speeds"]
        path = hold["path"]
        with self.log.step("set down", piece, why=why, waypoints=[waypoint("lift", path["lift"]), waypoint("pick", path["pick"])]):
            self.say(f"{piece}: {why}; putting it back to scan the table")
            self.cell.gripper_line(shifted(self.cell.gripper_pose(), [0, 0, CARRY_M]), speeds["approach_m_s"])
            self.cell.gripper_carry(path["lift"], speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(shifted(path["pick"], [0, 0, 0.002]), speeds["approach_m_s"])
            self.cell.open_to(hold["width_m"] + self.config["open_beyond_m"])
            self.cell.gripper_line(path["hover"], speeds["approach_m_s"])
        self.held, self.put_back = None, piece

    def _sees_base(self):
        """Whether the wrist camera has the base in view while the held piece is over its slot: it rides on the
        gripper and looks along it, so it does when the gripper points down there, and not when the gripper is level."""
        return bool(self.held["place"][2, 2] < -0.7)

    def _vouch(self, piece):
        """Check the base from over the slot, the piece in the hand, before it goes on. Steady: True. Moved, and the
        fit trusted: the slots move with it, what is left of the placement is simulated ahead (when the build simulates
        ahead), the hand comes over the slot's new place, and the base is checked again. Anything else: the piece
        is put back and the answer is False."""
        for _ in range(self.config["base_check"]["corrections"] + 1):
            self._shift_base(piece, "before")
            found = self._base_check(piece, "hover")
            noticed = {"sim_s": self.cell.steps / cell.PHYSICS_HZ, "wall": time.time()}
            if found["verdict"] == "steady":
                return True
            if found["verdict"] != "moved":
                self._set_down(piece, found["why"])
                return False
            with self.log.step("correct", piece, moved_mm=found["moved_mm"], turned_deg=found["turned_deg"]) as row:
                began = time.perf_counter()
                self._moved_base(piece, found)
                try:                                        # the gripper has to fit at the slot's new place, as it had to at the old one
                    self._corrected_fits(piece, self.held)
                except Misfit as misfit:
                    row["result"] = {"piece": piece, "where": "hover", "moved_mm": found["moved_mm"], "turned_deg": found["turned_deg"], "misfit": str(misfit)}
                    self._set_down(piece, f"the base moved and the gripper does not fit at the slot's new place ({misfit})")
                    return False
                laid_out = time.perf_counter() - began
                self.log.note(waypoints=[waypoint("over", self.held["path"]["over"]), waypoint("place", self.held["path"]["place"])])
                with self.log.step("lookahead", piece, of="what is left of the placement, after the base moved") as ahead:
                    began = time.perf_counter()
                    ahead["result"] = verdict = self._ahead_place(piece)
                    simulated = time.perf_counter() - began
                if verdict["verdict"] != "go":
                    self._set_down(piece, "the lookahead of the corrected placement said abort")
                    raise Refused(f"{piece}: the base moved and the lookahead of the corrected placement said abort")
                self.say(f"{piece}: the base moved {found['moved_mm']:.1f} mm and {found['turned_deg']:.1f} degrees; its slots go with it")
                self.cell.gripper_line(self.held["path"]["over"], self.config["speeds"]["approach_m_s"])
                row["result"] = record = {"piece": piece, "where": "hover", "moved_mm": found["moved_mm"], "turned_deg": found["turned_deg"],
                                          "lay_out_ms": round(1000 * laid_out, 2), "lookahead_wall_s": round(simulated, 2), "lookahead": verdict["verdict"],
                                          "replan_sim_s": round(self.cell.steps / cell.PHYSICS_HZ - noticed["sim_s"], 3),
                                          "replan_wall_s": round(time.time() - noticed["wall"], 2)}
                self.base_log["corrections"].append(record)
        self._set_down(piece, "the base keeps moving")
        return False

    def _look_first(self, piece):
        """Before a piece that goes in with the gripper level is picked: look at the base from above and check it, the
        hand empty. Over its slot the camera, which rides on the gripper, will not see the base, and a look with the
        piece in the hand swings it one more time, which is what makes a fork creep in the fingers (measured by the
        placement work: 8.1 mm along the fingers with that look, 4.9 without). A base found moved by a fit that is
        trusted is taken where it is, with its slots and seated pieces, before the grasp and the place are planned.
        Anything else is kept as ``unsure`` and acted on when the piece is to be carried in: put back, and the table
        scanned. A piece placed with the gripper pointing down is checked at its hover instead, and not here."""
        self.put_back = self.unsure = None
        if (self._seat(piece)[:3, :3] @ np.array(self.config["grasps"][piece]["approach_axis"], dtype=float))[2] < -0.7:
            return
        look = self.config["base_check"]["look"]
        with self.log.step("look", piece, **look):
            self.say(f"{piece}: a look at the base from above before it is picked")
            over = self.estimate[self.base][:3, 3] * [1, 1, 0] + [*look["offset_m"], 0.0]
            self.cell.camera_to(self._view(over + [0.0, 0.0, look["height_m"]], over, look["roll_deg"]))
        for _ in range(self.config["base_check"]["corrections"] + 1):
            self._shift_base(piece, "before")
            found = self._base_check(piece, "look")
            if found["verdict"] == "steady":
                return
            if found["verdict"] != "moved":
                self.unsure = found["why"]
                return
            with self.log.step("correct", piece, moved_mm=found["moved_mm"], turned_deg=found["turned_deg"]) as row:
                self.estimate[self.base] = self.base_goal = np.array(found["pose"])
                for name in self.placed:
                    self.estimate[name] = self._seat(name)
                self.say(f"{piece}: the base moved {found['moved_mm']:.1f} mm and {found['turned_deg']:.1f} degrees; its slots go with it")
                row["result"] = record = {"piece": piece, "where": "look", "moved_mm": found["moved_mm"], "turned_deg": found["turned_deg"],
                                          "lookahead": "go", "replan_sim_s": 0.0, "replan_wall_s": 0.0}
                self.base_log["corrections"].append(record)
        self.unsure = "the base keeps moving"

    def _look_check(self, piece):
        """Whether the piece in the hand may be carried to its slot. What the look before its pick left in doubt
        (``_look_first``) ends here: the piece is put back and the answer is False. A piece placed with the gripper
        pointing down is checked at its hover, with the piece in the hand, and passes here."""
        if self.unsure:
            self._set_down(piece, self.unsure)
            return False
        return True

    def _descend(self, piece):
        """Lower the piece into its slot. Simulating ahead, or with the gripper level (no view of the base): one
        line. Else in stages, the base checked after each while the piece is still above the slot: when the check
        does not say steady the arm goes back up to the pre-place hover and ``_vouch`` decides from there; nothing is
        lowered on a base not vouched for."""
        hold, speeds, watch = self.held, self.config["speeds"], self.config["base_check"]
        if self.ahead or not self._sees_base():
            self.cell.gripper_line(hold["path"]["place"], speeds["insert_m_s"], segment_m=0.005)
            return
        while True:
            start, goal = self.cell.gripper_pose(), hold["path"]["place"]
            far = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
            stages = max(1, math.ceil(far / watch["descent_step_m"]))
            for stage in range(1, stages + 1):
                self.cell.gripper_line(cell.blend(start, goal, stage / stages), speeds["insert_m_s"], segment_m=0.005)
                if stage == stages:
                    return
                if far * (1 - stage / stages) < hold["clear_m"] - MARGIN_M:      # in the slot: the base cannot leave without the arm
                    continue
                self._shift_base(piece, "descent")
                if self._base_check(piece, "descent")["verdict"] == "steady":
                    continue
                with self.log.step("retreat", piece, waypoints=[waypoint("over", hold["path"]["over"])]):
                    self.say(f"{piece}: the base is not where it was; back up to the pre-place hover")
                    self.cell.gripper_line(hold["path"]["over"], speeds["approach_m_s"])
                if not self._vouch(piece):
                    return
                break

    def _shift_base(self, piece, where):
        """``--shift-base PIECE:MM:DEG[:before|descent]``: slide the base, with what is seated in it, that far and
        turn it that much about its own middle, as a hand would: just before the check that precedes the piece's
        placement, or while the piece is lowered. Once each."""
        for spec in self.args.shift_base or ():
            name, mm, deg, when = (spec.split(":") + ["before"])[:4]
            if name != piece or when != where or spec in self.shifted:
                continue
            truth = self.truth()
            slide = cell.pose(puzzle_scan.yaw_matrix(math.radians(float(deg))), [0, 0, 0])
            slide[:3, 3] = truth[self.base][:3, 3] - slide[:3, :3] @ truth[self.base][:3, 3] + float(mm) / 1000 * np.array([0.8, 0.6, 0.0])
            with self.log.step("shift base", piece, mm=float(mm), deg=float(deg), when=when):
                self.say(f"{piece}: the base is slid {float(mm):g} mm and turned {float(deg):g} degrees behind the robot's back (--shift-base)")
                self.cell.place_bodies({self.paths[body]: slide @ truth[body] for body in (self.base, *self.placed)})
                self.cell.step(50)
            self.shifted[spec] = {"piece": piece, "mm": float(mm), "deg": float(deg), "when": when, "sim_s": round(self.cell.steps / cell.PHYSICS_HZ, 3), "wall": time.time()}
            self.base_log["shifts"].append(self.shifted[spec])

    def _base_target(self):
        """Where the base goes: the configured place, at the configured yaw or half a turn from it, whichever is nearer."""
        target, now = self.config["base_target"], self.estimate[self.base]
        yaw_now = math.atan2(now[1, 0], now[0, 0])
        yaw = min((math.radians(target["yaw_deg"]) + extra for extra in (0.0, math.pi)),
                  key=lambda value: abs((value - yaw_now + math.pi) % (2 * math.pi) - math.pi))
        return cell.pose(puzzle_scan.yaw_matrix(yaw), [*target["position_m"], now[2, 3]])

    def _grasp(self, piece, now, target, around_now, around_then, axis=(0, 0, 1), clear_m=0.0, grasp=None):
        """The first way of holding a piece that fits the CAD, that the arm reaches at both ends, and that a
        lookahead has not found blocked: of those, the first that keeps wrist 3 in its window (``in_window``), or
        the one that leaves it least. The gripper is checked as it comes down, open, and as it holds the piece. ``grasp`` is the
        piece's own definition unless another is given (the one a piece is turned on the table by)."""
        grasp = grasp or self.config["grasps"][piece]
        back, why = np.linalg.inv(self.cell.wrist_from_gripper), []
        with self.cell.timed("plan_s"):
            options = assembly.choose_grasps(self.pack, piece, grasp, now, target, around_now, around_then, cell.pad_setback, axis, clear_m,
                                             open_m=self.config["open_beyond_m"], hover_m=HOVER_M, refused=why)
        if not options:
            raise Refused(f"no grasp of {piece} fits: {'; '.join(why[:3]) or 'it does not lie the way its grasp takes it, from above'}")
        (low, high), rise = np.radians(self.config["arm"]["wrist_3_deg"]), math.radians(self.config["arm"]["wrist_3_rise_deg"])
        in_reach = []
        for way, option in enumerate(options):
            if way in self.avoid.get(piece, ()):
                continue
            try:
                turned = [self.cell.reconfigure(self._tcp(at, grasp["width_m"]) @ back)["wrist_3_joint"]
                          for at in (option["pick"], option["place"], shifted(option["place"], np.array(axis) * clear_m))]
            except cell.MoveFailed:
                continue
            # How far outside what arm allows: out of the range first (the cell's own joint window lies just beyond it), then how much
            # further up than a quarter turn (a little more cord than there is).
            out = (float(sum(max(0.0, low - q, q - high) for q in turned[:2])), float(max(0.0, turned[1] - turned[0] - rise)))
            hold = {**option, "width_m": grasp["width_m"], "piece": piece, "way": way, "wrist_3_deg": [round(math.degrees(q), 1) for q in turned],
                    "in_window": out == (0.0, 0.0)}
            if hold["in_window"]:                           # the camera's cord: a way that does not wind wrist 3 out of its window
                return hold
            in_reach.append((out, way, hold))
        if in_reach:                                        # none inside: the one least outside, the first of equals
            return min(in_reach, key=lambda found: found[:2])[2]
        raise Refused(f"no grasp of {piece} that fits is within reach")

    def _turn(self, piece, hold, others, axis, clear_m):
        """Turn a lying piece on the table, where it lies, so that it can be picked with wrist 3 inside its window.

        A piece that goes into its slot one way round (a fork) is gripped one way round, so how it lies decides
        where wrist 3 stands at its pick, and for about half the ways a fork can lie that is outside the window. The
        same grip half a turn round is no help: when the one is outside the window the other is inside, and a turn
        that brings the first inside takes the second out. So the piece is turned by another grip (``turn_grasps``,
        a quarter turn from its own), lifted ``TURN_LIFT_M``, by the angle that brings its own grip's pick nearest
        the middle of the window, and looked at again where it was set down. Returns whether it was turned: False
        when the piece has no such grip, or no angle leaves room, reach and both grips inside the window."""
        grasp = self.config["turn_grasps"].get(piece)
        if grasp is None:
            return False
        now, back = self.estimate[piece], np.linalg.inv(self.cell.wrist_from_gripper)
        low, high = self.config["arm"]["wrist_3_deg"]
        about = (now @ np.array([*grasp["point_m"], 1.0]))[:3] * [1.0, 1.0, 0.0]     # the turn is about the gripper's own axis: it stays where it is
        ways = []
        for deg in range(-165, 181, 15):
            spin = puzzle_scan.yaw_matrix(math.radians(deg))
            spin = cell.pose(spin, about - spin @ about)
            try:
                wrist = math.degrees(self.cell.reconfigure(self._tcp(spin @ hold["pick"], hold["width_m"]) @ back)["wrist_3_joint"])
            except cell.MoveFailed:
                continue
            if deg and low <= wrist <= high:
                ways.append((abs(wrist - (low + high) / 2), deg, spin @ now))
        for _, deg, there in sorted(ways, key=lambda way: way[:2]):
            grown = assembly.footprint(self.pack, piece, there, MARGIN_M)
            if not all(assembly._apart(grown, assembly.footprint(self.pack, other, at)) for other, at in others.items()):
                continue
            try:
                after = self._grasp(piece, there, self._seat(piece), others, others, axis, clear_m)
                by = self._grasp(piece, now, there, others, others, clear_m=self.TURN_LIFT_M, grasp=grasp)
            except Refused:
                continue
            if after["in_window"] and by["in_window"]:
                break
        else:
            return False
        width, speeds = by["width_m"], self.config["speeds"]
        pick, down = self._tcp(by["pick"], width), self._tcp(shifted(by["place"], [0, 0, 0.002]), width)
        path = {"hover": shifted(pick, [0, 0, HOVER_M]), "pick": pick, "lift": shifted(pick, [0, 0, self.TURN_LIFT_M]),
                "over": shifted(down, [0, 0, self.TURN_LIFT_M]), "down": down, "clear": shifted(down, [0, 0, HOVER_M])}
        with self.log.step("turn", piece, deg=deg, wrist_3_deg={"as_it_lay": hold["wrist_3_deg"], "turned": after["wrist_3_deg"], "turning": by["wrist_3_deg"]},
                           grasp={key: grasp[key] for key in ("point_m", "closing_axis", "approach_axis", "width_m", "what")},
                           waypoints=[waypoint(name, at) for name, at in path.items()]) as row:
            self.say(f"{piece}: its pick would put wrist 3 at {hold['wrist_3_deg'][0]:.0f} degrees; turning it {deg} degrees on the table first")
            self.cell.open_to(width + self.config["open_beyond_m"])
            self.cell.gripper_carry(path["hover"], speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(pick, speeds["approach_m_s"])
            self.cell.grip(width, self.config["grip_force_n"])
            self.cell.gripper_line(path["lift"], speeds["approach_m_s"])
            self.cell.gripper_carry(path["over"], speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(down, speeds["approach_m_s"])
            self.cell.open_to(width + self.config["open_beyond_m"])
            self.cell.gripper_line(path["clear"], speeds["approach_m_s"])
            # Where it lies now is measured, not reckoned: it slides in the fingers as they close. How it is turned is
            # the other way about: the fingers close on two flat ends and square the piece to themselves, so it lies
            # turned as the gripper was, nearer than a look reads a turn (1.5 degrees off on the first fork turned, which
            # then jammed in its slot). The look's turn is taken only when it disagrees by more than a look is ever off.
            seen, _ = self._look(f"{piece}_turned")
            if piece not in seen:
                raise Refused(f"{piece} was turned on the table and the look after it does not find it")
            metres, degrees = assembly.pose_error(self.pack, piece, seen[piece], there)
            squared = degrees <= self.TURN_LOOK_DEG
            self.estimate[piece] = cell.pose(there[:3, :3], seen[piece][:3, 3]) if squared else seen[piece]
            row["result"] = {"deg": deg, "off_mm": round(metres * 1000, 2), "off_deg": round(degrees, 2), "turn_taken_from": "the gripper" if squared else "the look"}
        self.metrics[piece]["turned_deg"] = deg
        return True

    @staticmethod
    def _tcp(pads, width_m):
        """The gripper frame's pose that puts the pad centres at a pose, with the pads ``width_m`` apart."""
        return shifted(pads, pads[:3, 2] * cell.pad_setback(width_m))

    @stepped("place_base")
    def place_base(self, piece):
        """Grip the base by its centre island and slide it on the table to the middle."""
        now, target, speeds = self.estimate[piece], self._base_target(), self.config["speeds"]
        others = {name: at for name, at in self.estimate.items() if name != piece}
        if not assembly.path_clear(self.pack, piece, now, target, others):
            raise Refused("the way to the middle is not free of pieces")
        hold = self._grasp(piece, now, target, others, others)
        width = hold["width_m"]
        pick, place = self._tcp(hold["pick"], width), self._tcp(hold["place"], width)
        hover, rise = shifted(pick, [0, 0, HOVER_M]), shifted(place, [0, 0, HOVER_M])
        self.log.note(grasp=self._grasp_plan(piece, hold), target=waypoint("middle", target),
                      waypoints=[waypoint(*point) for point in (("hover", hover), ("pick", pick), ("place", place), ("rise", rise))])
        with self.log.step("approach", piece, waypoints=[waypoint("hover", hover), waypoint("pick", pick)]):
            self.say(f"{piece}: gripping its centre island")
            self.cell.open_to(width + self.config["open_beyond_m"])
            self.cell.gripper_carry(hover, speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(pick, speeds["approach_m_s"])
        with self.log.step("grip", piece, force_n=self.config["grip_force_n"], width_m=width) as row:
            row["result"] = {"gap_m": self.cell.grip(width, self.config["grip_force_n"])}
        with self.log.step("slide", piece, waypoints=[waypoint("place", place)], speed_m_s=speeds["slide_m_s"]):
            self.say(f"{piece}: sliding it to the middle")
            self.cell.gripper_line(place, speeds["slide_m_s"])
        with self.log.step("release", piece, waypoints=[waypoint("rise", rise)]):
            self.cell.open_to(width + self.config["open_beyond_m"])
            self.cell.gripper_line(rise, speeds["approach_m_s"])
        self.base_slides += 1
        self.base_goal = self.estimate[piece] = target      # where it should now be; the next look measures it
        self.mark(f"{piece} in the middle")
        return {"result": "slid", "target": se3(target), "slide": self.base_slides}

    def _grasp_plan(self, piece, hold):
        """A chosen grasp as the run's log keeps it: its definition on the CAD and which way of holding was taken."""
        return {**{key: self.config["grasps"][piece][key] for key in ("point_m", "closing_axis", "approach_axis", "width_m", "what")},
                "way": hold["way"], "gripper_in_piece": np.round(hold["in_piece"], 5).tolist(), "force_n": self.config["grip_force_n"],
                "wrist_3_deg": hold["wrist_3_deg"], "wrist_3_in_window": hold["in_window"]}

    def _seat(self, piece):
        return self.estimate[self.base] @ self.slots[piece]

    @stepped("approach")
    def approach(self, piece):
        """Choose a grasp for the piece, open, and bring the open pads round it."""
        step = next(step for step in self.order if step["piece"] == piece)
        if set(step["needs"]) - set(self.placed):
            raise Refused(f"{piece} needs {step['needs']} in first")
        if piece not in self.estimate:
            raise Refused(f"{piece} was not seen")
        if not self.ahead:
            self._look_first(piece)
        # Everything else the robot believes is on the table, at the pick and at the slot alike: the base, the pieces
        # it has placed (in their slots) and the loose ones where the last look saw them. A loose piece lying in
        # this piece's slot, or where the open gripper comes down, is in the way.
        others = {name: self._twin_pose(name) if name in self.placed else at for name, at in self.estimate.items() if name != piece}
        axis = self.estimate[self.base][:3, :3] @ np.array(step["axis"], dtype=float)
        hold = self._grasp(piece, self.estimate[piece], self._seat(piece), others, others, axis, step["clear_m"] + MARGIN_M)
        if not hold["in_window"] and self.config["arm"]["turn_on_table"] and not self.ahead and self._turn(piece, hold, others, axis, step["clear_m"] + MARGIN_M):
            hold = self._grasp(piece, self.estimate[piece], self._seat(piece), others, others, axis, step["clear_m"] + MARGIN_M)
        hold.update(axis=axis, clear_m=step["clear_m"] + MARGIN_M, corrected_by=None, others=others)
        self.trying = hold
        if piece in self.adjust:                            # the lookahead's: how the piece really sat in the simulated hand
            hold.update(place=hold["seat"] @ self.adjust[piece], corrected_by="lookahead")
        elif piece in self.table:                           # the table's: how a piece of this type sat when it was measured
            hold.update(place=hold["seat"] @ hold["in_piece"] @ self._table_shift(piece, hold["flipped"]), corrected_by="table")
        if hold["corrected_by"]:
            self._corrected_fits(piece, hold)
        width, speeds = hold["width_m"], self.config["speeds"]
        if not self.ahead:
            self.metrics[piece]["attempts"] += 1
        pick, over = self._tcp(hold["pick"], width), self._tcp(shifted(hold["place"], hold["axis"] * hold["clear_m"]), width)
        hold["path"] = {"hover": shifted(pick, [0, 0, HOVER_M]), "pick": pick, "lift": shifted(pick, [0, 0, LIFT_M]),
                        "carry": shifted(over, [0, 0, CARRY_M]), "over": over, "place": self._tcp(shifted(hold["place"], hold["axis"] * 0.001), width)}
        self.log.note(grasp=self._grasp_plan(piece, hold), corrected_by=hold["corrected_by"], slot_axis=[round(float(v), 4) for v in hold["axis"]],
                      waypoints=[waypoint(name, at) for name, at in hold["path"].items()])
        self.say(f"{piece}: approaching")
        self.cell.open_to(width + self.config["open_beyond_m"])
        self.cell.gripper_carry(hold["path"]["hover"], speed_deg_s=speeds["joint_deg_s"])
        self.cell.gripper_line(pick, speeds["approach_m_s"])
        self.held = hold
        size = np.max([c + h for c, h in self.pack.shape(piece).boxes], axis=0) - np.min([c - h for c, h in self.pack.shape(piece).boxes], axis=0)
        return {"commanded": se3(pick), "width_m": width,
                "target_obb": {"center": dict(zip("xyz", map(float, self.estimate[piece][:3, 3]))),
                               "extent": dict(zip("xyz", map(float, size / 2))), "rotation": se3(self.estimate[piece])["orientation"]}}

    def _corrected_fits(self, piece, hold):
        """Refuse a corrected place at which the gripper does not fit. The grasp check passed the place as defined;
        a correction moves the gripper from there (a bar's lowers it toward the slot's walls), so the same check is
        made again where the gripper will really be."""
        with self.cell.timed("plan_s"):
            why = assembly.place_blocked(self.pack, piece, hold["width_m"], hold["place"], hold["seat"], assembly.solids(self.pack, hold["others"]),
                                         cell.pad_setback, hold["axis"], hold["clear_m"])
        if why:
            raise Misfit(f"the place of {piece} corrected from the {hold['corrected_by']} does not fit: {why}")

    def _table_shift(self, piece, flipped):
        """The table's shift for a piece, for a hold with the gripper as defined or half round from it. An entry was
        measured one way round; the other way round it is turned with the gripper (the two fingers are alike, so
        the piece sits in the turned gripper as it sat in the other: reasoned from that, not measured)."""
        row = self.table[piece]
        return row["in_hand"] if flipped == row["flipped"] else HALF_TURN @ row["in_hand"] @ HALF_TURN

    @stepped("close")
    def close(self, piece):
        """Close the RG2 on the piece at the configured force."""
        self.say(f"{piece}: gripping at {self.config['grip_force_n']:g} N")
        reached = self.cell.grip(self.held["width_m"], self.config["grip_force_n"])
        if not self.ahead:
            self.metrics[piece]["grip_gap_mm"] = round(reached * 1000, 2)
        return {"gap_m": reached}

    @stepped("lift")
    def lift(self, piece):
        """Lift the gripped piece straight up."""
        before = self.truth()[piece]
        self.say(f"{piece}: lifting")
        self.log.note(waypoints=[waypoint("lift", self.held["path"]["lift"])])
        self.cell.gripper_line(self.held["path"]["lift"], self.config["speeds"]["approach_m_s"])
        rise = float(self.truth()[piece][2, 3] - before[2, 3])
        if not self.ahead:
            self.metrics[piece].update(grasp_success=bool(rise > 0.8 * LIFT_M), lift_rise_mm=round(rise * 1000, 1))
        return {"rise_m": rise}

    @stepped("insert")
    def insert(self, piece):
        """Carry the held piece over its slot, lower it along the slot axis, let go, back away and rise."""
        hold, speeds = self.held, self.config["speeds"]
        width, path = hold["width_m"], hold["path"]
        if not self.ahead and not self._look_check(piece):
            return {"blocked": None, "result": "rescan"}
        with self.log.step("carry", piece, waypoints=[waypoint("carry", path["carry"]), waypoint("over", path["over"])]):
            self.say(f"{piece}: carrying it over its slot")
            self.cell.gripper_carry(path["carry"], speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(path["over"], speeds["approach_m_s"])
        gripper = self.cell.gripper_pose()
        hold["in_hand"] = np.linalg.inv(self.truth()[piece]) @ shifted(gripper, -gripper[:3, 2] * cell.pad_setback(width))
        if not self.ahead:                                  # how far the piece sits from the grasp as defined: what friction and grip force set
            defined = np.linalg.inv(hold["in_piece"])
            sits = [assembly.pose(turn, shift) @ hold["in_hand"] for turn, shift in self.pack.shape(piece).symmetries]
            self.metrics[piece]["in_hand_shift_mm"] = round(1000 * min(float(np.linalg.norm((defined @ sat)[:3, 3])) for sat in sits), 2)
            sat = min(sits, key=lambda sat: float(np.linalg.norm((defined @ sat)[:3, 3])))
            self.metrics[piece]["in_hand"] = {"what": "the gripper in the piece's frame, as it holds it over the slot", "xyz_mm": np.round(1000 * sat[:3, 3], 2).tolist(),
                                              "rotation": np.round(sat[:3, :3], 5).tolist(), "defined": np.round(hold["in_piece"], 5).tolist()}
            assumed = np.linalg.inv(np.linalg.inv(hold["seat"]) @ hold["place"])        # how the place assumes it sits: the grasp, or the grasp corrected
            self.metrics[piece].update(corrected_by=hold["corrected_by"],
                                       place_assumed_off_mm=round(1000 * min(float(np.linalg.norm((assumed @ sat)[:3, 3])) for sat in sits), 2))
        if self.ahead and piece not in self.adjust:         # simulating ahead: the twin shows how the piece sits in the hand, so the
            self.adjust[piece] = hold["in_hand"]            # place is corrected by it before lowering, and the real pick starts with it
            hold.update(place=hold["seat"] @ hold["in_hand"], corrected_by="lookahead")
            self._corrected_fits(piece, hold)
            path.update(over=self._tcp(shifted(hold["place"], hold["axis"] * hold["clear_m"]), width),
                        place=self._tcp(shifted(hold["place"], hold["axis"] * 0.001), width))
            with self.log.step("correct", piece, waypoints=[waypoint("over", path["over"])]):
                self.cell.gripper_line(path["over"], speeds["approach_m_s"])
        if not self.ahead and self._sees_base() and not self._vouch(piece):
            return {"blocked": None, "result": "rescan"}
        with self.log.step("lower", piece, waypoints=[waypoint("place", path["place"])], speed_m_s=speeds["insert_m_s"]) as row:
            self.say(f"{piece}: inserting along the slot axis at {speeds['insert_m_s'] * 1000:g} mm/s")
            try:
                self._descend(piece)
                blocked = None
            except cell.MoveFailed as error:
                blocked = str(error)
            row["result"] = {"blocked": blocked}
        if self.put_back == piece:
            return {"blocked": None, "result": "rescan"}
        with self.log.step("release", piece):
            self.cell.open_to(width + self.config["open_beyond_m"])
        back = shifted(self.cell.gripper_pose(), -self.cell.gripper_pose()[:3, 2] * RETREAT_M)
        rise = shifted(back, [0, 0, CARRY_M])               # and up, clear of the puzzle, before any large move
        with self.log.step("retreat", piece, waypoints=[waypoint("back", back), waypoint("rise", rise)]):
            self.cell.gripper_line(back, speeds["approach_m_s"])
            self.cell.gripper_line(rise, speeds["approach_m_s"])
            self.cell.step(250)
        self.held = None
        self.mark(f"{piece} inserted")
        return {"blocked": blocked, "result": "inserted"}

    def _seated(self, piece, world, base_at):
        """How far a piece is from its slot in a base, and whether that counts as seated."""
        metres, degrees = assembly.pose_error(self.pack, piece, world[piece], base_at @ self.slots[piece])
        limit = self.config["seated"]
        return {"position_error_mm": round(metres * 1000, 2), "rotation_error_deg": round(degrees, 2),
                "seated": bool(metres <= limit["position_m"] and degrees <= limit["rotation_deg"])}

    @stepped("lookahead")
    def lookahead(self, piece):
        """Simulate the pick and the insertion ahead on Newton, from where the scan says the pieces are. Over the
        slot the try reads how the piece sits in the hand and corrects the place by it before lowering; the real pick
        then uses that grasp. A try in which the arm is blocked is repeated with the next way of holding the piece.
        Then go or abort. The stage is put
        back as it was. Every try is filmed as a clip of its own, ``lookahead/<piece>_<try>``. A try whose corrected
        place would put the gripper into something at the slot ends there, and the answer is abort: held another
        way the piece would be corrected alike.

        With ``--correction table`` or ``none`` nothing is simulated: the answer is go at once, and the place is
        corrected from the table or not at all (``approach``)."""
        self.adjust.pop(piece, None)
        if self.correction != "lookahead":
            self.say(f"{piece}: no lookahead; the place is " + ("corrected from the table" if self.correction == "table" else "not corrected"))
            self.log.note(simulated=False, correction=self.correction)
            return {"verdict": "go", "tries": []}
        self.say(f"{piece}: lookahead on Newton")
        tries, placed = [], list(self.placed)
        self.avoid.pop(piece, None)
        self.cell.checkpoint(self.paths.values())
        self.ahead = True
        try:
            while len(tries) < LOOKAHEAD_TRIES:
                clip = f"{piece}_{len(tries) + 1}"
                self.cell.aside(self.out / "lookahead" / clip)
                with self.log.step("try", piece, clip=clip) as attempt:
                    self.cell.place_bodies({self.paths[name]: self._twin_pose(name) for name in self.estimate})
                    self.cell.step(100)
                    before, self.trying = self.truth(), None
                    try:
                        self.approach(piece)
                        self.close(piece)
                        rise = self.lift(piece)["rise_m"]
                        blocked = self.insert(piece)["blocked"]
                        after = self.truth()
                        moved, most = max((float(np.linalg.norm(after[name][:3, 3] - before[name][:3, 3])), name) for name in after if name != piece)
                        row = {**self._seated(piece, after, after[self.base]), "blocked": blocked, "rise_mm": round(rise * 1000, 1),
                               "others_moved_mm": round(moved * 1000, 2), "moved_most": most}
                        row["go"] = bool(row["seated"] and moved < 0.003)
                    except Misfit as error:
                        row = {"go": False, "seated": False, "misfit": str(error), "during": self.doing}
                    except (Refused, cell.MoveFailed) as error:
                        row = {"go": False, "error": str(error), "during": self.doing}
                    hold = self.trying
                    tries.append({**row, "way": hold and hold["way"], "adjusted": piece in self.adjust})
                    attempt["result"] = tries[-1]
                filmed = self.cell.back()
                self.tries[clip] = {**{key: filmed[key] for key in ("fps", "zero_s", "captions", "samples", "grips")}, "piece": piece}
                self.cell.rewind()
                if row["go"] or hold is None:
                    break
                if "error" not in row:
                    break
                self.avoid.setdefault(piece, set()).add(hold["way"])    # blocked on the way: hold it another way
                self.adjust.pop(piece, None)
        finally:
            self.cell.restore()
            self.ahead, self.placed, self.held = False, placed, None
        verdict = {"verdict": "go" if tries[-1]["go"] else "abort", "tries": tries}
        record = dict(verdict)
        if piece in self.adjust:                            # how the piece sat in the simulated hand: what a table of corrections is made of
            shift = np.linalg.inv(hold["in_piece"]) @ self.adjust[piece]
            record["correction"] = {"in_hand": np.round(shift, 9).tolist(), "flipped": hold["flipped"], "way": hold["way"],
                                    "shift_mm": in_hand_shift(shift)[0], "shift_deg": in_hand_shift(shift)[1]}
        if not tries[-1]["go"]:
            self.adjust.pop(piece, None)
        self.metrics[piece].setdefault("lookahead", []).append(record)
        return verdict

    def _twin_pose(self, name):
        """Where the scan says a piece is; one in the base, 0.1 mm above resting (see puzzle_scan.DROP_IN_M)."""
        if name in self.placed:
            return shifted(self._seat(name), [0, 0, puzzle_scan.DROP_IN_M])
        return self.estimate[name]

    @stepped("rescan")
    def rescan(self, piece):
        """Look again from the survey pose, update where everything is, and say whether the piece is where it should be."""
        self.say(f"{piece}: looking again")
        seen, report = self._look(piece)
        if self.base in seen:                               # a symmetric base can be read half a turn round: keep to the reading it was placed with
            seen[self.base] = assembly.nearest(self.pack, self.base, self.base_goal, seen[self.base])
        looked = dict(seen)
        if piece == self.base:
            self.estimate.update(seen)
            metres, degrees = assembly.pose_error(self.pack, piece, self.estimate[piece], self.base_goal)
            how_far = {"position_error_mm": round(metres * 1000, 2), "rotation_error_deg": round(degrees, 2)}
            if metres > BASE_PLACED_M or degrees > BASE_PLACED_DEG:
                # The plate slips in the fingers on its way (they hold a 7 mm island) and stops short. Where it stopped is
                # now measured, so one more slide from there, a short one, may close the rest.
                if self.base_slides < BASE_SLIDES and metres <= BASE_NEAR_M:
                    return {"result": "retry", **how_far}
                raise Refused(f"the base is {metres * 1000:.0f} mm and {degrees:.0f} degrees from the middle")
            return {"result": "placed", **how_far}
        if self.base in seen:                               # a base with pieces in it is measured worse than the bare one was:
            metres, degrees = assembly.pose_error(self.pack, self.base, seen[self.base], self.estimate[self.base])
            if metres <= BASE_MOVED_M and degrees <= BASE_MOVED_DEG:                 # its pose is replaced only when it has moved
                seen[self.base] = self.estimate[self.base]
        changed = self._changed(seen, piece)                # before the estimate takes this look's poses
        self.estimate.update(seen)
        seated = piece in looked and self.base in looked and self._seated(piece, looked, looked[self.base])    # against this look's own base
        truth = self.truth()
        self.metrics[piece].update(insert_success=self._seated(piece, truth, truth[self.base])["seated"],
                                   seen_seated=bool(seated and seated["seated"]))
        if seated and seated["seated"]:
            self.placed.append(piece)
            self.estimate[piece] = self._seat(piece)
            return {"result": "changed" if changed else "placed", **seated, "changed": changed}
        if piece in seen and self.metrics[piece]["attempts"] < 2 and abs(seen[piece][2, 2]) > 0.99 and seen[piece][2, 3] < 0.02:
            return {"result": "changed" if changed else "retry", **(seated or {}), "changed": changed}
        limit = self.config["seated"]
        if seated and seated["position_error_mm"] <= 2000 * limit["position_m"] and seated["rotation_error_deg"] <= 2 * limit["rotation_deg"] \
                and not self.metrics[piece].get("read_again"):
            # One look reads a piece up to 1.6 mm from where it is, and a seat allows 1.5: a bar that was seated 1.2 mm
            # from the CAD was read 1.6 mm off and the build ended there. A reading this near the seat is not the end:
            # the table is scanned once, and what the scan reads stands.
            self.metrics[piece]["read_again"] = True
            return {"result": "changed", **seated, "changed": sorted({*changed, piece})}
        raise Refused(f"{piece} is not in its slot and cannot be picked again")

    @stepped("release")
    def release(self):
        """Open the gripper: the graph's recovery when it gives up."""
        self.cell.open_to(0.08)
        return {"result": "open"}

    @stepped("picture")
    def picture(self, label):
        """One colour picture from the wrist camera, where it is now: what a vision-language model is shown."""
        clip = self.cell.record("wrist", self.out / "pictures" / label)
        return {"path": str(clip / "rgb/00000.png")}

    def state(self):
        """What Graph-as-Policy's connector reads: the gripper's pose and opening for the stock grasp checks, and
        the simulator's truth for the checkpoints (every piece's pose, and where each should be)."""
        truth = self.truth()
        goals = {f"slot_{name}": assembly.nearest(self.pack, name, truth[name], truth[self.base] @ self.slots[name]) for name in self.slots}
        goals["middle"] = getattr(self, "base_goal", truth[self.base])
        return {"ee_pose": se3(self.cell.gripper_pose()),
                "gripper": {"position": cell.pad_gap(self.cell.joints()["finger_joint"]) / cell.RG2_STROKE_M},
                "bodies": {name: at.tolist() for name, at in truth.items()}, "goals": {name: at.tolist() for name, at in goals.items()}}

    # ---- running the graph ------------------------------------------------------------------------
    def serve(self, graph, trace):
        """Run the graph with Graph-as-Policy's own interpreter and answer its tool calls until it ends."""
        checkout = gap.checkout()
        python = checkout.root / ".venv/bin/python"
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        # The graph's vision-language model is the owner's [planning] endpoint (twinsmith's config), as for the gap
        # tool; no key or provider setting passes from this shell. With none set, the graph's two questions to it
        # go unanswered and the build goes on.
        env = {key: value for key, value in os.environ.items() if not key.startswith(("ROS_", "GAP_", "OPENROUTER_"))}
        env.update(gap.model_env(False, gap.owner_vlm())[1], WORKCELL_SKILLS_PORT=str(server.getsockname()[1]))
        runner = subprocess.Popen([str(python), str(REGISTRY / "puzzle_graph.py"), "run", str(graph), "--trace-dir", str(trace)],
                                  env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(self.out))
        output = None
        try:
            server.settimeout(120)
            try:
                link, _ = server.accept()
            except TimeoutError:
                runner.kill()
                output = runner.communicate()[0]
                raise SystemExit(f"The graph's runner did not connect within 120 s (its output is in gap_run.log): {output[-800:]}") from None
            link.settimeout(None)
            with link, link.makefile("rw") as stream:
                for line in stream:
                    request = json.loads(line)
                    try:
                        if request["tool"] not in self.TOOLS:
                            raise Refused(f"no such tool: {request['tool']}")
                        self._disturb(request)
                        reply = {"ok": True, "result": getattr(self, request["tool"])(**request.get("inputs", {}))}
                    except (Refused, cell.MoveFailed) as error:
                        self.say(f"refused: {error}")
                        self.refusals.append({"tool": request["tool"], "piece": request.get("inputs", {}).get("piece"), "why": str(error)})
                        reply = {"ok": False, "error": str(error)}
                    stream.write(json.dumps(reply) + "\n")
                    stream.flush()
            output = runner.communicate(timeout=120)[0]
        finally:                                            # whatever stopped the run, the runner does not outlive it and its output is kept
            server.close()
            if self.checker is not None:
                self.cell.kit("result = base_check.close(1.0)")
            if runner.poll() is None:
                runner.kill()
            if output is None:
                output = runner.communicate()[0]
            (self.out / "gap_run.log").write_text(output or "")
        result = next((json.loads(line[8:]) for line in output.splitlines() if line.startswith("RESULT: ")), {"success": False, "error": output[-600:]})
        return result

    def _write_log(self, film, started):
        """``run_log.json``: the run as the viewer reads it."""
        (self.out / "run_log.json").write_text(json.dumps({
            "what": "The run as the viewer reads it (scripts/puzzle_viewer.py). Times are simulated seconds unless named wall.",
            "steps": self.log.steps, "looks": self.look_log, "tries": self.tries, "samples": self.cell.samples, "grips": self.cell.grips,
            "film": {key: film[key] for key in ("fps", "zero_s", "cameras", "captions")}, "sample_columns": ["seconds", "rail_m", *cell.ARM, "pad_gap_m"],
            "wall_start": started, "wall_s": round(time.time() - started, 1)}, default=lambda value: value.item()) + "\n")

    def why_stopped(self, graph):
        """Why a build did not finish, in the build's own words; None when every piece is seated and the graph ended in
        success. The graph's own error names the first node that failed, which may be one that decides nothing (the
        question to a vision-language model that is not up); this names what stopped the build: the lookahead that
        said abort, or the last thing the graph was refused."""
        if all(row.get("final_seated") for row in self.metrics.values()) and graph.get("success"):
            return None
        for piece, row in self.metrics.items():
            looks = row.get("lookahead") or []
            if looks and looks[-1]["verdict"] == "abort" and not row["attempts"]:
                last = looks[-1]["tries"][-1]
                return f"the lookahead said abort before {piece}: " + (last.get("error") or last.get("misfit") or (
                    f"its rehearsal seated the piece and moved {last.get('moved_most', 'another piece')} {last['others_moved_mm']} mm"
                    if last.get("seated") else f"its rehearsal left the piece {last['position_error_mm']} mm from its slot"))
        if self.refusals:
            last = self.refusals[-1]
            return f"{last['tool']}" + (f" of {last['piece']}" if last["piece"] else "") + f" was refused: {last['why']}"
        loose = [piece for piece, row in self.metrics.items() if not row.get("final_seated")]
        return f"the graph ended in {graph.get('exit_status')}" + (f" with {loose} not seated" if loose else "")

    def stopped(self, error, started):
        """What is known of a run that an invalid physics state ended. The physics is asked for nothing more (nothing
        it would answer means anything now); the film's recorders are closed, which is drawing and not physics, so
        that ``run_log.json`` is written and the run can still be looked at."""
        try:
            self._write_log(self.cell.stop_film(), started)
        except Exception:  # noqa: BLE001 - a simulator that cannot even close its film leaves no log; the record below is still written
            pass
        return {"completed": False, "stopped": f"PHYSICS_STATE_INVALID: {error}", "pieces": self.metrics,
                "simulated_seconds": round(self.cell.steps / cell.PHYSICS_HZ, 1), "wall_seconds": round(time.time() - started, 1),
                "survey": getattr(self, "survey_report", None), "order": self.order, "perception": self.args.perception, "correction": self.correction}

    def abandon(self):
        """A run that stopped early: close the film's recorders in Kit, so their render products do not stay on the
        simulator. The frames filmed so far are kept. Says nothing when the bridge itself is what failed."""
        if self.cell.film:
            try:
                self.cell.stop_film()
            except Exception:  # noqa: BLE001 - the error that stopped the run is the one to show
                pass

    # ---- what a reader wants ----------------------------------------------------------------------
    def finish(self, started):
        """Final metrics against the simulator's truth, the video and the key frames."""
        self.say("done")
        self.cell.step(500)
        truth = self.truth()
        for piece in self.slots:
            final = self._seated(piece, truth, truth[self.base])
            self.metrics[piece].update(final_position_error_mm=final["position_error_mm"], final_rotation_error_deg=final["rotation_error_deg"],
                                       final_seated=final["seated"])
        film = self.cell.stop_film()
        self._write_log(film, started)
        summary = {"completed": all(row["final_seated"] for row in self.metrics.values()),
                   "final_poses": {name: np.round(at, 6).tolist() for name, at in truth.items()},
                   "simulated_seconds": round(self.cell.steps / cell.PHYSICS_HZ, 1), "wall_seconds": round(time.time() - started, 1),
                   "pieces": self.metrics, "survey": getattr(self, "survey_report", None), "order": self.order,
                   "perception": self.args.perception, "correction": self.correction,
                   "corrections_table": str(self.args.corrections) if self.correction == "table" else None}
        summary["video"] = str(self._video(film))
        return summary

    def _video(self, film):
        """The film as one MP4, the cameras side by side with the step's caption, and the key frames as PNGs."""
        folders = [film["folder"] / name / "rgb" for name in film["cameras"][::-1]]
        count = min(len(list(folder.glob("*.jpg"))) for folder in folders)
        size = (960 * len(folders), 540)
        out = self.out / "build.mp4"
        encoder = subprocess.Popen([_ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                    "-s", f"{size[0]}x{size[1]}", "-framerate", str(3 * film["fps"]), "-i", "-",
                                    "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
        marks = {index: label for label, index in self.marks}
        for index in range(count):
            frame = Image.new("RGB", size)
            for column, folder in enumerate(folders):
                frame.paste(Image.open(folder / f"{index:05d}.jpg").resize((960, 540)), (960 * column, 0))
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 512, size[0], 540), fill=(0, 0, 0))
            draw.text((10, 520), f"{index / film['fps']:6.1f} s   {film['captions'][index]}", fill=(255, 255, 255))
            encoder.stdin.write(frame.tobytes())
            if index in marks:
                frame.save(self.out / f"key_{index:05d}_{marks[index].replace(' ', '_')}.png")
        encoder.stdin.close()
        encoder.wait()
        return out


def write_corrections(path, readings, built, config, pack, seeds, skipped):
    """A table of corrections from a measuring run (``measure``): one entry per piece type, the mean of that type's
    readings, with every reading and how they were made. ``readings`` are the lookaheads' own records of rehearsals
    that seated the piece. An entry is refused on fewer than MEASURE_LEAST readings. With each entry goes a check of
    whether it helps: how far each reading is from the grasp as defined, against how far it is from the mean of the
    other layouts' readings (a reading does not vouch for an entry it is part of)."""
    entries = []
    for group in piece_types(pack, config, config["corrected_from_table"]["pieces"]):
        mine = [reading for reading in readings if reading["piece"] in group]
        if len(mine) < MEASURE_LEAST:
            raise SystemExit(f"No table written: {len(mine)} readings for {group}, and an entry needs {MEASURE_LEAST}.")
        flipped = mine[0]["flipped"]                        # every reading as if held the way the first was
        shifts = [np.array(row["in_hand"]) if row["flipped"] == flipped else HALF_TURN @ np.array(row["in_hand"]) @ HALF_TURN for row in mine]
        mean, xyz = mean_shift(shifts), 1000 * np.array([shift[:3, 3] for shift in shifts])
        other_layouts = lambda index: np.mean([xyz[other] for other in range(len(mine)) if mine[other]["seed"] != mine[index]["seed"]], axis=0)
        uncorrected = [float(np.linalg.norm(row)) for row in xyz]
        corrected = [float(np.linalg.norm(xyz[index] - other_layouts(index))) for index in range(len(mine))]
        median = lambda values: round(float(np.median(values)), 2)
        entries.append({
            "pieces": group, "grasp": {key: config["grasps"][group[0]][key] for key in GRASP_KEYS}, "flipped": flipped,
            "in_hand": np.round(mean, 9).tolist(), "shift_mm": in_hand_shift(mean)[0], "shift_deg": in_hand_shift(mean)[1],
            "readings": len(mine), "spread_mm": [round(float(v), 2) for v in np.std(xyz, axis=0, ddof=1)],
            "check": {"uncorrected_median_mm": median(uncorrected), "corrected_median_mm": median(corrected),
                      "by_piece": {piece: {"readings": sum(row["piece"] == piece for row in mine),
                                           "uncorrected_median_mm": median([u for u, row in zip(uncorrected, mine) if row["piece"] == piece]),
                                           "corrected_median_mm": median([c for c, row in zip(corrected, mine) if row["piece"] == piece])}
                                   for piece in group if any(row["piece"] == piece for row in mine)}},
            "each": [{"seed": row["seed"], "piece": row["piece"], "lookahead_verdict": row["verdict"], "xyz_mm": [round(float(v), 2) for v in at],
                      "shift_deg": row["shift_deg"]} for row, at in zip(mine, xyz)]})
    table = {
        "description": "The puzzle build's table of in-hand corrections (--correction table): one entry per piece type, for the pieces "
                       "the build's settings name (corrected_from_table); every other piece is placed by its grasp as defined. A piece "
                       "does not sit in the closed gripper exactly as its grasp is defined on the CAD. in_hand is the gripper as a piece "
                       "of the type sat in it over its slot, in the frame of the gripper as the grasp defines it (4 x 4, metres; +x the "
                       "closing axis, +z out through the fingertips): the mean of the readings in each (xyz_mm is a reading's position "
                       "part). shift_mm and shift_deg are its distance from the grasp as defined; spread_mm the readings' standard "
                       "deviation along x, y and z. check: the median distance of a reading from the grasp as defined (uncorrected) and "
                       "from the mean of the other layouts' readings (corrected); an entry earns its place when the second is the "
                       "smaller. flipped: measured with the gripper half round from the grasp as defined; held the other way round, the "
                       "shift is turned with the gripper. Simulation only: nothing here was measured on a real gripper or a real piece.",
        "measured": {"how": "One lookahead per corrected piece on each of several layouts, on Newton, in one measuring run of "
                            "scripts/puzzle_build.py: after the survey and the base's slide, the piece's pick and insertion is rehearsed "
                            "from the poses the run believes, with the pieces before it in the build order put in the base where the CAD "
                            "has them. Over the slot the piece's pose and the gripper's are read from the simulator. A reading is kept "
                            "when the rehearsal seated the piece. No piece is placed for real.",
                     "command": f"python3 scripts/puzzle_build.py --port <port> --out <folder> --seed {seeds[0]} --mutate-stage --discard-changes "
                                "--perception oracle --write-corrections <this file>",
                     "date": time.strftime("%Y-%m-%d"), "seeds": seeds, "layouts_without_a_reading": skipped,
                     "pack": config["pack"], "base": config["base"], "cell_pack": assembly.cell_pack(built["cell_pack"])[0],
                     "backend": built["backend"], "grip_force_n": config["grip_force_n"], "friction_in_the_solver": built["friction_in_the_solver"]},
        "entries": entries}
    Path(path).write_text(json.dumps(table, indent=1) + "\n")
    for entry in entries:
        print(f"corrections written to {path}: {entry['pieces']} {entry['shift_mm']} mm {entry['shift_deg']} deg from {entry['readings']} readings; "
              f"a reading is {entry['check']['uncorrected_median_mm']} mm from the grasp as defined and "
              f"{entry['check']['corrected_median_mm']} mm from the other layouts' mean (medians)")
    return table


def graph_report(trace, pieces):
    """What the graph's own checks said, from its trace: the stock reach and grip checks per piece, and the
    vision-language model's two answers."""
    def output(node):
        path = Path(trace) / "node_data" / node / "output.json"
        return json.loads(path.read_text()) if path.is_file() else None
    return {"pieces": {piece: {"reach": output(f"assemble_{piece}.reach"), "grip": output(f"assemble_{piece}.grip")} for piece in pieces},
            "vlm_count": output("count_parts.compare"), "vlm_judge": output("judge_assembly.ask")}


def arguments(argv=None):
    """The command line, checked. The script exits 0 for a build that finished, 1 for one that stopped (``stopped`` in
    ``metrics.json`` says where and why) and 2 for a command line that is refused."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", type=int, required=True, help="the simulator bridge to use; its stage is replaced")
    parser.add_argument("--out", type=Path, required=True, help="folder for metrics.json, the video and the pictures")
    parser.add_argument("--seed", type=int, default=1, help="the random layout")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--colours", type=Path, help="measured piece colours for this camera and light (puzzle_scan.py colours)")
    parser.add_argument("--perception", choices=["sam3", "oracle"], default="sam3",
                        help="oracle reads the simulator's own poses in place of the camera, to tell a handling failure from a seeing one")
    parser.add_argument("--correction", choices=["table", "lookahead", "none"], default="table",
                        help="where a place's in-hand correction comes from. table (the default): one fixed shift per piece type, read "
                             "from --corrections, for the piece types the settings name; nothing is simulated ahead. lookahead: every "
                             "pick is simulated ahead on Newton, which reads the shift for that pick and can say abort. none: the grasp "
                             "as defined, uncorrected")
    parser.add_argument("--corrections", type=Path, default=CORRECTIONS, help="the table of corrections for --correction table")
    parser.add_argument("--write-corrections", type=Path,
                        help=f"a measuring run in place of a build: on {MEASURE_LAYOUTS} layouts (--seed and the seeds after it), the survey "
                             "and the base's slide, then one lookahead per piece the settings want corrected. The mean shift per piece "
                             "type is written to this file as a table of corrections. No piece is placed")
    parser.add_argument("--table-camera", action="store_true", help="draw the config's table camera, for the video")
    parser.add_argument("--mutate-stage", action="store_true", help="required: this run replaces the open stage")
    parser.add_argument("--discard-changes", action="store_true", help="let quick_start drop unsaved edits")
    parser.add_argument("--unqualified-cell-pack", action="store_true",
                        help=f"run on a cell pack other than {assembly.KEEP_OUT_PACK}, the one the gripper and camera keep-out boxes were "
                             "measured on; metrics.json records cell_pack_qualified false")
    parser.add_argument("--allow-concurrent-gpu", action="store_true", help="required with sam3: SAM 3.1 runs on the simulator's GPU")
    parser.add_argument("--scan-cycle", type=Path, help="required with sam3: the multi-view cycle that turns the scan step's views into one estimate "
                                                        "(the puzzle project's cycle/multiview/mv_cycle.sh)")
    parser.add_argument("--disturb", metavar="PIECE", help="move this piece aside just before its first pick, to see the graph scan again and go on")
    parser.add_argument("--shift-base", metavar="PIECE:MM:DEG[:before|descent]", action="append",
                        help="slide the base that far and turn it that much just before this piece's placement is checked (before) or while it "
                             "is lowered (descent), to see the build notice and still seat the piece; may be given more than once")
    args = parser.parse_args(argv)
    if not args.mutate_stage or (args.perception == "sam3" and not args.allow_concurrent_gpu):
        parser.error("pass --mutate-stage, and --allow-concurrent-gpu with SAM 3.1: the run replaces the stage and starts GPU jobs beside the simulator")
    if args.write_corrections:
        args.correction = "lookahead"                       # a measuring run reads its shifts in lookaheads, whatever a build would do
    if args.perception == "sam3" and not (args.scan_cycle and args.scan_cycle.is_file()):
        parser.error("pass --scan-cycle with SAM 3.1: the scan step combines its views with that cycle")
    return args


def scattered(pack, config, seed):
    """The random layout of a seed: {piece: pose}."""
    target = cell.pose(puzzle_scan.yaw_matrix(math.radians(config["base_target"]["yaw_deg"])),
                       [*config["base_target"]["position_m"], pack.shape(config["base"]).rests[0].drop])
    return assembly.scatter(pack, config["base"], target, np.random.default_rng(seed),
                            blind=assembly.hidden_on_table(config["survey"]["look_at_m"], config["survey"]["height_m"]))


def main():
    args = arguments()
    config = json.loads(args.config.read_text())
    pack = puzzle_scan.load_pack(config["pack"], args.colours)
    if args.write_corrections:
        measure(args, config, pack)
        return
    build, started = Build(args, config, pack), time.time()
    try:
        record = run(build, args, config, scattered(pack, config, args.seed), started)
    except BaseException:
        build.abandon()
        raise
    print(json.dumps({key: record[key] for key in ("completed", "stopped", "simulated_seconds", "wall_seconds", "video", "viewer")}, indent=1))
    if record["stopped"]:                                   # 0 only for a build that finished; metrics.json says where this one stopped
        raise SystemExit(1)


def run(build, args, config, layout, started):
    """Build the cell, author the graph, serve it and write the run's records; returns ``metrics.json``'s content.
    ``stopped`` in it is None for a build that finished (every piece seated, the graph ended in success) and
    otherwise says in words what stopped it. A run that the simulator ends with values that are not finite numbers
    stops there: ``metrics.json`` and ``run_log.json`` are written with what was known, and the script exits with an
    error."""
    record = {"seed": args.seed, "layout": {name: at.tolist() for name, at in layout.items()}, "cell": build.start(layout)}
    graph = build.out / "graph"
    plan = build.out / "plan.json"
    plan.write_text(json.dumps({"base": config["base"], "order": build.order}, indent=1) + "\n")
    authored = subprocess.run([str(gap.checkout().root / ".venv/bin/python"), str(REGISTRY / "puzzle_graph.py"), "author", str(plan), str(graph)],
                              capture_output=True, text=True)
    if authored.returncode:
        raise SystemExit(f"The graph was not authored: {authored.stdout[-800:]}{authored.stderr[-800:]}")
    try:
        record["graph"] = {"folder": str(graph), "run": build.serve(graph, build.out / "trace")}
        record["graph"]["checks"] = graph_report(build.out / "trace", build.slots)
        record.update(build.finish(started))
        record["base_checks"] = build.base_log
        record["stopped"] = build.why_stopped(record["graph"]["run"])
    except cell.PhysicsStateInvalid as error:
        record.update(build.stopped(error, started))
        (build.out / "metrics.json").write_text(json.dumps(record, indent=1) + "\n")
        raise SystemExit(f"The build stopped: {error}. {build.out / 'metrics.json'} has what was known until then.") from None
    (build.out / "metrics.json").write_text(json.dumps(record, indent=1) + "\n")
    record["viewer"] = str(puzzle_viewer.build(build.out))
    return record


def measure(args, config, pack):
    """The measuring run for a table of corrections. On each of MEASURE_LAYOUTS layouts (``--seed`` and the seeds
    after it): the survey and the base's slide as in a build, then one lookahead for each piece the settings want
    corrected, with the pieces before it in the build order put in the base where the CAD has them (as a lookahead in
    a build has the pieces already placed), and nothing placed. The graph is not run. A layout whose base cannot be
    put in the middle, or on which the simulator returns values that are not finite, gives no further reading; the
    next layout builds the scene again. Writes the table, and every reading to ``<out>/readings.json``."""
    wanted, seeds = config["corrected_from_table"]["pieces"], list(range(args.seed, args.seed + MEASURE_LAYOUTS))
    readings, built, skipped = [], None, []
    for seed in seeds:
        build = Build(argparse.Namespace(**{**vars(args), "seed": seed, "out": Path(args.out) / f"layout_{seed}"}), config, pack)
        try:
            cell_built = build.start(scattered(pack, config, seed))
            if built and cell_built["friction_in_the_solver"] != built["friction_in_the_solver"]:
                raise SystemExit("The solver's friction changed between two layouts of one measuring run.")
            built = cell_built
            build.survey()
            build.place_base(build.base)
            while build.rescan(build.base)["result"] == "retry":
                build.place_base(build.base)
            for step in build.order:
                piece = step["piece"]
                if piece in wanted:
                    said = build.lookahead(piece)
                    look, last = build.metrics[piece]["lookahead"][-1], said["tries"][-1]
                    print(f"  layout {seed}, {piece}: lookahead said {said['verdict']}; " + (last.get("error") or last.get("misfit") or
                          f"piece {last['position_error_mm']} mm from its slot, another moved {last['others_moved_mm']} mm"), flush=True)
                    if last.get("seated") and "correction" in look:
                        readings.append({"seed": seed, "piece": piece, "verdict": said["verdict"], **look["correction"]})
                build.placed.append(piece)                  # the next rehearsal has it in the base
                build.estimate[piece] = build._seat(piece)
        except (Refused, cell.MoveFailed, cell.PhysicsStateInvalid) as error:
            skipped.append({"seed": seed, "why": str(error)})
            print(f"  layout {seed}: no further reading: {error}", flush=True)
        finally:
            build.abandon()
    Path(args.out).mkdir(parents=True, exist_ok=True)
    (Path(args.out) / "readings.json").write_text(json.dumps({"seeds": seeds, "skipped": skipped, "readings": readings}, indent=1) + "\n")
    return write_corrections(args.write_corrections, readings, built, config, pack, seeds, skipped)


if __name__ == "__main__":
    main()
