#!/usr/bin/env python3
"""Build one puzzle with the UR5e, in simulation only: survey, slide the base to the middle, pick and insert each piece.

Against a disposable Isaac Sim bridge on Newton, with the work cell (``ur5e-workcell``) and the ``puzzles`` pack:

1. A random layout: one puzzle's pieces lying loose on the table, its base off to the side (``puzzle_assembly.scatter``).
2. The rail is parked (its own blocking move), then the arm is solved by six-joint IK from the UR base. The wrist D415
   looks straight down, the ``sam3`` tool (SAM 3.1) masks the pieces and ``puzzle_scan`` names and poses them.
3. The build order comes from taking the CAD apart (``puzzle_assembly.build_order``). Graph-as-Policy runs it as a
   graph (``exts/workcell/gap``): this script serves the work cell's skills to that graph, one call at a time.
4. The base is gripped by its centre island and slid to the middle. Each piece is then simulated ahead on Newton
   (``lookahead``: the stage's state is saved, the pieces are put where the scan says they are, the pick and the
   insertion are run, the result is judged and the state is put back), picked, carried over its slot, lowered along
   the slot axis, released, and looked at again from the survey pose.
5. ``metrics.json``: per piece, whether it was named, gripped and seated, how far its final pose is from the CAD's,
   and what the lookahead said; a video from the table camera and the wrist camera, and key frames.

Nothing here reaches a real robot: every call goes to the simulator bridge on this machine. It replaces the stage and
starts GPU jobs (SAM 3.1) beside the simulator.

    python3 scripts/puzzle_build.py --port <port> --out <folder> --seed 1 --mutate-stage --discard-changes \
        --allow-concurrent-gpu [--table-camera] [--colours <colours.json>] [--perception oracle]
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
import puzzle_assembly as assembly  # noqa: E402
import puzzle_scan  # noqa: E402
import puzzle_viewer  # noqa: E402
import workcell_session as cell  # noqa: E402
from step_log import StepLog, stepped  # noqa: E402
from twinsmith import gap  # noqa: E402
from twinsmith.isaaclab import jobs  # noqa: E402
from twinsmith.robosim.bridge import _ffmpeg_exe  # noqa: E402
from twinsmith.sam3 import Sam3  # noqa: E402

CONFIG = Path(__file__).with_name("puzzle_build.json")
REGISTRY = ROOT / "exts/workcell/gap"
PUZZLE = "/World/Puzzle"
HOVER_M, LIFT_M, CARRY_M, RETREAT_M, MARGIN_M = 0.10, 0.12, 0.15, 0.08, 0.02
LOOKAHEAD_TRIES = 5
BASE_MOVED_M, BASE_MOVED_DEG = 0.003, 2.0       # a later look that puts the base further than this from where it was: it has moved
TINTS = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230)]


class Refused(Exception):
    """A skill cannot do what was asked; the graph routes on it."""


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


class Build:
    """The work cell's skills for one layout. Each public method without a leading underscore that ``TOOLS`` names is
    one tool of the graph: it takes and returns plain data."""

    TOOLS = ("survey", "picture", "place_base", "rescan", "lookahead", "approach", "close", "lift", "insert", "release", "state")

    def __init__(self, args, config, pack):
        self.args, self.config, self.pack = args, config, pack
        self.base, self.out = config["base"], Path(args.out).resolve()
        self.cell = cell.Session(args.port, self.out)
        self.order = assembly.build_order(pack, self.base)
        self.slots = assembly.slot_poses(pack, self.base)
        self.names = [self.base, *self.slots]
        self.paths = {name: f"{PUZZLE}/{name}" for name in self.names}
        self.estimate, self.placed, self.held, self.looks, self.marks, self.adjust, self.avoid = {}, [], None, 0, [], {}, {}
        self.metrics = {name: {"attempts": 0} for name in self.slots}
        self.ahead = False                                  # true while simulating ahead: no metric is recorded
        self.log, self.look_log, self.tries = StepLog(self.cell.clock), [], {}
        self.started = time.time()

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
        with self.log.step("build stage"):
            task = puzzle_scan.write_task(assembly.layout_estimate(self.pack, layout), self.out / "task", "puzzle_build.py, a random layout")
            built = self.cell.build(task, self.config["start_joint_deg"], 0.05, self.args.discard_changes)
            if self.args.table_camera:
                self.cell.add_camera("table", self.config["table_camera"])
            self.cell.start_film(self.out / "film", fps=5)
        with self.log.step("park rail", rail_mm=self.config["rail_mm"]) as row:
            self.say("parking the rail")
            row["result"] = rail = self.cell.park_rail(self.config["rail_mm"])
        return {"backend": built["backend"], "cell_pack": built["pack"], "rigid_bodies": built["newton"]["rigid_bodies"], "rail": rail}

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

    def _base_target(self):
        """Where the base goes: the configured place, at the configured yaw or half a turn from it, whichever is nearer."""
        target, now = self.config["base_target"], self.estimate[self.base]
        yaw_now = math.atan2(now[1, 0], now[0, 0])
        yaw = min((math.radians(target["yaw_deg"]) + extra for extra in (0.0, math.pi)),
                  key=lambda value: abs((value - yaw_now + math.pi) % (2 * math.pi) - math.pi))
        return cell.pose(puzzle_scan.yaw_matrix(yaw), [*target["position_m"], now[2, 3]])

    def _grasp(self, piece, now, target, around_now, around_then, axis=(0, 0, 1), clear_m=0.0):
        """The first way of holding a piece that fits the CAD, that the arm reaches at both ends, and that a
        lookahead has not found blocked."""
        grasp = self.config["grasps"][piece]
        back = np.linalg.inv(self.cell.wrist_from_gripper)
        with self.cell.timed("plan_s"):
            options = assembly.choose_grasps(self.pack, piece, grasp, now, target, around_now, around_then, cell.pad_setback, axis, clear_m)
        for way, option in enumerate(options):
            if way in self.avoid.get(piece, ()):
                continue
            try:
                for at in (option["pick"], option["place"], shifted(option["place"], np.array(axis) * clear_m)):
                    self.cell.reconfigure(self._tcp(at, grasp["width_m"]) @ back)
            except cell.MoveFailed:
                continue
            return {**option, "width_m": grasp["width_m"], "piece": piece, "way": way}
        raise Refused(f"no grasp of {piece} fits and is within reach")

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
        self.base_goal = self.estimate[piece] = target      # where it should now be; the next look measures it
        self.mark(f"{piece} in the middle")
        return {"result": "slid", "target": se3(target)}

    def _grasp_plan(self, piece, hold):
        """A chosen grasp as the run's log keeps it: its definition on the CAD and which way of holding was taken."""
        return {**{key: self.config["grasps"][piece][key] for key in ("point_m", "closing_axis", "approach_axis", "width_m", "what")},
                "way": hold["way"], "gripper_in_piece": np.round(hold["in_piece"], 5).tolist(), "force_n": self.config["grip_force_n"]}

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
        loose = {name: at for name, at in self.estimate.items() if name != piece}
        built = {name: self.estimate[name] for name in [self.base, *self.placed]}
        axis = self.estimate[self.base][:3, :3] @ np.array(step["axis"], dtype=float)
        hold = self._grasp(piece, self.estimate[piece], self._seat(piece), loose, built, axis, step["clear_m"] + MARGIN_M)
        hold.update(axis=axis, clear_m=step["clear_m"] + MARGIN_M)
        self.trying = hold
        if piece in self.adjust:                            # the lookahead's grasp: how the piece really sat in the hand
            hold["place"] = hold["seat"] @ self.adjust[piece]
        width, speeds = hold["width_m"], self.config["speeds"]
        if not self.ahead:
            self.metrics[piece]["attempts"] += 1
        pick, over = self._tcp(hold["pick"], width), self._tcp(shifted(hold["place"], hold["axis"] * hold["clear_m"]), width)
        hold["path"] = {"hover": shifted(pick, [0, 0, HOVER_M]), "pick": pick, "lift": shifted(pick, [0, 0, LIFT_M]),
                        "carry": shifted(over, [0, 0, CARRY_M]), "over": over, "place": self._tcp(shifted(hold["place"], hold["axis"] * 0.001), width)}
        self.log.note(grasp=self._grasp_plan(piece, hold), adjusted=piece in self.adjust, slot_axis=[round(float(v), 4) for v in hold["axis"]],
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
        with self.log.step("carry", piece, waypoints=[waypoint("carry", path["carry"]), waypoint("over", path["over"])]):
            self.say(f"{piece}: carrying it over its slot")
            self.cell.gripper_carry(path["carry"], speed_deg_s=speeds["joint_deg_s"])
            self.cell.gripper_line(path["over"], speeds["approach_m_s"])
        gripper = self.cell.gripper_pose()
        hold["in_hand"] = np.linalg.inv(self.truth()[piece]) @ shifted(gripper, -gripper[:3, 2] * cell.pad_setback(width))
        if not self.ahead:                                  # how far the piece sits from the grasp as defined: what friction and grip force set
            defined = np.linalg.inv(hold["in_piece"])
            self.metrics[piece]["in_hand_shift_mm"] = round(1000 * min(float(np.linalg.norm((defined @ assembly.pose(turn, shift) @ hold["in_hand"])[:3, 3]))
                                                                       for turn, shift in self.pack.shape(piece).symmetries), 2)
        if self.ahead and piece not in self.adjust:         # simulating ahead: the twin shows how the piece sits in the hand, so the
            self.adjust[piece] = hold["in_hand"]            # place is corrected by it before lowering, and the real pick starts with it
            hold["place"] = hold["seat"] @ hold["in_hand"]
            path.update(over=self._tcp(shifted(hold["place"], hold["axis"] * hold["clear_m"]), width),
                        place=self._tcp(shifted(hold["place"], hold["axis"] * 0.001), width))
            with self.log.step("correct", piece, waypoints=[waypoint("over", path["over"])]):
                self.cell.gripper_line(path["over"], speeds["approach_m_s"])
        with self.log.step("lower", piece, waypoints=[waypoint("place", path["place"])], speed_m_s=speeds["insert_m_s"]) as row:
            self.say(f"{piece}: inserting along the slot axis at {speeds['insert_m_s'] * 1000:g} mm/s")
            try:
                self.cell.gripper_line(path["place"], speeds["insert_m_s"], segment_m=0.005)
                blocked = None
            except cell.MoveFailed as error:
                blocked = str(error)
            row["result"] = {"blocked": blocked}
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
        return {"blocked": blocked}

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
        back as it was. Every try is filmed as a clip of its own, ``lookahead/<piece>_<try>``."""
        self.say(f"{piece}: lookahead on Newton")
        if self.args.no_lookahead:
            return {"verdict": "go", "tries": []}
        tries, placed = [], list(self.placed)
        self.adjust.pop(piece, None)
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
                        moved = max(float(np.linalg.norm(after[name][:3, 3] - before[name][:3, 3])) for name in after if name != piece)
                        row = {**self._seated(piece, after, after[self.base]), "blocked": blocked, "rise_mm": round(rise * 1000, 1),
                               "others_moved_mm": round(moved * 1000, 2)}
                        row["go"] = bool(row["seated"] and moved < 0.003)
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
        if not tries[-1]["go"]:
            self.adjust.pop(piece, None)
        verdict = {"verdict": "go" if tries[-1]["go"] else "abort", "tries": tries}
        self.metrics[piece].setdefault("lookahead", []).append(verdict)
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
            if metres > 0.01 or degrees > 5.0:
                raise Refused(f"the base is {metres * 1000:.0f} mm and {degrees:.0f} degrees from the middle")
            return {"result": "placed", "position_error_mm": round(metres * 1000, 2), "rotation_error_deg": round(degrees, 2)}
        if self.base in seen:                               # a base with pieces in it is measured worse than the bare one was:
            metres, degrees = assembly.pose_error(self.pack, self.base, seen[self.base], self.estimate[self.base])
            if metres <= BASE_MOVED_M and degrees <= BASE_MOVED_DEG:                 # its pose is replaced only when it has moved
                seen[self.base] = self.estimate[self.base]
        self.estimate.update(seen)
        seated = piece in looked and self.base in looked and self._seated(piece, looked, looked[self.base])    # against this look's own base
        truth = self.truth()
        self.metrics[piece].update(insert_success=self._seated(piece, truth, truth[self.base])["seated"],
                                   seen_seated=bool(seated and seated["seated"]))
        if seated and seated["seated"]:
            self.placed.append(piece)
            self.estimate[piece] = self._seat(piece)
            return {"result": "placed", **seated}
        if piece in seen and self.metrics[piece]["attempts"] < 2 and abs(seen[piece][2, 2]) > 0.99 and seen[piece][2, 3] < 0.02:
            return {"result": "retry", **(seated or {})}
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
        server.settimeout(120)
        link, _ = server.accept()
        link.settimeout(None)
        stream = link.makefile("rw")
        for line in stream:
            request = json.loads(line)
            try:
                if request["tool"] not in self.TOOLS:
                    raise Refused(f"no such tool: {request['tool']}")
                reply = {"ok": True, "result": getattr(self, request["tool"])(**request.get("inputs", {}))}
            except (Refused, cell.MoveFailed) as error:
                self.say(f"refused: {error}")
                reply = {"ok": False, "error": str(error)}
            stream.write(json.dumps(reply) + "\n")
            stream.flush()
        output = runner.communicate(timeout=120)[0]
        (self.out / "gap_run.log").write_text(output)
        result = next((json.loads(line[8:]) for line in output.splitlines() if line.startswith("RESULT: ")), {"success": False, "error": output[-600:]})
        return result

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
        (self.out / "run_log.json").write_text(json.dumps({
            "what": "The run as the viewer reads it (scripts/puzzle_viewer.py). Times are simulated seconds unless named wall.",
            "steps": self.log.steps, "looks": self.look_log, "tries": self.tries, "samples": self.cell.samples, "grips": self.cell.grips,
            "film": {key: film[key] for key in ("fps", "zero_s", "cameras", "captions")}, "sample_columns": ["seconds", "rail_m", *cell.ARM, "pad_gap_m"],
            "wall_start": started, "wall_s": round(time.time() - started, 1)}, default=lambda value: value.item()) + "\n")
        summary = {"completed": all(row["final_seated"] for row in self.metrics.values()),
                   "final_poses": {name: np.round(at, 6).tolist() for name, at in truth.items()},
                   "simulated_seconds": round(self.cell.steps / cell.PHYSICS_HZ, 1), "wall_seconds": round(time.time() - started, 1),
                   "pieces": self.metrics, "survey": getattr(self, "survey_report", None), "order": self.order,
                   "perception": self.args.perception}
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


def graph_report(trace, pieces):
    """What the graph's own checks said, from its trace: the stock reach and grip checks per piece, and the
    vision-language model's two answers."""
    def output(node):
        path = Path(trace) / "node_data" / node / "output.json"
        return json.loads(path.read_text()) if path.is_file() else None
    return {"pieces": {piece: {"reach": output(f"assemble_{piece}.reach"), "grip": output(f"assemble_{piece}.grip")} for piece in pieces},
            "vlm_count": output("count_parts.compare"), "vlm_judge": output("judge_assembly.ask")}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", type=int, required=True, help="the simulator bridge to use; its stage is replaced")
    parser.add_argument("--out", type=Path, required=True, help="folder for metrics.json, the video and the pictures")
    parser.add_argument("--seed", type=int, default=1, help="the random layout")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--colours", type=Path, help="measured piece colours for this camera and light (puzzle_scan.py colours)")
    parser.add_argument("--perception", choices=["sam3", "oracle"], default="sam3",
                        help="oracle reads the simulator's own poses in place of the camera, to tell a handling failure from a seeing one")
    parser.add_argument("--no-lookahead", action="store_true", help="skip simulating each piece ahead")
    parser.add_argument("--table-camera", action="store_true", help="draw the config's table camera, for the video")
    parser.add_argument("--mutate-stage", action="store_true", help="required: this run replaces the open stage")
    parser.add_argument("--discard-changes", action="store_true", help="let quick_start drop unsaved edits")
    parser.add_argument("--allow-concurrent-gpu", action="store_true", help="required with sam3: SAM 3.1 runs on the simulator's GPU")
    args = parser.parse_args()
    if not args.mutate_stage or (args.perception == "sam3" and not args.allow_concurrent_gpu):
        parser.error("pass --mutate-stage, and --allow-concurrent-gpu with SAM 3.1: the run replaces the stage and starts GPU jobs beside the simulator")
    config = json.loads(args.config.read_text())
    pack = puzzle_scan.load_pack(config["pack"], args.colours)
    target = cell.pose(puzzle_scan.yaw_matrix(math.radians(config["base_target"]["yaw_deg"])),
                       [*config["base_target"]["position_m"], pack.shape(config["base"]).rests[0].drop])
    layout = assembly.scatter(pack, config["base"], target, np.random.default_rng(args.seed))
    build, started = Build(args, config, pack), time.time()
    record = {"seed": args.seed, "layout": {name: at.tolist() for name, at in layout.items()}, "cell": build.start(layout)}
    graph = build.out / "graph"
    plan = build.out / "plan.json"
    plan.write_text(json.dumps({"base": config["base"], "order": build.order}, indent=1) + "\n")
    authored = subprocess.run([str(gap.checkout().root / ".venv/bin/python"), str(REGISTRY / "puzzle_graph.py"), "author", str(plan), str(graph)],
                              capture_output=True, text=True)
    if authored.returncode:
        raise SystemExit(f"The graph was not authored: {authored.stdout[-800:]}{authored.stderr[-800:]}")
    record["graph"] = {"folder": str(graph), "run": build.serve(graph, build.out / "trace")}
    record["graph"]["checks"] = graph_report(build.out / "trace", build.slots)
    record.update(build.finish(started))
    (build.out / "metrics.json").write_text(json.dumps(record, indent=1) + "\n")
    record["viewer"] = str(puzzle_viewer.build(build.out))
    print(json.dumps({key: record[key] for key in ("completed", "simulated_seconds", "wall_seconds", "video", "viewer")}, indent=1))


if __name__ == "__main__":
    main()
