"""One robot turn at a Jenga tower, in simulation only: look, predict, push, look again, compare.

Against a disposable Isaac Sim bridge on Newton, with the work cell and a tower task:

1. Park the rail (its own blocking move, in the real axis's millimetres), then move the arm so the wrist D415 faces
   the tower, and record colour and depth.
2. ``sam3`` finds the blocks in that clip and, from depth, each visible block's pose in the world. The blocks are put
   into layers and slots, and the push is aimed at the perceived block, not at where the task file put it.
3. Lookahead: the ``robosim`` tower template pushes each candidate block the same distance on Newton's MuJoCo-Warp,
   and the candidate that leans the tower least is chosen.
4. The arm pushes the chosen block slowly with one RG2 fingertip while the wrist camera records; ``sam3`` runs on
   that clip and on a second look from the first pose.
5. What the tower did in Isaac Sim (the stand-in for the real cell) is compared with what the lookahead predicted
   and with what the camera saw.

Why one fingertip: the two fingertips closed are 22.8 x 20.5 mm, more than a block's 14 mm end, and would push the
layers above and below too. One fingertip is 11.4 mm in the closing direction (its collision shape is measured on
the stage, not assumed). The gripper is opened and turned so
its fingers are one above the other; the lower one pushes and the upper one passes over the top of the tower, which
it does only for a block in the upper part of the tower.

It replaces the stage, starts GPU jobs beside the simulator and writes into ``--out``. Nothing here reaches a real
robot: every call goes to the simulator bridge on this machine.

    python3 scripts/jenga_robot_loop.py --port <port> --task <task.json> --out <folder> --mutate-stage \
        --discard-changes --allow-concurrent-gpu
"""

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith.client import BridgeClient, BridgeError  # noqa: E402
from twinsmith.isaaclab import jobs  # noqa: E402
from twinsmith.robosim.bridge import RoboSimBridge, _ffmpeg_exe  # noqa: E402
from twinsmith.sam3 import Sam3  # noqa: E402
from workcell_rail_acceptance import trapezoid  # noqa: E402

ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
START_DEG = (180, -80, 100, -110, -90, 0)       # the cell's presentation pose turned to face the tower: tool down, elbow up
BLOCK = (0.072, 0.024, 0.014)
SURVEY_DISTANCE_M = 0.45                        # camera to the tower's near face; a real D415 at 1280 x 720 needs about this
FINGER_OPEN_RAD = math.radians(25.0)            # nearly fully open: the fingertips are about 110 mm apart, centre to centre
CONTROL_STEPS = 25                              # physics steps (2 ms each) between joint targets: 50 ms
KIT = Path(__file__).with_name("jenga_robot_loop_kit.py")


def rotation_quat(r):
    """[w, x, y, z] of a rotation matrix."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0] * 4
        q[0], q[1 + i], q[1 + j], q[1 + k] = (r[k, j] - r[j, k]) / s, s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    return [float(v) for v in q]


def tower_change(start, end, moved=()):
    """What a push did to the rest of a tower, from {name: 4 x 4 pose} before and after: the same quantities as the
    robosim tower template's ``tower_state``."""
    names = [name for name in start if name not in moved]
    a, b = (np.array([poses[name] for name in names]) for poses in (start, end))
    heights = a[:, 2, 3]
    low, high = heights <= heights.min() + 0.003, heights >= heights.max() - 0.003
    line = lambda p: p[high, :3, 3].mean(0) - p[low, :3, 3].mean(0)
    u, v = line(a), line(b)
    shift = np.linalg.norm(b[:, :3, 3] - a[:, :3, 3], axis=1)
    tip = np.degrees(np.arccos(np.clip((a[:, :3, 2] * b[:, :3, 2]).sum(1), -1, 1)))
    return {"lean_deg": round(math.degrees(math.acos(float(np.clip(u @ v / np.linalg.norm(u) / np.linalg.norm(v), -1, 1)))), 4),
            "standing": bool(((a[:, 2, 3] - b[:, 2, 3] < BLOCK[2] / 2) & (tip < 20.0)).all()),
            "max_other_shift_m": round(float(shift.max()), 5), "mean_other_shift_m": round(float(shift.mean()), 5),
            "max_other_tip_deg": round(float(tip.max()), 3),
            "top_shift_m": round(float(np.linalg.norm((b[high, :2, 3] - a[high, :2, 3]).mean(0))), 5)}


def tower_from_blocks(objects):
    """Layers and slots from SAM 3's box poses of a tower seen face on from -x.

    A block seen by its end lies along x (an even layer, three ends side by side); one seen by its side lies along y
    (an odd layer, the near block only). Layers are the distinct heights, counted from the table. Returns
    ``{"L<layer>_<slot>": {"centre": [x, y, z], "id": sam3 id}}`` and the tower's axis.
    """
    seen = [(o["id"], o["pose"]) for o in objects if o.get("pose") and o["pose"]["face"] in ("end", "side")
            and o["pose"]["extent_error_m"] < 0.012]
    if not seen:
        raise SystemExit("SAM 3 gave no block pose with a recognisable face.")
    levels, rows = [], sorted(seen, key=lambda item: item[1]["position_world_m"][2])
    for item in rows:                                    # heights within 5 mm are one layer
        if levels and item[1]["position_world_m"][2] - levels[-1][-1][1]["position_world_m"][2] < 0.005:
            levels[-1].append(item)
        else:
            levels.append([item])
    ends = np.array([pose["position_world_m"] for _, pose in seen if pose["face"] == "end"])
    axis = [float(np.median(ends[:, 0])), float((ends[:, 1].min() + ends[:, 1].max()) / 2)]
    blocks = {}
    for layer, level in enumerate(levels):
        for sam_id, pose in level:
            centre = pose["position_world_m"]
            slot = int(round((centre[1] - axis[1]) / BLOCK[1])) + 1 if pose["face"] == "end" else 0
            if 0 <= slot <= 2:
                blocks[f"L{layer:02d}_{slot}"] = {"centre": centre, "id": sam_id, "face": pose["face"]}
    return blocks, axis, len(levels)


def robosim_result(bridge, run):
    """Wait for a robosim run's current operation and return its result."""
    while True:
        status = bridge.call("status", run=run, wait_s=60)
        operation = status.get("operation") or {}
        if operation.get("state") != "running":
            return operation.get("result") or {"ok": False, "error": status.get("error")}


class Loop:
    def __init__(self, args):
        self.args, self.out = args, Path(args.out).resolve()
        self.b = BridgeClient(host=args.host, port=args.port)
        self.work = self.out / "work"
        self.work.mkdir(parents=True, exist_ok=True)
        self.record, self.clock = {"task": str(args.task)}, 0.0        # the clock is simulated seconds stepped

    # ---- the bridge -------------------------------------------------------------------------------
    def call(self, name, arguments=None, timeout=300):
        reply = self.b.call(name, {**self.ids, **(arguments or {})} if name.startswith("sim.") else (arguments or {}), timeout=timeout)
        if isinstance(reply, dict) and reply.get("status") == "error":
            raise BridgeError(reply.get("message", "error"), reply)
        return reply

    def step(self, steps):
        self.clock += steps / 500.0
        while steps > 0:
            count = min(120, steps)
            self.call("sim.simulation.step", {"steps": count})
            steps -= count

    def creep(self, seconds=5.0):
        """How fast the untouched tower moves on its own: the top layer's drift over ``seconds``, in mm per second.
        The work cell's tower creeps on Newton; every before-and-after number here contains some of it."""
        before = self.blocks()
        self.step(round(seconds * 500))
        change = tower_change(before, self.blocks())
        return {"top_mm_per_s": round(1000 * change["top_shift_m"] / seconds, 4), "lean_deg_per_s": round(change["lean_deg"] / seconds, 5),
                "measured_over_s": seconds}

    def undisturbed(self, before, what):
        """Stop if a move bumped the tower: more than its own creep over a move (about 0.5 mm) would explain."""
        if tower_change(before, self.blocks())["max_other_shift_m"] > 0.0015:
            raise SystemExit(f"The arm disturbed the tower {what}.")

    def kit(self, code, timeout=600):
        reply = self.b.call("run_python", {"code": code, "session_id": "jenga_robot_loop", "persistent": True}, timeout=timeout)
        if reply.get("status") == "error" or reply.get("error"):
            raise BridgeError(str(reply.get("message") or reply.get("error"))[:1500], reply)
        return reply.get("result")

    def joints(self):
        state = self.call("sim.joints.state", {"path": self.root})
        return dict(zip(state["names"], state["positions"]))

    def poses(self, bodies=(), attached=None):
        return {path: np.array(m) for path, m in self.kit(f"result = poses({list(bodies)!r}, {attached or {}!r})").items()}

    def blocks(self):
        """Ground truth: every block's live pose in the simulator, by its tower name."""
        found = self.poses(self.block_paths.values())
        return {name: found[path] for name, path in self.block_paths.items()}

    def touching(self, colliders):
        """What these colliders are in contact with now: a tower block's name or the other collider's path, and the force."""
        names = {path: name for name, path in self.block_paths.items()}
        found = []
        for pair in self.call("sim.physics.contacts", {"paths": list(colliders)}, timeout=120).get("pairs") or []:
            ours = [str(pair.get(key)) for key in ("collider_a", "collider_b")]
            other = next((path for path in ours if not any(path.startswith(collider) for collider in colliders)), ours[-1])
            block = next((name for path, name in names.items() if other.startswith(path + "/") or other == path), None)
            force = pair.get("force") or [0.0, 0.0, 0.0]
            found.append({"with": block or other, "force_n": [round(float(v), 3) for v in force]})
        return found

    # ---- the robot --------------------------------------------------------------------------------
    def park_rail(self, goal_mm):
        """The rail's own blocking move: a trapezoid of targets at the configured speed, then wait until it holds."""
        start = self.b.call("workcell.robot_rail", {})["position_mm"]
        total, at = trapezoid(abs(goal_mm - start) / 1000.0, self.rail["speed_m_s"], self.rail["acceleration_m_s2"])
        elapsed = 0.0
        while elapsed < total:
            elapsed = min(total, elapsed + 0.02)
            self.b.call("workcell.robot_rail", {"position_mm": start + math.copysign(at(elapsed) * 1000.0, goal_mm - start)})
            self.step(10)
        self.b.call("workcell.robot_rail", {"position_mm": goal_mm})
        self.step(500)
        arrived = self.b.call("workcell.robot_rail", {})
        self.step(500)
        held = self.b.call("workcell.robot_rail", {})
        if abs(arrived["position_mm"] - goal_mm) > 0.5 or abs(held["position_mm"] - arrived["position_mm"]) > 0.1:
            raise SystemExit(f"The rail did not arrive: asked {goal_mm} mm, at {held['position_mm']} mm.")
        return {"commanded_mm": goal_mm, "reached_mm": held["position_mm"], "move_seconds": round(total, 2)}

    def ik(self, position, rotation, offset=None):
        """Six arm joints that put tool0 (or a point ``offset`` in its frame) at a world pose, from the UR base."""
        answer = self.call("sim.arm.ik", {"path": self.root, "link": "wrist_3_link", "position": [float(v) for v in position],
                                          "orientation": rotation_quat(rotation), "frame": "world", "position_tolerance": 0.0003,
                                          **({"offset": [float(v) for v in offset]} if offset is not None else {})}, timeout=180)
        if not answer.get("reached"):
            raise SystemExit(f"The arm cannot reach {np.round(position, 3).tolist()}: {json.dumps(answer)[:400]}")
        if "rail_joint" in answer["joint_targets"]:
            raise SystemExit("The arm solution moves the rail; the rail is parked separately.")
        return answer["joint_targets"]

    def move(self, targets, seconds, each_tick=None, smooth=True):
        """Ramp the joints to ``targets`` in joint space (Newton applies no joint speed limit of its own): eased in
        and out, or with ``smooth=False`` at a constant rate, for the pieces of one straight push."""
        start = self.joints()
        ticks = max(1, round(seconds * 500 / CONTROL_STEPS))
        for tick in range(1, ticks + 1):
            blend = 0.5 - 0.5 * math.cos(math.pi * tick / ticks) if smooth else tick / ticks
            self.call("sim.joints.command", {"path": self.root, "targets": {j: start[j] + (targets[j] - start[j]) * blend for j in targets}})
            self.step(CONTROL_STEPS)
            if each_tick:
                each_tick(tick)
        if smooth:
            self.step(250)
            now = self.joints()
            worst = max(abs(math.degrees(now[j] - targets[j])) for j in targets)
            if worst > 0.5:                                # a drive that cannot get there: the arm is against something
                raise SystemExit(f"The arm stopped {worst:.1f} degrees from its targets; it is blocked or cannot hold this pose.")

    def look(self, name, frames=3):
        """Record a short clip from where the camera is now; returns the clip folder."""
        folder = self.work / name
        if folder.exists():
            shutil.rmtree(folder)
        self.kit(f"recorder = Recorder({self.camera!r}, {self.wrist!r}, {str(folder)!r}, {self.size!r}, {self.intrinsics!r})")
        for _ in range(frames):
            self.kit("result = await recorder.frame('still')")
        self.kit("result = recorder.close(10.0)")
        return folder

    # ---- perception and prediction ----------------------------------------------------------------
    def segment(self, clip, name, prompts=("tower=tower of wooden blocks", "blocks=wooden block")):
        """Run the sam3 tool on a recorded clip; returns its run folder and report. A ``blocks`` prompt gets box poses."""
        sam3 = Sam3(job_status=jobs.runs_status)
        run = f"{self.out.name}-{name}-{time.strftime('%H%M%S')}"
        boxes = {"blocks": list(BLOCK)} if any(prompt.startswith("blocks=") for prompt in prompts) else None
        started = sam3.call("track", source=str(clip), name=run, prompts=list(prompts), box_size_m=boxes,
                            allow_concurrent_gpu=self.args.allow_concurrent_gpu)
        if not started.get("ok"):
            raise SystemExit(f"sam3 refused: {started.get('error')} {started.get('hint', '')}")
        while jobs.runs_status(started["job_id"])["state"] == "running":
            time.sleep(5)
        report = sam3.call("report", name=run)
        if report.get("state") != "finished":
            raise SystemExit(f"sam3 failed: {json.dumps(report)[:600]}")
        return Path(report["run_dir"]), report

    def lookahead(self, candidates, state):
        """Push each candidate in the robosim tower template; returns one row per candidate."""
        bridge = RoboSimBridge()
        template = {"shape": "tower", "layers": state["layers"], "missing": state["missing"],
                    "table_friction": self.args.table_friction}
        first = bridge.call("start", name="jenga-lookahead", device="cuda:0", template=template,
                            allow_concurrent_gpu=self.args.allow_concurrent_gpu)
        if not first.get("ok"):
            raise SystemExit(f"robosim refused: {first.get('error')}")
        run = first["run"]
        robosim_result(bridge, run)
        rows = []
        try:
            for block in candidates:
                push = {"block": block, "distance": self.args.distance, "seconds": self.args.distance / self.args.speed}
                bridge.call("rollout", run=run, take=f"push-{block}", push=push, duration_s=push["seconds"] + 2.0)
                result = robosim_result(bridge, run)
                applied = result.get("push_applied") or {}
                rows.append({"block": block, "ok": bool(result.get("ok")), "error": result.get("error"),
                             "pushed_m": applied.get("pushed_m"), **(applied.get("tower") or {}),
                             "video": result.get("mp4"), "trajectory": applied.get("trajectory")})
                print(f"  lookahead {block}: {json.dumps({k: rows[-1].get(k) for k in ('lean_deg', 'max_other_shift_m', 'standing', 'pushed_m')})}", flush=True)
        finally:
            bridge.call("stop", run=run)
            robosim_result(bridge, run)
        return rows, run, template

    # ---- the cell ---------------------------------------------------------------------------------
    def build(self, start_deg, before_play=None):
        """Build the cell with the task's tower on Newton, give the blocks a look a camera can use, start physics with
        the arm at ``start_deg`` and the gripper open, and let the tower settle. ``before_play`` is Kit code run on the
        stopped stage. Returns the robot and tower reports."""
        args, b = self.args, self.b
        before = b.call("sim.stage.info", {})
        if not before.get("stopped"):
            b.call("sim.timeline", {"session_id": before["session_id"], "stage_id": before["stage_id"], "action": "stop"})
        built = b.call("workcell.workcell_quick_start", {"discard_changes": args.discard_changes, "task": str(args.task)}, timeout=900)
        if built.get("backend") != "newton":
            raise SystemExit(f"This loop is for Newton; the bridge runs {built.get('backend')}.")
        robot = b.call("workcell.robot_status", {})["robot"]
        camera = b.call("workcell.perception_status", {})["cameras"]["wrist"]
        tower = b.call("workcell.jenga_status", {})["jenga"]
        self.rail, self.root, self.wrist = robot["rail"], robot["articulation_root"], f"{robot['robot_path']}/wrist_3_link"
        self.camera, self.size = camera["prim_path"], [camera["intrinsics"]["width"], camera["intrinsics"]["height"]]
        self.intrinsics = {key: camera["intrinsics"][key] for key in ("fx", "fy", "cx", "cy", "width", "height")}
        self.tool0_from_camera = np.array(camera["optical"]["matrix4x4_row_major"])
        self.block_paths = {f"L{block['layer']:02d}_{block['slot']}": block["prim_path"] for block in tower["blocks"]}
        stage = b.call("sim.stage.info", {})
        self.ids = {"session_id": stage["session_id"], "stage_id": stage["stage_id"]}
        self.record.update(backend=built["backend"], pack=built["pack"], tower={"layers": tower["layers"], "blocks": tower["block_count"],
                                                                               "variation_profile": tower["variation_profile"]})
        self.kit(f"exec(open({str(KIT)!r}).read())")
        self.record["block_look"] = {
            "blocks_dressed": self.kit("result = dress_blocks()"),
            "what": "look only, on the stage: each block is drawn 1.5 mm smaller than it is, in its own shade of the pack's colour",
            "why": "the pack's block is a plain one-colour box with sharp edges; a tower of them renders as a featureless slab"}
        if before_play:
            self.kit(before_play)
        for action in ("play", "pause"):
            self.call("sim.timeline", {"action": action})
        home = {joint: math.radians(value) for joint, value in zip(ARM, start_deg)}
        self.call("sim.joints.teleport", {"path": self.root, "positions": home})
        self.call("sim.joints.command", {"path": self.root, "targets": {**home, "finger_joint": FINGER_OPEN_RAD}})
        self.step(1500)                                    # the tower settles and the gripper opens
        return robot, tower

    def camera_at(self, eye, world_from_camera):
        """Arm joints that put the wrist camera's optical frame at ``eye`` with the axes ``world_from_camera`` (columns)."""
        rotation = np.asarray(world_from_camera) @ self.tool0_from_camera[:3, :3].T
        return self.ik(np.asarray(eye) - rotation @ self.tool0_from_camera[:3, 3], rotation)

    # ---- the turn ---------------------------------------------------------------------------------
    def run(self):
        args = self.args
        self.record.update(candidates=args.candidates, push_distance_m=args.distance, push_speed_m_s=args.speed)
        robot, tower = self.build(START_DEG)

        print("1. rail", flush=True)
        self.record["rail"] = self.park_rail(args.rail_mm)
        self.record["tower_creep_untouched"] = self.creep()
        settled = self.blocks()
        tower_xy = np.mean([pose[:2, 3] for pose in settled.values()], axis=0)
        top = max(pose[2, 3] for pose in settled.values()) + BLOCK[2] / 2
        # Each fingertip's collision shape as a box in tool0: its frame is tool0's turned about the approach axis, so
        # the box stays a box. The end face is the box's far side along the approach (tool0 z).
        paths = [f"{robot['gripper_path']}/{side}_inner_finger" for side in ("left", "right")]
        fingers = self.poses([*paths, self.wrist])
        tips = []
        for path in paths:
            low, high = (np.array(corner) for corner in self.kit(f"result = box_in_body({path + '/fingertip'!r}, {path!r})"))
            corners = np.array([[x, y, z, 1.0] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
            in_tool0 = (np.linalg.inv(fingers[self.wrist]) @ fingers[path] @ corners.T).T[:, :3]
            tips.append((in_tool0.min(0), in_tool0.max(0)))
        self.record["tool"] = {"fingertip_boxes_in_tool0_m": [[low.round(5).tolist(), high.round(5).tolist()] for low, high in tips]}

        print("2. look", flush=True)
        # The camera looks along +x at the middle of the tower's near face, upright (image right is world -y).
        world_from_camera = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        eye = np.array([tower_xy[0] - BLOCK[0] / 2 - SURVEY_DISTANCE_M, tower_xy[1], top / 2])
        survey = self.camera_at(eye, world_from_camera)
        self.move(survey, 5.0)
        self.step(250)
        self.record["survey"] = {"camera_asked_m": eye.round(4).tolist(), "joints_deg": {j: round(math.degrees(v), 2) for j, v in survey.items()}}
        self.undisturbed(settled, "on its way to the survey pose")
        truth_before, time_before = self.blocks(), self.clock
        look_before, report = self.segment(self.look("survey_before"), "before")
        seen, axis, levels = tower_from_blocks(json.loads((look_before / "blocks.json").read_text())["frames"][-1]["objects"])
        errors = {name: float(np.linalg.norm(np.array(block["centre"]) - truth_before[name][:3, 3])) for name, block in seen.items() if name in truth_before}
        self.record["perception_before"] = {
            "sam3_run": str(look_before), "blocks_found": report["prompts"][1]["objects_last_frame"], "blocks_with_pose_in_layers": len(seen),
            "layers_seen": levels, "tower_axis_xy_m": axis, "tower_axis_truth_xy_m": tower_xy.round(5).tolist(),
            "visible_blocks_expected": sum(3 if layer % 2 == 0 else 1 for layer in range(tower["layers"])),
            "centre_error_mm": {"median": round(1000 * float(np.median(list(errors.values()))), 2), "max": round(1000 * max(errors.values()), 2)}}
        print("   ", json.dumps(self.record["perception_before"])[:400], flush=True)
        missing = [c for c in args.candidates if c not in seen]
        if levels != tower["layers"] or missing:
            raise SystemExit(f"The camera found {levels} layers and not these candidates: {missing}.")

        print("3. lookahead", flush=True)
        hidden_assumed = [name for name in self.block_paths if name not in seen]
        if args.predictions:                                # an earlier run's lookahead of the same tower and push
            earlier = json.loads(args.predictions.read_text())
            if (earlier["candidates"], earlier["push_distance_m"], earlier["push_speed_m_s"]) != (args.candidates, args.distance, args.speed):
                raise SystemExit("--predictions is from a run with other candidates or another push.")
            rows, lookahead_run, template = (earlier["lookahead"][key] for key in ("candidates", "robosim_run", "template"))
        else:
            rows, lookahead_run, template = self.lookahead(args.candidates, {"layers": levels, "missing": []})
        usable = [row for row in rows if row["ok"] and row.get("standing")]
        if not usable:
            raise SystemExit("No candidate leaves the predicted tower standing.")
        choice = min(usable, key=lambda row: (row["lean_deg"], row["max_other_shift_m"]))
        self.record["lookahead"] = {"robosim_run": lookahead_run, "template": template, "engine": "Newton MuJoCo-Warp (cuda:0)",
                                    "candidates": rows, "chosen": choice["block"], "rule": "least predicted lean among towers that stay standing",
                                    "blocks_not_seen_and_assumed_present": len(hidden_assumed)}
        print("   chosen", choice["block"], flush=True)

        print("4. push", flush=True)
        target = seen[choice["block"]]
        # Fingers one above the other, the approach along +x: tool0 x (the opening axis) is up, so its y is world -y.
        push_rotation = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
        (low, high), (upper_low, _) = sorted(tips, key=lambda box: box[0][0])        # tool0 -x is down: the lower fingertip first
        tip = np.array([(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, high[2]])     # the middle of the lower fingertip's end face
        other_tip_bottom = target["centre"][2] + upper_low[0] - tip[0]
        if other_tip_bottom < top + 0.005:
            raise SystemExit(f"{choice['block']} is too low for a one-fingertip push: the upper finger would meet the tower "
                             f"({other_tip_bottom:.3f} m against a top at {top:.3f} m).")
        face = np.array(target["centre"]) - [BLOCK[0] / 2, 0.0, 0.0]    # the perceived end face, from the perceived centre
        truth_face = truth_before[choice["block"]][:3, 3] - [BLOCK[0] / 2, 0.0, 0.0]
        stops = [("above", face + [-0.10, 0.0, 0.06], 5.0), ("ready", face + [-0.015, 0.0, 0.0], 3.0)]
        for label, point, seconds in stops:
            self.move(self.ik(point, push_rotation, tip), seconds)
        self.step(250)
        self.undisturbed(truth_before, "on its way to the block")
        # Where the fingertip really is against where it was asked to be (IK tolerance, the arm's droop): the
        # difference is added to every target of the push, which has 1.3 mm of room above and below the fingertip.
        correction = stops[-1][1] - (self.poses([self.wrist])[self.wrist] @ [*tip, 1.0])[:3]
        clip = self.work / "push"
        if clip.exists():
            shutil.rmtree(clip)
        self.kit(f"recorder = Recorder({self.camera!r}, {self.wrist!r}, {str(clip)!r}, {self.size!r}, {self.intrinsics!r})")
        every = max(1, round(0.1 * 500 / CONTROL_STEPS))                # a frame every 0.1 s of simulated time
        ticks = [0]

        def shoot(_):
            ticks[0] += 1
            if ticks[0] % every == 0:
                self.kit("result = await recorder.frame('push')")

        travel = 0.015 + args.distance
        self.kit("result = await recorder.frame('ready')")
        pieces = max(1, round(travel / 0.0025))                         # joint targets every 2.5 mm keep the path straight
        for piece in range(1, pieces + 1):
            point = face + correction + [-0.015 + travel * piece / pieces, 0.0, 0.0]
            self.move(self.ik(point, push_rotation, tip), travel / pieces / args.speed, shoot, smooth=False)
        try:
            touched = self.touching([path + "/fingertip" for path in paths])
        except BridgeError as error:                                    # the contact report is a diagnostic, not the push
            touched = str(error)[:300]
        for _ in range(10):                                             # hold for a second
            self.step(50)
            self.kit("result = await recorder.frame('hold')")
        reached_tip = (self.poses([self.wrist])[self.wrist] @ [*tip, 1.0])[:3]
        truth_pushed = self.blocks()
        self.move(self.ik(face + correction + [-0.04, 0.0, 0.0], push_rotation, tip), 4.0, shoot)
        self.kit("result = await recorder.frame('withdrawn')")
        frames = self.kit("result = recorder.close(10.0)")
        self.move(self.ik(stops[0][1], push_rotation, tip), 3.0)
        self.move(survey, 5.0)
        self.step(500)
        truth_after, elapsed = self.blocks(), self.clock - time_before

        print("5. look again", flush=True)
        look_after, _ = self.segment(self.look("survey_after"), "after")
        seen_after, _, _ = tower_from_blocks(json.loads((look_after / "blocks.json").read_text())["frames"][-1]["objects"])
        feed, feed_report = self.segment(clip, "push")

        real = tower_change(truth_before, truth_after, [choice["block"]])
        pushed = float(truth_after[choice["block"]][0, 3] - truth_before[choice["block"]][0, 3])
        common = [name for name in seen if name in seen_after and name != choice["block"]]
        camera_shift = [float(np.linalg.norm(np.array(seen_after[name]["centre"]) - seen[name]["centre"])) for name in common]
        truth_shift = [float(np.linalg.norm(truth_after[name][:3, 3] - truth_before[name][:3, 3])) for name in common]
        top_names = [name for name in common if name.startswith(f"L{levels - 1:02d}")] or common[-1:]
        self.record["push"] = {
            "block": choice["block"], "aimed_at_perceived_face_m": face.round(5).tolist(), "true_face_m": truth_face.round(5).tolist(),
            "aim_error_mm": round(1000 * float(np.linalg.norm(face - truth_face)), 2), "fingertip_end_in_tool0_m": tip.round(5).tolist(),
            "fingertip_end_face_mm": [round(1000 * float(high[0] - low[0]), 1), round(1000 * float(high[1] - low[1]), 1)],
            "fingertip_reached_m": reached_tip.round(5).tolist(), "fingertip_asked_m": (face + [args.distance, 0.0, 0.0]).round(5).tolist(),
            "fingertip_correction_mm": (1000 * correction).round(2).tolist(), "fingertips_touch_at_the_end_of_the_push": touched,
            "wrist_frames": frames, "pushed_at_hold_m": round(float(truth_pushed[choice["block"]][0, 3] - truth_before[choice["block"]][0, 3]), 5)}
        self.record["comparison"] = {
            "what": "'real' is the Isaac Sim work cell on Newton (the stand-in for the real cell); 'predicted' is the robosim tower "
                    "template's lookahead; 'camera' is SAM 3 with depth from the wrist camera before and after",
            "block": choice["block"],
            "pushed_m": {"asked": args.distance, "predicted": choice["pushed_m"], "real": round(pushed, 5),
                         "camera": round(float(seen_after[choice["block"]]["centre"][0] - target["centre"][0]), 5) if choice["block"] in seen_after else None},
            "tower_lean_deg": {"predicted": choice["lean_deg"], "real": real["lean_deg"]},
            "top_layer_shift_m": {"predicted": choice["top_shift_m"], "real": real["top_shift_m"],
                                  "camera": round(float(np.mean([np.linalg.norm(np.array(seen_after[n]["centre"][:2]) - seen[n]["centre"][:2]) for n in top_names])), 5)},
            "largest_shift_of_another_block_m": {"predicted": choice["max_other_shift_m"], "real": real["max_other_shift_m"],
                                                 "camera_over_the_blocks_it_saw_twice": round(max(camera_shift), 5),
                                                 "real_over_those_blocks": round(max(truth_shift), 5)},
            "standing": {"predicted": choice["standing"], "real": real["standing"]},
            "blocks_seen_before_and_after": len(common),
            "creep_in_the_real_numbers": {
                "simulated_seconds_between_the_two_looks": round(elapsed, 1),
                "top_layer_shift_from_creep_alone_m": round(self.record["tower_creep_untouched"]["top_mm_per_s"] * elapsed / 1000.0, 5),
                "lean_from_creep_alone_deg": round(self.record["tower_creep_untouched"]["lean_deg_per_s"] * elapsed, 3),
                "note": "the untouched tower's drift rate times the time between the looks; its direction is not the push's, so it is an upper bound on what creep adds"}}
        self.record["perception_push_feed"] = {"sam3_run": str(feed), "frames": feed_report["frames"],
                                               "blocks": {key: feed_report["prompts"][1][key] for key in ("objects", "objects_first_frame", "objects_last_frame", "poses")}}
        self.record["perception_after"] = {"sam3_run": str(look_after), "blocks_with_pose_in_layers": len(seen_after)}
        self.save(clip, feed, look_before, look_after, choice)
        return self.record

    def save(self, clip, feed, look_before, look_after, choice):
        """The files a reader wants, copied or encoded into the output folder."""
        ffmpeg, out = _ffmpeg_exe(), self.out
        subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-framerate", "10", "-i", str(clip / "rgb/%05d.png"),
                        "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", str(out / "wrist_cam.mp4")], check=True)
        shutil.copyfile(feed / "overlay.mp4", out / "masks_overlay.mp4")
        for name, run in (("survey_before", look_before), ("survey_after", look_after)):
            subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-i", str(run / "overlay.mp4"), "-frames:v", "1",
                            str(out / f"{name}_masks.png")], check=True)
            shutil.copyfile(self.work / name / "rgb/00000.png", out / f"{name}.png")
        if choice.get("video") and Path(choice["video"]).is_file():
            shutil.copyfile(choice["video"], out / "predicted_push.mp4")
        (out / "comparison.json").write_text(json.dumps(self.record, indent=1) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--task", required=True, type=Path, help="absolute path of a tower task file for the work cell")
    parser.add_argument("--out", required=True, help="folder for the videos and comparison.json")
    parser.add_argument("--candidates", nargs="+", default=["L12_0", "L12_1", "L12_2", "L14_1", "L16_1"],
                        help="blocks to try in the lookahead, L<layer>_<slot>; even layers lie along the push")
    parser.add_argument("--distance", type=float, default=0.025, help="how far the block is pushed, metres")
    parser.add_argument("--speed", type=float, default=0.005, help="push speed, metres per second")
    parser.add_argument("--rail-mm", type=float, default=435.0, help="where the rail is parked, the real axis's millimetres")
    parser.add_argument("--table-friction", type=float, default=0.392,
                        help="block on bench in the lookahead: the larger of the block's and the bench's friction, as Newton combines them")
    parser.add_argument("--predictions", type=Path, help="an earlier run's comparison.json: reuse its lookahead instead of simulating it again")
    parser.add_argument("--mutate-stage", action="store_true", help="required: this run replaces the open stage")
    parser.add_argument("--discard-changes", action="store_true", help="let quick_start drop unsaved edits")
    parser.add_argument("--allow-concurrent-gpu", action="store_true", help="required: SAM 3 and RoboSimStudio run on the simulator's GPU")
    args = parser.parse_args()
    if not args.mutate_stage or not args.allow_concurrent_gpu:
        parser.error("pass --mutate-stage and --allow-concurrent-gpu: the run replaces the stage and starts GPU jobs beside the simulator")
    if not args.task.is_absolute() or not args.task.is_file():
        parser.error("--task must be an existing absolute path")
    record = Loop(args).run()
    print(json.dumps(record["comparison"], indent=1))


if __name__ == "__main__":
    main()
