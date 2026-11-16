"""The simulated work cell as a script drives it: build a task, step, park the rail, move the arm, grip, look.

``Session`` talks to one twinsmith simulator bridge on this machine and nothing else; no call here can reach a real
robot. The Kit side is ``workcell_session_kit.py``. Conventions:

* The rail is parked with its own blocking move (``park_rail``, the real axis's millimetres). The arm is then solved
  by six-joint inverse kinematics from the UR base, wherever the rail has put it (``ur_ik``, the UR5e's closed-form
  solution; where the base is comes from the bridge's forward kinematics); no call moves both.
* Newton applies no joint speed limit, so every arm move is a ramp of joint targets, one every 10 ms.
* The gripper frame: origin at the TCP (the centre of the fingertip pads with the pads together, read off the
  fingertip meshes on the stage), +x from the left fingertip to the right one (the closing axis), +z from the
  gripper body out through the fingertips.
* Poses are 4 x 4, world from the thing, for column vectors.
"""

import collections
import contextlib
import json
import math
import shutil
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith.client import BridgeClient, BridgeError  # noqa: E402
from workcell_rail_acceptance import ARM_M, ZERO_RAD, finger_for_gap, pad_gap, trapezoid  # noqa: E402

KIT = Path(__file__).with_name("workcell_session_kit.py")
ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
PHYSICS_HZ = 500
SAMPLE_STEPS = 25                               # physics steps between joint samples: 50 ms
CONTROL_STEPS = 1                               # physics steps between joint targets: every step, 2 ms. A target that
                                                # jumps jerks the gripper, and what it holds slips a little each time:
                                                # sliding the 1 kg puzzle base 0.28 m, it slipped 24 mm in the fingers
                                                # with a target every 10 ms and 7 mm with one every step, at any speed
RG2_STROKE_M = 0.110                            # OnRobot RG2 datasheet: stroke 110 mm, grip force 3 to 40 N
RG2_FORCE_N = (3.0, 40.0)
GRIP_STIFFNESS = 57.3                           # N m per rad: stiff enough that the torque cap, not the spring, sets the grip
OPTICAL_FROM_USD = np.diag([1.0, -1.0, -1.0, 1.0])
UR5E = (0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996)     # Universal Robots' UR5e parameters d1, a2, a3, d4, d5, d6, metres
CLEAR_OF_TABLE_M = (0.075, 0.05, 0.05, 0.05)    # the elbow's and the three wrist joints' centres stay this far above the
                                                # table top: their housings' radii (58 and 38 mm) and a margin


class MoveFailed(Exception):
    """The arm cannot reach a pose, or stopped short of its targets."""


def quat_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quat(r):
    """[w, x, y, z] of a rotation matrix."""
    i = int(np.argmax([np.trace(r), r[0, 0], r[1, 1], r[2, 2]]))
    if i == 0:
        s = 2 * math.sqrt(1 + np.trace(r))
        q = [s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        a, b, c = i - 1, i % 3, (i + 1) % 3
        s = 2 * math.sqrt(1 + r[a, a] - r[b, b] - r[c, c])
        q = [0.0] * 4
        q[0], q[1 + a], q[1 + b], q[1 + c] = (r[c, b] - r[b, c]) / s, s / 4, (r[b, a] + r[a, b]) / s, (r[c, a] + r[a, c]) / s
    return [float(v) for v in q]


def pose(rotation, position):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rotation, position
    return out


def blend(start, goal, part):
    """The pose a part of the way from one pose to another: position along the line, rotation about one axis."""
    turn = Rotation.from_matrix(goal[:3, :3] @ start[:3, :3].T).as_rotvec()
    return pose(Rotation.from_rotvec(turn * part).as_matrix() @ start[:3, :3], start[:3, 3] + (goal[:3, 3] - start[:3, 3]) * part)


def _link(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def ur_fk(joints):
    """The UR5e's link frames in its base frame for six joint angles, shoulder pan first: [4 x 4] * 6. The last is
    wrist_3_link's."""
    d1, a2, a3, d4, d5, d6 = UR5E
    out, at = [], np.eye(4)
    for theta, d, a, alpha in zip(joints, (d1, 0, 0, d4, d5, d6), (0, a2, a3, 0, 0, 0), (math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0)):
        at = at @ _link(theta, d, a, alpha)
        out.append(at)
    return out


def ur_ik(target):
    """Every set of six joint angles that puts wrist_3_link at a pose in the UR5e's base frame: up to eight, each
    angle within half a turn of zero. The arm's closed-form solution: shoulder left or right, wrist up or down,
    elbow up or down."""
    d1, a2, a3, d4, d5, d6 = UR5E
    out, back = [], np.linalg.inv(target)
    wrist = target @ [0, 0, -d6, 1]
    reach = math.hypot(wrist[0], wrist[1])
    if reach < d4:
        return out
    for shoulder in (1, -1):
        q1 = math.atan2(wrist[1], wrist[0]) + shoulder * math.acos(d4 / reach) + math.pi / 2
        s1, c1 = math.sin(q1), math.cos(q1)
        c5 = (target[0, 3] * s1 - target[1, 3] * c1 - d4) / d6
        if abs(c5) > 1:
            continue
        for q5 in (math.acos(c5), -math.acos(c5)):
            s5 = math.sin(q5)
            if abs(s5) < 1e-6:                              # the wrist's two outer axes are in line: no single answer
                continue
            q6 = math.atan2((-back[1, 0] * s1 + back[1, 1] * c1) / s5, (back[0, 0] * s1 - back[0, 1] * c1) / s5)
            arm = np.linalg.inv(_link(q1, d1, 0, math.pi / 2)) @ target @ np.linalg.inv(_link(q5, d5, 0, -math.pi / 2) @ _link(q6, d6, 0, 0))
            x, y = arm[:2, 3] - d4 * arm[:2, 1]
            c3 = (x * x + y * y - a2 * a2 - a3 * a3) / (2 * a2 * a3)
            if abs(c3) > 1:
                continue
            for q3 in (math.acos(c3), -math.acos(c3)):
                q2 = math.atan2(-y, -x) - math.asin(-a3 * math.sin(q3) / math.hypot(x, y))
                last = np.linalg.inv(_link(q2, 0, a2, 0) @ _link(q3, 0, a3, 0)) @ arm
                out.append([(q + math.pi) % (2 * math.pi) - math.pi for q in (q1, q2, q3, math.atan2(last[1, 0], last[0, 0]), q5, q6)])
    return out


def pad_setback(gap_m):
    """The pads swing on arcs: at a gap they are this much nearer the gripper body than with the pads together."""
    return ARM_M * (math.cos(ZERO_RAD + finger_for_gap(0.0)) - math.cos(ZERO_RAD + finger_for_gap(gap_m)))


class Session:
    def __init__(self, port, out, host="127.0.0.1"):
        self.out = Path(out).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.bridge = BridgeClient(host=host, port=port)
        self.steps, self.film, self.caption = 0, None, ""
        self.samples, self.grips = [], []                  # [[seconds, rail m, six arm joints rad, pad gap m]] and [[seconds, grip force N]]
        self.spent = collections.Counter()                 # wall seconds by what took them, and the number of bridge calls

    # ---- the bridge -------------------------------------------------------------------------------
    def call(self, name, arguments=None, timeout=300):
        with self.timed("bridge_s"):
            reply = self.bridge.call(name, {**self.ids, **(arguments or {})} if name.startswith("sim.") else (arguments or {}), timeout=timeout)
        if isinstance(reply, dict) and reply.get("status") == "error":
            raise BridgeError(reply.get("message", "error"), reply)
        return reply

    def kit(self, code, timeout=600):
        with self.timed("bridge_s"):
            reply = self.bridge.call("run_python", {"code": code, "session_id": "workcell_session", "persistent": True}, timeout=timeout)
        if reply.get("status") == "error" or reply.get("error"):
            raise BridgeError(str(reply.get("message") or reply.get("error"))[:1500], reply)
        return reply.get("result")

    @contextlib.contextmanager
    def timed(self, what):
        """Count the wall time of a block under a name in ``spent``; a bridge call also counts as one call."""
        started = time.time()
        try:
            yield
        finally:
            self.spent[what] += time.time() - started
            if what == "bridge_s":
                self.spent["bridge_calls"] += 1

    def clock(self):
        """The run's counters now, as a StepLog reads them."""
        return {"sim_s": self.steps / PHYSICS_HZ, "frame": len(self.film["captions"]) if self.film else 0,
                "sample": len(self.samples), "spent": dict(self.spent)}

    def step(self, steps):
        """Advance physics; while filming, one frame per camera every ``film["every"]`` steps."""
        self.drive([{}], steps)

    def drive(self, rows, steps):
        """Give the robot joint targets row by row ([{joint: radians}]) and advance physics ``steps`` after each, in
        one call to Kit per 100 rows. The joints are sampled every SAMPLE_STEPS on the way."""
        watched = [self.dofs[joint] for joint in ("rail_joint", *ARM, "finger_joint")]
        for start in range(0, len(rows), 100):
            part = [{self.dofs[joint]: value for joint, value in row.items()} for row in rows[start:start + 100]]
            film = f"film, {self.film['every']}" if self.film else "(), 0"
            done = self.kit(f"result = await drive({self.root!r}, {part!r}, {steps}, {film}, {self.steps}, {SAMPLE_STEPS})")
            self.steps += len(part) * steps
            self.spent.update(physics_s=done["physics_s"], render_s=done["render_s"])
            for count, *positions in done["samples"]:
                rail, *arm, finger = (positions[index] for index in watched)
                self.samples.append([round(count / PHYSICS_HZ, 3), round(rail, 5), *(round(q, 5) for q in arm), round(pad_gap(finger), 5)])
            if self.film:
                self.film["captions"] += [self.caption] * done["shot"]

    def joints(self):
        state = self.call("sim.joints.state", {"path": self.root})
        return dict(zip(state["names"], state["positions"]))

    def poses(self, bodies=(), attached=None):
        return {path: np.array(m) for path, m in self.kit(f"result = poses({list(bodies)!r}, {attached or {}!r})").items()}

    # ---- the scene --------------------------------------------------------------------------------
    def build(self, task, start_deg, open_gap_m, discard_changes=False, settle_steps=1500):
        """Build the cell with a task (a name or an absolute task file) on Newton, start physics with the arm at
        ``start_deg`` and the gripper open, and let the scene settle. Returns quick_start's answer."""
        before = self.bridge.call("sim.stage.info", {})
        if not before.get("stopped"):
            self.bridge.call("sim.timeline", {"session_id": before["session_id"], "stage_id": before["stage_id"], "action": "stop"})
        built = self.bridge.call("workcell.workcell_quick_start", {"discard_changes": discard_changes, "task": str(task)}, timeout=900)
        if built.get("backend") != "newton":
            raise SystemExit(f"This is for Newton; the bridge runs {built.get('backend')}.")
        self.attach()
        for action in ("play", "pause"):
            self.call("sim.timeline", {"action": action})
        self.dofs = {name: index for index, name in enumerate(self.call("sim.joints.state", {"path": self.root})["names"])}
        home = {joint: math.radians(value) for joint, value in zip(ARM, start_deg)}
        self.posture = [home[joint] for joint in ARM]
        self.call("sim.joints.teleport", {"path": self.root, "positions": home})
        self.commanded = dict(home)
        self.call("sim.joints.command", {"path": self.root, "targets": {**home, "finger_joint": finger_for_gap(open_gap_m)}})
        self.step(settle_steps)
        self.frames()
        return built

    def attach(self):
        """Take up the cell the bridge holds: its robot, gripper and wrist camera. Returns the cell's status."""
        status = self.bridge.call("workcell.workcell_status", {})
        if status.get("state") != "ready":
            raise SystemExit("The bridge holds no work cell; build one first.")
        robot, camera = status["robot"], status["cameras"]["wrist"]
        self.rail, self.root, self.wrist = robot["rail"], robot["articulation_root"], f"{robot['robot_path']}/wrist_3_link"
        gripper = robot["gripper_path"]
        self.rg2, self.fingers = f"{gripper}/onrobot_rg2_base_link", [f"{gripper}/{side}_inner_finger" for side in ("left", "right")]
        self.tcp_from_body = robot["tool"]["tcp_in_tool0_m"][2] - robot["tool"]["rg2_body_in_tool0"]["translation_m"][2]
        intrinsics = {key: camera["intrinsics"][key] for key in ("fx", "fy", "cx", "cy", "width", "height")}
        self.cameras = {"wrist": (camera["prim_path"], self.wrist, intrinsics)}
        stage = self.bridge.call("sim.stage.info", {})
        self.ids = {"session_id": stage["session_id"], "stage_id": stage["stage_id"]}
        self.kit(f"exec(open({str(KIT)!r}).read())")
        self.commanded, self.film = {}, None
        return status

    def frames(self):
        """Where the gripper frame and the camera's optical frame are in wrist_3_link's frame, from the arm's own
        forward kinematics and the live body poses, so no mount offset is assumed."""
        fk = self.call("sim.arm.fk", {"path": self.root, "link": "wrist_3_link"})["end_effector_pose"]
        wrist = pose(quat_matrix(*fk["orientation_wxyz"]), fk["position"])
        now = self.joints()
        self.world_from_base = wrist @ np.linalg.inv(ur_fk([now[joint] for joint in ARM])[-1])    # where the rail has put the UR base
        camera = self.cameras["wrist"][0]
        found = self.poses([self.rg2, *self.fingers], {camera: self.wrist})
        body, left, right = (found[path][:3, 3] for path in (self.rg2, *self.fingers))
        out = (left + right) / 2 - body
        out = out / np.linalg.norm(out)
        closing = right - left
        closing = closing - (closing @ out) * out
        closing = closing / np.linalg.norm(closing)
        across = np.cross(out, closing)
        tips = self.kit(f"result = mesh_centres({ {f'{finger}/fingertip': finger for finger in self.fingers}!r})")
        origin = body + out * self.tcp_from_body
        origin = origin + across * float((np.mean(list(tips.values()), axis=0) - origin) @ across)   # the pads are not on the body's axis
        gripper = pose(np.column_stack([closing, across, out]), origin)
        self.wrist_from_gripper = np.linalg.inv(wrist) @ gripper
        self.wrist_from_camera = np.linalg.inv(wrist) @ found[camera] @ OPTICAL_FROM_USD

    # ---- the robot --------------------------------------------------------------------------------
    def park_rail(self, goal_mm):
        """The rail's own blocking move: a trapezoid of targets at the configured speed, then wait until it holds."""
        start = self.bridge.call("workcell.robot_rail", {})["position_mm"]
        total, at = trapezoid(abs(goal_mm - start) / 1000.0, self.rail["speed_m_s"], self.rail["acceleration_m_s2"])
        elapsed = 0.0
        while elapsed < total:
            elapsed = min(total, elapsed + 0.02)
            self.bridge.call("workcell.robot_rail", {"position_mm": start + math.copysign(at(elapsed) * 1000.0, goal_mm - start)})
            self.step(10)
        self.bridge.call("workcell.robot_rail", {"position_mm": goal_mm})
        self.step(500)
        arrived = self.bridge.call("workcell.robot_rail", {})["position_mm"]
        self.step(500)
        held = self.bridge.call("workcell.robot_rail", {})["position_mm"]
        if abs(arrived - goal_mm) > 0.5 or abs(held - arrived) > 0.1:
            raise SystemExit(f"The rail did not arrive: asked {goal_mm} mm, at {held} mm.")
        self.frames()                                       # the UR base has moved with the rail
        return {"commanded_mm": goal_mm, "reached_mm": held, "move_seconds": round(total, 2)}

    def targets(self):
        """The arm's joint targets now: the last commanded, or where a joint is when it has not been commanded."""
        measured = self.joints()
        return {joint: self.commanded.get(joint, measured[joint]) for joint in ARM}

    def ik(self, world_from_wrist, near=None):
        """Six arm joints that put wrist_3_link at a world pose, from the UR base where the rail has put it: of the
        arm's closed-form solutions, the one nearest ``near`` ({joint: radians}; default the arm's targets now), each
        joint within a turn of zero. Solving a path waypoint by waypoint, each near the one before, keeps it in one
        piece, which the bridge's own solver (nearest the arm's pose now, with no seed) does not."""
        near = near or self.targets()
        best = None
        for joints in ur_ik(np.linalg.inv(self.world_from_base) @ world_from_wrist):
            joints = [q + 2 * math.pi * round((near[joint] - q) / (2 * math.pi)) for joint, q in zip(ARM, joints)]
            joints = [q - math.copysign(2 * math.pi, q) if abs(q) > 2 * math.pi else q for q in joints]
            far = max(abs(q - near[joint]) for joint, q in zip(ARM, joints))
            if best is None or far < best[0]:
                best = (far, joints)
        if best is None:
            raise MoveFailed(f"the arm cannot reach {np.round(world_from_wrist[:3, 3], 3).tolist()}")
        return dict(zip(ARM, best[1]))

    def move(self, waypoints, seconds=None, speed_deg_s=45.0):
        """Drive the joints through joint-space waypoints (one {joint: radians}, or a list), eased in and out over
        the whole path, then check they arrived. Without ``seconds`` the fastest joint averages ``speed_deg_s``."""
        waypoints = [waypoints] if isinstance(waypoints, dict) else list(waypoints)
        now = {**self.joints(), **self.commanded}          # a path starts from the last targets: the arm hangs a little below
        path = [{j: now[j] for j in waypoints[0]}, *waypoints]                       # them, and starting from where it hangs dips it again
        if seconds is None:
            travel = max(sum(abs(b[j] - a[j]) for a, b in zip(path, path[1:])) for j in path[0])
            seconds = max(0.6, math.degrees(travel) / speed_deg_s)
        ticks, rows = max(1, round(seconds * PHYSICS_HZ / CONTROL_STEPS)), []
        for tick in range(1, ticks + 1):
            along = (0.5 - 0.5 * math.cos(math.pi * tick / ticks)) * (len(path) - 1)
            leg = min(int(along), len(path) - 2)
            a, b = path[leg], path[leg + 1]
            rows.append({j: a[j] + (b[j] - a[j]) * (along - leg) for j in a})
        self.drive(rows, CONTROL_STEPS)
        self.commanded.update(path[-1])
        self.step(150)
        now = self.joints()
        worst = max(abs(math.degrees(now[j] - path[-1][j])) for j in path[-1])
        if worst > 0.5:
            raise MoveFailed(f"the arm stopped {worst:.1f} degrees from its targets; it is blocked or cannot hold this pose")

    def gripper_line(self, world_from_gripper, speed_m_s, segment_m=0.01):
        """Move the gripper frame to a world pose along a straight line at a mean speed, turning evenly from its
        orientation now to the goal's: the line is cut into segments, each solved near the one before, and driven as
        one path."""
        start = self.gripper_pose()
        travel = float(np.linalg.norm(world_from_gripper[:3, 3] - start[:3, 3]))
        turn = np.linalg.norm(Rotation.from_matrix(world_from_gripper[:3, :3] @ start[:3, :3].T).as_rotvec())
        count = max(1, math.ceil(travel / segment_m), math.ceil(turn / math.radians(5)))
        back, path = np.linalg.inv(self.wrist_from_gripper), [self.targets()]
        for index in range(1, count + 1):
            path.append(self.ik(blend(start, world_from_gripper, index / count) @ back, near=path[-1]))
        self.move(path[1:], seconds=max(0.4, travel / speed_m_s, turn / math.radians(20)))

    def reconfigure(self, world_from_wrist):
        """Six arm joints that put wrist_3_link at a world pose with the elbow and the wrist clear of the table: of the
        arm's closed-form solutions, the one whose first five joints are nearest the posture the cell was built in,
        each joint within half a turn of that posture. ``ik`` stays with the configuration the arm is in, and a wrist
        that follows it through turn after turn runs into its limit; this returns the arm to one family of
        configurations and unwinds it, so it is the one to ask before a large move."""
        turn = lambda angle: (angle + math.pi) % (2 * math.pi) - math.pi
        found = []
        for joints in ur_ik(np.linalg.inv(self.world_from_base) @ world_from_wrist):
            heights = [(self.world_from_base @ link)[2, 3] for link in ur_fk(joints)[1:5]]
            if all(height >= floor for height, floor in zip(heights, CLEAR_OF_TABLE_M)):
                found.append(joints)
        if not found:
            raise MoveFailed(f"the arm cannot reach {np.round(world_from_wrist[:3, 3], 3).tolist()} with its elbow and wrist clear of the table")
        best = min(found, key=lambda joints: max(abs(turn(q - home)) for q, home in zip(joints[:5], self.posture)))
        return {joint: home + turn(q - home) for joint, q, home in zip(ARM, best, self.posture)}

    def carry(self, world_from_wrist, speed_deg_s=45.0):
        """Take wrist_3_link to a world pose that may be far from, or turned far from, the one it has: one joint-space
        move to the arm's configuration for that pose (``reconfigure``)."""
        self.move(self.reconfigure(world_from_wrist), speed_deg_s=speed_deg_s)

    def gripper_carry(self, world_from_gripper, speed_deg_s=45.0):
        """``carry`` for the gripper frame."""
        self.carry(world_from_gripper @ np.linalg.inv(self.wrist_from_gripper), speed_deg_s)

    def gripper_pose(self):
        fk = self.call("sim.arm.fk", {"path": self.root, "link": "wrist_3_link"})["end_effector_pose"]
        return pose(quat_matrix(*fk["orientation_wxyz"]), fk["position"]) @ self.wrist_from_gripper

    def open_to(self, gap_m, steps=400):
        """Open (or close) the pads to a gap, within the RG2's stroke."""
        if not 0.0 <= gap_m <= RG2_STROKE_M:
            raise ValueError(f"The RG2 opens 0 to {RG2_STROKE_M * 1000:g} mm, not {gap_m * 1000:.1f}")
        self.call("sim.joints.command", {"path": self.root, "targets": {"finger_joint": finger_for_gap(gap_m)}, "clamp": True})
        self.grips.append([self.steps / PHYSICS_HZ, 0.0])
        self.step(steps)

    def grip(self, width_m, force_n, steps=750):
        """Close on something ``width_m`` wide until the drive stalls at ``force_n`` per fingertip. One motor closes
        both fingers, and the pads ride on arms of length ARM_M: by virtual work a drive torque T presses each pad
        with T / (2 ARM_M cos(arm angle)), so the torque cap is set for the asked force at this width. Returns the
        pad gap reached. The finger drive's stiffness and torque cap stay as set here, also after ``open_to`` and
        ``Checkpoint.restore``: the scene keeps them until the cell is built again."""
        if not RG2_FORCE_N[0] <= force_n <= RG2_FORCE_N[1]:
            raise ValueError(f"The RG2 grips with {RG2_FORCE_N[0]:g} to {RG2_FORCE_N[1]:g} N, not {force_n:g}")
        torque = 2 * ARM_M * math.cos(ZERO_RAD + finger_for_gap(width_m)) * force_n
        self.call("sim.joints.gains", {"path": self.root, "dofs": ["finger_joint"], "stiffness": GRIP_STIFFNESS, "max_effort": torque})
        self.call("sim.joints.command", {"path": self.root, "targets": {"finger_joint": finger_for_gap(0.0)}, "clamp": True})
        self.grips.append([self.steps / PHYSICS_HZ, force_n])
        self.step(steps)
        return pad_gap(self.joints()["finger_joint"])

    # ---- the cameras ------------------------------------------------------------------------------
    def add_camera(self, name, camera):
        """A camera that stands still, from a generic description: width, height, fx, fy, position_m and look_at_m.
        Drawn on the stage's session layer; it is not part of the cell."""
        forward = np.subtract(camera["look_at_m"], camera["position_m"])
        forward = forward / np.linalg.norm(forward)
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right = right / np.linalg.norm(right)
        described = {**{key: camera[key] for key in ("width", "height", "fx", "fy")}, "cx": camera["width"] / 2, "cy": camera["height"] / 2}
        at = pose(np.column_stack([right, np.cross(forward, right), forward]), camera["position_m"])
        path = f"/World/Session{name.title()}Camera"
        self.kit(f"result = fixed_camera({path!r}, {dict(described, world_from_camera=at.tolist())!r})")
        self.cameras[name] = (path, None, described)

    def camera_to(self, world_from_camera):
        """``carry`` for the wrist camera's optical frame (+z forward, +y image down)."""
        self.carry(np.asarray(world_from_camera) @ np.linalg.inv(self.wrist_from_camera))

    def record(self, camera, folder, frames=1):
        """A clip of ``frames`` colour and depth stills from one camera, where it is now; returns the clip folder."""
        prim, body, intrinsics = self.cameras[camera]
        folder = Path(folder)
        if folder.exists():
            shutil.rmtree(folder)
        self.kit(f"clip = Recorder({prim!r}, {body!r}, {str(folder)!r}, {[intrinsics['width'], intrinsics['height']]!r}, {intrinsics!r})")
        with self.timed("render_s"):
            for _ in range(frames):
                self.kit("result = await shoot([clip])")
            self.kit("result = clip.close(10.0)")
        return folder

    def start_film(self, folder, fps):
        """From now on every camera films the run at ``fps`` of simulated time, into ``folder/<camera>/rgb``."""
        folder = Path(folder)
        if folder.exists():
            shutil.rmtree(folder)
        made = ", ".join(f"Recorder({prim!r}, {body!r}, {str(folder / name)!r}, {[i['width'], i['height']]!r}, {i!r}, depth=False)"
                         for name, (prim, body, i) in self.cameras.items())
        self.kit(f"film = [{made}]")
        every = round(PHYSICS_HZ / fps)
        self.film = {"folder": folder, "every": every, "fps": fps, "captions": [], "cameras": list(self.cameras),
                     "zero_s": (self.steps // every + 1) * every / PHYSICS_HZ}                # the simulated time of its first frame

    def stop_film(self):
        """Close the film's recorders; returns {"folder", "fps", "captions", "cameras"}."""
        film, self.film = self.film, None
        self.kit(f"result = [recorder.close({film['fps']!r}) for recorder in film]")
        return film

    # ---- simulating ahead -------------------------------------------------------------------------
    def checkpoint(self, bodies):
        """Remember the robot's and these bodies' physics state, to simulate ahead."""
        self.kit(f"checkpoint = Checkpoint({self.root!r}, {list(bodies)!r})")
        self.saved = {"commanded": dict(self.commanded), "steps": self.steps, "caption": self.caption}

    def aside(self, folder):
        """From now the film's cameras write to another folder, counted from zero, and the joint samples go to
        lists of their own: a clip of something tried, apart from the run's film."""
        folder = Path(folder)
        if folder.exists():
            shutil.rmtree(folder)
        self.kit(f"[recorder.switch(path) for recorder, path in zip(film, {[str(folder / name) for name in self.film['cameras']]!r})]")
        self.kept = (self.film, self.samples, self.grips)
        zero = (self.steps // self.film["every"] + 1) * self.film["every"] / PHYSICS_HZ
        self.film, self.samples, self.grips = {**self.film, "folder": folder, "captions": [], "zero_s": zero}, [], []

    def back(self):
        """End the clip ``aside`` began and go on with the run's film. Returns ``{"folder", "fps", "zero_s",
        "captions", "samples", "grips"}`` of the clip."""
        clip = {**{key: self.film[key] for key in ("folder", "fps", "zero_s", "captions")}, "samples": self.samples, "grips": self.grips}
        self.kit("[recorder.back() for recorder in film]")
        self.film, self.samples, self.grips = self.kept
        return clip

    def place_bodies(self, placed):
        """Put rigid bodies at world poses, at rest: {path: 4 x 4}."""
        rows = {path: [*at[:3, 3], *np.roll(matrix_quat(at[:3, :3]), -1)] for path, at in placed.items()}
        self.kit(f"place_bodies({ {path: [float(v) for v in row] for path, row in rows.items()}!r})")

    def rewind(self):
        """Put the remembered physics state back and let the scene rest."""
        self.kit("checkpoint.restore()")
        self.kit(f"result = await drive({self.root!r}, [{{}}], 50)")
        self.commanded = dict(self.saved["commanded"])

    def restore(self):
        """``rewind``, and the run's clock and caption as they were: the steps simulated ahead are not counted."""
        self.rewind()
        self.steps, self.caption = self.saved["steps"], self.saved["caption"]
