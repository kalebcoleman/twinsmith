"""Acceptance scenario for the work cell's rail: move the rail, solve the arm from the new base, grasp a block.

One continuous run against a bridge, on the session's engine:

1. Rail move. The cell is built with one Jenga block that the arm cannot reach from the rail's default position (it
   is further from the robot's base than the UR5e's 0.85 m reach). The same arm call that step 2 uses is tried from
   the default base and must fail. Then the rail is sent to its target with the work cell's own rail command, in the
   real axis's millimetres, along a trapezoid at the configured speed and acceleration.
2. Arm from the new base. An inverse-kinematics call written as before the rail existed (articulation root, link,
   position, orientation, offset; no base link) places the gripper over the block and then on it. The rail must not
   move while the arm does.
3. Grasp. The RG2 closes with its one driven joint, the arm lifts 0.10 m, holds 3 s, puts the block down and lets go.

Nothing is tuned to pass: every step records what was commanded and what was reached, and a step that misses its
check is reported as failed. It mutates the stage, so use a disposable simulator. The block's place is written to a
temporary task file beside a copy of the push_block BOM, so Kit must run on this machine.

    python3 scripts/workcell_rail_acceptance.py --port <port> --mutate-stage [--discard-changes] [--captures <folder>]
"""

import argparse
import json
import math
import shutil
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith.client import BridgeClient, BridgeError  # noqa: E402

ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
SURVEY_DEG = (90, -80, 100, -110, -90, 0)       # tool pointing down over the table, beside the robot
RAIL_M = {"plus": 0.60, "minus": -0.60}         # where the rail goes, metres from the built pose
BLOCK_BEYOND_M = 0.25                           # the block is this far past the rail's target, along the rail
BLOCK_Y_M = -0.42                               # across the table from the rail, as in the push_block task
UR5E_REACH_M = 0.85                             # Universal Robots, UR5e technical details
HOVER_M, LIFT_M = 0.08, 0.10
TCP_ABOVE_TABLE_M = 0.017                       # fingertip pad centres 10 mm above the centre of a block lying flat
PRE_GRASP_GAP_M = 0.040
# The RG2's linkage, from OnRobot's CAD (the pack's rg2.json): pin A, arm length, pad face inboard of pin C, zero pose.
PIN_A_M, ARM_M, PAD_INSET_M, ZERO_RAD = 0.0170, 0.054992, 0.018951, math.atan2(39.592, 38.177)
BLOCK_SIDE_M = 0.024
BLOCK_MASS_KG = 0.015                           # the push_block task's block (jenga.block_mass_kg, unmeasured)


def pad_gap(finger_rad):
    return 2.0 * (PIN_A_M + ARM_M * math.sin(ZERO_RAD + finger_rad) - PAD_INSET_M)


def finger_for_gap(gap_m):
    return math.asin((gap_m / 2.0 + PAD_INSET_M - PIN_A_M) / ARM_M) - ZERO_RAD


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def scale(a, k):
    return [x * k for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    return scale(a, 1.0 / norm(a))


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw]


def rotate(q, v, inverse=False):
    w, x, y, z = q
    if inverse:
        x, y, z = -x, -y, -z
    t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])]
    return [v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0])]


def trapezoid(distance, speed, acceleration):
    """Time and a position function for a move of ``distance`` with a speed and an acceleration limit."""
    ramp = speed / acceleration
    if acceleration * ramp * ramp > distance:
        ramp = math.sqrt(distance / acceleration)
        peak, cruise = acceleration * ramp, 0.0
    else:
        peak, cruise = speed, (distance - acceleration * ramp * ramp) / speed
    total = 2 * ramp + cruise

    def at(t):
        if t <= ramp:
            return 0.5 * acceleration * t * t
        if t <= ramp + cruise:
            return 0.5 * acceleration * ramp * ramp + peak * (t - ramp)
        if t <= total:
            return distance - 0.5 * acceleration * (total - t) ** 2
        return distance
    return total, at


class Run:
    def __init__(self, bridge, end, config, captures):
        self.b, self.end, self.config, self.captures = bridge, end, config, captures
        self.out = {"end": end, "steps": {}, "failed": []}

    def check(self, name, passed, detail):
        self.out["steps"].setdefault(name, {}).update(detail, passed=bool(passed))
        if not passed:
            self.out["failed"].append(name)
        print(f"  {'ok  ' if passed else 'FAIL'} {name}: {json.dumps(detail, default=str)[:420]}")

    def call(self, name, args=None, timeout=300):
        reply = self.b.call(name, {**self.ids, **(args or {})} if name.startswith("sim.") else (args or {}), timeout=timeout)
        if isinstance(reply, dict) and reply.get("status") == "error":
            raise BridgeError(reply.get("message", "error"), reply)
        return reply

    def step(self, steps):
        done = 0
        while done < steps:
            count = min(120, steps - done)
            reply = self.b.call("sim.simulation.step", {**self.ids, "steps": count}, timeout=300)
            if reply.get("status") != "ok":
                raise BridgeError(reply.get("message", "step refused"), reply)
            done += count

    def joints(self):
        state = self.call("sim.joints.state", {"path": self.root})
        return dict(zip(state["names"], state["positions"])), state["names"]

    def position(self, path):
        return self.call("sim.body.state", {"path": path})["world_position"]

    def tool(self):
        """The gripper as positions say it is: body origin, tool axis, finger axis and the tool centre point."""
        body, left, right = (self.position(p) for p in (self.rg2, self.left, self.right))
        axis = unit(sub(scale(add(left, right), 0.5), body))
        return {"body": body, "axis": axis, "fingers": unit(sub(right, left)), "tcp": add(body, scale(axis, self.tcp_from_body))}

    def wrench(self):
        reading = self.b.call("workcell.robot_wrench", {})
        force = reading.get("force_n") or [float("nan")] * 3
        return {"source": reading.get("source"), "force_n": force, "magnitude_n": norm(force),
                "expected_static_n": (reading.get("load_below_sensor") or {}).get("weight_n")}

    def contacts(self):
        reply = self.b.call("sim.physics.contacts", {**self.ids, "paths": self.tips}, timeout=120)
        rows = []
        for pair in reply.get("pairs") or []:
            names = [str(pair.get("collider_a")), str(pair.get("collider_b"))]
            force = pair.get("force") or [0.0, 0.0, 0.0]
            rows.append({"between": [n.rsplit("/", 2)[-2] + "/" + n.rsplit("/", 1)[-1] for n in names], "force_n": force,
                         "with_block": any(n.startswith(self.block) for n in names)})
        return rows

    def capture(self, camera, label):
        if not self.captures:
            return None
        name = f"rail_acceptance_{self.end}_{label}_{self.stamp}.png"
        shot = self.b.call("sim.camera.capture", {**self.ids, "path": self.cameras[camera], "filename": name}, timeout=300)
        if shot.get("file"):
            target = Path(self.captures) / f"{self.end}_{label}.png"
            shutil.copy(shot["file"], target)
            self.out.setdefault("captures", {})[label] = str(target)
        return shot.get("file")

    def ik(self, tcp, orientation):
        """The arm call, as it was written before the rail existed: no base link, no rail."""
        return self.b.call("sim.arm.ik", {**self.ids, "path": self.root, "link": "wrist_3_link", "position": tcp,
                                          "orientation": orientation, "offset": self.tcp_in_wrist, "frame": "world"}, timeout=180)

    def move_arm(self, targets, seconds):
        start, _ = self.joints()
        steps = max(1, int(seconds * 500 / 20))
        for index in range(1, steps + 1):
            blend = index / steps
            self.call("sim.joints.command", {"path": self.root, "targets": {j: start[j] + (targets[j] - start[j]) * blend for j in targets}})
            self.step(20)
        self.step(400)

    def run(self, task_file, discard_changes):
        b = self.b
        before = b.call("sim.stage.info", {})
        if not before.get("stopped"):
            b.call("sim.timeline", {"session_id": before["session_id"], "stage_id": before["stage_id"], "action": "stop"})
        summary = b.call("workcell.workcell_quick_start", {"discard_changes": discard_changes, "task": str(task_file),
                                                           **({"config_path": self.config} if self.config else {})}, timeout=600)
        robot = b.call("workcell.robot_status", {})["robot"]
        rail = robot.get("rail")
        if not rail:
            raise SystemExit("this cell has no moving rail (rail.moving is false)")
        self.root, self.out["backend"] = robot["articulation_root"], summary["backend"]
        gripper = robot["gripper_path"]
        self.rg2, self.left, self.right = (f"{gripper}/{name}" for name in ("onrobot_rg2_base_link", "left_inner_finger", "right_inner_finger"))
        self.tips = [f"{self.left}/fingertip", f"{self.right}/fingertip"]
        self.block = "/World/Jenga/Block_01"
        tool0 = robot["tool"]
        self.tcp_from_body = tool0["tcp_in_tool0_m"][2] - tool0["rg2_body_in_tool0"]["translation_m"][2]
        target_m = RAIL_M[self.end]
        block_xy = [target_m + math.copysign(BLOCK_BEYOND_M, target_m), BLOCK_Y_M]
        stage = b.call("sim.stage.info", {})
        self.ids = {"session_id": stage["session_id"], "stage_id": stage["stage_id"]}
        self.stamp = int(time.time())
        self.cameras = {}
        if self.captures:
            middle = block_xy[0] / 2.0
            views = {"wide": dict(position=[middle, -2.05, 1.25], target=[middle, -0.15, 0.20], fov=62.0),
                     "close": dict(position=[block_xy[0] + math.copysign(0.42, target_m), -0.95, 0.34], target=[block_xy[0], BLOCK_Y_M, 0.10], fov=55.0)}
            for name, view in views.items():
                made = b.call("sim.camera.create", {**self.ids, "path": f"/World/Acceptance_{name}", "position": view["position"], "target": view["target"],
                                                    "frame": "world", "resolution": [1024, 768], "width": 1024, "height": 768,
                                                    "focal_length": 20.955 / (2 * math.tan(math.radians(view["fov"] / 2))),
                                                    "horizontal_aperture": 20.955, "clipping_range": [0.01, 50.0]}, timeout=120)
                self.cameras[name] = made.get("path")
        for action in ("play", "pause"):
            b.call("sim.timeline", {**self.ids, "action": action})
        pose = {joint: math.radians(value) for joint, value in zip(ARM, SURVEY_DEG)}
        self.call("sim.joints.teleport", {"path": self.root, "positions": pose})
        self.call("sim.joints.command", {"path": self.root, "targets": {**pose, "finger_joint": finger_for_gap(PRE_GRASP_GAP_M)}})
        self.step(1000)

        # ---- the frames the arm call needs, from the arm tool's own forward kinematics and from body positions
        fk = self.call("sim.arm.fk", {"path": self.root, "link": "wrist_3_link"})["end_effector_pose"]
        tool = self.tool()
        self.tcp_in_wrist = rotate(fk["orientation_wxyz"], sub(tool["tcp"], fk["position"]), inverse=True)
        yaw = math.atan2(tool["fingers"][1], tool["fingers"][0])           # the finger axis now, about the vertical
        turn = -yaw if abs(yaw) <= math.pi / 2 else math.copysign(math.pi, yaw) - yaw   # bring it onto world X: across the block's 24 mm side
        down = quat_mul([math.cos(turn / 2), 0.0, 0.0, math.sin(turn / 2)], fk["orientation_wxyz"])
        hover = [block_xy[0], block_xy[1], TCP_ABOVE_TABLE_M + HOVER_M]
        grasp = [block_xy[0], block_xy[1], TCP_ABOVE_TABLE_M]
        positions, names = self.joints()
        self.out["joint_names"] = names
        base = self.position(robot["arm_base_link"])
        block_start = self.position(self.block)
        self.out["setup"] = {"rail_default_m": positions["rail_joint"], "rail_target_m": target_m, "block_world_m": block_start,
                             "base_world_m": base, "block_from_default_base_m": norm(sub(block_start[:2], base[:2])),
                             "ur5e_reach_m": UR5E_REACH_M, "tcp_from_rg2_body_m": self.tcp_from_body, "tcp_in_wrist_3_m": self.tcp_in_wrist}
        self.capture("wide", "1_start")

        # ---- 1. the rail move
        try:
            from_default = self.ik(hover, down)
        except BridgeError as error:
            from_default = {"reached": False, "message": str(error)}
        self.check("0_block_is_out_of_reach_from_the_default_base",
                   not from_default.get("reached") and self.out["setup"]["block_from_default_base_m"] > UR5E_REACH_M,
                   {"ik_reached": bool(from_default.get("reached")), "ik_position_error_m": from_default.get("position_error"),
                    "block_from_base_m": round(self.out["setup"]["block_from_default_base_m"], 4), "reach_m": UR5E_REACH_M,
                    "message": str(from_default.get("message", ""))[:160]})
        reading = b.call("workcell.robot_rail", {})                       # the real axis's unit: millimetres from home
        start_mm = reading["position_mm"]
        goal_mm = start_mm + (target_m - positions["rail_joint"]) * rail["real_axis"]["mm_per_joint_m"]
        force_before = self.wrench()
        total, at = trapezoid(abs(goal_mm - start_mm) / 1000.0, rail["speed_m_s"], rail["acceleration_m_s2"])
        direction, elapsed, arm_drift = (1.0 if goal_mm >= start_mm else -1.0), 0.0, 0.0
        while elapsed < total:
            elapsed = min(total, elapsed + 0.02)
            b.call("workcell.robot_rail", {"position_mm": start_mm + direction * at(elapsed) * 1000.0})
            self.step(10)
            now, _ = self.joints()
            arm_drift = max(arm_drift, max(abs(math.degrees(now[j] - pose[j])) for j in ARM))
        b.call("workcell.robot_rail", {"position_mm": goal_mm})
        self.step(500)
        arrived = b.call("workcell.robot_rail", {})
        self.step(1000)
        held = b.call("workcell.robot_rail", {})
        force_after = self.wrench()
        self.check("1_rail_move", abs(arrived["position_mm"] - goal_mm) < 0.5 and abs(held["position_mm"] - arrived["position_mm"]) < 0.1 and arm_drift < 0.5,
                   {"commanded_mm": goal_mm, "reached_mm": arrived["position_mm"], "error_mm": arrived["position_mm"] - goal_mm,
                    "joint_reached_m": arrived["joint_position_m"], "move_seconds": round(total, 2), "settle_seconds": 1.0,
                    "drift_over_2s_mm": held["position_mm"] - arrived["position_mm"], "arm_worst_joint_error_deg": arm_drift,
                    "flange_force_before_n": force_before["magnitude_n"], "flange_force_after_n": force_after["magnitude_n"],
                    "flange_force_expected_n": force_after["expected_static_n"], "base_world_m": self.position(robot["arm_base_link"])})
        self.capture("wide", "2_after_rail_move")

        # ---- 2. the arm, from the new base, with an unchanged call
        rail_before = self.joints()[0]["rail_joint"]
        solved = {}
        for label, target, seconds in (("hover", hover, 4.0), ("grasp", grasp, 2.0)):
            answer = self.ik(target, down)
            solved[label] = answer
            if not answer.get("reached"):
                break
            self.move_arm(answer["joint_targets"], seconds)
            if label == "hover":
                self.capture("close", "3_pre_grasp")
        reached = self.tool()
        rail_after = self.joints()[0]["rail_joint"]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -reached["axis"][2]))))
        across = math.degrees(math.asin(min(1.0, abs(reached["fingers"][1]))))       # the finger axis off world X
        position_error = norm(sub(reached["tcp"], grasp))
        answer = solved.get("grasp") or solved.get("hover") or {}
        self.check("2_arm_ik_from_the_new_base",
                   bool(answer.get("reached")) and "rail_joint" not in (answer.get("joint_targets") or {}) and position_error < 0.003
                   and tilt < 1.0 and across < 1.0 and abs(rail_after - rail_before) < 1e-4,
                   {"call": "sim__arm(action='ik', path=<articulation root>, link='wrist_3_link', position, orientation, offset, frame='world')",
                    "ik_joints": list((answer.get("joint_targets") or {}).keys()), "ik_chain_base": (answer.get("chain") or {}).get("base"),
                    "target_tcp_world_m": grasp, "reached_tcp_world_m": reached["tcp"], "position_error_mm": position_error * 1000.0,
                    "tool_axis_off_vertical_deg": tilt, "finger_axis_off_the_blocks_narrow_side_deg": across,
                    "ik_reported_position_error_m": answer.get("position_error"), "ik_reported_orientation_error_rad": answer.get("orientation_error"),
                    "rail_joint_before_m": rail_before, "rail_joint_after_m": rail_after, "rail_moved_mm": (rail_after - rail_before) * 1000.0})
        if not answer.get("reached"):
            return self.out

        # ---- 3. the grasp: one driven joint, lift, hold, put down, let go
        force_free = self.wrench()
        self.call("sim.joints.command", {"path": self.root, "targets": {"finger_joint": -1.0}, "clamp": True})   # its lower limit
        self.step(1000)
        closed, _ = self.joints()
        gap = pad_gap(closed["finger_joint"])
        touching = [row for row in self.contacts() if row["with_block"]]
        normal_before = [abs(dot(row["force_n"], reached["fingers"])) for row in touching]
        block_before, body_before = self.position(self.block), self.position(self.rg2)
        lift = self.ik(add(grasp, [0.0, 0.0, LIFT_M]), down)
        self.move_arm(lift["joint_targets"], 3.0)
        block_lifted, body_lifted = self.position(self.block), self.position(self.rg2)
        self.capture("close", "4_holding")
        self.step(1500)
        block_held, body_held = self.position(self.block), self.position(self.rg2)
        holding = [row for row in self.contacts() if row["with_block"]]
        normal_held = [abs(dot(row["force_n"], reached["fingers"])) for row in holding]
        force_holding = self.wrench()
        slip_lift = norm(sub(sub(block_lifted, body_lifted), sub(block_before, body_before)))
        slip_hold = norm(sub(sub(block_held, body_held), sub(block_lifted, body_lifted)))
        weight = BLOCK_MASS_KG * 9.81
        rise = force_holding["magnitude_n"] - force_free["magnitude_n"]
        self.check("3_grasp_lift_hold",
                   len(touching) == 2 and len(holding) == 2 and block_held[2] - block_before[2] > LIFT_M - 0.005 and slip_hold < 0.001 and abs(rise - weight) < 0.02,
                   {"closed_with": "finger_joint alone, at its lower limit", "finger_joint_deg": math.degrees(closed["finger_joint"]), "pad_gap_mm": gap * 1000.0,
                    "penetration_per_side_mm": (BLOCK_SIDE_M - gap) / 2.0 * 1000.0, "fingertip_normal_force_before_lift_n": normal_before,
                    "fingertip_normal_force_holding_n": normal_held, "block_lift_m": block_held[2] - block_before[2],
                    "slip_during_lift_mm": slip_lift * 1000.0, "slip_over_3s_hold_mm": slip_hold * 1000.0,
                    "flange_force_free_n": force_free["magnitude_n"], "flange_force_holding_n": force_holding["magnitude_n"],
                    "flange_force_rise_n": rise, "block_weight_n": weight, "rail_joint_m": self.joints()[0]["rail_joint"]})
        self.move_arm(self.ik(grasp, down)["joint_targets"], 3.0)
        self.call("sim.joints.command", {"path": self.root, "targets": {"finger_joint": finger_for_gap(PRE_GRASP_GAP_M)}})
        self.step(1000)
        self.move_arm(self.ik(hover, down)["joint_targets"], 2.0)
        self.step(500)
        rest = self.position(self.block)
        left_over = [row for row in self.contacts() if row["with_block"]]
        self.check("4_release", not left_over and abs(rest[2] - block_start[2]) < 0.003 and norm(sub(rest[:2], block_start[:2])) < 0.01,
                   {"fingertip_contacts_with_the_block": len(left_over), "block_world_m": rest, "moved_from_its_start_mm": norm(sub(rest, block_start)) * 1000.0})
        return self.out


def task_file(folder, end):
    """A copy of the push_block task with its one block out of reach of the default base, and of its BOM beside it."""
    tasks = ROOT / "exts/workcell/tasks"
    task = json.loads((tasks / "push_block.json").read_text())
    target = RAIL_M[end]
    task["jenga"]["block_poses"]["value"] = [[target + math.copysign(BLOCK_BEYOND_M, target), BLOCK_Y_M, 90.0]]
    task["jenga"]["block_poses"]["source"] = "scripts/workcell_rail_acceptance.py: one block beyond the arm's reach from the rail's default position."
    shutil.copy(tasks / task["documentation"]["bom_csv"]["value"], folder / task["documentation"]["bom_csv"]["value"])
    path = folder / f"rail_acceptance_{end}.json"
    path.write_text(json.dumps(task, indent=2))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--config", help="absolute hardware configuration path on the Kit host; default: the shipped one")
    parser.add_argument("--end", choices=("plus", "minus", "both"), default="both", help="which end of the rail's travel to run at")
    parser.add_argument("--captures", help="folder for the four camera captures of each run")
    parser.add_argument("--out", help="write the results as JSON here")
    parser.add_argument("--mutate-stage", action="store_true", required=True, help="acknowledge that the stage is replaced")
    parser.add_argument("--discard-changes", action="store_true", help="allow replacement of an unsaved stage")
    args = parser.parse_args()
    if args.config and not Path(args.config).is_absolute():
        parser.error("--config must be absolute on the Kit host")
    if args.captures:
        Path(args.captures).mkdir(parents=True, exist_ok=True)
    results = {}
    folder = Path(tempfile.mkdtemp(prefix="rail-acceptance-"))
    try:
        with BridgeClient(host=args.host, port=args.port, timeout=300) as bridge:
            for end in (("plus", "minus") if args.end == "both" else (args.end,)):
                print(f"rail acceptance, {end} end of the travel")
                results[end] = Run(bridge, end, args.config, args.captures).run(task_file(folder, end), args.discard_changes)
                if args.out:                                             # kept even if a later run or the stop fails
                    Path(args.out).write_text(json.dumps(results, indent=1, default=str))
            try:
                stage = bridge.call("sim.stage.info", {})
                bridge.call("sim.timeline", {"session_id": stage["session_id"], "stage_id": stage["stage_id"], "action": "stop"}, timeout=120)
            except (BridgeError, TimeoutError) as error:
                print(f"note: the timeline was not stopped: {error}")
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    failed = {end: result["failed"] for end, result in results.items() if result["failed"]}
    print(f"{'FAIL' if failed else 'PASS'} ({', '.join(sorted({r.get('backend', '?') for r in results.values()}))}): "
          + (json.dumps(failed) if failed else "rail move, arm IK from the new base, grasp, lift, hold and release at every end run."))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
