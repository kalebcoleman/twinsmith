"""Gravity check for the work cell: at rest, the simulated Bota F/T sensor must read the weight of what it carries.

Builds the work cell, puts the arm in two poses (tool axis down, tool axis horizontal) by a state reset
(teleport) with matching drive targets, so it never sweeps through the bench on the way, and after it settles compares the force magnitude in the sensor frame with the engine's mass of every body below the sensor times g.
Runs on the session's engine, PhysX or Newton. It mutates the stage, so use a disposable simulator.
"""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mcp"))
from twinsmith.client import BridgeClient  # noqa: E402

ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
POSES = {   # degrees
    "tool_down": (0, -80, 100, -110, -90, 0),        # the config's presentation pose
    "tool_horizontal": (0, -80, 100, -10, -90, 0),   # wrist_1 turned so the tool axis lies flat
}
# With the tool axis (the Bota's +x) pointing down, the parent holds the load up: the force lies along -x.
EXPECTED_AXIS = {"tool_down": (0, -1.0)}


def check(reading, tolerance_fraction, tolerance_n, axis=None):
    """PASS when |F| is within tolerance of the weight carried and, given axis=(index, sign), the force points along
    that sensor axis (a wrong frame or sign keeps |F| but fails this); the reading and expected values are returned."""
    force = reading.get("force_n")
    load = reading.get("load_below_sensor") or {}
    if reading.get("status") != "measured" or not force or "weight_n" not in load:
        return {"pass": False, "reason": reading.get("reason") or "no measurement", "reading": reading}
    magnitude = math.sqrt(sum(value * value for value in force))
    expected = load["weight_n"]
    allowed = tolerance_fraction * expected + tolerance_n
    aligned = None
    if axis is not None and magnitude > 0:
        index, sign = axis
        aligned = sign * force[index] / magnitude
    passed = abs(magnitude - expected) <= allowed and (aligned is None or aligned >= 0.999)
    return {"pass": passed, "alignment": aligned, "force_magnitude_n": magnitude, "expected_n": expected,
            "allowed_n": allowed, "error_n": magnitude - expected, "force_n": force,
            "torque_nm": reading.get("torque_nm"), "mass_below_kg": load.get("mass_kg"),
            "physics_time_s": reading.get("physics_time_s")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--config", help="Absolute config path on the Kit host (default: the shipped config)")
    parser.add_argument("--mutate-stage", action="store_true", required=True)
    parser.add_argument("--discard-changes", action="store_true")
    parser.add_argument("--settle-steps", type=int, default=360, help="Extra exact steps after each move")
    parser.add_argument("--tolerance", type=float, default=0.01, help="Allowed error as a fraction of the weight")
    parser.add_argument("--tolerance-n", type=float, default=0.02, help="Allowed error added in newtons")
    args = parser.parse_args()
    if args.config and not Path(args.config).is_absolute():
        parser.error("--config must be absolute on the Kit host")
    with BridgeClient(host=args.host, port=args.port, timeout=300) as bridge:
        before = bridge.call("sim.stage.info", {})
        if not before.get("stopped"):
            bridge.call("sim.timeline", {"session_id": before["session_id"], "stage_id": before["stage_id"],
                                                 "action": "stop"})
        build = {"discard_changes": args.discard_changes, **({"config_path": args.config} if args.config else {})}
        summary = bridge.call("workcell.workcell_quick_start", build, timeout=300)
        backend = summary["backend"]
        root = summary["robot"]["articulation_root"]
        stage = bridge.call("sim.stage.info", {})
        ids = {"session_id": stage["session_id"], "stage_id": stage["stage_id"]}
        for action in ("play", "pause"):
            bridge.call("sim.timeline", {**ids, "action": action})
        results = {}
        for name, degrees in POSES.items():
            targets = {joint: math.radians(value) for joint, value in zip(ARM, degrees)}
            bridge.call("sim.joints.teleport", {**ids, "path": root, "positions": targets})
            bridge.call("sim.joints.command", {**ids, "path": root, "targets": targets})
            for _ in range(-(-args.settle_steps // 120)):
                bridge.call("sim.simulation.step", {**ids, "steps": 120}, timeout=300)
            result = check(bridge.call("workcell.robot_wrench", {}), args.tolerance, args.tolerance_n,
                           EXPECTED_AXIS.get(name))
            state = bridge.call("sim.joints.state", {**ids, "path": root})
            reached = dict(zip(state.get("names", ARM), state.get("positions", [])))
            result["max_joint_error_deg"] = max(abs(math.degrees(reached.get(joint, float("nan"))) - value)
                                                for joint, value in zip(ARM, degrees))
            results[name] = result
        bridge.call("sim.timeline", {**ids, "action": "stop"})
    passed = all(result["pass"] for result in results.values())
    print(json.dumps({"backend": backend, "poses": results, "pass": passed}, indent=2, default=str))
    print(f"{'PASS' if passed else 'FAIL'} ({backend}): static Bota force equals the weight it carries in "
          f"{sum(r['pass'] for r in results.values())}/{len(results)} poses.")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
