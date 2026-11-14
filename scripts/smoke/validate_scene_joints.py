"""Live validation: joint command by name, velocity targets, clamping, gains, settle and teleport.

One robot per run (``--robot ur5e`` or ``--robot franka`` from sim__quick_start). It plays to initialize
physics, then drives the robot while paused, so every assertion is on real PhysX/Newton tensor data.
A robot lying on the ground is blocked in one direction: that contact case must report ``settled=false``,
and the free direction must settle within tolerance.
"""

import argparse
import asyncio
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mcp import ClientSession
from mcp.client.stdio import stdio_client
from scene_mcp_client import call, parameters


def candidates(names):
    """Movable DOFs in state order, preferring those that are not fingers or grippers."""
    preferred = [name for name in names
                 if not any(word in name.lower() for word in ("finger", "gripper"))]
    return (preferred + [name for name in names if name not in preferred])[:8]


def inside(value, low, high, margin=0.02):
    return max(low + margin, min(high - margin, value))


def payload(result):
    """An MCP error arrives as text; its JSON envelope (with the code) follows the message."""
    if isinstance(result, dict):
        return result
    if isinstance(result, str):
        start = result.find("{")
        if start >= 0:
            try:
                return json.loads(result[start:])
            except ValueError:
                pass
    return {}


SETTLE_BUDGET_S = 400 / 60  # the simulated time PhysX had for the old 400 steps


async def validate(port, output, robot):
    records = []

    async with stdio_client(parameters(port)) as (r, w):
        async with ClientSession(r, w) as client:
            await client.initialize()

            async def invoke(name, error=False, **args):
                item = await call(client, name, args)
                records.append(item)
                Path(output).write_text(json.dumps(records, indent=2))
                assert item["isError"] == error, item
                return payload(item["result"])

            async def settle(**kw):
                """sim__joints__move until settled, within the same simulated time on every engine.

                The tool takes at most 600 steps per call, and a step is 1/60 s on PhysX but 2 ms on Newton, so the
                old 400 steps were 6.7 s on PhysX and 0.8 s on Newton, less than the drive's settling time (the Franka's
                drive has a 0.2 s time constant: about 0.9 to 1.2 s to come within 0.01 rad). The budget is the PhysX
                one; the result adds the whole run's steps and simulated seconds."""
                steps, seconds = 0, 0.0
                request = 400
                while True:
                    got = await invoke("sim__joints__move", **kw, max_steps=request, tolerance=0.01)
                    steps += got["steps_used"]
                    seconds += got["simulated_seconds"]
                    if got["settled"] or seconds >= SETTLE_BUDGET_S - 1e-9:
                        return {**got, "steps_used": steps, "simulated_seconds": seconds}
                    dt = got["simulated_seconds"] / got["steps_used"]
                    request = max(1, min(600, math.ceil((SETTLE_BUDGET_S - seconds) / dt - 1e-9)))

            info = await invoke("sim__stage__info")
            target = {k: info[k] for k in ("session_id", "stage_id")}
            if not info["stopped"]:
                await invoke("sim__timeline", **target, action="stop")
            started = await invoke("sim__quick_start", robot=robot, discard_changes=True)
            target = {k: started[k] for k in ("session_id", "stage_id")}
            root = started["robot"]["articulation_root"]
            assert root, started

            await invoke("sim__timeline", **target, action="play")
            for _ in range(12):
                await asyncio.sleep(0.5)
                state = await invoke("sim__joints__state", **target, path=root)
                if state["positions"]:
                    break
            await invoke("sim__timeline", **target, action="pause")
            state = await invoke("sim__joints__state", **target, path=root)
            names = state["names"]
            assert len(names) >= 3, state

            # The tool reports a refused target's runtime limits; one refusal per candidate reads them.
            limits = {}
            for name in candidates(names):
                index = names.index(name)
                value = state["positions"][index]
                for probe in (value + 1e6, value - 1e6):
                    item = await call(client, "sim__joints__command", dict(
                        **target, path=root, targets={name: probe}))
                    if item["isError"] and payload(item["result"]).get("code") == "JOINT_LIMIT":
                        low, high = payload(item["result"])["limits"]
                        limits[name] = (float(low), float(high))
                        break
                else:
                    limits[name] = (-1e9, 1e9)
            joints = [name for name in candidates(names)
                      if limits[name][1] - limits[name][0] > 0.5][:3]
            assert len(joints) == 3, (joints, limits)
            home = {name: state["positions"][names.index(name)] for name in joints}
            report = dict(robot=robot, articulation_root=root, dof_count=len(names), joints=joints,
                          units=state.get("units", ""), limits=limits, steps={})

            # Command by name and settle with move: 0.5 rad away, then back.
            up = {name: inside(home[name] + 0.5, *limits[name]) for name in joints}
            commanded = await invoke("sim__joints__command", **target, path=root, targets=up)
            assert set(commanded["names"]) == set(names)
            assert {item["name"]: item["applied"] for item in commanded["targets"]} == up
            assert all(item["previous"] is not None for item in commanded["targets"])
            assert commanded["backend"] in ("physx", "newton")
            away = await settle(**target, path=root, targets=up)
            direction = up
            if away["settled"]:
                assert all(abs(item["error"]) <= 0.01 for item in away["joints"]), away
                report["steps"]["move_out"] = dict(steps=away["steps_used"], settled=True,
                                                   simulated_seconds=away["simulated_seconds"],
                                                   step_s=away["simulated_seconds"] / away["steps_used"])
            else:
                # A real contact: the robot rests on the ground and this direction presses into it.
                assert away.get("blocked") and away["blocked"]["name"] in joints, away
                assert abs(away["blocked"]["error"]) > 0.01
                report["steps"]["contact_blocked"] = dict(
                    steps=away["steps_used"], settled=away["settled"], blocked=away["blocked"],
                    note="blocked in this direction, honestly reported; the opposite direction follows",
                )
                direction = {name: inside(home[name] - 0.5, *limits[name]) for name in joints}
                away = await settle(**target, path=root, targets=direction)
                assert away["settled"] is True, away
                assert all(abs(item["error"]) <= 0.01 for item in away["joints"]), away
                report["steps"]["move_out"] = dict(direction="opposite", steps=away["steps_used"],
                                                   settled=True,
                                                   simulated_seconds=away["simulated_seconds"])
            assert away["steps_used"] >= 1 and away["simulated_seconds"] > 0
            moved_to = {item["name"]: item["position"] for item in away["joints"]}
            assert moved_to and all(name in joints for name in moved_to), away

            returned = await settle(**target, path=root, targets=home)
            assert returned["settled"] is True, returned
            assert all(abs(item["error"]) <= 0.01 for item in returned["joints"]), returned
            report["steps"]["move_back"] = dict(steps=returned["steps_used"], settled=True,
                                                simulated_seconds=returned["simulated_seconds"],
                                                step_s=returned["simulated_seconds"] / returned["steps_used"])

            # Velocity targets are acknowledged per name and reported with the previous ones.
            velocity = await invoke("sim__joints__command", **target, path=root,
                                    velocities={joints[0]: 0.0})
            assert velocity["velocities"][0]["name"] == joints[0]
            assert velocity["velocities"][0]["previous"] is not None

            # Unknown DOF names list the valid ones and change nothing.
            unknown = await invoke("sim__joints__command", error=True, **target, path=root,
                                   targets={"not_a_joint": 0.0})
            assert unknown["code"] == "UNKNOWN_DOF" and unknown["valid_names"] == names

            # Out of limits: refused by default, clamped and reported with clamp=true.
            first = joints[0]
            low, high = limits[first]
            beyond = high + 0.5 if math.isfinite(high) and high < 1e6 else low - 0.5
            refused = await invoke("sim__joints__command", error=True, **target, path=root,
                                   targets={first: beyond})
            assert refused["code"] == "JOINT_LIMIT", refused
            clamped = await invoke("sim__joints__command", **target, path=root,
                                   targets={first: beyond}, clamp=True)
            assert clamped["clamped"] and clamped["clamped"][0]["name"] == first
            assert clamped["clamped"][0]["applied"] in (low, high)
            await settle(**target, path=root, targets=home)

            # Runtime gains: read, change, read back, restore.
            gains = await invoke("sim__joints__gains", **target, path=root)
            assert gains["read_only"] is True and gains["dofs"]
            before = next(item for item in gains["dofs"] if item["name"] == first)["before"]
            changed = await invoke("sim__joints__gains", **target, path=root, dofs=[first],
                                   stiffness=440.0, damping=44.0, max_effort=120.0)
            after = changed["dofs"][0]["after"]
            assert changed["applied"] is True
            assert abs(after["stiffness"] - 440.0) < 1.0 and abs(after["damping"] - 44.0) < 1.0
            assert abs(after["max_effort"] - 120.0) < 1.0, after
            restored = await invoke("sim__joints__gains", **target, path=root, dofs=[first],
                                    stiffness=before["stiffness"], damping=before["damping"],
                                    max_effort=max(before["max_effort"], 1.0))
            assert abs(restored["dofs"][0]["after"]["stiffness"] - before["stiffness"]) < 1.0
            report["steps"]["gains"] = dict(before=before, changed=after,
                                            restored=restored["dofs"][0]["after"])

            # A joint that cannot reach its target must not be reported as settled.
            targets = {name: inside(home[name] + 0.5, *limits[name]) for name in joints}
            await invoke("sim__joints__gains", **target, path=root, dofs=joints,
                         stiffness=0.0, damping=0.0, max_effort=1e-6)
            await invoke("sim__joints__command", **target, path=root, targets=targets)
            blocked = await invoke("sim__joints__move", **target, path=root, targets=targets,
                                   max_steps=60, tolerance=0.01)
            assert blocked["settled"] is False, blocked
            assert blocked.get("blocked"), blocked
            assert any(abs(item["error"]) > 0.01 for item in blocked["joints"])
            report["steps"]["weak_drive_not_settled"] = dict(
                settled=blocked["settled"], steps=blocked["steps_used"], blocked=blocked["blocked"],
                reason="stiffness 0 and max_effort 1e-6: the same not-settled result a contact produces",
            )
            await invoke("sim__joints__gains", **target, path=root, dofs=joints,
                         stiffness=1000.0, damping=100.0, max_effort=1000.0)
            await settle(**target, path=root, targets=home)

            # Teleport is a state reset, reported before and after, with drive targets left alone.
            teleported = await invoke("sim__joints__teleport", **target, path=root,
                                      positions={joints[1]: home[joints[1]] - 0.05})
            entry = teleported["joints"][0]
            assert abs(entry["after"] - (home[joints[1]] - 0.05)) < 0.02, entry
            assert teleported["physics_bypassed"] is True
            assert teleported["velocities_zeroed"] is True
            again = await invoke("sim__joints__state", **target, path=root)
            assert abs(again["positions"][names.index(joints[1])]
                       - (home[joints[1]] - 0.05)) < 0.02, again
            report["steps"]["teleport"] = entry

            await invoke("sim__timeline", **target, action="stop")
            info = await invoke("sim__stage__info")
            assert info["stopped"] is True
            print(json.dumps(dict(passed=True, calls=len(records), **report)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--robot", choices=("ur5e", "franka"), required=True)
    args = parser.parse_args()
    asyncio.run(validate(args.port, args.output, args.robot))
