"""Move an arm's point along a straight line in one tool call: ``sim__arm__ik move=true speed=...``.

Solves and commands a point every frame of a 30 Hz path, waiting for simulation time to pass between them. Each point is an ordinary
``sim.arm.ik`` call, so its target check, its model check and its refusal to command a failed solve all still apply.
"""

from __future__ import annotations

import math
import time

from twinsmith.project_assets import AssetError, refuse

RATE_HZ = 30.0
MAX_STEPS = 3600
MAX_SECONDS = 120.0
MAX_WALL_S = 300.0  # other tools wait while a path runs, and a slow simulation stretches it
STUCK_WALL_S = 5.0
SETTLE_S = 0.25


async def _wait_sim_time(seconds, now, next_frame) -> bool:
    """Wait for *seconds* of simulation time. False if the simulation stopped advancing, or went back (a looping timeline)."""
    end, last, last_change = now() + seconds, now(), time.monotonic()
    while now() < end:
        await next_frame()
        current = now()
        if current < last:
            return False
        if current != last:
            last, last_change = current, time.monotonic()
        elif time.monotonic() - last_change > STUCK_WALL_S:
            return False
    return True


async def straight(args: dict, dispatch, is_playing, now, next_frame) -> dict:
    """*dispatch* runs a tool by name; *now* reads simulation time; *next_frame* awaits one frame."""
    speed = args.get("speed")
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
        refuse("INVALID_VALUE", "speed must be a positive number of stage units per second.")
    if args.get("move") is not True:
        refuse("INVALID_VALUE", "speed needs move=true: a path moves the arm.")
    if not is_playing():
        refuse("PHYSICS_NOT_READY", "A path needs a playing timeline: sim__timeline action=play.")
    goal = args["position"]
    if not isinstance(goal, (list, tuple)) or len(goal) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in goal):
        refuse("INVALID_VALUE", "position must be three finite numbers.")
    frame = args.get("frame", "world")
    ik_args = {k: v for k, v in args.items() if k != "speed"}
    fk_args = {k: args[k] for k in ("session_id", "stage_id", "path", "link", "base_link", "offset") if args.get(k) is not None}

    def tip() -> list:
        pose = dispatch("sim.arm.fk", dict(fk_args))
        return (pose["in_base_frame"] if frame == "base" else pose["end_effector_pose"])["position"]

    start = tip()
    distance = math.dist(start, goal)
    seconds = distance / speed
    if seconds > MAX_SECONDS:
        refuse("LIMIT", f"That path takes {seconds:.0f} s at this speed (the limit is {MAX_SECONDS:.0f}): go faster, or in parts.")
    steps = max(1, min(MAX_STEPS, math.ceil(seconds * RATE_HZ - 1e-9)))
    pause = seconds / steps
    result, done, stopped, index = None, 0, None, 1
    began, wall_began = now(), time.monotonic()
    while index <= steps:
        point = [start[i] + (goal[i] - start[i]) * index / steps for i in range(3)]
        try:
            result = dispatch("sim.arm.ik", dict(ik_args, position=point))
        except AssetError as exc:
            if result is None:
                raise  # nothing has moved: the refusal stands as it is
            stopped = f"{exc.code}: {str(exc)[:300]}"
            break
        if not result["reached"]:
            stopped = f"The point {[round(v, 4) for v in point]} could not be reached, so the arm stopped short of it."
            break
        done = index
        if index == steps:
            break
        if time.monotonic() - wall_began > MAX_WALL_S:
            stopped = f"The path was stopped after {MAX_WALL_S:.0f} s of real time: the simulation is running too slowly for it."
            break
        if not await _wait_sim_time(pause, now, next_frame):
            stopped = "The simulation stopped advancing (paused, stopped or looped)."
            break
        index = max(index + 1, min(steps, int((now() - began) / pause) + 1))  # a slow frame rate skips points rather than slowing the arm down
    if stopped is None:
        await _wait_sim_time(SETTLE_S, now, next_frame)  # the joints follow their targets a moment later
    try:
        final = tip()
    except AssetError:
        final = None
    result["path"] = dict(
        start=start, goal=list(goal), final=final, final_error=math.dist(final, goal) if final else None, steps=steps, steps_done=done, speed=speed,
        **({"stopped_early": stopped} if stopped else {}),
        note="final is where the point really is; the arm can lag its command, and contact can hold it back. There is no collision check.",
    )
    return result
