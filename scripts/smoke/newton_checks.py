"""Live Newton checks, for a disposable simulator whose physics engine is Newton (Isaac Sim 6.2, MuJoCo-Warp).

Needs a free GPU and a running bridge on Newton; it replaces the open stage twice (a plain scene, then the
Jenga scene). Run it through ``python3 scripts/smoke.py sim --backend newton --discard-changes --check newton_checks``
or directly: ``python3 scripts/smoke/newton_checks.py --port 8766 --output results.json --discard-changes``.

Every check is reported on its own, with what it measured. A check whose set-up failed is "not_run" and says why;
nothing passes on missing evidence. All calls go through the real stdio MCP surface (scripts/scene_mcp_client.py).

  quick_scene            the quick scene builds on Newton and reports its timestep and contact buffers
  exact_steps_default    20 exact steps on the untouched quick scene advance the clock by 20 * dt (the Sim 6.2 mismatch)
  exact_steps_configured the same after sim__physics configure with dt=1/120
  camera_after_motion    a capture after the probe fell shows it where physics put it (Fabric reaches the renderer)
  push_moves_body        a one-off impulse changes the body's velocity by impulse / mass, and it moves
  tower_build            the 54-block tower builds with its Newton settings and contact buffers
  tower_hold             after 10 simulated seconds no block moved more than the tolerance
  contact_buffer         MuJoCo-Warp's contact and constraint buffers did not overflow (counters read, not assumed)
  contacts_pairs         sim__physics contacts returns pairs and forces for a block, with the table among them
"""

import argparse
import asyncio
import json
import math
import sys
import traceback
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

LAYERS, PER_LAYER = 18, 3
BLOCK_MASS_KG = 0.12
TOWER_ROOT = "/World/Jenga/Tower"
EXPECTED_CONTACT_BUFFER = 54 * 32  # what the quick-scene tower must configure at least (jenga.py / newton_support.py)
CHUNK = 120  # sim__simulation__step takes 1..120 steps
CAMERA = dict(width=640, height=480, focal_length_mm=50.0, horizontal_aperture_mm=20.955)  # the defaults of sim__camera__create
SETTINGS = dict(hold_seconds=10.0, tolerance_m=0.015, tolerance_deg=5.0, push_speed=0.5, clock_rel_tol=1e-5)


class ToolError(Exception):
    """A tool answered with an error; ``payload`` holds its code and details when they could be parsed."""

    def __init__(self, tool, payload):
        super().__init__(f"{tool}: {str(payload)[:400]}")
        self.tool, self.payload = tool, payload


class CheckFailed(AssertionError):
    pass


def fail(message, **evidence):
    raise CheckFailed(message + (" " + json.dumps(evidence, default=str)[:700] if evidence else ""))


def need(condition, message, **evidence):
    if not condition:
        fail(message, **evidence)


def block_path(layer, index):
    return f"{TOWER_ROOT}/L{layer:02d}_B{index}"


BLOCKS = [block_path(layer, index) for layer in range(LAYERS) for index in range(PER_LAYER)]
CHECK_ORDER = ("quick_scene", "exact_steps_default", "exact_steps_configured", "camera_after_motion", "push_moves_body",
               "tower_build", "tower_hold", "contact_buffer", "contacts_pairs")


# --- pure helpers (unit-tested offline) ---------------------------------------------------------------------


def quaternion_angle_deg(a, b):
    """Angle between two [w, x, y, z] rotations, in degrees."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, dot)))


def block_motion(before, after):
    """``(max displacement m, block, max rotation deg, block)`` between two {path: (position, wxyz)} maps."""
    worst_move, worst_turn = (-1.0, None), (-1.0, None)  # the first block always counts, even when nothing moved
    for path, (position, quaternion) in before.items():
        moved = math.dist(position, after[path][0])
        turned = quaternion_angle_deg(quaternion, after[path][1])
        if moved > worst_move[0]:
            worst_move = (moved, path)
        if turned > worst_turn[0]:
            worst_turn = (turned, path)
    return worst_move[0], worst_move[1], worst_turn[0], worst_turn[1]


def project(point, eye, target, camera=CAMERA):
    """Pixel (u, v) of a world point seen by a camera at *eye* aimed at *target* (world up +Z, USD camera: -Z forward)."""
    def sub(a, b):
        return [x - y for x, y in zip(a, b)]

    def unit(v):
        length = math.sqrt(sum(x * x for x in v))
        return [x / length for x in v]

    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]

    forward = unit(sub(target, eye))
    right = unit(cross(forward, [0.0, 0.0, 1.0]))
    up = cross(right, forward)
    relative = sub(point, eye)
    depth = sum(a * b for a, b in zip(relative, forward))
    scale = camera["focal_length_mm"] / camera["horizontal_aperture_mm"] * camera["width"]
    return (camera["width"] / 2 + scale * sum(a * b for a, b in zip(relative, right)) / depth,
            camera["height"] / 2 - scale * sum(a * b for a, b in zip(relative, up)) / depth)


def red_row(rgb):
    """Mean pixel row of the strongly red pixels of an HxWx3 array, or None when there are none."""
    import numpy as np

    rgb = np.asarray(rgb).astype(float)
    mask = (rgb[:, :, 0] > 1.5 * rgb[:, :, 1]) & (rgb[:, :, 0] > 1.5 * rgb[:, :, 2]) & (rgb[:, :, 0] > 50)
    return float(np.nonzero(mask)[0].mean()) if mask.any() else None


def load_rgb(path):
    import numpy as np
    from PIL import Image

    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"))


def render_table(results):
    width = max([len(r["check"]) for r in results] + [5])
    lines = [f"{'Check':{width}}  Result   Measured", f"{'-' * width}  -------  --------"]
    for row in results:
        lines.append(f"{row['check']:{width}}  {row['status'].upper():7}  {row['summary']}")
        if row["status"] != "pass" and row.get("reason"):
            lines.append(f"{'':{width}}           {row['reason']}")
    return "\n".join(lines)


# --- the checks ---------------------------------------------------------------------------------------------------


class Run:
    """Holds the tool caller, the settings and one result row per check."""

    def __init__(self, invoke, settings, rgb_loader=None):
        self.invoke, self.settings, self.rgb_loader = invoke, settings, rgb_loader or load_rgb
        self.results = []
        self.ids = {}
        self.unavailable = None

    def tool(self, name, **args):
        return self.invoke(name, **self.ids, **args) if name != "sim__stage__info" else self.invoke(name, **args)

    async def check(self, name, work):
        """Run one check on its own; not_run when the set-up it depends on failed."""
        if self.unavailable:
            self.results.append(dict(check=name, status="not_run", summary="-", reason=self.unavailable))
            return None
        try:
            summary, measured = await work()
        except CheckFailed as error:
            self.results.append(dict(check=name, status="fail", summary="-", reason=str(error)[:900]))
        except ToolError as error:
            self.results.append(dict(check=name, status="fail", summary="-", reason=str(error)[:900],
                                     error=error.payload if isinstance(error.payload, dict) else None))
        except Exception as error:  # noqa: BLE001 - one broken check must not hide the others
            self.results.append(dict(check=name, status="fail", summary="-",
                                     reason=f"{type(error).__name__}: {str(error)[:300]}",
                                     trace=traceback.format_exc(limit=4)))
        else:
            self.results.append(dict(check=name, status="pass", summary=summary, measured=measured))
            return measured
        return None

    def not_run(self, name, reason):
        self.results.append(dict(check=name, status="not_run", summary="-", reason=reason))

    async def new_stage(self, **args):
        info = await self.invoke("sim__stage__info")
        if not info.get("stopped"):
            await self.invoke("sim__timeline", session_id=info["session_id"], stage_id=info["stage_id"], action="stop")
        started = await self.invoke("sim__quick_start", discard_changes=True, **args)
        self.ids = {key: started[key] for key in ("session_id", "stage_id")}
        return started

    async def poses(self, paths):
        reply = await self.tool("sim__body__state", paths=paths)
        return {body["path"]: (body["world_position"], body["orientation_wxyz"]) for body in reply["bodies"]}

    async def step(self, steps, **args):
        return await self.tool("sim__simulation__step", steps=steps, **args)

    async def initialize(self):
        await self.tool("sim__timeline", action="initialize")


async def exact_steps(run, label):
    """20 exact steps; on a clock mismatch the tool's own error carries Newton's clocks, so report them."""
    try:
        reply = await run.step(20)
    except ToolError as error:
        payload = error.payload if isinstance(error.payload, dict) else {}
        details = payload.get("details") or payload
        fail(f"{label}: {payload.get('code') or 'step refused'}; expected the clock to advance by 20*dt", details=details)
    dt, delta = reply["physics_dt"], reply["measured_delta"]
    need(math.isclose(delta, 20 * dt, rel_tol=run.settings["clock_rel_tol"], abs_tol=1e-7),
         "the physics clock did not advance by 20 * dt", dt=dt, measured_delta=delta, expected=20 * dt)
    newton = reply.get("newton", {})
    need(newton.get("timestep_check", {}).get("ran") is True, "the timestep cross-check did not run", newton=newton)
    clocks = newton.get("clocks_after", {})
    return (f"dt={dt:.6g} s, clock +{delta:.6g} s = 20*dt; Newton clock {clocks.get('newton_sim_time')}",
            dict(dt=dt, measured_delta=delta, expected_delta=20 * dt, newton=newton))


async def scene_a(run):
    """The plain quick scene with a free red probe: stepping, the picture and the push."""
    started = await run.new_stage()
    report = started.get("newton")

    async def quick_scene():
        need(report is not None, "quick_start reported no Newton setup: this is not a Newton session, or the build lacks it")
        need("error" not in report, "the quick scene's Newton setup failed", newton=report)
        need(report["buffers"]["nconmax"] >= 512 and report["rate_hz"] > 0, "buffers or rate missing", newton=report)
        return (f"rate {report['rate_hz']} Hz, nconmax {report['buffers']['nconmax']}, njmax {report['buffers']['njmax']}", report)

    ok = await run.check("quick_scene", quick_scene)
    if ok is None:
        run.unavailable = "the quick scene did not build on Newton, so nothing after it can be measured"
        return
    prefix = "/World/NewtonCheck"
    floor, probe, camera = prefix + "/Floor", prefix + "/Probe", prefix + "/Camera"
    eye, target = [80.0, -12.0, 5.0], [80.0, 0.0, 1.3]
    scene_path = started["physics_scene"]
    await run.tool("sim__prim__create", path=prefix, kind="Xform")
    await run.tool("sim__prim__create", path=floor, kind="Cube", position=[80, 0, -0.05], scale=[8, 8, 0.1], collision=True)
    await run.tool("sim__prim__create", path=probe, kind="Cube", position=[80, 0, 2.5], size=0.25, rigid=True, mass=0.5)
    await run.tool("sim__light__create", path=prefix + "/Light", kind="DomeLight", intensity=1000)
    await run.tool("sim__material__create", path=prefix + "/Red", color=[0.9, 0.01, 0.01])
    await run.tool("sim__material__apply", path=probe, material_path=prefix + "/Red")
    await run.tool("sim__camera__create", path=camera, width=CAMERA["width"], height=CAMERA["height"], position=eye,
                   target=target, frame="world")

    async def default_steps():
        await run.initialize()
        return await exact_steps(run, "untouched quick scene")

    await run.check("exact_steps_default", default_steps)

    async def configured_steps():
        await run.tool("sim__timeline", action="stop")
        configured = await run.tool("sim__physics__configure", path=scene_path, gravity=[0, 0, -9.81], dt=1 / 120)
        need(abs(configured["dt"] - 1 / 120) < 1e-9, "configure did not take dt=1/120", configured=configured)
        await run.initialize()
        return await exact_steps(run, "after configure dt=1/120")

    await run.check("exact_steps_configured", configured_steps)

    async def camera_after_motion():
        stamp = uuid.uuid4().hex[:8]
        try:
            import numpy  # noqa: F401
            from PIL import Image  # noqa: F401
        except ImportError as error:
            fail(f"cannot read pictures here: {error}; install numpy and Pillow for this check")
        before_pose = (await run.poses([probe]))[probe][0]

        async def capture(phase):
            shot = await run.tool("sim__camera__capture", path=camera, filename=f"newton-{stamp}-{phase}.png")
            row = red_row(run.rgb_loader(shot["file"]))
            need(row is not None, f"no red pixels in the {phase} picture", file=shot["file"])
            return row, shot.get("physics_render")

        first, render_before = await capture("before")
        for steps in (10, 30, 80, 120):
            await run.step(steps)
        last = await run.tool("sim__body__state", path=probe)
        second, render_after = await capture("after")
        after_pose = last["world_position"]
        height = CAMERA["height"]
        expected = project(after_pose, eye, target)[1] - project(before_pose, eye, target)[1]
        measured = second - first
        need(measured > height * 0.15, "the probe fell but the picture barely changed: the renderer does not show the moved pose",
             before_row=first, after_row=second, physics_render=render_after)
        need(abs(measured - expected) <= 0.35 * abs(expected) + 6,
             "the red pixels moved, but not to where physics put the probe", measured_px=measured, expected_px=expected,
             before=before_pose, after=after_pose)
        need(abs(after_pose[2] - 0.125) < 0.03, "the probe did not come to rest on the floor", after=after_pose)
        agrees = (last.get("render_pose") or {}).get("agrees")
        need(agrees is not False, "render_pose says Fabric and USD both disagree with physics", render_pose=last.get("render_pose"))
        return (f"probe fell {before_pose[2]:.2f} -> {after_pose[2]:.3f} m; red rows {first:.0f} -> {second:.0f} px "
                f"(physics predicts {expected:.0f}); render_pose.agrees={agrees}",
                dict(before_row=first, after_row=second, expected_px=expected, render_pose=last.get("render_pose"),
                     physics_render=render_after or render_before))

    await run.check("camera_after_motion", camera_after_motion)

    async def push_moves_body():
        before = await run.tool("sim__body__state", path=probe, include_dynamics=True)
        mass = before["dynamics"]["mass_kg"]
        speed = run.settings["push_speed"]
        pushed = await run.tool("sim__physics__apply", path=probe, impulse=[mass * speed, 0.0, 0.0])
        need(pushed.get("backend") == "newton" and pushed.get("applied") is True, "the push was not applied by Newton", pushed=pushed)
        await run.step(1)
        moving = await run.tool("sim__body__state", path=probe, include_dynamics=True)
        dv = moving["dynamics"]["linear_velocity"][0] - before["dynamics"]["linear_velocity"][0]
        need(0.7 * speed <= dv <= 1.1 * speed, "one step after the push the velocity change is not impulse / mass",
             delta_v=dv, expected=speed, mass_kg=mass)
        await run.step(19)
        after = await run.tool("sim__body__state", path=probe)
        moved = after["world_position"][0] - before["world_position"][0]
        need(moved > 0.005, "the pushed body did not move", moved_m=moved)
        return (f"impulse {mass * speed:.3f} N s on {mass:.2f} kg: dv {dv:.3f} m/s (expected {speed}), moved {moved * 1000:.1f} mm in 20 steps",
                dict(delta_v=dv, expected_delta_v=speed, moved_m=moved, force_n=pushed.get("applied_force_n"), duration_s=pushed.get("force_duration_s")))

    await run.check("push_moves_body", push_moves_body)


async def scene_b(run):
    """The full 54-block tower: build, hold, buffers, contacts."""
    run.unavailable = None
    started = await run.new_stage(environment="jenga")
    environment = started.get("environment", {})

    async def tower_build():
        need(environment.get("blocks") == 54, "the tower is not 54 blocks", environment=environment)
        need("newton_error" not in environment, "a Newton setting of the tower could not be authored", error=environment.get("newton_error"))
        buffers = environment.get("newton_buffers") or {}
        need(buffers.get("nconmax", 0) >= EXPECTED_CONTACT_BUFFER and buffers.get("njmax", 0) >= 8 * EXPECTED_CONTACT_BUFFER,
             "the tower did not size the contact buffers for 54 blocks", buffers=buffers)
        need(environment.get("newton_settings") is not None, "no Newton scene settings were authored", environment=environment)
        extra = started.get("newton") or {}
        need("error" not in extra, "the final buffer sizing failed", newton=extra)
        return (f"54 blocks; nconmax {extra.get('buffers', {}).get('nconmax', buffers['nconmax'])}, "
                f"njmax {extra.get('buffers', {}).get('njmax', buffers['njmax'])}, rate {environment.get('physics_hz')} Hz",
                dict(environment={k: environment.get(k) for k in ("blocks", "newton_settings", "newton_buffers")}, final=extra))

    if await run.check("tower_build", tower_build) is None:
        run.unavailable = "the tower did not build with its Newton settings"
        return
    await run.initialize()
    state = {}

    async def tower_hold():
        before = await run.poses(BLOCKS)
        need(len(before) == 54, "not all 54 blocks have runtime poses", count=len(before))
        done, first = 0, await run.step(CHUNK, observe_prims=[environment["top_block"]])
        dt = first["physics_dt"]
        need(dt > 0, "no usable physics timestep", dt=dt)
        done = CHUNK
        total = math.ceil(run.settings["hold_seconds"] / dt)
        midpoint = None
        while done < total:
            count = min(CHUNK, total - done)
            await run.step(count)
            done += count
            if midpoint is None and done * dt >= run.settings["hold_seconds"] / 2:
                midpoint = (await run.tool("sim__physics__inspect", root="/World/Jenga", limit=1)).get("newton")
        after = await run.poses(BLOCKS)
        move, move_block, turn, turn_block = block_motion(before, after)
        state.update(steps=done, dt=dt, midpoint=midpoint)
        need(move <= run.settings["tolerance_m"], "a block moved more than the tolerance", max_move_m=move, block=move_block,
             tolerance_m=run.settings["tolerance_m"])
        need(turn <= run.settings["tolerance_deg"], "a block turned more than the tolerance", max_turn_deg=turn, block=turn_block)
        return (f"{done} steps = {done * dt:.2f} s; max move {move * 1000:.1f} mm ({move_block.rsplit('/', 1)[-1]}), "
                f"max turn {turn:.2f} deg", dict(steps=done, simulated_s=done * dt, max_move_m=move, max_turn_deg=turn))

    await run.check("tower_hold", tower_hold)

    async def contact_buffer():
        newton = (await run.tool("sim__physics__inspect", root="/World/Jenga", limit=1)).get("newton")
        need(newton is not None, "physics inspect returned no Newton section: not a Newton session or the build lacks it")
        configured = newton.get("configured") or {}
        need((configured.get("nconmax") or 0) >= EXPECTED_CONTACT_BUFFER, "the configured contact buffer is below what 54 blocks need",
             configured=configured)
        need(newton.get("active_contacts") is not None and newton.get("contact_capacity"),
             "the solver's contact counters could not be read, so overflow cannot be ruled out", newton=newton)
        need(not newton.get("warnings"), "physics inspect warns about the buffers", warnings=newton.get("warnings"),
             flags=newton.get("overflow_flags"))
        need(newton.get("overflow_history_available"), "the solver's overflow flags are unavailable", newton=newton)
        for earlier in (state.get("midpoint"),):
            need(not earlier or not earlier.get("warnings"), "the midpoint inspect already warned", midpoint=earlier)
        active, capacity = max(newton["active_contacts"]), newton["contact_capacity"]
        return (f"{active} active contacts of {capacity} (configured nconmax {configured.get('nconmax')}); overflow flags clear",
                dict(active_contacts=active, contact_capacity=capacity, configured=configured, flags=newton.get("overflow_flags")))

    await run.check("contact_buffer", contact_buffer)

    async def contacts_pairs():
        block = block_path(0, 1)
        reply = await run.tool("sim__physics__contacts", paths=[block])
        need(reply.get("backend") == "newton", "contacts were not produced by Newton", backend=reply.get("backend"))
        pairs = reply.get("pairs") or []
        need(pairs, "no contact pair for a block at the bottom of a standing tower", reply=reply)
        for pair in pairs:
            need({"body_a", "body_b", "collider_a", "collider_b", "contact_points", "force"} <= set(pair), "a pair lacks fields", pair=pair)
            need(all(math.isfinite(v) for v in pair["force"]), "a pair has a nonfinite force", pair=pair)
        table = [p for p in pairs if "Table" in str(p["collider_a"]) + str(p["collider_b"])]
        need(table and max(p["force_magnitude"] for p in table) > 0.0, "the bottom block shows no force on the table", pairs=pairs)
        support = max(p["force_magnitude"] for p in table)
        weight = BLOCK_MASS_KG * 9.81
        return (f"{len(pairs)} pairs for {block.rsplit('/', 1)[-1]}; table support {support:.2f} N ({support / weight:.1f} x its own weight)",
                dict(pairs=len(pairs), table_support_n=support, weight_n=weight, truncated=reply.get("truncated")))

    await run.check("contacts_pairs", contacts_pairs)


async def run_checks(invoke, settings=None, rgb_loader=None):
    """Every check, independently. ``invoke(name, **arguments)`` is an async tool caller that raises ToolError."""
    run = Run(invoke, dict(SETTINGS, **(settings or {})), rgb_loader)
    for name, scene in (("a", scene_a), ("b", scene_b)):
        try:
            await scene(run)
        except Exception as error:  # noqa: BLE001 - a failed set-up fails its scene, not the other one
            reason = (str(error) if isinstance(error, ToolError) else f"{type(error).__name__}: {error}")[:500]
            run.results.append(dict(check=f"scene_{name}_setup", status="fail", summary="-", reason=reason))
            run.unavailable = f"scene {name} could not be set up: {reason[:200]}"
    for name in CHECK_ORDER:  # a check that never started is reported, not forgotten
        if not any(row["check"] == name for row in run.results):
            run.not_run(name, run.unavailable or "an earlier set-up failed")
    run.results.sort(key=lambda row: (CHECK_ORDER.index(row["check"]) if row["check"] in CHECK_ORDER else -1))
    return run.results


# --- the real MCP surface -------------------------------------------------------------------------------------------


def payload(result):
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


async def validate(port, output, hold_seconds=None, tolerance_m=None, **_):
    from mcp import ClientSession
    from mcp.client.stdio import stdio_client
    from scene_mcp_client import call, parameters

    settings = {key: value for key, value in dict(hold_seconds=hold_seconds, tolerance_m=tolerance_m).items() if value is not None}
    records = []
    async with stdio_client(parameters(port)) as (reader, writer):
        async with ClientSession(reader, writer) as client:
            await client.initialize()

            async def invoke(name, **arguments):
                item = await call(client, name, arguments)
                records.append(dict(tool=name, arguments=arguments, isError=item["isError"],
                                    result=item["result"] if item["isError"] or name != "sim__camera__capture" else "(capture)"))
                if item["isError"]:
                    raise ToolError(name, payload(item["result"]) or item["result"])
                return item["result"]

            results = await run_checks(invoke, settings)
    failed = [row for row in results if row["status"] != "pass"]
    table = render_table(results)
    print(table)
    Path(output).write_text(json.dumps(dict(passed=not failed, checks=results, settings=dict(SETTINGS, **settings),
                                            calls=records[-400:]), indent=2, default=str))
    if failed:
        raise RuntimeError("Newton checks not passed: " + ", ".join(f"{row['check']} ({row['status']})" for row in failed))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", required=True, help="JSON result file")
    parser.add_argument("--discard-changes", action="store_true", help="required: both scenes replace the open stage")
    parser.add_argument("--hold-seconds", type=float, default=None)
    parser.add_argument("--tolerance-m", type=float, default=None)
    args = parser.parse_args()
    if not args.discard_changes:
        parser.error("--discard-changes is required: the checks replace the open stage")
    try:
        asyncio.run(validate(args.port, args.output, args.hold_seconds, args.tolerance_m))
    except RuntimeError as error:
        print(error, file=sys.stderr)
        sys.exit(1)
