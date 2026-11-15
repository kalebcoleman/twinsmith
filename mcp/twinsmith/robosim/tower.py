"""The multi-block scene: a Jenga tower of free rigid blocks, a scripted hand and a pushing finger.

``TEMPLATE`` is trusted Python that the pinned RSS scene loader executes, like ``template.TEMPLATE``; every
parameter is plain data near its top and ``resolve`` checks the data a caller gives. The tower is 18 layers of
3 blocks of 72 x 24 x 14 mm by default, each layer across the one below (the work cell's ``jenga_tower`` task has
the same layout and block). ``blocks`` replaces the regular tower with fitted poses, as a clip needs.

Where the physical defaults come from: the first Jenga real-to-sim run (2026-11-14, one slow-motion clip of a
retail set toppling, 197 rollouts on Newton's MuJoCo-Warp). That clip bounded block-on-block friction from below
only (0.2 is worse, 0.3 to 0.55 cannot be told apart), preferred block-on-table friction above 0.4, placed the
contact damping ratio at 0.7 to 0.85 and did not identify the mass. The values below are that run's best point:
consistent with one clip, not measured. The per-block spread of size, mass and friction mattered as much as the
friction range did, so it is on by default.

MuJoCo has no restitution coefficient: the contact's damping ratio stands in for it (0.74 bounces back about
0.13 to 0.19 of the speed in a drop test; below 0.5 a contact gains energy and a tower blows apart). MuJoCo also
takes the larger of two surfaces' friction, so the table is given contact priority and a block on the table uses
the table's value.

How the hand and the finger move a block: as ``template.py`` explains, a free body is moved by writing its own
coordinates (``joint_q``, ``joint_qd``); ``body_q`` is an output. A held block follows its path until the release
time and is an ordinary body afterwards. The finger is a slender box written along the pushed block's long axis.
"""

import re

from .template import _finite, _vector, colour_rgb, mask_key

TEMPLATE = '''import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import robosimstudio as rss
from robosimstudio.specs.physics import MUJOCO_500HZ, backend

# ---- parameters: plain data, edit these -------------------------------------------------------
# Tower frame: metres, +z up, origin on the table under the tower's axis. Yaw in radians about +z.
BLOCK_SIZE = (0.072, 0.024, 0.014)    # one block: length, width, height
LAYERS = 18                           # layers of three; an even layer's blocks lie along x, an odd layer's along y
TOWER_XY_YAW = (0.0, 0.0, 0.0)        # where the tower stands on the table, and its turn
MISSING = []                          # blocks left out of the regular tower, by name "L<layer>_<slot>"
BLOCKS = None                         # or [[name, layer, x, y, yaw, section], ...]: fitted poses instead of the regular tower
BLOCK_MASS = 0.013                    # kg
BLOCK_FRICTION = 0.392                # block on block
TABLE_FRICTION = 0.539                # block on table
DAMPING_RATIO = 0.737                 # contact damping ratio; stands in for restitution (see the module that wrote this)
VARIATION = {"size_mm": [0.3, 0.3, 0.1], "mass_frac": 0.05, "friction_frac": 0.1, "seed": 0}  # per-block sigma, cut at 2 sigma
BLOCK_COLOUR = (0.76, 0.6, 0.42)
HOLD = None                           # or {"block", "path": [[s, d_along, d_up], ...], "release_s", "axis_only"}: a scripted hand
PUSH = None                           # or {"block", "distance", "seconds", "direction"}: the finger; rollout(push={...}) overrides
DURATION_S = 4.0                      # used when rollout gives neither duration_s nor steps
CAMERA_EYE = (0.45, -0.45, 0.25)
CAMERA_LOOK_AT = (0.0, 0.0, 0.126)
CAMERA_FOV_Y = 40.0
CAMERA_SIZE = (480, 360)

# ---- fixed numerics: reported with every result, not tuned ------------------------------------
CONTROL_HZ = 125.0                    # one control step = 8 ms
SUBSTEPS = 4                          # physics step 2 ms (500 Hz)
SOLVER_ITERATIONS = 50                # 20 printed "iterations limit reached" on a 44-block tower
LS_ITERATIONS = 20
CONTACT_TIMECONST = 0.004
TABLE_HALF = (0.6, 0.6)
TABLE_TOP = 0.003                     # above z = 0, where RSS draws its ground plane
FINGER_HALF = (0.03, 0.004, 0.004)    # the pushing finger: a 60 x 8 x 8 mm rod, long side along the push
FINGER_CLEARANCE = 0.005              # the finger starts this far from the block's end
OBJECT_NAME = "tower"


# ---- layout -----------------------------------------------------------------------------------
def layout():
    """Every block as [name, layer, x, y, yaw, section] in the tower frame."""
    if BLOCKS is not None:
        return [list(block) for block in BLOCKS]
    rows = []
    for layer in range(LAYERS):
        for slot in range(3):
            name = f"L{layer:02d}_{slot}"
            if name not in MISSING:
                across = (slot - 1) * BLOCK_SIZE[1]
                rows.append([name, layer, 0.0, across, 0.0, "tower"] if layer % 2 == 0
                            else [name, layer, across, 0.0, math.pi / 2, "tower"])
    return rows


_rng = np.random.RandomState(int(VARIATION["seed"]))


def _draw(sigma):
    return float(np.clip(_rng.normal(0.0, sigma), -2 * sigma, 2 * sigma)) if sigma > 0 else 0.0


ROWS = layout()
NAMES = [row[0] for row in ROWS]
SECTIONS = {row[0]: row[5] for row in ROWS}
HALF, MASS_FACTOR, FRICTION_FACTOR = [], [], []
for _ in ROWS:                        # one block at a time: three sides, mass, friction
    HALF.append([(side + _draw(sigma / 1000.0)) / 2 for side, sigma in zip(BLOCK_SIZE, VARIATION["size_mm"])])
    MASS_FACTOR.append(1.0 + _draw(VARIATION["mass_frac"]))
    FRICTION_FACTOR.append(1.0 + _draw(VARIATION["friction_frac"]))
HALF = np.array(HALF)
_REFERENCE_FRICTION = 0.5             # what the blocks are built with; apply_contact scales it in the solver


def start_poses(shift=None):
    """World start pose of every block, [N, 7] (x y z qx qy qz qw). ``shift`` moves a block or a whole section
    in the tower frame: {"<block or section name>": [dx, dy]}."""
    x0, y0, turn = TOWER_XY_YAW
    cos, sin = math.cos(turn), math.sin(turn)
    poses = []
    for name, layer, x, y, yaw, section in ROWS:
        for key in (section, name):
            if shift and key in shift:
                x, y = x + shift[key][0], y + shift[key][1]
        angle = yaw + turn
        poses.append([x0 + cos * x - sin * y, y0 + sin * x + cos * y, (layer + 0.5) * BLOCK_SIZE[2] + TABLE_TOP,
                      0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
    return np.array(poses)


# ---- scene ------------------------------------------------------------------------------------
physics = replace(MUJOCO_500HZ, control_hz=CONTROL_HZ, substeps=SUBSTEPS,
                  backends=(("*", backend("mujoco", nconmax=8000, njmax=30000, enable_multiccd=True,
                                          iterations=SOLVER_ITERATIONS, ls_iterations=LS_ITERATIONS)),))
scene = rss.Scene(cell="void", name="tower_scene", physics=physics)
scene.add(rss.box(name="table", half=(TABLE_HALF[0], TABLE_HALF[1], 0.02), at=(0.0, 0.0, TABLE_TOP - 0.02), static=True,
                  friction=_REFERENCE_FRICTION, color=(0.9, 0.9, 0.9)))
_shades = (1.0, 0.92, 1.08, 0.85)     # neighbouring blocks differ a little, so a render shows them apart
for _index, _pose in enumerate(start_poses()):
    scene.add(rss.box(name=NAMES[_index], half=tuple(HALF[_index]), at=tuple(_pose[:3]), quat=tuple(_pose[3:]),
                      mass=BLOCK_MASS * MASS_FACTOR[_index], friction=_REFERENCE_FRICTION * FRICTION_FACTOR[_index],
                      color=tuple(min(1.0, channel * _shades[_index % 4]) for channel in BLOCK_COLOUR)))
scene.add(rss.box(name="finger", half=FINGER_HALF, at=(0.0, -TABLE_HALF[1] + 0.05, FINGER_HALF[2] + TABLE_TOP + 0.001),
                  mass=0.5, friction=0.4, color=(0.1, 0.1, 0.1)))
scene.cameras.append(rss.CameraSpec(name="camera", width=CAMERA_SIZE[0], height=CAMERA_SIZE[1], eye=CAMERA_EYE,
                                    look_at=CAMERA_LOOK_AT, up=(0.0, 0.0, 1.0), fov_y=CAMERA_FOV_Y))
BUILD = {"renderer": "tiled"}


# ---- contact parameters, changed without a rebuild --------------------------------------------
def _solver_array(sim, name):
    """A MuJoCo model array as (values with a leading world axis, write): MuJoCo-Warp on a GPU, MuJoCo's C model on a CPU."""
    solver = sim.pipeline.solver
    if solver.use_mujoco_cpu:
        array = getattr(solver.mj_model, name)
        return array.reshape((1,) + array.shape).copy(), lambda values: array.__setitem__(Ellipsis, values[0])
    array = getattr(solver.mjw_model, name)
    return array.numpy().copy(), array.assign


def apply_contact(sim, friction=BLOCK_FRICTION, table_friction=TABLE_FRICTION, damping_ratio=DAMPING_RATIO):
    """Write friction and contact damping into the solver's own arrays; returns what was written."""
    sizes = _solver_array(sim, "geom_size")[0]
    sizes = sizes.reshape(-1, sizes.shape[-1])[-sim.pipeline.solver.mj_model.ngeom:]
    table = np.where(np.isclose(sizes[:, 0], TABLE_HALF[0], atol=1e-4) & np.isclose(sizes[:, 1], TABLE_HALF[1], atol=1e-4))[0]
    blocks = np.where(np.abs(sizes - np.array(BLOCK_SIZE) / 2).max(1) < 0.002)[0]
    if not hasattr(sim, "_tower_friction"):
        sim._tower_friction = _solver_array(sim, "geom_friction")[0]
    values, write = _solver_array(sim, "geom_friction")
    values[:, blocks, 0] = sim._tower_friction[:, blocks, 0] / _REFERENCE_FRICTION * friction
    values[:, table, 0] = table_friction
    write(values)
    values, write = _solver_array(sim, "geom_priority")
    values.reshape(-1, sizes.shape[0])[:, table] = 1
    write(values)
    values, write = _solver_array(sim, "geom_solref")
    values[..., 0], values[..., 1] = CONTACT_TIMECONST, damping_ratio
    write(values)
    return {"block_geoms": int(len(blocks)), "table_geoms": int(len(table)), "friction": friction,
            "table_friction": table_friction, "damping_ratio": damping_ratio}


# ---- moving things ----------------------------------------------------------------------------
def _write(sim, name, pose=None, velocity=None, along=None):
    """Write a free body's coordinates: its whole ``pose`` and 6-D ``velocity``, or with ``along`` = (axis, position,
    speed) only its place and speed along one world direction, leaving it free in every other way."""
    entity = sim.scene.by_name[name]
    q, qd = sim.state.joint_q.numpy(), sim.state.joint_qd.numpy()
    c, d = entity.coord_start, entity.dof_start
    if along is not None:
        axis, position, speed = along
        q[c:c + 3] += axis * (position - q[c:c + 3] @ axis)
        qd[d:d + 3] += axis * (speed - qd[d:d + 3] @ axis)
    else:
        q[c:c + 7] = pose
        qd[d:d + 6] = velocity
    sim.state.joint_q.assign(q)
    sim.state.joint_qd.assign(qd)


def _long_axis(pose):
    """World direction of a block's long side, from its start pose (a yaw about +z)."""
    yaw = 2.0 * math.atan2(pose[5], pose[6])
    return np.array([math.cos(yaw), math.sin(yaw), 0.0])


def _hold(sim, hold, start, t):
    """While the hand holds: put the held block on its path. False once released."""
    if not hold or t >= hold["release_s"]:
        return False
    times = [row[0] for row in hold["path"]]
    pose = start[NAMES.index(hold["block"])]
    axis = _long_axis(pose)

    def at(time):
        return np.array([np.interp(time, times, [row[1] for row in hold["path"]]),
                         np.interp(time, times, [row[2] for row in hold["path"]])])

    (along, up), (speed_along, speed_up) = at(t), (at(t + 0.005) - at(t)) / 0.005
    if hold.get("axis_only"):         # the hand sets only the motion along the pull; the block still settles with the tower
        _write(sim, hold["block"], along=(axis, pose[:3] @ axis + along, speed_along))
    else:
        moved = pose.copy()
        moved[:3] += axis * along + np.array([0.0, 0.0, up])
        _write(sim, hold["block"], moved, [*(axis * speed_along)[:2], speed_up, 0.0, 0.0, 0.0])
    return True


def _finger(sim, push, start, t):
    """Put the finger on its straight path into the pushed block's end; returns the path error after the last step."""
    pose = start[NAMES.index(push["block"])]
    axis = _long_axis(pose) * (1 if push.get("direction", 1) >= 0 else -1)
    travel = push["distance"] + FINGER_CLEARANCE
    done = min(max(t / push["seconds"], 0.0), 1.0)
    back = HALF[NAMES.index(push["block"])][0] + FINGER_CLEARANCE + FINGER_HALF[0]
    centre = pose[:3] + axis * (travel * done - back)
    speed = travel / push["seconds"] if 0.0 <= t < push["seconds"] else 0.0
    error = float(np.linalg.norm(np.asarray(sim["finger"].position, dtype=float) - centre)) if t > 0 else 0.0
    yaw = math.atan2(axis[1], axis[0])
    _write(sim, "finger", [*centre, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)], [*(axis * speed), 0.0, 0.0, 0.0])
    return error


def read_poses(sim):
    """Pose of every block, [N, 7], in one read of the solver's body array."""
    if not hasattr(sim, "_tower_rows"):
        sim._tower_rows = np.array([sim.scene.by_name[name].body for name in NAMES])
    sim.sync()
    return np.asarray(sim.state.body_q.numpy(), dtype=np.float64)[sim._tower_rows]


def _up(poses):
    """Each block's own +z in the world, [N, 3]."""
    x, y, z, w = poses[:, 3], poses[:, 4], poses[:, 5], poses[:, 6]
    return np.stack([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)], 1)


def tower_state(start, end, moved=()):
    """What a push or a pull did to the rest of the tower, from the first and last poses.

    lean_deg: how far the line from the lowest layer's centre to the highest layer's centre turned from where it
    started. standing: no block other than ``moved`` dropped half a block height or tipped past 20 degrees."""
    others = np.array([name not in moved for name in NAMES])
    heights = start[:, 2]
    low, high = heights <= heights.min() + 1e-6, heights >= heights.max() - 1e-6
    line = lambda poses: poses[high & others, :3].mean(0) - poses[low & others, :3].mean(0)
    a, b = line(start), line(end)
    lean = math.degrees(math.acos(float(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))))
    shift = np.linalg.norm(end[:, :3] - start[:, :3], axis=1)
    tip = np.degrees(np.arccos(np.clip((_up(start) * _up(end)).sum(1), -1.0, 1.0)))
    dropped = start[:, 2] - end[:, 2]
    standing = bool(((dropped < BLOCK_SIZE[2] / 2) & (tip < 20.0))[others].all())
    return {"lean_deg": round(lean, 4), "standing": standing,
            "max_other_shift_m": round(float(shift[others].max()), 5), "mean_other_shift_m": round(float(shift[others].mean()), 5),
            "max_other_tip_deg": round(float(tip[others].max()), 3),
            "top_shift_m": round(float(np.linalg.norm((end[high & others, :2] - start[high & others, :2]).mean(0))), 5)}


def simulate(sim, steps, params=None, until_rest=False):
    """Step the tower; a generator that yields once per control step and returns the trajectory and its summary.

    ``params`` overrides, for this run only and without a rebuild: friction, table_friction, damping_ratio,
    shift (see start_poses), hold and push (as HOLD and PUSH; None switches one off)."""
    params = dict(params or {})
    contact = apply_contact(sim, params.get("friction", BLOCK_FRICTION), params.get("table_friction", TABLE_FRICTION),
                            params.get("damping_ratio", DAMPING_RATIO))
    hold, push = params.get("hold", HOLD), params.get("push", PUSH)
    start = start_poses(params.get("shift"))
    for name, pose in zip(NAMES, start):
        _write(sim, name, pose, [0.0] * 6)
    dt = 1.0 / CONTROL_HZ
    times, poses, released, worst, quiet = [0.0], [start], None, 0.0, 0
    for k in range(steps):
        if hold and not _hold(sim, hold, start, k * dt) and released is None:
            released = k * dt
        if push:
            worst = max(worst, _finger(sim, push, start, k * dt))
        sim.run(1)
        times.append((k + 1) * dt)
        poses.append(read_poses(sim))
        yield
        quiet = quiet + 1 if np.abs(poses[-1][:, :3] - poses[-2][:, :3]).max() / dt < 0.01 else 0
        busy = (k + 1) * dt < max(hold["release_s"] if hold else 0.0, push["seconds"] if push else 0.0) + 0.5
        if until_rest and quiet >= 12 and not busy:
            break                      # at rest: the last pose stands for the remaining time
    poses = np.array(poses)
    moved = [spec["block"] for spec in (hold, push) if spec]
    result = {"applied": True, "steps": len(times) - 1, "control_hz": CONTROL_HZ, "solver": type(sim.pipeline.solver).__name__,
              "finite": bool(np.isfinite(poses).all()), "contact": contact, "hold": hold, "push": push,
              "released_at_s": released, "blocks": len(NAMES), "tower": tower_state(poses[0], poses[-1], moved)}
    if push:
        index = NAMES.index(push["block"])
        axis = _long_axis(start[index]) * (1 if push.get("direction", 1) >= 0 else -1)
        result.update(pushed_m=round(float((poses[-1][index, :3] - start[index, :3]) @ axis), 5),
                      pusher_tracking_error_max_m=round(worst, 5), pusher_tracked=worst < 0.005)
    return {"t": np.array(times), "poses": poses, "names": NAMES, "half": HALF, "result": result}


def rollout(sim, params=None, steps=None, until_rest=True):
    """One rollout for a script that drives the built scene itself (a tuning loop): reset, simulate, return."""
    sim.reset()
    run = simulate(sim, int(steps or round(DURATION_S * CONTROL_HZ)), params, until_rest)
    try:
        while True:
            next(run)
    except StopIteration as stop:
        return stop.value


def _routine(sim, kind):
    asked = globals().get("ROLLOUT") or {}
    steps, take = int(asked.get("steps") or round(DURATION_S * CONTROL_HZ)), asked.get("take") or "run"
    params = {"push": asked.get("push") or PUSH if kind == "push" else None, "hold": HOLD if kind == "pull" else None}
    if kind == "push" and not params["push"]:
        raise ValueError("The push routine needs a push: rollout(push={'block': 'L08_1', 'distance': 0.03, 'seconds': 3}).")
    if kind == "pull" and not HOLD:
        raise ValueError("The pull routine needs the template's hold: {'block', 'path', 'release_s'}.")
    out = yield from simulate(sim, steps, params)
    folder = Path(__file__).resolve().parent
    np.savez_compressed(folder / f"traj-{take}.npz", t=out["t"], poses=out["poses"], names=np.array(NAMES), half=HALF)
    (folder / f"push-{take}.json").write_text(json.dumps(
        dict(out["result"], routine=kind, take=take, trajectory=str(folder / f"traj-{take}.npz"))))


def push_routine(sim):
    yield from _routine(sim, "push")


def pull_routine(sim):
    yield from _routine(sim, "pull")


def settle_routine(sim):
    yield from _routine(sim, "settle")


routines = {"push": push_routine, "pull": pull_routine, "settle": settle_routine}
'''

KEYS = ("shape", "size", "layers", "position", "yaw", "color", "mass", "friction", "table_friction", "damping_ratio",
        "variation", "missing", "blocks", "hold", "push", "camera")
DEFAULTS = {"size": [0.072, 0.024, 0.014], "layers": 18, "position": [0.0, 0.0], "yaw": 0.0, "color": "wood",
            "mass": 0.013, "friction": 0.392, "table_friction": 0.539, "damping_ratio": 0.737,
            "variation": {"size_mm": [0.3, 0.3, 0.1], "mass_frac": 0.05, "friction_frac": 0.1, "seed": 0},
            "missing": [], "blocks": None, "hold": None, "push": None,
            "camera": {"eye": [0.45, -0.45, 0.25], "target": [0.0, 0.0, 0.126], "fov_y": 40.0}}
MAX_LAYERS = 40
MAX_BLOCKS = 120
PUSH_KEYS = ("block", "distance", "seconds", "direction")
SIZE_MEANING = ("[length, width, height] of one block in metres; default 0.072 x 0.024 x 0.014. The tower is layers x 3 "
                "blocks, an even layer's blocks along x and an odd layer's along y")
PARAMETERS = {
    "shape": "tower",
    "size": "one block [length, width, height], metres; default [0.072, 0.024, 0.014]",
    "layers": f"layers of three blocks (default 18, at most {MAX_LAYERS}); block names are L<layer>_<slot>, e.g. L08_1 (slot 1 is the middle)",
    "position": "[x, y] of the tower's axis on the table, metres", "yaw": "radians about +z",
    "color": "wood (default), another colour name, '#rrggbb' or [r, g, b] in 0-1",
    "mass": "kg per block (default 0.013)", "friction": "block on block (default 0.392)",
    "table_friction": "block on table (default 0.539)",
    "damping_ratio": "contact damping ratio, 0.5 to 1 (default 0.737); stands in for restitution",
    "variation": "{'size_mm': [l, w, h], 'mass_frac', 'friction_frac', 'seed'}: per-block sigma, cut at 2 sigma; zeros for identical blocks",
    "missing": "block names left out of the regular tower, e.g. ['L07_1']",
    "blocks": "fitted poses instead of the regular tower: [{'name', 'layer', 'x', 'y', 'yaw', 'section'}], x and y from the tower's axis",
    "hold": "a scripted hand: {'block', 'path': [[seconds, along, up], ...] metres along the block's long side and up, 'release_s', 'axis_only'}",
    "push": "the finger: {'block', 'distance' (m), 'seconds', 'direction' (1 or -1 along the block's long side)}",
    "camera": "{'eye': [x, y, z], 'target': [x, y, z], 'fov_y': degrees}",
}
_BLOCK_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,31}")


def _positive(value, what, high):
    value = _finite(value, what)
    if not 0 < value <= high:
        raise ValueError(f"template {what} must be above 0 and at most {high}.")
    return value


def validate_push(push, names=None):
    """A finger push ``{"block", "distance", "seconds", "direction"}``, checked; ``names`` are the tower's blocks."""
    if not isinstance(push, dict) or set(push) - set(PUSH_KEYS) or not {"block", "distance", "seconds"} <= set(push):
        raise ValueError("a tower push is {'block': 'L08_1', 'distance': 0.03, 'seconds': 3.0, 'direction': 1}: metres along "
                         "the block's long side, the seconds the finger takes, and 1 or -1 for the way it pushes.")
    if not isinstance(push["block"], str) or (names is not None and push["block"] not in names):
        raise ValueError(f"push block {push['block']!r} is not a block of this tower.")
    if push.get("direction", 1) not in (1, -1):
        raise ValueError("push direction must be 1 or -1.")
    return {"block": push["block"], "distance": _positive(push["distance"], "push distance", 0.2),
            "seconds": _positive(push["seconds"], "push seconds", 600.0), "direction": push.get("direction", 1)}


def block_names(resolved):
    """The names of a resolved tower's blocks."""
    if resolved["blocks"] is not None:
        return [row[0] for row in resolved["blocks"]]
    return [name for name in (f"L{layer:02d}_{slot}" for layer in range(resolved["layers"]) for slot in range(3))
            if name not in resolved["missing"]]


def _blocks(value, height_layers):
    if not isinstance(value, list) or not 1 <= len(value) <= MAX_BLOCKS:
        raise ValueError(f"template blocks must be a list of 1 to {MAX_BLOCKS} blocks.")
    rows = []
    for block in value:
        if not isinstance(block, dict) or set(block) - {"name", "layer", "x", "y", "yaw", "section"} or not {"name", "layer", "x", "y"} <= set(block):
            raise ValueError("each template block is {'name', 'layer', 'x', 'y', 'yaw', 'section'}; yaw and section may be left out.")
        name, layer, section = block["name"], block["layer"], block.get("section", "tower")
        if not isinstance(name, str) or not _BLOCK_NAME.fullmatch(name) or name in [row[0] for row in rows] or name in ("table", "finger"):
            raise ValueError(f"template block name {name!r} must be unique: a letter, then letters, digits or _.")
        if isinstance(layer, bool) or not isinstance(layer, int) or not 0 <= layer < height_layers:
            raise ValueError(f"template block {name} layer must be a whole number from 0 to {height_layers - 1}.")
        if not isinstance(section, str) or not _BLOCK_NAME.fullmatch(section):
            raise ValueError(f"template block {name} section must be a short name.")
        x, y, yaw = (_finite(block.get(key, 0.0), f"block {name} {key}") for key in ("x", "y", "yaw"))
        if abs(x) > 0.3 or abs(y) > 0.3:
            raise ValueError(f"template block {name} x and y are measured from the tower's axis and must be within 0.3 m.")
        rows.append([name, layer, round(x, 6), round(y, 6), round(yaw, 6), section])
    return rows


def _hold(value, names):
    if not isinstance(value, dict) or set(value) - {"block", "path", "release_s", "axis_only"} or not {"block", "path", "release_s"} <= set(value):
        raise ValueError("template hold is {'block', 'path': [[seconds, along, up], ...], 'release_s', 'axis_only'}.")
    if value["block"] not in names:
        raise ValueError(f"hold block {value['block']!r} is not a block of this tower.")
    path = value["path"]
    if not isinstance(path, list) or not 2 <= len(path) <= 2000:
        raise ValueError("hold path must be 2 to 2000 rows [seconds, along, up].")
    rows = [_vector(row, 3, "hold path rows") for row in path]
    if rows[0][0] != 0 or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])) or any(abs(v) > 0.3 for row in rows for v in row[1:]):
        raise ValueError("hold path times start at 0 and strictly increase; its displacements are within 0.3 m.")
    return {"block": value["block"], "path": rows, "release_s": _positive(value["release_s"], "hold release_s", 600.0),
            "axis_only": bool(value.get("axis_only", False))}


def resolve(spec=None, previous=None):
    """Tower template data with every default filled in, checked. ``previous`` (a resolved tower) supplies what a
    later ``update_scene`` leaves out; ``camera`` and ``variation`` merge key by key."""
    spec = dict(spec or {})
    unknown = sorted(set(spec) - set(KEYS))
    if unknown:
        raise ValueError(f"template has unknown keys {unknown}; a tower uses {list(KEYS)}.")
    base = previous if previous and previous.get("shape") == "tower" else DEFAULTS
    out = {"shape": "tower"}
    for key in KEYS[1:]:
        out[key] = spec[key] if key in spec else base[key]
    out["size"] = _vector(out["size"], 3, "size")
    if not all(0.004 <= v <= 0.5 for v in out["size"]):
        raise ValueError("template size values must be between 0.004 and 0.5 metres.")
    if isinstance(out["layers"], bool) or not isinstance(out["layers"], int) or not 1 <= out["layers"] <= MAX_LAYERS:
        raise ValueError(f"template layers must be a whole number from 1 to {MAX_LAYERS}.")
    out["position"] = _vector(out["position"], 2, "position")
    if max(abs(v) for v in out["position"]) > 0.4:
        raise ValueError("template position must be within 0.4 m of the table's centre.")
    out["yaw"] = _finite(out["yaw"], "yaw")
    colour_rgb(out["color"])
    out["mass"] = _positive(out["mass"], "mass", 10.0)
    out["friction"] = _positive(out["friction"], "friction", 5.0)
    out["table_friction"] = _positive(out["table_friction"], "table_friction", 5.0)
    out["damping_ratio"] = _finite(out["damping_ratio"], "damping_ratio")
    if not 0.5 <= out["damping_ratio"] <= 1.0:
        raise ValueError("template damping_ratio must be from 0.5 to 1: below 0.5 a contact gains energy and the tower blows apart.")
    variation = dict(base["variation"])
    if "variation" in spec:
        if not isinstance(spec["variation"], dict) or set(spec["variation"]) - set(variation):
            raise ValueError(f"template variation keys are {sorted(variation)}.")
        variation.update(spec["variation"])
    size_mm = _vector(variation["size_mm"], 3, "variation size_mm")
    fractions = [_finite(variation[key], f"variation {key}") for key in ("mass_frac", "friction_frac")]
    if not all(0 <= v <= 2 for v in size_mm) or not all(0 <= v <= 0.3 for v in fractions) or isinstance(variation["seed"], bool) \
            or not isinstance(variation["seed"], int) or not 0 <= variation["seed"] < 2 ** 31:
        raise ValueError("template variation: size_mm 0 to 2 each, mass_frac and friction_frac 0 to 0.3, seed a whole number.")
    out["variation"] = {"size_mm": size_mm, "mass_frac": fractions[0], "friction_frac": fractions[1], "seed": variation["seed"]}
    if out["blocks"] is not None:
        if "blocks" in spec:                 # a previous template's rows are already checked
            out["blocks"] = _blocks(spec["blocks"], MAX_LAYERS)
        if spec.get("missing"):
            raise ValueError("template missing names blocks of the regular tower; with blocks, leave them out of the list.")
        out["missing"] = []
    else:
        regular = [f"L{layer:02d}_{slot}" for layer in range(out["layers"]) for slot in range(3)]
        if not isinstance(out["missing"], list) or set(out["missing"]) - set(regular) or len(set(out["missing"])) >= len(regular):
            raise ValueError("template missing must name blocks of the tower, L<layer>_<slot> such as 'L07_1', and leave at least one.")
        out["missing"] = sorted(set(out["missing"]))
    names = block_names(out)
    out["hold"] = None if out["hold"] is None else _hold(out["hold"], names)
    out["push"] = None if out["push"] is None else validate_push(out["push"], names)
    camera = dict(base["camera"])
    if spec.get("camera") is not None:
        if not isinstance(spec["camera"], dict) or set(spec["camera"]) - {"eye", "target", "fov_y"}:
            raise ValueError("template camera must be {'eye': [x, y, z], 'target': [x, y, z], 'fov_y': degrees}.")
        camera.update(spec["camera"])
    out["camera"] = {"eye": _vector(camera["eye"], 3, "camera eye"), "target": _vector(camera["target"], 3, "camera target"),
                     "fov_y": _finite(camera["fov_y"], "camera fov_y")}
    if out["camera"]["eye"] == out["camera"]["target"] or out["camera"]["eye"][2] <= 0 or not 1 <= out["camera"]["fov_y"] <= 120:
        raise ValueError("template camera eye must be above the table and differ from its target; fov_y is 1 to 120 degrees.")
    out["mask_key"] = mask_key(out["color"], "tower")
    return out


def scene_source(resolved):
    """The scene file for a resolved tower template."""
    values = {"BLOCK_SIZE": tuple(resolved["size"]), "LAYERS": resolved["layers"],
              "TOWER_XY_YAW": (*resolved["position"], resolved["yaw"]), "MISSING": resolved["missing"],
              "BLOCKS": resolved["blocks"], "BLOCK_MASS": resolved["mass"], "BLOCK_FRICTION": resolved["friction"],
              "TABLE_FRICTION": resolved["table_friction"], "DAMPING_RATIO": resolved["damping_ratio"],
              "VARIATION": resolved["variation"], "BLOCK_COLOUR": tuple(colour_rgb(resolved["color"])),
              "HOLD": resolved["hold"], "PUSH": resolved["push"], "CAMERA_EYE": tuple(resolved["camera"]["eye"]),
              "CAMERA_LOOK_AT": tuple(resolved["camera"]["target"]), "CAMERA_FOV_Y": resolved["camera"]["fov_y"]}
    text = TEMPLATE
    for name, value in values.items():
        text, count = re.subn(rf"^{name} = [^#\n]*?( *#.*)?$", lambda m, n=name, v=repr(value): f"{n} = {v}" + (m.group(1) or ""),
                              text, count=1, flags=re.M)
        if count != 1:
            raise ValueError(f"tower template text has no line for {name}.")
    return text
