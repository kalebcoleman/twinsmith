"""Scene source for the pinned RSS live scene loader.

The text is trusted Python that RSS executes. It holds a static table, one coloured object (a box, a
T made of two boxes on one rigid body, or a Jenga-sized block), a small cylinder pusher and one camera. Every parameter is
plain data near the top. The ``push`` and ``settle`` routines read ``ROLLOUT`` (set by the bridge's
``rollout`` action) and fall back to ``PUSH`` and ``DURATION_S`` below.

Which solver steps it, from the RSS source (pin ba3650c): ``rss.Scene`` defaults to the ``GENERIC``
arrangement, whose catch-all backend is ``vbd`` (``specs/arrangement.py``). On a GPU that puts every free
rigid body, the pusher included, on SolverVBD, which does not take its coordinates from ``joint_q`` and
``joint_qd``: the pusher write below is silently ignored and the pusher stays at its start. A CPU device
swaps ``GENERIC`` for MuJoCo (``scene/physics/devices.py``), which is why the same scene worked there. This
scene therefore names ``MUJOCO_500HZ`` (the arm and every rigid body in one MuJoCo island at 50 Hz control,
10 substeps) so that cpu and cuda:0 run the same physics: plain MuJoCo on the CPU, Newton's MuJoCo-Warp on
the GPU. The push result records the solver class that stepped the scene.

How things move, from the RSS source (checkout paths, pin ba3650c):
* ``domains/rigid/entity.py`` ``_add_primitive`` records ``coord_start`` and ``dof_start`` for a free
  body, and its ``reset`` writes ``state.joint_q[coord_start:coord_start+7]``. The pusher is placed
  the same way, every control step, with its velocity written to ``joint_qd[dof_start:dof_start+6]``.
* ``body_q`` and ``body_qd`` are outputs: the solver rebuilds them from ``joint_q`` and ``joint_qd``
  every step, so a write to them is discarded (``Sim.reset_scene`` in ``sim/sim.py`` zeroes
  ``joint_qd`` first for the same reason).
* RSS draws a ground plane at z = 0. A table top at exactly z = 0 is coplanar with it and the two fight
  per pixel (black and white speckle in every render, on CPU and GPU alike), so the table top is
  ``TABLE_TOP`` = 3 mm above it.
* ``specs/asset.py`` ``ObjectSpec.shape.add_to(...)`` draws the shape onto the body. ``TeeShape``
  overrides it to draw two boxes on that one body, so the T moves as one rigid body.
"""

TEMPLATE = '''import json
import math
from pathlib import Path

import numpy as np
import robosimstudio as rss
from robosimstudio.specs import ObjectSpec
from robosimstudio.specs.asset import ShapeSpec
from robosimstudio.specs.physics import MUJOCO_500HZ

# ---- parameters: plain data, edit these -------------------------------------------------------
# Table frame: metres, +z up. In the default camera image +x is right and +y is up (away from the
# camera); image y points down. Yaw in radians about +z.
OBJECT_NAME = "block"           # the name masks and results use; compare with keys=["block=red"]
SHAPE = "__SHAPE__"              # "box", "tee" or "jenga" (a box lying flat; set BOX_HALF)
BOX_HALF = (0.065, 0.07, 0.02)   # box half sizes x, y, z
TEE_BAR_HALF = (0.065, 0.0175)   # tee bar: half length (x), half thickness (y)
TEE_STEM_HALF = (0.0175, 0.0525) # tee stem: half width (x), half length (y)
OBJECT_HALF_HEIGHT = 0.02        # tee half height (z); a box uses BOX_HALF[2]
OBJECT_XY_YAW = (0.0, 0.0, 0.0)  # start x, y [m] and yaw [rad]; yaw 0 = bar far, stem toward camera
OBJECT_MASS = 0.1
OBJECT_FRICTION = 0.4
OBJECT_COLOUR = (0.85, 0.05, 0.05)
TABLE_HALF = (0.4, 0.3)          # table half size x, y
TABLE_TOP = 0.003                # table top height [m]; keep it above 0, where RSS draws a ground plane
TABLE_COLOUR = (0.9, 0.9, 0.9)
PUSHER_RADIUS = 0.0125
PUSHER_HALF_HEIGHT = 0.03
PUSHER_COLOUR = (0.1, 0.1, 0.1)
PUSHER_Z = PUSHER_HALF_HEIGHT + TABLE_TOP + 0.001  # the pusher's centre height; also where it is placed
CAMERA_EYE = (-0.05, -0.05, 0.7)
CAMERA_LOOK_AT = (-0.05, 0.12, 0.0)
CAMERA_SIZE = (320, 240)

# The push. waypoints are [seconds, x, y] pusher positions on the table; the pusher moves in straight
# lines between them. object_velocity is an optional initial [vx, vy, yaw_rate] of the object in m/s
# and rad/s (friction stops it again; a push-only run leaves it None). rollout(push={...}) overrides this.
PUSH = {"waypoints": [[0.0, 0.0, -0.20], [2.0, 0.0, 0.0]], "object_velocity": None}
DURATION_S = 2.0                 # used when rollout gives neither duration_s nor steps


# ---- shapes -----------------------------------------------------------------------------------
class TeeShape(ShapeSpec):
    """A ShapeSpec that draws a T (bar and stem, two boxes) on one body, so it moves as one."""

    def add_to(self, builder, *, body, xform, label, cfg=None):
        import warp as wp

        common = {"body": body, "label": label}
        if cfg is not None:
            common["cfg"] = cfg
        (bar_x, bar_y), (stem_x, stem_y), half_z = self._tee
        total = 2 * bar_y + 2 * stem_y
        first = None
        for half_x, half_y, centre_y in ((bar_x, bar_y, total / 2 - bar_y), (stem_x, stem_y, -total / 2 + stem_y)):
            placed = wp.transform_multiply(xform, wp.transform((0.0, centre_y, 0.0), wp.quat_identity()))
            index = builder.add_shape_box(hx=half_x, hy=half_y, hz=half_z, xform=placed, **common)
            first = index if first is None else first
        return first


def tee_shape():
    shape = TeeShape(kind="box", half_extents=(TEE_BAR_HALF[0], TEE_BAR_HALF[1] + TEE_STEM_HALF[1], OBJECT_HALF_HEIGHT))
    object.__setattr__(shape, "_tee", (TEE_BAR_HALF, TEE_STEM_HALF, OBJECT_HALF_HEIGHT))
    return shape


# ---- scene ------------------------------------------------------------------------------------
if SHAPE == "tee":
    shape, half_height = tee_shape(), OBJECT_HALF_HEIGHT
elif SHAPE in ("box", "jenga"):
    shape, half_height = ShapeSpec(kind="box", half_extents=BOX_HALF), BOX_HALF[2]
else:
    raise ValueError("SHAPE must be 'box', 'tee' or 'jenga'")

# MUJOCO_500HZ, not the default GENERIC: on a GPU GENERIC runs free bodies on VBD, which ignores the pusher's
# joint_q / joint_qd writes below (see the module docstring). A CPU device uses MuJoCo either way.
scene = rss.Scene(cell="void", name="push_scene", physics=MUJOCO_500HZ)
scene.add(rss.box(name="table", half=(TABLE_HALF[0], TABLE_HALF[1], 0.02), at=(0.0, 0.0, TABLE_TOP - 0.02),
                  static=True, friction=OBJECT_FRICTION, color=TABLE_COLOUR))
scene.add(ObjectSpec(name=OBJECT_NAME, shape=shape, mass=OBJECT_MASS, friction=OBJECT_FRICTION,
                     color=OBJECT_COLOUR, pose=(OBJECT_XY_YAW[0], OBJECT_XY_YAW[1], half_height + TABLE_TOP),
                     quat=(0.0, 0.0, math.sin(OBJECT_XY_YAW[2] / 2), math.cos(OBJECT_XY_YAW[2] / 2))))
_first = PUSH["waypoints"][0]
scene.add(rss.cylinder(name="pusher", radius=PUSHER_RADIUS, half_height=PUSHER_HALF_HEIGHT,
                       at=(_first[1], _first[2], PUSHER_Z), mass=2.0,
                       friction=0.2, color=PUSHER_COLOUR))
scene.cameras.append(rss.CameraSpec(name="camera", width=CAMERA_SIZE[0], height=CAMERA_SIZE[1],
                                    eye=CAMERA_EYE, look_at=CAMERA_LOOK_AT, up=(0.0, 0.0, 1.0)))
BUILD = {"renderer": "tiled"}


# ---- moving things ----------------------------------------------------------------------------
def place_free_body(sim, name, x=None, y=None, velocity=None, z=None):
    """Write a free body's planar position and/or velocity into the solver's own coordinates.

    joint_q[coord_start:+7] is (x, y, z, qx, qy, qz, qw); joint_qd[dof_start:+6] is
    (vx, vy, vz, wx, wy, wz). Writing body_q or body_qd instead does nothing: the solver rebuilds them.
    Given ``z``, the body is also set upright (identity rotation, no spin) at that height: the pusher uses this
    so it slides on a plane like a planar actuator. Left free in z, the cylinder drifted (up, then 1 cm down into
    the table) under MuJoCo-Warp and rode over a Jenga block instead of pushing it, while plain MuJoCo held it.
    """
    entity = sim.scene.by_name[name]
    state = sim.state
    if x is not None:
        q = state.joint_q.numpy()
        c = entity.coord_start
        q[c], q[c + 1] = x, y
        if z is not None:
            q[c + 2] = z
            q[c + 3:c + 7] = [0.0, 0.0, 0.0, 1.0]
        state.joint_q.assign(q)
    if velocity is not None:
        qd = state.joint_qd.numpy()
        d = entity.dof_start
        qd[d:d + 6] = [velocity[0], velocity[1], 0.0, 0.0, 0.0, velocity[2] if len(velocity) > 2 else 0.0]
        state.joint_qd.assign(qd)


def pusher_at(waypoints, t):
    """Position and velocity on the piecewise-linear path at time t; clamped before and after."""
    if t <= waypoints[0][0]:
        return waypoints[0][1], waypoints[0][2], 0.0, 0.0
    for (t0, x0, y0), (t1, x1, y1) in zip(waypoints, waypoints[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0), (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)
    return waypoints[-1][1], waypoints[-1][2], 0.0, 0.0


def _rollout(hz):
    asked = globals().get("ROLLOUT") or {}
    return asked.get("push") or PUSH, int(asked.get("steps") or round(DURATION_S * hz)), asked.get("take") or "run"


def _yaw(pose):
    return 2.0 * math.atan2(float(pose[5]), float(pose[6]))


def push_routine(sim):
    """Move the pusher along the PUSH waypoints for the asked number of steps, one step per yield."""
    hz = float(sim.request.physics.control_hz)
    push, steps, take = _rollout(hz)
    waypoints = [list(map(float, w)) for w in push["waypoints"]]
    dt = 1.0 / hz
    start = [float(v) for v in sim[OBJECT_NAME].position]
    start_yaw = _yaw(sim[OBJECT_NAME].pose)
    if push.get("object_velocity"):
        place_free_body(sim, OBJECT_NAME, velocity=[float(v) for v in push["object_velocity"]])
    worst = 0.0
    for k in range(steps):
        x, y, vx, vy = pusher_at(waypoints, k * dt)
        place_free_body(sim, "pusher", x, y, velocity=[vx, vy, 0.0], z=PUSHER_Z)
        sim.run(1)
        want = pusher_at(waypoints, (k + 1) * dt)
        have = sim["pusher"].position
        worst = max(worst, math.hypot(float(have[0]) - want[0], float(have[1]) - want[1]))
        yield
    end = [float(v) for v in sim[OBJECT_NAME].position]
    result = {"applied": True, "routine": "push", "take": take, "steps": steps, "control_hz": hz,
              "waypoints": waypoints, "object_velocity": push.get("object_velocity"),
              "pusher_tracking_error_max_m": round(worst, 5), "pusher_tracked": worst < 0.005,
              "solver": type(sim.pipeline.solver).__name__,
              "object": OBJECT_NAME, "object_start_xy_m": start[:2], "object_end_xy_m": end[:2],
              "object_start_yaw_rad": round(start_yaw, 4), "object_end_yaw_rad": round(_yaw(sim[OBJECT_NAME].pose), 4)}
    Path(__file__).with_name(f"push-{take}.json").write_text(json.dumps(result))


def settle_routine(sim):
    """Step with no push for the asked number of steps (a control run: nothing should move)."""
    _, steps, take = _rollout(float(sim.request.physics.control_hz))
    for _ in range(steps):
        sim.run(1)
        yield
    Path(__file__).with_name(f"push-{take}.json").write_text(json.dumps(
        {"applied": False, "routine": "settle", "take": take, "steps": steps}))


routines = {"push": push_routine, "settle": settle_routine}
'''


import json
import math
import re

SHAPES = ("tee", "box", "jenga", "tower")
TEMPLATE_KEYS = ("shape", "size", "position", "yaw", "color", "mass", "friction", "camera", "push")
COLOURS = {  # name -> RGB in [0, 1]; each name is also an RSS mask colour word (compare.py NAMED_COLOURS)
    "red": (0.85, 0.05, 0.05), "orange": (0.95, 0.50, 0.10), "yellow": (0.90, 0.80, 0.10),
    "green": (0.10, 0.60, 0.15), "blue": (0.10, 0.25, 0.85), "wood": (0.76, 0.60, 0.42),
}
SIZE_MEANING = {
    "tee": "[bar_length, stem_length, thickness, height] in metres; the bar is bar_length x thickness and the stem "
           "thickness x stem_length, so the T is thickness + stem_length deep",
    "box": "[x_length, y_length, height] in metres",
    "jenga": "[x_length, y_length, height] in metres, lying flat: x is the long axis. Default 0.072 x 0.024 x 0.014 "
             "(72 x 24 x 14 mm, a Jenga-style block), so its long side lies along x and a push along +y hits the long face",
}
DEFAULT_CAMERA = {"eye": [-0.05, -0.05, 0.7], "target": [-0.05, 0.12, 0.0]}
CLOSE_CAMERA = {"eye": [0.0, -0.06, 0.45], "target": [0.0, 0.0, 0.0]}
FRAME = ("Table frame in metres. In the default camera image +x is right and +y is up (away from the camera); image y "
         "points down, so a block that moves up-left in the picture has -dx, -dy pixels and moves toward -x, +y.")
TABLE_LIMIT = (0.38, 0.28)
DEFAULTS = {
    "tee": {"size": [0.13, 0.105, 0.035, 0.04], "color": "red", "mass": 0.1, "friction": 0.4, "camera": DEFAULT_CAMERA},
    "box": {"size": [0.13, 0.14, 0.04], "color": "red", "mass": 0.1, "friction": 0.4, "camera": DEFAULT_CAMERA},
    "jenga": {"size": [0.072, 0.024, 0.014], "color": "wood", "mass": 0.02, "friction": 0.4, "camera": CLOSE_CAMERA},
}


def default_push(shape, position):
    """A push that reaches the object: the pusher starts below it and drives up (+y) through it."""
    x, y = position
    if shape == "jenga":
        waypoints = [[0.0, x, round(y - 0.10, 5)], [2.0, x, round(y + 0.04, 5)]]
    else:
        waypoints = [[0.0, x, round(y - 0.20, 5)], [2.0, x, y]]
    return {"waypoints": waypoints, "object_velocity": None}


def _finite(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"template {what} must be finite numbers.")
    return float(value)


def _vector(value, count, what):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"template {what} must be a list of {count} numbers.")
    return [_finite(v, what) for v in value]


def colour_rgb(colour):
    """RGB in [0, 1] for a colour name, ``"#rrggbb"`` or a ``[r, g, b]`` list."""
    if isinstance(colour, str):
        if colour.lower() in COLOURS:
            return COLOURS[colour.lower()]
        if re.fullmatch(r"#[0-9a-fA-F]{6}", colour):
            return tuple(round(int(colour[i:i + 2], 16) / 255, 4) for i in (1, 3, 5))
        raise ValueError(f"template color {colour!r} is not one of {sorted(COLOURS)}, '#rrggbb' or [r, g, b].")
    rgb = _vector(colour, 3, "color")
    if not all(0.0 <= v <= 1.0 for v in rgb):
        raise ValueError("template color [r, g, b] values must be between 0 and 1.")
    return tuple(rgb)


def mask_key(colour, name="block"):
    """The RSS ``keys`` entry that masks the object: a colour word where there is one, else a hex colour."""
    if isinstance(colour, str) and colour.lower() in COLOURS:
        return f"{name}={colour.lower()}"
    r, g, b = colour_rgb(colour)
    return f"{name}=#{round(r * 255):02x}{round(g * 255):02x}{round(b * 255):02x}~0.1"


def resolve(spec=None, previous=None):
    """Template data with every default filled in, checked. ``previous`` (a resolved template of the same
    shape) supplies the values a later ``update_scene`` leaves out; ``camera`` merges key by key."""
    spec = {} if spec is None else spec
    if not isinstance(spec, dict):
        raise ValueError("template must be an object such as {'shape': 'tee', 'position': [0, 0], 'yaw': 0}.")
    shape = spec.get("shape") or (previous or {}).get("shape") or "tee"
    if shape == "tower":
        from . import tower

        return tower.resolve(spec, previous)
    unknown = sorted(set(spec) - set(TEMPLATE_KEYS))
    if unknown:
        raise ValueError(f"template has unknown keys {unknown}; use {list(TEMPLATE_KEYS)}.")
    if shape not in SHAPES:
        raise ValueError(f"template shape must be one of {list(SHAPES)}.")
    base = previous if previous and previous.get("shape") == shape else None
    out = {"shape": shape}
    for key, default in (("size", DEFAULTS[shape]["size"]), ("color", DEFAULTS[shape]["color"]),
                         ("mass", DEFAULTS[shape]["mass"]), ("friction", DEFAULTS[shape]["friction"]),
                         ("position", [0.0, 0.0]), ("yaw", 0.0)):
        out[key] = spec[key] if key in spec else (base[key] if base else default)
    out["size"] = _vector(out["size"], len(DEFAULTS[shape]["size"]), "size")
    if not all(0.004 <= v <= 0.5 for v in out["size"]):
        raise ValueError("template size values must be between 0.004 and 0.5 metres.")
    out["position"] = _vector(out["position"], 2, "position")
    if abs(out["position"][0]) > TABLE_LIMIT[0] or abs(out["position"][1]) > TABLE_LIMIT[1]:
        raise ValueError(f"template position must lie on the table (|x| <= {TABLE_LIMIT[0]}, |y| <= {TABLE_LIMIT[1]} m).")
    out["yaw"] = _finite(out["yaw"], "yaw")
    for key in ("mass", "friction"):
        out[key] = _finite(out[key], key)
        if out[key] <= 0:
            raise ValueError(f"template {key} must be positive.")
    colour_rgb(out["color"])
    camera = dict((base or DEFAULTS[shape])["camera"])
    given = spec.get("camera")
    if given is not None:
        if not isinstance(given, dict) or set(given) - {"eye", "target"}:
            raise ValueError("template camera must be {'eye': [x, y, z], 'target': [x, y, z]}.")
        camera.update(given)
    out["camera"] = {"eye": _vector(camera["eye"], 3, "camera eye"), "target": _vector(camera["target"], 3, "camera target")}
    if out["camera"]["eye"] == out["camera"]["target"] or out["camera"]["eye"][2] <= 0:
        raise ValueError("template camera eye must be above the table and differ from its target.")
    if "push" in spec and spec["push"] is not None:
        from .plan import validate_push

        push, _ = validate_push(spec["push"])
        if spec["push"].get("waypoints") is None:
            raise ValueError("template push needs waypoints [[seconds, x, y], ...].")
        out["push"] = push
    elif base and not base.get("push_default"):
        out["push"] = base["push"]
    else:
        out["push"] = default_push(shape, out["position"])
        out["push_default"] = True
    out["mask_key"] = mask_key(out["color"])
    return out


def constants(resolved):
    """The scene file's parameter lines (name -> value) for a resolved template."""
    size, shape = resolved["size"], resolved["shape"]
    values = {"SHAPE": shape, "OBJECT_XY_YAW": (*[round(v, 6) for v in resolved["position"]], round(resolved["yaw"], 6)),
              "OBJECT_COLOUR": tuple(colour_rgb(resolved["color"])), "OBJECT_MASS": resolved["mass"],
              "OBJECT_FRICTION": resolved["friction"], "CAMERA_EYE": tuple(resolved["camera"]["eye"]),
              "CAMERA_LOOK_AT": tuple(resolved["camera"]["target"]), "PUSH": resolved["push"]}
    if shape == "tee":
        bar, stem, thick, height = size
        values.update(TEE_BAR_HALF=(round(bar / 2, 6), round(thick / 2, 6)), TEE_STEM_HALF=(round(thick / 2, 6), round(stem / 2, 6)),
                      OBJECT_HALF_HEIGHT=round(height / 2, 6))
    else:
        values["BOX_HALF"] = tuple(round(v / 2, 6) for v in size)
    return values


def scene_text(shape="tee", spec=None, previous=None):
    """The scene source for a template: defaults for ``shape``, then ``spec`` (a dict) over ``previous``.

    Returns the text only; ``resolve`` returns the data the text was built from.
    """
    return scene_source(resolve(dict(spec or {}, shape=(spec or {}).get("shape", shape)), previous))


def scene_source(resolved):
    if resolved["shape"] == "tower":
        from . import tower

        return tower.scene_source(resolved)
    text = TEMPLATE
    for name, value in constants(resolved).items():
        shown = json.dumps(value) if isinstance(value, str) else repr(value)
        text, count = re.subn(rf"^{name} = [^#\n]*?( *#.*)?$", lambda m, n=name, v=shown: f"{n} = {v}" + (m.group(1) or ""),
                              text, count=1, flags=re.M)
        if count != 1:
            raise ValueError(f"template text has no line for {name}.")
    return text


PARAMETERS = {
    "shape": "tee (default), box or jenga: one object and a pusher. tower: a Jenga tower, with its own parameters",
    "size": ("tee [bar_length, stem_length, thickness, height]; box and jenga [x, y, height]; metres. "
             "jenga default [0.072, 0.024, 0.014]: x is the long axis"),
    "position": "[x, y] of the object on the table, metres",
    "yaw": "radians about +z; tee yaw 0 has the bar far and the stem toward the camera",
    "color": "red, orange, yellow, green, blue, wood, '#rrggbb' or [r, g, b] in 0-1",
    "mass": "kg (default 0.1; jenga 0.02)",
    "friction": "coefficient between object and table (default 0.4)",
    "camera": "{'eye': [x, y, z], 'target': [x, y, z]} metres",
    "push": "{'waypoints': [[seconds, x, y], ...], 'object_velocity': [vx, vy, wz]}: the pusher path in metres",
}
SCENE = scene_text("tee")
