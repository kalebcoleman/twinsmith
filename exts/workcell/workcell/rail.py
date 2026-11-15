"""The linear axis the robot stands on: what the configuration says about it, checked (no Kit imports).

The real cell's UR5e is bolted to the carriage of a Vention linear axis, so the base is a seventh axis. With
``rail.moving`` true the builder makes it one: a fixed rail link is the articulation's root, a prismatic joint
(``rail.joint_name``) carries the carriage, and the carriage carries the robot's base. With it false (or without a
``rail`` section) the base is fixed, as before.

Units. The simulated joint is in metres, zero at the cell's built pose (``rail.default_position_m``; the carriage
where the bench's CAD has it, the middle of the travel), positive along ``rail.axis_in_world``. The real axis is
commanded in millimetres, absolute from its home position (``interfaces.rail_position_command``). Where the home
position is on the rail, and which way positive runs, are not known from any source: ``rail.real_zero_at_joint_m``
and ``rail.real_positive_direction`` hold an assumption, marked unconfirmed, and ``to_real_mm`` / ``from_real_mm``
use it.
"""

import math

DRIVE_KEYS = {"stiffness_n_per_m", "damping_ns_per_m"}


def _number(value, name, positive=True):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{name} must be a finite{' positive' if positive else ''} number")
    return float(value)


def moving(config):
    """Whether the cell has the rail as a moving axis."""
    section = config.get("rail")
    if section is None:
        return False
    value = section.get("moving", False)
    if type(value) is not bool:
        raise ValueError("rail.moving must be true or false")
    return value


def check(config):
    """The rail's record, validated: joint name, axis, limits, default, speed and acceleration, force, carriage mass
    and both engines' drives. Raises ValueError for anything missing or inconsistent."""
    section = config.get("rail") or {}
    name = section.get("joint_name")
    if not isinstance(name, str) or not name.isidentifier():
        raise ValueError("rail.joint_name must be a joint name")
    axis = section.get("axis_in_world")
    if axis not in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]):
        raise ValueError("rail.axis_in_world must be one of the world's axes: [1, 0, 0], [0, 1, 0] or [0, 0, 1]")
    limits = section.get("limits_m")
    if not isinstance(limits, (list, tuple)) or len(limits) != 2:
        raise ValueError("rail.limits_m must be [lower, upper] in metres")
    lower, upper = _number(limits[0], "rail.limits_m[0]", False), _number(limits[1], "rail.limits_m[1]", False)
    default = _number(section.get("default_position_m"), "rail.default_position_m", False)
    if not lower < upper or not lower <= default <= upper:
        raise ValueError("rail.limits_m must be increasing and hold rail.default_position_m")
    if default != 0.0:
        raise ValueError("rail.default_position_m must be 0: the joint's zero is the pose the cell is built in")
    travel = _number(section.get("travel_m"), "rail.travel_m")
    if upper - lower > travel + 1e-9:
        raise ValueError("rail.limits_m is wider than rail.travel_m, the travel the actuator has")
    direction = section.get("real_positive_direction")
    if direction not in (1, -1):
        raise ValueError("rail.real_positive_direction must be 1 or -1")
    record = {"joint_name": name, "axis_in_world": [float(v) for v in axis], "limits_m": [lower, upper], "default_position_m": default,
              "travel_m": travel, "speed_m_s": _number(section.get("speed_m_s"), "rail.speed_m_s"),
              "acceleration_m_s2": _number(section.get("acceleration_m_s2"), "rail.acceleration_m_s2"),
              "max_force_n": _number(section.get("max_force_n"), "rail.max_force_n"),
              "carriage_mass_kg": _number(section.get("carriage_mass_kg"), "rail.carriage_mass_kg"),
              "rail_mass_kg": _number(section.get("rail_mass_kg"), "rail.rail_mass_kg"),
              "real_zero_at_joint_m": _number(section.get("real_zero_at_joint_m"), "rail.real_zero_at_joint_m", False),
              "real_positive_direction": int(direction), "drive": {}}
    drive = section.get("drive")
    if not isinstance(drive, dict) or set(drive) != {"physx", "newton"}:
        raise ValueError("rail.drive must hold the position drive for both engines: physx and newton")
    for engine, values in drive.items():
        keys = DRIVE_KEYS | ({"armature_kg"} if engine == "newton" else set())
        if not isinstance(values, dict) or set(values) != keys:
            raise ValueError(f"rail.drive.{engine} needs exactly {sorted(keys)}")
        record["drive"][engine] = {key: _number(values[key], f"rail.drive.{engine}.{key}") for key in sorted(keys)}
    return record


def stability(record, rate_hz):
    """``dt^2 k / m`` for the Newton drive with ``m`` the armature alone, against the explicit-spring limit."""
    dt, drive = 1.0 / rate_hz, record["drive"]["newton"]
    return dict(ratio=round(dt * dt * drive["stiffness_n_per_m"] / drive["armature_kg"], 4),
                limit=round(4 + 2 * dt * drive["damping_ns_per_m"] / drive["armature_kg"], 4))


def to_real_mm(record, joint_m):
    """The real axis position (millimetres, absolute from home) for a simulated joint value, under the configured
    assumption about where home is and which way positive runs."""
    return (float(joint_m) - record["real_zero_at_joint_m"]) * record["real_positive_direction"] * 1000.0


def from_real_mm(record, position_mm):
    """The simulated joint value (metres) for a real axis position in millimetres; refuses one outside the limits."""
    value = _number(position_mm, "position_mm", False) / 1000.0 * record["real_positive_direction"] + record["real_zero_at_joint_m"]
    lower, upper = record["limits_m"]
    if not lower - 1e-9 <= value <= upper + 1e-9:
        low, high = sorted((to_real_mm(record, lower), to_real_mm(record, upper)))
        raise ValueError(f"rail position {position_mm:g} mm is outside the simulated travel [{low:g}, {high:g}] mm")
    return min(max(value, lower), upper)


def caller_view(record):
    """What a caller needs to command the rail: the joint, its units and limits, and the real interface's units."""
    lower, upper = record["limits_m"]
    return {"joint": record["joint_name"], "units": "metres, zero at the built pose, positive along the world axis below",
            "axis_in_world": record["axis_in_world"], "limits_m": [lower, upper], "default_position_m": record["default_position_m"],
            "speed_m_s": record["speed_m_s"], "acceleration_m_s2": record["acceleration_m_s2"], "max_force_n": record["max_force_n"],
            "real_axis": {"units": "millimetres, absolute from the axis's home position",
                          "at_default_mm": to_real_mm(record, record["default_position_m"]),
                          "limits_mm": sorted((to_real_mm(record, lower), to_real_mm(record, upper))),
                          "mm_per_joint_m": 1000.0 * record["real_positive_direction"],
                          "status": "unconfirmed: where home is on the rail and which way positive runs are assumed, not known"}}
