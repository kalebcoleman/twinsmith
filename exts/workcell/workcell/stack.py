"""What sits between the UR5e's tool flange and the fingertips, from the configuration (no Kit imports).

One switch decides the stack: ``bota.mounted``. Off (the arm as it is): flange, Quick Changer, RG2. On: flange,
adapter plate, Bota Rokubi, adapter plate, Quick Changer, RG2. Everything that depends on the stack is derived
here from that switch and the parts' own records, so nothing else is edited to turn the sensor on: the order,
the lengths, the RG2's pose in tool0 (and with it the wrist camera's), the tool centre point, the payload a
pendant needs (mass and centre of gravity in tool0) and the values that are still unmeasured.

Frames: the flange frame has +X out of the flange (the pack's convention); tool0 is that frame turned by UR's
fixed convention, tool0 +Z = flange +X. A part's inlet is the frame that lands on the flange frame, moved along
the tool axis by the lengths of the parts before it.
"""

import math

import numpy as np

TOOL0_IN_FLANGE = (0.5, 0.5, 0.5, 0.5)            # ROS-Industrial: tool0 +Z = flange +X, tool0 +X = flange +Y
PLATES = {"bota_robot_plate": "robot_side_plate", "bota_tool_plate": "tool_side_plate"}   # stack role -> record in bota
ORDER_OFF = ("quick_changer", "rg2")
ORDER_ON = ("bota_robot_plate", "bota", "bota_tool_plate", "quick_changer", "rg2")


def bota_mounted(config):
    value = (config.get("bota") or {}).get("mounted", False)
    if type(value) is not bool:
        raise ValueError("bota.mounted must be true or false")
    return value


def order(config):
    """The stack's parts from the flange outward. It follows ``bota.mounted``; there is no separate order to edit."""
    if "order" in (config.get("stack") or {}):
        raise ValueError("stack.order is not read any more: the stack follows bota.mounted (false: flange, Quick Changer, "
                         "RG2; true: the Bota and its two adapter plates come first). Remove stack.order.")
    return list(ORDER_ON if bota_mounted(config) else ORDER_OFF)


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _three(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3 or not all(
            isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item) for item in value):
        raise ValueError(f"{name} must contain 3 finite numbers")
    return [float(item) for item in value]


def plate(config, role):
    """One adapter plate's record: thickness, diameter and mass, all positive."""
    record = (config.get("bota") or {}).get(PLATES[role])
    if not isinstance(record, dict):
        raise ValueError(f"bota.{PLATES[role]} is missing: the Bota cannot be mounted without its adapter plates")
    return {key: _positive(record.get(key), f"bota.{PLATES[role]}.{key}") for key in ("thickness_m", "diameter_m", "mass_kg")}


def lengths(config):
    """Each part's length along the tool axis, metres; the RG2 ends the stack."""
    result = {"quick_changer": _positive((config.get("quick_changer") or {}).get("standoff_m"), "quick_changer.standoff_m"),
              "rg2": 0.0}
    if bota_mounted(config):
        result["bota"] = _positive(config["bota"].get("length_m"), "bota.length_m")
        for role in PLATES:
            result[role] = plate(config, role)["thickness_m"]
    return result


def masses(config):
    """Each rigid stack part's mass, kilograms, except the RG2 (its links carry their own, ``inertials``)."""
    result = {"quick_changer": _positive((config.get("quick_changer") or {}).get("mass_kg"), "quick_changer.mass_kg")}
    if bota_mounted(config):
        result["bota"] = _positive(config["bota"].get("mass_kg"), "bota.mass_kg")
        for role in PLATES:
            result[role] = plate(config, role)["mass_kg"]
    return result


def axis(config):
    value = np.array(_three((config.get("stack") or {}).get("axis_in_flange", [1.0, 0.0, 0.0]), "stack.axis_in_flange"))
    if not np.isclose(np.linalg.norm(value), 1.0):
        raise ValueError("stack.axis_in_flange must be a unit vector")
    return value


def offsets(config):
    """Where each part's inlet is, metres along the tool axis from the flange plane."""
    sizes, position, result = lengths(config), 0.0, {}
    for role in order(config):
        result[role] = position
        position += sizes[role]
    return result


# ------------------------------------------------------------------ quaternions (w, x, y, z), plain tuples

def _mul(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (lw * rw - lx * rx - ly * ry - lz * rz, lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx, lw * rz + lx * ry - ly * rx + lz * rw)


def _conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _unit(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{name} must contain 4 finite numbers")
    norm = math.sqrt(sum(float(item) ** 2 for item in value))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError(f"{name} must be a non-zero quaternion")
    return tuple(float(item) / norm for item in value)


def _rotate(quaternion, vector):
    w, x, y, z = quaternion
    matrix = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return (matrix @ np.asarray(vector, dtype=float)).tolist()


def rg2_in_tool0(config):
    """The RG2 body frame in tool0 as ``(translation_m, quaternion_wxyz, stack_length_m)``.

    ``robot.build_robot`` puts every stack part's inlet on the flange frame, moved along the tool axis by the
    lengths of the parts before it; ``asset_layout.rg2.mount_pose`` is that inlet in the RG2 body frame.
    ``stack_length_m`` is the flange plane to the RG2's inlet.
    """
    length = offsets(config)["rg2"]
    pose = (((config.get("asset_layout") or {}).get("rg2") or {}).get("mount_pose")) or {}
    inlet_t = _three(pose.get("position_m", [0.0, 0.0, 0.0]), "asset_layout.rg2.mount_pose.position_m")
    inlet_q = _unit(pose.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0]), "asset_layout.rg2.mount_pose.orientation_wxyz")
    body_q = _conjugate(inlet_q)                                         # the RG2 body in its inlet (= flange) frame
    body_t = [-value for value in _rotate(body_q, inlet_t)]
    flange_q = _conjugate(TOOL0_IN_FLANGE)                               # the flange frame in tool0
    in_flange = (axis(config) * length + np.array(body_t)).tolist()
    return _rotate(flange_q, in_flange), _mul(flange_q, body_q), length


def _in_tool0(config, point_in_rg2):
    translation, quaternion, _ = rg2_in_tool0(config)
    return (np.array(translation) + np.array(_rotate(quaternion, point_in_rg2))).tolist()


def _on_axis(config, distance, sideways=(0.0, 0.0)):
    """A point given in a part's inlet frame (x along the tool axis, then the flange's y and z), in tool0."""
    in_flange = axis(config) * distance + np.array([0.0, sideways[0], sideways[1]])
    return _rotate(_conjugate(TOOL0_IN_FLANGE), in_flange.tolist())


def carried_on_rg2(config):
    """What is bolted to the RG2 body and has no rigid body of its own: the printed bracket with its screws and the
    D415. Each is ``{item, mass_kg, cog_in_rg2_m}``; the builder adds them to the RG2 body's mass properties."""
    camera = config.get("wrist_camera") or {}
    items = []
    if camera.get("bracket_mass_kg") is not None:
        mass = _positive(camera["bracket_mass_kg"], "wrist_camera.bracket_mass_kg")
        items.append({"item": "wrist camera bracket and its screws", "mass_kg": mass,
                      "cog_in_rg2_m": _three(camera.get("bracket_cog_in_rg2_m"), "wrist_camera.bracket_cog_in_rg2_m")})
    if camera.get("mass_kg") is not None:
        items.append({"item": "Intel RealSense D415", "mass_kg": _positive(camera["mass_kg"], "wrist_camera.mass_kg"),
                      "cog_in_rg2_m": _three(camera.get("cog_in_rg2_m"), "wrist_camera.cog_in_rg2_m")})
    return items


def rg2_body_with_carried(config, base):
    """The RG2 body's mass properties with what it carries added: ``base`` and the result are records of
    ``inertials`` (mass_kg, com_m, diagonal_inertia_kgm2, principal_axes_wxyz). Each carried item is a point mass at
    its centre of gravity: its own inertia about that point is left out."""
    items = carried_on_rg2(config)
    if not items:
        return dict(base), []
    mass = float(base["mass_kg"])
    centre = np.array(base["com_m"], dtype=float)
    turn = np.array([_rotate(_unit(base["principal_axes_wxyz"], "principal_axes_wxyz"), column) for column in np.eye(3)]).T
    tensor = turn @ np.diag(np.array(base["diagonal_inertia_kgm2"], dtype=float)) @ turn.T
    total = mass + sum(item["mass_kg"] for item in items)
    new = (mass * centre + sum(item["mass_kg"] * np.array(item["cog_in_rg2_m"]) for item in items)) / total

    def about_new(point_mass, at):
        arm = np.asarray(at, dtype=float) - new
        return point_mass * (arm @ arm * np.eye(3) - np.outer(arm, arm))

    tensor = tensor + about_new(mass, centre) + sum(about_new(item["mass_kg"], item["cog_in_rg2_m"]) for item in items)
    moments, vectors = np.linalg.eigh(tensor)
    if np.linalg.det(vectors) < 0:
        vectors[:, 2] = -vectors[:, 2]
    m = vectors
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w > 1e-6:
        quaternion = (w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w))
    else:                                                                # a half turn: take the largest diagonal term
        index = int(np.argmax(np.diag(m)))
        vector = np.zeros(3)
        vector[index] = math.sqrt(max(0.0, (1.0 + m[index, index]) / 2.0))
        for other in range(3):
            if other != index:
                vector[other] = (m[index, other] + m[other, index]) / (4 * vector[index])
        quaternion = (0.0, *vector)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return ({"mass_kg": total, "com_m": new.tolist(), "diagonal_inertia_kgm2": moments.tolist(),
             "principal_axes_wxyz": [value / norm for value in quaternion]}, items)


def payload(config):
    """Everything below the flange as a pendant wants it: the items, the total mass and the centre of gravity in
    tool0 (tool0's origin is the centre of the flange plane, +Z along the tool axis). Fingers at joint zero."""
    places, weights, items = offsets(config), masses(config), []
    changer = config.get("quick_changer") or {}
    cog = _three(changer.get("cog_in_part_m"), "quick_changer.cog_in_part_m")
    items.append({"item": "OnRobot Quick Changer, robot side", "mass_kg": weights["quick_changer"],
                  "cog_in_tool0_m": _on_axis(config, places["quick_changer"] + cog[0], cog[1:])})
    if bota_mounted(config):
        sizes = lengths(config)
        for role, label in (("bota_robot_plate", "adapter plate, robot side of the Bota"), ("bota", "Bota Rokubi"),
                            ("bota_tool_plate", "adapter plate, tool side of the Bota")):
            items.append({"item": label, "mass_kg": weights[role],
                          "cog_in_tool0_m": _on_axis(config, places[role] + sizes[role] / 2.0)})
    gripper = config.get("rg2") or {}
    items.append({"item": "OnRobot RG2 with the Quick Changer's tool side", "mass_kg": _positive(gripper.get("datasheet_mass_kg"), "rg2.datasheet_mass_kg"),
                  "cog_in_tool0_m": _in_tool0(config, _three(gripper.get("cog_in_body_m"), "rg2.cog_in_body_m"))})
    for item in carried_on_rg2(config):
        items.append({"item": item["item"], "mass_kg": item["mass_kg"], "cog_in_tool0_m": _in_tool0(config, item["cog_in_rg2_m"])})
    total = sum(item["mass_kg"] for item in items)
    centre = sum(item["mass_kg"] * np.array(item["cog_in_tool0_m"]) for item in items) / total
    return {"items": items, "mass_kg": total, "cog_in_tool0_m": centre.tolist(), "not_included": ["cables"]}


def unmeasured(config):
    """What a build with this stack rests on that nobody has measured yet."""
    notes = ["wrist camera bracket: weigh it with its screws (the mass is an estimate from the CAD volume)",
             "RG2: weigh it on its Quick Changer tool side (the datasheet's 0.78 kg is taken to include the tool side)",
             "fingertip tape: measure its friction with a tilt test"]
    if bota_mounted(config):
        notes = ["Bota adapter plate, robot side: measure thickness, diameter and mass (placeholder disc)",
                 "Bota adapter plate, tool side: measure thickness, diameter and mass (placeholder disc)",
                 "Bota Rokubi: confirm the model, which face is toward the tool, and where its connector points"] + notes
    return notes


def tool(config):
    """The stack in one record, for the build report: what is mounted, how long it is, where the RG2, the tool
    centre point and the centre of gravity are in tool0, and what the pendant's payload should be."""
    translation, quaternion, length = rg2_in_tool0(config)
    gripper = config.get("rg2") or {}
    record = {"bota_mounted": bota_mounted(config), "order": order(config), "lengths_m": lengths(config),
              "flange_to_rg2_inlet_m": length, "rg2_body_in_tool0": {"translation_m": translation, "quaternion_wxyz": list(quaternion)},
              "unmeasured": unmeasured(config)}
    try:
        record["payload"] = payload(config)
    except ValueError as error:                                          # a config without the payload records: say so
        record["payload"], record["payload_unavailable"] = None, str(error)
    if gripper.get("tcp_in_body_m") is not None:
        record["tcp_in_tool0_m"] = _in_tool0(config, _three(gripper["tcp_in_body_m"], "rg2.tcp_in_body_m"))
        record["tcp"] = "centre of the fingertips' rubber pads with the pads together"
    return record
