"""Portable, provenance-bearing work-cell configuration (no Kit imports)."""
import json
import math
from pathlib import Path

try:  # package import inside Kit
    from . import stack as stack
except ImportError:  # loaded by file path in host tests
    import importlib.util as _importlib_util
    _spec = _importlib_util.spec_from_file_location("workcell_stack", Path(__file__).with_name("stack.py"))
    stack = _importlib_util.module_from_spec(_spec)
    _spec.loader.exec_module(stack)

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/lab_hardware.json"
STATUSES = {"confirmed", "unconfirmed"}


def read_values(path):
    """A provenance file flattened to its values: every leaf is {value, status, source}. Returns (values, raw)."""
    raw = json.loads(Path(path).read_text())
    values = {}
    for section, fields in raw.items():
        if not isinstance(fields, dict):
            continue
        values[section] = {}
        for key, record in fields.items():
            if not isinstance(record, dict) or set(record) != {"value", "status", "source"} or record["status"] not in STATUSES or not record["source"]:
                raise ValueError(f"Missing provenance for {section}.{key}")
            values[section][key] = record["value"]
    return values, raw


def load_config(path=None):
    """The base cell: robot, gripper stack, wrist camera, bench and mounting hardware. No task objects."""
    values, raw = read_values(path or DEFAULT_CONFIG)
    if "jenga" in values or "jenga_block" in values.get("assets", {}):
        raise ValueError("The cell configuration must not hold task objects; the Jenga tower is a task (tasks/jenga_tower.json)")
    stack.order(values)                    # the stack follows bota.mounted; a leftover stack.order is refused
    stack.lengths(values)
    stack.masses(values)
    if values["gravity"]["mode"] not in ("physics", "mirror_real"):
        raise ValueError("gravity.mode must be physics or mirror_real")
    value = values["world"]["bench_height_m"]
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError("world.bench_height_m must be finite and positive")
    if values["overhead_mount"]["type"] != "gantry":
        raise ValueError("Only the gantry mount is implemented; clamp is a BOM alternative")
    return values, raw


def parse_width_command(command, max_width_m=0.110):
    """Max's string grammar -> metres; query returns None. Reject out-of-range input."""
    command = command.lower()
    if command == "query":
        return None
    if command == "open":
        return max_width_m
    if command == "close":
        return 0.0
    if not command.isascii() or not command.isdigit():
        raise ValueError("Expected open, close, query, or integer width in 0.1 mm")
    width = int(command) * 0.0001
    if not 0 <= width <= max_width_m:
        raise ValueError("RG2 width outside 0..110 mm")
    return width


def camera_intrinsics(resolution, fov):
    width, height = resolution
    if min(width, height) <= 0 or not all(0 < x < 180 for x in fov):
        raise ValueError("Invalid camera resolution/FOV")
    return (width / (2 * math.tan(math.radians(fov[0] / 2))),
            height / (2 * math.tan(math.radians(fov[1] / 2))), width / 2, height / 2)
