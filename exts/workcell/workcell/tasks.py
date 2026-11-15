"""Task packs: objects loaded on top of the base cell (no Kit imports).

The base cell (``config/lab_hardware.json``) is the robot, its gripper stack, the wrist camera, the bench and the
mounting hardware. Everything a task adds is a task file in ``tasks/``: its own asset pack, its own parameters and
its own BOM rows. A task is of one kind, named by its parameter section: ``jenga`` (a tower, or loose blocks) or
``puzzle`` (base plates and the pieces that go into them). ``workcell__cell(action="quick_start", task="<name>")``
loads one.
"""
import re
from pathlib import Path

TASKS = Path(__file__).resolve().parents[1] / "tasks"
KINDS = ("jenga", "puzzle")


def names():
    return sorted(path.stem for path in TASKS.glob("*.json"))


def _builder(kind):
    from . import jenga, puzzle
    return {"jenga": jenga, "puzzle": puzzle}[kind]


def load(task):
    """A task by name (a file in ``tasks/``) or absolute path: its kind, values, BOM rows and asset paths by role."""
    from . import asset_pack, bom, hardware_config

    if not isinstance(task, str) or not task:
        raise ValueError("task must be a task name or an absolute path to a task file")
    path = Path(task)
    if not path.is_absolute():
        if not re.fullmatch(r"[a-z][a-z0-9_]*", task) or task not in names():
            raise ValueError(f"Unknown task {task!r}; tasks: {names()}")
        path = TASKS / f"{task}.json"
    values, _ = hardware_config.read_values(path)
    kinds = [kind for kind in KINDS if kind in values]
    if len(kinds) != 1 or set(values) != {"pack", "assets", "documentation", kinds[0]} or not values["assets"]:
        raise ValueError(f"A task file needs exactly the sections pack, assets, documentation and one of {list(KINDS)}")
    kind = kinds[0]
    quantities = _builder(kind).quantities(values)
    rows = bom.load_bom(path.parent / values["documentation"]["bom_csv"])
    if {row["asset_id"]: row["quantity"] for row in rows} != quantities or len(rows) != len(quantities):
        raise ValueError(f"The task BOM must be one row per asset the task places, with its number of objects: {quantities}")
    pack = asset_pack.resolve_pack(values)
    return {"name": path.stem, "path": str(path), "kind": kind, "values": values, "rows": rows, "pack": pack,
            "assets": {role: Path(pack.asset(asset_id)) for role, asset_id in values["assets"].items()}}


def merged(config, task):
    """The cell configuration with a loaded task's parameters and asset roles added, for the builders."""
    return {**config, task["kind"]: task["values"][task["kind"]],
            "assets": {**config["assets"], **task["values"]["assets"]}}


def build(stage, assets, config, task):
    """Place a loaded task's objects on the stage; returns the builder's report."""
    if task["kind"] == "jenga":
        return _builder("jenga").build(stage, assets["jenga_block"], config)
    return _builder(task["kind"]).build(stage, assets, config)
