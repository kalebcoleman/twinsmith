"""Task packs: objects loaded on top of the base cell (no Kit imports).

The base cell (``config/lab_hardware.json``) is the robot, its gripper stack, the wrist camera, the bench and the
mounting hardware. Everything a task adds, a Jenga tower or one block to push, is a task file in ``tasks/``: its own
asset pack, its own parameters and its own BOM rows. ``workcell__cell(action="quick_start", task="<name>")`` loads one.
"""
import re
from pathlib import Path

TASKS = Path(__file__).resolve().parents[1] / "tasks"
SECTIONS = {"pack", "assets", "jenga", "documentation"}


def names():
    return sorted(path.stem for path in TASKS.glob("*.json"))


def load(task):
    """A task by name (a file in ``tasks/``) or absolute path: its values, BOM rows and block asset path."""
    from . import asset_pack, bom, hardware_config, jenga

    if not isinstance(task, str) or not task:
        raise ValueError("task must be a task name or an absolute path to a task file")
    path = Path(task)
    if not path.is_absolute():
        if not re.fullmatch(r"[a-z][a-z0-9_]*", task) or task not in names():
            raise ValueError(f"Unknown task {task!r}; tasks: {names()}")
        path = TASKS / f"{task}.json"
    values, _ = hardware_config.read_values(path)
    if set(values) != SECTIONS or set(values["assets"]) != {"jenga_block"}:
        raise ValueError(f"A task file needs exactly the sections {sorted(SECTIONS)} and the asset role jenga_block")
    report = jenga.realize(values)
    rows = bom.load_bom(path.parent / values["documentation"]["bom_csv"])
    asset_id = values["assets"]["jenga_block"]
    if [(row["asset_id"], row["quantity"]) for row in rows] != [(asset_id, len(report["blocks"]))]:
        raise ValueError("The task BOM must be one row for its block asset, with the number of blocks the task places")
    pack = asset_pack.resolve_pack(values)
    return {"name": path.stem, "path": str(path), "values": values, "rows": rows, "pack": pack,
            "asset": Path(pack.asset(asset_id))}


def merged(config, task):
    """The cell configuration with a loaded task's parameters and asset role added, for the builders."""
    return {**config, "jenga": task["values"]["jenga"],
            "assets": {**config["assets"], **task["values"]["assets"]}}
