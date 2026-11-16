"""workcell tool bundle: each tool asks a running ``scripts/puzzle_build.py`` to do it on the simulated work cell."""

import json
import os
import socket
from typing import Any, TypedDict

import numpy as np
from gap_core.tools import tool
from gap_core.types import OrientedBoundingBox, Se3Pose

_link = None


def ask(name: str, **inputs: Any) -> dict:
    """One call to the work cell's host: a JSON line each way on 127.0.0.1:$WORKCELL_SKILLS_PORT."""
    global _link
    if _link is None:
        port = os.environ.get("WORKCELL_SKILLS_PORT")
        if not port:
            raise RuntimeError("WORKCELL_SKILLS_PORT is not set: run the graph through scripts/puzzle_build.py")
        _link = socket.create_connection(("127.0.0.1", int(port))).makefile("rw")
    _link.write(json.dumps({"tool": name, "inputs": inputs}) + "\n")
    _link.flush()
    reply = json.loads(_link.readline())
    if not reply["ok"]:
        raise RuntimeError(reply["error"])
    return reply["result"]


class Survey(TypedDict):
    found: int
    pieces: list[str]
    missing: list[str]
    unexplained: int
    position_error_mm: float | None


class Picture(TypedDict):
    image: np.ndarray
    path: str


class Slid(TypedDict):
    result: str
    target: Se3Pose


class Lookahead(TypedDict):
    verdict: str
    tries: list[dict]


class Approach(TypedDict):
    commanded: Se3Pose
    width_m: float
    target_obb: OrientedBoundingBox


class Closed(TypedDict):
    gap_m: float


class Lifted(TypedDict):
    rise_m: float


class Inserted(TypedDict):
    blocked: str | None


class Rescan(TypedDict):
    result: str
    position_error_mm: float | None
    rotation_error_deg: float | None


class Released(TypedDict):
    result: str


class State(TypedDict):
    ee_pose: Se3Pose
    gripper: dict
    bodies: dict
    goals: dict


@tool(name="workcell.survey", summary="Find every piece of the puzzle on the table (wrist camera, SAM 3.1, scan to scene).",
      tags=("perception", "sim_step"))
def survey() -> Survey:
    return ask("survey")


@tool(name="workcell.picture", summary="One colour picture from the wrist camera, where it is now.", tags=("perception",))
def picture(label: str) -> Picture:
    from PIL import Image

    path = ask("picture", label=label)["path"]
    return {"image": np.asarray(Image.open(path).convert("RGB")), "path": path}


@tool(name="workcell.place_base", summary="Grip the puzzle base by its centre island and slide it to the middle of the table.",
      tags=("sim_step",))
def place_base(piece: str) -> Slid:
    return ask("place_base", piece=piece)


@tool(name="workcell.lookahead", summary="Simulate a piece's pick and insertion ahead on Newton and say go or abort.",
      tags=("planning",))
def lookahead(piece: str) -> Lookahead:
    return ask("lookahead", piece=piece)


@tool(name="workcell.approach", summary="Choose a grasp for a piece from its CAD, open the gripper and bring the pads round it.",
      tags=("sim_step",))
def approach(piece: str) -> Approach:
    return ask("approach", piece=piece)


@tool(name="workcell.close", summary="Close the RG2 on the piece at the configured force.", tags=("sim_step",))
def close(piece: str) -> Closed:
    return ask("close", piece=piece)


@tool(name="workcell.lift", summary="Lift the gripped piece straight up.", tags=("sim_step",))
def lift(piece: str) -> Lifted:
    return ask("lift", piece=piece)


@tool(name="workcell.insert", summary="Carry the held piece over its slot, lower it along the slot axis, let go and back away.",
      tags=("sim_step",))
def insert(piece: str) -> Inserted:
    return ask("insert", piece=piece)


@tool(name="workcell.rescan", summary="Look again from the survey pose and say whether a piece is where it should be.",
      tags=("perception", "sim_step"))
def rescan(piece: str) -> Rescan:
    found = ask("rescan", piece=piece)
    return {"result": found["result"], "position_error_mm": found.get("position_error_mm"),
            "rotation_error_deg": found.get("rotation_error_deg")}


@tool(name="workcell.release", summary="Open the gripper.", tags=("sim_step",))
def release() -> Released:
    return ask("release")


@tool(name="workcell.state", summary="The robot's gripper pose and opening, and every piece's true pose with its slot's.")
def state() -> State:
    return ask("state")
