"""Which tools a client is shown as one grouped tool with an ``action`` argument.

A group lists the single-purpose tools behind it. Those keep their names and stay callable, so a
script or an older prompt that names one still works, but they are left out of the tool list: a
client reads compact grouped tools. Tools named in no group are always listed.

Pure data and stdlib only: the server builds the grouped tools from this (grouped_tools.py), and
the documentation checks read it without the MCP SDK.
"""

import re
from typing import Dict, List, Optional, Set

# group name -> (one-line summary, {action: tool it runs})
GROUPS: Dict[str, tuple] = {
    "sim__stage": ("The open scene: identify it, list it, open, save, or clear what these tools made.", {
        "info": "sim__stage__info", "tree": "sim__stage__tree", "open": "sim__stage__open",
        "save": "sim__stage__save", "clear_owned": "sim__stage__clear_owned"}),
    "sim__prim": ("Prims: read, find, measure, create, move, edit, copy, delete and undo.", {
        "get": "sim__prim__get", "find": "sim__prim__find", "bounds": "sim__prim__bounds",
        "measure": "sim__prim__measure", "create": "sim__prim__create", "create_many": "sim__prim__create_many",
        "set_transform": "sim__prim__set_transform", "set_attribute": "sim__prim__set_attribute",
        "clone": "sim__prim__clone", "delete": "sim__prim__delete", "revert": "sim__prim__revert"}),
    "sim__physics": ("Physics: inspect, body state, contacts, raycasts, scene settings, colliders, mass and pushes.", {
        "inspect": "sim__physics__inspect", "body_state": "sim__body__state", "contacts": "sim__physics__contacts",
        "raycast": "sim__physics__raycast", "configure": "sim__physics__configure", "apply": "sim__physics__apply"}),
    "sim__joints": ("A robot's joints: read them, command targets, set drives and gains, move and teleport.", {
        "inspect": "sim__joints__inspect", "state": "sim__joints__state", "command": "sim__joints__command",
        "move": "sim__joints__move", "drive": "sim__joints__drive", "gains": "sim__joints__gains",
        "teleport": "sim__joints__teleport"}),
    "sim__arm": ("An arm's end effector: inverse and forward kinematics.", {
        "ik": "sim__arm__ik", "fk": "sim__arm__fk"}),
    "sim__simulation": ("Advance physics yourself: exact steps, a run with your own controller in the loop, "
                        "or a saved trajectory replayed to video.", {
        "step": "sim__simulation__step", "run": "sim__simulation__run", "replay": "sim__trajectory__replay"}),
    "sim__camera": ("Cameras: list, create, capture a picture.", {
        "list": "sim__camera__list", "create": "sim__camera__create", "capture": "sim__camera__capture"}),
    "sim__lidar": ("LiDAR: create one, read a scan.", {
        "create": "sim__lidar__create", "read": "sim__lidar__read"}),
    "sim__light": ("Lights: create or change one.", {
        "create": "sim__light__create", "modify": "sim__light__modify"}),
    "sim__material": ("Materials: create one, bind it to geometry.", {
        "create": "sim__material__create", "apply": "sim__material__apply"}),
    "sim__graph": ("OmniGraph action graphs: node types, build, inspect, show in the editor.", {
        "node_types": "sim__graph__node_types", "build": "sim__graph__build", "inspect": "sim__graph__inspect",
        "show": "sim__graph__show"}),
    "sim__controller": ("OPT-IN CODE EXECUTION: a script-node controller that ticks while the timeline plays.", {
        "create": "sim__controller__create", "update": "sim__controller__update"}),
    "sim__ui": ("Isaac Sim's own windows: find a widget, act on it.", {
        "find": "sim__ui__find", "act": "sim__ui__act"}),
    "sim__logs": ("The simulator's log: tail it, mark a place, summarise.", {
        "tail": "sim__logs__tail", "mark": "sim__logs__mark", "summary": "sim__logs__summary"}),
    "assets__library": ("Your asset folders: approved roots, files in one, a search, one file's details.", {
        "roots": "assets__roots", "scan": "assets__scan", "search": "assets__search", "info": "assets__info"}),
    "assets__pack": ("Named asset packs: list, check, dependencies, adopt a folder, pin, build.", {
        "list": "assets__packs", "doctor": "assets__pack_doctor", "dependencies": "assets__dependencies",
        "adopt": "assets__pack_adopt", "pin": "assets__pack_pin", "build": "assets__pack_build"}),
    "assets__mesh": ("Mesh preparation: inspect, decimate render meshes, collision shapes, split render and collision.", {
        "inspect": "assets__mesh_inspect", "decimate": "assets__mesh_decimate", "hulls": "assets__mesh_hulls",
        "split": "assets__mesh_split"}),
    "workcell__cell": ("Build and inspect the pack-backed workcell.", {
        "quick_start": "workcell__workcell_quick_start", "status": "workcell__workcell_status",
        "shutdown": "workcell__workcell_shutdown"}),
    "workcell__robot": ("Inspect the workcell robot and read its wrist force/torque sensor.", {
        "status": "workcell__robot_status", "wrench": "workcell__robot_wrench"}),
    "workcell__perception": ("Inspect workcell cameras.", {"status": "workcell__perception_status"}),
    "workcell__jenga": ("Build and inspect the workcell Jenga tower.", {
        "build": "workcell__jenga_build", "status": "workcell__jenga_status"}),
    "files": ("Files on the simulator's machine: write, read, check size and hash.", {
        "put": "put_file", "get": "get_file", "stat": "stat_file"}),
    "jobs": ("Every job, simulator or process: list, status (with wait_s), log tail, cancel.", {
        "status": "job_status", "cancel": "job_cancel", "list": "isaaclab__runs_list", "tail": "isaaclab__runs_tail",
        "lab_status": "isaaclab__runs_status"}),
    "isaaclab__launch": ("Start an Isaac Lab job (uses the GPU) and get its job_id: train, evaluate, play, record or check a task.", {
        "train": "isaaclab__train", "eval": "isaaclab__eval", "play": "isaaclab__play",
        "record": "isaaclab__record", "env_check": "isaaclab__env_check"}),
    "isaaclab__inspect": ("Before launching: the Isaac Lab install, its tasks, GPU room, and checkpoints.", {
        "install": "isaaclab__install_info", "envs": "isaaclab__envs_list", "preflight": "isaaclab__preflight",
        "checkpoints": "isaaclab__checkpoints_list"}),
    "isaaclab__metrics": ("Training results: a summary of scalars or evaluation files, or one curve's last points.", {
        "summary": "isaaclab__metrics_summary", "curve": "isaaclab__tensorboard_tail"}),
    "screen_recording": ("Record the desktop or a window to MP4: start, stop, status.", {
        "start": "start_screen_recording", "stop": "stop_screen_recording", "status": "screen_recording_status"}),
    "windows": ("Desktop windows: info, list, raise, move, tile.", {
        "info": "screen_info", "list": "list_windows", "raise": "raise_window", "move": "move_window", "tile": "tile_windows"}),
    "computer": ("Mouse and keyboard into one X11 window, named by its id and PID (computer group).", {
        "click": "computer_click", "drag": "computer_drag", "scroll": "computer_scroll", "type": "computer_type",
        "key": "computer_key", "batch": "computer_batch"}),
    "simulators": ("Simulators this server manages (computer group): launch, list, status, stop.", {
        "launch": "launch_simulator", "list": "list_simulators", "status": "simulator_status",
        "stop": "stop_simulator"}),
}

# Tools whose answer, or whose error, quotes the caller's own data, code or a log: returned exactly
# as it is. Every other answer and error has tool names put in their listed form (listed_form).
VERBATIM = frozenset({"run_python", "run_script", "get_file", "put_file", "stat_file", "job_status", "files", "jobs",
                      "isaaclab__runs_tail", "isaaclab__runs_status", "sim__logs", "sim__logs__tail",
                      "sim__logs__summary", "sim__logs__mark"})


def members() -> Dict[str, tuple]:
    """tool name -> (group, action) for every grouped tool."""
    return {tool: (group, action) for group, (_, actions) in GROUPS.items() for action, tool in actions.items()}


def hidden(registered: Set[str]) -> Set[str]:
    """The registered tools a client is not shown by default."""
    return set(members()) & set(registered)


def listed(registered: Set[str]) -> List[str]:
    """What a client is shown: groups with at least one registered member, and every ungrouped tool."""
    registered = set(registered)
    groups = {group for group, (_, actions) in GROUPS.items() if registered & set(actions.values())}
    return sorted((registered - hidden(registered)) | groups)


def replacement(tool: str) -> Optional[str]:
    """How to say the same thing with a listed tool, for the documentation."""
    found = members().get(tool)
    return f'{found[0]}(action="{found[1]}")' if found else None


def _spoken(tool: str) -> str:
    """A hidden tool as plain text names it: ``sim__prim action=create``."""
    found = members().get(tool)
    if found:
        return f"{found[0]} action={found[1]}"
    return tool


def _names(tools) -> str:
    return "|".join(sorted(map(re.escape, tools), key=len, reverse=True))


# A name with a namespace (sim__prim__get) is only ever a tool. A bare one (get_file, job_status,
# computer_type) is also an ordinary word for a file, a prim or a JSON key, so it is rewritten only
# where prose names a tool: after a space, a backtick or a bracket, and not inside a path or a key.
_NAMESPACED = re.compile(r"(?<![A-Za-z0-9_/.\-])(" + _names(t for t in list(members()) if "__" in t)
                         + r")(?![A-Za-z0-9_*-])")
_BARE_NAMES = _names(t for t in list(members()) if "__" not in t)
# ... or where a JSON value starts with a call, as in "next": "job_status(job_id=...)".
_BARE = re.compile(r"(?:(?:(?<=[ `(])|^)(" + _BARE_NAMES + r")(?=[ `().,;:]|$)(?!\.[A-Za-z0-9])|(?<=\")(" + _BARE_NAMES + r")(?=\())")


def listed_form(text: str) -> str:
    """``text`` with every single-purpose tool name replaced by the listed tool and action that
    runs it, so instructions, descriptions and error messages name what the client can call. A
    family such as ``workcell__quick_start*`` is left as written, and so is a name that is part of
    a path, a file name or a JSON key. Uses no quote characters, so JSON text stays valid."""
    spoken = lambda match: _spoken(match.group(1) or match.group(2))  # noqa: E731
    return _BARE.sub(spoken, _NAMESPACED.sub(spoken, text))
