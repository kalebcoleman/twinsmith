"""Tool ownership and profiles. Pure data and standard-library helpers only."""

from dataclasses import dataclass

from .tool_groups import GROUPS


@dataclass(frozen=True)
class Toolset:
    summary: str
    default: bool = False
    explicit_only: bool = False


TOOLSETS = {
    "core": Toolset("Connection, discovery, help, jobs and toolsets", True),
    "computer": Toolset("Screen, windows, recordings, mouse and keyboard, simulator processes", explicit_only=True),
    "sim": Toolset("Isaac Sim and Isaac Lab: scenes, physics, sensors, code in Kit, ROS 2, the work cell, Lab jobs"),
    "assets": Toolset("Import and manage assets"),
    "cad": Toolset("PCB checks, fab files, renders"),
    "real2sim": Toolset("RoboSimStudio real-to-sim and MuJoCo sysid"),
    "planning": Toolset("Graph-as-Policy skill graphs"),
    "eval": Toolset("VABAR benchmark"),
}

# One line per group: the tools a client is listed when the group is on. Grouped tools name their
# actions in tool_groups.GROUPS. Edit only your group's line, so parallel branches rebase cleanly.
_ROOTS = {
    "core": "status help refresh_tools use_bridge jobs toolsets",
    "computer": "screenshot screen_recording windows join_videos doctor computer simulators",
    "sim": ("sim__quick_start sim__stage sim__prim sim__timeline sim__light sim__material "
            "sim__physics sim__joints sim__arm sim__simulation sim__camera sim__lidar sim__viewport "
            "run_python files reload_tools sim__controller sim__ui sim__logs "
            "sim__ros2_tf sim__graph sim__occupancy_map run_script"),
    "assets": "assets__import_path assets__library assets__pack assets__mesh assets__validate",
    "cad": "hardware",
    "real2sim": "robosim sysid",
    "planning": "gap",
    "eval": "vabar",
}

MEMBERS = {
    name: frozenset(tool for root in roots.split()
                    for tool in (root, *GROUPS.get(root, ("", {}))[1].values()))
    for name, roots in _ROOTS.items()
}
_OWNERS = {tool: name for name, tools in MEMBERS.items() for tool in tools}
if sum(map(len, MEMBERS.values())) != len(_OWNERS):
    raise ValueError("A tool belongs to multiple toolsets")

DEFAULT = frozenset(name for name, spec in TOOLSETS.items() if spec.default)
PROFILES = {"default": DEFAULT,
            "all": frozenset(name for name, spec in TOOLSETS.items() if not spec.explicit_only)}


def owner(name: str, namespace: str | None = None,
          catalog_namespaces=frozenset({"sim", "assets", "workcell"})) -> str:
    """Resolve listed, single-purpose or canonical bridge names; reject unowned tools."""
    name = name.replace(".", "__")
    if name in _OWNERS:
        return _OWNERS[name]
    if name.startswith(("workcell__", "isaaclab__")) or (namespace and namespace not in catalog_namespaces):
        return "sim"  # the work cell, Isaac Lab, and tools of uncatalogued Kit extensions
    raise ValueError(f"Tool {name!r} has no toolset")


STARTUP = "Add +name to tools.toolsets in ~/.config/twinsmith/config.toml or set TWINSMITH_TOOLSETS, then restart the MCP server."
EXPLICIT = ("Explicit-only toolsets ({names}) cannot be enabled at run time: the user adds them at startup "
            "(+name in TWINSMITH_TOOLSETS, tools.toolsets in config.toml, or --toolsets).")
_UNGROUPED = ("isaaclab__checkpoint_register",)


def listed_count(name: str) -> int:
    """How many catalogued tools a client is shown for a toolset (grouped tools count once). In
    ``sim`` the work cell's and Isaac Lab's tools are counted; an uncatalogued Kit extension adds more."""
    roots = set(_ROOTS.get(name, "").split())
    if name == "sim":
        roots |= {group for group in GROUPS if group.startswith(("isaaclab__", "workcell__"))} | set(_UNGROUPED)
    return len(roots)


def explicit_only() -> list:
    return [name for name, spec in TOOLSETS.items() if spec.explicit_only]


def enable_hint(names, runtime_enable: bool = True) -> str:
    """How to turn on the named toolsets, as one sentence the agent can act on."""
    names = sorted(names)
    explicit = [name for name in names if TOOLSETS[name].explicit_only]
    runtime = [name for name in names if name not in explicit]
    parts = []
    if runtime and runtime_enable:
        parts.append('Enable ' + ('it' if len(runtime) == 1 else 'them') + ' with toolsets(action="enable", names=['
                     + ", ".join(f'"{name}"' for name in runtime) + "]) (this server process only), or set it at startup: "
                     + STARTUP[0].lower() + STARTUP[1:])
    elif runtime:
        parts.append("Run-time enabling is switched off; " + STARTUP[0].lower() + STARTUP[1:])
    if explicit:
        parts.append(EXPLICIT.format(names=", ".join(explicit)))
    return " ".join(parts)


def describe(enabled, runtime_enable: bool = True) -> dict:
    """Stable status shared by the host status tool, the toolsets tool and the MCP resource."""
    off = [name for name in TOOLSETS if name not in enabled]
    can = [name for name in off if not TOOLSETS[name].explicit_only] if runtime_enable else []
    enable = ('Call toolsets(action="enable", names=[...]) to turn one on for this server process (the tool list '
              "then changes). " if runtime_enable else "Run-time enabling is off (runtime_enable). ")
    enable += EXPLICIT.format(names=", ".join(explicit_only())) + " " + STARTUP.replace("Add +name", "Otherwise add +name")
    return {"on": [name for name in TOOLSETS if name in enabled],
            "off": {name: TOOLSETS[name].summary for name in off},
            "tools": {name: listed_count(name) for name in TOOLSETS},
            "can_enable": can, "runtime_enable": runtime_enable, "enable": enable}
