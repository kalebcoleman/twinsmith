"""Pack-backed UR5e workcell bridge extension."""

import os

import omni.ext
from twinsmith_bridge import ToolDefinition, get_mcp_bridge, knobs

from .bridge_binding import OWNER_ID, attach
from .commands import Commands

MCP_TOOL_REGISTRY = {
    "workcell_quick_start": {
        "description": "Build a stopped workcell from an external asset pack, on PhysX or Newton (Newton uses the config's per-engine drives; mirror_real needs PhysX). Refuses unsaved stage changes unless explicitly allowed. Returns a summary; action=status returns the full report.",
        "parameters": {
            "config_path": {"type": "string", "description": "Optional hardware configuration path."},
            "mode": {"type": "string", "enum": ["physics", "mirror_real"], "default": "physics", "description": "Physics mode, or zero-gravity mirror mode."},
            "discard_changes": {"type": "boolean", "default": False, "description": "Allow replacement of an unsaved stage."},
            "task": {"type": "string", "description": "Task objects to load on top of the base cell: a task name (jenga_tower: the 54-block tower; push_block: one loose Jenga block) or an absolute task file. Omitted: the base cell only, with no objects."},
            "environment": {"type": "string", "enum": ["none", "simple_room"], "default": "none",
                            "description": "A room around the cell: simple_room references NVIDIA's Simple Room from the Isaac Sim assets server (needs network), floor level with the bench feet."},
        },
    },
    "workcell_status": {"description": "Read the current workcell build report.", "parameters": {}},
    "workcell_shutdown": {"description": "Release workcell state without clearing the stage.", "parameters": {}},
    "robot_status": {"description": "Read robot build details from the workcell report.", "parameters": {}},
    "robot_wrench": {"description": "Read the simulated Bota F/T sensor (its mount joint's reaction, sensor frame, N and N m) on PhysX or Newton, with the engine's mass of everything below the sensor and its weight. Needs an initialized, paused or playing simulation; not gravity-compensated or hardware-calibrated.", "parameters": {}},
    "perception_status": {"description": "Read camera build details; this does not capture sensor data.", "parameters": {}},
    "jenga_status": {"description": "Read Jenga build details from the workcell report.", "parameters": {}},
    "jenga_build": {
        "description": "Replace the Jenga tower in the current workcell on a stopped timeline using configured or overridden variation. Resamples blocks; not a physics-state reset.",
        "parameters": {
            "profile": {"type": "string", "description": "Optional name in jenga.variation_profiles; omitted uses the configured variation_profile."},
            "seed": {"type": "integer", "description": "Optional integer seed; omitted uses the configured variation_seed. A selected profile requires a seed."},
        },
    },
}


class WorkcellExtension(Commands, omni.ext.IExt):
    def on_startup(self, ext_id):
        from .scene import Workcell

        self.workcell = Workcell()
        self._mcp = None
        try:
            bridge = get_mcp_bridge()
            requested_port = os.getenv("TWINSMITH_PORT")
            configured_port = knobs.PORT.read()
            attach(bridge, MCP_TOOL_REGISTRY, self, ToolDefinition,
                   requested_port=int(requested_port) if requested_port is not None else None,
                   configured_port=int(configured_port) if configured_port is not None else None,
                   host=knobs.BIND.read() or "localhost")
            self._mcp = bridge
        except Exception:
            self.workcell.shutdown()
            raise

    def on_shutdown(self):
        bridge = getattr(self, "_mcp", None)
        self._mcp = None
        if bridge is not None:
            bridge.unregister_owner(OWNER_ID, on_drained=self.workcell.shutdown)
        elif getattr(self, "workcell", None) is not None:
            self.workcell.shutdown()
