"""Typed scene operations: one handler per tool, grouped by family in the *_tools modules.

No app.update(), render(), World.step(), network fetch or arbitrary attribute setter.
"""

import hashlib
import os
from pxr import Usd
from twinsmith.project_assets import refuse
from .joint_tools import JointTools
from .prim_tools import PrimTools
from .sensor_tools import SensorTools
from .arm_tools import ArmTools
from .robot_import import RobotImport
from .step_tool import StepTool


MUTATIONS = {
    "sim.graph.build",
    "sim.prim.delete",
    "sim.prim.clone",
    "sim.prim.create_many",
    "sim.stage.clear_owned",
    "sim.light.create",
    "sim.light.modify",
    "sim.material.create",
    "sim.material.apply",
    "sim.physics.configure",
    "sim.physics.apply",
    "sim.prim.set_attribute",
    "sim.prim.revert",
    "sim.joints.drive",
    "sim.camera.create",
    "sim.lidar.create",
    "sim.controller.create",
    "sim.controller.update",
    "sim.ros2_tf",
}


HANDLERS = {
    "sim.prim.bounds": "tool_prim_bounds",
    "sim.prim.measure": "tool_prim_bounds",
    "sim.camera.list": "tool_camera_list",
    "sim.physics.raycast": "tool_physics_raycast",
    "sim.physics.contacts": "tool_physics_raycast",
    "sim.prim.delete": "tool_prim_delete",
    "sim.stage.clear_owned": "tool_stage_clear_owned",
    "sim.prim.clone": "tool_prim_clone",
    "sim.light.create": "tool_light",
    "sim.light.modify": "tool_light",
    "sim.material.create": "tool_material_create",
    "sim.material.apply": "tool_material_apply",
    "sim.prim.create_many": "tool_prim_create_many",
    "sim.physics.configure": "tool_physics_configure",
    "sim.prim.set_attribute": "tool_prim_set_attribute",
    "sim.physics.apply": "tool_physics_apply",
    "sim.prim.revert": "tool_prim_revert",
    "sim.joints.inspect": "tool_joints_inspect",
    "sim.joints.drive": "tool_joints_drive",
    "sim.joints.command": "tool_joints_command",
    "sim.joints.gains": "tool_joints_gains",
    "sim.joints.move": "tool_joints_move",
    "sim.joints.teleport": "tool_joints_teleport",
    "sim.arm.ik": "tool_arm_ik",
    "sim.arm.fk": "tool_arm_fk",
    "sim.joints.state": "tool_joints_state",
    "sim.simulation.step": "tool_simulation_step",
    "sim.camera.create": "tool_camera_create",
    "sim.lidar.create": "tool_camera_create",
    "sim.camera.capture": "tool_camera_capture",
    "sim.lidar.read": "tool_camera_capture",
    "sim.graph.build": "tool_graph_build",
    "sim.graph.inspect": "tool_graph_inspect",
    "sim.graph.show": "tool_graph_inspect",
    "sim.occupancy_map": "tool_graph_inspect",
    "sim.ros2_tf": "tool_ros2_tf",
    "sim.controller.create": "tool_controller",
    "sim.controller.update": "tool_controller",
}


def physics_manager():
    from .physics import manager

    return manager()


class Advanced(JointTools, PrimTools, SensorTools, ArmTools, RobotImport, StepTool):
    def __init__(self, scene, dispatch):
        self.s = scene
        self.dispatch = dispatch
        self.sensors = {}
        self.controllers = set()
        self.stage_id = None
        self.cleanup_errors = []

    def targetless(self, name, a):
        """Reads with no stage dependency. Deliberately no invalidate(): its sensor cleanup must
        follow a real stage check, and these run without one."""
        if name == "sim.logs.tail":
            from . import logs

            return logs.tail({**a, "scene_events": self.s.public_events()})
        if name == "sim.logs.mark":
            from . import logs

            return logs.mark()
        if name == "sim.logs.summary":
            from . import logs

            return logs.summary(a)
        from . import graph

        contains = a.get("contains")
        limit = a.get("limit", 20)
        if not isinstance(contains, str) or not 1 <= len(contains.strip()) <= 64:
            refuse("INVALID_VALUE", "contains: 1..64 characters.")
        if type(limit) is not int or not 1 <= limit <= 40:
            refuse("LIMIT", "limit must be 1..40.")
        return graph.node_types(contains, limit, a.get("ports"))

    def invalidate(self):
        if self.stage_id != self.s.stage_id:
            self.close_sensors()
            self.controllers.clear()
            self.stage_id = self.s.stage_id

    def tool_graph_build(self, name, stage, a, camera_frame=None):
        s = self.s
        from .graph_build import build

        return build(stage, a, s.timeline.is_stopped())

    def tool_graph_inspect(self, name, stage, a, camera_frame=None):
        s = self.s
        from . import graph

        if name == "sim.graph.inspect":
            limit = a.get("limit", 64)
            if type(limit) is not int or not 1 <= limit <= 128:
                refuse("LIMIT", "limit must be 1..128.")
            return graph.inspect_graph(stage, a["path"], limit)
        if name == "sim.graph.show":
            if a.get("close"):
                if graph._headless():
                    refuse("HEADLESS", "No Action Graph editor in a headless Isaac Sim.")
                return graph.close_editor()
            if not a.get("path"):
                refuse("INVALID_VALUE", "path: the graph prim to show (or close=true).")
            w, h = a.get("width", 0), a.get("height", 0)
            if type(w) is not int or type(h) is not int or (w or h) and not (300 <= w <= 8000 and 200 <= h <= 8000):
                refuse("INVALID_VALUE", "width 300..8000 and height 200..8000, or both 0.")
            if a.get("layout") and not s.timeline.is_stopped():
                refuse("TIMELINE_ACTIVE", "layout=true writes node positions into the stage: stop the timeline "
                       "first, or retry sim__graph__show with the same arguments and layout=false.",
                       retry_with={"session_id": a.get("session_id"), "stage_id": a.get("stage_id"),
                                   "path": a["path"], "width": w, "height": h, "layout": False})
            ready = graph.show_graph_open(stage, a["path"], w, h)
            return {"_finish": "sim.graph.show", "ready": ready}  # extension.py completes it
        return graph.occupancy_map(s, stage, a)

    def tool_ros2_tf(self, name, stage, a, camera_frame=None):
        s = self.s
        from .ros2 import setup

        return setup(s, stage, a)

    def tool_controller(self, name, stage, a, camera_frame=None):
        s = self.s
        if os.getenv("TWINSMITH_ENABLE_SCENE_SCRIPTS") != "1":
            refuse(
                "SCRIPT_DISABLED",
                "User must opt in at Kit launch with TWINSMITH_ENABLE_SCENE_SCRIPTS=1. This executes arbitrary code.",
            )
        code = a["code"]
        p = a["path"]
        if not isinstance(code, str) or len(code) > 16384:
            refuse("LIMIT", "Script is limited to 16384 characters.")
        compile(code, "<trusted MCP controller>", "exec")
        import omni.graph.core as og

        if name.endswith("create"):
            s.new_path(stage, p)
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                og.Controller.edit(
                    dict(graph_path=p, evaluator_name="execution"),
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("Tick", "omni.graph.action.OnPlaybackTick"),
                            ("Script", "omni.graph.scriptnode.ScriptNode"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("Tick.outputs:tick", "Script.inputs:execIn")
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("Script.inputs:usePath", False),
                            ("Script.inputs:script", code),
                        ],
                    },
                )
            self.controllers.add(p)
            s.owned.add(p)
        else:
            if p not in self.controllers:
                refuse("NOT_OWNED", "Not a pack-created controller.")
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                og.Controller.set(
                    og.Controller.attribute(p + "/Script.inputs:script"), code
                )
            node = og.Controller.node(p + "/Script")
            og.Controller.set(node.get_attribute("state:omni_initialized"), False)
        return dict(
            path=p,
            code_sha256=hashlib.sha256(code.encode()).hexdigest(),
            executes="on playback tick",
            sandboxed=False,
        )
    
    def physics_manager(self):
        return physics_manager()
    
    def execute(self, name, stage, a, *, camera_frame=None):
        self.invalidate()
        handler = HANDLERS.get(name)
        if handler is None:
            refuse("UNKNOWN_TOOL", "No handler for " + name)
        return getattr(self, handler)(name, stage, a, camera_frame)
