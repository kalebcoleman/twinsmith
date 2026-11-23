"""Run a Graph-as-Policy workflow on the simulated work cell: the connector ``gap.execute`` needs, and a runner.

The workflow itself belongs to a project: it authors a graph whose nodes call this registry's ``workcell.*`` tools
(and the project's own skills), and starts a build host that serves those tools on ``$WORKCELL_SKILLS_PORT``. This
module is what is the same for every such graph. Run it with Graph-as-Policy's own interpreter:

    python workcell_graph.py run <graph folder> --trace-dir <folder> [--skills <another registry> ...]

or import it (``sys.path`` + ``import workcell_graph``) and call ``execute``. The registries searched are this one
(the ``workcell`` tool bundle), the ones given, and every registry Graph-as-Policy resolves (open-robot-skills).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import gap
from gap.runtime.verify import Body, World
from gap.skills import resolve_registries
from gap_core.tools import ToolRegistry

REGISTRY = Path(__file__).resolve().parent
SCRIPTS = REGISTRY / "scripts"                  # graph scripts any work-cell graph may copy in: next_step.py, cross_check.py
RG2_STROKE_M = 0.110


def stock_scripts():
    """open-robot-skills' verifying-grasps scripts, from the registry Graph-as-Policy resolves."""
    for registry in resolve_registries(None):
        folder = Path(registry.path) / "skills/verifying-grasps/scripts"
        if folder.is_dir():
            return folder
    raise SystemExit("open-robot-skills (skills/verifying-grasps) is not among Graph-as-Policy's registries: see gap registry list")


class Cell:
    """What gap.execute needs of a connector, for the simulated work cell: the robot reads that open-robot-skills'
    verifying-grasps scripts use, and the simulator's truth for the checkpoints. Every read is ``workcell.state``."""

    def __init__(self):
        self.tool_registry = ToolRegistry()
        add = self.tool_registry.register_callable
        add("robot.get_ee_pose", self.get_ee_pose, summary="Get the end-effector pose in world frame.")
        add("robot.get_gripper", self.get_gripper, summary="Get the gripper open fraction (0 closed, 1 open).")
        add("robot.describe_gripper", self.describe_gripper, summary="The gripper's stroke and the gap its open fraction stands for.")

    def state(self) -> dict:
        return self.tool_registry.invoke("workcell.state")

    def get_ee_pose(self, arm_id: int = 0) -> dict:
        return self.state()["ee_pose"]

    def get_gripper(self, arm_id: int = 0) -> dict:
        return self.state()["gripper"]

    def describe_gripper(self, arm_id: int = 0) -> dict:
        return {"name": "OnRobot RG2", "span_m": RG2_STROKE_M, "width_fit": {"at_open_m": RG2_STROKE_M, "at_closed_m": 0.0}}

    def world_snapshot(self) -> World:
        state = self.state()
        return World(env_id=0, bodies={name: body(name, at) for name, at in {**state["bodies"], **state["goals"]}.items()})


def body(name, at):
    """A pose (4 x 4, world from the body) as a checkpoint body. Only its position and rotation are true: the
    checkpoints here measure distances between origins."""
    at = np.array(at, dtype=float)
    w = 0.5 * np.sqrt(max(0.0, 1.0 + np.trace(at[:3, :3])))
    xyz = np.array([at[2, 1] - at[1, 2], at[0, 2] - at[2, 0], at[1, 0] - at[0, 1]]) / (4 * w) if w > 1e-6 else np.array([1.0, 0.0, 0.0])
    return Body(name=name, position=at[:3, 3], quaternion_wxyz=np.array([w, *xyz]), aabb_lower=at[:3, 3], aabb_upper=at[:3, 3],
                linear_velocity=np.zeros(3), angular_velocity=np.zeros(3), contacts=frozenset())


def execute(graph, trace_dir, skills=()):
    """Run the workflow in ``graph`` on the work cell through ``Cell``. Prints one ``RESULT: {...}`` line (success,
    exit status, error, trace, checkpoints, seconds) and returns the process's exit code: 0 for a run that succeeded."""
    registries = [REGISTRY, *(Path(path) for path in skills), *(Path(r.path) for r in resolve_registries(None))]
    result = gap.execute(Path(graph), Cell(), skills=registries, trace_dir=Path(trace_dir), checkpoints="warn")
    checks = [{"name": c.name, "subgraph": c.subgraph, "passed": bool(c.passed)} for c in result.checkpoint_results or []]
    print("RESULT: " + json.dumps({"success": bool(result.success), "exit_status": result.exit_status, "error": result.error and str(result.error)[:600],
                                   "trace": str(result.trace_path), "checkpoints": checks, "seconds": round(result.duration_s, 1)}))
    return 0 if result.success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("graph", type=Path)
    run.add_argument("--trace-dir", type=Path, required=True)
    run.add_argument("--skills", type=Path, action="append", default=[], help="another skill registry the graph's subgraphs name")
    args = parser.parse_args()
    sys.exit(execute(args.graph, args.trace_dir, args.skills))


if __name__ == "__main__":
    main()
