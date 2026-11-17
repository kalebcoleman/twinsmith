"""The puzzle build as a Graph-as-Policy workflow: author it with gap.builder, run it with gap.execute.

Run with Graph-as-Policy's own interpreter, by ``scripts/puzzle_build.py``:

    python puzzle_graph.py author <plan.json> <graph folder>      # plan: {"base": name, "order": [{"piece", ...}]}
    python puzzle_graph.py run <graph folder> --trace-dir <folder>

The graph has one subgraph per skill it needs (this registry's three, and two of no registered skill that ask a
vision-language model for a second opinion), and a checkpoint after every placement against the simulator's truth.
It begins with the scan and comes back to it whenever a pick closed on nothing or a look found the table changed;
a router after the scan sends the build to whatever is next on the table as it now stands.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

import gap
from gap.builder import Ref, Subgraph, Workflow
from gap.runtime.verify import Body, World
from gap.skills import resolve_registries
from gap_core.tools import ToolRegistry

REGISTRY = Path(__file__).resolve().parent
RG2_STROKE_M = 0.110
COUNT_PROMPT = ("This is a robot's view straight down onto a table. Count the separate coloured plastic parts lying on "
                "the table, including the large flat base plate. Do not count the robot's own grey gripper fingers at "
                "the top edge. Reply with the number first, then one short line naming the parts.")
JUDGE_PROMPT = ("This is a robot's view straight down onto a table with a square plastic puzzle base. Are {count} "
                "smaller coloured parts seated inside the base, with no loose coloured part lying on the table beside it?")
CHECKPOINT = '''"""Postcondition of `{subgraph}`, against the simulator's truth (workcell.state)."""

from gap.runtime.verify import Checkpoint


def _there(world) -> bool:
    return world.body("{body}").distance_to(world.body("{goal}")) <= {limit}


def _how_far(world) -> dict:
    return {{"distance_mm": round(1000 * float(world.body("{body}").distance_to(world.body("{goal}"))), 2)}}


CHECKPOINTS = [Checkpoint(name="{name}", subgraph="{subgraph}", predicate=_there, diagnostics_fn=_how_far,
                          rationale="{rationale}")]
'''


def stock_scripts():
    """open-robot-skills' verifying-grasps scripts, from the registry Graph-as-Policy resolves."""
    for registry in resolve_registries(None):
        folder = Path(registry.path) / "skills/verifying-grasps/scripts"
        if folder.is_dir():
            return folder
    raise SystemExit("open-robot-skills (skills/verifying-grasps) is not among Graph-as-Policy's registries: see gap registry list")


def assemble(piece):
    """One piece: simulate ahead, grasp with the stock reach and grip checks between the motions, insert, look."""
    sg = Subgraph(name=f"assemble_{piece}", skill="assembling-puzzle-pieces")
    for name in ("lookahead", "approach", "close", "lift", "insert"):
        sg.add_node(name, type="tool", tool=f"workcell.{name}", inputs={"piece": piece})
    sg.add_node("look", type="tool", tool="workcell.rescan", inputs={"piece": piece})
    sg.add_node("ee", type="tool", tool="robot.get_ee_pose")
    sg.add_node("reach", type="script", script="scripts/verify_reach.py",
                inputs={"commanded": Ref("approach.commanded"), "observed": Ref("ee"), "target_obb": Ref("approach.target_obb"),
                        "object_name": piece})
    sg.add_node("grip", type="script", script="scripts/verify_grip.py",
                inputs={"target_obb": Ref("approach.target_obb"), "object_name": piece})
    for outcome in ("placed", "retry", "changed", "aborted", "missed", "empty"):
        sg.add_exit(outcome)
        sg.add_edge(outcome, "END")
    sg.set_on_error("failed")
    for a, b in (("START", "lookahead"), ("approach", "ee"), ("ee", "reach"), ("close", "grip"), ("lift", "insert"), ("insert", "look")):
        sg.add_edge(a, b)
    sg.add_conditional_edges("lookahead", {"go": "approach", "abort": "aborted"}, router_field="verdict")
    sg.add_conditional_edges("reach", {"reached": "close", "missed": "missed"}, router_field="route")
    sg.add_conditional_edges("grip", {"held": "lift", "unknown": "lift", "empty": "empty"}, router_field="route")
    sg.add_conditional_edges("look", {"placed": "placed", "retry": "retry", "changed": "changed"}, router_field="result")
    return sg


def second_opinion(name, prompt_tool, prompt, script=None):
    """A subgraph of no registered skill: a picture from the wrist camera and a question to the vision-language model."""
    sg = Subgraph(name=name)
    sg.add_node("picture", type="tool", tool="workcell.picture", inputs={"label": name})
    sg.add_node("ask", type="tool", tool=prompt_tool, inputs={"prompt": prompt, "image": Ref("picture.image")})
    sg.add_exit("answered")
    sg.set_on_error("unanswered")
    sg.add_edge("START", "picture")
    sg.add_edge("picture", "ask")
    if script:
        sg.add_node("compare", type="script", script=f"scripts/{script}", inputs={"answer": Ref("ask.text"), "found": Ref("in.found")})
        sg.add_input("found", type_name="int")
        sg.add_edge("ask", "compare")
        sg.add_edge("compare", "answered")
    else:
        sg.add_edge("ask", "answered")
    sg.add_edge("answered", "END")
    return sg


def author(plan, folder):
    """Write the workflow, its scripts and its checkpoints to ``folder``."""
    base, pieces = plan["base"], [step["piece"] for step in plan["order"]]
    wf = Workflow(name="puzzle_build", description=f"Assemble the {base} puzzle on the simulated UR5e work cell: scan the table, slide the "
                  "base to the middle, then pick and insert each piece in the order its CAD disassembly gives, reversed; scan again "
                  "whenever a pick fails or the table has changed.")

    scan = Subgraph(name="scan", skill="scanning-the-table")
    scan.add_node("sweep", type="tool", tool="workcell.scan")
    scan.add_exit("scanned")
    scan.set_on_error("failed")
    for a, b in (("START", "sweep"), ("sweep", "scanned"), ("scanned", "END")):
        scan.add_edge(a, b)
    scan.set_outputs(found=Ref("sweep.found"), next=Ref("sweep.next"))

    place = Subgraph(name="place_base", skill="placing-puzzle-base")
    place.add_node("slide", type="tool", tool="workcell.place_base", inputs={"piece": base})
    place.add_node("look", type="tool", tool="workcell.rescan", inputs={"piece": base})
    for outcome in ("placed", "retry"):
        place.add_exit(outcome)
        place.add_edge(outcome, "END")
    place.set_on_error("failed")
    for a, b in (("START", "slide"), ("slide", "look")):
        place.add_edge(a, b)
    place.add_conditional_edges("look", {"placed": "placed", "retry": "retry"}, router_field="result")

    count = second_opinion("count_parts", "vlm.query", COUNT_PROMPT, "cross_check.py")
    judge = second_opinion("judge_assembly", "vlm.query_yes_no", JUDGE_PROMPT.format(count=len(pieces)))
    steps = [assemble(piece) for piece in pieces]
    for sg in (scan, count, place, *steps, judge):
        wf.add_subgraph(sg)
        wf.add_node(sg.name, type="subgraph", ref=sg.name)
    wf.add_node("next_step", type="router", script="scripts/next_step.py", inputs={"next": Ref("scan.next")})
    wf.add_node("done", type="end", status="success")
    wf.add_node("abort", type="end", status="failure", recovery=[{"tool": "workcell.release", "inputs": {}}])
    wf.add_edge("START", "scan")
    wf.add_conditional_edges("scan", {"scanned": "next_step", "failed": "abort"}, router_field="exit")
    wf.add_conditional_edges("next_step", {base: "count_parts", **{piece: f"assemble_{piece}" for piece in pieces}, "done": "judge_assembly"},
                             router_field=None)
    wf.add_conditional_edges("count_parts", {"answered": "place_base", "unanswered": "place_base"}, router_field="exit")
    following = [sg.name for sg in steps] + ["judge_assembly"]
    wf.add_conditional_edges("place_base", {"placed": following[0], "retry": "place_base", "failed": "abort"}, router_field="exit")
    for sg, after in zip(steps, following[1:]):
        wf.add_conditional_edges(sg.name, {"placed": after, "retry": sg.name, **{outcome: "scan" for outcome in ("changed", "missed", "empty")},
                                           "aborted": "abort", "failed": "abort"}, router_field="exit")
    wf.add_conditional_edges("judge_assembly", {"answered": "done", "unanswered": "done"}, router_field="exit")

    folder = Path(folder)
    if folder.exists():
        shutil.rmtree(folder)
    (folder / "scripts").mkdir(parents=True)
    (folder / "checkpoints").mkdir()
    stock = stock_scripts()
    for script in (stock / "verify_reach.py", stock / "verify_grip.py", REGISTRY / "scripts/cross_check.py", REGISTRY / "scripts/next_step.py"):
        shutil.copy(script, folder / "scripts" / script.name)
    made = [("place_base", "base_in_the_middle", base, "middle", 0.010, "The base must stop within 10 mm of the middle of the table.")]
    made += [(f"assemble_{piece}", f"{piece}_seated", piece, f"slot_{piece}", 0.0015,
              f"{piece} must rest within 1.5 mm of its slot in the base, wherever the base now is.") for piece in pieces]
    for subgraph, name, body, goal, limit, rationale in made:
        (folder / "checkpoints" / f"{subgraph}.py").write_text(CHECKPOINT.format(subgraph=subgraph, name=name, body=body, goal=goal,
                                                                                limit=limit, rationale=rationale))
    wf.save(folder / "workflow.json")


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


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    one = commands.add_parser("author")
    one.add_argument("plan", type=Path)
    one.add_argument("graph", type=Path)
    two = commands.add_parser("run")
    two.add_argument("graph", type=Path)
    two.add_argument("--trace-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "author":
        author(json.loads(args.plan.read_text()), args.graph)
        return
    result = gap.execute(args.graph, Cell(), skills=[REGISTRY, *(Path(r.path) for r in resolve_registries(None))],
                         trace_dir=args.trace_dir, checkpoints="warn")
    checks = [{"name": c.name, "subgraph": c.subgraph, "passed": bool(c.passed)} for c in result.checkpoint_results or []]
    print("RESULT: " + json.dumps({"success": bool(result.success), "exit_status": result.exit_status, "error": result.error and str(result.error)[:600],
                                   "trace": str(result.trace_path), "checkpoints": checks, "seconds": round(result.duration_s, 1)}))
    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
