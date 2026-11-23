# twinsmith-workcell-skills

A [Graph-as-Policy](https://github.com/graph-robots/graph-as-policy) skill registry for twinsmith's simulated UR5e
work cell: one tool bundle (`tools/workcell`), in the Agent Skills format, made with `gap registry init`. It holds
no task's skills: a project that builds something on the cell keeps its own skills in a registry of its own and
passes both (`--skills`).

| File | What it is |
|---|---|
| `tools/workcell` | Twelve `workcell.*` tools (scan, survey, picture, place_base, lookahead, approach, close, lift, insert, rescan, release, state). Each one is a call over `127.0.0.1:$WORKCELL_SKILLS_PORT` to a build host that a project runs beside the simulator bridge; the bundle does nothing itself and cannot reach a real robot |
| `workcell_graph.py` | What every work-cell graph needs: `Cell`, the connector `gap.execute` takes (the robot reads of open-robot-skills' `verifying-grasps`, and the simulator's truth for checkpoints, both through `workcell.state`), `stock_scripts()`, and `execute(graph, trace_dir, skills=[...])`, the runner. `python workcell_graph.py run <graph> --trace-dir <folder> [--skills <registry>]` |
| `scripts/next_step.py`, `scripts/cross_check.py` | Graph scripts a project's graph may copy in: a router on the scan's `next`, and a check of a vision-language model's part count against the scan's |

A build host is a project's own driver: it builds the cell through the simulator bridge, serves these tools one call
at a time and runs the project's graph with `workcell_graph.execute`. The scan-to-scene matcher, the assembly
geometry, the arm session and the step viewer it uses are twinsmith's (`scripts/`), imported by path from a checkout.

    gap check --skills <repo>/exts/workcell/gap --skills <workspace>/tools/open-robot-skills
    cd <repo>/exts/workcell/gap && <workspace>/tools/graph-as-policy/.venv/bin/python -m pytest -q
