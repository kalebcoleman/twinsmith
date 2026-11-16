# twinsmith-workcell-skills

A [Graph-as-Policy](https://github.com/graph-robots/graph-as-policy) skill registry for twinsmith's simulated UR5e
work cell: one tool bundle (`tools/workcell`) and three skills (`skills/`), in the Agent Skills format, made with
`gap registry init` and `gap skills new`.

`puzzle_graph.py` authors the puzzle build as a workflow with `gap.builder` and runs it with `gap.execute`. It is
started by `scripts/puzzle_build.py`, which serves the tools; see [building a puzzle](../workcell/README.md#building-a-puzzle).

    gap check --skills <repo>/exts/workcell/gap --skills <workspace>/tools/open-robot-skills
    gap skills check --skills <repo>/exts/workcell/gap
    cd <repo>/exts/workcell/gap && <workspace>/tools/graph-as-policy/.venv/bin/python -m pytest -q
