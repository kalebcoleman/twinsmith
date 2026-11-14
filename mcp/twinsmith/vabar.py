"""``vabar``: the VABAR benchmark's command line, from its own pinned checkout.

VABAR runs task variants and frozen benchmark cells on Newton through RoboSimStudio, solving them with scripted routines
or Graph-as-Policy graphs. Its environment holds RoboSimStudio and Graph-as-Policy at VABAR's own pins; the
Graph-as-Policy and skill-library checkouts beside it are the ones ``gap`` uses. Nothing here imports VABAR.
``warmup`` and ``studio`` are not wrapped: the owner runs the one-time warmup, and studio serves a network viewer.
"""

from __future__ import annotations

import asyncio
import json
import os
import re
from collections import Counter
from pathlib import Path
from typing import Annotated, Literal

from mcp.types import ToolAnnotations
from pydantic import Field

from . import gap, sibling

PIN = "64b594ed2e1ae25e431d235de086c7cfd99d9aaf"       # "VABAR 1.0 release"
REPOSITORY = "https://github.com/BerkeleyAutomation/VABAR"
ACTIONS = ("status", "list", "run", "bench", "results")
SOLVERS = ("routine", "graph", "idle")
_TASK = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]*/[A-Za-z0-9_.-]+")
_CELL = re.compile(r"[a-z0-9_]+/L[0-9]+")
_LABEL = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,79}")
_EPISODES = re.compile(r"[0-9]+(:[0-9]+(:[0-9]+)?|(,[0-9]+)+)?")
RESULT_FILES = 200
RESULT_BYTES = 8 * 1024 * 1024   # read at most this much of one episode_results.jsonl
WALK_ENTRIES = 200_000            # stop walking a run folder after this many entries
REASONS = 3                       # failure reasons listed per results row
# A graph run's tool-call wait when none is given: what upstream's bench cells use (vabar/benchmark.py:68 and :73 at the
# pin), since upstream's own 30 s default fails a cold first run while a kernel compiles.
GRAPH_BUS_TIMEOUT_S = 180
CABLE_SUITES = ("cable_route", "usbc_insert")
CABLE_BUS_TIMEOUT_S = 600


def checkout(workspace=None, root=None) -> sibling.Checkout:
    return sibling.Checkout("vabar", "TWINSMITH_VABAR_ROOT", PIN, REPOSITORY, workspace=workspace, root=root)


def warmup(co: sibling.Checkout) -> dict:
    """What VABAR's warmup leaves in its renderer cache: the compiled-pipelines stamp and one stamp per warmed cell."""
    caches = sorted(co.root.glob(".venv/lib/python3*/site-packages/ovrtx/bin/cache"))
    if not caches:
        return {"ran": False, "note": "No renderer cache in VABAR's environment."}
    cells = sorted(path.name for path in (caches[0] / "vabar_warm").glob("*") if path.is_file())
    return {"ran": (caches[0] / "nv_shadercache/vabar_compiled").is_file(), "warm_cells": cells}


def argv(co: sibling.Checkout, action: str, *, task=None, solve="routine", graph=None, episodes=None, cell=None,
         limit=None, workers=1, out=None, bus_timeout_s=None) -> list[str]:
    """The command line for one action, from validated values only."""
    base = [str(co.bin("python")), "-m", "vabar"]
    if action == "list":
        return [*base, "list"]
    if action == "run":
        return [*base, "run", "--task", task, "--solve", solve, *(["--graph", graph] if graph else []),
                *(["--episodes", episodes] if episodes else []),
                *(["--bus-timeout", str(bus_timeout_s)] if bus_timeout_s else []), "--out", str(out)]
    return [*base, "bench", "--cell", cell, "--solve", solve, *(["--limit", str(limit)] if limit else []),
            "--workers", str(workers), "--out", str(out)]


def _result_files(folder: Path):
    """``episode_results.jsonl`` files under a run folder in sorted order, never following a link and never walking
    more than ``WALK_ENTRIES`` entries."""
    seen = 0
    for top, dirs, files in os.walk(folder, followlinks=False):
        dirs.sort()
        seen += len(dirs) + len(files)
        if seen > WALK_ENTRIES:
            return
        path = Path(top) / "episode_results.jsonl"
        if "episode_results.jsonl" in files and not path.is_symlink() and path.is_file():
            yield path


def results(co: sibling.Checkout, run=None) -> dict:
    """The summary table of one run folder (success per task and solver), or the newest run folders."""
    runs = co.workspace / "runs/vabar"
    if run is None:
        names = sorted((path for path in runs.glob("*") if path.is_dir()), key=lambda path: path.stat().st_mtime)
        return {"runs": [path.name for path in names[-10:][::-1]], "folder": str(runs)}
    folder = runs / sibling.run_name(run, "")
    if folder.is_symlink() or not folder.is_dir():
        raise sibling.Refusal("NOT_FOUND", f"No run folder {folder}.")
    inside = folder.resolve()
    found = (path for path in _result_files(folder) if path.resolve().is_relative_to(inside))
    table, files, cut = {}, [], []
    for path in found:
        if len(files) == RESULT_FILES:
            break
        files.append(path)
        with path.open("rb") as stream:
            data = stream.read(RESULT_BYTES + 1)
        if len(data) > RESULT_BYTES:
            cut.append(str(path.relative_to(folder)))
            data = data[:RESULT_BYTES].rsplit(b"\n", 1)[0]
        for line in data.decode(errors="replace").splitlines():
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if isinstance(row, dict):
                entry = table.setdefault((str(row.get("task")), str(row.get("solution"))), [0, 0, Counter()])
                entry[0] += 1
                entry[1] += bool(row.get("success"))
                if not row.get("success"):
                    entry[2][str(row.get("termination_reason") or row.get("graph_error") or "none recorded")[:300]] += 1
    rows = [{"task": task, "solution": solution, "episodes": count, "successes": wins, "success_rate": round(wins / count, 4),
             **({"failures": [{"reason": reason, "episodes": n} for reason, n in reasons.most_common(REASONS)]}
                if reasons else {})}
            for (task, solution), (count, wins, reasons) in sorted(table.items())]
    return {"run": folder.name, "folder": str(folder), "result_files": [str(path.relative_to(folder)) for path in files],
            "rows": rows, "episodes": sum(row["episodes"] for row in rows),
            "successes": sum(row["successes"] for row in rows),
            **({"truncated": cut, "note": f"Only the first {RESULT_BYTES} bytes of each listed file were read."} if cut else {})}


def call(co: sibling.Checkout, action: str, task=None, solve="routine", graph=None, episodes=None, cell=None, limit=None,
         workers: int = 1, print_only: bool = True, name=None, run=None, bus_timeout_s=None, timeout_s: float = 6 * 3600,
         allow_concurrent_gpu: bool = False) -> dict:
    info = co.info("python")
    try:
        if action not in ACTIONS:
            raise sibling.Refusal("INVALID_VALUE", f"action must be one of {', '.join(ACTIONS)}.")
        if action == "status":
            library = gap.checkout(workspace=co.workspace)
            beside = {key: value for key, value in library.info("gap").items() if key != "setup"}
            return sibling.answer("vabar", action, info, result={
                **{key: info[key] for key in ("checkout", "environment_present", "setup") if key in info},
                "graph_as_policy": beside, "skills": gap.skills_info(library), "warmup": warmup(co),
                "allow_paid": gap.owner_flags()[0], "vlm": gap.vlm_status(),
                "runs": str(co.workspace / "runs/vabar")})
        if action == "results":
            return sibling.answer("vabar", action, info, result=results(co, run))
        if "setup" in info:
            raise sibling.Refusal("NOT_INSTALLED", info["setup"])
        allowed = gap.owner_flags()[0]
        skills_env = {"VABAR_OPEN_ROBOT_SKILLS": str(gap.skills(gap.checkout(workspace=co.workspace)).root)}
        if action == "list":
            env_pass, env = gap.run_env(allowed), skills_env      # lists tasks; asks no model, so no endpoint or key
            done = sibling.run(argv(co, action), cwd=co.root, env_pass=env_pass, env_set=env, timeout_s=300,
                               stdout_chars=sibling.WHOLE)
            tasks = [line.strip() for line in done["stdout"].splitlines() if _TASK.fullmatch(line.strip())]
            if done["exit_code"] != 0:
                return sibling.answer("vabar", action, info, exit_code=done["exit_code"], error="vabar list failed.",
                                      code="UPSTREAM_FAILED", stderr=done["stderr"])
            return sibling.answer("vabar", action, info, exit_code=0, result={"tasks": tasks})
        if solve not in SOLVERS:
            raise sibling.Refusal("INVALID_VALUE", f"solve must be one of {', '.join(SOLVERS)}.")
        if graph is not None and (solve != "graph" or not isinstance(graph, str) or not _LABEL.fullmatch(graph)):
            raise sibling.Refusal("INVALID_VALUE", "graph is a label the task ships, such as gt, and needs solve=graph.")
        if action == "run":
            if not isinstance(task, str) or not _TASK.fullmatch(task):
                raise sibling.Refusal("INVALID_VALUE", "task is required for run: SUITE/VARIANT; vabar(action=\"list\") lists them.")
            if episodes is not None and not (isinstance(episodes, str) and _EPISODES.fullmatch(episodes)):
                raise sibling.Refusal("INVALID_VALUE", "episodes is a range '0:16', a strided range '0:100:4' or a list '0,4,9'.")
        else:
            if not isinstance(cell, str) or not _CELL.fullmatch(cell):
                raise sibling.Refusal("INVALID_VALUE", "cell is required for bench: TASK_CLASS/L<level>, such as tool_hanging/L1.")
            if type(workers) is not int or not 1 <= workers <= 4:
                raise sibling.Refusal("INVALID_VALUE", "workers must be 1 to 4.")
            if limit is not None and (type(limit) is not int or limit < 1):
                raise sibling.Refusal("INVALID_VALUE", "limit must be a positive number of instances.")
        if bus_timeout_s is not None and (action != "run" or type(bus_timeout_s) is not int or not 30 <= bus_timeout_s <= 3600):
            raise sibling.Refusal("INVALID_VALUE", "bus_timeout_s is for run only: 30 to 3600 seconds (bench sets its own).")
        if action == "run" and solve == "graph" and bus_timeout_s is None:
            bus_timeout_s = CABLE_BUS_TIMEOUT_S if task.split("/")[0] in CABLE_SUITES else GRAPH_BUS_TIMEOUT_S
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not 0 < timeout_s <= 7 * 86400:
            raise sibling.Refusal("INVALID_VALUE", "timeout_s must be more than 0 and at most 7 days in seconds.")
        folder = co.workspace / "runs/vabar" / sibling.run_name(name, action)
        command = argv(co, action, task=task, solve=solve, graph=graph, episodes=episodes, cell=cell, limit=limit,
                       workers=workers, out=folder, bus_timeout_s=bus_timeout_s)
        if action == "bench" and print_only:
            # Upstream's --print resolves the cell's jobs and exits before any warmup, MPS daemon or folder; the GPU is
            # hidden from it anyway. It asks no model, so it gets no key, endpoint or token.
            if folder.exists():
                raise sibling.Refusal("ALREADY_EXISTS", f"{folder} exists; choose another name.")
            done = sibling.run([*command, "--print"], cwd=co.root, env_pass=gap.PLAIN_ENV,
                               env_set={**skills_env, "CUDA_VISIBLE_DEVICES": ""}, timeout_s=120, stdout_chars=sibling.WHOLE)
            summary = [line for line in done["stderr"].splitlines() if line.strip()]
            if done["exit_code"] != 0:
                reason = "did not finish in 120 s." if done["timed_out"] else summary[-1] if summary else "gave no reason."
                what = "refused the cell or config" if done["exit_code"] == 2 else "failed"
                return sibling.answer("vabar", action, info, exit_code=done["exit_code"], code="UPSTREAM_FAILED",
                                      error=f"vabar bench --print {what}: {reason}", stderr=done["stderr"])
            return sibling.answer("vabar", action, info, exit_code=0, result={
                "planned": command, "summary": summary, "jobs": [line for line in done["stdout"].splitlines() if line.strip()],
                "note": "Nothing ran. Call again with print_only=false (and the same name) to run it as a job; it uses the GPU."})
        local = gap.owner_vlm()
        env_pass, env, env_warnings = gap.model_env(allowed, local)
        env.update(skills_env)
        sibling.new_run_folder(folder)
        started = sibling.start_job(command, task=f"vabar-{folder.name}", kind=f"vabar-{action}", cwd=folder,
                                    env_pass=env_pass, env_set={**env, "PYTHONUNBUFFERED": "1"}, timeout_s=timeout_s,
                                    uses_gpu=True, allow_concurrent_gpu=allow_concurrent_gpu)
        if not started["ok"]:
            folder.rmdir()
            return sibling.answer("vabar", action, info, error=started["message"], code=started["code"], hint=started["hint"])
        warnings = env_warnings + (gap.no_model_warning(allowed, local) if solve == "graph" else [])
        return sibling.answer("vabar", action, info, warnings=warnings, result={
            "job_id": started["job_id"], "log_dir": started["log_dir"], "run": folder.name, "command": command,
            "next": f'jobs(action="status", job_id="{started["job_id"]}"), then vabar(action="results", run="{folder.name}")'})
    except sibling.Refusal as refusal:
        return sibling.answer("vabar", action, info, error=str(refusal), code=refusal.code)


def register(server, *, workspace=None, root=None):
    co = checkout(workspace, root)

    @server.tool(name="vabar", annotations=ToolAnnotations(
        readOnlyHint=False, destructiveHint=False, idempotentHint=False, openWorldHint=True,
    ))
    async def vabar(
        action: Annotated[Literal["status", "list", "run", "bench", "results"], Field(description="Action to perform.")],
        task: Annotated[str | None, Field(description="run (required): SUITE/VARIANT from list, such as tool_hanging/wrenches_level1.")] = None,
        solve: Annotated[Literal["routine", "graph", "idle"], Field(description="run/bench: the scripted routine, a Graph-as-Policy graph, or idle.")] = "routine",
        graph: Annotated[str | None, Field(description="run with solve=graph: a graph label the task ships, such as gt; omit for its default.")] = None,
        episodes: Annotated[str | None, Field(description="run: '0:16', '0:100:4' or '0,4,9'; default the first episode.")] = None,
        cell: Annotated[str | None, Field(description="bench (required): benchmark cell TASK_CLASS/L<level>, such as tool_hanging/L1.")] = None,
        limit: Annotated[int | None, Field(description="bench: run only the first N instances.", ge=1)] = None,
        workers: Annotated[int, Field(description="bench: worker processes, 1 to 4.", ge=1, le=4)] = 1,
        print_only: Annotated[bool, Field(description="bench: return upstream's resolved jobs (--print, CPU, writes nothing) without running.")] = True,
        name: Annotated[str | None, Field(description="run/bench: run folder name under <workspace>/runs/vabar; default action and time.")] = None,
        bus_timeout_s: Annotated[int | None, Field(description="run: seconds a graph's tool call may wait (--bus-timeout); default for solve=graph 180, cable suites 600, as bench does.", ge=30, le=3600)] = None,
        run: Annotated[str | None, Field(description="results: run folder name; omit to list the newest runs.")] = None,
        timeout_s: Annotated[float, Field(description="run/bench: job time limit in seconds.", gt=0, le=604800)] = 21600,
        allow_concurrent_gpu: Annotated[bool, Field(description="run/bench: knowingly start beside another GPU job or a running Isaac Sim.")] = False,
    ) -> str:
        """VABAR benchmark from its own pinned checkout: status (pins, warmup), list (tasks), run (a task's episodes, as a
        job; uses the GPU), bench (a frozen cell; print_only=true returns upstream's plan, false runs it as a job),
        results (success table and failure reasons of a run folder). Follow jobs with jobs(action="status").
        """
        arguments = dict(task=task, solve=solve, graph=graph, episodes=episodes, cell=cell, limit=limit, workers=workers,
                         print_only=print_only, name=name, run=run, bus_timeout_s=bus_timeout_s, timeout_s=timeout_s,
                         allow_concurrent_gpu=allow_concurrent_gpu)
        return json.dumps(await asyncio.to_thread(call, co, action, **arguments))

    return co
