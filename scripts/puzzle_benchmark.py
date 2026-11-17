#!/usr/bin/env python3
"""The puzzle build's benchmark: the same named cases, built by any checkout's ``puzzle_build.py``, with one row of
numbers per case. Simulation only.

The cases are in ``puzzle_benchmark.json`` and are frozen: a case is a kind of layout, a seed, and perhaps a
disturbance or other settings. They were fixed before anything was tuned on them and are not changed to make a
number better.

    python3 scripts/puzzle_benchmark.py cases [--group base]
    python3 scripts/puzzle_benchmark.py lay                      # once, before the freeze: lay out the tables that are missing
    python3 scripts/puzzle_benchmark.py run --port <port> --out <folder> --label <name> [--checkout <repo>]
        [--group base | --cases <name> ...] --colours <colours.json> [--scan-cycle <mv_cycle.sh>]
        [--sam-client <sam3_client.py>] [--bridge <the puzzle project's ros2 folder>]
    python3 scripts/puzzle_benchmark.py summary --out <folder> [--main <label> --ours <label> --reference <label>]

``run`` builds each case with ``--checkout``'s build (this repository's unless given) on the simulator at ``--port``,
which must have been started from the checkout whose physics is meant: the build replaces its stage. Every build is
started through this file's ``child`` mode, which gives the build the case's layout in place of the build's own
scatter; that is how a checkout that knows nothing of this benchmark (main) is given the same tables. The tables
themselves are numbers in ``puzzle_benchmark.json`` (``tables``: where every piece lies, x, y and yaw): they were laid
out once by ``lay``, inside the region and outside the hidden patches the benchmark names, and no build lays one out
again, so every checkout gets the same table for the same case whatever its survey or its scatter. A disturbance
is made from outside too, by ``puzzle_disturb.inject``, when the build prints the caption the case names. A build that
ends is judged by the dry-run bridge when ``--bridge`` is given (``puzzle_disturb.replay``). One case leaves
``<out>/<label>/<case>/``: the build's own folder (``metrics.json``, ``run_log.json``, ``build.mp4``, ``viewer/``),
``build.log``, ``layout.json`` and ``row.json``.

``summary`` gathers every ``row.json`` under ``<out>`` into ``<out>/metrics.json`` and writes ``<out>/summary.md``:
the labels side by side, and the pass bar.

``--sam-client`` is for a machine that keeps one SAM 3.1 worker up: a file with ``track(clip, name, prompts) ->
(report, how)``. The build's own looks then go to it and not to a SAM 3.1 job of the build's own.
"""

import argparse
import importlib.util
import json
import math
import os
import statistics
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).with_name("puzzle_benchmark.json")
BASE_SIDE_M, PIECE_TRIES = (0.24, 0.32), 300      # how far to a side of its target the base starts (the scatter's own)


# ---- the layouts ----------------------------------------------------------------------------------------------

def lay_out(assembly, puzzle_scan, pack, base, target, rng, spec, region, hidden=(), tries=20000):
    """One puzzle's pieces lying flat on the table in the way a kind of layout says, the base off to one side of
    ``target`` and both the target and the way there left free, as ``puzzle_assembly.scatter`` leaves them. ``spec``
    is the kind's entry of the benchmark: ``side`` (-1 the base starts on the left, +1 on the right; either without
    it), ``base_side_m`` (how far to that side of its target it starts; 0.24 to 0.32 m without it), ``pieces_x`` and ``pieces_y`` (the band the loose pieces' middles lie in; ``band_for`` names the pieces it
    holds for when not all of them), ``cluster_m`` (every loose piece's middle within this of one random point),
    ``gap_m`` (the least room between pieces, 35 mm without it). ``region``
    is ((x low, y low), (x high, y high)) of the table that every piece lies inside, and ``hidden`` the rectangles of
    it, given the same way, that no piece may touch. Returns {piece: world pose} with the base first. ``assembly`` and
    ``puzzle_scan`` are the build's own modules, so the footprints are the build's."""
    low, high = np.array(region[0]), np.array(region[1])
    gap = spec.get("gap_m", 0.035)
    band = (np.array([spec.get("pieces_x", (low[0], high[0]))[0], spec.get("pieces_y", (low[1], high[1]))[0]]),
            np.array([spec.get("pieces_x", (low[0], high[0]))[1], spec.get("pieces_y", (low[1], high[1]))[1]]))
    hidden = [np.array([a, [b[0], a[1]], b, [a[0], b[1]]]) for a, b in hidden]

    def lying(name, xy):
        rest = pack.shape(name).rests[0]
        return assembly.pose(puzzle_scan.yaw_matrix(rng.uniform(0, 2 * math.pi)) @ rest.turn, [xy[0], xy[1], rest.drop])

    def inside(name, at):
        corners, grown = assembly.footprint(pack, name, at), assembly.footprint(pack, name, at, gap)
        return (corners > low).all() and (corners < high).all() and all(assembly._apart(grown, part) for part in hidden)

    for _ in range(tries):
        side = spec.get("side") or rng.choice([-1, 1])
        start = lying(base, [target[0, 3] + side * rng.uniform(*spec.get("base_side_m", BASE_SIDE_M)), rng.uniform(low[1] + 0.13, high[1] - 0.13)])
        if not inside(base, start):
            continue
        poses, keep_free = {base: start}, [assembly.footprint(pack, base, at, 0.03) for at in assembly.slide_path(start, target)]
        middle = rng.uniform(*band)
        names = list(assembly.slot_poses(pack, base))
        for name in sorted(names, key=lambda name: name not in spec.get("band_for", names)):       # the pieces a band holds for first: they have least room
            band_low, band_high = band if name in spec.get("band_for", [name]) else (low, high)
            for _ in range(PIECE_TRIES):
                reach = spec.get("cluster_m")
                xy = middle + rng.uniform(-reach, reach, 2) if reach else rng.uniform(band_low, band_high)
                if not ((xy >= band_low).all() and (xy <= band_high).all()):
                    continue
                at = lying(name, xy)
                grown = assembly.footprint(pack, name, at, gap)
                if inside(name, at) and all(assembly._apart(grown, kept) for kept in keep_free) \
                        and all(assembly._apart(grown, assembly.footprint(pack, other, there)) for other, there in poses.items()):
                    poses[name] = at
                    break
            else:
                break
        if len(poses) == len(names) + 1:
            return {name: poses[name] for name in (base, *names)}
    raise ValueError(f"no layout of this kind was found: {spec}")


def with_others(assembly, puzzle_scan, pack, base, target, rng, poses, spec, hidden=()):
    """A laid table with another puzzle's pieces on it too: ``spec["also"]`` names them, and each lies flat anywhere
    in ``spec["also_region"]`` (given like ``region``; they are seen and not picked, so it may reach beyond what the
    arm reaches), outside ``hidden``, ``spec["gap_m"]`` (35 mm without it) from every piece, and clear of the base's
    way to ``target``. The table's own pieces stay where they are. The region of ``lay_out`` has no room for a
    second puzzle beside the first's own pieces and the base's way: of 600 spots drawn for one more bar, none was free."""
    low, high, gap = np.array(spec["also_region"][0]), np.array(spec["also_region"][1]), spec.get("gap_m", 0.035)
    hidden = [np.array([a, [b[0], a[1]], b, [a[0], b[1]]]) for a, b in hidden]
    keep_free = [assembly.footprint(pack, base, at, 0.03) for at in assembly.slide_path(poses[base], target)]
    out = dict(poses)
    for name in spec["also"]:
        rest = pack.shape(name).rests[0]
        for _ in range(20 * PIECE_TRIES):
            xy = rng.uniform(low, high)
            at = assembly.pose(puzzle_scan.yaw_matrix(rng.uniform(0, 2 * math.pi)) @ rest.turn, [xy[0], xy[1], rest.drop])
            corners, grown = assembly.footprint(pack, name, at), assembly.footprint(pack, name, at, gap)
            if (corners > low).all() and (corners < high).all() and all(assembly._apart(grown, part) for part in (*hidden, *keep_free)) \
                    and all(assembly._apart(grown, assembly.footprint(pack, other, there)) for other, there in out.items()):
                out[name] = at
                break
        else:
            raise ValueError(f"no room on the table for {name} of {spec['also']}")
    return out


def table_of(assembly, puzzle_scan, pack, poses):
    """A layout as the benchmark keeps it: {piece: [x m, y m, yaw degrees]}; every piece lies on its first rest."""
    out = {}
    for name, at in poses.items():
        turn = at[:3, :3] @ pack.shape(name).rests[0].turn.T
        out[name] = [round(float(at[0, 3]), 6), round(float(at[1, 3]), 6), round(math.degrees(math.atan2(turn[1, 0], turn[0, 0])), 4)]
    return out


def poses_of(assembly, puzzle_scan, pack, table):
    """A kept layout as poses: {piece: 4 x 4}, in the table's order (the base first)."""
    out = {}
    for name, (x, y, yaw) in table.items():
        rest = pack.shape(name).rests[0]
        out[name] = assembly.pose(puzzle_scan.yaw_matrix(math.radians(yaw)) @ rest.turn, [x, y, rest.drop])
    return out


# ---- the cases ------------------------------------------------------------------------------------------------

def cases(config):
    """Every case of the benchmark, in order: ``[{"name", "group", "layout", "seed", "disturbance", "settings",
    "table"}]``. ``settings`` are changes to the build's own settings (``puzzle_build.json``) for that case; ``table``
    names its layout in the benchmark's ``tables`` (a disturbed case has the table of the base case it disturbs)."""
    out = []
    for layout, row in config["base"].items():
        out += [{"name": f"base/{layout}/{seed}", "group": "base", "layout": layout, "seed": seed} for seed in row["seeds"]]
    described = lambda rows: ((name, row) for name, row in rows.items() if name != "what")
    for kind, row in described(config["disturbance"]):
        made = {key: row[key] for key in ("kind", "on_caption", "mm", "deg", "piece") if key in row}
        out += [{"name": f"disturbance/{kind}/{layout}_{seed}", "group": "disturbance", "layout": layout, "seed": seed, "disturbance": made}
                for layout, seed in row["on"]]
    for name, row in described(config["stretch"]):
        out += [{"name": f"stretch/{name}/{step['name']}", "group": f"stretch/{name}", "layout": step.get("layout", row.get("layout")),
                 "seed": step.get("seed", row.get("seed")), "settings": step.get("settings")} for step in row["cases"]]
    return [{"disturbance": None, "settings": None, **case,
             "table": case["name"] if case["group"].startswith("stretch") else f"{case['layout']}/{case['seed']}"} for case in out]


def lay(args):
    """Lay out every table the cases name and the benchmark does not have yet, and write them into it. A table that is
    there is never laid out again: that is what keeps a frozen case the same."""
    sys.path.insert(0, str(ROOT / "mcp"))
    import puzzle_assembly
    import puzzle_scan
    import workcell_session

    config = json.loads(args.config.read_text())
    settings = json.loads((ROOT / "scripts/puzzle_build.json").read_text())
    pack, tables = puzzle_scan.load_pack(settings["pack"]), config.setdefault("tables", {})
    for case in cases(config):
        if case["table"] in tables or (args.cases and case["table"] not in args.cases):
            continue
        mine = merged(settings, case["settings"])
        target = workcell_session.pose(puzzle_scan.yaw_matrix(math.radians(mine["base_target"]["yaw_deg"])),
                                       [*mine["base_target"]["position_m"], pack.shape(mine["base"]).rests[0].drop])
        started, spec = time.time(), config["layouts"][case["layout"]]
        try:
            if "on" in spec:                                # another table of the benchmark, with another puzzle's pieces added
                poses = with_others(puzzle_assembly, puzzle_scan, pack, mine["base"], target, np.random.default_rng(case["seed"]),
                                    poses_of(puzzle_assembly, puzzle_scan, pack, tables[spec["on"]]), spec, config["hidden"])
            else:
                poses = lay_out(puzzle_assembly, puzzle_scan, pack, mine["base"], target, np.random.default_rng(case["seed"]),
                                spec, config["region"], config["hidden"])
        except (ValueError, KeyError) as error:
            print(f"{case['table']}: {error}", flush=True)
            continue
        tables[case["table"]] = table_of(puzzle_assembly, puzzle_scan, pack, poses)
        args.config.write_text(json.dumps(config, indent=1) + "\n")
        print(f"{case['table']}: laid out in {time.time() - started:.0f} s", flush=True)


def merged(settings, changes):
    """Settings with changes laid over them, dictionaries within dictionaries merged."""
    out = dict(settings)
    for key, value in (changes or {}).items():
        out[key] = merged(out[key], value) if isinstance(value, dict) and isinstance(out.get(key), dict) else value
    return out


# ---- one build, started as a child ----------------------------------------------------------------------------

def child(argv):
    """Run a checkout's ``puzzle_build.py`` with a layout of this benchmark in place of its own scatter:
    ``child <checkout> <the case's table as JSON> <layout.json to write> <sam client file or -> <the build's arguments>``."""
    checkout, table, layout_file, sam_client = Path(argv[0]), json.loads(argv[1]), Path(argv[2]), argv[3]
    sys.path[:] = [str(checkout / "scripts"), str(checkout / "mcp"), *(entry for entry in sys.path if Path(entry or ".").resolve() != Path(__file__).resolve().parent)]
    import puzzle_assembly
    import puzzle_build
    import puzzle_scan

    def laid_out(pack, base, target, rng, **given):         # in place of the build's scatter: the case's table, whatever the dice
        poses = poses_of(puzzle_assembly, puzzle_scan, pack, table)
        if next(iter(poses)) != base:
            raise SystemExit(f"the case's table is of {next(iter(poses))}, and the build's settings name {base}")
        layout_file.write_text(json.dumps({name: at.tolist() for name, at in poses.items()}) + "\n")
        return poses

    puzzle_assembly.scatter = laid_out
    served = []
    if sam_client != "-":                                   # the build's own looks to the worker that is up, not to a job of its own
        found = importlib.util.spec_from_file_location("sam3_client", sam_client)
        client = importlib.util.module_from_spec(found)
        found.loader.exec_module(client)

        class Served:
            def __init__(self, job_status=None):
                self.report = None

            def call(self, action, **inputs):
                if action == "report":
                    return self.report
                self.report, how = client.track(Path(inputs["source"]), inputs["name"], inputs["prompts"])
                served.append({"name": inputs["name"], **how})
                return {"ok": True, "job_id": "served"}

        puzzle_build.Sam3 = Served
        puzzle_build.jobs = types.SimpleNamespace(runs_status=lambda job_id: {"state": "finished"})
    sys.argv = [str(checkout / "scripts/puzzle_build.py"), *argv[4:]]
    try:
        puzzle_build.main()
    finally:
        if served:
            layout_file.with_name("sam3_served.json").write_text(json.dumps(served, indent=1) + "\n")


def build(case, config, args, out):
    """Build one case as a child process, making its disturbance when its caption is printed. Returns
    ``{"exit", "wall_s", "timed_out", "injected"}``."""
    checkout = Path(args.checkout).resolve()
    source = (checkout / "scripts/puzzle_build.py").read_text()
    settings = out / "puzzle_build.json"
    settings.write_text(json.dumps(merged(json.loads((checkout / "scripts/puzzle_build.json").read_text()), case["settings"]), indent=1) + "\n")
    if case["table"] not in config.get("tables", {}):
        raise SystemExit(f"{case['name']} has no table yet: run the lay action before the benchmark is frozen")
    command = [sys.executable, "-u", str(Path(__file__).resolve()), "child", str(checkout), json.dumps(config["tables"][case["table"]]),
               str(out / "layout.json"), str(args.sam_client or "-"),
               "--port", str(args.port), "--out", str(out), "--seed", str(case["seed"]), "--config", str(settings), "--mutate-stage",
               "--discard-changes", "--allow-concurrent-gpu", "--table-camera", "--perception", config["perception"], "--colours", str(args.colours)]
    if "--scan-cycle" in source:                            # a build with the scan step needs the cycle that combines its views
        if not args.scan_cycle:
            raise SystemExit(f"{checkout}'s build has a scan step: pass --scan-cycle")
        command += ["--scan-cycle", str(args.scan_cycle)]
    made, started, disturbance = {"injected": None, "timed_out": False}, time.time(), case["disturbance"]
    with (out / "build.log").open("w") as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(checkout))

        def too_long():
            made["timed_out"] = True
            process.kill()

        timer = threading.Timer(config["timeout_s"], too_long)
        timer.start()
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if disturbance and made["injected"] is None and disturbance["on_caption"] in line:
                    import puzzle_disturb                   # this benchmark's, whatever checkout is being built
                    try:
                        made["injected"] = {**puzzle_disturb.inject(args.port, disturbance, json.loads((out / "layout.json").read_text())),
                                            "on": line.strip()}
                    except Exception as error:  # noqa: BLE001 - a disturbance that could not be made is a result of the case, and is recorded
                        made["injected"] = {"failed": f"{type(error).__name__}: {error}", "on": line.strip()}
                    log.write(f"  [benchmark] disturbance: {json.dumps(made['injected'])}\n")
            code = process.wait()
        finally:
            timer.cancel()
    return {"exit": code, "wall_s": round(time.time() - started, 1), **made}


# ---- one row --------------------------------------------------------------------------------------------------

def stopped(metrics, log):
    """Where and why a build that did not seat everything stopped, in the build's own words; None when it did not stop."""
    if "stopped" in metrics:                                # a build that says it itself
        return metrics["stopped"]
    if metrics["completed"] and metrics["graph"]["run"].get("success"):
        return None
    for piece, row in metrics["pieces"].items():
        looks = row.get("lookahead") or []
        if looks and looks[-1]["verdict"] == "abort" and not row.get("attempts"):
            last = looks[-1]["tries"][-1]
            return f"lookahead said abort before {piece}: " + (last.get("error") or last.get("misfit") or (
                f"another piece moved {last['others_moved_mm']} mm" if last.get("seated") else f"the piece ended {last['position_error_mm']} mm off"))
    refused = [step for step in log["steps"] if step.get("error") and step.get("parent") is None]
    if refused:
        return f"{refused[-1]['kind']} of {refused[-1]['piece']} refused: {refused[-1]['error'][:200]}"
    return f"the graph ended in {metrics['graph']['run'].get('exit_status')}"


def row_of(out, case, label, ran, replayed=None):
    """A case's row from the folder its build left. A build that left no ``metrics.json`` (it crashed, or was
    stopped at the time limit) seats nothing as far as the benchmark can say."""
    row = {"case": case["name"], "group": case["group"], "layout": case["layout"], "seed": case["seed"], "label": label,
           "disturbance": case["disturbance"], "injected": ran.get("injected"), "exit": ran["exit"], "process_wall_s": ran["wall_s"],
           "folder": str(out), "bridge": replayed}
    file = out / "metrics.json"
    if not file.is_file():
        last = (out / "build.log").read_text().strip().splitlines()[-1:] if (out / "build.log").is_file() else []
        return {**row, "pieces_total": None, "pieces_seated": 0, "seat_error_mm": None, "wall_s": ran["wall_s"], "rescans": None,
                "failed_picks": None, "graph_success": False, "recovered": False if case["disturbance"] else None,
                "stopped": "the time limit" if ran.get("timed_out") else f"no metrics.json; the build's last line: {' '.join(last)[:300]}"}
    metrics = json.loads(file.read_text())
    log = json.loads((out / "run_log.json").read_text()) if (out / "run_log.json").is_file() else {"steps": []}
    pieces = metrics["pieces"]
    seated = [name for name, piece in pieces.items() if piece.get("final_seated")]
    errors = [pieces[name]["final_position_error_mm"] for name in seated]
    success = bool(metrics.get("graph", {}).get("run", {}).get("success"))
    checks = metrics.get("base_checks")
    row.update(
        pieces_total=len(pieces), pieces_seated=len(seated),
        seat_error_mm={"mean": round(statistics.mean(errors), 2), "max": round(max(errors), 2)} if errors else None,
        final_error_mm={name: piece.get("final_position_error_mm") for name, piece in pieces.items()},
        wall_s=metrics["wall_seconds"], simulated_s=metrics["simulated_seconds"],
        rescans=max(0, sum(step["kind"] == "scan" and step.get("parent") is None for step in log["steps"]) - 1),
        looks=len(log.get("looks", [])),
        failed_picks=sum(piece.get("attempts", 0) for piece in pieces.values()) - len(seated),
        graph_success=success, stopped=stopped(metrics, log),
        recovered=bool(len(seated) == len(pieces) and success) if case["disturbance"] else None,
        correction=metrics.get("correction"), friction_in_the_solver=metrics.get("cell", {}).get("friction_in_the_solver"),
        base_checks={key: len(value) if isinstance(value, list) else value for key, value in checks.items()} if isinstance(checks, dict) else None,
        video=metrics.get("video"), viewer=str(out / "viewer/index.html") if (out / "viewer/index.html").is_file() else None)
    return row


def run(args):
    config = json.loads(args.config.read_text())
    wanted = [case for case in cases(config) if (case["name"] in args.cases if args.cases else case["group"] == args.group or args.group == "all")]
    if not wanted:
        raise SystemExit("no case of that name or group")
    for case in wanted:
        out = Path(args.out).resolve() / args.label / case["name"]
        if (out / "row.json").is_file():
            print(f"{case['name']}: already built", flush=True)
            continue
        out.mkdir(parents=True, exist_ok=True)
        ran = build(case, config, args, out)
        replayed = None
        if args.bridge and (out / "run_log.json").is_file():
            import puzzle_disturb
            replayed = puzzle_disturb.replay(out, args.bridge)
        row = row_of(out, case, args.label, ran, replayed)
        row["checkout"] = {"path": str(Path(args.checkout).resolve()), "commit": subprocess.run(
            ["git", "-C", str(args.checkout), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(), "port": args.port}
        (out / "row.json").write_text(json.dumps(row, indent=1) + "\n")
        print(f"{case['name']}: {row['pieces_seated']} of {row['pieces_total']} seated, {row['wall_s']} s"
              + (f", recovered {row['recovered']}" if case["disturbance"] else "") + (f"; stopped: {row['stopped']}" if row["stopped"] else ""), flush=True)


# ---- the summary and the bar ----------------------------------------------------------------------------------

def rate(rows):
    """(pieces seated, pieces there were) over rows; a build that left no count is counted as the puzzle's pieces, none seated."""
    total = max((row["pieces_total"] or 0 for row in rows), default=0)
    return sum(row["pieces_seated"] for row in rows), sum(row["pieces_total"] or total for row in rows)


def bar(rows, main, ours, reference=None):
    """The pass bar, item by item, over the cases both ``main`` and ``ours`` built: ``[{"item", "passed", "detail"}]``.
    ``passed`` is None where the rows cannot say (nothing built yet, or not this benchmark's to measure)."""
    by = lambda label: {row["case"]: row for row in rows if row["label"] == label}
    theirs, mine = by(main), by(ours)
    shared = [name for name in mine if name in theirs and not mine[name]["group"].startswith("stretch")]
    percent = lambda pair: f"{pair[0]} of {pair[1]}" + (f" ({100 * pair[0] / pair[1]:.0f} percent)" if pair[1] else "")
    share = lambda pair: pair[0] / pair[1] if pair[1] else 0.0
    out = []
    a, b = rate([mine[name] for name in shared]), rate([theirs[name] for name in shared])
    layouts = sorted({mine[name]["layout"] for name in shared})
    worse = [layout for layout in layouts if share(rate([mine[n] for n in shared if mine[n]["layout"] == layout]))
             < share(rate([theirs[n] for n in shared if theirs[n]["layout"] == layout]))]
    out.append({"item": f"{ours} seats more than {main}, and no layout is worse", "passed": bool(share(a) > share(b) and not worse) if shared else None,
                "detail": f"{ours} {percent(a)}, {main} {percent(b)}, over {len(shared)} cases built by both" + (f"; worse on {worse}" if worse else "")})
    base = [row for row in mine.values() if row["group"] == "base"]
    out.append({"item": "at least 95 percent seated on the base cases", "passed": bool(share(rate(base)) >= 0.95) if base else None,
                "detail": f"{percent(rate(base))} over {len(base)} base cases"})
    disturbed = [row for row in mine.values() if row["group"] == "disturbance"]
    recovered = sum(bool(row["recovered"]) for row in disturbed)
    out.append({"item": "at least 90 percent recovery on the disturbance cases", "passed": bool(recovered >= 0.9 * len(disturbed)) if disturbed else None,
                "detail": f"{recovered} of {len(disturbed)} recovered"})
    known = by(reference) if reference else {}
    both = [name for name in mine if name in known and mine[name]["group"] == "base"]
    if both:
        ours_s, known_s = statistics.median(mine[n]["wall_s"] for n in both), statistics.median(known[n]["wall_s"] for n in both)
        out.append({"item": f"median time not worse than {reference}", "passed": bool(ours_s <= known_s),
                    "detail": f"{ours} {ours_s:.0f} s, {reference} {known_s:.0f} s, medians over {len(both)} base cases built by both"})
    else:
        out.append({"item": "median time not worse than the correction table's own build", "passed": None, "detail": "no reference label was built"})
    whole = [row for row in mine.values() if row["pieces_total"] and row["pieces_seated"] == row["pieces_total"]]
    judged = [row for row in whole if row.get("bridge")]
    refused = sum(row["bridge"].get("goals_refused", 0) for row in judged)
    out.append({"item": "every seated plan through the dry-run bridge with no gate refusing", "passed": bool(len(judged) == len(whole) and not refused) if whole else None,
                "detail": f"{len(judged)} of {len(whole)} seated builds judged; {refused} goals refused in "
                          f"{sum(bool(row['bridge'].get('goals_refused')) for row in judged)} of them"})
    return out


def summary(args):
    out = Path(args.out).resolve()
    rows = [json.loads(path.read_text()) for path in out.glob("*/**/row.json")]
    rows = sorted((row for row in rows if Path(row["folder"]) == out / row["label"] / row["case"]),          # a column's rows, not a trial's kept beside them
                  key=lambda row: (row["case"], row["label"]))
    (out / "metrics.json").write_text(json.dumps(rows, indent=1) + "\n")
    labels = [label for label in (args.main, args.ours, args.reference) if label and any(row["label"] == label for row in rows)]
    labels += sorted({row["label"] for row in rows} - set(labels))
    cell = lambda row: "not built" if row is None else (
        f"{row['pieces_seated']} of {row['pieces_total'] or '?'}"
        + (f", {row['seat_error_mm']['mean']} / {row['seat_error_mm']['max']} mm" if row["seat_error_mm"] else "")
        + f", {row['wall_s']:.0f} s, {row['rescans'] if row['rescans'] is not None else '?'} rescans, {row['failed_picks'] if row['failed_picks'] is not None else '?'} failed picks"
        + ("" if row["recovered"] is None else ", recovered" if row["recovered"] else ", NOT recovered")
        + (f", bridge {row['bridge'].get('goals_refused')} of {row['bridge'].get('goals')} goals refused" if row.get("bridge") else ""))
    lines = [f"# The puzzle benchmark: {', '.join(labels)}", "",
             "Per case and label: pieces seated, mean / max error of the seated pieces against the CAD, the build's wall time, scans of the table "
             "after the first, pick attempts that did not end with the piece seated, and for a disturbed build whether it recovered "
             "(every piece seated and the graph ended in success).", "",
             "| Case | " + " | ".join(labels) + " |", "|---|" + "---|" * len(labels)]
    for name in dict.fromkeys(row["case"] for row in rows):
        found = {row["label"]: row for row in rows if row["case"] == name}
        lines.append(f"| {name} | " + " | ".join(cell(found.get(label)) for label in labels) + " |")
    lines += ["", "## Totals", "", "| Group | " + " | ".join(labels) + " |", "|---|" + "---|" * len(labels)]
    for group in dict.fromkeys(row["group"] for row in rows):
        totals = []
        for label in labels:
            mine = [row for row in rows if row["group"] == group and row["label"] == label]
            a, b = rate(mine)
            totals.append("not built" if not mine else f"{a} of {b} seated in {len(mine)} builds, {sum(row['pieces_seated'] == row['pieces_total'] for row in mine)} finished"
                          + (f", median {statistics.median(row['wall_s'] for row in mine):.0f} s" if mine else "")
                          + (f", {sum(bool(row['recovered']) for row in mine)} recovered" if group == "disturbance" else ""))
        lines.append(f"| {group} | " + " | ".join(totals) + " |")
    if args.main and args.ours:
        lines += ["", f"## The bar: {args.ours} against {args.main}", "", "| Item | Passed | What the rows say |", "|---|---|---|"]
        lines += [f"| {item['item']} | {'not known' if item['passed'] is None else 'yes' if item['passed'] else 'NO'} | {item['detail']} |"
                  for item in bar(rows, args.main, args.ours, args.reference)]
        lines += ["| the repository's tests pass | not this file's to say | run them |"]
    stops = [row for row in rows if row["stopped"]]
    if stops:
        lines += ["", "## Builds that stopped", ""] + [f"- {row['case']} ({row['label']}): {row['stopped']}. `{row['folder']}`" for row in stops]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "child":
        return child(sys.argv[2:])
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("action", choices=["cases", "lay", "run", "summary"])
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--group", default="base", help="base, disturbance, stretch/<name> or all")
    parser.add_argument("--cases", nargs="+", help="run: these cases by name, in place of a group; lay: only these tables")
    parser.add_argument("--port", type=int, help="run: the simulator to build on; its stage is replaced")
    parser.add_argument("--out", type=Path, help="run and summary: the folder of the whole benchmark run")
    parser.add_argument("--label", help="run: what these builds are called in the summary (a branch, say)")
    parser.add_argument("--checkout", type=Path, default=ROOT, help="run: the repository whose build is run")
    parser.add_argument("--colours", type=Path, help="run: the pieces' colours for the simulator's camera (the build's --colours)")
    parser.add_argument("--scan-cycle", type=Path, help="run: the build's --scan-cycle, for a build that has a scan step")
    parser.add_argument("--sam-client", type=Path, help="run: a file with track(clip, name, prompts); the build's looks go to it")
    parser.add_argument("--bridge", type=Path, help="run: the puzzle project's ros2 folder; every build that ends is judged by its dry-run bridge")
    parser.add_argument("--main", default="main", help="summary: the label the bar compares against")
    parser.add_argument("--ours", default="integ", help="summary: the label the bar is about")
    parser.add_argument("--reference", help="summary: the label whose median time is the bar's")
    args = parser.parse_args()
    if args.action == "cases":
        for case in cases(json.loads(args.config.read_text())):
            if args.group in ("all", case["group"]):
                print(json.dumps(case))
    elif args.action == "lay":
        lay(args)
    elif args.action == "run":
        if not (args.port and args.out and args.label and args.colours):
            parser.error("run needs --port, --out, --label and --colours")
        os.environ.setdefault("PYTHONUNBUFFERED", "1")
        run(args)
    else:
        if not args.out:
            parser.error("summary needs --out")
        summary(args)


if __name__ == "__main__":
    main()
