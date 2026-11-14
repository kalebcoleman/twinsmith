"""Run checks on disposable simulators; --list is offline and never starts Kit."""

import argparse
import asyncio
import importlib.util
import json
import os
import shlex
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback

SCRIPTS = Path(__file__).resolve().parent
# The checks import the MCP SDK, which a stock python3 lacks; --list does not. Only validate_scene_edges also imports pxr
# on the host; every other pxr import (the work cell check's among them) is code sent to Kit.
RUN = "cd mcp && uv run --locked --isolated --python 3.12 --extra test --extra isaaclab --extra scene-test python ../scripts/smoke.py"
TOOLSETS = {
    "core": ["validate_scene_sessions"],
    "sim": ["validate_scene_tools", "validate_scene_prim_edit", "validate_scene_clear", "validate_scene_reopen",
            "validate_generic_scene", "validate_scene_joints", "validate_scene_edges", "validate_scene_equivalents",
            "newton_checks", "validate_scene_sensors", "validate_scene_camera_freshness",
            "smoke_test", "workcell_smoke"],
    "assets": [], "computer": [],
}


# Checks that only mean something on Newton: on PhysX they are skipped, with a note, not failed.
NEWTON_ONLY = ("newton_checks",)
# Checks that run only when named with --check: the Isaac Lab job takes no --backend, and the work cell needs
# --config and --mutate-stage. A whole-toolset run reports them as skipped with the command that runs them.
ON_REQUEST = {"smoke_test": "--check smoke_test", "workcell_smoke": "--check workcell_smoke --config <absolute-config> --mutate-stage"}

# The TWINSMITH_TOOLSETS value each check's MCP server is started with, so a check never relies on the
# default selection: the smallest selection that lists every tool the check calls (a test compares this
# table with the tool names in each check's source). Lab and workcell run their own processes.
DEFAULT_TOOLSETS = "default,+sim,+assets"
CHECK_TOOLSETS = {
    "validate_scene_tools": "all",                      # checks the listing against the whole registry
    "smoke_test": "default,+sim",
    "workcell_smoke": "default,+sim",
}

# These checks work on one shared stage, each on what the earlier ones leave (their docstrings say "after
# validate_scene_tools", "after the core/equivalents suite", "final destructive clear"). They run in this order,
# with only the timeline stopped between them; every other check starts on a new empty stage.
# Camera freshness runs first after tools: every camera capture holds renderer descriptors, and after the
# equivalents, edges and sensors checks (hundreds of captures) its cold capture exhausted them ("Out of resource
# descriptors") and stalled the renderer of a Newton session.
SUITE = ("validate_scene_tools", "validate_scene_camera_freshness", "validate_scene_equivalents", "validate_scene_edges",
         "validate_scene_sensors", "validate_scene_clear")
# The checks whose stage a check needs. They are added to the run, and a check whose prerequisite did not pass
# is "not_run" rather than failed on a stage it cannot trust.
PREREQUISITE = {
    "validate_scene_equivalents": ("validate_scene_tools",),
    "validate_scene_edges": ("validate_scene_tools", "validate_scene_equivalents"),
    "validate_scene_sensors": ("validate_scene_tools", "validate_scene_equivalents"),
    "validate_scene_camera_freshness": ("validate_scene_tools",),
    "validate_scene_clear": ("validate_scene_tools", "validate_scene_equivalents"),
}

# Checks that need a simulator session that was set up by hand, and the setup to report when it is absent.
NEEDS_SAVED_SESSION = "validate_scene_reopen"

# The bridge answers status before the physics engine is switched (seen on Newton: about 2.4 s early), so the
# engine is read until it has been the same twice in a row, within this bound.
PROBE_TIMEOUT_S = 60.0
PROBE_INTERVAL_S = 1.0
PROBE_STABLE_READINGS = 2


class BackendMismatch(RuntimeError):
    pass


def probe(client, requested, timeout=None, interval=None, sleep=time.sleep, clock=time.monotonic):
    """The simulator's identity and engine. Waits, bounded, until the reported engine is stable and, when one
    was requested, equals it; only a mismatch that persists to the end of the bound is refused."""
    timeout = PROBE_TIMEOUT_S if timeout is None else timeout
    interval = PROBE_INTERVAL_S if interval is None else interval
    deadline = clock() + timeout
    readings, metadata, last_error = [], {}, None
    while True:
        try:
            identity = client.call("status", {})
            # The engine the status tool reports as physics.active: sim.stage.info runtime.physics_engine.
            backend = (client.call("sim.stage.info", {}).get("runtime") or {}).get("physics_engine")
            if backend is None:  # a bridge without the field: ask the simulation manager
                backend = client.call("run_python", {"code": (
                    "from isaacsim.core.simulation_manager import SimulationManager\n"
                    "from sim_tools.compat import adapt_simulation_manager\n"
                    "result = adapt_simulation_manager(SimulationManager).get_active_physics_engine()"
                )})["result"]
            metadata = {"backend": backend, "simulator_build": identity.get("app_version"),
                        "pid": identity.get("pid"), "port": client.port}
            readings = (readings + [backend])[-PROBE_STABLE_READINGS:]
            last_error = None
            stable = len(readings) == PROBE_STABLE_READINGS and len(set(readings)) == 1
            if stable and backend in ("newton", "physx") and (not requested or requested == backend):
                return metadata
        except Exception as error:  # a bridge that is still starting answers with errors
            last_error, readings = error, []
        if clock() >= deadline:
            break
        sleep(interval)
    if last_error is not None and not metadata:
        raise last_error
    backend = metadata.get("backend")
    error = BackendMismatch(f"Requested {requested or 'known engine'}, but simulator reports {backend!r} "
                            f"(unchanged for {timeout:g} s); refusing to run")
    error.metadata = metadata
    raise error


def reset(client, new_stage=True):
    """Stop the timeline so that a check that failed, or was cut off, cannot leave the next one on a playing
    scene, and (unless the next check continues on the previous one's stage) open a new empty stage so that no
    prim, sensor or physics state is carried over. Returns a note when it could not (the next check's engine
    probe finds a dead bridge)."""
    try:
        info = client.call("sim.stage.info", {})
        if isinstance(info, dict) and info.get("stopped") is False:
            client.call("sim.timeline", {"action": "stop", "session_id": info["session_id"],
                                                 "stage_id": info["stage_id"]})
        if new_stage:
            client.call("run_python", {"code": "import omni.usd\nresult = omni.usd.get_context().new_stage()"})
    except Exception as error:  # noqa: BLE001 - reported, not fatal
        return f"reset: {type(error).__name__}: {error}"
    return None


def plan(checks):
    """The checks in run order: the prerequisites of a selected check are added, and the checks that share a
    stage run together, in their own order, where the first of them was."""
    wanted = list(checks)
    for name in checks:
        wanted += [need for need in PREREQUISITE.get(name, ()) if need not in wanted]
    ordered = []
    for name in wanted:
        if name in SUITE:
            if None not in ordered:
                ordered.append(None)  # where the suite goes
        else:
            ordered.append(name)
    suite = [name for name in SUITE if name in wanted]
    return [item for name in ordered for item in (suite if name is None else [name])]


def reopen_setup(args):
    """What validate_scene_reopen needs: a simulator that has a saved scene open. The launcher cannot open a
    file at startup, so the scene is opened through the stage tool before the check is run."""
    port = args.port
    launch = f" --physics-backend {args.backend}" if args.backend else ""   # none: the configured default engine
    check = f" --backend {args.backend}" if args.backend else ""
    return ("needs a session opened with a saved file. Save a scene with a rigid body (sim__stage action=save), "
            f"start a simulator ./run.sh --headless{launch} --port {port}, open that file in it "
            f"(sim__stage action=open path=<saved.usda>), then run: {RUN} sim "
            f"{check.strip()} --port {port} --check validate_scene_reopen "
            "--saved <saved.usda> --body </World/Body> --expected-z <rest height in metres>")


def leaves(error):
    if isinstance(error, BaseExceptionGroup):
        for inner in error.exceptions:
            yield from leaves(inner)
    else:
        yield error


def describe(error):
    """``Type: message`` of the innermost exception, with the failing source line when it has no message."""
    text = f"{type(error).__name__}: {error}" if str(error) else type(error).__name__
    frames = traceback.extract_tb(error.__traceback__)
    if not str(error) and frames:
        text += f" at {Path(frames[-1].filename).name}:{frames[-1].lineno}: {frames[-1].line}"
    return text


def load_check(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / "smoke" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(args, *names):
    missing = ["--" + name.replace("_", "-") for name in names if getattr(args, name) is None]
    if missing:
        raise ValueError("This check requires " + ", ".join(missing))


def execute(name, args, output, client_factory, metadata):
    if name in ("smoke_test", "workcell_smoke"):
        command = [sys.executable, str(SCRIPTS / f"{name}.py")]
        if name == "workcell_smoke":
            require(args, "config")
            if not args.mutate_stage:
                raise ValueError("workcell requires --mutate-stage")
            if metadata["backend"] not in ("physx", "newton"):
                raise ValueError(f"workcell_smoke supports PhysX and Newton, not {metadata['backend']!r}")
            command += ["--host", args.host, "--port", str(args.port), "--config", args.config,
                        "--mutate-stage"]
            if args.discard_changes:
                command.append("--discard-changes")
        elif args.backend:
            raise ValueError("Lab smoke chooses its task's engine; --backend cannot be verified by this check")
        log_path = output.with_suffix(".log")
        with log_path.open("w") as log:
            done = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if done.returncode:
            tail = "\n".join(log_path.read_text(errors="replace").splitlines()[-12:])
            raise RuntimeError(f"{name} exited with status {done.returncode}; last lines of {log_path}:\n{tail}")
        return
    if name == "validate_generic_scene":
        if not args.discard_changes:
            raise ValueError("Generic scene replaces the stage; requires --discard-changes")
        source = (SCRIPTS / "smoke" / f"{name}.py").read_text()
        code = f"namespace = {{'__name__': 'smoke_generic'}}\nexec({source!r}, namespace)\nresult = namespace['run_checks']()"
        with client_factory(host=args.host, port=args.port, timeout=120) as client:
            result = client.call("run_python", {"code": code})
        if result.get("result", {}).get("status") != "passed":
            raise RuntimeError(f"Generic scene did not pass: {result}")
        output.write_text(json.dumps(result, indent=2))
        return
    if name == "newton_checks":
        if not args.discard_changes:
            raise ValueError("Newton checks replace the open stage twice; requires --discard-changes")
        asyncio.run(load_check(name).validate(args.port, str(output)))
        return
    module = load_check(name)
    if name == "validate_scene_sessions":
        require(args, "second_port")
        with client_factory(host=args.host, port=args.second_port) as client:
            metadata["second_simulator"] = probe(client, args.backend)
        coroutine = module.validate(args.port, args.second_port, str(output))
    elif name == "validate_scene_reopen":
        require(args, "saved", "body", "expected_z")
        options = argparse.Namespace(**vars(args))
        options.output = str(output)
        coroutine = module.validate(options)
    elif name == "validate_scene_prim_edit":
        require(args, "directory")
        coroutine = module.validate(args.port, str(output), args.directory, args.robot)
    elif name == "validate_scene_joints":
        coroutine = module.validate(args.port, str(output), args.robot)
    elif name in ("validate_scene_tools", "validate_scene_equivalents"):
        coroutine = module.validate(args.port, str(output), backend=args.backend)
    else:
        coroutine = module.validate(args.port, str(output))
    asyncio.run(coroutine)


def main(argv=None, client_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("toolset", nargs="?", choices=[*TOOLSETS, "all"],
                        help="the toolset to check, or all; optional only with --list")
    parser.add_argument("--backend", choices=("newton", "physx"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--check", action="append", help="Run only this named check from the toolset; repeatable")
    parser.add_argument("--second-port", type=int)
    parser.add_argument("--directory")
    parser.add_argument("--robot", choices=("ur5e", "franka"), default="franka")
    parser.add_argument("--saved")
    parser.add_argument("--body")
    parser.add_argument("--expected-z", type=float)
    parser.add_argument("--config")
    parser.add_argument("--mutate-stage", action="store_true")
    parser.add_argument("--discard-changes", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--toolsets", help="TWINSMITH_TOOLSETS for every check's MCP server, instead of each check's own")
    args = parser.parse_args(argv)
    if args.toolset is None:
        if not args.list:
            parser.error("a toolset (or all) is required unless --list is given")
        args.toolset = "all"
    selected = TOOLSETS if args.toolset == "all" else {args.toolset: TOOLSETS[args.toolset]}
    if args.list:
        print("Toolset     Check                               GPU   Running simulator")
        for toolset, checks in selected.items():
            for name in checks:
                needs_sim = "no (Lab job)" if name == "smoke_test" else "yes (two)" if name == "validate_scene_sessions" else "yes"
                print(f"{toolset:11} {name:35} yes   {needs_sim}")
            if not checks:
                print(f"{toolset:11} no checks registered                no    no")
        return 0
    checks = [name for names in selected.values() for name in names]
    if args.check:
        unknown = set(args.check) - set(checks)
        if unknown:
            parser.error(f"Checks not in this toolset: {', '.join(sorted(unknown))}")
        checks = [name for name in checks if name in args.check]
    on_request = [] if args.check else [name for name in checks if name in ON_REQUEST]
    checks = [name for name in checks if name not in on_request]
    if not checks:
        print("No checks registered for this toolset", file=sys.stderr)
        return 2
    if client_factory is None:
        missing = [m for m in ("mcp", "pxr") if (m == "mcp" or "validate_scene_edges" in checks)
                   and importlib.util.find_spec(m) is None]
        if missing:
            print(f"This Python ({sys.executable}) has no {' or '.join(missing)}, which the checks import. "
                  f"Run from the checkout root: {RUN} {shlex.join(argv if argv is not None else sys.argv[1:])}",
                  file=sys.stderr)
            return 2
        sys.path.insert(0, str(SCRIPTS.parent / "mcp"))
        from twinsmith.client import BridgeClient
        client_factory = BridgeClient
    output_dir = args.output_dir or Path(tempfile.mkdtemp(prefix="smoke-"))
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    exit_code = 0
    previous_host = os.environ.get("TWINSMITH_HOST")
    previous_toolsets = os.environ.get("TWINSMITH_TOOLSETS")
    os.environ["TWINSMITH_HOST"] = args.host
    checks = plan(checks)
    results = {}
    halted = None
    on_suite_stage = False
    try:
        for name in checks:
            if halted:
                rows.append({"check": name, "status": "not_run", "backend": None, "simulator_build": None,
                             "seconds": 0, "error": halted})
                continue
            failed = [need for need in PREREQUISITE.get(name, ()) if results.get(need) != "pass"]
            if failed:
                rows.append({"check": name, "status": "not_run", "backend": None, "simulator_build": None, "seconds": 0,
                             "error": f"Needs the stage {', '.join(failed)} leaves, and that check did not pass"})
                results[name] = "not_run"
                continue
            if name == NEEDS_SAVED_SESSION and any(getattr(args, option) is None for option in ("saved", "body", "expected_z")):
                row = {"check": name, "status": "needs_setup", "backend": None, "simulator_build": None, "seconds": 0,
                       "error": reopen_setup(args)}
                rows.append(row)
                results[name] = "needs_setup"
                continue
            start = time.monotonic()
            metadata = {"backend": None, "simulator_build": None}
            row = {"check": name, "status": "fail"}
            os.environ["TWINSMITH_TOOLSETS"] = args.toolsets or CHECK_TOOLSETS.get(name, DEFAULT_TOOLSETS)
            row["toolsets"] = os.environ["TWINSMITH_TOOLSETS"]
            try:
                if name != "smoke_test":
                    with client_factory(host=args.host, port=args.port) as client:
                        metadata = probe(client, args.backend)
                        # the reopen check works on the saved scene the session already has open
                        note = reset(client, new_stage=not (name in SUITE and on_suite_stage) and name != NEEDS_SAVED_SESSION)
                        if note:
                            row["note"] = note
                if name in NEWTON_ONLY and metadata["backend"] != "newton":
                    row["status"] = "skipped"
                    row["note"] = f"Newton-only check; this simulator runs {metadata['backend']}, so it was not run"
                else:
                    execute(name, args, output_dir / f"{name}.json", client_factory, metadata)
                    row["status"] = "pass"
            except BackendMismatch as error:
                metadata.update(getattr(error, "metadata", {}))
                row["error"] = str(error)
                exit_code = 2
                halted = f"Stopped: {error}"
            except (Exception, SystemExit, BaseExceptionGroup) as error:
                found = [leaf for leaf in leaves(error)]
                row["error"] = "; ".join(describe(leaf) for leaf in found)
                trace = "".join("".join(traceback.format_exception(type(leaf), leaf, leaf.__traceback__)) for leaf in found)
                (output_dir / f"{name}.traceback.txt").write_text(trace)
                row["traceback"] = str(output_dir / f"{name}.traceback.txt")
                print(f"=== {name}: leaf exception ===\n{trace}", file=sys.stderr)
                exit_code = exit_code or 1
            row.update(metadata)
            row["seconds"] = round(time.monotonic() - start, 3)
            rows.append(row)
            results[name] = row["status"]
            on_suite_stage = name in SUITE
    finally:
        if previous_host is None:
            os.environ.pop("TWINSMITH_HOST", None)
        else:
            os.environ["TWINSMITH_HOST"] = previous_host
        if previous_toolsets is None:
            os.environ.pop("TWINSMITH_TOOLSETS", None)
        else:
            os.environ["TWINSMITH_TOOLSETS"] = previous_toolsets
    for name in checks[len(rows):]:
        rows.append({"check": name, "status": "not_run", "backend": None,
                     "simulator_build": None, "seconds": 0, "error": "Not reached"})
    rows += [{"check": name, "status": "skipped", "backend": None, "simulator_build": None, "seconds": 0,
              "note": f"Runs only when named: smoke.py {args.toolset} {ON_REQUEST[name]}"} for name in on_request]
    report = output_dir / "results.json"
    report.write_text(json.dumps({"toolset": args.toolset, "requested_backend": args.backend,
                                  "checks": rows}, indent=2) + "\n")
    print("Check                               Result   Backend  Build        Seconds")
    for row in rows:
        print(f"{row['check']:35} {row['status']:8} {str(row['backend']):8} {str(row['simulator_build']):12} {row['seconds']}")
        if "error" in row:
            print("  " + (row["error"] if len(row["error"]) <= 1200 else row["error"][:1200] + " ... (full text in results.json)"))
        if "note" in row:
            print("  " + row["note"])
    print(f"JSON: {report}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
