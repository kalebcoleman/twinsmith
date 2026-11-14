"""Isolated RSS commands, durable run ledgers, and bounded asynchronous waits."""

from __future__ import annotations

import ast
import concurrent.futures
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import threading
import time
import uuid

from twinsmith import sibling

from . import plan
from . import template as scene_template_module
from .template import FRAME, PARAMETERS, SHAPES, SIZE_MEANING

PIN = "ba3650c"
REPOSITORY = "https://github.com/robosimstudio/robosimstudio"
ACTIONS = ("status", "start", "scene_template", "update_scene", "reference", "snapshot", "compare",
           "rollout", "tune_push", "compare_take", "stop", "report")
RECENT_RUNS = 5
STOP_WAIT_S = 15.0
BASE_PORT = 8080


def _bundled_ffmpeg():
    """The ffmpeg binary that the imageio-ffmpeg package ships (a dependency of this server), or None."""
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def _ffmpeg_exe():
    """ffmpeg on PATH, else the bundled one, else None. A PC without a system ffmpeg still trims clips."""
    return shutil.which("ffmpeg") or _bundled_ffmpeg()


NO_BRIDGE = ("robosim needs no Isaac Sim or Kit bridge: it runs RoboSimStudio in its own Python environment. "
             "The device (cpu or cuda:0) is chosen at start.")


def _now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _name(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", value):
        raise ValueError("Use a name of 1–80 letters, digits, underscores or hyphens.")
    return value


def _identity(pid):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text().rsplit(") ", 1)[1].split()
        if stat[0] == "Z":
            return None
        return {"pid": int(pid), "start_ticks": stat[19],
                "cmdline": Path(f"/proc/{pid}/cmdline").read_bytes().decode().split("\0")[:-1]}
    except (OSError, ValueError, IndexError):
        return None


class RoboSimBridge:
    """One instance per MCP host; only RSS's private environment executes scenes.

    Scene Python is trusted code, not sandboxed. A run is portable on disk, but
    an in-flight host worker cannot be resumed after the host exits.
    """

    def __init__(self, workspace=None, root=None):
        self.checkout = sibling.Checkout("robosimstudio", "TWINSMITH_ROBOSIM_ROOT", PIN, REPOSITORY,
                                         workspace=workspace, root=root)
        self.workspace, self.root = self.checkout.workspace, self.checkout.root
        self.runs = self.workspace / "runs/robosim"
        self._jobs = {}
        self._mutex = threading.RLock()

    def _installation(self):
        info = self.checkout.info()
        info.pop("setup", None)
        commit = info["rss_commit"] = info.pop("commit")
        agent = self.root / "robosimstudio_real2sim_agent/__main__.py"
        if not info["environment_present"] or not agent.is_file() or not commit:
            info.update(ok=False, error=f"RSS checkout/private environment missing or unreadable. Clone {REPOSITORY} "
                        f"at commit {PIN} into {self.root}, then provision its own .venv/bin/python. "
                        "The bridge never installs RSS or changes its environment.")
        return info

    def _load(self, run):
        path = self.runs / _name(run)
        if path.is_symlink() or path.resolve().parent != self.runs.resolve():
            raise ValueError("Run must be a direct, non-symlink bridge run folder.")
        ledger = json.loads((path / "run.json").read_text())
        if ledger.get("bridge") != "twinsmith-robosim-v1" or ledger.get("run") != run:
            raise ValueError("Not a bridge-owned RSS run.")
        if ledger["checkout"] != str(self.root):
            raise ValueError("Run belongs to another RSS checkout; set TWINSMITH_ROBOSIM_ROOT accordingly.")
        return path, ledger

    def _save(self, path, ledger):
        ledger["updated_at"] = _now()
        temporary = path / f".run-{uuid.uuid4().hex}.json"
        temporary.write_text(json.dumps(ledger, indent=2) + "\n")
        temporary.replace(path / "run.json")

    def _environment(self, device):
        env = os.environ.copy()
        for key in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
            env.pop(key, None)
        env.update(CUDA_VISIBLE_DEVICES="" if device == "cpu" else "0",
                   ROBOSIMSTUDIO_DEVICE=device, PYTHONDONTWRITEBYTECODE="1",
                   TMPDIR=str(self.root / ".tmp"), XDG_CACHE_HOME=str(self.root / ".cache"),
                   WARP_CACHE_PATH=str(self.root / ".cache/warp"))
        for key in ("TMPDIR", "XDG_CACHE_HOME", "WARP_CACHE_PATH"):
            Path(env[key]).mkdir(parents=True, exist_ok=True)
        env["PATH"] = str(self.root / ".venv/bin") + os.pathsep + env.get("PATH", "")
        return env

    def _gpu_guard(self, allow, owned_pid=None):
        if allow:
            return
        try:
            result = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader,nounits"],
                                    capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            raise ValueError("Cannot check GPU workloads; refusing GPU use. Choose cpu or explicitly allow_concurrent_gpu.") from None
        if result.returncode:
            raise ValueError("Cannot check GPU workloads; refusing GPU use. Choose cpu or explicitly allow_concurrent_gpu.")
        others = [line for line in result.stdout.splitlines()
                  if line.strip() and line.split(",", 1)[0].strip() != str(owned_pid)]
        if others:
            raise ValueError("Other GPU work is running: " + "; ".join(others)
                             + ". Use cpu, or knowingly set allow_concurrent_gpu=true. Nothing was stopped.")

    def _claim_server(self, path, ledger):
        if ledger.get("server"):
            return
        try:
            identity = _identity(int((path / "serve.pid").read_text().strip()))
        except (OSError, ValueError):
            return
        if identity and "robosimstudio_real2sim_agent.serve" in identity["cmdline"]:
            arguments = identity["cmdline"]
            if "--run-dir" in arguments and arguments[arguments.index("--run-dir") + 1] == str(path):
                ledger["server"] = identity
                self._save(path, ledger)

    def _owned_alive(self, ledger):
        server = ledger.get("server")
        return bool(server and _identity(server["pid"]) == server)

    def _command(self, path, ledger, args, timeout=630, argv=None):
        argv = argv or [str(self.root / ".venv/bin/python"), "-m", "robosimstudio_real2sim_agent", "--run-dir", str(path), *args,
                        "--json"]
        record = {"argv": argv, "started_at": _now()}
        ledger["commands"].append(record)
        number = len(ledger["commands"])
        stdout = path / f"command-{number:04d}.stdout"
        stderr = path / f"command-{number:04d}.stderr"
        record.update(stdout=str(stdout), stderr=str(stderr))
        self._save(path, ledger)
        started = time.monotonic()
        try:
            with stdout.open("w") as out, stderr.open("w") as err:
                result = subprocess.run(argv, cwd=path, env=self._environment(ledger["device"]),
                                        stdin=subprocess.DEVNULL, stdout=out, stderr=err, timeout=timeout)
            record["returncode"] = result.returncode
            text = stdout.read_text(errors="replace")
            try:
                reply = json.loads(text)
            except ValueError:
                reply = {"ok": False, "error": "RSS returned non-JSON output; inspect command logs.", "tail": text[-2000:]}
            if not isinstance(reply, dict):
                reply = {"ok": False, "error": "RSS returned a non-object JSON response."}
            if result.returncode or not reply.get("ok", False):
                reply["ok"] = False
                reply.setdefault("error", stderr.read_text(errors="replace")[-2000:] or "RSS command failed.")
            record["result"] = reply
            return reply
        finally:
            record.update(finished_at=_now(), wall_s=round(time.monotonic() - started, 3))
            if args and args[0] == "up":
                self._claim_server(path, ledger)
            self._save(path, ledger)

    @staticmethod
    def _sha256(file):
        digest = hashlib.sha256()
        with Path(file).open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _copy_input(self, source, destination):
        original = Path(source).expanduser()
        if not original.is_absolute() or not original.is_file():
            raise ValueError("Input must be an existing absolute file path.")
        shutil.copyfile(original, destination)
        return {"source": str(original.resolve()), "copy": str(destination), "sha256": self._sha256(destination)}

    @staticmethod
    def _current_template(ledger):
        """The template data behind the scene version now loaded, or None when it came from a file."""
        for entry in reversed(ledger.get("scene_versions") or []):
            if entry.get("loaded"):
                return entry.get("template")
        return None

    def _scene_template(self, info, run, shape, template):
        """Write a template's scene file (into the run folder, else the shared templates folder) and describe it."""
        spec = dict(template or {})
        spec.setdefault("shape", shape)
        resolved = scene_template_module.resolve(spec)
        text = scene_template_module.scene_source(resolved)
        if run:
            folder, _ = self._load(run)
        else:
            folder = self.runs / "_templates"
            folder.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256(text.encode()).hexdigest()[:8]
        path = folder / f"scene-template-{resolved['shape']}-{digest}.py"
        path.write_text(text)
        shown = {key: value for key, value in resolved.items() if key not in ("mask_key", "push_default")}
        return dict(info, ok=True, path=str(path), shape=resolved["shape"], template=shown, parameters=PARAMETERS,
                    size_meaning=SIZE_MEANING[resolved["shape"]], mask_key=resolved["mask_key"], frame=FRAME,
                    use=("start(name=..., device='cpu', template={...}) builds this scene from data, no file needed; "
                         f"scene='{path}' starts from the file (advanced: edit its constants, trusted Python, not sandboxed)."),
                    note=("Image comparison only; no mass or friction fitting. Physics: cpu runs plain MuJoCo, cuda:0 runs "
                          "Newton's MuJoCo-Warp. The scene has a static table, the object, a pusher cylinder and one camera."))

    def call(self, action, *, run=None, name=None, scene=None, device=None, video=None,
             keys=None, take=None, routine=None, camera=None, at=None, shape="tee", push=None,
             duration_s=None, steps=None, record_fps=10.0, start_s=None, end_s=None,
             sim_start_s=None, sim_end_s=None, size="320x240", width=320, port=None, wait_s=0,
             allow_concurrent_gpu=False, template=None, show_all=False, max_trials=None):
        info = self._installation()
        try:
            if info.get("error"):
                return info
            if not isinstance(wait_s, (int, float)) or not math.isfinite(wait_s) or not 0 <= wait_s <= 120:
                raise ValueError("wait_s must be between 0 and 120 seconds.")
            if action not in ACTIONS:
                raise ValueError("action must be one of: " + ", ".join(ACTIONS))
            if action == "scene_template":
                return self._scene_template(info, run, shape, template)
            if action == "status":
                return dict(info, bridge_needed=False, note=NO_BRIDGE, **self._status(run, wait_s, show_all))
            scene_source = resolved = None
            if action in ("start", "update_scene"):
                if template is not None and scene:
                    raise ValueError("Give template (plain data) or scene (a .py file), not both.")
                if template is None and not scene:
                    raise ValueError("Give template={...} (plain data; {} is the default T) or scene=<absolute .py file>.")
                if template is not None:
                    if action == "start":
                        resolved = scene_template_module.resolve(template)
                        scene_source = scene_template_module.scene_source(resolved)
                else:
                    if Path(scene).suffix != ".py" or not Path(scene).is_absolute() or not Path(scene).is_file():
                        raise ValueError("scene must be an existing absolute .py file containing trusted scene code.")
                    problems = plan.scene_problems(Path(scene).read_text(errors="replace"))
                    if problems:
                        raise ValueError(" ".join(problems))
            if action == "start":
                if device not in ("cpu", "cuda:0"):
                    raise ValueError("start requires explicit device='cpu' or device='cuda:0'. No automatic GPU selection.")
                _name(name)
                if len(name) > 64:
                    raise ValueError("Run name must be at most 64 characters (80 including the timestamp).")
                if port is not None and (not isinstance(port, int) or isinstance(port, bool) or not 1024 <= port <= 65535):
                    raise ValueError("port must be an integer from 1024 to 65535, or left out to pick a free one.")
                if device == "cuda:0":
                    self._gpu_guard(allow_concurrent_gpu)
                self.runs.mkdir(parents=True, exist_ok=True)
                run = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + name
                path = self.runs / run
                path.mkdir(exist_ok=False)
                ledger = dict(bridge="twinsmith-robosim-v1", run=run, checkout=str(self.root),
                              rss_commit=info["rss_commit"], device=device, created_at=_now(),
                              inputs={}, commands=[], operations=[], server=None, scene_versions=[],
                              reference=None, allow_concurrent_gpu=allow_concurrent_gpu)
                self._install_scene(path, ledger, scene, text=scene_source, template=resolved)
                self._save(path, ledger)
            else:
                path, ledger = self._load(run)
                if action == "update_scene" and template is not None:
                    resolved = scene_template_module.resolve(template, self._current_template(ledger))
                    scene_source = scene_template_module.scene_source(resolved)
                if device is not None and device != ledger["device"]:
                    raise ValueError("device cannot change within a run; start a separate run.")
                if action not in ("stop", "report") and not self._owned_alive(ledger):
                    raise ValueError("Run's bridge-owned server is not alive. Start a new run; no other server was contacted.")
                if action in ("compare", "compare_take", "tune_push") and (not keys or any(not isinstance(key, str) or "=" not in key for key in keys)):
                    raise ValueError("keys must contain RSS masks such as ['block=red'].")
                if action == "tune_push":
                    take = take or "tune"
                    if max_trials is None:
                        max_trials = plan.TUNE_DEFAULT_TRIALS
                    if isinstance(max_trials, bool) or not isinstance(max_trials, int) or not 1 <= max_trials <= plan.TUNE_MAX_TRIALS:
                        raise ValueError(f"max_trials must be a whole number from 1 to {plan.TUNE_MAX_TRIALS}.")
                if action in ("rollout", "compare_take", "tune_push"):
                    _name(take)
                if routine:
                    _name(routine)
                if action in ("rollout", "tune_push"):
                    taken = [take] if action == "rollout" else [f"{take}-{k}" for k in range(1, max_trials + 1)]
                    if any((path / f"{each}.json").exists() for each in taken):
                        raise ValueError("Take already exists; choose a new take name.")
                    if push is not None:
                        push, _ = plan.validate_push(push)
                    for value, label in ((duration_s, "duration_s"), (steps, "steps"), (record_fps, "record_fps")):
                        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
                            raise ValueError(f"{label} must be a finite number.")
                for value, label in ((start_s, "start_s"), (end_s, "end_s"), (sim_start_s, "sim_start_s"), (sim_end_s, "sim_end_s")):
                    if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0):
                        raise ValueError(f"{label} must be a number of seconds, zero or more.")
                if action == "reference" and start_s is not None and end_s is not None and end_s <= start_s:
                    raise ValueError("end_s must be greater than start_s.")
            if not re.fullmatch(r"[1-9][0-9]{0,3}x[1-9][0-9]{0,3}", size):
                raise ValueError("size must be WIDTHxHEIGHT (1–9999 pixels each).")
            if not isinstance(width, int) or not 1 <= width <= 4096:
                raise ValueError("width must be 1–4096.")
            arguments = dict(video=video, keys=keys, take=take, routine=routine, camera=camera, at=at,
                             push=push, duration_s=duration_s, steps=steps, record_fps=record_fps,
                             start_s=start_s, end_s=end_s, sim_start_s=sim_start_s, sim_end_s=sim_end_s,
                             size=size, width=width, port=port, scene=scene if action == "update_scene" else None,
                             scene_source=scene_source if action == "update_scene" else None,
                             template=resolved if action == "update_scene" else None, max_trials=max_trials)
            with self._mutex:
                if run in self._jobs and not self._jobs[run].done():
                    raise ValueError("Run has an active operation; poll status before another action.")
                lock = (path / ".bridge.lock").open("a")
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    lock.close()
                    raise ValueError("Another host is operating this run; poll status.") from None
                try:
                    if action != "start":
                        path, ledger = self._load(run)
                    future = concurrent.futures.Future()
                    operation = {"action": action, "arguments": arguments, "state": "running", "started_at": _now()}
                    ledger["operations"].append(operation)
                    self._save(path, ledger)
                    self._jobs[run] = future
                    threading.Thread(target=self._worker, args=(future, lock, path, ledger, operation, arguments),
                                     daemon=True, name=f"robosim-{run}").start()
                except Exception:
                    lock.close()
                    self._jobs.pop(run, None)
                    raise
            return dict(info, **self._status(run, wait_s))
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
            return dict(info, ok=False, error=str(exc))

    def _worker(self, future, lock, path, ledger, operation, arguments):
        started = time.monotonic()
        try:
            result = self._execute(operation["action"], path, ledger, **arguments)
        except Exception as exc:
            result = {"ok": False, "error": f"{type(exc).__name__}: {exc}", "hint": "Inspect the run's command logs; stop this run if its server remains live."}
        operation.update(state="completed" if result.get("ok") else "failed", result=result,
                         finished_at=_now(), wall_s=round(time.monotonic() - started, 3))
        try:
            self._save(path, ledger)
        finally:
            lock.close()
            future.set_result(result)

    def _status(self, run, wait_s, show_all=False):
        if not run:
            return self._overview(show_all)
        future = self._jobs.get(run)
        if future and wait_s:
            try:
                future.result(timeout=wait_s)
            except concurrent.futures.TimeoutError:
                pass
        path, ledger = self._load(run)
        operation = ledger["operations"][-1] if ledger["operations"] else None
        return {"ok": not operation or operation["state"] != "failed", "run": run, "run_dir": str(path),
                "device": ledger["device"], "engine": plan.engine(ledger["device"]), "run_rss_commit": ledger["rss_commit"],
                "server": ledger.get("server"), "server_alive": self._owned_alive(ledger),
                "operation": operation, "ledger": str(path / "run.json")}

    @staticmethod
    def _age(created_at):
        try:
            seconds = (datetime.datetime.now(datetime.timezone.utc) - datetime.datetime.fromisoformat(created_at)).total_seconds()
        except (TypeError, ValueError):
            return "unknown"
        for unit, size in (("d", 86400), ("h", 3600), ("m", 60)):
            if seconds >= size:
                return f"{int(seconds // size)}{unit}"
        return f"{max(0, int(seconds))}s"

    def _one_line(self, name, ledger):
        operation = ledger["operations"][-1] if ledger.get("operations") else None
        alive = self._owned_alive(ledger)
        if alive:
            state = f"running {operation['action']}" if operation and operation["state"] == "running" else "live"
        else:
            state = "stopped" if ledger.get("server") else "never started"
        engine, age = plan.engine_short(ledger["device"]), self._age(ledger.get("created_at"))
        return {"run": name, "state": state, "device": ledger["device"], "engine": engine, "age": age,
                "line": f"{name} | {state} | {ledger['device']} | {engine} | {age} old", "alive": alive}

    def _overview(self, show_all):
        """Without a run: live servers and the five newest runs, one line each; ``all`` lists every run in full."""
        loaded = []
        for ledger_path in sorted(self.runs.glob("*/run.json"), reverse=True):
            try:
                loaded.append((ledger_path.parent.name, self._load(ledger_path.parent.name)[1]))
            except (OSError, ValueError, KeyError):
                continue
        lines = [self._one_line(name, ledger) for name, ledger in loaded]
        live = [line for line in lines if line["alive"]]
        for line in lines:
            del line["alive"]
        if show_all:
            runs = []
            for name, _ in loaded:
                try:
                    runs.append(self._status(name, 0))
                except (OSError, ValueError, KeyError):
                    continue
            return {"ok": True, "runs": runs, "live_servers": live, "total_runs": len(loaded)}
        return {"ok": True, "live_servers": live, "recent_runs": lines[:RECENT_RUNS], "total_runs": len(loaded),
                "listing": (f"Live servers, then the {RECENT_RUNS} most recent runs, one line each (name | state | device | engine | age). "
                            "status(run=NAME) gives one run's full detail; status(all=true) lists every run in full.")}

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _free_port():
        for candidate in range(BASE_PORT, BASE_PORT + 200):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                try:
                    probe.bind(("", candidate))
                except OSError:
                    continue
                return candidate
        raise ValueError(f"No free port from {BASE_PORT} to {BASE_PORT + 199}; pass port.")

    def _install_scene(self, path, ledger, source=None, text=None, template=None):
        """Install a scene (a file, or text built from template data) as the next scene-vN.py and make
        scene.py (the file RSS loads) the same bytes."""
        ledger.setdefault("scene_versions", [])
        version = len(ledger["scene_versions"]) + 1
        versioned = path / f"scene-v{version}.py"
        if text is not None:
            versioned.write_text(text)
            entry = {"source": "template", "copy": str(versioned), "sha256": self._sha256(versioned), "template": template}
        else:
            entry = self._copy_input(source, versioned)
        shutil.copyfile(versioned, path / "scene.py")
        entry.update(version=version, file=str(versioned), installed_at=_now(), loaded=None)
        ledger["scene_versions"].append(entry)
        ledger["inputs"]["scene"] = dict(entry, copy=str(path / "scene.py"))
        return entry

    def _timeline(self, path):
        try:
            return json.loads((path / "ref/timeline.json").read_text())
        except (OSError, ValueError):
            return {}

    def _poll_idle(self, path, ledger, deadline_s, take=None):
        deadline = time.monotonic() + deadline_s
        while time.monotonic() < deadline:
            status = self._command(path, ledger, ["status"], timeout=30)
            if not status.get("ok") or status.get("last_error"):
                return {"ok": False, "error": "RSS recording failed.", "status": status}
            if not status.get("playing") and not status.get("recording"):
                return {"ok": True, "status": status}
            time.sleep(1)
        return {"ok": False, "error": f"Recording exceeded {deadline_s / 60:.0f} minutes; poll then stop the run."}

    # ------------------------------------------------------------------ actions
    def _execute(self, action, path, ledger, *, video, keys, take, routine, camera, at, push, duration_s, steps,
                 record_fps, start_s, end_s, sim_start_s, sim_end_s, size, width, port, scene, scene_source=None,
                 template=None, max_trials=None):
        camera_args = ["--camera", camera] if camera else []
        key_args = [part for key in keys or [] for part in ("--key", key)]
        sequence = len(ledger["operations"])
        if action == "start":
            chosen = port if port is not None else self._free_port()
            reply = self._command(path, ledger, ["up", "--scene", str(path / "scene.py"), "--port", str(chosen),
                                               "--hold", "--no-watch"], timeout=330)
            if reply.get("already"):
                ledger["server"] = None
                return {"ok": False, "error": "RSS reported an existing server; the bridge will not adopt or stop it."}
            reply.update(port_requested=port, port_chosen=chosen,
                         port_note="Picked a free port." if port is None else "Port as requested; RSS takes the next free one if it is busy.",
                         network=("RSS's viewer listens on all network interfaces; RSS offers no localhost-only option "
                                  "(robosimstudio_studio/session.py: the viewer has no host option)."),
                         scene_frozen="The scene is frozen at this copy (RSS runs with --no-watch). Edit it with update_scene.")
            if ledger["device"] == "cuda:0":
                reply["gpu_note"] = plan.GPU_FIRST_RUN_NOTE
            if reply.get("ok"):
                status = self._command(path, ledger, ["status"], timeout=30)
                if not status.get("ok") or status.get("last_error"):
                    ledger["scene_versions"][-1]["loaded"] = False
                    return {"ok": False, "error": "Scene failed to load. Fix it with update_scene, or stop this run.", "status": status}
                ledger["scene_versions"][-1]["loaded"] = True
                reply["status"] = status
            return reply
        if action == "update_scene":
            return self._update_scene(path, ledger, scene, scene_source, template)
        if action == "stop":
            if not self._owned_alive(ledger):
                return {"ok": True, "stopped": False, "final_state": "not running",
                        "message": "Owned server is no longer alive; no process was signalled."}
            pid = ledger["server"]["pid"]
            started = time.monotonic()
            reply = self._command(path, ledger, ["quit"], timeout=30)
            while self._owned_alive(ledger) and time.monotonic() - started < STOP_WAIT_S:
                time.sleep(0.1)
            reply["stopped"] = not self._owned_alive(ledger)
            reply["waited_s"] = round(time.monotonic() - started, 2)
            reply["final_state"] = (f"process {pid} has exited" if reply["stopped"]
                                    else f"process {pid} still alive after {STOP_WAIT_S:g} s")
            if not reply["stopped"]:
                reply.update(ok=False, error=f"Owned server still alive {STOP_WAIT_S:g} s after quit; no process was force-killed.")
            return reply
        if action == "reference":
            return self._reference(path, ledger, video, at, width, start_s, end_s, sequence)
        if action == "snapshot":
            return self._command(path, ledger, ["snapshot", *camera_args, "--size", size,
                                               "--out", str(path / f"snapshot-{sequence}.png")])
        if action == "compare":
            reply = self._command(path, ledger, ["compare", *camera_args, *key_args, "--size", size,
                                               "--out", str(path / f"comparison-{sequence}.png")])
            reply["objects"] = {name: plan.relabel_ratios(reading) for name, reading in (reply.get("objects") or {}).items()}
            reply["metrics"] = reply["objects"]
            reply["ratio_note"] = ("area_ratio = sim pixels / photo pixels. linear_scale_ratio is RSS's size_ratio, "
                                   "the square root of the area ratio; RSS's report text calls it 'size'.")
            reply["scope"] = plan.engine(ledger["device"])["scope"]
            return reply
        if action == "compare_take":
            return self._compare_take(path, ledger, take, keys, key_args, camera_args, start_s, end_s,
                                      sim_start_s, sim_end_s, sequence)
        if action == "rollout":
            return self._rollout(path, ledger, take, routine, camera_args, push, duration_s, steps, record_fps)
        if action == "tune_push":
            return self._tune_push(path, ledger, take, keys, key_args, camera_args, push, duration_s, record_fps,
                                   start_s, end_s, sim_start_s, sim_end_s, max_trials, sequence)
        if action == "report":
            return self._report(path, ledger)
        raise ValueError(f"Unsupported action {action}")

    def _update_scene(self, path, ledger, scene, scene_source=None, template=None):
        before = self._command(path, ledger, ["status"], timeout=30)
        previous = (path / "scene.py").read_bytes()
        previous_inputs = ledger["inputs"]["scene"]
        entry = self._install_scene(path, ledger, scene, text=scene_source, template=template)
        reply = self._command(path, ledger, ["reload"], timeout=330)
        status = self._command(path, ledger, ["status"], timeout=30)
        generation = (before.get("generation"), status.get("generation"))
        loaded = bool(reply.get("ok") and status.get("ok") and not status.get("last_error")
                      and generation[0] is not None and generation[1] != generation[0])
        entry["loaded"] = loaded
        if not loaded:
            (path / "scene.py").write_bytes(previous)
            ledger["inputs"]["scene"] = previous_inputs
            return {"ok": False, "version": entry["version"], "versioned_file": entry["file"], "loaded": False,
                    "error": reply.get("error") or status.get("last_error") or "RSS did not rebuild the scene.",
                    "what_happened": (f"Saved your scene as {Path(entry['file']).name} and asked the running RSS server to reload it; "
                                      "the build failed. RSS keeps the previous scene when a build fails (server.py reload), and "
                                      "scene.py was put back to the previous version, so the run still has the old scene loaded."),
                    "status": status}
        return {"ok": True, "version": entry["version"], "versioned_file": entry["file"], "active_file": str(path / "scene.py"),
                "previous_file": str(path / f"scene-v{entry['version'] - 1}.py") if entry["version"] > 1 else None,
                "restarted": False, "reloaded": True, "generation": {"before": generation[0], "after": generation[1]},
                "load_seconds": reply.get("seconds"), "names": status.get("names"), "routines": status.get("routines"),
                "cameras": status.get("cameras"),
                "template": template, "mask_key": (template or {}).get("mask_key"),
                "what_happened": (f"Saved your scene as {Path(entry['file']).name} (older versions are kept) and copied it to scene.py. "
                                  "Asked the running RSS server (same process, not restarted) to reload; it rebuilt the scene in place. "
                                  "The physics restarted from the new scene's start state. Earlier takes, reference frames and the log "
                                  "are untouched; an open viewer tab keeps its connection.")}

    @staticmethod
    def _clip_info(ffmpeg, clip):
        """``(duration_s, fps)`` read from ffmpeg's header report; either may be None."""
        text = subprocess.run([ffmpeg, "-hide_banner", "-i", str(clip)], capture_output=True, text=True, timeout=60).stderr
        found = re.search(r"Duration:\s*(\d+):(\d+):(\d+(?:\.\d+)?)", text)
        rate = re.search(r"(\d+(?:\.\d+)?) fps", text)
        return (int(found[1]) * 3600 + int(found[2]) * 60 + float(found[3]) if found else None,
                float(rate[1]) if rate and float(rate[1]) > 0 else None)

    def _trim(self, source, destination, start, end):
        """Cut ``start``-``end`` seconds of ``source`` into ``destination`` with ffmpeg (RSS cannot trim)."""
        ffmpeg = _ffmpeg_exe()
        if not ffmpeg:
            raise ValueError("start_s/end_s cut the clip with ffmpeg, which was not found on PATH or in the imageio-ffmpeg package. Install ffmpeg, or leave them out "
                             "and pass `at` anchor times instead.")
        for codec in (["libx264", "-preset", "veryfast", "-crf", "16"], ["mpeg4", "-q:v", "2"]):
            result = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-ss", f"{start:.3f}", "-i", str(source),
                                     "-t", f"{end - start:.3f}", "-an", "-c:v", *codec, "-pix_fmt", "yuv420p", str(destination)],
                                    capture_output=True, text=True, timeout=300)
            if result.returncode == 0 and Path(destination).is_file():
                return
        raise ValueError("ffmpeg could not cut the clip: " + (result.stderr or "no output")[-400:])

    @staticmethod
    def _shift_at(at, offset, span):
        """Anchor times given in clip seconds, as seconds into the trimmed clip (``end`` stays ``end``)."""
        parts = []
        for item in at.split(","):
            when, colon, label = item.strip().partition(":")
            if when.strip().lower() != "end":
                try:
                    shifted = float(when) - offset
                except ValueError:
                    raise ValueError(f"at has an unreadable time {when!r}; use seconds like '0:start,3:middle,end'.") from None
                if shifted < -1e-6 or shifted > span + 1e-6:
                    raise ValueError(f"at time {float(when):g} s is outside the trimmed span {offset:g}-{offset + span:g} s.")
                when = f"{max(0.0, shifted):g}"
            parts.append(when.strip() + colon + label)
        return ",".join(parts)

    def _reference(self, path, ledger, video, at, width, start_s, end_s, sequence):
        suffix = Path(video or "").suffix
        if not re.fullmatch(r"\.[A-Za-z0-9]{1,10}", suffix):
            raise ValueError("video must be an absolute file path with a video extension.")
        windowed = start_s is not None or end_s is not None
        first = 0.0 if start_s is None else start_s
        copied = path / f"reference-{sequence}{suffix}"
        ledger["inputs"][f"reference-{sequence}"] = self._copy_input(video, copied)
        feed, offset, trimmed, source_duration, end_anchor = copied, 0.0, None, None, None
        if windowed:
            ffmpeg = _ffmpeg_exe()
            source_duration, fps = self._clip_info(ffmpeg, copied) if ffmpeg else (None, None)
            frame = 1.0 / fps if fps else 0.0
            last = end_s if end_s is not None else source_duration
            if source_duration is not None and last is not None and last > source_duration + 1e-6:
                return {"ok": False, "error": f"end_s {last:g} is beyond the clip's duration {source_duration:g} s."}
            if first > 0 or (end_s is not None and (source_duration is None or end_s < source_duration - 1e-6)):
                if last is None:
                    raise ValueError("Cannot read the clip's length, so end_s is needed to cut it.")
                # one frame past end_s goes into the cut so that the frame at end_s itself is its last
                cut_end = min(last + frame, source_duration) if source_duration is not None else last
                feed = path / f"reference-{sequence}-trim{suffix}"
                self._trim(copied, feed, first, cut_end)
                offset = first
                end_anchor = last if cut_end > last + 1e-6 else (cut_end - frame if frame else None)
                ledger["inputs"][f"reference-{sequence}-trim"] = {"source": str(copied), "copy": str(feed), "sha256": self._sha256(feed),
                                                                  "source_start_s": first, "source_end_s": cut_end, "tool": "ffmpeg"}
                trimmed = {"file": str(feed), "source_start_s": first, "source_end_s": last, "duration_s": round(last - first, 3),
                           "file_duration_s": round(cut_end - first, 3),
                           "note": (f"The clip was trimmed with ffmpeg to {first:g}-{last:g} s of the original and RSS was given that cut; the "
                                    "contact sheet and the reported duration cover that span (the file holds one extra frame so the frame at "
                                    f"{last:g} s is in it). Times you give and read stay in original clip seconds.")}
            elif source_duration is not None and frame:
                end_anchor = source_duration - frame
        span = (trimmed or {}).get("file_duration_s")
        if at is None:
            if not windowed:
                at = "0:start,end"
            elif end_s is None:
                at = f"{first:g}:start,{end_anchor:g}:end" if end_anchor is not None else f"{first:g}:start,end:end"
            else:
                top = end_anchor if end_anchor is not None else end_s
                at = f"{first:g}:start,{(first + top) / 2:g}:middle,{top:g}:end"
        if trimmed:
            at = self._shift_at(at, offset, span)
        reply = self._command(path, ledger, ["ref", "frames", str(feed), "--at", at, "--width", str(width), "--out", str(path / "ref")])
        if not reply.get("ok"):
            return reply
        duration = reply.get("duration")
        last = end_s if end_s is not None else (source_duration if trimmed else duration)
        if not trimmed and isinstance(duration, (int, float)) and last is not None and last > duration + 1e-6:
            return dict(reply, ok=False, error=f"end_s {last:g} is beyond the clip's duration {duration:g} s.")
        timeline = self._timeline(path)
        anchors = timeline.get("anchors", [])
        reply["window"] = {"start_s": first, "end_s": last} if windowed else None
        reply["anchors"] = [{"t": round(float(a["t"]) + offset, 3), "label": a.get("label"), "file": a.get("file")} for a in anchors]
        if trimmed:
            reply.update(trimmed=trimmed, duration=trimmed["duration_s"], source_duration_s=source_duration)
        reply["window_note"] = ("Times are seconds in the original clip (anchors, start_s, end_s). The clip was trimmed to the window, so "
                                "the contact sheet and duration cover the trimmed span." if trimmed else "Times are seconds in the clip.")
        ledger["reference"] = {"clip": str(feed), "duration_s": trimmed["duration_s"] if trimmed else duration, "fps": reply.get("fps"),
                               "start_s": start_s, "end_s": end_s, "offset_s": offset, "trimmed": bool(trimmed),
                               "source_duration_s": source_duration, "timeline": str(path / "ref/timeline.json")}
        return reply

    def _compare_take(self, path, ledger, take, keys, key_args, camera_args, start_s, end_s, sim_start_s, sim_end_s, sequence):
        timeline = self._timeline(path)
        known = ledger.get("reference") or {}
        offset = float(known.get("offset_s") or 0.0)
        anchors = [dict(a, t=round(float(a["t"]) + offset, 3)) for a in timeline.get("anchors") or []]
        if not anchors:
            raise ValueError("No reference anchors: run reference first.")
        try:
            meta = json.loads((path / f"{take}.json").read_text())
            take_duration = float(meta["frames"][-1]["t"])
        except (OSError, ValueError, KeyError, IndexError):
            raise ValueError(f"No readable take named {take!r}; run rollout first.") from None
        mapping = plan.time_mapping([float(a["t"]) for a in anchors], take_duration,
                                    clip_start=start_s if start_s is not None else known.get("start_s"),
                                    clip_end=end_s if end_s is not None else known.get("end_s"),
                                    sim_start=sim_start_s, sim_end=sim_end_s)
        mapped = dict(timeline, anchors=[dict(a, sim_t=mapping["sim_t"][float(a["t"])]) for a in anchors
                                         if float(a["t"]) in mapping["sim_t"]])
        mapped_file = path / "ref" / f"timeline-map-{sequence}.json"
        mapped_file.write_text(json.dumps(mapped, indent=1) + "\n")
        reply = self._command(path, ledger, ["compare", "--take", take, *key_args, "--timeline", str(mapped_file), *camera_args])
        if not reply.get("ok"):
            return reply
        rows = reply.get("anchors") or []
        if keys and rows and not any(row.get("objects") for row in rows):
            scored = self._take_scores(path, ledger, take, keys, rows, sequence)
            if not scored.get("ok"):
                return dict(reply, ok=False, error=f"The take's masks could not be read: {scored.get('error')}")
        for row in rows:
            row["objects"] = {name: plan.relabel_ratios(reading) for name, reading in (row.get("objects") or {}).items()}
        names = [key.split("=", 1)[0] for key in keys]
        displacement = {name: plan.centroid_displacement(rows[0], rows[-1], name) for name in names} if len(rows) >= 2 else {}
        reply["anchors"] = rows
        reply["displacement_px"] = displacement
        self._verdict(reply, rows, displacement)
        reply["time_mapping"] = {**{k: v for k, v in mapping.items() if k not in ("sim_t", "warnings")},
                                 "timeline": str(mapped_file),
                                 "anchors": [{"clip_t": row.get("t"), "sim_t_requested": mapping["sim_t"].get(float(row["t"])),
                                              "sim_t_used": row.get("sim_t"), "sim_step": row.get("sim_step")} for row in rows]}
        reply["warnings"] = mapping["warnings"]
        reply["ratio_note"] = "area_ratio = sim pixels / photo pixels; linear_scale_ratio = RSS size_ratio = sqrt(area_ratio)."
        reply["scope"] = plan.engine(ledger["device"])["scope"]
        return reply

    def _take_scores(self, path, ledger, take, keys, rows, sequence):
        """Fill each anchor's ``objects`` from the take's segmentation and the anchor's photograph.

        RSS's ``compare --take`` at the pin returns only the frame each anchor was held against (its keys outline things
        on the strip and nothing more), so ``take_scores.py`` runs RSS's own still-compare functions on that frame in
        RSS's interpreter. Rows that already carry ``objects`` (a later RSS) are left as RSS gave them."""
        if any("frame" not in row or "file" not in row for row in rows):
            return {"ok": False, "error": "RSS's take reply has no frame or file per anchor."}
        request = path / f"take-scores-{sequence}.json"     # one per compare, as timeline-map-N.json
        request.write_text(json.dumps({"run_dir": str(path), "take": take, "keys": list(keys),
                                       "anchors": [{"file": row["file"], "frame": row["frame"]} for row in rows]}) + "\n")
        script = Path(__file__).with_name("take_scores.py")
        reply = self._command(path, ledger, [], timeout=300,
                              argv=[str(self.root / ".venv/bin/python"), "-I", str(script), str(request)])
        scored = reply.get("anchors") if reply.get("ok") else None
        if not isinstance(scored, list) or len(scored) != len(rows):
            record = ledger["commands"][-1]
            stderr = Path(record["stderr"]).read_text(errors="replace").strip()[-500:]
            if record.get("returncode"):     # the script's own refusal or traceback
                return {"ok": False, "error": stderr or reply.get("error") or "take_scores.py failed."}
            return {"ok": False, "error": "take_scores.py gave no reading per anchor." + (f" stderr: {stderr}" if stderr else "")}
        for row, found in zip(rows, scored):
            row["objects"] = found.get("objects") or {}
        return {"ok": True}

    @staticmethod
    def _verdict(reply, rows, displacement):
        """Put the bridge's own verdict at ``verdict``; RSS's score and verdict stay, labelled as RSS's."""
        reply["rss_score"], reply["rss_verdict"] = reply.pop("score", None), reply.pop("verdict", None)
        reply["rss_note"] = ("rss_score and rss_verdict are RSS's own: its overlap score and a did-it-move count. They do not "
                             "measure how far the block moved; the verdict above them is the bridge's, with its thresholds.")
        by_object = {}
        for name, moved in displacement.items():
            overlap = ((rows[-1].get("objects") or {}).get(name) or {}).get("iou")
            by_object[name] = plan.bridge_verdict(moved, overlap)
        overall = plan.overall_verdict(by_object)
        reply["verdict"] = overall
        reply["bridge_verdict"] = {"verdict": overall, "by_object": by_object, "thresholds": plan.VERDICT_THRESHOLDS,
                                   "source": "the bridge's own check on image-centroid displacement and end-frame overlap, not RSS's"}

    def _rollout(self, path, ledger, take, routine, camera_args, push, duration_s, steps, record_fps):
        if push is not None:
            push, push_warnings = plan.validate_push(push)
        else:
            push_warnings = []
        hz_reply = self._command(path, ledger, ["eval", "float(sim.request.physics.control_hz)"], timeout=60)
        try:
            hz = float(hz_reply["value"])
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "error": "Could not read the scene's control rate from RSS.", "reply": hz_reply}
        asked = plan.rollout_plan(hz, duration_s=duration_s, steps=steps, record_fps=record_fps, push=push)
        literal = repr(json.loads(json.dumps({"steps": asked["steps"], "take": take, "push": push})))
        set_reply = self._command(path, ledger, ["eval", "ROLLOUT = " + literal], timeout=60)
        if not set_reply.get("ok"):
            return {"ok": False, "error": "RSS refused the rollout settings.", "reply": set_reply}
        before = self._command(path, ledger, ["status"], timeout=30)
        reset = self._command(path, ledger, ["reset"], timeout=30)
        deadline = time.monotonic() + 60
        while reset.get("ok") and time.monotonic() < deadline:
            status = self._command(path, ledger, ["status"], timeout=30)
            if status.get("resets", 0) > before.get("resets", 0) and not status.get("playing"):
                break
            time.sleep(0.5)
        play = self._command(path, ledger, ["play", *([routine] if routine else []), "--record", take,
                                          "--every", str(asked["every"]), *camera_args], timeout=330)
        if not play.get("ok"):
            return play
        idle = self._poll_idle(path, ledger, 1800)
        if not idle["ok"]:
            return idle
        status = idle["status"]
        outputs = {"mp4": str(path / f"{take}.mp4"), "json": str(path / f"{take}.json"), "seg_npz": str(path / f"{take}_seg.npz")}
        valid = status.get("last_take") == take and all(Path(outputs[key]).is_file() for key in ("mp4", "json"))
        if not valid:
            return {"ok": False, "take": take, "status": status, **outputs,
                    "error": "Routine ended without the requested take files."}
        meta = json.loads(Path(outputs["json"]).read_text())
        frames = meta.get("frames") or []
        executed = int(frames[-1]["step"]) if frames else 0
        seconds = float(frames[-1]["t"]) if frames else 0.0
        warnings = asked["warnings"] + push_warnings
        applied = None
        try:
            applied = json.loads((path / f"push-{take}.json").read_text())
        except (OSError, ValueError):
            pass
        untracked = plan.untracked_message(applied)
        if untracked:
            warnings.insert(0, untracked)
        result = {**({"headline": untracked, "pusher_tracked": False} if untracked else {}), "ok": True,
                  "take": take, "routine": play.get("playing"), **outputs,
                  "steps_asked": asked["steps"], "steps": executed, "duration_s": seconds,
                  "control_hz": meta.get("control_hz"), "every": meta.get("every"),
                  "record_fps_asked": asked["record_fps_asked"], "record_fps": asked["record_fps"],
                  "video_fps": meta.get("fps"), "frames": len(frames), "frames_expected": asked["frames_expected"],
                  "push": push, "push_applied": applied, "warnings": warnings,
                  "status": {key: value for key, value in status.items() if key != "board"},
                  "board_note": ("RSS's own board verdict at the end of a take maps the clip proportionally; "
                                 "use compare_take with an explicit time mapping instead."),
                  "engine": plan.engine(ledger["device"])}
        if executed != asked["steps"]:
            warnings.append(f"The routine ran {executed} steps but {asked['steps']} were asked; a routine that does not read "
                            "ROLLOUT['steps'] sets its own length.")
        if push is not None and not (applied and applied.get("applied")):
            result.update(ok=False, error="The routine did not apply push: it wrote no push result. Use the template's "
                          "'push' routine (rollout routine='push'), or read ROLLOUT['push'] in your own routine.")
        return result

    def _starting_push(self, path, ledger, push):
        """The push tune_push starts from: the one given, the loaded template's, else the scene's own ``PUSH``."""
        if push is not None:
            return push
        template = self._current_template(ledger)
        if template:
            return template["push"]
        reply = self._command(path, ledger, ["eval", "PUSH"], timeout=60)
        try:
            found = ast.literal_eval(reply["value"])
            return plan.validate_push(found)[0]
        except (KeyError, TypeError, ValueError, SyntaxError):
            raise ValueError("Give push={'waypoints': [[seconds, x, y], ...]}: the scene has no readable PUSH to start from.") from None

    def _tune_push(self, path, ledger, take, keys, key_args, camera_args, push, duration_s, record_fps,
                   start_s, end_s, sim_start_s, sim_end_s, max_trials, sequence):
        known = ledger.get("reference")
        if not known or not (path / "ref/timeline.json").is_file():
            raise ValueError("tune_push needs reference frames: run reference first.")
        name = keys[0].split("=", 1)[0]
        start_push = self._starting_push(path, ledger, push)
        template = self._current_template(ledger)
        eye, target = ((template["camera"]["eye"], template["camera"]["target"]) if template
                       else (scene_template_module.DEFAULT_CAMERA["eye"], scene_template_module.DEFAULT_CAMERA["target"]))
        axes = plan.image_axes(eye, target)
        if duration_s is None:
            anchors = [float(a["t"]) + float(known.get("offset_s") or 0.0) for a in self._timeline(path).get("anchors", [])]
            first = start_s if start_s is not None else (known.get("start_s") if known.get("start_s") is not None else min(anchors))
            last = end_s if end_s is not None else (known.get("end_s") if known.get("end_s") is not None else max(anchors))
            duration_s = last - first
        takes = []

        def evaluate(trial_push, number):
            trial_take = f"{take}-{number}"
            rolled = self._rollout(path, ledger, trial_take, None, camera_args, trial_push, duration_s, None, record_fps)
            if not rolled.get("ok"):
                raise ValueError(f"trial {number}: rollout failed: {rolled.get('error')}")
            compared = self._compare_take(path, ledger, trial_take, keys, key_args, camera_args, start_s, end_s,
                                          sim_start_s, sim_end_s, f"{sequence}-{number}")
            if not compared.get("ok"):
                raise ValueError(f"trial {number}: compare_take failed: {compared.get('error')}")
            moved = (compared.get("displacement_px") or {}).get(name) or {}
            if moved.get("clip_px") is None or moved.get("sim_px") is None:
                raise ValueError(f"trial {number}: the mask for {name!r} was not found at the first and last anchors; "
                                 "check keys with compare.")
            rows = compared["anchors"]
            iou = lambda row: ((row.get("objects") or {}).get(name) or {}).get("iou")
            takes.append(trial_take)
            applied = rolled.get("push_applied") or {}
            return {"clip_px": moved["clip_px"], "sim_px": moved["sim_px"], "start_overlap": iou(rows[0]), "end_overlap": iou(rows[-1]),
                    "take": trial_take, "video": rolled.get("mp4"), "bridge_verdict": compared.get("verdict"),
                    "pivot": applied.get("object_start_xy_m"), "warnings": rolled.get("warnings"),
                    **({"pusher_tracked": False, "pusher_message": rolled.get("headline")} if rolled.get("pusher_tracked") is False else {})}

        initial_pivot = template["position"] if template else None
        trials, best, stopped = plan.search_push(start_push, initial_pivot, axes, evaluate, max_trials=max_trials)
        best_push = {"waypoints": best["push"]["waypoints"], "object_velocity": best["push"].get("object_velocity")}
        within = stopped == "within tolerance"
        low, high = plan.TUNE_RATIO_OK
        untracked = stopped == "pusher did not track"
        failure = ({"headline": trials[-1]["pusher_message"], "ok": False, "pusher_tracked": False,
                    "error": trials[-1]["pusher_message"]} if untracked else {"ok": True})
        return {**failure, "object": name, "trials": [{k: v for k, v in row.items() if k != "warnings"} for row in trials],
                "best_trial": best["trial"], "best_push": best_push, "within_tolerance": within, "stopped_because": stopped,
                "tolerance": {"magnitude_ratio": [low, high], "direction_error_deg_below": plan.TUNE_DIRECTION_OK_DEG},
                "clip_px": best["clip_px"], "best_sim_px": best["sim_px"], "best_magnitude_ratio": best["magnitude_ratio"],
                "best_direction_error_deg": best["direction_error_deg"], "takes": takes, "duration_s": duration_s,
                "scope": plan.TUNE_SCOPE,
                "use": {"rollout": "rollout(run, take=NEW, push=best_push, duration_s=..., record_fps=10), then compare_take",
                        "update_scene": "update_scene(run, template={'push': best_push}) writes it into the scene"},
                "note": (f"Stopped after trial {len(trials)} of {max_trials}: the pusher did not follow its path, so the trials say "
                         "nothing about the push. Fix the simulation setup (see headline) and start again." if untracked else
                         "Each trial is a full rollout plus the comparison, on the take named <take>-<n>. The search rescales the push about the "
                         "object's start and turns it by the measured direction error; it stops early within tolerance." if within else
                         f"Not within tolerance after {len(trials)} trials; best_push is the best seen (trial {best['trial']}). "
                         "Run tune_push again from it (push=best_push) or change the scene; this tunes the push path only.")}

    def _report(self, path, ledger):
        report = path / "REPORT.md"
        engine = plan.engine(ledger["device"])
        lines = ["# RSS image-space comparison", "",
                 "Compares images. It does not fit mass or friction; there is no automatic estimation of either.",
                 "", f"RSS commit: `{ledger['rss_commit']}` (tested pin `{PIN}`). Device: `{ledger['device']}`.",
                 f"Physics engine that ran: {engine['physics']}.", ""]
        if ledger.get("scene_versions"):
            lines += ["## Scene versions", ""]
            lines += [f"- v{v['version']}: `{Path(v['file']).name}` sha256 `{v['sha256'][:12]}`, loaded: {v.get('loaded')}"
                      for v in ledger["scene_versions"]]
            lines.append("")
        lines += self._summary(ledger)
        lines += ["## Inputs", "", "```json", json.dumps(ledger["inputs"], indent=2), "```", "", "## Operations"]
        for previous in ledger["operations"][:-1]:
            lines += ["", f"### {previous['action']} — {previous['state']}",
                      f"Wall time: {previous.get('wall_s', '?')} seconds. Started: {previous['started_at']}.",
                      "", "```json", json.dumps(previous.get("result", {}), indent=2), "```"]
        lines += ["", "## Media", *[f"- `{file}`" for file in sorted(path.rglob("*"))
                                     if file.suffix in (".png", ".jpg", ".mp4", ".webm")]]
        report.write_text("\n".join(lines) + "\n")
        return {"ok": True, "report": str(report)}

    @staticmethod
    def _summary(ledger):
        lines = []
        for operation in ledger["operations"][:-1]:
            result = operation.get("result") or {}
            if operation["action"] == "rollout" and result.get("take"):
                lines.append(f"- rollout `{result['take']}`: {result.get('steps')} steps = {result.get('duration_s')} s simulated at "
                             f"{result.get('control_hz')} Hz, {result.get('frames')} frames at {result.get('record_fps')} per simulated second"
                             f"{'' if operation['state'] == 'completed' else ' (failed: ' + str(result.get('error')) + ')'}"
                             f"{', solver ' + str((result.get('push_applied') or {}).get('solver')) if (result.get('push_applied') or {}).get('solver') else ''}"
                             f"{'. ' + result['headline'].rstrip('.') if result.get('headline') else ''}.")
            if operation["action"] == "tune_push" and result.get("trials"):
                if result.get("headline"):
                    lines.append(f"- tune_push stopped early: {result['headline']}")
                lines.append(f"- tune_push `{result.get('object')}`: {len(result['trials'])} trials, {result.get('stopped_because')}; best trial "
                             f"{result.get('best_trial')} gave sim {result.get('best_sim_px')} px against the clip's {result.get('clip_px')} px "
                             f"(ratio {result.get('best_magnitude_ratio')}, direction error {result.get('best_direction_error_deg')} degrees). "
                             "It tunes the push path, not mass or friction.")
                lines += [f"  - trial {t['trial']}: scale {t['scale']}, rotation {t['rotation_deg']} deg, sim {t['sim_px']} px, "
                          f"ratio {t['magnitude_ratio']}, direction error {t['direction_error_deg']} deg" for t in result["trials"]]
            if operation["action"] == "compare_take" and result.get("verdict"):
                mapping = result.get("time_mapping") or {}
                lines.append(f"- compare_take `{result.get('take')}`: bridge verdict **{result.get('verdict')}** "
                             f"(RSS's own score {result.get('rss_score')}, RSS's verdict: {result.get('rss_verdict')}). Clip "
                             f"{mapping.get('clip_start_s')}–{mapping.get('clip_end_s')} s against sim "
                             f"{mapping.get('sim_start_s')}–{mapping.get('sim_end_s')} s.")
                for name, moved in (result.get("displacement_px") or {}).items():
                    lines.append(f"  - {name}: centroid displacement clip {moved['clip_px']} px, sim {moved['sim_px']} px, "
                                 f"difference {moved['difference_px']} px; magnitude ratio {moved['magnitude_ratio']}, "
                                 f"direction error {moved['direction_error_deg']} degrees.")
                for name, found in ((result.get("bridge_verdict") or {}).get("by_object") or {}).items():
                    lines += [f"  - {name} fails: {text}" for text in found["failed"]]
                lines += [f"  - warning: {text}" for text in result.get("warnings") or []]
        return ["## Summary", "", *lines, ""] if lines else []
