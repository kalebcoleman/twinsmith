"""sim.simulation.run: the caller's own controller in the physics loop, as a job.

Every physics step the controller gets the body's measured state and answers with the force and
torque to apply for that step. The controller is either a Python callable loaded into Kit (opt-in,
it is arbitrary code) or a process on this machine speaking one JSON line per step over TCP, which
runs no code in Kit.

Two ways to advance physics. ``mode="step"`` (the default) steps it exactly, a few steps per app
frame, from a paused timeline: repeatable, and as fast as the machine allows. ``mode="play"`` lets
the timeline play and calls the controller from the engine's own pre-step callback, so everything
else that runs on playback (ROS publishers, action graphs, other robots' controllers) runs too.
"""

import csv
import importlib
import importlib.util
import json
import math
import os
import re
import socket
import time
from pathlib import Path

from twinsmith.project_assets import AssetError, refuse
from twinsmith_bridge.jobs import JobError

from . import physics

MAX_DURATION_S = 3600.0
MAX_STEPS = 2_000_000
MAX_STEPS_PER_FRAME = 120
TCP_TIMEOUT_S = 5.0
READY_FRAMES = 60
MAX_OBSERVED = 8
MAX_VIDEO_FRAMES = 3600
LOG_NAME = r"[A-Za-z0-9][A-Za-z0-9_-]{0,59}\.csv"
LOG_COLUMNS = {
    "position": ("px", "py", "pz"),
    "orientation": ("qw", "qx", "qy", "qz"),
    "linear_velocity": ("vx", "vy", "vz"),
    "angular_velocity": ("wx", "wy", "wz"),
    "force": ("fx", "fy", "fz"),
    "torque": ("tx", "ty", "tz"),
}
DEFAULT_LOG = ("position", "orientation", "linear_velocity", "angular_velocity", "force", "torque")
SCRIPTS_ENV = "TWINSMITH_ENABLE_SCENE_SCRIPTS"
_PYTHON_SPEC = re.compile(r"(?P<source>[^:]+):(?P<name>[A-Za-z_][A-Za-z0-9_]*)")
_TCP_SPEC = re.compile(r"tcp://(?P<host>[A-Za-z0-9_.-]+|\[[0-9A-Fa-f:]+\]):(?P<port>\d{1,5})")
LOOPBACK = ("127.0.0.1", "localhost", "::1", "[::1]")


def parse_controller(spec):
    """("tcp", host, port) or ("python", module-or-file, function); refuses anything else."""
    if not isinstance(spec, str) or not spec or len(spec) > 1024:
        refuse("INVALID_VALUE", "controller must be 'tcp://127.0.0.1:<port>', 'package.module:function' or "
               "'/abs/path/file.py:function'.")
    tcp = _TCP_SPEC.fullmatch(spec)
    if tcp:
        port = int(tcp["port"])
        if not 1 <= port <= 65535:
            refuse("INVALID_VALUE", "The controller port must be 1..65535.")
        if tcp["host"] not in LOOPBACK and os.getenv(SCRIPTS_ENV) != "1":
            refuse("CONTROLLER_NOT_ALLOWED", "A network controller must listen on this machine (tcp://127.0.0.1:<port>). "
                   f"Another host needs {SCRIPTS_ENV}=1 in Kit's launch environment. Nothing ran.")
        return "tcp", tcp["host"].strip("[]"), port
    python = _PYTHON_SPEC.fullmatch(spec)
    if not python:
        refuse("INVALID_VALUE", "controller must be 'tcp://127.0.0.1:<port>', 'package.module:function' or "
               "'/abs/path/file.py:function'.")
    return "python", python["source"], python["name"]


def load_python(source, name):
    """Import the callable. Opt-in: this runs the caller's code inside Kit, like sim__controller__create."""
    if os.getenv(SCRIPTS_ENV) != "1":
        refuse("CONTROLLER_NOT_ALLOWED",
               f"A Python controller runs your code inside Kit, which needs {SCRIPTS_ENV}=1 in Kit's launch "
               "environment (the same opt-in as sim__controller__create). Without it, run the controller as its own "
               "process and pass controller='tcp://127.0.0.1:<port>': twinsmith.controller_server.serve(fn, port) "
               "is a ready-made server, and run_script starts it as a managed job. Nothing ran.")
    try:
        if source.endswith(".py"):
            file = Path(os.path.expanduser(source))
            if not file.is_absolute() or not file.is_file():
                refuse("NOT_FOUND", f"{source} is not an existing absolute .py path on Kit's machine.")
            spec = importlib.util.spec_from_file_location("twinsmith_controller_" + file.stem, str(file))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        else:
            module = importlib.import_module(source)
    except AssetError:
        raise
    except Exception as exc:  # noqa: BLE001 - the caller's module failed to import
        refuse("CONTROLLER_FAILED", f"Importing {source} failed: {type(exc).__name__}: {str(exc)[:600]}. Nothing ran.")
    function = getattr(module, name, None)
    if not callable(function):
        refuse("NOT_FOUND", f"{source} has no callable named {name}.")
    return function


class TcpController:
    """One JSON object per line each way: the state out, the command back."""

    def __init__(self, host, port):
        try:
            self.sock = socket.create_connection((host, port), timeout=TCP_TIMEOUT_S)
        except OSError as exc:
            refuse("CONTROLLER_UNREACHABLE", f"No controller answers on {host}:{port} ({exc}). Start it first "
                   "(twinsmith.controller_server.serve, e.g. through run_script), then call again. Nothing ran.")
        self.sock.settimeout(TCP_TIMEOUT_S)
        self.reader = self.sock.makefile("rb")

    def __call__(self, state):
        self.sock.sendall(json.dumps(state, separators=(",", ":")).encode() + b"\n")
        line = self.reader.readline(65536)
        if not line:
            raise ConnectionError("the controller closed the connection")
        return json.loads(line)

    def close(self):
        for closing in (self.reader, self.sock):
            try:
                closing.close()
            except OSError:
                pass


def _vec3(value, label):
    if value is None:
        return (0.0, 0.0, 0.0)
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
        raise ValueError(f"{label} must be three finite numbers, not {value!r}")
    return tuple(float(v) for v in value)


def _numbers_by_name(value, label, names):
    """{joint name: number} from a controller, checked against the articulation's own joint names."""
    if value is None:
        return {}
    if not isinstance(value, dict) or not all(type(v) in (int, float) and math.isfinite(v) for v in value.values()):
        raise ValueError(f"{label} must map joint names to finite numbers")
    if names is None:
        raise ValueError(f"{label} needs the run's articulation argument")
    unknown = [name for name in value if name not in names]
    if unknown:
        raise ValueError(f"{label} names unknown joint(s) {unknown[:4]}; this articulation has {names[:12]}")
    return {names.index(name): float(v) for name, v in value.items()}


def check_command(command, default_frame, bodies=(), joints=None):
    """What a controller returned, checked: the wrench on the main body, wrenches on other bodies of
    the run, joint efforts and targets, done and its own log numbers. None means nothing to apply."""
    empty = dict(force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0), frame=default_frame, done=False, extra={},
                 wrenches={}, efforts=None, targets={}, velocity_targets={})
    if command is None:
        return empty
    if not isinstance(command, dict):
        raise ValueError(f"a controller returns a dict such as {{'force': [0, 0, 9.81]}} or None, not {type(command).__name__}")
    frame = command.get("frame", default_frame)
    if frame not in ("world", "body"):
        raise ValueError("frame must be world or body")
    extra = command.get("log") or {}
    if not isinstance(extra, dict) or len(extra) > 16 or not all(
            isinstance(k, str) and type(v) in (int, float) for k, v in extra.items()):
        raise ValueError("log must be a dict of at most 16 numbers")
    wrenches = {}
    given = command.get("wrenches") or {}
    if not isinstance(given, dict):
        raise ValueError("wrenches must map body paths to {'force': [...], 'torque': [...]}")
    for path, wrench in given.items():
        if path not in bodies or not isinstance(wrench, dict):
            raise ValueError(f"wrenches must map a body of this run ({', '.join(bodies) or 'none besides body'}) to "
                             "{'force': [...], 'torque': [...]}")
        local = wrench.get("frame", frame)
        if local not in ("world", "body"):
            raise ValueError("frame must be world or body")
        wrenches[path] = (_vec3(wrench.get("force"), "force"), _vec3(wrench.get("torque"), "torque"), local)
    efforts = command.get("joint_efforts")
    return dict(force=_vec3(command.get("force"), "force"), torque=_vec3(command.get("torque"), "torque"), frame=frame,
                done=bool(command.get("done", False)), extra=extra, wrenches=wrenches,
                efforts=None if efforts is None else _numbers_by_name(efforts, "joint_efforts", joints),
                targets=_numbers_by_name(command.get("joint_targets"), "joint_targets", joints),
                velocity_targets=_numbers_by_name(command.get("joint_velocity_targets"), "joint_velocity_targets", joints))


def log_fields(log):
    if log is None:
        return list(DEFAULT_LOG)
    if not isinstance(log, list) or not all(isinstance(f, str) for f in log):
        refuse("INVALID_VALUE", "log must be a list of fields: " + ", ".join(LOG_COLUMNS) + ".")
    unknown = [f for f in log if f not in LOG_COLUMNS]
    if unknown:
        refuse("INVALID_VALUE", f"Unknown log field(s) {unknown}; choose from " + ", ".join(LOG_COLUMNS) + ".")
    return list(dict.fromkeys(log))


def rotate(q_xyzw, v):
    """Rotate vector v by the unit quaternion [x, y, z, w]."""
    x, y, z, w = q_xyzw
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)]


class BodyIO:
    """Measured state of, and wrench onto, one rigid body through the selected engine's tensor view."""

    def __init__(self, path):
        self.path = path
        self.view = physics.view_for(path)
        self.backend = physics.manager().get_active_physics_engine()
        sample = self.view.get_transforms()
        self._module = type(sample).__module__
        self._device = getattr(sample, "device", None)
        masses = physics.array(self.view.get_masses()).reshape(-1).tolist()
        self.mass = float(masses[0]) if masses else None
        # PhysX applies a force at the body's origin unless told where; Newton applies it at the
        # centre of mass. So that "force" means the same thing on both, PhysX is given the centre of mass.
        self.com = None
        if self.backend == "physx":
            try:
                com = physics.array(self.view.get_coms()).tolist()[0][:3]
                if len(com) == 3 and all(math.isfinite(v) for v in com) and any(abs(v) > 1e-9 for v in com):
                    self.com = [float(v) for v in com]
            except Exception:  # noqa: BLE001 - a view without it applies at the origin, as documented
                self.com = None
        self._pose = None

    def read(self):
        transform = physics.array(self.view.get_transforms()).tolist()[0]
        velocity = physics.array(self.view.get_velocities()).tolist()[0]
        if len(transform) != 7 or len(velocity) != 6:
            refuse("PHYSICS_FORMAT", "Expected a 7-component transform and a 6-component velocity.")
        position = physics.runtime_vector(transform[:3], 3, path=self.path, field="world_position", backend=self.backend)
        q = physics.runtime_vector(transform[3:], 4, path=self.path, field="orientation_xyzw", backend=self.backend)
        physics.runtime_vector(velocity, 6, path=self.path, field="velocities", backend=self.backend)
        self._pose = (list(position), list(q))
        return dict(position=list(position), orientation_wxyz=[q[3], q[0], q[1], q[2]],
                    linear_velocity=velocity[:3], angular_velocity=velocity[3:], mass=self.mass)

    def _data(self, row, integer=False):
        import numpy as np

        data = np.array(row, dtype=np.uint32 if integer else np.float32)
        if self._module.startswith("warp"):
            import warp as wp

            return wp.array(data, dtype=wp.uint32 if integer else wp.float32, device=self._device)
        if self._module.startswith("torch"):
            import torch

            data = data.astype(np.int32) if integer else data
            return torch.from_numpy(data).to(device=self._device)
        return data

    def apply(self, force, torque, frame, at=None):
        """Apply a wrench for the next physics step only: at the centre of mass, or at the world position *at* (world frame)."""
        if not any(force) and not any(torque):
            return
        apply = getattr(self.view, "apply_forces_and_torques_at_position", None)
        if not callable(apply):
            refuse("WRENCH_UNSUPPORTED", f"The {self.backend} rigid-body view on this build cannot apply forces; "
                   "no fallback was used.")
        if at is not None:
            if frame != "world":
                refuse("INVALID_VALUE", "A world position needs a world-frame wrench.")
            apply(self._data([list(force)]), self._data([list(torque)]), self._data([list(at)]),
                  self._data([0], integer=True), True)
            return
        at = None
        if self.com is not None and any(force):
            at = list(self.com)  # in the body frame
            if frame == "world":
                if self._pose is None:
                    self.read()
                position, q = self._pose
                at = [p + r for p, r in zip(position, rotate(q, self.com))]
        apply(self._data([list(force)]), self._data([list(torque)]), None if at is None else self._data([at]),
              self._data([0], integer=True), frame == "world")


class JointIO:
    """Joint state of, and efforts and drive targets onto, one articulation through its tensor view."""

    def __init__(self, path):
        self.path = path
        self.view = physics.view_for(path, articulation=True)
        self.backend = physics.manager().get_active_physics_engine()
        self.names = physics.dof_names(self.view)
        self.wrote_efforts = False

    def read(self):
        positions = physics.array(self.view.get_dof_positions()).tolist()[0]
        velocities = physics.array(self.view.get_dof_velocities()).tolist()[0]
        for field, values in (("joint_positions", positions), ("joint_velocities", velocities)):
            physics.runtime_vector(values, len(self.names), path=self.path, field=field, backend=self.backend)
        return dict(names=self.names, positions=positions, velocities=velocities)

    def apply(self, efforts, targets, velocity_targets):
        if efforts is not None:
            # The whole row every step: a joint the controller leaves out gets no effort, not last step's.
            row = physics.dof_row(self.view, self.view.get_dof_positions()) * 0
            for index, value in efforts.items():
                row[0, index] = value
            physics.dof_write(self.view, "set_dof_actuation_forces", row)
            self.wrote_efforts = True
        if targets:
            physics.dof_set(self.view, "get_dof_position_targets", "set_dof_position_targets", targets)
        if velocity_targets:
            physics.dof_set(self.view, "get_dof_velocity_targets", "set_dof_velocity_targets", velocity_targets)

    def release(self):
        """Leave no effort behind when the run ends."""
        if self.wrote_efforts:
            try:
                physics.dof_write(self.view, "set_dof_actuation_forces",
                                  physics.dof_row(self.view, self.view.get_dof_positions()) * 0)
            except Exception:  # noqa: BLE001 - the view may be gone with the stage
                pass


class CsvLog:
    """One row per sample, in a new file in the output folder; never overwrites."""

    def __init__(self, folder, filename, fields, joint_names=()):
        self.joint_names = list(joint_names)
        if not isinstance(filename, str) or not re.fullmatch(LOG_NAME, filename):
            refuse("INVALID_OUTPUT", "filename: a new name of letters, digits, _ and -, ending .csv (<=64 characters).")
        folder.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.path = folder / filename
        try:
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        except FileExistsError:
            refuse("CONFLICT", f"{filename} already exists in the output folder; use another filename. No overwrite.")
        self.stream = os.fdopen(fd, "w", newline="")
        self.fields = fields
        self.extra = None
        self.writer = csv.writer(self.stream)
        self.rows = 0

    def write(self, step, time_s, state, force, torque, extra):
        if self.extra is None:  # the controller's own columns are fixed by its first answer
            self.extra = sorted(extra)
            self.writer.writerow(["step", "time_s", *(c for f in self.fields for c in LOG_COLUMNS[f]),
                                  *(f"q:{n}" for n in self.joint_names), *(f"dq:{n}" for n in self.joint_names), *self.extra])
        values = dict(position=state["position"], orientation=state["orientation_wxyz"],
                      linear_velocity=state["linear_velocity"], angular_velocity=state["angular_velocity"],
                      force=force, torque=torque)
        joints = state.get("joints") or {}
        self.writer.writerow([step, f"{time_s:.9g}", *(f"{v:.9g}" for f in self.fields for v in values[f]),
                              *(f"{v:.9g}" for key in ("positions", "velocities") if self.joint_names for v in joints[key]),
                              *(f"{extra.get(k, float('nan')):.9g}" for k in self.extra)])
        self.rows += 1

    def close(self):
        try:
            self.stream.close()
        except OSError:
            pass


def validate(args):
    """Check the call's own values before anything is started. Returns the cleaned settings."""
    duration = args.get("duration_s")
    if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 < duration <= MAX_DURATION_S:
        refuse("INVALID_VALUE", f"duration_s must be a number of simulated seconds, more than 0 and at most {MAX_DURATION_S:g}.")
    per_frame = args.get("steps_per_frame")
    if per_frame is not None and (type(per_frame) is not int or not 1 <= per_frame <= MAX_STEPS_PER_FRAME):
        refuse("INVALID_VALUE", f"steps_per_frame must be 1..{MAX_STEPS_PER_FRAME}.")
    every = args.get("log_every", 1)
    if type(every) is not int or not 1 <= every <= 10000:
        refuse("INVALID_VALUE", "log_every must be 1..10000 physics steps.")
    frame = args.get("frame", "world")
    if frame not in ("world", "body"):
        refuse("INVALID_VALUE", "frame must be world or body.")
    params = args.get("params")
    if params is not None and not isinstance(params, dict):
        refuse("INVALID_VALUE", "params must be an object; it is passed to the controller as state['params'].")
    if params is not None and len(json.dumps(params)) > 16384:
        refuse("LIMIT", "params must be at most 16 KiB of JSON.")
    observe = args.get("observe") or []
    if not isinstance(observe, list) or len(observe) > MAX_OBSERVED or not all(isinstance(p, str) and p for p in observe):
        refuse("INVALID_VALUE", f"observe must list at most {MAX_OBSERVED} rigid body paths.")
    fps = args.get("fps", 30)
    if type(fps) not in (int, float) or not 1 <= fps <= 60:
        refuse("INVALID_VALUE", "fps must be 1..60 video frames per simulated second.")
    label = args.get("label")
    if label is not None and (not isinstance(label, str) or len(label) > 80 or "\n" in label):
        refuse("INVALID_VALUE", "label must be one line of at most 80 characters.")
    mode = args.get("mode", "step")
    if mode not in ("step", "play"):
        refuse("INVALID_VALUE", "mode must be step (exact stepping from a paused timeline) or play (the timeline plays freely).")
    if mode == "play" and per_frame is not None:
        refuse("INVALID_VALUE", "steps_per_frame applies to mode=step; with mode=play the timeline sets the pace.")
    return dict(mode=mode, duration=float(duration), per_frame=per_frame, every=every, frame=frame, params=params or {},
                fields=log_fields(args.get("log")), observe=list(dict.fromkeys(observe)), fps=float(fps), label=label or None)


class Run:
    """One controller run. Kit-specific pieces (the next app frame, the operation lock, the target
    check) are passed in, so the loop itself is plain Python."""

    def __init__(self, body, controller, log, settings, stage, check_target, timeline, next_frame, lock,
                 others=None, joints=None, capture=None):
        self.body, self.controller, self.log = body, controller, log
        self.others, self.joints, self.capture = others or {}, joints, capture
        self.frames = []
        self.settings, self.stage = settings, stage
        self.check_target, self.timeline = check_target, timeline
        self.next_frame, self.lock = next_frame, lock
        sm = physics.manager()
        self.sm = sm
        self.dt = float(sm.get_physics_dt())
        if not math.isfinite(self.dt) or self.dt <= 0:
            refuse("PHYSICS_CLOCK_INVALID", "The physics timestep must be finite and positive; nothing ran.")
        self.total = min(MAX_STEPS, max(1, round(settings["duration"] / self.dt)))
        self.per_frame = settings["per_frame"] or max(1, min(MAX_STEPS_PER_FRAME, round((1 / 60) / self.dt)))
        if capture is not None:
            # One camera frame per app frame, so the steps between app frames set the video's frame rate.
            self.per_frame = max(1, min(MAX_STEPS_PER_FRAME, round(1 / (settings.get("fps", 30) * self.dt))))
            if self.total / self.per_frame > MAX_VIDEO_FRAMES:
                refuse("LIMIT", f"Recording this run would save more than {MAX_VIDEO_FRAMES} frames; lower fps or "
                       "duration_s, or run without camera and replay a part with sim__trajectory__replay.")

    def _step(self, index):
        if self._control(index):
            return True
        physics.step(self.stage, 1)
        return False

    def _control(self, index):
        """One control period: read, ask the controller, log, apply. True when the controller is done."""
        state = self.body.read()
        if self.others:
            state["observed"] = {path: other.read() for path, other in self.others.items()}
        if self.joints is not None:
            state["joints"] = self.joints.read()
        observed = dict(step=index, time_s=index * self.dt, dt=self.dt, **state, params=self.settings["params"])
        try:
            command = check_command(self.controller(observed), self.settings["frame"], (self.body.path, *self.others),
                                    self.joints.names if self.joints is not None else None)
        except Exception as exc:  # noqa: BLE001 - the caller's controller: name it, with the step
            refuse("CONTROLLER_FAILED", f"The controller failed at step {index}: {type(exc).__name__}: {str(exc)[:600]}",
                   step=index, effects="The run stopped; the simulation stays paused where it was.")
        if index % self.settings["every"] == 0:
            self.log.write(index, observed["time_s"], state, command["force"], command["torque"], command["extra"])
        if command["done"]:
            return True
        self.body.apply(command["force"], command["torque"], command["frame"])
        for path, (force, torque, frame) in command["wrenches"].items():
            (self.body if path == self.body.path else self.others[path]).apply(force, torque, frame)
        if self.joints is not None:
            self.joints.apply(command["efforts"], command["targets"], command["velocity_targets"])
        return False

    def on_physics_step(self, step_dt=None, context=None):
        """mode=play: called by the engine before each physics step. It must not raise into the engine."""
        if self._ended_by or self._callback_error is not None:
            return
        try:
            if self._control(self._done):
                self._ended_by = "controller"
                return
            self._done += 1
            if self._done >= self.total:
                self._ended_by = "duration"
        except BaseException as exc:  # noqa: BLE001 - kept for the job to raise
            self._callback_error = exc

    async def work_playing(self, job):
        """mode=play: the timeline advances physics; this waits, reports, records and watches for a stop."""
        started = time.monotonic()
        self._done, self._ended_by, self._callback_error = 0, None, None
        callback = None
        try:
            callback = physics.on_physics_step(self.on_physics_step)
            while not self._ended_by and self._callback_error is None:
                job.checkpoint()
                self.check_target()
                if not self.timeline.is_playing():
                    refuse("TIMELINE_INTERRUPTED", "The timeline was paused or stopped while the controller run was "
                           f"following it (after {self._done} steps). The run ended; its log holds the steps so far.",
                           steps=self._done)
                job.report(self._done / self.total, f"step {self._done}/{self.total}", steps=self._done,
                           sim_time_s=round(self._done * self.dt, 6))
                await self._frame()
            if self._callback_error is not None:
                raise self._callback_error
        except AssetError as exc:
            exc.details.setdefault("steps", self._done)
            exc.details.setdefault("log_file", str(self.log.path))
            raise
        finally:
            if callback is not None:
                physics.remove_physics_step(callback)
            self.close()
        return self.summary(self._done, self._ended_by, started)

    async def _frame(self):
        """Yield one app frame to Kit: by capturing the camera when recording, else by waiting for it."""
        if self.capture is None or len(self.frames) >= MAX_VIDEO_FRAMES:
            # mode=play renders as fast as the scene allows, so a slow scene can pass the limit the
            # stepped mode checks up front: the run goes on, the video ends here.
            await self.next_frame()
            return
        shot = await self.capture(f"{self.log.path.stem}-{len(self.frames) + 1:05d}.png")
        if shot.get("status") != "ok":
            raise AssetError(shot.get("code", "CAPTURE_FAILED"),
                             f"Frame {len(self.frames) + 1} could not be captured: {shot.get('message')}",
                             frames=len(self.frames))
        self.frames.append(shot["file"])

    async def work(self, job):
        if self.settings.get("mode") == "play":
            return await self.work_playing(job)
        started = time.monotonic()
        done, ended_by = 0, "duration"
        try:
            while done < self.total:
                job.checkpoint()
                async with self.lock:
                    job.checkpoint()  # a reset cancels while it holds this lock
                    self.check_target()
                    if self.timeline.is_stopped() or self.timeline.is_playing():
                        refuse("TIMELINE_INTERRUPTED", "The timeline was played or stopped while the controller run "
                               f"was stepping it (after {done} steps). The run ended; its log holds the steps so far.",
                               steps=done)
                    for _ in range(min(self.per_frame, self.total - done)):
                        if self._step(done):
                            ended_by = "controller"
                            break
                        done += 1
                    physics.sync_fabric(self.stage)
                if ended_by == "controller":
                    break
                job.report(done / self.total, f"step {done}/{self.total}", steps=done, sim_time_s=round(done * self.dt, 6))
                await self._frame()
        except AssetError as exc:
            exc.details.setdefault("steps", done)
            exc.details.setdefault("log_file", str(self.log.path))
            raise
        finally:
            self.close()
        return self.summary(done, ended_by, started)

    def close(self):
        self.log.close()
        if self.joints is not None:
            self.joints.release()
        close = getattr(self.controller, "close", None)
        if callable(close):
            close()

    def summary(self, done, ended_by, started):
        try:
            final = self.body.read()
        except AssetError as exc:
            final = dict(error=str(exc)[:300])
        video = {}
        playing = self.settings.get("mode") == "play"
        if self.frames:
            # The MCP host encodes these when it reads this result (job_status), as for a replay. While
            # playing, frames come as fast as they render, so the rate is what makes the video last as long as the run.
            rate = (len(self.frames) / max(done * self.dt, self.dt)) if playing else 1 / (self.per_frame * self.dt)
            video = dict(frames=len(self.frames), encode=dict(
                folder=str(self.log.path.parent), pattern=f"{self.log.path.stem}-*.png", frames=len(self.frames),
                fps=round(min(60.0, max(1.0, rate)), 3), label=self.settings.get("label"),
                output=f"{self.log.path.stem}.mp4"))
        return dict(steps=done, physics_dt=self.dt, sim_time_s=round(done * self.dt, 9), ended_by=ended_by,
                    wall_time_s=round(time.monotonic() - started, 3), steps_per_frame=self.per_frame,
                    log_file=str(self.log.path), log_rows=self.log.rows, final_state=final,
                    backend=self.body.backend, mode=self.settings.get("mode", "step"), **video,
                    timeline=("still playing, without the controller: pause or stop it with sim__timeline."
                              if playing else
                              "paused: read state with sim__body__state, replay the log with "
                              "sim__trajectory__replay, or return to a clean stop with "
                              "sim__timeline action=reset."))


def job_store(ext):
    store = getattr(ext.bridge, "jobs", None)
    if store is None:
        refuse("JOBS_UNAVAILABLE", "This Kit runs a twinsmith.bridge without background jobs; update the "
               "extension (it ships in the same checkout as sim-tools) and restart Kit.")
    return store


async def run(ext, args):
    """Validate, bring a stopped simulation to paused-and-initialized, and start the job."""
    import omni.kit.app as kit_app
    from pxr import UsdPhysics

    s = ext.scene
    ids = dict(session_id=args["session_id"], stage_id=args["stage_id"])
    stage = s.target(**ids)
    s.require_runtime()
    settings = validate(args)
    store = job_store(ext)
    path = args.get("body")
    for body_path in (path, *settings["observe"]):
        prim = s.prim(stage, body_path)
        if not prim.IsActive() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            refuse("INVALID_PHYSICS", f"{body_path} is not an active rigid body. Pass the body the controller pushes, "
                   "e.g. a drone's base link; find it with sim__physics__inspect.")
    articulation = args.get("articulation")
    if articulation is not None and not s.prim(stage, articulation).HasAPI(UsdPhysics.ArticulationRootAPI):
        refuse("INVALID_PHYSICS", f"{articulation} is not an articulation root. sim__physics__inspect lists them; "
               "it is often <robot>/root_joint.")
    camera = args.get("camera")
    playing_mode = settings["mode"] == "play"
    if s.timeline.is_playing() and not playing_mode:
        refuse("TIMELINE_ACTIVE", "The timeline is playing. Pause it (sim__timeline action=pause) or stop "
               "it first: mode=step steps physics itself, one exact step at a time. mode=play follows a playing timeline.")
    if playing_mode and not s.timeline.is_playing() and not s.timeline.is_stopped():
        refuse("TIMELINE_ACTIVE", "The timeline is paused. mode=play follows a playing timeline: play it "
               "(sim__timeline action=play resume=true), or use mode=step, which steps from a pause.")
    kind, first, second = parse_controller(args.get("controller"))
    controller = TcpController(first, second) if kind == "tcp" else load_python(first, second)
    next_frame = kit_app.get_app().next_update_async
    log = None
    try:
        initialized = None
        if s.timeline.is_stopped():
            initialized = await ext._timeline_control({**ids, "action": "play" if playing_mode else "initialize"})
        s.check_runtime_edit(stage)
        for _ in range(READY_FRAMES):  # the tensor view appears a few frames after the first Play
            if physics.physics_ready() is not False:
                break
            await next_frame()
        body = BodyIO(path)
        others = {other: BodyIO(other) for other in settings["observe"] if other != path}
        joints = JointIO(articulation) if articulation is not None else None
        name = args.get("filename") or time.strftime("run-%Y%m%d-%H%M%S.csv")
        capture = None
        if camera is not None:
            take = ext.handler("sim.camera.capture")
            adopt = {"adopt": True} if args.get("adopt") else {}

            async def capture(frame_name):
                return await take(**ids, path=camera, filename=frame_name, **adopt)

            if isinstance(name, str) and (s.output / f"{name[:-4]}-00001.png").exists():
                refuse("CONFLICT", f"Frames named {name[:-4]}-*.png already exist in the output folder; use another filename.")
        log = CsvLog(s.output, name, settings["fields"], joints.names if joints is not None else ())
        runner = Run(body, controller, log, settings, stage, lambda: s.target(**ids), s.timeline, next_frame,
                     ext._operation_lock, others=others, joints=joints, capture=capture)
        job = store.start("sim.simulation.run", runner.work, label=settings["label"])
    except BaseException as exc:
        if log is not None:
            log.close()
        close = getattr(controller, "close", None)
        if callable(close):
            close()
        if playing_mode and initialized is not None:      # this call started the timeline: do not leave it playing
            try:
                await ext._timeline_control({**ids, "action": "stop"})
            except Exception:  # noqa: BLE001 - the original refusal is what the caller needs
                pass
        if isinstance(exc, JobError):
            refuse("JOB_LIMIT", str(exc))
        raise
    return dict(job_id=job.id, state=job.state, body=path, controller=kind, physics_dt=runner.dt,
                steps=runner.total, steps_per_frame=runner.per_frame, log_file=str(log.path),
                backend=body.backend, mass=body.mass,
                **({"joints": joints.names} if joints is not None else {}),
                **({"camera": camera, "video": f"{log.path.stem}.mp4"} if camera is not None else {}),
                mode=settings["mode"],
                **({"initialized": ("The stopped timeline was started; it keeps playing after the run." if playing_mode else
                                    "The stopped timeline was played briefly and paused; physics may have advanced.")}
                   if initialized else {}),
                next=f"job_status(job_id=\"{job.id}\", wait_s=30) until state is not running; job_cancel stops it.")
