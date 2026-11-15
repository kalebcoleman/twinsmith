"""``sam3``: SAM 3.1 masks, and box poses from depth, for a video file or a recorded RGB-D clip.

The model is SAM 3.1 (the Object Multiplex checkpoint) and runs in its own environment,
``<workspace>/tools/sam3/.venv`` (``TWINSMITH_SAM3_ROOT`` names another folder), made by the owner; this module
never imports torch or SAM 3 and never installs anything. A ``track`` is a job (``jobs`` follows and cancels it)
that runs ``sam3_driver.py`` with that interpreter and writes into ``<workspace>/runs/sam3/<name>/``.

A source is a video file, or a folder holding ``rgb/`` (frames in name order), and for 3-D results ``depth/`` (one
image per colour frame: 16-bit PNG counts or ``.npy`` metres) with ``camera.json``: ``{"fx", "fy", "cx", "cy"}`` in
pixels, optionally ``"depth_scale"`` (metres per PNG count, default 0.001), ``"fps"``, ``"world_from_camera"`` (4 x 4,
camera optical frame: +X right, +Y down, +Z forward) or ``"frames": [{"world_from_camera": ...}, ...]`` for a camera
that moves. Nothing here reads a camera or a ROS topic: a recorder writes the folder.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
from pathlib import Path
import re
import shutil
from typing import Annotated, Literal

from mcp.types import ToolAnnotations
from pydantic import Field

from . import pack_manifest, sibling
from .robosim.bridge import _ffmpeg_exe

PIN = "f1647e8d0c1d280991f4d1b7d974c2704537fe66"      # facebookresearch/sam3: the code SAM 3.1 needs
REPOSITORY = "https://github.com/facebookresearch/sam3"
MODEL, CHECKPOINT = "facebook/sam3.1", "sam3.1_multiplex.pt"
ACTIONS = ("status", "track", "report", "fuse")
VIDEOS = (".mp4", ".mov", ".avi", ".mkv", ".webm")
IMAGES = (".jpg", ".jpeg", ".png")
MAX_PROMPTS = 8
MAX_FRAMES = 3000
MAX_VIEWS = 16
FUSE_EXTENT_ERROR_M = 0.012        # a face whose extents miss the box's sides by more than this is not used
RECENT = 5
_DRIVER = Path(__file__).with_name("sam3_driver.py")
_TAG = re.compile(r"[a-z][a-z0-9_]{0,31}")
_PROBE = ("import importlib.metadata, json, torch, sam3\nfrom huggingface_hub import try_to_load_from_cache\n"
          "source = json.loads(importlib.metadata.distribution('sam3').read_text('direct_url.json') or '{}')\n"
          "print(json.dumps([torch.__version__, torch.cuda.is_available(), source.get('vcs_info', {}).get('commit_id'), "
          f"isinstance(try_to_load_from_cache({MODEL!r}, {CHECKPOINT!r}), str)]))")


def setup_steps(root: Path) -> list[str]:
    """What the owner runs to make the environment. The tool prints these and never runs them."""
    python = f"{root}/.venv/bin/python"
    return [f"uv venv --python 3.12 {root}/.venv",
            f"printf 'numpy>=1.26\\n' > {root}/overrides.txt",
            f'uv pip install --python {python} --override {root}/overrides.txt torch torchvision '
            f'"sam3 @ git+{REPOSITORY}.git@{PIN}" decord2 pycocotools pillow einops psutil',
            f"{python} -c \"from huggingface_hub import hf_hub_download; hf_hub_download('{MODEL}', '{CHECKPOINT}')\"   "
            f"# once, logged in to a Hugging Face account that has been granted access to {MODEL}"]


def parse_prompts(prompts, box_size_m=None) -> list[dict]:
    """``["blocks=wooden block", ...]`` as ``[{"tag", "text", "size_m"?}]``; a tag names the outputs."""
    if not isinstance(prompts, list) or not 1 <= len(prompts) <= MAX_PROMPTS:
        raise sibling.Refusal("INVALID_VALUE", f"prompts must be 1 to {MAX_PROMPTS} entries such as 'blocks=wooden block'.")
    sizes = dict(box_size_m or {})
    parsed = []
    for prompt in prompts:
        tag, _, text = prompt.partition("=") if isinstance(prompt, str) else ("", "", "")
        if not _TAG.fullmatch(tag) or not text.strip() or len(text) > 200:
            raise sibling.Refusal("INVALID_VALUE", f"prompt {prompt!r} is not TAG=TEXT (tag: lower-case letters, digits, _; "
                                  "text: what SAM 3 should find, such as 'wooden block').")
        if tag in [p["tag"] for p in parsed] or tag == "frames":
            raise sibling.Refusal("INVALID_VALUE", f"prompt tag {tag!r} is used twice or is the reserved name 'frames'.")
        entry = {"tag": tag, "text": text.strip()}
        if tag in sizes:
            size = sizes.pop(tag)
            if (not isinstance(size, list) or len(size) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                                                    or not math.isfinite(v) or not 0.001 <= v <= 2 for v in size)):
                raise sibling.Refusal("INVALID_VALUE", f"box_size_m[{tag!r}] must be [length, width, height] in metres, each 0.001 to 2.")
            entry["size_m"] = [float(v) for v in size]
        parsed.append(entry)
    if sizes:
        raise sibling.Refusal("INVALID_VALUE", f"box_size_m names {sorted(sizes)}, which is no prompt tag.")
    return parsed


def parse_zones(zone, tags) -> list[list[str]]:
    """``{"hand": "tower"}`` as ``[["hand", "tower"]]``: report when an object of the first tag is at the second's."""
    zone = dict(zone or {})
    for tag, where in zone.items():
        if tag not in tags or where not in tags or tag == where:
            raise sibling.Refusal("INVALID_VALUE", f"zone {{{tag!r}: {where!r}}} must name two different prompt tags ({tags}).")
    return [[tag, where] for tag, where in zone.items()]


def _axes(quaternion):
    """The three box axes (length, width, height) of a pose quaternion [x, y, z, w], as unit vectors."""
    x, y, z, w = quaternion
    return [[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
            [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]]


def fuse_boxes(views, size) -> list[dict]:
    """One list of boxes from several views' box poses in the world frame.

    ``views`` is ``[(view name, [object rows of that view's frame])]``. Two detections are the same box when their
    centres are nearer than half the box's smallest side and their long axes agree: boxes side by side or in
    crossed layers are further apart than that, or turned. A fused box is the mean of its detections' centres, with
    the orientation of the detection whose face matched best. Views that see a box by different faces correct one
    another; a box no view saw is not in the list."""
    limit, found = min(size) / 2, []
    for view, objects in views:
        for row in objects:
            pose = row.get("pose")
            if pose and pose.get("position_world_m") and pose["extent_error_m"] <= FUSE_EXTENT_ERROR_M:
                found.append({"view": view, "id": row["id"], "face": pose["face"], "error": pose["extent_error_m"],
                              "centre": pose["position_world_m"], "quaternion": pose["quaternion_world_xyzw"],
                              "long": _axes(pose["quaternion_world_xyzw"])[0]})
    boxes = []
    for item in sorted(found, key=lambda item: item["error"]):
        for box in boxes:
            mean = [sum(member["centre"][k] for member in box) / len(box) for k in range(3)]
            near = math.dist(mean, item["centre"]) < limit
            if near and abs(sum(a * b for a, b in zip(box[0]["long"], item["long"]))) > 0.8:
                box.append(item)
                break
        else:
            boxes.append([item])
    fused = []
    for box in boxes:
        centre = [sum(member["centre"][k] for member in box) / len(box) for k in range(3)]
        long = box[0]["long"] if max(box[0]["long"], key=abs) > 0 else [-v for v in box[0]["long"]]   # one of the two ways
        fused.append({"position_world_m": centre, "quaternion_world_xyzw": box[0]["quaternion"], "long_axis_world": long,
                      "views": sorted({member["view"] for member in box}), "faces": sorted({member["face"] for member in box}),
                      "detections": len(box), "spread_m": max(math.dist(centre, member["centre"]) for member in box)})
    return sorted(fused, key=lambda box: (round(box["position_world_m"][2], 3), box["position_world_m"][0], box["position_world_m"][1]))


def _matrix(value, what):
    if (not isinstance(value, list) or len(value) != 4 or any(not isinstance(row, list) or len(row) != 4 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row) for row in value)):
        raise sibling.Refusal("CAMERA_INVALID", f"{what} must be a 4 x 4 matrix of finite numbers.")


def check_source(source, every=1) -> dict:
    """What a source holds: ``{"source", "kind", "frames", "camera"}``. A folder's ``camera.json`` is checked here."""
    path = Path(source) if isinstance(source, str) else None
    if path is None or not path.is_absolute():
        raise sibling.Refusal("INVALID_VALUE", "source must be an absolute path to a video file or a recorded clip folder.")
    if path.is_file():
        if path.suffix.lower() not in VIDEOS:
            raise sibling.Refusal("SOURCE_INVALID", f"{path.name} is not a video ({', '.join(VIDEOS)}).")
        return {"source": str(path), "kind": "video", "frames": None, "camera": None}
    if not (path / "rgb").is_dir():
        raise sibling.Refusal("SOURCE_INVALID", f"{path} is neither a video file nor a folder with rgb/ frames.")
    frames = sorted(p for p in (path / "rgb").iterdir() if p.suffix.lower() in IMAGES)
    if not frames or len(frames[::every]) > MAX_FRAMES:
        raise sibling.Refusal("SOURCE_INVALID", f"rgb/ must hold 1 to {MAX_FRAMES} .jpg or .png frames (after every).")
    camera = None
    if (path / "depth").is_dir():
        depth = sorted(p for p in (path / "depth").iterdir() if p.suffix in (".png", ".npy"))
        if len(depth) != len(frames):
            raise sibling.Refusal("SOURCE_INVALID", f"depth/ holds {len(depth)} images for {len(frames)} colour frames; they must pair up.")
        try:
            camera = json.loads((path / "camera.json").read_text())
            if any(isinstance(camera[key], bool) or not isinstance(camera[key], (int, float)) or not math.isfinite(camera[key])
                   for key in ("fx", "fy", "cx", "cy")) or camera["fx"] <= 0 or camera["fy"] <= 0:
                raise ValueError("fx, fy, cx and cy must be finite numbers, fx and fy positive")
            if any(isinstance(camera[key], bool) or not isinstance(camera[key], (int, float)) or not math.isfinite(camera[key])
                   or camera[key] <= 0 for key in ("depth_scale", "fps") if key in camera):
                raise ValueError("depth_scale and fps, when given, must be positive finite numbers")
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise sibling.Refusal("CAMERA_INVALID", f"A clip with depth/ needs camera.json with fx, fy, cx, cy in pixels: {error}") from None
        if "world_from_camera" in camera:
            _matrix(camera["world_from_camera"], "camera.json world_from_camera")
        if "frames" in camera:
            if not isinstance(camera["frames"], list) or len(camera["frames"]) != len(frames):
                raise sibling.Refusal("CAMERA_INVALID", "camera.json frames must have one entry per colour frame.")
            for entry in camera["frames"]:
                _matrix(entry.get("world_from_camera") if isinstance(entry, dict) else None, "camera.json frames[].world_from_camera")
    return {"source": str(path), "kind": "frames", "frames": len(frames[::every]), "camera": camera}


class Sam3:
    def __init__(self, workspace=None, root=None, launch=None, job_status=None):
        self.workspace = Path(workspace or pack_manifest.workspace_dir()).expanduser().resolve()
        self.root = Path(root or os.environ.get("TWINSMITH_SAM3_ROOT") or self.workspace / "tools/sam3").expanduser().resolve()
        self.python = self.root / ".venv/bin/python"
        self.runs = self.workspace / "runs/sam3"
        self._launch, self._job_status = launch or sibling.start_job, job_status

    def environment(self, probe=True) -> dict:
        info = {"environment": str(self.root), "python": str(self.python), "model": MODEL, "checkpoint": CHECKPOINT,
                "repository": REPOSITORY, "pin": PIN, "commit": None, "pin_match": False, "warnings": []}
        if not (self.python.is_file() and os.access(self.python, os.X_OK)):
            return dict(info, ok=False, code="ENVIRONMENT_MISSING", setup=setup_steps(self.root),
                        error=f"No SAM 3 environment at {self.root}. The owner creates it with the setup steps; "
                              "this tool never installs anything.")
        if not probe:
            return dict(info, ok=True)
        done = sibling.run([self.python, "-I", "-c", _PROBE], env_pass=("HF_HOME",), env_set={"HF_HUB_OFFLINE": "1"},
                           timeout_s=180, stdout_chars=sibling.WHOLE)
        try:
            torch_version, cuda, commit, weights = json.loads(done["stdout"].strip().splitlines()[-1])
        except (ValueError, IndexError):
            return dict(info, ok=False, code="ENVIRONMENT_BROKEN", setup=setup_steps(self.root),
                        error="The SAM 3 environment cannot import torch and sam3. " + done["stderr"][-600:])
        info.update(torch=torch_version, cuda=cuda, commit=commit, pin_match=commit == PIN, weights_cached=weights)
        if commit != PIN:
            info["warnings"].append(f"The installed sam3 code is at {commit or 'an unknown commit'}, not the tested pin {PIN}.")
        if not cuda or not weights:
            return dict(info, ok=False, code="ENVIRONMENT_BROKEN", setup=setup_steps(self.root),
                        error=f"SAM 3.1's video predictor needs CUDA and the cached {MODEL} checkpoint: "
                              f"cuda={cuda}, weights_cached={weights}.")
        return dict(info, ok=True)

    def call(self, action, *, source=None, name=None, prompts=None, box_size_m=None, zone=None, start_s=None, end_s=None,
             every=1, fps=None, timeout_s=3600.0, allow_concurrent_gpu=False, tag=None, frame=None, runs=None):
        try:
            if action not in ACTIONS:
                raise sibling.Refusal("INVALID_VALUE", "action must be one of: " + ", ".join(ACTIONS))
            if action == "report":
                return dict(self._report(name, tag, frame), tool="sam3", action=action)
            if action == "fuse":
                return dict(self._fuse(name, runs, tag, box_size_m), tool="sam3", action=action)
            if action == "status":
                return dict(self.environment(), tool="sam3", action=action, bridge_needed=False, recent=self._recent())
            env = self.environment(probe=False)
            if not env["ok"]:
                return dict(env, tool="sam3", action=action)
            return dict(env, tool="sam3", action=action,
                        **self._track(source, name, prompts, box_size_m, zone, start_s, end_s, every, fps, timeout_s, allow_concurrent_gpu))
        except sibling.Refusal as refusal:
            return {"ok": False, "tool": "sam3", "action": action, "code": refusal.code, "error": str(refusal)}
        except OSError as error:
            return {"ok": False, "tool": "sam3", "action": action, "code": "IO_ERROR", "error": str(error)}

    def _track(self, source, name, prompts, box_size_m, zone, start_s, end_s, every, fps, timeout_s, allow_concurrent_gpu) -> dict:
        for value, label, low, high in ((start_s, "start_s", 0, 86400), (end_s, "end_s", 0, 86400), (fps, "fps", 0.01, 1000),
                                        (timeout_s, "timeout_s", 10, 86400)):
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high):
                raise sibling.Refusal("INVALID_VALUE", f"{label} must be a number from {low} to {high}.")
        if isinstance(every, bool) or not isinstance(every, int) or not 1 <= every <= 1000:
            raise sibling.Refusal("INVALID_VALUE", "every must be a whole number from 1 to 1000.")
        if start_s is not None and end_s is not None and end_s <= start_s:
            raise sibling.Refusal("INVALID_VALUE", "end_s must be greater than start_s.")
        checked = check_source(source, every)
        parsed = parse_prompts(prompts, box_size_m)
        zones = parse_zones(zone, [prompt["tag"] for prompt in parsed])
        if checked["kind"] == "frames" and (start_s is not None or end_s is not None):
            raise sibling.Refusal("INVALID_VALUE", "start_s and end_s cut a video file; a frames folder is used whole (thin it with every).")
        if any("size_m" in prompt for prompt in parsed) and not checked["camera"]:
            raise sibling.Refusal("DEPTH_MISSING", "box_size_m gives 3-D box poses, which need a clip folder with depth/ and camera.json.")
        ffmpeg = _ffmpeg_exe()
        if not ffmpeg:
            raise sibling.Refusal("FFMPEG_MISSING", "ffmpeg was not found on PATH or in the imageio-ffmpeg package.")
        folder = self.runs / sibling.run_name(name, "sam3")
        sibling.new_run_folder(folder)
        shutil.copyfile(_DRIVER, folder / "driver.py")
        request = {"model": MODEL, "checkpoint": CHECKPOINT, "source": checked["source"], "kind": checked["kind"],
                   "camera": checked["camera"], "prompts": parsed, "zones": zones,
                   "start_s": start_s, "end_s": end_s, "every": every, "fps": fps, "ffmpeg": ffmpeg, "out": str(folder)}
        (folder / "request.json").write_text(json.dumps(request, indent=1) + "\n")
        started = self._launch([self.python, "-I", folder / "driver.py", folder / "request.json"], task=f"sam3-{folder.name}",
                               kind="sam3-track", cwd=folder, env_pass=("HF_HOME",),
                               env_set={"HF_HUB_OFFLINE": "1", "PYTHONUNBUFFERED": "1"}, timeout_s=float(timeout_s),
                               uses_gpu=True, allow_concurrent_gpu=allow_concurrent_gpu)
        if not started["ok"]:
            shutil.rmtree(folder)
            return {"ok": False, "code": started["code"], "error": started["message"], "hint": started["hint"]}
        (folder / "job.json").write_text(json.dumps({"job_id": started["job_id"]}) + "\n")
        return {"name": folder.name, "run_dir": str(folder), "job_id": started["job_id"], "source_kind": checked["kind"],
                "frames": checked["frames"], "depth": bool(checked["camera"]), "prompts": parsed,
                "next": f'jobs(action="status", job_id="{started["job_id"]}", wait_s=60), then sam3(action="report", name="{folder.name}").'}

    def _report(self, name, tag, frame) -> dict:
        folder = self.runs / sibling.run_name(name, "sam3") if isinstance(name, str) else None
        if folder is None or not (folder / "request.json").is_file():
            raise sibling.Refusal("RUN_NOT_FOUND", f"No sam3 run named {name!r} in {self.runs}.")
        if (folder / "result.json").is_file():
            result = json.loads((folder / "result.json").read_text())
            report = {"ok": True, "state": "finished", "name": folder.name, "run_dir": str(folder), **result}
            if tag is not None:
                rows = json.loads((folder / f"{tag}.json").read_text())["frames"] if tag in [p["tag"] for p in result["prompts"]] else None
                index = 0 if frame is None else frame
                if rows is None or isinstance(index, bool) or not isinstance(index, int) or not -len(rows) <= index < len(rows):
                    raise sibling.Refusal("INVALID_VALUE", f"tag must be one of {[p['tag'] for p in result['prompts']]} and frame "
                                          f"an index into its {result['frames']} frames (negative counts from the end).")
                report["frame"] = rows[index]
            return report
        job_id = json.loads((folder / "job.json").read_text())["job_id"] if (folder / "job.json").is_file() else None
        try:
            job = self._job_status(job_id) if job_id and self._job_status else {}
        except ValueError:
            job = {}
        state = job.get("state") or "unknown"
        if state == "running":
            return {"ok": True, "state": "running", "name": folder.name, "job_id": job_id,
                    "next": f'jobs(action="status", job_id="{job_id}", wait_s=60)'}
        return {"ok": False, "state": state, "name": folder.name, "job_id": job_id, "code": "TRACK_FAILED",
                "error": "The run has no result.json: it failed, was cancelled, or its job record is gone.",
                "exit_code": job.get("exit_code"), "log_dir": job.get("log_dir"), "last_stderr_lines": job.get("last_stderr_lines")}

    def _fuse(self, name, runs, tag, box_size_m) -> dict:
        """Merge the last frame's box poses of ``tag`` from several finished runs (views) into ``<name>/fused.json``."""
        if not isinstance(runs, list) or not 1 <= len(runs) <= MAX_VIEWS or len(set(runs)) != len(runs) or not isinstance(tag, str):
            raise sibling.Refusal("INVALID_VALUE", f"fuse needs tag and runs: 1 to {MAX_VIEWS} different finished sam3 run names, one per view.")
        views, size = [], None
        for run in runs:
            report = self._report(run, tag, -1)
            if report.get("state") != "finished":
                raise sibling.Refusal("RUN_NOT_FINISHED", f"sam3 run {run!r} has not finished.")
            prompt = next(p for p in json.loads((self.runs / run / "request.json").read_text())["prompts"] if p["tag"] == tag)
            if "size_m" not in prompt or (size or prompt["size_m"]) != prompt["size_m"]:
                raise sibling.Refusal("INVALID_VALUE", f"every run must have tracked {tag!r} with the same box_size_m.")
            size = prompt["size_m"]
            views.append((run, report["frame"]["objects"]))
        boxes = fuse_boxes(views, size)
        folder = self.runs / sibling.run_name(name, "sam3-fused")
        sibling.new_run_folder(folder)
        result = {"runs": runs, "tag": tag, "size_m": size, "boxes": boxes}
        (folder / "fused.json").write_text(json.dumps(result, indent=1) + "\n")
        return {"ok": True, "name": folder.name, "fused_json": str(folder / "fused.json"), "views": len(runs), "boxes": len(boxes),
                "seen_by_one_view": sum(len(box["views"]) == 1 for box in boxes),
                "largest_spread_m": max((box["spread_m"] for box in boxes), default=0.0),
                "detections_used": sum(box["detections"] for box in boxes),
                "detections": {run: sum(1 for row in objects if row.get("pose")) for run, objects in views}}

    def _recent(self) -> list[dict]:
        runs = sorted(self.runs.glob("*/request.json"), key=lambda p: p.stat().st_mtime, reverse=True)
        return [{"name": p.parent.name, "finished": (p.parent / "result.json").is_file()} for p in runs[:RECENT]]


def register_sam3_tools(server, *, sam3=None):
    from .isaaclab import jobs

    sam3 = sam3 or Sam3(job_status=jobs.runs_status)

    @server.tool(name="sam3", annotations=ToolAnnotations(
        readOnlyHint=False, destructiveHint=False, idempotentHint=False, openWorldHint=False,
    ))
    async def sam3_tool(
        action: Annotated[Literal["status", "track", "report", "fuse"], Field(description="status: environment, recent runs. track: start a job. report: read a run. fuse: merge views' box poses.")],
        source: Annotated[str | None, Field(description="track: absolute video file, or clip folder with rgb/ (depth/ and camera.json for 3-D).")] = None,
        prompts: Annotated[list[str] | None, Field(description="track: TAG=TEXT per thing to find, e.g. ['blocks=wooden block'].")] = None,
        box_size_m: Annotated[dict[str, list[float]] | None, Field(description="track: {tag: [length, width, height]} metres; that tag's objects get box poses. Needs depth.")] = None,
        zone: Annotated[dict[str, str] | None, Field(description="track: {tag: zone tag}, e.g. {'hand': 'tower'}: report when a hand is at the tower.")] = None,
        name: Annotated[str | None, Field(description="track/report/fuse: run name; folder <workspace>/runs/sam3/<name>/. track, fuse: omit for a timestamp.")] = None,
        start_s: Annotated[float | None, Field(description="track, video: window start, seconds.", ge=0)] = None,
        end_s: Annotated[float | None, Field(description="track, video: window end, seconds.", ge=0)] = None,
        every: Annotated[int, Field(description="track: use every n-th frame.", ge=1, le=1000)] = 1,
        fps: Annotated[float | None, Field(description="track: frame rate, if the source gives none.", gt=0)] = None,
        timeout_s: Annotated[float, Field(description="track: stop the job after this long.", ge=10, le=86400)] = 3600.0,
        allow_concurrent_gpu: Annotated[bool, Field(description="track: knowingly run beside other GPU work.")] = False,
        tag: Annotated[str | None, Field(description="report: also return one frame's objects of this tag. fuse: the boxes' tag.")] = None,
        runs: Annotated[list[str] | None, Field(description="fuse: finished runs of one scene from different sides, each with world poses.")] = None,
        frame: Annotated[int | None, Field(description="report with tag: frame index (default 0; -1 is the last).")] = None,
    ) -> str:
        """SAM 3.1 masks for prompted things through a video or a recorded RGB-D clip, in its own environment on the GPU; no
        simulator bridge. track is a job (follow with jobs, then report) writing per-frame id masks, object rows and
        overlay.mp4. With depth and intrinsics an object gets a 3-D point, and a box of known size a pose; fuse merges
        the box poses of several views. Reads files only: no camera, no ROS topic."""
        result = await asyncio.to_thread(sam3.call, action, source=source, name=name, prompts=prompts, box_size_m=box_size_m,
                                         zone=zone, start_s=start_s, end_s=end_s, every=every, fps=fps, timeout_s=timeout_s,
                                         allow_concurrent_gpu=allow_concurrent_gpu, tag=tag, frame=frame, runs=runs)
        return json.dumps(result)

    return sam3
