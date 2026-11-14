"""Host side of ``assets__mesh`` and ``assets__validate``: check the paths, then run ``mesh_worker``.

``inspect`` and ``validate`` read and answer directly. ``decimate``, ``hulls`` and ``split`` write a new
folder and run as a job (``jobs`` action ``lab_status``). Sources are files under an approved root or
the asset root, or a pack's asset by id; outputs are new folders under an approved root, never inside
the asset root, where a pack version is only written by ``assets__pack``.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from . import asset_packs, mesh_worker, pack_manifest
from .asset_packs import PackError

DIRECT_TIMEOUT_S = 600
JOB_TIMEOUT_S = 6 * 3600
WRITES = {"assets.mesh_decimate": "decimate", "assets.mesh_hulls": "hulls", "assets.mesh_split": "split"}
READS = {"assets.mesh_inspect": "inspect", "assets.validate": "validate"}
INSTALL = ("UV_PROJECT_ENVIRONMENT=<repo>/mcp/.venv uv sync --project <repo>/mcp --locked --extra isaaclab "
           "--extra mesh --no-dev   (or: uv pip install 'twinsmith[mesh]' --python {python})")


def _require_extra() -> None:
    missing = mesh_worker.missing_modules()
    if missing:
        raise PackError(f"The mesh extra is not installed in this server's interpreter ({sys.executable}); "
                        f"missing: {', '.join(missing)}. Install it with: {INSTALL.format(python=sys.executable)}",
                        "MESH_EXTRA_MISSING")


def _source(params: dict, broker, root, selected: str | None) -> Path:
    if params.get("source_path"):
        if params.get("asset_id"):
            raise PackError("Give source_path or pack with asset_id, not both.", "INVALID_VALUE")
        path = asset_packs._absolute(params["source_path"], "source_path").resolve()
        inside_root = root is not None and asset_packs._under(path, root.path.resolve()) is not None
        if not inside_root:
            asset_packs._approved_root(path, broker)
    elif params.get("asset_id"):
        if root is None:
            raise PackError("No asset root is configured, so a pack cannot be read.", "NO_ASSET_ROOT")
        pack_id, version = asset_packs._pack_argument(params, "pack", selected)
        try:
            path = pack_manifest.AssetPack(root.path, pack_id, version).asset(params["asset_id"])
        except (pack_manifest.ManifestError, OSError) as exc:
            raise PackError(f"{pack_id}/{version}: {exc}", "PACK_NOT_VERIFIED") from exc
    else:
        raise PackError("Name the file: source_path (absolute, under an approved root or the asset root) "
                        "or pack and asset_id.", "INVALID_VALUE")
    if path.suffix not in asset_packs.USD_SUFFIXES - {".usdz"} or not path.is_file():
        raise PackError(f"not a USD file (.usd, .usda, .usdc): {path}", "INVALID_PATH")
    return path


def _output(params: dict, broker, root) -> Path:
    if not params.get("output"):
        raise PackError("output: an absolute folder that does not exist yet, under an approved root.", "INVALID_VALUE")
    out = asset_packs._absolute(params["output"], "output")
    if out.exists():
        raise PackError(f"output exists: {out}. Outputs are never overwritten; name a new folder.", "OUTPUT_EXISTS")
    if not out.parent.is_dir():
        raise PackError(f"the folder that holds output does not exist: {out.parent}", "NOT_FOUND")
    if root is not None and asset_packs._under(out.parent.resolve(), root.path.resolve()) is not None:
        raise PackError("output is inside the asset root. Write to a staging folder, then make a new pack "
                        "version with assets__pack action=build.", "INVALID_PATH")
    asset_packs._approved_root(out.parent, broker)
    return out.parent.resolve() / out.name


def _collision(params: dict) -> dict:
    spec = dict(params.get("collision") or {}) if "collision" in params else {
        k: params[k] for k in ("method", "max_hulls", "threshold") if k in params}
    unknown = set(spec) - {"method", "max_hulls", "threshold"}
    if unknown:
        raise PackError(f"collision: unknown keys {sorted(unknown)}; use method, max_hulls, threshold", "INVALID_VALUE")
    if spec.setdefault("method", "box") not in ("box", "convex", "coacd", "auto"):
        raise PackError("method must be box, convex, coacd or auto", "INVALID_VALUE")
    return spec


def _request_file(request: dict) -> str:
    from .isaaclab import jobs

    folder = jobs.jobs_dir() / "mesh-requests"
    folder.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=folder, prefix=request["action"] + "-", suffix=".json", delete=False) as f:
        json.dump(request, f)
    return f.name


def _run_direct(request: dict) -> dict:
    path = _request_file(request)
    try:
        done = subprocess.run([sys.executable, mesh_worker.__file__, path], capture_output=True, text=True,
                              timeout=DIRECT_TIMEOUT_S, check=False, cwd=Path(path).parent)
    except subprocess.TimeoutExpired as exc:
        raise PackError(f"{request['action']} took longer than {DIRECT_TIMEOUT_S} s", "TIMEOUT") from exc
    finally:
        Path(path).unlink(missing_ok=True)
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    if not lines:
        raise PackError(f"the mesh worker answered nothing (exit {done.returncode}): {done.stderr[-800:]}", "MESH_FAILED")
    result = json.loads(lines[-1])
    if result.get("status") != "ok":
        raise PackError(result.get("message") or json.dumps(result)[:800], result.get("code", "MESH_FAILED"))
    return result


def stl_to_usd(stl: Path, dest: Path, *, scale: float = 0.001) -> dict:
    """*stl* as a new USD file *dest* in metres, Z up (``mesh_usd.from_stl``), in a worker process. The caller has
    checked both paths. Raises ``PackError`` (``MESH_EXTRA_MISSING`` without the mesh extra)."""
    _require_extra()
    return _run_direct(dict(action="from_stl", source=str(stl), output=str(dest), scale=scale))


def run_tool(canonical: str, params: dict, broker) -> dict:
    try:
        root = pack_manifest.resolve_asset_root()
    except pack_manifest.RootError:
        root = None
    broker.require_profile()
    selected = pack_manifest.project_pack()
    source = _source(params, broker, root, selected)
    _require_extra()
    prims = params.get("prims") or None
    if prims is not None and (not isinstance(prims, list) or not all(isinstance(p, str) and p.startswith("/") for p in prims)):
        raise PackError("prims: a list of absolute prim paths", "INVALID_VALUE")
    if canonical in READS:
        request = dict(action=READS[canonical], source=str(source), prims=prims)
        if canonical == "assets.validate":
            if params.get("kit"):
                raise PackError("kit=true is not available yet: Isaac Sim's Asset Validator has not been confirmed "
                                "in 6.2. The file rules ran nowhere; call again without kit.", "NOT_AVAILABLE")
            request.update(engine=params.get("engine", "newton"))
            if "triangle_budget" in params:
                request["triangle_budget"] = int(params["triangle_budget"])
        return _run_direct(request)

    from .isaaclab import jobs

    output = _output(params, broker, root)
    action = WRITES[canonical]
    request = dict(action=action, source=str(source), output=str(output), prims=prims)
    if action == "decimate":
        if ("target_tris" in params) == ("ratio" in params):
            raise PackError("give exactly one of target_tris or ratio", "INVALID_VALUE")
        request.update({k: params[k] for k in ("target_tris", "ratio") if k in params})
    else:
        request["collision"] = _collision(params)
    request_file = _request_file(request)
    try:
        job = jobs.run_script(sys.executable, mesh_worker.__file__, args=[request_file],
                              label=f"mesh-{action}-{source.stem}", timeout_s=JOB_TIMEOUT_S, uses_gpu=False)
    except Exception:
        Path(request_file).unlink(missing_ok=True)
        raise
    return dict(status="ok", job_id=job["job_id"], action=action, source=str(source), output=str(output),
                report=str(output / mesh_worker.REPORT),
                next=f"jobs(action=\"lab_status\", job_id=\"{job['job_id']}\", wait_s=60) until it ends; "
                     f"the result is the last log line and {mesh_worker.REPORT} in output")
