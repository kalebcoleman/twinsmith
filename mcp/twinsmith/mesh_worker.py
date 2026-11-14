"""Subprocess entry for mesh preparation and validation: one action, a JSON request in, a JSON result out.

    python -m twinsmith.mesh_worker <request.json>

The server starts this with its own interpreter, so ``pxr``, trimesh and CoACD never load in the server.
The request names an action and paths the server has already checked. The result is printed as the
last line of output and, for actions that write, also saved as ``mesh_report.json`` in the output folder.
"""

from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

if __package__ in (None, ""):                         # run as a file (a job): import the package, not siblings
    here = Path(__file__).resolve().parent
    sys.path[:] = [p for p in sys.path if Path(p or ".").resolve() != here]
    sys.path.insert(0, str(here.parent))

REPORT = "mesh_report.json"
NEEDS = ("pxr", "trimesh", "coacd", "fast_simplification", "numpy")
SUMMARY_KEYS = ("method", "max_hulls", "threshold", "meshes", "boxes", "hulls", "triangles", "source_triangles", "thickened")
METHODS = ("box", "convex", "coacd", "auto")
MAX_REPORT_BYTES = 4 << 20


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def collision_record(path: Path) -> dict | None:
    """The collision summary that ``split`` or ``hulls`` left in the report beside *path*, if it describes
    *path* and no file the report lists has changed since. ``assets__pack`` build copies it into the record."""
    report_path = Path(path).parent / REPORT
    try:
        if report_path.is_symlink() or not report_path.is_file() or report_path.stat().st_size > MAX_REPORT_BYTES:
            return None
        report = json.loads(report_path.read_text())
        summary, hashes = report.get("collision"), report.get("sha256")
        if not (isinstance(summary, dict) and isinstance(hashes, dict) and summary.get("for") == Path(path).name):
            return None
        for name, digest in hashes.items():
            listed = report_path.parent / name
            if "/" in name or name.startswith(".") or listed.is_symlink() or not listed.is_file() or _sha256(listed) != digest:
                return None
    except (OSError, ValueError, TypeError, AttributeError):
        return None
    out = {k: summary[k] for k in SUMMARY_KEYS if k in summary}
    numbers_ok = all(type(v) is int and v >= 0 for k, v in out.items() if k not in ("method", "threshold"))
    threshold_ok = isinstance(out.get("threshold", 0), (int, float)) and not isinstance(out.get("threshold"), bool)
    return out if out.get("method") in METHODS and numbers_ok and threshold_ok else None


def missing_modules() -> list[str]:
    import importlib.util

    return [name for name in NEEDS if importlib.util.find_spec(name) is None]


def run(request: dict) -> dict:
    from twinsmith import asset_rules, mesh_usd

    action = request["action"]
    source = Path(request["source"])
    prims = request.get("prims") or None
    if action == "inspect":
        return mesh_usd.inspect(source, prims)
    if action == "validate":
        return asset_rules.validate(source, request.get("engine", "newton"),
                                    triangle_budget=request.get("triangle_budget", asset_rules.TRIANGLE_BUDGET))
    if action not in ("decimate", "hulls", "split"):
        raise ValueError(f"unknown action {action!r}")
    out = Path(request["output"])
    if out.exists() or out.is_symlink():
        raise FileExistsError(f"{out} exists; outputs are never overwritten")
    # Written beside the output and renamed into place: a failed run leaves nothing, and relative paths stay valid.
    work = out.parent / f".{out.name}.partial-{os.getpid()}"
    work.mkdir()
    try:
        if action == "decimate":
            result = mesh_usd.decimate(source, work, target_tris=request.get("target_tris"), ratio=request.get("ratio"), prims=prims)
        elif action == "hulls":
            result = mesh_usd.author_collision_layer(mesh_usd.open_stage(source), work / f"{source.stem}_collision.usdc",
                                                     prims=prims, **request.get("collision", {}))
        else:
            result = mesh_usd.split(source, work, collision=request.get("collision"), prims=prims)
        result = json.loads(json.dumps(dict(action=action, source=str(source), **result)).replace(str(work), str(out)))
        report = dict(result, sha256={f.name: _sha256(f) for f in sorted(work.iterdir())})
        if action != "decimate":
            shapes = result["shapes"] if action == "split" else result
            report["collision"] = {"for": Path(result["output"]).name, **{k: shapes[k] for k in SUMMARY_KEYS if k in shapes}}
        (work / REPORT).write_text(json.dumps(report, indent=2) + "\n")
        os.rename(work, out)
    except BaseException:
        shutil.rmtree(work, ignore_errors=True)
        raise
    return result


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(json.dumps(dict(status="error", code="INVALID_VALUE", message="usage: mesh_worker <request.json>")))
        return 2
    missing = missing_modules()
    if missing:
        print(json.dumps(dict(status="error", code="MESH_EXTRA_MISSING", missing=missing)))
        return 3
    request = Path(argv[0])
    try:
        payload = json.loads(request.read_text())
        request.unlink(missing_ok=True)               # the request is read once; nothing is left in the jobs folder
        result = run(payload)
    except Exception as exc:  # noqa: BLE001 - any failure must still end in one JSON line
        print(json.dumps(dict(status="error", code="MESH_FAILED", message=f"{type(exc).__name__}: {str(exc)[:2000]}")))
        return 1
    print(json.dumps(dict(status="ok", **result)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
