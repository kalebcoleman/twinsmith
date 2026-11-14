#!/usr/bin/env python3
"""Build ``ur5e-workcell/v3``: v2 with the placeholder wrist bracket replaced by a copy of the real one.

* ``wrist_mount``: ``exts/workcell/cad/rg2_camera_bracket.py`` run by ``freecadcmd`` for the camera pose in
  ``exts/workcell/config/lab_hardware.json`` (``wrist_camera.offset_in_rg2_m``). The printed bracket clamps the RG2
  body, so the USD is in the pack's RG2 body frame and the builder places it there. The pack keeps the USD (render
  mesh plus a box collision approximation), and, in the CAD frame, the STEP of both pieces as mounted and the STEP
  and STL of each printable piece.
* Every other file is copied from v2 unchanged, including the D415 body.

v2 is only read (its hashes are verified first); no other pack or version is read or written. The result is complete
on its own: it holds copies, not references into v2. An existing v3 is left alone. Needs ``freecadcmd`` on PATH.
Run from ``mcp/``:

    uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_bracket_assets.py
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "mcp"))
sys.path.insert(0, str(REPO / "scripts"))

import workcell_camera_assets as camera_assets  # noqa: E402  (STL reading, welding and the USD writer)
from twinsmith import asset_packs, pack_manifest  # noqa: E402

PACK, SOURCE, TARGET = "ur5e-workcell", "v2", "v3"
CAD = "exts/workcell/cad/rg2_camera_bracket.py"
PIECES = ("clamp_front", "camera_arm")
PRINTED = dict(diffuseColor=(0.10, 0.11, 0.14), metallic=0.0, roughness=0.6)
FIDELITY = ("Parametric copy of the real printed bracket, estimated from one photo; the RG2 and D415 dimensions are from "
            "the vendors' datasheets, everything else must be measured (exts/workcell/cad/README.md)")


def to_body(points):
    """CAD frame (+X finger-opening axis, +Y out of the front face) to the pack's RG2 body frame, a quarter turn about Z.

    The pack's ``onrobot_rg2_base_link`` has +Y along the finger-opening axis and +X out of the back face.
    """
    points = np.asarray(points, dtype=float)
    return np.stack([-points[..., 1], points[..., 0], points[..., 2]], axis=-1)


def camera_pose(config):
    """The configured camera origin in the CAD frame, in millimetres; only the pose the CAD model covers is accepted."""
    section = config["wrist_camera"]
    offset, rotation = section["offset_in_rg2_m"]["value"], section["rotation_in_rg2_deg"]["value"]
    if [float(value) for value in rotation] != [0.0, 0.0, -90.0]:
        raise SystemExit("the bracket model covers a camera looking along the RG2's +Z with optical +x along the body's -Y")
    x, y, z = (float(value) * 1000.0 for value in offset)
    return [y, -x, z]                                                    # the inverse of to_body


def d415_extent(path: Path):
    """Back face along the optical axis and body centre along optical x, in millimetres, from the pack's D415 mesh."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))                                     # kept alive while its prims are read
    points = [np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float) for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    points = np.vstack(points) * 1000.0
    return float(points[:, 2].min()), float((points[:, 0].min() + points[:, 0].max()) / 2)


def run_cad(out: Path, camera_mm, back_mm, body_cx_mm):
    # FreeCAD brings its own Python: this interpreter's variables (uv's virtual environment) must not reach it.
    clean = {key: value for key, value in os.environ.items() if not key.startswith(("PYTHON", "VIRTUAL_ENV", "UV_"))}
    env = dict(clean, BRACKET_OUT=str(out), CAM_X_MM=str(camera_mm[0]), CAM_Y_MM=str(camera_mm[1]), CAM_Z_MM=str(camera_mm[2]),
               BACK_MM=str(back_mm), BODY_CX_MM=str(body_cx_mm))
    done = subprocess.run(["freecadcmd", str(REPO / CAD)], env=env, capture_output=True, text=True, timeout=600)
    report = out / "bracket.json"
    if done.returncode or not report.is_file():
        raise SystemExit(f"freecadcmd did not write the bracket: {done.stdout[-800:]} {done.stderr[-800:]}")
    return json.loads(report.read_text())


def add_collision(path: Path, report):
    """Box colliders (and the truss as a convex prism) under ``/Asset/Collision``, guide purpose, in metres."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    stage = Usd.Stage.Open(str(path))
    scope = UsdGeom.Scope.Define(stage, "/Asset/Collision")
    UsdGeom.Imageable(scope.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    names = []
    for name, low, high in report["collision_boxes_mm"]:
        corners = to_body([low, high]) / 1000.0
        low, high = corners.min(0), corners.max(0)
        cube = UsdGeom.Cube.Define(stage, f"/Asset/Collision/{name}")
        cube.CreateSizeAttr(1.0)
        cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        cube.AddTranslateOp().Set(Gf.Vec3d(*((low + high) / 2)))
        cube.AddScaleOp().Set(Gf.Vec3f(*(high - low)))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        names.append(name)
    prism = report["collision_prism_mm"]
    corners = (to_body([[x, y, z] for z in prism["z"] for x, y in prism["xy"]]) / 1000.0).tolist()
    mesh = UsdGeom.Mesh.Define(stage, f"/Asset/Collision/{prism['name']}")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*corner) for corner in corners]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3, 3, 4, 4, 4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([Gf.Vec3f(*np.min(corners, axis=0)), Gf.Vec3f(*np.max(corners, axis=0))])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    names.append(prism["name"])
    stage.GetRootLayer().Save()
    return names


def build_bracket(work: Path, d415_path: Path):
    config = json.loads((REPO / "exts/workcell/config/lab_hardware.json").read_text())
    camera_mm = camera_pose(config)
    back_mm, body_cx_mm = d415_extent(d415_path)
    folder = work / "wrist_mount"
    folder.mkdir()
    report = run_cad(folder, camera_mm, round(back_mm, 2), round(body_cx_mm, 2))
    triangles = to_body(np.concatenate([camera_assets.read_stl(folder / f"{name}.stl") for name in PIECES])) * 0.001
    points, faces = camera_assets.weld(triangles)
    usd = folder / "wrist_mount.usdc"
    camera_assets.write_usd(usd, points, faces, {"Printed": (np.ones(len(faces), bool), PRINTED)},
                            f"FreeCAD model ({CAD}) moved to the pack's RG2 body frame; estimated from one photo, not measured")
    colliders = add_collision(usd, report)
    return report, {"triangles": int(len(faces)), "colliders": colliders, "camera_in_cad_frame_mm": camera_mm,
                    "d415_back_mm": round(back_mm, 2), "d415_body_centre_x_mm": round(body_cx_mm, 2)}


def main() -> int:
    root = pack_manifest.resolve_asset_root().path
    if (root / PACK / TARGET / "pack.yaml").is_file():
        print(json.dumps({f"{PACK}/{TARGET}": "exists, left alone"}))
        return 0
    source = pack_manifest.AssetPack(root, PACK, SOURCE)                  # verifies every hash first
    folder = root / PACK / SOURCE
    manifest = pack_manifest.read_manifest(folder, PACK, SOURCE)
    work = Path(tempfile.mkdtemp(prefix="workcell-bracket-"))
    try:
        report, summary = build_bracket(work, source.asset("d415"))
        files, records = {}, {}
        for record in manifest["files"]:
            if not record["path"].startswith("wrist_mount/"):
                files[record["path"]], records[record["path"]] = folder / record["path"], record
        common = dict(source=f"{CAD} (FreeCAD), for wrist_camera.offset_in_rg2_m in config/lab_hardware.json",
                      license="Locally authored lab geometry; private pack", redistributable=False, fidelity=FIDELITY)
        made = work / "wrist_mount"
        files["wrist_mount/wrist_mount.usdc"] = made / "wrist_mount.usdc"
        records["wrist_mount/wrist_mount.usdc"] = dict(
            asset_id="wrist_mount", **common,
            changes="Both printed pieces as one render mesh in the RG2 body frame, plus a box collision approximation "
                    "under /Asset/Collision (the work cell builder removes it: camera parts are visual only).")
        for name in ("wrist_bracket.step", "bracket.json", *(f"{piece}.{kind}" for piece in PIECES for kind in ("step", "stl"))):
            files[f"wrist_mount/{name}"] = made / name
            records[f"wrist_mount/{name}"] = dict(common)
        extra = {key: manifest[key] for key in ("mimic",) if key in manifest}
        extra.update(qualification=f"{PACK}/{SOURCE} with the wrist bracket replaced by a copy of the real one on the RG2",
                     derived_from=f"{PACK}/{SOURCE}")
        written = asset_packs.write_version(root, PACK, TARGET, files, records, defaults={}, notes=[], extra=extra)
        print(json.dumps({"written": str(written)[:160], "bracket": summary, "pieces": report["pieces"],
                          "interference_mm3": report["interference_mm3"], "clearance_mm": report["clearance_mm"]}, indent=1))
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
