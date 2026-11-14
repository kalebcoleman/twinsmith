#!/usr/bin/env python3
"""Build ``ur5e-workcell/v2``: v1 with a real D415 body and a modelled wrist bracket in place of the box proxies.

* ``d415``: RealSense's own D415 mesh, fetched from the vendor's ROS description at a pinned commit and checked by
  SHA-256 (Apache-2.0; nothing is kept in this repository). It is converted to USD in the colour camera's optical
  frame (origin at the colour sensor, +x right, +y down, +z forward) from the offsets in the same package's
  ``_d415.urdf.xacro``, with an aluminium housing and a black glass front.
* ``wrist_mount``: ``exts/workcell/cad/wrist_bracket.py`` run by ``freecadcmd`` for the camera pose in
  ``exts/workcell/config/lab_hardware.json``; the STEP file is kept in the pack beside the USD. Modelled in tool0.

v1 is only read; an existing v2 is left alone. Needs ``freecadcmd`` on PATH. Run from ``mcp/``:

    uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_camera_assets.py
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "mcp"))

from twinsmith import asset_packs, pack_manifest  # noqa: E402

PACK, SOURCE, TARGET = "ur5e-workcell", "v1", "v2"
COMMIT = "7a87097c499e3b0b69518d54a8e7a9ac1924e87c"
MESH_URL = f"https://raw.githubusercontent.com/realsenseai/realsense-ros/{COMMIT}/realsense2_description/meshes/d415.stl"
MESH_SHA256 = "5ffbe0d4b4ffee3133b4b55b1fa68c053e46d186f4a4d2b4c0174a89f423481c"
# realsense2_description/urdf/_d415.urdf.xacro at that commit: the mesh in camera_link (x forward, y left, z up),
# and the colour sensor from the depth (left imager) frame, which sits on camera_link's origin.
MESH_XYZ, MESH_RPY = (0.00987, -0.020, 0.0), (math.pi / 2, 0.0, math.pi / 2)
COLOR_Y = 0.015
ALUMINIUM = dict(diffuseColor=(0.74, 0.75, 0.77), metallic=1.0, roughness=0.38)
GLASS = dict(diffuseColor=(0.01, 0.01, 0.012), metallic=0.0, roughness=0.06, clearcoat=1.0, clearcoatRoughness=0.03)
BRACKET = dict(diffuseColor=(0.03, 0.03, 0.035), metallic=0.0, roughness=0.55)


def fetch(cache: Path) -> Path:
    path = cache / f"d415-{MESH_SHA256[:12]}.stl"
    if not path.is_file():
        cache.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(urllib.request.Request(MESH_URL, headers={"User-Agent": "twinsmith"}), timeout=120) as reply:
            path.write_bytes(reply.read())
    if hashlib.sha256(path.read_bytes()).hexdigest() != MESH_SHA256:
        path.unlink()
        raise SystemExit("the downloaded D415 mesh does not match its pinned SHA-256; nothing was built")
    return path


def read_stl(path: Path):
    """Triangles of a binary STL as (n, 3, 3) float64."""
    data = path.read_bytes()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    if len(data) != 84 + 50 * count:
        raise SystemExit(f"{path.name} is not a binary STL")
    records = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=count, offset=84)
    return records["v"].astype(np.float64)


def weld(triangles):
    points, index = np.unique(triangles.reshape(-1, 3).round(7), axis=0, return_inverse=True)
    faces = index.reshape(-1, 3)
    return points, faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]


def d415_optical(triangles):
    """The vendor mesh in the colour optical frame, metres."""
    if np.ptp(triangles.reshape(-1, 3), axis=0).max() > 1.0:
        triangles = triangles * 0.001                                     # millimetres
    roll, _, yaw = MESH_RPY
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    link = triangles @ (rz @ rx).T + np.array(MESH_XYZ)
    return np.stack([-(link[..., 1] - COLOR_Y), -link[..., 2], link[..., 0]], axis=-1)


def write_usd(path: Path, points, faces, groups, note):
    """One mesh with a material per face group: ``groups`` is {name: (face mask, surface inputs)}."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Asset")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("fidelity", note)
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Body")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(len(faces), 3))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.astype(np.int32).reshape(-1)))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
    for name, (mask, surface) in groups.items():
        material = UsdShade.Material.Define(stage, f"/Asset/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/Asset/Looks/{name}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        for key, value in surface.items():
            kind = Sdf.ValueTypeNames.Color3f if key == "diffuseColor" else Sdf.ValueTypeNames.Float
            shader.CreateInput(key, kind).Set(Gf.Vec3f(*value) if key == "diffuseColor" else float(value))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        subset = UsdGeom.Subset.CreateGeomSubset(mesh, name, UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(np.nonzero(mask)[0].astype(np.int32)),
                                                 UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
    stage.GetRootLayer().Save()


def build_d415(work: Path, cache: Path):
    points, faces = weld(d415_optical(read_stl(fetch(cache))))
    corners = points[faces]
    normal = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    front = (normal[:, 2] > 0.9) & (corners[:, :, 2].min(1) > points[:, 2].max() - 0.0008)       # the glass on the front face
    (work / "d415").mkdir()
    write_usd(work / "d415" / "d415.usdc", points, faces, {"Housing": (~front, ALUMINIUM), "FrontGlass": (front, GLASS)},
              "vendor mesh (RealSense ROS description); colour optical frame from the vendor's URDF offsets")
    low, high = points.min(0), points.max(0)
    return {"triangles": int(len(faces)), "glass_triangles": int(front.sum()), "size_mm": [round(float(v) * 1000, 2) for v in high - low],
            "min_mm": [round(float(v) * 1000, 2) for v in low], "max_mm": [round(float(v) * 1000, 2) for v in high]}


def build_bracket(work: Path, d415):
    config = json.loads((REPO / "exts/workcell/config/lab_hardware.json").read_text())["wrist_camera"]
    offset, rotation = config["offset_in_tool0_m"]["value"], config["rotation_in_tool0_deg"]["value"]
    if abs(offset[0]) > 1e-9 or rotation[1] != 0.0 or rotation[2] != 180.0 or rotation[0] > 0:
        raise SystemExit("the bracket model covers a camera on tool0 +y, turned 180 degrees about the tool axis and tilted toward it")
    (work / "wrist_mount").mkdir()
    # FreeCAD brings its own Python: this interpreter's variables (uv's virtual environment) must not reach it.
    clean = {key: value for key, value in os.environ.items() if not key.startswith(("PYTHON", "VIRTUAL_ENV", "UV_"))}
    env = dict(clean, BRACKET_OUT=str(work / "wrist_mount"), CAMERA_Y_MM=str(offset[1] * 1000), CAMERA_Z_MM=str(offset[2] * 1000),
               TILT_DEG=str(-rotation[0]), BACK_MM=str(d415["min_mm"][2]), BODY_CX_MM=str((d415["min_mm"][0] + d415["max_mm"][0]) / 2))
    done = subprocess.run(["freecadcmd", str(REPO / "exts/workcell/cad/wrist_bracket.py")], env=env, capture_output=True, text=True, timeout=600)
    step, stl = work / "wrist_mount" / "wrist_bracket.step", work / "wrist_mount" / "wrist_bracket.stl"
    if not (step.is_file() and stl.is_file()):
        raise SystemExit(f"freecadcmd did not write the bracket: {done.stdout[-800:]} {done.stderr[-800:]}")
    points, faces = weld(read_stl(stl) * 0.001)
    write_usd(work / "wrist_mount" / "wrist_mount.usdc", points, faces, {"Bracket": (np.ones(len(faces), bool), BRACKET)},
              "FreeCAD model in tool0 (exts/workcell/cad/wrist_bracket.py); placeholder design, fit on the real arm not measured")
    stl.unlink()
    return {"triangles": int(len(faces)), "camera": {"offset_m": offset, "rotation_deg": rotation},
            "summary": next((line for line in done.stdout.splitlines() if line.startswith("BRACKET")), "")}


def main() -> int:
    root = pack_manifest.resolve_asset_root().path
    if (root / PACK / TARGET / "pack.yaml").is_file():
        print(json.dumps({f"{PACK}/{TARGET}": "exists, left alone"}))
        return 0
    source = pack_manifest.AssetPack(root, PACK, SOURCE)                  # verifies every hash first
    folder = root / PACK / SOURCE
    manifest = pack_manifest.read_manifest(folder, PACK, SOURCE)
    work = Path(tempfile.mkdtemp(prefix="workcell-camera-"))
    try:
        d415 = build_d415(work, Path.home() / ".cache/twinsmith/downloads")
        bracket = build_bracket(work, d415)
        files, records = {}, {}
        for record in manifest["files"]:
            if record.get("asset_id") not in ("d415", "wrist_mount"):
                files[record["path"]], records[record["path"]] = folder / record["path"], record
        files["d415/d415.usdc"] = work / "d415" / "d415.usdc"
        records["d415/d415.usdc"] = dict(
            asset_id="d415", source=f"{MESH_URL} (sha256 {MESH_SHA256})", license="Apache-2.0 (realsenseai/realsense-ros)",
            redistributable=True, fidelity="Vendor mesh; optical frame from the vendor URDF offsets; materials are look only",
            changes="Converted by scripts/workcell_camera_assets.py: welded, moved to the colour optical frame, two materials.")
        common = dict(source="exts/workcell/cad/wrist_bracket.py (FreeCAD), for the camera pose in config/lab_hardware.json",
                      license="Locally authored lab geometry; private pack", redistributable=False,
                      fidelity="Placeholder design modelled as one solid; wrist diameter and fit on the real arm not measured")
        files["wrist_mount/wrist_mount.usdc"] = work / "wrist_mount" / "wrist_mount.usdc"
        records["wrist_mount/wrist_mount.usdc"] = dict(asset_id="wrist_mount", **common)
        files["wrist_mount/wrist_bracket.step"] = work / "wrist_mount" / "wrist_bracket.step"
        records["wrist_mount/wrist_bracket.step"] = dict(common)
        extra = {key: manifest[key] for key in ("mimic",) if key in manifest}
        extra.update(qualification=f"{PACK}/{SOURCE} with a vendor D415 body and a modelled wrist bracket", derived_from=f"{PACK}/{SOURCE}")
        written = asset_packs.write_version(root, PACK, TARGET, files, records, defaults={}, notes=[], extra=extra)
        print(json.dumps({"written": str(written)[:160], "d415": d415, "bracket": bracket}, indent=1))
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
