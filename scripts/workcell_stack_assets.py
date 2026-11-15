#!/usr/bin/env python3
"""The two parts between the UR5e's tool flange and the RG2, from the vendors' official CAD (used by
``scripts/workcell_bracket_assets.py`` for pack v3).

* ``quick_changer``: OnRobot's Quick Changer, robot side, "Quick Changer Robot side" v2 with cable holder,
  https://onrobot.com/storage/quick-changer-robot-side_v2_with_cable_holder.step, linked as "Step file" from
  https://onrobot.com/en/products/quick-changer. The tool side is part of the RG2 (it is in OnRobot's RG2 STEP).
  The large cable holder around the connector (parts 113292 and 113293) is left out: the photos of the real wrist
  show the connector free and only the small clip on the opposite side (part 102899), which is kept.
* ``bota``: Bota Systems' Rokubi with the connector on the side (the lab's BFT-ROKS-SER-M8), "ROK-A000",
  https://shop.botasys.com/shop/rokubi-medusa-gen-0-series-kit-1877/document/180, listed as
  "Rokubi/Medusa Gen 0 Series Side.step" on https://shop.botasys.com/shop/rokubi-medusa-gen-0-series-kit-1877.
  The sensor is not on the real arm (``bota.mounted`` is false in the config); the asset is there for the switch.

Neither vendor states redistribution terms for these files: each is read from a local path (``TWINSMITH_QC_STEP``,
``TWINSMITH_BOTA_STEP``), checked against the SHA-256 pinned here, and its geometry goes into the private pack only.
Nothing of them is kept in this repository. ``exts/workcell/cad/part_from_step.py`` (FreeCAD) does the reading.

Frames, as the pack had them: each part's +X is the tool axis. The Quick Changer's origin is on the flange plane (its
seat for the flange); its turn about the tool axis is the one OnRobot's RG2 STEP shows it in, connector on the RG2's
finger-opening axis. The Rokubi's origin is its centre, 16 mm from each face; the face with the sensing element is
taken as the tool side and the connector's direction about the axis is arbitrary (neither is confirmed for the lab's
unit). Masses are set by the work cell from its config, not here.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

import workcell_camera_assets as camera_assets  # noqa: E402  (STL reading and welding)
from workcell_rg2_assets import convex_hull  # noqa: E402

CAD = "exts/workcell/cad/part_from_step.py"
LOOKS = {"blue": dict(diffuseColor=(0.02, 0.27, 0.62), metallic=1.0, roughness=0.32),             # anodised ring
         "aluminium": dict(diffuseColor=(0.70, 0.71, 0.73), metallic=1.0, roughness=0.42),
         "steel": dict(diffuseColor=(0.55, 0.56, 0.58), metallic=1.0, roughness=0.3),
         "black": dict(diffuseColor=(0.02, 0.02, 0.022), metallic=0.0, roughness=0.5),
         "anodised_black": dict(diffuseColor=(0.03, 0.03, 0.035), metallic=1.0, roughness=0.45)}
PARTS = {
    "quick_changer": dict(
        env="TWINSMITH_QC_STEP", url="https://onrobot.com/storage/quick-changer-robot-side_v2_with_cable_holder.step",
        page="https://onrobot.com/en/products/quick-changer", name="Quick Changer Robot side (v2, with cable holder)",
        sha256="ac3cb44b90b27b172423400def5150a9af8b6e9338baf358f413118cbf5f38db",
        root="onrobot_quick_changer", body="quick_changer_link", file="quick_changer.usdc",
        # CAD: +Y out of the robot's flange toward the robot, the flange seat at y = 0, the connector at -Z, the rod at -X.
        spec=dict(frame=[[0, -1, 0], [0, 0, 1], [-1, 0, 0]], origin_mm=[0, 0, 0], default_material="blue",
                  drop=["113292", "113293", "102010"], materials={"100022": "steel", "101396": "aluminium", "102689": "steel", "102899": "black"},
                  collider=dict(labels=["102697"], clip_x_mm=[0.0, 13.6]), caps={"blue": 12000, "black": 3000, "aluminium": 1500, "steel": 800}),
        expect=dict(height_mm=13.6, diameter_mm=71.0),
        licence="OnRobot's CAD; redistribution terms not known. Private pack only: do not publish or redistribute",
        left_out="the large cable holder (113292, 113293), which the photos of the real wrist do not show, and a label (102010)"),
    "bota": dict(
        env="TWINSMITH_BOTA_STEP", url="https://shop.botasys.com/shop/rokubi-medusa-gen-0-series-kit-1877/document/180",
        page="https://shop.botasys.com/shop/rokubi-medusa-gen-0-series-kit-1877", name="Rokubi/Medusa Gen 0 Series Side (ROK-A000)",
        sha256="90cb4bd0551a8090428c7fa470b3c3bcb1cf3c7e60e8363cc976efd8a82c7063",
        root="Asset", body=None, file="bota.usdc",
        # CAD: +Z from the electronics cap to the sensing element, the body's axis at (1.265, 1.028), faces at z = -34.733 and -2.733.
        spec=dict(frame=[[0, 0, 1], [1, 0, 0], [0, 1, 0]], origin_mm=[1.265, 1.028, -18.733], default_material="anodised_black",
                  drop=["Connector", "ISO-7046", "PAR-00006", "PAR-00009", "ORING", "ROK-P004"], min_volume_mm3=50.0,
                  materials={"ROK-P000": "aluminium", "ROK-P003": "steel"},
                  collider=dict(labels=["ROK-P014", "ROK-P000"], clip_x_mm=[-16.0, 16.0]), caps={"anodised_black": 9000, "aluminium": 4000, "steel": 1500}),
        expect=dict(height_mm=32.0, diameter_mm=48.0),
        licence="Bota Systems' CAD; redistribution terms not known. Private pack only: do not publish or redistribute",
        left_out="connector pins, screws, light pipes and seals"),
}


def step_path(role, environ=None) -> Path:
    """The local copy of a vendor's STEP, checked against the pinned SHA-256; nothing is downloaded here."""
    part = PARTS[role]
    value = (os.environ if environ is None else environ).get(part["env"])
    if not value:
        raise SystemExit(f"set {part['env']} to a local copy of {part['url']} (listed on {part['page']})")
    path = Path(value).expanduser()
    if not path.is_file():
        raise SystemExit(f"{part['env']} does not name a file")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    if digest.hexdigest() != part["sha256"]:
        raise SystemExit(f"the {role} STEP does not match its pinned SHA-256; nothing was built")
    return path


def run_cad(role, step: Path, out: Path) -> dict:
    # FreeCAD brings its own Python: this interpreter's variables (uv's virtual environment) must not reach it.
    clean = {key: value for key, value in os.environ.items() if not key.startswith(("PYTHON", "VIRTUAL_ENV", "UV_"))}
    env = dict(clean, STEP_SRC=str(step), PART_OUT=str(out), PART_SPEC=json.dumps(PARTS[role]["spec"]))
    done = subprocess.run(["freecadcmd", str(REPO / CAD)], env=env, capture_output=True, text=True, errors="replace", timeout=1800)
    report = out / "part.json"
    if done.returncode or not report.is_file():
        raise SystemExit(f"freecadcmd did not convert the {role}: {done.stdout[-600:]} {done.stderr[-600:]}")
    return json.loads(report.read_text())


def outline_prism(points, bins=32):
    """A convex prism around ``points`` (metres): their outline about the tool axis (+X), in ``bins`` directions,
    between their two ends along it. Exact in height, never inside the outline, at most ``2 * bins`` corners."""
    low, high = points[:, 0].min(), points[:, 0].max()
    radius = np.hypot(points[:, 1], points[:, 2])
    sector = np.floor((np.arctan2(points[:, 2], points[:, 1]) + np.pi) / (2 * np.pi) * bins).astype(int) % bins
    corners = []
    for index in range(bins):
        inside = sector == index
        if inside.any():                                 # a corner on the sector's middle, far enough out to hold its arc
            angle, reach = (index + 0.5) / bins * 2 * np.pi - np.pi, radius[inside].max() / np.cos(np.pi / bins)
            corners += [[end, reach * np.cos(angle), reach * np.sin(angle)] for end in (low, high)]
    return convex_hull(np.array(corners), grid=1e-6)


def build_part(role, cad: Path, dest: Path) -> dict:
    """Write ``dest``: one rigid body with a render mesh (a material per face group) and one convex collider."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    part, report = PARTS[role], json.loads((cad / "part.json").read_text())
    stage = Usd.Stage.CreateNew(str(dest))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{part['root']}").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("fidelity", f"{part['name']}: the vendor's official STEP ({part['url']}, sha256 {part['sha256']}); "
                                        "decimated; materials are look only")
    body = UsdGeom.Xform.Define(stage, root.GetPath().AppendChild(part["body"])).GetPrim() if part["body"] else root
    UsdPhysics.RigidBodyAPI.Apply(body)
    looks = UsdGeom.Scope.Define(stage, root.GetPath().AppendChild("Looks")).GetPath()
    points, faces, subsets, offset = [], [], {}, 0
    for path in sorted(cad.glob("*.stl")):
        welded, indices = camera_assets.weld(camera_assets.read_stl(path).reshape(-1, 3, 3) / 1000.0)
        subsets[path.stem] = (len(faces), len(faces) + len(indices))
        points.append(welded)
        faces.extend((indices + offset).tolist())
        offset += len(welded)
    points, faces = np.vstack(points), np.array(faces)
    mesh = UsdGeom.Mesh.Define(stage, body.GetPath().AppendChild("visuals"))
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(len(faces), 3))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.astype(np.int32).reshape(-1)))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
    for name, (first, last) in subsets.items():
        material = UsdShade.Material.Define(stage, looks.AppendChild(name.capitalize()))
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        for key, value in LOOKS[name].items():
            kind = Sdf.ValueTypeNames.Color3f if key == "diffuseColor" else Sdf.ValueTypeNames.Float
            shader.CreateInput(key, kind).Set(Gf.Vec3f(*value) if key == "diffuseColor" else float(value))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        subset = UsdGeom.Subset.CreateGeomSubset(mesh, name, UsdGeom.Tokens.face, Vt.IntArray(list(range(first, last))), UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
    vertices, hull_faces = outline_prism(np.array(json.loads((cad / "collider_points.json").read_text())) / 1000.0)
    collider = UsdGeom.Mesh.Define(stage, body.GetPath().AppendChild("collisions"))
    collider.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)))
    collider.CreateFaceVertexCountsAttr(Vt.IntArray(len(hull_faces), 3))
    collider.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(hull_faces.astype(np.int32).reshape(-1)))
    collider.CreateSubdivisionSchemeAttr("none")
    collider.CreateExtentAttr([Gf.Vec3f(*vertices.min(0)), Gf.Vec3f(*vertices.max(0))])
    collider.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr("convexHull")
    stage.GetRootLayer().Save()

    low, high = vertices.min(0) * 1000.0, vertices.max(0) * 1000.0
    height, diameter = float(high[0] - low[0]), float(min(high[1] - low[1], high[2] - low[2]))     # a boss may widen one direction
    for key, measured in (("height_mm", height), ("diameter_mm", diameter)):
        if abs(measured - part["expect"][key]) > 0.3:
            raise SystemExit(f"{role}: {key} is {measured:.2f}, not the datasheet's {part['expect'][key]}; this is not the expected STEP")
    return {"root_prim": f"/{part['root']}", "body": part["body"] or ".", "triangles": int(len(faces)),
            "triangles_before_decimation": report["triangles_before_decimation"], "collider_hull_vertices": int(len(vertices)),
            "body_height_mm": round(height, 2), "body_diameter_mm": round(diameter, 2),
            "body_along_tool_axis_mm": [round(float(low[0]), 2), round(float(high[0]), 2)],
            "bounds_mm": report["bounds_mm"], "volume_cm3": report["volume_cm3"], "centre_of_volume_mm": report["centre_of_volume_mm"],
            "parts": report["kept"], "left_out": report["left_out"]}


def manifest_record(role) -> dict:
    part = PARTS[role]
    return dict(asset_id=role, source=f"{part['name']}: the vendor's official STEP ({part['url']}, listed on {part['page']}; sha256 {part['sha256']})",
                license=part["licence"], redistributable=False,
                fidelity="Official vendor CAD, tessellated and decimated; materials are look only; the mass comes from the work "
                         "cell's config, not from the CAD",
                changes=f"Converted by {CAD} and scripts/workcell_stack_assets.py: one rigid body, +X along the tool axis, a render "
                        f"mesh and one convex collider (a prism around the body, 32 sides). Left out: {part['left_out']}.")
