#!/usr/bin/env python3
"""The bench of pack v3, split so the robot's rail can move (used by ``scripts/workcell_bracket_assets.py``).

The bench asset is Vention's CAD of the lab's bench (``VentionAssembly_393251_v88``). It holds the linear axis the
robot stands on: a heavy-duty enclosed timing belt actuator (carriage group ``MO_LM_027_0001``, 1530 mm extrusion
``ST_EXT_007_0585_1530``, belt ``MO_LM_024_XXXX_1720``) with Vention's robot plate ``ST_RB_001_0002`` on its carriage.
In v2 all of it is one static asset. Here:

* ``bench/bench_full.usda`` is v2's ``bench.usda`` under another name: the whole assembly, render and collision layers.
* ``bench/bench.usda`` is the static bench: ``bench_full.usda`` under ``bench_cell.usda``, which deactivates the moving
  parts and the cable carrier, and gives the parts their look.
* ``rail_carriage/rail_carriage.usda`` is what moves: the carriage group and the robot plate, referenced from
  ``bench_full.usda`` (the vendor meshes stay where they are, in the private pack), in metres, with its origin on the
  robot plate's top face at the robot's axis, and the world's axes.

Left out of the static bench, besides the moving parts: the cable carrier (drag chain ``MO_DC_001``), its bracket on
the carriage and the tray it lies in with the tray's supports. The photos of the real cell show no cable carrier, and
cable management is not modelled.

Looks are set from the photos of the real cell (black rail, dark blue plates, a blue robot plate); they change nothing
in the physics. The carriage's mass is set by the work cell from its config, not here.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ASSEMBLY = "VentionAssembly_393251_v88_0"
MOVING = ("MO_LM_027_0001_78_robo_base1", "ST_RB_001_0002_400_robo_base2")       # carriage group, robot plate
CABLE_CARRIER = (
    "_0_6",                                                    # the drag chain, 27 links (MO_DC_001_000x)
    "ST_GP_001_0004_325",                                      # its bracket on the carriage
    "ST_EXT_101_1260_37",                                      # the tray it lies in, below the rail's outer side
    "ST_EXT_101_0135_39", "ST_EXT_101_0135_41", "ST_EXT_101_0135_333",          # the tray's three cross pieces
    "HW_EC_001_0001__2_42", "HW_EC_001_0001__2_44", "HW_EC_001_0001__2_45", "HW_EC_001_0001__2_46", "HW_EC_001_0001__2_328",   # end caps
    "ST_GP_001_0022_66", "ST_GP_001_0022_68", "ST_GP_001_0022_69", "ST_GP_001_0022_70",                  # corner plates
    "ST_HP_003_0001_151", "ST_HP_003_0001_152", "ST_HP_003_0001_329",           # connectors under the tray
)
ACTUATOR = ("_0_71", "MO_LM_024_XXXX_1720_133_belt")           # rail body with its end blocks; the belt
MOTOR = ("_0_337",)                                            # motor and gearbox at the rail's +X end
CONTROLS = ("_0_275", "_0_422")                                # controller and pendant boxes on the frame
PANEL = "imported_PANEL_B2F52AEB2A0D_804849_271"               # the table top: the work cell gives it its look
# Linear RGB, read off the photos of the real cell (2026-11-14). Looks only.
LOOKS = {
    "RailBlack": dict(diffuseColor=(0.012, 0.016, 0.020), metallic=0.5, roughness=0.42),        # the actuator's extrusion
    "BeltBlack": dict(diffuseColor=(0.010, 0.010, 0.011), metallic=0.0, roughness=0.8),
    "FrameNavy": dict(diffuseColor=(0.010, 0.022, 0.045), metallic=0.5, roughness=0.42),        # the bench's own extrusions
    "MotorBlack": dict(diffuseColor=(0.020, 0.020, 0.022), metallic=0.2, roughness=0.6),
    "CarriagePlate": dict(diffuseColor=(0.018, 0.032, 0.052), metallic=0.4, roughness=0.5),     # dark blue hole-grid plate
    "RobotPlate": dict(diffuseColor=(0.035, 0.050, 0.130), metallic=0.4, roughness=0.5),        # blue plate under the base
}


def _material(stage, scope, name):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, scope.AppendChild(name))
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    for key, value in LOOKS[name].items():
        kind = Sdf.ValueTypeNames.Color3f if key == "diffuseColor" else Sdf.ValueTypeNames.Float
        shader.CreateInput(key, kind).Set(Gf.Vec3f(*value) if key == "diffuseColor" else float(value))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def build_bench(source: Path, dest: Path) -> dict:
    """Write the static bench into ``dest`` (a ``bench`` folder) from v2's ``source`` folder; returns what was done."""
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    dest.mkdir(parents=True)
    for name in ("bench_render.usdc", "bench_collision.usdc"):
        shutil.copyfile(source / name, dest / name)
    shutil.copyfile(source / "bench.usda", dest / "bench_full.usda")
    full = Usd.Stage.Open(str(dest / "bench_full.usda"))
    root = full.GetDefaultPrim()
    assembly = root.GetChild(ASSEMBLY)
    names = [child.GetName() for child in assembly.GetChildren()]
    missing = [name for name in (*MOVING, *CABLE_CARRIER, *ACTUATOR, *MOTOR, *CONTROLS, PANEL) if name not in names]
    if missing:
        raise SystemExit(f"the bench is not the expected Vention assembly: {missing} not found")

    cell = Sdf.Layer.CreateNew(str(dest / "bench_cell.usda"))
    top = Sdf.Layer.CreateNew(str(dest / "bench.usda"))
    top.subLayerPaths[:] = ["./bench_cell.usda", "./bench_full.usda"]
    top.defaultPrim = root.GetName()
    for layer in (cell, top):
        pseudo = layer.pseudoRoot
        pseudo.SetInfo("metersPerUnit", UsdGeom.GetStageMetersPerUnit(full))
        pseudo.SetInfo("upAxis", UsdGeom.GetStageUpAxis(full))
    top.Save()
    stage = Usd.Stage.Open(str(dest / "bench.usda"))
    stage.SetEditTarget(cell)
    base = stage.GetDefaultPrim().GetChild(ASSEMBLY).GetPath()
    for name in (*MOVING, *CABLE_CARRIER):
        stage.OverridePrim(base.AppendChild(name)).SetActive(False)
    scope = UsdGeom.Scope.Define(stage, stage.GetDefaultPrim().GetPath().AppendChild("CellLooks")).GetPath()
    made = {name: _material(stage, scope, name) for name in ("RailBlack", "BeltBlack", "FrameNavy", "MotorBlack")}
    looks = {}
    for name in names:
        if name in MOVING or name in CABLE_CARRIER or name == PANEL:
            continue
        look = ("BeltBlack" if name == ACTUATOR[1] else "RailBlack" if name == ACTUATOR[0]
                else "MotorBlack" if name in MOTOR or name in CONTROLS else "FrameNavy")
        prim = stage.OverridePrim(base.AppendChild(name))
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(made[look], bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        looks[look] = looks.get(look, 0) + 1
    cell.Save()
    return {"static_parts": len(names) - len(MOVING) - len(CABLE_CARRIER), "moved_to_rail_carriage": list(MOVING),
            "cable_carrier_left_out": list(CABLE_CARRIER), "looks": looks}


def build_carriage(bench: Path, dest: Path, config: dict) -> dict:
    """Write ``dest`` (``rail_carriage.usda``): the carriage group and the robot plate as one asset in metres.

    The parts are referenced from ``bench_full.usda`` and put where the bench's placement in the cell puts them, with
    the origin moved to the robot's base (``robot.placement``), so the builder can place the asset at the robot.
    """
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    world = config["world"]
    xy, scale, top = world["workstation_xy_m"]["value"], world["bench_scale"]["value"], world["bench_asset_top_z_m"]["value"]
    base = config["robot"]["placement"]["value"]["position_m"]
    full = Usd.Stage.Open(str(bench / "bench_full.usda"))
    assembly = full.GetDefaultPrim().GetChild(ASSEMBLY)
    cache = UsdGeom.XformCache()
    parent = cache.GetLocalToWorldTransform(assembly)                    # what a reference to a part leaves behind
    if abs(UsdGeom.GetStageMetersPerUnit(full) - scale) > 1e-12:
        raise SystemExit("world.bench_scale is not the bench asset's metersPerUnit")

    dest.parent.mkdir(parents=True)
    stage = Usd.Stage.CreateNew(str(dest))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("fidelity", "Vention's CAD of the lab's bench: the linear axis's carriage and the robot plate on it; "
                                        "referenced from bench_full.usda; looks from photos")
    geometry = UsdGeom.Xform.Define(stage, "/Asset/geometry")           # the bench's units and placement, origin at the robot's base
    geometry.AddTranslateOp().Set(Gf.Vec3d(xy[0] - base[0], xy[1] - base[1], -top - base[2]))
    geometry.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    looks = UsdGeom.Scope.Define(stage, "/Asset/Looks").GetPath()
    relative = "../bench/bench_full.usda"
    report, low, high = {}, np.full(3, 1e9), np.full(3, -1e9)
    for name, look in ((MOVING[0], "CarriagePlate"), (MOVING[1], "RobotPlate")):
        part = UsdGeom.Xform.Define(stage, f"/Asset/geometry/{name}")
        part.MakeMatrixXform().Set(parent)
        inner = stage.DefinePrim(part.GetPath().AppendChild("part"))
        inner.GetReferences().AddReference(relative, assembly.GetPath().AppendChild(name))
        UsdShade.MaterialBindingAPI.Apply(part.GetPrim()).Bind(_material(stage, looks, look), bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    stage.GetRootLayer().Save()
    check = Usd.Stage.Open(str(dest))
    boxes = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    for name in MOVING:
        box = boxes.ComputeWorldBound(check.GetPrimAtPath(f"/Asset/geometry/{name}")).ComputeAlignedRange()
        if box.IsEmpty():
            raise SystemExit(f"{name} did not compose in the carriage asset")
        report[name] = {"min_m": [round(v, 4) for v in box.GetMin()], "max_m": [round(v, 4) for v in box.GetMax()]}
        low, high = np.minimum(low, box.GetMin()), np.maximum(high, box.GetMax())
    plate = report[MOVING[1]]
    if abs(plate["max_m"][2]) > 0.006 or max(abs(plate["min_m"][0] + plate["max_m"][0]), abs(plate["min_m"][1] + plate["max_m"][1])) > 0.004:
        raise SystemExit(f"the robot plate's top is not at the robot's base ({plate}): check robot.placement against the bench")
    return {"parts": report, "bounds_m": [[round(float(v), 4) for v in low], [round(float(v), 4) for v in high]],
            "origin": "the robot's base: on the robot plate's top face at the robot's axis; the world's axes"}


def manifest_record(asset_id, what) -> dict:
    return dict(asset_id=asset_id, source="Vention's CAD of the lab's bench (VentionAssembly_393251_v88), from pack v2's bench/ files",
                license="Vention's CAD of the lab's own bench; private pack only", redistributable=False,
                fidelity="Vendor CAD, flattened; looks from photos of the real cell; colliders are one box per part",
                changes=what)
