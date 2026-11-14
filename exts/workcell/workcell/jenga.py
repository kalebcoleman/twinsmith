"""Build and reset the workcell's externally sourced Jenga tower.

``realize(config, profile=None, seed=None)`` is a CPU-only preflight. Optional
arguments override ``jenga.variation_profile`` and ``jenga.variation_seed``.
A selected profile requires an explicit integer seed and an entry in
``jenga.variation_profiles`` with exactly these fields: ``mass_kg``,
``static_friction``, ``dynamic_friction``, and ``size_tolerance_m``. The first
three are distribution objects; size_tolerance_m is three distribution objects
for additive length/width/height deviations from 72 x 24 x 14 mm:

* {"distribution": "constant", "value": number}
* {"distribution": "uniform", "min": number, "max": number}
* {"distribution": "normal", "mean": number, "std": number,
   "min": number, "max": number}

Normal bounds are mandatory, std is positive, and mean lies within the bounds.
Normals and friction pairs use at most MAX_SAMPLE_ATTEMPTS rejection attempts,
never clipping. Friction pairs are conditioned on dynamic <= static. Tolerances
are capped at +/-1 mm per axis as a software guard, not a measured wood spec.
Only the selected profile is validated. Without one, the existing config's
mass/frictions and nominal dimensions are used; no calibration is inferred.
"""

import math
import random
import struct


NOMINAL_SIZE_M = (0.072, 0.024, 0.014)
MAX_SIZE_TOLERANCE_M = 0.001
MAX_SAMPLE_ATTEMPTS = 128
_FLOAT_MAX = 3.4028234663852886e38


def _number(value, label):
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be a finite number")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def _distribution(spec, label, minimum, maximum, positive=False):
    fields = {"constant": {"distribution", "value"},
              "uniform": {"distribution", "min", "max"},
              "normal": {"distribution", "mean", "std", "min", "max"}}
    if not isinstance(spec, dict) or not isinstance(spec.get("distribution"), str):
        raise ValueError(f"{label} must be a named distribution object")
    kind = spec["distribution"]
    if kind not in fields or set(spec) != fields[kind]:
        raise ValueError(f"{label}: invalid {kind!r} distribution fields")
    result = {key: _number(value, f"{label}.{key}") if key != "distribution" else value
              for key, value in spec.items()}
    lower = result["value"] if kind == "constant" else result["min"]
    upper = result["value"] if kind == "constant" else result["max"]
    if not minimum <= lower <= upper <= maximum or (positive and lower <= 0):
        raise ValueError(f"{label}: invalid distribution bounds")
    if positive and struct.unpack("f", struct.pack("f", lower))[0] <= 0:
        raise ValueError(f"{label}: positive mass must be representable in USD float storage")
    if kind == "normal" and (lower == upper or result["std"] <= 0
                              or not lower <= result["mean"] <= upper):
        raise ValueError(f"{label}: normal needs min < max, positive std and bounded mean")
    return result


def _draw(rng, spec):
    kind = spec["distribution"]
    if kind == "constant":
        return spec["value"]
    if kind == "uniform":
        return rng.uniform(spec["min"], spec["max"])
    for _ in range(MAX_SAMPLE_ATTEMPTS):
        value = rng.gauss(spec["mean"], spec["std"])
        if spec["min"] <= value <= spec["max"]:
            return value
    raise ValueError(f"Bounded normal exhausted {MAX_SAMPLE_ATTEMPTS} rejection attempts")


def _bounds(spec):
    if spec["distribution"] == "constant":
        return spec["value"], spec["value"]
    return spec["min"], spec["max"]


def _layers(config):
    layers = config["jenga"]["tower_layers_built"]
    if type(layers) is not int or not 1 <= layers <= 18:
        raise ValueError("jenga.tower_layers_built must be an integer from 1 to 18")
    return layers


def _poses(config):
    """Loose blocks lying flat, ``[[x, y, yaw_deg], ...]``, for a task that is not a tower; None for a tower."""
    settings = config["jenga"]
    poses = settings.get("block_poses")
    if poses is None:
        return None
    if "tower_layers_built" in settings or "tower_xy_m" in settings:
        raise ValueError("jenga.block_poses and a tower (tower_layers_built, tower_xy_m) cannot both be given")
    if not isinstance(poses, list) or not 1 <= len(poses) <= 54 or not all(
            isinstance(pose, list) and len(pose) == 3 for pose in poses):
        raise ValueError("jenga.block_poses must be 1 to 54 [x, y, yaw_deg] entries")
    return [[_number(value, "jenga.block_poses") for value in pose] for pose in poses]


def block_count(config):
    poses = _poses(config)
    return len(poses) if poses is not None else _layers(config) * 3


def realize(config, profile=None, seed=None):
    """Validate and sample the whole tower without USD imports or stage edits."""
    settings = config["jenga"]
    poses = _poses(config)
    layers = 0 if poses is not None else _layers(config)
    _distribution({"distribution": "constant", "value": settings["block_mass_kg"]},
                  "block_mass_kg", 0, _FLOAT_MAX, positive=True)
    if tuple(settings["block_size_m"]) != NOMINAL_SIZE_M:
        raise ValueError("jenga.block_size_m must be nominal 72 x 24 x 14 mm")
    if poses is None:
        if len(settings["tower_xy_m"]) != 2:
            raise ValueError("jenga.tower_xy_m must contain two finite coordinates")
        center_x, center_y = [_number(value, "jenga.tower_xy_m") for value in settings["tower_xy_m"]]
    profile = settings.get("variation_profile") if profile is None else profile
    seed = settings.get("variation_seed") if seed is None else seed
    if seed is not None and type(seed) is not int:
        raise ValueError("jenga.variation_seed must be an integer")
    if profile is None:
        specs = {key: {"distribution": "constant", "value": settings[source]}
                 for key, source in (("mass_kg", "block_mass_kg"),
                                     ("static_friction", "static_friction"),
                                     ("dynamic_friction", "dynamic_friction"))}
        specs["size_tolerance_m"] = [{"distribution": "constant", "value": 0.0} for _ in range(3)]
    else:
        profiles = settings.get("variation_profiles", {})
        if not isinstance(profile, str) or not isinstance(profiles, dict) or profile not in profiles:
            raise ValueError("Unknown jenga.variation_profile")
        if seed is None:
            raise ValueError("A variation profile requires an explicit integer variation_seed")
        specs = profiles[profile]
    required = {"mass_kg", "static_friction", "dynamic_friction", "size_tolerance_m"}
    if not isinstance(specs, dict) or set(specs) != required:
        raise ValueError(f"Jenga profile must contain exactly {sorted(required)}")
    distributions = {key: _distribution(specs[key], key, 0, _FLOAT_MAX, positive=key == "mass_kg")
                     for key in ("mass_kg", "static_friction", "dynamic_friction")}
    tolerances = specs["size_tolerance_m"]
    if not isinstance(tolerances, (list, tuple)) or len(tolerances) != 3:
        raise ValueError("size_tolerance_m must contain three distributions")
    tolerances = [_distribution(spec, f"size_tolerance_m[{axis}]",
                               -MAX_SIZE_TOLERANCE_M, MAX_SIZE_TOLERANCE_M)
                  for axis, spec in enumerate(tolerances)]
    if _bounds(distributions["dynamic_friction"])[0] > _bounds(distributions["static_friction"])[1]:
        raise ValueError("Friction distributions cannot satisfy dynamic <= static")
    rng = random.Random(0 if seed is None else seed)
    blocks = []
    for index in range(block_count(config)):
        try:
            mass = _draw(rng, distributions["mass_kg"])
            for _ in range(MAX_SAMPLE_ATTEMPTS):
                static = _draw(rng, distributions["static_friction"])
                dynamic = _draw(rng, distributions["dynamic_friction"])
                if dynamic <= static:
                    break
            else:
                raise ValueError(f"Friction pair exhausted {MAX_SAMPLE_ATTEMPTS} rejection attempts")
            offsets = [_draw(rng, spec) for spec in tolerances]
            size = [nominal + offset for nominal, offset in zip(NOMINAL_SIZE_M, offsets)]
            if not all(math.isfinite(value) and value > 0 for value in size):
                raise ValueError("Realized dimensions must be finite and positive")
        except ValueError as exc:
            raise ValueError(f"Jenga block {index + 1}: {exc}") from exc
        blocks.append({"prim_path": f"/World/Jenga/Block_{index + 1:02d}",
                       "material_path": f"/World/Jenga/Materials/Block_{index + 1:02d}",
                       "layer": index // 3, "slot": index % 3, "mass_kg": mass,
                       "static_friction": static, "dynamic_friction": dynamic,
                       "size_m": size, "size_tolerance_m": offsets})
    for block, pose in zip(blocks, poses or ()):
        block.update(layer=0, slot=0, position_m=[pose[0], pose[1], block["size_m"][2] / 2], rotation_z_deg=pose[2])
    bottom = 0.0
    for layer in range(layers):
        row = blocks[layer * 3:layer * 3 + 3]
        edge = -sum(block["size_m"][1] for block in row) / 2
        for block in row:
            width, height = block["size_m"][1:]
            offset = edge + width / 2
            position = [center_x + (offset if layer % 2 else 0),
                        center_y + (0 if layer % 2 else offset), bottom + height / 2]
            if not all(math.isfinite(value) for value in position):
                raise ValueError("Realized positions must be finite")
            block.update(position_m=position, rotation_z_deg=90.0 if layer % 2 else 0.0)
            edge += width
        bottom += max(block["size_m"][2] for block in row)
    return {"variation_profile": profile, "variation_seed": seed, "blocks": blocks,
            "sampling_policy": {"rng": "random.Random", "max_rejection_attempts": MAX_SAMPLE_ATTEMPTS,
                                "normal": "reject outside inclusive min/max; fail on exhaustion",
                                "friction": "resample both coefficients until dynamic <= static",
                                "max_size_tolerance_m": MAX_SIZE_TOLERANCE_M}}


def positions(config):
    settings = config["jenga"]
    length, width, height = settings["block_size_m"]
    center_x, center_y = settings["tower_xy_m"]
    return [((center_x, center_y + (slot - 1) * width, (layer + 0.5) * height), 0.0)
            if layer % 2 == 0 else
            ((center_x + (slot - 1) * width, center_y, (layer + 0.5) * height), 90.0)
            for layer in range(settings["tower_layers_built"]) for slot in range(3)]


def build(stage, asset, config, profile=None, seed=None):
    """Preflight all samples and asset geometry before authoring the tower."""
    report = realize(config, profile, seed)
    asset_id = config["assets"]["jenga_block"]
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    if (UsdGeom.GetStageMetersPerUnit(stage) != 1
            or UsdPhysics.GetStageKilogramsPerUnit(stage) != 1
            or UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z):
        raise ValueError("Jenga requires a Z-up stage in metres and kilograms")
    if stage.GetPrimAtPath("/World/Jenga"):
        raise ValueError("Jenga tower already exists; use reset rather than rebuilding in place")
    probe = Usd.Stage.CreateInMemory("jenga-preflight.usda", stage.GetPathResolverContext())
    probe_root = UsdGeom.Xform.Define(probe, "/Probe")
    source = UsdGeom.Xform.Define(probe, "/Probe/Source")
    source.GetPrim().GetReferences().AddReference(str(asset))
    if probe.GetCompositionErrors():
        raise ValueError("Jenga asset has unresolved USD composition")
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeLocalBound(probe_root.GetPrim()).ComputeAlignedRange()
    if bounds.IsEmpty():
        raise ValueError(f"Jenga asset has no visual bounds: {asset}")
    actual = bounds.GetSize()
    if any(not math.isfinite(actual[axis]) or actual[axis] <= 0
           or abs(actual[axis] - NOMINAL_SIZE_M[axis]) > 0.001 for axis in range(3)):
        raise ValueError(f"Jenga asset bounds {tuple(actual)} do not match {NOMINAL_SIZE_M} metres")
    center = (bounds.GetMin() + bounds.GetMax()) * 0.5
    if not all(math.isfinite(value) for value in center):
        raise ValueError("Jenga asset center must be finite")
    if not any(prim.IsA(UsdGeom.Mesh) for prim in Usd.PrimRange(source.GetPrim())):
        raise ValueError("Jenga asset must contain mesh geometry for collision")
    tower = UsdGeom.Xform.Define(stage, "/World/Jenga").GetPrim()
    tower.CreateAttribute("jenga:blockCount", Sdf.ValueTypeNames.Int, custom=True).Set(len(report["blocks"]))
    UsdGeom.Scope.Define(stage, "/World/Jenga/Materials")
    parts = []
    for block in report["blocks"]:
        path = block["prim_path"]
        root = UsdGeom.Xform.Define(stage, path)
        position = Gf.Vec3d(*block["position_m"])
        root.AddTranslateOp().Set(position)
        root.AddRotateZOp().Set(block["rotation_z_deg"])
        root.GetPrim().CreateAttribute("jenga:initialPosition", Sdf.ValueTypeNames.Double3, custom=True).Set(position)
        root.GetPrim().CreateAttribute("jenga:initialRotationZ", Sdf.ValueTypeNames.Double, custom=True).Set(block["rotation_z_deg"])
        root.GetPrim().CreateAttribute("jenga:sizeM", Sdf.ValueTypeNames.Double3, custom=True).Set(Gf.Vec3d(*block["size_m"]))
        shape = UsdGeom.Xform.Define(stage, path + "/Asset")
        shape.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[block["size_m"][axis] / actual[axis] for axis in range(3)]))
        shape.AddTranslateOp().Set(-center)
        UsdGeom.Xform.Define(stage, path + "/Asset/Source").GetPrim().GetReferences().AddReference(str(asset))
        meshes = [prim for prim in Usd.PrimRange(shape.GetPrim()) if prim.IsA(UsdGeom.Mesh)]
        for mesh in meshes:
            UsdPhysics.CollisionAPI.Apply(mesh)
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr().Set("convexHull")
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(block["mass_kg"])
        material = UsdShade.Material.Define(stage, block["material_path"])
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(block["static_friction"])
        physics_material.CreateDynamicFrictionAttr().Set(block["dynamic_friction"])
        UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
        parts.append({"prim_path": path, "asset_id": asset_id, "bom_id": "jenga_block"})
    report.update(parts=parts, layers=config["jenga"].get("tower_layers_built", 0), block_count=len(parts),
                  block_size_m=list(config["jenga"]["block_size_m"]), block_mass_kg=config["jenga"]["block_mass_kg"],
                  limitations=["Distribution parameters are inputs, not measured calibration or provenance.",
                               "Block dimensions, mass, convex-hull collision and friction require physical validation.",
                               "Layers start above the tallest block below; shorter blocks leave gaps requiring settling.",
                               "Report mass/friction are sampled values; USD physics attributes use float precision.",
                               "Reset restores authored realized poses only, not velocities or solver state."])
    return report


def reset(stage, config):
    """Reset authored block poses on a stopped timeline; caller controls the timeline."""
    from pxr import Gf, UsdGeom

    count = block_count(config)
    tower = stage.GetPrimAtPath("/World/Jenga")
    if not tower or tower.GetAttribute("jenga:blockCount").Get() != count:
        raise ValueError("Jenga tower does not match the configuration or lacks realized reset metadata")
    if len([prim for prim in tower.GetChildren() if prim.GetName().startswith("Block_")]) != count:
        raise ValueError("Jenga tower does not match the configuration")
    targets = []
    for index in range(count):
        prim = stage.GetPrimAtPath(f"/World/Jenga/Block_{index + 1:02d}")
        if not prim or not prim.IsValid():
            raise ValueError("Jenga tower does not match the configuration")
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        if (len(ops) != 2 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTranslate
                or ops[1].GetOpType() != UsdGeom.XformOp.TypeRotateZ
                or any(operation.IsInverseOp() for operation in ops)):
            raise ValueError("Unexpected Jenga block transform")
        position = prim.GetAttribute("jenga:initialPosition").Get()
        angle = prim.GetAttribute("jenga:initialRotationZ").Get()
        if (position is None or len(position) != 3 or not all(math.isfinite(value) for value in position)
                or angle is None or not math.isfinite(angle)):
            raise ValueError("Jenga block lacks valid realized reset metadata")
        targets.append((ops, position, angle))
    for ops, position, angle in targets:
        ops[0].Set(Gf.Vec3d(*position))
        ops[1].Set(angle)
