"""Place the pieces of an assembly puzzle: base plates with slots and the pieces that go into them.

A puzzle task's ``puzzle`` section lists ``pieces``. Each has a ``name``, the ``asset`` role it is drawn from,
``position_m`` and ``rpy_deg`` (fixed-axis roll, pitch, yaw; USD's ``rotateXYZ``), and optionally ``parent``: the name
of an earlier piece whose frame the pose is in, so the pieces assembled into a base move with it. Without a parent the
pose is in the world, whose z = 0 is the bench top. A piece's frame is its asset's own.

``realize(config)`` is a CPU-only preflight that resolves every pose into the world (``rotation`` is the 3 x 3 matrix
for column vectors). ``build`` then references each
piece's asset as one rigid body. The asset brings its own collision shapes (the puzzles pack's are analytic boxes) and
its solid volume; the mass is that volume times ``density_kg_m3``.
"""

import math

KEYS = {"pieces", "density_kg_m3", "static_friction", "dynamic_friction"}
PIECE_KEYS = {"name", "asset", "position_m", "rpy_deg"}
ROOT = "/World/Puzzle"


def _vector(value, label):
    if (not isinstance(value, list) or len(value) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError(f"{label} must be three finite numbers")
    return [float(v) for v in value]


def _rotation(rpy_deg):
    roll, pitch, yaw = (math.radians(v) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def realize(config):
    """Validate the section and resolve every piece's pose into the world, without USD imports or stage edits."""
    settings = config["puzzle"]
    if set(settings) != KEYS:
        raise ValueError(f"puzzle needs exactly {sorted(KEYS)}")
    numbers = [settings[key] for key in ("density_kg_m3", "static_friction", "dynamic_friction")]
    if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in numbers) or numbers[0] == 0:
        raise ValueError("puzzle density must be positive and its friction not negative")
    if numbers[2] > numbers[1]:
        raise ValueError("puzzle.dynamic_friction must not exceed static_friction")
    pieces = settings["pieces"]
    if not isinstance(pieces, list) or not 1 <= len(pieces) <= 64:
        raise ValueError("puzzle.pieces must be 1 to 64 pieces")
    frames, out = {}, []
    for piece in pieces:
        if not isinstance(piece, dict) or not PIECE_KEYS <= set(piece) <= PIECE_KEYS | {"parent"}:
            raise ValueError(f"A puzzle piece needs {sorted(PIECE_KEYS)} and may name a parent")
        name = piece["name"]
        if not isinstance(name, str) or not name.isidentifier() or name in frames:
            raise ValueError(f"puzzle piece name {name!r} must be an identifier used once")
        if piece["asset"] not in config["assets"]:
            raise ValueError(f"puzzle piece {name} names the asset role {piece['asset']!r}, which the task does not map")
        position = _vector(piece["position_m"], f"puzzle piece {name} position_m")
        rotation = _rotation(_vector(piece["rpy_deg"], f"puzzle piece {name} rpy_deg"))
        if "parent" in piece:
            if piece["parent"] not in frames:
                raise ValueError(f"puzzle piece {name} names the parent {piece['parent']!r}, which is not an earlier piece")
            at, turn = frames[piece["parent"]]
            position = [at[i] + sum(turn[i][k] * position[k] for k in range(3)) for i in range(3)]
            rotation = [[sum(turn[i][k] * rotation[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        frames[name] = (position, rotation)
        out.append({"name": name, "prim_path": f"{ROOT}/{name}", "asset_id": config["assets"][piece["asset"]],
                    "role": piece["asset"], "parent": piece.get("parent"),
                    "position_m": position, "rotation": rotation})
    return {"pieces": out}


def quantities(config):
    """How many pieces the task draws from each asset: what its BOM must list."""
    counts = {}
    for piece in realize(config)["pieces"]:
        counts[piece["asset_id"]] = counts.get(piece["asset_id"], 0) + 1
    return counts


def build(stage, assets, config):
    """Author every piece as a rigid body. ``assets`` maps the task's asset roles to files."""
    report = realize(config)
    settings = config["puzzle"]
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    if (UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdPhysics.GetStageKilogramsPerUnit(stage) != 1
            or UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z):
        raise ValueError("A puzzle requires a Z-up stage in metres and kilograms")
    if stage.GetPrimAtPath(ROOT):
        raise ValueError("The puzzle already exists; rebuild the workcell")
    UsdGeom.Xform.Define(stage, ROOT)
    material = UsdShade.Material.Define(stage, f"{ROOT}/ContactMaterial")
    contact = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    contact.CreateStaticFrictionAttr().Set(float(settings["static_friction"]))
    contact.CreateDynamicFrictionAttr().Set(float(settings["dynamic_friction"]))
    parts = []
    for piece in report["pieces"]:
        root = UsdGeom.Xform.Define(stage, piece["prim_path"])
        root.AddTranslateOp().Set(Gf.Vec3d(*piece["position_m"]))
        turn = Gf.Matrix3d(*[piece["rotation"][j][i] for i in range(3) for j in range(3)]).ExtractRotation().GetQuat()   # Gf is row-vector
        root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(turn)
        piece["orientation_wxyz"] = [turn.GetReal(), *turn.GetImaginary()]
        shape = UsdGeom.Xform.Define(stage, piece["prim_path"] + "/Asset").GetPrim()
        shape.GetReferences().AddReference(str(assets[piece["role"]]))
        volume = shape.GetAttribute("puzzle:volumeM3").Get()
        colliders = [prim for prim in Usd.PrimRange(shape, Usd.TraverseInstanceProxies()) if prim.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders or volume is None or not math.isfinite(volume) or volume <= 0:
            raise ValueError(f"Puzzle asset {piece['asset_id']} needs its own collision shapes and a positive puzzle:volumeM3")
        piece.update(volume_m3=volume, mass_kg=volume * settings["density_kg_m3"], colliders=len(colliders))
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(piece["mass_kg"])
        UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
        parts.append({"prim_path": piece["prim_path"], "asset_id": piece["asset_id"], "bom_id": piece["asset_id"]})
    report.update(parts=parts, piece_count=len(parts), density_kg_m3=settings["density_kg_m3"],
                  static_friction=settings["static_friction"], dynamic_friction=settings["dynamic_friction"],
                  limitations=["Piece geometry is upstream CAD; the printed parts' dimensions and slot clearances are not measured.",
                               "Mass is solid volume times density_kg_m3; a print with infill is lighter. Friction is not measured.",
                               "No fiducial tags are modelled."])
    return report
