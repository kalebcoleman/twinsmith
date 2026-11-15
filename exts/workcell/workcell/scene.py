"""Pack-backed workcell assembly: the base cell, and optionally one task pack loaded on top of it."""

import copy
import math
from pathlib import Path

ROLES = ("robot", "bench", "base_plate", "bota", "quick_changer", "rg2",
         "d415", "wrist_mount", "overhead_mount", "overhead_release", "overhead_shelf", "materials")
# Every role's asset must exist in the pack, mounted or not, so one switch in the config is all a variant needs:
# the Bota (bota.mounted) and the overhead camera (overhead_camera.enabled) are built only when switched on.


# What a Newton build has not shown yet. PhysX is the work cell's qualified engine; see the README's Newton section.
NEWTON_LIMITATIONS = [
    "Newton: the arm's drives are the config's newton.drives (per engine), not the pack's PhysX gains; tracking differs.",
    "Newton: the cameras are not qualified on this engine; the wrist force/torque read passes the static gravity check.",
]


def _engine(config, mode):
    """The selected engine, checked against what the work cell can build on it. Never switches engines."""
    from .physics_compat import selected_engine

    engine = selected_engine()
    if engine == "newton":
        from . import newton_cell

        if "newton" not in config:
            raise RuntimeError("This configuration has no newton section (per-engine drives and solver settings); "
                               "the work cell cannot be built on Newton without it. Use a PhysX session or add the section.")
        try:
            newton_cell.validate(config)
        except ValueError as exc:
            raise RuntimeError(f"Invalid newton section, so no stage was replaced: {exc}") from exc
        if mode == "mirror_real":
            raise RuntimeError("mirror_real (gravity off on the robot) is not qualified on Newton; use physics mode or PhysX.")
        if config.get("asset_layout", {}).get("rg2", {}).get("articulation_root"):
            raise RuntimeError("A separate RG2 articulation does not load on Newton (its importer mis-lists the bodies); "
                               "use a pack whose RG2 joins the arm, such as ur5e-jenga-workcell/v2.")
    return engine


def _bom_rows(config, config_path=None):
    from . import bom, hardware_config

    selected = Path(config_path or hardware_config.DEFAULT_CONFIG).resolve()
    path = config.get("documentation", {}).get("bom_csv", "../workcell/bom.csv")
    rows = bom.load_bom(selected.parent / path)
    bom.verify_config_assets(config, rows)
    return rows


def _preflight(config_path, mode):
    from . import asset_pack, hardware_config

    if mode not in ("physics", "mirror_real"):
        raise ValueError("mode must be physics or mirror_real")
    config, raw = hardware_config.load_config(config_path)
    roles = config.get("assets", {})
    if any(not isinstance(roles.get(role), str) or not roles[role] for role in ROLES):
        raise ValueError("assets must map every workcell role to a logical asset ID")
    world = config["world"]
    xy = world["workstation_xy_m"]
    scale = world["bench_scale"]
    top = world["bench_asset_top_z_m"]
    if (len(xy) != 2 or not all(math.isfinite(float(value)) for value in xy)
            or not math.isfinite(float(scale)) or scale <= 0
            or not math.isfinite(float(top)) or world["tabletop_z_m"] != 0):
        raise ValueError("invalid bench offset, scale, asset tabletop height, or tabletop_z_m")
    if "bench_top" in config:
        check_bench_top(config["bench_top"])
    if "contact" in config:
        check_contact(config["contact"])
    if "inertials" in config:
        from .robot import check_inertials
        check_inertials(config["inertials"])
    _bom_rows(config, config_path)
    pack = asset_pack.resolve_pack(config)
    pack.verify_dependencies()
    assets = {}
    for role in ROLES:
        path = Path(pack.asset(roles[role]))
        if not path.is_absolute() or not path.is_file():
            raise ValueError(f"asset {role} must resolve to an existing absolute file")
        assets[role] = path
    if "inertials" in config:
        from .robot import check_inertial_links
        check_inertial_links(config["inertials"], [assets["robot"], assets["rg2"]])
    return config, raw, pack, assets


def contact_materials(stage, contact, root="/World"):
    """Same contact on both engines: bind a default material to every collider without one (Newton would give it
    mu 1.0, PhysX 0.5), make every physics material combine by the configured mode (MuJoCo's rule is max) and
    not bounce. Returns counts. Newton's single coefficient is set in newton_cell.prepare."""
    from pxr import Sdf, Usd, UsdPhysics, UsdShade

    default = UsdShade.Material.Define(stage, f"{root}/CellDefaultContactMaterial")
    api = UsdPhysics.MaterialAPI.Apply(default.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(contact["default_static_friction"]))
    api.CreateDynamicFrictionAttr().Set(float(contact["default_dynamic_friction"]))
    bound = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            if not material or not material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(default, materialPurpose="physics")
                bound += 1
    materials = [prim for prim in Usd.PrimRange(stage.GetPrimAtPath(root)) if prim.HasAPI(UsdPhysics.MaterialAPI)]
    for prim in materials:
        UsdPhysics.MaterialAPI(prim).CreateRestitutionAttr().Set(0.0)
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(contact["combine_mode"])
        prim.CreateAttribute("physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
    return {"default_bound_to": bound, "materials": len(materials), "combine_mode": contact["combine_mode"]}


CONTACT_KEYS = {"default_static_friction", "default_dynamic_friction", "combine_mode", "newton_friction"}


def check_contact(contact):
    if not isinstance(contact, dict) or set(contact) != CONTACT_KEYS:
        raise ValueError(f"contact needs exactly {sorted(CONTACT_KEYS)}")
    values = (contact["default_static_friction"], contact["default_dynamic_friction"])
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0 for v in values):
        raise ValueError("contact default friction must be finite numbers >= 0")
    if values[1] > values[0]:
        raise ValueError("contact.default_dynamic_friction must not exceed default_static_friction")
    if contact["combine_mode"] != "max":
        raise ValueError("contact.combine_mode must be max: MuJoCo (Newton) always combines by max, so any other "
                         "PhysX mode would make the two engines disagree")
    if contact["newton_friction"] not in ("static", "dynamic"):
        raise ValueError("contact.newton_friction must be static or dynamic")


BENCH_TOP_KEYS = {"panel_path", "color_rgb", "roughness", "static_friction", "dynamic_friction"}


def check_bench_top(top):
    """Refuse a malformed bench_top section before any stage is replaced."""
    if not isinstance(top, dict) or set(top) != BENCH_TOP_KEYS:
        raise ValueError(f"bench_top needs exactly {sorted(BENCH_TOP_KEYS)}")
    if not isinstance(top["panel_path"], str) or not top["panel_path"] or top["panel_path"].startswith("/"):
        raise ValueError("bench_top.panel_path must be a path relative to /World/Bench")
    from pxr import Sdf

    if not Sdf.Path.IsValidPathString(f"/World/Bench/{top['panel_path']}"):
        raise ValueError(f"bench_top.panel_path {top['panel_path']!r} is not a valid USD prim path")
    rgb = top["color_rgb"]
    if (not isinstance(rgb, (list, tuple)) or len(rgb) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and 0 <= v <= 1
                       for v in rgb)):
        raise ValueError("bench_top.color_rgb must be three numbers from 0 to 1")
    numbers = {key: top[key] for key in ("roughness", "static_friction", "dynamic_friction")}
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0
               for v in numbers.values()):
        raise ValueError("bench_top roughness and friction must be finite numbers >= 0")
    if numbers["roughness"] > 1:
        raise ValueError("bench_top.roughness must be from 0 to 1")
    if numbers["dynamic_friction"] > numbers["static_friction"]:
        raise ValueError("bench_top.dynamic_friction must not exceed static_friction")


def _configure_robot_gravity(stage, assembly_path, disabled):
    from pxr import PhysxSchema, Usd, UsdPhysics

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(assembly_path)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(disabled)


def _tag_parts(stage, parts):
    from pxr import Sdf

    seen = set()
    for part in parts:
        path, asset_id, bom_id = (part[key] for key in ("prim_path", "asset_id", "bom_id"))
        if not all(isinstance(value, str) and value for value in (path, asset_id, bom_id)) or path in seen:
            raise ValueError(f"invalid or duplicate part mapping: {part!r}")
        seen.add(path)
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise ValueError(f"part prim does not exist: {path}")
        prim.CreateAttribute("workcell:asset_id", Sdf.ValueTypeNames.String, custom=True).Set(asset_id)
        prim.CreateAttribute("workcell:bom_id", Sdf.ValueTypeNames.String, custom=True).Set(bom_id)


def bench_colliders(stage, path="/World/Bench", engine="physx") -> str:
    """Keep a bench's own collision layer (assets__mesh split: boxes or hulls, purpose guide); without one,
    make every bench mesh a triangle-mesh collider, which PhysX handles and Newton cannot. Says which."""
    from pxr import Usd, UsdGeom, UsdPhysics

    bench = list(Usd.PrimRange(stage.GetPrimAtPath(path)))
    if any(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in bench):
        return "own"
    meshes = [prim for prim in bench if prim.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise ValueError("Bench pack asset has no editable mesh colliders")
    if engine == "newton":
        raise ValueError("The bench has no collision layer of its own, and its triangle meshes overflow Newton's contact "
                         "buffer; use a pack with the split bench, such as ur5e-jenga-workcell/v2.")
    for prim in meshes:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")
    return "triangle meshes"


def scene_rows(config, rows):
    """The BOM rows the scene must hold. With the overhead camera off, its mount rows and its D415 unit
    (the second one) are left out, and so are the Bota while it is not mounted and the separate base plate while it
    is switched off; the BOM itself still lists them."""
    from . import stack
    from .cameras import OVERHEAD_ASSET_ROLES, overhead_enabled

    absent = set() if stack.bota_mounted(config) else {config["assets"].get("bota")}  # owned, not on the arm
    if (config.get("base_plate") or {}).get("present", True) is False:                  # the bench holds the real plates
        absent.add(config["assets"].get("base_plate"))
    if overhead_enabled(config):
        return [row for row in rows if row["asset_id"] not in absent]
    absent |= {config["assets"][role] for role in OVERHEAD_ASSET_ROLES}
    d415 = config["assets"]["d415"]
    return [{**row, "quantity": row["quantity"] - (row["asset_id"] == d415)}
            for row in rows if row["asset_id"] not in absent]


def _finish_scene(stage, config, assets, parts, rows, engine="physx"):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    from . import bom

    materials = UsdGeom.Scope.Define(stage, "/World/Materials")
    materials.GetPrim().GetReferences().AddReference(str(assets["materials"]))
    fingertip_friction(stage, config, parts)
    bench_colliders(stage, engine=engine)
    bom.verify_config_assets(config, rows)
    from .environment import ROOT
    # Labelled placeholders (the Bota's unmeasured adapter plates) are not BOM hardware: they are reported, not audited.
    proxies = tuple(str(prim.GetPath()) for prim in Usd.PrimRange(stage.GetPseudoRoot()) if prim.GetCustomDataByKey("workcell:proxy"))
    return bom.audit_parts(parts, scene_rows(config, rows), stage=stage, strict=True, ignore=(ROOT, *proxies))


def fingertip_friction(stage, config, parts):
    """Set ``fingertips`` friction on the physics material the RG2's two fingertip colliders are bound to.

    The grip is on the gripper's own fingertips (the pack's RG2 has a ``fingertip`` collision shape on each finger,
    bound to one physics material); there are no add-on pads. Returns the material's path and the colliders."""
    from pxr import Usd, UsdPhysics, UsdShade

    section = config.get("fingertips")
    if not section:
        return None
    static, dynamic = float(section["static_friction"]), float(section["dynamic_friction"])
    if not (math.isfinite(static) and math.isfinite(dynamic) and 0 <= dynamic <= static):
        raise ValueError("fingertips friction must be finite, with 0 <= dynamic_friction <= static_friction")
    colliders, materials = [], set()
    for part in parts:
        if part["asset_id"] != config["assets"]["rg2"]:
            continue
        for prim in Usd.PrimRange(stage.GetPrimAtPath(part["prim_path"])):
            if prim.GetName() != "fingertip" or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            if not material or not material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                raise ValueError(f"{prim.GetPath()} has no physics material; the RG2 asset must bind one to its fingertips")
            api = UsdPhysics.MaterialAPI(material.GetPrim())
            api.CreateStaticFrictionAttr().Set(static)
            api.CreateDynamicFrictionAttr().Set(dynamic)
            api.CreateRestitutionAttr().Set(0.0)
            colliders.append(str(prim.GetPath()))
            materials.add(str(material.GetPath()))
    if len(colliders) != 2 or len(materials) != 1:
        raise ValueError("The RG2 asset needs one fingertip collision shape on each finger, bound to one physics material")
    return {"material": materials.pop(), "colliders": colliders, "static_friction": static, "dynamic_friction": dynamic}


def bench_top(stage, top, bench="/World/Bench"):
    """Give the bench's top panel the configured look (its render mesh) and friction (its colliders), on both
    engines: a UsdPreviewSurface colour and a UsdPhysics material. Returns what was bound."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    panel = stage.GetPrimAtPath(f"{bench}/{top['panel_path']}")
    if not panel or not panel.IsValid():
        raise ValueError(f"bench_top.panel_path {top['panel_path']!r} is not a prim under {bench}")
    material = UsdShade.Material.Define(stage, "/World/BenchTopMaterial")
    shader = UsdShade.Shader.Define(stage, "/World/BenchTopMaterial/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*top["color_rgb"]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(top["roughness"]))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr().Set(float(top["static_friction"]))
    physics.CreateDynamicFrictionAttr().Set(float(top["dynamic_friction"]))
    physics.CreateRestitutionAttr().Set(0.0)
    meshes, colliders = [], []
    for prim in Usd.PrimRange(panel):
        binding = None
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(material, materialPurpose="physics")
            colliders.append(str(prim.GetPath()))
        if prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputePurpose() != UsdGeom.Tokens.guide:
            (binding or UsdShade.MaterialBindingAPI.Apply(prim)).Bind(
                material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            meshes.append(str(prim.GetPath()))
    if not colliders:
        raise ValueError(f"bench_top.panel_path {top['panel_path']!r} has no collider to carry the friction")
    return {"material": str(material.GetPath()), "color_rgb": list(top["color_rgb"]),
            "static_friction": float(top["static_friction"]), "dynamic_friction": float(top["dynamic_friction"]),
            "render_meshes": meshes, "colliders": colliders}


def _replace_tower(stage, config, assets, rows, parts, profile=None, seed=None, engine="physx"):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    from . import jenga

    root = stage.GetRootLayer()
    if any(layer.GetPrimAtPath("/World/Jenga") for layer in stage.GetLayerStack() if layer != root):
        raise RuntimeError("Jenga has competing layer opinions; rebuild the workcell first")
    temporary = Usd.Stage.CreateInMemory("workcell-tower.usda", stage.GetPathResolverContext())
    UsdGeom.SetStageUpAxis(temporary, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(temporary, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(temporary, 1.0)
    report = jenga.build(temporary, assets["jenga_block"], config, profile=profile, seed=seed)
    updated = [part for part in parts if not part["prim_path"].startswith("/World/Jenga/")] + report["parts"]
    snapshot = root.ExportToString()
    try:
        with Usd.EditContext(stage, root):
            stage.RemovePrim("/World/Jenga")
            if not Sdf.CopySpec(temporary.GetRootLayer(), "/World/Jenga", root, "/World/Jenga"):
                raise RuntimeError("Could not replace Jenga tower")
            _tag_parts(stage, updated)
            audit = _finish_scene(stage, config, assets, updated, rows, engine)
            if "contact" in config:
                contact_materials(stage, config["contact"])
            if stage.GetCompositionErrors():
                raise RuntimeError("Replacement tower has USD composition errors")
    except Exception:
        root.ImportFromString(snapshot)
        raise
    return report, updated, audit


def bodies_below(stage, start):
    """Rigid-body names reached from ``start`` through joints, start included: what the start body carries."""
    from pxr import Usd, UsdPhysics

    joints = [UsdPhysics.Joint(prim) for prim in Usd.PrimRange(stage.GetPseudoRoot()) if prim.IsA(UsdPhysics.Joint)]
    found, frontier = {str(start)}, [str(start)]
    while frontier:
        parent = frontier.pop()
        for joint in joints:
            if [str(p) for p in joint.GetBody0Rel().GetTargets()] == [parent]:
                for child in joint.GetBody1Rel().GetTargets():
                    if str(child) not in found:
                        found.add(str(child))
                        frontier.append(str(child))
    return [stage.GetPrimAtPath(path).GetName() for path in sorted(found)]


class Workcell:
    def __init__(self):
        self._report = {"state": "idle"}
        self._runtime = None

    def status(self):
        return copy.deepcopy(self._report)

    async def quick_start(self, config_path=None, mode="physics", discard_changes=False, environment="none", task=None):
        """Build the base cell; ``task`` (a name in ``tasks/`` or an absolute task file) loads its objects on top."""
        if type(discard_changes) is not bool:
            raise ValueError("discard_changes must be a boolean")
        from . import environment as rooms
        if environment not in rooms.NAMES:
            raise ValueError(f"environment must be one of {list(rooms.NAMES)}, not {environment!r}")
        config, raw, pack, assets = _preflight(config_path, mode)
        rows = _bom_rows(config, config_path)
        loaded = None
        if task is not None:                        # checked in full before the stage is replaced
            from . import tasks
            loaded = tasks.load(task)
            loaded["pack"].verify_dependencies()
            config, rows = tasks.merged(config, loaded), rows + loaded["rows"]
            assets = {**assets, "jenga_block": loaded["asset"]}
        engine = _engine(config, mode)
        room_url = rooms.resolve(environment)       # checked (reachable) before the stage is replaced
        import omni.usd
        import omni.timeline
        from sim_tools.stage_open import stage_has_unsaved_work
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        from . import robot, cameras, jenga

        if not callable(getattr(robot, "build_robot", None)) or not callable(getattr(cameras, "build_cameras", None)):
            raise RuntimeError("workcell robot and camera builders are not installed")
        context = omni.usd.get_context()
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_stopped():
            raise RuntimeError("Stop the timeline before replacing the stage")
        stage = context.get_stage()
        if stage and stage_has_unsaved_work(stage) and not discard_changes:
            raise RuntimeError("UNSAVED_CHANGES: pass discard_changes=True to replace the existing stage")
        ok, error = await context.new_stage_async()
        if not ok:
            raise RuntimeError(f"Could not create workcell stage: {error}")
        self._runtime = None
        self._report = {"state": "building", "mode": mode}
        try:
            stage = context.get_stage()
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)
            UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
            world = UsdGeom.Xform.Define(stage, "/World")
            stage.SetDefaultPrim(world.GetPrim())
            physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            physics.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
            physics.CreateGravityMagnitudeAttr().Set(9.81)
            light = UsdLux.DistantLight.Define(stage, "/World/Light")
            light.CreateIntensityAttr().Set(config["lighting"]["key_intensity"])
            dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
            dome.CreateIntensityAttr().Set(config["lighting"]["dome_intensity"])
            bench = UsdGeom.Xform.Define(stage, "/World/Bench")
            bench.GetPrim().GetReferences().AddReference(str(assets["bench"]))
            xy = config["world"]["workstation_xy_m"]
            scale = config["world"]["bench_scale"]
            bench.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], -config["world"]["bench_asset_top_z_m"]))
            bench.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
            robot_report = robot.build_robot(stage, pack, config)
            _configure_robot_gravity(stage, robot_report["assembly_path"], mode == "mirror_real")
            camera_report = cameras.build_cameras(stage, assets, config, robot_report)
            block_report = jenga.build(stage, assets["jenga_block"], config) if loaded else None
            parts = ([{"prim_path": "/World/Bench", "asset_id": config["assets"]["bench"], "bom_id": "bench"}]
                     + robot_report["parts"] + camera_report["parts"] + (block_report["parts"] if loaded else []))
            missing = {row["asset_id"] for row in scene_rows(config, rows)} - {part["asset_id"] for part in parts}
            if missing:
                raise ValueError(f"Physical assets missing from BOM part mappings: {sorted(missing)}")
            _tag_parts(stage, parts)
            audit = _finish_scene(stage, config, assets, parts, rows, engine)
            top_report = bench_top(stage, config["bench_top"]) if "bench_top" in config else None
            room_report = rooms.place(stage, environment, room_url) if room_url else None
            contact_report = contact_materials(stage, config["contact"]) if "contact" in config else None
            newton_report = None
            if engine == "newton":
                from . import newton_cell
                newton_report = newton_cell.prepare(stage, config, robot_report["assembly_path"], timeline)
            from sim_tools.compat import composition_errors
            errors = composition_errors(stage)
            if errors:
                raise RuntimeError(f"Workcell has USD composition errors: {errors}")
            self._report = {"state": "ready", "mode": mode, "backend": engine, "timeline": "stopped",
                            "gravity_m_s2": -9.81, "robot_gravity_disabled": mode == "mirror_real", "tabletop_z_m": 0.0,
                            "pack": copy.deepcopy(pack.manifest), "robot": robot_report,
                            "cameras": camera_report["cameras"], "parts": parts,
                            "task": {"name": loaded["name"], "file": loaded["path"],
                                     "pack": {key: loaded["pack"].manifest[key] for key in ("id", "version")}}
                            if loaded else None,
                            **({"jenga": block_report} if loaded else {}),
                            "bom_audit": audit, **({"newton": newton_report} if newton_report else {}),
                            **({"bench_top": top_report} if top_report else {}),
                            **({"environment": room_report} if room_report else {}),
                            **({"contact": contact_report} if contact_report else {}),
                            "limitations": [line for line in robot_report["limitations"]
                                            if not (engine == "newton" and "drives are preserved" in line)]
                            + camera_report["limitations"] + (block_report["limitations"] if loaded else [])
                            + (NEWTON_LIMITATIONS if engine == "newton" else [])}
            self._runtime = (stage, config, assets, rows)
        except Exception as exc:
            self._report = {"state": "failed", "mode": mode, "error": str(exc), "stage_replaced": True}
            raise
        return self.status()

    def build_jenga(self, profile=None, seed=None):
        if self._runtime is None or self._report["state"] != "ready":
            raise RuntimeError("Build the workcell before rebuilding its tower")
        if "jenga" not in self._report:
            raise RuntimeError("The cell has no task objects; build it with a Jenga task first, "
                               "for example quick_start with task='jenga_tower'")
        import omni.usd
        import omni.timeline
        stage, config, assets, rows = self._runtime
        if omni.usd.get_context().get_stage() != stage:
            raise RuntimeError("Workcell stage changed; quick_start is required")
        if not omni.timeline.get_timeline_interface().is_stopped():
            raise RuntimeError("Stop the timeline before rebuilding the tower")
        engine = _engine(config, self._report.get("mode", "physics"))
        report, parts, audit = _replace_tower(stage, config, assets, rows, self._report["parts"], profile, seed, engine)
        self._report.update(jenga=report, parts=parts, bom_audit=audit)      # the tower is replaced: report it first
        if engine == "newton":
            from . import newton_cell
            self._report["newton"] = newton_cell.prepare(stage, config, self._report["robot"]["assembly_path"],
                                                         omni.timeline.get_timeline_interface())
        return copy.deepcopy(report)

    def read_wrench(self):
        """The wrist force/torque reading now, plus the engine's mass of every body the measured joint carries, so a
        caller can compare the static force with that weight. Without a Bota (the default) this is the UR5e's tool
        flange sensor; with ``bota.mounted`` it is the Bota's reading, and the flange reading comes with it."""
        if self._runtime is None or self._report.get("state") != "ready":
            raise RuntimeError("Build the workcell before reading its sensor")
        import omni.usd
        from sim_tools import physics
        from .physics_compat import selected_engine
        from . import sensors

        stage, config, _, _ = self._runtime
        if omni.usd.get_context().get_stage() != stage:
            raise RuntimeError("Workcell stage changed; quick_start is required")
        robot = self._report["robot"]
        engine = selected_engine()
        view = physics.view_for(robot["articulation_root"], articulation=True)
        gravity = 0.0 if self._report.get("robot_gravity_disabled") else abs(self._report.get("gravity_m_s2", -9.81))

        def read(source):
            reading = sensors.read_wrench(stage, view, robot, config, source=source,
                                          physics_time_s=physics.manager().get_simulation_time(), backend=engine)
            if reading.get("status") != "measured":
                return reading
            below = bodies_below(stage, reading["load_body_path"])
            names = view.shared_metatype.link_indices
            masses = sensors._numpy(view.get_masses()).reshape(-1)
            links = {name: float(masses[names[name]]) for name in below if name in names}
            total = sum(links.values())
            reading["load_below_sensor"] = {"links_kg": links, "missing": sorted(set(below) - set(links)),
                                            "mass_kg": total, "gravity_m_s2": gravity, "weight_n": total * gravity,
                                            "note": "Engine masses of what the measured joint carries. At rest the "
                                                    "force magnitude should equal weight_n. Raw reaction: the real "
                                                    "controller subtracts the payload set on the pendant."}
            return reading

        flange = read(sensors.FLANGE_SOURCE)
        if not robot.get("bota"):
            return flange
        reading = read(sensors.BOTA_SOURCE)
        reading["flange"] = flange
        return reading

    def shutdown(self):
        self._runtime = None
        self._report = {"state": "shutdown", "stage_preserved": True}
        return self.status()
