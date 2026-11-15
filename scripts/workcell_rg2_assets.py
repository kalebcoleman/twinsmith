#!/usr/bin/env python3
"""The work cell's RG2, rebuilt from OnRobot's official STEP (used by ``scripts/workcell_bracket_assets.py`` for pack v3).

* Source: "RG2 Tier 2 - 101631", https://onrobot.com/storage/303_rg2_tool.step, linked as "Step file" from
  https://onrobot.com/en/products/rg2-finger-gripper. OnRobot's redistribution terms are not known: the file is read
  from a local path (``TWINSMITH_RG2_STEP``), checked against the SHA-256 pinned here, and its geometry goes into the
  private pack only. Nothing of it is kept in this repository.
* ``exts/workcell/cad/rg2_from_step.py`` (FreeCAD) turns the STEP into one mesh per link and material, in the joint-zero
  pose; this module puts those meshes on the articulation the cell already uses (pack v2's ``rg2.usdc``: same link and
  joint names, same body frame, drives, schemas), with:
  - joint frames from the CAD's pin axes, made an exact parallelogram, so the followers' mimic ratios are exactly -1 and 1;
  - ONE driven joint, ``finger_joint``, as the real gripper has one motor: ``right_outer_knuckle_joint`` follows it;
  - limits from the CAD and the datasheet: closed where the fingertips' rubber pads meet, open at the datasheet's
    149 mm overall width;
  - one convex collider per link, and on each finger a second one for the fingertip (block and rubber pad) bound to
    the ``FingertipTape`` physics material, whose friction the work cell sets from ``fingertips`` in the config.
  Masses and inertias are not authored here: the work cell sets them from the config's ``inertials``.
* The STEP also holds the Quick Changer's robot side. It is left out of the RG2 (the pack's ``quick_changer`` is built
  from OnRobot's own robot-side file, ``scripts/workcell_stack_assets.py``); it tells where the tool flange sits: 2.5 mm
  below the body frame's origin, with the robot side's face toward the tool 13.6 mm further.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

import workcell_camera_assets as camera_assets  # noqa: E402  (STL reading and welding)

STEP_URL = "https://onrobot.com/storage/303_rg2_tool.step"
PRODUCT_PAGE = "https://onrobot.com/en/products/rg2-finger-gripper"
STEP_SHA256 = "b3c41f38efafd9eab44c53f61d618e437f2fd0248cbc774c7384e838290ad184"
STEP_ENV = "TWINSMITH_RG2_STEP"
CAD = "exts/workcell/cad/rg2_from_step.py"
DATASHEET_OPEN_WIDTH_MM = 149.0     # OnRobot RG2 datasheet v1.8, section 1.3: overall width fully open
LINKS = {"onrobot_rg2_base_link": "base", "left_outer_knuckle": "left_outer", "right_outer_knuckle": "right_outer",
         "left_inner_knuckle": "left_inner", "right_inner_knuckle": "right_inner",
         "left_inner_finger": "left_finger", "right_inner_finger": "right_finger"}
PIVOT_OF_LINK = {"left_outer_knuckle": ("left", "A"), "right_outer_knuckle": ("right", "A"), "left_inner_knuckle": ("left", "B"),
                 "right_inner_knuckle": ("right", "B"), "left_inner_finger": ("left", "C"), "right_inner_finger": ("right", "C")}
PIVOT_OF_JOINT = {"finger_joint": ("left", "A"), "right_outer_knuckle_joint": ("right", "A"),
                  "left_inner_finger_joint": ("left", "C"), "right_inner_finger_joint": ("right", "C"),
                  "left_finger_knuckle_joint": ("left", "D"), "RevoluteJoint": ("right", "D")}
DRIVER = "finger_joint"
FOLLOWERS = {"right_outer_knuckle_joint": 1.0, "left_inner_finger_joint": -1.0, "right_inner_finger_joint": -1.0,
             "left_finger_knuckle_joint": 1.0, "RevoluteJoint": 1.0}
# Linear RGB read off the photos of the real gripper (2026-11-14); looks only. A fully metallic material mirrors the
# bright sky dome and reads as white plastic, so the metals are kept mostly diffuse.
MATERIALS = {"aluminium": dict(diffuseColor=(0.30, 0.30, 0.31), metallic=0.35, roughness=0.55),      # bead-blasted, grey anodised
             "black": dict(diffuseColor=(0.02, 0.02, 0.022), metallic=0.0, roughness=0.5),             # moulded covers
             "rubber": dict(diffuseColor=(0.012, 0.012, 0.012), metallic=0.0, roughness=0.9),          # fingertip pads
             "steel": dict(diffuseColor=(0.36, 0.36, 0.37), metallic=0.5, roughness=0.4)}              # pins and screws
FINGERTIP_MATERIAL = "FingertipTape"


def step_path(environ=None) -> Path:
    """The local copy of OnRobot's STEP, checked against the pinned SHA-256; nothing is downloaded here."""
    value = (os.environ if environ is None else environ).get(STEP_ENV)
    if not value:
        raise SystemExit(f"set {STEP_ENV} to a local copy of {STEP_URL} (linked from {PRODUCT_PAGE})")
    path = Path(value).expanduser()
    if not path.is_file():
        raise SystemExit(f"{STEP_ENV} does not name a file")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    if digest.hexdigest() != STEP_SHA256:
        raise SystemExit("the RG2 STEP does not match its pinned SHA-256; nothing was built")
    return path


def run_cad(step: Path, out: Path) -> dict:
    # FreeCAD brings its own Python: this interpreter's variables (uv's virtual environment) must not reach it.
    clean = {key: value for key, value in os.environ.items() if not key.startswith(("PYTHON", "VIRTUAL_ENV", "UV_"))}
    done = subprocess.run(["freecadcmd", str(REPO / CAD)], env=dict(clean, STEP_SRC=str(step), RG2_OUT=str(out)),
                          capture_output=True, text=True, errors="replace", timeout=3000)
    report = out / "rg2.json"
    if done.returncode or not report.is_file():
        raise SystemExit(f"freecadcmd did not convert the RG2: {done.stdout[-600:]} {done.stderr[-600:]}")
    return json.loads(report.read_text())


def symmetric_pivots(report: dict) -> dict:
    """The four pin axes of each side, in metres in the gripper frame (x, z), averaged over both sides and made an
    exact parallelogram (the CAD's own pins agree within 0.1 mm)."""
    right, left = report["pivots_mm"]["right"], report["pivots_mm"]["left"]
    ax, az = (right["A"][0] - left["A"][0]) / 2, (right["A"][1] + left["A"][1]) / 2
    bx, bz = (right["B"][0] - left["B"][0]) / 2, (right["B"][1] + left["B"][1]) / 2
    arm = (right["arm_AC"] + right["arm_BD"] + left["arm_AC"] + left["arm_BD"]) / 4
    zero = math.radians(report["zero_pose_deg"])
    cx, cz = ax + arm * math.sin(zero), az + arm * math.cos(zero)
    one = {"A": (ax, az), "B": (bx, bz), "C": (cx, cz), "D": (cx + bx - ax, cz + bz - az)}
    return {"arm_mm": arm, "zero_rad": zero,
            "right": {name: (x / 1000.0, z / 1000.0) for name, (x, z) in one.items()},
            "left": {name: (-x / 1000.0, z / 1000.0) for name, (x, z) in one.items()}}


def stroke(report: dict, pivots: dict, meshes: dict) -> dict:
    """Joint limits and what they mean: closed where the rubber pads meet, open at the datasheet's overall width."""
    arm, zero = pivots["arm_mm"], pivots["zero_rad"]
    ax, az = (value * 1000.0 for value in pivots["right"]["A"])
    cx = pivots["right"]["C"][0] * 1000.0
    pads = report["fingertips"]
    face = (pads["right"]["pad"]["face_x_mm"] - pads["left"]["pad"]["face_x_mm"]) / 2            # at joint zero
    inset = cx - face                                                                            # pad face inboard of pin C

    def gap(q):
        return 2.0 * (ax + arm * math.sin(zero + q) - inset)

    closed = math.asin((inset - ax) / arm) - zero
    # Overall width: the outer arm turned about A, the finger carried along without turning.
    outer = np.vstack([meshes[key].reshape(-1, 3) for key in meshes if key[0] == "right_outer"])
    finger = np.vstack([meshes[key].reshape(-1, 3) for key in meshes if key[0] == "right_finger"])

    def width(q):
        c, s = math.cos(q), math.sin(q)
        turned = ax + (outer[:, 0] - ax) * c + (outer[:, 2] - az) * s
        carried = finger[:, 0] + arm * (math.sin(zero + q) - math.sin(zero))
        return 2.0 * max(turned.max(), carried.max())

    low, high = 0.0, math.radians(60.0)
    for _ in range(60):
        middle = (low + high) / 2
        low, high = (middle, high) if width(middle) < DATASHEET_OPEN_WIDTH_MM else (low, middle)
    opened = (low + high) / 2
    tip = lambda q: az + arm * math.cos(zero + q)                                                # noqa: E731  pin C along the axis
    return {"lower_deg": math.degrees(closed), "upper_deg": math.degrees(opened), "pad_gap_closed_mm": gap(closed),
            "pad_gap_open_mm": gap(opened), "pad_gap_at_zero_mm": gap(0.0), "pad_gap_at_old_upper_limit_20_deg_mm": gap(math.radians(20.0)),
            "bare_fingertip_gap_open_mm": gap(opened) + 2 * pads["right"]["pad"]["thickness_mm"],
            "finger_mount_face_gap_open_mm": gap(opened) + 2 * (pads["right"]["pad"]["thickness_mm"] + pads["right"]["block_size_mm"][0]),
            "overall_width_open_mm": width(opened), "overall_width_at_zero_mm": width(0.0),
            "fingertip_drop_over_stroke_mm": tip(closed) - tip(opened), "fingertip_drop_zero_to_closed_mm": tip(closed) - tip(0.0),
            "pad_face_inboard_of_pin_c_mm": inset}


def convex_hull(points, grid=0.001):
    """Vertices and outward triangles of the convex hull of ``points`` (metres), merged to ``grid`` first."""
    from scipy.spatial import ConvexHull

    points = np.unique(np.round(points / grid) * grid, axis=0)
    hull = ConvexHull(points)
    used = np.unique(hull.simplices)
    index = {old: new for new, old in enumerate(used)}
    vertices, centre = points[used], points[used].mean(0)
    faces = []
    for simplex in hull.simplices:
        a, b, c = (points[i] for i in simplex)
        outward = np.dot(np.cross(b - a, c - a), a - centre) > 0
        faces.append([index[i] for i in (simplex if outward else simplex[::-1])])
    return vertices, np.array(faces)


def build_rg2(source: Path, cad: Path, dest: Path) -> dict:
    """Write ``dest`` from pack v2's RG2 articulation (``source``) and the converted CAD in ``cad``."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    report = json.loads((cad / "rg2.json").read_text())
    pivots = symmetric_pivots(report)
    meshes = {}
    for path in sorted(cad.glob("*__*.stl")):
        link, material = path.stem.split("__")
        meshes[(link, material)] = camera_assets.read_stl(path)                                  # millimetres, gripper frame
    tips = {side: camera_assets.read_stl(cad / f"fingertip_{side}.stl") for side in ("left", "right")}
    limits = stroke(report, pivots, meshes)

    shutil.copyfile(source, dest)
    stage = Usd.Stage.Open(str(dest))
    root = stage.GetDefaultPrim()
    editor = Usd.NamespaceEditor(stage)                                                          # the old root was named after another robot
    if not editor.RenamePrim(root, "onrobot_rg2") or not editor.ApplyEdits():
        raise SystemExit("could not rename the RG2 asset's root prim")
    root = stage.GetPrimAtPath("/onrobot_rg2")
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("fidelity", f"OnRobot's official STEP ({STEP_URL}, sha256 {STEP_SHA256}); decimated; materials are look only")

    for name, (side, pivot) in PIVOT_OF_LINK.items():
        x, z = pivots[side][pivot]
        root.GetChild(name).GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 0.0, z))
    cache = UsdGeom.XformCache()
    to_link = {name: np.array(cache.GetLocalToWorldTransform(root.GetChild(name)).GetInverse()) for name in LINKS}

    def local(name, points_mm):
        points = np.asarray(points_mm, dtype=float).reshape(-1, 3) / 1000.0
        return (np.c_[points, np.ones(len(points))] @ to_link[name])[:, :3]

    looks = root.GetChild("Looks")
    for child in list(looks.GetChildren()):
        stage.RemovePrim(child.GetPath())
    stage.RemovePrim(root.GetPath().AppendChild("PhysicsMaterial"))
    materials = {}
    for name, surface in MATERIALS.items():
        material = UsdShade.Material.Define(stage, looks.GetPath().AppendChild(name.capitalize()))
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        for key, value in surface.items():
            kind = Sdf.ValueTypeNames.Color3f if key == "diffuseColor" else Sdf.ValueTypeNames.Float
            shader.CreateInput(key, kind).Set(Gf.Vec3f(*value) if key == "diffuseColor" else float(value))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[name] = material
    tape = UsdShade.Material.Define(stage, root.GetPath().AppendChild(FINGERTIP_MATERIAL))
    friction = UsdPhysics.MaterialAPI.Apply(tape.GetPrim())                                       # the work cell sets these from its config
    friction.CreateStaticFrictionAttr().Set(0.6)
    friction.CreateDynamicFrictionAttr().Set(0.5)
    friction.CreateRestitutionAttr().Set(0.0)

    def collider(path, points, box=False):
        if box:                                            # a flat-faced block: its own bounding box, in the link's axes
            low, high = points.min(0), points.max(0)
            points = np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
        vertices, faces = convex_hull(points, grid=1e-6 if box else 0.001)
        stage.RemovePrim(path)
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(len(faces), 3))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.astype(np.int32).reshape(-1)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr([Gf.Vec3f(*vertices.min(0)), Gf.Vec3f(*vertices.max(0))])
        mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        return mesh.GetPrim(), len(vertices)

    summary = {"links": {}, "triangles": 0}
    for name, key in LINKS.items():
        prim = root.GetChild(name)
        UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
        parts = {material: local(name, triangles) for (link, material), triangles in meshes.items() if link == key}
        points, faces, subsets, offset = [], [], {}, 0
        for material, triangles in parts.items():
            welded, indices = camera_assets.weld(triangles.reshape(-1, 3, 3))
            subsets[material] = (len(faces), len(faces) + len(indices))
            points.append(welded)
            faces.extend((indices + offset).tolist())
            offset += len(welded)
        points, faces = np.vstack(points), np.array(faces)
        path = prim.GetPath().AppendChild("visuals")
        stage.RemovePrim(path)
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(len(faces), 3))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.astype(np.int32).reshape(-1)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
        for material, (first, last) in subsets.items():
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, material, UsdGeom.Tokens.face, Vt.IntArray(list(range(first, last))),
                                                     UsdShade.Tokens.materialBind)
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[material])
        everything = np.vstack(list(parts.values()))
        shapes = {}
        if key.endswith("_finger"):
            tip = local(name, tips[key.split("_")[0]])
            low, high = tip.min(0) - 0.0002, tip.max(0) + 0.0002
            carrier = everything[~np.all((everything >= low) & (everything <= high), axis=1)]
            _, shapes["collisions"] = collider(prim.GetPath().AppendChild("collisions"), carrier)
            fingertip, shapes["fingertip"] = collider(prim.GetPath().AppendChild("fingertip"), tip, box=True)
            summary.setdefault("fingertip_box_in_link_m", {})[name] = [np.round(tip.min(0), 5).tolist(), np.round(tip.max(0), 5).tolist()]
            UsdShade.MaterialBindingAPI.Apply(fingertip).Bind(tape, materialPurpose="physics")
        else:
            _, shapes["collisions"] = collider(prim.GetPath().AppendChild("collisions"), everything)
        summary["links"][name] = {"triangles": int(len(faces)), "collider_hull_vertices": shapes}
        summary["triangles"] += int(len(faces))

    link_world = {name: np.linalg.inv(matrix) for name, matrix in to_link.items()}
    driver = None
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        side, pivot = PIVOT_OF_JOINT[prim.GetName()]
        x, z = pivots[side][pivot]
        for attr, rel in ((joint.GetLocalPos0Attr(), joint.GetBody0Rel()), (joint.GetLocalPos1Attr(), joint.GetBody1Rel())):
            body = rel.GetTargets()[0].name
            attr.Set(Gf.Vec3f(*(np.array([x, 0.0, z, 1.0]) @ to_link[body])[:3].tolist()))
        if prim.GetName() == DRIVER:
            driver = prim
            joint.GetLowerLimitAttr().Set(round(limits["lower_deg"], 3))
            joint.GetUpperLimitAttr().Set(round(limits["upper_deg"], 3))
    for prim in Usd.PrimRange(root):
        ratio = FOLLOWERS.get(prim.GetName())
        if ratio is None or not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        ends = sorted((ratio * limits["lower_deg"], ratio * limits["upper_deg"]))            # PhysX wants finite limits on a follower
        joint.GetLowerLimitAttr().Set(round(ends[0] - 1.0, 3))
        joint.GetUpperLimitAttr().Set(round(ends[1] + 1.0, 3))
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):                                       # one motor: this side follows
            prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
            for attribute in list(prim.GetAttributes()):
                if attribute.GetName().startswith("drive:angular:"):
                    prim.RemoveProperty(attribute.GetName())
        if "NewtonMimicAPI" not in prim.GetAppliedSchemas():
            prim.AddAppliedSchema("NewtonMimicAPI")
        prim.CreateRelationship("newton:mimicJoint").SetTargets([driver.GetPath()])
        prim.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(0.0)
        prim.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(ratio)
    stage.GetRootLayer().Save()

    pad = report["fingertips"]["right"]["pad"]
    old = {"A": (17.18, 125.80), "B": (7.70, 142.30), "C": (56.77, 163.98), "D": (47.16, 180.56)}       # pack v2, millimetres
    summary.update(
        root_prim="/onrobot_rg2", driven_joint=DRIVER, followers=FOLLOWERS, limits_deg=[round(limits["lower_deg"], 3), round(limits["upper_deg"], 3)],
        stroke={key: round(value, 3) for key, value in limits.items()},
        pivots_mm={name: [round(v * 1000.0, 3) for v in pivots["right"][name]] for name in "ABCD"},
        pivots_moved_from_v2_mm={name: round(math.hypot(pivots["right"][name][0] * 1000.0 - old[name][0], pivots["right"][name][1] * 1000.0 - old[name][1]), 3)
                                 for name in "ABCD"},
        arm_length_mm=round(pivots["arm_mm"], 3), fingertip_block_mm=report["fingertips"]["right"]["block_size_mm"],
        fingertip_pad_mm=[pad["thickness_mm"], pad["width_along_y_mm"], pad["length_along_z_mm"]],
        triangles_before_decimation=report["triangles_before_decimation"], parts_left_out=len(report["left_out"]),
        flange_plane_z_mm=report["robot_side"]["flange_plane_z_mm"], robot_side_tool_end_z_mm=report["robot_side"]["tool_end_z_mm"],
        robot_side_height_mm=report["robot_side"]["height_mm"])
    return summary


def manifest_records() -> dict:
    return dict(asset_id="rg2", source=f"OnRobot's official STEP 'RG2 Tier 2 - 101631' ({STEP_URL}, linked from {PRODUCT_PAGE}; sha256 {STEP_SHA256})",
                license="OnRobot's CAD; redistribution terms not known. Private pack only: do not publish or redistribute",
                redistributable=False,
                fidelity="Official vendor CAD, tessellated and decimated; hidden internal parts left out; materials are look only; "
                         "masses come from the work cell's config, not from the CAD",
                changes=f"Converted by {CAD} and scripts/workcell_rg2_assets.py onto pack v2's articulation: joint frames from the CAD's "
                        "pin axes, one driven joint (finger_joint) with five followers through NewtonMimicAPI at exactly -1 or 1, "
                        "limits from the CAD and the datasheet, one convex collider per link plus a fingertip collider per finger.")
