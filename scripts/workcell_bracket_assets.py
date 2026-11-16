#!/usr/bin/env python3
"""Build ``ur5e-workcell/v3``: v2 with the real wrist bracket and with the tool stack rebuilt from the vendors' official CAD.

* ``rg2``: OnRobot's official STEP (a local copy named by ``TWINSMITH_RG2_STEP``, checked by SHA-256), converted by
  ``scripts/workcell_rg2_assets.py``; see that module. The add-on finger pad proxy of v2 is not carried over: the real
  fingertips, with their rubber pads, are part of the RG2.
* ``quick_changer`` and ``bota``: OnRobot's Quick Changer robot side and Bota Systems' Rokubi, from their official STEP
  files (``TWINSMITH_QC_STEP``, ``TWINSMITH_BOTA_STEP``, checked by SHA-256), converted by
  ``scripts/workcell_stack_assets.py``; see that module. The Rokubi is not on the real arm: the work cell builds it
  only when ``bota.mounted`` is true in its config.

* ``wrist_mount``: ``exts/workcell/cad/rg2_camera_bracket_v3.py``, the frozen model of v3, run by ``freecadcmd`` at
  v3's own photo-estimated camera pose (``V3_POSE`` below), not at the pose in ``exts/workcell/config/lab_hardware.json``:
  that file now holds the 2026-11-16 measured pose, which belongs to v4 (``scripts/workcell_calibrated_camera_assets.py``
  replaces this bracket there). The printed bracket clamps the RG2
  body, so the USD is in the pack's RG2 body frame and the builder places it there. The pack keeps the USD (render
  mesh plus a collision approximation of five convex hulls, which the builder attaches to the RG2 body), and, in the
  CAD frame, the STEP of both pieces as mounted and the STEP and STL of each printable piece.
* ``bench`` and ``rail_carriage``: v2's bench (Vention's CAD of the lab's bench) split by
  ``scripts/workcell_rail_assets.py`` into the static bench and the carriage of the linear axis the robot stands on,
  with the robot plate; the cable carrier is left out. The vendor meshes are v2's, unchanged.
* Every other file is copied from v2 unchanged, including the D415 body. The vendors' STEP files are not copied.

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
import workcell_rail_assets as rail_assets  # noqa: E402
import workcell_rg2_assets as rg2_assets  # noqa: E402
import workcell_stack_assets as stack_assets  # noqa: E402
from twinsmith import asset_packs, pack_manifest  # noqa: E402

PACK, SOURCE, TARGET = "ur5e-workcell", "v2", "v3"
CAD = "exts/workcell/cad/rg2_camera_bracket.py"                  # the current model: v4, around the measured optical pose
CAD_V3 = "exts/workcell/cad/rg2_camera_bracket_v3.py"            # the frozen model v3 was built from
# v3's camera pose, as lab_hardware.json held it when v3 was built (2026-11-14, estimated from one photo). Pinned here so
# a rebuilt v3 keeps v3's bracket whatever the configuration says now; the current model cannot build this pose.
V3_POSE = {"offset_in_rg2_m": [0.127, 0.035, 0.102], "rotation_in_rg2_deg": [39.0, 0.0, -90.0]}
PIECES = ("clamp_front", "camera_arm")
PRINTED = dict(diffuseColor=(0.10, 0.11, 0.14), metallic=0.0, roughness=0.6)
FIDELITY = ("Parametric copy of the real printed bracket, estimated from one photo; the RG2 and D415 dimensions are from "
            "the vendors' datasheets, everything else must be measured (exts/workcell/cad/README.md)")       # v3's record


def to_body(points):
    """CAD frame (+X finger-opening axis, +Y out of the front face) to the pack's RG2 body frame, a quarter turn about Z.

    The pack's ``onrobot_rg2_base_link`` has +Y along the finger-opening axis and +X out of the back face.
    """
    points = np.asarray(points, dtype=float)
    return np.stack([-points[..., 1], points[..., 0], points[..., 2]], axis=-1)


def camera_pose(config):
    """The configured camera origin in the CAD frame (millimetres) and its tilt; only poses the CAD model covers pass.

    ``rotation_in_rg2_deg`` is extrinsic XYZ in the RG2 body frame: tilt 0 to 45 degrees,
    with pitch and yaw departures below 5 degrees from [tilt, 0, -90].
    """
    section = config["wrist_camera"]
    offset, rotation = section["offset_in_rg2_m"]["value"], section["rotation_in_rg2_deg"]["value"]
    tilt, pitch, yaw = (float(value) for value in rotation)
    if not (abs(pitch) < 5.0 and abs(yaw + 90.0) < 5.0 and 0.0 <= tilt <= 45.0):
        raise SystemExit("the bracket model covers a camera with optical +x along the RG2 body's -Y, tilted 0 to 45 degrees "
                         "toward the tool axis, with less than 5 degrees of pitch/yaw mounting deviation")
    x, y, z = (float(value) * 1000.0 for value in offset)
    return [y, -x, z], tilt                                              # the inverse of to_body


def d415_extent(path: Path):
    """Back face along the optical axis and body centre along optical x, in millimetres, from the pack's D415 mesh."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))                                     # kept alive while its prims are read
    points = [np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float) for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    points = np.vstack(points) * 1000.0
    return float(points[:, 2].min()), float((points[:, 0].min() + points[:, 0].max()) / 2)


def run_cad(out: Path, camera_mm, tilt_deg, back_mm, body_cx_mm, rotation=None, cad=CAD):
    # FreeCAD brings its own Python: this interpreter's variables (uv's virtual environment) must not reach it.
    clean = {key: value for key, value in os.environ.items() if not key.startswith(("PYTHON", "VIRTUAL_ENV", "UV_"))}
    env = dict(clean, BRACKET_OUT=str(out), CAM_X_MM=str(camera_mm[0]), CAM_Y_MM=str(camera_mm[1]), CAM_Z_MM=str(camera_mm[2]),
               TILT_DEG=str(tilt_deg), BACK_MM=str(back_mm), BODY_CX_MM=str(body_cx_mm))
    if rotation is not None:
        env.update(PITCH_DEG=str(rotation[1]), YAW_DEG=str(rotation[2]))
    done = subprocess.run(["freecadcmd", str(REPO / cad)], env=env, capture_output=True, text=True, timeout=600)
    report = out / "bracket.json"
    if done.returncode or not report.is_file():
        raise SystemExit(f"freecadcmd did not write the bracket: {done.stdout[-800:]} {done.stderr[-800:]}")
    return json.loads(report.read_text())


COLLISION_SCOPE = "Collision"       # child of the asset's default prim; the work cell builder references it onto the RG2 body


BOX_FACES = [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3]
PRISM_FACES = [0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5]


def hull_faces(name, points):
    """Outward triangles of one piece's convex hull; a piece that has no volume is refused by name."""
    from scipy.spatial import ConvexHull, QhullError

    try:
        hull = ConvexHull(points)
    except (QhullError, ValueError) as error:
        raise SystemExit(f"collision piece {name} has no convex hull ({len(points)} points; flat, too few or not finite): "
                         f"{str(error).splitlines()[0] if str(error) else type(error).__name__}") from error
    faces = hull.simplices.copy()
    for face, equation in zip(faces, hull.equations):
        a, b, c = points[face]
        if np.dot(np.cross(b - a, c - a), equation[:3]) < 0:
            face[1], face[2] = face[2], face[1]
    return [3] * len(faces), faces.ravel().tolist()


def add_collision(path: Path, report, v3_layout=False):
    """The bracket's collision approximation under ``/Asset/Collision``: a few convex hulls, guide purpose, metres.

    Each piece is a small point cloud with ``physics:approximation = "convexHull"`` set explicitly, the form the pack's
    RG2 links already use. Nothing here is a rigid body: the builder references this scope under the RG2 body.
    The faces are the triangles of each piece's convex hull. ``v3_layout`` writes v3's form instead: the frozen model
    reports each piece as a box (8 corners) or a triangular prism (6) in a fixed order, drawn with quads.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    stage = Usd.Stage.Open(str(path))
    scope = UsdGeom.Scope.Define(stage, f"/Asset/{COLLISION_SCOPE}")
    UsdGeom.Imageable(scope.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    names = []
    for name, points in report["collision_pieces_mm"].items():
        points = to_body(points) / 1000.0
        if v3_layout:
            if len(points) not in (6, 8):
                raise SystemExit(f"collision piece {name} must be a box (8 corners) or a triangular prism (6)")
            counts, indices = ([4] * 6, BOX_FACES) if len(points) == 8 else ([3, 3, 4, 4, 4], PRISM_FACES)
        else:
            counts, indices = hull_faces(name, points)
        mesh = UsdGeom.Mesh.Define(stage, f"/Asset/{COLLISION_SCOPE}/{name}")
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points.tolist()]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        names.append(name)
    stage.GetRootLayer().Save()
    return names


def build_bracket(work: Path, d415_path: Path, v3=False):
    """The bracket for the configured camera pose with the current model, or (``v3``) v3's own bracket: the frozen
    model at ``V3_POSE``, with v3's collision layout, whatever the configuration holds."""
    if v3:
        config = {"wrist_camera": {key: {"value": value} for key, value in V3_POSE.items()}}
        if V3_POSE["rotation_in_rg2_deg"][1:] != [0.0, -90.0]:
            raise SystemExit("the frozen v3 model has no pitch or yaw: rotation_in_rg2_deg = [tilt, 0, -90]")
    else:
        config = json.loads((REPO / "exts/workcell/config/lab_hardware.json").read_text())
    camera_mm, tilt_deg = camera_pose(config)
    back_mm, body_cx_mm = d415_extent(d415_path)
    folder = work / "wrist_mount"
    folder.mkdir()
    report = run_cad(folder, camera_mm, tilt_deg, round(back_mm, 2), round(body_cx_mm, 2),
                     None if v3 else config["wrist_camera"]["rotation_in_rg2_deg"]["value"], CAD_V3 if v3 else CAD)
    triangles = to_body(np.concatenate([camera_assets.read_stl(folder / f"{name}.stl") for name in PIECES])) * 0.001
    points, faces = camera_assets.weld(triangles)
    usd = folder / "wrist_mount.usdc"
    camera_assets.write_usd(usd, points, faces, {"Printed": (np.ones(len(faces), bool), PRINTED)},
                            f"FreeCAD model ({CAD_V3}) moved to the pack's RG2 body frame; estimated from one photo, not measured" if v3 else
                            f"FreeCAD model ({CAD}) moved to the pack's RG2 body frame; photo-estimated bracket geometry around the configured optical pose")
    colliders = add_collision(usd, report, v3_layout=v3)
    return report, {"triangles": int(len(faces)), "colliders": colliders, "camera_in_cad_frame_mm": camera_mm, "tilt_deg": tilt_deg,
                    "d415_back_mm": round(back_mm, 2), "d415_body_centre_x_mm": round(body_cx_mm, 2)}


def main() -> int:
    root = pack_manifest.resolve_asset_root().path
    if (root / PACK / TARGET / "pack.yaml").is_file():
        print(json.dumps({f"{PACK}/{TARGET}": "exists, left alone"}))
        return 0
    step = rg2_assets.step_path()                                         # refuses before anything is built
    stack_steps = {role: stack_assets.step_path(role) for role in stack_assets.PARTS}
    source = pack_manifest.AssetPack(root, PACK, SOURCE)                  # verifies every hash first
    folder = root / PACK / SOURCE
    manifest = pack_manifest.read_manifest(folder, PACK, SOURCE)
    work = Path(tempfile.mkdtemp(prefix="workcell-bracket-"))
    try:
        report, summary = build_bracket(work, source.asset("d415"), v3=True)
        (work / "rg2_cad").mkdir()
        (work / "rg2").mkdir()
        rg2_assets.run_cad(step, work / "rg2_cad")
        rg2 = rg2_assets.build_rg2(source.asset("rg2"), work / "rg2_cad", work / "rg2" / "rg2.usdc")
        (work / "rg2" / "rg2.json").write_text(json.dumps(rg2, indent=1) + "\n")
        files, records = {}, {}
        for record in manifest["files"]:
            if not record["path"].startswith(("wrist_mount/", "rg2/", "finger_pad/", "quick_changer/", "bota/", "bench/")):
                files[record["path"]], records[record["path"]] = folder / record["path"], record
        files["rg2/rg2.usdc"], records["rg2/rg2.usdc"] = work / "rg2" / "rg2.usdc", rg2_assets.manifest_records()
        files["rg2/rg2.json"] = work / "rg2" / "rg2.json"
        records["rg2/rg2.json"] = {key: value for key, value in rg2_assets.manifest_records().items() if key != "asset_id"}
        config = json.loads((REPO / "exts/workcell/config/lab_hardware.json").read_text())
        bench = rail_assets.build_bench(folder / "bench", work / "bench")
        carriage = rail_assets.build_carriage(work / "bench", work / "rail_carriage" / "rail_carriage.usda", config)
        old = {record["path"]: record for record in manifest["files"] if record["path"].startswith("bench/")}
        for name in ("bench_render.usdc", "bench_collision.usdc"):       # v2's files, unchanged
            files[f"bench/{name}"], records[f"bench/{name}"] = work / "bench" / name, old[f"bench/{name}"]
        inherited = {key: value for key, value in old["bench/bench.usda"].items() if key not in ("path", "sha256", "asset_id", "changes")}
        split = ("Pack v2's bench.usda is bench_full.usda here. bench.usda is the static bench: the carriage of the linear axis and "
                 "the robot plate are deactivated (they are the rail_carriage asset), the cable carrier and its tray are left out, "
                 "and the parts have looks from photos (scripts/workcell_rail_assets.py).")
        files["bench/bench_full.usda"], records["bench/bench_full.usda"] = work / "bench" / "bench_full.usda", dict(inherited, changes=old["bench/bench.usda"]["changes"])
        files["bench/bench_cell.usda"], records["bench/bench_cell.usda"] = work / "bench" / "bench_cell.usda", dict(inherited, changes=split)
        files["bench/bench.usda"], records["bench/bench.usda"] = work / "bench" / "bench.usda", dict(inherited, asset_id="bench", changes=split)
        files["rail_carriage/rail_carriage.usda"] = work / "rail_carriage" / "rail_carriage.usda"
        records["rail_carriage/rail_carriage.usda"] = dict(inherited, asset_id="rail_carriage", changes=(
            "The carriage of the bench's linear axis (MO_LM_027_0001) and the robot plate on it (ST_RB_001_0002), referenced from "
            "bench/bench_full.usda, in metres, origin at the robot's base; looks from photos (scripts/workcell_rail_assets.py)."))
        stack = {}
        for role, path in stack_steps.items():
            (work / f"{role}_cad").mkdir()
            (work / role).mkdir()
            stack_assets.run_cad(role, path, work / f"{role}_cad")
            name = stack_assets.PARTS[role]["file"]
            stack[role] = stack_assets.build_part(role, work / f"{role}_cad", work / role / name)
            (work / role / f"{role}.json").write_text(json.dumps(stack[role], indent=1) + "\n")
            files[f"{role}/{name}"], records[f"{role}/{name}"] = work / role / name, stack_assets.manifest_record(role)
            files[f"{role}/{role}.json"] = work / role / f"{role}.json"
            records[f"{role}/{role}.json"] = {key: value for key, value in stack_assets.manifest_record(role).items() if key != "asset_id"}
        common = dict(source=f"{CAD_V3} (FreeCAD), at v3's photo-estimated camera pose: offset_in_rg2_m "
                             f"{V3_POSE['offset_in_rg2_m']}, rotation_in_rg2_deg {V3_POSE['rotation_in_rg2_deg']}",
                      license="Locally authored lab geometry; private pack", redistributable=False, fidelity=FIDELITY)
        made = work / "wrist_mount"
        files["wrist_mount/wrist_mount.usdc"] = made / "wrist_mount.usdc"
        records["wrist_mount/wrist_mount.usdc"] = dict(
            asset_id="wrist_mount", **common,
            changes="Both printed pieces as one render mesh in the RG2 body frame, plus a collision approximation of five "
                    "convex hulls under /Asset/Collision, which the work cell builder attaches to the RG2 body.")
        for name in ("wrist_bracket.step", "bracket.json", *(f"{piece}.{kind}" for piece in PIECES for kind in ("step", "stl"))):
            files[f"wrist_mount/{name}"] = made / name
            records[f"wrist_mount/{name}"] = dict(common)
        extra = dict(mimic={"driven": rg2["driven_joint"], "followers": rg2["followers"], "limits_deg": rg2["limits_deg"],
                            "note": "exact parallelogram from the CAD's pin axes: ratios are exactly -1 and 1"},
                     qualification=f"{PACK}/{SOURCE} with the RG2, the Quick Changer's robot side and the Rokubi from the vendors' "
                                   "official CAD, a copy of the real wrist bracket on the RG2, and the bench split so the robot's "
                                   "rail carriage can move",
                     derived_from=f"{PACK}/{SOURCE}")
        written = asset_packs.write_version(root, PACK, TARGET, files, records, defaults={}, notes=[], extra=extra)
        print(json.dumps({"written": str(written)[:160], "rg2": {key: value for key, value in rg2.items() if key != "links"},
                          "stack": {role: {key: value for key, value in part.items() if key not in ("parts", "left_out")}
                                    for role, part in stack.items()},
                          "bench": bench, "rail_carriage": carriage,
                          "bracket": summary, "pieces": report["pieces"],
                          "interference_mm3": report["interference_mm3"], "clearance_mm": report["clearance_mm"]}, indent=1))
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
