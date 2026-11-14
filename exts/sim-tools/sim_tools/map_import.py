"""Import a ROS/Nav2 map (a YAML with ``image``, ``resolution`` and ``origin``, plus its PGM or PNG) as a floor plan of walls.

Each run of occupied cells becomes a box with a static collider, merged into as few boxes as the image allows, under one wrapper
prim. It is the inverse of ``sim__occupancy_map``: export a scene's map, import it back, and the walls are where the cells were.
"""

from __future__ import annotations

import math
from pathlib import Path

from twinsmith.project_assets import refuse

MAX_BOXES = 4000
MAX_PIXELS = 25_000_000


def read_pgm(data: bytes):
    """``(width, height, maxval, pixels)`` of a P2 or P5 PGM, pixels row-major top row first."""
    import re

    match = re.match(rb"(P[25])\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s", data)
    if not match:
        raise ValueError("not a PGM")
    kind, width, height, maxval = match.group(1), int(match.group(2)), int(match.group(3)), int(match.group(4))
    if width * height > MAX_PIXELS or not 0 < maxval < 65536:
        refuse("LIMIT", "The map image is too large or not 8 or 16 bit.")
    body = data[match.end():]
    if kind == b"P5":
        if maxval < 256:
            pixels = list(body[:width * height])
        else:
            pixels = [(body[2 * i] << 8) | body[2 * i + 1] for i in range(min(width * height, len(body) // 2))]
    else:
        pixels = [int(v) for v in body.split()[:width * height]]
    if len(pixels) != width * height:
        raise ValueError("the PGM is truncated")
    return width, height, maxval, pixels


def read_png(data: bytes):
    try:
        from PIL import Image
    except ImportError:
        refuse("IMPORTER_UNAVAILABLE", "This Kit cannot read a PNG map (no PIL). Use a PGM image.")
    import io

    try:
        image = Image.open(io.BytesIO(data))  # reads the header only: the size is checked before any pixel is decoded
        width, height = image.size
        if width * height > MAX_PIXELS:
            refuse("LIMIT", "The map image is too large.")
        if image.mode in ("I;16", "I;16B", "I;16L", "I"):  # 16-bit grey: converting to 8 bits would saturate it
            return width, height, 65535, list(image.getdata())
        return width, height, 255, list(image.convert("L").getdata())
    except Image.DecompressionBombError as exc:
        refuse("LIMIT", f"The map image is too large: {str(exc)[:200]}")


def rectangles(occupied, width, height):
    """Merge occupied cells into rectangles ``(x, y, w, h)`` in pixels: runs along a row, then equal runs of consecutive rows."""
    open_runs, done = {}, []
    for y in range(height):
        row = occupied[y * width:(y + 1) * width]
        runs, x = [], 0
        while x < width:
            if row[x]:
                start = x
                while x < width and row[x]:
                    x += 1
                runs.append((start, x - start))
            else:
                x += 1
        current = {}
        for run in runs:
            if run in open_runs:
                top, rows = open_runs.pop(run)
                current[run] = (top, rows + 1)
            else:
                current[run] = (y, 1)
        for run, (top, rows) in open_runs.items():
            done.append((run[0], top, run[1], rows))
        open_runs = current
    for run, (top, rows) in open_runs.items():
        done.append((run[0], top, run[1], rows))
    return done


def run(ext, args: dict) -> dict:
    from twinsmith import urdf_refs, xacro_lite
    from twinsmith.urdf_refs import normal_path
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    s = ext.scene
    args = dict(args)
    snapshot_id, digest, prim_path = args.pop("snapshot_id"), args.pop("manifest_sha256"), args.pop("prim_path")
    height_m = args.pop("wall_height", None)
    height_m = 1.0 if height_m is None else height_m
    if isinstance(height_m, bool) or not isinstance(height_m, (int, float)) or not 0 < height_m <= 100:
        refuse("INVALID_VALUE", "wall_height must be a number of metres above 0 and at most 100.")
    pose_keys = {k: args[k] for k in ("position", "orientation", "frame") if k in args}
    stage = s.get_stage()
    s.new_path(stage, prim_path)
    up, units = str(UsdGeom.GetStageUpAxis(stage)), UsdGeom.GetStageMetersPerUnit(stage)
    if up != "Z" or not math.isclose(units, 1.0, rel_tol=1e-9):
        from twinsmith.project_assets import units_mismatch_message

        # The walls are built Z up in metres: on any other stage they would lie on their side or be the wrong size.
        refuse("UNITS_MISMATCH", units_mismatch_message("Z", 1.0, up, units), retry_with=dict(up_axis="Z", meters_per_unit=1.0))
    filename, manifest = s.broker.verify(snapshot_id, digest, kind="map")
    entry = manifest["entry"]
    project_dir = filename.parents[len(Path(entry).parts) - 1]
    project = urdf_refs.project_from_manifest(project_dir, manifest["files"], manifest.get("root_name", ""))
    try:
        meta = xacro_lite.load_yaml_text(filename.read_text("utf-8"))
    except (UnicodeDecodeError, xacro_lite.XacroError) as exc:
        refuse("MAP_YAML", f"{entry} is not a readable map file: {str(exc)[:200]}")
    if not isinstance(meta, dict) or "image" not in meta or "resolution" not in meta:
        refuse("MAP_YAML", f"{entry} is not a ROS map: it needs image and resolution.")
    resolution = meta["resolution"]
    origin = meta.get("origin") or [0.0, 0.0, 0.0]
    if isinstance(resolution, bool) or not isinstance(resolution, (int, float)) or not 0 < resolution < 100 \
            or not isinstance(origin, list) or len(origin) < 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in origin):
        refuse("MAP_YAML", "resolution must be a positive number of metres per pixel and origin a list of numbers.")
    image_name = str(meta["image"])
    image = normal_path(urdf_refs._parent(entry), image_name) if "\\" not in image_name and not image_name.startswith("/") and "://" not in image_name else None
    if image is None or image not in project.files or project._unsafe(image):
        refuse("MAP_IMAGE", f"The map's image {image_name[:80]!r} is not inside the approved folder.")
    data = project.read(image)
    try:
        width, height, maxval, pixels = read_png(data) if data[:4] == b"\x89PNG" else read_pgm(data)
    except ValueError as exc:
        refuse("MAP_IMAGE", f"{image} is not a readable PGM or PNG image: {exc}")
    negate = str(meta.get("negate", 0)).strip().lower() in ("1", "true", "yes")
    try:
        occupied_threshold = float(meta.get("occupied_thresh", 0.65))
    except (TypeError, ValueError):
        refuse("MAP_YAML", "occupied_thresh must be a number.")
    warnings = []
    if len(origin) >= 3 and abs(float(origin[2])) > 1e-6:
        warnings.append(f"The map's origin has a yaw of {float(origin[2]):.3f} rad, which was not applied: use orientation on this import to turn the map.")
    occupied = []
    for value in pixels:
        probability = (value / maxval) if negate else (maxval - value) / maxval
        occupied.append(probability > occupied_threshold)
    boxes = rectangles(occupied, width, height)
    if len(boxes) > MAX_BOXES:
        refuse("LIMIT", f"The map needs {len(boxes)} wall boxes (the limit is {MAX_BOXES}). Use a coarser or cleaner image.")
    ox, oy = float(origin[0]), float(origin[1])
    position, orientation = pose_keys.get("position"), pose_keys.get("orientation")
    for name, value, size in (("position", position, 3), ("orientation", orientation, 4)):
        if value is not None and (not isinstance(value, list) or len(value) != size or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in value)):
            refuse("INVALID_VALUE", f"{name} must be {size} finite numbers.")
    # The session layer like every other tool edit, and owned, so sim__prim__delete and sim__stage__clear_owned work on it.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        root = UsdGeom.Xform.Define(stage, prim_path)
        if position is not None:
            UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))
        if orientation is not None:
            w, x, y, z = [float(v) for v in orientation]
            UsdGeom.Xformable(root).AddOrientOp().Set(Gf.Quatf(w, x, y, z).GetNormalized())
        for index, (x, top, w, h) in enumerate(boxes):
            cube = UsdGeom.Cube.Define(stage, f"{prim_path}/wall_{index}")
            cube.CreateSizeAttr(1.0)
            centre_x = ox + (x + w / 2.0) * resolution
            centre_y = oy + (height - top - h / 2.0) * resolution  # image row 0 is the top (largest y)
            xform = UsdGeom.Xformable(cube)
            xform.AddTranslateOp().Set(Gf.Vec3d(centre_x, centre_y, height_m / 2.0))
            xform.AddScaleOp().Set(Gf.Vec3f(w * resolution, h * resolution, height_m))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    s.owned.add(prim_path)
    s.bulk_owned = {*getattr(s, "bulk_owned", ()), prim_path}
    return dict(
        prim_path=prim_path, image=image, size_px=[width, height], resolution=resolution, origin=[ox, oy],
        extent_m=[round(width * resolution, 4), round(height * resolution, 4)], occupied_cells=sum(occupied), walls=len(boxes),
        wall_height=height_m, warnings=warnings + ([] if boxes else ["No occupied cells were found: check negate and occupied_thresh."]),
        note="Walls are static boxes at the map's own coordinates; add a ground plane separately.",
        validation=dict(ok=bool(boxes), checks=[dict(name="has_walls", ok=bool(boxes), detail=f"{len(boxes)} boxes")]),
    )
