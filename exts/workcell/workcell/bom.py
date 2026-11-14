"""Pure BOM, pack-manifest and stage-part audit helpers; no Kit imports."""

import csv
import hashlib
import json
import math
import re
from pathlib import Path, PurePosixPath

DEFAULT_BOM_PATH = Path(__file__).resolve().parents[1] / "workcell/bom.csv"
HARDWARE_STATUSES = frozenset({"confirmed", "unconfirmed", "conflict", "placeholder", "to_buy"})
PRICE_STATUSES = frozenset({"verified_web_list", "quote_required"})
NON_BOM_ASSETS = frozenset({"materials"})
COLUMNS = ("bom_id", "area", "item", "model", "quantity", "unit", "asset_id", "unit_price",
           "currency", "price_status", "price_source", "hardware_status", "source", "notes")
PHYSICAL_APIS = ("PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysicsRigidBodyAPI")
ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class BomError(ValueError):
    """A BOM, pack manifest or stage mapping is not self-consistent."""


def load_bom(path=None):
    path = Path(path or DEFAULT_BOM_PATH)
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != COLUMNS:
            raise BomError(f"BOM columns must be {', '.join(COLUMNS)}")
        rows = []
        for line, raw in enumerate(reader, start=2):
            row = {key: (raw.get(key) or "").strip() for key in COLUMNS}
            _validate_row(row, line)
            rows.append(row)
    _validate_rows(rows)
    return rows


def _validate_row(row, line):
    if not ID_PATTERN.match(row["bom_id"]):
        raise BomError(f"line {line}: invalid bom_id {row['bom_id']!r}")
    if not row["asset_id"] or not ID_PATTERN.match(row["asset_id"]):
        raise BomError(f"line {line}: invalid logical asset_id {row['asset_id']!r}")
    for key in ("area", "item", "model", "unit", "source"):
        if not row[key]:
            raise BomError(f"line {line}: {key} is required")
    if row["hardware_status"] not in HARDWARE_STATUSES:
        raise BomError(f"line {line}: invalid hardware_status {row['hardware_status']!r}")
    try:
        quantity = int(row["quantity"])
    except ValueError as exc:
        raise BomError(f"line {line}: quantity must be an integer") from exc
    if quantity <= 0:
        raise BomError(f"line {line}: quantity must be positive")
    row["quantity"] = quantity
    if row["price_status"] not in PRICE_STATUSES:
        raise BomError(f"line {line}: invalid price_status {row['price_status']!r}")
    if not row["currency"]:
        raise BomError(f"line {line}: currency is required")
    if row["price_status"] == "quote_required":
        if row["unit_price"]:
            raise BomError(f"line {line}: quote_required prices must stay blank, not invented")
        row["unit_price"] = None
        return
    try:
        price = float(row["unit_price"])
    except ValueError as exc:
        raise BomError(f"line {line}: verified price must be numeric") from exc
    if not math.isfinite(price) or price < 0:
        raise BomError(f"line {line}: verified price must be finite and non-negative")
    if not row["price_source"]:
        raise BomError(f"line {line}: verified price needs a source")
    row["unit_price"] = price


def _validate_rows(rows):
    bom_ids = [row["bom_id"] for row in rows]
    assets = [row["asset_id"] for row in rows]
    if len(bom_ids) != len(set(bom_ids)):
        raise BomError("bom_id values must be unique")
    if len(assets) != len(set(assets)):
        raise BomError("asset_id values must be unique")
    if not rows:
        raise BomError("BOM must contain at least one row")


def load_manifest(path):
    try:
        text = Path(path).read_text(encoding="utf-8")
        try:
            manifest = json.loads(text)
        except json.JSONDecodeError:
            import yaml
            try:
                manifest = yaml.safe_load(text)
            except yaml.YAMLError as exc:
                raise ValueError(str(exc)) from exc
    except (OSError, ValueError, ImportError) as exc:
        raise BomError(f"Cannot read pack manifest {path}: {exc}") from exc
    if not isinstance(manifest, dict):
        raise BomError("Pack manifest must be a mapping")
    return manifest


def verify_manifest(manifest, rows=None, root=None, expected_id=None, expected_version=None):
    if isinstance(manifest, (str, Path)):
        manifest = load_manifest(manifest)
    if not isinstance(manifest, dict):
        raise BomError("Pack manifest must be a mapping")
    pack_id = manifest.get("id")
    version = manifest.get("version")
    files = manifest.get("files")
    if not isinstance(pack_id, str) or not pack_id:
        raise BomError("Pack manifest needs a nonempty id")
    if not isinstance(version, str) or not version:
        raise BomError("Pack manifest needs a nonempty version")
    if expected_id is not None and pack_id != expected_id:
        raise BomError(f"Pack id is {pack_id!r}, expected {expected_id!r}")
    if expected_version is not None and version != expected_version:
        raise BomError(f"Pack version is {version!r}, expected {expected_version!r}")
    if not isinstance(files, list) or not files:
        raise BomError("Pack manifest needs a nonempty files list")
    root = Path(root) if root is not None else None
    seen_paths, asset_ids = set(), []
    for index, entry in enumerate(files):
        if not isinstance(entry, dict):
            raise BomError(f"Pack file {index} must be a mapping")
        path = entry.get("path")
        if not isinstance(path, str) or not path:
            raise BomError(f"Pack file {index} needs a path")
        pure = PurePosixPath(path)
        if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != path:
            raise BomError(f"Pack file path must be normalized and relative: {path!r}")
        if path in seen_paths:
            raise BomError(f"Duplicate pack path {path!r}")
        seen_paths.add(path)
        digest = entry.get("sha256")
        if not isinstance(digest, str) or not SHA256_PATTERN.match(digest):
            raise BomError(f"Pack file {path!r} needs a lowercase SHA-256")
        for key in ("source", "license"):
            if not isinstance(entry.get(key), str) or not entry[key]:
                raise BomError(f"Pack file {path!r} needs {key}")
        if type(entry.get("redistributable")) is not bool:
            raise BomError(f"Pack file {path!r} needs a boolean redistributable flag")
        if "asset_id" in entry:     # a file without one is a layer of the asset beside it (a bench collision layer)
            asset_id = entry["asset_id"]
            if not isinstance(asset_id, str) or not ID_PATTERN.match(asset_id):
                raise BomError(f"Pack file {path!r} has an invalid logical asset_id")
            if asset_id in asset_ids:
                raise BomError(f"Duplicate pack asset_id {asset_id!r}")
            asset_ids.append(asset_id)
        if root is not None:
            target = root / path
            if not target.is_file():
                raise BomError(f"Pack file is missing: {target}")
            actual = hashlib.sha256(target.read_bytes()).hexdigest()
            if actual != digest:
                raise BomError(f"Pack file hash changed: {path}")
    report = {"id": pack_id, "version": version, "files": len(files),
              "asset_ids": sorted(asset_ids), "hashes_verified": root is not None}
    if rows is not None:
        bom_assets = {row["asset_id"] for row in rows}
        pack_assets = set(asset_ids) - NON_BOM_ASSETS
        missing = sorted(pack_assets - bom_assets)
        extra = sorted(bom_assets - pack_assets)
        if missing:
            raise BomError(f"Pack assets have no BOM row: {missing}")
        if extra:
            raise BomError(f"BOM assets are absent from the pack: {extra}")
        report["bom_assets"] = len(bom_assets)
    return report


def verify_config_assets(config, rows):
    assets = config.get("assets") if isinstance(config, dict) else None
    if not isinstance(assets, dict) or not assets:
        raise BomError("Workcell config needs an assets mapping of roles to logical IDs")
    bom_assets = {row["asset_id"] for row in rows}
    used = set()
    for role, asset_id in assets.items():
        if role in NON_BOM_ASSETS:
            continue
        if not isinstance(asset_id, str) or not asset_id:
            raise BomError(f"Role {role!r} needs a logical asset_id")
        if asset_id not in bom_assets:
            raise BomError(f"Role {role!r} maps to {asset_id!r}, which has no BOM row")
        used.add(asset_id)
    missing = sorted(bom_assets - used)
    if missing:
        raise BomError(f"BOM assets are not mapped by the config: {missing}")
    return used


def _prim_path(prim):
    return str(prim.GetPath())


def _prim_attr(prim, name):
    attribute = prim.GetAttribute(name)
    if not attribute:
        return None
    return attribute.Get()


def _is_physical(prim):
    return (prim.GetTypeName() in {"Mesh", "Cube", "Sphere", "Cylinder", "Capsule", "Cone", "Plane"}
            or bool(set(prim.GetAppliedSchemas()).intersection(PHYSICAL_APIS)))


def _stage_prims(stage):
    if isinstance(stage, (list, tuple, set)):
        return list(stage)
    return list(stage.Traverse())


def _under(path, parents):
    return any(path == parent or path.startswith(parent + "/") for parent in parents)


def audit_parts(parts, rows, stage=None, require_all=True, strict=False, ignore=()):
    """``ignore``: subtrees that are not work-cell hardware (the room around it), left out of the physical scan."""
    errors = []
    rows_by_id = {row["bom_id"]: row for row in rows}
    seen_paths = set()
    mapped = {}
    for index, part in enumerate(parts):
        if not isinstance(part, dict):
            errors.append(f"part {index} must be a mapping")
            continue
        path = part.get("prim_path")
        asset_id = part.get("asset_id")
        bom_id = part.get("bom_id")
        if not all(isinstance(value, str) and value for value in (path, asset_id, bom_id)):
            errors.append(f"part {index} needs nonempty prim_path, asset_id and bom_id")
            continue
        if path in seen_paths:
            errors.append(f"Duplicate report prim_path {path}")
            continue
        seen_paths.add(path)
        mapped.setdefault(bom_id, []).append(path)
        row = rows_by_id.get(bom_id)
        if row is None:
            errors.append(f"{path} maps to unknown BOM id {bom_id!r}")
        elif row["asset_id"] != asset_id:
            errors.append(f"{path} asset_id {asset_id!r} does not match BOM row {bom_id!r}")
        if stage is None:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            errors.append(f"Part prim does not exist: {path}")
            continue
        if _prim_attr(prim, "workcell:asset_id") != asset_id:
            errors.append(f"{path} workcell:asset_id does not match the report")
        if _prim_attr(prim, "workcell:bom_id") != bom_id:
            errors.append(f"{path} workcell:bom_id does not match the report")
    missing = sorted(set(rows_by_id) - set(mapped))
    if require_all and missing:
        errors.append(f"BOM rows have no mapped physical part: {missing}")
    if require_all:
        for bom_id, row in rows_by_id.items():
            count = len(mapped.get(bom_id, ()))
            if count != row["quantity"]:
                errors.append(f"BOM quantity mismatch for {bom_id}: {count} mapped, {row['quantity']} expected")
    unmatched, empty, physical_paths = [], [], []
    if stage is not None:
        physical_paths = [_prim_path(prim) for prim in _stage_prims(stage)
                          if _is_physical(prim) and not _under(_prim_path(prim), tuple(ignore))]
        unmatched = [path for path in physical_paths if not _under(path, seen_paths)]
        empty = [path for path in sorted(seen_paths)
                 if not any(_under(item, (path,)) for item in physical_paths)]
        if unmatched:
            errors.append(f"Mesh or collider has no mapped BOM ancestor: {sorted(unmatched)}")
        if empty:
            errors.append(f"Part mappings contain no physical geometry: {empty}")
        if not physical_paths:
            errors.append("Stage has no mesh or collider prims to audit")
    report = {"ok": not errors, "errors": errors, "parts": len(parts),
              "physical_prims": len(physical_paths),
              "bom_ids": {key: len(value) for key, value in sorted(mapped.items())},
              "unmatched_prims": sorted(unmatched), "unmapped_bom_ids": missing,
              "empty_mappings": empty}
    if strict and errors:
        raise BomError("; ".join(errors))
    return report
