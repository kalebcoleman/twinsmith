#!/usr/bin/env python3
"""Split ``ur5e-jenga-workcell/v2`` into the base cell pack and the first task pack.

* ``ur5e-workcell/v1``: every file of the source except the Jenga block. The robot, gripper stack, cameras, bench
  and mounting hardware; no task objects.
* ``jenga/v1``: the 72 x 24 x 14 mm block alone. The tower and the single push block are tasks that use it
  (``exts/workcell/tasks/``).

The source is only read. Each pack is written by the pack library's own writer (staged, verified, then moved into
place), and a pack that already exists is left alone, so the script can be run again. Run from ``mcp/``:

    uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_split_packs.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "mcp"))

from twinsmith import asset_packs, pack_manifest  # noqa: E402

SOURCE = ("ur5e-jenga-workcell", "v2")
BLOCK = "jenga_block"


def main() -> int:
    root = pack_manifest.resolve_asset_root().path
    source = pack_manifest.AssetPack(root, *SOURCE)          # verifies every hash first
    folder = root / SOURCE[0] / SOURCE[1]
    manifest = pack_manifest.read_manifest(folder, *SOURCE)
    block = {r["path"]: r for r in manifest["files"] if r.get("asset_id") == BLOCK}
    cell = {r["path"]: r for r in manifest["files"] if r["path"] not in block}
    origin = f"{SOURCE[0]}/{SOURCE[1]}"
    written = {}
    for pack_id, records, extra in (
            ("ur5e-workcell", cell, dict(qualification=f"{origin} without its task object (the Jenga block)",
                                         derived_from=origin, mimic=manifest.get("mimic"))),
            ("jenga", block, dict(qualification="task pack: the Jenga block alone", derived_from=origin))):
        if (root / pack_id / "v1" / "pack.yaml").is_file():
            written[pack_id] = "exists, left alone"
            continue
        files = {rel: folder / rel for rel in records}
        result = asset_packs.write_version(root, pack_id, "v1", files, records, defaults={}, notes=[], extra=extra)
        written[pack_id] = {"files": len(files), "result": str(result)[:200]}
    print(json.dumps(written, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
