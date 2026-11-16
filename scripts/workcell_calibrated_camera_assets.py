#!/usr/bin/env python3
"""Build ur5e-workcell/v4 from v3, replacing only the wrist mount around the measured optical pose.

Run from mcp/: uv run --locked --isolated --extra scene-test python ../scripts/workcell_calibrated_camera_assets.py
The source is hash-verified and copied, never modified. Existing v4 is left alone. FreeCAD runs on CPU.
"""
from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path

import workcell_bracket_assets as bracket
from twinsmith import asset_packs, pack_manifest

PACK, SOURCE, TARGET = 'ur5e-workcell', 'v3', 'v4'


def main():
    root = pack_manifest.resolve_asset_root().path
    if (root / PACK / TARGET / 'pack.yaml').exists():
        print(f'{PACK}/{TARGET} exists, left alone')
        return
    source = pack_manifest.AssetPack(root, PACK, SOURCE)
    folder = root / PACK / SOURCE
    manifest = pack_manifest.read_manifest(folder, PACK, SOURCE)
    work = Path(tempfile.mkdtemp(prefix='workcell-calibrated-camera-'))
    try:
        report, summary = bracket.build_bracket(work, source.asset('d415'))
        files, records = {}, {}
        for record in manifest['files']:
            name = record['path']
            if name.startswith('wrist_mount/'):
                continue
            files[name], records[name] = folder / name, record
        for path in sorted((work / 'wrist_mount').iterdir()):
            name = f'wrist_mount/{path.name}'
            files[name] = path
            records[name] = dict(source='exts/workcell/cad/rg2_camera_bracket.py; 2026-11-16 hand-eye optical pose',
                                 license='Locally authored lab geometry; private pack', redistributable=False,
                                 fidelity='Photo-estimated clamp/truss/plate around measured optical pose; not manufacturer bracket CAD')
            if path.suffix == '.usdc':
                records[name]['asset_id'] = 'wrist_mount'
        extra = {key: value for key, value in manifest.items() if key not in ('id', 'version', 'files', 'defaults', 'notes')}
        extra.update(derived_from=f'{PACK}/{SOURCE}', qualification='v3 with a rebuilt wrist bracket at the 2026-11-16 measured camera pose; geometry remains approximate')
        result = asset_packs.write_version(root, PACK, TARGET, files, records,
                                          defaults=manifest.get('defaults', {}), notes=manifest.get('notes', []), extra=extra)
        print(json.dumps({'written': result, 'bracket': summary, 'pieces': report['pieces'],
                          'interference_mm3': report['interference_mm3'], 'clearance_mm': report['clearance_mm']}, indent=2))
    finally:
        shutil.rmtree(work)


if __name__ == '__main__':
    main()
