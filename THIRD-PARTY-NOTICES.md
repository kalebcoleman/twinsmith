# Third-party notices

This repository ships **no assets**. The work cell is designed to use the third-party assets
below, which each user keeps in packs in their local asset root (see `docs/ASSET-PACKS.md`; each
pack's `pack.yaml` records the source and licence of every file). Older revisions of the repository
carried them; these notices apply to anyone who obtains or redistributes them.
NVIDIA Isaac Sim assets and the Vention workstation are **not** redistributed;
they come from their official source or are supplied by the operator.

## Husarion OnRobot RG2 description — Apache-2.0

Files:

- `<asset-folder>/<pack>/adapter_ref/`
- `<asset-folder>/<pack>/gripper/husarion_rg2/`
- `<asset-folder>/<pack>/gripper/QuickChanger.usd`
- `<asset-folder>/<pack>/gripper/configuration/QuickChanger_*.usd`

Source: <https://github.com/husarion/ur_onrobot_rg2_ros> (`onrobot_rg2_description`).

Copyright Husarion and contributors. Licensed under the Apache License, Version
2.0. A copy of the licence is at
[`docs/provenance/husarion-ur-onrobot-rg2-LICENSE`](docs/provenance/husarion-ur-onrobot-rg2-LICENSE).

Changes: the URDF/xacro sources were converted to USD with the Isaac Sim URDF
importer; visuals and colliders were reparented under the links; the
Quick-Changer was converted and reoriented for the flange mount. The files in
`adapter_ref/` are unmodified copies.

## FMB — Functional Manipulation Benchmark — CC BY 4.0

Files:

- `<asset-folder>/<pack>/objects/fmb1/`
- `<asset-folder>/<pack>/objects/fmb2/`
- `<asset-folder>/<pack>/objects/fmb3/`
- `<asset-folder>/<pack>/fmb_assembly1.json`
- `<asset-folder>/<pack>/fmb_assembly2.json`
- `<asset-folder>/<pack>/fmb_assembly3.json`
- `<asset-folder>/puzzles/v1/` (the `fmb1` and `fmb3` pieces, flattened by `scripts/workcell_puzzle_pack.py`)

Source: *FMB: A Functional Manipulation Benchmark for Generalizable Robotic
Learning*, Jianlan Luo, Charles Xu, Xinyang Geng, Gilbert Feng, Kuan Fang,
Liam Tan, Stefan Schaal and Sergey Levine (UC Berkeley),
<https://functional-manipulation-benchmark.github.io/>.

The FMB project releases its CAD and data under the Creative Commons
Attribution 4.0 International License:
<https://creativecommons.org/licenses/by/4.0/>.

Changes: the object CAD was converted to USD; materials, colliders and physics
overlays were authored; some objects were renamed, rescaled or repositioned for
the UR5e workcell; ArUco poses were added. The `doc` metadata that named the
original workstation path was removed.

## Generated ArUco markers

`<asset-folder>/<pack>/objects/aruco/` contains marker images generated from the
standard OpenCV 4x4 and 5x5 ArUco dictionaries, plus pose JSON produced by the
twin's tooling. OpenCV is Apache-2.0; the generated dictionary patterns are not
copyrightable.

## Assets that are not redistributed

- **NVIDIA Isaac Sim** — the UR5e robot, RealSense D455 model and NVIDIA
  material libraries remain under the NVIDIA Isaac Sim Additional Software and
  Materials License. The simulator's own asset root supplies them by name; the
  repository does not download or ship them.
- **Vention workstation** — the Vention MachineBuilder export is not
  redistributed. An operator may supply their own copy in the asset cache; the
  twin otherwise builds a simple placeholder bench.
- **Poliigon** — the `WoodVeneerOak_7760` textures were removed. Their licence
  does not permit embedding them in a redistributed scene.
