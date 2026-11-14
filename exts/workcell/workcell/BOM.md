# UR5e Jenga workcell bill of materials

This is the human-readable companion to [bom.csv](bom.csv): the same 13 logical rows,
asset IDs, quantities and units. Repeated instances share an asset ID. Geometry and
installed-hardware confidence are separate from procurement status; see the
[workcell README](README.md) for the external pack contract and measurement gaps.

Prices below are the list prices recorded in the CSV on 2026-11-12, not newly verified
quotes. Only D415 and KB3X3 have known item prices. Every other item requires a quote;
missing prices are not zero, and no complete workcell total is available.

| BOM / asset ID | Item / model | Quantity | Unit | Unit price (USD) | Hardware and fidelity notes |
| --- | --- | --- | --- | --- | --- |
| `bench` | Vention custom workbench with lab benchtop | 1 | each | Quote required | Preserved vendor mesh; top size and height need measurement |
| `robot` | Universal Robots UR5e collaborative arm | 1 | each | Quote required | Model confirmed; vendor CAD does not qualify runtime behavior |
| `base_plate` | UR5e mounting base plate | 1 | each | Quote required | Provisional 5 mm plate replaces the zero placeholder; installed thickness unconfirmed |
| `bota` | Bota Rokubi BFT-ROKS-SER-M8 force/torque sensor | 1 | each | Quote required | Product identity confirmed; geometry is a proxy, dimensions and mass unconfirmed |
| `quick_changer` | OnRobot Quick Changer, robot and tool sides | 1 | each | Quote required | CAD standoff 15.92 mm confirmed; installed size and mass unconfirmed |
| `rg2` | OnRobot RG2 standard parallel gripper | 1 | each | Quote required | Standard RG2, not RG2-FT; nominal maximum opening 110 mm confirmed |
| `finger_pad` | Black add-on RG2 finger pads | 2 | each | Quote required | Provisional 0.012 x 0.008 x 0.025 m proxy; installed geometry and friction unconfirmed |
| `d415` | Intel RealSense D415 depth camera | 2 | each | $272.00 each | Wrist unit present, drawn with the vendor's mesh; second overhead unit to buy and not built |
| `wrist_mount` | Printed camera bracket clamped to the RG2 body | 1 | each | Quote required | Real part exists; the FreeCAD model is a copy estimated from one photo; clamp and arm sizes and the camera pose need measurement |
| `overhead_mount` | Removable 80/20 1515 camera gantry | 1 | set | Quote required | Planned three-member gantry, not slab; two posts and cross-member, cantilever and real attachments unresolved; not purchased |
| `overhead_release` | Thorlabs KB3X3 kinematic quick-release base | 1 | each | $117.27 | To buy; camera-only release, not load-bearing gantry base; installed attachment unconfirmed |
| `overhead_shelf` | Printed overhead camera shelf | 1 | each | Quote required | Proposed proxy; dimensions, manufacture and attachment unconfirmed |
| `jenga_block` (task pack `jenga_tower`, not the base cell; `push_block` uses 1) | Wooden Jenga block, nominal 72 x 24 x 14 mm | 54 | each | Quote required | Nominal size confirmed; tolerances, chamfers, mass and friction unconfirmed |

Price provenance is retained in the CSV's `price_source` fields for the D415 and KB3X3.
A stock-profile rate is not a price for the assembled overhead gantry; its cut list and
fasteners remain unresolved. No purchase or financial action is performed here.

The CSV records the provisional 5 mm plate and distinguishes the confirmed Quick Changer
CAD standoff from its unconfirmed installed size. Neither record claims new measurements.
The private pack remains a candidate: no runtime, render or lab-fidelity qualification
is implied by this inventory. Seeded per-block variation is configured in
`lab_hardware.json` and exposed through the Jenga build action; its distributions remain
unconfirmed until measured.
