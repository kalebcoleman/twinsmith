# Vendoring and Upstream Provenance

This document records the provenance, upstream source repositories, fork locations, pinned commit SHAs, license status, and patch histories for all vendored components.

---

## 1. Provenance Record

| Component | Origin | License status | State here |
|---|---|---|---|
| **`exts/workcell`** | Fork of `https://github.com/inbarajaldrin/isaac-sim-mcp` at `27f6100` (`https://github.com/kalebcoleman/isaac-sim-mcp`) | MIT (`docs/provenance/isaac-sim-mcp-LICENSE`) | Developed further here; the extension is now one module per area under `exts/workcell/workcell/`. |
| **`exts/twinsmith.bridge`**, the bridge client and server core in **`mcp/`** | First built on `https://github.com/inbarajaldrin/omni-kit-mcp` at `9d0366b`, used with its author's permission | Upstream has no LICENSE file | **Rewritten in this repository.** The bridge, its protocol, client, the Kit registration script, the templates and their documentation and tests are this project's own implementation under the repository's MIT license. |

> [!NOTE]
> Aldrin Inbaraj, a lab mate, wrote both upstream projects. The UR5e workstation digital twin, its
> scene definitions, controllers, checkpoints and checks are this project's own work, developed in a fork of his
> repository.

---

## 2. What was rewritten, and what stayed compatible

`omni-kit-mcp` was published without a license file, so its code was replaced rather than kept. The replacement is a
different design (request ids, a token, read-only mode, three scheduling rules, one job record format; see
[the protocol](../mcp/docs/protocol.md)), not a paraphrase. What deliberately stayed the same is the interface other
code depends on: the tool names (`run_python`, `list_tools`, `put_file`, ...), the tool-package contract
(`MCP_NAMESPACE`, `register(registrar)`), the settings names (`TWINSMITH_PORT`, `TWINSMITH_TOOL_MODULES`, ...), the
JSON-lines message shape and the portfile location, so existing tool packages and clients keep working.

`isaac-sim-mcp` (the work cell's origin):
- **Pinned Commit:** `27f6100` (*"workcell: add valid convexHull collider to the Quick-Changer"*)
- **Vendored Path:** `exts/workcell/` (extension code only; the robot, gripper, and scene assets are not in this tree).

---

## 3. License Status

- **`isaac-sim-mcp`:** MIT License. A copy of the upstream license is archived at `docs/provenance/isaac-sim-mcp-LICENSE`.
- **`omni-kit-mcp`:** no upstream license file. No file in this repository is a copy of it any longer.
- **Assets:** the repository ships none (it is code-only; see `docs/ASSET-PACKS.md`). Earlier revisions
  carried `exts/workcell/assets/`, and not all of it could be redistributed. The NVIDIA robot,
  sensor and material files, the Vention bench and the Poliigon oak textures must stay out of a
  public repository; the Husarion gripper files (Apache-2.0) and the FMB objects (CC BY 4.0) may
  ship with notices (`THIRD-PARTY-NOTICES.md`). Each pack records the source, licence and
  `redistributable` flag of every file in its `pack.yaml`. Only the owner's own pinned packs are pulled, and a pack with
  files that are not redistributable is pushed only to a private repository.

## Project scene implementation references

The new `sim-tools` pack and project broker are independently authored; no code was copied from
whats2000/isaacsim-mcp-server (`bfcd2111a9a98657ea87ba3be8564ea19c1a89e6`, MIT),
omni-mcp/isaac-sim-mcp (`949c4886be6e9091ceac8be4272207471b2d3d80`, MIT), or
dcc-mcp/dcc-mcp-openusd (`84595bde4f2dc87ab953688ff5a3123bcec9e88c`, MIT).
Their public sources and licenses were inspected as references. The comparison includes source-derived
signatures and descriptions from whats2000; its notice is retained in `docs/provenance/whats2000-LICENSE`. Work-r-labs/robots
(`ab6aa489855256de07ba46c7b45887fdbc42b607`, root MIT) was evaluated, but individual asset
provenance was not sufficiently established for the candidate and it was not imported or redistributed.

The live non-UR5e fixture was Agilebot GBT-C5A at
[`33281110dd75017fcb821ef4a7316960c0702e18`](https://github.com/sh-agilebot/agilebot_isaac_usd_assets/tree/33281110dd75017fcb821ef4a7316960c0702e18/gbt-c5a).
Its BSD-3-Clause license and repository THIRD_PARTY_LICENSES identify the robot as original
Agilebot work and state that former third-party gripper/camera integrations were removed.
Only the selected robot folder was imported. Robot files are not bundled; screenshots are attributed
in the evidence. The license notice is retained in `docs/provenance/agilebot-LICENSE`.
The generated multifile regression fixture is original CC0 data, not third-party robot geometry.

README pictures (`docs/media/`) are renders and screen captures made with Isaac Sim; no robot or asset files are shipped
with them. NVIDIA's Omniverse licence lets users distribute stills and screen captures they make with Omniverse
(`LICENSE.txt` in the installed `isaacsim` package, section 2.1). The Franka Panda, Nova Carter, UR5e and the Isaac Lab
G1 in the walking clip are NVIDIA Isaac Sim assets. The G1 in `g1-github-scene.png` is from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) (BSD-3-Clause, Copyright Unitree Robotics),
and the SO-ARM100 is from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0). Each
picture says which in `docs/media/README.md`. This is a reading of the licences, not legal advice.
