# Rebuild from a clean machine

The order to bring a new Ubuntu machine to a working UR5e + RG2 work cell with the wrist camera, and to check the
result. Each step names the page that explains it; this page is the sequence. It has been followed piecewise on two
machines (an RTX PRO 6000 with 96 GB and an RTX 5080 with 16 GB); **nobody has yet run it end to end on a wiped
machine**, so treat the first such run as its test.

## What Git does not hold

Losing the machines loses these unless they are backed up somewhere else:

| What | Where it lives | Why it is not in Git |
|---|---|---|
| The source asset pack `ur5e-jenga-workcell/v2` | `<workspace>/assets/` | Vendor CAD (UR5e, RG2, Quick Changer, Vention bench) that may not be redistributed. The cell pack is built from it; without it the cell cannot be rebuilt |
| Secrets | `~/.config/agent-secrets/`, `~/.cache/huggingface/token` | Secrets. Re-issue them, do not copy them through chat or a repository |
| Your own helper scripts (secret helpers, the local VLM launcher, the laptop setup) | `~/.local/bin/`, `<workspace>/tools/` | Personal machine setup, outside this repository |
| Runs, recordings, datasets, project folders | `<workspace>/runs/`, `datasets/`, `projects/` | Outputs and work in progress |

Everything else below is rebuilt from this repository and public sources.

## Two tiers

`scripts/verify_setup.py` reads the GPU memory and names the tier.

| Tier | GPU memory | What runs |
|---|---|---|
| `sim` | about 12 to 47 GB (for example 16 GB) | Isaac Sim 6.2 with Newton, Isaac Lab 3.0, the work cell and its task packs |
| `full` | about 48 GB and up | The above, plus a local vision model, SAM 3 and GroundingDINO, RoboSimStudio, Graph-as-Policy and VABAR beside the simulator |

## The sequence

1. **Prerequisites.** Ubuntu 22.04 or 24.04, an NVIDIA driver, `git`, `curl` and
   [`uv`](https://docs.astral.sh/uv/). FreeCAD (`freecadcmd` on `PATH`) only to rebuild the wrist bracket. See
   [platforms and prerequisites](LINUX-SETUP.md#platforms-and-prerequisites).

2. **Workspace and checkout.** [Set up a new machine](LINUX-SETUP.md#set-up-a-new-machine), steps 1 to 3:

   ```bash
   export TWINSMITH_WORKSPACE=~/robotics-workspace
   mkdir -p "$TWINSMITH_WORKSPACE"
   git clone <repo-url> "$TWINSMITH_WORKSPACE/twinsmith"
   cd "$TWINSMITH_WORKSPACE/twinsmith"
   python3 scripts/env.py init
   echo 'export TWINSMITH_WORKSPACE=~/robotics-workspace' >> ~/.bashrc
   scripts/check_machine.sh
   ```

3. **Secrets.** Create the folder closed to other users, then put each secret in its own file with your secret
   helper. Nothing in this repository reads a secret from a config file or a command line.

   ```bash
   mkdir -p ~/.config/agent-secrets && chmod 700 ~/.config/agent-secrets
   ```

4. **Simulator and host server.** A new pinned Isaac Sim 6.2 with
   [the pinned installation](LINUX-SETUP.md#new-pinned-62-installation), or an Isaac Sim that is already installed with
   [I already have Isaac Sim installed](LINUX-SETUP.md#i-already-have-isaac-sim-installed). Isaac Lab is optional:
   [Lab, desktop controls and project assets](LINUX-SETUP.md#optional-lab-desktop-controls-and-project-assets).

   ```bash
   ./install.sh --accept-eula          # downloads the locked Isaac Sim 6.2 (about 20 GB), or:
   ./install.sh --no-sim               # host server only; uses the Isaac Sim it finds
   ```

5. **MCP clients.** `./install.sh --claude-code` writes the Claude Code entry and `--print-opencode` prints the
   OpenCode one; the others are in [client configuration](MCP-SERVER.md#4-client-configuration).

6. **Asset packs.** Restore `ur5e-jenga-workcell/v2` from your backup into `<workspace>/assets/`, then build the
   packs the cell uses. Both scripts only read their source and leave an existing pack alone. The second one
   downloads RealSense's D415 mesh (pinned commit, checked by SHA-256) and runs FreeCAD.

   ```bash
   cd mcp
   uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_split_packs.py
   uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/workcell_camera_assets.py
   for pack in ur5e-workcell/v2 jenga/v1; do
     uv run --locked --isolated --python 3.12 --extra scene-test python ../scripts/assets.py doctor "$pack"
   done
   cd ..
   ```

   On a second machine, pull the built packs from your own repository (`twinsmith assets pull --all`,
   [pulling packs](ASSET-PACKS.md#pulling-and-pushing-packs)) or copy the pack folders:
   [move to a second machine](LINUX-SETUP.md#move-to-a-second-machine).

7. **Build the cell.** Start the simulator on Newton, then from an MCP client call `status` once and build:

   ```bash
   ./run.sh --headless --physics-backend newton
   ```

   `workcell__cell(action="quick_start")` builds the base cell; `task="push_block"` or `task="jenga_tower"` adds
   that task's objects ([work cell](../exts/workcell/workcell/README.md)).

8. **Optional tools, `full` tier only.** Each has its own setup page and a pinned commit that the server checks:

   | Tool | Setup | Pin |
   |---|---|---|
   | RoboSimStudio (real-to-sim) | [ROBOSIM.md](ROBOSIM.md) | `ba3650c` |
   | Graph-as-Policy and its skills | [PLANNING-EVAL.md](PLANNING-EVAL.md#setup-the-owner-runs-this) | `06dd619`, skills `b15859a` |
   | VABAR | [PLANNING-EVAL.md](PLANNING-EVAL.md#setup-the-owner-runs-this) | `64b594e` |
   | A vision model endpoint | [PLANNING-EVAL.md](PLANNING-EVAL.md#a-vlm-endpoint-for-graph-nodes) | your own launcher |

   SAM 3 is a gated model: request access on Hugging Face and keep the token in `~/.cache/huggingface/token`.

9. **Verify.**

   ```bash
   python3 scripts/verify_setup.py
   ```

   One line per check: machine, tier, workspace folders, an Isaac Sim 6 interpreter, each asset pack against its
   `pack.yaml`, each optional tool against its pin, the vision model endpoint, and whether the secrets folder is
   closed to other users. It opens no secret. `FAIL` means the work cell will not build; `WARN` is optional or only
   needed to rebuild something.
