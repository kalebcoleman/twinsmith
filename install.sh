#!/bin/bash
# Layout: mcp/ is the host MCP package, exts/ holds the Kit extensions (twinsmith.bridge, workcell),
# config/ and scripts/ hold the launch/check helpers (see docs/ARCHITECTURE.md).
#
# install.sh — idempotent one-time setup for twinsmith.
#
# Usage:
#   ./install.sh [options]
#
# Options:
#   --no-sim          Never download or sync Isaac Sim: install the host MCP server only, and record
#                     an installed Isaac Sim 6.x in client configurations when one is found
#   --download-sim    Create the locked 6.2 environment even when an Isaac Sim is already installed
#   --claude          Configure both Claude Code and Claude Desktop
#   --claude-code     Configure Claude Code only
#   --claude-desktop  Configure Claude Desktop only
#   --print-opencode  Print configuration snippet for OpenCode
#   --port N          Bridge port for MCP client configuration (default: 8766)
#   --isaaclab-python PATH  Isaac Lab venv python -> ISAACLAB_PYTHON in client configs ($ISAACLAB_PYTHON by default)
#   --isaaclab-root PATH    Isaac Lab checkout -> ISAACLAB_ROOT in client configs ($ISAACLAB_ROOT by default)
#   --accept-eula     Acknowledge the NVIDIA Omniverse EULA (sets OMNI_KIT_ACCEPT_EULA=YES)
#   -h, --help       Show this help message and exit
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=8766
DISPLAY_NAME="${DISPLAY:-:0}"
NO_SIM=0
DOWNLOAD_SIM=0
DO_CLAUDE_CODE=0
DO_CLAUDE_DESKTOP=0
DO_PRINT_OPENCODE=0
# Optional Isaac Lab install for the isaaclab__* tools (defaults to the same env vars the tools read).
LAB_PYTHON="${ISAACLAB_PYTHON:-}"
LAB_ROOT="${ISAACLAB_ROOT:-}"

usage() {
  cat <<'EOF'
Usage: ./install.sh [options]

Idempotent setup for twinsmith: the host MCP server and, only when needed, Isaac Sim.
An installed Isaac Sim 6.x is used and recorded in the client configurations this writes.

Options:
  --no-sim          Never download or sync Isaac Sim (host MCP server only); an installed
                    Isaac Sim 6.x is still found and recorded in client configurations
  --download-sim    Download the locked Isaac Sim 6.2 environment into .venv even when an
                    Isaac Sim 6.x is already installed (by default an existing install is used)
  (5.1 is used only when named: TWINSMITH_SIM_PYTHON=/path/to/5.1/python ./install.sh --no-sim)
  --claude          Configure both Claude Code and Claude Desktop
  --claude-code     Configure Claude Code only
  --claude-desktop  Configure Claude Desktop only
  --print-opencode  Print configuration snippet for OpenCode
  --port N          Bridge port for MCP client configuration (default: 8766)
  --isaaclab-python PATH  Isaac Lab venv python, written to client configs as ISAACLAB_PYTHON
                          (default: $ISAACLAB_PYTHON if set)
  --isaaclab-root PATH    Isaac Lab checkout, written as ISAACLAB_ROOT (default: $ISAACLAB_ROOT if set)
  --accept-eula    Acknowledge NVIDIA Omniverse EULA (sets OMNI_KIT_ACCEPT_EULA=YES)
  -h, --help        Show this help message and exit
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-sim)
      NO_SIM=1
      shift
      ;;
    --download-sim)
      DOWNLOAD_SIM=1
      shift
      ;;
    --claude)
      DO_CLAUDE_CODE=1
      DO_CLAUDE_DESKTOP=1
      shift
      ;;
    --claude-code)
      DO_CLAUDE_CODE=1
      shift
      ;;
    --claude-desktop)
      DO_CLAUDE_DESKTOP=1
      shift
      ;;
    --print-opencode)
      DO_PRINT_OPENCODE=1
      shift
      ;;
    --port)
      [ "$#" -ge 2 ] || { echo "--port needs a value" >&2; exit 2; }
      PORT="$2"
      shift 2
      ;;
    --isaaclab-python)
      [ "$#" -ge 2 ] || { echo "--isaaclab-python needs a value" >&2; exit 2; }
      LAB_PYTHON="$2"
      shift 2
      ;;
    --isaaclab-root)
      [ "$#" -ge 2 ] || { echo "--isaaclab-root needs a value" >&2; exit 2; }
      LAB_ROOT="$2"
      shift 2
      ;;
    --accept-eula)
      export OMNI_KIT_ACCEPT_EULA=YES
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "$PORT" in
  ''|*[!0-9]*) echo "port must be a number from 1 to 65535, not '$PORT'" >&2; exit 2 ;;
esac
if [ "$PORT" -lt 1 ] || [ "$PORT" -gt 65535 ]; then echo "port must be a number from 1 to 65535, not '$PORT'" >&2; exit 2; fi

if [ "$NO_SIM" = 1 ] && [ "$DOWNLOAD_SIM" = 1 ]; then
  echo "--no-sim and --download-sim cannot be combined" >&2
  exit 2
fi

# Use an Isaac Sim that is already installed (a lab PC's, another environment's) instead of
# downloading ~20 GB. Only the checkout's own .venv is re-synced without asking, because that is
# this installer's environment. The interpreter found here is written into the client
# configuration (TWINSMITH_SIM_PYTHON), so launch_simulator/run.sh use it even from a shell where the
# variables that found it (VIRTUAL_ENV, CONDA_PREFIX, ISAACSIM_PATH, ...) are not set.
# --no-sim never downloads, so a failed look there is only a warning; it still records what it finds.
SIM_PYTHON=""
HOST_ONLY="$NO_SIM"
if [ "$DOWNLOAD_SIM" = 0 ] && { [ "$NO_SIM" = 1 ] || [ ! -x "$ROOT/.venv/bin/python" ]; } \
    && { [ "$NO_SIM" = 0 ] || command -v python3 >/dev/null 2>&1; }; then
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to look for an installed Isaac Sim (or pass --download-sim)" >&2
    exit 1
  fi
  set +e
  FOUND="$(python3 - "$ROOT/scripts/env.py" <<'PY'
import json, os, subprocess, sys
tool = sys.argv[1]
INTERP_VARS = ("TWINSMITH_SIM_PYTHON",)   # env.py EXPLICIT_VARS
ROOT_VARS = ("ISAACSIM_PATH", "ISAAC_PATH")                # env.py STANDALONE_ENV_VARS
listed = subprocess.run([sys.executable, tool, "find", "--json"], capture_output=True, text=True)
if listed.returncode:
    print(listed.stderr.strip() or "env.py find failed", file=sys.stderr)
    sys.exit(3)
found = json.loads(listed.stdout or "[]")
cands = [c for c in found if str(c.get("isaacsim") or "").split(".")[0] == "6"]
# Same precedence as env.py: an interpreter variable wins over an install-root variable.
# Only the variables the user named count (not an activated VIRTUAL_ENV/CONDA_PREFIX), so a typo
# is reported instead of being replaced by whatever environment happens to be active.
named = [v for v in INTERP_VARS if os.environ.get(v, "").strip()]
wanted = {f"env:{v}" for v in named}
if not named:
    named = [v for v in ROOT_VARS if os.environ.get(v, "").strip()]
    wanted = {f"standalone:{v}" for v in named}
if named:
    # The user chose an install: it must be Isaac Sim 6.x, or 5.1 when named this way (auto-detection
    # never picks 5.1); never fall back to a download.
    chosen = [c for c in found if c.get("source") in wanted and (
        c in cands or str(c.get("isaacsim") or "").split(".")[:2] == ["5", "1"])]
    if not chosen:
        print(f"{', '.join(named)} is set but does not point at an Isaac Sim 6.x or 5.1 install "
              "(check it with: python3 scripts/env.py find). Fix or unset it; "
              "pass --download-sim only if you want the locked 6.2 download.", file=sys.stderr)
        sys.exit(3)
    cands = chosen
if not cands:
    sys.exit(4)
c = cands[0]
print(f"{c['path']}\t{c['isaacsim']}\t{c.get('source', '')}")
PY
)"
  rc=$?
  set -e
  if [ "$rc" = 3 ]; then
    [ "$HOST_ONLY" = 1 ] || exit 1
    echo "WARNING: no interpreter recorded (see above); continuing with the host server only" >&2
  elif [ "$rc" = 0 ] && [ -n "$FOUND" ]; then
    IFS=$'\t' read -r SIM_PYTHON SIM_VERSION SIM_SOURCE <<<"$FOUND"
    echo "== using the Isaac Sim already installed: $SIM_PYTHON (Isaac Sim $SIM_VERSION, found via $SIM_SOURCE)"
    echo "   no download. Client configurations written by this installer get TWINSMITH_SIM_PYTHON=$SIM_PYTHON;"
    echo "   for other clients, set it in their twinsmith entry so launch_simulator uses this install."
    echo "   Pass --download-sim to install the locked 6.2 environment instead."
    case "$SIM_VERSION" in
      6.2*) ;;
      5.1*)
        echo "WARNING: Isaac Sim $SIM_VERSION is an explicitly pinned older build. It is used only because you named it;" >&2
        echo "         auto-detection never picks it. PhysX only, no Newton, and fewer checks than 6.2: see docs/COMPATIBILITY.md." >&2 ;;
      *) echo "WARNING: Isaac Sim $SIM_VERSION is not the tested 6.2 release; see docs/COMPATIBILITY.md" >&2 ;;
    esac
    case "$SIM_SOURCE" in
      project-venv|sibling-venv)
        echo "NOTE: this is another checkout's environment ($SIM_PYTHON); keep it in place, or set TWINSMITH_SIM_PYTHON to choose another" >&2 ;;
    esac
    NO_SIM=1
  elif [ "$rc" != 4 ]; then
    echo "could not check for an installed Isaac Sim (exit $rc); not downloading. Pass --no-sim or --download-sim." >&2
    [ "$HOST_ONLY" = 1 ] || exit 1
  fi
fi

if [ "$NO_SIM" = 1 ] && [ -z "$SIM_PYTHON" ]; then
  echo "== --no-sim: no Isaac Sim 6.x found to record. launch_simulator/run.sh look again when they start. If yours is only reachable"
  echo "   through shell variables (an activated venv/conda env, ISAACSIM_PATH), put TWINSMITH_SIM_PYTHON in the client's twinsmith entry."
fi

echo "== checking preconditions"

# Check uv
if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 1
fi

if [ "$NO_SIM" = 0 ]; then
  # 1. OS check: Ubuntu 22.04 / 24.04, x86_64
  codename="$(. /etc/os-release 2>/dev/null; echo "${VERSION_CODENAME:-unknown}")"
  arch="$(uname -m)"
  if [ "$arch" != "x86_64" ] || { [ "$codename" != "jammy" ] && [ "$codename" != "noble" ]; }; then
    echo "Isaac Sim 6.2 supports Ubuntu 22.04/24.04 x86_64" >&2
    exit 1
  fi

  # 2. NVIDIA GPU + driver
  if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "nvidia-smi not found: install the NVIDIA driver" >&2
    exit 1
  fi
  gpu_info="$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1 || true)"
  if [ -z "$gpu_info" ]; then
    echo "nvidia-smi found but reports no GPU" >&2
    exit 1
  fi
  echo "NVIDIA GPU and driver: $gpu_info"
  drv="$(echo "$gpu_info" | awk -F', ' '{print $NF}' | cut -d. -f1)"
  case "$gpu_info" in
    *Blackwell*|*"RTX 50"*) min_drv=580 ;;
    *)                      min_drv=570 ;;
  esac
  if [ "${drv:-0}" -lt "$min_drv" ] 2>/dev/null; then
    echo "WARNING: driver ${drv:-unknown} is below the tested floor (>= $min_drv for $gpu_info)" >&2
  fi

  # 3. Free disk >= 40 GB
  free_gb="$(df -BG --output=avail "$ROOT" | tail -1 | tr -dc '0-9')"
  echo "free disk: ${free_gb} GB"
  if [ "${free_gb:-0}" -lt 40 ]; then
    echo "free disk: ${free_gb:-0} GB (need at least 40 GB for Isaac Sim)" >&2
    exit 1
  fi

  # 4. EULA acknowledged
  if [ "${OMNI_KIT_ACCEPT_EULA:-}" != "YES" ]; then
    echo "NVIDIA Omniverse EULA must be acknowledged: set OMNI_KIT_ACCEPT_EULA=YES" >&2
    exit 1
  fi
fi

# Step 1: Sync Isaac Sim environment (if not --no-sim)
if [ "$NO_SIM" = 0 ]; then
  echo "== syncing Isaac Sim 6.2 environment (Python 3.12)"
  uv sync --python 3.12
fi

# Step 2: Install host MCP package (editable)
MCP_DIR="$ROOT/mcp"
if [ ! -d "$MCP_DIR" ]; then
  echo "FAIL: MCP host package directory not found at $MCP_DIR" >&2
  exit 1
fi

# uv tool install does not read uv.lock (uv 0.13 installs the newest match for the
# pyproject range and ignores hashes). uv sync --locked installs mcp/uv.lock and
# verifies those hashes. UV_PROJECT_ENVIRONMENT keeps the host env out of an
# activated simulator .venv. The isaaclab extra adds tensorboard for the metrics tools.
echo "== syncing locked host environment from mcp/uv.lock (extra: isaaclab)"
uv tool uninstall twinsmith >/dev/null 2>&1 || true
UV_PROJECT_ENVIRONMENT="$MCP_DIR/.venv" \
  uv sync --project "$MCP_DIR" --locked --extra isaaclab --no-dev
if [ ! -x "$MCP_DIR/.venv/bin/twinsmith" ]; then
  echo "FAIL: mcp/.venv has no twinsmith. The locked host sync did not install the project." >&2
  exit 1
fi

# Step 3: Put the locked executable on PATH, then the ffmpeg shim.
BIN_DIR="${HOME}/.local/bin"
mkdir -p "$BIN_DIR"
ln -sfn "$MCP_DIR/.venv/bin/twinsmith" "$BIN_DIR/twinsmith"
FFMPEG_SHIM="$BIN_DIR/ffmpeg"
if [ -L "$FFMPEG_SHIM" ] || { [ -e "$FFMPEG_SHIM" ] && ! grep -Fq '# ffmpeg shim: prefer the static build shipped with imageio-ffmpeg.' "$FFMPEG_SHIM" 2>/dev/null; }; then
  echo "== preserving existing $FFMPEG_SHIM (not a twinsmith shim)" >&2
else
  echo "== installing ffmpeg shim at $FFMPEG_SHIM"
  cat > "$FFMPEG_SHIM" <<'SH'
#!/bin/bash
# ffmpeg shim: prefer the static build shipped with imageio-ffmpeg.
# The Isaac Sim workcell extension shells out to a bare `ffmpeg`; this keeps it
# working on machines with no system package. Re-resolved on every call so a
# version bump in any uv tool env cannot leave a dangling path.
set -euo pipefail
HOST_VENV=__HOST_VENV__
BIN="$(ls -1 "$HOST_VENV"/lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-* "$HOME"/.local/share/uv/tools/*/lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-* 2>/dev/null | sort -V | tail -1 || true)"
if [ -z "$BIN" ]; then
  for candidate in /usr/bin/ffmpeg /usr/local/bin/ffmpeg /snap/bin/ffmpeg; do
    if [ -x "$candidate" ]; then
      exec "$candidate" "$@"
    fi
  done
  echo "ffmpeg shim: no ffmpeg found (install imageio-ffmpeg or system ffmpeg)" >&2
  exit 127
fi
exec "$BIN" "$@"
SH
  # Quoted heredoc, so substitute the checkout venv without expanding $HOME.
  shim_tmp="${FFMPEG_SHIM}.tmp"
  while IFS= read -r line || [ -n "$line" ]; do
    printf '%s\n' "${line/__HOST_VENV__/$MCP_DIR/.venv}"
  done < "$FFMPEG_SHIM" > "$shim_tmp"
  mv "$shim_tmp" "$FFMPEG_SHIM"
  chmod +x "$FFMPEG_SHIM"
fi

# Step 4: Verify CLI help runs
echo "== verifying CLI"
if [ -x "$BIN_DIR/twinsmith" ]; then
  CLI_BIN="$BIN_DIR/twinsmith"
elif command -v twinsmith >/dev/null 2>&1; then
  CLI_BIN="$(command -v twinsmith)"
else
  echo "FAIL: twinsmith not found in $BIN_DIR or PATH" >&2
  exit 1
fi

"$CLI_BIN" --help >/dev/null
echo "  verified $CLI_BIN --help runs"

# Optional client configurations
if [ "$DO_CLAUDE_CODE" = 1 ]; then
  if command -v claude >/dev/null 2>&1; then
    echo "== configuring Claude Code (twinsmith on port $PORT)"
    claude mcp remove --scope user twinsmith >/dev/null 2>&1 || true
    claude mcp remove --scope user twin >/dev/null 2>&1 || true
    lab_env=()
    [ -n "$LAB_PYTHON" ] && lab_env+=(-e "ISAACLAB_PYTHON=$LAB_PYTHON")
    [ -n "$LAB_ROOT" ] && lab_env+=(-e "ISAACLAB_ROOT=$LAB_ROOT")
    [ -n "$SIM_PYTHON" ] && lab_env+=(-e "TWINSMITH_SIM_PYTHON=$SIM_PYTHON")
    claude mcp add --scope user twinsmith \
      -e "DISPLAY=$DISPLAY_NAME" -e "TWINSMITH_PORT=$PORT" "${lab_env[@]}" \
      -- "$CLI_BIN"
  else
    echo "== claude CLI not found; skipped Claude Code configuration" >&2
  fi
fi

if [ "$DO_CLAUDE_DESKTOP" = 1 ]; then
  echo "== configuring Claude Desktop (twinsmith on port $PORT)"
  PORT="$PORT" DISPLAY_NAME="$DISPLAY_NAME" CLI_PATH="$CLI_BIN" LAB_PYTHON="$LAB_PYTHON" LAB_ROOT="$LAB_ROOT" SIM_PYTHON="$SIM_PYTHON" python3 - <<'PY'
import json
import os
import shutil
import sys
import time

port = os.environ["PORT"]
display = os.environ["DISPLAY_NAME"]
cli_path = os.environ["CLI_PATH"]
path = os.path.expanduser("~/.config/Claude/claude_desktop_config.json")
if not os.path.exists(path):
    print(f"Claude Desktop config not found at {path}; skipped")
    sys.exit(0)

env = {"TWINSMITH_PORT": port, "DISPLAY": display}
if os.environ.get("LAB_PYTHON"):
    env["ISAACLAB_PYTHON"] = os.environ["LAB_PYTHON"]
if os.environ.get("LAB_ROOT"):
    env["ISAACLAB_ROOT"] = os.environ["LAB_ROOT"]
if os.environ.get("SIM_PYTHON"):
    env["TWINSMITH_SIM_PYTHON"] = os.environ["SIM_PYTHON"]
entry = {"command": cli_path, "args": [], "env": env}
try:
    with open(path, encoding="utf-8") as f:
        config = json.load(f)
    config.setdefault("mcpServers", {})["twinsmith"] = entry
    backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
    shutil.copy2(path, backup)
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)
except (OSError, ValueError, AttributeError) as exc:
    # The rest of the install succeeded, and this file is the user's: leave it alone and say what to add.
    print(f"Claude Desktop config not changed: {path}: {exc}", file=sys.stderr)
    print('Add this under "mcpServers" yourself, then restart Claude Desktop:', file=sys.stderr)
    print(json.dumps({"twinsmith": entry}, indent=2), file=sys.stderr)
    sys.exit(0)
print(f"updated {path}")
print(f"backup  {backup}")
print("restart Claude Desktop to load the new server")
PY
fi

if [ "$DO_PRINT_OPENCODE" = 1 ]; then
  jstr() { python3 -c 'import json, sys; print(json.dumps(sys.argv[1]))' "$1"; }
  lab_json=""
  [ -n "$LAB_PYTHON" ] && lab_json+=", \"ISAACLAB_PYTHON\": $(jstr "$LAB_PYTHON")"
  [ -n "$LAB_ROOT" ] && lab_json+=", \"ISAACLAB_ROOT\": $(jstr "$LAB_ROOT")"
  [ -n "$SIM_PYTHON" ] && lab_json+=", \"TWINSMITH_SIM_PYTHON\": $(jstr "$SIM_PYTHON")"
  cat <<EOF

OpenCode entry (~/.config/opencode/opencode.jsonc):

  "twinsmith": {
    "type": "local",
    "command": ["$CLI_BIN"],
    "environment": { "TWINSMITH_PORT": "$PORT", "DISPLAY": "$DISPLAY_NAME"$lab_json }
  }
EOF
fi

echo "== install complete"
