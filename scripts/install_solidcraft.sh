#!/usr/bin/env bash
# Install SolidCraft (parametric CAD with a headless MCP server) beside twinsmith, in user space.
#
#   scripts/install_solidcraft.sh [--workspace DIR] [--register] [--dry-run]
#
# Downloads the pinned Linux x86_64 tarball from the project's GitHub release, checks it against the
# pinned SHA-256 (the Linux packages are not signed), unpacks it to <workspace>/tools/solidcraft/,
# points `current` at it and links `solidcraft` and `solidcraft-cli` into ~/.local/bin. No sudo.
# --register then adds a headless `solidcraft` MCP server, limited to <workspace>/projects, to every
# MCP client config found (scripts/register_solidcraft_mcp.py). Running it again changes nothing.
# It never starts the desktop app: the app adds its own client entries when it starts.
set -euo pipefail

SOLIDCRAFT_VERSION="0.42.0"   # tag v0.42.0, commit 1b1e940, MIT OR Apache-2.0
SOLIDCRAFT_SHA256="5c05463c8ea247a49269ba63a0c23a3d6cba4d17ae713c655d2f94872fc4424b"
SOLIDCRAFT_ASSET="solidcraft-${SOLIDCRAFT_VERSION}-linux-x86_64.tar.gz"
SOLIDCRAFT_URL="https://github.com/tangency-labs/solidcraft/releases/download/v${SOLIDCRAFT_VERSION}/${SOLIDCRAFT_ASSET}"

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${TWINSMITH_WORKSPACE:-$HOME/robotics-workspace}"
register=0
dry_run=0
while [ $# -gt 0 ]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --register) register=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ "$(uname -s)-$(uname -m)" != "Linux-x86_64" ]; then
  echo "only Linux x86_64 is pinned here; see https://github.com/tangency-labs/solidcraft/releases" >&2
  exit 1
fi

dest="$workspace/tools/solidcraft"
unpacked="$dest/solidcraft-${SOLIDCRAFT_VERSION}-linux-x86_64"
bindir="$HOME/.local/bin"

if [ "$dry_run" = 1 ]; then
  echo "would download $SOLIDCRAFT_URL (sha256 $SOLIDCRAFT_SHA256)"
  echo "would unpack to $unpacked, link $dest/current and $bindir/solidcraft{,-cli}"
  [ "$register" = 1 ] && python3 -I "$here/register_solidcraft_mcp.py" --dry-run \
    --cli "$bindir/solidcraft-cli" --allow-path "$workspace/projects"
  exit 0
fi

if [ ! -x "$unpacked/bin/solidcraft-cli" ]; then
  download="$dest/dl-${SOLIDCRAFT_VERSION}"
  mkdir -p "$download"
  if [ ! -f "$download/$SOLIDCRAFT_ASSET" ]; then
    curl -fL --retry 3 -o "$download/$SOLIDCRAFT_ASSET.part" "$SOLIDCRAFT_URL"
    mv "$download/$SOLIDCRAFT_ASSET.part" "$download/$SOLIDCRAFT_ASSET"
  fi
  echo "$SOLIDCRAFT_SHA256  $download/$SOLIDCRAFT_ASSET" | sha256sum -c -
  tar -xzf "$download/$SOLIDCRAFT_ASSET" -C "$dest"
fi
ln -sfn "solidcraft-${SOLIDCRAFT_VERSION}-linux-x86_64" "$dest/current"
mkdir -p "$bindir"
ln -sf "$dest/current/bin/solidcraft" "$bindir/solidcraft"
ln -sf "$dest/current/bin/solidcraft-cli" "$bindir/solidcraft-cli"
"$bindir/solidcraft-cli" --version

if [ "$register" = 1 ]; then
  mkdir -p "$workspace/projects"
  python3 -I "$here/register_solidcraft_mcp.py" --cli "$bindir/solidcraft-cli" --allow-path "$workspace/projects"
fi
