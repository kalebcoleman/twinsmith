#!/usr/bin/env python3
"""``twinsmith assets`` from a checkout that is not installed: the commands are in ``mcp/twinsmith/assets_cli.py``."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mcp"))
from twinsmith.assets_cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
