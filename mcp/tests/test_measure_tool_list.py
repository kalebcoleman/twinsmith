"""The measurement CLI reports a stable shape, not a fixed tool budget."""

import json
from pathlib import Path
import subprocess
import sys


def test_measure_tool_list():
    script = Path(__file__).resolve().parents[2] / "scripts" / "measure_tool_list.py"
    result = subprocess.run(
        [sys.executable, str(script), "--json", "TWINSMITH_TOOLSETS=default,+computer,+sim"],
        check=True, capture_output=True, text=True,
    )
    report = json.loads(result.stdout)
    assert report["tool_count"] == len(report["tools"])
    assert report["characters"] == sum(row["characters"] for row in report["tools"])
    assert report["estimated_tokens"] == report["characters"] // 4
    assert isinstance(report["instructions_characters"], int)
    assert all(set(row) == {"name", "characters", "estimated_tokens"} for row in report["tools"])
    assert report["tools"] == sorted(report["tools"], key=lambda row: (-row["characters"], row["name"]))
