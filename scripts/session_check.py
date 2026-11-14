#!/usr/bin/env python3
"""One-line twinsmith health report for an agent's SessionStart hook.

Prints whether a simulator bridge answers, and reminds the agent to use the
twinsmith tools. Always exits 0 so it can never block a session.

    python3 scripts/session_check.py [--port N]
"""
import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mcp"))
try:
    import twinsmith  # noqa: F401  (applies the old variable names, for one release)
except Exception:     # never block a session
    pass


def report(port=None):
    from twinsmith import client
    try:
        client.call("status", {}, port=port, timeout=3)
        state = "a simulator bridge answers"
    except Exception as exc:  # unreachable, several bridges, or an error envelope
        state = f"no bridge answered ({type(exc).__name__}); start one with launch_simulator or ./run.sh"
    return ("twinsmith: " + state + ". Load its tools with ToolSearch 'mcp__twinsmith', call help(task), and use them for "
            "screenshots, recordings, contacts and Isaac Lab runs before writing your own scripts.")


def main(argv=None):
    try:
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.add_argument("--port", type=int, default=None)
        args = ap.parse_args(argv)
        port = args.port if args.port is not None else (int(os.environ["TWINSMITH_PORT"]) if os.environ.get("TWINSMITH_PORT") else None)
        print(report(port))
    except SystemExit:
        print("twinsmith: bad arguments to session_check. Still try the mcp__twinsmith tools first.")
    except Exception as exc:
        print(f"twinsmith: check failed ({type(exc).__name__}). Still try the mcp__twinsmith tools first.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
