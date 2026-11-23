"""Disturb a puzzle build from outside, and judge a finished one through the dry-run bridge: the benchmark's two hands.

``inject`` moves things on the table of a simulator while a build is running on it, through a second client of the
same bridge (the bridge takes one call at a time, so the move lands between two calls of the build). The build is not
told: it has to notice. Because nothing of it lives in ``puzzle_build.py``, any branch's build can be disturbed the
same way, main's included. Four kinds (``moves``):

``piece_knocked``    one loose piece put ``mm`` aside, across the way it is gripped, on the side with more room;
``base_moved``       the base, with what lies in it, slid ``mm`` and turned ``deg`` about its own middle;
``table_shift``      the base, what lies in it and every loose piece moved ``mm`` together, as if the mat were pulled.

Everything is put where it goes at rest (a teleport, not a push). The simulator's renderer is one picture behind such
a move: the first picture drawn after it still shows the old table, so an injection belongs before a motion of the
arm, not directly before a picture.

``replay`` hands a finished build's run folder to the puzzle project's GaP bridge (``ros2/gap_bridge``, not part of this
repository; no ROS, nothing is sent anywhere): ``export`` then a dry ``run``, and returns the gate log's summary.

``report`` reads what ``replay`` left for every case of a benchmark folder and writes ``bridge_report.md`` there: per
case the goals sent and refused, by which gate and for what, and the goals that follow a corrected base
(``python scripts/puzzle_disturb.py report --out <the benchmark's folder> [--label integ]``).

Simulation only. Nothing here can reach a robot.
"""

import argparse
import collections
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PUZZLE = "/World/Puzzle"
ALONG = np.array([0.8, 0.6, 0.0])                 # the way a slide goes when none fits better: across the table, a little toward its far edge
REGION = ((-0.36, -0.64), (0.40, -0.30))          # what the survey sees and the arm reaches (puzzle_assembly.scatter's)


def _yaw(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    out = np.eye(4)
    out[:2, :2] = [[c, -s], [s, c]]
    return out


def in_base(truth, base, half_m):
    """The pieces that lie in or on the base: within its outline (a square of ``half_m``) as it lies now."""
    back = np.linalg.inv(truth[base])
    return [name for name, at in truth.items() if name != base and np.abs((back @ at)[:2, 3]).max() < half_m]


def moves(kind, truth, base, half_m, mm=0.0, deg=0.0, piece=None, closing=None):
    """Where a disturbance puts things: ``{name: new world pose}``. ``truth`` is ``{name: 4 x 4}`` of everything on the
    table; ``half_m`` half the base's side; ``closing`` the piece's closing axis in its own frame (``piece_knocked``)."""
    far = mm / 1000.0
    if kind == "piece_knocked":
        across = truth[piece][:3, :3] @ np.asarray(closing, dtype=float)
        across[2] = 0.0
        across = across / np.linalg.norm(across)
        room = lambda at: min(float(np.linalg.norm(at[:2, 3] - other[:2, 3])) for name, other in truth.items() if name != piece)      # noqa: E731
        options = []
        for sign in (1, -1):
            at = truth[piece].copy()
            at[:3, 3] += sign * far * across
            options.append(at)
        return {piece: max(options, key=room)}
    if kind == "base_moved":
        slide = _yaw(deg)
        slide[:3, 3] = truth[base][:3, 3] - slide[:3, :3] @ truth[base][:3, 3] + far * ALONG
        return {name: slide @ truth[name] for name in (base, *in_base(truth, base, half_m))}
    if kind == "table_shift":
        low, high = np.array(REGION[0]), np.array(REGION[1])
        inside = lambda way: sum(bool(np.all(at[:2, 3] + far * way[:2] >= low) and np.all(at[:2, 3] + far * way[:2] <= high)) for at in truth.values())      # noqa: E731
        ways = [np.array([math.cos(a), math.sin(a), 0.0]) for a in np.radians(np.arange(0, 360, 45))]
        way = max(ways, key=lambda w: (inside(w), float(w @ ALONG)))     # the pull that leaves most of it where the camera sees and the arm reaches
        out = {}
        for name, at in truth.items():
            out[name] = at.copy()
            out[name][:3, 3] += far * way
        return out
    raise ValueError(f"no such disturbance: {kind}")


def _quat(turn):
    """A rotation matrix as x, y, z, w (what the simulator's ``place_bodies`` takes)."""
    w = math.sqrt(max(0.0, 1.0 + turn[0, 0] + turn[1, 1] + turn[2, 2])) / 2
    if w > 1e-6:
        return [(turn[2, 1] - turn[1, 2]) / (4 * w), (turn[0, 2] - turn[2, 0]) / (4 * w), (turn[1, 0] - turn[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(turn)[:3]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + turn[i, i] - turn[j, j] - turn[k, k])) * 2
    q = [0.0, 0.0, 0.0, (turn[k, j] - turn[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (turn[j, i] + turn[i, j]) / s, (turn[k, i] + turn[i, k]) / s
    return q


def inject(port, disturbance, layout, config=None, host="127.0.0.1"):
    """Make one disturbance on the simulator at ``port``, now. ``disturbance`` is the benchmark's
    ``{"kind", "mm", "deg", "piece"?}``; ``layout`` names what is on the table (its keys; the first is the base).
    Returns what was moved: ``{"kind", "moved": {name: mm it went}, "turned_deg"}``. The poses are read from, and
    written to, the simulator through the session the build itself set up there."""
    sys.path.insert(0, str(ROOT / "mcp"))
    from twinsmith.client import BridgeClient

    import puzzle_scan

    config = config or json.loads((ROOT / "scripts/puzzle_build.json").read_text())
    names = list(layout)
    base, paths = names[0], {name: f"{PUZZLE}/{name}" for name in names}
    client = BridgeClient(host=host, port=port)

    def kit(code):
        reply = client.call("run_python", {"code": code, "session_id": "workcell_session", "persistent": True}, timeout=300)
        if reply.get("status") == "error" or reply.get("error"):
            raise RuntimeError(str(reply.get("message") or reply.get("error"))[:600])
        return reply.get("result")

    found = kit(f"result = poses({list(paths.values())!r})")
    truth = {name: np.array(found[path]) for name, path in paths.items()}
    kind = "table_shift" if disturbance["kind"].startswith("table_shift") else disturbance["kind"]
    piece = disturbance.get("piece")
    boxes = puzzle_scan.load_pack(config["pack"]).shape(base).boxes
    half = float(min((np.max([c + h for c, h in boxes], axis=0) - np.min([c - h for c, h in boxes], axis=0))[:2]) / 2)
    put = moves(kind, truth, base, half, disturbance.get("mm", 0.0), disturbance.get("deg", 0.0), piece,
                config["grasps"][piece]["closing_axis"] if piece else None)
    rows = {paths[name]: [*map(float, at[:3, 3]), *map(float, _quat(at[:3, :3]))] for name, at in put.items()}
    kit(f"place_bodies({rows!r})")
    return {"kind": disturbance["kind"], "turned_deg": disturbance.get("deg", 0.0) if kind == "base_moved" else 0.0,
            "moved": {name: round(1000 * float(np.linalg.norm(at[:3, 3] - truth[name][:3, 3])), 2) for name, at in put.items()}}


def replay(run, bridge):
    """A finished build through the GaP bridge's gates, dry: the gate log's summary (goals, goals sent to the mock,
    goals refused and by which gate, corrected goals), or None when the bridge could make no plan of the run (it
    says why on stderr, kept in ``<run>/bridge/export.log``). ``bridge`` is the puzzle project's ``ros2`` folder.
    A refused goal is a result, not a failure of the replay."""
    run, out = Path(run), Path(run) / "bridge"
    out.mkdir(exist_ok=True)
    made = subprocess.run([sys.executable, "-m", "gap_bridge.cli", "export", str(run), "-o", str(out / "plan.json"), "--keep-refused"],
                          cwd=str(bridge), capture_output=True, text=True)
    (out / "export.log").write_text(made.stdout + made.stderr)
    if made.returncode or not (out / "plan.json").is_file():
        return None
    ran = subprocess.run([sys.executable, "-m", "gap_bridge.cli", "run", str(out / "plan.json"), "--log", str(out / "dry_run")],
                         cwd=str(bridge), capture_output=True, text=True)
    (out / "run.log").write_text(ran.stdout + ran.stderr)
    log = out / "dry_run" / "gate_log.json"
    if not log.is_file():
        return None
    plan = json.loads((out / "plan.json").read_text())
    corrected = [a["corrected"]["gates_at_export"]["pass"] for a in plan["actions"] if a.get("corrected")]
    return {**json.loads(log.read_text())["summary"], "corrected_goals": len(corrected), "corrected_goals_refused": corrected.count(False)}


def cause(why):
    """A gate's reason in a few words, so that reasons can be counted: the numbers taken out."""
    if "camera cord" in why:
        return "camera cord over its spare loop"
    joint = re.search(r"(\w+_joint) goes .* outside .* of (the cell's joint limits|the cable's window)", why)
    if joint:
        return f"{joint.group(1)} outside {joint.group(2)}"
    return re.sub(r"-?\d+(\.\d+)?", "N", why)[:80]


def judged(folder):
    """One replayed build as the report reads it: ``{"goals", "sent", "refused": [{"goal", "gate", "causes"}],
    "corrected", "corrected_refused", "no_plan"}``. ``no_plan`` is the export's reason when the bridge made no plan of
    the run (a build in which the arm was blocked is not one path), else None."""
    bridge = Path(folder) / "bridge"
    log, plan = bridge / "dry_run" / "gate_log.json", bridge / "plan.json"
    if not log.is_file():
        said = (bridge / "export.log").read_text().strip().splitlines() if (bridge / "export.log").is_file() else []
        return {"goals": 0, "sent": 0, "refused": [], "corrected": 0, "corrected_refused": 0, "no_plan": said[-1][:300] if said else "not replayed"}
    record, refused = json.loads(log.read_text()), []
    for action in record["actions"]:
        failed = [gate for gate in action.get("gates") or [] if not gate["pass"]]
        if failed:
            refused.append({"goal": action["id"], "gate": failed[0]["gate"], "causes": sorted({cause(why) for gate in failed for why in gate["why"]})})
    corrected = [action["corrected"]["gates_at_export"]["pass"] for action in json.loads(plan.read_text())["actions"] if action.get("corrected")]
    return {"goals": record["summary"]["goals"], "sent": record["summary"]["goals_sent"], "refused": refused, "corrected": len(corrected),
            "corrected_refused": corrected.count(False), "no_plan": None}


def report(out, label=None):
    """Write ``<out>/bridge_report.md`` from the rows of a benchmark folder (``row.json`` of every case, of ``label``
    when one is given) and what ``replay`` left beside each build. Returns the text."""
    out = Path(out).resolve()
    found = ((path.parent, json.loads(path.read_text())) for path in out.glob("*/**/row.json"))
    # A column's rows lie at <out>/<label>/<case>: rows kept beside them (a superseded head's, a trial's) are not the benchmark's.
    rows = sorted(({**row, "folder": str(folder)} for folder, row in found if folder == out / row["label"] / row["case"]),
                  key=lambda row: (row["label"], row["case"]))
    rows = [row for row in rows if label in (None, row["label"])]
    lines = ["# The puzzle builds through the GaP bridge, dry", "",
             "Every build of the benchmark replayed through the puzzle project's `ros2/gap_bridge` (`export`, then a dry `run`: the built-in mock "
             "controller, no ROS, nothing sent anywhere). The limits are the bridge's shipped ones; its cable limits are placeholders that are NOT "
             "MEASURED. A goal is what the bridge would send as one trajectory; the plan re-solves the simulator's sampled joint feedback on the "
             "calibrated arm, so this is no statement about how closely a real arm would follow it.", "",
             "| Case | Label | Seated | Goals | Sent | Refused | By gate | After a corrected base (refused) | Why |", "|---|---|---|--:|--:|--:|---|---|---|"]
    totals, causes = collections.defaultdict(lambda: collections.Counter()), collections.defaultdict(lambda: collections.Counter())
    for row in rows:
        found = judged(row["folder"])
        seated = row["pieces_total"] is not None and row["pieces_seated"] == row["pieces_total"]
        gates = collections.Counter(goal["gate"] for goal in found["refused"])
        why = collections.Counter(reason for goal in found["refused"] for reason in goal["causes"])
        replayed = seated and found["no_plan"] is None
        totals[row["label"]].update(builds=1, seated=int(seated), seated_replayed=int(replayed), seated_clean=int(replayed and not found["refused"]),
                                    goals=found["goals"] * replayed, refused=len(found["refused"]) * replayed)
        causes[row["label"]].update(why)
        lines.append(f"| {row['case']} | {row['label']} | {'yes' if seated else 'no'} | {found['goals']} | {found['sent']} | {len(found['refused'])} | "
                     + (", ".join(f"gate {gate}: {count}" for gate, count in sorted(gates.items())) or "-") + f" | {found['corrected']} ({found['corrected_refused']}) | "
                     + (found["no_plan"] or "; ".join(f"{reason} ({count})" for reason, count in why.most_common()) or "-") + " |")
    lines += ["", "## Totals", "", "| Label | Builds | Seated | Seated and replayed | Seated with no goal refused | Goals of the seated builds | Refused of them |", "|---|--:|--:|--:|--:|--:|--:|"]
    lines += [f"| {name} | {total['builds']} | {total['seated']} | {total['seated_replayed']} | {total['seated_clean']} | {total['goals']} | {total['refused']} |"
              for name, total in totals.items()]
    for name, counted in causes.items():
        if counted:
            lines += ["", f"## Why goals were refused: {name}", ""] + [f"- {reason}: {count} goals" for reason, count in counted.most_common()]
    text = "\n".join(lines) + "\n"
    (out / "bridge_report.md").write_text(text)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("action", choices=["report"])
    parser.add_argument("--out", type=Path, required=True, help="the benchmark's folder (the one puzzle_benchmark.py run and summary use)")
    parser.add_argument("--label", help="only the builds of this label")
    args = parser.parse_args()
    print(report(args.out, args.label))


if __name__ == "__main__":
    main()
