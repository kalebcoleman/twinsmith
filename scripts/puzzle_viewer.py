#!/usr/bin/env python3
"""Build the step viewer of a puzzle build: a static page that replays what was planned and what was done.

    python3 scripts/puzzle_viewer.py <run folder>          # the --out folder of scripts/puzzle_build.py

It reads the run's own records (``run_log.json``, ``metrics.json``, ``graph/workflow.json``, ``trace/dag_trace.json``,
the film's frames) and writes ``<run folder>/viewer``: ``index.html`` with the page's script and style (copied from
``scripts/viewer``), ``run.js`` (everything the page shows, as one JSON value), ``timing.json`` (where the wall time
went) and ``media`` (the build from each camera, a clip per step, a clip per lookahead try, the looks' pictures). The page needs no server and nothing from the
network. The simulator is not involved: a finished run can be rebuilt into a viewer at any time.

Time on the page is the film's: seconds of simulated time since its first frame. A step that takes no simulated time
(a SAM 3.1 job, a lookahead) is a moment on that line; its wall time is in the step list.
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

PAGE = Path(__file__).with_name("viewer")
CATEGORIES = ("physics", "rendering", "bridge", "sam3", "scan", "planning", "other")
PANELS = ("video", "future", "graph", "steps", "plan", "plots", "perception", "timing")


def graph(workflow):
    """A workflow as the page draws it: its nodes in the order they were written, a subgraph with its inner nodes
    in the order a run meets them and its exits, and the labelled edges between the top-level nodes."""
    def inner(sub):
        after = {}
        for a, b in sub["edges"]:
            after.setdefault(a, []).append((b, None))
        for a, routes in sub.get("conditional_edges", {}).items():
            after.setdefault(a, []).extend((b, value) for value, b in routes["mapping"].items())
        exits, order, queue = set(sub["exit"]["success_values"]), [], ["START"]
        while queue:
            name = queue.pop(0)
            for following, _ in after.get(name, []):
                if following not in order and following not in exits and following != "END":
                    order.append(following)
                    queue.append(following)
        return {"children": [{"name": name, "type": sub["nodes"][name]["type"],
                              "does": sub["nodes"][name].get("tool") or sub["nodes"][name].get("script", ""),
                              "routes": {value: b for b, value in after.get(name, []) if value}} for name in order],
                "exits": [*sub["exit"]["success_values"], sub["on_error"]]}

    nodes = []
    for name, node in workflow["nodes"].items():
        row = {"name": name, "type": node["type"], "status": node.get("status")}
        if node["type"] == "subgraph":
            sub = workflow["subgraphs"][node["ref"]]
            row.update(skill=sub["skill"], **inner(sub))
        nodes.append(row)
    edges = [{"from": a, "to": b, "label": None} for a, b in workflow["edges"]]
    for a, routes in workflow.get("conditional_edges", {}).items():
        edges += [{"from": a, "to": b, "label": value} for value, b in routes["mapping"].items()]
    return {"name": workflow["meta"]["name"], "description": workflow["meta"].get("description", ""), "nodes": nodes, "edges": edges}


def film_time(steps, fps):
    """A function from wall time (epoch seconds) to the film's time, from the top-level steps: inside a step the
    two run together, between steps the film stands still."""
    rows = sorted((row for row in steps if row["parent"] is None), key=lambda row: row["wall_start"])

    def at(wall):
        time = 0.0
        for row in rows:
            if wall < row["wall_start"]:
                break
            start, end = row["frame_start"] / fps, row["frame_end"] / fps
            part = min(1.0, (wall - row["wall_start"]) / row["wall_s"]) if row["wall_s"] > 0 else 1.0
            time = start + (end - start) * part
        return round(time, 3)
    return at


def node_spans(trace, at):
    """When each node of the graph ran, in the film's time: ``{node: [[start, end, status]]}``, one span per
    visit, and after them the visit's wall seconds and the wall time it ended. A node that never finished has no
    end and the status ``running``. The trace times a subgraph's inner nodes (``subgraph.node``) and not the
    subgraph, so a subgraph gets one span, from its first inner node's start to its last one's end."""
    spans = {}
    for event in sorted(trace["events"], key=lambda event: event["seq"]):
        if event["event_type"] == "node_started":
            spans.setdefault(event["node_name"], []).append([at(event["timestamp"]), None, "running", event["timestamp"]])
        elif event["event_type"] == "node_finished" and spans.get(event["node_name"]):
            span = spans[event["node_name"]][-1]
            span[1], span[2] = at(event["timestamp"]), event["status"] or "ok"
            span[3:] = [round(event["timestamp"] - span[3], 3), event["timestamp"]]
    for outer in sorted({name.split(".")[0] for name in spans if "." in name}):
        inner = [span for name, visits in spans.items() if name.startswith(outer + ".") for span in visits]
        ended = all(len(span) == 5 for span in inner)
        status = "running" if not ended else "ok" if all(span[2] == "ok" for span in inner) else "error"
        last = max(inner, key=lambda span: span[4]) if ended else None
        spans[outer] = [[min(span[0] for span in inner), last[1] if ended else None, status, *([round(sum(span[3] for span in inner), 3), last[4]] if ended else [])]]
    return spans


def timing(steps, wall_s, simulated_s, spans):
    """Where the wall time went. Per kind of top-level step and in all: seconds stepping physics, rendering frames,
    in bridge calls beyond those two, waiting for SAM 3.1, matching the scan, choosing grasps, and the rest of the
    step; then what lies outside the steps (the graph's own work and the vision-language model's answers)."""
    def split(row):
        spent = row["spent"]
        parts = {"physics": spent.get("physics_s", 0.0), "rendering": spent.get("render_s", 0.0), "sam3": spent.get("sam3_s", 0.0),
                 "scan": spent.get("scan_s", 0.0), "planning": spent.get("plan_s", 0.0)}
        parts["bridge"] = max(0.0, spent.get("bridge_s", 0.0) - parts["physics"] - parts["rendering"])
        parts["other"] = max(0.0, row["wall_s"] - sum(parts.values()))
        return parts

    kinds = {}
    for row in steps:
        if row["parent"] is None:
            group = kinds.setdefault(row["kind"], {"kind": row["kind"], "count": 0, "wall_s": 0.0, "sim_s": 0.0, "bridge_calls": 0,
                                                   **{name: 0.0 for name in CATEGORIES}})
            group["count"] += 1
            group["wall_s"] += row["wall_s"]
            group["sim_s"] += row["sim_s"]
            group["bridge_calls"] += int(row["spent"].get("bridge_calls", 0))
            for name, seconds in split(row).items():
                group[name] += seconds
    rows = sorted(kinds.values(), key=lambda group: -group["wall_s"])
    total = {"kind": "all steps", **{key: sum(group[key] for group in rows) for key in ("count", "wall_s", "sim_s", "bridge_calls", *CATEGORIES)}}
    model = sum(span[3] for name, visits in spans.items() if name.endswith(".ask") for span in visits if len(span) == 5)
    for group in (*rows, total):
        group.update({key: round(group[key], 1) for key in ("wall_s", "sim_s", *CATEGORIES)})
    return {"categories": list(CATEGORIES), "rows": rows, "total": total, "wall_s": wall_s, "simulated_s": simulated_s,
            "slower_than_real_time": round(wall_s / simulated_s, 1) if simulated_s else None,
            "outside_steps": {"vlm_s": round(model, 1), "graph_and_other_s": round(max(0.0, wall_s - total["wall_s"] - model), 1)}}


def data(run):
    """Everything the page shows, from a run folder's records."""
    run = Path(run)
    log, metrics = json.loads((run / "run_log.json").read_text()), json.loads((run / "metrics.json").read_text())
    workflow = json.loads((run / "graph/workflow.json").read_text())
    trace = json.loads((run / "trace/dag_trace.json").read_text())
    fps, zero = log["film"]["fps"], log["film"]["zero_s"]
    at = film_time(log["steps"], fps)
    spans = node_spans(trace, at)
    inside = {}                                             # step id -> the lookahead try it is part of
    steps = []
    for row in log["steps"]:
        clip = row["plan"].get("clip") if row["kind"] == "try" else inside.get(row["parent"])
        inside[row["id"]] = clip
        depth = 0 if row["parent"] is None else steps[row["parent"]]["depth"] + 1
        steps.append({**{key: row.get(key) for key in ("id", "kind", "piece", "parent", "plan", "result", "error", "sim_s", "wall_s", "spent")},
                      "depth": depth, "try": clip, "t0": round(row["frame_start"] / fps, 3), "t1": round(row["frame_end"] / fps, 3),
                      "wall_at": round(row["wall_start"] - log["wall_start"], 1),
                      "clip": f"media/steps/{row['id']:03d}_{row['kind'].replace(' ', '_')}.mp4" if clip is None and row["frame_end"] - row["frame_start"] > 1 else None})

    def samples(rows, zero_s):
        return [[round(seconds - zero_s, 3), round(rail * 1000, 2), *(round(q * 57.29578, 2) for q in arm), round(gap * 1000, 2)]
                for seconds, rail, *arm, gap in rows]

    tries = {name: {"piece": clip["piece"], "src": f"media/lookahead/{name}.mp4", "duration": round(len(clip["captions"]) / clip["fps"], 2),
                    "captions": clip["captions"], "samples": samples(clip["samples"], clip["zero_s"]),
                    "grips": [[round(seconds - clip["zero_s"], 3), force] for seconds, force in clip["grips"]]}
             for name, clip in log["tries"].items()}
    looks = [{**look, "t": at_step["t1"]} for look in log["looks"]
             for at_step in [next(step for step in steps if step["plan"].get("look") == look["index"] and step["try"] is None)]]
    for look in looks:
        for key in ("picture", "masks"):
            if look.get(key):
                look[key] = f"media/looks/{look[key]}"
    ending = next(node for node, row in workflow["nodes"].items() if row["type"] == "end" and row.get("status") == metrics["graph"]["run"]["exit_status"])
    spans[ending] = [[round(len(log["film"]["captions"]) / fps, 2)] * 2 + ["ok" if metrics["graph"]["run"]["success"] else "error"]]
    checks = []                                             # each checkpoint, after the last step that ended before its subgraph did
    for check in metrics["graph"]["run"].get("checkpoints", []):
        visit = next((visit for visit in reversed(spans.get(check["subgraph"], [])) if len(visit) == 5), None)
        if visit:
            ended = [row["id"] for row in log["steps"] if row["parent"] is None and row["wall_start"] + row["wall_s"] <= visit[4] + 0.5]
            checks.append({**check, "t": visit[1], "after": ended[-1] if ended else None})
    return {
        "title": run.name, "duration": round(len(log["film"]["captions"]) / fps, 2), "fps": fps,
        "views": [{"name": name, "src": f"media/{name}.mp4"} for name in reversed(log["film"]["cameras"])],
        "summary": {key: metrics.get(key) for key in ("completed", "simulated_seconds", "wall_seconds", "perception", "seed", "order")},
        "pieces": metrics["pieces"], "result": metrics["graph"]["run"], "checks": metrics["graph"].get("checks"),
        "steps": steps, "tries": tries, "captions": log["film"]["captions"],
        "columns": ["seconds", "rail, mm", *(name.replace("_joint", "") + ", deg" for name in log["sample_columns"][2:8]), "pad gap, mm"],
        "samples": samples(log["samples"], zero), "grips": [[round(seconds - zero, 3), force] for seconds, force in log["grips"]],
        "looks": looks, "graph": graph(workflow), "spans": {name: [span[:3] for span in visits] for name, visits in spans.items()},
        "checkpoints": checks, "timing": timing(log["steps"], log["wall_s"], metrics["simulated_seconds"], spans), "panels": list(PANELS)}


def encode(ffmpeg, folders, start, count, fps, out):
    """Frames ``start`` to ``start + count`` of one or more cameras (folders of ``%05d.jpg``), side by side, as an MP4
    that plays at the simulation's own speed."""
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y"]
    for folder in folders:
        command += ["-framerate", str(fps), "-start_number", str(start), "-i", str(folder / "%05d.jpg")]
    width = 960 if len(folders) == 1 else 640
    scaled = "".join(f"[{index}:v]scale={width}:-2[v{index}];" for index in range(len(folders)))
    joined = scaled + "".join(f"[v{index}]" for index in range(len(folders))) + f"hstack=inputs={len(folders)}" if len(folders) > 1 else f"scale={width}:-2"
    command += ["-frames:v", str(count), "-filter_complex" if len(folders) > 1 else "-vf", joined,
                "-c:v", "libx264", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)]
    subprocess.run(command, check=True)


def build(run, ffmpeg=None):
    """Write ``<run>/viewer`` and return its ``index.html``. ``ffmpeg`` is the encoder to use; without one, the one
    that comes with twinsmith's host environment."""
    run = Path(run).resolve()
    if ffmpeg is None:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mcp"))
        from twinsmith.robosim.bridge import _ffmpeg_exe
        ffmpeg = _ffmpeg_exe()
    page = data(run)
    out = run / "viewer"
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(PAGE, out)
    media, fps = out / "media", page["fps"]
    cameras = [view["name"] for view in page["views"]]
    film = [run / "film" / name / "rgb" for name in cameras]
    frames = round(page["duration"] * fps)
    for name, folder in zip(cameras, film):
        encode(ffmpeg, [folder], 0, frames, fps, media / f"{name}.mp4")
    for step in page["steps"]:
        if step["clip"]:
            encode(ffmpeg, film, round(step["t0"] * fps), round((step["t1"] - step["t0"]) * fps), fps, out / step["clip"])
    for name, clip in page["tries"].items():
        encode(ffmpeg, [run / "lookahead" / name / camera / "rgb" for camera in cameras], 0, len(clip["captions"]), fps, out / clip["src"])
    for look in page["looks"]:
        for key in ("picture", "masks"):
            if look.get(key):
                (media / "looks").mkdir(parents=True, exist_ok=True)
                shutil.copy(run / Path(look[key]).name, out / look[key])
    (out / "run.js").write_text("window.RUN = " + json.dumps(page, separators=(",", ":")) + ";\n")
    (out / "timing.json").write_text(json.dumps(page["timing"], indent=1) + "\n")
    return out / "index.html"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run", type=Path, help="the --out folder of a finished scripts/puzzle_build.py run")
    print(build(parser.parse_args().run))


if __name__ == "__main__":
    main()
