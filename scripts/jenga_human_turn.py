"""A person's turn at the Jenga tower, in simulation only: wait while a hand is at the tower, then look again from
several sides and rebuild the twin's tower from what the camera saw.

Against a disposable Isaac Sim bridge on Newton, with the work cell and a tower task:

1. The arm goes to a watch pose: the wrist D415 looks at the tower from the robot's own side of it, above it and
   0.6 m away, so the tower and a hand at it are in the picture and the arm is not between the person and the tower.
2. Waiting. The person's turn is scripted: one block is drawn out of the tower, carried off and put down, with a
   stand-in hand (shapes that only a camera sees) going along. The robot holds the watch pose and records until the
   turn is over. **The "hand present" signal is the script's own timer here**: it is true until the scripted turn
   has ended. Finding a real hand is ``sam3``'s zone ``{"hand": "tower"}``, which is to be validated on real camera
   footage and is not exercised by this script.
3. Rescan. The rail and the arm take the camera to the tower's two faces along the rail, to the face toward the rail
   and to a view from above; each look is a clip with depth and the camera's pose, and ``sam3`` gives every visible
   block a pose in the world. A camera enabled in the camera file (any RGB-D camera that stands still, described by
   its intrinsics, depth scale and pose in the world) is recorded and segmented the same way.
4. ``sam3(action="fuse")`` merges the views of every camera into one pose per block, and the blocks become the
   ``robosim`` tower template's ``blocks``: the twin's tower on Newton's MuJoCo-Warp, rebuilt from the cameras alone.
5. The twin is compared with the simulator's own block poses (the stand-in for the real tower): the error of every
   block, the blocks the camera missed and the ones it made up.

It replaces the stage, starts GPU jobs beside the simulator and writes into ``--out``. Nothing here reaches a real
robot. The work cell's tower creeps on its own on Newton, so the views are taken of a tower that moves a little
between them; the result says how much.

    python3 scripts/jenga_human_turn.py --port <port> --task <task.json> --out <folder> --mutate-stage \\
        --discard-changes --allow-concurrent-gpu
"""

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jenga_robot_loop import ARM, BLOCK, Loop, robosim_result, tower_change  # noqa: E402
from twinsmith.isaaclab import jobs  # noqa: E402
from twinsmith.robosim.bridge import RoboSimBridge, _ffmpeg_exe  # noqa: E402
from twinsmith.sam3 import Sam3  # noqa: E402

CARRY_DEG = (90, -110, 130, -110, -90, 0)       # arm folded over its own base, tool down and above the tower: how it rides the rail
VIEW_DISTANCE_M = 0.45                          # camera to a face of the tower
CAMERAS = Path(__file__).with_name("jenga_cameras.json")
TOWER_RADIUS_M = 0.15                           # a block seen further than this from the tower's axis is not in the tower


def look_at(eye, target):
    """Camera axes in the world (columns right, down, forward) for an upright camera at ``eye`` aimed at ``target``."""
    forward = np.asarray(target, dtype=float) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    return np.stack([right, np.cross(forward, right), forward], 1)


def fixed_cameras(path, enable=()):
    """The enabled cameras of a camera file, checked: {name: description}. ``enable`` turns cameras on for this run."""
    listed = json.loads(Path(path).read_text())["cameras"]
    unknown = sorted(set(enable) - set(listed))
    if unknown:
        raise SystemExit(f"{path} lists no camera named {unknown}; it has {sorted(listed)}.")
    chosen = {}
    for name, camera in listed.items():
        if not (camera.get("enabled") or name in enable):
            continue
        numbers = [camera.get(key) for key in ("width", "height", "fx", "fy", "cx", "cy", "depth_scale")]
        pose = np.array(camera.get("world_from_camera"), dtype=float)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0 for v in numbers) or pose.shape != (4, 4) \
                or not np.allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-4) or np.linalg.det(pose[:3, :3]) < 0:
            raise SystemExit(f"camera {name}: needs positive width, height, fx, fy, cx, cy, depth_scale and a 4 x 4 world_from_camera "
                             "whose rotation is a proper rotation.")
        chosen[name] = camera
    return chosen


def views(axis, top, distance=VIEW_DISTANCE_M):
    """Where the wrist camera goes: name, eye, target and where the robot's base stands on the rail (world x).

    The first is the watch pose for a person's turn: on the robot's side of the tower (the rail is at y = 0, the
    tower at negative y), above it and further off, so the arm is not between the person and the tower."""
    x, y = axis
    middle, reach = [x, y, top / 2], BLOCK[0] / 2 + distance
    return [("watch", np.array([x - 0.42, y + 0.34, top + 0.12]), [x, y, top * 0.55], x - reach - 0.26),
            ("front", np.array([x - reach, y, top / 2]), middle, x - reach - 0.26),
            ("above", np.array([x - 0.25, y, top + 0.38]), [x, y, top - 0.04], x - 0.50),
            ("rail_side", np.array([x, y + reach, top / 2]), middle, x - 0.50),
            ("back", np.array([x + reach, y, top / 2]), middle, x + reach - 0.19)]


def tower_from_boxes(boxes, table_z=0.0):
    """Fused boxes as the tower template's blocks. Returns the template rows by name, the tower's axis, the boxes
    that lie away from the tower and the boxes left out because they contradict it.

    A layer is a height: the table is at ``table_z`` and a layer is one block high, less what the stack has sunk,
    which is fitted. Even and odd layers are crossed, and the boxes decide by majority which lie along x; a box
    lying the other way in its layer is a face read wrongly (a long side seen in part looks like an end) and is
    left out. A slot is where a block lies across its layer, from the middle of the tower's footprint."""
    centres = np.array([box["position_world_m"] for box in boxes])
    long = np.array([box["long_axis_world"] for box in boxes])
    weight = np.array([box["detections"] for box in boxes])
    middle = np.median(centres[:, :2], axis=0)
    inside = np.linalg.norm(centres[:, :2] - middle, axis=1) < TOWER_RADIUS_M
    pitch = BLOCK[2]
    for _ in range(3):                                   # layers, then the height of a layer from them, and again
        layers = np.round((centres[:, 2] - table_z) / pitch - 0.5).astype(int)
        pitch = float(((centres[inside, 2] - table_z) * (layers[inside] + 0.5)).sum() / ((layers[inside] + 0.5) ** 2).sum())
    along_x = np.abs(long[:, 0]) >= np.abs(long[:, 1])
    even_along_x = weight[inside & (along_x == (layers % 2 == 0))].sum() * 2 >= weight[inside].sum()
    fits = inside & (along_x == ((layers % 2 == 0) == even_along_x)) & (layers >= 0)
    axis = (centres[fits, :2].min(0) + centres[fits, :2].max(0)) / 2
    rows = {}
    for index in np.flatnonzero(fits):
        box, centre = boxes[index], centres[index]
        across = centre[1] - axis[1] if along_x[index] else centre[0] - axis[0]
        slot = int(round(across / BLOCK[1])) + 1
        name = f"L{layers[index]:02d}_{slot}"
        if 0 <= slot <= 2 and (name not in rows or box["detections"] > rows[name]["detections"]):
            rows[name] = {"name": name, "layer": int(layers[index]), "x": float(centre[0] - axis[0]), "y": float(centre[1] - axis[1]),
                          "yaw": math.atan2(long[index][1], long[index][0]), "z": float(centre[2]), "views": box["views"],
                          "detections": box["detections"]}
    used = {id(boxes[index]) for index in np.flatnonzero(fits)}
    return (rows, axis, [box for box, near in zip(boxes, inside) if not near],
            [box for box, near in zip(boxes, inside) if near and id(box) not in used])


def view_error(objects, truth):
    """One view against the tower as it stood at that look: each box centre to the nearest true block centre, in mm."""
    centres = np.array([pose[:3, 3] for pose in truth.values()])
    gaps = [1000 * float(np.linalg.norm(centres - o["pose"]["position_world_m"], axis=1).min()) for o in objects
            if o.get("pose") and o["pose"]["extent_error_m"] < 0.012]
    return {"box_poses": len(gaps), "centre_error_mm": {"median": round(float(np.median(gaps)), 2), "max": round(max(gaps), 2)}} if gaps else {"box_poses": 0}


def compare(rows, axis, truth):
    """Per-block error of the rebuilt tower against the true poses {name: 4 x 4}: in the plane, in height and in yaw."""
    table = []
    for name in sorted(set(rows) & set(truth)):
        row, pose = rows[name], truth[name]
        true_yaw = math.atan2(pose[1, 0], pose[0, 0])
        turn = (row["yaw"] - true_yaw + math.pi / 2) % math.pi - math.pi / 2          # a block is the same after half a turn
        table.append({"block": name, "xy_error_mm": round(1000 * float(np.hypot(axis[0] + row["x"] - pose[0, 3], axis[1] + row["y"] - pose[1, 3])), 2),
                      "seen_height_error_mm": round(1000 * (row["z"] - float(pose[2, 3])), 2),
                      "twin_height_error_mm": round(1000 * ((row["layer"] + 0.5) * BLOCK[2] - float(pose[2, 3])), 2),
                      "yaw_error_deg": round(math.degrees(turn), 2), "views": len(row["views"])})
    return table, sorted(set(truth) - set(rows)), sorted(set(rows) - set(truth))


class HumanTurn(Loop):
    def go(self, name, eye, target, base_x):
        """Fold the arm, park the rail for this view (its own blocking move), then point the camera."""
        before = self.blocks()
        self.move(self.carry, 3.0)
        self.undisturbed(before, f"folding up before the {name} view")
        before = self.blocks()
        rail_mm = self.rail["real_axis"]["at_default_mm"] + 1000.0 * float(np.clip(base_x, -0.60, 0.60))
        moved = self.park_rail(rail_mm)
        self.undisturbed(before, f"while the rail moved for the {name} view")
        before = self.blocks()
        self.move(self.camera_at(eye, look_at(eye, target)), 4.0)
        self.undisturbed(before, f"on its way to the {name} view")
        return moved

    def look_fixed(self, name, camera, frames=3):
        """Record a short clip from a camera that stands still; returns the clip folder."""
        folder = self.work / f"view_{name}"
        if folder.exists():
            shutil.rmtree(folder)
        intrinsics = {key: camera[key] for key in ("fx", "fy", "cx", "cy", "width", "height", "depth_scale")}
        self.kit(f"recorder = Recorder({'/World/ExtraCameras/' + name!r}, None, {str(folder)!r}, {[camera['width'], camera['height']]!r}, {intrinsics!r})")
        for _ in range(frames):
            self.kit("result = await recorder.frame('still')")
        self.kit("result = recorder.close(10.0)")
        return folder

    def wait(self):
        """Hold the watch pose through the scripted turn, recording the wrist camera; returns what happened when.

        The hand-present signal is faked: it is the scripted turn's own state, true until the hand has gone."""
        folder = self.work / "watch"
        if folder.exists():
            shutil.rmtree(folder)
        self.kit(f"recorder = Recorder({self.camera!r}, {self.wrist!r}, {str(folder)!r}, {self.size!r}, {self.intrinsics!r})")
        steps = 5                                            # the block's pose is written every 10 ms while it is in the hand
        ticks_per_frame = round(0.25 * 500 / steps)
        started, phases, tick = self.clock, [], 0
        while True:
            state = self.kit(f"result = removal.step({self.clock - started})")
            if not phases or phases[-1][0] != state["phase"]:
                phases.append((state["phase"], round(self.clock - started, 2)))
            if state["phase"] == "gone":
                break
            self.step(steps)
            tick += 1
            if tick % ticks_per_frame == 0:
                self.kit("result = await recorder.frame('watch')")
        self.step(500)                                       # one quiet second before the robot moves
        return {"hand_present_signal": "faked: the scripted turn's own timer, not a camera", "phases_at_s": phases,
                "waited_sim_s": round(self.clock - started, 2), "watch_frames": self.kit("result = recorder.close(4.0)")}

    def run(self):
        args = self.args
        extra = fixed_cameras(args.cameras, args.enable_camera)
        robot, tower = self.build(CARRY_DEG, before_play="hand = Hand()\n" + "\n".join(
            f"fixed_camera({'/World/ExtraCameras/' + name!r}, {camera!r})" for name, camera in extra.items()))
        self.carry = {joint: math.radians(value) for joint, value in zip(ARM, CARRY_DEG)}
        truth = self.blocks()
        self.axis = np.mean([pose[:2, 3] for pose in truth.values()], axis=0)
        top = max(pose[2, 3] for pose in truth.values()) + BLOCK[2] / 2
        plan = views(self.axis, top)

        self.record["tower_creep_untouched"] = self.creep()
        print("1. watch and wait", flush=True)
        name, eye, target, base_x = plan[0]
        self.record["views"] = {name: {"camera": "wrist", "rail": self.go(name, eye, target, base_x), "camera_m": eye.round(4).tolist(),
                                       "why": "the watch pose: the tower and a hand at it in view from the robot's side of the tower"}}
        removed = self.block_paths[args.remove]
        toward = [0.0, -1.0, 0.0]                                 # the person sits across the table from the rail
        discard = [float(self.axis[0]) - 0.30, float(self.axis[1]) - 0.25, BLOCK[2] / 2 + 0.002]
        self.kit(f"removal = Removal(hand, {removed!r}, {toward!r}, {discard!r})")
        self.record["waiting"] = self.wait()
        after_turn, rescan_started = self.blocks(), self.clock
        in_tower = {name: pose for name, pose in after_turn.items() if np.hypot(*(pose[:2, 3] - self.axis)) < TOWER_RADIUS_M}
        self.record["removed"] = {"block": args.remove, "left_the_tower": args.remove not in in_tower, "blocks_in_the_tower": len(in_tower),
                                  "what_the_turn_did_to_the_rest": tower_change(truth, after_turn, [args.remove])}

        print("2. rescan", flush=True)
        runs = {}

        def see(name, clip, view):
            """Segment one look and score it against the tower as it stands now."""
            now = self.blocks()
            run, report = self.segment(clip, name)
            objects = json.loads((run / "blocks.json").read_text())["frames"][-1]["objects"]
            runs[name] = run
            self.record["views"][name] = dict(view, sam3_run=str(run), blocks_found=report["prompts"][1]["objects_last_frame"],
                                              sim_time_s=round(self.clock - rescan_started, 1), **view_error(objects, now))
            print(f"  {name}: {json.dumps({key: self.record['views'][name][key] for key in ('blocks_found', 'box_poses', 'centre_error_mm')})}", flush=True)

        for name, camera in extra.items():                  # while the arm is still at the watch pose, out of the way
            see(name, self.look_fixed(name, camera), {"camera": "fixed", "camera_m": [row[3] for row in camera["world_from_camera"][:3]],
                                                      "image": [camera["width"], camera["height"]], "depth_scale": camera["depth_scale"]})
        for name, eye, target, base_x in plan[1:]:
            moved = self.go(name, eye, target, base_x)
            see(name, self.look(f"view_{name}"), {"camera": "wrist", "rail": moved, "camera_m": eye.round(4).tolist()})
        final = self.blocks()
        (self.work / "truth_after_rescan.json").write_text(json.dumps({name: pose.tolist() for name, pose in final.items()}))
        in_tower = {name: pose for name, pose in final.items() if name in in_tower}
        drift = tower_change(after_turn, final, [args.remove])
        self.record["rescan"] = {"simulated_seconds": round(self.clock - rescan_started, 1),
                                 "tower_moved_on_its_own_meanwhile": {"top_layer_mm": round(1000 * drift["top_shift_m"], 2),
                                                                      "largest_block_mm": round(1000 * drift["max_other_shift_m"], 2)},
                                 "scored_against": "the tower as it stood when the last view was taken"}

        print("3. fuse and rebuild the twin", flush=True)
        sam3, stamp = Sam3(job_status=jobs.runs_status), time.strftime("%H%M%S")

        def fuse(label, names):
            answer = sam3.call("fuse", name=f"{self.out.name}-fused-{label}-{stamp}", runs=[runs[name].name for name in names], tag="blocks")
            if not answer.get("ok"):
                raise SystemExit(f"sam3 fuse failed: {answer.get('error')}")
            return answer, json.loads(Path(answer["fused_json"]).read_text())["boxes"]

        fused, boxes = fuse("all", list(runs))
        rows, axis, loose, contradicting = tower_from_boxes(boxes)
        if extra:                                           # what the wrist camera alone would have given
            _, wrist_boxes = fuse("wrist", [name for name in runs if name not in extra])
            wrist_rows, wrist_axis, _, _ = tower_from_boxes(wrist_boxes)
            wrist_table, wrist_missed, wrist_extra = compare(wrist_rows, wrist_axis, in_tower)
            both = [box for box in boxes if set(box["views"]) & {runs[name].name for name in extra} and set(box["views"]) - {runs[name].name for name in extra}]
            self.record["cameras"] = {
                "fixed": sorted(extra), "wrist_only": {"blocks": len(wrist_rows), "missed": wrist_missed, "made_up": wrist_extra,
                                                       "xy_error_mm_median": round(float(np.median([row["xy_error_mm"] for row in wrist_table])), 2)},
                "blocks_seen_by_both": len(both),
                "agreement_between_cameras_mm": {"median": round(1000 * float(np.median([box["spread_m"] for box in both])), 2),
                                                 "max": round(1000 * max(box["spread_m"] for box in both), 2)} if both else None,
                "note": "the fixed camera's pose is the simulator's own, so this agreement is the method's floor, not a calibration result"}
        template = {"shape": "tower", "table_friction": args.table_friction,
                    "blocks": [{key: row[key] for key in ("name", "layer", "x", "y", "yaw")} for row in rows.values()]}
        bridge = RoboSimBridge()
        twin = bridge.call("start", name="jenga-twin", device="cuda:0", template=template, allow_concurrent_gpu=args.allow_concurrent_gpu)
        if not twin.get("ok"):
            raise SystemExit(f"robosim refused the rebuilt tower: {twin.get('error')}")
        robosim_result(bridge, twin["run"])
        bridge.call("rollout", run=twin["run"], take="settle", routine="settle", duration_s=2.0)
        settled = robosim_result(bridge, twin["run"])
        bridge.call("stop", run=twin["run"])
        robosim_result(bridge, twin["run"])

        table, missed, extra = compare(rows, axis, in_tower)
        column = lambda key: [abs(row[key]) for row in table]
        self.record["fusion"] = {key: fused[key] for key in ("fused_json", "views", "boxes", "seen_by_one_view", "largest_spread_m", "detections")}
        self.record["twin"] = {
            "robosim_run": twin["run"], "engine": "Newton MuJoCo-Warp (cuda:0)", "blocks": len(rows), "blocks_in_the_real_tower": len(in_tower),
            "missed": missed, "made_up": extra, "removed_block_absent": args.remove not in rows,
            "seen_away_from_the_tower": [[round(v, 3) for v in box["position_world_m"]] for box in loose],
            "boxes_left_out_as_contradicting_the_tower": len(contradicting),
            "xy_error_mm": {"median": round(float(np.median(column("xy_error_mm"))), 2), "max": max(column("xy_error_mm"))},
            "yaw_error_deg": {"median": round(float(np.median(column("yaw_error_deg"))), 2), "max": max(column("yaw_error_deg"))},
            "seen_height_error_mm": {"median": round(float(np.median(column("seen_height_error_mm"))), 2), "max": max(column("seen_height_error_mm"))},
            "twin_height_error_mm": {"median": round(float(np.median(column("twin_height_error_mm"))), 2), "max": max(column("twin_height_error_mm")),
                                     "why": "the template stacks layers at 14 mm; the work cell's softer contacts let its tower sink"},
            "worst_blocks": sorted(table, key=lambda row: -row["xy_error_mm"])[:5],
            "settles_in_the_twin": (settled.get("push_applied") or {}).get("tower")}
        self.save(table)
        return self.record

    def save(self, table):
        ffmpeg, out = _ffmpeg_exe(), self.out
        with open(out / "per_block_error.csv", "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)
        subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-framerate", "4", "-i", str(self.work / "watch/rgb/%05d.png"),
                        "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", str(out / "watch_pose_during_the_turn.mp4")], check=True)
        for name, view in self.record["views"].items():
            if "sam3_run" not in view:                   # the watch pose: its frames are the waiting clips
                continue
            shutil.copyfile(self.work / f"view_{name}/rgb/00000.png", out / f"view_{name}.png")
            subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-i", str(Path(view["sam3_run"]) / "overlay.mp4"),
                            "-frames:v", "1", str(out / f"view_{name}_masks.png")], check=True)
        (out / "rescan.json").write_text(json.dumps(self.record, indent=1) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--task", required=True, type=Path, help="absolute path of a tower task file; the tower must stand where the camera can go round it")
    parser.add_argument("--out", required=True, help="folder for the videos, rescan.json and per_block_error.csv")
    parser.add_argument("--remove", default="L15_1", help="the block the scripted hand takes: an odd layer's blocks lie across the rail")
    parser.add_argument("--cameras", type=Path, default=CAMERAS, help="camera file: RGB-D cameras besides the wrist camera, each off unless enabled")
    parser.add_argument("--enable-camera", action="append", default=[], metavar="NAME", help="use this camera of the camera file in this run")
    parser.add_argument("--table-friction", type=float, default=0.392, help="block on bench in the twin, as in jenga_robot_loop.py")
    parser.add_argument("--mutate-stage", action="store_true", help="required: this run replaces the open stage")
    parser.add_argument("--discard-changes", action="store_true", help="let quick_start drop unsaved edits")
    parser.add_argument("--allow-concurrent-gpu", action="store_true", help="required: SAM 3 and RoboSimStudio run on the simulator's GPU")
    args = parser.parse_args()
    if not args.mutate_stage or not args.allow_concurrent_gpu:
        parser.error("pass --mutate-stage and --allow-concurrent-gpu: the run replaces the stage and starts GPU jobs beside the simulator")
    if not args.task.is_absolute() or not args.task.is_file():
        parser.error("--task must be an existing absolute path")
    record = HumanTurn(args).run()
    print(json.dumps({key: record[key] for key in ("removed", "twin")}, indent=1)[:3000])


if __name__ == "__main__":
    main()
