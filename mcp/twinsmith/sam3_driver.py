"""SAM 3 over a clip's frames, run by ``sam3.py`` with the SAM 3 tool bundle's own interpreter.

    sam3_driver.py <request.json>

The host never imports this file's heavy parts: ``torch`` and the upstream ``sam3`` package are imported in
``main`` only, so the functions above it need numpy alone and are what the host tests exercise.

Per prompt (``tag``) it writes ``<out>/<tag>/labels_NNNN.png`` (uint8 object ids, 0 = nothing; ids are 1..N in order
of first appearance, larger objects drawn first so small ones stay visible) and ``<out>/<tag>.json`` with, per frame
and object: area, centroid, box, principal-axis angle and score. With depth and intrinsics an object also gets
``xyz_m`` (the median of its points) and, when the prompt has a box size, ``pose``: the box that the visible face
belongs to. Everything 3-D is in the camera's optical frame (+X right, +Y down, +Z forward), and in the world frame
too when the camera's pose is given. ``<out>/overlay.mp4`` shows every mask over the frames; ``<out>/result.json`` is
the summary ``sam3(action="report")`` reads.
"""
import colorsys
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

IMAGES = (".jpg", ".jpeg", ".png")
# A box face seen from outside: (long in-plane axis, short in-plane axis, hidden axis) as indices into [length, width, height].
FACES = {"top": (0, 1, 2), "side": (0, 2, 1), "end": (1, 2, 0)}
PLANE_TOLERANCE_M = 0.002          # a point this close to the fitted face plane belongs to the face
MIN_FACE_POINTS = 30
ZONE_MARGIN = 0.5                  # a zone is its objects' box grown on every side by this fraction of its longer side
ZONE_DEPTH_M = 0.30                # with depth, an object counts only within this distance of the zone along the optical axis


def frame_objects(ids, masks, probs, remap):
    """One frame of SAM 3 output as ``(label map, object rows)``. ``remap`` (SAM 3 id -> 1..N) is extended in place."""
    label = np.zeros(masks.shape[1:], np.uint8)
    rows = []
    for j in np.argsort([-int(m.sum()) for m in masks]):
        mask = masks[j] > 0
        area = int(mask.sum())
        if area == 0:
            continue
        oid = remap.setdefault(int(ids[j]), len(remap) + 1)
        if oid < 255:
            label[mask] = oid
        ys, xs = np.nonzero(mask)
        axis_deg, spread = 0.0, [0.0, 0.0]
        if area > 3:
            evals, evecs = np.linalg.eigh(np.cov(np.stack([xs, ys]).astype(np.float64)))
            axis_deg = float(np.degrees(np.arctan2(-evecs[1, 1], evecs[0, 1])) % 180.0)      # image y is down
            spread = [float(np.sqrt(max(evals[1], 0))), float(np.sqrt(max(evals[0], 0)))]
        rows.append({"id": oid, "area": area, "cx": float(xs.mean()), "cy": float(ys.mean()),
                     "bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                     "axis_deg": axis_deg, "spread_px": spread, "score": float(probs[j])})
    return label, sorted(rows, key=lambda row: row["id"])


def backproject(depth, mask, camera):
    """Camera-frame points [N, 3] of a mask's pixels that have a depth (metres along the optical axis)."""
    ys, xs = np.nonzero(mask)
    z = depth[ys, xs].astype(np.float64)
    keep = np.isfinite(z) & (z > 0)
    xs, ys, z = xs[keep], ys[keep], z[keep]
    return np.stack([(xs - camera["cx"]) * z / camera["fx"], (ys - camera["cy"]) * z / camera["fy"], z], 1)


def _plane(points, rng, tries=96):
    """The plane holding the most points (RANSAC, then a least-squares fit of its inliers): ``(inliers, centre, axes)``.
    ``axes`` rows are two in-plane directions and the normal, which points at the camera (the origin)."""
    best = np.zeros(len(points), bool)
    for _ in range(tries):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-12:
            continue
        inliers = np.abs((points - a) @ (normal / length)) < PLANE_TOLERANCE_M
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < MIN_FACE_POINTS:
        return best, None, None
    for _ in range(2):                                # refit on the inliers, and take the inliers of the refit
        centre = points[best].mean(0)
        axes = np.linalg.svd(points[best] - centre, full_matrices=False)[2]
        best = np.abs((points - centre) @ axes[2]) < PLANE_TOLERANCE_M
    if axes[2] @ centre > 0:
        axes[2] = -axes[2]
    return best, centre, axes


def _quaternion(rotation):
    """Unit quaternion [x, y, z, w] of a rotation matrix."""
    m = rotation
    candidates = [1 + m[0, 0] - m[1, 1] - m[2, 2], 1 - m[0, 0] + m[1, 1] - m[2, 2],
                  1 - m[0, 0] - m[1, 1] + m[2, 2], 1 + m[0, 0] + m[1, 1] + m[2, 2]]
    k = int(np.argmax(candidates))
    q = [[candidates[0], m[0, 1] + m[1, 0], m[0, 2] + m[2, 0], m[2, 1] - m[1, 2]],
         [m[0, 1] + m[1, 0], candidates[1], m[1, 2] + m[2, 1], m[0, 2] - m[2, 0]],
         [m[0, 2] + m[2, 0], m[1, 2] + m[2, 1], candidates[2], m[1, 0] - m[0, 1]],
         [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], candidates[3]]][k]
    q = np.array(q) / (2 * np.sqrt(candidates[k]))
    return (q if q[3] >= 0 else -q).tolist()


def box_pose(points, size, world_from_camera=None):
    """Pose of a box of known ``size`` [length, width, height] from the points of the one face a camera sees.

    The largest plane in ``points`` is taken as a face, its rectangle found by the rotation of least area, and the
    face named by the pair of box sides its extents are nearest to. The centre lies half the remaining side behind
    the face. Returns None for too few points. A box is the same after a half turn about any of its axes, so the
    rotation is one of four equivalent ones; ``extent_error_m`` says how well the face matched (a face cut by an
    occluder or the image edge reads short).
    """
    if len(points) < MIN_FACE_POINTS:
        return None
    rng = np.random.RandomState(0)
    if len(points) > 4000:
        points = points[rng.choice(len(points), 4000, replace=False)]
    inliers, centre, axes = _plane(points, rng)
    if centre is None or inliers.sum() < MIN_FACE_POINTS:
        return None
    flat = (points[inliers] - centre) @ axes[:2].T
    best = None
    for angle in np.radians(np.arange(0.0, 90.0, 1.0)):
        c, s = np.cos(angle), np.sin(angle)
        turned = flat @ np.array([[c, -s], [s, c]])
        low, high = np.percentile(turned, [1, 99], axis=0)
        if best is None or np.prod(high - low) < best[0]:
            best = (np.prod(high - low), angle, low, high)
    _, angle, low, high = best
    c, s = np.cos(angle), np.sin(angle)
    in_plane = np.array([[c, s], [-s, c]]) @ axes[:2]             # rows: the rectangle's two edge directions
    extent, middle = high - low, (high + low) / 2
    if extent[1] > extent[0]:
        in_plane, extent, middle = in_plane[::-1], extent[::-1], middle[::-1]
    face = min(FACES, key=lambda name: abs(extent[0] - size[FACES[name][0]]) + abs(extent[1] - size[FACES[name][1]]))
    long_axis, short_axis, hidden = FACES[face]
    rotation = np.zeros((3, 3))
    rotation[:, long_axis], rotation[:, short_axis], rotation[:, hidden] = in_plane[0], in_plane[1], axes[2]
    if np.linalg.det(rotation) < 0:
        rotation[:, long_axis] = -rotation[:, long_axis]
    position = centre + middle @ in_plane - axes[2] * size[hidden] / 2
    pose = {"position_m": position.tolist(), "quaternion_xyzw": _quaternion(rotation), "face": face,
            "extent_m": extent.tolist(),
            "extent_error_m": float(abs(extent[0] - size[long_axis]) + abs(extent[1] - size[short_axis])),
            "face_points": int(inliers.sum()), "face_fraction": float(inliers.mean()),
            "plane_rms_m": float(np.sqrt((((points[inliers] - centre) @ axes[2]) ** 2).mean()))}
    if world_from_camera is not None:
        transform = np.asarray(world_from_camera, dtype=np.float64)
        pose["position_world_m"] = (transform[:3, :3] @ position + transform[:3, 3]).tolist()
        pose["quaternion_world_xyzw"] = _quaternion(transform[:3, :3] @ rotation)
    return pose


def in_zone(objects, zone_objects):
    """The ids of a frame's ``objects`` that are in the zone of ``zone_objects`` (another prompt's rows, same frame).

    The zone is the box around the zone objects' boxes, grown on every side by ``ZONE_MARGIN`` of its longer side
    (a tower is slender: half its width would be a few centimetres). An object is in it when its own box overlaps it
    and, where both have a depth point, the two are within ``ZONE_DEPTH_M`` along the optical axis: a hand between
    the camera and the tower, far in front of it, is not at the tower."""
    if not zone_objects:
        return []
    boxes = [o["bbox"] for o in zone_objects]
    left, top = min(b[0] for b in boxes), min(b[1] for b in boxes)
    right, bottom = max(b[2] for b in boxes), max(b[3] for b in boxes)
    grow_x = grow_y = ZONE_MARGIN * max(right - left + 1, bottom - top + 1)
    depths = [o["xyz_m"][2] for o in zone_objects if o.get("xyz_m")]
    found = []
    for o in objects:
        x0, y0, x1, y1 = o["bbox"]
        if x1 < left - grow_x or x0 > right + grow_x or y1 < top - grow_y or y0 > bottom + grow_y:
            continue
        if depths and o.get("xyz_m") and min(abs(o["xyz_m"][2] - depth) for depth in depths) > ZONE_DEPTH_M:
            continue
        found.append(o["id"])
    return found


def mark_zones(out, zones):
    """Write ``in_zone`` into each object row of a zoned prompt's JSON; returns one summary per zone."""
    summary = []
    for tag, zone in zones:
        data, zone_rows = (json.loads((out / f"{name}.json").read_text()) for name in (tag, zone))
        by_frame = {row["frame"]: row["objects"] for row in zone_rows["frames"]}
        present = []
        for row in data["frames"]:
            inside = in_zone(row["objects"], by_frame.get(row["frame"], []))
            for o in row["objects"]:
                o["in_zone"] = o["id"] in inside
            present.append(bool(inside))
        (out / f"{tag}.json").write_text(json.dumps(data))
        frames = [index for index, value in enumerate(present) if value]
        summary.append({"tag": tag, "zone": zone, "frames": len(present), "frames_present": len(frames),
                        "first_present_frame": frames[0] if frames else None, "last_present_frame": frames[-1] if frames else None,
                        "present_in_last_frame": bool(present and present[-1])})
    return summary


def colour(oid):
    """A stable, distinct RGB colour for an object id."""
    return tuple(int(255 * v) for v in colorsys.hsv_to_rgb((oid * 0.618034) % 1.0, 0.75, 1.0))


def overlay(image, labels, alpha=0.55):
    """``image`` [H, W, 3] with each label map's objects tinted and their outlines drawn; later maps over earlier."""
    out = image.astype(np.float32)
    for label in labels:
        edge = np.zeros(label.shape, bool)
        edge[:-1] |= label[:-1] != label[1:]
        edge[:, :-1] |= label[:, :-1] != label[:, 1:]
        for oid in np.unique(label)[1:]:
            mask = label == oid
            tint = np.array(colour(int(oid)), np.float32)
            out[mask] = (1 - alpha) * out[mask] + alpha * tint
            out[mask & edge] = tint
    return out.astype(np.uint8)


def read_depth(path, scale):
    """A depth image in metres: ``.npy`` holds metres, a 16-bit ``.png`` holds counts of ``scale`` metres."""
    if path.suffix == ".npy":
        return np.load(path).astype(np.float32)
    from PIL import Image

    return np.asarray(Image.open(path)).astype(np.float32) * scale


def prepare_frames(request, out):
    """Write the clip as ``<out>/frames/00000.jpg ...`` (what SAM 3's loader reads); returns the files and the rate."""
    frames = out / "frames"
    frames.mkdir()
    source = Path(request["source"])
    if source.is_dir():
        from PIL import Image

        files = sorted(p for p in (source / "rgb").iterdir() if p.suffix.lower() in IMAGES)[::request["every"]]
        for index, file in enumerate(files):
            Image.open(file).convert("RGB").save(frames / f"{index:05d}.jpg", quality=95)
        fps = request["fps"] or (request.get("camera") or {}).get("fps") or 30.0
        return sorted(frames.iterdir()), fps / request["every"]
    ffmpeg = request["ffmpeg"]
    header = subprocess.run([ffmpeg, "-hide_banner", "-i", str(source)], capture_output=True, text=True, timeout=60).stderr
    rate = re.search(r"(\d+(?:\.\d+)?) fps", header)
    fps = request["fps"] or (float(rate[1]) if rate else 30.0)
    window = (["-ss", f"{request['start_s']:.3f}"] if request["start_s"] else []) + (
        ["-to", f"{request['end_s']:.3f}"] if request["end_s"] else [])
    select = ["-vf", f"select=not(mod(n\\,{request['every']}))", "-fps_mode", "vfr"] if request["every"] > 1 else []
    subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-i", str(source), *window, *select, "-q:v", "2",
                    "-start_number", "0", str(frames / "%05d.jpg")], check=True, timeout=1800)
    return sorted(frames.iterdir()), fps / request["every"]


def write_overlay(request, out, files, tags, fps):
    """``overlay.mp4``: every tag's masks over the frames, encoded with the host's ffmpeg."""
    from PIL import Image

    width, height = Image.open(files[0]).size
    encoder = subprocess.Popen(
        [request["ffmpeg"], "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{width}x{height}", "-r", f"{fps:.4f}", "-i", "-", "-an", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", str(out / "overlay.mp4")],
        stdin=subprocess.PIPE)
    for index, file in enumerate(files):
        labels = [np.asarray(Image.open(out / tag / f"labels_{index:04d}.png")) for tag in tags
                  if (out / tag / f"labels_{index:04d}.png").is_file()]
        encoder.stdin.write(overlay(np.asarray(Image.open(file).convert("RGB")), labels).tobytes())
    encoder.stdin.close()
    if encoder.wait(timeout=600):
        raise RuntimeError("ffmpeg could not encode overlay.mp4")


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    out = Path(request["out"])
    for candidate in (os.environ.get("CC"), "/usr/bin/gcc-12", "/usr/bin/gcc-11", "/usr/bin/gcc", shutil.which("gcc")):
        if candidate and os.path.exists(candidate):      # SAM 3's tracker compiles its NMS kernels with $CC
            os.environ["CC"] = candidate
            break
    files, fps = prepare_frames(request, out)
    camera, depth_files = request.get("camera"), []
    if camera:
        depth_files = sorted(p for p in (Path(request["source"]) / "depth").iterdir()
                             if p.suffix in (".png", ".npy"))[::request["every"]]
        if len(depth_files) != len(files):
            raise SystemExit(f"{len(depth_files)} depth images for {len(files)} colour frames; they must pair up.")
    import torch
    from PIL import Image
    from sam3.model_builder import build_sam3_video_predictor

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    predictor = build_sam3_video_predictor(gpus_to_use=[0])
    width, height = Image.open(files[0]).size
    summary = []
    for prompt in request["prompts"]:
        tag, started = prompt["tag"], time.time()
        (out / tag).mkdir()
        session = predictor.handle_request({"type": "start_session", "resource_path": str(out / "frames")})["session_id"]
        predictor.handle_request({"type": "add_prompt", "session_id": session, "frame_index": 0, "text": prompt["text"]})
        remap, rows, poses = {}, [], 0
        for reply in predictor.handle_stream_request({"type": "propagate_in_video", "session_id": session,
                                                      "propagation_direction": "forward", "start_frame_index": 0,
                                                      "max_frame_num_to_track": None}):
            index, found = int(reply["frame_index"]), reply["outputs"]
            ids, masks = np.asarray(found["out_obj_ids"]), np.asarray(found["out_binary_masks"])
            if masks.ndim != 3:                          # a frame with nothing in it
                masks = np.zeros((0, height, width), bool)
            label, objects = frame_objects(ids, masks, np.asarray(found["out_probs"]), remap)
            if camera and len(masks):
                depth = read_depth(depth_files[index], camera.get("depth_scale", 0.001))
                moving = camera.get("frames")
                pose_of_camera = (moving[index * request["every"]] if moving else camera).get("world_from_camera")
                by_id = {remap[int(i)]: mask > 0 for i, mask in zip(ids, masks)}
                for row in objects:
                    points = backproject(depth, by_id[row["id"]], camera)
                    if not len(points):
                        continue
                    row["xyz_m"] = np.median(points, 0).tolist()
                    if prompt.get("size_m"):
                        row["pose"] = box_pose(points, prompt["size_m"], pose_of_camera)
                        poses += row["pose"] is not None
            Image.fromarray(label).save(out / tag / f"labels_{index:04d}.png")
            rows.append({"frame": index, "t": index / fps, "objects": objects})
            if index % 20 == 0:
                print(f"TWINSMITH_PROGRESS: {tag} frame {index + 1}/{len(files)}, {len(remap)} objects", flush=True)
        predictor.handle_request({"type": "close_session", "session_id": session})
        rows.sort(key=lambda row: row["frame"])
        (out / f"{tag}.json").write_text(json.dumps(
            {"clip": request["source"], "fps": fps, "width": width, "height": height, "n_frames": len(files),
             "prompt": prompt, "n_objects": len(remap), "frames": rows}))
        summary.append({"tag": tag, "text": prompt["text"], "frames": len(rows), "objects": len(remap),
                        "objects_first_frame": len(rows[0]["objects"]) if rows else 0,
                        "objects_last_frame": len(rows[-1]["objects"]) if rows else 0, "poses": poses,
                        "seconds": round(time.time() - started, 1), "json": str(out / f"{tag}.json"),
                        "labels": str(out / tag)})
        print(f"TWINSMITH_PROGRESS: {tag} done, {len(rows)} frames, {len(remap)} objects", flush=True)
    write_overlay(request, out, files, [prompt["tag"] for prompt in request["prompts"]], fps)
    (out / "result.json").write_text(json.dumps(
        {"source": request["source"], "frames": len(files), "fps": fps, "depth": bool(camera), "prompts": summary,
         "zones": mark_zones(out, request.get("zones") or []),
         "overlay": str(out / "overlay.mp4"), "frames_dir": str(out / "frames")}, indent=1) + "\n")


if __name__ == "__main__":
    main()
