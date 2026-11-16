#!/usr/bin/env python3
"""Scan to scene: which puzzle pieces lie on the table and where, from one RGB-D look and its ``sam3`` masks.

File in, file out, on the CPU; no simulator, no camera, no model. The steps a caller chains are subcommands:

* ``match --clip C --run R --out estimate.json``: ``C`` is an RGB-D clip folder (``rgb/``, ``depth/``,
  ``camera.json``, the folder the ``sam3`` tool reads) and ``R`` that tool's run folder for it (one label image per
  prompt and frame). Every segment is matched to a piece of the puzzles pack and given a pose.
* ``task --estimate estimate.json --out DIR``: the estimate as a puzzle task file and its BOM, which
  ``workcell__cell(action="quick_start", task=<file>)`` builds.
* ``diff --estimate new.json --previous old.json``: which pieces were added, removed or moved.
* ``score --estimate estimate.json --truth truth.json``: the estimate against known poses.
* ``colours --clip C --truth truth.json --out colours.json``: what colour each piece is to this camera, from a clip
  of a known layout; ``match --colours colours.json`` then uses those in place of the pack's own.
* ``synthetic --task FILE --out DIR``: a clip and perfect label images of a task's layout, drawn by ray casting the
  pack's boxes. For testing the matcher alone.

How a segment becomes a piece. Its depth points are put in the world frame (or, when the clip has no camera pose, in
a frame fitted to the table) and reduced to a height map seen from above. Every piece has a few ways to rest on a
flat surface; each is a height-map template made from the piece's collision boxes. A template is slid and turned over
the segment until the heights agree, and what the template says should be seen but the scan shows empty counts
against it, unless something is in the way. Shape decides the kind of piece; pieces of one shape differ only in
colour, so colour decides among them, with every piece used at most once. Once a base plate is found, each of its
slots is checked directly against the scan, so a piece cut into fragments by the pieces crossing it is still found,
and is given its exact assembled pose.

What it reports besides poses: pieces it did not see, segments it could not explain, and for each piece whether it
lies flat, on its side or on end, what it touches, and whether its identity is uncertain.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
CELL = 0.002                    # height-map cell
TABLE_M = 0.004                 # at or below this height a point is the table
CEILING_M = 0.25                # above this it is not on the table: the gripper the camera rides on, an arm, a hand
CAP_M = 0.010                   # a height disagreement counts up to this much
TOLERANCE_M = 0.004             # heights this close agree
MAX_FRAMES = 5
# A clip with a camera pose is measured in that pose's frame, whose z = 0 is meant to be the table top. A real camera's
# pose and depth put the table a few millimetres off that, and tilted: heights are taken from the table the camera
# sees instead, when it is within these bounds of z = 0. Beyond them the frame is kept as given and the report says so.
TABLE_BIAS_M, TABLE_TILT_DEG, TABLE_POINTS = 0.010, 1.0, 2000
# Costs are 0 (perfect) to 1. A match needs a shape cost under SHAPE_MAX; colour is weighed in to choose among pieces.
SHAPE_MAX, COLOUR_WEIGHT, COLOUR_MAX, MARGIN = 0.30, 0.6, 0.4, 0.08
SLOT_AGREE, SLOT_CELLS = 0.7, 25
# An assembled piece is written this far above its resting pose. In exact contact (a zero gap between parallel box
# faces) Newton's MuJoCo-Warp returned non-finite states for the whole scene at some yaws of the base (132.0 degrees;
# not 131.9, not 0).
DROP_IN_M = 0.0001


def _rotations():
    """The 24 right-angle rotations."""
    found = []
    for a in range(3):
        for b in range(3):
            if a != b:
                for sa in (1, -1):
                    for sb in (1, -1):
                        x, y = np.zeros(3), np.zeros(3)
                        x[a], y[b] = sa, sb
                        found.append(np.column_stack([x, y, np.cross(x, y)]))
    return found


ROTATIONS = _rotations()


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(rpy_deg):
    roll, pitch, yaw = (math.radians(v) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def matrix_rpy(r):
    """Fixed-axis roll, pitch, yaw in degrees of a rotation matrix (the puzzle task's ``rpy_deg``)."""
    if abs(r[2, 0]) > 1 - 1e-9:                                # pitch of 90 degrees: roll and yaw are one turn
        return [0.0, math.degrees(-math.copysign(math.pi / 2, r[2, 0])), math.degrees(math.atan2(-r[0, 1], r[1, 1]))]
    return [math.degrees(v) for v in (math.atan2(r[2, 1], r[2, 2]), -math.asin(r[2, 0]), math.atan2(r[1, 0], r[0, 0]))]


def angle_between(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1))))


def union_volume(boxes):
    """Volume the boxes cover together, counted once where they overlap."""
    low, high = (np.array([centre + sign * half for centre, half in boxes]) for sign in (-1, 1))
    cuts = [np.unique(np.concatenate([low[:, axis], high[:, axis]]).round(7)) for axis in range(3)]
    mids = np.meshgrid(*[(c[:-1] + c[1:]) / 2 for c in cuts], indexing="ij")
    inside = np.zeros(mids[0].shape, dtype=bool)
    for lo, hi in zip(low, high):
        inside |= np.logical_and.reduce([(mids[a] > lo[a]) & (mids[a] < hi[a]) for a in range(3)])
    return float(np.einsum("i,j,k->ijk", *[np.diff(c) for c in cuts])[inside].sum())


def _hull(points):
    """Convex hull of 2-D points, counter-clockwise."""
    points = sorted(set(map(tuple, np.round(points, 6))))
    if len(points) < 3:
        return points
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    halves = []
    for sequence in (points, points[::-1]):
        half = []
        for point in sequence:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return halves[0] + halves[1]


class Rest:
    """One way a shape rests on a flat surface: the rotation that puts it there (before any turn about the vertical),
    how high its origin then is, and the heights of its top surface over a grid, as seen from above."""

    def __init__(self, shape, turn):
        self.turn = turn
        placed = [(turn @ centre, np.abs(turn @ half)) for centre, half in shape.boxes]
        low = np.min([c - h for c, h in placed], axis=0)
        high = np.max([c + h for c, h in placed], axis=0)
        self.drop, self.height, self.origin = float(-low[2]), float(high[2] - low[2]), low[:2]
        cells = np.maximum(1, np.ceil((high[:2] - low[:2]) / CELL - 1e-6).astype(int))
        self.heights = np.full(cells, np.nan)
        for centre, half in placed:
            a = np.clip(np.round((centre[:2] - half[:2] - low[:2]) / CELL).astype(int), 0, cells)
            b = np.clip(np.round((centre[:2] + half[:2] - low[:2]) / CELL).astype(int), 0, cells)
            patch = self.heights[a[0]:b[0], a[1]:b[1]]
            self.heights[a[0]:b[0], a[1]:b[1]] = np.fmax(patch, centre[2] + half[2] - low[2])
        filled = np.argwhere(~np.isnan(self.heights))
        self.area = len(filled) * CELL * CELL
        self.top_area = int((self.heights >= np.nanmax(self.heights) - TOLERANCE_M).sum()) * CELL * CELL      # of its highest level
        self.centroid = low[:2] + (filled.mean(0) + 0.5) * CELL
        self.label = "flat" if self.height < shape.sizes[0] + 1e-4 else "on end" if self.height > shape.sizes[2] - 1e-4 else "on its side"
        foot = [(c, h) for c, h in placed if c[2] - h[2] < low[2] + 1e-4]
        corners = [c[:2] + np.array([sx, sy]) * h[:2] for c, h in foot for sx in (-1, 1) for sy in (-1, 1)]
        hull, com = _hull(corners), (turn @ shape.centroid)[:2]
        self.stable = len(hull) >= 3 and all(
            (b[0] - a[0]) * (com[1] - a[1]) - (b[1] - a[1]) * (com[0] - a[0]) > 0.002 * math.dist(a, b)
            for a, b in zip(hull, hull[1:] + hull[:1]))


class Shape:
    """A solid made of axis-aligned boxes [(centre, half extents)] in its own frame."""

    def __init__(self, boxes):
        self.boxes = [(np.asarray(c, dtype=float), np.asarray(h, dtype=float)) for c, h in boxes]
        volumes = np.array([8 * h.prod() for _, h in self.boxes])
        self.volume = union_volume(self.boxes)
        self.centroid = (np.array([c for c, _ in self.boxes]) * volumes[:, None]).sum(0) / volumes.sum()
        low = np.min([c - h for c, h in self.boxes], axis=0)
        high = np.max([c + h for c, h in self.boxes], axis=0)
        middle, self.middle, self.sizes = (low + high) / 2, (low + high) / 2, sorted(high - low)
        # A rotation S (with the shift that keeps the bounding box in place) is a symmetry when it maps the solid onto itself.
        self.symmetries = []
        for turn in ROTATIONS:
            shift = middle - turn @ middle
            moved = [(turn @ c + shift, np.abs(turn @ h)) for c, h in self.boxes]
            if abs(union_volume(self.boxes + moved) - self.volume) < 1e-4 * self.volume:
                self.symmetries.append((turn, shift))
        rests, seen = [], []
        for turn in ROTATIONS:                                 # one rest per way up, up to symmetry; the first is as modelled
            up = turn[2]
            if not any(np.allclose(up, other @ s) for other in seen for s, _ in self.symmetries):
                seen.append(up)
                rests.append(Rest(self, turn))
        self.rests = [rest for rest in rests if rest.stable]

    def same_as(self, other):
        return abs(self.volume - other.volume) < 1e-4 * self.volume and \
            abs(union_volume(self.boxes + other.boxes) - self.volume) < 1e-4 * self.volume

    def top(self, turn, at, cell=2 * CELL):
        """Points of the top surface, seen from above, of the shape at a pose: (n, 3) in the world."""
        middles = [turn @ centre + at for centre, _ in self.boxes]
        spans = [np.abs(turn[:2] * half).sum(1) for _, half in self.boxes]
        low = np.min([m[:2] - span for m, span in zip(middles, spans)], axis=0)
        high = np.max([m[:2] + span for m, span in zip(middles, spans)], axis=0)
        xs, ys = (np.arange(low[a] + cell / 2, high[a], cell) for a in (0, 1))
        grid = np.stack(np.meshgrid(xs, ys, indexing="ij"), -1).reshape(-1, 2)
        best = np.full(len(grid), -np.inf)
        for middle, (_, half) in zip(middles, self.boxes):
            best = np.maximum(best, _box_top(grid, turn, middle, half))
        keep = np.isfinite(best)
        return np.column_stack([grid[keep], best[keep]])


def _box_top(grid, axes, middle, half):
    """Highest point of a box over each grid point (-inf where the box is not above it): the vertical line through the
    point is clipped against the box's three slabs. ``axes`` holds the box's axes as columns."""
    origin = np.column_stack([grid, np.zeros(len(grid))]) - middle
    low, high = np.full(len(grid), -np.inf), np.full(len(grid), np.inf)
    for axis in range(3):
        along, start = axes[2, axis], origin @ axes[:, axis]
        if abs(along) < 1e-9:
            outside = np.abs(start) > half[axis]
            low, high = np.where(outside, np.inf, low), np.where(outside, -np.inf, high)
        else:
            a, b = (-half[axis] - start) / along, (half[axis] - start) / along
            low, high = np.maximum(low, np.minimum(a, b)), np.minimum(high, np.maximum(a, b))
    return np.where(high >= low, high, -np.inf)


class Pack:
    """The pieces a scan can hold: ``pieces`` is {name: {"boxes": [(centre, half)], "colour": linear RGB}} and
    ``assemblies`` {base name: [{"piece", "position_m", "rpy_deg"}]}. Pieces of one shape share a ``Shape``."""

    def __init__(self, pieces, assemblies=None, name="pack", colours=None):
        """``colours`` {piece: linear RGB} replaces the pack's own for the pieces it names: what a camera actually
        sees, measured with ``measure_colours``."""
        self.name, self.assemblies, self.shapes, self.shape_of, self.colour = name, assemblies or {}, [], {}, {}
        for piece, record in pieces.items():
            shape = Shape(record["boxes"])
            index = next((i for i, other in enumerate(self.shapes) if other.same_as(shape)), None)
            if index is None:
                index = len(self.shapes)
                self.shapes.append(shape)
            self.shape_of[piece], self.colour[piece] = index, np.asarray((colours or {}).get(piece, record["colour"]), dtype=float)
        self.pieces = list(pieces)

    def shape(self, piece):
        return self.shapes[self.shape_of[piece]]

    def alike(self, piece):
        return [other for other in self.pieces if self.shape_of[other] == self.shape_of[piece]]


def load_pack(pack="puzzles/v1", colours=None):
    """A pack version of the asset root, verified, as a ``Pack``: every USD asset's box colliders and colour.
    ``colours`` is a JSON file of measured colours, {piece: linear RGB}."""
    sys.path.insert(0, str(REPO / "mcp"))
    from pxr import Usd, UsdGeom, UsdShade
    from twinsmith import pack_manifest

    pack_id, version = pack.split("/")
    root = pack_manifest.resolve_asset_root().path
    pack_manifest.AssetPack(root, pack_id, version)            # every hash, or it raises
    folder = root / pack_id / version
    manifest = pack_manifest.read_manifest(folder, pack_id, version)
    pieces = {}
    for record in manifest["files"]:
        stage = Usd.Stage.Open(str(folder / record["path"]))
        boxes = [(np.array(prim.GetAttribute("xformOp:translate").Get()), np.array(prim.GetAttribute("xformOp:scale").Get()))
                 for prim in stage.Traverse() if prim.IsA(UsdGeom.Cube)]
        colour = next(UsdShade.Shader(prim).GetInput("diffuseColor").Get() for prim in stage.Traverse() if prim.IsA(UsdShade.Shader))
        pieces[record["asset_id"]] = {"boxes": boxes, "colour": list(colour)}
    return Pack(pieces, manifest.get("assemblies"), pack, json.loads(Path(colours).read_text())["colours"] if colours else None)


# ---- the scan ---------------------------------------------------------------------------------------------

def _linear(srgb):
    c = np.asarray(srgb, dtype=float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _srgb(linear):
    c = np.clip(np.asarray(linear, dtype=float), 0, 1)
    return np.round(255 * np.where(c <= 0.0031308, 12.92 * c, 1.055 * c ** (1 / 2.4) - 0.055)).astype(np.uint8)


def _table_frame(points):
    """world_from_camera for a clip without one: the largest plane is the table (z = 0, z toward the camera), the
    origin is under the camera and x is the camera's x laid on the table."""
    rng, best = np.random.default_rng(0), (0, None)
    sample = points[rng.choice(len(points), min(len(points), 20000), replace=False)]
    for _ in range(200):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        if np.linalg.norm(normal) < 1e-9:
            continue
        normal = normal / np.linalg.norm(normal)
        count = int((np.abs((sample - a) @ normal) < 0.003).sum())
        if count > best[0]:
            best = (count, (normal, a))
    normal, point = best[1]
    near = sample[np.abs((sample - point) @ normal) < 0.003]
    middle = near.mean(0)
    normal = np.linalg.svd(near - middle)[2][2]
    normal = -normal if normal @ middle > 0 else normal        # toward the camera, which is at the origin
    x = np.array([1.0, 0, 0]) - normal[0] * normal
    x = x / np.linalg.norm(x)
    pose = np.eye(4)
    pose[:3, :3], pose[2, 3] = np.array([x, np.cross(normal, x), normal]), -float(normal @ middle)
    return pose


def _table_plane(points):
    """The table top among points that are in no mask: ``(a, b, c)`` of z = a x + b y + c fitted to the points within
    3 mm of it, starting level at their median height, and how many points that is. None when too few are near z = 0."""
    near = points[np.abs(points[:, 2]) < 2 * TABLE_BIAS_M]
    if len(near) < TABLE_POINTS:
        return None
    if len(near) > 60000:
        near = near[np.random.default_rng(0).choice(len(near), 60000, replace=False)]
    plane, design = np.array([0.0, 0.0, float(np.median(near[:, 2]))]), np.column_stack([near[:, :2], np.ones(len(near))])
    for _ in range(5):
        close = np.abs(near[:, 2] - design @ plane) < 0.003
        if close.sum() < TABLE_POINTS:
            return None
        plane = np.linalg.lstsq(design[close], near[close, 2], rcond=None)[0]
    return plane, int(close.sum())


class Scan:
    """What one look holds: ``objects`` (each a segment's top surface on the height grid, its colour and its name
    ``tag:id``), the grid itself (``height``, ``colour`` and ``owner`` per cell) and the cameras that looked."""

    def __init__(self, clip, run):
        from PIL import Image
        from scipy import ndimage

        clip, run = Path(clip), Path(run)
        camera = json.loads((clip / "camera.json").read_text())
        colour = sorted(p for p in (clip / "rgb").iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
        depth = sorted((clip / "depth").iterdir())
        tags = sorted(p.name for p in run.iterdir() if p.is_dir() and any(p.glob("labels_*.png")))
        if not colour or len(depth) != len(colour) or not tags:
            raise SystemExit(f"{clip} needs rgb/ and depth/ of equal length, and {run} a label folder per prompt")
        picked = sorted(set(np.linspace(0, len(colour) - 1, min(MAX_FRAMES, len(colour))).round().astype(int).tolist()))
        scale = camera.get("depth_scale", 0.001)
        self.frame, self.cameras, gathered, levels = "world", [], {}, {}
        fixed = None
        for index in picked:
            image = np.asarray(Image.open(colour[index]).convert("RGB"))
            metres = (np.load(depth[index]) if depth[index].suffix == ".npy" else np.asarray(Image.open(depth[index]), dtype=np.float32) * scale)
            height, width = metres.shape
            v, u = np.mgrid[0:height, 0:width]
            rays = np.stack([(u - camera["cx"]) / camera["fx"], (v - camera["cy"]) / camera["fy"], np.ones_like(metres)], -1)
            points, valid = rays * metres[..., None], metres > 0
            pose = camera["frames"][index].get("world_from_camera") if "frames" in camera else camera.get("world_from_camera")
            if pose is None:
                fixed = _table_frame(points[valid]) if fixed is None else fixed
                pose, self.frame = fixed, "table"
            pose = np.array(pose, dtype=float)
            world = points @ pose[:3, :3].T + pose[:3, 3]
            blind = ~valid | (world[..., 2] > CEILING_M)       # no depth, or something between the camera and the table
            valid = valid & ~blind
            self.cameras.append({"pose": pose, "fx": camera["fx"], "fy": camera["fy"], "cx": camera["cx"], "cy": camera["cy"],
                                 "width": width, "height": height, "blind": ndimage.binary_dilation(blind, iterations=3)})
            masks = []
            for tag in tags:
                files = sorted((run / tag).glob("labels_*.png"))
                labels = np.asarray(Image.open(files[min(index, len(files) - 1)]))
                masks += [(f"{tag}:{i}", labels == i) for i in np.unique(labels) if i]
            if self.frame == "world":                                    # heights from the table the camera sees
                covered = np.zeros(metres.shape, dtype=bool)
                for _, mask in masks:
                    covered |= mask
                found, reference = _table_plane(world[valid & ~covered]), {"applied": False}
                if found is None:
                    reference["why"] = f"fewer than {TABLE_POINTS} points of bare table within {2 * TABLE_BIAS_M * 1000:g} mm of the frame's z = 0"
                else:
                    (a, b, c), count = found
                    under = world[valid][:, :2].mean(0)                  # the middle of what the camera sees
                    reference.update(height_mm=round(float(a * under[0] + b * under[1] + c) * 1000, 2),
                                     tilt_deg=round(math.degrees(math.atan(math.hypot(a, b))), 3), points=count)
                    if abs(reference["height_mm"]) > TABLE_BIAS_M * 1000 or reference["tilt_deg"] > TABLE_TILT_DEG:
                        reference["why"] = (f"the table seen is more than {TABLE_BIAS_M * 1000:g} mm or {TABLE_TILT_DEG:g} degrees from the "
                                            "frame's z = 0: the camera pose or the depth is off by more than a bias")
                    else:
                        reference["applied"] = True
                        world = world - np.stack([np.zeros_like(metres), np.zeros_like(metres), a * world[..., 0] + b * world[..., 1] + c], -1)
                if index == picked[0]:
                    self.reference = reference
            step, level = np.zeros(metres.shape), ndimage.median_filter(world[..., 2], size=3)
            for axis in (0, 1):                                         # the height change to the neighbouring pixels
                change = np.abs(np.diff(level, axis=axis))
                pad = [(0, 0), (0, 0)]
                for pad[axis] in ((0, 1), (1, 0)):
                    step = np.maximum(step, np.pad(change, pad))
            # A wall seen at an angle is a ramp a few pixels wide: steeper than about 60 degrees is a step, not a surface.
            surface = step < np.maximum(0.0015, 1.7 * metres / camera["fx"])
            taken = np.zeros(metres.shape, dtype=bool)
            for name, mask in sorted(masks, key=lambda item: item[1].sum()):     # a pixel goes to the smallest mask over it
                own = mask & ~taken & valid & (world[..., 2] > TABLE_M)
                if own.sum() < 0.25 * (mask & valid & (world[..., 2] > TABLE_M)).sum() or own.sum() < 30:
                    continue                                                    # another mask of the same thing, or the table
                taken |= own
                gathered.setdefault(name, []).append((world[own], image[own]))
                if name not in levels:                                           # its level surfaces, should the whole fit nothing
                    parts, count = ndimage.label(ndimage.binary_opening(own & surface, iterations=1))
                    sizes = np.bincount(parts.ravel())[1:]
                    levels[name] = [(world[parts == part], image[parts == part]) for part in np.flatnonzero(sizes >= 30) + 1]
            rest = valid & ~taken & (world[..., 2] > 1.5 * TABLE_M)                # raised and in no mask: what SAM 3 left out
            if index == picked[0] and rest.any():
                blobs, count = ndimage.label(ndimage.binary_opening(rest & surface, iterations=1))
                for blob in range(1, count + 1):
                    own = blobs == blob
                    if own.sum() * (metres[own].mean() / camera["fx"]) ** 2 > 2e-4:   # more than 2 cm2 of surface
                        gathered[f"unsegmented:{blob}"] = [(world[own], image[own])]
            if index == picked[0]:
                flat = world[valid & ~taken][:, 2]
                flat = flat[np.abs(flat) < 0.02]
                self.table = {"blind_percent": round(100 * float(self.cameras[-1]["blind"].mean()), 1), "points": int(len(flat)), "median_height_mm": round(float(np.median(flat)) * 1000, 2) if len(flat) else None,
                              "spread_mm": round(float(np.percentile(flat, 84) - np.percentile(flat, 16)) * 500, 2) if len(flat) else None,
                              **({"reference": self.reference} if self.frame == "world" else {})}
        everything = np.concatenate([points for parts in gathered.values() for points, _ in parts]) if gathered else np.zeros((1, 3))
        self.origin = everything[:, :2].min(0) - 0.05
        shape = np.ceil((everything[:, :2].max(0) + 0.05 - self.origin) / CELL).astype(int)
        self.height, self.owner, self.colour, self.objects = np.full(shape, np.nan), np.full(shape, -1), np.zeros((*shape, 3)), []
        first = np.full(shape, -1)                              # which object took each cell first

        def surface_of(name, parts):
            """An object's top surface on the grid: its cells, their centres and heights, its area and colour."""
            points = np.concatenate([p for p, _ in parts])
            pixels = _linear(np.concatenate([c for _, c in parts]))
            cells = np.floor((points[:, :2] - self.origin) / CELL).astype(int)
            order = np.argsort(points[:, 2])                 # the highest point of a cell is written last
            top, tint = np.full(shape, -np.inf), np.zeros((*shape, 3))
            top[cells[order, 0], cells[order, 1]] = points[order, 2]
            tint[cells[order, 0], cells[order, 1]] = pixels[order]
            filled = np.argwhere(np.isfinite(top))
            if len(filled) < 20:
                return None, tint
            z = top[filled[:, 0], filled[:, 1]]
            return {"name": name, "cells": filled, "xy": self.origin + (filled + 0.5) * CELL, "z": z, "area": len(filled) * CELL * CELL,
                    "top_area": int((z >= np.percentile(z, 90) - TOLERANCE_M).sum()) * CELL * CELL,
                    # Of its upper surface: side faces, which a mask also covers, are lit differently.
                    "colour": np.median(tint[filled[:, 0], filled[:, 1]][z >= np.percentile(z, 90) - 1.5 * TOLERANCE_M], axis=0),
                    "centroid": (self.origin + (filled + 0.5) * CELL).mean(0)}, tint

        for name, parts in gathered.items():
            item, tint = surface_of(name, parts)
            if item is None:
                continue
            filled, z = item["cells"], item["z"]
            under = first[filled[:, 0], filled[:, 1]]
            shared = np.bincount(under[under >= 0], minlength=len(self.objects) + 1)
            if any(count > 0.5 * max(len(filled), len(self.objects[other]["z"])) for other, count in enumerate(shared[:len(self.objects)])):
                continue                                       # the same thing again: another prompt's mask, or another frame's
            index = len(self.objects)
            first[filled[:, 0], filled[:, 1]] = np.where(under >= 0, under, index)
            higher = ~(self.height[filled[:, 0], filled[:, 1]] >= z)
            self.height[filled[higher, 0], filled[higher, 1]] = z[higher]
            self.owner[filled[higher, 0], filled[higher, 1]] = index
            self.colour[filled[higher, 0], filled[higher, 1]] = tint[filled[higher, 0], filled[higher, 1]]
            # A mask that took in two touching pieces at different heights: its level surfaces, tried if the whole fits nothing.
            split = [surface_of(f"{name}.{number}", [part])[0] for number, part in enumerate(levels.get(name, []), start=1)]
            item["levels"] = [part for part in split if part] if len(split) > 1 else []
            self.objects.append(item)

    def cell(self, xy):
        index = np.floor((xy - self.origin) / CELL).astype(int)
        inside = ((index >= 0) & (index < self.height.shape)).all(1)
        return np.where(inside[:, None], index, 0), inside

    def evidence(self, top):
        """For points a pose says are top surface: 1 where the scan agrees, -1 where it shows something lower though it
        could see there, 0 where it could not tell (hidden, covered or out of view); and the object and the colour under each."""
        index, inside = self.cell(top[:, :2])
        seen = np.where(inside, self.height[index[:, 0], index[:, 1]], np.nan)
        owner = np.where(inside, self.owner[index[:, 0], index[:, 1]], -1)
        verdict = np.zeros(len(top), dtype=int)
        verdict[np.abs(seen - top[:, 2]) <= TOLERANCE_M] = 1
        lower = ~(seen >= top[:, 2] - TOLERANCE_M)                        # nothing there, or something lower
        visible = np.zeros(len(top), dtype=bool)
        ceiling = float(np.nanmax(self.height)) if np.isfinite(self.height).any() else 0.0
        for camera in self.cameras:
            eye = camera["pose"][:3, 3]
            local = (top - eye) @ camera["pose"][:3, :3]
            u = camera["fx"] * local[:, 0] / local[:, 2] + camera["cx"]
            v = camera["fy"] * local[:, 1] / local[:, 2] + camera["cy"]
            clear = (local[:, 2] > 0) & (u > 2) & (u < camera["width"] - 2) & (v > 2) & (v < camera["height"] - 2)
            clear &= ~camera["blind"][np.clip(v, 0, camera["height"] - 1).astype(int), np.clip(u, 0, camera["width"] - 1).astype(int)]
            reach = np.clip((ceiling + 0.002 - top[:, 2]) / np.maximum(eye[2] - top[:, 2], 1e-6), 0, 1)
            for step in np.linspace(0.08, 1.0, 12):                       # along the line of sight, up to the tallest thing
                at = top + (eye - top) * (reach * step)[:, None]
                spot, on_grid = self.cell(at[:, :2])
                clear &= ~(on_grid & (self.height[spot[:, 0], spot[:, 1]] > at[:, 2] + 0.003))
            visible |= clear
        verdict[lower & visible & (verdict == 0)] = -1
        return verdict, owner, np.where(inside[:, None], self.colour[index[:, 0], index[:, 1]], 0.0)


# ---- matching ---------------------------------------------------------------------------------------------

def _fit_cost(xy, z, rest, yaw, at):
    c, s = math.cos(yaw), math.sin(yaw)
    index = np.floor(((xy - at) @ np.array([[c, -s], [s, c]]) - rest.origin) / CELL).astype(int)
    inside = ((index >= 0) & (index < rest.heights.shape)).all(1)
    h = np.where(inside, rest.heights[np.where(inside, index[:, 0], 0), np.where(inside, index[:, 1], 0)], np.nan)
    inside = ~np.isnan(h)
    if inside.sum() < 10:
        return 1.0, 0.0
    lift = float(np.median((z - h)[inside]))
    counted = ~(inside & (z - h - lift > 2 * TOLERANCE_M))         # what stands higher is something lying on it
    if counted.sum() < 0.3 * len(z):
        return 1.0, lift
    error = np.where(inside, np.minimum(np.abs(z - h - lift), CAP_M), CAP_M)[counted]
    return float(error.mean() / CAP_M) + (1.0 if lift < -TABLE_M else 0.0), lift


def fit(scan, item, shape):
    """The best pose of ``shape`` for one object in each way the shape can rest, cheapest first:
    [{"cost", "turn", "at", "rest", "agree", "lift"}]."""
    found = []
    for rest in shape.rests:
        best = None
        # Too big to be this shape lying this way: by its whole footprint and by its upper surface alone. A camera that
        # looks at an angle also sees the piece's walls, and a depth camera draws a skirt of mixed pixels down every
        # edge; on a thin piece those add half again to the footprint, and nothing to the upper surface.
        if (item["area"] > 1.3 * rest.area + 2e-4 and item.get("top_area", item["area"]) > 1.3 * rest.top_area + 2e-4) \
                or item["area"] < 0.12 * rest.area:
            continue
        rng = np.random.default_rng(0)
        some, most = (rng.choice(len(item["z"]), min(len(item["z"]), count), replace=False) for count in (700, 2500))
        xy, z, all_xy, all_z = item["xy"][some], item["z"][some], item["xy"][most], item["z"][most]
        starts = []
        for yaw in np.radians(np.arange(0, 360, 15)):
            at = item["centroid"] - yaw_matrix(yaw)[:2, :2] @ rest.centroid
            starts.append((_fit_cost(xy, z, rest, yaw, at)[0], yaw, at))
        for _, yaw, at in sorted(starts, key=lambda start: start[0])[:3]:
            cost = _fit_cost(all_xy, all_z, rest, yaw, at)[0]
            turn_step, move_step = math.radians(6), 0.004
            while turn_step > math.radians(0.1):
                moved = False
                for d_yaw, d_at in ((turn_step, (0, 0)), (-turn_step, (0, 0)), (0, (move_step, 0)), (0, (-move_step, 0)),
                                    (0, (0, move_step)), (0, (0, -move_step))):
                    trial = _fit_cost(all_xy, all_z, rest, yaw + d_yaw, at + d_at)[0]
                    if trial < cost - 1e-6:
                        cost, yaw, at, moved = trial, yaw + d_yaw, at + d_at, True
                if not moved:
                    turn_step, move_step = turn_step / 2, move_step / 2
            lift = _fit_cost(all_xy, all_z, rest, yaw, at)[1]
            lift = 0.0 if abs(lift) <= TABLE_M else lift               # on the table: exactly on it
            turn = yaw_matrix(yaw) @ rest.turn
            position = np.array([at[0], at[1], lift + rest.drop])
            verdict, _, tint = scan.evidence(shape.top(turn, position))
            agree, against = int((verdict == 1).sum()), int((verdict == -1).sum())
            total = 0.5 * cost + 0.5 * against / max(1, agree + against)
            if best is None or total < best["cost"]:
                best = {"cost": total, "turn": turn, "at": position, "rest": rest, "lift": lift, "agree": agree / max(1, len(verdict)),
                        # Of the surface this pose says is the piece's own, whatever else the object's mask took in.
                        "colour": np.median(tint[verdict == 1], axis=0) if agree >= 20 else item["colour"]}
        if best:
            found.append(best)
    return sorted(found, key=lambda one: one["cost"])


def _overlap(pack, a, b):
    """How deep two posed pieces are inside each other, in metres (0 when apart): over their boxes, the smallest
    overlap along the separating axes of each pair."""
    def placed(piece):
        turn, at = piece["pose"][:3, :3], piece["pose"][:3, 3]
        return [(turn @ centre + at, np.diag(half) @ turn.T) for centre, half in pack.shape(piece["name"]).boxes]

    worst = 0.0
    for ca, ha in placed(a):
        for cb, hb in placed(b):
            axes = [h / np.linalg.norm(h) for h in (*ha, *hb)]
            axes += [c / np.linalg.norm(c) for u in axes[:3] for v in axes[3:] for c in [np.cross(u, v)] if np.linalg.norm(c) > 1e-6]
            depth = min(np.abs(ha @ axis).sum() + np.abs(hb @ axis).sum() - abs((cb - ca) @ axis) for axis in axes)
            worst = max(worst, depth)
    return worst


def _hue(colour):
    """Hue in degrees of a linear RGB colour, or None for a grey."""
    high, low = float(np.max(colour)), float(np.min(colour))
    if high - low < 0.02 or high - low < 0.15 * high:
        return None
    r, g, b = (float(c) for c in colour)
    if high == r:
        return 60 * ((g - b) / (high - low)) % 360
    return 60 * ((b - r) / (high - low)) + 120 if high == g else 60 * ((r - g) / (high - low)) + 240


def _colour_cost(seen, wanted, gain):
    """0 to 1: how unlike two colours are. Mostly hue, which light and shadow leave alone; partly lightness, corrected
    by the scan's overall brightness (``gain``, a log ratio), which is all that tells maroon from red."""
    a, b = _hue(seen), _hue(wanted)
    turn = 0.5 if a is None or b is None else min(1.0, min(abs(a - b), 360 - abs(a - b)) / 30.0)
    shade = abs(math.log(max(float(np.mean(seen)), 1e-4) / max(float(np.mean(wanted)), 1e-4)) - gain)
    return 0.7 * turn + 0.3 * min(1.0, shade / math.log(3))


def _gain(scan, pack):
    """How much brighter than the pack's colours the scan is, as a log ratio: the median over objects, each against the
    pack colour nearest in hue."""
    ratios = []
    for item in scan.objects:
        nearest = min(pack.pieces, key=lambda piece: _colour_cost(item["colour"], pack.colour[piece], 0.0))
        ratios.append(math.log(max(float(item["colour"].mean()), 1e-4) / float(pack.colour[nearest].mean())))
    return float(np.median(ratios)) if ratios else 0.0


def _pose(turn, at):
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = turn, at
    return pose


def _slots(scan, pack, base, pose, gain, free):
    """The pieces standing in a base's slots at ``pose``: every slot whose template the scan agrees with, each given
    the free piece of that shape whose colour fits best (cheapest pairings first), and the objects it accounts for."""
    seen, pairs = [], []
    for number, slot in enumerate(pack.assemblies.get(base, [])):
        shape = pack.shape(slot["piece"])
        placed = pose @ _pose(rpy_matrix(slot["rpy_deg"]), slot["position_m"])
        top = shape.top(placed[:3, :3], placed[:3, 3])
        verdict, owner, tint = scan.evidence(top)
        agree, against = int((verdict == 1).sum()), int((verdict == -1).sum())
        owners = owner[(verdict == 1) & (owner >= 0)]
        if agree < SLOT_CELLS or agree < SLOT_AGREE * (agree + against) or not len(owners):
            seen.append(None)
            continue
        counts = np.bincount(owners)
        colour = np.median(tint[verdict == 1], axis=0)       # of the surface itself, whatever mask it fell in
        ranked = sorted((_colour_cost(colour, pack.colour[piece], gain), piece) for piece in pack.alike(slot["piece"]) if piece in free)
        seen.append({"slot": slot, "pose": placed, "agree": agree / len(top), "cost": against / (agree + against), "ranked": ranked,
                     "cells": {int(i): int(n) for i, n in enumerate(counts) if n}})
        pairs += [(cost, number, piece) for cost, piece in ranked if cost <= COLOUR_MAX]
    found, taken = [], set()
    for cost, number, piece in sorted(pairs):
        if seen[number] is not None and "piece" not in seen[number] and piece not in taken:
            taken.add(piece)
            others = [(c, other) for c, other in seen[number]["ranked"] if other != piece]
            seen[number].update(piece=piece, colour_cost=cost, runner_up=others[0][1] if others and others[0][0] - cost < MARGIN else None)
            found.append(seen[number])
    return found


def match(scan, pack):
    """Every object of the scan against the pack. Returns the estimate (plain data)."""
    from scipy import ndimage
    from scipy.optimize import linear_sum_assignment
    from scipy.spatial import cKDTree

    gain = _gain(scan, pack)
    every = {(o, s): fit(scan, item, shape) for o, item in enumerate(scan.objects) for s, shape in enumerate(pack.shapes)}
    fits = {key: found[0] if found else None for key, found in every.items()}
    colours = {(o, piece): _colour_cost((fits[o, pack.shape_of[piece]] or item)["colour"], pack.colour[piece], gain)
               for o, item in enumerate(scan.objects) for piece in pack.pieces}
    free, used, pieces = set(pack.pieces), set(), []

    def record(piece, pose, **more):
        free.discard(piece)
        pieces.append({"name": piece, "pose": pose, **more})

    # 1. Base plates first, each the way up and with the turn about its centre that its slots' contents agree with best.
    bases = [piece for piece in pack.pieces if piece in pack.assemblies]
    candidates = sorted((fits[o, pack.shape_of[base]]["cost"] + COLOUR_WEIGHT * colours[o, base], o, base)
                        for o in range(len(scan.objects)) for base in bases if fits[o, pack.shape_of[base]])
    for _, o, base in candidates:
        found = fits[o, pack.shape_of[base]]
        if o in used or base not in free or found["cost"] > SHAPE_MAX or colours[o, base] > COLOUR_MAX:
            continue
        shape, options = pack.shape(base), []
        for way, found in enumerate(one for one in every[o, pack.shape_of[base]] if one["cost"] <= SHAPE_MAX):
            for quarter in range(4):                           # about the vertical, in place
                spin = yaw_matrix(quarter * math.pi / 2) @ found["turn"]
                pose = _pose(spin, found["at"] + found["turn"] @ shape.middle - spin @ shape.middle)
                verdict = scan.evidence(shape.top(pose[:3, :3], pose[:3, 3]))[0]
                against = (verdict == -1).sum() / max(1, (verdict != 0).sum())
                slots = _slots(scan, pack, base, pose, gain, free - {base}) if found["rest"] is shape.rests[0] else []
                options.append((-len(slots), round(found["cost"] + against, 3), way, quarter, pose, slots, found))
        *_, pose, slots, found = min(options, key=lambda option: option[:4])
        used.add(o)
        record(base, pose, object=scan.objects[o]["name"], rest=found["rest"].label, support="table" if found["lift"] == 0.0 else "raised",
               shape_cost=found["cost"], colour_cost=colours[o, base], seen=found["agree"], lift=found["lift"],
               upside_down=found["rest"] is not shape.rests[0] and found["rest"].label == "flat")
        for slot in slots:
            claimed = [i for i, cells in slot["cells"].items() if i not in used and cells > 0.5 * len(scan.objects[i]["z"]) / 4]
            used.update(claimed)
            record(slot["piece"], slot["pose"], object=", ".join(scan.objects[i]["name"] for i in claimed) or None, rest="assembled",
                   support=base, parent=base, slot=slot["slot"], shape_cost=slot["cost"], colour_cost=slot["colour_cost"],
                   seen=slot["agree"], lift=0.0, runner_up=slot["runner_up"])

    # 2. Everything else: one piece per object, the cheapest pairing overall.
    rows, columns = [o for o in range(len(scan.objects)) if o not in used], sorted(free)
    table = np.full((len(rows), len(columns)), 9.0)
    for i, o in enumerate(rows):
        for j, piece in enumerate(columns):
            found = fits[o, pack.shape_of[piece]]
            if found and found["cost"] <= SHAPE_MAX and colours[o, piece] <= COLOUR_MAX:
                table[i, j] = found["cost"] + COLOUR_WEIGHT * colours[o, piece]
    for i, j in zip(*linear_sum_assignment(table)) if table.size else ():
        if table[i, j] >= 9.0:
            continue
        o, piece = rows[i], columns[j]
        found = fits[o, pack.shape_of[piece]]
        others = sorted((table[i, k], columns[k]) for k in range(len(columns)) if k != j and table[i, k] < 9.0)
        stances = [one["rest"].label for one in every[o, pack.shape_of[piece]][1:] if one["cost"] - found["cost"] < MARGIN]
        used.add(o)
        record(piece, _pose(found["turn"], found["at"]), object=scan.objects[o]["name"], rest=found["rest"].label,
               support="table" if found["lift"] == 0.0 else "raised", shape_cost=found["cost"], colour_cost=colours[o, piece],
               seen=found["agree"], lift=found["lift"], other_stance=stances[0] if stances else None,
               runner_up=others[0][1] if others and others[0][0] - table[i, j] < MARGIN else None)

    # Fragments: objects still unmatched that are of one colour and within 3 cm of each other are tried as one object
    # (a piece SAM 3 cut in two, or one with the gripper across it).
    left = [o for o in range(len(scan.objects)) if o not in used]
    groups = []
    for o in left:
        near = [group for group in groups if any(
            _colour_cost(scan.objects[o]["colour"], scan.objects[other]["colour"], 0.0) < 0.15
            and np.abs(scan.objects[o]["cells"][:, None, :] - scan.objects[other]["cells"][None, ::7, :]).max(2).min() * CELL < 0.03
            for other in group)]
        merged = [o] + [other for group in near for other in group]
        groups = [group for group in groups if group not in near] + [merged]
    for group in (group for group in groups if len(group) > 1):
        parts = [scan.objects[o] for o in group]
        whole = {"name": " + ".join(part["name"] for part in parts), "cells": np.concatenate([part["cells"] for part in parts]),
                 "xy": np.concatenate([part["xy"] for part in parts]), "z": np.concatenate([part["z"] for part in parts]),
                 "area": sum(part["area"] for part in parts), "colour": np.median([part["colour"] for part in parts], axis=0)}
        whole["centroid"] = whole["xy"].mean(0)
        options = []
        for piece in sorted(free):
            found = fit(scan, whole, pack.shape(piece))
            cost = _colour_cost(found[0]["colour"] if found else whole["colour"], pack.colour[piece], gain)
            if found and found[0]["cost"] <= SHAPE_MAX and cost <= COLOUR_MAX:
                options.append((found[0]["cost"] + COLOUR_WEIGHT * cost, piece, found[0], cost))
        if options:
            _, piece, found, cost = min(options, key=lambda option: option[0])
            used.update(group)
            record(piece, _pose(found["turn"], found["at"]), object=whole["name"], rest=found["rest"].label,
                   support="table" if found["lift"] == 0.0 else "raised", shape_cost=found["cost"], colour_cost=cost,
                   seen=found["agree"], lift=found["lift"], in_fragments=len(group))

    # A mask that fits nothing whole may hold two touching pieces at different heights: its level surfaces, one by one.
    for o in [o for o in range(len(scan.objects)) if o not in used and scan.objects[o]["levels"]]:
        options = []
        for part in scan.objects[o]["levels"]:
            for piece in sorted(free):
                found = fit(scan, part, pack.shape(piece))
                if found and found[0]["cost"] <= SHAPE_MAX:
                    cost = _colour_cost(found[0]["colour"], pack.colour[piece], gain)
                    if cost <= COLOUR_MAX:
                        options.append((found[0]["cost"] + COLOUR_WEIGHT * cost, part["name"], piece, found[0], cost))
        taken = set()
        for _, part, piece, found, cost in sorted(options, key=lambda option: option[0]):
            if part not in taken and piece in free:
                taken.add(part)
                used.add(o)
                record(piece, _pose(found["turn"], found["at"]), object=part, rest=found["rest"].label,
                       support="table" if found["lift"] == 0.0 else "raised", shape_cost=found["cost"], colour_cost=cost,
                       seen=found["agree"], lift=found["lift"], split_from=scan.objects[o]["name"])

    # Two pieces cannot be in one place: of a pair more than 3 mm inside each other, the worse fit is withdrawn.
    unexplained = []
    for a in sorted(pieces, key=lambda piece: (piece["rest"] != "assembled" and piece["name"] not in pack.assemblies, piece["shape_cost"])):
        if a not in pieces:
            continue
        for b in [piece for piece in pieces if piece is not a]:
            depth = _overlap(pack, a, b)
            if depth > 0.003:
                pieces.remove(b)
                free.add(b["name"])
                unexplained.append({"object": b["object"], "why": f"it fits {b['name']}, but there it would be {depth * 1000:.0f} mm inside {a['name']}",
                                    "centre_m": [round(float(v), 4) for v in b["pose"][:2, 3]], "nearest_shape": pack.alike(b["name"])})

    # 3. What is left: objects a placed piece already accounts for, and objects nothing accounts for.
    for o, item in enumerate(scan.objects):
        if o in used:
            continue
        points = np.column_stack([item["xy"], item["z"]])
        owner = None
        for piece in pieces:
            top = pack.shape(piece["name"]).top(piece["pose"][:3, :3], piece["pose"][:3, 3], cell=CELL)
            if not len(top):
                continue
            near, _ = cKDTree(top).query(points, distance_upper_bound=2 * TOLERANCE_M)
            if np.isfinite(near).mean() > 0.6:
                owner = piece["name"]
                break
        if owner:
            continue
        best = min(((fits[o, s]["cost"], s) for s in range(len(pack.shapes)) if fits[o, s]), default=None)
        if best is None:
            why = "no piece is this size: several pieces touching, a fragment, or not a puzzle piece"
        elif best[0] > SHAPE_MAX:
            why = "no piece has this shape lying on a flat surface: tilted, leaning on or touching another piece"
        else:
            why = "its shape fits a piece, but no piece of that shape and this colour is left"
        unexplained.append({"object": item["name"], "area_cm2": round(item["area"] * 1e4, 1), "top_mm": round(float(item["z"].max()) * 1000, 1),
                            "centre_m": [round(float(v), 4) for v in item["centroid"]], "why": why,
                            "nearest_shape": None if best is None else [piece for piece in pack.pieces if pack.shape_of[piece] == best[1]],
                            "nearest_colours": [piece for _, piece in sorted((colours[o, piece], piece) for piece in pack.pieces)[:2]]})

    # 4. The report: poses as plain numbers, and what to doubt.
    out = []
    for piece in pieces:
        shape, pose = pack.shape(piece["name"]), piece.pop("pose")
        flags = []
        if piece["rest"] not in ("flat", "assembled"):
            flags.append(f"lying {piece['rest']}")
        if piece["support"] == "raised":
            flags.append(f"raised {piece['lift'] * 1000:.0f} mm: on top of something")
        if piece.get("upside_down"):
            flags.append("upside down")
        if piece.get("runner_up"):
            flags.append(f"could be {piece['runner_up']}")
        if piece.get("split_from"):
            flags.append("its mask also covered another piece")
        if piece.get("in_fragments"):
            flags.append(f"put together from {piece['in_fragments']} fragments")
        if piece.get("other_stance"):
            flags.append(f"from this view it could as well be standing another way ({piece['other_stance']})")
        if piece["seen"] < 0.5:
            flags.append("less than half of it seen")
        if piece["shape_cost"] > 0.6 * SHAPE_MAX and piece["rest"] != "assembled":
            flags.append("poor fit: tilted, leaning or partly hidden")
        index, inside = scan.cell(shape.top(pose[:3, :3], pose[:3, 3], cell=CELL)[:, :2])
        piece["_cells"] = np.zeros(scan.height.shape, dtype=bool)
        piece["_cells"][index[inside, 0], index[inside, 1]] = True
        out.append({**{k: v for k, v in piece.items() if k not in ("slot", "_cells", "lift", "upside_down", "other_stance", "in_fragments", "split_from")},
                    "position_m": [round(float(v), 5) for v in pose[:3, 3]], "rotation": np.round(pose[:3, :3], 6).tolist(),
                    "rpy_deg": [round(v, 3) for v in matrix_rpy(pose[:3, :3])],
                    "centroid_m": [round(float(v), 5) for v in pose[:3, :3] @ shape.centroid + pose[:3, 3]],
                    "shape_cost": round(piece["shape_cost"], 3), "colour_cost": round(piece["colour_cost"], 3),
                    "seen": round(piece["seen"], 3), "flags": flags, "touching": []})
        if "slot" in piece:
            out[-1]["in_parent"] = {"position_m": piece["slot"]["position_m"], "rpy_deg": piece["slot"]["rpy_deg"]}
    family = lambda piece: piece.get("parent") or piece["name"]
    for a in range(len(pieces)):                                # footprints within 4 mm of each other, outside one assembly
        near = ndimage.binary_dilation(pieces[a]["_cells"], iterations=2)
        out[a]["touching"] = [pieces[b]["name"] for b in range(len(pieces))
                              if a != b and family(pieces[a]) != family(pieces[b]) and (near & pieces[b]["_cells"]).any()]
    return {"pack": pack.name, "frame": scan.frame, "table": scan.table, "brightness_gain": round(math.exp(gain), 3),
            "objects": len(scan.objects), "pieces": out, "not_seen": sorted(free), "unexplained": unexplained,
            "doubtful": [piece["name"] for piece in out if piece["flags"]]}


# ---- what follows a match -----------------------------------------------------------------------------------

def difference(new, old, move_m=0.005):
    """What changed between two estimates, by piece name."""
    before, after = ({piece["name"]: piece for piece in estimate["pieces"]} for estimate in (old, new))
    moved = []
    for name in sorted(set(before) & set(after)):
        shift = math.dist(before[name]["centroid_m"], after[name]["centroid_m"])
        if shift > move_m or (after[name].get("parent") != before[name].get("parent")):
            moved.append({"name": name, "moved_mm": round(shift * 1000, 1), "from": before[name]["support"], "to": after[name]["support"]})
    return {"added": sorted(set(after) - set(before)), "removed": sorted(set(before) - set(after)), "moved": moved,
            "changed": bool(set(after) ^ set(before) or moved)}


def score(estimate, truth, pack, near_m=0.02):
    """The estimate against true poses {name: 4 x 4}. A true piece is found when an estimated piece's centre of volume
    is within ``near_m`` of its own; the pose error is then taken over the shape's symmetries."""
    rows, taken = [], set()
    for name, pose in truth.items():
        pose, shape = np.array(pose), pack.shape(name)
        centre = pose[:3, :3] @ shape.centroid + pose[:3, 3]
        near = sorted((math.dist(centre, piece["centroid_m"]), i) for i, piece in enumerate(estimate["pieces"])
                      if i not in taken and pack.shape_of[piece["name"]] == pack.shape_of[name])
        if not near or near[0][0] > near_m:
            rows.append({"name": name, "result": "missed"})
            continue
        found = estimate["pieces"][near[0][1]]
        taken.add(near[0][1])
        turn = min(angle_between(np.array(found["rotation"]) @ s, pose[:3, :3]) for s, _ in shape.symmetries)
        rows.append({"name": name, "result": "correct" if found["name"] == name else "misidentified", "as": found["name"],
                     "position_error_mm": round(near[0][0] * 1000, 2), "rotation_error_deg": round(turn, 2),
                     "flags": found["flags"], "rest": found["rest"]})
    extra = [piece["name"] for i, piece in enumerate(estimate["pieces"]) if i not in taken]
    good = [row for row in rows if row["result"] == "correct"]
    return {"pieces": rows, "true_pieces": len(rows), "correct": len(good),
            "misidentified": [f"{row['name']} as {row['as']}" for row in rows if row["result"] == "misidentified"],
            "missed": [row["name"] for row in rows if row["result"] == "missed"], "not_there": extra,
            "position_error_mm": {"median": round(float(np.median([r["position_error_mm"] for r in good])), 2),
                                  "max": max(r["position_error_mm"] for r in good)} if good else None,
            "rotation_error_deg": {"median": round(float(np.median([r["rotation_error_deg"] for r in good])), 2),
                                   "max": max(r["rotation_error_deg"] for r in good)} if good else None}


def write_task(estimate, folder, source, like=REPO / "exts/workcell/tasks/puzzle_table.json"):
    """The estimate as a puzzle task file and its BOM in ``folder``; density, friction and BOM wording come from the
    task file ``like``. Returns the task file's path."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    model = json.loads(Path(like).read_text())
    pack_id, version = estimate["pack"].split("/")
    record = lambda value, text: {"value": value, "status": "unconfirmed", "source": text}
    order = sorted(estimate["pieces"], key=lambda piece: (piece.get("parent") is not None and piece["rest"] == "assembled",))
    pieces = []
    for piece in order:
        placed = piece.get("in_parent")
        pieces.append({"name": piece["name"], "asset": piece["name"], **({"parent": piece["parent"]} if placed else {}),
                       "position_m": [*placed["position_m"][:2], round(placed["position_m"][2] + DROP_IN_M, 6)] if placed else piece["position_m"],
                       "rpy_deg": placed["rpy_deg"] if placed else piece["rpy_deg"]})
    names = [piece["name"] for piece in pieces]
    task = {"schema": 1, "optional": True, "description": f"Puzzle layout estimated from a scan: {source}",
            "pack": {"id": record(pack_id, "The pack the scan was matched against."), "version": record(version, "As above.")},
            "assets": {name: record(name, "Matched by puzzle_scan.py from colour and shape.") for name in names},
            "puzzle": {"pieces": record(pieces, f"Estimated by scripts/puzzle_scan.py from {source}; assembled pieces are at the pack's assembled poses, {DROP_IN_M * 1000:g} mm above resting."),
                       **{key: model["puzzle"][key] for key in ("density_kg_m3", "static_friction", "dynamic_friction")}},
            "documentation": {"bom_csv": record("task.bom.csv", "BOM rows of the pieces the scan found, next to this file.")}}
    (folder / "task.json").write_text(json.dumps(task, indent=1) + "\n")
    with (Path(like).with_suffix(".bom.csv")).open(newline="") as stream:
        rows = list(csv.reader(stream))
    with (folder / "task.bom.csv").open("w", newline="") as stream:
        csv.writer(stream, lineterminator="\n").writerows([rows[0]] + [row for row in rows[1:] if row[6] in names])
    return folder / "task.json"


# ---- a synthetic look, for testing the matcher --------------------------------------------------------------

def render(pack, poses, camera, noise_m=0.0, seed=0):
    """Colour, depth (metres along the optical axis) and a label image (1..n in ``poses`` order, 0 the table) of pieces
    at ``poses`` {name: 4 x 4}, by casting a ray per pixel at the pack's boxes. ``camera`` has fx, fy, cx, cy, width,
    height and pose (world_from_camera, optical frame)."""
    v, u = np.mgrid[0:camera["height"], 0:camera["width"]]
    rays = np.stack([(u - camera["cx"]) / camera["fx"], (v - camera["cy"]) / camera["fy"], np.ones(u.shape)], -1).reshape(-1, 3)
    pose = np.array(camera["pose"], dtype=float)
    eye, rays = pose[:3, 3], rays @ pose[:3, :3].T
    depth = np.where(rays[:, 2] < 0, -eye[2] / np.minimum(rays[:, 2], -1e-9), np.inf)       # the table
    label = np.zeros(len(rays), dtype=np.uint8)
    shade = np.full(len(rays), 0.9)
    colour = np.tile(np.array([0.02, 0.02, 0.02]), (len(rays), 1))
    light = np.array([0.3, -0.4, 0.86])
    for number, (name, placed) in enumerate(poses.items(), start=1):
        placed = np.array(placed)
        for centre, half in pack.shape(name).boxes:
            corners = np.array([placed[:3, :3] @ (centre + half * [sx, sy, sz]) + placed[:3, 3]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            local = (corners - eye) @ pose[:3, :3]
            if (local[:, 2] <= 0.01).any():
                continue
            us, vs = camera["fx"] * local[:, 0] / local[:, 2] + camera["cx"], camera["fy"] * local[:, 1] / local[:, 2] + camera["cy"]
            u0, u1 = max(0, int(us.min()) - 1), min(camera["width"], int(us.max()) + 2)
            v0, v1 = max(0, int(vs.min()) - 1), min(camera["height"], int(vs.max()) + 2)
            if u0 >= u1 or v0 >= v1:
                continue
            window = (np.arange(v0, v1)[:, None] * camera["width"] + np.arange(u0, u1)[None, :]).ravel()
            start = (eye - placed[:3, 3]) @ placed[:3, :3] - centre
            along = rays[window] @ placed[:3, :3]
            with np.errstate(divide="ignore", invalid="ignore"):
                a, b = (-half - start) / along, (half - start) / along
            near, far = np.minimum(a, b), np.maximum(a, b)
            enter, leave = near.max(1), far.min(1)
            hit = (enter < leave) & (enter > 0) & (enter < depth[window])
            face = near.argmax(1)[hit]
            normal = placed[:3, :3][:, face].T * -np.sign(along[hit, face])[:, None]
            window = window[hit]
            depth[window], label[window], colour[window] = enter[hit], number, pack.colour[name]
            shade[window] = 0.35 + 0.65 * np.clip(normal @ light, 0, 1)
    rng = np.random.default_rng(seed)
    depth = np.where(np.isfinite(depth), depth + rng.normal(0, noise_m, len(depth)) if noise_m else depth, 0.0)
    shape = (camera["height"], camera["width"])
    return _srgb(colour * shade[:, None]).reshape(*shape, 3), depth.reshape(shape), label.reshape(shape)


def synthetic(pack, poses, camera, clip, run, noise_m=0.0, seed=0):
    """Write ``render`` as a clip folder and a label folder (prompt ``pieces``), the two inputs of ``Scan``."""
    from PIL import Image

    image, depth, label = render(pack, poses, camera, noise_m, seed)
    for folder in (Path(clip) / "rgb", Path(clip) / "depth", Path(run) / "pieces"):
        folder.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(Path(clip) / "rgb/00000.png")
    Image.fromarray(np.round(depth / 0.0001).astype(np.uint16)).save(Path(clip) / "depth/00000.png")
    Image.fromarray(label).save(Path(run) / "pieces/labels_0000.png")
    (Path(clip) / "camera.json").write_text(json.dumps({**{k: camera[k] for k in ("fx", "fy", "cx", "cy")}, "depth_scale": 0.0001,
                                                        "frames": [{"world_from_camera": np.array(camera["pose"]).tolist()}]}))


def measure_colours(clip, truth, pack):
    """What colour each piece is to this camera, from a clip of a known layout: {piece: linear RGB}, the median over
    the pixels where the picture shows the piece's own top surface (``truth`` is {piece: 4 x 4 world pose})."""
    from PIL import Image

    clip, camera = Path(clip), json.loads((Path(clip) / "camera.json").read_text())
    image = _linear(np.asarray(Image.open(sorted((clip / "rgb").iterdir())[0]).convert("RGB")))
    first = sorted((clip / "depth").iterdir())[0]
    metres = np.load(first) if first.suffix == ".npy" else np.asarray(Image.open(first), dtype=np.float32) * camera.get("depth_scale", 0.001)
    pose = np.array((camera["frames"][0] if "frames" in camera else camera)["world_from_camera"], dtype=float)
    colours = {}
    for name, placed in truth.items():
        placed = np.array(placed)
        top = pack.shape(name).top(placed[:3, :3], placed[:3, 3], cell=CELL)
        local = (top - pose[:3, 3]) @ pose[:3, :3]
        u = np.round(camera["fx"] * local[:, 0] / local[:, 2] + camera["cx"]).astype(int)
        v = np.round(camera["fy"] * local[:, 1] / local[:, 2] + camera["cy"]).astype(int)
        inside = (u >= 0) & (u < metres.shape[1]) & (v >= 0) & (v < metres.shape[0])
        seen = inside & (np.abs(np.where(inside, metres[np.where(inside, v, 0), np.where(inside, u, 0)], 0) - local[:, 2]) < TOLERANCE_M)
        seen &= top[:, 2] >= top[:, 2].max() - 1.5 * TOLERANCE_M          # its upper surface, as a scan's objects are read
        if seen.sum() >= 50:
            colours[name] = [round(float(c), 4) for c in np.median(image[v[seen], u[seen]], axis=0)]
    return colours


def task_poses(path):
    """{piece name: world pose 4 x 4} of a puzzle task file."""
    frames = {}
    for piece in json.loads(Path(path).read_text())["puzzle"]["pieces"]["value"]:
        frames[piece["name"]] = frames.get(piece.get("parent"), np.eye(4)) @ _pose(rpy_matrix(piece["rpy_deg"]), piece["position_m"])
    return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    add = lambda name, **kw: commands.add_parser(name, **kw)
    p = add("match")
    p.add_argument("--clip", required=True, type=Path)
    p.add_argument("--run", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p = add("task")
    p.add_argument("--estimate", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p = add("diff")
    p.add_argument("--estimate", required=True, type=Path)
    p.add_argument("--previous", required=True, type=Path)
    p = add("score")
    p.add_argument("--estimate", required=True, type=Path)
    p.add_argument("--truth", required=True, type=Path, help="JSON {piece name: 4 x 4 world pose}, or a puzzle task file")
    p = add("colours")
    p.add_argument("--clip", required=True, type=Path)
    p.add_argument("--truth", required=True, type=Path, help="JSON {piece name: 4 x 4 world pose}, or a puzzle task file")
    p.add_argument("--out", required=True, type=Path)
    p = add("synthetic")
    p.add_argument("--task", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--camera", type=Path, help="JSON camera: fx, fy, cx, cy, width, height, pose; default 0.52 m above the layout")
    for command in ("match", "score", "synthetic", "colours"):
        commands.choices[command].add_argument("--pack", default="puzzles/v1")
    commands.choices["match"].add_argument("--colours", type=Path, help="measured colours, from the colours command")
    args = parser.parse_args(argv)
    read = lambda path: json.loads(path.read_text())
    if args.command == "task":
        estimate = read(args.estimate)
        print(write_task(estimate, args.out, str(args.estimate)))
        return 0
    if args.command == "diff":
        print(json.dumps(difference(read(args.estimate), read(args.previous)), indent=1))
        return 0
    pack = load_pack(args.pack, getattr(args, "colours", None) if args.command == "match" else None)
    if args.command == "colours":
        truth = read(args.truth)
        truth = {name: pose.tolist() for name, pose in task_poses(args.truth).items()} if "puzzle" in truth else truth
        found = measure_colours(args.clip, truth, pack)
        args.out.write_text(json.dumps({"clip": str(args.clip), "colours": found, "pack_colours": {name: pack.colour[name].tolist() for name in found}}, indent=1) + "\n")
        print(json.dumps(found))
    elif args.command == "match":
        estimate = match(Scan(args.clip, args.run), pack)
        estimate.update(clip=str(args.clip), run=str(args.run))
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(estimate, indent=1) + "\n")
        print(json.dumps({key: estimate[key] for key in ("objects", "not_seen", "unexplained", "doubtful")}
                         | {"pieces": {piece["name"]: piece["support"] for piece in estimate["pieces"]}}, indent=1))
    elif args.command == "score":
        truth = read(args.truth)
        truth = {name: pose.tolist() for name, pose in task_poses(args.truth).items()} if "puzzle" in truth else truth
        print(json.dumps(score(read(args.estimate), truth, pack), indent=1))
    else:
        poses = task_poses(args.task)
        middle = np.mean([pose[:2, 3] for pose in poses.values()], axis=0)
        camera = read(args.camera) if args.camera else {
            "fx": 924.0, "fy": 924.0, "cx": 640.0, "cy": 360.0, "width": 1280, "height": 720,
            "pose": _pose(np.diag([1.0, -1.0, -1.0]), [middle[0], middle[1], 0.52]).tolist()}
        synthetic(pack, poses, camera, args.out / "clip", args.out / "run")
        (args.out / "truth.json").write_text(json.dumps({name: pose.tolist() for name, pose in poses.items()}))
        print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
