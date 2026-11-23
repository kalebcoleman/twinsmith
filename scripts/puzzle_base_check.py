"""Where is the puzzle's base now? A fast answer from ONE depth picture, with no masks and no SAM pass.

The base lies flat on the table and its last pose is known. The picture's depth becomes points in the world; the ones
round that pose and not above the base are kept; and the base's CAD (its box colliders, with the pieces already seated
in it) is moved in x, y and yaw from the last pose until its height map agrees best with the heights seen: where the
CAD is, its top; elsewhere, the table. No object is segmented: the table's own points count, which is what tells the
base's edge.

``check`` answers with a pose, how far that is from the last one, and a verdict:

``steady``   the base is where it was (within the thresholds);
``moved``    it is somewhere else, and the fit is trusted: slots and seated pieces move with it;
``changed``  the fit is trusted, but something else on the table is not where the estimate has it;
``unsure``   the picture cannot vouch for the base: too little of it in view, heights that do not agree, a direction
             the fit could slide in for free, or a move as large as the fit looks for. The table has to be scanned.

NumPy only; the pack and its shapes are ``puzzle_scan``'s. Nothing here reads a simulator's truth.

    python3 scripts/puzzle_base_check.py <check_NNNNN.json>     # a check a build kept (its picture beside it): fit it again

The puzzle build keeps every check's picture and what the fit was given (``<out>/base_checks/``), so a fit can be run
again, here, with no simulator; a real camera's depth picture and pose in the same form can be tried the same way.
"""

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import puzzle_scan

CELL = 0.0005                                            # the height map's cell: the matcher's 2 mm cells put an edge up to 1 mm off
SETTINGS = {"moved_mm": 1.5, "moved_deg": 1.5,          # further than this from the last pose: the base has moved (a fork jams 2.6 mm off)
            "search_mm": 40.0, "search_deg": 9.0,        # how far from the last pose the fit looks
            "points": 8000,                              # depth points the fit uses, at most
            "stride": 3,                                 # every n-th pixel of the picture each way
            "tolerance_mm": 3.0, "cap_mm": 8.0,          # heights this close agree; a disagreement counts up to the cap
            "least_on_base": 300,                        # points that agree with the CAD's top: fewer, and too little of the base is in view
            "least_agreement": 0.80,                     # share of all points that agree with the fitted scene
            "least_rise": 0.002,                         # the cost must rise by this when the fit is pushed 3 mm or 3 degrees any way
            "changed_share": 0.03}                       # share of points that nothing known explains: something else has moved


def _yaw(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def template(pack, base, seated=None):
    """The height map of the base as modelled, with pieces seated in it ({piece: 4 x 4 in the base's frame}): heights
    above the base's own origin over a grid of ``CELL`` cells, nan where there is nothing; the grid's
    low corner and its middle, in the base's frame."""
    boxes = list(pack.shape(base).boxes)
    for piece, at in (seated or {}).items():
        at = np.asarray(at, dtype=float)
        boxes += [(at[:3, :3] @ centre + at[:3, 3], np.abs(at[:3, :3]) @ half) for centre, half in pack.shape(piece).boxes]
    low = np.min([c - h for c, h in boxes], axis=0)
    high = np.max([c + h for c, h in boxes], axis=0)
    cells = np.maximum(1, np.ceil((high[:2] - low[:2]) / CELL - 1e-6).astype(int))
    heights = np.full(cells, np.nan)
    for centre, half in boxes:
        a = np.clip(np.round((centre[:2] - half[:2] - low[:2]) / CELL).astype(int), 0, cells)
        b = np.clip(np.round((centre[:2] + half[:2] - low[:2]) / CELL).astype(int), 0, cells)
        heights[a[0]:b[0], a[1]:b[1]] = np.fmax(heights[a[0]:b[0], a[1]:b[1]], centre[2] + half[2])
    return {"padded": np.pad(heights, 1, constant_values=np.nan), "last": np.array(cells, dtype=float) + 1, "low": low[:2],
            "middle": (low[:2] + high[:2]) / 2, "reach": float(np.linalg.norm(high[:2] - low[:2]) / 2), "top": float(high[2])}


def world_points(depth_m, camera, stride=1):
    """The picture's depth as points in the world: (n, 3), pixels without a reading left out. ``camera`` has fx, fy,
    cx, cy and ``pose`` (world from the camera's optical frame)."""
    rows, columns = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = depth_m[::stride, ::stride]
    seen = z > 0
    z, rows, columns = z[seen], rows[seen], columns[seen]
    local = np.column_stack([(columns - camera["cx"]) / camera["fx"] * z, (rows - camera["cy"]) / camera["fy"] * z, z])
    pose = np.asarray(camera["pose"], dtype=float)
    return local @ pose[:3, :3].T + pose[:3, 3]


def _heights(model, xy, middle, yaw, lift):
    """The model's height in the world over each point, with its middle at ``middle`` and turned ``yaw``: 0 (the
    table) where it has nothing."""
    cells = np.floor(((xy - middle) @ _yaw(yaw) + model["middle"] - model["low"]) / CELL)
    index = np.clip(cells + 1.0, 0.0, model["last"]).astype(int)
    found = model["padded"][index[:, 0], index[:, 1]]
    return np.where(np.isnan(found), 0.0, found + lift)


def check(points, pack, base, last, seated=None, loose=None, settings=None):
    """The base's pose from depth points of the world (``world_points``), seeded from its ``last`` pose (4 x 4).
    ``seated`` are the pieces in it ({piece: pose in the base's frame}); ``loose`` the other pieces on the table as
    the estimate has them ({piece: world pose}), which explain their own points and are not fitted. Returns
    ``{"verdict", "pose", "moved_mm", "turned_deg", "agreement", "on_base", "points", "cost", "cost_at_last",
    "rise", "unexplained", "why", "fit_ms"}``."""
    began = time.perf_counter()
    opts = {**SETTINGS, **(settings or {})}
    last = np.asarray(last, dtype=float)
    if last[2, 2] < 0.99:
        return {"verdict": "unsure", "why": "the base's last pose does not lie flat", "pose": last, "moved_mm": 0.0, "turned_deg": 0.0,
                "fit_ms": round(1000 * (time.perf_counter() - began), 1)}
    model = template(pack, base, seated)
    yaw0, lift = math.atan2(last[1, 0], last[0, 0]), float(last[2, 3])
    middle0 = last[:2, 3] + _yaw(yaw0) @ model["middle"]
    tolerance, cap, search = opts["tolerance_mm"] / 1000, opts["cap_mm"] / 1000, opts["search_mm"] / 1000
    near = np.linalg.norm(points[:, :2] - middle0, axis=1) < model["reach"] + search + 0.01
    kept = points[near & (points[:, 2] < lift + model["top"] + tolerance) & (points[:, 2] > -0.02)]       # what stands higher is the hand and what it holds
    if len(kept) > opts["points"]:
        kept = kept[np.random.default_rng(0).choice(len(kept), opts["points"], replace=False)]
    xy, z = kept[:, :2], kept[:, 2]
    other = np.zeros(len(kept))                                # the loose pieces, where the estimate has them
    for piece, at in (loose or {}).items():
        at = np.asarray(at, dtype=float)
        other = np.maximum(other, np.maximum(puzzle_scan._top_at(pack.shape(piece), at[:3, :3], at[:3, 3], xy), 0.0))

    def cost(middle, yaw, some=slice(None)):
        over = np.abs(z[some] - np.maximum(_heights(model, xy[some], middle, yaw, lift), other[some]))
        return float(np.minimum(over, cap).mean() / cap) if len(over) else 1.0

    result = {"points": len(kept), "pose": last, "moved_mm": 0.0, "turned_deg": 0.0}
    if len(kept) < opts["least_on_base"]:
        return {**result, "verdict": "unsure", "why": f"only {len(kept)} depth points round the base's last place", "fit_ms": round(1000 * (time.perf_counter() - began), 1)}
    at_last = cost(middle0, yaw0)
    # Coarse: a grid round the last pose on a fifth of the points. Then downhill from the best of it, the steps halved.
    few = slice(None, None, 5)
    steps = np.linspace(-search, search, 9)
    turns = np.radians(np.linspace(-opts["search_deg"], opts["search_deg"], 7))
    best = min(((cost(middle0 + [dx, dy], yaw0 + turn, few), abs(dx) + abs(dy) + abs(turn), dx, dy, turn) for dx in steps for dy in steps for turn in turns))
    middle, yaw = middle0 + [best[2], best[3]], yaw0 + best[4]
    value, move, turn = cost(middle, yaw), search / 8, math.radians(opts["search_deg"]) / 6
    while move > 0.00005:
        better = False
        for d_middle, d_yaw in (((move, 0), 0), ((-move, 0), 0), ((0, move), 0), ((0, -move), 0), ((0, 0), turn), ((0, 0), -turn)):
            trial = cost(middle + d_middle, yaw + d_yaw)
            if trial < value - 1e-7:
                value, middle, yaw, better = trial, middle + d_middle, yaw + d_yaw, True
        if not better:
            move, turn = move / 2, turn / 2
    heights = _heights(model, xy, middle, yaw, lift)
    agree = np.abs(z - np.maximum(heights, other)) < tolerance
    on_base = int(np.count_nonzero(agree & (heights > other) & (heights > tolerance)))
    push, twist = 0.003, math.radians(3.0)                     # is every way of moving it in view? pushed either way, the cost must rise
    rise = [min(cost(middle + [push, 0], yaw), cost(middle - [push, 0], yaw)) - value,
            min(cost(middle + [0, push], yaw), cost(middle - [0, push], yaw)) - value,
            min(cost(middle, yaw + twist), cost(middle, yaw - twist)) - value]
    unexplained = int(np.count_nonzero(~agree & (heights <= tolerance) & (other <= tolerance) & (z > puzzle_scan.TABLE_M))) + \
        int(np.count_nonzero(~agree & (other > tolerance) & (heights <= tolerance)))
    pose = last.copy()
    pose[:2, :2] = _yaw(yaw)
    pose[:2, 3] = middle - _yaw(yaw) @ model["middle"]
    moved, turned = float(np.linalg.norm(middle - middle0)), abs((yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    result.update(pose=pose, moved_mm=round(moved * 1000, 2), turned_deg=round(math.degrees(turned), 3), agreement=round(float(agree.mean()), 3),
                  on_base=on_base, cost=round(value, 4), cost_at_last=round(at_last, 4), rise=[round(r, 4) for r in rise],
                  unexplained=unexplained)
    why = None
    if on_base < opts["least_on_base"]:
        why = f"only {on_base} points agree with the base's CAD: too little of it is in view"
    elif agree.mean() < opts["least_agreement"]:
        why = f"only {agree.mean() * 100:.0f} % of the heights agree with the fitted scene"
    elif min(rise) < opts["least_rise"]:
        why = f"the fit can slide {('along x', 'along y', 'in yaw')[int(np.argmin(rise))]} at no cost: that way is not in view"
    elif moved > search - 0.004 or math.degrees(turned) > opts["search_deg"] - 1.0:
        why = f"the base moved {moved * 1000:.0f} mm and {math.degrees(turned):.1f} degrees, as far as the fit looks"
    if why:
        verdict = "unsure"
    elif unexplained > opts["changed_share"] * len(kept):
        verdict, why = "changed", f"{unexplained} of {len(kept)} points are explained by nothing the estimate has"
    else:
        verdict = "moved" if moved * 1000 > opts["moved_mm"] or math.degrees(turned) > opts["moved_deg"] else "steady"
    return {**result, "verdict": verdict, "why": why, "fit_ms": round(1000 * (time.perf_counter() - began), 1)}


def again(path):
    """Fit a kept check again: ``check_NNNNN.json`` with its depth picture in ``depth/NNNNN.png`` beside it (16 bit,
    ``camera.depth_scale`` metres a count). Returns ``check``'s answer, and how far it is from the kept truth when
    the check has one (a simulator's)."""
    from PIL import Image

    path = Path(path)
    kept = json.loads(path.read_text())
    camera = kept["camera"]
    depth = np.asarray(Image.open(path.parent / "depth" / f"{path.stem.split('_')[-1]}.png"), dtype=np.float32) * camera["depth_scale"]
    pack = puzzle_scan.load_pack(kept["pack"])
    slots = {slot["piece"]: puzzle_scan._pose(puzzle_scan.rpy_matrix(slot["rpy_deg"]), slot["position_m"]) for slot in pack.assemblies[kept["base"]]}
    found = check(world_points(depth, camera, kept["settings"].get("stride", SETTINGS["stride"])), pack, kept["base"], kept["last"],
                  {piece: slots[piece] for piece in kept["seated"]}, kept["loose"], kept["settings"])
    if kept.get("truth"):
        off = np.linalg.inv(np.array(kept["truth"])) @ found["pose"]
        found["fit_against_truth_mm"] = round(1000 * float(np.linalg.norm(off[:2, 3])), 2)
        found["fit_against_truth_deg"] = round(abs(math.degrees(math.atan2(off[1, 0], off[0, 0]))), 3)
    return found


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    print(json.dumps({key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in again(sys.argv[1]).items()}, indent=1))
