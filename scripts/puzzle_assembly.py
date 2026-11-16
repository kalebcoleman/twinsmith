"""Plan the assembly of a puzzle from its CAD, on the CPU: no simulator, no camera, no model.

Everything is worked out from the puzzles pack's collision boxes (a piece is exactly the union of its boxes) and its
assembled poses:

* ``disassembly``: take the assembled puzzle apart piece by piece, each along a direction in which nothing is in
  its way. Reversed, that is a build order; which pieces were in which piece's way is the task graph (``needs``), and
  the direction a piece came out along is its slot axis.
* ``choose_grasps``: a grasp is defined on the CAD, in the piece's own frame: a point, a closing axis, an approach
  axis and the width gripped. Every way that grasp can be held (the piece's own symmetries, the gripper turned half
  round) is checked at the pick and in the slot against the pieces, the table and a keep-out model of the gripper,
  and the ones that fit are returned, best first.
* ``scatter``: a random layout of one puzzle's pieces lying loose with its base off to the side, for testing.

Frames: a pose is 4 x 4, world (or base) from the piece, for column vectors. The gripper frame is the one of
``workcell_session``: +x the closing axis, +z out through the fingertips, origin at the centre of the pads.
"""

from __future__ import annotations

import math

import numpy as np

import puzzle_scan

AXES = ((0, 0, 1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))    # ways out of a base, in its frame; up first
TOUCH_M = 0.0003                # boxes this deep in each other are resting on each other, not in each other's way
NEAR_M = 0.003                  # a piece on its way out is clear of the rest when nothing is this near
# Keep-out boxes of the RG2 and the wrist camera, measured on the built cell (ur5e-workcell/v3) in the gripper frame:
# ((x low, x high) beyond half the pad gap, (y low, y high), (z low, z high) from the pad centre), mirrored in x.
FINGER_KEEP_OUT = (((0.0, 0.0114), (-0.0103, 0.0103), (-0.015, 0.015)),          # a fingertip pad
                   ((0.004, 0.024), (-0.012, 0.012), (-0.043, -0.016)),           # the finger link above it
                   ((-0.030, 0.026), (-0.017, 0.017), (-0.090, -0.043)))          # the knuckles
# (x, y, z) ranges from the TCP (the pad centre with the pads together): the body with the wrist, and the camera on
# its bracket.
BODY_KEEP_OUT = (((-0.042, 0.042), (-0.038, 0.044), (-0.262, -0.070)),
                 ((-0.058, 0.042), (-0.150, -0.027), (-0.147, -0.113)))


def pose(turn, at):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = turn, at
    return out


def slot_poses(pack, base):
    """{piece: its assembled pose in the base's frame}, in the pack's order."""
    return {slot["piece"]: pose(puzzle_scan.rpy_matrix(slot["rpy_deg"]), slot["position_m"]) for slot in pack.assemblies[base]}


def placed(at, boxes):
    """A solid's boxes at a pose: [(centre, 3 half-extent vectors as rows)]."""
    return [(at[:3, :3] @ centre + at[:3, 3], (at[:3, :3] * half).T) for centre, half in boxes]


def overlap(a, b):
    """How deep two oriented boxes are in each other, metres (0 when apart): the least overlap over the separating
    axis test's axes."""
    (ca, ha), (cb, hb) = a, b
    axes = [h / np.linalg.norm(h) for h in (*ha, *hb)]
    axes += [c / np.linalg.norm(c) for u in axes[:3] for v in axes[3:] for c in [np.cross(u, v)] if np.linalg.norm(c) > 1e-6]
    depth = math.inf
    for axis in axes:
        depth = min(depth, np.abs(ha @ axis).sum() + np.abs(hb @ axis).sum() - abs((cb - ca) @ axis))
        if depth <= 0:
            return 0.0
    return float(depth)


def deepest(first, second):
    """The deepest overlap between any box of one solid and any box of another."""
    worst = 0.0
    for a in first:
        reach = np.linalg.norm(np.abs(a[1]).sum(0))
        for b in second:
            if np.linalg.norm(a[0] - b[0]) < reach + np.linalg.norm(np.abs(b[1]).sum(0)):     # the boxes' spheres meet
                worst = max(worst, overlap(a, b))
    return worst


def _spans(boxes):
    """Boxes whose sides lie along the frame's axes, as (low corners, high corners)."""
    reach = np.array([np.abs(half).sum(0) for _, half in boxes])
    if any(np.abs(half).sum() - np.abs(half).max(0).sum() > 1e-9 for _, half in boxes):
        raise ValueError("an assembled piece is not at right angles to its base")
    centres = np.array([centre for centre, _ in boxes])
    return centres - reach, centres + reach


def _depth(first, second, shift=0.0, grow=0.0):
    """The deepest overlap between two sets of axis-aligned boxes, the first moved by ``shift`` and grown by ``grow``."""
    low = np.maximum(first[0][:, None] + shift - grow, second[0][None])
    high = np.minimum(first[1][:, None] + shift + grow, second[1][None])
    return float(np.max(np.min(high - low, axis=2), initial=0.0))


# ---- the build order ------------------------------------------------------------------------------------------

def disassembly(pack, base, reach_m=0.16, step_m=0.002):
    """Take the assembled puzzle apart. Returns the removals in order, each ``{"piece", "axis", "clear_m",
    "blocked_by"}``: the direction it comes out along in the base's frame, how far along it the piece is clear of
    everything, and the pieces that are in its way along that direction while they are still in the base."""
    slots = slot_poses(pack, base)
    solid = {name: _spans(placed(at, pack.shape(name).boxes)) for name, at in slots.items()}
    solid[base] = _spans(placed(np.eye(4), pack.shape(base).boxes))

    def sweep(piece, axis):
        """Who the piece meets on its way out along an axis, and for each other solid how far out the piece is last
        within NEAR_M of it."""
        met, near = set(), {}
        for count in range(1, round(reach_m / step_m) + 1):
            shift = np.array(axis) * count * step_m
            for other in solid:
                if other != piece and _depth(solid[piece], solid[other], shift, NEAR_M) > 0:
                    near[other] = count * step_m
                    if _depth(solid[piece], solid[other], shift) > TOUCH_M:
                        met.add(other)
        return met, near

    ways = {(piece, axis): sweep(piece, axis) for piece in slots for axis in AXES}
    left, steps = list(slots), []
    while left:
        free = [(piece, axis) for axis in AXES for piece in reversed(left)
                if not ways[piece, axis][0] & (set(left) | {base})]
        if not free:
            raise ValueError(f"{base}: no piece of {left} comes out along a straight line")
        piece, axis = free[0]
        left.remove(piece)
        met, near = ways[piece, axis]
        steps.append({"piece": piece, "axis": list(axis), "blocked_by": sorted(met - {base}),
                      "clear_m": round(max(near.get(other, 0.0) for other in [base, *left]), 4)})
    return steps


def build_order(pack, base):
    """The disassembly reversed: ``[{"piece", "axis", "clear_m", "needs"}]`` in an order that can be built. ``axis``
    is the slot axis pointing out of the base; ``needs`` are the pieces that must be in before this one, because this
    one would be in their way."""
    steps = disassembly(pack, base)
    return [{"piece": step["piece"], "axis": step["axis"], "clear_m": step["clear_m"],
             "needs": sorted(other["piece"] for other in steps if step["piece"] in other["blocked_by"])}
            for step in reversed(steps)]


# ---- grasps ---------------------------------------------------------------------------------------------------

def grasp_frame(grasp):
    """A grasp definition as the gripper's pose in the piece's frame, with the pad centre at the grasp point."""
    closing, approach = (np.array(grasp[key], dtype=float) for key in ("closing_axis", "approach_axis"))
    return pose(np.column_stack([closing, np.cross(approach, closing), approach]), grasp["point_m"])


def gripper_boxes(gripper, gap_m, setback_m):
    """The gripper's keep-out boxes at a pose of its pad centre, with the pads ``gap_m`` apart. ``setback_m`` is how
    much nearer the body the pads are at this gap than when together (``workcell_session.pad_setback``)."""
    boxes = []
    for side in (-1, 1):
        for (x0, x1), (y0, y1), (z0, z1) in FINGER_KEEP_OUT:
            low, high = np.array([gap_m / 2 + x0, y0, z0]), np.array([gap_m / 2 + x1, y1, z1])
            boxes.append((np.array([side, 1, 1]) * (low + high) / 2, (high - low) / 2))
    for ranges in BODY_KEEP_OUT:
        low, high = np.array([r[0] for r in ranges]), np.array([r[1] for r in ranges])
        boxes.append(((low + high) / 2 + [0, 0, setback_m], (high - low) / 2))
    return placed(gripper, boxes)


def below_table(boxes, margin_m=0.001):
    return any(centre[2] - np.abs(half[:, 2]).sum() < margin_m for centre, half in boxes)


def choose_grasps(pack, piece, grasp, now, target, obstacles_now, obstacles_then, setback, axis=(0, 0, 1), clear_m=0.0, slack_m=0.001):
    """The ways to hold ``piece`` that fit both where it lies (``now``, world) and where it goes (``target``, world).

    ``grasp`` is its definition on the CAD; ``obstacles_now`` and ``obstacles_then`` are {name: world pose} of the
    other pieces at the pick and at the place; ``setback`` is ``workcell_session.pad_setback``; ``axis`` is the slot
    axis in the world and ``clear_m`` how far out along it the piece is clear, so the gripper is checked along the whole
    insertion. Returns ``[{"in_piece", "pick", "place", "symmetry", "camera_up"}]``, best first: gripper poses for the
    pad centre in the piece's frame and in the world. A grasp approaches the lying piece from above, and neither the
    gripper nor the camera may be in anything by more than ``slack_m`` (the fingertips press on the piece)."""
    shape, width = pack.shape(piece), grasp["width_m"]
    half_turn = pose(np.diag([-1.0, -1.0, 1.0]), [0, 0, 0])
    own = lambda at: placed(at, shape.boxes)
    solids = lambda poses: [box for name, at in poses.items() for box in placed(at, pack.shape(name).boxes)]
    around_now, around_then = solids(obstacles_now), solids(obstacles_then)
    found = []
    for turn, shift in shape.symmetries:                       # how the piece's frame may sit on the lying piece
        for flip in (np.eye(4), half_turn):
            held = pose(turn, shift) @ grasp_frame(grasp) @ flip
            pick = now @ held
            if pick[2, 2] > -0.99:                             # from above
                continue
            hand = gripper_boxes(pick, width, setback(width))
            if below_table(hand) or deepest(hand, own(now)) > slack_m or deepest(hand, around_now) > slack_m:
                continue
            for again, back in shape.symmetries:               # which of its equal poses it takes in the slot
                seat = target @ pose(again, back)
                place = seat @ held
                fits = True
                for out in np.linspace(0.0, clear_m, max(2, round(clear_m / 0.01) + 1)):
                    lift = pose(np.eye(3), np.array(axis) * out)
                    hand = gripper_boxes(lift @ place, width, setback(width))
                    if below_table(hand) or deepest(hand, around_then) > slack_m or deepest(hand, own(lift @ seat)) > slack_m:
                        fits = False
                        break
                if fits:
                    camera = (place @ [0.0, -1.0, 0.0, 0.0])[2]          # the camera rides on the gripper's -y side
                    found.append({"in_piece": held, "pick": pick, "place": place, "seat": seat, "camera_up": float(camera)})
    return sorted(found, key=lambda option: -option["camera_up"])


def nearest(pack, piece, at, target):
    """Of the poses a symmetric piece can take in a target pose, the one nearest ``at``."""
    return min((target @ pose(turn, shift) for turn, shift in pack.shape(piece).symmetries),
               key=lambda other: np.linalg.norm(at[:3, 3] - other[:3, 3]) + 0.001 * puzzle_scan.angle_between(at[:3, :3], other[:3, :3]))


def pose_error(pack, piece, at, target):
    """How far a pose is from a target, allowing for the piece's symmetries: (metres, degrees)."""
    other = nearest(pack, piece, at, target)
    return float(np.linalg.norm(at[:3, 3] - other[:3, 3])), puzzle_scan.angle_between(at[:3, :3], other[:3, :3])


# ---- a layout to build from -----------------------------------------------------------------------------------

def footprint(pack, piece, at, margin_m=0.0):
    """The rectangle around a piece on the table at a pose, grown by ``margin_m`` on every side: corners (4, 2)."""
    shape = pack.shape(piece)
    low = np.min([centre - half for centre, half in shape.boxes], axis=0)
    high = np.max([centre + half for centre, half in shape.boxes], axis=0)
    centre, half = placed(at, [((low + high) / 2, (high - low) / 2)])[0]
    a, b = sorted((side[:2] for side in half), key=np.linalg.norm)[1:]          # the two sides that lie on the table
    a, b = (side * (1 + margin_m / np.linalg.norm(side)) for side in (a, b))
    return np.array([centre[:2] + sa * a + sb * b for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _apart(a, b):
    """True when two rectangles (corners in order) do not overlap: one of their sides separates them."""
    for quad in (a, b):
        for edge in (quad[1] - quad[0], quad[2] - quad[1]):
            if np.max(a @ edge) <= np.min(b @ edge) or np.max(b @ edge) <= np.min(a @ edge):
                return True
    return False


def slide_path(start, target, count=12):
    """Poses of a base slid in a straight line on the table from one pose to another, turning evenly on the way."""
    turn = math.atan2(*(target[:3, :3] @ start[:3, :3].T)[[1, 0], [0, 0]])
    return [pose(puzzle_scan.yaw_matrix(turn * part) @ start[:3, :3], start[:3, 3] + (target[:3, 3] - start[:3, 3]) * part)
            for part in np.linspace(0.0, 1.0, count)]


def path_clear(pack, base, start, target, others, margin_m=0.02):
    """True when the base can slide from ``start`` to ``target`` without touching ``others`` ({piece: pose})."""
    return all(_apart(footprint(pack, base, at, margin_m), footprint(pack, name, there))
               for at in slide_path(start, target) for name, there in others.items())


def scatter(pack, base, target, rng, region=((-0.36, -0.64), (0.40, -0.30)), blind=((-0.10, -0.34), (0.14, -0.15)),
            side_m=(0.24, 0.32), gap_m=0.035, tries=4000):
    """One puzzle's pieces lying flat at random on the table, apart from each other, with the base off to one side of
    ``target`` (its pose in the middle) and both the middle and the way there left free. ``region`` is what the wrist
    camera sees from the survey pose and the arm reaches; ``blind`` what the camera's own fingertips hide. Returns
    {piece: world pose} with the base first."""
    low, high = np.array(region[0]), np.array(region[1])

    def lying(name, xy):
        rest = pack.shape(name).rests[0]
        return pose(puzzle_scan.yaw_matrix(rng.uniform(0, 2 * math.pi)) @ rest.turn, [xy[0], xy[1], rest.drop])

    hidden = np.array([blind[0], [blind[1][0], blind[0][1]], blind[1], [blind[0][0], blind[1][1]]])

    def inside(name, at):
        corners = footprint(pack, name, at)
        return (corners > low).all() and (corners < high).all() and _apart(footprint(pack, name, at, gap_m), hidden)

    for _ in range(tries):
        side = rng.choice([-1, 1])
        start = lying(base, [target[0, 3] + side * rng.uniform(*side_m), rng.uniform(low[1] + 0.13, high[1] - 0.13)])
        if not inside(base, start):
            continue
        poses, keep_free = {base: start}, [footprint(pack, base, at, 0.03) for at in slide_path(start, target)]
        for name in slot_poses(pack, base):
            for _ in range(300):
                at = lying(name, rng.uniform(low, high))
                grown = footprint(pack, name, at, gap_m)
                if inside(name, at) and all(_apart(grown, kept) for kept in keep_free) \
                        and all(_apart(grown, footprint(pack, other, there)) for other, there in poses.items()):
                    poses[name] = at
                    break
            else:
                break
        if len(poses) == len(pack.assemblies[base]) + 1:
            return poses
    raise ValueError("no scattered layout found; the region is too small for these pieces")


def layout_estimate(pack, poses):
    """A layout in the form ``puzzle_scan.write_task`` takes."""
    return {"pack": pack.name, "pieces": [{"name": name, "position_m": [round(float(v), 6) for v in at[:3, 3]],
                                           "rpy_deg": [round(v, 4) for v in puzzle_scan.matrix_rpy(at[:3, :3])], "rest": "flat"}
                                          for name, at in poses.items()]}
