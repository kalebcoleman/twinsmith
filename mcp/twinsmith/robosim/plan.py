"""Plain-data checks and arithmetic for the robosim bridge: push, timing, time mapping, scene scan.

Nothing here touches RSS, the disk or the network, so the arithmetic is tested on its own.
"""

from __future__ import annotations

import math
import re

PUSH_KEYS = ("waypoints", "object_velocity")
MAX_WAYPOINTS = 200
MAX_STEPS = 20000
MAX_COORDINATE_M = 2.0
FAST_PUSHER_M_S = 1.0
DURATION_TOLERANCE = 0.10

ENGINES = {
    "cpu": "plain MuJoCo (MuJoCo's C library) on the CPU",
    "cuda:0": "Newton's MuJoCo-Warp solver on the GPU",
}


def engine(device):
    """Which physics engine a run's device selects: CPU is plain MuJoCo, CUDA is MuJoCo-Warp."""
    return {"device": device, "physics": ENGINES.get(device, "unknown"),
            "scope": "Compares images. It does not fit mass or friction."}


GPU_FIRST_RUN_NOTE = ("The first action on cuda:0 can take minutes while Warp compiles its kernels (the cache under "
                      "~/.cache/warp makes later runs fast); status shows whether the run is still working.")
PUSHER_TRACK_LIMIT_M = 0.005  # the template's own limit for pusher_tracked


def untracked_message(applied):
    """Why a rollout whose pusher left its path is not a path problem; ``None`` when the pusher tracked (or is unknown)."""
    if not applied or applied.get("pusher_tracked") is not False:
        return None
    solver = applied.get("solver")
    cause = (f"The scene was stepped by {solver}, which does not take the pusher's coordinate writes; the template names "
             "physics=MUJOCO_500HZ for this reason, so a hand-written scene must do the same." if solver and solver != "SolverMuJoCo"
             else "Check the scene's physics arrangement and that the pusher is a free body moved by place_free_body.")
    return (f"PUSHER DID NOT TRACK ITS PATH: it missed it by up to {applied.get('pusher_tracking_error_max_m')} m (limit "
            f"{PUSHER_TRACK_LIMIT_M} m), so the block was not pushed. This is a fault in the simulation setup, not in the push "
            f"path: changing the waypoints will not help. {cause}")


ENGINES_SHORT = {"cpu": "MuJoCo (CPU)", "cuda:0": "MuJoCo-Warp (GPU)"}


def engine_short(device):
    return ENGINES_SHORT.get(device, "unknown")


def _number(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number.")
    return float(value)


def validate_push(push):
    """Check and normalise ``push``; return ``(push, warnings)``.

    ``push`` is ``{"waypoints": [[t, x, y], ...], "object_velocity": [vx, vy, yaw_rate]}``. Times are
    seconds and strictly increase; x and y are pusher positions on the table in metres. Either key may
    be left out, but not both. A tower's push is ``{"block", "distance", "seconds", "direction"}`` instead
    (``tower.validate_push``): the finger pushes that block along its long side.
    """
    if not isinstance(push, dict):
        raise ValueError("push must be an object like {'waypoints': [[0, x, y], [2, x, y]], 'object_velocity': [vx, vy, wz]}.")
    if "block" in push:
        from . import tower

        return tower.validate_push(push), []
    unknown = sorted(set(push) - set(PUSH_KEYS))
    if unknown:
        raise ValueError(f"push has unknown keys {unknown}; use {list(PUSH_KEYS)}.")
    if not any(push.get(key) for key in PUSH_KEYS):
        raise ValueError("push needs waypoints [[seconds, x, y], ...] or object_velocity [vx, vy, yaw_rate].")
    out, warnings = {"waypoints": None, "object_velocity": None}, []
    raw = push.get("waypoints")
    if raw:
        if not isinstance(raw, (list, tuple)) or not 2 <= len(raw) <= MAX_WAYPOINTS:
            raise ValueError(f"waypoints must be a list of 2 to {MAX_WAYPOINTS} items [seconds, x, y].")
        points = []
        for i, item in enumerate(raw):
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                raise ValueError(f"waypoints[{i}] must be [seconds, x, y].")
            t, x, y = (_number(v, f"waypoints[{i}]") for v in item)
            if t < 0 or abs(x) > MAX_COORDINATE_M or abs(y) > MAX_COORDINATE_M:
                raise ValueError(f"waypoints[{i}] needs seconds >= 0 and x, y within {MAX_COORDINATE_M} m.")
            if points and t <= points[-1][0]:
                raise ValueError(f"waypoints times must strictly increase; waypoints[{i}] has {t} after {points[-1][0]}.")
            points.append([t, x, y])
        fastest = max(math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(points, points[1:]))
        if fastest > FAST_PUSHER_M_S:
            warnings.append(f"The pusher moves up to {fastest:.2f} m/s; above {FAST_PUSHER_M_S} m/s it can pass through the object between steps.")
        out["waypoints"] = points
    velocity = push.get("object_velocity")
    if velocity:
        if not isinstance(velocity, (list, tuple)) or len(velocity) != 3:
            raise ValueError("object_velocity must be [vx, vy, yaw_rate] in m/s and rad/s.")
        out["object_velocity"] = [_number(v, "object_velocity") for v in velocity]
    if out["waypoints"] is None:
        out["waypoints"] = [[0.0, 0.0, -1.0], [1.0, 0.0, -1.0]]
        warnings.append("No waypoints: the pusher stays off the table at (0, -1).")
    return out, warnings


def rollout_plan(control_hz, *, duration_s=None, steps=None, record_fps=10.0, push=None):
    """Turn a duration or step count and a record rate into steps, a recording stride and the real rates.

    Exactly one of ``duration_s`` and ``steps`` may be given; with neither, the push's last waypoint time
    decides, else 2 s. ``record_fps`` is how many frames per simulated second to record; the stride is
    the nearest whole number of control steps, so the real rate is reported too.
    """
    hz = _number(control_hz, "control_hz")
    if hz <= 0:
        raise ValueError("The scene reports a control rate of zero or less.")
    fps_asked = _number(record_fps, "record_fps")
    if not 0 < fps_asked <= hz:
        raise ValueError(f"record_fps must be above 0 and at most the control rate ({hz:g} Hz).")
    if duration_s is not None and steps is not None:
        raise ValueError("Give duration_s or steps, not both.")
    warnings = []
    if steps is not None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be a positive integer.")
        n = steps
    else:
        if duration_s is None:
            waypoints = (push or {}).get("waypoints")
            duration_s = waypoints[-1][0] if waypoints else (push["seconds"] + 1.0 if push and "seconds" in push else 2.0)
        duration_s = _number(duration_s, "duration_s")
        if duration_s <= 0:
            raise ValueError("duration_s must be positive.")
        n = max(1, round(duration_s * hz))
    if n > MAX_STEPS:
        raise ValueError(f"{n} steps is more than the limit of {MAX_STEPS} ({MAX_STEPS / hz:.0f} s at {hz:g} Hz).")
    every = max(1, round(hz / fps_asked))
    real_fps = hz / every
    if abs(real_fps - fps_asked) > 0.01 * fps_asked:
        warnings.append(f"record_fps {fps_asked:g} is not a whole number of control steps at {hz:g} Hz; recording every "
                        f"{every} steps = {real_fps:g} frames per simulated second.")
    return {"steps": n, "duration_s": n / hz, "control_hz": hz, "every": every, "record_fps_asked": fps_asked,
            "record_fps": real_fps, "frames_expected": n // every + (1 if n % every else 0) + 1, "warnings": warnings}


def time_mapping(anchor_times, take_duration, *, clip_start=None, clip_end=None, sim_start=None, sim_end=None):
    """Map clip seconds to simulated seconds, linearly, with explicit windows.

    Defaults: the clip window is the span of the anchors; the simulated window has the same duration and
    starts at zero. Returns the windows, the scale (simulated seconds per clip second), a ``sim_t`` for
    every anchor inside the clip window, the anchors left out, and warnings (durations that differ by
    more than 10 percent).
    """
    if not anchor_times:
        raise ValueError("The reference has no anchors; run reference first.")
    c0 = min(anchor_times) if clip_start is None else _number(clip_start, "start_s")
    c1 = max(anchor_times) if clip_end is None else _number(clip_end, "end_s")
    if c0 < 0 or c1 <= c0:
        raise ValueError("The clip window needs start_s >= 0 and end_s > start_s.")
    s0 = 0.0 if sim_start is None else _number(sim_start, "sim_start_s")
    s1 = s0 + (c1 - c0) if sim_end is None else _number(sim_end, "sim_end_s")
    if s0 < 0 or s1 <= s0:
        raise ValueError("The simulated window needs sim_start_s >= 0 and sim_end_s > sim_start_s.")
    if s1 > take_duration + 1e-6:
        raise ValueError(f"The take lasts {take_duration:g} s of simulated time but the simulated window ends at {s1:g} s. "
                         "Lower sim_end_s, or record a longer rollout.")
    scale = (s1 - s0) / (c1 - c0)
    inside = [t for t in anchor_times if c0 - 1e-6 <= t <= c1 + 1e-6]
    outside = [t for t in anchor_times if t not in inside]
    if len(inside) < 2:
        raise ValueError(f"Fewer than two reference anchors fall inside the clip window {c0:g}–{c1:g} s; "
                         "run reference again with 'at' times that cover it.")
    warnings = []
    if abs((s1 - s0) - (c1 - c0)) > DURATION_TOLERANCE * (c1 - c0):
        warnings.append(f"The clip window lasts {c1 - c0:g} s but the simulated window {s1 - s0:g} s; sim time is mapped at "
                        f"{scale:.3g} simulated s per clip s. A speed difference can look like different motion.")
    return {"clip_start_s": c0, "clip_end_s": c1, "sim_start_s": s0, "sim_end_s": s1,
            "sim_seconds_per_clip_second": scale, "take_duration_s": take_duration,
            "sim_t": {t: round(s0 + (t - c0) * scale, 6) for t in inside}, "anchors_outside_window": outside,
            "warnings": warnings}


def relabel_ratios(reading):
    """Replace RSS's ``size_ratio`` by what it is: a linear scale, with the area ratio from pixel counts.

    RSS computes ``size_ratio = sqrt(sim_pixels / photo_pixels)`` (compare.py ``compare_masks``), so it is
    the square root of the area ratio. The area ratio is ``sim.pixels / photo.pixels``.
    """
    if not isinstance(reading, dict):
        return reading
    out = {key: value for key, value in reading.items() if key != "size_ratio"}
    photo, sim = reading.get("photo"), reading.get("sim")
    if photo and sim and photo.get("pixels"):
        out["area_ratio"] = round(sim["pixels"] / photo["pixels"], 3)
    else:
        out["area_ratio"] = None
    out["linear_scale_ratio"] = reading.get("size_ratio")
    return out


def centroid_displacement(first, last, name):
    """Image-space centroid displacement of ``name`` between two anchors, for the clip and the sim."""
    def pick(anchor, side):
        found = ((anchor.get("objects") or {}).get(name) or {}).get(side)
        return found["centre"] if found else None

    sides = {}
    for side in ("photo", "sim"):
        a, b = pick(first, side), pick(last, side)
        sides[side] = None if a is None or b is None else [round(b[0] - a[0], 1), round(b[1] - a[1], 1)]
    clip, sim = sides["photo"], sides["sim"]
    entry = {"clip_px": clip, "sim_px": sim, "difference_px": None, "same_sign_x": None, "same_sign_y": None,
             "magnitude_ratio": None, "clip_length_px": None, "sim_length_px": None, "direction_error_deg": None}
    if clip is not None and sim is not None:
        entry["difference_px"] = [round(sim[0] - clip[0], 1), round(sim[1] - clip[1], 1)]
        entry["same_sign_x"] = bool(clip[0] * sim[0] > 0)
        entry["same_sign_y"] = bool(clip[1] * sim[1] > 0)
        length = math.hypot(*clip)
        entry["clip_length_px"] = round(length, 1)
        entry["sim_length_px"] = round(math.hypot(*sim), 1)
        entry["magnitude_ratio"] = round(math.hypot(*sim) / length, 3) if length else None
        entry["direction_error_deg"] = direction_error_deg(clip, sim)
    return entry


def direction_error_deg(a, b):
    """The angle between two image displacements in degrees (0 to 180), or None if either has no length."""
    la, lb = math.hypot(*a), math.hypot(*b)
    if la < 1e-9 or lb < 1e-9:
        return None
    cosine = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (la * lb)))
    return round(math.degrees(math.acos(cosine)), 1)


_BODY_WRITE = re.compile(r"\bbody_qd?\s*\.\s*assign\s*\(")


def scene_problems(text):
    """Writes the scene makes that the solver silently discards, found by scanning its source."""
    problems = []
    if _BODY_WRITE.search(text):
        problems.append(
            "The scene writes sim.state.body_q or body_qd with .assign(...). The solver rebuilds those from joint_q "
            "and joint_qd every step, so the write is discarded and nothing moves. Use scene_template's pusher with "
            "rollout(push={'waypoints': [[seconds, x, y], ...]}), or write the free body's joint_q and joint_qd slices "
            "(entity.coord_start and entity.dof_start, as the template's place_free_body does).")
    return problems


# ---- the bridge's own verdict --------------------------------------------------------------------
VERDICT_THRESHOLDS = {
    "matches": "magnitude_ratio from 0.5 to 2.0, direction_error_deg below 30 and end_frame_overlap above 0.2",
    "moved, but not like the clip": "moved 3 px or more but at least one test for 'matches' failed",
    "did not move": "the simulated displacement is under 3 px",
    "magnitude_ratio_range": [0.5, 2.0], "direction_error_deg_below": 30.0, "end_frame_overlap_above": 0.2,
    "did_not_move_below_px": 3.0,
    "definitions": "displacement = image-centroid displacement of the mask from the first to the last anchor; "
                   "magnitude_ratio = sim length / clip length; direction_error_deg = angle between the two "
                   "displacements; end_frame_overlap = RSS's IoU at the last anchor.",
}
VERDICT_ORDER = ("did not move", "moved, but not like the clip", "matches")


def bridge_verdict(moved, end_overlap):
    """The bridge's own verdict for one object from its centroid displacement entry and end-frame IoU.

    ``matches`` needs magnitude ratio 0.5-2.0, direction error under 30 degrees and end overlap above 0.2;
    ``did not move`` is a simulated displacement under 3 px; anything else is ``moved, but not like the
    clip`` and lists the numbers that failed. RSS's own score and verdict are not used.
    """
    t = VERDICT_THRESHOLDS
    numbers = {"clip_px": moved.get("clip_px"), "sim_px": moved.get("sim_px"), "magnitude_ratio": moved.get("magnitude_ratio"),
               "direction_error_deg": moved.get("direction_error_deg"), "end_frame_overlap": end_overlap,
               "sim_length_px": moved.get("sim_length_px"), "clip_length_px": moved.get("clip_length_px")}
    if moved.get("sim_px") is None or moved.get("clip_px") is None:
        return {"verdict": "did not move", "failed": ["the object was not found in the masks at the first and last anchors"],
                "numbers": numbers}
    if moved["sim_length_px"] < t["did_not_move_below_px"]:
        also = " (the clip's block moved only %.1f px too)" % moved["clip_length_px"] if moved["clip_length_px"] < t["did_not_move_below_px"] else ""
        return {"verdict": "did not move", "numbers": numbers,
                "failed": [f"simulated displacement {moved['sim_length_px']} px is under {t['did_not_move_below_px']:g} px{also}"]}
    failed = []
    ratio, error = moved["magnitude_ratio"], moved["direction_error_deg"]
    low, high = t["magnitude_ratio_range"]
    if ratio is None or not low <= ratio <= high:
        failed.append(f"magnitude_ratio {ratio} is outside {low:g} to {high:g}")
    if error is None or error >= t["direction_error_deg_below"]:
        failed.append(f"direction_error_deg {error} is not under {t['direction_error_deg_below']:g}")
    if end_overlap is None or end_overlap <= t["end_frame_overlap_above"]:
        failed.append(f"end_frame_overlap {end_overlap} is not above {t['end_frame_overlap_above']:g}")
    return {"verdict": "moved, but not like the clip" if failed else "matches", "failed": failed, "numbers": numbers}


def overall_verdict(by_object):
    """The least favourable per-object verdict (None for an empty mapping)."""
    verdicts = [entry["verdict"] for entry in by_object.values()]
    return min(verdicts, key=VERDICT_ORDER.index) if verdicts else None


# ---- tune_push: a bounded search over the pusher path --------------------------------------------
TUNE_DEFAULT_TRIALS = 6
TUNE_MAX_TRIALS = 12
TUNE_RATIO_OK = (0.8, 1.25)
TUNE_DIRECTION_OK_DEG = 15.0
TUNE_SCOPE = ("This tunes the PUSH PATH (the pusher's waypoints) so the block's image displacement matches the clip. "
              "It does not change mass, friction, size or any other scene value, and it matches displacement only, "
              "not the block's rotation.")


def image_axes(eye, target):
    """Orthographic image axes of a camera whose up is +z: ``((right_x, right_y), (down_x, down_y))``.

    A table vector ``(vx, vy)`` appears in the image as ``(dot(v, right), dot(v, down))`` (up to scale).
    """
    f = [t - e for t, e in zip(target, eye)]
    norm = math.sqrt(sum(c * c for c in f))
    f = [c / norm for c in f]
    right = [f[1], -f[0], 0.0]  # f x (0, 0, 1)
    scale = math.hypot(right[0], right[1])
    if scale < 1e-9:
        raise ValueError("The camera looks straight down the vertical; its image axes are undefined.")
    right = [c / scale for c in right]
    up = [right[1] * f[2] - right[2] * f[1], right[2] * f[0] - right[0] * f[2], right[0] * f[1] - right[1] * f[0]]  # right x f
    return (right[0], right[1]), (-up[0], -up[1])


def table_angle(image_vector, axes):
    """The direction of an image displacement on the table (radians, counter-clockwise from +x)."""
    (rx, ry), (dx, dy) = axes
    det = rx * dy - ry * dx
    vx = (image_vector[0] * dy - image_vector[1] * ry) / det
    vy = (rx * image_vector[1] - dx * image_vector[0]) / det
    return math.atan2(vy, vx)


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def transform_push(push, pivot, scale, rotation):
    """The push turned by ``rotation`` (radians) about ``pivot`` (the object's start), then stretched by ``scale``
    about its own first waypoint.

    Turning about the object keeps the pusher passing through it with the new heading. Stretching about the
    pusher's start keeps where the run begins and lengthens the travel past the object, which is what moves
    the object further (stretching about the object would leave the end of the push on the object).
    """
    c, s = math.cos(rotation), math.sin(rotation)
    turned = []
    for t, x, y in push["waypoints"]:
        dx, dy = x - pivot[0], y - pivot[1]
        turned.append([t, pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy])
    x0, y0 = turned[0][1], turned[0][2]
    return {"waypoints": [[t, round(x0 + scale * (x - x0), 5), round(y0 + scale * (y - y0), 5)] for t, x, y in turned],
            "object_velocity": push.get("object_velocity")}


def aimed_push(push, pivot, angle, reach=0.10, run_up=0.15):
    """A straight push through ``pivot`` heading along ``angle``: it starts ``run_up`` before the pivot and ends
    ``reach`` past it, over the same times as ``push``."""
    u = (math.cos(angle), math.sin(angle))
    times = [w[0] for w in push["waypoints"]]
    return {"waypoints": [[times[0], round(pivot[0] - run_up * u[0], 5), round(pivot[1] - run_up * u[1], 5)],
                          [times[-1], round(pivot[0] + reach * u[0], 5), round(pivot[1] + reach * u[1], 5)]],
            "object_velocity": push.get("object_velocity")}


def measure(clip_px, sim_px, axes):
    """How the simulated displacement compares with the clip's: ratio, direction errors, the cost to minimise."""
    clip_len, sim_len = math.hypot(*clip_px), math.hypot(*sim_px)
    out = {"sim_length_px": round(sim_len, 1), "clip_length_px": round(clip_len, 1), "magnitude_ratio": None,
           "direction_error_deg": None, "signed_error_rad": None, "moved": sim_len >= VERDICT_THRESHOLDS["did_not_move_below_px"]}
    if clip_len > 0:
        out["magnitude_ratio"] = round(sim_len / clip_len, 3)
    if out["moved"] and clip_len > 0:
        out["direction_error_deg"] = direction_error_deg(clip_px, sim_px)
        out["signed_error_rad"] = wrap_angle(table_angle(clip_px, axes) - table_angle(sim_px, axes))
        out["cost"] = abs(math.log(max(out["magnitude_ratio"], 1e-3))) + abs(out["signed_error_rad"])
    else:
        out["cost"] = 10.0
    return out


def within_tolerance(m):
    low, high = TUNE_RATIO_OK
    return bool(m["moved"] and m["magnitude_ratio"] is not None and low <= m["magnitude_ratio"] <= high
                and m["direction_error_deg"] is not None and m["direction_error_deg"] < TUNE_DIRECTION_OK_DEG)


def search_push(push, pivot, axes, evaluate, *, max_trials=TUNE_DEFAULT_TRIALS):
    """Bounded search over the pusher path; ``evaluate(push, trial)`` returns ``{"clip_px", "sim_px", ...}``.

    Trial 1 runs ``push`` as given. Each next trial stretches the path from the pusher's start by the measured length
    ratio (using the exponent seen so far, since displacement is not exactly proportional to travel) and
    turns it by the measured direction error. A trial that did not move re-aims a straight push through the
    pivot along the clip's direction. ``pivot`` is the object's start xy; an ``evaluate`` result's ``pivot`` replaces
    it. Stops at the first trial within tolerance, else after ``max_trials``. Returns ``(trials, best, stopped_because)``; every trial row keeps ``push`` and the evaluate extras.
    """
    if isinstance(max_trials, bool) or not isinstance(max_trials, int) or not 1 <= max_trials <= TUNE_MAX_TRIALS:
        raise ValueError(f"max_trials must be a whole number from 1 to {TUNE_MAX_TRIALS}.")
    base, scale, rotation = push, 1.0, 0.0
    trials, best, last = [], None, None
    for number in range(1, max_trials + 1):
        if (scale, rotation) == (1.0, 0.0):
            trial_push = base
        elif pivot is None:
            raise ValueError("The object's start position is unknown, so the push cannot be scaled about it.")
        else:
            trial_push = transform_push(base, pivot, scale, rotation)
        found = evaluate(trial_push, number)
        pivot = found.get("pivot") or pivot
        clip_px, sim_px = found["clip_px"], found["sim_px"]
        if math.hypot(*clip_px) < VERDICT_THRESHOLDS["did_not_move_below_px"]:
            raise ValueError("The clip's block moved less than 3 px between the first and last anchor: there is no displacement to match.")
        m = measure(clip_px, sim_px, axes)
        row = {"trial": number, "scale": round(scale, 3), "rotation_deg": round(math.degrees(rotation), 1), "push": trial_push,
               "sim_px": sim_px, "clip_px": clip_px, "magnitude_ratio": m["magnitude_ratio"],
               "direction_error_deg": m["direction_error_deg"], "cost": round(m["cost"], 3),
               **{k: v for k, v in found.items() if k not in ("clip_px", "sim_px", "pivot")}}
        trials.append(row)
        if best is None or m["cost"] < best[1]:
            best = (row, m["cost"])
        if found.get("pusher_tracked") is False:
            return trials, best[0], "pusher did not track"
        if within_tolerance(m):
            return trials, best[0], "within tolerance"
        if number == max_trials:
            break
        if pivot is None:
            raise ValueError("The object's start position is unknown, so the push cannot be changed about it.")
        if not m["moved"]:
            base = aimed_push(trial_push, pivot, table_angle(clip_px, axes))
            scale, rotation, last = 1.0, 0.0, None
            continue
        exponent = 1.0
        if last is not None and abs(math.log(scale / last[0])) > 1e-6 and last[1] > 0:
            exponent = min(3.0, max(0.3, math.log(m["sim_length_px"] / last[1]) / math.log(scale / last[0])))
        last = (scale, m["sim_length_px"])
        factor = (m["clip_length_px"] / m["sim_length_px"]) ** (1.0 / exponent)
        scale = min(6.0, max(0.1, scale * min(3.0, max(1 / 3, factor))))
        rotation += max(-math.radians(60), min(math.radians(60), m["signed_error_rad"]))
    return trials, best[0], "max_trials"
