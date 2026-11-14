"""Per-anchor mask readings for a recorded take, in RSS's own terms. Runs in RSS's interpreter, never the server's.

RSS's ``compare --take`` (``robosimstudio_real2sim_agent/checks.py`` ``compare_take`` at the pin) returns, per
anchor, only which frame of the take it was held against: its keys only outline things on the strip. The bridge
needs what a still ``compare`` gives for each keyed thing (both sides' centre and size, their overlap), so this
script reads the take's segmentation and each anchor's photograph and calls the same RSS functions a still compare
uses (``photo_masks``, ``fit_mask``, ``sim_masks``, ``compare_masks``). It writes nothing.

    python -I take_scores.py <request.json>

``request.json`` (written by the bridge): ``run_dir``, ``take``, ``keys`` (``name=colour`` strings) and
``anchors`` (each ``{"file", "frame"}`` from RSS's reply). It prints ``{"ok": true, "anchors": [{"frame", "objects"}]}``.
"""

import json
import sys
from pathlib import Path

import numpy as np
from robosimstudio_real2sim_agent import compare as C
from robosimstudio_real2sim_agent.checks import _plain, load_take, photo_masks


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    run_dir = Path(request["run_dir"])
    meta, segs = load_take(run_dir, request["take"])
    if segs is None:
        raise SystemExit(f"take {request['take']!r} has no segmentation ({request['take']}_seg.npz); record it again")
    names = {int(k): v for k, v in meta["names"].items()}
    width, height = int(meta["width"]), int(meta["height"])
    keys = [C.parse_key(text) for text in request["keys"]]
    for key, text in zip(keys, request["keys"]):
        # RSS's own compare_take leaves these off the clip's side: they key the one still they were drawn on
        if key.seed is not None or key.mask:
            raise SystemExit(f"key {text!r} is a seed or a mask, which marks one still only, so it cannot follow a thing "
                             "across a take's anchors. Use a colour key (name=colour) for compare_take and tune_push.")
    rows = []
    for anchor in request["anchors"]:
        frame = max(int(anchor["frame"]), 0)
        photo_path = Path(anchor["file"])
        photo = C.load_photo(photo_path if photo_path.is_absolute() else run_dir / "ref" / photo_path)
        found = photo_masks(photo, keys)
        # as RSS: a frame past the stored segmentation has no sim mask (the video read alone is clamped)
        masks = (C.sim_masks(np.asarray(segs[frame]).astype(np.int32), names, [key.name for key in keys])
                 if frame < len(segs) else {key.name: None for key in keys})
        objects = {}
        for key in keys:
            photo_mask = found[key.name]
            photo_mask = C.fit_mask(photo_mask, width, height) if photo_mask is not None else None
            sim_mask = masks[key.name]        # a take's segmentation may be stored smaller than its render
            sim_mask = C.fit_mask(sim_mask, width, height) if sim_mask is not None else None
            objects[key.name] = C.compare_masks(photo_mask, sim_mask)
        rows.append({"frame": frame, "objects": objects})
    print(json.dumps({"ok": True, "anchors": rows}, default=_plain))


if __name__ == "__main__":
    main()
