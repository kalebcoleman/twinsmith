"""OmniGraph discovery, graph inspection, the Action Graph editor, and Nav2 occupancy maps.

Added after a fresh agent did a full ROS 2 lab with these tools and had to dig through Isaac Sim's
install to learn node type ids and port names ("Make Array" is ``omni.graph.nodes.ConstructArray``,
Break 3-Vector takes ``inputs:tuple``), drive private editor APIs to show a graph in a video, and
reverse-engineer the occupancy-map buffer layout to write a map for Nav2.

``pgm_and_origin`` is pure (numpy only) so the map layout can be unit-tested without Kit.
"""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path

PORT_TEXT = 160
MAX_DETAILED = 8


def _enable(extension):
    """Enable a Kit extension on demand (it is installed with Isaac Sim but not loaded by run.sh)."""
    import omni.kit.app

    em = omni.kit.app.get_app().get_extension_manager()
    if not em.is_extension_enabled(extension):
        em.set_extension_enabled_immediate(extension, True)
    return em.is_extension_enabled(extension)


def _short(value, limit=8):
    """A JSON-friendly, size-bounded copy of an attribute value."""
    try:
        import numpy as np

        if isinstance(value, np.ndarray):
            value = value.tolist()
    except ImportError:
        pass
    if isinstance(value, (list, tuple)):
        items = [_short(v, limit) for v in list(value)[:limit]]
        return items + ([f"... {len(value)} items"] if len(value) > limit else [])
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (int, bool, str)) or value is None:
        return value if not isinstance(value, str) else value[:200]
    return str(value)[:200]


def _text(value):
    text = " ".join(value) if isinstance(value, list) else str(value or "")
    return re.sub(r"\s+", " ", text).strip()[:PORT_TEXT]


def ogn_ports(ogn: dict, type_name: str) -> dict:
    """Inputs and outputs of *type_name* from a parsed .ogn file, with og-style names."""
    node = ogn.get(type_name) or next(iter(ogn.values()), {})
    out = {}
    for section in ("inputs", "outputs"):
        ports = {}
        for name, spec in (node.get(section) or {}).items():
            entry = {"type": spec.get("type")}
            if "default" in spec:
                entry["default"] = _short(spec["default"])
            allowed = allowed_values(spec)
            if allowed:
                entry["allowed_values"] = allowed
            if spec.get("optional"):
                entry["optional"] = True
            desc = _text(spec.get("description"))
            if desc:
                entry["description"] = desc
            ports[f"{section}:{name}"] = entry
        out[section] = ports
    return out


def allowed_values(spec):
    """Values a token input accepts, from an .ogn port spec. The stored value is the *display* string:
    Scale To/From Stage Units takes "Convert to stage units", not its key "toStage" (setting the key
    makes the node silently output zeros)."""
    tokens = spec.get("allowedTokens") or (spec.get("metadata") or {}).get("allowedTokens")
    if isinstance(tokens, dict):
        return [str(v) for v in tokens.values()]
    if isinstance(tokens, list):
        return [str(v) for v in tokens]
    if isinstance(tokens, str):
        return [t.strip() for t in tokens.split(",") if t.strip()]
    return []


_RST_NAME = re.compile(r"\(\*(?P<name>(?:inputs|outputs):[^*]+)\*\)\s*$")


def rst_ports(text: str) -> dict:
    """Ports from a generated OmniGraph node .rst page (C++ nodes ship no .ogn file)."""
    import csv

    out = {"inputs": {}, "outputs": {}}
    for line in text.splitlines():
        if "(*inputs:" not in line and "(*outputs:" not in line:
            continue
        try:  # csv-table rows: quotes inside cells are doubled ("")
            cells = next(csv.reader([line.strip()], skipinitialspace=True))
        except (csv.Error, StopIteration):
            continue
        if len(cells) < 4:
            continue
        m = _RST_NAME.search(cells[0])
        if not m or not (cells[1].startswith("``") and cells[1].endswith("``")):
            continue
        m = {"name": m["name"], "type": cells[1][2:-2], "desc": cells[2], "default": cells[3]}
        name, kind = m["name"], m["type"]
        if kind.startswith("["):  # extended type: any of a long list
            n = kind.count("'") // 2
            kind = f"any of {n} types (resolved when connected)" if n > 6 else kind.replace("'", "")
        entry = {"type": kind}
        if m["default"] not in ("", "None"):
            entry["default"] = m["default"][:120]
        if m["desc"]:
            entry["description"] = _text(m["desc"])
        out["inputs" if name.startswith("inputs:") else "outputs"][name] = entry
    return out


_OGN_INDEX = {}


def _find_ogn(ext_path: str, type_name: str):
    """Ports of *type_name* from its .ogn file (or, for compiled nodes, its generated .rst page)."""
    if not ext_path:
        return None
    short = type_name.rsplit(".", 1)[-1]
    index = _OGN_INDEX.get(ext_path)
    if index is None:
        index = {}
        for pattern in ("*.ogn", "*.rst"):
            for path in Path(ext_path).rglob(pattern):
                index.setdefault(path.name, path)  # OgnROS2PublishClock.ogn -> path
        _OGN_INDEX[ext_path] = index
    # Most ids drop the Ogn prefix (omni.graph.nodes.ConstructArray -> OgnConstructArray.ogn); Isaac's
    # often keep it (isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit -> OgnIsaacScaleToFromStageUnit.ogn).
    stems = [f"Ogn{short}"] + ([short] if short.startswith("Ogn") else [])
    try:
        for stem in stems:
            path = index.get(f"{stem}.ogn")
            if path is not None:
                text = path.read_text()
                return ogn_ports(json.loads(text), type_name), text
        for stem in stems:
            path = index.get(f"{stem}.rst")
            if path is not None:
                text = path.read_text()
                ports = rst_ports(text)
                if ports["inputs"] or ports["outputs"]:
                    return ports, text
    except (OSError, ValueError):
        return None
    return None


def node_types(contains, limit=20, ports=None):
    """Registered OmniGraph node types whose id or UI name contains *contains*."""
    import omni.graph.core as og
    import omni.kit.app

    needle = contains.strip().lower()
    em = omni.kit.app.get_app().get_extension_manager()
    found = []
    for name in sorted(og.get_registered_nodes()):
        nt = og.get_node_type(name)
        if not nt or not (nt.is_valid() if callable(getattr(nt, "is_valid", None)) else True):
            continue
        md = nt.get_all_metadata()
        ui = md.get("uiName", "")
        if needle in name.lower() or needle in ui.lower():
            found.append((name, nt, md))
    detailed = ports if ports is not None else len(found) <= 5
    result = []
    for i, (name, nt, md) in enumerate(found[:limit]):
        entry = {"type": name, "ui_name": md.get("uiName", ""), "extension": md.get("__extension", ""),
                 "description": _text(md.get("__description", ""))}
        dep = getattr(nt, "is_deprecated", False)
        if callable(dep):  # a method in some Kit versions, a property in others
            dep = dep()
        if dep:
            msg = getattr(nt, "deprecation_message", "")
            entry["deprecated"] = _text(msg() if callable(msg) else msg) or True
        if detailed and i < MAX_DETAILED:
            ext = md.get("__extension")
            ext_id = em.get_enabled_extension_id(ext) if ext else None
            found_ports = _find_ogn(em.get_extension_path(ext_id), name) if ext_id else None
            if found_ports:
                ports, source = found_ports
                entry.update(ports)
                if not ports.get("outputs") and any(n.endswith(":value") for n in ports.get("inputs", {})):
                    entry["constant_note"] = (
                        "No output port: this is a constant node, and its inputs:value is the value other nodes "
                        "read. Connect from '<node>.inputs:value' (e.g. Constant Token -> Make Array).")
                if "arraySize" in source:
                    entry["dynamic_inputs_note"] = (
                        "Only inputs:input0 exists at first. Setting inputs:arraySize from a script does not add "
                        "inputs:input1..; add each with og.Controller.create_attribute(node, 'inputs:input1', "
                        "'any', og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT), then connect it.")
            else:
                entry["ports"] = "not available (no .ogn file found); build one and use sim__graph__inspect"
        result.append(entry)
    return dict(
        matches=len(found),
        returned=len(result),
        node_types=result,
        usage=("Create nodes with og.Controller.edit(graph, {keys.CREATE_NODES: [(name, type)]}); connect with "
               "full paths, e.g. '<graph>/<node>.outputs:tick' -> '<graph>/<other>.inputs:execIn'."),
    )


def _graph_path(path):
    from twinsmith.project_assets import refuse

    from pxr import Sdf

    valid = Sdf.Path.IsValidPathString(path) if isinstance(path, str) else False
    if isinstance(valid, tuple):  # some USD versions return (False, message) for invalid paths
        valid = valid[0]
    if not bool(valid):
        refuse("INVALID_VALUE", "path must be an absolute prim path like /World/ROS_Clock.")
    p = Sdf.Path(path)
    if not (p.IsAbsolutePath() and p.IsPrimPath()):
        refuse("INVALID_VALUE", "path must be an absolute prim path like /World/ROS_Clock.")
    return path


def _headless():
    import carb.settings
    import sys

    settings = carb.settings.get_settings()
    return (settings.get("/app/window/hideUi") or settings.get("/app/window/enabled") is False
            or os.environ.get("TWINSMITH_HEADLESS") == "1" or "--no-window" in sys.argv)


def _allowed_tokens(attr):
    from .compat import metadata_keys

    try:
        raw = attr.get_metadata(metadata_keys().ALLOWED_TOKENS)
    except Exception:  # noqa: BLE001
        return []
    return [t for t in (raw or "").split(",") if t]


def _messages(node):
    """Errors and warnings from the node's last compute: the only sign of e.g. a bad token value."""
    import omni.graph.core as og

    out = {}
    for key, severity in (("errors", og.Severity.ERROR), ("warnings", og.Severity.WARNING)):
        try:
            msgs = [str(m)[:300] for m in node.get_compute_messages(severity)][:5]
        except Exception:  # noqa: BLE001
            msgs = []
        if msgs:
            out[key] = msgs
    return out


def inspect_graph(stage, path, limit=64):
    """Nodes of an OmniGraph with their types, input values and connections."""
    import omni.graph.core as og
    from twinsmith.project_assets import refuse

    graph = og.get_graph_by_path(_graph_path(path))
    if graph is None or not graph.is_valid():
        prim = stage.GetPrimAtPath(path)
        refuse("NOT_A_GRAPH", f"No OmniGraph at {path}" + (f" (prim type {prim.GetTypeName()})" if prim else "")
               + ". Find graphs with sim__prim__find type_name=OmniGraph.")
    nodes = list(graph.get_nodes())
    out = []
    for node in nodes[:limit]:
        inputs, outputs = {}, {}
        for attr in node.get_attributes():
            name = attr.get_name()
            if name.startswith("inputs:"):
                entry = {"type": attr.get_resolved_type().get_ogn_type_name()}
                upstream = [c.get_path() for c in attr.get_upstream_connections()]
                if upstream:
                    entry["from"] = upstream
                else:
                    try:
                        entry["value"] = _short(og.Controller.get(attr))
                    except Exception as exc:  # unresolved extended types have no value yet
                        entry["value"] = f"<unreadable: {type(exc).__name__}>"
                allowed = _allowed_tokens(attr)
                if allowed:
                    entry["allowed_values"] = allowed
                inputs[name] = entry
            elif name.startswith("outputs:"):
                downstream = [c.get_path() for c in attr.get_downstream_connections()]
                outputs[name] = {"type": attr.get_resolved_type().get_ogn_type_name(),
                                 **({"to": downstream} if downstream else {})}
        record = dict(path=node.get_prim_path(), type=node.get_type_name(), inputs=inputs, outputs=outputs)
        record.update(_messages(node))
        out.append(record)
    failing = [n["path"] for n in out if n.get("errors")]
    return dict(graph=path, evaluator=graph.get_evaluator_name(), node_count=len(nodes),
                truncated=len(nodes) > limit, nodes=out,
                **_ros_render_status(stage, nodes[:limit]),
                **_ros_domain_note(out),
                **({"nodes_with_errors": failing,
                    "note": "Compute errors come from the last time the graph ran (play the timeline first)."}
                   if failing else {}))


def _ros_domain_note(records, env=None):
    """A saved ROS2Context can override ROS_DOMAIN_ID, which is all sim__stage__info reports."""
    env_domain = str((os.environ if env is None else env).get("ROS_DOMAIN_ID", "0"))
    fixed = {}
    for record in records:
        if record["type"].rsplit(".", 1)[-1] != "ROS2Context":
            continue
        inputs = record["inputs"]
        domain = inputs.get("inputs:domain_id", {}).get("value")
        if inputs.get("inputs:useDomainIDEnvVar", {}).get("value") is False and str(domain) != env_domain:
            fixed[record["path"]] = domain
    if not fixed:
        return {}
    return dict(ros_domain_note=dict(
        authored_domains=fixed, ros_domain_id=env_domain,
        note="These ROS2Context nodes publish on their authored domain_id, not ROS_DOMAIN_ID (useDomainIDEnvVar is off). "
             "Use that domain in your ROS shell, or turn the option on."))


def _writer_computes(stage, product_path):
    """Compute count of the Replicator writer nodes attached to a product, or None when there are none."""
    import omni.graph.core as og

    pipeline = stage.GetPrimAtPath("/Render/PostProcess/SDGPipeline")
    if not pipeline or not product_path:
        return None
    prefix = product_path.rsplit("/", 1)[-1] + "_NodeWriterWriter"
    names = [c.GetName() for c in pipeline.GetChildren()
             if c.GetName() == prefix or c.GetName().startswith(prefix + "_")]
    if not names:
        return None
    return sum(og.Controller.node(f"{pipeline.GetPath()}/{name}").get_compute_count() for name in names)


def _ros_render_status(stage, nodes):
    """Result fields for ROS sensor helpers whose compute reports nothing when no frame ever arrives.

    A helper can look clean while its product has no texture and its writer never runs, so show the
    render facts Replicator's reuse path reads. Publisher count stays out of scope: see PUBLISHER_NOTE.
    """
    import omni.graph.core as og
    import omni.timeline

    from .render_export import HELPERS, PUBLISHER_NOTE, helper_render_status, retained_product_paths, writer_problem

    helpers = [node for node in nodes if node.get_type_name() in HELPERS]
    if not helpers:
        return {}
    retained = retained_product_paths()
    playing = omni.timeline.get_timeline_interface().is_playing()
    status = []
    for node in helpers:
        helper = stage.GetPrimAtPath(node.get_prim_path())
        try:
            runtime = og.Controller.get(node.get_attribute("inputs:renderProductPath"))
            entry = helper_render_status(stage, helper, runtime, retained, playing)
            entry["writer_computes"] = _writer_computes(stage, entry["render_product"])
            problem = writer_problem(entry, playing)
            if problem:
                entry["problem"] = problem
        except Exception as exc:  # noqa: BLE001 - diagnostics must never break inspection
            entry = dict(node=node.get_prim_path(), status="unknown", error=f"{type(exc).__name__}: {str(exc)[:200]}")
        status.append(entry)
    problems = [entry["node"] for entry in status if entry.get("problem")]
    return dict(ros_render_status=status, ros_publisher_note=PUBLISHER_NOTE,
                **({"ros_helpers_with_problems": problems} if problems else {}))


def _editor():
    import omni.graph.window.action.action_graph_extension as editor

    return editor


def editor_ready():
    """True once the Action Graph window and its graph widget exist (built on later frames)."""
    try:
        import omni.ui as ui

        window = ui.Workspace.get_window("Action Graph")
        ext = _editor()._extension_instance  # noqa: SLF001
        return bool(window is not None and window.visible and ext._window._main_widget is not None)  # noqa: SLF001
    except (ImportError, AttributeError):
        return False


def show_graph_open(stage, path, width=0, height=0):
    """Step 1 of sim.graph.show: open the editor on *path*. Returns True when the editor was
    already built, so no frames need to pass before step 2."""
    from twinsmith.project_assets import refuse

    if _headless():
        refuse("HEADLESS", "The Action Graph editor needs a windowed Isaac Sim: start Kit without --headless.")
    import omni.graph.core as og

    graph = og.get_graph_by_path(_graph_path(path))
    if graph is None or not graph.is_valid():
        refuse("NOT_A_GRAPH", f"No OmniGraph at {path}.")
    _enable("omni.graph.window.action")
    ready = editor_ready()
    try:
        _editor().ActionGraphExtension.show_graph([stage.GetPrimAtPath(path)])
    except (ImportError, AttributeError) as exc:
        _editor_changed(exc)
    return ready


def close_editor():
    """Hide the Action Graph editor window (Kit keeps it for the next show)."""
    import omni.ui as ui

    window = ui.Workspace.get_window("Action Graph")
    was_visible = bool(window is not None and window.visible)
    if window is not None:
        window.visible = False
    return dict(closed=was_visible, note="The editor window was hidden." if was_visible else "The editor was not open.")


def _place_window(window, width, height):
    """Undock/size on request, and keep any floating editor inside the main window: a Kit
    window floats inside the Isaac Sim window, so a stale size can push its close button off it."""
    import omni.ui as ui
    from .placement import clamp_window

    if window is None:
        return None
    if width and height:
        window.undock()
        w, h, x, y = float(width), float(height), 40.0, 60.0
    elif getattr(window, "docked", True):
        return None
    else:
        w, h, x, y = window.width, window.height, window.position_x, window.position_y
    main_w, main_h = ui.Workspace.get_main_window_width(), ui.Workspace.get_main_window_height()
    wanted = (w, h, x, y)
    w, h, x, y = clamp_window(w, h, x, y, main_w, main_h)
    window.width, window.height = w, h
    window.position_x, window.position_y = x, y
    return dict(width=int(w), height=int(h), x=int(x), y=int(y), main_window=[int(main_w), int(main_h)],
                clamped=any(abs(a - b) > 0.5 for a, b in zip(wanted, (w, h, x, y))))


def _needs_layout(stage, path):
    """Nodes without a stored editor position or expansion; re-arranging laid-out graphs costs
    editor rebuild frames (seconds on a loaded GPU) and moves nodes a person placed."""
    from pxr import Usd

    nodes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(path)) if p.GetTypeName() == "OmniGraphNode"]
    missing = [p for p in nodes if not (p.GetAttribute("ui:nodegraph:node:pos") and p.GetAttribute("ui:nodegraph:node:pos").HasAuthoredValue())]
    closed = [p for p in nodes if (p.GetAttribute("ui:nodegraph:node:expansionState").Get()
                                   if p.HasAttribute("ui:nodegraph:node:expansionState") else None) != "open"]
    return nodes, missing, closed


def store_positions(path):
    """The editor's auto-layout only updates its in-memory model (it writes
    ui:nodegraph:node:pos after a mouse drag). Store the positions in the stage so the layout
    survives reopening the editor or saving, and the next call can skip re-arranging.
    Returns the number of nodes stored."""
    from pxr import Gf, Sdf, Usd, UsdUI

    try:
        model = _editor()._extension_instance._actions._get_view()._model  # noqa: SLF001
    except (ImportError, AttributeError):
        return 0
    root, stored = Sdf.Path(path), 0
    for node in list(model.nodes):
        if not (isinstance(node, Usd.Prim) and node.IsValid() and node.GetPath().HasPrefix(root)):
            continue
        position = model[node].position
        if position is not None:
            UsdUI.NodeGraphNodeAPI.Apply(node).CreatePosAttr().Set(Gf.Vec2f(*position[:2]))
            stored += 1
    return stored


def show_graph_finish(stage, path, layout=False, width=0, height=0):
    """Step 2 of sim.graph.show: size the window, load the graph, lay out only what needs it."""
    laid_out = False
    try:
        import omni.ui as ui
        from pxr import Sdf

        window_info = _place_window(ui.Workspace.get_window("Action Graph"), width, height)
        nodes, missing, closed = _needs_layout(stage, path) if layout else ([], [], [])
        for prim in closed:  # expanded nodes show their port names in a video
            prim.CreateAttribute("ui:nodegraph:node:expansionState", Sdf.ValueTypeNames.Token).Set("open")
        ext = _editor()._extension_instance  # noqa: SLF001 - the editor has no public API for this
        ext._window._main_widget._open_graph(path)  # noqa: SLF001
        view = ext._actions._get_view()  # noqa: SLF001
        if layout and (missing or closed):
            view.layout_all()  # positions land in the editor model on later frames: store_positions
            laid_out = True
        view.focus_on_nodes()
    except (ImportError, AttributeError) as exc:
        _editor_changed(exc)
    result = dict(graph=path, shown=True, laid_out=laid_out,
                  note="Take a screenshot or record the window to capture it.")
    if layout and not laid_out:
        result["layout_skipped"] = "Every node already has a stored position and is expanded."
    if window_info:
        result["window"] = window_info
    return result


def show_graph_refocus():
    """Zoom to fit all nodes; returns the view state so the caller can wait until it settles
    (node sizes are known only after the editor has drawn them)."""
    try:
        view = _editor()._extension_instance._actions._get_view()  # noqa: SLF001
        view.focus_on_nodes()
        model = getattr(view, "_model", None)
        bbox = tuple(view.get_bbox_of_nodes(model.nodes)) if model is not None and model.nodes else None
        return (*bbox, round(view.zoom, 4), round(view.pan_x, 1), round(view.pan_y, 1)) if bbox else None
    except (ImportError, AttributeError):
        return None


def _editor_changed(exc):
    from twinsmith.project_assets import refuse

    refuse("EDITOR_API_CHANGED", f"This Isaac Sim's Action Graph editor API differs ({type(exc).__name__}: "
           f"{str(exc)[:200]}); open the graph from Window > Graph Editors > Action Graph instead.")


def pgm_and_origin(buffer, dims_x, dims_y, min_bound, cell_size, meters_per_unit, clear=()):
    """Nav2 PGM bytes and YAML origin from an Isaac occupancy-map buffer.

    The buffer is row-major, ``dims_x`` cells per row, with 1.0 = occupied and 0.0 = free; Isaac's
    occupancy-map window shows it rotated 180 degrees for ROS (its default "180" rotation), which
    puts +x to the right and +y up as map_server expects. The origin is the world position of the
    lower-left corner of the lower-left cell, in metres. ``clear`` circles ``[x, y, radius]``
    (metres) are marked free, e.g. the robot's own footprint at its start pose.
    """
    import numpy as np

    grid = np.asarray(buffer, dtype=np.float32).reshape(dims_y, dims_x)
    grid = np.rot90(grid, 2).copy()
    origin = (float(min_bound[0]) * meters_per_unit, float(min_bound[1]) * meters_per_unit)
    res = cell_size * meters_per_unit
    if len(clear):
        rows, cols = np.indices(grid.shape)
        wx = origin[0] + (cols + 0.5) * res
        wy = origin[1] + (grid.shape[0] - 1 - rows + 0.5) * res
        for cx, cy, r in clear:
            grid[(wx - cx) ** 2 + (wy - cy) ** 2 <= r * r] = 0.0
    pgm = np.full(grid.shape, 205, dtype=np.uint8)  # unknown
    pgm[grid == 0.0] = 254                          # free
    pgm[grid == 1.0] = 0                            # occupied
    header = f"P5\n# twinsmith sim__occupancy_map\n{dims_x} {dims_y}\n255\n".encode()
    return header + pgm.tobytes(), origin, dict(occupied=int((grid == 1.0).sum()), free=int((grid == 0.0).sum()),
                                                unknown=int(grid.size - (grid == 1.0).sum() - (grid == 0.0).sum()))


def _numbers(v, n):
    """A list of *n* finite numbers (bools are not numbers here)."""
    return isinstance(v, list) and len(v) == n and all(type(x) in (int, float) and math.isfinite(x) for x in v)


def validate_occupancy(scene, stage, a):
    """Check every argument with no side effects, so refusals happen before any Play."""
    from twinsmith.project_assets import refuse
    from pxr import UsdGeom

    lo, hi, z = a.get("min"), a.get("max"), a.get("z", [0.05, 0.3])
    for v, n in ((lo, "min"), (hi, "max"), (z, "z")):
        if not _numbers(v, 2):
            refuse("INVALID_VALUE", f"{n} must be two finite numbers (metres).")
    if not (hi[0] > lo[0] and hi[1] > lo[1] and z[1] > z[0]):
        refuse("INVALID_VALUE", "max must exceed min, and z[1] must exceed z[0].")
    cell = a.get("cell_size", 0.05)
    if type(cell) not in (int, float) or not 0.005 <= cell <= 1.0:
        refuse("INVALID_VALUE", "cell_size must be 0.005..1.0 m.")
    start = a.get("start") or [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2]
    if not (_numbers(start, 2) and lo[0] < start[0] < hi[0] and lo[1] < start[1] < hi[1]):
        refuse("INVALID_VALUE", "start must be [x, y] metres inside the bounds, in free space.")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        refuse("UNSUPPORTED_STAGE", "Occupancy maps need a Z-up stage (ROS convention).")
    if (hi[0] - lo[0]) * (hi[1] - lo[1]) / (cell * cell) > 16_000_000:
        refuse("LIMIT", "Map larger than 16 million cells; increase cell_size or shrink the bounds.")
    base = a.get("basename", "map")
    if not isinstance(base, str) or not re.fullmatch(r"[A-Za-z0-9_\-]{1,60}", base):
        refuse("INVALID_OUTPUT", "basename: letters, digits, _ and - only (<=60), no extension.")
    clear = a.get("clear") or []
    if not (isinstance(clear, list) and len(clear) <= 16 and all(_numbers(c, 3) and c[2] > 0 for c in clear)):
        refuse("INVALID_VALUE", "clear: up to 16 [x, y, radius] circles in metres.")
    pgm_path, yaml_path = scene.output / f"{base}.pgm", scene.output / f"{base}.yaml"
    for p in (pgm_path, yaml_path):
        if p.exists():
            refuse("CONFLICT", f"{p} exists; choose another basename.")
    return dict(lo=lo, hi=hi, z=z, cell=float(cell), start=start, base=base, clear=clear,
                pgm_path=pgm_path, yaml_path=yaml_path)


def cell_value(pgm, width, height, origin, res, x, y):
    """PGM value (0 occupied, 254 free, 205 unknown) of the cell containing world (x, y)."""
    col = int(math.floor((x - origin[0]) / res))
    row = height - 1 - int(math.floor((y - origin[1]) / res))
    if not (0 <= col < width and 0 <= row < height):
        return None
    body = pgm[len(pgm) - width * height:]
    return body[row * width + col]


def cells_at(pgm, width, height, origin, res, x, y):
    """PGM values of the cells touching world (x, y): one cell, or up to four when the point lies
    on a cell edge (floating-point floor could pick either side)."""
    eps = res * 1e-6
    cols = {int(math.floor((x - origin[0] + d) / res)) for d in (-eps, eps)}
    rows = {height - 1 - int(math.floor((y - origin[1] + d) / res)) for d in (-eps, eps)}
    body = pgm[len(pgm) - width * height:]
    return [body[r * width + c] for r in rows for c in cols if 0 <= c < width and 0 <= r < height]


def start_blocked(counts, start_cells, start_in_collider=False, start_occupied=False):
    """Why the flood-fill start makes the map unusable, or None. Pure, so it is unit-tested."""
    total = max(1, sum(counts.values()))
    if start_in_collider:
        return "is inside a collider (PhysX overlap at the start point)"
    if start_occupied and counts["occupied"]:
        # The fill erased whatever touches the start and left the rest: a wall with a hole looks like a good map.
        return "is on an occupied cell in this z slice, so the obstacle under it would be mapped as free"
    if counts["occupied"] == 0 and counts["unknown"] > 0.5 * total:
        # Seen live: with start inside a solid collider the generator marks that collider's
        # interior free, finds no walls, and leaves everything else unknown.
        return "seems to be inside an obstacle or a closed space: no walls were found and most of the map is unknown"
    if 254 not in start_cells:
        return "is occupied" if 0 in start_cells else "is not in mapped free space"
    return None


def _start_in_collider(stage, start, z_mid, cell, to_su):
    """PhysX overlap test at the start point (physics is playing during generation)."""
    try:
        from omni.physx import get_physx_scene_query_interface

        hits = []
        get_physx_scene_query_interface().overlap_sphere(
            cell * 0.25 * to_su, (start[0] * to_su, start[1] * to_su, z_mid * to_su),
            lambda hit: hits.append(hit.rigid_body or hit.collision) or False, False)
        return bool(hits)
    except Exception:  # no PhysX scene query available: fall back to the map-based checks
        return False


def _colliders_in_bounds(lo, hi, z, to_su, limit=5):
    """Colliders PhysX finds in the mapped volume. Unlike a bounding box, this tests a mesh room's walls, not the
    space they enclose, so the inside of a large room counts as empty. Physics is playing during generation."""
    try:
        from omni.physx import get_physx_scene_query_interface

        half = tuple((high - low) * 0.5 * to_su for low, high in zip((*lo, z[0]), (*hi, z[1])))
        center = tuple((high + low) * 0.5 * to_su for low, high in zip((*lo, z[0]), (*hi, z[1])))
        found = set()

        def report(hit):
            found.add(str(hit.collision or hit.rigid_body))
            return len(found) < limit

        get_physx_scene_query_interface().overlap_box(half, center, (0.0, 0.0, 0.0, 1.0), report, False)
        return sorted(found)
    except Exception:  # noqa: BLE001 - no scene query available: nothing proves the map is wrong
        return []


def _omap_before_2_3_3():
    """True for omap 2.2.1, 2.3.1 and 2.3.2 (Isaac Sim 6.0.0.0, 6.0.0.1, 6.0.1); False if the version is unreadable."""
    try:
        import omni.kit.app

        em = omni.kit.app.get_app().get_extension_manager()
        version = em.get_extension_dict(em.get_enabled_extension_id("isaacsim.asset.gen.omap")).get("package", {}).get("version")
        return tuple(int(x) for x in str(version).split(".")[:3]) < (2, 3, 3)
    except (ImportError, AttributeError, TypeError, ValueError):
        return False


def buffer_from_occupied(points, min_xy, max_xy, cell, start_xy):
    """The buffer MapGenerator::getBuffer() means to return, built without calling it, because calling it can crash Kit.
    Native getBuffer sizes the grid by truncating (max - min) / cell in float32, so it can be a cell short (118 for 119
    rows), then writes every occupied leaf with no bounds check: a leaf on the last row overruns the heap block and Kit
    aborts in malloc. Measured on 6.0.0.0 through 6.2, and the code is the same in omap 2.2.1 to 2.3.5. Layout matches
    native: x descending, y ascending, 1.0 occupied, 0.0 free, 0.5 unknown. Like native, the flood fill replaces whatever
    value the start cell holds, so the fourth value says whether that erased the occupied cells joined to it."""
    width = int(round((max_xy[0] - min_xy[0]) / cell))
    height = int(round((max_xy[1] - min_xy[1]) / cell))
    grid = [0.5] * (width * height)

    def index(x, y):
        col = min(width - 1, max(0, int((max_xy[0] - x) / cell)))
        return min(height - 1, max(0, int((y - min_xy[1]) / cell))) * width + col

    for x, y in points:
        grid[index(x, y)] = 1.0
    first = index(*start_xy)
    target, stack = grid[first], [first]
    start_occupied = target == 1.0
    if target != 0.0:
        while stack:
            i = stack.pop()
            if grid[i] != target:
                continue
            grid[i] = 0.0
            if i % width:
                stack.append(i - 1)
            if i % width != width - 1:
                stack.append(i + 1)
            if i >= width:
                stack.append(i - width)
            if i + width < len(grid):
                stack.append(i + width)
    return grid, width, height, start_occupied


FLOOR_RETRIES = 3


def raised_slice(z_low, z_high, cell, occupied, attempt):
    """The next bottom of the z slice to try, or None to stop. A floor in the slice marks every cell occupied, and the flood fill
    from start then turns them all free (measured on 6.0.1: floor top 0, cell 0.05, z_min 0.05 fails and 0.06 works)."""
    if occupied or attempt >= FLOOR_RETRIES:
        return None
    raised = z_low + cell * 1.2
    return raised if raised < z_high else None


def slice_bottom_note(cell):
    """What a raised slice bottom does and does not exclude, worded for every omap version we have seen."""
    return ("Bottom of the slice raised to z_used, because a floor within one cell of it fills the slice. "
            "omap releases before 2.3.3 can still include colliders below z_used, within about one cell "
            f"({cell:g} m); on later releases the slice starts at z_used.")


def occupancy_map(scene, stage, a):
    """Generate a 2D occupancy map from the stage's colliders and write a Nav2 PGM + YAML."""
    from twinsmith.project_assets import refuse
    from pxr import UsdGeom

    v = validate_occupancy(scene, stage, a)
    lo, hi, z, cell, start, base, clear = (v[k] for k in ("lo", "hi", "z", "cell", "start", "base", "clear"))
    pgm_path, yaml_path = v["pgm_path"], v["yaml_path"]
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    to_su = 1.0 / mpu
    if not _enable("isaacsim.asset.gen.omap"):
        refuse("EXTENSION_UNAVAILABLE", "isaacsim.asset.gen.omap could not be enabled in this Isaac Sim.")
    from isaacsim.asset.gen.omap.bindings import _omap

    from .newton_support import engine_name

    newton = engine_name() == "newton"  # the two collider probes below are PhysX scene queries: they do not run under Newton
    in_collider = False if newton else _start_in_collider(stage, start, (z[0] + z[1]) / 2, cell, to_su)
    om = _omap.acquire_omap_interface()
    # Before omap 2.3.3 a collider within one cell below a positive z_min still hits: a floor at z=0 fills the default slice.
    z_low = min(z[0] + 1.01 * cell, (z[0] + z[1]) / 2) if _omap_before_2_3_3() and z[0] > 0 else z[0]
    try:
        for attempt in range(FLOOR_RETRIES + 1):
            # The generator flood-fills free space from this origin ("must be in unoccupied space").
            cz = (z_low + z[1]) / 2 * to_su
            cx, cy = start[0] * to_su, start[1] * to_su
            om.set_cell_size(cell * to_su)
            om.set_transform((cx, cy, cz), (lo[0] * to_su - cx, lo[1] * to_su - cy, z_low * to_su - cz),
                             (hi[0] * to_su - cx, hi[1] * to_su - cy, z[1] * to_su - cz))
            om.update()
            om.generate()
            min_bound, top = om.get_min_bound(), om.get_max_bound()
            buffer, width, height, start_occupied = buffer_from_occupied(
                [(p.x, p.y) for p in om.get_occupied_positions()], (min_bound[0], min_bound[1]), (top[0], top[1]),
                cell * to_su, (cx, cy))
            if not len(buffer) or width * height != len(buffer):
                refuse("MAP_EMPTY", "The occupancy map came back empty. It needs colliders (UsdPhysics.CollisionAPI) "
                       "inside the bounds and a UsdPhysics.Scene on the stage.")
            pgm, origin, counts = pgm_and_origin(buffer, width, height, min_bound, cell * to_su, mpu, clear)
            next_low = raised_slice(z_low, z[1], cell, counts["occupied"] and not start_occupied, attempt)
            if next_low is None or newton or not _colliders_in_bounds(lo, hi, (z_low, z[1]), to_su):
                break
            z_low = next_low
    finally:
        _omap.release_omap_interface(om)
    reason = start_blocked(counts, cells_at(pgm, width, height, origin, cell, start[0], start[1]), in_collider, start_occupied)
    if reason:
        refuse("START_BLOCKED", f"start {start} {reason}. The generator flood-fills free space from it, so the "
               "map would be empty or wrong. Move start to open floor next to (not on) the robot; `clear` only "
               "edits the saved image. No files were written.", cells=counts)
    colliders = _colliders_in_bounds(lo, hi, (z_low, z[1]), to_su) if counts["occupied"] == 0 and not newton else []
    if colliders:
        refuse("MAP_NO_OBSTACLES", f"The generator marked no cell occupied, but colliders are inside the bounds and z "
               f"slice ({', '.join(colliders)}), so this map would be wrong. Raising the slice's bottom by up to "
               f"{FLOOR_RETRIES} steps did not help. If a floor is one of them, set z above it; check that the others are "
               "enabled and that physics has run since they were added. No files were written.", colliders=colliders, cells=counts)
    scene.output.mkdir(mode=0o700, parents=True, exist_ok=True)
    def publish(path, data):  # never a partial file at the final name, if Kit dies here
        temp = path.with_name(f".{path.name}.tmp")
        temp.unlink(missing_ok=True)  # left by a crash, or a symlink someone planted
        descriptor = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
        os.replace(temp, path)

    publish(pgm_path, pgm)
    yaml_text = (f"image: {base}.pgm\nmode: trinary\nresolution: {cell:g}\n"
                 f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\nnegate: 0\n"
                 "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    publish(yaml_path, yaml_text.encode())  # last: a map with its YAML is a complete map
    total = max(1, sum(counts.values()))
    if counts["unknown"] > 0.5 * total:
        warn = (f"{counts['unknown'] * 100 // total}% of the map is unknown (not reachable from start "
                f"{start}): move start into the main free area, or the bounds into one room.")
    elif counts["occupied"] == 0:
        warn = "No occupied cells reached from start: nothing with a collider intersects the z slice there."
    else:
        warn = None
    raised = {"z_used": [round(z_low, 4), z[1]], "z_note": slice_bottom_note(cell)} if z_low != z[0] else {}
    skipped = dict(backend="newton", checks_skipped=[
        "start inside a collider (PhysX overlap query)", "colliders inside the mapped volume (PhysX overlap query)",
        "raising the slice bottom to clear a floor, which depends on that second check"],
        checks_note="Not run under Newton, which has no overlap query here: a start point inside a collider is caught only by the "
                    "map-based checks (no walls found, start cell occupied or not free). The map generator itself is native "
                    "(isaacsim.asset.gen.omap) and was not verified under Newton, so inspect the saved map.") if newton else {}
    return dict(pgm=str(pgm_path), **skipped, yaml=str(yaml_path), width=width, height=height, resolution=cell, **raised,
                origin=[round(origin[0], 4), round(origin[1], 4), 0.0], cells=counts, yaml_text=yaml_text,
                **({"warning": warn} if warn else {}),
                usage="ros2 launch nav2_bringup localization_launch.py map:=<yaml> use_sim_time:=true")
