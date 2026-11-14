"""Kit-native UI computer use: find Isaac Sim widgets by name and drive them through Kit's input.

Why: on a machine that is used remotely, the X11 session is often locked. The lock screen holds the X
keyboard and pointer grabs, so X11 input (the computer tool) would reach the lock screen,
not Isaac Sim, and monitor screenshots are black. Kit's own input provider (``carb.input``, the
path ``omni.kit.ui_test`` uses) feeds events straight into the app, so these tools keep working
while the desktop is locked, and window screenshots (XComposite) still show the result.

The design follows OpenAI's computer-use action set (click, double_click, drag, move, scroll,
keypress, type, wait), plus two things a widget tree makes possible: targeting a widget by its
reference from ``sim.ui.find`` instead of guessing pixels, and Kit menu paths.

Coordinates are pixels in the Isaac Sim main window, the same pixels as
``screenshot(window_id=<the Isaac Sim window>)``. Kit widget geometry is in UI points;
pixels = points * dpi scale. ``matches``/``widget_record``/``parse_keys`` are pure and unit-tested.
"""

from __future__ import annotations

INTERACTIVE = {
    "Button", "ToolButton", "RadioButton", "CheckBox", "ComboBox", "StringField", "FloatField",
    "IntField", "FloatDrag", "IntDrag", "FloatSlider", "IntSlider", "MultiFloatField", "MultiIntField",
    "MultiFloatDragField", "MultiIntDragField", "TreeView", "CollapsableFrame", "Image",
}  # no MenuItem: closed menus still report visible items; menus go through action=menu
TEXTUAL = {"Label"}
MAX_TEXT = 500
MAX_WIDGETS_PER_WINDOW = 6000
ACTIONS = ("click", "double_click", "right_click", "move", "drag", "scroll", "type", "keypress", "menu", "tab", "close",
           "wait")
POINTER_ACTIONS = ("click", "double_click", "right_click", "move", "drag", "scroll")
# The arguments one action takes; in a batch each step carries its own, and none may be given at the top level.
STEP_FIELDS = ("ref", "x", "y", "end_x", "end_y", "text", "keys", "window", "menu", "scroll_x", "scroll_y")
MAX_BATCH = 10
MAX_BATCH_FRAMES = 600  # a whole batch waits at most this many frames, well inside the bridge's 300 s timeout

# Key names (OpenAI-style, case-insensitive) -> carb.input.KeyboardInput attribute names.
KEY_NAMES = {
    "enter": "ENTER", "return": "ENTER", "tab": "TAB", "escape": "ESCAPE", "esc": "ESCAPE",
    "backspace": "BACKSPACE", "delete": "DEL", "del": "DEL", "insert": "INSERT", "space": "SPACE",
    "left": "LEFT", "right": "RIGHT", "up": "UP", "down": "DOWN", "home": "HOME", "end": "END",
    "pageup": "PAGE_UP", "pagedown": "PAGE_DOWN", "minus": "MINUS", "equal": "EQUAL", "comma": "COMMA",
    "period": "PERIOD", "slash": "SLASH", "semicolon": "SEMICOLON", "apostrophe": "APOSTROPHE",
    "backslash": "BACKSLASH", "left_bracket": "LEFT_BRACKET", "right_bracket": "RIGHT_BRACKET",
    "grave": "GRAVE_ACCENT", **{f"f{i}": f"F{i}" for i in range(1, 13)},
}
MODIFIERS = {"ctrl": "CONTROL", "control": "CONTROL", "shift": "SHIFT", "alt": "ALT", "super": "SUPER",
             "meta": "SUPER", "cmd": "SUPER"}
MODIFIER_KEYS = {"CONTROL": "LEFT_CONTROL", "SHIFT": "LEFT_SHIFT", "ALT": "LEFT_ALT", "SUPER": "LEFT_SUPER"}


class UIRefusal(ValueError):
    """``effects`` overrides what the bridge says about input already sent; ``details`` join the error envelope."""

    def __init__(self, code, message, effects=None, **details):
        super().__init__(message)
        self.code, self.effects, self.details = code, effects, details


# Refusals raised before the first event: the action sent nothing.
NO_INPUT_CODES = ("INVALID_VALUE", "LIMIT", "STALE_WIDGET", "NOT_VISIBLE", "OUT_OF_WINDOW", "NO_UI", "DISABLED",
                  "NO_WINDOW")


def refusal(name, exc):
    """The bridge's error envelope for a UIRefusal from sim.ui.find or sim.ui.act. A failed batch step
    says itself what the steps before it sent (exc.effects)."""
    sent_nothing = name == "sim.ui.find" or exc.code in NO_INPUT_CODES
    return dict(status="error", code=exc.code, message=str(exc), **exc.details, effects=exc.effects or (
        "No input was sent." if sent_nothing else "Inspect the UI before retrying."))


def parse_keys(keys):
    """``["ctrl", "s"]`` -> (["CONTROL"], "S"). One non-modifier key, any modifiers, OpenAI-style names."""
    if not isinstance(keys, list) or not 1 <= len(keys) <= 5 or not all(isinstance(k, str) and k for k in keys):
        raise UIRefusal("INVALID_VALUE", "keys: 1..5 key names, e.g. [\"ctrl\", \"s\"] or [\"enter\"].")
    mods, main = [], []
    for raw in keys:
        k = raw.strip().lower().replace(" ", "_")
        if k in MODIFIERS:
            if MODIFIERS[k] in mods:
                raise UIRefusal("INVALID_VALUE", f"modifier {raw!r} given twice.")
            mods.append(MODIFIERS[k])
        elif k in KEY_NAMES:
            main.append(KEY_NAMES[k])
        elif len(k) == 1 and k.isascii() and k.isalnum():
            main.append(k.upper() if k.isalpha() else f"KEY_{k}")
        else:
            raise UIRefusal("INVALID_VALUE", f"unknown key {raw!r}; use letters, digits, enter, tab, escape, "
                                             "backspace, delete, arrows, home/end, pageup/pagedown, f1..f12, "
                                             "space or punctuation names, plus ctrl/shift/alt/super.")
    if len(main) != 1:
        raise UIRefusal("INVALID_VALUE", "keys must contain exactly one non-modifier key.")
    return mods, main[0]


def menu_path(path):
    """Validated menu path: "Layouts" (a top-level menu, opened to show its items) or "Window/Console".
    Surrounding spaces of each level are dropped; empty levels ("Window//X", "/Edit") are refused."""
    if not isinstance(path, str) or not 1 <= len(path) <= 200:
        raise UIRefusal("INVALID_VALUE", "menu: a menu path such as \"Window/Graph Editors/Action Graph\" or a "
                                         "top-level menu such as \"Layouts\".")
    parts = [p.strip() for p in path.split("/")]
    if not all(parts):
        raise UIRefusal("INVALID_VALUE", f"menu {path!r} has an empty level; separate levels with single '/', "
                                         "e.g. \"Window/Console\".")
    return "/".join(parts)


def window_title(title, action):
    if not isinstance(title, str) or not title.strip() or len(title) > 120:
        raise UIRefusal("INVALID_VALUE", f"window: for action={action}, the title of a Kit window or tab, e.g. 'Console'.")
    return title


def matches(record, query):
    """Case-insensitive substring match on the text a person or agent would use to name a widget."""
    if not query:
        return True
    q = query.lower()
    return any(q in str(record.get(k) if record.get(k) is not None else "").lower()
               for k in ("name", "text", "tooltip", "identifier", "type", "value"))


def widget_record(type_name, ref, window, name, text, tooltip, value, center, size, dpi, enabled=True,
                  identifier=""):
    """A JSON-friendly widget entry in window pixels."""
    cx, cy = center
    w, h = size
    entry = dict(ref=ref, type=type_name, window=window, x=round(cx * dpi), y=round(cy * dpi),
                 width=round(w * dpi), height=round(h * dpi))
    for key, val in (("name", name), ("identifier", identifier), ("text", text), ("tooltip", tooltip)):
        if val:
            entry[key] = str(val)[:120]
    if value is not None:
        entry["value"] = value if isinstance(value, (bool, int, float)) else str(value)[:120]
    if not enabled:
        entry["enabled"] = False
    return entry


# ---------------------------------------------------------------- Kit side (needs omni.ui / carb)

def _ui_test():
    import omni.kit.app

    em = omni.kit.app.get_app().get_extension_manager()
    if not em.is_extension_enabled("omni.kit.ui_test"):
        em.set_extension_enabled_immediate("omni.kit.ui_test", True)
    import omni.kit.ui_test as ui_test

    return ui_test


def _value(widget):
    model = getattr(widget, "model", None)
    if model is None:
        return None
    try:
        type_name = type(widget).__name__
        if type_name == "ComboBox":
            index = model.get_item_value_model().as_int
            items = model.get_item_children()
            if 0 <= index < len(items):
                return model.get_item_value_model(items[index]).as_string
            return index
        if type_name == "CheckBox":
            return model.as_bool
        if type_name in ("FloatField", "FloatDrag", "FloatSlider"):
            return round(model.as_float, 6)
        if type_name in ("IntField", "IntDrag", "IntSlider"):
            return model.as_int
        if type_name == "StringField":
            return model.as_string
    except Exception:  # noqa: BLE001 - models differ; a value is a convenience, not a contract
        return None
    return None


def hidden_tabs():
    """Titles of docked windows behind another tab (Console behind Content): open with action=tab."""
    import omni.ui as ui

    out = []
    for win in ui.Workspace.get_windows():
        try:
            if win.visible and win.title and win.docked and not win.is_selected_in_dock():
                out.append(win.title)
        except Exception:  # noqa: BLE001
            continue
    return out


def windows():
    """Visible Kit windows with their pixel rectangles inside the main window."""
    import omni.ui as ui

    dpi = ui.Workspace.get_dpi_scale()
    out = []
    for win in ui.Workspace.get_windows():
        if not win.visible or not win.title or win.title == BADGE_TITLE:
            continue
        try:  # a docked window behind another tab reports visible but cannot be clicked
            if win.docked and not win.is_selected_in_dock():
                continue
        except Exception:  # noqa: BLE001
            pass
        try:
            x, y, w, h = win.position_x, win.position_y, win.width, win.height
        except Exception:  # noqa: BLE001
            continue
        out.append(dict(title=win.title, x=round(x * dpi), y=round(y * dpi), width=round(w * dpi),
                        height=round(h * dpi), docked=bool(getattr(win, "docked", False))))
    return out


def fingerprint(type_name, identifier="", name="", text="", context=""):
    """What a ref must still point at: Kit paths are positional (".../Button[2]"), so after the UI is
    rebuilt (another prim in Property, a moved Stage row) the same path can name a different widget.
    Unnamed fields (Property values) use the nearest row label as context ("@Translate")."""
    label = identifier or name or ("" if type_name == "StringField" else str(text or "")[:40])
    if not label and context:
        label = "@" + str(context)[:40]
    return f"{type_name}:{label}".replace("|", "/")


def split_ref(ref):
    """(path, fingerprint or None) from a sim.ui.find ref ("<kit path>|<type>:<label>")."""
    path, sep, fp = ref.rpartition("|")
    if not sep or ":" not in fp or "//" not in path:
        return ref, None
    return path, fp


def inside(point, rect):
    x, y = point
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def _ancestors(path):
    """Paths of a widget's ancestors, outermost first, that Kit can query. A window's bare "Title/" or
    "Title//Frame" matches nothing and logs a warning that lands in the status bar and in recordings."""
    title, sep, rest = path.partition("//")
    segments = rest.split("/")
    if not sep:
        return []
    return [f"{title}//{'/'.join(segments[:i])}" for i in range(2, len(segments))]


def _context(path, ui_test):
    """Text of the nearest Label beside the widget (up to three levels up), e.g. a Property row name."""
    import omni.ui as ui

    for ancestor in _ancestors(path)[:-4:-1]:
        try:
            parent = ui_test.find(ancestor).widget
        except Exception:  # noqa: BLE001
            continue
        for child in ui.Inspector.get_children(parent):
            text = getattr(child, "text", "") if type(child).__name__ == "Label" else ""
            if text:
                return text
    return ""


def _bound_attribute(widget):
    """'/Cube.xformOp:translate#1' for a Property field: the USD attribute (and vector channel) it edits.
    Property fields share generic identifiers ("value"), so this is what tells rows apart."""
    model = getattr(widget, "model", None)
    getter = getattr(model, "get_attribute_paths", None)
    if not callable(getter):
        return ""
    try:
        paths = [str(p) for p in getter()]
    except Exception:  # noqa: BLE001
        return ""
    if not paths:
        return ""
    channel = getattr(model, "_channel_index", None)
    return ",".join(paths)[:120] + (f"#{channel}" if isinstance(channel, int) else "")


def _widget_fingerprint(widget, path, ui_test):
    type_name = type(widget).__name__
    fp = fingerprint(type_name, getattr(widget, "identifier", ""), getattr(widget, "name", ""),
                     getattr(widget, "text", ""))
    bound = _bound_attribute(widget)
    if bound:
        return f"{fp}@{bound}".replace("|", "/")
    if fp.endswith(":"):  # unnamed: add the nearest label so another row's field does not match
        fp = fingerprint(type_name, context=_context(path, ui_test))
    return fp


def _shown(path, center, ui_test, cache=None):
    """None when the centre point (UI points) is really clickable, else why not: inside the owning
    Kit window, under no hidden container (a panel keeps the rows of a view it has switched away from),
    inside every enclosing ScrollingFrame, and not under a floating Kit window. ``cache`` (a dict) keeps
    what was read about each container for the widgets of one find."""
    import omni.ui as ui

    title = path.split("//", 1)[0]
    win = ui.Workspace.get_window(title)
    if win is None:
        return "its window is closed"
    if not inside(center, (win.position_x, win.position_y, win.width, win.height)):
        return f"it is outside the visible part of the '{title}' window"
    parts = path.split("/")
    cache = {} if cache is None else cache
    for i in range(3, len(parts) - 1):  # every container between the window's root frame and the widget
        prefix = "/".join(parts[: i + 1])
        if prefix not in cache:
            try:
                frame = ui_test.find(prefix)
                pos, size = frame.position, frame.size
                cache[prefix] = (bool(getattr(frame.widget, "visible", True)), (pos.x, pos.y, size.x, size.y))
            except Exception:  # noqa: BLE001
                cache[prefix] = None
        scrolling = parts[i].startswith("ScrollingFrame[")
        if cache[prefix] is None:
            if scrolling:
                return "its scrolling panel could not be measured"
            continue
        visible, rect = cache[prefix]
        if not visible:
            return "it is inside a hidden part of its panel"
        if scrolling and not inside(center, rect):
            return "it is scrolled out of view; scroll its panel first"
    if getattr(win, "docked", False):
        for other in ui.Workspace.get_windows():
            if (other is not win and other.visible and not getattr(other, "docked", False) and other.title
                    and other.title not in ("Main ToolBar", "Status Bar", BADGE_TITLE)
                    and inside(center, (other.position_x, other.position_y, other.width, other.height))):
                return f"the floating window '{other.title}' covers it"
    return None


def _candidates(window_filter, include_labels, query):
    """Cheap pass: (title, ref, widget, type) that match, before computing paths (each realpath
    walks the window tree again, so it is computed only for what is returned)."""
    import omni.ui as ui

    ui_test = _ui_test()
    # Kit queries only ui.Window; asking for another (the status bar) just logs "Failed to find window".
    queryable = {w.title for w in ui.Workspace.get_windows() if isinstance(w, ui.Window)}
    for win in windows():
        title = win["title"]
        if title not in queryable or (window_filter and window_filter.lower() not in title.lower()):
            continue
        try:
            refs = ui_test.find_all(f"{title}//Frame/**/*")
        except Exception:  # noqa: BLE001 - some windows cannot be queried by title
            continue
        for ref in refs[:MAX_WIDGETS_PER_WINDOW]:
            widget = ref.widget
            type_name = type(widget).__name__
            if type_name not in INTERACTIVE and not (include_labels and type_name in TEXTUAL):
                continue
            if not getattr(widget, "visible", True):
                continue
            name, ident, tip = getattr(widget, "name", ""), getattr(widget, "identifier", ""), getattr(widget, "tooltip", "")
            if type_name == "Image" and not (name or tip or ident):
                continue  # decorative icon
            text = getattr(widget, "text", "") if type_name != "StringField" else ""
            value = _value(widget)
            cheap = dict(type=type_name, name=name, identifier=ident, text=text, tooltip=tip, value=value)
            if matches(cheap, query):
                yield title, ref, widget, cheap


def _menu_bar():
    """Top-level menu names: menus are not widgets in find's results; open them with action=menu."""
    try:
        import omni.ui as ui

        return [t for t in (getattr(c, "text", "") for c in ui.Inspector.get_children(_ui_test().get_menubar().widget)) if t]
    except Exception:  # noqa: BLE001
        return []


def menu_walk(path):
    """The menu widgets along *path* from the menu bar, as far as they exist (the menu bar first)."""
    import omni.ui as ui

    found = [_ui_test().get_menubar().widget]
    for part in (p for p in path.split("/") if p):
        named = [c for c in ui.Inspector.get_children(found[-1]) if getattr(c, "text", "") == part]
        if not named:
            break
        found.append(next((c for c in named if getattr(c, "visible", True)), named[0]))  # the one a person sees
    return found


def menu_children(path, inside=False):
    """Items a person sees under the deepest existing level of *path* ("" = the menu bar); with ``inside``
    the last level is walked too, so this lists what the menu *path* itself contains."""
    import omni.ui as ui

    parts = [p for p in path.split("/") if p]
    found = menu_walk("/".join(parts if inside else parts[:-1]))
    items = [getattr(c, "text", "") for c in ui.Inspector.get_children(found[-1])
             if getattr(c, "visible", True) and "Separator" not in type(c).__name__]
    return "/".join(parts[: len(found) - 1]), [t for t in items if t][:40]


def find(a):
    import json

    import omni.ui as ui

    from .graph import _headless

    if a.get("readiness"):  # internal: the host's doctor tool; reads, never enables anything
        return readiness()
    if _headless():  # no layout to report, and walking every widget took about 38 s per call on 6.1
        raise UIRefusal("NO_UI", "This Kit has no application window (headless); finding widgets needs a windowed Isaac Sim.")
    if a.get("zoom_ref"):
        return zoom_rect(a["zoom_ref"])
    query = a.get("query", "") or ""
    window = a.get("window", "") or ""
    limit = a.get("limit", 40)
    if not isinstance(query, str) or len(query) > 120 or not isinstance(window, str) or len(window) > 120:
        raise UIRefusal("INVALID_VALUE", "query and window: strings of at most 120 characters.")
    if type(limit) is not int or not 1 <= limit <= 100:
        raise UIRefusal("LIMIT", "limit must be 1..100.")
    ui_test = _ui_test()
    dpi = ui.Workspace.get_dpi_scale()
    widgets, matched, budget, hidden, examined = [], 0, 48_000, 0, 0  # stay well under the 64 KiB cap
    rects = {w.title: (w.position_x, w.position_y, w.width, w.height) for w in ui.Workspace.get_windows()}
    containers = {}
    for title, ref, widget, cheap in _candidates(window or None, bool(query), query):
        matched += 1
        if len(widgets) >= limit or budget <= 0 or examined >= 4 * limit:
            continue
        try:
            size, center = ref.size, ref.center
            if size.x <= 0 or size.y <= 0 or not inside((center.x, center.y), rects.get(title, (0, 0, 0, 0))):
                hidden += 1  # cheap check first: no path walk for rows outside the window
                continue
            examined += 1
            path = ref.realpath
        except Exception:  # noqa: BLE001 - stale or not yet laid out
            hidden += 1
            continue
        if _shown(path, (center.x, center.y), ui_test, containers) is not None:
            hidden += 1
            continue
        record = widget_record(cheap["type"], f"{path}|{_widget_fingerprint(widget, path, ui_test)}", title, cheap["name"],
                               cheap["text"], cheap["tooltip"], cheap["value"], (center.x, center.y),
                               (size.x, size.y), dpi, getattr(widget, "enabled", True), cheap["identifier"])
        budget -= len(json.dumps(record))
        widgets.append(record)
    return dict(
        main_window=dict(zip(("width", "height"), main_window_pixels()), dpi_scale=dpi),
        windows=windows() if not window else [w for w in windows() if window.lower() in w["title"].lower()],
        hidden_tabs=[t for t in hidden_tabs() if not window or window.lower() in t.lower()],
        menu_bar=_menu_bar(),
        menu_note="Menus are not listed as widgets: open them with sim__ui__act action=menu, e.g. "
                  "menu='Window/Graph Editors/Action Graph' (menu_bar lists the top-level names).",
        matches=matched, widgets=widgets, truncated=len(widgets) + hidden < matched,
        skipped_not_shown=hidden,
        coordinates="x, y are the widget centre in pixels of the Isaac Sim main window, the same pixels as "
                    "screenshot(window_id=<Isaac Sim window>). Pass ref to sim__ui__act to target a widget; "
                    "widgets scrolled out of view, outside their window or under a floating window are left out.",
    )


def readiness():
    """What the host's doctor tool reports about Kit's UI. Reads only: never enables an extension."""
    import omni.kit.app
    import omni.ui as ui

    from .graph import _headless

    def read(getter):
        try:
            return getter()
        except Exception:  # noqa: BLE001 - a headless Kit may have no workspace to ask
            return None

    em = omni.kit.app.get_app().get_extension_manager()
    width, height = read(main_window_pixels) or (None, None)
    return dict(ui_test_enabled=bool(em.is_extension_enabled("omni.kit.ui_test")), headless=bool(_headless()),
                dpi_scale=read(ui.Workspace.get_dpi_scale), main_window=dict(width=width, height=height))


def zoom_rect(ref):
    """The widget's centre (x, y, as find reports it) and size in window pixels, for the host to crop
    around. The ref is checked as for an action: a stale, hidden or covered widget is refused."""
    x, y, hit = _resolve({"ref": ref})
    return dict(ref=ref, type=hit["type"], name=hit["name"], x=x, y=y, width=hit["width"], height=hit["height"])


def _resolve(a):
    """(x, y) in window pixels and the targeted widget, from ref or x/y. Sends nothing."""
    import omni.ui as ui

    dpi = ui.Workspace.get_dpi_scale()
    ref = a.get("ref")
    if ref:
        if not isinstance(ref, str) or "//" not in ref or len(ref) > 800:
            raise UIRefusal("INVALID_VALUE", "ref must be a widget ref from sim__ui__find.")
        path, expected = split_ref(ref)
        if expected is None:
            raise UIRefusal("INVALID_VALUE", "ref must be a ref returned by sim__ui__find (it ends in |Type:label); "
                                             "for a raw position use x and y.")
        ui_test = _ui_test()
        try:
            found = ui_test.find(path)
        except Exception:  # noqa: BLE001
            found = None
        if found is None or found.widget is None:
            raise UIRefusal("STALE_WIDGET", "That widget no longer exists (the UI was rebuilt); call sim__ui__find again.")
        actual = _widget_fingerprint(found.widget, path, ui_test)
        if expected is not None and actual != expected:
            raise UIRefusal("STALE_WIDGET", f"That ref now points at a different widget ({actual}, expected "
                                            f"{expected}): the UI was rebuilt. Call sim__ui__find again.")
        if not getattr(found.widget, "visible", True):
            # sim__ui__find lists only visible widgets, so a hidden one means the UI changed since then.
            raise UIRefusal("STALE_WIDGET", "That widget is hidden now: its panel changed or an inline editor (such "
                                            "as a Stage rename) replaced it. Finish or cancel that, then call "
                                            "sim__ui__find again.")
        c, s = found.center, found.size
        if s.x <= 0 or s.y <= 0:
            raise UIRefusal("NOT_VISIBLE", "That widget has no size yet (not laid out or collapsed).")
        why = _shown(path, (c.x, c.y), ui_test)
        if why:
            raise UIRefusal("NOT_VISIBLE", f"That widget cannot be clicked: {why}.")
        enabled = bool(getattr(found.widget, "enabled", True))
        for ancestor in _ancestors(path):  # a disabled container disables its children
            if not enabled:
                break
            try:
                enabled = bool(getattr(ui_test.find(ancestor).widget, "enabled", True))
            except Exception:  # noqa: BLE001
                pass
        return round(c.x * dpi), round(c.y * dpi), dict(ref=ref, type=type(found.widget).__name__,
                                                          name=getattr(found.widget, "name", ""), enabled=enabled,
                                                          width=round(s.x * dpi), height=round(s.y * dpi))
    x, y = a.get("x"), a.get("y")
    if type(x) is not int or type(y) is not int:
        raise UIRefusal("INVALID_VALUE", "Give ref (from sim__ui__find) or integer x and y window pixels.")
    return x, y, None


def main_window_pixels():
    """The Isaac Sim window's size in pixels: the unit of x and y, of screenshots and of the X11 window.
    omni.ui measures the workspace in UI points, so at a display scale of 1.5 a 1440x900 window says 960x600."""
    import omni.ui as ui

    dpi = ui.Workspace.get_dpi_scale() or 1.0
    return round(ui.Workspace.get_main_window_width() * dpi), round(ui.Workspace.get_main_window_height() * dpi)


def _check_point(x, y):
    w, h = main_window_pixels()
    if not (0 <= x < w and 0 <= y < h):
        raise UIRefusal("OUT_OF_WINDOW", f"({x}, {y}) is outside the Isaac Sim window ({w}x{h} pixels).")
    return w, h


def check_step(a):
    """Refuse a malformed action before anything is sent (pure, no Kit). Returns the frames it waits."""
    action = a.get("action")
    if action not in ACTIONS:
        raise UIRefusal("INVALID_VALUE", f"action must be one of {', '.join(ACTIONS)}.")
    frames = a.get("frames", 3)
    if type(frames) is not int or not 0 <= frames <= 600:
        raise UIRefusal("INVALID_VALUE", "frames: 0..600 frames to wait after the action.")
    if action == "type":
        text = a.get("text")
        if not isinstance(text, str) or not 1 <= len(text) <= MAX_TEXT or any(c in "\r\n\t" or ord(c) < 32 for c in text):
            raise UIRefusal("INVALID_VALUE", f"text: 1..{MAX_TEXT} characters without control characters; "
                                             "send enter or tab with action=keypress.")
    elif action == "keypress":
        parse_keys(a.get("keys"))
    elif action == "scroll":
        sx, sy = a.get("scroll_x", 0), a.get("scroll_y", 0)
        if type(sx) is not int or type(sy) is not int or not (sx or sy) or abs(sx) > 20 or abs(sy) > 20:
            raise UIRefusal("INVALID_VALUE", "scroll_x/scroll_y: wheel steps, -20..20, not both 0 (negative "
                                             "scroll_y scrolls down).")
    elif action == "drag" and (type(a.get("end_x")) is not int or type(a.get("end_y")) is not int):
        raise UIRefusal("INVALID_VALUE", "drag needs integer end_x and end_y window pixels.")
    elif action == "menu":
        menu_path(a.get("menu"))
    elif action in ("tab", "close"):
        window_title(a.get("window"), action)
    if action in POINTER_ACTIONS and not a.get("ref") and (type(a.get("x")) is not int or type(a.get("y")) is not int):
        raise UIRefusal("INVALID_VALUE", "Give ref (from sim__ui__find) or integer x and y window pixels.")
    return frames


def _batch_steps(a):
    """The batch's steps, every one checked before the first is run (its x/y points too; a ref or a window
    title is resolved when its step runs, since an earlier step may open it). The top-level ``frames`` is
    each step's wait unless the step gives its own; other top-level keys that are not an action's own
    (internal flags the host adds) are passed to each step."""
    steps = a["actions"]
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_BATCH:
        raise UIRefusal("INVALID_VALUE", f"actions: 1..{MAX_BATCH} action objects.")
    # scroll_x/scroll_y arrive as 0 when unset (their default); a top-level action is ignored here.
    given = [k for k in STEP_FIELDS if a.get(k) is not None and not (k in ("scroll_x", "scroll_y") and a[k] == 0)]
    if given:
        raise UIRefusal("INVALID_VALUE", f"With actions, give {', '.join(given)} inside each step, not at the top level.")
    shared = {k: v for k, v in a.items() if k not in STEP_FIELDS + ("action", "actions", "find_after")}
    out, frames = [], 0
    for i, step in enumerate(steps):
        if not isinstance(step, dict) or set(step) - set(STEP_FIELDS + ("action", "frames")):
            raise UIRefusal("INVALID_VALUE", f"actions[{i}]: an object with action (inside a step the key is "
                                             f"action, not sub_action) and its own arguments "
                                             f"({', '.join(STEP_FIELDS)}, frames) only.")
        step = {**shared, **step}
        try:
            frames += max(check_step(step), 2)
            if step["action"] in POINTER_ACTIONS and not step.get("ref"):
                _check_point(step["x"], step["y"])
            if step["action"] == "drag":
                _check_point(step["end_x"], step["end_y"])
        except UIRefusal as exc:
            raise UIRefusal(exc.code, f"actions[{i}]: {exc}") from None
        out.append(step)
    if frames > MAX_BATCH_FRAMES:
        raise UIRefusal("INVALID_VALUE", f"actions wait {frames} frames in all (a step counts as at least 2); "
                                         f"keep the batch at {MAX_BATCH_FRAMES} or less, or split it.")
    return out


def _find_after(query):
    """Up to 10 widgets matching *query*, compactly. A failure is a note: the action already happened."""
    try:
        found = find(dict(query=query, limit=10))
    except Exception as exc:  # noqa: BLE001
        return dict(query=query, note=f"find_after failed: {exc}")
    keys = ("ref", "type", "name", "text", "value", "x", "y")
    return dict(query=query, matches=found["matches"],
                widgets=[{k: w[k] for k in keys if k in w} for w in found["widgets"]])


# ---------------------------------------------------------------- cursor badge (opt-in: the host passes badge=True)
# The protocol of the host's mcp/twinsmith/cursor_badge.py, which Kit cannot import: in the folder the host
# names (its own, so both use the same whatever their environments say), the shown badge keeps the flag file
# badge-<pid>-kit.shown (holding this process's start time, as cursor_badge.process_start reads it), hides
# while a host screenshot holds capture.lock (probed with a non-blocking exclusive flock) and deletes the
# flag once the hidden frame is drawn, so no screenshot shows it. Recordings take no lock and show it.

BADGE_TITLE = "twinsmith cursor badge"
BADGE_SIZE, BADGE_OFFSET = 14, 6  # UI points; its corner sits below and right of the hotspot, never on it
BADGE_GLIDE_S, BADGE_LINGER_S = 0.15, 1.5
BADGE_LOCK = "capture.lock"
_badge_state = []  # [the _KitBadge], or [None] once it failed


def _capture_held(fd):
    import fcntl

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True
    fcntl.flock(fd, fcntl.LOCK_UN)
    return False


class _KitBadge:
    """A borderless omni.ui window without inputs, moved over frames by an update subscription that
    exists only while it is shown."""

    def __init__(self, directory):
        import os
        import stat
        from pathlib import Path

        import omni.ui as ui

        # Without these it could take the next click or the keyboard focus (_shown() ignores it): stay off.
        missing = [n for n in ("NO_MOUSE_INPUTS", "NO_FOCUS_ON_APPEARING") if not hasattr(ui, f"WINDOW_FLAGS_{n}")]
        if missing:
            raise RuntimeError(f"omni.ui has no WINDOW_FLAGS_{' or WINDOW_FLAGS_'.join(missing)}")
        flags = ui.WINDOW_FLAGS_NO_MOUSE_INPUTS | ui.WINDOW_FLAGS_NO_FOCUS_ON_APPEARING
        for name in ("NO_TITLE_BAR", "NO_RESIZE", "NO_MOVE", "NO_SCROLLBAR", "NO_SCROLL_WITH_MOUSE", "NO_COLLAPSE",
                     "NO_CLOSE", "NO_SAVED_SETTINGS", "NO_BACKGROUND", "NO_DOCKING"):
            flags |= getattr(ui, f"WINDOW_FLAGS_{name}", 0)  # cosmetic
        self.directory = directory = Path(directory)
        # The host made the folder; nothing is created from a path that arrives over the bridge. As the
        # host's cursor_badge.trusted: a private folder of this account, in a folder of the same kind.
        if not (directory.is_absolute() and directory.name == "twinsmith-desktop" and all(
                stat.S_ISDIR(info.st_mode) and info.st_uid == os.getuid() and not info.st_mode & 0o022
                for info in (os.lstat(directory), os.lstat(directory.parent)))):
            raise RuntimeError(f"{directory} is not a private twinsmith-desktop folder of this account")
        self.flag = directory / f"badge-{os.getpid()}-kit.shown"
        try:  # the host's cursor_badge.process_start: a reused PID is then not mistaken for this Kit
            self.started = Path(f"/proc/{os.getpid()}/stat").read_bytes().rsplit(b")", 1)[1].split()[19].decode()
        except (OSError, IndexError, ValueError):
            self.started = ""
        self.fd = os.open(directory / BADGE_LOCK, os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0), 0o600)
        self.window = ui.Window(BADGE_TITLE, width=BADGE_SIZE, height=BADGE_SIZE, flags=flags, visible=False)
        for attr in ("padding_x", "padding_y"):
            try:
                setattr(self.window, attr, 0)
            except Exception:  # noqa: BLE001 - cosmetic
                pass
        with self.window.frame:
            ui.Circle(style={"background_color": 0xFF008CFF, "border_color": 0xFFFFFFFF, "border_width": 1})
        self.sub, self.pos, self.hiding = None, None, 0
        self.start = self.end = (0.0, 0.0)
        self.t0 = self.last = 0.0

    def glide_to(self, x, y, jump=False):
        """Glide over the next frames (or with ``jump`` move at once) to window pixel (x, y)."""
        import time

        import omni.ui as ui

        dpi = ui.Workspace.get_dpi_scale() or 1.0
        end = (x / dpi + BADGE_OFFSET, y / dpi + BADGE_OFFSET)
        start = end if jump or self.pos is None else self.pos
        if not self.window.visible or self.hiding:
            self.window.position_x, self.window.position_y = start
            if not self._show():
                return
        self.start, self.end = start, end
        self.t0 = self.last = time.monotonic()

    def _show(self):
        """Flag, then probe, then show: a capture that locks after the probe still sees the flag."""
        import os

        draft = self.flag.with_suffix(".draft")
        draft.write_text(self.started)
        os.replace(draft, self.flag)  # put in place whole: a capture never reads it half written
        if _capture_held(self.fd):
            if not self.hiding:  # while hiding, the last shown frame may still be on screen: _update drops the flag
                self.flag.unlink(missing_ok=True)
            return False
        self.hiding = 0
        self.window.visible = True
        if self.sub is None:
            import omni.kit.app

            self.sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._update, name="twinsmith cursor badge")
        return True

    def close(self):
        """After a failure, or when another host's folder takes over: no badge left on screen, no flag
        left to hold up every screenshot, and nothing kept open."""
        import os

        self.sub = None
        try:
            self.window.visible = False
            getattr(self.window, "destroy", lambda: None)()
        finally:
            self.flag.unlink(missing_ok=True)
            os.close(self.fd)

    def _update(self, _event=None):
        import time

        if self.hiding:  # hidden: drop the flag once that frame has been drawn
            self.hiding -= 1
            if not self.hiding:
                self.flag.unlink(missing_ok=True)
                self.sub = None
            return
        now = time.monotonic()
        if now - self.last > BADGE_LINGER_S or _capture_held(self.fd):
            self.window.visible = False
            self.hiding = 2
            return
        t = min(1.0, (now - self.t0) / BADGE_GLIDE_S)
        self.pos = tuple(a + (b - a) * t for a, b in zip(self.start, self.end))
        self.window.position_x, self.window.position_y = self.pos


def badge(x, y, directory, jump=False):
    """Show the cursor badge at window pixel (x, y), hiding for the screenshots of the host whose folder
    ``directory`` is. Best effort: a failure turns it off, never the act."""
    if _badge_state and _badge_state[0] is None:
        return
    try:
        if _badge_state and str(_badge_state[0].directory) != directory:  # another host asks: follow its lock
            _badge_state.pop().close()
        if not _badge_state:
            _badge_state.append(_KitBadge(directory))
        _badge_state[0].glide_to(x, y, jump)
    except Exception as exc:  # noqa: BLE001
        failed, _badge_state[:] = (_badge_state or [None])[0], [None]
        try:
            if failed is not None:
                failed.close()
            import carb

            carb.log_warn(f"twinsmith cursor badge off: {exc}")
        except Exception:  # noqa: BLE001
            pass


async def run_act(a):
    """What sim.ui.act runs: one action, or a batch of up to MAX_BATCH (``actions``) that stops at the first
    failure, then optionally up to 10 widgets matching ``find_after``. Internal flags such as ``badge``
    reach every step."""
    query = a.get("find_after")
    if query is not None and (not isinstance(query, str) or not 1 <= len(query) <= 120):
        raise UIRefusal("INVALID_VALUE", "find_after: text of 1..120 characters, matched like sim__ui__find query.")
    if a.get("actions") is None:
        result = await act(a)
        return dict(result, find_after=_find_after(query)) if query else result
    steps = _batch_steps(a)
    done = []
    for i, step in enumerate(steps):
        try:
            done.append(await act(step))
        except Exception as exc:  # noqa: BLE001 - reported with the steps that ran
            extra = dict(find_after=_find_after(query)) if query else {}
            raise UIRefusal(getattr(exc, "code", "ACT_FAILED"), f"actions[{i}] ({step['action']}): {exc}",
                            effects=f"The {i} step(s) before it ran and sent input; inspect the UI before retrying." if i else None,
                            completed=i, failed_index=i, steps=done, **extra) from exc
    result = dict(action="batch", completed=len(done), steps=done)
    return dict(result, find_after=_find_after(query)) if query else result


async def act(a):
    """Perform one UI action through Kit's input provider; never moves the desktop cursor except for menus."""
    import carb.input
    import omni.appwindow
    import omni.kit.app
    from carb.input import KeyboardEventType, MouseEventType

    frames = check_step(a)  # everything checkable without Kit, before the first event
    action = a["action"]
    app = omni.kit.app.get_app()
    appwin = omni.appwindow.get_default_app_window()
    if appwin is None:
        raise UIRefusal("NO_UI", "This Kit has no application window (headless); UI actions need a windowed Isaac Sim.")
    provider = carb.input.acquire_input_provider()
    mouse, keyboard = appwin.get_mouse(), appwin.get_keyboard()

    async def tick(n=2):
        for _ in range(n):
            await app.next_update_async()

    def mouse_event(kind, x, y, w, h):
        provider.buffer_mouse_event(mouse, kind, (x / w, y / h), 0, (float(x), float(y)))

    # Validate everything before the first event, so a refusal really means nothing was sent.
    target = None
    text = a.get("text") if action == "type" else None
    keys = parse_keys(a.get("keys")) if action == "keypress" else None
    scroll = (a.get("scroll_x", 0), a.get("scroll_y", 0)) if action == "scroll" else None
    end = (a.get("end_x"), a.get("end_y")) if action == "drag" else None
    if end:
        _check_point(*end)
    focus_first = action in ("type", "keypress") and (a.get("ref") or a.get("x") is not None)
    if action in POINTER_ACTIONS or focus_first:
        target = _resolve(a)
        _check_point(target[0], target[1])
        hit = target[2]
        if hit is not None and not hit["enabled"] and action != "move":
            raise UIRefusal("DISABLED", "That widget is disabled (greyed out); it would ignore the input.")
        if a.get("badge"):  # internal: the host's [computer] cursor_badge; adds no wait to the action
            badge(target[0], target[1], a["badge"])

    menu = menu_path(a.get("menu")) if action == "menu" else None
    result = dict(action=action)
    if action in ("tab", "close"):
        # Dock tab headers are drawn by the dock, not widgets: bring the window to the front the way a
        # tab click does (selected in its dock, focused), or hide it the way its tab's close button does.
        title = window_title(a.get("window"), action)
        import omni.ui as ui

        win = ui.Workspace.get_window(title)
        if win is None:
            names = sorted({w.title for w in ui.Workspace.get_windows() if w.title and w.title != BADGE_TITLE})
            raise UIRefusal("NO_WINDOW", f"No Kit window titled {title!r}. Known windows: {', '.join(names[:40])}. "
                                         "Closed windows open from the Window menu (action=menu).")
        if action == "close":
            was_visible = bool(win.visible)
            ui.Workspace.show_window(title, False)
            await tick(max(frames, 2))
            win = ui.Workspace.get_window(title)
            hidden = win is None or not bool(win.visible)
            return dict(result, window=title, hidden=hidden, was_visible=was_visible, frames_waited=max(frames, 2),
                        note=("reopen it with action=tab, window=" + repr(title) + " (or from the Window menu)")
                        if hidden else "Kit did not hide it; check with sim__ui__find or a screenshot")
        ui.Workspace.show_window(title, True)
        await tick(1)
        win.focus()
        await tick(max(frames, 2))
        selected = bool(win.visible) and (not win.docked or win.is_selected_in_dock())
        return dict(result, window=title, shown=selected, docked=bool(win.docked), frames_waited=max(frames, 2),
                    **({} if win.docked else {"note": "a floating window can still be covered by another one; "
                                                       "check with a screenshot"}))
    if action == "wait":
        await tick(max(frames, 1))
        return dict(result, frames_waited=max(frames, 1))

    if action == "menu":
        import omni.ui as ui

        path = menu
        ui_test = _ui_test()
        top = path.split("/")[0]

        async def close():
            """Close the menu this action opened (Kit menus ignore an Escape keypress sent this way), so
            none is left covering the viewport. True when it is closed."""
            try:
                await ui_test.menu_click(top, show=False)
                await tick(2)
                menu_ref = ui_test.get_menubar().find_menu(top)
                return not getattr(getattr(menu_ref, "widget", None), "shown", False)
            except Exception:  # noqa: BLE001
                return False

        def not_found(closed, effects, hint=""):
            try:
                level, items = menu_children(path)
            except Exception:  # noqa: BLE001
                level, items = "", []
            where = f"under '{level}'" if level else "on the menu bar"
            hint = hint or (f" Items {where}: {', '.join(items)}." if items else
                            f" Nothing is listed {where}: its extension may not be loaded yet (for the Action "
                            "Graph editor use sim__graph__show).")
            state = "" if closed else (" A menu may still be open: action=menu with its top-level name "
                                       "(e.g. 'Window') opens and closes it.")
            return UIRefusal("MENU_NOT_FOUND", f"Could not open menu {path!r}.{hint}{state}", effects=effects)

        levels = menu_walk(path)[1:]
        item = next((name for name, level in zip(path.split("/")[:-1], levels) if not isinstance(level, ui.Menu)), None)
        if item:  # a walk down the path would click it on its way
            raise not_found(True, "No input was sent.", f" '{item}' is an item, not a menu: nothing is under it.")
        if any(not getattr(level, "visible", True) for level in levels):
            # A hidden entry (its extension or layout is off): nothing is opened.
            raise not_found(True, "No input was sent.")
        try:
            await ui_test.menu_click(path)
        except Exception as exc:  # noqa: BLE001
            if not levels:
                raise not_found(True, "No input was sent.") from exc
            closed = await close()
            raise not_found(closed, "The menu was opened, and closed again." if closed else None) from exc
        await tick(frames)
        if len(levels) == path.count("/") + 1 and isinstance(levels[-1], ui.Menu):
            # A menu, not an item: clicking it only opens it. Report what it offers, then close it again.
            try:
                _, items = menu_children(path, inside=True)
            except Exception:  # noqa: BLE001
                items = []
            items = [t for t in items if t != "placeholder"]
            closed = await close()
            return dict(result, menu=path, items=items, closed_again=closed, frames_waited=frames,
                        note=f"'{path}' is a menu, not an item: it was opened to list its items" +
                             (" and closed again" if closed else "; it may still be open, check with a screenshot") +
                             f". Run an item with menu='{path}/<item>'.")
        return dict(result, menu=path, frames_waited=frames,
                    note="menu clicks move Kit's cursor through the menu; verify with a screenshot")

    if action in ("type", "keypress"):
        if focus_first:  # focus the target first, like a person would
            x, y, hit = target
            w, h = _check_point(x, y)
            for kind in (MouseEventType.MOVE, MouseEventType.LEFT_BUTTON_DOWN, MouseEventType.LEFT_BUTTON_UP):
                mouse_event(kind, x, y, w, h)
                await tick(2)
            result.update(focused_at=dict(x=x, y=y), widget=hit)
            if hit is None:
                result["note"] = "No known widget at that point, so the input may have gone nowhere. Check with a screenshot, or use a ref from sim__ui__find."
        if action == "type":
            for i, ch in enumerate(text):
                provider.buffer_keyboard_char_event(keyboard, ch, 0)
                if i % 20 == 19:
                    await tick(1)
            await tick(frames)
            if not focus_first:
                result["note"] = "No target given, so this went to whatever has keyboard focus. Pass ref to choose a widget."
            return dict(result, typed=len(text), frames_waited=frames)
        mods, key = keys
        flags = 0
        for m in mods:
            flags |= getattr(carb.input, f"KEYBOARD_MODIFIER_FLAG_{m}")
        KI = carb.input.KeyboardInput
        for m in mods:
            provider.buffer_keyboard_key_event(keyboard, KeyboardEventType.KEY_PRESS, getattr(KI, MODIFIER_KEYS[m]), flags)
        provider.buffer_keyboard_key_event(keyboard, KeyboardEventType.KEY_PRESS, getattr(KI, key), flags)
        await tick(2)
        provider.buffer_keyboard_key_event(keyboard, KeyboardEventType.KEY_RELEASE, getattr(KI, key), flags)
        for m in reversed(mods):
            provider.buffer_keyboard_key_event(keyboard, KeyboardEventType.KEY_RELEASE, getattr(KI, MODIFIER_KEYS[m]), 0)
        await tick(frames)
        names = {"CONTROL": "ctrl", "SHIFT": "shift", "ALT": "alt", "SUPER": "super"}
        return dict(result, keys=[*(names[m] for m in mods), key.lower().removeprefix("key_")], frames_waited=frames)

    x, y, hit = target
    w, h = _check_point(x, y)
    result.update(at=dict(x=x, y=y), widget=hit)
    mouse_event(MouseEventType.MOVE, x, y, w, h)
    await tick(2)
    if action == "move":
        await tick(frames)
        return dict(result, frames_waited=frames)
    if action in ("click", "double_click", "right_click"):
        down, up = ((MouseEventType.RIGHT_BUTTON_DOWN, MouseEventType.RIGHT_BUTTON_UP) if action == "right_click"
                    else (MouseEventType.LEFT_BUTTON_DOWN, MouseEventType.LEFT_BUTTON_UP))
        for _ in range(2 if action == "double_click" else 1):
            mouse_event(down, x, y, w, h)
            await tick(1)
            mouse_event(up, x, y, w, h)
            await tick(1)
        await tick(frames)
        return dict(result, frames_waited=frames)
    if action == "scroll":
        sx, sy = scroll
        # Kit reads the wheel steps from the first (normalised) slot as they are. Dividing them by the
        # window size, as omni.kit.ui_test.emulate_mouse_scroll does, scrolls a hundredth of a step.
        provider.buffer_mouse_event(mouse, MouseEventType.SCROLL, (float(sx), float(sy)), 0, (float(sx), float(sy)))
        await tick(frames)
        return dict(result, scroll=dict(x=sx, y=sy), frames_waited=frames)
    # drag
    ex, ey = end
    mouse_event(MouseEventType.LEFT_BUTTON_DOWN, x, y, w, h)
    await tick(2)
    for i in range(1, 13):
        px, py = round(x + (ex - x) * i / 12), round(y + (ey - y) * i / 12)
        mouse_event(MouseEventType.MOVE, px, py, w, h)
        if a.get("badge"):
            badge(px, py, a["badge"], jump=True)
        await tick(1)
    mouse_event(MouseEventType.LEFT_BUTTON_UP, ex, ey, w, h)
    await tick(frames)
    return dict(result, to=dict(x=ex, y=ey), frames_waited=frames)
