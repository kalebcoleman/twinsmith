"""USD operations. Session-layer-only edits, no Kit update/step calls."""

import collections
import math
import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
import uuid
from pxr import Ar, Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
from .compat import composition_errors
from twinsmith import material_refs
from twinsmith.project_assets import (
    units_mismatch_message,
    Broker,
    AssetError,
    refuse,
    relative,
    open_dir,
    read_regular,
)

CAP = 10000


def vector(value, n):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != n
        or any(
            type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1e8
            for x in value
        )
    ):
        refuse("INVALID_POSE", f"Expected {n} finite number{'' if n == 1 else 's'} with magnitude <=1e8"
               + (f", got {len(value)} values." if isinstance(value, (list, tuple)) else "."))
    return value


def path(value, root=False):
    if (
        not isinstance(value, str)
        or len(value) > 240
        or not Sdf.Path.IsValidPathString(value)
    ):
        refuse("INVALID_PRIM_PATH", "Invalid USD path.")
    p = Sdf.Path(value)
    if value == "/" and root:
        return p
    if (
        not p.IsAbsolutePath()
        or not p.IsPrimPath()
        or p.ContainsPrimVariantSelection()
        or len(p.GetPrefixes()) > 16
    ):
        refuse(
            "INVALID_PRIM_PATH",
            "Absolute prim path without variants, at most 16 levels required.",
        )
    return p


RUNTIME_ROOTS = ("/Render",)  # Replicator/render-product prims Kit adds at runtime; they crowded out the scene
# Kit's own viewport and render prims: never a scene edit target.
KIT_ROOTS = ("Render", "OmniverseKit_Persp", "OmniverseKit_Front", "OmniverseKit_Top", "OmniverseKit_Right")


def _ros_environment():
    try:
        from .ros_env import describe

        return describe()
    except Exception:  # noqa: BLE001 - informational only
        return None


def bounded(stage, root, limit, depth=None, include_runtime=True):
    """Prims under *root* (pre-order), at most *limit*; *depth* levels below root; runtime /Render prims
    skipped unless *include_runtime* or the root is inside them."""
    if type(limit) is not int or not 1 <= limit <= 200:
        refuse("LIMIT", "limit must be 1..200.")
    if depth is not None and (type(depth) is not int or not 1 <= depth <= 64):
        refuse("LIMIT", "depth must be 1..64.")
    p = stage.GetPrimAtPath(path(root, True))
    if not p:
        refuse("NOT_FOUND", "No prim at requested root. List paths with sim__stage__tree root=\"/\".")
    base = p.GetPath().pathElementCount
    skip = () if include_runtime or any(str(p.GetPath()).startswith(r) for r in RUNTIME_ROOTS) else RUNTIME_ROOTS
    it = iter(Usd.PrimRange(p, Usd.PrimAllPrimsPredicate))
    nodes = []
    for prim in it:
        if prim.IsPseudoRoot():
            continue
        text = str(prim.GetPath())
        too_deep = depth is not None and prim.GetPath().pathElementCount - base > depth
        if too_deep or any(text == r or text.startswith(r + "/") for r in skip):
            it.PruneChildren()
            continue
        if len(nodes) == limit:
            return nodes, True
        nodes.append(prim)
    return nodes, False


def matrix(m):
    return [[float(m[i][j]) for j in range(4)] for i in range(4)]


def facts(prim):
    mass = prim.GetAttribute("physics:mass") if prim.HasAttribute("physics:mass") else None
    density = prim.GetAttribute("physics:density") if prim.HasAttribute("physics:density") else None
    return dict(
        rigid_body=prim.HasAPI(UsdPhysics.RigidBodyAPI),
        collision=prim.HasAPI(UsdPhysics.CollisionAPI),
        articulation_root=prim.HasAPI(UsdPhysics.ArticulationRootAPI),
        joint=prim.IsA(UsdPhysics.Joint),
        mass_authored=mass is not None and mass.HasAuthoredValueOpinion(),
        mass=mass.Get() if mass is not None else None,
        density_authored=density is not None and density.HasAuthoredValueOpinion(),
        density=density.Get() if density is not None else None,
        kinematic=(
            prim.GetAttribute("physics:kinematicEnabled").Get()
            if prim.HasAttribute("physics:kinematicEnabled")
            else None
        ),
        runtime_readiness="unknown",
    )


USD_LAYER_SUFFIXES = (".usd", ".usda", ".usdc", ".usdz")


def visuals_report(stage, root):
    """What of an imported asset will be drawn, and a warning when a physics asset has nothing to draw.

    Instanced ``visuals`` prims are expanded (a session-layer override, never an edit to the asset):
    on some Kit builds an instanced visual stays at its first pose while physics moves its link.
    Returns (report, warning or None)."""
    expanded = 0
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in Usd.PrimRange(root):
            if prim.GetName() == "visuals" and prim.IsInstance() and prim.SetInstanceable(False):
                expanded += 1
    drawn = bodies = count = 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        count += 1
        if count > CAP:
            break
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI):
            bodies += 1
        if prim.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(prim).ComputePurpose() != UsdGeom.Tokens.guide:
            drawn += 1
    report = dict(drawn_geometry=drawn, expanded_instances=expanded, truncated=count > CAP)
    if drawn or not bodies:
        return report, None
    selections = {name: root_variant.GetVariantSelection()
                  for name in root.GetVariantSets().GetNames()
                  for root_variant in [root.GetVariantSets().GetVariantSet(name)]}
    report["variant_selections"] = selections
    return report, ("The asset has physics but nothing to draw: no visible geometry composed under "
                    f"{root.GetPath()}, so the robot would be invisible. Its visual meshes are usually in a "
                    "payload or variant that did not compose; check variant_selections and the asset's own "
                    "sub-files with assets__dependencies, and import the top-level file rather than a "
                    "physics-only layer.")


# Tools which change the stage or timeline themselves, by event family. Anything else that may
# run code (run_python, work cell tools, Kit UI input) can only be reported as "was running".
STAGE_TOOLS = {"sim.stage.open": "sim__stage__open", "sim.quick_start": "sim__quick_start"}
TIMELINE_TOOLS = {"sim.timeline": "sim__timeline", "sim.occupancy_map": "sim__occupancy_map",
                  "sim.quick_start": "sim__quick_start"}
INERT_TOOLS = {"status", "list_tools", "reload_tools", "stat_file", "get_file", "put_file", "job_status"}


def attribute_change(activity, certain):
    """Who changed the stage or timeline, as far as this process can tell.

    *activity* maps bridge tools running (True) or finished moments ago (False) to that state; None
    if unknown. Kit reports no originator, so a named tool is credited only while it is running and
    nothing else that could do it is. "outside_tools" means no such tool was running or just
    finished: a person in the GUI or a Kit script, never a guess when a tool was involved.
    """
    if activity is None:
        return dict(origin="unknown", source="unknown; tool activity is unavailable")
    could_do_it = sorted(
        n for n in activity
        if n in certain or n == "sim.ui.act" or not (n.startswith(("sim.", "assets.")) or n in INERT_TOOLS)
    )
    running = [n for n in could_do_it if activity[n]]
    if len(running) == 1 and running[0] in certain:
        return dict(origin="tool", source=certain[running[0]])
    if could_do_it:
        return dict(origin="unattributed", tools_running=could_do_it,
                    source=", ".join(could_do_it) + " was running or just finished; that, the GUI or a Kit "
                    "script may have done it")
    return dict(origin="outside_tools",
                source="outside the MCP tools: no tool call was running, so a person in the GUI or a Kit script")


LARGE_SAVE_BYTES = 100 * 1024 * 1024


def flattened_size_note(size):
    """Why a saved copy can be far bigger than the scene file it came from."""
    return (f"This file is {size / (1024 * 1024):.0f} MB because it is flattened: the meshes and other data of every "
            "referenced USD file are copied into this one file, in text. It keeps no references to them. "
            "Texture files stay external. Check free disk space before saving often.")


def stop_call(session_id, stage_id):
    """The call that clears TIMELINE_ACTIVE; a different tool, so not a retry_with."""
    return dict(tool="sim__timeline",
                arguments=dict(session_id=session_id, stage_id=stage_id, action="stop"))


class Scene:
    def __init__(self, get_stage, timeline, broker=None, output=None, render_binding=None):
        self.get_stage = get_stage
        self.timeline = timeline
        self.broker = broker or Broker()
        self.output = Path(
            output
            or os.getenv(
                "TWINSMITH_SCENE_OUTPUT",
                tempfile.gettempdir() + "/twinsmith-scenes-" + str(os.getpid()),
            )
        )
        self.session_id = uuid.uuid4().hex
        self._stage = None
        self.stage_id = None
        self.owned = set()
        self.bulk_owned = set()  # large tool-owned subtrees, scoped to this stage
        # Session-layer opinions these tools authored, per prim: what sim.prim.revert undoes.
        self.edits = {}
        self.runtime_assets = {}
        self._runtime_edit_target = None
        self.device_error = None
        self.render_binding = render_binding
        self.events = collections.deque(maxlen=32)  # stage/timeline changes, for sim.logs.tail
        self._retired_stages = {}  # old stage_id -> what replaced it, for STALE_TARGET
        self.previous_sessions = []  # session_ids of earlier instances in this Kit (hot reload)

    def record_event(self, kind, what, attribution, stage=None):
        """Remember a stage or timeline change (never frame-rate events); returns its record."""
        record = dict(kind=kind, what=what, at=time.strftime("%Y-%m-%dT%H:%M:%S"),
                      monotonic=time.monotonic(), **attribution, **({"stage": stage} if stage else {}))
        self.events.append(record)
        return record

    def public_events(self):
        return [{k: v for k, v in e.items() if k != "monotonic"} for e in self.events]

    def last_event(self, kind):
        return next((e for e in reversed(self.events) if e["kind"] == kind), None)

    def retire_stage(self, record=None):
        """The current stage_id ended; keep why, so a later call with it gets a true explanation."""
        if self.stage_id is not None:
            self._retired_stages[self.stage_id] = record
            while len(self._retired_stages) > 16:
                self._retired_stages.pop(next(iter(self._retired_stages)))

    def _stale_reason(self, session_id, stage_id, source):
        if session_id in self.previous_sessions:
            return ("Session changed: sim-tools hot-reloaded inside this same Kit process (a file in its folder "
                    "changed), so that session_id ended. The stage is unchanged, but the tools forgot which prims "
                    "they created. See sim__logs__tail; launch with TWINSMITH_NO_HOT_RELOAD=1 to prevent this."), None
        if session_id != self.session_id:
            return ("Session changed: this session_id is not this Kit process (Kit restarted, or the IDs "
                    "came from another bridge)."), None
        change = self._retired_stages.get(stage_id)
        if change is None:
            return ("Session or stage changed (Kit restarted, the stage was opened or reloaded)."
                    + (" Current stage: " + source + "." if source else "")), None
        changed = "Stage changed at " + change["at"] + " "
        origin = change["origin"]
        if origin == "tool":
            changed += "by " + change["source"] + " (this or another client)."
        elif origin == "outside_tools":
            changed += ("outside this session's tools: no MCP tool call was running, so a person in the GUI "
                        "(e.g. File > Open) or a Kit script replaced it. Edits not saved on the old stage may be lost.")
        else:
            changed += "while the tools could not tell who did it (" + change["source"] + ")."
        return changed + " Current stage: " + source + ".", {k: v for k, v in change.items() if k != "monotonic"}

    def require_runtime(self):
        if self.device_error:
            refuse(
                "DEVICE_ERROR",
                "A native GPU error was reported in this session. Save/inspect/Stop remain available; restart the disposable Kit before runtime or sensor operations. "
                + self.device_error,
            )

    def begin_runtime_edit(self, stage):
        """Keep native physics writeback above session-authored spawn poses."""
        if self._runtime_edit_target is not None:
            self.check_runtime_edit(stage)
            return
        if not self.timeline.is_stopped():
            last = self.last_event("timeline")
            refuse(
                "TIMELINE_ALREADY_ACTIVE",
                "The timeline is already running or paused and was not started by sim__timeline"
                + (" (last change: " + last["what"] + ", " + last["source"] + ")" if last else "")
                + ". To continue: call sim__timeline action=stop, then action=play, so physics writeback "
                "uses the session layer. Stop rewinds physics and returns robots to their start pose.",
                before_retry=stop_call(self.session_id, self.stage_id),
            )
        self._runtime_edit_target = stage.GetEditTarget()
        stage.SetEditTarget(stage.GetSessionLayer())

    def check_runtime_edit(self, stage):
        if self._runtime_edit_target is None:
            # Play came from outside these tools (the UI, a script, workcell__quick_start), so this
            # session never moved physics writeback to the session layer. It cannot be moved now:
            # frames already played wrote to the selected layer, and Stop would restore poses to a
            # different layer than those writes, leaving them behind.
            # The event that started it, not a later pause by these same tools.
            last = next((e for e in reversed(self.events) if e["what"] == "timeline playing"), None)
            ids = dict(session_id=self.session_id, stage_id=self.stage_id)
            refuse(
                "EDIT_TARGET_CHANGED",
                "This timeline was not started by sim__timeline"
                + (" (last change: " + last["what"] + ", " + last["source"] + ")" if last else "")
                + ", so physics writeback is not on the session layer and a tool step could write into your "
                "scene's own layers. To step: call sim__timeline with action=stop, then action=play, "
                "then action=pause, then resend this call. Stop rewinds physics and returns robots to their start "
                "pose; save first (sim__stage__save needs the timeline paused or stopped) if the current state matters.",
                before_retry=stop_call(**ids),
                then=[dict(tool="sim__timeline", arguments=dict(**ids, action=action))
                      for action in ("play", "pause")],
                started_outside_tools=True,
                **({"timeline_change": {k: v for k, v in last.items() if k != "monotonic"}} if last else {}),
            )
        if stage.GetEditTarget().GetLayer() != stage.GetSessionLayer():
            refuse(
                "EDIT_TARGET_CHANGED",
                "Physics writeback requires the managed session edit target, but another editor selected a different "
                "layer during this run. Stop, inspect the selected layer, then play with sim__timeline before stepping.",
                before_retry=stop_call(self.session_id, self.stage_id),
            )

    def end_runtime_edit(self, stage):
        previous = self._runtime_edit_target
        if previous is None:
            return True
        if stage.GetEditTarget().GetLayer() != stage.GetSessionLayer():
            self._runtime_edit_target = None
            return False  # Preserve a target deliberately chosen by an external editor.
        stage.SetEditTarget(previous)
        self._runtime_edit_target = None
        return True

    def stage(self):
        self.broker.require_profile()
        st = self.get_stage()
        if st is None:
            refuse(
                "NO_STAGE",
                "No stage is open. Call sim__quick_start, or sim__stage__open with any session_id and stage_id.",
            )
        if self._stage != st:
            if self._stage is not None:
                self.end_runtime_edit(self._stage)
                self.retire_stage()
            self._stage = st
            self.stage_id = uuid.uuid4().hex
            self.owned.clear()
            self.bulk_owned.clear()
            self.edits.clear()
        return st

    def _current_target(self, stage):
        return dict(
            session_id=self.session_id,
            stage_id=self.stage_id,
            pid=os.getpid(),
            source=stage.GetRootLayer().identifier,
            default_prim=str(stage.GetDefaultPrim().GetPath())
            if stage.GetDefaultPrim()
            else None,
            meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
            up_axis=str(UsdGeom.GetStageUpAxis(stage)),
        )

    def target(self, session_id, stage_id, mutation=False):
        stage = self.stage()
        if not session_id or not stage_id:
            refuse(
                "MISSING_TARGET",
                "session_id and stage_id are required. Call sim__stage__info first and pass its exact IDs "
                f"(currently session_id=\"{self.session_id}\", stage_id=\"{self.stage_id}\"; check "
                "current_target if this is not the stage you meant). This call did nothing.",
                current_target=self._current_target(stage),
            )
        if session_id != self.session_id or stage_id != self.stage_id:
            reason, change = self._stale_reason(session_id, stage_id, stage.GetRootLayer().identifier)
            refuse(
                "STALE_TARGET",
                reason + " This call was not "
                f"retargeted. Resend it with session_id=\"{self.session_id}\" and stage_id=\"{self.stage_id}\" "
                "(check current_target first if the stage is not the one you meant).",
                current_target=self._current_target(stage),
                **({"stage_change": change} if change else {}),
            )
        if mutation and not self.timeline.is_stopped():
            refuse(
                "TIMELINE_ACTIVE",
                "Stop the timeline before authored edits: call sim__timeline action=stop with these IDs, "
                "then resend this call unchanged. Stop rewinds physics and returns robots to their start pose; "
                "pausing is not enough, because paused physics also retains runtime state.",
                before_retry=stop_call(session_id, stage_id),
            )
        return stage

    def close_stage(self):
        """Release the old USD object and lease at CLOSING, not on the next call."""
        if self._stage is not None:
            self.end_runtime_edit(self._stage)
        self.retire_stage(self.last_event("stage"))
        self._stage = None
        self.stage_id = None
        self.owned.clear()
        self.bulk_owned.clear()
        self.edits.clear()

    def stage_info(self):
        stage = self.stage()
        n = 0
        for _ in stage.TraverseAll():
            n += 1
            if n > CAP:
                break
        return dict(
            session_id=self.session_id,
            stage_id=self.stage_id,
            pid=os.getpid(),
            source=stage.GetRootLayer().identifier,
            default_prim=str(stage.GetDefaultPrim().GetPath())
            if stage.GetDefaultPrim()
            else None,
            meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
            up_axis=str(UsdGeom.GetStageUpAxis(stage)),
            prim_count=min(n, CAP),
            truncated=n > CAP,
            playing=self.timeline.is_playing(),
            stopped=self.timeline.is_stopped(),
            time_s=self.timeline.get_current_time(),
            profile="trusted_project",
            sandboxed=False,
            staging=str(self.broker.staging) if self.broker.staging else None,
            usd_version=list(Usd.GetVersion()),
            runtime_edit_target_managed=self._runtime_edit_target is not None,
            device_error=self.device_error,
            ros=_ros_environment(),
        )

    def prim(self, stage, value):
        p = stage.GetPrimAtPath(path(value))
        if not p:
            refuse("NOT_FOUND", "No prim at " + value + ". Find it with sim__prim__find or sim__stage__tree.")
        return p

    def get(self, stage, path):
        p = self.prim(stage, path)
        xf = UsdGeom.Xformable(p)
        local = xf.GetLocalTransformation(Usd.TimeCode.Default()) if xf else None
        world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) if xf else None
        return dict(
            path=str(p.GetPath()),
            type=str(p.GetTypeName()),
            active=p.IsActive(),
            local_matrix=matrix(local) if local is not None else None,
            world_matrix=matrix(world) if world is not None else None,
            world_position=list(world.ExtractTranslation())
            if world is not None
            else None,
            units="stage units",
            time_sample="default",
            matrix_convention="row vectors; translation in last row",
            physics=facts(p),
            pose_source="USD; not a runtime physics tensor observation",
        )

    def pose(self, stage, parent, position, orientation, frame, scale=(1, 1, 1)):
        vector(position, 3)
        vector(orientation, 4)
        if abs(sum(x * x for x in orientation) - 1) > 1e-5:
            refuse("INVALID_POSE", "orientation must be a unit quaternion [w,x,y,z].")
        if frame not in ("local", "world"):
            refuse("INVALID_FRAME", "frame must be local or world.")
        m = Gf.Matrix4d(1)
        m.SetRotate(Gf.Quatd(orientation[0], Gf.Vec3d(*orientation[1:])))
        vector(scale, 3)
        if any(not 1e-6 <= x <= 1e4 for x in scale):
            refuse("INVALID_POSE", "Positive scale 1e-6..1e4 required.")
        m = Gf.Matrix4d().SetScale(Gf.Vec3d(*scale)) * m
        m.SetTranslateOnly(Gf.Vec3d(*position))
        if frame == "world" and not parent.IsPseudoRoot():
            pm = UsdGeom.XformCache().GetLocalToWorldTransform(parent)
            if abs(pm.GetDeterminant()) < 1e-10:
                refuse("UNSUPPORTED_TRANSFORM", "Parent matrix is singular.")
            m = m * pm.GetInverse()
        return m

    def new_path(self, stage, value):
        p = path(value)
        existing = stage.GetPrimAtPath(p)
        if existing and not existing.IsActive():
            refuse(
                "CONFLICT",
                "Destination already exists as a deactivated prim, which keeps its path; no overwrite. Use a new "
                "path, or save a copy with sim__stage__save (it omits the prim) and open that.",
            )
        if existing:
            refuse("CONFLICT", "Destination already exists; no overwrite. Choose a free path, or delete it with sim__prim__delete (a robot cannot be deleted: "
                   "use another path, or start a new scene).")
        parent = stage.GetPrimAtPath(p.GetParentPath())
        if (
            not parent
            or not parent.IsActive()
            or parent.IsInstance()
            or parent.IsInstanceProxy()
        ):
            refuse(
                "INVALID_PARENT", f"Parent {str(p.GetParentPath())!r} must exist, be active and not an instance: create each ancestor first."
                + (" A new stage has no /World: create it first with sim__prim__create kind=Xform." if not stage.GetPrimAtPath("/World") else "")
            )
        if not stage.GetSessionLayer().permissionToEdit:
            refuse("READ_ONLY", "Session layer is not editable.")
        return p, parent

    def create(
        self,
        stage,
        path,
        kind,
        size=1,
        rigid=False,
        position=(0, 0, 0),
        orientation=(1, 0, 0, 0),
        frame="local",
        collision=False,
        scale=(1, 1, 1),
        mass=None,
        density=None,
    ):
        p, parent = self.new_path(stage, path)
        volume = ("Cube", "Sphere", "Cylinder", "Capsule", "Cone")
        if kind not in (*volume, "Plane", "Xform", "PhysicsScene"):
            refuse(
                "UNSUPPORTED_TYPE",
                "Supported types: Cube, Sphere, Cylinder, Capsule, Cone, Plane, Xform, PhysicsScene.",
            )
        if (
            type(size) not in (int, float)
            or not math.isfinite(size)
            or not 0 < size <= 10000
        ):
            refuse("INVALID_SIZE", "size must be finite and 0<size<=10000.")
        if type(rigid) is not bool:
            refuse("INVALID_PHYSICS", "rigid must be true or false.")
        if rigid and kind not in volume:
            refuse(
                "INVALID_PHYSICS",
                "rigid needs volume geometry: Cube, Sphere, Cylinder, Capsule or Cone. A Plane can be a "
                "static collider with collision=true.",
            )
        if mass is not None and density is not None:
            refuse("CONFLICT", "Give mass or density, not both: a mass overrides the density-derived one. "
                   "Nothing was created.", retry_with={"path": path, "kind": kind, "mass": mass})
        for label, value in (("mass", mass), ("density", density)):
            if value is None:
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                refuse("INVALID_PHYSICS", f"{label} must be a finite number greater than 0.")
            if kind not in volume:
                refuse(
                    "INVALID_PHYSICS",
                    f"{label} needs volume geometry (Cube, Sphere, Cylinder, Capsule or Cone): a "
                    f"{kind} has none to derive it from. Nothing was created.",
                )
        if kind == "PhysicsScene" and any(
            p.IsA(UsdPhysics.Scene) for p in stage.Traverse()
        ):
            existing = next(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            refuse("CONFLICT", f"A PhysicsScene already exists at {existing} (creating a sensor adds one). "
                   "Change it with sim__physics__configure instead of creating another.", existing=existing)
        ancestor = parent
        while ancestor and not ancestor.IsPseudoRoot():
            if rigid and ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                refuse("NESTED_BODY", "Ancestor already carries a rigid body.")
            ancestor = ancestor.GetParent()
        m = self.pose(stage, parent, position, orientation, frame, scale)
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                if kind == "PhysicsScene":
                    if list(position) != [0, 0, 0] or list(orientation) != [1, 0, 0, 0]:
                        refuse("INVALID_POSE", "PhysicsScene does not have a pose.")
                    prim = UsdPhysics.Scene.Define(stage, p).GetPrim()
                else:
                    prim = stage.DefinePrim(p, kind)
                    if kind == "Cube":
                        UsdGeom.Cube(prim).CreateSizeAttr(size)
                    if kind == "Sphere":
                        UsdGeom.Sphere(prim).CreateRadiusAttr(size / 2)
                    if kind == "Plane":
                        plane = UsdGeom.Plane(prim)
                        plane.CreateWidthAttr(size)
                        plane.CreateLengthAttr(size)
                        plane.CreateAxisAttr(
                            UsdGeom.Tokens.z
                            if UsdGeom.GetStageUpAxis(stage) == "Z"
                            else UsdGeom.Tokens.y
                        )
                        plane.CreateDoubleSidedAttr(True)
                    UsdGeom.Xformable(prim).AddTransformOp().Set(m)
                    if kind in ("Cylinder", "Capsule", "Cone"):
                        geometry = getattr(UsdGeom, kind)(prim)
                        geometry.CreateRadiusAttr(size / 2)
                        geometry.CreateHeightAttr(size)
                    if rigid or collision:
                        UsdPhysics.CollisionAPI.Apply(prim)
                    if rigid:
                        UsdPhysics.RigidBodyAPI.Apply(prim)
                        # Let sim.physics.contacts read this body's contacts without a later edit.
                        try:
                            from pxr import PhysxSchema

                            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)
                        except ImportError:  # no PhysX schema on this build; contacts will refuse
                            pass
                    if mass is not None or density is not None:
                        self.remember_api(stage, str(p), ["PhysicsMassAPI"])
                        mass_api = UsdPhysics.MassAPI.Apply(prim)
                        if mass is not None:
                            self.remember_properties(stage, str(p), ["physics:mass"])
                            mass_api.CreateMassAttr(float(mass))
                        else:
                            self.remember_properties(stage, str(p), ["physics:density"])
                            mass_api.CreateDensityAttr(float(density))
            self.owned.add(str(p))
            return self.get(stage, str(p))
        except Exception:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                stage.RemovePrim(p)
            raise

    def _edit(self, prim_path):
        return self.edits.setdefault(prim_path, {"properties": {}, "api": {}})

    def remember_properties(self, stage, prim_path, names):
        """Record the session layer's state before this pack authors these attributes.

        sim.prim.revert restores a prior session opinion or removes the one this pack added;
        without the record it could not tell either from a source-layer value.
        """
        layer = stage.GetSessionLayer()
        entry = self._edit(prim_path)
        for name in names:
            prop = str(Sdf.Path(prim_path).AppendProperty(name))
            if prop in entry["properties"]:
                continue
            spec = layer.GetPropertyAtPath(prop)
            has_default = spec is not None and spec.HasDefaultValue()
            entry["properties"][prop] = dict(
                existed=spec is not None,
                had_default=has_default,
                default=spec.default if has_default else None,
            )

    def remember_api(self, stage, prim_path, tokens):
        """Record whether the session layer already applied these API schemas."""
        entry = self._edit(prim_path)
        spec = stage.GetSessionLayer().GetPrimAtPath(prim_path)
        value = spec.GetInfo("apiSchemas") if spec is not None else None
        prepended = list(value.prependedItems) if value is not None else []
        for token in tokens:
            if token not in entry["api"]:
                entry["api"][token] = token in prepended

    def revert(self, stage, path, subtree=True):
        """Undo only this pack's session-layer opinions on one prim (and optionally its subtree)."""
        target = Sdf.Path(path)
        layer = stage.GetSessionLayer()
        if not layer.permissionToEdit:
            refuse("READ_ONLY", "Session layer is not editable.")
        paths = sorted(
            (p for p in self.edits if p == path or (subtree and Sdf.Path(p).HasPrefix(target))),
            key=lambda p: p.count("/"),
            reverse=True,
        )
        composed_before = {p: self.get(stage, p) for p in paths if stage.GetPrimAtPath(p)}
        removed, restored, not_reverted = [], [], []
        with Usd.EditContext(stage, layer):
            for prim_path in paths:
                entry = self.edits.pop(prim_path)
                spec = layer.GetPrimAtPath(prim_path)
                if spec is None:
                    continue
                for prop, prior in entry["properties"].items():
                    prop_spec = layer.GetPropertyAtPath(prop)
                    if prop_spec is None:
                        continue
                    if prior["existed"]:
                        if prior["had_default"]:
                            prop_spec.default = prior["default"]
                            restored.append(prop)
                        else:
                            not_reverted.append(
                                dict(property=prop, reason="the session layer already had this opinion "
                                     "with no static default, so it was left alone")
                            )
                        continue
                    spec.RemoveProperty(prop_spec)
                    removed.append(prop)
                if entry["api"]:
                    value = spec.GetInfo("apiSchemas")
                    prepended = list(value.prependedItems) if value is not None else []
                    drop = [token for token, existed in entry["api"].items() if not existed]
                    keep = [token for token in prepended if token not in drop]
                    if drop:
                        if keep:
                            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=tuple(str(item) for item in keep)))
                        else:
                            spec.ClearInfo("apiSchemas")
                        removed.extend("apiSchemas:" + token for token in drop)
                    restored.extend(
                        "apiSchemas:" + token
                        for token, existed in entry["api"].items()
                        if existed
                    )
        after = {}
        for prim_path in paths:
            if stage.GetPrimAtPath(prim_path):
                after[prim_path] = self.get(stage, prim_path)
        report = [dict(path=p, before=composed_before.get(p), after=after.get(p)) for p in paths[:16]]
        return dict(
            path=path,
            subtree=subtree,
            reverted_paths=paths,
            reverted_path_count=len(paths),
            removed=removed[:200],
            removed_count=len(removed),
            restored=restored[:200],
            restored_count=len(restored),
            not_reverted=not_reverted[:20],
            prims=report,
            prims_truncated=len(paths) > 16,
            created_by_pack=[prim_path for prim_path in paths[:16] if prim_path in self.owned],
            layer="session",
            source_files_changed=False,
            note="Only session-layer opinions these tools recorded were touched; source layers and the "
                 "current stage are unchanged.",
        )

    def transform(
        self,
        stage,
        path,
        position=(0, 0, 0),
        orientation=(1, 0, 0, 0),
        frame="local",
        scale=None,
    ):
        prim = self.prim(stage, path)
        if prim.IsPseudoRoot() or prim.GetPath() == Sdf.Path("/"):
            refuse("INVALID_PRIM_PATH", "The stage root has no pose to set.")
        if prim.IsInstanceProxy():
            refuse(
                "INSTANCE_PROXY",
                f"{path} is an instance proxy inside an instance and cannot be edited directly. Edit the "
                "instance root instead: find it with sim__stage__tree root=\"/\" (it composes the proxy), and "
                "pass that path.",
            )
        if prim.GetPath().GetPrefixes()[0].name in KIT_ROOTS:
            refuse("PROTECTED_PRIM", "Kit's runtime render and viewport prims are protected.")
        xf = UsdGeom.Xformable(prim)
        if not xf:
            refuse(
                "WRONG_TYPE",
                f"{path} is a {prim.GetTypeName() or 'prim'} with no Xformable schema, so it has no pose. "
                "Move or wrap it under an Xform parent instead.",
            )
        root = self.articulation_root_for(stage, prim)
        if root is not None:
            movable = root.get("move_instead")
            refuse(
                "ARTICULATION_LINK",
                f"{path} is attached by a joint, so it is a link of the articulation at {root['root']}; moving it "
                "would break its joints."
                + (f" Move the robot instead: call sim__prim__set_transform with path=\"{movable}\"." if movable
                   else " Move the Xform above the joints instead."),
                articulation_root=root["root"],
                **({"retry_with": dict(session_id=self.session_id, stage_id=self.stage_id, path=movable,
                                       position=list(position), orientation=list(orientation),
                                       frame=frame, **({"scale": list(scale)} if scale is not None else {}))}
                   if movable else {}),
            )
        ops = xf.GetOrderedXformOps()
        animated = [op.GetName() for op in ops if op.GetAttr().GetNumTimeSamples()]
        if animated:
            refuse(
                "ANIMATED_TRANSFORM",
                f"{path} has a time-sampled transform ({', '.join(animated)}); a static override would remove "
                "the animation. Animate or offset a parent Xform instead, or edit the source clip.",
            )
        before = self.get(stage, path)
        if scale is None:
            # Keep the prim's own scale: moving a scaled table must not resize it.
            try:
                scale = list(Gf.Transform(xf.GetLocalTransformation(Usd.TimeCode.Default())).GetScale())
            except (RuntimeError, ValueError):
                scale = [1, 1, 1]
        m = self.pose(stage, prim.GetParent(), position, orientation, frame, scale)
        if xf.GetResetXformStack() and frame == "local":
            m = m * UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
        single = len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if single:
                op = ops[0]
                self.remember_properties(stage, path, [op.GetAttr().GetName()])
                op.Set(m)
                authored = [op.GetAttr().GetName()]
            else:
                existing = next(
                    (op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTransform), None
                )
                name = existing.GetName() if existing is not None else "xformOp:transform"
                self.remember_properties(stage, path, [name, "xformOpOrder"])
                op = existing if existing is not None else xf.AddTransformOp()
                op.Set(m)
                xf.SetXformOpOrder([op])
                authored = [op.GetName(), "xformOpOrder"]
        result = self.get(stage, path)
        result.update(
            before=before,
            authored=authored,
            transform_override=(
                "one transform op replaces the composed op order"
                if not single
                else "the existing transform op now has a session-layer value"
            ),
            layer="session",
            source_files_changed=False,
            revert="sim__prim__revert",
        )
        return result

    def articulation_root_for(self, stage, prim):
        """What to move instead of a joint-attached link: {'root': articulation root, 'move_instead': Xform}.

        None when this prim is not a joint body, or is itself the articulation root. An articulation's
        root is often a joint sibling (``<robot>/root_joint``) that has no pose of its own, so the
        useful target is the Xform above it.
        """
        target = prim.GetPath()
        attached = False
        for index, candidate in enumerate(Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate)):
            if index >= 20000:
                break
            if not candidate.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(candidate)
            bodies = {
                path
                for relationship in (joint.GetBody0Rel(), joint.GetBody1Rel())
                for path in relationship.GetTargets()
            }
            if target in bodies:
                attached = True
                break
        if not attached or prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return None
        parent = prim.GetParent()
        scope = stage.GetPseudoRoot() if parent.IsPseudoRoot() else parent
        root_prim = None
        for index, candidate in enumerate(Usd.PrimRange(scope)):
            if index >= 20000:
                break
            if candidate.HasAPI(UsdPhysics.ArticulationRootAPI):
                root_prim = candidate
                break
        root_path = str((root_prim or parent).GetPath())
        if root_prim is not None and UsdGeom.Xformable(root_prim):
            movable = root_prim  # the robot Xform itself is the articulation root
        else:
            movable = root_prim.GetParent() if root_prim is not None else parent
        while movable and not movable.IsPseudoRoot() and not UsdGeom.Xformable(movable):
            movable = movable.GetParent()
        return dict(
            root=root_path,
            move_instead=str(movable.GetPath()) if movable and not movable.IsPseudoRoot() else None,
        )

    def inspect_asset(self, snapshot_id, manifest_sha256):
        filename, manifest = self.broker.verify(snapshot_id, manifest_sha256)
        # Trusted profile only: this API invokes native parsers/resolvers. Its output
        # diagnoses dependencies; it is NOT pre-load security enforcement.
        layers, assets, missing = UsdUtils.ComputeAllDependencies(
            Sdf.AssetPath(str(filename))
        )
        root = filename.parents[len(Path(manifest["entry"]).parts) - 1]
        outside = []
        runtime_resolved = []
        for dep in [str(layer.realPath) for layer in layers] + list(assets):
            # Compared as real paths: an environment reached through a symlink resolves to its target.
            known = next(
                (k for k, v in self.runtime_assets.items() if os.path.realpath(dep) == os.path.realpath(v["path"])), None
            )
            if known:
                runtime_resolved.append(known)
                continue
            try:
                package_member = None
                outer = dep
                if Ar.IsPackageRelativePath(dep):
                    outer, package_member = Ar.SplitPackageRelativePathOuter(dep)
                    relative(package_member)
                rel = str(Path(outer).relative_to(root))
                relative(rel)
                if rel not in manifest["files"]:
                    outside.append(dep)
                if package_member is not None and package_member not in manifest.get(
                    "packages", {}
                ).get(rel, {}).get("members", {}):
                    outside.append(dep)
            except (ValueError, AssetError):
                outside.append(dep)
        runtime_used = runtime_resolved + [
            str(x) for x in missing if str(x) in self.runtime_assets
        ]
        missing = [x for x in missing if str(x) not in self.runtime_assets]
        asset = None
        unused = []
        if missing and not outside and all(str(x).lower().endswith(USD_LAYER_SUFFIXES) for x in missing):
            # A missing layer the composed asset never asks for: named only by a variant that is not
            # selected (a robot's optional gripper). Every missing layer the selection does use is a
            # composition error, so an asset that composes cleanly is complete as it stands.
            asset = Usd.Stage.Open(str(filename))
            if asset and asset.GetDefaultPrim() and not composition_errors(asset):
                unused, missing = [str(x) for x in missing], []
        if missing or outside:
            refuse(
                "DEPENDENCIES",
                "Unresolved or outside-snapshot dependencies: "
                + repr(dict(missing=list(missing)[:12], outside=outside[:12]))[:1500],
            )
        asset = asset or Usd.Stage.Open(str(filename))
        if not asset or not asset.GetDefaultPrim():
            refuse("DEFAULT_PRIM", "Asset needs a valid defaultPrim. Set defaultPrim in the file, or import a USD that has one.")
        count = 0
        for prim in asset.TraverseAll():
            count += 1
            if count > CAP:
                refuse("LIMIT", "Asset exceeds 10000 composed prims.")
            # Explicitly refuse known script-bearing/graph prims even in trusted mode.
            # Not a claim of comprehensive script detection or execution prevention.
            if prim.GetTypeName() in (
                "OmniGraph",
                "OmniGraphNode",
            ) or prim.HasAttribute("node:type"):
                refuse(
                    "SCRIPT_GRAPH",
                    "OmniGraph/script-bearing assets are not supported by this importer.",
                )
        return (
            filename,
            asset,
            dict(
                snapshot_id=snapshot_id,
                manifest_sha256=manifest_sha256,
                default_prim=str(asset.GetDefaultPrim().GetPath()),
                meters_per_unit=UsdGeom.GetStageMetersPerUnit(asset),
                up_axis=str(UsdGeom.GetStageUpAxis(asset)),
                prim_count=count,
                dependency_layers=len(layers),
                dependency_assets=len(assets),
                packages=[
                    dict(
                        path=p,
                        default_layer=v["default_layer"],
                        member_count=len(v["members"]),
                        extracted=False,
                    )
                    for p, v in list(manifest.get("packages", {}).items())[:12]
                ],
                packages_truncated=len(manifest.get("packages", {})) > 12,
                runtime_dependencies={k: self.runtime_assets[k] for k in runtime_used},
                **({"unselected_variant_dependencies": dict(
                    missing=[Path(x).name for x in unused[:12]],
                    note="Named only by variants that are not selected; the asset composes without them. "
                         "Selecting such a variant later needs these files.")} if unused else {}),
                dependency_check="UsdUtils.ComputeAllDependencies; trusted, not a containment proof",
                sandboxed=False,
            ),
        )

    def import_asset(self, stage, snapshot_id, manifest_sha256, prim_path, **pose):
        p, parent = self.new_path(stage, prim_path)
        m = self.pose(
            stage,
            parent,
            pose.get("position", [0, 0, 0]),
            pose.get("orientation", [1, 0, 0, 0]),
            pose.get("frame", "local"),
        )
        filename, asset, info = self.inspect_asset(snapshot_id, manifest_sha256)
        if info["up_axis"] != UsdGeom.GetStageUpAxis(stage) or not math.isclose(
            info["meters_per_unit"], UsdGeom.GetStageMetersPerUnit(stage), rel_tol=1e-9
        ):
            refuse(
                "UNITS_MISMATCH",
                units_mismatch_message(
                    info["up_axis"], info["meters_per_unit"],
                    str(UsdGeom.GetStageUpAxis(stage)), UsdGeom.GetStageMetersPerUnit(stage),
                ),
                retry_with=dict(up_axis=info["up_axis"], meters_per_unit=info["meters_per_unit"]),
            )
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                wrapper = UsdGeom.Xform.Define(stage, p)
                wrapper.AddTransformOp().Set(m)
                child = stage.DefinePrim(p.AppendChild("Asset"))
                if (
                    not child.GetReferences().AddReference(str(filename))
                    or not child.IsValid()
                ):
                    refuse("IMPORT_FAILED", "Reference could not be authored/composed.")
                errors = composition_errors(stage)
                if errors:
                    refuse("COMPOSITION_ERROR", str(errors)[:1500])
            self.owned.add(str(p))
            visuals, warning = visuals_report(stage, child)
            return dict(
                **info,
                prim=self.get(stage, str(p)),
                asset_path=str(p.AppendChild("Asset")),
                source_preserved=True,
                visuals=visuals,
                **({"warnings": [warning]} if warning else {}),
            )
        except Exception:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                stage.RemovePrim(p)
            raise

    def validate_project_materials(self, snapshot_id, manifest_sha256):
        """Validate bounded literal MDL and MaterialX project dependencies."""
        try:
            return self._validate_project_materials(snapshot_id, manifest_sha256)
        except material_refs.MaterialRefError as exc:
            refuse(exc.code, str(exc))

    def _validate_project_materials(self, snapshot_id, manifest_sha256):
        filename, manifest = self.broker.verify(snapshot_id, manifest_sha256)
        root = filename.parents[len(Path(manifest["entry"]).parts) - 1]
        layers, assets, missing = UsdUtils.ComputeAllDependencies(
            Sdf.AssetPath(str(filename))
        )
        # A standalone resolver can report a known Kit material by name rather
        # than by its installed path. Do not accept that fallback if the file
        # recorded at startup is gone or has changed.
        for dependency in missing:
            runtime = self.runtime_assets.get(str(dependency))
            if runtime is not None:
                runtime_path = Path(runtime["path"])
                if (
                    not runtime_path.is_file()
                    or hashlib.sha256(runtime_path.read_bytes()).hexdigest()
                    != runtime["sha256"]
                ):
                    refuse(
                        "DEPENDENCIES",
                        "The required installed runtime material is missing or changed.",
                    )
        dependency_names = [str(layer.realPath) for layer in layers] + [
            str(a) for a in assets
        ]
        for package, details in manifest.get("packages", {}).items():
            package_path = str(root / package)
            used = package == manifest["entry"] or any(
                package_path in value for value in dependency_names
            )
            if used and any(
                Path(member).suffix.lower() in {".mdl", ".mtlx"}
                for member in details.get("members", {})
            ):
                refuse(
                    "DEPENDENCIES_UNSUPPORTED",
                    "USDZ packages containing MDL or MaterialX need package-aware material validation: "
                    + package,
                )
        textures = []
        other_mdl_resources = []
        materialx = []

        def reference(parent, value, kind):
            material_refs.check_value(value, kind)
            target = Path(os.path.normpath(str(parent / value)))
            try:
                rel = target.relative_to(root).as_posix()
                relative(rel)
            except ValueError:
                refuse("DEPENDENCIES", kind + " escapes the approved project: " + value)
            if rel not in manifest["files"]:
                refuse("DEPENDENCIES", "Missing " + kind + ": " + rel)
            return target, rel

        pending_mtlx = []
        # USD releases disagree on whether MaterialX is reported as a layer or
        # an asset. Inspect both collections and de-duplicate their paths.
        for dependency in dict.fromkeys(dependency_names):
            dep = Path(str(dependency))
            runtime = next(
                (v for v in self.runtime_assets.values() if os.path.realpath(str(dep)) == os.path.realpath(v["path"])),
                None,
            )
            if runtime is not None:
                # Kit may report its built-in MDL as resolved, while standalone
                # OpenUSD reports the same module as an unresolved name. Both
                # forms are accepted only for the pinned installed runtime file.
                if (
                    not dep.is_file()
                    or hashlib.sha256(dep.read_bytes()).hexdigest() != runtime["sha256"]
                ):
                    refuse(
                        "DEPENDENCIES",
                        "The required installed runtime material is missing or changed.",
                    )
                continue
            if dep.suffix.lower() == ".mtlx":
                pending_mtlx.append(dep)
                continue
            if dep.suffix.lower() != ".mdl":
                continue
            try:
                mdl_rel = dep.relative_to(root).as_posix()
            except ValueError:
                refuse(
                    "DEPENDENCIES_UNSUPPORTED",
                    "External MDL module is outside the approved project snapshot.",
                )
            relative(mdl_rel)
            if mdl_rel not in manifest["files"]:
                refuse(
                    "DEPENDENCIES", "MDL material missing from the snapshot: " + mdl_rel
                )
            resources, modules = material_refs.scan_mdl(dep.read_text(encoding="utf-8"), mdl_rel)
            if "OmniPBR" in modules:
                runtime = self.runtime_assets.get("OmniPBR.mdl")
                if (
                    not runtime
                    or not Path(runtime["path"]).is_file()
                    or hashlib.sha256(Path(runtime["path"]).read_bytes()).hexdigest()
                    != runtime["sha256"]
                ):
                    refuse(
                        "DEPENDENCIES",
                        "The required installed OmniPBR MDL module is missing or changed.",
                    )
            for kind, value in resources:
                _, rel = reference(dep.parent, value, kind)
                (textures if kind == "MDL texture" else other_mdl_resources).append(rel)
        visited = set()
        while pending_mtlx:
            dep = pending_mtlx.pop()
            try:
                rel = dep.relative_to(root).as_posix()
                relative(rel)
            except ValueError:
                refuse(
                    "DEPENDENCIES_UNSUPPORTED",
                    "MaterialX is outside the project snapshot.",
                )
            if rel in visited:
                continue
            if rel not in manifest["files"] or len(visited) >= 1000:
                refuse(
                    "DEPENDENCIES", "Missing or excessive MaterialX dependency: " + rel
                )
            visited.add(rel)
            for value in material_refs.scan_materialx(dep.read_bytes(), rel):
                target, name = reference(dep.parent, value, "MaterialX dependency")
                materialx.append(name)
                if target.suffix.lower() == ".mtlx":
                    pending_mtlx.append(target)
                else:
                    material_refs.check_materialx_target(target, name)
        return dict(
            mdl_literal_textures=textures[:32],
            mdl_texture_count=len(textures),
            mdl_other_resources=other_mdl_resources[:32],
            mdl_other_resource_count=len(other_mdl_resources),
            materialx_dependencies=materialx[:32],
            materialx_dependency_count=len(materialx),
            truncated=len(textures) > 32
            or len(other_mdl_resources) > 32
            or len(materialx) > 32,
            note="Literal MDL and MaterialX paths checked. Custom MDL modules, dynamic/tiled paths and MaterialX fileprefix are refused; this is not a sandbox.",
        )

    def import_path(self, stage, snapshot_id, manifest_sha256, prim_path, **pose):
        self.inspect_asset(snapshot_id, manifest_sha256)
        materials = self.validate_project_materials(snapshot_id, manifest_sha256)
        result = self.import_asset(
            stage, snapshot_id, manifest_sha256, prim_path, **pose
        )
        result["material_checks"] = materials
        return result

    def save_directory(self, directory):
        """Where sim.stage.save writes: the configured output root, or an existing folder under the
        user's home (lab checkpoints are kept in a work folder there)."""
        if directory in (None, ""):
            self.output.mkdir(mode=0o700, parents=True, exist_ok=True)
            return self.output
        if not isinstance(directory, str) or not os.path.isabs(os.path.expanduser(directory)):
            refuse("INVALID_OUTPUT", "directory must be an absolute path to an existing folder.")
        folder = Path(os.path.expanduser(directory)).resolve()
        home = Path.home().resolve()
        if not folder.is_dir():
            refuse("INVALID_OUTPUT", f"{folder} is not an existing folder; create it first.")
        if folder != home and home not in folder.parents and folder != self.output.resolve():
            refuse("INVALID_OUTPUT", f"directory must be inside {home} (or the configured output root).")
        return folder

    def save(self, stage, filename, runtime_poses=None, reset_camera_render_handles=False, directory=None):
        if self.timeline.is_playing():
            refuse(
                "TIMELINE_ACTIVE",
                "Pause or stop before saving a stable composed snapshot.",
            )
        # A bare basename only lacks the extension; anything with another dot still refuses.
        if isinstance(filename, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,59}", filename):
            filename += ".usda"
        if not isinstance(filename, str) or not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9_-]{0,59}\.usda", filename
        ):
            hint = (" This tool writes text .usda files: pass the name alone, or with .usda, not .usd or .usdc."
                    if isinstance(filename, str) and filename.lower().endswith((".usd", ".usdc", ".usdz")) else "")
            refuse(
                "INVALID_OUTPUT",
                "Use a simple name of letters, digits, _ and -, ending .usda (<=65 characters). No spaces or folders; use directory for a folder." + hint,
            )
        from .render_export import camera_handles, reset_export_handles, restore_export_camera_targets, save_report

        handles, camera_repairs = camera_handles(stage, reset_camera_render_handles, self.render_binding)
        folder = self.save_directory(directory)
        from twinsmith.project_assets import open_dir

        fd = open_dir(folder)
        tmp = None
        try:
            # Export to private temporary file; hard-link publication is no-replace,
            # unlike an exists check followed by Export to the destination.
            handle, tmp = tempfile.mkstemp(prefix=".twinsmith-tmp-", suffix=".usda", dir=folder)
            os.close(handle)
            layer = stage.Flatten()
            from .flatten import drop_unused_prototypes

            drop_unused_prototypes(layer)
            omitted_render_properties = reset_export_handles(layer, handles)
            restored_camera_targets = restore_export_camera_targets(layer, camera_repairs)
            omitted_render_paths = strip_runtime_render_state(layer)
            if runtime_poses:
                apply_snapshot_poses(layer, runtime_poses)
            report = save_report(layer)
            if not layer.Export(tmp):
                refuse("SAVE_FAILED", "USD export returned false.")
            try:
                os.link(tmp, filename, dst_dir_fd=fd, follow_symlinks=False)
            except FileExistsError:
                refuse("CONFLICT", "Output already exists; no overwrite.")
            size = os.stat(filename, dir_fd=fd).st_size
            return dict(
                file=str(folder / filename),
                bytes=size,
                flattened=True,
                **({"size_note": flattened_size_note(size)} if size > LARGE_SAVE_BYTES else {}),
                stage_switched=False,
                runtime_pose_count=len(runtime_poses or []),
                omitted_runtime_render_paths=omitted_render_paths,
                omitted_runtime_render_properties=omitted_render_properties,
                restored_camera_targets=restored_camera_targets,
                snapshot="composed USD with runtime rigid-body poses"
                if runtime_poses
                else "composed USD",
                checkpoint=False,
                dependencies="Texture files may still reference retained snapshots; do not clean those while using saved scenes.",
                **report,
            )
        finally:
            os.close(fd)
            if tmp is not None:
                os.unlink(tmp)


def apply_snapshot_poses(layer, poses):
    """Apply world poses to a flattened copy, preserving scale and hierarchy.

    The live stage and its layers are never edited. Parents precede children so
    a child's local matrix is computed against the exported parent's runtime pose.
    This is a pose snapshot, not a velocity/controller/solver checkpoint.
    """
    output = Usd.Stage.Open(layer)
    original = {}
    for entry in poses:
        prim = output.GetPrimAtPath(path(entry["path"]))
        if not prim or prim.IsInstance() or prim.IsInstanceProxy():
            refuse(
                "SNAPSHOT_UNSUPPORTED",
                "Runtime body cannot be represented in the flattened output.",
            )
        original[entry["path"]] = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
    for entry in sorted(poses, key=lambda e: e["path"].count("/")):
        prim = output.GetPrimAtPath(entry["path"])
        position = vector(entry["world_position"], 3)
        q = vector(entry["orientation_wxyz"], 4)
        if abs(sum(x * x for x in q) - 1) > 1e-4:
            refuse("PHYSICS_FORMAT", "Runtime orientation is not a unit quaternion.")
        transform = Gf.Transform(original[entry["path"]])
        transform.SetTranslation(Gf.Vec3d(*position))
        transform.SetRotation(Gf.Rotation(Gf.Quatd(q[0], Gf.Vec3d(*q[1:]))))
        parent = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
        xf = UsdGeom.Xformable(prim)
        world = transform.GetMatrix()
        local = world if xf.GetResetXformStack() else world * parent.GetInverse()
        reset = xf.GetResetXformStack()
        xf.MakeMatrixXform().Set(local)
        xf.SetResetXformStack(reset)


def strip_runtime_render_state(layer):
    """Omit process-bound SyntheticData graphs from the private saved copy.

    Cameras, LiDAR definitions, materials and user Action Graphs remain. Native
    graph registrations and render-product handles cannot be reopened as USD.
    Unknown content at a native pipeline path refuses, rather than being lost.
    """
    output = Usd.Stage.Open(layer)
    known_nodes = {
        "omni.replicator.core.OgnRefTimeGate",
        "omni.syntheticdata.SdOnNewFrame",
        "omni.syntheticdata.SdOnNewRenderProductFrame",
        "omni.syntheticdata.SdPostRenderVarToHost",
        "omni.syntheticdata.SdRenderVarPtr",
        "omni.graph.nodes.GpuInteropRenderProductEntry",
        "isaacsim.sensors.rtx.IsaacCreateRTXLidarScanBuffer",
        # The point-cloud writer's annotator; the helper regenerates it at Play.
        "isaacsim.sensors.rtx.nodes.OgnIsaacExtractRTXSensorPointCloud",
        "omni.graph.action.RationalTimeSyncGate",
        # Replicator 1.13.38 creates this writer/annotator synchronization gate
        # instead of the older action gate. It is regenerated on attachment.
        "omni.replicator.core.OgnRationalTimeSyncGateOnce",
        "omni.replicator.nv.ReadRpFabricTime",
        "omni.syntheticdata.SdNoOp",
        "isaacsim.core.nodes.IsaacSimulationGate",
        "isaacsim.core.nodes.IsaacReadSimulationTimeAnnotator",
        "isaacsim.core.nodes.IsaacConvertRGBAToRGB",
        # Isaac Sim 5.1's laser-scan path and Replicator time node; regenerated the same way.
        "isaacsim.sensors.rtx.IsaacComputeRTXLidarFlatScan",
        "omni.replicator.core.ReadRpFabricTime",
    }
    # Camera and LiDAR helpers regenerate these session-only writers on playback. A
    # flattened export includes their process handles, unlike a normal USD
    # root-layer save. Only omit recognized writers backed by a preserved
    # helper with the same topic and frame; an arbitrary ROS node is not safe
    # to discard. NVIDIA documents the helper-generated SDGPipeline as runtime
    # state, not part of the asset.
    ros_writers = {
        "isaacsim.ros2.bridge.ROS2PublishImage":
            "isaacsim.ros2.bridge.ROS2CameraHelper",
        "isaacsim.ros2.bridge.ROS2PublishCameraInfo":
            "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        "isaacsim.ros2.bridge.ROS2PublishLaserScan":
            "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        "isaacsim.ros2.bridge.ROS2PublishPointCloud":
            "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
    }
    helpers = {}
    for prim in output.Traverse():
        node_type = prim.GetAttribute("node:type").Get()
        if node_type in ros_writers.values() and not str(prim.GetPath()).startswith("/Render/"):
            key = (node_type, prim.GetAttribute("inputs:topicName").Get(),
                   prim.GetAttribute("inputs:frameId").Get())
            helpers.setdefault(key, []).append(prim)

    def regenerable_writer(prim):
        node_type = prim.GetAttribute("node:type").Get()
        helper_type = ros_writers.get(node_type)
        writer_id = prim.GetAttribute("inputs:writerId").Get()
        key = (helper_type, prim.GetAttribute("inputs:topicName").Get(),
               prim.GetAttribute("inputs:frameId").Get())
        if not helper_type or not isinstance(writer_id, str) or not re.fullmatch(
            r"NodeWriter_[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", writer_id
        ):
            return False
        for helper in helpers.get(key, []):
            # A standalone helper without its authored render-product source
            # cannot rebuild the writer after reopening.
            for source in helper.GetAttribute("inputs:renderProductPath").GetConnections():
                producer = output.GetPrimAtPath(source.GetPrimPath())
                if (producer and producer.GetAttribute("node:type").Get()
                        == "isaacsim.core.nodes.IsaacCreateRenderProduct"
                        and source.name == "outputs:renderProductPath"):
                    return True
        return False
    omitted = []
    owned_products = set()
    for prim in output.Traverse():
        name = str(prim.GetPath())
        if name == "/Orchestrator":
            # Replicator's session graph has a Python event subscription which
            # USD cannot serialize. In 1.13.4 the reopened graph makes
            # _is_tick_graph() true, skipping _subscribe_update(); render frames
            # complete but CPU annotators remain empty. Let Replicator rebuild
            # its own graph/subscription in the next process. Do not call its
            # private lifecycle APIs or mutate the live stage to repair this.
            native_nodes = {
                "OgnReadFabricTime": "omni.replicator.core.ReadFabricTime",
                "OrchestratorUpdate": "omni.replicator.core.OgnUpdateOrchestrator",
            }
            children = list(prim.GetChildren())
            if (
                prim.GetTypeName() != "OmniGraph"
                or prim.GetAttribute("evaluator:type").Get() != "push"
                or prim.GetAttribute("pipelineStage").Get() != "pipelineStageSimulation"
                or not children
                or not any(c.GetName() == "OgnReadFabricTime" for c in children)
                or any(
                    c.GetTypeName() != "OmniGraphNode"
                    or c.GetName() not in native_nodes
                    or c.GetAttribute("node:type").Get()
                    != native_nodes.get(c.GetName())
                    or c.GetChildren()
                    for c in children
                )
            ):
                refuse(
                    "SAVE_UNSUPPORTED",
                    "Unrecognized /Orchestrator content; saved copy not published. Live stage preserved.",
                )
            omitted.append(name)
            continue
        if not name.startswith("/Render/"):
            continue
        if prim.GetTypeName() == "OmniGraph" and prim.GetName() == "SDGPipeline":
            for child in Usd.PrimRange(prim):
                if child == prim:
                    continue
                if (
                    child.GetTypeName() != "OmniGraphNode"
                    or (child.GetAttribute("node:type").Get() not in known_nodes
                        and not regenerable_writer(child))
                ):
                    refuse(
                        "SAVE_UNSUPPORTED",
                        "Unknown content in native SyntheticData pipeline; saved copy not published. Live stage preserved.",
                        path=str(child.GetPath()),
                        node_type=child.GetAttribute("node:type").Get(),
                    )
            omitted.append(name)
        elif prim.GetTypeName() == "RenderProduct" and re.fullmatch(
            r"IsaacMCP_[0-9a-f]{32}", prim.GetName()
        ):
            omitted.append(name)
            owned_products.add(prim.GetPath())
    # A user controller may reference a native graph. Dropping that dependency
    # silently would change its behavior; refuse before publishing the copy.
    omitted_paths = [Sdf.Path(name) for name in omitted]
    for prim in output.Traverse():
        if any(prim.GetPath().HasPrefix(path) for path in omitted_paths):
            continue
        for attr in prim.GetAttributes():
            if any(
                target.HasPrefix(path)
                for target in attr.GetConnections()
                for path in omitted_paths
            ):
                refuse(
                    "SAVE_UNSUPPORTED",
                    "Preserved content connects to process-bound render state; saved copy not published. Live stage preserved.",
                )
        for relationship in prim.GetRelationships():
            targets = relationship.GetTargets()
            removed = [target for target in targets
                       if any(target.HasPrefix(path) for path in omitted_paths)]
            if not removed:
                continue
            if (str(relationship.GetPath()) == "/Render/OmniverseGlobalRenderSettings.products"
                    and prim.GetTypeName() == "RenderSettings"
                    and all(target in owned_products for target in removed)):
                # Kit automatically enrolls every live render product here.
                # Remove only our omitted handles from the private copy; keep
                # user products and their order. Other dependencies refuse.
                relationship.SetTargets([target for target in targets if target not in removed])
            else:
                refuse(
                    "SAVE_UNSUPPORTED",
                    "Preserved relationship targets process-bound render state; saved copy not published. Live stage preserved.",
                    relationship=str(relationship.GetPath()),
                    targets=[str(target) for target in removed],
                )
    # Remove deepest paths first. Never edit the live stage or its layers.
    for name in sorted(omitted, key=len, reverse=True):
        output.RemovePrim(name)
    return sorted(omitted)
