"""Rules that say whether a USD asset will load on Newton (MuJoCo-Warp) or PhysX, read from the file alone.

``RULES`` is the table: each rule has an id, its level per engine, what it checks, and the evidence it
came from. ``validate`` opens the file read-only with ``pxr`` (the ``mesh`` extra) and runs every rule.
It runs in ``mesh_worker``, never in the server. It sees what the file authors: colliders a scene adds
when it is built (as the work cell does for the v1 bench) are not in the file and not checked.
"""

from __future__ import annotations

import math

# Collision triangles a file may carry as triangle-mesh colliders before Newton's contact buffer is at risk.
# The work-cell bench's 1.63 M triangles (385 colliders) asked for 7,500,052 triangle pairs against a buffer
# of 1,000,000 (Isaac Sim 6.2, Newton 1.6.1). 200,000 is a starting value: re-measure it with pack v2.
TRIANGLE_BUDGET = 200_000
MJ_MINVAL = 1e-15            # MuJoCo's mjMINVAL: moving bodies need mass and inertia above it
DEFAULT_RATE_HZ = 500.0      # physics rate the drive-gain rule assumes
DEFAULT_ARMATURE = 0.1       # what Isaac Sim's Newton integration uses when a joint authors none (newton_config.py)
REFERENCE_INERTIA = 0.01     # kg m^2 (or kg for a linear drive) when the driven body authors none

RULES = {
    "UNITS": dict(newton="error", physx="error", rule="metersPerUnit and upAxis are authored", evidence="Import contract"),
    "TRI_COLLIDER": dict(newton="error", physx="info",
                         rule=f"Triangle-mesh colliders stay under the budget (default {TRIANGLE_BUDGET:,} triangles)",
                         evidence="The bench's 1.63 M collider triangles needed 7.5 M triangle pairs; the buffer holds 1 M"),
    "TRI_DYNAMIC": dict(newton="warning", physx="warning",
                        rule="No triangle-mesh collider (approximation none) sits on a moving rigid body",
                        evidence="PhysX does not simulate triangle meshes on dynamic bodies (Asset Validator, "
                                 "CookingApproximationFallbackChecker); on Newton they are costly"),
    "COLLIDER_RENDERED": dict(newton="warning", physx="warning",
                              rule="A collision-only shape has purpose \"guide\"", evidence="Split contract"),
    "DRIVE_ZERO_FORCE": dict(newton="error", physx="info", rule="No joint drive has maxForce 0",
                             evidence="MuJoCo: \"actfrcrange[0] should be smaller than actfrcrange[1]\""),
    "NESTED_ARTICULATION": dict(newton="error", physx="info",
                                rule="No articulation root lies inside another articulation or is jointed into one",
                                evidence="Newton's importer mis-listed the RG2 bodies in the probe"),
    "MASS_MIN": dict(newton="error", physx="info",
                     rule="Every moving body has colliders, or an authored mass and inertia above mjMINVAL",
                     evidence="MuJoCo: \"mass and inertia of moving bodies must be larger than mjMINVAL\""),
    "DRIVE_GAIN": dict(newton="warning", physx="info",
                       rule="dt^2 k / I < 4 + 2 dt c / I at the physics rate: k and c in SI per radian or metre; I the "
                            f"driven body's largest inertia (else {REFERENCE_INERTIA}) plus the joint's armature (else "
                            f"{DEFAULT_ARMATURE}); MuJoCo-Warp integrates stiffness explicitly and damping implicitly",
                       evidence="9400 N m per degree went non-finite at 500 Hz"),
    "MIMIC": dict(newton="error", physx="error",
                  rule="A mimic joint and its leader are in the articulation (not excluded), the mimic names exactly one "
                       "leader, and a revolute follower or leader has finite limits",
                  evidence="PhysX 6.2 rejects such a mimic (omni.physx plugin messages); Newton skips one on a loop joint"),
}
FIXES = {
    "UNITS": "Author metersPerUnit and upAxis on the root layer.",
    "TRI_COLLIDER": "assets__mesh action=split with collision={\"method\": \"box\"} (or \"auto\") adds explicit shapes.",
    "TRI_DYNAMIC": "Give the moving body convex shapes: assets__mesh action=hulls, or approximation convexHull.",
    "COLLIDER_RENDERED": "Set purpose = \"guide\" on the shape, or let assets__mesh split author it.",
    "DRIVE_ZERO_FORCE": "Remove the drive, or give it a positive maxForce.",
    "NESTED_ARTICULATION": "Make one articulation: remove the inner ArticulationRootAPI and joint the bodies into the outer tree.",
    "MASS_MIN": "Give the body a collider, or author a positive mass and diagonalInertia (MassAPI).",
    "DRIVE_GAIN": "Lower the stiffness, raise the damping or the physics rate, or author armature; or set the gains per engine.",
    "MIMIC": "Put the mimic on a joint inside the tree (not the excluded loop joint), name one leader, and author finite limits.",
}


def _schemas(prim, prefix):
    from . import mesh_usd

    return [n for n in mesh_usd.applied_schemas(prim) if n.startswith(prefix)]


def _authored(prim, name):
    attr = prim.GetAttribute(name) if prim else None
    return attr.Get() if attr and attr.HasAuthoredValue() else None


def _check(rule, engine, ok, prim=None, detail="", level=None):
    return dict(id=rule, level=level or RULES[rule][engine], ok=bool(ok), prim=prim, detail=detail,
                fix=None if ok else FIXES[rule])


def _finite_limits(prim) -> bool:
    lo, hi = _authored(prim, "physics:lowerLimit"), _authored(prim, "physics:upperLimit")
    return lo is not None and hi is not None and math.isfinite(lo) and math.isfinite(hi)


def validate(path, engine="newton", *, triangle_budget=TRIANGLE_BUDGET, rate_hz=DEFAULT_RATE_HZ) -> dict:
    from pxr import Usd, UsdGeom, UsdPhysics

    from . import mesh_usd

    if engine not in ("newton", "physx"):
        raise ValueError("engine must be newton or physx")
    stage = mesh_usd.open_stage(path)
    root = stage.GetRootLayer()
    checks = []
    missing = [k for k in ("metersPerUnit", "upAxis") if not root.pseudoRoot.HasInfo(k)]
    checks.append(_check("UNITS", engine, not missing, detail="missing " + ", ".join(missing) if missing else "authored"))

    every = Usd.TraverseInstanceProxies()
    prims = list(stage.Traverse(every))
    bodies = {str(p.GetPath()): p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}

    def moving(prim):
        return not (_authored(prim, "physics:kinematicEnabled") or _authored(prim, "physics:rigidBodyEnabled") is False)

    def body_of(path):
        while path and path != "/":
            if path in bodies:
                return bodies[path]
            path = path.rsplit("/", 1)[0] or "/"
        return None

    tri_total, tri_prims = 0, []
    for prim, _points, faces in mesh_usd.meshes(stage):
        if mesh_usd.approximation(prim) == "none":
            tri_total += len(faces)
            tri_prims.append(str(prim.GetPath()))
            body = body_of(str(prim.GetPath()))
            if body is not None and moving(body):
                checks.append(_check("TRI_DYNAMIC", engine, False, str(prim.GetPath()), f"on the moving body {body.GetPath()}"))
    checks.append(_check("TRI_COLLIDER", engine, tri_total <= triangle_budget,
                         prim=tri_prims[0] if len(tri_prims) == 1 else None,
                         detail=f"{tri_total:,} triangles in {len(tri_prims)} triangle-mesh colliders; budget {triangle_budget:,}"))

    for prim in prims:
        if not mesh_usd.is_collider(prim) or not prim.IsA(UsdGeom.Imageable):
            continue
        names = [str(p.name) for p in prim.GetPath().GetPrefixes()]
        if any(n.endswith("_collision") or n in ("collisions", "collision") for n in names):
            purpose = UsdGeom.Imageable(prim).ComputePurpose()
            if purpose != UsdGeom.Tokens.guide:
                checks.append(_check("COLLIDER_RENDERED", engine, False, str(prim.GetPath()), f"purpose {purpose}"))

    roots = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]

    def owner(path):
        """The articulation root at or above ``path``."""
        best = None
        for r in roots:
            rp = str(r.GetPath())
            if (path == rp or path.startswith(rp + "/")) and (best is None or len(rp) > len(best)):
                best = rp
        return best

    for r in roots:
        outer = owner(str(r.GetPath().GetParentPath()))
        if outer:
            checks.append(_check("NESTED_ARTICULATION", engine, False, str(r.GetPath()), f"inside the articulation at {outer}"))

    joints = {str(p.GetPath()): p for p in prims if p.IsA(UsdPhysics.Joint)}
    dt = 1.0 / rate_hz
    for path, prim in joints.items():
        joint = UsdPhysics.Joint(prim)
        targets = [str(t) for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()) for t in rel.GetTargets()]
        owners = {owner(t) for t in targets} - {None}
        excluded = bool(_authored(prim, "physics:excludeFromArticulation"))
        if len(owners) > 1 and not excluded:
            checks.append(_check("NESTED_ARTICULATION", engine, False, path, "joins the articulations at " + ", ".join(sorted(owners))))
        for drive in _schemas(prim, "PhysicsDriveAPI:"):
            axis = drive.split(":", 1)[1]
            force = _authored(prim, f"drive:{axis}:physics:maxForce")
            if force is not None and float(force) == 0:
                checks.append(_check("DRIVE_ZERO_FORCE", engine, False, path, f"drive {axis} has maxForce 0"))
            k = _authored(prim, f"drive:{axis}:physics:stiffness")
            if k:
                angular = axis in ("angular", "rotX", "rotY", "rotZ")
                per = 180 / math.pi if angular else 1
                k_si, c_si = float(k) * per, float(_authored(prim, f"drive:{axis}:physics:damping") or 0) * per
                body = bodies.get(targets[-1]) if targets else None
                inertia = _authored(body, "physics:diagonalInertia" if angular else "physics:mass") if body else None
                link = (max(inertia) if angular and inertia else float(inertia) if inertia else 0) or REFERENCE_INERTIA
                authored = (_authored(prim, "newton:armature"), _authored(prim, "physxJoint:armature"))
                armature = next((float(a) for a in authored if a is not None), DEFAULT_ARMATURE)  # an authored 0 counts
                i_eff = link + armature
                lhs, rhs = dt * dt * k_si / i_eff, 4 + 2 * dt * c_si / i_eff
                if lhs >= rhs:
                    checks.append(_check("DRIVE_GAIN", engine, False, path,
                                         f"dt^2 k / I = {lhs:.3g} >= 4 + 2 dt c / I = {rhs:.3g} (k {k_si:.4g}, c {c_si:.4g}, "
                                         f"I {link:.4g} + armature {armature:.4g}, {rate_hz:g} Hz)"))

    for path, prim in joints.items():
        leaders = []
        if _schemas(prim, "NewtonMimicAPI"):
            leaders.append([str(t) for t in prim.GetRelationship("newton:mimicJoint").GetTargets()]
                           if prim.GetRelationship("newton:mimicJoint") else [])
        for name in _schemas(prim, "PhysxMimicJointAPI:"):
            rel = prim.GetRelationship(f"physxMimicJoint:{name.split(':', 1)[1]}:referenceJoint")
            leaders.append([str(t) for t in rel.GetTargets()] if rel else [])
        for named in leaders:
            problems = []
            if len(named) != 1:
                problems.append(f"names {len(named)} leaders, not 1")
            if _authored(prim, "physics:excludeFromArticulation"):
                problems.append("the follower is excluded from the articulation (a loop joint)")
            leader = joints.get(named[0]) if len(named) == 1 else None
            if len(named) == 1 and leader is None:
                problems.append(f"leader {named[0]} is not a joint in this file")
            if leader is not None and _authored(leader, "physics:excludeFromArticulation"):
                problems.append("the leader is excluded from the articulation")
            for role, joint_prim in (("follower", prim), ("leader", leader)):
                if joint_prim is not None and joint_prim.IsA(UsdPhysics.RevoluteJoint) and not _finite_limits(joint_prim):
                    problems.append(f"the {role} is a revolute joint without finite limits")
            if problems:
                checks.append(_check("MIMIC", engine, False, path, "; ".join(problems)))

    for path, prim in bodies.items():
        if not moving(prim):
            continue
        mass = _authored(prim, "physics:mass")
        inertia = _authored(prim, "physics:diagonalInertia")
        # a collider under a nested rigid body belongs to that body, not to this one
        if any(mesh_usd.is_collider(p) and body_of(str(p.GetPath())) == prim for p in Usd.PrimRange(prim, every)):
            tiny = [v for v in ([mass] if mass else []) if 0 < float(v) <= MJ_MINVAL]
            if tiny:
                checks.append(_check("MASS_MIN", engine, False, path, f"mass {tiny[0]} at or below {MJ_MINVAL}"))
            continue
        values = [mass] + (list(inertia) if inertia else [None])
        if any(v is None or float(v) <= MJ_MINVAL for v in values):
            checks.append(_check("MASS_MIN", engine, False, path,
                                 f"no collider, so mass and inertia must be authored above {MJ_MINVAL}: mass {mass}, inertia {inertia}"))

    for rule in RULES:                     # one passing line per rule that found nothing
        if not any(c["id"] == rule for c in checks):
            checks.append(_check(rule, engine, True, detail="no findings"))
    failed = [c for c in checks if not c["ok"]]
    totals = dict(errors=sum(c["level"] == "error" for c in failed), warnings=sum(c["level"] == "warning" for c in failed),
                  info=sum(c["level"] == "info" for c in failed), checks=len(checks))
    return dict(ok=totals["errors"] == 0, engine=engine, file=str(path), checks=checks, totals=totals,
                triangle_budget=triangle_budget)
