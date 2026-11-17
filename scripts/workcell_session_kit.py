"""Kit side of ``workcell_session.py``: live poses, cameras, an RGB-D recorder, joint moves and a physics checkpoint.

The host loads this file once into a persistent ``run_python`` session. Poses are 4 x 4, world from the prim, for
column vectors. A recorded clip is the folder the ``sam3`` tool reads: ``rgb/00000.png``, ``depth/00000.png`` (16-bit
counts of distance along the optical axis) and ``camera.json`` with the intrinsics and one camera pose per frame, in
the optical frame (+X right, +Y down, +Z forward). Cameras are drawn on the session layer only.
"""
import json
import os
import time

import numpy as np
import omni.replicator.core as rep
import omni.usd
import usdrt
import warp as wp
from PIL import Image
from pxr import Gf, Usd, UsdGeom
from sim_tools import physics

DEPTH_SCALE_M = 0.0001
USD_FROM_OPTICAL = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))   # a USD camera looks along -Z, +Y up


def _stage():
    return omni.usd.get_context().get_stage()


def _matrix(m):
    """A USD or Fabric matrix (row vectors) as nested lists for column vectors."""
    return [[float(m[column][row]) for column in range(4)] for row in range(4)]


def _fabric(path):
    fabric = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    m = fabric.GetPrimAtPath(path).GetAttribute("omni:fabric:worldMatrix").Get()
    return Gf.Matrix4d(*[m[i][j] for i in range(4) for j in range(4)])


def _attached(path, body):
    """World matrix of a prim fixed to ``body`` (its authored offset at the body's live pose), or of a prim that
    stands still when ``body`` is None."""
    cache = UsdGeom.XformCache()
    authored = cache.GetLocalToWorldTransform(_stage().GetPrimAtPath(path))
    if body is None:
        return authored
    return authored * cache.GetLocalToWorldTransform(_stage().GetPrimAtPath(body)).GetInverse() * _fabric(body)


def poses(bodies=(), attached=None):
    """Live world poses: ``bodies`` are rigid bodies, ``attached`` is {prim path: the body it is fixed to}."""
    physics.sync_fabric(_stage())
    out = {path: _matrix(_fabric(path).RemoveScaleShear()) for path in bodies}
    out.update({path: _matrix(_attached(path, body).RemoveScaleShear()) for path, body in (attached or {}).items()})
    return out


def mesh_centres(meshes):
    """World centres of meshes' bounding boxes at their bodies' live poses: {mesh path: body path}."""
    physics.sync_fabric(_stage())
    out = {}
    for path, body in meshes.items():
        points = np.array(UsdGeom.Mesh(_stage().GetPrimAtPath(path)).GetPointsAttr().Get())
        middle = (points.min(0) + points.max(0)) / 2
        out[path] = list(_attached(path, body).Transform(Gf.Vec3d(*[float(v) for v in middle])))
    return out


def fixed_camera(path, camera):
    """A camera that stands still in the world, from a generic RGB-D description: ``width``, ``height``, ``fx``,
    ``fy`` in pixels and ``world_from_camera`` for its optical frame. The principal point is the image centre."""
    stage = _stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        prim = UsdGeom.Camera.Define(stage, path)
        aperture = 36.0
        prim.CreateHorizontalApertureAttr(aperture)
        prim.CreateVerticalApertureAttr(aperture * camera["height"] / camera["width"] * camera["fx"] / camera["fy"])
        prim.CreateFocalLengthAttr(camera["fx"] * aperture / camera["width"])
        prim.CreateClippingRangeAttr(Gf.Vec2f(0.02, 10.0))
        optical = Gf.Matrix4d(*[float(camera["world_from_camera"][column][row]) for row in range(4) for column in range(4)])
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(USD_FROM_OPTICAL * optical)
    return path


class Recorder:
    """Frames of one camera, written as a clip folder. ``body`` is the rigid body the camera rides on, or None for a
    camera that stands still. With ``depth`` false it writes colour only, as JPEG: frames for a video."""

    def __init__(self, camera, body, folder, size, intrinsics, depth=True):
        self.camera, self.body, self.folder, self.frames, self.depth = camera, body, folder, [], depth
        self.intrinsics = dict({"depth_scale": DEPTH_SCALE_M}, **intrinsics)
        self.product = rep.create.render_product(camera, tuple(size), force_new=True)
        self.annotators = {}
        for name in ("rgb", "distance_to_image_plane") if depth else ("rgb",):
            os.makedirs(os.path.join(folder, "depth" if name != "rgb" else "rgb"), exist_ok=True)
            self.annotators[name] = rep.AnnotatorRegistry.get_annotator(name, device="cpu", do_array_copy=True)
            self.annotators[name].attach([self.product])

    def save(self):
        """Save what the last render step drew; returns the frame's index."""
        data = {name: np.asarray(annotator.get_data()) for name, annotator in self.annotators.items()}
        if not all(image.size for image in data.values()):
            raise RuntimeError(f"The renderer returned no frame for {self.camera}.")
        index = len(self.frames)
        colour = Image.fromarray(data["rgb"][..., :3])
        if self.depth:
            colour.save(os.path.join(self.folder, "rgb", f"{index:05d}.png"))
            metres, scale = data["distance_to_image_plane"].astype(np.float32), 1.0 / self.intrinsics["depth_scale"]
            counts = np.where(np.isfinite(metres) & (metres * scale < 65535), np.round(metres * scale), 0)
            Image.fromarray(counts.astype(np.uint16)).save(os.path.join(self.folder, "depth", f"{index:05d}.png"))
        else:
            colour.save(os.path.join(self.folder, "rgb", f"{index:05d}.jpg"), quality=90)
        optical = USD_FROM_OPTICAL * _attached(self.camera, self.body).RemoveScaleShear()
        self.frames.append({"world_from_camera": _matrix(optical)})
        return index

    def switch(self, folder):
        """A film's recorder writes its next frames to another folder, counted from zero, until ``back``."""
        self.kept = (self.folder, self.frames)
        self.folder, self.frames = folder, []
        os.makedirs(os.path.join(folder, "rgb"), exist_ok=True)

    def back(self):
        self.folder, self.frames = self.kept

    def close(self, fps):
        """Write camera.json and release the render product; returns the number of frames."""
        with open(os.path.join(self.folder, "camera.json"), "w") as stream:
            json.dump(dict(self.intrinsics, fps=fps, frames=self.frames), stream)
        for annotator in self.annotators.values():
            annotator.detach()
        self.product.destroy()
        return len(self.frames)


async def shoot(recorders, subframes=2):
    """Publish the paused physics poses, render them without advancing time, and save one frame per recorder."""
    physics.sync_fabric(_stage())
    for attempt in range(8):                             # a new render product can need a few steps before it has data
        await rep.orchestrator.step_async(rt_subframes=subframes, delta_time=0.0)
        try:
            return [recorder.save() for recorder in recorders]
        except RuntimeError:
            if attempt == 7:
                raise


async def drive(articulation, rows, steps, film=(), every=0, done=0, sample=0):
    """Give an articulation joint position targets row by row ([{DOF index: radians}], an empty row changes nothing)
    and advance physics ``steps`` after each, without a round trip per row. The count of steps starts at ``done``:
    each time it reaches a multiple of ``every`` every recorder in ``film`` saves a frame, and at each multiple of
    ``sample`` the joint positions are read. Returns ``{"shot": frames per recorder, "samples": [[count, position
    per DOF]], "physics_s", "render_s"}``, the last two being the wall time spent stepping (until the GPU has
    finished the steps) and filming."""
    view, out = physics.view_for(articulation, articulation=True), {"shot": 0, "samples": [], "physics_s": 0.0, "render_s": 0.0}
    targets = physics.dof_row(view, view.get_dof_position_targets())        # read once: reading it back before every row cost 3 ms a row
    for row in rows:
        if row:
            for index, value in row.items():
                targets[0, int(index)] = value
            physics.dof_write(view, "set_dof_position_targets", targets)
        left = steps
        while left:
            count = min([left] + ([every - done % every] if film else []) + ([sample - done % sample] if sample else []))
            started = time.perf_counter()
            physics.step(_stage(), count)
            wp.synchronize()                             # the step returns while the GPU still has steps queued: wait, or the clock reads short
            out["physics_s"] += time.perf_counter() - started
            left, done = left - count, done + count
            if sample and done % sample == 0:
                out["samples"].append([done, *physics.dof_row(view, view.get_dof_positions())[0].tolist()])
            if film and done % every == 0:
                started = time.perf_counter()
                await shoot(film)
                out["render_s"] += time.perf_counter() - started
                out["shot"] += 1
    return out


class Checkpoint:
    """The physics state of one articulation and some rigid bodies, to put back after simulating ahead: joint
    positions, velocities and drive targets, and each body's pose and velocity. Not Newton's whole solver state: the
    contact solver starts cold after ``restore``, so let the scene rest for a few steps."""

    def __init__(self, articulation, bodies):
        self.robot = physics.view_for(articulation, articulation=True)
        self.joints = {name: physics.dof_row(self.robot, getattr(self.robot, "get_" + name)())
                       for name in ("dof_positions", "dof_velocities", "dof_position_targets")}
        self.bodies = []
        for path in bodies:
            view = physics.view_for(path)
            self.bodies.append((view, physics.array(view.get_transforms()).copy(), physics.array(view.get_velocities()).copy()))

    def restore(self):
        for name, row in self.joints.items():
            physics.dof_write(self.robot, "set_" + name, row)
        for view, transforms, velocities in self.bodies:
            device = view.get_transforms().device
            index = wp.array(np.array([0], dtype=np.uint32), dtype=wp.uint32, device=device)
            view.set_transforms(wp.array(transforms.astype(np.float32), dtype=wp.float32, device=device), index)
            view.set_velocities(wp.array(velocities.astype(np.float32), dtype=wp.float32, device=device), index)


def place_bodies(placed):
    """Put rigid bodies at world poses, at rest: {path: [x, y, z, qx, qy, qz, qw]}."""
    for path, pose in placed.items():
        view = physics.view_for(path)
        device = view.get_transforms().device
        index = wp.array(np.array([0], dtype=np.uint32), dtype=wp.uint32, device=device)
        view.set_transforms(wp.array(np.array([pose], dtype=np.float32), dtype=wp.float32, device=device), index)
        view.set_velocities(wp.array(np.zeros((1, 6), dtype=np.float32), dtype=wp.float32, device=device), index)
