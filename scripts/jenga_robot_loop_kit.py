"""Kit side of scripts/jenga_robot_loop.py: live poses and an RGB-D recorder for the work cell's wrist camera.

The host script loads this file once into a persistent ``run_python`` session and then calls ``poses`` and
``Recorder``. Nothing here moves the robot or edits the cell: it reads Fabric and renders.

Poses are 4 x 4 matrices, world from the prim, translation in the last column. On Newton a body's pose is its Fabric
world matrix once ``sim_tools.physics.sync_fabric`` has published the paused step; a prim fixed to a body (the
camera, the tool frame) is that body's live pose times the prim's authored offset from it.

A recorded clip is the folder the ``sam3`` tool reads: ``rgb/00000.png``, ``depth/00000.png`` (16-bit counts of
distance along the optical axis, 0.1 mm each unless the camera names its own ``depth_scale``) and ``camera.json``
with the intrinsics and one camera pose per frame, in the optical frame (+X right, +Y down, +Z forward). The camera
is any camera prim: the work cell's wrist D415, or one that ``fixed_camera`` makes from a generic description.
"""
import json
import os

import numpy as np
import omni.replicator.core as rep
import omni.usd
import usdrt
from PIL import Image
from pxr import Gf, UsdGeom

DEPTH_SCALE_M = 0.0001


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
    """World matrix of a prim fixed to ``body``: its authored offset from the body, at the body's live pose. With no
    body the prim stands still in the world and its authored pose is its pose."""
    cache = UsdGeom.XformCache()
    if body is None:
        return cache.GetLocalToWorldTransform(_stage().GetPrimAtPath(path))
    offset = (cache.GetLocalToWorldTransform(_stage().GetPrimAtPath(path))
              * cache.GetLocalToWorldTransform(_stage().GetPrimAtPath(body)).GetInverse())
    return offset * _fabric(body)


def poses(bodies=(), attached=None):
    """Live world poses: ``bodies`` are rigid bodies, ``attached`` is {prim path: the body it is fixed to}."""
    from sim_tools import physics

    physics.sync_fabric(_stage())
    out = {path: _matrix(_fabric(path).RemoveScaleShear()) for path in bodies}
    out.update({path: _matrix(_attached(path, body).RemoveScaleShear()) for path, body in (attached or {}).items()})
    return out


def box_in_body(path, body):
    """A prim's bounding box, colliders included, in the frame of the body it is fixed to: [min, max] in metres."""
    from pxr import Usd

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    box = cache.ComputeRelativeBound(_stage().GetPrimAtPath(path), _stage().GetPrimAtPath(body)).ComputeAlignedRange()
    return [[float(v) for v in box.GetMin()], [float(v) for v in box.GetMax()]]


def fixed_camera(path, camera):
    """A camera that stands still in the world, from the generic description any RGB-D camera has: ``width``,
    ``height``, ``fx``, ``fy`` in pixels and ``world_from_camera`` for its optical frame. Session layer only. The
    simulated camera's principal point is the image centre."""
    from pxr import Usd

    stage = _stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        prim = UsdGeom.Camera.Define(stage, path)
        aperture = 36.0
        prim.CreateHorizontalApertureAttr(aperture)
        prim.CreateVerticalApertureAttr(aperture * camera["height"] / camera["width"] * camera["fx"] / camera["fy"])
        prim.CreateFocalLengthAttr(camera["fx"] * aperture / camera["width"])
        prim.CreateClippingRangeAttr(Gf.Vec2f(0.02, 10.0))
        optical = Gf.Matrix4d(*[float(camera["world_from_camera"][column][row]) for row in range(4) for column in range(4)])
        prim.AddTransformOp().Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)) * optical)
    return path


class Recorder:
    """Colour and depth frames of one camera, written as a clip folder. ``body`` is the rigid body the camera rides on,
    or None for a camera that stands still; ``intrinsics`` may name its own ``depth_scale``."""

    def __init__(self, camera, body, folder, size, intrinsics):
        self.camera, self.body, self.folder, self.frames = camera, body, folder, []
        self.intrinsics = dict({"depth_scale": DEPTH_SCALE_M}, **intrinsics)
        self.scale = 1.0 / self.intrinsics["depth_scale"]
        for name in ("rgb", "depth"):
            os.makedirs(os.path.join(folder, name), exist_ok=True)
        self.product = rep.create.render_product(camera, tuple(size), force_new=True)
        self.annotators = {}
        for name in ("rgb", "distance_to_image_plane"):
            self.annotators[name] = rep.AnnotatorRegistry.get_annotator(name, device="cpu", do_array_copy=True)
            self.annotators[name].attach([self.product])

    async def frame(self, label=None, subframes=2):
        """Publish the paused physics poses, render them without advancing time, and save one colour and depth pair."""
        from sim_tools import physics

        physics.sync_fabric(_stage())
        for _ in range(8):                               # a new render product can need a few steps before it has data
            await rep.orchestrator.step_async(rt_subframes=subframes, delta_time=0.0)
            rgb = np.asarray(self.annotators["rgb"].get_data())
            depth = np.asarray(self.annotators["distance_to_image_plane"].get_data(), dtype=np.float32)
            if rgb.size and depth.size:
                break
        else:
            raise RuntimeError("The renderer returned no frame for the wrist camera.")
        index = len(self.frames)
        Image.fromarray(rgb[..., :3]).save(os.path.join(self.folder, "rgb", f"{index:05d}.png"))
        counts = np.where(np.isfinite(depth) & (depth * self.scale < 65535), np.round(depth * self.scale), 0)
        Image.fromarray(counts.astype(np.uint16)).save(os.path.join(self.folder, "depth", f"{index:05d}.png"))
        # The USD camera looks along -Z with +Y up; the optical frame is that turned half a turn about X.
        usd_camera = _attached(self.camera, self.body).RemoveScaleShear()
        optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)) * usd_camera
        self.frames.append({"label": label, "world_from_camera": _matrix(optical)})
        return index

    def close(self, fps):
        """Write camera.json and release the render product; returns the number of frames."""
        with open(os.path.join(self.folder, "camera.json"), "w") as stream:
            json.dump(dict(self.intrinsics, fps=fps, frames=self.frames), stream)
        for annotator in self.annotators.values():
            annotator.detach()
        self.product.destroy()
        return len(self.frames)


def dress_blocks(root="/World/Jenga", groove_m=0.0015, spread=0.2, seed=0):
    """Make the blocks tell apart in a picture, on the session layer only; call it before Play. Physics is untouched.

    The pack's block is a plain box of one colour with sharp edges, so a rendered tower is a featureless slab and no
    camera can separate its blocks. Real blocks have rounded edges, which show as dark lines between them, and differ
    in tone. Each block's own mesh stays the collider and is hidden from the picture (purpose guide); in its place a
    box smaller by ``groove_m`` in every direction is drawn, in the block's colour times a shade of its own."""
    from pxr import Usd, UsdGeom, UsdShade, Vt

    stage, rng, count = _stage(), np.random.RandomState(seed), 0
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for block in stage.GetPrimAtPath(root).GetChildren():
            size = block.GetAttribute("jenga:sizeM").Get()
            if size is None:
                continue
            colour = (0.65, 0.43, 0.22)
            for prim in Usd.PrimRange(block):
                if prim.IsA(UsdGeom.Mesh):
                    UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
                found = UsdShade.Shader(prim).GetInput("diffuseColor") if prim.IsA(UsdShade.Shader) else None
                if found and found.Get() is not None:
                    colour = tuple(found.Get())
            shade = 1.0 + rng.uniform(-spread, spread)
            look = UsdGeom.Cube.Define(stage, block.GetPath().AppendChild("Look"))
            look.CreateSizeAttr(1.0)
            look.AddScaleOp().Set(Gf.Vec3f(*[float(side) - groove_m for side in size]))
            look.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*[min(1.0, channel * shade) for channel in colour])]))
            count += 1
    return count


def move_body(path, pose=None, velocity=None):
    """Write a rigid body's pose [x, y, z, qx, qy, qz, qw] and/or velocity [vx, vy, vz, wx, wy, wz] into the solver."""
    import warp as wp
    from sim_tools import physics

    view = physics.view_for(path)
    device = view.get_transforms().device
    index = wp.array(np.array([0], dtype=np.uint32), dtype=wp.uint32, device=device)
    if pose is not None:
        view.set_transforms(wp.array(np.array([pose], dtype=np.float32), dtype=wp.float32, device=device), index)
    if velocity is not None:
        view.set_velocities(wp.array(np.array([velocity], dtype=np.float32), dtype=wp.float32, device=device), index)


class Hand:
    """A stand-in for a person's hand and forearm: skin-coloured shapes that a camera sees and physics does not.

    Its frame is at the fingertips, +x pointing from the wrist to the fingertips: ``place`` puts the fingertips at a
    world point with the fingers pointing along a yaw. Built on the session layer; call it before Play."""

    SKIN = (0.87, 0.67, 0.54)

    def __init__(self, path="/World/HumanHand"):
        from pxr import Usd, Vt

        stage = _stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            root = UsdGeom.Xform.Define(stage, path)
            self.translate, self.turn = root.AddTranslateOp(), root.AddRotateZOp()

            def shape(kind, name, at, scale=None, radius=None, height=None, yaw=0.0):
                prim = kind.Define(stage, f"{path}/{name}")
                prim.AddTranslateOp().Set(Gf.Vec3d(*at))
                if yaw:
                    prim.AddRotateZOp().Set(yaw)
                if scale:
                    prim.CreateSizeAttr(1.0)
                    prim.AddScaleOp().Set(Gf.Vec3f(*scale))
                else:
                    prim.CreateAxisAttr("X")
                    prim.CreateRadiusAttr(radius)
                    prim.CreateHeightAttr(height)
                prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*self.SKIN)]))

            shape(UsdGeom.Cube, "palm", (-0.115, 0.0, 0.0), scale=(0.09, 0.085, 0.028))
            for index, (across, length) in enumerate(((-0.032, 0.062), (-0.011, 0.07), (0.011, 0.066), (0.032, 0.052))):
                shape(UsdGeom.Capsule, f"finger_{index}", (-0.07 + length / 2 - 0.009, across, 0.0), radius=0.009, height=length - 0.018)
            shape(UsdGeom.Capsule, "thumb", (-0.10, 0.058, -0.004), radius=0.011, height=0.05, yaw=-35.0)
            shape(UsdGeom.Capsule, "forearm", (-0.30, 0.0, 0.0), radius=0.032, height=0.24)
        self.place((0.0, -5.0, 1.0), 0.0)                # out of every view until it is used

    def place(self, fingertips, yaw_deg):
        self.translate.Set(Gf.Vec3d(*[float(v) for v in fingertips]))
        self.turn.Set(float(yaw_deg))


class Removal:
    """A scripted person's turn: a hand comes to one block, draws it out along its long side, carries it off and leaves.

    ``step(t)`` is called every few physics steps with the simulated time since the turn began. While the block is
    drawn and carried, its pose and velocity are written on its path each call (a velocity alone is stopped by
    friction within a step: an impulse is not a hand that keeps pulling). It is then put down at ``discard``.
    Returns the phase and where the fingertips are."""

    def __init__(self, hand, block, toward, discard, speed=0.02, approach_s=2.0, carry_s=1.5, leave_s=2.0):
        from sim_tools import physics

        self.hand, self.block, self.speed, self.discard = hand, block, speed, list(discard)
        self.approach_s, self.draw_s, self.carry_s, self.leave_s = approach_s, 0.09 / speed, carry_s, leave_s
        self.pose = np.array(physics.array(physics.view_for(block).get_transforms()).reshape(-1), dtype=float)   # x y z qx qy qz qw
        axis = np.array(poses([block])[block])[:3, 0]
        self.out = axis if axis @ np.asarray(toward, dtype=float) > 0 else -axis      # the block's long side, the way it is drawn
        self.yaw_deg = float(np.degrees(np.arctan2(-self.out[1], -self.out[0])))      # the fingers point at the tower
        self.phase = "approach"

    def step(self, t):
        up = np.array([0.0, 0.0, 1.0])
        drawn, carried = t - self.approach_s, t - self.approach_s - self.draw_s
        if drawn < 0:
            self.phase, centre = "approach", self.pose[:3]
            fingers = centre + self.out * 0.45 * (-drawn / self.approach_s)
        elif carried < 0:                                    # clear of the tower after 90 mm
            self.phase, centre, velocity = "draw", self.pose[:3] + self.out * self.speed * drawn, self.out * self.speed
        elif carried < self.carry_s:
            self.phase, velocity = "carry", self.out * 0.12 + up * 0.08
            centre = self.pose[:3] + self.out * 0.09 + velocity * carried
        else:
            if self.phase == "carry":                        # put it down, flat, away from the tower
                move_body(self.block, pose=[*self.discard, *self.pose[3:]], velocity=[0.0] * 6)
            left = (carried - self.carry_s) / self.leave_s
            self.phase = "leave" if left < 1.0 else "gone"
            centre = self.pose[:3] + self.out * (0.09 + 0.12 * self.carry_s + 0.5 * min(left, 1.0)) + up * 0.08 * self.carry_s
        if self.phase in ("draw", "carry"):
            move_body(self.block, pose=[*centre, *self.pose[3:]], velocity=[*velocity, 0.0, 0.0, 0.0])
        if self.phase != "approach":
            fingers = centre
        if self.phase == "gone":
            self.hand.place((0.0, -5.0, 1.0), 0.0)
            return {"phase": "gone", "fingertips_m": None}
        fingers = fingers + self.out * 0.021 + up * 0.016    # over the block's outer end
        self.hand.place(fingers, self.yaw_deg)
        return {"phase": self.phase, "fingertips_m": [float(v) for v in fingers]}
