"""Camera and LiDAR tools: create, adopt, capture and read."""

import hashlib
import os
import re
from pxr import Gf, Usd, UsdGeom
from twinsmith.project_assets import refuse
from .core import vector


CAPTURE_NAME = r"[A-Za-z0-9][A-Za-z0-9_-]{0,59}\.png"


CAPTURE_NAME_RULE = "Use a new name of letters, digits, _ and -, ending .png (<=64 characters). No spaces or folders."


def _render_report():
    """``physics_render``: how Newton's poses reach this capture (nothing on PhysX, which writes USD)."""
    from .physics import render_publication

    info = render_publication()
    return {} if info is None else dict(physics_render=info)


class SensorTools:
    """Mixed into Advanced: uses its scene (``self.s``), sensors and ``physics_manager``."""

    def close_sensors(self, stage_closing=False):
        from .sensors import dispose

        for p, (_, sensor) in list(self.sensors.items()):
            self.cleanup_errors.extend(
                {"path": p, "error": e}
                for e in dispose(sensor, stage_closing=stage_closing)
            )
        self.cleanup_errors = self.cleanup_errors[-16:]
        self.sensors.clear()

    def adopt(self, stage, path, kind):
        """Register render resources for an existing sensor, on an explicit adopt=true. The prim is
        not added to the pack-owned set: deleting it still needs allow_foreign."""
        from .sensors import adopt

        self.s.prim(stage, path)
        if len(self.sensors) >= 8:
            refuse("LIMIT", "At most eight live sensors per session; delete an owned sensor to release its resources.")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            sensor = adopt(stage, path, kind)
        if stage.GetRootLayer().GetPrimAtPath(sensor["product_path"]):
            # Hydra authors a new render product in the root layer too, whatever the edit target.
            sensor["root_layer_note"] = (
                "Kit's render product is also in the in-memory root layer; nothing is on disk. Do not File > Save "
                "over the source. sim__stage__save copies omit it."
            )
        self.sensors[path] = (kind, sensor)
        return self.sensors[path]

    def tool_camera_list(self, name, stage, a, camera_frame=None):
        from . import geometry

        return geometry.cameras(stage, a.get("root", "/"), a.get("limit", 100))

    def tool_camera_create(self, name, stage, a, camera_frame=None):
        s = self.s
        from . import placement

        p, parent = s.new_path(stage, a["path"])
        if str(p) in self.sensors:
            refuse(
                "CONFLICT",
                "Path still has registered sensor resources after external USD changes. "
                "Use a new stage to release them before reusing this path.",
            )
        kind = "camera" if name == "sim.camera.create" else "lidar"
        parent_prim = stage.GetPrimAtPath(str(p).rsplit("/", 1)[0] or "/")
        if kind == "lidar" and parent_prim.IsA(UsdGeom.Gprim):
            refuse("INVALID_PARENT", f"{parent_prim.GetPath()} is a {parent_prim.GetTypeName()}, and USD can't nest a sensor "
                   "under geometry. Use an Xform parent, such as the link above it.")
        frame = a.get("frame", "local")
        lidar_asset = optics = None
        try:
            orientation, rotate_xyz = placement.orientation_args(a.get("orientation"), a.get("rotate_xyz"))
            tick = placement.tick_rate(a.get("tick_rate")) if kind == "lidar" else None
            if kind == "lidar" and a.get("config"):
                from .compat import lidar_config_registry

                lidar_asset = placement.resolve_lidar_config(
                    a["config"], a.get("variant") or None, lidar_config_registry()
                )
            elif kind == "lidar" and a.get("variant"):
                refuse("INVALID_VALUE", "variant needs config.")
            if kind == "camera":
                optics = placement.camera_optics(
                    a.get("focal_length"), a.get("horizontal_aperture"),
                    a.get("clipping_range"), UsdGeom.GetStageMetersPerUnit(stage),
                )
        except placement.PlacementError as e:
            refuse(e.code, str(e), **e.details)
        except ImportError as e:
            refuse("SENSOR_API_UNAVAILABLE", "Config LiDARs need isaacsim.sensors.rtx: " + str(e))
        pose = s.pose(stage, parent, a.get("position", [0, 0, 0]), orientation, frame)
        if name == "sim.camera.create" and a.get("target") is not None:
            if rotate_xyz is not None:
                refuse("INVALID_POSE", "target overrides the orientation; omit rotate_xyz.")
            eye = Gf.Vec3d(*vector(a.get("position", [0, 0, 0]), 3))
            if frame != "world":      # the target is a world point; a local position is taken to the world first
                eye = UsdGeom.XformCache().GetLocalToWorldTransform(parent).Transform(eye)
            target = Gf.Vec3d(*vector(a["target"], 3))
            up = (
                Gf.Vec3d(0, 0, 1)
                if UsdGeom.GetStageUpAxis(stage) == "Z"
                else Gf.Vec3d(0, 1, 0)
            )
            direction = target - eye
            if (
                direction.GetLength() < 1e-8
                or Gf.Cross(direction, up).GetLength() < 1e-8
            ):
                refuse(
                    "INVALID_POSE",
                    "Look-at needs distinct eye/target not parallel to stage up-axis.",
                )
            pose = (
                Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse()
                * UsdGeom.XformCache().GetLocalToWorldTransform(parent).GetInverse()
            )
            rotate_xyz = None
        if len(self.sensors) >= 8:
            refuse(
                "LIMIT",
                "At most eight live sensors per session; delete an owned sensor to release its resources.",
            )
        if name == "sim.camera.create":
            w = a.get("width", 640)
            h = a.get("height", 480)
            if (
                type(w) is not int
                or type(h) is not int
                or not 64 <= w <= 1024
                or not 64 <= h <= 1024
            ):
                refuse("LIMIT", "width/height must be integers 64..1024.")
        from .sensors import create as create_sensor

        with Usd.EditContext(stage, stage.GetSessionLayer()):
            sensor = create_sensor(
                stage,
                p,
                kind,
                w if kind == "camera" else 128,
                h if kind == "camera" else 128,
                lidar_asset=lidar_asset,
                tick_rate=tick,
                optics=optics,
            )
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(p))
            xf.ClearXformOpOrder()
            if rotate_xyz is not None and frame == "local":
                # The ops a person authors in the Property panel: readable and editable there.
                xf.AddTranslateOp().Set(Gf.Vec3d(*a.get("position", [0, 0, 0])))
                xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
            else:
                xf.AddTransformOp().Set(pose)
        self.sensors[str(p)] = (kind, sensor)
        s.owned.add(str(p))
        world = UsdGeom.Xformable(stage.GetPrimAtPath(p)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        result = dict(
            path=str(p),
            kind=kind,
            prim_type=str(stage.GetPrimAtPath(sensor.get("sensor_path", str(p))).GetTypeName()),
            parent=str(parent.GetPath()),
            local_translate=list(pose.ExtractTranslation()),
            local_orientation_wxyz=[pose.ExtractRotationQuat().GetReal(), *pose.ExtractRotationQuat().GetImaginary()],
            **({"rotate_xyz_deg": rotate_xyz} if rotate_xyz is not None and frame == "local" else {}),
            world_position=list(world.ExtractTranslation()),
            ready=False,
            adapter=sensor["adapter"],
            release=sensor["release"],
            build=sensor["build"],
            note="Needs ordinary rendered warm-up; no implicit Play. Direct owned-sensor deletion releases render resources.",
        )
        if kind == "camera":
            hfov = placement.fov_degrees(sensor["focal_length_mm"], sensor["horizontal_aperture_mm"])
            result.update(
                clipping_range_m=optics["clipping_range_m"],
                clipping_range_stage_units=sensor["clipping_range_stage_units"],
                resolution=[w, h],
                focal_length_mm=sensor["focal_length_mm"],
                horizontal_aperture_mm=sensor["horizontal_aperture_mm"],
                vertical_aperture_mm=sensor["vertical_aperture_mm"],
                horizontal_fov_deg=round(hfov, 3),
                rate="A USD camera has no tick rate: it renders every frame. Set a ROS publishing rate on the ROS2 Camera Helper (frameSkipCount).",
            )
        else:
            from .ros2 import scan_graph_args, tf_name

            try:
                import carb.settings

                motion = bool(carb.settings.get_settings().get("/renderer/raytracingMotion/enabled"))
            except ImportError:  # host unit tests without Kit
                motion = None
            result.update(
                sensor_path=sensor["sensor_path"],
                render_product=sensor.get("product_path"),
                ros2_scan_graph=scan_graph_args(sensor["sensor_path"], tf_name(stage.GetPrimAtPath(sensor["sensor_path"]))),
                ros2_note=(
                    "To publish /scan, pass ros2_scan_graph to sim__graph__build: it targets sensor_path, the OmniLidar, "
                    "and makes its own render product at Play, which survives a save. render_product is this "
                    "session's own, for sim__lidar__read; saves drop it. Scans are stamped with the sensor's own frame: pass "
                    "sensor_path in sim__ros2_tf extra_frames so that frame is in the tree."
                ),
                tick_rate_hz=sensor["tick_rate_hz"],
                scan_rate_hz=sensor["scan_rate_hz"],
                config=a.get("config") or None,
                **({"config_asset": sensor["config_asset"]} if "config_asset" in sensor else {}),
                **({"variant": a.get("variant")} if a.get("variant") else {}),
                motion_bvh=motion,
                **({"warning": sensor["warning"]} if "warning" in sensor else {}),
            )
            if motion is False:
                result["motion_note"] = (
                    "Motion BVH is off: scans are correct, but motion distortion within one scan "
                    "(sensor or objects moving during the sweep) is not modelled. Launch with "
                    "run.sh --motion-bvh for that."
                )
        return result

    def tool_camera_capture(self, name, stage, a, camera_frame=None):
        s = self.s
        entry = self.sensors.get(a["path"])
        expected = "camera" if name == "sim.camera.capture" else "lidar"
        if entry is None or entry[0] != expected:
            refuse("NOT_OWNED", "Sensor is not registered in this session.")
        sensor = entry[1]
        adoption = (
            dict(adopted=True, session_overrides=sensor.get("session_overrides", []),
                 **({"root_layer_note": sensor["root_layer_note"]} if "root_layer_note" in sensor else {}))
            if sensor.get("adopted") else {}
        )
        prim = s.prim(stage, a["path"])
        if not prim.IsActive():
            refuse("SENSOR_INVALIDATED", "Sensor prim is inactive.")
        if expected == "lidar":
            limit = a.get("limit", 64)
            if type(limit) is not int or not 1 <= limit <= 256:
                refuse("LIMIT", "limit must be 1..256.")
            sensor_prim = stage.GetPrimAtPath(sensor.get("sensor_path") or a["path"])
            attr = sensor_prim.GetAttribute("omni:sensor:Core:outputFrameOfReference")
            if not attr or attr.Get() != "SENSOR":
                refuse(
                    "SENSOR_MODIFIED",
                    "LiDAR output frame changed outside this tool; restore SENSOR before reading.",
                )
            from .sensors import lidar_points

            return dict(path=a["path"], **adoption, **lidar_points(sensor, limit), **_render_report())
        h, w = sensor["resolution"]
        if not 64 <= h <= 1024 or not 64 <= w <= 1024:
            refuse(
                "SENSOR_FORMAT",
                "Camera resolution changed outside configured bounds.",
            )
        from .camera_capture import CameraFrame

        if not isinstance(camera_frame, CameraFrame):
            refuse(
                "NOT_READY",
                "Capture requires validated renderer acquisition or an explicit tool-acquired cache read.",
            )
        data = camera_frame.data
        if expected == "camera":
            import numpy as np
            from PIL import Image

            if (
                not isinstance(data, np.ndarray)
                or data.dtype != np.uint8
                or data.shape not in ((h, w, 3), (h, w, 4))
            ):
                refuse(
                    "SENSOR_FORMAT",
                    "Expected bounded CPU uint8 RGB/RGBA data; no GPU-pointer fallback.",
                )
            array = np.array(data[:, :, :3], copy=True, order="C")
            filename = a["filename"]
            if not re.fullmatch(CAPTURE_NAME, filename):
                refuse("INVALID_OUTPUT", CAPTURE_NAME_RULE)
            s.output.mkdir(mode=0o700, parents=True, exist_ok=True)
            from twinsmith.project_assets import open_dir

            fd = open_dir(s.output)
            try:
                # Write beside it, then link: a no-replace publish, so a crash or a failed encode can't leave a
                # partial image at the name a retry would then be refused for.
                temp = f".{filename}.{os.getpid()}.tmp"
                try:
                    os.unlink(temp, dir_fd=fd)  # left by a crash under a reused PID
                except FileNotFoundError:
                    pass
                out = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=fd)
                try:
                    with os.fdopen(out, "wb") as stream:
                        Image.fromarray(array).save(stream, format="PNG")
                    os.link(temp, filename, src_dir_fd=fd, dst_dir_fd=fd, follow_symlinks=False)
                except FileExistsError:
                    refuse(
                        "CONFLICT",
                        f"{filename} already exists; use another filename. No overwrite. "
                        "The acquisition stays cached, so wait_for_render=false can save it.",
                    )
                finally:
                    os.unlink(temp, dir_fd=fd)
            finally:
                os.close(fd)
            file = s.output / filename
            return dict(
                **adoption,
                **_render_report(),
                file=str(file),
                width=array.shape[1],
                height=array.shape[0],
                sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                source=(
                    "Hydra renderer CPU callback (owned copy)"
                    if camera_frame.fresh
                    else "Cached owned CPU pixels from the last validated tool acquisition"
                ),
                adapter="hydra-renderer-cpu-v1",
                freshness=(
                    "new render and CPU transfer completed"
                    if camera_frame.fresh
                    else "cached acquisition; may predate Stop/reset, pose edits or physics steps"
                ),
            )
        refuse("UNKNOWN_TOOL", "No handler for " + name)
