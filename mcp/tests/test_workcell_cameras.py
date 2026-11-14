"""CPU-only checks for the work-cell wrist and overhead D415 camera slice.

The camera module is loaded by path, as the other work cell host tests do. No Kit,
pxr, GPU or asset store is touched: ``build_cameras`` uses a fake author seam and
a role-to-path pack double, so intrinsics, optical offsets and BOM tagging are
checked without a simulator.
"""

import copy
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "exts/workcell/workcell"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))


def _load(module_name, filename):
    spec = importlib.util.spec_from_file_location(module_name, PACKAGE / f"{filename}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


hardware = _load("workcell_test_hardware_config", "hardware_config")
cameras = _load("workcell_test_cameras", "cameras")

ROBOT = {
    "flange_path": "/World/UR5e/wrist_3_link/flange",
    "tool0_path": "/World/UR5e/wrist_3_link/flange/tool0",
    "gripper_path": "/World/UR5e/wrist_3_link/flange/tool0/rg2",
    "articulation_root": "/World/UR5e/root_joint",
    "robot_path": "/World/UR5e",
}


class Pack:
    """Pack double: logical role/id -> an absolute path outside the checkout."""

    def __init__(self, root, missing=(), substitute=None):
        self.root = Path(root)
        self.missing = set(missing)
        self.substitute = substitute

    def asset(self, asset_id):
        if asset_id in self.missing:
            raise FileNotFoundError(asset_id)
        if self.substitute is not None:
            return self.substitute
        return self.root / f"{asset_id}.usd"


@pytest.fixture()
def config():
    values, _ = hardware.load_config()
    values = copy.deepcopy(values)
    values["pack"] = {"id": "ur5e-jenga-workcell", "version": "v1"}
    values["assets"] = {role: role for role in cameras.REQUIRED_ASSET_ROLES}
    values["overhead_camera"]["enabled"] = True     # the shipped config is wrist-only; these tests cover both cameras
    return values


def build(config, tmp_path, **kwargs):
    pack = kwargs.pop("assets", None) or Pack(tmp_path)
    return cameras.build_cameras(
        object(), pack, config, ROBOT,
        author=lambda stage, report: {"authored": True}, **kwargs)


def test_report_shape_and_prim_paths(config, tmp_path):
    report = build(config, tmp_path)
    assert cameras.REQUIRED_ASSET_ROLES == (
        "d415", "wrist_mount", "overhead_mount", "overhead_release", "overhead_shelf")
    assert report["status"] == "ok"
    assert set(report) >= {"schema", "status", "cameras", "parts", "limitations", "provenance",
                           "required_config", "asset_conventions", "capture", "fiducials"}
    wrist = report["cameras"]["wrist"]
    overhead = report["cameras"]["overhead"]
    assert wrist["prim_path"] == "/World/UR5e/wrist_3_link/flange/tool0/wrist_d415/D415_color"
    assert overhead["prim_path"] == "/World/Workcell/cameras/overhead/D415_color"
    assert wrist["frames"]["optical"] == "wrist_camera_color_optical_frame"
    assert overhead["frames"]["optical"] == "overhead_camera_color_optical_frame"
    assert wrist["physics"]["attachment"] == ROBOT["tool0_path"]
    assert "none" in wrist["physics"]["rigid_body"]
    json.dumps(report)
    assert str(tmp_path) not in json.dumps(report)


def test_intrinsics_follow_config_resolution_and_fov(config, tmp_path):
    report = build(config, tmp_path)
    for key in ("wrist", "overhead"):
        section = config[f"{key}_camera"]
        resolution = section["resolution"]
        fov = section["color_fov_deg"]
        fx, fy, cx, cy = hardware.camera_intrinsics(resolution, fov)
        camera = report["cameras"][key]
        intrinsics = camera["intrinsics"]
        assert intrinsics["width"] == resolution[0]
        assert intrinsics["height"] == resolution[1]
        assert intrinsics["fx"] == pytest.approx(fx)
        assert intrinsics["fy"] == pytest.approx(fy)
        assert intrinsics["cx"] == pytest.approx(cx)
        assert intrinsics["cy"] == pytest.approx(cy)
        focal = fx * 36.0 / resolution[0]
        assert intrinsics["focal_length_mm"] == pytest.approx(focal)
        assert intrinsics["vertical_aperture_mm"] == pytest.approx(resolution[1] * focal / fy)
        assert camera["camera_info"]["K"] == pytest.approx(
            [fx, 0, cx, 0, fy, cy, 0, 0, 1])
        assert camera["camera_info"]["P"] == pytest.approx(
            [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0])
        assert camera["camera_info"]["D"] == [0.0] * 5
        assert camera["rgb"]["width"] == resolution[0]
        assert camera["rgb"]["encoding"] == "rgb8"
        assert camera["depth"]["annotator"] == "distance_to_image_plane"
        assert camera["depth"]["encoding"] == "32FC1"
        assert camera["depth"]["units"] == "m"


def test_wrist_offset_and_optical_to_usd_conversion(config, tmp_path):
    config["wrist_camera"]["rotation_in_tool0_deg"] = [0.0, 0.0, 0.0]
    wrist = build(config, tmp_path)["cameras"]["wrist"]
    offset = config["wrist_camera"]["offset_in_tool0_m"]
    assert wrist["optical"]["translation_m"] == pytest.approx(offset)
    assert wrist["optical"]["quaternion_wxyz"] == pytest.approx([1, 0, 0, 0])
    matrix = wrist["optical"]["matrix4x4_row_major"]
    assert [matrix[0][3], matrix[1][3], matrix[2][3]] == pytest.approx(offset)
    assert [matrix[row][2] for row in range(3)] == pytest.approx([0, 0, 1])
    assert wrist["usd_camera"]["translation_m"] == pytest.approx([0, 0, 0])
    assert wrist["usd_camera"]["quaternion_wxyz"] == pytest.approx([0, 1, 0, 0])
    assert wrist["usd_camera"]["optical_from_usd"]["rotation_deg_xyz"] == [180.0, 0.0, 0.0]

    rotated = copy.deepcopy(config)
    rotated["wrist_camera"]["rotation_in_tool0_deg"] = [0, 0, 90]
    camera = build(rotated, tmp_path)["cameras"]["wrist"]
    half = math.sqrt(0.5)
    assert camera["optical"]["quaternion_wxyz"] == pytest.approx([half, 0, 0, half])
    # The wrist root carries the optical rotation; the camera prim keeps only Rx(180 deg).
    assert camera["usd_camera"]["quaternion_wxyz"] == pytest.approx([0, 1, 0, 0])

    shifted = copy.deepcopy(config)
    shifted["wrist_camera"]["offset_in_tool0_m"] = [-0.1, -0.2, -0.3]
    camera = build(shifted, tmp_path)["cameras"]["wrist"]
    assert camera["optical"]["translation_m"] == pytest.approx([-0.1, -0.2, -0.3])
    bracket = next(part for part in build(shifted, tmp_path)["parts"]
                   if part["role"] == "wrist_mount")
    assert bracket["local"]["translation_m"] == pytest.approx([0.05, 0.1, 0.15])


def test_overhead_look_at_pose_and_reach_clearance(config, tmp_path):
    overhead = build(config, tmp_path)["cameras"]["overhead"]
    position = config["overhead_camera"]["position_m"]
    assert overhead["optical"]["translation_m"] == pytest.approx(position)
    matrix = overhead["optical"]["matrix4x4_row_major"]
    # Optical +Z looks from the camera at look_at; down (+Y) opposes the projected up_hint.
    target = config["overhead_camera"]["look_at_m"]
    forward = [t - p for t, p in zip(target, position)]
    norm = math.sqrt(sum(v * v for v in forward))
    assert [matrix[row][2] for row in range(3)] == pytest.approx([v / norm for v in forward])
    assert forward[2] < 0
    right = [matrix[row][0] for row in range(3)]
    down = [matrix[row][1] for row in range(3)]
    assert sum(r * f for r, f in zip(right, forward)) == pytest.approx(0, abs=1e-9)
    assert down[0] < 0
    reach = overhead["mount"]["reach_check"]
    assert reach["ok"] is True
    assert reach["arm_reach_m"] == pytest.approx(0.85)
    assert reach["camera_m"] > 0.85
    assert all(item["ground_m"] > 0.85 for item in reach["supports"])
    assert overhead["usd_camera"]["quaternion_wxyz"] != pytest.approx(
        overhead["optical"]["quaternion_wxyz"])


def test_gantry_outside_reach_and_removable_are_enforced(config, tmp_path):
    inside = copy.deepcopy(config)
    inside["overhead_mount"]["post_xy_m"] = [[0.2, -0.1], [0.3, 0.2]]
    with pytest.raises(ValueError, match="reach"):
        build(inside, tmp_path)

    camera_inside = copy.deepcopy(config)
    camera_inside["overhead_camera"]["position_m"] = [0.3, -0.1, 0.5]
    with pytest.raises(ValueError, match="reach"):
        build(camera_inside, tmp_path)

    fixed = copy.deepcopy(config)
    fixed["overhead_mount"]["removable"] = False
    with pytest.raises(ValueError, match="removable"):
        build(fixed, tmp_path)

    clamped = copy.deepcopy(config)
    clamped["overhead_mount"]["type"] = "clamp"
    with pytest.raises(ValueError, match="gantry"):
        build(clamped, tmp_path)


def test_parts_carry_asset_id_and_bom_tag(config, tmp_path):
    config["wrist_camera"]["offset_in_tool0_m"] = [0.012, 0.09, -0.005]   # farther out than the 60 mm bracket proxy
    report = build(config, tmp_path)
    parts = report["parts"]
    assert len(parts) == 6
    assert all({"prim_path", "asset_id", "bom_id", "role", "fidelity"} <= set(part) for part in parts)
    assert {part["bom_id"] for part in parts} == set(cameras.REQUIRED_ASSET_ROLES)
    assert len({part["prim_path"] for part in parts}) == 6
    assert [part for part in parts if part["role"] == "d415"] and len(
        [part for part in parts if part["role"] == "d415"]) == 2
    for part in parts:
        assert part["asset_id"] == part["role"]
        assert part["prim_path"].startswith("/")
        assert part["local"]["matrix4x4_row_major"][3] == [0, 0, 0, 1]
    mount = next(part for part in parts if part["role"] == "overhead_mount")
    posts = config["overhead_mount"]["post_xy_m"]
    height = config["overhead_mount"]["height_m"]
    span_y = abs(posts[0][1] - posts[1][1])
    assert mount["local"]["scale"] == pytest.approx([0.0381, span_y, height])
    assert mount["local"]["translation_m"] == pytest.approx(
        [posts[0][0], (posts[0][1] + posts[1][1]) / 2, height / 2])
    bracket = next(part for part in parts if part["role"] == "wrist_mount")
    offset = config["wrist_camera"]["offset_in_tool0_m"]
    assert bracket["local"]["scale"][1] > 1.0  # stretched to span the tool0-to-camera gap
    assert bracket["local"]["translation_m"][0] == pytest.approx(0.5 * -offset[0] * 1.0, abs=0.02)
    assert str(tmp_path) not in json.dumps(report)


def test_asset_layout_bom_fidelity_and_mount_pose(config, tmp_path):
    config["asset_layout"] = {
        "d415": {"bom_id": "CAM-D415", "fidelity": "placeholder"},
        "wrist_mount": {"bom_id": "BRK-WRIST-01"},
        "overhead_shelf": {
            "bom_id": "SH-OVERHEAD-01", "fidelity": "placeholder",
            "mount_pose": {"position_m": [0, 0, 0.02], "orientation_wxyz": [1, 0, 0, 0]},
        },
    }
    parts = {part["role"]: part for part in build(config, tmp_path)["parts"]}
    assert parts["d415"]["bom_id"] == "CAM-D415"
    assert parts["d415"]["fidelity"] == "placeholder"
    assert parts["wrist_mount"]["bom_id"] == "BRK-WRIST-01"
    assert parts["wrist_mount"]["fidelity"] == "unverified"
    assert parts["overhead_shelf"]["bom_id"] == "SH-OVERHEAD-01"
    assert parts["overhead_shelf"]["local"]["translation_m"][2] == pytest.approx(1.04)
    assert parts["overhead_shelf"]["pose_source"] == "config.asset_layout.overhead_shelf.mount_pose"


def test_missing_asset_is_partial_and_explicit(config, tmp_path):
    missing = copy.deepcopy(config)
    del missing["assets"]["overhead_shelf"]
    report = build(missing, tmp_path)
    assert report["status"] == "partial"
    assert report["cameras"]["overhead"]["status"] == "built"
    assert report["cameras"]["overhead"]["geometry_status"] == "partial"
    assert not any(part["bom_id"] == "overhead_shelf" for part in report["parts"])
    assert any("overhead_shelf" in item for item in report["limitations"])

    no_assets = copy.deepcopy(config)
    no_assets.pop("assets")
    report = build(no_assets, tmp_path)
    assert report["status"] == "partial"
    assert report["parts"] == []
    assert any("config.assets." in item for item in report["limitations"])


def test_missing_pose_marks_the_camera_unavailable(config, tmp_path):
    no_offset = copy.deepcopy(config)
    del no_offset["wrist_camera"]["offset_in_tool0_m"]
    report = build(no_offset, tmp_path)
    assert report["cameras"]["wrist"]["status"] == "unavailable"
    assert "offset_in_tool0_m" in report["cameras"]["wrist"]["reason"]
    assert report["cameras"]["overhead"]["status"] == "built"
    assert report["status"] == "partial"

    no_position = copy.deepcopy(config)
    del no_position["overhead_camera"]["position_m"]
    report = build(no_position, tmp_path)
    assert report["cameras"]["overhead"]["status"] == "unavailable"
    assert "position" in report["cameras"]["overhead"]["reason"]


def test_overhead_camera_is_left_out_when_disabled(config, tmp_path):
    config["overhead_camera"]["enabled"] = False
    report = build(config, tmp_path)
    assert report["status"] == "ok"
    assert report["cameras"]["wrist"]["status"] == "built"
    overhead = report["cameras"]["overhead"]
    assert overhead["status"] == "disabled" and overhead["prim_path"] is None and "wrist camera only" in overhead["reason"]
    assert {part["role"] for part in report["parts"]} == {"d415", "wrist_mount"}
    assert [root["name"] for root in report["roots"]] == ["wrist_mount"]
    config["overhead_mount"]["type"] = "clamp"      # the unused mount is not checked
    assert build(config, tmp_path)["status"] == "ok"
    config["overhead_camera"]["enabled"] = "no"
    with pytest.raises(ValueError, match="overhead_camera.enabled"):
        build(config, tmp_path)


def test_shipped_config_has_the_wrist_camera_only_on_the_underside():
    values, _ = hardware.load_config()
    assert values["overhead_camera"]["enabled"] is False
    wrist = values["wrist_camera"]
    assert wrist["offset_in_tool0_m"][1] > 0        # tool0 +y is down with the tool forward and wrist_3 at 0
    forward = cameras._rotate_vector(cameras._euler_xyz_deg_to_quat(wrist["rotation_in_tool0_deg"]), [0.0, 0.0, 1.0])
    assert forward[2] > 0.98 and forward[1] <= 0     # looks along the tool axis, angled toward it if at all
    assert 0.06 <= wrist["offset_in_tool0_m"][1] <= 0.15         # held out from the wrist by a bracket, not on the flange
    assert wrist["resolution"] == [1280, 720] and wrist["color_fov_deg"] == [69.4, 42.5]
    # The near plane clears the camera's own body proxy, or the camera renders the inside of its box.
    assert wrist["body_size_m"][2] / 2 < wrist["clipping_range_m"][0] <= 0.03


def test_topics_capture_and_fiducials_are_declared_not_verified(config, tmp_path):
    report = build(config, tmp_path)
    for key in ("wrist", "overhead"):
        topics = report["cameras"][key]["topics"]
        assert topics["status"] == "proposed"
        assert topics["verified"] is False
        assert "not published or verified" in topics["note"]
        assert topics["names"]
    assert report["capture"]["verified"] is False
    assert set(report["capture"]["functions"]) == {"make_camera_sensor", "read_frame"}
    assert report["fiducials"]["used"] is False
    text = json.dumps(report).lower()
    assert "apriltag" not in text
    assert "aruco" not in text
    assert "fiducial" in text  # only as an explicit statement that none is used
    assert any("pyrealsense2" in item for item in report["limitations"])


def test_d415_is_the_unconfirmed_proxy_and_never_d455(config, tmp_path):
    report = build(config, tmp_path)
    convention = report["asset_conventions"]["d415"]
    assert convention["dimensions_m"] == pytest.approx([0.099, 0.020, 0.023])
    assert convention["forbidden_substitute"] == "D455/rsd455"
    assert report["cameras"]["wrist"]["body_geometry"]["verified"] is False
    assert "unconfirmed dimensioned proxy" in json.dumps(report)
    assert report["asset_conventions"]["wrist_mount"]["dimensions_m"] == pytest.approx(
        [0.030, 0.060, 0.012])


def test_repository_internal_old_cache_and_d455_paths_are_refused(config, tmp_path):
    internal = build(config, tmp_path, assets=_FixedPack(ROOT / "exts/workcell/workcell/d415.usd"))
    assert internal["parts"] == []
    assert any("inside the checkout" in item for item in internal["limitations"])

    old_cache = build(config, tmp_path, assets=_FixedPack(tmp_path / "ur5e-digital-twin" / "d415.usd"))
    assert old_cache["parts"] == []
    assert any("old cache" in item for item in old_cache["limitations"])

    with pytest.raises(ValueError, match="D455"):
        build(config, tmp_path, assets=Pack(tmp_path, substitute=tmp_path / "rsd455.usd"))


class _FixedPack:
    def __init__(self, path):
        self.path = Path(path)

    def asset(self, asset_id):
        return self.path


def test_required_config_and_provenance_are_returned(config, tmp_path):
    report = build(config, tmp_path, config_path="/tmp/lab_hardware.json")
    required = report["required_config"]
    for key in ("assets.d415", "assets.wrist_mount", "assets.overhead_mount",
                "assets.overhead_release", "assets.overhead_shelf",
                "wrist_camera.offset_in_tool0_m", "wrist_camera.rotation_in_tool0_deg",
                "wrist_camera.resolution", "wrist_camera.color_fov_deg",
                "overhead_camera.position_m", "overhead_camera.look_at_m",
                "overhead_camera.resolution", "overhead_camera.color_fov_deg",
                "overhead_mount.post_xy_m", "overhead_mount.height_m",
                "overhead_mount.removable", "robot.reach_m",
                "interfaces.wrist_camera_topics", "interfaces.overhead_camera_topics"):
        assert key in required
    assert required["assets.d415"] == "d415"
    assert required["wrist_camera.offset_in_tool0_m"] == pytest.approx(
        config["wrist_camera"]["offset_in_tool0_m"])
    assert report["provenance"]["pack"] == {"id": "ur5e-jenga-workcell", "version": "v1"}
    assert report["provenance"]["assets"]["loader"] == "Pack"
    assert set(report["provenance"]["assets"]["resolved_roles"]) == set(cameras.REQUIRED_ASSET_ROLES)
    assert report["provenance"]["robot"]["tool0_path"] == ROBOT["tool0_path"]
    assert report["provenance"]["config"]["path"] == "/tmp/lab_hardware.json"
    assert str(tmp_path) not in json.dumps(report)


def test_build_validates_inputs_and_strips_internal_paths(config, tmp_path):
    with pytest.raises(ValueError, match="tool0_path"):
        cameras.plan_cameras(config, {"robot_path": "/World/R"}, assets=Pack(tmp_path))
    with pytest.raises(ValueError, match="stage"):
        cameras.build_cameras(None, Pack(tmp_path), config, ROBOT, author=lambda *_: None)

    seen = {}

    def author(stage, report):
        seen["stage"] = stage
        seen["report"] = report
        return {"authored": True}

    stage = object()
    report = cameras.build_cameras(stage, Pack(tmp_path), config, ROBOT, author=author)
    assert seen["stage"] is stage
    assert report["authoring"] == {"authored": True}
    assert all("_path" not in part for part in report["parts"])
    assert all("_path" in part for part in seen["report"]["parts"])

    raw = copy.deepcopy(config)
    raw["assets"]["d415"] = {"value": "d415", "status": "confirmed", "source": "test"}
    with pytest.raises(ValueError, match="flattened"):
        cameras.plan_cameras(raw, ROBOT, assets=Pack(tmp_path))


def test_authoring_without_pxr_refuses_instead_of_faking(config, tmp_path):
    if importlib.util.find_spec("pxr") is not None:
        pytest.skip("pxr is installed; the ImportError seam is not exercised")
    with pytest.raises(ImportError, match="pxr"):
        cameras.build_cameras(object(), Pack(tmp_path), config, ROBOT)


def test_authoring_creates_referenced_parts_and_cameras(config, tmp_path):
    pxr = pytest.importorskip("pxr")
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    pack = Pack(tmp_path)
    asset_paths = []
    for role in cameras.REQUIRED_ASSET_ROLES:
        path = tmp_path / f"{role}.usd"
        _write_asset(Usd, UsdGeom, UsdPhysics, path)
        asset_paths.append(path)
    before = {path: path.read_bytes() for path in asset_paths}

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, ROBOT["tool0_path"])

    report = cameras.build_cameras(stage, pack, config, ROBOT)
    assert report["authoring"]["cameras"] == [
        report["cameras"]["wrist"]["prim_path"], report["cameras"]["overhead"]["prim_path"]]
    assert len(report["authoring"]["parts"]) == 6
    assert report["authoring"]["rigid_bodies_remaining"] == []
    assert report["authoring"]["physics_removed"]
    assert not any("Rigid bodies remain" in item for item in report["limitations"])

    for part in report["parts"]:
        prim = stage.GetPrimAtPath(part["prim_path"])
        assert prim and prim.IsValid() and prim.HasAuthoredReferences()
        assert prim.GetCustomDataByKey("workcell:asset_id") == part["asset_id"]
        assert prim.GetCustomDataByKey("workcell:bom_id") == part["bom_id"]
        assert prim.GetCustomDataByKey("workcell:fidelity") == "unverified"
        prims = list(Usd.PrimRange(prim))
        assert not any(item.HasAPI(UsdPhysics.RigidBodyAPI) for item in prims)
        assert not any(item.HasAPI(UsdPhysics.CollisionAPI) for item in prims)
        assert not any(item.IsA(UsdPhysics.Joint) for item in prims)

    wrist = report["cameras"]["wrist"]
    root = stage.GetPrimAtPath(wrist["root_path"])
    assert root.GetParent().GetPath() == Sdf.Path(ROBOT["tool0_path"])
    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(root).GetOrderedXformOps()}
    assert tuple(ops["xformOp:translate"].Get()) == pytest.approx(
        config["wrist_camera"]["offset_in_tool0_m"])

    camera = UsdGeom.Camera(stage.GetPrimAtPath(wrist["prim_path"]))
    assert camera
    assert camera.GetFocalLengthAttr().Get() == pytest.approx(wrist["intrinsics"]["focal_length_mm"])
    assert camera.GetHorizontalApertureAttr().Get() == pytest.approx(36.0)
    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(camera.GetPrim()).GetOrderedXformOps()}
    orient = ops["xformOp:orient"].Get()
    assert orient.GetReal() == pytest.approx(0.0, abs=1e-12)
    assert tuple(orient.GetImaginary()) == pytest.approx((1.0, 0.0, 0.0))

    overhead = report["cameras"]["overhead"]
    camera = UsdGeom.Camera(stage.GetPrimAtPath(overhead["prim_path"]))
    assert camera
    assert camera.GetClippingRangeAttr().Get() == pytest.approx(overhead["depth"]["clipping_range_m"])

    mount_path = next(part["prim_path"] for part in report["parts"]
                      if part["role"] == "overhead_mount")
    mount_ops = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(mount_path)).GetOrderedXformOps()}
    posts = config["overhead_mount"]["post_xy_m"]
    assert tuple(mount_ops["xformOp:scale"].Get()) == pytest.approx(
        (0.0381, abs(posts[0][1] - posts[1][1]), config["overhead_mount"]["height_m"]))

    assert {path: path.read_bytes() for path in asset_paths} == before


def test_authoring_refuses_bad_stage_without_editing(config, tmp_path):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom

    pack = Pack(tmp_path)
    for role in cameras.REQUIRED_ASSET_ROLES:
        (tmp_path / f"{role}.usd").touch()

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError, match="metresPerUnit"):
        cameras.build_cameras(stage, pack, config, ROBOT)
    assert stage.GetRootLayer().ExportToString() == before

    empty = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(empty, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(empty, 1.0)
    before = empty.GetRootLayer().ExportToString()
    with pytest.raises(ValueError, match="not on the stage"):
        cameras.build_cameras(empty, pack, config, ROBOT)
    assert empty.GetRootLayer().ExportToString() == before


def _write_asset(Usd, UsdGeom, UsdPhysics, path):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.Cube.Define(stage, "/Asset/geom")
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/Asset/geom"))
    body = UsdGeom.Xform.Define(stage, "/Asset/body").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    assert stage.GetRootLayer().Save()


def test_capture_helper_uses_official_sensor_and_never_invents_pixels(monkeypatch):
    calls = {}

    class Sensor:
        def __init__(self, path, *, resolution, annotators=("rgb",)):
            calls.update(path=path, resolution=resolution, annotators=annotators)

        def get_data(self, annotator):
            return None, {}

    rtx = types.ModuleType("isaacsim.sensors.experimental.rtx")
    rtx.CameraSensor = Sensor
    for name in ("isaacsim", "isaacsim.sensors", "isaacsim.sensors.experimental"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, rtx.__name__, rtx)

    sensor = cameras.make_camera_sensor("/World/Workcell/cameras/wrist/D415_color", [1280, 720])
    assert calls == {"path": "/World/Workcell/cameras/wrist/D415_color",
                     "resolution": (720, 1280),
                     "annotators": ["rgb", "distance_to_image_plane"]}
    with pytest.raises(RuntimeError, match="No rgb frame"):
        cameras.read_frame(sensor)

    class SensorWithData(Sensor):
        def get_data(self, annotator):
            return ["renderer pixels"], {"annotator": annotator}

    data, info = cameras.read_frame(SensorWithData("/World/cam", resolution=(1, 1)))
    assert data == ["renderer pixels"]
    assert info == {"annotator": "rgb"}
