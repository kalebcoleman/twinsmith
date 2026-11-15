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


def same_rotation(first, second):
    """Two unit quaternions that are the same rotation (q and -q are)."""
    return abs(sum(a * b for a, b in zip(first, second))) == pytest.approx(1.0)


def test_wrist_pose_in_the_rg2_frame_and_optical_to_usd_conversion(config, tmp_path):
    """The pose is configured in the RG2 body frame; with the shipped stack that frame is tool0 moved 47.92 mm along
    the tool axis (Bota 32 mm + Quick Changer 15.92 mm) and turned +90 degrees about it: the RG2's front face (body -X)
    is on tool0 -Y, the side of wrist 3's tool connector, as on the real arm."""
    rg2_t, rg2_q, length = cameras.rg2_in_tool0(config)
    assert length == pytest.approx(config["bota"]["length_m"] + config["quick_changer"]["standoff_m"])
    assert rg2_t == pytest.approx([0, 0, length])
    half = math.sqrt(0.5)
    assert same_rotation(rg2_q, [half, 0, 0, half])
    assert cameras._rotate_vector(rg2_q, [0, 0, 1]) == pytest.approx([0, 0, 1])      # approach axis = tool0 +Z
    assert cameras._rotate_vector(rg2_q, [0, 1, 0]) == pytest.approx([-1, 0, 0])     # finger-opening axis
    assert cameras._rotate_vector(rg2_q, [1, 0, 0]) == pytest.approx([0, 1, 0])      # out of the back face: tool0 +Y
    assert cameras._rotate_vector(rg2_q, [-1, 0, 0]) == pytest.approx([0, -1, 0])    # the front face: the connector's side

    config["wrist_camera"]["rotation_in_rg2_deg"] = [0.0, 0.0, 0.0]
    config["wrist_camera"]["offset_in_rg2_m"] = [0.01, -0.02, 0.03]
    wrist = build(config, tmp_path)["cameras"]["wrist"]
    offset = [0.02, 0.01, 0.03 + length]
    assert wrist["optical"]["translation_m"] == pytest.approx(offset)
    assert same_rotation(wrist["optical"]["quaternion_wxyz"], rg2_q)
    assert wrist["mount"]["offset_in_tool0_m"] == pytest.approx(offset)
    assert wrist["mount"]["frame"] == "rg2" and wrist["mount"]["stack_length_m"] == pytest.approx(length)
    matrix = wrist["optical"]["matrix4x4_row_major"]
    assert [matrix[0][3], matrix[1][3], matrix[2][3]] == pytest.approx(offset)
    assert [matrix[row][2] for row in range(3)] == pytest.approx([0, 0, 1])
    assert wrist["usd_camera"]["translation_m"] == pytest.approx([0, 0, 0])
    assert wrist["usd_camera"]["quaternion_wxyz"] == pytest.approx([0, 1, 0, 0])
    assert wrist["usd_camera"]["optical_from_usd"]["rotation_deg_xyz"] == [180.0, 0.0, 0.0]

    rotated = copy.deepcopy(config)
    rotated["wrist_camera"]["rotation_in_rg2_deg"] = [0, 0, 90]
    camera = build(rotated, tmp_path)["cameras"]["wrist"]
    assert same_rotation(camera["optical"]["quaternion_wxyz"], [0, 0, 0, 1])         # +90 (stack) + 90 (camera)
    # The wrist root carries the optical rotation; the camera prim keeps only Rx(180 deg).
    assert camera["usd_camera"]["quaternion_wxyz"] == pytest.approx([0, 1, 0, 0])
    bracket = next(part for part in build(rotated, tmp_path)["parts"] if part["role"] == "wrist_mount")
    assert bracket["local"]["quaternion_wxyz"] == pytest.approx([half, 0, 0, -half])
    assert bracket["local"]["translation_m"] == pytest.approx([0.02, 0.01, -0.03])   # the RG2 frame, seen from the wrist root


def test_wrist_camera_tilt_is_carried_through_the_pose_math(config, tmp_path):
    """A tilt about the camera's long axis needs no special case: the tool0 pose and the bracket's placement follow."""
    config["wrist_camera"]["offset_in_rg2_m"] = [0.07, -0.03, 0.12]
    config["wrist_camera"]["rotation_in_rg2_deg"] = [30.0, 0.0, -90.0]
    report = build(config, tmp_path)
    wrist = report["cameras"]["wrist"]
    rg2_t, rg2_q, length = cameras.rg2_in_tool0(config)
    optical = wrist["optical"]["quaternion_wxyz"]
    half, sine, cosine = math.sqrt(0.5), math.sin(math.radians(30)), math.cos(math.radians(30))
    # In the RG2 body frame the optical axis is (-sin, 0, cos); the body's +X is tool0 +Y and its +Y is tool0 -X.
    assert cameras._rotate_vector(optical, [0, 0, 1]) == pytest.approx([0, -sine, cosine])
    assert cameras._rotate_vector(optical, [1, 0, 0]) == pytest.approx([1, 0, 0])
    assert cameras._rotate_vector(optical, [0, 1, 0]) == pytest.approx([0, cosine, sine])
    assert wrist["optical"]["translation_m"] == pytest.approx([0.03, 0.07, 0.12 + length])
    bracket = next(part for part in report["parts"] if part["role"] == "wrist_mount")["local"]
    # The bracket asset is in the RG2 body frame: seen from the camera it is the inverse of the camera's pose there.
    in_rg2 = cameras._euler_xyz_deg_to_quat([30.0, 0.0, -90.0])
    assert bracket["quaternion_wxyz"] == pytest.approx(cameras._quat_conjugate(in_rg2))
    back = cameras._rotate_vector(in_rg2, bracket["translation_m"])
    assert [value + offset for value, offset in zip(back, [0.07, -0.03, 0.12])] == pytest.approx([0, 0, 0], abs=1e-12)
    assert cameras._rotate_vector(in_rg2, [0, 0, 1]) == pytest.approx([-sine, 0, cosine])
    assert same_rotation(rg2_q, [half, 0, 0, half])


def test_wrist_camera_follows_the_rg2_when_the_stack_changes(config, tmp_path):
    """The bracket is on the gripper: a longer stack moves the camera in tool0 and leaves it where it was on the RG2."""
    before = build(config, tmp_path)
    longer = copy.deepcopy(config)
    longer["bota"]["length_m"] += 0.02
    longer["stack"]["order"] = ["quick_changer", "bota", "rg2"]
    after = build(longer, tmp_path)
    old, new = (report["cameras"]["wrist"]["optical"]["translation_m"] for report in (before, after))
    assert new == pytest.approx([old[0], old[1], old[2] + 0.02])

    def bracket(report):
        return next(part for part in report["parts"] if part["role"] == "wrist_mount")["local"]

    assert bracket(after) == bracket(before)

    turned = copy.deepcopy(config)          # the RG2 mounted a quarter turn further round the tool axis
    turned["asset_layout"]["rg2"]["mount_pose"]["orientation_wxyz"] = list(cameras._quat_mul(
        config["asset_layout"]["rg2"]["mount_pose"]["orientation_wxyz"], [math.sqrt(0.5), math.sqrt(0.5), 0, 0]))   # inlet +X is the tool axis
    moved = build(turned, tmp_path)["cameras"]["wrist"]["optical"]["translation_m"]
    assert moved[2] == pytest.approx(old[2])
    assert math.hypot(moved[0], moved[1]) == pytest.approx(math.hypot(old[0], old[1]))
    assert [moved[0], moved[1]] != pytest.approx([old[0], old[1]])
    assert bracket(build(turned, tmp_path)) == bracket(before)

    unknown = copy.deepcopy(config)
    del unknown["quick_changer"]["standoff_m"]
    report = build(unknown, tmp_path)
    assert report["cameras"]["wrist"]["status"] == "unavailable"
    assert "quick_changer" in report["cameras"]["wrist"]["reason"]


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
    offset = config["wrist_camera"]["offset_in_rg2_m"]
    # The bracket is modelled in the RG2 body frame, so it sits on that frame: the inverse of the camera's pose in it.
    assert "scale" not in bracket["local"]
    inverse = cameras._quat_conjugate(cameras._euler_xyz_deg_to_quat(config["wrist_camera"]["rotation_in_rg2_deg"]))
    assert bracket["local"]["quaternion_wxyz"] == pytest.approx(inverse)
    assert bracket["local"]["translation_m"] == pytest.approx(cameras._rotate_vector(inverse, [-v for v in offset]))
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
    del no_offset["wrist_camera"]["offset_in_rg2_m"]
    report = build(no_offset, tmp_path)
    assert report["cameras"]["wrist"]["status"] == "unavailable"
    assert "offset_in_rg2_m" in report["cameras"]["wrist"]["reason"]
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


def test_shipped_config_has_the_wrist_camera_only_on_the_rg2_bracket():
    values, _ = hardware.load_config()
    assert values["overhead_camera"]["enabled"] is False
    wrist = values["wrist_camera"]
    assert "offset_in_tool0_m" not in wrist and "rotation_in_tool0_deg" not in wrist     # derived, never typed
    x, y, z = wrist["offset_in_rg2_m"]
    # Off the RG2's back face (+X; the body is 36 mm thick), clear of the plane the fingers move in, level with the body.
    assert 0.018 + 0.0115 + 0.005 <= x <= 0.12
    assert abs(y) <= 0.1 and 0.07 <= z <= 0.148
    rotation = cameras._euler_xyz_deg_to_quat(wrist["rotation_in_rg2_deg"])
    forward = cameras._rotate_vector(rotation, [0.0, 0.0, 1.0])
    tilt = wrist["rotation_in_rg2_deg"][0]
    assert wrist["rotation_in_rg2_deg"][1:] == [0.0, -90.0] and 10.0 <= tilt <= 40.0
    # Tilted about its long axis toward the tool axis (the camera is on the body's +X side), so it sees the fingertips.
    assert forward == pytest.approx([-math.sin(math.radians(tilt)), 0.0, math.cos(math.radians(tilt))])
    assert cameras._rotate_vector(rotation, [1.0, 0.0, 0.0]) == pytest.approx([0.0, -1.0, 0.0])   # long axis along the finger-opening axis
    assert cameras._rotate_vector(rotation, [0.0, 1.0, 0.0])[0] > 0.75           # image down is away from the gripper
    # The optical axis crosses the tool axis beyond the fingertips (0.217 m from the RG2 origin), within reach of them.
    crossing = z + x / math.tan(math.radians(tilt))
    assert 0.217 < crossing < 0.40
    fingertip = [-x, 0.0, 0.217 - z]                                # a fingertip on the tool axis, from the camera, RG2 axes
    along = sum(a * b for a, b in zip(fingertip, forward))
    down = sum(a * b for a, b in zip(fingertip, cameras._rotate_vector(rotation, [0.0, 1.0, 0.0])))
    assert along > 0 and -wrist["color_fov_deg"][1] / 2 < math.degrees(math.atan2(down, along)) < 0     # in the upper half of the image
    report = cameras.plan_cameras(values, ROBOT)
    in_tool0 = report["cameras"]["wrist"]["optical"]["translation_m"]
    assert in_tool0[2] == pytest.approx(z + values["bota"]["length_m"] + values["quick_changer"]["standoff_m"])
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
    assert report["asset_conventions"]["wrist_mount"]["axes"] == "RG2 body frame"


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
                "wrist_camera.offset_in_rg2_m", "wrist_camera.rotation_in_rg2_deg",
                "wrist_camera.resolution", "wrist_camera.color_fov_deg",
                "overhead_camera.position_m", "overhead_camera.look_at_m",
                "overhead_camera.resolution", "overhead_camera.color_fov_deg",
                "overhead_mount.post_xy_m", "overhead_mount.height_m",
                "overhead_mount.removable", "robot.reach_m",
                "interfaces.wrist_camera_topics", "interfaces.overhead_camera_topics"):
        assert key in required
    assert required["assets.d415"] == "d415"
    assert required["wrist_camera.offset_in_rg2_m"] == pytest.approx(
        config["wrist_camera"]["offset_in_rg2_m"])
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
    # These plain assets carry no collision scope, so the bracket gets no collider, and the report says so.
    assert [item["status"] for item in report["authoring"]["colliders"]] == ["absent"]
    assert any("has no collision approximation" in item for item in report["limitations"])
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
    assert tuple(ops["xformOp:translate"].Get()) == pytest.approx(wrist["mount"]["offset_in_tool0_m"])

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


RG2_BODY = ROBOT["gripper_path"] + "/onrobot_rg2_base_link"


def test_bracket_collider_is_planned_on_the_rg2_body(config, tmp_path):
    """The bracket clamps the gripper, so its collider belongs to the RG2 body: same body, no contact with it."""
    report = build(config, tmp_path)
    (collider,) = report["colliders"]
    assert collider["body_path"] == RG2_BODY and collider["prim_path"] == RG2_BODY + "/wrist_mount_collision"
    assert collider["source_prim"] == cameras.COLLISION_SCOPE == "Collision"
    assert collider["filtered_pairs"] == [] and collider["mass"] == "none added"
    assert collider["asset_id"] == "wrist_mount" and "_path" not in collider
    wrist = report["cameras"]["wrist"]
    assert wrist["physics"]["attachment"] == ROBOT["tool0_path"]            # the camera prims themselves stay on tool0
    assert wrist["physics"]["bracket_collider"] == collider
    assert "bracket_collider" not in report["cameras"]["overhead"]["physics"]
    assert any("belongs to the RG2 body and adds no mass" in item for item in report["limitations"])

    elsewhere = copy.deepcopy(config)           # modelled in a frame that is not the rigid body: not attached, and said
    elsewhere["asset_layout"]["rg2"]["mount_frame"] = "some_other_frame"
    report = build(elsewhere, tmp_path)
    assert report["colliders"] == [] and "bracket_collider" not in report["cameras"]["wrist"]["physics"]
    assert any("collider is not attached" in item for item in report["limitations"])

    no_bracket = build(config, tmp_path, assets=Pack(tmp_path, missing={"wrist_mount"}))
    assert no_bracket["colliders"] == []


def _write_bracket(Usd, UsdGeom, UsdPhysics, path, approximation="convexHull", rigid=False):
    """A bracket asset with a render mesh and a collision scope, as scripts/workcell_bracket_assets.py writes it."""
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/Asset").GetPrim())
    UsdGeom.Mesh.Define(stage, "/Asset/Body")
    scope = UsdGeom.Scope.Define(stage, "/Asset/Collision")
    UsdGeom.Imageable(scope.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    for name in ("collar", "truss"):
        mesh = UsdGeom.Mesh.Define(stage, f"/Asset/Collision/{name}")
        mesh.CreatePointsAttr([(x, y, z) for x in (0, 0.01) for y in (0, 0.01) for z in (0, 0.01)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(approximation)
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Asset/Collision/collar"))
    assert stage.GetRootLayer().Save()


def _bracket_stage(tmp_path, config, rigid_body=True, **bracket):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics

    for role in cameras.REQUIRED_ASSET_ROLES:
        _write_asset(Usd, UsdGeom, UsdPhysics, tmp_path / f"{role}.usd")
    (tmp_path / "wrist_mount.usd").unlink()
    _write_bracket(Usd, UsdGeom, UsdPhysics, tmp_path / "wrist_mount.usd", **bracket)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, ROBOT["tool0_path"])
    body = UsdGeom.Xform.Define(stage, RG2_BODY).GetPrim()
    if rigid_body:
        UsdPhysics.RigidBodyAPI.Apply(body)
    return stage, Usd, UsdGeom, UsdPhysics


def test_bracket_collider_is_authored_under_the_rg2_body_and_nowhere_else(config, tmp_path):
    stage, Usd, UsdGeom, UsdPhysics = _bracket_stage(tmp_path, config)
    report = cameras.build_cameras(stage, Pack(tmp_path), config, ROBOT)
    (authored,) = report["authoring"]["colliders"]
    assert authored["status"] == "attached" and authored["body_path"] == RG2_BODY
    assert [(Path(shape["prim_path"]).name, shape["type"], shape["approximation"]) for shape in authored["shapes"]] == [
        ("collar", "Mesh", "convexHull"), ("truss", "Mesh", "convexHull")]
    root = stage.GetPrimAtPath(RG2_BODY + "/wrist_mount_collision")
    assert root.HasAuthoredReferences() and root.GetCustomDataByKey("workcell:bom_id") == "wrist_mount"
    assert UsdGeom.Imageable(root).ComputePurpose() == UsdGeom.Tokens.guide
    shapes = [prim for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(shapes) == 2 and not any(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in Usd.PrimRange(root))
    for prim in shapes:         # the nearest rigid body above every shape is the RG2 body
        owner = prim.GetParent()
        while not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        assert str(owner.GetPath()) == RG2_BODY
    # The copy under tool0 (the visual bracket) and the D415 body keep no collider and no rigid body.
    visual = stage.GetPrimAtPath(report["cameras"]["wrist"]["root_path"])
    assert not any(prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in Usd.PrimRange(visual))
    assert every_collider_is_under(stage, Usd, UsdPhysics, RG2_BODY + "/wrist_mount_collision")


def every_collider_is_under(stage, Usd, UsdPhysics, path):
    return all(str(prim.GetPath()).startswith(path) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI))


def test_bracket_collider_refuses_a_body_that_is_not_rigid_and_shapes_that_are_not_convex(config, tmp_path):
    stage, *_ = _bracket_stage(tmp_path, config, rigid_body=False)
    with pytest.raises(ValueError, match="is not a rigid body"):
        cameras.build_cameras(stage, Pack(tmp_path), config, ROBOT)
    for folder, bracket, message in (("mesh", dict(approximation="none"), "explicit convexHull"),
                                     ("nested", dict(rigid=True), "shapes only")):
        (tmp_path / folder).mkdir()
        stage, *_ = _bracket_stage(tmp_path / folder, config, **bracket)
        with pytest.raises(ValueError, match=message):
            cameras.build_cameras(stage, Pack(tmp_path / folder), config, ROBOT)


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
