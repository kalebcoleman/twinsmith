"""sam3: prompt and source checks, the wrapper around a fake launcher, and the driver's geometry (numpy only).

The SAM 3 model itself is not run here: it needs the bundle's environment, its weights and a GPU.
"""

import asyncio
import json

import pytest

from twinsmith import sam3 as sam3_module
from twinsmith import sibling
from twinsmith.sam3 import Sam3, check_source, fuse_boxes, parse_prompts, parse_zones

BLOCK = [0.072, 0.024, 0.014]
CAMERA = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}


def code(call):
    with pytest.raises(sibling.Refusal) as caught:
        call()
    return caught.value.code


def clip(root, frames=3, depth=True, camera=CAMERA):
    """A recorded clip folder; the files are empty, which is enough for everything short of the model."""
    (root / "rgb").mkdir(parents=True)
    for index in range(frames):
        (root / "rgb" / f"{index:05d}.png").write_bytes(b"")
    if depth:
        (root / "depth").mkdir()
        for index in range(frames):
            (root / "depth" / f"{index:05d}.npy").write_bytes(b"")
    if camera is not None:
        (root / "camera.json").write_text(json.dumps(camera))
    return root


def test_prompts_are_tag_equals_text_and_a_box_size_belongs_to_a_tag():
    parsed = parse_prompts(["tower=tower of wooden blocks", "blocks= wooden block"], {"blocks": BLOCK})
    assert parsed == [{"tag": "tower", "text": "tower of wooden blocks"},
                      {"tag": "blocks", "text": "wooden block", "size_m": BLOCK}]
    for bad in (["wooden block"], ["Blocks=wooden block"], ["a=x", "a=y"], ["frames=x"], [], ["a="]):
        assert code(lambda: parse_prompts(bad)) == "INVALID_VALUE"
    assert code(lambda: parse_prompts(["a=x"], {"b": BLOCK})) == "INVALID_VALUE"
    assert code(lambda: parse_prompts(["a=x"], {"a": [0.072, 0.024]})) == "INVALID_VALUE"
    assert code(lambda: parse_prompts(["a=x"], {"a": [0.072, 0.024, 0.0]})) == "INVALID_VALUE"


def test_a_source_is_a_video_file_or_a_clip_folder(tmp_path):
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")
    assert check_source(str(video)) == {"source": str(video), "kind": "video", "frames": None, "camera": None}
    checked = check_source(str(clip(tmp_path / "a", frames=5)), every=2)
    assert (checked["kind"], checked["frames"], checked["camera"]) == ("frames", 3, CAMERA)
    assert check_source(str(clip(tmp_path / "b", depth=False, camera=None)))["camera"] is None
    assert code(lambda: check_source("relative/clip.mp4")) == "INVALID_VALUE"
    (tmp_path / "notes.txt").write_text("x")
    assert code(lambda: check_source(str(tmp_path / "notes.txt"))) == "SOURCE_INVALID"
    assert code(lambda: check_source(str(tmp_path))) == "SOURCE_INVALID"


def test_a_clip_with_depth_needs_matching_frames_and_a_usable_camera(tmp_path):
    uneven = clip(tmp_path / "a")
    (uneven / "depth" / "00002.npy").unlink()
    assert code(lambda: check_source(str(uneven))) == "SOURCE_INVALID"
    assert code(lambda: check_source(str(clip(tmp_path / "b", camera=None)))) == "CAMERA_INVALID"
    assert code(lambda: check_source(str(clip(tmp_path / "c", camera={"fx": 600.0})))) == "CAMERA_INVALID"
    assert code(lambda: check_source(str(clip(tmp_path / "d", camera=dict(CAMERA, fx=0))))) == "CAMERA_INVALID"
    assert code(lambda: check_source(str(clip(tmp_path / "e", camera=dict(CAMERA, world_from_camera=[[1, 0, 0]]))))) == "CAMERA_INVALID"
    moving = dict(CAMERA, frames=[{"world_from_camera": [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]}] * 2)
    assert code(lambda: check_source(str(clip(tmp_path / "f", camera=moving)))) == "CAMERA_INVALID"   # 2 poses, 3 frames


def wrapper(tmp_path, launch=None, job_status=None):
    """A Sam3 over a fake skills checkout whose SAM 3 interpreter exists but is never run."""
    python = tmp_path / "tools/open-robot-skills/tools/sam3/.venv/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text("")
    launched = []

    def fake_launch(argv, **options):
        launched.append({"argv": [str(part) for part in argv], **options})
        return {"ok": True, "job_id": "job-1", "log_dir": str(tmp_path), "command": argv}

    return Sam3(workspace=tmp_path, launch=launch or fake_launch, job_status=job_status), launched, python


def test_track_writes_the_request_and_starts_one_gpu_job_with_a_scrubbed_environment(tmp_path):
    sam3, launched, python = wrapper(tmp_path)
    source = clip(tmp_path / "clip")
    answer = sam3.call("track", source=str(source), name="first", prompts=["blocks=wooden block"], box_size_m={"blocks": BLOCK})
    assert answer["ok"] and answer["job_id"] == "job-1" and answer["depth"] and answer["frames"] == 3
    folder = tmp_path / "runs/sam3/first"
    request = json.loads((folder / "request.json").read_text())
    assert request["prompts"] == [{"tag": "blocks", "text": "wooden block", "size_m": BLOCK}]
    assert request["camera"] == CAMERA and request["out"] == str(folder) and request["every"] == 1
    assert (folder / "driver.py").read_text() == sam3_module._DRIVER.read_text()
    (call,) = launched
    assert call["argv"] == [str(python), "-I", str(folder / "driver.py"), str(folder / "request.json")]
    assert call["uses_gpu"] is True and call["allow_concurrent_gpu"] is False
    assert call["env_pass"] == ("HF_HOME",) and call["env_set"]["HF_HUB_OFFLINE"] == "1"   # no token reaches the job
    assert sam3.call("track", source=str(source), name="first", prompts=["blocks=wooden block"])["code"] == "ALREADY_EXISTS"


def test_track_refuses_before_it_makes_a_folder(tmp_path):
    sam3, launched, _ = wrapper(tmp_path)
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")
    assert sam3.call("track", source=str(video), name="a", prompts=["blocks=wooden block"],
                     box_size_m={"blocks": BLOCK})["code"] == "DEPTH_MISSING"
    assert sam3.call("track", source=str(video), name="a", prompts=["blocks=wooden block"], start_s=2, end_s=1)["code"] == "INVALID_VALUE"
    assert sam3.call("track", source=str(clip(tmp_path / "clip")), name="a", prompts=["blocks=wooden block"], start_s=1)["code"] == "INVALID_VALUE"
    assert sam3.call("track", source=str(video), name="a", prompts=["blocks=wooden block"], every=0)["code"] == "INVALID_VALUE"
    assert not launched and not (tmp_path / "runs/sam3").exists()


def test_a_refused_launch_leaves_no_run_folder(tmp_path):
    busy = lambda argv, **options: {"ok": False, "code": "GPU_BUSY", "message": "A Kit bridge is running.", "hint": "wait"}
    sam3, _, _ = wrapper(tmp_path, launch=busy)
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")
    answer = sam3.call("track", source=str(video), name="a", prompts=["blocks=wooden block"])
    assert (answer["ok"], answer["code"]) == (False, "GPU_BUSY") and not (tmp_path / "runs/sam3/a").exists()


def test_a_missing_environment_is_reported_with_the_owner_steps(tmp_path):
    sam3 = Sam3(workspace=tmp_path)
    for action in ("status", "track"):
        answer = sam3.call(action, source="/x.mp4", prompts=["a=b"])
        assert (answer["ok"], answer["code"]) == (False, "ENVIRONMENT_MISSING") and "never installs" in answer["setup"]


def test_report_follows_the_job_and_returns_one_frame_on_request(tmp_path):
    states = iter(["running", "failed"])
    sam3, _, _ = wrapper(tmp_path, job_status=lambda job_id: {"state": next(states), "exit_code": 1})
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")
    sam3.call("track", source=str(video), name="a", prompts=["blocks=wooden block"])
    assert sam3.call("report", name="a")["state"] == "running"
    assert sam3.call("report", name="a")["code"] == "TRACK_FAILED"
    folder = tmp_path / "runs/sam3/a"
    rows = [{"frame": 0, "t": 0.0, "objects": [{"id": 1}]}, {"frame": 1, "t": 0.1, "objects": []}]
    (folder / "blocks.json").write_text(json.dumps({"frames": rows}))
    (folder / "result.json").write_text(json.dumps({"frames": 2, "prompts": [{"tag": "blocks", "objects": 1}]}))
    assert sam3.call("report", name="a")["state"] == "finished"
    assert sam3.call("report", name="a", tag="blocks", frame=-1)["frame"] == rows[1]
    assert sam3.call("report", name="a", tag="blocks")["frame"] == rows[0]
    assert sam3.call("report", name="a", tag="hand")["code"] == "INVALID_VALUE"
    assert sam3.call("report", name="a", tag="blocks", frame=2)["code"] == "INVALID_VALUE"
    assert sam3.call("report", name="nope")["code"] == "RUN_NOT_FOUND"


def test_the_tool_is_one_grouped_tool_that_returns_json(tmp_path):
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("test")
    sam3_module.register_sam3_tools(server, sam3=wrapper(tmp_path)[0])
    content, _ = asyncio.run(server.call_tool("sam3", {"action": "report", "name": "nope"}))
    assert json.loads(content[0].text)["code"] == "RUN_NOT_FOUND"


def test_a_zone_names_two_prompt_tags_and_goes_into_the_request(tmp_path):
    assert parse_zones({"hand": "tower"}, ["tower", "hand"]) == [["hand", "tower"]] and parse_zones(None, ["a"]) == []
    for bad in ({"hand": "hand"}, {"hand": "table"}, {"glove": "tower"}):
        assert code(lambda: parse_zones(bad, ["tower", "hand"])) == "INVALID_VALUE"
    sam3, _, _ = wrapper(tmp_path)
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")
    sam3.call("track", source=str(video), name="z", prompts=["tower=tower of blocks", "hand=hand"], zone={"hand": "tower"})
    assert json.loads((tmp_path / "runs/sam3/z/request.json").read_text())["zones"] == [["hand", "tower"]]
    assert sam3.call("track", source=str(video), name="y", prompts=["hand=hand"], zone={"hand": "tower"})["code"] == "INVALID_VALUE"


def seen(sam_id, centre, long_axis="x", face="end", error=0.003):
    """A box pose row as the driver writes it: the box's long side along world x or y."""
    quaternion = [0.0, 0.0, 0.0, 1.0] if long_axis == "x" else [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]
    return {"id": sam_id, "pose": {"face": face, "extent_error_m": error, "position_world_m": list(centre), "quaternion_world_xyzw": quaternion}}


def test_views_are_fused_into_one_box_per_block():
    front = [seen(1, (0.700, 0.000, 0.021)), seen(2, (0.700, 0.024, 0.021)),               # two neighbours, by their ends
             seen(3, (0.676, 0.000, 0.035), "y", "side"), seen(4, (0.70, 0.0, 0.09), error=0.05), {"id": 5}]
    back = [seen(7, (0.702, 0.001, 0.021)), seen(8, (0.700, 0.000, 0.035), "y", "end", error=0.002)]   # the first again; a crossed block above it
    boxes = fuse_boxes([("front", front), ("back", back)], BLOCK)
    assert len(boxes) == 4                                    # the bad face and the row without a pose are left out
    twice = next(box for box in boxes if box["detections"] == 2)
    assert twice["views"] == ["back", "front"] and twice["position_world_m"] == pytest.approx([0.701, 0.0005, 0.021])
    assert twice["spread_m"] == pytest.approx(0.00112, abs=1e-5) and twice["long_axis_world"] == pytest.approx([1.0, 0.0, 0.0])
    crossed = [box for box in boxes if abs(box["long_axis_world"][1]) > 0.9]
    assert len(crossed) == 2 and {box["faces"][0] for box in crossed} == {"end", "side"}       # 14 mm above, turned: its own box
    assert [round(box["position_world_m"][2], 3) for box in boxes] == [0.021, 0.021, 0.035, 0.035]


def test_fuse_reads_finished_runs_and_writes_one_file(tmp_path):
    sam3, _, _ = wrapper(tmp_path)
    for run, rows in (("front", [seen(1, (0.7, 0.0, 0.021))]), ("back", [seen(1, (0.701, 0.0, 0.021)), seen(2, (0.7, 0.024, 0.021))])):
        folder = tmp_path / "runs/sam3" / run
        folder.mkdir(parents=True)
        (folder / "request.json").write_text(json.dumps({"prompts": [{"tag": "blocks", "text": "wooden block", "size_m": BLOCK}]}))
        (folder / "blocks.json").write_text(json.dumps({"frames": [{"frame": 0, "objects": rows}]}))
        (folder / "result.json").write_text(json.dumps({"frames": 1, "prompts": [{"tag": "blocks"}]}))
    answer = sam3.call("fuse", name="tower", runs=["front", "back"], tag="blocks")
    assert answer["ok"] and (answer["boxes"], answer["views"], answer["seen_by_one_view"], answer["detections_used"]) == (2, 2, 1, 3)
    saved = json.loads((tmp_path / "runs/sam3/tower/fused.json").read_text())
    assert saved["size_m"] == BLOCK and len(saved["boxes"]) == 2 and saved["runs"] == ["front", "back"]
    assert sam3.call("fuse", name="tower", runs=["front", "back"], tag="blocks")["code"] == "ALREADY_EXISTS"
    assert sam3.call("fuse", name="t2", runs=["front", "nope"], tag="blocks")["code"] == "RUN_NOT_FOUND"
    assert sam3.call("fuse", name="t3", runs=["front", "front"], tag="blocks")["code"] == "INVALID_VALUE"
    assert sam3.call("fuse", name="t4", runs=["front"], tag="hand")["code"] == "INVALID_VALUE"


# ---- the driver's geometry: numpy only, no model ----------------------------------------------

def face_points(np, driver, face, rotation, centre, noise=0.0, count=40):
    """Points on one face of a block at ``centre`` with axes ``rotation`` (columns), seen from the origin."""
    long_axis, short_axis, hidden = driver.FACES[face]
    grid = np.stack(np.meshgrid(np.linspace(-0.5, 0.5, count), np.linspace(-0.5, 0.5, count)), -1).reshape(-1, 2)
    local = np.zeros((len(grid), 3))
    local[:, long_axis], local[:, short_axis] = grid[:, 0] * BLOCK[long_axis], grid[:, 1] * BLOCK[short_axis]
    outward = rotation[:, hidden] if rotation[:, hidden] @ centre < 0 else -rotation[:, hidden]
    points = centre + local @ rotation.T + outward * BLOCK[hidden] / 2
    return points + np.random.RandomState(1).normal(0, noise, points.shape)


def turned(np, yaw, tilt):
    cy, sy, ct, st = np.cos(yaw), np.sin(yaw), np.cos(tilt), np.sin(tilt)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]]) @ np.array([[1.0, 0, 0], [0, ct, -st], [0, st, ct]])


@pytest.mark.parametrize("face", ["end", "side", "top"])
def test_a_box_pose_comes_from_the_one_face_the_camera_sees(face):
    np = pytest.importorskip("numpy")
    from twinsmith import sam3_driver as driver

    rotation, centre = turned(np, 0.4, 0.3), np.array([0.03, -0.02, 0.45])
    hidden = driver.FACES[face][2]
    # Turn the block so that the wanted face looks at the camera.
    rotation = rotation @ {0: np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]]),
                           1: np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]]), 2: np.eye(3)}[hidden]
    world_from_camera = np.eye(4)
    world_from_camera[:3, 3] = [1.0, 2.0, 3.0]
    pose = driver.box_pose(face_points(np, driver, face, rotation, centre, noise=0.0003), BLOCK, world_from_camera.tolist())
    assert pose["face"] == face and pose["extent_error_m"] < 0.003 and pose["plane_rms_m"] < 0.001
    assert np.linalg.norm(np.array(pose["position_m"]) - centre) < 0.0015
    assert np.allclose(np.array(pose["position_world_m"]) - pose["position_m"], [1.0, 2.0, 3.0])
    x, y, z, w = pose["quaternion_xyzw"]
    found = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    assert np.linalg.det(found) > 0.999
    assert all(abs(found[:, axis] @ rotation[:, axis]) > 0.995 for axis in range(3))   # a box is the same after a half turn


def test_the_larger_of_two_visible_faces_gives_the_pose_and_too_few_points_give_none():
    np = pytest.importorskip("numpy")
    from twinsmith import sam3_driver as driver

    centre = np.array([0.0, 0.0, 0.5])
    side = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])       # the 72 x 14 face toward the camera
    points = np.concatenate([face_points(np, driver, "side", side, centre),
                             face_points(np, driver, "end", side, centre, count=12)])
    pose = driver.box_pose(points, BLOCK)
    assert pose["face"] == "side" and np.linalg.norm(np.array(pose["position_m"]) - centre) < 0.0015
    assert 0.8 < pose["face_fraction"] < 1.0 and "position_world_m" not in pose
    assert driver.box_pose(points[:10], BLOCK) is None


def test_a_mask_is_backprojected_through_the_intrinsics():
    np = pytest.importorskip("numpy")
    from twinsmith import sam3_driver as driver

    depth = np.full((480, 640), 0.5, np.float32)
    depth[240, 322] = np.nan
    mask = np.zeros((480, 640), bool)
    mask[240, 320:324] = True
    points = driver.backproject(depth, mask, CAMERA)
    assert points.shape == (3, 3) and np.allclose(points[:, 2], 0.5)
    assert np.allclose(points[:, 0], [0.0, 0.5 / 600, 3 * 0.5 / 600]) and np.allclose(points[:, 1], 0.0)


def test_frame_objects_numbers_objects_by_first_appearance_and_keeps_small_ones_visible():
    np = pytest.importorskip("numpy")
    from twinsmith import sam3_driver as driver

    masks = np.zeros((3, 20, 30), bool)
    masks[0, 2:18, 2:28] = True              # large
    masks[1, 8:10, 5:25] = True              # small and wide, inside the large one
    remap = {}
    label, rows = driver.frame_objects(np.array([7, 3, 9]), masks, np.array([0.9, 0.8, 0.1]), remap)
    assert remap == {7: 1, 3: 2}             # the empty mask gets no id
    assert label[9, 10] == 2 and label[3, 3] == 1 and label[0, 0] == 0
    assert [row["id"] for row in rows] == [1, 2] and rows[1]["area"] == 40 and rows[1]["bbox"] == [5, 8, 24, 9]
    assert min(rows[1]["axis_deg"], 180 - rows[1]["axis_deg"]) < 1 and rows[1]["score"] == pytest.approx(0.8)
    label, rows = driver.frame_objects(np.array([3]), masks[1:2], np.array([0.7]), remap)
    assert [row["id"] for row in rows] == [2]                      # the same object keeps its number
    tinted = driver.overlay(np.zeros((20, 30, 3), np.uint8), [label])
    assert tinted[9, 10].any() and not tinted[0, 0].any()


def test_a_hand_is_in_the_tower_zone_when_its_box_is_near_and_its_depth_agrees(tmp_path):
    pytest.importorskip("numpy")
    from twinsmith import sam3_driver as driver

    tower = [{"id": 1, "bbox": [560, 100, 720, 610], "xyz_m": [0.0, 0.0, 0.45]}]
    touching = {"id": 1, "bbox": [730, 300, 900, 420], "xyz_m": [0.1, 0.0, 0.47]}
    beside = {"id": 2, "bbox": [960, 300, 1100, 420]}                      # within half the tower's height of it, no depth
    far_left = {"id": 3, "bbox": [0, 300, 300, 420], "xyz_m": [-0.3, 0.0, 0.45]}
    in_front = {"id": 4, "bbox": [600, 300, 700, 420], "xyz_m": [0.0, 0.0, 0.10]}   # over the tower in the picture, 35 cm before it
    assert driver.in_zone([touching, beside, far_left, in_front], tower) == [1, 2]
    assert driver.in_zone([touching], []) == []
    frames = lambda rows: {"frames": [{"frame": index, "objects": objects} for index, objects in enumerate(rows)]}
    (tmp_path / "tower.json").write_text(json.dumps(frames([tower, tower, tower, tower])))
    (tmp_path / "hand.json").write_text(json.dumps(frames([[], [far_left], [touching, far_left], []])))
    (summary,) = driver.mark_zones(tmp_path, [["hand", "tower"]])
    assert summary == {"tag": "hand", "zone": "tower", "frames": 4, "frames_present": 1, "first_present_frame": 2,
                       "last_present_frame": 2, "present_in_last_frame": False}
    marked = json.loads((tmp_path / "hand.json").read_text())["frames"][2]["objects"]
    assert [o["in_zone"] for o in marked] == [True, False]
