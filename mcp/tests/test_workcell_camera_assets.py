"""CPU-only checks for the script that builds the D415 body and wrist bracket assets (no download, no FreeCAD)."""

import importlib.util
import math
import struct
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("workcell_camera_assets", ROOT / "scripts/workcell_camera_assets.py")
assets = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(assets)


def test_the_vendor_mesh_is_pinned_by_commit_and_checksum():
    assert assets.COMMIT in assets.MESH_URL and len(assets.COMMIT) == 40
    assert assets.MESH_URL.startswith("https://raw.githubusercontent.com/realsenseai/realsense-ros/")
    assert len(assets.MESH_SHA256) == 64 and int(assets.MESH_SHA256, 16)


def test_binary_stl_round_trip_and_weld(tmp_path):
    triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0], [0, 1, 0]]], dtype="<f4")
    path = tmp_path / "two.stl"
    path.write_bytes(b"\0" * 80 + struct.pack("<I", 2) + b"".join(
        struct.pack("<3f", 0, 0, 1) + triangle.tobytes() + b"\0\0" for triangle in triangles))
    read = assets.read_stl(path)
    assert read.shape == (2, 3, 3) and np.allclose(read, triangles)
    points, faces = assets.weld(read)
    assert len(points) == 4 and faces.shape == (2, 3) and np.allclose(points[faces], triangles)
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(SystemExit, match="binary STL"):
        assets.read_stl(path)


def test_colour_optical_frame_follows_the_vendor_offsets():
    """camera_link is x forward, y left, z up; the colour sensor is 15 mm to the left of its origin."""
    roll, _, yaw = assets.MESH_RPY
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])

    def from_link(point):       # the mesh point that lands on a camera_link point, in millimetres
        return ((np.array(point) - np.array(assets.MESH_XYZ)) @ (rz @ rx)) * 1000.0

    colour, ahead, left, up = (from_link(p) for p in ((0, 0.015, 0), (0.1, 0.015, 0), (0, 0.115, 0), (0, 0.015, 0.1)))
    filler = np.array([500.0, 0, 0])                                   # makes the mesh span over a metre: read as mm
    optical = assets.d415_optical(np.array([[colour, ahead, left], [colour, up, filler]]))
    assert optical[0, 0] == pytest.approx([0, 0, 0], abs=1e-9)          # the colour sensor is the origin
    assert optical[0, 1] == pytest.approx([0, 0, 0.1], abs=1e-9)        # forward is +z
    assert optical[0, 2] == pytest.approx([-0.1, 0, 0], abs=1e-9)       # left is -x
    assert optical[1, 1] == pytest.approx([0, -0.1, 0], abs=1e-9)       # up is -y
