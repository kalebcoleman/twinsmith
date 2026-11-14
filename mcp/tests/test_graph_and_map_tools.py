"""sim.graph.node_types / inspect / show and sim.occupancy_map: the parts that run without Kit."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("sim_tools_graph_under_test", REPO / "exts/sim-tools/sim_tools/graph.py")
graph = importlib.util.module_from_spec(spec)
spec.loader.exec_module(graph)


@pytest.fixture
def np():
    return pytest.importorskip("numpy")


def test_ogn_ports_use_og_style_names_and_keep_defaults():
    ogn = {"isaacsim.ros2.bridge.ROS2PublishClock": {
        "inputs": {"topicName": {"type": "string", "default": "clock", "description": ["Name of", "ROS2 Topic"]},
                   "execIn": {"type": "execution", "description": "The input execution port"}},
        "outputs": {}}}
    ports = graph.ogn_ports(ogn, "isaacsim.ros2.bridge.ROS2PublishClock")
    assert ports["inputs"]["inputs:topicName"] == {"type": "string", "default": "clock",
                                                   "description": "Name of ROS2 Topic"}
    assert "inputs:execIn" in ports["inputs"] and ports["outputs"] == {}


RST = '''Inputs
------
.. csv-table::
    :header: "Name", "Type", "Descripton", "Default"
    "Array Size (*inputs:arraySize*)", "``int``", "The size of the array to construct.", "1"
    "", "Metadata", "*literalOnly* = 1", ""
    "Input0 (*inputs:input0*)", "``['bool', 'colord[3]', 'double', 'float', 'half', 'int', 'int64', 'token', 'uint']``", "Input array element", "None"
Outputs
-------
    "X (*outputs:x*)", "``double``", "The x component", "None"
'''


def test_rst_ports_parse_compiled_node_docs():
    ports = graph.rst_ports(RST)
    assert ports["inputs"]["inputs:arraySize"] == {"type": "int", "default": "1",
                                                   "description": "The size of the array to construct."}
    assert ports["inputs"]["inputs:input0"]["type"].startswith("any of 9 types")
    assert ports["outputs"] == {"outputs:x": {"type": "double", "description": "The x component"}}


def test_short_bounds_values():
    assert graph._short(list(range(20)))[-1] == "... 20 items"
    assert graph._short(float("nan")) == "nan" and graph._short("x" * 500) == "x" * 200


def _buffer_with(np, cells, dims_x, dims_y):
    """Isaac buffer: row-major, dims_x per row, in Isaac's (180-degree-rotated) layout."""
    buf = np.zeros(dims_y * dims_x, dtype=np.float32)
    for r, c in cells:
        buf[r * dims_x + c] = 1.0
    return buf.tolist()


def test_map_layout_matches_the_live_probe(np):
    # Live (Isaac Sim 6.2): boxes at world (2, 1) and (-1, -2) in a [-3, 3] map landed there after
    # the 180-degree rotation. Here: one occupied buffer cell must come out at the mirrored pixel.
    dims_x, dims_y = 4, 3
    pgm, origin, counts = graph.pgm_and_origin(_buffer_with(np, [(0, 0)], dims_x, dims_y), dims_x, dims_y,
                                               (-1.0, -0.5, 0.0), 0.5, 1.0)
    header, body = pgm.split(b"255\n", 1)
    img = np.frombuffer(body, dtype=np.uint8).reshape(dims_y, dims_x)
    assert b"4 3" in header and origin == (-1.0, -0.5)
    assert img[dims_y - 1, dims_x - 1] == 0 and (img == 0).sum() == 1  # buffer (0,0) -> bottom-right pixel
    assert counts == {"occupied": 1, "free": 11, "unknown": 0}
    assert set(np.unique(img)) <= {0, 254}


def test_unknown_cells_and_units(np):
    buf = [0.5] * 4
    pgm, origin, counts = graph.pgm_and_origin(buf, 2, 2, (-100.0, -50.0, 0.0), 5.0, 0.01)  # centimetre stage
    assert origin == (-1.0, -0.5) and counts["unknown"] == 4
    assert set(pgm.split(b"255\n", 1)[1]) == {205}


def test_clear_circles_mark_the_robot_footprint_free(np):
    dims = 10
    buf = np.ones(dims * dims).tolist()  # everything occupied
    pgm, origin, counts = graph.pgm_and_origin(buf, dims, dims, (0.0, 0.0, 0.0), 0.1, 1.0, clear=[[0.5, 0.5, 0.15]])
    img = np.frombuffer(pgm.split(b"255\n", 1)[1], dtype=np.uint8).reshape(dims, dims)
    # Cells whose centres are within 0.15 m of (0.5, 0.5): the 2x2 block around it (row 4-5 from top).
    assert counts["free"] == 4 and (img[4:6, 4:6] == 254).all()


def test_contracts_registered_with_the_expected_fields():
    sys.path.insert(0, str(REPO / "mcp"))
    from twinsmith.scene_contracts import REGISTRY

    assert {"contains", "limit", "ports"} <= set(REGISTRY["sim.graph.node_types"]["parameters"])
    assert {"path", "limit"} <= set(REGISTRY["sim.graph.inspect"]["parameters"])
    assert {"path", "layout", "width", "height"} <= set(REGISTRY["sim.graph.show"]["parameters"])
    occ = REGISTRY["sim.occupancy_map"]["parameters"]
    assert occ["min"]["required"] and occ["max"]["required"] and occ["cell_size"]["default"] == 0.05
    assert "clear" in occ and not occ["clear"]["required"]
    assert "start" in occ and not occ["start"]["required"]


def test_rst_rows_with_escaped_quotes_are_not_dropped():
    # Real case (ArraySlice): descriptions with "" quotes made the old regex skip the row.
    text = ('    "End (*inputs:end*)", "``int``", "The end index, exclusive. ""end"" may be negative.", "0"\n'
            '    "Use Length (*inputs:useLength*)", "``bool``", "Use ""length"" instead of ""end"".", "False"\n')
    ports = graph.rst_ports(text)["inputs"]
    assert set(ports) == {"inputs:end", "inputs:useLength"}
    assert ports["inputs:end"]["description"] == 'The end index, exclusive. "end" may be negative.'


def test_find_ogn_ignores_an_empty_extension_path():
    assert graph._find_ogn("", "omni.graph.nodes.ArraySlice") is None


def test_find_ogn_handles_ids_that_keep_the_ogn_prefix(tmp_path):
    # "Scale To/From Stage Units" is isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit,
    # and the lookup searched for OgnOgnIsaacScaleToFromStageUnit.ogn, so it reported no ports.
    nodes = tmp_path / "python" / "nodes"
    nodes.mkdir(parents=True)
    tid = "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"
    (nodes / "OgnIsaacScaleToFromStageUnit.ogn").write_text(json.dumps({tid: {
        "inputs": {"value": {"type": ["numerics"], "description": "Input value"}},
        "outputs": {"result": {"type": ["numerics"], "description": "Output value"}}}}))
    ports, _ = graph._find_ogn(str(tmp_path), tid)
    assert set(ports["inputs"]) == {"inputs:value"} and set(ports["outputs"]) == {"outputs:result"}
    # Ids without the prefix still find Ogn<name>.ogn.
    (nodes / "OgnConstructArray.ogn").write_text(json.dumps({"omni.graph.nodes.ConstructArray": {
        "inputs": {"arraySize": {"type": "int"}}, "outputs": {}}}))
    graph._OGN_INDEX.clear()
    ports, _ = graph._find_ogn(str(tmp_path), "omni.graph.nodes.ConstructArray")
    assert "inputs:arraySize" in ports["inputs"]


def test_cell_value_reads_the_start_cell(np):
    dims = 10
    buf = np.zeros(dims * dims)
    buf[0] = 1.0  # buffer cell 0 -> after the 180-degree turn, the bottom-right pixel
    pgm, origin, _ = graph.pgm_and_origin(buf.tolist(), dims, dims, (0.0, 0.0, 0.0), 0.1, 1.0)
    assert graph.cell_value(pgm, dims, dims, origin, 0.1, 0.95, 0.05) == 0     # bottom-right cell: occupied
    assert graph.cell_value(pgm, dims, dims, origin, 0.1, 0.05, 0.95) == 254   # top-left: free
    assert graph.cell_value(pgm, dims, dims, origin, 0.1, 5.0, 5.0) is None    # outside the map


def test_numbers_reject_booleans_and_wrong_lengths():
    assert graph._numbers([1, 2.5], 2) and not graph._numbers([True, 1], 2) and not graph._numbers([1], 2)
    assert not graph._numbers([1, float("inf")], 2)


def test_graph_paths_accept_unicode_and_refuse_property_paths(monkeypatch):
    pytest.importorskip("pxr")
    import types

    class Refused(Exception):
        pass

    def refuse(code, message, **kw):
        raise Refused(code)

    monkeypatch.setitem(sys.modules, "twinsmith.project_assets", types.SimpleNamespace(refuse=refuse))
    assert graph._graph_path("/World/Graphe_\u00e9") == "/World/Graphe_\u00e9"
    for bad in ("World/Graph", "/World/Graph.inputs:x", "/World/Graph{v=a}", "", "/World/a b", "/World/1abc"):
        with pytest.raises(Refused):
            graph._graph_path(bad)


def test_buffer_from_occupied_keeps_last_row_and_floods_from_start():
    # 119 cells: native getDimensions truncates this span to 118, and the last-row leaf overruns its buffer.
    cell = 0.05
    lo, hi = (-3.0, -3.0), (2.95, 2.95)
    wall = [(-3.0 + (i + 0.5) * cell, 2.925) for i in range(119)]  # the whole last row
    buf, w, h, on_wall = graph.buffer_from_occupied(wall, lo, hi, cell, (-2.0, -2.0))
    assert not on_wall
    assert (w, h, len(buf)) == (119, 119, 119 * 119)
    assert buf.count(1.0) == 119 and buf[118 * 119:] == [1.0] * 119
    assert buf.count(0.0) == 118 * 119 and buf.count(0.5) == 0


def test_buffer_from_occupied_leaves_unreachable_cells_unknown():
    wall = [(0.25, 0.05 + i * 0.1) for i in range(4)]  # a column that splits the 4x4 map
    buf, w, h, _ = graph.buffer_from_occupied(wall, (0.0, 0.0), (0.4, 0.4), 0.1, (0.05, 0.05))
    assert (w, h) == (4, 4) and buf.count(1.0) == 4 and buf.count(0.0) == 8 and buf.count(0.5) == 4


def test_a_start_on_a_wall_is_reported_instead_of_erasing_the_wall():
    wall = [(0.05 + i * 0.1, 0.05) for i in range(3)]  # touches the start
    other = [(0.35, 0.35)]
    buf, _, _, on_wall = graph.buffer_from_occupied(wall + other, (0.0, 0.0), (0.4, 0.4), 0.1, (0.05, 0.05))
    assert on_wall and buf.count(1.0) == 1  # the wall was erased, the far cell was not
    counts = {"occupied": 1, "free": 15, "unknown": 0}
    assert "occupied cell" in graph.start_blocked(counts, [254], False, on_wall)
    # A floor filling the slice erases everything: that stays the raise-the-slice case, not a blocked start.
    assert graph.start_blocked({"occupied": 0, "free": 16, "unknown": 0}, [254], False, True) is None


def test_start_blocked_decisions():
    walls_mostly_unknown = {"occupied": 50, "free": 100, "unknown": 850}  # big bounds, walls exist
    assert graph.start_blocked(walls_mostly_unknown, [254]) is None
    open_field = {"occupied": 0, "free": 1000, "unknown": 0}
    assert graph.start_blocked(open_field, [254]) is None
    inside = {"occupied": 0, "free": 40, "unknown": 960}  # seen live with start inside a box
    assert "inside an obstacle" in graph.start_blocked(inside, [254])
    assert graph.start_blocked(open_field, [0]) == "is occupied"
    assert graph.start_blocked(open_field, [205]) == "is not in mapped free space"
    assert "inside a collider" in graph.start_blocked(open_field, [254], start_in_collider=True)


def test_start_on_a_cell_edge_checks_the_touching_cells(np):
    dims = 10
    buf = np.zeros(dims * dims)
    pgm, origin, _ = graph.pgm_and_origin(buf.tolist(), dims, dims, (0.0, 0.0, 0.0), 0.1, 1.0)
    assert len(graph.cells_at(pgm, dims, dims, origin, 0.1, 0.3, 0.3)) == 4   # a corner: four cells
    assert len(graph.cells_at(pgm, dims, dims, origin, 0.1, 0.35, 0.35)) == 1  # a cell centre: one


def test_map_slice_rises_past_a_floor_only_while_nothing_is_occupied():
    raised = graph.raised_slice(0.05, 0.3, 0.05, 0, 0)
    assert raised == pytest.approx(0.11)
    assert graph.raised_slice(0.05, 0.3, 0.05, 12, 0) is None  # a real map: leave it alone
    assert graph.raised_slice(0.05, 0.3, 0.05, 0, graph.FLOOR_RETRIES) is None
    assert graph.raised_slice(0.28, 0.3, 0.05, 0, 0) is None  # no room left in the slice


def test_raised_slice_note_does_not_claim_lower_geometry_is_never_mapped():
    note = graph.slice_bottom_note(0.05)
    assert "before 2.3.3" in note and "0.05" in note
    assert "anything lower" not in note.lower()
    assert "still include colliders below" in note
