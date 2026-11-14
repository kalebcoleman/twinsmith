"""Extra unit tests for twinsmith.isaaclab.metrics.

Complements tests/test_isaaclab_metrics.py (which covers the happy path) with
the edge cases around TensorBoard event-file discovery and parsing:

* missing / empty / non-directory run dirs
* run dirs that contain no event files (or only look-alike files)
* corrupt, truncated and zero-byte event files
* scalar tag filtering (including the unknown-tag error path)
* merging several event files that belong to one run
* the ``last_n`` window of ``tensorboard_tail``
* ordering of events that were written out of step order
* NaN / infinite scalar values

Every test writes a *real* TensorBoard event file when the ``tensorboard``
package is importable (it ships with the ``isaaclab`` extra); tests that need
the package are skipped when it is missing.
"""

import asyncio
import json
import os
import struct

import pytest


@pytest.fixture(autouse=True)
def _allow_tmp(monkeypatch):
    """Allow pytest's tmp_path (under the system temp dir) as a log root."""
    monkeypatch.setenv("ISAACLAB_ALLOW_TMP", "1")


from twinsmith.isaaclab import metrics  # noqa: E402


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _write_events(directory, tag, values, steps=None):
    """Write a real TensorBoard event file with ``tag`` -> ``values``."""
    pytest.importorskip("tensorboard")
    from tensorboard.compat.proto import event_pb2, summary_pb2
    from tensorboard.summary.writer.event_file_writer import EventFileWriter

    directory.mkdir(parents=True, exist_ok=True)
    if steps is None:
        steps = list(range(len(values)))
    writer = EventFileWriter(str(directory))
    try:
        for wall, step, value in zip(range(len(values)), steps, values):
            event = event_pb2.Event(wall_time=1000.0 + wall, step=step)
            event.summary.value.append(
                summary_pb2.Summary.Value(tag=tag, simple_value=float(value))
            )
            writer.add_event(event)
    finally:
        writer.close()
    return directory


def _event_files(directory):
    return sorted(
        name for name in os.listdir(directory) if name.startswith("events.out.tfevents.")
    )


def _tfrecord_frames(raw):
    """Yield (start, end) byte ranges of the TFRecord frames in ``raw``.

    A frame is ``uint64 length | uint32 masked_crc | payload | uint32 crc``.
    """
    frames = []
    offset = 0
    while offset + 12 <= len(raw):
        (length,) = struct.unpack("<Q", raw[offset : offset + 8])
        end = offset + 12 + length + 4
        if end > len(raw):
            break
        frames.append((offset, end))
        offset = end
    return frames


def _frames_with_tag(raw, tag):
    """Return the frame ranges that carry a scalar summary for ``tag``."""
    from tensorboard.compat.proto import event_pb2

    tagged = []
    for start, end in _tfrecord_frames(raw):
        event = event_pb2.Event()
        event.ParseFromString(raw[start + 12 : end - 4])
        if event.HasField("summary") and any(
            value.tag == tag for value in event.summary.value
        ):
            tagged.append((start, end))
    return tagged


def _truncate_after_n_tagged_events(path, tag, n):
    """Cut the event file right after its n-th event for ``tag``.

    Returns the number of ``tag`` events that survive the cut.
    """
    raw = open(path, "rb").read()
    tagged = _frames_with_tag(raw, tag)
    assert len(tagged) >= n, "not enough events to truncate"
    cut = tagged[n - 1][1]
    open(path, "wb").write(raw[:cut])
    return n


# --------------------------------------------------------------------------
# missing / empty / unusable run dirs
# --------------------------------------------------------------------------


def test_metrics_summary_missing_run_dir_returns_empty_scalars(tmp_path):
    """A run dir that does not exist yields no scalars (and no exception)."""
    missing = tmp_path / "does-not-exist"
    res = metrics.metrics_summary(run_dir=str(missing))
    assert "error" not in res
    assert res["scalars"] == {}


def test_tensorboard_tail_missing_run_dir_reports_unknown_tag(tmp_path):
    missing = tmp_path / "does-not-exist"
    res = metrics.tensorboard_tail(run_dir=str(missing), tag="loss")
    assert res.get("error") == "unknown tag"
    assert res["available"] == []


def test_metrics_summary_empty_run_dir_returns_empty_scalars(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    res = metrics.metrics_summary(run_dir=str(run))
    assert res["scalars"] == {}


def test_metrics_summary_run_dir_is_a_file_returns_empty_scalars(tmp_path):
    """A regular file passed as run_dir is walked as an empty tree."""
    path = tmp_path / "not-a-dir"
    path.write_text("nope")
    res = metrics.metrics_summary(run_dir=str(path))
    assert res["scalars"] == {}


def test_metrics_summary_ignores_lookalike_non_event_files(tmp_path):
    """Only ``events.out.tfevents.*`` files are treated as event files."""
    run = tmp_path / "run"
    run.mkdir()
    (run / "events.out.tfevents").write_bytes(b"not an event file")
    (run / "events.out.tfeventsX").write_bytes(b"not an event file")
    (run / "notes.txt").write_text("hello")
    (run / "events.out.tfevents.1").write_bytes(b"not an event file")
    res = metrics.metrics_summary(run_dir=str(run))
    assert res["scalars"] == {}


def test_metrics_summary_run_dir_none_requires_run_dir():
    res = metrics.metrics_summary(run_dir=None)
    assert res.get("error") == "run_dir is required"


def test_tensorboard_tail_run_dir_none_requires_run_dir():
    res = metrics.tensorboard_tail(run_dir=None, tag="loss")
    assert res.get("error") == "run_dir is required"


# --------------------------------------------------------------------------
# path safety
# --------------------------------------------------------------------------


def _isolate_roots(monkeypatch, allowed):
    """Make ``allowed`` the only log root; the system temp dir is *not* allowed."""
    monkeypatch.setenv("ISAACLAB_ALLOW_TMP", "0")
    monkeypatch.setenv("ISAACLAB_LOGS_DIR", str(allowed))


def test_run_dir_symlink_to_outside_root_is_refused(tmp_path, monkeypatch):
    """The *run dir* itself may not escape the allowed roots."""
    allowed = tmp_path / "allowed"
    allowed.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    _write_events(outside, "loss", [1.0])
    _isolate_roots(monkeypatch, allowed)
    link = allowed / "link"
    link.symlink_to(outside)
    res = metrics.metrics_summary(run_dir=str(link))
    assert "error" in res
    assert "outside allowed roots" in res["error"]


def test_run_dir_outside_allowed_roots_is_refused(tmp_path, monkeypatch):
    allowed = tmp_path / "allowed"
    allowed.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    _write_events(outside, "loss", [1.0])
    _isolate_roots(monkeypatch, allowed)
    res = metrics.metrics_summary(run_dir=str(outside))
    assert res.get("error") == "path outside allowed roots"
    res = metrics.tensorboard_tail(run_dir=str(outside), tag="loss")
    assert res.get("error") == "path outside allowed roots"


def test_event_file_symlink_to_outside_root_is_ignored(tmp_path, monkeypatch):
    allowed = tmp_path / "allowed"
    allowed.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    _write_events(outside, "secret", [42.0])
    run = allowed / "run"
    run.mkdir()
    source = os.path.join(str(outside), _event_files(outside)[0])
    os.symlink(source, os.path.join(str(run), "events.out.tfevents.linked"))
    _isolate_roots(monkeypatch, allowed)

    res = metrics.metrics_summary(run_dir=str(run))
    assert "secret" not in res.get("scalars", {})


# --------------------------------------------------------------------------
# corrupt / truncated event files
# --------------------------------------------------------------------------


def test_corrupt_event_file_is_skipped_without_error(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "events.out.tfevents.corrupt").write_bytes(b"\x00\x01not a tfrecord")
    res = metrics.metrics_summary(run_dir=str(run))
    assert res["scalars"] == {}


def test_corrupt_event_file_does_not_hide_valid_event_file(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "events.out.tfevents.corrupt").write_bytes(b"garbage garbage garbage")
    _write_events(run, "loss", [1.0, 0.5])
    res = metrics.metrics_summary(run_dir=str(run))
    assert res["scalars"]["loss"]["count"] == 2
    assert res["scalars"]["loss"]["last"] == 0.5


def test_zero_byte_event_file_is_ignored(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "events.out.tfevents.empty").write_bytes(b"")
    res = metrics.metrics_summary(run_dir=str(run))
    assert res["scalars"] == {}


def test_truncated_event_file_keeps_the_complete_events(tmp_path):
    """A cut event file must not lose the records that are intact."""
    pytest.importorskip("tensorboard")
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0, 3.0, 4.0, 5.0])
    path = os.path.join(str(run), _event_files(run)[0])

    survived = _truncate_after_n_tagged_events(path, "loss", 2)
    assert survived == 2

    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["count"] == 2
    assert summary["last"] == 2.0
    assert summary["min"] == 1.0
    assert summary["max"] == 2.0

    tail = metrics.tensorboard_tail(run_dir=str(run), tag="loss")
    assert [point["value"] for point in tail["points"]] == [1.0, 2.0]


def test_partially_written_final_event_is_dropped(tmp_path):
    """A half-written trailing record is discarded, earlier ones survive."""
    pytest.importorskip("tensorboard")
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0, 3.0, 4.0, 5.0])
    path = os.path.join(str(run), _event_files(run)[0])

    raw = open(path, "rb").read()
    tagged = _frames_with_tag(raw, "loss")
    # cut in the middle of the last tagged frame
    start = tagged[-1][0]
    open(path, "wb").write(raw[: start + 8])

    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["count"] == len(tagged) - 1
    assert summary["last"] == 4.0


# --------------------------------------------------------------------------
# scalar tag filtering
# --------------------------------------------------------------------------


def test_metrics_summary_tags_filter_selects_only_requested_tags(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 0.5])
    _write_events(run, "reward", [2.0, 3.0])
    res = metrics.metrics_summary(run_dir=str(run), tags=["loss"])
    assert set(res["scalars"]) == {"loss"}
    assert res["scalars"]["loss"]["count"] == 2


def test_metrics_summary_tags_filter_accepts_several_tags(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0])
    _write_events(run, "reward", [2.0])
    _write_events(run, "ignored", [3.0])
    res = metrics.metrics_summary(run_dir=str(run), tags=["loss", "reward"])
    assert set(res["scalars"]) == {"loss", "reward"}


def test_metrics_summary_empty_tag_list_returns_no_scalars(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0])
    res = metrics.metrics_summary(run_dir=str(run), tags=[])
    assert res["scalars"] == {}


def test_metrics_summary_unknown_tag_lists_available_tags(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0])
    _write_events(run, "reward", [2.0])
    res = metrics.metrics_summary(run_dir=str(run), tags=["nope"])
    assert res.get("error") == "unknown tag"
    assert set(res["available"]) == {"loss", "reward"}


def test_metrics_summary_unknown_tag_available_list_is_capped_at_50(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    for i in range(55):
        _write_events(run, "tag%02d" % i, [float(i)])
    res = metrics.metrics_summary(run_dir=str(run), tags=["nope"])
    assert res.get("error") == "unknown tag"
    assert len(res["available"]) == 50


def test_metrics_summary_duplicate_tags_do_not_double_count(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0, 3.0])
    once = metrics.metrics_summary(run_dir=str(run), tags=["loss"])
    twice = metrics.metrics_summary(run_dir=str(run), tags=["loss", "loss"])
    assert once["scalars"]["loss"]["count"] == 3
    assert twice["scalars"]["loss"]["count"] == 3


def test_metrics_summary_accepts_a_single_tag_given_as_a_string(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0])
    res = metrics.metrics_summary(run_dir=str(run), tags="loss")
    assert res["scalars"]["loss"]["count"] == 2


# --------------------------------------------------------------------------
# several event files in one run
# --------------------------------------------------------------------------


def test_metrics_summary_merges_event_files_of_one_run(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0])
    _write_events(run, "loss", [3.0, 4.0])
    assert len(_event_files(run)) == 2
    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["count"] == 4
    assert summary["min"] == 1.0
    assert summary["max"] == 4.0
    assert summary["last"] == 4.0


def test_metrics_summary_merges_distinct_tags_across_event_files(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0])
    _write_events(run, "reward", [2.0])
    res = metrics.metrics_summary(run_dir=str(run))
    assert set(res["scalars"]) == {"loss", "reward"}


def test_tensorboard_tail_merges_points_from_several_event_files(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, 2.0])
    _write_events(run, "loss", [3.0, 4.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=50)
    steps = [point["step"] for point in res["points"]]
    assert steps == sorted(steps)
    assert len(res["points"]) == 4


def test_tensorboard_tail_unknown_tag_lists_available_tags(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="nope")
    assert res.get("error") == "unknown tag"
    assert "loss" in res["available"]


# --------------------------------------------------------------------------
# last_n window
# --------------------------------------------------------------------------


def _write_series(run, tag, values):
    _write_events(run, tag, values)
    return run


def test_tensorboard_tail_default_last_n_caps_at_fifty_points(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [float(i) for i in range(60)])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss")
    assert len(res["points"]) == 50
    assert res["points"][-1]["value"] == 59.0


def test_tensorboard_tail_last_n_zero_returns_every_point(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0, 3.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=0)
    assert len(res["points"]) == 3


def test_tensorboard_tail_negative_last_n_returns_every_point(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0, 3.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=-5)
    assert len(res["points"]) == 3


def test_tensorboard_tail_last_n_none_returns_every_point(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0, 3.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=None)
    assert len(res["points"]) == 3


def test_tensorboard_tail_last_n_larger_than_run_returns_every_point(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0, 3.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=999)
    assert len(res["points"]) == 3


def test_tensorboard_tail_last_n_keeps_the_highest_steps(tmp_path):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0, 3.0, 4.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=2)
    assert [point["value"] for point in res["points"]] == [3.0, 4.0]


@pytest.mark.parametrize("bad", [1.5, "5", True, [1]])
def test_tensorboard_tail_rejects_a_non_integer_last_n(tmp_path, bad):
    run = _write_series(tmp_path / "run", "loss", [1.0, 2.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss", last_n=bad)
    assert res.get("error") == "last_n must be an integer or null"


# --------------------------------------------------------------------------
# ordering
# --------------------------------------------------------------------------


def test_metrics_summary_sorts_events_by_step(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [5.0, 1.0, 3.0], steps=[2, 0, 1])
    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["count"] == 3
    assert summary["last"] == 5.0
    assert summary["min"] == 1.0
    assert summary["max"] == 5.0
    assert summary["step"] == 2


def test_tensorboard_tail_sorts_points_by_step(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [5.0, 1.0, 3.0], steps=[2, 0, 1])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss")
    assert [point["step"] for point in res["points"]] == [0, 1, 2]
    assert [point["value"] for point in res["points"]] == [1.0, 3.0, 5.0]


# --------------------------------------------------------------------------
# NaN / infinite values
# --------------------------------------------------------------------------


def test_metrics_summary_ignores_nan_for_min_and_max(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [float("nan"), 1.0, 2.0])
    summary = metrics.metrics_summary(run_dir=str(run))["scalars"]["loss"]
    assert summary["min"] == 1.0
    assert summary["max"] == 2.0
    assert summary["count"] == 3


def test_metrics_summary_reports_nan_as_a_json_string(tmp_path):
    """Non-finite values become 'nan'/'inf'/'-inf' so the result is strict JSON."""
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, float("nan")])
    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["last"] == "nan"
    assert summary["min"] == 1.0
    assert summary["max"] == 1.0
    json.dumps(res, allow_nan=False)


def test_metrics_summary_reports_infinite_values_as_json_strings(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, float("inf"), float("-inf")])
    res = metrics.metrics_summary(run_dir=str(run))
    summary = res["scalars"]["loss"]
    assert summary["min"] == "-inf"
    assert summary["max"] == "inf"
    assert summary["last"] == "-inf"
    assert summary["count"] == 3
    json.dumps(res, allow_nan=False)


def test_metrics_summary_of_an_all_nan_run_is_nan(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [float("nan"), float("nan")])
    summary = metrics.metrics_summary(run_dir=str(run))["scalars"]["loss"]
    assert summary["min"] == "nan"
    assert summary["max"] == "nan"
    assert summary["last"] == "nan"
    assert summary["count"] == 2


def test_tensorboard_tail_reports_nan_values_verbatim(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    _write_events(run, "loss", [1.0, float("nan"), 3.0])
    res = metrics.tensorboard_tail(run_dir=str(run), tag="loss")
    values = [point["value"] for point in res["points"]]
    assert values == [1.0, "nan", 3.0]
    json.dumps(res, allow_nan=False)


# --------------------------------------------------------------------------
# checkpoints_list (same module, different code path)
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "filename,expected_step",
    [
        ("model_150.pt", 150),
        ("epoch_12.pth", 12),
        ("step_5.pt", 5),
        ("iter_7.pt", 7),
        ("iteration_3.pt", 3),
        ("checkpoint_9.pt", 9),
        ("ep_4.pt", 4),
        ("net_0012.pt", 12),
        ("policy_100.pth", 100),
        ("final.pth", None),
        ("model_best.pt", None),
    ],
)
def test_checkpoints_list_step_parsing_variants(tmp_path, filename, expected_step):
    (tmp_path / filename).write_bytes(b"x")
    res = metrics.checkpoints_list(run_dir=str(tmp_path))
    assert len(res["checkpoints"]) == 1
    assert res["checkpoints"][0]["step"] == expected_step


def test_checkpoints_list_ignores_other_file_types(tmp_path):
    (tmp_path / "model_1.pt").write_bytes(b"x")
    (tmp_path / "model_2.pth").write_bytes(b"x")
    (tmp_path / "notes.txt").write_text("x")
    (tmp_path / "model_3.bin").write_bytes(b"x")
    res = metrics.checkpoints_list(run_dir=str(tmp_path))
    assert len(res["checkpoints"]) == 2


def test_checkpoints_list_excludes_symlink_pointing_outside_allowed_roots(
    tmp_path, monkeypatch
):
    allowed = tmp_path / "allowed"
    allowed.mkdir()
    run = allowed / "run"
    run.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    target = outside / "real.pt"
    target.write_bytes(b"x")
    (run / "model_1.pt").symlink_to(target)
    _isolate_roots(monkeypatch, allowed)
    res = metrics.checkpoints_list(run_dir=str(run))
    assert res["checkpoints"] == []


def test_checkpoints_list_honours_explicit_search_roots(tmp_path):
    (tmp_path / "model_1.pt").write_bytes(b"x")
    res = metrics.checkpoints_list(search_roots=[str(tmp_path)])
    assert len(res["checkpoints"]) == 1


def test_checkpoints_list_search_roots_without_run_dir_skips_missing_roots(tmp_path):
    res = metrics.checkpoints_list(search_roots=[str(tmp_path / "missing")])
    assert res["checkpoints"] == []


def test_outside_roots_error_says_which_roots_and_what_to_do(tmp_path):
    from twinsmith.isaaclab import metrics

    res = metrics.checkpoints_list(run_dir="/etc")
    assert res["error"] == "path outside allowed roots" and isinstance(res["allowed_roots"], list)
    assert "ISAACLAB_LOGS_DIR" in res["hint"] and "runs_status" in res["hint"]


def _tail_tool(tmp_path, monkeypatch, last_n):
    from twinsmith.isaaclab import metrics
    from twinsmith.server import build_server

    seen = {}

    def tail(run_dir, tag, last_n=50):
        seen["last_n"] = last_n
        return {"points": []}

    monkeypatch.setattr(metrics, "tensorboard_tail", tail)
    server = build_server({"TWINSMITH_TOOLSETS": "default,+sim"})
    args = {"run_dir": str(tmp_path), "tag": "loss", "last_n": last_n}
    return asyncio.run(server.call_tool("isaaclab__tensorboard_tail", args)), seen


def test_tensorboard_tail_refuses_a_boolean_through_the_tool_layer(tmp_path, monkeypatch):
    from mcp.server.fastmcp.exceptions import ToolError

    for value in (True, False):
        with pytest.raises(ToolError, match="not a boolean"):
            _tail_tool(tmp_path, monkeypatch, value)


@pytest.mark.parametrize("value, expected", [(None, None), (3, 3), (0, 0)])
def test_tensorboard_tail_accepts_integers_and_null(tmp_path, monkeypatch, value, expected):
    _, seen = _tail_tool(tmp_path, monkeypatch, value)
    assert seen["last_n"] == expected
