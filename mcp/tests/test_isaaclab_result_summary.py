"""metrics_summary eval_json mode: caller-supplied criteria over evaluation-result JSON files, with synthetic files."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twinsmith.isaaclab import evalsummary as ev
from twinsmith.isaaclab import tools


def write(tmp_path, name, data):
    p = tmp_path / name
    p.write_text(json.dumps(data))
    return str(p)


def result(success=0.9, reward=120.0, episodes=None):
    return {"task": "T", "checkpoint": "/ckpt/a.pt", "stats": {"success_rate": success, "mean_reward": reward, "ok": True},
            "episodes": episodes or [{"reward": reward}, {"reward": reward + 1}], "dotted.key": 5}


CRIT = [{"key": "stats.success_rate", "op": ">=", "value": 0.9, "label": "success"}, {"key": "stats.mean_reward", "op": ">", "value": 100}]


def test_criteria_score_each_file_and_summarise(tmp_path):
    files = [write(tmp_path, "a.json", result(0.95, 130)), write(tmp_path, "b.json", result(0.85, 90))]
    out = ev.summarize(files, CRIT, label="cand")
    assert out["label"] == "cand" and out["n_files"] == 2 and out["runs_passing_all"] == 1
    a, b = out["runs"]
    assert a["passed"] is True and b["passed"] is False and a["checkpoint"] == "/ckpt/a.pt"
    assert [r["passed"] for r in b["results"]] == [False, False]
    s = out["summary"][0]
    assert s["label"] == "success" and s["values"] == [0.95, 0.85] and s["passed_runs"] == 1 and s["mean"] == 0.9
    assert out["summary"][1]["label"] == "stats.mean_reward > 100"  # default label


@pytest.mark.parametrize("op,value,expected", [(">=", 0.9, True), (">", 0.9, False), ("<=", 0.9, True), ("<", 0.9, False),
                                               ("==", 0.9, True), ("!=", 0.9, False), ("ge", 0.9, True), ("lt", 1, True)])
def test_operators(tmp_path, op, value, expected):
    out = ev.summarize([write(tmp_path, "a.json", result(0.9))], [{"key": "stats.success_rate", "op": op, "value": value}])
    assert out["runs"][0]["passed"] is expected


def test_keys_index_lists_booleans_and_dotted_names(tmp_path):
    f = write(tmp_path, "a.json", result())
    crit = [{"key": "episodes.1.reward", "op": ">", "value": 120}, {"key": "stats.ok", "value": 1}, {"key": "dotted.key", "value": 5}]
    out = ev.summarize([f], crit)
    assert [r["value"] for r in out["runs"][0]["results"]] == [121, 1, 5] and out["runs"][0]["passed"]


def test_no_criteria_reports_metrics_without_pass_fail(tmp_path):
    out = ev.summarize([write(tmp_path, "a.json", result(0.9, 100)), write(tmp_path, "b.json", result(0.7, 200))])
    assert "passed" not in out["runs"][0] and "summary" not in out and "No criteria" in out["note"]
    m = out["metrics"]["stats.success_rate"]
    assert (m["mean"], m["min"], m["max"], m["n"]) == (0.8, 0.7, 0.9, 2)
    assert "stats.ok" not in out["metrics"] and "episodes.0.reward" in out["metrics"]  # booleans are not metrics unless asked for


def test_glob_patterns_expand_sorted_and_must_match(tmp_path):
    for i in (2, 1):
        write(tmp_path, f"eval_{i}.json", result(0.5 + i / 10))
    out = ev.summarize([str(tmp_path / "eval_*.json")], CRIT[:1])
    assert [Path(r["path"]).name for r in out["runs"]] == ["eval_1.json", "eval_2.json"]
    with pytest.raises(ValueError, match="matched no files"):
        ev.summarize([str(tmp_path / "none_*.json")])
    with pytest.raises(ValueError, match="expands to"):
        for i in range(ev.MAX_FILES + 1):
            write(tmp_path, f"many_{i:03}.json", result())
        ev.summarize([str(tmp_path / "many_*.json")])


def test_baseline_delta_with_and_without_criteria(tmp_path):
    cand = [write(tmp_path, "c.json", result(0.95, 130))]
    base = [write(tmp_path, "b.json", result(0.75, 100))]
    out = ev.summarize(cand, CRIT, baseline_json=base, baseline_label="base")
    assert out["baseline"]["label"] == "base" and out["baseline"]["delta_mean"] == {"success": 0.2, "stats.mean_reward > 100": 30}
    out = ev.summarize(cand, baseline_json=base)
    assert out["baseline"]["delta_mean"]["stats.mean_reward"] == 30


def test_criteria_from_json_and_yaml_files(tmp_path):
    f = write(tmp_path, "a.json", result())
    cj = write(tmp_path, "crit.json", {"criteria": CRIT})
    assert ev.summarize([f], cj)["runs_passing_all"] == 1
    cy = tmp_path / "crit.yaml"
    cy.write_text("- key: stats.success_rate\n  op: '>='\n  value: 0.95\n  label: strict\n")
    out = ev.summarize([f], str(cy))
    assert out["runs_passing_all"] == 0 and out["summary"][0]["label"] == "strict"
    with pytest.raises(ValueError, match="criteria file not found"):
        ev.summarize([f], str(tmp_path / "missing.json"))


@pytest.mark.parametrize("crit,text", [
    ([{"op": ">=", "value": 1}], "needs a 'key'"),
    ([{"key": "a", "op": "~", "value": 1}], "must be one of"),
    ([{"key": "a"}], "numeric 'value'"),
    ([{"key": "a", "value": "x"}], "numeric 'value'"),
    ([{"key": "a", "value": 1, "label": "x"}, {"key": "b", "value": 1, "label": "x"}], "unique"),
    ("not-a-list", "criteria file not found"),
    (7, "criteria must be"),
])
def test_bad_criteria(tmp_path, crit, text):
    with pytest.raises(ValueError, match=text):
        ev.summarize([write(tmp_path, "a.json", result())], crit)


def test_missing_key_and_non_numeric_value_name_the_file(tmp_path):
    f = write(tmp_path, "a.json", result())
    with pytest.raises(ValueError, match=r"a\.json: expected key stats\.nope, but stats has no 'nope' \(it has: mean_reward"):
        ev.summarize([f], [{"key": "stats.nope", "value": 1}])
    with pytest.raises(ValueError, match="is not a number"):
        ev.summarize([f], [{"key": "task", "value": 1}])


@pytest.mark.parametrize("args", [{"eval_json": []}, {"eval_json": None}, {"eval_json": ["x"] * 0}])
def test_needs_files(args):
    with pytest.raises(ValueError, match="eval_json"):
        ev.summarize(**args)


def test_file_problems(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{")
    arr = write(tmp_path, "arr.json", [1])
    with pytest.raises(ValueError, match="not readable JSON"):
        ev.summarize([str(bad)])
    with pytest.raises(ValueError, match="expected a JSON object"):
        ev.summarize([arr])


def test_relative_path_error_says_where_it_looked_and_to_use_absolute(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match=r"looked for .*nope\.json.*working directory.*absolute path"):
        ev.summarize(["nope.json"])


def test_tool_layer(tmp_path):
    from mcp.server.fastmcp import FastMCP

    server = FastMCP("t")
    tools.register_isaaclab_tools(server, groups={"core", "sim"})
    t = server._tool_manager._tools["isaaclab__metrics_summary"]
    assert t.annotations.readOnlyHint is True
    f = write(tmp_path, "a.json", result())
    ok = json.loads(t.fn(eval_json=[f], criteria=CRIT, label="x"))
    assert ok["runs_passing_all"] == 1 and ok["label"] == "x"
    assert "metrics" in json.loads(t.fn(eval_json=[f]))
    assert "do not combine" in json.loads(t.fn(eval_json=[f], job_id="20261102-100000-T"))["error"]
    assert "only with eval_json" in json.loads(t.fn(run_dir="/x", criteria=CRIT))["error"]
    props = server._tool_manager._tools["isaaclab__metrics_summary"].parameters["properties"]
    assert "allow_test" not in props and {"eval_json", "criteria", "baseline_json"} <= set(props)
