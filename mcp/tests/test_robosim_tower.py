"""The robosim tower template: its data checks, and its scene text run against a stand-in for RoboSimStudio.

The stand-in has no physics: a body stays where its coordinates were last written. That is enough to check what
the scene writes (layout, the hand's hold, the finger's path, the contact arrays) and what it reports.
"""

import ast
import json
import math
import sys
import types

import pytest

from twinsmith.robosim import plan, template, tower

PUSH = {"block": "L08_1", "distance": 0.03, "seconds": 1.0}


def test_the_default_tower_is_18_layers_of_the_lab_block_with_the_first_runs_contact_values():
    resolved = template.resolve({"shape": "tower"})
    assert (resolved["size"], resolved["layers"]) == ([0.072, 0.024, 0.014], 18)
    assert (resolved["mass"], resolved["friction"], resolved["table_friction"], resolved["damping_ratio"]) == (0.013, 0.392, 0.539, 0.737)
    assert resolved["variation"] == {"size_mm": [0.3, 0.3, 0.1], "mass_frac": 0.05, "friction_frac": 0.1, "seed": 0}
    assert resolved["mask_key"] == "tower=wood" and resolved["push"] is None and resolved["hold"] is None
    names = tower.block_names(resolved)
    assert len(names) == 54 and names[0] == "L00_0" and names[-1] == "L17_2"
    text = template.scene_source(resolved)
    ast.parse(text)
    assert "BLOCK_SIZE = (0.072, 0.024, 0.014)" in text and "LAYERS = 18  " in text and "BLOCK_FRICTION = 0.392  " in text
    assert "physics=physics" in text and "MUJOCO_500HZ" in text and "joint_q.assign" in text and "body_q.assign" not in text
    assert plan.scene_problems(text) == []


def test_tower_data_merges_on_update_and_keeps_what_was_not_named():
    first = template.resolve({"shape": "tower", "missing": ["L07_1"], "variation": {"seed": 3}, "camera": {"fov_y": 30}})
    assert first["variation"]["seed"] == 3 and first["variation"]["mass_frac"] == 0.05 and first["camera"]["fov_y"] == 30
    assert first["camera"]["eye"] == tower.DEFAULTS["camera"]["eye"] and "L07_1" not in tower.block_names(first)
    later = template.resolve({"friction": 0.3, "push": PUSH}, first)          # update_scene names no shape
    assert later["shape"] == "tower" and later["missing"] == ["L07_1"] and later["friction"] == 0.3
    assert later["variation"]["seed"] == 3 and later["push"] == dict(PUSH, direction=1)
    assert template.resolve({"shape": "tower"}, template.resolve({"shape": "jenga"}))["layers"] == 18   # another shape gives no defaults


def test_fitted_blocks_replace_the_regular_tower_and_a_hold_names_one_of_them():
    blocks = [{"name": "base_0", "layer": 0, "x": 0.0, "y": 0.0, "section": "base"},
              {"name": "top", "layer": 1, "x": 0.004, "y": -0.002, "yaw": 1.57, "section": "upper"}]
    hold = {"block": "top", "path": [[0, 0, 0], [0.2, -0.008, -0.003]], "release_s": 0.25}
    resolved = template.resolve({"shape": "tower", "blocks": blocks, "hold": hold})
    assert resolved["blocks"] == [["base_0", 0, 0.0, 0.0, 0.0, "base"], ["top", 1, 0.004, -0.002, 1.57, "upper"]]
    assert resolved["hold"] == {"block": "top", "path": [[0.0, 0.0, 0.0], [0.2, -0.008, -0.003]], "release_s": 0.25, "axis_only": False}
    assert "BLOCKS = [['base_0', 0, 0.0, 0.0, 0.0, 'base'], ['top', 1, 0.004, -0.002, 1.57, 'upper']]" in template.scene_source(resolved)
    assert template.resolve({"friction": 0.3}, resolved)["blocks"] == resolved["blocks"]     # resolved rows pass through an update


@pytest.mark.parametrize("bad", [
    {"layers": 0}, {"layers": 41}, {"size": [0.072, 0.024]}, {"damping_ratio": 0.4}, {"friction": 0}, {"mass": -1},
    {"missing": ["L18_0"]}, {"missing": ["L99"]}, {"variation": {"sigma": 1}}, {"variation": {"mass_frac": 0.9}},
    {"push": {"block": "L18_0", "distance": 0.03, "seconds": 1}}, {"push": {"block": "L08_1", "distance": 0, "seconds": 1}},
    {"push": {"block": "L08_1", "distance": 0.03, "seconds": 1, "direction": 2}}, {"push": {"waypoints": [[0, 0, 0], [1, 0, 0.1]]}},
    {"hold": {"block": "L08_1", "path": [[0, 0, 0]], "release_s": 1}}, {"hold": {"block": "nope", "path": [[0, 0, 0], [1, 0, 0]], "release_s": 1}},
    {"hold": {"block": "L08_1", "path": [[0.1, 0, 0], [1, 0, 0]], "release_s": 1}},
    {"blocks": [{"name": "a", "layer": 0, "x": 0, "y": 0}, {"name": "a", "layer": 1, "x": 0, "y": 0}]},
    {"blocks": [{"name": "finger", "layer": 0, "x": 0, "y": 0}]}, {"blocks": [{"name": "a", "layer": 0, "x": 1.0, "y": 0}]},
    {"blocks": [{"name": "a", "layer": 0, "x": 0, "y": 0}], "missing": ["L00_0"]}, {"blocks": [["a", 0, 0.0, 0.0, 0.0, "tower"]]},
    {"camera": {"eye": [0, 0, -1]}}, {"camera": {"fov_y": 0}}, {"position": [1.0, 0]}, {"waypoints": 1}])
def test_bad_tower_data_is_refused_with_a_reason(bad):
    with pytest.raises(ValueError):
        template.resolve(dict(bad, shape="tower"))


def test_a_tower_push_is_its_own_form_and_sets_the_rollout_length():
    push, warnings = plan.validate_push({"block": "L08_1", "distance": 0.03, "seconds": 3})
    assert push == {"block": "L08_1", "distance": 0.03, "seconds": 3.0, "direction": 1} and warnings == []
    assert plan.rollout_plan(125.0, push=push)["steps"] == 500                 # the push, then one second to come to rest
    with pytest.raises(ValueError, match="not a block of this tower"):
        tower.validate_push(push, ["L00_0"])
    with pytest.raises(ValueError, match="a tower push is"):
        plan.validate_push({"block": "L08_1", "distance": 0.03})


# ---- the scene text, run against a stand-in for RoboSimStudio ----------------------------------

class Array:
    """A warp-like array: ``numpy()`` gives a copy, ``assign`` stores one."""

    def __init__(self, values):
        self.values = values

    def numpy(self):
        return self.values.copy()

    def assign(self, values):
        self.values = values.copy()


def load_scene(np, text, monkeypatch, cpu):
    """Execute the scene text with a stand-in ``robosimstudio``; returns its namespace and a sim without physics."""
    from dataclasses import dataclass

    @dataclass(frozen=True)
    class Physics:
        control_hz: float = 50.0
        substeps: int = 10
        backends: tuple = ()

    class Scene:
        def __init__(self, **options):
            self.options, self.bodies, self.cameras = options, [], []

        def add(self, body):
            self.bodies.append(body)

    rss = types.ModuleType("robosimstudio")
    rss.Scene = Scene
    rss.box = lambda **options: options
    rss.CameraSpec = lambda **options: options
    physics = types.ModuleType("robosimstudio.specs.physics")
    physics.MUJOCO_500HZ = Physics()
    physics.backend = lambda solver, **options: (solver, options)
    for name, module in (("robosimstudio", rss), ("robosimstudio.specs", types.ModuleType("robosimstudio.specs")),
                         ("robosimstudio.specs.physics", physics)):
        monkeypatch.setitem(sys.modules, name, module)
    scene = {"__file__": "scene.py"}
    exec(compile(text, "scene.py", "exec"), scene)
    bodies = [body for body in scene["scene"].bodies if not body.get("static")]
    q = np.concatenate([[*body["at"], *body.get("quat", (0.0, 0.0, 0.0, 1.0))] for body in bodies])
    entities = {body["name"]: types.SimpleNamespace(coord_start=7 * i, dof_start=6 * i, body=i) for i, body in enumerate(bodies)}
    geoms = np.array([[*scene["TABLE_HALF"], 0.02]] + [list(body["half"]) for body in bodies])
    lead = (lambda values: values) if cpu else (lambda values: Array(values[None].copy()))
    model = types.SimpleNamespace(ngeom=len(geoms), geom_size=lead(geoms), geom_solref=lead(np.tile([0.02, 1.0], (len(geoms), 1))),
                                  geom_friction=lead(np.array([[0.5, 0.005, 0.0001]] + [[body["friction"], 0.005, 0.0001] for body in bodies])),
                                  geom_priority=np.zeros(len(geoms), int) if cpu else Array(np.zeros(len(geoms), int)))

    class Sim:
        def __init__(self):
            self.scene = types.SimpleNamespace(by_name=entities)
            self.state = types.SimpleNamespace(joint_q=Array(q), joint_qd=Array(np.zeros(6 * len(bodies))), body_q=Array(q.reshape(-1, 7)))
            self.pipeline = types.SimpleNamespace(solver=types.SimpleNamespace(use_mujoco_cpu=cpu, mj_model=model, mjw_model=model))
            self.steps = self.resets = 0

        def run(self, count):
            self.steps += count
            self.state.body_q.assign(self.state.joint_q.numpy().reshape(-1, 7))

        def sync(self):
            pass

        def reset(self):
            self.resets += 1

        def __getitem__(self, name):
            row = self.state.body_q.numpy()[entities[name].body]
            return types.SimpleNamespace(position=row[:3], pose=row)

    return scene, Sim(), model


@pytest.mark.parametrize("cpu", [True, False])
def test_the_scene_lays_out_the_tower_and_writes_contact_values_into_the_solver(monkeypatch, cpu):
    np = pytest.importorskip("numpy")
    text = template.scene_source(template.resolve({"shape": "tower", "missing": ["L17_1"], "position": [0.1, 0.0]}))
    scene, sim, model = load_scene(np, text, monkeypatch, cpu)
    assert scene["physics"].control_hz == 125.0 and scene["physics"].substeps == 4       # 2 ms steps
    assert len(scene["NAMES"]) == 53 and [body["name"] for body in scene["scene"].bodies][-1] == "finger"
    poses = dict(zip(scene["NAMES"], scene["start_poses"]()))
    assert np.allclose(poses["L00_0"][:3], [0.1, -0.024, 0.007 + 0.003]) and np.allclose(poses["L00_0"][3:], [0, 0, 0, 1])
    assert np.allclose(poses["L01_2"][:3], [0.1 + 0.024, 0.0, 0.021 + 0.003]) and poses["L01_2"][5] == pytest.approx(math.sin(math.pi / 4))
    shifted = dict(zip(scene["NAMES"], scene["start_poses"]({"tower": [0.001, 0.0], "L00_0": [0.0, 0.002]})))
    assert np.allclose(shifted["L00_0"][:2] - poses["L00_0"][:2], [0.001, 0.002]) and np.allclose(shifted["L05_1"][:2] - poses["L05_1"][:2], [0.001, 0.0])
    assert np.abs(scene["HALF"] * 2 - [0.072, 0.024, 0.014]).max() <= 2 * 0.0003 + 1e-12 and np.ptp(scene["HALF"][:, 0]) > 0   # blocks differ
    written = scene["apply_contact"](sim, friction=0.3, table_friction=0.6, damping_ratio=0.8)
    assert written["block_geoms"] == 53 and written["table_geoms"] == 1
    read = (lambda name: getattr(model, name)) if cpu else (lambda name: getattr(model, name).values[0] if name != "geom_priority" else getattr(model, name).values)
    friction = read("geom_friction")
    assert friction[0, 0] == 0.6 and friction[-1, 0] == 0.4                              # the table; the finger keeps its own
    assert np.allclose(friction[1:54, 0], 0.3 * np.array(scene["FRICTION_FACTOR"])) and np.ptp(friction[1:54, 0]) > 0
    assert list(read("geom_priority")[:2]) == [1, 0] and np.allclose(read("geom_solref"), [0.004, 0.8])
    scene["apply_contact"](sim, friction=0.6)                                          # a second call scales from the built values
    assert np.allclose(read("geom_friction")[1:54, 0], 0.6 * np.array(scene["FRICTION_FACTOR"]))


def test_the_finger_follows_its_path_and_the_push_routine_writes_the_trajectory_and_summary(monkeypatch, tmp_path):
    np = pytest.importorskip("numpy")
    scene, sim, _ = load_scene(np, template.scene_source(template.resolve({"shape": "tower"})), monkeypatch, cpu=True)
    scene["__file__"] = str(tmp_path / "scene.py")
    scene["ROLLOUT"] = {"steps": 250, "take": "t1", "push": dict(PUSH, direction=-1)}
    steps = sum(1 for _ in scene["routines"]["push"](sim))
    assert steps == 250 and sim.steps == 250
    result = json.loads((tmp_path / "push-t1.json").read_text())
    assert result["routine"] == "push" and result["applied"] and result["pusher_tracked"] and result["blocks"] == 54
    assert result["pushed_m"] == 0.0 and result["tower"]["standing"] and result["tower"]["lean_deg"] == 0.0   # no physics: nothing moved
    saved = np.load(tmp_path / "traj-t1.npz")
    assert saved["poses"].shape == (251, 54, 7) and saved["t"][-1] == pytest.approx(2.0) and list(saved["names"][:2]) == ["L00_0", "L00_1"]
    block = scene["start_poses"]()[scene["NAMES"].index("L08_1")]                      # layer 8 lies along x
    finger = sim["finger"].position
    back = scene["HALF"][scene["NAMES"].index("L08_1")][0] + 0.005 + 0.03               # block end, clearance, finger half
    assert finger[1] == pytest.approx(block[1]) and finger[2] == pytest.approx(block[2])
    assert finger[0] == pytest.approx(block[0] + back - 0.035)                          # came from +x, travelled distance + clearance
    with pytest.raises(ValueError, match="needs a push"):
        scene["ROLLOUT"] = {"steps": 5, "take": "t2", "push": None}
        list(scene["routines"]["push"](sim))


def test_a_held_block_follows_its_path_until_release_and_a_script_can_drive_rollouts(monkeypatch):
    np = pytest.importorskip("numpy")
    scene, sim, _ = load_scene(np, template.scene_source(template.resolve({"shape": "tower", "layers": 4})), monkeypatch, cpu=True)
    hold = {"block": "L02_1", "path": [[0.0, 0.0, 0.0], [0.08, -0.008, -0.002]], "release_s": 0.08}
    out = scene["rollout"](sim, {"hold": hold, "friction": 0.3}, steps=40, until_rest=False)
    index = out["names"].index("L02_1")
    moved = out["poses"][:, index, :3] - out["poses"][0, index, :3]
    assert sim.resets == 1 and out["result"]["released_at_s"] == pytest.approx(0.08) and out["result"]["contact"]["friction"] == 0.3
    assert np.allclose(moved[5], [-0.0032, 0.0, -0.0008], atol=1e-6)                    # written at 32 ms, before the fifth step
    assert np.allclose(moved[-1], moved[10]) and abs(moved[10, 0]) < 0.008              # released: no more writes
    others = np.delete(out["poses"], index, axis=1)
    assert np.allclose(others[-1], others[0])
    axis_only = scene["rollout"](sim, {"hold": dict(hold, axis_only=True)}, steps=5, until_rest=False)
    assert np.allclose((axis_only["poses"][5, index, :3] - axis_only["poses"][0, index, :3])[1:], 0.0)   # only along the pull
    assert scene["rollout"](sim, {}, steps=400)["result"]["steps"] < 400                # at rest: stops early


def test_tower_state_reports_lean_shift_and_a_fallen_block(monkeypatch):
    np = pytest.importorskip("numpy")
    scene, _, _ = load_scene(np, template.scene_source(template.resolve({"shape": "tower", "layers": 10})), monkeypatch, cpu=True)
    start = scene["start_poses"]()
    assert scene["tower_state"](start, start)["lean_deg"] == 0.0
    leaning = start.copy()
    height = start[-1, 2] - start[0, 2]
    leaning[:, 0] += (start[:, 2] - start[0, 2]) * math.tan(math.radians(3.0))          # shear the tower 3 degrees
    state = scene["tower_state"](start, leaning)
    assert state["lean_deg"] == pytest.approx(3.0, abs=0.01) and state["standing"]
    assert state["top_shift_m"] == pytest.approx(height * math.tan(math.radians(3.0)), abs=1e-4)
    fallen = start.copy()
    fallen[15, 2] -= 0.02
    assert not scene["tower_state"](start, fallen)["standing"]
    assert scene["tower_state"](start, fallen, moved=[scene["NAMES"][15]])["standing"]    # the pushed block may leave
