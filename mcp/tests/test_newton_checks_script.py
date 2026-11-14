"""scripts/smoke/newton_checks.py is run later on a machine with a free GPU. Its logic is proven here against a fake bridge
that behaves like a Newton session, with a switch for each way the real one can be wrong."""

import asyncio
import importlib.util
import json
import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
Image = pytest.importorskip("PIL.Image")

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("newton_checks", ROOT / "scripts" / "smoke" / "newton_checks.py")
checks = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checks)
SMOKE = importlib.util.spec_from_file_location("smoke_runner", ROOT / "scripts" / "smoke.py")
smoke = importlib.util.module_from_spec(SMOKE)
SMOKE.loader.exec_module(smoke)

EYE, TARGET = [80.0, -12.0, 5.0], [80.0, 0.0, 1.3]
GRAVITY = 9.81


class FakeBridge:
    """A Newton session as the tools describe it. ``faults`` names what is wrong with it."""

    def __init__(self, tmp_path, *faults):
        self.tmp, self.faults = tmp_path, set(faults)
        self.stopped, self.stage, self.dt, self.clock = True, None, 0.002, 0.0
        self.probe = dict(z=2.5, vz=0.0, x=80.0, vx=0.0)
        self.calls, self.drift, self.pushed = [], 0.0, False

    async def __call__(self, name, **args):
        assert (("session_id" in args) == ("stage_id" in args))  # always sent together
        args = {k: v for k, v in args.items() if k not in ("session_id", "stage_id")}
        self.calls.append((name, args))
        return getattr(self, name)(**args)

    def error(self, tool, code, **details):
        raise checks.ToolError(tool, dict(code=code, details=details))

    # --- stage and scene
    def sim__stage__info(self):
        return dict(session_id="s", stage_id="g", stopped=self.stopped)

    def sim__quick_start(self, discard_changes, environment=None):
        self.stage, self.stopped, self.dt = ("b" if environment else "a"), True, 0.002
        self.probe = dict(z=2.5, vz=0.0, x=80.0, vx=0.0)
        result = dict(session_id="s", stage_id="g", physics_scene="/World/PhysicsScene")
        if "no_newton_report" not in self.faults:
            buffers = dict(nconmax=512 if not environment else 1728, njmax=4096 if not environment else 13824)
            result["newton"] = dict(rate_hz=500 if not environment else 120, buffers=buffers)
        if environment:
            self.dt = 1 / 120
            buffers = dict(nconmax=1728 if "small_buffers" not in self.faults else 200, njmax=13824)
            result["environment"] = dict(name="jenga", blocks=54, top_block=checks.block_path(17, 1), physics_hz=120,
                                         newton_settings={"newton:timeStepsPerSecond": 120}, newton_buffers=buffers)
            if "tower_error" in self.faults:
                result["environment"]["newton_error"] = dict(code="PHYSICS_API_UNAVAILABLE")
        return result

    def sim__prim__create(self, **args):
        return dict(path=args["path"])

    sim__light__create = sim__material__create = sim__material__apply = sim__camera__create = sim__prim__create

    def sim__physics__configure(self, path, gravity, dt):
        self.dt = dt if "configure_ignored" not in self.faults else self.dt
        return dict(path=path, dt=self.dt)

    def sim__timeline(self, action):
        if action == "stop":
            self.stopped = True
            self.probe = dict(z=2.5, vz=0.0, x=80.0, vx=0.0)
        else:
            self.stopped = False
        return dict(action=action)

    # --- stepping and bodies
    def advance(self, steps):
        for _ in range(steps):
            probe = self.probe
            probe["vz"] -= GRAVITY * self.dt
            probe["z"] += probe["vz"] * self.dt
            if probe["z"] <= 0.125:
                probe["z"], probe["vz"] = 0.125, 0.0
                decel = GRAVITY * self.dt
                probe["vx"] = max(0.0, probe["vx"] - decel)
            probe["x"] += probe["vx"] * self.dt
            self.clock += self.dt
        if "drift" in self.faults and self.stage == "b":
            self.drift += 0.0015 * steps / 120

    def sim__simulation__step(self, steps, observe_prims=None):
        if "clock_half" in self.faults:
            self.advance(steps)
            self.error("sim__simulation__step", "STEP_TIMING_MISMATCH", step=dict(measured_delta=steps * self.dt / 2),
                       newton=dict(clocks_after=dict(newton_sim_time=steps * self.dt)))
        if "dt_disagree" in self.faults and self.stage == "a" and self.dt == 0.002:
            self.error("sim__simulation__step", "STEP_DT_SOURCES_DISAGREE", dt_sources=dict(scene_dt=0.002, newton_sim_dt=0.001))
        self.advance(steps)
        return dict(steps=steps, physics_dt=self.dt, measured_delta=steps * self.dt, expected_delta=steps * self.dt,
                    newton=dict(timestep_check=dict(ran="check_not_run" not in self.faults, scene_dt=self.dt, newton_sim_dt=self.dt),
                                clocks_before=dict(newton_sim_time=0.0), clocks_after=dict(newton_sim_time=self.clock)))

    def body(self, path, dynamics=False):
        if self.stage == "b":
            index = checks.BLOCKS.index(path) if path in checks.BLOCKS else 0
            offset = self.drift if index == 3 else 0.0
            result = dict(path=path, world_position=[0.7 + offset, 0.001 * index, 0.35], orientation_wxyz=[1, 0, 0, 0])
        else:
            result = dict(path=path, world_position=[self.probe["x"], 0.0, self.probe["z"]], orientation_wxyz=[1, 0, 0, 0],
                          render_pose=dict(source="Fabric", agrees="stale_render" not in self.faults))
            if dynamics:
                result["dynamics"] = dict(linear_velocity=[self.probe["vx"], 0.0, self.probe["vz"]], mass_kg=0.5)
        return result

    def sim__body__state(self, path=None, paths=None, include_dynamics=False):
        if paths is not None:
            return dict(bodies=[self.body(p) for p in paths])
        return self.body(path, include_dynamics)

    def sim__physics__apply(self, path, impulse):
        if "push_noop" not in self.faults:
            self.probe["vx"] += impulse[0] / 0.5
        return dict(backend="newton", applied="push_noop" not in self.faults, applied_force_n=[v / self.dt for v in impulse],
                    force_duration_s=self.dt)

    def sim__camera__capture(self, path, filename):
        image = np.full((480, 640, 3), 90, dtype=np.uint8)
        z = 2.5 if "stale_render" in self.faults else self.probe["z"]
        u, v = checks.project([80.0, 0.0, z], EYE, TARGET)
        image[max(0, int(v) - 12):int(v) + 12, max(0, int(u) - 12):int(u) + 12] = (230, 5, 5)
        file = self.tmp / filename
        Image.fromarray(image).save(file)
        return dict(file=str(file), physics_render=dict(backend="newton", fabric_updates=True))

    # --- diagnostics
    def sim__physics__inspect(self, root, limit):
        newton = dict(backend="newton", configured=dict(nconmax=1728, njmax=13824, rigid_contact_max=1728),
                      active_contacts=[1100], contact_capacity=1728, active_constraints=[2000], constraint_capacity=13824,
                      overflow_history_available=True, overflow_flags=[dict(raw=0, names=[], unknown_bits=0)], warnings=[])
        if "overflow" in self.faults:
            newton["warnings"] = [dict(code="CONTACT_BUFFER_FULL", message="full")]
        if "no_counters" in self.faults:
            newton["active_contacts"] = None
        return dict(nodes=[], truncated=False, newton=newton)

    def sim__physics__contacts(self, paths):
        if "no_contacts" in self.faults:
            return dict(paths=paths, pairs=[], backend="newton", truncated=False)
        pair = lambda other, force: dict(body_a=paths[0], body_b=other, collider_a=paths[0], collider_b=other or "/World/Jenga/Table",
                                         contact_points=4, force=[0, 0, force], force_magnitude=force)
        return dict(paths=paths, backend="newton", truncated=False,
                    pairs=[pair(None, 5.2), pair(checks.block_path(1, 0), 1.1)])


def run(tmp_path, *faults, settings=None, rgb=None):
    fake = FakeBridge(tmp_path, *faults)
    results = asyncio.run(checks.run_checks(fake, {**dict(hold_seconds=2.0), **(settings or {})}))
    return fake, {row["check"]: row for row in results}, results


def test_every_check_passes_on_a_healthy_newton_session(tmp_path):
    fake, rows, results = run(tmp_path)
    assert [row["check"] for row in results] == list(checks.CHECK_ORDER)
    assert {row["check"]: row["status"] for row in results} == {name: "pass" for name in checks.CHECK_ORDER}, results
    json.dumps(results, default=str)
    assert "20*dt" in rows["exact_steps_default"]["summary"] and "dv" in rows["push_moves_body"]["summary"]
    assert "54 blocks" in rows["tower_build"]["summary"] and "active contacts" in rows["contact_buffer"]["summary"]
    table = checks.render_table(results)
    assert table.count("PASS") == len(checks.CHECK_ORDER) and "Check" in table.splitlines()[0]


def test_the_tower_is_held_for_the_requested_simulated_time(tmp_path):
    fake, rows, _ = run(tmp_path, settings=dict(hold_seconds=10.0))
    held = rows["tower_hold"]["measured"]
    assert held["simulated_s"] == pytest.approx(10.0, abs=0.01) and held["steps"] == math.ceil(10.0 / (1 / 120))
    steps = [args["steps"] for name, args in fake.calls if name == "sim__simulation__step"]
    assert max(steps) <= 120 and sum(steps[-11:]) >= held["steps"]  # chunks the tool accepts


def test_a_drifting_block_fails_only_the_hold_check(tmp_path):
    _, rows, _ = run(tmp_path, "drift", settings=dict(hold_seconds=10.0))
    assert rows["tower_hold"]["status"] == "fail" and "moved more than the tolerance" in rows["tower_hold"]["reason"]
    assert all(rows[name]["status"] == "pass" for name in ("tower_build", "contact_buffer", "contacts_pairs", "push_moves_body"))


def test_the_sim_6_2_clock_mismatch_is_reported_with_the_tools_own_evidence(tmp_path):
    _, rows, _ = run(tmp_path, "clock_half")
    for name in ("exact_steps_default", "exact_steps_configured"):
        assert rows[name]["status"] == "fail" and "STEP_TIMING_MISMATCH" in rows[name]["reason"]
        assert "newton_sim_time" in rows[name]["reason"]  # Newton's own clock, for the live diagnosis
    assert rows["tower_hold"]["status"] == "fail"  # every step on this fake is wrong, so the hold cannot run either
    assert rows["quick_scene"]["status"] == "pass" and rows["tower_build"]["status"] == "pass"


def test_a_disagreeing_timestep_is_a_failure_of_the_default_scene_only(tmp_path):
    _, rows, _ = run(tmp_path, "dt_disagree")
    assert rows["exact_steps_default"]["status"] == "fail" and "STEP_DT_SOURCES_DISAGREE" in rows["exact_steps_default"]["reason"]
    assert rows["exact_steps_configured"]["status"] == "pass"


def test_a_cleared_buffer_check_needs_the_counters_not_their_absence(tmp_path):
    _, rows, _ = run(tmp_path, "no_counters")
    assert rows["contact_buffer"]["status"] == "fail" and "cannot be ruled out" in rows["contact_buffer"]["reason"]
    _, rows, _ = run(tmp_path, "overflow")
    assert rows["contact_buffer"]["status"] == "fail" and "CONTACT_BUFFER_FULL" in rows["contact_buffer"]["reason"]


def test_small_buffers_fail_the_tower_build_and_stop_what_depends_on_it(tmp_path):
    _, rows, _ = run(tmp_path, "small_buffers")
    assert rows["tower_build"]["status"] == "fail"
    for name in ("tower_hold", "contact_buffer", "contacts_pairs"):
        assert rows[name]["status"] == "not_run" and "tower did not build" in rows[name]["reason"]
    assert rows["push_moves_body"]["status"] == "pass"  # the other scene is independent


def test_a_session_without_a_newton_report_fails_the_quick_scene_and_stops_the_plain_scene(tmp_path):
    _, rows, _ = run(tmp_path, "no_newton_report")
    assert rows["quick_scene"]["status"] == "fail"
    for name in ("exact_steps_default", "exact_steps_configured", "camera_after_motion", "push_moves_body"):
        assert rows[name]["status"] == "not_run"
    assert rows["tower_build"]["status"] == "pass"


def test_a_picture_that_does_not_follow_physics_fails_the_camera_check(tmp_path):
    _, rows, _ = run(tmp_path, "stale_render")
    assert rows["camera_after_motion"]["status"] == "fail"
    assert "renderer does not show the moved pose" in rows["camera_after_motion"]["reason"]
    assert rows["push_moves_body"]["status"] == "pass"


def test_a_push_that_does_nothing_fails_the_push_check(tmp_path):
    _, rows, _ = run(tmp_path, "push_noop")
    assert rows["push_moves_body"]["status"] == "fail" and "not applied" in rows["push_moves_body"]["reason"]


def test_no_contacts_for_a_standing_block_fails_the_contacts_check(tmp_path):
    _, rows, _ = run(tmp_path, "no_contacts")
    assert rows["contacts_pairs"]["status"] == "fail"


def test_the_timestep_cross_check_must_have_run(tmp_path):
    _, rows, _ = run(tmp_path, "check_not_run")
    assert rows["exact_steps_default"]["status"] == "fail" and "did not run" in rows["exact_steps_default"]["reason"]


def test_a_failing_set_up_is_recorded_and_the_other_scene_still_runs(tmp_path):
    class Broken(FakeBridge):
        def sim__prim__create(self, **args):
            self.error("sim__prim__create", "CONFLICT")

    fake = Broken(tmp_path)
    results = asyncio.run(checks.run_checks(fake, dict(hold_seconds=1.0)))
    rows = {row["check"]: row for row in results}
    assert rows["scene_a_setup"]["status"] == "fail" and rows["exact_steps_default"]["status"] == "not_run"
    assert rows["quick_scene"]["status"] == "pass"
    assert all(rows[name]["status"] == "pass" for name in ("tower_build", "tower_hold", "contact_buffer", "contacts_pairs"))


def test_projection_puts_the_aim_point_at_the_centre_and_higher_points_higher():
    u, v = checks.project(TARGET, EYE, TARGET)
    assert (u, v) == pytest.approx((320, 240))
    _, higher = checks.project([80.0, 0.0, 2.3], EYE, TARGET)
    assert higher < 240
    left, _ = checks.project([79.0, 0.0, 1.3], EYE, TARGET)
    assert left < 320  # world -x is to the camera's left when it looks along +y... with up +z, right is +x
    assert checks.project([81.0, 0.0, 1.3], EYE, TARGET)[0] > 320


def test_block_motion_and_rotation_find_the_worst_block():
    before = {"a": ([0, 0, 0], [1, 0, 0, 0]), "b": ([1, 0, 0], [1, 0, 0, 0])}
    after = {"a": ([0.001, 0, 0], [1, 0, 0, 0]), "b": ([1, 0.02, 0], [math.cos(math.radians(5)), math.sin(math.radians(5)), 0, 0])}
    move, move_block, turn, turn_block = checks.block_motion(before, after)
    assert move == pytest.approx(0.02) and move_block == "b" and turn == pytest.approx(10.0) and turn_block == "b"


def test_red_row_finds_red_and_nothing_else():
    image = np.full((20, 20, 3), 100, dtype=np.uint8)
    assert checks.red_row(image) is None
    image[10:12, 3:5] = (240, 10, 10)
    assert checks.red_row(image) == pytest.approx(10.5)


def test_the_registered_check_is_newton_only_and_skipped_with_a_note_on_physx(tmp_path, monkeypatch):
    assert "newton_checks" in smoke.TOOLSETS["sim"] and smoke.NEWTON_ONLY == ("newton_checks",)

    class Client:
        port = 8766

        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def call(self, name, arguments):
            return {"app_version": "test-build"} if name == "status" else {"runtime": {"physics_engine": "physx"}}

    executed = []
    monkeypatch.setattr(smoke, "execute", lambda name, *args: executed.append(name))
    assert smoke.main(["sim", "--output-dir", str(tmp_path), "--check", "validate_scene_joints", "--check", "newton_checks"],
                      Client) == 0
    rows = {row["check"]: row for row in json.loads((tmp_path / "results.json").read_text())["checks"]}
    assert "newton_checks" not in executed and rows["newton_checks"]["status"] == "skipped"
    assert "Newton-only" in rows["newton_checks"]["note"] and rows["newton_checks"]["backend"] == "physx"
    assert all(rows[name]["status"] == "pass" for name in executed)


def test_the_runner_demands_discard_changes_for_the_newton_checks(tmp_path):
    args = type("Args", (), dict(discard_changes=False, port=1))()
    with pytest.raises(ValueError, match="--discard-changes"):
        smoke.execute("newton_checks", args, tmp_path / "x.json", None, {"backend": "newton"})
