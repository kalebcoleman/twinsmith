"""Backend failures must restore controls and never borrow another engine's state."""

from types import SimpleNamespace
import sys
from pathlib import Path

import pytest

pytest.importorskip("pxr.Usd")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "exts/sim-tools"))
from sim_tools import physics, runtime
from sim_tools.physics import body_path
from twinsmith.project_assets import AssetError


@pytest.mark.parametrize(
    "release", ["6.0.0.0", "6.0.0.1", "6.0.1.0", "6.1.0.0", "6.2.0.0"]
)
@pytest.mark.parametrize("fail", [False, True])
def test_paused_step_restores_backend_controls_even_after_partial_failure(
    monkeypatch, release, fail
):
    captured = object()
    native = SimpleNamespace(
        playing=False, cfg=SimpleNamespace(use_cuda_graph=True), graph=captured
    )
    kept = release in physics.CAPTURE_KEPT                 # this release's own step is used, graph capture included
    calls = []

    def run(*, steps):
        assert native.cfg.use_cuda_graph is kept and (native.graph is captured) is kept
        if release in ("6.0.0.0", "6.0.0.1", "6.0.1.0"):
            assert native.playing  # old native solver otherwise silently skips work
        calls.append(steps)
        if fail:
            raise RuntimeError("native partial step failure")

    monkeypatch.setattr(
        physics,
        "manager",
        lambda: SimpleNamespace(get_active_physics_engine=lambda: "newton", step=run),
    )
    monkeypatch.setattr(physics, "newton_stage", lambda stage: native)
    monkeypatch.setattr(runtime, "runtime_identity", lambda: {"release": release})
    if fail:
        with pytest.raises(RuntimeError, match="partial"):
            physics.step(object(), 17)
    else:
        physics.step(object(), 17)
    assert calls == [17]
    assert not native.playing and native.cfg.use_cuda_graph
    assert native.graph is (captured if kept else None)   # a release that steps directly drops the graph its steps made stale
    assert physics.CAPTURE_KEPT == ("6.2.0.0",)


def test_invalid_runtime_view_refuses_no_usd_or_other_backend_fallback(monkeypatch):
    view = SimpleNamespace(count=1, check=lambda: False)
    monkeypatch.setattr(
        physics,
        "manager",
        lambda: SimpleNamespace(
            get_physics_simulation_view=lambda: SimpleNamespace(
                create_rigid_body_view=lambda p: view
            )
        ),
    )
    with pytest.raises(AssetError, match="valid runtime view"):
        physics.view_for("/World/Body")


def test_runtime_dynamics_units_and_nonfinite_refusal(monkeypatch):
    import numpy as np
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdPhysics.SetStageKilogramsPerUnit(stage, 10)
    velocities = np.array([[1.0, 2.0, 3.0, 0.1, 0.2, 0.3]])
    view = SimpleNamespace(
        get_transforms=lambda: np.array([[4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0]]),
        get_velocities=lambda: velocities,
        get_masses=lambda: np.array([[2.0]]),
    )
    monkeypatch.setattr(
        physics,
        "manager",
        lambda: SimpleNamespace(
            get_active_physics_engine=lambda: "newton", get_simulation_time=lambda: 2.5
        ),
    )
    monkeypatch.setattr(physics, "view_for", lambda p: view)
    result = physics.body_state("/Body", 2, True, stage)
    assert result["orientation_wxyz"] == [1, 0, 0, 0]
    assert result["dynamics"]["mass_kg"] == 20
    assert result["dynamics"]["linear_velocity"] == [1, 2, 3]
    assert result["dynamics"]["angular_velocity"] == [0.1, 0.2, 0.3]
    velocities[0, 0] = float("nan")
    with pytest.raises(AssetError, match="finite"):
        physics.body_state("/Body", 2, True, stage)


def test_body_path_accepts_aliases_and_rejects_conflicting_targets():
    for key in ("path", "prim", "prim_path"):
        assert body_path({key: "/Robot/base"}) == "/Robot/base"
    assert body_path({"path": "/Robot", "prim": "/Robot"}) == "/Robot"
    for args in ({}, {"prim": ""}, {"path": "/A", "prim": "/B"}):
        with pytest.raises(AssetError) as error:
            body_path(args)
        assert error.value.code == "INVALID_VALUE"
