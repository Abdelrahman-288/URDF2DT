"""Closed-form integration, sampled control, failure semantics and metrics."""

from dataclasses import replace
import numpy as np
import pytest
from tests.test_dynamics import direct, source_xml
from urdf2dt.control import ControllerConfig
from urdf2dt.dynamics import DynamicsConfig
from urdf2dt.simulation import SimulationConfig, SimulationError, simulate
from urdf2dt.simulation.engine import rk4_step
from urdf2dt.simulation.metrics import tracking_metrics, compare_runs
from urdf2dt.trajectory import MotionLimits, joint_trajectory


def setup():
    source = source_xml(kind="prismatic", axis="0 0 1")
    model = direct(source, DynamicsConfig(gravity=(0.,0.,0.)))
    reference = joint_trajectory(model.chain, [[.2],[.2],[.2]], MotionLimits((1.,),(2.,)), durations=[.1,.2])
    config = ControllerConfig("pd", (10.,), (2.,), (100.,), .01)
    return source, model, reference, config


def test_rk4_fourth_order_against_analytic_oscillator():
    errors = []
    for step in (.1,.05,.025):
        q, v = np.array([1.]), np.array([0.])
        for _ in range(round(1/step)):
            q,v = rk4_step(lambda q,v,u: -q, q,v,np.array([0.]),step)
        errors.append(np.linalg.norm([q[0]-np.cos(1),v[0]+np.sin(1)]))
    assert 15 < errors[0]/errors[1] < 17
    assert 15 < errors[1]/errors[2] < 17


def test_sampled_pd_matches_exact_constant_acceleration_recurrence():
    _, model, reference, config = setup()
    run = simulate(model,reference,model,config,[.1],[.02],SimulationConfig(.002))
    q,v = .1,.02
    for i, command in enumerate(run["commands"]):
        np.testing.assert_allclose([run["position"][i][0],run["velocity"][i][0]],[q,v],atol=1e-13)
        effort = 10*(.2-q)-2*v
        assert command["effort"][0] == pytest.approx(effort)
        a = effort/2
        q,v = q+.01*v+.5*.01**2*a,v+.01*a
    assert run["integration_steps"] == 150
    assert run["commands"][-1]["hold_until"] == reference.duration
    again = simulate(model,reference,model,config,[.1],[.02],SimulationConfig(.002))
    assert run["position"] == again["position"]


@pytest.mark.parametrize("config", [dict(step=0),dict(step=float("nan")),dict(backend="unknown"),dict(max_steps=0)])
def test_bad_configuration(config):
    with pytest.raises(SimulationError):
        SimulationConfig(**config)


@pytest.mark.parametrize("settings, q, v, message", [
    (SimulationConfig(.003),[.1],[0.],"integer multiple"),
    (SimulationConfig(.002,max_steps=1),[.1],[0.],"budget"),
    (SimulationConfig(),[float("nan")],[0.],"nonfinite"),
    (SimulationConfig(),[.1,.2],[0.],"state"),
    (SimulationConfig(),[2.1],[0.],"limit"),
    (SimulationConfig(),[1.99],[10.],"limit"),
    (SimulationConfig(backend="mujoco_rk4_reference"),[.1],[0.],"source URDF"),
])
def test_invalid_grid_state_or_missing_source(settings,q,v,message):
    _,model,reference,config = setup()
    with pytest.raises(SimulationError,match=message):
        simulate(model,reference,model,config,q,v,settings)


def test_saturation_is_applied_to_plant():
    _,model,reference,config = setup()
    run = simulate(model,reference,model,replace(config,effort_limits=(.1,)),[0.],[0.])
    assert all(c["saturated"][0] for c in run["commands"])
    np.testing.assert_allclose(run["position"][-1],[.5*(.1/2)*.3**2],atol=1e-14)


def test_metrics_known_constant_error_and_no_settling():
    _,model,reference,config = setup()
    run = simulate(model,reference,model,replace(config,kp=(0.,),kd=(0.,)),[.1],[0.])
    metrics = tracking_metrics(run,model,reference,move_end=.1,steady_window=.1,settling_band=[.01])
    np.testing.assert_allclose(metrics["joint_position_rmse"],[.1])
    assert metrics["cartesian_position_rmse_m"] == pytest.approx(.1)
    assert metrics["orientation_rmse_rad"] == 0
    assert metrics["settling_time_after_move_s"] == [None]
    assert metrics["overshoot_percent"] == [None]
    assert metrics["saturation_duration_s"] == [0.]
    run["time"][-1] -= .001
    with pytest.raises(ValueError,match="complete, uniform"):
        tracking_metrics(run,model,reference,move_end=.1,steady_window=.1,settling_band=[.01])


def test_metrics_stationary_exact_tracking_settles_immediately():
    _,model,reference,config = setup()
    run = simulate(model,reference,model,config,[.2],[0.])
    metrics = tracking_metrics(run,model,reference,move_end=.1,steady_window=.1,settling_band=[.01])
    assert metrics["settling_time_after_move_s"] == [0.]
    assert metrics["steady_state_rmse"] == [0.]
    assert compare_runs(run,run)["position_max_difference"] == [0.]


def test_independent_mujoco_closed_loop():
    pytest.importorskip("mujoco")
    source,model,reference,config = setup()
    primary = simulate(model,reference,model,config,[.1],[.02])
    other = simulate(model,reference,model,config,[.1],[.02],SimulationConfig(backend="mujoco_rk4_reference"),source=source)
    assert max(compare_runs(primary,other)["position_max_difference"]) < 1e-10


def test_replay_from_model_reference_controller_archives(tmp_path):
    from urdf2dt.dynamics.io import export_model, load_model_archive
    from urdf2dt.trajectory.io import export_trajectory, load_trajectory
    from urdf2dt.control import Controller
    from urdf2dt.control.io import save_controller, load_controller
    source,model,reference,config = setup()
    export_model(model,source,tmp_path/"plant.json")
    export_trajectory(reference,source,tmp_path/"reference.json",samples=31)
    save_controller(Controller(model,reference,config),tmp_path/"controller.json")
    loaded_model = load_model_archive(tmp_path/"plant.json")
    loaded_reference = load_trajectory(tmp_path/"reference.json")
    loaded_controller = load_controller(tmp_path/"controller.json",loaded_model,loaded_reference)
    original = simulate(model,reference,model,config,[.1],[.02])
    replay = simulate(loaded_model,loaded_reference,loaded_model,
                      ControllerConfig(**loaded_controller.snapshot()["config"]),[.1],[.02])
    assert original["position"] == replay["position"]
    assert original["commands"] == replay["commands"]


def test_terminal_hold_required():
    _,model,_,config = setup()
    moving = joint_trajectory(model.chain,[[.1],[.2]],MotionLimits((1.,),(10.,)),durations=[.3])
    run = simulate(model,moving,model,config,[.1],[0.])
    with pytest.raises(ValueError,match="stationary hold"):
        tracking_metrics(run,model,moving,move_end=.1,steady_window=.1,settling_band=[.01])
