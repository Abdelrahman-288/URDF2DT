"""Requires optional SDP solver; mandatory in Python 3.12 CI jobs."""

from dataclasses import replace
import numpy as np
import pytest

pytest.importorskip("cvxpy")
from urdf2dt.dynamics.model import DynamicsConfig, JointFriction
from urdf2dt.identification.fit import identify, evaluate
from urdf2dt.identification.io import export_parameters, load_parameters
from urdf2dt.identification.regressor import InertialRegressor
from tests.test_dynamics import source_xml, direct
from tests.test_identification import pendulum_data


def setup_pendulum():
    source = source_xml()
    model = direct(source, DynamicsConfig(friction=(JointFriction("j", .2, .1),)))
    regressor = InertialRegressor(model.chain)
    truth = regressor.parameters(model)
    train = pendulum_data(regressor, truth)
    held = pendulum_data(regressor, truth, "held", 91)
    return source, regressor, truth, train, held


def test_known_identifiable_combinations_and_friction_recovery(tmp_path):
    source, regressor, truth, train, held = setup_pendulum()
    baseline = truth*.7
    result = identify(regressor, train, prior=baseline)
    assert result.report["identifiable_rank"] == 5  # hy-axis Iyy, hx,hz,viscous,Coulomb
    assert result.report["nullity"] == 7
    estimated = np.asarray(result.parameters)
    np.testing.assert_allclose(estimated[[1, 3, 5, 10, 11]], truth[[1, 3, 5, 10, 11]], atol=2e-6)
    report = evaluate(regressor, result, train, held, baseline)
    assert report["fitted_rmse_per_joint"][0] < 1e-6
    assert report["baseline_rmse_per_joint"][0] > 1
    assert result.report["minimum_pseudo_inertia_eigenvalue"] > 0
    path = export_parameters(tmp_path/"params.json", source, regressor, result)
    restored, archive = load_parameters(path)
    assert archive["report"]["training_sha256"] == train.digest
    q, v, a = held.q[2], held.velocity[2], held.acceleration[2]
    np.testing.assert_allclose(restored.inverse_dynamics(q, v, a), regressor.matrix(q, v, a)@estimated, atol=1e-12)


def test_insufficient_excitation_rejected_before_solver():
    _, regressor, truth, train, _ = setup_pendulum()
    zero = ((0.,),)*len(train.time)
    with pytest.raises(ValueError, match="Insufficient excitation"):
        identify(regressor, replace(train, q=zero, velocity=zero, acceleration=zero))


def test_uncertainty_increases_with_noise_and_no_held_out_dependency():
    _, regressor, truth, train, held = setup_pendulum()
    noise = np.random.default_rng(27).normal(0, .02, (len(train.time), 1))
    result = identify(regressor, replace(train, effort=np.asarray(train.effort)+noise))
    assert min(result.report["beta_standard_error"]) > 0
    assert result.report["hardware_accuracy_validated"] is False
    with pytest.raises(ValueError, match="differs"):
        evaluate(regressor, result, train, held)


def test_identification_works_when_urdf_inertias_missing():
    from urdf2dt.identification.io import source_regressor
    _, _, truth, _, _ = setup_pendulum()
    regressor = source_regressor(source_xml(body=""))
    train = pendulum_data(regressor, truth)
    result = identify(regressor, train)
    assert result.report["solver_status"] == "optimal"
    np.testing.assert_allclose(np.asarray(result.parameters)[[1, 3, 5, 10, 11]], truth[[1, 3, 5, 10, 11]], atol=2e-6)


@pytest.mark.parametrize("robot", ["ur5", "scara"])
def test_independent_robot_benchmark(robot, tmp_path):
    pytest.importorskip("mujoco")
    from urdf2dt.identification.benchmark import run_benchmark
    from tests.test_identification import ROOT
    report = run_benchmark(ROOT/f"robots/dynamics/{robot}_dynamics.urdf", tmp_path/robot, samples=80)
    assert report["passed"]
    assert report["identification"]["nullity"] > 0
    assert report["identifiable_combination_relative_error"] < .01
