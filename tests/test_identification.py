"""Source/units, linear regressor, physical reconstruction and persistent datasets."""

from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest

from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.dynamics.model import DynamicModel, DynamicsConfig, JointFriction
from urdf2dt.identification.data import IdentificationData, CurrentCalibration, joint_units, require_held_out
from urdf2dt.identification.regressor import InertialRegressor
from urdf2dt.identification.io import source_regressor, trust_manufacturer, load_parameters
from urdf2dt.parser.urdf_input import URDFInput
from tests.test_dynamics import source_xml, direct

ROOT = Path(__file__).resolve().parents[1]


def pendulum_data(regressor, parameters, name="train", seed=12):
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 15, 160)
    phase = rng.uniform(-2, 2)
    q = (np.sin(t*.7+phase)+.3*np.sin(t*1.3))[:, None]
    v = (.7*np.cos(t*.7+phase)+.39*np.cos(t*1.3))[:, None]
    a = (-.49*np.sin(t*.7+phase)-.507*np.sin(t*1.3))[:, None]
    effort = np.array([regressor.matrix(qi, vi, ai)@parameters for qi, vi, ai in zip(q, v, a)])
    return IdentificationData(str(regressor.chain.source_sha256), regressor.chain.joint_names,
        joint_units(regressor.chain), (name,)*len(t), tuple(t), tuple(map(tuple, q)), tuple(map(tuple, v)),
        tuple(map(tuple, a)), tuple(map(tuple, effort)), "simulated", "Analytic derivatives; exact effort; synchronized; no noise/filtering", "test")


@pytest.mark.parametrize("robot", ["ur5", "scara"])
def test_regressor_matches_validated_dynamics(robot):
    model = load_dynamic_model(ROOT/f"robots/dynamics/{robot}_dynamics.urdf")
    markers = ("tool0",) if robot == "ur5" else ()
    model = DynamicModel(model.chain, model.inertials, DynamicsConfig(friction=tuple(
        JointFriction(name, .2, .1) for name in model.chain.joint_names)))
    regressor = InertialRegressor(model.chain, model.config.gravity, markers)
    parameters = regressor.parameters(model)
    rebuilt = regressor.model(parameters, "round-trip check")
    rng = np.random.default_rng(2503)
    for _ in range(15):
        q, v, a = rng.normal(size=(3, model.dof))
        expected = model.inverse_dynamics(q, v, a)
        np.testing.assert_allclose(regressor.matrix(q, v, a)@parameters, expected, atol=1e-12)
        np.testing.assert_allclose(rebuilt.inverse_dynamics(q, v, a), expected, atol=1e-12)


def test_missing_inertias_do_not_prevent_regressor_or_generic_prior():
    regressor = source_regressor(source_xml(body=""))
    assert regressor.model(regressor.generic_prior(), "arbitrary initialization").dof == 1


def test_dataset_versioning_ownership_and_integrity(tmp_path):
    model = direct(source_xml())
    regressor = InertialRegressor(model.chain)
    data = pendulum_data(regressor, regressor.parameters(model))
    path = data.save(tmp_path/"data.json")
    assert IdentificationData.load(path) == data
    with pytest.raises(FileExistsError):
        data.save(path)
    envelope = json.loads(path.read_text())
    envelope["payload"]["effort"][0][0] += 1
    path.write_text(json.dumps(envelope))
    with pytest.raises(ValueError, match="checksum"):
        IdentificationData.load(path)


@pytest.mark.parametrize("change", [
    {"position_units": ("degrees",)}, {"source_sha256": "bad"}, {"processing": ""},
    {"time": (0.,)*160}, {"q": ((float("nan"),),)*160}, {"effort": ((1., 2.),)*160},
    {"origin": "hardware-proven"}, {"acquisition_id": ""},
])
def test_bad_dataset_rejected(change):
    model = direct(source_xml())
    regressor = InertialRegressor(model.chain)
    with pytest.raises(ValueError):
        replace(pendulum_data(regressor, regressor.parameters(model)), **change)


def test_split_rejects_trajectory_leakage_and_relabelled_samples():
    model = direct(source_xml())
    regressor = InertialRegressor(model.chain)
    data = pendulum_data(regressor, regressor.parameters(model))
    with pytest.raises(ValueError, match="identifiers overlap"):
        require_held_out(data, data)
    with pytest.raises(ValueError, match="identical states"):
        require_held_out(data, replace(data, trajectory=("renamed",)*len(data.time)))
    with pytest.raises(ValueError, match="source"):
        replace(data, source_sha256="0"*64).validate_chain(regressor.chain)


def test_current_calibration_signed_gain_and_linear_units():
    cal = CurrentCalibration((2., -3.), (.1, -.2), ("N*m", "N"), "Bench calibration R2; transmission included")
    np.testing.assert_allclose(cal.convert([[1., 2.]], ("rad", "m")), [[1.8, -6.6]])
    with pytest.raises(ValueError, match="units"):
        cal.convert([[1., 2.]], ("rad", "rad"))
    with pytest.raises(ValueError):
        CurrentCalibration((0.,), (0.,), ("N*m",), "invalid")


def test_current_dataset_retains_calibration_and_checks_conversion(tmp_path):
    model = direct(source_xml())
    regressor = InertialRegressor(model.chain)
    data = pendulum_data(regressor, regressor.parameters(model))
    cal = CurrentCalibration((2.,), (.1,), ("N*m",), "Test-only calibrated torque/A")
    currents = np.asarray(data.effort)/2+.1
    converted = replace(data, origin="measured", current_calibration=cal, current_ampere=currents)
    assert IdentificationData.load(converted.save(tmp_path/"current.json")) == converted
    with pytest.raises(ValueError, match="does not match"):
        replace(converted, current_ampere=currents+1)
    with pytest.raises(ValueError, match="requires"):
        replace(data, current_ampere=currents)


def test_physical_parameters_rejected():
    regressor = source_regressor(source_xml())
    p = regressor.generic_prior()
    p[4] = 2
    with pytest.raises(ValueError, match="triangle"):
        regressor.model(p, "bad inertia")
    p = regressor.generic_prior()
    p[1] = 10
    with pytest.raises(ValueError):
        regressor.model(p, "COM outside valid moments")


def test_trusted_manufacturer_mode_is_explicit_and_preserves_dynamics(tmp_path):
    source = source_xml()
    with pytest.raises(ValueError, match="provenance"):
        trust_manufacturer(source, tmp_path/"bad.json", "")
    path = trust_manufacturer(source, tmp_path/"trusted.json", "Analytical test fixture, accepted for this test")
    model, data = load_parameters(path)
    assert data["report"]["fitting_performed"] is False
    np.testing.assert_allclose(model.inverse_dynamics([.2], [.3], [.4]), direct(source).inverse_dynamics([.2], [.3], [.4]))


def test_calibrated_motor_inertia_and_friction_recovery(tmp_path):
    from urdf2dt.identification.motor import identify_motor, evaluate_motor, save_motor_parameters, load_motor_parameters
    model = direct(source_xml())
    regressor = InertialRegressor(model.chain)
    parameters = regressor.parameters(model)
    def measured_signal(name, seed):
        data = pendulum_data(regressor, parameters, name, seed)
        effort = (np.asarray(data.effort)+.3*np.asarray(data.acceleration)
                  +.2*np.asarray(data.velocity)+.1*np.sign(data.velocity))
        return replace(data, effort=effort, current_calibration=CurrentCalibration((2.,), (0.,),
            ("N*m",), "Synthetic calibration 2 Nm/A"), current_ampere=effort/2)
    train, held = measured_signal("train", 12), measured_signal("held", 91)
    result = identify_motor(model, train, motor_speed_ratio=[10.], calibration_provenance="Analytical known rigid model, synthetic gearbox")
    np.testing.assert_allclose(result["rotor_inertia_kg_m2"], [.003], atol=1e-12)
    np.testing.assert_allclose(result["reflected_inertia_viscous_coulomb"], [[.3, .2, .1]], atol=1e-12)
    assert evaluate_motor(model, result, train, held)["fitted_rmse_per_joint"][0] < 1e-12
    archived = load_motor_parameters(save_motor_parameters(tmp_path/"motor.json", result), model)
    assert evaluate_motor(model, archived, train, held)["fitted_rmse_per_joint"][0] < 1e-12
    with pytest.raises(ValueError, match="calibration"):
        identify_motor(model, pendulum_data(regressor, parameters), motor_speed_ratio=[10.], calibration_provenance="known")
