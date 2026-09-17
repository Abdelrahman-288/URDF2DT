"""Analytical controller signs, dynamics cancellation, saturation and timing."""

from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest

from urdf2dt.control import Controller, ControllerConfig, ControlError
from urdf2dt.control.experiments import scaled_inertias, run_experiment
from urdf2dt.trajectory import MotionLimits, joint_trajectory
from urdf2dt.dynamics import DynamicsConfig, JointFriction
from tests.test_dynamics import source_xml, direct


def setup(mode="pd", kp=10., kd=2., limit=100., friction=False):
    model = direct(source_xml(), DynamicsConfig(friction=(JointFriction("j", .2, .1),) if friction else ()))
    reference = joint_trajectory(model.chain, [[.2], [.4]], MotionLimits((2.,), (5.,), (limit,)), durations=[1.])
    config = ControllerConfig(mode, (kp,), (kd,), (limit,), period=.1)
    return model, reference, Controller(model, reference, config)


def test_pd_feedback_sign_and_velocity_tracking():
    _, _, controller = setup()
    command = controller.step(0., [.1], [.3])
    np.testing.assert_allclose(command.position_error, [.1])
    np.testing.assert_allclose(command.velocity_error, [-.3])
    np.testing.assert_allclose(command.effort, [.4], atol=1e-14)


def test_gravity_compensation_at_measured_state():
    model, _, controller = setup("pd_gravity", kp=0, kd=0)
    command = controller.step(0, [.7], [0])
    np.testing.assert_allclose(command.effort, [-9.81*np.cos(.7)], atol=1e-13)
    np.testing.assert_allclose(model.forward_dynamics([.7], [0], command.effort), [0], atol=1e-13)


def test_computed_torque_cancels_dynamics_including_friction():
    model, reference, controller = setup("computed_torque", friction=True)
    for tick in range(11):
        t = tick*.1
        qd, vd, ad = reference.evaluate(t)
        q, v = qd-.03, vd+.02
        command = controller.step(t, q, v)
        expected = ad+10*.03-2*.02
        np.testing.assert_allclose(model.forward_dynamics(q, v, command.effort), expected, atol=1e-12)
        assert not any(command.saturated)


@pytest.mark.parametrize("mode", ["pd", "pd_gravity", "computed_torque"])
def test_effort_clipping_and_flags(mode):
    _, _, controller = setup(mode, kp=1000., limit=.1)
    command = controller.step(0, [-.5], [0])
    assert abs(command.effort[0]) <= .1
    assert command.saturated == (True,)
    assert abs(command.unsaturated_effort[0]) > .1


def test_reference_effort_limit_cannot_be_relaxed():
    model, reference, _ = setup(limit=.5)
    controller = Controller(model, reference, ControllerConfig("pd", (100.,), (2.,), (100.,)))
    assert controller.effective_effort_limits == (.5,)


def test_continuous_joint_error_preserves_multiple_turns():
    model = direct(source_xml())
    reference = joint_trajectory(model.chain, [[2*np.pi+.1], [2*np.pi+.1]], MotionLimits((2.,), (5.,)), durations=[1.])
    controller = Controller(model, reference, ControllerConfig("pd", (1.,), (0.,), (100.,)))
    np.testing.assert_allclose(controller.step(0, [.1], [0]).effort, [2*np.pi], atol=1e-12)


def test_timestamp_rejection_does_not_consume_tick_and_reset_is_deterministic():
    _, _, controller = setup()
    with pytest.raises(ControlError, match="Expected tick"):
        controller.step(.1, [.1], [0])
    first = controller.step(0, [.1], [0])
    assert first.hold_until == .1
    with pytest.raises(ControlError, match="Expected tick"):
        controller.step(0, [.1], [0])
    with pytest.raises(ControlError, match="finite"):
        controller.step(.1, [float("nan")], [0])
    controller.step(.1, [.1], [0])
    controller.reset()
    assert controller.step(0, [.1], [0]) == first


def test_reference_acceleration_and_end_time():
    _, reference, controller = setup("computed_torque")
    for i in range(11):
        state = reference.evaluate(i*.1)
        command = controller.step(i*.1, state[0], state[1])
        np.testing.assert_allclose(command.desired_acceleration, state[2])
    assert command.hold_until == reference.duration
    with pytest.raises(ControlError, match="duration"):
        controller.step(1.1, [.4], [0])


@pytest.mark.parametrize("changes", [{"period": 0}, {"kp": (-1.,)}, {"kd": (float("nan"),)},
                                     {"mode": "pid"}, {"effort_limits": (0.,)}, {"kd": (1., 2.)}])
def test_invalid_configuration(changes):
    with pytest.raises(ControlError):
        replace(ControllerConfig("pd", (1.,), (1.,), (10.,)), **changes)


def test_model_mismatch_and_dimension_rejected():
    model, reference, _ = setup()
    with pytest.raises(ControlError, match="match"):
        Controller(direct(source_xml(kind="prismatic")), reference, ControllerConfig("pd", (1.,), (1.,), (10.,)))
    with pytest.raises(ControlError, match="match"):
        Controller(model, reference, ControllerConfig("pd", (1., 1.), (1., 1.), (10., 10.)))


def test_inertia_sensitivity_is_physical_and_changes_predicted_acceleration():
    model, reference, _ = setup("computed_torque")
    for factor in (.8, 1., 1.2):
        altered = scaled_inertias(model, factor)
        np.testing.assert_allclose(altered.mass_matrix([.2]), model.mass_matrix([.2])*factor)
        controller = Controller(altered, reference, ControllerConfig("computed_torque", (10.,), (2.,), (100.,)))
        command = controller.step(0, [.2], [0])
        acceleration = model.forward_dynamics([.2], [0], command.effort)
        if factor == 1:
            np.testing.assert_allclose(acceleration, 0, atol=1e-13)
        else:
            assert abs(acceleration[0]) > .1


def test_stage25_identified_model_and_stage26_archive_interoperate():
    from urdf2dt.identification.io import load_parameters
    from urdf2dt.trajectory.io import load_trajectory
    root = Path(__file__).resolve().parents[1]
    model, _ = load_parameters(root/"outputs/validation_reports/stage25/scara/parameters.json")
    reference = load_trajectory(root/"outputs/validation_reports/stage26/scara/joint_quintic.json")
    controller = Controller(model, reference, ControllerConfig("computed_torque", (25.,)*4, (10.,)*4, (30.,30.,100.,10.)))
    q, v, _ = reference.evaluate(0)
    assert np.isfinite(controller.step(0, q, v).effort).all()


def test_persisted_controller_binds_model_and_reference(tmp_path):
    from urdf2dt.control.io import save_controller, load_controller
    model, reference, controller = setup()
    path = save_controller(controller, tmp_path/"controller.json")
    loaded = load_controller(path, model, reference)
    assert loaded.step(0, [.1], [0]) == controller.step(0, [.1], [0])
    with pytest.raises(ControlError, match="differ"):
        load_controller(path, scaled_inertias(model, 1.2), reference)
    changed = joint_trajectory(model.chain, [[.2], [.5]], reference.limits, durations=[1.])
    with pytest.raises(ControlError, match="differ"):
        load_controller(path, model, changed)


def test_prismatic_units_and_measured_limit_rejection():
    model = direct(source_xml(kind="prismatic", axis="0 0 1"))
    reference = joint_trajectory(model.chain, [[.1], [.1]], MotionLimits((1.,), (2.,)), durations=[1.])
    controller = Controller(model, reference, ControllerConfig("pd_gravity", (100.,), (10.,), (100.,)))
    with pytest.raises(ControlError, match="joint limits"):
        controller.step(0, [3.], [0])
    command = controller.step(0, [.08], [.1])
    np.testing.assert_allclose(command.effort, [2*9.81+2-1], atol=1e-12)


@pytest.mark.parametrize("robot", ["scara", "ur5"])
def test_reproducible_experiment(robot, tmp_path):
    report = run_experiment(robot, tmp_path/robot)
    assert len(report["cases"]) == 9
    for case in report["cases"]:
        assert len(case["commands"]) == 201
        limits = case["controller"]["effective_effort_limits"]
        assert np.all(np.abs([c["effort"] for c in case["commands"]]) <= limits)


@pytest.mark.parametrize("robot", ["scara", "ur5"])
def test_computed_torque_against_independent_engine(robot):
    pytest.importorskip("mujoco")
    from urdf2dt.dynamics.io import load_dynamic_model
    from urdf2dt.dynamics.reference import MuJoCoReference
    from urdf2dt.parser.urdf_input import URDFInput
    root = Path(__file__).resolve().parents[1]
    source = URDFInput.from_path(root/f"robots/dynamics/{robot}_dynamics.urdf")
    model = load_dynamic_model(source)
    engine = MuJoCoReference(source, model)
    start = np.array([.3,.8,.06,.2] if robot == "scara" else [.3,-1.,1.2,-1.,.8,.2])
    reference = joint_trajectory(model.chain, [start, start+.01], MotionLimits((1.,)*model.dof, (2.,)*model.dof), durations=[1.])
    controller = Controller(model, reference, ControllerConfig("computed_torque", (25.,)*model.dof, (10.,)*model.dof, (200.,)*model.dof, .1))
    for tick in range(5):
        desired, vd, ad = reference.evaluate(tick*.1)
        q, v = desired-.002, vd+.003
        command = controller.step(tick*.1, q, v)
        np.testing.assert_allclose(engine.forward(q, v, command.effort), ad+25*.002-10*.003, atol=1e-7)
