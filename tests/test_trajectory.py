"""Analytical derivatives, interval constraints, Cartesian IK and durable references."""

from dataclasses import FrozenInstanceError
from pathlib import Path
import json
import numpy as np
import pytest

from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.identification.data import canonical
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.trajectory import MotionLimits, Trajectory, TrajectoryError, joint_trajectory, cartesian_line
from urdf2dt.trajectory.cartesian import geometric_jacobian
from urdf2dt.trajectory.io import export_trajectory, load_trajectory
from urdf2dt.trajectory.validation import dynamic_feasibility

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def scara():
    return load_dynamic_model(ROOT/"robots/dynamics/scara_dynamics.urdf")


@pytest.fixture(scope="module")
def ur5():
    return load_dynamic_model(ROOT/"robots/dynamics/ur5_dynamics.urdf")


LIMITS = MotionLimits((1., 1., .15, 1.), (2., 2., .3, 2.), (30., 30., 100., 10.))
START = np.array([.3, .8, .06, .2])
END = np.array([.34, .77, .075, .2])


@pytest.mark.parametrize("method", ["cubic", "quintic"])
def test_endpoints_and_analytic_derivatives(scara, method):
    t = joint_trajectory(scara.chain, [START, END], LIMITS, method=method, durations=[2.])
    np.testing.assert_allclose(t.evaluate(0)[0], START, atol=1e-14)
    np.testing.assert_allclose(t.evaluate(2)[0], END, atol=1e-14)
    np.testing.assert_allclose(t.evaluate(0)[1], 0, atol=1e-14)
    np.testing.assert_allclose(t.evaluate(2)[1], 0, atol=1e-14)
    if method == "quintic":
        np.testing.assert_allclose(t.evaluate(0)[2], 0, atol=1e-14)
        np.testing.assert_allclose(t.evaluate(2)[2], 0, atol=1e-14)
    dt, center = 1e-5, .7
    np.testing.assert_allclose((t.evaluate(center+dt)[0]-t.evaluate(center-dt)[0])/(2*dt), t.evaluate(center)[1], atol=1e-10)
    np.testing.assert_allclose((t.evaluate(center+dt)[1]-t.evaluate(center-dt)[1])/(2*dt), t.evaluate(center)[2], atol=1e-10)


def test_waypoint_stops_are_c2(scara):
    points = np.array([START, END, START])
    t = joint_trajectory(scara.chain, points, LIMITS)
    for i, time in enumerate(t.times):
        q, v, a = t.evaluate(time)
        np.testing.assert_allclose(q, points[i], atol=1e-12)
        np.testing.assert_allclose(v, 0, atol=1e-12)
        np.testing.assert_allclose(a, 0, atol=1e-11)
    for d in range(3):
        np.testing.assert_allclose(t.segment(0, 1, d), t.segment(1, 0, d), atol=1e-10)
    with pytest.raises(TrajectoryError, match="discontinuity"):
        joint_trajectory(scara.chain, points, LIMITS, method="cubic")


def test_minimum_time_and_limits_detect_between_samples(scara):
    t = joint_trajectory(scara.chain, [START, END], LIMITS)
    assert max(np.array(t.bounds["velocity_max"])/LIMITS.velocity) <= 1
    assert max(np.array(t.bounds["acceleration_max"])/LIMITS.acceleration) <= 1
    with pytest.raises(TrajectoryError, match="duration_too_short"):
        joint_trajectory(scara.chain, [START, END], LIMITS, durations=[.01])
    # Same endpoints and zero endpoint velocity; interior velocity is excessive.
    with pytest.raises(TrajectoryError, match="limit_violation"):
        Trajectory(t.chain, [0, .01], t.coefficients, LIMITS)


@pytest.mark.parametrize("values", [(0, 1, 1, 1), (1, float("nan"), 1, 1), (-1, 1, 1, 1)])
def test_invalid_limits(values):
    with pytest.raises(TrajectoryError, match="invalid_limits"):
        MotionLimits(values, (1,)*4)


def test_invalid_waypoints_position_and_time(scara):
    with pytest.raises(TrajectoryError, match="limit_violation"):
        joint_trajectory(scara.chain, [START, [.3,.8,.3,.2]], LIMITS)
    with pytest.raises(TrajectoryError, match="invalid_waypoints"):
        joint_trajectory(scara.chain, [[0], [1]], LIMITS)
    with pytest.raises(TrajectoryError, match="invalid_duration"):
        joint_trajectory(scara.chain, [START, END], LIMITS, durations=[float("nan")])
    t = joint_trajectory(scara.chain, [START, START], LIMITS)
    np.testing.assert_allclose(t.evaluate(t.duration/2)[0], START)
    for time in (-1, t.duration+1, float("nan")):
        with pytest.raises(TrajectoryError, match="time_out_of_range"):
            t.evaluate(time)
    with pytest.raises(FrozenInstanceError):
        t.times = (0, 1)


def test_continuous_coordinates_are_not_wrapped(scara):
    end = START.copy()
    end[-1] += 2*np.pi
    t = joint_trajectory(scara.chain, [START, end], LIMITS)
    np.testing.assert_allclose(t.evaluate(t.duration)[0], end, atol=1e-12)


def test_geometric_jacobian_finite_difference(scara):
    jac = geometric_jacobian(scara.chain, START)
    for j in range(4):
        delta = np.eye(4)[j]*1e-6
        numeric = (scara.tip_transform(START+delta)[:3,3]-scara.tip_transform(START-delta)[:3,3])/2e-6
        np.testing.assert_allclose(jac[:3,j], numeric, atol=1e-9)


def test_scara_cartesian_path_and_analytic_derivatives(scara):
    target = scara.tip_transform(END)
    t = cartesian_line(scara.chain, START, target, LIMITS, knots=17)
    samples = t.sample(127)
    actual = np.asarray(samples["tip_pose"])[:,:3,3]
    np.testing.assert_allclose(actual, samples["desired_tip_position"], atol=1e-7)
    assert t.metadata["max_position_error_m"] < 1e-7
    assert "desired_tip_rotation" not in samples
    np.testing.assert_allclose(t.evaluate(0)[1], 0, atol=1e-11)
    np.testing.assert_allclose(t.evaluate(t.duration)[2], 0, atol=1e-9)
    time, dt = t.duration*.43, 1e-6
    np.testing.assert_allclose((t.evaluate(time+dt)[0]-t.evaluate(time-dt)[0])/(2*dt), t.evaluate(time)[1], atol=1e-8)
    np.testing.assert_allclose((t.evaluate(time+dt)[1]-t.evaluate(time-dt)[1])/(2*dt), t.evaluate(time)[2], atol=1e-7)


def test_ur5_full_pose_path(ur5):
    start = np.array([.3,-1.,1.2,-1.,.8,.2])
    target = ur5.tip_transform(start+[.03,-.02,.03,.01,-.02,.01])
    t = cartesian_line(ur5.chain, start, target, MotionLimits((1,)*6,(2,)*6), task="pose", knots=17)
    samples = t.sample(91)
    np.testing.assert_allclose(np.asarray(samples["tip_pose"])[:,:3,:3], samples["desired_tip_rotation"], atol=1e-7)
    np.testing.assert_allclose(ur5.tip_transform(t.evaluate(t.duration)[0]), target, atol=1e-7)


def test_cartesian_unreachable_singular_and_branch_jump(scara, ur5):
    target = scara.tip_transform(END)
    unreachable = target.copy()
    unreachable[0,3] += 10
    with pytest.raises(TrajectoryError, match="ik_unreachable"):
        cartesian_line(scara.chain, START, unreachable, LIMITS, knots=5)
    with pytest.raises(TrajectoryError, match="singularity"):
        cartesian_line(scara.chain, START, target, LIMITS, task="pose")
    with pytest.raises(TrajectoryError, match="singularity"):
        cartesian_line(ur5.chain, np.zeros(6), ur5.tip_transform(np.zeros(6)), MotionLimits((1,)*6,(2,)*6), task="pose")
    with pytest.raises(TrajectoryError, match="branch_jump"):
        cartesian_line(scara.chain, START, target, LIMITS, max_normalized_step=1e-8, knots=5)


def test_cartesian_duration_rejected(scara):
    with pytest.raises(TrajectoryError, match="duration_too_short"):
        cartesian_line(scara.chain, START, scara.tip_transform(END), LIMITS, knots=5, duration=.001)


def test_dynamic_effort_gate_and_stage25_model(scara):
    t = joint_trajectory(scara.chain, [START, END], LIMITS)
    report = dynamic_feasibility(t, scara, samples=31)
    assert report["effort_limits_checked"]
    limited = joint_trajectory(scara.chain, [START, END], MotionLimits(LIMITS.velocity,LIMITS.acceleration,(.001,)*4))
    with pytest.raises(TrajectoryError, match="effort_limit"):
        dynamic_feasibility(limited, scara, samples=31)
    from urdf2dt.identification.io import load_parameters
    fitted, _ = load_parameters(ROOT/"outputs/validation_reports/stage25/scara/parameters.json")
    assert dynamic_feasibility(t, fitted, samples=31)["effort_limits_checked"]


def test_persistence_and_tampered_samples(scara, tmp_path):
    from hashlib import sha256
    source = URDFInput.from_path(ROOT/"robots/dynamics/scara_dynamics.urdf")
    t = joint_trajectory(scara.chain, [START, END, START], LIMITS)
    path = export_trajectory(t, source, tmp_path/"reference.json", 31)
    restored = load_trajectory(path)
    assert restored.sample(31) == t.sample(31)
    with pytest.raises(FileExistsError):
        export_trajectory(t, source, path)
    data = json.loads(path.read_text())
    data["payload"]["samples"]["velocity"][2][0] += 1
    data["sha256"] = sha256(canonical(data["payload"])).hexdigest()
    path.write_text(json.dumps(data))
    with pytest.raises(TrajectoryError, match="archive_mismatch"):
        load_trajectory(path)


@pytest.mark.parametrize("robot", ["ur5", "scara"])
def test_end_to_end_demo(robot, tmp_path):
    from urdf2dt.trajectory.demo import run_demo
    report = run_demo(robot, tmp_path/robot)
    assert report["passed"]
    for name in ("joint_quintic", "joint_cubic", "cartesian"):
        reference = load_trajectory(tmp_path/robot/f"{name}.json")
        assert reference.duration > 0
