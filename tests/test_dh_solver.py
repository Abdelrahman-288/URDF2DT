"""Automatic DH equivalence against independent XML/SciPy and matrix oracles."""

from dataclasses import FrozenInstanceError, replace
from math import pi
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.verify_stage05 import reference_urdf_fk
from tests.fixtures.ur5_automatic_dh import make_ur5_fixture
from urdf2dt.config import EditorConfig
from urdf2dt.dh.dh_solver import DHSolverError, StandardDHSolver
from urdf2dt.dh.types import DHModel, DHRow, Joint, JointLimit, JointType, KinematicChain
from urdf2dt.kinematics import dh_fk, dh_frame_transforms, urdf_fk, urdf_link_transforms
from urdf2dt.pipeline import generate_automatic_model

ROOT = Path(__file__).resolve().parents[1]


def test_ur5_matches_nominal_table_without_fixture_substitution():
    run = generate_automatic_model(ROOT / "robots/ur5/ur5_serial.urdf")
    reference = make_ur5_fixture()
    model = run.automatic_model
    for actual, expected in zip(model.rows, reference.rows):
        np.testing.assert_allclose([actual.a, actual.alpha, actual.d, actual.theta_offset],
                                   [expected.a, expected.alpha, expected.d, expected.theta_offset],
                                   rtol=0, atol=1e-10)
    np.testing.assert_allclose(model.base_transform, np.diag([-1, -1, 1, 1]), atol=1e-15)
    assert not model.is_temporary_fixture
    assert model.source_sha256 == run.source.source.sha256
    assert model.joint_names == run.chain.joint_names
    with pytest.raises(FrozenInstanceError):
        model.rows[0].d = 20
    with pytest.raises(TypeError):
        model.base_transform[0][0] = 20
    changed = replace(model, rows=(replace(model.rows[0], d=20),) + model.rows[1:])
    assert model.rows[0].d == .089159 and changed.rows[0].d == 20


@pytest.mark.parametrize("fixture", ["robots/ur5/ur5_serial.urdf", "robots/examples/mixed_joints.urdf"])
def test_real_input_pipeline_against_independent_xml_fk(fixture):
    run = generate_automatic_model(ROOT / fixture)
    rng = np.random.default_rng(42)
    movable = [j for j in run.chain.joints if j.joint_type != JointType.FIXED]
    configurations = [np.zeros(len(movable))]
    for _ in range(49):
        configurations.append([rng.uniform(j.limit.lower, j.limit.upper) if j.limit else rng.uniform(-pi, pi)
                               for j in movable])
    for q in configurations:
        expected = reference_urdf_fk(run.source, q)
        np.testing.assert_allclose(urdf_fk(run.chain, q), expected, atol=1e-12, rtol=0)
        np.testing.assert_allclose(dh_fk(run.automatic_model, q), expected, atol=1e-10, rtol=0)
    assert len(urdf_link_transforms(run.chain, configurations[0])) == len(run.chain.joints) + 1
    assert len(dh_frame_transforms(run.automatic_model, configurations[0])) == len(movable) + 1


def independent_chain_fk(chain, q):
    values = iter(q)
    pose = np.eye(4)
    for joint in chain.joints:
        pose = pose @ np.array(joint.origin)
        if joint.joint_type == JointType.FIXED:
            continue
        value = next(values)
        motion = np.eye(4)
        if joint.joint_type == JointType.PRISMATIC:
            motion[:3, 3] = np.array(joint.axis) * value
        else:
            motion[:3, :3] = Rotation.from_rotvec(np.array(joint.axis) * value).as_matrix()
        pose = pose @ motion
    return pose


def as_tuple(matrix):
    return tuple(tuple(float(v) for v in row) for row in matrix)


def synthetic_chain(axes, positions, kinds=None):
    joints = []
    if kinds is None:
        kinds = [JointType.CONTINUOUS] * len(axes)
    for i, (axis, translation, kind) in enumerate(zip(axes, positions, kinds)):
        origin = np.eye(4)
        origin[:3, 3] = translation
        limit = JointLimit(-1, 1) if kind in (JointType.REVOLUTE, JointType.PRISMATIC) else None
        joints.append(Joint(f"j{i}", f"l{i}", f"l{i+1}", kind, as_tuple(origin), axis, limit))
    return KinematicChain("synthetic", "l0", f"l{len(joints)}", tuple(joints), "synthetic.urdf")


@pytest.mark.parametrize("axes,positions", [
    ([(0, 0, 1)], [(1, 2, 3)]),
    ([(0, 0, 1), (0, 0, 1)], [(0, 0, 0), (1, 0, 2)]),  # parallel
    ([(0, 0, 1), (0, 0, -1)], [(0, 0, 0), (1, 0, 2)]),  # antiparallel
    ([(0, 0, 1), (0, 0, 1)], [(0, 0, 0), (0, 0, 2)]),  # coincident
    ([(0, 0, 1), (0, 0, -1)], [(0, 0, 0), (0, 0, 2)]),
    ([(0, 0, 1), (0, 1, 0)], [(0, 0, 0), (0, 2, 3)]),  # intersecting
    ([(0, 0, 1), (0, 1, 0)], [(0, 0, 0), (1, 2, 3)]),  # skew
    ([(0, 0, 1), (0, 1, 0)], [(0, 0, 0), (1e-8, 2, 3)]),  # no distance snapping
])
@pytest.mark.parametrize("prismatic", [False, True])
def test_axis_relationships_and_single_joint(axes, positions, prismatic):
    kinds = [JointType.PRISMATIC if prismatic else JointType.CONTINUOUS] * len(axes)
    chain = synthetic_chain(axes, positions, kinds)
    model = StandardDHSolver().solve(chain, EditorConfig())
    for q in (np.zeros(len(axes)), np.linspace(-.4, .7, len(axes))):
        np.testing.assert_allclose(dh_fk(model, q), independent_chain_fk(chain, q), atol=1e-12, rtol=0)


@pytest.mark.parametrize("seed", range(20))
def test_general_chains_with_nontrivial_fixed_transforms_and_mixed_joints(seed):
    rng = np.random.default_rng(seed)
    count = 1 + seed % 7
    joints = []
    kinds = [JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC]
    for i in range(2 * count + 1):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        matrix[:3, 3] = rng.uniform(-.7, .7, 3)
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        kind = JointType.FIXED if i % 2 == 0 else kinds[(i + seed) % 3]
        limit = JointLimit(-1, 1) if kind in (JointType.REVOLUTE, JointType.PRISMATIC) else None
        joints.append(Joint(f"j{i}", f"l{i}", f"l{i+1}", kind, as_tuple(matrix), tuple(axis), limit))
    chain = KinematicChain(f"generated_{seed}", "l0", f"l{2*count+1}", tuple(joints), "synthetic.urdf")
    model = StandardDHSolver().solve(chain, EditorConfig())
    assert len(model.rows) == count
    for q in [np.zeros(count)] + [rng.uniform(-1, 1, count) for _ in range(10)]:
        expected = independent_chain_fk(chain, q)
        np.testing.assert_allclose(urdf_fk(chain, q), expected, atol=1e-12, rtol=0)
        np.testing.assert_allclose(dh_fk(model, q), expected, atol=1e-10, rtol=0)


def test_near_parallel_does_not_get_silently_approximated():
    angle = 1e-7
    chain = synthetic_chain([(0, 0, 1), (np.sin(angle), 0, np.cos(angle))], [(0, 0, 0), (1, 0, 0)])
    with pytest.raises(DHSolverError) as exc:
        StandardDHSolver().solve(chain, EditorConfig())
    assert exc.value.code == "ill_conditioned_axes"
    assert exc.value.joint_names == ("j0", "j1")


def test_finite_but_overflowing_origins_raise_structured_solver_error():
    chain = synthetic_chain([(0, 0, 1), (0, 1, 0)], [(1e308, 0, 0), (1e308, 0, 0)])
    with pytest.raises(DHSolverError):
        StandardDHSolver().solve(chain, EditorConfig())


def test_joint_sign_offset_and_prismatic_displacement_in_dh_fk():
    model = DHModel("signed", (DHRow(0, 0, .5, .3, "r", joint_sign=-1),
                              DHRow(0, 0, .4, .1, "p", JointType.PRISMATIC, -1)), "test")
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler("z", .3 - .2 + .1).as_matrix()
    expected[2, 3] = .5 + .4 - .15
    np.testing.assert_allclose(dh_fk(model, [.2, .15]), expected, atol=1e-15)


@pytest.mark.parametrize("q", [[], [0, 1], [True], [float("nan")], [float("inf")]])
def test_fk_rejects_wrong_dimension_and_nonfinite_values(q):
    chain = synthetic_chain([(0, 0, 1)], [(0, 0, 0)])
    model = StandardDHSolver().solve(chain, EditorConfig())
    for evaluator, data in ((urdf_fk, chain), (dh_fk, model)):
        with pytest.raises(ValueError):
            evaluator(data, q)
