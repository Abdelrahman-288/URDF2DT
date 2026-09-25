from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
import sympy as sp
from urdf2dt.symbolic import symbolic_jacobian
from urdf2dt.jacobian import geometric_jacobian
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.kinematics import dh_fk, urdf_fk
from urdf2dt.dh.types import DHModel, DHRow, JointType
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt._transforms import rpy_transform

ROOT = Path(__file__).resolve().parents[1]


def finite_difference(fk, q, step=1e-6):
    q = np.asarray(q, dtype=float)
    rotation = np.asarray(fk(q))[:3, :3]
    result = np.zeros((6, len(q)))
    for i in range(len(q)):
        delta = np.zeros_like(q)
        delta[i] = step
        difference = (np.asarray(fk(q + delta)) - np.asarray(fk(q - delta))) / (2 * step)
        result[:3, i] = difference[:3, 3]
        # dR/dq R^T gives angular velocity in the reference/base frame.
        skew = difference[:3, :3] @ rotation.T
        result[3:, i] = [(skew[2, 1] - skew[1, 2])/2,
                        (skew[0, 2] - skew[2, 0])/2,
                        (skew[1, 0] - skew[0, 1])/2]
    return result


def test_exact_planar_jacobian_and_singular_pose():
    model = DHModel('planar', (DHRow(2, 0, 0, 0, 'shoulder'), DHRow(1, 0, 0, 0, 'elbow')), 'test')
    result = symbolic_jacobian(model)
    q1, q2 = result.symbols
    expected = sp.Matrix([[-2*sp.sin(q1)-sp.sin(q1+q2), -sp.sin(q1+q2)],
                          [2*sp.cos(q1)+sp.cos(q1+q2), sp.cos(q1+q2)],
                          [0,0], [0,0], [0,0], [1,1]])
    assert sp.trigsimp(result.matrix - expected) == sp.zeros(6,2)
    assert sp.trigsimp(result.matrix[:2,:].det() - 2*sp.sin(q2)) == 0
    np.testing.assert_allclose(result.evaluate([0,0]), [[0,0],[3,1],[0,0],[0,0],[0,0],[1,1]])
    text, latex = result.documents()
    assert 'J(q)' in text and 'not RPY' in text and 'shoulder' in text
    assert 'J(q)' in latex and r'\begin{matrix}' in latex


@pytest.mark.parametrize('robot', ['ur5/ur5_serial.urdf','scara/scara_rrpr.urdf'])
@pytest.mark.parametrize('flip', [None, 'x', 'z'])
def test_symbolic_against_urdf_finite_differences(robot, flip):
    run = generate_automatic_model(ROOT/'robots'/robot)
    model = run.automatic_model
    if flip:
        for index in range(1, len(model.rows)+1):
            model = recompute_model(model, index, FrameEdit(axis_flip=flip))
        model = recompute_model(model, len(model.rows), FrameEdit(.017, .24))
    result = symbolic_jacobian(model)
    for q in np.random.default_rng(72).uniform(-.2,.2,(2,len(model.rows))):
        numeric = geometric_jacobian(model,q)
        np.testing.assert_allclose(result.evaluate(q),numeric,atol=2e-12)
        np.testing.assert_allclose(numeric,finite_difference(lambda p: urdf_fk(run.chain,p),q),atol=2e-9)
        velocity = np.linspace(-.2,.3,len(q))
        np.testing.assert_allclose(np.asarray(result.evaluate(q)) @ velocity,
                                   finite_difference(lambda p: dh_fk(model,p),q) @ velocity,atol=2e-9)


def test_signed_mixed_joints_fixed_base_tool_and_tiny_constants():
    model = DHModel('mixed',(DHRow(1e-13,.23,.4,-.12,'turn',JointType.CONTINUOUS,-1),
                            DHRow(.3,-.4,.2,.13,'slide',JointType.PRISMATIC,-1)), 'test',
                    base_transform=rpy_transform((.3,-.2,.1),(.4,.2,-.3)),
                    tool_transform=rpy_transform((.1,.2,.3),(-.2,.3,.1)))
    result = symbolic_jacobian(model)
    for q in ((.2,.07),(-.1,-.03)):
        np.testing.assert_allclose(result.evaluate(q),finite_difference(lambda p: dh_fk(model,p),q),atol=2e-10)
        np.testing.assert_allclose(result.evaluate(q),geometric_jacobian(model,q),atol=1e-14)
    assert list(result.matrix[3:,1]) == [0,0,0]
    for q in [(1,), (float('nan'),0), (float('inf'),0)]:
        with pytest.raises(ValueError): result.evaluate(q)
        with pytest.raises(ValueError): geometric_jacobian(model,q)
    with pytest.raises(ValueError,match='12'):
        symbolic_jacobian(replace(model,rows=tuple(replace(model.rows[0],joint_name=str(i)) for i in range(13))))
