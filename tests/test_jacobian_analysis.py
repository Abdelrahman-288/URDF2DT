import numpy as np
import pytest
from pathlib import Path
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.jacobian_analysis import analyze_jacobian
from urdf2dt.inspection import physical_link_poses
from urdf2dt.kinematics import urdf_link_transforms
from urdf2dt._transforms import inverse, multiply, rpy_transform
from urdf2dt.dh.recompute import FrameEdit, recompute_model

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize("robot", ["ur5/ur5_serial.urdf", "scara/scara_rrpr.urdf"])
@pytest.mark.parametrize("dh", [False, True])
@pytest.mark.parametrize("moving", [False, True])
def test_relative_jacobian_finite_difference(robot, dh, moving):
    run = generate_automatic_model(ROOT/"robots"/robot)
    chain = run.chain
    model = run.automatic_model if dh else None
    if dh:
        model = recompute_model(model, 1, FrameEdit(axis_flip="z"))
    q = np.linspace(.03,.12,len(chain.joint_names))
    names = [chain.base_link] + [j.child_link for j in chain.joints]
    reference = names[min(2,len(names)-1)] if moving else names[0]
    frames = {"offset": {"link": names[-1], "xyz": [.1,-.02,.03], "rpy": [.2,.1,-.3]}}
    result = analyze_jacobian(chain,q,reference,"offset",np.ones(len(q)),model,frames)
    def fk(p):
        poses = urdf_link_transforms(chain,p) if model is None else physical_link_poses(chain,model,p)
        target = multiply(poses[-1],rpy_transform(frames["offset"]["xyz"],frames["offset"]["rpy"]))
        return np.asarray(multiply(inverse(poses[names.index(reference)]),target))
    expected = np.zeros((6,len(q)))
    for i in range(len(q)):
        dq = np.zeros(len(q)); dq[i]=1e-6
        derivative=(fk(q+dq)-fk(q-dq))/2e-6
        expected[:3,i]=derivative[:3,3]
        skew=derivative[:3,:3] @ fk(q)[:3,:3].T
        expected[3:,i]=[skew[2,1],skew[0,2],skew[1,0]]
    np.testing.assert_allclose(result["matrix"],expected,atol=2e-9)
    np.testing.assert_allclose(result["twist"],expected @ np.ones(len(q)),atol=4e-9)
    np.testing.assert_array_equal(q,np.linspace(.03,.12,len(chain.joint_names)))
    same=analyze_jacobian(chain,q,reference,reference)
    np.testing.assert_allclose(same["matrix"],0,atol=1e-15)
    assert same["rank"] == 0 and same["condition_number"] is None

@pytest.mark.parametrize("length", [0,-1,float("nan")])
def test_invalid_scale(length):
    run=generate_automatic_model(ROOT/"robots/scara/scara_rrpr.urdf")
    with pytest.raises(ValueError):
        analyze_jacobian(run.chain,[0]*4,run.chain.base_link,run.chain.base_link,characteristic_length=length)
