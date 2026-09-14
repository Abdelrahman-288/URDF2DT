from dataclasses import replace
from math import pi
from pathlib import Path

import numpy as np
import pytest

from urdf2dt.config import EditorConfig, ValidationConfig
from urdf2dt.dh.global_validation import FKFunctions, pose_errors, sample_configurations, validate_global_fk
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.types import IDENTITY
from urdf2dt.pipeline import generate_automatic_model
from scripts.verify_stage05 import reference_urdf_fk


@pytest.fixture
def run():
    return generate_automatic_model(Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf")


def test_automatic_and_independent_oracle(run):
    report = validate_global_fk(run.chain, run.automatic_model)
    data = report.to_dict()
    assert report.passed and len(data["samples"]) == 50
    fk = FKFunctions(run.chain, run.automatic_model)
    for sample in data["samples"]:
        q = sample["q"]
        np.testing.assert_allclose(np.array(fk.urdf(q)), reference_urdf_fk(run.source, q), atol=1e-12)
    assert report.json_text == validate_global_fk(run.chain, run.automatic_model).json_text


def test_edited_session(run):
    session = EditorSession(run.automatic_model)
    for i in range(1, 7):
        session.unlock_next()
        edit = FrameEdit(.05, .2 if i == 6 else 0.) if i in (2, 3, 6) else FrameEdit()
        session.propose_edit(i, edit)
        session.accept()
    assert validate_global_fk(run.chain, session.state.working_model).passed


def test_corruption_and_attribution(run):
    rows = list(run.automatic_model.rows)
    rows[2] = replace(rows[2], alpha=rows[2].alpha + .1)
    report = validate_global_fk(run.chain, replace(run.automatic_model, rows=tuple(rows)))
    assert not report.passed
    assert report.to_dict()["diagnostic"]["first_frame"] is not None
    assert "frame" in report.markdown()


def test_tool_fault_cannot_hide_at_zero(run):
    tool = ((1.,0.,0.,.2),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))
    report = validate_global_fk(run.chain, replace(run.automatic_model, tool_transform=tool))
    assert not report.passed
    assert report.to_dict()["diagnostic"]["first_frame"] is None
    assert "alignment" in report.markdown()


def test_sampling_limits(run):
    cfg = EditorConfig(validation=ValidationConfig(samples=21, random_seed=17))
    samples = sample_configurations(run.chain, cfg)
    assert len(samples) == 21 and samples[0] == (0.,)*6
    joints = [j for j in run.chain.joints if j.limit is not None]
    assert all(j.limit.lower <= q[i] <= j.limit.upper for q in samples[1:] for i,j in enumerate(joints))


def test_orientation_clipping_and_pi():
    assert pose_errors(IDENTITY, IDENTITY) == (0., 0.)
    half = ((-1.,0.,0.,0.),(0.,-1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))
    assert pose_errors(IDENTITY, half)[1] == pytest.approx(pi)
    rounding = tuple(tuple(v*(1+1e-15) for v in row) for row in IDENTITY)
    assert pose_errors(IDENTITY, rounding)[1] == 0


def test_wrong_identity(run):
    with pytest.raises(ValueError):
        validate_global_fk(run.chain, replace(run.automatic_model, source_sha256='a'*64))


def test_mixed_fixed_continuous_prismatic():
    run = generate_automatic_model(Path(__file__).resolve().parents[1] / "robots/examples/mixed_joints.urdf")
    report = validate_global_fk(run.chain, run.automatic_model)
    assert report.passed
    functions = FKFunctions(run.chain, run.automatic_model)
    for sample in report.to_dict()["samples"]:
        np.testing.assert_allclose(np.array(functions.urdf(sample["q"])),
                                   reference_urdf_fk(run.source, sample["q"]), atol=1e-12)


def test_wrong_joint_sign_fails(run):
    rows = list(run.automatic_model.rows)
    rows[1] = replace(rows[1], joint_sign=-1)
    assert not validate_global_fk(run.chain, replace(run.automatic_model, rows=tuple(rows))).passed
