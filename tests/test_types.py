"""Data invariants and deep immutability, independent of UI or solver code."""

from dataclasses import FrozenInstanceError, replace
from math import pi

import numpy as np
import pytest

from urdf2dt.config import EditorConfig, ValidationConfig
from urdf2dt.dh.types import (
    DHModel, DHRow, EditRecord, EditorState, FrameState, Joint, JointLimit,
    JointType, KinematicChain, ValidationResult, ValidationSample,
)
from tests.fixtures.ur5_automatic_dh import make_ur5_fixture


def row():
    return DHRow(0.2, 0., 0.1, 0., "joint")


def test_model_copies_mutable_inputs_and_working_edits_preserve_baseline():
    rows = [row()]
    matrix = np.eye(4)
    model = DHModel("test", rows, "synthetic test", base_transform=matrix)
    rows.clear()
    matrix[0, 3] = 20
    assert len(model.rows) == 1
    assert model.base_transform[0][3] == 0
    working = replace(model, rows=(replace(model.rows[0], a=0.3),))
    assert model.rows[0].a == 0.2
    assert working.rows[0].a == 0.3
    with pytest.raises(FrozenInstanceError):
        model.rows[0].a = 1
    with pytest.raises(TypeError):
        model.base_transform[0][3] = 1
    assert isinstance(hash(model), int)


@pytest.mark.parametrize("matrix", [
    np.eye(3), np.diag([-1., 1., 1., 1.]), np.diag([2., 1., 1., 1.]),
    np.diag([1., 1., 1., 0.]), np.full((4, 4), np.nan),
])
def test_invalid_transforms(matrix):
    with pytest.raises(ValueError):
        DHModel("test", (row(),), "synthetic", tool_transform=matrix)


@pytest.mark.parametrize("kwargs", [
    {"a": float("nan")}, {"alpha": float("inf")}, {"d": True},
    {"theta_offset": "1"}, {"joint_name": ""}, {"joint_sign": 0},
    {"joint_sign": True}, {"joint_type": "fixed"}, {"joint_type": "floating"},
])
def test_bad_dh_rows(kwargs):
    with pytest.raises(ValueError):
        replace(row(), **kwargs)


def test_model_requires_unique_rows_and_boolean_fixture_flag():
    for rows in ((), (row(), row()), ({"a": 1},)):
        with pytest.raises(ValueError):
            DHModel("test", rows, "test")
    with pytest.raises(ValueError):
        DHModel("test", (row(),), "test", is_temporary_fixture="yes")


def test_chain_retains_fixed_joints_and_q_order():
    matrix = np.eye(4)
    axis = [0., 0., 1.]
    joints = [
        Joint("mount", "base", "mount_link", JointType.FIXED, origin=matrix),
        Joint("spin", "mount_link", "arm", JointType.CONTINUOUS, axis=axis),
        Joint("slide", "arm", "tip", JointType.PRISMATIC, limit=JointLimit(0, 1)),
    ]
    chain = KinematicChain("test", "base", "tip", joints, "synthetic.urdf")
    matrix[0, 3], axis[2] = 9, 0
    joints.clear()
    assert chain.joint_names == ("spin", "slide")
    assert chain.joints[0].origin[0][3] == 0
    assert chain.joints[1].axis == (0., 0., 1.)


def test_chain_rejects_cycle_gap_duplicate_and_wrong_tip():
    first = Joint("a", "base", "link", JointType.FIXED)
    for second in (
        Joint("b", "other", "tip", JointType.FIXED),
        Joint("b", "link", "base", JointType.FIXED),
        Joint("a", "link", "tip", JointType.FIXED),
    ):
        with pytest.raises(ValueError):
            KinematicChain("test", "base", "tip", (first, second), "test.urdf")
    with pytest.raises(ValueError):
        KinematicChain("test", "base", "tip", (first,), "test.urdf")


def test_joint_contract():
    with pytest.raises(ValueError):
        JointLimit(1, -1)
    with pytest.raises(ValueError):
        Joint("j", "a", "b", JointType.REVOLUTE)
    with pytest.raises(ValueError):
        Joint("j", "a", "b", JointType.CONTINUOUS, limit=JointLimit(-1, 1))
    with pytest.raises(ValueError):
        Joint("j", "a", "b", JointType.FIXED, axis=(0, 0, 2))


def test_editor_snapshot_and_history_are_immutable():
    baseline = make_ur5_fixture()
    before = baseline.rows[0]
    edit = EditRecord(1, 1, before, replace(before, d=0.1), False, "test rejection")
    history = [edit]
    frames = [FrameState.EDITABLE] + [FrameState.LOCKED] * 5
    state = EditorState(baseline, baseline, frames, history)
    history.clear()
    frames[0] = FrameState.ACCEPTED
    assert state.frames[0] == FrameState.EDITABLE
    assert state.history == (edit,)
    assert state.automatic_model == baseline
    with pytest.raises(ValueError):
        replace(state, frames=())
    with pytest.raises(ValueError):
        replace(state, history=(replace(edit, sequence=2),))
    with pytest.raises(ValueError):
        replace(state, history=(replace(edit, frame_index=7),))
    with pytest.raises(ValueError):
        replace(edit, proposed=replace(before, joint_name="other"))


def test_validation_uses_recorded_tolerances_and_copies_q():
    q = np.array([0.])
    sample = ValidationSample(q, 1e-4, 1e-4)
    config = EditorConfig(validation=ValidationConfig(samples=1))
    result = ValidationResult(["joint"], [sample], config)
    q[0] = 10
    assert result.samples[0].q == (0.,)
    assert result.passed
    assert result.max_position_error_m == 1e-4
    assert not replace(result, samples=(replace(sample, orientation_error_rad=2e-4),)).passed
    with pytest.raises(ValueError):
        replace(result, samples=())
    with pytest.raises(ValueError):
        replace(result, joint_names=("a", "b"))
    with pytest.raises(ValueError):
        replace(sample, position_error_m=-1)
    with pytest.raises(ValueError):
        replace(sample, q=(float("nan"),))


def test_fixture_is_explicitly_temporary_and_matches_nominal_table():
    model = make_ur5_fixture()
    assert model.is_temporary_fixture and model.source_urdf is None
    assert "unverified" in model.provenance
    assert [r.a for r in model.rows] == [0, -0.425, -0.39225, 0, 0, 0]
    assert [r.d for r in model.rows] == [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
    assert [r.alpha for r in model.rows] == [pi/2, 0, 0, pi/2, -pi/2, 0]
