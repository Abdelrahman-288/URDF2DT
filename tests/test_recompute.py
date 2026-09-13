from dataclasses import replace
from pathlib import Path
from math import pi

import numpy as np
import pytest

from urdf2dt.dh.recompute import FrameEdit, FrameEditError, recompute_model, validate_frame
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.types import DHModel, DHRow, IDENTITY, JointType
from urdf2dt.kinematics import dh_fk, dh_frame_transforms
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.recompute import common_normal_points


@pytest.fixture
def model():
    return generate_automatic_model(Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf").automatic_model


@pytest.mark.parametrize("index", [2, 3, 6])
def test_ur5_sweep(model, index):
    rng = np.random.default_rng(93)
    for offset in np.linspace(-1., 1., 11):
        edited = recompute_model(model, index, FrameEdit(axial_translation=offset))
        for q in rng.uniform(-pi, pi, (10, 6)):
            np.testing.assert_allclose(dh_fk(edited, q), dh_fk(model, q), atol=1e-10)
        poses = dh_frame_transforms(edited, (0.,)*6)
        old = dh_frame_transforms(model, (0.,)*6)
        for j in range(7):
            if j != index:
                np.testing.assert_allclose(poses[j], old[j], atol=1e-10)


def test_terminal_rotation_and_tool(model):
    for angle in np.linspace(-pi, pi, 13):
        edited = recompute_model(model, 6, FrameEdit(.3, angle))
        np.testing.assert_allclose(dh_fk(edited, (.2,)*6), dh_fk(model, (.2,)*6), atol=1e-10)
    assert edited.tool_transform != model.tool_transform


@pytest.mark.parametrize("index,edit", [(1, FrameEdit(.1)), (4, FrameEdit(.1)), (2, FrameEdit(0, .2))])
def test_illegal_controls(model, index, edit):
    with pytest.raises(FrameEditError, match="edit_space"):
        recompute_model(model, index, edit)


def test_local_rules(model):
    poses = dh_frame_transforms(model, (0.,)*6)
    invalid = ((-1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.))
    assert validate_frame(poses[1], poses[2], invalid).issues[0].rule == "rigid_frame"
    assert not validate_frame(poses[1], poses[2], IDENTITY).valid
    assert validate_frame(poses[1], poses[2], poses[2]).valid


@pytest.mark.parametrize("kind,alpha,sign", [(JointType.PRISMATIC, 0., -1), (JointType.REVOLUTE, pi, 1)])
def test_prismatic_and_antiparallel(kind, alpha, sign):
    model = DHModel("test", (DHRow(.4, alpha, .1, .2, "a", kind, sign),
                             DHRow(.3, 0, .2, .1, "b", kind, sign)), "synthetic")
    edited = recompute_model(model, 1, FrameEdit(.27))
    for q in [(0., 0.), (.2, -.3), (-.8, 1.2)]:
        np.testing.assert_allclose(dh_fk(model, q), dh_fk(edited, q), atol=1e-10)


def test_session_atomic_compensation_restore(model):
    session = EditorSession(model)
    session.propose_edit(1, model.rows[0]); session.accept()
    session.propose_edit(2, FrameEdit(.25))
    assert session.state.working_model == model
    session.accept()
    assert session.state.working_model.rows[2] != model.rows[2]
    np.testing.assert_allclose(dh_fk(session.state.working_model, (.4,)*6), dh_fk(model, (.4,)*6), atol=1e-10)
    session.restore_frame(2)
    np.testing.assert_allclose(dh_fk(session.state.working_model, (.4,)*6), dh_fk(model, (.4,)*6), atol=1e-10)
    session.restore_automatic()
    assert session.state.working_model == model


def test_rejected_geometry_is_atomic(model):
    session = EditorSession(model)
    before = session.state
    with pytest.raises(FrameEditError):
        session.propose_edit(1, FrameEdit(.1))
    assert session.state is before and session.pending is None


def test_nonfinite_and_indices(model):
    with pytest.raises(ValueError):
        FrameEdit(float("nan"))
    with pytest.raises(ValueError):
        recompute_model(model, True, FrameEdit())


def test_common_normal_points():
    second = ((1., 0., 0., 2.), (0., 0., -1., 3.), (0., 1., 0., 4.), (0., 0., 0., 1.))
    p, q = common_normal_points(IDENTITY, second)
    np.testing.assert_allclose(p, (0, 0, 4))
    np.testing.assert_allclose(q, (2, 0, 4))
