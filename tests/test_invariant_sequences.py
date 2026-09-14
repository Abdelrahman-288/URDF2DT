"""Seeded multi-edit regressions across revolute and mixed-joint robots."""

from dataclasses import asdict, replace
import json
from pathlib import Path
from random import Random

import numpy as np
import pytest

from urdf2dt.app import Application
from urdf2dt.config import EditorConfig, ValidationConfig
from urdf2dt.dh.classification import classify_dh_model, get_editable_params
from urdf2dt.dh.global_validation import sample_configurations
from urdf2dt.dh.recompute import FrameEdit, validate_frame
from urdf2dt.kinematics import dh_fk, dh_frame_transforms, urdf_fk

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ["robots/ur5/ur5_serial.urdf", "robots/scara/scara_rrpr.urdf"]


def confirm_all(session):
    for i in range(1, len(session.state.frames) + 1):
        if session.state.frames[i - 1].value not in ("editable", "accepted"):
            assert session.unlock_next() == i
        session.propose_edit(i, FrameEdit())
        assert session.accept().valid


def check_geometry(app, poses):
    model = app.session.state.working_model
    zero = (0.0,) * len(model.rows)
    current = dh_frame_transforms(model, zero)
    original = dh_frame_transforms(app.run.automatic_model, zero)
    for i in range(1, len(current)):
        assert validate_frame(current[i - 1], original[i], current[i]).valid
    for q in poses:
        frames = dh_frame_transforms(model, q)
        for transform in (*frames, model.base_transform, model.tool_transform):
            rotation = np.array(transform)[:3, :3]
            np.testing.assert_allclose(
                rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0
            )
            assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-12)
        np.testing.assert_allclose(
            dh_fk(model, q), urdf_fk(app.run.chain, q), atol=1e-10, rtol=0
        )


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("seed", [7, 42, 113])
def test_seeded_edit_reject_restore_and_archive_invariants(robot, seed, tmp_path):
    config = EditorConfig(validation=ValidationConfig(samples=10, random_seed=seed))
    app = Application(ROOT / robot, config)
    session = app.session
    baseline = session.state.automatic_model
    baseline_bytes = json.dumps(asdict(baseline), sort_keys=True)
    rng = Random(seed)
    poses = sample_configurations(app.run.chain, config)
    zero = (0.0,) * len(baseline.rows)
    confirm_all(session)
    for step in range(12):
        options = [
            (i, get_editable_params(c))
            for i, c in enumerate(classify_dh_model(session.state.working_model), 1)
            if get_editable_params(c)
        ]
        index, parameters = rng.choice(options)
        edit = FrameEdit(**{p.name: rng.uniform(-0.2, 0.2) for p in parameters})
        before = session.state.working_model
        frames_before = dh_frame_transforms(before, zero)
        states_before = session.state.frames
        session.propose_edit(index, edit)
        assert session.state.working_model is before
        if step % 4 == 0:
            session.reject("Seeded rejection")
            assert session.state.working_model is before
            assert session.state.frames == states_before
        else:
            assert session.accept().valid
            after = session.state.working_model
            frames_after = dh_frame_transforms(after, zero)
            assert after.rows[: index - 1] == before.rows[: index - 1]
            assert all(f.value == "accepted" for f in session.state.frames[:index])
            assert all(f.value == "invalidated" for f in session.state.frames[index:])
            assert validate_frame(
                frames_before[index - 1], frames_before[index], frames_after[index]
            ).valid
            for i, (a, b) in enumerate(zip(frames_before, frames_after)):
                if i != index:
                    np.testing.assert_allclose(a, b, atol=1e-12, rtol=0)
            if step % 3 == 0:
                session.restore_frame(index)
                restored = dh_frame_transforms(session.state.working_model, zero)[index]
                original = dh_frame_transforms(baseline, zero)[index]
                np.testing.assert_allclose(restored, original, atol=1e-12, rtol=0)
            confirm_all(session)
        assert session.state.automatic_model is baseline
        assert json.dumps(asdict(baseline), sort_keys=True) == baseline_bytes
        check_geometry(app, poses)
    assert app.validate().passed
    resumed = Application.resume(app.export(tmp_path / "archive"))
    assert resumed.session.state == session.state
    assert resumed.session.pending is None
    check_geometry(resumed, poses)
    session.restore_automatic()
    assert session.state.working_model is baseline
    assert session.state.frames[0].value == "editable"
    assert all(f.value == "locked" for f in session.state.frames[1:])
    assert session.pending is None


@pytest.mark.parametrize("robot", ROBOTS)
def test_sampling_repeats_and_respects_mixed_joint_limits(robot):
    app = Application(ROOT / robot)
    config = app.run.config
    a = sample_configurations(app.run.chain, config)
    assert a == sample_configurations(app.run.chain, config)
    changed = replace(config, validation=replace(config.validation, random_seed=43))
    b = sample_configurations(app.run.chain, changed)
    assert a[0] == b[0] and a[1:] != b[1:]
    joints = [j for j in app.run.chain.joints if j.joint_type.value != "fixed"]
    for q in a[1:]:
        for joint, value in zip(joints, q):
            if joint.limit:
                assert joint.limit.lower <= value <= joint.limit.upper
