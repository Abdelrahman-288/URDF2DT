"""Independent closed-form SCARA oracle and mixed-joint editing regressions."""

from pathlib import Path

import pytest

from urdf2dt.app import Application
from urdf2dt.config import EditorConfig, ValidationConfig
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.dh.global_validation import sample_configurations
from urdf2dt.kinematics import urdf_fk, dh_fk

SOURCE = Path(__file__).resolve().parents[1] / "robots/scara/scara_rrpr.urdf"


from urdf2dt.research.scara_oracle import analytic_fk


def edited_application():
    app = Application(SOURCE, EditorConfig(validation=ValidationConfig(samples=200)))
    for i in range(1, 5):
        app.session.unlock_next()
        app.session.propose_edit(i, FrameEdit(0.025, 0.2 if i in (3, 4) else 0.0))
        assert app.session.accept().valid
    return app


def test_scara_matches_independent_oracle_and_round_trips(tmp_path):
    app = edited_application()
    for q in sample_configurations(app.run.chain, app.run.config) + (
        (0.0, 0.0, 0.18, 0.0),
        (1.0, -1.0, 0.09, 0.4),
    ):
        expected = analytic_fk(q)
        for actual in (
            urdf_fk(app.run.chain, q),
            dh_fk(app.run.automatic_model, q),
            dh_fk(app.session.state.working_model, q),
        ):
            assert (
                max(
                    abs(a - b)
                    for row, ref in zip(actual, expected)
                    for a, b in zip(row, ref)
                )
                < 1e-12
            )
    assert app.validate().passed
    restored = Application.resume(app.export(tmp_path / "scara"))
    assert restored.session.state == app.session.state
    assert restored.validate().passed


def test_scara_downward_stroke_and_non_six_joint_shape():
    app = Application(SOURCE)
    assert len(app.run.automatic_model.rows) == 4
    a = dh_fk(app.run.automatic_model, (0.0, 0.0, 0.0, 0.0))
    b = dh_fk(app.run.automatic_model, (0.0, 0.0, 0.18, 0.0))
    assert b[2][3] - a[2][3] == pytest.approx(-0.18)
    assert b[0][3] == pytest.approx(a[0][3])
    assert b[1][3] == pytest.approx(a[1][3])


def test_scara_restore_and_upstream_reedit_invalidates_downstream():
    app = edited_application()
    baseline = app.run.automatic_model
    app.session.propose_edit(1, FrameEdit(0.04))
    assert app.session.accept().valid
    assert all(f.value == "invalidated" for f in app.session.state.frames[1:])
    with pytest.raises(ValueError, match="Accept all frames"):
        app.validate()
    app.session.restore_automatic()
    assert app.session.state.working_model == baseline
    assert app.session.state.automatic_model == baseline
