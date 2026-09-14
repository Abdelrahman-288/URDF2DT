"""Optional widget callbacks exercise the real headless session."""
from pathlib import Path

import pytest

pytest.importorskip("ipywidgets")
from urdf2dt.ui.dh_editor import DHEditor
from urdf2dt.dh.types import FrameState as F

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def editor():
    ui = DHEditor(render_scene=False)
    ui.path.value = str(ROOT / "robots/ur5/ur5_serial.urdf")
    ui.load_button.click()
    assert ui.session is not None
    return ui


def test_guided_session(editor):
    for i in range(1, 7):
        editor.unlock_button.click()
        assert editor.frame.value == i
        editor.preview_button.click()
        assert not editor.accept_button.disabled
        editor.accept_button.click()
        assert editor.session.state.frames[i-1] == F.ACCEPTED
    assert editor.unlock_button.disabled
    assert "accepted" in editor.table.value


def test_preview_change_and_cascade(editor):
    for i in range(1, 4):
        editor.unlock_button.click(); editor.preview_button.click(); editor.accept_button.click()
    editor.frame.value = 2
    editor.controls["axial_translation"].value = .1
    before = editor.session.state.working_model
    editor.preview_button.click()
    assert editor.session.state.working_model == before
    assert "Pending preview" in editor.table.value
    editor.controls["axial_translation"].value = .2
    assert editor.session.pending is None and editor.accept_button.disabled
    editor.preview_button.click(); editor.accept_button.click()
    assert editor.session.state.frames[2:] == (F.INVALIDATED,) * 4
    assert "invalidated" in editor.frame.options[2][0]
    editor.restore_button.click()
    editor.reset_button.click()
    assert editor.session.state.working_model == editor.session.state.automatic_model


def test_locked_selection_and_cancel(editor):
    editor.frame.value = 6
    assert editor.preview_button.disabled and editor.restore_button.disabled
    assert all(c.disabled for c in editor.controls.values())
    editor.frame.value = 1
    editor.preview_button.click(); editor.reject_button.click()
    assert editor.session.pending is None
    assert editor.accept_button.disabled


def test_bad_load_preserves_session(editor):
    before = editor.session
    editor.path.value = str(ROOT / "robots/ur5/ur5_upstream.urdf")
    editor.load_button.click()
    assert editor.session is before
    assert "alert" in editor.status.value


def test_nonfinite_preview_cannot_accept(editor):
    editor.preview_button.click(); editor.accept_button.click(); editor.unlock_button.click()
    editor.controls["axial_translation"].value = float("inf")
    editor.preview_button.click()
    assert editor.accept_button.disabled and editor.session.pending is None
    assert "alert" in editor.status.value


def test_global_validation_requires_acceptance_and_expires(editor):
    assert editor.validate_button.disabled
    for i in range(1, 7):
        editor.unlock_button.click(); editor.preview_button.click(); editor.accept_button.click()
    assert not editor.validate_button.disabled
    editor.validate_button.click()
    assert editor.validation_report.passed
    assert "Sampled FK: PASS" in editor.validation_status.value
    editor.preview_button.click()
    assert editor.validation_report is None
    assert editor.validate_button.disabled
    editor.reject_button.click()
    editor.reset_button.click()
    assert editor.validation_report is None and editor.validate_button.disabled


def test_ui_save_and_reload(editor, tmp_path):
    for i in range(1, 7):
        editor.unlock_button.click(); editor.preview_button.click(); editor.accept_button.click()
    editor.archive_path.value = str(tmp_path / "saved")
    before = editor.session.state
    editor.save_button.click()
    assert (tmp_path / "saved/session.json").exists()
    editor.reset_button.click()
    editor.reload_button.click()
    assert editor.session.state == before
    assert editor.validation_report.passed
