from dataclasses import replace
import json
from pathlib import Path

import pytest

from urdf2dt.app import Application, main
from urdf2dt.dh.editor_session import EditDecision, EditorSession
from urdf2dt.dh.recompute import FrameEdit

SOURCE = Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf"


def test_full_edited_workflow(tmp_path):
    app = Application(SOURCE)
    for i in range(1, 7):
        app.session.unlock_next()
        app.session.propose_edit(i, FrameEdit(.05, .2 if i == 6 else 0.) if i in (2, 3, 6) else FrameEdit())
        app.session.accept()
    assert app.validate().passed
    path = app.export(tmp_path / "complete")
    restored = Application.resume(path)
    assert restored.session.state == app.session.state
    assert restored.validate().passed


def test_failed_validation_blocks_export(tmp_path):
    app = Application(SOURCE)
    # Explicitly faulty injected validator tests the application-level FK gate.
    app.session = EditorSession(app.run.automatic_model, validator=lambda s,p: EditDecision(True, "test fault"))
    for i in range(1,7):
        app.session.unlock_next()
        row = app.session.state.working_model.rows[i-1]
        app.session.propose_edit(i, replace(row, a=row.a+.1) if i==3 else row)
        app.session.accept()
    assert not app.validate().passed
    with pytest.raises(ValueError, match="Export requires passing"):
        app.export(tmp_path / "failed")
    assert not (tmp_path / "failed").exists()


def test_cli_end_to_end(tmp_path):
    output = tmp_path / "cli"
    assert main([str(SOURCE), "--output", str(output), "--edit", "2:0.02:0", "--edit", "6:0.01:0.1"]) == 0
    saved = json.loads((output / "session.json").read_text())
    assert saved["validation"]["passed"]
    assert saved["state"]["frames"] == ["accepted"] * 6


@pytest.mark.parametrize("edits", [["--edit", "1:0.1:0"], ["--edit", "7:0:0"],
                                   ["--edit", "2:0:0", "--edit", "2:0.1:0"]])
def test_cli_invalid_edits_no_export(tmp_path, edits):
    assert main([str(SOURCE), "--output", str(tmp_path / "invalid"), *edits]) == 1
    assert not (tmp_path / "invalid").exists()


def test_incomplete_and_pending_gates():
    app = Application(SOURCE)
    with pytest.raises(ValueError): app.validate()
    app.session.propose_edit(1, FrameEdit())
    with pytest.raises(ValueError): app.validate()
