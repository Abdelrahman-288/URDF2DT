import json
from pathlib import Path

import pytest

from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.export.persistence import save_session, load_session


@pytest.fixture
def completed():
    run = generate_automatic_model(Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf")
    session = EditorSession(run.automatic_model)
    for i in range(1,7):
        session.unlock_next()
        session.propose_edit(i, FrameEdit(.03) if i in (2, 6) else FrameEdit())
        session.accept()
    return run, session


def test_round_trip_and_continue(completed, tmp_path):
    run, session = completed
    path = save_session(tmp_path / "export", run, session)
    loaded = load_session(path)
    assert loaded.session.state == session.state
    assert loaded.session.events == session.events
    assert loaded.session.geometric_frames == session.geometric_frames
    assert loaded.report.passed
    loaded.session.restore_frame(2)
    session.restore_frame(2)
    assert loaded.session.state == session.state
    assert (path.parent / "validation_report.md").exists()
    assert "reproducibility" in json.loads(path.read_text())


@pytest.mark.parametrize("mutation", ["version", "samples", "model", "extra", "baseline"])
def test_tampered_archive_rejected(completed, tmp_path, mutation):
    run, session = completed
    path = save_session(tmp_path / "export", run, session)
    data = json.loads(path.read_text())
    if mutation == "version": data["schema_version"] = "99"
    elif mutation == "samples": data["validation"]["samples"][0]["q"][0] = 1.
    elif mutation == "model": data["state"]["working_model"]["rows"][0]["a"] += 1.
    elif mutation == "extra": data["state"]["working_model"]["unknown"] = 1
    else: data["state"]["automatic_model"]["rows"][0]["a"] += 1.
    path.write_text(json.dumps(data))
    with pytest.raises(Exception):
        load_session(path)


def test_pending_incomplete_and_no_overwrite(completed, tmp_path):
    run, session = completed
    save_session(tmp_path / "export", run, session)
    with pytest.raises(FileExistsError): save_session(tmp_path / "export", run, session)
    session.propose_edit(1, FrameEdit())
    with pytest.raises(ValueError): save_session(tmp_path / "pending", run, session)
    session.restore_automatic()
    with pytest.raises(ValueError): save_session(tmp_path / "incomplete", run, session)


def test_duplicate_json_and_nonfinite(tmp_path):
    path = tmp_path / "bad.json"
    for value in ('{"a":1,"a":2}', '{"a":NaN}'):
        path.write_text(value)
        with pytest.raises(ValueError): load_session(path)


def test_logging(completed, tmp_path, caplog):
    caplog.set_level("INFO", logger="urdf2dt")
    run, session = completed
    path = save_session(tmp_path / "export", run, session)
    load_session(path)
    assert "Export completed" in caplog.text
    assert "Session reloaded" in caplog.text
