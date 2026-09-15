"""Stage 18 input atomicity, import boundaries, and archive compatibility."""

from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import pytest
from jsonschema import Draft202012Validator

from urdf2dt.parser.robot_document import RobotDocument
from urdf2dt.ui.desktop import DesktopEditor
from urdf2dt.export.schemas import SESSION_SCHEMA, decode_state

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "values",
    [[0.1], [0.1, 0.2, 0.3], [0.1, float("nan")], [0.1, float("inf")], [0.1, True]],
)
def test_pose_rejection_does_not_update_any_joint(values):
    calls = []
    ui = SimpleNamespace(
        joint_controls=[(None, None, -1.0, 1.0), (None, None, 0.0, 0.18)],
        change_joint=lambda *args: calls.append(args),
    )
    with pytest.raises(ValueError):
        DesktopEditor.set_pose(ui, values)
    assert not calls


def test_valid_pose_clamps_after_complete_validation():
    calls = []
    ui = SimpleNamespace(
        joint_controls=[(None, None, -1.0, 1.0), (None, None, 0.0, 0.18)],
        change_joint=lambda *args: calls.append(args),
    )
    DesktopEditor.set_pose(ui, [2.0, -0.4])
    assert calls == [(0, 1.0, False), (1, 0.0, False)]


@pytest.mark.parametrize("index", [-1, True, 100])
def test_document_rejects_ambiguous_chain_indices(index):
    doc = RobotDocument.load(ROOT / "robots/scara/scara_rrpr.urdf")
    with pytest.raises(ValueError, match="zero-based"):
        doc.select(index)


def test_parser_does_not_import_ui_and_old_import_remains_available():
    code = "import sys; from urdf2dt.parser.robot_document import RobotDocument; assert 'urdf2dt.ui' not in sys.modules; assert 'PySide6' not in sys.modules; from urdf2dt.ui.robot_document import RobotDocument as Old; assert Old is RobotDocument"
    subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT)


def test_existing_stage15_archive_schema_and_domain_round_trip():
    data = json.loads(
        (ROOT / "outputs/generalization/stage15_scara/session/session.json").read_text(
            encoding="utf-8"
        )
    )
    Draft202012Validator(SESSION_SCHEMA).validate(data)
    assert data["schema_version"] == "1.0" and data["dh_convention"] == "standard"
    assert json.loads(json.dumps(asdict(decode_state(data["state"])))) == data["state"]


@pytest.mark.parametrize("row", [True, "invalid", {}])
def test_invalid_proposal_type_fails_without_mutating_session(row):
    from urdf2dt.app import Application

    app = Application(ROOT / "robots/scara/scara_rrpr.urdf")
    before = app.session.state
    with pytest.raises(TypeError, match="DHRow or FrameEdit"):
        app.session.propose_edit(1, row)
    assert app.session.state is before and app.session.pending is None
    assert not app.session.events
