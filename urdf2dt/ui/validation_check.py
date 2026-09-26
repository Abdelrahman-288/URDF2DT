"""Real Qt dashboard checks, reusable in source and packaged diagnostics."""
from dataclasses import replace
import json
from unittest.mock import patch

from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.dh.recompute import FrameEdit


def verify_validation(e, app, folder):
    panel = e.workspace.validation
    e.validate()
    assert 'NOT COMPLETED' in e.validation_label.text()
    assert panel.report is None and panel.summary.rowCount() == 0
    session = e.application.session
    for i in range(1, len(session.state.frames) + 1):
        session.unlock_next()
        session.propose_edit(i, FrameEdit())
        session.accept()
    e.validate()
    report = panel.report
    assert report is not None and report.passed
    data = report.to_dict()
    assert panel.poses.rowCount() == len(data['samples'])
    assert panel.summary.item(0, 3).text() == 'PASS'
    assert float(panel.summary.item(0, 1).toolTip()) == data['summary']['max_position_error_m']
    assert 'hardware accuracy' in panel.scope.toPlainText()
    assert 'continuous joints' in panel.scope.toPlainText()
    path = folder / 'validation-dashboard.json'
    with patch.object(e.qt.QFileDialog, 'getSaveFileName', return_value=(str(path), 'JSON')):
        panel.export()
    assert json.loads(path.read_text(encoding='utf-8')) == data
    e.set_pose([.1] * len(e.q))
    panel.sync()
    assert not panel.stale
    e.workspace.vertical.setSizes([340, 420])
    app.processEvents()
    e.window.grab().save(str(folder / 'validation-summary.png'))
    panel.pages.setCurrentIndex(1)
    app.processEvents()
    e.window.grab().save(str(folder / 'validation-poses.png'))
    model = session.state.working_model
    rows = list(model.rows)
    rows[0] = replace(rows[0], a=rows[0].a + .1)
    failed = validate_global_fk(e.application.run.chain, replace(model, rows=tuple(rows)), e.application.run.config)
    assert not failed.passed
    panel.show_report(failed)
    assert panel.summary.item(0, 3).text() == 'FAIL'
    assert any(panel.poses.item(i, 3).text() == 'FAIL' for i in range(panel.poses.rowCount()))
    panel.show_report(report)
    session.restore_automatic()
    panel.sync()
    assert panel.stale and 'STALE' in e.validation_label.text()
    assert not panel.export_button.isEnabled()
    # A stale result must not export even if the export method is called directly.
    with patch.object(e.qt.QFileDialog, 'getSaveFileName', side_effect=AssertionError('stale export')):
        panel.export()
    from urdf2dt.runtime import asset_path
    e.load(str(asset_path('robots/ur5/ur5_serial.urdf')))
    panel.sync()
    assert panel.report is None and panel.poses.rowCount() == 0
    assert 'NOT RUN' in e.validation_label.text()
