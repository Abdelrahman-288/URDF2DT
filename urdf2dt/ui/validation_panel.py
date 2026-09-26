"""Sampled FK evidence dashboard; presentation never changes validation precision."""
from pathlib import Path
from typing import Any

from urdf2dt.dh.types import FrameState
from urdf2dt.ui.number_display import decimal, residual


class ValidationPanel:
    def __init__(self, e):
        self.e = e
        self.report = None
        self.owner: Any = None
        self.model = None
        self.configuration = None
        self.stale = False
        qt = e.qt
        self.page = qt.QWidget()
        layout = qt.QVBoxLayout(self.page)
        actions = qt.QHBoxLayout()
        layout.addLayout(actions)
        e._button(actions, 'Run global FK validation', e.validate)
        self.export_button = e._button(actions, 'Export evidence JSON…', self.export)
        self.export_button.setEnabled(False)
        layout.addWidget(e.validation_label)
        self.pages = qt.QTabWidget()
        layout.addWidget(self.pages)
        self.summary = self.make_table(['Metric', 'Maximum error', 'Tolerance (≤)', 'Result', 'Worst pose index'])
        self.pages.addTab(self.summary, 'Summary')
        self.poses = self.make_table(['Pose index', 'Position error [m]', 'Orientation error [rad]', 'Result', 'Joint coordinates q [rad / m]'])
        self.pages.addTab(self.poses, 'Tested poses')
        self.scope = qt.QPlainTextEdit()
        self.scope.setReadOnly(True)
        self.pages.addTab(self.scope, 'What was tested / limits')
        self.clear('NOT RUN — accept all DH frames, then run validation.')

    def make_table(self, columns):
        qt = self.e.qt
        widget = qt.QTableWidget(0, len(columns))
        widget.setHorizontalHeaderLabels(columns)
        widget.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
        widget.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        widget.horizontalHeader().setSectionResizeMode(qt.QHeaderView.ResizeToContents)
        widget.horizontalHeader().setStretchLastSection(True)
        widget.verticalHeader().hide()
        return widget

    def rows(self, widget, rows):
        widget.setRowCount(len(rows))
        for r, values in enumerate(rows):
            for c, value in enumerate(values):
                text = residual(value) if isinstance(value, float) else str(value)
                item = self.e.qt.QTableWidgetItem(text)
                item.setToolTip(repr(value) if isinstance(value, float) else str(value))
                widget.setItem(r, c, item)

    def clear(self, message):
        self.report = None
        self.owner = None
        self.stale = False
        self.summary.setRowCount(0)
        self.poses.setRowCount(0)
        self.scope.setPlainText('No completed test. A missing or blocked result is not a PASS.')
        self.export_button.setEnabled(False)
        self.e.validation_label.setText(message)

    def show_report(self, report):
        e = self.e
        self.report = report
        self.owner = e.application
        self.model = e.application.session.state.working_model
        self.configuration = e.application.run.config.snapshot()
        self.stale = False
        data = report.to_dict()
        config = data['configuration']['validation']
        summary = data['summary']
        pt, ot = config['position_tolerance_m'], config['orientation_tolerance_rad']
        self.rows(self.summary, [
            ['Position [m]', summary['max_position_error_m'], pt,
             'PASS' if summary['max_position_error_m'] <= pt else 'FAIL', summary['worst_position_sample']],
            ['Orientation [rad]', summary['max_orientation_error_rad'], ot,
             'PASS' if summary['max_orientation_error_rad'] <= ot else 'FAIL', summary['worst_orientation_sample']],
        ])
        self.rows(self.poses, [[s['index'], s['position_error_m'], s['orientation_error_rad'],
                               'PASS' if s['position_error_m'] <= pt and s['orientation_error_rad'] <= ot else 'FAIL',
                               ', '.join(decimal(q) for q in s['q'])] for s in data['samples']])
        for i, sample in enumerate(data['samples']):
            self.poses.item(i, 4).setToolTip(repr(sample['q']))
        failed = sum(s['position_error_m'] > pt or s['orientation_error_rad'] > ot for s in data['samples'])
        e.validation_label.setText(
            f"Sampled FK {'PASS' if report.passed else 'FAIL'} — {len(data['samples'])} tested poses; "
            f"{failed} failed. Seed: {config['random_seed']}. Tolerances: {data['configuration']['reference_status']}.")
        chain = e.application.run.chain
        self.scope.setPlainText(
            f'URDF FK versus committed DH FK, including base/tool transforms.\n'
            f'Reference: {chain.base_link}. Target point: {chain.tip_link} origin. Active serial chain only.\n'
            f'Joint order: {", ".join(data["joint_names"])}. Revolute/continuous: rad; prismatic: m.\n\n'
            'Position error: Euclidean distance between target origins. Orientation error: relative rotation angle (rad), '
            'using a clamped trace formula. Each tested pose must meet BOTH tolerances, including equality.\n\n'
            f'Sampling: zero pose plus {len(data["samples"]) - 1} seeded random poses within bounded joint limits; '
            'continuous joints use [-pi, pi]. Pose indices start at 0. Duplicate poses may occur.\n'
            f'Zero pose within joint limits: {data["zero_within_limits"]}. Zero is tested even when outside limits. '
            'These are sampled configurations, not a collision-checked trajectory.\n\n'
            f'Tolerance provenance: {data["configuration"]["reference_note"]}\n\n'
            'The automatic baseline must pass before the committed model is tested. All frames must be accepted; '
            'pending previews are excluded. Moving the live sliders does not change this fixed sample set.\n\n'
            f'Diagnostic: {data["diagnostic"]["message"]}\n'
            'Prefix diagnostics align frames at zero; they guide inspection and do not prove which edit caused an error.\n\n'
            'A PASS supports agreement at the recorded poses under the recorded tolerances. It does not prove agreement '
            'over all configurations, correctness of the source URDF, symbolic FK/Jacobian correctness, mesh alignment, '
            'collision freedom, dynamics, physical calibration, hardware accuracy or safe robot operation. '
            'Floating-point roundoff affects very small errors.\n\n'
            'Display values are formatted only; hover for full precision. Export contains original SI values, '
            'all tested coordinates, prefix errors, configuration and model/source hashes.\n'
            f'Candidate SHA256: {data["candidate_hash"]}\nSource SHA256: {data["source_sha256"]}'
        )
        self.export_button.setEnabled(True)
        self.pages.setCurrentIndex(0)

    def sync(self):
        if self.report is None:
            return
        app = self.e.application
        if app is None or app is not self.owner:
            self.clear('NOT RUN — robot or active chain changed. Run validation again.')
            return
        if (app.session.state.working_model != self.model or app.session.pending is not None
                or app.run.config.snapshot() != self.configuration
                or any(f != FrameState.ACCEPTED for f in app.session.state.frames)):
            self.stale = True
        if self.stale:
            self.e.validation_label.setText('STALE — historical results below; model, preview or acceptance changed. Run validation again.')
            self.export_button.setEnabled(False)

    def export(self):
        self.sync()
        if self.report is None or self.stale:
            return
        path, _ = self.e.qt.QFileDialog.getSaveFileName(self.e.window, 'Export current sampled FK evidence',
                                                     'validation_report.json', 'JSON (*.json)')
        if path:
            Path(path).write_text(self.report.json_text, encoding='utf-8')
