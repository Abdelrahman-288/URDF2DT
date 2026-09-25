"""Live geometric Jacobian tables and velocity analysis, with SI exports."""
import json
from pathlib import Path
from urdf2dt.jacobian_analysis import analyze_jacobian
from urdf2dt.ui.number_display import decimal, spin_box


def table(editor, rows, columns):
    widget = editor.qt.QTableWidget(rows, columns)
    widget.setEditTriggers(editor.qt.QAbstractItemView.NoEditTriggers)
    widget.horizontalHeader().setSectionResizeMode(editor.qt.QHeaderView.Stretch)
    widget.horizontalHeader().setMinimumSectionSize(65)
    widget.horizontalHeader().setStretchLastSection(False)
    widget.setMinimumHeight(100)
    widget.verticalHeader().setDefaultSectionSize(24)
    widget.setStyleSheet("QTableWidget { gridline-color: #63758b; } QTableWidget::item { padding: 2px; }")
    return widget


def fill(editor, widget, values, labels, columns, digits=3):
    widget.setRowCount(len(values))
    widget.setColumnCount(len(columns))
    widget.setHorizontalHeaderLabels(columns)
    widget.setVerticalHeaderLabels(labels)
    for r, row in enumerate(values):
        for c, value in enumerate(row):
            item = editor.qt.QTableWidgetItem(decimal(float(value), digits))
            item.setTextAlignment(editor.core.Qt.AlignRight | editor.core.Qt.AlignVCenter)
            item.setToolTip(repr(float(value)))
            widget.setItem(r, c, item)


class AnalysisPanel:
    def __init__(self, e):
        self.e, self.result, self.rates, self.key = e, {}, [], None
        qt = e.qt
        page = qt.QWidget()
        self.page = page
        layout = qt.QVBoxLayout(page)
        self.expand_button = e._button(layout, "Expand analysis…", self.open_wide)
        self.context = qt.QLabel()
        self.context.setWordWrap(True)
        layout.addWidget(self.context)
        e._button(layout, 'Choose physical frames…', self.choose_frames)
        self.model = qt.QComboBox()
        self.model.addItems(['URDF', 'Automatic DH', 'Committed DH', 'Pending DH preview'])
        layout.addWidget(self.model)
        self.tables = []
        for title, labels in [('Geometric Jacobian J (6 × n)', ['vx','vy','vz','ωx','ωy','ωz']),
                              ('Linear Jacobian Jv (3 × n)', ['vx','vy','vz']),
                              ('Angular Jacobian Jω (3 × n)', ['ωx','ωy','ωz'])]:
            layout.addWidget(qt.QLabel(title))
            widget = table(e, len(labels), 0)
            widget.setMinimumHeight(220 if len(labels) == 6 else 135)
            layout.addWidget(widget)
            self.tables.append((widget, labels))
        layout.addWidget(qt.QLabel('Joint rates — rad/s or m/s; these do not change the pose'))
        self.rate_widget = qt.QWidget()
        self.rate_form = qt.QFormLayout(self.rate_widget)
        layout.addWidget(self.rate_widget)
        layout.addWidget(qt.QLabel('Relative target twist, expressed in reference'))
        self.twist = table(e, 6, 1)
        self.twist.setMinimumHeight(220)
        layout.addWidget(self.twist)
        self.length = spin_box(qt)
        self.length.setRange(.001, 1e6)
        self.length.setValue(1)
        self.length.setSuffix(' m')
        layout.addWidget(qt.QLabel('Characteristic length L for SVD normalization'))
        layout.addWidget(self.length)
        self.diagnostics = qt.QLabel()
        self.diagnostics.setWordWrap(True)
        layout.addWidget(self.diagnostics)
        e._button(layout, 'Export analysis JSON (full precision, SI)…', self.export)
        scroll = qt.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.scroll = scroll
        self.index = e.tabs.addTab(scroll, 'Jacobian analysis')
        self.model.currentIndexChanged.connect(e.fk_panel.model.setCurrentIndex)
        e.fk_panel.model.currentIndexChanged.connect(self.model.setCurrentIndex)
        self.model.currentIndexChanged.connect(lambda _: self.refresh())
        self.length.valueChanged.connect(lambda _: self.refresh())

    def refresh(self):
        e = self.e
        if not e.application or getattr(e, '_loading_robot', False):
            return
        app, fk = e.application, e.fk_panel
        if not fk.reference.currentText() or not fk.target.currentText():
            return
        key = (id(app), app.run.chain.joint_names)
        if key != self.key:
            e._clear(self.rate_form)
            self.rates = []
            for joint in app.run.chain.joints:
                if joint.joint_type.value == 'fixed':
                    continue
                spin = spin_box(e.qt)
                spin.setRange(-1e6, 1e6)
                spin.setSingleStep(.01)
                self.rate_form.addRow(joint.name + (' [m/s]' if joint.joint_type.value == 'prismatic' else ' [rad/s]'), spin)
                spin.valueChanged.connect(lambda _: self.refresh())
                self.rates.append(spin)
            self.key = key
        mode = self.model.currentIndex()
        if mode == 3 and app.session.pending is None:
            self.context.setText('No pending DH preview. Choose a committed model.')
            self.result = {}
            for widget, _ in self.tables:
                widget.setRowCount(0)
            self.twist.setRowCount(0)
            self.diagnostics.clear()
            return
        model = [None, app.session.state.automatic_model, app.session.state.working_model, e.current_model()][mode]
        self.result = analyze_jacobian(app.run.chain, e.q, fk.reference.currentText(), fk.target.currentText(),
            [s.value() for s in self.rates], model, fk.frames, self.length.value())
        self.result['model'] = self.model.currentText()
        self.result['source_file'] = str(app.run.chain.source_urdf or '')
        self.result['pending_preview'] = mode == 3
        names = self.result['joint_names']
        digits = getattr(getattr(e, 'workspace', None), 'digits', 3)
        for (widget, labels), result_key in zip(self.tables, ('matrix','linear','angular')):
            fill(e, widget, self.result[result_key], labels, [f"J{i+1}" for i in range(len(names))], digits)
            for i, name in enumerate(names):
                widget.horizontalHeaderItem(i).setToolTip(name)
        fill(e, self.twist, [[v] for v in self.result['twist']], ['vx [m/s]','vy [m/s]','vz [m/s]','ωx [rad/s]','ωy [rad/s]','ωz [rad/s]'], ['Velocity'], digits)
        from urdf2dt.ui.result_help import jacobian_help
        self.context.setText(jacobian_help(self.result['reference'], self.result['target'], app.run.chain))
        result = self.result
        condition = '∞ / rank deficient' if result['condition_number'] is None else f"{result['condition_number']:.3g}"
        self.diagnostics.setText(f"Normalized rank: {result['rank']} / {result['maximum_rank']} (full spatial task requires 6).\n"
            f"Condition: {condition}\nSingular values: " + ', '.join(f'{v:.3e}' for v in result['singular_values']) +
            '\nNormalization: linear rows / L; prismatic columns × L. Relative rank threshold: 1e-9. '
            'This local diagnostic is not a global FK validation result.')

    def export(self):
        self.refresh()
        if not self.result:
            raise ValueError('Choose an available model first')
        path, _ = self.e.qt.QFileDialog.getSaveFileName(self.e.window, 'Export Jacobian analysis', 'jacobian-analysis.json', 'JSON (*.json)')
        if path:
            Path(path).write_text(json.dumps(self.result, indent=2, allow_nan=False), encoding='utf-8')

    def choose_frames(self):
        dialog = getattr(self, '_wide_dialog', None)
        if dialog is not None:
            dialog.accept()
        self.e.tabs.setCurrentIndex(self.e.fk_panel.index)

    def open_wide(self):
        qt = self.e.qt
        dialog = qt.QDialog(self.e.window)
        dialog.setWindowTitle('Jacobian analysis - SI source coordinates')
        dialog.resize(850, 850)
        self._wide_dialog = dialog
        self.expand_button.hide()
        layout = qt.QVBoxLayout(dialog)
        expanded = qt.QScrollArea()
        expanded.setWidgetResizable(True)
        self.scroll.takeWidget()
        expanded.setWidget(self.page)
        layout.addWidget(expanded)
        try:
            dialog.exec()
        finally:
            expanded.takeWidget()
            self.scroll.setWidget(self.page)
            self._wide_dialog = None
            self.expand_button.show()
