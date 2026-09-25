"""On-demand symbolic FK with cancellable isolated computation."""

from multiprocessing import get_context
from queue import Empty
from pathlib import Path
from time import monotonic
import json
from urdf2dt.symbolic import generate_documents
from urdf2dt.jacobian import geometric_jacobian
from urdf2dt.ui.number_display import decimal, display
from urdf2dt.ui.matrix_view import MatrixView, fill_matrix


class SymbolicPanel:
    def __init__(self, e, kind="FK"):
        self.e = e
        self.kind = kind
        self.title = "Symbolic " + kind
        self.numeric_signature = None
        self.numeric_result = None
        self.process = None
        self.queue = None
        self.signature = None
        self.result = None
        self.cache = {}
        page = e.qt.QWidget()
        layout = e.qt.QVBoxLayout(page)
        self.scope = e.qt.QLabel("Open a robot to generate symbolic FK.")
        self.scope.setWordWrap(True)
        layout.addWidget(self.scope)
        self.model = e.qt.QComboBox()
        self.model.addItems(["Committed DH model", "Automatic DH model"])
        layout.addWidget(self.model)
        actions = e.qt.QHBoxLayout()
        layout.addLayout(actions)
        self.generate = e._button(actions, "Generate " + self.title.lower(), self.start)
        self.cancel = e._button(actions, "Cancel", self.stop)
        self.status = e.qt.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.output = e.qt.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setLineWrapMode(e.qt.QPlainTextEdit.NoWrap)
        font = self.output.font()
        font.setFamily("Consolas")
        self.output.setFont(font)
        self.output.setStyleSheet("QPlainTextEdit { font-family: Consolas; font-size: 12px; }")
        self.matrix_view = MatrixView(e, self.output)
        layout.addWidget(self.matrix_view.widget, 1)
        self.expand = e._button(layout, "Open full matrix view…", self.full_view)
        layout.removeWidget(self.expand)
        layout.insertWidget(layout.indexOf(self.matrix_view.widget), self.expand)
        copies = e.qt.QHBoxLayout()
        layout.addLayout(copies)
        self.copy = e._button(copies, "Copy symbolic text", lambda: self.copy_result(0))
        self.latex = e._button(copies, "Copy LaTeX", lambda: self.copy_result(1))
        self.export_button = e._button(layout, "Export " + self.title.lower() + "…", self.export)
        if kind == "Jacobian":
            numeric_page = e.qt.QWidget()
            numeric_layout = e.qt.QVBoxLayout(numeric_page)
            numeric_layout.setContentsMargins(0,0,0,0)
            self.numeric = e.qt.QPlainTextEdit(numeric_page)
            self.numeric.setReadOnly(True)
            self.numeric.setLineWrapMode(e.qt.QPlainTextEdit.NoWrap)
            self.numeric.setFont(font)
            self.numeric.setStyleSheet(self.output.styleSheet())
            self.numeric.setMinimumHeight(180)
            self.numeric.hide()  # retain exact text for accessibility/tests; display a labelled table
            self.numeric_label = e.qt.QLabel('Numerical Jacobian at current pose')
            self.numeric_label.setWordWrap(True)
            numeric_layout.addWidget(self.numeric_label)
            self.numeric_table = e.qt.QTableWidget()
            self.numeric_table.setMinimumHeight(200)
            numeric_layout.addWidget(self.numeric_table, 1)
            self.numeric_export = e._button(numeric_layout, "Export numerical Jacobian JSON…", self.export_numeric)
            self.matrix_view.pages.addWidget(numeric_page)
            self.matrix_view.mode.addItem("Numerical J(q)")
        scroll = e.qt.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.index = e.tabs.addTab(scroll, self.title)
        self.timer = e.core.QTimer(e.window)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        e.qt.QApplication.instance().aboutToQuit.connect(self.stop)
        self.model.currentIndexChanged.connect(self.sync)
        self.sync()

    def current(self):
        app = self.e.application
        if not app:
            return None
        chain = app.run.chain
        state = app.session.state
        model = state.automatic_model if self.model.currentIndex() else state.working_model
        return (chain.base_link, chain.tip_link, self.model.currentIndex(), model)

    def sync(self, *_):
        if getattr(self.e, "_loading_robot", False):
            return
        current = self.current()
        if current != self.signature:
            self.stop()
            self.signature = current
            self.result = self.cache.get(current)
            self.output.setPlainText(self.result[0] if self.result else "")
            self.matrix_view.set_data(self.result[2] if self.result else None)
            self.status.setText("Cached result ready." if self.result else "Generate on demand. Joint motion does not require regeneration.")
        self.scope.setText(
            f"{current[1]} relative to {current[0]}. Standard DH; q uses source joint coordinates.\n"
            "This tab covers the selected chain endpoints, independently of the numerical FK link selectors.\n"
            "Pending previews are excluded. Dimensions remain numeric constants."
            if current else "Open a robot to generate " + self.title.lower() + ".")
        if self.kind == "Jacobian" and current:
            self.scope.setText(
                f"Target: {current[1]} origin. Relative to / expressed in: {current[0]}.\n"
                "[v; ω] = J(q) q̇; rows vx, vy, vz, ωx, ωy, ωz.\n"
                "Source q: rad / m. Velocities: m/s, rad/s; not RPY rates. Pending previews excluded.")
        self.generate.setEnabled(current is not None and self.process is None)
        self.cancel.setEnabled(self.process is not None)
        for button in (self.copy, self.latex, self.export_button, self.expand):
            button.setEnabled(self.result is not None)
        if self.kind == "Jacobian":
            self.sync_numeric()

    def sync_numeric(self):
        signature = (self.signature, tuple(self.e.q)) if self.result else None
        self.numeric_export.setEnabled(signature is not None)
        if signature == self.numeric_signature:
            return
        self.numeric_signature = signature
        self.numeric_result = None
        self.numeric.clear()
        self.numeric_table.setRowCount(0)
        if signature is None:
            return
        current, q = signature
        assert current is not None
        model = current[3]
        matrix = geometric_jacobian(model, q)
        fill_matrix(self.e.qt, self.numeric_table, matrix,
                    ['vx','vy','vz','ωx','ωy','ωz'],
                    [f'q{i+1}' for i in range(len(q))], numeric=True)
        self.numeric_label.setText('Numerical J(q) • three decimals\nq [rad/m]: ' + display(list(q)))
        self.numeric_result = {
            "kind": "geometric_jacobian", "target_point": "tip_link_origin",
            "target_link": current[1], "reference_link": current[0],
            "expressed_in": current[0], "model": self.model.currentText(),
            "rows": ["vx", "vy", "vz", "wx", "wy", "wz"],
            "velocity_units": ["m/s"] * 3 + ["rad/s"] * 3,
            "joint_names": [row.joint_name for row in model.rows],
            "joint_coordinate_units": ["m" if row.joint_type.value == "prismatic" else "rad" for row in model.rows],
            "q_si": list(q), "matrix": matrix, "pending_preview_excluded": True,
        }
        self.numeric.setPlainText("Numerical J(q), live pose (three-decimal display):\n"
            + "\n".join("  ".join(decimal(v) for v in row) for row in matrix)
            + "\nq [rad/m]: " + display(list(q))
            + "\nColumns follow q1…qn above. JSON retains full precision.")

    def export_numeric(self):
        self.sync()
        if self.numeric_result is None:
            return
        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Export numerical Jacobian", "jacobian.json", "JSON (*.json)")
        if path:
            Path(path).write_text(json.dumps(self.numeric_result, indent=2), encoding="utf-8")

    def start(self):
        self.sync()
        if not self.signature or self.process:
            return
        context = get_context("spawn")
        self.queue = context.Queue()
        self.process = context.Process(target=generate_documents, args=(self.signature[3], self.queue, self.kind), daemon=True)
        self.started = monotonic()
        try:
            self.process.start()
        except Exception:
            self.process = None
            self.queue.close()
            self.queue = None
            raise
        self.result = None
        self.output.clear()
        self.matrix_view.set_data(None)
        self.sync()
        self.status.setText("Generating in a separate process… Cancel is available (60-second limit).")

    def stop(self):
        if self.process is not None:
            if self.process.is_alive():
                self.process.terminate()
            self.process.join(timeout=1)
            self.process.close()
            self.process = None
        if self.queue is not None:
            self.queue.close()
            self.queue = None
        self.status.setText("Generation cancelled.")

    def poll(self):
        self.sync()
        if self.process is None or self.queue is None or self.signature is None:
            return
        try:
            ok, payload = self.queue.get_nowait()
        except Empty:
            if monotonic() - self.started > 60 or not self.process.is_alive():
                self.stop()
                self.status.setText("Generation stopped: timed out or worker exited. Try a shorter chain.")
            return
        self.stop()
        if ok:
            title = f"{self.signature[1]} relative to {self.signature[0]} — {self.model.currentText()}\n\n"
            self.result = (title + payload[0], "% " + title.replace("\n", " ") + "\n" + payload[1], payload[2])
            if len(self.cache) >= 4:
                self.cache.pop(next(iter(self.cache)))
            self.cache[self.signature] = self.result
            self.output.setPlainText(self.result[0])
            self.matrix_view.set_data(self.result[2])
            self.status.setText("Ready. M entries are defined below the matrix; t symbols are reusable expressions.")
        else:
            self.status.setText(self.title + " failed: " + payload)
        self.sync()

    def copy_result(self, index):
        self.sync()
        if self.result:
            self.e.qt.QApplication.clipboard().setText(self.result[index])

    def full_view(self):
        self.sync()
        if not self.result:
            return
        dialog = self.e.qt.QDialog(self.e.window)
        dialog.setWindowTitle(self.title + " — generated snapshot")
        dialog.resize(1100, 750)
        layout = self.e.qt.QVBoxLayout(dialog)
        output = self.e.qt.QPlainTextEdit()
        output.setReadOnly(True)
        output.setLineWrapMode(self.e.qt.QPlainTextEdit.NoWrap)
        output.setFont(self.output.font())
        output.setStyleSheet(self.output.styleSheet())
        output.setPlainText(self.result[0])
        view = MatrixView(self.e, output)
        view.set_data(self.result[2])
        if self.kind == 'Jacobian' and self.numeric_result:
            table = self.e.qt.QTableWidget()
            fill_matrix(self.e.qt, table, self.numeric_result['matrix'],
                        ['vx','vy','vz','ωx','ωy','ωz'],
                        [f'q{i+1}' for i in range(len(self.e.q))], numeric=True)
            view.pages.addWidget(table)
            view.mode.addItem('Numerical J(q) snapshot')
        view.mode.setCurrentIndex(self.matrix_view.mode.currentIndex())
        layout.addWidget(view.widget)
        self.e._button(layout, "Close", dialog.accept)
        dialog.exec()

    def export(self):
        self.sync()
        if not self.result:
            return
        path, selected = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Export " + self.title, "symbolic_" + self.kind.lower() + ".txt", "Text (*.txt);;LaTeX (*.tex)")
        if path:
            Path(path).write_text(self.result[1 if "LaTeX" in selected else 0], encoding="utf-8")
