"""On-demand symbolic FK with cancellable isolated computation."""

from multiprocessing import get_context
from queue import Empty
from pathlib import Path
from time import monotonic
from urdf2dt.symbolic import generate_documents


class SymbolicPanel:
    def __init__(self, e):
        self.e = e
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
        self.generate = e._button(layout, "Generate symbolic FK", self.start)
        self.cancel = e._button(layout, "Cancel generation", self.stop)
        self.status = e.qt.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.output = e.qt.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setLineWrapMode(e.qt.QPlainTextEdit.NoWrap)
        font = self.output.font()
        font.setFamily("Consolas")
        self.output.setFont(font)
        layout.addWidget(self.output, 1)
        self.expand = e._button(layout, "Open full matrix view…", self.full_view)
        self.copy = e._button(layout, "Copy symbolic text", lambda: self.copy_result(0))
        self.latex = e._button(layout, "Copy LaTeX", lambda: self.copy_result(1))
        self.export_button = e._button(layout, "Export symbolic FK…", self.export)
        e.tabs.addTab(page, "Symbolic FK")
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
        current = self.current()
        if current != self.signature:
            self.stop()
            self.signature = current
            self.result = self.cache.get(current)
            self.output.setPlainText(self.result[0] if self.result else "")
            self.status.setText("Cached result ready." if self.result else "Generate on demand. Joint motion does not require regeneration.")
        self.scope.setText(
            f"{current[1]} relative to {current[0]}. Standard DH; q uses source joint coordinates.\n"
            "This tab covers the selected chain endpoints, independently of the numerical FK link selectors.\n"
            "Pending previews are excluded. Dimensions remain numeric constants."
            if current else "Open a robot to generate symbolic FK.")
        self.generate.setEnabled(current is not None and self.process is None)
        self.cancel.setEnabled(self.process is not None)
        for button in (self.copy, self.latex, self.export_button, self.expand):
            button.setEnabled(self.result is not None)

    def start(self):
        self.sync()
        if not self.signature or self.process:
            return
        context = get_context("spawn")
        self.queue = context.Queue()
        self.process = context.Process(target=generate_documents, args=(self.signature[3], self.queue), daemon=True)
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
            self.result = (title + payload[0], "% " + title.replace("\n", " ") + "\n" + payload[1])
            if len(self.cache) >= 4:
                self.cache.pop(next(iter(self.cache)))
            self.cache[self.signature] = self.result
            self.output.setPlainText(self.result[0])
            self.status.setText("Ready. M entries are defined below the matrix; t symbols are reusable expressions.")
        else:
            self.status.setText("Symbolic FK failed: " + payload)
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
        dialog.setWindowTitle("Symbolic FK — generated snapshot")
        dialog.resize(1100, 750)
        layout = self.e.qt.QVBoxLayout(dialog)
        output = self.e.qt.QPlainTextEdit()
        output.setReadOnly(True)
        output.setLineWrapMode(self.e.qt.QPlainTextEdit.NoWrap)
        output.setFont(self.output.font())
        output.setPlainText(self.result[0])
        layout.addWidget(output)
        self.e._button(layout, "Close", dialog.accept)
        dialog.exec()

    def export(self):
        self.sync()
        if not self.result:
            return
        path, selected = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Export symbolic FK", "symbolic_fk.txt", "Text (*.txt);;LaTeX (*.tex)")
        if path:
            Path(path).write_text(self.result[1 if "LaTeX" in selected else 0], encoding="utf-8")
