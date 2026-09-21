"""Native physical-frame FK calculator and named pose library."""

import json
from math import pi
from pathlib import Path
from urdf2dt.inspection import inspect_fk
from urdf2dt.dh.types import JointType
from urdf2dt.visualization.assets import numbers


class FKPanel:
    def __init__(self, e):
        self.e = e
        qt = e.qt
        self.poses = {}
        self.result = {}
        self.frames = {}
        self.pose_notes = {}
        self.thumbnails = {}
        self._signature = None
        page = qt.QWidget()
        layout = qt.QVBoxLayout(page)
        self.reference = qt.QComboBox()
        self.target = qt.QComboBox()
        layout.addWidget(qt.QLabel("Reference physical link"))
        layout.addWidget(self.reference)
        layout.addWidget(qt.QLabel("Target physical link"))
        layout.addWidget(self.target)
        self.order = qt.QLabel()
        self.order.setWordWrap(True)
        layout.addWidget(self.order)
        self.units = qt.QComboBox()
        self.units.addItems(["rad / m", "deg / m"])
        layout.addWidget(self.units)
        self.input = qt.QLineEdit()
        layout.addWidget(self.input)
        e._button(layout, "Apply joint inputs to robot", self.apply)
        e._button(layout, "Calculate / refresh", self.calculate)
        self.output = qt.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMinimumHeight(230)
        layout.addWidget(self.output, 1)
        e._button(
            layout,
            "Copy FK result",
            lambda: qt.QApplication.clipboard().setText(self.output.toPlainText()),
        )
        e._button(layout, "Export FK JSON…", self.export)
        self.presets = qt.QComboBox()
        layout.addWidget(self.presets)
        e._button(layout, "Save named pose…", self.save_pose)
        e._button(layout, "Recall named pose", self.recall)
        e._button(layout, "Add named tool/reference frame…", self.add_frame)
        scroll = qt.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.index = e.tabs.addTab(scroll, "FK calculator")
        e._action(
            e.window.menuBar().addMenu("Inspect"), "FK to selected link", self.selected
        )
        self.units.currentTextChanged.connect(lambda _: self.sync())
        self.reference.currentTextChanged.connect(lambda _: self.sync())
        self.target.currentTextChanged.connect(lambda _: self.sync())

    def rebuild(self):
        chain = self.e.application.run.chain
        names = (
            [chain.base_link] + [j.child_link for j in chain.joints] + list(self.frames)
        )
        for widget in (self.reference, self.target):
            widget.blockSignals(True)
            widget.clear()
            widget.addItems(names)
            widget.blockSignals(False)
        self.target.setCurrentIndex(len(names) - 1)
        self.sync()

    def sync(self):
        if not self.e.application:
            return
        joints = [
            j
            for j in self.e.application.run.chain.joints
            if j.joint_type != JointType.FIXED
        ]
        values = [
            (
                q * 180 / pi
                if self.units.currentIndex() and j.joint_type != JointType.PRISMATIC
                else q
            )
            for q, j in zip(self.e.q, joints)
        ]
        if not self.input.hasFocus():
            self.input.setText(" ".join(f"{q:.8g}" for q in values))
        self.order.setText(
            "Order: "
            + ", ".join(
                j.name
                + (
                    " [m]"
                    if j.joint_type == JointType.PRISMATIC
                    else " [deg]" if self.units.currentIndex() else " [rad]"
                )
                for j in joints
            )
        )

        signature = (
            tuple(self.e.q),
            self.reference.currentText(),
            self.target.currentText(),
            repr(self.frames),
            self.e.application.session.state.working_model,
            self.e.application.session.pending,
        )
        if (
            self.reference.currentText()
            and self.target.currentText()
            and signature != self._signature
        ):
            self.calculate()
            self._signature = signature

    def apply(self):
        values = numbers(self.input.text(), len(self.e.q))
        joints = [
            j
            for j in self.e.application.run.chain.joints
            if j.joint_type != JointType.FIXED
        ]
        si = [
            (
                q * pi / 180
                if self.units.currentIndex() and j.joint_type != JointType.PRISMATIC
                else q
            )
            for q, j in zip(values, joints)
        ]
        if any(
            q < control[2] or q > control[3]
            for q, control in zip(si, self.e.joint_controls)
        ):
            raise ValueError("Joint input outside permitted slider limits")
        self.e.set_pose(si)
        self.calculate()

    def calculate(self):
        app = self.e.application
        if not app:
            return
        self.result = inspect_fk(
            app.run.chain,
            app.session.state.automatic_model,
            app.session.state.working_model,
            self.e.q,
            self.reference.currentText(),
            self.target.currentText(),
            self.frames,
        )
        self.result["pending_preview_excluded"] = app.session.pending is not None
        result = self.result
        matrix = "\n".join(
            "  ".join(f"{value: .6f}" for value in row) for row in result["matrix"]
        )
        lines = [
            f"{result['target']} relative to {result['reference']}",
            matrix,
            f"Position [m]: {result['position_m']}",
            f"Quaternion [x,y,z,w]: {result['quaternion_xyzw']}",
            f"RPY [rad]: {result['rpy_rad']}",
            result["rpy_convention"],
            result["orientation_warning"],
        ]
        for name, comparison in result["comparison"].items():
            lines.append(
                f"URDF vs {name}: {comparison['position_error_m']:.3g} m / {comparison['orientation_error_rad']:.3g} rad"
            )
        lines.extend(
            [
                (
                    "Pending DH preview excluded"
                    if result["pending_preview_excluded"]
                    else "Committed DH model"
                ),
                "Joint order: " + ", ".join(result["joint_names"]),
                f"q [rad/m]: {result['q_si']}",
                result["scope"],
            ]
        )
        self.output.setPlainText("\n\n".join(line for line in lines if line))

    def selected(self):
        if self.e.studio.selected:
            name = self.e.studio.link_name(self.e.studio.selected)
            if self.target.findText(name) < 0:
                raise ValueError(
                    "Select a DH chain containing this component to compare physical FK"
                )
            self.target.setCurrentText(name)
            self.e.tabs.setCurrentIndex(self.index)
            self.calculate()

    def save_pose(self):
        name, ok = self.e.qt.QInputDialog.getText(self.e.window, "Named pose", "Name")
        if ok and name.strip():
            self.poses[name] = list(self.e.q)
            self.presets.clear()
            self.presets.addItems(self.poses)
            notes, _ = self.e.qt.QInputDialog.getText(
                self.e.window, "Pose notes", "Notes"
            )
            self.pose_notes[name] = notes
            import base64

            array = self.e.core.QByteArray()
            buffer = self.e.core.QBuffer(array)
            buffer.open(self.e.core.QIODevice.WriteOnly)
            self.e.window.grab().scaledToWidth(240).save(buffer, "PNG")
            self.thumbnails[name] = base64.b64encode(bytes(array)).decode()
            self.refresh_presets()
            self.e.studio.dirty = True

    def refresh_presets(self):
        from importlib import import_module
        import base64

        gui = import_module("PySide6.QtGui")
        self.presets.clear()
        self.presets.setIconSize(self.e.core.QSize(80, 50))
        for name in self.poses:
            pixmap = gui.QPixmap()
            if name in self.thumbnails:
                try:
                    pixmap.loadFromData(
                        base64.b64decode(self.thumbnails[name], validate=True), "PNG"
                    )
                except (ValueError, TypeError):
                    pass
            self.presets.addItem(gui.QIcon(pixmap), name)
            self.presets.setItemData(
                self.presets.count() - 1,
                self.pose_notes.get(name, ""),
                self.e.core.Qt.ToolTipRole,
            )

    def recall(self):
        if self.presets.currentText() in self.poses:
            self.e.set_pose(self.poses[self.presets.currentText()])
            self.calculate()

    def export(self):
        self.calculate()
        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Export FK", "fk.json", "JSON (*.json)"
        )
        if path:
            Path(path).write_text(json.dumps(self.result, indent=2), encoding="utf-8")

    def add_frame(self):
        link = self.e.studio.link_name(self.e.studio.selected)
        if link not in [self.e.application.run.chain.base_link] + [
            j.child_link for j in self.e.application.run.chain.joints
        ]:
            raise ValueError(
                "Select a link in the active DH chain before adding an FK frame"
            )
        name, ok = self.e.qt.QInputDialog.getText(
            self.e.window, "Named attached frame", f"Frame name attached to {link}"
        )
        if not ok:
            return
        if not name.strip() or name in self.e.studio.names or name in self.frames:
            raise ValueError("Frame name must be unique")
        xyz, ok = self.e.qt.QInputDialog.getText(
            self.e.window, "Frame offset", "XYZ [m]", text="0 0 0"
        )
        if not ok:
            return
        rpy, ok = self.e.qt.QInputDialog.getText(
            self.e.window, "Frame orientation", "Fixed XYZ RPY [rad]", text="0 0 0"
        )
        if not ok:
            return
        self.frames[name] = {
            "link": link,
            "xyz": numbers(xyz, 3),
            "rpy": numbers(rpy, 3),
        }
        self.e.studio.dirty = True
        self.rebuild()
        self.target.setCurrentText(name)
        self.calculate()
