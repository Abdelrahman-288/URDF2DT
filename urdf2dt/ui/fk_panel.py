"""Native physical-frame FK calculator and named pose library."""

import json
import warnings
from scipy.spatial.transform import Rotation
from math import pi
from pathlib import Path
from urdf2dt.inspection import inspect_fk
from urdf2dt.ui.number_display import decimal, display, residual, number_line
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
        self.model = qt.QComboBox()
        self.model.addItems(["URDF", "Automatic DH", "Committed DH", "Pending DH preview"])
        layout.addWidget(self.model)
        self.units = qt.QComboBox()
        self.units.addItems(["rad / m", "deg / m", "rad / mm", "deg / mm"])
        layout.addWidget(self.units)
        self.input = number_line(qt)
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
        self.model.currentIndexChanged.connect(lambda _: self.sync())
        self.units.currentTextChanged.connect(lambda _: self.sync())
        self.reference.currentTextChanged.connect(lambda _: self.sync())
        self.target.currentTextChanged.connect(lambda _: self.sync())

    def rebuild(self):
        self.input.clearFocus()
        self._signature = None
        self.refresh_presets()
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
        if getattr(self.e, "_loading_robot", False):
            return
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
                if self.units.currentIndex() % 2 and j.joint_type != JointType.PRISMATIC
                else q * 1000 if self.units.currentIndex() >= 2 and j.joint_type == JointType.PRISMATIC else q
            )
            for q, j in zip(self.e.q, joints)
        ]
        if not self.input.hasFocus():
            self.input.setText(" ".join(repr(q) for q in values))
            self._input_si = list(self.e.q)
            self._input_text = self.input.preciseText()
            self._input_units = self.units.currentIndex()
        self.order.setText(
            "Order: "
            + ", ".join(
                j.name
                + (
                    (" [mm]" if self.units.currentIndex() >= 2 else " [m]")
                    if j.joint_type == JointType.PRISMATIC
                    else " [deg]" if self.units.currentIndex() % 2 else " [rad]"
                )
                for j in joints
            )
        )

        signature = (
            self.units.currentIndex(),
            self.model.currentIndex(),
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
        values = numbers(self.input.preciseText(), len(self.e.q))
        joints = [
            j
            for j in self.e.application.run.chain.joints
            if j.joint_type != JointType.FIXED
        ]
        si = [
            (
                q * pi / 180
                if self.units.currentIndex() % 2 and j.joint_type != JointType.PRISMATIC
                else q / 1000 if self.units.currentIndex() >= 2 and j.joint_type == JointType.PRISMATIC else q
            )
            for q, j in zip(values, joints)
        ]
        if (not self.input.isModified()
                and self.input.preciseText() == getattr(self, "_input_text", None)
                and self.units.currentIndex() == getattr(self, "_input_units", None)):
            si = list(self._input_si)
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
        if self.model.currentIndex() == 3 and app.session.pending is None:
            self.result = {}
            self.output.setPlainText("No pending DH preview. Choose a committed model.")
            return
        self.result = inspect_fk(
            app.run.chain,
            app.session.state.automatic_model,
            self.e.current_model() if self.model.currentIndex() == 3 else app.session.state.working_model,
            self.e.q,
            self.reference.currentText(),
            self.target.currentText(),
            self.frames,
        )
        mode = self.model.currentIndex()
        self.result["model"] = self.model.currentText()
        self.result["pending_preview"] = mode == 3
        self.result["source_file"] = str(app.run.chain.source_urdf or "")
        self.result["pending_preview_excluded"] = app.session.pending is not None and mode != 3
        if mode:
            matrix = self.result["comparison"]["automatic" if mode == 1 else "accepted"]["matrix"]
            self.result["matrix"] = matrix
            self.result["position_m"] = [row[3] for row in matrix[:3]]
            self.result["rotation_matrix"] = [row[:3] for row in matrix[:3]]
            rotation = Rotation.from_matrix(self.result["rotation_matrix"])
            self.result["quaternion_xyzw"] = rotation.as_quat().tolist()
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                self.result["rpy_rad"] = rotation.as_euler("xyz").tolist()
            self.result["orientation_warning"] = "Gimbal lock: roll/yaw are not unique" if caught else ""
        result = self.result
        factor = 1000 if self.units.currentIndex() >= 2 else 1
        length_unit = "mm" if factor == 1000 else "m"
        angle_factor = 180/pi if self.units.currentIndex() % 2 else 1
        angle_unit = "deg" if angle_factor != 1 else "rad"
        matrix = "\n".join(
            "  ".join(decimal(value * factor if c == 3 and r < 3 else value) for c,value in enumerate(row)) for r,row in enumerate(result["matrix"])
        )
        lines = [
            self.model.currentText() + (" - NOT COMMITTED" if mode == 3 else ""),
            f"{result['target']} relative to {result['reference']}",
            matrix,
            f"Position [{length_unit}]: {display([v*factor for v in result['position_m']])}",
            f"Quaternion [x,y,z,w]: {display(result['quaternion_xyzw'])}",
            f"RPY [{angle_unit}]: {display([v*angle_factor for v in result['rpy_rad']])}",
            result["rpy_convention"],
            result["orientation_warning"],
        ]
        for name, comparison in result["comparison"].items():
            lines.append(
                f"URDF vs {name}: {residual(comparison['position_error_m'])} m / {residual(comparison['orientation_error_rad'])} rad"
            )
        lines.extend(
            [
                (
                    "Pending DH preview excluded"
                    if result["pending_preview_excluded"]
                    else "Pending preview shown" if mode == 3 else self.model.currentText()
                ),
                "Joint order: " + ", ".join(result["joint_names"]),
                f"q [rad/m]: {display(result['q_si'])}",
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
        if not self.result:
            raise ValueError("Choose an available model first")
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
