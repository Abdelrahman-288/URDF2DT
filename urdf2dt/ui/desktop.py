"""Native Qt/VTK application with live pose sliders and constrained DH editing."""

from importlib import import_module
from math import pi, hypot, isfinite
from pathlib import Path
from random import uniform
import sys
import logging
from typing import Any

from urdf2dt._transforms import cross, multiply, position, rpy_transform, subtract, unit
from urdf2dt.app import Application
from urdf2dt.dh.classification import classify_dh_model, get_editable_params
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt.dh.types import FrameState, IDENTITY, JointType
from urdf2dt.kinematics import _configuration, dh_frame_transforms, urdf_link_transforms
from urdf2dt.parser.robot_document import RobotDocument, resolve_mesh
from urdf2dt.runtime import asset_path


def _numbers(text: str, count: int) -> tuple[float, ...]:
    values = tuple(float(v) for v in text.split())
    if len(values) != count or any(not isfinite(v) for v in values):
        raise ValueError("Invalid visual geometry coordinates")
    return values


class DesktopEditor:
    """Owns a native window; pose updates change actor matrices without rebuilding."""

    def __init__(self):
        self.qt = import_module("PySide6.QtWidgets")
        self.core = import_module("PySide6.QtCore")
        self.pv = import_module("pyvista")
        self.np = import_module("numpy")
        qt, core = self.qt, self.core
        self.window = qt.QMainWindow()
        self.window.setWindowTitle("URDF2DT — Robot & DH Studio[*]")
        icon = import_module("PySide6.QtGui").QIcon(
            str(asset_path("assets/urdf2dt.svg"))
        )
        self.window.setWindowIcon(icon)
        self.window.resize(1500, 940)
        self.settings = core.QSettings("URDF2DT", "Desktop")
        self.application: Application | None = None
        self.document: RobotDocument | None = None
        self.package_root: Path | None = None
        self.package_mappings: dict[str, str] = {}
        self.texture_mappings: dict[str, str] = {}
        self._chain_index = 0
        self.q: list[float] = []
        self.joint_controls: list[tuple[Any, Any, float, float]] = []
        self.edit_controls: dict[str, Any] = {}
        self.mesh_actors: list[tuple[Any, int, Any, str]] = []
        self.frame_actors: list[tuple[Any, int, str]] = []
        self.label_actors: list[tuple[Any, int, str]] = []
        self.bone_actors: list[Any] = []
        self.mesh_data: list[tuple[Any, int, Any]] = []
        self._refreshing = False
        self._pose_dirty = False
        self.warnings: list[str] = []
        self.last_render_ms = 0.0
        self.view: Any
        self.timer = core.QTimer(self.window)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        toolbar = self.window.addToolBar("Project")
        toolbar.setMovable(False)
        self._action(toolbar, "Open URDF…", self.open_dialog)
        self._action(toolbar, "Open example", self.open_example)
        self._action(toolbar, "Load session…", self.load_archive)
        self._action(toolbar, "Save session…", self.save_archive)
        self._action(toolbar, "Fit view", lambda: self.view.reset_camera())
        self.theme = qt.QComboBox()
        self.theme.addItems(["Light", "Dark"])
        toolbar.addWidget(self.theme)
        self.theme.currentTextChanged.connect(self.set_theme)
        splitter = qt.QSplitter()
        self.window.setCentralWidget(splitter)
        left = qt.QWidget()
        ll = qt.QVBoxLayout(left)
        left.setMinimumWidth(275)
        left.setMaximumWidth(500)
        ll.addWidget(qt.QLabel("ROBOT STRUCTURE"))
        self.chain = qt.QComboBox()
        ll.addWidget(self.chain)
        self.chain.currentIndexChanged.connect(
            lambda i: self.guard(lambda: self.select_chain(i))
        )
        self.links = qt.QListWidget()
        ll.addWidget(self.links, 1)
        self.links.currentRowChanged.connect(lambda _: self.queue_render())
        self._button(ll, "Edit URDF…", self.edit_source)
        self._button(ll, "Mesh package directory…", self.choose_package)
        self._button(ll, "Decompose collision (VHACD)", self.decompose)
        ll.addWidget(qt.QLabel("Opacity"))
        self.opacity = qt.QSlider(core.Qt.Horizontal)
        self.opacity.setRange(5, 100)
        self.opacity.setValue(100)
        self.opacity.valueChanged.connect(lambda _: self.queue_render())
        ll.addWidget(self.opacity)
        self.layers = {}
        for name, text, checked in (
            ("visual", "Robot meshes", True),
            ("urdf", "Link frames", True),
            ("dh", "Standard-DH frames", True),
            ("collision", "Collision geometry", False),
            ("bones", "Kinematic links", False),
            ("labels", "Frame names", False),
            ("joint_axes", "Joint-axis markers", False),
            ("automatic", "Automatic DH comparison overlay", False),
        ):
            box = qt.QCheckBox(text)
            box.setChecked(checked)
            box.toggled.connect(lambda _: self.queue_render())
            ll.addWidget(box)
            self.layers[name] = box
        self.notice = qt.QLabel("Open a URDF to begin.")
        self.notice.setWordWrap(True)
        ll.addWidget(self.notice)
        left_scroll = qt.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left)
        splitter.addWidget(left_scroll)
        center = qt.QWidget()
        cl = qt.QVBoxLayout(center)
        cl.setContentsMargins(0, 0, 0, 0)
        self.view = import_module("pyvistaqt").QtInteractor(center, auto_update=False)
        cl.addWidget(self.view.interactor, 3)
        self.table = qt.QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            [
                "Frame",
                "Joint",
                "State",
                "a [m]",
                "α [rad]",
                "d [m]",
                "θ offset [rad]",
                "q sign",
            ]
        )
        self.table.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        self.table.setMaximumHeight(220)
        cl.addWidget(self.table, 1)
        splitter.addWidget(center)
        right = qt.QWidget()
        rl = qt.QVBoxLayout(right)
        right.setMinimumWidth(300)
        right.setMaximumWidth(430)
        self.tabs = qt.QTabWidget()
        rl.addWidget(self.tabs)
        motion = qt.QWidget()
        ml = qt.QVBoxLayout(motion)
        ml.addWidget(qt.QLabel("Joint pose — live motion (rad / m)"))
        buttons = qt.QHBoxLayout()
        ml.addLayout(buttons)
        self._button(buttons, "Reset pose", lambda: self.set_pose([0.0] * len(self.q)))
        self._button(buttons, "Random pose", self.random_pose)
        scroll = qt.QScrollArea()
        scroll.setWidgetResizable(True)
        self.joint_panel = qt.QWidget()
        self.joint_layout = qt.QVBoxLayout(self.joint_panel)
        scroll.setWidget(self.joint_panel)
        ml.addWidget(scroll)
        self.tabs.addTab(motion, "Joint motion")
        edits = qt.QWidget()
        el = qt.QVBoxLayout(edits)
        el.addWidget(qt.QLabel("DH frame definition — live preview"))
        self.frame_select = qt.QComboBox()
        el.addWidget(self.frame_select)
        self.frame_select.currentIndexChanged.connect(self.select_frame)
        self.case_label = qt.QLabel()
        self.case_label.setWordWrap(True)
        el.addWidget(self.case_label)
        self.edit_panel = qt.QWidget()
        self.edit_layout = qt.QVBoxLayout(self.edit_panel)
        el.addWidget(self.edit_panel)
        self.accept_button = self._button(el, "Accept frame", self.accept)
        self.reject_button = self._button(el, "Reject preview", self.reject)
        self._button(el, "Unlock next", self.unlock)
        self._button(el, "Restore frame", self.restore_frame)
        self._button(el, "Restore all", self.restore_all)
        self._button(el, "Validate FK", self.validate)
        self.flip_buttons = {
            axis: self._button(el, label, lambda a=axis: self.preview_flip(a))
            for axis, label in (
                ("x", "Preview Flip X (X/Y reversed)"),
                ("z", "Preview Flip Z (Y/Z reversed)"),
            )
        }
        self._button(el, "Undo DH edit", lambda: self.history_step(False))
        self._button(el, "Redo DH edit", lambda: self.history_step(True))
        self.validation_label = qt.QLabel("No sampled FK result.")
        self.validation_label.setWordWrap(True)
        el.addWidget(self.validation_label)
        el.addStretch()
        edit_scroll = qt.QScrollArea()
        edit_scroll.setWidgetResizable(True)
        edit_scroll.setWidget(edits)
        self.tabs.addTab(edit_scroll, "DH editor")
        splitter.addWidget(right)
        splitter.setSizes([370, 790, 340])
        self.view.add_axes()
        mode = self.settings.value("theme", "Light")
        self.theme.setCurrentText(mode)
        self.set_theme(mode)
        self.window.statusBar().showMessage("Ready • Open a URDF or a saved session")
        from urdf2dt.ui.studio import Studio

        self.studio = Studio(self, ll)
        self.body_status = qt.QLabel("Open a robot to inspect its body geometry.")
        self.body_status.setWordWrap(True)
        ll.insertWidget(2, self.body_status)
        from urdf2dt.ui.fk_panel import FKPanel

        self.fk_panel = FKPanel(self)
        from urdf2dt.ui.project_panel import ProjectPanel

        self.project_panel = ProjectPanel(self)

    def _action(self, bar: Any, label: str, callback: Any) -> None:
        action = bar.addAction(label)
        action.triggered.connect(lambda _: self.guard(callback))

    def _button(self, layout: Any, text: str, callback: Any) -> Any:
        button = self.qt.QPushButton(text)
        layout.addWidget(button)
        button.clicked.connect(lambda _: self.guard(callback))
        return button

    def guard(self, function: Any) -> None:
        """Report callback failures through the application logger and visible status."""
        try:
            function()
        except Exception as exc:
            logging.getLogger(__name__).warning("Desktop action failed: %s", exc)
            self.window.statusBar().showMessage(str(exc))
            self.notice.setText(str(exc))

    def set_theme(self, mode: str) -> None:
        """Apply and persist the requested light or dark presentation theme."""
        dark = mode == "Dark"
        bg, panel, text, border = (
            ("#111827", "#1e293b", "#e5edf7", "#3b4a60")
            if dark
            else ("#f4f6fa", "#ffffff", "#172b4d", "#cbd5e1")
        )
        self.window.setStyleSheet(f"""
            QMainWindow, QWidget {{background:{bg};color:{text};font-family:'Segoe UI';font-size:12px;}}
            QToolBar {{spacing:12px;padding:9px;border-bottom:1px solid {border};}}
            QPushButton {{background:{panel};border:1px solid {border};border-radius:5px;padding:8px;}}
            QPushButton:hover {{border:1px solid #3b82f6;}} QPushButton:disabled {{color:#8794a8;}}
            QComboBox,QListWidget,QDoubleSpinBox,QTableWidget,QPlainTextEdit {{background:{panel};border:1px solid {border};padding:4px;}}
            QCheckBox::indicator {{width:14px;height:14px;border:1px solid {text};border-radius:3px;background:{panel};}}
            QCheckBox::indicator:checked {{background:#2689ee;border:1px solid #2689ee;}}
            QHeaderView::section {{background:{panel};padding:6px;border:0;}}
            QSlider::groove:horizontal {{height:5px;background:{border};border-radius:2px;}}
            QSlider::handle:horizontal {{background:#2689ee;width:14px;margin:-5px 0;border-radius:6px;}}
            QTabBar::tab {{padding:10px;background:{panel};}} QTabBar::tab:selected {{border-bottom:3px solid #2689ee;}}
        """)
        self.view.set_background(bg)
        self.settings.setValue("theme", mode)
        self.queue_render()

    def open_dialog(self) -> None:
        """Ask for a local URDF and load it if a file is selected."""
        if not self.project_panel.confirm_discard():
            return
        path, _ = self.qt.QFileDialog.getOpenFileName(
            self.window, "Open robot URDF", "", "URDF (*.urdf *.URDF)"
        )
        if path:
            self.load(path)

    def open_example(self):
        if self.project_panel.confirm_discard():
            self.load(str(asset_path("robots/scara/scara_rrpr.urdf")))

    def load(self, path: str) -> None:
        """Validate a rooted URDF and its initial serial path before replacing the project."""
        document = RobotDocument.load(path)
        from urdf2dt.visualization.assets import document_poses

        document_poses(document.root, {})
        # Validate the selected serial path before replacing a working project.
        application = Application(document.select(0))
        if hasattr(self, "studio"):
            self.studio.overrides = {}
            self.studio.appearance = {}
            self.studio.records = {}
            self.studio.undo_stack = []
            self.studio.redo_stack = []
            self.studio.selected = ""
            self.studio.preview_before = None
            self.fk_panel.frames = {}
            self.fk_panel.poses = {}
            self.fk_panel.pose_notes = {}
            self.fk_panel.thumbnails = {}
            self.project_panel.folder = None
        bundled = Path(path).resolve().parent / "STL_Files"
        self.package_root = bundled if bundled.is_dir() else None
        self.document = document
        self.package_mappings = {}
        self.texture_mappings = {}
        self.application = application
        self._chain_index = 0
        self.chain.blockSignals(True)
        self.chain.clear()
        for p in document.paths:
            self.chain.addItem(f"{p[0]} → {p[-1]} ({len(p)} links)")
        self.chain.blockSignals(False)
        self.rebuild()
        self.studio.dirty = False

    def select_chain(self, index: int) -> None:
        """Validate a selected serial path and start its independent editor session."""
        if self.document is None or index < 0:
            return
        if not self.project_panel.confirm_discard():
            self.chain.blockSignals(True)
            self.chain.setCurrentIndex(self._chain_index)
            self.chain.blockSignals(False)
            return
        try:
            app = Application(self.document.select(index))
        except Exception:
            self.chain.blockSignals(True)
            self.chain.setCurrentIndex(self._chain_index)
            self.chain.blockSignals(False)
            raise
        self._chain_index = index
        self.application = app
        self.fk_panel.frames = {}
        self.fk_panel.poses = {}
        self.fk_panel.pose_notes = {}
        self.fk_panel.thumbnails = {}
        self.rebuild()
        self.studio.dirty = True

    def choose_package(self) -> None:
        """Select a local mesh package root and rebuild available visual assets."""
        path = self.qt.QFileDialog.getExistingDirectory(
            self.window, "ROS package directory"
        )
        if path:
            name, ok = self.qt.QInputDialog.getText(
                self.window,
                "ROS package mapping",
                "Package name (blank uses this folder as the fallback root)",
                text=Path(path).name,
            )
            if not ok:
                return
            if name.strip():
                self.package_mappings[name.strip()] = path
            else:
                self.package_root = Path(path)
            self.studio.dirty = True
            if self.application:
                self.build_scene()
                self.update_scene()

    def _clear(self, layout: Any) -> None:
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def rebuild(self) -> None:
        """Recreate controls and scene actors for the current validated chain."""
        assert self.application is not None
        chain = self.application.run.chain
        self.validation_label.setText("No sampled FK result for this session.")
        self.q = [0.0] * len(chain.joint_names)
        self.links.clear()
        self.links.addItems([chain.base_link] + [j.child_link for j in chain.joints])
        self._clear(self.joint_layout)
        self.joint_controls = []
        for i, j in enumerate(
            j for j in chain.joints if j.joint_type != JointType.FIXED
        ):
            low, high = (j.limit.lower, j.limit.upper) if j.limit else (-pi, pi)
            self.joint_layout.addWidget(
                self.qt.QLabel(
                    j.name
                    + (" [m]" if j.joint_type == JointType.PRISMATIC else " [rad]")
                )
            )
            slider = self.qt.QSlider(self.core.Qt.Horizontal)
            slider.setRange(0, 10000)
            spin = self.qt.QDoubleSpinBox()
            spin.setDecimals(5)
            spin.setRange(low, high)
            spin.setSingleStep(0.01)
            self.joint_layout.addWidget(slider)
            self.joint_layout.addWidget(spin)
            self.joint_controls.append((slider, spin, low, high))
            slider.valueChanged.connect(lambda v, i=i: self.change_joint(i, v, True))
            spin.valueChanged.connect(lambda v, i=i: self.change_joint(i, v, False))
        self.joint_layout.addStretch()
        self.set_pose(self.q)
        self.refresh_editor(1)
        self.build_scene()
        self.fk_panel.rebuild()
        self.update_scene()
        self.view.reset_camera()

    def change_joint(self, index: int, value: float, from_slider: bool) -> None:
        """Synchronize one native slider and numeric pose field, then schedule a render."""
        slider, spin, low, high = self.joint_controls[index]
        q = low + (high - low) * value / 10000 if from_slider else value
        if self.q[index] != q:
            self.studio.dirty = True
        self.q[index] = q
        slider.blockSignals(True)
        spin.blockSignals(True)
        slider.setValue(round((q - low) / (high - low) * 10000) if high > low else 0)
        spin.setValue(q)
        slider.blockSignals(False)
        spin.blockSignals(False)
        self.queue_render()

    def set_pose(self, values: list[float]) -> None:
        """Validate the entire finite pose first, then clamp each joint to display limits."""
        checked = _configuration(values, len(self.joint_controls))
        for i, value in enumerate(checked):
            low, high = self.joint_controls[i][2:]
            self.change_joint(i, max(low, min(high, value)), False)

    def random_pose(self) -> None:
        """Choose a pose within each joint control interval without editing DH geometry."""
        self.set_pose([uniform(c[2], c[3]) for c in self.joint_controls])

    def queue_render(self) -> None:
        """Coalesce pose and presentation changes into the next render tick."""
        self._pose_dirty = True

    def _tick(self) -> None:
        self.window.setWindowModified(
            self.studio.dirty if hasattr(self, "studio") else False
        )
        if self._pose_dirty:
            self._pose_dirty = False
            self.guard(self.update_scene)

    def _mesh(self, node: Any) -> Any:
        geometry = node.find("geometry")
        if geometry is None:
            raise ValueError("Visual has no geometry")
        if geometry.find("box") is not None:
            size = _numbers(geometry.find("box").get("size"), 3)
            return self.pv.Cube(x_length=size[0], y_length=size[1], z_length=size[2])
        if geometry.find("sphere") is not None:
            return self.pv.Sphere(radius=float(geometry.find("sphere").get("radius")))
        if geometry.find("cylinder") is not None:
            c = geometry.find("cylinder")
            return self.pv.Cylinder(
                direction=(0, 0, 1),
                radius=float(c.get("radius")),
                height=float(c.get("length")),
            )
        mesh = geometry.find("mesh")
        if mesh is None:
            raise ValueError("Unsupported visual geometry")
        assert self.application is not None
        source = Path(self.application.run.chain.source_urdf or ".")
        filename = mesh.get("filename", "")
        path = resolve_mesh(filename, source, self.package_root, self.package_mappings)
        from urdf2dt.visualization.assets import SUPPORTED

        if path.suffix.lower() not in SUPPORTED:
            raise ValueError(f"Unsupported mesh format: {path.suffix}")
        cache_key = (
            str(path),
            path.stat().st_mtime_ns,
            mesh.get("scale", "1 1 1"),
            node.get("_mesh_units", "URDF / format default"),
        )
        if cache_key in self.studio.cache:
            return self.studio.cache[cache_key].copy()
        if Path(filename).suffix.lower() == ".dae" and path.suffix.lower() == ".stl":
            self.warnings.append(
                f"Visual proxy: {path.name} (bundled STL replaces unavailable DAE)"
            )
        if path.suffix.lower() in (".dae", ".obj"):
            tri = import_module("trimesh").load(str(path), force="mesh")
            faces = self.np.column_stack(
                (self.np.full(len(tri.faces), 3), tri.faces)
            ).ravel()
            result = self.pv.PolyData(tri.vertices, faces)
            if getattr(tri.visual, "uv", None) is not None:
                result.active_texture_coordinates = self.np.asarray(tri.visual.uv)
            if tri.visual.defined:
                colored = (
                    tri.visual.to_color()
                    if hasattr(tri.visual, "to_color")
                    else tri.visual
                )
                colors = self.np.asarray(colored.vertex_colors)
                if len(colors) == result.n_points:
                    result.point_data["source_rgba"] = colors
            if (
                path.suffix.lower() == ".dae"
                and node.get("_mesh_units", "URDF / format default")
                == "URDF / format default"
            ):
                from urdf2dt.visualization.assets import format_unit

                result.scale(format_unit(path), inplace=True)
        else:
            result = self.pv.read(path)
        result.scale(_numbers(mesh.get("scale", "1 1 1"), 3), inplace=True)
        self.studio.cache[cache_key] = result.copy()
        return result

    def build_scene(self) -> None:
        """Show loading progress while rebuilding geometry actors."""
        active = self.timer.isActive()
        self.timer.stop()
        self._building_scene = True
        progress = self.qt.QProgressDialog(
            "Loading robot geometry…", "", 0, 0, self.window
        )
        progress.setCancelButton(None)
        progress.setWindowModality(self.core.Qt.ApplicationModal)
        progress.setMinimumDuration(0)
        progress.show()
        self.qt.QApplication.processEvents(self.core.QEventLoop.ExcludeUserInputEvents)
        try:
            self._build_scene(progress)
        finally:
            progress.close()
            self._building_scene = False
            if active:
                self.timer.start()

    def _build_scene(self, progress) -> None:
        """Create local mesh, frame and label actors once for the loaded chain."""
        assert self.application is not None
        self.view.clear()
        self.view.add_axes()
        self.view.enable_lightkit()
        self.mesh_actors = []
        self.frame_actors = []
        self.label_actors = []
        self.bone_actors = []
        self.mesh_data = []
        self.joint_actors = []
        self.warnings = []
        self.studio.records = {}
        self.studio.nodes = {}
        self.studio.actors = {}
        from defusedxml.ElementTree import fromstring

        root = fromstring(
            (
                self.document.source.content
                if self.document
                else self.application.run.source.source.content
            ),
            forbid_dtd=True,
            forbid_entities=True,
            forbid_external=True,
        )
        from urdf2dt.visualization.assets import add_assignments

        add_assignments(root, self.studio.overrides)
        chain = self.application.run.chain
        names = [chain.base_link] + [j.child_link for j in chain.joints]
        names += [
            link.get("name", "")
            for link in root.findall("link")
            if link.get("name") not in names
        ]
        for link in root.findall("link"):
            index = names.index(link.get("name", ""))
            for kind in ("visual", "collision"):
                for element_index, source_node in enumerate(link.findall(kind)):
                    key = f"{link.get('name')}|{kind}|{element_index}"
                    progress.setLabelText(f"Loading {key}")
                    self.qt.QApplication.processEvents(
                        self.core.QEventLoop.ExcludeUserInputEvents
                    )
                    try:
                        node = self.studio.effective(key, source_node)
                        mesh = self._mesh(node)
                        origin = node.find("origin")
                        transform = (
                            IDENTITY
                            if origin is None
                            else rpy_transform(
                                _numbers(origin.get("xyz", "0 0 0"), 3),
                                _numbers(origin.get("rpy", "0 0 0"), 3),
                            )
                        )
                        color: Any = "#9daec3" if kind == "visual" else "#ef9b36"
                        from urdf2dt.visualization.assets import source_rgba

                        rgba = source_rgba(root, node, kind)
                        color = rgba[:3]
                        texture = None
                        material = node.find("material")
                        if material is not None:
                            if material.find("texture") is None:
                                material = next(
                                    (
                                        m
                                        for m in root.findall("material")
                                        if m.get("name") == material.get("name")
                                    ),
                                    material,
                                )
                            texture_node = material.find("texture")
                            if texture_node is not None:
                                texture_name = texture_node.get("filename", "")
                                try:
                                    texture_path = resolve_mesh(
                                        self.texture_mappings.get(
                                            texture_name, texture_name
                                        ),
                                        Path(chain.source_urdf or "."),
                                        self.package_root,
                                        self.package_mappings,
                                    )
                                    if mesh.active_texture_coordinates is None:
                                        raise ValueError(
                                            "Material texture needs UV coordinates; showing source color"
                                        )
                                    texture = self.pv.read_texture(texture_path)
                                except (ValueError, OSError) as exc:
                                    self.studio.records[key]["message"] = str(exc)
                                    self.warnings.append(f"{key}: {exc}")
                        actor = self.view.add_mesh(
                            mesh,
                            color=color,
                            texture=texture,
                            scalars=(
                                "source_rgba"
                                if "source_rgba" in mesh.point_data
                                and node.find("material") is None
                                else None
                            ),
                            rgb="source_rgba" in mesh.point_data,
                            smooth_shading=True,
                            ambient=0.15,
                            diffuse=0.8,
                            specular=0.25,
                            render=False,
                        )
                        self.mesh_actors.append((actor, index, transform, kind))
                        self.studio.actors[id(actor)] = {
                            "key": key,
                            "link": names[index],
                            "rgba": rgba,
                            "texture": texture,
                            "use_scalars": "source_rgba" in mesh.point_data
                            and node.find("material") is None,
                        }
                        self.studio.records[key]["dimensions_m"] = [
                            float(mesh.bounds[i + 1] - mesh.bounds[i])
                            for i in (0, 2, 4)
                        ]
                        self.studio.records[key]["source_rgba"] = list(rgba)
                        if kind == "visual":
                            self.mesh_data.append((mesh, index, transform))
                    except Exception as exc:
                        self.warnings.append(f"{link.get('name')} {kind}: {exc}")
                        if key not in self.studio.records:
                            self.studio.records[key] = {
                                "status": "unsupported",
                                "message": str(exc),
                                "original": "",
                                "resolved": "",
                                "format": "unknown",
                            }
                        if key in self.studio.records:
                            self.studio.records[key]["message"] = str(exc)
                            if self.studio.records[key]["status"] != "missing":
                                self.studio.records[key]["status"] = "unsupported"
        for kind, count in (
            ("urdf", len(names)),
            ("dh", len(chain.joint_names) + 1),
            ("automatic", len(chain.joint_names) + 1),
        ):
            for i in range(count):
                for j, color in enumerate(("#ed4b59", "#30b878", "#398bf5")):
                    direction = tuple(float(k == j) for k in range(3))
                    actor = self.view.add_mesh(
                        self.pv.Arrow(
                            direction=direction,
                            scale=0.055 if kind == "urdf" else 0.085,
                        ),
                        color="#aa82cf" if kind == "automatic" else color,
                        opacity=0.4 if kind == "automatic" else 1.0,
                        render=False,
                    )
                    self.frame_actors.append((actor, i, kind))
        for i, joint in enumerate(chain.joints):
            if joint.joint_type != JointType.FIXED:
                actor = self.view.add_mesh(
                    self.pv.Arrow(direction=joint.axis, scale=0.08),
                    color="#f39c32",
                    render=False,
                )
                self.joint_actors.append((actor, i))
        billboard = import_module("vtkmodules.vtkRenderingCore").vtkBillboardTextActor3D
        for kind, labels in (
            ("urdf", names),
            ("dh", [f"F{i}" for i in range(len(self.q) + 1)]),
        ):
            for i, label in enumerate(labels):
                actor = billboard()
                actor.SetInput(label)
                actor.SetDisplayOffset(6, 6 if kind == "urdf" else -18)
                actor.GetTextProperty().SetFontSize(13)
                self.view.add_actor(actor, render=False)
                self.label_actors.append((actor, i, kind))
        for _ in chain.joints:
            self.bone_actors.append(
                self.view.add_mesh(
                    self.pv.Cylinder(
                        center=(0.5, 0, 0), direction=(1, 0, 0), radius=0.009, height=1
                    ),
                    color="#6a8cad",
                    render=False,
                )
            )
        self.layers["bones"].setChecked(
            not any(k == "visual" for _, _, _, k in self.mesh_actors)
        )
        self.notice.setText(
            f"{len(names)} links • {len(self.q)} movable joints\n{len(self.warnings)} mesh notices (hover for details)."
            if self.warnings
            else f"{len(names)} links • {len(self.q)} movable joints\nMeshes loaded locally. Standard-DH convention."
        )
        self.notice.setToolTip("\n".join(self.warnings))
        loaded = sum(kind == "visual" for _, _, _, kind in self.mesh_actors)
        expected = sum("|visual|" in key for key in self.studio.records)
        self.body_status.setText(
            f"Body geometry: {loaded}/{expected} elements loaded."
            if loaded == expected and loaded
            else (
                f"Body incomplete: {loaded}/{expected} elements loaded. Inspect missing files in Geometry."
                if loaded
                else "Body unavailable: diagnostic skeleton only. Locate the missing files in Geometry."
            )
        )
        self.body_status.setToolTip("\n".join(self.warnings))
        self.studio.scene_built(names, root)

    def current_model(self) -> Any:
        """Return the committed model or its compensated pending geometric preview."""
        assert self.application is not None
        session = self.application.session
        pending = session.pending
        if pending is not None and pending.geometric_edit is not None:
            return recompute_model(
                session.state.working_model,
                pending.frame_index,
                pending.geometric_edit,
                self.application.run.config.geometry,
            )
        return session.state.working_model

    def update_scene(self) -> None:
        """Update cached actor transforms and visibility for the current pose and preview."""
        if self.application is None:
            return
        from time import perf_counter

        start = perf_counter()
        poses = urdf_link_transforms(self.application.run.chain, self.q)
        if self.document:
            from urdf2dt.visualization.assets import document_poses

            all_poses = document_poses(
                self.document.root,
                dict(zip(self.application.run.chain.joint_names, self.q)),
            )
            poses = tuple(all_poses[name] for name in self.studio.names)
        dh = dh_frame_transforms(self.current_model(), self.q)
        automatic = dh_frame_transforms(
            self.application.session.state.automatic_model, self.q
        )
        for actor, i, origin, kind in self.mesh_actors:
            actor.user_matrix = self.np.array(multiply(poses[i], origin))
            actor.visibility = self.layers[
                "collision" if kind == "generated" else kind
            ].isChecked()
            actor.prop.opacity = self.opacity.value() / 100 if kind == "visual" else 1.0
            actor.prop.show_edges = i == self.links.currentRow()
        for actor, i, kind in self.frame_actors:
            actor.user_matrix = self.np.array(
                (poses if kind == "urdf" else automatic if kind == "automatic" else dh)[
                    i
                ]
            )
            actor.visibility = self.layers[
                "collision" if kind == "generated" else kind
            ].isChecked()
            if kind == "urdf":
                frame = self.studio.appearance.get(self.studio.names[i], {}).get(
                    "frame", {}
                )
                actor.user_matrix = actor.user_matrix @ self.np.diag(
                    [frame.get("axis_size", 0.055) / 0.055] * 3 + [1.0]
                )
                actor.visibility = actor.visibility and frame.get("visible", True)
        for actor, i in self.joint_actors:
            joint = self.application.run.chain.joints[i]
            frame = self.studio.appearance.get(joint.child_link, {}).get("frame", {})
            actor.user_matrix = self.np.asarray(
                multiply(poses[i], joint.origin)
            ) @ self.np.diag([frame.get("marker_size", 0.08) / 0.08] * 3 + [1.0])
            actor.visibility = self.layers["joint_axes"].isChecked()
            actor.prop.color = frame.get("marker_color", "#f39c32")
            actor.prop.opacity = frame.get("marker_alpha", 1.0)
        for actor, i, kind in self.label_actors:
            if kind == "urdf":
                actor.GetTextProperty().SetFontSize(
                    self.studio.appearance.get(self.studio.names[i], {})
                    .get("frame", {})
                    .get("label_size", 13)
                )
            actor.SetPosition(*position((poses if kind == "urdf" else dh)[i]))
            actor.SetVisibility(
                self.layers["labels"].isChecked() and self.layers[kind].isChecked()
            )
            dark = self.theme.currentText() == "Dark"
            actor.GetTextProperty().SetColor(
                *((0.45, 0.7, 1.0) if dark else (0.1, 0.3, 0.65))
            )
        for actor, a, b in zip(self.bone_actors, poses, poses[1:]):
            delta = subtract(position(b), position(a))
            length = hypot(*delta)
            actor.visibility = self.layers["bones"].isChecked() and length > 1e-12
            if length > 1e-12:
                x = unit(delta)
                helper = (0.0, 0.0, 1.0) if abs(x[2]) < 0.9 else (0.0, 1.0, 0.0)
                y = unit(cross(helper, x))
                z = cross(x, y)
                actor.user_matrix = self.np.array(
                    tuple((delta[k], y[k], z[k], a[k][3]) for k in range(3))
                    + ((0.0, 0.0, 0.0, 1.0),)
                )
        self.studio.update()
        self.studio.sync_joint()
        self.fk_panel.sync()
        self.view.render()
        self.last_render_ms = (perf_counter() - start) * 1000

    def select_frame(self, index: int) -> None:
        """Discard a pending preview and synchronize controls for a zero-based selection."""
        if self._refreshing or not self.application or index < 0:
            return
        if self.application.session.pending:
            self.application.session.reject("Frame selection changed")
        self.refresh_editor(index + 1)
        self.queue_render()

    def refresh_editor(self, selected: int | None = None) -> None:
        """Expose only legal freedoms for the selected one-based DH frame."""
        if not self.application:
            return
        session = self.application.session
        self._refreshing = True
        index = selected or max(1, self.frame_select.currentIndex() + 1)
        self.frame_select.clear()
        for i, status in enumerate(session.state.frames, 1):
            self.frame_select.addItem(f"F{i} — {status.value}")
        self.frame_select.setCurrentIndex(index - 1)
        case = classify_dh_model(
            session.state.working_model, self.application.run.config.geometry
        )[index - 1]
        self.case_label.setText(case.description)
        self._clear(self.edit_layout)
        self.edit_controls = {}
        editable = session.state.frames[index - 1] in (
            FrameState.EDITABLE,
            FrameState.ACCEPTED,
        ) and all(f == FrameState.ACCEPTED for f in session.state.frames[: index - 1])
        for parameter in get_editable_params(case):
            self.edit_layout.addWidget(
                self.qt.QLabel(
                    parameter.name.replace("_", " ").title() + f" [{parameter.unit}]"
                )
            )
            slider = self.qt.QSlider(self.core.Qt.Horizontal)
            slider.setRange(-1000, 1000)
            limit = 0.5 if parameter.unit == "m" else pi
            spin = self.qt.QDoubleSpinBox()
            spin.setRange(-1e6, 1e6)
            spin.setDecimals(5)
            spin.setSingleStep(0.01)
            slider.setEnabled(editable)
            spin.setEnabled(editable)
            self.edit_layout.addWidget(slider)
            self.edit_layout.addWidget(spin)
            self.edit_controls[parameter.name] = spin
            slider.valueChanged.connect(
                lambda v, s=spin, l=limit: s.setValue(v * l / 1000)
            )
            spin.valueChanged.connect(
                lambda _, s=spin, sl=slider, l=limit: self.edit_value_changed(s, sl, l)
            )
        self.edit_layout.addWidget(
            self.qt.QLabel(
                "Slider preview range: ±0.5 m / ±π rad.\nNumeric fields allow larger values; edits are checked."
            )
        )
        self.accept_button.setEnabled(editable)
        for axis, button in self.flip_buttons.items():
            reason = "Accept upstream frames and unlock this frame first."
            allowed = editable
            if allowed:
                try:
                    recompute_model(
                        session.state.working_model,
                        index,
                        FrameEdit(axis_flip=axis),
                        self.application.run.config.geometry,
                    )
                except ValueError as exc:
                    allowed = False
                    reason = str(exc)
            button.setEnabled(allowed)
            button.setToolTip(
                "Preview a proper-rotation convention change; physical URDF joint coordinates stay unchanged."
                if allowed
                else reason
            )
        self.reject_button.setEnabled(session.pending is not None)
        self._refreshing = False
        self.update_table()

    def edit_value_changed(self, spin: Any, slider: Any, limit: float) -> None:
        """Replace the current preview with checked control values and invalidate displayed FK."""
        if self._refreshing or not self.application:
            return
        slider.blockSignals(True)
        slider.setValue(round(max(-1, min(1, spin.value() / limit)) * 1000))
        slider.blockSignals(False)
        session = self.application.session
        if session.pending:
            session.reject("Replaced by live slider preview")
        try:
            session.propose_edit(
                self.frame_select.currentIndex() + 1,
                FrameEdit(**{k: v.value() for k, v in self.edit_controls.items()}),
            )
            self.accept_button.setEnabled(True)
            self.reject_button.setEnabled(True)
            self.window.statusBar().showMessage(
                "Live DH preview • local checks passed • Accept to commit"
            )
        except ValueError as exc:
            self.accept_button.setEnabled(False)
            self.reject_button.setEnabled(False)
            self.window.statusBar().showMessage(str(exc))
        self.validation_label.setText("Current model has no sampled FK result.")
        self.update_table()
        self.queue_render()

    def update_table(self) -> None:
        """Display current preview parameters alongside committed acceptance states."""
        if not self.application:
            return
        model = self.current_model()
        self.table.setRowCount(len(model.rows))
        for i, row in enumerate(model.rows):
            values = [
                f"F{i+1}",
                row.joint_name,
                self.application.session.state.frames[i].value,
                f"{row.a:.6g}",
                f"{row.alpha:.6g}",
                f"{row.d:.6g}",
                f"{row.theta_offset:.6g}",
                str(row.joint_sign),
            ]
            for j, value in enumerate(values):
                item = self.qt.QTableWidgetItem(value)
                baseline = self.application.session.state.automatic_model.rows[i]
                if j >= 3:
                    original = (
                        baseline.a,
                        baseline.alpha,
                        baseline.d,
                        baseline.theta_offset,
                        baseline.joint_sign,
                    )[j - 3]
                    current = (
                        row.a,
                        row.alpha,
                        row.d,
                        row.theta_offset,
                        row.joint_sign,
                    )[j - 3]
                    item.setToolTip(f"Automatic baseline: {original:.9g}")
                    if abs(current - original) > 1e-10:
                        font = item.font()
                        font.setBold(True)
                        item.setFont(font)
                self.table.setItem(i, j, item)

    def accept(self) -> None:
        """Confirm an unchanged frame or accept its pending geometric preview."""
        if not self.application:
            return
        session = self.application.session
        if not session.pending:
            session.propose_edit(self.frame_select.currentIndex() + 1, FrameEdit())
        decision = session.accept()
        self.studio.dirty = True
        self.window.statusBar().showMessage(decision.reason)
        self.validation_label.setText("Run sampled FK after accepting all frames.")
        self.refresh_editor()
        self.queue_render()

    def preview_flip(self, axis: str) -> None:
        if not self.application:
            return
        session = self.application.session
        if session.pending:
            session.reject("Replaced by axis convention preview")
        session.propose_edit(
            self.frame_select.currentIndex() + 1, FrameEdit(axis_flip=axis)
        )
        self.validation_label.setText(
            "Axis convention preview; accept/reject required. Previous validation stale."
        )
        self.accept_button.setEnabled(True)
        self.reject_button.setEnabled(True)
        self.update_table()
        self.queue_render()

    def history_step(self, redo: bool) -> None:
        if not self.application:
            return
        session = self.application.session
        if session.pending:
            session.reject("Preview discarded for history navigation")
        session.redo() if redo else session.undo()
        self.validation_label.setText("History restored; sampled FK is stale")
        self.studio.dirty = True
        self.refresh_editor()
        self.queue_render()

    def reject(self) -> None:
        """Discard the live preview and refresh the committed display."""
        if self.application and self.application.session.pending:
            self.application.session.reject()
        self.refresh_editor()
        self.queue_render()

    def unlock(self) -> None:
        """Open the first unaccepted frame after discarding any preview."""
        if not self.application:
            return
        if self.application.session.pending:
            self.application.session.reject("Preview discarded")
        index = self.application.session.unlock_next()
        self.refresh_editor(index)
        self.queue_render()

    def restore_frame(self) -> None:
        """Restore the selected baseline frame and invalidate the displayed FK result."""
        if not self.application:
            return
        if self.application.session.pending:
            self.application.session.reject("Preview discarded")
        self.application.session.restore_frame(self.frame_select.currentIndex() + 1)
        self.studio.dirty = True
        self.validation_label.setText("Restored frame; previous FK result is stale.")
        self.refresh_editor()
        self.queue_render()

    def restore_all(self) -> None:
        """Restore the exact automatic baseline and refresh every editor control."""
        if not self.application:
            return
        self.application.session.restore_automatic()
        self.studio.dirty = True
        self.validation_label.setText("Automatic baseline restored.")
        self.refresh_editor(1)
        self.queue_render()

    def validate(self) -> None:
        """Display fresh sampled FK or a readable incomplete-session error."""
        if not self.application:
            return
        self.validation_label.setText("Validating current session…")
        try:
            report = self.application.validate()
        except Exception as exc:
            self.validation_label.setText(str(exc))
            raise
        data = report.to_dict()
        self.validation_label.setText(
            f"Sampled FK {'PASS' if report.passed else 'FAIL'}\n{data['diagnostic']['message']}\nProvisional tolerances; not a continuous-space proof."
        )

    def save_archive(self) -> None:
        """Revalidate the session and save it to a new selected directory."""
        if not self.application:
            return
        path, _ = self.qt.QFileDialog.getSaveFileName(
            self.window,
            "New session folder (must not exist)",
            str(
                Path(
                    self.core.QStandardPaths.writableLocation(
                        self.core.QStandardPaths.DocumentsLocation
                    )
                )
                / "URDF2DT"
                / "sessions"
                / "desktop_run"
            ),
            "Session directory (*)",
        )
        if path:
            self.application.export(path)
            self.window.statusBar().showMessage("Validated session saved")

    def load_archive(self) -> None:
        """Reload a validated JSON session and resolve local visual assets separately."""
        if not self.project_panel.confirm_discard():
            return
        path, _ = self.qt.QFileDialog.getOpenFileName(
            self.window, "Load session", "", "JSON (*.json)"
        )
        if path:
            loaded = Application.resume(path)
            self.studio.overrides = {}
            self.studio.appearance = {}
            self.studio.undo_stack = []
            self.studio.redo_stack = []
            self.studio.preview_before = None
            self.fk_panel.frames = {}
            self.fk_panel.poses = {}
            self.fk_panel.pose_notes = {}
            self.fk_panel.thumbnails = {}
            self.project_panel.folder = None
            self.application = loaded
            self.document = None
            source = Path(self.application.run.chain.source_urdf or ".")
            bundled = source.parent / "STL_Files"
            self.package_root = bundled if bundled.is_dir() else None
            self.chain.blockSignals(True)
            self.chain.clear()
            self.chain.addItem("Saved serial chain")
            self.chain.blockSignals(False)
            self.rebuild()

    def edit_source(self) -> None:
        """Open the source XML editor; save a new copy before attempting to load it."""
        if not self.project_panel.confirm_discard():
            return
        if self.document is None:
            raise ValueError("Open a URDF file first")
        dialog = self.qt.QDialog(self.window)
        dialog.setWindowTitle("Edit URDF — save a copy and validate")
        dialog.resize(900, 650)
        layout = self.qt.QVBoxLayout(dialog)
        text = self.qt.QPlainTextEdit()
        text.setPlainText(self.document.source.content.decode("utf-8"))
        layout.addWidget(text)

        def save() -> None:
            path, _ = self.qt.QFileDialog.getSaveFileName(
                dialog, "Save URDF copy", "robot_edited.urdf", "URDF (*.urdf)"
            )
            if path:
                with Path(path).open("x", encoding="utf-8") as stream:
                    stream.write(text.toPlainText())
                self.load(path)
                dialog.accept()

        self._button(layout, "Save copy and load", save)
        dialog.exec()

    def decompose(self) -> None:
        """Generate view-only convex collision parts, replacing previous generated parts."""
        if not self.mesh_data:
            raise ValueError(
                "Load visual meshes before generating convex collision parts"
            )
        trimesh = import_module("trimesh")
        self.window.statusBar().showMessage("Generating VHACD convex collision parts…")
        self.qt.QApplication.processEvents()
        parts = []
        for mesh, index, origin in self.mesh_data:
            surface = mesh.extract_surface(algorithm="dataset_surface").triangulate()
            tri = trimesh.Trimesh(
                vertices=surface.points,
                faces=surface.faces.reshape(-1, 4)[:, 1:],
                process=True,
            )
            for hull in tri.convex_decomposition(maxConvexHulls=8, resolution=10000):
                faces = self.np.column_stack(
                    (self.np.full(len(hull.faces), 3), hull.faces)
                ).ravel()
                parts.append((self.pv.PolyData(hull.vertices, faces), index, origin))
        for actor, _, _, kind in self.mesh_actors:
            if kind == "generated":
                self.view.remove_actor(actor, render=False)
        self.mesh_actors = [item for item in self.mesh_actors if item[3] != "generated"]
        for mesh, index, origin in parts:
            actor = self.view.add_mesh(mesh, color="#f0a43b", opacity=0.3, render=False)
            self.mesh_actors.append((actor, index, origin, "generated"))
        self.layers["collision"].setChecked(True)
        self.queue_render()
        self.window.statusBar().showMessage(
            f"Generated {len(parts)} convex collision parts for viewing (source unchanged)"
        )

    def close(self) -> None:
        """Stop rendering and release the native window and VTK resources."""
        self._closing_programmatically = True
        self.timer.stop()
        self.view.close()
        self.window.close()


def main() -> int:
    """Launch the native editor, optionally opening the requested URDF."""
    import argparse

    parser = argparse.ArgumentParser(description="URDF2DT native robot and DH editor")
    parser.add_argument("source", nargs="?", help="Optional URDF file to open")
    parser.add_argument(
        "--verify-package",
        metavar="NEW_FOLDER",
        help="Run packaged GUI diagnostics and exit",
    )
    parser.add_argument(
        "--verify-studio",
        metavar="NEW_FOLDER",
        help="Verify new desktop features and portable projects",
    )
    parser.add_argument(
        "--mesh-robot", help="External mesh-backed URDF for native verification"
    )
    parser.add_argument(
        "--verify-library",
        metavar="NEW_FOLDER",
        help="Verify body rendering from an external robot collection",
    )
    parser.add_argument("--robot-library", help="Root of the external robots directory")
    args = parser.parse_args()
    qt = import_module("PySide6.QtWidgets")
    app = qt.QApplication.instance() or qt.QApplication(sys.argv[:1])
    app.setApplicationName("URDF2DT")
    app.setOrganizationName("URDF2DT")
    from logging.handlers import RotatingFileHandler

    core = import_module("PySide6.QtCore")
    log_dir = (
        Path(
            core.QStandardPaths.writableLocation(
                core.QStandardPaths.AppLocalDataLocation
            )
        )
        / "logs"
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    handler = RotatingFileHandler(
        log_dir / "desktop.log", maxBytes=1_000_000, backupCount=2, encoding="utf-8"
    )
    handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(name)s %(message)s")
    )
    logger = logging.getLogger("urdf2dt")
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    if args.verify_library:
        if not args.robot_library:
            parser.error("--verify-library requires --robot-library")
        from urdf2dt.ui.library_check import verify_library

        return verify_library(args.verify_library, args.robot_library, app)
    if args.verify_studio:
        from urdf2dt.ui.studio_check import verify_studio

        return verify_studio(args.verify_studio, app, args.mesh_robot)
    if args.verify_package:
        from urdf2dt.ui.package_check import verify_package

        try:
            return verify_package(Path(args.verify_package), app)
        except Exception:
            logger.exception("Packaged diagnostic failed")
            return 1
    editor = DesktopEditor()
    if args.source:
        editor.guard(lambda: editor.load(args.source))
    app.aboutToQuit.connect(editor.timer.stop)
    editor.window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())
