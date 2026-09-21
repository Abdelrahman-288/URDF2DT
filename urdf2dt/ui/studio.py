"""Desktop presentation state and selection inspector, separate from DH state."""

from copy import deepcopy
from importlib import import_module
from pathlib import Path
import json
from xml.etree.ElementTree import tostring

from urdf2dt.visualization.assets import (
    describe,
    geometry_node,
    numbers,
    UNITS,
    SUPPORTED,
)


class Studio:
    def __init__(self, editor, layout):
        self.e = editor
        self.overrides: dict = {}
        self.appearance: dict = {}
        self.records: dict = {}
        self.actors: dict = {}
        self.nodes: dict = {}
        self.cache: dict = {}
        self.undo_stack: list = []
        self.redo_stack: list = []
        self.preview_before = None
        self.selected = ""
        self.names: list = []
        self.dirty = False
        qt = editor.qt
        editor.links.hide()
        self.search = qt.QLineEdit()
        self.search.setPlaceholderText("Search links, joints and geometry…")
        layout.insertWidget(2, self.search)
        self.tree = qt.QTreeWidget()
        self.tree.setHeaderLabels(["Component", "Status"])
        self.tree.setSelectionMode(qt.QAbstractItemView.ExtendedSelection)
        self.tree.setMinimumHeight(160)
        self.tree.setIndentation(12)
        self.tree.header().setSectionResizeMode(0, qt.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        layout.insertWidget(3, self.tree, 2)
        self.crumb = qt.QLabel()
        self.crumb.setWordWrap(True)
        layout.insertWidget(4, self.crumb)
        self.properties = qt.QTabWidget()
        self.properties.setMaximumHeight(360)
        layout.insertWidget(5, self.properties, 1)
        page = qt.QWidget()
        form = qt.QFormLayout(page)
        self.alias = qt.QLineEdit()
        form.addRow("Display alias", self.alias)
        self.notes = qt.QLineEdit()
        form.addRow("Notes", self.notes)
        self.alpha = qt.QDoubleSpinBox()
        self.alpha.setRange(0, 1)
        self.alpha.setSingleStep(0.1)
        self.alpha.setValue(1)
        form.addRow("Opacity", self.alpha)
        self.visible = qt.QCheckBox("Visible")
        self.visible.setChecked(True)
        form.addRow(self.visible)
        self.style = qt.QComboBox()
        self.style.addItems(["solid", "wireframe", "edges"])
        form.addRow("Style", self.style)
        self.color = ""
        editor._button(form, "Choose color…", self.pick_color)
        editor._button(form, "Apply appearance to selection", self.apply_appearance)
        editor._button(form, "Reset selected appearance", self.reset_appearance)
        self.properties.addTab(page, "Properties")
        geo = qt.QWidget()
        gf = qt.QFormLayout(geo)
        self.path = qt.QLineEdit()
        self.path.setReadOnly(True)
        gf.addRow("Resolved file", self.path)
        self.asset_info = qt.QLabel()
        self.asset_info.setWordWrap(True)
        gf.addRow(self.asset_info)
        self.units = qt.QComboBox()
        self.units.addItems(list(UNITS))
        gf.addRow("Mesh units", self.units)
        self.scale = qt.QLineEdit("1 1 1")
        gf.addRow("XYZ scale", self.scale)
        self.xyz = qt.QLineEdit("0 0 0")
        gf.addRow("Position [m]", self.xyz)
        self.rpy = qt.QLineEdit("RPY [rad]")
        self.rpy.setText("0 0 0")
        gf.addRow("RPY [rad]", self.rpy)
        for label, callback in (
            ("Browse / replace / locate mesh…", self.browse),
            ("Copy path", self.copy_path),
            ("Open containing folder", self.open_folder),
            ("Preview transform", self.preview),
            ("Apply preview", self.apply_preview),
            ("Cancel preview", self.cancel_preview),
        ):
            editor._button(gf, label, callback)
        scroll = qt.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(geo)
        self.properties.addTab(scroll, "Geometry")
        self.source = qt.QPlainTextEdit()
        self.source.setReadOnly(True)
        source_page = qt.QWidget()
        source_layout = qt.QVBoxLayout(source_page)
        source_layout.addWidget(self.source)
        self.joint_value = qt.QDoubleSpinBox()
        self.joint_value.setDecimals(6)
        self.joint_index = None
        self.joint_value.setToolTip(
            "Selected joint position in rad or m; physical joint definitions remain read-only"
        )
        source_layout.addWidget(self.joint_value)
        self.joint_value.valueChanged.connect(
            lambda value: (
                self.e.change_joint(self.joint_index, value, False)
                if self.joint_index is not None
                else None
            )
        )
        self.properties.addTab(source_page, "Source / joints")
        frames = qt.QWidget()
        ff = qt.QFormLayout(frames)
        self.axis_size = qt.QDoubleSpinBox()
        self.axis_size.setRange(0.005, 1)
        self.axis_size.setValue(0.055)
        self.axis_size.setSingleStep(0.01)
        ff.addRow("Frame axes [m]", self.axis_size)
        self.label_size = qt.QSpinBox()
        self.label_size.setRange(8, 40)
        self.label_size.setValue(13)
        ff.addRow("Label size [pt]", self.label_size)
        self.frame_visible = qt.QCheckBox("Selected link frame visible")
        self.frame_visible.setChecked(True)
        ff.addRow(self.frame_visible)
        self.marker_size = qt.QDoubleSpinBox()
        self.marker_size.setRange(0.005, 1)
        self.marker_size.setValue(0.08)
        ff.addRow("Joint marker [m]", self.marker_size)
        self.marker_alpha = qt.QDoubleSpinBox()
        self.marker_alpha.setRange(0, 1)
        self.marker_alpha.setValue(1)
        ff.addRow("Marker opacity", self.marker_alpha)
        self.marker_color = "#f39c32"
        editor._button(ff, "Joint marker color…", self.pick_marker_color)
        editor._button(ff, "Apply frame / marker display", self.apply_frames)
        editor._button(ff, "Reset selected joint", self.reset_joint)
        editor._button(ff, "Explain adjacent DH geometry", self.explain)
        editor._button(ff, "Measure frame origins / Z axes", self.measure)
        frame_scroll = qt.QScrollArea()
        frame_scroll.setWidgetResizable(True)
        frame_scroll.setWidget(frames)
        self.properties.addTab(frame_scroll, "Frames")
        actions = qt.QHBoxLayout()
        layout.insertLayout(6, actions)
        for label, callback in (
            ("Focus", self.focus),
            ("Isolate", self.isolate),
            ("Show all", self.show_all),
        ):
            editor._button(actions, label, callback)
        menu = editor.window.menuBar().addMenu("Structure")
        for label, callback in (
            ("Undo property change", self.undo),
            ("Redo property change", self.redo),
            ("Health checks", self.health),
            ("Export component report…", self.export_report),
        ):
            editor._action(menu, label, callback)
        self.tree.itemSelectionChanged.connect(self.select)
        self.tree.itemChanged.connect(self.toggle_item)
        self.search.textChanged.connect(self.filter)
        self.filter_mode = qt.QComboBox()
        self.filter_mode.addItems(
            ["All components", "Links", "Joints", "Geometry", "Hidden", "Asset issues"]
        )
        layout.insertWidget(3, self.filter_mode)
        self.filter_mode.currentTextChanged.connect(
            lambda _: self.filter(self.search.text())
        )
        editor._action(menu, "Select subtree", self.select_subtree)
        try:
            editor.view.enable_mesh_picking(
                callback=self.picked,
                use_actor=True,
                show=False,
                left_clicking=True,
                show_message=False,
            )
        except Exception as exc:
            editor.window.statusBar().showMessage(
                f"Viewport picking unavailable: {exc}"
            )

    def pick_marker_color(self):
        color = self.e.qt.QColorDialog.getColor(parent=self.e.window)
        if color.isValid():
            self.marker_color = color.name()

    def apply_frames(self):
        before = self.snapshot()
        for name in self.selected_links():
            self.appearance.setdefault(name, {})["frame"] = {
                "visible": self.frame_visible.isChecked(),
                "axis_size": self.axis_size.value(),
                "label_size": self.label_size.value(),
                "marker_size": self.marker_size.value(),
                "marker_alpha": self.marker_alpha.value(),
                "marker_color": self.marker_color,
            }
        self.changed(before)

    def reset_joint(self):
        name = self.link_name(self.selected)
        joints = [
            j
            for j in self.e.application.run.chain.joints
            if j.joint_type.value != "fixed"
        ]
        for i, joint in enumerate(joints):
            if joint.child_link == name:
                self.e.change_joint(
                    i,
                    max(
                        self.e.joint_controls[i][2], min(self.e.joint_controls[i][3], 0)
                    ),
                    False,
                )
                return
        raise ValueError("Selected component has no movable incoming joint")

    def explain(self):
        from urdf2dt.dh.classification import classify_dh_model
        from dataclasses import asdict

        name = self.link_name(self.selected)
        chain = self.e.application.run.chain
        links = [chain.base_link] + [j.child_link for j in chain.joints]
        if name not in links:
            raise ValueError("Select a DH path containing this link first")
        prefix = chain.joints[: links.index(name)]
        index = max(0, sum(j.joint_type.value != "fixed" for j in prefix) - 1)
        self.e.frame_select.setCurrentIndex(index)
        case = classify_dh_model(
            self.e.application.session.state.working_model,
            self.e.application.run.config.geometry,
        )[index]
        self.e.qt.QMessageBox.information(
            self.e.window,
            f"DH frame F{index+1}",
            case.description
            + "\n"
            + json.dumps(asdict(case), indent=2, default=str)
            + "\nFlip X = Rz(pi): X/Y reverse. Flip Z = Rx(pi): Y/Z reverse; next joint sign compensates.\n"
            "Both preserve right-handed frames and physical URDF coordinates. Factorization and FK are checked.\n"
            "The automatic baseline remains immutable. Accept/reject the preview in the DH editor.",
        )

    def measure(self):
        from urdf2dt.visualization.assets import document_poses
        from defusedxml.ElementTree import fromstring
        import math

        names = list(self.selected_links())
        if len(names) != 2:
            raise ValueError(
                "Select exactly two links for model frame-origin / Z-axis measurements"
            )
        source = (
            self.e.document.source
            if self.e.document
            else self.e.application.run.source.source
        )
        coordinates = dict(zip(self.e.application.run.chain.joint_names, self.e.q))
        poses = document_poses(fromstring(source.content), coordinates)
        a, b = [self.e.np.asarray(poses[name]) for name in names]
        distance = float(self.e.np.linalg.norm(a[:3, 3] - b[:3, 3]))
        angle = math.degrees(
            math.acos(float(self.e.np.clip(a[:3, 2] @ b[:3, 2], -1, 1)))
        )
        self.e.qt.QMessageBox.information(
            self.e.window,
            "Model-frame measurement",
            f"{names[0]} ↔ {names[1]}\nOrigin distance: {distance:.8g} m\nZ-axis angle: {angle:.8g} deg\nKinematic quantities, not mesh surface measurements.",
        )

    def snapshot(self):
        return deepcopy((self.overrides, self.appearance))

    def changed(self, before):
        self.undo_stack.append(before)
        self.redo_stack.clear()
        self.dirty = True
        self.e.queue_render()

    def undo(self):
        if self.undo_stack:
            self.redo_stack.append(self.snapshot())
            self.overrides, self.appearance = self.undo_stack.pop()
            self.dirty = True
            self.e.build_scene()
            self.e.queue_render()

    def redo(self):
        if self.redo_stack:
            self.undo_stack.append(self.snapshot())
            self.overrides, self.appearance = self.redo_stack.pop()
            self.dirty = True
            self.e.build_scene()
            self.e.queue_render()

    def effective(self, key, node):
        self.nodes[key] = node
        source = Path(self.e.application.run.chain.source_urdf or ".")
        self.records[key] = describe(
            node,
            source,
            self.e.package_root,
            self.overrides.get(key, {}),
            self.e.package_mappings,
        )
        return geometry_node(node, self.overrides.get(key, {}))

    def scene_built(self, names, root):
        self.names = names
        self.root = root
        selected = self.selected
        self.tree.blockSignals(True)
        self.tree.clear()
        self.items = {}
        links = {}
        joints = {}
        for name in names:
            item = self.e.qt.QTreeWidgetItem(
                [self.appearance.get(name, {}).get("alias") or name, "Link"]
            )
            item.setData(0, 256, name)
            item.setToolTip(0, name)
            item.setCheckState(
                0,
                (
                    self.e.core.Qt.Checked
                    if self.appearance.get(name, {}).get("visible", True)
                    else self.e.core.Qt.Unchecked
                ),
            )
            item.setIcon(
                0, self.e.window.style().standardIcon(self.e.qt.QStyle.SP_DirIcon)
            )
            links[name] = item
            self.items[name] = item
        self.tree.addTopLevelItem(links[names[0]])
        for joint in root.findall("joint"):
            name = joint.get("name")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            item = self.e.qt.QTreeWidgetItem([name, joint.get("type")])
            item.setData(0, 256, "joint:" + name)
            item.setIcon(
                0,
                self.e.window.style().standardIcon(
                    self.e.qt.QStyle.SP_ArrowRight
                    if joint.get("type") == "prismatic"
                    else (
                        self.e.qt.QStyle.SP_BrowserReload
                        if joint.get("type") != "fixed"
                        else self.e.qt.QStyle.SP_FileLinkIcon
                    )
                ),
            )
            links[parent].addChild(item)
            item.addChild(links[child])
            self.items["joint:" + name] = item
        for key, record in self.records.items():
            name, kind, index = key.rsplit("|", 2)
            item = self.e.qt.QTreeWidgetItem(
                [f"{kind} {int(index)+1}", record["status"]]
            )
            item.setData(0, 256, key)
            item.setToolTip(1, record.get("message", "") or record["resolved"])
            item.setCheckState(
                0,
                (
                    self.e.core.Qt.Checked
                    if self.appearance.get(key, {}).get("visible", True)
                    else self.e.core.Qt.Unchecked
                ),
            )
            links[name].addChild(item)
            self.items[key] = item
        self.tree.expandToDepth(2)
        self.tree.blockSignals(False)
        if selected in self.items:
            self.tree.setCurrentItem(self.items[selected])
        elif names:
            self.tree.setCurrentItem(links[names[0]])
        self.filter(self.search.text())

    def toggle_item(self, item, column):
        if column != 0:
            return
        key = item.data(0, 256)
        if not key or key.startswith("joint:"):
            return
        before = self.snapshot()
        self.appearance.setdefault(key, {})["visible"] = (
            item.checkState(0) == self.e.core.Qt.Checked
        )
        self.changed(before)

    def filter(self, text):
        def visit(item):
            children = [visit(item.child(i)) for i in range(item.childCount())]
            key = item.data(0, 256)
            mode = self.filter_mode.currentText()
            kind = (
                "Geometry"
                if "|" in key
                else "Joints" if key.startswith("joint:") else "Links"
            )
            eligible = mode == "All components" or mode == kind
            if mode == "Hidden":
                eligible = not self.appearance.get(key, {}).get(
                    "visible", True
                ) or not self.appearance.get(self.link_name(key), {}).get(
                    "visible", True
                )
            if mode == "Asset issues":
                eligible = key in self.records and self.records[key]["status"] not in (
                    "loaded",
                    "primitive geometry",
                )
            match = (
                eligible
                and text.lower() in (item.text(0) + item.text(1) + str(key)).lower()
            )
            item.setHidden(not (match or any(children)))
            return match or any(children)

        for i in range(self.tree.topLevelItemCount()):
            visit(self.tree.topLevelItem(i))

    def select_subtree(self):
        def descend(item):
            item.setSelected(True)
            for i in range(item.childCount()):
                descend(item.child(i))

        current = self.tree.currentItem()
        if current is not None:
            descend(current)

    def link_name(self, key):
        if key.startswith("joint:"):
            return next(
                j.find("child").get("link")
                for j in self.root.findall("joint")
                if j.get("name") == key[6:]
            )
        return key.split("|")[0]

    def selected_links(self):
        return {self.link_name(item.data(0, 256)) for item in self.tree.selectedItems()}

    def selected_targets(self):
        return {
            (
                item.data(0, 256)
                if "|" in item.data(0, 256)
                else self.link_name(item.data(0, 256))
            )
            for item in self.tree.selectedItems()
        }

    def select(self):
        item = self.tree.currentItem()
        if item is None:
            return
        self.selected = item.data(0, 256)
        name = self.link_name(self.selected)
        self.e.links.setCurrentRow(self.names.index(name))
        crumbs: list[str] = []
        current = item
        while current is not None:
            crumbs.insert(0, current.text(0))
            current = current.parent()
        self.crumb.setText(" → ".join(crumbs))
        appearance = self.appearance.get(
            self.selected if "|" in self.selected else name, {}
        )
        self.alias.setText(appearance.get("alias", ""))
        self.notes.setText(appearance.get("notes", ""))
        self.alpha.setValue(appearance.get("opacity", 1))
        self.visible.setChecked(appearance.get("visible", True))
        self.style.setCurrentText(appearance.get("style", "solid"))
        self.color = appearance.get("color", "")
        frame = appearance.get("frame", {})
        self.axis_size.setValue(frame.get("axis_size", 0.055))
        self.label_size.setValue(frame.get("label_size", 13))
        self.frame_visible.setChecked(frame.get("visible", True))
        self.marker_size.setValue(frame.get("marker_size", 0.08))
        self.marker_alpha.setValue(frame.get("marker_alpha", 1))
        self.marker_color = frame.get("marker_color", "#f39c32")
        key = (
            self.selected
            if self.selected in self.records
            else next((k for k in self.records if k.startswith(name + "|visual|")), "")
        )
        self.active_geometry = key
        record = self.records.get(key, {})
        self.path.setText(record.get("resolved", ""))
        self.path.setToolTip(record.get("resolved", ""))
        self.asset_info.setText(
            f"{record.get('status','No geometry')} ({record.get('format','')})\nDimensions [m]: {record.get('dimensions_m','unavailable')}\nSource RGBA: {record.get('source_rgba','unavailable')}\nOriginal: {record.get('original','')}\nEffective scale: {record.get('effective_scale','')}\n{record.get('message','')}"
        )
        values = self.overrides.get(key, {})
        node = self.nodes.get(key)
        mesh = node.find("geometry/mesh") if node is not None else None
        self.scale.setText(
            values.get(
                "scale", mesh.get("scale", "1 1 1") if mesh is not None else "1 1 1"
            )
        )
        self.xyz.setText(record.get("xyz", "0 0 0"))
        self.rpy.setText(record.get("rpy", "0 0 0"))
        self.units.setCurrentText(values.get("units", "URDF / format default"))
        self.joint_index = next(
            (
                i
                for i, j in enumerate(
                    j
                    for j in self.e.application.run.chain.joints
                    if j.joint_type.value != "fixed"
                )
                if j.child_link == name
            ),
            None,
        )
        self.sync_joint()
        if self.selected.startswith("joint:"):
            source = next(
                n
                for n in self.root.findall("joint")
                if n.get("name") == self.selected[6:]
            )
        else:
            source = next(n for n in self.root.findall("link") if n.get("name") == name)
        inertial = source.find("inertial")
        self.source.setPlainText(
            (
                "Source inertial data: unavailable\n"
                if inertial is None and source.tag == "link"
                else "Read-only source data\n"
            )
            + tostring(source, encoding="unicode")
        )
        self.e.queue_render()

    def sync_joint(self):
        self.joint_value.blockSignals(True)
        self.joint_value.setEnabled(self.joint_index is not None)
        if self.joint_index is not None:
            index = self.joint_index
            if index < len(self.e.joint_controls):
                self.joint_value.setRange(*self.e.joint_controls[index][2:])
                self.joint_value.setValue(self.e.q[index])
                joint = [
                    j
                    for j in self.e.application.run.chain.joints
                    if j.joint_type.value != "fixed"
                ][index]
                self.joint_value.setSuffix(
                    " m" if joint.joint_type.value == "prismatic" else " rad"
                )
        self.joint_value.blockSignals(False)

    def picked(self, actor):
        key = self.actors.get(id(actor), {}).get("key")
        if key in self.items:
            self.tree.setCurrentItem(self.items[key])

    def pick_color(self):
        color = self.e.qt.QColorDialog.getColor(parent=self.e.window)
        if color.isValid():
            self.color = color.name()

    def apply_appearance(self):
        before = self.snapshot()
        for name in self.selected_targets():
            self.appearance.setdefault(name, {}).update(
                alias=self.alias.text(),
                notes=self.notes.text(),
                opacity=self.alpha.value(),
                visible=self.visible.isChecked(),
                style=self.style.currentText(),
                color=self.color,
            )
        self.changed(before)
        self.scene_built(self.names, self.root)

    def reset_appearance(self):
        before = self.snapshot()
        for name in self.selected_targets():
            self.appearance.pop(name, None)
        self.changed(before)
        self.select()

    def browse(self):
        if not self.active_geometry:
            if not self.selected:
                raise ValueError("Select a link first")
            self.active_geometry = self.link_name(self.selected) + "|visual|0"
        path, _ = self.e.qt.QFileDialog.getOpenFileName(
            self.e.window,
            "Locate / assign / replace geometry",
            self.e.settings.value("mesh_directory", ""),
            "Meshes (*.stl *.STL *.obj *.ply *.vtp *.vtk *.dae)",
        )
        if path:
            self.e.settings.setValue("mesh_directory", str(Path(path).parent))
            before = self.snapshot()
            self.overrides.setdefault(self.active_geometry, {})["path"] = path
            self.changed(before)
            self.e.build_scene()
            self.e.queue_render()

    def copy_path(self):
        self.e.qt.QApplication.clipboard().setText(self.path.text())

    def open_folder(self):
        if self.path.text():
            import_module("PySide6.QtGui").QDesktopServices.openUrl(
                self.e.core.QUrl.fromLocalFile(str(Path(self.path.text()).parent))
            )

    def preview(self):
        if not self.active_geometry:
            raise ValueError("Select geometry first")
        numbers(self.scale.text(), 3, True)
        numbers(self.xyz.text(), 3)
        numbers(self.rpy.text(), 3)
        if self.preview_before is None:
            self.preview_before = self.snapshot()
        self.overrides.setdefault(self.active_geometry, {}).update(
            scale=self.scale.text(),
            xyz=self.xyz.text(),
            rpy=self.rpy.text(),
            units=self.units.currentText(),
        )
        self.e.build_scene()
        self.e.queue_render()

    def apply_preview(self):
        if self.preview_before is not None:
            self.changed(self.preview_before)
            self.preview_before = None

    def cancel_preview(self):
        if self.preview_before is not None:
            self.overrides, self.appearance = self.preview_before
            self.preview_before = None
            self.e.build_scene()
            self.e.queue_render()

    def focus(self):
        selected = self.selected_links()
        actors = [
            a
            for a, _, _, _ in self.e.mesh_actors
            if self.actors.get(id(a), {}).get("link") in selected
        ]
        if actors:
            bounds = self.e.np.asarray([a.bounds for a in actors])
            self.e.view.reset_camera(
                bounds=[
                    bounds[:, i].min() if i % 2 == 0 else bounds[:, i].max()
                    for i in range(6)
                ]
            )

    def isolate(self):
        before = self.snapshot()
        selected = self.selected_links()
        for name in self.names:
            self.appearance.setdefault(name, {})["visible"] = name in selected
        self.changed(before)

    def show_all(self):
        before = self.snapshot()
        for name in set(self.names) | set(self.appearance):
            self.appearance.setdefault(name, {})["visible"] = True
        self.changed(before)
        self.scene_built(self.names, self.root)

    def update(self):
        selected = self.selected_links()
        for actor, _, _, kind in self.e.mesh_actors:
            record = self.actors.get(id(actor))
            if record is None:
                continue
            values = self.appearance.get(record["link"], {})
            actor.visibility = actor.visibility and values.get("visible", True)
            element = self.appearance.get(record["key"], {})
            actor.visibility = actor.visibility and element.get("visible", True)
            values = {**values, **element}
            actor.prop.opacity *= values.get("opacity", 1) * record["rgba"][3]
            actor.prop.color = values.get("color") or record["rgba"][:3]
            if record.get("texture") is not None:
                actor.texture = None if values.get("color") else record["texture"]
            if "source_rgba" in actor.mapper.dataset.point_data:
                actor.mapper.scalar_visibility = record.get(
                    "use_scalars", False
                ) and not bool(values.get("color"))
            actor.prop.style = (
                "wireframe" if values.get("style") == "wireframe" else "surface"
            )
            actor.prop.show_edges = (
                values.get("style") == "edges" or record["link"] in selected
            )

    def health(self):
        qt = self.e.qt
        dialog = qt.QDialog(self.e.window)
        dialog.setWindowTitle("Robot health — double-click an asset to inspect")
        dialog.resize(700, 450)
        layout = qt.QVBoxLayout(dialog)
        items = qt.QListWidget()
        layout.addWidget(items)
        for key, record in self.records.items():
            item = qt.QListWidgetItem(
                f"{key}: {record['status']} {record.get('message','')}"
            )
            item.setData(256, key)
            items.addItem(item)
        label = qt.QLabel(
            self.e.validation_label.text()
            + "\nOnly the selected DH path is certified; inactive branches remain at zero pose."
        )
        label.setWordWrap(True)
        layout.addWidget(label)

        def focus(item):
            self.tree.setCurrentItem(self.items[item.data(256)])
            self.focus()
            dialog.accept()

        items.itemDoubleClicked.connect(focus)
        dialog.exec()

    def export_report(self):
        from urdf2dt.visualization.assets import document_poses

        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Component report", "component.json", "JSON (*.json)"
        )
        if path:
            Path(path).write_text(
                json.dumps(
                    {
                        "selection": self.selected,
                        "geometry": self.records.get(self.active_geometry),
                        "appearance": self.appearance.get(
                            self.link_name(self.selected), {}
                        ),
                        "source": self.source.toPlainText(),
                        "pose": self.e.q,
                        "link_world_transform": document_poses(
                            self.root,
                            dict(
                                zip(self.e.application.run.chain.joint_names, self.e.q)
                            ),
                        )[self.link_name(self.selected)],
                        "validation": self.e.validation_label.text(),
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            self.e.view.screenshot(str(Path(path).with_suffix(".png")))
