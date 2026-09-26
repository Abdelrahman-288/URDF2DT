"""Presentation-only workspace organization and engineering display preferences."""
from math import pi
from pathlib import Path
from urdf2dt.ui.analysis_panel import AnalysisPanel, table, fill
from urdf2dt.ui.number_display import decimal


class Workspace:
    def __init__(self, e, splitter, left_layout, center_layout, toolbar):
        self.e, self.splitter = e, splitter
        splitter.widget(0).setMinimumWidth(290)
        self.grid_actor = None
        self.validated_model = None
        self.digits = int(e.settings.value('display_digits', 3))
        self.angular = str(e.settings.value('angle_unit', 'rad'))
        self.linear = str(e.settings.value('length_unit', 'm'))
        qt = e.qt
        inspector = qt.QWidget()
        layout = qt.QVBoxLayout(inspector)
        for widget in (e.studio.crumb, e.studio.properties):
            left_layout.removeWidget(widget)
            layout.addWidget(widget)
        e.studio.properties.setMaximumHeight(16777215)
        for i in reversed(range(left_layout.count())):
            widget = left_layout.itemAt(i).widget()
            if isinstance(widget, qt.QPushButton) or widget is e.opacity or (isinstance(widget, qt.QLabel) and widget.text() == 'Opacity'):
                left_layout.removeWidget(widget)
                layout.addWidget(widget)
        self.inspector_index = e.tabs.addTab(inspector, 'Model & Appearance')
        e.studio.tree.setSizePolicy(qt.QSizePolicy.Expanding, qt.QSizePolicy.Expanding)
        self.context = qt.QLabel('Whole robot shown • choose an active serial chain above')
        self.context.setWordWrap(True)
        left_layout.insertWidget(2, self.context)
        self.analysis = AnalysisPanel(e)
        for i in range(e.tool_selector.count(), e.tabs.count()):
            e.tool_selector.addItem(e.tabs.tabText(i))
        self.vertical = qt.QSplitter(e.core.Qt.Vertical)
        center_layout.removeWidget(e.view.interactor)
        center_layout.removeWidget(e.table)
        viewport = qt.QWidget()
        vl = qt.QVBoxLayout(viewport)
        vl.setContentsMargins(0, 0, 0, 0)
        cameras = qt.QHBoxLayout()
        vl.addLayout(cameras)
        for name, callback in [('Fit robot', self.fit_robot), ('Fit selection', e.studio.focus),
                               ('Front', e.view.view_xz), ('Side', e.view.view_yz),
                               ('Top', e.view.view_xy), ('Isometric', e.view.view_isometric)]:
            e._button(cameras, name, callback)
        vl.addWidget(e.view.interactor, 1)
        self.vertical.addWidget(viewport)
        self.results = qt.QTabWidget()
        dhpage = qt.QWidget()
        dl = qt.QVBoxLayout(dhpage)
        self.dh_context = qt.QLabel()
        self.dh_context.setWordWrap(True)
        dl.addWidget(self.dh_context)
        e.table.setMaximumHeight(16777215)
        e.table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.ResizeToContents)
        e.table.horizontalHeader().setStretchLastSection(True)
        e.table.setColumnHidden(7, True)
        e.table.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        e.table.cellClicked.connect(self.select_dh_row)
        dl.addWidget(e.table)
        e._button(dl, 'Copy displayed DH table (with units)', self.copy_dh)
        self.results.addTab(dhpage, 'DH parameters')
        self.fk_table = table(e, 4, 4)
        self.results.addTab(self.fk_table, 'FK transform')
        from urdf2dt.ui.validation_panel import ValidationPanel
        self.validation = ValidationPanel(e)
        self.results.addTab(self.validation.page, 'Validation')
        self.vertical.addWidget(self.results)
        self.vertical.setSizes([450, 300])
        center_layout.addWidget(self.vertical)
        for action in e.window.menuBar().actions():
            if action.text() == 'Project':
                action.setText('File')
                e._action(action.menu(), 'Open URDF...', e.open_dialog)
                include = action.menu().addAction('Include local assets when saving projects')
                include.setCheckable(True)
                include.setChecked(e.project_panel.include.isChecked())
                include.toggled.connect(e.project_panel.include.setChecked)
                e.project_panel.include.toggled.connect(include.setChecked)
            elif action.text() == 'Structure':
                action.setText('Edit')
        e.project_panel.include.hide()
        view = e.window.menuBar().addMenu('View')
        e._action(view, 'Display units and precision…', self.preferences)
        e._action(view, 'Reset workspace layout', self.reset)
        self.grid_action = view.addAction('Ground grid')
        self.grid_action.setCheckable(True)
        self.grid_action.toggled.connect(lambda _: e.queue_render())
        e._action(view, 'Show / hide results', lambda: self.results.setVisible(not self.results.isVisible()))
        e._action(view, 'Advanced DH sign column', lambda: e.table.setColumnHidden(7, not e.table.isColumnHidden(7)))
        themes = view.addMenu('Theme')
        for name in ('Light','Dark'):
            e._action(themes, name, lambda n=name: e.theme.setCurrentText(n))
        # Retain the existing theme widget API for session/verification compatibility.
        for action in toolbar.actions():
            if toolbar.widgetForAction(action) is e.theme:
                action.setVisible(False)
        layers = view.addMenu('Scene layers')
        for widget in e.layers.values():
            left_layout.removeWidget(widget)
            widget.hide()
            action = layers.addAction(widget.text())
            action.setCheckable(True)
            action.setChecked(widget.isChecked())
            action.toggled.connect(widget.setChecked)
            widget.toggled.connect(action.setChecked)
        tools = e.window.menuBar().addMenu('Tools')
        for i in range(e.tabs.count()):
            e._action(tools, e.tabs.tabText(i), lambda index=i: e.tabs.setCurrentIndex(index))
        e._action(tools, 'Inspect selected component', lambda: e.tabs.setCurrentIndex(self.inspector_index))
        e.studio.tree.setContextMenuPolicy(e.core.Qt.CustomContextMenu)
        e.studio.tree.customContextMenuRequested.connect(self.context_menu)
        help_menu = e.window.menuBar().addMenu('Help')
        e._action(help_menu, 'Workspace guide', lambda: qt.QMessageBox.information(e.window, 'Workspace guide',
            'Open URDF loads a complete robot. The active chain determines controlled joints and DH results.\n\n'
            'Click a body to select its link; click empty background to clear selection. Drag the background to orbit.\n\n'
            'Choose a right-side tool beside Fit view. Model & Appearance contains the selected-part inspector.\n\n'
            'View controls display units, precision, themes and scene layers. Drag panel dividers to resize.\n\n'
            'Jacobian analysis uses the same physical reference, target and model as FK. J*qdot changes displayed velocity, not pose.\n\n'
            'F11 toggles presentation mode. Ctrl+S saves the project. Numerical exports retain full precision in SI.'))
        self.file_label = qt.QLabel()
        e.window.statusBar().addWidget(self.file_label)
        e.fk_panel.units.currentIndexChanged.connect(self.units_changed)
        e._action(help_menu, 'Copy current robot path', lambda: qt.QApplication.clipboard().setText(str(e.application.run.chain.source_urdf or '') if e.application else ''))
        self.restore()
        qt.QApplication.instance().aboutToQuit.connect(self.save)

    def select_dh_row(self, row, column):
        e = self.e
        if not e.application:
            return
        movable = [j for j in e.application.run.chain.joints if j.joint_type.value != 'fixed']
        if row < len(movable):
            item = e.studio.items.get(movable[row].child_link)
            if item is not None:
                e.studio.tree.setCurrentItem(item)
                e.studio.tree.scrollToItem(item)

    def fit_robot(self):
        e = self.e
        actors = [a for a,_,_,_ in e.mesh_actors if a.visibility]
        if not actors:
            e.view.reset_camera()
            return
        bounds = e.np.asarray([a.bounds for a in actors])
        e.view.reset_camera(bounds=[bounds[:,i].min() if i%2 == 0 else bounds[:,i].max() for i in range(6)])

    def units_changed(self, index):
        self.angular = "deg" if index % 2 else "rad"
        self.linear = "mm" if index >= 2 else "m"
        self.e.queue_render()

    def context_menu(self, point):
        e = self.e
        menu = e.qt.QMenu(e.studio.tree)
        for name, callback in [('Inspect', lambda: e.tabs.setCurrentIndex(self.inspector_index)),
                               ('Focus', e.studio.focus), ('Isolate / restore visibility', e.studio.isolate),
                               ('Show all', e.studio.show_all)]:
            e._action(menu, name, callback)
        menu.exec(e.studio.tree.viewport().mapToGlobal(point))

    def reset(self):
        self.splitter.setSizes([290, 800, 400])
        self.vertical.setSizes([450, 300])
        self.results.show()

    def save(self):
        e = self.e
        for key, value in [('workspace_split', self.splitter.saveState()), ('results_split', self.vertical.saveState()),
                           ('display_digits', self.digits), ('angle_unit', self.angular), ('length_unit', self.linear)]:
            e.settings.setValue(key, value)

    def restore(self):
        for key, widget in [('workspace_split', self.splitter), ('results_split', self.vertical)]:
            state = self.e.settings.value(key)
            if state is not None:
                widget.restoreState(state)

    def preferences(self):
        qt = self.e.qt
        dialog = qt.QDialog(self.e.window)
        dialog.setWindowTitle('Display preferences — model remains full-precision SI')
        layout = qt.QFormLayout(dialog)
        angular, linear, precision = qt.QComboBox(), qt.QComboBox(), qt.QSpinBox()
        angular.addItems(['rad','deg'])
        linear.addItems(['m','mm'])
        angular.setCurrentText(self.angular)
        linear.setCurrentText(self.linear)
        precision.setRange(1, 9)
        precision.setValue(self.digits)
        layout.addRow('Joint / DH angle display', angular)
        layout.addRow('Joint / DH length display', linear)
        layout.addRow('Table / joint decimal places', precision)
        layout.addRow(qt.QLabel('Models and exports retain full precision in SI.\nGeometric Jacobian entries and rates use explicit SI units.'))
        buttons = qt.QDialogButtonBox(qt.QDialogButtonBox.Ok | qt.QDialogButtonBox.Cancel)
        layout.addRow(buttons)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        if dialog.exec():
            self.angular, self.linear, self.digits = angular.currentText(), linear.currentText(), precision.value()
            self.save()
            self.sync()

    def copy_dh(self):
        t = self.e.table
        columns = [i for i in range(t.columnCount()) if not t.isColumnHidden(i)]
        rows = [[t.horizontalHeaderItem(i).text() for i in columns]]
        rows += [[t.item(r,c).text() for c in columns] for r in range(t.rowCount())]
        self.e.qt.QApplication.clipboard().setText('\n'.join('\t'.join(row) for row in rows))

    def sync(self):
        e = self.e
        if not e.application or getattr(e, '_loading_robot', False):
            return
        chain = e.application.run.chain
        source = str(chain.source_urdf or '')
        self.file_label.setText(Path(source).name)
        self.file_label.setToolTip(source)
        status = e.window.statusBar().currentMessage()
        if source and source in status:
            e.window.statusBar().showMessage(status.replace(source, Path(source).name))
        self.context.setText(f'Active chain: {chain.base_link} → {chain.joints[-1].child_link if chain.joints else chain.base_link}\n'
                             f'{len(chain.joint_names)} movable joints controlled. Other branches remain visible at their default pose.')
        active = {chain.base_link, *(j.child_link for j in chain.joints)}
        e.studio.tree.blockSignals(True)
        for name, item in e.studio.items.items():
            if name in e.studio.names:
                font = item.font(0)
                font.setBold(name in active)
                item.setFont(0, font)
                item.setToolTip(0, 'Active serial chain' if name in active else 'Other branch: visible at default pose')
        e.studio.tree.blockSignals(False)
        pending = e.application.session.pending is not None
        if self.validated_model is not None and (self.validated_model != e.application.session.state.working_model or pending):
            e.validation_label.setText('STALE: model or preview changed after validation. Run global FK validation again.')
        self.validation.sync()
        selected = e.studio.selected_links()
        matching = next((i for i,j in enumerate(j for j in chain.joints if j.joint_type.value != 'fixed') if j.child_link in selected), None)
        if matching is None:
            e.table.clearSelection()
        else:
            e.table.selectRow(matching)
        self.dh_context.setText(('PENDING PREVIEW — not committed' if pending else 'Committed DH model') +
                               ' • bold = changed from automatic • hover for full-precision SI')
        e.motion_label.setText(f"Joint pose — live motion ({self.angular} / {self.linear})")
        unit_index = (1 if self.angular == 'deg' else 0) + (2 if self.linear == 'mm' else 0)
        if e.fk_panel.units.currentIndex() != unit_index:
            e.fk_panel.units.setCurrentIndex(unit_index)
        af, lf = (180/pi if self.angular == 'deg' else 1), (1000 if self.linear == 'mm' else 1)
        movable = [j for j in chain.joints if j.joint_type.value != 'fixed']
        for i, (joint, (_, spin, low, high)) in enumerate(zip(movable, e.joint_controls)):
            factor = lf if joint.joint_type.value == 'prismatic' else af
            unit = self.linear if joint.joint_type.value == 'prismatic' else self.angular
            spin.set_display(factor, self.digits)
            spin.setSingleStep((1.0 if unit in ('deg','mm') else .01) / factor)
            label = e.joint_panel.findChild(e.qt.QLabel, f'pose-label-{i}')
            limits = e.joint_panel.findChild(e.qt.QLabel, f'pose-limits-{i}')
            if label:
                label.setText(f'{joint.name} ({joint.joint_type.value}) [{unit}]')
            if limits:
                limits.setText(f'{decimal(low*factor,self.digits)} to {decimal(high*factor,self.digits)} {unit}' +
                    (' - viewing range' if joint.limit is None else ' - URDF limits'))
        e.table.setHorizontalHeaderLabels(['Frame','Joint','State',f'a [{self.linear}]',f'α [{self.angular}]',f'd [{self.linear}]',f'θ offset [{self.angular}]','q sign'])
        for r, row in enumerate(e.current_model().rows):
            for c, value, factor in ((3,row.a,lf),(4,row.alpha,af),(5,row.d,lf),(6,row.theta_offset,af)):
                item = e.table.item(r,c)
                if item:
                    item.setText(decimal(value*factor, self.digits))
                    item.setTextAlignment(e.core.Qt.AlignRight | e.core.Qt.AlignVCenter)
                    item.setToolTip(f'Full-precision SI: {value!r}')
        if e.fk_panel.result:
            matrix = [[v*lf if c == 3 and r < 3 else v for c,v in enumerate(row)] for r,row in enumerate(e.fk_panel.result['matrix'])]
            fill(e, self.fk_table, matrix, ['x','y','z','h'], ['R·x','R·y','R·z',f'p [{self.linear}]'], self.digits)
            self.fk_table.setToolTip(f"{e.fk_panel.result.get('model', 'URDF')}: {e.fk_panel.result['target']} relative to {e.fk_panel.result['reference']}; full precision in FK JSON")
        else:
            self.fk_table.setRowCount(0)
        if self.grid_action.isChecked() and self.grid_actor is None:
            actors = [a for a,_,_,kind in e.mesh_actors if kind == 'visual' and a.visibility]
            if actors:
                bounds = e.np.asarray([a.bounds for a in actors])
                low = bounds[:,[0,2,4]].min(axis=0)
                high = bounds[:,[1,3,5]].max(axis=0)
                size = max(float(max(high-low))*1.3, .1)
                center = ((low[0]+high[0])/2, (low[1]+high[1])/2, low[2]-.001)
                mesh = e.pv.Plane(center=center, i_size=size, j_size=size, i_resolution=20, j_resolution=20)
                self.grid_actor = e.view.add_mesh(mesh, style='wireframe', color='#788596', opacity=.3,
                    lighting=False, pickable=False, reset_camera=False, render=False)
        if self.grid_actor is not None:
            self.grid_actor.visibility = self.grid_action.isChecked()
        if e.tabs.currentIndex() == self.analysis.index:
            self.analysis.refresh()
