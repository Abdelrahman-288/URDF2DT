"""Local start screen, recent files and an optional non-destructive walkthrough."""
import json
import sys
from pathlib import Path
from importlib import import_module


STEPS = (
    ('Load', 'Open a URDF with its mesh folders nearby, or reopen a saved project. '
     'The active chain determines the joints and DH model. Missing meshes are reported separately.'),
    ('Inspect', 'Click a robot body or a component in Robot Structure. A box marks the selection. '
     'Use Model & Appearance to inspect geometry and missing files; click empty space to deselect.'),
    ('Move', 'Drag a joint slider or enter a value. Watch the body move immediately. '
     'Reset pose returns joints to zero within their limits; motion does not edit DH geometry.'),
    ('Calculate', 'Choose the reference and target in FK calculator, then Calculate / refresh. '
     'Position is expressed in the reference frame. Symbolic FK and Jacobian are separate tools.'),
    ('Validate', 'For global FK validation, review and accept the DH frames in order using DH editor. '
     'Then run validation. Read the result; visiting this step does not mean validation passed.'),
    ('Save', 'Save Project stores your working project. Choose an empty folder for a new project '
     'and include local assets for portability. Export Portable Project ZIP when sharing. '
     'Recovery drafts do not replace an explicit save.'),
)


def read_recent(settings):
    try:
        data = json.loads(str(settings.value('recent_files_v1', '[]')))
        if not isinstance(data, list):
            return []
        return [x for x in data if isinstance(x, dict) and x.get('kind') in ('URDF', 'Project')
                and isinstance(x.get('path'), str)][:12]
    except (ValueError, TypeError):
        return []


class Welcome:
    def __init__(self, e):
        self.e = e
        qt = e.qt
        self.step = 0
        self.active = False
        self.stack = qt.QStackedWidget()
        self.editor = e.window.takeCentralWidget()
        self.stack.addWidget(self.editor)
        self.page = qt.QWidget()
        outer = qt.QVBoxLayout(self.page)
        outer.setContentsMargins(50, 30, 50, 30)
        title = qt.QLabel('Welcome to URDF2DT')
        title.setStyleSheet('font-size:28px;font-weight:600;')
        outer.addWidget(title)
        subtitle = qt.QLabel('Load a robot, explore its motion, and create a validated DH model.')
        subtitle.setWordWrap(True)
        outer.addWidget(subtitle)
        row = qt.QHBoxLayout()
        for label, callback in [('Open URDF…', e.open_dialog), ('Open project…', e.project_panel.open),
                                ('User manual', self.manual), ('Start walkthrough', self.start)]:
            e._button(row, label, callback)
        outer.addLayout(row)
        outer.addWidget(qt.QLabel('Recent projects and URDF files — stored only on this computer'))
        self.recent = qt.QListWidget()
        self.recent.setMaximumHeight(300)
        self.recent.setAccessibleName('Recent projects and robot files')
        outer.addWidget(self.recent, 1)
        self.recent.itemDoubleClicked.connect(lambda _: e.guard(self.open_recent))
        controls = qt.QHBoxLayout()
        self.open_button = e._button(controls, 'Open selected', self.open_recent)
        e._button(controls, 'Remove selected from history', self.remove_recent)
        self.resume = e._button(controls, 'Return to workspace', self.show_editor)
        outer.addLayout(controls)
        self.message = qt.QLabel()
        self.message.setWordWrap(True)
        outer.addWidget(self.message)
        outer.addSpacing(20)
        overview = qt.QLabel('Your first robot, in six steps')
        overview.setStyleSheet('font-size:20px;font-weight:600;')
        outer.addWidget(overview)
        for i, (name, detail) in enumerate(STEPS):
            label = qt.QLabel(f'{i + 1}. {name} — {detail.split(". ")[0]}.')
            label.setWordWrap(True)
            outer.addWidget(label)
        outer.addStretch()
        self.stack.addWidget(self.page)
        e.window.setCentralWidget(self.stack)
        self.dock = qt.QDockWidget('Getting started', e.window)
        body = qt.QWidget()
        layout = qt.QVBoxLayout(body)
        self.heading = qt.QLabel()
        self.description = qt.QLabel()
        self.description.setWordWrap(True)
        layout.addWidget(self.heading)
        layout.addWidget(self.description)
        nav = qt.QHBoxLayout()
        self.back = e._button(nav, 'Back', lambda: self.advance(-1))
        self.action = e._button(nav, 'Open step tool', self.open_step)
        self.next = e._button(nav, 'Next', lambda: self.advance(1))
        e._button(nav, 'Close walkthrough', self.finish)
        layout.addLayout(nav)
        self.dock.setWidget(body)
        e.window.addDockWidget(e.core.Qt.BottomDockWidgetArea, self.dock)
        self.dock.hide()
        help_menu = next(a.menu() for a in e.window.menuBar().actions() if a.text() == 'Help')
        e._action(help_menu, 'Start screen', self.show)
        e._action(help_menu, 'Getting started walkthrough', self.start)
        e._action(help_menu, 'User manual', self.manual)
        self.refresh()

    def refresh(self):
        self.recent.clear()
        for record in read_recent(self.e.settings):
            path = Path(record['path'])
            missing = '' if path.is_file() else ' — unavailable'
            item = self.e.qt.QListWidgetItem(f"{record['kind']}: {path.name}{missing}\n{path}")
            item.setData(256, record)
            self.recent.addItem(item)
        self.message.setText('Select a recent item and choose Open selected, or double-click it.'
                             if self.recent.count() else 'No recent projects yet. Choose Open URDF to begin.')
        self.open_button.setEnabled(bool(self.recent.count()))
        self.resume.setEnabled(self.e.application is not None)

    def remember(self, path, kind):
        path = str(Path(path).resolve())
        records = [r for r in read_recent(self.e.settings) if r['path'].casefold() != path.casefold()]
        self.e.settings.setValue('recent_files_v1', json.dumps([{'path': path, 'kind': kind}] + records[:11]))
        self.refresh()
        self.show_editor()
        if self.active and self.step == 0:
            self.step = 1
            self.render_step()

    def show(self):
        self.refresh()
        self.dock.hide()
        self.stack.setCurrentWidget(self.page)

    def show_editor(self):
        self.stack.setCurrentWidget(self.editor)
        if self.active:
            self.dock.show()

    def open_recent(self):
        item = self.recent.currentItem()
        if item is None:
            self.message.setText('Select a recent item first.')
            return
        record = item.data(256)
        if not Path(record['path']).is_file():
            self.message.setText('This file moved or is unavailable. Use Open URDF or Open project to locate it. '
                                 'Your current robot has not changed.')
            return
        if record['kind'] == 'Project':
            self.e.project_panel.open_path(record['path'])
        elif self.e.project_panel.confirm_discard():
            self.e.load(record['path'])

    def remove_recent(self):
        item = self.recent.currentItem()
        if item:
            records = [r for r in read_recent(self.e.settings) if r != item.data(256)]
            self.e.settings.setValue('recent_files_v1', json.dumps(records))
            self.refresh()

    def start(self):
        self.active = True
        self.step = 1 if self.e.application else 0
        self.show_editor()
        self.render_step()
        self.dock.show()

    def finish(self):
        self.active = False
        self.dock.hide()

    def advance(self, amount):
        if amount > 0 and self.e.application is None:
            self.description.setText('Load a robot first using Open step tool. You can close the walkthrough at any time.')
            return
        if self.step == 5 and amount > 0:
            self.finish()
            return
        self.step = max(0, min(5, self.step + amount))
        self.render_step()

    def render_step(self):
        title, description = STEPS[self.step]
        self.heading.setText(f'{self.step + 1} / 6 — {title}   |   Load → Inspect → Move → Calculate → Validate → Save')
        self.description.setText(description)
        self.back.setEnabled(self.step > 0)
        self.next.setText('Finish walkthrough' if self.step == 5 else 'Next')

    def open_step(self):
        e = self.e
        if self.step == 0:
            e.open_dialog()
            return
        if e.application is None:
            return
        self.show_editor()
        if self.step == 5:
            e.project_panel.save()
            return
        name = {1: 'Model & Appearance', 2: 'Joint motion', 3: 'FK calculator', 4: 'DH editor'}[self.step]
        for i in range(e.tabs.count()):
            if e.tabs.tabText(i) == name:
                e.tabs.setCurrentIndex(i)
        if self.step == 4:
            e.workspace.results.show()
            e.workspace.results.setCurrentIndex(2)

    def manual(self):
        root = Path(__file__).resolve().parents[2]
        candidates = [root / 'User Manual/URDF2DT_User_Manual.pdf']
        if getattr(sys, 'frozen', False):
            home = Path(sys.executable).parent
            candidates = [home / 'User Manual/URDF2DT_User_Manual.pdf',
                          home.parent / 'User Manual/URDF2DT_User_Manual.pdf']
        path = next((p for p in candidates if p.is_file()), None)
        if path is None:
            self.e.qt.QMessageBox.information(self.e.window, 'User manual unavailable',
                'The PDF was not found. Keep the User Manual folder with the application. '
                'Help → Getting started walkthrough is available without the PDF.')
            return
        gui = import_module('PySide6.QtGui')
        if not gui.QDesktopServices.openUrl(self.e.core.QUrl.fromLocalFile(str(path))):
            raise RuntimeError(f'Could not open the PDF viewer. Open this file manually: {path}')
