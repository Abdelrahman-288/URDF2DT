"""Project actions and recoverable committed drafts for the native application."""

from datetime import datetime
from pathlib import Path
import shutil
import tempfile
import os
import json
from urdf2dt.projects import save_project, load_project, portable_zip


class ProjectPanel:
    def __init__(self, e):
        self.e = e
        self.folder = None
        self.draft_source = None
        self.last_draft = None
        self.recovery_error = None
        self.asset_note = ''
        menu = e.window.menuBar().addMenu("Project")
        for label, callback in (
            ("Open Project…", self.open),
            ("Save Project", self.save),
            ("Save Project As…", self.save_as),
            ("Export Portable Project ZIP…", self.zip),
            ("Create checkpoint…", self.checkpoint),
            ("Open recovery draft…", self.recover),
            ("Project and recovery status…", self.details),
            ("Presentation mode", self.presentation),
        ):
            e._action(menu, label, callback)
        e._action(menu, "Engineering report…", self.report)
        e._action(menu, "Batch URDF / project validation…", self.batch)
        e._action(menu, "Export geometry URDF with assets…", self.export_urdf)
        self.include = e.qt.QCheckBox("Include local assets when saving")
        self.include.setChecked(True)
        e.window.statusBar().addPermanentWidget(self.include)
        self.status = e.qt.QLabel('No project loaded')
        e.window.statusBar().addPermanentWidget(self.status)
        self.recovery = (
            Path(
                e.core.QStandardPaths.writableLocation(
                    e.core.QStandardPaths.AppLocalDataLocation
                )
            )
            / "recovery"
        )
        self.timer = e.core.QTimer(e.window)
        self.timer.setInterval(60000)
        self.timer.timeout.connect(self.autosave)
        self.timer.start()
        owner = self

        class CloseFilter(e.core.QObject):  # type: ignore[name-defined]
            def eventFilter(self, watched, event):
                if event.type() == e.core.QEvent.Close and not getattr(
                    e, "_closing_programmatically", False
                ):
                    if not owner.confirm_discard():
                        event.ignore()
                        return True
                return False

        self.close_filter = CloseFilter(e.window)
        e.window.installEventFilter(self.close_filter)
        gui = __import__("PySide6.QtGui", fromlist=["QShortcut"])
        for key, callback in (
            ("Ctrl+S", self.save),
            ("Ctrl+Shift+S", self.save_as),
            ("F11", self.presentation),
        ):
            shortcut = gui.QShortcut(gui.QKeySequence(key), e.window)
            shortcut.activated.connect(lambda cb=callback: e.guard(cb))

    def confirm_discard(self):
        if not self.e.application or not (
            self.e.studio.dirty
            or self.e.application.session.pending
            or self.e.studio.preview_before is not None
        ):
            return True
        qt = self.e.qt
        answer = qt.QMessageBox.question(
            self.e.window,
            "Unsaved project changes",
            "Save project changes before continuing?",
            qt.QMessageBox.Save | qt.QMessageBox.Discard | qt.QMessageBox.Cancel,
            qt.QMessageBox.Cancel,
        )
        if answer == qt.QMessageBox.Cancel:
            return False
        if answer == qt.QMessageBox.Save:
            self.save()
            return not self.e.studio.dirty and not self.e.application.session.pending and self.e.studio.preview_before is None
        return True

    def open(self):
        if not self.confirm_discard():
            return
        path, _ = self.e.qt.QFileDialog.getOpenFileName(
            self.e.window, "Open project", "", "URDF2DT project (project.json)"
        )
        if path:
            self.open_path(path, confirmed=True)

    def open_path(self, path, confirmed=False):
        if not confirmed and not self.confirm_discard():
            return
        folder = load_project(self.e, path)
        self.read_asset_status(folder)
        if (folder / 'RECOVERY_DRAFT.json').is_file() or folder.resolve().is_relative_to(self.recovery.resolve()):
            self.folder = None
            self.draft_source = folder
            self.e.studio.dirty = True
            self.e.welcome.show_editor()
        else:
            self.folder = folder
            self.draft_source = None
            self.e.welcome.remember(folder / 'project.json', 'Project')
        self.sync_status()

    def read_asset_status(self, folder):
        data = json.loads((folder / 'project.json').read_text(encoding='utf-8'))
        self.asset_note = ('assets unresolved' if data.get('unresolved') else
                           'external assets' if data.get('external_assets') else 'assets included')

    def save_as(self):
        qt = self.e.qt
        dialog = qt.QFileDialog(self.e.window, "Choose an empty project folder")
        dialog.setOption(qt.QFileDialog.DontUseNativeDialog, True)
        dialog.setFileMode(qt.QFileDialog.Directory)
        dialog.setOption(qt.QFileDialog.ShowDirsOnly, True)
        include = qt.QCheckBox("Include local mesh and texture assets")
        include.setChecked(self.include.isChecked())
        grid = dialog.layout()
        grid.addWidget(include, grid.rowCount(), 0, 1, grid.columnCount())
        if dialog.exec() != qt.QDialog.Accepted or not dialog.selectedFiles():
            return
        path = dialog.selectedFiles()[0]
        self.include.setChecked(include.isChecked())
        if path:
            self.folder = save_project(self.e, path, self.include.isChecked())
            self.read_asset_status(self.folder)
            self.e.studio.dirty = False
            self.draft_source = None
            self.e.window.statusBar().showMessage(f"Project saved: {self.folder}")
            self.e.welcome.remember(self.folder / 'project.json', 'Project')
            self.sync_status()

    def save(self):
        if self.folder is None:
            return self.save_as()
        with tempfile.TemporaryDirectory() as temporary:
            staged = save_project(
                self.e, Path(temporary) / "project", self.include.isChecked()
            )
            checkpoint = (
                self.folder
                / "checkpoints"
                / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            )
            checkpoint.mkdir(parents=True)
            for name in ("project.json", "source-original.urdf", "robot.urdf"):
                shutil.copyfile(self.folder / name, checkpoint / name)
            if (self.folder / "meshes").exists():
                shutil.copytree(self.folder / "meshes", checkpoint / "meshes")
            if (self.folder / "materials").exists():
                shutil.copytree(self.folder / "materials", checkpoint / "materials")
            # Preserve a complete last-good checkpoint before replacing any core file.
            try:
                for asset in staged.rglob("*"):
                    if asset.is_file() and asset.name != "project.json":
                        target = self.folder / asset.relative_to(staged)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        temporary_file = target.with_name(target.name + '.new')
                        shutil.copyfile(asset, temporary_file)
                        os.replace(temporary_file, target)
                temp = self.folder / "project.json.new"
                shutil.copyfile(staged / "project.json", temp)
                os.replace(temp, self.folder / "project.json")
            except Exception:
                # Restore metadata and original URDFs; content-addressed assets were retained.
                for name in ("source-original.urdf", "robot.urdf", "project.json"):
                    shutil.copyfile(checkpoint / name, self.folder / name)
                raise
        self.e.studio.dirty = False
        self.draft_source = None
        self.read_asset_status(self.folder)
        self.e.welcome.remember(self.folder / 'project.json', 'Project')
        self.sync_status()

    def zip(self):
        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Portable project ZIP", "RobotProject.zip", "ZIP (*.zip)"
        )
        if path:
            evidence = portable_zip(self.e, path)
            self.e.qt.QMessageBox.information(self.e.window, 'Portable project checked',
                f"Exported {evidence['asset_files']} required asset files. File checksums, dependency closure "
                'and extracted-project model reopening passed locally.\n\n'
                'This does not test a second computer. Send the ZIP and a compatible URDF2DT application. '
                'The recipient should extract everything and open RobotProject/project.json.\n\n'
                'Export does not save or clear unsaved changes in your working project.')

    def checkpoint(self):
        if self.folder is None:
            raise ValueError("Save a project before creating named checkpoints")
        name, ok = self.e.qt.QInputDialog.getText(
            self.e.window, "Checkpoint", "Name (letters, numbers, spaces)"
        )
        if ok and name.strip():
            if any(
                c
                not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_"
                for c in name
            ):
                raise ValueError("Use letters, numbers, spaces, - or _")
            save_project(self.e, self.folder / "checkpoints" / name, True)

    def autosave(self):
        if getattr(self.e, "_building_scene", False):
            return
        if self.e.application is None or not self.e.studio.dirty:
            return
        if (
            self.e.application.session.pending
            or self.e.studio.preview_before is not None
        ):
            return
        try:
            draft = save_project(
                self.e,
                self.recovery / datetime.now().strftime("%Y%m%d-%H%M%S-%f"),
                True,
            )
            data = json.loads((draft / 'project.json').read_text(encoding='utf-8'))
            (draft / 'RECOVERY_DRAFT.json').write_text(json.dumps({
                'kind': 'urdf2dt.recovery-draft', 'created_at': datetime.now().isoformat(),
                'saved_project': str(self.folder) if self.folder else None,
                'unresolved': data['unresolved'],
                'scope': 'Committed state only; pending previews excluded. Save As to keep recovered work.'
            }, indent=2), encoding='utf-8')
            self.last_draft = draft
            self.recovery_error = None
            self.e.window.statusBar().showMessage(
                "Recovery draft saved separately; explicit project save still required"
            )
        except Exception as exc:
            self.recovery_error = str(exc)
            self.e.window.statusBar().showMessage(
                f"Recovery draft could not be saved: {exc}"
            )
        self.sync_status()

    def recover(self):
        if not self.confirm_discard():
            return
        path, _ = self.e.qt.QFileDialog.getOpenFileName(
            self.e.window,
            "Open recovery draft",
            str(self.recovery),
            "Project (project.json)",
        )
        if path:
            folder = load_project(self.e, path)
            self.folder = None
            self.draft_source = folder
            self.read_asset_status(folder)
            self.e.studio.dirty = True
            self.e.welcome.show_editor()
            self.sync_status()

    def sync_status(self):
        e = self.e
        if e.application is None:
            self.status.setText('No project loaded')
            return
        pending = e.application.session.pending is not None or e.studio.preview_before is not None
        if self.draft_source:
            state = 'Recovered draft — Save As required'
        elif self.folder:
            state = 'Unsaved changes' if e.studio.dirty else 'Project saved'
            state += ': ' + self.folder.name
        else:
            state = 'Unsaved project' if e.studio.dirty else 'URDF open — no saved project'
        if pending:
            state += ' • preview not saved'
        if self.asset_note and (self.folder or self.draft_source):
            state += ' • ' + self.asset_note
        if self.recovery_error:
            state += ' • recovery failed'
        elif self.last_draft:
            state += ' • recovery draft available'
        self.status.setText(state)
        self.status.setToolTip(
            f'Saved project: {self.folder or "None"}\nRecovered draft: {self.draft_source or "None"}\n'
            f'Last draft this session: {self.last_draft or "None"}\n'
            'Drafts do not clear unsaved changes. Pending DH/geometry previews are not saved.\n'
            'File → Project and recovery status for details.')

    def details(self):
        self.sync_status()
        files = list(self.recovery.glob('*/project.json')) if self.recovery.exists() else []
        latest = max(files, key=lambda p: p.stat().st_mtime) if files else None
        self.e.qt.QMessageBox.information(self.e.window, 'Project and recovery status',
            self.status.text() + '\n\n' + self.status.toolTip() +
            f'\n\nRecovery folder: {self.recovery}\nLatest available draft: {latest or "None"}\n'
            f'Recovery error: {self.recovery_error or "None"}\n\n'
            'Every minute, changed committed work is saved to a separate draft with available assets. '
            'Unresolved assets remain unresolved. Pending previews postpone recovery. '
            'Reopening a draft requires Save As; it never becomes your saved project automatically. '
            'Portable ZIP export checks assets and a local extracted reopen; it is not a second-computer test.')

    def export_urdf(self):
        """Write visual geometry separately from project state or DH certification."""
        folder = self.e.qt.QFileDialog.getExistingDirectory(
            self.e.window, "Choose an empty URDF export folder"
        )
        if not folder:
            return
        target = Path(folder)
        if any(target.iterdir()):
            raise ValueError(
                "Choose an empty folder to avoid overwriting unrelated files"
            )
        with tempfile.TemporaryDirectory() as temporary:
            staged = save_project(self.e, Path(temporary) / "export", True)
            import json

            issues = json.loads((staged / "project.json").read_text(encoding="utf-8"))[
                "unresolved"
            ]
            if issues:
                raise ValueError(
                    "Resolve dependencies before URDF export: " + ", ".join(issues)
                )
            for name in ("meshes", "materials"):
                if (staged / name).exists():
                    shutil.copytree(staged / name, target / name)
            shutil.copyfile(staged / "robot.urdf", target / "robot.urdf")
        self.e.window.statusBar().showMessage(
            "Exported robot.urdf and local assets; accepted DH conventions remain in the project/session"
        )

    def presentation(self):
        splitter = self.e.window.centralWidget()
        visible = splitter.widget(0).isVisible()
        splitter.widget(0).setVisible(not visible)
        splitter.widget(2).setVisible(not visible)
        self.e.table.setVisible(not visible)

    def report(self):
        from dataclasses import asdict
        import json

        if not self.e.application:
            raise ValueError("Load a robot first")
        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Engineering report", "engineering.json", "JSON (*.json)"
        )
        if path:
            app = self.e.application
            data = {
                "source_sha256": app.run.chain.source_sha256,
                "state": asdict(app.session.state),
                "events": [asdict(event) for event in app.session.events],
                "assets": self.e.studio.records,
                "q_si": self.e.q,
                "pending_preview": app.session.pending is not None,
                "validation": self.e.validation_label.text(),
            }
            try:
                data["sampled_fk"] = app.validate().to_dict()
            except ValueError as exc:
                data["validation_incomplete"] = str(exc)
            Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.e.view.screenshot(str(Path(path).with_suffix(".png")))

    def batch(self):
        from urdf2dt.engineering import batch_check
        import json

        paths, _ = self.e.qt.QFileDialog.getOpenFileNames(
            self.e.window,
            "Batch URDF / project checks",
            "",
            "Robot files (*.urdf *.URDF *.json)",
        )
        if not paths:
            return
        output, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Batch report", "batch.json", "JSON (*.json)"
        )
        if not output:
            return
        progress = self.e.qt.QProgressDialog(
            "Checking kinematic paths and assets…",
            "Cancel",
            0,
            len(paths),
            self.e.window,
        )
        reports = []
        for i, path in enumerate(paths):
            progress.setValue(i)
            self.e.qt.QApplication.processEvents()
            if progress.wasCanceled():
                break
            reports.extend(batch_check([path])["robots"])
        progress.setValue(len(paths))
        Path(output).write_text(
            json.dumps(
                {"robots": reports, "cancelled": len(reports) != len(paths)}, indent=2
            ),
            encoding="utf-8",
        )
