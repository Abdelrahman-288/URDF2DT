"""Project actions and recoverable committed drafts for the native application."""

from datetime import datetime
from pathlib import Path
import shutil
import tempfile
import os
from urdf2dt.projects import save_project, load_project, portable_zip


class ProjectPanel:
    def __init__(self, e):
        self.e = e
        self.folder = None
        menu = e.window.menuBar().addMenu("Project")
        for label, callback in (
            ("Open Project…", self.open),
            ("Save Project", self.save),
            ("Save Project As…", self.save_as),
            ("Export Portable Project ZIP…", self.zip),
            ("Create checkpoint…", self.checkpoint),
            ("Open recovery draft…", self.recover),
            ("Presentation mode", self.presentation),
        ):
            e._action(menu, label, callback)
        e._action(menu, "Engineering report…", self.report)
        e._action(menu, "Batch URDF / project validation…", self.batch)
        e._action(menu, "Export geometry URDF with assets…", self.export_urdf)
        self.include = e.qt.QCheckBox("Include local assets when saving")
        self.include.setChecked(True)
        e.window.statusBar().addPermanentWidget(self.include)
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
            return not self.e.studio.dirty
        return True

    def open(self):
        if not self.confirm_discard():
            return
        path, _ = self.e.qt.QFileDialog.getOpenFileName(
            self.e.window, "Open project", "", "URDF2DT project (project.json)"
        )
        if path:
            self.folder = load_project(self.e, path)

    def save_as(self):
        path = self.e.qt.QFileDialog.getExistingDirectory(
            self.e.window, "Choose an empty project folder"
        )
        if path:
            self.folder = save_project(self.e, path, self.include.isChecked())
            self.e.studio.dirty = False
            self.e.window.statusBar().showMessage(f"Project saved: {self.folder}")

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
            # Asset groups are content-addressed; preserve unrelated project contents.
            for asset in staged.rglob("*"):
                if asset.is_file() and asset.name != "project.json":
                    target = self.folder / asset.relative_to(staged)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(asset, target)
            temp = self.folder / "project.json.new"
            shutil.copyfile(staged / "project.json", temp)
            os.replace(temp, self.folder / "project.json")
        self.e.studio.dirty = False

    def zip(self):
        path, _ = self.e.qt.QFileDialog.getSaveFileName(
            self.e.window, "Portable project ZIP", "RobotProject.zip", "ZIP (*.zip)"
        )
        if path:
            portable_zip(self.e, path)

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
            save_project(
                self.e,
                self.recovery / datetime.now().strftime("%Y%m%d-%H%M%S-%f"),
                False,
            )
            self.e.window.statusBar().showMessage(
                "Recovery draft saved separately; explicit project save still required"
            )
        except Exception as exc:
            self.e.window.statusBar().showMessage(
                f"Recovery draft could not be saved: {exc}"
            )

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
            load_project(self.e, path)
            self.folder = None
            self.e.studio.dirty = True

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
