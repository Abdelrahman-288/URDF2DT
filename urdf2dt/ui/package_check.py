"""Automated native checks executed inside the shipped frozen interpreter."""

import json
from pathlib import Path
import shutil
import sys
from typing import Any

from urdf2dt.app import Application
from urdf2dt.runtime import asset_path
from urdf2dt.logging_config import git_provenance


def verify_package(folder: Path, app: Any) -> int:
    """Exercise native callbacks, external URDF input, rendering and persistence."""
    from urdf2dt.ui.desktop import DesktopEditor

    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    results: dict[str, Any] = {"frozen": bool(getattr(sys, "frozen", False)),
                               "provenance": git_provenance(), "robots": {}}
    for robot, relative in (("scara", "robots/scara/scara_rrpr.urdf"),
                             ("ur5", "robots/ur5/ur5_serial.urdf")):
        source = folder / (robot + " external input.urdf")
        shutil.copyfile(asset_path(relative), source)
        editor = DesktopEditor()
        try:
            editor.window.show()
            app.processEvents()
            assert editor.application is None  # empty launch must work
            dialog = editor.qt.QFileDialog
            original_open = dialog.getOpenFileName
            original_save = dialog.getSaveFileName
            try:
                # Invoke actual application callbacks, substituting only the chooser result.
                dialog.getOpenFileName = lambda *a, **k: (str(source), "URDF")
                editor.open_dialog()
                assert editor.application is not None
                for index, control in enumerate(editor.joint_controls):
                    control[0].setValue(6000)
                editor.update_scene()
                actors = tuple(id(item[0]) for item in editor.mesh_actors)
                changes = 0
                for index in range(1, len(editor.application.run.automatic_model.rows) + 1):
                    editor.unlock()
                    if "axial_translation" in editor.edit_controls:
                        editor.edit_controls["axial_translation"].setValue(0.025)
                        changes += 1
                    editor.accept()
                assert changes
                editor.validate()
                assert "PASS" in editor.validation_label.text()
                assert actors == tuple(id(item[0]) for item in editor.mesh_actors)
                target = folder / robot / "session"
                dialog.getSaveFileName = lambda *a, **k: (str(target), "")
                editor.save_archive()
                before = editor.application.session.state
                dialog.getOpenFileName = lambda *a, **k: (str(target / "session.json"), "JSON")
                editor.load_archive()
                assert editor.application.session.state == before
                if robot == "scara":
                    editor.decompose()
                    count = sum(item[3] == "generated" for item in editor.mesh_actors)
                    assert count > 0
                    editor.decompose()
                    assert sum(item[3] == "generated" for item in editor.mesh_actors) == count
                    results["collision_parts"] = count
            finally:
                dialog.getOpenFileName = original_open
                dialog.getSaveFileName = original_save
            for theme in ("Light", "Dark"):
                editor.theme.setCurrentText(theme)
                editor.update_scene()
                app.processEvents()
                assert editor.window.grab().save(str(folder / f"{robot}_{theme.lower()}.png"))
            results["robots"][robot] = {"joints": len(editor.joint_controls),
                                        "edited_frames": changes, "fk_passed": True,
                                        "round_trip_equal": True, "actors_reused": True,
                                        "mesh_warnings": editor.warnings}
            results["graphics"] = editor.view.render_window.ReportCapabilities()
        finally:
            editor.close()
            app.processEvents()
        reopened = DesktopEditor()
        try:
            reopened.application = Application.resume(folder / robot / "session/session.json")
            reopened.rebuild()
            reopened.window.show()
            app.processEvents()
            reopened.validate()
            assert "PASS" in reopened.validation_label.text()
        finally:
            reopened.close()
            app.processEvents()
    results["passed"] = True
    (folder / "verification.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return 0
