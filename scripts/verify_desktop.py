"""Exercise live native actor updates and capture local application previews."""

from pathlib import Path
from time import perf_counter
import json

from PySide6.QtWidgets import QApplication
from urdf2dt.ui.desktop import DesktopEditor


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    app = QApplication.instance() or QApplication([])
    editor = DesktopEditor()
    source = root / "references/doctor/universalUR5.urdf"
    if not source.exists():
        source = root / "robots/ur5/ur5_serial.urdf"
    try:
        editor.load(str(source))
        editor.window.show()
        app.processEvents()
        editor.set_pose([0.4, -0.8, 1.2, -0.5, 0.8, 0.2])
        editor.update_scene()
        app.processEvents()
        actors = tuple(id(a) for a, _, _, _ in editor.mesh_actors)
        assert editor.application is not None
        before_motion = editor.application.session.state
        before_matrix = editor.mesh_actors[-1][0].user_matrix.copy() if actors else None
        times = []
        for i in range(30):
            start = perf_counter()
            editor.change_joint(0, -0.5 + i / 30, False)
            editor.update_scene()
            times.append((perf_counter() - start) * 1000)
        assert actors == tuple(id(a) for a, _, _, _ in editor.mesh_actors)
        assert editor.application is not None
        assert editor.application.session.state == before_motion
        if before_matrix is not None:
            assert not editor.np.allclose(
                before_matrix, editor.mesh_actors[-1][0].user_matrix
            )
        baseline = editor.application.session.state.automatic_model
        for i in range(1, 7):
            editor.unlock()
            if i in (2, 6):
                committed = editor.application.session.state.working_model
                editor.edit_controls["axial_translation"].setValue(0.06)
                assert editor.current_model() != committed
                editor.reject()
                assert editor.current_model() == committed
                editor.edit_controls["axial_translation"].setValue(0.03)
            editor.accept()
        editor.validate()
        assert "PASS" in editor.validation_label.text()
        assert editor.application.session.state.automatic_model == baseline
        folder = root / "outputs/tmp/desktop"
        folder.mkdir(parents=True, exist_ok=True)
        for theme in ("Light", "Dark"):
            editor.theme.setCurrentText(theme)
            editor.set_theme(theme)
            app.processEvents()
            editor.update_scene()
            app.processEvents()
            editor.window.grab().save(str(folder / f"{theme.lower()}.png"))
        editor.layers["labels"].setChecked(True)
        editor.tabs.setCurrentIndex(1)
        editor.update_scene()
        app.processEvents()
        assert all(a.GetVisibility() for a, _, _ in editor.label_actors)
        editor.window.grab().save(str(folder / "dh-editor.png"))
        # Exercise the installed VHACD backend and replacement of old generated parts.
        generated_count = 0
        if editor.mesh_data:
            editor.decompose()
            editor.update_scene()
            generated_count = sum(
                kind == "generated" for _, _, _, kind in editor.mesh_actors
            )
            assert generated_count > 0
            editor.decompose()
            editor.update_scene()
            assert (
                sum(kind == "generated" for _, _, _, kind in editor.mesh_actors)
                == generated_count
            )
        report = {
            "generated_collision_parts": generated_count,
            "source": str(source),
            "mesh_actor_count": len(actors),
            "updates": len(times),
            "mean_update_ms": sum(times) / len(times),
            "max_update_ms": max(times),
            "actors_reused": True,
            "fk": editor.validation_label.text(),
            "warnings": editor.warnings,
        }
        (folder / "verification.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        # Reloading must not retain the previous session's displayed PASS.
        editor.load(str(source))
        assert "PASS" not in editor.validation_label.text()
        print(json.dumps(report, indent=2))
    finally:
        editor.close()
        app.processEvents()


if __name__ == "__main__":
    main()
