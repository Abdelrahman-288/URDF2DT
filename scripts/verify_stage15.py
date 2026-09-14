"""SCARA end-to-end evidence, independent analytic checks and optional native view."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from urdf2dt.app import Application
from urdf2dt.config import EditorConfig, ValidationConfig
from urdf2dt.dh.classification import classify_dh_model
from urdf2dt.dh.global_validation import sample_configurations, validate_global_fk
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.kinematics import dh_fk, urdf_fk
from urdf2dt.logging_config import git_provenance
from urdf2dt.research.scara_oracle import analytic_fk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--desktop", action="store_true")
    args = parser.parse_args()
    folder = Path(args.output)
    if folder.exists():
        parser.error("Choose a new output directory")
    source = Path(__file__).resolve().parents[1] / "robots/scara/scara_rrpr.urdf"
    app = Application(source, EditorConfig(validation=ValidationConfig(samples=200)))
    baseline = validate_global_fk(
        app.run.chain, app.run.automatic_model, app.run.config
    )
    assert baseline.passed
    edits = {i: FrameEdit(0.025, 0.2 if i in (3, 4) else 0.0) for i in range(1, 5)}
    for i, edit in edits.items():
        app.session.unlock_next()
        app.session.propose_edit(i, edit)
        assert app.session.accept().valid
    report = app.validate()
    assert report.passed
    errors = []
    for q in sample_configurations(app.run.chain, app.run.config):
        reference = analytic_fk(q)
        errors.append(
            max(
                abs(a - b)
                for model in (
                    urdf_fk(app.run.chain, q),
                    dh_fk(app.run.automatic_model, q),
                    dh_fk(app.session.state.working_model, q),
                )
                for row, ref in zip(model, reference)
                for a, b in zip(row, ref)
            )
        )
    assert max(errors) < 1e-12
    folder.mkdir(parents=True)
    archive = app.export(folder / "session")
    resumed = Application.resume(archive)
    assert resumed.session.state == app.session.state and resumed.validate().passed
    result = {
        "robot": "Original idealized RRPR SCARA",
        "source_sha256": app.run.chain.source_sha256,
        "configuration": app.run.config.snapshot(),
        "provenance": git_provenance(),
        "joint_types": [r.joint_type.value for r in app.run.automatic_model.rows],
        "classifications": [
            asdict(c) for c in classify_dh_model(app.run.automatic_model)
        ],
        "edits": {str(i): asdict(e) for i, e in edits.items()},
        "analytic_max_matrix_element_error": max(errors),
        "analytic_samples": len(errors),
        "baseline": baseline.to_dict()["summary"],
        "edited": report.to_dict()["summary"],
        "round_trip_equal": True,
        "human_usability": "pending independent participant observations",
    }
    if args.desktop:
        from PySide6.QtWidgets import QApplication
        from urdf2dt.ui.desktop import DesktopEditor

        qt = QApplication.instance() or QApplication([])
        ui = DesktopEditor()
        try:
            ui.load(str(source))
            ui.window.show()
            qt.processEvents()
            assert len(ui.joint_controls) == 4
            assert ui.joint_controls[2][2:] == (0.0, 0.18)
            ui.joint_controls[2][0].setValue(10000)
            assert abs(ui.q[2] - 0.18) < 1e-12
            ui.set_pose([0.6, -1.1, 0.09, 0.4])
            ui.update_scene()
            actors = tuple(id(a) for a, _, _, _ in ui.mesh_actors)
            for i in range(1, 5):
                ui.unlock()
                ui.edit_controls["axial_translation"].setValue(0.025)
                if i in (3, 4):
                    ui.edit_controls["axial_rotation"].setValue(0.2)
                ui.accept()
            ui.validate()
            assert "PASS" in ui.validation_label.text()
            assert actors == tuple(id(a) for a, _, _, _ in ui.mesh_actors)
            for theme in ("Light", "Dark"):
                ui.theme.setCurrentText(theme)
                ui.update_scene()
                qt.processEvents()
                assert ui.window.grab().save(
                    str(folder / f"desktop_{theme.lower()}.png")
                )
            result["desktop"] = {
                "controls": 4,
                "mesh_actors": len(actors),
                "mesh_warnings": ui.warnings,
                "prismatic_slider_endpoint_m": 0.18,
                "actor_reuse": True,
                "edited_fk_passed": True,
            }
        finally:
            ui.close()
            qt.processEvents()
    (folder / "verification.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print("SCARA pipeline, analytic oracle and archive checks passed.")
    print("Independent human usability observations remain pending.")


if __name__ == "__main__":
    main()
