"""Body-rendering acceptance against the user's external robot collection."""

from pathlib import Path
import json
import sys
from typing import Any
from urdf2dt.logging_config import git_provenance

EXAMPLES = (
    "abb_irb140/urdf/irb140.urdf",
    "abb_irb6700_support/urdf/irb6700_200_260.urdf",
    "fanuc_lrmate200id_support/urdf/lrmate200id.urdf",
    "kuka_kr120_support/urdf/kr120r2500pro.urdf",
    "iiwa_description/urdf/iiwa14_no_collision.urdf",
)


def verify_library(folder, root, qt_app):
    from urdf2dt.ui.desktop import DesktopEditor
    from urdf2dt.projects import save_project, load_project

    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    root = Path(root).resolve()
    report: dict[str, Any] = {
        "frozen": bool(getattr(sys, "frozen", False)),
        "source": git_provenance(),
        "robots": [],
    }
    for relative in EXAMPLES:
        editor = DesktopEditor()
        try:
            editor.load(str(root / relative))
            editor.window.show()
            qt_app.processEvents()
            assert editor.application is not None
            expected = sum("|visual|" in key for key in editor.studio.records)
            visual = [a for a, _, _, kind in editor.mesh_actors if kind == "visual"]
            assert expected > 0 and len(visual) == expected, (
                relative,
                editor.studio.records,
            )
            assert all(
                r["status"] in ("loaded", "primitive geometry")
                for r in editor.studio.records.values()
            ), editor.studio.records
            state = editor.application.session.state
            identities = [id(actor) for actor in visual]
            before = [editor.np.asarray(actor.user_matrix).copy() for actor in visual]
            editor.change_joint(0, 0.12, False)
            editor.update_scene()
            qt_app.processEvents()
            assert any(
                not editor.np.allclose(old, actor.user_matrix)
                for old, actor in zip(before, visual)
            )
            assert identities == [
                id(a) for a, _, _, kind in editor.mesh_actors if kind == "visual"
            ]
            assert editor.application.session.state == state
            editor.theme.setCurrentText("Light")
            editor.layers["urdf"].setChecked(False)
            editor.layers["dh"].setChecked(False)
            editor.view.reset_camera()
            editor.update_scene()
            qt_app.processEvents()
            name = Path(relative).stem
            editor.window.grab().save(str(folder / (name + ".png")))
            if relative == EXAMPLES[0]:
                project = save_project(editor, folder / "ABB-project", True)
                load_project(editor, project)
                assert all(
                    not r["resolved"] or Path(r["resolved"]).is_relative_to(project)
                    for r in editor.studio.records.values()
                )
            report["robots"].append(
                {
                    "file": relative,
                    "visual_elements": len(visual),
                    "all_body_elements_rendered": True,
                    "joint_motion_reuses_actors": True,
                    "kinematics_unchanged": True,
                    "warnings": editor.warnings,
                }
            )
        finally:
            editor.close()
            qt_app.processEvents()
    (folder / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0
