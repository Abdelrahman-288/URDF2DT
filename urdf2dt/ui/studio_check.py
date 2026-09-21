"""Native regression checks also callable inside the frozen Windows application."""

from pathlib import Path
from time import perf_counter
import json
import shutil
import sys
from unittest.mock import patch
from importlib import import_module
from typing import Any
from urdf2dt.runtime import asset_path
from urdf2dt.projects import save_project, load_project, portable_zip
from urdf2dt.logging_config import git_provenance


def verify_studio(folder, qt_app, mesh_robot=None):
    from urdf2dt.ui.desktop import DesktopEditor

    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "frozen": bool(getattr(sys, "frozen", False)),
        "source": git_provenance(),
        "robots": {},
    }
    sources = {"scara": asset_path("robots/scara/scara_rrpr.urdf")}
    if mesh_robot:
        sources["ur5_mesh"] = Path(mesh_robot)
    for label, source in sources.items():
        e = DesktopEditor()
        try:
            e.load(str(source))
            e.window.show()
            qt_app.processEvents()
            assert e.application is not None
            assert e.mesh_actors, "Body geometry must exist"
            initial = e.application.session.state
            actors = tuple(id(a) for a, _, _, _ in e.mesh_actors)
            timings = []
            for k in range(10):
                start = perf_counter()
                e.change_joint(0, k * 0.02, False)
                e.update_scene()
                qt_app.processEvents()
                timings.append((perf_counter() - start) * 1000)
            assert actors == tuple(id(a) for a, _, _, _ in e.mesh_actors)
            assert e.application.session.state == initial
            if label == "scara":
                # Exercise real inspector callbacks; only replace native chooser responses.
                snapshot = e.studio.snapshot()
                key = next(k for k in e.studio.records if "|visual|" in k)
                e.studio.overrides[key] = {"path": str(folder / "missing.stl")}
                e.build_scene()
                assert e.studio.records[key]["status"] == "missing"
                broken = folder / "broken.dae"
                broken.write_text("<COLLADA>", encoding="utf-8")
                e.studio.overrides[key] = {"path": str(broken)}
                e.build_scene()
                assert e.studio.records[key]["status"] == "unsupported"
                assert e.application.session.state == initial
                e.studio.tree.setCurrentItem(e.studio.items[key])
                mesh = folder / "millimetre cube.stl"
                e.pv.Cube(x_length=1000, y_length=1000, z_length=1000).save(mesh)
                chooser = e.qt.QFileDialog.getOpenFileName
                try:
                    e.qt.QFileDialog.getOpenFileName = lambda *a, **kw: (
                        str(mesh),
                        "STL",
                    )
                    e.studio.browse()
                finally:
                    e.qt.QFileDialog.getOpenFileName = chooser
                e.studio.units.setCurrentText("mm")
                e.studio.scale.setText("2 1 1")
                e.studio.xyz.setText("0.1 0.2 0.3")
                e.studio.preview()
                e.studio.apply_preview()
                e.np.testing.assert_allclose(
                    e.studio.records[key]["dimensions_m"], [2, 1, 1], atol=1e-6
                )
                assert e.application.session.state == initial
                e.studio.overrides, e.studio.appearance = snapshot
                e.build_scene()
            e.studio.tree.setCurrentItem(e.studio.items[e.studio.names[1]])
            target = e.studio.names[1]
            previous = e.studio.snapshot()
            history = len(e.studio.undo_stack)
            e.studio.alpha.sliderPressed.emit()
            e.studio.alpha.setValue(80)
            e.studio.alpha.setValue(65)
            e.update_scene()
            assert e.studio.appearance[target]["opacity"] == 0.65
            for actor, _, _, kind in e.mesh_actors:
                record = e.studio.actors.get(id(actor), {})
                if kind == "visual" and record.get("link") == target:
                    e.np.testing.assert_allclose(
                        actor.prop.opacity,
                        0.65 * record["rgba"][3] * e.opacity.value() / 100,
                    )
            assert e.studio.alpha_label.text() == "65%"
            assert len(e.studio.undo_stack) == history
            e.studio.alpha.sliderReleased.emit()
            assert len(e.studio.undo_stack) == history + 1
            e.studio.undo()
            assert e.studio.snapshot() == previous
            e.studio.redo()
            assert e.studio.alpha.value() == 65
            color_type = import_module("PySide6.QtGui").QColor
            with patch.object(
                e.qt.QColorDialog, "getColor", return_value=color_type("#3979b9")
            ):
                e.studio.pick_color()
            assert e.studio.appearance[target]["color"] == "#3979b9"
            e.update_scene()
            for actor, _, _, kind in e.mesh_actors:
                if (
                    kind == "visual"
                    and e.studio.actors.get(id(actor), {}).get("link") == target
                ):
                    e.np.testing.assert_allclose(
                        actor.prop.color.float_rgb, [57 / 255, 121 / 255, 185 / 255]
                    )
            previous = e.studio.snapshot()
            with patch.object(e.qt.QColorDialog, "getColor", return_value=color_type()):
                e.studio.pick_color()
            assert e.studio.snapshot() == previous
            e.studio.tree.setCurrentItem(e.studio.items[e.studio.names[0]])
            assert e.studio.alpha.value() == 100
            assert e.studio.snapshot() == previous
            e.studio.tree.setCurrentItem(e.studio.items[target])
            assert e.studio.alpha.value() == 65
            assert not hasattr(e.studio, "notes")
            assert not any(
                "Apply appearance" in button.text()
                for button in e.window.findChildren(e.qt.QPushButton)
            )
            assert e.application.session.state == initial
            key = e.studio.active_geometry
            e.studio.xyz.setText("0.01 0 0")
            e.studio.preview()
            assert e.studio.preview_before is not None
            e.studio.cancel_preview()
            assert not e.studio.overrides
            e.fk_panel.calculate()
            assert (
                e.fk_panel.result["comparison"]["automatic"]["position_error_m"] < 1e-8
            )
            for index in range(1, len(e.q) + 1):
                e.unlock()
                e.preview_flip("z" if index % 2 else "x")
                e.accept()
            e.validate()
            assert "PASS" in e.validation_label.text()
            e.fk_panel.poses["Verification pose"] = list(e.q)
            e.fk_panel.frames["Inspection reference"] = {
                "link": e.application.run.chain.base_link,
                "xyz": [0.1, 0.2, 0.3],
                "rpy": [0, 0, 0],
            }
            e.fk_panel.rebuild()
            e.fk_panel.calculate()
            before = e.application.session.state.working_model
            appearance = dict(e.studio.appearance)
            saved = save_project(e, folder / (label + "-original"), True)
            portable_zip(e, folder / (label + ".zip"))
            relocated = folder / (label + "-relocated")
            shutil.move(str(saved), str(relocated))
            load_project(e, relocated)
            assert e.application.session.state.working_model == before
            assert e.fk_panel.poses["Verification pose"] == e.q
            assert e.studio.appearance == appearance
            assert "Inspection reference" in e.fk_panel.frames
            save_project(e, folder / (label + "-resaved"), True)
            load_project(e, folder / (label + "-resaved"))
            assert e.application.session.state.working_model == before
            load_project(e, relocated)
            metadata = relocated / "project.json"
            original_metadata = metadata.read_text(encoding="utf-8")
            invalid = json.loads(original_metadata)
            invalid["pose"][0] = "not a number"
            metadata.write_text(json.dumps(invalid), encoding="utf-8")
            active = e.application
            try:
                try:
                    load_project(e, relocated)
                except ValueError:
                    pass
                else:
                    raise AssertionError("Invalid project unexpectedly loaded")
                assert e.application is active
            finally:
                metadata.write_text(original_metadata, encoding="utf-8")
            e.update_scene()
            qt_app.processEvents()
            assert all(
                not r["resolved"] or Path(r["resolved"]).is_relative_to(relocated)
                for r in e.studio.records.values()
            )
            for theme in ("Light", "Dark"):
                e.theme.setCurrentText(theme)
                e.update_scene()
                qt_app.processEvents()
                e.window.grab().save(str(folder / f"{label}-{theme.lower()}.png"))
            e.tabs.setCurrentIndex(e.fk_panel.index)
            e.fk_panel.calculate()
            qt_app.processEvents()
            e.window.grab().save(str(folder / f"{label}-fk.png"))
            e.window.resize(1280, 800)
            qt_app.processEvents()
            assert e.window.width() <= 1280 and e.window.height() <= 800
            e.studio.properties.setCurrentIndex(1)
            e.window.grab().save(str(folder / f"{label}-1280x800.png"))
            from urdf2dt.engineering import batch_check

            assert batch_check([relocated / "project.json"])["robots"][0]["valid"]
            report["robots"][label] = {
                "actors": len(e.mesh_actors),
                "elements": len(e.studio.records),
                "actor_reuse": True,
                "fk_after_flips": True,
                "project_relocated": True,
                "invalid_project_preserved_active_state": True,
                "appearance_preserved": True,
                "live_opacity_color_and_selection": True,
                "opacity_drag_single_undo": True,
                "missing_asset_located_and_mm_scale_checked": label == "scara",
                "named_frame_preserved": True,
                "layout_1280x800": True,
                "pose_update_mean_ms": sum(timings) / len(timings),
                "asset_status": e.studio.records,
            }
        finally:
            e.close()
            qt_app.processEvents()
    (folder / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0
