"""Native and packaged diagnostics/replacement-preview acceptance."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
from urdf2dt.runtime import asset_path


def verify_mesh_help(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    from urdf2dt.ui.mesh_help import details
    from urdf2dt.projects import save_project
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    previous = e.settings.value('recent_files_v1', None)
    try:
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        assert e.application is not None
        key = next(k for k in e.studio.records if '|visual|' in k)
        e.studio.overrides[key] = {'path': str(folder / 'missing-body.stl')}
        e.build_scene()
        e.studio.tree.setCurrentItem(e.studio.items[key])
        record = e.studio.records[key]
        assert record['status'] == 'missing'
        assert str(folder/'missing-body.stl') in details(key, record)
        assert record['searched'] == [str(folder/'missing-body.stl')]
        before = e.studio.snapshot()
        model = e.application.session.state
        replacement = folder/'replacement.stl'
        e.pv.Cube(x_length=.1, y_length=.2, z_length=.3).save(replacement)
        with patch.object(e.qt.QFileDialog, 'getOpenFileName', return_value=(str(replacement), 'STL')):
            e.studio.browse()
        assert e.studio.preview_before == before
        assert e.studio.records[key]['status'] == 'loaded'
        e.np.testing.assert_allclose(e.studio.records[key]['dimensions_m'], [.1,.2,.3], atol=1e-6)
        try:
            save_project(e, folder/'must-not-save', True)
            raise AssertionError('Pending replacement must not be saved')
        except ValueError:
            pass
        e.studio.cancel_preview()
        assert e.studio.snapshot() == before and e.application.session.state == model
        e.studio.preview_replacement(replacement)
        e.tabs.setCurrentIndex(e.workspace.inspector_index)
        e.window.showMaximized()
        app.processEvents()
        e.window.grab().save(str(folder/'replacement-preview.png'))
        e.studio.apply_preview()
        assert e.studio.preview_before is None and e.studio.dirty
        e.studio.undo()
        assert e.studio.snapshot() == before
        broken = folder/'invalid.unknown'
        broken.write_text('not a mesh')
        try:
            e.studio.preview_replacement(broken)
            raise AssertionError('Invalid mesh should fail')
        except ValueError:
            pass
        assert e.studio.snapshot() == before and e.studio.preview_before is None
        captured = []
        def capture():
            dialog = e.qt.QApplication.activeModalWidget()
            if dialog:
                dialog.grab().save(str(folder/'search-details.png'))
                captured.append(True)
                dialog.reject()
        e.core.QTimer.singleShot(150, capture)
        e.studio.mesh_details()
        assert captured
        e.fk_panel.units.setCurrentIndex(3)
        e.fk_panel.calculate()
        assert 'in mm' in e.fk_panel.explanation.text() and 'use deg' in e.fk_panel.explanation.text()
        e.symbolic_panel.sync()
        e.jacobian_panel.sync()
        assert 'q1 = shoulder [rad]' in e.symbolic_panel.scope.text()
        assert 'not the numerical FK selectors' in e.jacobian_panel.scope.text()
        e.tabs.setCurrentIndex(e.fk_panel.index)
        app.processEvents()
        e.window.grab().save(str(folder/'fk-explanation.png'))
        (folder/'verification.json').write_text(json.dumps({'passed':True,
            'frozen':bool(getattr(sys,'frozen',False)), 'checks':[
            'exact missing path and search trace', 'actual replacement mesh rendered at expected dimensions',
            'save blocked while preview pending', 'cancel restores previous geometry',
            'apply enters undo history', 'invalid file preserves state', 'DH state unchanged',
            'diagnostics dialog and preview screenshots captured']}, indent=2))
        return 0
    finally:
        if previous is None:
            e.settings.remove('recent_files_v1')
        else:
            e.settings.setValue('recent_files_v1', previous)
        e.close()
