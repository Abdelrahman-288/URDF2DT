"""Native and standalone repeated-open, rollback and viewport gesture checks."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
from urdf2dt.runtime import asset_path


def verify_switching(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    from PySide6.QtTest import QTest
    from PySide6.QtCore import QPoint, Qt
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    third = folder/'third.urdf'
    source = asset_path('robots/scara/scara_rrpr.urdf').read_text()
    import xml.etree.ElementTree as ET
    root = ET.fromstring(source)
    root.set('name', 'third_robot')
    third.write_text(ET.tostring(root, encoding='unicode'))
    invalid = folder/'invalid.urdf'
    invalid.write_text('<robot>broken')
    paths = [asset_path('robots/scara/scara_rrpr.urdf'), asset_path('robots/ur5/ur5_serial.urdf'), third]
    e = DesktopEditor()
    errors = []
    messages = []
    previous_hook = sys.excepthook
    sys.excepthook = lambda kind, value, trace: errors.append(str(value))
    opened = []
    try:
        e.window.show()
        assert all(a.text().lower() != 'open example' for bar in e.window.findChildren(e.qt.QToolBar) for a in bar.actions())
        with patch.object(e.qt.QMessageBox, 'question', return_value=e.qt.QMessageBox.Discard), patch.object(e.qt.QMessageBox, 'warning', side_effect=lambda *a: messages.append(a[-1])):
            for path in paths + paths + paths + [paths[0]]:
                old = e.application
                with patch.object(e.qt.QFileDialog, 'getOpenFileName', return_value=(str(path), 'URDF')):
                    e.open_dialog()
                app.processEvents()
                assert e.application is not None and e.application is not old, messages
                assert e.document is not None
                assert Path(e.application.run.chain.source_urdf).resolve() == path.resolve()
                n = len(e.application.run.chain.joint_names)
                assert len(e.q) == len(e.joint_controls) == e.table.rowCount() == n
                assert e.fk_panel.target.currentText() == e.application.run.chain.tip_link
                assert e.fk_panel.result['joint_names'] == list(e.application.run.chain.joint_names)
                assert set(e.studio.names) == {node.get('name') for node in e.document.root.findall('link')}
                assert e.symbolic_panel.process is None and e.jacobian_panel.process is None
                # Exercise controls and selection, rather than only loading files.
                e.studio.tree.setCurrentItem(e.studio.items[e.application.run.chain.tip_link])
                e.set_pose([.1]*n)
                e.update_scene()
                e.unlock()
                next(iter(e.edit_controls.values())).setValue(.01) if e.edit_controls else None
                e.symbolic_panel.start()
                opened.append(str(path))
            e.symbolic_panel.stop()
            old = e.application
            assert old is not None
            pose = list(e.q)
            model_state = old.session.state
            selected = e.studio.selected
            for failure in ('parse', 'scene'):
                original = e.build_scene
                calls = []
                def fail_once():
                    calls.append(1)
                    if len(calls) == 1:
                        raise RuntimeError('Injected scene failure')
                    return original()
                with patch.object(e.qt.QFileDialog, 'getOpenFileName', return_value=(str(invalid if failure == 'parse' else paths[1]), 'URDF')):
                    if failure == 'scene':
                        with patch.object(e, 'build_scene', side_effect=fail_once): e.open_dialog()
                    else: e.open_dialog()
                app.processEvents()
                assert e.application is old and e.q == pose and old.session.state == model_state
                assert e.studio.selected == selected and e.timer.isActive()
                e.update_scene()
            assert len(messages) == 2 and 'Active robot' in messages[0] and 'restored' in messages[1]
        # Test actual Qt gestures on a visible VTK viewport.
        point = QPoint(e.view.width()-12,12)
        e.studio.tree.setCurrentItem(e.studio.items[old.run.chain.tip_link])
        before = e.view.camera_position
        QTest.mousePress(e.view,Qt.MouseButton.LeftButton,Qt.KeyboardModifier.NoModifier,point)
        QTest.mouseMove(e.view,point-QPoint(90,-70),40)
        QTest.mouseMove(e.view,point,40)  # drag returning to start must not deselect
        QTest.mouseRelease(e.view,Qt.MouseButton.LeftButton,Qt.KeyboardModifier.NoModifier,point)
        app.processEvents()
        assert e.studio.selected == selected and e.studio.tree.selectedItems()
        QTest.mouseClick(e.view,Qt.MouseButton.LeftButton,Qt.KeyboardModifier.NoModifier,point)
        app.processEvents()
        e.update_scene()
        assert not e.studio.selected and not e.studio.tree.selectedItems()
        assert e.links.currentRow() == -1 and e.studio.crumb.text() == 'No selection'
        assert all(not a.prop.show_edges for a,_,_,_ in e.mesh_actors)
        # Click a visible body by projecting its center; background clears again.
        picked = False
        for actor,_,_,kind in e.mesh_actors:
            if kind != 'visual': continue
            center = actor.center
            e.view.renderer.SetWorldPoint(*center,1)
            e.view.renderer.WorldToDisplay()
            x,y,_ = e.view.renderer.GetDisplayPoint()
            width,height = e.view.render_window.GetSize()
            point_body = QPoint(round(x*e.view.width()/width),round((height-1-y)*e.view.height()/height))
            QTest.mouseClick(e.view,Qt.MouseButton.LeftButton,Qt.KeyboardModifier.NoModifier,point_body)
            app.processEvents()
            if e.studio.selected:
                picked = True
                break
        assert picked, 'Body clicking must still select a component'
        QTest.mouseClick(e.view,Qt.MouseButton.LeftButton,Qt.KeyboardModifier.NoModifier,point)
        app.processEvents()
        e.update_scene()
        assert not e.studio.selected
        assert not errors, errors
        e.window.grab().save(str(folder/'viewport-deselected.png'))
        (folder/'verification.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),
            'opens':opened,'checks':['ten Open URDF cycles with pose, selection and preview edits','stale worker cancelled',
            'scene/tree/sliders/DH/FK synchronized','parse and scene failure preserve active robot and pose',
            'clear load error dialogs','Open Example toolbar removed','body click selects','background click clears tree and highlight',
            'drag returning to origin preserves selection','no Qt callback exceptions']},indent=2))
        return 0
    finally:
        sys.excepthook = previous_hook
        e.close()

