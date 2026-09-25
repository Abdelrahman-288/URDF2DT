"""Real Qt/VTK loading acceptance checks, also callable in the Windows package."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
import time
import xml.etree.ElementTree as ET
from urdf2dt.runtime import asset_path


def verify_loading(folder, app, mesh_robot=None):
    from urdf2dt.ui.desktop import DesktopEditor
    from urdf2dt.ui import loading
    from urdf2dt.parser.robot_document import RobotDocument
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    scara = asset_path('robots/scara/scara_rrpr.urdf')
    ur5 = asset_path('robots/ur5/ur5_serial.urdf')
    tree = ET.fromstring(scara.read_text())
    geometry = tree.find('.//visual/geometry')
    assert geometry is not None
    geometry.clear()
    ET.SubElement(geometry, 'mesh', filename='missing-body-for-load-test.stl')
    missing = folder / 'missing.urdf'
    missing.write_text(ET.tostring(tree, encoding='unicode'))
    large = folder / 'large.urdf'
    root_link = tree.find('link')
    assert root_link is not None
    base = root_link.get('name')
    assert base is not None
    for i in range(160):
        link = ET.SubElement(tree, 'link', name=f'branch_{i}')
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size='.02 .02 .02')
        joint = ET.SubElement(tree, 'joint', name=f'fixed_{i}', type='fixed')
        ET.SubElement(joint, 'parent', link=base)
        ET.SubElement(joint, 'child', link=f'branch_{i}')
    large.write_text(ET.tostring(tree, encoding='unicode'))
    e = DesktopEditor()
    errors = []
    hook = sys.excepthook
    sys.excepthook = lambda *args: errors.append(str(args[1]))
    checks = []
    try:
        for path in (scara, ur5, missing, scara, large, ur5, scara):
            assert e.load(str(path))
            assert e.application is not None
            assert Path(e.application.run.chain.source_urdf).resolve() == path.resolve()
            assert len(e.q) == len(e.joint_controls) == e.table.rowCount()
            assert e.application.run.chain.base_link in e.workspace.context.text()
            assert e.application.run.chain.joints[-1].child_link in e.workspace.context.text()
            if path in (missing, large):
                assert any('missing-body' in warning for warning in e.warnings)
                assert e.mesh_actors and e.fk_panel.result
            e.set_pose([.05] * len(e.q))
            e.update_scene()
        checks.append('seven mixed robot switches; 165+ link tree; missing mesh still usable')
        old, pose = e.application, list(e.q)
        assert old is not None
        e.studio.tree.setCurrentItem(e.studio.items[old.run.chain.tip_link])
        selected = e.studio.selected
        e.studio.dirty = True
        original_load = RobotDocument.load
        def slow(path):
            time.sleep(.15)
            return original_load(path)
        e.core.QTimer.singleShot(40, lambda: getattr(e, '_load_progress').cancel())
        with patch.object(RobotDocument, 'load', side_effect=slow):
            assert e.load(str(ur5)) is False
        getattr(e, '_load_worker').join(timeout=10)
        assert e.application is old and e.q == pose and e.studio.dirty
        checks.append('cancel during background validation; late result discarded')
        real_checkpoint = loading.checkpoint
        for label in ('Loading geometry', 'Building urdf frames'):
            cancelled: list[bool] = []
            def cancel_scene(editor, progress, text=None):
                if text and text.startswith(label) and not cancelled:
                    cancelled.append(True)
                    progress.cancel()
                return real_checkpoint(editor, progress, text)
            with patch.object(loading, 'checkpoint', side_effect=cancel_scene):
                assert e.load(str(large)) is False
            assert cancelled
            assert e.application is old and e.q == pose and e.studio.selected == selected
            assert e.studio.dirty and e.timer.isActive()
            e.update_scene()
        checks.append('cancel during geometry and frames restores pose, selection and dirty state')
        bad = folder / 'invalid.urdf'
        bad.write_text('<robot>broken')
        messages = []
        with patch.object(e.project_panel, 'confirm_discard', return_value=True), patch.object(
                e.qt.QFileDialog, 'getOpenFileName', return_value=(str(bad), 'URDF')), patch.object(
                e.qt.QMessageBox, 'warning', side_effect=lambda *args: messages.append(args[-1])):
            e.open_dialog()
        assert messages and 'Active robot was not changed' in messages[0]
        assert e.application is old and e.q == pose
        assert e.load(str(ur5))
        checks.append('invalid input gives actionable error; next successful load works')
        if mesh_robot:
            assert e.load(str(mesh_robot))
            assert e.mesh_actors
            e.window.grab().save(str(folder / 'large-body.png'))
            checks.append(f'local body robot: {mesh_robot}; {len(e.mesh_actors)} geometry actors')
        assert not errors, errors
        e.window.grab().save(str(folder / 'loaded.png'))
        (folder / 'verification.json').write_text(json.dumps({
            'passed': True, 'frozen': bool(getattr(sys, 'frozen', False)), 'checks': checks}, indent=2))
        return 0
    finally:
        sys.excepthook = hook
        e.close()
