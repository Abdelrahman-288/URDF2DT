"""Native and packaged recovery/portable-project acceptance with hidden source assets."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
import zipfile
from xml.etree.ElementTree import fromstring, tostring, SubElement

from urdf2dt.projects import portable_zip, save_project, load_project
from urdf2dt.project_integrity import check_portable
from urdf2dt.runtime import asset_path


def verify_recovery(folder, qt_app):
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    settings = e.settings
    saved_settings = {k:settings.value(k) for k in settings.allKeys()}
    try:
        source = folder / 'original-assets'; source.mkdir()
        e.pv.Cube(x_length=.1,y_length=.1,z_length=.1).save(source / 'body.stl')
        root = fromstring(asset_path('robots/scara/scara_rrpr.urdf').read_bytes())
        geometry = root.find('.//visual/geometry')
        assert geometry is not None
        geometry.clear()
        SubElement(geometry,'mesh',filename='body.stl')
        (source/'robot.urdf').write_bytes(tostring(root))
        e.load(str(source/'robot.urdf'))
        assert e.application is not None
        e.window.show(); e.window.resize(1400,900)
        panel=e.project_panel; panel.recovery=folder/'recovery'
        panel.sync_status()
        assert 'no saved project' in panel.status.text()
        e.change_joint(0,.2,False)
        panel.sync_status(); assert 'Unsaved project' in panel.status.text()
        panel.folder=save_project(e,folder/'saved',True)
        e.studio.dirty=False; panel.sync_status()
        assert 'Project saved' in panel.status.text()
        e.change_joint(0,.4,False); panel.autosave()
        draft=panel.last_draft
        assert draft and (draft/'RECOVERY_DRAFT.json').is_file()
        assert e.studio.dirty and panel.folder == folder/'saved'
        assert 'Unsaved changes' in panel.status.text()
        assert check_portable(draft)['asset_files']==1
        e.studio.preview_before = ({}, {})
        panel.autosave()
        assert panel.last_draft == draft
        panel.sync_status(); assert 'preview not saved' in panel.status.text()
        e.studio.preview_before = None
        with patch.object(e.qt.QMessageBox, 'question', return_value=e.qt.QMessageBox.Cancel):
            assert not panel.confirm_discard()
        assert e.studio.dirty
        model=e.application.session.state.working_model
        expected_pose=list(e.q)
        portable_zip(e,folder/'portable.zip')
        archive_bytes = (folder/'portable.zip').read_bytes()
        try:portable_zip(e,folder/'portable.zip')
        except FileExistsError:pass
        else:raise AssertionError('Existing ZIP overwritten')
        assert (folder/'portable.zip').read_bytes() == archive_bytes
        assert e.studio.dirty  # Export must not clear working changes.
        with zipfile.ZipFile(folder/'portable.zip') as archive:
            assert archive.testzip() is None
            archive.extractall(folder/'another-location')
        # Hide original files: any accidental absolute fallback is now unavailable.
        source.rename(folder/'original-assets-unavailable')
        extracted=folder/'another-location/RobotProject'
        panel.open_path(extracted/'project.json',confirmed=True)
        assert e.application is not None
        assert e.application.session.state.working_model==model
        assert e.q==expected_pose
        assert all(not r['resolved'] or Path(r['resolved']).is_relative_to(extracted)
                   for r in e.studio.records.values())
        assert all(r['status'] in ('loaded','primitive geometry') for r in e.studio.records.values())
        panel.open_path(draft/'project.json',confirmed=True)
        assert panel.folder is None and panel.draft_source==draft and e.studio.dirty
        assert 'Recovered draft' in panel.status.text()
        qt_app.processEvents(); e.window.grab().save(str(folder/'recovered-draft.png'))
        # Save into an existing saved project; failed publication restores last-good core files.
        panel.open_path(folder/'saved/project.json',confirmed=True)
        saved_folder = folder/'saved'
        e.change_joint(0,.5,False)
        metadata=(saved_folder/'project.json').read_bytes()
        import os
        real_replace=os.replace
        def fail_metadata(src,dst):
            if Path(dst).name=='project.json':raise OSError('simulated save failure')
            return real_replace(src,dst)
        with patch('urdf2dt.ui.project_panel.os.replace',side_effect=fail_metadata):
            try:panel.save()
            except OSError:pass
            else:raise AssertionError('Expected simulated save failure')
        assert e.studio.dirty and (saved_folder/'project.json').read_bytes()==metadata
        check_portable(saved_folder)
        panel.save(); assert not e.studio.dirty
        assert list((saved_folder/'checkpoints').glob('*/project.json'))
        active=e.application
        mesh=next((extracted/'meshes').rglob('*.stl')); original=mesh.read_bytes()
        mesh.write_bytes(b'corrupted asset')
        try:load_project(e,extracted)
        except ValueError:pass
        else:raise AssertionError('Corrupt asset accepted')
        assert e.application is active
        mesh.write_bytes(original)
        # Unresolved asset export must not publish a partial archive.
        key=next(k for k in e.studio.records if '|visual|' in k)
        e.studio.overrides[key]={'path':str(folder/'missing.stl')}
        e.build_scene()
        try:portable_zip(e,folder/'must-not-exist.zip')
        except ValueError:pass
        else:raise AssertionError('Missing asset export succeeded')
        assert not (folder/'must-not-exist.zip').exists()
        e.studio.dirty=True
        with patch('urdf2dt.ui.project_panel.save_project',side_effect=OSError('simulated disk error')):
            panel.autosave()
        assert 'recovery failed' in panel.status.text() and e.studio.dirty
        (folder/'verification.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),
            'checks':['saved/unsaved/draft distinction','self-contained recovery with original source hidden',
                      'portable extraction and actual UI reopening without original assets','same model and pose',
                      'core-file rollback on ordinary save failure','pending-preview recovery deferred','cancel preserves changes',
                      'existing ZIP preserved','checkpoint preserved','corrupt asset rejected before replacement',
                      'missing asset export leaves no ZIP','recovery failure keeps unsaved state'],
            'different_computer_tested':False},indent=2),encoding='utf-8')
        return 0
    finally:
        e.close(); qt_app.processEvents()
        for key in settings.allKeys():
            if key not in saved_settings:settings.remove(key)
        for key,value in saved_settings.items():settings.setValue(key,value)
        settings.sync()
