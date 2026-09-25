"""Native/frozen acceptance for the start screen and guided workflow."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
from urdf2dt.runtime import asset_path


def verify_welcome(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    from urdf2dt.ui.welcome import read_recent
    from urdf2dt.projects import save_project
    from PySide6.QtGui import QDesktopServices
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    settings = e.settings
    old = settings.value('recent_files_v1', None)
    settings.setValue('recent_files_v1', '[]')
    errors = []
    previous_hook = sys.excepthook
    sys.excepthook = lambda *args: errors.append(str(args[1]))
    try:
        w = e.welcome
        w.show()
        e.window.showMaximized()
        app.processEvents()
        assert w.stack.currentWidget() is w.page and not w.recent.count()
        assert not w.resume.isEnabled()
        e.window.grab().save(str(folder / 'start-screen.png'))
        w.start()
        w.advance(1)
        assert w.step == 0
        with patch.object(e.qt.QFileDialog, 'getOpenFileName', return_value=(str(asset_path('robots/scara/scara_rrpr.urdf')), 'URDF')):
            w.open_step()
        assert e.application is not None and w.step == 1
        model = e.application.session.state
        pose = list(e.q)
        for step, expected in [(1, 'Model & Appearance'), (2, 'Joint motion'), (3, 'FK calculator'), (4, 'DH editor')]:
            w.step = step
            w.render_step()
            w.open_step()
            assert e.tabs.tabText(e.tabs.currentIndex()) == expected
            assert e.application.session.state == model and e.q == pose
        assert e.workspace.results.currentIndex() == 2
        w.step = 2
        w.render_step()
        w.open_step()
        app.processEvents()
        e.window.grab().save(str(folder / 'walkthrough.png'))
        with patch.object(QDesktopServices, 'openUrl', return_value=True) as opened:
            w.manual()
            assert opened.call_count == 1
            assert Path(opened.call_args.args[0].toLocalFile()).is_file()
        project = save_project(e, folder / 'saved-project', True)
        e.project_panel.folder = project
        w.remember(project / 'project.json', 'Project')
        w.remember(project / 'project.json', 'Project')
        assert len([r for r in read_recent(settings) if r['kind'] == 'Project']) == 1
        w.show()
        w.recent.setCurrentRow(0)
        with patch.object(e.project_panel, 'confirm_discard', return_value=False):
            before = e.application
            w.open_recent()
            assert e.application is before
        with patch.object(e.project_panel, 'confirm_discard', return_value=True):
            w.open_recent()
        assert e.project_panel.folder == project and w.stack.currentWidget() is w.editor
        w.remember(folder / 'moved-project.json', 'Project')
        w.show()
        w.recent.setCurrentRow(0)
        before = e.application
        w.open_recent()
        assert e.application is before and 'unavailable' in w.message.text()
        w.remove_recent()
        assert not any('moved-project' in r['path'] for r in read_recent(settings))
        w.step = 5
        with patch.object(e.project_panel, 'save') as saved:
            w.open_step()
            saved.assert_called_once()
        w.advance(1)
        assert not w.active
        settings.setValue('recent_files_v1', 'malformed')
        assert read_recent(settings) == []
        assert not errors, errors
        (folder / 'verification.json').write_text(json.dumps({'passed': True,
            'frozen': bool(getattr(sys, 'frozen', False)), 'checks': [
            'empty start screen', 'load-gated walkthrough', 'step tools match instructions',
            'walkthrough does not mutate model or pose', 'PDF exists and opens via local URL',
            'deduplicated project history', 'recent project reopens', 'discard cancellation respected',
            'unavailable path preserves active robot', 'history removal does not delete files',
            'save action connected', 'malformed history safely ignored', 'no Qt callback errors']}, indent=2))
        return 0
    finally:
        sys.excepthook = previous_hook
        if old is None:
            settings.remove('recent_files_v1')
        else:
            settings.setValue('recent_files_v1', old)
        e.close()
