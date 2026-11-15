"""Measured window/scaling and frozen runtime evidence; not clean-PC certification."""
import json
import os
from pathlib import Path
import shutil
import sys
from urdf2dt.runtime import asset_path
from urdf2dt.ui.release_info import release_details


def verify_release(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    settings = e.settings
    saved = {k:settings.value(k) for k in settings.allKeys()}
    measurements = []
    try:
        e.window.showMaximized(); app.processEvents()
        assert e.window.isMaximized()
        e.welcome.show()
        app.processEvents()
        e.window.grab().save(str(folder/'welcome.png'))
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        e.window.showNormal()
        available = e.window.screen().availableGeometry()
        sizes = [(1366,768),(1280,720),(1024,768),(1024,640),
                 (min(1280,available.width()-20),min(720,available.height()-50))]
        for width,height in dict.fromkeys(sizes):
            e.workspace.reset()
            e.window.resize(width,height)
            app.processEvents()
            for theme in ('Light','Dark'):
                e.theme.setCurrentText(theme)
                for index in range(e.tabs.count()):
                    e.tool_selector.setCurrentIndex(index)
                    app.processEvents()
                    actual = [e.window.width(),e.window.height()]
                    screenshot=f'{theme}-{width}-{index}.png'
                    e.window.grab().save(str(folder/screenshot))
                    measurements.append({'requested_logical':[width,height],'actual_logical':actual,
                        'device_pixel_ratio':e.window.devicePixelRatioF(),'tool':e.tabs.tabText(index),
                        'fits_available_screen':actual[0]<=available.width() and actual[1]<=available.height()-40,
                        'theme':theme,'screenshot':screenshot})
                    assert actual[0] <= width and actual[1] <= height, (width,height,actual,e.tabs.tabText(index))
                    assert e.tool_selector.isVisible()
                e.tool_selector.setCurrentIndex(0)
                e.change_joint(0,.3,False); e.update_scene()
                assert abs(e.q[0]-.3)<1e-12
        text=release_details()
        assert 'KNOWN LIMITATIONS' in text and 'Build:' in text
        (folder/'about.txt').write_text(text,encoding='utf-8')
        from urdf2dt.ui.release_info import show_release
        def inspect_dialog():
            dialog=app.activeModalWidget()
            assert dialog is not None
            editor=dialog.findChild(e.qt.QPlainTextEdit)
            assert 'KNOWN LIMITATIONS' in editor.toPlainText()
            copy=next(b for b in dialog.findChildren(e.qt.QPushButton) if b.text().startswith('Copy build'))
            copy.click()
            assert app.clipboard().text()==editor.toPlainText()
            dialog.grab().save(str(folder/'about.png'))
            dialog.accept()
        e.core.QTimer.singleShot(100,inspect_dialog)
        show_release(e)
        external=[]
        if getattr(sys,'frozen',False):
            bundle=Path(sys.executable).resolve().parent
            for module in list(sys.modules.values()):
                name=getattr(module,'__file__',None)
                if name and Path(name).is_file() and not Path(name).resolve().is_relative_to(bundle):
                    external.append(name)
            assert not external,external
        report={'passed':True,'frozen':bool(getattr(sys,'frozen',False)),
            'qt_scale_factor':os.environ.get('QT_SCALE_FACTOR','1'),
            'screen_available_logical':[e.window.screen().availableGeometry().width(),e.window.screen().availableGeometry().height()],
            'measurements':measurements,'external_module_files':external,
            'python_on_path':shutil.which('python'),'python3_on_path':shutil.which('python3'),
            'clean_machine_verified':False,'scope':'Qt override scaling and logical window constraints on this host; not a separate clean Windows PC or native multi-monitor DPI test'}
        (folder/'verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        return 0
    finally:
        e.close();app.processEvents()
        for key in settings.allKeys():
            if key not in saved:settings.remove(key)
        for key,value in saved.items():settings.setValue(key,value)
        settings.sync()
