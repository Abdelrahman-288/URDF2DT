"""Native/frozen geometric Jacobian UI acceptance checks."""
from pathlib import Path
from time import monotonic, sleep
from unittest.mock import patch
import json
import sys
import numpy as np
from urdf2dt.runtime import asset_path
from urdf2dt.symbolic import symbolic_jacobian


def verify_jacobian(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    checks = []

    def wait(panel):
        deadline = monotonic() + 65
        ticks = []
        timer = e.core.QTimer()
        timer.setInterval(20)
        timer.timeout.connect(lambda: ticks.append(1))
        timer.start()
        while panel.process is not None and monotonic() < deadline:
            app.processEvents()
            sleep(.02)
        timer.stop()
        assert panel.result is not None, panel.status.text()
        assert ticks, 'UI event loop must remain responsive during generation'
        panel.sync()

    try:
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        e.window.show()
        panel = e.jacobian_panel
        e.tabs.setCurrentIndex(panel.index)
        panel.start()
        wait(panel)
        result = panel.result
        assert result is not None and 'vertical_slide [m]' in result[0] and 'J(q)' in result[0]
        assert 'world' in panel.scope.text() and 'origin' in panel.scope.text()
        checks.append('SCARA worker, frame conventions and responsive UI')
        pose = [.123456789, -.234567891, .0123456789, .345678912]
        e.set_pose(pose)
        panel.sync()
        assert panel.result == result
        assert e.application is not None and panel.numeric_result is not None
        matrix = panel.numeric_result['matrix']
        np.testing.assert_allclose(matrix,symbolic_jacobian(e.application.session.state.working_model).evaluate(pose),atol=2e-12)
        assert np.asarray(matrix).shape == (6,4)
        assert np.all(np.asarray(matrix)[3:,2] == 0)
        assert '0.123' in panel.numeric.toPlainText()
        checks.append('cached expression, live numerical evaluation and prismatic column')
        panel.copy_result(1)
        assert 'J(q)' in e.qt.QApplication.clipboard().text()
        for label, extension in (('Text (*.txt)','txt'),('LaTeX (*.tex)','tex')):
            target = folder/('jacobian.'+extension)
            with patch.object(e.qt.QFileDialog,'getSaveFileName',return_value=(str(target),label)):
                panel.export()
            assert target.stat().st_size > 100
        target = folder/'jacobian.json'
        with patch.object(e.qt.QFileDialog,'getSaveFileName',return_value=(str(target),'JSON')):
            panel.export_numeric()
        saved = json.loads(target.read_text())
        assert saved['q_si'] == pose and saved['expressed_in'] == 'world'
        np.testing.assert_array_equal(saved['matrix'],matrix)
        checks.append('text, LaTeX, clipboard and full precision numerical JSON')
        def capture():
            dialog = app.activeModalWidget()
            assert dialog is not None
            dialog.grab().save(str(folder/'jacobian-full-view.png'))
            dialog.accept()
        e.core.QTimer.singleShot(300,capture)
        panel.full_view()
        e.window.grab().save(str(folder/'jacobian-scara.png'))
        e.window.resize(1280, 800)
        app.processEvents()
        assert e.window.width() <= 1280 and e.window.height() <= 800
        e.window.grab().save(str(folder/'jacobian-1280x800.png'))
        checks.append('full matrix dialog')
        panel.model.setCurrentIndex(1)
        assert panel.result is None and panel.numeric_result is None
        panel.start()
        panel.stop()
        assert panel.process is None
        panel.model.setCurrentIndex(0)
        assert panel.result == result
        checks.append('model invalidation, cancellation and cached restoration')
        e.unlock()
        e.edit_controls['axial_translation'].setValue(.025)
        assert e.application.session.pending is not None
        panel.sync()
        assert panel.result == result
        e.accept()
        panel.sync()
        assert panel.result is None and panel.numeric_result is None
        checks.append('preview excluded; committed edits invalidate symbolic and numeric results')
        panel.start()
        e.load(str(asset_path('robots/ur5/ur5_serial.urdf')))
        panel.sync()
        assert panel.process is None and panel.result is None
        panel.start()
        wait(panel)
        assert panel.numeric_result is not None
        assert np.asarray(panel.numeric_result['matrix']).shape == (6,6)
        e.window.grab().save(str(folder/'jacobian-ur5.png'))
        checks.append('robot replacement cancels stale worker; UR5 six columns')
        (folder/'verification.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),'checks':checks},indent=2))
        return 0
    finally:
        e.close()
