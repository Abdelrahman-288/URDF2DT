"""Native/frozen symbolic FK integration verification."""
from pathlib import Path
from time import monotonic, sleep
from unittest.mock import patch
import json
import sys

from urdf2dt.runtime import asset_path


def verify_symbolic(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    checks = []
    def wait(panel):
        deadline = monotonic() + 65
        while panel.process is not None and monotonic() < deadline:
            app.processEvents()
            sleep(.02)
        assert panel.result is not None, panel.status.text()
    try:
        e.load(str(asset_path("robots/scara/scara_rrpr.urdf")))
        e.window.show()
        panel = e.symbolic_panel
        panel.sync()
        e.tabs.setCurrentIndex(e.tabs.count()-1)
        panel.start()
        wait(panel)
        assert panel.result is not None
        assert "vertical_slide [m]" in panel.result[0]
        checks.append("SCARA worker and joint mapping")
        result = panel.result
        e.set_pose([.1, .2, .03, .4])
        panel.sync()
        assert panel.result == result
        checks.append("pose changes reuse symbolic result")
        panel.copy_result(1)
        assert "\\begin{matrix}" in e.qt.QApplication.clipboard().text()
        for label, extension in (("Text (*.txt)", "txt"), ("LaTeX (*.tex)", "tex")):
            target = folder / ("scara_symbolic." + extension)
            with patch.object(e.qt.QFileDialog, "getSaveFileName", return_value=(str(target), label)):
                panel.export()
            assert target.stat().st_size > 100
        checks.append("clipboard and both exports")
        e.window.grab().save(str(folder / "symbolic-scara.png"))
        panel.model.setCurrentIndex(1)
        assert panel.result is None
        panel.start()
        panel.stop()
        assert panel.process is None
        checks.append("model selection invalidation and cancellation")
        panel.model.setCurrentIndex(0)
        assert panel.result == result
        checks.append("cached model restored")
        e.unlock()
        e.edit_controls["axial_translation"].setValue(.025)
        assert e.application is not None
        assert e.application.session.pending is not None
        panel.sync()
        assert panel.result == result
        checks.append("pending preview excluded")
        e.accept()
        panel.sync()
        assert panel.result is None
        checks.append("committed edit invalidates symbolic result")
        panel.start()
        e.load(str(asset_path("robots/ur5/ur5_serial.urdf")))
        panel.sync()
        assert panel.process is None and panel.result is None
        checks.append("robot change cancels stale generation")
        panel.start()
        wait(panel)
        final_result = panel.result
        assert final_result is not None and "q6:" in final_result[0]
        e.window.grab().save(str(folder / "symbolic-ur5.png"))
        checks.append("UR5 worker result")
        (folder / "verification.json").write_text(json.dumps({"passed": True, "frozen": bool(getattr(sys, "frozen", False)), "checks": checks}, indent=2), encoding="utf-8")
        return 0
    finally:
        e.close()
