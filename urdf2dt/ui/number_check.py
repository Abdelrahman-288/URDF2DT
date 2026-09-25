"""Native display-only precision regression, also runnable in the frozen app."""
from pathlib import Path
import json
import sys
from unittest.mock import patch
from urdf2dt.runtime import asset_path


def verify_numbers(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    from urdf2dt.ui.number_display import number_line, residual
    folder=Path(folder).resolve()
    folder.mkdir(parents=True,exist_ok=False)
    e=DesktopEditor()
    try:
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        e.window.show()
        pose=[.123456789, -.234567891, .0123456789, .345678912]
        e.set_pose(pose)
        e.fk_panel.sync()
        assert e.application is not None
        state=e.application.session.state
        for q, (_,spin,_,_) in zip(pose,e.joint_controls):
            assert spin.text()==f'{q:.3f}'
            assert abs(spin.value()-q)<1e-15
            spin.interpretText()
            assert abs(spin.value()-q)<1e-15
        e.fk_panel.apply()
        assert e.q==pose, 'Unchanged formatted input must preserve original pose'
        e.fk_panel.units.setCurrentIndex(1)
        e.fk_panel.apply()
        assert e.q == pose, 'Degree presentation must not round-trip the original pose'
        e.fk_panel.units.setCurrentIndex(0)
        for row in range(e.table.rowCount()):
            assert len(e.table.item(row,3).text().split('.')[1])==3
        result=e.fk_panel.result
        text=e.fk_panel.output.toPlainText()
        assert 'q [rad/m]: [0.123, -0.235, 0.012, 0.346]' in text
        assert '-0.000' not in text
        target=folder/'fk.json'
        with patch.object(e.qt.QFileDialog,'getSaveFileName',return_value=(str(target),'JSON')):
            e.fk_panel.export()
        saved=json.loads(target.read_text())
        assert saved['q_si']==pose and saved['matrix']==result['matrix']
        assert any(float(f'{v:.3f}')!=v for row in saved['matrix'] for v in row)
        field=number_line(e.qt,'1.23456789 0.0000001 -0.0')
        assert field.text()=='1.235 0.000 0.000'
        assert field.preciseText()=='1.23456789 0.0000001 -0.0'
        field.setModified(True)
        assert field.preciseText()==field.text()
        assert residual(1e-12)=='1.000e-12'
        assert e.application.session.state==state
        e.tabs.setCurrentIndex(e.fk_panel.index)
        app.processEvents()
        e.window.grab().save(str(folder/'three-decimal-fk.png'))
        (folder/'verification.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),
            'checks':['three-decimal matrix/vectors/table/spins','negative zero removed','unmodified spin focus precision',
                      'unchanged form retains pose','JSON retains full precision','geometry field raw values preserved',
                      'small residual scientific notation','model state unchanged']},indent=2))
        return 0
    finally:
        e.close()
