"""Native/frozen toolbar, readable matrices and optional real Atlas checks."""
from pathlib import Path
from time import monotonic, sleep
import json
import sys
from urdf2dt.runtime import asset_path


def verify_presentation(folder, app, robots=()):
    from urdf2dt.ui.desktop import DesktopEditor
    folder=Path(folder).resolve()
    folder.mkdir(parents=True,exist_ok=False)
    e=DesktopEditor()
    evidence: list = []
    try:
        e.window.show()
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        assert e.tool_selector.count() == e.tabs.count() and e.tabs.count() >= 7
        assert e.tabs.tabBar().isHidden()
        for i in range(e.tool_selector.count()):
            e.tool_selector.setCurrentIndex(i)
            assert e.tabs.currentIndex() == i and e.panel_title.text() == e.tabs.tabText(i)
        for panel in (e.symbolic_panel,e.jacobian_panel):
            e.tool_selector.setCurrentIndex(panel.index)
            panel.start()
            deadline=monotonic()+65
            while panel.process is not None and monotonic()<deadline:
                app.processEvents()
                sleep(.02)
            assert panel.result is not None, panel.status.text()
            view=panel.matrix_view
            assert view.matrix.rowCount() == (6 if panel.kind=='Jacobian' else 4)
            assert view.definitions.rowCount()>0
            for i in range(view.matrix.rowCount()):
                found=False
                for j in range(view.matrix.columnCount()):
                    if view.matrix.item(i,j).text().startswith('M_'):
                        view.follow_entry(i,j)
                        assert view.mode.currentIndex()==1 and view.detail.toPlainText()
                        found=True
                        break
                if found:break
            view.mode.setCurrentIndex(0)
            panel.copy_result(0)
            assert e.qt.QApplication.clipboard().text() == panel.result[0]
            if panel.kind=='Jacobian':
                assert panel.numeric_table.rowCount()==6
                e.set_pose([.12,.23,.04,.34])
                panel.sync()
                assert panel.numeric_result is not None
                assert panel.numeric_table.item(0,0).text() == f"{panel.numeric_result['matrix'][0][0]:.3f}"
            e.window.resize(1280,800)
            app.processEvents()
            assert e.window.width()<=1280 and e.window.height()<=800
            e.window.grab().save(str(folder/(panel.kind.lower()+'-1280.png')))
            if panel.kind == 'Jacobian':
                view.mode.setCurrentIndex(3)
                app.processEvents()
                assert panel.numeric_table.isVisible()
                e.window.grab().save(str(folder/'numerical-jacobian-1280.png'))
        evidence.append('toolbar switches all registered tools; hidden tab bar; matrix/definitions; exact clipboard; numeric cells; 1280 layout')
        for robot in robots:
            e.load(str(robot))
            app.processEvents()
            visuals=[(key,record) for key,record in e.studio.records.items() if '|visual|' in key]
            assert visuals and all(r['status'] in ('loaded','primitive geometry') for _,r in visuals)
            assert len([a for a,_,_,kind in e.mesh_actors if kind=='visual'])==len(visuals)
            if Path(robot).stem.lower()=='atlas':
                meshes = [r for _,r in visuals if r['format'] != 'primitive']
                assert len({r['resolved'] for r in meshes})==24
                assert all('STL substitute' in r['message'] for r in meshes)
            matrices=[e.np.asarray(a.user_matrix).copy() for a,_,_,kind in e.mesh_actors if kind=='visual']
            e.set_pose([.1]*len(e.q))
            e.update_scene()
            after=[e.np.asarray(a.user_matrix) for a,_,_,kind in e.mesh_actors if kind=='visual']
            assert any(not e.np.allclose(a,b) for a,b in zip(matrices,after))
            e.tool_selector.setCurrentIndex(0)
            e.layers['labels'].setChecked(False)
            e.layers['urdf'].setChecked(False)
            e.layers['dh'].setChecked(False)
            e.layers['joint_axes'].setChecked(False)
            e.update_scene()
            e.view.reset_camera()
            e.window.resize(1500,940)
            app.processEvents()
            e.window.grab().save(str(folder/(Path(robot).stem+'-body.png')))
            evidence.append({'robot':str(robot),'visuals':len(visuals),'all_loaded':True,'pose_motion':True})
        (folder/'verification.json').write_text(json.dumps({'passed':True,'frozen':bool(getattr(sys,'frozen',False)),'checks':evidence},indent=2))
        return 0
    finally:e.close()
