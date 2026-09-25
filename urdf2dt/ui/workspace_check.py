"""Native and frozen regression evidence for workspace changes."""
import json
import os
from pathlib import Path
from copy import deepcopy
from urdf2dt.runtime import asset_path


def verify_workspace(folder, app):
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    e = DesktopEditor()
    previous = (e.workspace.angular, e.workspace.linear, e.workspace.digits)
    try:
        e.window.showMaximized()
        app.processEvents()
        assert e.window.isMaximized()
        e.load(str(asset_path('robots/scara/scara_rrpr.urdf')))
        app.processEvents()
        assert e.application is not None
        movable = [j.child_link for j in e.application.run.chain.joints if j.joint_type.value != 'fixed']
        actor = next(a for a,_,_,kind in e.mesh_actors if kind == 'visual' and e.studio.actors[id(a)]['link'] == movable[-1])
        e.studio.picked(actor)
        e.update_scene()
        selected = e.studio.actors[id(actor)]['link']
        assert e.studio.selected == selected
        assert e.studio.tree.currentItem().data(0,256) == selected
        assert e.studio.selection_box is not None
        assert e.studio.selection_box.visibility
        old_bounds = tuple(e.studio.selection_box.bounds)
        e.set_pose([.3,.2,.03,.4])
        e.update_scene()
        assert tuple(e.studio.selection_box.bounds) != old_bounds
        appearance = deepcopy(e.studio.appearance)
        e.studio.isolate()
        e.studio.isolate()
        assert e.studio.appearance == appearance
        e.studio.clear_selection()
        e.update_scene()
        assert not e.studio.selection_box.visibility
        q = list(e.q)
        e.workspace.grid_action.setChecked(True)
        e.update_scene()
        assert e.workspace.grid_actor is not None
        assert e.workspace.grid_actor.visibility
        e.workspace.fit_robot()
        e.workspace.grid_action.setChecked(False)
        e.update_scene()
        assert not e.workspace.grid_actor.visibility
        model = e.application.session.state.working_model
        for _ in range(10):
            for angular, linear in [('deg','mm'),('rad','m')]:
                e.workspace.angular, e.workspace.linear = angular, linear
                e.workspace.sync()
                assert e.q == q and e.application.session.state.working_model == model
                for (_, spin, _, _), value in zip(e.joint_controls,q):
                    spin.interpretText()
                    assert abs(spin.value()-value) < 1e-14
                e.fk_panel.apply()
                assert e.q == q
        panel = e.workspace.analysis
        e.tabs.setCurrentIndex(panel.index)
        panel.refresh()
        assert panel.tables[0][0].rowCount() == 6
        assert panel.tables[0][0].columnCount() == 4
        panel.rates[0].setValue(.25)
        assert e.q == q
        e.np.testing.assert_allclose(panel.result['twist'],e.np.asarray(panel.result['matrix']) @ [.25,0,0,0])
        panel.model.setCurrentIndex(3)
        assert not panel.result and panel.twist.rowCount() == 0
        panel.model.setCurrentIndex(0)
        for mode in (1,2,0):
            panel.model.setCurrentIndex(mode)
            assert e.fk_panel.model.currentIndex() == mode
            assert e.fk_panel.result['model'] == panel.model.currentText()
        from urdf2dt.dh.recompute import FrameEdit
        e.application.session.propose_edit(1, FrameEdit())
        pending = e.application.session.pending
        e.workspace.select_dh_row(1,0)
        e.update_scene()
        assert e.application.session.pending is pending
        e.application.session.reject()
        panel.refresh()
        e.window.showNormal()
        for theme in ['Light','Dark']:
            e.theme.setCurrentText(theme)
            for width,height in [(1366,768),(1920,1080)]:
                e.window.resize(width,height)
                e.workspace.reset()
                app.processEvents()
                e.window.grab().save(str(folder/f'{theme}-{width}.png'))
        (folder/'verification.json').write_text(json.dumps(dict(passed=True, scale=os.environ.get('QT_SCALE_FACTOR','1'),
            checks=['maximized startup','physical-link picking and tree synchronization','moving bounding box',
                    'empty selection hides box','isolation restores appearance','20 unit switches preserve q/model',
                    '6x4 Jacobian, Jv/Jw tables','rates produce J*qdot without pose changes','unavailable preview clears stale values',
                    'light/dark window captures requested at 1366x768 and 1920x1080']),indent=2),encoding='utf-8')
        return 0
    finally:
        e.workspace.angular, e.workspace.linear, e.workspace.digits = previous
        e.close()
