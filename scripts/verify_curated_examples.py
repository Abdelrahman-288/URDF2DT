"""Render curated bodies, test every active branch and capture real appearance."""
from pathlib import Path
import json
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import QSettings
    from urdf2dt.ui.desktop import DesktopEditor
    folder = Path(sys.argv[1]).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    app = QApplication.instance() or QApplication([])
    settings = QSettings('URDF2DT', 'Desktop')
    saved = {k: settings.value(k) for k in settings.allKeys()}
    evidence = []
    try:
        e = DesktopEditor()
        try:
            e.window.show()
            e.window.resize(1400, 900)
            for example in json.loads((ROOT/'robots/curated/manifest.json').read_text())['examples']:
                e.load(str(ROOT/'robots/curated'/example['urdf']))
                e.theme.setCurrentText('Light')
                assert len(e.document.paths) == example['paths']
                for i in range(example['paths']):
                    with patch.object(e.project_panel, 'confirm_discard', return_value=True):
                        e.chain.setCurrentIndex(i)
                    assert len(e.q) == example['joints_per_path'][i]
                    actors = [a for a, _, _, kind in e.mesh_actors if kind == 'visual']
                    assert len(actors) == example['visuals']
                    assert all(r['status'] == 'primitive geometry' for r in e.studio.records.values())
                    e.set_pose([0.] * len(e.q)); e.update_scene()
                    before = [e.np.asarray(a.user_matrix).copy() for a in actors]
                    e.set_pose([.2] * len(e.q)); e.update_scene()
                    changed = sum(not e.np.allclose(old, a.user_matrix) for old, a in zip(before, actors))
                    assert changed > 0
                    if example['id'] == 'branched_robot':
                        assert changed == 2  # other arm and shared base/torso remain fixed
                    evidence.append({'robot':example['id'], 'tip':e.application.run.chain.tip_link,
                                     'rendered_bodies':len(actors), 'moving_bodies':changed})
                with patch.object(e.project_panel, 'confirm_discard', return_value=True):
                    e.chain.setCurrentIndex(0)
                pose = {'serial_arm':[.4,-.8,.4], 'scara':[.5,-.8,.06,.3], 'branched_robot':[.4,.7]}[example['id']]
                e.set_pose(pose)
                for name in ('labels','urdf','dh','joint_axes','bones'):
                    e.layers[name].setChecked(False)
                e.studio.clear_selection()
                e.update_scene(); e.view.view_isometric(); e.view.reset_camera()
                app.processEvents()
                e.view.screenshot(str(folder/(example['id']+'.png')))
                e.window.grab().save(str(folder/(example['id']+'-workspace.png')))
        finally:
            e.close(); app.processEvents()
    finally:
        for key in settings.allKeys():
            if key not in saved: settings.remove(key)
        for key, value in saved.items():settings.setValue(key,value)
        settings.sync()
    (folder/'verification.json').write_text(json.dumps({'passed':True,'checks':evidence},indent=2))


if __name__ == '__main__':
    main()
