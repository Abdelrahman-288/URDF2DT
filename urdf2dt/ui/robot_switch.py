"""Validated robot replacement with suspended observers and rollback."""
from pathlib import Path


def replace_robot(e, path):
    from urdf2dt.ui.loading import LoadCancelled, prepare, checkpoint
    if getattr(e, '_loading_robot', False) or getattr(e, '_load_in_progress', False):
        raise RuntimeError('A robot is already loading')
    worker = getattr(e, '_load_worker', None)
    if worker is not None and worker.is_alive():
        raise RuntimeError('Cancelled input is still finishing validation. Please try again shortly.')
    e._load_in_progress = True
    progress = e.qt.QProgressDialog(f'Validating {Path(path).name}…', 'Cancel', 0, 0, e.window)
    progress.setWindowModality(e.core.Qt.ApplicationModal)
    progress.setMinimumDuration(0)
    progress.setAutoClose(False)
    progress.show()
    e._load_progress = progress
    timers = [(timer, timer.isActive()) for timer in
              (e.timer, e.project_panel.timer, e.symbolic_panel.timer, e.jacobian_panel.timer)]
    for timer, _ in timers:
        timer.stop()
    try:
        document, application = prepare(e, path, progress)
        checkpoint(e, progress, 'Preparing robot scene…')
        _replace_prepared(e, path, document, application)
        return True
    except LoadCancelled:
        e.window.statusBar().showMessage('Loading cancelled. Previous robot remains usable.')
        return False
    finally:
        e._load_progress = None
        e._load_in_progress = False
        progress.close()
        progress.deleteLater()
        for timer, active in timers:
            if active:
                timer.start()


def _replace_prepared(e, path, document, application):
    from urdf2dt.ui.loading import LoadCancelled
    groups = [
        (e, ('document', 'application', 'package_root', 'package_mappings', 'texture_mappings', '_chain_index')),
        (e.studio, ('overrides', 'appearance', 'undo_stack', 'redo_stack', 'selected', 'preview_before', 'dirty', 'opacity_before', 'isolation_before', 'cache')),
        (e.fk_panel, ('frames', 'poses', 'pose_notes', 'thumbnails')),
        (e.project_panel, ('folder',)),
        (e.workspace, ('validated_model',)),
    ]
    saved = [(owner, {name: getattr(owner, name) for name in names}) for owner, names in groups]
    pose = list(e.q)
    camera = e.view.camera_position
    frame = e.frame_select.currentIndex() + 1
    reference, target = e.fk_panel.reference.currentText(), e.fk_panel.target.currentText()
    validation = e.validation_label.text()
    layers = {name: widget.isChecked() for name, widget in e.layers.items()}
    timers = [(timer, timer.isActive()) for timer in
              (e.timer, e.project_panel.timer, e.symbolic_panel.timer, e.jacobian_panel.timer)]
    for timer, _ in timers:
        timer.stop()
    e._loading_robot = True
    try:
        e._install_robot(path, document, application)
        e._loading_robot = False
        e.fk_panel.sync()
        e.symbolic_panel.sync()
        e.jacobian_panel.sync()
    except Exception as exc:
        e._loading_robot = True
        # Restoration must not reuse the cancelled progress dialog.
        progress = e._load_progress
        e._load_progress = None
        progress.hide()
        for owner, values in saved:
            for name, value in values.items():
                setattr(owner, name, value)
        e._refreshing = False
        if e.application is not None:
            e.chain.blockSignals(True)
            e.chain.clear()
            if e.document is not None:
                for p in e.document.paths:
                    e.chain.addItem(f'{p[0]} → {p[-1]} ({len(p)} links)')
            else:
                e.chain.addItem('Saved serial chain')
            e.chain.setCurrentIndex(e._chain_index)
            e.chain.blockSignals(False)
            e.rebuild()
            e.set_pose(pose)
            e.refresh_editor(frame)
            e.fk_panel.reference.setCurrentText(reference)
            e.fk_panel.target.setCurrentText(target)
            for name, checked in layers.items():
                e.layers[name].setChecked(checked)
            e.view.camera_position = camera
            e.validation_label.setText(validation)
            e.studio.dirty = saved[1][1]['dirty']
            e.update_scene()
        else:
            e.view.clear()
            e.q = []
            e.chain.clear()
            e.links.clear()
            e.studio.tree.clear()
            e.studio.items = {}
            e.studio.names = []
            e.studio.records = {}
            e.studio.clear_selection()
            e._clear(e.joint_layout)
            e.joint_controls = []
            e.table.setRowCount(0)
            e.frame_select.clear()
            e.fk_panel.reference.clear()
            e.fk_panel.target.clear()
        if isinstance(exc, LoadCancelled):
            raise
        raise RuntimeError(f'{exc}. Previous robot restored; you can continue using it.'
                           if e.application else f'{exc}. No robot loaded.') from exc
    finally:
        e._loading_robot = False
        e.fk_panel._signature = None
        try:
            e.fk_panel.sync()
            e.symbolic_panel.sync()
            e.jacobian_panel.sync()
            e.workspace.sync()
        finally:
            e.window.setUpdatesEnabled(True)
            for timer, active in timers:
                if active:
                    timer.start()
            e.queue_render()
    message = f'Opened {Path(path).name}'
    if e.warnings:
        message += f' with {len(e.warnings)} geometry notices. Inspect Model & Appearance → Geometry to locate missing assets.'
    e.window.statusBar().showMessage(message)
