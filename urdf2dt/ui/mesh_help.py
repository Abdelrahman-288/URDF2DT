"""Explain actual asset resolution and stage local replacement candidates."""
from pathlib import Path
from urdf2dt.parser.robot_document import resolve_mesh


def folder_candidate(record, folder, searched):
    requested = record.get('requested') or record['original']
    if requested.startswith('package://'):
        return resolve_mesh(requested, Path(record['source_urdf']), Path(folder), {}, searched)
    if '://' in requested:
        raise FileNotFoundError('Only local mesh files are supported. Choose Locate file.')
    path = Path(requested)
    # For absolute references from another machine, use the explicitly chosen
    # folder and exact filename. This is only a preview proposal, never auto-apply.
    return resolve_mesh(path.name if path.is_absolute() else requested,
                        Path(folder) / 'robot.urdf', searched=searched)


def details(key, record):
    requested = record.get('requested') or record.get('original', '')
    package = requested[len('package://'):].partition('/')[0] if requested.startswith('package://') else None
    guidance = (f'Locate the folder for ROS package "{package}": it should contain the referenced meshes subfolder. '
                'You can also choose the parent folder containing that package.' if package else
                'Relative paths are resolved beside the URDF. Keep its mesh folders with it, or select the exact local mesh file.')
    return '\n'.join([
        f'Component: {key}', f'Status: {record.get("status", "unknown")}',
        f'URDF: {record.get("source_urdf", "")}', f'Original reference: {record.get("original", "")}',
        f'Current requested reference: {requested}', f'Resolved: {record.get("resolved") or "Not resolved"}',
        f'Details: {record.get("message", "")}', '', 'How to fix', guidance,
        'Locate file or Locate folder stages a preview for this geometry element only. '
        'Inspect the body in the main view, then Apply preview or Cancel preview in Geometry. '
        'A matching filename does not prove it is the correct robot part.', '',
        'Exact file candidates checked (in search order):',
        *record.get('searched', []),
        *(['No local candidates: primitive geometry or rejected/unsupported URI.'] if not record.get('searched') else []),
        '', 'Search is local. No online download or unrestricted filename search was performed.'
    ])


def show_details(studio):
    e, qt = studio.e, studio.e.qt
    dialog = qt.QDialog(e.window)
    dialog.setWindowTitle('Mesh troubleshooting — files and searched locations')
    dialog.resize(880, 650)
    layout = qt.QVBoxLayout(dialog)
    combo = qt.QComboBox()
    keys = sorted(studio.records, key=lambda k: (studio.records[k].get('status') not in ('missing', 'unsupported'), k))
    if not keys:
        raise ValueError('Open a robot with geometry first.')
    for key in keys:
        combo.addItem(f"{key} — {studio.records[key].get('status')}", key)
    layout.addWidget(combo)
    text = qt.QPlainTextEdit()
    text.setReadOnly(True)
    layout.addWidget(text)
    row = qt.QHBoxLayout()
    layout.addLayout(row)

    def refresh():
        key = combo.currentData()
        text.setPlainText(details(key, studio.records[key]))

    def locate(folder=False):
        key = combo.currentData()
        record = studio.records[key]
        studio.tree.setCurrentItem(studio.items[key])
        studio.active_geometry = key
        if folder:
            path = qt.QFileDialog.getExistingDirectory(dialog, 'Locate package or mesh folder')
            if not path:
                return
            searched: list[str] = []
            try:
                candidate = folder_candidate(record, path, searched)
            except FileNotFoundError as exc:
                text.setPlainText(details(key, record) + '\n\nSelected folder did not resolve this reference:\n'
                                  + str(exc) + '\n' + '\n'.join(searched)
                                  + '\nChoose Locate file for a renamed file or a different folder layout.')
                return
        else:
            path, _ = qt.QFileDialog.getOpenFileName(dialog, 'Preview a replacement mesh', '',
                'Meshes (*.stl *.STL *.obj *.ply *.vtk *.vtp *.dae)')
            if not path:
                return
            candidate = Path(path)
        try:
            studio.preview_replacement(candidate)
        except Exception as exc:
            qt.QMessageBox.warning(dialog, 'Replacement not loaded', str(exc))
            return
        e.tabs.setCurrentIndex(e.workspace.inspector_index)
        studio.properties.setCurrentIndex(1)
        dialog.accept()

    e._button(row, 'Locate file and preview…', lambda: locate(False))
    e._button(row, 'Locate folder and preview…', lambda: locate(True))
    e._button(row, 'Copy diagnostics', lambda: qt.QApplication.clipboard().setText(text.toPlainText()))
    e._button(row, 'Close', dialog.reject)
    combo.currentIndexChanged.connect(lambda _: refresh())
    if studio.active_geometry in keys:
        combo.setCurrentIndex(keys.index(studio.active_geometry))
    refresh()
    dialog.exec()
