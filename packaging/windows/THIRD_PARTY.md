# Third-party runtime notices

URDF2DT's MIT licence applies to its own code. Python, Qt/PySide6, VTK, CasADi,
NumPy, SciPy and other bundled components retain their upstream licences.
`inventory.json` records the build environment and copied licence/notice files;
some entries are build tools rather than shipped runtime modules.

Qt for Python is offered under LGPLv3/GPL and commercial terms:
https://doc.qt.io/qtforpython-6/
https://doc.qt.io/qt-6/licensing.html
This application uses the open-source distribution, with dynamically loaded
libraries kept as separate replaceable files in `_internal`. The build does not
intentionally use Qt WebEngine or other unrelated application frameworks.
CasADi core is LGPLv3-or-later; optional optimization plugins are not required by
the FK workflow. Copied upstream notices may describe components not collected.

Upstream corresponding source and build information:
- Qt: https://download.qt.io/archive/qt/ (match version in inventory)
- PySide/Shiboken: https://code.qt.io/pyside/pyside-setup.git
- CasADi: https://github.com/casadi/casadi (match version in inventory)
- VTK: https://gitlab.kitware.com/vtk/vtk
- Python: https://www.python.org/downloads/source/

Users may replace or rebuild the dynamically linked LGPL components, and may
reverse engineer the combined work for debugging modifications to those components.
URDF2DT source and the PyInstaller spec are available in the public repository.
Rebuild on Windows with the pinned dependency versions; keep ABI-compatible DLLs
beside their Python extension modules. Modifying a dependency may change validation
results; preserve your original session evidence.

Before final public distribution, check the collected binary inventory against
the notices and provide the applicable corresponding-source materials alongside
the download. The candidate and these notices do not establish clean-PC acceptance.
