# One-folder desktop build. Run via scripts/build_windows.py.
from pathlib import Path
import pkgutil
import PySide6
from PyInstaller.utils.hooks import collect_all, collect_submodules, copy_metadata

root = Path(SPECPATH).parents[1]
data, binaries, hidden = [], [], []
qt_needed = {"QtCore", "QtGui", "QtWidgets", "QtSvg", "QtSvgWidgets", "QtOpenGL",
             "QtOpenGLWidgets", "QtNetwork", "QtPrintSupport"}
unused_qt = ["PySide6." + module.name for module in pkgutil.iter_modules(PySide6.__path__)
             if module.name.startswith("Qt") and module.name not in qt_needed]
for package in ("pyvista", "pyvistaqt", "trimesh", "collada", "vhacdx"):
    package_data, package_bins, package_hidden = collect_all(package)
    data += package_data
    binaries += package_bins
    hidden += package_hidden
hidden += collect_submodules("vtkmodules")
hidden += ["casadi", "casadi.casadi", "casadi._casadi"]
hidden += ["PySide6.QtWidgets", "PySide6.QtGui", "PySide6.QtCore", "PySide6.QtSvg",
           "urdf2dt.ui.package_check", "jsonschema", "defusedxml.ElementTree"]
for name in ("urdf2dt", "pyvista", "pyvistaqt", "trimesh", "casadi", "jsonschema"):
    data += copy_metadata(name)
data += [(str(root / "config"), "resources/config"),
         (str(root / "assets"), "resources/assets"),
         (str(root / "schemas"), "resources/schemas"),
         (str(root / "robots/scara"), "resources/robots/scara"),
         (str(root / "robots/ur5/ur5_serial.urdf"), "resources/robots/ur5"),
         (str(root / "build/windows/build-info.json"), "resources"),
         (str(root / "build/windows/session.schema.json"), "resources/schemas")]

a = Analysis([str(root / "packaging/windows/entry.py")], pathex=[str(root)],
             binaries=binaries, datas=data, hiddenimports=hidden,
             excludes=unused_qt + ["PyQt5", "PyQt6", "PySide2", "IPython", "jupyter", "notebook",
                       "pytest", "mypy", "tkinter"], noarchive=False)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="URDF2DT", console=False,
          debug=False, strip=False, upx=False, icon=str(root / "build/windows/urdf2dt.ico"))
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="URDF2DT")
