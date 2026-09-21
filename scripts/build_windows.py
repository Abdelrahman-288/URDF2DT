"""Build a one-folder Windows desktop distribution and retain dependency notices."""

from importlib.metadata import distributions
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main() -> None:
    """Prepare reproducible assets, invoke the spec and add user documentation."""
    root = Path(__file__).resolve().parents[1]
    if sys.platform != "win32":
        raise RuntimeError("Build on Windows using the pinned Windows environment")
    build = root / "build/windows"
    build.mkdir(parents=True, exist_ok=True)
    from urdf2dt.export.schemas import SESSION_SCHEMA
    from urdf2dt.logging_config import git_provenance
    metadata = git_provenance()
    metadata.update(python=sys.version, distribution="stage23-windows-candidate",
                    clean_machine_verified=False)
    (build / "build-info.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    (build / "session.schema.json").write_text(json.dumps(SESSION_SCHEMA, indent=2), encoding="utf-8")
    from PySide6.QtGui import QImage, QPainter
    from PySide6.QtSvg import QSvgRenderer
    from PIL import Image
    canvas = QImage(256, 256, QImage.Format_ARGB32)
    canvas.fill(0)
    painter = QPainter(canvas)
    QSvgRenderer(str(root / "assets/urdf2dt.svg")).render(painter)
    painter.end()
    canvas.save(str(build / "icon.png"))
    Image.open(build / "icon.png").save(build / "urdf2dt.ico", sizes=[(16, 16), (32, 32), (48, 48), (256, 256)])
    # Unrelated native toolchains on PATH can supply incompatible Qt dependencies.
    # Keep Windows system DLLs and this isolated Python environment discoverable.
    build_env = os.environ.copy()
    windows = Path(os.environ["SystemRoot"])
    build_env["PATH"] = os.pathsep.join(map(str, (
        Path(sys.executable).parent, Path(sys.base_prefix), windows / "System32", windows)))
    for variable in ("PYTHONPATH", "PYTHONHOME", "QT_PLUGIN_PATH", "QML2_IMPORT_PATH"):
        build_env.pop(variable, None)
    subprocess.run([sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
                    "--distpath", str(root / "dist"), "--workpath", str(build / "pyinstaller"),
                    str(root / "packaging/windows/URDF2DT.spec")], cwd=root, env=build_env, check=True)
    release = root / "dist/URDF2DT"
    shutil.copyfile(root / "packaging/windows/README.txt", release / "README.txt")
    shutil.copyfile(root / "docs/desktop_improvements.md", release / "DESKTOP_GUIDE.md")
    shutil.copyfile(root / "packaging/windows/Verify URDF2DT.cmd", release / "Verify URDF2DT.cmd")
    shutil.copyfile(root / "packaging/windows/CLEAN_PC_TEST.txt", release / "CLEAN_PC_TEST.txt")
    shutil.copyfile(root / "LICENSE", release / "LICENSE.txt")
    shutil.copyfile(build / "build-info.json", release / "build-info.json")
    shutil.copytree(root / "robots/scara", release / "examples/scara", dirs_exist_ok=True)
    (release / "examples/ur5").mkdir(parents=True, exist_ok=True)
    for name in ("ur5_serial.urdf", "LICENSE.upstream"):
        shutil.copyfile(root / "robots/ur5" / name, release / "examples/ur5" / name)
    notices = release / "third-party"
    notices.mkdir(exist_ok=True)
    inventory = []
    for dist in sorted(distributions(), key=lambda d: d.metadata["Name"].lower()):
        name = dist.metadata["Name"]
        record = {"name": name, "version": dist.version,
                  "license": dist.metadata.get("License-Expression") or dist.metadata.get("License"),
                  "project_urls": dist.metadata.get_all("Project-URL") or [], "license_files": []}
        for item in dist.files or []:
            if not any(word in item.name.lower() for word in ("license", "copying", "notice")):
                continue
            source = Path(dist.locate_file(item))
            if not source.is_file() or source.suffix.lower() in (".py", ".pyc", ".pyd", ".dll", ".exe"):
                continue
            relative = Path(*[part for part in item.parts if part not in ("..", ".")])
            target = notices / name / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            record["license_files"].append(target.relative_to(release).as_posix())
        inventory.append(record)
    # The CPython runtime is embedded independently of pip distributions.
    python_license = Path(sys.base_prefix) / "LICENSE.txt"
    if python_license.is_file():
        shutil.copyfile(python_license, notices / "Python-LICENSE.txt")
    shutil.copyfile(root / "packaging/windows/THIRD_PARTY.md", notices / "README.md")
    shutil.copytree(root / "packaging/windows/licenses", notices / "licenses", dirs_exist_ok=True)
    (notices / "inventory.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
    (release / "SHA256SUMS").write_text("".join(
        hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.relative_to(release).as_posix() + "\n"
        for p in sorted(release.rglob("*")) if p.is_file() and p.name != "SHA256SUMS"), encoding="utf-8")
    print("Built:", release / "URDF2DT.exe")


if __name__ == "__main__":
    main()
