"""Archive, independently extract and exercise a standalone Windows candidate."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="New delivery directory outside the repository")
    parser.add_argument("--verify-existing", action="store_true", help="Recheck an already created delivery")
    args = parser.parse_args()
    if sys.platform != "win32":
        raise RuntimeError("Run the native acceptance check on Windows")
    root = Path(__file__).resolve().parents[1]
    destination = args.destination.resolve()
    if destination.is_relative_to(root):
        raise ValueError("Choose a delivery directory outside the repository")
    bundle = root / "dist/URDF2DT"
    metadata = json.loads((bundle / "build-info.json").read_text(encoding="utf-8"))
    if not metadata.get("working_tree_clean"):
        raise ValueError("Rebuild from a committed, clean source checkout first")
    if args.verify_existing:
        archive = destination / "URDF2DT-v1.0-Windows.zip"
        metadata = json.loads((destination / "URDF2DT/build-info.json").read_text(encoding="utf-8"))
    else:
        destination.mkdir(parents=True, exist_ok=False)
        archive = Path(shutil.make_archive(str(destination / "URDF2DT-v1.0-Windows"),
                                         "zip", root_dir=bundle.parent, base_dir=bundle.name))
    with archive.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    checksum_file = destination / (archive.name + ".sha256")
    if args.verify_existing:
        if checksum_file.read_text(encoding="utf-8").split()[0] != digest:
            raise ValueError("Archive checksum mismatch")
    else:
        checksum_file.write_text(digest + "  " + archive.name + "\n", encoding="utf-8")
        shutil.unpack_archive(archive, destination)
    extracted = destination / "URDF2DT"
    checked = 0
    for line in (extracted / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        expected, name = line.split("  ", 1)
        path = (extracted / name).resolve()
        if not path.is_relative_to(extracted):
            raise ValueError("Invalid manifest path")
        with path.open("rb") as stream:
            actual = hashlib.file_digest(stream, "sha256").hexdigest()
        if actual != expected:
            raise ValueError(f"Extracted file checksum mismatch: {name}")
        checked += 1
    environment = os.environ.copy()
    windows = Path(os.environ["SystemRoot"])
    environment["PATH"] = os.pathsep.join(map(str, (windows / "System32", windows)))
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "QT_PLUGIN_PATH", "QML2_IMPORT_PATH"):
        environment.pop(name, None)
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    report_dir = destination / "verification"
    completed = subprocess.run([str(extracted / "URDF2DT.exe"), "--verify-package", str(report_dir)],
                               cwd=destination, env=environment, startupinfo=startup,
                               timeout=240, check=True)
    native = json.loads((report_dir / "verification.json").read_text(encoding="utf-8"))
    if not native.get("passed") or not native.get("frozen"):
        raise RuntimeError("Frozen application verification did not pass")
    if native["provenance"]["git_commit"] != metadata["git_commit"]:
        raise RuntimeError("Runtime provenance differs from the build record")
    evidence = {"archive": archive.name, "sha256": digest,
                "archive_bytes": archive.stat().st_size, "manifest_files_verified": checked,
                "build": metadata, "native_exit_code": completed.returncode,
                "relocated_outside_repository": True, "python_and_git_removed_from_path": True,
                "clean_machine_verified": False,
                "note": "Development-host relocation test; separate clean-PC acceptance remains pending."}
    (destination / "delivery-verification.json").write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(evidence, indent=2))
    print("Ready:", extracted / "URDF2DT.exe")


if __name__ == "__main__":
    main()
