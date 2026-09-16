"""Run release-candidate checks with the invoking environment and retain evidence."""

import argparse
import json
import os
from pathlib import Path
import platform
from importlib.metadata import version
import subprocess
import sys
from xml.etree import ElementTree


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--desktop", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    tracked = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root, text=True)
    if tracked.strip():
        raise RuntimeError("Commit tracked changes before candidate verification")
    environment = dict(os.environ, PYTHONUTF8="1", MPLBACKEND="Agg")

    def run(label, *command):
        print(label, flush=True)
        with (output / (label + ".log")).open("w", encoding="utf-8") as log:
            subprocess.run([sys.executable, *command], cwd=root, env=environment,
                           stdout=log, stderr=subprocess.STDOUT, check=True)

    run("dependencies", "-m", "pip", "check")
    run("tests", "-m", "pytest", "-q", "-ra", "-o", "xfail_strict=true", "--junitxml=" + str(output / "tests.xml"))
    xml = ElementTree.parse(output / "tests.xml")
    cases = xml.findall(".//testcase")
    assert cases and not xml.findall(".//skipped") and not xml.findall(".//failure") and not xml.findall(".//error")
    run("typing", "-m", "mypy")
    run("notebook", "scripts/verify_stage19.py", "--output", str(output / "ur5_executed.ipynb"))
    run("scara", "scripts/verify_stage15.py", "--output", str(output / "scara"),
        *(["--desktop"] if args.desktop else []))

    import nbformat
    from shutil import copytree
    from jsonschema import Draft202012Validator
    from urdf2dt.export.schemas import SESSION_SCHEMA, SCHEMA_VERSION

    nb = nbformat.read(output / "ur5_executed.ipynb", as_version=4)
    # Read the explicit archive path emitted by this execution, not a previous run.
    archive_lines = [line.split("Archive:", 1)[1].strip()
                     for cell in nb.cells if cell.cell_type == "code"
                     for item in cell.outputs for line in item.get("text", "").splitlines()
                     if line.startswith("Reloaded state matches. Archive:")]
    assert len(archive_lines) == 1
    archive = Path(archive_lines[0]).resolve()
    assert archive.is_relative_to(root / "outputs/notebooks")
    copytree(archive.parent.parent, output / "ur5")
    for robot in ("ur5", "scara"):
        data = json.loads((output / robot / "session/session.json").read_text(encoding="utf-8"))
        Draft202012Validator(SESSION_SCHEMA).validate(data)
        assert data["schema_version"] == SCHEMA_VERSION == "1.0"
        assert data["dh_convention"] == "standard" and data["validation"]["passed"]
        assert data["reproducibility"]["git_commit"] == commit
    result = {"candidate_commit": commit, "python": sys.version,
              "platform": platform.platform(), "package_version": version("urdf2dt"),
              "schema_version": SCHEMA_VERSION, "tests_passed": len(cases),
              "skipped": 0, "xfail": 0, "notebook_code_cells": sum(c.cell_type == "code" for c in nb.cells),
              "scara_desktop": args.desktop, "all_checks_passed": True}
    (output / "verification.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
