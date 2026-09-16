"""Execute the worked notebook in a fresh kernel using this Python interpreter."""

import argparse
import json
from pathlib import Path
import sys

import nbformat
from jupyter_client import KernelManager
from nbclient import NotebookClient


def main() -> None:
    """Fail on cell errors and retain an executed notebook as review evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="New executed .ipynb path; existing files are preserved")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new output file")
    root = Path(__file__).resolve().parents[1]
    source = root / "examples/ur5_full_pipeline.ipynb"
    notebook = nbformat.read(source, as_version=4)
    nbformat.validate(notebook)
    manager = KernelManager(kernel_name="python3")
    manager.kernel_spec.argv = [sys.executable, "-m", "ipykernel_launcher",
                                "-f", "{connection_file}"]
    client = NotebookClient(notebook, km=manager, timeout=180,
                            resources={"metadata": {"path": str(root / "examples")}})
    try:
        client.execute()
    finally:
        if manager.has_kernel:
            manager.shutdown_kernel(now=True)
    code_cells = [cell for cell in notebook.cells if cell.cell_type == "code"]
    assert all(cell.execution_count is not None for cell in code_cells)
    assert not any(output.output_type == "error"
                   for cell in code_cells for output in cell.outputs)
    assert any("image/png" in output.get("data", {})
               for cell in code_cells for output in cell.outputs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    nbformat.write(notebook, args.output)
    print(json.dumps({"notebook": str(args.output), "code_cells": len(code_cells),
                      "errors": 0, "embedded_scene": True,
                      "python": sys.executable}, indent=2))


if __name__ == "__main__":
    main()
