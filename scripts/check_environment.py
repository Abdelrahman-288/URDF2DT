"""Check Stage 0 dependencies and optionally exercise offscreen VTK rendering."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", type=Path, help="Write an offscreen sphere PNG")
    args = parser.parse_args()
    print(f"Python: {sys.version}")
    if sys.version_info < (3, 10):
        print("Python 3.10 or newer is required.")
        return 1
    failed = []
    for name in (
        "numpy", "scipy", "casadi", "pandas", "pyvista", "ipywidgets",
        "pythreejs", "jupyterlab", "yaml", "defusedxml", "pytest", "mypy",
        "trame", "trame_vtk", "trame_vuetify",
    ):
        try:
            module = importlib.import_module(name)
            print(f"{name}: {getattr(module, '__version__', 'import OK')}")
        except Exception as exc:
            failed.append(name)
            print(f"{name}: FAILED ({exc})")
    if failed:
        return 1
    import casadi as ca

    x = ca.SX.sym("x")
    square = ca.Function("square", [x], [x * x])
    if float(square(3)) != 9.0:
        raise RuntimeError("CasADi numeric evaluation failed")
    if args.render:
        import pyvista as pv

        args.render.parent.mkdir(parents=True, exist_ok=True)
        plotter = pv.Plotter(off_screen=True, window_size=(640, 480))
        try:
            plotter.add_mesh(pv.Sphere(), color="steelblue")
            plotter.show(screenshot=str(args.render))
        finally:
            plotter.close()
        print(f"Render saved: {args.render.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
