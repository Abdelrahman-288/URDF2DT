# URDF2DT

Constraint-aware Standard-DH frame editor for serial robots, with an immutable
automatic reference model and local geometry/global forward-kinematics validation.

Stage 2 package scaffolding and the Stage 3 configuration/data contracts are
implemented. The parser, solver, editor, and FK validator remain future work.
See [the project plan](docs/URDF2DT_Final_Plan.md) and
[Stage 3 decisions and contracts](docs/stages/03_dh_editor.md).

Repository: https://github.com/Abdelrahman-288/URDF2DT

Dependency imports, CasADi evaluation, and PyVista offscreen rendering have passed.
For the exact verified Windows/Python 3.12 environment, install
`requirements-windows-py312.lock.txt` instead of `requirements-dev.txt`.

## Development environment (Windows PowerShell)

With Python 3.10 or newer installed:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m mypy
```

Activation is optional; invoking the environment's Python directly avoids shell
execution-policy changes. Select `.venv\Scripts\python.exe` in your editor.

The first setup used the Python 3.12 runtime bundled with Codex to create `.venv`.
Recreate the environment with an independently installed Python if that runtime
is removed or relocated.

For notebook/3-D dependencies, install `.[dev,ui]`. For the complete original
development environment (including pandas), use `requirements-dev.txt`.

## Stage 3 usage

```python
from dataclasses import replace
from urdf2dt.config import load_config
from urdf2dt.dh.types import DHModel, DHRow

config = load_config("config/default.yaml")  # explicit repository config
assert config.reference_status == "provisional"
baseline = DHModel(
    robot_name="synthetic",
    rows=(DHRow(a=0.2, alpha=0.0, d=0.1, theta_offset=0.0, joint_name="j1"),),
    provenance="Synthetic example; no URDF or FK validation",
)
working = replace(baseline, rows=(replace(baseline.rows[0], a=0.25),))
assert baseline.rows[0].a == 0.2  # original remains unchanged
```

`load_config()` uses equivalent built-in defaults from any working directory.
`config.snapshot()` returns an independent JSON-compatible record of all values.
Replacing a data value does not perform geometric validation or accept an editor
action; these functions belong to later stages.

The schema in `schemas/dh-model-0.1.schema.json` is a provisional model envelope,
not a validated-session export. The classic UR5 fixture lives only under
`tests/fixtures/`; it has no verified URDF/MATLAB frame alignment.

## Next dependencies

- Implement the parser and DH solver; the user confirmed no existing modules.
- Supply the UR5 URDF, MATLAB reference files, DH technical report, and architecture
  reference before their associated integration/comparison stages.
- Confirm reference tolerances before implementing geometry-dependent behavior.
