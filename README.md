# URDF2DT

Constraint-aware Standard-DH frame editor for serial robots, with an immutable
automatic reference model and local geometry/global forward-kinematics validation.

Stage 5 now converts validated serial URDF into a real immutable Standard-DH
model, with numeric URDF/DH FK and independent integration checks. Frame editing,
visualization, and the formal global-validation service remain future work.
See [the project plan](docs/URDF2DT_Final_Plan.md) and
[Stage 3 decisions and contracts](docs/stages/03_dh_editor.md).

See [Stage 4 behavior and limits](docs/stages/04_urdf_input.md) for input validation.
See [Stage 5 conventions and evidence](docs/stages/05_parser_integration.md) for
automatic DH construction, base/tool alignment, and current numerical limits.

## Generate automatic DH

```powershell
python -m urdf2dt robots/ur5/ur5_serial.urdf
python -m urdf2dt robots/ur5/ur5_serial.urdf --json
```

The output includes base/tool transforms and source provenance. It is an automatic
baseline, not an edited or globally certified export. No UR5 table is used as a
production fallback; ill-conditioned near-parallel geometry is explicitly rejected.

```python
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.kinematics import dh_fk, urdf_fk

run = generate_automatic_model("robots/ur5/ur5_serial.urdf")
baseline = run.automatic_model
q = (0.0,) * len(baseline.rows)
print(baseline.rows)
print(urdf_fk(run.chain, q))
print(dh_fk(baseline, q))
```

## Validate a robot file

After installing the package, run:

```powershell
python -m urdf2dt.parser robots/ur5/ur5_serial.urdf
python -m urdf2dt.parser robots/ur5/ur5_serial.urdf --json
```

The bundled serial fixture has six movable joints. The unchanged upstream UR5
description is also included and is intentionally rejected for auxiliary branches;
see [fixture provenance](robots/ur5/README.md). Acceptance checks structure and
kinematic input fields only; it does not certify DH solvability or FK equivalence.

From Python:

```python
from urdf2dt.parser.urdf_validator import validate_urdf

result = validate_urdf("robots/ur5/ur5_serial.urdf")
document = result.require_valid()
print(document.movable_joint_names)
```

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

- Implement the static scene renderer, then the classifier and editor stages.
- Supply the project-specific `universalUR5.urdf`, MATLAB reference files, DH technical report, and architecture
  reference before their associated integration/comparison stages.
- Confirm reference tolerances before implementing geometry-dependent behavior.
