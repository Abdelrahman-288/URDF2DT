# URDF2DT

[![Tests](https://github.com/Abdelrahman-288/URDF2DT/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/Abdelrahman-288/URDF2DT/actions/workflows/tests.yml?query=branch%3Amain)

Constraint-aware Standard-DH frame editor for serial robots, with an immutable automatic reference model and local geometry/global forward-kinematics validation.

Includes a native desktop application with live robot motion and DH previews,
notebook and scripted workflows, validated session persistence, and reproducible
UR5 research ablations. Implementation now extends through Stage 18; external
reference reconciliation and advisor acceptance remain pending.
See [Stage 18 review](docs/stages/18_interface_review.md) for interface fixes and verification.
See [the project plan](docs/URDF2DT_Final_Plan.md) and
[Stage 3 decisions and contracts](docs/stages/03_dh_editor.md).

See [Stage 4 behavior and limits](docs/stages/04_urdf_input.md) for input validation.
See [Stage 5 conventions and evidence](docs/stages/05_parser_integration.md) for
automatic DH construction, base/tool alignment, and current numerical limits.

## Desktop application (Stage 13 extension)

A native window now provides live joint sliders, constrained DH previews, robot
meshes, chain selection, visibility/opacity controls, and persistent light/dark mode.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[desktop]"
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/ur5/ur5_serial.urdf
```

Or double-click **Launch URDF2DT.cmd** and choose **Open URDF**.
See [the desktop guide](docs/stages/13_desktop_editor.md) for the doctor's mesh-backed
UR5 example, control instructions, verification, limitations, and Stage 14 handoff.

## Research ablation studies (Stage 14)

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[research]"
.\.venv\Scripts\python.exe scripts/run_stage14.py robots/ur5/ur5_serial.urdf --output outputs/research/my_stage14_run
```

Use a new output directory. [Stage 14 methods and results](docs/stages/14_research_ablations.md)
cover threshold sensitivity, legal edit sweeps, sample density and synthetic
boundaries. These are sampled numerical experiments with provisional tolerances.

## Second robot (Stage 15 technical verification)

The original four-axis SCARA fixture adds a downward prismatic slide and an
independent closed-form FK oracle. Its automated pipeline and desktop checks pass;
the user subsequently confirmed successful unaided frame acceptance.

```powershell
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/scara/scara_rrpr.urdf
```

See [Stage 15 evidence and usability observation](docs/stages/15_generalization.md).

[Stage 16 invariant regressions](docs/stages/16_invariant_regressions.md) exercise
seeded multi-edit sequences across both robots, including rotation validity,
FK preservation, baseline immutability, restores, and archive round trips.

## Interactive notebook editor (Stage 10)

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[ui]"
.\.venv\Scripts\python.exe -m jupyterlab examples/interactive_editor.ipynb
```

Run the code cell, then use Load path, Preview, Accept and Unlock next. See
[the UI workflow and limits](docs/stages/10_interactive_editor.md).

After accepting all frames, click **Validate FK**. See
[Stage 11 conventions, reports and limitations](docs/stages/11_global_fk_validation.md).
Regenerate the automatic/edited UR5 evidence with
`.\.venv\Scripts\python.exe scripts/verify_stage11.py`.

Stage 12 adds **Save session / Load session** to the notebook. Use a new output
folder; save and reload both recheck sampled FK. See
[archive contents, API and logging](docs/stages/12_export_persistence.md).

For a scripted end-to-end run:

```powershell
.\.venv\Scripts\python.exe app.py robots/ur5/ur5_serial.urdf --output outputs/sessions/ur5_run --edit 2:0.05:0
```

Unspecified frames are confirmed unchanged. See [Stage 13 workflow and API](docs/stages/13_end_to_end_application.md).

See [Stage 8 session API and transition rules](docs/stages/08_editor_session.md)
for sequential unlocking, proposals, acceptance/rejection and restores. Changed
raw rows require an injected validator. Stage 9 provides the built-in `FrameEdit`
path; see [recomputation usage and limits](docs/stages/09_dh_recomputation.md).

Stage 7 geometric classification is also implemented, with reference acceptance
pending the DH report and MATLAB inputs. See [Stage 7 status and geometry](docs/stages/07_geometric_classification.md).
Run `.\.venv\Scripts\python.exe -m urdf2dt robots/ur5/ur5_serial.urdf --classify`
to inspect cases and local frame freedoms.

## Render a static scene

Install the optional renderer with `python -m pip install -e ".[ui]"`.

```powershell
python -m urdf2dt.visualization robots/ur5/ur5_serial.urdf --selected-frame F2 --output outputs/scenes/ur5_zero_pose.png
python -m urdf2dt.visualization robots/ur5/ur5_serial.urdf
```

The second command opens a window with camera orbit and zoom. Use `--q` followed
by movable joint coordinates (radians/metres) for another pose; the default is
zero. `--hide-urdf`, `--hide-dh`, and `--hide-labels` simplify dense views.
See [Stage 6 geometry, API and verification](docs/stages/06_static_scene.md).

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

## Continuous integration

[Stage 17 CI](docs/stages/17_continuous_integration.md) runs pytest and mypy on
Ubuntu (Python 3.10/3.12) and Windows (Python 3.12). The badge follows the Stage 17
branch until these changes are merged into the default branch. Native desktop
rendering remains a separately verified local workflow.

## Next dependencies

- Continue with Stage 18 code quality and interface review.
  Reference labels/rules still need reconciliation.
- The supplied doctor repository now provides the project URDF and MATLAB files
  in a local reference checkout. Execute/reconcile MATLAB comparisons and obtain
  the DH technical report and architecture reference for remaining acceptance checks.
- Confirm reference tolerances before implementing geometry-dependent behavior.
