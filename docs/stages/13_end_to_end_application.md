# Stage 13: End-to-end URDF application

`urdf2dt.app.Application` now coordinates source loading, secure validation,
parsing, automatic DH, the authoritative EditorSession, sampled FK checks and
versioned export/reload. The notebook and scripted path use this same layer.

## Interactive workflow

```powershell
.\.venv\Scripts\python.exe -m jupyterlab examples/interactive_editor.ipynb
```

Run the cell and select/load a URDF. Preview and accept frames sequentially,
using only available controls. Run Validate FK when all are accepted; choose a
new Session folder and Save session. Load session can later resume that archive.
Saving independently revalidates, so a stale displayed PASS cannot bypass the gate.

The notebook uses `launch_editor()` to show the start screen. Importing the core
application does not load widgets or graphics. The notebook still provides camera
viewing and guided editing; the command below is a separate scripted workflow.

## Scripted workflow

```powershell
.\.venv\Scripts\python.exe app.py robots/ur5/ur5_serial.urdf --output outputs/sessions/ur5_run --edit 2:0.05:0 --edit 6:0.03:0.2
```

Equivalent installed entry point: `python -m urdf2dt.app ...`.
Each `--edit FRAME:TRANSLATION_M:ROTATION_RAD` specifies an incremental legal edit.
Unspecified frames are explicitly confirmed unchanged by the scripted workflow.
Repeated/out-of-range frame indices and forbidden geometric controls are rejected.
Optional `--config` loads a complete configuration YAML. Existing output folders
are never overwritten. The CLI returns zero only after the validated bundle is
saved; ordinary input/validation/output errors return one with a log message.

If sampled FK fails, no validated bundle is written. Optional `--debug-report`
writes that failed report to a new JSON file, retaining passed=false, sample
metrics, diagnostics and provenance. It does not relax any export gate. A debug
report is available only for a completed run that reaches sampled validation,
not an input-parser failure or a rejected local edit.

## Core API

```python
from urdf2dt.app import Application
from urdf2dt.dh.recompute import FrameEdit

app = Application("robots/ur5/ur5_serial.urdf")
for i in range(1, len(app.run.automatic_model.rows) + 1):
    app.session.unlock_next()
    app.session.propose_edit(i, FrameEdit())
    app.session.accept()
report = app.validate()
path = app.export("outputs/sessions/confirmed_baseline")
resumed = Application.resume(path)
```

## Verification

272 tests pass, with mypy clean across 31 package files. Non-UI integration tests
exercise edited UR5 from source to archive/reload, incomplete/pending rejection,
scripted edits and invalid indices. A deliberately faulty injected row validator
permits a corrupted model through local acceptance; global FK fails and export
still refuses it without creating an output folder. Existing notebook callback
tests cover shared application validation, save/reload and session synchronization.

Reference-specific report/MATLAB reconciliation and advisor acceptance remain
pending. All PASS labels mean sampled evidence under recorded tolerances, not a
universal proof. Stage 14 research studies are next.
