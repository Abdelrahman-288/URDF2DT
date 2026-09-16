# Stage 19: Final worked notebook

[UR5 full pipeline](../../examples/ur5_full_pipeline.ipynb) demonstrates all thirteen
planned steps: imports/configuration, source selection, structural validation,
parsing, automatic Standard DH, the baseline table, 3-D scene, classification,
guided editing, final table, global FK, metrics, and export artifacts.

## Run locally

From the repository root:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[ui]"
.\.venv\Scripts\python.exe -m jupyterlab examples/ur5_full_pipeline.ipynb
```

Choose a kernel using the project virtual environment, then **Restart Kernel and
Run All**. The first cell prints its interpreter path. Start Jupyter from the root
or examples directory. Local VTK/OpenGL rendering is required for the scene.
Every execution creates a new `outputs/notebooks/ur5_worked_*` directory, containing
the PNG, verification summary, and six-file session bundle. These generated runs
are ignored by Git; the committed notebook retains readable executed outputs.

The optional final cell opens an independent live editor when
`OPEN_LIVE_EDITOR = True`. It is disabled during unattended execution. The
scripted guided edit remains fully exercised: preview, reject without mutation,
re-propose, accept, and confirm all remaining frames. Native joint sliders are
available through the desktop command included in the notebook.

## Verification

```powershell
.\.venv\Scripts\python.exe scripts/verify_stage19.py --output outputs/tmp/ur5_executed.ipynb
```

Choose a new output filename for each verification. The runner starts a fresh
kernel with the invoking Python interpreter and executes from `examples`, rejecting
cell failures and requiring an embedded PNG. The existing project environment was
used, not a newly installed environment. All **14 code cells passed** without code
repair or manual button clicks. The rendered scene was visually inspected.

The example applies a 0.025 m compensated translation to F2, retains the automatic
baseline, and accepts all six frames. Both baseline and edited sampled FK pass;
the maximum edited position error is approximately 4.44e-16 m and orientation
error 2.98e-8 rad. Reload reproduces the complete editor state. All six expected
export files exist. [Recorded results](../../outputs/review/stage19_notebook.json)
retain the source hash and metrics.

GitHub's existing headless regression matrix does not execute this VTK notebook.
The native fresh-kernel verification above is separate evidence. The optional
live-widget cell was not executed in that run; existing UI tests cover callbacks.

## Limits and next step

This is the bundled serial UR5 fixture, not a new MATLAB/reference reconciliation.
The schematic scene is not a CAD mesh rendering. Sampled FK does not establish
continuous-space equivalence, and tolerance/reference acceptance remains pending.
Stage 20 is final documentation and the research record.
