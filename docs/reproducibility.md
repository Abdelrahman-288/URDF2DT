# Reproducing the documented results

Use a checkout containing Stage 20 and record its exact commit. Python 3.10 is
the declared minimum; native development and notebook verification used Windows
Python 3.12. Hosted CI tests Ubuntu 3.10/3.12 and Windows 3.12. macOS native graphics
and standalone clean-machine distribution are not verified by this evidence.

## Environment and checks

From the repository root in PowerShell, create a virtual environment if one does
not already exist. Use its interpreter for every command:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev,research,ui,desktop]"
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pip freeze > outputs/environment-rerun.txt
git rev-parse HEAD
git status --short
.\.venv\Scripts\python.exe -m pytest -q -ra
.\.venv\Scripts\python.exe -m mypy
```

Skip environment creation when preserving an existing environment. Dependency
ranges are not a lockfile; record resolved versions. Research JSON also records
versions, configuration, source and implementation fingerprints. Historical runs
may have dirty-tree provenance; fingerprints and source hashes identify their
inputs more precisely than a branch name. Do not claim a fresh installation from
a fresh-kernel test alone.

## Configuration and numerical budgets

`load_config()` returns built-in defaults; pass a YAML path explicitly to load
another configuration. It does not automatically discover a working-directory
file. Unknown/duplicate keys and invalid/nonfinite values are rejected.

| Setting | Default | Meaning |
|---|---:|---|
| geometry.parallel_threshold | 1e-5 | Dimensionless normalized cross magnitude |
| geometry.intersection_threshold | 1e-6 m | Nonparallel axis distance |
| geometry.common_normal_threshold | 1e-6 m | Coincidence classification / geometric distance budget |
| validation.position_tolerance_m | 1e-4 m | Maximum end-effector position error |
| validation.orientation_tolerance_rad | 1e-4 rad | Maximum end-effector orientation error |
| validation.samples | 50 | Total sampled poses, including zero |
| validation.random_seed | 42 | Python Random seed |
| ui.live_preview | true | Stored preference; current notebook uses explicit Preview |
| logging.level | INFO | Application log level |
| reference_status | provisional | Thresholds await reference/advisor reconciliation |

Representation checks use 1e-9 separately from these research tolerances.
Axis round-off uses 64 times machine epsilon; the common-normal helper preserves
its historical 1e-14 parallel cutoff. These guards are not tunable substitutes
for experimental tolerances. UI slider bounds are display intervals, not complete
legal domains. Changing thresholds requires recording the new configuration and
rerunning validation, rather than relabeling an existing report as confirmed.

## Worked UR5 and native interface

```powershell
.\.venv\Scripts\python.exe -m jupyterlab examples/ur5_full_pipeline.ipynb
.\.venv\Scripts\python.exe scripts/verify_stage19.py --output outputs/tmp/ur5-rerun.ipynb
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/ur5/ur5_serial.urdf
```

The notebook runner uses a fresh kernel with the invoking interpreter and must
write a new filename. It exercises a 0.025 m F2 frame edit, rejection, acceptance,
50-pose validation, six-file export and exact state reload. Each notebook run
creates a unique ignored `outputs/notebooks/ur5_worked_*` directory.
The optional live notebook editor is not enabled in unattended verification.
VTK rendering requires working graphics. The desktop offers actual joint sliders;
its light/dark theme changes do not affect kinematic results.

## Ablations and the second robot

Always choose new output folders:

```powershell
.\.venv\Scripts\python.exe scripts/run_stage14.py robots/ur5/ur5_serial.urdf --output outputs/research/ur5-rerun
.\.venv\Scripts\python.exe scripts/verify_stage15.py --output outputs/generalization/scara-rerun --desktop
```

Omit `--desktop` for headless SCARA verification. The SCARA run evaluates 200
poses against an independent analytic oracle and validates/reloads its archive.
The ablation runner records five classification thresholds, 84 legal edit points,
three sample densities, a deliberately faulty model, and 24 synthetic boundary
probes. The faulty model deliberately bypasses acceptance only inside the
experiment; it is never exported as validated.

For the second UR5 description, first obtain the pinned external source as
specified in [the desktop reference setup](stages/13_desktop_editor.md).
The reference revision is `89314a5976b04bf3a26d4a721e22b1b6a99916d5`:

```powershell
.\.venv\Scripts\python.exe scripts/run_stage14.py references/doctor/universalUR5.urdf --chain 0 --output outputs/research/doctor-rerun
```

No reference meshes or MATLAB installation are required for that numerical run.
Running it does not execute or verify the MATLAB scripts. Compare source hashes,
selected chain, sample sets, settings and error summaries with the committed
records. Do not require equal timestamps, absolute paths or whole-model hashes
across machines. Archive reload may reject fresh floating-point reports that
differ across library/platform versions; it deliberately fails closed.

## Known failures and recovery

| Symptom | Meaning and next action |
|---|---|
| Missing yaml, Qt, widgets or VTK module | Use the project interpreter and install the appropriate extras |
| Invalid XML, cycle, disconnected links, unsupported joint | Fix the source; structural failure must not fall through to a default robot |
| Branching source rejected by serial pipeline | Select a serial path in the desktop first |
| Near-boundary frame has no controls | Inspect geometry and recorded review flag; do not bypass it by raising tolerances |
| Local edit rejected | Use only exposed freedoms; inspect factorization and joint-line diagnostics |
| Validation/save refused | Accept every frame and resolve pending previews; an upstream edit invalidates successors |
| FK failure | Inspect source, joint order, signs, base/tool alignment and report; prefix localization is only a diagnostic |
| Existing export folder | Choose a new directory; exports do not overwrite earlier evidence |
| Archive rejected on reload | Compare saved configuration/source and dependency versions; retain the original artifact |
| Missing mesh / STL proxy notice | Resolve local assets; kinematic validity does not imply visual fidelity |
| Slow collision decomposition | Synchronous mesh operation; generated parts are view-only |
| Blank or failed rendering | Check local graphics/VTK; headless numerical tests do not verify the GPU/display path |

Input resource defaults are 5 MiB, 20,000 XML elements and depth 100. DTD/entities
are rejected. Xacro expansion, closed loops, floating/planar joints and coupled
mimic dynamics are not supplied by the serial solver. Collision decomposition is
not collision-free motion planning, and no dynamics/physical calibration claim
follows from the FK tests.
