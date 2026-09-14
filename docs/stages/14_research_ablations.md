# Stage 14: Research ablation studies

The headless experiment runner uses the existing solver, classifier, real sequential
editor acceptance, local frame checks, and sampled global FK validator. It does not
relax production gates or use desktop slider positions as experimental evidence.

## Reproduce

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[research]"
.\.venv\Scripts\python.exe scripts/run_stage14.py robots/ur5/ur5_serial.urdf --output outputs/research/stage14_bundled_rerun
.\.venv\Scripts\python.exe scripts/run_stage14.py references/doctor/universalUR5.urdf --chain 0 --output outputs/research/stage14_doctor_rerun
```

Use a new output folder each time. For the doctor's external checkout, follow the
[pinned reference setup](13_desktop_editor.md#doctor-s-reference-repository).
The original tree hash, selected serial-chain hash and source repository revision
are recorded independently. No external meshes are needed for these experiments.

## Experimental controls

- **Threshold sensitivity:** reclassify the same automatic DH model with parallel
  thresholds `1e-6, 3e-6, 1e-5, 3e-5, 1e-4`. Keep distance thresholds and FK
  tolerances fixed. This isolates classifier sensitivity; it is not a solver
  threshold ablation. Record coincidence, review flags, and available freedoms.
- **Legal sweeps:** discover every currently editable frame/parameter. Test 21
  equally spaced points, including zero and endpoints. Axial translation is
  tested over `[-1,1]` m; axial rotation over `[-pi,pi]` rad. Each point starts a
  fresh session, accepts all frames sequentially, checks the changed frame locally,
  then evaluates the same 50 joint poses. Rejected points remain in the record.
- **Sample density:** compare automatic and combined legally edited models at
  10, 50, and 200 total joint poses. Seed 42, zero first, nested random samples.
  Also test a deliberately corrupted model with `a += 0.001 m` at F3. That negative
  control bypasses acceptance only inside the research runner and is never exported
  as a validated model. Its expected result is failure.
- **Synthetic boundaries:** probe parallelism, intersection distance and coincidence
  distance at 0, 0.1, 0.5, 0.999, 1, 1.001, 2, and 10 times each default threshold.
  Save the actual origins/directions, measured sine/distance, classification and
  edit permissions. Threshold equality uses the existing inclusive classifier rule.

The UR5 exposes translation at F2/F3 and translation/rotation at terminal F6, giving
84 sweep points. F6 includes the solver's synthetic terminal z-axis convention;
it is not an additional physical robot joint.

## Artifacts and evidence

Each run writes `results.json`, CSV tables, two PNG plots, and `summary.md`.
JSON includes input hashes, automatic/edited/control models, tested edits, full
sample sets, per-sample end-effector errors, summaries/worst indices, configuration,
dependency versions, Git provenance and SHA-256 hashes of the actual source code.
The code fingerprints identify implementation files even if an unrelated notebook
leaves the working tree dirty. Paths/timestamps and whole-model hashes may differ
between machines; compare source-byte hashes, settings and numerical results.

## Interpretation limits

Axial translation is unbounded: no finite interval is the complete legal domain.
Rotation covers one period at discrete points. Neither sweep proves continuous
coverage or every simultaneous edit combination. A sampled FK PASS is conditional
on the recorded tolerances (`1e-4` m and `1e-4` rad) and joint samples.

The orientation metric is clamped trace/acos, which can produce a floating-point
floor near `1e-8` rad. These numerical residuals are not physical robot accuracy.
Near-threshold classifications requiring review do not unlock continuous edits.
Local checks use rigidity, directed joint-line preservation and common-normal
conditions; their report-specific R1–R3 labels remain unreconciled.

This stage supplies reproducible internal numerical experiments. It does not
claim MATLAB execution, external implementation equivalence, confirmed thresholds,
or advisor acceptance. The next implementation stage is Stage 15: a structurally
different second robot and a separate human usability check.
