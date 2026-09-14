# Stage 11: Sampled global FK validation

The automatic UR5 model and a completely accepted edited session pass the current
provisional tolerances. Report/MATLAB reconciliation and advisor acceptance remain
pending; this is reproducible sampled evidence, not a proof for all configurations.

## Usage

In the interactive notebook, accept all frames and resolve any pending preview,
then click **Validate FK**. The automatic baseline is checked first. The result
displays sample count, maximum errors and any localized failure. **Go to flagged
frame** selects the first implicated prefix when available. Subsequent proposals,
acceptance changes and restores invalidate the displayed report.

```powershell
.\.venv\Scripts\python.exe scripts/verify_stage11.py
```

This regenerates automatic/edited JSON and Markdown under
`outputs/validation_reports/stage11_*`. The edited session translates F2, F3 and
F6 by 0.05 m, rotates F6 by 0.2 rad, and confirms all six frames in order.

```python
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.global_validation import validate_global_fk
run = generate_automatic_model("robots/ur5/ur5_serial.urdf")
report = validate_global_fk(run.chain, run.automatic_model, run.config)
print(report.markdown())
```

## FK conventions and sampling

CasADi functions compile once per validation run and are reused for all samples.
FKFunctions also exposes reusable URDF, DH and prefix functions to callers.
URDF q follows movable-joint order in the validated base-to-tip chain, retaining
all intervening fixed transforms. DH uses Standard-DH Rz(theta) Tz(d) Tx(a)
Rx(alpha), explicit base/tool transforms, theta offsets and joint_sign. Prismatic
motion modifies d; revolute/continuous motion modifies theta.

The default is 50 total configurations, including zero, using Python Random with
configured seed 42. Subsequent samples honor bounded limits; continuous joints
use [-pi, pi]. Zero is included even outside physical limits and that condition
is recorded. Actual q values are saved; there is no requirement to reproduce a
particular Python RNG implementation to replay the stored samples.

Position error is Euclidean translation distance. Orientation is acos of clipped
(trace(R_urdf.T R_dh)-1)/2. The clipping prevents domain errors but acos still has
an approximately 1e-8 rad numerical floor near identity. PASS uses inclusive
configured position/orientation tolerances (currently 1e-4 m/rad), matching the
existing ValidationResult contract. Nonfinite FK results fail explicitly.

Reports include the full candidate and SHA-256 identity, source digest, joint
order, effective configuration, actual samples, per-sample end-effector/prefix
errors, means, maxima and separate worst-position/orientation sample indices.
JSON and Markdown are development reports; Stage 12 export schemas are not implied.

## Failure attribution

URDF link origins and DH frame origins are not directly comparable. For each
movable link, compute a fixed zero-pose alignment from the candidate DH frame to
its URDF link, then compare aligned prefixes at every sampled q. Report the first
prefix that exceeds tolerance across the sample set. This identifies where
motion disagrees and is not a guarantee that the indexed row caused the fault.

Constant geometry/base/tool errors may be absorbed by zero-pose prefix alignment.
The independent, unaligned end-effector comparison still detects them. If no
prefix localizes a failing tip comparison, report an explicit alignment/constant
geometry diagnostic instead of inventing a frame index; the jump button stays
disabled. Tests cover a corrupted alpha, reversed sign and tool-only fault.
This is a deliberate limitation relative to a universal frame-blame guarantee.

## Verification

- 255 tests passed; mypy clean across 27 source files; dependency check passed.
- Automatic: max position 4.44e-16 m; max orientation 2.98e-8 rad.
- Edited: max position 2.50e-16 m; max orientation 2.98e-8 rad.
- CasADi URDF FK checked against independent raw-XML/SciPy FK on UR5 and the
  mixed fixed/revolute/continuous/prismatic fixture.
- Deterministic reports, limits, clipping, pi rotations, bad identity, corruption,
  completed-session gating and stale UI report invalidation tested.

Formal reference acceptance remains pending the missing project-specific inputs.
