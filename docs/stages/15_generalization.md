# Stage 15: Structurally different robot

**Status: automated technical work verified; independent human usability
observation pending. Do not mark the complete stage finished until that observation
is recorded.**

The new [original SCARA fixture](../../robots/scara/README.md) is a four-axis RRPR
robot, compared with the UR5's six revolute joints. The prismatic axis points down,
which exercises antiparallel axes and mixed metre/radian controls. Both fixed
base and tool transforms are nontrivial. It needs no external meshes.

## Run locally

```powershell
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/scara/scara_rrpr.urdf
```

Reproduce the full automated record into a new directory:

```powershell
.\.venv\Scripts\python.exe scripts/verify_stage15.py --output outputs/generalization/scara_rerun --desktop
```

The native check needs `.[desktop]` installed and a working display. Omit
`--desktop` for headless source-to-archive and analytic checks.

## Verification scope

The pipeline validates and parses the URDF, builds four Standard-DH rows,
classifies their geometry, accepts legal translation edits at every frame and
rotation edits at F3/F4, compares URDF/DH FK over 200 deterministic samples,
exports a validated session, reloads it, and revalidates it. The regression tests
also exercise full downward stroke, an explicit bent pose, upstream re-edit
invalidation, the export/validation gate, and exact baseline restoration.

A separate closed-form SCARA calculation checks all matrix elements of URDF FK,
automatic DH FK and edited DH FK. This avoids relying solely on agreement between
two production paths. The full report retains the sampled FK evidence in the
session archive; the verification JSON summarizes the independent oracle check.

The native test checks four joint controls, a prismatic slider endpoint of 0.18 m,
six primitive visual actors, actor reuse, live DH edits, and sampled FK acceptance.
Screenshots cover light and dark themes.

A search and execution review found no six-joint/UR5 dependency in the production
solver, acceptance, persistence or native control construction for this fixture.
No production algorithm changes were needed. The existing UR5-specific example
verification scripts remain explicitly fixture-specific; they are not application
constraints.

## Results

- [Machine-readable verification](../../outputs/generalization/stage15_scara/verification.json)
- [Validated exported session](../../outputs/generalization/stage15_scara/session/session.json)
- [Desktop dark view](../../outputs/generalization/stage15_scara/desktop_dark.png)
- [Desktop light view](../../outputs/generalization/stage15_scara/desktop_light.png)

The independent 200-sample maximum matrix-element residual was
`7.771561172376096e-16`. Edited sampled FK had maximum position error
`3.5135752366389115e-16 m` and orientation error `3.6500241499888574e-8 rad`.
The latter uses the existing clamped trace/acos metric and includes its numerical
floor. These numbers establish numerical agreement for this idealized fixture,
not physical robot accuracy or continuous-space proof. Reference tolerances and
report/MATLAB reconciliation remain provisional.

## Human usability observation

The Stage 15 plan also calls for a person other than the implementation author to
try loading a robot and accepting a frame using only the interface. Automated
callbacks and screenshots do not substitute for that observation. Record the
participant's report in [the usability note](15_usability_observation.md), including
hesitations and requests for help. The user has been asked to supply this evidence;
no participant outcome is invented here.

Stage 16 (stronger invariant and regression testing) follows after this remaining
Stage 15 observation is recorded and any resulting fixes are verified.

## Final automated verification

292 tests passed. Mypy checked 36 package files without errors, and pip check
reported no broken requirements. The native SCARA run completed successfully with
four joint controls, six visual actors, no mesh warnings, and equal restored
session state. The dark desktop screenshot was inspected after adjusting only
the schematic slide rod geometry to remain connected throughout its travel.

The user confirmed successful local launch on 2026-09-15. Their frame-acceptance
observation remains pending in the usability note.
