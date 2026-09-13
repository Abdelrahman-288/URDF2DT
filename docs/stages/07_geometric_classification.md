# Stage 7: Geometric classification

Implementation and automated verification are available. **Reference acceptance
is pending:** DH report Table 3, MATLAB/reference logic and the advisor checkpoint
are not available. The plan's full Stage 7 done criterion is therefore not met.

## API and command

```powershell
.\.venv\Scripts\python.exe -m urdf2dt robots/ur5/ur5_serial.urdf --classify
.\.venv\Scripts\python.exe -m urdf2dt robots/ur5/ur5_serial.urdf --classify --json
```

`urdf2dt.dh.classification` provides `classify_axis_pair(origin_a, direction_a,
origin_b, direction_b, config=None)`, `classify_dh_model(model, config=None)` and
`get_editable_params(case)`. Results are immutable. Directions are normalized;
invalid dimensions, nonfinite coordinates, zero directions and distance overflow
are rejected. No optional UI dependency is imported.

## Geometry and threshold policy

For normalized directions u and v, use s = |u cross v|. Parallel means
s <= configured parallel_threshold (including antiparallel axes). Otherwise the
line distance is |(b-a) dot (u cross v)| / s. Distances at or below the configured
intersection_threshold classify as intersecting; larger distances are skew.

In the parallel branch, transverse anchor separation is the larger of
|(b-a) cross u| and |(b-a) cross v|. Coincidence uses common_normal_threshold.
For exactly parallel lines this is the shortest distance. Near-parallel lines
instead receive an explicit review flag: their remote closest-point distance is
ill-conditioned, and the anchor-based diagnostic can change when anchors slide.

Approximate parallelism, intersection or coincidence beyond a 64-machine-epsilon
roundoff allowance locks all edit controls. That allowance is dimensionless for
angles and metres for distance, deliberately conservative for large-scale models.
Thresholds never snap or modify model geometry. The solver retains its stricter
near-parallel rejection behavior; classification does not override it.

## Local frame freedoms

| Geometry | Continuous local controls |
|---|---|
| Distinct parallel axes | Translate the origin along its z axis |
| Coincident axes | Translate along z and rotate x/y about z |
| Intersecting axes | None |
| Skew axes | None |
| Any result requiring review | None |

These describe frame assignment with fixed joint-axis directions. They are not
raw DH-row edits or independent joint motion. Later editing must compensate
adjacent rows/base/tool alignment and validate FK. Discrete axis flips, numeric
slider bounds and acceptance states are not implemented. Pass the full result
to get_editable_params so tolerance locking and coincidence are respected; a bare
PARALLEL enum only supplies axial translation.

The construction was checked against the axis/common-normal rules in
[IRIS Lab's Standard-DH lecture](https://irislab.tech/course_robotics/lec4-fk/fk.html)
and [Lehigh's frame-assignment notes](https://www.cse.lehigh.edu/~trink/Courses/Intro_to_Robotics/DHcookbook.html).
Lehigh uses different frame indices; it is a conceptual comparison, not the
project's missing label definition or a MATLAB equivalence result.

## UR5 result and unresolved reference mapping

| Frame | Descriptive case | Distance (m) | Plan label, unverified |
|---|---|---:|---|
| F1 | intersecting | 0 | B2 |
| F2 | parallel | 0.425 | A |
| F3 | parallel | 0.39225 | A |
| F4 | intersecting | approximately 0 | B1 |
| F5 | intersecting | approximately 0 | B1 |
| F6 | parallel, coincident | approximately 0 | A |

Five physical joint-axis pairs agree with the URDF-based calculation. The sixth
pair uses the solver's synthetic terminal z axis, not a seventh joint or the
arbitrary tool orientation. Both B1 and B2 occupy intersecting rows here, so a
one-to-one mapping from the three descriptive categories cannot reproduce the
plan. No A/B1/B2 labels are asserted by production code. Resolve this with Table 3
and the MATLAB logic before implementing reference-specific editing rules.

## Verification

Full suite: 208 tests passed. Mypy: 23 source files clean. Dependency check: no
broken requirements.

Tests cover analytic parallel/antiparallel/coincident/intersecting/skew examples,
60 independently computed NumPy least-squares line distances, rigid-transform
invariance, threshold boundaries, invalid inputs, control mapping, CLI JSON and
URDF-versus-DH physical pair agreement. UR5 cases are unchanged for parallel
thresholds 1e-6, 1e-5 and 1e-4; a synthetic near-parallel case changes as expected
and remains locked when classified approximately parallel.

Development evidence: `outputs/validation_reports/stage07_ur5.json`. This records
the provisional config and source digest, not a global-validation certificate.
