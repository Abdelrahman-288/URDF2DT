# Stage 14: Research ablation results

Source: `universalUR5.urdf`; chain `world → tool0`.

Selected source SHA-256: `44fa0e76ad79c258370d00f39dbbb20e8a7d558b4b73fea71c2822c8f58fae22`.

Full settings, source/code fingerprints, sampled poses and errors are in `results.json`.

## Threshold sensitivity

The same automatic model is reclassified at 0.1, 0.3, 1, 3 and 10 times the default parallel threshold. Other thresholds stay fixed.

| Parallel threshold | Changed frames |
|---:|---|
| 1e-06 | None |
| 3e-06 | None |
| 1e-05 | None |
| 3e-05 | None |
| 0.0001 | None |

## Legal edit sweeps

Each point starts from the automatic model and traverses the real acceptance gates. Translation is tested over [-1,1] m; rotation over one period [-pi,pi]. These are finite samples, not full-domain coverage.

| Frame / freedom | Evaluated / total | Local + FK passed | Max position [m] | Max orientation [rad] |
|---|---:|---:|---:|---:|
| F2 axial_translation | 21 / 21 | 21 | 4.56928e-16 | 2.98023e-08 |
| F3 axial_translation | 21 / 21 | 21 | 4.44211e-16 | 2.98023e-08 |
| F6 axial_rotation | 21 / 21 | 21 | 5.66487e-16 | 2.98023e-08 |
| F6 axial_translation | 21 / 21 | 21 | 5.6882e-16 | 2.98023e-08 |

## Sample density

Zero pose is included. Seed 42 gives nested 10/50/200 sample sets. The edited model combines the explicitly recorded legal edits. The negative control deliberately bypasses acceptance and is never a validated export.

| Model | Samples | FK pass | Max position [m] | Max orientation [rad] |
|---|---:|---|---:|---:|
| automatic | 10 | True | 2.43554e-16 | 0 |
| edited | 10 | True | 2.498e-16 | 2.10734e-08 |
| negative_control | 10 | False | 0.001 | 0 |
| automatic | 50 | True | 4.44103e-16 | 2.98023e-08 |
| edited | 50 | True | 2.498e-16 | 2.98023e-08 |
| negative_control | 50 | False | 0.001 | 2.98023e-08 |
| automatic | 200 | True | 4.44103e-16 | 2.98023e-08 |
| edited | 200 | True | 3.64275e-16 | 2.98023e-08 |
| negative_control | 200 | False | 0.001 | 2.98023e-08 |

## Synthetic boundaries

| Family | Factor | Case | Coincident | Review required | Editable freedoms |
|---|---:|---|---|---|---|
| parallel | 0.999 | parallel | False | True | None |
| intersection | 0.999 | intersecting | False | True | None |
| coincidence | 0.999 | parallel | True | True | None |
| parallel | 1.0 | parallel | False | True | None |
| intersection | 1.0 | intersecting | False | True | None |
| coincidence | 1.0 | parallel | True | True | None |
| parallel | 1.001 | intersecting | False | False | None |
| intersection | 1.001 | skew | False | False | None |
| coincidence | 1.001 | parallel | False | False | axial_translation |

## Interpretation and limits

- Boundary changes are intentional: normalized cross magnitude and shortest distance determine the recorded cases. Approximate parallel/coincident cases remain locked for review; labels alone do not authorize edits.
- Passing finite sweeps supports the tested gauge edits; it does not prove every value or simultaneous edit combination legal. Axial translation has no finite complete legal interval.
- Increasing sample count tests more poses, not a stricter geometric tolerance. The injected link-length fault tests validator sensitivity separately from valid candidates.
- Orientation uses clamped trace/acos and can show a numerical floor near 1e-8 rad; this is not a physical accuracy measurement.
- Thresholds and report-specific rule labels remain provisional. No MATLAB execution/comparison or advisor acceptance is claimed.

All expected experiment outcomes met: **True**.
