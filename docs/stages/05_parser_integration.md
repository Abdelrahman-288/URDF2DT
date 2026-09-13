# Stage 5: parser and automatic DH integration

## Outcome

The Stage 4 validated serial URDF now produces a real, immutable Standard-DH
model through `SerialURDFParser` and `StandardDHSolver`. No nominal UR5 table is
imported into production, used as a fallback, or fitted to FK samples. The older
test fixture remains an independent reference only.

`generate_automatic_model(source, config=None, *, parser=None, solver=None)` is the
integration entry point. It validates before calling either adapter and returns
`AutomaticDHResult(source, chain, automatic_model, config, warnings)`. Adapters must
preserve robot identity, chain endpoints, joint order/types and source checksum.
Parser output retains all fixed joints; each DH row corresponds to one movable
joint. The source bytes' SHA-256 now accompanies both KinematicChain and DHModel.

## Coordinate conventions

The parser consumes `ValidatedURDF`, never reopens its source path, and converts
origin XYZ/RPY into `Rz(yaw) @ Ry(pitch) @ Rx(roll)`. Joint axes remain expressed
in the joint-local frame. Motion follows the origin transform. Noncommuting angles,
normalized oblique axes, limits and fixed transforms are preserved. Changing
geometry thresholds does not modify URDF coordinates.

The solver accumulates zero-pose origins, including fixed joints, to obtain
movable-axis lines in base coordinates. It selects frames as follows:

1. F0's z axis is the first movable axis in the same direction. Its origin is the
   point on that line closest to the selected URDF base origin. The first common
   normal sets x0; a deterministic perpendicular is used for a single axis.
2. For each nonparallel pair, the next origin Q is the closest point on the next
   axis. If `n = z × z_next` and `w = p_next - p`, then
   `Q = p_next + z_next * ((w × z) · n) / ||n||²`.
   The x direction is parallel to n, signed to align with the preceding x where
   possible. Signed link lengths preserve this gauge choice.
3. For parallel/antiparallel lines, Q projects the preceding origin onto the next
   line. Coincident lines retain the previous x direction.
4. The terminal frame projects the physical tip onto the final joint line and
   retains x/z directions. Residual geometry is kept in `tool_transform`.

Each relative frame transform is factored as
`Rz(theta_offset) @ Tz(d) @ Tx(a) @ Rx(alpha)` and numerically checked against the
constructed transform. Joint i acts about/along z_(i-1), so theta varies for
revolute/continuous joints and d varies for prismatic joints. Keeping the original
axis direction makes generated `joint_sign` +1; the evaluator also supports
imported -1 signs. Base/tool transforms are essential model data, not alignment
estimated from samples. These frames are rigidly attached to the corresponding
links: matching joint axes and fixed transforms preserves the joint motion factors.

## Numerical policy

Near-parallel pairs above roundoff but at/below the configured parallel threshold
raise `DHSolverError(ill_conditioned_axes)` with the joint names. They are not
silently flattened. Exact parallel/coincident handling permits 64 machine epsilons
of unit-vector roundoff. That arithmetic budget and the core rigid-transform
tolerance are separate from research classification/FK thresholds. No distance is
snapped using the provisional intersection/common-normal thresholds. Overflowing
or invalid factorizations produce structured errors.

This implements construction, not Stage 7's reference A/B1/B2 classifier or legal
edit mapping, and not Stage 9's editing/recompute API.

## UR5 reference comparison

The prepared public `robots/ur5/ur5_serial.urdf` yields:

| Row | a (m) | alpha (rad) | d (m) |
|---|---:|---:|---:|
| 1 | 0 | pi/2 | 0.089159 |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.39225 | 0 | 0 |
| 4 | 0 | pi/2 | 0.10915 |
| 5 | approximately 0 | -pi/2 | 0.09465 |
| 6 | 0 | 0 | 0.0823 |

These agree with the [manufacturer's classic UR5 nominal table](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics).
Largest absolute column differences are approximately `a: 5.6e-17 m`,
`alpha: 0 rad`, `d: 1.4e-17 m`, `theta_offset: 4.9e-12 rad`.
Tiny offsets/tool corrections preserve rounded input XML angles. F0 has a
180-degree base z rotation; dropping it would invalidate the FK alignment even
though the DH table matches.

The exact MATLAB files and `universalUR5.urdf` are still missing. This comparison
uses the manufacturer and documented public fixture, not verified MATLAB frame
conventions. Advisor thresholds remain provisional.

## Numeric FK and development evidence

`urdf_link_transforms`, `urdf_fk`, `dh_frame_transforms`, and `dh_fk` accept finite
q coordinates in movable-joint order. FK does not enforce limits; zero pose may
be useful even outside a joint's range. These Python evaluators have no UI dependency.
Stage 11 still needs the formal global validator, CasADi integration, failure
attribution and edited-model validation.

`scripts/verify_stage05.py` independently rereads XML using defusedxml and evaluates
rotations with SciPy, without production parser/FK matrix helpers. It checks zero
pose plus 49 seeded, joint-limit-aware UR5 samples against DH FK and records q,
metrics, configuration, package versions, source checksum and code/working-tree
state. Orientation error is the norm of the relative rotation vector: the SO(3)
geodesic angle. This avoids trace/arccos cancellation near machine precision.

Measured maximum errors: approximately **6.9e-16 m** and **4.7e-16 rad**.
The saved report is `outputs/validation_reports/stage05_ur5.json`. Its
`implementation_commit` identifies the code used; a subsequent evidence-only
commit stores the report. It is a development check, not schema 1.0 or an
editor/global-validation certificate.

## Verification and usage

- Full suite: **183 passed**, no skips or xfails.
- Mypy: **20 production source files**, no issues.
- Tests include independent XML/SciPy FK for UR5 and mixed joints, 20 seeded
  synthetic chains of 1–7 movable joints with interleaved fixed transforms,
  parallel/antiparallel/coincident/intersecting/skew pairs, single joints,
  signs/offsets, ill-conditioned rejection, immutability, adapters and isolated CLI.
- Runtime: Windows, Python 3.12.14. Other runtime platforms remain untested.

```powershell
python -m urdf2dt robots/ur5/ur5_serial.urdf
python -m urdf2dt robots/ur5/ur5_serial.urdf --json
python scripts/verify_stage05.py --output outputs/tmp/stage05_recheck.json
python -m pytest -q
python -m mypy
```

Branch: `feature/stage-05-parser-integration`, based on Stage 4 `ee88b38`.
Next: Stage 6 static scene renderer, reusing numeric link/frame transforms.
