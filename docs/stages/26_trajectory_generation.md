# Stage 26 — Trajectory Generation

Stage 26 provides source-bound controller references for joint and Cartesian
motion. It separates geometric path construction from time parameterization and
checks every generated polynomial between output samples.

## Reproduce

```powershell
python -m urdf2dt.trajectory scara --output outputs/validation_reports/stage26/scara
python -m urdf2dt.trajectory ur5 --output outputs/validation_reports/stage26/ur5
```

Each output contains cubic and quintic joint references, a Cartesian reference,
and a checksummed report. Use a new output directory for every run.

`joint_trajectory()` supports bounded cubic/quintic point-to-point and waypoint
motion. Quintic segments have zero velocity and acceleration at stops and are C2
across waypoints. Continuous joints preserve supplied unwrapped coordinates.
Position, velocity and acceleration maxima are checked from polynomial extrema,
not only sampled output. Effort limits are evaluated separately by
`dynamic_feasibility(trajectory, model)` using Stage 24 inverse dynamics.

`cartesian_line()` provides a straight base-frame translation or a full pose path
with shortest relative SO(3) rotation. Bounded seeded IK follows the previous
solution, checks residuals, joint limits, Jacobian singular values, and branch
jumps, then joins knots with a C2 cubic geometric spline and quintic timing.
Local IK failure is reported as an actionable diagnostic; collision and global
continuous-space certificates are outside this stage.

`export_trajectory()` stores exact source bytes, hash, ordered joint names, units,
coefficients, limits, metadata and analytic samples in a versioned checksum
envelope. Loading recomputes samples and rejects tampering. These are desired
reference states only; Stage 27 adds controllers and Stage 28 adds simulation.

Tests cover derivatives, endpoints, C2 continuity, limits, unwrapped joints,
finite-difference Jacobians, SCARA and UR5 Cartesian paths, unreachable/singular
targets, effort checks, Stage 25 parameter binding, archive integrity, and both
end-to-end demonstrations. Explicit example limits are not manufacturer limits.

The Cartesian spline uses SciPy's [CubicSpline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html),
which requires finite strictly increasing breakpoints and produces a twice
continuously differentiable piecewise cubic interpolant.
