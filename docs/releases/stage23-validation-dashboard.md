# Sampled FK validation dashboard

The desktop results area now provides a Validation dashboard. Accept all DH frames,
resolve any pending preview, then select Run global FK validation. Validate FK in
the DH editor opens the same dashboard. Drag the divider above the results upward
for more space.

- Summary: maximum position error in metres and orientation error in radians,
  configured tolerances, separate metric PASS/FAIL, worst pose indices, total tested
  poses, failed pose count, seed, and tolerance reference status.
- Tested poses: every sampled coordinate vector and both errors with per-pose
  PASS/FAIL. Hover for full precision; pose indices start at zero.
- What was tested / limits: active physical chain endpoints, source joint order,
  units, error definitions, sampling policy, zero-pose limit status, threshold
  provenance, diagnostic interpretation, model/source hashes and explicit limits.
- Export evidence JSON: the unchanged full-precision validation report, including
  every sampled pose, prefix diagnostics and configuration. No display rounding is
  written into the report.

The automatic DH baseline must pass before the accepted committed model is tested.
Both position and orientation errors must be less than or equal to their respective
tolerances for every sampled pose. Small residuals use scientific notation.
The zero pose is deliberately included even if outside the robot's joint limits;
remaining samples use the configured seed and bounded limits, with continuous joints
sampled in [-pi, pi]. Moving the live pose sliders does not alter the sample set.

Model, preview, acceptance or configuration changes mark the displayed report STALE
and disable export until validation runs again. A robot/chain replacement clears
the prior evidence. An incomplete session or baseline failure shows NOT COMPLETED,
not PASS. Historical metric rows under a STALE banner are not current evidence.

A sampled PASS is not a continuous-space proof. It does not establish source URDF
correctness, symbolic FK/Jacobian correctness, body-mesh alignment, collision freedom,
dynamics, physical calibration, hardware accuracy or safe robot operation. Thresholds
retain their configured provisional/confirmed status. Prefix diagnostics are hints,
not causal attribution. The original validation calculations are unchanged.
