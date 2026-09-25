# Symbolic geometric Jacobian

Open a robot and select a serial chain. Choose **Symbolic Jacobian** (use the tab
bar arrows if necessary), choose the committed or automatic DH model, and press
**Generate symbolic Jacobian**. The result is a 6 by n matrix, where n is the
number of movable joints. Fixed joints are absorbed into the model transforms.

The target point is the physical selected tip link's origin. Both linear velocity
and angular velocity are relative to, and expressed in, the selected base link.
Rows are `[vx, vy, vz, wx, wy, wz]`. Columns follow the source joint-coordinate
order printed above the symbolic matrix. Thus `[v; omega] = J(q) qdot`, with
linear velocity in m/s and angular velocity in rad/s when joint rates are rad/s
(revolute/continuous) or m/s (prismatic). Angular velocity is not an RPY rate.

For each Standard-DH row, let z and o be the preceding frame's z axis and origin
in base coordinates, p the physical tip origin, and s the stored joint sign:

```text
Revolute/continuous: J_i = [s z × (p - o); s z]
Prismatic:          J_i = [s z;           0  ]
```

The construction includes the fixed base and tool transforms, offsets and source
joint signs. Compensated DH-axis flips therefore preserve physical velocities.
No matrix inversion is required at singular configurations; this feature does
not diagnose singularities or solve inverse velocity kinematics.

Generation runs in a cancellable separate process, with a 60-second limit and
at most 12 movable joints. Each symbolic tab caches four results. Pose changes
reuse the expressions; robot, chain, model choice and committed edits invalidate
stale results and cancel obsolete workers. Pending DH previews are excluded.

The lower pane updates J at the current slider pose by evaluating the same
axis/cross-product formula numerically. This avoids expensive symbolic substitution
on the GUI thread. Automated checks compare it to direct symbolic substitution
and central finite differences of translation and rotation (`dR/dq R^T`) from FK.
Displayed values use three decimal places. Numerical JSON exports retain full
precision and record the pose, joint order, units, model and frame convention.

Use the full matrix dialog, copy text/LaTeX, or symbolic export to inspect and
share expressions. M entries and t intermediate definitions belong with the
matrix; include all definitions. Joint coordinates are symbolic; geometry remains
at its stored numeric precision without rounding or snapping constants to pi.

Scope: selected chain endpoints only, with no arbitrary target point or expression
frame selector. The numerical FK tab's named-frame/link selectors do not change
this result. Symbolic results are separate exports, not persisted project data.
After reopening a project, generate them again. This remains within Stages 0–23.

APIs: `symbolic_jacobian(model)` in `urdf2dt.symbolic` exposes `matrix`, `symbols`,
`evaluate(q)` and `documents()`. `geometric_jacobian(model, q)` in
`urdf2dt.jacobian` evaluates the same geometric formula numerically.

Checks: `python -m pytest tests/test_jacobian.py` and
`python -m urdf2dt.ui.desktop --verify-jacobian NEW_FOLDER`. The latter is also
available on the executable and verifies the worker, UI responsiveness, live pose,
exports, cache, cancellation, preview exclusion and robot/model replacement.
