# Symbolic forward kinematics

Open a robot, select its serial chain, and choose **Symbolic FK** from the Tool
selector beside Fit view. Select **Committed DH model** or **Automatic DH model**,
then **Generate symbolic FK**. The physical tip's 4 by 4 transform is relative to
the selected chain base. It includes fixed base/tool alignments, DH offsets, and
joint signs. Revolute/continuous symbols use radians; prismatic symbols use metres.
The symbol key maps q1 through qN to original joint names.

Use **Open full matrix view** for a larger resizable snapshot, **Copy symbolic
text**, **Copy LaTeX**, or **Export symbolic FK** (text or LaTeX). M_ij names denote
the explicitly listed matrix entries; t0, t1, ... are common intermediate
expressions, evaluated in order. Keep these definitions with the exported matrix.

Geometry stays numeric while joint coordinates are symbolic. Stored floating-point
constants are retained: tiny terms are not discarded and nearly pi/2 angles are
not changed into exact pi/2 values. Expressions can therefore be longer than a
manually idealized model. Common-subexpression elimination reduces repetition
without global trigonometric simplification or geometry rounding.

Generation uses a separate cancellable process with a 60-second limit and at most
12 movable joints. Four results are cached in memory. Slider motion does not
regenerate expressions. Robot, chain, model-choice and committed-model changes
invalidate the displayed answer and cancel obsolete workers. Pending DH previews
are excluded. Cancelling generation does not change robot/editor state.

This version covers chain endpoints only. The numerical calculator's physical-link
and named-frame selectors do not change this symbolic result. The symbolic
Jacobian is available in its own tab; see [its guide](symbolic_jacobian.md).
Symbolic geometry parameters remain outside scope. Projects
retain source/model state; symbolic exports are separate artifacts, regenerated
after reopening.

Developer API: `urdf2dt.symbolic.symbolic_fk(model)` returns the immutable symbolic
matrix, joint symbols, `evaluate(q)` numerical substitution, and `documents()`
text/LaTeX output. SymPy is a lazily loaded core dependency. Run
`python -m pytest tests/test_symbolic.py -q` for analytic/numeric checks.
The desktop/EXE flag `--verify-symbolic NEW_FOLDER` checks the native worker,
cancellation, caching, robot switching, preview exclusion and exports.
