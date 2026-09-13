# Stage 9: DH recomputation and local validation

Implemented Standard-DH frame edits, adjacent compensation, closest P/Q helpers,
structured local errors and EditorSession integration. Reference-specific R1–R3
and A/B1/B2 reconciliation remains pending the report and MATLAB inputs. This
implementation uses descriptive Standard-DH constraints and is not that sign-off.

## Usage

```python
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.recompute import FrameEdit

run = generate_automatic_model("robots/ur5/ur5_serial.urdf")
session = EditorSession(run.automatic_model, geometry=run.config.geometry)
session.propose_edit(1, run.automatic_model.rows[0])
session.accept()
session.propose_edit(2, FrameEdit(axial_translation=0.05))
session.accept()
session.restore_frame(2)
session.restore_automatic()
```

FrameEdit values are increments relative to the current frame: metres along z
and radians about z. The classifier determines allowed controls; forbidden or
tolerance-ambiguous edits raise FrameEditError with structured `issues` before
creating pending state. No values are clamped. There are no arbitrary XYZ/RPY
controls or invented finite slider bounds. Raw changed DHRow proposals continue
to require an explicitly injected validator; FrameEdit uses the built-in geometry
path and rechecks its candidate at acceptance.

## Compensation and correctness

The candidate is F_i multiplied by an axial rotation/translation. Recompute the
incoming DH row and the next row to preserve the next world frame. For Fn, update
the fixed tool transform instead. These axial transforms commute with revolute
and prismatic motion on the same axis, retaining the kinematic chain across q.
Joint names, kinds, signs, source metadata and base alignment are preserved.

`recompute_dh_row` extracts Standard-DH parameters from the relative transform and
checks reconstruction at 1e-9 absolute precision. `validate_frame` checks rigid
right-handed orientation, preservation of the directed joint line, common-normal
perpendicularity and DH representability. It returns structured descriptive rules:
`rigid_frame`, `joint_axis`, `common_normal`, `dh_factorization`. These names are
not asserted to be the missing report's R1–R3 numbering. Axis-line tolerance is
the smaller of configured common-normal tolerance and 1e-9 m. Classification
thresholds retain Stage 7 semantics. Common-normal P/Q uses a deterministic
anchor choice for exact parallel axes and refuses ill-conditioned cases.

Proposals do not alter working state. Acceptance commits the compensated model
atomically and invalidates downstream acceptance. Rejection discards it. Once
geometric edits exist, single-frame restore restores the baseline world frame
and compensates neighboring rows, rather than blindly restoring a row whose
neighbor may have moved. Whole restore still returns the exact baseline model.
The row-oriented EditRecord records the selected row; it is not yet a complete
serialized replay of compensated models. Session events record actions. A full
session export remains future work.

## Verification

240 tests pass; mypy is clean across 25 source files. Tests sweep translation from
-1 to +1 m on UR5 F2/F3/F6 and terminal rotation from -pi to +pi, comparing FK
at varied joint configurations. These are tested intervals, not imposed control
bounds. Other tests cover antiparallel/prismatic geometry, common-normal points,
illegal controls, nonfinite inputs, reflection rejection, preserved other frames,
atomic compensation and restore. This is development verification, not the
Stage 11 global validator or MATLAB reference comparison.
