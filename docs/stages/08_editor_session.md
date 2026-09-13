# Stage 8: EditorSession state machine

Implemented in `urdf2dt/dh/editor_session.py`, independently of any UI. This stage
manages state; it does not implement geometric recomputation or global validation.

## Usage

```python
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.editor_session import EditorSession

run = generate_automatic_model("robots/ur5/ur5_serial.urdf")
session = EditorSession(run.automatic_model)
session.propose_edit(1, session.state.working_model.rows[0])
decision = session.accept()  # unchanged frame confirmation
assert decision.valid
assert session.unlock_next() == 2
session.restore_automatic()
assert session.state.working_model == run.automatic_model
```

`state`, `pending` and `events` expose immutable snapshots. The automatic model
is never overwritten. All frame indices are one-based. Sessions have a single
owner; concurrent use is not supported.

## Transition rules

- Initially F1 is editable and later frames are locked.
- `unlock_next()` opens the first unaccepted frame, is idempotent for an already
  editable frame, and returns None when all frames are accepted.
- `propose_edit(index, row)` checks joint identity/type/sign and predecessor
  acceptance. It can revisit an accepted frame, but cannot skip predecessors.
  It stores a pending proposal without changing working values or statuses.
- `accept()` validates the proposal against the current immutable snapshot.
  Success commits the row and marks the frame accepted. An unchanged confirmation
  opens the next locked frame. A changed row invalidates every downstream frame;
  call `unlock_next()` to reopen the first invalidated frame explicitly.
- Reaccepting an unchanged row preserves downstream acceptance.
- `reject(reason)` discards the proposal and adds a rejection record, preserving
  working values and statuses. Validator rejection has the same effect. Validator
  exceptions or invalid return types leave both state and pending proposal intact.
- Only one proposal may be pending. Accept/reject it before another edit, unlock
  or single-frame restore. Validator callbacks cannot reenter session transitions.
- `restore_frame(index)` uses the exact automatic row for an editable or accepted
  frame, withdraws its acceptance and invalidates affected downstream frames.
  Unvisited locked frames remain locked if the row was already unchanged.
- `restore_automatic()` works even with a pending proposal. It restores the exact
  automatic model and initial frame statuses, discarding the pending proposal.

Invalidated rows remain visible but are no longer accepted. Immutable EditRecord
history records accepted/rejected proposals. SessionEvent history also records
unlocks, proposals and restores. Whole-session restore retains this audit; it
resets model and frame state, not history. Construct a new session for a fresh audit.

## Stage 9 boundary

An injected `EditValidator(state, proposal) -> EditDecision` decides whether a
candidate may commit. The built-in default accepts unchanged confirmations only
and rejects changed rows with a clear Stage 9 message. Tests inject explicitly
synthetic validators solely to exercise transition mechanics.

No arbitrary changed row is accepted by default, and an accepted frame does not
mean global FK certification. Stage 9 must add case-specific recomputation and
local checks, including any required compensation in adjacent transforms. The
current proposal carries a single row; its API may expand for those atomic updates.
Stage 7's missing report/MATLAB label reconciliation is still pending.

## Verification

- Full suite: 226 tests passed; mypy clean across 24 source files.
- F1–F3 accepted, F2 changed: later frames invalidated.
- All six accepted, F1 changed: F2–F6 invalidated.
- Sequential gating, baseline immutability, snapshot stability, exact restore,
  no-op acceptance, rejection, identity protection and invalid indices tested.
- Exceptions, bad validator returns and reentrant callbacks preserve state.
- Core import test explicitly blocks optional UI packages while importing the session.
