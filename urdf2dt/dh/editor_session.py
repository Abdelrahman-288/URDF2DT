"""Headless, atomic editor transitions; geometric validation is injected.

Geometric FrameEdit proposals use built-in compensated recomputation and local
checks. Raw changed DH rows require an injected validator. Local acceptance never
constitutes global FK certification.
"""

from dataclasses import dataclass, replace
import logging
from typing import Protocol

from urdf2dt.dh.types import DHModel, DHRow, EditRecord, EditorState, FrameState
from urdf2dt.config import GeometryConfig
from urdf2dt.dh.recompute import FrameEdit, recompute_model, replace_frame
from urdf2dt.kinematics import dh_frame_transforms


@dataclass(frozen=True, slots=True)
class EditProposal:
    """Uncommitted one-based row proposal with an optional compensated frame edit."""
    frame_index: int
    before: DHRow
    proposed: DHRow
    geometric_edit: FrameEdit | None = None


@dataclass(frozen=True, slots=True)
class EditDecision:
    """Local acceptance result and its audit explanation; not a global certificate."""
    valid: bool
    reason: str

    def __post_init__(self) -> None:
        if type(self.valid) is not bool or not isinstance(self.reason, str) or not self.reason.strip():
            raise ValueError("decision requires a boolean and a nonempty reason")


class EditValidator(Protocol):
    """Contract for optional raw-row validation against an immutable snapshot."""
    def __call__(self, state: EditorState, proposal: EditProposal) -> EditDecision:
        """Validate against an immutable snapshot; must not mutate the session."""
        ...


@dataclass(frozen=True, slots=True)
class SessionEvent:
    """Ordered audit event retaining the corresponding model and acceptance states."""
    sequence: int
    action: str
    frame_index: int | None
    reason: str
    working_model: DHModel | None = None
    frames: tuple[FrameState, ...] = ()
    pending: EditProposal | None = None


def _confirm_only(state: EditorState, proposal: EditProposal) -> EditDecision:
    return EditDecision(proposal.proposed == proposal.before,
                        "Unchanged frame confirmation." if proposal.proposed == proposal.before else
                        "Raw DH row changes require an injected validator; use FrameEdit for built-in geometry checks.")


class EditorSession:
    """Single-owner session with immutable public snapshots and one pending edit.

    Frame indices are one-based. Accepted earlier frames can be revisited, but
    every predecessor must be accepted. Invalidated rows remain visible while
    losing acceptance; unlock_next explicitly reopens the first such frame.
    """

    def __init__(self, automatic_model: DHModel, *, validator: EditValidator | None = None,
                 geometry: GeometryConfig | None = None):
        if not isinstance(automatic_model, DHModel):
            raise TypeError("automatic_model must be DHModel")
        self._state = EditorState(automatic_model, automatic_model,
                                  (FrameState.EDITABLE,) + (FrameState.LOCKED,) * (len(automatic_model.rows) - 1))
        self._pending: EditProposal | None = None
        self._validator = validator if validator is not None else _confirm_only
        self._events: tuple[SessionEvent, ...] = ()
        self._validating = False
        self._geometry = geometry if geometry is not None else GeometryConfig()
        self._geometric_frames: set[int] = set()
        self._undo: list[tuple[EditorState, set[int]]] = []
        self._redo: list[tuple[EditorState, set[int]]] = []

    @property
    def state(self) -> EditorState:
        """Return the immutable committed state snapshot."""
        return self._state

    @property
    def pending(self) -> EditProposal | None:
        """Return the sole uncommitted proposal, or None."""
        return self._pending

    @property
    def events(self) -> tuple[SessionEvent, ...]:
        """Return the immutable ordered audit trail."""
        return self._events

    def _event(self, action: str, index: int | None, reason: str) -> None:
        self._events += (SessionEvent(len(self._events) + 1, action, index, reason,
                                     self.state.working_model, self.state.frames, self.pending),)
        logging.getLogger(__name__).info("session %s frame=%s reason=%s", action, index, reason)

    @classmethod
    def from_snapshot(cls, state: EditorState, events: tuple[SessionEvent, ...],
                      geometric_frames: tuple[int, ...], geometry: GeometryConfig) -> "EditorSession":
        """Restore checked data; no callbacks or code are deserialized."""
        session = cls(state.automatic_model, geometry=geometry)
        if any(e.sequence != i for i, e in enumerate(events, 1)):
            raise ValueError("event sequence is not consecutive")
        if not events:
            raise ValueError("Saved session requires an audit history")
        for event in events:
            if type(event.sequence) is not int or event.action not in {
                    "unlock", "propose", "accept", "reject", "restore_frame", "restore_automatic", "undo", "redo"}:
                raise ValueError("Invalid audit event")
            if not isinstance(event.reason, str) or not event.reason.strip():
                raise ValueError("Audit reason must be nonempty")
            if event.frame_index is not None:
                session._index(event.frame_index)
            if event.working_model is None:
                raise ValueError("Audit event requires its full working model")
            EditorState(state.automatic_model, event.working_model, event.frames)
        if events and (events[-1].working_model != state.working_model or events[-1].frames != state.frames
                       or events[-1].pending is not None):
            raise ValueError("last event does not match saved session")
        for index in geometric_frames:
            session._index(index)
        session._state, session._events = state, events
        session._geometric_frames = set(geometric_frames)
        return session

    @property
    def geometric_frames(self) -> tuple[int, ...]:
        """Return sorted indices whose accepted proposals used geometric recomputation."""
        return tuple(sorted(self._geometric_frames))

    @property
    def geometry_config(self) -> GeometryConfig:
        """Return the immutable tolerances used for local geometric validation."""
        return self._geometry

    def _index(self, index: int) -> int:
        if type(index) is not int or not 1 <= index <= len(self.state.frames):
            raise ValueError("frame_index must be a one-based index within the model")
        return index - 1

    def _idle(self) -> None:
        self._not_validating()
        if self.pending is not None:
            raise ValueError("accept or reject the pending proposal first")

    def _not_validating(self) -> None:
        if self._validating:
            raise RuntimeError("session transitions are forbidden during validation")

    def _editable(self, index: int) -> int:
        i = self._index(index)
        if any(f != FrameState.ACCEPTED for f in self.state.frames[:i]):
            raise ValueError("all predecessor frames must be accepted")
        if self.state.frames[i] not in (FrameState.EDITABLE, FrameState.ACCEPTED):
            raise ValueError("frame must be unlocked before editing")
        return i

    def unlock_next(self) -> int | None:
        """Unlock the first unaccepted frame; return its one-based index or None."""
        self._idle()
        for i, status in enumerate(self.state.frames):
            if status != FrameState.ACCEPTED:
                frames = list(self.state.frames)
                frames[i] = FrameState.EDITABLE
                self._state = replace(self.state, frames=tuple(frames))
                self._event("unlock", i + 1, "First unaccepted frame unlocked.")
                return i + 1
        return None

    def propose_edit(self, frame_index: int, row: DHRow | FrameEdit) -> EditProposal:
        """Check a row or geometric edit without committing it; require accepted predecessors."""
        self._idle()
        i = self._editable(frame_index)
        before = self.state.working_model.rows[i]
        edit = row if isinstance(row, FrameEdit) else None
        if edit is not None:
            row = recompute_model(self.state.working_model, frame_index, edit, self._geometry).rows[i]
        # Reuse the domain identity/sign guard before creating any pending state.
        if not isinstance(row, DHRow):
            raise TypeError("row must be DHRow or FrameEdit")
        EditRecord(1, frame_index, before, row, False, "Proposal identity check.")
        self._pending = EditProposal(frame_index, before, row, edit)
        self._event("propose", frame_index, "Pending proposal; working model unchanged.")
        return self._pending

    def _proposal(self) -> EditProposal:
        self._not_validating()
        if self.pending is None:
            raise ValueError("no pending proposal")
        return self.pending

    def _record(self, proposal: EditProposal, accepted: bool, reason: str) -> tuple[EditRecord, ...]:
        return self.state.history + (EditRecord(len(self.state.history) + 1, proposal.frame_index,
                                                proposal.before, proposal.proposed, accepted, reason),)

    def accept(self) -> EditDecision:
        """Validate and atomically commit the pending proposal, invalidating affected successors."""
        proposal = self._proposal()
        snapshot = self.state
        self._validating = True
        try:
            candidate = (recompute_model(snapshot.working_model, proposal.frame_index,
                         proposal.geometric_edit, self._geometry) if proposal.geometric_edit is not None else None)
            decision = (EditDecision(True, "Geometric frame edit validated with adjacent compensation.")
                        if candidate is not None else self._validator(snapshot, proposal))
        finally:
            self._validating = False
        if not isinstance(decision, EditDecision):
            raise TypeError("validator must return EditDecision")
        if self.state is not snapshot or self.pending is not proposal:
            raise RuntimeError("validator changed the session during validation")
        if not decision.valid:
            self.reject(decision.reason)
            return decision
        i = proposal.frame_index - 1
        rows, frames = list(snapshot.working_model.rows), list(snapshot.frames)
        rows[i] = proposal.proposed
        if proposal.proposed != proposal.before:
            frames[i + 1:] = [FrameState.INVALIDATED] * (len(frames) - i - 1)
            if i + 1 < len(frames):
                logging.getLogger(__name__).info("Cascade invalidation: frames %d through %d", i + 2, len(frames))
        frames[i] = FrameState.ACCEPTED
        if i + 1 < len(frames) and frames[i + 1] == FrameState.LOCKED:
            frames[i + 1] = FrameState.EDITABLE
        new_state = replace(snapshot, working_model=candidate if candidate is not None else replace(snapshot.working_model, rows=tuple(rows)),
                            frames=tuple(frames), history=self._record(proposal, True, decision.reason))
        self._undo.append((snapshot,set(self._geometric_frames)));self._redo.clear()
        self._state, self._pending = new_state, None
        if candidate is not None:
            self._geometric_frames.add(proposal.frame_index)
        self._event("accept", proposal.frame_index, decision.reason)
        return decision

    def reject(self, reason: str = "Proposal cancelled by user.") -> None:
        """Discard the pending proposal while retaining its reason in the audit history."""
        proposal = self._proposal()
        new_state = replace(self.state, history=self._record(proposal, False, reason))
        self._state, self._pending = new_state, None
        self._event("reject", proposal.frame_index, reason)

    def restore_frame(self, frame_index: int) -> None:
        """Restore one baseline frame with compensation and withdraw downstream acceptance."""
        self._idle()
        i = self._editable(frame_index)
        rows, frames = list(self.state.working_model.rows), list(self.state.frames)
        original = self.state.automatic_model.rows[i]
        restored_model = None
        if self._geometric_frames:
            baseline_pose = dh_frame_transforms(self.state.automatic_model, (0.,) * len(rows))[frame_index]
            restored_model = replace_frame(self.state.working_model, frame_index, baseline_pose, self._geometry,allow_reversal=True)
        changed = rows[i] != original
        rows[i] = original
        frames[i] = FrameState.EDITABLE
        # Even unchanged restoration withdraws this frame's acceptance.
        for j in range(i + 1, len(frames)):
            if changed or frames[j] != FrameState.LOCKED:
                frames[j] = FrameState.INVALIDATED
        self._undo.append((self.state,set(self._geometric_frames)));self._redo.clear()
        self._state = replace(self.state, working_model=restored_model if restored_model is not None else replace(self.state.working_model, rows=tuple(rows)),
                              frames=tuple(frames))
        self._geometric_frames.discard(frame_index)
        self._event("restore_frame", frame_index, "Automatic row restored; acceptance withdrawn.")

    def restore_automatic(self) -> None:
        """Discard pending edits and restore the exact baseline; retain the audit trail."""
        self._not_validating()
        # Reset can always recover a session, including one with a pending edit.
        if self.pending is not None:
            self.reject("Pending proposal discarded by whole-session restore.")
        self._undo.append((self.state,set(self._geometric_frames)));self._redo.clear()
        self._state = replace(self.state, working_model=self.state.automatic_model,
                              frames=(FrameState.EDITABLE,) + (FrameState.LOCKED,) * (len(self.state.frames) - 1))
        self._geometric_frames.clear()
        self._event("restore_automatic", None, "Exact automatic model and initial frame states restored; audit retained.")

    def undo(self) -> None:
        self._idle()
        if self._undo:
            self._redo.append((self.state,set(self._geometric_frames)))
            self._state,self._geometric_frames=self._undo.pop()
            self._event("undo",None,"Previous committed frame state restored; validation must be rerun.")

    def redo(self) -> None:
        self._idle()
        if self._redo:
            self._undo.append((self.state,set(self._geometric_frames)))
            self._state,self._geometric_frames=self._redo.pop()
            self._event("redo",None,"Next committed frame state restored; validation must be rerun.")
