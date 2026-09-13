"""State mechanics use explicit test validators, never claimed geometry proofs."""

from dataclasses import FrozenInstanceError, replace

import pytest

from urdf2dt.dh.editor_session import EditDecision, EditorSession
from urdf2dt.dh.types import DHModel, DHRow, FrameState as F


@pytest.fixture
def model():
    return DHModel("synthetic", tuple(DHRow(.1, 0., 0., 0., f"j{i}") for i in range(6)), "state tests")


def approve(state, proposal):
    return EditDecision(True, "Test-only transition approval; not geometric validation.")


def confirm(session, count):
    for i in range(1, count + 1):
        session.unlock_next()
        session.propose_edit(i, session.state.working_model.rows[i-1])
        assert session.accept().valid


def test_initial_and_sequential_completion(model):
    session = EditorSession(model)
    assert session.state.frames == (F.EDITABLE,) + (F.LOCKED,) * 5
    assert session.unlock_next() == 1
    with pytest.raises(ValueError):
        session.propose_edit(2, model.rows[1])
    confirm(session, 6)
    assert session.state.frames == (F.ACCEPTED,) * 6
    assert session.unlock_next() is None
    assert session.state.automatic_model == model


@pytest.mark.parametrize("accepted,edited", [(3, 2), (6, 1), (6, 2)])
def test_cascade(model, accepted, edited):
    session = EditorSession(model, validator=approve)
    confirm(session, accepted)
    before = session.state
    session.propose_edit(edited, replace(model.rows[edited-1], d=.2))
    assert session.state is before
    session.accept()
    assert session.state.frames[:edited] == (F.ACCEPTED,) * edited
    assert session.state.frames[edited:] == (F.INVALIDATED,) * (6-edited)
    assert session.state.automatic_model == model
    assert session.state.working_model.rows[:edited-1] == model.rows[:edited-1]
    assert session.unlock_next() == edited + 1
    with pytest.raises(ValueError):
        session.propose_edit(edited + 2, model.rows[edited + 1])


def test_rejection_preserves_state_and_records_reason(model):
    session = EditorSession(model)
    confirm(session, 3)
    before = session.state
    session.propose_edit(2, replace(model.rows[1], a=.9))
    decision = session.accept()
    assert not decision.valid
    assert session.state.working_model == before.working_model
    assert session.state.frames == before.frames
    assert session.pending is None
    assert not session.state.history[-1].accepted
    assert "Stage 9" in session.state.history[-1].reason


def test_cancel_and_pending_guards(model):
    session = EditorSession(model)
    session.propose_edit(1, model.rows[0])
    for action in (session.unlock_next, lambda: session.restore_frame(1),
                   lambda: session.propose_edit(1, model.rows[0])):
        with pytest.raises(ValueError):
            action()
    with pytest.raises(ValueError):
        session.reject("")
    assert session.pending is not None
    session.reject()
    assert session.state.working_model == model
    with pytest.raises(ValueError):
        session.accept()


def test_restore_frame_and_whole_session(model):
    session = EditorSession(model, validator=approve)
    session.propose_edit(1, replace(model.rows[0], d=.5))
    session.accept()
    confirm(session, 6)
    session.restore_frame(1)
    assert session.state.working_model.rows[0] == model.rows[0]
    assert session.state.frames == (F.EDITABLE,) + (F.INVALIDATED,) * 5
    session.propose_edit(1, replace(model.rows[0], a=.7))
    session.restore_automatic()
    assert session.pending is None
    assert session.state.working_model == model
    assert session.state.frames == (F.EDITABLE,) + (F.LOCKED,) * 5
    assert session.state.history
    assert session.events[-1].action == "restore_automatic"
    assert [e.sequence for e in session.events] == list(range(1, len(session.events) + 1))


def test_noop_reaccept_does_not_invalidate(model):
    session = EditorSession(model)
    confirm(session, 6)
    session.propose_edit(2, model.rows[1])
    session.accept()
    assert session.state.frames == (F.ACCEPTED,) * 6


@pytest.mark.parametrize("index", [0, -1, 7, True, 1.5])
def test_bad_indices(model, index):
    session = EditorSession(model)
    with pytest.raises(ValueError):
        session.propose_edit(index, model.rows[0])
    assert session.pending is None


def test_identity_and_snapshot(model):
    session = EditorSession(model)
    with pytest.raises(ValueError):
        session.propose_edit(1, replace(model.rows[0], joint_name="other"))
    snapshot = session.state
    with pytest.raises(FrozenInstanceError):
        snapshot.frames = ()
    confirm(session, 1)
    assert snapshot.frames[0] == F.EDITABLE


@pytest.mark.parametrize("failure", ["raise", "wrong_type", "reenter"])
def test_validator_failure_is_atomic(model, failure):
    def validator(state, proposal):
        if failure == "raise":
            raise RuntimeError("failed")
        if failure == "reenter":
            session.restore_automatic()
        return True
    session = EditorSession(model, validator=validator)
    session.propose_edit(1, model.rows[0])
    before, pending = session.state, session.pending
    with pytest.raises((RuntimeError, TypeError)):
        session.accept()
    assert session.state is before and session.pending is pending
    session.reject()


def test_single_frame(model):
    session = EditorSession(replace(model, rows=model.rows[:1]))
    confirm(session, 1)
    assert session.unlock_next() is None
    session.restore_automatic()
    assert session.state.frames == (F.EDITABLE,)
