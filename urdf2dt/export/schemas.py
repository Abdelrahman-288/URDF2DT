"""Version 1.0 archive contract plus strict domain decoders."""
from dataclasses import fields
from typing import Any

from urdf2dt.dh.types import DHModel, DHRow, EditRecord, EditorState, FrameState
from urdf2dt.dh.editor_session import EditProposal, SessionEvent
from urdf2dt.dh.recompute import FrameEdit

SCHEMA_VERSION = "1.0"
SESSION_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "URDF2DT validated session archive 1.0",
    "type": "object", "additionalProperties": False,
    "required": ["schema_version", "dh_convention", "source", "configuration", "state", "events",
                 "geometric_frames", "validation", "reproducibility"],
    "properties": {
        "schema_version": {"const": SCHEMA_VERSION}, "dh_convention": {"const": "standard"},
        "source": {"type": "object", "additionalProperties": False,
                   "required": ["name", "source_path", "content_base64"],
                   "properties": {"name": {"type": "string"}, "source_path": {"type": ["string", "null"]},
                                  "content_base64": {"type": "string"}}},
        "configuration": {"type": "object"}, "state": {"type": "object"},
        "events": {"type": "array", "items": {"type": "object"}},
        "geometric_frames": {"type": "array", "uniqueItems": True, "items": {"type": "integer", "minimum": 1}},
        "validation": {"type": "object"}, "reproducibility": {"type": "object"},
    },
}


def exact(data: dict[str, Any], cls: Any) -> dict[str, Any]:
    """Require exactly the dataclass fields; reject missing or unknown archive fields."""
    if not isinstance(data, dict) or set(data) != {f.name for f in fields(cls)}:
        raise ValueError(f"Unexpected or missing fields in {cls.__name__}")
    return dict(data)


def decode_model(data: dict[str, Any]) -> DHModel:
    """Reconstruct a checked immutable DH model from its JSON field mapping."""
    values = exact(data, DHModel)
    values["rows"] = tuple(DHRow(**exact(r, DHRow)) for r in values["rows"])
    return DHModel(**values)


def decode_state(data: dict[str, Any]) -> EditorState:
    """Decode both models and edit records into an immutable editor snapshot."""
    values = exact(data, EditorState)
    values["automatic_model"] = decode_model(values["automatic_model"])
    values["working_model"] = decode_model(values["working_model"])
    history = []
    for record in values["history"]:
        record = exact(record, EditRecord)
        record["before"] = DHRow(**exact(record["before"], DHRow))
        record["proposed"] = DHRow(**exact(record["proposed"], DHRow))
        history.append(EditRecord(**record))
    values["history"] = tuple(history)
    return EditorState(**values)


def decode_event(data: dict[str, Any]) -> SessionEvent:
    """Reconstruct one audit event and its optional geometric proposal without code execution."""
    values = exact(data, SessionEvent)
    values["working_model"] = decode_model(values["working_model"])
    values["frames"] = tuple(FrameState(f) for f in values["frames"])
    if values["pending"] is not None:
        pending = exact(values["pending"], EditProposal)
        for name in ("before", "proposed"):
            pending[name] = DHRow(**exact(pending[name], DHRow))
        if pending["geometric_edit"] is not None:
            pending["geometric_edit"] = FrameEdit(**exact(pending["geometric_edit"], FrameEdit))
        values["pending"] = EditProposal(**pending)
    return SessionEvent(**values)
