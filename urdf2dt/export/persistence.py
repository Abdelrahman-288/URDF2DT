"""JSON-only validated-session archives; reload rechecks source and sampled FK."""
from base64 import b64decode, b64encode
from dataclasses import asdict, dataclass
from importlib import import_module
import json
import logging
import os
from pathlib import Path
import tempfile
from typing import Any

from urdf2dt.config import config_from_mapping
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.global_validation import GlobalFKReport, validate_global_fk
from urdf2dt.dh.types import FrameState
from urdf2dt.export.schemas import SESSION_SCHEMA, decode_event, decode_state
from urdf2dt.logging_config import git_provenance
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.pipeline import AutomaticDHResult, generate_automatic_model


def _json(data: Any) -> str:
    return json.dumps(data, indent=2, allow_nan=False) + "\n"


def _unique(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ValueError(f"Invalid JSON number: {value}")


def save_session(folder: str | Path, run: AutomaticDHResult, session: EditorSession) -> Path:
    """Export a newly validated completed session to a new directory, never overwrite."""
    if session.pending is not None or any(f != FrameState.ACCEPTED for f in session.state.frames):
        raise ValueError("Only complete accepted sessions without pending previews can be exported")
    if session.state.automatic_model != run.automatic_model:
        raise ValueError("Session baseline does not match source run")
    if session.geometry_config != run.config.geometry:
        raise ValueError("Session geometry configuration differs from source run")
    baseline = validate_global_fk(run.chain, session.state.automatic_model, run.config)
    report = validate_global_fk(run.chain, session.state.working_model, run.config)
    if not baseline.passed or not report.passed:
        raise ValueError("Export requires passing automatic and current sampled FK validation")
    target = Path(folder).resolve()
    if target.exists():
        raise FileExistsError("Choose a new export directory; existing exports are not overwritten")
    data = {"schema_version": "1.0", "dh_convention": "standard",
            "source": {"name": run.source.source.name, "source_path": run.source.source.source_path,
                       "content_base64": b64encode(run.source.source.content).decode("ascii")},
            "configuration": run.config.snapshot(), "state": asdict(session.state),
            "events": [asdict(e) for e in session.events], "geometric_frames": list(session.geometric_frames),
            "validation": report.to_dict(), "reproducibility": git_provenance()}
    import_module("jsonschema").Draft202012Validator(SESSION_SCHEMA).validate(data)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".urdf2dt-export-", dir=target.parent) as temporary:
        staging = Path(temporary) / "bundle"
        staging.mkdir()
        files = {"session.json": data, "dh_model.json": {"schema_version": "1.0", "dh_convention": "standard",
                 "model": asdict(session.state.working_model), "reproducibility": data["reproducibility"]},
                 "edit_history.json": {"records": asdict(session.state)["history"], "events": data["events"],
                                       "reproducibility": data["reproducibility"]},
                 "validation_report.json": report.to_dict(), "configuration.json": run.config.snapshot()}
        for name, content in files.items():
            (staging / name).write_text(_json(content), encoding="utf-8")
        (staging / "validation_report.md").write_text(report.markdown() + "\nReproducibility: " + _json(data["reproducibility"]), encoding="utf-8")
        os.rename(staging, target)
    logging.getLogger(__name__).info("Export completed: %s", target)
    return target / "session.json"


@dataclass(frozen=True)
class LoadedSession:
    run: AutomaticDHResult
    session: EditorSession
    report: GlobalFKReport


def load_session(path: str | Path) -> LoadedSession:
    """Read at most 50 MiB; never execute content or dereference stored source paths."""
    with Path(path).open("rb") as stream:
        content = stream.read(50 * 1024 * 1024 + 1)
    if len(content) > 50 * 1024 * 1024:
        raise ValueError("Session archive exceeds 50 MiB")
    data = json.loads(content, object_pairs_hook=_unique, parse_constant=_reject_constant)
    import_module("jsonschema").Draft202012Validator(SESSION_SCHEMA).validate(data)
    config = config_from_mapping(data["configuration"])
    source = data["source"]
    raw = b64decode(source["content_base64"], validate=True)
    uploaded = URDFInput.from_upload(source["name"], raw)
    run = generate_automatic_model(URDFInput(uploaded.name, uploaded.content, source["source_path"]), config)
    state = decode_state(data["state"])
    if state.automatic_model != run.automatic_model or any(f != FrameState.ACCEPTED for f in state.frames):
        raise ValueError("Archive baseline or completion state is inconsistent")
    events = tuple(decode_event(e) for e in data["events"])
    session = EditorSession.from_snapshot(state, events, tuple(data["geometric_frames"]), config.geometry)
    report = validate_global_fk(run.chain, state.working_model, config)
    saved, fresh = dict(data["validation"]), report.to_dict()
    saved.pop("reproducibility", None)
    fresh.pop("reproducibility", None)
    if not report.passed or saved != fresh:
        raise ValueError("Saved validation does not reproduce for this source, model and configuration")
    logging.getLogger(__name__).info("Session reloaded and sampled FK rechecked: %s", path)
    return LoadedSession(run, session, report)
