"""Application orchestration shared by notebook and scripted URDF workflows."""
import argparse
import logging
from pathlib import Path
from typing import Any

from urdf2dt.config import EditorConfig, load_config
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.global_validation import GlobalFKReport, validate_global_fk
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.dh.types import FrameState
from urdf2dt.export.persistence import load_session, save_session
from urdf2dt.logging_config import configure_logging
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.pipeline import generate_automatic_model


class Application:
    """One source snapshot and authoritative session; export always revalidates."""
    def __init__(self, source: str | Path | URDFInput, config: EditorConfig | None = None):
        self.run = generate_automatic_model(source, config)
        self.session = EditorSession(self.run.automatic_model, geometry=self.run.config.geometry)

    @classmethod
    def resume(cls, path: str | Path) -> "Application":
        loaded = load_session(path)
        app = cls.__new__(cls)
        app.run, app.session = loaded.run, loaded.session
        return app

    def validate(self) -> GlobalFKReport:
        if self.session.pending is not None or any(f != FrameState.ACCEPTED for f in self.session.state.frames):
            raise ValueError("Accept all frames and resolve the preview before global validation.")
        baseline = validate_global_fk(self.run.chain, self.session.state.automatic_model, self.run.config)
        if not baseline.passed:
            raise ValueError("Automatic baseline failed FK validation; investigate before validating edits.")
        return validate_global_fk(self.run.chain, self.session.state.working_model, self.run.config)

    def export(self, directory: str | Path) -> Path:
        return save_session(directory, self.run, self.session)


def launch_editor() -> Any:
    """Show the start/load screen inside a running notebook; UI imports are lazy."""
    from urdf2dt.ui.dh_editor import DHEditor
    editor = DHEditor()
    editor.display()
    return editor


def _edit(value: str) -> tuple[int, FrameEdit]:
    try:
        index, translation, rotation = value.split(":")
        frame = int(index)
        if frame < 1:
            raise ValueError("frame must be positive")
        return frame, FrameEdit(float(translation), float(rotation))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Use FRAME:TRANSLATION_M:ROTATION_RAD, e.g. 2:0.05:0") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Load URDF, confirm/edit every frame in order, validate and export.")
    parser.add_argument("source", help="Serial .urdf input")
    parser.add_argument("--output", required=True, help="New directory for validated session bundle")
    parser.add_argument("--config", help="Complete YAML configuration")
    parser.add_argument("--edit", action="append", type=_edit, default=[],
                        help="One incremental edit per frame; unspecified frames are confirmed unchanged")
    parser.add_argument("--debug-report", help="Optional new JSON file for a failed FK report; never a validated bundle")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        configure_logging(config.logging.level)
        app = Application(args.source, config)
        edits = dict(args.edit)
        if len(edits) != len(args.edit) or any(i > len(app.run.automatic_model.rows) for i in edits):
            raise ValueError("Edit frames must be unique and within this robot's F1..Fn")
        for index in range(1, len(app.run.automatic_model.rows) + 1):
            app.session.unlock_next()
            app.session.propose_edit(index, edits.get(index, FrameEdit()))
            decision = app.session.accept()
            if not decision.valid:
                raise ValueError(decision.reason)
        report = app.validate()
        if not report.passed:
            if args.debug_report:
                path = Path(args.debug_report)
                with path.open("x", encoding="utf-8") as stream:
                    stream.write(report.json_text + "\n")
            raise ValueError("Sampled FK failed. No validated export was created.")
        path = app.export(args.output)
        logging.getLogger(__name__).info("Workflow complete: %s", path)
        return 0
    except (ValueError, OSError) as exc:
        logging.getLogger(__name__).error("Workflow stopped: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
