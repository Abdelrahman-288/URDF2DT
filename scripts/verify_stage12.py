"""Generate and reload an edited UR5 archive in a new output directory."""
import argparse
from pathlib import Path

from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.export.persistence import save_session, load_session
from urdf2dt.logging_config import configure_logging


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    configure_logging()
    root = Path(__file__).resolve().parents[1]
    run = generate_automatic_model(root / "robots/ur5/ur5_serial.urdf")
    session = EditorSession(run.automatic_model, geometry=run.config.geometry)
    for i in range(1, 7):
        session.unlock_next()
        session.propose_edit(i, FrameEdit(.03) if i in (2, 6) else FrameEdit())
        session.accept()
    path = save_session(args.output, run, session)
    loaded = load_session(path)
    assert loaded.session.state == session.state
    assert loaded.session.events == session.events
    assert loaded.report.passed


if __name__ == "__main__":
    main()
