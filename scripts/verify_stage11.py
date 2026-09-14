"""Write reproducible automatic and completely accepted edited UR5 FK reports."""
from pathlib import Path

from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.recompute import FrameEdit
from urdf2dt.dh.global_validation import validate_global_fk


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    run = generate_automatic_model(root / "robots/ur5/ur5_serial.urdf")
    session = EditorSession(run.automatic_model, geometry=run.config.geometry)
    automatic = validate_global_fk(run.chain, run.automatic_model, run.config)
    if not automatic.passed:
        raise RuntimeError("Automatic baseline failed; edited validation cancelled.")
    for i in range(1, 7):
        session.unlock_next()
        edit = FrameEdit(.05, .2 if i == 6 else 0.) if i in (2, 3, 6) else FrameEdit()
        session.propose_edit(i, edit)
        session.accept()
    edited = validate_global_fk(run.chain, session.state.working_model, run.config)
    folder = root / "outputs/validation_reports"
    folder.mkdir(parents=True, exist_ok=True)
    for name, report in (("automatic", automatic), ("edited", edited)):
        (folder / f"stage11_{name}.json").write_text(report.json_text + "\n", encoding="utf-8")
        (folder / f"stage11_{name}.md").write_text(report.markdown(), encoding="utf-8")
        print(report.markdown())
    if not edited.passed:
        raise RuntimeError("Edited model failed sampled FK validation.")


if __name__ == "__main__":
    main()
