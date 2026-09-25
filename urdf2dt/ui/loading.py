"""Responsive preflight and cooperative GUI-thread scene cancellation."""
from queue import Queue, Empty
from threading import Thread
from typing import Any


class LoadCancelled(Exception):
    """User cancellation, not an invalid robot."""


def checkpoint(editor, progress, label=None):
    if label:
        progress.setLabelText(label)
    cancellable = getattr(editor, '_load_progress', None) is progress
    editor.qt.QApplication.processEvents(
        editor.core.QEventLoop.AllEvents if cancellable
        else editor.core.QEventLoop.ExcludeUserInputEvents)
    if cancellable and progress.wasCanceled():
        raise LoadCancelled('Loading cancelled')


def prepare(editor, path, progress):
    """Run model-only preparation off the GUI thread; never touch active state."""
    from urdf2dt.parser.robot_document import RobotDocument
    from urdf2dt.visualization.assets import document_poses
    from urdf2dt.app import Application
    result: Queue[tuple[Any, Any, Exception | None]] = Queue()

    def work():
        try:
            document = RobotDocument.load(path)
            document_poses(document.root, {})
            result.put((document, Application(document.select(0)), None))
        except Exception as exc:
            result.put((None, None, exc))

    worker = Thread(target=work, daemon=True, name='URDF preflight')
    editor._load_worker = worker
    worker.start()
    # A nested Qt loop sleeps between polls and keeps the modal Cancel button live.
    while True:
        checkpoint(editor, progress)
        try:
            document, application, error = result.get_nowait()
            if error:
                raise ValueError(f'{error}. Active robot was not changed. '
                                 'Check the URDF file and its link/joint definitions.') from error
            return document, application
        except Empty:
            loop = editor.core.QEventLoop()
            editor.core.QTimer.singleShot(20, loop.quit)
            loop.exec()
