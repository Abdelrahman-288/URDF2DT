"""Pick only on a completed click, preserving camera drag behavior."""
from importlib import import_module


def install_selection(studio):
    e = studio.e
    core = e.core

    class ClickFilter(core.QObject):  # type: ignore[name-defined]
        def __init__(self):
            super().__init__(e.view)
            self.press = None
            self.dragged = False

        def eventFilter(self, watched, event):
            kind = event.type()
            if kind == core.QEvent.MouseButtonPress:
                self.press = None
                if event.button() == core.Qt.LeftButton and event.modifiers() == core.Qt.NoModifier:
                    self.press = event.position().toPoint()
                    self.dragged = False
            elif kind == core.QEvent.MouseMove and self.press is not None:
                if (event.position().toPoint() - self.press).manhattanLength() >= e.qt.QApplication.startDragDistance():
                    self.dragged = True
            elif kind == core.QEvent.MouseButtonRelease:
                if self.press is not None and event.button() == core.Qt.LeftButton:
                    point = event.position().toPoint()
                    click = not self.dragged and (point - self.press).manhattanLength() < e.qt.QApplication.startDragDistance()
                    self.press = None
                    if click:
                        self.pick(point)
            elif kind in (core.QEvent.FocusOut, core.QEvent.Hide):
                self.press = None
            return False  # VTK continues receiving all camera events.

        def pick(self, point):
            if getattr(e, '_loading_robot', False) or e.application is None:
                return
            width, height = e.view.render_window.GetSize()
            x = point.x() * width / max(1, e.view.width())
            y = height - 1 - point.y() * height / max(1, e.view.height())
            picker = import_module('vtkmodules.vtkRenderingCore').vtkPropPicker()
            picker.Pick(x, y, 0, e.view.renderer)
            actor = picker.GetViewProp()
            if actor is None:
                studio.clear_selection()
            else:
                studio.picked(actor)

    handler = ClickFilter()
    e.view.installEventFilter(handler)
    return handler
