"""Presentation-only numbers; never use formatted strings as model values."""


def decimal(value: float, digits: int = 3) -> str:
    text = f"{value:.{digits}f}"
    return f"{0:.{digits}f}" if float(text) == 0 else text


def residual(value: float) -> str:
    return f"{value:.3e}" if 0 < abs(value) < .001 else decimal(value)


def display(value) -> str:
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(display(v) for v in value) + "]"
    if isinstance(value, float):
        return decimal(value)
    return str(value)


def spin_box(qt):
    """Keep full internal precision while painting three decimal places."""
    class DisplaySpinBox(qt.QDoubleSpinBox):  # type: ignore[name-defined]  # Qt loaded at runtime
        display_factor = 1.0
        display_digits = 3

        def textFromValue(self, value):
            return decimal(value * self.display_factor, self.display_digits)

        def clean_number(self, text):
            return text.removeprefix(self.prefix()).removesuffix(self.suffix()).strip()

        def valueFromText(self, text):
            # Focus changes and arrows must not parse the rounded presentation.
            if self.clean_number(text) == self.textFromValue(self.value()) and not self.lineEdit().isModified():
                return self.value()
            try:
                return float(self.clean_number(text)) / self.display_factor
            except ValueError:
                return self.value()

        def validate(self, text, position):
            from PySide6.QtGui import QValidator
            from math import isfinite
            try:
                value = float(self.clean_number(text)) / self.display_factor
                state = QValidator.State.Acceptable if isfinite(value) and self.minimum() <= value <= self.maximum() else QValidator.State.Intermediate
            except ValueError:
                state = QValidator.State.Intermediate
            return state, text, position

        def set_display(self, factor=1.0, digits=3):
            if (factor, digits) == (self.display_factor, self.display_digits):
                return
            self.display_factor, self.display_digits = factor, digits
            self.lineEdit().setText(self.prefix()+self.textFromValue(self.value())+self.suffix())
            self.lineEdit().setModified(False)

    widget = DisplaySpinBox()
    widget.setDecimals(17)
    return widget


def number_line(qt, text=""):
    """Display a vector compactly; unchanged form submission retains raw values."""
    class NumberLine(qt.QLineEdit):  # type: ignore[name-defined]  # Qt loaded at runtime
        def setText(self, value):
            self.raw = value
            try:
                shown = " ".join(decimal(float(v)) for v in value.split())
            except ValueError:
                shown = value
            super().setText(shown)

        def preciseText(self):
            return self.text() if self.isModified() else self.raw

    widget = NumberLine()
    widget.setText(text)
    return widget
