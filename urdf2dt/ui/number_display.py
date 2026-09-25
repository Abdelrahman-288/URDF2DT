"""Presentation-only numbers; never use formatted strings as model values."""


def decimal(value: float) -> str:
    text = f"{value:.3f}"
    return "0.000" if text == "-0.000" else text


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
        def textFromValue(self, value):
            return decimal(value)

        def valueFromText(self, text):
            # Focus changes and arrows must not parse the rounded presentation.
            if text.strip() == decimal(self.value()) and not self.lineEdit().isModified():
                return self.value()
            return super().valueFromText(text)

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
