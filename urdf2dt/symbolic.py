"""Symbolic Standard-DH FK, preserving the stored model's numeric constants.

No rounding, geometry snapping, or expensive global simplification is performed.
Common-subexpression elimination makes the result compact without approximation.
"""

from importlib import import_module
from dataclasses import dataclass
from typing import Any

from urdf2dt.dh.types import DHModel, JointType
from urdf2dt.kinematics import _configuration


@dataclass(frozen=True)
class SymbolicFK:
    model: DHModel
    symbols: tuple[Any, ...]
    matrix: Any

    def evaluate(self, q):
        values = _configuration(q, len(self.symbols))
        return tuple(tuple(float(v) for v in row) for row in
                     self.matrix.subs(dict(zip(self.symbols, values))).evalf().tolist())

    def documents(self):
        sp = import_module("sympy")
        replacements, reduced = sp.cse(self.matrix, symbols=sp.numbered_symbols("t"))
        entries = []
        display = reduced[0].as_mutable()
        for i in range(3):
            for j in range(4):
                if display[i, j].free_symbols:
                    name = sp.Symbol(f"M_{i+1}{j+1}")
                    entries.append((name, display[i, j]))
                    display[i, j] = name
        key = [f"q{i+1}: {r.joint_name} [{ 'm' if r.joint_type == JointType.PRISMATIC else 'rad' }]"
               for i, r in enumerate(self.model.rows)]
        note = "Constants retain stored floating-point precision; tiny coefficients are not rounded away."
        text = "\n".join(key) + "\n\n" + note + "\n\n"
        text += "T(q) =\n" + sp.pretty(display, use_unicode=True, wrap_line=False) + "\n\n"
        text += "Intermediate expressions (evaluate in order):\n"
        text += "\n".join(f"{symbol} = {expr}" for symbol, expr in replacements)
        text += "\n\nMatrix entries:\n" + "\n".join(f"{s} = {e}" for s, e in entries)
        latex = "% " + "; ".join(key) + "\n% " + note + "\n"
        latex += "\n".join("\\[" + sp.latex(s) + " = " + sp.latex(e) + "\\]" for s, e in replacements)
        latex += "\n".join("\\[" + sp.latex(s) + " = " + sp.latex(e) + "\\]" for s, e in entries)
        latex += "\n\\[T(q) = " + sp.latex(display) + "\\]\n"
        return text, latex


def symbolic_fk(model: DHModel) -> SymbolicFK:
    """Physical selected-tip pose relative to selected base; source q convention.

    A 12-joint bound limits interactive expression growth in this first version.
    """
    if len(model.rows) > 12:
        raise ValueError("Symbolic FK currently supports at most 12 movable joints")
    sp = import_module("sympy")
    def constant(value):
        return sp.Integer(int(value)) if float(value).is_integer() else sp.Float(value, 17)
    symbols = tuple(sp.Symbol(f"q{i+1}", real=True) for i in range(len(model.rows)))
    matrix = sp.Matrix([[constant(v) for v in row] for row in model.base_transform])
    for row, q in zip(model.rows, symbols):
        a, alpha, d, theta = map(constant, (row.a, row.alpha, row.d, row.theta_offset))
        if row.joint_type == JointType.PRISMATIC:
            d += row.joint_sign * q
        else:
            theta += row.joint_sign * q
        c, s, ca, sa = sp.cos(theta), sp.sin(theta), sp.cos(alpha), sp.sin(alpha)
        matrix = matrix * sp.Matrix([[c, -s*ca, s*sa, a*c],
                                    [s, c*ca, -c*sa, a*s],
                                    [0, sa, ca, d], [0, 0, 0, 1]])
    matrix *= sp.Matrix([[constant(v) for v in row] for row in model.tool_transform])
    return SymbolicFK(model, symbols, sp.ImmutableMatrix(matrix))


def generate_documents(model, queue):
    """Spawn-safe worker; only text crosses the process boundary."""
    try:
        queue.put((True, symbolic_fk(model).documents()))
    except Exception as exc:
        queue.put((False, f"{type(exc).__name__}: {exc}"))
