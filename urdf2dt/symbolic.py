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
    label: str = "T"

    def evaluate(self, q):
        values = _configuration(q, len(self.symbols))
        return tuple(tuple(float(v) for v in row) for row in
                     self.matrix.subs(dict(zip(self.symbols, values))).evalf().tolist())

    def documents(self, include_layout=False):
        sp = import_module("sympy")
        replacements, reduced = sp.cse(self.matrix, symbols=sp.numbered_symbols("t"))
        entries = []
        display = reduced[0].as_mutable()
        for i in range(self.matrix.rows):
            for j in range(self.matrix.cols):
                if display[i, j].free_symbols:
                    name = sp.Symbol(f"M_{i+1}{j+1}")
                    entries.append((name, display[i, j]))
                    display[i, j] = name
        key = [f"q{i+1}: {r.joint_name} [{ 'm' if r.joint_type == JointType.PRISMATIC else 'rad' }]"
               for i, r in enumerate(self.model.rows)]
        note = "Constants retain stored floating-point precision; tiny coefficients are not rounded away."
        text = "\n".join(key) + "\n\n" + note + "\n\n"
        if self.label == "J":
            note += " Geometric Jacobian: tip origin, expressed in selected base; rows [vx, vy, vz, wx, wy, wz]. [v; omega] = J(q) qdot; not RPY rates."
            text = "\n".join(key) + "\n\n" + note + "\n\n"
        text += self.label + "(q) =\n" + sp.pretty(display, use_unicode=True, wrap_line=False) + "\n\n"
        text += "Intermediate expressions (evaluate in order):\n"
        text += "\n".join(f"{symbol} = {expr}" for symbol, expr in replacements)
        text += "\n\nMatrix entries:\n" + "\n".join(f"{s} = {e}" for s, e in entries)
        latex = "% " + "; ".join(key) + "\n% " + note + "\n"
        latex += "\n".join("\\[" + sp.latex(s) + " = " + sp.latex(e) + "\\]" for s, e in replacements)
        latex += "\n".join("\\[" + sp.latex(s) + " = " + sp.latex(e) + "\\]" for s, e in entries)
        latex += "\n\\[" + self.label + "(q) = " + sp.latex(display) + "\\]\n"
        if include_layout:
            # Expansion means the original expressions without CSE aliases, not
            # distributive/trigonometric rewriting or approximate simplification.
            # Bound each entry so large robots retain usable compact output.
            expanded, expanded_latex = [], []
            remaining = 1_000_000
            for row in self.matrix.tolist():
                texts, formulas = [], []
                for value in row:
                    allowed = sp.count_ops(value) <= 20000
                    raw = str(value) if allowed else ''
                    formula = sp.latex(value) if allowed else ''
                    allowed = allowed and len(raw) + len(formula) <= remaining
                    if allowed:
                        remaining -= len(raw) + len(formula)
                    texts.append(raw if allowed else None)
                    formulas.append(formula if allowed else None)
                expanded.append(texts)
                expanded_latex.append(formulas)
            return text, latex, {
                'matrix': [[str(v) for v in row] for row in display.tolist()],
                'definitions': [(str(s), str(expr)) for s, expr in [*replacements, *entries]],
                'joint_key': key, 'label': self.label,
                'expanded': expanded, 'expanded_latex': expanded_latex,
                'definition_latex': [(sp.latex(s), sp.latex(expr)) for s, expr in [*replacements, *entries]],
                'matrix_latex': sp.latex(display),
            }
        return text, latex


def _symbolic_chain(model: DHModel):
    """Physical selected-tip pose relative to selected base; source q convention.

    A 12-joint bound limits interactive expression growth in this first version.
    """
    if len(model.rows) > 12:
        raise ValueError("Symbolic kinematics currently supports at most 12 movable joints")
    sp = import_module("sympy")
    def constant(value):
        return sp.Integer(int(value)) if float(value).is_integer() else sp.Float(value, 17)
    symbols = tuple(sp.Symbol(f"q{i+1}", real=True) for i in range(len(model.rows)))
    matrix = sp.Matrix([[constant(v) for v in row] for row in model.base_transform])
    frames = []
    for row, q in zip(model.rows, symbols):
        frames.append(matrix)
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
    return symbols, frames, matrix


def symbolic_fk(model: DHModel) -> SymbolicFK:
    """Physical selected-tip pose relative to selected base; source q convention."""
    sp = import_module("sympy")
    symbols, _, matrix = _symbolic_chain(model)
    return SymbolicFK(model, symbols, sp.ImmutableMatrix(matrix))


def symbolic_jacobian(model: DHModel) -> SymbolicFK:
    """Geometric 6xn Jacobian at the physical tip origin, expressed in base.

    Standard DH uses the preceding frame's z axis. Multiplying both blocks by
    joint_sign differentiates with respect to the original source coordinate.
    Fixed base/tool alignment and terminal translations are included.
    """
    sp = import_module("sympy")
    symbols, frames, tip = _symbolic_chain(model)
    columns = []
    for row, frame in zip(model.rows, frames):
        axis = frame[:3, 2] * row.joint_sign
        if row.joint_type == JointType.PRISMATIC:
            column = axis.col_join(sp.zeros(3, 1))
        else:
            column = axis.cross(tip[:3, 3] - frame[:3, 3]).col_join(axis)
        columns.append(column)
    matrix = sp.Matrix.hstack(*columns) if columns else sp.zeros(6, 0)
    return SymbolicFK(model, symbols, sp.ImmutableMatrix(matrix), "J")


def generate_documents(model, queue, kind="FK"):
    """Spawn-safe worker; only text crosses the process boundary."""
    try:
        result = symbolic_jacobian(model) if kind == "Jacobian" else symbolic_fk(model)
        queue.put((True, result.documents(include_layout=True)))
    except Exception as exc:
        queue.put((False, f"{type(exc).__name__}: {exc}"))
