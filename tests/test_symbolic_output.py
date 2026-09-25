import sympy as sp
import numpy as np
import pytest
from urdf2dt.symbolic import symbolic_fk, symbolic_jacobian
from urdf2dt.symbolic_publication import publication_latex
from urdf2dt.dh.types import DHModel, DHRow, JointType


@pytest.mark.parametrize('factory', [symbolic_fk, symbolic_jacobian])
def test_expanded_and_compact_reconstruct_same_model(factory):
    model = DHModel('mixed', (DHRow(1e-13, .23, .4, -.12, 'turn', JointType.REVOLUTE, -1),
                            DHRow(.3, -.4, .2, .13, 'slide', JointType.PRISMATIC, -1)), 'test')
    result = factory(model)
    _, _, data = result.documents(True)
    symbols = {str(q): q for q in result.symbols}
    definitions = {}
    for name, expr in data['definitions']:
        definitions[name] = sp.sympify(expr, locals=symbols | definitions)
    q = dict(zip(result.symbols, [.2, .07]))
    compact = sp.Matrix([[sp.sympify(x, locals=symbols | definitions) for x in row] for row in data['matrix']])
    expanded = sp.Matrix([[sp.sympify(x, locals=symbols) for x in row] for row in data['expanded']])
    np.testing.assert_allclose(np.array(compact.subs(q), dtype=float), np.array(result.matrix.subs(q),dtype=float), atol=1e-14)
    np.testing.assert_allclose(np.array(expanded.subs(q), dtype=float), np.array(result.matrix.subs(q),dtype=float), atol=1e-14)
    # Exact generated representation is unchanged; reparsing floating constants
    # may reorder arithmetic, so reconstruction is checked numerically above.
    assert data['expanded'] == [[str(v) for v in row] for row in result.matrix.tolist()]
    assert data['expanded'][0][0] is not None
    for mode in (False, True):
        text = publication_latex(data, r'base_1 to tool%{x}', mode)
        assert r'\documentclass' in text and r'\end{document}' in text
        assert r'base\_1 to tool\%\{x\}' in text
        assert 'LuaLaTeX' in text


def test_publication_refuses_partial_expansion():
    result = symbolic_fk(DHModel('a', (DHRow(1,0,0,0,'j'),), 'test'))
    data = result.documents(True)[2]
    data['expanded_latex'][0][0] = None
    with pytest.raises(ValueError, match='compact'):
        publication_latex(data, 'context', True)
    assert publication_latex(data, 'context', False)
