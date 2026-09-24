from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
import sympy as sp

from urdf2dt.symbolic import symbolic_fk
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.kinematics import dh_fk, urdf_fk
from urdf2dt.dh.types import DHModel, DHRow, JointType
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt._transforms import rpy_transform

ROOT = Path(__file__).resolve().parents[1]


def test_planar_analytic_and_exports():
    model = DHModel("planar", (DHRow(2, 0, 0, 0, "shoulder"), DHRow(1, 0, 0, 0, "elbow")), "test")
    result = symbolic_fk(model)
    q1, q2 = result.symbols
    assert sp.trigsimp(result.matrix[0, 3] - (2*sp.cos(q1)+sp.cos(q1+q2))) == 0
    assert sp.trigsimp(result.matrix[1, 3] - (2*sp.sin(q1)+sp.sin(q1+q2))) == 0
    assert list(result.matrix[3, :]) == [0, 0, 0, 1]
    text, latex = result.documents()
    assert "shoulder" in text and "q2" in text and "\\begin{matrix}" in latex


@pytest.mark.parametrize("robot", ["ur5/ur5_serial.urdf", "scara/scara_rrpr.urdf"])
@pytest.mark.parametrize("edit", [False, True])
def test_source_and_numeric_agreement(robot, edit):
    run = generate_automatic_model(ROOT / "robots" / robot)
    model = run.automatic_model
    if edit:
        model = recompute_model(model, len(model.rows), FrameEdit(.017, .24))
        model = recompute_model(model, 1, FrameEdit(axis_flip="z"))
    result = symbolic_fk(model)
    for q in np.random.default_rng(57).uniform(-.3, .3, (3, len(model.rows))):
        np.testing.assert_allclose(result.evaluate(q), dh_fk(model, q), atol=2e-12)
        np.testing.assert_allclose(result.evaluate(q), urdf_fk(run.chain, q), atol=2e-12)


def test_offsets_signs_alignment_and_small_constants():
    model = DHModel("mixed", (DHRow(1e-13, .23, .4, -.12, "turn", JointType.CONTINUOUS, -1),
                              DHRow(.3, -.4, .2, .13, "slide", JointType.PRISMATIC, -1)),
                    "test", base_transform=rpy_transform((.3, -.2, .1), (.4, .2, -.3)),
                    tool_transform=rpy_transform((.1, .2, .3), (-.2, .3, .1)))
    result = symbolic_fk(model)
    np.testing.assert_allclose(result.evaluate((.2, .07)), dh_fk(model, (.2, .07)), atol=1e-14)
    for q in [(1,), (float('nan'), 0)]:
        with pytest.raises(ValueError):
            result.evaluate(q)
    with pytest.raises(ValueError, match="12"):
        symbolic_fk(replace(model, rows=tuple(replace(model.rows[0], joint_name=str(i)) for i in range(13))))
