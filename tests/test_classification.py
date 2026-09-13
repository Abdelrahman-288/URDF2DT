from dataclasses import replace
from pathlib import Path
import json
import subprocess
import sys

import numpy as np
import pytest

from urdf2dt.config import GeometryConfig
from urdf2dt.dh.classification import classify_axis_pair, classify_dh_model, get_editable_params
from urdf2dt.dh.types import AxisCase
from urdf2dt.pipeline import generate_automatic_model

O = (0., 0., 0.)
Z = (0., 0., 1.)


@pytest.mark.parametrize("point,axis,case,distance", [
    ((1, 0, 0), Z, AxisCase.PARALLEL, 1),
    ((1, 0, 4), (0, 0, -2), AxisCase.PARALLEL, 1),
    ((0, 0, 4), Z, AxisCase.PARALLEL, 0),
    ((0, 3, 2), (0, 1, 0), AxisCase.INTERSECTING, 0),
    ((2, 3, 2), (0, 1, 0), AxisCase.SKEW, 2),
])
def test_analytic_cases(point, axis, case, distance):
    result = classify_axis_pair(O, Z, point, axis)
    assert result.case == case
    assert result.distance_m == pytest.approx(distance)
    assert result.description
    assert classify_axis_pair(point, axis, O, Z).case == case


def test_edit_freedoms_and_tolerance_lock():
    assert [p.name for p in get_editable_params(AxisCase.PARALLEL)] == ["axial_translation"]
    assert len(get_editable_params(classify_axis_pair(O, Z, O, Z))) == 2
    assert get_editable_params(AxisCase.INTERSECTING) == ()
    assert get_editable_params(AxisCase.SKEW) == ()
    close = classify_axis_pair(O, Z, (1e-7, 0, 0), Z)
    assert close.coincident and close.requires_review
    assert get_editable_params(close) == ()
    with pytest.raises(TypeError):
        get_editable_params("parallel")


def test_thresholds_and_near_parallel():
    cfg = GeometryConfig()
    at = classify_axis_pair(O, Z, (cfg.intersection_threshold, 0, 0), (0, 1, 0), cfg)
    assert at.case == AxisCase.INTERSECTING and at.requires_review
    assert classify_axis_pair(O, Z, (1.01e-6, 0, 0), (0, 1, 0), cfg).case == AxisCase.SKEW
    near = (0, 5e-6, 1)
    assert classify_axis_pair(O, Z, (1, 0, 0), near, cfg).requires_review
    assert classify_axis_pair(O, Z, (1, 0, 0), near,
                              replace(cfg, parallel_threshold=1e-6)).case == AxisCase.SKEW
    assert get_editable_params(classify_axis_pair(O, Z, (1, 0, 0), near)) == ()


@pytest.mark.parametrize("axis", [(0, 0, 0), (0, 1), (0, float('nan'), 1), (False, 0, 1)])
def test_bad_axes(axis):
    with pytest.raises(ValueError):
        classify_axis_pair(O, axis, O, Z)


def test_extreme_direction_and_overflow():
    assert classify_axis_pair(O, (0, 0, 1e308), O, (0, 0, 1e-300)).coincident
    with pytest.raises(ValueError):
        classify_axis_pair((1e308, 0, 0), Z, (-1e308, 0, 0), Z)


def test_independent_least_squares_and_rigid_invariance():
    rng = np.random.default_rng(7)
    for _ in range(60):
        a, b, u, v = rng.normal(size=(4, 3))
        result = classify_axis_pair(a, u, b, v)
        matrix = np.column_stack((u, -v))
        coordinates = np.linalg.lstsq(matrix, b - a, rcond=None)[0]
        distance = np.linalg.norm(a + coordinates[0]*u - b - coordinates[1]*v)
        assert result.distance_m == pytest.approx(distance, abs=1e-12)
        rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        shift = rng.normal(size=3)
        moved = classify_axis_pair(rotation @ a + shift, rotation @ u,
                                   rotation @ b + shift, rotation @ v)
        assert moved.case == result.case
        assert moved.distance_m == pytest.approx(result.distance_m, abs=1e-12)


def test_ur5_sequence_and_threshold_sweep():
    path = Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf"
    run = generate_automatic_model(path)
    for threshold in (1e-6, 1e-5, 1e-4):
        cases = classify_dh_model(run.automatic_model, GeometryConfig(parallel_threshold=threshold))
        assert [case.case.value for case in cases] == [
            "intersecting", "parallel", "parallel", "intersecting", "intersecting", "parallel"]
    # The final pair is the synthetic terminal frame, not physical joint seven.
    assert len(cases) == len(run.chain.joint_names)


def test_classification_cli_json():
    path = Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf"
    result = subprocess.run([sys.executable, "-m", "urdf2dt", str(path), "--classify", "--json"],
                            capture_output=True, text=True, check=True)
    data = json.loads(result.stdout)
    assert data["globally_validated"] is False
    assert len(data["axis_classifications"]) == 6
    assert data["axis_classifications"][-1]["terminal_convention"] is True
    assert data["axis_classifications"][1]["editable_params"][0]["name"] == "axial_translation"


def test_physical_urdf_axes_agree_with_dh_pairs():
    from urdf2dt._transforms import position, rotate
    from urdf2dt.dh.types import JointType
    from urdf2dt.kinematics import urdf_link_transforms
    path = Path(__file__).resolve().parents[1] / "robots/ur5/ur5_serial.urdf"
    run = generate_automatic_model(path)
    poses = urdf_link_transforms(run.chain, (0.,) * 6)
    axes = [(position(p), rotate(p, j.axis)) for j, p in zip(run.chain.joints, poses[1:])
            if j.joint_type != JointType.FIXED]
    physical = [classify_axis_pair(*a, *b) for a, b in zip(axes, axes[1:])]
    assert [c.case for c in physical] == [c.case for c in classify_dh_model(run.automatic_model)[:-1]]
