"""Optional external-engine checks, required by the dedicated CI job."""

from pathlib import Path
import pytest
from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.dynamics.validation import ValidationSettings, validate_dynamics
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.dynamics import DynamicsConfig, JointFriction

pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("robot", ["ur5", "scara"])
def test_independent_engine(robot):
    source = URDFInput.from_path(ROOT / f"robots/dynamics/{robot}_dynamics.urdf")
    model = load_dynamic_model(source)
    report = validate_dynamics(model, source, ValidationSettings(samples=12))
    assert report["passed"], report


def test_independent_rigid_body_with_analytical_friction_reference():
    source = URDFInput.from_path(ROOT / "robots/dynamics/scara_dynamics.urdf")
    model = load_dynamic_model(source, DynamicsConfig(friction=(JointFriction("vertical_slide", .4, .7),)))
    assert validate_dynamics(model, source, ValidationSettings(samples=8))["passed"]
