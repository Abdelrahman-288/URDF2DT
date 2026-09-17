"""Versioned controller configuration bound to exact model and reference values."""

from pathlib import Path
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.trajectory import Trajectory
from urdf2dt.identification.data import write_record, read_record, canonical
from .controller import Controller, ControllerConfig, ControlError


def save_controller(controller: Controller, path: str | Path) -> Path:
    """Save configuration only; restoring starts a fresh tick sequence at zero."""
    return write_record(path, "urdf2dt.controller_configuration", controller.snapshot())


def load_controller(path: str | Path, model: DynamicModel, reference: Trajectory) -> Controller:
    record = read_record(path, "urdf2dt.controller_configuration")
    controller = Controller(model, reference, ControllerConfig(**record["config"]))
    expected = controller.snapshot()
    expected.pop("schema_version")
    if canonical(record) != canonical(expected):
        raise ControlError("Saved controller model, reference, limits or conventions differ")
    return controller
