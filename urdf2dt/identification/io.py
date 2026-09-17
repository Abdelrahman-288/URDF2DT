"""Source-bound identified parameter archives; original URDF remains unchanged."""

from pathlib import Path
from typing import Any
import numpy as np
from urdf2dt.config import EditorConfig
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import validate_urdf
from .data import write_record, read_record
from .regressor import InertialRegressor
from .fit import IdentificationResult


def source_regressor(source: URDFInput, gravity: Any = (0., 0., -9.81),
                     massless_links: tuple[str, ...] = ()) -> InertialRegressor:
    chain = SerialURDFParser().parse(validate_urdf(source).require_valid(), EditorConfig())
    return InertialRegressor(chain, gravity, massless_links)


def export_parameters(path: str | Path, source: URDFInput, regressor: InertialRegressor,
                      result: IdentificationResult, *, mode: str = "identified") -> Path:
    checked = source_regressor(source, regressor.gravity, regressor.massless_links)
    if checked.chain != regressor.chain:
        raise ValueError("Parameter archive source does not match regressor chain")
    if mode not in {"identified", "trusted_manufacturer"}:
        raise ValueError("Unknown parameter source mode")
    regressor.model(result.parameters, f"Stage 25 {mode}")
    if mode == "identified" and not result.report.get("training_sha256"):
        raise ValueError("Identified parameters require training provenance")
    if mode == "trusted_manufacturer" and not result.report.get("manufacturer_provenance", "").strip():
        raise ValueError("Trusted parameters require explicit manufacturer provenance")
    return write_record(path, "urdf2dt.identified_parameters", {
        "mode": mode, "source_name": source.name, "source_hex": source.content.hex(),
        "source_sha256": source.sha256, "joint_names": regressor.chain.joint_names,
        "gravity": regressor.gravity, "massless_links": regressor.massless_links,
        "parameter_names": regressor.names, "parameter_units": regressor.units, "parameters": result.parameters,
        "parameter_convention": "SI: link-origin m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz; joint viscous,Coulomb",
        "report": result.report})


def load_parameters(path: str | Path) -> tuple[DynamicModel, dict]:
    data = read_record(path, "urdf2dt.identified_parameters")
    source = URDFInput.from_upload(data["source_name"], bytes.fromhex(data["source_hex"]))
    if source.sha256 != data["source_sha256"]:
        raise ValueError("Parameter archive source checksum mismatch")
    regressor = source_regressor(source, data["gravity"], tuple(data["massless_links"]))
    if (list(regressor.names) != data["parameter_names"] or list(regressor.chain.joint_names) != data["joint_names"]
        or list(regressor.units) != data["parameter_units"]):
        raise ValueError("Parameter archive ordering mismatch")
    mode = data["mode"]
    if mode not in {"identified", "trusted_manufacturer"}:
        raise ValueError("Unsupported parameter mode")
    if mode == "identified" and not data["report"].get("training_sha256"):
        raise ValueError("Missing training provenance")
    if mode == "trusted_manufacturer" and not data["report"].get("manufacturer_provenance", "").strip():
        raise ValueError("Missing manufacturer provenance")
    if mode == "trusted_manufacturer":
        nominal = load_dynamic_model(source)
        if not np.array_equal(regressor.parameters(nominal), data["parameters"]):
            raise ValueError("Trusted parameters differ from the manufacturer source snapshot")
    model = regressor.model(data["parameters"], f"Stage 25 {mode}; see parameter archive for provenance")
    return model, data


def trust_manufacturer(source: URDFInput, path: str | Path, provenance: str) -> Path:
    """Explicit opt-in, physical validation, no claim of experimental identification."""
    if not provenance.strip():
        raise ValueError("Supply manufacturer provenance: document/revision and acceptance rationale")
    model = load_dynamic_model(source)
    markers = tuple(r.link_name for r in model.inertials.records
                    if r.properties is not None and r.properties.mass == 0
                    and any(j.child_link == r.link_name for j in model.chain.joints))
    moving = set(InertialRegressor(model.chain).links)
    regressor = InertialRegressor(model.chain, model.config.gravity, tuple(m for m in markers if m in moving))
    result = IdentificationResult(tuple(regressor.parameters(model)), {
        "manufacturer_provenance": provenance, "fitting_performed": False,
        "hardware_accuracy_validated": False, "mode": "trusted_manufacturer"})
    return export_parameters(path, source, regressor, result, mode="trusted_manufacturer")
