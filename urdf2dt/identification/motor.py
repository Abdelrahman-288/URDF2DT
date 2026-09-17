"""Conditional motor identification with independently trusted rigid-body dynamics."""

from dataclasses import asdict
from hashlib import sha256
from importlib import import_module
from pathlib import Path
from typing import Any
import numpy as np

from urdf2dt.dynamics.model import DynamicModel
from .data import IdentificationData, canonical, require_held_out, write_record, read_record


def identify_motor(model: DynamicModel, training: IdentificationData, *,
                   motor_speed_ratio: Any, calibration_provenance: str) -> dict:
    """Fit reflected rotor inertia and kinetic friction with known rigid inertias.

    Requires calibrated current-to-joint-effort data and constant transmission
    speed ratios (motor rad / joint rad, or motor rad / joint m). Unknown torque
    gains cannot be jointly identified with unknown inertial scales. Do not use
    a fitted nonunique rigid-body representative as an independently known model.
    """
    training.validate_chain(model.chain)
    ratio = np.asarray(motor_speed_ratio, dtype=float)
    if (training.current_calibration is None or not calibration_provenance.strip()
        or ratio.shape != (model.dof,) or not np.isfinite(ratio).all() or np.any(ratio == 0)):
        raise ValueError("Motor identification needs current calibration, trusted rigid-model provenance and nonzero speed ratios")
    optimize = import_module("scipy.optimize")
    rigid = np.array([model.inverse_dynamics(q, v, a)-model.friction_effort(v)
                      for q, v, a in zip(training.q, training.velocity, training.acceleration)])
    residual = np.asarray(training.effort)-rigid
    velocity, acceleration = np.asarray(training.velocity), np.asarray(training.acceleration)
    coefficients, errors, conditions = [], [], []
    for i in range(model.dof):
        x = np.column_stack([acceleration[:, i], velocity[:, i], np.sign(velocity[:, i])])
        norm = np.linalg.norm(x, axis=0)
        if np.any(norm == 0):
            raise ValueError("Insufficient motor excitation")
        scaled = x/norm
        singular = np.linalg.svd(scaled, compute_uv=False)
        if singular[-1] <= singular[0]*1e-8:
            raise ValueError("Insufficient motor excitation: inertia/friction cannot be separated")
        fit, _ = optimize.nnls(scaled, residual[:, i])
        beta = fit/norm
        noise = residual[:, i]-x@beta
        if len(noise) <= 3:
            raise ValueError("Motor uncertainty requires more than three observations")
        variance = float(noise@noise/(len(noise)-3))
        standard_error = np.sqrt(np.diag(np.linalg.inv(x.T@x))*variance)
        coefficients.append(beta.tolist())
        errors.append(standard_error.tolist())
        conditions.append(float(singular[0]/singular[-1]))
    fitted = np.asarray(coefficients)
    return {
        "mode": "calibrated_motor_with_known_rigid_model", "source_sha256": model.chain.source_sha256,
        "rigid_model_sha256": sha256(canonical({"inertials": asdict(model.inertials), "gravity": model.config.gravity})).hexdigest(),
        "joint_names": model.chain.joint_names, "training_sha256": training.digest,
        "calibration_provenance": calibration_provenance, "current_calibration": asdict(training.current_calibration),
        "motor_speed_ratio": ratio.tolist(), "reflected_inertia_viscous_coulomb": coefficients,
        "rotor_inertia_kg_m2": (fitted[:, 0]/ratio**2).tolist(),
        "standard_error_reflected_inertia_viscous_coulomb": errors, "excitation_condition_number": conditions,
        "assumptions": "Known rigid inertias and gravity, rigid constant-ratio transmission, calibrated joint-side current conversion, kinetic friction, no contact. Approximate unconstrained linear standard errors; exact derivatives and independent effort noise.",
        "hardware_accuracy_validated": False,
    }


def evaluate_motor(model: DynamicModel, result: dict, training: IdentificationData,
                   held_out: IdentificationData) -> dict:
    require_held_out(training, held_out)
    held_out.validate_chain(model.chain)
    model_digest = sha256(canonical({"inertials": asdict(model.inertials), "gravity": model.config.gravity})).hexdigest()
    if result["training_sha256"] != training.digest or result["rigid_model_sha256"] != model_digest:
        raise ValueError("Motor result training or rigid-model identity differs")
    beta = np.asarray(result["reflected_inertia_viscous_coulomb"])
    predictions, baselines = [], []
    for q, v, a in zip(held_out.q, held_out.velocity, held_out.acceleration):
        baseline = model.inverse_dynamics(q, v, a)
        rigid = baseline-model.friction_effort(v)
        predictions.append(rigid+beta[:, 0]*a+beta[:, 1]*v+beta[:, 2]*np.sign(v))
        baselines.append(baseline)
    effort = np.asarray(held_out.effort)
    return {"held_out_sha256": held_out.digest,
            "fitted_rmse_per_joint": np.sqrt(np.mean((np.asarray(predictions)-effort)**2, axis=0)).tolist(),
            "baseline_rmse_per_joint": np.sqrt(np.mean((np.asarray(baselines)-effort)**2, axis=0)).tolist(),
            "effort_units": ["N" if u == "m" else "N*m" for u in held_out.position_units]}


def save_motor_parameters(path: str | Path, result: dict) -> Path:
    """Versioned motor record; retain the associated source/model and datasets too."""
    return write_record(path, "urdf2dt.identified_motor_parameters", result)


def load_motor_parameters(path: str | Path, model: DynamicModel) -> dict:
    result = read_record(path, "urdf2dt.identified_motor_parameters")
    digest = sha256(canonical({"inertials": asdict(model.inertials), "gravity": model.config.gravity})).hexdigest()
    beta = np.asarray(result["reflected_inertia_viscous_coulomb"], dtype=float)
    ratio = np.asarray(result["motor_speed_ratio"], dtype=float)
    if (result["rigid_model_sha256"] != digest or result["source_sha256"] != model.chain.source_sha256
        or tuple(result["joint_names"]) != model.chain.joint_names
        or beta.shape != (model.dof, 3) or not np.isfinite(beta).all() or np.any(beta < 0)
        or ratio.shape != (model.dof,) or not np.isfinite(ratio).all() or np.any(ratio == 0)
        or not np.allclose(result["rotor_inertia_kg_m2"], beta[:, 0]/ratio**2, rtol=1e-12, atol=1e-12)):
        raise ValueError("Motor parameter archive/model association or physical values invalid")
    return result
