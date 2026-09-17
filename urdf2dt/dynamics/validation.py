"""Seeded numerical evidence; thresholds are engineering defaults, not hardware acceptance."""

from dataclasses import asdict, dataclass
from importlib.metadata import version
from time import perf_counter
from typing import Any
import numpy as np

from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.dynamics.reference import MuJoCoReference
from urdf2dt.kinematics import urdf_fk
from urdf2dt.logging_config import git_provenance
from urdf2dt.parser.urdf_input import URDFInput


@dataclass(frozen=True)
class ValidationSettings:
    samples: int = 64
    seed: int = 2401
    effort_atol: float = 1e-8
    acceleration_atol: float = 1e-8
    mass_atol: float = 1e-9
    pose_atol: float = 1e-9

    def __post_init__(self) -> None:
        if type(self.samples) is not int or self.samples < 2 or type(self.seed) is not int or self.seed < 0:
            raise ValueError("Use at least two samples and a nonnegative integer seed")
        if any(not np.isfinite(v) or v <= 0 for v in
               (self.effort_atol, self.acceleration_atol, self.mass_atol, self.pose_atol)):
            raise ValueError("Validation tolerances must be positive finite values")


def validate_dynamics(model: DynamicModel, source: URDFInput,
                      settings: ValidationSettings = ValidationSettings()) -> dict[str, Any]:
    """Compare instantaneous dynamics with a separate engine over saved sampled states."""
    reference = MuJoCoReference(source, model)
    rng = np.random.default_rng(settings.seed)
    joints = [j for j in model.chain.joints if j.name in model.chain.joint_names]
    effort_errors, acceleration_errors, samples = [], [], []
    maxima = dict(mass_error=0., symmetry_error=0., pose_error=0., inverse_forward_error=0., gravity_equilibrium_error=0.)
    minimum_eigenvalue = float("inf")
    timings = []
    for index in range(settings.samples):
        q = np.array([rng.uniform(j.limit.lower, j.limit.upper) if j.limit else rng.uniform(-np.pi, np.pi)
                      for j in joints]) if index else np.zeros(model.dof)
        v, a = rng.uniform(-1, 1, (2, model.dof))
        start = perf_counter()
        effort = model.inverse_dynamics(q, v, a)
        restored = model.forward_dynamics(q, v, effort)
        timings.append(perf_counter() - start)
        other = reference.inverse(q, v, a)
        accelerated = reference.forward(q, v, effort)
        mass = model.mass_matrix(q)
        gravity = model.gravity_effort(q)
        effort_errors.append(effort - other)
        acceleration_errors.append(a - accelerated)
        maxima["mass_error"] = max(maxima["mass_error"], float(np.max(np.abs(mass-reference.mass(q)))))
        maxima["symmetry_error"] = max(maxima["symmetry_error"], float(np.max(np.abs(mass-mass.T))))
        maxima["pose_error"] = max(maxima["pose_error"], reference.link_pose_error(q),
            float(np.max(np.abs(model.tip_transform(q)-np.array(urdf_fk(model.chain, q.tolist()), dtype=float)))))
        maxima["inverse_forward_error"] = max(maxima["inverse_forward_error"], float(np.max(np.abs(restored-a))))
        maxima["gravity_equilibrium_error"] = max(maxima["gravity_equilibrium_error"],
            float(np.max(np.abs(reference.forward(q, np.zeros(model.dof), gravity)))))
        minimum_eigenvalue = min(minimum_eigenvalue, float(np.linalg.eigvalsh(mass)[0]))
        samples.append({"q": q.tolist(), "velocity": v.tolist(), "acceleration": a.tolist(),
                        "effort": effort.tolist(), "reference_effort": other.tolist()})
    torque, acceleration = np.asarray(effort_errors), np.asarray(acceleration_errors)
    per_joint: list[dict[str, Any]] = [{"joint": j.name, "effort_unit": "N" if j.joint_type.value == "prismatic" else "N*m",
                  "acceleration_unit": "m/s^2" if j.joint_type.value == "prismatic" else "rad/s^2",
                  "effort_rmse": float(np.sqrt(np.mean(torque[:, i]**2))),
                  "effort_max_error": float(np.max(np.abs(torque[:, i]))),
                  "acceleration_rmse": float(np.sqrt(np.mean(acceleration[:, i]**2))),
                  "acceleration_max_error": float(np.max(np.abs(acceleration[:, i])))} for i, j in enumerate(joints)]
    passed = (all(j["effort_max_error"] <= settings.effort_atol and
                  j["acceleration_max_error"] <= settings.acceleration_atol for j in per_joint)
              and maxima["mass_error"] <= settings.mass_atol
              and maxima["symmetry_error"] <= settings.mass_atol
              and maxima["pose_error"] <= settings.pose_atol
              and maxima["inverse_forward_error"] <= settings.acceleration_atol
              and maxima["gravity_equilibrium_error"] <= settings.acceleration_atol
              and minimum_eigenvalue > 0)
    return {"schema_version": "1.0", "passed": passed, "source_sha256": source.sha256,
            "robot": model.chain.robot_name, "settings": asdict(settings), "provenance": git_provenance(),
            "versions": {name: version(name) for name in ("numpy", "casadi", "mujoco")},
            "parameter_provenance": model.config.parameter_provenance,
            "gravity": model.config.gravity, "friction": [asdict(f) for f in model.config.friction],
            "friction_reference": "independent scalar viscous/Coulomb law; MuJoCo compares rigid-body terms, not stiction",
            "reference_adapter": "MuJoCo raw-URDF import with tensors rotated into link axes for upstream #3559; imported masses/COMs/tensors and all link poses checked",
            "sampling": "zero plus uniform bounded positions; continuous [-pi,pi]; velocities/accelerations [-1,1] in joint SI units",
            "minimum_mass_eigenvalue": minimum_eigenvalue, "maxima": maxima, "per_joint": per_joint,
            "mean_inverse_and_forward_seconds": float(np.mean(timings)), "samples": samples,
            "limitations": ["nominal URDF parameters, not physical-robot accuracy",
                            "engineering thresholds awaiting advisor agreement",
                            "no contact, constraints, compliance, transmissions or stiction",
                            "mass and pose maxima are numerical matrix-element checks; mixed-joint entries have different SI units",
                            "zero sample may lie outside position limits; limits are not enforced by dynamics"]}
