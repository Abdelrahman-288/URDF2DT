"""Training-only SVD diagnostics and physically constrained least squares."""

from dataclasses import dataclass
from importlib import import_module
from typing import Any
import numpy as np

from urdf2dt.dh.types import JointType
from .data import IdentificationData, require_held_out
from .regressor import InertialRegressor, pseudo_inertia


@dataclass(frozen=True)
class IdentificationResult:
    parameters: tuple[float, ...]
    report: dict[str, Any]


def _probe(regressor: InertialRegressor) -> np.ndarray:
    """Generic numerical rank estimate, independent of evaluation observations."""
    rng = np.random.default_rng(2501)
    joints = [j for j in regressor.chain.joints if j.joint_type != JointType.FIXED]
    rows = []
    for _ in range(max(128, 3*len(regressor.names)//regressor.dof)):
        q = [rng.uniform(j.limit.lower, j.limit.upper) if j.limit is not None
             else rng.uniform(-np.pi, np.pi) for j in joints]
        rows.append(regressor.matrix(q, rng.normal(size=regressor.dof), rng.normal(size=regressor.dof)))
    return np.vstack(rows)


def identify(regressor: InertialRegressor, training: IdentificationData, *,
             prior: Any = None, effort_scale: Any = None, rank_tolerance: float = 1e-8,
             nullspace_penalty: float = 1e-6) -> IdentificationResult:
    """Fit on training data only; no evaluation data accepted by this function.

    Full-rank means the numerically probed identifiable subspace, never the full
    raw inertial vector. A nullspace-only prior selects one physical representative.
    Physical constraints use a 1e-6 margin after reference scaling by 1kg and 1m.
    Weights are fixed per joint (1 N or 1 N*m by default), not learned on test data.
    """
    if (not np.isfinite(rank_tolerance) or not 0 < rank_tolerance < 1e-2
        or not np.isfinite(nullspace_penalty) or nullspace_penalty <= 0):
        raise ValueError("Supply a small positive rank tolerance and positive nullspace penalty")
    y = regressor.stack(training)
    effort = np.asarray(training.effort).ravel()
    scale = np.ones(regressor.dof) if effort_scale is None else np.asarray(effort_scale, dtype=float)
    if scale.shape != (regressor.dof,) or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError("Effort scales must be finite and positive for each joint")
    weights = np.tile(scale, len(training.time))
    weighted = y/weights[:, None]
    target = effort/weights
    probe = _probe(regressor)
    probe /= np.tile(scale, len(probe)//regressor.dof)[:, None]
    norm = np.linalg.norm(probe, axis=0)
    parameter_scale = 1/np.where(norm > 1e-12, norm, 1.)
    generic_singular = np.linalg.svd(probe*parameter_scale, compute_uv=False)
    expected_rank = int(np.sum(generic_singular > generic_singular[0]*rank_tolerance))
    # Reduced SVD avoids samples² memory; more rows than parameters are required.
    if len(target) <= len(regressor.names):
        raise ValueError("Supply more training observations than raw parameters")
    u, singular, vt = np.linalg.svd(weighted*parameter_scale, full_matrices=False)
    rank = int(np.sum(singular > max(singular[0]*rank_tolerance, 1e-14)))
    if rank < expected_rank:
        raise ValueError(f"Insufficient excitation: rank {rank}, generic probe rank {expected_rank}")
    basis, nullspace = vt[:rank], vt[rank:]
    initial = regressor.generic_prior() if prior is None else np.asarray(prior, dtype=float)
    regressor.model(initial, "Physical prior check")
    cp = import_module("cvxpy")  # optional dependency; core parsing does not import it
    p = cp.Variable(len(regressor.names))
    constraints = []
    for i in range(len(regressor.links)):
        b = p[10*i:10*i+10]
        inertia = cp.bmat([[b[4], b[7], b[8]], [b[7], b[5], b[9]], [b[8], b[9], b[6]]])
        covariance = .5*cp.trace(inertia)*np.eye(3)-inertia
        moment = cp.bmat([[covariance, cp.reshape(b[1:4], (3, 1), order="C")],
                         [cp.reshape(b[1:4], (1, 3), order="C"), cp.reshape(b[0], (1, 1), order="C")]])
        constraints.append(moment >> 1e-6*np.eye(4))
    constraints.append(p[10*len(regressor.links):] >= 0)
    objective = cp.sum_squares(weighted@p-target)
    if len(nullspace):
        objective += nullspace_penalty*cp.sum_squares(nullspace@cp.multiply(1/parameter_scale, p-initial))
    problem = cp.Problem(cp.Minimize(objective), constraints)
    problem.solve(solver="CLARABEL", tol_gap_abs=1e-9, tol_gap_rel=1e-9, tol_feas=1e-9, max_iter=300)
    if problem.status != "optimal" or p.value is None:
        raise ValueError(f"Identification did not converge to optimal: {problem.status}")
    parameters = np.asarray(p.value).ravel()
    friction = parameters[10*len(regressor.links):]
    if np.min(friction) < -1e-8:
        raise ValueError("Solver returned negative friction")
    clipped = int(np.sum(friction < 0))
    friction[friction < 0] = 0  # only feasibility roundoff, explicitly reported
    # Reconstruct using Stage 24's independent COM/tensor physical checks.
    model = regressor.model(parameters, "Identified physical representative; raw parameters may be unobservable")
    for q in training.q:
        model.mass_matrix(q)
    residual = weighted@parameters-target
    variance = float(residual@residual/(len(target)-rank))
    unconstrained = (u[:, :rank].T@target)/singular[:rank]
    report = {
        "method": "linear inertial regressor; Clarabel semidefinite constrained least squares",
        "training_sha256": training.digest, "training_trajectories": sorted(set(training.trajectory)),
        "raw_parameter_count": len(parameters), "identifiable_rank": rank,
        "generic_probe_rank": expected_rank, "generic_probe_seed": 2501,
        "rank_tolerance": rank_tolerance, "nullity": len(parameters)-rank,
        "excitation_condition_number": float(singular[0]/singular[rank-1]),
        "singular_values": singular.tolist(), "parameter_names": regressor.names, "parameter_units": regressor.units,
        "parameter_scale": parameter_scale.tolist(),
        "identifiable_basis": basis.tolist(),
        "basis_definition": "beta = identifiable_basis @ (parameters / parameter_scale)",
        "estimated_beta": (basis@(parameters/parameter_scale)).tolist(),
        "unconstrained_beta": unconstrained.tolist(),
        "beta_standard_error": (np.sqrt(variance)/singular[:rank]).tolist(),
        "uncertainty_assumptions": "Approximate unconstrained linear standard errors; independent homoscedastic weighted effort noise, exact q/v/a. Not constrained posterior intervals or raw-parameter confidence bounds. Derivative noise can bias estimates.",
        "raw_parameter_interpretation": "One physically consistent representative, not individually recovered masses/COM/inertias when nullity > 0",
        "effort_scale": scale.tolist(), "nullspace_penalty": nullspace_penalty,
        "prior": initial.tolist(), "prior_source": "generic 1kg/.02kg*m²" if prior is None else "supplied physical prior",
        "physical_margin": "pseudo-inertia >= 1e-6 I after 1kg/1m reference scaling",
        "minimum_pseudo_inertia_eigenvalue": min(float(np.linalg.eigvalsh(pseudo_inertia(parameters[k:k+10]))[0])
                                                     for k in range(0, 10*len(regressor.links), 10)),
        "friction_roundoff_clipped_count": clipped, "solver_status": problem.status,
        "training_rmse_per_joint": np.sqrt(np.mean((y@parameters-effort).reshape(-1, regressor.dof)**2, axis=0)).tolist(),
        "hardware_accuracy_validated": False,
    }
    return IdentificationResult(tuple(float(x) for x in parameters), report)


def evaluate(regressor: InertialRegressor, result: IdentificationResult,
             training: IdentificationData, held_out: IdentificationData,
             baseline: Any = None) -> dict:
    require_held_out(training, held_out)
    if result.report["training_sha256"] != training.digest:
        raise ValueError("Evaluation training data differs from fitted dataset")
    y = regressor.stack(held_out)
    effort = np.asarray(held_out.effort)
    prediction = (y@np.asarray(result.parameters)).reshape(effort.shape)
    report = {"dataset_sha256": held_out.digest, "trajectories": sorted(set(held_out.trajectory)),
              "fitted_rmse_per_joint": np.sqrt(np.mean((prediction-effort)**2, axis=0)).tolist(),
              "effort_units": ["N" if u == "m" else "N*m" for u in held_out.position_units],
              "independence": "Entire trajectories excluded from fitting; no held-out hyperparameter selection",
              "origin": held_out.origin}
    if baseline is not None:
        nominal = np.asarray(baseline, dtype=float)
        regressor.model(nominal, "Evaluation baseline")
        difference = (y@nominal).reshape(effort.shape)-effort
        report["baseline_rmse_per_joint"] = np.sqrt(np.mean(difference**2, axis=0)).tolist()
    return report
