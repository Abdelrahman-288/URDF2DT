"""Reproducible analytic excitation with independently generated MuJoCo efforts."""

from importlib.metadata import version
from pathlib import Path
import platform
import subprocess
from typing import Any
import numpy as np

from urdf2dt.dh.types import JointType
from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.dynamics.model import DynamicModel, DynamicsConfig, JointFriction
from urdf2dt.dynamics.reference import MuJoCoReference
from urdf2dt.parser.urdf_input import URDFInput
from .data import IdentificationData, joint_units, write_record
from .regressor import InertialRegressor
from .fit import identify, evaluate
from .io import export_parameters


def excitation(model: DynamicModel, seed: int, samples: int = 240) -> tuple[np.ndarray, ...]:
    """Position-bounded multisine, exact derivatives; not a hardware motion command."""
    if samples < 80:
        raise ValueError("Use at least 80 time samples per trajectory")
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 12, samples)
    q, v, a = (np.zeros((samples, model.dof)) for _ in range(3))
    joints = [j for j in model.chain.joints if j.joint_type != JointType.FIXED]
    for i, joint in enumerate(joints):
        center, amplitude = (0., 1.) if joint.limit is None else (
            (joint.limit.lower+joint.limit.upper)/2, .38*(joint.limit.upper-joint.limit.lower))
        frequency = rng.uniform(.35, 1.7, 4)
        phase = rng.uniform(-np.pi, np.pi, 4)
        wave = t[:, None]*frequency+phase
        q[:, i] = center+amplitude*np.sin(wave).sum(axis=1)/4
        v[:, i] = amplitude*(np.cos(wave)*frequency).sum(axis=1)/4
        a[:, i] = -amplitude*(np.sin(wave)*frequency**2).sum(axis=1)/4
    return t, q, v, a


def synthetic_data(source: URDFInput, model: DynamicModel, seeds: tuple[int, ...],
                   label: str, samples: int = 240, noise_std: float = .001) -> IdentificationData:
    if not seeds or not np.isfinite(noise_std) or noise_std < 0:
        raise ValueError("Supply trajectory seeds and a nonnegative effort noise level")
    reference = MuJoCoReference(source, model)
    times: list[float] = []
    trajectories: list[str] = []
    positions: list[tuple[float, ...]] = []
    velocities: list[tuple[float, ...]] = []
    accelerations: list[tuple[float, ...]] = []
    efforts: list[tuple[float, ...]] = []
    for seed in seeds:
        t, q, v, a = excitation(model, seed, samples)
        noise = np.random.default_rng(seed+10000).normal(0, noise_std, q.shape)
        effort = np.array([reference.inverse(qi, vi, ai) for qi, vi, ai in zip(q, v, a)])+noise
        times.extend(t)
        trajectories.extend([f"{label}-{seed}"]*len(t))
        positions.extend(map(tuple, q))
        velocities.extend(map(tuple, v))
        accelerations.extend(map(tuple, a))
        efforts.extend(map(tuple, effort))
    return IdentificationData(str(model.chain.source_sha256), model.chain.joint_names, joint_units(model.chain),
        tuple(trajectories), tuple(times), tuple(positions), tuple(velocities), tuple(accelerations), tuple(efforts),
        "simulated", f"Analytic multisine derivatives; no filtering; synchronized; MuJoCo {version('mujoco')} inverse dynamics plus kinetic friction; independent Gaussian effort noise std={noise_std} in each joint effort SI unit; exact q/v/a; seeds={seeds}; samples={samples}; duration=12s",
        f"synthetic-{label}")


def run_benchmark(source_path: str | Path, output: str | Path, samples: int = 240) -> dict:
    source = URDFInput.from_path(source_path)
    nominal = load_dynamic_model(source)
    config = DynamicsConfig(friction=tuple(JointFriction(name, .12+.02*i, .06+.01*i)
                                           for i, name in enumerate(nominal.chain.joint_names)),
                            parameter_provenance="Synthetic benchmark truth: nominal URDF inertia and prescribed friction")
    truth = DynamicModel(nominal.chain, nominal.inertials, config)
    moving = set(InertialRegressor(truth.chain).links)
    markers = tuple(r.link_name for r in truth.inertials.records if r.link_name in moving
                    and r.properties is not None and r.properties.mass == 0)
    regressor = InertialRegressor(truth.chain, truth.config.gravity, markers)
    known = regressor.parameters(truth)
    baseline = known.copy()
    baseline[:10*len(regressor.links)] *= .7  # physically valid biased nominal baseline
    baseline[10*len(regressor.links):] = 0
    train = synthetic_data(source, truth, (2511, 2512, 2513), "train", samples)
    held = synthetic_data(source, truth, (2591, 2592), "held-out", samples)
    result = identify(regressor, train, prior=baseline)
    evaluation = evaluate(regressor, result, train, held, baseline)
    basis = np.asarray(result.report["identifiable_basis"])
    scale = np.asarray(result.report["parameter_scale"])
    actual_beta = basis@(known/scale)
    estimated_beta = basis@(np.asarray(result.parameters)/scale)
    relative = float(np.linalg.norm(estimated_beta-actual_beta)/max(np.linalg.norm(actual_beta), 1e-15))
    improved = bool(np.all(np.asarray(evaluation["fitted_rmse_per_joint"]) < np.asarray(evaluation["baseline_rmse_per_joint"])))
    if not improved or relative >= .01:
        raise ValueError(f"Benchmark acceptance failed: per-joint improvement={improved}, combination relative error={relative}")
    root = Path(__file__).resolve().parents[2]
    def git(*args: str) -> str:
        return subprocess.check_output(["git", *args], cwd=root, text=True).strip()
    report: dict[str, Any] = {
        "passed": True, "source_sha256": source.sha256,
        "training_sha256": train.digest, "held_out_sha256": held.digest,
        "identification": result.report, "held_out": evaluation,
        "truth_parameters": known.tolist(), "truth_beta": actual_beta.tolist(),
        "identifiable_combination_relative_error": relative,
        "acceptance": "Each joint held-out RMSE below deliberately biased baseline; identifiable combination relative error < 1%",
        "noise_std_per_joint": .001, "training_seeds": [2511, 2512, 2513], "held_out_seeds": [2591, 2592],
        "samples_per_trajectory": samples,
        "limitations": "Synthetic recovery only. Not hardware calibration. Rank is numerical, not a symbolic identifiability proof. Raw inertias are one physical representative.",
        "versions": {name: version(name) for name in ("numpy", "scipy", "casadi", "mujoco", "cvxpy", "clarabel")},
        "python": platform.python_version(), "source_commit": git("rev-parse", "HEAD"),
        "working_tree_dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
    }
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    train.save(destination/"training.json")
    held.save(destination/"held_out.json")
    export_parameters(destination/"parameters.json", source, regressor, result)
    write_record(destination/"report.json", "urdf2dt.identification_benchmark", report)
    return report
