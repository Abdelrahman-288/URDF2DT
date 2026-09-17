"""Reproducible controller scenarios for Stage 28, with instantaneous diagnostics."""

from dataclasses import asdict, replace
from pathlib import Path
from importlib.metadata import version
import subprocess
import platform
import numpy as np

from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.dynamics.io import load_dynamic_model, export_model
from urdf2dt.parser.inertial_extractor import InertialModel
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.trajectory import MotionLimits, joint_trajectory
from urdf2dt.trajectory.io import export_trajectory
from urdf2dt.identification.data import write_record
from .controller import Controller, ControllerConfig, model_digest
from .io import save_controller


def scaled_inertias(model: DynamicModel, factor: float) -> DynamicModel:
    """Physically valid mass/tensor scaling at unchanged COMs and kinematics."""
    if not np.isfinite(factor) or factor <= 0:
        raise ValueError("Inertial scale must be finite and positive")
    records = []
    for record in model.inertials.records:
        body = record.properties
        if body is not None:
            body = replace(body, mass=body.mass*factor,
                           tensor=tuple(tuple(x*factor for x in row) for row in body.tensor))
            record = replace(record, properties=body)
        records.append(record)
    config = replace(model.config, parameter_provenance=f"Sensitivity scenario: inertias scaled by {factor:g}; {model.config.parameter_provenance}")
    return DynamicModel(model.chain, InertialModel(model.inertials.source_sha256, tuple(records)), config)


def run_experiment(robot: str, output: str | Path) -> dict:
    if robot not in {"scara", "ur5"}:
        raise ValueError("Choose scara or ur5")
    root = Path(__file__).resolve().parents[2]
    source = URDFInput.from_path(root/f"robots/dynamics/{robot}_dynamics.urdf")
    plant = load_dynamic_model(source)
    pd_kp: tuple[float, ...]
    pd_kd: tuple[float, ...]
    if robot == "scara":
        start = np.array([.3, .8, .06, .2])
        goal = start + [.04, -.03, .015, .01]
        limits = MotionLimits((1., 1., .15, 1.), (2., 2., .3, 2.), (30., 30., 100., 10.))
        offset = np.array([.02, -.01, .002, .01])
        pd_kp, pd_kd = (30., 30., 150., 10.), (6., 6., 20., 2.)
    else:
        start = np.array([.3, -1., 1.2, -1., .8, .2])
        goal = start + [.03, -.02, .03, .01, -.02, .01]
        limits = MotionLimits((1.,)*6, (2.,)*6, (150., 150., 150., 28., 28., 28.))
        offset = np.array([.02, -.01, .01, -.02, .01, -.01])
        pd_kp, pd_kd = (30.,)*6, (6.,)*6
    reference = joint_trajectory(plant.chain, [start, goal], limits, durations=[2.])
    period = .01
    assert limits.effort is not None  # Both explicit demo policies above include effort limits.
    configs = [ControllerConfig("pd", pd_kp, pd_kd, limits.effort, period),
               ControllerConfig("pd_gravity", pd_kp, pd_kd, limits.effort, period),
               ControllerConfig("computed_torque", (25.,)*plant.dof, (10.,)*plant.dof, limits.effort, period)]
    cases = []
    configurations = []
    for factor in (.8, 1., 1.2):
        controller_model = scaled_inertias(plant, factor)
        for config in configs:
            controller = Controller(controller_model, reference, config)
            configurations.append((f"{config.mode}-scale-{factor:g}.json", controller))
            commands, acceleration_errors = [], []
            for tick in range(201):
                time = tick*period
                qd, vd, ad = reference.evaluate(time)
                measured_q, measured_v = qd+offset, vd+offset*.5
                command = controller.step(time, measured_q, measured_v)
                acceleration = plant.forward_dynamics(measured_q, measured_v, command.effort)
                commands.append(asdict(command))
                acceleration_errors.append(acceleration-ad)
            cases.append({"inertial_scale": factor, "controller": controller.snapshot(),
                "acceleration_error_rmse_per_joint": np.sqrt(np.mean(np.asarray(acceleration_errors)**2, axis=0)).tolist(),
                "saturated_ticks_per_joint": np.sum([c["saturated"] for c in commands], axis=0).tolist(),
                "commands": commands})
    report = {"robot": robot, "source_sha256": source.sha256, "plant_model_sha256": model_digest(plant),
        "period_s": period, "duration_s": reference.duration, "position_offset": offset.tolist(),
        "velocity_offset": (offset*.5).tolist(), "cases": cases,
        "scope": "Instantaneous prescribed-state sensitivity sweep, not closed-loop simulation or measured tracking accuracy",
        "gain_provenance": "Explicit demonstration gains, not hardware-tuned; CT gains have different units from PD gains",
        "stage28_protocol": {"controllers": [asdict(c) for c in configs], "controller_inertial_scales": [.8, 1., 1.2],
            "plant": "Keep nominal plant fixed; perturb only controller model for fair sensitivity comparisons",
            "initial_position": (start+offset).tolist(), "initial_velocity": (offset*.5).tolist(),
            "required_metrics": ["position/velocity tracking RMSE and max per joint", "effort and saturation fraction", "integration timestep convergence"],
            "acceptance": "Configure and record Stage 28 scenario-specific tracking thresholds before simulation; no stability claim from these instantaneous checks"},
        "python": platform.python_version(), "versions": {n: version(n) for n in ("numpy", "scipy", "casadi")},
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "working_tree_dirty": bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root, text=True).strip())}
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    export_model(plant, source, destination/"plant.json")
    export_trajectory(reference, source, destination/"reference.json", samples=201)
    for name, controller in configurations:
        save_controller(controller, destination/name)
    write_record(destination/"controllers.json", "urdf2dt.controller_study", report)
    return report
