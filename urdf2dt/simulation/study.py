"""Recorded closed-loop studies, step refinement and independent dynamics checks."""

from importlib.metadata import version
from pathlib import Path
from time import perf_counter
import platform
import subprocess
import numpy as np

from urdf2dt.control.experiments import scaled_inertias
from urdf2dt.control.io import save_controller
from urdf2dt.control import Controller
from urdf2dt.dynamics.io import load_dynamic_model, export_model
from urdf2dt.dynamics.reference import MuJoCoReference
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.trajectory.io import export_trajectory
from urdf2dt.identification.data import write_record, canonical
from hashlib import sha256
from .engine import simulate, SimulationConfig
from .metrics import tracking_metrics, compare_runs
from .protocol import scenario, assess_run


def _comparison_pass(comparison: dict, thresholds: dict) -> bool:
    mapping = {"max_position_difference": "position_max_difference",
               "max_velocity_difference": "velocity_max_difference",
               "max_acceleration_difference": "acceleration_max_difference",
               "max_effort_difference": "effort_max_difference"}
    return all(max(comparison[value]) <= thresholds[key] for key, value in mapping.items() if key in thresholds)


def run_study(robot: str, output: str | Path) -> dict:
    root = Path(__file__).resolve().parents[2]
    source = URDFInput.from_path(root/f"robots/dynamics/{robot}_dynamics.urdf")
    plant = load_dynamic_model(source)
    reference, configs, initial_q, initial_v, protocol = scenario(robot, plant)
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    # Freeze and write thresholds BEFORE any evaluation; failed runs leave evidence.
    write_record(destination/"protocol.json", "urdf2dt.simulation_protocol", protocol)
    export_model(plant, source, destination/"plant.json")
    export_trajectory(reference, source, destination/"reference.json", samples=401)
    started = perf_counter()
    cases, comparisons, failures = [], [], []
    nominal = {}
    for factor in protocol["controller_inertial_scales"]:
        controller_model = scaled_inertias(plant, factor)
        for config in configs:
            name = f"{config.mode}-scale-{factor:g}"
            save_controller(Controller(controller_model,reference,config), destination/f"{name}-config.json")
            try:
                run = simulate(plant,reference,controller_model,config,initial_q,initial_v,SimulationConfig(.002))
                metrics = tracking_metrics(run,plant,reference,move_end=protocol["move_end_s"],
                    steady_window=protocol["steady_window_s"],settling_band=protocol["settling_band"])
                assessment = assess_run(run,metrics,protocol,factor)
                write_record(destination/f"{name}.json", "urdf2dt.simulation_run", {"run": run, "metrics": metrics, "assessment": assessment})
                cases.append({"name": name, "mode": config.mode, "inertial_scale": factor,
                              "metrics": metrics, "assessment": assessment, "runtime_seconds": run["runtime_seconds"]})
                if factor == 1.:
                    nominal[config.mode] = run
            except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
                failures.append({"name": name, "error": str(exc)})
    independent = MuJoCoReference(source, plant)
    for config in configs:
        if config.mode not in nominal:
            continue
        controller_model = scaled_inertias(plant, 1.)
        coarse = nominal[config.mode]
        medium = simulate(plant,reference,controller_model,config,initial_q,initial_v,SimulationConfig(.001))
        fine = simulate(plant,reference,controller_model,config,initial_q,initial_v,SimulationConfig(.0005))
        other = simulate(plant,reference,controller_model,config,initial_q,initial_v,SimulationConfig(.002,"mujoco_rk4_reference"),source=source)
        refinement = [compare_runs(coarse,medium), compare_runs(medium,fine)]
        agreement = compare_runs(coarse,other)
        torque_errors = []
        for q,v,a,c in zip(coarse["position"],coarse["velocity"],coarse["acceleration"],coarse["commands"]):
            torque_errors.append(independent.inverse(np.asarray(q),np.asarray(v),np.asarray(a))-np.asarray(c["effort"]))
        torque_error = np.asarray(torque_errors)
        checks = {"refinement_within_tolerance": all(_comparison_pass(r,protocol["gates"]["convergence"]) for r in refinement),
                  "independent_dynamics_agree": _comparison_pass(agreement,protocol["gates"]["independent_dynamics"]),
                  "independent_inverse_effort_agrees": bool(np.max(np.abs(torque_error)) <= protocol["gates"]["independent_dynamics"]["max_effort_difference"])}
        comparison = {"mode": config.mode, "refinement_steps_s": [.002,.001,.0005], "refinement": refinement,
            "independent_dynamics": agreement, "independent_inverse_effort_rmse": np.sqrt(np.mean(torque_error**2,axis=0)).tolist(),
            "independent_inverse_effort_max_error": np.max(np.abs(torque_error),axis=0).tolist(),
            "checks": checks, "passed": all(checks.values()),
            "scope": "Same controller and RK4 algorithm; independent MuJoCo dynamics. Refinement varies integration timestep only; no hardware accuracy inference."}
        comparisons.append(comparison)
        for label, run in (("dt-0.001",medium),("dt-0.0005",fine),("mujoco",other)):
            write_record(destination/f"{config.mode}-{label}.json", "urdf2dt.simulation_comparison_run", {"run":run})
    def git(*args: str) -> str:
        return subprocess.check_output(["git",*args],cwd=root,text=True).strip()
    report = {"passed": not failures and len(cases)==9 and len(comparisons)==3
              and all(c["assessment"]["passed"] for c in cases) and all(c["passed"] for c in comparisons),
        "robot": robot, "source_sha256": source.sha256,
        "protocol_sha256": sha256(canonical(protocol)).hexdigest(), "cases":cases, "comparisons":comparisons,"failures":failures,
        "source_commit":git("rev-parse","HEAD"), "working_tree_dirty":bool(git("status","--porcelain","--untracked-files=no")),
        "versions":{n:version(n) for n in ("numpy","scipy","casadi","mujoco")},
        "python":platform.python_version(), "platform":platform.platform(), "processor":platform.processor(),
        "total_runtime_seconds":perf_counter()-started,
        "formal_advisor_acceptance":"Pending; user confirmed backend, numerical thresholds are recorded engineering criteria",
        "hardware_accuracy_validated":False}
    write_record(destination/"report.json", "urdf2dt.simulation_study", report)
    return report
