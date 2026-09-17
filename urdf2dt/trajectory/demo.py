"""Reproducible UR5/SCARA reference-generation evidence, without robot control."""

from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path
import platform
import subprocess
import numpy as np

from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.identification.io import load_parameters
from urdf2dt.identification.data import write_record
from urdf2dt.parser.urdf_input import URDFInput
from .core import MotionLimits, joint_trajectory
from .cartesian import cartesian_line
from .io import export_trajectory
from .validation import dynamic_feasibility


def run_demo(robot: str, output: str | Path, identified_parameters: str | Path | None = None) -> dict:
    if robot not in {"ur5", "scara"}:
        raise ValueError("Demo robot must be ur5 or scara")
    root = Path(__file__).resolve().parents[2]
    source = URDFInput.from_path(root/f"robots/dynamics/{robot}_dynamics.urdf")
    model = load_dynamic_model(source) if identified_parameters is None else load_parameters(identified_parameters)[0]
    if model.chain.source_sha256 != source.sha256:
        raise ValueError("Identified parameters do not match the demo robot source")
    if robot == "ur5":
        start = np.array([.3, -1., 1.2, -1., .8, .2])
        delta = np.array([.03, -.02, .03, .01, -.02, .01])
        limits = MotionLimits((1.,)*6, (2.,)*6, (150., 150., 150., 28., 28., 28.))
        task = "pose"
    else:
        start = np.array([.3, .8, .06, .2])
        delta = np.array([.04, -.03, .015, .0])
        limits = MotionLimits((1., 1., .15, 1.), (2., 2., .3, 2.), (30., 30., 100., 10.))
        task = "position"
    joint = joint_trajectory(model.chain, [start, start+delta*2, start+delta], limits)
    cubic = joint_trajectory(model.chain, [start, start+delta], limits, method="cubic")
    cartesian = cartesian_line(model.chain, start, model.tip_transform(start+delta), limits, task=task, knots=33)
    trajectories = {"joint_quintic": joint, "joint_cubic": cubic, "cartesian": cartesian}
    checks = {name: dynamic_feasibility(t, model, samples=301) for name, t in trajectories.items()}
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    for name, trajectory in trajectories.items():
        export_trajectory(trajectory, source, destination/f"{name}.json", samples=201)
    def git(*args: str) -> str:
        return subprocess.check_output(["git", *args], cwd=root, text=True).strip()
    report = {"passed": True, "robot": robot, "source_sha256": source.sha256,
        "source_commit": git("rev-parse", "HEAD"), "working_tree_dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
        "python": platform.python_version(), "versions": {n: version(n) for n in ("numpy", "scipy", "casadi")},
        "limits": asdict(limits), "limit_provenance": "Explicit demonstration limits, not manufacturer-certified acceleration/control limits",
        "dynamic_parameters": "nominal URDF" if identified_parameters is None else "Stage 25 identified archive",
        "trajectories": {name: {"duration_s": t.duration, "bounds": t.bounds, "metadata": t.metadata,
                                 "dynamics": checks[name]} for name, t in trajectories.items()},
        "scope": "Controller references only; no controller/simulation/hardware execution, no collision check"}
    write_record(destination/"report.json", "urdf2dt.trajectory_demo", report)
    return report
