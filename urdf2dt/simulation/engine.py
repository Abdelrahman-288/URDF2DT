"""Deterministic RK4 with sampled controllers and independently selectable dynamics."""

from dataclasses import dataclass, asdict
from time import perf_counter
from typing import Any, Callable
import numpy as np

from urdf2dt.control import Controller, ControllerConfig
from urdf2dt.control.controller import model_digest
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.dynamics.reference import MuJoCoReference
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.trajectory import Trajectory
from urdf2dt.trajectory.core import position_bounds


class SimulationError(ValueError):
    """Fail without fabricating or silently clipping invalid physical states."""


@dataclass(frozen=True)
class SimulationConfig:
    step: float = .001
    backend: str = "python_rk4"
    max_steps: int = 1000000

    def __post_init__(self) -> None:
        if (not np.isfinite(self.step) or self.step <= 0
            or self.backend not in {"python_rk4", "mujoco_rk4_reference"}
            or type(self.max_steps) is not int or self.max_steps < 1):
            raise SimulationError("Require positive timestep/step budget and a supported dynamics backend")


def _ratio(value: float, unit: float, name: str) -> int:
    ratio = value/unit
    if not np.isfinite(ratio) or ratio < 1 or abs(ratio-round(ratio)) > 1e-8:
        raise SimulationError(f"{name} must be an integer multiple of its sampling interval")
    return int(round(ratio))


def rk4_step(acceleration: Callable, q: np.ndarray, v: np.ndarray,
             effort: np.ndarray, step: float) -> tuple[np.ndarray, np.ndarray]:
    """Held actuator effort across all four stages; evaluate dynamics at each state."""
    k1q, k1v = v, acceleration(q, v, effort)
    k2q = v+.5*step*k1v
    k2v = acceleration(q+.5*step*k1q, k2q, effort)
    k3q = v+.5*step*k2v
    k3v = acceleration(q+.5*step*k2q, k3q, effort)
    k4q = v+step*k3v
    k4v = acceleration(q+step*k3q, k4q, effort)
    return q+step*(k1q+2*k2q+2*k3q+k4q)/6, v+step*(k1v+2*k2v+2*k3v+k4v)/6


def simulate(plant: DynamicModel, reference: Trajectory, controller_model: DynamicModel,
             controller_config: ControllerConfig, initial_position: Any, initial_velocity: Any,
             config: SimulationConfig = SimulationConfig(), *, source: URDFInput | None = None) -> dict:
    """Return controller-rate state/effort logs; check every integration-step state.

    MuJoCo mode supplies independent forward accelerations to the same RK4 loop.
    This isolates dynamics disagreement; timestep refinement separately checks
    integration error. It is not a comparison of two independent integrators.
    """
    controller = Controller(controller_model, reference, controller_config)
    chain = reference.chain
    if (plant.chain.source_sha256 != chain.source_sha256 or plant.chain.joints != chain.joints
        or plant.chain.base_link != chain.base_link or plant.chain.tip_link != chain.tip_link):
        raise SimulationError("Plant and reference kinematics differ")
    substeps = _ratio(controller_config.period, config.step, "Controller period")
    ticks = _ratio(reference.duration, controller_config.period, "Reference duration")
    if ticks*substeps > config.max_steps:
        raise SimulationError("Simulation exceeds configured step budget")
    q, v = np.asarray(initial_position, dtype=float).copy(), np.asarray(initial_velocity, dtype=float).copy()
    lower, upper = position_bounds(chain)

    def check_state(time: float) -> None:
        if any(x.shape != (plant.dof,) or not np.isfinite(x).all() for x in (q, v)):
            raise SimulationError(f"Invalid/nonfinite state at {time:g}s")
        if np.any(q < lower-1e-9) or np.any(q > upper+1e-9):
            raise SimulationError(f"Joint position limit crossed at {time:g}s; no contact/limit solver is enabled")

    check_state(0.)
    started = perf_counter()
    acceleration: Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray]
    if config.backend == "mujoco_rk4_reference":
        if source is None:
            raise SimulationError("Independent dynamics require the exact source URDF snapshot")
        backend = MuJoCoReference(source, plant)
        acceleration = backend.forward
    else:
        acceleration = plant.forward_dynamics
    positions, velocities, accelerations, commands = [], [], [], []
    max_velocity = np.abs(v)
    for tick in range(ticks+1):
        time = min(tick*controller_config.period, reference.duration)
        command = controller.step(time, q, v)
        effort = np.asarray(command.effort)
        positions.append(q.tolist())
        velocities.append(v.tolist())
        accelerations.append(acceleration(q, v, effort).tolist())
        commands.append(asdict(command))
        if tick == ticks:
            break
        for k in range(substeps):
            q, v = rk4_step(acceleration, q, v, effort, config.step)
            max_velocity = np.maximum(max_velocity, np.abs(v))
            check_state(time+(k+1)*config.step)
    elapsed = perf_counter()-started
    return {"schema_version": "1.0", "source_sha256": chain.source_sha256,
        "plant_model_sha256": model_digest(plant), "controller": controller.snapshot(),
        "simulation": asdict(config), "integrator": "classical explicit fourth-order Runge-Kutta",
        "initial_position": np.asarray(initial_position).tolist(), "initial_velocity": np.asarray(initial_velocity).tolist(),
        "time": [c["time"] for c in commands], "position": positions, "velocity": velocities,
        "acceleration": accelerations, "commands": commands, "integration_steps": ticks*substeps,
        "maximum_integration_state_velocity": max_velocity.tolist(),
        "runtime_seconds": elapsed, "runtime_scope": "backend initialization, controller, RK4, state checks and logging; excludes metrics/export",
        "assumptions": ["fixed base, rigid serial chain, ideal joint effort actuators", "zero-order-held clipped effort",
                        "no contact, collision, joint-stop impulses, sensor noise or latency", "unwrapped continuous joints",
                        "deterministic; no random seed or measurement noise used"]}
