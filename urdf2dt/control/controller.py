"""PD, PD+gravity and computed-torque controllers with explicit sampled timing."""

from dataclasses import dataclass, asdict
from hashlib import sha256
from typing import Any
import numpy as np

from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.identification.data import canonical, joint_units
from urdf2dt.trajectory.core import Trajectory, position_bounds


class ControlError(ValueError):
    """Invalid inputs produce no command and do not consume a controller tick."""


@dataclass(frozen=True)
class ControllerConfig:
    """Diagonal gains and joint effort limits in the reference's joint order.

    PD gains: effort/position and effort/velocity. Computed-torque gains:
    s^-2 and s^-1. Period is seconds, effort is N*m (rotary) or N (linear).
    """

    mode: str
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    effort_limits: tuple[float, ...]
    period: float = .01

    def __post_init__(self) -> None:
        if self.mode not in {"pd", "pd_gravity", "computed_torque"}:
            raise ControlError("Choose pd, pd_gravity or computed_torque")
        for key in ("kp", "kd", "effort_limits"):
            values = np.asarray(getattr(self, key), dtype=float)
            if values.ndim != 1 or not len(values) or not np.isfinite(values).all() or np.any(values < 0):
                raise ControlError("Gains and limits must be finite nonnegative joint vectors")
            object.__setattr__(self, key, tuple(float(v) for v in values))
        if (len(self.kp) != len(self.kd) or len(self.kp) != len(self.effort_limits)
            or any(v <= 0 for v in self.effort_limits)
            or not np.isfinite(self.period) or self.period <= 0):
            raise ControlError("Matching gain/limit lengths and positive effort limits/period are required")


@dataclass(frozen=True)
class Command:
    time: float
    hold_until: float
    desired_position: tuple[float, ...]
    desired_velocity: tuple[float, ...]
    desired_acceleration: tuple[float, ...]
    position_error: tuple[float, ...]
    velocity_error: tuple[float, ...]
    unsaturated_effort: tuple[float, ...]
    effort: tuple[float, ...]
    saturated: tuple[bool, ...]


def model_digest(model: DynamicModel) -> str:
    return sha256(canonical({"inertials": asdict(model.inertials), "config": asdict(model.config)})).hexdigest()


class Controller:
    """Consume consecutive ticks starting at zero, with zero-order-held output.

    Continuous joints use the reference's unwrapped coordinates. No integral
    state, implicit angle wrapping, extrapolation, actuator I/O or automatic
    recovery after rejected input. reset() explicitly restarts reference time.
    """

    def __init__(self, model: DynamicModel, reference: Trajectory, config: ControllerConfig):
        chain = reference.chain
        if (chain.source_sha256 != model.chain.source_sha256 or chain.joints != model.chain.joints
            or chain.base_link != model.chain.base_link or chain.tip_link != model.chain.tip_link
            or len(config.kp) != model.dof):
            raise ControlError("Controller model, reference chain and gain dimensions must match")
        self._model, self._reference, self._config = model, reference, config
        limits = np.asarray(config.effort_limits)
        if reference.limits.effort is not None:
            limits = np.minimum(limits, reference.limits.effort)
        self._limits = tuple(float(x) for x in limits)
        self._tick = 0

    @property
    def config(self) -> ControllerConfig:
        return self._config

    @property
    def effective_effort_limits(self) -> tuple[float, ...]:
        return self._limits

    def reset(self) -> None:
        self._tick = 0

    def step(self, time: float, position: Any, velocity: Any) -> Command:
        expected = self._tick*self.config.period
        tolerance = max(1e-12, self.config.period*1e-6)
        if not np.isfinite(time) or abs(time-expected) > tolerance:
            raise ControlError(f"Expected tick time {expected:.12g}s; duplicate, skipped or jittered timestamp")
        if time < 0 or time > self._reference.duration:
            raise ControlError("Reference time is outside its duration; no extrapolation")
        q, v = np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)
        if any(x.shape != (self._model.dof,) or not np.isfinite(x).all() for x in (q, v)):
            raise ControlError("Measured position and velocity must be finite joint vectors")
        lo, hi = position_bounds(self._model.chain)
        if np.any(q < lo-1e-9) or np.any(q > hi+1e-9):
            raise ControlError("Measured position is outside joint limits; stop and review the state")
        desired, desired_v, desired_a = self._reference.evaluate(time)
        error, velocity_error = desired-q, desired_v-v
        correction = np.asarray(self.config.kp)*error + np.asarray(self.config.kd)*velocity_error
        if self.config.mode == "computed_torque":
            raw = self._model.inverse_dynamics(q, v, desired_a+correction)
        elif self.config.mode == "pd_gravity":
            raw = correction+self._model.gravity_effort(q)
        else:
            raw = correction
        if not np.isfinite(raw).all():
            raise ControlError("Nonfinite controller output; no command issued")
        limited = np.clip(raw, -np.asarray(self._limits), self._limits)
        command = Command(float(time), min(float(time+self.config.period), self._reference.duration),
            tuple(desired), tuple(desired_v), tuple(desired_a), tuple(error), tuple(velocity_error),
            tuple(raw), tuple(limited), tuple(bool(x) for x in (raw != limited)))
        self._tick += 1
        return command

    def snapshot(self) -> dict:
        return {"schema_version": "1.0", "source_sha256": self._model.chain.source_sha256,
                "joint_names": self._model.chain.joint_names, "model_sha256": model_digest(self._model),
                "position_units": joint_units(self._model.chain),
                "effort_units": tuple("N" if u == "m" else "N*m" for u in joint_units(self._model.chain)),
                "parameter_provenance": self._model.config.parameter_provenance,
                "config": asdict(self.config), "effective_effort_limits": self._limits,
                "gain_units": "kp=s^-2; kd=s^-1" if self.config.mode == "computed_torque"
                              else "kp=joint effort/position; kd=joint effort/velocity",
                "timing": "Consecutive fixed-period ticks starting at zero; zero-order hold; no extrapolation",
                "continuous_joints": "Unwrapped coordinates supplied by the reference and measurement",
                "reference_sha256": sha256(canonical({"times": self._reference.times,
                    "coefficients": self._reference.coefficients})).hexdigest()}
