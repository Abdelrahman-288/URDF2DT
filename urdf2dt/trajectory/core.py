"""Piecewise polynomials with analytic derivatives and interval limit checks."""

from dataclasses import dataclass
from typing import Any
import numpy as np
from numpy.polynomial import Polynomial

from urdf2dt.dh.types import KinematicChain, JointType
from urdf2dt.kinematics import urdf_fk


class TrajectoryError(ValueError):
    """Actionable failure, before a reference is returned."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(f"{code}: {message}")


@dataclass(frozen=True)
class MotionLimits:
    """Explicit SI maxima, in chain joint order; accelerations are not in URDF."""

    velocity: tuple[float, ...]
    acceleration: tuple[float, ...]
    effort: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        for name in ("velocity", "acceleration", "effort"):
            value = getattr(self, name)
            if value is None and name == "effort":
                continue
            values = np.asarray(value, dtype=float)
            if values.ndim != 1 or not len(values) or not np.isfinite(values).all() or np.any(values <= 0):  # type: ignore[operator]
                raise TrajectoryError("invalid_limits", f"{name} must contain finite positive maxima")
            object.__setattr__(self, name, tuple(float(v) for v in values))
        if len(self.velocity) != len(self.acceleration) or (self.effort is not None and len(self.effort) != len(self.velocity)):
            raise TrajectoryError("invalid_limits", "Limit vectors must have equal length")


def position_bounds(chain: KinematicChain) -> tuple[np.ndarray, np.ndarray]:
    joints = [j for j in chain.joints if j.joint_type != JointType.FIXED]
    return (np.array([j.limit.lower if j.limit else -np.inf for j in joints]),
            np.array([j.limit.upper if j.limit else np.inf for j in joints]))


def extrema(poly: Polynomial) -> tuple[float, float]:
    """Numerical polynomial extrema on [0,1], including both ends."""
    roots = poly.deriv().roots()
    candidates = [0., 1.] + [float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < 1]  # type: ignore[operator]
    values = np.asarray(poly(candidates), dtype=float)
    if not np.isfinite(values).all():
        raise TrajectoryError("numerical_failure", "Nonfinite polynomial extrema")
    return float(np.min(values)), float(np.max(values))


def time_law(method: str) -> Polynomial:
    if method == "quintic":
        return Polynomial([0., 0., 0., 10., -15., 6.])
    if method == "cubic":
        return Polynomial([0., 0., 3., -2.])
    raise TrajectoryError("invalid_method", "Choose cubic or quintic time parameterization")


@dataclass(frozen=True, init=False)
class Trajectory:
    """Owned local-u polynomials. q(t), qdot(t), qddot(t) are analytic.

    Coefficients ascend in u=(t-times[k])/(times[k+1]-times[k]). No extrapolation
    or implicit wrapping of continuous joints. Metadata is copied on access.
    """

    chain: KinematicChain
    limits: MotionLimits
    times: tuple[float, ...]
    coefficients: tuple[tuple[tuple[float, ...], ...], ...]
    _metadata: dict
    _bounds: dict

    def __init__(self, chain: KinematicChain, times: Any, coefficients: Any,
                 limits: MotionLimits, metadata: dict | None = None):
        import copy
        t, c = np.asarray(times, dtype=float), np.asarray(coefficients, dtype=float)
        n = len(chain.joint_names)
        if (not chain.source_sha256 or not n or len(limits.velocity) != n
            or t.ndim != 1 or len(t) < 2 or t[0] != 0 or not np.isfinite(t).all()
            or np.any(np.diff(t) <= 0) or c.ndim != 3 or c.shape[:2] != (len(t)-1, n)
            or not 2 <= c.shape[2] <= 16 or not np.isfinite(c).all()):
            raise TrajectoryError("invalid_trajectory", "Require source-bound chain, increasing times and finite polynomial coefficients")
        object.__setattr__(self, "chain", chain)
        object.__setattr__(self, "limits", limits)
        object.__setattr__(self, "times", tuple(float(v) for v in t))
        object.__setattr__(self, "coefficients", tuple(tuple(tuple(float(x) for x in row) for row in segment) for segment in c))
        object.__setattr__(self, "_metadata", copy.deepcopy(metadata or {}))
        self._check_continuity()
        object.__setattr__(self, "_bounds", self._check_bounds())

    @property
    def metadata(self) -> dict:
        import copy
        return copy.deepcopy(self._metadata)

    @property
    def duration(self) -> float:
        return self.times[-1]

    def segment(self, index: int, u: float, derivative: int = 0) -> np.ndarray:
        dt = self.times[index+1]-self.times[index]
        return np.array([Polynomial(row).deriv(derivative)(u)/dt**derivative for row in self.coefficients[index]])

    def evaluate(self, time: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not np.isfinite(time) or time < 0 or time > self.duration:
            raise TrajectoryError("time_out_of_range", f"Evaluate inside [0,{self.duration:g}] seconds")
        index = min(int(np.searchsorted(self.times, time, side="right"))-1, len(self.times)-2)
        u = (time-self.times[index])/(self.times[index+1]-self.times[index])
        return self.segment(index, u), self.segment(index, u, 1), self.segment(index, u, 2)

    def sample(self, count: int = 201) -> dict[str, Any]:
        if type(count) is not int or not 2 <= count <= 100000:
            raise TrajectoryError("invalid_samples", "Choose 2 to 100000 samples")
        # Always include boundaries so narrow segments and exact endpoints survive export.
        times = np.unique(np.r_[np.linspace(0, self.duration, count), self.times])
        values = [self.evaluate(float(t)) for t in times]
        result = {"time": times.tolist(), "q": [v[0].tolist() for v in values],
                "velocity": [v[1].tolist() for v in values], "acceleration": [v[2].tolist() for v in values],
                "tip_pose": [np.asarray(urdf_fk(self.chain, v[0].tolist())).tolist() for v in values]}  # type: ignore[arg-type]
        if self._metadata.get("kind") == "cartesian_line":
            from scipy.spatial.transform import Rotation
            start, target = np.asarray(self._metadata["start_pose"]), np.asarray(self._metadata["target_pose"])
            fraction = time_law("quintic")(times/self.duration)
            result["path_fraction"] = fraction.tolist()
            result["desired_tip_position"] = (start[:3, 3]+fraction[:, None]*(target[:3, 3]-start[:3, 3])).tolist()
            if self._metadata["task"] == "pose":
                rotation = Rotation.from_matrix(target[:3, :3]@start[:3, :3].T).as_rotvec()
                result["desired_tip_rotation"] = [(Rotation.from_rotvec(s*rotation).as_matrix()@start[:3, :3]).tolist() for s in fraction]
        return result

    def _check_continuity(self) -> None:
        # C2 interior continuity is mandatory. A cubic point-to-point segment has
        # nonzero endpoint acceleration, recorded explicitly by the generator.
        for i in range(len(self.times)-2):
            for d in range(3):
                if not np.allclose(self.segment(i, 1, d), self.segment(i+1, 0, d), rtol=1e-7, atol=1e-7):
                    raise TrajectoryError("discontinuity", f"Derivative order {d} jumps at waypoint {i+1}; use quintic stops")

    def _check_bounds(self) -> dict:
        lower, upper = position_bounds(self.chain)
        qmin, qmax = np.full(len(lower), np.inf), np.full(len(lower), -np.inf)
        vmax, amax = np.zeros(len(lower)), np.zeros(len(lower))
        for k, segment in enumerate(self.coefficients):
            dt = self.times[k+1]-self.times[k]
            for j, row in enumerate(segment):
                p = Polynomial(row)
                lo, hi = extrema(p)
                qmin[j], qmax[j] = min(qmin[j], lo), max(qmax[j], hi)
                vmax[j] = max(vmax[j], max(abs(x) for x in extrema(p.deriv()))/dt)
                amax[j] = max(amax[j], max(abs(x) for x in extrema(p.deriv(2)))/dt**2)
        for name, values, limit in (("position_lower", lower, qmin), ("position_upper", qmax, upper),
                                    ("velocity", vmax, self.limits.velocity), ("acceleration", amax, self.limits.acceleration)):
            violation = np.asarray(values) > np.asarray(limit)+1e-8
            if violation.any():
                j = int(np.flatnonzero(violation)[0])
                raise TrajectoryError("limit_violation", f"{name} exceeded for {self.chain.joint_names[j]}; increase duration or revise path")
        return {"position_min": qmin.tolist(), "position_max": qmax.tolist(),
                "velocity_max": vmax.tolist(), "acceleration_max": amax.tolist(),
                "method": "Polynomial interval extrema via numerical derivative roots; not just output samples"}

    @property
    def bounds(self) -> dict:
        import copy
        return copy.deepcopy(self._bounds)


def joint_trajectory(chain: KinematicChain, waypoints: Any, limits: MotionLimits, *,
                     method: str = "quintic", durations: Any = None) -> Trajectory:
    points = np.asarray(waypoints, dtype=float)
    law = time_law(method)
    if (points.ndim != 2 or points.shape[1] != len(chain.joint_names) or len(points) < 2
        or not np.isfinite(points).all() or len(limits.velocity) != points.shape[1]):
        raise TrajectoryError("invalid_waypoints", "Supply at least two finite configurations in joint order")
    if method == "cubic" and len(points) > 2:
        raise TrajectoryError("discontinuity", "Cubic stops can jump acceleration; use quintic for multiple waypoints")
    peak_v = max(abs(x) for x in extrema(law.deriv()))
    peak_a = max(abs(x) for x in extrema(law.deriv(2)))
    delta = np.diff(points, axis=0)
    minimum = np.maximum(np.max(np.abs(delta)*peak_v/limits.velocity, axis=1),
                         np.sqrt(np.max(np.abs(delta)*peak_a/limits.acceleration, axis=1)))
    dt = np.maximum(minimum*1.000001, .001) if durations is None else np.asarray(durations, dtype=float)
    if dt.shape != (len(delta),) or not np.isfinite(dt).all() or np.any(dt <= 0):
        raise TrajectoryError("invalid_duration", "Supply one positive finite duration per segment")
    if np.any(dt+1e-10 < minimum):
        raise TrajectoryError("duration_too_short", f"Segment durations must be at least {minimum.tolist()} seconds")
    coefficients = np.zeros((len(delta), points.shape[1], len(law.coef)))
    coefficients[:, :, :] = delta[:, :, None]*law.coef
    coefficients[:, :, 0] += points[:-1]
    return Trajectory(chain, np.r_[0., np.cumsum(dt)], coefficients, limits, {
        "kind": "joint_waypoints", "method": method, "waypoints": points.tolist(),
        "minimum_segment_durations": minimum.tolist(), "continuous_joints": "Explicit unwrapped coordinates; no automatic shortest-path choice",
        "endpoint_acceleration": "zero; C2 with stationary holds" if method == "quintic" else "nonzero one-sided acceleration; do not concatenate to stationary holds as C2 motion"})
