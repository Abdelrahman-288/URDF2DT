"""Tracking, effort and point-to-point hold metrics with explicit units/windows."""

from typing import Any
import numpy as np
from scipy.spatial.transform import Rotation
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.trajectory import Trajectory
from urdf2dt.identification.data import joint_units


def tracking_metrics(run: dict, model: DynamicModel, reference: Trajectory, *,
                     move_end: float, steady_window: float, settling_band: Any) -> dict:
    times = np.asarray(run["time"])
    q, v = np.asarray(run["position"]), np.asarray(run["velocity"])
    band = np.asarray(settling_band, dtype=float)
    n = model.dof
    if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
        or q.shape != (len(times), n) or v.shape != q.shape or not np.isfinite(q).all() or not np.isfinite(v).all()
        or run["source_sha256"] != model.chain.source_sha256
        or reference.chain.source_sha256 != model.chain.source_sha256):
        raise ValueError("Metrics require finite synchronized states and matching source identity")
    period = run["controller"]["config"]["period"]
    if (abs(times[0]) > 1e-12 or abs(times[-1]-reference.duration) > 1e-10
        or not np.allclose(np.diff(times), period, atol=1e-10, rtol=0)
        or len(run["commands"]) != len(times)):
        raise ValueError("Metrics require a complete, uniform controller-rate run")
    if (band.shape != (n,) or not np.isfinite(band).all() or np.any(band <= 0)
        or not 0 < move_end < reference.duration or not 0 < steady_window <= reference.duration-move_end):
        raise ValueError("Require positive per-joint settling bands and a terminal hold/steady-state window")
    desired = np.asarray([reference.evaluate(float(t))[0] for t in times])
    desired_v = np.asarray([reference.evaluate(float(t))[1] for t in times])
    error, velocity_error = q-desired, v-desired_v
    poses = np.asarray([model.tip_transform(state) for state in q])
    targets = np.asarray([model.tip_transform(state) for state in desired])
    position_error = np.linalg.norm(poses[:, :3, 3]-targets[:, :3, 3], axis=1)
    angular_error = np.array([Rotation.from_matrix(a[:3, :3]@b[:3, :3].T).magnitude() for a,b in zip(poses,targets)])
    target = reference.evaluate(reference.duration)[0]
    initial_target = reference.evaluate(0)[0]
    displacement = target-initial_target
    hold = times >= move_end-1e-12
    steady = times >= times[-1]-steady_window-1e-12
    # Require the entire specified terminal interval to be a stationary reference.
    for t in times[hold]:
        qr, vr, ar = reference.evaluate(float(t))
        if not (np.allclose(qr, target, atol=1e-10, rtol=0) and np.allclose(vr, 0, atol=1e-9, rtol=0)
                and np.allclose(ar, 0, atol=1e-8, rtol=0)):
            raise ValueError("Settling/steady-state metrics require an actual terminal stationary hold")
    settled = []
    for j in range(n):
        outside = np.flatnonzero(np.abs(q[hold, j]-target[j]) > band[j])
        first = 0 if not len(outside) else int(outside[-1]+1)
        hold_times = times[hold]
        settled.append(None if first >= len(hold_times) or hold_times[-1]-hold_times[first] < steady_window-1e-12
                       else float(hold_times[first]-move_end))
    overshoot: list[float | None] = []
    percent: list[float | None] = []
    for j, delta in enumerate(displacement):
        if abs(delta) < 1e-12:
            overshoot.append(None)
            percent.append(None)
        else:
            excess = max(0., float(np.max(np.sign(delta)*(q[:,j]-target[j]))))
            overshoot.append(excess)
            percent.append(100*excess/abs(float(delta)))
    # Last command has no hold interval; exclude it from effort integrals/fractions.
    effort = np.asarray([c["effort"] for c in run["commands"][:-1]])
    saturated = np.asarray([c["saturated"] for c in run["commands"][:-1]])
    period = run["controller"]["config"]["period"]
    return {"position_units": joint_units(model.chain), "effort_units": run["controller"]["effort_units"],
        "joint_position_rmse": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "joint_position_max_error": np.max(np.abs(error), axis=0).tolist(),
        "joint_velocity_rmse": np.sqrt(np.mean(velocity_error**2, axis=0)).tolist(),
        "joint_velocity_max_error": np.max(np.abs(velocity_error), axis=0).tolist(),
        "cartesian_position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "cartesian_position_max_error_m": float(np.max(position_error)),
        "orientation_rmse_rad": float(np.sqrt(np.mean(angular_error**2))),
        "orientation_max_error_rad": float(np.max(angular_error)),
        "steady_state_signed_mean_error": np.mean(error[steady], axis=0).tolist(),
        "steady_state_rmse": np.sqrt(np.mean(error[steady]**2, axis=0)).tolist(),
        "settling_time_after_move_s": settled, "settling_band": band.tolist(),
        "settling_definition": "First controller sample after move_end followed only by samples within the position band to run end, with at least steady_window seconds observed; finite horizon",
        "overshoot": overshoot, "overshoot_percent": percent,
        "overshoot_definition": "Peak beyond final reference in move direction, divided by reference displacement; zero-displacement joints are null",
        "effort_rms": np.sqrt(np.mean(effort**2, axis=0)).tolist(), "effort_peak": np.max(np.abs(effort), axis=0).tolist(),
        "effort_total_variation": np.sum(np.abs(np.diff(effort, axis=0)), axis=0).tolist(),
        "saturation_fraction": np.mean(saturated, axis=0).tolist(),
        "saturation_duration_s": (np.sum(saturated, axis=0)*period).tolist(),
        "move_end_s": move_end, "steady_state_window_s": [float(times[-1]-steady_window), float(times[-1])],
        "sample_scope": "Uniform controller-rate samples; per-joint quantities retain rad/m and are not pooled across unlike units"}


def compare_runs(first: dict, second: dict) -> dict:
    if first["time"] != second["time"] or first["source_sha256"] != second["source_sha256"]:
        raise ValueError("Comparison requires matching timestamps and source chain")
    result = {}
    for key in ("position", "velocity", "acceleration"):
        error = np.asarray(first[key])-np.asarray(second[key])
        result[key+"_rmse"] = np.sqrt(np.mean(error**2, axis=0)).tolist()
        result[key+"_max_difference"] = np.max(np.abs(error), axis=0).tolist()
    effort_error = np.asarray([c["effort"] for c in first["commands"]])-np.asarray([c["effort"] for c in second["commands"]])
    result["effort_rmse"] = np.sqrt(np.mean(effort_error**2, axis=0)).tolist()
    result["effort_max_difference"] = np.max(np.abs(effort_error), axis=0).tolist()
    return result
