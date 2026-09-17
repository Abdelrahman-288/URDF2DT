"""Model-associated sampled inverse-dynamics feasibility, separate from geometry."""

from dataclasses import asdict
from hashlib import sha256
import numpy as np
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.identification.data import canonical
from .core import Trajectory, TrajectoryError


def dynamic_feasibility(trajectory: Trajectory, model: DynamicModel, samples: int = 501) -> dict:
    chain = trajectory.chain
    if (chain.source_sha256 != model.chain.source_sha256 or chain.joints != model.chain.joints
        or chain.base_link != model.chain.base_link or chain.tip_link != model.chain.tip_link):
        raise TrajectoryError("model_mismatch", "Dynamics and reference must share the exact source chain")
    data = trajectory.sample(samples)
    effort = np.asarray([model.inverse_dynamics(q, v, a) for q, v, a in
                         zip(data["q"], data["velocity"], data["acceleration"])])
    if not np.isfinite(effort).all():
        raise TrajectoryError("dynamics_failure", "Nonfinite inverse-dynamics effort")
    maxima = np.max(np.abs(effort), axis=0)
    limits = trajectory.limits.effort
    if limits is not None and np.any(maxima > np.asarray(limits)+1e-8):
        j = int(np.argmax(maxima/limits))
        raise TrajectoryError("effort_limit", f"{chain.joint_names[j]} needs up to {maxima[j]:.6g}, limit {limits[j]:.6g}; revise path/timing/payload (gravity cannot be fixed by slowing)")
    return {"source_sha256": chain.source_sha256,
            "dynamic_model_sha256": sha256(canonical({"inertials": asdict(model.inertials), "config": asdict(model.config)})).hexdigest(),
            "parameter_provenance": model.config.parameter_provenance, "sample_count": len(data["time"]),
            "effort_max": maxima.tolist(), "effort_limits": limits, "effort_limits_checked": limits is not None,
            "time": data["time"], "feedforward_effort": effort.tolist(),
            "scope": "Sampled inverse-dynamics feasibility; no contact, tracking, controller saturation or continuous effort certificate"}
