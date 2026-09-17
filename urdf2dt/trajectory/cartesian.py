"""Straight end-effector paths, seeded bounded IK, and smooth time scaling."""

from importlib import import_module
from typing import Any
import numpy as np
from numpy.polynomial import Polynomial
from scipy.spatial.transform import Rotation

from urdf2dt.dh.types import KinematicChain, JointType, _transform
from urdf2dt.kinematics import urdf_link_transforms, urdf_fk
from .core import MotionLimits, Trajectory, TrajectoryError, position_bounds, time_law, extrema


def geometric_jacobian(chain: KinematicChain, q: Any) -> np.ndarray:
    poses = urdf_link_transforms(chain, q.tolist())  # type: ignore[arg-type]
    tip = np.asarray(poses[-1])[:3, 3]
    columns = []
    for index, joint in enumerate(chain.joints):
        if joint.joint_type == JointType.FIXED:
            continue
        frame = np.asarray(poses[index]) @ np.asarray(joint.origin)
        axis = frame[:3, :3]@joint.axis
        columns.append(np.r_[axis, np.zeros(3)] if joint.joint_type == JointType.PRISMATIC
                       else np.r_[np.cross(axis, tip-frame[:3, 3]), axis])
    return np.column_stack(columns)


def cartesian_line(chain: KinematicChain, start_q: Any, target_pose: Any, limits: MotionLimits, *,
                   task: str = "position", knots: int = 33, duration: float | None = None,
                   position_tolerance: float = 1e-5, orientation_tolerance: float = 1e-4,
                   singular_value_min: float = 1e-4, max_normalized_step: float = .25) -> Trajectory:
    """Straight base-frame translation; optional shortest SO(3) rotation (pose task).

    Position-only tasks leave orientation unconstrained (e.g. SCARA). Local IK
    failure is not proof of global unreachability. No collision/obstacle planning.
    FK accuracy and singularity checks use 8 subdivisions per IK interval.
    """
    n = len(chain.joint_names)
    q0 = np.asarray(start_q, dtype=float)
    if task not in {"position", "pose"} or type(knots) is not int or not 5 <= knots <= 257:
        raise TrajectoryError("invalid_cartesian_request", "Choose position/pose task and 5–257 IK knots")
    if (q0.shape != (n,) or not np.isfinite(q0).all() or len(limits.velocity) != n
        or any(not np.isfinite(x) or x <= 0 for x in (position_tolerance, orientation_tolerance, singular_value_min, max_normalized_step))):
        raise TrajectoryError("invalid_cartesian_request", "Supply finite joint seed, matching limits and positive tolerances")
    lower, upper = position_bounds(chain)
    if np.any(q0 < lower) or np.any(q0 > upper) or np.any(lower == upper):
        raise TrajectoryError("joint_limits", "Seed is outside limits or a movable joint has zero range")
    target = np.asarray(_transform(target_pose))
    start = np.asarray(urdf_fk(chain, q0.tolist()))  # type: ignore[arg-type]
    rotvec = Rotation.from_matrix(target[:3, :3]@start[:3, :3].T).as_rotvec()
    # At pi the shortest SO(3) path has an ambiguous axis sign.
    if task == "pose" and abs(np.linalg.norm(rotvec)-np.pi) < 1e-6:
        raise TrajectoryError("ambiguous_rotation", "Split a 180-degree orientation change into smaller requests")
    coordinate_scale = np.array([.1 if j.joint_type == JointType.PRISMATIC else 1.
                                 for j in chain.joints if j.joint_type != JointType.FIXED])

    def desired(s: float) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = start[:3, 3]+s*(target[:3, 3]-start[:3, 3])
        pose[:3, :3] = Rotation.from_rotvec(s*rotvec).as_matrix()@start[:3, :3]
        return pose

    def error(q: np.ndarray, s: float) -> tuple[np.ndarray, np.ndarray]:
        pose, wanted = np.asarray(urdf_fk(chain, q.tolist())), desired(s)  # type: ignore[arg-type]
        return pose[:3, 3]-wanted[:3, 3], Rotation.from_matrix(pose[:3, :3]@wanted[:3, :3].T).as_rotvec()

    def singularity(q: np.ndarray) -> float:
        jac = geometric_jacobian(chain, q)*coordinate_scale
        jac = jac[:3] if task == "position" else np.vstack([jac[:3], .1*jac[3:]])
        singular = np.linalg.svd(jac, compute_uv=False)
        if jac.shape[0] > jac.shape[1] or singular[-1] < singular_value_min:
            raise TrajectoryError("singularity", "Task Jacobian lacks usable rank; change seed/path/task or singularity threshold with justification")
        return float(singular[-1])

    minimum_singular = singularity(q0)
    optimize, interpolate = import_module("scipy.optimize"), import_module("scipy.interpolate")
    grid, configurations = np.linspace(0, 1, knots), [q0]
    for s in grid[1:]:
        previous = configurations[-1]
        lo, hi = lower.copy(), upper.copy()
        for j in range(n):
            if not np.isfinite(lo[j]):
                lo[j], hi[j] = previous[j]-np.pi, previous[j]+np.pi
        def residual(q: np.ndarray) -> np.ndarray:
            position, rotation = error(q, float(s))
            return position if task == "position" else np.r_[position, .1*rotation]
        solution = optimize.least_squares(residual, previous, bounds=(lo, hi), method="trf",
            x_scale=coordinate_scale, ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=250)
        position, rotation = error(solution.x, float(s))
        if np.linalg.norm(position) > position_tolerance/10 or (task == "pose" and np.linalg.norm(rotation) > orientation_tolerance/10):
            raise TrajectoryError("ik_unreachable", f"IK failed at path fraction {s:.4f}; target may be unreachable or seed/limits unsuitable")
        if np.max(np.abs(solution.x-previous)/coordinate_scale) > max_normalized_step:
            raise TrajectoryError("branch_jump", f"IK step too large at fraction {s:.4f}; use more knots or a closer seed")
        minimum_singular = min(minimum_singular, singularity(solution.x))
        configurations.append(solution.x)
    # q(s) is a C2 cubic geometric interpolant. It is not itself a timed motion.
    spline = interpolate.CubicSpline(grid, np.array(configurations), axis=0)
    pos_error, rot_error = 0., 0.
    for s in np.linspace(0, 1, 8*(knots-1)+1):
        q = spline(s)
        position, rotation = error(q, float(s))
        pos_error = max(pos_error, float(np.linalg.norm(position)))
        rot_error = max(rot_error, float(np.linalg.norm(rotation)))
        minimum_singular = min(minimum_singular, singularity(q))
    if pos_error > position_tolerance or (task == "pose" and rot_error > orientation_tolerance):
        raise TrajectoryError("cartesian_path_error", "Interpolated FK deviates from target path; increase IK knots or revise path")
    law = time_law("quintic")
    # Invert monotone s(t/T) at geometric knots; compose each cubic locally to
    # avoid ill-conditioned global-time coefficients near small knot intervals.
    normalized_times = np.array([0.] + [optimize.brentq(lambda u: law(u)-s, 0., 1.) for s in grid[1:-1]] + [1.])
    coefficients = []
    for k in range(knots-1):
        local_time = Polynomial([normalized_times[k], normalized_times[k+1]-normalized_times[k]])
        local_s = law(local_time)-grid[k]
        segment = []
        for j in range(n):
            poly = Polynomial([0.])
            for power in range(4):
                poly += spline.c[3-power, k, j]*local_s**power
            segment.append(np.pad(poly.coef, (0, 16-len(poly.coef))))
        coefficients.append(segment)
    c = np.asarray(coefficients)
    # Find actual maxima of composed q(u) derivatives, rather than guessing a
    # Cartesian speed that may violate joint speed/acceleration constraints.
    vpeak, apeak = np.zeros(n), np.zeros(n)
    for k in range(len(c)):
        dt = normalized_times[k+1]-normalized_times[k]
        for j in range(n):
            p = Polynomial(c[k, j])
            vpeak[j] = max(vpeak[j], max(abs(x) for x in extrema(p.deriv()))/dt)
            apeak[j] = max(apeak[j], max(abs(x) for x in extrema(p.deriv(2)))/dt**2)
    minimum_duration = max(float(np.max(vpeak/limits.velocity)), float(np.sqrt(np.max(apeak/limits.acceleration))), .001)
    actual_duration = minimum_duration*1.000001 if duration is None else duration
    if not np.isfinite(actual_duration) or actual_duration <= 0:
        raise TrajectoryError("invalid_duration", "Duration must be finite and positive")
    if actual_duration+1e-10 < minimum_duration:
        raise TrajectoryError("duration_too_short", f"Cartesian path needs at least {minimum_duration:.6g} seconds")
    return Trajectory(chain, normalized_times*actual_duration, c, limits, {
        "kind": "cartesian_line", "task": task, "method": "C2 cubic geometric IK path composed with global quintic timing",
        "start_pose": start.tolist(), "target_pose": target.tolist(), "geometric_knots": grid.tolist(),
        "ik_configurations": np.asarray(configurations).tolist(), "minimum_duration": minimum_duration,
        "minimum_scaled_jacobian_singular_value": minimum_singular,
        "jacobian_scaling": "Joint units: 1 rad or 0.1 m; angular task rows scaled by 0.1 m/rad",
        "max_position_error_m": pos_error, "max_orientation_error_rad": rot_error if task == "pose" else None,
        "position_tolerance_m": position_tolerance, "orientation_tolerance_rad": orientation_tolerance,
        "singular_value_min": singular_value_min, "max_normalized_step": max_normalized_step,
        "geometric_validation_samples": 8*(knots-1)+1,
        "geometric_validation_scope": "Sampled FK and singularity checks, not a continuous Cartesian certificate; position-only leaves orientation unconstrained"})
