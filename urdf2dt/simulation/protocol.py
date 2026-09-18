"""Predeclared Stage 28 scenarios and engineering acceptance thresholds."""

from dataclasses import asdict
import numpy as np
from urdf2dt.control import ControllerConfig
from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.trajectory import MotionLimits, joint_trajectory, Trajectory


def scenario(robot: str, plant: DynamicModel) -> tuple[Trajectory, list[ControllerConfig], np.ndarray, np.ndarray, dict]:
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    if robot == "scara":
        start = np.array([.3, .8, .06, .2])
        goal = start+[.04, -.03, .015, .01]
        offset = np.array([.02, -.01, .002, .01])
        limits = MotionLimits((1.,1.,.15,1.),(2.,2.,.3,2.),(30.,30.,100.,10.))
        kp, kd = (30.,30.,150.,3.), (6.,6.,20.,.4)
        nominal_rmse = [.01,.01,.002,.01]
        steady = [.001,.001,.0002,.001]
        band = [.005,.005,.001,.005]
        perturbation_bound = [.3,.3,.05,.3]
        speed_bound = [2.,2.,.5,2.]
    elif robot == "ur5":
        start = np.array([.3,-1.,1.2,-1.,.8,.2])
        goal = start+[.03,-.02,.03,.01,-.02,.01]
        offset = np.array([.02,-.01,.01,-.02,.01,-.01])
        limits = MotionLimits((1.,)*6,(2.,)*6,(150.,150.,150.,28.,28.,28.))
        kp, kd = (30.,30.,30.,30.,30.,3.), (6.,6.,6.,6.,6.,.6)
        nominal_rmse, steady, band = [.01]*6, [.001]*6, [.005]*6
        perturbation_bound, speed_bound = [.5]*6, [5.]*6
    else:
        raise ValueError("Choose ur5 or scara")
    if len(start) != plant.dof:
        raise ValueError("Demo joint count differs from selected robot")
    assert limits.effort is not None
    ct_kp = (25.,25.,100.,25.) if robot == "scara" else (25.,)*plant.dof
    ct_kd = (10.,10.,20.,10.) if robot == "scara" else (10.,)*plant.dof
    configs = [ControllerConfig(mode, ct_kp if mode == "computed_torque" else kp,
                ct_kd if mode == "computed_torque" else kd, limits.effort, .01)
               for mode in ("pd", "pd_gravity", "computed_torque")]
    reference = joint_trajectory(plant.chain, [start, goal, goal], limits, durations=[2.,2.])
    protocol = {"version": "1.0", "robot": robot, "backend": "python_rk4",
        "backend_confirmation": "Project owner confirmed Python RK4 with MuJoCo comparison in the Stage 28 conversation",
        "threshold_status": "Predeclared engineering demonstration criteria; formal advisor threshold acceptance not yet recorded",
        "controller_period_s": .01, "integration_steps_s": [.002,.001,.0005],
        "move_end_s": 2., "steady_window_s": 1., "settling_band": band,
        "controller_configs": [asdict(c) for c in configs], "controller_inertial_scales": [.8,1.,1.2],
        "initial_position": (start+offset).tolist(), "initial_velocity": (offset*.5).tolist(),
        "gain_revision": "Pilot-only Stage 27 100Hz wrist damping produced sampled-data oscillation. PD wrist Kp/Kd becomes SCARA=3/.4 and UR5=3/.6. Pilot SCARA CT prismatic gains 25/10 gave ~0.091/0.061m RMSE under -/+20% mass bias, beyond the 0.05m bound; gains become 100/20 (critical damping, predicted gravity-bias offset <=0.0246m). Formal runs use this frozen revision; other gains unchanged.",
        "gates": {
            "all_cases": {"effort_limits": list(limits.effort), "max_state_speed": speed_bound,
                          "scope": "finite states, source joint position bounds and effort clipping; no asymptotic stability proof"},
            "nominal_computed_torque": {"position_rmse": nominal_rmse, "steady_state_rmse": steady,
                "cartesian_rmse_m": .02, "orientation_rmse_rad": .02, "settling_after_move_s": 1., "max_saturation_fraction": .01},
            "perturbed_model_cases": {"position_rmse": perturbation_bound,
                "scope": "PD is model independent and retains its uncompensated-gravity baseline; PD+gravity/CT model perturbations must remain within declared bounded-error criteria"},
            "convergence": {"max_position_difference": 1e-4, "max_velocity_difference": 1e-3,
                            "scope": "Each nominal controller: dt=.002 versus .001 and .001 versus .0005 at fixed 100Hz control"},
            "independent_dynamics": {"max_position_difference": 1e-6, "max_velocity_difference": 1e-5,
                "max_acceleration_difference": 1e-4, "max_effort_difference": 1e-4,
                "scope": "Independent MuJoCo accelerations, same RK4 integration and held effort, all nominal controllers"}},
        "baseline_policy": "Report PD and parameter-biased residual errors honestly; strict tracking/settling acceptance applies to nominal computed torque",
        "hardware_validation": "Not performed; physical measurements remain Stage 29/30"}
    return reference, configs, start+offset, offset*.5, protocol


def assess_run(run: dict, metrics: dict, protocol: dict, scale: float) -> dict:
    gates = protocol["gates"]
    effort = np.asarray([c["effort"] for c in run["commands"]])
    checks = {"finite_states": bool(np.isfinite(run["position"]).all() and np.isfinite(run["velocity"]).all()),
              "effort_bounded": bool(np.all(np.max(np.abs(effort),axis=0) <= np.asarray(gates["all_cases"]["effort_limits"])+1e-10)),
              "speed_bounded": bool(np.all(np.asarray(run["maximum_integration_state_velocity"]) <= gates["all_cases"]["max_state_speed"]))}
    mode = run["controller"]["config"]["mode"]
    if mode == "computed_torque" and scale == 1.:
        goal = gates["nominal_computed_torque"]
        checks.update({"tracking": bool(np.all(np.asarray(metrics["joint_position_rmse"]) <= goal["position_rmse"])),
            "steady_state": bool(np.all(np.asarray(metrics["steady_state_rmse"]) <= goal["steady_state_rmse"])),
            "cartesian": metrics["cartesian_position_rmse_m"] <= goal["cartesian_rmse_m"],
            "orientation": metrics["orientation_rmse_rad"] <= goal["orientation_rmse_rad"],
            "settling": all(t is not None and t <= goal["settling_after_move_s"] for t in metrics["settling_time_after_move_s"]),
            "saturation": max(metrics["saturation_fraction"]) <= goal["max_saturation_fraction"]})
    elif mode != "pd" and scale != 1.:
        checks["perturbation_error_bounded"] = bool(np.all(np.asarray(metrics["joint_position_rmse"]) <= gates["perturbed_model_cases"]["position_rmse"]))
    return {"passed": all(checks.values()), "checks": checks}
