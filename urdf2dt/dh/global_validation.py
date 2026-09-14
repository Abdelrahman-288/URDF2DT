"""Deterministic sampled FK validation using reusable CasADi expressions.

A sampled PASS is not proof over the continuous configuration space. Prefix
diagnostics align differing frame conventions at zero and are not causal blame.
"""
from dataclasses import asdict, dataclass
from hashlib import sha256
from importlib import import_module
import json
from math import acos, hypot, isfinite, pi
from random import Random
from typing import Any

from urdf2dt._transforms import inverse, multiply
from urdf2dt.config import EditorConfig
from urdf2dt.dh.types import DHModel, JointType, KinematicChain, Transform


def model_hash(model: DHModel) -> str:
    return sha256(json.dumps(asdict(model), sort_keys=True, allow_nan=False).encode()).hexdigest()


def sample_configurations(chain: KinematicChain, config: EditorConfig) -> tuple[tuple[float, ...], ...]:
    """Total configured sample count includes zero; remaining samples honor limits.

    Continuous joints use [-pi, pi]. Zero is deliberately included even when
    outside a bounded joint's limits and is identified as such in the report.
    """
    rng = Random(config.validation.random_seed)
    joints = [j for j in chain.joints if j.joint_type != JointType.FIXED]
    samples = [(0.,) * len(joints)]
    for _ in range(config.validation.samples - 1):
        samples.append(tuple(rng.uniform(j.limit.lower, j.limit.upper) if j.limit else rng.uniform(-pi, pi)
                             for j in joints))
    return tuple(samples)


def _matrix(value: Any) -> Transform:
    return tuple(tuple(float(value[i, j]) for j in range(4)) for i in range(4))


def pose_errors(reference: Transform, candidate: Transform) -> tuple[float, float]:
    if not all(isfinite(v) for t in (reference, candidate) for row in t for v in row):
        raise ValueError("FK produced nonfinite values")
    position = hypot(*(reference[i][3] - candidate[i][3] for i in range(3)))
    trace = sum(reference[k][i] * candidate[k][i] for k in range(3) for i in range(3))
    angle = acos(max(-1., min(1., (trace - 1.) / 2.)))
    if not isfinite(position):
        raise ValueError("FK error exceeded finite precision")
    return position, angle


class FKFunctions:
    """Compile URDF and DH once per immutable chain/model pair, preserving q order."""
    def __init__(self, chain: KinematicChain, model: DHModel):
        joints = tuple(j for j in chain.joints if j.joint_type != JointType.FIXED)
        if (chain.joint_names != model.joint_names or chain.robot_name != model.robot_name
                or chain.source_sha256 != model.source_sha256
                or tuple(j.joint_type for j in joints) != tuple(r.joint_type for r in model.rows)):
            raise ValueError("FK chain/model source and ordered joint identities must match")
        ca = import_module("casadi")
        q = ca.SX.sym("q", len(joints))
        urdf = ca.SX.eye(4)
        u_frames = []
        index = 0
        for joint in chain.joints:
            urdf = ca.mtimes(urdf, ca.DM(joint.origin))
            if joint.joint_type != JointType.FIXED:
                motion = ca.SX.eye(4)
                x, y, z = joint.axis
                if joint.joint_type == JointType.PRISMATIC:
                    motion[:3, 3] = ca.DM(joint.axis) * q[index]
                else:
                    axis = ca.DM(joint.axis)
                    skew = ca.vertcat(ca.horzcat(0, -z, y), ca.horzcat(z, 0, -x), ca.horzcat(-y, x, 0))
                    motion[:3, :3] = ca.cos(q[index])*ca.SX.eye(3) + (1-ca.cos(q[index]))*ca.mtimes(axis, axis.T) + ca.sin(q[index])*skew
                urdf = ca.mtimes(urdf, motion)
                u_frames.append(urdf)
                index += 1
        dh = ca.SX(ca.DM(model.base_transform))
        d_frames = []
        for i, row in enumerate(model.rows):
            value = row.joint_sign * q[i]
            d = row.d + value if row.joint_type == JointType.PRISMATIC else row.d
            theta = row.theta_offset if row.joint_type == JointType.PRISMATIC else row.theta_offset + value
            ct, st, c, s = ca.cos(theta), ca.sin(theta), ca.cos(row.alpha), ca.sin(row.alpha)
            t = ca.vertcat(ca.horzcat(ct, -st*c, st*s, row.a*ct),
                           ca.horzcat(st, ct*c, -ct*s, row.a*st),
                           ca.horzcat(0, s, c, d), ca.horzcat(0, 0, 0, 1))
            dh = ca.mtimes(dh, t)
            d_frames.append(dh)
        self.urdf = ca.Function("urdf_fk", [q], [urdf])
        self.dh = ca.Function("dh_fk", [q], [ca.mtimes(dh, ca.DM(model.tool_transform))])
        self.urdf_prefixes = ca.Function("urdf_prefixes", [q], u_frames)
        self.dh_prefixes = ca.Function("dh_prefixes", [q], d_frames)
        self.count = len(joints)

    def prefixes(self, function: Any, q: tuple[float, ...]) -> tuple[Transform, ...]:
        values = function(q)
        return tuple(_matrix(v) for v in (values if isinstance(values, tuple) else (values,)))


@dataclass(frozen=True, slots=True)
class GlobalFKReport:
    """Owned JSON text prevents mutable report contents from changing after validation."""
    json_text: str

    def to_dict(self) -> dict[str, Any]:
        return json.loads(self.json_text)

    @property
    def passed(self) -> bool:
        return bool(self.to_dict()["passed"])

    def markdown(self) -> str:
        r = self.to_dict()
        s = r["summary"]
        return (f"# Sampled global FK: {'PASS' if r['passed'] else 'FAIL'}\n\n"
                f"Candidate: `{r['candidate_hash']}`\n\n"
                f"Samples: {len(r['samples'])}; reference status: {r['configuration']['reference_status']}.\n\n"
                f"Tolerances: {r['configuration']['validation']['position_tolerance_m']} m / "
                f"{r['configuration']['validation']['orientation_tolerance_rad']} rad.\n\n"
                f"Position max/mean: {s['max_position_error_m']:.9g} / {s['mean_position_error_m']:.9g} m.\n\n"
                f"Orientation max/mean: {s['max_orientation_error_rad']:.9g} / {s['mean_orientation_error_rad']:.9g} rad.\n\n"
                f"Worst position/orientation sample indices (zero-based): {s['worst_position_sample']} / {s['worst_orientation_sample']}.\n\n"
                f"{r['diagnostic']['message']}\n\nSampled evidence only; not a continuous-space proof.\n")


def validate_global_fk(chain: KinematicChain, model: DHModel, config: EditorConfig | None = None) -> GlobalFKReport:
    settings = config if config is not None else EditorConfig()
    functions = FKFunctions(chain, model)
    samples = sample_configurations(chain, settings)
    uz = functions.prefixes(functions.urdf_prefixes, samples[0])
    dz = functions.prefixes(functions.dh_prefixes, samples[0])
    # Gauge alignment is necessary: URDF link origins and editable DH origins differ.
    alignment = tuple(multiply(inverse(d), u) for u, d in zip(uz, dz))
    metrics: list[dict[str, Any]] = []
    first = None
    for index, q in enumerate(samples):
        pos, angle = pose_errors(_matrix(functions.urdf(q)), _matrix(functions.dh(q)))
        prefix = []
        us = functions.prefixes(functions.urdf_prefixes, q)
        ds = functions.prefixes(functions.dh_prefixes, q)
        for i, (u, d, correction) in enumerate(zip(us, ds, alignment), 1):
            p, a = pose_errors(u, multiply(d, correction))
            prefix.append({"frame_index": i, "position_error_m": p, "orientation_error_rad": a})
            if (p > settings.validation.position_tolerance_m or a > settings.validation.orientation_tolerance_rad):
                if first is None or i < first:
                    first = i
        metrics.append({"index": index, "q": q, "position_error_m": pos,
                        "orientation_error_rad": angle, "prefix_errors": prefix})
    positions = [m["position_error_m"] for m in metrics]
    angles = [m["orientation_error_rad"] for m in metrics]
    passed = max(positions) <= settings.validation.position_tolerance_m and max(angles) <= settings.validation.orientation_tolerance_rad
    if passed:
        message = "All sampled end-effector poses are within configured tolerances."
        first = None
    elif first is None:
        message = "End-effector mismatch with aligned prefixes within tolerance: inspect base/tool alignment or constant geometry."
    else:
        message = f"Aligned prefix deviation first exceeds tolerance at frame {first}; inspect this frame and upstream geometry."
    report = {"report_kind": "sampled-global-fk-v1", "passed": passed,
              "candidate_hash": model_hash(model), "candidate": asdict(model),
              "source_sha256": chain.source_sha256, "joint_names": chain.joint_names,
              "configuration": settings.snapshot(), "sampling": "Python Random(seed); sample count includes zero; continuous joints [-pi,pi]",
              "zero_within_limits": all(j.limit is None or j.limit.lower <= 0 <= j.limit.upper for j in chain.joints),
              "summary": {"max_position_error_m": max(positions), "mean_position_error_m": sum(positions)/len(samples),
                          "max_orientation_error_rad": max(angles), "mean_orientation_error_rad": sum(angles)/len(samples),
                          "worst_position_sample": positions.index(max(positions)), "worst_orientation_sample": angles.index(max(angles))},
              "diagnostic": {"first_frame": first, "message": message,
                             "method": "zero-pose gauge-aligned link prefixes; constant offsets may be unlocalizable"},
              "samples": metrics}
    return GlobalFKReport(json.dumps(report, indent=2, allow_nan=False))
