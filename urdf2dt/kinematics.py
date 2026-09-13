"""Numeric URDF and Standard-DH FK for integration checks and future rendering.

These are evaluators, not the Stage 11 sampled/global validation service. Limits
are preserved by the chain but not enforced here: the evaluators accept any finite
q of the correct dimension, including zero when it lies outside physical limits.
"""

from collections.abc import Sequence

from urdf2dt._transforms import axis_motion, dh_transform, multiply
from urdf2dt.dh.types import DHModel, IDENTITY, JointType, KinematicChain, Transform, Vector, _number


def _configuration(q: Sequence[float], count: int) -> Vector:
    values = tuple(_number(value, "q") for value in q)
    if len(values) != count:
        raise ValueError(f"expected {count} joint coordinates, got {len(values)}")
    return values


def urdf_link_transforms(chain: KinematicChain, q: Sequence[float]) -> tuple[Transform, ...]:
    """Base-relative poses: base identity followed by each joint's child link."""
    values = iter(_configuration(q, len(chain.joint_names)))
    poses = [IDENTITY]
    for joint in chain.joints:
        current = multiply(poses[-1], joint.origin)
        if joint.joint_type != JointType.FIXED:
            current = multiply(current, axis_motion(joint.axis, next(values), joint.joint_type == JointType.PRISMATIC))
        poses.append(current)
    return tuple(poses)


def urdf_fk(chain: KinematicChain, q: Sequence[float]) -> Transform:
    """Pose of the selected URDF tip in the selected base link coordinates."""
    return urdf_link_transforms(chain, q)[-1]


def dh_frame_transforms(model: DHModel, q: Sequence[float]) -> tuple[Transform, ...]:
    """Base-relative DH frames F0..Fn, before the fixed terminal tool transform."""
    values = _configuration(q, len(model.rows))
    frames = [model.base_transform]
    for row, value in zip(model.rows, values):
        displacement = row.joint_sign * value
        d = row.d + displacement if row.joint_type == JointType.PRISMATIC else row.d
        theta = row.theta_offset if row.joint_type == JointType.PRISMATIC else row.theta_offset + displacement
        frames.append(multiply(frames[-1], dh_transform(row.a, row.alpha, d, theta)))
    return tuple(frames)


def dh_fk(model: DHModel, q: Sequence[float]) -> Transform:
    """Pose of the URDF tip using DH rows with explicit base/tool alignment."""
    return multiply(dh_frame_transforms(model, q)[-1], model.tool_transform)
