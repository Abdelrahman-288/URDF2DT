"""Deterministic Standard-DH construction from zero-pose serial joint axes.

Frames are selected geometrically, not fitted to samples or a robot-specific
table. Joint i acts about/along z_(i-1). Signed common normals keep successive
x directions aligned where possible. Terminal residual geometry stays in tool.
"""

from math import atan2, hypot
from sys import float_info

from urdf2dt._transforms import (
    add, column, cross, dh_transform, dot, frame, inverse, multiply,
    position, rotate, scale, subtract, unit,
)
from urdf2dt.config import EditorConfig
from urdf2dt.dh.types import (
    DHModel, DHRow, IDENTITY, Joint, JointType, KinematicChain,
    REPRESENTATION_ATOL, Transform, Vector,
)

# Roundoff budget for unit-vector arithmetic, not a geometry classification setting.
_ROUND_OFF = 64 * float_info.epsilon


class DHSolverError(ValueError):
    """A geometry cannot be stably represented under the selected solver policy."""

    def __init__(self, code: str, message: str, joint_names: tuple[str, ...] = ()):
        self.code, self.joint_names = code, joint_names
        super().__init__(message)


def _perpendicular(z: Vector) -> Vector:
    # Choose the coordinate axis least aligned with z, with deterministic ties.
    index = min(range(3), key=lambda i: abs(z[i]))
    candidate = tuple(float(i == index) for i in range(3))
    return unit(subtract(candidate, scale(z, dot(candidate, z))))


def _next_frame(origin: Vector, z: Vector, next_origin: Vector, next_z: Vector,
                previous_x: Vector | None, threshold: float, names: tuple[str, ...]) -> Transform:
    normal = cross(z, next_z)
    sine = hypot(*normal)
    if _ROUND_OFF < sine <= threshold:
        raise DHSolverError("ill_conditioned_axes", "Near-parallel axes require reference review; "
                            "the solver will not approximate them as parallel.", names)
    delta = subtract(next_origin, origin)
    if sine <= _ROUND_OFF:
        # Coincident/opposite/parallel axes: choose the next origin in the same
        # normal plane as the current origin, placing no arbitrary axial offset.
        q = add(next_origin, scale(next_z, dot(subtract(origin, next_origin), next_z)))
        displacement = subtract(q, origin)
        distance = hypot(*displacement)
        if distance > _ROUND_OFF * max(1., hypot(*origin), hypot(*next_origin)):
            x = unit(displacement)
        else:
            x = previous_x if previous_x is not None else _perpendicular(z)
    else:
        # q = next_origin + t*next_z; t uses the line-line cross-product formula.
        t = dot(cross(delta, z), normal) / (sine * sine)
        q = add(next_origin, scale(next_z, t))
        x = scale(normal, 1. / sine)
    if previous_x is not None and dot(x, previous_x) < -_ROUND_OFF:
        x = scale(x, -1.)
    # Remove machine-scale projection onto next_z before assembling a rigid frame.
    x = unit(subtract(x, scale(next_z, dot(x, next_z))))
    return frame(q, x, next_z)


def _row(previous: Transform, following: Transform, joint: Joint) -> DHRow:
    x, z, next_z = column(following, 0), column(previous, 2), column(following, 2)
    delta = subtract(position(following), position(previous))
    result = DHRow(
        a=dot(delta, x), alpha=atan2(dot(x, cross(z, next_z)), dot(z, next_z)),
        d=dot(delta, z), theta_offset=atan2(dot(column(previous, 1), x), dot(column(previous, 0), x)),
        joint_name=joint.name, joint_type=joint.joint_type,
    )
    # Fail rather than accepting a numerically invalid frame factorization.
    expected = multiply(inverse(previous), following)
    actual = dh_transform(result.a, result.alpha, result.d, result.theta_offset)
    if any(abs(a - b) > REPRESENTATION_ATOL * max(1., abs(b))
           for ar, br in zip(actual, expected) for a, b in zip(ar, br)):
        raise DHSolverError("factorization_failed", "DH frame factorization exceeded numerical precision.", (joint.name,))
    return result


class StandardDHSolver:
    """Implement DHSolver for well-conditioned fixed/revolute/continuous/prismatic chains."""

    def solve(self, chain: KinematicChain, config: EditorConfig) -> DHModel:
        """Return one immutable row per movable joint, with unchanged q signs/order.

        The parallel threshold is a rejection boundary, not an approximation
        tolerance. No small distance is snapped using intersection tolerances.
        """
        if not isinstance(chain, KinematicChain) or not isinstance(config, EditorConfig):
            raise TypeError("solve requires KinematicChain and EditorConfig")
        try:
            return self._solve(chain, config)
        except DHSolverError:
            raise
        except (ValueError, OverflowError, ZeroDivisionError) as exc:
            raise DHSolverError("numerical_failure", "Joint geometry exceeded finite rigid-transform "
                                "precision; check origins, axes and model scale.", chain.joint_names) from exc

    def _solve(self, chain: KinematicChain, config: EditorConfig) -> DHModel:
        movable: list[Joint] = []
        origins: list[Vector] = []
        axes: list[Vector] = []
        tip = IDENTITY
        for joint in chain.joints:
            tip = multiply(tip, joint.origin)
            if joint.joint_type != JointType.FIXED:
                movable.append(joint)
                origins.append(position(tip))
                axes.append(unit(rotate(tip, joint.axis)))
        if not movable:
            raise DHSolverError("no_movable_joints", "DH construction requires a movable joint.")
        # Choose the point on joint 1's line closest to the selected URDF base origin.
        base_origin = subtract(origins[0], scale(axes[0], dot(origins[0], axes[0])))
        threshold = config.geometry.parallel_threshold
        if len(movable) > 1:
            first_next = _next_frame(base_origin, axes[0], origins[1], axes[1], None,
                                     threshold, (movable[0].name, movable[1].name))
            base_x = column(first_next, 0)
        else:
            base_x = _perpendicular(axes[0])
        base = frame(base_origin, base_x, axes[0])
        previous = base
        rows: list[DHRow] = []
        for i, joint in enumerate(movable):
            if i + 1 < len(movable):
                following = _next_frame(position(previous), axes[i], origins[i+1], axes[i+1],
                                        column(previous, 0), threshold, (joint.name, movable[i+1].name))
            else:
                # Project the physical tip onto the final joint line. The fixed
                # tool transform retains off-axis translation and orientation.
                q = add(origins[i], scale(axes[i], dot(subtract(position(tip), origins[i]), axes[i])))
                following = frame(q, column(previous, 0), axes[i])
            rows.append(_row(previous, following, joint))
            previous = following
        return DHModel(
            robot_name=chain.robot_name, rows=tuple(rows), source_urdf=chain.source_urdf,
            source_sha256=chain.source_sha256, base_transform=base,
            tool_transform=multiply(inverse(previous), tip),
            provenance=("StandardDHSolver v1: common-normal construction from URDF zero-pose axes; "
                        f"parallel rejection threshold={threshold}; references={config.reference_status}. "
                        "Numeric model; not a global-validation certificate."),
        )
