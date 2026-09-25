"""Fast numerical evaluation of the geometric Jacobian used by the symbolic view."""

import numpy as np

from urdf2dt.dh.types import DHModel, JointType
from urdf2dt.kinematics import dh_frame_transforms


def geometric_jacobian(model: DHModel, q) -> tuple[tuple[float, ...], ...]:
    """Tip-origin [linear; angular] velocity in selected base, per source qdot.

    This evaluates the same axis/cross-product construction as symbolic_jacobian
    without substituting into large expressions on the GUI thread.
    """
    frames = [np.asarray(frame) for frame in dh_frame_transforms(model, q)]
    tip = frames[-1] @ np.asarray(model.tool_transform)
    matrix = np.zeros((6, len(model.rows)))
    for index, (row, frame) in enumerate(zip(model.rows, frames)):
        axis = row.joint_sign * frame[:3, 2]
        if row.joint_type == JointType.PRISMATIC:
            matrix[:3, index] = axis
        else:
            matrix[:3, index] = np.cross(axis, tip[:3, 3] - frame[:3, 3])
            matrix[3:, index] = axis
    return tuple(tuple(float(value) for value in row) for row in matrix)
