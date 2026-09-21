"""Physical-link FK inspection with explicit DH-to-URDF frame alignment."""

import warnings
import numpy as np
from scipy.spatial.transform import Rotation
from urdf2dt._transforms import inverse, multiply, rpy_transform
from urdf2dt.kinematics import urdf_link_transforms, dh_frame_transforms
from urdf2dt.dh.types import JointType


def physical_link_poses(chain, model, q):
    urdf_zero = urdf_link_transforms(chain, [0.0] * len(q))
    dh_zero = dh_frame_transforms(model, [0.0] * len(q))
    dh = dh_frame_transforms(model, q)
    frames = [0]
    index = 0
    for joint in chain.joints:
        if joint.joint_type != JointType.FIXED:
            index += 1
        frames.append(index)
    return tuple(
        multiply(dh[k], multiply(inverse(dh_zero[k]), link))
        for k, link in zip(frames, urdf_zero)
    )


def inspect_fk(chain, automatic, accepted, q, reference, target, named_frames=None):
    names = [chain.base_link] + [j.child_link for j in chain.joints]
    original_names = list(names)
    attached = named_frames or {}
    for name, value in attached.items():
        if name in names or value["link"] not in original_names:
            raise ValueError(
                "Named frames require unique names and existing owning links"
            )
        names.append(name)

    def extend(poses):
        result = list(poses)
        for value in attached.values():
            result.append(
                multiply(
                    poses[original_names.index(value["link"])],
                    rpy_transform(value["xyz"], value["rpy"]),
                )
            )
        return result

    if reference not in names or target not in names:
        raise ValueError("Choose existing physical link frames")
    i, j = names.index(reference), names.index(target)
    poses = extend(urdf_link_transforms(chain, q))
    result = np.asarray(multiply(inverse(poses[i]), poses[j]))
    rotation = Rotation.from_matrix(result[:3, :3])
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        rpy = rotation.as_euler("xyz")
    comparisons = {}
    for label, model in (("automatic", automatic), ("accepted", accepted)):
        aligned = extend(physical_link_poses(chain, model, q))
        value = np.asarray(multiply(inverse(aligned[i]), aligned[j]))
        comparisons[label] = {
            "matrix": value.tolist(),
            "position_error_m": float(np.linalg.norm(value[:3, 3] - result[:3, 3])),
            "orientation_error_rad": float(
                Rotation.from_matrix(value[:3, :3] @ result[:3, :3].T).magnitude()
            ),
        }
    return {
        "joint_names": list(chain.joint_names),
        "q_si": list(q),
        "reference": reference,
        "target": target,
        "matrix": result.tolist(),
        "position_m": result[:3, 3].tolist(),
        "rotation_matrix": result[:3, :3].tolist(),
        "quaternion_xyzw": rotation.as_quat().tolist(),
        "rpy_rad": rpy.tolist(),
        "rpy_convention": "Extrinsic fixed-axis XYZ: Rz(yaw) Ry(pitch) Rx(roll)",
        "orientation_warning": "Gimbal lock: roll/yaw are not unique" if caught else "",
        "comparison": comparisons,
        "scope": "Single-pose calculation; not sampled global validation",
    }
