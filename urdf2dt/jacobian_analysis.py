"""Relative geometric Jacobians for physical links and attached frames."""
import numpy as np
from urdf2dt._transforms import multiply, rpy_transform
from urdf2dt.dh.types import JointType
from urdf2dt.kinematics import _configuration, urdf_link_transforms, dh_frame_transforms
from urdf2dt.inspection import physical_link_poses


def analyze_jacobian(chain, q, reference, target, rates=None, model=None,
                     named_frames=None, characteristic_length=1.0, threshold=1e-9):
    """Relative target-origin twist expressed in the possibly moving reference.

    model=None uses URDF; a DH model uses aligned physical-link poses. Rates use
    source coordinates. Normalized SVD scales linear rows by 1/L and prismatic
    columns by L, with L in metres; diagnostics depend on this explicit choice.
    """
    q = _configuration(q, len(chain.joint_names))
    rates = _configuration(rates if rates is not None else [0]*len(q), len(q))
    if not np.isfinite(characteristic_length) or characteristic_length <= 0:
        raise ValueError('Characteristic length must be finite and positive')
    if not np.isfinite(threshold) or not 0 < threshold < 1:
        raise ValueError('Relative SVD threshold must be between zero and one')
    names = [chain.base_link] + [j.child_link for j in chain.joints]
    urdf = urdf_link_transforms(chain, q)
    poses = list(urdf if model is None else physical_link_poses(chain, model, q))
    counts = [0]
    axes: list = []
    origins, types = [], []
    dh = dh_frame_transforms(model, q) if model is not None else None
    for i, joint in enumerate(chain.joints):
        if joint.joint_type != JointType.FIXED:
            index = len(axes)
            if model is None:
                frame = np.asarray(multiply(urdf[i], joint.origin))
                axis = frame[:3,:3] @ np.asarray(joint.axis)
            else:
                assert dh is not None
                frame = np.asarray(dh[index])
                axis = frame[:3,2] * model.rows[index].joint_sign
            axes.append(axis)
            origins.append(frame[:3,3])
            types.append(joint.joint_type)
        counts.append(len(axes))
    original = list(names)
    for name, value in (named_frames or {}).items():
        if name in names or value['link'] not in original:
            raise ValueError('Named frames require unique names and existing owning links')
        owner = original.index(value['link'])
        names.append(name)
        counts.append(counts[owner])
        poses.append(multiply(poses[owner],rpy_transform(value['xyz'],value['rpy'])))
    if reference not in names or target not in names:
        raise ValueError('Choose existing physical link frames')

    def absolute(index):
        result = np.zeros((6,len(q)))
        point = np.asarray(poses[index])[:3,3]
        for j in range(counts[index]):
            if types[j] == JointType.PRISMATIC:
                result[:3,j] = axes[j]
            else:
                result[:3,j] = np.cross(axes[j],point-origins[j])
                result[3:,j] = axes[j]
        return result
    ri, ti = names.index(reference), names.index(target)
    jr, jt = absolute(ri), absolute(ti)
    rotation = np.asarray(poses[ri])[:3,:3]
    offset = np.asarray(poses[ti])[:3,3] - np.asarray(poses[ri])[:3,3]
    matrix = np.vstack((rotation.T @ (jt[:3]-jr[:3]-np.cross(jr[3:].T,offset).T),
                        rotation.T @ (jt[3:]-jr[3:])))
    normalized = matrix.copy()
    normalized[:3] /= characteristic_length
    for j, kind in enumerate(types):
        if kind == JointType.PRISMATIC:
            normalized[:,j] *= characteristic_length
    singular = np.linalg.svd(normalized,compute_uv=False)
    cutoff = threshold * singular[0] if len(singular) else 0.0
    rank = int(np.sum(singular > cutoff))
    capacity = min(6,len(q))
    condition = float(singular[0]/singular[-1]) if capacity and rank == capacity else None
    return dict(matrix=matrix.tolist(), linear=matrix[:3].tolist(), angular=matrix[3:].tolist(),
                twist=(matrix @ rates).tolist(), q_si=list(q), rates_si=list(rates),
                reference=reference, target=target, expressed_in=reference,
                joint_names=list(chain.joint_names), joint_types=[j.value for j in types],
                convention='Geometric relative target-origin twist [v; omega], expressed in reference; not RPY rates',
                rank=rank, maximum_rank=capacity, full_spatial_rank=6,
                singular_values=singular.tolist(), condition_number=condition,
                rank_deficient=rank<capacity, characteristic_length_m=characteristic_length,
                relative_threshold=threshold, svd_cutoff=cutoff,
                normalization='linear rows / L; prismatic columns * L; rotational coordinates in radians')
