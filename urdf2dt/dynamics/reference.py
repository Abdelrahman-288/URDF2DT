"""Optional independent MuJoCo reference, imported only for validation.

The reference parses source URDF joints/inertias itself. Geometry and plugins
are removed before import; no external assets or user simulator extensions run.
This is an instantaneous dynamics check, not the Stage 28 simulation backend.
"""

from importlib import import_module
from xml.etree.ElementTree import Element, SubElement, tostring
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation

from urdf2dt.dynamics.model import DynamicModel
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.xml_snapshot import read_snapshot
from urdf2dt.kinematics import urdf_link_transforms


class MuJoCoReference:
    """Zero contact, limits, passive friction and armature; same fixed base/gravity."""

    def __init__(self, source: URDFInput, model: DynamicModel):
        if source.sha256 != model.chain.source_sha256:
            raise ValueError("Reference source digest mismatch")
        self.friction = {f.joint_name: f for f in model.config.friction}
        self.mj = import_module("mujoco")
        original = read_snapshot(source)
        safe = Element("robot", {"name": original.get("name", "reference")})
        for child in original:
            if child.tag == "link":
                link = SubElement(safe, "link", name=child.attrib["name"])
                for inertia in child.findall("inertial"):
                    copied = deepcopy(inertia)
                    # MuJoCo #3559: full-inertia diagonalization can discard URDF
                    # inertial-origin rotation. Independently rotate the XML tensor
                    # into link axes before import, preserving its COM location.
                    origin, tensor = copied.find("origin"), copied.find("inertia")
                    if origin is not None and tensor is not None:
                        rotation = Rotation.from_euler("xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]).as_matrix()
                        xx, yy, zz, xy, xz, yz = [float(tensor.attrib[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
                        rotated = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
                        for key, i, j in (("ixx",0,0), ("iyy",1,1), ("izz",2,2), ("ixy",0,1), ("ixz",0,2), ("iyz",1,2)):
                            tensor.set(key, format(rotated[i,j], ".17g"))
                        origin.set("rpy", "0 0 0")
                    link.append(copied)
            elif child.tag == "joint":
                joint = SubElement(safe, "joint", child.attrib)
                for field in child:
                    if field.tag in {"parent", "child", "origin", "axis", "limit"}:
                        joint.append(deepcopy(field))
        extension = SubElement(safe, "mujoco")
        SubElement(extension, "compiler", fusestatic="false", inertiafromgeom="false", balanceinertia="false")
        self.model = self.mj.MjModel.from_xml_string(tostring(safe, encoding="unicode"))
        self.data = self.mj.MjData(self.model)
        self.chain = model.chain
        if self.model.nq != model.dof or self.model.nv != model.dof:
            raise ValueError("Reference model changed the degrees of freedom")
        names = tuple(self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_JOINT, i)
                      for i in range(self.model.njnt))
        if names != self.chain.joint_names:
            raise ValueError("Reference model changed the joint ordering")
        self.model.opt.gravity[:] = model.config.gravity
        self.model.jnt_limited[:] = 0
        self.model.dof_damping[:] = 0
        self.model.dof_frictionloss[:] = 0
        self.model.dof_armature[:] = 0
        # Verify importer semantics, including masses, COMs and tensor axes.
        for record in model.inertials.records:
            body = record.properties
            if body is None or body.mass == 0 or record.link_name == self.chain.base_link:
                continue
            index = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, record.link_name)
            if index < 0:
                raise ValueError(f"Reference discarded inertial link {record.link_name}")
            rotation = np.zeros(9)
            self.mj.mju_quat2Mat(rotation, self.model.body_iquat[index])
            rotation = rotation.reshape(3, 3)
            expected_com, expected_tensor, _ = body.in_frame(tuple(map(tuple, np.eye(4))))
            actual_tensor = rotation @ np.diag(self.model.body_inertia[index]) @ rotation.T
            if (not np.isclose(self.model.body_mass[index], body.mass, rtol=1e-12, atol=1e-12)
                or not np.allclose(self.model.body_ipos[index], expected_com, rtol=1e-12, atol=1e-12)
                # MuJoCo diagonalizes full tensors with finite eigensolver precision.
                or not np.allclose(actual_tensor, expected_tensor, rtol=0, atol=1e-10)):
                raise ValueError(f"Reference changed inertial parameters for {record.link_name}")

    def inverse(self, q: np.ndarray, v: np.ndarray, a: np.ndarray) -> np.ndarray:
        self.data.qpos[:] = q
        self.data.qvel[:] = v
        self.data.qacc[:] = a
        self.mj.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse.copy() + self._friction(v)

    def _friction(self, velocity: np.ndarray) -> np.ndarray:
        """Analytical kinetic-friction reference, outside MuJoCo's stiction solver."""
        result = []
        for name, value in zip(self.chain.joint_names, velocity):
            f = self.friction.get(name)
            sign = 1 if value > 0 else -1 if value < 0 else 0
            result.append(0. if f is None else f.viscous * value + f.coulomb * sign)
        return np.array(result)

    def mass(self, q: np.ndarray) -> np.ndarray:
        n = self.model.nv
        zero = np.zeros(n)
        bias = self.inverse(q, zero, zero)
        return np.column_stack([self.inverse(q, zero, unit) - bias for unit in np.eye(n)])

    def forward(self, q: np.ndarray, v: np.ndarray, effort: np.ndarray) -> np.ndarray:
        self.data.qpos[:] = q
        self.data.qvel[:] = v
        self.data.qfrc_applied[:] = effort - self._friction(v)
        self.mj.mj_forward(self.model, self.data)
        return self.data.qacc.copy()

    def link_pose_error(self, q: np.ndarray) -> float:
        """Check every imported link transform, not only end-effector agreement."""
        self.data.qpos[:] = q
        self.mj.mj_kinematics(self.model, self.data)
        names = (self.chain.base_link,) + tuple(j.child_link for j in self.chain.joints)
        maximum = 0.
        for name, pose in zip(names, urdf_link_transforms(self.chain, q.tolist())):
            body = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, name)
            # MuJoCo represents the fixed URDF root by its world body.
            if body < 0 and name == self.chain.base_link:
                body = 0
            if body < 0:
                raise ValueError(f"Reference discarded link {name}")
            transform = np.eye(4)
            transform[:3, :3] = self.data.xmat[body].reshape(3, 3)
            transform[:3, 3] = self.data.xpos[body]
            maximum = max(maximum, float(np.max(np.abs(transform - np.asarray(pose)))))
        return maximum
