"""Exact linear dynamics in link-origin inertial coordinates (SI)."""

from typing import Any
import casadi as ca
import numpy as np

from urdf2dt.dh.types import KinematicChain, JointType, IDENTITY
from urdf2dt.dynamics.model import DynamicModel, DynamicsConfig, JointFriction
from urdf2dt.parser.inertial_extractor import InertialProperties, InertialRecord, InertialModel
from .data import IdentificationData

INERTIAL_NAMES = ("m", "hx", "hy", "hz", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")


def inertia_matrix(p: Any) -> np.ndarray:
    return np.array([[p[4], p[7], p[8]], [p[7], p[5], p[9]], [p[8], p[9], p[6]]], dtype=float)


def pseudo_inertia(p: Any) -> np.ndarray:
    tensor = inertia_matrix(p)
    return np.block([[.5*np.trace(tensor)*np.eye(3)-tensor, np.asarray(p[1:4]).reshape(3, 1)],
                     [np.asarray(p[1:4]).reshape(1, 3), np.array([[p[0]]])]])


class InertialRegressor:
    """Full moving-link parameters plus viscous/Coulomb joint friction.

    Explicit massless markers can be excluded by name; this is a fixed prior,
    recorded in archives. Missing inertias are supported: construction uses only
    the validated kinematic chain. Stationary base inertias are unobservable.
    """

    def __init__(self, chain: KinematicChain, gravity: Any = (0., 0., -9.81),
                 massless_links: tuple[str, ...] = ()):
        self.chain = chain
        self.gravity = DynamicsConfig(gravity=gravity).gravity
        self.massless_links = tuple(massless_links)
        moving, links = False, []
        for joint in chain.joints:
            moving |= joint.joint_type != JointType.FIXED
            if moving:
                links.append(joint.child_link)
        if (not chain.source_sha256 or not chain.joint_names
            or len(set(massless_links)) != len(massless_links)
            or not set(massless_links).issubset(links)):
            raise ValueError("Regressor requires a source-bound moving chain and valid marker names")
        self.links = tuple(name for name in links if name not in massless_links)
        if not self.links:
            raise ValueError("At least one moving body is required")
        self.names = tuple(f"{name}.{key}" for name in self.links for key in INERTIAL_NAMES) + tuple(
            f"{name}.{key}" for name in chain.joint_names for key in ("viscous", "coulomb"))
        self.units = ("kg", "kg*m", "kg*m", "kg*m", *("kg*m^2",)*6)*len(self.links) + tuple(
            unit for j in chain.joints if j.joint_type != JointType.FIXED
            for unit in (("N*s/m", "N") if j.joint_type == JointType.PRISMATIC else ("N*m*s/rad", "N*m")))
        self._compile()

    @property
    def dof(self) -> int:
        return len(self.chain.joint_names)

    def _compile(self) -> None:
        n = self.dof
        q, v, a = (ca.SX.sym(k, n) for k in ("q", "v", "a"))
        p = ca.SX.sym("parameters", len(self.names))
        pose, kinetic, potential = ca.SX.eye(4), ca.SX(0), ca.SX(0)
        axes: list[Any] = []
        index = 0
        for joint in self.chain.joints:
            pose = pose @ ca.DM(joint.origin)
            if joint.joint_type != JointType.FIXED:
                axis, motion = ca.DM(joint.axis), ca.SX.eye(4)
                if joint.joint_type == JointType.PRISMATIC:
                    axes.append(ca.SX.zeros(3, 1))
                    motion[:3, 3] = axis*q[index]
                else:
                    axes.append(pose[:3, :3] @ axis)
                    skew = ca.skew(axis)
                    motion[:3, :3] = ca.SX.eye(3)+ca.sin(q[index])*skew+(1-ca.cos(q[index]))*skew@skew
                pose = pose @ motion
                index += 1
            if joint.child_link not in self.links:
                continue
            k = 10*self.links.index(joint.child_link)
            b = p[k:k+10]
            tensor = ca.vertcat(ca.horzcat(b[4], b[7], b[8]),
                               ca.horzcat(b[7], b[5], b[9]), ca.horzcat(b[8], b[9], b[6]))
            rotation, position = pose[:3, :3], pose[:3, 3]
            linear = ca.jacobian(position, q) @ v
            omega = ca.horzcat(*(axes + [ca.SX.zeros(3, 1)]*(n-index))) @ v
            h_world = rotation @ b[1:4]
            kinetic += (b[0]*ca.dot(linear, linear)/2 + ca.dot(linear, ca.cross(omega, h_world))
                        + ca.mtimes([omega.T, rotation, tensor, rotation.T, omega])/2)
            potential -= ca.dot(ca.DM(self.gravity), b[0]*position+h_world)
        momentum = ca.gradient(kinetic, v)
        effort = ca.jacobian(momentum, v)@a + ca.jacobian(momentum, q)@v - ca.gradient(kinetic, q) + ca.gradient(potential, q)
        for i in range(n):
            k = 10*len(self.links)+2*i
            effort[i] += p[k]*v[i] + p[k+1]*ca.sign(v[i])
        self._function = ca.Function("inertial_regressor", [q, v, a], [ca.jacobian(effort, p)])

    def matrix(self, q: Any, velocity: Any, acceleration: Any) -> np.ndarray:
        vectors = [np.asarray(x, dtype=float) for x in (q, velocity, acceleration)]
        if any(x.shape != (self.dof,) or not np.isfinite(x).all() for x in vectors):
            raise ValueError("Regressor states must be finite joint vectors")
        result = np.asarray(self._function(*vectors))
        if not np.isfinite(result).all():
            raise ValueError("Nonfinite regressor")
        return result

    def stack(self, data: IdentificationData) -> np.ndarray:
        data.validate_chain(self.chain)
        return np.vstack([self.matrix(q, v, a) for q, v, a in zip(data.q, data.velocity, data.acceleration)])

    def parameters(self, model: DynamicModel) -> np.ndarray:
        if model.chain != self.chain or model.config.gravity != self.gravity:
            raise ValueError("Model chain or gravity differs from regressor")
        result = []
        for name in self.massless_links:
            body = model.inertials.for_link(name).properties
            if body is None or body.mass != 0:
                raise ValueError("Excluded marker must be explicitly massless")
        for name in self.links:
            body = model.inertials.for_link(name).properties
            if body is None or body.mass <= 0:
                raise ValueError("Supply positive moving-body nominal inertias or a generic prior")
            com, _, tensor = body.in_frame(IDENTITY)
            result.extend([body.mass, *(body.mass*com), tensor[0, 0], tensor[1, 1], tensor[2, 2],
                           tensor[0, 1], tensor[0, 2], tensor[1, 2]])
        friction = {f.joint_name: f for f in model.config.friction}
        for name in self.chain.joint_names:
            f = friction.get(name, JointFriction(name))
            result.extend([f.viscous, f.coulomb])
        return np.asarray(result)

    def generic_prior(self) -> np.ndarray:
        """Explicit arbitrary representative: 1kg, COM=0, diagonal inertia=.02kg*m²."""
        return np.array([1., 0., 0., 0., .02, .02, .02, 0., 0., 0.]*len(self.links)
                        + [0., 0.]*self.dof)

    def model(self, parameters: Any, provenance: str) -> DynamicModel:
        p = np.asarray(parameters, dtype=float)
        if p.shape != (len(self.names),) or not np.isfinite(p).all():
            raise ValueError("Parameter vector dimensions or values invalid")
        records = []
        for name in (self.chain.base_link, *(j.child_link for j in self.chain.joints)):
            if name in self.links:
                k = 10*self.links.index(name)
                b = p[k:k+10]
                if b[0] <= 0:
                    raise ValueError("Identified moving bodies must have positive mass")
                com = b[1:4]/b[0]
                tensor = inertia_matrix(b)-b[0]*(np.dot(com, com)*np.eye(3)-np.outer(com, com))
                pose = np.eye(4)
                pose[:3, 3] = com
                body = InertialProperties(name, b[0], tuple(map(tuple, pose)), tuple(map(tuple, tensor)))
                records.append(InertialRecord(name, "complete", body))
            elif name in self.massless_links:
                records.append(InertialRecord(name, "complete", InertialProperties(name, 0., IDENTITY,
                    ((0., 0., 0.),)*3)))
            else:
                records.append(InertialRecord(name, "missing", message="Stationary inertia is unobservable; omitted"))
        k = 10*len(self.links)
        friction = tuple(JointFriction(name, p[k+2*i], p[k+2*i+1]) for i, name in enumerate(self.chain.joint_names))
        return DynamicModel(self.chain, InertialModel(str(self.chain.source_sha256), tuple(records)),
                            DynamicsConfig(self.gravity, friction, provenance))
