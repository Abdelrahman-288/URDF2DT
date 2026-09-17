"""Energy-based dynamics with exact CasADi differentiation, without simulation."""

from dataclasses import dataclass
from typing import Any, Sequence
import numpy as np
import casadi as ca

VectorInput = Sequence[float] | np.ndarray

from urdf2dt.dh.types import KinematicChain, JointType
from urdf2dt.parser.inertial_extractor import InertialModel


@dataclass(frozen=True)
class JointFriction:
    """Viscous + Coulomb kinetic friction; sign(0)=0, no static-friction solver."""

    joint_name: str
    viscous: float = 0.
    coulomb: float = 0.

    def __post_init__(self) -> None:
        if not self.joint_name or any(not np.isfinite(v) or v < 0 for v in (self.viscous, self.coulomb)):
            raise ValueError("Friction coefficients must be finite and nonnegative")


@dataclass(frozen=True)
class DynamicsConfig:
    """Gravity in the chain base frame, in m/s²; parameter-source label is required."""

    gravity: tuple[float, ...] = (0., 0., -9.81)
    friction: tuple[JointFriction, ...] = ()
    parameter_provenance: str = "URDF inertias; nominal, not hardware calibrated"

    def __post_init__(self) -> None:
        gravity = tuple(float(v) for v in self.gravity)
        if len(gravity) != 3 or not np.isfinite(gravity).all() or not self.parameter_provenance.strip():
            raise ValueError("Supply three finite gravity components and parameter provenance")
        object.__setattr__(self, "gravity", gravity)
        object.__setattr__(self, "friction", tuple(self.friction))
        names = [f.joint_name for f in self.friction]
        if len(names) != len(set(names)):
            raise ValueError("Duplicate friction joint names")


class DynamicModel:
    """Immutable input contracts, compiled M/c/g, and forward/inverse evaluators.

    This model consumes the original link frames, so legal DH editor changes do
    not change any inertias. Fixed child links contribute their own mass/inertia.
    Position/effort limits are metadata: these evaluators do not enforce control
    constraints. No contact, external wrench, motor gearing or elastic dynamics.
    """

    def __init__(self, chain: KinematicChain, inertials: InertialModel,
                 config: DynamicsConfig = DynamicsConfig()):
        if not chain.source_sha256 or chain.source_sha256 != inertials.source_sha256:
            raise ValueError("Kinematics and inertias must originate from the same URDF snapshot")
        if not chain.joint_names:
            raise ValueError("Dynamics requires a movable joint")
        links = {chain.base_link, *(j.child_link for j in chain.joints)}
        if {r.link_name for r in inertials.records} != links:
            raise ValueError("Inertial records must match every selected chain link")
        if any(f.joint_name not in chain.joint_names for f in config.friction):
            raise ValueError("Friction refers to an unknown or fixed joint")
        moving = False
        for joint in chain.joints:
            moving |= joint.joint_type != JointType.FIXED
            record = inertials.for_link(joint.child_link)
            if moving and (record.status != "complete" or record.properties is None):
                raise ValueError(f"Dynamics unavailable: {record.link_name}: {record.status}: {record.message}")
        self._chain, self._inertials, self._config = chain, inertials, config
        self._compile()
        self.mass_matrix(np.zeros(self.dof))  # reject massless/singular mechanisms early

    @property
    def chain(self) -> KinematicChain:
        return self._chain

    @property
    def inertials(self) -> InertialModel:
        return self._inertials

    @property
    def config(self) -> DynamicsConfig:
        return self._config

    @property
    def dof(self) -> int:
        return len(self.chain.joint_names)

    def _compile(self) -> None:
        n = self.dof
        q, v = ca.SX.sym("q", n), ca.SX.sym("v", n)
        pose = ca.SX.eye(4)
        kinetic, potential = ca.SX(0), ca.SX(0)
        angular_axes: list[Any] = []
        index = 0
        for joint in self.chain.joints:
            pose = pose @ ca.DM(joint.origin)
            if joint.joint_type != JointType.FIXED:
                axis = ca.DM(joint.axis)
                motion = ca.SX.eye(4)
                if joint.joint_type == JointType.PRISMATIC:
                    angular_axes.append(ca.SX.zeros(3, 1))
                    motion[:3, 3] = axis * q[index]
                else:
                    angular_axes.append(pose[:3, :3] @ axis)
                    skew = ca.skew(axis)
                    motion[:3, :3] = ca.SX.eye(3) + ca.sin(q[index])*skew + (1-ca.cos(q[index]))*(skew @ skew)
                pose = pose @ motion
                index += 1
            record = self.inertials.for_link(joint.child_link)
            body = record.properties
            if body is None or body.mass == 0 or index == 0:
                continue
            com_pose = pose @ ca.DM(body.origin)
            com = com_pose[:3, 3]
            rotation = com_pose[:3, :3]
            inertia = rotation @ ca.DM(body.tensor) @ rotation.T
            linear = ca.jacobian(com, q) @ v
            omega = ca.horzcat(*(angular_axes + [ca.SX.zeros(3, 1)] * (n-index))) @ v
            kinetic += (body.mass * ca.dot(linear, linear) + ca.mtimes([omega.T, inertia, omega]))/2
            potential -= body.mass * ca.dot(ca.DM(self.config.gravity), com)
        momentum = ca.gradient(kinetic, v)
        mass = ca.jacobian(momentum, v)
        coriolis = ca.jacobian(momentum, q) @ v - ca.gradient(kinetic, q)
        gravity = ca.gradient(potential, q)
        self._terms = ca.Function("dynamics", [q, v], [mass, coriolis, gravity, kinetic, potential])
        self._pose = ca.Function("tip", [q], [pose])

    def _vector(self, value: VectorInput, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (self.dof,) or not np.isfinite(array).all():
            raise ValueError(f"{name} must be a finite vector of length {self.dof}")
        return array

    def mass_matrix(self, q: VectorInput) -> np.ndarray:
        matrix = np.asarray(self._terms(self._vector(q, "q"), np.zeros(self.dof))[0])
        if not np.isfinite(matrix).all():
            raise ValueError("Nonfinite mass matrix")
        try:
            np.linalg.cholesky(matrix)
        except np.linalg.LinAlgError as exc:
            raise ValueError("Mass matrix is not positive definite") from exc
        return matrix

    def coriolis(self, q: VectorInput, velocity: VectorInput) -> np.ndarray:
        """Return the unique velocity bias c=C(q,v)v, rather than a nonunique C matrix."""
        return np.asarray(self._terms(self._vector(q, "q"), self._vector(velocity, "velocity"))[1]).ravel()

    def gravity_effort(self, q: VectorInput) -> np.ndarray:
        return np.asarray(self._terms(self._vector(q, "q"), np.zeros(self.dof))[2]).ravel()

    def friction_effort(self, velocity: VectorInput) -> np.ndarray:
        v = self._vector(velocity, "velocity")
        result = np.zeros(self.dof)
        for f in self.config.friction:
            i = self.chain.joint_names.index(f.joint_name)
            result[i] = f.viscous * v[i] + f.coulomb * np.sign(v[i])
        return result

    def inverse_dynamics(self, q: VectorInput, velocity: VectorInput,
                         acceleration: VectorInput) -> np.ndarray:
        """Generalized actuator effort: N·m for revolute, N for prismatic joints."""
        return (self.mass_matrix(q) @ self._vector(acceleration, "acceleration")
                + self.coriolis(q, velocity) + self.gravity_effort(q) + self.friction_effort(velocity))

    def forward_dynamics(self, q: VectorInput, velocity: VectorInput,
                         effort: VectorInput) -> np.ndarray:
        rhs = (self._vector(effort, "effort") - self.coriolis(q, velocity)
               - self.gravity_effort(q) - self.friction_effort(velocity))
        return np.linalg.solve(self.mass_matrix(q), rhs)

    def energy(self, q: VectorInput, velocity: VectorInput) -> tuple[float, float]:
        terms = self._terms(self._vector(q, "q"), self._vector(velocity, "velocity"))
        return float(terms[3]), float(terms[4])

    def tip_transform(self, q: VectorInput) -> np.ndarray:
        return np.asarray(self._pose(self._vector(q, "q")))
