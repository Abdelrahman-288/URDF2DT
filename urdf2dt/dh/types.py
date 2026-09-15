"""Immutable domain values for Standard-DH modeling; SI units, column vectors.

Tuples own numeric data, including copies of supplied NumPy arrays. Frozen values
are protected against ordinary mutation, not deliberate object.__setattr__ use.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from numbers import Real
from typing import Iterable

from urdf2dt.config import EditorConfig
from urdf2dt._numeric import REPRESENTATION_ATOL as REPRESENTATION_ATOL

Vector = tuple[float, ...]
Transform = tuple[Vector, ...]
IDENTITY: Transform = ((1., 0., 0., 0.), (0., 1., 0., 0.),
                       (0., 0., 1., 0.), (0., 0., 0., 1.))


def _number(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(value):
        raise ValueError(f"{name} must be a finite real number")
    return float(value)


def _text(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


def _digest(value: str | None) -> None:
    if value is not None and (not isinstance(value, str) or len(value) != 64
                              or any(c not in "0123456789abcdef" for c in value)):
        raise ValueError("source_sha256 must be a lowercase SHA-256 digest or None")


def _index(value: int, name: str = "frame_index") -> None:
    if type(value) is not int or value < 1:
        raise ValueError(f"{name} must be a one-based positive integer")


def _vector(values: Iterable[float], name: str) -> Vector:
    return tuple(_number(v, name) for v in values)


def _transform(values: Iterable[Iterable[float]]) -> Transform:
    result = tuple(_vector(row, "transform") for row in values)
    if len(result) != 4 or any(len(row) != 4 for row in result):
        raise ValueError("transform must be 4x4")
    if any(abs(a - b) > REPRESENTATION_ATOL for a, b in zip(result[3], (0, 0, 0, 1))):
        raise ValueError("transform bottom row must be [0, 0, 0, 1]")
    orthogonal = all(
        abs(sum(result[k][i] * result[k][j] for k in range(3)) - int(i == j))
        <= REPRESENTATION_ATOL for i in range(3) for j in range(3)
    )
    a, b, c = (row[:3] for row in result[:3])
    determinant = (a[0] * (b[1]*c[2] - b[2]*c[1])
                   - a[1] * (b[0]*c[2] - b[2]*c[0])
                   + a[2] * (b[0]*c[1] - b[1]*c[0]))
    if not orthogonal or abs(determinant - 1) > REPRESENTATION_ATOL:
        raise ValueError("transform must contain a right-handed orthonormal rotation")
    return result


class AxisCase(str, Enum):
    """Descriptive cases; reference A/B1/B2 mappings are not yet established."""

    PARALLEL = "parallel"
    INTERSECTING = "intersecting"
    SKEW = "skew"


class JointType(str, Enum):
    """URDF joint kinds supported by the proposed serial-chain contract."""

    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"


class FrameState(str, Enum):
    """Snapshot statuses; transition enforcement belongs to EditorSession."""

    LOCKED = "locked"
    EDITABLE = "editable"
    ACCEPTED = "accepted"
    INVALIDATED = "invalidated"


@dataclass(frozen=True, slots=True)
class JointLimit:
    """Finite position bounds, in radians or meters according to joint type."""

    lower: float
    upper: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "lower", _number(self.lower, "lower"))
        object.__setattr__(self, "upper", _number(self.upper, "upper"))
        if self.lower > self.upper:
            raise ValueError("lower limit must not exceed upper limit")


@dataclass(frozen=True, slots=True)
class Joint:
    """Origin maps the zero child/joint frame into the parent; axis is joint-local."""

    name: str
    parent_link: str
    child_link: str
    joint_type: JointType
    origin: Transform = IDENTITY
    axis: Vector = (0., 0., 1.)
    limit: JointLimit | None = None

    def __post_init__(self) -> None:
        for name in ("name", "parent_link", "child_link"):
            _text(getattr(self, name), name)
        if self.parent_link == self.child_link:
            raise ValueError("a joint cannot connect a link to itself")
        object.__setattr__(self, "joint_type", JointType(self.joint_type))
        object.__setattr__(self, "origin", _transform(self.origin))
        axis = _vector(self.axis, "axis")
        if len(axis) != 3 or abs(sum(v*v for v in axis) - 1) > REPRESENTATION_ATOL:
            raise ValueError("axis must have three components and unit length")
        object.__setattr__(self, "axis", axis)
        if self.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
            if not isinstance(self.limit, JointLimit):
                raise ValueError("revolute/prismatic joints require finite position limits")
        elif self.limit is not None:
            raise ValueError("fixed/continuous joints do not have finite position limits")


@dataclass(frozen=True, slots=True)
class KinematicChain:
    """Single connected acyclic base-to-tip path, retaining fixed joints."""

    robot_name: str
    base_link: str
    tip_link: str
    joints: tuple[Joint, ...]
    source_urdf: str
    source_sha256: str | None = None

    def __post_init__(self) -> None:
        _digest(self.source_sha256)
        for name in ("robot_name", "base_link", "tip_link", "source_urdf"):
            _text(getattr(self, name), name)
        joints = tuple(self.joints)
        object.__setattr__(self, "joints", joints)
        if not joints or not all(isinstance(j, Joint) for j in joints):
            raise ValueError("chain requires Joint values")
        names: set[str] = set()
        links = {self.base_link}
        parent = self.base_link
        for joint in joints:
            if joint.parent_link != parent or joint.child_link in links:
                raise ValueError("joints must form an ordered acyclic serial path")
            if joint.name in names:
                raise ValueError("joint names must be unique")
            names.add(joint.name)
            links.add(joint.child_link)
            parent = joint.child_link
        if parent != self.tip_link:
            raise ValueError("last joint must terminate at tip_link")

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Movable joint names in q-vector order."""
        return tuple(j.name for j in self.joints if j.joint_type != JointType.FIXED)


@dataclass(frozen=True, slots=True)
class DHRow:
    """Standard DH: Rz(theta) Tz(d) Tx(a) Rx(alpha); one movable joint per row."""

    a: float
    alpha: float
    d: float
    theta_offset: float
    joint_name: str
    joint_type: JointType = JointType.REVOLUTE
    joint_sign: int = 1

    def __post_init__(self) -> None:
        for name in ("a", "alpha", "d", "theta_offset"):
            object.__setattr__(self, name, _number(getattr(self, name), name))
        _text(self.joint_name, "joint_name")
        object.__setattr__(self, "joint_type", JointType(self.joint_type))
        if self.joint_type == JointType.FIXED:
            raise ValueError("fixed joints must be absorbed into DH geometry/transforms")
        if type(self.joint_sign) is not int or self.joint_sign not in (-1, 1):
            raise ValueError("joint_sign must be +1 or -1")


@dataclass(frozen=True, slots=True)
class DHModel:
    """Immutable DH baseline or working value with explicit source provenance."""

    robot_name: str
    rows: tuple[DHRow, ...]
    provenance: str
    source_urdf: str | None = None
    is_temporary_fixture: bool = False
    base_transform: Transform = IDENTITY
    tool_transform: Transform = IDENTITY
    source_sha256: str | None = None

    def __post_init__(self) -> None:
        _digest(self.source_sha256)
        _text(self.robot_name, "robot_name")
        _text(self.provenance, "provenance")
        if self.source_urdf is not None:
            _text(self.source_urdf, "source_urdf")
        if type(self.is_temporary_fixture) is not bool:
            raise ValueError("is_temporary_fixture must be boolean")
        rows = tuple(self.rows)
        if not rows or not all(isinstance(row, DHRow) for row in rows):
            raise ValueError("model requires DHRow values")
        if len({row.joint_name for row in rows}) != len(rows):
            raise ValueError("DH joint names must be unique")
        object.__setattr__(self, "rows", rows)
        for name in ("base_transform", "tool_transform"):
            object.__setattr__(self, name, _transform(getattr(self, name)))

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Joint names in DH and q-vector order."""
        return tuple(row.joint_name for row in self.rows)


@dataclass(frozen=True, slots=True)
class EditRecord:
    """Accepted or rejected proposal; deterministic sequence, one-based frame."""

    sequence: int
    frame_index: int
    before: DHRow
    proposed: DHRow
    accepted: bool
    reason: str

    def __post_init__(self) -> None:
        _index(self.sequence, "sequence")
        _index(self.frame_index)
        if not isinstance(self.before, DHRow) or not isinstance(self.proposed, DHRow):
            raise ValueError("edit rows must be DHRow values")
        if ((self.before.joint_name, self.before.joint_type, self.before.joint_sign)
                != (self.proposed.joint_name, self.proposed.joint_type, self.proposed.joint_sign)):
            raise ValueError("edit cannot change joint identity, type or sign")
        if type(self.accepted) is not bool:
            raise ValueError("accepted must be boolean")
        _text(self.reason, "reason")


@dataclass(frozen=True, slots=True)
class EditorState:
    """Immutable session snapshot; this type does not implement transitions."""

    automatic_model: DHModel
    working_model: DHModel
    frames: tuple[FrameState, ...]
    history: tuple[EditRecord, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.automatic_model, DHModel) or not isinstance(self.working_model, DHModel):
            raise ValueError("session models must be DHModel values")
        auto, work = self.automatic_model, self.working_model
        def identities(model: DHModel) -> tuple[tuple[str, JointType, int], ...]:
            return tuple((r.joint_name, r.joint_type, r.joint_sign) for r in model.rows)
        if auto.robot_name != work.robot_name or identities(auto) != identities(work):
            raise ValueError("automatic and working model joint identities must match")
        frames = tuple(FrameState(f) for f in self.frames)
        history = tuple(self.history)
        if len(frames) != len(auto.rows):
            raise ValueError("one frame status is required per DH row")
        for i, edit in enumerate(history, 1):
            if not isinstance(edit, EditRecord) or edit.sequence != i:
                raise ValueError("history must have consecutive one-based sequence numbers")
            if edit.frame_index > len(frames):
                raise ValueError("edit frame is outside model")
            row = auto.rows[edit.frame_index - 1]
            if (edit.before.joint_name, edit.before.joint_type, edit.before.joint_sign) != (
                    row.joint_name, row.joint_type, row.joint_sign):
                raise ValueError("edit must reference its indexed joint")
        object.__setattr__(self, "frames", frames)
        object.__setattr__(self, "history", history)


@dataclass(frozen=True, slots=True)
class ValidationSample:
    """One q configuration and measured end-effector errors (not computed here)."""

    q: Vector
    position_error_m: float
    orientation_error_rad: float

    def __post_init__(self) -> None:
        q = _vector(self.q, "q")
        if not q:
            raise ValueError("q must not be empty")
        object.__setattr__(self, "q", q)
        for name in ("position_error_m", "orientation_error_rad"):
            value = _number(getattr(self, name), name)
            if value < 0:
                raise ValueError("validation errors must be nonnegative")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class ValidationResult:
    """Measured sample collection; PASS is derived, never a caller-supplied flag.

    This data type does not establish where the measurements came from. The
    global validator must supply genuine URDF/DH comparisons in Stage 11.
    """

    joint_names: tuple[str, ...]
    samples: tuple[ValidationSample, ...]
    config: EditorConfig

    def __post_init__(self) -> None:
        names, samples = tuple(self.joint_names), tuple(self.samples)
        for name in names:
            _text(name, "joint_name")
        if not names or len(set(names)) != len(names):
            raise ValueError("joint names must be nonempty and unique")
        if not isinstance(self.config, EditorConfig):
            raise ValueError("config must be EditorConfig")
        if len(samples) != self.config.validation.samples:
            raise ValueError("sample count must match effective configuration")
        if not all(isinstance(s, ValidationSample) and len(s.q) == len(names) for s in samples):
            raise ValueError("samples must match joint order dimensions")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "samples", samples)

    @property
    def max_position_error_m(self) -> float:
        """Worst sampled position error in meters."""
        return max(s.position_error_m for s in self.samples)

    @property
    def max_orientation_error_rad(self) -> float:
        """Worst sampled SO(3) geodesic orientation error in radians."""
        return max(s.orientation_error_rad for s in self.samples)

    @property
    def passed(self) -> bool:
        """Whether every supplied sample is within the recorded tolerances."""
        return (self.max_position_error_m <= self.config.validation.position_tolerance_m
                and self.max_orientation_error_rad <= self.config.validation.orientation_tolerance_rad)
