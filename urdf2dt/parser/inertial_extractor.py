"""Independent URDF inertias; absent or invalid data never disables kinematics."""

from dataclasses import dataclass
from math import isfinite
from xml.etree.ElementTree import Element
import numpy as np

from urdf2dt._transforms import rpy_transform
from urdf2dt.dh.types import Transform, _transform
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.xml_snapshot import read_snapshot


@dataclass(frozen=True)
class InertialProperties:
    """SI mass and COM-frame tensor; origin maps inertial frame into its link."""

    link_name: str
    mass: float
    origin: Transform
    tensor: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "origin", _transform(self.origin))
        tensor = np.asarray(self.tensor, dtype=float)
        if not self.link_name or not isfinite(self.mass) or self.mass < 0:
            raise ValueError("Mass must be finite and nonnegative; link name is required")
        if tensor.shape != (3, 3) or not np.isfinite(tensor).all():
            raise ValueError("Inertia must be a finite 3x3 tensor")
        scale = max(float(np.max(np.abs(tensor))), 1e-15)
        if not np.allclose(tensor, tensor.T, rtol=0, atol=1e-12 * scale):
            raise ValueError("Inertia tensor must be symmetric")
        eigenvalues = np.linalg.eigvalsh(tensor)
        if self.mass == 0:
            if np.any(tensor != 0):
                raise ValueError("An explicitly massless marker must have zero inertia")
        elif eigenvalues[0] <= 0 or eigenvalues[-1] > sum(eigenvalues[:2]) + 1e-12 * scale:
            raise ValueError("Positive mass requires positive principal inertias satisfying triangle inequalities")
        object.__setattr__(self, "mass", float(self.mass))
        object.__setattr__(self, "tensor", tuple(tuple(float(v) for v in row) for row in tensor))

    def in_frame(self, frame_from_link: Transform) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """COM, COM inertia, and inertia about a new frame origin (parallel-axis theorem)."""
        pose = np.asarray(_transform(frame_from_link)) @ np.asarray(self.origin)
        com = pose[:3, 3]
        inertia = pose[:3, :3] @ np.asarray(self.tensor) @ pose[:3, :3].T
        about_origin = inertia + self.mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
        return com, inertia, about_origin


@dataclass(frozen=True)
class InertialRecord:
    """One link's independently reported availability and diagnostic."""

    link_name: str
    status: str
    properties: InertialProperties | None = None
    message: str = ""

    def __post_init__(self) -> None:
        if self.status not in {"complete", "missing", "incomplete", "invalid"}:
            raise ValueError("Unknown inertial availability status")
        if self.status == "complete":
            if self.properties is None or self.properties.link_name != self.link_name:
                raise ValueError("Complete inertial records must match their owning link")
        elif self.properties is not None:
            raise ValueError("Unavailable records cannot carry usable properties")


@dataclass(frozen=True)
class InertialModel:
    """Immutable link-keyed records tied to the exact source snapshot."""

    source_sha256: str
    records: tuple[InertialRecord, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "records", tuple(self.records))
        names = [record.link_name for record in self.records]
        if len(names) != len(set(names)):
            raise ValueError("Inertial link names must be unique")

    def for_link(self, name: str) -> InertialRecord:
        return next(record for record in self.records if record.link_name == name)


def _values(raw: str, count: int) -> tuple[float, ...]:
    values = tuple(float(v) for v in raw.split())
    if len(values) != count or not all(isfinite(v) for v in values):
        raise ValueError(f"Expected {count} finite numeric values")
    return values


def _single(element: Element, tag: str) -> Element | None:
    found = element.findall(tag)
    if len(found) > 1:
        raise ValueError(f"Multiple <{tag}> elements")
    return found[0] if found else None


def extract_inertials(source: URDFInput) -> InertialModel:
    """Extract all links without kinematic validation, mesh access, or DH generation."""
    root = read_snapshot(source)
    records = []
    for link in root.findall("link"):
        name = link.get("name", "")
        if not name.strip():
            raise ValueError("Inertial records require named links")
        try:
            inertial = _single(link, "inertial")
            if inertial is None:
                records.append(InertialRecord(name, "missing", message="No inertial element supplied"))
                continue
            mass = _single(inertial, "mass")
            tensor = _single(inertial, "inertia")
            if mass is None or tensor is None or "value" not in mass.attrib or any(
                key not in tensor.attrib for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            ):
                records.append(InertialRecord(name, "incomplete", message="Mass and all six tensor fields are required"))
                continue
            origin = _single(inertial, "origin")
            xyz = _values(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3)
            rpy = _values(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3)
            xx, xy, xz, yy, yz, zz = (_values(tensor.attrib[key], 1)[0]
                                     for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
            properties = InertialProperties(name, _values(mass.attrib["value"], 1)[0],
                rpy_transform(xyz, rpy), ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz)))
            records.append(InertialRecord(name, "complete", properties))
        except ValueError as exc:
            records.append(InertialRecord(name, "invalid", message=str(exc)))
    return InertialModel(source.sha256, tuple(records))
