"""Secure structural/kinematic-field validation for the v1 serial URDF subset.

No solver, UI, mesh resolution, xacro execution, or remote resource fetching runs
here. Returned joint fields are normalized data, not a KinematicChain or DH model.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path
from xml.etree.ElementTree import Element, ParseError  # types/errors only; never raw parsing

from defusedxml import ElementTree as SafeET
from defusedxml.common import DefusedXmlException

from urdf2dt.dh.types import JointLimit, JointType, Vector
from urdf2dt.parser.urdf_input import (
    InputPolicy, URDFInput, URDFInputError, URDFIssue, _check_name, _check_size,
)


@dataclass(frozen=True, slots=True)
class ValidatedJoint:
    """Checked XML joint fields; axis is normalized in the joint-local frame."""

    name: str
    parent: str
    child: str
    joint_type: JointType
    xyz: Vector
    rpy: Vector
    axis: Vector
    limit: JointLimit | None


@dataclass(frozen=True, slots=True)
class ValidatedURDF:
    """Accepted source snapshot and immutable base-to-tip structural summary."""

    source: URDFInput
    robot_name: str
    links: tuple[str, ...]
    joints: tuple[ValidatedJoint, ...]

    @property
    def base_link(self) -> str:
        """First link of the accepted serial path."""
        return self.links[0]

    @property
    def tip_link(self) -> str:
        """Last link of the accepted serial path."""
        return self.links[-1]

    @property
    def movable_joint_names(self) -> tuple[str, ...]:
        """Independent q-vector order; fixed joints are retained in joints."""
        return tuple(j.name for j in self.joints if j.joint_type != JointType.FIXED)


class URDFValidationError(ValueError):
    """Raised only when a caller explicitly requires a successful result."""

    def __init__(self, errors: tuple[URDFIssue, ...]):
        self.errors = errors
        super().__init__("URDF rejected: " + "; ".join(e.message for e in errors))


@dataclass(frozen=True, slots=True)
class URDFValidationResult:
    """UI-friendly structured outcome; failures never carry an accepted document."""

    document: ValidatedURDF | None = None
    errors: tuple[URDFIssue, ...] = ()
    warnings: tuple[URDFIssue, ...] = ()

    @property
    def valid(self) -> bool:
        """Whether a checked serial document is available."""
        return self.document is not None and not self.errors

    def require_valid(self) -> ValidatedURDF:
        """Return the checked snapshot or raise an error preserving issue codes."""
        if not self.valid or self.document is None:
            raise URDFValidationError(self.errors)
        return self.document


def _one(parent: Element, tag: str, errors: list[URDFIssue], context: str,
         required: bool = False) -> Element | None:
    elements = parent.findall(tag)
    if len(elements) > 1:
        errors.append(URDFIssue("duplicate_element", f"{context} has multiple <{tag}> elements.", context))
    if not elements:
        if required:
            errors.append(URDFIssue("missing_element", f"{context} requires <{tag}>.", context))
        return None
    return elements[0]


def _attributes(element: Element | None, allowed: set[str], errors: list[URDFIssue], context: str) -> None:
    if element is not None and set(element.attrib) - allowed:
        errors.append(URDFIssue("unsupported_attribute", f"{context} has unrecognized attributes: {', '.join(sorted(set(element.attrib) - allowed))}.", context))


def _numbers(raw: str | None, count: int, errors: list[URDFIssue], context: str) -> Vector | None:
    try:
        if raw is None:
            raise ValueError("missing")
        values = tuple(float(v) for v in raw.split())
        if len(values) != count or not all(isfinite(v) for v in values):
            raise ValueError("not finite or wrong count")
        return values
    except ValueError:
        errors.append(URDFIssue("invalid_number", f"{context} requires {count} finite numeric value(s).", context))
        return None


def _joint(element: Element, errors: list[URDFIssue], warnings: list[URDFIssue]) -> ValidatedJoint | None:
    start = len(errors)
    name = element.get("name", "")
    context = f"joint '{name}'"
    _attributes(element, {"name", "type"}, errors, context)
    allowed_children = {"parent", "child", "origin", "axis", "limit", "mimic", "dynamics", "calibration", "safety_controller"}
    for nested in element:
        if nested.tag not in allowed_children:
            errors.append(URDFIssue("unsupported_element", f"{context} has an unsupported <{nested.tag}> element.", context))
    try:
        kind = JointType(element.get("type", ""))
    except ValueError:
        errors.append(URDFIssue("unsupported_joint_type", f"{context} must be fixed, revolute, continuous or prismatic.", context))
        return None
    parent = _one(element, "parent", errors, context, True)
    child = _one(element, "child", errors, context, True)
    _attributes(parent, {"link"}, errors, f"{context} parent")
    _attributes(child, {"link"}, errors, f"{context} child")
    parent_name = parent.get("link", "") if parent is not None else ""
    child_name = child.get("link", "") if child is not None else ""
    for tag, reference in (("parent", parent_name), ("child", child_name)):
        if not reference.strip():
            errors.append(URDFIssue("missing_reference", f"{context} requires a {tag} link name.", context))
    if parent_name and parent_name == child_name:
        errors.append(URDFIssue("self_loop", f"{context} connects a link to itself.", context))
    if element.find("mimic") is not None:
        errors.append(URDFIssue("unsupported_mimic", f"{context} uses mimic coupling, unsupported in v1.", context))
    origin = _one(element, "origin", errors, context)
    _attributes(origin, {"xyz", "rpy"}, errors, f"{context} origin")
    xyz = _numbers(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3, errors, f"{context} origin xyz")
    rpy = _numbers(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3, errors, f"{context} origin rpy")
    axis_element = _one(element, "axis", errors, context)
    _attributes(axis_element, {"xyz"}, errors, f"{context} axis")
    # URDF's default is X, unlike DH's conventionally chosen Z joint axes.
    axis = _numbers(axis_element.get("xyz") if axis_element is not None else "1 0 0", 3, errors, f"{context} axis xyz")
    if axis is not None:
        norm = hypot(*axis)
        if norm == 0 or not isfinite(norm):
            errors.append(URDFIssue("invalid_axis", f"{context} axis must have a finite nonzero norm.", context))
        else:
            axis = tuple(v / norm for v in axis)
    bounds = _one(element, "limit", errors, context, kind in (JointType.REVOLUTE, JointType.PRISMATIC))
    _attributes(bounds, {"lower", "upper", "effort", "velocity"}, errors, f"{context} limit")
    limit: JointLimit | None = None
    if bounds is not None:
        values: dict[str, float] = {}
        for attribute in ("lower", "upper", "effort", "velocity"):
            required = kind in (JointType.REVOLUTE, JointType.PRISMATIC) or (
                kind == JointType.CONTINUOUS and attribute in ("effort", "velocity"))
            raw = bounds.get(attribute)
            if raw is not None or required:
                parsed = _numbers(raw, 1, errors, f"{context} limit {attribute}")
                if parsed is not None:
                    values[attribute] = parsed[0]
                    if attribute in ("effort", "velocity") and parsed[0] < 0:
                        errors.append(URDFIssue("invalid_limit", f"{context} {attribute} limit must be nonnegative.", context))
        if "lower" in values and "upper" in values:
            if values["lower"] > values["upper"]:
                errors.append(URDFIssue("invalid_limit", f"{context} lower limit exceeds upper limit.", context))
            elif kind in (JointType.REVOLUTE, JointType.PRISMATIC):
                limit = JointLimit(values["lower"], values["upper"])
        if kind in (JointType.FIXED, JointType.CONTINUOUS) and ("lower" in values or "upper" in values):
            warnings.append(URDFIssue("ignored_position_limits", f"{context} position bounds do not apply to {kind.value} joints.", context))
    if len(errors) != start or xyz is None or rpy is None or axis is None:
        return None
    return ValidatedJoint(name, parent_name, child_name, kind, xyz, rpy, axis, limit)


def _topology(links: tuple[str, ...], joints: list[ValidatedJoint], errors: list[URDFIssue]) -> tuple[str, ...]:
    """Iterative O(V+E) graph checks; no recursion even for malicious cycles."""
    outgoing: dict[str, list[ValidatedJoint]] = {name: [] for name in links}
    incoming = dict.fromkeys(links, 0)
    neighbors: dict[str, list[str]] = {name: [] for name in links}
    for joint in joints:
        for name, role in ((joint.parent, "parent"), (joint.child, "child")):
            if name not in incoming:
                errors.append(URDFIssue("missing_link", f"Joint '{joint.name}' references missing {role} link '{name}'.", joint.name))
        if joint.parent not in incoming or joint.child not in incoming:
            continue
        outgoing[joint.parent].append(joint)
        incoming[joint.child] += 1
        neighbors[joint.parent].append(joint.child)
        neighbors[joint.child].append(joint.parent)
    for name in links:
        if len(outgoing[name]) > 1:
            errors.append(URDFIssue("branching", f"Link '{name}' branches; v1 supports serial chains only.", name))
        if incoming[name] > 1:
            errors.append(URDFIssue("multiple_parents", f"Link '{name}' has multiple parent joints.", name))
    roots = tuple(name for name in links if incoming[name] == 0)
    if len(roots) != 1:
        errors.append(URDFIssue("invalid_root_count", f"Expected one base link, found {len(roots)}."))
    pending = deque(roots)
    degrees = incoming.copy()
    visited = 0
    while pending:
        node = pending.popleft()
        visited += 1
        for joint in outgoing[node]:
            degrees[joint.child] -= 1
            if degrees[joint.child] == 0:
                pending.append(joint.child)
    if visited != len(links):
        errors.append(URDFIssue("cycle", "A directed cycle was detected in the joint graph."))
    seen = {links[0]}
    pending = deque([links[0]])
    while pending:
        for neighbor in neighbors[pending.popleft()]:
            if neighbor not in seen:
                seen.add(neighbor)
                pending.append(neighbor)
    if len(seen) != len(links):
        errors.append(URDFIssue("disconnected", "The robot contains disconnected links or components."))
    if errors:
        return ()
    ordered = [roots[0]]
    while outgoing[ordered[-1]]:
        ordered.append(outgoing[ordered[-1]][0].child)
    return tuple(ordered)


def validate_urdf(source: str | Path | URDFInput | None,
                  policy: InputPolicy = InputPolicy()) -> URDFValidationResult:
    """Validate a path, file selection or uploaded snapshot without side effects.

    Untrusted XML always uses defusedxml with DTDs, entities and external references
    forbidden. Passing does not imply mesh/dynamics validity or DH solvability.
    """
    try:
        snapshot = source if isinstance(source, URDFInput) else URDFInput.from_path(source, policy)
        _check_name(snapshot.name)
        _check_size(len(snapshot.content), policy)
    except URDFInputError as exc:
        return URDFValidationResult(errors=(exc.issue,))
    try:
        root = SafeET.fromstring(snapshot.content, forbid_dtd=True, forbid_entities=True, forbid_external=True)
    except DefusedXmlException:
        return URDFValidationResult(errors=(URDFIssue("unsafe_xml", "DTD declarations, entities and external XML references are forbidden."),))
    except (ParseError, ValueError, LookupError):
        return URDFValidationResult(errors=(URDFIssue("malformed_xml", "The selected file is not well-formed XML."),))
    if root.tag != "robot":
        return URDFValidationResult(errors=(URDFIssue("invalid_robot_root", "URDF requires an unnamespaced <robot> root."),))
    errors: list[URDFIssue] = []
    warnings: list[URDFIssue] = []
    stack = [(root, 1)]
    count = 0
    while stack:
        element, depth = stack.pop()
        count += 1
        if count > policy.max_elements or depth > policy.max_depth:
            return URDFValidationResult(errors=(URDFIssue("xml_resource_limit", "XML exceeds the configured element count or nesting limit."),))
        if isinstance(element.tag, str) and element.tag.startswith("{http://www.ros.org/wiki/xacro}"):
            errors.append(URDFIssue("unexpanded_xacro", "Expand Xacro into plain URDF before loading."))
        if isinstance(element.tag, str) and element.tag.startswith("{http://wiki.ros.org/xacro}"):
            errors.append(URDFIssue("unexpanded_xacro", "Expand Xacro into plain URDF before loading."))
        stack.extend((child, depth + 1) for child in element)
    if root.get("version", "1.0") != "1.0":
        errors.append(URDFIssue("unsupported_version", "v1 currently validates URDF format 1.0 only."))
    robot_name = root.get("name", "")
    if not robot_name.strip():
        errors.append(URDFIssue("missing_robot_name", "The <robot> must have a nonempty name."))
    for element in root:
        if element.tag not in {"link", "joint", "material", "transmission", "gazebo", "ros2_control"}:
            errors.append(URDFIssue("unsupported_element", f"Unsupported top-level <{element.tag}>; supply expanded serial URDF."))
    if any(root.find(tag) is not None for tag in ("transmission", "gazebo", "ros2_control")):
        warnings.append(URDFIssue("ignored_extensions", "Simulation/control extensions are not executed or validated."))
    link_elements, joint_elements = root.findall("link"), root.findall("joint")
    for tag, elements in (("link", link_elements), ("joint", joint_elements)):
        names: set[str] = set()
        if not elements:
            errors.append(URDFIssue(f"no_{tag}s", f"Robot must contain at least one <{tag}>."))
        for element in elements:
            name = element.get("name", "")
            if "${" in name or "$(" in name:
                errors.append(URDFIssue("unexpanded_xacro", f"Unexpanded substitution in {tag} name.", tag))
            if not name or any(c.isspace() for c in name):
                errors.append(URDFIssue("invalid_name", f"Each {tag} requires a nonempty name without whitespace.", tag))
            if name in names:
                errors.append(URDFIssue("duplicate_name", f"Duplicate {tag} name '{name}'.", tag))
            names.add(name)
    joints = [joint for element in joint_elements if (joint := _joint(element, errors, warnings)) is not None]
    if errors:
        return URDFValidationResult(errors=tuple(errors), warnings=tuple(warnings))
    links = _topology(tuple(element.attrib["name"] for element in link_elements), joints, errors)
    if not any(joint.joint_type != JointType.FIXED for joint in joints):
        errors.append(URDFIssue("no_movable_joints", "The DH editor requires at least one movable joint."))
    if errors:
        return URDFValidationResult(errors=tuple(errors), warnings=tuple(warnings))
    by_parent = {joint.parent: joint for joint in joints}
    ordered_joints = tuple(by_parent[link] for link in links[:-1])
    return URDFValidationResult(ValidatedURDF(snapshot, robot_name, links, ordered_joints), warnings=tuple(warnings))
