"""Standard-DH frame gauge edits with compensating adjacent-row/tool updates.

Rule names are descriptive, pending the project's report-specific R1–R3 mapping.
"""

from dataclasses import dataclass, replace
from math import atan2, hypot

from urdf2dt._transforms import (add, column, cross, dh_transform, dot, inverse, multiply,
                                 position, scale, subtract)
from urdf2dt.config import GeometryConfig
from urdf2dt.dh.classification import classify_dh_model, get_editable_params
from urdf2dt.dh.types import DHModel, DHRow, Transform, Vector, _number, _transform
from urdf2dt.kinematics import dh_frame_transforms


@dataclass(frozen=True, slots=True)
class FrameEdit:
    axial_translation: float = 0.
    axial_rotation: float = 0.

    def __post_init__(self) -> None:
        for key in ("axial_translation", "axial_rotation"):
            object.__setattr__(self, key, _number(getattr(self, key), key))


@dataclass(frozen=True, slots=True)
class FrameIssue:
    rule: str
    message: str


@dataclass(frozen=True, slots=True)
class FrameValidation:
    issues: tuple[FrameIssue, ...] = ()

    @property
    def valid(self) -> bool:
        return not self.issues


class FrameEditError(ValueError):
    def __init__(self, issues: tuple[FrameIssue, ...]):
        self.issues = issues
        super().__init__("; ".join(f"{i.rule}: {i.message}" for i in issues))


def common_normal_points(first: Transform, second: Transform,
                         config: GeometryConfig | None = None) -> tuple[Vector, Vector]:
    """Return closest P/Q on directed z lines; parallel P uses the first origin."""
    from urdf2dt.dh.classification import classify_axis_pair
    first, second = _transform(first), _transform(second)
    a, b, u, v = position(first), position(second), column(first, 2), column(second, 2)
    case = classify_axis_pair(a, u, b, v, config)
    if case.requires_review:
        raise FrameEditError((FrameIssue("ill_conditioned_axes", "Common normal requires geometric review."),))
    normal = cross(u, v)
    if case.sine_angle <= 1e-14:
        return a, add(b, scale(v, dot(subtract(a, b), v)))
    t = dot(cross(subtract(b, a), u), normal) / (case.sine_angle ** 2)
    q = add(b, scale(v, t))
    return add(a, scale(u, dot(subtract(q, a), u))), q


def recompute_dh_row(previous: Transform, following: Transform, template: DHRow) -> DHRow:
    """Factor a relative zero-pose transform, preserving joint metadata and sign."""
    relative = multiply(inverse(_transform(previous)), _transform(following))
    theta = atan2(relative[1][0], relative[0][0])
    row = replace(template, a=dot(position(relative), column(relative, 0)),
                  d=relative[2][3], theta_offset=theta,
                  alpha=atan2(relative[2][1], relative[2][2]))
    expected = dh_transform(row.a, row.alpha, row.d, row.theta_offset)
    if any(abs(x-y) > 1e-9 for a, b in zip(relative, expected) for x, y in zip(a, b)):
        raise FrameEditError((FrameIssue("dh_factorization", "Frames cannot be related by a Standard-DH row."),))
    return row


def validate_frame(previous: Transform, original: Transform, candidate: Transform,
                   config: GeometryConfig | None = None) -> FrameValidation:
    """Check rigidity, unchanged directed joint line and common-normal conditions."""
    settings = config if config is not None else GeometryConfig()
    try:
        previous, original, candidate = (_transform(t) for t in (previous, original, candidate))
    except (ValueError, TypeError) as exc:
        return FrameValidation((FrameIssue("rigid_frame", str(exc)),))
    issues = []
    z, old_z = column(candidate, 2), column(original, 2)
    if (hypot(*subtract(z, old_z)) > 1e-9 or
            hypot(*cross(subtract(position(candidate), position(original)), old_z))
            > min(settings.common_normal_threshold, 1e-9)):
        issues.append(FrameIssue("joint_axis", "Candidate must preserve the directed joint axis line."))
    x, prior_z = column(candidate, 0), column(previous, 2)
    if abs(dot(x, prior_z)) > 1e-9:
        issues.append(FrameIssue("common_normal", "Candidate x must be perpendicular to both joint axes."))
    try:
        recompute_dh_row(previous, candidate, DHRow(0., 0., 0., 0., "validation"))
    except FrameEditError as exc:
        issues.extend(exc.issues)
    return FrameValidation(tuple(issues))


def replace_frame(model: DHModel, frame_index: int, candidate: Transform,
                  config: GeometryConfig | None = None) -> DHModel:
    """Preserve all other world frames by recomputing two rows or the terminal tool."""
    if type(frame_index) is not int or not 1 <= frame_index <= len(model.rows):
        raise ValueError("frame_index must be within F1..Fn")
    poses = dh_frame_transforms(model, (0.,) * len(model.rows))
    check = validate_frame(poses[frame_index-1], poses[frame_index], candidate, config)
    if not check.valid:
        raise FrameEditError(check.issues)
    rows = list(model.rows)
    rows[frame_index-1] = recompute_dh_row(poses[frame_index-1], candidate, rows[frame_index-1])
    tool = model.tool_transform
    if frame_index < len(rows):
        rows[frame_index] = recompute_dh_row(candidate, poses[frame_index+1], rows[frame_index])
    else:
        tool = multiply(multiply(inverse(candidate), poses[-1]), tool)
    return replace(model, rows=tuple(rows), tool_transform=tool)


def recompute_model(model: DHModel, frame_index: int, edit: FrameEdit,
                    config: GeometryConfig | None = None) -> DHModel:
    """Apply only classifier-authorized controls; no clamping or arbitrary dragging."""
    if type(frame_index) is not int or not 1 <= frame_index <= len(model.rows):
        raise ValueError("frame_index must be within F1..Fn")
    if not isinstance(edit, FrameEdit):
        raise TypeError("edit must be FrameEdit")
    allowed = {p.name for p in get_editable_params(classify_dh_model(model, config)[frame_index-1])}
    for name in ("axial_translation", "axial_rotation"):
        if getattr(edit, name) != 0 and name not in allowed:
            raise FrameEditError((FrameIssue("edit_space", f"{name} is not permitted for this axis pair."),))
    if edit == FrameEdit():
        return model
    poses = dh_frame_transforms(model, (0.,) * len(model.rows))
    gauge = dh_transform(0., 0., edit.axial_translation, edit.axial_rotation)
    return replace_frame(model, frame_index, multiply(poses[frame_index], gauge), config)
