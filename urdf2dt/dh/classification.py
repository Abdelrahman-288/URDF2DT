"""Tolerance-aware infinite-axis classification, independent of rendering.

Descriptive cases do not assert the unavailable project report's A/B1/B2 labels.
Edit parameters are local frame freedoms, not permission to change DH rows alone.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from math import hypot, isfinite
from sys import float_info
import logging

from urdf2dt._transforms import column, cross, dot, position, scale, subtract
from urdf2dt.config import GeometryConfig
from urdf2dt.dh.types import AxisCase, DHModel, Vector, _number
from urdf2dt.kinematics import dh_frame_transforms

_ROUND_OFF = 64 * float_info.epsilon


@dataclass(frozen=True, slots=True)
class AxisClassification:
    case: AxisCase
    sine_angle: float
    distance_m: float
    coincident: bool
    requires_review: bool
    description: str


@dataclass(frozen=True, slots=True)
class EditableParameter:
    name: str
    unit: str
    description: str


def _point(values: Sequence[float]) -> Vector:
    result = tuple(_number(v, "axis coordinate") for v in values)
    if len(result) != 3:
        raise ValueError("axis points and directions require three coordinates")
    return result


def _direction(values: Sequence[float]) -> Vector:
    vector = _point(values)
    largest = max(abs(v) for v in vector)
    if largest == 0:
        raise ValueError("axis direction must be nonzero")
    scaled = tuple(v / largest for v in vector)
    return scale(scaled, 1 / hypot(*scaled))


def classify_axis_pair(origin_a: Sequence[float], direction_a: Sequence[float],
                       origin_b: Sequence[float], direction_b: Sequence[float],
                       config: GeometryConfig | None = None) -> AxisClassification:
    """Classify lines using normalized cross magnitude and shortest distance.

    Threshold equality belongs to parallel/intersecting. For near-parallel lines
    distance is the symmetric transverse separation of supplied anchors, not the
    unstable remote closest-point distance. Such results never unlock edits.
    """
    settings = config if config is not None else GeometryConfig()
    a, b = _point(origin_a), _point(origin_b)
    u, v = _direction(direction_a), _direction(direction_b)
    delta = subtract(b, a)
    normal = cross(u, v)
    sine = hypot(*normal)
    if not all(isfinite(x) for x in delta):
        raise ValueError("axis separation exceeds finite precision")
    if sine <= settings.parallel_threshold:
        distance = max(hypot(*cross(delta, u)), hypot(*cross(delta, v)))
        coincident = distance <= settings.common_normal_threshold
        review = sine > _ROUND_OFF or (coincident and distance > _ROUND_OFF)
        case = AxisCase.PARALLEL
        description = ("Coincident axes: axial translation and rotation are local frame freedoms."
                       if coincident else "Parallel axes: the common normal can slide along the axes.")
    else:
        distance = abs(dot(delta, scale(normal, 1 / sine)))
        coincident = False
        case = AxisCase.INTERSECTING if distance <= settings.intersection_threshold else AxisCase.SKEW
        review = case == AxisCase.INTERSECTING and distance > _ROUND_OFF
        description = ("Intersecting axes: the intersection fixes the origin; no continuous frame freedom."
                       if case == AxisCase.INTERSECTING else
                       "Skew axes: the unique common normal fixes the frame; no continuous frame freedom.")
    if not isfinite(distance):
        raise ValueError("axis distance exceeds finite precision")
    if review:
        description = "Tolerance-based classification; edits locked pending geometric review. " + description
    logging.getLogger(__name__).debug("Axis classification: %s distance=%g review=%s", case.value, distance, review)
    return AxisClassification(case, sine, distance, coincident, review, description)


def get_editable_params(case: AxisCase | AxisClassification) -> tuple[EditableParameter, ...]:
    """Continuous local freedoms with fixed z directions; no discrete x flips.

    Prefer a full classification: it handles coincidence and tolerance locking.
    A bare PARALLEL case exposes only the freedom shared by all parallel pairs.
    Applying these controls still requires refactorization and FK validation.
    """
    result = case if isinstance(case, AxisClassification) else None
    category = result.case if result is not None else case
    if not isinstance(category, AxisCase):
        raise TypeError("expected AxisCase or AxisClassification")
    if result is not None and result.requires_review:
        return ()
    if category != AxisCase.PARALLEL:
        return ()
    params: tuple[EditableParameter, ...] = (
        EditableParameter("axial_translation", "m", "Slide the frame origin along its z axis."),)
    if result is not None and result.coincident:
        params += (EditableParameter("axial_rotation", "rad", "Rotate x and y around the common z axis."),)
    return params


def classify_dh_model(model: DHModel, config: GeometryConfig | None = None) -> tuple[AxisClassification, ...]:
    """Classify zero-pose z_(i-1)/z_i for each DH row, including the synthetic terminal z.

    The final pair is a solver frame convention, not an extra physical joint.
    """
    frames = dh_frame_transforms(model, (0.,) * len(model.rows))
    return tuple(classify_axis_pair(position(a), column(a, 2), position(b), column(b, 2), config)
                 for a, b in zip(frames, frames[1:]))
