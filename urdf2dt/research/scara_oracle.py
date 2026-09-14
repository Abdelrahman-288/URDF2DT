"""Independent closed-form oracle only for the original SCARA research fixture."""

from math import cos, sin


def analytic_fk(q):
    """Planar two-link geometry plus downward stroke; no production transforms."""
    shoulder, elbow, slide, wrist = q
    x = 0.1 + 0.35 * cos(0.3 + shoulder) + 0.25 * cos(0.3 + shoulder + elbow)
    y = -0.05 + 0.35 * sin(0.3 + shoulder) + 0.25 * sin(0.3 + shoulder + elbow)
    yaw = 0.3 + shoulder + elbow + wrist
    return (
        (cos(yaw), -sin(yaw), 0.0, x),
        (sin(yaw), cos(yaw), 0.0, y),
        (0.0, 0.0, 1.0, 0.12 - slide),
        (0.0, 0.0, 0.0, 1.0),
    )
