"""TEMPORARY classic UR5 DH-only fixture, NOT automatic solver/URDF output.

Source: Universal Robots, nominal UR5 kinematics table, retrieved 2026-09-13:
https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics

Missing MATLAB and URDF references prevent frame-alignment validation. Synthetic
joint names, zero offsets and identity base/tool transforms are local conventions.
Stage 5 uses real solver output in production. This fixture remains only as an
independent nominal-table reference for regression tests, never as a fallback.
"""

from math import pi

from urdf2dt.dh.types import DHModel, DHRow


def make_ur5_fixture() -> DHModel:
    """Return fresh immutable nominal dimensions; do not claim FK equivalence."""
    parameters = (
        (0., pi / 2, 0.089159),
        (-0.425, 0., 0.),
        (-0.39225, 0., 0.),
        (0., pi / 2, 0.10915),
        (0., -pi / 2, 0.09465),
        (0., 0., 0.0823),
    )
    return DHModel(
        robot_name="UR5",
        rows=tuple(DHRow(a, alpha, d, 0., f"fixture_joint_{i}")
                   for i, (a, alpha, d) in enumerate(parameters, 1)),
        provenance="Temporary nominal UR5 table from Universal Robots; URDF/MATLAB alignment unverified.",
        is_temporary_fixture=True,
    )
