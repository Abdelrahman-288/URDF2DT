"""Compatibility imports; tree selection now belongs to the headless parser layer."""

from urdf2dt.parser.robot_document import (
    RobotDocument as RobotDocument,
    resolve_mesh as resolve_mesh,
)

__all__ = ["RobotDocument", "resolve_mesh"]
