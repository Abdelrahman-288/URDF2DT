"""Conversion of a checked URDF snapshot into the immutable kinematic chain."""

from urdf2dt._transforms import rpy_transform
from urdf2dt.config import EditorConfig
from urdf2dt.dh.types import Joint, KinematicChain
from urdf2dt.parser.urdf_validator import ValidatedURDF


class SerialURDFParser:
    """Implements URDFParser; retains every fixed transform and movable joint."""

    def parse(self, source: ValidatedURDF, config: EditorConfig) -> KinematicChain:
        """Interpret normalized fields without reopening the selected file.

        Angles are never snapped using classification tolerances. Geometry/FK
        thresholds do not affect the URDF coordinate system or joint directions.
        """
        if not isinstance(source, ValidatedURDF):
            raise TypeError("parse requires a ValidatedURDF from validate_urdf")
        if not isinstance(config, EditorConfig):
            raise TypeError("config must be EditorConfig")
        joints = tuple(Joint(
            name=j.name, parent_link=j.parent, child_link=j.child, joint_type=j.joint_type,
            origin=rpy_transform(j.xyz, j.rpy), axis=j.axis, limit=j.limit,
        ) for j in source.joints)
        return KinematicChain(source.robot_name, source.base_link, source.tip_link, joints,
                              source.source.source_path or source.source.name,
                              source_sha256=source.source.sha256)
