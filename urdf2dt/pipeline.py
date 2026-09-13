"""Stage 5 headless validate -> parse -> automatic Standard-DH integration."""

from dataclasses import dataclass
from pathlib import Path

from urdf2dt.config import EditorConfig, load_config
from urdf2dt.dh.dh_solver import StandardDHSolver
from urdf2dt.dh.types import DHModel, JointType, KinematicChain
from urdf2dt.interfaces import DHSolver, URDFParser
from urdf2dt.parser.urdf_input import InputPolicy, URDFInput, URDFIssue
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import ValidatedURDF, validate_urdf


@dataclass(frozen=True, slots=True)
class AutomaticDHResult:
    """Frozen automatic baseline plus exact source/configuration for an editor."""

    source: ValidatedURDF
    chain: KinematicChain
    automatic_model: DHModel
    config: EditorConfig
    warnings: tuple[URDFIssue, ...] = ()


def generate_automatic_model(source: str | Path | URDFInput, config: EditorConfig | None = None,
                             *, parser: URDFParser | None = None, solver: DHSolver | None = None,
                             policy: InputPolicy = InputPolicy()) -> AutomaticDHResult:
    """Generate a baseline; failed validation never reaches parser/solver adapters.

    Adapters must preserve source, robot and ordered movable-joint identity. This
    boundary does not claim full FK validation, and never falls back to UR5 data.
    """
    effective = config if config is not None else load_config()
    result = validate_urdf(source, policy)
    document = result.require_valid()
    chain = (parser if parser is not None else SerialURDFParser()).parse(document, effective)
    if (chain.joint_names != document.movable_joint_names or chain.robot_name != document.robot_name
            or chain.base_link != document.base_link or chain.tip_link != document.tip_link
            or tuple((j.name, j.parent_link, j.child_link, j.joint_type) for j in chain.joints)
            != tuple((j.name, j.parent, j.child, j.joint_type) for j in document.joints)
            or chain.source_sha256 != document.source.sha256):
        raise ValueError("parser adapter changed source, chain endpoints or joint order")
    model = (solver if solver is not None else StandardDHSolver()).solve(chain, effective)
    movable = tuple(j for j in chain.joints if j.joint_type != JointType.FIXED)
    if (model.joint_names != chain.joint_names or model.robot_name != chain.robot_name
            or tuple(r.joint_type for r in model.rows) != tuple(j.joint_type for j in movable)
            or model.source_sha256 != chain.source_sha256 or model.source_urdf != chain.source_urdf):
        raise ValueError("solver adapter changed source, joint identities or joint order")
    return AutomaticDHResult(document, chain, model, effective, result.warnings)
