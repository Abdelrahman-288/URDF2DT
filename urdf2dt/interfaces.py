"""Parser/solver contracts consumed by the headless application pipeline."""

from typing import Protocol

from urdf2dt.config import EditorConfig
from urdf2dt.dh.types import DHModel, KinematicChain
from urdf2dt.parser.urdf_validator import ValidatedURDF


class URDFParser(Protocol):
    """Convert a structurally validated serial URDF into an ordered chain."""

    def parse(self, source: ValidatedURDF, config: EditorConfig) -> KinematicChain:
        """Consume the validated snapshot without reopening a potentially changed file."""
        ...


class DHSolver(Protocol):
    """Compute Standard-DH output without editing the source chain."""

    def solve(self, chain: KinematicChain, config: EditorConfig) -> DHModel:
        """Return an immutable baseline with one row per movable joint."""
        ...
