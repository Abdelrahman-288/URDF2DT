"""Local parser/solver contracts; implementations are supplied in later stages."""

from pathlib import Path
from typing import Protocol

from urdf2dt.config import EditorConfig
from urdf2dt.dh.types import DHModel, KinematicChain


class URDFParser(Protocol):
    """Convert a structurally validated serial URDF into an ordered chain."""

    def parse(self, path: Path, config: EditorConfig) -> KinematicChain:
        """Return all joints, including fixed joints, in base-to-tip order."""
        ...


class DHSolver(Protocol):
    """Compute Standard-DH output without editing the source chain."""

    def solve(self, chain: KinematicChain, config: EditorConfig) -> DHModel:
        """Return an immutable baseline with one row per movable joint."""
        ...
