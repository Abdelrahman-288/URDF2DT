"""Opt-in application logging; importing the library never configures the root."""
import logging
from pathlib import Path
import subprocess
from typing import Any


def configure_logging(level: str = "INFO") -> None:
    """Configure only the urdf2dt logger, without replacing application root handlers."""
    logger = logging.getLogger("urdf2dt")
    logger.setLevel(level)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s %(message)s"))
        logger.addHandler(handler)
    logger.propagate = False


def git_provenance() -> dict[str, Any]:
    """Return checkout commit and dirty status, or unknown values outside Git."""
    root = Path(__file__).resolve().parents[1]
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
        return {"git_commit": commit, "working_tree_clean": not bool(dirty)}
    except (OSError, subprocess.CalledProcessError):
        return {"git_commit": None, "working_tree_clean": None}
