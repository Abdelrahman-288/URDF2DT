"""Locations and provenance shared by source and frozen desktop runtimes."""

import json
from pathlib import Path
import sys
from typing import Any


def asset_path(relative: str) -> Path:
    """Locate read-only bundled assets independently of the working directory."""
    root = Path(getattr(sys, "_MEIPASS")) / "resources" if getattr(sys, "frozen", False) else Path(__file__).resolve().parents[1]
    return root / relative


def build_provenance() -> dict[str, Any] | None:
    """Return embedded build metadata only for a frozen executable."""
    if not getattr(sys, "frozen", False):
        return None
    try:
        data = json.loads(asset_path("build-info.json").read_text(encoding="utf-8"))
        return {"git_commit": data["git_commit"], "working_tree_clean": data["working_tree_clean"]}
    except (OSError, ValueError, KeyError):
        return {"git_commit": None, "working_tree_clean": None}
