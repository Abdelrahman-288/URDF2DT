"""Headless Stage 4 entry point: python -m urdf2dt.parser robot.urdf [--json]."""

import argparse
from dataclasses import asdict
import json

from urdf2dt.parser.urdf_validator import validate_urdf


def main() -> int:
    """Print structural validation results; 0 means accepted, 1 rejected."""
    parser = argparse.ArgumentParser(description="Validate URDF input for URDF2DT's serial-chain subset.")
    parser.add_argument("path", help="Path to an expanded .urdf file")
    parser.add_argument("--json", action="store_true", help="Print machine-readable validation results")
    args = parser.parse_args()
    result = validate_urdf(args.path)
    document = result.document
    if args.json:
        print(json.dumps({
            "valid": result.valid,
            "errors": [asdict(issue) for issue in result.errors],
            "warnings": [asdict(issue) for issue in result.warnings],
            "robot": None if document is None else {
                "name": document.robot_name, "base_link": document.base_link,
                "tip_link": document.tip_link, "links": len(document.links),
                "joints": len(document.joints), "movable_joint_names": document.movable_joint_names,
                "source_sha256": document.source.sha256,
            },
        }, indent=2))
    elif document is not None:
        print(f"URDF accepted: {document.robot_name}; {len(document.links)} links, "
              f"{len(document.movable_joint_names)} movable joints; {document.base_link} -> {document.tip_link}.")
        for issue in result.warnings:
            print(f"Warning [{issue.code}]: {issue.message}")
    else:
        for issue in result.errors:
            print(f"URDF rejected [{issue.code}]: {issue.message}")
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
